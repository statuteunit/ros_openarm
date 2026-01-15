import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import mujoco
import numpy as np
import glfw
from cv_bridge import CvBridge
import time
import os
from ament_index_python.packages import get_package_share_directory

class OpenArmMujocoSync(Node):
    def __init__(self):
        super().__init__("openarm_mujoco_sync")
        
        # ========== 1. 初始化数据 ==========
        self.joint_data = {}
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 鼠标交互状态
        self.button_left = self.button_middle = self.button_right = False
        self.last_x = self.last_y = 0

        # ========== 2. 智能加载模型路径 (修复 FileNotFoundError) ==========
        final_model_path = None
        
        # [策略 1]: 尝试从 ROS 安装目录 (share) 加载
        # 这要求 openarm_mujoco 在 CMakeLists.txt 中 install 了 v1 文件夹
        try:
            share_dir = get_package_share_directory('openarm_mujoco')
            path_candidate = os.path.join(share_dir, 'v1', 'scene.xml')
            if os.path.exists(path_candidate):
                final_model_path = path_candidate
                self.get_logger().info(f"Using Installed Model: {final_model_path}")
        except Exception:
            pass # 如果包没找到或者路径不对，忽略，尝试下一个策略

        # [策略 2]: 尝试回溯到源码目录 (开发环境 Fallback)
        # 如果策略 1 失败，我们尝试从当前脚本位置向上找工作空间根目录
        if final_model_path is None:
            current_file_path = os.path.abspath(__file__)
            # 寻找 'install' 目录的父级，即工作空间根目录
            # 路径通常是: .../workspace/install/pkg/lib/...
            workspace_root = None
            temp_path = current_file_path
            
            # 向上搜索最多 10 层
            for _ in range(10):
                parent = os.path.dirname(temp_path)
                if os.path.basename(parent) == 'install':
                    workspace_root = os.path.dirname(parent) # 找到了 workspace root
                    break
                if parent == temp_path: # 到达系统根目录
                    break
                temp_path = parent
            
            if workspace_root:
                # 尝试构建源码路径: workspace/src/openarm_mujoco/v1/scene.xml
                # 注意：这里兼容 openarm_mujoco 和 openarm_mujuco (拼写容错)
                possible_paths = [
                    os.path.join(workspace_root, 'src', 'openarm_mujoco', 'v1', 'scene.xml'),
                    os.path.join(workspace_root, 'src', 'openarm_mujuco', 'v1', 'scene.xml')
                ]
                
                for p in possible_paths:
                    if os.path.exists(p):
                        final_model_path = p
                        self.get_logger().warn(f"Using Source Model (Dev Mode): {final_model_path}")
                        break

        # [最终检查]
        if final_model_path is None or not os.path.exists(final_model_path):
            self.get_logger().error("Critical Error: 无法找到 scene.xml")
            self.get_logger().error("请确认: 1. openarm_mujoco 包在 src 目录下; 或者 2. 已通过 CMake 安装了 v1 文件夹")
            raise FileNotFoundError(f"Scene file not found. Search failed.")

        self.model = mujoco.MjModel.from_xml_path(final_model_path)
        self.data = mujoco.MjData(self.model)
        
        # --- 获取 Body ID ---
        self.cam_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "d435")
        if self.cam_body_id == -1:
            self.get_logger().warn("未找到名为 d435 的 body，请检查 scene.xml")
        
        self.tcp_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "openarm_left_hand_tcp")
        
        # 夹爪与关节配置
        self.arm_ee_name = "openarm_left_hand_tcp"
        self.finger_ee_name = "openarm_left_right_finger"
        self.grip_act_name = 'left_finger1_ctrl'
        self.grip_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, self.grip_act_name)
        
        self.arm_act_names = [f'left_joint{i}_ctrl' for i in range(1, 8)]
        
        # 关节映射 (保持 openarm_ 前缀)
        self.arm_joint_names = [f"openarm_left_joint{i}" for i in range(1, 8)] + \
                               [f"openarm_right_joint{i}" for i in range(1, 8)] + \
                               ["openarm_left_finger_joint1", "openarm_right_finger_joint1"]
        
        self.joint_qpos_map = {n: self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)] 
                               for n in self.arm_joint_names if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n) != -1}

        # 香蕉 ID
        self.banana_name = "banana"
        self.banana_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.banana_name)
        
        # 抓取逻辑参数
        banana_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "banana_joint")
        self.banana_qpos_addr = self.model.jnt_qposadr[banana_joint_id]
        self.banana_dof_addr = self.model.jnt_dofadr[banana_joint_id]
        self.is_grasped = False
        self.GRASP_THRESHOLD = 0.04
        self.finger_joint_name = "openarm_left_finger_joint1"
        self.relative_pos_offset = np.zeros(3)
        self.relative_quat_offset = np.array([1.0, 0.0, 0.0, 0.0])

        # ========== 3. GLFW 初始化 ==========
        if not glfw.init(): raise Exception("GLFW 无法初始化")
        self.last_render_time = time.time()

        # 主窗口
        self.win_main = glfw.create_window(1200, 900, 'OpenArm - Interaction', None, None)
        glfw.make_context_current(self.win_main)
        self.ctx_main = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150.value)
        self.scn_main = mujoco.MjvScene(self.model, maxgeom=10000)
        self.cam_main = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.cam_main)
        self.cam_main.lookat = np.array([0.35, 0.0, 0.32]) 
        self.cam_main.distance = 2.5 
        self.cam_main.azimuth = 180
        self.cam_main.elevation = -25.0 
    
        glfw.set_cursor_pos_callback(self.win_main, self.mouse_move)
        glfw.set_mouse_button_callback(self.win_main, self.mouse_button)
        glfw.set_scroll_callback(self.win_main, self.scroll)

        # 离屏渲染窗口 (用于发布图像)
        self.cam_pub_width, self.cam_pub_height = 640, 480
        glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
        self.win_offscreen = glfw.create_window(self.cam_pub_width, self.cam_pub_height, "Offscreen", None, None)
        glfw.make_context_current(self.win_offscreen)
        self.ctx_off = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150.value)
        self.scn_off = mujoco.MjvScene(self.model, maxgeom=10000)
        self.cam_fixed = mujoco.MjvCamera()
        
        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "d435")
        self.cam_fixed.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self.cam_fixed.fixedcamid = cam_id if cam_id != -1 else 0
        self.vopt = mujoco.MjvOption()

        # ========== 4. ROS 通信 ==========
        self.image_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.banana_pose_pub = self.create_publisher(PoseStamped, "/banana_pose", 10)
        self.subscription = self.create_subscription(JointState, "/joint_states", self.joint_cb, 10)
        self.banana_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "banana")
        self.log_tick = 0

    def joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.joint_qpos_map:
                self.joint_data[name] = msg.position[i]

    def get_tcp_pose(self):
        pos = self.data.xpos[self.tcp_body_id]
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, self.data.xmat[self.tcp_body_id])
        return pos, quat

    def publish_banana_tf(self):
        # 发布 Banana TF
        if self.banana_id != -1:
            pos = self.data.xpos[self.banana_id]
            quat = self.data.xquat[self.banana_id]
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = 'banana_visual_only'
            t.transform.translation.x = pos[0]
            t.transform.translation.y = pos[1]
            t.transform.translation.z = pos[2]
            t.transform.rotation.x = quat[1]
            t.transform.rotation.y = quat[2]
            t.transform.rotation.z = quat[3]
            t.transform.rotation.w = quat[0]
            self.tf_broadcaster.sendTransform(t)

        # 发布 Camera TF (匹配 URDF 中的 overhead_camera_link)
        if self.cam_body_id != -1:
            c_pos = self.data.xpos[self.cam_body_id]
            c_quat = self.data.xquat[self.cam_body_id]
            
            t_cam = TransformStamped()
            t_cam.header.stamp = self.get_clock().now().to_msg()
            t_cam.header.frame_id = 'world'
            t_cam.child_frame_id = 'overhead_camera_link' 
            t_cam.transform.translation.x = c_pos[0]
            t_cam.transform.translation.y = c_pos[1]
            t_cam.transform.translation.z = c_pos[2]
            t_cam.transform.rotation.x = c_quat[1]
            t_cam.transform.rotation.y = c_quat[2]
            t_cam.transform.rotation.z = c_quat[3]
            t_cam.transform.rotation.w = c_quat[0]
            self.tf_broadcaster.sendTransform(t_cam)

    def run_once(self):
        now = time.time()
        elapsed = now - self.last_render_time
        self.last_render_time = now
        dt = self.model.opt.timestep
        steps = min(int(elapsed / dt), 100)

        for _ in range(max(steps, 1)):
            for name, pos in self.joint_data.items():
                if name in self.joint_qpos_map:
                    addr = self.joint_qpos_map[name]
                    self.data.qpos[addr] = pos
                    joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                    dof_addr = self.model.jnt_dofadr[joint_id]
                    self.data.qvel[dof_addr] = 0 
                    if "finger_joint1" in name:
                        m_name = name.replace("joint1", "joint2")
                        mid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, m_name)
                        if mid != -1:
                            self.data.qpos[self.model.jnt_qposadr[mid]] = pos
                            self.data.qvel[self.model.jnt_dofadr[mid]] = 0

            mujoco.mj_kinematics(self.model, self.data)
            tcp_pos = self.data.xpos[self.tcp_body_id].copy()
            tcp_mat = self.data.xmat[self.tcp_body_id].copy()
            banana_pos = self.data.xpos[self.banana_body_id].copy()
            finger_val = self.joint_data.get(self.finger_joint_name, 0.0)

            if finger_val > 0.03: 
                if self.is_grasped:
                    self.is_grasped = False
                    self.get_logger().info(f"Released! Finger val: {finger_val:.4f}")
            elif not self.is_grasped:
                distance = np.linalg.norm(tcp_pos - banana_pos)
                if finger_val < 0.01 and distance < self.GRASP_THRESHOLD:
                    self.is_grasped = True
                    self.relative_pos_offset = banana_pos - tcp_pos
                    tcp_quat = np.empty(4)
                    banana_quat = np.empty(4)
                    mujoco.mju_mat2Quat(tcp_quat, tcp_mat)
                    mujoco.mju_mat2Quat(banana_quat, self.data.xmat[self.banana_body_id])
                    tcp_quat_inv = np.empty(4)
                    mujoco.mju_negQuat(tcp_quat_inv, tcp_quat)
                    mujoco.mju_mulQuat(self.relative_quat_offset, tcp_quat_inv, banana_quat)
                    self.get_logger().info("Grasped!")

            if self.is_grasped:
                self.data.qpos[self.banana_qpos_addr : self.banana_qpos_addr + 3] = tcp_pos + self.relative_pos_offset
                current_tcp_quat = np.empty(4)
                mujoco.mju_mat2Quat(current_tcp_quat, self.data.xmat[self.tcp_body_id])
                target_banana_quat = np.empty(4)
                mujoco.mju_mulQuat(target_banana_quat, current_tcp_quat, self.relative_quat_offset)
                self.data.qpos[self.banana_qpos_addr + 3 : self.banana_qpos_addr + 7] = target_banana_quat
                self.data.qvel[self.banana_dof_addr : self.banana_dof_addr + 6] = 0

            mujoco.mj_step(self.model, self.data)

        self.publish_banana_tf()
        
        # 渲染
        glfw.make_context_current(self.win_main)
        w, h = glfw.get_framebuffer_size(self.win_main)
        mujoco.mjv_updateScene(self.model, self.data, self.vopt, None, self.cam_main, mujoco.mjtCatBit.mjCAT_ALL.value, self.scn_main)
        mujoco.mjr_render(mujoco.MjrRect(0,0,w,h), self.scn_main, self.ctx_main)
        glfw.swap_buffers(self.win_main)

        # 发布图像
        glfw.make_context_current(self.win_offscreen)
        mujoco.mjv_updateScene(self.model, self.data, self.vopt, None, self.cam_fixed, mujoco.mjtCatBit.mjCAT_ALL.value, self.scn_off)
        mujoco.mjr_render(mujoco.MjrRect(0,0,self.cam_pub_width, self.cam_pub_height), self.scn_off, self.ctx_off)
        rgb = np.empty((self.cam_pub_height, self.cam_pub_width, 3), dtype=np.uint8)
        mujoco.mjr_readPixels(rgb, None, mujoco.MjrRect(0,0,self.cam_pub_width, self.cam_pub_height), self.ctx_off)
        img_msg = self.bridge.cv2_to_imgmsg(np.flipud(rgb), encoding="rgb8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        # 关键修改：Frame ID 对应 URDF 中的 Link
        img_msg.header.frame_id = "overhead_camera_link" 
        self.image_pub.publish(img_msg)

        glfw.poll_events()

    def mouse_button(self, window, button, act, mods):
        self.button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
        self.button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
        self.button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)
        self.last_x, self.last_y = glfw.get_cursor_pos(window)

    def mouse_move(self, window, xpos, ypos):
        dx, dy = xpos - self.last_x, ypos - self.last_y
        self.last_x, self.last_y = xpos, ypos
        if not (self.button_left or self.button_middle or self.button_right): return
        width, height = glfw.get_window_size(window)
        action = mujoco.mjtMouse.mjMOUSE_PAN_V if self.button_middle else \
                 (mujoco.mjtMouse.mjMOUSE_ROTATE_V if self.button_left else mujoco.mjtMouse.mjMOUSE_ZOOM)
        mujoco.mjv_moveCamera(self.model, action, dx/height, dy/height, self.scn_main, self.cam_main)

    def scroll(self, window, xoffset, yoffset):
        mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0, -0.05 * yoffset, self.scn_main, self.cam_main)

def main(args=None):
    rclpy.init(args=args)
    node = OpenArmMujocoSync()
    try:
        while rclpy.ok() and not glfw.window_should_close(node.win_main):
            rclpy.spin_once(node, timeout_sec=0.005)
            node.run_once()
    except KeyboardInterrupt: pass
    finally:
        glfw.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
