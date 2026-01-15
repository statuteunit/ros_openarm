import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from tf_transformations import quaternion_from_euler

class BananaVisionNode(Node):
    def __init__(self):
        super().__init__('banana_vision_node')
        
        self.bridge = CvBridge()
        
        # 订阅图像
        self.image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 10)
            
        # 发布识别到的香蕉位置
        self.pose_pub = self.create_publisher(PoseStamped, "/banana_pose", 10)
        
        # TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 相机参数 (对应 XML: fovy=58, 640x480)
        self.img_w = 640
        self.img_h = 480
        f_y = self.img_h / (2 * np.tan(np.radians(58)/2)) # approx 428
        f_x = f_y
        self.camera_matrix = np.array([
            [f_x, 0, self.img_w/2],
            [0, f_y, self.img_h/2],
            [0, 0, 1]
        ])
        
        # 桌面高度假设
        self.KNOWN_Z_Height = 0.31

        # 关键修改：URDF 中的相机 Link 名称
        # 确保这个名字与 openarm_joint_state_subscriber.py 发布的一致
        self.camera_frame_id = "overhead_camera_link"

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # 1. 颜色识别 (HSV)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([15, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # 2. 寻找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        target_center = None
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    target_center = (cx, cy)
                    cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 2)

        cv2.imshow("Camera Vision", cv_image)
        cv2.waitKey(1)

        # 3. 计算 3D 坐标
        if target_center:
            self.calculate_and_publish_pose(target_center)

    def calculate_and_publish_pose(self, uv):
        u, v = uv
        
        # --- 步骤 A: 像素 -> 相机坐标系下的射线向量 ---
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        
        ray_x = (u - cx) / fx
        ray_y = (v - cy) / fy
        ray_z = 1.0 
        
        # --- 步骤 B: 射线变换到世界坐标系 ---
        try:
            # 获取 World -> Camera 的变换
            # 关键修改：使用 overhead_camera_link
            trans = self.tf_buffer.lookup_transform('world', self.camera_frame_id, rclpy.time.Time())
        except Exception as e:
            return

        p_cam = PointStamped()
        # 关键修改：源 Frame 必须是相机 Link
        p_cam.header.frame_id = self.camera_frame_id
        p_cam.point.x = ray_x
        p_cam.point.y = ray_y
        p_cam.point.z = ray_z
        
        try:
            p_world_ray = self.tf_buffer.transform(p_cam, "world")
        except:
            return

        cam_origin = np.array([trans.transform.translation.x, 
                               trans.transform.translation.y, 
                               trans.transform.translation.z])
        
        ray_dir_world = np.array([p_world_ray.point.x, p_world_ray.point.y, p_world_ray.point.z]) - cam_origin
        
        # --- 步骤 C: 射线与平面相交 ---
        if abs(ray_dir_world[2]) < 1e-6: return
        t = (self.KNOWN_Z_Height - cam_origin[2]) / ray_dir_world[2]
        if t < 0: return
        intersection = cam_origin + t * ray_dir_world
        
        # --- 步骤 D: 发布 Pose ---
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = intersection[0]
        pose_msg.pose.position.y = intersection[1]
        pose_msg.pose.position.z = self.KNOWN_Z_Height
        q = quaternion_from_euler(0, 0, 0)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BananaVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
