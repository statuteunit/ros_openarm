#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math

class TestPosePublisher(Node):
    def __init__(self):
        super().__init__("test_pose_publisher")
        
        # 替代 camera_vision，发布相同的话题名
        self.pub = self.create_publisher(PoseStamped, "/red_object_pose", 10)
        
        # 1Hz 的频率发布，频率不用太高
        self.timer = self.create_timer(1.0, self.timer_cb)
        
        # 设定固定的测试坐标 (对应 v1/scene.xml 中香蕉的位置)
        # XML 配置: <body name="banana" pos="0.35 0 0.45" ...>
        # 注意：Z=0.45 是物体中心，桌面高度约为 0.4
        self.target_x = 0.45
        self.target_y = 0.0
        self.target_z = 0.43 # 稍微比 0.45 低一点点或者保持一致，视抓取策略而定

        self.get_logger().info(f"正在发布固定测试坐标: [{self.target_x}, {self.target_y}, {self.target_z}]")

    def timer_cb(self):
        msg = PoseStamped()
        
        # 必须是 world 坐标系，因为 pick_task.py 是基于 world 规划的
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.position.x = self.target_x
        msg.pose.position.y = self.target_y
        msg.pose.position.z = self.target_z
        
        # 设定一个默认姿态 (水平放置)
        q = quaternion_from_euler(0, 0, 0)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
