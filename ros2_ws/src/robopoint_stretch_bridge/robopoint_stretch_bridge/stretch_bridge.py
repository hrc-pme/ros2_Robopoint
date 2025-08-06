#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
import cv2

from robopoint_interfaces.msg import SpatialAffordanceResponse, AffordancePoint

class RoboPointStretchBridge(Node):
    def __init__(self):
        super().__init__('robopoint_stretch_bridge')

        # 參數：深度影像主題、相機參數
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_frame', 'camera_frame')
        self.declare_parameter('fx', 525.0)
        self.declare_parameter('fy', 525.0)
        self.declare_parameter('cx', 319.5)
        self.declare_parameter('cy', 239.5)

        self.bridge = CvBridge()
        self.depth_image = None

        # 訂閱 affordance 結果與深度影像
        self.create_subscription(
            SpatialAffordanceResponse,
            'spatial_affordance_response',
            self.affordance_callback,
            10
        )
        self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self.depth_callback,
            10
        )

        # 發布給 Stretch visual servo 的 3D 目標點
        self.goal_pub = self.create_publisher(PointStamped, 'stretch_goal_point', 10)

    def depth_callback(self, msg: Image):
        try:
            # 假設深度單位為毫米或公尺視硬體而定
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth decode error: {e}')

    def affordance_callback(self, msg: SpatialAffordanceResponse):
        if not msg.affordance_points or self.depth_image is None:
            return

        # 選出最高信心（F1）點
        best_pt: AffordancePoint = max(
            msg.affordance_points,
            key=lambda p: p.confidence
        )

        h, w = self.depth_image.shape[:2]
        u = int(best_pt.x * w) if best_pt.x <= 1.0 else int(best_pt.x)
        v = int(best_pt.y * h) if best_pt.y <= 1.0 else int(best_pt.y)

        # 取深度並轉換單位（假設深度 image 為公尺）
        depth = float(self.depth_image[v, u])

        fx = self.get_parameter('fx').value
        fy = self.get_parameter('fy').value
        cx = self.get_parameter('cx').value
        cy = self.get_parameter('cy').value

        # 影像座標 → 相機座標
        point = PointStamped()
        point.header = msg.header
        point.header.frame_id = self.get_parameter('camera_frame').value
        point.point.x = (u - cx) * depth / fx
        point.point.y = (v - cy) * depth / fy
        point.point.z = depth

        self.goal_pub.publish(point)
        self.get_logger().info(
            f'Publish Stretch goal: ({point.point.x:.3f}, {point.point.y:.3f}, {point.point.z:.3f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RoboPointStretchBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
