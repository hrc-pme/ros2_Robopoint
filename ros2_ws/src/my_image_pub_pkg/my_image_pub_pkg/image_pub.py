# image_pub.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0, self.timer_callback)  # 每秒一次
        self.get_logger().info("Image Publisher Node has been started.")

    def timer_callback(self):
        img = cv2.imread('/workspace/pub_image.jpg')  # ✅←←← 替換為你的圖片完整路徑
        if img is None:
            self.get_logger().warn("Image not found!")
            return

        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info("Published image.")

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
