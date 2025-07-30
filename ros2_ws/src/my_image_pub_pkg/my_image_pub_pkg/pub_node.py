# pub_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(String, 'custom_topic', 10)
        timer_period = 1.0  # 每秒發布一次
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Publisher started.")

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        timestamp_str = f"{now.sec}.{now.nanosec:09d}"

        msg = String()
        msg.data = f"Received Message\nTimestamp: {timestamp_str}"
        self.publisher_.publish(msg)

        # 分兩次印出，每一行都帶有 ROS log 格式
        self.get_logger().info("Published Message")
        self.get_logger().info(f"Timestamp: {timestamp_str}")

def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
