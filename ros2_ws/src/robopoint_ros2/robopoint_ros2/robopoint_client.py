#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from robopoint_interfaces.msg import SpatialAffordanceQuery, SpatialAffordanceResponse
from robopoint_interfaces.srv import ProcessImage
from std_msgs.msg import Header


class RoboPointClient(Node):
    def __init__(self):
        super().__init__('robopoint_client')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.query_pub = self.create_publisher(
            SpatialAffordanceQuery,
            'spatial_affordance_query',
            10
        )
        
        # Subscribers
        self.response_sub = self.create_subscription(
            SpatialAffordanceResponse,
            'spatial_affordance_response',
            self.handle_response,
            10
        )
        
        self.vis_sub = self.create_subscription(
            Image,
            'affordance_visualization',
            self.handle_visualization,
            10
        )
        
        # Service client
        self.process_client = self.create_client(ProcessImage, 'process_image_affordance')
        
        self.get_logger().info('RoboPoint client initialized')

    def send_query(self, query_text, image_process_mode="Pad"):
        """Send affordance query using topic"""
        msg = SpatialAffordanceQuery()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.query_text = query_text
        msg.image_process_mode = image_process_mode
        
        self.query_pub.publish(msg)
        self.get_logger().info(f'Sent query: {query_text}')

    def process_image_with_query(self, image_path, query_text, image_process_mode="Pad"):
        """Process image with affordance query using service"""
        # Load image
        cv_image = cv2.imread(image_path)
        if cv_image is None:
            self.get_logger().error(f"Could not load image: {image_path}")
            return None
        
        # Convert to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        
        # Wait for service
        while not self.process_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        # Create request
        request = ProcessImage.Request()
        request.image = image_msg
        request.query_text = query_text
        request.image_process_mode = image_process_mode
        
        # Call service
        future = self.process_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('Service call successful')
                self.handle_response(response.affordance_response)
                
                # Display visualization if available
                if response.visualization.data:
                    vis_image = self.bridge.imgmsg_to_cv2(response.visualization, "bgr8")
                    cv2.imshow('Affordance Visualization', vis_image)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                
                return response.affordance_response
            else:
                self.get_logger().error(f'Service call failed: {response.error_message}')
        else:
            self.get_logger().error('Service call failed')
        
        return None

    def handle_response(self, msg):
        """Handle affordance response"""
        self.get_logger().info(f'Received response for query: {msg.query_text}')
        self.get_logger().info(f'Model response: {msg.model_response}')
        self.get_logger().info(f'Found {len(msg.affordance_points)} affordance points:')
        
        for i, point in enumerate(msg.affordance_points):
            self.get_logger().info(f'  Point {i+1}: ({point.x:.3f}, {point.y:.3f}) confidence: {point.confidence:.3f}')

    def handle_visualization(self, msg):
        """Handle visualization image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow('Affordance Visualization', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error displaying visualization: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    client = RoboPointClient()
    
    try:
        # Example usage
        import time
        time.sleep(2.0)  # Wait for connections
        
        # Method 1: Using topics (requires camera feed)
        client.send_query("Find a few spots within the vacant area on the rightmost white plate.")
        
        # Method 2: Using service with specific image
        # client.process_image_with_query(
        #     "/path/to/your/image.jpg",
        #     "Identify several places in the unoccupied space on the stair in the middle."
        # )
        
        rclpy.spin(client)
        
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()