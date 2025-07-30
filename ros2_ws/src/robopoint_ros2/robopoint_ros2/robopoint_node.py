#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped
from robopoint_interfaces.msg import SpatialAffordanceQuery, SpatialAffordanceResponse, AffordancePoint
from robopoint_interfaces.srv import ProcessImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as PILImage, ImageDraw
import requests
import json
import hashlib
import datetime
import os
import re
import time
from robopoint_controller.conversation import (default_conversation, conv_templates, SeparatorStyle)
from robopoint_controller.constants import LOGDIR
from robopoint_controller.utils import (
    build_logger, server_error_msg, violates_moderation, moderation_msg
)


class RoboPointNode(Node):
    def __init__(self):
        super().__init__('robopoint_node')
        
        # Initialize logger
        self.logger = build_logger("robopoint_ros2", "robopoint_ros2.log")
        
        # Parameters
        self.declare_parameter('controller_url', 'http://localhost:21001')
        self.declare_parameter('moderate', False)
        self.declare_parameter('temperature', 1.0)
        self.declare_parameter('top_p', 0.7)
        self.declare_parameter('max_output_tokens', 512)
        self.declare_parameter('model_name', '')
        
        self.controller_url = self.get_parameter('controller_url').value
        self.moderate = self.get_parameter('moderate').value
        self.temperature = self.get_parameter('temperature').value
        self.top_p = self.get_parameter('top_p').value
        self.max_output_tokens = self.get_parameter('max_output_tokens').value
        self.model_name = self.get_parameter('model_name').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize conversation state
        self.conversation_state = default_conversation.copy()
        
        # Get available models
        self.models = self.get_model_list()
        if not self.model_name and self.models:
            self.model_name = self.models[0]
        
        # Priority for model selection
        self.priority = {
            "vicuna-13b": "aaaaaaa",
            "koala-13b": "aaaaaab",
        }
        
        # Headers for HTTP requests
        self.headers = {"User-Agent": "RoboPoint Client"}
        
        # Publishers
        self.affordance_pub = self.create_publisher(
            SpatialAffordanceResponse, 
            'spatial_affordance_response', 
            10
        )
        
        self.visualization_pub = self.create_publisher(
            Image, 
            'affordance_visualization', 
            10
        )
        
        # Subscribers
        self.query_sub = self.create_subscription(
            SpatialAffordanceQuery,
            'spatial_affordance_query',
            self.handle_affordance_query,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.handle_image_callback,
            10
        )
        
        # Services
        self.process_service = self.create_service(
            ProcessImage,
            'process_image_affordance',
            self.process_image_service
        )
        
        # Store latest image
        self.latest_image = None
        self.latest_pil_image = None
        
        self.get_logger().info(f'RoboPoint node initialized with models: {self.models}')

    def get_model_list(self):
        """Get available models from controller"""
        try:
            ret = requests.post(self.controller_url + "/refresh_all_workers")
            if ret.status_code != 200:
                self.get_logger().warn("Failed to refresh workers")
                return []
            
            ret = requests.post(self.controller_url + "/list_models")
            if ret.status_code != 200:
                self.get_logger().warn("Failed to get model list")
                return []
                
            models = ret.json()["models"]
            models.sort(key=lambda x: self.priority.get(x, x))
            self.logger.info(f"Models: {models}")
            return models
        except Exception as e:
            self.get_logger().error(f"Error getting model list: {e}")
            return []

    def handle_image_callback(self, msg):
        """Handle incoming image messages"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            
            # Convert to PIL for processing
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            self.latest_pil_image = PILImage.fromarray(rgb_image)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def handle_affordance_query(self, msg):
        """Handle spatial affordance query messages"""
        if self.latest_pil_image is None:
            self.get_logger().warn("No image available for processing")
            return
        
        try:
            # Process the query
            response = self.process_affordance_request(
                self.latest_pil_image, 
                msg.query_text,
                msg.image_process_mode
            )
            
            # Publish response
            if response:
                self.affordance_pub.publish(response)
                
                # Create and publish visualization
                vis_image = self.create_visualization(
                    self.latest_pil_image, 
                    response.affordance_points
                )
                if vis_image is not None:
                    vis_msg = self.bridge.cv2_to_imgmsg(vis_image, "bgr8")
                    vis_msg.header = msg.header
                    self.visualization_pub.publish(vis_msg)
                    
        except Exception as e:
            self.get_logger().error(f"Error handling affordance query: {e}")

    def process_image_service(self, request, response):
        """Service to process image with affordance query"""
        try:
            # Convert ROS Image to PIL
            cv_image = self.bridge.imgmsg_to_cv2(request.image, "bgr8")
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            
            # Process the query
            affordance_response = self.process_affordance_request(
                pil_image,
                request.query_text,
                request.image_process_mode
            )
            
            if affordance_response:
                response.success = True
                response.affordance_response = affordance_response
                
                # Create visualization
                vis_image = self.create_visualization(
                    pil_image, 
                    affordance_response.affordance_points
                )
                if vis_image is not None:
                    response.visualization = self.bridge.cv2_to_imgmsg(vis_image, "bgr8")
            else:
                response.success = False
                response.error_message = "Failed to process affordance query"
                
        except Exception as e:
            response.success = False
            response.error_message = str(e)
            self.get_logger().error(f"Service error: {e}")
            
        return response

    def process_affordance_request(self, pil_image, query_text, image_process_mode):
        """Process affordance request and return response"""
        try:
            # Check moderation if enabled
            if self.moderate and violates_moderation(query_text):
                self.get_logger().warn("Query flagged by moderation")
                return None
            
            # Prepare query text with formatting instructions
            formatted_query = query_text + " Your answer should be formatted as a list of tuples, " \
                            "i.e. [(x1, y1), (x2, y2), ...], where each tuple contains the " \
                            "x and y coordinates of a point satisfying the conditions above." \
                            " The coordinates should be between 0 and 1, indicating the " \
                            "normalized pixel locations of the points in the image."
            
            # Truncate text
            formatted_query = formatted_query[:1536]
            if pil_image is not None:
                formatted_query = formatted_query[:1200]
            
            # Setup conversation
            state = default_conversation.copy()
            if pil_image is not None:
                text_input = ('<image>\n' + formatted_query, pil_image, image_process_mode)
            else:
                text_input = formatted_query
                
            state.append_message(state.roles[0], text_input)
            state.append_message(state.roles[1], None)
            
            # Get model response
            model_response = self.query_model(state)
            
            if model_response:
                # Parse response for coordinate points
                points = self.find_vectors(model_response)
                
                # Create response message
                response_msg = SpatialAffordanceResponse()
                response_msg.header.stamp = self.get_clock().now().to_msg()
                response_msg.query_text = query_text
                response_msg.model_response = model_response
                response_msg.success = True
                
                # Convert points to AffordancePoint messages
                for point in points:
                    if len(point) == 2:  # 2D points
                        affordance_point = AffordancePoint()
                        affordance_point.x = float(point[0])
                        affordance_point.y = float(point[1])
                        affordance_point.confidence = 1.0  # Could be extended
                        response_msg.affordance_points.append(affordance_point)
                
                return response_msg
            
        except Exception as e:
            self.get_logger().error(f"Error processing affordance request: {e}")
            
        return None

    def query_model(self, state):
        """Query the vision-language model"""
        try:
            # Select template based on model name
            if 'vicuna' in self.model_name.lower():
                template_name = "vicuna_v1"
            elif "llama" in self.model_name.lower():
                template_name = "llava_llama_2"
            elif "mistral" in self.model_name.lower():
                template_name = "mistral_instruct"
            elif "mpt" in self.model_name.lower():
                template_name = "mpt"
            else:
                template_name = "llava_v1"

            new_state = conv_templates[template_name].copy()
            new_state.append_message(new_state.roles[0], state.messages[-2][1])
            new_state.append_message(new_state.roles[1], None)
            state = new_state
            
            # Query worker address
            ret = requests.post(self.controller_url + "/get_worker_address",
                              json={"model": self.model_name})
            
            if ret.status_code != 200:
                self.get_logger().error("Failed to get worker address")
                return None
                
            worker_addr = ret.json()["address"]
            if worker_addr == "":
                self.get_logger().error("No available worker")
                return None
            
            # Prepare request
            prompt = state.get_prompt()
            pil_images, images, transforms = state.get_images()
            
            # Generate image hashes for logging
            image_hash = []
            if pil_images:
                image_hash = [hashlib.md5(image.tobytes()).hexdigest() for image in pil_images]
                # Save images for logging
                for image, hash_val in zip(pil_images, image_hash):
                    t = datetime.datetime.now()
                    filename = os.path.join(LOGDIR, "serve_images", 
                                          f"{t.year}-{t.month:02d}-{t.day:02d}", 
                                          f"{hash_val}.jpg")
                    if not os.path.isfile(filename):
                        os.makedirs(os.path.dirname(filename), exist_ok=True)
                        image.save(filename)
            
            # Prepare payload
            pload = {
                "model": self.model_name,
                "prompt": prompt,
                "temperature": self.temperature,
                "top_p": self.top_p,
                "max_new_tokens": min(int(self.max_output_tokens), 1536),
                "stop": state.sep if state.sep_style in [SeparatorStyle.SINGLE, SeparatorStyle.MPT] else state.sep2,
                "images": images,
            }
            
            # Make request
            response = requests.post(worker_addr + "/worker_generate_stream",
                                   headers=self.headers, json=pload, stream=True, timeout=30)
            
            output = ""
            for chunk in response.iter_lines(decode_unicode=False, delimiter=b"\0"):
                if chunk:
                    data = json.loads(chunk.decode())
                    if data["error_code"] == 0:
                        output = data["text"][len(prompt):].strip()
                    else:
                        self.get_logger().error(f"Model error: {data['text']}")
                        return None
            
            return output
            
        except Exception as e:
            self.get_logger().error(f"Error querying model: {e}")
            return None

    def find_vectors(self, text):
        """Extract coordinate vectors from model response"""
        # Pattern to match tuples with numeric values
        pattern = r"\(([-+]?\d+\.?\d*(?:,\s*[-+]?\d+\.?\d*)*?)\)"
        matches = re.findall(pattern, text)
        
        vectors = []
        for match in matches:
            try:
                # Split and convert to numbers
                vector = [float(num.strip()) if '.' in num else int(num.strip()) 
                         for num in match.split(',')]
                vectors.append(vector)
            except ValueError:
                continue
                
        return vectors

    def create_visualization(self, pil_image, affordance_points, cross_size=9, cross_width=4):
        """Create visualization of affordance points on image"""
        try:
            if not affordance_points:
                return None
                
            # Convert PIL to OpenCV
            cv_image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
            h, w = cv_image.shape[:2]
            
            # Draw points
            for point in affordance_points:
                # Convert normalized coordinates to pixel coordinates
                x = int(point.x * w) if point.x <= 1.0 else int(point.x)
                y = int(point.y * h) if point.y <= 1.0 else int(point.y)
                
                # Draw cross
                cv2.line(cv_image, 
                        (x - cross_size, y - cross_size), 
                        (x + cross_size, y + cross_size), 
                        (0, 0, 255), cross_width)
                cv2.line(cv_image, 
                        (x - cross_size, y + cross_size), 
                        (x + cross_size, y - cross_size), 
                        (0, 0, 255), cross_width)
            
            return cv_image
            
        except Exception as e:
            self.get_logger().error(f"Error creating visualization: {e}")
            return None

    def get_conv_log_filename(self):
        """Get conversation log filename"""
        t = datetime.datetime.now()
        name = os.path.join(LOGDIR, f"{t.year}-{t.month:02d}-{t.day:02d}-conv.json")
        return name


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RoboPointNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()