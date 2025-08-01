#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Point, PointStamped
from robopoint_interfaces.msg import SpatialAffordanceQuery, SpatialAffordanceResponse, AffordancePoint
from robopoint_interfaces.srv import (
    ProcessImage, 
    RefreshAllWorkers, 
    ListModels, 
    GetWorkerAddress, 
    WorkerGenerateStream
)
from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as PILImage, ImageDraw
import json
import hashlib
import datetime
import os
import re
import time
import base64
import io
import threading
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
        
        # Priority for model selection - 定義在使用之前
        self.priority = {
            "vicuna-13b": "aaaaaaa",
            "koala-13b": "aaaaaab",
        }
        
        # Parameters
        self.declare_parameter('moderate', False)
        self.declare_parameter('temperature', 1.0)
        self.declare_parameter('top_p', 0.7)
        self.declare_parameter('max_output_tokens', 512)
        self.declare_parameter('model_name', '')
        self.declare_parameter('service_timeout', 30.0)
        # Camera parameters
        self.declare_parameter('camera_device', '/dev/video4')
        self.declare_parameter('camera_fps', 30.0)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('enable_camera', True)
        
        self.moderate = self.get_parameter('moderate').value
        self.temperature = self.get_parameter('temperature').value
        self.top_p = self.get_parameter('top_p').value
        self.max_output_tokens = self.get_parameter('max_output_tokens').value
        self.model_name = self.get_parameter('model_name').value
        self.service_timeout = self.get_parameter('service_timeout').value
        # Camera parameters
        self.camera_device = self.get_parameter('camera_device').value
        self.camera_fps = self.get_parameter('camera_fps').value
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.enable_camera = self.get_parameter('enable_camera').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize conversation state
        self.conversation_state = default_conversation.copy()
        
        # Camera related variables
        self.camera = None
        self.camera_thread = None
        self.camera_running = False
        
        # ROS2 Service Clients for controller communication
        self.refresh_workers_client = self.create_client(
            RefreshAllWorkers, 
            '/refresh_all_workers'
        )
        self.list_models_client = self.create_client(
            ListModels, 
            '/list_models'
        )
        self.get_worker_client = self.create_client(
            GetWorkerAddress, 
            '/get_worker_address'
        )
        
        # Wait for controller services to be available
        self.wait_for_controller_services()
        
        # Get available models
        self.models = self.get_model_list()
        if not self.model_name and self.models:
            self.model_name = self.models[0]
        
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
        
        # Camera image publisher
        self.camera_pub = self.create_publisher(
            Image,
            'camera/image_raw',
            10
        )
        
        # Subscribers
        self.query_sub = self.create_subscription(
            SpatialAffordanceQuery,
            'spatial_affordance_query',
            self.handle_affordance_query,
            10
        )
        
        # Optional: still keep the external image subscription for flexibility
        self.image_sub = self.create_subscription(
            Image,
            'external_camera/image_raw',
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
        
        # Cache for worker service clients
        self.worker_clients = {}
        
        # Initialize camera if enabled
        if self.enable_camera:
            self.init_camera()
        
        self.get_logger().info(f'RoboPoint node initialized with models: {self.models}')
        if self.enable_camera:
            self.get_logger().info(f'Camera enabled on device: {self.camera_device}')

    def init_camera(self):
        """Initialize and start camera capture"""
        try:
            # Try to parse camera device (support both index and device path)
            if self.camera_device.startswith('/dev/video'):
                # Extract number from /dev/videoX
                device_num = int(self.camera_device.split('video')[1])
            else:
                device_num = int(self.camera_device)
            
            self.camera = cv2.VideoCapture(device_num)
            
            if not self.camera.isOpened():
                self.get_logger().error(f"Failed to open camera device: {self.camera_device}")
                return
            
            # Set camera properties
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
            self.camera.set(cv2.CAP_PROP_FPS, self.camera_fps)
            
            # Verify settings
            actual_width = self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
            
            self.get_logger().info(f"Camera initialized: {actual_width}x{actual_height} @ {actual_fps} FPS")
            
            # Start camera thread
            self.camera_running = True
            self.camera_thread = threading.Thread(target=self.camera_loop)
            self.camera_thread.daemon = True
            self.camera_thread.start()
            
        except Exception as e:
            self.get_logger().error(f"Error initializing camera: {e}")
            self.camera = None

    def camera_loop(self):
        """Camera capture loop running in separate thread"""
        if not self.camera:
            return
        
        # Calculate sleep time based on FPS
        sleep_time = 1.0 / self.camera_fps
        
        while self.camera_running and rclpy.ok():
            try:
                ret, frame = self.camera.read()
                if not ret:
                    self.get_logger().warn("Failed to read frame from camera")
                    time.sleep(sleep_time)
                    continue
                
                # Store latest image for affordance processing
                self.latest_image = frame.copy()
                
                # Convert to PIL for processing
                rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self.latest_pil_image = PILImage.fromarray(rgb_image)
                
                # Convert to ROS Image message and publish
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    ros_image.header.stamp = self.get_clock().now().to_msg()
                    ros_image.header.frame_id = "camera_frame"
                    self.camera_pub.publish(ros_image)
                except Exception as e:
                    self.get_logger().error(f"Error publishing camera image: {e}")
                
                time.sleep(sleep_time)
                
            except Exception as e:
                self.get_logger().error(f"Error in camera loop: {e}")
                time.sleep(sleep_time)

    def cleanup_camera(self):
        """Clean up camera resources"""
        self.camera_running = False
        if self.camera_thread and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=1.0)
        
        if self.camera:
            self.camera.release()
            self.camera = None
            self.get_logger().info("Camera resources cleaned up")

    def wait_for_controller_services(self):
        """Wait for controller services to become available"""
        services_to_wait = [
            (self.refresh_workers_client, 'refresh_all_workers'),
            (self.list_models_client, 'list_models'),
            (self.get_worker_client, 'get_worker_address')
        ]
        
        for client, service_name in services_to_wait:
            self.get_logger().info(f'Waiting for service: {service_name}')
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().warn(f'Service {service_name} not available after 10 seconds')
            else:
                self.get_logger().info(f'Service {service_name} is ready')

    def get_model_list(self):
        """Get available models from controller via ROS2 services"""
        try:
            # Refresh all workers first
            refresh_request = RefreshAllWorkers.Request()
            refresh_future = self.refresh_workers_client.call_async(refresh_request)
            
            rclpy.spin_until_future_complete(self, refresh_future, timeout_sec=5.0)
            
            if refresh_future.result() is None:
                self.get_logger().warn("Failed to refresh workers - service call timeout")
            elif not refresh_future.result().success:
                self.get_logger().warn(f"Failed to refresh workers: {refresh_future.result().message}")
            
            # Get model list
            list_request = ListModels.Request()
            list_future = self.list_models_client.call_async(list_request)
            
            rclpy.spin_until_future_complete(self, list_future, timeout_sec=5.0)
            
            if list_future.result() is None:
                self.get_logger().warn("Failed to get model list - service call timeout")
                return []
            
            result = list_future.result()
            if not result.success:
                self.get_logger().warn(f"Failed to get model list: {result.message}")
                return []
                
            models = result.model_names
            models.sort(key=lambda x: self.priority.get(x, x))
            self.logger.info(f"Models: {models}")
            return models
            
        except Exception as e:
            self.get_logger().error(f"Error getting model list: {e}")
            return []

    def handle_image_callback(self, msg):
        """Handle incoming external image messages (kept for flexibility)"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Only update if we're not using internal camera or as fallback
            if not self.enable_camera or self.latest_image is None:
                self.latest_image = cv_image
                
                # Convert to PIL for processing
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                self.latest_pil_image = PILImage.fromarray(rgb_image)
            
        except Exception as e:
            self.get_logger().error(f"Error processing external image: {e}")

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

    def get_worker_service_client(self, worker_name):
        """Get or create a service client for a specific worker"""
        if worker_name not in self.worker_clients:
            service_name = f"/model_worker_{worker_name}/generate_stream"
            self.worker_clients[worker_name] = self.create_client(
                WorkerGenerateStream, 
                service_name
            )
            self.get_logger().info(f"Created client for worker service: {service_name}")
            
            # Wait for the service to be available
            if not self.worker_clients[worker_name].wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f"Worker service {service_name} not available")
                
        return self.worker_clients[worker_name]

    def pil_image_to_base64(self, pil_image):
        """Convert PIL image to base64 string"""
        buffered = io.BytesIO()
        pil_image.save(buffered, format="JPEG", quality=85)
        img_str = base64.b64encode(buffered.getvalue()).decode()
        return img_str

    def query_model(self, state):
        """Query the vision-language model via ROS2 services"""
        try:
            # Get worker address
            get_worker_request = GetWorkerAddress.Request()
            get_worker_request.model = self.model_name
            
            worker_future = self.get_worker_client.call_async(get_worker_request)
            rclpy.spin_until_future_complete(self, worker_future, timeout_sec=5.0)
            
            if worker_future.result() is None:
                self.get_logger().error("Failed to get worker address - service timeout")
                return None
                
            worker_result = worker_future.result()
            if not worker_result.success or not worker_result.address:
                self.get_logger().error(f"No available worker for model {self.model_name}")
                return None
            
            worker_name = worker_result.address
            
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
            
            # Prepare request
            prompt = state.get_prompt()
            pil_images, images, transforms = state.get_images()
            
            # Convert PIL images to base64 strings for ROS2 message
            image_data = []
            if pil_images:
                for pil_img in pil_images:
                    img_base64 = self.pil_image_to_base64(pil_img)
                    image_data.append(img_base64)
                    
                    # Save images for logging
                    image_hash = hashlib.md5(pil_img.tobytes()).hexdigest()
                    t = datetime.datetime.now()
                    filename = os.path.join(LOGDIR, "serve_images", 
                                          f"{t.year}-{t.month:02d}-{t.day:02d}", 
                                          f"{image_hash}.jpg")
                    if not os.path.isfile(filename):
                        os.makedirs(os.path.dirname(filename), exist_ok=True)
                        pil_img.save(filename)
            
            # Get worker service client
            worker_client = self.get_worker_service_client(worker_name)
            
            # Prepare generate request
            generate_request = WorkerGenerateStream.Request()
            generate_request.model = self.model_name
            generate_request.prompt = prompt
            generate_request.temperature = self.temperature
            generate_request.top_p = self.top_p
            generate_request.max_new_tokens = min(int(self.max_output_tokens), 1536)
            generate_request.images = image_data
            
            # Determine stop tokens
            if state.sep_style in [SeparatorStyle.SINGLE, SeparatorStyle.MPT]:
                generate_request.stop = state.sep
            else:
                generate_request.stop = state.sep2
            
            # Call worker service
            generate_future = worker_client.call_async(generate_request)
            rclpy.spin_until_future_complete(self, generate_future, timeout_sec=self.service_timeout)
            
            if generate_future.result() is None:
                self.get_logger().error("Worker generate service timeout")
                return None
                
            generate_result = generate_future.result()
            if not generate_result.success:
                self.get_logger().error(f"Worker generate error: {generate_result.error_message}")
                return None
            
            # Extract generated text (remove prompt part)
            output = generate_result.text
            if output.startswith(prompt):
                output = output[len(prompt):].strip()
            
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

    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.cleanup_camera()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = RoboPointNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node:
            node.cleanup_camera()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()