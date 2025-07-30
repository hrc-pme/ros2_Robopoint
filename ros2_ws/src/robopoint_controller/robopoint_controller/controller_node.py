#!/usr/bin/env python3
"""
A ROS2 controller node that manages distributed workers.
It handles worker registration and provides worker addresses to clients.
"""
import argparse
import dataclasses
from enum import Enum, auto
import json
import threading
import time
from typing import List, Dict, Any

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# ROS2 message imports
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist  # Using as placeholder for custom messages

# You'll need to create these custom message/service types in a separate package
# For now, using standard types as placeholders
from robopoint_msgs.srv import (
    RegisterAllWorkers, 
    GetWorkerAddress, 
    ReceiveHeartBeat,
    ListModels,
    WorkerGenerateStream,
    WorkerGetStatus
)
from robopoint_msgs.msg import HeartBeat, WorkerStatus

from robopoint_controller.constants import CONTROLLER_HEART_BEAT_EXPIRATION
from robopoint_controller.utils import build_logger, server_error_msg

logger = build_logger("ros2_controller", "ros2_controller.log")


class DispatchMethod(Enum):
    LOTTERY = auto()
    SHORTEST_QUEUE = auto()

    @classmethod
    def from_str(cls, name):
        if name == "lottery":
            return cls.LOTTERY
        elif name == "shortest_queue":
            return cls.SHORTEST_QUEUE
        else:
            raise ValueError(f"Invalid dispatch method")


@dataclasses.dataclass
class WorkerInfo:
    model_names: List[str]
    speed: int
    queue_length: int
    check_heart_beat: bool
    last_heart_beat: float


class ControllerNode(Node):
    def __init__(self, dispatch_method: str):
        super().__init__('controller_node')
        
        # Initialize worker info dictionary
        self.worker_info: Dict[str, WorkerInfo] = {}
        self.dispatch_method = DispatchMethod.from_str(dispatch_method)
        
        # Create a reentrant callback group for concurrent service calls
        self.callback_group = ReentrantCallbackGroup()
        
        # Services
        self.register_worker_srv = self.create_service(
            RegisterAllWorkers, 
            '/register_all_workers', 
            self.register_worker_callback,
            callback_group=self.callback_group
        )
        
        self.refresh_workers_srv = self.create_service(
            Empty, 
            '/refresh_all_workers', 
            self.refresh_all_workers_callback,
            callback_group=self.callback_group
        )
        
        self.list_models_srv = self.create_service(
            ListModels, 
            '/list_models', 
            self.list_models_callback,
            callback_group=self.callback_group
        )
        
        self.get_worker_address_srv = self.create_service(
            GetWorkerAddress, 
            '/get_worker_address', 
            self.get_worker_address_callback,
            callback_group=self.callback_group
        )
        
        self.receive_heartbeat_srv = self.create_service(
            ReceiveHeartBeat, 
            '/receive_heart_beat', 
            self.receive_heart_beat_callback,
            callback_group=self.callback_group
        )
        
        self.worker_generate_stream_srv = self.create_service(
            WorkerGenerateStream, 
            '/worker_generate_stream', 
            self.worker_generate_stream_callback,
            callback_group=self.callback_group
        )
        
        self.worker_get_status_srv = self.create_service(
            WorkerGetStatus, 
            '/worker_get_status', 
            self.worker_get_status_callback,
            callback_group=self.callback_group
        )
        
        # Publishers
        self.status_publisher = self.create_publisher(
            WorkerStatus, 
            '/controller_status', 
            10
        )
        
        # Subscribers
        self.heartbeat_subscriber = self.create_subscription(
            HeartBeat,
            '/worker_heartbeat',
            self.heartbeat_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Service clients for communicating with workers
        self.worker_clients = {}
        
        # Heart beat timer
        self.heartbeat_timer = self.create_timer(
            CONTROLLER_HEART_BEAT_EXPIRATION,
            self.heartbeat_timer_callback
        )
        
        # Status publishing timer
        self.status_timer = self.create_timer(
            1.0,  # Publish status every second
            self.publish_status
        )
        
        self.get_logger().info("Controller node initialized")

    def register_worker_callback(self, request, response):
        """Handle worker registration requests"""
        try:
            success = self.register_worker(
                request.worker_name,
                request.check_heart_beat,
                json.loads(request.worker_status) if request.worker_status else None
            )
            response.success = success
            response.message = f"Worker {request.worker_name} registered successfully" if success else "Registration failed"
        except Exception as e:
            self.get_logger().error(f"Error in register_worker: {e}")
            response.success = False
            response.message = str(e)
        
        return response

    def register_worker(self, worker_name: str, check_heart_beat: bool, worker_status: dict = None):
        """Register a worker with the controller"""
        if worker_name not in self.worker_info:
            self.get_logger().info(f"Register a new worker: {worker_name}")
        else:
            self.get_logger().info(f"Register an existing worker: {worker_name}")

        if not worker_status:
            worker_status = self.get_worker_status_direct(worker_name)
        if not worker_status:
            return False

        self.worker_info[worker_name] = WorkerInfo(
            worker_status["model_names"], 
            worker_status["speed"], 
            worker_status["queue_length"],
            check_heart_beat, 
            time.time()
        )

        # Create service client for this worker if not exists
        if worker_name not in self.worker_clients:
            self.worker_clients[worker_name] = self.create_client(
                WorkerGetStatus, 
                f'{worker_name}/worker_get_status'
            )

        self.get_logger().info(f"Register done: {worker_name}, {worker_status}")
        return True

    def get_worker_status_direct(self, worker_name: str):
        """Get worker status directly via service call"""
        if worker_name not in self.worker_clients:
            return None
            
        client = self.worker_clients[worker_name]
        if not client.service_is_ready():
            self.get_logger().warning(f"Service not ready for worker: {worker_name}")
            return None
        
        request = WorkerGetStatus.Request()
        try:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                result = future.result()
                return {
                    "model_names": json.loads(result.model_names),
                    "speed": result.speed,
                    "queue_length": result.queue_length
                }
            else:
                self.get_logger().error(f"Service call failed for worker: {worker_name}")
                return None
        except Exception as e:
            self.get_logger().error(f"Error getting worker status: {worker_name}, {e}")
            return None

    def refresh_all_workers_callback(self, request, response):
        """Refresh all workers"""
        try:
            self.refresh_all_workers()
            return response
        except Exception as e:
            self.get_logger().error(f"Error refreshing workers: {e}")
            return response

    def refresh_all_workers(self):
        """Refresh all worker information"""
        old_info = dict(self.worker_info)
        self.worker_info = {}

        for w_name, w_info in old_info.items():
            if not self.register_worker(w_name, w_info.check_heart_beat, None):
                self.get_logger().info(f"Remove stale worker: {w_name}")

    def list_models_callback(self, request, response):
        """List all available models"""
        try:
            models = self.list_models()
            response.models = json.dumps(models)
            return response
        except Exception as e:
            self.get_logger().error(f"Error listing models: {e}")
            response.models = "[]"
            return response

    def list_models(self):
        """Get list of all available models"""
        model_names = set()
        for w_name, w_info in self.worker_info.items():
            model_names.update(w_info.model_names)
        return list(model_names)

    def get_worker_address_callback(self, request, response):
        """Get worker address for a specific model"""
        try:
            address = self.get_worker_address(request.model)
            response.address = address
            return response
        except Exception as e:
            self.get_logger().error(f"Error getting worker address: {e}")
            response.address = ""
            return response

    def get_worker_address(self, model_name: str):
        """Get worker address using the specified dispatch method"""
        if self.dispatch_method == DispatchMethod.LOTTERY:
            return self._lottery_dispatch(model_name)
        elif self.dispatch_method == DispatchMethod.SHORTEST_QUEUE:
            return self._shortest_queue_dispatch(model_name)
        else:
            raise ValueError(f"Invalid dispatch method: {self.dispatch_method}")

    def _lottery_dispatch(self, model_name: str):
        """Lottery-based worker selection"""
        worker_names = []
        worker_speeds = []
        
        for w_name, w_info in self.worker_info.items():
            if model_name in w_info.model_names:
                worker_names.append(w_name)
                worker_speeds.append(w_info.speed)
        
        if not worker_names:
            return ""
            
        worker_speeds = np.array(worker_speeds, dtype=np.float32)
        norm = np.sum(worker_speeds)
        if norm < 1e-4:
            return ""
            
        worker_speeds = worker_speeds / norm
        pt = np.random.choice(np.arange(len(worker_names)), p=worker_speeds)
        return worker_names[pt]

    def _shortest_queue_dispatch(self, model_name: str):
        """Shortest queue based worker selection"""
        worker_names = []
        worker_qlen = []
        
        for w_name, w_info in self.worker_info.items():
            if model_name in w_info.model_names:
                worker_names.append(w_name)
                worker_qlen.append(w_info.queue_length / w_info.speed)
        
        if len(worker_names) == 0:
            return ""
            
        min_index = np.argmin(worker_qlen)
        w_name = worker_names[min_index]
        self.worker_info[w_name].queue_length += 1
        
        self.get_logger().info(f"names: {worker_names}, queue_lens: {worker_qlen}, ret: {w_name}")
        return w_name

    def receive_heart_beat_callback(self, request, response):
        """Handle heart beat reception"""
        try:
            exists = self.receive_heart_beat(request.worker_name, request.queue_length)
            response.exists = exists
            return response
        except Exception as e:
            self.get_logger().error(f"Error receiving heartbeat: {e}")
            response.exists = False
            return response

    def receive_heart_beat(self, worker_name: str, queue_length: int):
        """Process heart beat from worker"""
        if worker_name not in self.worker_info:
            self.get_logger().info(f"Receive unknown heart beat: {worker_name}")
            return False

        self.worker_info[worker_name].queue_length = queue_length
        self.worker_info[worker_name].last_heart_beat = time.time()
        self.get_logger().info(f"Receive heart beat: {worker_name}")
        return True

    def heartbeat_callback(self, msg):
        """Handle heartbeat messages from topic"""
        self.receive_heart_beat(msg.worker_name, msg.queue_length)

    def heartbeat_timer_callback(self):
        """Timer callback for checking worker heartbeats"""
        self.remove_stale_workers_by_expiration()

    def remove_stale_workers_by_expiration(self):
        """Remove workers that haven't sent heartbeats"""
        expire = time.time() - CONTROLLER_HEART_BEAT_EXPIRATION
        to_delete = []
        
        for worker_name, w_info in self.worker_info.items():
            if w_info.check_heart_beat and w_info.last_heart_beat < expire:
                to_delete.append(worker_name)

        for worker_name in to_delete:
            self.remove_worker(worker_name)
            self.get_logger().info(f"Removed expired worker: {worker_name}")

    def remove_worker(self, worker_name: str):
        """Remove a worker from the registry"""
        if worker_name in self.worker_info:
            del self.worker_info[worker_name]
        if worker_name in self.worker_clients:
            self.worker_clients[worker_name].destroy()
            del self.worker_clients[worker_name]

    def worker_generate_stream_callback(self, request, response):
        """Handle stream generation requests"""
        try:
            # This would need to be implemented based on your streaming requirements
            # ROS2 services don't natively support streaming, so you might need to use actions
            # or implement a custom solution with topics
            worker_addr = self.get_worker_address(request.model)
            if not worker_addr:
                response.success = False
                response.error_message = f"No worker available for model: {request.model}"
            else:
                response.success = True
                response.worker_address = worker_addr
                # Additional streaming logic would go here
            return response
        except Exception as e:
            self.get_logger().error(f"Error in worker_generate_stream: {e}")
            response.success = False
            response.error_message = str(e)
            return response

    def worker_get_status_callback(self, request, response):
        """Get aggregated status of all workers"""
        try:
            status = self.worker_api_get_status()
            response.model_names = json.dumps(status["model_names"])
            response.speed = status["speed"]
            response.queue_length = status["queue_length"]
            return response
        except Exception as e:
            self.get_logger().error(f"Error getting worker status: {e}")
            response.model_names = "[]"
            response.speed = 0
            response.queue_length = 0
            return response

    def worker_api_get_status(self):
        """Get aggregated status from all workers"""
        model_names = set()
        speed = 0
        queue_length = 0

        for w_name in self.worker_info:
            worker_status = self.get_worker_status_direct(w_name)
            if worker_status is not None:
                model_names.update(worker_status["model_names"])
                speed += worker_status["speed"]
                queue_length += worker_status["queue_length"]

        return {
            "model_names": list(model_names),
            "speed": speed,
            "queue_length": queue_length,
        }

    def publish_status(self):
        """Publish controller status periodically"""
        try:
            status = self.worker_api_get_status()
            msg = WorkerStatus()
            msg.model_names = json.dumps(status["model_names"])
            msg.speed = status["speed"]
            msg.queue_length = status["queue_length"]
            msg.worker_count = len(self.worker_info)
            self.status_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--dispatch-method", type=str, 
                       choices=["lottery", "shortest_queue"], 
                       default="shortest_queue")
    parsed_args, unknown = parser.parse_known_args()
    
    # Create and run the controller node
    controller_node = ControllerNode(parsed_args.dispatch_method)
    
    # Use MultiThreadedExecutor for concurrent service calls
    executor = MultiThreadedExecutor()
    executor.add_node(controller_node)
    
    try:
        controller_node.get_logger().info("Controller node started")
        executor.spin()
    except KeyboardInterrupt:
        controller_node.get_logger().info("Controller node shutting down")
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()