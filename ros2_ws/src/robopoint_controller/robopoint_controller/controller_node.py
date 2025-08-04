#!/usr/bin/env python3
"""
A ROS2 controller node that manages distributed workers.
修正版本：完整的 worker 管理和訂閱機制
"""
import argparse
import dataclasses
from enum import Enum, auto
import json
import threading
import time
from typing import List, Dict, Any
from concurrent.futures import TimeoutError as FutureTimeout
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# ROS2 message imports
from std_msgs.msg import String
from std_srvs.srv import Empty

# 自定義訊息類型
from robopoint_interfaces.srv import (
    RegisterAllWorkers, 
    GetWorkerAddress, 
    ReceiveHeartBeat,
    ListModels,
    WorkerGenerateStream,
    WorkerGetStatus
)
from robopoint_interfaces.msg import WorkerStatus
from robopoint_interfaces.srv import RefreshAllWorkers  # RoboPointNode 使用的服務類型

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
        
        # 🔧 核心服務：供 RoboPointNode 調用
        self.refresh_workers_srv = self.create_service(
            RefreshAllWorkers,  # 使用 RoboPointNode 期望的服務類型
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
        
        # 🔧 Worker 管理服務：供 ModelWorker 註冊使用
        self.register_worker_srv = self.create_service(
            RegisterAllWorkers, 
            '/register_all_workers', 
            self.register_worker_callback,
            callback_group=self.callback_group
        )
        
        self.receive_heartbeat_srv = self.create_service(
            ReceiveHeartBeat, 
            '/receive_heart_beat', 
            self.receive_heart_beat_callback,
            callback_group=self.callback_group
        )
        
        # 🔧 訂閱者：接收 Worker 心跳
        self.heartbeat_subscriber = self.create_subscription(
            WorkerStatus,  # 使用正確的訊息類型
            '/worker_heartbeat',
            self.heartbeat_topic_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Publishers - 發布控制器狀態
        self.status_publisher = self.create_publisher(
            WorkerStatus, 
            '/controller_status', 
            10
        )
        
        # Service clients - 用於與各個 Worker 通信
        self.worker_clients: Dict[str, Any] = {}
        
        # Timers
        self.heartbeat_timer = self.create_timer(
            CONTROLLER_HEART_BEAT_EXPIRATION,
            self.heartbeat_timer_callback,
            callback_group=self.callback_group
        )
        
        self.status_timer = self.create_timer(
            5.0,  # 每 5 秒發布一次狀態
            self.publish_status,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("Controller node initialized")
        self.get_logger().info("Available services:")
        self.get_logger().info("  - /refresh_all_workers (for RoboPointNode)")
        self.get_logger().info("  - /list_models (for RoboPointNode)")
        self.get_logger().info("  - /get_worker_address (for RoboPointNode)")
        self.get_logger().info("  - /register_all_workers (for ModelWorker)")
        self.get_logger().info("Subscribed topics:")
        self.get_logger().info("  - /worker_heartbeat (from ModelWorkers)")

    def register_worker_callback(self, request, response):
        """處理 ModelWorker 的註冊請求"""
        try:
            self.get_logger().info(f"Received registration request from worker: {request.worker_name}")
            
            # 解析 worker 狀態
            if request.worker_status:
                worker_status = json.loads(request.worker_status)
            else:
                worker_status = {
                    "model_names": ["unknown"],
                    "speed": 1.0,
                    "queue_length": 0
                }
            
            success = self.register_worker(
                request.worker_name,
                request.check_heart_beat,
                worker_status
            )
            
            response.success = success
            response.message = f"Worker {request.worker_name} registered successfully" if success else "Registration failed"
            
            self.get_logger().info(f"Worker {request.worker_name} registration: {'SUCCESS' if success else 'FAILED'}")
            
        except Exception as e:
            self.get_logger().error(f"Error in register_worker: {e}")
            response.success = False
            response.message = str(e)
        
        return response

    def register_worker(self, worker_name: str, check_heart_beat: bool, worker_status: dict = None):
        """註冊 Worker"""
        if worker_name not in self.worker_info:
            self.get_logger().info(f"Register a new worker: {worker_name}")
        else:
            self.get_logger().info(f"Re-register existing worker: {worker_name}")

        if not worker_status:
            # 嘗試直接獲取 worker 狀態
            worker_status = self.get_worker_status_direct(worker_name)
        
        if not worker_status:
            self.get_logger().warn(f"Failed to get status for worker: {worker_name}")
            return False

        # 確保 model_names 是列表
        model_names = worker_status.get("model_names", [])
        if isinstance(model_names, str):
            try:
                # 如果是 JSON 格式的字串就轉成 list
                model_names = json.loads(model_names)
            except Exception:
                model_names = [model_names]

        self.worker_info[worker_name] = WorkerInfo(
            model_names, 
            int(worker_status.get("speed", 1)), 
            int(worker_status.get("queue_length", 0)),
            check_heart_beat, 
            time.time()
        )

        # 創建與該 worker 通信的 service client
        if worker_name not in self.worker_clients:
            try:
                self.worker_clients[worker_name] = self.create_client(
                    WorkerGetStatus, 
                    f'/model_worker_{worker_name}/worker_get_status'
                )
                self.get_logger().info(f"Created service client for worker: {worker_name}")
            except Exception as e:
                self.get_logger().error(f"Failed to create service client for {worker_name}: {e}")

        self.get_logger().info(f"Register done: {worker_name}, models: {model_names}")
        return True

    def get_worker_status_direct(self, worker_name: str):
        """直接從 Worker 獲取狀態"""
        if worker_name not in self.worker_clients:
            return None
            
        client = self.worker_clients[worker_name]
        if not client.service_is_ready():
            self.get_logger().warn(f"Service /model_worker_{worker_name}/worker_get_status not ready")
            return None

        request = WorkerGetStatus.Request()
        try:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

            if future.result() is not None:
                result = future.result()
                model_names = json.loads(result.model_names) if isinstance(result.model_names, str) else result.model_names
                return {
                    "model_names": model_names,
                    "speed": result.speed,
                    "queue_length": result.queue_length
                }
            else:
                self.get_logger().warn(f"Service call returned None for worker: {worker_name}")
                return None

        except FutureTimeout:
            self.get_logger().warn(f"Service call timed out for worker: {worker_name}")
            return None
        except Exception as e:
            self.get_logger().error(f"Error getting worker status: {worker_name}, {e}")
            return None

    def refresh_all_workers_callback(self, request, response):
        """處理來自 RoboPointNode 的刷新請求"""
        try:
            self.get_logger().info("Received refresh_all_workers request from RoboPointNode")
            
            # 👉 不再呼叫 self.refresh_all_workers()，因為不會刷新新 worker，只會複寫舊的
            refreshed_count = len(self.worker_info)
            all_models = self.list_models()

            response.success = refreshed_count > 0
            response.message = f"Controller has {refreshed_count} workers with models: {all_models}"
            self.get_logger().info(f"Controller status: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Error refreshing workers: {e}")
            response.success = False
            response.message = str(e)
        
        return response

    def refresh_all_workers(self):
        """刷新所有 Worker 資訊"""
        old_info = dict(self.worker_info)
        refreshed_count = 0

        for w_name, w_info in old_info.items():
            # 不要先清空 worker_info
            # 嘗試從原始記憶體中重新註冊
            if self.register_worker(w_name, w_info.check_heart_beat, None):
                refreshed_count += 1
            else:
                self.get_logger().info(f"Remove stale worker: {w_name}")
                # 清理失效 worker
                if w_name in self.worker_clients:
                    try:
                        self.worker_clients[w_name].destroy()
                        del self.worker_clients[w_name]
                    except:
                        pass
                if w_name in self.worker_info:
                    del self.worker_info[w_name]

        self.get_logger().info(f"Refresh completed: {refreshed_count}/{len(old_info)} workers active")
        for w_name, w_info in self.worker_info.items():
            self.get_logger().info(f"[DEBUG] Worker={w_name}, models={w_info.model_names}")

    def list_models_callback(self, request, response):
        """處理來自 RoboPointNode 的模型列表請求"""
        self.get_logger().info(">>> [list_models_callback] triggered")
        try:
            models = self.list_models()
            worker_ids = list(self.worker_info.keys())  # 獲取所有 worker IDs
            
            response.success = True
            response.model_names = models
            response.worker_ids = worker_ids  # ← 修復：添加缺失的字段
            response.message = f"Found {len(models)} models"
            
            self.get_logger().info(f"Returning model list: {models}")
            self.get_logger().info(f"Available workers: {worker_ids}")
            
        except Exception as e:
            self.get_logger().error(f"Error listing models: {e}")
            response.success = False
            response.model_names = []
            response.worker_ids = []  # ← 修復：添加缺失的字段
            response.message = str(e)
        
        return response

    def list_models(self):
        """獲取所有可用模型列表"""
        model_names = set()
        for w_name, w_info in self.worker_info.items():
            self.get_logger().debug(f"[DEBUG] worker={w_name}, model_names={w_info.model_names}")
            model_names.update(w_info.model_names)
        
        model_list = list(model_names)
        self.get_logger().debug(f"Available models: {model_list}")
        return model_list

    def get_worker_address_callback(self, request, response):
        """處理來自 RoboPointNode 的 worker 地址請求"""
        try:
            self.get_logger().info(f"Received get_worker_address request for model: {request.model}")
            address = self.get_worker_address(request.model)
            
            if address:
                # 獲取該 worker 的模型列表
                worker_models = []
                if address in self.worker_info:
                    worker_models = self.worker_info[address].model_names
                
                response.success = True
                response.worker_id = address        # ← 修復：設置 worker_id 而不是 address
                response.address = address          # ← 保持原有的 address 字段
                response.model_names = worker_models # ← 修復：添加 worker 的模型列表
                response.message = f"Found worker {address} for model {request.model}"
                
                self.get_logger().info(f"Returning worker: id={address}, models={worker_models}")
            else:
                response.success = False
                response.worker_id = ""             # ← 修復：設置空的 worker_id
                response.address = ""
                response.model_names = []           # ← 修復：設置空的模型列表
                response.message = f"No worker available for model: {request.model}"
                
                self.get_logger().warn(f"No worker found for model: {request.model}")
                
        except Exception as e:
            self.get_logger().error(f"Error getting worker address: {e}")
            response.success = False
            response.worker_id = ""                 # ← 修復：設置空的 worker_id
            response.address = ""
            response.model_names = []               # ← 修復：設置空的模型列表
            response.message = str(e)
        
        return response

    def get_worker_address(self, model_name: str):
        """根據調度方法獲取 worker 地址"""
        if self.dispatch_method == DispatchMethod.LOTTERY:
            return self._lottery_dispatch(model_name)
        elif self.dispatch_method == DispatchMethod.SHORTEST_QUEUE:
            return self._shortest_queue_dispatch(model_name)
        else:
            raise ValueError(f"Invalid dispatch method: {self.dispatch_method}")

    def _lottery_dispatch(self, model_name: str):
        """基於彩票的 worker 選擇"""
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
        """基於最短隊列的 worker 選擇"""
        worker_names = []
        worker_qlen = []
        
        for w_name, w_info in self.worker_info.items():
            if model_name in w_info.model_names:
                worker_names.append(w_name)
                worker_qlen.append(w_info.queue_length / max(w_info.speed, 1))
        
        if len(worker_names) == 0:
            return ""
            
        min_index = np.argmin(worker_qlen)
        w_name = worker_names[min_index]
        self.worker_info[w_name].queue_length += 1
        
        self.get_logger().info(f"Shortest queue dispatch - workers: {worker_names}, queue_lens: {worker_qlen}, selected: {w_name}")
        return w_name

    def receive_heart_beat_callback(self, request, response):
        """處理心跳服務請求"""
        try:
            exists = self.receive_heart_beat(request.worker_name, request.queue_length)
            response.exists = exists
            return response
        except Exception as e:
            self.get_logger().error(f"Error receiving heartbeat: {e}")
            response.exists = False
            return response

    def heartbeat_topic_callback(self, msg: WorkerStatus):
        """處理來自 topic 的心跳訊息"""
        try:
            if msg.is_registration:
                # 這是註冊訊息
                self.get_logger().info(f"Received registration heartbeat from: {msg.worker_id}")
                worker_status = {
                    "model_names": [msg.model_names] if isinstance(msg.model_names, str) else msg.model_names,
                    "speed": 1.0,
                    "queue_length": msg.queue_length
                }
                self.register_worker(msg.worker_id, True, worker_status)
            else:
                # 這是一般心跳
                self.receive_heart_beat(msg.worker_id, msg.queue_length)
                
        except Exception as e:
            self.get_logger().error(f"Error processing heartbeat topic message: {e}")

    def receive_heart_beat(self, worker_name: str, queue_length: int):
        """處理心跳"""
        if worker_name not in self.worker_info:
            self.get_logger().debug(f"Receive heartbeat from unknown worker: {worker_name}")
            return False

        self.worker_info[worker_name].queue_length = queue_length
        self.worker_info[worker_name].last_heart_beat = time.time()
        self.get_logger().debug(f"Heartbeat received: {worker_name}, queue: {queue_length}")
        return True

    def heartbeat_timer_callback(self):
        """心跳檢查定時器"""
        self.remove_stale_workers_by_expiration()

    def remove_stale_workers_by_expiration(self):
        """移除過期的 workers"""
        expire = time.time() - CONTROLLER_HEART_BEAT_EXPIRATION
        to_delete = []
        
        for worker_name, w_info in self.worker_info.items():
            if w_info.check_heart_beat and w_info.last_heart_beat < expire:
                to_delete.append(worker_name)

        for worker_name in to_delete:
            self.remove_worker(worker_name)
            self.get_logger().info(f"Removed expired worker: {worker_name}")

    def remove_worker(self, worker_name: str):
        """移除 worker"""
        if worker_name in self.worker_info:
            del self.worker_info[worker_name]
        if worker_name in self.worker_clients:
            try:
                self.worker_clients[worker_name].destroy()
                del self.worker_clients[worker_name]
            except:
                pass

    def publish_status(self):
        """發布控制器狀態"""
        try:
            status = self.get_controller_status()
            msg = WorkerStatus()
            msg.worker_id = "controller"
            msg.model_names = json.dumps(status["model_names"])
            msg.speed = status["total_speed"]
            msg.queue_length = status["total_queue_length"]
            msg.global_counter = len(self.worker_info)  # 使用 worker 數量作為計數器
            msg.is_registration = False
            
            self.status_publisher.publish(msg)
            
            self.get_logger().debug(
                f"Controller status - Workers: {len(self.worker_info)}, "
                f"Models: {len(status['model_names'])}, "
                f"Total queue: {status['total_queue_length']}"
            )
            
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")

    def get_controller_status(self):
        """獲取控制器狀態"""
        model_names = set()
        total_speed = 0
        total_queue_length = 0

        for w_name, w_info in self.worker_info.items():
            model_names.update(w_info.model_names)
            total_speed += w_info.speed
            total_queue_length += w_info.queue_length

        return {
            "model_names": list(model_names),
            "total_speed": total_speed,
            "total_queue_length": total_queue_length,
            "worker_count": len(self.worker_info)
        }


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
        controller_node.get_logger().info(f"Active workers: {len(controller_node.worker_info)}")
        executor.spin()
    except KeyboardInterrupt:
        controller_node.get_logger().info("Controller node shutting down")
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()