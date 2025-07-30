#!/usr/bin/env python3
"""
ROS2 Model Worker Node - 執行多模態模型的 ROS2 節點
"""
import json
import time
import threading
import uuid
from typing import Optional, List, Dict, Any

import torch
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from transformers import TextIteratorStreamer
from threading import Thread, Semaphore
import base64
from io import BytesIO
from PIL import Image as PILImage

# 自定義訊息類型 (需要在 package 中定義)
from robopoint_msgs.msg import GenerateRequest, GenerateResponse, WorkerStatus
from robopoint_msgs.srv import WorkerGetStatus, WorkerGenerateStream
# 新增：導入缺失的服務類型
from robopoint_interfaces.srv import RefreshAllWorkers, ListModels, GetWorkerAddress

from robopoint_worker.model.builder import load_pretrained_model
from robopoint_worker.mm_utils import process_images, load_image_from_base64, tokenizer_image_token
from robopoint_controller.constants import IMAGE_TOKEN_INDEX, DEFAULT_IMAGE_TOKEN, DEFAULT_IM_START_TOKEN, DEFAULT_IM_END_TOKEN
from robopoint_controller.utils import build_logger, server_error_msg


class ROS2ModelWorker(Node):
    def __init__(self):
        super().__init__('model_worker')
        
        # 聲明參數
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_path', 'facebook/opt-350m'),
                ('model_base', ''),
                ('model_name', ''),
                ('device', 'cuda'),
                ('load_8bit', False),
                ('load_4bit', False),
                ('use_flash_attn', False),
                ('limit_model_concurrency', 5),
                ('heartbeat_interval', 10.0),
                ('controller_topic', '/model_controller'),
                ('worker_id', str(uuid.uuid4())[:6])
            ]
        )
        
        # 獲取參數
        self.model_path = self.get_parameter('model_path').value
        self.model_base = self.get_parameter('model_base').value
        self.model_name = self.get_parameter('model_name').value
        self.device = self.get_parameter('device').value
        self.load_8bit = self.get_parameter('load_8bit').value
        self.load_4bit = self.get_parameter('load_4bit').value
        self.use_flash_attn = self.get_parameter('use_flash_attn').value
        self.limit_model_concurrency = self.get_parameter('limit_model_concurrency').value
        self.heartbeat_interval = self.get_parameter('heartbeat_interval').value
        self.controller_topic = self.get_parameter('controller_topic').value
        self.worker_id = self.get_parameter('worker_id').value
        
        # 初始化組件
        self.callback_group = ReentrantCallbackGroup()
        self.cv_bridge = CvBridge()
        self.logger = build_logger("ros2_model_worker", f"ros2_model_worker_{self.worker_id}.log")
        self.global_counter = 0
        
        # 模型並發控制
        self.model_semaphore = Semaphore(self.limit_model_concurrency)
        
        # 處理模型路徑和名稱
        if self.model_path.endswith("/"):
            self.model_path = self.model_path[:-1]
            
        if not self.model_name:
            model_paths = self.model_path.split("/")
            if model_paths[-1].startswith('checkpoint-'):
                self.model_name = model_paths[-2] + "_" + model_paths[-1]
            else:
                self.model_name = model_paths[-1]
        
        # 載入模型
        self.get_logger().info(f'Loading model {self.model_name} on worker {self.worker_id}...')
        self.tokenizer, self.model, self.image_processor, self.context_len = load_pretrained_model(
            self.model_path, 
            self.model_base if self.model_base else None, 
            self.model_name, 
            self.load_8bit, 
            self.load_4bit, 
            device=self.device, 
            use_flash_attn=self.use_flash_attn
        )
        
        # 建立 publishers
        self.heartbeat_pub = self.create_publisher(
            WorkerStatus, 
            f'{self.controller_topic}/heartbeat', 
            10
        )
        
        self.response_pub = self.create_publisher(
            GenerateResponse,
            f'/model_worker_{self.worker_id}/response',
            10
        )
        
        # 建立 subscribers
        self.request_sub = self.create_subscription(
            GenerateRequest,
            f'/model_worker_{self.worker_id}/request',
            self.generate_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 建立 services
        self.status_service = self.create_service(
            WorkerGetStatus,
            f'/model_worker_{self.worker_id}/get_status',
            self.get_status_callback,
            callback_group=self.callback_group
        )
        
        # 🔧 修正：新增缺失的三個關鍵服務
        self.refresh_service = self.create_service(
            RefreshAllWorkers,
            '/refresh_all_workers',
            self.handle_refresh_all_workers,
            callback_group=self.callback_group
        )
        
        self.list_models_service = self.create_service(
            ListModels,
            '/list_models',
            self.handle_list_models,
            callback_group=self.callback_group
        )
        
        self.get_worker_address_service = self.create_service(
            GetWorkerAddress,
            '/get_worker_address',
            self.handle_get_worker_address,
            callback_group=self.callback_group
        )
        
        # 建立 heartbeat timer
        self.heartbeat_timer = self.create_timer(
            self.heartbeat_interval,
            self.send_heartbeat,
            callback_group=self.callback_group
        )
        
        # 註冊到控制器
        self.register_to_controller()
        
        self.get_logger().info(f'Model worker {self.worker_id} initialized successfully')
        self.get_logger().info(f'Registered services: /refresh_all_workers, /list_models, /get_worker_address')
    
    def register_to_controller(self):
        """註冊到控制器"""
        self.get_logger().info('Registering to controller...')
        
        # 發布註冊訊息
        status_msg = WorkerStatus()
        status_msg.worker_id = self.worker_id
        status_msg.model_names = self.model_name
        status_msg.queue_length = self.get_queue_length()
        status_msg.is_registration = True
        
        self.heartbeat_pub.publish(status_msg)
    
    def send_heartbeat(self):
        """發送心跳訊息"""
        status_msg = WorkerStatus()
        status_msg.worker_id = self.worker_id
        status_msg.model_names = self.model_name
        status_msg.queue_length = self.get_queue_length()
        status_msg.global_counter = self.global_counter
        status_msg.is_registration = False
        
        self.heartbeat_pub.publish(status_msg)
        
        self.get_logger().debug(
            f'Heartbeat sent - Models: {[self.model_name]}, '
            f'Queue: {self.get_queue_length()}, '
            f'Counter: {self.global_counter}'
        )
    
    def get_queue_length(self) -> int:
        """獲取佇列長度"""
        if self.model_semaphore is None:
            return 0
        return self.limit_model_concurrency - self.model_semaphore._value
    
    def get_status_callback(self, request, response):
        """處理狀態查詢服務"""
        response.worker_id = self.worker_id
        response.model_names = self.model_name
        response.queue_length = self.get_queue_length()
        response.speed = 1.0
        return response
    
    # 🔧 修正：新增缺失的三個服務 callback
    def handle_refresh_all_workers(self, request, response):
        """處理刷新所有 worker 的請求"""
        self.get_logger().info("Received refresh_all_workers request")
        
        # 重新發送註冊訊息
        self.register_to_controller()
        
        # 可以在這裡添加其他刷新邏輯，比如重新載入配置等
        response.success = True
        response.message = f"Worker {self.worker_id} refreshed successfully"
        
        return response
    
    def handle_list_models(self, request, response):
        """處理列出模型的請求"""
        self.get_logger().info(f"Responding to list_models request with model: {self.model_name}, worker: {self.worker_id}")
        response.success = True
        response.model_names = [self.model_name]
        response.worker_ids = [self.worker_id]
        response.message = "OK"
        return response
    
    def handle_get_worker_address(self, request, response):
        """處理獲取 worker 地址的請求"""
        self.get_logger().info(f"Received get_worker_address request for model: {request.model}")
        
        # 檢查請求的模型是否匹配
        if request.model == self.model_name or not request.model:
            response.success = True
            response.worker_id = self.worker_id
            response.address = f"/model_worker_{self.worker_id}/request"  # 請求 topic
            response.model_names = [self.model_name]
            response.message = f"Worker address found for model {self.model_name}"
        else:
            response.success = False
            response.worker_id = ""
            response.address = ""
            response.model_names = []
            response.message = f"Model {request.model} not supported by this worker"
        
        return response
    
    def generate_callback(self, msg: GenerateRequest):
        """處理生成請求"""
        self.global_counter += 1
        
        # 獲取信號量
        self.model_semaphore.acquire()
        
        try:
            # 在後台執行生成
            thread = Thread(
                target=self._generate_stream_thread,
                args=(msg,),
                daemon=True
            )
            thread.start()
            
        except Exception as e:
            self.get_logger().error(f'Error starting generation thread: {e}')
            self.model_semaphore.release()
    
    def _generate_stream_thread(self, request: GenerateRequest):
        """在獨立執行緒中執行生成"""
        try:
            for response in self._generate_stream(request):
                self.response_pub.publish(response)
        except Exception as e:
            self.get_logger().error(f'Generation error: {e}')
            # 發送錯誤回應
            error_response = GenerateResponse()
            error_response.request_id = request.request_id
            error_response.text = server_error_msg
            error_response.error_code = 1
            error_response.is_final = True
            self.response_pub.publish(error_response)
        finally:
            self.model_semaphore.release()
    
    @torch.inference_mode()
    def _generate_stream(self, request: GenerateRequest):
        """生成回應流"""
        tokenizer, model, image_processor = self.tokenizer, self.model, self.image_processor
        
        prompt = request.prompt
        ori_prompt = prompt
        images = None
        num_image_tokens = 0
        
        # 處理圖像
        if request.images:
            if len(request.images) != prompt.count(DEFAULT_IMAGE_TOKEN):
                raise ValueError("Number of images does not match number of <image> tokens in prompt")
            
            # 從 ROS Image 訊息轉換為 PIL 圖像
            pil_images = []
            for img_msg in request.images:
                cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, "rgb8")
                pil_image = PILImage.fromarray(cv_image)
                pil_images.append(pil_image)
            
            image_sizes = [img.size for img in pil_images]
            images = process_images(pil_images, image_processor, model.config)
            
            if isinstance(images, list):
                images = [img.to(self.device, dtype=torch.float16) for img in images]
            else:
                images = images.to(self.device, dtype=torch.float16)
            
            replace_token = DEFAULT_IMAGE_TOKEN
            if getattr(model.config, 'mm_use_im_start_end', False):
                replace_token = DEFAULT_IM_START_TOKEN + replace_token + DEFAULT_IM_END_TOKEN
            prompt = prompt.replace(DEFAULT_IMAGE_TOKEN, replace_token)
            
            num_image_tokens = prompt.count(replace_token) * model.get_vision_tower().num_patches
            image_args = {"images": images, "image_sizes": image_sizes}
        else:
            image_args = {}
        
        # 生成參數
        temperature = max(float(request.temperature), 0.001) if request.temperature > 0 else 1.0
        top_p = float(request.top_p) if request.top_p > 0 else 1.0
        max_new_tokens = min(int(request.max_new_tokens), 1024) if request.max_new_tokens > 0 else 256
        stop_str = request.stop if request.stop else None
        do_sample = temperature > 0.001
        
        # Token 化
        input_ids = tokenizer_image_token(
            prompt, tokenizer, IMAGE_TOKEN_INDEX, return_tensors='pt'
        ).unsqueeze(0).to(self.device)
        
        # 設定串流器
        streamer = TextIteratorStreamer(
            tokenizer, skip_prompt=True, skip_special_tokens=True, timeout=15
        )
        
        max_context_length = getattr(model.config, 'max_position_embeddings', 2048)
        max_new_tokens = min(max_new_tokens, max_context_length - input_ids.shape[-1] - num_image_tokens)
        
        if max_new_tokens < 1:
            response = GenerateResponse()
            response.request_id = request.request_id
            response.text = ori_prompt + "Exceeds max token length. Please start a new conversation, thanks."
            response.error_code = 0
            response.is_final = True
            yield response
            return
        
        # 啟動生成執行緒
        generation_thread = Thread(
            target=model.generate,
            kwargs=dict(
                inputs=input_ids,
                do_sample=do_sample,
                temperature=temperature,
                top_p=top_p,
                max_new_tokens=max_new_tokens,
                streamer=streamer,
                use_cache=True,
                **image_args
            )
        )
        generation_thread.start()
        
        # 串流生成結果
        generated_text = ori_prompt
        for new_text in streamer:
            generated_text += new_text
            if stop_str and generated_text.endswith(stop_str):
                generated_text = generated_text[:-len(stop_str)]
            
            response = GenerateResponse()
            response.request_id = request.request_id
            response.text = generated_text
            response.error_code = 0
            response.is_final = False
            yield response
        
        # 發送最終回應
        final_response = GenerateResponse()
        final_response.request_id = request.request_id
        final_response.text = generated_text
        final_response.error_code = 0
        final_response.is_final = True
        yield final_response


def main(args=None):
    rclpy.init(args=args)
    
    worker = ROS2ModelWorker()
    
    # 使用多執行緒執行器
    executor = MultiThreadedExecutor()
    executor.add_node(worker)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        worker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()