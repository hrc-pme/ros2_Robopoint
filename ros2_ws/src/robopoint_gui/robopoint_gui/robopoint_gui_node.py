#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from PyQt5 import QtWidgets, QtGui, QtCore
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from robopoint_interfaces.msg import SpatialAffordanceQuery
from cv_bridge import CvBridge
import numpy as np
import cv2

class RoboPointGUI(Node):
    def __init__(self):
        super().__init__('robopoint_gui')
        self.bridge = CvBridge()
        
        # Publishers
        self.query_pub = self.create_publisher(SpatialAffordanceQuery, 'spatial_affordance_query', 10)
        # 新增：把 GUI 收到的原始影像送到 external_camera/image_raw
        self.external_image_pub = self.create_publisher(Image, 'external_camera/image_raw', 10)

        # Subscribers
        self.create_subscription(Image, '/camera/image_raw', self.update_raw_image, 10)
        self.create_subscription(Image, 'affordance_visualization', self.update_vis_image, 10)

        # GUI Setup
        self.app = QtWidgets.QApplication(sys.argv)
        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle("RoboPoint GUI")
        self.layout = QtWidgets.QHBoxLayout(self.win)

        # Left: Command area
        self.text_input = QtWidgets.QTextEdit()
        self.text_input.setPlaceholderText("Enter your spatial affordance query here...")
        self.send_button = QtWidgets.QPushButton("Send Query")
        self.send_button.clicked.connect(self.send_command)
        
        # Status label
        self.status_label = QtWidgets.QLabel("Ready")
        self.status_label.setStyleSheet("color: green; font-weight: bold;")
        
        left_layout = QtWidgets.QVBoxLayout()
        left_layout.addWidget(QtWidgets.QLabel("Spatial Affordance Query:"))
        left_layout.addWidget(self.text_input)
        left_layout.addWidget(self.send_button)
        left_layout.addWidget(self.status_label)

        # Right: Image displays
        self.raw_image_label = QtWidgets.QLabel("Camera Image")
        self.vis_image_label = QtWidgets.QLabel("Affordance Result")
        self.raw_image_label.setFixedSize(400, 300)
        self.vis_image_label.setFixedSize(400, 300)
        self.raw_image_label.setStyleSheet("border: 1px solid gray;")
        self.vis_image_label.setStyleSheet("border: 1px solid gray;")
        self.raw_image_label.setAlignment(QtCore.Qt.AlignCenter)
        self.vis_image_label.setAlignment(QtCore.Qt.AlignCenter)
        
        right_layout = QtWidgets.QVBoxLayout()
        right_layout.addWidget(QtWidgets.QLabel("Raw Camera Feed:"))
        right_layout.addWidget(self.raw_image_label)
        right_layout.addWidget(QtWidgets.QLabel("Affordance Visualization:"))
        right_layout.addWidget(self.vis_image_label)

        self.layout.addLayout(left_layout)
        self.layout.addLayout(right_layout)
        self.win.show()

        # Timer for Qt GUI loop
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.app.processEvents)
        self.timer.start(50)

        self.get_logger().info("RoboPoint GUI initialized")

    def update_raw_image(self, msg):
        """接收原始影像，同時轉發給 RoboPointNode 並顯示在 GUI"""
        self.get_logger().info("Received raw image, republishing to external_camera/image_raw")
        
        # 轉發給 RoboPointNode
        self.external_image_pub.publish(msg)
        
        # 顯示在 GUI 上
        self.display_image(msg, self.raw_image_label)

    def update_vis_image(self, msg):
        """接收推理結果影像並顯示"""
        self.get_logger().info("Received affordance visualization")
        self.display_image(msg, self.vis_image_label)

    def display_image(self, msg, label):
        """將 ROS Image 訊息轉換並顯示在指定的 QLabel 上"""
        try:
            # 轉換 ROS Image 到 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().debug(f"Converted image shape: {cv_image.shape}")
            
            # 轉換色彩空間 BGR -> RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            
            # 建立 Qt 影像
            qt_image = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
            pixmap = QtGui.QPixmap.fromImage(qt_image).scaled(
                label.width(), label.height(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
            )
            
            # 顯示影像
            label.setPixmap(pixmap)
            label.repaint()
            
        except Exception as e:
            self.get_logger().error(f"Image display error: {e}")
            label.setText(f"Image Error: {str(e)}")

    def send_command(self):
        """發送空間推理查詢"""
        text = self.text_input.toPlainText().strip()
        if not text:
            self.status_label.setText("Please enter a query")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
            return
            
        try:
            # 建立查詢訊息
            msg = SpatialAffordanceQuery()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            msg.query_text = text
            # ←–– change this to one of the modes your conversation code actually supports
            # e.g. "base64" or "vision", not "default"
            msg.image_process_mode = "Pad"
            
            # 發送查詢
            self.query_pub.publish(msg)
            
            # 更新狀態
            self.get_logger().info(f"Sent spatial affordance query: {text}")
            self.status_label.setText(f"Query sent: {text[:30]}...")
            self.status_label.setStyleSheet("color: blue; font-weight: bold;")
            
            # 清空輸入框
            self.text_input.clear()
            
        except Exception as e:
            self.get_logger().error(f"Failed to send query: {e}")
            self.status_label.setText(f"Send failed: {str(e)}")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")

def main(args=None):
    rclpy.init(args=args)
    gui = RoboPointGUI()

    try:
        # 建立執行器
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(gui)

        # 用 QTimer 定期處理 ROS 訊息
        timer = QtCore.QTimer()
        timer.timeout.connect(lambda: executor.spin_once(timeout_sec=0.001))
        timer.start(10)  # 每 10ms 處理一次

        # 進入 PyQt 主事件迴圈
        sys.exit(gui.app.exec_())
        
    except KeyboardInterrupt:
        gui.get_logger().info("GUI shutdown requested")
    finally:
        gui.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()