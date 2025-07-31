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
        
        # Publisher
        self.query_pub = self.create_publisher(SpatialAffordanceQuery, 'spatial_affordance_query', 10)

        # Subscriber to original and visualized image
        self.create_subscription(Image, '/camera/image_raw', self.update_raw_image, 10)
        self.create_subscription(Image, 'affordance_visualization', self.update_vis_image, 10)

        # GUI
        self.app = QtWidgets.QApplication(sys.argv)
        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle("RoboPoint GUI")
        self.layout = QtWidgets.QHBoxLayout(self.win)

        # Left: Command area
        self.text_input = QtWidgets.QTextEdit()
        self.send_button = QtWidgets.QPushButton("Send")
        self.send_button.clicked.connect(self.send_command)
        left_layout = QtWidgets.QVBoxLayout()
        left_layout.addWidget(self.text_input)
        left_layout.addWidget(self.send_button)

        # Right: Image displays
        self.raw_image_label = QtWidgets.QLabel("Camera Image")
        self.vis_image_label = QtWidgets.QLabel("Affordance Result")
        self.raw_image_label.setFixedSize(400, 300)
        self.vis_image_label.setFixedSize(400, 300)
        right_layout = QtWidgets.QVBoxLayout()
        right_layout.addWidget(self.raw_image_label)
        right_layout.addWidget(self.vis_image_label)

        self.layout.addLayout(left_layout)
        self.layout.addLayout(right_layout)
        self.win.show()

        # Timer for Qt GUI loop
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.app.processEvents)
        self.timer.start(50)

    def update_raw_image(self, msg):
        self.get_logger().info("Received raw image")
        self.display_image(msg, self.raw_image_label)

    def update_vis_image(self, msg):
        self.display_image(msg, self.vis_image_label)

    def display_image(self, msg, label):
        try:
            self.get_logger().info("Converting image msg...")
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info(f"Converted image shape: {cv_image.shape}")
            
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
            pixmap = QtGui.QPixmap.fromImage(qt_image).scaled(label.width(), label.height(), QtCore.Qt.KeepAspectRatio)
            label.setPixmap(pixmap)
            label.repaint()
        except Exception as e:
            self.get_logger().error(f"Image display error: {e}")

    def send_command(self):
        text = self.text_input.toPlainText().strip()
        if text:
            msg = SpatialAffordanceQuery()
            msg.header = Header()
            msg.query_text = text
            msg.image_process_mode = "default"
            self.query_pub.publish(msg)
            self.get_logger().info(f"Sent query: {text}")
            self.text_input.clear()

def main(args=None):
    rclpy.init(args=args)
    gui = RoboPointGUI()

    # 替代 while spin，改用 Qt event loop，並搭配 rclpy executor timer
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(gui)

    # 用 QTimer 每 10ms 呼叫一次 rclpy.spin_once
    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: executor.spin_once(timeout_sec=0.01))
    timer.start(10)

    # 進入 PyQt 主 event loop（不要再用 while spin）
    sys.exit(gui.app.exec_())

if __name__ == '__main__':
    main()
