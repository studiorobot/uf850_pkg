from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout, QHBoxLayout, QTextEdit, QFrame
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
from camera_feed import CameraFeedWidget
from worker_threads import RosWorker
from voice_visualizer import VoiceBarVisualizer
import os
from PyQt5.QtGui import QFont

class MainScreen(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Studio Control Panel")
        self.resize(1200, 800)
        self.setStyleSheet("background-color: white;")

        # --- Camera + LIVE VIEW + PURPLE ACTION BAR in one frame ---
        cam_and_status = QFrame()
        cam_and_status_layout = QVBoxLayout(cam_and_status)
        cam_and_status_layout.setContentsMargins(0, 0, 0, 0)
        cam_and_status_layout.setSpacing(0)

        # Camera feed
        self.cam_widget = CameraFeedWidget()
        self.cam_widget.setFixedHeight(440)

        # LIVE VIEW tag
        cam_tag = QLabel("LIVE VIEW", self.cam_widget)
        cam_tag.setStyleSheet("background-color: black; color: white; font-size: 13px; padding: 2px 8px;")
        cam_tag.move(0, 0)
        cam_tag.setAttribute(Qt.WA_TransparentForMouseEvents)

        # Action bar
        self.status_label = QLabel("CURRENT ACTION: PAINTING A CIRCLE")
        self.status_label.setStyleSheet(
            "background-color: #7A5FFF; color: white; padding: 4px; font-size: 16px;"
        )
        self.status_label.setFixedHeight(35)

        # Add both to same frame
        cam_and_status_layout.addWidget(self.cam_widget)
        cam_and_status_layout.addWidget(self.status_label)

        # --- Voice Visualizer ---
        self.voice_widget = VoiceBarVisualizer()

        voice_info = QLabel()
        voice_info.setStyleSheet("""
            background-color: #eeeeee;
            font-size: 13px;
            padding: 6px;
        """)
        voice_info.setText(
            "Preface your command with <span style='color:#7A5FFF;'>ROBOT</span> to activate.<br>"
            "Say <span style='color:#ff6666;'>STOP</span> to terminate the robot movement."
        )
        voice_info.setTextFormat(Qt.RichText)

        # --- Left Column Layout ---
        left_layout = QVBoxLayout()
        left_layout.setSpacing(8)
        left_layout.setContentsMargins(20, 10, 10, 10)
        left_layout.addWidget(cam_and_status)
        left_layout.addWidget(self.voice_widget)
        left_layout.addWidget(voice_info)

        # --- Log with overlay LOG tag ---
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setStyleSheet("background-color: black; color: lime; font-family: monospace; font-size: 14px;")

        log_container = QFrame()
        log_layout = QVBoxLayout(log_container)
        log_layout.setContentsMargins(0, 0, 0, 0)
        log_layout.setSpacing(0)
        log_layout.addWidget(self.log_box)

        log_tag = QLabel("LOG", self.log_box)
        log_tag.setStyleSheet("background-color: black; color: white; font-size: 13px; padding: 2px 8px;")
        log_tag.setFont(QFont("Norwester"))  # <--- Force Norwester
        log_tag.move(0, 0)
        log_tag.setAttribute(Qt.WA_TransparentForMouseEvents)

        # --- Overall Layout ---
        main_layout = QHBoxLayout()
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.addLayout(left_layout, 3)
        main_layout.addWidget(log_container, 2)

        self.setLayout(main_layout)

        self.worker = RosWorker(self.log_box)
        self.worker.start()
