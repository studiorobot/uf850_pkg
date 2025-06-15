
from PyQt5.QtCore import QThread
import time

class RosWorker(QThread):
    def __init__(self, log_widget):
        super().__init__()
        self.log_widget = log_widget

    def log(self, msg):
        self.log_widget.append(msg)

    def run(self):
        self.log("Simulating mount...")
        time.sleep(1)
        self.log("Simulating ROS node launch...")
        time.sleep(1)
        self.log("ROS Nodes Launched (mock)")
