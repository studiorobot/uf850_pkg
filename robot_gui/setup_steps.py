
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton
from main_screen import MainScreen

class SetupWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Arm Setup")
        self.resize(600, 300)

        self.instructions = [
            "Step 1: Power on the robot arm.",
            "Step 2: Connect the robot to the PC via Ethernet.",
            "Step 3: Ensure shared drive is mounted.",
            "Step 4: Turn on camera feed system.",
        ]
        self.current = 0

        self.layout = QVBoxLayout()
        self.label = QLabel(self.instructions[self.current])
        self.label.setStyleSheet("font-size: 18px;")
        self.next_btn = QPushButton("Next")
        self.next_btn.clicked.connect(self.next_step)

        self.layout.addWidget(self.label)
        self.layout.addWidget(self.next_btn)
        self.setLayout(self.layout)

    def next_step(self):
        self.current += 1
        if self.current < len(self.instructions):
            self.label.setText(self.instructions[self.current])
        else:
            self.close()
            self.main_screen = MainScreen()
            self.main_screen.show()
