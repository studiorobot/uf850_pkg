
import numpy as np
import sounddevice as sd
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QColor, QBrush
from PyQt5.QtCore import QTimer

class VoiceBarVisualizer(QWidget):
    def __init__(self, bar_count=32):
        super().__init__()
        self.bar_count = bar_count
        self.bar_values = np.zeros(bar_count)
        self.setMinimumHeight(80)

        self.stream = sd.InputStream(callback=self.audio_callback)
        self.stream.start()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(30)

    def audio_callback(self, indata, frames, time, status):
        volume_norm = np.linalg.norm(indata) * 10
        values = np.random.rand(self.bar_count) * volume_norm / 20
        self.bar_values = 0.6 * self.bar_values + 0.4 * values

    def paintEvent(self, event):
        painter = QPainter(self)
        w = self.width()
        h = self.height()
        spacing = 4
        bar_width = w // self.bar_count - spacing

        for i in range(self.bar_count):
            bar_height = int(h * self.bar_values[i])
            x = i * (bar_width + spacing)
            y = h - bar_height
            painter.setBrush(QBrush(QColor(60, 60, 60)))
            painter.setPen(QColor(60, 60, 60))
            painter.drawRoundedRect(x, y, bar_width, bar_height, 5, 5)
