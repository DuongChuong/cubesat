import sys
import time
import requests
import cv2
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QGridLayout, QLabel, QLineEdit, 
                             QPushButton, QFrame)
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtGui import QImage, QPixmap, QFont
import pyqtgraph as pg

# CONFIG INTERFACE
DARK_THEME = """
QMainWindow {
    background-color: #050510;
    background-image: radial-gradient(circle at top left, #281d4a 0%, #050510 45%);
}
QLabel {
    color: #f7f7f7;
    font-family: "Segoe UI", "Orbitron", sans-serif;
}
QFrame {
    background-color: rgba(15, 23, 42, 0.9);
    border: 1px solid rgba(148, 163, 255, 0.35);
    border-radius: 10px;
}
QLineEdit {
    background-color: rgba(15, 15, 30, 0.9);
    border: 1px solid rgba(148, 163, 184, 0.4);
    color: #e5e7eb;
    border-radius: 15px;
    padding: 5px 10px;
}
QPushButton {
    background-color: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #8b5cf6, stop:1 #ec4899);
    color: white;
    border-radius: 15px;
    padding: 6px 15px;
    font-weight: bold;
    border: none;
}
QPushButton:hover {
    background-color: #d946ef;
}
"""

# SNAPSHOT 5s
class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    
    def __init__(self, ip, snapshot_interval=5):
        super().__init__()
        self.ip = ip
        self.interval = snapshot_interval
        self.running = True

    def run(self):
        # Clean URL
        base_url = self.ip.strip()
        if not base_url.startswith("http://"):
            base_url = f"http://{base_url}"
        
        capture_url = f"{base_url}/capture"
        
        print(f"Camera mode: Snapshot every {self.interval}s")
        print(f"Target: {capture_url}")
        
        while self.running:
            try:
                resp = requests.get(capture_url, timeout=4)
                if resp.status_code == 200:
                    image_array = np.asarray(bytearray(resp.content), dtype=np.uint8)
                    frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        self.change_pixmap_signal.emit(frame)
                    else:
                        print("Error: Could not decode image")
                else:
                    print(f"Error: Server returned {resp.status_code}")
            except Exception as e:
                print(f"Snapshot error: {e}")
            
            for _ in range(self.interval * 10):
                if not self.running: 
                    break
                time.sleep(0.1)

# Process dataSensor
class SensorThread(QThread):
    data_signal = pyqtSignal(dict)

    def __init__(self, ip):
        super().__init__()
        self.ip = ip
        self.running = True

    def run(self):
        # Clean URL
        base_url = self.ip.strip()
        if not base_url.startswith("http://"):
            base_url = f"http://{base_url}"
        if base_url.endswith('/'):
            base_url = base_url[:-1]
        
        data_url = f"{base_url}/data"
        
        while self.running:
            try:
                resp = requests.get(data_url, timeout=2)
                if resp.status_code == 200:
                    data = resp.json()
                    self.data_signal.emit(data)
                else:
                    print(f"Sensor data error: {resp.status_code}")
            except Exception as e:
                print(f"Sensor error: {e}")
            time.sleep(0.5) 

# MAIN INTERFACE
class CubeSatDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HEXSTAR DASHBOARD - SNAPSHOT MODE")
        self.resize(1200, 800)
        self.setStyleSheet(DARK_THEME)

        self.current_ip = "10.11.202.234" 

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.main_layout = QVBoxLayout(central_widget)

        self.setup_header()
        self.grid_layout = QGridLayout()
        self.main_layout.addLayout(self.grid_layout)

        self.setup_temp_chart()
        self.setup_gyro_chart()
        self.setup_telemetry_table()
        self.setup_accel_chart()
        self.setup_camera_feed()
        self.setup_digital_temp()

        self.x_data = list(range(50))
        self.temp_data = [0]*50
        self.acc_data = {'x': [0]*50, 'y': [0]*50, 'z': [0]*50}
        self.gyro_data = {'x': [0]*50, 'y': [0]*50, 'z': [0]*50}

        self.video_thread = None
        self.sensor_thread = None

    def setup_header(self):
        header = QHBoxLayout()
        title = QLabel("HEXSTAR DASHBOARD")
        title.setFont(QFont("Orbitron", 16, QFont.Weight.Bold))
        title.setStyleSheet("color: #7d5fff; letter-spacing: 2px;")
        
        self.ip_input = QLineEdit(self.current_ip)
        self.ip_input.setPlaceholderText("ESP32 IP or Proxy")
        self.ip_input.setFixedWidth(200)
        
        btn_connect = QPushButton("CONNECT")
        btn_connect.clicked.connect(self.start_threads)

        header.addWidget(title)
        header.addStretch()
        header.addWidget(QLabel("Target IP:"))
        header.addWidget(self.ip_input)
        header.addWidget(btn_connect)
        self.main_layout.addLayout(header)

    def create_card(self, title):
        frame = QFrame()
        layout = QVBoxLayout(frame)
        lbl_title = QLabel(title)
        lbl_title.setStyleSheet("font-weight: bold; color: #a5b4fc; margin-bottom: 5px;")
        lbl_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(lbl_title)
        return frame, layout

    def setup_temp_chart(self):
        frame, layout = self.create_card("TEMPERATURE (°C)")
        self.temp_plot = pg.PlotWidget()
        self.temp_plot.setBackground('k')
        self.temp_plot.showGrid(x=True, y=True, alpha=0.3)
        self.temp_curve = self.temp_plot.plot(pen=pg.mkPen('#ff4b4b', width=2))
        layout.addWidget(self.temp_plot)
        self.grid_layout.addWidget(frame, 0, 0)

    def setup_gyro_chart(self):
        frame, layout = self.create_card("GYROSCOPE (rad/s)")
        self.gyro_plot = pg.PlotWidget()
        self.gyro_plot.setBackground('k')
        self.gyro_plot.showGrid(x=True, y=True, alpha=0.3)
        self.gyro_curves = [
            self.gyro_plot.plot(pen=pg.mkPen('#ff4b4b', width=2)),
            self.gyro_plot.plot(pen=pg.mkPen('#15d15d', width=2)),
            self.gyro_plot.plot(pen=pg.mkPen('#3ca1ff', width=2))
        ]
        layout.addWidget(self.gyro_plot)
        self.grid_layout.addWidget(frame, 0, 1)

    def setup_telemetry_table(self):
        frame, layout = self.create_card("DATA TELEMETRY")
        self.lbl_tele_temp = QLabel("Temp: -- °C")
        self.lbl_tele_acc = QLabel("Acc: --")
        self.lbl_tele_gyro = QLabel("Gyro: --")
        font = QFont("Consolas", 10)
        for l in [self.lbl_tele_temp, self.lbl_tele_acc, self.lbl_tele_gyro]:
            l.setFont(font)
            layout.addWidget(l)
        layout.addStretch()
        self.grid_layout.addWidget(frame, 0, 2)

    def setup_accel_chart(self):
        frame, layout = self.create_card("ACCELERATION (m/s²)")
        self.acc_plot = pg.PlotWidget()
        self.acc_plot.setBackground('k')
        self.acc_plot.showGrid(x=True, y=True, alpha=0.3)
        self.acc_curves = [
            self.acc_plot.plot(pen=pg.mkPen('#ff4b4b', width=2)),
            self.acc_plot.plot(pen=pg.mkPen('#15d15d', width=2)),
            self.acc_plot.plot(pen=pg.mkPen('#3ca1ff', width=2))
        ]
        layout.addWidget(self.acc_plot)
        self.grid_layout.addWidget(frame, 1, 0)

    def setup_camera_feed(self):
        frame, layout = self.create_card("SNAPSHOT (Every 5s)")
        self.video_label = QLabel("Waiting for snapshot...")
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setStyleSheet("background-color: #000; border: 1px solid #333;")
        self.video_label.setScaledContents(True)
        layout.addWidget(self.video_label)
        self.grid_layout.addWidget(frame, 1, 1)

    def setup_digital_temp(self):
        frame, layout = self.create_card("CORE TEMP DIGIT")
        self.lbl_big_temp = QLabel("--.-")
        self.lbl_big_temp.setFont(QFont("Orbitron", 40, QFont.Weight.Bold))
        self.lbl_big_temp.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_big_temp.setStyleSheet("color: #22c55e;")
        layout.addWidget(self.lbl_big_temp)
        layout.addStretch()
        self.grid_layout.addWidget(frame, 1, 2)

    def start_threads(self):
        ip = self.ip_input.text().strip()

        
        if self.video_thread:
            self.video_thread.stop()
        if self.sensor_thread: 
            self.sensor_thread.stop()

        self.video_thread = VideoThread(ip)
        self.video_thread.change_pixmap_signal.connect(self.update_video_image)
        self.video_thread.start()

        self.sensor_thread = SensorThread(ip)
        self.sensor_thread.data_signal.connect(self.update_sensor_data)
        self.sensor_thread.start()

    def update_video_image(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        p = convert_to_Qt_format.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio)
        self.video_label.setPixmap(QPixmap.fromImage(p))

    def update_sensor_data(self, data):
        temp = data.get('temp', 0)
        self.temp_data.pop(0)
        self.temp_data.append(temp)
        self.temp_curve.setData(self.temp_data)
        self.lbl_big_temp.setText(f"{temp:.1f}")
        self.lbl_tele_temp.setText(f"Temp: {temp:.2f} °C")

        gx, gy, gz = data.get('gx', 0), data.get('gy', 0), data.get('gz', 0)
        for lst, val, curve in zip([self.gyro_data['x'], self.gyro_data['y'], self.gyro_data['z']], 
                                   [gx, gy, gz], self.gyro_curves):
            lst.pop(0)
            lst.append(val)
            curve.setData(lst)
        self.lbl_tele_gyro.setText(f"Gyro: {gx:.2f}, {gy:.2f}, {gz:.2f}")

        ax, ay, az = data.get('ax', 0), data.get('ay', 0), data.get('az', 0)
        for lst, val, curve in zip([self.acc_data['x'], self.acc_data['y'], self.acc_data['z']], 
                                   [ax, ay, az], self.acc_curves):
            lst.pop(0)
            lst.append(val)
            curve.setData(lst)
        self.lbl_tele_acc.setText(f"Acc: {ax:.2f}, {ay:.2f}, {az:.2f}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CubeSatDashboard()
    window.show()
    sys.exit(app.exec())