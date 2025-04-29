import cv2
import numpy as np
import requests
import serial
import threading
import math
import time
import csv
from datetime import datetime
from inclinometer import draw_inclinometer

# ————— CONFIG —————
ESP32_URL     = "http://192.168.4.1"
VIDEO_BACKEND = cv2.CAP_FFMPEG    # or cv2.CAP_GSTREAMER
SIDEBAR_FRAC  = 1/3               # sidebar width fraction
INCL_H_FRAC   = 0.4               # inclinometer height fraction
SERIAL_PORT   = "COM6"            # or "/dev/ttyUSB0"
BAUDRATE      = 115200
LOG_INTERVAL  = 0.5               # seconds between logs

# ——— shared state ———
telemetry = {"Pitch":0.0, "Roll":0.0, "Yaw":0.0,
             "Temp":0.0, "Alt":0.0, "Battery":0.0}
lock = threading.Lock()

# ——— logging setup ———
start_time = time.time()
timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
logfile = open(f"logs/log_{timestamp_str}.csv", "w", newline="")
csv_writer = csv.writer(logfile)
csv_writer.writerow(["Timestamp", "Runtime (s)", "Pitch", "Roll", "Yaw", "Temp", "Alt", "Battery"])
last_log_time = start_time

# ——— serial reader thread ———
def serial_reader():
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
    buf = ""
    while True:
        data = ser.read(ser.in_waiting or 1).decode(errors='ignore')
        if not data:
            continue
        buf += data
        if "\n" in buf:
            line, buf = buf.split("\n", 1)
            parts = [p.strip() for p in line.split(",")]
            with lock:
                for p in parts:
                    if ":" in p:
                        k,v = p.split(":",1)
                        try:
                            telemetry[k.strip()] = float(v)
                        except:
                            pass

t = threading.Thread(target=serial_reader, daemon=True)
t.start()

# ——— camera controls ———
def set_resolution(idx):
    if idx in [10,9,8,7,6,5,4,3,0]:
        requests.get(f"{ESP32_URL}/control?var=framesize&val={idx}")

def set_awb(awb):
    try:
        awb = not awb
        requests.get(f"{ESP32_URL}/control?var=awb&val={1 if awb else 0}")
    except:
        pass
    return awb

# ——— main loop ———
if __name__ == "__main__":
    cap = cv2.VideoCapture(f"{ESP32_URL}:81/stream", VIDEO_BACKEND)
    if not cap.isOpened():
        raise RuntimeError("Cannot open video stream!")
    set_resolution(10)
    awb = True

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame grab failed")
            break

        H, W = frame.shape[:2]
        sb_w = int(W * SIDEBAR_FRAC)
        inc_h = int(H * INCL_H_FRAC)

        # sidebar
        sidebar = np.zeros((H, sb_w, 3), dtype=np.uint8)
        with lock:
            items = list(telemetry.items())
        y0, dy = 30, 30
        for i,(k,v) in enumerate(items):
            txt = f"{k}: {v:.2f}"
            cv2.putText(sidebar, txt, (10, y0 + i*dy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        # inclinometer
        with lock:
            roll = telemetry["Roll"]
            pitch = telemetry["Pitch"]
        inc_img = draw_inclinometer(roll, pitch, sb_w, inc_h)

        # compose
        top = np.hstack((frame, sidebar))
        blank = np.zeros((inc_h, W, 3), dtype=np.uint8)
        bot = np.hstack((blank, inc_img))
        comp = np.vstack((top, bot))

        # ——— telemetry logging every 0.5s ———
        now_time = time.time()
        if now_time - last_log_time >= LOG_INTERVAL:
            with lock:
                now = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                runtime = now_time - start_time
                csv_writer.writerow([
                    now,
                    f"{runtime:.2f}",
                    telemetry["Pitch"],
                    telemetry["Roll"],
                    telemetry["Yaw"],
                    telemetry["Temp"],
                    telemetry["Alt"],
                    telemetry["Battery"]
                ])
                logfile.flush()
            last_log_time = now_time

        # show
        cv2.imshow("Camera + Telemetry + Inclinometer", comp)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('r'):
            idx = int(input("Resolution index? "))
            set_resolution(idx)
        elif key == ord('a'):
            awb = set_awb(awb)
        elif key in (ord('q'), 27):
            break

    cap.release()
    logfile.close()
    cv2.destroyAllWindows()
