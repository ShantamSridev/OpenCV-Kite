import cv2
import numpy as np
import requests
import serial
import threading
import math

# ————— CONFIG —————
ESP32_URL     = "http://192.168.4.1"
VIDEO_BACKEND = cv2.CAP_FFMPEG    # or cv2.CAP_GSTREAMER
SIDEBAR_FRAC  = 1/3               # sidebar width fraction
INCL_H_FRAC   = 0.4               # inclinometer height fraction
SERIAL_PORT   = "COM6"           # or "/dev/ttyUSB0"
BAUDRATE      = 115200

# ——— shared state ———
telemetry = {"Pitch":0.0, "Roll":0.0, "Yaw":0.0,
             "Temp":0.0, "Alt":0.0, "Battery":0.0}
lock = threading.Lock()

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

# ——— draw inclinometer in-place ———
def draw_inclinometer(roll_deg, pitch_deg, W, H):
    img = np.zeros((H, W, 3), dtype=np.uint8)
    cx, cy = W // 2, H // 2
    radius = min(cx, cy) - 5

    # horizon offset and tilt
    y_off = int((pitch_deg / 90.0) * radius)
    theta = math.radians(-roll_deg)
    c, s = math.cos(theta), math.sin(theta)

    # coordinate grid
    ys, xs = np.indices((H, W))
    dx = xs - cx
    dy = ys - cy

    # rotated grid for horizon test
    xr = dx * c + (dy + y_off) * s
    yr = -dx * s + (dy + y_off) * c

    # mask inside circle
    circle_mask = (dx**2 + dy**2) <= radius**2

    # sky/ground masks
    top_mask  = (yr < 0) & circle_mask
    bot_mask  = (yr >= 0) & circle_mask
    img[top_mask] = (255, 0, 0)
    img[bot_mask] = (0, 0, 255)

    # draw fixed circle & crosshair
    cv2.circle(img, (cx, cy), radius, (255,255,255), 2)
    cv2.line(img, (cx-radius, cy), (cx+radius, cy), (255,255,255), 1)
    cv2.line(img, (cx, cy-radius), (cx, cy+radius), (255,255,255), 1)

    # labels
    cv2.putText(img, f"Roll: {roll_deg:.1f}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
    cv2.putText(img, f"Pitch: {pitch_deg:.1f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

    return img

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
    cv2.destroyAllWindows()
