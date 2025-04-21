import cv2
import numpy as np
import requests
import serial
import threading
import math

# ————— CONFIG —————
ESP32_URL    = "http://192.168.4.1"
VIDEO_BACKEND = cv2.CAP_FFMPEG   # try cv2.CAP_GSTREAMER if you’re on Linux
SIDEBAR_FRAC = 1/3               # sidebar width = 1/3 of frame
INCL_H_FRAC  = 0.4               # inclinometer height = 40% of frame height
SERIAL_PORT  = "COM6"            # or "/dev/ttyUSB0"
BAUDRATE     = 115200

# ——— shared state ———
telemetry = {
    "Pitch": 0.0,
    "Roll":  0.0,
    "Yaw":   0.0,
    "Temp":  0.0,
    "Alt":   0.0,
    "Battery": 0.0
}
lock = threading.Lock()

# ——— read the “Pitch: …, Roll: …” lines into telemetry{} ———
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

threading.Thread(target=serial_reader, daemon=True).start()


# ——— draw an “attitude indicator” (horizon) into a WxH image ———
def draw_inclinometer(roll_deg, pitch_deg, W, H):
    img = np.zeros((H, W, 3), dtype=np.uint8)
    cx, cy = W//2, H//2
    radius = min(cx, cy) - 5

    # pixel offset of horizon vertical position
    y_off = int((pitch_deg/90.0) * radius)
    θ = math.radians(-roll_deg)  # invert so positive roll → horizon tilts “right”

    # build coordinate grid
    ys, xs = np.indices((H, W))
    dx = xs - cx
    dy = ys - (cy + y_off)

    # rotate
    c, s = math.cos(θ), math.sin(θ)
    xr = dx*c + dy*s
    yr = -dx*s + dy*c

    dist = np.hypot(dx, dy)
    top_mask    = (dist <= radius) & (yr < 0)
    bottom_mask = (dist <= radius) & (yr >= 0)

    img[top_mask]    = (255,   0,   0)  # red top half
    img[bottom_mask] = (  0,   0, 255)  # blue bottom

    # circle & crosshair
    cv2.circle(img, (cx, cy), radius, (255,255,255), 2)
    cv2.line  (img, (cx-radius,cy), (cx+radius,cy), (255,255,255),1)
    cv2.line  (img, (cx,cy-radius), (cx,cy+radius), (255,255,255),1)
    return img


# ——— control funcs for the ESP32 camera ———
def set_resolution(idx):
    if idx in [10,9,8,7,6,5,4,3,0]:
        requests.get(f"{ESP32_URL}/control?var=framesize&val={idx}")

def set_awb(awb):
    try:
        awb = not awb
        requests.get(f"{ESP32_URL}/control?var=awb&val={1 if awb else 0}")
    except: pass
    return awb


# ——— main loop ———
if __name__ == "__main__":
    cap = cv2.VideoCapture(f"{ESP32_URL}:81/stream", VIDEO_BACKEND)
    if not cap.isOpened():
        raise RuntimeError("Cannot open video stream!")

    set_resolution(10)     # start at max res
    awb = True

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame grab failed")
            break

        H, W = frame.shape[:2]
        sb_w = int(W * SIDEBAR_FRAC)
        inc_h = int(H * INCL_H_FRAC)

        # — Telemetry sidebar (full frame height) — 
        sidebar = np.zeros((H, sb_w, 3), dtype=np.uint8)
        with lock:
            items = list(telemetry.items())
        y0, dy = 30, 30
        for i,(k,v) in enumerate(items):
            txt = f"{k}: {v:.2f}"
            cv2.putText(sidebar, txt, (10, y0 + i*dy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2, cv2.LINE_AA)

        # — Inclinometer panel ——
        # pull the latest roll/pitch
        with lock:
            roll  = telemetry["Roll"]
            pitch = telemetry["Pitch"]
        inc = draw_inclinometer(roll, pitch, sb_w, inc_h)

        # — Build the final composite ——
        # top row: [ camera | telemetry ]
        top = np.hstack((frame, sidebar))

        # bottom row: [ blank      | inclinometer ]
        blank = np.zeros((inc_h, W, 3), dtype=np.uint8)
        bot   = np.hstack((blank, inc))

        # stack vertically
        comp = np.vstack((top, bot))

        cv2.imshow("Camera + Telemetry + Inclinometer", comp)
        key = cv2.waitKey(1) & 0xFF

        if   key == ord('r'):
            idx = int(input("Resolution index? "))
            set_resolution(idx)
        elif key == ord('a'):
            awb = set_awb(awb)
        elif key in (ord('q'), 27):
            break

    cap.release()
    cv2.destroyAllWindows()
