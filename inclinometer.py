import cv2
import numpy as np
import math
import serial
import threading

# ---------------- CONFIGURATION ----------------
SERIAL_PORT = 'COM6'     # adjust to your port, e.g. '/dev/ttyUSB1'
BAUDRATE    = 115200     # match your sensor's baud rate
REFRESH_RATE_MS = 10     # window update delay (ms)

# Shared roll/pitch values
_angles = {'roll': 0.0, 'pitch': 0.0}
_lock = threading.Lock()

# -------------- SERIAL READER ----------------
def _serial_reader():
    """
    Background thread: reads lines like 'Pitch: -45.11, Roll: -20.45' from serial
    and updates internal _angles dict.
    """
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
    buffer = ''
    while True:
        chunk = ser.read(ser.in_waiting or 1).decode(errors='ignore')
        if not chunk:
            continue
        buffer += chunk
        if '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            parts = [p.strip() for p in line.split(',')]
            vals = {}
            for p in parts:
                if ':' in p:
                    k, v = p.split(':', 1)
                    try:
                        vals[k.lower()] = float(v)
                    except ValueError:
                        pass
            with _lock:
                if 'roll' in vals and 'pitch' in vals:
                    _angles['roll'] = vals['roll']
                    _angles['pitch'] = vals['pitch']

# Start reader thread
_thread = threading.Thread(target=_serial_reader, daemon=True)
_thread.start()

# -------------- PUBLIC API ----------------
def get_angles():
    """
    Returns the latest (roll_deg, pitch_deg) tuple.
    """
    with _lock:
        return _angles['roll'], _angles['pitch']


def draw_inclinometer(roll_deg, pitch_deg, width, height):
    """
    Returns a BGR OpenCV image (height x width) of an attitude indicator:
    - red = sky
    - blue = ground
    - crosshairs + numeric labels
    """
    img = np.zeros((height, width, 3), dtype=np.uint8)
    cx, cy = width // 2, height // 2
    radius = min(cx, cy) - 5

    # vertical horizon shift by pitch
    y_offset = int((pitch_deg / 90.0) * radius)
    theta = math.radians(-roll_deg)
    c, s = math.cos(theta), math.sin(theta)

    ys, xs = np.indices((height, width))
    dx = xs - cx
    dy = ys - (cy + y_offset)

    # rotate coords
    xr = dx * c + dy * s
    yr = -dx * s + dy * c

    dist = np.hypot(dx, dy)
    top_mask = (dist <= radius) & (yr < 0)
    bot_mask = (dist <= radius) & (yr >= 0)

    img[top_mask] = (255, 0, 0)
    img[bot_mask] = (0, 0, 255)

    # circle + crosshairs
    cv2.circle(img, (cx, cy), radius, (255, 255, 255), 2)
    cv2.line(img, (cx - radius, cy), (cx + radius, cy), (255, 255, 255), 1)
    cv2.line(img, (cx, cy - radius), (cx, cy + radius), (255, 255, 255), 1)

    # labels
    for i, text in enumerate([f"Roll: {roll_deg:.1f}", f"Pitch: {pitch_deg:.1f}"]):
        cv2.putText(img, text, (10, 25 + i * 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2, cv2.LINE_AA)

    return img
