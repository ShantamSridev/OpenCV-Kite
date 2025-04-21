import cv2
import numpy as np
import requests
import serial
import re
import time

# --- CONFIG ---
URL       = "http://192.168.4.1"
STREAM    = URL + ":81/stream"
SERIAL_PORT = 'COM6'         # change to your port, e.g. '/dev/ttyUSB0'
BAUDRATE  = 115200
TIMEOUT   = 0.1              # seconds
# --------------

# Open video stream
cap = cv2.VideoCapture(STREAM)
if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

# Open serial port
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
    time.sleep(2)  # give it a moment
except Exception as e:
    print(f"Error opening serial port: {e}")
    ser = None

# Simple regex to grab numbers
line_re = re.compile(
    r'Pitch:\s*([-+]?\d*\.?\d+),\s*'
    r'Roll:\s*([-+]?\d*\.?\d+),\s*'
    r'Yaw:\s*([-+]?\d*\.?\d+),\s*'
    r'Temp:\s*([-+]?\d*\.?\d+),\s*'
    r'Alt:\s*([-+]?\d*\.?\d+),\s*'
    r'Battery:\s*([-+]?\d*\.?\d+)'
)

def read_hud():
    """Read one line from serial and parse into tuple of floats.
       Returns None on failure."""
    if ser is None:
        return None
    raw = ser.readline().decode(errors='ignore').strip()
    m = line_re.match(raw)
    if not m:
        return None
    return tuple(float(m.group(i)) for i in range(1,7))

def draw_hud(frame, data):
    """Overlay the HUD data onto the frame."""
    if data is None:
        return frame
    pitch, roll, yaw, temp, alt, batt = data

    # Prepare text lines
    lines = [
        f"Pitch:   {pitch:6.2f}",
        f"Roll:    {roll:6.2f}",
        f"Yaw:     {yaw:6.2f}",
        f"Temp:    {temp:6.2f} °C",
        f"Alt:     {alt:6.2f} m",
        f"Battery: {batt:4.2f} V",
    ]

    # Parameters
    margin  = 10
    line_h  = 20  # approx text height
    font    = cv2.FONT_HERSHEY_SIMPLEX
    scale   = 0.6
    color   = (0, 255, 0)
    thickness = 1
    alpha   = 0.4  # transparency for background

    # Calculate background rectangle size
    box_width  = 200
    box_height = line_h * len(lines) + margin*2
    overlay = frame.copy()
    cv2.rectangle(
        overlay,
        (margin, margin),
        (margin + box_width, margin + box_height),
        (0,0,0),
        -1
    )
    # blend with original
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

    # Draw each line of text
    for i, text in enumerate(lines):
        y = margin + (i+1)*line_h
        cv2.putText(
            frame,
            text,
            (margin + 5, y),
            font,
            scale,
            color,
            thickness,
            cv2.LINE_AA
        )
    return frame

if __name__ == '__main__':
    # Optionally set resolution once
    # requests.get(URL + "/control?var=framesize&val=10")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        # Read and overlay HUD data
        hud_data = read_hud()
        frame = draw_hud(frame, hud_data)

        cv2.imshow('IP Camera Stream with HUD', frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        # (you can re‑use your existing 'r' and 'a' handlers here)

    cap.release()
    if ser:
        ser.close()
    cv2.destroyAllWindows()
