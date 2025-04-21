import cv2
import numpy as np
import requests
import serial
import threading

# ESP32 URL
URL = "http://192.168.4.1"
AWB = True

# Serial port settings (adjust COM port & baudrate as needed)
SERIAL_PORT = "COM6"
BAUDRATE = 115200

# Open the video stream
cap = cv2.VideoCapture(URL + ":81/stream")
if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

# Open the serial port
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)

# Shared telemetry dict
telemetry = {
    "Pitch": 0,
    "Roll": 0,
    "Yaw": 0,
    "Temp": 0,
    "Alt": 0,
    "Battery": 0
}

def read_serial():
    """Continuously read & parse lines from serial port."""
    buffer = ""
    while True:
        chunk = ser.read(ser.in_waiting or 1).decode(errors='ignore')
        if not chunk:
            continue
        buffer += chunk
        if "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            # Expected format: Pitch: -45.11, Roll: -20.45, Yaw: -22.23, …
            parts = [p.strip() for p in line.split(",")]
            for p in parts:
                if ":" in p:
                    key, val = p.split(":", 1)
                    key = key.strip()
                    try:
                        telemetry[key] = float(val)
                    except ValueError:
                        pass  # ignore parse errors

# Start serial reader thread
threading.Thread(target=read_serial, daemon=True).start()

def set_resolution(url: str, index: int=1, verbose: bool=False):
    try:
        if verbose:
            resolutions = (
                "10: UXGA(1600x1200)\n9: SXGA(1280x1024)\n8: XGA(1024x768)\n"
                "7: SVGA(800x600)\n6: VGA(640x480)\n5: CIF(400x296)\n"
                "4: QVGA(320x240)\n3: HQVGA(240x176)\n0: QQVGA(160x120)"
            )
            print("available resolutions\n{}".format(resolutions))

        if index in [10,9,8,7,6,5,4,3,0]:
            requests.get(f"{url}/control?var=framesize&val={index}")
        else:
            print("Wrong index")
    except Exception as e:
        print("SET_RESOLUTION:", e)

def set_quality(url: str, value: int=1, verbose: bool=False):
    try:
        if 10 <= value <= 63:
            requests.get(f"{url}/control?var=quality&val={value}")
    except Exception as e:
        print("SET_QUALITY:", e)

def set_awb(url: str, awb: bool):
    try:
        awb = not awb
        requests.get(f"{url}/control?var=awb&val={1 if awb else 0}")
    except Exception as e:
        print("SET_AWB:", e)
    return awb

if __name__ == '__main__':
    # bump resolution before start
    set_resolution(URL, index=10)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        h, w = frame.shape[:2]
        # create a black sidebar 1/3 width of frame
        sidebar_w = w // 3
        sidebar = np.zeros((h, sidebar_w, 3), dtype=np.uint8)

        # overlay telemetry text
        y0, dy = 30, 30
        for i, (key, val) in enumerate(telemetry.items()):
            text = f"{key}: {val:.2f}"
            cv2.putText(sidebar, text, (10, y0 + i*dy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)

        # concatenate frame + sidebar
        combined = np.hstack((frame, sidebar))

        cv2.imshow('IP Camera Stream', combined)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('r'):
            idx = int(input("Select resolution index: "))
            set_resolution(URL, index=idx, verbose=True)
        elif key == ord('a'):
            AWB = set_awb(URL, AWB)
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    ser.close()
