import cv2
import numpy as np
import math
import serial
import threading

# ——— draw inclinometer in-place ———
def draw_inclinometer(roll_deg, pitch_deg, W, H):
    img = np.zeros((H, W, 3), dtype=np.uint8)
    cx, cy = W // 2, H // 2
    radius = min(cx, cy) - 5

    # horizon offset and tilt
    y_off = int((pitch_deg / 90.0) * radius)
    theta = math.radians(-roll_deg)
    c, s = math.cos(theta), math.sin(theta)

    # coordinate grid relative to center
    ys, xs = np.indices((H, W))
    dx = xs - cx
    dy = ys - cy

    # rotated coords for horizon check
    xr = dx * c + (dy + y_off) * s
    yr = -dx * s + (dy + y_off) * c

    # mask inside circle
    circle_mask = (dx**2 + dy**2) <= radius**2

    # sky (blue) and ground (brown)
    sky_color    = (230,   216,   173)
    ground_color = ( 29,  101, 181)
    white_color = (255,255,255)
    top_mask = (yr < 0) & circle_mask
    bot_mask = (yr >= 0) & circle_mask
    img[top_mask] = sky_color
    img[bot_mask] = ground_color

    # # draw horizon/pitch marker line
    # y_line = cy + y_off
    # dy0 = y_off
    # if abs(dy0) <= radius:
    #     dx_line = int(math.sqrt(radius**2 - dy0**2))
    #     x1, x2 = cx - dx_line, cx + dx_line
    #     cv2.line(img, (x1, y_line), (x2, y_line), (255,255,255), 1)

    # draw roll arrow
    arrow_len = int(radius * 0.5)
    arrow_angle = -roll_deg - 90
    ax = cx + int(arrow_len * math.cos(math.radians(arrow_angle)))
    ay = cy + int(arrow_len * math.sin(math.radians(arrow_angle)))
    black_color = (0,0,0)
    cv2.arrowedLine(img, (cx, cy), (ax, ay),black_color, 1, tipLength=0.3)

     # — roll markers every 30° —
    for mark in [0, -30, -60, -90, -120, -150, -180]:
        ang = math.radians(mark)
        x1 = int(cx + (radius-8)*math.cos(ang))
        y1 = int(cy + (radius-8)*math.sin(ang))
        x2 = int(cx + radius*math.cos(ang))
        y2 = int(cy + radius*math.sin(ang))
        cv2.line(img, (x1,y1), (x2,y2), black_color, 1)

    # — pitch markers at ±15°, ±30° —
    for mark in [-30, -15, 15, 30]:
        y_line = cy + int((mark/90.0)*radius)
        cv2.line(img,
                 (cx - radius//4, y_line),
                 (cx + radius//4, y_line),
                 black_color, 1)

    # draw fixed circle & crosshair
    cv2.circle(img, (cx, cy), radius, (255,255,255), 2)
    cv2.line(img, (cx-radius, cy), (cx+radius, cy), (255,255,255), 2)
    cv2.line(img, (cx, cy-radius), (cx, cy+radius), (255,255,255), 2)

    # labels
    #cv2.putText(img, f"Roll: {roll_deg:.1f}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
    #cv2.putText(img, f"Pitch: {pitch_deg:.1f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

   
        
    return img