#!/usr/bin/env python3
import cv2, sys, termios, tty, select, time

# ---- tiny helper to read one keystroke without blocking ----
def read_key():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None

# put terminal into raw mode so we get each keypress instantly
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)   # USB cam is /dev/video0 on most Pi
if not cap.isOpened():
    sys.exit("Cannot open camera")

print("Camera opened. Press 's' to save photo.jpg, 'q' to quit.")

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Can't receive frame"); break

        key = read_key()
        if key == 's':
            cv2.imwrite("photo.jpg", frame)
            print("Saved photo.jpg")
        elif key == 'q':
            break

        time.sleep(0.01)  # tiny sleep so CPU isn’t maxed out

finally:                    # always clean up
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    cap.release()