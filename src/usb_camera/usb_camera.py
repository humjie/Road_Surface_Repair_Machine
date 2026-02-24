import cv2

# On your Pi, USB webcam is usually index 0 (from your test)
# If needed, change 0 to 1
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

print("Camera opened. Press 's' to save photo, 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame")
        break

    cv2.imshow("Webcam Test", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        cv2.imwrite("photo.jpg", frame)
        print("Saved photo.jpg")

    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()