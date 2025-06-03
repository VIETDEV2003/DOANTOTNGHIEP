import cv2

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Không mở được camera ở index 0")
else:
    ret, frame = cap.read()
    if not ret:
        print("Mở được camera ở index 0 nhưng không lấy được frame")
    else:
        print("✅ Mở và chụp được frame từ camera ở index 0")
        cv2.imshow('Camera 0', frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    cap.release()