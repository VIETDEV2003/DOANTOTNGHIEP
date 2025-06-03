import cv2
import threading
import time

global_frame = None
frame_lock = threading.Lock()
stop_flag = False

def camera_capture_loop(index):
    global global_frame, stop_flag
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print(f"Không mở được camera ở index {index}")
        stop_flag = True
        return
    print(f"✅ Đã mở camera ở index {index}")
    while not stop_flag:
        ret, frame = cap.read()
        if not ret:
            print("Không lấy được frame, dừng thread camera.")
            break
        with frame_lock:
            global_frame = frame.copy()
        time.sleep(0.03)
    cap.release()

if __name__ == "__main__":
    camera_index = 0  # Đổi về 1 nếu cần
    t = threading.Thread(target=camera_capture_loop, args=(camera_index,))
    t.start()

    while True:
        with frame_lock:
            frame = global_frame.copy() if global_frame is not None else None
        if frame is not None:
            cv2.imshow('Camera', frame)
        key = cv2.waitKey(1) & 0xFF
        # Nhấn q để thoát
        if key == ord('q'):
            stop_flag = True
            break
        time.sleep(0.01)
    t.join()
    cv2.destroyAllWindows()