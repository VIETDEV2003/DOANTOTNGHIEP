import cv2

def test_resolutions(index=0):
    # Danh sách độ phân giải phổ biến để thử
    resolutions = [
        (1920, 1080), (1280, 720), (1024, 768),
        (800, 600), (640, 480), (320, 240)
    ]
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print("Không mở được camera!")
        return
    for w, h in resolutions:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        ret, frame = cap.read()
        if ret:
            print(f"Yêu cầu: {w}x{h} -> Kết quả thực tế: {frame.shape[1]}x{frame.shape[0]}")
        else:
            print(f"Không lấy được frame ở {w}x{h}")
    cap.release()

if __name__ == "__main__":
    test_resolutions(0)