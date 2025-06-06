import cv2
import threading
import time
from flask import Flask, Response

global_frame = None
frame_lock = threading.Lock()

def camera_capture_loop(index):
    global global_frame
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print(f"Không mở được camera ở index {index}")
        return
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Không lấy được frame, dừng thread camera.")
            break
        with frame_lock:
            global_frame = frame.copy()
        time.sleep(0.03)
    cap.release()

app = Flask(__name__)

@app.route('/video_feed')
def video_feed():
    def gen():
        while True:
            with frame_lock:
                frame = global_frame.copy() if global_frame is not None else None
            if frame is not None:
                _, buffer = cv2.imencode('.jpg', frame)
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.03)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    threading.Thread(target=camera_capture_loop, args=(0,), daemon=True).start()
    app.run(host='0.0.0.0', port=5000)