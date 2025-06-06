import cv2
import base64
from ultralytics import YOLO
from collections import Counter, deque
import numpy as np
from flask import Flask, render_template, request, jsonify, Response
import paho.mqtt.client as mqtt
import threading
import time
from datetime import datetime
import os
import json
import glob
from collections import Counter, deque, defaultdict

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

model = YOLO('runs/detect/contrung_model/weights/best.pt')

# MQTT cấu hình
MQTT_HOST = "103.146.22.13"
MQTT_PORT = 1883
MQTT_USER = "user1"
MQTT_PASS = "12345678"
MQTT_TOPIC = "doan/contrung/control"
SENSOR_TOPIC = "doan/contrung/sensor"
MQTT_SCHEDULE_RESP = "doan/contrung/schedule"


# Thư mục lưu ảnh và log
CAPTURE_DIR = "captures"
LOG_DIR = "logs"
os.makedirs(CAPTURE_DIR, exist_ok=True)
os.makedirs(LOG_DIR, exist_ok=True)

CONFIG_FILE = "config.json"
DEFAULT_CONFIG = {"speed": 255, "time": 1000}

def load_config():
    if not os.path.exists(CONFIG_FILE):
        save_config(DEFAULT_CONFIG)
    with open(CONFIG_FILE, "r") as f:
        return json.load(f)

def save_config(cfg):
    with open(CONFIG_FILE, "w") as f:
        json.dump(cfg, f)

# Toàn cục cho nhận diện liên tục
detection_thread = None
detection_running = False
# Kết quả nhận diện liên tục mới nhất (cho frontend lấy về hiển thị)
latest_result = {"image": "", "counts": {}}

# Dữ liệu cảm biến
sensor_data_buffer = deque(maxlen=200)


def get_latest_frame():
    with frame_lock:
        return global_frame.copy() if global_frame is not None else None

def process_frame_with_yolo(frame):
    insect_counts = Counter()
    results = model(frame)
    for result in results:
        if hasattr(result, 'boxes') and result.boxes is not None:
            for box in result.boxes:
                class_index = int(box.cls.cpu().numpy())
                class_name = model.names[class_index]
                insect_counts[class_name] += 1
                x1, y1, x2, y2 = map(int, box.xyxy.cpu().numpy()[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{class_name}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return frame, insect_counts

def send_conveyor_control(speed, time_ms):
    client = mqtt.Client()
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.connect(MQTT_HOST, MQTT_PORT, 60)
    payload = {
        "speed": speed,
        "time": time_ms
    }
    client.publish(MQTT_TOPIC, json.dumps(payload))
    client.disconnect()

def log_detection(dt, counts, image_path, detect_id):
    date_str = dt.strftime("%Y-%m-%d")
    log_file = os.path.join(LOG_DIR, f"detect_{date_str}.csv")
    total_insects = sum(counts.values())
    # Lưu theo format: [lần], [thời gian], [tổng], [chi tiết], [ảnh]
    with open(log_file, "a", encoding="utf-8") as f:
        f.write(f"{detect_id},{dt.strftime('%Y-%m-%d %H:%M:%S')},{total_insects},{json.dumps(dict(counts), ensure_ascii=False)},{image_path}\n")

def continuous_detect():
    global detection_running, latest_result
    detect_id = 1
    while detection_running:
        config = load_config()  # lấy lại config mới nhất
        send_conveyor_control(speed=config.get("speed", 255), time_ms=config.get("time", 1000))
        time.sleep(config.get("time", 1000) / 1000.0)

        for _ in range(7):
            frame = get_latest_frame()
        if frame is None:
            print("Không lấy được hình từ camera.")
            break

        dt = datetime.now()
        frame_draw, counts = process_frame_with_yolo(frame)

        # Lưu ảnh
        date_str = dt.strftime("%Y-%m-%d")
        time_str = dt.strftime("%H-%M-%S")
        image_name = f"detect_{date_str}_{time_str}_{detect_id}.jpg"
        image_path = os.path.join(CAPTURE_DIR, image_name)
        cv2.imwrite(image_path, frame_draw)

        # Encode ảnh base64 cho web
        _, buffer = cv2.imencode('.jpg', frame_draw)
        img_base64 = base64.b64encode(buffer).decode()

        # Cập nhật kết quả mới nhất cho web
        latest_result = {"image": img_base64, "counts": dict(counts)}

        # Ghi log
        log_detection(dt, counts, image_path, detect_id)

        print(f"Lần {detect_id} | {dt.strftime('%H:%M:%S')} | Tổng: {sum(counts.values())} | {dict(counts)}")

        detect_id += 1
        time.sleep(1)  # Delay giữa các lần nhận diện

    cap.release()
    print("Dừng nhận diện liên tục.")

# --- MQTT Sensor subscriber thread ---
def on_sensor_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        t = datetime.now().strftime("%H:%M:%S")
        data_point = {
            "time": t,
            "temperature": data.get("temperature"),
            "humidity": data.get("humidity"),
            'light': data.get("light")
        }
        sensor_data_buffer.append(data_point)
    except Exception as e:
        print("Sensor MQTT parse error:", e)

def start_sensor_subscribe():
    def _run():
        client = mqtt.Client()
        client.username_pw_set(MQTT_USER, MQTT_PASS)
        client.connect(MQTT_HOST, MQTT_PORT, 60)
        client.subscribe(SENSOR_TOPIC)
        client.on_message = on_sensor_message
        client.loop_forever()
    threading.Thread(target=_run, daemon=True).start()

start_sensor_subscribe()  # Khởi động subscriber sensor khi server start

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/stat_counts')
def stat_counts():
    # Đọc tất cả file logs/detect_*.csv, trích xuất ngày, loại côn trùng, số lượng -> gom nhóm
    log_files = glob.glob(os.path.join('logs', 'detect_*.csv'))
    stat = defaultdict(lambda: Counter())
    for file in log_files:
        try:
            with open(file, encoding="utf-8") as f:
                for line in f:
                    parts = line.strip().split(",")
                    # [detect_id],[time],[total],[json_counts],[img_path]
                    if len(parts) < 5: continue
                    day = parts[1].split(" ")[0] # yyyy-mm-dd
                    try:
                        counts = json.loads(parts[3])
                        for k, v in counts.items():
                            stat[day][k] += int(v)
                    except: continue
        except: continue
    # Chuẩn hóa data cho chart stacked bar
    days = sorted(stat.keys())
    # lấy tất cả loại côn trùng xuất hiện
    all_types = set()
    for ct in stat.values():
        all_types.update(ct.keys())
    all_types = sorted(list(all_types))
    # Chuẩn bị datasets cho chart.js
    colors = ["#f44336", "#2196f3", "#4caf50", "#ff9800", "#9c27b0", "#009688", "#e91e63", "#607d8b"]
    datasets = []
    for i, insect_type in enumerate(all_types):
        data = []
        for day in days:
            data.append(stat[day][insect_type] if insect_type in stat[day] else 0)
        datasets.append({
            "label": insect_type,
            "data": data,
            "backgroundColor": colors[i % len(colors)]
        })
    return jsonify({
        "labels": days,
        "datasets": datasets
    })

@app.route('/sensor_data')
def sensor_data():
    # Trả về list data mới nhất cho chart
    return jsonify(list(sensor_data_buffer))

def send_mqtt(payload):
    client = mqtt.Client()
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.connect(MQTT_HOST, MQTT_PORT, 60)
    client.publish(MQTT_TOPIC, json.dumps(payload))
    client.disconnect()

@app.route('/control', methods=['POST'])
def control():
    try:
        data = request.get_json()
        send_mqtt(data)
        return jsonify({"status": "ok", "msg": "Đã gửi qua MQTT", "data": data})
    except Exception as e:
        return jsonify({"status": "fail", "msg": str(e)}), 400

@app.route('/capture', methods=['POST'])
def capture():
    config = load_config()
    speed = config.get("speed", 255)
    time_action = config.get("time_action", 1000)  # Lấy time_action từ config
    time_after = config.get("time", 1000)          # Lấy time (sau) từ config

    # 1. Điều khiển băng tải chạy time_action trước
    send_conveyor_control(speed=speed, time_ms=time_action)
    time.sleep(time_action / 1000.0)

    # 2. Chụp hình
    frame = get_latest_frame()
    if frame is None:
        return jsonify({"error": "Không truy cập được camera"}), 500
    dt = datetime.now()
    frame, counts = process_frame_with_yolo(frame)
    _, buffer = cv2.imencode('.jpg', frame)
    img_base64 = base64.b64encode(buffer).decode()

    # 3. Điều khiển băng tải chạy tiếp time (sau)
    send_conveyor_control(speed=speed, time_ms=time_after)
    time.sleep(time_after / 1000.0)

    # Lưu ảnh
    date_str = dt.strftime("%Y-%m-%d")
    time_str = dt.strftime("%H-%M-%S")
    image_name = f"detect_{date_str}_{time_str}_capture.jpg"
    image_path = os.path.join(CAPTURE_DIR, image_name)
    cv2.imwrite(image_path, frame)

    # Ghi log
    log_detection(dt, counts, image_path, "capture")

    return jsonify({"image": img_base64, "counts": dict(counts)})

@app.route('/turn_off_led', methods=['POST'])
def turn_off_led():
    try:
        client = mqtt.Client()
        client.username_pw_set(MQTT_USER, MQTT_PASS)
        client.connect(MQTT_HOST, MQTT_PORT, 60)
        client.publish(MQTT_TOPIC, json.dumps({"led1": "off"}))
        client.disconnect()
        return jsonify({"msg": "Đã gửi lệnh tắt đèn qua MQTT!"})
    except Exception as e:
        return jsonify({"msg": "Lỗi gửi MQTT: " + str(e)}), 500


@app.route('/turn_on_led', methods=['POST'])
def turn_on_led():
    try:
        client = mqtt.Client()
        client.username_pw_set(MQTT_USER, MQTT_PASS)
        client.connect(MQTT_HOST, MQTT_PORT, 60)
        client.publish(MQTT_TOPIC, json.dumps({"led1": "on"}))
        client.disconnect()
        return jsonify({"msg": "Đã gửi lệnh bật đèn qua MQTT!"})
    except Exception as e:
        return jsonify({"msg": "Lỗi gửi MQTT: " + str(e)}), 500

@app.route('/turn_off_uva', methods=['POST'])
def turn_off_uva():
    try:
        client = mqtt.Client()
        client.username_pw_set(MQTT_USER, MQTT_PASS)
        client.connect(MQTT_HOST, MQTT_PORT, 60)
        client.publish(MQTT_TOPIC, json.dumps({"led2": "off"}))
        client.disconnect()
        return jsonify({"msg": "Đã gửi lệnh tắt đèn UVA qua MQTT!"})
    except Exception as e:
        return jsonify({"msg": "Lỗi gửi MQTT: " + str(e)}), 500


@app.route('/turn_on_uva', methods=['POST'])
def turn_on_uva():
    try:
        client = mqtt.Client()
        client.username_pw_set(MQTT_USER, MQTT_PASS)
        client.connect(MQTT_HOST, MQTT_PORT, 60)
        client.publish(MQTT_TOPIC, json.dumps({"led2": "on"}))
        client.disconnect()
        return jsonify({"msg": "Đã gửi lệnh bật đèn UVA qua MQTT!"})
    except Exception as e:
        return jsonify({"msg": "Lỗi gửi MQTT: " + str(e)}), 500

@app.route('/camera_stream')
def camera_stream():
    prev_frame = [None]

    def gen():
        while True:
            frame = get_latest_frame()
            if frame is not None:
                vis_frame = frame.copy()
                h, w = vis_frame.shape[:2]

                # --- Vùng nhỏ: 30% bên phải, cao 60% ở giữa ---
                x1 = int(w * 0.7)
                x2 = w
                y1 = int(h * 0.2)
                y2 = int(h * 0.8)
                roi = vis_frame[y1:y2, x1:x2]

                motion = False

                if prev_frame[0] is not None:
                    prev_roi = prev_frame[0][y1:y2, x1:x2]
                    diff = cv2.absdiff(roi, prev_roi)
                    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
                    blur = cv2.GaussianBlur(gray, (5,5), 0)
                    _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
                    dilated = cv2.dilate(thresh, None, iterations=3)
                    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for c in contours:
                        if cv2.contourArea(c) < 500:
                            continue
                        motion = True
                        (x, y, w_box, h_box) = cv2.boundingRect(c)
                        # Chuyển tọa độ từ ROI sang ảnh gốc
                        cv2.rectangle(vis_frame, (x1 + x, y1 + y), (x1 + x + w_box, y1 + y + h_box), (0,0,255), 2)

                # Vẽ khung vùng quan sát
                cv2.rectangle(vis_frame, (x1, y1), (x2, y2), (0,0,255), 2)
                if motion:
                    cv2.putText(vis_frame, "Co chuyen dong!", (x1+10, y1+40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

                prev_frame[0] = frame.copy()

                _, buffer = cv2.imencode('.jpg', vis_frame)
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.03)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video', methods=['POST'])
def process_video():
    if 'video' not in request.files:
        return jsonify({"error": "Không có file video"}), 400

    video_file = request.files['video']
    video_path = 'temp_video.mp4'
    video_file.save(video_path)

    cap = cv2.VideoCapture(video_path)
    ret, frame = cap.read()
    cap.release()
    if not ret:
        return jsonify({"error": "Không đọc được video"}), 500

    dt = datetime.now()
    frame, counts = process_frame_with_yolo(frame)
    _, buffer = cv2.imencode('.jpg', frame)
    img_base64 = base64.b64encode(buffer).decode()

    # Lưu ảnh
    date_str = dt.strftime("%Y-%m-%d")
    time_str = dt.strftime("%H-%M-%S")
    image_name = f"detect_{date_str}_{time_str}_video.jpg"
    image_path = os.path.join(CAPTURE_DIR, image_name)
    cv2.imwrite(image_path, frame)

    # Ghi log
    log_detection(dt, counts, image_path, "video")

    return jsonify({"image": img_base64, "counts": dict(counts)})

@app.route('/start_detect', methods=['POST'])
def start_detect():
    global detection_thread, detection_running
    if detection_running:
        return jsonify({"status": "Đã chạy nhận diện liên tục"}), 200
    detection_running = True
    detection_thread = threading.Thread(target=continuous_detect, daemon=True)
    detection_thread.start()
    return jsonify({"status": "Bắt đầu nhận diện liên tục"}), 200

@app.route('/stop_detect', methods=['POST'])
def stop_detect():
    global detection_running
    detection_running = False
    return jsonify({"status": "Đã dừng nhận diện liên tục"}), 200

@app.route('/latest_detect')
def latest_detect():
    # Trả về kết quả nhận diện liên tục mới nhất dưới dạng JSON
    return jsonify(latest_result)

# API: get config
@app.route('/get_config')
def get_config():
    return jsonify(load_config())

# API: save config
@app.route('/save_config', methods=['POST'])
def save_config_api():
    try:
        data = request.get_json()
        speed = int(data.get("speed", 255))
        time_ = int(data.get("time", 1000))
        ddos = int(data.get("ddos", 5))
        time_action = int(data.get("time_action", 1000))
        cfg = {"speed": speed, "time": time_, "ddos": ddos, "time_action": time_action}  # <-- thêm time_action
        save_config(cfg)
        return jsonify({"status": "ok", "msg": "Đã lưu cấu hình!"})
    except Exception as e:
        return jsonify({"status": "fail", "msg": str(e)}), 400


schedule_cache = []

def on_schedule_resp(client, userdata, msg):
    global schedule_cache
    try:
        data = json.loads(msg.payload.decode())
        if isinstance(data, list):
            schedule_cache = data
    except:
        pass

def start_mqtt_sub():
    def _run():
        client = mqtt.Client()
        client.username_pw_set(MQTT_USER, MQTT_PASS)
        client.connect(MQTT_HOST, MQTT_PORT, 60)
        client.subscribe(MQTT_SCHEDULE_RESP)
        client.on_message = on_schedule_resp
        client.loop_forever()
    threading.Thread(target=_run, daemon=True).start()

start_mqtt_sub()

@app.route("/list_schedule")
def list_schedule():
    # Gửi lệnh lên MQTT để lấy schedule
    client = mqtt.Client()
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.connect(MQTT_HOST, MQTT_PORT, 60)
    # Publish lấy schedule
    client.publish(MQTT_TOPIC, json.dumps({"action": "get_schedule"}))
    client.disconnect()
    # Đợi tối đa 2 giây để nhận về (tùy thực tế nhé)
    for _ in range(20):
        if schedule_cache:
            break
        time.sleep(0.1)
    # Trả về list (nếu chưa có thì trả mảng rỗng)
    return jsonify(schedule_cache)


if __name__ == '__main__':
    # Khởi động thread đọc camera khi server start
    threading.Thread(target=camera_capture_loop, args=(0,), daemon=True).start()
    app.run(host='0.0.0.0', port=5000)