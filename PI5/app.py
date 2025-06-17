import cv2
import base64
from collections import Counter, deque, defaultdict
import numpy as np
from flask import Flask, render_template, request, jsonify, Response
import paho.mqtt.client as mqtt
import threading
import time
from datetime import datetime
import os
import json
import glob
import queue
import supervision as sv

# --------- Hailo imports ----------
from utils import HailoAsyncInference

# ==== Hailo model and label config ====
HEF_PATH = "contrung_model.hef"  # Đường dẫn file model .hef của bạn
LABEL_PATH = "contrung_labels.txt"  # Đường dẫn file txt nhãn của bạn

with open(LABEL_PATH, "r", encoding="utf-8") as f:
    class_names = [line.strip() for line in f]
input_queue = queue.Queue()
output_queue = queue.Queue()
hailo_inference = HailoAsyncInference(
    hef_path=HEF_PATH,
    input_queue=input_queue,
    output_queue=output_queue,
)
# Start Hailo inference thread
threading.Thread(target=hailo_inference.run, daemon=True).start()

global_frame = None
frame_lock = threading.Lock()

PIXEL_TO_MM = 0.05  # <--- Cập nhật hệ số này theo thực tế camera/máy của bạn

def camera_capture_loop(index):
    global global_frame
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print(f"Khong mo duoc camera o index {index}")
        return
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Khong lay duoc frame, dung thread camera.")
            break
        with frame_lock:
            global_frame = frame.copy()
        time.sleep(0.03)
    cap.release()

app = Flask(__name__)

# MQTT cau hinh
MQTT_HOST = "103.146.22.13"
MQTT_PORT = 1883
MQTT_USER = "user1"
MQTT_PASS = "12345678"
MQTT_TOPIC = "doan/contrung/control"
SENSOR_TOPIC = "doan/contrung/sensor"
MQTT_SCHEDULE_RESP = "doan/contrung/schedule"

# Thu muc luu anh va log
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

sensor_data_buffer = deque(maxlen=200)

def get_latest_frame():
    with frame_lock:
        return global_frame.copy() if global_frame is not None else None

def extract_detections(
    hailo_output, h, w, threshold=0.5
):
    xyxy = []
    confidence = []
    class_id = []
    num_detections = 0
    for i, detections in enumerate(hailo_output):
        if len(detections) == 0:
            continue
        for detection in detections:
            bbox, score = detection[:4], detection[4]
            if score < threshold:
                continue
            bbox[0], bbox[1], bbox[2], bbox[3] = (
                bbox[1] * w,
                bbox[0] * h,
                bbox[3] * w,
                bbox[2] * h,
            )
            xyxy.append(bbox)
            confidence.append(score)
            class_id.append(i)
            num_detections += 1
    return {
        "xyxy": np.array(xyxy, dtype=np.float32).reshape(-1, 4),
        "confidence": np.array(confidence, dtype=np.float32),
        "class_id": np.array(class_id, dtype=np.int32),
        "num_detections": num_detections,
    }

def process_frame_with_hailo(frame):
    insect_info_list = []
    insect_counts = Counter()
    model_h, model_w, _ = hailo_inference.get_input_shape()
    input_frame = cv2.resize(frame, (model_w, model_h))
    input_queue.put([input_frame])
    _, hailo_results = output_queue.get()
    if len(hailo_results) == 1:
        hailo_results = hailo_results[0]
    detections = extract_detections(hailo_results, frame.shape[0], frame.shape[1], threshold=0.5)
    xyxy = detections["xyxy"]
    class_id = detections["class_id"]
    for i in range(xyxy.shape[0]):
        x1, y1, x2, y2 = xyxy[i].astype(int)
        cls = class_id[i]
        class_name = class_names[cls]
        insect_counts[class_name] += 1

        # Tính kích thước côn trùng (mm)
        width_pixel = abs(x2-x1)
        height_pixel = abs(y2-y1)
        width_mm = round(width_pixel * PIXEL_TO_MM, 2)
        height_mm = round(height_pixel * PIXEL_TO_MM, 2)

        insect_info_list.append({
            "class": class_name,
            "width_mm": width_mm,
            "height_mm": height_mm,
            "box": [int(x1), int(y1), int(x2), int(y2)]
        })
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"{class_name}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return frame, insect_counts, insect_info_list

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
    with open(log_file, "a", encoding="utf-8") as f:
        f.write(f"{detect_id},{dt.strftime('%Y-%m-%d %H:%M:%S')},{total_insects},{json.dumps(dict(counts), ensure_ascii=False)},{image_path}\n")

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

start_sensor_subscribe()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/stat_counts')
def stat_counts():
    log_files = glob.glob(os.path.join('logs', 'detect_*.csv'))
    stat = defaultdict(lambda: Counter())
    for file in log_files:
        try:
            with open(file, encoding="utf-8") as f:
                for line in f:
                    parts = line.strip().split(",")
                    if len(parts) < 5: continue
                    day = parts[1].split(" ")[0]
                    try:
                        counts = json.loads(parts[3])
                        for k, v in counts.items():
                            stat[day][k] += int(v)
                    except: continue
        except: continue
    days = sorted(stat.keys())
    all_types = set()
    for ct in stat.values():
        all_types.update(ct.keys())
    all_types = sorted(list(all_types))
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
        return jsonify({"status": "ok", "msg": "Da gui qua MQTT", "data": data})
    except Exception as e:
        return jsonify({"status": "fail", "msg": str(e)}), 400

@app.route('/capture', methods=['POST'])
def capture():
    config = load_config()
    speed = config.get("speed", 255)
    time_action = config.get("time_action", 1000)
    time_after = config.get("time", 1000)

    send_conveyor_control(speed=speed, time_ms=time_action)
    time.sleep(time_action / 1000.0)

    frame = get_latest_frame()
    if frame is None:
        return jsonify({"error": "Khong truy cap duoc camera"}), 500
    dt = datetime.now()
    date_str = dt.strftime("%Y-%m-%d")
    time_str = dt.strftime("%H-%M-%S")

    raw_dir = "raw"
    os.makedirs(raw_dir, exist_ok=True)
    raw_image_name = f"raw_{date_str}_{time_str}_capture.jpg"
    raw_image_path = os.path.join(raw_dir, raw_image_name)
    cv2.imwrite(raw_image_path, frame)

    frame, counts, insect_info_list = process_frame_with_hailo(frame)
    _, buffer = cv2.imencode('.jpg', frame)
    img_base64 = base64.b64encode(buffer).decode()

    send_conveyor_control(speed=speed, time_ms=time_after)
    time.sleep(time_after / 1000.0)

    image_name = f"detect_{date_str}_{time_str}_capture.jpg"
    image_path = os.path.join(CAPTURE_DIR, image_name)
    cv2.imwrite(image_path, frame)
    log_detection(dt, counts, image_path, "capture")

    global latest_result
    latest_result = {
        "image": img_base64,
        "counts": dict(counts),
        "insects": [
            {
                "class": item["class"],
                "width_mm": item["width_mm"],
                "height_mm": item["height_mm"],
                "detected_at": dt.strftime("%Y-%m-%d %H:%M:%S")
            }
            for item in insect_info_list
        ]
    }

    return jsonify(latest_result)

@app.route('/camera_stream')
def camera_stream():
    def gen():
        while True:
            frame = get_latest_frame()
            if frame is not None:
                dt = datetime.now()
                frame_draw, counts, insect_info_list = process_frame_with_hailo(frame.copy())
                global latest_result
                _, buffer = cv2.imencode('.jpg', frame_draw)
                img_base64 = base64.b64encode(buffer).decode()
                latest_result = {
                    "image": img_base64,
                    "counts": dict(counts),
                    "insects": [
                        {
                            "class": item["class"],
                            "width_mm": item["width_mm"],
                            "height_mm": item["height_mm"],
                            "detected_at": dt.strftime("%Y-%m-%d %H:%M:%S")
                        }
                        for item in insect_info_list
                    ]
                }
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.03)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video', methods=['POST'])
def process_video():
    if 'video' not in request.files:
        return jsonify({"error": "Khong co file video"}), 400

    video_file = request.files['video']
    video_path = 'temp_video.mp4'
    video_file.save(video_path)

    cap = cv2.VideoCapture(video_path)
    ret, frame = cap.read()
    cap.release()
    if not ret:
        return jsonify({"error": "Khong doc duoc video"}), 500

    dt = datetime.now()
    frame, counts, insect_info_list = process_frame_with_hailo(frame)
    _, buffer = cv2.imencode('.jpg', frame)
    img_base64 = base64.b64encode(buffer).decode()

    date_str = dt.strftime("%Y-%m-%d")
    time_str = dt.strftime("%H-%M-%S")
    image_name = f"detect_{date_str}_{time_str}_video.jpg"
    image_path = os.path.join(CAPTURE_DIR, image_name)
    cv2.imwrite(image_path, frame)
    log_detection(dt, counts, image_path, "video")

    global latest_result
    latest_result = {
        "image": img_base64,
        "counts": dict(counts),
        "insects": [
            {
                "class": item["class"],
                "width_mm": item["width_mm"],
                "height_mm": item["height_mm"],
                "detected_at": dt.strftime("%Y-%m-%d %H:%M:%S")
            }
            for item in insect_info_list
        ]
    }

    return jsonify(latest_result)

@app.route('/get_config')
def get_config():
    return jsonify(load_config())

@app.route('/save_config', methods=['POST'])
def save_config_api():
    try:
        data = request.get_json()
        speed = int(data.get("speed", 255))
        time_ = int(data.get("time", 1000))
        ddos = int(data.get("ddos", 5))
        time_action = int(data.get("time_action", 1000))
        cfg = {"speed": speed, "time": time_, "ddos": ddos, "time_action": time_action}
        save_config(cfg)
        return jsonify({"status": "ok", "msg": "Da luu cau hinh!"})
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
    client = mqtt.Client()
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.connect(MQTT_HOST, MQTT_PORT, 60)
    client.publish(MQTT_TOPIC, json.dumps({"action": "get_schedule"}))
    client.disconnect()
    for _ in range(20):
        if schedule_cache:
            break
        time.sleep(0.1)
    return jsonify(schedule_cache)

# ------- API latest_detect -------
latest_result = {
    "image": "",
    "counts": {},
    "insects": []
}

@app.route('/latest_detect')
def latest_detect():
    # Trả về kết quả nhận diện liên tục mới nhất dưới dạng JSON
    # bao gồm loại, kích thước (mm), thời gian nhận diện
    return jsonify(latest_result)

if __name__ == '__main__':
    threading.Thread(target=camera_capture_loop, args=(0,), daemon=True).start()
    app.run(host='0.0.0.0', port=5000)