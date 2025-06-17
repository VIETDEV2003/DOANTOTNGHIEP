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

detection_thread = None
detection_running = False
latest_result = {"image": "", "counts": {}}
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
    with open(log_file, "a", encoding="utf-8") as f:
        f.write(f"{detect_id},{dt.strftime('%Y-%m-%d %H:%M:%S')},{total_insects},{json.dumps(dict(counts), ensure_ascii=False)},{image_path}\n")

def continuous_detect():
    global detection_running, latest_result
    detect_id = 1

    config = load_config()
    speed = config.get("speed", 255)
    time_action = config.get("time_action", 1000)
    time_after = config.get("time", 1000)

    send_conveyor_control(speed=speed, time_ms=time_action)
    time.sleep(time_action / 1000.0)

    for _ in range(7):
        frame = get_latest_frame()
    if frame is None:
        print("Khong lay duoc hinh tu camera.")
        return

    dt = datetime.now()
    date_str = dt.strftime("%Y-%m-%d")
    time_str = dt.strftime("%H-%M-%S")

    raw_dir = "raw"
    os.makedirs(raw_dir, exist_ok=True)
    raw_image_name = f"raw_{date_str}_{time_str}_{detect_id}.jpg"
    raw_image_path = os.path.join(raw_dir, raw_image_name)
    cv2.imwrite(raw_image_path, frame)

    frame_draw, counts = process_frame_with_hailo(frame)

    send_conveyor_control(speed=speed, time_ms=time_after)
    time.sleep(time_after / 1000.0)

    image_name = f"detect_{date_str}_{time_str}_{detect_id}.jpg"
    image_path = os.path.join(CAPTURE_DIR, image_name)
    cv2.imwrite(image_path, frame_draw)

    _, buffer = cv2.imencode('.jpg', frame_draw)
    img_base64 = base64.b64encode(buffer).decode()

    latest_result = {"image": img_base64, "counts": dict(counts)}

    log_detection(dt, counts, image_path, detect_id)

    print(f"Lan {detect_id} | {dt.strftime('%H:%M:%S')} | Tong: {sum(counts.values())} | {dict(counts)}")
    print("Nhan dien xong 1 lan, dung.")

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

    frame, counts = process_frame_with_hailo(frame)
    _, buffer = cv2.imencode('.jpg', frame)
    img_base64 = base64.b64encode(buffer).decode()

    send_conveyor_control(speed=speed, time_ms=time_after)
    time.sleep(time_after / 1000.0)

    image_name = f"detect_{date_str}_{time_str}_capture.jpg"
    image_path = os.path.join(CAPTURE_DIR, image_name)
    cv2.imwrite(image_path, frame)
    log_detection(dt, counts, image_path, "capture")

    return jsonify({"image": img_base64, "counts": dict(counts)})

# (Các route MQTT LED/UVA giữ nguyên như code cũ...)

motion_lock = threading.Lock()

@app.route('/camera_stream')
def camera_stream():
    prev_frame = [None]
    global detection_running, detection_thread

    def gen():
        global detection_running, detection_thread
        while True:
            frame = get_latest_frame()
            if frame is not None:
                vis_frame = frame.copy()
                h, w = vis_frame.shape[:2]
                x1 = int(w * 0.7)
                x2 = w
                y1 = int(h * 0.28)
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
                        cv2.rectangle(vis_frame, (x1 + x, y1 + y), (x1 + x + w_box, y1 + y + h_box), (0,0,255), 2)

                cv2.rectangle(vis_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                if motion:
                    cv2.putText(vis_frame, "Co chuyen dong!", (x1 + 10, y1 + 40), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (0, 0, 255), 2)

                    if detection_running:
                        if detection_thread is None or not detection_thread.is_alive():
                            check_frame = get_latest_frame()
                            if check_frame is not None:
                                roi_check = check_frame[y1:y2, x1:x2]
                                frame_roi, counts = process_frame_with_hailo(roi_check)
                                has_object = sum(counts.values()) > 0
                                if has_object:
                                    object_names = list(counts.keys())
                                    print(
                                        f">>> Trigger NHAN DIEN (co object trong vung chuyen dong): {', '.join(object_names)}")
                                    detection_thread = threading.Thread(target=continuous_detect, daemon=True)
                                    detection_thread.start()
                                else:
                                    print(">>> Chuyen dong nhung KHONG co object (bo qua) <<<")
                            else:
                                print(">>> Khong lay duoc frame moi de kiem tra object <<<")
                        else:
                            print(">>> Thread nhan dien dang chay, khong tao moi <<<")
                    else:
                        print(">>> detection_running=False, khong tao thread <<<")
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
    frame, counts = process_frame_with_hailo(frame)
    _, buffer = cv2.imencode('.jpg', frame)
    img_base64 = base64.b64encode(buffer).decode()

    date_str = dt.strftime("%Y-%m-%d")
    time_str = dt.strftime("%H-%M-%S")
    image_name = f"detect_{date_str}_{time_str}_video.jpg"
    image_path = os.path.join(CAPTURE_DIR, image_name)
    cv2.imwrite(image_path, frame)
    log_detection(dt, counts, image_path, "video")

    return jsonify({"image": img_base64, "counts": dict(counts)})

@app.route('/start_detect', methods=['POST'])
def start_detect():
    global detection_running
    if detection_running:
        return jsonify({"status": "Da chay nhan dien lien tuc"}), 200
    detection_running = True
    return jsonify({"status": "Bat dau nhan dien lien tuc"}), 200

@app.route('/stop_detect', methods=['POST'])
def stop_detect():
    global detection_running
    detection_running = False
    return jsonify({"status": "Da dung nhan dien lien tuc"}), 200

@app.route('/latest_detect')
def latest_detect():
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

if __name__ == '__main__':
    threading.Thread(target=camera_capture_loop, args=(0,), daemon=True).start()
    app.run(host='0.0.0.0', port=5000)