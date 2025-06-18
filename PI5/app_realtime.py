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
import uuid

# --------- Hailo imports ----------
from utils import HailoAsyncInference

HEF_PATH = "yolov8n.hef"
LABEL_PATH = "contrung_labels.txt"

with open(LABEL_PATH, "r", encoding="utf-8") as f:
    class_names = [line.strip() for line in f]
input_queue = queue.Queue()
output_queue = queue.Queue()
hailo_inference = HailoAsyncInference(
    hef_path=HEF_PATH,
    input_queue=input_queue,
    output_queue=output_queue,
)
threading.Thread(target=hailo_inference.run, daemon=True).start()

global_frame = None
frame_lock = threading.Lock()
PIXEL_TO_MM = 0.05

# ==== TRACKING ====
tracker = sv.ByteTrack()  # tracking toàn cục
box_annotator = sv.RoundBoxAnnotator()
label_annotator = sv.LabelAnnotator()


def camera_capture_loop(index):
    global global_frame
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print(f"Khong mo duoc camera o index {index}")
        return

    # Set Full HD resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

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

MQTT_HOST = "103.146.22.13"
MQTT_PORT = 1883
MQTT_USER = "user1"
MQTT_PASS = "12345678"
MQTT_TOPIC = "doan/contrung/control"
SENSOR_TOPIC = "doan/contrung/sensor"
MQTT_SCHEDULE_RESP = "doan/contrung/schedule"
MQTT_DETECT_RESULT = "doan/contrung/result"

CAPTURE_DIR = "captures"
LOG_DIR = "logs"
os.makedirs(CAPTURE_DIR, exist_ok=True)
os.makedirs(LOG_DIR, exist_ok=True)

CONFIG_FILE = "config.json"
DEFAULT_CONFIG = {"speed": 255, "time": 1000, "auto_stop_delay": 5}

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

def extract_detections(hailo_output, h, w, threshold=0.5):
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
    insects_list = []
    model_h, model_w, _ = hailo_inference.get_input_shape()
    input_frame = cv2.resize(frame, (model_w, model_h))
    input_queue.put([input_frame])
    _, hailo_results = output_queue.get()
    if len(hailo_results) == 1:
        hailo_results = hailo_results[0]
    detections = extract_detections(hailo_results, frame.shape[0], frame.shape[1], threshold=0.5)
    # ---------- TRACKING ----------
    sv_detections = sv.Detections(
        xyxy=detections["xyxy"],
        confidence=detections["confidence"],
        class_id=detections["class_id"],
    )
    sv_detections = tracker.update_with_detections(sv_detections)
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    labels = []
    for i, (class_id, tracker_id, box) in enumerate(zip(
            sv_detections.class_id,
            sv_detections.tracker_id,
            sv_detections.xyxy,
        )):
        name = class_names[class_id]
        x1, y1, x2, y2 = box
        width_pixel = abs(x2 - x1)
        height_pixel = abs(y2 - y1)
        width_mm = round(width_pixel * PIXEL_TO_MM, 2)
        height_mm = round(height_pixel * PIXEL_TO_MM, 2)
        insects_list.append({
            "tracker_id": int(tracker_id) if tracker_id is not None else None,
            "class": name,
            "width_mm": width_mm,
            "height_mm": height_mm,
            "detected_at": now,
        })
        # Label: ID + class + size
        labels.append(f"#{tracker_id} {name} {width_mm}x{height_mm}mm")
    # Annotate frame
    annotated_frame = box_annotator.annotate(
        scene=frame.copy(), detections=sv_detections
    )
    annotated_labeled_frame = label_annotator.annotate(
        scene=annotated_frame, detections=sv_detections, labels=labels
    )
    return annotated_labeled_frame, insects_list

def send_detect_mqtt(insects_list):
    client = mqtt.Client()
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.connect(MQTT_HOST, MQTT_PORT, 60)
    client.publish(MQTT_DETECT_RESULT, json.dumps({"insects": insects_list}, ensure_ascii=False))
    client.disconnect()

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

logged_tracker_ids = set()
conveyor_running = False
last_insect_time = time.time()

@app.route('/camera_stream')
def camera_stream():
    global logged_tracker_ids
    def gen():
        global conveyor_running, last_insect_time

        frame_count = 0
        last_fps_time = time.time()
        fps = 0.0
        resolution = None

        while True:
            frame = get_latest_frame()
            if frame is not None:
                frame_draw, insects_list = process_frame_with_hailo(frame.copy())

                # Điều khiển băng tải tự động
                now = time.time()
                has_insect = len(insects_list) > 0

                cfg = load_config()
                auto_stop_delay = cfg.get("auto_stop_delay", 5)

                if has_insect:
                    last_insect_time = now
                    if not conveyor_running:
                        speed = cfg.get("speed", 170)
                        send_conveyor_control(speed, -1)
                        conveyor_running = True
                        print("Đã gửi lệnh CHẠY băng tải vì phát hiện côn trùng")
                else:
                    print("Khong phat hien con trung.... Dung sau " + str(now - last_insect_time) + " giay")
                    if conveyor_running and (now - last_insect_time > auto_stop_delay):
                        send_conveyor_control(0, 0)
                        conveyor_running = False
                        print("Da gui lenh DUNG băng tải sau " + str(auto_stop_delay) + " giay khong phat hien con trung")

                # ==== Thêm đo và in FPS, độ phân giải lên frame ====
                frame_count += 1
                current_time = time.time()
                if resolution is None:
                    h, w = frame_draw.shape[:2]
                    resolution = f"{w}x{h}"
                if current_time - last_fps_time >= 1.0:
                    fps = frame_count / (current_time - last_fps_time)
                    frame_count = 0
                    last_fps_time = current_time
                overlay_text = f"Resolution: {resolution} | FPS: {fps:.2f}"
                cv2.putText(
                    frame_draw, overlay_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA
                )

                # --- Chỉ gửi MQTT cho tracker_id mới ---
                new_insects = []
                new_tracker_ids = set()
                for item in insects_list:
                    tid = item.get("tracker_id")
                    if tid is not None and tid not in logged_tracker_ids and tid not in new_tracker_ids:
                        new_insects.append(item)
                        new_tracker_ids.add(tid)
                if new_insects:
                    counts = Counter(item['class'] for item in new_insects)
                    detect_id = str(uuid.uuid4())
                    image_path = os.path.join(CAPTURE_DIR, f"{detect_id}.jpg")
                    cv2.imwrite(image_path, frame_draw)
                    log_detection(datetime.now(), counts, image_path, detect_id)
                    send_detect_mqtt(new_insects)
                    # Đảm bảo tracker_id đã gửi sẽ không bao giờ gửi lại nữa
                    logged_tracker_ids.update(new_tracker_ids)

                _, buffer = cv2.imencode('.jpg', frame_draw)
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.03)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/get_config')
def get_config():
    return jsonify(load_config())


@app.route('/save_config', methods=['POST'])
def save_config_api():
    try:
        data = request.get_json()
        speed = int(data.get("speed", 170))
        ddos = int(data.get("ddos", 5))
        auto_stop_delay = float(data.get("auto_stop_delay", 5))  # mới thêm

        cfg = {
            "speed": speed,
            "ddos": ddos,
            "auto_stop_delay": auto_stop_delay
        }
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

def send_conveyor_forever():
    cfg = load_config()
    speed = cfg.get("speed", 170)
    time_ms = -1
    send_conveyor_control(speed, time_ms)
    print(f"Đã gửi lệnh chạy băng tải: speed={speed}, time={time_ms}")

if __name__ == '__main__':
    send_conveyor_control(0, 0)
    threading.Thread(target=camera_capture_loop, args=(0,), daemon=True).start()
    app.run(host='0.0.0.0', port=5000)