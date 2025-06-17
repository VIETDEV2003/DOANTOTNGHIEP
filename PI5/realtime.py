#!/usr/bin/env python3
"""Detection + ByteTrack + Supervision, log ten con trung va chieu dai mm len console va frame."""

import argparse
import supervision as sv
import numpy as np
from tqdm import tqdm
import cv2
import queue
import sys
import os
import time
from typing import Dict, List
import threading
from collections import deque

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import HailoAsyncInference

def initialize_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Detection Example - Tracker with ByteTrack and Supervision"
    )
    parser.add_argument(
        "-n", "--net", help="Path for the HEF model.", default="best.hef"
    )
    parser.add_argument(
        "-i", "--input_video", default="input_video.mp4", help="Path to the input video or camera index (int)."
    )
    parser.add_argument(
        "-l", "--labels", default="coco.txt", help="Path to a text file containing labels."
    )
    parser.add_argument(
        "-s", "--score_thresh", type=float, default=0.5, help="Score threshold - between 0 and 1."
    )
    parser.add_argument(
        "--mm_per_pixel", type=float, default=0.25, help="Ty le mm tren moi pixel (vi du: 0.25)."
    )
    return parser

def is_camera_source(source):
    # kiem tra input la so nguyen (camera index) hay la file video
    try:
        int(source)
        return True
    except ValueError:
        return False

def preprocess_frame(
    frame: np.ndarray, model_h: int, model_w: int, video_h: int, video_w: int
) -> np.ndarray:
    # resize frame neu kich thuoc khac voi kich thuoc model
    if model_h != video_h or model_w != video_w:
        return cv2.resize(frame, (model_w, model_h))
    return frame

def extract_detections(
    hailo_output: List[np.ndarray], h: int, w: int, threshold: float = 0.5
) -> Dict[str, np.ndarray]:
    # tach thong tin detection tu hailo_output
    xyxy: List[np.ndarray] = []
    confidence: List[float] = []
    class_id: List[int] = []
    num_detections: int = 0

    for i, detections in enumerate(hailo_output):
        if len(detections) == 0:
            continue
        for detection in detections:
            bbox, score = detection[:4], detection[4]
            if score < threshold:
                continue
            # chuyen bbox sang toa do pixel
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

def postprocess_detections(
    frame: np.ndarray,
    detections: Dict[str, np.ndarray],
    class_names: List[str],
    tracker: sv.ByteTrack,
    box_annotator: sv.RoundBoxAnnotator,
    label_annotator: sv.LabelAnnotator,
    mm_per_pixel: float = 0.25,
) -> np.ndarray:
    # ve bbox va label len frame, dong thoi log thong tin con trung va chieu dai mm
    sv_detections = sv.Detections(
        xyxy=detections["xyxy"],
        confidence=detections["confidence"],
        class_id=detections["class_id"],
    )
    sv_detections = tracker.update_with_detections(sv_detections)
    labels: List[str] = []

    for i, (class_id, tracker_id, box) in enumerate(zip(
            sv_detections.class_id,
            sv_detections.tracker_id,
            sv_detections.xyxy)):
        name = class_names[class_id]
        x1, y1, x2, y2 = box
        width = abs(x2 - x1)
        height = abs(y2 - y1)
        length_pixel = max(width, height)
        length_mm = length_pixel * mm_per_pixel
        # log ra ten va chieu dai mm cua con trung phat hien duoc
        print(f"Con trung phat hien: {name}, Chieu dai: {length_mm:.2f} mm")
        # hien thi label len frame gom ten va chieu dai mm
        labels.append(f"#{tracker_id} {name} {length_mm:.1f}mm")

    annotated_frame: np.ndarray = box_annotator.annotate(
        scene=frame.copy(), detections=sv_detections
    )
    annotated_labeled_frame: np.ndarray = label_annotator.annotate(
        scene=annotated_frame, detections=sv_detections, labels=labels
    )
    return annotated_labeled_frame

def display_frames(display_queue: queue.Queue, window_name="Detections"):
    # thread hien thi frame len cua so opencv
    while True:
        frame = display_queue.get()
        if frame is None:
            break
        cv2.imshow(window_name, frame)
        # nhan q de thoat
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()

def main() -> None:
    args = initialize_arg_parser().parse_args()
    mm_per_pixel = args.mm_per_pixel

    input_queue: queue.Queue = queue.Queue()
    output_queue: queue.Queue = queue.Queue()
    display_queue: queue.Queue = queue.Queue(maxsize=5)

    hailo_inference = HailoAsyncInference(
        hef_path=args.net,
        input_queue=input_queue,
        output_queue=output_queue,
    )
    model_h, model_w, _ = hailo_inference.get_input_shape()

    # kiem tra nguon la camera hay file video
    if is_camera_source(args.input_video):
        cam_index = int(args.input_video)
        cap = cv2.VideoCapture(cam_index)
        if not cap.isOpened():
            print(f"Khong mo duoc camera index {cam_index}")
            return
        video_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        video_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        def cam_frame_gen():
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                yield frame
            cap.release()
        frame_generator = cam_frame_gen()
        total_frames = None
    else:
        # neu la file video
        frame_generator = sv.get_video_frames_generator(source_path=args.input_video)
        video_info = sv.VideoInfo.from_video_path(video_path=args.input_video)
        video_w, video_h = video_info.resolution_wh
        total_frames = video_info.total_frames

    box_annotator = sv.RoundBoxAnnotator()
    label_annotator = sv.LabelAnnotator()
    tracker = sv.ByteTrack()

    with open(args.labels, "r", encoding="utf-8") as f:
        class_names: List[str] = f.read().splitlines()

    # thread chay hailo inference
    inference_thread = threading.Thread(target=hailo_inference.run)
    inference_thread.start()

    # thread hien thi video
    display_thread = threading.Thread(target=display_frames, args=(display_queue,))
    display_thread.start()

    fps_queue = deque(maxlen=10)
    prev_time = time.time()

    # neu la video file thi dung tqdm, neu la camera thi khong can
    if total_frames is not None:
        frame_iter = tqdm(frame_generator, total=total_frames)
    else:
        frame_iter = frame_generator

    for frame in frame_iter:
        preprocessed_frame: np.ndarray = preprocess_frame(
            frame, model_h, model_w, video_h, video_w
        )

        input_queue.put([preprocessed_frame])

        results: List[np.ndarray]
        _, results = output_queue.get()

        if len(results) == 1:
            results = results[0]

        detections: Dict[str, np.ndarray] = extract_detections(
            results, video_h, video_w, args.score_thresh
        )

        annotated_labeled_frame: np.ndarray = postprocess_detections(
            frame, detections, class_names, tracker, box_annotator, label_annotator, mm_per_pixel
        )

        # tinh va hien thi FPS len frame
        curr_time = time.time()
        fps = 1.0 / (curr_time - prev_time) if curr_time != prev_time else 0
        fps_queue.append(fps)
        avg_fps = sum(fps_queue) / len(fps_queue)
        prev_time = curr_time
        cv2.putText(
            annotated_labeled_frame,
            f"FPS: {avg_fps:.2f}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        try:
            display_queue.put_nowait(annotated_labeled_frame)
        except queue.Full:
            # bo qua frame neu queue bi day
            pass

    # ket thuc, dong cac thread con lai
    display_queue.put(None)
    input_queue.put(None)
    inference_thread.join()
    display_thread.join()

if __name__ == "__main__":
    main()