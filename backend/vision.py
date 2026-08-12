import os
import cv2
import time
import rclpy
import asyncio
import threading
import logging
import numpy as np
import mediapipe as mp
import aiohttp_cors
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCIceServer, RTCConfiguration
from av import VideoFrame
from collections import deque
from rclpy.node import Node
import onnxruntime as ort
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from mavros_msgs.msg import StatusText

WEBRTC_HOST = "0.0.0.0"
WEBRTC_PORT = 5000
STREAM_WIDTH = 640
STREAM_HEIGHT = 480
LOCAL_PREVIEW = False
USE_STUN = False

# === KONFIGURASI MODEL ONNX ===
ONNX_MODEL_PATH = "yolov8n.onnx"
ONNX_INPUT_SIZE = 640
ONNX_CONF_THRESHOLD = 0.4
ONNX_NMS_THRESHOLD = 0.45
ONNX_PERSON_CLASS_ID = 0

ICE_SERVERS = [RTCIceServer(urls=["stun:stun.l.google.com:19302"])] if USE_STUN else []
ICE_GATHERING_TIMEOUT = 5.0
DISCONNECT_GRACE_PERIOD = 8.0

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
webrtc_logger = logging.getLogger("hawkeye.webrtc")
pcs = set()

class SharedFrameBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._frame = None

    def update(self, frame):
        with self._lock:
            self._frame = frame.copy()

    def get(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

frame_buffer = SharedFrameBuffer()

class AnnotatedFrameTrack(VideoStreamTrack):
    kind = "video"

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        frame = frame_buffer.get()
        if frame is None:
            frame = np.zeros((STREAM_HEIGHT, STREAM_WIDTH, 3), dtype=np.uint8)
        vf = VideoFrame.from_ndarray(frame, format="bgr24")
        vf.pts = pts
        vf.time_base = time_base
        return vf


async def _wait_ice_gathering_complete(pc):
    while pc.iceGatheringState != "complete":
        await asyncio.sleep(0.1)

async def offer(request: web.Request):
    params = await request.json()
    if "sdp" not in params or "type" not in params:
        return web.json_response({"status": "error", "message": "Payload SDP tidak lengkap"}, status=400)

    pc = RTCPeerConnection(configuration=RTCConfiguration(iceServers=ICE_SERVERS))
    pcs.add(pc)
    webrtc_logger.info("Peer created. Total peers: %d", len(pcs))

    disconnect_timer = {"handle": None}

    async def force_close():
        webrtc_logger.info("Peer connection timeout. Closing peer.")
        await pc.close()
        pcs.discard(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        webrtc_logger.info("Connection state: %s", pc.connectionState)
        if pc.connectionState in ("failed", "closed"):
            if disconnect_timer["handle"]:
                disconnect_timer["handle"].cancel()
            await pc.close()
            pcs.discard(pc)

        elif pc.connectionState == "disconnected":
            loop = asyncio.get_event_loop()
            disconnect_timer["handle"] = loop.call_later(DISCONNECT_GRACE_PERIOD, lambda: asyncio.ensure_future(force_close()))

        elif pc.connectionState == "connected":
            if disconnect_timer["handle"]:
                disconnect_timer["handle"].cancel()
                disconnect_timer["handle"] = None

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        webrtc_logger.info("ICE state: %s", pc.iceConnectionState)

    pc.addTrack(AnnotatedFrameTrack())

    try:
        try:
            offer_desc = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        except Exception as e:
            return web.json_response({"status": "error", "message": f"SDP offer invalid: {e or type(e).__name__}"}, status=400)
        try:
            await pc.setRemoteDescription(offer_desc)
        except Exception as e:
            webrtc_logger.error("setRemoteDescription gagal", exc_info=True)
            await pc.close(); pcs.discard(pc)
            return web.json_response({"status": "error", "message": f"setRemoteDescription: {e or type(e).__name__}"}, status=500)
        try:
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
        except Exception as e:
            webrtc_logger.error("createAnswer/setLocalDescription gagal", exc_info=True)
            await pc.close(); pcs.discard(pc)
            return web.json_response({"status": "error", "message": f"createAnswer: {e or type(e).__name__}"}, status=500)
        try:
            await asyncio.wait_for(_wait_ice_gathering_complete(pc), timeout=ICE_GATHERING_TIMEOUT)
        except asyncio.TimeoutError:
            webrtc_logger.warning("ICE gathering timeout reached.", ICE_GATHERING_TIMEOUT) 
    except Exception as e:
        error_detail = str(e) or f"{type(e).__module__}.{type(e).__name__} (tanpa pesan — lihat log server)"
        webrtc_logger.error("Gagal SDP: %s", error_detail, exc_info=True)
        await pc.close()
        pcs.discard(pc)
        return web.json_response({"status": "error", "message": error_detail}, status=500)

    return web.json_response({
        "sdp": pc.localDescription.sdp,
        "type": pc.localDescription.type
    })


async def on_shutdown(app):
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros, return_exceptions=True)
    pcs.clear()


def start_webrtc_server():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    app = web.Application()
    offer_route = app.router.add_post("/webrtc/offer", offer)
    app.on_shutdown.append(on_shutdown)

    cors = aiohttp_cors.setup(app, defaults={
        "*": aiohttp_cors.ResourceOptions(
            allow_credentials=True,
            expose_headers="*",
            allow_headers="*",
        )
    })
    cors.add(offer_route)

    runner = web.AppRunner(app)
    loop.run_until_complete(runner.setup())
    site = web.TCPSite(runner, WEBRTC_HOST, WEBRTC_PORT)
    loop.run_until_complete(site.start())
    webrtc_logger.info(f"WebRTC signaling server di {WEBRTC_HOST}:{WEBRTC_PORT}")
    loop.run_forever()


class HawkeyeVision(Node):
    TRACK_MATCH_MAX_DIST = 90
    TRACK_MAX_MISS = 5
    WAVE_HISTORY_LEN = 20
    WAVE_MIN_DELTA = 0.015
    WAVE_MIN_DIRECTION_CHANGES = 4
    WAVE_MISS_RESET_FRAMES = 10
    DETECTION_INTERVAL = 3
    ARM_CONNECTIONS = ((11, 13), (13, 15), (12, 14), (14, 16))
    ARM_ROI_PADDING_RATIO = 0.15

    def __init__(self, camera_index=0):
        super().__init__('yolo_vision_node')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pub_statustext = self.create_publisher(StatusText, '/mavros/statustext/send', qos)
        self.sub_gps = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, qos_profile_sensor_data)
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.last_sent_time = 0.0
        self.output_dir = "Korban_Ditemukan"
        os.makedirs(self.output_dir, exist_ok=True)
        if not os.path.isfile(ONNX_MODEL_PATH):
            self.get_logger().error(f"Model ONNX tidak ditemukan: {ONNX_MODEL_PATH}")
            exit()

        sess_options = ort.SessionOptions()
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        cpu_count = os.cpu_count() or 4
        sess_options.intra_op_num_threads = cpu_count

        self.ort_session = ort.InferenceSession(
            ONNX_MODEL_PATH,
            sess_options=sess_options,
            providers=["CPUExecutionProvider"]
        )
        self.onnx_input_name = self.ort_session.get_inputs()[0].name
        self.onnx_output_name = self.ort_session.get_outputs()[0].name

        input_shape = self.ort_session.get_inputs()[0].shape  # biasanya [1, 3, H, W]
        h_dim, w_dim = input_shape[2], input_shape[3]
        if isinstance(h_dim, int) and isinstance(w_dim, int):
            self.onnx_input_size = (w_dim, h_dim)  # (width, height)
            self.get_logger().info(f"ONNX input size terdeteksi dari model: {w_dim}x{h_dim}")
        else:
            # dynamic axes (bukan int, misal 'batch'/'height') -> model tidak fixed-size,
            # fallback ke konstanta config
            self.onnx_input_size = (ONNX_INPUT_SIZE, ONNX_INPUT_SIZE)
            self.get_logger().warning(
                f"Model punya dynamic input shape {input_shape}, pakai fallback "
                f"ONNX_INPUT_SIZE={ONNX_INPUT_SIZE}. Pastikan ini sesuai training size model."
            )

        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(model_complexity=0, min_detection_confidence=0.6, min_tracking_confidence=0.6)
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=2, model_complexity=0, min_detection_confidence=0.6, min_tracking_confidence=0.6)
        self.camera_index = camera_index
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f"Kamera index {self.camera_index} gagal dibuka!")
            exit()
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.WINDOW_NAME = "HAWKEYE SAR VISION"
        self.WINDOW_VISIBLE = LOCAL_PREVIEW
        if self.WINDOW_VISIBLE:
            try:
                cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.WINDOW_NAME, 640, 480)
            except Exception as exc:
                self.WINDOW_VISIBLE = False
                self.get_logger().warning(f"Error cuy: {exc}")
        
        self.cached_boxes = []
        self.lacak_orang = []
        self.frame_count = 0
        self.jumlah_orang = 0
        self.next_track_id = 1
        self.jika_melambai = False
        self.fps = 0.0
        self._last_frame_time = None
        self.FPS_SMOOTHING_ALPHA = 0.1

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def update_fps(self):
        now = time.time()
        if self._last_frame_time is not None:
            delta = now - self._last_frame_time
            if delta > 0:
                instant_fps = 1.0 / delta
                self.fps = (
                    instant_fps if self.fps == 0.0
                    else self.fps + (instant_fps - self.fps) * self.FPS_SMOOTHING_ALPHA
                )
        self._last_frame_time = now

    def run(self):
        MAX_CONSECUTIVE_FRAME_ERRORS = 30
        consecutive_errors = 0

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.005)
                success, frame = self.cap.read()
                if not success or frame is None:
                    self.get_logger().warning("Gagal baca frame dari kamera, skip iterasi ini.")
                    continue

                try:
                    frame = cv2.resize(frame, (640, 480))
                    self.frame_count += 1
                    self.update_fps()
                    display_frame = frame.copy()
                    should_run_detection = self.frame_count % self.DETECTION_INTERVAL == 0
                    if should_run_detection:
                        self.process_detection(frame, display_frame)
                    self.draw_overlays(display_frame, frame)
                    frame_buffer.update(display_frame)
                    consecutive_errors = 0
                except Exception as exc:
                    consecutive_errors += 1
                    self.get_logger().error(
                        f"Gagal proses frame #{self.frame_count} ({consecutive_errors}x berturut-turut): {exc}"
                    )
                    frame_buffer.update(frame)
                    if consecutive_errors >= MAX_CONSECUTIVE_FRAME_ERRORS:
                        self.get_logger().error(
                            f"{MAX_CONSECUTIVE_FRAME_ERRORS} frame gagal berturut-turut -- "
                            f"kemungkinan bug struktural, menghentikan node."
                        )
                        break

                if self.WINDOW_VISIBLE:
                    try:
                        cv2.imshow(self.WINDOW_NAME, display_frame)
                        key = cv2.waitKey(1) & 0xFF
                        if key == 27 or cv2.getWindowProperty(self.WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                            break
                    except Exception:
                        self.WINDOW_VISIBLE = False
        except Exception as exc:
            self.get_logger().error(f"Error fatal di luar loop frame: {exc}")
        finally:
            self.cleanup()

    def process_detection(self, frame, display_frame):
        raw_boxes = self.detect_persons(frame)
        self.jumlah_orang = len(raw_boxes)
        self.lacak_orang = self.match_tracks(self.lacak_orang, raw_boxes)
        for track in self.lacak_orang:
            self.update_track_wave_status(track, frame)
        self.jika_melambai = any(t["jika_melambai"] for t in self.lacak_orang)
        self.cached_boxes = raw_boxes
        if self.jumlah_orang > 0:
            self.report_victim(display_frame)
            gesture = "MELAMBAI (minta tolong)" if self.jika_melambai else "tidak melambai"
            print(f"[DETEKSI] Korban terdeteksi! Jumlah: {self.jumlah_orang} | Gesture: {gesture}")
        else:
            print("[DETEKSI] Tidak ada korban terdeteksi.")
 
        if self.jumlah_orang > 0:
            self.report_victim(display_frame)

    def report_victim(self, display_frame):
        now = time.time()
        if now - self.last_sent_time > 2.0:
            # kondisi_str = "melambai" if self.jika_melambai else "korban_tewas"
            # waktu_lokal = time.strftime("%Y-%m-%d_%H-%M-%S")
            # nama_file = f"{self.output_dir}/KORBAN_{kondisi_str.upper()}_{waktu_lokal}_Lat{self.current_lat:.6f}_Lon{self.current_lon:.6f}.jpg"
            # cv2.imwrite(nama_file, display_frame)
            # self.get_logger().info(f"Foto korban disimpan: {nama_file}")
            msg_txt = StatusText()
            if self.jika_melambai:
                msg_txt.severity = 0 # EMERGENCY
                msg_txt.text = f"[SAR];H;{self.jumlah_orang};{self.current_lat:.6f};{self.current_lon:.6f}"[:50]
            else:
                msg_txt.severity = 6 # NOTICE
                msg_txt.text = f"[SAR];D;{self.jumlah_orang};{self.current_lat:.6f};{self.current_lon:.6f}"[:50]
            self.pub_statustext.publish(msg_txt)

            n_sub = self.pub_statustext.get_subscription_count()
            if n_sub == 0:
                print(f"[REPORT][WARNING] Tidak ada subscriber di /mavros/statustext/send")
            else:
                print(f"[REPORT] StatusText -> severity={msg_txt.severity} | text='{msg_txt.text}' | subscribers={n_sub}")
            self.last_sent_time = now

    def draw_overlays(self, display_frame, original_frame):
        for track in self.lacak_orang:
            if track.get("pose_landmarks") and track.get("pose_roi"):
                px, py, pw, ph = track["pose_roi"]
                self.draw_arm_skeleton(display_frame, track["pose_landmarks"], px, py, pw, ph)
            if track.get("hand_landmarks") and track.get("hand_box"):
                hx, hy, hw, hh = track["hand_box"]
                self.landmark_tangan(display_frame, track["hand_landmarks"], hx, hy, hw, hh)
        for (x, y, w, h) in self.cached_boxes:
            cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            self.put_text_with_bg(display_frame, "KORBAN", (x, max(18, y - 10)), color=(0, 255, 0), bg_color=(0, 60, 0))
        status = "KORBAN TERDETEKSI" if self.jumlah_orang else "MENCARI"
        self.put_text_with_bg(display_frame, status, (20, 30), color=(0, 255, 0) if self.jumlah_orang else (0, 0, 255))
        self.put_text_with_bg(display_frame, f"JUMLAH: {self.jumlah_orang}", (20, 60), color=(0, 255, 255))
        gesture_status = "MINTA TOLONG" if self.jika_melambai else "NO STATUS"
        self.put_text_with_bg(display_frame, f"STATUS: {gesture_status}", (20, 90), color=(0, 0, 255) if self.jika_melambai else (200, 200, 200))
        self.put_text_with_bg(display_frame, f"FPS: {self.fps:.1f}", (20, 120), color=(255, 255, 0))

    def preprocess_onnx(self, frame):
        h0, w0 = frame.shape[:2]
        target_w, target_h = self.onnx_input_size
        scale = min(target_w / w0, target_h / h0)
        new_w, new_h = int(round(w0 * scale)), int(round(h0 * scale))
        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        pad_w, pad_h = target_w - new_w, target_h - new_h
        top, bottom = pad_h // 2, pad_h - pad_h // 2
        left, right = pad_w // 2, pad_w - pad_w // 2
        padded = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))

        blob = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        blob = blob.transpose(2, 0, 1)[None, ...]  # HWC -> CHW -> NCHW
        return np.ascontiguousarray(blob), scale, left, top

    def detect_persons(self, frame):
        blob, scale, pad_left, pad_top = self.preprocess_onnx(frame)
        outputs = self.ort_session.run([self.onnx_output_name], {self.onnx_input_name: blob})[0]

        # Output YOLOv8 ONNX: (1, 4 + num_classes, num_boxes) -> transpose ke (num_boxes, 4 + num_classes)
        preds = np.squeeze(outputs, axis=0).T

        boxes_xywh = preds[:, :4]
        class_scores = preds[:, 4:]
        class_ids = np.argmax(class_scores, axis=1)
        confidences = class_scores[np.arange(len(class_scores)), class_ids]

        keep = (confidences >= ONNX_CONF_THRESHOLD) & (class_ids == ONNX_PERSON_CLASS_ID)
        boxes_xywh = boxes_xywh[keep]
        confidences = confidences[keep]

        if len(boxes_xywh) == 0:
            return []

        # cx, cy, w, h (skala input model) -> x1, y1, w, h (skala frame asli)
        cx, cy, w, h = boxes_xywh[:, 0], boxes_xywh[:, 1], boxes_xywh[:, 2], boxes_xywh[:, 3]
        x1 = (cx - w / 2 - pad_left) / scale
        y1 = (cy - h / 2 - pad_top) / scale
        bw = w / scale
        bh = h / scale

        nms_boxes = np.stack([x1, y1, bw, bh], axis=1).tolist()
        nms_scores = confidences.tolist()
        indices = cv2.dnn.NMSBoxes(nms_boxes, nms_scores, ONNX_CONF_THRESHOLD, ONNX_NMS_THRESHOLD)

        boxes = []
        frame_h, frame_w = frame.shape[:2]
        for i in np.array(indices).flatten():
            bx, by, bw_, bh_ = nms_boxes[i]
            bx = int(max(0, bx))
            by = int(max(0, by))
            bw_ = int(min(bw_, frame_w - bx))
            bh_ = int(min(bh_, frame_h - by))
            if bw_ > 0 and bh_ > 0:
                boxes.append((bx, by, bw_, bh_))
        return boxes

    def match_tracks(self, tracks, boxes):
        unmatched_boxes = list(boxes)
        result = []
        for track in tracks:
            tx, ty = track["center"]
            best_idx, best_dist = None, self.TRACK_MATCH_MAX_DIST
            for i, box in enumerate(unmatched_boxes):
                cx, cy = box[0] + box[2] / 2, box[1] + box[3] / 2
                dist = ((cx - tx) ** 2 + (cy - ty) ** 2) ** 0.5
                if dist < best_dist:
                    best_idx, best_dist = i, dist
            if best_idx is not None:
                box = unmatched_boxes.pop(best_idx)
                track["box"] = box
                track["center"] = (box[0] + box[2] / 2, box[1] + box[3] / 2)
                track["track_miss"] = 0
                result.append(track)
            else:
                track["track_miss"] += 1
                if track["track_miss"] <= self.TRACK_MAX_MISS:
                    result.append(track)
        for box in unmatched_boxes:
            result.append(self.create_track(box))
        return result

    def create_track(self, box):
        track = {
            "id": self.next_track_id,
            "box": box,
            "center": (box[0] + box[2] / 2, box[1] + box[3] / 2),
            "wave_history": deque(maxlen=self.WAVE_HISTORY_LEN),
            "hand_miss": 0,
            "track_miss": 0,
            "jika_melambai": False,
            "hand_box": None,
            "hand_landmarks": None,
            "pose_landmarks": None,
            "pose_roi": None,
        }
        self.next_track_id += 1
        return track

    def update_track_wave_status(self, track, frame):
        x, y, w, h = track["box"]
        person_roi = frame[y:y + h, x:x + w]
        if person_roi.size == 0:
            track["hand_landmarks"], track["pose_landmarks"] = None, None
            return
        pose_result = self.pose.process(cv2.cvtColor(person_roi, cv2.COLOR_BGR2RGB))
        arm_box = (x, y, w, h)
        if pose_result and pose_result.pose_landmarks:
            arm_candidate = self.get_arm_roi(pose_result.pose_landmarks, (x, y, w, h))
            if arm_candidate: arm_box = arm_candidate
            track["pose_landmarks"] = pose_result.pose_landmarks
            track["pose_roi"] = (x, y, w, h)
        else:
            track["pose_landmarks"], track["pose_roi"] = None, None
        ax, ay, aw, ah = arm_box
        arm_roi = frame[ay:ay + ah, ax:ax + aw]
        if arm_roi.size == 0:
            track["hand_landmarks"] = None
            return
        hand_result = self.hands.process(cv2.cvtColor(arm_roi, cv2.COLOR_BGR2RGB))
        if hand_result and hand_result.multi_hand_landmarks:
            primary_hand = hand_result.multi_hand_landmarks[0]
            track["hand_landmarks"] = primary_hand
            track["hand_box"] = arm_box
            track["hand_miss"] = 0
            track["wave_history"].append(self.get_palm_center_x(primary_hand))
            hand_is_open = self.count_extended_fingers(primary_hand) >= 3
            track["jika_melambai"] = hand_is_open and self.detect_wave(track["wave_history"])
        else:
            track["hand_landmarks"], track["hand_box"] = None, None
            track["hand_miss"] += 1
            if track["hand_miss"] > self.WAVE_MISS_RESET_FRAMES:
                track["wave_history"].clear()
                track["jika_melambai"] = False

    def detect_wave(self, history):
        if len(history) < 6: 
            return False
        directions = [1 if history[i+1] - history[i] > 0 else -1 for i in range(len(history)-1) if abs(history[i+1] - history[i]) >= self.WAVE_MIN_DELTA]
        if len(directions) < 4: 
            return False
        return sum(1 for i in range(len(directions)-1) if directions[i] != directions[i+1]) >= self.WAVE_MIN_DIRECTION_CHANGES

    def get_arm_roi(self, pose_landmarks, person_box):
        x, y, w, h = person_box
        lm = pose_landmarks.landmark
        valid = [i for i in (11, 12, 15, 16) if lm[i].visibility > 0.3]
        if not valid: 
            return None
        ys, xs = [lm[i].y for i in valid], [lm[i].x for i in valid]
        top, bottom, left, right = min(ys)*h, max(ys)*h, min(xs)*w, max(xs)*w
        pad_y, pad_x = (bottom - top)*self.ARM_ROI_PADDING_RATIO + 10, (right - left)*self.ARM_ROI_PADDING_RATIO + 10
        ax1, ay1 = int(max(0, left - pad_x)), int(max(0, top - pad_y))
        ax2, ay2 = int(min(w, right + pad_x)), int(min(h, bottom + pad_y))
        if ax2 <= ax1 or ay2 <= ay1: 
            return None
        return (x + ax1, y + ay1, ax2 - ax1, ay2 - ay1)

    @staticmethod
    def put_text_with_bg(img, text, org, font=cv2.FONT_HERSHEY_SIMPLEX, font_scale=0.7, color=(255, 255, 255), thickness=2, bg_color=(0, 0, 0), padding=6, alpha=0.55):
        (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        x, y = org
        overlay = img.copy()
        cv2.rectangle(overlay, (max(0, x-padding), max(0, y-th-padding)), (min(img.shape[1], x+tw+padding), min(img.shape[0], y+baseline+padding)), bg_color, -1)
        cv2.addWeighted(overlay, alpha, img, 1-alpha, 0, img)
        cv2.putText(img, text, org, font, font_scale, color, thickness, cv2.LINE_AA)

    @staticmethod
    def get_palm_center_x(hand_landmarks):
        return sum(hand_landmarks.landmark[i].x for i in (0, 5, 9, 13, 17)) / 5

    @staticmethod
    def count_extended_fingers(hand_landmarks):
        def dist(a, b): 
            return ((a.x - b.x)**2 + (a.y - b.y)**2)**0.5
        wrist = hand_landmarks.landmark[0]
        return sum(1 for pip, tip in ((6, 8), (10, 12), (14, 16), (18, 20)) if dist(wrist, hand_landmarks.landmark[tip]) > dist(wrist, hand_landmarks.landmark[pip]) * 1.15)

    def draw_arm_skeleton(self, frame, pose_landmarks, roi_x, roi_y, roi_w, roi_h):
        lm = pose_landmarks.landmark
        pts = {i: (int(roi_x + lm[i].x*roi_w), int(roi_y + lm[i].y*roi_h)) for i in (11, 12, 13, 14, 15, 16) if lm[i].visibility > 0.3}
        for a, b in self.ARM_CONNECTIONS:
            if a in pts and b in pts: cv2.line(frame, pts[a], pts[b], (255, 0, 0), 2)
        for p in pts.values(): cv2.circle(frame, p, 4, (255, 0, 0), -1)

    def landmark_tangan(self, frame, hand_landmarks, roi_x, roi_y, roi_w, roi_h):
        pts = [(int(roi_x + lm.x*roi_w), int(roi_y + lm.y*roi_h)) for lm in hand_landmarks.landmark]
        for a, b in self.mp_hands.HAND_CONNECTIONS: cv2.line(frame, pts[a], pts[b], (0, 200, 255), 2)
        for i, p in enumerate(pts):
            cv2.circle(frame, p, 4, (0, 140, 255), -1)
            if i in (0, 4, 8, 12, 16, 20): cv2.putText(frame, str(i), (p[0]+4, p[1]-4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

    def cleanup(self):
        self.get_logger().info("Mematikan Node dan Kamera...")
        if self.cap.isOpened():
            self.cap.release()
        if self.WINDOW_VISIBLE:
            cv2.destroyAllWindows()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)

    webrtc_thread = threading.Thread(target=start_webrtc_server, daemon=True)
    webrtc_thread.start()

    vision_node = HawkeyeVision(camera_index=0)
    vision_node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()