# jangan lupa install
# pip install aiortc aiohttp aiohttp_cors av

import os
import cv2
import time
import rclpy
import json
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
from ultralytics import YOLO
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
        offer_desc = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        await pc.setRemoteDescription(offer_desc)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        try:
            async with asyncio.timeout(ICE_GATHERING_TIMEOUT):
                while pc.iceGatheringState != "complete":
                    await asyncio.sleep(0.1)
        except asyncio.TimeoutError:
            webrtc_logger.warning("ICE gathering timeout reached.", ICE_GATHERING_TIMEOUT) 
    except Exception as e:
        webrtc_logger.error("Gagal SDP: %s", e, exc_info=True)
        await pc.close()
        pcs.discard(pc)
        return web.json_response({"status": "error", "message": str(e)}, status=500)

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
    ZOOM_SMOOTH_ALPHA = 0.15
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
        self.model = YOLO("yolov8n.pt")
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
        self.zoom_scale = 1.0
        self.smooth_zoom_scale = 1.0
        self.jika_melambai = False
        self.auto_zoom = True
        self.zoom_center = None
        self.smooth_zoom_center = None
        self.auto_enhance = True
        self.auto_enhance_person = True

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def run(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.005)
                success, frame = self.cap.read()
                if not success or frame is None:
                    break
                frame = cv2.resize(frame, (640, 480))
                if self.auto_enhance:
                    frame = self.auto_brightness_contrast(frame)
                self.frame_count += 1
                display_frame = frame.copy()
                should_run_detection = self.frame_count % self.DETECTION_INTERVAL == 0
                if should_run_detection:
                    self.process_detection(frame, display_frame)
                else:
                    display_frame, zoom_region = self.apply_zoom(frame, self.zoom_scale, self.zoom_center)
                self.draw_overlays(display_frame, frame, zoom_region if not should_run_detection else self.last_zoom_region)
                frame_buffer.update(display_frame)
                if self.WINDOW_VISIBLE:
                    try:
                        cv2.imshow(self.WINDOW_NAME, display_frame)
                        key = cv2.waitKey(1) & 0xFF
                        if key == 27 or cv2.getWindowProperty(self.WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                            break
                    except Exception:
                        self.WINDOW_VISIBLE = False
        except Exception as exc:
            self.get_logger().error(f"Error: {exc}")
        finally:
            self.cleanup()

    def process_detection(self, frame, display_frame):
        raw_boxes = self.detect_persons(frame)
        self.jumlah_orang = len(raw_boxes)
        self.lacak_orang = self.match_tracks(self.lacak_orang, raw_boxes)
        for track in self.lacak_orang:
            self.update_track_wave_status(track, frame)
        target_track = self.pick_zoom_target(self.lacak_orang)
        self.jika_melambai = bool(target_track and target_track["jika_melambai"])
        if self.auto_zoom and target_track is not None:
            target_box = target_track["box"]
            target_center = (target_box[0] + target_box[2] // 2, target_box[1] + target_box[3] // 2)
            target_scale = self.compute_auto_zoom_scale(target_box, (frame.shape[1], frame.shape[0]))
        else:
            target_center = None
            target_scale = 1.0
        self.smooth_zoom_center = self.lerp_point(self.smooth_zoom_center, target_center, self.ZOOM_SMOOTH_ALPHA)
        self.smooth_zoom_scale = self.lerp(self.smooth_zoom_scale, target_scale, self.ZOOM_SMOOTH_ALPHA)
        self.zoom_center = self.smooth_zoom_center
        self.zoom_scale = self.smooth_zoom_scale
        display_frame[:], zoom_region = self.apply_zoom(frame, self.zoom_scale, self.zoom_center)
        self.last_zoom_region = zoom_region
        self.cached_boxes = []
        for b in raw_boxes:
            mapped = self.map_box_to_zoom(b, zoom_region, (frame.shape[1], frame.shape[0]))
            if mapped is not None:
                self.cached_boxes.append(mapped)
        if self.jumlah_orang > 0:
            self.report_victim(display_frame)

    def report_victim(self, display_frame):
        now = time.time()
        if now - self.last_sent_time > 2.0:
            kondisi_str = "melambai" if self.jika_melambai else "korban_tewas"
            waktu_lokal = time.strftime("%Y-%m-%d_%H-%M-%S")
            nama_file = f"{self.output_dir}/KORBAN_{kondisi_str.upper()}_{waktu_lokal}_Lat{self.current_lat:.6f}_Lon{self.current_lon:.6f}.jpg"
            cv2.imwrite(nama_file, display_frame)
            self.get_logger().info(f"Foto korban disimpan: {nama_file}")
            msg_txt = StatusText()
            if self.jika_melambai:
                msg_txt.severity = 0 # EMERGENCY
                msg_txt.text = f"[SAR];H;{self.jumlah_orang};{self.current_lat:.6f};{self.current_lon:.6f}"[:50]
            else:
                msg_txt.severity = 6 # NOTICE
                msg_txt.text = f"[SAR];D;{self.jumlah_orang};{self.current_lat:.6f};{self.current_lon:.6f}"[:50]
            self.pub_statustext.publish(msg_txt)
            self.last_sent_time = now

    def draw_overlays(self, display_frame, original_frame, zoom_region):
        if self.auto_enhance_person and self.cached_boxes:
            for person_box in self.cached_boxes:
                display_frame[:] = self.enhance_person_region(display_frame, person_box)
        for track in self.lacak_orang:
            if track.get("pose_landmarks") and track.get("pose_roi"):
                mapped_pose = self.map_box_to_zoom(track["pose_roi"], zoom_region, (display_frame.shape[1], display_frame.shape[0]))
                if mapped_pose:
                    self.draw_arm_skeleton(display_frame, track["pose_landmarks"], *mapped_pose)
            if track.get("hand_landmarks") and track.get("hand_box"):
                mapped_hand = self.map_box_to_zoom(track["hand_box"], zoom_region, (display_frame.shape[1], display_frame.shape[0]))
                if mapped_hand:
                    self.landmark_tangan(display_frame, track["hand_landmarks"], *mapped_hand)
        for (x, y, w, h) in self.cached_boxes:
            cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            self.put_text_with_bg(display_frame, "KORBAN", (x, max(18, y - 10)), color=(0, 255, 0), bg_color=(0, 60, 0))
        status = "KORBAN TERDETEKSI" if self.jumlah_orang else "MENCARI"
        self.put_text_with_bg(display_frame, status, (20, 30), color=(0, 255, 0) if self.jumlah_orang else (0, 0, 255))
        self.put_text_with_bg(display_frame, f"JUMLAH: {self.jumlah_orang}", (20, 60), color=(0, 255, 255))
        gesture_status = "MINTA TOLONG" if self.jika_melambai else "NO STATUS"
        self.put_text_with_bg(display_frame, f"STATUS: {gesture_status}", (20, 90), color=(0, 0, 255) if self.jika_melambai else (200, 200, 200))
        self.put_text_with_bg(display_frame, f"Zoom: {self.zoom_scale:.1f}x {'AUTO' if self.auto_zoom else ''}", (20, 470), font_scale=0.6)
        self.add_pip(display_frame, original_frame)

    def detect_persons(self, frame):
        results = self.model(frame, conf=0.4, classes=[0], verbose=False)
        boxes = []
        for r in results:
            if r.boxes is not None:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    boxes.append((x1, y1, x2 - x1, y2 - y1))
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

    def pick_zoom_target(self, tracks):
        if not tracks: 
            return None
        melambai = [t for t in tracks if t["jika_melambai"]]
        pool = melambai if melambai else tracks
        return max(pool, key=lambda t: t["box"][2] * t["box"][3])

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
    def auto_brightness_contrast(gambar, clip_hist_percent=1):
        if gambar is None or gambar.size == 0: 
            return gambar
        gray = cv2.cvtColor(gambar, cv2.COLOR_BGR2GRAY)
        hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
        accumulator = hist.cumsum()
        max_value = accumulator[-1]
        clip_hist_value = (clip_hist_percent * max_value / 100.0) / 2.0
        min_gray, max_gray = np.searchsorted(accumulator, clip_hist_value), np.searchsorted(accumulator, max_value - clip_hist_value)
        if min_gray >= max_gray: 
            return gambar.copy()
        alpha = 255.0 / (max_gray - min_gray)
        beta = -min_gray * alpha
        auto_result = cv2.convertScaleAbs(gambar, alpha=alpha, beta=beta)
        beta += (128.0 - np.mean(cv2.cvtColor(auto_result, cv2.COLOR_BGR2GRAY)))
        return cv2.convertScaleAbs(gambar, alpha=alpha, beta=beta)

    @staticmethod
    def enhance_person_region(frame, box, clip_limit=3.0, tile_grid=(8, 8), brightness_boost=18):
        x, y, w, h = [max(0, v) for v in box]
        w, h = min(w, frame.shape[1] - x), min(h, frame.shape[0] - y)
        if w <= 0 or h <= 0: 
            return frame
        roi = frame[y:y+h, x:x+w]
        if roi.size == 0: 
            return frame
        lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        l_ch, a_ch, b_ch = cv2.split(lab)
        l_ch = cv2.convertScaleAbs(cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid).apply(l_ch), alpha=1.0, beta=brightness_boost)
        frame[y:y+h, x:x+w] = cv2.cvtColor(cv2.merge((l_ch, a_ch, b_ch)), cv2.COLOR_LAB2BGR)
        return frame

    @staticmethod
    def put_text_with_bg(img, text, org, font=cv2.FONT_HERSHEY_SIMPLEX, font_scale=0.7, color=(255, 255, 255), thickness=2, bg_color=(0, 0, 0), padding=6, alpha=0.55):
        (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        x, y = org
        overlay = img.copy()
        cv2.rectangle(overlay, (max(0, x-padding), max(0, y-th-padding)), (min(img.shape[1], x+tw+padding), min(img.shape[0], y+baseline+padding)), bg_color, -1)
        cv2.addWeighted(overlay, alpha, img, 1-alpha, 0, img)
        cv2.putText(img, text, org, font, font_scale, color, thickness, cv2.LINE_AA)

    @staticmethod
    def add_pip(main_frame, small_source, size=(200, 150), margin=12):
        h, w = main_frame.shape[:2]
        sw, sh = min(size[0], w - 2*margin), min(size[1], h - 2*margin)
        x1, y1 = w - sw - margin, margin
        main_frame[y1:y1+sh, x1:x1+sw] = cv2.resize(small_source, (sw, sh), interpolation=cv2.INTER_AREA)
        cv2.rectangle(main_frame, (x1, y1), (x1+sw, y1+sh), (255, 255, 255), 2)
        HawkeyeVision.put_text_with_bg(main_frame, "FULL VIEW (No Zoom)", (x1, max(18, y1-8)), font_scale=0.45, thickness=1)

    @staticmethod
    def apply_zoom(frame, zoom, center=None):
        if zoom <= 1.0: 
            return frame.copy(), (0, 0, frame.shape[1], frame.shape[0])
        h, w = frame.shape[:2]
        center = center or (w // 2, h // 2)
        crop_w, crop_h = min(int(w / zoom), w), min(int(h / zoom), h)
        x1, y1 = max(0, min(center[0] - crop_w // 2, w - crop_w)), max(0, min(center[1] - crop_h // 2, h - crop_h))
        return cv2.resize(frame[y1:y1+crop_h, x1:x1+crop_w], (w, h), interpolation=cv2.INTER_LINEAR), (x1, y1, crop_w, crop_h)

    @staticmethod
    def map_box_to_zoom(box, zoom_region, output_size):
        bx, by, bw, bh = box
        zx, zy, zw, zh = zoom_region
        if zw <= 0 or zh <= 0: 
            return None
        scale_x, scale_y = output_size[0] / zw, output_size[1] / zh
        nx1, ny1, nx2, ny2 = (bx-zx)*scale_x, (by-zy)*scale_y, (bx+bw-zx)*scale_x, (by+bh-zy)*scale_y
        if nx2 <= 0 or ny2 <= 0 or nx1 >= output_size[0] or ny1 >= output_size[1]: 
            return None
        nx1, ny1, nx2, ny2 = max(0, int(nx1)), max(0, int(ny1)), min(output_size[0], int(nx2)), min(output_size[1], int(ny2))
        return (nx1, ny1, nx2-nx1, ny2-ny1) if nx2 > nx1 and ny2 > ny1 else None

    @staticmethod
    def compute_auto_zoom_scale(box, frame_size, margin=1.3):
        if box[2] <= 0 or box[3] <= 0: 
            return 1.0
        return float(np.clip(min(frame_size[0] / (box[2]*margin), frame_size[1] / (box[3]*margin)), 1.0, 2.5))

    @staticmethod
    def lerp(curr, target, alpha): 
        return curr + (target - curr) * alpha

    @staticmethod
    def lerp_point(curr, target, alpha):
        if curr is None: 
            return target
        if target is None: 
            return curr
        return (HawkeyeVision.lerp(curr[0], target[0], alpha), HawkeyeVision.lerp(curr[1], target[1], alpha))

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