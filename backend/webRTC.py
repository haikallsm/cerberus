import asyncio
import logging
import aiohttp_cors
import cv2
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCIceServer, RTCConfiguration
from av import VideoFrame

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
logger = logging.getLogger("webrtc")

pcs = set()

CAMERA_DEVICE_ID = 0
CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 480
CAPTURE_FPS = 20

ICE_SERVERS = [
    RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
]
# ICE_SERVERS = []

class WebcamVideoTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self, device_id: int = CAMERA_DEVICE_ID):
        super().__init__()
        self.cap = cv2.VideoCapture(device_id, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, CAPTURE_FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError(f"Tidak bisa membuka kamera di device {device_id}")

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        loop = asyncio.get_running_loop()
        ret, frame = await loop.run_in_executor(None, self.cap.read)
        if not ret or frame is None:
            raise RuntimeError("Gagal membaca frame dari webcam")
        vf = VideoFrame.from_ndarray(frame, format="bgr24")
        vf.pts = pts
        vf.time_base = time_base
        return vf

    def stop(self):
        super().stop()
        if self.cap is not None:
            self.cap.release()
            self.cap = None

async def offer(request: web.Request):
    params = await request.json()
    if "sdp" not in params or "type" not in params:
        return web.json_response({"status": "error", "message": "Payload SDP tidak lengkap"}, status=400)

    pc = RTCPeerConnection(configuration=RTCConfiguration(iceServers=ICE_SERVERS))
    pcs.add(pc)
    logger.info("Peer created. Total peers: %d", len(pcs))

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        logger.info("Connection state: %s", pc.connectionState)
        if pc.connectionState in ("failed", "closed"):
            await pc.close()
            pcs.discard(pc)

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        logger.info("ICE state: %s", pc.iceConnectionState)

    @pc.on("icegatheringstatechange")
    async def on_icegatheringstatechange():
        logger.info("ICE gathering: %s", pc.iceGatheringState)

    try:
        track = WebcamVideoTrack()
    except Exception as e:
        await pc.close()
        pcs.discard(pc)
        return web.json_response({"status": "error", "message": str(e)}, status=500)

    pc.addTrack(track)

    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    while pc.iceGatheringState != "complete":
        await asyncio.sleep(0.1)

    return web.json_response({
        "sdp": pc.localDescription.sdp,
        "type": pc.localDescription.type
    })

async def on_shutdown(app):
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros, return_exceptions=True)
    pcs.clear()

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

if __name__ == "__main__":
    web.run_app(app, host="0.0.0.0", port=5000)