"""WebRTC camera streaming server for grain sampling robot."""
from __future__ import annotations
import argparse
import asyncio
import json
import logging
import time
from fractions import Fraction
from typing import Optional

import cv2
import numpy as np
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack

import aiortc.rtcpeerconnection as _rtcpc
_orig_and = _rtcpc.and_direction
_rtcpc.and_direction = lambda a, b: _orig_and(a or 'sendonly', b or 'sendonly')


class CameraVideoTrack(VideoStreamTrack):
    """Opens /dev/video0 via OpenCV and yields frames."""

    def __init__(self, device: int = 0):
        super().__init__()
        self._cap = cv2.VideoCapture(device)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    async def recv(self):
        import av

        ok, frame = self._cap.read() if self._cap else (False, None)
        if not ok or frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        video_frame = av.VideoFrame.from_ndarray(frame_rgb, format="rgb24")
        video_frame.pts = int(time.time() * 90000)
        video_frame.time_base = Fraction(1, 90000)
        return video_frame


class WebRTCServer:
    def __init__(self, device: int = 0, port: int = 8080):
        self._device = device
        self._port = port
        self._pcs: set[RTCPeerConnection] = set()
        self._track: Optional[CameraVideoTrack] = None

    async def _offer(self, request):
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        pc = RTCPeerConnection()
        self._pcs.add(pc)

        @pc.on("connectionstatechange")
        async def on_state():
            if pc.connectionState in ("failed", "closed"):
                self._pcs.discard(pc)

        if self._track is None:
            self._track = CameraVideoTrack(device=self._device)
        pc.addTrack(self._track)
        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        return web.json_response(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        )

    async def _index(self, request):
        html = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Camera - Sampling Robot</title>
<style>body{background:#0D1117;color:#E6EDF3;display:flex;flex-direction:column;align-items:center;font-family:sans-serif}
h1{margin:20px;color:#58A6FF}video{width:90vw;max-width:854px;border-radius:12px}
.status{margin:10px;padding:6px 20px;border-radius:16px}.connecting{background:#1F2937}
.connected{background:#0E4429;color:#3FB950}.failed{background:#492A2A;color:#F85149}
button{margin:10px;padding:8px 24px;background:#238636;color:#fff;border:none;border-radius:8px;cursor:pointer}
</style></head><body>
<h1>Sampling Robot Camera</h1>
<video id="v" autoplay muted playsinline></video>
<div id="s" class="status connecting">Connecting...</div>
<button onclick="connect()">Reconnect</button>
<script>
const pc=new RTCPeerConnection({iceServers:[{urls:"stun:stun.l.google.com:19302"}]});
pc.ontrack=e=>document.getElementById("v").srcObject=e.streams[0];
pc.oniceconnectionstatechange=()=>{
 const s=document.getElementById("s");
 if(pc.iceConnectionState==="connected"){s.className="status connected";s.textContent="Connected"}
 else if(pc.iceConnectionState==="failed"||pc.iceConnectionState==="disconnected"){s.className="status failed";s.textContent="Disconnected"}
};
async function connect(){
 const s=document.getElementById("s");s.className="status connecting";s.textContent="Connecting...";
 const o=await pc.createOffer();await pc.setLocalDescription(o);
 const r=await fetch("/offer",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({sdp:pc.localDescription.sdp,type:pc.localDescription.type})});
 const a=await r.json();await pc.setRemoteDescription(new RTCSessionDescription(a));
}
connect();
</script></body></html>"""
        return web.Response(content_type="text/html", text=html)

    async def start(self):
        app = web.Application()
        app.router.add_get("/", self._index)
        app.router.add_post("/offer", self._offer)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "0.0.0.0", self._port)
        await site.start()
        logging.getLogger("webrtc_server").info(
            "WebRTC on http://0.0.0.0:%d", self._port
        )

    async def shutdown(self):
        for pc in self._pcs:
            await pc.close()
        self._pcs.clear()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--device", type=int, default=0)
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO, format="[%(levelname)s] %(name)s: %(message)s"
    )
    srv = WebRTCServer(device=args.device, port=args.port)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(srv.start())
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        loop.run_until_complete(srv.shutdown())
        loop.close()


if __name__ == "__main__":
    main()
