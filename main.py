import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib, GstApp
from fastapi import FastAPI, Request
from fastapi.responses import StreamingResponse
import uvicorn
import threading
import cv2
import numpy as np
from contextlib import asynccontextmanager

app = FastAPI()

rtsp_url = "rtsp://210.99.70.120:1935/live/cctv001.stream"

# GStreamer 파이프라인 설정
Gst.init(None)
pipeline = Gst.parse_launch(
    f"rtspsrc location={rtsp_url} latency=50 ! decodebin ! videoconvert ! video/x-raw,format=BGR ! appsink name=sink max-buffers=1 drop=true"
)
appsink = pipeline.get_by_name("sink")
appsink.set_property("emit-signals", True)
appsink.set_property("sync", False)

# 파이프라인 상태를 관리하는 변수
pipeline_active = False

def gstreamer_thread():
    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        pass

async def generate_frames(request: Request):
    global pipeline_active
    pipeline.set_state(Gst.State.PLAYING)
    pipeline_active = True
    
    while pipeline_active:
        if await request.is_disconnected():
            break

        sample = appsink.emit("pull-sample")
        if sample:
            buf = sample.get_buffer()
            caps = sample.get_caps()
            arr = np.ndarray(
                (caps.get_structure(0).get_value("height"),
                 caps.get_structure(0).get_value("width"),
                 3),
                buffer=buf.extract_dup(0, buf.get_size()),
                dtype=np.uint8
            )
            
            # 프레임을 JPEG 형식으로 인코딩
            ret, buffer = cv2.imencode('.jpg', arr)
            if not ret:
                continue

            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    pipeline.set_state(Gst.State.NULL)

@app.get("/video_feed")
async def video_feed(request: Request):
    return StreamingResponse(generate_frames(request), media_type="multipart/x-mixed-replace; boundary=frame")

@asynccontextmanager
async def lifespan(app: FastAPI):
    threading.Thread(target=gstreamer_thread, daemon=True).start()
    yield
    global pipeline_active
    pipeline_active = False
    pipeline.set_state(Gst.State.NULL)

app.router.lifespan_context = lifespan

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
