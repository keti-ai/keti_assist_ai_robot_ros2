"""
FastAPI backend for the Zenoh robot GUI: a Home tab and a Video tab that
can show several live camera streams at once in a grid.

Run with:
    .venv/bin/python -m app.main --zenoh-host 127.0.0.1 --zenoh-port 7447
"""

import argparse
from pathlib import Path

from fastapi import FastAPI, Query
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from .stream_manager import StreamManager
from .zenoh_scanner import scan_image_topics

STATIC_DIR = Path(__file__).resolve().parent.parent / "static"

app = FastAPI(title="Zenoh Robot GUI")
app.state.zenoh_host = "127.0.0.1"
app.state.zenoh_port = 7447
app.state.stream_manager = None


@app.get("/")
def index():
    return FileResponse(STATIC_DIR / "index.html")


@app.get("/api/status")
def status():
    return {
        "zenoh_host": app.state.zenoh_host,
        "zenoh_port": app.state.zenoh_port,
    }


@app.get("/api/topics")
def list_topics():
    try:
        topics = scan_image_topics(app.state.zenoh_host, app.state.zenoh_port)
        return {"ok": True, "topics": topics}
    except Exception as e:
        return {"ok": False, "error": str(e), "topics": []}


def _mjpeg_generator(stream):
    version = 0
    while True:
        jpeg, version = stream.get_latest(version, timeout=1.0)
        if jpeg is None:
            continue
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"


@app.get("/api/stream")
def stream(topic: str = Query(..., description="zenoh key expression")):
    stream_obj = app.state.stream_manager.get_stream(topic)
    return StreamingResponse(
        _mjpeg_generator(stream_obj),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


def main():
    parser = argparse.ArgumentParser(description="Zenoh robot web GUI (FastAPI)")
    parser.add_argument("--zenoh-host", default="127.0.0.1", help="Zenoh bridge IP or hostname")
    parser.add_argument("--zenoh-port", type=int, default=7447, help="Zenoh bridge port")
    parser.add_argument("--web-host", default="0.0.0.0", help="Web server bind address")
    parser.add_argument("--web-port", type=int, default=8080, help="Web server port")
    args = parser.parse_args()

    app.state.zenoh_host = args.zenoh_host
    app.state.zenoh_port = args.zenoh_port
    app.state.stream_manager = StreamManager(args.zenoh_host, args.zenoh_port)

    import uvicorn

    print(f"[web] Open http://<this-pc-ip>:{args.web_port}/ from any browser on the network")
    uvicorn.run(app, host=args.web_host, port=args.web_port)


if __name__ == "__main__":
    main()
