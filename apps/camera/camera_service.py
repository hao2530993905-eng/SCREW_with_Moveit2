#!/usr/bin/env python
# PROJECT FILE HEADER
# 文件：apps/camera/camera_service.py
# 作用：本文件属于 SCREW07091444 自动螺丝拧紧项目，负责该目录所对应的功能模块。
# 用法：请从项目根目录按 README/项目说明.md 中的命令运行；修改参数前先确认单位、设备和输出路径。
# 注意：本文件不应把运行日志、缓存、模型输出或临时数据写回源码目录。
# END PROJECT FILE HEADER

from __future__ import annotations

import argparse
import datetime as dt
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

import numpy as np

from ring import CameraRingFitProcessor, CircleFitConfig


REPO_ROOT = Path(__file__).resolve().parents[2]


def resolve_repo_path(path: str | Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


class CameraService:
    def __init__(self, save_ply: bool, only_outer: bool, output_dir: str, epoch: int, warmup: bool):
        config = CircleFitConfig()
        config.output_dir = str(resolve_repo_path(output_dir))
        config.epoch = epoch
        config.save_ply = save_ply
        config.only_outer = only_outer
        Path(config.output_dir).mkdir(parents=True, exist_ok=True)
        self.processor = CameraRingFitProcessor(config)
        self.lock = threading.Lock()
        if warmup:
            self._warmup_model()

    def _warmup_model(self) -> None:
        print("正在预热 YOLO/ONNX 模型...")
        started = time.perf_counter()
        try:
            image = np.zeros((640, 640, 3), dtype=np.uint8)
            self.processor.model(image, verbose=False)
        except TypeError:
            self.processor.model(np.zeros((640, 640, 3), dtype=np.uint8))
        print(f"模型预热耗时: {time.perf_counter() - started:.3f}s")

    def capture(self, ip: str, image_id: int | None = None) -> dict[str, Any]:
        if image_id is None:
            image_id = int(dt.datetime.now().timestamp()) % 10000
        started = time.perf_counter()
        with self.lock:
            result = self.processor.run_camera_pipeline(ip, image_id)
        result["service_elapsed"] = time.perf_counter() - started
        return result


def make_handler(service: CameraService):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            if parsed.path == "/health":
                self._send_json({"ok": True})
                return
            if parsed.path != "/capture":
                self._send_json({"ok": False, "error": "not_found"}, status=404)
                return

            query = parse_qs(parsed.query)
            ip = (query.get("ip") or [""])[0]
            if not ip:
                self._send_json({"ok": False, "error": "missing ip"}, status=400)
                return
            image_id = None
            if query.get("image_id"):
                try:
                    image_id = int(query["image_id"][0])
                except ValueError:
                    self._send_json({"ok": False, "error": "invalid image_id"}, status=400)
                    return

            try:
                result = service.capture(ip, image_id=image_id)
            except Exception as exc:
                self._send_json(
                    {"ok": False, "error": type(exc).__name__, "message": str(exc)},
                    status=500,
                )
                return
            self._send_json({"ok": True, "result": result})

        def log_message(self, format: str, *args: Any) -> None:
            print(f"[HTTP] {self.address_string()} - {format % args}")

        def _send_json(self, payload: dict[str, Any], status: int = 200) -> None:
            data = (json.dumps(payload, ensure_ascii=False, default=str) + "\n").encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description="Persistent camera and YOLO ring-detection service.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5060)
    parser.add_argument(
        "--output-dir",
        default=str(REPO_ROOT / "artifacts" / "camera_test_data"),
    )
    parser.add_argument("--epoch", type=int, default=3)
    parser.add_argument("--no-save-ply", dest="save_ply", action="store_false")
    parser.add_argument("--only-outer", action="store_true")
    parser.add_argument("--no-warmup", dest="warmup", action="store_false")
    args = parser.parse_args()

    print("正在初始化相机常驻服务...")
    service = CameraService(
        save_ply=args.save_ply,
        only_outer=args.only_outer,
        output_dir=args.output_dir,
        epoch=args.epoch,
        warmup=args.warmup,
    )
    server = ThreadingHTTPServer((args.host, args.port), make_handler(service))
    print(f"camera_service listening on http://{args.host}:{args.port}")
    print("capture endpoint: /capture?ip=192.168.1.66")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nCtrl+C received, stopping camera_service.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
