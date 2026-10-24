#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
"""Browser controller backed by Go2 ROS 2 interfaces."""

from __future__ import annotations

import asyncio
from concurrent.futures import TimeoutError as FutureTimeoutError
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from http.client import HTTPConnection, HTTPException
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import threading
import time
from typing import Any
from urllib.parse import urlsplit

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy
from unitree_go2.srv import SetMode
from go2_network import APPLICATION_ID, RobotNetwork, valid_robot_id
from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription

MODES = {
    "passive": "passive",
    "passive_direct": "passive_direct",
    "crouch": "crouch",
    "sit": "sit",
    "forward_jump": "forward_jump",
    "stand": "stand",
    "walk": "walk",
    "wave": "wave",
    "terrain": "terrain",
    "rollover": "rollover",
    "navigation": "walk",
    "ai_agent": "walk",
}
# Accept the largest command envelope used by the current policies.  The
# browser chooses the mode-specific envelope, and the controller applies the
# final policy clamp again before constructing an observation.
AXIS_LIMITS = (3.00, 1.00, 2.50)
COMMAND_TIMEOUT = 0.30


class CommandError(ValueError):
    pass


def finite(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CommandError("axes must contain numbers")
    result = float(value)
    if not math.isfinite(result):
        raise CommandError("axes must be finite")
    return result


def parse_axes(payload: Any) -> tuple[float, float, float]:
    if not isinstance(payload, dict) or not isinstance(payload.get("axes"), list) or len(payload["axes"]) != 3:
        raise CommandError("axes must contain three numbers")
    values = [finite(item) for item in payload["axes"]]
    return tuple(max(-limit, min(limit, value)) for value, limit in zip(values, AXIS_LIMITS))


class WebControl:
    """A dedicated event loop keeps joystick reception independent of HTTP replies."""

    def __init__(self, node) -> None:
        self.node = node
        self.loop = asyncio.new_event_loop()
        self.peers: set[RTCPeerConnection] = set()
        self.owner = None
        self.thread = threading.Thread(target=self.loop.run_forever, name="go2-webrtc", daemon=True)
        self.thread.start()

    def answer(self, payload: Any) -> dict:
        if (not isinstance(payload, dict) or payload.get("type") != "offer"
                or not isinstance(payload.get("sdp"), str) or len(payload["sdp"]) > 60000):
            raise CommandError("invalid WebRTC offer")
        future = asyncio.run_coroutine_threadsafe(self._answer(payload), self.loop)
        try:
            return future.result(timeout=8)
        except FutureTimeoutError as exc:
            future.cancel()
            raise CommandError("WebRTC negotiation timed out") from exc
        except (ValueError, TypeError) as exc:
            raise CommandError(f"invalid WebRTC offer: {exc}") from exc

    async def _answer(self, payload: dict) -> dict:
        if len(self.peers) >= 8:
            raise CommandError("too many WebRTC sessions")
        peer = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        self.peers.add(peer)
        received_at = self.loop.time()

        async def release() -> None:
            self.peers.discard(peer)
            if self.owner is peer:
                self.owner = None
                self.node.set_velocity((0.0, 0.0, 0.0))
            await peer.close()

        @peer.on("connectionstatechange")
        async def state_changed() -> None:
            if peer.connectionState == "failed":
                await release()
                return
            if peer.connectionState in ("failed", "closed"):
                self.peers.discard(peer)
                if self.owner is peer:
                    self.owner = None
                    self.node.set_velocity((0.0, 0.0, 0.0))

        @peer.on("datachannel")
        def channel_received(channel) -> None:
            if channel.label != "go2-velocity":
                channel.close()
                return
            sequence = -1

            @channel.on("message")
            def message_received(raw) -> None:
                nonlocal sequence, received_at
                try:
                    if not isinstance(raw, str) or len(raw) > 1024:
                        raise CommandError("invalid joystick packet")
                    packet = json.loads(raw)
                    axes = parse_axes(packet)
                    current = packet.get("seq")
                    if isinstance(current, bool) or not isinstance(current, int) or current < 0:
                        raise CommandError("invalid joystick sequence")
                    if current <= sequence:
                        return  # Unordered packets must never restore an older stick position.
                    sequence = current
                    received_at = self.loop.time()
                    ready = self.node.controller_ready()
                    with self.node._lock:
                        source = self.node._lights["control_source"]
                    if ready and (not self.node.shared_control or source != "sbus"):
                        self.owner = peer
                        self.node.set_velocity(axes)
                    if channel.readyState == "open" and channel.bufferedAmount < 4096:
                        channel.send(json.dumps({"seq": sequence, "connected": ready,
                                                 "control_source": source}))
                except (ValueError, TypeError) as exc:
                    self.node.get_logger().warning(f"Invalid WebRTC command: {exc}")

        async def expire() -> None:
            try:
                while peer in self.peers:
                    await asyncio.sleep(1)
                    if self.loop.time() - received_at > 10:
                        await release()
            finally:
                if peer in self.peers:
                    await release()

        try:
            await peer.setRemoteDescription(RTCSessionDescription(sdp=payload["sdp"], type="offer"))
            await peer.setLocalDescription(await peer.createAnswer())
            self.loop.create_task(expire())
            return {"type": peer.localDescription.type, "sdp": peer.localDescription.sdp}
        except BaseException:
            await release()
            raise

    def close(self) -> None:
        async def shutdown() -> None:
            for peer in tuple(self.peers):
                await peer.close()
            self.peers.clear()
            tasks = [task for task in asyncio.all_tasks() if task is not asyncio.current_task()]
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

        try:
            asyncio.run_coroutine_threadsafe(shutdown(), self.loop).result(timeout=5)
        finally:
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.thread.join(timeout=5)
            self.loop.close()


class Camera:
    """One hardware encoder publishes RTSP; MediaMTX serves WebRTC to all viewers."""

    def __init__(self, device: str = "auto", width: int = 640, height: int = 360, fps: int = 30) -> None:
        self.device, self.width, self.height, self.fps = device, width, height, max(1, fps)
        self._active_device = None
        self._processes = []
        self._logs = []
        self._fault = ""
        self._lock = threading.Lock()
        self._watch_stop = threading.Event()

    def _candidates(self) -> list[str]:
        paths = ([Path(self.device)] if self.device != "auto" else
                 sorted(Path("/dev/v4l/by-id").glob("*video-index0")) + sorted(Path("/dev").glob("video*")))
        candidates, seen = [], set()
        for path in paths:
            device = path.resolve()
            if device in seen:
                continue
            seen.add(device)
            info = Path("/sys/class/video4linux") / device.name
            try:
                if (info / "index").read_text().strip() != "0":
                    continue
                for usb in (info / "device").resolve().parents:
                    if (usb / "idVendor").is_file():
                        # Only this RGB camera is used by the web view. Never
                        # fall back to another camera or the board video codec.
                        if ((usb / "idVendor").read_text().strip().lower() == "1bcf" and
                                (usb / "idProduct").read_text().strip().lower() == "28c4"):
                            candidates.append(str(path))
                        break
            except OSError:
                continue
        return candidates

    def _ready(self) -> bool:
        connection = HTTPConnection("127.0.0.1", 9997, timeout=0.2)
        try:
            connection.request("GET", "/v3/paths/get/camera")
            response = connection.getresponse()
            return response.status == 200 and json.loads(response.read(65536)).get("ready") is True
        except (OSError, HTTPException, ValueError):
            return False
        finally:
            connection.close()

    def _stop(self) -> None:
        self._watch_stop.set()
        for process in reversed(self._processes):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=1)
        self._processes.clear()
        for log in self._logs:
            log.close()
        self._logs.clear()
        self._active_device = None

    def _watch_viewers(self, stop: threading.Event) -> None:
        last_viewer = time.monotonic()
        while not stop.wait(1.0):
            connection = HTTPConnection("127.0.0.1", 9997, timeout=0.5)
            try:
                connection.request("GET", "/v3/paths/get/camera")
                response = connection.getresponse()
                readers = json.loads(response.read(65536)).get("readers", []) if response.status == 200 else []
            except (OSError, HTTPException, ValueError):
                continue
            finally:
                connection.close()
            with self._lock:
                # An earlier monitor must never stop a newly opened camera.
                if stop is not self._watch_stop or stop.is_set():
                    return
                if readers:
                    last_viewer = time.monotonic()
                elif time.monotonic() - last_viewer >= 15:
                    # Allow WebRTC startup/reconnection, then release capture
                    # when the browser has gone away or switched robots.
                    self._stop()
                    return

    def _failure(self, message: str) -> None:
        details = []
        for log in self._logs:
            log.seek(max(0, os.fstat(log.fileno()).st_size - 1200))
            details.append(log.read().decode(errors="replace").strip())
        self._fault = message + ("：" + "\n".join(filter(None, details)) if any(details) else "")
        self._stop()

    def _start(self, arguments: list[str], env: dict[str, str] | None = None) -> None:
        log = tempfile.TemporaryFile()
        self._logs.append(log)
        self._processes.append(subprocess.Popen(
            arguments, stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True, env=env,
        ))

    def set_enabled(self, enabled: bool) -> bool:
        with self._lock:
            if not enabled:
                self._stop()
                self._fault = ""
                return False
            if len(self._processes) == 2 and all(p.poll() is None for p in self._processes):
                return True
            self._stop()
            self._fault = ""
            candidates = self._candidates()
            if not candidates:
                self._fault = "未识别到 USB 摄像头（1bcf:28c4），请检查 USB 接线和供电"
                return False
            if not shutil.which("mediamtx") or not shutil.which("ffmpeg"):
                self._fault = "摄像头预览依赖未安装：需要 MediaMTX 和 FFmpeg"
                return False
            try:
                # MediaMTX v1.20.1: keep preview settings inside this child process.
                # /dev/null skips config discovery; also discard legacy RTSP_ overrides.
                env = {key: value for key, value in os.environ.items()
                       if not key.startswith(("MTX_", "RTSP_"))}
                env.update({
                    "MTX_LOGLEVEL": "warn", "MTX_LOGDESTINATIONS": "stdout",
                    "MTX_API": "true", "MTX_APIADDRESS": "127.0.0.1:9997",
                    "MTX_METRICS": "false", "MTX_PPROF": "false", "MTX_PLAYBACK": "false",
                    "MTX_RTSP": "true", "MTX_RTSPADDRESS": "127.0.0.1:18554",
                    "MTX_RTSPTRANSPORTS": "tcp", "MTX_RTMP": "false", "MTX_HLS": "false",
                    "MTX_SRT": "false", "MTX_MOQ": "false",
                    "MTX_WEBRTC": "true", "MTX_WEBRTCADDRESS": ":8889",
                    "MTX_WEBRTCENCRYPTION": "false", "MTX_WEBRTCALLOWORIGINS": "*",
                    "MTX_WEBRTCLOCALUDPADDRESS": ":8189", "MTX_WEBRTCLOCALTCPADDRESS": "",
                    "MTX_WEBRTCIPSFROMINTERFACES": "true", "MTX_WEBRTCADDITIONALHOSTS": "",
                    "MTX_WEBRTCICESERVERS2": "", "MTX_PATHS_CAMERA_SOURCE": "publisher",
                })
                self._start(["mediamtx", "/dev/null"], env=env)
                # Wait for this server to bind before starting its publisher.
                deadline = time.monotonic() + 2
                while time.monotonic() < deadline:
                    if self._processes[0].poll() is not None:
                        self._failure("MediaMTX 启动失败")
                        return False
                    connection = HTTPConnection("127.0.0.1", 9997, timeout=0.1)
                    try:
                        connection.request("GET", "/v3/paths/list")
                        if connection.getresponse().status == 200:
                            break
                    except (OSError, HTTPException):
                        time.sleep(0.05)
                    finally:
                        connection.close()
                else:
                    self._failure("MediaMTX 启动超时")
                    return False
                self._active_device = candidates[0]
                self._start([
                    "ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin",
                    "-f", "v4l2", "-input_format", "mjpeg", "-framerate", str(self.fps),
                    "-video_size", f"{self.width}x{self.height}", "-i", self._active_device,
                    "-an", "-vf", "format=nv12", "-c:v", "h264_stcodec", "-b:v", "1500k",
                    "-g", str(max(1, self.fps // 2)), "-bf", "0", "-flush_packets", "1",
                    "-f", "rtsp", "-rtsp_transport", "tcp", "rtsp://127.0.0.1:18554/camera",
                ])
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    if any(p.poll() is not None for p in self._processes):
                        break
                    if self._ready():
                        self._watch_stop = threading.Event()
                        threading.Thread(target=self._watch_viewers, args=(self._watch_stop,), daemon=True).start()
                        return True
                    time.sleep(0.05)
                self._failure("摄像头未能输出 H.264 画面")
            except OSError as exc:
                self._failure(f"摄像头启动失败：{exc}")
            return False

    def status(self) -> dict[str, Any]:
        with self._lock:
            if self._processes and any(p.poll() is not None for p in self._processes):
                self._failure("摄像头预览进程已退出")
            return {
                "enabled": len(self._processes) == 2,
                "device": self._active_device or self.device,
                "model": "Tstc USB Full HD Camera",
                "width": self.width, "height": self.height, "fps": self.fps,
                "transport": "webrtc", "webrtc_port": 8889,
                "message": self._fault,
            }

    def close(self) -> None:
        self.set_enabled(False)


class Go2WebNode(Node):
    def __init__(self) -> None:
        super().__init__("go2_web")
        share = Path(get_package_share_directory("unitree_go2"))
        self.robot = str(self.declare_parameter("robot", "go2").value)
        if not valid_robot_id(self.robot):
            raise ValueError("unknown robot: " + self.robot)
        root = str(self.declare_parameter("web_root", str(share / "web")).value)
        port = int(self.declare_parameter("web_port", 8082).value)
        self.web_root = Path(root)
        self.shared_control = bool(self.declare_parameter("shared_control", False).value)
        self._lock = threading.Lock()
        self._velocity = (0.0, 0.0, 0.0)
        self._velocity_at = 0.0
        self._lights = {
            "white_light_enabled": False,
            "white_light_available": False,
            "status_light": "",
            "light_fault": "",
            "control_source": "web",
            "sbus_connected": False,
            "sbus_slot": 0,
        }
        self._last_mode: str | None = None
        self._status_received_at = 0.0
        self._last_result: dict[str, Any] = {"accepted": None, "message": ""}
        self._navigation_enabled = False
        self._ai_agent_enabled = False
        self.light_client = self.create_client(Trigger, "/go2/toggle_white_light")
        self.alive_pub = self.create_publisher(Bool, "/go2/web_alive", 10)
        self.status_sub = self.create_subscription(
            String, "/go2/status", self.update_lights, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.mode_client = self.create_client(
            SetMode, "/go2/set_mode_web" if self.shared_control else "/go2/set_mode"
        )
        self.velocity_pub = self.create_publisher(
            Twist, "/cmd_vel_web" if self.shared_control else "/cmd_vel_direct", 10
        )
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.camera = Camera(
            str(self.declare_parameter("camera_device", "auto").value),
            int(self.declare_parameter("camera_width", 640).value),
            int(self.declare_parameter("camera_height", 360).value),
            int(self.declare_parameter("camera_fps", 30).value),
        )
        self.http = ThreadingHTTPServer(("0.0.0.0", port), self.handler())
        self.http.daemon_threads = True
        try:
            self.network = RobotNetwork(
                self.robot, str(self.declare_parameter("network_mode", "wifi").value), port,
                str(self.declare_parameter("ap_address", "10.0.91.250/23").value),
                str(self.declare_parameter("wifi_connection", "").value),
                str(self.declare_parameter("ap_password", "go2control").value),
                self.controller_ready, self.get_logger(),
                str(self.declare_parameter("ap_band", "a").value),
                int(self.declare_parameter("ap_channel", 0).value),
                str(self.declare_parameter("robot_name", self.robot).value),
                str(self.declare_parameter("ap_ssid", "").value),
            )
        except BaseException:
            self.http.server_close()
            raise
        self.http_thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        try:
            self.control = WebControl(self)
        except BaseException:
            self.http.server_close()
            self.network.close()
            raise
        self.http_thread.start()
        self.network.start()
        self.get_logger().info(f"Go2 web controller listening on port {port}")

    def publish_velocity(self, force: bool = False) -> None:
        with self._lock:
            age = time.monotonic() - self._velocity_at if self._velocity_at else None
            velocity = self._velocity if age is not None and age <= COMMAND_TIMEOUT else (0.0, 0.0, 0.0)
            navigation = self._navigation_enabled
            ai_agent = self._ai_agent_enabled
            web_control = not self.shared_control or self._lights["control_source"] != "sbus"
        self.alive_pub.publish(Bool(data=age is not None and age <= COMMAND_TIMEOUT))
        # During navigation / AI-agent, go2_node forwards the external
        # `/cmd_vel` / `/cmd_vel_ai` stream to `/cmd_vel_direct`.  Suppress the
        # web joystick stream so it cannot race the external command on the
        # robot input topic.
        if not web_control or ((navigation or ai_agent) and not force):
            return
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = velocity
        self.velocity_pub.publish(message)

    def update_lights(self, message: String) -> None:
        try:
            state = json.loads(message.data)
        except (ValueError, TypeError):
            return
        with self._lock:
            self._status_received_at = time.monotonic()
            previous_control = self._lights["control_source"]
            for key in self._lights:
                if key in state:
                    self._lights[key] = state[key]
            if self.shared_control and previous_control != self._lights["control_source"]:
                self._velocity, self._velocity_at = (0.0, 0.0, 0.0), 0.0
            self._navigation_enabled = bool(state.get("navigation_enabled", False))
            self._ai_agent_enabled = bool(state.get("ai_agent_enabled", False))

    def toggle_light(self) -> dict[str, Any]:
        if not self.light_client.service_is_ready():
            return {"success": False, "message": "white light service unavailable"}
        future = self.light_client.call_async(Trigger.Request())
        event = threading.Event()
        result = {"success": False, "message": "white light service timed out"}

        def done(completed: Any) -> None:
            try:
                response = completed.result()
                result.update(success=bool(response.success), message=str(response.message))
                if response.success:
                    result["white_light_enabled"] = response.message == "on"
                    with self._lock:
                        self._lights["white_light_enabled"] = result["white_light_enabled"]
            except Exception as exc:
                result["message"] = str(exc)
            event.set()

        future.add_done_callback(done)
        event.wait(timeout=3.0)
        return result

    def set_velocity(self, velocity: tuple[float, float, float]) -> None:
        with self._lock:
            web_control = not self.shared_control or self._lights["control_source"] != "sbus"
            self._velocity = velocity if web_control else (0., 0., 0.)
            self._velocity_at = time.monotonic()

    def stop(self) -> None:
        self.set_velocity((0.0, 0.0, 0.0))
        self.publish_velocity(force=True)

    def set_mode(self, requested: str) -> dict[str, Any]:
        if requested not in MODES:
            raise CommandError("unknown mode")
        with self._lock:
            if self.shared_control and self._lights["control_source"] == "sbus":
                return {"accepted": False, "message": "遥控器控制中；切到4号位或关闭遥控器后使用网页"}
        mode = MODES[requested]
        if not self.mode_client.wait_for_service(timeout_sec=0.2):
            result = {"accepted": False, "message": "/go2/set_mode service is not available"}
        else:
            request = SetMode.Request()
            # Keep the navigation / AI-agent mode name across the service
            # boundary.  The controller maps it to the flat ONNX policy, while
            # go2_node uses the name to enable `/cmd_vel` (or `/cmd_vel_ai`)
            # -> `/cmd_vel_direct` forwarding.
            request.mode = requested if requested in ("navigation", "ai_agent") else mode
            future = self.mode_client.call_async(request)
            event = threading.Event()
            result = {"accepted": False, "message": "service call timed out"}

            def done(completed: Any) -> None:
                try:
                    response = completed.result()
                    result.update(accepted=bool(response.accepted), message=str(response.message))
                except Exception as exc:
                    result["message"] = str(exc)
                event.set()

            future.add_done_callback(done)
            event.wait(timeout=3.0)
        with self._lock:
            self._last_mode, self._last_result = requested, result.copy()
            if result.get("accepted"):
                self._navigation_enabled = requested == "navigation"
                self._ai_agent_enabled = requested == "ai_agent"
        if mode in ("passive", "passive_direct", "crouch", "stand") or requested in ("navigation", "ai_agent"):
            self.stop()
        return result

    def controller_ready(self) -> bool:
        return self.mode_client.service_is_ready() and time.monotonic() - self._status_received_at < 3.0

    def status(self) -> dict[str, Any]:
        # Process cleanup must not block joystick reception on the velocity lock.
        camera = {**self.camera.status(), "webrtc_host": self.network.address}
        with self._lock:
            age = None if not self._velocity_at else round((time.monotonic() - self._velocity_at) * 1000)
            velocity = self._velocity if age is not None and age <= COMMAND_TIMEOUT * 1000 else (0.0, 0.0, 0.0)
            ready = self.controller_ready()
            return {
                **self._lights,
                "application": APPLICATION_ID,
                "robot": self.robot,
                "network_mode": self.network.mode,
                "connected": ready,
                "service_available": ready,
                "axes": list(velocity),
                "velocity": list(velocity),
                "last_mode": self._last_mode,
                "last_accepted": self._last_result["accepted"],
                "last_message": self._last_result["message"],
                "camera": camera,
                "control_transport": "webrtc",
                "navigation_enabled": self._navigation_enabled,
                "ai_agent_enabled": self._ai_agent_enabled,
            }

    def handler(self):
        node = self
        static = {
            "/": ("index.html", "text/html; charset=utf-8"),
            "/index.html": ("index.html", "text/html; charset=utf-8"),
            "/app.js": ("app.js", "text/javascript; charset=utf-8"),
            "/styles.css": ("styles.css", "text/css; charset=utf-8"),
            "/manifest.webmanifest": ("manifest.webmanifest", "application/manifest+json; charset=utf-8"),
            "/icon.svg": ("icon.svg", "image/svg+xml"),
        }

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, fmt: str, *args: Any) -> None:
                node.get_logger().debug(f"web {fmt % args}")

            def send_json(self, status: int, value: Any) -> None:
                data = json.dumps(value, ensure_ascii=False).encode()
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                if self.command == "POST" and not self._body_read:
                    # An early rejection leaves unread bytes on this connection.
                    # Close it so they cannot become the next HTTP request.
                    self.close_connection = True
                    self.send_header("Connection", "close")
                self.end_headers()
                self.wfile.write(data)

            def body(self) -> Any:
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                except ValueError as exc:
                    raise CommandError("invalid body size") from exc
                if length < 1 or length > 65536:
                    raise CommandError("invalid body size")
                data = self.rfile.read(length)
                self._body_read = len(data) == length
                try:
                    return json.loads(data.decode())
                except (ValueError, UnicodeDecodeError) as exc:
                    raise CommandError("request body must be valid JSON") from exc

            def robot_route(self) -> str | None:
                if self.command == "POST" and self.headers.get("X-Go2-Application") != APPLICATION_ID:
                    self.send_json(409, {"error": "this control endpoint requires the Go2 SDK client"})
                    return None
                expected = self.headers.get("X-Go2-Robot")
                if expected and expected != node.robot:
                    self.send_json(409, {"error": "robot identity changed; refresh device status"})
                    return None
                parsed = urlsplit(self.path)
                path = parsed.path
                if not path.startswith("/api/robots/"):
                    return path
                parts = path.split("/", 4)
                if len(parts) != 5 or not valid_robot_id(parts[3]):
                    self.send_json(404, {"error": "unknown robot"})
                    return None
                robot, suffix = parts[3:]
                allowed = {
                    "status", "command", "stop", "light/toggle", "rtc/offer", "mode",
                    "camera/status", "camera/toggle",
                }
                if suffix not in allowed:
                    self.send_json(404, {"error": "unknown robot API"})
                    return None
                route = "/api/" + suffix
                if robot == node.robot:
                    return route
                endpoint = node.network.endpoint(robot)
                if endpoint is None:
                    self.send_json(503, {"error": "robot is not connected to this network"})
                    return None
                connection = HTTPConnection(*endpoint, timeout=8 if suffix in {"rtc/offer", "camera/toggle"} else 4)
                headers = {"X-Go2-Robot": robot, "X-Go2-Application": APPLICATION_ID,
                           "Content-Type": "application/json"}
                headers_sent = False
                try:
                    body = json.dumps(self.body()).encode() if self.command == "POST" else None
                    query = "?" + parsed.query if parsed.query else ""
                    connection.request(self.command, route + query, body=body, headers=headers)
                    response = connection.getresponse()
                    content_type = response.getheader("Content-Type", "application/json")
                    data = response.read(65536)
                    self.send_response(response.status)
                    self.send_header("Content-Type", content_type)
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    headers_sent = True
                    self.wfile.write(data)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                except (OSError, HTTPException) as exc:
                    if not headers_sent:
                        self.send_json(503, {"error": str(exc)})
                finally:
                    connection.close()
                return None

            def do_GET(self) -> None:  # noqa: N802
                path = self.robot_route()
                if path is None:
                    return
                if path == "/api/robots":
                    self.send_json(200, {"application": APPLICATION_ID, "local_robot": node.robot,
                                         "robots": node.network.devices()})
                    return
                if path == "/api/status":
                    self.send_json(200, node.status())
                    return
                if path == "/api/camera/status":
                    self.send_json(200, node.camera.status())
                    return
                if path not in static:
                    self.send_json(404, {"error": "not found"})
                    return
                filename, content_type = static[path]
                try:
                    data = (node.web_root / filename).read_bytes()
                except OSError:
                    self.send_json(404, {"error": "web asset unavailable"})
                    return
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(data)))
                # The web controller is deployed by rebuilding the ROS package.
                # Do not let a phone or desktop keep an older joystick script.
                self.send_header("Cache-Control", "no-store, max-age=0")
                self.send_header("Pragma", "no-cache")
                self.end_headers()
                self.wfile.write(data)

            def do_POST(self) -> None:  # noqa: N802
                self._body_read = False
                try:
                    path = self.robot_route()
                    if path is None:
                        return
                    if path != "/api/camera/toggle" and not node.controller_ready():
                        self.send_json(503, {"error": "robot controller is not running"})
                        return
                    payload = self.body()
                    if path == "/api/rtc/offer":
                        self.send_json(200, node.control.answer(payload))
                        return
                    if path == "/api/mode":
                        if not isinstance(payload, dict) or not isinstance(payload.get("mode"), str):
                            raise CommandError("mode must be a string")
                        self.send_json(200, node.set_mode(payload["mode"]))
                        return
                    if path == "/api/command":
                        axes = parse_axes(payload)
                        with node._lock:
                            rc_control = node.shared_control and node._lights["control_source"] == "sbus"
                        if rc_control:
                            node.set_velocity((0.0, 0.0, 0.0))
                            self.send_json(200, {"accepted": False, "axes": [0., 0., 0.],
                                "message": "遥控器控制中；切到4号位或关闭遥控器后使用网页"})
                            return
                        result = {"accepted": True, "message": "velocity updated"}
                        if payload.get("mode") is not None:
                            result = node.set_mode(str(payload["mode"]))
                        node.set_velocity(axes)
                        result["axes"] = list(axes)
                        self.send_json(200, result)
                        return
                    if path == "/api/light/toggle":
                        result = node.toggle_light()
                        self.send_json(200 if result["success"] else 503, result)
                        return
                    if path == "/api/camera/toggle":
                        if not isinstance(payload, dict) or not isinstance(payload.get("enabled"), bool):
                            raise CommandError("enabled must be a boolean")
                        if payload["enabled"] and not node.controller_ready():
                            self.send_json(503, {"error": "robot controller is not running"})
                            return
                        enabled = node.camera.set_enabled(payload["enabled"])
                        if payload["enabled"] and not enabled:
                            self.send_json(503, {"error": node.camera.status()["message"]})
                            return
                        self.send_json(200, {**node.camera.status(), "webrtc_host": node.network.address})
                        return
                    if path == "/api/stop":
                        node.stop()
                        self.send_json(200, {"ok": True})
                        return
                    self.send_json(404, {"error": "not found"})
                except CommandError as exc:
                    self.send_json(400, {"error": str(exc)})

            def do_OPTIONS(self) -> None:  # noqa: N802
                self.send_response(204)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type,X-Go2-Application,X-Go2-Robot")
                self.send_header("Content-Length", "0")
                self.end_headers()

        return Handler

    def destroy_node(self) -> bool:
        try:
            self.control.close()
            if rclpy.ok():
                self.stop()
            self.camera.close()
            self.http.shutdown()
            self.http.server_close()
        finally:
            # Ctrl+C can shut the ROS context down before this method runs.
            # Always withdraw the shared name and restore the Wi-Fi profile.
            self.network.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Go2WebNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
