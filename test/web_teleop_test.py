# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
"""隔离验证网页控制、摄像头预览与网络请求，不启动 ROS 或硬件。"""

import ast
from http.client import HTTPConnection, HTTPException
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import os
from pathlib import Path
import runpy
import shutil
import signal
import subprocess
import threading
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from urllib.parse import urlsplit


source = Path(__file__).resolve().parents[1] / "scripts/web_teleop_server.py"
tree = ast.parse(source.read_text())
web_node = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == "Go2WebNode")
with patch.dict("sys.modules", {"dbus": Mock()}):
    network_module = runpy.run_path(str(source.with_name("go2_network.py")))
namespace = {"BaseHTTPRequestHandler": BaseHTTPRequestHandler, "json": json,
             "urlsplit": urlsplit, "CommandError": ValueError, "time": time, "Any": object,
             "HTTPConnection": HTTPConnection, "HTTPException": HTTPException,
             "APPLICATION_ID": "unitree-go2-sdk", "valid_robot_id": network_module["valid_robot_id"]}
namespace["MODES"] = ast.literal_eval(next(item.value for item in tree.body
    if isinstance(item, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "MODES"
                                           for target in item.targets)))
namespace.update({"Path": Path, "os": os, "shutil": shutil, "signal": signal,
                  "subprocess": subprocess, "threading": threading, "tempfile": tempfile})
camera_class = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == "Camera")
exec(compile(ast.Module(body=[camera_class], type_ignores=[]), str(source), "exec"), namespace)
Camera = namespace["Camera"]
for name in ("handler", "set_mode"):
    method = next(item for item in web_node.body if isinstance(item, ast.FunctionDef) and item.name == name)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)


class CameraTests(unittest.TestCase):
    def test_missing_usb_camera_reports_cause_without_starting_processes(self):
        camera = Camera()
        with patch.object(camera, "_candidates", return_value=[]), patch.object(subprocess, "Popen") as start:
            self.assertFalse(camera.set_enabled(True))
            self.assertIn("未识别到 USB 摄像头", camera.status()["message"])
            start.assert_not_called()
        camera.close()
        self.assertFalse(camera.status()["enabled"])

    def test_missing_runtime_reports_dependencies(self):
        camera = Camera()
        with patch.object(camera, "_candidates", return_value=["/dev/video1"]), \
                patch.object(shutil, "which", return_value=None), patch.object(subprocess, "Popen") as start:
            self.assertFalse(camera.set_enabled(True))
            self.assertIn("MediaMTX", camera.status()["message"])
            start.assert_not_called()

    def test_publisher_failure_stops_server_and_keeps_diagnostic(self):
        camera = Camera()
        publisher = Mock(pid=999999, returncode=1)
        publisher.poll.return_value = 1
        server = Mock(pid=999998)
        server.poll.return_value = None
        camera._processes = [server, publisher]
        log = namespace["tempfile"].TemporaryFile()
        log.write(b"USB disconnected")
        log.flush()
        camera._logs = [log]
        with patch.object(os, "killpg") as kill:
            state = camera.status()
            self.assertFalse(state["enabled"])
            self.assertIn("USB disconnected", state["message"])
            kill.assert_called_once_with(server.pid, signal.SIGTERM)
            server.wait.assert_called_once_with(timeout=1)
        self.assertTrue(log.closed)
        self.assertEqual(camera._processes, [])

    def test_preview_publisher_matches_server_without_navigation_rtsp_port(self):
        camera = Camera()

        def start(*_args, **_kwargs):
            process = Mock(pid=999999)
            process.poll.return_value = None
            return process

        connection = Mock()
        connection.getresponse.return_value.status = 200
        inherited = {
            "MTX_RTSPADDRESS": ":8554", "MTX_PATHS_UNWANTED_SOURCE": "publisher",
            "MTX_RUNONCONNECT": "unexpected-command", "RTSP_CONFKEY": "unrelated-key",
            "RTSP_RTSPADDRESS": ":8555", "GO2_PREVIEW_TEST_MARKER": "preserved",
        }
        with patch.object(camera, "_candidates", return_value=["/dev/video3"]), \
                patch.object(shutil, "which", return_value="/usr/bin/executable"), \
                patch.object(subprocess, "Popen", side_effect=start) as popen, \
                patch.object(camera, "_ready", return_value=True), \
                patch.dict(namespace, {"HTTPConnection": Mock(return_value=connection)}), \
                patch.dict(os.environ, inherited), patch.object(os, "killpg"), \
                patch.object(threading, "Thread") as thread:
            try:
                for cycle in range(2):
                    previous_stop = camera._watch_stop
                    self.assertTrue(camera.set_enabled(True))
                    self.assertTrue(camera.set_enabled(True))
                    self.assertEqual(popen.call_count, 2 * (cycle + 1))
                    self.assertEqual(thread.call_count, cycle + 1)
                    self.assertIsNot(camera._watch_stop, previous_stop)
                    server, publisher = popen.call_args_list[-2:]
                    self.assertEqual(server.args[0], ["mediamtx", "/dev/null"])
                    env = server.kwargs["env"]
                    self.assertEqual(env["MTX_APIADDRESS"], "127.0.0.1:9997")
                    self.assertEqual(env["MTX_WEBRTCADDRESS"], ":8889")
                    self.assertEqual(env["MTX_WEBRTCLOCALUDPADDRESS"], ":8189")
                    self.assertEqual(env["MTX_RTSPTRANSPORTS"], "tcp")
                    self.assertEqual(env["MTX_PATHS_CAMERA_SOURCE"], "publisher")
                    self.assertEqual(env["GO2_PREVIEW_TEST_MARKER"], "preserved")
                    for key, value in inherited.items():
                        self.assertEqual(os.environ[key], value)
                        if key not in ("MTX_RTSPADDRESS", "GO2_PREVIEW_TEST_MARKER"):
                            self.assertNotIn(key, env)
                    destination = urlsplit(publisher.args[0][-1])
                    self.assertEqual(destination.netloc, env["MTX_RTSPADDRESS"])
                    self.assertEqual(destination.port, 18554)
                    self.assertEqual(destination.path, "/camera")
                    self.assertIn("h264_stcodec", publisher.args[0])
                    logs, processes = list(camera._logs), list(camera._processes)
                    self.assertFalse(camera.set_enabled(False))
                    self.assertTrue(camera._watch_stop.is_set())
                    self.assertTrue(all(log.closed for log in logs))
                    for process in processes:
                        process.wait.assert_called_once_with(timeout=1)
                    self.assertEqual(camera._processes, [])
                    self.assertEqual(camera._logs, [])
                    self.assertIsNone(camera._active_device)
            finally:
                camera.close()

    def test_preview_stops_after_browser_leaves_or_never_connects(self):
        for readers, clock in (([[], []], [0, 5, 16]), ([[{"type": "webRTCSession"}], []], [0, 10, 26])):
            with self.subTest(readers=readers):
                camera = Camera()
                process = Mock(pid=999999)
                process.poll.return_value = None
                camera._processes = [process, process]
                connection = Mock()
                connection.getresponse.return_value.status = 200
                connection.getresponse.return_value.read.side_effect = [
                    json.dumps({"readers": value}).encode() for value in readers
                ]
                with patch.dict(namespace, {"HTTPConnection": Mock(return_value=connection)}), \
                        patch.object(time, "monotonic", side_effect=clock), \
                        patch.object(camera._watch_stop, "wait", return_value=False), \
                        patch.object(os, "killpg"):
                    camera._watch_viewers(camera._watch_stop)
                self.assertFalse(camera.status()["enabled"])
                self.assertTrue(camera._watch_stop.is_set())

    def test_previous_monitor_cannot_stop_reopened_camera(self):
        camera = Camera()
        previous = camera._watch_stop
        process = Mock()
        process.poll.return_value = None
        camera._processes = [process, process]
        connection = Mock()
        response = Mock(status=200)
        response.read.return_value = b'{"readers": []}'

        def reopens():
            camera._watch_stop = threading.Event()
            return response

        connection.getresponse.side_effect = reopens
        with patch.dict(namespace, {"HTTPConnection": Mock(return_value=connection)}), \
                patch.object(previous, "wait", return_value=False), \
                patch.object(time, "monotonic", side_effect=[0, 100]), \
                patch.object(os, "killpg") as kill:
            camera._watch_viewers(previous)
        kill.assert_not_called()
        self.assertTrue(camera.status()["enabled"])


class WifiStartupTests(unittest.TestCase):
    def test_ap_name_updates_new_and_existing_profiles(self):
        network_type = network_module["RobotNetwork"]
        globals_ = network_type.__init__.__globals__
        for existing in (False, True):
            with self.subTest(existing=existing):
                state = {"ssid": "OldHotspot"}

                def command(*args):
                    if args[1:4] == ("-t", "-f", "DEVICE,TYPE"):
                        return "wlan0:wifi"
                    if args[1:3] == ("-g", "GENERAL.CON-UUID"):
                        return "wifi-uuid"
                    if args[1:3] == ("-g", "NAME"):
                        return "go2-sdk-ap-robot-a" if existing else ""
                    if args[1:3] == ("-g", "connection.uuid"):
                        return "ap-uuid"
                    if args[1:3] == ("connection", "modify") and "802-11-wireless.ssid" in args:
                        state["ssid"] = args[args.index("802-11-wireless.ssid") + 1]
                    if args[0] == "ip":
                        return json.dumps([{"addr_info": [
                            {"scope": "global", "local": "10.0.91.250", "prefixlen": 23},
                        ]}])
                    return ""

                command_mock = Mock(side_effect=command)
                settings = Mock()
                with patch.dict(globals_, {"command": command_mock, "open": Mock(return_value=Mock())}), \
                        patch.object(globals_["fcntl"], "flock"), \
                        patch.object(globals_["socket"], "socket"), \
                        patch.object(globals_["dbus"], "Interface", return_value=settings), \
                        patch.object(globals_["dbus"], "Dictionary", side_effect=lambda value, **_: value), \
                        patch.object(globals_["dbus"], "ByteArray", side_effect=bytes):
                    manager = network_type("robot-a", "ap", 8082, "10.0.91.250/23", "", "go2control",
                                           lambda: True, Mock(), ap_ssid="NewHotspot")
                    try:
                        self.assertEqual(state["ssid"], "NewHotspot")
                        self.assertEqual(manager.ssid, state["ssid"])
                        self.assertEqual(manager._restore_connection, "wifi-uuid")
                        self.assertFalse(any(call.args[1:3] == ("connection", "add")
                                             for call in command_mock.call_args_list))
                        self.assertEqual(settings.AddConnection.call_count, 0 if existing else 1)
                        if not existing:
                            profile = settings.AddConnection.call_args.args[0]
                            self.assertEqual(profile["802-11-wireless"], {"ssid": b"NewHotspot", "mode": "ap"})
                            self.assertEqual(profile["connection"]["interface-name"], "wlan0")
                            self.assertFalse(profile["connection"]["autoconnect"])
                            self.assertEqual(profile["ipv4"]["method"], "shared")
                    finally:
                        manager.close()

    def test_sdk_discovery_rejects_legacy_and_unrelated_peers(self):
        with patch.dict("sys.modules", {"dbus": Mock()}):
            module = runpy.run_path(str(source.with_name("go2_network.py")))
        network = module["RobotNetwork"].__new__(module["RobotNetwork"])
        network.robot = "robot-a"
        network.name = "Local robot"
        network.ready = lambda: True
        network.ssid = "Test Wi-Fi"
        network.mode = "wifi"
        network.hostname = "go2-sdk.local"
        network.subnet = ipaddress.ip_network("192.168.1.0/24")
        network._lock = threading.Lock()
        network._peers = {}
        packet = {"protocol": "go2-sdk-web-1", "robot": "robot-b", "port": 8082,
                  "network_mode": "wifi", "hostname": "go2-sdk.local", "connected": True}
        for changed in ({"protocol": "go2-web-1"}, {"hostname": "go2.local"},
                        {"robot": "../robot"}, {"robot": ""}, {"robot": ["bad"]}):
            network._receive(json.dumps({**packet, **changed}).encode(), "192.168.1.20")
            self.assertIsNone(network.endpoint("robot-b"))
        network._receive(json.dumps(packet).encode(), "192.168.1.20")
        self.assertEqual(network.endpoint("robot-b"), ("192.168.1.20", 8082))
        self.assertEqual([row["robot"] for row in network.devices()], ["robot-a", "robot-b"])
        self.assertEqual(network.devices()[0]["name"], "Local robot")
        packet.update(robot="custom_robot-003", name="Third robot")
        network._receive(json.dumps(packet).encode(), "192.168.1.21")
        devices = {row["robot"]: row for row in network.devices()}
        self.assertEqual(devices["custom_robot-003"]["name"], "Third robot")
        self.assertTrue(devices["custom_robot-003"]["connected"])

    def test_unavailable_preferred_wifi_preserves_current_or_uses_fallback(self):
        with patch.dict("sys.modules", {"dbus": Mock()}):
            network_type = runpy.run_path(str(source.with_name("go2_network.py")))["RobotNetwork"]
        globals_ = network_type.__init__.__globals__
        for failure, active, expected_fallback in (
            ("modify", "current-uuid", False),
            ("modify", "--", True),
            ("up", "current-uuid", False),
            ("timeout", "current-uuid", False),
            (None, "current-uuid", False),
            ("empty", "current-uuid", False),
            ("empty", "--", True),
        ):
            with self.subTest(failure=failure, active=active):
                state = {"active": active}

                def command(*args):
                    if args[1:4] == ("-t", "-f", "DEVICE,TYPE"):
                        return "wlan0:wifi"
                    if args[1:3] == ("-g", "GENERAL.CON-UUID"):
                        return state["active"]
                    if args[1:3] == ("-g", "GENERAL.CONNECTION"):
                        return "Current Wi-Fi"
                    if args[1:3] == ("connection", "modify"):
                        if failure == "modify":
                            raise RuntimeError("unknown connection JDSK")
                        if failure == "timeout":
                            raise subprocess.TimeoutExpired(args, 30)
                    if args[1:5] == ("--wait", "20", "connection", "up"):
                        if failure == "up":
                            raise RuntimeError("preferred network unavailable")
                        state["active"] = "preferred-uuid"
                    if args[1:3] == ("-g", "802-11-wireless.ssid"):
                        return "Connected Wi-Fi"
                    if args[0] == "ip":
                        return json.dumps([{"addr_info": [
                            {"scope": "global", "local": "192.168.1.10", "prefixlen": 24},
                        ]}])
                    return ""

                def fallback(_network, preferred):
                    self.assertEqual(preferred, "" if failure == "empty" else "JDSK")
                    state["active"] = "fallback-uuid"

                command_mock = Mock(side_effect=command)
                with patch.dict(globals_, {"command": command_mock, "open": Mock(return_value=Mock())}), \
                        patch.object(globals_["fcntl"], "flock"), \
                        patch.object(globals_["socket"], "socket"), \
                        patch.object(network_type, "_connect_fallback", autospec=True,
                                     side_effect=fallback) as connect_fallback:
                    network = network_type("robot-a", "wifi", 8080, "10.0.91.250/23",
                                           "" if failure == "empty" else "JDSK",
                                           "go2control", lambda: True, Mock())
                    try:
                        self.assertEqual(network.address, "192.168.1.10")
                        self.assertEqual(connect_fallback.called, expected_fallback)
                        if failure == "empty":
                            self.assertFalse(any("" in call.args for call in command_mock.call_args_list))
                        if failure in ("modify", "timeout"):
                            self.assertNotIn(unittest.mock.call(
                                "nmcli", "--wait", "20", "connection", "up", "JDSK", "ifname", "wlan0"
                            ), command_mock.call_args_list)
                    finally:
                        network.close()


class WebHttpTests(unittest.TestCase):
    def setUp(self):
        self.camera = Mock()
        self.camera.status.return_value = {"enabled": False, "message": ""}
        self.ready = Mock(return_value=True)
        self.network = Mock()
        self.network.address = "192.168.1.20"
        self.node = SimpleNamespace(
            robot="robot-b", web_root=source.parent,
            camera=self.camera, controller_ready=self.ready, network=self.network, get_logger=lambda: Mock(),
            status=lambda: {"application": "unitree-go2-sdk", "robot": "robot-b", "connected": self.ready()},
            stop=Mock(),
        )
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), namespace["handler"](self.node))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

    def request(self, suffix, payload=None, headers=None):
        connection = HTTPConnection(*self.server.server_address, timeout=3)
        body = None if payload is None else json.dumps(payload)
        try:
            path = suffix if suffix.startswith("/") else "/api/robots/robot-b/" + suffix
            connection.request("GET" if body is None else "POST", path,
                               body=body, headers={"X-Go2-Application": "unitree-go2-sdk", **(headers or {})})
            response = connection.getresponse()
            return response.status, json.loads(response.read())
        finally:
            connection.close()

    def test_camera_can_be_closed_when_motion_controller_is_offline(self):
        self.ready.return_value = False
        for prefix in ("", "/api/"):
            with self.subTest(prefix=prefix):
                self.camera.set_enabled.reset_mock()
                self.assertEqual(self.request(prefix + "camera/status"), (200, self.camera.status()))
                self.assertEqual(self.request(prefix + "camera/toggle", {"enabled": False})[0], 200)
                self.camera.set_enabled.assert_called_once_with(False)
                self.camera.set_enabled.reset_mock()
                self.assertEqual(self.request(prefix + "camera/toggle", {"enabled": True})[0], 503)
                self.camera.set_enabled.assert_not_called()

    def test_camera_preview_returns_player_address(self):
        self.camera.set_enabled.return_value = True
        self.camera.status.return_value = {"enabled": True, "webrtc_port": 8889}
        for prefix in ("", "/api/"):
            with self.subTest(prefix=prefix):
                self.camera.set_enabled.reset_mock()
                self.assertEqual(self.request(prefix + "camera/toggle", {"enabled": True}),
                                 (200, {"enabled": True, "webrtc_port": 8889, "webrtc_host": "192.168.1.20"}))
                self.camera.set_enabled.assert_called_once_with(True)

    def test_camera_toggle_rejects_invalid_payload(self):
        for payload in ({}, {"enabled": "yes"}, [], {"enabled": 1}):
            self.assertEqual(self.request("camera/toggle", payload)[0], 400)
        self.camera.set_enabled.assert_not_called()

    def test_missing_camera_returns_actionable_failure(self):
        self.camera.set_enabled.return_value = False
        self.camera.status.return_value = {"enabled": False, "message": "未识别到 USB 摄像头"}
        self.assertEqual(self.request("camera/toggle", {"enabled": True}),
                         (503, {"error": "未识别到 USB 摄像头"}))

    def test_removed_mode_is_rejected_without_calling_controller(self):
        self.node.mode_client = Mock()
        self.node.set_mode = lambda mode: namespace["set_mode"](self.node, mode)
        for mode in ("run", "heart", "right_flip"):
            with self.subTest(mode=mode):
                self.assertEqual(self.request("mode", {"mode": mode}), (400, {"error": "unknown mode"}))
        self.node.mode_client.wait_for_service.assert_not_called()
        self.node.mode_client.call_async.assert_not_called()

    def test_rejected_post_does_not_corrupt_next_request(self):
        for suffix, headers, expected in (
            ("mode", {}, 503),
            ("stop", {"X-Go2-Application": "wrong-client"}, 409),
            ("unknown", {}, 404),
        ):
            with self.subTest(suffix=suffix, headers=headers):
                connection = HTTPConnection(*self.server.server_address, timeout=3)
                try:
                    # Exercise an accepted POST first, on the same connection.
                    self.ready.return_value = True
                    connection.request("POST", "/api/robots/robot-b/stop",
                                       json.dumps({}),
                                       {"X-Go2-Application": "unitree-go2-sdk"})
                    response = connection.getresponse()
                    self.assertEqual(response.status, 200)
                    response.read()
                    self.ready.return_value = False
                    connection.request("POST", "/api/robots/robot-b/" + suffix,
                                       json.dumps({"mode": "terrain"}),
                                       {"X-Go2-Application": "unitree-go2-sdk", **headers})
                    response = connection.getresponse()
                    self.assertEqual(response.status, expected)
                    response.read()
                    connection.request("GET", "/api/robots/robot-b/status")
                    response = connection.getresponse()
                    self.assertEqual(response.status, 200)
                    self.assertEqual(json.loads(response.read()), self.node.status())
                finally:
                    connection.close()

    def test_invalid_body_length_returns_json_error_and_recovers(self):
        for length in ("invalid", "0", "65537"):
            with self.subTest(length=length):
                connection = HTTPConnection(*self.server.server_address, timeout=3)
                try:
                    connection.request("POST", "/api/robots/robot-b/stop", "{}",
                                       {"X-Go2-Application": "unitree-go2-sdk", "Content-Length": length})
                    response = connection.getresponse()
                    self.assertEqual(response.status, 400)
                    self.assertIn("error", json.loads(response.read()))
                    connection.request("GET", "/api/robots/robot-b/status")
                    response = connection.getresponse()
                    self.assertEqual(response.status, 200)
                    response.read()
                finally:
                    connection.close()


if __name__ == "__main__":
    unittest.main()
