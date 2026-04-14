"""Gripper bridge for ManiSkill — exposes Robotiq-compatible gripper control via ZMQ.

Protocol: 2 ZMQ sockets, msgpack serialization (same as MuJoCo version).
  CMD   REP  port 5570
  STATE PUB  port 5571
"""

import threading
import time

import msgpack
import zmq

from .config import (
    GRIPPER_CMD_PORT, GRIPPER_STATE_PORT,
    MSG_ACTIVATE, MSG_CALIBRATE, MSG_CLOSE, MSG_MOVE, MSG_OPEN,
    MSG_RESET, MSG_RESPONSE, MSG_STOP, STATE_HZ,
)

GRIPPER_OPEN_VAL = 0.0
GRIPPER_CLOSED_VAL = 0.81


class GripperBridge:
    """Protocol bridge: ZMQ REP + PUB for Robotiq-style gripper control."""

    def __init__(self, server, cmd_port=GRIPPER_CMD_PORT, state_port=GRIPPER_STATE_PORT,
                 env_idx=0):
        self._server = server
        self._env_idx = env_idx
        self._cmd_port = cmd_port
        self._state_port = state_port
        self._running = False
        self._threads = []

    def start(self):
        self._running = True
        for target, name in [
            (self._state_publisher, "gripper-state-pub"),
            (self._command_handler, "gripper-cmd-handler"),
        ]:
            t = threading.Thread(target=target, daemon=True, name=name)
            t.start()
            self._threads.append(t)

    def stop(self):
        self._running = False
        for t in self._threads:
            t.join(timeout=3)
        self._threads.clear()
        print("[gripper-bridge] Stopped")

    def _state_publisher(self):
        ctx = zmq.Context()
        sock = ctx.socket(zmq.PUB)
        sock.bind(f"tcp://*:{self._state_port}")
        print(f"[gripper-bridge] State PUB on {self._state_port}, CMD REP on {self._cmd_port}")

        interval = 1.0 / STATE_HZ
        while self._running:
            state = self._server.get_state(env_idx=self._env_idx)
            msg = self._build_state_dict(state)
            sock.send(msgpack.packb(msg, use_bin_type=True))
            time.sleep(interval)

        sock.close()
        ctx.term()

    def _command_handler(self):
        ctx = zmq.Context()
        sock = ctx.socket(zmq.REP)
        sock.bind(f"tcp://*:{self._cmd_port}")
        sock.setsockopt(zmq.RCVTIMEO, 500)

        while self._running:
            try:
                raw = sock.recv()
            except zmq.Again:
                continue
            try:
                req = msgpack.unpackb(raw, raw=False)
                resp = self._dispatch_rpc(req)
            except Exception as e:
                resp = {"error": str(e)}
            sock.send(msgpack.packb(resp, use_bin_type=True))

        sock.close()
        ctx.term()

    def _wait_gripper_settled(self, timeout=3.0, tol_mm=0.5, settle_reads=3):
        """Poll sim state until gripper position stabilizes or timeout."""
        # Initial delay so the sim has time to start applying the action
        time.sleep(0.15)
        prev_mm = None
        stable_count = 0
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(0.05)
            state = self._server.get_state(env_idx=self._env_idx)
            mm = state.gripper_position_mm
            if prev_mm is not None and abs(mm - prev_mm) < tol_mm:
                stable_count += 1
                if stable_count >= settle_reads:
                    return state
            else:
                stable_count = 0
            prev_mm = mm
        return self._server.get_state(env_idx=self._env_idx)

    def _dispatch_rpc(self, req):
        msg_type = req.get("msg_type")

        def _ok(data=None):
            return {"msg_type": MSG_RESPONSE, "success": True, "message": "", "data": data}

        if msg_type == MSG_ACTIVATE:
            return _ok({"result": True})
        elif msg_type == MSG_OPEN:
            self._server.set_gripper_action(env_idx=self._env_idx, value=GRIPPER_OPEN_VAL)
            state = self._wait_gripper_settled()
            return _ok({"position": self._mm_to_robotiq(state.gripper_position_mm),
                         "object_detected": False})
        elif msg_type == MSG_CLOSE:
            self._server.set_gripper_action(env_idx=self._env_idx, value=GRIPPER_CLOSED_VAL)
            state = self._wait_gripper_settled()
            return _ok({"position": self._mm_to_robotiq(state.gripper_position_mm),
                         "object_detected": state.gripper_object_detected})
        elif msg_type == MSG_MOVE:
            position = req.get("position", 0)
            val = GRIPPER_CLOSED_VAL if position >= 128 else GRIPPER_OPEN_VAL
            self._server.set_gripper_action(env_idx=self._env_idx, value=val)
            state = self._wait_gripper_settled()
            return _ok({"position": self._mm_to_robotiq(state.gripper_position_mm),
                         "object_detected": state.gripper_object_detected})
        elif msg_type in (MSG_STOP, MSG_RESET, MSG_CALIBRATE):
            return _ok()
        else:
            return {"msg_type": MSG_RESPONSE, "success": False,
                    "message": f"unknown msg_type: {msg_type}", "data": None}

    @staticmethod
    def _mm_to_robotiq(position_mm):
        return int(255 * (1.0 - min(position_mm / 85.0, 1.0)))

    @staticmethod
    def _build_state_dict(state):
        position_mm = state.gripper_position_mm
        position = int(255 * (1.0 - min(position_mm / 85.0, 1.0)))
        return {
            "timestamp": time.time(),
            "position": position,
            "position_mm": position_mm,
            "is_activated": True,
            "is_moving": False,
            "object_detected": state.gripper_object_detected,
            "is_calibrated": True,
            "current": 0,
            "current_ma": 0.0,
            "fault_code": 0,
            "fault_message": "",
            "gripper_type": 1,
        }
