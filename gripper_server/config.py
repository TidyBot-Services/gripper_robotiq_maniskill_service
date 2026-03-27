"""Gripper server configuration constants — delegates to gripper_protocol."""

from gripper_protocol.protocol import (
    DEFAULT_CMD_PORT as GRIPPER_CMD_PORT,
    DEFAULT_STATE_PORT as GRIPPER_STATE_PORT,
    MessageType,
)

STATE_HZ = 10

# Backward-compatible aliases used by bridge server.py
MSG_ACTIVATE = MessageType.ACTIVATE
MSG_RESET = MessageType.RESET
MSG_MOVE = MessageType.MOVE
MSG_OPEN = MessageType.OPEN
MSG_CLOSE = MessageType.CLOSE
MSG_STOP = MessageType.STOP
MSG_CALIBRATE = MessageType.CALIBRATE
MSG_RESPONSE = MessageType.RESPONSE
