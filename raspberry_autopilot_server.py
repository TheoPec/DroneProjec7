"""
Single Raspberry server for indoor Qualisys-assisted Pixhawk flight.

This one process does both jobs:
- reads the existing QTM HTTP bridge, usually http://<qtm-pc>:8080/
- sends MAVLink VISION_POSITION_ESTIMATE to the Pixhawk continuously
- exposes POST /command on 0.0.0.0:8081 for the web interface
- handles arm, disarm, takeoff, land, and set_target

It does NOT do raw motor test. Keep raspberry_motor_test_server.py for bench motor tests.

Before real flight:
- configure ArduCopter EKF to use external vision
- verify axes in Mission Planner/QGroundControl
- test without propellers first
- then test tethered/secured
"""

import asyncio
import math
import os
import threading
import time
from aiohttp import ClientSession, web

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8081

QTM_URL = os.environ.get("QTM_URL", "http://127.0.0.1:8080/")
VISION_SEND_HZ = float(os.environ.get("VISION_SEND_HZ", "30"))

MAVLINK_PORT = os.environ.get("MAVLINK_PORT", "/dev/serial0")
MAVLINK_BAUD = int(os.environ.get("MAVLINK_BAUD", "57600"))

MAX_TAKEOFF_ALT_M = 2.0
MAX_TARGET_ALT_M = 2.0
MAX_TARGET_XY_M = 3.0
COMMAND_TIMEOUT_S = 5.0
VISION_STALE_TIMEOUT_S = 0.5

# Default: QTM is millimeters, usually Z-up. MAVLink external vision uses meters.
# z is negative by default because ArduPilot local frames often use NED (Z down).
X_SIGN = float(os.environ.get("VISION_X_SIGN", "1"))
Y_SIGN = float(os.environ.get("VISION_Y_SIGN", "1"))
Z_SIGN = float(os.environ.get("VISION_Z_SIGN", "-1"))


state = {
    "armed": False,
    "mode": "DISCONNECTED",
    "target": {"x": 0.0, "y": 0.0, "z": 0.0},
    "pixhawk_connected": False,
    "qtm_connected": False,
    "vision_rate_hz": 0.0,
    "last_vision_age_ms": None,
    "failsafe_triggered": False,
    "vision_stale_timeout_s": VISION_STALE_TIMEOUT_S,
    "max_takeoff_alt_m": MAX_TAKEOFF_ALT_M,
    "max_target_alt_m": MAX_TARGET_ALT_M,
    "max_target_xy_m": MAX_TARGET_XY_M,
}


def clamp_float(value, low, high):
    value = float(value)
    if math.isnan(value) or math.isinf(value):
        raise ValueError("invalid numeric value")
    return max(low, min(high, value))


def rot_mat_to_euler(m):
    sy = math.sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0])
    if sy > 1e-6:
        roll = math.atan2(m[2][1], m[2][2])
        pitch = math.atan2(-m[2][0], sy)
        yaw = math.atan2(m[1][0], m[0][0])
    else:
        roll = math.atan2(-m[1][2], m[1][1])
        pitch = math.atan2(-m[2][0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def convert_qtm_pose(data):
    position = data.get("position")
    rotation = data.get("rotation")
    if not position or not rotation:
        return None

    x = X_SIGN * (float(position["x"]) / 1000.0)
    y = Y_SIGN * (float(position["y"]) / 1000.0)
    z = Z_SIGN * (float(position["z"]) / 1000.0)
    roll, pitch, yaw = rot_mat_to_euler(rotation)
    return x, y, z, roll, pitch, yaw


class Pixhawk:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.master = None
        self._lock = threading.RLock()

    def connect(self):
        if mavutil is None:
            print("[MAVLINK] pymavlink is not installed")
            return False

        with self._lock:
            try:
                print(f"[MAVLINK] Connecting on {self.port} @ {self.baud} baud")
                self.master = mavutil.mavlink_connection(self.port, baud=self.baud)
                heartbeat = self.master.wait_heartbeat(timeout=10)
                if heartbeat is None:
                    self.master = None
                    print("[MAVLINK] Pixhawk heartbeat timeout")
                    return False
                print("Pixhawk connected")
                return True
            except Exception as exc:
                self.master = None
                print(f"[MAVLINK] Pixhawk connection failed: {exc}")
                return False

    def is_connected(self):
        return self.master is not None

    def _require_connection(self):
        if not self.is_connected():
            raise RuntimeError("pixhawk not connected")

    def send_vision(self, pose):
        with self._lock:
            self._require_connection()
            x, y, z, roll, pitch, yaw = pose
            usec = int(time.time() * 1_000_000)
            self.master.mav.vision_position_estimate_send(
                usec,
                x,
                y,
                z,
                roll,
                pitch,
                yaw,
            )

    def set_guided(self):
        self._require_connection()
        self.master.set_mode_apm("GUIDED")

    def arm(self):
        with self._lock:
            self._require_connection()
            self.set_guided()
            self.master.arducopter_arm()

            deadline = time.monotonic() + COMMAND_TIMEOUT_S
            while time.monotonic() < deadline:
                self.master.wait_heartbeat(timeout=0.5)
                if self.master.motors_armed():
                    return
            raise RuntimeError("pixhawk arm failed")

    def disarm(self):
        with self._lock:
            self._require_connection()
            self.master.arducopter_disarm()

    def takeoff(self, altitude_m):
        with self._lock:
            self._require_connection()
            altitude = clamp_float(altitude_m, 0.2, MAX_TAKEOFF_ALT_M)
            if not self.master.motors_armed():
                raise RuntimeError("cannot takeoff: pixhawk is not armed")
            self.set_guided()
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                altitude,
            )

    def land(self):
        with self._lock:
            self._require_connection()
            self.master.set_mode_apm("LAND")

    def goto_local(self, x_m, y_m, z_m):
        with self._lock:
            self._require_connection()
            x = clamp_float(x_m, -MAX_TARGET_XY_M, MAX_TARGET_XY_M)
            y = clamp_float(y_m, -MAX_TARGET_XY_M, MAX_TARGET_XY_M)
            z = clamp_float(z_m, 0.2, MAX_TARGET_ALT_M)
            if not self.master.motors_armed():
                raise RuntimeError("cannot set target: pixhawk is not armed")
            self.set_guided()
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,
                x,
                y,
                -z,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )


pixhawk = Pixhawk(MAVLINK_PORT, MAVLINK_BAUD)
last_vision_time = 0.0


async def run_blocking(func, *args):
    return await asyncio.to_thread(func, *args)


def update_connection_state():
    state["pixhawk_connected"] = pixhawk.is_connected()
    if last_vision_time > 0:
        state["last_vision_age_ms"] = int((time.monotonic() - last_vision_time) * 1000)
    else:
        state["last_vision_age_ms"] = None


async def fetch_qtm_pose(session):
    async with session.get(QTM_URL, timeout=1.0) as response:
        response.raise_for_status()
        return await response.json()


async def vision_loop():
    global last_vision_time
    period = 1.0 / VISION_SEND_HZ
    sent_count = 0
    last_rate_log = time.monotonic()

    print(f"[QTM] Reading pose from {QTM_URL}")
    print(f"[VISION] Sending VISION_POSITION_ESTIMATE at {VISION_SEND_HZ:.1f} Hz")
    print(f"[VISION] Axis signs: x={X_SIGN}, y={Y_SIGN}, z={Z_SIGN}")

    async with ClientSession() as session:
        while True:
            loop_start = time.monotonic()
            try:
                data = await fetch_qtm_pose(session)
                pose = convert_qtm_pose(data)
                if pose is not None:
                    await run_blocking(pixhawk.send_vision, pose)
                    sent_count += 1
                    last_vision_time = time.monotonic()
                    state["qtm_connected"] = True
            except Exception as exc:
                state["qtm_connected"] = False
                print(f"[VISION WARNING] {exc}")

            now = time.monotonic()
            if now - last_rate_log >= 1.0:
                state["vision_rate_hz"] = round(sent_count / (now - last_rate_log), 1)
                sent_count = 0
                last_rate_log = now

            elapsed = time.monotonic() - loop_start
            await asyncio.sleep(max(0.0, period - elapsed))


def vision_is_fresh():
    return last_vision_time > 0 and time.monotonic() - last_vision_time <= VISION_STALE_TIMEOUT_S


async def require_fresh_vision():
    if not vision_is_fresh():
        raise RuntimeError("external vision is not fresh")


async def cmd_arm():
    await require_fresh_vision()
    await run_blocking(pixhawk.arm)
    state["armed"] = True
    state["mode"] = "GUIDED"


async def cmd_disarm():
    await run_blocking(pixhawk.disarm)
    state["armed"] = False
    state["mode"] = "DISARMED"


async def cmd_takeoff(payload):
    await require_fresh_vision()
    z = payload.get("z", 1.0)
    await run_blocking(pixhawk.takeoff, z)
    state["armed"] = True
    state["mode"] = "TAKEOFF"
    state["target"] = {"x": state["target"]["x"], "y": state["target"]["y"], "z": clamp_float(z, 0.2, MAX_TAKEOFF_ALT_M)}


async def cmd_land():
    await run_blocking(pixhawk.land)
    state["mode"] = "LAND"


async def cmd_emergency_stop():
    try:
        await run_blocking(pixhawk.land)
        state["mode"] = "EMERGENCY_LAND"
    except Exception as land_exc:
        print(f"[EMERGENCY] LAND failed: {land_exc}; trying disarm")
        await run_blocking(pixhawk.disarm)
        state["armed"] = False
        state["mode"] = "EMERGENCY_DISARM"


async def cmd_set_target(payload):
    await require_fresh_vision()
    x = payload.get("x", 0.0)
    y = payload.get("y", 0.0)
    z = payload.get("z", 1.0)
    await run_blocking(pixhawk.goto_local, x, y, z)
    state["mode"] = "GUIDED_TARGET"
    state["target"] = {
        "x": clamp_float(x, -MAX_TARGET_XY_M, MAX_TARGET_XY_M),
        "y": clamp_float(y, -MAX_TARGET_XY_M, MAX_TARGET_XY_M),
        "z": clamp_float(z, 0.2, MAX_TARGET_ALT_M),
    }


async def vision_failsafe_watchdog():
    while True:
        await asyncio.sleep(0.05)
        if state["armed"] and not state["failsafe_triggered"] and not vision_is_fresh():
            state["failsafe_triggered"] = True
            state["mode"] = "VISION_FAILSAFE_LAND"
            print("[FAILSAFE] Vision lost, switching to LAND")
            try:
                await run_blocking(pixhawk.land)
            except Exception as exc:
                print(f"[FAILSAFE WARNING] LAND command failed: {exc}")


async def handle_command(request):
    update_connection_state()
    try:
        payload = await request.json()
    except Exception:
        return web.json_response({"ok": False, "error": "invalid JSON", "state": state}, status=400)

    command = payload.get("command")

    try:
        if command == "status":
            pass
        elif command == "arm":
            await cmd_arm()
        elif command == "disarm":
            await cmd_disarm()
        elif command == "takeoff":
            await cmd_takeoff(payload)
        elif command == "land":
            await cmd_land()
        elif command == "emergency_stop":
            await cmd_emergency_stop()
        elif command == "clear_failsafe":
            state["failsafe_triggered"] = False
            if state["mode"] == "VISION_FAILSAFE_LAND":
                state["mode"] = "ARMED" if state["armed"] else "READY"
        elif command == "set_target":
            await cmd_set_target(payload)
        elif command in ("stop_motors", "motor_test"):
            return web.json_response(
                {"ok": False, "error": f"{command} is disabled in autopilot server", "state": state},
                status=400,
            )
        else:
            return web.json_response(
                {"ok": False, "error": f"unknown command: {command}", "state": state},
                status=400,
            )
    except Exception as exc:
        update_connection_state()
        return web.json_response({"ok": False, "error": str(exc), "state": state}, status=400)

    update_connection_state()
    return web.json_response({"ok": True, "state": state})


def cors_headers():
    return {
        "Access-Control-Allow-Origin": "*",
        "Access-Control-Allow-Methods": "POST, OPTIONS",
        "Access-Control-Allow-Headers": "Content-Type",
    }


async def handle_options(_request):
    return web.Response(status=204, headers=cors_headers())


@web.middleware
async def cors_middleware(request, handler):
    if request.method == "OPTIONS":
        return await handle_options(request)
    response = await handler(request)
    response.headers.update(cors_headers())
    return response


async def main():
    state["pixhawk_connected"] = await run_blocking(pixhawk.connect)
    state["mode"] = "READY" if state["pixhawk_connected"] else "DISCONNECTED"

    app = web.Application(middlewares=[cors_middleware])
    app.router.add_post("/command", handle_command)
    app.router.add_options("/command", handle_options)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, SERVER_HOST, SERVER_PORT)
    await site.start()
    asyncio.create_task(vision_loop())
    asyncio.create_task(vision_failsafe_watchdog())

    print(f"Raspberry autopilot server running on http://{SERVER_HOST}:{SERVER_PORT}")
    print("ONE PROCESS: QTM HTTP -> Pixhawk vision + web commands -> Pixhawk")
    print("REAL FLIGHT - VERIFY EKF, AXES, FAILSAFE, AND TEST SAFELY")

    while True:
        await asyncio.sleep(3600)


if __name__ == "__main__":
    asyncio.run(main())
