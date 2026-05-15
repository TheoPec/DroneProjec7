import asyncio
import threading
import time
from aiohttp import web

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8081
MAVLINK_PORT = "/dev/serial0"
MAVLINK_BAUD = 57600
MAX_MOTOR_POWER = 30
MOTOR_TIMEOUT_SECONDS = 1.0
PWM_MIN = 1000
PWM_MAX_TEST = 1300


class MavlinkMotorOutput:
    """
    TEST ONLY WITHOUT PROPELLERS.

    This class sends only basic MAVLink commands for safe bench motor tests.
    It does not implement takeoff, landing, navigation, autonomous flight, or Qualisys feedback.

    Motor power is sent with RC channel override on throttle channel 3.
    Keep propellers removed and keep MAX_MOTOR_POWER low during first tests.
    """

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
                heartbeat = self.master.wait_heartbeat(timeout=5)
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

    def arm(self):
        with self._lock:
            self._require_connection()
            self.master.arducopter_arm()

            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                self.master.wait_heartbeat(timeout=0.5)
                if self.master.motors_armed():
                    return

        raise RuntimeError("pixhawk arm failed")

    def disarm(self):
        with self._lock:
            self.stop()
            self._require_connection()
            self.master.arducopter_disarm()

    def _send_throttle_pwm(self, throttle_pwm):
        self._require_connection()
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            65535,
            65535,
            throttle_pwm,
            65535,
            65535,
            65535,
            65535,
            65535,
        )

    def stop(self):
        with self._lock:
            self._send_throttle_pwm(PWM_MIN)
            print(f"[MAVLINK MOTOR OUTPUT] power=0% pwm={PWM_MIN}")

    def set_power(self, power):
        with self._lock:
            safe_power = max(0, min(MAX_MOTOR_POWER, int(power)))
            throttle_pwm = PWM_MIN + int((safe_power / 100.0) * 1000)
            throttle_pwm = max(PWM_MIN, min(PWM_MAX_TEST, throttle_pwm))
            self._send_throttle_pwm(throttle_pwm)
            print(f"[MAVLINK MOTOR OUTPUT] power={safe_power}% pwm={throttle_pwm}")


motor_output = MavlinkMotorOutput(MAVLINK_PORT, MAVLINK_BAUD)

state = {
    "armed": False,
    "mode": "DISARMED",
    "motor_power": 0,
    "pixhawk_connected": False,
}
last_motor_command_time = 0.0


def clamp_motor_power(power):
    return max(0, min(MAX_MOTOR_POWER, int(power)))


def set_motor_power(power):
    safe_power = clamp_motor_power(power)
    motor_output.set_power(safe_power)
    state["motor_power"] = safe_power


def stop_motors_now():
    motor_output.stop()
    state["motor_power"] = 0


def safe_stop_motors_now():
    try:
        stop_motors_now()
    except Exception as exc:
        print(f"[SAFETY WARNING] Could not send motor stop: {exc}")
        state["motor_power"] = 0


async def run_blocking(func, *args):
    return await asyncio.to_thread(func, *args)


async def arm():
    await run_blocking(motor_output.arm)
    state["armed"] = True
    state["mode"] = "ARMED"


async def disarm():
    await run_blocking(motor_output.disarm)
    state["armed"] = False
    state["mode"] = "DISARMED"
    state["motor_power"] = 0


async def stop_motors():
    await run_blocking(stop_motors_now)
    state["mode"] = "ARMED" if state["armed"] else "DISARMED"


async def motor_test(power):
    global last_motor_command_time
    if not state["armed"]:
        await run_blocking(safe_stop_motors_now)
        state["mode"] = "DISARMED"
        return
    await run_blocking(set_motor_power, power)
    last_motor_command_time = time.monotonic()
    state["mode"] = "MOTOR_TEST"


async def handle_command(request):
    state["pixhawk_connected"] = motor_output.is_connected()

    try:
        payload = await request.json()
    except Exception:
        return web.json_response({"ok": False, "error": "invalid JSON", "state": state}, status=400)

    command = payload.get("command")

    try:
        if command == "status":
            pass
        elif command == "arm":
            await arm()
        elif command == "disarm":
            await disarm()
        elif command == "stop_motors":
            await stop_motors()
        elif command == "motor_test":
            await motor_test(payload.get("power", 0))
        else:
            return web.json_response(
                {"ok": False, "error": f"unknown command: {command}", "state": state},
                status=400,
            )
    except Exception as exc:
        if command in ("motor_test", "stop_motors", "disarm"):
            await run_blocking(safe_stop_motors_now)
        state["pixhawk_connected"] = motor_output.is_connected()
        return web.json_response({"ok": False, "error": str(exc), "state": state}, status=400)

    state["pixhawk_connected"] = motor_output.is_connected()
    return web.json_response({"ok": True, "state": state})


async def watchdog():
    while True:
        await asyncio.sleep(0.05)
        if state["motor_power"] > 0 and time.monotonic() - last_motor_command_time > MOTOR_TIMEOUT_SECONDS:
            print("[SAFETY] Command timeout: stopping motors")
            await run_blocking(safe_stop_motors_now)
            state["mode"] = "TIMEOUT_STOP"


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
    state["armed"] = False
    state["mode"] = "DISARMED"
    state["motor_power"] = 0
    state["pixhawk_connected"] = await run_blocking(motor_output.connect)
    await run_blocking(safe_stop_motors_now)

    app = web.Application(middlewares=[cors_middleware])
    app.router.add_post("/command", handle_command)
    app.router.add_options("/command", handle_options)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, SERVER_HOST, SERVER_PORT)
    await site.start()
    asyncio.create_task(watchdog())

    print(f"Raspberry motor test server running on http://{SERVER_HOST}:{SERVER_PORT}")
    print("TEST ONLY WITHOUT PROPELLERS")
    print("Allowed commands: status, arm, disarm, stop_motors, motor_test")

    while True:
        await asyncio.sleep(3600)


if __name__ == "__main__":
    asyncio.run(main())
