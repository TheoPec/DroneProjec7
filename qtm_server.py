"""
    QTM 6DOF streaming server
    - Connects to QTM, tracks a configured rigid body
    - Serves latest position + rotation as JSON on http://0.0.0.0:8080
    - Any PC on the network can GET http://<this-pc-ip>:8080 to get the data

    (start QTM first, load file, Play->Play with Real-Time output)

    Optional environment variables:
    - QTM_HOST=<ip-or-hostname>
    - QTM_BODY=<rigid-body-name>
"""

import asyncio
import json
import os
import xml.etree.ElementTree as ET
from threading import Lock
from aiohttp import web

import qtm_rt

SERVER_PORT = 8080
QTM_HOST = os.environ.get("QTM_HOST", "127.0.0.1")
BODY_NAME = os.environ.get("QTM_BODY", "Obelix")

# Shared state: latest data from QTM
_lock = Lock()
_latest = {
    "frame": None,
    "position": None,
    "rotation": None,
}

BODY_INDEX = None


def on_packet(packet):
    """Callback function that is called everytime a data packet arrives from QTM"""
    global BODY_INDEX

    if BODY_INDEX is None:
        return

    frame = packet.framenumber

    # --- 6DOF rigid body ---
    header, bodies = packet.get_6d()

    position = None
    rotation = None

    if BODY_INDEX < len(bodies):
        pos, rot = bodies[BODY_INDEX]
        position = {"x": pos.x, "y": pos.y, "z": pos.z}
        rotation = [
            [rot.matrix[0], rot.matrix[1], rot.matrix[2]],
            [rot.matrix[3], rot.matrix[4], rot.matrix[5]],
            [rot.matrix[6], rot.matrix[7], rot.matrix[8]],
        ]

    with _lock:
        _latest["frame"] = frame
        _latest["position"] = position
        _latest["rotation"] = rotation

    print("Frame {} | pos={}".format(frame, position))


# ---- HTTP server ----

async def handle_get(request):
    """Return latest rigid body data as JSON"""
    with _lock:
        data = json.dumps(_latest)
    return web.Response(
        text=data,
        content_type="application/json",
        headers={
            "Access-Control-Allow-Origin": "*",
            "Access-Control-Allow-Methods": "GET",
        },
    )


async def start_http_server():
    app = web.Application()
    app.router.add_get("/", handle_get)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", SERVER_PORT)
    await site.start()
    print("HTTP server running on http://0.0.0.0:{}".format(SERVER_PORT))


# ---- QTM connection ----

async def setup():
    """Connect to QTM, find the configured rigid body, start streaming"""
    global BODY_INDEX

    connection = await qtm_rt.connect(QTM_HOST)
    if connection is None:
        print("Failed to connect to QTM at {}".format(QTM_HOST))
        return

    # Get 6DOF body settings to find the configured rigid body index.
    xml_string = await connection.get_parameters(parameters=["6d"])
    xml = ET.fromstring(xml_string)

    body_names = [
        body.find("Name").text
        for body in xml.findall("*/Body")
    ]

    print("Available rigid bodies: {}".format(body_names))

    if BODY_NAME in body_names:
        BODY_INDEX = body_names.index(BODY_NAME)
        print("Found '{}' at index {}".format(BODY_NAME, BODY_INDEX))
    else:
        print("ERROR: Rigid body '{}' not found in QTM!".format(BODY_NAME))
        print("Available bodies: {}".format(body_names))
        await connection.disconnect()
        return

    await connection.stream_frames(components=["6d"], on_packet=on_packet)


async def main():
    await start_http_server()
    await setup()


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.run_forever()
