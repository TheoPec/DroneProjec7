"""
    QTM 6DOF streaming server
    - Connects to QTM, tracks rigid body "Fatima"
    - Serves latest position + rotation as JSON on http://0.0.0.0:8080
    - Any PC on the network can GET http://<this-pc-ip>:8080 to get the data

    (start QTM first, load file, Play->Play with Real-Time output)
"""

import asyncio
import json
import xml.etree.ElementTree as ET
from threading import Lock
from aiohttp import web

import qtm_rt

SERVER_PORT = 8080

# Shared state: latest data from QTM
_lock = Lock()
_latest = {
    "frame": None,
    "position": None,
    "rotation": None,
}

FATIMA_INDEX = None


def on_packet(packet):
    """Callback function that is called everytime a data packet arrives from QTM"""
    global FATIMA_INDEX

    if FATIMA_INDEX is None:
        return

    frame = packet.framenumber

    # --- 6DOF Rigid Body "Fatima" ---
    header, bodies = packet.get_6d()

    position = None
    rotation = None

    if FATIMA_INDEX < len(bodies):
        pos, rot = bodies[FATIMA_INDEX]
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
    """Return latest Fatima data as JSON"""
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
    """Connect to QTM, find Fatima, start streaming"""
    global FATIMA_INDEX

    connection = await qtm_rt.connect("127.0.0.1")
    if connection is None:
        print("Failed to connect to QTM")
        return

    # Get 6DOF body settings to find the index of "Fatima"
    xml_string = await connection.get_parameters(parameters=["6d"])
    xml = ET.fromstring(xml_string)

    body_names = [
        body.find("Name").text
        for body in xml.findall("*/Body")
    ]

    print("Available rigid bodies: {}".format(body_names))

    if "Fatima" in body_names:
        FATIMA_INDEX = body_names.index("Fatima")
        print("Found 'Fatima' at index {}".format(FATIMA_INDEX))
    else:
        print("ERROR: Rigid body 'Fatima' not found in QTM!")
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