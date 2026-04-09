from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import zmq
import zmq.asyncio
import asyncio
import time
import math
from app.config import settings
from app.zmq_client import zmq_client
from app.routes.maps import update_live_map_png
from app.services.log_service import LogService
from app.services.session_service import SessionService
from app.services.telemetry_service import TelemetryService
from app.services.home_service import HomeService, voltage_to_percent, WARN_THRESHOLD
from app.services.path_service import PathService

router = APIRouter()
context = zmq.asyncio.Context()

_last_cmd_log = 0
MIN_REAL_PATH_DIST = 0.05  # meters between real path points
REAL_PATH_INTERVAL = 1.0   # seconds between DB writes


@router.websocket("/ws")
async def scada_websocket(websocket: WebSocket):
    await websocket.accept()

    session = await SessionService.get_current_session()
    session_id = session['id'] if session else None

    await LogService.log_event("SYSTEM", "ws_connected", "Client connected to SCADA WebSocket", session_id=session_id)

    sub_socket = context.socket(zmq.SUB)
    sub_socket.connect(f"tcp://{settings.robot_ip}:{settings.zmq_telemetry_port}")
    sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

    camera_socket = context.socket(zmq.SUB)
    camera_socket.connect(f"tcp://{settings.robot_ip}:{settings.zmq_camera_port}")
    camera_socket.setsockopt_string(zmq.SUBSCRIBE, "")

    # Shared mutable state between receive_ws and forward_telemetry
    shared = {
        "home": await HomeService.get_home(),
        "path_id": None,           # Current active nav_path DB id
        "last_global_plan": None,  # Cache to detect plan changes
    }

    async def receive_ws():
        global _last_cmd_log
        try:
            while True:
                data = await websocket.receive_json()
                msg_type = data.get("type")
                payload = data.get("payload")

                if msg_type == "cmd_vel":
                    await zmq_client.send_command("cmd_vel", payload)
                    current_time = time.time()
                    if current_time - _last_cmd_log > 2.0:
                        _last_cmd_log = current_time
                        await LogService.log_event("COMMAND", "cmd_vel_sent", "Manual velocity command sent", metadata=payload, session_id=session_id)

                elif msg_type == "nav_goal":
                    gx = payload.get("x", 0)
                    gy = payload.get("y", 0)
                    await zmq_client.send_command("nav_goal", payload)

                    # Create new path record in DB (completes any previous active path)
                    path_id = await PathService.create_path(session_id, gx, gy, [])
                    shared["path_id"] = path_id
                    shared["last_global_plan"] = None

                    # Notify frontend to clear actual path display
                    await websocket.send_json({"type": "path_started", "payload": {"path_id": path_id, "goal_x": gx, "goal_y": gy}})
                    await LogService.log_event("NAVIGATION", "nav_goal_sent", f"Nav goal ({gx:.2f}, {gy:.2f}), path_id={path_id}", metadata=payload, session_id=session_id)

                elif msg_type == "set_home":
                    x = float(payload.get("x", 0))
                    y = float(payload.get("y", 0))
                    ok = await HomeService.set_home(x, y, 0.0)
                    if ok:
                        shared["home"] = {"x": x, "y": y, "yaw": 0.0}
                        await websocket.send_json({"type": "home_set", "payload": shared["home"]})
                        await LogService.log_event("NAVIGATION", "home_set", f"Home set ({x:.3f}, {y:.3f})", metadata=payload, session_id=session_id)

                elif msg_type == "go_home":
                    home = await HomeService.get_home()
                    if home:
                        await zmq_client.send_command("nav_goal", {"x": home["x"], "y": home["y"]})
                        # Also create path record for go_home
                        path_id = await PathService.create_path(session_id, home["x"], home["y"], [])
                        shared["path_id"] = path_id
                        shared["last_global_plan"] = None
                        await websocket.send_json({"type": "path_started", "payload": {"path_id": path_id, "goal_x": home["x"], "goal_y": home["y"]}})
                        await websocket.send_json({"type": "going_home", "payload": home})
                        await LogService.log_event("NAVIGATION", "go_home", f"Go-home -> ({home['x']:.3f}, {home['y']:.3f}), path_id={path_id}", metadata=home, session_id=session_id)
                    else:
                        await websocket.send_json({"type": "error", "payload": {"message": "Home position not set"}})

                elif msg_type == "slam_control":
                    await zmq_client.send_command("slam_control", payload)
                    await LogService.log_event("COMMAND", "slam_control", f"SLAM control: {payload.get('action')}", metadata=payload, session_id=session_id)
        except WebSocketDisconnect:
            await LogService.log_event("SYSTEM", "ws_disconnected", "Client disconnected", session_id=session_id)
        except Exception as e:
            print(f"WS receive error: {e}")

    async def forward_telemetry():
        _last_voltage = None
        _return_triggered = False
        _warning_sent = False
        _last_real_pos = None
        _last_real_save = 0.0
        try:
            while True:
                msg = await sub_socket.recv_json()

                voltage = msg.get("voltage", 0)
                is_charging = msg.get("charging", False)
                battery_pct = voltage_to_percent(voltage)

                # Voltage threshold logging (calibrated for 24V 6S LiPo)
                if voltage > 0 and _last_voltage is not None:
                    if battery_pct <= 10 and voltage_to_percent(_last_voltage) > 10:
                        await LogService.log_event("POWER", "voltage_critical", f"Critical: {battery_pct:.0f}% ({voltage:.1f}V)", severity="CRITICAL", session_id=session_id)
                    elif battery_pct <= 20 and voltage_to_percent(_last_voltage) > 20:
                        await LogService.log_event("POWER", "voltage_low", f"Low: {battery_pct:.0f}% ({voltage:.1f}V)", severity="WARNING", session_id=session_id)
                _last_voltage = voltage

                # --- AUTO-RETURN at 10% ---
                if battery_pct <= 10 and not is_charging and not _return_triggered and voltage > 0:
                    home = shared["home"]
                    if home:
                        try:
                            await zmq_client.send_command("nav_goal", {"x": home["x"], "y": home["y"]})
                            _return_triggered = True
                            await LogService.log_event("POWER", "auto_return_triggered", f"Battery {battery_pct:.0f}% — auto-navigating home", severity="CRITICAL", session_id=session_id)
                            await websocket.send_json({"type": "auto_return", "payload": {"status": "started", "percent": round(battery_pct, 1), "home": home}})
                        except Exception as e:
                            print(f"[AutoReturn] Failed: {e}")

                # --- WARNING at 20% ---
                if 10 < battery_pct <= WARN_THRESHOLD and not _warning_sent and voltage > 0:
                    _warning_sent = True
                    await websocket.send_json({"type": "battery_warning", "payload": {"percent": round(battery_pct, 1), "voltage": round(voltage, 2)}})
                    await LogService.log_event("POWER", "battery_warning_20", f"Battery low: {battery_pct:.0f}% ({voltage:.1f}V)", severity="WARNING", session_id=session_id)

                if battery_pct > 25:
                    _warning_sent = False
                if is_charging:
                    _return_triggered = False

                # --- PATH TRACKING ---
                path_id = shared.get("path_id")
                if path_id:
                    now = time.time()

                    # Save global plan (only when it changes)
                    plan = msg.get("plan", [])
                    if plan and plan != shared.get("last_global_plan"):
                        shared["last_global_plan"] = plan
                        await PathService.update_global_plan(path_id, plan)

                    # Save local plan (overwrite on each change)
                    local_plan = msg.get("local_plan", [])
                    if local_plan:
                        await PathService.update_local_plan(path_id, local_plan)

                    # Append real path (rate limited, distance gated)
                    map_pose = msg.get("map_pose")
                    if map_pose and now - _last_real_save >= REAL_PATH_INTERVAL:
                        rx, ry = map_pose["x"], map_pose["y"]
                        should_save = True
                        if _last_real_pos:
                            dx = rx - _last_real_pos[0]
                            dy = ry - _last_real_pos[1]
                            if math.sqrt(dx*dx + dy*dy) < MIN_REAL_PATH_DIST:
                                should_save = False
                        if should_save:
                            await PathService.append_real_point(path_id, rx, ry)
                            _last_real_pos = (rx, ry)
                            _last_real_save = now

                # Inject enriched data
                msg["battery_pct"] = round(battery_pct, 1)
                if shared["home"]:
                    msg["home_position"] = shared["home"]

                await TelemetryService.maybe_save_snapshot(msg, session_id)
                await websocket.send_json({"type": "telemetry", "payload": msg})
        except asyncio.CancelledError:
            # Complete any active path on disconnect
            if shared.get("path_id"):
                await PathService.complete_path(shared["path_id"])

    async def forward_camera():
        try:
            while True:
                frame = await camera_socket.recv()
                if frame[:4] == b'MAP:':
                    update_live_map_png(frame[4:])
                else:
                    await websocket.send_bytes(frame)
        except asyncio.CancelledError:
            pass

    task1 = asyncio.create_task(receive_ws())
    task2 = asyncio.create_task(forward_telemetry())
    task3 = asyncio.create_task(forward_camera())

    done, pending = await asyncio.wait(
        [task1, task2, task3],
        return_when=asyncio.FIRST_COMPLETED
    )
    for task in pending:
        task.cancel()

    sub_socket.close()
    camera_socket.close()

@router.websocket("/ws/ai/detection")
async def ai_detection_websocket(websocket: WebSocket):
    await websocket.accept()
    ai_socket = context.socket(zmq.SUB)
    ai_socket.connect(f"tcp://{settings.robot_ip}:{settings.zmq_ai_detection_port}")
    ai_socket.setsockopt_string(zmq.SUBSCRIBE, "")

    try:
        while True:
            frame = await ai_socket.recv()
            if frame.startswith(b'DETECTION:'):
                frame = frame[10:]
            await websocket.send_bytes(frame)
    except Exception as e:
        print(f"Detection WebSocket Error: {e}")
    finally:
        ai_socket.close()

@router.websocket("/ws/ai/tracking")
async def ai_tracking_websocket(websocket: WebSocket):
    await websocket.accept()
    ai_socket = context.socket(zmq.SUB)
    ai_socket.connect(f"tcp://{settings.robot_ip}:{settings.zmq_human_tracking_port}")
    ai_socket.setsockopt_string(zmq.SUBSCRIBE, "")

    try:
        while True:
            frame = await ai_socket.recv()
            if frame.startswith(b'TRACKING:'):
                frame = frame[9:]
            await websocket.send_bytes(frame)
    except Exception as e:
        print(f"Tracking WebSocket Error: {e}")
    finally:
        ai_socket.close()

