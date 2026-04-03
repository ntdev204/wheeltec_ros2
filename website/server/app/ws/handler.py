from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import zmq
import zmq.asyncio
import asyncio
import time
from app.config import settings
from app.zmq_client import zmq_client
from app.routes.maps import update_live_map_png
from app.services.log_service import LogService
from app.services.session_service import SessionService
from app.services.telemetry_service import TelemetryService

router = APIRouter()
context = zmq.asyncio.Context()

# Global config to rate limit log events 
_last_cmd_log = 0

@router.websocket("/ws")
async def scada_websocket(websocket: WebSocket):
    await websocket.accept()
    
    session = await SessionService.get_current_session()
    session_id = session['id'] if session else None
    
    await LogService.log_event("SYSTEM", "ws_connected", "Client connected to SCADA WebSocket", session_id=session_id)
    
    # Setup ZMQ SUB sockets for telemetry and camera data from ROS2 layer
    sub_socket = context.socket(zmq.SUB)
    sub_socket.connect(f"tcp://{settings.robot_ip}:{settings.zmq_telemetry_port}")
    sub_socket.setsockopt_string(zmq.SUBSCRIBE, "") # Subscribe to all telemetry
    
    camera_socket = context.socket(zmq.SUB)
    camera_socket.connect(f"tcp://{settings.robot_ip}:{settings.zmq_camera_port}")
    camera_socket.setsockopt_string(zmq.SUBSCRIBE, "")

    async def receive_ws():
        global _last_cmd_log
        try:
            while True:
                data = await websocket.receive_json()
                msg_type = data.get("type")
                payload = data.get("payload")
                
                # Handle incoming commands from UI
                if msg_type == "cmd_vel":
                    await zmq_client.send_command("cmd_vel", payload)
                    
                    # Rate limit cmd_vel logs (max 1 per 2 seconds)
                    current_time = time.time()
                    if current_time - _last_cmd_log > 2.0:
                        _last_cmd_log = current_time
                        await LogService.log_event("COMMAND", "cmd_vel_sent", "Manual velocity command sent", metadata=payload, session_id=session_id)
                        
                elif msg_type == "nav_goal":
                    await zmq_client.send_command("nav_goal", payload)
                    await LogService.log_event("NAVIGATION", "nav_goal_sent", f"Navigation goal sent to ({payload.get('x', 0):.2f}, {payload.get('y', 0):.2f})", metadata=payload, session_id=session_id)
                    
                elif msg_type == "slam_control":
                    await zmq_client.send_command("slam_control", payload)
                    await LogService.log_event("COMMAND", "slam_control", f"SLAM control command: {payload.get('action')}", metadata=payload, session_id=session_id)
        except WebSocketDisconnect:
            await LogService.log_event("SYSTEM", "ws_disconnected", "Client disconnected from WebSocket", session_id=session_id)
        except Exception as e:
            print(f"WS receive error: {e}")

    async def forward_telemetry():
        _last_voltage = None
        try:
            while True:
                msg = await sub_socket.recv_json()
                
                # Check for voltage drops
                voltage = msg.get("voltage", 0)
                if voltage > 0 and _last_voltage is not None:
                    if voltage < 9.5 and _last_voltage >= 9.5:
                        await LogService.log_event("POWER", "voltage_critical", f"Critical battery level: {voltage}V", severity="CRITICAL", session_id=session_id)
                    elif voltage < 10.5 and _last_voltage >= 10.5:
                        await LogService.log_event("POWER", "voltage_low", f"Low battery warning: {voltage}V", severity="WARNING", session_id=session_id)
                _last_voltage = voltage
                
                # Save snapshot if needed
                saved = await TelemetryService.maybe_save_snapshot(msg, session_id)
                if saved:
                    # Optional: Could log silent event if needed, but omitted to prevent DB bloat
                    pass
                
                # Forward telemetry
                await websocket.send_json({"type": "telemetry", "payload": msg})
        except asyncio.CancelledError:
            pass
            
    async def forward_camera():
        try:
            while True:
                frame = await camera_socket.recv()
                if frame[:4] == b'MAP:':
                    # Map PNG update — store for the REST endpoint
                    update_live_map_png(frame[4:])
                else:
                    # JPEG camera frame — forward as binary to UI
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

