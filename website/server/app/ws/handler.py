from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import zmq
import zmq.asyncio
import asyncio
from app.config import settings
from app.zmq_client import zmq_client

router = APIRouter()
context = zmq.asyncio.Context()

@router.websocket("/ws")
async def scada_websocket(websocket: WebSocket):
    await websocket.accept()
    print(f"L2: UI Client Connected. Attempting ZMQ link to Robot @ {settings.robot_ip}")
    
    sub_socket = context.socket(zmq.SUB)
    sub_socket.connect(f"tcp://{settings.robot_ip}:{settings.zmq_telemetry_port}")
    sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
    
    camera_socket = context.socket(zmq.SUB)
    camera_socket.connect(f"tcp://{settings.robot_ip}:{settings.zmq_camera_port}")
    camera_socket.setsockopt_string(zmq.SUBSCRIBE, "")

    async def receive_ws():
        try:
            while True:
                data = await websocket.receive_json()
                if data.get("type") == "cmd_vel":
                    await zmq_client.send_command("cmd_vel", data.get("payload"))
                elif data.get("type") == "nav_goal":
                    await zmq_client.send_command("nav_goal", data.get("payload"))
                elif data.get("type") == "slam_control":
                    await zmq_client.send_command("slam_control", data.get("payload"))
        except WebSocketDisconnect:
            pass
        except Exception as e:
            print(f"WS receive error: {e}")

    async def forward_telemetry():
        try:
            while True:
                msg = await sub_socket.recv_json()
                await websocket.send_json({"type": "telemetry", "payload": msg})
        except asyncio.CancelledError:
            pass
            
    async def forward_camera():
        try:
            while True:
                frame = await camera_socket.recv()
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
