import asyncio
import zmq
import zmq.asyncio
from app.config import settings


class ZMQClient:
    def __init__(self):
        self.context = zmq.asyncio.Context()
        self.cmd_socket = self.context.socket(zmq.REQ)
        self.cmd_socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5s timeout
        self.cmd_socket.setsockopt(zmq.SNDTIMEO, 3000)  # 3s timeout
        self.cmd_socket.setsockopt(zmq.LINGER, 0)
        self.cmd_socket.connect(f"tcp://{settings.robot_ip}:{settings.zmq_cmd_port}")
        self._lock = asyncio.Lock()

    async def send_command(self, action: str, payload: dict = None):
        if payload is None:
            payload = {}
        request = {"action": action, "payload": payload}
        async with self._lock:
            try:
                await self.cmd_socket.send_json(request)
                response = await self.cmd_socket.recv_json()
                return response
            except zmq.ZMQError as e:
                # Socket in bad state — recreate it
                self.cmd_socket.close()
                self.cmd_socket = self.context.socket(zmq.REQ)
                self.cmd_socket.setsockopt(zmq.RCVTIMEO, 5000)
                self.cmd_socket.setsockopt(zmq.SNDTIMEO, 3000)
                self.cmd_socket.setsockopt(zmq.LINGER, 0)
                self.cmd_socket.connect(
                    f"tcp://{settings.robot_ip}:{settings.zmq_cmd_port}"
                )
                return {"status": "error", "message": str(e)}


zmq_client = ZMQClient()
