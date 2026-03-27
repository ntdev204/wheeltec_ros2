import zmq
import zmq.asyncio
from app.config import settings

class ZMQClient:
    def __init__(self):
        self.context = zmq.asyncio.Context()
        self.cmd_socket = self.context.socket(zmq.REQ)
        self.cmd_socket.connect(f"tcp://{settings.robot_ip}:{settings.zmq_cmd_port}")
        
    async def send_command(self, action: str, payload: dict = None):
        if payload is None:
            payload = {}
        request = {"action": action, "payload": payload}
        await self.cmd_socket.send_json(request)
        response = await self.cmd_socket.recv_json()
        return response

zmq_client = ZMQClient()
