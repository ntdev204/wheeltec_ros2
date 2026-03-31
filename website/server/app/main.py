from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import asyncio
import zmq
import zmq.asyncio
from app.config import settings
from app.db.models import init_db
from app.ws.handler import router as ws_router
from app.routes.maps import router as maps_router, update_live_map_png
from app.routes.robot import router as robot_router
from app.routes.analytics import router as analytics_router

async def zmq_background_listener():
    context = zmq.asyncio.Context()
    camera_socket = context.socket(zmq.SUB)
    camera_socket.connect(f"tcp://{settings.robot_ip}:{settings.zmq_camera_port}")
    camera_socket.setsockopt_string(zmq.SUBSCRIBE, "")
    print(f"[Main] ZMQ Background Listener connected to udp/tcp://{settings.robot_ip}:{settings.zmq_camera_port}")
    try:
        while True:
            frame = await camera_socket.recv()
            if frame.startswith(b'MAP:'):
                update_live_map_png(frame[4:])
    except asyncio.CancelledError:
        print("[Main] ZMQ Background Listener shutting down.")
    finally:
        camera_socket.close()

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Initialize the SQLite database schema
    await init_db()
    
    # Start ZMQ background listener for Live Map
    listener_task = asyncio.create_task(zmq_background_listener())
    
    yield
    # Shutdown logic can go here
    listener_task.cancel()
    try:
        await listener_task
    except asyncio.CancelledError:
        pass
app = FastAPI(title="Wheeltec SCADA API", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include WebSocket routes
app.include_router(ws_router)
app.include_router(maps_router)
app.include_router(robot_router)
app.include_router(analytics_router)

@app.get("/api/health")
async def health_check():
    return {"status": "ok", "layer": "FastAPI Business Layer"}
