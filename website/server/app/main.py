from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import asyncio
import zmq
import zmq.asyncio
from app.config import settings
from app.db.models import init_db
from app.ws.handler import router as ws_router
from app.routes.maps import router as maps_router
from app.routes.robot import router as robot_router
from app.routes.coverage import router as coverage_router
from app.routes.analytics import router as analytics_router
from app.routes.logs import router as logs_router
from app.services.session_service import SessionService
from app.services.patrol_service import PatrolService

async def zmq_background_listener():
    context = zmq.asyncio.Context()
    telemetry_socket = context.socket(zmq.SUB)
    telemetry_socket.connect(f"tcp://{settings.robot_ip}:{settings.zmq_telemetry_port}")
    telemetry_socket.setsockopt_string(zmq.SUBSCRIBE, "")
    print(f"[Main] ZMQ Background Listener connected to tcp://{settings.robot_ip}:{settings.zmq_telemetry_port}")
    try:
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        print("[Main] ZMQ Background Listener shutting down.")
    finally:
        telemetry_socket.close()

@asynccontextmanager
async def lifespan(app: FastAPI):
    await init_db()

    session_id = await SessionService.start_session()
    print(f"[Main] Started new SCADA session: {session_id}")

    await PatrolService.ensure_default_schedule()

    listener_task = asyncio.create_task(zmq_background_listener())
    patrol_scheduler_task = asyncio.create_task(PatrolService.scheduler_loop(session_id))

    yield
    if session_id:
        await SessionService.end_session(session_id)
        print(f"[Main] Ended SCADA session: {session_id}")

    listener_task.cancel()
    patrol_scheduler_task.cancel()
    try:
        await listener_task
    except asyncio.CancelledError:
        pass
    try:
        await patrol_scheduler_task
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

app.include_router(ws_router)
app.include_router(maps_router)
app.include_router(robot_router)
app.include_router(coverage_router)
app.include_router(analytics_router)
app.include_router(logs_router)

@app.get("/api/health")
async def health_check():
    return {"status": "ok", "layer": "FastAPI Business Layer"}
