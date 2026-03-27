from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from app.db.models import init_db
from app.ws.handler import router as ws_router
from app.routes.maps import router as maps_router
from app.routes.robot import router as robot_router
from app.routes.analytics import router as analytics_router

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Initialize the SQLite database schema
    await init_db()
    yield
    # Shutdown logic can go here

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
