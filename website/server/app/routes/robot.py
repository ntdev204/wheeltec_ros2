from fastapi import APIRouter
from fastapi.responses import StreamingResponse
from app.services.session_service import SessionService
from app.services.home_service import HomeService
from app.services.path_service import PathService
import io

router = APIRouter(prefix="/api/robot", tags=["robot"])


@router.get("/status")
async def get_status():
    session = await SessionService.get_current_session()
    return {
        "status": "online" if session else "idle",
        "has_active_session": bool(session)
    }


@router.get("/home")
async def get_home():
    home = await HomeService.get_home()
    if home:
        return {"home": home}
    return {"home": None, "message": "Home position not set"}


@router.post("/home")
async def set_home(data: dict):
    x = float(data.get("x", 0))
    y = float(data.get("y", 0))
    yaw = float(data.get("yaw", 0))
    ok = await HomeService.set_home(x, y, yaw)
    if ok:
        return {"status": "ok", "home": {"x": x, "y": y, "yaw": yaw}}
    return {"status": "error", "message": "Failed to save home position"}


@router.get("/paths")
async def get_paths(session_id: int = None, limit: int = 50):
    paths = await PathService.get_paths(session_id, limit)
    return {"paths": paths}


@router.get("/paths/csv")
async def export_paths_csv(session_id: int = None):
    csv_data = await PathService.export_csv(session_id)
    return StreamingResponse(
        io.StringIO(csv_data),
        media_type="text/csv",
        headers={"Content-Disposition": "attachment; filename=nav_paths.csv"}
    )

