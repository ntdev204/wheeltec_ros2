from fastapi import APIRouter
from fastapi.responses import StreamingResponse
from app.services.session_service import SessionService
from app.services.home_service import HomeService
from app.services.path_service import PathService
from app.services.patrol_service import PatrolService
import io
from fastapi import HTTPException

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
    safe_limit = max(1, min(limit, 500))
    paths = await PathService.get_paths(session_id, safe_limit)
    return {"paths": paths}


@router.get("/paths/csv")
async def export_paths_csv(session_id: int = None):
    csv_data = await PathService.export_csv(session_id)
    return StreamingResponse(
        io.StringIO(csv_data),
        media_type="text/csv",
        headers={"Content-Disposition": "attachment; filename=nav_paths.csv"}
    )


@router.get("/patrol/status")
async def get_patrol_status():
    return await PatrolService.get_status()


@router.post("/patrol/route")
async def save_patrol_route(data: dict):
    try:
        route = await PatrolService.save_route(
            name=str(data.get("name") or "Patrol Route"),
            waypoints=data.get("waypoints") or [],
            map_id=data.get("map_id"),
        )
        return {"route": route}
    except ValueError as exc:
        raise HTTPException(status_code=400, detail="Invalid patrol route payload") from exc


@router.post("/patrol/schedule")
async def update_patrol_schedule(data: dict):
    try:
        schedule = await PatrolService.update_schedule(data)
        return {"schedule": schedule}
    except ValueError as exc:
        raise HTTPException(status_code=400, detail="Invalid patrol schedule payload") from exc


@router.post("/patrol/start")
async def start_patrol(data: dict | None = None):
    try:
        run = await PatrolService.start_run(session_id=await _get_current_session_id(), source="manual")
        return {"run": run}
    except ValueError as exc:
        raise HTTPException(status_code=400, detail="Unable to start patrol") from exc


@router.post("/patrol/stop")
async def stop_patrol(data: dict | None = None):
    payload = data or {}
    try:
        status = await PatrolService.stop_run(
            reason=str(payload.get("reason") or "Stopped from API"),
            session_id=await _get_current_session_id(),
        )
        return status
    except ValueError as exc:
        raise HTTPException(status_code=400, detail="Unable to stop patrol") from exc


async def _get_current_session_id() -> int | None:
    session = await SessionService.get_current_session()
    return session["id"] if session else None

