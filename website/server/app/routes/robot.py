from fastapi import APIRouter
from app.services.session_service import SessionService

router = APIRouter(prefix="/api/robot", tags=["robot"])

@router.get("/status")
async def get_status():
    session = await SessionService.get_current_session()
    return {
        "status": "online" if session else "idle",
        "has_active_session": bool(session)
    }
