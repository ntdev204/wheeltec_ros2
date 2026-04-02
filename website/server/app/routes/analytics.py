from fastapi import APIRouter
from app.services.session_service import SessionService
from app.services.telemetry_service import TelemetryService
from app.services.log_service import LogService

router = APIRouter(prefix="/api/analytics", tags=["analytics"])

@router.get("/sessions/current")
async def get_current_session():
    session = await SessionService.get_current_session()
    return {"session": session}

@router.get("/voltage-history")
async def get_voltage_history(hours: int = 24):
    history = await TelemetryService.get_voltage_history(hours)
    return {"history": history}

@router.get("/summary")
async def get_dashboard_summary():
    # Gather KPIs for the dashboard
    session = await SessionService.get_current_session()
    log_stats = await LogService.get_log_stats()
    
    return {
        "active_session": session,
        "log_stats": log_stats
    }
