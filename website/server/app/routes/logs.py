from fastapi import APIRouter, Query
from app.services.log_service import LogService

router = APIRouter(prefix="/api/logs", tags=["logs"])

@router.get("/")
async def get_logs(
    category: str = None,
    severity: str = None,
    session_id: int = None,
    limit: int = Query(100, le=1000),
    offset: int = 0
):
    logs = await LogService.get_logs(category, severity, limit, offset, session_id)
    return {"logs": logs}

@router.get("/latest")
async def get_latest_logs(n: int = Query(50, le=200)):
    logs = await LogService.get_logs(limit=n)
    return {"logs": logs}

@router.get("/stats")
async def get_log_stats():
    stats = await LogService.get_log_stats()
    return stats
