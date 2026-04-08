"""
Coverage route API endpoints.
"""
from fastapi import APIRouter, HTTPException

from app.services.coverage_service import CoverageService

router = APIRouter(prefix="/api/robot/coverage", tags=["coverage"])


@router.post("/generate")
async def generate_coverage_route(data: dict):
    """Generate full-map coverage route."""
    try:
        route = await CoverageService.generate_coverage_route(
            name=data.get("name"),
            robot_width=float(data.get("robot_width", 0.5)),
            overlap=float(data.get("overlap", 0.1)),
            pattern=str(data.get("pattern", "boustrophedon"))
        )
        return {"route": route}
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=500, detail="Coverage generation failed") from exc


@router.get("/routes")
async def get_coverage_routes(limit: int = 50):
    """Get all coverage routes."""
    routes = await CoverageService.get_coverage_routes(limit=limit)
    return {"routes": routes}


@router.get("/statistics")
async def get_coverage_statistics():
    """Get coverage route statistics."""
    stats = await CoverageService.get_coverage_statistics()
    return stats


@router.post("/estimate")
async def estimate_coverage_time(data: dict):
    """Estimate time to complete coverage route."""
    waypoint_count = int(data.get("waypoint_count", 0))
    avg_speed = float(data.get("avg_speed", 0.3))

    if waypoint_count <= 0:
        raise HTTPException(status_code=400, detail="Invalid waypoint count")

    estimated_seconds = await CoverageService.estimate_coverage_time(waypoint_count, avg_speed)

    return {
        "waypoint_count": waypoint_count,
        "estimated_seconds": estimated_seconds,
        "estimated_minutes": estimated_seconds / 60.0
    }
