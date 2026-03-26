from fastapi import APIRouter

router = APIRouter(prefix="/api/analytics", tags=["analytics"])

@router.get("/sessions")
async def get_sessions():
    return {"sessions": []}
