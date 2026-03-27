from fastapi import APIRouter

router = APIRouter(prefix="/api/robot", tags=["robot"])

@router.get("/status")
async def get_status():
    return {"status": "online"}
