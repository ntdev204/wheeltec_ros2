from fastapi import APIRouter
from fastapi.responses import Response
from app.db.queries import get_all_maps, get_active_map
import os
import yaml
from PIL import Image
import io

router = APIRouter(prefix="/api/maps", tags=["maps"])

MAP_PATH_PGM = r'D:\wheeltec_ros2\data\map\WHEELTEC.pgm'
MAP_PATH_YAML = r'D:\wheeltec_ros2\data\map\WHEELTEC.yaml'

# Shared cache - PNG bytes from Robot bridge
live_map_cache = {"png": None}

def update_live_map_png(png_bytes: bytes):
    """Called from ws/handler.py when a map PNG arrives from the bridge."""
    live_map_cache["png"] = png_bytes
    print(f"[MapCache] Updated PNG cache: {len(png_bytes)} bytes")

@router.get("/")
async def list_maps():
    maps = await get_all_maps()
    return {"maps": maps}

@router.get("/active")
async def active_map():
    map_data = await get_active_map()
    return {"map": map_data}

@router.get("/live/image")
async def get_live_map_image():
    if live_map_cache["png"] is None:
        return Response(status_code=404, content="Live map not yet available")
    return Response(content=live_map_cache["png"], media_type="image/png")

@router.get("/static/image")
async def get_static_map_image():
    if not os.path.exists(MAP_PATH_PGM):
        return Response(status_code=404, content="Map file not found")
    img = Image.open(MAP_PATH_PGM)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return Response(content=buf.getvalue(), media_type="image/png")

@router.get("/static/yaml")
async def get_static_map_yaml():
    if not os.path.exists(MAP_PATH_YAML):
        return Response(status_code=404, content="YAML file not found")
    with open(MAP_PATH_YAML, 'r') as f:
        data = yaml.safe_load(f)
    return data
