from fastapi import APIRouter
from fastapi.responses import Response
from app.db.queries import get_all_maps, get_active_map
import os
import yaml
from PIL import Image
import io

router = APIRouter(prefix="/api/maps", tags=["maps"])

MAP_PATH_PGM = r'E:\tailieu\UTC\NCKH\wheeltec_ros2\data\map\WHEELTEC.pgm'
MAP_PATH_YAML = r'E:\tailieu\UTC\NCKH\wheeltec_ros2\data\map\WHEELTEC.yaml'
LIVE_MAP_PATH = r'E:\tailieu\UTC\NCKH\wheeltec_ros2\data\map\live_map.png'

# Shared cache - PNG bytes from ZMQ background listener
live_map_cache = {"png": None}

def update_live_map_png(png_bytes: bytes):
    """Called from main.py background task when a map PNG arrives from ZMQ."""
    live_map_cache["png"] = png_bytes
    try:
        with open(LIVE_MAP_PATH, 'wb') as f:
            f.write(png_bytes)
    except Exception as e:
        print(f"[MapCache] Error saving live map to disk: {e}")

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
    if live_map_cache["png"]:
        return Response(content=live_map_cache["png"], media_type="image/png")
    
    if os.path.exists(LIVE_MAP_PATH):
        try:
            with open(LIVE_MAP_PATH, 'rb') as f:
                return Response(content=f.read(), media_type="image/png")
        except Exception as e:
            print(f"[MapCache] Error reading live map: {e}")
            
    return Response(status_code=404, content="Live map not available")

@router.post("/live/trigger")
async def trigger_map_resend():
    """Gửi lệnh cho ROS2 node gửi lại map PNG qua ZMQ."""
    from app.zmq_client import zmq_client
    try:
        result = await zmq_client.send_command("resend_map", {})
        return {"status": "ok", "robot_response": result}
    except Exception as e:
        return {"status": "error", "detail": str(e)}

@router.get("/live/info")
async def get_live_map_info():
    """Fallback endpoint for frontend to get map dimensions if telemetry map_info is missing."""
    if not os.path.exists(MAP_PATH_YAML) or not os.path.exists(MAP_PATH_PGM):
        return Response(status_code=404, content="Map files not found")
        
    try:
        with open(MAP_PATH_YAML, 'r') as f:
            data = yaml.safe_load(f)
            
        img = Image.open(MAP_PATH_PGM)
        width, height = img.size
        
        return {
            "resolution": data.get("resolution", 0.05),
            "width": width,
            "height": height,
            "origin": {
                "x": data.get("origin", [0, 0, 0])[0],
                "y": data.get("origin", [0, 0, 0])[1]
            }
        }
    except Exception as e:
        print(f"[MapCache] Error reading map info: {e}")
        return Response(status_code=500, content=str(e))


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
