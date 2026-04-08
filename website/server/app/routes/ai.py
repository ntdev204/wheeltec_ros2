"""AI detection and tracking routes."""

from fastapi import APIRouter, HTTPException
from typing import List, Dict, Optional
import sqlite3
from pathlib import Path
from app.config import settings

router = APIRouter(prefix="/api/ai", tags=["ai"])


@router.get("/detections/stats")
async def get_detection_stats():
    """Get AI detection statistics."""
    try:
        # This would be populated by the detection node
        # For now, return placeholder data
        return {
            "fps": 0.0,
            "latency_ms": 0.0,
            "total_detections": 0,
            "detections_by_class": {}
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/tracking/stats")
async def get_tracking_stats():
    """Get human tracking statistics."""
    try:
        return {
            "active_tracks": 0,
            "total_humans_tracked": 0,
            "average_track_duration": 0.0
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/images")
async def get_images(limit: int = 50, offset: int = 0):
    """Get stored images with metadata."""
    try:
        db_path = "/home/jetson/wheeltec_data/images/metadata.db"

        if not Path(db_path).exists():
            return {"images": [], "total": 0}

        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()

        # Get total count
        cursor.execute("SELECT COUNT(*) FROM images")
        total = cursor.fetchone()[0]

        # Get images with pagination
        cursor.execute("""
            SELECT id, timestamp, frame_id, raw_path, annotated_path,
                   object_count, classes, created_at
            FROM images
            ORDER BY created_at DESC
            LIMIT ? OFFSET ?
        """, (limit, offset))

        images = []
        for row in cursor.fetchall():
            images.append({
                "id": row[0],
                "timestamp": row[1],
                "frame_id": row[2],
                "raw_path": row[3],
                "annotated_path": row[4],
                "object_count": row[5],
                "classes": row[6].split(',') if row[6] else [],
                "created_at": row[7]
            })

        conn.close()

        return {
            "images": images,
            "total": total,
            "limit": limit,
            "offset": offset
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.delete("/images/clear")
async def clear_images():
    """Clear all stored images."""
    try:
        base_path = Path("/home/jetson/wheeltec_data/images")

        # Delete all images
        for img_type in ["raw", "annotated"]:
            img_dir = base_path / img_type
            if img_dir.exists():
                for img_file in img_dir.glob("*.jpg"):
                    img_file.unlink()

        # Clear database
        db_path = base_path / "metadata.db"
        if db_path.exists():
            conn = sqlite3.connect(str(db_path))
            cursor = conn.cursor()
            cursor.execute("DELETE FROM images")
            conn.commit()
            conn.close()

        return {"status": "success", "message": "All images cleared"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
