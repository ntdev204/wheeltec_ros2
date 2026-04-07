import json
import io
import csv
import aiosqlite
from app.config import settings

MAX_REAL_PATH_POINTS = 5000


class PathService:

    @staticmethod
    async def create_path(session_id: int, goal_x: float, goal_y: float, global_plan: list) -> int | None:
        """Create a new nav_path record when a nav_goal is sent."""
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                # Mark any previous active path as completed
                await db.execute(
                    "UPDATE nav_paths SET status = 'completed' WHERE session_id = ? AND status = 'active'",
                    (session_id,)
                )
                cursor = await db.execute(
                    "INSERT INTO nav_paths (session_id, goal_x, goal_y, global_plan, real_path) VALUES (?, ?, ?, ?, ?)",
                    (session_id, goal_x, goal_y, json.dumps(global_plan), json.dumps([]))
                )
                await db.commit()
                return cursor.lastrowid
        except Exception as e:
            print(f"[PathService] Error creating path: {e}")
            return None

    @staticmethod
    async def update_global_plan(path_id: int, global_plan: list):
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                await db.execute(
                    "UPDATE nav_paths SET global_plan = ? WHERE id = ?",
                    (json.dumps(global_plan), path_id)
                )
                await db.commit()
        except Exception as e:
            print(f"[PathService] Error updating global plan: {e}")

    @staticmethod
    async def update_local_plan(path_id: int, local_plan: list):
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                await db.execute(
                    "UPDATE nav_paths SET local_plan = ? WHERE id = ?",
                    (json.dumps(local_plan), path_id)
                )
                await db.commit()
        except Exception as e:
            print(f"[PathService] Error updating local plan: {e}")

    @staticmethod
    async def append_real_point(path_id: int, x: float, y: float):
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                cursor = await db.execute("SELECT real_path FROM nav_paths WHERE id = ?", (path_id,))
                row = await cursor.fetchone()
                if not row:
                    return

                points = json.loads(row[0] or "[]")
                if len(points) >= MAX_REAL_PATH_POINTS:
                    return

                points.append({"x": round(x, 4), "y": round(y, 4)})
                await db.execute(
                    "UPDATE nav_paths SET real_path = ? WHERE id = ?",
                    (json.dumps(points), path_id)
                )
                await db.commit()
        except Exception as e:
            print(f"[PathService] Error appending real point: {e}")

    @staticmethod
    async def complete_path(path_id: int):
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                await db.execute(
                    "UPDATE nav_paths SET status = 'completed' WHERE id = ?",
                    (path_id,)
                )
                await db.commit()
        except Exception as e:
            print(f"[PathService] Error completing path: {e}")

    @staticmethod
    async def get_paths(session_id: int = None, limit: int = 50):
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                db.row_factory = aiosqlite.Row
                if session_id:
                    cursor = await db.execute(
                        "SELECT id, session_id, timestamp, goal_x, goal_y, status FROM nav_paths WHERE session_id = ? ORDER BY timestamp DESC LIMIT ?",
                        (session_id, limit)
                    )
                else:
                    cursor = await db.execute(
                        "SELECT id, session_id, timestamp, goal_x, goal_y, status FROM nav_paths ORDER BY timestamp DESC LIMIT ?",
                        (limit,)
                    )
                rows = await cursor.fetchall()
                return [dict(row) for row in rows]
        except Exception as e:
            print(f"[PathService] Error getting paths: {e}")
            return []

    @staticmethod
    async def export_csv(session_id: int = None) -> str:
        """Export all paths as CSV with columns: path_id, timestamp, type, point_index, x, y"""
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                db.row_factory = aiosqlite.Row
                if session_id:
                    cursor = await db.execute(
                        "SELECT * FROM nav_paths WHERE session_id = ? ORDER BY timestamp ASC",
                        (session_id,)
                    )
                else:
                    cursor = await db.execute("SELECT * FROM nav_paths ORDER BY timestamp ASC")
                rows = await cursor.fetchall()

                output = io.StringIO()
                writer = csv.writer(output)
                writer.writerow(["path_id", "timestamp", "goal_x", "goal_y", "type", "point_index", "x", "y"])

                for row in rows:
                    r = dict(row)
                    ts = r["timestamp"]
                    gx = r["goal_x"]
                    gy = r["goal_y"]
                    pid = r["id"]

                    for ptype, field in [("global", "global_plan"), ("real", "real_path")]:
                        points = json.loads(r.get(field) or "[]")
                        for i, pt in enumerate(points):
                            writer.writerow([pid, ts, gx, gy, ptype, i, pt.get("x", 0), pt.get("y", 0)])

                return output.getvalue()
        except Exception as e:
            print(f"[PathService] Error exporting CSV: {e}")
            return ""
