import aiosqlite
from app.config import settings

BATTERY_MIN = 21.0
BATTERY_MAX = 25.2
WARN_THRESHOLD = 20
RETURN_THRESHOLD = 10


def voltage_to_percent(voltage: float) -> float:
    if voltage <= 0:
        return 0.0
    pct = ((voltage - BATTERY_MIN) / (BATTERY_MAX - BATTERY_MIN)) * 100
    return max(0.0, min(100.0, pct))


class HomeService:
    @staticmethod
    async def get_home() -> dict | None:
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                db.row_factory = aiosqlite.Row
                cursor = await db.execute("SELECT x, y, yaw FROM home_position WHERE id = 1")
                row = await cursor.fetchone()
                if row:
                    return {"x": row["x"], "y": row["y"], "yaw": row["yaw"]}
                return None
        except Exception as e:
            print(f"[HomeService] Error reading home: {e}")
            return None

    @staticmethod
    async def set_home(x: float, y: float, yaw: float = 0.0) -> bool:
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                await db.execute("""
                    INSERT INTO home_position (id, x, y, yaw, updated_at)
                    VALUES (1, ?, ?, ?, CURRENT_TIMESTAMP)
                    ON CONFLICT(id) DO UPDATE SET
                        x = excluded.x,
                        y = excluded.y,
                        yaw = excluded.yaw,
                        updated_at = CURRENT_TIMESTAMP