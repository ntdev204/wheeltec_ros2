import aiosqlite
from app.config import settings

class SessionService:
    @staticmethod
    async def get_current_session():
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                db.row_factory = aiosqlite.Row
                cursor = await db.execute("SELECT * FROM sessions WHERE ended_at IS NULL ORDER BY started_at DESC LIMIT 1")
                row = await cursor.fetchone()
                if row:
                    return dict(row)
                return None
        except Exception as e:
            print(f"[SessionService] Error getting current session: {e}")
            return None

    @staticmethod
    async def start_session():
        current = await SessionService.get_current_session()
        if current:
            await SessionService.end_session(current['id'])
            
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                cursor = await db.execute("""
                    INSERT INTO sessions (started_at, total_distance, max_speed, emergency_stops)
                    VALUES (CURRENT_TIMESTAMP, 0, 0, 0)
                    UPDATE sessions 
                    SET total_distance = total_distance + ?,
                        max_speed = ?,
                        emergency_stops = emergency_stops + ?
                    WHERE id = ?