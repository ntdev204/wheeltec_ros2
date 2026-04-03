import aiosqlite
from app.config import settings

class SessionService:
    @staticmethod
    async def get_current_session():
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                db.row_factory = aiosqlite.Row
                # Phiên hiện tại là phiên có ended_at = NULL (hoặc mới nhất)
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
        # Kiểm tra xem có session nào đang chạy không, nếu có thì kết thúc trước
        current = await SessionService.get_current_session()
        if current:
            await SessionService.end_session(current['id'])
            
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                cursor = await db.execute("""
                    INSERT INTO sessions (started_at, total_distance, max_speed, emergency_stops)
                    VALUES (CURRENT_TIMESTAMP, 0, 0, 0)
                """)
                await db.commit()
                return cursor.lastrowid
        except Exception as e:
            print(f"[SessionService] Error starting session: {e}")
            return None

    @staticmethod
    async def end_session(session_id: int):
        if not session_id:
            return
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                await db.execute("UPDATE sessions SET ended_at = CURRENT_TIMESTAMP WHERE id = ?", (session_id,))
                await db.commit()
        except Exception as e:
            print(f"[SessionService] Error ending session: {e}")

    @staticmethod
    async def update_session_stats(session_id: int, distance_added: float = 0, current_speed: float = 0, e_stop_added: int = 0):
        if not session_id:
            return
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                # Lấy max speed hiện tại
                db.row_factory = aiosqlite.Row
                cursor = await db.execute("SELECT max_speed FROM sessions WHERE id = ?", (session_id,))
                row = await cursor.fetchone()
                max_speed = row['max_speed'] if row else 0
                new_max_speed = max(max_speed, current_speed)
                
                await db.execute("""
                    UPDATE sessions 
                    SET total_distance = total_distance + ?,
                        max_speed = ?,
                        emergency_stops = emergency_stops + ?
                    WHERE id = ?
                """, (distance_added, new_max_speed, e_stop_added, session_id))
                await db.commit()
        except Exception as e:
            print(f"[SessionService] Error updating session stats: {e}")
