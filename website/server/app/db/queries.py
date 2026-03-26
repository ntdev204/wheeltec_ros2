import aiosqlite
from app.config import settings

async def get_all_maps():
    async with aiosqlite.connect(settings.db_path) as db:
        db.row_factory = aiosqlite.Row
        cursor = await db.execute("SELECT * FROM maps ORDER BY created_at DESC")
        rows = await cursor.fetchall()
        return [dict(row) for row in rows]
        
async def get_active_map():
    async with aiosqlite.connect(settings.db_path) as db:
        db.row_factory = aiosqlite.Row
        cursor = await db.execute("SELECT * FROM maps WHERE is_active = 1 LIMIT 1")
        row = await cursor.fetchone()
        return dict(row) if row else None
