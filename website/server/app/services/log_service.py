import aiosqlite
from app.config import settings
import json
from datetime import datetime

class LogService:
    CATEGORIES = ["NAVIGATION", "POWER", "COMMAND", "SYSTEM", "TELEMETRY"]
    SEVERITIES = ["INFO", "WARNING", "ERROR", "CRITICAL"]

    @staticmethod
    async def log_event(category: str, event_type: str, message: str, severity: str = "INFO", metadata: dict = None, session_id: int = None):
        if category not in LogService.CATEGORIES:
            category = "SYSTEM"
        if severity not in LogService.SEVERITIES:
            severity = "INFO"
        
        meta_str = json.dumps(metadata) if metadata else None
        
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                await db.execute("""
                    INSERT INTO event_logs (session_id, category, severity, event_type, message, metadata)
                    VALUES (?, ?, ?, ?, ?, ?)