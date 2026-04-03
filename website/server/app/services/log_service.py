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
                """, (session_id, category, severity, event_type, message, meta_str))
                await db.commit()
        except Exception as e:
            print(f"[LogService] Error saving log: {e}")

    @staticmethod
    async def get_logs(category: str = None, severity: str = None, limit: int = 100, offset: int = 0, session_id: int = None):
        query = "SELECT * FROM event_logs WHERE 1=1"
        params = []
        
        if category:
            query += " AND category = ?"
            params.append(category)
        if severity:
            query += " AND severity = ?"
            params.append(severity)
        if session_id:
            query += " AND session_id = ?"
            params.append(session_id)
            
        query += " ORDER BY timestamp DESC LIMIT ? OFFSET ?"
        params.extend([limit, offset])
        
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                db.row_factory = aiosqlite.Row
                cursor = await db.execute(query, params)
                rows = await cursor.fetchall()
                logs = []
                for row in rows:
                    log_dict = dict(row)
                    if log_dict.get('metadata'):
                        try:
                            log_dict['metadata'] = json.loads(log_dict['metadata'])
                        except:
                            pass
                    logs.append(log_dict)
                return logs
        except Exception as e:
            print(f"[LogService] Error getting logs: {e}")
            return []

    @staticmethod
    async def get_log_stats():
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                db.row_factory = aiosqlite.Row
                # Severity counts
                cursor = await db.execute("SELECT severity, COUNT(*) as count FROM event_logs GROUP BY severity")
                severities = {row['severity']: row['count'] for row in await cursor.fetchall()}
                
                # Category counts
                cursor = await db.execute("SELECT category, COUNT(*) as count FROM event_logs GROUP BY category")
                categories = {row['category']: row['count'] for row in await cursor.fetchall()}
                
                return {
                    "severities": severities,
                    "categories": categories,
                    "total": sum(severities.values())
                }
        except Exception as e:
            print(f"[LogService] Error getting log stats: {e}")
            return {"severities": {}, "categories": {}, "total": 0}
