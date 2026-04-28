import time
import aiosqlite
import math
from app.config import settings

class TelemetryService:
    _last_save_time = 0
    SAVE_INTERVAL = 5
    
    _last_pos = None

    @staticmethod
    async def maybe_save_snapshot(telemetry_data: dict, session_id: int = None):
        current_time = time.time()
        
        odom = telemetry_data.get("odom", {})
        if TelemetryService._last_pos:
            dx = odom.get("x", 0) - TelemetryService._last_pos["x"]
            dy = odom.get("y", 0) - TelemetryService._last_pos["y"]
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist > 0.01 and session_id:
                from app.services.session_service import SessionService
                speed = math.sqrt(odom.get("v_x", 0)**2 + odom.get("v_y", 0)**2)
                await SessionService.update_session_stats(session_id, distance_added=dist, current_speed=speed)
        
        TelemetryService._last_pos = {"x": odom.get("x", 0), "y": odom.get("y", 0)}
        
        if current_time - TelemetryService._last_save_time < TelemetryService.SAVE_INTERVAL:
            return False
            
        TelemetryService._last_save_time = current_time
        
        try:
            async with aiosqlite.connect(settings.db_path) as db:
                await db.execute("""
                    INSERT INTO telemetry_snapshots 
                    (session_id, pos_x, pos_y, yaw, vel_x, vel_y, vel_z, voltage, imu_ax, imu_ay, imu_az, charging)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    SELECT 
                        strftime('%H:%M', timestamp) as time_label,
                        timestamp,
                        AVG(voltage) as voltage
                    FROM telemetry_snapshots
                    WHERE timestamp >= datetime('now', '-{hours} hours')
                    GROUP BY strftime('%Y-%m-%d %H:%M', timestamp)
                    ORDER BY timestamp ASC