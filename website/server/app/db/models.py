import aiosqlite
from app.config import settings

async def init_db():
    async with aiosqlite.connect(settings.db_path) as db:
        await db.execute("""
            CREATE TABLE IF NOT EXISTS maps (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                resolution REAL DEFAULT 0.05,
                width INTEGER, height INTEGER,
                origin_x REAL, origin_y REAL,
                pgm_path TEXT, yaml_path TEXT,
                thumbnail BLOB,
                created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
                is_active BOOLEAN DEFAULT 0
            )
        """)
        await db.execute("""
            CREATE TABLE IF NOT EXISTS sessions (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                started_at DATETIME DEFAULT CURRENT_TIMESTAMP,
                ended_at DATETIME,
                total_distance REAL DEFAULT 0,
                max_speed REAL DEFAULT 0,
                avg_voltage REAL,
                emergency_stops INTEGER DEFAULT 0
            )
        """)
        await db.execute("""
            CREATE TABLE IF NOT EXISTS event_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                session_id INTEGER REFERENCES sessions(id),
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                category TEXT NOT NULL,
                severity TEXT DEFAULT 'INFO',
                event_type TEXT NOT NULL,
                message TEXT NOT NULL,
                metadata TEXT
            )
        """)
        # Create indexes for Logs
        await db.execute("CREATE INDEX IF NOT EXISTS idx_logs_category ON event_logs(category)")
        await db.execute("CREATE INDEX IF NOT EXISTS idx_logs_timestamp ON event_logs(timestamp)")
        await db.execute("CREATE INDEX IF NOT EXISTS idx_logs_session ON event_logs(session_id)")
        
        await db.execute("""
            CREATE TABLE IF NOT EXISTS telemetry_snapshots (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                session_id INTEGER REFERENCES sessions(id),
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                pos_x REAL, pos_y REAL, yaw REAL,
                vel_x REAL, vel_y REAL, vel_z REAL,
                voltage REAL,
                imu_ax REAL, imu_ay REAL, imu_az REAL,
                charging BOOLEAN DEFAULT 0
            )
        """)
        await db.commit()

