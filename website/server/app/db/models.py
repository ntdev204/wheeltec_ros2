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
        await db.execute("""
            CREATE TABLE IF NOT EXISTS home_position (
                id INTEGER PRIMARY KEY CHECK (id = 1),
                x REAL NOT NULL,
                y REAL NOT NULL,
                yaw REAL DEFAULT 0,
                updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        """)
        await db.execute("""
            CREATE TABLE IF NOT EXISTS nav_paths (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                session_id INTEGER REFERENCES sessions(id),
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                goal_x REAL,
                goal_y REAL,
                global_plan TEXT,
                local_plan TEXT,
                real_path TEXT,
                status TEXT DEFAULT 'active'
            )
        """)
        await db.execute("CREATE INDEX IF NOT EXISTS idx_nav_paths_session ON nav_paths(session_id)")

        await db.execute("""
            CREATE TABLE IF NOT EXISTS patrol_routes (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                map_id INTEGER REFERENCES maps(id),
                waypoints_json TEXT NOT NULL,
                is_active BOOLEAN DEFAULT 1,
                created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
                updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        """)
        await db.execute("CREATE INDEX IF NOT EXISTS idx_patrol_routes_active ON patrol_routes(is_active)")

        await db.execute("""
            CREATE TABLE IF NOT EXISTS patrol_schedules (
                id INTEGER PRIMARY KEY CHECK (id = 1),
                route_id INTEGER REFERENCES patrol_routes(id),
                enabled BOOLEAN DEFAULT 0,
                interval_minutes INTEGER DEFAULT 30,
                loops_per_run INTEGER DEFAULT 5,
                start_from_home BOOLEAN DEFAULT 1,
                return_to_home BOOLEAN DEFAULT 1,
                last_triggered_at DATETIME,
                next_trigger_at DATETIME,
                created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
                updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        """)

        await db.execute("""
            CREATE TABLE IF NOT EXISTS patrol_runs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                schedule_id INTEGER REFERENCES patrol_schedules(id),
                route_id INTEGER REFERENCES patrol_routes(id),
                session_id INTEGER REFERENCES sessions(id),
                status TEXT DEFAULT 'pending',
                current_loop INTEGER DEFAULT 0,
                total_loops INTEGER DEFAULT 5,
                current_waypoint_index INTEGER DEFAULT -1,
                started_at DATETIME,
                ended_at DATETIME,
                failure_reason TEXT,
                started_from_home_x REAL,
                started_from_home_y REAL,
                started_from_home_yaw REAL DEFAULT 0,
                ended_at_home BOOLEAN DEFAULT 0,
                updated_at DATETIME DEFAULT CURRENT_TIMESTAMP
            )
        """)
        await db.execute("CREATE INDEX IF NOT EXISTS idx_patrol_runs_status ON patrol_runs(status)")
        await db.execute("CREATE INDEX IF NOT EXISTS idx_patrol_runs_started_at ON patrol_runs(started_at)")

        await db.commit()

