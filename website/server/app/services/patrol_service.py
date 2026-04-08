import asyncio
import json
import math
from datetime import datetime, timedelta, timezone
from typing import Any

import aiosqlite

from app.config import settings
from app.services.home_service import HomeService
from app.services.log_service import LogService
from app.zmq_client import zmq_client

PATROL_PENDING_STATUSES = {"pending", "starting", "running", "returning_home"}
PATROL_TERMINAL_STATUSES = {"completed", "aborted", "failed", "stopped"}
DEFAULT_INTERVAL_MINUTES = 30
DEFAULT_LOOPS_PER_RUN = 5
MIN_INTERVAL_MINUTES = 1
MAX_INTERVAL_MINUTES = 24 * 60
MIN_LOOPS_PER_RUN = 1
MAX_LOOPS_PER_RUN = 20
MAX_ROUTE_WAYPOINTS = 200
MIN_START_BATTERY_PERCENT = 15.0
WAYPOINT_TOLERANCE_METERS = 0.25


class PatrolService:
    _lock = asyncio.Lock()
    _runtime_state: dict[str, Any] = {
        "connected": False,
        "status": "idle",
        "run_id": None,
        "schedule_id": None,
        "route_id": None,
        "current_loop": 0,
        "total_loops": 0,
        "current_waypoint_index": -1,
        "total_waypoints": 0,
        "last_goal": None,
        "message": None,
        "updated_at": None,
        "battery_pct": None,
        "charging": False,
        "map_pose": None,
    }
    _last_snapshot_signature: str | None = None

    @staticmethod
    def _utcnow() -> datetime:
        return datetime.now(timezone.utc)

    @staticmethod
    def _serialize_dt(value: str | datetime | None) -> str | None:
        if value is None:
            return None
        if isinstance(value, str):
            return value
        return value.isoformat()

    @staticmethod
    def _safe_float(value: Any, field_name: str) -> float:
        try:
            numeric = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Invalid numeric value for {field_name}") from exc
        if not math.isfinite(numeric):
            raise ValueError(f"{field_name} must be a finite number")
        return numeric

    @staticmethod
    def _parse_waypoints(raw: Any) -> list[dict[str, float]]:
        if isinstance(raw, str):
            raw = json.loads(raw)
        if not isinstance(raw, list):
            raise ValueError("Waypoints must be a list")
        if len(raw) > MAX_ROUTE_WAYPOINTS:
            raise ValueError(f"Waypoints cannot exceed {MAX_ROUTE_WAYPOINTS} points")

        waypoints: list[dict[str, float]] = []
        for index, waypoint in enumerate(raw):
            if not isinstance(waypoint, dict):
                raise ValueError(f"Waypoint at index {index} must be an object")
            x = PatrolService._safe_float(waypoint.get("x"), f"waypoints[{index}].x")
            y = PatrolService._safe_float(waypoint.get("y"), f"waypoints[{index}].y")
            yaw = PatrolService._safe_float(waypoint.get("yaw", 0.0), f"waypoints[{index}].yaw")
            waypoints.append({"x": x, "y": y, "yaw": yaw})
        if len(waypoints) < 2:
            raise ValueError("Patrol route must contain at least 2 waypoints")
        return waypoints

    @staticmethod
    async def ensure_default_schedule() -> None:
        async with aiosqlite.connect(settings.db_path) as db:
            await db.execute(
                """
                INSERT INTO patrol_schedules (id, route_id, enabled, interval_minutes, loops_per_run, start_from_home, return_to_home, next_trigger_at)
                VALUES (1, NULL, 0, ?, ?, 1, 1, NULL)
                ON CONFLICT(id) DO NOTHING
                """,
                (DEFAULT_INTERVAL_MINUTES, DEFAULT_LOOPS_PER_RUN),
            )
            await db.commit()

    @staticmethod
    async def get_active_map() -> dict[str, Any] | None:
        async with aiosqlite.connect(settings.db_path) as db:
            db.row_factory = aiosqlite.Row
            cursor = await db.execute("SELECT * FROM maps WHERE is_active = 1 LIMIT 1")
            row = await cursor.fetchone()
            return dict(row) if row else None

    @staticmethod
    async def get_route() -> dict[str, Any] | None:
        async with aiosqlite.connect(settings.db_path) as db:
            db.row_factory = aiosqlite.Row
            cursor = await db.execute(
                "SELECT * FROM patrol_routes WHERE is_active = 1 ORDER BY updated_at DESC, id DESC LIMIT 1"
            )
            row = await cursor.fetchone()
            if not row:
                return None
            route = dict(row)
            route["waypoints"] = json.loads(route["waypoints_json"] or "[]")
            return route

    @staticmethod
    async def save_route(name: str, waypoints: list[dict[str, float]], map_id: int | None = None) -> dict[str, Any]:
        normalized_waypoints = PatrolService._parse_waypoints(waypoints)
        if map_id is None:
            active_map = await PatrolService.get_active_map()
            map_id = active_map["id"] if active_map else None

        async with aiosqlite.connect(settings.db_path) as db:
            await db.execute("UPDATE patrol_routes SET is_active = 0 WHERE is_active = 1")
            cursor = await db.execute(
                """
                INSERT INTO patrol_routes (name, map_id, waypoints_json, is_active, updated_at)
                VALUES (?, ?, ?, 1, CURRENT_TIMESTAMP)
                """,
                (name, map_id, json.dumps(normalized_waypoints)),
            )
            route_id = cursor.lastrowid
            await db.commit()

        await LogService.log_event(
            "NAVIGATION",
            "patrol_route_saved",
            f"Saved patrol route '{name}' with {len(normalized_waypoints)} waypoints",
            metadata={"route_id": route_id, "map_id": map_id, "waypoints": normalized_waypoints},
        )
        route = await PatrolService.get_route()
        if route is None:
            raise RuntimeError("Failed to reload saved patrol route")
        await PatrolService.sync_schedule_route(route["id"])
        return route

    @staticmethod
    async def sync_schedule_route(route_id: int) -> None:
        await PatrolService.ensure_default_schedule()
        async with aiosqlite.connect(settings.db_path) as db:
            await db.execute(
                "UPDATE patrol_schedules SET route_id = COALESCE(route_id, ?) WHERE id = 1",
                (route_id,),
            )
            await db.commit()

    @staticmethod
    async def get_schedule() -> dict[str, Any]:
        await PatrolService.ensure_default_schedule()
        async with aiosqlite.connect(settings.db_path) as db:
            db.row_factory = aiosqlite.Row
            cursor = await db.execute("SELECT * FROM patrol_schedules WHERE id = 1")
            row = await cursor.fetchone()
            schedule = dict(row)
            schedule["enabled"] = bool(schedule["enabled"])
            schedule["start_from_home"] = bool(schedule["start_from_home"])
            schedule["return_to_home"] = bool(schedule["return_to_home"])
            return schedule

    @staticmethod
    async def update_schedule(payload: dict[str, Any]) -> dict[str, Any]:
        schedule = await PatrolService.get_schedule()
        route = await PatrolService.get_route()

        requested_route_id = payload.get("route_id")
        if requested_route_id is not None:
            route_id = int(requested_route_id)
        elif route:
            route_id = int(route["id"])
        else:
            route_id = None
        enabled = bool(payload.get("enabled", schedule["enabled"]))
        interval_minutes = int(payload.get("interval_minutes", schedule["interval_minutes"]))
        loops_per_run = int(payload.get("loops_per_run", schedule["loops_per_run"]))
        start_from_home = bool(payload.get("start_from_home", schedule["start_from_home"]))
        return_to_home = bool(payload.get("return_to_home", schedule["return_to_home"]))

        if not MIN_INTERVAL_MINUTES <= interval_minutes <= MAX_INTERVAL_MINUTES:
            raise ValueError(f"Interval minutes must be between {MIN_INTERVAL_MINUTES} and {MAX_INTERVAL_MINUTES}")
        if not MIN_LOOPS_PER_RUN <= loops_per_run <= MAX_LOOPS_PER_RUN:
            raise ValueError(f"Loops per run must be between {MIN_LOOPS_PER_RUN} and {MAX_LOOPS_PER_RUN}")
        if enabled and route_id is None:
            raise ValueError("Cannot enable patrol schedule without an active route")
        if enabled:
            await PatrolService.validate_start_conditions(route_id=route_id)

        next_trigger_at = schedule.get("next_trigger_at")
        if enabled:
            next_trigger_at = (PatrolService._utcnow() + timedelta(minutes=interval_minutes)).isoformat()
        else:
            next_trigger_at = None

        async with aiosqlite.connect(settings.db_path) as db:
            await db.execute(
                """
                UPDATE patrol_schedules
                SET route_id = ?, enabled = ?, interval_minutes = ?, loops_per_run = ?,
                    start_from_home = ?, return_to_home = ?, next_trigger_at = ?, updated_at = CURRENT_TIMESTAMP
                WHERE id = 1
                """,
                (
                    route_id,
                    int(enabled),
                    interval_minutes,
                    loops_per_run,
                    int(start_from_home),
                    int(return_to_home),
                    next_trigger_at,
                ),
            )
            await db.commit()

        await LogService.log_event(
            "NAVIGATION",
            "patrol_schedule_updated",
            f"Patrol schedule {'enabled' if enabled else 'disabled'}",
            metadata={
                "route_id": route_id,
                "interval_minutes": interval_minutes,
                "loops_per_run": loops_per_run,
                "next_trigger_at": next_trigger_at,
            },
        )
        return await PatrolService.get_schedule()

    @staticmethod
    async def get_latest_run() -> dict[str, Any] | None:
        async with aiosqlite.connect(settings.db_path) as db:
            db.row_factory = aiosqlite.Row
            cursor = await db.execute("SELECT * FROM patrol_runs ORDER BY id DESC LIMIT 1")
            row = await cursor.fetchone()
            return dict(row) if row else None

    @staticmethod
    async def get_active_run() -> dict[str, Any] | None:
        async with aiosqlite.connect(settings.db_path) as db:
            db.row_factory = aiosqlite.Row
            cursor = await db.execute(
                "SELECT * FROM patrol_runs WHERE status IN ('pending', 'starting', 'running', 'returning_home') ORDER BY id DESC LIMIT 1"
            )
            row = await cursor.fetchone()
            return dict(row) if row else None

    @staticmethod
    async def validate_start_conditions(route_id: int | None = None) -> dict[str, Any]:
        route = await PatrolService.get_route()
        if route_id is not None:
            async with aiosqlite.connect(settings.db_path) as db:
                db.row_factory = aiosqlite.Row
                cursor = await db.execute("SELECT * FROM patrol_routes WHERE id = ? LIMIT 1", (route_id,))
                row = await cursor.fetchone()
                route = dict(row) if row else None
                if route is not None:
                    route["waypoints"] = json.loads(route["waypoints_json"] or "[]")
        if route is None:
            raise ValueError("Patrol route is not configured")

        home = await HomeService.get_home()
        if home is None:
            raise ValueError("Home position is not configured")

        active_map = await PatrolService.get_active_map()
        if active_map is None:
            raise ValueError("No active map is configured")
        if route.get("map_id") and route["map_id"] != active_map["id"]:
            raise ValueError("Patrol route does not match the active map")

        runtime = dict(PatrolService._runtime_state)

        battery_pct = runtime.get("battery_pct")
        if battery_pct is not None and battery_pct < MIN_START_BATTERY_PERCENT:
            raise ValueError("Battery level is too low to start patrol")
        if runtime.get("charging"):
            raise ValueError("Robot is charging")
        if runtime.get("status") in {"starting", "running", "returning_home"}:
            raise ValueError("Patrol mission is already running")
        active_run = await PatrolService.get_active_run()
        if active_run is not None:
            raise ValueError("Another patrol run is still active")

        return {"route": route, "home": home, "runtime": runtime}

    @staticmethod
    async def create_run(schedule_id: int, route: dict[str, Any], home: dict[str, Any], session_id: int | None) -> int:
        schedule = await PatrolService.get_schedule()
        async with aiosqlite.connect(settings.db_path) as db:
            cursor = await db.execute(
                """
                INSERT INTO patrol_runs (
                    schedule_id, route_id, session_id, status, current_loop, total_loops,
                    current_waypoint_index, started_at, started_from_home_x, started_from_home_y, started_from_home_yaw, ended_at_home
                )
                VALUES (?, ?, ?, 'pending', 0, ?, -1, CURRENT_TIMESTAMP, ?, ?, ?, 0)
                """,
                (
                    schedule_id,
                    route["id"],
                    session_id,
                    int(schedule["loops_per_run"]),
                    float(home["x"]),
                    float(home["y"]),
                    float(home.get("yaw", 0.0)),
                ),
            )
            run_id = cursor.lastrowid
            await db.commit()
        return run_id

    @staticmethod
    async def start_run(session_id: int | None, source: str) -> dict[str, Any]:
        async with PatrolService._lock:
            validated = await PatrolService.validate_start_conditions()
            route = validated["route"]
            home = validated["home"]
            schedule = await PatrolService.get_schedule()
            run_id = await PatrolService.create_run(schedule_id=1, route=route, home=home, session_id=session_id)

            payload = {
                "run_id": run_id,
                "route_id": route["id"],
                "schedule_id": 1,
                "loops": int(schedule["loops_per_run"]),
                "home": home,
                "waypoints": route["waypoints"],
                "waypoint_tolerance": WAYPOINT_TOLERANCE_METERS,
                "source": source,
            }
            response = await zmq_client.send_command("patrol_start", payload)
            if response.get("status") not in {"ok", "started"}:
                await PatrolService.mark_run_terminal(run_id, "failed", "Patrol executor rejected start request")
                raise ValueError("Failed to start patrol executor")

            PatrolService._runtime_state = {
                **PatrolService._runtime_state,
                "status": "starting",
                "run_id": run_id,
                "route_id": route["id"],
                "schedule_id": 1,
                "current_loop": 0,
                "total_loops": int(schedule["loops_per_run"]),
                "current_waypoint_index": -1,
                "total_waypoints": len(route["waypoints"]),
                "message": "Patrol run requested",
                "updated_at": PatrolService._utcnow().isoformat(),
            }

            next_trigger_at = PatrolService._utcnow() + timedelta(minutes=int(schedule["interval_minutes"]))
            async with aiosqlite.connect(settings.db_path) as db:
                await db.execute(
                    """
                    UPDATE patrol_schedules
                    SET last_triggered_at = CURRENT_TIMESTAMP, next_trigger_at = ?, updated_at = CURRENT_TIMESTAMP
                    WHERE id = 1
                    """,
                    (next_trigger_at.isoformat(),),
                )
                await db.execute(
                    "UPDATE patrol_runs SET status = 'starting' WHERE id = ?",
                    (run_id,),
                )
                await db.commit()

        await LogService.log_event(
            "NAVIGATION",
            "patrol_run_started",
            f"Started patrol run {run_id} from {source}",
            metadata={"run_id": run_id, "route_id": route["id"], "source": source},
            session_id=session_id,
        )
        run = await PatrolService.get_latest_run()
        if run is None:
            raise RuntimeError("Failed to load patrol run after start")
        return run

    @staticmethod
    async def stop_run(reason: str, session_id: int | None = None) -> dict[str, Any]:
        active_run = await PatrolService.get_active_run()
        if active_run is None:
            raise ValueError("No active patrol run")

        response = await zmq_client.send_command("patrol_stop", {"run_id": active_run["id"], "reason": reason})
        if response.get("status") not in {"ok", "stopped"}:
            raise ValueError(response.get("message", "Failed to stop patrol executor"))

        await PatrolService.mark_run_terminal(active_run["id"], "aborted", reason, session_id=session_id)
        return await PatrolService.get_status()

    @staticmethod
    async def mark_run_terminal(run_id: int, status: str, reason: str | None = None, session_id: int | None = None) -> None:
        if status not in PATROL_TERMINAL_STATUSES and status != "aborted":
            status = "failed"
        async with aiosqlite.connect(settings.db_path) as db:
            await db.execute(
                """
                UPDATE patrol_runs
                SET status = ?, ended_at = CURRENT_TIMESTAMP, failure_reason = ?, updated_at = CURRENT_TIMESTAMP
                WHERE id = ?
                """,
                (status, reason, run_id),
            )
            await db.commit()
        await LogService.log_event(
            "NAVIGATION",
            f"patrol_run_{status}",
            f"Patrol run {run_id} ended with status {status}",
            severity="WARNING" if status in {"aborted", "failed", "stopped"} else "INFO",
            metadata={"run_id": run_id, "reason": reason},
            session_id=session_id,
        )

    @staticmethod
    async def process_telemetry(msg: dict[str, Any], session_id: int | None = None) -> None:
        patrol = msg.get("patrol") or {}
        map_pose = msg.get("map_pose")
        battery_pct = msg.get("battery_pct")
        charging = bool(msg.get("charging", False))

        snapshot = {
            "connected": True,
            "status": patrol.get("status", "idle"),
            "run_id": patrol.get("run_id"),
            "schedule_id": patrol.get("schedule_id"),
            "route_id": patrol.get("route_id"),
            "current_loop": patrol.get("current_loop", 0),
            "total_loops": patrol.get("total_loops", 0),
            "current_waypoint_index": patrol.get("current_waypoint_index", -1),
            "total_waypoints": patrol.get("total_waypoints", 0),
            "last_goal": patrol.get("last_goal"),
            "message": patrol.get("message"),
            "updated_at": PatrolService._utcnow().isoformat(),
            "battery_pct": battery_pct,
            "charging": charging,
            "map_pose": map_pose,
        }

        async with PatrolService._lock:
            PatrolService._runtime_state = snapshot

        await PatrolService._sync_run_from_snapshot(snapshot, session_id=session_id)

    @staticmethod
    async def _sync_run_from_snapshot(snapshot: dict[str, Any], session_id: int | None = None) -> None:
        run_id = snapshot.get("run_id")
        if not run_id:
            return
        status = snapshot.get("status", "idle")
        message = snapshot.get("message")
        current_loop = int(snapshot.get("current_loop") or 0)
        current_waypoint_index = int(snapshot.get("current_waypoint_index") or -1)

        previous = await PatrolService.get_active_run()
        async with aiosqlite.connect(settings.db_path) as db:
            if status in {"starting", "running", "returning_home"}:
                await db.execute(
                    """
                    UPDATE patrol_runs
                    SET status = ?, current_loop = ?, current_waypoint_index = ?, updated_at = CURRENT_TIMESTAMP
                    WHERE id = ?
                    """,
                    (status, current_loop, current_waypoint_index, run_id),
                )
            elif status == "completed":
                ended_at_home = 0
                last_goal = snapshot.get("last_goal")
                map_pose = snapshot.get("map_pose")
                if last_goal and map_pose:
                    dx = float(map_pose.get("x", 0.0)) - float(last_goal.get("x", 0.0))
                    dy = float(map_pose.get("y", 0.0)) - float(last_goal.get("y", 0.0))
                    ended_at_home = int((dx * dx + dy * dy) ** 0.5 <= WAYPOINT_TOLERANCE_METERS)
                await db.execute(
                    """
                    UPDATE patrol_runs
                    SET status = 'completed', current_loop = ?, current_waypoint_index = ?, ended_at = CURRENT_TIMESTAMP,
                        ended_at_home = ?, updated_at = CURRENT_TIMESTAMP
                    WHERE id = ?
                    """,
                    (current_loop, current_waypoint_index, ended_at_home, run_id),
                )
            elif status in {"failed", "aborted", "stopped"}:
                await db.execute(
                    """
                    UPDATE patrol_runs
                    SET status = ?, current_loop = ?, current_waypoint_index = ?, ended_at = CURRENT_TIMESTAMP,
                        failure_reason = ?, updated_at = CURRENT_TIMESTAMP
                    WHERE id = ?
                    """,
                    (status, current_loop, current_waypoint_index, message, run_id),
                )
            await db.commit()

        signature = json.dumps({
            "run_id": run_id,
            "status": status,
            "current_loop": current_loop,
            "current_waypoint_index": current_waypoint_index,
            "message": message,
        }, sort_keys=True)

        if signature != PatrolService._last_snapshot_signature:
            PatrolService._last_snapshot_signature = signature
            await LogService.log_event(
                "NAVIGATION",
                "patrol_runtime_update",
                f"Patrol run {run_id} status={status} loop={current_loop} waypoint={current_waypoint_index}",
                severity="WARNING" if status in {"failed", "aborted", "stopped"} else "INFO",
                metadata={
                    "run_id": run_id,
                    "status": status,
                    "current_loop": current_loop,
                    "current_waypoint_index": current_waypoint_index,
                    "message": message,
                    "previous_active_run": previous["id"] if previous else None,
                },
                session_id=session_id,
            )

    @staticmethod
    async def set_disconnected() -> None:
        async with PatrolService._lock:
            PatrolService._runtime_state = {
                **PatrolService._runtime_state,
                "connected": False,
                "updated_at": PatrolService._utcnow().isoformat(),
            }

    @staticmethod
    async def get_runtime_snapshot() -> dict[str, Any]:
        async with PatrolService._lock:
            return dict(PatrolService._runtime_state)

    @staticmethod
    async def get_status() -> dict[str, Any]:
        schedule = await PatrolService.get_schedule()
        route = await PatrolService.get_route()
        latest_run = await PatrolService.get_latest_run()
        runtime = await PatrolService.get_runtime_snapshot()
        return {
            "route": route,
            "schedule": schedule,
            "latest_run": latest_run,
            "runtime": runtime,
        }

    @staticmethod
    async def scheduler_loop(session_id: int | None) -> None:
        await PatrolService.ensure_default_schedule()
        try:
            while True:
                try:
                    schedule = await PatrolService.get_schedule()
                    if schedule.get("enabled"):
                        next_trigger = schedule.get("next_trigger_at")
                        active_run = await PatrolService.get_active_run()
                        if active_run is None and next_trigger:
                            trigger_at = datetime.fromisoformat(next_trigger)
                            if trigger_at.tzinfo is None:
                                trigger_at = trigger_at.replace(tzinfo=timezone.utc)
                            if PatrolService._utcnow() >= trigger_at:
                                try:
                                    await PatrolService.start_run(session_id=session_id, source="scheduler")
                                except Exception as exc:
                                    await LogService.log_event(
                                        "NAVIGATION",
                                        "patrol_schedule_skip",
                                        f"Skipped scheduled patrol: {exc}",
                                        severity="WARNING",
                                        metadata={"reason": str(exc)},
                                        session_id=session_id,
                                    )
                                    retry_at = PatrolService._utcnow() + timedelta(minutes=int(schedule["interval_minutes"]))
                                    async with aiosqlite.connect(settings.db_path) as db:
                                        await db.execute(
                                            "UPDATE patrol_schedules SET next_trigger_at = ?, updated_at = CURRENT_TIMESTAMP WHERE id = 1",
                                            (retry_at.isoformat(),),
                                        )
                                        await db.commit()
                    await asyncio.sleep(2.0)
                except asyncio.CancelledError:
                    raise
                except Exception as exc:
                    await LogService.log_event(
                        "SYSTEM",
                        "patrol_scheduler_error",
                        f"Patrol scheduler error: {exc}",
                        severity="ERROR",
                        metadata={"error": str(exc)},
                        session_id=session_id,
                    )
                    await asyncio.sleep(2.0)
        finally:
            await PatrolService.set_disconnected()
