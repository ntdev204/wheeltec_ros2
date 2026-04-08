"""
Coverage route generation and management service.
Integrates with ROS2 coverage planner to generate full-map coverage routes.
"""
import asyncio
import json
from datetime import datetime, timezone
from typing import Any

import aiosqlite

from app.config import settings
from app.services.log_service import LogService
from app.services.patrol_service import PatrolService
from app.zmq_client import zmq_client


class CoverageService:
    """Service for generating and managing coverage routes."""

    @staticmethod
    async def generate_coverage_route(
        name: str | None = None,
        robot_width: float = 0.5,
        overlap: float = 0.1,
        pattern: str = "boustrophedon"
    ) -> dict[str, Any]:
        """
        Generate full-map coverage route using ROS2 coverage planner.

        Args:
            name: Optional name for the coverage route
            robot_width: Width of robot footprint in meters
            overlap: Overlap ratio between passes (0.0-1.0)
            pattern: Coverage pattern type ("boustrophedon" or "spiral")

        Returns:
            Generated route dict with waypoints

        Raises:
            ValueError: If generation fails or validation fails
        """
        # Validate parameters
        if not 0.1 <= robot_width <= 2.0:
            raise ValueError("Robot width must be between 0.1 and 2.0 meters")
        if not 0.0 <= overlap <= 0.5:
            raise ValueError("Overlap must be between 0.0 and 0.5")
        if pattern not in {"boustrophedon", "spiral"}:
            raise ValueError("Pattern must be 'boustrophedon' or 'spiral'")

        # Check active map exists
        active_map = await PatrolService.get_active_map()
        if active_map is None:
            raise ValueError("No active map configured")

        # Call ROS2 coverage planner via ZMQ bridge
        payload = {
            "robot_width": robot_width,
            "overlap": overlap,
            "pattern": pattern
        }

        try:
            response = await zmq_client.send_command("generate_coverage", payload, timeout=30.0)
        except asyncio.TimeoutError as exc:
            raise ValueError("Coverage generation timed out after 30 seconds") from exc

        if response.get("status") != "ok":
            error_msg = response.get("message", "Unknown error from coverage planner")
            raise ValueError(f"Coverage generation failed: {error_msg}")

        waypoints = response.get("waypoints", [])
        if not waypoints:
            raise ValueError("Coverage planner returned no waypoints")

        # Generate route name if not provided
        if name is None:
            timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
            name = f"Coverage_{pattern}_{timestamp}"

        # Save route using existing patrol service
        route = await PatrolService.save_route(
            name=name,
            waypoints=waypoints,
            map_id=active_map["id"]
        )

        # Log coverage generation
        await LogService.log_event(
            "NAVIGATION",
            "coverage_route_generated",
            f"Generated coverage route '{name}' with {len(waypoints)} waypoints",
            metadata={
                "route_id": route["id"],
                "pattern": pattern,
                "robot_width": robot_width,
                "overlap": overlap,
                "waypoint_count": len(waypoints)
            }
        )

        return route

    @staticmethod
    async def get_coverage_routes(limit: int = 50) -> list[dict[str, Any]]:
        """
        Get all coverage routes (routes with 'Coverage_' prefix).

        Args:
            limit: Maximum number of routes to return

        Returns:
            List of coverage route dicts
        """
        safe_limit = max(1, min(limit, 500))

        async with aiosqlite.connect(settings.db_path) as db:
            db.row_factory = aiosqlite.Row
            cursor = await db.execute(
                """
                SELECT * FROM patrol_routes
                WHERE name LIKE 'Coverage_%'
                ORDER BY created_at DESC
                LIMIT ?
                """,
                (safe_limit,)
            )
            rows = await cursor.fetchall()

        routes = []
        for row in rows:
            route = dict(row)
            route["waypoints"] = json.loads(route["waypoints_json"] or "[]")
            routes.append(route)

        return routes

    @staticmethod
    async def estimate_coverage_time(waypoint_count: int, avg_speed: float = 0.3) -> float:
        """
        Estimate time to complete coverage route.

        Args:
            waypoint_count: Number of waypoints in route
            avg_speed: Average robot speed in m/s

        Returns:
            Estimated time in seconds
        """
        # Rough estimate: assume 2m between waypoints on average
        avg_distance_per_waypoint = 2.0
        total_distance = waypoint_count * avg_distance_per_waypoint

        # Add time for turns (assume 2 seconds per waypoint)
        turn_time = waypoint_count * 2.0

        travel_time = total_distance / avg_speed
        total_time = travel_time + turn_time

        return total_time

    @staticmethod
    async def get_coverage_statistics() -> dict[str, Any]:
        """
        Get statistics about coverage routes.

        Returns:
            Dict with coverage statistics
        """
        async with aiosqlite.connect(settings.db_path) as db:
            db.row_factory = aiosqlite.Row

            # Count total coverage routes
            cursor = await db.execute(
                "SELECT COUNT(*) as count FROM patrol_routes WHERE name LIKE 'Coverage_%'"
            )
            row = await cursor.fetchone()
            total_routes = row["count"] if row else 0

            # Get most recent coverage route
            cursor = await db.execute(
                """
                SELECT * FROM patrol_routes
                WHERE name LIKE 'Coverage_%'
                ORDER BY created_at DESC
                LIMIT 1
                """
            )
            row = await cursor.fetchone()
            latest_route = dict(row) if row else None

            # Count coverage runs (patrol runs using coverage routes)
            cursor = await db.execute(
                """
                SELECT COUNT(*) as count FROM patrol_runs pr
                JOIN patrol_routes rt ON pr.route_id = rt.id
                WHERE rt.name LIKE 'Coverage_%'
                """
            )
            row = await cursor.fetchone()
            total_runs = row["count"] if row else 0

        return {
            "total_coverage_routes": total_routes,
            "total_coverage_runs": total_runs,
            "latest_route": latest_route
        }
