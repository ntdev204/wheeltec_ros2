from context_aware_bridge.obstacle_guard import ObstacleGuard

try:
    from context_aware_bridge.bridge_node import ContextAwareBridgeNode
except ImportError:  # pragma: no cover - allows pure unit tests without ROS2 installed
    ContextAwareBridgeNode = None

__all__ = ['ContextAwareBridgeNode', 'ObstacleGuard']
