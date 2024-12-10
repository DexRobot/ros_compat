# First, determine ROS version
try:
    import rclpy
    ROS_VERSION = 2
except ImportError:
    try:
        import rospy
        ROS_VERSION = 1
    except ImportError:
        raise ImportError("Neither ROS1 (rospy) nor ROS2 (rclpy) was found")

from .node import ROSNode
from .time import ROSTime
from .logging import ROSLogger

__version__ = "0.1.0"
__all__ = ['ROSNode', 'ROSTime', 'ROSLogger']
