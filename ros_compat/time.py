from . import ROS_VERSION

try:
    import rclpy as ros
    from rclpy.time import Time as ROS2Time
except ImportError:
    import rospy as ros
    from rospy import Time as ROS1Time

class ROSTime:
    """Wrapper for ROS time operations."""
    @staticmethod
    def now() -> float:
        """Get current ROS time in seconds."""
        if ROS_VERSION == 2:
            return float(ros.time.Time().nanoseconds / 1e9)
        return float(ros.Time.now().to_sec())

    @staticmethod
    def sleep(duration: float) -> None:
        """Sleep for specified duration in seconds."""
        if ROS_VERSION == 2:
            ros.time.sleep(duration)
        else:
            ros.sleep(duration)
