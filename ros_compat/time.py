from . import ROS_VERSION

try:
    import rclpy as ros
    from rclpy.time import Time as ROS2Time
except ImportError:
    try:
        import rospy as ros
        from rospy import Time as ROS1Time
    except ImportError:
        ros = None


class ROSTime:
    """Wrapper for ROS time operations.

    This class provides a unified interface for time operations that work
    consistently across ROS1 and ROS2.
    """

    @staticmethod
    def now() -> float:
        """Get the current ROS time.

        Returns:
            float: Current time in seconds.

        Examples:
            >>> current_time = ROSTime.now()
            >>> print(f"Current time: {current_time}")
        """
        if ROS_VERSION is None:
            raise ImportError("Neither ROS1 (rospy) nor ROS2 (rclpy) was found")
        if ROS_VERSION == 2:
            return float(ros.time.Time().nanoseconds / 1e9)
        return float(ros.Time.now().to_sec())

    @staticmethod
    def sleep(duration: float) -> None:
        """Sleep for specified duration in seconds."""
        if ROS_VERSION is None:
            raise ImportError("Neither ROS1 (rospy) nor ROS2 (rclpy) was found")
        if ROS_VERSION == 2:
            ros.time.sleep(duration)
        else:
            ros.sleep(duration)
