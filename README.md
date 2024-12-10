# ROS Compatibility Layer

A Python package that provides a unified interface for writing nodes that work with both ROS1 (rospy) and ROS2 (rclpy).

## Features

- Unified node initialization and management
- Common interface for publishers and subscribers
- Timer support
- Service client and server support
- Common message type handling
- Unified logging interface
- Time operations wrapper

## Installation

```bash
# For ROS1 support
pip install ros-compat[ros1]

# For ROS2 support
pip install ros-compat[ros2]
```

## Quick Start

```python
from ros_compat import ROSNode
from std_msgs.msg import String

class MyNode(ROSNode):
    def __init__(self):
        super().__init__('my_node')
        self.pub = self.create_publisher('topic', String)
        self.create_subscription('input', String, self.callback)
        self.create_timer(1.0, self.timer_callback)

    def callback(self, msg):
        self.logger.info(f"Received: {msg.data}")

    def timer_callback(self):
        msg = String()
        msg.data = "Hello"
        self.pub.publish(msg)

if __name__ == '__main__':
    node = MyNode()
    node.spin()
```

## API Reference

### ROSNode

The base class for creating ROS nodes that work with both ROS1 and ROS2.

#### Methods

- `create_publisher(topic: str, msg_type: Type, queue_size: int = 10) -> Publisher`
- `create_subscription(topic: str, msg_type: Type, callback: Callable, queue_size: int = 10) -> Subscriber`
- `create_timer(period: float, callback: Callable) -> Timer`
- `create_service(name: str, srv_type: Type, callback: Callable) -> Service`
- `create_client(name: str, srv_type: Type, timeout: Optional[float] = None) -> Client`
- `spin() -> None`
- `spin_once() -> None`
- `shutdown() -> None`

### ROSTime

Wrapper for ROS time operations.

#### Methods

- `now() -> float`: Get current ROS time in seconds
- `sleep(duration: float) -> None`: Sleep for specified duration

### ROSLogger

Wrapper for ROS logging functions.

#### Methods

- `info(msg: str) -> None`
- `warn(msg: str) -> None`
- `error(msg: str) -> None`
- `debug(msg: str) -> None`

## Examples

See the `examples` directory for complete example nodes.

## License

MIT License
