"""Legacy adapter: ``ball_position_3d_mm`` (Float32MultiArray) -> ``BallState``.

Kept as a fallback for older tracker builds. Current ``ball_tracking_cpp`` publishes
native, timestamped ``BallState`` directly. The adapter stamps at reception because
the legacy Float32MultiArray stream has no event time; frame_id is a fixed camera
frame parameter.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from ur3e_catch_msgs.msg import BallState


class Float32Adapter(Node):
    def __init__(self) -> None:
        super().__init__("ball_float32_adapter")
        self.declare_parameter("input_topic", "ball_position_3d_mm")
        self.declare_parameter("output_topic", "ball_state")
        self.declare_parameter("camera_frame", "camera_optical")
        self.declare_parameter("units", "mm")  # tracker publishes millimetres

        self._frame = self.get_parameter("camera_frame").value
        self._scale = 1e-3 if self.get_parameter("units").value == "mm" else 1.0
        self._pub = self.create_publisher(BallState, self.get_parameter("output_topic").value, 10)
        self._sub = self.create_subscription(
            Float32MultiArray, self.get_parameter("input_topic").value, self._on_msg, 10
        )
        self.get_logger().info(
            f"Float32->BallState adapter: frame={self._frame!r}, scale={self._scale}"
        )

    def _on_msg(self, msg: Float32MultiArray) -> None:
        out = BallState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame
        if len(msg.data) >= 3:
            out.position.x = float(msg.data[0]) * self._scale
            out.position.y = float(msg.data[1]) * self._scale
            out.position.z = float(msg.data[2]) * self._scale
            out.valid = True
            out.confidence = 1.0
        else:
            out.valid = False
            out.confidence = 0.0
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Float32Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
