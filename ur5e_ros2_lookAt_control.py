#!/usr/bin/env python3
"""Move a UR5e through predefined look-at poses via ROS 2 URScript commands.

This is the Python equivalent of ``ur5e_ros2_lookAt_control.m``. It computes
the TCP orientation so the tool Z axis points at a fixed target while keeping a
zero-roll convention, then publishes a URScript ``movej(p[...])`` command to
``/urscript_interface/script_command``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    z: float


def _normalize(vector: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vector)
    if norm < 1e-12:
        raise ValueError("Cannot normalize a zero-length vector")
    return vector / norm


def look_at_no_roll(current_pos: Point, target: Point) -> tuple[np.ndarray, np.ndarray]:
    """Return rotation matrix and rotation vector matching lookAtNoRoll.m."""
    direction = np.array(
        [
            target.x - current_pos.x,
            target.y - current_pos.y,
            target.z - current_pos.z,
        ],
        dtype=float,
    )
    z_axis = _normalize(direction)

    world_up = np.array([0.0, 0.0, 1.0])
    up_proj = world_up - np.dot(world_up, z_axis) * z_axis

    if np.linalg.norm(up_proj) < 1e-3:
        world_up = np.array([0.0, 1.0, 0.0])
        up_proj = world_up - np.dot(world_up, z_axis) * z_axis

    x_axis = _normalize(up_proj)
    y_axis = np.cross(z_axis, x_axis)

    rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))
    rotation_vector = rotation_matrix_to_rotation_vector(rotation_matrix)
    return rotation_matrix, rotation_vector


def rotation_matrix_to_rotation_vector(rotation_matrix: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to an axis-angle rotation vector."""
    trace = float(np.trace(rotation_matrix))
    cos_angle = (trace - 1.0) / 2.0
    cos_angle = max(-1.0, min(1.0, cos_angle))
    angle = math.acos(cos_angle)

    if abs(angle) < 1e-12:
        return np.zeros(3)

    if abs(math.pi - angle) < 1e-6:
        axis = np.empty(3)
        axis[0] = math.sqrt(max(0.0, (rotation_matrix[0, 0] + 1.0) / 2.0))
        axis[1] = math.sqrt(max(0.0, (rotation_matrix[1, 1] + 1.0) / 2.0))
        axis[2] = math.sqrt(max(0.0, (rotation_matrix[2, 2] + 1.0) / 2.0))

        if rotation_matrix[0, 1] < 0.0:
            axis[1] = -axis[1]
        if rotation_matrix[0, 2] < 0.0:
            axis[2] = -axis[2]

        return _normalize(axis) * angle

    axis = np.array(
        [
            rotation_matrix[2, 1] - rotation_matrix[1, 2],
            rotation_matrix[0, 2] - rotation_matrix[2, 0],
            rotation_matrix[1, 0] - rotation_matrix[0, 1],
        ],
        dtype=float,
    )
    axis = axis / (2.0 * math.sin(angle))
    return axis * angle


class UR5eLookAtController(Node):
    def __init__(self) -> None:
        super().__init__("ur5e_look_at_controller")
        self.publisher = self.create_publisher(
            String,
            "/urscript_interface/script_command",
            10,
        )

    def move_to_pose(
        self,
        position: Point,
        rotation_vector: np.ndarray,
        acceleration: float = 1.2,
        velocity: float = 0.25,
        blend_radius: float = 0.0,
    ) -> None:
        script = (
            "def my_prog():\n"
            "set_digital_out(1, True)\n"
            f"movej(p[{position.x:.6f}, {position.y:.6f}, {position.z:.6f}, "
            f"{rotation_vector[0]:.6f}, {rotation_vector[1]:.6f}, "
            f"{rotation_vector[2]:.6f}], "
            f"a={acceleration:.6f}, v={velocity:.6f}, r={blend_radius:.6f})\n"
            'textmsg("motion finished")\n'
            "end"
        )

        message = String()
        message.data = script
        self.publisher.publish(message)
        self.get_logger().info(
            "Published movej pose: "
            f"p[{position.x:.3f}, {position.y:.3f}, {position.z:.3f}, "
            f"{rotation_vector[0]:.3f}, {rotation_vector[1]:.3f}, "
            f"{rotation_vector[2]:.3f}]"
        )


def iter_positions() -> Iterable[Point]:
    positions_x = [0.2, -0.2, 0.3]
    positions_y = [-0.3, -0.4, 0.4]
    positions_z = [0.6, 0.6, 0.6]

    for x, y, z in zip(positions_x, positions_y, positions_z):
        yield Point(x=x, y=y, z=z)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = UR5eLookAtController()

    target = Point(x=0.0, y=-0.3, z=0.0)

    try:
        for index, position in enumerate(iter_positions(), start=1):
            _, rotation_vector = look_at_no_roll(position, target)
            node.get_logger().info(
                f"Moving to point {index}: "
                f"position=({position.x}, {position.y}, {position.z}), "
                f"target=({target.x}, {target.y}, {target.z})"
            )
            node.move_to_pose(position, rotation_vector)

            rclpy.spin_once(node, timeout_sec=0.1)

            if index < 3:
                input("Press Enter to move to the next position...")
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
