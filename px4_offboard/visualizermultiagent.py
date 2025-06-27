#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from typing import List, Tuple

from px4_offboard.visualizer import PX4Visualizer  # Ensure this path is correct


class PX4VisualizerWithNamespace(PX4Visualizer):
    def __init__(self, namespace: str, prefix: str):
        super().__init__(namespace=namespace, prefix=prefix)
        self.get_logger().info(
            f"[PX4VisualizerWithNamespace] Initialized with namespace='{self.namespace}', prefix='{self.prefix}'"
        )


class PX4VisualizerArray(Node):
    def __init__(self, config: List[Tuple[str, str]]):
        super().__init__('visualizer_array')
        self.visualizers = []

        for namespace, prefix in config:
            visualizer = PX4VisualizerWithNamespace(namespace, prefix)
            self.visualizers.append(visualizer)
            self.get_logger().info(
                f"[PX4VisualizerArray] Added visualizer: namespace='{namespace}', prefix='{prefix}'"
            )


def main(args=None):
    rclpy.init(args=args)

    config = [
        ('drone1', 'drone1'),
        ('drone2', 'drone2'),
        ('drone3', 'drone3'),
    ]

    visualizer_array = PX4VisualizerArray(config)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(visualizer_array)

    for visualizer in visualizer_array.visualizers:
        executor.add_node(visualizer)

    try:
        rate_hz = 2  # <-- Spin 10 times per second (100ms delay)
        sleep_duration = 1.0 / rate_hz
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)  # Optional timeout to keep thread responsive
            time.sleep(sleep_duration)
    except KeyboardInterrupt:
        pass
    finally:
        for visualizer in visualizer_array.visualizers:
            visualizer.destroy_node()
        visualizer_array.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()