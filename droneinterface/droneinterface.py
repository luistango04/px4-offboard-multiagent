import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.logging import LoggingSeverity


from typing import Optional
import threading

from px4_msgs.msg import (
    VehicleStatus, VehicleAttitude, VehicleLocalPosition, VehicleGlobalPosition,
    BatteryStatus, ManualControlSetpoint, FailsafeFlags,
    EstimatorStatusFlags, VtolVehicleStatus,SensorCombined
)

class DroneDataInterface(Node):
    def __init__(self, namespace: str = '/drone1/fmu/out', QoSprofile=None):
        super().__init__('drone_data_interface')
        self.get_logger().set_level(LoggingSeverity.DEBUG)  #

        self.namespace = namespace.rstrip('/')
        self._latest_msgs = {}
        self._lock = threading.Lock()

        # Default QoS if none provided
        if QoSprofile is None:
            from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
            QoSprofile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5
            )
            self.get_logger().debug("No QoS profile provided. Using default BEST_EFFORT.")

        topics = [
            (VehicleStatus, 'vehicle_status_v1'),
            (VehicleAttitude, 'vehicle_attitude'),
            (VehicleLocalPosition, 'vehicle_local_position'),
            (VehicleGlobalPosition, 'vehicle_global_position'),
            (BatteryStatus, 'battery_status_v1'),
            (ManualControlSetpoint, 'manual_control_setpoint'),
            (FailsafeFlags, 'failsafe_flags'),
            (EstimatorStatusFlags, 'estimator_status_flags'),
            (VtolVehicleStatus, 'vtol_vehicle_status'),
            (SensorCombined,'sensor_combined')

        ]

        for msg_type, topic_suffix in topics:
            self._subscribe(msg_type, topic_suffix, QoSprofile)

    def _subscribe(self, msg_type, topic_suffix, qos_profile):
        topic = f"{self.namespace}/{topic_suffix}"
        self.create_subscription(
            msg_type,
            topic,
            lambda msg, t=topic_suffix: self._msg_callback(t, msg),
            qos_profile
        )
        self.get_logger().debug(f"Subscribed to topic: {topic} with QoS: {qos_profile}")

    def _msg_callback(self, topic_suffix, msg):
        with self._lock:
            self._latest_msgs[topic_suffix] = msg
        #self.get_logger().debug(f"Received message on '{topic_suffix}': {msg}")

    def get_latest(self, topic_suffix) -> Optional[object]:
        with self._lock:
            msg = self._latest_msgs.get(topic_suffix)
        if msg:
            self.get_logger().debug(f"Retrieved latest message from '{topic_suffix}': {msg}")
        else:
            self.get_logger().debug(f"No message available yet for '{topic_suffix}'")
        return msg

    # Service-style methods
    def get_vehicle_status(self) -> Optional[VehicleStatus]:

        return self.get_latest('vehicle_status_v1')

    def get_attitude(self) -> Optional[VehicleAttitude]:
        return self.get_latest('vehicle_attitude')

    def get_local_position(self) -> Optional[VehicleLocalPosition]:
        return self.get_latest('vehicle_local_position')

    def get_global_position(self) -> Optional[VehicleGlobalPosition]:
        return self.get_latest('vehicle_global_position')

    def get_battery_status(self) -> Optional[BatteryStatus]:
        return self.get_latest('battery_status_v1')

    def get_manual_control(self) -> Optional[ManualControlSetpoint]:
        return self.get_latest('manual_control_setpoint')

    def get_failsafe_flags(self) -> Optional[FailsafeFlags]:
        return self.get_latest('failsafe_flags')

    def get_estimator_flags(self) -> Optional[EstimatorStatusFlags]:
        return self.get_latest('estimator_status_flags')

    def get_vtol_status(self) -> Optional[VtolVehicleStatus]:
        return self.get_latest('vtol_vehicle_status')
    
    def get_sensorcombined(self) -> Optional[SensorCombined]:
        self.get_logger().debug("Debug message")

        return self.get_latest('sensor_combined')  # match case exactly



