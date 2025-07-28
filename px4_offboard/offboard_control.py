#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer inT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"
__Forkedby__ = "Luis Carpi"
__Contact__ = "luiscarpi1989@gmail.com"


import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"
__Forkedby__ = "Luis Carpi"
__Contact__ = "luiscarpi1989@gmail.com"


import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand  # ✅ Required import
from px4_msgs.msg import VehicleOdometry  # Ensure this import is present


from droneinterface.droneinterface import DroneDataInterface
class OffboardControl(Node):
    def __init__(self, namespace='drone1', targetsystemid=2):
        super().__init__('minimal_publisher')
        self.namespace = namespace
        self.targetsystemid = targetsystemid
        self.namespace_prefix = f'/{self.namespace}' if self.namespace else ''
        self.get_logger().info("🚀 Flight Modes Initialized")

        self.FLIGHT_MODES = {
            "HOLD": 4.0,
            "OFFBOARD": 6.0,
            "MANUAL": 1.0,
            "ALTCTL": 2.0,
            "POSCTL": 3.0,
            "AUTO_MISSION": 10.0,
            "AUTO_LOITER": 11.0,
            "AUTO_RTL": 12.0
        }

        self.latest_odometry = None
        self.latest_status = None
        self.timertoggle = 0
        self.dt = 0.02

        # QoS Profiles
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Data Interface
        self.data_interface = DroneDataInterface(f'{self.namespace_prefix}/fmu/out', qos_profile_sub)

        # # Subscribers
        # self.status_sub = self.create_subscription(
        #     VehicleStatus,
        #     f'{self.namespace_prefix}/fmu/out/vehicle_status_v1',
        #     self.vehicle_status_callback,
        #     qos_profile_sub
        # )

        # Publishers
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            f'{self.namespace_prefix}/fmu/in/offboard_control_mode',
            qos_profile_pub
        )

        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint,
            f'{self.namespace_prefix}/fmu/in/trajectory_setpoint',
            qos_profile_pub
        )

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            f'{self.namespace_prefix}/fmu/in/vehicle_command',
            qos_profile_pub
        )

        # Timer
        self.timer = self.create_timer(self.dt, self.cmdloop_callback)

        # Initial States
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        print(f"[INIT] Class Object Created: {self.namespace_prefix}")

    def odometry_callback(self, msg):
        self.latest_odometry = msg
        self.get_logger().debug("📡 Odometry Received")

    def get_position(self):
        if not self.latest_odometry:
            self.get_logger().warn("❗ No odometry data received yet.")
            return None

        pos = {
            'x': self.latest_odometry.position[0],
            'y': self.latest_odometry.position[1],
            'z': self.latest_odometry.position[2]
        }
        self.get_logger().info(f"📍 Position (NED): x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f}")
        return pos

    def vehicle_status(self):
        self.latest_status = self.data_interface.get_vehicle_status()
        self.nav_state = self.latest_status.nav_state
        self.arming_state = self.latest_status.arming_state

    def get_status(self):
        self.vehicle_status()
        if self.latest_status:
            self.get_logger().info(f"📈 Nav State: {self.nav_state}, Arming State: {self.arming_state}")
        else:
            self.get_logger().warn("⚠️ Vehicle status not yet received.")
        return self.latest_status

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0,
                                param3=0.0, param4=float('nan'), param5=float('nan'),
                                param6=float('nan'), param7=float('nan'), frame=0):
        msg = VehicleCommand()
        msg.param1, msg.param2, msg.param3 = param1, param2, param3
        msg.param4, msg.param5, msg.param6, msg.param7 = param4, param5, param6, param7
        msg.command = command
        msg.target_system = self.targetsystemid
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info(f"📤 Sent VehicleCommand {command} with param7={param7}, frame={frame}")

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("✅ Arm command sent")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("🛑 Disarm command sent")

    def set_position_mode(self):
        PX4_MAIN_MODE_POSCTL = 1
        self.publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=float(PX4_MAIN_MODE_POSCTL)
        )
        self.get_logger().info("🔁 Position Control mode requested")

    def switch_flight_mode(self, mode_name: str):
        mode_value = self.FLIGHT_MODES.get(mode_name.upper())
        if mode_value is None:
            self.get_logger().error(f"❌ Invalid flight mode: '{mode_name}'")
            return

        self.timertoggle = 1 if mode_name.upper() == "OFFBOARD" else 0
        self.publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=mode_value
        )
        self.get_logger().info(f"✈️ Switching to {mode_name.upper()} mode...")

    def takeoff(self, altitude=10.0):
        self.publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param7=altitude
        )
        self.get_logger().info(f"⬆️ Takeoff to {altitude} meters initiated")

    def land(self):
        self.publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_NAV_LAND
        )
        self.get_logger().info(f"⬇️ Landing command sent for drone: {self.namespace}")

    def gototargetposition(self, target_x, target_y, target_z):
        status = self.data_interface.get_vehicle_status()
        if (
            status and
            status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
            status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        ):
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            offboard_msg.position = True

            self.publisher_offboard_mode.publish(offboard_msg)

            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            trajectory_msg.position[0] = target_x
            trajectory_msg.position[1] = target_y
            trajectory_msg.position[2] = -target_z  # Z-down

            self.publisher_trajectory.publish(trajectory_msg)
            self.get_logger().info("🎯 Trajectory setpoint published")
        else:
            self.get_logger().warn("❌ Drone not armed or not in OFFBOARD mode")
            self.switch_flight_mode("OFFBOARD")

    def cmdloop_callback(self):
        if self.timertoggle:
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            offboard_msg.position = True
            self.publisher_offboard_mode.publish(offboard_msg)


def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
