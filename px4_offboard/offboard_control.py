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




class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        # Declare and retrieve the namespace parameter
        self.declare_parameter('namespace', '')  # Default to empty namespace
        self.namespace = self.get_parameter('namespace').value
        self.targetsystemid = 2
        self.namespace_prefix = f'/{self.namespace}' if self.namespace else ''
        
                # QoS profiles
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
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            f'{self.namespace_prefix}/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile_sub)

        self.get_logger().info(f"Class Created: '{self.namespace_prefix}/fmu/in/'")

        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, f'{self.namespace_prefix}/fmu/in/offboard_control_mode', qos_profile_pub)
        
        self.subscription_odometry = self.create_subscription(
            VehicleOdometry,
            f'{self.namespace_prefix}/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile_sub  # Replace with your actual QoS profile
        )


        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, f'{self.namespace_prefix}/fmu/in/offboard_control_mode', qos_profile_pub)

        
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, f'{self.namespace_prefix}/fmu/in/trajectory_setpoint', qos_profile_pub)
        timer_period = 0.02  # seconds.
        
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'{self.namespace_prefix}/fmu/in/vehicle_command', qos_profile_pub)  # ✅ Needed

        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period

        self.declare_parameter('radius', 10.0)
        self.declare_parameter('omega', 5.0)
        self.declare_parameter('altitude', 5.0)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        # Note: no parameter callbacks are used to prevent sudden inflight changes of radii and omega 
        # which would result in large discontinuities in setpoints
        self.theta = 0.0
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.altitude = self.get_parameter('altitude').value
 

    def __init__(self, namespace='drone1',targetsystemid = 2):
        super().__init__('minimal_publisher')
        self.namespace = namespace
        self.targetsystemid = targetsystemid
        self.namespace_prefix = f'/{self.namespace}' if self.namespace else ''
        # Declare and retrieve the namespace parameter

                # QoS profiles
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
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            f'{self.namespace_prefix}/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile_sub)

        print(f"ClassobjectCreated: {self.namespace_prefix}")
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, f'{self.namespace_prefix}/fmu/in/offboard_control_mode', qos_profile_pub)


        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, f'{self.namespace_prefix}/fmu/in/offboard_control_mode', qos_profile_pub)

        
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, f'{self.namespace_prefix}/fmu/in/trajectory_setpoint', qos_profile_pub)
        timer_period = 0.02  # seconds.
        
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'{self.namespace_prefix}/fmu/in/vehicle_command', qos_profile_pub)  # ✅ Needed

        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period

        self.declare_parameter('radius', 10.0)
        self.declare_parameter('omega', 5.0)
        self.declare_parameter('altitude', 5.0)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        # Note: no parameter callbacks are used to prevent sudden inflight changes of radii and omega 
        # which would result in large discontinuities in setpoints
        self.theta = 0.0
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.altitude = self.get_parameter('altitude').value
    def odometry_callback(self, msg):
        self.latest_odometry = msg
    def get_position(self):
        if not self.latest_odometry:
            self.get_logger().warn("No odometry data received yet.")
            return None

        position = {
            'x': self.latest_odometry.position[0],
            'y': self.latest_odometry.position[1],
            'z': self.latest_odometry.position[2],
        }

        self.get_logger().info(f"Current position (NED): x={position['x']:.2f}, y={position['y']:.2f}, z={position['z']:.2f}")
        return position


    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.latest_status = msg  # Store the whole message
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
    def odometry_callback(self, msg):
        self.latest_odometry = msg
    def get_status(self):

        if self.latest_status:
            self.get_logger().info(f"Current nav state: {self.nav_state}, arming state: {self.arming_state}")
        else:
            self.get_logger().warn("Vehicle status not yet received.")

        return getattr(self, 'latest_status', None)

    def publish_vehicle_command(self,
                                command,
                                param1=0.0,
                                param2=0.0,
                                param3=0.0,
                                param4=float('nan'),
                                param5=float('nan'),
                                param6=float('nan'),
                                param7=float('nan'),
                                frame=0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command
        #msg.frame = frame  # 🔹 Set frame if applicable

        msg.target_system = self.targetsystemid
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # microseconds

        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info(
            f"📤 Sent VehicleCommand {command} with altitude={param7}, frame={frame}"
        )



    def set_position_mode(self):
        # PX4 main mode for Position Control is 1
        PX4_MAIN_MODE_POSCTL = 1

        self.publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,  # Custom mode (ignored for PX4, but required)
            param2=float(PX4_MAIN_MODE_POSCTL)
        )

        self.get_logger().info(f"Position Control mode command sent to '{self.namespace_prefix}/fmu/in/vehicle_command'")

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        
        self.get_logger().info(f"Arm command sent to '{self.namespace_prefix}/fmu/in/vehicle_command'")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command sent")

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher_offboard_mode.publish(msg)

    def takeoff(self, altitude=10.0):
        self.publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param7= altitude

        )
        #self.get_logger().info(f"Takeoff command sent to altitude {altitude}m")


    def land(self):
        # Altitude is given in param7 (takeoff altitude above home)
        self.publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_NAV_LAND,
            
        )
        self.get_logger().info(f"Landing Drone {self.namespace}")

    def gototargetposition(self,target_x,target_y,target_z):
        # Step 1: Publish the offboard control mode (position only)
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.publisher_offboard_mode.publish(offboard_msg)

        # Step 2: Only send trajectory if armed and in OFFBOARD mode
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED):

            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

            # Set target position (ENU frame)
            trajectory_msg.position[0] = target_x  # X
            trajectory_msg.position[1] = target_y  # Y
            trajectory_msg.position[2] = -target_z  # Z-down in PX4

            self.publisher_trajectory.publish(trajectory_msg)
# void OffboardControl::arm()
# {
# 	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

# 	RCLCPP_INFO(this->get_logger(), "Arm command send");
# }

# void OffboardControl::disarm()  
# {
# 	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

# 	RCLCPP_INFO(this->get_logger(), "Disarm command send");
# }

# void OffboardControl::publish_offboard_control_mode()
# {
# 	OffboardControlMode msg{};
# 	msg.position = true;
# 	msg.velocity = false;
# 	msg.acceleration = false;
# 	msg.attitude = false;
# 	msg.body_rate = false;
# 	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
# 	offboard_control_mode_publisher_->publish(msg);
# }



    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):

            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = self.radius * np.cos(self.theta)
            trajectory_msg.position[1] = self.radius * np.sin(self.theta)
            trajectory_msg.position[2] = -self.altitude
            self.publisher_trajectory.publish(trajectory_msg)

            self.theta = self.theta + self.omega * self.dt


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
