import rclpy
from px4_offboard.offboard_control import OffboardControl  # Adjust import if needed
import code

def main():
    rclpy.init()
    drone1 = OffboardControl(namespace='drone1')

    print("\n🟢 OffboardControl started via offboard_control_agent.py")
    print("Type 'drone1.arm()', 'drone1.disarm()', etc. to interact.\n")

    # Drop into an interactive shell with access to the 'event' object
    code.interact(local=locals())

    drone1.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

