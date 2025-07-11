import rclpy
from rclpy.executors import MultiThreadedExecutor
from px4_offboard.offboard_control import OffboardControl  # Adjust if needed
import threading
import code

def main():
    rclpy.init()

    # Create drone nodes
    drone1 = OffboardControl(namespace='drone1',targetsystemid = 2)
    drone2 = OffboardControl(namespace='drone2',targetsystemid = 3)
    drone3 = OffboardControl(namespace='drone3',targetsystemid = 4)

    drones = {drone1, drone2, drone3}

    # Use a MultiThreadedExecutor to handle all drone nodes concurrently
    executor = MultiThreadedExecutor()
    for drone in drones:
        executor.add_node(drone)

    # Spin the executor in a background thread
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("\n🟢 OffboardControl started for all drones via offboard_control_agent.py")
    print("You can now interact with: drone1, drone2, drone3\n")

    # Interactive shell with access to drone objects
    code.interact(local={
        'drone1': drone1,
        'drone2': drone2,
        'drone3': drone3,
        'drones': drones
    })

    # Shutdown cleanly
    executor.shutdown()
    for drone in drones:
        drone.destroy_node()
    rclpy.shutdown()