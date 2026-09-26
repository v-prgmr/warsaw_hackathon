"""Interactive capture: press Enter to record a sample, q + Enter to quit.

    ros2 run g1_calibration calib_trigger
"""
import rclpy
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = rclpy.create_node("calib_trigger")
    client = node.create_client(Trigger, "/calib_capture/capture")
    if not client.wait_for_service(timeout_sec=10.0):
        raise SystemExit("/calib_capture/capture not available: is calib_capture running?")
    try:
        while True:
            key = input("Board and robot STILL? Enter = capture, q = quit: ").strip().lower()
            if key == "q":
                break
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)
            res = future.result()
            status = "OK   " if res and res.success else "WARN "
            print(status + (res.message if res else "no response"))
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
