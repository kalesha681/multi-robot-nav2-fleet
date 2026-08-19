import time
import rclpy
from rclpy.node import Node
from amr_msgs.msg import SafetyStatus

def main():
    rclpy.init()
    node = Node('safety_monitor_live')

    amr1_statuses = []
    amr2_statuses = []

    node.create_subscription(SafetyStatus, '/bcr_bot_amr1/safety_status', lambda m: amr1_statuses.append(m), 10)
    node.create_subscription(SafetyStatus, '/bcr_bot_amr2/safety_status', lambda m: amr2_statuses.append(m), 10)

    print("Capturing safety telemetry for 5 seconds...", flush=True)
    start = time.time()
    while time.time() - start < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"\nCaptured {len(amr1_statuses)} AMR-1 messages, {len(amr2_statuses)} AMR-2 messages.", flush=True)
    
    print("\n--- AMR-1 TELEMETRY (Recent Samples) ---", flush=True)
    for s in amr1_statuses[-5:]:
        print(f"[AMR1] speed={s.current_speed:.2f} m/s | limit={s.dynamic_speed_limit:.2f} m/s | obs_dist={s.closest_obstacle_distance:.2f} m | d_safe={s.min_stopping_distance:.2f} m | estop={s.emergency_stop_active} | state={s.safety_reason}", flush=True)

    print("\n--- AMR-2 TELEMETRY (Recent Samples) ---", flush=True)
    for s in amr2_statuses[-5:]:
        print(f"[AMR2] speed={s.current_speed:.2f} m/s | limit={s.dynamic_speed_limit:.2f} m/s | obs_dist={s.closest_obstacle_distance:.2f} m | d_safe={s.min_stopping_distance:.2f} m | estop={s.emergency_stop_active} | state={s.safety_reason}", flush=True)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
