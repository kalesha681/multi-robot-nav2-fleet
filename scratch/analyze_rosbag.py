import sqlite3
import os
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def analyze_bag():
    db_path = '/home/abhinash/AMR_ws/fleet_diag_bag_0.db3'
    if not os.path.exists(db_path):
        print(f"Error: {db_path} does not exist.")
        return

    conn = sqlite3.connect(db_path)
    c = conn.cursor()
    c.execute('SELECT id, name, type FROM topics')
    topics = {row[0]: (row[1], get_message(row[2])) for row in c.fetchall()}

    c.execute('SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp ASC')
    rows = c.fetchall()

    rosout_events = []
    amr1_odom = []
    amr2_odom = []
    slope_msgs = []
    stats_msgs = []
    cmd_vel_amr1 = []

    for topic_id, t_stamp, data in rows:
        name, msg_type = topics[topic_id]
        msg = deserialize_message(data, msg_type)
        if name == '/rosout':
            rosout_events.append((t_stamp, msg.name, msg.level, msg.msg))
        elif name == '/bcr_bot_amr1/odom':
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            v = msg.twist.twist.linear.x
            w = msg.twist.twist.angular.z
            amr1_odom.append((t_stamp, p.x, p.y, p.z, v, w))
        elif name == '/bcr_bot_amr2/odom':
            p = msg.pose.pose.position
            v = msg.twist.twist.linear.x
            amr2_odom.append((t_stamp, p.x, p.y, p.z, v))
        elif name == '/bcr_bot_amr1/cmd_vel':
            cmd_vel_amr1.append((t_stamp, msg.linear.x, msg.angular.z))
        elif name == '/fleet/slope_cost_zone':
            slope_msgs.append((t_stamp, msg.zone_name, msg.dynamic_cost_penalty, msg.incline_angle_deg))
        elif name == '/fleet/amr1_selective_stats':
            stats_msgs.append((t_stamp, msg.data))

    report = []
    report.append("================================================================================")
    report.append("                   FLEET DIAGNOSTIC ROSBAG ANALYSIS REPORT                     ")
    report.append("================================================================================")
    report.append(f"Total recorded messages: {len(rows)}")
    report.append(f"AMR-1 Odometry samples:  {len(amr1_odom)}")
    report.append(f"AMR-2 Odometry samples:  {len(amr2_odom)}")
    report.append(f"AMR-1 CmdVel samples:    {len(cmd_vel_amr1)}")
    report.append(f"Slope Cost messages:     {len(slope_msgs)}")
    report.append(f"Selective Stats msgs:    {len(stats_msgs)}")
    report.append(f"Rosout Log entries:      {len(rosout_events)}")
    report.append("")

    if amr1_odom:
        t0 = amr1_odom[0][0] * 1e-9
        t_end = amr1_odom[-1][0] * 1e-9
        xs = [pt[1] for pt in amr1_odom]
        ys = [pt[2] for pt in amr1_odom]
        zs = [pt[3] for pt in amr1_odom]
        vs = [pt[4] for pt in amr1_odom]
        ws = [pt[5] for pt in amr1_odom]

        report.append("--------------------------------------------------------------------------------")
        report.append("1. AMR-1 TRAJECTORY & ELEVATION KINEMATICS")
        report.append("--------------------------------------------------------------------------------")
        report.append(f"Recording Duration:      {t_end - t0:.2f} seconds")
        report.append(f"Initial Spawn Pose:      X = {xs[0]:.3f} m,  Y = {ys[0]:.3f} m,  Z = {zs[0]:.3f} m")
        report.append(f"Final Destination Pose:  X = {xs[-1]:.3f} m,  Y = {ys[-1]:.3f} m,  Z = {zs[-1]:.3f} m")
        report.append(f"X Displacement Range:    [{min(xs):.3f} m  ->  {max(xs):.3f} m] (Delta: {max(xs)-min(xs):.3f} m)")
        report.append(f"Y Displacement Range:    [{min(ys):.3f} m  ->  {max(ys):.3f} m] (Delta: {max(ys)-min(ys):.3f} m)")
        report.append(f"Z Elevation Range:       [{min(zs):.3f} m  ->  {max(zs):.3f} m] (Peak Ramp Deck: {max(zs):.3f} m)")
        report.append(f"Max Linear Speed (vx):   {max(vs):.3f} m/s")
        report.append(f"Max Angular Rate (wz):   {max(abs(w) for w in ws):.3f} rad/s")
        
        # Check ramp traversal criteria (Ramp Platform is at X = -3.4, Z = 0.53m)
        ramp_samples = [pt for pt in amr1_odom if abs(pt[1] - (-3.4)) < 1.0 and pt[3] > 0.15]
        if ramp_samples:
            max_ramp_z = max(pt[3] for pt in ramp_samples)
            report.append(f"Ramp Traversal Status:   [CONFIRMED TRAVERSED] ({len(ramp_samples)} points at X ~ -3.4m, Max Z = {max_ramp_z:.3f}m)")
        else:
            report.append(f"Ramp Traversal Status:   [NOT ON RAMP / FLAT ROUTE] (Max Z = {max(zs):.3f}m, Min X = {min(xs):.3f}m)")
        report.append("")

    if slope_msgs:
        report.append("--------------------------------------------------------------------------------")
        report.append("2. SLOPE TRAVERSABILITY COST MESSAGES (/fleet/slope_cost_zone)")
        report.append("--------------------------------------------------------------------------------")
        report.append(f"Total cost updates published: {len(slope_msgs)}")
        unique_zones = set(sm[1] for sm in slope_msgs)
        report.append(f"Zones monitored: {unique_zones}")
        for sm in slope_msgs[:6]:
            t_rel = (sm[0] - rows[0][1]) * 1e-9
            report.append(f"  [t={t_rel:6.2f}s] Zone: {sm[1]:20s} | Cost: {sm[2]:5.1f} | Slope Angle: {sm[3]:.1f}°")
        report.append("")

    if stats_msgs:
        report.append("--------------------------------------------------------------------------------")
        report.append("3. SELECTIVE MAPPING STATS (/fleet/amr1_selective_stats)")
        report.append("--------------------------------------------------------------------------------")
        for st in stats_msgs[-5:]:
            report.append(f"  {st[1]}")
        report.append("")

    report.append("--------------------------------------------------------------------------------")
    report.append("4. ROSOUT MISSION & PLANNER LOGS (Filtered by Key Events)")
    report.append("--------------------------------------------------------------------------------")
    for t_stamp, name, lvl, text in rosout_events:
        t_rel = (t_stamp - rows[0][1]) * 1e-9
        if any(k in text.lower() for k in ['mission', 'goal', 'succeeded', 'aborted', 'failed', 'slope', 'ramp', 'cost', 'navigate', 'starting point', 'mppi', 'planner', 'bt_navigator']):
            report.append(f"  [{t_rel:6.2f}s][{name}][lvl={lvl}]: {text}")

    full_report_text = "\n".join(report)
    out_file = '/home/abhinash/AMR_ws/scratch/rosbag_analysis_report.txt'
    with open(out_file, 'w') as f:
        f.write(full_report_text)
    print(f"Report written to {out_file}")
    print(full_report_text)

if __name__ == '__main__':
    analyze_bag()
