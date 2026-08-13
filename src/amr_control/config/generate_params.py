import yaml
import sys

def update_params(filename, ns, max_lin_vel, max_lin_acc, max_ang_vel, max_ang_acc):
    with open(filename, 'r') as f:
        data = yaml.safe_load(f)

    # frames
    data['amcl']['ros__parameters']['base_frame_id'] = f"{ns}/base_footprint"
    data['amcl']['ros__parameters']['global_frame_id'] = f"{ns}/map"
    data['amcl']['ros__parameters']['odom_frame_id'] = f"{ns}/odom"
    data['amcl']['ros__parameters']['scan_topic'] = f"/{ns}/scan"
    
    data['bt_navigator']['ros__parameters']['global_frame'] = f"{ns}/map"
    data['bt_navigator']['ros__parameters']['robot_base_frame'] = f"{ns}/base_link"
    data['bt_navigator']['ros__parameters']['odom_topic'] = f"/{ns}/odom"

    data['local_costmap']['local_costmap']['ros__parameters']['global_frame'] = f"{ns}/odom"
    data['local_costmap']['local_costmap']['ros__parameters']['robot_base_frame'] = f"{ns}/base_link"
    data['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']['scan']['topic'] = f"/{ns}/scan"
    data['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']['inflation_radius'] = 0.35
    data['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']['cost_scaling_factor'] = 4.0

    data['global_costmap']['global_costmap']['ros__parameters']['global_frame'] = f"{ns}/map"
    data['global_costmap']['global_costmap']['ros__parameters']['robot_base_frame'] = f"{ns}/base_link"
    data['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['scan']['topic'] = f"/{ns}/scan"
    data['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']['inflation_radius'] = 0.35
    data['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']['cost_scaling_factor'] = 4.0

    data['behavior_server']['ros__parameters']['global_frame'] = f"{ns}/odom"
    data['behavior_server']['ros__parameters']['robot_base_frame'] = f"{ns}/base_link"
    data['behavior_server']['ros__parameters']['max_rotational_vel'] = max_ang_vel
    data['behavior_server']['ros__parameters']['min_rotational_vel'] = 0.2
    data['behavior_server']['ros__parameters']['rotational_acc_lim'] = max_ang_acc

    # DWB FollowPath
    dwb = data['controller_server']['ros__parameters']['FollowPath']
    dwb['max_vel_x'] = max_lin_vel
    dwb['min_vel_x'] = -max_lin_vel
    dwb['max_speed_xy'] = max_lin_vel
    dwb['min_speed_xy'] = 0.0
    dwb['max_vel_theta'] = max_ang_vel
    dwb['acc_lim_x'] = max_lin_acc
    dwb['acc_lim_theta'] = max_ang_acc
    dwb['decel_lim_x'] = -max_lin_acc
    dwb['decel_lim_theta'] = -max_ang_acc
    
    # Velocity smoother
    vs = data['velocity_smoother']['ros__parameters']
    vs['max_velocity'] = [max_lin_vel, 0.0, max_ang_vel]
    vs['min_velocity'] = [-max_lin_vel, 0.0, -max_ang_vel]
    vs['max_accel'] = [max_lin_acc, 0.0, max_ang_acc]
    vs['max_decel'] = [-max_lin_acc, 0.0, -max_ang_acc]
    vs['odom_topic'] = f"/{ns}/odom"

    with open(filename, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)

if __name__ == '__main__':
    # AMR1
    update_params('/home/cp-lab/Amr_ws/src/amr_control/config/nav2_params_amr1.yaml',
                  'bcr_bot_amr1', 1.0, 1.0, 1.0, 1.0)
    # AMR2
    update_params('/home/cp-lab/Amr_ws/src/amr_control/config/nav2_params_amr2.yaml',
                  'bcr_bot_amr2', 1.5, 2.0, 1.5, 2.5)
