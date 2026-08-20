# Copyright 2026 Abhinash
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('amr_localization')
    params_file = os.path.join(pkg_dir, 'config', 'ekf_params.yaml')

    return LaunchDescription([
        # EKF Node for AMR-1
        Node(
            package='amr_localization',
            executable='ekf_fusion_node',
            name='ekf_fusion_node',
            namespace='bcr_bot_amr1',
            output='screen',
            parameters=[
                params_file,
                {
                    'robot_name': 'bcr_bot_amr1',
                    'use_sim_time': True,
                }
            ],
            remappings=[
                ('/tf', '/tf'),
                ('tf', '/tf'),
                ('/tf_static', '/tf_static'),
                ('tf_static', '/tf_static'),
            ],
        ),
        # EKF Node for AMR-2
        Node(
            package='amr_localization',
            executable='ekf_fusion_node',
            name='ekf_fusion_node',
            namespace='bcr_bot_amr2',
            output='screen',
            parameters=[
                params_file,
                {
                    'robot_name': 'bcr_bot_amr2',
                    'use_sim_time': True,
                }
            ],
            remappings=[
                ('/tf', '/tf'),
                ('tf', '/tf'),
                ('/tf_static', '/tf_static'),
                ('tf_static', '/tf_static'),
            ],
        ),
    ])
