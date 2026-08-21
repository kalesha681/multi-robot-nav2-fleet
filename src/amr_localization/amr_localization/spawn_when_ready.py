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

"""Robust entity spawner with immediate retry loop for synchronous Gazebo bringup."""

import sys
import time
import subprocess


def main():
    create_cmd = ['ros2', 'run', 'ros_gz_sim', 'create'] + sys.argv[1:]

    max_attempts = 30
    for attempt in range(1, max_attempts + 1):
        result = subprocess.run(create_cmd)
        if result.returncode == 0:
            sys.exit(0)
        time.sleep(0.5)

    sys.exit(result.returncode)


if __name__ == '__main__':
    main()
