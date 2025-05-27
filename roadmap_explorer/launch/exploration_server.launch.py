# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    exploration_params_dir = os.path.join(
        get_package_share_directory('roadmap_explorer'), 'params')

    params_file = os.path.join(
        exploration_params_dir, 'exploration_params.yaml')

    roadmap_explorer_node = Node(
        package='roadmap_explorer',
        executable='roadmap_exploration_server',
        name='roadmap_explorer_node',
        # prefix=['gdbserver localhost:3000'],
        output='screen',
        parameters=[params_file,
                    {'use_sim_time': False}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_exploration',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': ['roadmap_explorer_node']}, {'use_sim_time': False}],
    )

    return launch.LaunchDescription([
        roadmap_explorer_node,
        lifecycle_manager
    ])
