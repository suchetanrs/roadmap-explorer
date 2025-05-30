#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

#---------------------------------------------

    #Essential_paths
    explore_pkg = get_package_share_directory('roadmap_explorer')
#---------------------------------------------

    # LAUNCH ARGS
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    robot_namespace =  LaunchConfiguration('robot_namespace')
    robot_namespace_arg = DeclareLaunchArgument('robot_namespace', default_value=TextSubstitution(text=""),
        description='The namespace of the robot')
    
#---------------------------------------------

    def all_nodes_launch(context):
        params_file = LaunchConfiguration('params_file')
        declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(explore_pkg, 'params', 'exploration_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes')
        xml_file = os.path.join(explore_pkg, 'xml', 'exploration.xml')
        print("XML file: " + xml_file)
        base_frame = ""
        if(context.launch_configurations['robot_namespace'] == ""):
            base_frame = ""
        else:
            base_frame = context.launch_configurations['robot_namespace'] + "/"

        param_substitutions = {
            'robot_base_frame': base_frame + 'base_footprint',
            'use_sim_time': context.launch_configurations['use_sim_time'],
            'bt_xml_path': xml_file
            }

        configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=context.launch_configurations['robot_namespace'],
            param_rewrites=param_substitutions,
            convert_types=True)
        
        explore_server = Node(
            package='roadmap_explorer',
            executable='explore_server',
            output='screen',
            # prefix=['gdbserver localhost:3000'],
            # emulate_tty=True,
            namespace=context.launch_configurations['robot_namespace'],
            parameters=[configured_params])

        return [declare_params_file_cmd, 
                explore_server
                ]

    opaque_function = OpaqueFunction(function=all_nodes_launch)
#---------------------------------------------

    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_namespace_arg,
        opaque_function
    ])
