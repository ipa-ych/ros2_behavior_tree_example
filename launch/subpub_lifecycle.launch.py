from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from pathlib import Path

import pdb

def generate_launch_description():
    package_dir = Path(get_package_share_directory('ros2_behavior_tree_example'))
    behavior_tree_dir = package_dir / 'behavior_trees'

    # limit choices so we can only have available files
    mode_choices = ["sequence", "reactive_sequence"]
    tree_choices = ["ping_pong.xml", "ping_pong_no_decorator.xml", "ping_pong_executor.xml", "pubsub.xml", "pubsub_test.xml"]
    enable_choices = ["True", "False"]

    node1_enable_arg = DeclareLaunchArgument("node1_enable", 
                                            default_value="True",
                                            choices=enable_choices,
                                            description="Enable Primary Node in case you want to launch separately")
    
    node1_mode_arg = DeclareLaunchArgument("node1_mode", 
                                            default_value="reactive_sequence", 
                                            choices=mode_choices, 
                                            description="Set trees to reactive sequence")


    node1_behaviortree_arg = DeclareLaunchArgument("node1_behaviortree", 
                                                    default_value="pubsub.xml", 
                                                    choices=tree_choices, 
                                                    description="Set behevior tree file to use desired nodes")


    primary_ping_pong_node = Node(
        condition=IfCondition(LaunchConfiguration("node1_enable")),
        package='ros2_behavior_tree_example',
        executable='behavior_tree_example',
        name='pubsub',
        output='screen',
        parameters=[{
            "rate_hz" : 1.0,
            "num_republish": 5,
            "ping_starter" : True,
            "behaviortree_file" : PathJoinSubstitution([str(behavior_tree_dir),
                                                        LaunchConfiguration("node1_mode"),
                                                        LaunchConfiguration("node1_behaviortree")])
        }]
    )



    return LaunchDescription([
        node1_enable_arg,
        node1_mode_arg,
        node1_behaviortree_arg,
        primary_ping_pong_node
    ])