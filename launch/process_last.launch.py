import os.path

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwarg):

    nodes_to_start = list()

    data_processor_node = Node(
        package="dmp_ros2",
        executable="data_preprocessor",
        parameters=[{
            # "demonstrations_directory": ""
            "filter_weights": [2.0, 1.0, 1.0, 0.5],
            "segmentation": {
                "velocity_th": 0.1,
                "minium_window_length": 200,
                "window_size": 40,
                "tail_size": 60,
            },
        }])
    nodes_to_start += [
        data_processor_node,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments +
                             [OpaqueFunction(function=launch_setup)])
