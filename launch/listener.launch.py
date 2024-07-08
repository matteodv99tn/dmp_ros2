import os.path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch_ros.actions import Node
from launch_ros.substitutions.find_package import get_package_share_directory


def launch_setup(context, *args, **kwarg):

    nodes_to_start = list()

    urdf_file = os.path.join(get_package_share_directory("magician_material"),
                             "urdf", "car_chassis.urdf")
    rviz_file = os.path.join(get_package_share_directory("magician_material"),
                             "rviz2", "application.rviz")
    with open(urdf_file, "r") as fh:
        urdf_desc = fh.read()

    cad_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="CAD_model_publisher",
        output="screen",
        parameters=[{
            "robot_description": urdf_desc
        }],
        remappings=[("/robot_description", "/cad_description")],
    )

    # DMP nodes
    trajectory_writer_node = Node(package="dmp_ros2",
                                  executable="trajectory_listener",
                                  parameters=[{
                                      "input_topic": "/tool_pose"
                                  }])
    nodes_to_start += [trajectory_writer_node, ]

    # Optitrack-related nodes
    world_to_optitrack_node = Node(package="magician_estimators",
                                   executable="smooth_pose_to_tf",
                                   parameters=[{
                                       "base_frame":
                                       "optitrack",
                                       "child_frame":
                                       "world",
                                       "input_topic":
                                       "/optitrack_world_pose"
                                   }])
    optitrack_to_chassis_node = Node(package="magician_estimators",
                                     executable="smooth_pose_to_tf",
                                     parameters=[{
                                         "base_frame": "optitrack",
                                         "child_frame": "optitrack_chassis",
                                         "input_topic": "/chassis_pose"
                                     }])
    optitrack_to_tool_node = Node(package="magician_estimators",
                                  executable="smooth_pose_to_tf",
                                  parameters=[{
                                      "base_frame": "optitrack",
                                      "child_frame": "tool_marker",
                                      "input_topic": "/tool_pose"
                                  }])
    tool_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.035",
            "0.035",
            "-0.19",
            "-3.1415927",
            "0.0",
            "-1.5707963",
            "tool_marker",
            "tool",
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file],
    )

    nodes_to_start += [
        cad_pub_node,
        world_to_optitrack_node,
        optitrack_to_chassis_node,
        optitrack_to_tool_node,
        tool_static_tf,
        rviz,
    ]

    world_pose_broadcaster_node = Node(
        package="mocap_conversions",
        executable="mocap_to_posestamped",
        parameters=[{
            "pose_topic": "/optitrack_world_pose",
            "marker_id": "1",
        }],
    )
    tool_pose_broadcaster_node = Node(
        package="mocap_conversions",
        executable="mocap_to_posestamped",
        parameters=[{
            "pose_topic": "/tool_pose",
            "marker_id": "4",
        }],
    )
    chassis_pose_broadcaster_node = Node(
        package="mocap_conversions",
        executable="mocap_to_posestamped",
        parameters=[{
            "pose_topic": "/chassis_pose",
            "marker_id": "3",
        }],
    )
    nodes_to_start += [
        world_pose_broadcaster_node,
        tool_pose_broadcaster_node,
        chassis_pose_broadcaster_node,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments +
                             [OpaqueFunction(function=launch_setup)])
