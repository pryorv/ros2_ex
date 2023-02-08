from launch import LaunchDescription
import launch_ros.actions
import sys
# def generate_launch_description():
#     return LaunchDescription([
#         launch_ros.actions.Node(
#             namespace= "turtlesim1", package='turtlesim', executable='turtlesim_node', output='screen'),
#         launch_ros.actions.Node(
#             namespace= "turtlesim2", package='turtlesim', executable='turtlesim_node', output='screen'),
#     ])


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "sensor_srvcli", package='sensor_srvcli', executable='server'),
        launch_ros.actions.Node(
            namespace= "sensor_srvcli", package='sensor_srvcli', executable='client'),
    ])