from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "sensor_srvcli", package='sensor_srvcli', executable='server'),
        launch_ros.actions.Node(
            namespace= "sensor_srvcli", package='sensor_srvcli', executable='client'),
    ])