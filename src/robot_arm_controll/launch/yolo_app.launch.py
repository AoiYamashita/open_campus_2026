import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    yolo = launch_ros.actions.Node(
            package = "robot_arm_controll",
            executable = "yolo",
            output = "screen"
            )
    controller = launch_ros.actions.Node(
            package = "robot_arm_controll",
            executable = "controll",
            output = "screen"
            )
    web_socket = launch_ros.actions.Node(
		package = "rosbridge_server",
		executable = "rosbridge_websocket",
		output = "screen"
		)

    return launch.LaunchDescription([yolo,controller,web_socket])