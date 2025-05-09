from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch_ros.parameter_descriptions import ParameterValue  # ADD THIS


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Dynamically build robot_description from xacro
    robot_description_content = ParameterValue(  # FIX HERE
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare("articubot_one"), "description", "robot.urdf.xacro"])
        ]),
        value_type=str
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true"
        ),
        robot_state_publisher_node
    ])
