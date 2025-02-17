import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    my_robot_dir = get_package_share_directory("my_robot")

    # Declare argument for model path
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(my_robot_dir, "urdf", "my_robot.urdf.xacro"),
        description="Absolute path to the robot URDF file"
    )

    # Declare argument for RViz config file
    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=os.path.join(my_robot_dir, "rviz", "urdf_config.rviz"),
        description="Absolute path to RViz configuration file"
    )

    # Generate robot_description from Xacro
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # Start robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Set Gazebo resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(my_robot_dir).parent.resolve())]
    )

    # Include Gazebo simulation launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"
        ]),
        launch_arguments=[("gz_args", [" -v 4", " -r ", "empty.sdf"])]
    )

    # Spawn the robot in Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "test_robot"]
    )

    # ROS-Gazebo Bridge for Communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',  # Bridge for cmd_vel
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',    # Bridge for odometry
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',  # Camera image bridge
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'  # Camera info bridge
        ],
        output='screen'
    )

    # Teleop Twist Keyboard for cmd_vel input
    teleop_twist_keyboard_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        output="screen",
        name="teleop_twist_keyboard",
        remappings=[("/cmd_vel", "/cmd_vel")]
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")]
    )

    # Return launch description
    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        bridge,
        teleop_twist_keyboard_node,
        rviz_node  # Added RViz node
    ])
