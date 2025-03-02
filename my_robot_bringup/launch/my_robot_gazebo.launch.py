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
        parameters=[{"robot_description": robot_description,
                             "use_sim_time": True  
                    }]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen"
    )

    # Set Gazebo model path (adjust as needed to locate your models)
    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[os.path.join(my_robot_dir, "models")]
    )

    # Form the path to the Gazebo Classic launch file from gazebo_ros package
    gazebo_launch_path = os.path.join(
        get_package_share_directory("gazebo_ros"),
        "launch",
        "gazebo.launch.py"
    )

    # Include Gazebo Classic simulation launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={"verbose": "true"}.items()
    )

    # Spawn the robot in Gazebo Classic using spawn_entity.py
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "/robot_description", "-entity", "my_robot"]
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")]
    )

    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_model_path,
        gazebo,
        spawn_entity,
        rviz_node
    ])
