import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. KHAI BÁO ĐƯỜNG DẪN (Đã cập nhật tên package)
    pkg_robot = get_package_share_directory('bring_up')
    pkg_control = get_package_share_directory('rnea_control')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # File Xacro & Config
    xacro_file = os.path.join(pkg_robot, 'urdf', 'robot.urdf.xacro')
    controller_yaml_path = os.path.join(pkg_control, 'config', 'rnea_params.yaml')

    # 2. XỬ LÝ XACRO
    print(f"Processing Xacro: {xacro_file}")
    doc = xacro.process_file(xacro_file, mappings={'config_file': controller_yaml_path})
    robot_desc = doc.toxml()
    
    # Fix lỗi XML Header
    if robot_desc.startswith("<?xml"):
        robot_desc = robot_desc.split("\n", 1)[1]

    # 3. NODES
    # A. Robot State Publisher
    node_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # B. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file ' + controller_yaml_path
        }.items()
    )

    # C. Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'sr_arm_robot', '-z', '0.05'],
        output='screen'
    )

    # D. Controllers
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': True}]
    )

    spawn_rnea = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rnea_controller", "--controller-manager", "/controller_manager"],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    spawn_gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 4. EVENT HANDLERS (Trình tự load)
    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_entity, on_exit=[spawn_jsb])
    )
    delay_rnea = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_rnea])
    )
    delay_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_rnea, on_exit=[spawn_gripper])
    )

    return LaunchDescription([
        node_rsp,
        gazebo,
        spawn_entity,
        delay_jsb,
        delay_rnea,
        delay_gripper
    ])