import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    # ========================================================================
    # 1. KHAI BÁO ĐƯỜNG DẪN
    # ========================================================================
    pkg_robot = get_package_share_directory('my_sr_arm_rb')
    pkg_control = get_package_share_directory('rnea_control')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Đường dẫn file URDF thuần
    urdf_file_path = os.path.join(pkg_robot, 'urdf', 'my_sr_arm_rb.urdf')
    
    # Đường dẫn file YAML Controller
    controller_yaml_path = os.path.join(pkg_control, 'config', 'rnea_params.yaml')

    # ========================================================================
    # 2. XỬ LÝ URDF (MẤU CHỐT CỦA VẤN ĐỀ)
    # ========================================================================
    # Đọc nội dung file URDF
    with open(urdf_file_path, 'r') as file:
        urdf_content = file.read()
    urdf_content = urdf_content.replace('<?xml version="1.0" encoding="utf-8"?>', '')
    # Tìm từ khóa 'PATH_TO_MY_CONFIG' và thay thế bằng đường dẫn tuyệt đối
    # Điều này giúp Gazebo tìm được file config dù bạn chạy ở máy nào
    robot_description_content = urdf_content.replace("PATH_TO_MY_CONFIG", controller_yaml_path)

    # ========================================================================
    # 3. CẤU HÌNH CÁC NODE
    # ========================================================================
    
    # A. Robot State Publisher (Gửi URDF đã sửa đổi lên topic)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # B. Gazebo Server & Client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        # Vẫn truyền thêm tham số này để đảm bảo controller_manager node của ROS nhận được params
        launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file ' + controller_yaml_path
        }.items()
    )

    # C. Spawn Robot (Lấy model từ topic robot_description)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'sr_arm_robot'],
        output='screen'
    )

    # D. Spawner: Joint State Broadcaster
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    # E. Spawner: RNEA Controller
    load_rnea_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rnea_controller", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    # ========================================================================
    # 4. QUẢN LÝ TRÌNH TỰ CHẠY (EVENT HANDLERS)
    # ========================================================================
    
    # Đợi Robot Spawn xong mới load Joint State Broadcaster
    delay_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    # Đợi Joint State Broadcaster xong mới load RNEA Controller
    delay_rnea_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_rnea_controller],
        )
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        delay_jsb_after_spawn,
        delay_rnea_after_jsb
    ])