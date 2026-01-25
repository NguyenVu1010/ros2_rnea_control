import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # ========================================================================
    # 1. KHAI BÁO ĐƯỜNG DẪN
    # ========================================================================
    pkg_robot = get_package_share_directory('bring_up')
    pkg_control = get_package_share_directory('rnea_control')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # File Xacro chính
    xacro_file = os.path.join(pkg_robot, 'urdf', 'robot.urdf.xacro')
    
    # File Config (Cần truyền vào dù không chạy controller để Xacro không báo lỗi thiếu arg)
    controller_yaml_path = os.path.join(pkg_control, 'config', 'rnea_params.yaml')

    # ========================================================================
    # 2. XỬ LÝ XACRO
    # ========================================================================
    print(f"Processing Xacro: {xacro_file}")
    
    # Xử lý file xacro và truyền biến config_file
    doc = xacro.process_file(xacro_file, mappings={'config_file': controller_yaml_path})
    robot_description_content = doc.toxml()
    
    # [FIX LỖI] Xóa dòng XML header để tránh lỗi spawn_entity
    # Tìm và xóa dòng <?xml ... ?>
    if robot_description_content.startswith("<?xml"):
        robot_description_content = robot_description_content.split("\n", 1)[1]

    # ========================================================================
    # 3. CẤU HÌNH NODE
    # ========================================================================
    
    # A. Robot State Publisher (Bắt buộc để có TF)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # B. Gazebo (Chỉ chạy môi trường vật lý, không load controller manager parameters)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        # Không cần extra_gazebo_args ở đây vì ta chỉ test model
    )

    # C. Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description', 
            '-entity', 'sr_arm_robot', 
            '-z', '0.05' # Nâng nhẹ lên
        ],
        output='screen'
    )

    # D. RViz2 (Để soi kỹ Visual và Collision)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ========================================================================
    # 4. KẾT THÚC (KHÔNG CÓ CONTROLLER SPAWNER)
    # ========================================================================
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        rviz_node
    ])