ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['joint_gripper1','joint_gripper2'], points: [{positions: [-0,0], time_from_start: {sec: 2, nanosec: 0}}]}"

ros2 topic pub --once /rnea_controller/commands std_msgs/msg/Float64MultiArray "{data: [-0.15, 1.0, 0.0, 0.0, 0.0, 3.0]}"

ros2 topic pub --once /rnea_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.3, 0.3, 0.5, 0.0, 0.0, 0.0, 0.1, 0.05]}"^C