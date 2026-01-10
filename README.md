# ros2_rnea_control

Workspace ROS 2 (Humble) phục vụ mô phỏng và điều khiển robot tay máy, bao gồm:
- Mô tả robot, mô phỏng và hiển thị
- Controller động lực học dựa trên RNEA (Recursive Newton–Euler Algorithm) tích hợp với `ros2_control`

---

## 1. Cấu trúc workspace

```text
ros2_rnea_control/
├── src/
│   ├── my_sr_arm_rb/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── launch/
│   │   ├── config/
│   │   ├── urdf/
│   │   ├── meshes/
│   │   ├── textures/
│   │   ├── rviz.rviz
│   │   └── src/
│   └── rnea_control/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── include/
│       ├── src/
│       ├── config/
│       └── rnea_controller_plugins.xml
├── build/
├── install/
└── log/
