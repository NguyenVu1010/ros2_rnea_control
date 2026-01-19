#ifndef RNEA_CONTROL_CORE_TYPES_HPP
#define RNEA_CONTROL_CORE_TYPES_HPP

#include <Eigen/Dense>  // Cần Eigen cho Vector3d, Matrix3d, Quaterniond
#include <vector>       // Cần vector cho std::vector

namespace rnea_control {
namespace core {

// Định nghĩa loại khớp (Prismatic = Tịnh tiến, Revolute = Xoay)
enum JointType { REVOLUTE, PRISMATIC };

// Cấu trúc chứa thông số động lực học và hình học (DH) của một Link
struct LinkParams {
    JointType type;         // Loại khớp nối Link này với Link trước
    double m;               // Khối lượng của Link (kg)
    Eigen::Vector3d com;    // Vị trí trọng tâm của Link (so với gốc Link) (m)
    Eigen::Matrix3d I;      // Ma trận quán tính của Link (kg*m^2)
    Eigen::Vector3d p;      // Vector vị trí từ gốc của Frame trước đến gốc của Frame này (để RNEA dùng)
    
    // Tham số Denavit-Hartenberg (DH)
    double a;               // Chiều dài Link (khoảng cách x giữa Z(i) và Z(i-1))
    double alpha;           // Góc xoắn Link (góc xoay Z(i-1) quanh X(i-1) để trùng Z(i))
    double d;               // Offset Link (khoảng cách z giữa X(i) và X(i-1))
    double theta;           // Góc khớp (góc xoay X(i-1) quanh Z(i-1) để trùng X(i))

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Cần thiết cho Eigen để căn chỉnh bộ nhớ
};

// Cấu trúc mô tả trạng thái Cartesian của một điểm/khâu
// Dùng cho cả trạng thái hiện tại (current) và mong muốn (desired)
struct CartesianState {
    Eigen::Vector3d p;      // Vị trí tuyến tính (x, y, z) (m)
    Eigen::Quaterniond q;   // Hướng xoay (Quaternion)
    
    Eigen::Vector3d v_lin;  // Vận tốc tuyến tính (vx, vy, vz) (m/s)
    Eigen::Vector3d v_ang;  // Vận tốc góc (wx, wy, wz) (rad/s)
    
    Eigen::Vector3d a_lin;  // Gia tốc tuyến tính (ax, ay, az) (m/s^2)
    Eigen::Vector3d a_ang;  // Gia tốc góc (ax, ay, az) (rad/s^2)

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Cần thiết cho Eigen
};

// Cấu trúc mô tả một điểm trên quỹ đạo khớp
struct JointTrajectoryPoint {
    double q = 0.0;     // Vị trí khớp
    double dq = 0.0;    // Vận tốc khớp
    double ddq = 0.0;   // Gia tốc khớp
};

} // namespace core
} // namespace rnea_control
#endif // RNEA_CONTROL_CORE_TYPES_HPP