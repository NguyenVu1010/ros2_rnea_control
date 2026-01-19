#ifndef RNEA_CONTROL_CORE_MATH_UTILS_HPP
#define RNEA_CONTROL_CORE_MATH_UTILS_HPP

#include <Eigen/Dense>      // Cần Eigen cho các kiểu Vector/Matrix
#include <Eigen/Geometry>   // Cần cho Quaternion
#include <cmath>            // Cần cho sin/cos

namespace rnea_control {
namespace core {

class MathUtils {
public:
    // Hàm tính ma trận xoay quanh trục Z (dùng cho RNEA)
    static Eigen::Matrix3d get_rotation_matrix_z(double theta);
    
    // Hàm tính ma trận biến đổi thuần nhất Denavit-Hartenberg (DH Classic)
    static Eigen::Matrix4d get_dh_matrix(double a, double alpha, double d, double theta);

    // Hàm chuyển đổi góc Euler (Roll, Pitch, Yaw) sang Quaternion
    // Roll: X, Pitch: Y, Yaw: Z
    static Eigen::Quaterniond euler_to_quat(double roll, double pitch, double yaw);
};

} // namespace core
} // namespace rnea_control
#endif // RNEA_CONTROL_CORE_MATH_UTILS_HPP