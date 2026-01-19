#include "rnea_control/core/math_utils.hpp"

namespace rnea_control {
namespace core {

// =============================================================================
// Hàm tính Ma trận xoay quanh trục Z
// =============================================================================
Eigen::Matrix3d MathUtils::get_rotation_matrix_z(double theta) {
    double s, c;
    #ifdef __linux__ // Dùng sincos cho hiệu suất cao trên Linux
        sincos(theta, &s, &c);
    #else
        s = std::sin(theta); c = std::cos(theta);
    #endif
    Eigen::Matrix3d R;
    R << c, -s, 0,
         s,  c, 0,
         0,  0, 1;
    return R;
}

// =============================================================================
// Hàm tính Ma trận biến đổi thuần nhất DH (Standard DH Parameters)
// Công thức: T = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
// =============================================================================
Eigen::Matrix4d MathUtils::get_dh_matrix(double a, double alpha, double d, double theta) {
    double st, ct, sa, ca;
    #ifdef __linux__ // Dùng sincos cho hiệu suất cao trên Linux
        sincos(theta, &st, &ct);
        sincos(alpha, &sa, &ca);
    #else
        st = std::sin(theta); ct = std::cos(theta);
        sa = std::sin(alpha); ca = std::cos(alpha);
    #endif
    
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0,0) = ct; T(0,1) = -st * ca; T(0,2) = st * sa; T(0,3) = a * ct;
    T(1,0) = st; T(1,1) =  ct * ca; T(1,2) = -ct * sa; T(1,3) = a * st;
    T(2,0) = 0;  T(2,1) =  sa;      T(2,2) = ca;       T(2,3) = d;
    // T(3,0)=0; T(3,1)=0; T(3,2)=0; T(3,3)=1; (đã là Identity)
    return T;
}

// =============================================================================
// Hàm chuyển đổi Euler (Roll, Pitch, Yaw) sang Quaternion
// = =============================================================================
Eigen::Quaterniond MathUtils::euler_to_quat(double roll, double pitch, double yaw) {
    // Dùng Eigen::AngleAxis để chuyển đổi dễ hơn
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

} // namespace core
} // namespace rnea_control