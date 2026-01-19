#include "rnea_control/control/motion_control.hpp"
#include <iostream>

namespace rnea_control {
namespace control {

MotionControl::MotionControl() 
    : kp_pos_(10.0), kp_ori_(10.0), kp_null_(1.0) {}

void MotionControl::set_gains(double kp_pos, double kp_ori, double kp_null) {
    kp_pos_ = kp_pos;
    kp_ori_ = kp_ori;
    kp_null_ = kp_null;
}

// =============================================================================
// THUẬT TOÁN CLIK (Closed-Loop Inverse Kinematics)
// =============================================================================
void MotionControl::compute_cartesian_tracking(
    const core::CartesianState& desired,
    const core::CartesianState& current,
    const Eigen::MatrixXd& J,
    const std::vector<double>& dq_curr,
    std::vector<double>& ddq_cmd_out
) {
    size_t n_joints = dq_curr.size();
    if (ddq_cmd_out.size() != n_joints) ddq_cmd_out.resize(n_joints);

    // 1. Tính sai số Vị trí (Position Error)
    // e_p = p_des - p_curr
    Eigen::Vector3d error_pos = desired.p - current.p;

    // 2. Tính sai số Hướng (Orientation Error)
    // e_o = current.q * log(current.q^-1 * desired.q)
    // Cách tính nhanh: Lấy phần vector của Quaternion sai lệch
    Eigen::Quaterniond q_err = current.q.inverse() * desired.q;
    Eigen::Vector3d error_ori = q_err.vec(); 
    
    // Xử lý trường hợp quaternion đôi (q và -q là một)
    if (q_err.w() < 0) {
        error_ori = -error_ori; 
    }
    
    // Chuyển sai số từ Body Frame về Base Frame
    error_ori = current.q.toRotationMatrix() * error_ori;

    // 3. Tổng hợp Vector vận tốc Task Space mong muốn (v_task)
    // v_task = v_feedforward + Kp * error
    Eigen::VectorXd v_task(6);
    v_task.head<3>() = desired.v_lin + kp_pos_ * error_pos;
    v_task.tail<3>() = desired.v_ang + kp_ori_ * error_ori;

    // 4. Tính Jacobian Nghịch đảo (Pseudo-Inverse)
    // Dùng phương pháp SVD (CompleteOrthogonalDecomposition) để ổn định khi qua điểm kỳ dị
    Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();

    // 5. Tính vận tốc khớp mong muốn (Inverse Kinematics vi phân)
    // dq_cmd = J# * v_task
    Eigen::VectorXd dq_cmd = J_pinv * v_task;

    // --- (Tùy chọn) Null-space optimization ---
    // Nếu robot dư dẫn động (n > 6), thêm thành phần null-space để giữ tư thế
    // Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_joints, n_joints);
    // Eigen::VectorXd null_task = -kp_null_ * Eigen::Map<const Eigen::VectorXd>(dq_curr.data(), n_joints);
    // dq_cmd += (I - J_pinv * J) * null_task;

    // 6. Tính gia tốc khớp cho RNEA (Output)
    // RNEA cần gia tốc (ddq). Ta có vận tốc mong muốn (dq_cmd).
    // Dùng bộ điều khiển P vận tốc để tạo ra gia tốc:
    // ddq = K_v * (dq_cmd - dq_curr)
    // K_v nên chọn lớn (ví dụ 100.0) để phản ứng nhanh
    
    double kv_joint = 50.0; 

    for (size_t i = 0; i < n_joints; ++i) {
        // Feedforward Acceleration (nếu có, từ J_dot * dq, nhưng thường bỏ qua vì phức tạp)
        // Cộng với Feedback vận tốc
        ddq_cmd_out[i] = kv_joint * (dq_cmd[i] - dq_curr[i]);
    }
}

// =============================================================================
// THUẬT TOÁN JOINT TRACKING (Computed Torque Control cơ bản)
// =============================================================================
void MotionControl::compute_joint_tracking(
    const std::vector<double>& q_des,
    const std::vector<double>& dq_des,
    const std::vector<double>& ddq_des,
    const std::vector<double>& q_curr,
    const std::vector<double>& dq_curr,
    const std::vector<double>& kp,
    const std::vector<double>& kd,
    std::vector<double>& ddq_cmd_out
) {
    size_t n = q_curr.size();
    if (ddq_cmd_out.size() != n) ddq_cmd_out.resize(n);

    for (size_t i = 0; i < n; ++i) {
        double e = q_des[i] - q_curr[i];
        double de = dq_des[i] - dq_curr[i];

        // Công thức CTC: ddq_ref = ddq_des + Kp*e + Kd*de
        ddq_cmd_out[i] = ddq_des[i] + kp[i] * e + kd[i] * de;
    }
}

} // namespace control
} // namespace rnea_control