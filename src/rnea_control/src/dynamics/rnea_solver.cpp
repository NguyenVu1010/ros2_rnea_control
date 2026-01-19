#include "rnea_control/dynamics/rnea_solver.hpp"
#include "rnea_control/core/math_utils.hpp"
#include <cmath>

namespace rnea_control {
namespace dynamics {

RNEASolver::RNEASolver(const std::vector<core::LinkParams>& params) 
    : params_(params) {
    n_ = static_cast<int>(params.size());
    
    // Resize các vector động lực học (n + 1 vì tính cả Base)
    w_.resize(n_ + 1, Eigen::Vector3d::Zero());
    dw_.resize(n_ + 1, Eigen::Vector3d::Zero());
    dv_.resize(n_ + 1, Eigen::Vector3d::Zero());
    
    f_ext_.resize(n_ + 1, Eigen::Vector3d::Zero());
    n_ext_.resize(n_ + 1, Eigen::Vector3d::Zero());
    
    F_i_.resize(n_, Eigen::Vector3d::Zero());
    N_i_.resize(n_, Eigen::Vector3d::Zero());
}

void RNEASolver::solve(const std::vector<double>& q_in, 
                       const std::vector<double>& dq_in, 
                       const std::vector<double>& ddq_in, 
                       std::vector<double>& tau_out) {
    
    // Map std::vector sang Eigen để tính toán nhanh (Zero-copy)
    Eigen::Map<const Eigen::VectorXd> q(q_in.data(), n_);
    Eigen::Map<const Eigen::VectorXd> dq(dq_in.data(), n_);
    Eigen::Map<const Eigen::VectorXd> ddq(ddq_in.data(), n_);

    // Resize output nếu cần
    if (tau_out.size() != static_cast<size_t>(n_)) {
        tau_out.resize(n_);
    }

    // Vector đơn vị trục Z (Trục chuyển động chuẩn DH)
    const Eigen::Vector3d z0(0, 0, 1);
    
    // --- KHỞI TẠO ĐIỀU KIỆN BIÊN (BASE) ---
    // Base cố định, vận tốc = 0
    w_[0].setZero();
    dw_[0].setZero();
    // Gia tốc trọng trường ảo hướng lên (để tạo lực trọng trường hướng xuống)
    dv_[0] = Eigen::Vector3d(0, 0, 9.81); 

    // =========================================================================
    // 1. FORWARD RECURSION (Lan truyền Vận tốc & Gia tốc: Base -> End-Effector)
    // =========================================================================
    for (int i = 0; i < n_; ++i) {
        Eigen::Matrix3d R;
        Eigen::Vector3d p_curr = params_[i].p;

        // Xử lý biến khớp (q) tùy theo loại khớp
        if (params_[i].type == core::REVOLUTE) {
            // Khớp xoay: q là góc quay -> R thay đổi
            R = core::MathUtils::get_rotation_matrix_z(q[i]);
        } else {
            // Khớp tịnh tiến: q là độ dài -> p thay đổi dọc trục Z
            R = Eigen::Matrix3d::Identity();
            p_curr += z0 * q[i]; 
        }

        // Chuyển vị ma trận xoay (Transpose = Inverse) để chuyển từ Frame i-1 sang i
        Eigen::Matrix3d RT = R.transpose();

        if (params_[i].type == core::REVOLUTE) {
            // --- CÔNG THỨC CHO KHỚP XOAY ---
            // Vận tốc góc: w(i) = R^T * (w(i-1) + z0 * dq)
            w_[i+1] = RT * (w_[i] + z0 * dq[i]);
            
            // Gia tốc góc: dw(i) = R^T * (dw(i-1) + z0 * ddq + w(i-1) x (z0 * dq))
            dw_[i+1] = RT * (dw_[i] + z0 * ddq[i] + w_[i].cross(z0 * dq[i]));
            
            // Gia tốc tuyến tính: dv(i) = R^T * dv(i-1) + dw x p + w x (w x p)
            dv_[i+1] = RT * dv_[i] + dw_[i+1].cross(p_curr) + 
                       w_[i+1].cross(w_[i+1].cross(p_curr));
        } else {
            // --- CÔNG THỨC CHO KHỚP TỊNH TIẾN ---
            // Vận tốc góc không đổi (chỉ quay theo frame trước)
            w_[i+1] = RT * w_[i];
            dw_[i+1] = RT * dw_[i];
            
            // Gia tốc tuyến tính có thêm thành phần Coriolis (2 * w x v_rel) và gia tốc tịnh tiến
            // dv(i) = R^T * (dv(i-1) + z0*ddq + 2*w x (z0*dq)) + ...
            Eigen::Vector3d coriolis = 2.0 * w_[i].cross(z0 * dq[i]);
            Eigen::Vector3d linear_acc = z0 * ddq[i];
            
            dv_[i+1] = RT * (dv_[i] + linear_acc + coriolis) + 
                       dw_[i+1].cross(p_curr) + 
                       w_[i+1].cross(w_[i+1].cross(p_curr));
        }
        
        // Gia tốc tại trọng tâm (CoM)
        Eigen::Vector3d dv_c = dv_[i+1] + dw_[i+1].cross(params_[i].com) + 
                              w_[i+1].cross(w_[i+1].cross(params_[i].com));
        
        // Tính Lực và Momen quán tính (Newton-Euler Equations)
        // F = m * a_com
        F_i_[i] = params_[i].m * dv_c;
        // N = I * dw + w x (I * w)
        N_i_[i] = params_[i].I * dw_[i+1] + w_[i+1].cross(params_[i].I * w_[i+1]);
    }

    // =========================================================================
    // 2. BACKWARD RECURSION (Cân bằng Lực: End-Effector -> Base)
    // =========================================================================
    
    // Giả sử không có lực tác động tại điểm cuối (No external load)
    f_ext_[n_].setZero();
    n_ext_[n_].setZero();

    for (int i = n_ - 1; i >= 0; --i) {
        Eigen::Matrix3d R_next;
        Eigen::Vector3d p_next_curr;

        // Tính ma trận xoay và vị trí tương đối của khớp kế tiếp (để chiếu lực về)
        if (i < n_ - 1) {
            p_next_curr = params_[i+1].p; // Vị trí gốc i+1 so với i
            
            if (params_[i+1].type == core::REVOLUTE) {
                R_next = core::MathUtils::get_rotation_matrix_z(q[i+1]);
            } else {
                R_next = Eigen::Matrix3d::Identity();
                p_next_curr += z0 * q[i+1]; // Cộng thêm phần tịnh tiến
            }
        } else {
            // Khớp cuối cùng, không có khớp sau nó
            R_next.setIdentity();
            p_next_curr.setZero();
        }
        
        // Cân bằng Lực: f(i) = R_next * f(i+1) + F_inertia(i)
        f_ext_[i] = R_next * f_ext_[i+1] + F_i_[i];
        
        // Cân bằng Momen: n(i) = N_inertia(i) + R_next * n(i+1) + p_com x F_inertia + p_next x (R_next * f(i+1))
        // Lưu ý: R_next * f_ext_[i+1] chính là lực của khớp sau tác động lên khớp trước (đã chiếu về frame hiện tại)
        Eigen::Vector3d f_next_projected = R_next * f_ext_[i+1];
        
        n_ext_[i] = N_i_[i] + R_next * n_ext_[i+1] + 
                    params_[i].com.cross(F_i_[i]) + 
                    p_next_curr.cross(f_next_projected); 
        
        // --- OUTPUT: Lấy thành phần dọc trục chuyển động (Z) ---
        if (params_[i].type == core::REVOLUTE) {
            tau_out[i] = n_ext_[i].dot(z0); // Momen xoay (Nm)
        } else {
            tau_out[i] = f_ext_[i].dot(z0); // Lực đẩy (N)
        }
    }
}

} // namespace dynamics
} // namespace rnea_control