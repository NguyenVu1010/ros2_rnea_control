#ifndef RNEA_CONTROL__RNEA_SOLVER_HPP_
#define RNEA_CONTROL__RNEA_SOLVER_HPP_

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include <string>

namespace rnea_control {

// 1. Định nghĩa Enum cho loại khớp
enum JointType { REVOLUTE, PRISMATIC };

struct LinkParams {
    JointType type; // <--- Thêm biến này
    double m;
    Eigen::Vector3d com;
    Eigen::Matrix3d I;
    Eigen::Vector3d p; 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class rneasolver {
public:
    explicit rneasolver(const std::vector<LinkParams>& params) : params_(params) {
        n_ = params.size();
        
        // Resize các vector
        w_.resize(n_ + 1, Eigen::Vector3d::Zero());
        dw_.resize(n_ + 1, Eigen::Vector3d::Zero());
        dv_.resize(n_ + 1, Eigen::Vector3d::Zero());
        f_ext_.resize(n_ + 1, Eigen::Vector3d::Zero());
        n_ext_.resize(n_ + 1, Eigen::Vector3d::Zero());
        
        F_i_.resize(n_, Eigen::Vector3d::Zero());
        N_i_.resize(n_, Eigen::Vector3d::Zero());
    }

    void solve(const std::vector<double>& q_in, 
               const std::vector<double>& dq_in, 
               const std::vector<double>& ddq_in, 
               std::vector<double>& tau_out) {
        
        Eigen::Map<const Eigen::VectorXd> q(q_in.data(), n_);
        Eigen::Map<const Eigen::VectorXd> dq(dq_in.data(), n_);
        Eigen::Map<const Eigen::VectorXd> ddq(ddq_in.data(), n_);

        if (tau_out.size() != static_cast<size_t>(n_)) tau_out.resize(n_);

        const Eigen::Vector3d z0(0, 0, 1); // Trục chuyển động luôn là Z trong hệ toạ độ gắn liền
        
        // Gia tốc trọng trường (hướng lên để tạo phản lực)
        dv_[0] = Eigen::Vector3d(0, 0, 9.81); 
        w_[0].setZero();
        dw_[0].setZero();

        // --- 1. Forward Recursion ---
        for (int i = 0; i < n_; ++i) {
            Eigen::Matrix3d R;
            Eigen::Vector3d p_curr = params_[i].p;

            // Xử lý riêng cho từng loại khớp
            if (params_[i].type == REVOLUTE) {
                // Khớp xoay: Ma trận xoay thay đổi theo q
                R = get_rotation_matrix(q[i]);
            } else {
                // Khớp tịnh tiến: Ma trận xoay cố định (Identity), 
                // Vị trí p thay đổi theo q (dịch chuyển dọc trục Z)
                R = Eigen::Matrix3d::Identity();
                p_curr += z0 * q[i]; 
            }

            Eigen::Matrix3d RT = R.transpose(); // Chuyển từ frame i-1 sang i

            if (params_[i].type == REVOLUTE) {
                // --- CÔNG THỨC KHỚP XOAY ---
                w_[i+1] = RT * (w_[i] + z0 * dq[i]);
                dw_[i+1] = RT * (dw_[i] + z0 * ddq[i] + w_[i].cross(z0 * dq[i]));
                // Gia tốc tuyến tính
                dv_[i+1] = RT * dv_[i] + dw_[i+1].cross(p_curr) + 
                           w_[i+1].cross(w_[i+1].cross(p_curr));
            } else {
                // --- CÔNG THỨC KHỚP TỊNH TIẾN ---
                w_[i+1] = RT * w_[i]; 
                dw_[i+1] = RT * dw_[i];
                // Gia tốc tuyến tính có thêm thành phần Coriolis (2 * w x v) và gia tốc tịnh tiến
                dv_[i+1] = RT * (dv_[i] + z0 * ddq[i] + 2 * w_[i].cross(z0 * dq[i])) + 
                           dw_[i+1].cross(p_curr) + 
                           w_[i+1].cross(w_[i+1].cross(p_curr));
            }
            
            // Lực quán tính tại trọng tâm (CoM)
            Eigen::Vector3d dv_c = dv_[i+1] + dw_[i+1].cross(params_[i].com) + 
                                  w_[i+1].cross(w_[i+1].cross(params_[i].com));
            
            F_i_[i] = params_[i].m * dv_c;
            N_i_[i] = params_[i].I * dw_[i+1] + w_[i+1].cross(params_[i].I * w_[i+1]);
        }

        // --- 2. Backward Recursion ---
        f_ext_[n_].setZero();
        n_ext_[n_].setZero();

        for (int i = n_ - 1; i >= 0; --i) {
            Eigen::Matrix3d R_next;
            Eigen::Vector3d p_next_curr = (i < n_ - 1) ? params_[i+1].p : Eigen::Vector3d::Zero();

            // Tính R và p của khớp kế tiếp để chiếu lực về
            if (i < n_ - 1) {
                if (params_[i+1].type == REVOLUTE) {
                    R_next = get_rotation_matrix(q[i+1]);
                } else {
                    R_next = Eigen::Matrix3d::Identity();
                    p_next_curr += z0 * q[i+1];
                }
            } else {
                R_next = Eigen::Matrix3d::Identity();
            }
            
            // Cân bằng Lực
            f_ext_[i] = R_next * f_ext_[i+1] + F_i_[i];
            
            // Cân bằng Momen
            n_ext_[i] = N_i_[i] + R_next * n_ext_[i+1] + 
                        params_[i].com.cross(F_i_[i]) + 
                        p_next_curr.cross(R_next * f_ext_[i+1]); 
            
            // TÍNH ĐẦU RA (TORQUE HOẶC FORCE)
            if (params_[i].type == REVOLUTE) {
                tau_out[i] = n_ext_[i].dot(z0); // Momen (Nm)
            } else {
                tau_out[i] = f_ext_[i].dot(z0); // Lực (N)
            }
        }
    }

private:
    int n_;
    std::vector<LinkParams> params_;
    std::vector<Eigen::Vector3d> w_, dw_, dv_, f_ext_, n_ext_;
    std::vector<Eigen::Vector3d> F_i_, N_i_;

    inline Eigen::Matrix3d get_rotation_matrix(double qi) {
        Eigen::Matrix3d R;
        double s = std::sin(qi), c = std::cos(qi);
        R << c, -s, 0,
             s,  c, 0,
             0,  0, 1;
        return R;
    }
};

} // namespace rnea_control
#endif