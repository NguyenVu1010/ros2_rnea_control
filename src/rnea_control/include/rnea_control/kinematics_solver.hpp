#ifndef RNEA_CONTROL__KINEMATICS_SOLVER_HPP_
#define RNEA_CONTROL__KINEMATICS_SOLVER_HPP_

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include "rnea_control/rneasolver.hpp"

namespace rnea_control {

class KinematicsSolver {
public:
    explicit KinematicsSolver(const std::vector<LinkParams>& params) 
        : params_(params), n_(params.size()) {
        
        // 1. Pre-allocate memory (Cấp phát trước bộ nhớ)
        T_global_.resize(n_ + 1, Eigen::Matrix4d::Identity());
        
        // Cấp phát trước Jacobian để không phải malloc trong vòng lặp
        J_.resize(6, n_);
        J_.setZero();

        // Khởi tạo bộ đệm q cũ kích thước khác để ép buộc tính toán lần đầu
        last_q_.resize(0); 
    }

    // Trả về tham chiếu const (không copy) để tối ưu
    const Eigen::Vector3d& computeFK(const std::vector<double>& q) {
        updateTransforms(q);
        // Lưu vị trí End-Effector vào biến thành viên để trả về tham chiếu
        p_ee_cache_ = T_global_[n_].block<3,1>(0,3);
        return p_ee_cache_;
    }

    // Trả về tham chiếu const tới ma trận Jacobian đã tính sẵn
    const Eigen::MatrixXd& computeJacobian(const std::vector<double>& q) {
        updateTransforms(q);
        
        // Chỉ tính toán Jacobian khi cần thiết, sử dụng lại T_global_ đã update
        Eigen::Vector3d p_ee = T_global_[n_].block<3,1>(0,3);

        for (int i = 0; i < n_; ++i) {
            // Truy cập trực tiếp bộ nhớ, tránh tạo biến tạm
            Eigen::Vector3d z_axis = T_global_[i].block<3,1>(0,2); 
            Eigen::Vector3d p_joint = T_global_[i].block<3,1>(0,3);

            if (params_[i].type == PRISMATIC) {
                J_.block<3,1>(0,i) = z_axis;
                J_.block<3,1>(3,i).setZero();
            } else {
                // Tối ưu toán học: z.cross(p_ee - p_joint)
                J_.block<3,1>(0,i) = z_axis.cross(p_ee - p_joint);
                J_.block<3,1>(3,i) = z_axis;
            }
        }
        return J_;
    }

private:
    std::vector<LinkParams> params_;
    int n_;
    
    // --- CÁC BIẾN CACHE (Để tránh cấp phát động) ---
    std::vector<Eigen::Matrix4d> T_global_;
    Eigen::MatrixXd J_;
    std::vector<double> last_q_;
    Eigen::Vector3d p_ee_cache_;

    // Hàm update có cơ chế Caching (Quan trọng nhất để tăng tốc)
    void updateTransforms(const std::vector<double>& q) {
        // Kiểm tra: Nếu q không đổi so với lần trước -> Không tính lại
        if (q == last_q_) {
            return; 
        }

        // Nếu q thay đổi, cập nhật lại cache
        last_q_ = q; 

        // Frame 0 luôn là Identity
        // T_global_[0] đã là Identity từ lúc khởi tạo, không cần gán lại

        // Biến tạm để tính toán DH mà không cần tạo object mới
        double c_th, s_th, c_al, s_al;
        Eigen::Matrix4d T_i;

        for (int i = 0; i < n_; ++i) {
            double q_val = q[i];
            double a = params_[i].a;
            double alpha = params_[i].alpha;
            double d = params_[i].d;
            double theta = params_[i].theta;

            if (params_[i].type == PRISMATIC) d += q_val;
            else theta += q_val;

            // Tính nhanh sin/cos
            // Nếu trình biên dịch hỗ trợ (Linux), sincos() nhanh hơn gọi sin/cos riêng
            #ifdef __linux__
                sincos(theta, &s_th, &c_th);
                sincos(alpha, &s_al, &c_al);
            #else
                c_th = cos(theta); s_th = sin(theta);
                c_al = cos(alpha); s_al = sin(alpha);
            #endif

            // Xây dựng ma trận DH trực tiếp (Tránh gọi hàm con)
            T_i << c_th, -s_th * c_al,  s_th * s_al, a * c_th,
                   s_th,  c_th * c_al, -c_th * s_al, a * s_th,
                   0,     s_al,         c_al,        d,
                   0,     0,            0,           1;
            
            // Nhân ma trận: T_global[i+1] = T_global[i] * T_i
            // Eigen đã tối ưu phép nhân này rất tốt (dùng SIMD/AVX)
            T_global_[i+1].noalias() = T_global_[i] * T_i; 
            // .noalias() báo cho Eigen biết không có sự chồng lấn vùng nhớ, giúp nhân nhanh hơn
        }
    }
};

} // namespace
#endif