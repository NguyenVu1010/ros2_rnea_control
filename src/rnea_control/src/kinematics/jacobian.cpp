#include "rnea_control/kinematics/jacobian.hpp"

namespace rnea_control {
namespace kinematics {

JacobianSolver::JacobianSolver(const std::vector<core::LinkParams>& params)
    : params_(params), n_(params.size()) {
    
    // Cấp phát trước bộ nhớ cho Jacobian (6 hàng, n cột)
    J_.resize(6, n_);
    J_.setZero();
}

const Eigen::MatrixXd& JacobianSolver::compute(const std::vector<Eigen::Matrix4d>& transforms) {
    // Reset về 0 trước khi tính
    J_.setZero();

    // Vị trí End-Effector (gốc của frame cuối cùng n)
    Eigen::Vector3d p_ee = transforms[n_].block<3,1>(0,3);

    for (size_t i = 0; i < n_; ++i) {
        // Trong chuẩn DH Standard:
        // Khớp i+1 thực hiện chuyển động quanh/dọc trục Z của Frame i.
        // transforms[0] là Base Frame -> Trục Z của nó là trục quay của Joint 1.
        // transforms[1] là Link 1 Frame -> Trục Z của nó là trục quay của Joint 2.
        
        Eigen::Vector3d z_axis = transforms[i].block<3,1>(0,2); // Cột 3 (index 2) là trục Z
        Eigen::Vector3d p_joint = transforms[i].block<3,1>(0,3); // Cột 4 (index 3) là vị trí gốc

        if (params_[i].type == core::PRISMATIC) {
            // --- KHỚP TỊNH TIẾN ---
            // Vận tốc dài = z_axis
            // Vận tốc góc = 0
            J_.block<3,1>(0,i) = z_axis;
            // Phần góc (3 hàng dưới) để 0 (đã setZero từ đầu)
        } else {
            // --- KHỚP XOAY ---
            // Vận tốc dài = omega x r = z_axis x (p_ee - p_joint)
            J_.block<3,1>(0,i) = z_axis.cross(p_ee - p_joint);
            
            // Vận tốc góc = z_axis
            J_.block<3,1>(3,i) = z_axis;
        }
    }

    return J_;
}

} // namespace kinematics
} // namespace rnea_control