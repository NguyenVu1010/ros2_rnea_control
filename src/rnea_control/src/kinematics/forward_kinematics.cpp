#include "rnea_control/kinematics/forward_kinematics.hpp"
#include "rnea_control/core/math_utils.hpp"

namespace rnea_control {
namespace kinematics {

ForwardKinematics::ForwardKinematics(const std::vector<core::LinkParams>& params)
    : params_(params), n_(params.size()) {
    
    // T_global có n+1 phần tử (Frame 0 -> Frame n)
    // Khởi tạo tất cả là ma trận đơn vị
    T_global_.resize(n_ + 1, Eigen::Matrix4d::Identity());
    
    // Khởi tạo last_q rỗng để ép buộc tính toán lần đầu
    last_q_.clear();
}

const std::vector<Eigen::Matrix4d>& ForwardKinematics::update(const std::vector<double>& q) {
    // 1. Kiểm tra Cache: Nếu q chưa đổi thì trả về kết quả cũ ngay lập tức
    if (!last_q_.empty() && q == last_q_) {
        return T_global_;
    }
    
    // 2. Cập nhật q mới
    last_q_ = q;

    // 3. Tính toán chuỗi động học (Standard DH)
    // T_global[0] luôn là Identity (Base Frame)
    
    for (size_t i = 0; i < n_; ++i) {
        double d = params_[i].d;
        double theta = params_[i].theta;

        // Cộng biến khớp vào tham số DH
        if (params_[i].type == core::PRISMATIC) {
            d += q[i];
        } else {
            theta += q[i];
        }

        // Tính ma trận chuyển đổi từ Frame i-1 sang Frame i
        Eigen::Matrix4d T_rel = core::MathUtils::get_dh_matrix(
            params_[i].a, params_[i].alpha, d, theta
        );

        // Nhân chuỗi: T_0_i = T_0_(i-1) * T_(i-1)_i
        // .noalias() giúp Eigen tối ưu hóa phép nhân, không tạo biến tạm
        T_global_[i+1].noalias() = T_global_[i] * T_rel;
    }

    return T_global_;
}

Eigen::Vector3d ForwardKinematics::get_ee_position() const {
    // Lấy phần tịnh tiến (cột 4, 3 dòng đầu) của ma trận cuối cùng
    return T_global_[n_].block<3,1>(0,3);
}

Eigen::Quaterniond ForwardKinematics::get_ee_orientation() const {
    // Lấy phần xoay (3x3 góc trên trái) của ma trận cuối cùng
    Eigen::Matrix3d rot = T_global_[n_].block<3,3>(0,0);
    return Eigen::Quaterniond(rot);
}

} // namespace kinematics
} // namespace rnea_control