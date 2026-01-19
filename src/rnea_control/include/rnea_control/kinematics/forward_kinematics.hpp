#ifndef RNEA_CONTROL_KINEMATICS_FORWARD_KINEMATICS_HPP
#define RNEA_CONTROL_KINEMATICS_FORWARD_KINEMATICS_HPP

#include "rnea_control/core/types.hpp"
#include <vector>
#include <Eigen/Dense>

namespace rnea_control {
namespace kinematics {

class ForwardKinematics {
public:
    explicit ForwardKinematics(const std::vector<core::LinkParams>& params);

    /**
     * @brief Cập nhật trạng thái động học thuận
     * @param q Vector vị trí khớp (rad hoặc m)
     * @return Tham chiếu hằng đến chuỗi ma trận biến đổi toàn cục
     */
    const std::vector<Eigen::Matrix4d>& update(const std::vector<double>& q);

    // Lấy vị trí End-Effector (sau khi update)
    Eigen::Vector3d get_ee_position() const;

    // Lấy hướng End-Effector dạng Quaternion (sau khi update)
    Eigen::Quaterniond get_ee_orientation() const;

    // Lấy chuỗi transforms hiện tại (để Jacobian dùng lại)
    const std::vector<Eigen::Matrix4d>& get_transforms() const { return T_global_; }

private:
    std::vector<core::LinkParams> params_;
    size_t n_;

    // Cache variables (Tránh cấp phát động liên tục)
    std::vector<Eigen::Matrix4d> T_global_; // Lưu T_0_0, T_0_1, ..., T_0_n
    std::vector<double> last_q_;            // Lưu q của lần tính trước để so sánh
};

} // namespace kinematics
} // namespace rnea_control
#endif