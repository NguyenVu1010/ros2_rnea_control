#ifndef RNEA_CONTROL_KINEMATICS_JACOBIAN_HPP
#define RNEA_CONTROL_KINEMATICS_JACOBIAN_HPP

#include "rnea_control/core/types.hpp"
#include <vector>
#include <Eigen/Dense>

namespace rnea_control {
namespace kinematics {

class JacobianSolver {
public:
    explicit JacobianSolver(const std::vector<core::LinkParams>& params);

    /**
     * @brief Tính Jacobian Hình học (Geometric Jacobian)
     * @param transforms Chuỗi ma trận T_global (Lấy từ ForwardKinematics::update)
     * @return Tham chiếu hằng đến Ma trận Jacobian (6xn)
     */
    const Eigen::MatrixXd& compute(const std::vector<Eigen::Matrix4d>& transforms);

private:
    std::vector<core::LinkParams> params_;
    size_t n_;
    
    // Cache Jacobian để không cấp phát bộ nhớ trong vòng lặp
    Eigen::MatrixXd J_;
};

} // namespace kinematics
} // namespace rnea_control
#endif