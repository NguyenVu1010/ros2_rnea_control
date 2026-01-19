#ifndef RNEA_CONTROL_DYNAMICS_RNEA_SOLVER_HPP
#define RNEA_CONTROL_DYNAMICS_RNEA_SOLVER_HPP

#include "rnea_control/core/types.hpp"
#include <vector>
#include <Eigen/Dense>

namespace rnea_control {
namespace dynamics {

class RNEASolver {
public:
    /**
     * @brief Constructor
     * @param params Danh sách thông số động lực học của các Link
     */
    explicit RNEASolver(const std::vector<core::LinkParams>& params);

    /**
     * @brief Giải bài toán Động lực học ngược
     * @param q_in Vị trí khớp
     * @param dq_in Vận tốc khớp
     * @param ddq_in Gia tốc khớp mong muốn
     * @param tau_out (Output) Momen/Lực cần thiết
     */
    void solve(const std::vector<double>& q_in, 
               const std::vector<double>& dq_in, 
               const std::vector<double>& ddq_in, 
               std::vector<double>& tau_out);

private:
    std::vector<core::LinkParams> params_;
    int n_; // Số lượng khớp

    // --- CÁC BIẾN CACHE (Pre-allocated variables) ---
    // w: Vận tốc góc
    // dw: Gia tốc góc
    // dv: Gia tốc tuyến tính
    // f_ext: Lực tương tác giữa các khâu
    // n_ext: Momen tương tác giữa các khâu
    // F_i: Lực quán tính
    // N_i: Momen quán tính
    std::vector<Eigen::Vector3d> w_, dw_, dv_, f_ext_, n_ext_;
    std::vector<Eigen::Vector3d> F_i_, N_i_;
};

} // namespace dynamics
} // namespace rnea_control

#endif // RNEA_CONTROL_DYNAMICS_RNEA_SOLVER_HPP