#ifndef RNEA_CONTROL_TRAJECTORY_CARTESIAN_INTERPOLATOR_HPP
#define RNEA_CONTROL_TRAJECTORY_CARTESIAN_INTERPOLATOR_HPP

#include "rnea_control/core/types.hpp"
#include <Eigen/Dense>

namespace rnea_control {
namespace trajectory {

class CartesianInterpolator {
public:
    CartesianInterpolator();

    /**
     * @brief Thiết lập mục tiêu di chuyển (Position + Orientation)
     * 
     * @param start Trạng thái bắt đầu (Vị trí + Góc)
     * @param end Trạng thái đích
     * @param max_lin_vel Vận tốc tuyến tính tối đa (m/s)
     * @param max_lin_acc Gia tốc tuyến tính tối đa (m/s^2)
     * @return true nếu cài đặt thành công, false nếu quãng đường quá ngắn
     */
    bool set_target(const core::CartesianState& start, 
                    const core::CartesianState& end,
                    double max_lin_vel, double max_lin_acc);

    /**
     * @brief Lấy mẫu trạng thái tại thời điểm t
     * @param t Thời gian trôi qua kể từ lúc bắt đầu (giây)
     * @return CartesianState (p, v, a, q) mong muốn
     */
    core::CartesianState get_sample(double t);

    // Kiểm tra xem quỹ đạo đã kết thúc chưa
    bool is_moving() const { return is_moving_; }

    // Reset về trạng thái nghỉ
    void reset();

private:
    core::CartesianState start_, end_;
    
    // --- Linear Path Parameters (LSPB) ---
    Eigen::Vector3d u_vec_; // Vector hướng đơn vị (Unit vector)
    double total_dist_;     // Tổng quãng đường (m)
    
    double a_limit_;        // Gia tốc giới hạn thực tế
    double v_limit_;        // Vận tốc đỉnh thực tế
    
    // Các mốc thời gian
    double t_acc_;          // Thời điểm kết thúc tăng tốc
    double t_flat_;         // Khoảng thời gian đi đều
    double t_dec_;          // Thời điểm bắt đầu giảm tốc
    double tf_;             // Tổng thời gian di chuyển (Time Final)

    bool is_moving_ = false;
};

} // namespace trajectory
} // namespace rnea_control

#endif // RNEA_CONTROL_TRAJECTORY_CARTESIAN_INTERPOLATOR_HPP