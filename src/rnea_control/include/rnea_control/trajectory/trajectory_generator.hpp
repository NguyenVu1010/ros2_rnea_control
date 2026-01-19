#ifndef RNEA_CONTROL_TRAJECTORY_TRAJECTORY_GENERATOR_HPP
#define RNEA_CONTROL_TRAJECTORY_TRAJECTORY_GENERATOR_HPP

#include "rnea_control/core/types.hpp"
#include <cmath>

namespace rnea_control {
namespace trajectory {

class TrajectoryGenerator {
public:
    TrajectoryGenerator();

    /**
     * @brief Cài đặt mục tiêu di chuyển cho 1 khớp
     * @param current_q Vị trí hiện tại
     * @param target_q Vị trí đích
     * @param duration Thời gian di chuyển (giây)
     */
    void set_target(double current_q, double target_q, double duration);

    /**
     * @brief Tính toán trạng thái tại thời điểm t
     * @param time_from_start Thời gian trôi qua (giây)
     * @return JointTrajectoryPoint (q, dq, ddq)
     */
    core::JointTrajectoryPoint get_sample(double time_from_start);

    bool is_moving() const { return is_moving_; }
    void reset();

private:
    double q_start_;
    double q_end_;
    double duration_;
    bool is_moving_;
};

} // namespace trajectory
} // namespace rnea_control

#endif // RNEA_CONTROL_TRAJECTORY_TRAJECTORY_GENERATOR_HPP