#include "rnea_control/trajectory/trajectory_generator.hpp"
#include <algorithm> // for std::max

namespace rnea_control {
namespace trajectory {

TrajectoryGenerator::TrajectoryGenerator() {
    reset();
}

void TrajectoryGenerator::reset() {
    is_moving_ = false;
    q_start_ = 0.0;
    q_end_ = 0.0;
    duration_ = 0.0;
}

void TrajectoryGenerator::set_target(double current_q, double target_q, double duration) {
    q_start_ = current_q;
    q_end_ = target_q;
    
    // Đảm bảo thời gian không quá nhỏ để tránh chia cho 0
    duration_ = std::max(duration, 1e-4);
    
    is_moving_ = true;
}

core::JointTrajectoryPoint TrajectoryGenerator::get_sample(double t) {
    core::JointTrajectoryPoint point;

    // 1. Nếu đã hết thời gian -> Giữ nguyên tại đích
    if (t >= duration_) {
        point.q = q_end_;
        point.dq = 0.0;
        point.ddq = 0.0;
        is_moving_ = false;
        return point;
    }

    // 2. Tính toán Quintic Polynomial (Đa thức bậc 5)
    // q(tau) = q0 + h * (10*tau^3 - 15*tau^4 + 6*tau^5)
    
    // Chuẩn hóa thời gian về [0, 1]
    double T = duration_;
    double tau = t / T;
    double h = q_end_ - q_start_;

    double tau2 = tau * tau;
    double tau3 = tau2 * tau;
    double tau4 = tau3 * tau;
    double tau5 = tau4 * tau;

    // Vị trí
    point.q = q_start_ + h * (10 * tau3 - 15 * tau4 + 6 * tau5);

    // Vận tốc: dq/dt = (dq/dtau) * (1/T)
    // s'(tau) = 30*tau^2 - 60*tau^3 + 30*tau^4
    point.dq = (h / T) * (30 * tau2 - 60 * tau3 + 30 * tau4);

    // Gia tốc: ddq/dt = (ddq/dtau) * (1/T^2)
    // s''(tau) = 60*tau - 180*tau^2 + 120*tau^3
    point.ddq = (h / (T * T)) * (60 * tau - 180 * tau2 + 120 * tau3);

    return point;
}

} // namespace trajectory
} // namespace rnea_control