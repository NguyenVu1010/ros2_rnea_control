#ifndef RNEA_CONTROL__TRAJECTORY_GENERATOR_HPP_
#define RNEA_CONTROL__TRAJECTORY_GENERATOR_HPP_

#include <cmath>
#include <algorithm>

namespace rnea_control {

// Cấu trúc chứa giá trị đầu ra tại một thời điểm
struct TrajectoryPoint {
    double q = 0.0;   // Vị trí
    double dq = 0.0;  // Vận tốc
    double ddq = 0.0; // Gia tốc
};

class TrajectoryGenerator {
public:
    TrajectoryGenerator() {
        reset();
    }

    // Cài đặt mục tiêu mới (từ vị trí hiện tại đến đích trong thời gian T)
    void set_target(double current_q, double target_q, double duration) {
        q_start_ = current_q;
        q_end_ = target_q;
        duration_ = duration;
        
        // Tránh chia cho 0
        if (duration_ <= 1e-6) duration_ = 1e-6; 
        
        is_moving_ = true;
    }

    // Tính toán trạng thái tại thời điểm 'time_from_start'
    // Input: Thời gian trôi qua kể từ khi bắt đầu lệnh (giây)
    TrajectoryPoint get_sample(double time_from_start) {
        TrajectoryPoint point;

        // 1. Nếu đã hết thời gian -> Giữ nguyên tại đích
        if (time_from_start >= duration_) {
            point.q = q_end_;
            point.dq = 0.0;
            point.ddq = 0.0;
            is_moving_ = false;
            return point;
        }

        // 2. Tính toán Quintic Polynomial (Đa thức bậc 5)
        // Đảm bảo vị trí, vận tốc, gia tốc đều mượt tại điểm đầu và cuối
        
        // Chuẩn hóa thời gian về [0, 1]
        double t = time_from_start;
        double T = duration_;
        double tau = t / T;
        
        double h = q_end_ - q_start_;

        // Các hệ số của đa thức bậc 5: s(tau) = 10*t^3 - 15*t^4 + 6*t^5
        double tau2 = tau * tau;
        double tau3 = tau2 * tau;
        double tau4 = tau3 * tau;
        double tau5 = tau4 * tau;

        // Vị trí: q(t)
        point.q = q_start_ + h * (10 * tau3 - 15 * tau4 + 6 * tau5);

        // Vận tốc: dq(t) = h/T * s'(tau)
        point.dq = (h / T) * (30 * tau2 - 60 * tau3 + 30 * tau4);

        // Gia tốc: ddq(t) = h/T^2 * s''(tau)
        point.ddq = (h / (T * T)) * (60 * tau - 180 * tau2 + 120 * tau3);

        return point;
    }

    bool is_moving() const {
        return is_moving_;
    }

    void reset() {
        is_moving_ = false;
        q_start_ = 0.0;
        q_end_ = 0.0;
        duration_ = 0.0;
    }

private:
    double q_start_;
    double q_end_;
    double duration_;
    bool is_moving_;
};

} // namespace rnea_control

#endif