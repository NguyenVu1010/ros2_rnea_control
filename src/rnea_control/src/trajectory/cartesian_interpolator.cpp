#include "rnea_control/trajectory/cartesian_interpolator.hpp"
#include <cmath>
#include <iostream>

namespace rnea_control {
namespace trajectory {

CartesianInterpolator::CartesianInterpolator() {
    reset();
}

void CartesianInterpolator::reset() {
    is_moving_ = false;
    start_ = core::CartesianState();
    end_ = core::CartesianState();
    u_vec_.setZero();
    total_dist_ = 0.0;
    tf_ = 0.0;
}

bool CartesianInterpolator::set_target(const core::CartesianState& start, 
                                       const core::CartesianState& end,
                                       double max_lin_vel, double max_lin_acc) {
    start_ = start;
    end_ = end;

    // 1. Tính vector chỉ phương và khoảng cách
    Eigen::Vector3d diff = end.p - start.p;
    total_dist_ = diff.norm();

    // Nếu khoảng cách quá nhỏ -> Coi như đã đến nơi
    if (total_dist_ < 1e-6) {
        reset();
        return false;
    }

    u_vec_ = diff / total_dist_; // Normalize

    // 2. Tính toán Profile LSPB (Linear Segment with Parabolic Blend)
    
    // Thời gian cần để tăng tốc từ 0 lên max_vel: t = v/a
    double t_ramp = max_lin_vel / max_lin_acc;
    
    // Quãng đường đi được trong pha tăng tốc: s = 0.5 * a * t^2
    double dist_ramp = 0.5 * max_lin_acc * t_ramp * t_ramp;

    // Kiểm tra xem tổng đường đi có đủ cho 2 pha (tăng + giảm) không
    if (total_dist_ < 2.0 * dist_ramp) {
        // --- TRƯỜNG HỢP 1: Profile TAM GIÁC (Không đạt được max_vel) ---
        // total_dist = 2 * (0.5 * a * t_acc^2) = a * t_acc^2
        // -> t_acc = sqrt(total / a)
        t_acc_ = std::sqrt(total_dist_ / max_lin_acc);
        t_flat_ = 0.0;
        
        // Tính lại vận tốc đỉnh thực tế
        v_limit_ = max_lin_acc * t_acc_;
    } else {
        // --- TRƯỜNG HỢP 2: Profile HÌNH THANG (Đạt max_vel) ---
        t_acc_ = t_ramp;
        v_limit_ = max_lin_vel;
        
        // Quãng đường đi đều
        double dist_flat = total_dist_ - 2.0 * dist_ramp;
        t_flat_ = dist_flat / v_limit_;
    }

    a_limit_ = max_lin_acc;
    
    // Các mốc thời gian
    t_dec_ = t_acc_ + t_flat_;      // Lúc bắt đầu giảm tốc
    tf_ = t_acc_ + t_flat_ + t_acc_; // Tổng thời gian

    is_moving_ = true;
    return true;
}

core::CartesianState CartesianInterpolator::get_sample(double t) {
    core::CartesianState state;

    // Nếu đã hết thời gian -> Trả về đích
    if (!is_moving_ || t >= tf_) {
        state = end_;
        state.v_lin.setZero();
        state.a_lin.setZero();
        state.v_ang.setZero();
        state.a_ang.setZero();
        is_moving_ = false;
        return state;
    }

    // --- A. TÍNH TOÁN VỊ TRÍ (LINEAR) ---
    double s = 0.0;   // Quãng đường scalar
    double v = 0.0;   // Vận tốc scalar
    double a = 0.0;   // Gia tốc scalar

    if (t < t_acc_) {
        // Pha Tăng tốc
        a = a_limit_;
        v = a * t;
        s = 0.5 * a * t * t;
    } else if (t < t_dec_) {
        // Pha Đi đều
        a = 0.0;
        v = v_limit_;
        double s_acc = 0.5 * a_limit_ * t_acc_ * t_acc_;
        double t_in_flat = t - t_acc_;
        s = s_acc + v * t_in_flat;
    } else {
        // Pha Giảm tốc
        double dt = tf_ - t; // Thời gian còn lại
        a = -a_limit_;
        v = a_limit_ * dt;   // Vận tốc giảm dần về 0
        
        double s_dec = 0.5 * a_limit_ * dt * dt; // Quãng đường còn lại
        s = total_dist_ - s_dec;
    }

    // Mapping sang Vector 3D
    state.p = start_.p + s * u_vec_;
    state.v_lin = v * u_vec_;
    state.a_lin = a * u_vec_;

    // --- B. TÍNH TOÁN GÓC (ORIENTATION) ---
    // Sử dụng SLERP (Spherical Linear Interpolation)
    // Giả sử góc quay đều trong suốt thời gian tf_
    double ratio = t / tf_;
    if (ratio > 1.0) ratio = 1.0;
    
    // Nội suy Quaternion
    state.q = start_.q.slerp(ratio, end_.q);

    // Tính vận tốc góc (Angular Velocity) - Xấp xỉ đơn giản
    // Để chính xác, cần dùng công thức: w = 2 * dq/dt * inv(q)
    // Ở đây ta có thể để 0 nếu chỉ cần giữ hướng, hoặc tính sai số P ở tầng điều khiển.
    // Với CLIK cơ bản, ta thường dùng sai số hướng (Orientation Error) trực tiếp trong vòng lặp điều khiển
    // nên feed-forward vận tốc góc có thể để 0 nếu chuyển động chậm.
    state.v_ang.setZero();
    state.a_ang.setZero();

    return state;
}

} // namespace trajectory
} // namespace rnea_control