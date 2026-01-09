#ifndef RNEA_CONTROL__CARTESIAN_GENERATOR_HPP_
#define RNEA_CONTROL__CARTESIAN_GENERATOR_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

namespace rnea_control {

// Cấu trúc dữ liệu trả về cho Controller
struct CartesianState {
    Eigen::Vector3d p;   // Vị trí (x, y, z)
    Eigen::Vector3d v;   // Vận tốc (vx, vy, vz)
    Eigen::Vector3d a;   // Gia tốc (ax, ay, az)
    
    // Macro của Eigen để căn chỉnh bộ nhớ (quan trọng cho hiệu suất)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class CartesianGenerator {
public:
    CartesianGenerator() {
        reset();
    }

    /**
     * @brief Thiết lập mục tiêu di chuyển đường thẳng (Linear Motion)
     * Hàm này tính toán sẵn Profile vận tốc hình thang (LSPB).
     * 
     * @param start_pos Vị trí hiện tại
     * @param end_pos Vị trí đích
     * @param max_vel Vận tốc tối đa cho phép (m/s)
     * @param max_acc Gia tốc tối đa cho phép (m/s^2)
     * @return true nếu cài đặt thành công, false nếu quãng đường quá ngắn (=0)
     */
    bool set_target(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos, 
                    double max_vel, double max_acc) {
        
        p_start_ = start_pos;
        p_end_ = end_pos;
        
        // Tính vector chỉ phương di chuyển
        Eigen::Vector3d diff = p_end_ - p_start_;
        total_dist_ = diff.norm();

        // Kiểm tra nếu quãng đường quá ngắn -> Coi như đã đến nơi
        if (total_dist_ < 1e-6) {
            is_moving_ = false;
            u_vec_.setZero();
            return false;
        }

        // Vector đơn vị (Unit Vector) hướng di chuyển
        u_vec_ = diff / total_dist_;

        // --- TÍNH TOÁN PROFILE LSPB (Hình thang vận tốc) ---
        // Về cơ bản, ta giải bài toán 1D cho biến s (quãng đường)
        
        // Thời gian cần thiết để tăng tốc từ 0 lên max_vel
        // t = v / a
        double t_ramp = max_vel / max_acc;
        
        // Quãng đường đi được trong giai đoạn tăng tốc
        // s = 0.5 * a * t^2
        double dist_ramp = 0.5 * max_acc * t_ramp * t_ramp;

        // Kiểm tra xem quãng đường tổng có đủ để đạt max_vel không?
        // Cần 2 lần quãng đường tăng tốc (tăng + giảm)
        if (total_dist_ < 2.0 * dist_ramp) {
            // KHÔNG ĐẠT ĐƯỢC V_MAX -> Profile Hình Tam Giác (Triangle)
            // Tính lại thời gian tăng tốc dựa trên quãng đường
            // total_dist = 2 * (0.5 * a * t_acc^2) = a * t_acc^2
            // -> t_acc = sqrt(total / a)
            t_acc_ = std::sqrt(total_dist_ / max_acc);
            t_flat_ = 0.0; // Không có giai đoạn đi đều
            
            // Vận tốc đỉnh thực tế đạt được (< max_vel)
            v_limit_ = max_acc * t_acc_;
        } else {
            // ĐẠT ĐƯỢC V_MAX -> Profile Hình Thang (Trapezoidal)
            t_acc_ = t_ramp;
            v_limit_ = max_vel;
            
            // Quãng đường đi đều (Constant velocity)
            double dist_flat = total_dist_ - 2.0 * dist_ramp;
            
            // Thời gian đi đều
            t_flat_ = dist_flat / v_limit_;
        }

        a_limit_ = max_acc;
        t_dec_ = t_acc_ + t_flat_;        // Thời điểm bắt đầu giảm tốc
        t_end_ = t_acc_ + t_flat_ + t_acc_; // Thời điểm kết thúc
        
        is_moving_ = true;
        return true;
    }

    /**
     * @brief Lấy trạng thái tại thời điểm t (Real-time safe)
     * @param time_elapsed Thời gian trôi qua kể từ lúc set_target (giây)
     */
    CartesianState get_sample(double t) {
        CartesianState state;

        // 1. Nếu đã kết thúc hoặc chưa kích hoạt -> Trả về đích
        if (!is_moving_ || t >= t_end_) {
            state.p = p_end_;
            state.v.setZero();
            state.a.setZero();
            is_moving_ = false;
            return state;
        }

        double s = 0.0;   // Quãng đường vô hướng
        double v = 0.0;   // Vận tốc vô hướng
        double a = 0.0;   // Gia tốc vô hướng

        // 2. Tính toán trên không gian 1D (Biến s)
        if (t < t_acc_) {
            // --- GIAI ĐOẠN TĂNG TỐC ---
            // a = a_lim
            // v = a * t
            // s = 0.5 * a * t^2
            a = a_limit_;
            v = a_limit_ * t;
            s = 0.5 * a_limit_ * t * t;
        } 
        else if (t < t_dec_) {
            // --- GIAI ĐOẠN ĐI ĐỀU (CRUISE) ---
            // a = 0
            // v = v_lim
            // s = s_acc + v_lim * (t - t_acc)
            double dt = t - t_acc_;
            double s_acc = 0.5 * a_limit_ * t_acc_ * t_acc_;
            
            a = 0.0;
            v = v_limit_;
            s = s_acc + v_limit_ * dt;
        } 
        else {
            // --- GIAI ĐOẠN GIẢM TỐC ---
            // Tính toán ngược từ cuối về cho dễ
            double dt_end = t_end_ - t; // Thời gian còn lại
            
            // a = -a_lim
            // v = a_lim * dt_end (Vận tốc giảm dần về 0)
            // s = Total - (quãng đường còn lại)
            
            a = -a_limit_;
            v = a_limit_ * dt_end;
            double s_remain = 0.5 * a_limit_ * dt_end * dt_end;
            s = total_dist_ - s_remain;
        }

        // 3. Mapping từ 1D sang 3D Vector
        // P(t) = P_start + s * Unit_Vector
        state.p = p_start_ + s * u_vec_;
        state.v = v * u_vec_;
        state.a = a * u_vec_;

        return state;
    }

    bool is_moving() const { return is_moving_; }

    void reset() {
        is_moving_ = false;
        p_start_.setZero();
        p_end_.setZero();
        u_vec_.setZero();
        total_dist_ = 0.0;
        t_acc_ = 0.0;
        t_flat_ = 0.0;
        t_dec_ = 0.0;
        t_end_ = 0.0;
        v_limit_ = 0.0;
        a_limit_ = 0.0;
    }

private:
    // State variables
    bool is_moving_;
    
    // Geometry
    Eigen::Vector3d p_start_;
    Eigen::Vector3d p_end_;
    Eigen::Vector3d u_vec_; // Vector hướng đơn vị
    double total_dist_;

    // Motion Profile Parameters (LSPB)
    double a_limit_;    // Gia tốc giới hạn
    double v_limit_;    // Vận tốc đỉnh thực tế
    
    // Time checkpoints
    double t_acc_;      // Thời điểm kết thúc tăng tốc
    double t_flat_;     // Khoảng thời gian đi đều
    double t_dec_;      // Thời điểm bắt đầu giảm tốc (t_acc + t_flat)
    double t_end_;      // Tổng thời gian di chuyển
};

} // namespace rnea_control

#endif // RNEA_CONTROL__CARTESIAN_GENERATOR_HPP_