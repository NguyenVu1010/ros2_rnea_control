#ifndef RNEA_CONTROL_CONTROL_MOTION_CONTROL_HPP
#define RNEA_CONTROL_CONTROL_MOTION_CONTROL_HPP

#include "rnea_control/core/types.hpp"
#include <Eigen/Dense>
#include <vector>

namespace rnea_control {
namespace control {

class MotionControl {
public:
    MotionControl();

    /**
     * @brief Cài đặt hệ số khuếch đại (Gains)
     * @param kp_pos Hệ số P cho vị trí (x, y, z)
     * @param kp_ori Hệ số P cho hướng (rotation)
     * @param kp_null Hệ số P cho Null-space (nếu robot dư dẫn động)
     */
    void set_gains(double kp_pos, double kp_ori, double kp_null = 1.0);

    /**
     * @brief Tính toán bám quỹ đạo Cartesian (CLIK)
     * 
     * @param desired Trạng thái mong muốn (từ Trajectory Generator)
     * @param current Trạng thái hiện tại (từ FK)
     * @param J Ma trận Jacobian hiện tại
     * @param dq_curr Vận tốc khớp hiện tại
     * @param ddq_cmd_out (Output) Gia tốc khớp cần thiết cho RNEA
     */
    void compute_cartesian_tracking(
        const core::CartesianState& desired,
        const core::CartesianState& current,
        const Eigen::MatrixXd& J,
        const std::vector<double>& dq_curr,
        std::vector<double>& ddq_cmd_out
    );

    /**
     * @brief Tính toán bám quỹ đạo Khớp (Joint Space PD)
     * 
     * @param q_des, dq_des, ddq_des Trạng thái khớp mong muốn
     * @param q_curr, dq_curr Trạng thái khớp hiện tại
     * @param kp, kd Vector hệ số P, D cho từng khớp
     * @param ddq_cmd_out (Output) Gia tốc khớp cần thiết cho RNEA
     */
    void compute_joint_tracking(
        const std::vector<double>& q_des,
        const std::vector<double>& dq_des,
        const std::vector<double>& ddq_des,
        const std::vector<double>& q_curr,
        const std::vector<double>& dq_curr,
        const std::vector<double>& kp,
        const std::vector<double>& kd,
        std::vector<double>& ddq_cmd_out
    );

private:
    double kp_pos_;  // Gain vị trí Cartesian
    double kp_ori_;  // Gain hướng Cartesian
    double kp_null_; // Gain Null-space damping
};

} // namespace control
} // namespace rnea_control

#endif // RNEA_CONTROL_CONTROL_MOTION_CONTROL_HPP