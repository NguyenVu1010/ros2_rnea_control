#ifndef RNEA_CONTROL__RNEA_CONTROLLER_HPP_
#define RNEA_CONTROL__RNEA_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// INCLUDE CÁC MODULE ĐÃ VIẾT
#include "rnea_control/rneasolver.hpp"
#include "rnea_control/trajectory_generator.hpp"
#include "rnea_control/kinematics_solver.hpp"   
#include "rnea_control/cartesian_generator.hpp" 

namespace rnea_control
{

// Enum để phân biệt chế độ điều khiển
enum ControlMode {
    MODE_IDLE,
    MODE_JOINT_TRAJECTORY,    // Nội suy khớp (Spline)
    MODE_CARTESIAN_TRAJECTORY // Nội suy đường thẳng (LSPB + CLIK)
};

class rneacontroller : public controller_interface::ControllerInterface
{
public:
  rneacontroller() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override { return controller_interface::CallbackReturn::SUCCESS; }
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  std::vector<std::string> joint_names_;
  size_t num_joints_;

  // --- SOLVERS ---
  std::unique_ptr<rnea_control::rneasolver> solver_;
  std::unique_ptr<rnea_control::KinematicsSolver> kin_solver_; // <--- Mới: Tính FK, Jacobian
  
  // --- GENERATORS ---
  std::vector<TrajectoryGenerator> traj_generators_; // Cho Joint Space
  rnea_control::CartesianGenerator cart_gen_;        // Cho Cartesian Space (Mới)

  // --- STATE VECTORS ---
  std::vector<double> q_;
  std::vector<double> dq_;
  
  // --- DESIRED VECTORS ---
  std::vector<double> q_des_;
  std::vector<double> dq_des_;
  std::vector<double> ddq_des_;
  std::vector<double> ddq_ref_;
  std::vector<double> tau_cmd_;

  // --- HELPER FOR CLIK ---
  std::vector<double> dq_cmd_prev_; // Lưu vận tốc vòng trước để tính gia tốc

  // --- GAINS ---
  std::vector<double> kp_;
  std::vector<double> kd_;

  // --- CONTROL LOGIC ---
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscriber_;
  rclcpp::Time motion_start_time_;
  ControlMode control_mode_ = MODE_IDLE; // Biến lưu chế độ hiện tại
  
  std_msgs::msg::Float64MultiArray::SharedPtr pending_command_;
  bool has_pending_command_ = false;

  // Cờ đánh dấu lần chạy đầu tiên để khởi tạo thời gian
  bool is_first_update_ = true;

  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace rnea_control

#endif