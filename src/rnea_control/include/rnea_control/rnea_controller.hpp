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

// --- CÁC MODULE CON ---
#include "rnea_control/core/types.hpp"
#include "rnea_control/dynamics/rnea_solver.hpp"
#include "rnea_control/kinematics/forward_kinematics.hpp"
#include "rnea_control/kinematics/jacobian.hpp"
#include "rnea_control/control/motion_control.hpp"
#include "rnea_control/trajectory/cartesian_interpolator.hpp"

// !!! QUAN TRỌNG: THÊM DÒNG NÀY ĐỂ SỬA LỖI TRAJECTORY GENERATOR !!!
#include "rnea_control/trajectory/trajectory_generator.hpp" 

namespace rnea_control
{

enum ControlMode {
    MODE_IDLE,
    MODE_JOINT_TRAJECTORY,
    MODE_CARTESIAN_TRAJECTORY
};

class rneacontroller : public controller_interface::ControllerInterface
{
public:
  rneacontroller() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  std::vector<std::string> joint_names_;
  size_t num_joints_;

  // --- MODULES ---
  std::unique_ptr<dynamics::RNEASolver> solver_;
  std::unique_ptr<kinematics::ForwardKinematics> kin_solver_;
  std::unique_ptr<kinematics::JacobianSolver> jac_solver_;
  
  // Khai báo vector chứa các bộ sinh quỹ đạo khớp
  std::vector<trajectory::TrajectoryGenerator> traj_generators_; 
  
  trajectory::CartesianInterpolator cart_gen_;
  std::unique_ptr<control::MotionControl> motion_control_;

  // --- VECTORS ---
  std::vector<double> q_;
  std::vector<double> dq_;
  std::vector<double> q_des_;
  std::vector<double> dq_des_;
  std::vector<double> ddq_des_;
  std::vector<double> ddq_ref_;
  std::vector<double> tau_cmd_;
  std::vector<double> dq_cmd_prev_;

  // --- GAINS ---
  std::vector<double> kp_;
  std::vector<double> kd_;

  // --- LOGIC ---
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscriber_;
  
  bool has_pending_command_ = false;
  std_msgs::msg::Float64MultiArray::SharedPtr pending_command_;
  bool is_first_update_ = true;
  rclcpp::Time motion_start_time_;
  
  ControlMode control_mode_ = MODE_IDLE;

  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace rnea_control

#endif