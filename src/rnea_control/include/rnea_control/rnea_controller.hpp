#ifndef RNEA_CONTROL__RNEA_CONTROLLER_HPP_
#define RNEA_CONTROL__RNEA_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map> // Thêm map để lưu gain

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rnea_control/rneasolver.hpp"

namespace rnea_control
{

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
  std::unique_ptr<rnea_control::rneasolver> solver_;

  // State vectors
  std::vector<double> q_;         // Góc hiện tại
  std::vector<double> dq_;        // Vận tốc hiện tại
  
  // Desired vectors (Mục tiêu)
  std::vector<double> q_des_;     // Góc mong muốn (Target Position)
  std::vector<double> dq_des_;    // Vận tốc mong muốn (Target Velocity)
  std::vector<double> ddq_des_;   // Gia tốc mong muốn (Feedforward Accel)
  
  // Reference vector (Input cho RNEA sau khi cộng PD)
  std::vector<double> ddq_ref_;   
  
  // Output Torque
  std::vector<double> tau_cmd_;

  // Gains (Hệ số điều khiển)
  std::vector<double> kp_;
  std::vector<double> kd_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscriber_;
  
  // Hàm callback
  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace rnea_control

#endif