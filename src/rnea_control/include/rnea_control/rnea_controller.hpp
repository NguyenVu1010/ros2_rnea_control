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

// Include file solver của bạn (nơi định nghĩa class rneasolver và struct LinkParams)
#include "rnea_control/rnea_solver.hpp"

namespace rnea_control
{

class rneacontroller : public controller_interface::ControllerInterface
{
public:
  rneacontroller() = default;

  // Các hàm Lifecycle bắt buộc phải override
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  // Hàm update chạy trong vòng lặp thời gian thực
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Cấu hình Interface
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  // Danh sách tên khớp
  std::vector<std::string> joint_names_;
  size_t num_joints_;

  // Solver RNEA (Sử dụng smart pointer)
  std::unique_ptr<rnea_control::rneasolver> solver_;

  // Các vector lưu trạng thái và tính toán (dùng std::vector hoặc Eigen tùy vào solver của bạn)
  // Ở đây để std::vector double cho khớp với file .cpp
  std::vector<double> q_;         // Góc khớp hiện tại
  std::vector<double> dq_;        // Vận tốc khớp hiện tại
  std::vector<double> ddq_des_;   // Gia tốc mong muốn (Input cho Inverse Dynamics)
  std::vector<double> tau_cmd_;   // Momen tính toán được (Output)

  // --- PHẦN THÊM MỚI CHO SUBSCRIBER ---
  
  // Subscriber nhận lệnh từ topic
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscriber_;
  
  // Buffer lưu lệnh nhận được (tránh race condition đơn giản)
  std::vector<double> commands_;

  // Hàm callback xử lý tin nhắn
  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace rnea_control

#endif  // RNEA_CONTROL__RNEA_CONTROLLER_HPP_