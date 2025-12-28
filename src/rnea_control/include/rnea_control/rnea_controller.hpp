#ifndef RNEA_CONTROL__RNEA_CONTROLLER_HPP_
#define RNEA_CONTROL__RNEA_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "rnea_control/rnea_solver.hpp" // Đảm bảo đường dẫn này đúng với thực tế
#include "rclcpp/rclcpp.hpp"
#include <memory>

namespace rnea_control {

class rneacontroller : public controller_interface::ControllerInterface {
public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

private:
    // Sử dụng tên đầy đủ hoặc đảm bảo RNEASolver đã nằm trong namespace rnea_control
    std::unique_ptr<rnea_control::rneasolver> solver_;
    
    Eigen::VectorXd q_, dq_, ddq_des_, tau_cmd_;
    std::vector<std::string> joint_names_;
    size_t num_joints_;
};

} // namespace rnea_control
#endif