#include "rnea_control/rnea_controller.hpp"
#include <sys/mman.h>
#include <pthread.h>

namespace rnea_control {

controller_interface::CallbackReturn rneacontroller::on_init() {
    // Để trống hoặc khởi tạo các giá trị mặc định nếu cần
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rneacontroller::on_configure(const rclcpp_lifecycle::State &) {
    auto node = get_node();

    if (!node->get_parameter("joints", joint_names_)) {
        RCLCPP_ERROR(node->get_logger(), "Không tìm thấy tham số 'joints'!");
        return controller_interface::CallbackReturn::ERROR;
    }
    num_joints_ = joint_names_.size();

    std::vector<LinkParams> robot_links;
    for (size_t i = 0; i < num_joints_; ++i) {
        std::string prefix = "robot_description_parameters.link" + std::to_string(i+1) + ".";
        LinkParams lp;
        
        node->declare_parameter(prefix + "mass", 0.0);
        node->declare_parameter(prefix + "com", std::vector<double>{0,0,0});
        node->declare_parameter(prefix + "p", std::vector<double>{0,0,0});
        node->declare_parameter(prefix + "inertia", std::vector<double>(9, 0.0));

        lp.m = node->get_parameter(prefix + "mass").as_double();
        auto com_v = node->get_parameter(prefix + "com").as_double_array();
        lp.com = Eigen::Vector3d(com_v[0], com_v[1], com_v[2]);
        auto p_v = node->get_parameter(prefix + "p").as_double_array();
        lp.p = Eigen::Vector3d(p_v[0], p_v[1], p_v[2]);
        auto i_v = node->get_parameter(prefix + "inertia").as_double_array();
        lp.I = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(i_v.data());

        robot_links.push_back(lp);
    }

    solver_ = std::make_unique<rnea_control::rneasolver>(robot_links);

    q_.resize(num_joints_);
    dq_.resize(num_joints_);
    ddq_des_.resize(num_joints_);
    tau_cmd_.resize(num_joints_);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        RCLCPP_ERROR(node->get_logger(), "mlockall failed!");
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rneacontroller::on_activate(const rclcpp_lifecycle::State &) {
    struct sched_param param;
    param.sched_priority = 99;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        RCLCPP_WARN(get_node()->get_logger(), "Không thể đặt ưu tiên SCHED_FIFO.");
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type rneacontroller::update(const rclcpp::Time &, const rclcpp::Duration &) {
    for (size_t i = 0; i < num_joints_; ++i) {
        q_[i] = state_interfaces_[i*2].get_value();
        dq_[i] = state_interfaces_[i*2 + 1].get_value();
    }

    ddq_des_.setZero();
    solver_->solve(q_, dq_, ddq_des_, tau_cmd_);

    for (size_t i = 0; i < num_joints_; ++i) {
        command_interfaces_[i].set_value(tau_cmd_[i]);
    }

    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration rneacontroller::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_) {
        config.names.push_back(joint + "/effort");
    }
    return config;
}

controller_interface::InterfaceConfiguration rneacontroller::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_) {
        config.names.push_back(joint + "/position");
        config.names.push_back(joint + "/velocity");
    }
    return config;
}

} // namespace rnea_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rnea_control::rneacontroller, controller_interface::ControllerInterface)