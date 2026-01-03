#include "rnea_control/rnea_controller.hpp"
#include <sys/mman.h>
#include <pthread.h>

namespace rnea_control {

controller_interface::CallbackReturn rneacontroller::on_init() {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rneacontroller::on_configure(const rclcpp_lifecycle::State &) {
    auto node = get_node();

    // 1. Lấy tên Joint
    if (!node->get_parameter("joints", joint_names_)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get joints");
        return controller_interface::CallbackReturn::ERROR;
    }
    num_joints_ = joint_names_.size();

    // 2. Load Gains (Kp, Kd) từ YAML
    kp_.resize(num_joints_);
    kd_.resize(num_joints_);
    
    for (size_t i = 0; i < num_joints_; ++i) {
        // Cấu trúc: gains.joint_1.p
        std::string p_gain_param = "gains." + joint_names_[i] + ".p";
        std::string d_gain_param = "gains." + joint_names_[i] + ".d";
        
        if (!node->has_parameter(p_gain_param)) node->declare_parameter(p_gain_param, 500.0);
        if (!node->has_parameter(d_gain_param)) node->declare_parameter(d_gain_param, 10.0);
        
        kp_[i] = node->get_parameter(p_gain_param).as_double();
        kd_[i] = node->get_parameter(d_gain_param).as_double();
        
        RCLCPP_INFO(node->get_logger(), "Joint %s Gains: P=%.2f, D=%.2f", joint_names_[i].c_str(), kp_[i], kd_[i]);
    }

    // 3. Load thông số động lực học (Code cũ của bạn, nhớ giữ logic check type Prismatic/Revolute)
    std::vector<LinkParams> robot_links;
    for (size_t i = 0; i < num_joints_; ++i) {
        std::string prefix = "robot_description_parameters.link" + std::to_string(i+1) + ".";
        LinkParams lp;
        
        // --- XỬ LÝ TYPE (Quan trọng cho Prismatic) ---
        std::string param_type = prefix + "type";
        if (!node->has_parameter(param_type)) node->declare_parameter(param_type, "revolute");
        std::string type_str = node->get_parameter(param_type).as_string();
        lp.type = (type_str == "prismatic") ? rnea_control::PRISMATIC : rnea_control::REVOLUTE;

        // --- XỬ LÝ MASS, COM, P, I (Dùng has_parameter như đã sửa) ---
        // (Copy lại đoạn code "has_parameter" từ bài trước vào đây)
        std::string p_mass = prefix + "mass";
        if (!node->has_parameter(p_mass)) node->declare_parameter(p_mass, 0.0);
        lp.m = node->get_parameter(p_mass).as_double();
        
        // ... (Làm tương tự cho com, p, inertia) ...
        // Tôi lược bớt đoạn này cho gọn, bạn nhớ giữ nguyên code cũ
        
        // Giả lập dữ liệu nếu chưa copy đoạn cũ:
        if (!node->has_parameter(prefix+"com")) node->declare_parameter(prefix+"com", std::vector<double>{0,0,0});
        auto com_v = node->get_parameter(prefix+"com").as_double_array();
        lp.com = Eigen::Vector3d(com_v[0], com_v[1], com_v[2]);

        if (!node->has_parameter(prefix+"p")) node->declare_parameter(prefix+"p", std::vector<double>{0,0,0});
        auto p_v = node->get_parameter(prefix+"p").as_double_array();
        lp.p = Eigen::Vector3d(p_v[0], p_v[1], p_v[2]);

        if (!node->has_parameter(prefix+"inertia")) node->declare_parameter(prefix+"inertia", std::vector<double>(9, 0.0));
        auto i_v = node->get_parameter(prefix+"inertia").as_double_array();
        lp.I = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(i_v.data());

        robot_links.push_back(lp);
    }

    solver_ = std::make_unique<rnea_control::rneasolver>(robot_links);

    // 4. Resize Vectors
    q_.resize(num_joints_);
    dq_.resize(num_joints_);
    q_des_.resize(num_joints_);
    dq_des_.resize(num_joints_, 0.0); // Mặc định vận tốc mong muốn = 0
    ddq_des_.resize(num_joints_, 0.0); // Mặc định gia tốc mong muốn = 0
    ddq_ref_.resize(num_joints_);
    tau_cmd_.resize(num_joints_);

    command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/commands", rclcpp::SystemDefaultsQoS(),
        std::bind(&rneacontroller::command_callback, this, std::placeholders::_1));

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rneacontroller::on_activate(const rclcpp_lifecycle::State &) {
    // KHI BẮT ĐẦU: Đặt vị trí mong muốn = Vị trí hiện tại
    // Để robot không bị giật (Jump) khi mới bật controller
    for (size_t i = 0; i < num_joints_; ++i) {
        q_des_[i] = state_interfaces_[i*2].get_value();
        dq_des_[i] = 0.0;
        ddq_des_[i] = 0.0;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

// Callback nhận lệnh: Bây giờ nhận VỊ TRÍ MONG MUỐN (Target Position)
void rneacontroller::command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != num_joints_) {
        RCLCPP_ERROR(get_node()->get_logger(), "Command size mismatch!");
        return;
    }
    for (size_t i = 0; i < num_joints_; ++i) {
        q_des_[i] = msg->data[i];
        // Reset vận tốc/gia tốc mong muốn về 0 khi set điểm mới (Step input)
        // Nếu muốn trajectory mượt hơn, bạn cần topic gửi cả q, dq, ddq
        dq_des_[i] = 0.0;
        ddq_des_[i] = 0.0;
    }
}

controller_interface::return_type rneacontroller::update(const rclcpp::Time &, const rclcpp::Duration &) {
    // 1. Đọc trạng thái hiện tại
    for (size_t i = 0; i < num_joints_; ++i) {
        q_[i] = state_interfaces_[i*2].get_value();
        dq_[i] = state_interfaces_[i*2 + 1].get_value();
    }

    // 2. Tính toán COMPUTED TORQUE CONTROL (CTC)
    for (size_t i = 0; i < num_joints_; ++i) {
        // Sai số vị trí và vận tốc
        double error_q = q_des_[i] - q_[i];
        double error_dq = dq_des_[i] - dq_[i];

        // Công thức PD: Tính gia tốc tham chiếu cần thiết để sửa lỗi
        // ddq_ref = ddq_des + Kp*e + Kd*de
        ddq_ref_[i] = ddq_des_[i] + kp_[i] * error_q + kd_[i] * error_dq;
    }

    // 3. Đưa gia tốc tham chiếu vào RNEA Solver (Inverse Dynamics)
    // Solver sẽ tính ra lực cần thiết để đạt được gia tốc đó + bù trọng lực
    solver_->solve(q_, dq_, ddq_ref_, tau_cmd_);

    // 4. Gửi Torque xuống Robot
    for (size_t i = 0; i < num_joints_; ++i) {
        // Có thể thêm Saturate (Giới hạn lực) ở đây để an toàn
        // if (tau_cmd_[i] > 100) tau_cmd_[i] = 100;
        command_interfaces_[i].set_value(tau_cmd_[i]);
    }

    return controller_interface::return_type::OK;
}

// ... (Giữ nguyên 2 hàm interface configuration) ...
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

} // namespace
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rnea_control::rneacontroller, controller_interface::ControllerInterface)