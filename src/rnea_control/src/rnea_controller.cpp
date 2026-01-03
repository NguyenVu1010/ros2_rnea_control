#include "rnea_control/rnea_controller.hpp"
#include <sys/mman.h>
#include <pthread.h>

namespace rnea_control {

controller_interface::CallbackReturn rneacontroller::on_init() {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rneacontroller::on_configure(const rclcpp_lifecycle::State &) {
    auto node = get_node();

    // 1. Lấy danh sách joint
    if (!node->get_parameter("joints", joint_names_)) {
        RCLCPP_ERROR(node->get_logger(), "Không tìm thấy tham số 'joints'!");
        return controller_interface::CallbackReturn::ERROR;
    }
    num_joints_ = joint_names_.size();

    // 2. Load thông số động lực học (ĐÃ SỬA LỖI DECLARE)
    std::vector<LinkParams> robot_links;
    for (size_t i = 0; i < num_joints_; ++i) {
        std::string prefix = "robot_description_parameters.link" + std::to_string(i+1) + ".";
        LinkParams lp;
        
        // --- 1. ĐỌC LOẠI KHỚP (TYPE) TỪ YAML ---
        std::string param_type = prefix + "type";
        std::string type_str = "revolute"; // Mặc định là khớp xoay
        
        if (!node->has_parameter(param_type)) {
            node->declare_parameter(param_type, "revolute");
        }
        // Lấy chuỗi từ yaml
        type_str = node->get_parameter(param_type).as_string();

        // Chuyển đổi chuỗi sang Enum
        if (type_str == "prismatic") {
            lp.type = rnea_control::PRISMATIC;
            RCLCPP_INFO(node->get_logger(), "Link %ld is PRISMATIC", i+1);
        } else {
            lp.type = rnea_control::REVOLUTE;
            RCLCPP_INFO(node->get_logger(), "Link %ld is REVOLUTE", i+1);
        }

        // --- 2. CÁC THAM SỐ KHÁC (MASS, COM...) GIỮ NGUYÊN ---
        std::string p_mass = prefix + "mass";
        if (!node->has_parameter(p_mass)) node->declare_parameter(p_mass, 0.0);
        lp.m = node->get_parameter(p_mass).as_double();

        std::string p_com = prefix + "com";
        if (!node->has_parameter(p_com)) node->declare_parameter(p_com, std::vector<double>{0,0,0});
        auto com_v = node->get_parameter(p_com).as_double_array();
        lp.com = Eigen::Vector3d(com_v[0], com_v[1], com_v[2]);

        std::string p_p = prefix + "p";
        if (!node->has_parameter(p_p)) node->declare_parameter(p_p, std::vector<double>{0,0,0});
        auto p_vec = node->get_parameter(p_p).as_double_array();
        lp.p = Eigen::Vector3d(p_vec[0], p_vec[1], p_vec[2]);

        std::string p_inertia = prefix + "inertia";
        if (!node->has_parameter(p_inertia)) node->declare_parameter(p_inertia, std::vector<double>(9, 0.0));
        auto i_v = node->get_parameter(p_inertia).as_double_array();
        lp.I = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(i_v.data());

        robot_links.push_back(lp);
    }

    // 3. Khởi tạo Solver và Vector
    solver_ = std::make_unique<rnea_control::rneasolver>(robot_links);

    q_.resize(num_joints_);
    dq_.resize(num_joints_);
    ddq_des_.resize(num_joints_);
    tau_cmd_.resize(num_joints_);
    
    // Resize buffer lệnh và đặt mặc định là 0
    commands_.resize(num_joints_, 0.0);

    // 4. Khởi tạo Subscriber (ĐÃ THÊM MỚI)
    // Topic: /controller_manager/rnea_controller/commands (hoặc tương tự)
    command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/commands", 
        rclcpp::SystemDefaultsQoS(),
        std::bind(&rneacontroller::command_callback, this, std::placeholders::_1)
    );

    // 5. Cấu hình Real-time (Sửa thành WARN để không chết simulation)
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        RCLCPP_WARN(node->get_logger(), "mlockall failed! (Không sao nếu chạy Simulation)");
        // Không return ERROR ở đây nữa
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rneacontroller::on_activate(const rclcpp_lifecycle::State &) {
    struct sched_param param;
    param.sched_priority = 99; // Priority cao nhất
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        RCLCPP_WARN(get_node()->get_logger(), "Không thể đặt ưu tiên SCHED_FIFO (Cần quyền root/ulimit).");
    }
    
    // Reset lệnh về 0 khi mới bật controller để an toàn
    std::fill(commands_.begin(), commands_.end(), 0.0);
    
    return controller_interface::CallbackReturn::SUCCESS;
}

// Hàm Callback nhận lệnh từ Topic (ĐÃ THÊM MỚI)
void rneacontroller::command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != num_joints_) {
        RCLCPP_ERROR(get_node()->get_logger(), 
            "Kích thước lệnh không khớp! Nhận %ld, Cần %ld", 
            msg->data.size(), num_joints_);
        return;
    }
    // Copy dữ liệu vào buffer (đây là input gia tốc mong muốn)
    for (size_t i = 0; i < num_joints_; ++i) {
        commands_[i] = msg->data[i];
    }
}

controller_interface::return_type rneacontroller::update(const rclcpp::Time &, const rclcpp::Duration &) {
    // 1. Đọc trạng thái từ phần cứng (Gazebo)
    for (size_t i = 0; i < num_joints_; ++i) {
        q_[i] = state_interfaces_[i*2].get_value();      // Position
        dq_[i] = state_interfaces_[i*2 + 1].get_value(); // Velocity
        
        // Cập nhật gia tốc mong muốn từ Subscriber
        ddq_des_[i] = commands_[i]; 
    }

    // 2. Tính toán RNEA (Inverse Dynamics)
    // Input: q, dq, ddq_des (từ subscriber)
    // Output: tau_cmd (lực cần thiết)
    solver_->solve(q_, dq_, ddq_des_, tau_cmd_);

    // 3. Gửi lệnh Torque xuống phần cứng
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