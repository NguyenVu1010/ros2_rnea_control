#include "rnea_control/rnea_controller.hpp"
#include <sys/mman.h>
#include <pthread.h>
#include <algorithm> // for std::fill

namespace rnea_control {

// =============================================================================
// 1. KHỞI TẠO (INIT)
// =============================================================================
controller_interface::CallbackReturn rneacontroller::on_init() {
    return controller_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// 2. CẤU HÌNH (CONFIGURE) - Đọc YAML và Khởi tạo Module
// =============================================================================
controller_interface::CallbackReturn rneacontroller::on_configure(const rclcpp_lifecycle::State &) {
    auto node = get_node();

    // --- A. Lấy danh sách Joint ---
    if (!node->get_parameter("joints", joint_names_)) {
        RCLCPP_ERROR(node->get_logger(), "Không tìm thấy tham số 'joints'!");
        return controller_interface::CallbackReturn::ERROR;
    }
    num_joints_ = joint_names_.size();

    // --- B. Resize các Vector ---
    kp_.resize(num_joints_);
    kd_.resize(num_joints_);
    q_.resize(num_joints_);
    dq_.resize(num_joints_);
    q_des_.resize(num_joints_);
    dq_des_.resize(num_joints_);
    ddq_des_.resize(num_joints_);
    ddq_ref_.resize(num_joints_);
    tau_cmd_.resize(num_joints_);
    
    traj_generators_.resize(num_joints_); // Resize bộ sinh quỹ đạo khớp
    dq_cmd_prev_.resize(num_joints_, 0.0); // Buffer cho CLIK

    // --- C. Load Gains (PD) ---
    for (size_t i = 0; i < num_joints_; ++i) {
        std::string p_param = "gains." + joint_names_[i] + ".p";
        std::string d_param = "gains." + joint_names_[i] + ".d";
        
        if (!node->has_parameter(p_param)) node->declare_parameter(p_param, 1000.0);
        if (!node->has_parameter(d_param)) node->declare_parameter(d_param, 50.0);
        
        kp_[i] = node->get_parameter(p_param).as_double();
        kd_[i] = node->get_parameter(d_param).as_double();
    }

    // --- D. Load Dynamic Params & Kinematics Params ---
    std::vector<LinkParams> robot_links;
    for (size_t i = 0; i < num_joints_; ++i) {
        std::string prefix = "robot_description_parameters.link" + std::to_string(i+1) + ".";
        LinkParams lp;
        
        // 1. Type (Prismatic/Revolute)
        std::string p_type = prefix + "type";
        if (!node->has_parameter(p_type)) node->declare_parameter(p_type, "revolute");
        std::string type_str = node->get_parameter(p_type).as_string();
        lp.type = (type_str == "prismatic") ? rnea_control::PRISMATIC : rnea_control::REVOLUTE;

        // 2. Mass
        std::string p_mass = prefix + "mass";
        if (!node->has_parameter(p_mass)) node->declare_parameter(p_mass, 0.0);
        lp.m = node->get_parameter(p_mass).as_double();

        // 3. CoM
        std::string p_com = prefix + "com";
        if (!node->has_parameter(p_com)) node->declare_parameter(p_com, std::vector<double>{0,0,0});
        auto com_v = node->get_parameter(p_com).as_double_array();
        lp.com = Eigen::Vector3d(com_v[0], com_v[1], com_v[2]);

        // 4. Inertia
        std::string p_inertia = prefix + "inertia";
        if (!node->has_parameter(p_inertia)) node->declare_parameter(p_inertia, std::vector<double>(9, 0.0));
        auto i_v = node->get_parameter(p_inertia).as_double_array();
        lp.I = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(i_v.data());

        // 5. P Vector (Động lực học)
        std::string p_p = prefix + "p";
        if (!node->has_parameter(p_p)) node->declare_parameter(p_p, std::vector<double>{0,0,0});
        auto p_vec = node->get_parameter(p_p).as_double_array();
        lp.p = Eigen::Vector3d(p_vec[0], p_vec[1], p_vec[2]);

        // 6. DH Parameters (Động học) [a, alpha, d, theta]
        std::string p_dh = prefix + "dh_params";
        if (!node->has_parameter(p_dh)) node->declare_parameter(p_dh, std::vector<double>{0,0,0,0});
        auto dh_vec = node->get_parameter(p_dh).as_double_array();
        lp.a = dh_vec[0];
        lp.alpha = dh_vec[1];
        lp.d = dh_vec[2];
        lp.theta = dh_vec[3];

        robot_links.push_back(lp);
        
        RCLCPP_INFO(node->get_logger(), "Link %ld: Type=%s, Mass=%.3f, DH=[%.2f, %.2f, %.2f, %.2f]", 
                    i+1, type_str.c_str(), lp.m, lp.a, lp.alpha, lp.d, lp.theta);
    }

    // --- E. Khởi tạo Solvers ---
    // 1. RNEA Solver (Tính Torque)
    solver_ = std::make_unique<rnea_control::rneasolver>(robot_links);
    
    // 2. Kinematics Solver (Tính FK & Jacobian cho chế độ Cartesian)
    kin_solver_ = std::make_unique<rnea_control::KinematicsSolver>(robot_links);

    // --- F. Khởi tạo Subscriber ---
    // Topic này nhận cả lệnh Joint (6 phần tử) và lệnh Cartesian (5 phần tử)
    command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/commands", rclcpp::SystemDefaultsQoS(),
        std::bind(&rneacontroller::command_callback, this, std::placeholders::_1));

    // --- G. Cấu hình Real-time ---
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        RCLCPP_WARN(node->get_logger(), "Warning: mlockall failed.");
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// 3. KÍCH HOẠT (ACTIVATE)
// =============================================================================
controller_interface::CallbackReturn rneacontroller::on_activate(const rclcpp_lifecycle::State &) {
    // Reset trạng thái về vị trí hiện tại để robot đứng yên
    for (size_t i = 0; i < num_joints_; ++i) {
        double current_pos = state_interfaces_[i*2].get_value();
        q_des_[i] = current_pos;
        dq_des_[i] = 0.0;
        ddq_des_[i] = 0.0;
        dq_cmd_prev_[i] = 0.0;
        
        traj_generators_[i].reset();
    }
    
    // Reset Cartesian Generator
    cart_gen_.reset();
    control_mode_ = MODE_IDLE;
    // --- KHỞI TẠO LẠI CỜ ---
    is_first_update_ = true;
    has_pending_command_ = false;
    // -----------------------

    RCLCPP_INFO(get_node()->get_logger(), "RNEA Controller Activated. Mode: IDLE");
    return controller_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// 4. XỬ LÝ LỆNH (CALLBACK) - Phân loại lệnh Joint hay Cartesian
// =============================================================================
void rneacontroller::command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    motion_start_time_ = get_node()->now();

    pending_command_ = msg;
    has_pending_command_ = true;
    
    // CASE 1: LỆNH JOINT SPACE (Size = NumJoints + 1)
    // Format: [q1, q2, ..., qn, DURATION]
    if (msg->data.size() == num_joints_ + 1) {
        double duration = msg->data[num_joints_];
        
        // Cài đặt mục tiêu cho từng khớp
        for (size_t i = 0; i < num_joints_; ++i) {
            traj_generators_[i].set_target(q_[i], msg->data[i], duration);
        }
        
        control_mode_ = MODE_JOINT_TRAJECTORY;
        RCLCPP_INFO(get_node()->get_logger(), "Mode: JOINT TRAJECTORY (T=%.1fs)", duration);
    }
    
    // CASE 2: LỆNH CARTESIAN SPACE (Size = 5)
    // Format: [x, y, z, v_max, a_max]
    else if (msg->data.size() == 5) {
        // Lấy vị trí hiện tại qua FK
        Eigen::Vector3d p_start = kin_solver_->computeFK(q_);
        
        // Lấy đích từ lệnh
        Eigen::Vector3d p_target(msg->data[0], msg->data[1], msg->data[2]);
        double v_max = msg->data[3];
        double a_max = msg->data[4];

        // Cài đặt bộ sinh quỹ đạo đường thẳng
        if (cart_gen_.set_target(p_start, p_target, v_max, a_max)) {
            control_mode_ = MODE_CARTESIAN_TRAJECTORY;
            
            // Reset buffer vận tốc cũ để tính gia tốc mượt
            std::fill(dq_cmd_prev_.begin(), dq_cmd_prev_.end(), 0.0);
            
            RCLCPP_INFO(get_node()->get_logger(), "Mode: CARTESIAN LINE (v=%.2f, a=%.2f)", v_max, a_max);
        } else {
            RCLCPP_WARN(get_node()->get_logger(), "Cartesian Target too close!");
            control_mode_ = MODE_IDLE;
        }
    } 
    else {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid Command Size! Joint=[%ld], Cart=[5]", num_joints_+1);
    }
}

// =============================================================================
// 5. VÒNG LẶP ĐIỀU KHIỂN (REAL-TIME UPDATE)
// =============================================================================
controller_interface::return_type rneacontroller::update(const rclcpp::Time & time, const rclcpp::Duration & period) {
    double dt = period.seconds();
    if (dt == 0.0) dt = 0.001;

    // 1. ĐỒNG BỘ THỜI GIAN LẦN ĐẦU
    if (is_first_update_) {
        motion_start_time_ = time;
        is_first_update_ = false;
    }

    // 2. XỬ LÝ LỆNH TỪ CALLBACK (NẾU CÓ)
    if (has_pending_command_) {
        auto msg = pending_command_;
        
        // Đặt mốc thời gian bằng chính thời gian hiện tại của Controller
        motion_start_time_ = time; 

        // --- Logic phân loại lệnh (Copy từ code cũ vào đây) ---
        
        // CASE 1: JOINT TRAJECTORY
        if (msg->data.size() == num_joints_ + 1) {
            double duration = msg->data[num_joints_];
            for (size_t i = 0; i < num_joints_; ++i) {
                traj_generators_[i].set_target(q_[i], msg->data[i], duration);
            }
            control_mode_ = MODE_JOINT_TRAJECTORY;
            RCLCPP_INFO(get_node()->get_logger(), "New Command: JOINT TRAJECTORY (%.1fs)", duration);
        }
        // CASE 2: CARTESIAN TRAJECTORY
        else if (msg->data.size() == 5) {
            Eigen::Vector3d p_start = kin_solver_->computeFK(q_);
            Eigen::Vector3d p_target(msg->data[0], msg->data[1], msg->data[2]);
            double v_max = msg->data[3];
            double a_max = msg->data[4];

            if (cart_gen_.set_target(p_start, p_target, v_max, a_max)) {
                control_mode_ = MODE_CARTESIAN_TRAJECTORY;
                std::fill(dq_cmd_prev_.begin(), dq_cmd_prev_.end(), 0.0);
                RCLCPP_INFO(get_node()->get_logger(), "New Command: CARTESIAN (v=%.2f)", v_max);
            } else {
                RCLCPP_WARN(get_node()->get_logger(), "Cartesian Target Invalid!");
                control_mode_ = MODE_IDLE;
            }
        } 
        else {
            RCLCPP_ERROR(get_node()->get_logger(), "Invalid Command Size!");
        }

        // Đánh dấu đã xử lý xong
        has_pending_command_ = false;
    }

    // 3. TÍNH THỜI GIAN TRÔI QUA (An toàn tuyệt đối)
    // Vì motion_start_time_ được gán bằng 'time', nên chúng cùng nguồn đồng hồ
    double t_elapsed = (time - motion_start_time_).seconds();

    // 4. ĐỌC TRẠNG THÁI HIỆN TẠI
    for (size_t i = 0; i < num_joints_; ++i) {
        q_[i] = state_interfaces_[i*2].get_value();
        dq_[i] = state_interfaces_[i*2 + 1].get_value();
    }

    // 5. TẠO QUỸ ĐẠO (Trajectory Generation)
    if (control_mode_ == MODE_JOINT_TRAJECTORY) {
        bool all_done = true;
        for (size_t i = 0; i < num_joints_; ++i) {
            if (traj_generators_[i].is_moving()) {
                auto pt = traj_generators_[i].get_sample(t_elapsed);
                q_des_[i] = pt.q;
                dq_des_[i] = pt.dq;
                ddq_des_[i] = pt.ddq;
                all_done = false;
            } else {
                dq_des_[i] = 0.0; ddq_des_[i] = 0.0;
            }
        }
        if (all_done) control_mode_ = MODE_IDLE;

    } else if (control_mode_ == MODE_CARTESIAN_TRAJECTORY) {
        if (cart_gen_.is_moving()) {
            auto cart_des = cart_gen_.get_sample(t_elapsed);
            
            // --- Logic CLIK / Jacobian (Copy lại đoạn cũ vào đây) ---
            Eigen::Vector3d p_curr = kin_solver_->computeFK(q_);
            Eigen::MatrixXd J = kin_solver_->computeJacobian(q_);
            Eigen::MatrixXd J_pos = J.block<3, 5>(0, 0);
            Eigen::MatrixXd J_pinv = J_pos.completeOrthogonalDecomposition().pseudoInverse();
            
            Eigen::Vector3d error_pos = cart_des.p - p_curr;
            double Kp_cart = 50.0;
            Eigen::Vector3d v_task = cart_des.v + Kp_cart * error_pos;
            Eigen::VectorXd dq_cmd = J_pinv * v_task;

            for(size_t i=0; i<num_joints_; ++i) {
                q_des_[i] += dq_cmd[i] * dt; 
                dq_des_[i] = dq_cmd[i];
                ddq_des_[i] = (dq_des_[i] - dq_cmd_prev_[i]) / dt;
                dq_cmd_prev_[i] = dq_des_[i];
            }
        } else {
            control_mode_ = MODE_IDLE;
            std::fill(dq_des_.begin(), dq_des_.end(), 0.0);
            std::fill(ddq_des_.begin(), ddq_des_.end(), 0.0);
        }
    }

    // 6. TÍNH TOÁN RNEA + PD
    for (size_t i = 0; i < num_joints_; ++i) {
        double error_q = q_des_[i] - q_[i];
        double error_dq = dq_des_[i] - dq_[i];
        ddq_ref_[i] = ddq_des_[i] + kp_[i] * error_q + kd_[i] * error_dq;
    }

    // 7. GIẢI & GỬI TORQUE
    solver_->solve(q_, dq_, ddq_ref_, tau_cmd_);

    for (size_t i = 0; i < num_joints_; ++i) {
        command_interfaces_[i].set_value(tau_cmd_[i]);
    }

    return controller_interface::return_type::OK;
}

// =============================================================================
// 6. CẤU HÌNH INTERFACE (BOILERPLATE)
// =============================================================================
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