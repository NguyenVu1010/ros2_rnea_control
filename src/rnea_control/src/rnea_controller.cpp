#include "rnea_control/rnea_controller.hpp"
#include "rnea_control/core/math_utils.hpp" 

#include <sys/mman.h>
#include <pthread.h>
#include <algorithm> 

namespace rnea_control {

// =============================================================================
// 1. INIT
// =============================================================================
controller_interface::CallbackReturn rneacontroller::on_init() {
    return controller_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// 2. CONFIGURE
// =============================================================================
controller_interface::CallbackReturn rneacontroller::on_configure(const rclcpp_lifecycle::State &) {
    auto node = get_node();

    // A. Lấy danh sách Joint
    if (!node->get_parameter("joints", joint_names_)) {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'joints' not found!");
        return controller_interface::CallbackReturn::ERROR;
    }
    num_joints_ = joint_names_.size();

    // B. Resize Vectors
    kp_.resize(num_joints_);
    kd_.resize(num_joints_);
    q_.resize(num_joints_);
    dq_.resize(num_joints_);
    q_des_.resize(num_joints_);
    dq_des_.resize(num_joints_);
    ddq_des_.resize(num_joints_);
    ddq_ref_.resize(num_joints_);
    tau_cmd_.resize(num_joints_);
    
    // Resize modules
    traj_generators_.resize(num_joints_); 
    dq_cmd_prev_.resize(num_joints_, 0.0);

    // C. Load Gains (PD)
    for (size_t i = 0; i < num_joints_; ++i) {
        std::string p_param = "gains." + joint_names_[i] + ".p";
        std::string d_param = "gains." + joint_names_[i] + ".d";
        
        if (!node->has_parameter(p_param)) node->declare_parameter(p_param, 500.0);
        if (!node->has_parameter(d_param)) node->declare_parameter(d_param, 10.0);
        
        kp_[i] = node->get_parameter(p_param).as_double();
        kd_[i] = node->get_parameter(d_param).as_double();
    }

    // D. Load Dynamic Params
    std::vector<core::LinkParams> robot_links;
    for (size_t i = 0; i < num_joints_; ++i) {
        std::string prefix = "robot_description_parameters.link" + std::to_string(i+1) + ".";
        core::LinkParams lp;
        
        // Type
        std::string p_type = prefix + "type";
        if (!node->has_parameter(p_type)) node->declare_parameter(p_type, "revolute");
        std::string type_str = node->get_parameter(p_type).as_string();
        lp.type = (type_str == "prismatic") ? core::PRISMATIC : core::REVOLUTE;

        // Mass
        std::string p_mass = prefix + "mass";
        if (!node->has_parameter(p_mass)) node->declare_parameter(p_mass, 0.0);
        lp.m = node->get_parameter(p_mass).as_double();

        // CoM
        std::string p_com = prefix + "com";
        if (!node->has_parameter(p_com)) node->declare_parameter(p_com, std::vector<double>{0,0,0});
        auto com_v = node->get_parameter(p_com).as_double_array();
        lp.com = Eigen::Vector3d(com_v[0], com_v[1], com_v[2]);

        // Inertia
        std::string p_inertia = prefix + "inertia";
        if (!node->has_parameter(p_inertia)) node->declare_parameter(p_inertia, std::vector<double>(9, 0.0));
        auto i_v = node->get_parameter(p_inertia).as_double_array();
        lp.I = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(i_v.data());

        // P Vector
        std::string p_p = prefix + "p";
        if (!node->has_parameter(p_p)) node->declare_parameter(p_p, std::vector<double>{0,0,0});
        auto p_vec = node->get_parameter(p_p).as_double_array();
        lp.p = Eigen::Vector3d(p_vec[0], p_vec[1], p_vec[2]);

        // DH Params
        std::string p_dh = prefix + "dh_params";
        if (!node->has_parameter(p_dh)) node->declare_parameter(p_dh, std::vector<double>{0,0,0,0});
        auto dh_vec = node->get_parameter(p_dh).as_double_array();
        lp.a = dh_vec[0]; lp.alpha = dh_vec[1]; lp.d = dh_vec[2]; lp.theta = dh_vec[3];

        robot_links.push_back(lp);
        RCLCPP_INFO(node->get_logger(), "Link %ld Configured: %s", i+1, type_str.c_str());
    }

    // E. Init Modules
    solver_ = std::make_unique<dynamics::RNEASolver>(robot_links);
    kin_solver_ = std::make_unique<kinematics::ForwardKinematics>(robot_links);
    jac_solver_ = std::make_unique<kinematics::JacobianSolver>(robot_links);
    motion_control_ = std::make_unique<control::MotionControl>();
    motion_control_->set_gains(20.0, 10.0);

    // F. Subscriber
    command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/commands", rclcpp::SystemDefaultsQoS(),
        std::bind(&rneacontroller::command_callback, this, std::placeholders::_1));

    // G. Realtime
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        RCLCPP_WARN(node->get_logger(), "mlockall failed");
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// 3. ACTIVATE
// =============================================================================
controller_interface::CallbackReturn rneacontroller::on_activate(const rclcpp_lifecycle::State &) {
    for (size_t i = 0; i < num_joints_; ++i) {
        double current_pos = state_interfaces_[i*2].get_value();
        q_des_[i] = current_pos;
        dq_des_[i] = 0.0; ddq_des_[i] = 0.0; dq_cmd_prev_[i] = 0.0;
        
        traj_generators_[i].reset();
        traj_generators_[i].set_target(current_pos, current_pos, 0.0);
    }
    cart_gen_.reset();
    control_mode_ = MODE_IDLE;
    is_first_update_ = true;
    has_pending_command_ = false;

    RCLCPP_INFO(get_node()->get_logger(), "RNEA Controller Activated.");
    return controller_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// 3.5 DEACTIVATE (QUAN TRỌNG ĐỂ TRÁNH LỖI SYMBOL)
// =============================================================================
controller_interface::CallbackReturn rneacontroller::on_deactivate(const rclcpp_lifecycle::State &) {
    // Dừng robot
    for (size_t i = 0; i < num_joints_; ++i) {
        command_interfaces_[i].set_value(0.0);
    }
    control_mode_ = MODE_IDLE;
    return controller_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// 4. CALLBACK
// =============================================================================
void rneacontroller::command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    pending_command_ = msg;
    has_pending_command_ = true;
}

// =============================================================================
// 5. UPDATE
// =============================================================================
controller_interface::return_type rneacontroller::update(const rclcpp::Time & time, const rclcpp::Duration & period) {
    double dt = period.seconds();
    if (dt <= 1e-6) dt = 0.001;

    // --- TIME SYNC ---
    if (is_first_update_) {
        motion_start_time_ = time;
        is_first_update_ = false;
    }

    // --- PROCESS COMMAND ---
    if (has_pending_command_) {
        auto msg = pending_command_;
        motion_start_time_ = time; 

        // Joint Traj
        if (msg->data.size() == num_joints_ + 1) {
            double duration = msg->data[num_joints_];
            for (size_t i = 0; i < num_joints_; ++i) {
                traj_generators_[i].set_target(q_[i], msg->data[i], duration);
            }
            control_mode_ = MODE_JOINT_TRAJECTORY;
            RCLCPP_INFO(get_node()->get_logger(), "Mode: JOINT (T=%.1fs)", duration);
        }
        // Cartesian Traj
        else if (msg->data.size() == 8) {
            kin_solver_->update(q_);
            
            core::CartesianState start;
            start.p = kin_solver_->get_ee_position();
            start.q = kin_solver_->get_ee_orientation();

            core::CartesianState end;
            end.p = Eigen::Vector3d(msg->data[0], msg->data[1], msg->data[2]);
            end.q = core::MathUtils::euler_to_quat(msg->data[3], msg->data[4], msg->data[5]);

            double v_max = msg->data[6];
            double a_max = msg->data[7];

            if (cart_gen_.set_target(start, end, v_max, a_max)) {
                control_mode_ = MODE_CARTESIAN_TRAJECTORY;
                std::fill(dq_cmd_prev_.begin(), dq_cmd_prev_.end(), 0.0);
                RCLCPP_INFO(get_node()->get_logger(), "Mode: CARTESIAN (v=%.2f)", v_max);
            } else {
                RCLCPP_WARN(get_node()->get_logger(), "Cartesian Path Invalid");
                control_mode_ = MODE_IDLE;
            }
        } 
        has_pending_command_ = false;
    }

    double t_elapsed = (time - motion_start_time_).seconds();

    // --- READ STATE ---
    for (size_t i = 0; i < num_joints_; ++i) {
        q_[i] = state_interfaces_[i*2].get_value();
        dq_[i] = state_interfaces_[i*2 + 1].get_value();
    }

    // --- TRAJECTORY ---
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

        motion_control_->compute_joint_tracking(q_des_, dq_des_, ddq_des_, q_, dq_, kp_, kd_, ddq_ref_);

    } else if (control_mode_ == MODE_CARTESIAN_TRAJECTORY) {
        if (cart_gen_.is_moving()) {
            auto des_state = cart_gen_.get_sample(t_elapsed);
            kin_solver_->update(q_);
            
            core::CartesianState curr_state;
            curr_state.p = kin_solver_->get_ee_position();
            curr_state.q = kin_solver_->get_ee_orientation();
            
            const auto& J = jac_solver_->compute(kin_solver_->get_transforms());

            motion_control_->compute_cartesian_tracking(des_state, curr_state, J, dq_, ddq_ref_);

            // Update q_des for debug
            // ...
        } else {
            control_mode_ = MODE_IDLE;
            std::fill(dq_des_.begin(), dq_des_.end(), 0.0);
            std::fill(ddq_des_.begin(), ddq_des_.end(), 0.0);
            motion_control_->compute_joint_tracking(q_des_, dq_des_, ddq_des_, q_, dq_, kp_, kd_, ddq_ref_);
        }
    } else {
        // IDLE
        std::fill(dq_des_.begin(), dq_des_.end(), 0.0);
        std::fill(ddq_des_.begin(), ddq_des_.end(), 0.0);
        motion_control_->compute_joint_tracking(q_des_, dq_des_, ddq_des_, q_, dq_, kp_, kd_, ddq_ref_);
    }

    // --- DYNAMICS ---
    solver_->solve(q_, dq_, ddq_ref_, tau_cmd_);

    // --- WRITE ---
    for (size_t i = 0; i < num_joints_; ++i) {
        double limit = (i == 0) ? 200.0 : 100.0;
        double cmd = std::clamp(tau_cmd_[i], -limit, limit);
        command_interfaces_[i].set_value(cmd);
    }

    return controller_interface::return_type::OK;
}

// =============================================================================
// 6. INTERFACE
// =============================================================================
controller_interface::InterfaceConfiguration rneacontroller::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_) config.names.push_back(joint + "/effort");
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