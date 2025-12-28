#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <numeric>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// Struct để lưu cấu hình cho mỗi cánh tay 2-DOF
struct ArmConfig
{
    std::vector<std::string> joint_names;
    std::string base_frame; // Gốc của mỗi cánh tay, ví dụ "link_sr_11"
};

class VirtualConstraintController : public rclcpp::Node
{
public:
    VirtualConstraintController()
        : Node("virtual_constraint_controller_node")
    {
        // Khởi tạo thông số
        controller_topic_ = "/sr_joints_position_controller/command";
        l1_ = 0.16;
        l2_ = 0.16;
        base_frame_ = "base_link";
        z_source_frame_ = "link_1";
        z_source_offset_ = 0.0641527277482495;
        control_rate_ = 100.0; // Hz
        q_offset = 0.745;
        scale = 0.97;
        target_x_extension_ = 0.0;

        arm_configs_ = {
            {{"joint_sr_11", "joint_sr_21"}, "link_sr_11"},
            {{"joint_sr_12", "joint_sr_22"}, "link_sr_12"},
            {{"joint_sr_13", "joint_sr_23"}, "link_sr_13"},
            {{"joint_sr_14", "joint_sr_24"}, "link_sr_14"}
        };

        // Thiết lập TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publisher trong ROS 2
        command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(controller_topic_, 10);

        // Timer trong ROS 2
        auto period = std::chrono::milliseconds(static_cast<long>(1000.0 / control_rate_));
        timer_ = this->create_wall_timer(period, std::bind(&VirtualConstraintController::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Virtual Constraint Controller (ROS 2) started.");
        RCLCPP_INFO(this->get_logger(), "Publishing commands to: %s", controller_topic_.c_str());
    }

private:
    bool solveIk2d(double x, double y, double& q1, double& q2)
    {
        double d_sq = x * x + y * y;
        double d = sqrt(d_sq);

        if (d > l1_ + l2_ - 1e-6 || d < std::abs(l1_ - l2_) + 1e-6) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "IK target (x=%.2f, y=%.2f, d=%.2f) out of reach.", x, y, d);
            return false;
        }

        double cos_q2 = (d_sq - l1_ * l1_ - l2_ * l2_) / (2 * l1_ * l2_);
        cos_q2 = std::max(-1.0, std::min(1.0, cos_q2));

        double q2_sol1 = acos(cos_q2);
        double q2_sol2 = -acos(cos_q2);
        double alpha = atan2(y, x);

        double beta1 = atan2(l2_ * sin(q2_sol1), l1_ + l2_ * cos(q2_sol1));
        double q1_sol1 = alpha - beta1;

        double beta2 = atan2(l2_ * sin(q2_sol2), l1_ + l2_ * cos(q2_sol2));
        double q1_sol2 = alpha - beta2;

        if (q1_sol1 <= q1_sol2) {
            q1 = q1_sol1; q2 = q2_sol1;
        } else {
            q1 = q1_sol2; q2 = q2_sol2;
        }
        return true;
    }

    void controlLoop()
    {
        try
        {
            // Tra cứu TF trong ROS 2
            auto ts_base_to_z_source = tf_buffer_->lookupTransform(
                base_frame_, z_source_frame_, tf2::TimePointZero);
            
            double target_z_global = ts_base_to_z_source.transform.translation.z + z_source_offset_ * scale;

            std::vector<double> all_solutions;
            all_solutions.reserve(arm_configs_.size() * 2);
            bool all_ik_succeeded = true;

            for (const auto& arm : arm_configs_) {
                auto ts_base_to_arm_base = tf_buffer_->lookupTransform(
                    base_frame_, arm.base_frame, tf2::TimePointZero);
                
                double arm_base_z_global = ts_base_to_arm_base.transform.translation.z;

                double x_target_local = target_x_extension_;
                double y_target_local = target_z_global - arm_base_z_global - 0.003;
                double q1, q2;

                if (solveIk2d(x_target_local, y_target_local, q1, q2)) {
                    all_solutions.push_back((q1 - q_offset) * scale);
                    all_solutions.push_back((q2 - 3.14159 + 2 * q_offset) * scale);
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "IK failed for arm '%s'.", arm.base_frame.c_str());
                    all_ik_succeeded = false;
                    break;
                }
            }
            
            if (all_ik_succeeded) {
                publishCommand(all_solutions);
            }
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "TF Exception: %s", ex.what());
        }
    }
    
    void publishCommand(const std::vector<double>& joint_positions)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = joint_positions;
        command_pub_->publish(msg);
    }

    // Các biến thành viên ROS 2
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string controller_topic_;
    double l1_, l2_;
    std::string base_frame_;
    std::string z_source_frame_;
    double z_source_offset_;
    double control_rate_;
    double target_x_extension_;
    double q_offset;
    double scale;
    std::vector<ArmConfig> arm_configs_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VirtualConstraintController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}