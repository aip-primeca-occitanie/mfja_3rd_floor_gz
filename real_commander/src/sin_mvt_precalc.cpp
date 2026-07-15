#include <chrono>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

namespace staubli_tx2_60l_controller {

class SinusoidalMotionController : public rclcpp::Node
{
public:
    SinusoidalMotionController() : rclcpp::Node("sinusoidal_motion_controller") {
        this->declare_parameter<double>("AMPLITUDE", 0.05); // rad
        this->declare_parameter<double>("OMEGA", 0.5); // rad/s
        this->declare_parameter<int>("LOOP_FREQ", 250); // Hz (echantillonnage de la trajectoire)
        this->declare_parameter<double>("DURATION", 15.0); // s
        this->declare_parameter<int>("JOINT_INDEX", 2); // index 0-based -> joint_3

        amplitude = this->get_parameter("AMPLITUDE_RAD").as_double();
        omega = this->get_parameter("OMEGA").as_double();
        rate_hz = this->get_parameter("LOOP_FREQ").as_int();
        duration = this->get_parameter("DURATION").as_double();
        joint_index = this->get_parameter("JOINT_INDEX").as_int();
        joint_names = std::vector<std::string>{"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};

        sub_joint_state = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&SinusoidalMotionController::onJointState, this, std::placeholders::_1));

        pub_joint_path = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_path_command", 10);

        init_timer = this->create_wall_timer(100ms,
            std::bind(&SinusoidalMotionController::tryPublishOnce, this));

        RCLCPP_INFO(this->get_logger(),
            "SinusoidalMotionController (joint) pret : A=%.4f rad, w=%.3f rad/s, duree=%.1f s, joint_index=%d",
            amplitude, omega, duration, joint_index);
        }

private:
    double amplitude;
    double omega;
    int rate_hz;
    double duration;
    int joint_index;
    std::vector<std::string> joint_names;

    bool joint_state_received = false;
    bool trajectory_sent = false;
    std::vector<double> base_positions;

    rclcpp::TimerBase::SharedPtr init_timer;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_joint_path;

    void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (joint_state_received) return;
        base_positions.assign(joint_names.size(), 0.0);
        for (size_t i = 0; i < joint_names.size(); ++i) {
            for (size_t j = 0; j < msg->name.size(); ++j) {
                if (msg->name[j] == joint_names[i]) {
                    base_positions[i] = msg->position[j];
                    break;
                    }
                }
            }
        joint_state_received = true;
        }

    void tryPublishOnce() {
        if (trajectory_sent) {
            init_timer->cancel();
            return;
            }
        if (!joint_state_received) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "waiting for /joint_states...");
            return;
            }
        publishFullTrajectory();
        trajectory_sent = true;
        init_timer->cancel();
        }

    void publishFullTrajectory()
        {
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.header.stamp = this->now();
        traj_msg.header.frame_id = "";
        traj_msg.joint_names = joint_names;

        size_t n_points = static_cast<size_t>(duration * rate_hz);
        traj_msg.points.reserve(n_points);

        for (size_t i = 0; i < n_points; ++i) {
            double t = static_cast<double>(i) / static_cast<double>(rate_hz);
            double delta = amplitude * std::sin(omega * t); // rad, deplacement angulaire

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = base_positions; // copie des positions de base
            point.positions[joint_index] = base_positions[joint_index] + delta;

            point.velocities.assign(joint_names.size(), 0.0);
            point.velocities[joint_index] = amplitude * omega * std::cos(omega * t);

            point.time_from_start = rclcpp::Duration::from_seconds(t);

            traj_msg.points.push_back(point);
            }

        pub_joint_path->publish(traj_msg);

        RCLCPP_INFO(this->get_logger(),
            "Trajectoire complete publiee : %zu points sur %.1f s (joint_index=%d)",
            n_points, duration, joint_index);
        }
    };
} // namespace real_commander

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<staubli_tx2_60l_controller::SinusoidalMotionController>());
    rclcpp::shutdown();
    return 0;
}