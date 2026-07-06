#include <chrono>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

#include "staubli_tx2_60l_controller/pid.hpp"

using namespace std::chrono_literals;

namespace staubli_tx2_60l_controller {

class ForcePIDController : public rclcpp::Node {
public:
    ForcePIDController() : rclcpp::Node("force_pid_controller")
    {
        this->declare_parameter<double>("KP", 1e-3); // N/m
        this->declare_parameter<double>("KI", 1e-4); // N/s/m
        this->declare_parameter<double>("KD", 2e-3); // N.s/m
        this->declare_parameter<double>("MAX_OUTPUT", 0.05); // 0.5 cm max par frame
        this->declare_parameter<double>("TARGET", 300); // N
        this->declare_parameter<int>("FREQ", 300); // Hz
        const double KP = this->get_parameter("KP").as_double();
        const double KD = this->get_parameter("KD").as_double();
        const double KI = this->get_parameter("KI").as_double();
        const double MAX_OUTPUT = this->get_parameter("MAX_OUTPUT").as_double();
        int rate_hz = this->get_parameter("FREQ").as_int();
        target_force = this->get_parameter("TARGET").as_double();

        PIDGains g;
        g.kp = KP;
        g.ki = KI;
        g.kd = KD;
        g.max_output = MAX_OUTPUT;
        pid = PIDController(g);

        sub_force = this->create_subscription<std_msgs::msg::Float64>(
            "/Fz", 10,
            std::bind(&ForcePIDController::onForce, this, std::placeholders::_1));

        sub_cartesian_state = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cartesian_state", 10,
            std::bind(&ForcePIDController::onCartesianState, this, std::placeholders::_1));

        sub_target_force = this->create_subscription<std_msgs::msg::Float64>(
            "/target_force", 10,
            std::bind(&ForcePIDController::onTargetForce, this, std::placeholders::_1));

        pub_cartesian_target = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/cartesian_target", 10);

        auto period = std::chrono::milliseconds(1000 / rate_hz);
        control_timer = this->create_wall_timer(period, std::bind(&ForcePIDController::controlLoop, this));
        last_time = this->now();

        RCLCPP_INFO(this->get_logger(), "ForcePIDController started");
    }

private:
    PIDController pid;

    double current_force = 0.0;
    double target_force;
    double prev_target_force = 0.0;

    double current_z = 0.0;
    double current_x = 0.0;
    double current_y = 0.0;

    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;

    bool force_received = false;
    bool cartesian_received = false;

    rclcpp::Time last_time;
    rclcpp::TimerBase::SharedPtr control_timer;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_force;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_cartesian_state;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_target_force;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cartesian_target;

    void onForce(const std_msgs::msg::Float64::SharedPtr msg) {
        current_force = msg->data;
        force_received = true;
    }

    void onCartesianState(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_x = msg->pose.position.x;
        current_y = msg->pose.position.y;
        current_z = - msg->pose.position.z; //compute negative here to avoid reapplying negative later and ending up with the wrong sign
        qx = msg->pose.orientation.x;
        qy = msg->pose.orientation.y;
        qz = msg->pose.orientation.z;
        qw = msg->pose.orientation.w;
        cartesian_received = true;
        }

    void onTargetForce(const std_msgs::msg::Float64::SharedPtr msg) {
        double new_target = msg->data;
        if (new_target != prev_target_force) {
            pid.reset();
            RCLCPP_INFO(this->get_logger(),"Target force changed %.3f -> %.3f",prev_target_force, new_target);
            prev_target_force = new_target;
            }
        target_force = new_target;
        }

    void controlLoop()
    {
        if (!force_received || !cartesian_received) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "waiting for topics...");
            return;
            }

        auto now = this->now();
        double dt = (now - last_time).seconds();
        last_time = now;
        if (dt <= 0.0 || dt > 0.5) return;

        double force_error = current_force - target_force;
        double delta_z = - pid.compute(force_error, dt);

        geometry_msgs::msg::PoseStamped target;
        target.header.stamp = now;
        target.header.frame_id = "base_link";
        target.pose.position.x = current_x;
        target.pose.position.y = current_y;
        target.pose.position.z = (current_z + delta_z);
        target.pose.orientation.x = qx;
        target.pose.orientation.y = qy;
        target.pose.orientation.z = qz;
        target.pose.orientation.w = qw;

        pub_cartesian_target->publish(target);
        }
    };

} // namespace real_commander

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<staubli_tx2_60l_controller::ForcePIDController>());
    rclcpp::shutdown();
    return 0;
    }