#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

namespace staubli_tx2_60l_controller {

class SinusoidalMotionController : public rclcpp::Node
{
public:
    SinusoidalMotionController() : rclcpp::Node("sinusoidal_motion_controller")
    {
        this->declare_parameter<double>("AMPLITUDE", 0.01); // m
        this->declare_parameter<double>("OMEGA", 0.5); // rad/s (frequence d'oscillation)
        this->declare_parameter<int>("LOOP_FREQ", 250); // Hz (frequence de la boucle de controle)

        amplitude = this->get_parameter("AMPLITUDE").as_double();
        omega = this->get_parameter("OMEGA").as_double();
        int rate_hz = this->get_parameter("LOOP_FREQ").as_int();

        sub_cartesian_state = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cartesian_state", 10,
            std::bind(&SinusoidalMotionController::onCartesianState, this, std::placeholders::_1));

        pub_cartesian_target = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/cartesian_target", 10);

        auto period = std::chrono::milliseconds(1000 / rate_hz);
        control_timer = this->create_wall_timer(period, std::bind(&SinusoidalMotionController::controlLoop, this));
        last_time = this->now();

        RCLCPP_INFO(this->get_logger(),
            "SinusoidalMotionController started (A=%.4f m, w=%.3f Hz)", amplitude, omega);
        
        //move to initial position
        geometry_msgs::msg::PoseStamped target;
        auto now = this->now();
        target.header.stamp = now;
        target.header.frame_id = "base_link";
        target.pose.position.x = base_x;
        target.pose.position.y = base_y;
        target.pose.position.z = base_z - 0.2;
        target.pose.orientation.x = qx;
        target.pose.orientation.y = qy;
        target.pose.orientation.z = qz;
        target.pose.orientation.w = qw;
        pub_cartesian_target->publish(target);
    }

private:
    double amplitude;
    double omega;
    double elapsed_t = 0.0; // temps ecoule depuis le debut de l'oscillation
    double z_offset = 0.0; // deplacement cumule en z

    double base_x = 0.0, base_y = 0.0, base_z = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;

    bool cartesian_received = false;

    rclcpp::Time last_time;
    rclcpp::TimerBase::SharedPtr control_timer;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_cartesian_state;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cartesian_target;

    void onCartesianState(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!cartesian_received) {
            base_x = msg->pose.position.x;
            base_y = msg->pose.position.y;
            base_z = - msg->pose.position.z;
            qx = msg->pose.orientation.x;
            qy = msg->pose.orientation.y;
            qz = msg->pose.orientation.z;
            qw = msg->pose.orientation.w;
            cartesian_received = true;
        }
    }

    void controlLoop()
    {
        if (!cartesian_received) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "waiting for /cartesian_state...");
            return;
        }

        auto now = this->now();
        double dt = (now - last_time).seconds();
        last_time = now;
        if (dt <= 0.0 || dt > 0.5) return;

        elapsed_t += dt;
        z_offset += amplitude * std::cos(omega * elapsed_t) * dt;

        geometry_msgs::msg::PoseStamped target;
        target.header.stamp    = now;
        target.header.frame_id = "base_link";
        target.pose.position.x = base_x;
        target.pose.position.y = base_y;
        target.pose.position.z = base_z + z_offset;
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
    rclcpp::spin(std::make_shared<staubli_tx2_60l_controller::SinusoidalMotionController>());
    rclcpp::shutdown();
    return 0;
}