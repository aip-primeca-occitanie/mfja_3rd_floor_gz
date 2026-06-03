#include <chrono>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "staubli_tx2_60l_controller/pid.hpp"

using namespace std::chrono_literals;

namespace staubli_tx2_60l_controller {

class StaubliController : public rclcpp::Node {
    public:
    StaubliController() : rclcpp::Node("staubli_tx2_60l_controller")
    {
        const double KP = 3.;
        const double KI = 0.;
        const double KD = 0.3;
        const double max_v=.5;

        PIDGains g;
        g.kp = KP;
        g.ki = KI;
        g.kd = KD;
        g.max_output = max_v;

        for (int i = 0; i < 6; i++) {
            pid[i] = PIDController(g);}

        // sub to self
        sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&StaubliController::onJointStates, this, std::placeholders::_1));

        // pub
        pub_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>( //command (velocity)
            "velocity_controller/commands", 10); //topic "/velocity_controller/commands" standard format of forward_command_controller

        // timer
        int rate_hz = 250;
        auto period = std::chrono::milliseconds(1000 / rate_hz);

        control_timer_ = this->create_wall_timer(
            period, std::bind(&StaubliController::controlLoop, this));

        last_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
            "StaubliController started");}

    private:
    std::array<double, 6> error{};
    std::array<double, 6> cmd{};
    std::array<PIDController, 6> pid;
    
    std::array<double, 6> pos_joint{};
    bool joint_states_received_ = false;

    rclcpp::Time last_time_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
    
    void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // joints in msg->name / msg->position / msg->velocity
        for (size_t i = 0; i < msg->name.size(); ++i) {
        const auto & name = msg->name[i];
        if (name.find("1") != std::string::npos) pos_joint[0] = msg->position[i];
        if (name.find("2") != std::string::npos) pos_joint[1] = msg->position[i];
        if (name.find("3") != std::string::npos) pos_joint[2] = msg->position[i];
        if (name.find("4") != std::string::npos) pos_joint[3] = msg->position[i];
        if (name.find("5") != std::string::npos) pos_joint[4] = msg->position[i];
        if (name.find("6") != std::string::npos) pos_joint[5] = msg->position[i];
        }
        joint_states_received_ = true;
        }

    //-----------
    // main loop
    //-----------
    void controlLoop()
    {
        if (!joint_states_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "waiting...");
        return;
        }

        // dt
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0 || dt > 0.5) return; //avoid massive dt at start

        // compute errors for PID
        for (int i = 0 ; i < 6 ; i++) {
            error[i] = (pos_joint[(i+5)%6] + pos_joint[(i+1)%6])/2 - pos_joint[i]; //compared to mean angle of two adjacent joints)
            cmd[i] = pid[i].compute(error[i],dt);
            }

        // pub to ros2_control
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = {cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]};
        pub_cmd_->publish(cmd_msg);
        }
    };
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<staubli_tx2_60l_controller::StaubliController>());
  rclcpp::shutdown();
  return 0;
}