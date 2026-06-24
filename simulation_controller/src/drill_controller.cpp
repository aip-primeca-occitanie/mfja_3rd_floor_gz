#include <chrono>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

#include "staubli_tx2_60l_controller/pid.hpp"

using namespace std::chrono_literals;

namespace staubli_tx2_60l_controller {

class DrillPIDController : public rclcpp::Node {
public:
    enum ContactState {
        NO_CONTACT = 0,
        GOING_TO_CONTACT,
        ACTIVE_CONTACT,
        RELEASING_CONTACT
        };

    DrillPIDController() : rclcpp::Node("force_pid_controller")
    {
        this->declare_parameter<double>("KP0", 1e-4);
        this->declare_parameter<double>("KP1", 1e-3);
        this->declare_parameter<double>("KI1", 1e-4);
        this->declare_parameter<double>("KD1", 2e-3);
        this->declare_parameter<double>("MAX_OUTPUT", 0.05);
        this->declare_parameter<double>("TARGET", 300.0);
        this->declare_parameter<double>("FORCE_THRESHOLD", 5.0);

        const double KP0 = this->get_parameter("KP0").as_double();
        const double KP1 = this->get_parameter("KP1").as_double();
        const double KI1 = this->get_parameter("KI1").as_double();
        const double KD1 = this->get_parameter("KD1").as_double();
        const double MAX_OUTPUT = this->get_parameter("MAX_OUTPUT").as_double();
        target_force = this->get_parameter("TARGET").as_double();
        force_threshold = this->get_parameter("FORCE_THRESHOLD").as_double();

        PIDGains g0;
        g0.kp = KP0; g0.ki = 0.0; g0.kd = 0.0; g0.max_output = MAX_OUTPUT;
        pid0 = PIDController(g0);

        PIDGains g1;
        g1.kp = KP1; g1.ki = KI1; g1.kd = KD1; g1.max_output = MAX_OUTPUT;
        pid1 = PIDController(g1);

        sub_force = this->create_subscription<std_msgs::msg::Float64>(
            "/Fz", 10,
            std::bind(&DrillPIDController::onForce, this, std::placeholders::_1));

        sub_cartesian_state = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cartesian_state", 10,
            std::bind(&DrillPIDController::onCartesianState, this, std::placeholders::_1));

        sub_target_force = this->create_subscription<std_msgs::msg::Float64>(
            "/target_force", 10,
            std::bind(&DrillPIDController::onTargetForce, this, std::placeholders::_1));

        pub_cartesian_target = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/cartesian_target", 10);

        int rate_hz = 250;
        auto period = std::chrono::milliseconds(1000 / rate_hz);
        control_timer = this->create_wall_timer(period, std::bind(&DrillPIDController::controlLoop, this));
        last_time = this->now();

        RCLCPP_INFO(this->get_logger(), "ForcePIDController started");
        }

private:
    PIDController pid0;
    PIDController pid1;

    double current_force = 0.0;
    double target_force;
    double prev_target_force = 0.0;
    double force_threshold;

    double current_z = 0.0;
    double current_x = 0.0;
    double current_y = 0.0;

    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;

    bool force_received = false;
    bool cartesian_received = false;

    ContactState contactState_ = NO_CONTACT;
    std::size_t contactCounter_ = 0;
    static constexpr std::size_t nContactIterations_ = 3;

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
        current_z = - msg->pose.position.z;
        qx = msg->pose.orientation.x;
        qy = msg->pose.orientation.y;
        qz = msg->pose.orientation.z;
        qw = msg->pose.orientation.w;
        cartesian_received = true;
        }

    void onTargetForce(const std_msgs::msg::Float64::SharedPtr msg) {
        double new_target = msg->data;
        if (new_target != prev_target_force) {
            pid1.reset();
            RCLCPP_INFO(this->get_logger(), "Target force changed %.3f -> %.3f", prev_target_force, new_target);
            prev_target_force = new_target;
            }
        target_force = new_target;
        }

    void controlLoop() {
        if (!force_received || !cartesian_received) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "waiting for topics...");
            return;
            }

        auto now = this->now();
        double dt = (now - last_time).seconds();
        last_time = now;
        if (dt <= 0.0 || dt > 0.5) return;

        // state machine (inspired by 2022 CNRS ContactAdmittance https://github.com/agimus/agimus-sot/blob/devel/src/contact-admittance.cc)
        bool above_threshold = (current_force >= force_threshold);
        double force_error;
        double delta_z = 0.0;

        switch (contactState_) {
            case NO_CONTACT:{
                if (above_threshold) {
                    contactCounter_++;
                    if (contactCounter_ >= nContactIterations_) {
                        contactState_ = GOING_TO_CONTACT;
                        contactCounter_ = 0;
                        pid1.reset();
                        RCLCPP_INFO(this->get_logger(), "-> GOING_TO_CONTACT");
                        }
                } else {
                    contactCounter_ = 0;
                    }
                // slow proportional approach
                double approach_error = force_threshold - current_force;
                delta_z = - pid0.compute(approach_error, dt);
                break;}

            case GOING_TO_CONTACT:{
                if (above_threshold) {
                    contactCounter_++;
                    if (contactCounter_ >= nContactIterations_) {
                        contactState_ = ACTIVE_CONTACT;
                        contactCounter_ = 0;
                        RCLCPP_INFO(this->get_logger(), "-> ACTIVE_CONTACT");
                    }
                } else {
                    contactCounter_ = 0;
                    contactState_ = NO_CONTACT;
                    pid0.reset();
                    RCLCPP_INFO(this->get_logger(), "-> NO_CONTACT");
                }
                force_error = current_force - target_force;
                delta_z = - pid1.compute(force_error, dt);
                break;}

            case ACTIVE_CONTACT:{
                if (!above_threshold) {
                    contactCounter_++;
                    if (contactCounter_ >= nContactIterations_) {
                        contactState_ = RELEASING_CONTACT;
                        contactCounter_ = 0;
                        RCLCPP_INFO(this->get_logger(), "-> RELEASING_CONTACT");
                    }
                } else {
                    contactCounter_ = 0;
                }
                force_error = current_force - target_force;
                delta_z = - pid1.compute(force_error, dt);
                break;}

            case RELEASING_CONTACT:{
                if (above_threshold) {
                    contactState_ = ACTIVE_CONTACT;
                    contactCounter_ = 0;
                    RCLCPP_INFO(this->get_logger(), "-> ACTIVE_CONTACT");
                }
                break;}
            }
        
        geometry_msgs::msg::PoseStamped target;
        target.header.stamp = now;
        target.header.frame_id = "base_link";
        if (contactState_ == RELEASING_CONTACT) { //retour à la position initiale
            target.pose.position.x = 0.;
            target.pose.position.y = 0.;
            target.pose.position.z = 0.;
            target.pose.orientation.x = 0.;
            target.pose.orientation.y = 0.;
            target.pose.orientation.z = 0.;
            target.pose.orientation.w = 1.;
        } else {
            target.pose.position.x = current_x;
            target.pose.position.y = current_y;
            target.pose.position.z = - (current_z + delta_z);
            target.pose.orientation.x = qx;
            target.pose.orientation.y = qy;
            target.pose.orientation.z = qz;
            target.pose.orientation.w = qw;
            }

        pub_cartesian_target->publish(target);
        }
    };

} // namespace simulation_controller

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<staubli_tx2_60l_controller::DrillPIDController>());
    rclcpp::shutdown();
    return 0;
}