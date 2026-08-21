#include <chrono>
#include <string>
#include <cmath>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "std_msgs/msg/float64.hpp"

#include "staubli_tx2_60l_controller/pid.hpp"

static const std::vector<double> REF_ANGLES = {
    0.0,
    M_PI / 4.0,
    M_PI / 4.0,
    0.0,
    M_PI / 2.0,
    0.0
    };

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
        this->declare_parameter<int>("FREQ_CONTROL", 80);
        this->declare_parameter<double>("MAX_VELOCITY", 0.001);

        const double KP0 = this->get_parameter("KP0").as_double();
        const double KP1 = this->get_parameter("KP1").as_double();
        const double KI1 = this->get_parameter("KI1").as_double();
        const double KD1 = this->get_parameter("KD1").as_double();
        const double MAX_OUTPUT = this->get_parameter("MAX_OUTPUT").as_double();
        max_velocity = this->get_parameter("MAX_VELOCITY").as_double();
        target_force = this->get_parameter("TARGET").as_double();
        force_threshold = this->get_parameter("FORCE_THRESHOLD").as_double();
        const int FREQ_CONTROL = this->get_parameter("FREQ_CONTROL").as_int();
        
        kp1=KP1;

        pub_traj = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/position_controller/joint_trajectory", 10);

        PIDGains g0;
        g0.kp = KP0; g0.ki = 0.0; g0.kd = 0.0; g0.max_output = MAX_OUTPUT;
        pid0 = PIDController(g0);

        PIDGains g1;
        g1.kp = KP1; g1.ki = KI1; g1.kd = KD1; g1.max_output = max_velocity;
        pid1 = PIDController(g1);

        sub_force = this->create_subscription<std_msgs::msg::Float64>(
            "/Fz", 10,
            std::bind(&DrillPIDController::onForce, this, std::placeholders::_1));

        sub_cartesian_state = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cartesian_state", 10,
            std::bind(&DrillPIDController::onCartesianState, this, std::placeholders::_1));

        sub_cartesian_vel = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cartesian_vel", 10,
            std::bind(&DrillPIDController::onCartesianVel, this, std::placeholders::_1));

        sub_target_force = this->create_subscription<std_msgs::msg::Float64>(
            "/target_force", 10,
            std::bind(&DrillPIDController::onTargetForce, this, std::placeholders::_1));

        pub_cartesian_target = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/cartesian_target", 10);

        auto period = std::chrono::milliseconds(1000 / FREQ_CONTROL);
        control_timer = this->create_wall_timer(period, std::bind(&DrillPIDController::controlLoop, this));
        last_time = this->now();

        RCLCPP_INFO(this->get_logger(), "ForcePIDController started");
        }

private:
    PIDController pid0;
    PIDController pid1;
    double kp1;

    double current_force = 0.0;
    double target_force;
    double prev_target_force = 0.0;
    double force_threshold;

    double current_z = 0.0;
    double current_x = 0.0;
    double current_y = 0.0;

    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;

    double drill_velocity;

    double max_velocity;
    double command_velocity;

    bool force_received = false;
    bool cartesian_received = false;

    ContactState contactState_ = NO_CONTACT;
    std::size_t contactCounter_ = 0;
    static constexpr std::size_t nContactIterations_ = 3;

    rclcpp::Time last_time;
    rclcpp::TimerBase::SharedPtr control_timer;

    static constexpr std::size_t nDecelIterations_ = 20;
    double decel_start_velocity_ = 0.05;
    std::size_t decelCounter_ = 0;
    double drill_start_velocity_ = 0.001; // m/s

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_force;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_cartesian_state;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cartesian_vel;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_target_force;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cartesian_target;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_traj;

    void onForce(const std_msgs::msg::Float64::SharedPtr msg) {
        current_force = msg->data;
        force_received = true;
        }

    void onCartesianState(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_x = msg->pose.position.x;
        current_y = msg->pose.position.y;
        current_z = msg->pose.position.z;
        qx = msg->pose.orientation.x;
        qy = msg->pose.orientation.y;
        qz = msg->pose.orientation.z;
        qw = msg->pose.orientation.w;
        cartesian_received = true;
        }
    
    void onCartesianVel(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        drill_velocity=msg->twist.linear.z;
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
        RCLCPP_INFO(this->get_logger(), "control, vz : %.5f",drill_velocity);
        RCLCPP_INFO(this->get_logger(), "control, Fz: %.3f",current_force);
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
                double approach_error = current_force - force_threshold;
                delta_z = std::abs(pid0.compute(approach_error, dt));
                break;}

            case GOING_TO_CONTACT:{
                if (above_threshold) {
                    contactCounter_++;
                    if (contactCounter_ == 1) {
                        //decel_start_velocity_ = std::max(std::abs(drill_velocity), drill_start_velocity_);
                        command_velocity = drill_start_velocity_;
                        decelCounter_ = 0;
                    }
                    if (contactCounter_ >= nContactIterations_) {
                        /*decelCounter_++;
                        //double ramp = std::min(1.0, static_cast<double>(decelCounter_) / nDecelIterations_);
                        //command_velocity = decel_start_velocity_ + (drill_start_velocity_ - decel_start_velocity_) * ramp;
                        if (ramp >= 1.0) {*/
                        command_velocity = drill_velocity;
                        contactState_ = ACTIVE_CONTACT;
                        contactCounter_ = 0;
                        RCLCPP_INFO(this->get_logger(), "-> ACTIVE_CONTACT");
                        //}
                    //} else {
                    //    command_velocity = decel_start_velocity_;
                    }
                } else {
                    contactCounter_ = 0;
                    //decelCounter_ = 0;
                    contactState_ = NO_CONTACT;
                    pid0.reset();
                    RCLCPP_INFO(this->get_logger(), "-> NO_CONTACT");
                }
                delta_z = command_velocity * dt;
                break;
            }

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
                // en regime etabli (force_error ~ 0), la correction de pid1 tend vers 0 :
                // drill_velocity n'est alors plus modifiee (dv=0) et reste a sa derniere
                // valeur -> c'est ca qui donne le "dz/dt constant" recherche, explicitement
                // porte par cet etat plutot que par un mecanisme interne a pid1.
                force_error = current_force - target_force;
                double dv_force = std::clamp(kp1 * force_error * dt,-max_velocity,max_velocity); //pid1.compute(force_error, dt)) * dt;
                RCLCPP_INFO(this->get_logger(), "pid dv : %.5f",dv_force);
                //double dv_tracking = 0.01 * (drill_velocity - command_velocity) * dt;
                double dv = dv_force ;//+ dv_tracking; (permet d'aider la )
                double v_start = command_velocity;
                command_velocity += dv; //vitesse virtuelle, vk (utilisée pour garder un modele cohérent pour tout le calcul, remplacer par la vraie vitesse ne semble pas fonctionner)
                RCLCPP_INFO(this->get_logger(), "pid vk : %.5f",command_velocity);
                command_velocity = std::clamp(command_velocity, -max_velocity, max_velocity);

                delta_z = - (v_start * dt + 0.5 * dv * dt); //zk+1 = zk + dz = zk + vk dt + 1/2 * dv * dt²
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
            rclcpp::sleep_for(std::chrono::seconds(1));
            trajectory_msgs::msg::JointTrajectory traj;
            traj.header.stamp = this->now();
            traj.header.frame_id = "base_link";
            traj.joint_names = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = REF_ANGLES;
            point.time_from_start = rclcpp::Duration::from_seconds(3.0);
            traj.points.push_back(point);
            pub_traj->publish(traj);
            RCLCPP_INFO(this->get_logger(), "Moving to initial position...");
        } else {
            target.pose.position.x = current_x;
            target.pose.position.y = current_y;
            target.pose.position.z = current_z - delta_z;
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