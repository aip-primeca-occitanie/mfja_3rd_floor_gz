#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

namespace real_commander {

class ForceSimulation : public rclcpp::Node
{
public:
    ForceSimulation()
    : rclcpp::Node("force_simulation")
    {
        this->declare_parameter<double>("z_contact",  -0.4); // [m]
        this->declare_parameter<double>("stiffness",  5000.0); // [N/m]
        this->declare_parameter<double>("damping",    50.0); // [N.s/m]

        z_contact = this->get_parameter("z_contact").as_double();
        k = this->get_parameter("stiffness").as_double();
        c = this->get_parameter("damping").as_double();

        RCLCPP_INFO(this->get_logger(),
            "Parameters : z_contact=%.4f m  k=%.1f N/m  c=%.1f N.s/m",
            z_contact, k, c);

        pub_force = this->create_publisher<std_msgs::msg::Float64>("/Fz", 10);

        sub_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cartesian_state", 10,
            std::bind(&ForceSimulation::poseCb, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "force_simulation started");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_force;
            std_msgs::msg::Float64 fz_msg;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose;

    double z_contact; // [m] altitude de la surface dans ref_frame
    double k; // [N/m]
    double c; // [N.s/m]

    double prev_delta{0.0};
    rclcpp::Time prev_time{0, 0, RCL_ROS_TIME};
    bool have_prev{false};

    void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        const double z = msg->pose.position.z;
        rclcpp::Time now = msg->header.stamp;

        const double delta = z_contact - z; //(si z < z_contact on pénètre la surface)

        double fz = 0.0;

        if (delta > 0.0) { //contact
            double delta_dot = 0.0;

            if (have_prev) {
                const double dt = (now - prev_time).seconds();
                if (dt > 1e-9) {
                    delta_dot = (delta - prev_delta) / dt;
                }
            }

            fz = k * delta + c * delta_dot; //Fz = k*δ + c*dδ/dt

            if (fz < 0.0) fz = 0.0;

            RCLCPP_DEBUG(this->get_logger(),
                "Contact : z=%.4f  δ=%.4f m  dδ/dt=%.4f m/s  Fz=%.2f N",
                z, delta, delta_dot, fz);
        } 

        fz_msg.data = fz;
        pub_force->publish(fz_msg);

        // update prev
        prev_delta = delta;
        prev_time  = now;
        have_prev  = true;
        }
    };
} // namespace real_commander

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<real_commander::ForceSimulation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}