// Modèle basé sur https://www.sandvik.coromant.com/fr-fr/knowledge/machining-formulas-definitions/drilling-formulas-definitions

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>

namespace simulation_controller {

class DrillForceSimulation : public rclcpp::Node
{
public:
    DrillForceSimulation()
    : rclcpp::Node("drill_force_simulation")
    {
        // parameters
        this->declare_parameter<double>("drill_Dc", 0.006); 
        this->declare_parameter<double>("drill_n", 3000.0); 
        this->declare_parameter<double>("drill_kr", M_PI / 2.0); 
        this->declare_parameter<double>("drill_g0", 25.0 * M_PI / 180.0);
        Dc = this->get_parameter("drill_Dc").as_double();
        n = this->get_parameter("drill_n").as_double();
        kr = this->get_parameter("drill_kr").as_double();
        g0 = this->get_parameter("drill_g0").as_double();
        RCLCPP_INFO(this->get_logger(),
            "Outil  : Dc=%.4f m  n=%.1f tr/min  κr=%.2f°  γ0=%.2f°",
            Dc, n, kr * 180.0 / M_PI, g0 * 180.0 / M_PI);

        this->declare_parameter<double>("mat_Kc1", 600.0); 
        this->declare_parameter<double>("mat_m0", 0.20);
        this->declare_parameter<double>("mat_z0", -0.2);
        this->declare_parameter<double>("mat_zf", -0.4);
        Kc1 = this->get_parameter("mat_Kc1").as_double();
        m0 = this->get_parameter("mat_m0").as_double();
        z0 = this->get_parameter("mat_z0").as_double();
        zf = this->get_parameter("mat_zf").as_double();
        RCLCPP_INFO(this->get_logger(), "Matière: Kc1=%.0f MPa  m0=%.2f", Kc1, m0);

        // pub
        pub_Fz = this->create_publisher<std_msgs::msg::Float64>("/Fz", 10);
        pub_Fc = this->create_publisher<std_msgs::msg::Float64>("/Fc", 10);
        pub_Mz = this->create_publisher<std_msgs::msg::Float64>("/Mz", 10);

        // sub
        sub_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cartesian_state", 10, std::bind(&DrillForceSimulation::poseCb, this, std::placeholders::_1));

        sub_vel = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cartesian_vel", 10, std::bind(&DrillForceSimulation::velCb, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "drill_force_simulation démarré");
    }

private:
    // Publishers / Subscriber
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_Fz;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_Fc;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_Mz;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_vel;

    // Paramètres outil
    double Dc; // Diamètre du foret [m]
    double n; // Vitesse de rotation [tr/min]
    double kr; // Angle d'attaque principal κr [rad] ; 90° = foret standard
    double g0; // Angle de coupe γ0 [rad], influe sur la facilité de coupe

    // Paramètres matériau
    double Kc1; // Kc1 : pression spécifique de coupe pour fz = 1 mm [N/mm² = MPa]
    double m0; // m0 : exposant d'épaisseur de copeau (sensibilité à fz)
    double z0; // z0 [m] plan de début de la plaque
    double zf; //zf [m] plan de fin de la plaque

    double current_z = 0.;

    std_msgs::msg::Float64 msg_;

    void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_z = - msg->pose.position.z;
        }

    void velCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        const double vz = msg->twist.linear.z; // Vitesse d'avance axiale [m/s]

        if (n < 1.0 || vz <= 0.0 || current_z>z0 || zf>current_z) {
            msg_.data = 0.;
            pub_Fz->publish(msg_);
            pub_Fc->publish(msg_);
            pub_Mz->publish(msg_);
            return;
        }
        const double fz = vz / n / 60.; // avance par tour [m/tr]

        const double corr_rake = 1.0 - g0 * 180.0 / M_PI / 100.0; // Correction angle de coupe, terme empirique Sandvik : (1 - γ0 / 100), γ0 en degrés
        const double chip_thickness = fz * std::sin(kr);  // épaisseur copeau effective [m]

        const double Kc = Kc1 * std::pow(chip_thickness, -m0) * corr_rake; // Pression spécifique de coupe Kc [N/m²]
        // Kc = Kc1 * (fz * sin(κr))^(-m0) * (1 * γ0°/100)

        const double vc = M_PI * Dc * n / 60.; // [m/s] Vitesse de coupe
        // vc = π * Dc * n

        const double Pc = vz * vc * Dc * Kc / (240. * n / 60.); // [W]
        // Pc = vz * vc * Dc * Kc / (240 * n)

        const double Mz = (Pc * 30.) / (M_PI * n);  // [N m]
        // Dérivée de P = Mz * ω = Mz * 2π * n/60 -> Mz = 60 * P / (2π * n)

        const double Fc = (2. * Mz) / Dc;  // [N]
        // Fc = 2 * Mz / Dc

        const double Fz = 0.5 * Kc * Dc * fz * std::sin(kr);  // [N]
        // Fz = 0.5 * Kc * Dc * fz * sin(κr)
        // Kc [N/mm²], Dc [m], fz [m/tr] -> Fz [N]

        msg_.data = Fz;
        pub_Fz->publish(msg_);

        msg_.data = Fc;
        pub_Fc->publish(msg_);

        msg_.data = Mz;
        pub_Mz->publish(msg_);
    }
};

} // simulation_controller

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<simulation_controller::DrillForceSimulation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}