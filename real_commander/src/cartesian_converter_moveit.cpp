#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

static const std::string PLANNING_GROUP = "manipulator";
static const std::string EEF_LINK       = "flange";

static const std::vector<double> REF_ANGLES = {
    0.0,
    M_PI / 4.0,
    M_PI / 4.0,
    0.0,
    M_PI / 2.0,
    0.0
};

namespace real_commander {

class CartesianConverter : public rclcpp::Node
{
public:
    CartesianConverter()
    : rclcpp::Node("cartesian_converter")
    {
        pub_pose  = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/cartesian_state", 10);
        pub_twist = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/cartesian_twist", 10);
    }

    void init() {

            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
                this, "move_group");

            RCLCPP_INFO(this->get_logger(), "Waiting for move_group...");
            while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) return;
                RCLCPP_WARN(this->get_logger(), "move_group not available, waiting...");
            }

            // Copier les paramètres robot depuis move_group vers ce nœud
            auto params = parameters_client->get_parameters({
                "robot_description",
                "robot_description_semantic"
            });

            for (auto & p : params) {
                this->declare_parameter(p.get_name(), p.get_parameter_value());
            }

            RCLCPP_INFO(this->get_logger(), "Parameters copied, loading robot model...");

            loader = std::make_shared<robot_model_loader::RobotModelLoader>(
                this->shared_from_this(), "robot_description"); //cras if in constructor
            robot_model = loader->getModel();

        if (!robot_model) {
            RCLCPP_FATAL(this->get_logger(),
                "[robot_description not working]");
            rclcpp::shutdown();
            return;
        }

        robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
        robot_state->setToDefaultValues();
        joint_model_group = robot_model->getJointModelGroup(PLANNING_GROUP);
        if (!joint_model_group) {
            RCLCPP_FATAL(this->get_logger(),
                "Groupe '%s' introuvable dans le SRDF", PLANNING_GROUP.c_str());
            rclcpp::shutdown();
            return;
        }

        // Calcul de la pose de référence (repère local = 0 pour REF_ANGLES)
        robot_state->setJointGroupPositions(joint_model_group, REF_ANGLES);
        robot_state->updateLinkTransforms();
        ref_transform = robot_state->getGlobalLinkTransform(EEF_LINK);
        ref_transform_inv = ref_transform.inverse();

        sub_js = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&CartesianConverter::jointStatesCb, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "cartesian_converter started");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js;

    robot_model_loader::RobotModelLoaderPtr loader;
    moveit::core::RobotModelPtr robot_model;
    moveit::core::RobotStatePtr robot_state;
    const moveit::core::JointModelGroup* joint_model_group{nullptr};

    Eigen::Isometry3d ref_transform;
    Eigen::Isometry3d ref_transform_inv;

    sensor_msgs::msg::JointState prev_js{};
    bool have_prev{false};

    void jointStatesCb(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // positions
        if (msg->position.size() < 6) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "joint_states incomplet");
            return;
        }
        if (!msg->name.empty()) {
            for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
                robot_state->setVariablePosition(msg->name[i], msg->position[i]);
            }
        } else {
            robot_state->setJointGroupPositions(joint_model_group, msg->position);
        }
        robot_state->updateLinkTransforms();

        Eigen::Isometry3d current_tf = robot_state->getGlobalLinkTransform(EEF_LINK);
        Eigen::Isometry3d relative_tf = ref_transform_inv * current_tf;

        //Eigen::Vector3d rpy = relative_tf.rotation().eulerAngles(0, 1, 2); // R-P-Y issue Rz Rx ?

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = msg->header.stamp;
        pose_msg.header.frame_id = "tool_ref_frame";
        auto raw_pose = tf2::toMsg(relative_tf);
        pose_msg.pose.position.x = raw_pose.position.z; //x and z inverted (?)
        pose_msg.pose.position.y = raw_pose.position.y;
        pose_msg.pose.position.z = raw_pose.position.x;
        pub_pose->publish(pose_msg);

        // velocities
        if (!msg->velocity.empty() && msg->velocity.size() >= 6) {
            Eigen::MatrixXd jacobian;
            Eigen::Vector3d ref_point(0, 0, 0);
            robot_state->getJacobian(joint_model_group, robot_model->getLinkModel(EEF_LINK), ref_point, jacobian);

            Eigen::VectorXd dq(6);
            for (int i = 0; i < 6; ++i) dq(i) = msg->velocity[i];

            Eigen::VectorXd v_cart = jacobian * dq; // [vx, vy, vz, wx, wy, wz] //vx <-> vz, wx <-> wz ?

            geometry_msgs::msg::TwistStamped twist_msg;
            twist_msg.header.stamp    = msg->header.stamp;
            twist_msg.header.frame_id = "base_link";
            twist_msg.twist.linear.x  = v_cart(0);
            twist_msg.twist.linear.y  = v_cart(1);
            twist_msg.twist.linear.z  = v_cart(2);
            twist_msg.twist.angular.x = v_cart(3);
            twist_msg.twist.angular.y = v_cart(4);
            twist_msg.twist.angular.z = v_cart(5);

            pub_twist->publish(twist_msg);
        }

        prev_js = *msg;
        have_prev = true;
    }
};

} // namespace real_commander

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<real_commander::CartesianConverter>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}