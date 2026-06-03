#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

static const std::string PLANNING_GROUP = "manipulator";
static const std::string EEF_LINK = "flange";

static const std::vector<double> REF_ANGLES = {
    0.0,
    M_PI / 4.0,
    M_PI / 4.0,
    0.0,
    M_PI / 2.0,
    0.0
};

static constexpr double IK_TIMEOUT     = 0.05;   // second

namespace real_commander {

class CartesianPublisher : public rclcpp::Node
{
public:
    CartesianPublisher()
    : rclcpp::Node("cartesian_publisher")
    {
        pub_traj = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_path_command", 10);
    }

    void init(){

        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "move_group");

        RCLCPP_INFO(this->get_logger(), "Waiting for move_group...");
        while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) return;
            RCLCPP_WARN(this->get_logger(), "move_group not available, waiting...");
        }

        // Copier les paramètres robot depuis move_group vers ce nœud
        auto params = parameters_client->get_parameters({"robot_description", "robot_description_semantic"});

        for (auto & p : params) {
            this->declare_parameter(p.get_name(), p.get_parameter_value());
        }

        RCLCPP_INFO(this->get_logger(), "Parameters copied, loading robot model...");

        loader = std::make_shared<robot_model_loader::RobotModelLoader>(
            this->shared_from_this(), "robot_description");
        robot_model = loader->getModel();

        if (!robot_model) {
            RCLCPP_FATAL(this->get_logger(),
                "robot_model not working");
            rclcpp::shutdown();
            return;
            }

        robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
        robot_state->setToDefaultValues();

        joint_model_group = robot_model->getJointModelGroup(PLANNING_GROUP);
        if (!joint_model_group) {
            RCLCPP_FATAL(this->get_logger(),
                "Groupe '%s' introuvable", PLANNING_GROUP.c_str());
            rclcpp::shutdown();
            return;
            }

        // Calcul de la pose de référence (repère local = 0 pour REF_ANGLES)
        robot_state->setJointGroupPositions(joint_model_group, REF_ANGLES);
        robot_state->updateLinkTransforms();
        ref_transform = robot_state->getGlobalLinkTransform(EEF_LINK);

        sub_js = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr m) {last_js = *m;});

        sub_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cartesian_target", 10,
            std::bind(&CartesianPublisher::cartesianTargetCb, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "cartesian_publisher started");
        }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_traj;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js;

    robot_model_loader::RobotModelLoaderPtr loader;
    moveit::core::RobotModelPtr robot_model;
    moveit::core::RobotStatePtr robot_state;
    const moveit::core::JointModelGroup* joint_model_group{nullptr};

    Eigen::Isometry3d ref_transform;
    sensor_msgs::msg::JointState last_js{};

    void cartesianTargetCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
    geometry_msgs::msg::Pose corrected_pose = msg->pose;
    corrected_pose.position.x = msg->pose.position.z;
    corrected_pose.position.z = msg->pose.position.x;

    Eigen::Isometry3d pose_relative;
    tf2::fromMsg(corrected_pose, pose_relative);
    Eigen::Isometry3d pose_base = ref_transform * pose_relative;

        if (!last_js.position.empty() && last_js.position.size() >= 6) { //to converge the closest to the current position
            if (!last_js.name.empty()) {
                for (size_t i = 0; i < last_js.name.size(); ++i)
                    robot_state->setVariablePosition(last_js.name[i],last_js.position[i]);
            } else {
                robot_state->setJointGroupPositions(joint_model_group, last_js.position);
                }
            robot_state->updateLinkTransforms();
            }

        bool ik_found = robot_state->setFromIK(joint_model_group,pose_base,EEF_LINK,IK_TIMEOUT);
        if (!ik_found) {
            RCLCPP_WARN(this->get_logger(),
                "cannot reach the required target (IK failed)");
            return;
            }
        std::vector<double> joint_positions;
        robot_state->copyJointGroupPositions(joint_model_group, joint_positions);

        std::vector<std::string> joint_names = {
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6"
            };
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = msg->header.stamp;
        traj.header.frame_id = "base_link";
        traj.joint_names = joint_names;
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joint_positions;
        //point.time_from_start  = rclcpp::Duration::from_seconds(1.0);
        traj.points.push_back(point);
        pub_traj->publish(traj);
        }
    };

} //namespace real_commander

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<real_commander::CartesianPublisher>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}