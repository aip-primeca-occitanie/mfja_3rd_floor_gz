#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdio> 

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

static const std::string EEF_LINK = "flange";

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
        pub_vel = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/cartesian_vel", 10);
        }

    void init() {

        //load urdf
        std::string pkg_path = ament_index_cpp::get_package_share_directory(
            "staubli_tx2_60l_moveit_config");
        std::string xacro_path = pkg_path + "/config/staubli_tx2_60l.urdf.xacro";
        std::string urdf_string;
        FILE* pipe = popen(("xacro " + xacro_path).c_str(), "r");
        if (!pipe) {
            RCLCPP_FATAL(this->get_logger(), "issue with xacro");
            rclcpp::shutdown();
            return;
        }
        char buf[256];
        while (fgets(buf, sizeof(buf), pipe)) urdf_string += buf;
        pclose(pipe);
        try {
            pinocchio::urdf::buildModelFromXML(urdf_string, model);
        } catch (const std::exception & e) {
            RCLCPP_FATAL(this->get_logger(), "urdf : %s", e.what());
            rclcpp::shutdown();
            return;
            }

        data = std::make_unique<pinocchio::Data>(model);

        if (!model.existFrame(EEF_LINK)) {
            RCLCPP_FATAL(this->get_logger(), "Frame '%s' unfound.", EEF_LINK.c_str());
            rclcpp::shutdown();
            return;
            }
        eef_frame_id = model.getFrameId(EEF_LINK);

        // compute reference (position = 0)
        Eigen::VectorXd q_ref = Eigen::Map<const Eigen::VectorXd>(REF_ANGLES.data(), REF_ANGLES.size());
        pinocchio::forwardKinematics(model, *data, q_ref);
        pinocchio::framesForwardKinematics(model, *data, q_ref);
        ref_transform = data->oMf[eef_frame_id];
        ref_transform_inv = ref_transform.inverse();
        
        //sub
        sub_js = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&CartesianConverter::jointStatesCb, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "cartesian_converter started");
        }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_vel;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js;

    pinocchio::Model model;
    std::unique_ptr<pinocchio::Data> data;
    pinocchio::FrameIndex eef_frame_id;
    pinocchio::SE3 ref_transform;
    pinocchio::SE3 ref_transform_inv;

    sensor_msgs::msg::JointState prev_js{};
    bool have_prev{false};

    void jointStatesCb(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // positions
        Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(msg->position.data(), model.nv);

        pinocchio::forwardKinematics(model, *data, q);
        pinocchio::framesForwardKinematics(model, *data, q);

        const pinocchio::SE3 & current_se3 = data->oMf[eef_frame_id];
        pinocchio::SE3 relative_se3 = ref_transform_inv * current_se3;

        Eigen::Isometry3d relative_eigen;
        relative_eigen.matrix() = relative_se3.toHomogeneousMatrix();

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp    = msg->header.stamp;
        pose_msg.header.frame_id = "tool_ref_frame";
        auto raw_pose = tf2::toMsg(relative_eigen);
        pose_msg.pose.position.x = raw_pose.position.x; //x and z might be inverted
        pose_msg.pose.position.y = raw_pose.position.y;
        pose_msg.pose.position.z = raw_pose.position.z;
        //pose_msg.pose.orientation = raw_pose.orientation;
        pub_pose->publish(pose_msg);

        // velocities
        if (!msg->velocity.empty() && (int)msg->velocity.size() >= model.nv) {

            pinocchio::Data::Matrix6x J(6, model.nv);
            J.setZero();
            pinocchio::computeJointJacobians(model, *data, q);
            pinocchio::getJointJacobian(model, *data, model.frames[eef_frame_id].parent, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

            Eigen::VectorXd dq = Eigen::Map<const Eigen::VectorXd>(msg->velocity.data(), model.nv);

            Eigen::VectorXd v_cart = J * dq; // [vx, vy, vz, wx, wy, wz] //vx <-> vz, wx <-> wz ?

            geometry_msgs::msg::TwistStamped vel_msg;
            vel_msg.header.stamp    = msg->header.stamp;
            vel_msg.header.frame_id = "base_link";
            vel_msg.twist.linear.x  = v_cart(0);
            vel_msg.twist.linear.y  = v_cart(1);
            vel_msg.twist.linear.z  = v_cart(2);
            vel_msg.twist.angular.x = v_cart(3);
            vel_msg.twist.angular.y = v_cart(4);
            vel_msg.twist.angular.z = v_cart(5);
            pub_vel->publish(vel_msg);
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