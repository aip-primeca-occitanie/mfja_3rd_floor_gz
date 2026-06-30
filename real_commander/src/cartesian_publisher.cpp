#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdio> 

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp> 

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

static constexpr double IK_TIMEOUT = 0.05; // seconds
static constexpr double IK_TRESHOLD = 1e-4;
static constexpr int IK_MAX_ITER = 1000;
static constexpr double IK_DT = 1e-1;
static constexpr double IK_DAMP = 1e-2;
static constexpr double IK_NULL_GAIN = 0.1;  

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
            RCLCPP_FATAL(this->get_logger(), "Frame '%s' not found", EEF_LINK.c_str());
            rclcpp::shutdown();
            return;
            }
        eef_frame_id = model.getFrameId(EEF_LINK);

        // compute reference (position = 0)
        Eigen::VectorXd q_ref = Eigen::Map<const Eigen::VectorXd>(REF_ANGLES.data(), REF_ANGLES.size());
        pinocchio::forwardKinematics(model, *data, q_ref);
        pinocchio::framesForwardKinematics(model, *data, q_ref); //?
        ref_transform = data->oMf[eef_frame_id];
        q_current = q_ref;

        //sub
        sub_js = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr m) {last_js = *m;});

        sub_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cartesian_target", 10,
            std::bind(&CartesianPublisher::cartesianTargetCb, this, std::placeholders::_1));
        
        //move to initial position (simulation only ?)
        rclcpp::sleep_for(std::chrono::seconds(2));
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

        RCLCPP_INFO(this->get_logger(),
            "cartesian_publisher started");
        }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_traj;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js;

    pinocchio::Model model;
    std::unique_ptr<pinocchio::Data> data;
    pinocchio::FrameIndex eef_frame_id;
    pinocchio::SE3 ref_transform;

    Eigen::VectorXd q_current;

    sensor_msgs::msg::JointState last_js{};

    bool solveIK(const pinocchio::SE3 & target_se3, Eigen::VectorXd & q_out)
    {
        const int nv = model.nv;
        Eigen::VectorXd q = q_current;   // seed = current state

        for (int iter = 0; iter < IK_MAX_ITER; ++iter)
        {
            Eigen::VectorXd q_ref = Eigen::Map<const Eigen::VectorXd>(REF_ANGLES.data(), REF_ANGLES.size());

            pinocchio::forwardKinematics(model, *data, q);
            pinocchio::framesForwardKinematics(model, *data, q);

            const pinocchio::SE3 & current_se3 = data->oMf[eef_frame_id];

            pinocchio::Motion err = pinocchio::log6(current_se3.actInv(target_se3));
            Eigen::Matrix<double,6,1> err_vec = err.toVector();

            if (err_vec.norm() < IK_TRESHOLD) {
                q_out = q;
                return true;
            }

            pinocchio::Data::Matrix6x J(6, nv);
            J.setZero();
            pinocchio::computeFrameJacobian(model, *data, q, eef_frame_id, pinocchio::ReferenceFrame::LOCAL, J);

            Eigen::MatrixXd JJt = J * J.transpose(); //Jpinv(q)*(ds-lambda e) + (I - JpinvJ)p
            JJt.diagonal().array() += IK_DAMP;
            Eigen::MatrixXd J_pinv = J.transpose() * JJt.inverse();

            Eigen::VectorXd dq_task = J_pinv * err_vec; //ds = 0 ?

            /*
            Eigen::VectorXd p = IK_NULL_GAIN * (q_ref - q); //p=?
            Eigen::MatrixXd null_proj = Eigen::MatrixXd::Identity(nv, nv) - J_pinv * J;
            Eigen::VectorXd dq_null = null_proj * p;
            */

            Eigen::VectorXd dq = dq_task; //+ dq_null;
            q = pinocchio::integrate(model, q, IK_DT * dq);
        }
        return false; // did not converge
    }

    void cartesianTargetCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        geometry_msgs::msg::Pose corrected_pose = msg->pose;
        corrected_pose.position.x = msg->pose.position.z;
        corrected_pose.position.z = msg->pose.position.x; //inverted ?

        Eigen::Isometry3d pose_relative;
        tf2::fromMsg(corrected_pose, pose_relative);
        
        Eigen::Isometry3d ref_eigen;
        ref_eigen.matrix() = ref_transform.toHomogeneousMatrix();
        Eigen::Isometry3d pose_base_eigen = ref_eigen * pose_relative;

        pinocchio::SE3 pose_base(pose_base_eigen.rotation(),pose_base_eigen.translation());

        if (!last_js.position.empty() && last_js.position.size() >= 6) { //to converge the closest to the current position

            if (!last_js.name.empty()) {
                for (int i = 1; i < model.nq; ++i) {   // i=0 = universe
                    const std::string & jname = model.names[i];
                    auto it = std::find(last_js.name.begin(), last_js.name.end(), jname);
                    if (it != last_js.name.end()) {
                        size_t idx = std::distance(last_js.name.begin(), it);
                        q_current[i - 1] = last_js.position[idx];
                        }
                    }
            } else {
                q_current = Eigen::Map<const Eigen::VectorXd>(
                    last_js.position.data(), model.nv);
                }
            }
        Eigen::VectorXd q_solution;
        bool ik_found = solveIK(pose_base, q_solution);
        if (!ik_found) {
            RCLCPP_WARN(this->get_logger(),
                "cannot reach the required target (IK failed)");
            return;
            }
        
        q_current = q_solution;

        std::vector<std::string> joint_names = {
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6"
            };
        
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = msg->header.stamp;
        traj.header.frame_id = "base_link";
        traj.joint_names = joint_names;
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.assign(q_solution.data(), q_solution.data() + q_solution.size());
        point.time_from_start  = rclcpp::Duration::from_seconds(0.005);
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