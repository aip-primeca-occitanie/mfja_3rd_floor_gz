#include <chrono>
#include <future>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
 
using namespace std::chrono_literals;
 
namespace staubli {
 
class PositionCommander : public rclcpp::Node {
public:
    PositionCommander() : rclcpp::Node("position_commander")
    {
        pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/position_controller/joint_trajectory", 10);
 
        sub_js = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr m){ js = *m; });
    }
 
    void runCLI()
    {
        std::string line;
        printBanner();
        while (std::getline(std::cin, line)) {
            if (processLine(line)) break;
            printBanner();
        }
        rclcpp::shutdown();
    }
 
private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js;
    sensor_msgs::msg::JointState js{};

    bool processLine(const std::string & line)
    {
        std::istringstream iss(line);

        int joint_id;
        double value;

        if (!(iss >> joint_id >> value)) {
            std::cout << "Usage: <joint_id 1-6> <value rad>\n";
            return false;
        }

        std::vector<std::string> joint_names = {
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6"
        };
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = joint_names;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        std::vector<double> positions(6, 0.0);
        if (!js.position.empty() && js.position.size() == 6) {
            positions = js.position;
        }
        positions[joint_id - 1] = value;
        point.positions = positions;

        //point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        traj.points.push_back(point);
        pub->publish(traj);

        std::cout << "Sent joint_" << joint_id << " -> " << value << " rad\n";
        return false;
    }
 
    void printBanner() {
        std::cout <<
            "<joint id> <value (rad)>\n";
        }
 
    };
} // namespace staubli_tx2_60l_position_commander
 
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<staubli::PositionCommander>();
    std::thread ros_thread([&node]() { rclcpp::spin(node); });
    node->runCLI();
    if (rclcpp::ok()) rclcpp::shutdown();
    ros_thread.join();
    return 0;
}