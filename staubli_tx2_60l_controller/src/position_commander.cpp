#include <chrono>
#include <future>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
 
using namespace std::chrono_literals;
using SwitchController = controller_manager_msgs::srv::SwitchController;
 
namespace spring_robots {
 
class PositionCommander : public rclcpp::Node {
public:
    PositionCommander() : rclcpp::Node("position_commander")
    {
        pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/position_controller/joint_trajectory", 10);
 
        sub_js = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr m){ js = *m; });

        sw = this->create_client<SwitchController>(
            "/controller_manager/switch_controller");
    }
 
    void runCLI()
    {
        std::cout << "Waiting for controller_managers... ";
        sw->wait_for_service();
        std::cout << "OK\n";

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
    rclcpp::Client<SwitchController>::SharedPtr sw;
    sensor_msgs::msg::JointState js{};

    bool doSwitch(const std::string & activate,
                  const std::string & deactivate)
    {
        auto & client = sw;
        auto req = std::make_shared<SwitchController::Request>();
        req->activate_controllers   = {activate};
        req->deactivate_controllers = {deactivate};
        req->strictness  = SwitchController::Request::BEST_EFFORT;
        req->activate_asap = true;
        req->timeout = rclcpp::Duration::from_seconds(2.0);
 
        auto future = client->async_send_request(req);
 
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
        while (rclcpp::ok()) {
            if (future.wait_for(20ms) == std::future_status::ready) break;
            if (std::chrono::steady_clock::now() > deadline) {
                std::cout << "X timeout\n"; return false;
            }
            std::this_thread::sleep_for(20ms);
        }
        return true;
    }


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
        positions[joint_id - 1] = std::fmod(value, 2.0*M_PI);
        point.positions = positions;

        std::cout << "  [1/4] switching velocity -> position ...\n";
        if (!doSwitch("position_controller", "velocity_controller")) {
            std::cout << "quitting...\n"; return false;
            }
        std::cout << " OK\n";
        
        std::cout << "  [2/4] publishing ..."; std::cout.flush();
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        traj.points.push_back(point);
        pub->publish(traj);
        std::cout << " OK\n";

        std::cout << "  [3/4] going to position ..."; std::cout.flush();
        double tol = 0.001; // tolérance en radians
        bool reached = false;
        while (rclcpp::ok() && !reached) {
            reached = true;
            if (std::abs(js.position[joint_id-1] - value) > tol) {
                reached = false;
                }
            if (!reached) std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << " OK\n";

        std::cout << "  [4/4] switching position -> velocity ..."; std::cout.flush();
        if (!doSwitch("velocity_controller", "position_controller"))
            std::cout << "failed\n";
        else
            std::cout << "OK\n";

        std::cout << "Sent joint_" << joint_id << " -> " << value << " rad\n";
        return false;
    }
 
    void printBanner() {
        std::cout <<
            "<joint id> <value (rad)>\n";
        }
 
    };
} // namespace staubli_tx2_60l_controller
 
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<spring_robots::PositionCommander>();
    std::thread ros_thread([&node]() { rclcpp::spin(node); });
    node->runCLI();
    if (rclcpp::ok()) rclcpp::shutdown();
    ros_thread.join();
    return 0;
}