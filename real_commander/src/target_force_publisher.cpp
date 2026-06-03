#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

namespace real_commander {

class TargetForcePublisher : public rclcpp::Node {
public:
    TargetForcePublisher() : rclcpp::Node("target_force_publisher")
    {
        pub = this->create_publisher<std_msgs::msg::Float64>("/target_force", 10);
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
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub;
    double current_target = 0.0;

    bool processLine(const std::string & line)
    {
        std::istringstream iss(line);
        double fz;
        if (!(iss >> fz)) {
            std::cout << "something wrong happened\n";
            return false;
        }

        if (fz < 0.0) {
            fz = 0.0;
        }
        current_target = fz;

        std_msgs::msg::Float64 msg;
        msg.data = current_target;
        pub->publish(msg);

        std::cout << "target_force published : " << current_target << " N\n";
        return false;
    }

    void printBanner() {
        std::cout << "\ninput the target Fz in Newton : \n";
    }
};

} // namespace real_commander

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<real_commander::TargetForcePublisher>();
    std::thread ros_thread([&node]() {rclcpp::spin(node);});
    node->runCLI();
    if (rclcpp::ok()) rclcpp::shutdown();
    ros_thread.join();
    return 0;
}