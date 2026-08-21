#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <chrono>

using namespace std::chrono_literals;

class PlateMarkerPublisher : public rclcpp::Node
{
public:
    PlateMarkerPublisher() : Node("plate_marker_publisher")
    {
        rclcpp::QoS qos(1);
        qos.transient_local();
        this->declare_parameter<double>("thickness", 0.2);
        thickness = this->get_parameter("thickness").as_double();

        pub_ = this->create_publisher<visualization_msgs::msg::Marker>("plate_marker", qos);
        timer_ = this->create_wall_timer(1s, std::bind(&PlateMarkerPublisher::publish_marker, this));
    }

private:
    double thickness;
    void publish_marker()
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "plate";
        m.header.stamp = this->get_clock()->now();
        m.ns = "plate";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;

        m.pose.position.x = 0.0;
        m.pose.position.y = 0.0;
        m.pose.position.z = 0.0;
        m.pose.orientation.w = 1.0;

        m.scale.x = 0.4;
        m.scale.y = 0.4;
        m.scale.z = thickness;

        m.color.r = 0.6f;
        m.color.g = 0.6f;
        m.color.b = 0.6f;
        m.color.a = 0.8f;

        pub_->publish(m);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlateMarkerPublisher>());
    rclcpp::shutdown();
    return 0;
}