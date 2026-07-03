#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <deque>
#include <vector>
#include <algorithm>

namespace real_commander {

class MedianFilter { //dans le doute, tester les perfs en vrai pour voir si nécessaire
    public:
        explicit MedianFilter(int window) : window(window) {}
        double update(double raw) {
            buf.push_back(raw);
            if (static_cast<int>(buf.size()) > window)
                buf.pop_front();
            std::vector<double> tmp(buf.begin(), buf.end());
            int mid = static_cast<int>(tmp.size()) / 2;
            std::nth_element(tmp.begin(), tmp.begin() + mid, tmp.end());
            return tmp[mid];
            }

    private:
        int window;
        std::deque<double> buf;
    };

class KalmanFilter1D {
    public:
        KalmanFilter1D(double Q, double R)
            : Q(Q), R(R), x(0.0), P(1.0), initialized(false) {}

        double update(double measurement) {
            if (!initialized) {
                x = measurement;
                initialized = true;
                }
            P += Q;
            double K = P / (P + R);
            x += K * (measurement - x);
            P *= (1.0 - K);
            return x;
            }
        void reset() { initialized = false; P = 1.0; }

    private:
        double Q; // bruit de processus ; si la force change vite -> augmenter Q ; si la force est quasi-statique -> diminuer Q
        double R; // bruit de mesure ;mesurer l'écart-type du capteur au repos, puis R = std²
        double x; // état estimé
        double P; // covariance de l'erreur
        bool   initialized;
    };

class ForceFilterNode : public rclcpp::Node {
    public:
        ForceFilterNode()
        : rclcpp::Node("force_filter_node") {
            this->declare_parameter<int> ("median_window", 5);
            this->declare_parameter<double>("kalman_q", 1e-3);
            this->declare_parameter<double>("kalman_r", 0.1);

            int win = this->get_parameter("median_window").as_int();
            double Q = this->get_parameter("kalman_q").as_double();
            double R = this->get_parameter("kalman_r").as_double();

            median_filter = std::make_unique<MedianFilter>(win);
            kalman_filter = std::make_unique<KalmanFilter1D>(Q, R);

            RCLCPP_INFO(this->get_logger(),
                "ForceFilterNode démarré : médian(fenêtre=%d), Kalman(Q=%.1e, R=%.1e)",
                win, Q, R);

            pub = this->create_publisher<std_msgs::msg::Float64>("/Fz", 10);

            sub_fz = this->create_subscription<std_msgs::msg::Float64>(
                "/Fz_raw", rclcpp::SensorDataQoS(), std::bind(&ForceFilterNode::callback, this, std::placeholders::_1));
            }

    private:
        std::unique_ptr<MedianFilter> median_filter;
        std::unique_ptr<KalmanFilter1D> kalman_filter;

        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_fz;
        rclcpp::Publisher  <std_msgs::msg::Float64>::SharedPtr pub;

        void callback(const std_msgs::msg::Float64::SharedPtr msg){

            double fz_median = median_filter->update(msg->data);
            double fz_kalman = kalman_filter->update(fz_median);
            auto out_filtered = std_msgs::msg::Float64();
            out_filtered.data = fz_kalman;
            pub->publish(out_filtered);
            }
    };
} // namespace real_commander

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<real_commander::ForceFilterNode>());
    rclcpp::shutdown();
    return 0;
}