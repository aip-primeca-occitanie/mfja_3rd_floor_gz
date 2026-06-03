#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <fstream>
#include <chrono>

class ForceMomentSubscriber : public rclcpp::Node {
public:
  ForceMomentSubscriber() : Node("force_moment_subscriber") {
    fz_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "/Fz", 10, std::bind(&ForceMomentSubscriber::fz_callback, this, std::placeholders::_1));

    mz_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "/Mz", 10, std::bind(&ForceMomentSubscriber::mz_callback, this, std::placeholders::_1));

    // Ouvre le fichier CSV
    file_.open("data_log.csv");
    file_ << "Time,Fz,Mz\n";  // Entête
    start_time_ = this->now();
  }

  ~ForceMomentSubscriber() {
    if (file_.is_open()) {
      file_.close();
    }
  }

private:
  void fz_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    fz_ = msg->data;
    log_data();  // Essaye de logger si les deux valeurs sont valides
  }

  void mz_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    mz_ = msg->data;
    log_data();  // Essaye de logger si les deux valeurs sont valides
  }

  void log_data() {
    if (fz_.has_value() && mz_.has_value()) {
      double elapsed = (this->now() - start_time_).seconds();
      RCLCPP_INFO(this->get_logger(), "t=%.3f, Fz=%.3f, Mz=%.3f", elapsed, fz_.value(), mz_.value());
      if (file_.is_open()) {
        file_ << elapsed << "," << fz_.value() << "," << mz_.value() << "\n";
        file_.flush();
      }
      fz_.reset();
      mz_.reset();
    }
  }

  //ici attributs privés de la classe. On a besoin que ces valeurs persistent entre les appels callbacks.
  // On peut accéder à ces membres depuis n'importe quelle méthode de la classe
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr fz_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr mz_subscription_;

  std::optional<double> fz_;
  std::optional<double> mz_;

  // Flux de sortie vers un fichier CSV
  // On l’ouvre dans le constructeur et on y écrit les données synchronisées Fz/Mz
  std::ofstream file_;
  rclcpp::Time start_time_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForceMomentSubscriber>());
  rclcpp::shutdown();
  return 0;
}