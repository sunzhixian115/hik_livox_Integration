#include <rclcpp/rclcpp.hpp>
#include <livox_interfaces/msg/custom_msg.hpp>

class Stamper : public rclcpp::Node {
 public:
  Stamper() : Node("stamper") {
    auto input = declare_parameter("input_topic", "/livox/lidar");
    auto output = declare_parameter("output_topic", "/livox/stamped");
    pub_ = create_publisher<livox_interfaces::msg::CustomMsg>(output, 10);
    sub_ = create_subscription<livox_interfaces::msg::CustomMsg>(input, 10,
      [this](livox_interfaces::msg::CustomMsg::UniquePtr msg) {
        msg->header.stamp = now();
        pub_->publish(std::move(msg));
      });
  }
 private:
  rclcpp::Publisher<livox_interfaces::msg::CustomMsg>::SharedPtr pub_;
  rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr sub_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<Stamper>()); rclcpp::shutdown();
}
