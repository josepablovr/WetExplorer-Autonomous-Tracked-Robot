#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class NavSatCovNode : public rclcpp::Node
{
public:
  NavSatCovNode()
  : Node("navsat_cov_node")
  {
    // Subscriber to incoming NavSatFix messages
    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/navsat/fix", 10,
      std::bind(&NavSatCovNode::navsatCallback, this, std::placeholders::_1)
    );

    // Publisher for NavSatFix with modified covariance
    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/navsat/fix_cov", 10
    );

    RCLCPP_INFO(this->get_logger(), "NavSatCovNode initialized, republishing to '/navsat/fix_cov'.");
  }

private:
  void navsatCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    // Copy input message
    auto out_msg = *msg;

    // Set the diagonal position covariance matrix [lat, lon, alt]
    // Covariance values: [0.000196, 0.0, 0.0,
    //                    0.0, 0.000196, 0.0,
    //                    0.0, 0.0, 0.000144]
    out_msg.position_covariance = {0.00019600000000000002, 0.0, 0.0,
                                  0.0, 0.00019600000000000002, 0.0,
                                  0.0, 0.0, 0.000144};
    out_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    // Publish modified message
    publisher_->publish(out_msg);
  }
  
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavSatCovNode>());
  rclcpp::shutdown();
  return 0;
}
