#include <memory>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdometryCovInflator : public rclcpp::Node
{
public:
  OdometryCovInflator()
  : Node("odometry_cov_inflator")
  {
    // Subscriber to local odometry (for conditionals)
    local_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/local", 10,
      std::bind(&OdometryCovInflator::localCallback, this, std::placeholders::_1)
    );

    // Subscriber to GPS odometry whose covariance we want to inflate
    gps_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/gps", 10,
      std::bind(&OdometryCovInflator::gpsCallback, this, std::placeholders::_1)
    );

    // Publisher for modified GPS odometry
    gps_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/odometry/gps_cov", 10
    );

    RCLCPP_INFO(this->get_logger(), "OdometryCovInflator initialized: inflating /odometry/gps covariance with local odometry influence.");
  }

private:
  void localCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(local_mutex_);
    latest_local_ = *msg;
  }

  void gpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Copy input message to modify
    auto out_msg = *msg;

    double extra_x = 0.0;
    double extra_y = 0.0;
    double extra_z = 0.0;
    double extra_yaw = 0.0;

    const double factor_linear = 0.01; // in meters
    const double factor_angular = 0.07; // in meters
    const double inflation = 0.02; // in meters

    const double alpha1 = pow(factor_linear, 2);
    const double alpha2  = pow(factor_angular, 2);
    // Base offset to add to x, y, z pose covariance
    const double base_offset = pow(inflation, 2);

    // Lock and read latest local odometry safely
    {
      std::lock_guard<std::mutex> lk(local_mutex_);
      if (latest_local_.header.stamp.sec != 0 || latest_local_.header.stamp.nanosec != 0)
      {
        const auto & twist = latest_local_.twist.twist;
        double vx = twist.linear.x;
        double angular_z = twist.angular.z;

        if (vx > 0.05) {
          extra_x += vx * alpha1;
        }
        if (std::abs(angular_z) > 0.05) {
          // Influence on yaw covariance
          extra_yaw += angular_z * alpha2;
        }
      }
    }

    // Inflating pose covariance:
    // Odometry pose.covariance layout is 6x6 row-major:
    // indices: x=0, y=7, z=14, roll=21, pitch=28, yaw=35
    if (out_msg.pose.covariance.size() != 36) {
      RCLCPP_WARN(this->get_logger(), "Unexpected pose covariance size (%zu), skipping inflation.", out_msg.pose.covariance.size());
    } else {
      // X
      out_msg.pose.covariance[0] += base_offset + extra_x + extra_yaw;
      // Y
      out_msg.pose.covariance[7] += base_offset + extra_x + extra_yaw;
     
    }

    // Publish modified message
    gps_pub_->publish(out_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gps_pub_;

  // Latest local odometry cache
  nav_msgs::msg::Odometry latest_local_;
  std::mutex local_mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryCovInflator>());
  rclcpp::shutdown();
  return 0;
}
