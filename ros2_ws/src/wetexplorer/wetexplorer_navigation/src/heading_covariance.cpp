#include <memory>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuCovInflator : public rclcpp::Node
{
public:
  ImuCovInflator()
  : Node("imu_cov_inflator")
  {
    local_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/local", 10,
      std::bind(&ImuCovInflator::localCallback, this, std::placeholders::_1)
    );

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/gps/navheading", 10,
      std::bind(&ImuCovInflator::imuCallback, this, std::placeholders::_1)
    );

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "/imu/heading_cov", 10
    );

    RCLCPP_INFO(this->get_logger(), "ImuCovInflator initialized: applying base yaw covariance offset and conditional inflation from local odometry angular.z.");
  }

private:
  void localCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(local_mutex_);
    latest_local_ = *msg;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto out_msg = *msg;

    // Base yaw covariance offset

    const double deg_offset = 1.0;
    const double deg_extra = 5.0;


    const double base_yaw_offset = pow((deg_offset*3.14/180),2);
    const double yaw_factor = pow((deg_extra*3.14/180),2);


    double extra_yaw = 0.0;


    

    {
      std::lock_guard<std::mutex> lk(local_mutex_);
      if (latest_local_.header.stamp.sec != 0 || latest_local_.header.stamp.nanosec != 0)
      {
        double angular_z = latest_local_.twist.twist.angular.z;
        if (std::abs(angular_z) > 0.05) {
          extra_yaw += std::abs(angular_z) * yaw_factor;
        }
      }
    }

    // orientation_covariance is a 3x3 row-major array; yaw variance is element [2][2] -> index 8
    if (out_msg.orientation_covariance.size() != 9) {
      RCLCPP_WARN(this->get_logger(), "Unexpected orientation covariance size (%zu), skipping inflation.", out_msg.orientation_covariance.size());
    } else {
      out_msg.orientation_covariance[8] += base_yaw_offset + extra_yaw;
    }

    imu_pub_->publish(out_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  nav_msgs::msg::Odometry latest_local_;
  std::mutex local_mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuCovInflator>());
  rclcpp::shutdown();
  return 0;
}
