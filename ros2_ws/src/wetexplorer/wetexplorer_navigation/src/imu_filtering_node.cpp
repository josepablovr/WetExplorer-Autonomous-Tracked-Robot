// ROS 2 port of the SensorOffsetCorrector node

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include <memory>
#include <cmath>

class SensorOffsetCorrector : public rclcpp::Node {
public:
  SensorOffsetCorrector() : Node("sensor_offset_corrector"), publish_transform(false), acceleration_calibration(false) {
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_transformed", 10,
      std::bind(&SensorOffsetCorrector::imuCallback, this, std::placeholders::_1));

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_filtered", 10);
    imu_publisher2_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/gravity", 10);

    br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    alpha_acc = 0.05;
    alpha_ang_vel = 0.5;
    alpha_grav = 0.3;

    prev_acc = Eigen::Vector3d::Zero();
    prev_ang_vel = Eigen::Vector3d::Zero();
    prev_grav = Eigen::Vector3d::Zero();

    initial_yaw = last_yaw = yaw_drift = yaw_output = 0.0;

    max_acc = 1.0;
    max_vel = 1.0;
    max_acc_grav = 9.81 + max_acc;
    min_acc = 0.01;
    calibration_time = 0.1;
    gravity_magnitude = 0.0;
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!acceleration_calibration) {
      calibrateAcceleration(msg);
      return;
    }

    double roll, pitch, yaw;
    tf2::Quaternion quat;
    tf2::fromMsg(msg->orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    roll = roll + M_PI;
    pitch = -pitch;
    yaw = -yaw;

    if (initial_yaw == 0.0) initial_yaw = yaw;
    yaw -= initial_yaw;
    last_yaw = yaw;
    yaw += yaw_drift;
    yaw_output = yaw;

    tf2::Quaternion corrected_quat;
    corrected_quat.setRPY(roll, pitch, yaw_output);

    sensor_msgs::msg::Imu corrected_imu = *msg;
    corrected_imu.orientation = tf2::toMsg(corrected_quat);

    Eigen::Vector3d grav(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Eigen::Vector3d filtered_grav = lowPassFilter(grav, prev_grav, alpha_grav);    
    filtered_grav = saturateVector(filtered_grav, -max_acc_grav, max_acc_grav);
    
    Eigen::Vector3d g = gravityRemover(roll, pitch);
    
    Eigen::Vector3d acc = filtered_grav - g;
    Eigen::Vector3d filtered_acc = lowPassFilter(acc, prev_acc, alpha_acc);
    filtered_acc = saturateVector(filtered_acc, -max_acc, max_acc);
    filtered_acc = accRejection(filtered_acc);

    Eigen::Vector3d ang_vel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    Eigen::Vector3d filtered_ang_vel = lowPassFilter(ang_vel, prev_ang_vel, alpha_ang_vel);
    filtered_ang_vel = saturateVector(filtered_ang_vel, -max_vel, max_vel);

    prev_acc = filtered_acc;
    prev_ang_vel = filtered_ang_vel;
    prev_grav = filtered_grav;

    corrected_imu.angular_velocity.x = filtered_ang_vel.x();
    corrected_imu.angular_velocity.y = filtered_ang_vel.y();
    corrected_imu.angular_velocity.z = filtered_ang_vel.z();

    corrected_imu.linear_acceleration.x = filtered_acc.x();
    corrected_imu.linear_acceleration.y = filtered_acc.y();
    corrected_imu.linear_acceleration.z = filtered_acc.z();

    imu_publisher_->publish(corrected_imu);

    corrected_imu.linear_acceleration.x = g.x();
    corrected_imu.linear_acceleration.y = g.y();
    corrected_imu.linear_acceleration.z = g.z();

    imu_publisher2_->publish(corrected_imu);

    if (publish_transform) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->now();
      t.header.frame_id = "world";
      t.child_frame_id = "base_link";
      t.transform.rotation = corrected_imu.orientation;
      br_->sendTransform(t);
    }
  }

  Eigen::Vector3d lowPassFilter(const Eigen::Vector3d& new_val, const Eigen::Vector3d& prev_val, double alpha) {
    return prev_val.isZero() ? new_val : alpha * new_val + (1 - alpha) * prev_val;
  }

  Eigen::Vector3d saturateVector(const Eigen::Vector3d& vec, double min_val, double max_val) {
    Eigen::Vector3d result = vec;
    for (int i = 0; i < 3; ++i) {
      result[i] = std::max(std::min(vec[i], max_val), min_val);
    }
    return result;
  }

  Eigen::Vector3d accRejection(const Eigen::Vector3d& acc) {
    Eigen::Vector3d result = acc;
    for (int i = 0; i < 3; ++i) {
      if (fabs(acc[i]) <= min_acc) result[i] = 0.0;
      else if (acc[i] < 0.0) result[i] += min_acc;
      else result[i] -= min_acc;
    }
    return result;
  }

  void calibrateAcceleration(const sensor_msgs::msg::Imu::SharedPtr& msg) {
    if (!init_calib_time_.nanoseconds()) {
      init_calib_time_ = this->now();
    } else {
      rclcpp::Time now = this->now();
      Eigen::Vector3d grav(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
      Eigen::Vector3d filtered_grav = lowPassFilter(grav, prev_grav, 0.001);
      if ((now - init_calib_time_).seconds() >= calibration_time) {
        gravity_magnitude = filtered_grav.norm();
        RCLCPP_INFO(this->get_logger(), "GRAVITY MAGNITUDE: %f", gravity_magnitude);
        acceleration_calibration = true;
      }
    }
  }

  Eigen::Vector3d gravityRemover(double roll, double pitch) {
    Eigen::Vector3d g;
    g.x() = -gravity_magnitude * sin(pitch);
    g.y() = gravity_magnitude * sin(roll) * cos(pitch);
    g.z() = gravity_magnitude * cos(roll) * cos(pitch);
    return g;
  }

  bool publish_transform;
  double alpha_acc, alpha_ang_vel, alpha_grav, max_acc, max_vel, max_acc_grav, min_acc;
  double initial_yaw, yaw_drift, yaw_output, last_yaw, calibration_time, gravity_magnitude;
  bool acceleration_calibration;
  rclcpp::Time init_calib_time_;

  Eigen::Vector3d prev_acc, prev_ang_vel, prev_grav;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_, imu_publisher2_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorOffsetCorrector>());
  rclcpp::shutdown();
  return 0;
}
