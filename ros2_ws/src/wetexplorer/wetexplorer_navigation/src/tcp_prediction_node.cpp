#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <string>
#include <memory>
#include <cmath>

class TcpPredictionNode : public rclcpp::Node
{
public:
  TcpPredictionNode()
  : rclcpp::Node("tcp_prediction_node")
  {

   
    // --- Geometry and limits (defaults). You can also set them as ROS params.
    t1x_ = this->declare_parameter<double>("t1x", 0.53912 );   // fixed offset x (m)
    t1z_ = this->declare_parameter<double>("t1z", 0.45488);   // fixed offset z (m)
    t2_  = this->declare_parameter<double>("t2",  0.30);   // EE offset along +x of last joint (m)

    // Prismatic joint along +z of base_link (constant, inside class as requested)
    qz_  = this->declare_parameter<double>("qz",  -0.18);   // meters

    // Angle limits (deg) → radians
    double max_roll_deg  = this->declare_parameter<double>("max_roll_deg",  40.0);
    double max_pitch_deg = this->declare_parameter<double>("max_pitch_deg", 40.0);
    max_roll_rad_  = max_roll_deg  * M_PI / 180.0;
    max_pitch_rad_ = max_pitch_deg * M_PI / 180.0;

    base_frame_  = this->declare_parameter<std::string>("base_frame",  "base_link");
    child_frame_ = this->declare_parameter<std::string>("child_frame", "TCP_prediction");
    imu_topic_   = this->declare_parameter<std::string>("imu_topic",   "/imu/data_filtered");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // IMU subscriber (SensorDataQoS)
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TcpPredictionNode::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "tcp_prediction_node started. Publishing %s -> %s",
                base_frame_.c_str(), child_frame_.c_str());
  }

private:
  // Rx about +X
  static tf2::Matrix3x3 Rx(double r) {
    const double c = std::cos(r), s = std::sin(r);
    return tf2::Matrix3x3(1, 0, 0,
                          0, c,-s,
                          0, s, c);
  }
  // Ry about +Y
  static tf2::Matrix3x3 Ry(double p) {
    const double c = std::cos(p), s = std::sin(p);
    return tf2::Matrix3x3( c, 0, s,
                           0, 1, 0,
                          -s, 0, c);
  }
  // Fixed alignment so that at roll=pitch=0:
  //   x_J = -z_B (points down), y_J = y_B, z_J = x_B
  static tf2::Matrix3x3 RAlign() {
    // Columns are [x_J, y_J, z_J] expressed in base
    // x_J=[0,0,-1], y_J=[0,1,0], z_J=[1,0,0]
    return tf2::Matrix3x3(
      /*row0*/ 0, 0, 1,
      /*row1*/ 0, 1, 0,
      /*row2*/-1, 0, 0
    );
  }

  static double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Extract roll/pitch from IMU orientation (assumed in base_link frame)
    tf2::Quaternion q_imu;
    tf2::fromMsg(msg->orientation, q_imu);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_imu).getRPY(roll, pitch, yaw);

    // Saturate
    roll  = clamp(-roll,  -max_roll_rad_,  max_roll_rad_);
    pitch = clamp(-pitch, -max_pitch_rad_, max_pitch_rad_);

    // Orientation of EE frame (relative to base): R = Rx(roll) * Ry(pitch) * R_ALIGN
    const tf2::Matrix3x3 R_total = Rx(roll) * Ry(pitch) * RAlign();

    // Position of EE frame:
    // p = [t1x, 0, t1z + qz]  +  R_total * [t2, 0, 0]
    const tf2::Vector3 p_fixed(t1x_, 0.0, t1z_ + qz_);
    const tf2::Vector3 offset_local(t2_, 0.0, 0.0);
    const tf2::Vector3 p = p_fixed + R_total * offset_local;

    // Convert R_total to quaternion
    tf2::Quaternion q_rot;
    R_total.getRotation(q_rot);

    // Publish TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = base_frame_;
    tf_msg.child_frame_id  = child_frame_;
    tf_msg.transform.translation.x = p.x();
    tf_msg.transform.translation.y = p.y();
    tf_msg.transform.translation.z = p.z();
    tf_msg.transform.rotation = tf2::toMsg(q_rot);

    tf_broadcaster_->sendTransform(tf_msg);
  }

  // --- Members ---
  double t1x_{0.35}, t1z_{0.42}, t2_{0.25};
  double qz_{0.0};
  double max_roll_rad_{M_PI/6.0}, max_pitch_rad_{M_PI/6.0}; // 30 deg default

  std::string base_frame_{"base_link"};
  std::string child_frame_{"TCP_prediction"};
  std::string imu_topic_{"/imu/data_filtered"};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TcpPredictionNode>());
  rclcpp::shutdown();
  return 0;
}
