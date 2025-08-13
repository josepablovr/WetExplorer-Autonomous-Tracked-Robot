// src/move_tcp_action_server.cpp

/*********************************************************************
 * BSD 3-Clause License
 * 
 * Adapted MoveTCP action server with “ref” parameter support
 *********************************************************************/

#include <memory>
#include <cmath>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <Eigen/Dense>

#include "wetexplorer_navigation/action/move_tcp.hpp"

using MoveTCP    = wetexplorer_navigation::action::MoveTCP;
using GoalHandle = rclcpp_action::ServerGoalHandle<MoveTCP>;

class MoveTcpActionServer : public rclcpp::Node
{
public:
  MoveTcpActionServer()
  : Node("move_tcp_action_server"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    goal_active_(false),
    steady_count_(0)
  {
    // Declare and read "ref" parameter (either "odom" or "map")
    this->declare_parameter<std::string>("ref", "map");
    this->get_parameter("ref", ref_frame_);

    // Decide odometry subscription topic based on ref_frame_
    std::string odom_topic = (ref_frame_ == "odom") 
                              ? "/odometry/local" 
                              : "/odometry/global";

    // Publishers & subscribers
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/commands/cmd_vel", 10);
    error_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/error_tcp", 10);
 
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/odometry/tcp", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&MoveTcpActionServer::odomCallback, this, std::placeholders::_1));

    // Action server
    action_server_ = rclcpp_action::create_server<MoveTCP>(
      this,
      "/MoveTCP",
      std::bind(&MoveTcpActionServer::handleGoal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveTcpActionServer::handleCancel,   this, std::placeholders::_1),
      std::bind(&MoveTcpActionServer::handleAccepted, this, std::placeholders::_1));

    // Controller params
    tolerance_ = 0.03;
    max_v_      = 0.1;
    max_w_      = 0.1;
    KPxte_ = 0.1;
    KPxte_ = 0.1;
    KPp_   = 1.0;
    KPt_   = 0.1;
    KIp_   = 0.0;
    KIxte_ = 0.0;
    KIt_   = 0.0;
    position_integral_    = 0.0;
    orientation_integral_ = 0.0;
    cross_track_integral_ = 0.0;
  }

private:
  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       cmd_vel_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       error_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
  rclcpp_action::Server<MoveTCP>::SharedPtr                    action_server_;

  // TF
  tf2_ros::Buffer             tf_buffer_;
  tf2_ros::TransformListener  tf_listener_;

  // Current TCP pose
  double cur_x_{0}, cur_y_{0}, cur_yaw_{0};

  // Active goal
  bool   goal_active_;
  double goal_x_, goal_y_;
  size_t steady_count_;

  // Controller params
  double tolerance_, max_v_, max_w_;
  double KPp_, KPxte_, KPt_;
  double KIp_, KIxte_, KIt_;
  double position_integral_, orientation_integral_, cross_track_integral_;

  // "ref" frame: "odom" or "map"
  std::string ref_frame_;

  double computeDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
  }

  // === Odometry callback: compute & publish TCP pose ===
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr /*msg*/)
  {
    try {
      // 1) ref_frame_ -> base_link
      auto tf_ref_base = tf_buffer_.lookupTransform(
        ref_frame_, "base_link", tf2::TimePointZero);

      // 2) base_link -> chamber_link
      auto tf_base_chamber = tf_buffer_.lookupTransform(
        "base_link", "chamber_link", tf2::TimePointZero);

      // Rotation & translation (ref->base)
      tf2::Quaternion q1; 
      tf2::fromMsg(tf_ref_base.transform.rotation, q1);
      tf2::Matrix3x3 R1_mat(q1);
      Eigen::Matrix3d R1;
      for (int i=0; i<3; ++i) {
        for (int j=0; j<3; ++j) {
          R1(i,j) = R1_mat[i][j];
        }
      }
      Eigen::Vector3d T1(
        tf_ref_base.transform.translation.x,
        tf_ref_base.transform.translation.y,
        tf_ref_base.transform.translation.z
      );

      // Rotation & translation (base->chamber)
      tf2::Quaternion q2;
      tf2::fromMsg(tf_base_chamber.transform.rotation, q2);
      tf2::Matrix3x3 R2_mat(q2);
      Eigen::Matrix3d R2;
      for (int i=0; i<3; ++i) {
        for (int j=0; j<3; ++j) {
          R2(i,j) = R2_mat[i][j];
        }
      }
      Eigen::Vector3d T2(
        tf_base_chamber.transform.translation.x,
        tf_base_chamber.transform.translation.y,
        tf_base_chamber.transform.translation.z
      );

      // Combined transform (ref->chamber)
      Eigen::Matrix3d R  = R1 * R2;
      Eigen::Vector3d T  = R1 * T2 + T1;

      double x   = T(0);
      double y   = T(1);
      double yaw = std::atan2(R1(1,0), R1(0,0));

      // Publish TCP pose
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp    = this->get_clock()->now();
      pose_msg.header.frame_id = ref_frame_;
      pose_msg.pose.position.x = x;
      pose_msg.pose.position.y = y;
      pose_msg.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose_msg.pose.orientation = tf2::toMsg(q);
      pose_pub_->publish(pose_msg);

      // Update internal pose
      cur_x_   = x;
      cur_y_   = y;
      cur_yaw_ = yaw;
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(),
                  "TF lookup failed in odomCallback: %s",
                  ex.what());
    }
  }

  // === Action callbacks ===

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const MoveTCP::Goal> goal)
  {
    RCLCPP_INFO(get_logger(),
      "Received MoveTCP goal: (%.2f, %.2f)",
      goal->target_pose.pose.position.x,
      goal->target_pose.pose.position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "MoveTCP goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&MoveTcpActionServer::execute, this, goal_handle)}
      .detach();
  }

  // === Execution ===

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto result   = std::make_shared<MoveTCP::Result>();
    auto feedback = std::make_shared<MoveTCP::Feedback>();

    goal_x_      = goal_handle->get_goal()->target_pose.pose.position.x;
    goal_y_      = goal_handle->get_goal()->target_pose.pose.position.y;
    goal_active_ = true;
    steady_count_= 0;

    rclcpp::Rate rate(20);  // 20 Hz
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
        result->success = false;
        goal_handle->canceled(result);
        goal_active_ = false;
        return;
      }

      double dx = goal_x_ - cur_x_;
      double dy = goal_y_ - cur_y_;
      double dist = std::hypot(dx, dy);
      feedback->remaining_distance = dist;
      goal_handle->publish_feedback(feedback);

      if (dist < tolerance_) {
        steady_count_++;
      } else {
        steady_count_ = 0;
      }

      if (steady_count_ >= 150) {
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "MoveTCP goal succeeded");
        goal_active_ = false;
        steady_count_ = 0;
        steady_count_ = 0;
        return;
      }

      // Compute heading & control
      double alpha = std::atan2(dy, dx);
      double beta = alpha - cur_yaw_;
      beta = std::fmod(beta + M_PI, 2 * M_PI);
      if (beta < 0) beta += 2 * M_PI;
      beta -= M_PI;

      double cross_track = std::sqrt(dx*dx + dy*dy) * std::sin(beta);
      double pos_error   = std::sqrt(dx*dx + dy*dy);
      
      double ori_error = alpha - cur_yaw_;
      ori_error = std::fmod(ori_error + M_PI, 2 * M_PI);
      if (ori_error < 0) ori_error += 2 * M_PI;
      ori_error -= M_PI;

      double dx_bl =  std::cos(cur_yaw_) * dx + std::sin(cur_yaw_) * dy;   // forward-axis component
      //double dy_bl = -std::sin(cur_yaw_) * dx + std::cos(cur_yaw_) * dy;   // left-axis  component
      double direction = std::copysign(1.0,dx_bl);
      
      
            
      // Gain scheduling
      if (pos_error >= 0.30) {
        KPt_        = 1.0;
        KPp_        = 1.0;
        KPt_        = 1.0;
        KPp_        = 110;
        max_w_      = 0.5;
        KIp_        = 0.1;
        KIt_        = 0.02;
        direction = 1.0;
        
      }
      else if (pos_error >= 0.15) {
        KPt_        = 1.0;
        KPp_        = 1.0;}
      else if (pos_error >= 0.15) {
        KPt_        = 1.0;
        KPp_        = 1.0;
        max_w_      = 0.2;
        KIp_        = 0.1;
        KIt_        = 0.02;
        direction = 1.0;
       
        
      }
      else if (pos_error >= 0.005) {
        KPt_        = 1.0;
        KPt_        = 1.0;
        KPp_        = 1.0;
        max_w_      = 0.1;
        KIp_        = 0.05;
        KIt_        = 0.02;
        max_w_      = 0.1;
    
      }
      else if (pos_error < 0.005){
        KPt_        = 0.00;
        KPp_        = 0.00;
      }
      else if (pos_error < 0.005){
        KPt_        = 0.00;
        KPp_        = 0.00;
        max_w_      = 0.05;
        KIp_        = 0.005;
        if (ori_error < 0.02) //1.1grad
          KIt_        = 0.02;
        else {
          KIt_        = 0.00;
          KIp_        = 0.005;}
      }

      
      

      position_integral_    += pos_error;
      orientation_integral_ += ori_error;
      cross_track_integral_ += cross_track;

      position_integral_    = std::clamp(position_integral_, -1.0, 1.0);
      orientation_integral_ = std::clamp(orientation_integral_, -1.0, 1.0);
      cross_track_integral_ = std::clamp(cross_track_integral_, -1.0, 1.0);

      double v = KPp_ * pos_error + KIp_ * position_integral_;
      double w = KPt_ * ori_error + KPxte_ * cross_track
                 + KIt_ * orientation_integral_
                 + KIxte_ * cross_track_integral_;

      v = std::clamp(direction*v, -max_v_, max_v_);
      v = std::clamp(direction*v, -max_v_, max_v_);
      w = std::clamp(w, -max_w_, max_w_);

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x  = v;
      cmd.angular.z = w;
      cmd_vel_pub_->publish(cmd);

      cmd.linear.x  = pos_error;
      cmd.angular.z = ori_error;
      error_vel_pub_->publish(cmd);

    

      rate.sleep();
    }

    // Abort if we exit unexpectedly
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    result->success = false;
    goal_handle->abort(result);
    goal_active_ = false;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveTcpActionServer>());
  rclcpp::shutdown();
  return 0;
}
