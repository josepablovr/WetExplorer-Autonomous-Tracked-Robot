#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <string>
#include <unordered_map>

class ForwardKinematics : public rclcpp::Node
{
public:
    ForwardKinematics()
    : Node("forward_kinematics_node"),
      tracks_separation_(0.6108),
      gear_ratio_(1.0),
      radius_sprocket_(0.075),
      x_(0.0),
      y_(0.0),
      cov_x_(0.0),
      cov_y_(0.0),
      cov_theta_(0.0),
      alpha1_(0.000118),
      alpha2_(0.35),
      alpha3_(0.0),
      alpha4_(0.15),
      alpha5_(7.61e-5),
      alpha6_(9e-6),
      offset_bvx_(1.2e-4),
      offset_bomega_(5e-6),
      theta_(0.0),
      last_time_(this->now())
    {
        using std::placeholders::_1;

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&ForwardKinematics::jointStateCallback, this, _1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odometry/forward_kinematics", 10);
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "Received JointState message with %zu joints", msg->name.size());

        for (size_t i = 0; i < msg->name.size(); ++i) {
            std::string joint_name = msg->name[i];
            double velocity = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;

            //RCLCPP_INFO(this->get_logger(), "Joint: '%s', Velocity: %.6f", joint_name.c_str(), velocity);
        }

        double omega_L = 0.0;
        double omega_R = 0.0;
        bool left_found = false;
        bool right_found = false;

        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (i >= msg->velocity.size()) continue;

            if (msg->name[i] == "left_wheel_joint") {
                omega_L = msg->velocity[i];
                left_found = true;
            } else if (msg->name[i] == "right_wheel_joint") {
                omega_R = msg->velocity[i];
                right_found = true;
            }
        }

        if (!left_found || !right_found) {
            //RCLCPP_WARN(this->get_logger(), "Missing joint velocity data for left or right wheel joint.");
            return;
        }

        // Convert rad/s -> linear m/s (accounting for gear ratio & sprocket radius)
        double v_L = (omega_L * radius_sprocket_) / gear_ratio_;
        double v_R = (omega_R * radius_sprocket_) / gear_ratio_;

        // Forward kinematics
        double V_x = (v_L + v_R) / 2.0;
        double theta_dot = (v_R - v_L) / tracks_separation_;

        publishOdometry(V_x, theta_dot);
    }

   

    void publishOdometry(double V, double omega)
    {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();

        // --- Model variances (expected values, not samples) ---
        double e_vx = alpha1_ * V + alpha2_ * omega + offset_bvx_;
        double e_vy = alpha3_ * V + alpha4_ * omega;
        double e_theta = alpha5_ * V + alpha6_ * omega + offset_bomega_;

        double b_vx = std::pow(e_vx,2);
        double b_vy = std::pow(e_vy,2);
        double b_theta = std::pow(e_theta,2);


        // --- Pose integration using midpoint rule ---
        double delta_theta = omega * dt;
        double theta_mid = theta_ + 0.5 * delta_theta;
        double delta_x = V * std::cos(theta_mid) * dt;
        double delta_y = V * std::sin(theta_mid) * dt;


        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        // --- Model variances (expected values, not samples) ---
        double e_dx = e_vx*dt ;
        double e_dy = e_vy*dt;
        double e_dtheta = e_theta*dt;

        double b_dx = dt*b_vx;
        double b_dy = dt*b_vy;
        double b_dtheta = dt*b_theta;

        // if (std::abs(V) <= 0.005) {
        //     b_dx = 0.00;
        //     b_dy = 0.00;
        // }

        // if (std::abs(omega) <= 0.005) {
        //     b_dtheta = 0.0000;
        // }

        // --- Accumulate pose covariance ---
        cov_x_ += b_dx;
        cov_y_ += b_dy;
        cov_theta_ += b_dtheta;

        // --- Prepare odometry message ---
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom.twist.twist.linear.x = V;
        odom.twist.twist.angular.z = omega;

        // --- Pose covariance (x, y, yaw only) ---
        for (int i = 0; i < 36; ++i)
            odom.pose.covariance[i] = 0.0;

        odom.pose.covariance[0] = cov_x_;       // x
        odom.pose.covariance[7] = cov_y_;       // y
        odom.pose.covariance[35] = cov_theta_;  // yaw
        
        // --- Twist covariance ---
        for (int i = 0; i < 36; ++i) {
            if (i == 0) {
                odom.twist.covariance[i] = (std::abs(V) <= 0.005) ? 0.0001 : (b_vx);
            } else if (i == 7) {
                odom.twist.covariance[i] = (std::abs(omega) <= 0.005) ? 0.0001 : (b_vy);
            }
             else if (i == 35) {
                odom.twist.covariance[i] = (std::abs(omega) <= 0.005) ? 0.0001 : b_theta;
            } else if (i == 14 || i == 21 || i == 28) {
                odom.twist.covariance[i] = 99999.0;
            } else {
                odom.twist.covariance[i] = 0.0;
            }
        }

        odom_pub_->publish(odom);
        last_time_ = current_time;
    }


    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    double tracks_separation_;
    double gear_ratio_;
    double radius_sprocket_;

   

    // --- Position covariance accumulation ---
    double cov_x_;
    double cov_y_;
    double cov_theta_;

    double alpha1_;
    double alpha2_;
    double alpha3_;
    double alpha4_;
    double alpha5_;
    double alpha6_;
    double offset_bvx_;
    double offset_bomega_;


    double x_, y_, theta_;
    rclcpp::Time last_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForwardKinematics>());
    rclcpp::shutdown();
    return 0;
}
