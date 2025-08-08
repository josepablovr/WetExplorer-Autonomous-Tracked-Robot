#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

class OdometryErrorEstimator : public rclcpp::Node
{
public:
    OdometryErrorEstimator()
    : Node("odometry_error_estimator")
    {
        using std::placeholders::_1;

        fk_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/forward_kinematics", 10, std::bind(&OdometryErrorEstimator::fkCallback, this, _1));

        gt_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/ground_truth", 10, std::bind(&OdometryErrorEstimator::gtCallback, this, _1));

        error_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/odometry/errors", 10);
    }

private:
    nav_msgs::msg::Odometry::SharedPtr latest_fk_;
    nav_msgs::msg::Odometry::SharedPtr latest_gt_;

    void fkCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_fk_ = msg;
        computeAndPublishError();
    }

    void gtCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_gt_ = msg;
        computeAndPublishError();
    }

    void computeAndPublishError()
    {
        if (!latest_fk_ || !latest_gt_)
            return;

        // Time sync check (optional)
        rclcpp::Time t_fk = latest_fk_->header.stamp;
        rclcpp::Time t_gt = latest_gt_->header.stamp;
        double dt = (t_fk - t_gt).seconds();

        if (std::abs(dt) > 0.05)  // skip if too far apart
            return;

        const auto& v_fk = latest_fk_->twist.twist;
        const auto& v_gt = latest_gt_->twist.twist;

        // Error = estimated - ground truth
        geometry_msgs::msg::Twist error;
        error.linear.x = v_fk.linear.x - v_gt.linear.x;
        error.linear.y = v_fk.linear.y - v_gt.linear.y;
        error.angular.z = v_fk.angular.z - v_gt.angular.z;

        error_pub_->publish(error);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fk_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr error_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryErrorEstimator>());
    rclcpp::shutdown();
    return 0;
}
