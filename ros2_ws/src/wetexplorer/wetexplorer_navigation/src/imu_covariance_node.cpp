#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuCovarianceNode : public rclcpp::Node
{
public:
    ImuCovarianceNode()
    : Node("imu_covariance_node")
    {
        using std::placeholders::_1;

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&ImuCovarianceNode::imuCallback, this, _1));

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);

        // Example covariance values
        orientation_covariance_ = {0.0001, 0, 0,
                                   0, 0.0001, 0,
                                   0, 0, 0.0001};

        angular_velocity_covariance_ = {0.0001225, 0, 0,
                                        0, 0.0001225, 0,
                                        0, 0, 0.0001225};

        linear_acceleration_covariance_ = {0.000864, 0, 0,
                                           0, 0.000864, 0,
                                           0, 0, 0.000864};
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto modified_msg = *msg;

        modified_msg.orientation_covariance = orientation_covariance_;
        modified_msg.angular_velocity_covariance = angular_velocity_covariance_;
        modified_msg.linear_acceleration_covariance = linear_acceleration_covariance_;

        imu_pub_->publish(modified_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    std::array<double, 9> orientation_covariance_;
    std::array<double, 9> angular_velocity_covariance_;
    std::array<double, 9> linear_acceleration_covariance_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuCovarianceNode>());
    rclcpp::shutdown();
    return 0;
}
