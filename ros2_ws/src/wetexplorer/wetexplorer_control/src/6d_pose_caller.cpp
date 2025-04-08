#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>

using std::placeholders::_1;

class PoseCaller : public rclcpp::Node
{
public:
    PoseCaller()
    : Node("Object_pose_caller"), prev_button_state_(0)
    {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy_teleop/joy", 10, std::bind(&PoseCaller::joy_callback, this, _1));

        client_ = this->create_client<std_srvs::srv::Trigger>("/trigger_publish_transform");

        // Wait for the service to be available
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for /trigger_publish_transform service...");
        }
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        int current_button_state = msg->buttons[0];

        if (current_button_state == 1 && prev_button_state_ == 0) {
            RCLCPP_INFO(this->get_logger(), "Button 0 pressed. Calling service...");
            call_trigger_service();
        }

        prev_button_state_ = current_button_state;
    }

    void call_trigger_service()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        auto result_future = client_->async_send_request(request,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", response->message.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Service call failed: %s", response->message.c_str());
                }
            });
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    int prev_button_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseCaller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
