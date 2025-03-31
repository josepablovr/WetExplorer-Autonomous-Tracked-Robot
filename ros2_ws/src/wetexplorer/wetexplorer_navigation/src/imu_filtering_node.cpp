#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include <array>
#include <optional>
#include <vector>

class SensorOffsetCorrector : public rclcpp::Node
{
public:
    SensorOffsetCorrector() : Node("sensor_offset_corrector"), publish_transform_(false)
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&SensorOffsetCorrector::imuCallback, this, std::placeholders::_1));

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_filtered", 10);
        gravity_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/gravity", 10);

        if (publish_transform_)
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        alpha_acc_ = 0.05;
        alpha_ang_vel_ = 0.5;
        alpha_grav_ = 0.3;

        sample_frequency_ = 250.0;
        cutoff_frequency_ = 50.0;
        max_acc_ = 1.0;
        max_vel_ = 1.0;
        min_acc_ = 0.01;
        max_acc_grav_ = 9.8 + max_acc_;
        calibration_time_ = 0.1;

        acceleration_calibration_ = false;
    }

private:
    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_, gravity_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Parameters and state
    bool publish_transform_;
    double alpha_acc_, alpha_ang_vel_, alpha_grav_;
    double sample_frequency_, cutoff_frequency_;
    double max_acc_, max_vel_, min_acc_;
    double max_acc_grav_;
    double calibration_time_;

    bool acceleration_calibration_;
    std::optional<rclcpp::Time> init_calib_time_;
    std::optional<double> gravity_magnitude_;

    std::array<double, 3> prev_acc_{};
    std::array<double, 3> prev_ang_vel_{};
    std::array<double, 3> prev_grav_{};
    std::optional<double> initial_yaw_;
    double yaw_offset_ = 0.0, yaw_drift_ = 0.0, yaw_output_ = 0.0;

    double saturate(double value, double min_value, double max_value)
    {
        return std::max(min_value, std::min(value, max_value));
    }

    double lowPassFilter(double new_val, std::optional<double> prev_val, double alpha)
    {
        return prev_val ? alpha * new_val + (1.0 - alpha) * *prev_val : new_val;
    }

    double accRejection(double acc)
    {
        if (std::abs(acc) <= min_acc_) return 0.0;
        return acc < 0.0 ? acc + min_acc_ : acc - min_acc_;
    }

    std::tuple<double, double> calculateAngles(double ax, double ay, double az)
    {
        double roll = std::atan2(ay, std::sqrt(ax * ax + az * az));
        double pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
        return {roll, pitch};
    }

    std::tuple<double, double, double> anglesToVector(double roll, double pitch, double magnitude)
    {
        double ax = -magnitude * std::sin(pitch);
        double ay = magnitude * std::sin(roll) * std::cos(pitch);
        double az = magnitude * std::cos(roll) * std::cos(pitch);
        return {ax, ay, az};
    }

    void calibrateAcceleration(const sensor_msgs::msg::Imu &msg)
    {
        if (!init_calib_time_) {
            init_calib_time_ = now();
            return;
        }

        double gx = lowPassFilter(msg.linear_acceleration.x, prev_grav_[0], 0.001);
        double gy = lowPassFilter(msg.linear_acceleration.y, prev_grav_[1], 0.001);
        double gz = lowPassFilter(msg.linear_acceleration.z, prev_grav_[2], 0.001);

        if ((now() - *init_calib_time_).seconds() >= calibration_time_) {
            gravity_magnitude_ = 9.7;
            acceleration_calibration_ = true;
            RCLCPP_INFO(this->get_logger(), "Gravity magnitude calibrated: %f", *gravity_magnitude_);
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!acceleration_calibration_) {
            calibrateAcceleration(*msg);
            return;
        }

        tf2::Quaternion q_orig(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);

        roll += M_PI;     // add 180 deg
        pitch = -pitch;   // negate pitch
        yaw = -yaw;

        if (!initial_yaw_) initial_yaw_ = yaw;
        yaw = yaw - *initial_yaw_ + yaw_offset_;
        yaw_output_ = yaw + yaw_drift_;

        tf2::Quaternion q_corrected;
        q_corrected.setRPY(roll, pitch, yaw_output_);
        q_corrected.normalize();

        auto corrected = *msg;
        corrected.orientation = tf2::toMsg(q_corrected);

        std::array<double, 3> filtered_grav = {
            saturate(lowPassFilter(msg->linear_acceleration.x, prev_grav_[0], alpha_grav_), -max_acc_grav_, max_acc_grav_),
            saturate(lowPassFilter(msg->linear_acceleration.y, prev_grav_[1], alpha_grav_), -max_acc_grav_, max_acc_grav_),
            saturate(lowPassFilter(msg->linear_acceleration.z, prev_grav_[2], alpha_grav_), -max_acc_grav_, max_acc_grav_)
        };
        prev_grav_ = filtered_grav;

        auto [gx, gy, gz] = anglesToVector(roll, pitch, *gravity_magnitude_);

        std::array<double, 3> acc = {
            saturate(lowPassFilter(filtered_grav[0] - gx, prev_acc_[0], alpha_acc_), -max_acc_, max_acc_),
            saturate(lowPassFilter(filtered_grav[1] - gy, prev_acc_[1], alpha_acc_), -max_acc_, max_acc_),
            saturate(lowPassFilter(filtered_grav[2] - gz, prev_acc_[2], alpha_acc_), -max_acc_, max_acc_)
        };

        for (auto &a : acc) a = accRejection(a);
        prev_acc_ = acc;

        corrected.linear_acceleration.x = acc[0];
        corrected.linear_acceleration.y = acc[1];
        corrected.linear_acceleration.z = acc[2];

        std::array<double, 3> ang = {
            saturate(lowPassFilter(msg->angular_velocity.x, prev_ang_vel_[0], alpha_ang_vel_), -max_vel_, max_vel_),
            saturate(lowPassFilter(msg->angular_velocity.y, prev_ang_vel_[1], alpha_ang_vel_), -max_vel_, max_vel_),
            saturate(lowPassFilter(msg->angular_velocity.z, prev_ang_vel_[2], alpha_ang_vel_), -max_vel_, max_vel_)
        };
        prev_ang_vel_ = ang;

        corrected.angular_velocity.x = ang[0];
        corrected.angular_velocity.y = ang[1];
        corrected.angular_velocity.z = ang[2];

        corrected.orientation_covariance = {
            0.0001, 0, 0,
            0, 0.0001, 0,
            0, 0, 0.0001
        };
        corrected.angular_velocity_covariance = {
            0.0001225, 0, 0,
            0, 0.0001225, 0,
            0, 0, 0.0001225
        };
        corrected.linear_acceleration_covariance = {
            0.000864, 0, 0,
            0, 0.000864, 0,
            0, 0, 0.000864
        };

        imu_pub_->publish(corrected);

        corrected.linear_acceleration.x = gx;
        corrected.linear_acceleration.y = gy;
        corrected.linear_acceleration.z = gz;
        gravity_pub_->publish(corrected);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorOffsetCorrector>());
    rclcpp::shutdown();
    return 0;
}
