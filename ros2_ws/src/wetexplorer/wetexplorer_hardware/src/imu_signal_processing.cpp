#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <Eigen/Geometry> // For quaternion operations

class SensorOffsetCorrector {
public:
    SensorOffsetCorrector() {
        ros::NodeHandle nh;

        publish_transform = false;
        imu_subscriber = nh.subscribe("/imu/data_transformed", 10, &SensorOffsetCorrector::imuCallback, this);
        
        if (publish_transform) {
            br.reset(new tf2_ros::TransformBroadcaster());
        }
        listener.reset(new tf::TransformListener());

        imu_publisher = nh.advertise<sensor_msgs::Imu>("/imu/data_filtered2", 10);
        imu_publisher2 = nh.advertise<sensor_msgs::Imu>("/imu/gravity2", 10);

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
        acceleration_calibration = false;
        calibration_time = 0.1;
        init_calib_time = ros::Time(0);
        gravity_magnitude = 0.0;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        
        if (!acceleration_calibration) {
            calibrateAcceleration(msg);
        } else {
            double roll, pitch, yaw;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(msg->orientation, quat);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            roll = roll + M_PI; // Correct roll by 180 degrees
            pitch = -pitch; // Negate pitch
            yaw = -yaw; // Negate yaw

            if (initial_yaw == 0.0) {
                initial_yaw = yaw;
            }

            yaw = yaw - initial_yaw;
            last_yaw = yaw;

            yaw = yaw + yaw_drift;
            yaw_output = yaw;

            tf::Quaternion corrected_quat = tf::createQuaternionFromRPY(roll, pitch, yaw_output);

            sensor_msgs::Imu corrected_imu;
            corrected_imu.header = msg->header;
            tf::quaternionTFToMsg(corrected_quat, corrected_imu.orientation);

            corrected_imu.orientation_covariance = msg->orientation_covariance;

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

            corrected_imu.angular_velocity_covariance = msg->angular_velocity_covariance;
            corrected_imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;

            imu_publisher.publish(corrected_imu);

            corrected_imu.linear_acceleration.x = g.x();
            corrected_imu.linear_acceleration.y = g.y();
            corrected_imu.linear_acceleration.z = g.z();

            imu_publisher2.publish(corrected_imu);

            if (publish_transform) {
                geometry_msgs::TransformStamped t;
                t.header.stamp = ros::Time::now();
                t.header.frame_id = "world";
                t.child_frame_id = "base_link";
                t.transform.rotation = corrected_imu.orientation;

                br->sendTransform(t);
            }
        }
        
    }

    void run() {
        ros::spin();
    }

private:
    bool publish_transform;
    double alpha_acc, alpha_ang_vel, alpha_grav, alpha;
    double max_acc, max_vel, max_acc_grav, min_acc;
    double initial_yaw, yaw_drift, yaw_output, last_yaw;
    double calibration_time;
    int sample_frequency, cutoff_frequency;
    bool acceleration_calibration;
    double gravity_magnitude;
    ros::Time init_calib_time;

    Eigen::Vector3d prev_acc, prev_ang_vel, prev_grav;
    ros::Subscriber imu_subscriber;
    ros::Publisher imu_publisher, imu_publisher2;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br;
    std::unique_ptr<tf::TransformListener> listener;

    Eigen::Vector3d lowPassFilter(const Eigen::Vector3d& new_value, const Eigen::Vector3d& prev_value, double alpha) {
        if (prev_value.isZero()) {
            return new_value;
        }
        return alpha * new_value + (1 - alpha) * prev_value;
    }

    Eigen::Vector3d saturateVector(const Eigen::Vector3d& vec, double min_value, double max_value) {
        Eigen::Vector3d saturated = vec;
        for (int i = 0; i < 3; ++i) {
            saturated[i] = std::max(std::min(vec[i], max_value), min_value);
        }
        return saturated;
    }

    Eigen::Vector3d accRejection(const Eigen::Vector3d& acc) {
        Eigen::Vector3d acc_corrected = acc;
        for (int i = 0; i < 3; ++i) {
            if (fabs(acc[i]) <= min_acc) {
                acc_corrected[i] = 0.0;
            } else if (acc[i] < 0.0) {
                acc_corrected[i] += min_acc;
            } else {
                acc_corrected[i] -= min_acc;
            }
        }
        return acc_corrected;
    }

    void calibrateAcceleration(const sensor_msgs::Imu::ConstPtr& msg) {
        if (init_calib_time.isZero()) {
            init_calib_time = ros::Time::now();
        } else {
            ros::Time current_time = ros::Time::now();
            Eigen::Vector3d grav(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            Eigen::Vector3d filtered_grav = lowPassFilter(grav, prev_grav, 0.001);

            if ((current_time - init_calib_time).toSec() >= calibration_time) {
                gravity_magnitude = filtered_grav.norm();
                ROS_INFO_STREAM("GRAVITY MAGNITUDE: " << gravity_magnitude);
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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_offset_corrector");

    SensorOffsetCorrector node;
    node.run();

    return 0;
}
