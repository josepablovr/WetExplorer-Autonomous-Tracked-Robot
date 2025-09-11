#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include <yolo_msgs/msg/detection.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

class ObjectDepthCloudNode : public rclcpp::Node
{
public:
  ObjectDepthCloudNode() : Node("object_depth_cloud_node")
  {
    // --- Parameters ---
    target_class_   = this->declare_parameter<std::string>("target_class", "ring");
    score_thresh_   = this->declare_parameter<double>("score_threshold", 0.9);
    min_area_       = this->declare_parameter<double>("min_area", 500.0); // if bbox is in pixels
    min_depth_      = this->declare_parameter<double>("min_depth", 0.1);
    max_depth_      = this->declare_parameter<double>("max_depth", 10.0);
    depth_scale_    = this->declare_parameter<double>("depth_scale_uint16", 0.001); // mm->m
    output_topic_   = this->declare_parameter<std::string>("output_topic", "/object/depth_cloud");
    output_frame_   = this->declare_parameter<std::string>("output_frame", ""); // empty => use camera frame
    use_sim_time_ = this->get_parameter("use_sim_time").as_bool();

    if (use_sim_time_){
      depth_scale_ = 0.001;
    }
    else{
      depth_scale_ = 1.0;
    }
    // --- QoS ---
    rclcpp::SensorDataQoS sensor_qos;

    // --- Publisher ---
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, sensor_qos);

    // --- Read CameraInfo once, then unsubscribe ---
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", 10,
      std::bind(&ObjectDepthCloudNode::cameraInfoCb, this, std::placeholders::_1));

    // --- Sync depth + detections ---
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, "/camera/depth/raw", sensor_qos.get_rmw_qos_profile());
    det_sub_   = std::make_shared<message_filters::Subscriber<yolo_msgs::msg::DetectionArray>>(
        this, "/yolo/detections", sensor_qos.get_rmw_qos_profile());

    using Policy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, yolo_msgs::msg::DetectionArray>;
    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(
        Policy(20), *depth_sub_, *det_sub_);
    sync_->registerCallback(std::bind(&ObjectDepthCloudNode::syncedCb, this,
                                      std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "ObjectDepthCloudNode ready.");
  }

private:
  // Camera intrinsics
  bool have_K_ = false;
  double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
  std::string camera_frame_;

  // Params
  std::string target_class_;
  double score_thresh_;
  double min_area_;
  double min_depth_, max_depth_;
  double depth_scale_;
  std::string output_topic_;
  std::string output_frame_;

  bool use_sim_time_ = false;

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::shared_ptr<message_filters::Subscriber<yolo_msgs::msg::DetectionArray>> det_sub_;
  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::Image, yolo_msgs::msg::DetectionArray>>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  void cameraInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // Read K once
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    camera_frame_ = msg->header.frame_id;
    have_K_ = (fx_ > 0 && fy_ > 0);

    RCLCPP_INFO(get_logger(), "CameraInfo received (fx=%.3f, fy=%.3f, cx=%.3f, cy=%.3f), unsubscribing.",
                fx_, fy_, cx_, cy_);

    // Unsubscribe to save bandwidth/CPU
    camera_info_sub_.reset();
  }

  void syncedCb(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
                const yolo_msgs::msg::DetectionArray::ConstSharedPtr &det_array)
  {
    if (!have_K_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No CameraInfo yet; skipping.");
      return;
    }
    if (det_array->detections.empty()) {
      return;
    }

    // Pick best matching detection (by score) of the target class
    const yolo_msgs::msg::Detection* best = nullptr;
    for (const auto &d : det_array->detections) {
      if (d.class_name == target_class_) {
        if (!best || d.score > best->score) best = &d;
      }
    }
    if (!best) return;

    // Optional check: area (if your bbox size is in pixels; if it's meters, this check may be meaningless)
    const double area = best->bbox.size.x * best->bbox.size.y;
    if (best->score < score_thresh_ || area < min_area_) {
      return;
    }

    // Convert depth to cv::Mat
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      // Keep original encoding
      cv_ptr = cv_bridge::toCvShare(depth_msg);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat depth_raw = cv_ptr->image; // could be 16UC1 or 32FC1

    // Build a polygon mask from detection.mask.data (vector of points with x,y)
    if (best->mask.data.empty()) {
      return;
    }
    std::vector<cv::Point> poly;
    poly.reserve(best->mask.data.size());
    for (const auto &p : best->mask.data) {
      int u = std::max(0, std::min(static_cast<int>(depth_raw.cols) - 1, static_cast<int>(p.x)));
      int v = std::max(0, std::min(static_cast<int>(depth_raw.rows) - 1, static_cast<int>(p.y)));
      poly.emplace_back(u, v);
    }
    cv::Mat mask(depth_raw.rows, depth_raw.cols, CV_8UC1, cv::Scalar(0));
    const std::vector<std::vector<cv::Point>> polys{poly};
    cv::fillPoly(mask, polys, cv::Scalar(255));

    // Create masked depth (float32 meters)
    cv::Mat depth_m;
    if (depth_msg->encoding == "16UC1") {
      cv::Mat depth_u16 = depth_raw;
      depth_u16.convertTo(depth_m, CV_32FC1, depth_scale_); // scale mm->m (or per param)
    } else {
      // expect 32FC1 meters
      depth_m = depth_raw.clone();
    }

    // Apply mask + valid range
    cv::Mat valid = (depth_m > static_cast<float>(min_depth_)) &
                    (depth_m < static_cast<float>(max_depth_));
    cv::Mat masked;
    depth_m.copyTo(masked, mask & valid);

    cv::Mat valid_mask = (masked > 0.0f);
    int N = cv::countNonZero(valid_mask);
    if (N == 0) return;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = depth_msg->header.stamp;
    cloud.header.frame_id = output_frame_.empty() ? camera_frame_ : output_frame_;
    cloud.height = 1;
    cloud.width  = static_cast<uint32_t>(N);

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(N);

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

    uint32_t filled = 0;
    for (int v = 0; v < masked.rows; ++v) {
    const float* zptr = masked.ptr<float>(v);
    for (int u = 0; u < masked.cols; ++u) {
        float z = zptr[u];
        if (z <= 0.0f) continue;
        float x = (static_cast<float>(u) - static_cast<float>(cx_)) * z / static_cast<float>(fx_);
        float y = (static_cast<float>(v) - static_cast<float>(cy_)) * z / static_cast<float>(fy_);
        *it_x = x; *it_y = y; *it_z = z;
        ++it_x; ++it_y; ++it_z;
        ++filled;
    }
    }

    // If the pre-count was off, shrink to actual number of points
    if (filled != cloud.width) {
    sensor_msgs::PointCloud2Modifier mod2(cloud);
    mod2.resize(filled);
    cloud.width = filled;
    cloud.row_step = cloud.point_step * cloud.width;
    }

    cloud_pub_->publish(cloud);
  }

private:
  // disallow copying
  ObjectDepthCloudNode(const ObjectDepthCloudNode&) = delete;
  ObjectDepthCloudNode& operator=(const ObjectDepthCloudNode&) = delete;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDepthCloudNode>());
  rclcpp::shutdown();
  return 0;
}
