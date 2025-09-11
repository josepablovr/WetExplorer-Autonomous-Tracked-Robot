#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <unordered_map>
#include <string>
#include <vector>
#include <tuple>
#include <optional>
#include <cmath>
#include <sstream>
#include <filesystem>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class RGBDImageToPointCloudNode : public rclcpp::Node
{
public:
  RGBDImageToPointCloudNode() : Node("rgbdimage_to_pointcloud")
  {
    // --- Parameters (matching Python) ---
    mask_path_              = this->declare_parameter<std::string>(
                                "mask_path",
                                "/ros2_ws/src/wetexplorer/wetexplorer_vision/wetexplorer_vision/chamber_mask.png");
    output_topic_           = this->declare_parameter<std::string>("output_topic", "/map/points");
    min_depth_              = this->declare_parameter<double>("min_depth", 0.1);
    max_depth_              = this->declare_parameter<double>("max_depth", 10.0);
    voxel_size_             = this->declare_parameter<double>("voxel_size", 0.025);
    publish_full_every_n_   = this->declare_parameter<int>("publish_full_every_n", 2);
    publish_deltas_         = this->declare_parameter<bool>("publish_deltas", true);
    tf_refresh_every_n_     = this->declare_parameter<int>("tf_refresh_every_n", 0);
    map_max_radius_         = this->declare_parameter<double>("map_max_radius", 0.0);
    depth_scale_uint16_     = this->declare_parameter<double>("depth_scale_uint16", 1.0); // mm->m

    map_frame_  = "map";
    base_frame_ = "base_link";

    // --- QoS ---
    rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
    rclcpp::QoS odom_qos(10);
    odom_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    rclcpp::QoS pc_qos(10);
    pc_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    pc_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    // --- Publishers ---
    pub_cloud_full_  = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, pc_qos);
    pub_cloud_delta_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_ + "_delta", pc_qos);

    // --- TF ---
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // --- Read CameraInfo once, then unsubscribe ---
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", 10,
      std::bind(&RGBDImageToPointCloudNode::cameraInfoCb, this, std::placeholders::_1));

    // --- Sync depth + detections ---
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, "/camera/depth/raw", sensor_qos.get_rmw_qos_profile());

    image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, "/camera/image/raw", sensor_qos.get_rmw_qos_profile());    

    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
        this, "/odometry/global", odom_qos.get_rmw_qos_profile());

    using Policy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image, nav_msgs::msg::Odometry>;

    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(35), *image_sub_, *depth_sub_, *odom_sub_);
    sync_->registerCallback(std::bind(&RGBDImageToPointCloudNode::syncedCb, this, _1, _2, _3));


    // Load mask once
    loadMask();

    RCLCPP_INFO(this->get_logger(),
                "Listening to /camera/image/raw + /camera/depth/raw + /odometry/global → publishing %s (+ _delta)",
                output_topic_.c_str());
  }

private:
  // ---------------- State / Params ----------------
  std::string mask_path_, output_topic_;
  double min_depth_{0.1}, max_depth_{10.0}, voxel_size_{0.025};
  int publish_full_every_n_{2};
  bool publish_deltas_{true};
  int tf_refresh_every_n_{0};
  double map_max_radius_{0.0};
  double depth_scale_uint16_{0.001}; // meters per uint16 unit

  std::string map_frame_, base_frame_;
  int frame_count_{0};


  // Camera intrinsics (from CameraInfo)
  bool  have_K_{false};
  float fx_{0.f}, fy_{0.f}, cx_{0.f}, cy_{0.f};
  std::string camera_frame_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // TF cache
  std::optional<geometry_msgs::msg::TransformStamped> tf_cam_in_base_;

  // mask cache
  cv::Mat mask_img_; // original
  std::unordered_map<std::string, cv::Mat> mask_cache_; // key "w x h" -> resized mask

  // grid cache: key by (w,h,fx,fy,cx,cy) string
  struct Grids { cv::Mat u_norm, v_norm; };
  std::unordered_map<std::string, Grids> grid_cache_;

  // Global voxel map: key->(x,y,z,r,g,b)
  struct XYZRGB { float x,y,z; uint8_t r,g,b; };
  std::unordered_map<std::string, XYZRGB> voxel_map_; // key = 12-byte voxel key as string

  // Latest pose map_T_base
  float latest_T_[16] = {1,0,0,0,
                         0,1,0,0,
                         0,0,1,0,
                         0,0,0,1};

  // ROS I/O
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_full_, pub_cloud_delta_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::Image, sensor_msgs::msg::Image, nav_msgs::msg::Odometry>>> sync_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---------------- Helpers ----------------
  static void transformToMat44(const geometry_msgs::msg::TransformStamped &tf, float T[16])
  {
    const auto &q = tf.transform.rotation;
    const auto &p = tf.transform.translation;
    const float w = q.w, x = q.x, y = q.y, z = q.z;

    const float xx = x*x, yy=y*y, zz=z*z;
    const float xy = x*y, xz=x*z, yz=y*z;
    const float wx = w*x, wy=w*y, wz=w*z;

    float R[9] = {
      1.f - 2.f*(yy+zz), 2.f*(xy - wz),   2.f*(xz + wy),
      2.f*(xy + wz),     1.f - 2.f*(xx+zz), 2.f*(yz - wx),
      2.f*(xz - wy),     2.f*(yz + wx),   1.f - 2.f*(xx+yy)
    };
    // Row-major 4x4
    T[0]=R[0]; T[1]=R[1]; T[2]=R[2]; T[3]=p.x;
    T[4]=R[3]; T[5]=R[4]; T[6]=R[5]; T[7]=p.y;
    T[8]=R[6]; T[9]=R[7]; T[10]=R[8];T[11]=p.z;
    T[12]=0;   T[13]=0;   T[14]=0;   T[15]=1;
  }

  static inline std::string voxelKey(int vx, int vy, int vz)
  {
    // 12-byte key (3x int32) stored in string
    char buf[12];
    std::memcpy(buf+0,  &vx, 4);
    std::memcpy(buf+4,  &vy, 4);
    std::memcpy(buf+8,  &vz, 4);
    return std::string(buf, 12);
  }

  void loadMask()
  {
    namespace fs = std::filesystem;
    try {
      fs::path p(mask_path_);
      if (!fs::exists(p)) {
        RCLCPP_WARN(get_logger(), "Mask file not found: %s (continuing without mask)", mask_path_.c_str());
        return;
      }
      mask_img_ = cv::imread(mask_path_, cv::IMREAD_GRAYSCALE);
      if (mask_img_.empty()) {
        RCLCPP_WARN(get_logger(), "Failed to read mask image: %s (continuing without mask)", mask_path_.c_str());
        return;
      }
      cv::threshold(mask_img_, mask_img_, 127, 255, cv::THRESH_BINARY);
      mask_cache_.clear();
      RCLCPP_INFO(get_logger(), "Loaded mask: %s (%dx%d)", mask_path_.c_str(), mask_img_.cols, mask_img_.rows);
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Exception reading mask: %s", mask_path_.c_str());
    }
  }

  const cv::Mat* maskForShape(int h, int w)
  {
    if (mask_img_.empty()) return nullptr;
    std::ostringstream key; key << w << "x" << h;
    auto it = mask_cache_.find(key.str());
    if (it != mask_cache_.end()) return &it->second;
    cv::Mat resized;
    if (mask_img_.rows == h && mask_img_.cols == w)
      resized = mask_img_;
    else
      cv::resize(mask_img_, resized, cv::Size(w,h), 0,0, cv::INTER_NEAREST);
    auto res = mask_cache_.emplace(key.str(), resized);
    return &res.first->second;
  }

  const Grids& normGrids(int w, int h, float fx, float fy, float cx, float cy)
  {
    std::ostringstream key; key << w << "x" << h << ":" << fx << "," << fy << "," << cx << "," << cy;
    auto it = grid_cache_.find(key.str());
    if (it != grid_cache_.end()) return it->second;

    cv::Mat u(1, w, CV_32F), v(1, h, CV_32F);
    for (int i=0;i<w;++i) u.at<float>(0,i) = static_cast<float>(i);
    for (int j=0;j<h;++j) v.at<float>(0,j) = static_cast<float>(j);
    cv::Mat us, vs;
    cv::repeat(u, h, 1, us);     // h x w
    cv::repeat(v.t(), 1, w, vs); // h x w

    Grids g;
    g.u_norm = (us - cx) / fx;
    g.v_norm = (vs - cy) / fy;

    auto res = grid_cache_.emplace(key.str(), g);
    return res.first->second;
  }

  
  void cameraInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    fx_ = static_cast<float>(msg->k[0]);
    fy_ = static_cast<float>(msg->k[4]);
    cx_ = static_cast<float>(msg->k[2]);
    cy_ = static_cast<float>(msg->k[5]);
    camera_frame_ = msg->header.frame_id;
    have_K_ = (fx_ > 0.f && fy_ > 0.f);

    RCLCPP_INFO(this->get_logger(),
                "CameraInfo received (fx=%.3f, fy=%.3f, cx=%.3f, cy=%.3f), unsubscribing.",
                fx_, fy_, cx_, cy_);
    // Unsubscribe to save bandwidth/CPU
    camera_info_sub_.reset();
  }
  // ---------------- Main callback ----------------
  // ---------------- Main callback (image + depth + odom) ----------------
  void syncedCb(const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
                const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
                const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
  {
    ++frame_count_;

    // map_T_base from odom
    {
      const auto &p = odom_msg->pose.pose.position;
      const auto &q = odom_msg->pose.pose.orientation;
      // reuse transformToMat44 using a synthetic TransformStamped
      geometry_msgs::msg::TransformStamped tf;
      tf.transform.translation.x = p.x;
      tf.transform.translation.y = p.y;
      tf.transform.translation.z = p.z;
      tf.transform.rotation = q;
      transformToMat44(tf, latest_T_);
    }

    // --- Read color ---
    cv::Mat rgb_bgr;
    try {
      auto cvp = cv_bridge::toCvCopy(*rgb_msg, "bgr8");
      rgb_bgr = cvp->image;
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Failed to convert RGB image: %s", e.what());
      return;
    }
    if (rgb_bgr.empty()) {
      RCLCPP_WARN(this->get_logger(), "RGB image empty.");
      return;
    }

    // --- Read depth ---
    cv::Mat depth_raw;
    try {
      // keep native encoding, then convert below
      auto cvp = cv_bridge::toCvCopy(*depth_msg);
      depth_raw = cvp->image;
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Failed to convert depth image: %s", e.what());
      return;
    }
    if (depth_raw.empty()) {
      RCLCPP_WARN(this->get_logger(), "Depth image empty.");
      return;
    }

    const int h = depth_raw.rows, w = depth_raw.cols;
    if (w <= 0 || h <= 0) {
    RCLCPP_WARN(get_logger(), "Depth image has invalid size %dx%d; skipping.", w, h);
    return;
    }

    // Resize RGB to depth size as needed
    if (rgb_bgr.channels() == 1) cv::cvtColor(rgb_bgr, rgb_bgr, cv::COLOR_GRAY2BGR);
    if (rgb_bgr.rows != h || rgb_bgr.cols != w)
    cv::resize(rgb_bgr, rgb_bgr, cv::Size(w,h), 0,0, cv::INTER_NEAREST);

    // Optional mask
    const cv::Mat* raw_mask = maskForShape(h, w);
    cv::Mat inv_mask;
    if (raw_mask && !raw_mask->empty()) {
    cv::bitwise_not(*raw_mask, inv_mask);
    cv::bitwise_and(rgb_bgr, rgb_bgr, rgb_bgr, inv_mask);
    }

    // Depth → meters (float32)
    cv::Mat Z;
    if (depth_raw.type() == CV_16UC1) {
    if (depth_scale_uint16_ <= 0.0) {
        RCLCPP_WARN(get_logger(), "depth_scale_uint16 <= 0; using default 0.001");
    }
    depth_raw.convertTo(Z, CV_32F, depth_scale_uint16_ > 0.0 ? depth_scale_uint16_ : 1.0);
    } else if (depth_raw.type() == CV_32FC1) {
    Z = depth_raw; // already meters
    } else {
    // Best-effort convert
    depth_raw.convertTo(Z, CV_32F);
    }
    Z *= depth_scale_uint16_;  // or: cv::multiply(Z, 1000.0, Z);
    // Build validity mask (8U) in range (min_depth_, max_depth_)
    cv::Mat valid;
    cv::inRange(Z, (float)min_depth_, (float)max_depth_, valid); // 255 where in range
    double minZ = 0.0, maxZ = 0.0;
    // max over ALL pixels in Z:
    cv::minMaxLoc(Z, &minZ, &maxZ);
    RCLCPP_INFO(get_logger(), "Z (all) min=%.3f m  max=%.3f m", minZ, maxZ);
    // AND with mask==0 if we have a mask
    if (!inv_mask.empty()) {
    // inv_mask is 255 where we KEEP pixels → just AND with valid
    cv::bitwise_and(valid, inv_mask, valid);
    }

    if (cv::countNonZero(valid) == 0) {
    RCLCPP_INFO(get_logger(), "No valid depth after masking/range.");
    return;
    }

    // Back-projection (grids must exist & match size)
    const auto &grids = normGrids(w, h, fx_, fy_, cx_, cy_);
    if (grids.u_norm.empty() || grids.v_norm.empty()) {
    RCLCPP_WARN(get_logger(), "Grids empty (w=%d h=%d fx=%.3f fy=%.3f); skipping.", w, h, fx_, fy_);
    return;
    }

    cv::Mat X = grids.u_norm.mul(Z);
    cv::Mat Y = grids.v_norm.mul(Z);

    // Collect linear indices of valid pixels
    std::vector<int> idx;
    idx.reserve(h*w/4);
    for (int v=0; v<h; ++v) {
    const uint8_t* vp = valid.ptr<uint8_t>(v);
    for (int u=0; u<w; ++u) if (vp[u]) idx.push_back(v*w + u);
    }
    if (idx.empty()) {
    RCLCPP_INFO(get_logger(), "Valid index set empty after scan.");
    return;
    }

    // Prepare RGB (convert BGR→RGB once)
    cv::Mat rgb_rgb;
    cv::cvtColor(rgb_bgr, rgb_rgb, cv::COLOR_BGR2RGB);


    // TF: base <- camera (cache + optional refresh)
    std::string cam_frame;
    if (depth_msg->header.frame_id.size())
      cam_frame = depth_msg->header.frame_id;
    else if (depth_msg->header.frame_id.size())
      cam_frame = depth_msg->header.frame_id;
    else
      cam_frame = "camera1_link_output";

    try {
      if (!tf_cam_in_base_.has_value() ||
          (tf_refresh_every_n_ > 0 && (frame_count_ % tf_refresh_every_n_) == 0)) {
        tf_cam_in_base_ = tf_buffer_->lookupTransform(
            base_frame_, cam_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "TF lookup failed %s->%s: %s", cam_frame.c_str(), base_frame_.c_str(), e.what());
      return;
    }

    // Compose map <- camera
    float base_T_cam[16];
    transformToMat44(*tf_cam_in_base_, base_T_cam);

    // map_T_cam = map_T_base * base_T_cam (row-major)
    float map_T_cam[16];
    multiply44(latest_T_, base_T_cam, map_T_cam);

    // Apply to points
    const float *R = map_T_cam;              // 3x3 in first 3 columns
    const float tx = map_T_cam[3];
    const float ty = map_T_cam[7];
    const float tz = map_T_cam[11];

    // Per-frame voxel dedup
    std::unordered_map<std::string, XYZRGB> frame_voxels;

    for (int k : idx) {
      const int v = k / w, u = k % w;
      const float x_cam = X.at<float>(v,u);
      const float y_cam = Y.at<float>(v,u);
      const float z_cam = Z.at<float>(v,u);

      // P_map = R * P_cam + t   (R row-major)
      const float xm = R[0]*x_cam + R[1]*y_cam + R[2]*z_cam + tx;
      const float ym = R[4]*x_cam + R[5]*y_cam + R[6]*z_cam + ty;
      const float zm = R[8]*x_cam + R[9]*y_cam + R[10]*z_cam + tz;

      // voxel
      const int vx = static_cast<int>(std::floor(xm / voxel_size_));
      const int vy = static_cast<int>(std::floor(ym / voxel_size_));
      const int vz = static_cast<int>(std::floor(zm / voxel_size_));
      const std::string key = voxelKey(vx,vy,vz);

      const cv::Vec3b &c = rgb_rgb.at<cv::Vec3b>(v,u); // RGB
      frame_voxels[key] = XYZRGB{xm, ym, zm, c[0], c[1], c[2]}; // last wins
    }

    // Merge frame_voxels into global voxel_map_
    // Also build delta vector (points to publish for this frame)
    std::vector<XYZRGB> frame_pts;
    frame_pts.reserve(frame_voxels.size());
    for (auto &kv : frame_voxels) {
      voxel_map_[kv.first] = kv.second; // overwrite or insert
      frame_pts.push_back(kv.second);
    }

    // Optional radius pruning around robot
    if (map_max_radius_ > 0.0 && !voxel_map_.empty()) {
      const float cx0 = latest_T_[3], cy0 = latest_T_[7], cz0 = latest_T_[11];
      const float r2 = static_cast<float>(map_max_radius_ * map_max_radius_);
      std::vector<std::string> to_erase;
      to_erase.reserve(voxel_map_.size()/10 + 1);
      for (auto &kv : voxel_map_) {
        const auto &p = kv.second;
        const float dx = p.x - cx0, dy = p.y - cy0, dz = p.z - cz0;
        if (dx*dx + dy*dy + dz*dz > r2) to_erase.push_back(kv.first);
      }
      for (auto &k : to_erase) voxel_map_.erase(k);
    }

    // Publish deltas
    const auto stamp = odom_msg->header.stamp;
    if (publish_deltas_ && !frame_pts.empty()) {
      auto cloud = packCloudXYZRGB(frame_pts, map_frame_, stamp);
      pub_cloud_delta_->publish(cloud);
    }

    // Publish full every N
    if (publish_full_every_n_ > 0 && (frame_count_ % publish_full_every_n_) == 0) {
      std::vector<XYZRGB> all_pts;
      all_pts.reserve(voxel_map_.size());
      for (auto &kv : voxel_map_) all_pts.push_back(kv.second);
      if (!all_pts.empty()) {
        auto cloud = packCloudXYZRGB(all_pts, map_frame_, stamp);
        pub_cloud_full_->publish(cloud);
      }
    }
  }

  static void multiply44(const float A[16], const float B[16], float C[16])
  {
    // Row-major: C = A * B
    for (int r=0;r<4;++r) {
      for (int c=0;c<4;++c) {
        C[r*4+c] = A[r*4+0]*B[0*4+c] + A[r*4+1]*B[1*4+c] + A[r*4+2]*B[2*4+c] + A[r*4+3]*B[3*4+c];
      }
    }
  }

  static sensor_msgs::msg::PointCloud2 packCloudXYZRGB(const std::vector<XYZRGB> &pts,
                                                       const std::string &frame_id,
                                                       const builtin_interfaces::msg::Time &stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame_id;
    cloud.header.stamp    = stamp;
    cloud.height = 1;
    cloud.width  = static_cast<uint32_t>(pts.size());

    sensor_msgs::PointCloud2Modifier mod(cloud);
    // x,y,z + rgb (packed float32 as in Python)
    mod.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32
    );
    mod.resize(cloud.width);

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> it_rgb(cloud, "rgb");

    for (const auto &p : pts) {
      *it_x = p.x; *it_y = p.y; *it_z = p.z;

      // pack rgb into float32
      uint32_t rgb = (static_cast<uint32_t>(p.r) << 16) |
                     (static_cast<uint32_t>(p.g) << 8)  |
                      static_cast<uint32_t>(p.b);
      float rgb_f;
      std::memcpy(&rgb_f, &rgb, sizeof(float));
      *it_rgb = rgb_f;

      ++it_x; ++it_y; ++it_z; ++it_rgb;
    }
    cloud.is_bigendian = false;
    cloud.is_dense = false;
    return cloud;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RGBDImageToPointCloudNode>());
  rclcpp::shutdown();
  return 0;
}
