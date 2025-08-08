// src/ring_path_planner.cpp

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <wetexplorer_navigation/action/move_tcp.hpp>
#include <wetexplorer_navigation/action/spin_yaw.hpp>
#include <wetexplorer_navigation/action/spin_yaw.hpp>
#include <wetexplorer_navigation/action/update_map.hpp>
#include <wetexplorer_hardware/action/move_joint.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // updated include


#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <chrono>          // for chrono literals
#include <random>
#include "nav2_msgs/action/back_up.hpp"
#include "wetexplorer_navigation/action/localize_object.hpp"  // the action definition

using namespace std::chrono_literals;

using GetState   = lifecycle_msgs::srv::GetState;
using Navigate   = nav2_msgs::action::NavigateToPose;
//using Spin   = nav2_msgs::action::Spin;
using Spin   = wetexplorer_navigation::action::SpinYaw;
//using Spin   = nav2_msgs::action::Spin;
using Spin   = wetexplorer_navigation::action::SpinYaw;
using MoveTCP    = wetexplorer_navigation::action::MoveTCP;
using UpdateMap  = wetexplorer_navigation::action::UpdateMap;
using MoveJoint = wetexplorer_hardware::action::MoveJoint;
using LocalizeObj = wetexplorer_navigation::action::LocalizeObject;

using NavGoalH   = rclcpp_action::ClientGoalHandle<Navigate>;
using SpinGoalH   = rclcpp_action::ClientGoalHandle<Spin>;
using LocalGoalH = rclcpp_action::ClientGoalHandle<MoveTCP>;
using MapGoalH   = rclcpp_action::ClientGoalHandle<UpdateMap>;
using LocalizeObjGoalHandle = rclcpp_action::ClientGoalHandle<LocalizeObj>;


class RingPathPlanner : public rclcpp::Node
{
public:
  enum class State { IDLE, START, GLOBAL_APPROACH, ROUGH_OBJECT_LOCALIZATION, SPIN, FORWARD, OBJECT_LOCALIZATION, LOCAL_APPROACH, PICK_UP, PUT_DOWN, BACKUP, FINISHED };
  enum class State { IDLE, START, GLOBAL_APPROACH, ROUGH_OBJECT_LOCALIZATION, SPIN, FORWARD, OBJECT_LOCALIZATION, LOCAL_APPROACH, PICK_UP, PUT_DOWN, BACKUP, FINISHED };
  
  RingPathPlanner()
  : Node("ring_path_planner"),
    state_(State::IDLE),
    got_odom_(false),
    start_flag_(false),
    prev_button_state_(false),
    current_goal_idx_(0),
    rd_(),
    gen_(rd_()),
    dist_(0.0, 1.5),
    start_yaw_(0.0),
    current_yaw_(0.0),
    pos_error_(0.0),
    pos_error_(0.0),
    yaw_correction_(0.0),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Declare & read the “ref” parameter
    declare_parameter<std::string>("ref", "map");
    get_parameter("ref", ref_frame_);

    // Choose which odometry topic to subscribe to
    std::string odom_topic = (ref_frame_ == "odom") ? "/odometry/local"
                                                    : "/odometry/global";

    // Subscriptions
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&RingPathPlanner::odomCallback, this, std::placeholders::_1));

    marker_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/map_rings", 10,
      std::bind(&RingPathPlanner::markerCallback, this, std::placeholders::_1));
    
    rclcpp::SubscriptionOptions joy_opts;
    joy_opts.callback_group = joy_cb_group_;

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy_teleop/joy",      // topic
      10,                     // queue depth
      std::bind(&RingPathPlanner::joy_callback, this, std::placeholders::_1),
      joy_opts);              // ← tie to the re-entrant group



    // Publisher for current goal arrow
    current_goal_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/current_goal", 10);

    // Action & service clients
    nav2_client_       = rclcpp_action::create_client<Navigate>(this, "/navigate_to_pose");
    nav2_spin_client_       = rclcpp_action::create_client<Spin>(this, "/spin_control"); //spin
    nav2_spin_client_       = rclcpp_action::create_client<Spin>(this, "/spin_control"); //spin
    local_client_      = rclcpp_action::create_client<MoveTCP>(this, "/MoveTCP");
    update_map_client_ = rclcpp_action::create_client<UpdateMap>(this, "/update_map");
    localize_client_ = rclcpp_action::create_client<LocalizeObj>(this, "/localize_object");
    localize_client_ = rclcpp_action::create_client<LocalizeObj>(this, "/localize_object");
    bt_client_         = create_client<GetState>("/bt_navigator/get_state");
    back_up_client_ = rclcpp_action::create_client<BackUp>(this, "backup");
    action_client_ = rclcpp_action::create_client<MoveJoint>(this, "/move_joint");
    // Start polling for BT‐Navigator to become ACTIVE
    // bt_timer_ = create_wall_timer(
    //   500ms, std::bind(&RingPathPlanner::checkBT, this));
  }

private:
  // --------------------------------------------------------------------------
  // Callbacks & state‐machine handling
  // --------------------------------------------------------------------------

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pos_.first  = msg->pose.pose.position.x;
    current_pos_.second = msg->pose.pose.position.y;
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;

    if (!got_odom_) {
      //start_pos_ = current_pos_;
      got_odom_  = true;
      start_yaw_ = yaw;
      RCLCPP_INFO(get_logger(),
        "Start set to (%.2f, %.2f)", start_pos_.first, start_pos_.second);
    }
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    int current_button_state = msg->buttons[9];

    // Detect rising edge of button 0
    if (current_button_state == 1 && prev_button_state_ == 0 && start_flag_ == false) {
      RCLCPP_INFO(get_logger(), "Button Start pressed. STARTING MISSION...");
      start_flag_ = true;
      start_pos_ = current_pos_;
      transitionTo(State::START);
    }
    prev_button_state_ = current_button_state;
  }

  void markerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    if (!start_flag_){
      ring_positions_.clear();
      for (auto & m : msg->markers) {
        ring_positions_.emplace_back(m.pose.position.x, m.pose.position.y);
      }

      if (ring_positions_.empty()) {
        RCLCPP_WARN(get_logger(), "No rings found on /map_rings");
        return;
      }

    }
    

    
  }

  void checkBT()
  {
    if (!bt_client_->service_is_ready()) {
      return;
    }
    bt_timer_->cancel();

    auto req = std::make_shared<GetState::Request>();
    bt_client_->async_send_request(
      req,
      [this](rclcpp::Client<GetState>::SharedFuture fut) {
        if (fut.get()->current_state.id == 3) {
          RCLCPP_INFO(get_logger(), "BT ACTIVE → starting state machine");
          transitionTo(State::IDLE);
        } else {
          RCLCPP_WARN(get_logger(), "BT not active yet, retrying...");
          bt_timer_->reset();
        }
      });
  }

  void transitionTo(State s)
  {
    state_ = s;
    switch (s) {

      case State::START:
        doStart();
        break;
      case State::GLOBAL_APPROACH:
        doGlobalApproach();
        break;

      case State::ROUGH_OBJECT_LOCALIZATION:
        doRoughObjectLocalization();
        break;
      case State::SPIN:
        doSpin();
        break;
      case State::FORWARD:
        doForward();
        break;
      case State::FORWARD:
        doForward();
        break;
      case State::OBJECT_LOCALIZATION:
        doObjectLocalization();
        break;
      case State::LOCAL_APPROACH:
        doLocalApproach();
        break;
      case State::PUT_DOWN:
        doPutDown();
        break;
      case State::PICK_UP:
        doPickUp();
        break;
      case State::BACKUP:
        doBackup();
        break;
      case State::FINISHED:
        RCLCPP_INFO(get_logger(), "All goals visited → state=FINISHED");
        break;
      default:
        break;
    }
  }

  void doStart()
  {
    // Compute TSP tour over ring_positions_
    best_path_ = findTSP(ring_positions_);
    RCLCPP_INFO(get_logger(),
      "Tour with %zu rings computed", best_path_.size());   // maintain 1 Hz loop rate

    
    transitionTo(State::GLOBAL_APPROACH);
  }



  // --------------------------------------------------------------------------
  // 1) Global approach: send Nav2 a “1 m behind the ring” goal
  // --------------------------------------------------------------------------
 void doGlobalApproach()
{
  // Extract current ring goal
  auto [gx, gy] = best_path_[current_goal_idx_];
  double dx  = gx - current_pos_.first;
  double dy  = gy - current_pos_.second;
  double yaw = std::atan2(dy, dx);
  double yaw_circle = yaw;
  double mag = std::hypot(dx, dy);
  double dyaw = yaw - current_yaw_;
  double offset_angle_deg = 45;
  double offset_angle_rad = offset_angle_deg * M_PI / 180.0;

  double distance_approach = 0.85;
  // Place the goal 0.85 m behind the ring
  double px = gx - distance_approach * std::cos(yaw);
  double py = gy - distance_approach * std::sin(yaw);
  if (current_goal_idx_ + 1 == best_path_.size()){
    px = gx;
    py = gy;
  }
  

  else if (std::abs(mag) < 1.0 && std::abs(dyaw) > 0.39) {
    // Normalize direction vector
    double dir_x = dx / mag;
    double dir_y = dy / mag;

    // Rotate by ±angle
    double rot1_x = distance_approach*std::cos(offset_angle_rad + yaw_circle);
    double rot1_y = distance_approach*std::sin(offset_angle_rad + yaw_circle);

    double rot2_x = distance_approach*std::cos(-offset_angle_rad + yaw_circle);
    double rot2_y = distance_approach*std::sin(-offset_angle_rad + yaw_circle);

    // Heading vector from current yaw
    double hx = std::cos(current_yaw_+3.1459);
    double hy = std::sin(current_yaw_+3.1459);

    // Dot product to decide direction
    double dot1 = hx * rot1_x + hy * rot1_y;
    double dot2 = hx * rot2_x + hy * rot2_y;

    if (dot1 > dot2) {
      px = gx - rot1_x;
      py = gy - rot1_y;
    } else {
      px = gx - rot2_x;
      py = gy - rot2_y;
    }

    
  }
  // Yaw should face the ring center
  yaw = std::atan2(gy - py, gx - px);


  publishMarker(px, py, yaw);

  if (!nav2_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(get_logger(), "Nav2 action server not available");
    transitionTo(State::FINISHED);
    return;
  }

  Navigate::Goal goal_msg;
  goal_msg.pose.header.frame_id    = ref_frame_;
  goal_msg.pose.header.stamp       = now();
  goal_msg.pose.pose.position.x    = px;
  goal_msg.pose.pose.position.y    = py;
  goal_msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal_msg.pose.pose.orientation.w = std::cos(yaw / 2.0);

  auto opts = rclcpp_action::Client<Navigate>::SendGoalOptions{};
  opts.goal_response_callback = [](auto) { /* no-op */ };

  // Feedback callback with logging
  opts.feedback_callback = [this](NavGoalH::SharedPtr, const std::shared_ptr<const Navigate::Feedback> feedback)
  {
    const auto& pose = feedback->current_pose.pose.position;
    float dist_remain = feedback->distance_remaining;
    int recoveries = feedback->number_of_recoveries;
    // RCLCPP_INFO(get_logger(),
    //   "[Nav2 Feedback] Current position: (%.2f, %.2f), distance remaining: %.2f, recoveries: %d",
    //   pose.x, pose.y, dist_remain, recoveries);
  };

  // Result callback
  opts.result_callback = [this](const NavGoalH::WrappedResult & res)
  {
    if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
      if (current_goal_idx_ + 1 < best_path_.size()) {
        RCLCPP_INFO(get_logger(),
          "Global approach succeeded → waiting 7 s before update_map");
        delay_timer_ = create_wall_timer(
          3s,
          [this]() {
            delay_timer_->cancel();
            transitionTo(State::ROUGH_OBJECT_LOCALIZATION);
          });
      } else {
        transitionTo(State::FINISHED);
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Global approach failed");
      transitionTo(State::FINISHED);
    }
  };

  nav2_client_->async_send_goal(goal_msg, opts);
}


  

  // --------------------------------------------------------------------------
  // 2) Object localization (now calls “update_map” action instead of Trigger service)
  // --------------------------------------------------------------------------
  void doObjectLocalization()
  {
    RCLCPP_INFO(get_logger(), "Starting Object Localization ...");
    // Wait for update_map action server
    if (!update_map_client_->wait_for_action_server(20s)) {
      RCLCPP_ERROR(get_logger(), "update_map action server unavailable");
      transitionTo(State::FINISHED);
      return;
    }

    // Send empty goal (no fields) to /update_map
    UpdateMap::Goal update_goal;

    auto send_opts = rclcpp_action::Client<UpdateMap>::SendGoalOptions{};
    send_opts.goal_response_callback = [](auto) { /* no‐op */ };
    send_opts.feedback_callback      = [](auto, auto) { /* no‐op */ };

    // When update_map completes, recopute TSP & move on to LOCAL_APPROACH
    send_opts.result_callback =
      [this](const MapGoalH::WrappedResult & wrapped) {
        if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {   
          auto & result = wrapped.result; 
         

          //auto & result = wrapped.result;
          // Overwrite the *current* best_path_ entry with the newly localized pose:
          if (current_goal_idx_ < best_path_.size() &&
              current_goal_idx_ < result->poses.size()) {
            const auto & p = result->poses[rings_index_[current_goal_idx_]].pose.position;
            best_path_[current_goal_idx_] = {p.x, p.y};
            RCLCPP_INFO(get_logger(),
              "Re‐localized ring %zu to (%.2f, %.2f)",
              current_goal_idx_+1, p.x, p.y);
          } else {
            RCLCPP_WARN(get_logger(),
              "update_map returned unexpected number of poses; skipping update");
          }
          //best_path_ = findTSP(ring_positions_);
          
          transitionTo(State::LOCAL_APPROACH);
        } else {
          RCLCPP_ERROR(get_logger(),
            "update_map failed (code %d)",
            static_cast<int>(wrapped.code));
          transitionTo(State::FINISHED);
        }
      };

    update_map_client_->async_send_goal(update_goal, send_opts);
  }

  inline double bearingRad(double x1, double y1, double x2, double y2)
  {
    return std::atan2(y2 - y1, x2 - x1);
  }
  
  void processLocalizedObject(const geometry_msgs::msg::PoseStamped &ps_object_in){
      //--------------------------------------------------------------------
    // 1.  Transform object pose → odom
    //--------------------------------------------------------------------
    geometry_msgs::msg::PoseStamped ps_odom;
    try
    {
      // ps_object_in.header.frame_id is whatever the server returned (“map”, “camera_rgb_optical_frame”, …)
      auto tf = tf_buffer_.lookupTransform(
          "odom",                                // ─ target frame
          ps_object_in.header.frame_id,          // ─ source frame
          ps_object_in.header.stamp,             // ─ use same time as object pose
          rclcpp::Duration::from_seconds(0.2));  // ─ wait up to 0.2 s

      tf2::doTransform(ps_object_in, ps_odom, tf);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(get_logger(), "TF2 error while converting object pose to odom: %s", ex.what());
      return;
    }

    //--------------------------------------------------------------------
    // 2.  Compute bearing robot → object  (robot position already in odom)
    //-------------------------------------------------------------------
    
    
    
    double dx = ps_odom.pose.position.x - current_pos_.first;
    double dy = ps_odom.pose.position.y - current_pos_.second;
    best_path_[current_goal_idx_] = {ps_odom.pose.position.x, ps_odom.pose.position.y};
    double yaw_to_object = std::atan2(dy, dx);            // radians, (-π, π]
    //-------------------------------------------------------------------
    
    
    
    double dx = ps_odom.pose.position.x - current_pos_.first;
    double dy = ps_odom.pose.position.y - current_pos_.second;
    best_path_[current_goal_idx_] = {ps_odom.pose.position.x, ps_odom.pose.position.y};
    double yaw_to_object = std::atan2(dy, dx);            // radians, (-π, π]
    yaw_correction_ = yaw_to_object;
    //--------------------------------------------------------------------
    // 3.  Use it (log, store, convert to quaternion …)
    //--------------------------------------------------------------------
    RCLCPP_INFO(get_logger(),
                "Bearing to object: %.3f rad (%.1f deg)", yaw_to_object,
                yaw_to_object * 180.0 / M_PI);

    // If you want a quaternion with only that yaw:
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_to_object);
    geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q);

    // … use q_msg however you need (goal orientation, published marker, etc.)
  }
  void doRoughObjectLocalization(){
    if (!localize_client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(),
        "/localize_object action server not ready, cannot send goal");
      return;
    }

    // Construct an empty goal (LocalizeObject has no fields)
    auto goal_msg = LocalizeObj::Goal();

    // Set up callbacks
    auto send_options = rclcpp_action::Client<LocalizeObj>::SendGoalOptions{};
    send_options.goal_response_callback =
      [this](std::shared_ptr<LocalizeObjGoalHandle> goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "LocalizeObject goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "LocalizeObject goal accepted; waiting for result");
        }
      };

    send_options.feedback_callback =
  [this](LocalizeObjGoalHandle::SharedPtr,
         const std::shared_ptr<const LocalizeObj::Feedback> /*feedback*/) {
    RCLCPP_DEBUG(this->get_logger(), "LocalizeObject feedback received");
  };

    send_options.result_callback =
      [this](
        const LocalizeObjGoalHandle::WrappedResult & wrapped_result)
      {
        switch (wrapped_result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
          {
            auto result = wrapped_result.result;
            auto pose_stamped = result->pose;
            RCLCPP_INFO(get_logger(),
              "LocalizeObject succeeded: pose = (%.3f, %.3f, %.3f) orientation = (%.3f, %.3f, %.3f, %.3f) in frame '%s'",
              pose_stamped.pose.position.x,
              pose_stamped.pose.position.y,
              pose_stamped.pose.position.z,
              pose_stamped.pose.orientation.x,
              pose_stamped.pose.orientation.y,
              pose_stamped.pose.orientation.z,
              pose_stamped.pose.orientation.w,
              pose_stamped.header.frame_id.c_str());
            
            processLocalizedObject(wrapped_result.result->pose);
            
            
            transitionTo(State::SPIN);
            break;
          }
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "LocalizeObject was aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "LocalizeObject was canceled");
            break;
          default:
            RCLCPP_ERROR(get_logger(), "Unknown result code for LocalizeObject");
            break;
        }
      };

    // Actually send the goal
    localize_client_->async_send_goal(goal_msg, send_options);
  }


  void doSpin()
  {
    //------------------------------------------------------------------
    // 1.  Make sure the behavior server is up
    //------------------------------------------------------------------
    if (!nav2_spin_client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(), "/spin action server not ready");
      return;
    }

    auto [gx, gy] = best_path_[current_goal_idx_];
    double dx  = gx - current_pos_.first;
    double dy  = gy - current_pos_.second;
    pos_error_   = std::sqrt(dx*dx + dy*dy);

    auto [gx, gy] = best_path_[current_goal_idx_];
    double dx  = gx - current_pos_.first;
    double dy  = gy - current_pos_.second;
    pos_error_   = std::sqrt(dx*dx + dy*dy);

    //------------------------------------------------------------------
    // 2.  Build the goal
    //------------------------------------------------------------------
    Spin::Goal goal_msg;
    goal_msg.target_yaw     = static_cast<float>(yaw_correction_);           // required
    // goal_msg.time_allowance =                                            // optional
    //     rclcpp::Duration::from_seconds(15.0);  // 0 → no timeout
    // goal_msg.time_allowance =                                            // optional
    //     rclcpp::Duration::from_seconds(15.0);  // 0 → no timeout

      

    //------------------------------------------------------------------
    // 3.  Configure callbacks
    //------------------------------------------------------------------
    auto options                       = rclcpp_action::Client<Spin>::SendGoalOptions{};
    options.goal_response_callback     =
        [this](std::shared_ptr<SpinGoalH> gh)
        {
          if (!gh) {
            RCLCPP_ERROR(get_logger(), "Spin goal rejected");
          } else {
            RCLCPP_INFO(get_logger(), "Spin goal accepted; waiting for result…");
          }
        };

    options.feedback_callback          =
        [this](SpinGoalH::SharedPtr,
                const std::shared_ptr<const Spin::Feedback> feedback)
        {
          // feedback->angular_distance_traveled is in radians
          RCLCPP_DEBUG(get_logger(), "Spun %.2f / %.2f rad",
                      feedback->remaining_angle, //angular_distance_traveled
                      feedback->remaining_angle, //angular_distance_traveled
                      yaw_correction_);
        };

    options.result_callback            =
        [this](const SpinGoalH::WrappedResult &wr)
        {
          switch (wr.code) {
            case rclcpp_action::ResultCode::SUCCEEDED: {
              const auto &res = wr.result;              
              RCLCPP_INFO(get_logger(),
                          "Spin Finished");
                          "Spin Finished");
              delay_timer_ = create_wall_timer(
              4s,
              [this]() {
                delay_timer_->cancel();

                if (pos_error_ > 1.10){
                  transitionTo(State::FORWARD);}
                else{
                transitionTo(State::OBJECT_LOCALIZATION);}
              });
              
              
              break;
            }
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(get_logger(), "Spin was aborted");
              break;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_WARN(get_logger(), "Spin was canceled");
              break;
            default:
              RCLCPP_ERROR(get_logger(), "Unknown result code for Spin");
              break;
          }
        };

    //------------------------------------------------------------------
    // 4.  Fire it off
    //------------------------------------------------------------------
    nav2_spin_client_->async_send_goal(goal_msg, options);
  }


  void doForward()
  {
    auto [gx, gy] = best_path_[current_goal_idx_];
    double dx  = gx - current_pos_.first;
    double dy  = gy - current_pos_.second;
    double yaw = std::atan2(dy, dx);
    double px = gx - 0.45*std::cos(yaw);
    double py = gy - 0.45*std::sin(yaw);

    if (!local_client_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(get_logger(), "MoveTCP action server unavailable");
      transitionTo(State::FINISHED);
      return;
    }

    MoveTCP::Goal goal_msg;
    goal_msg.target_pose.header.frame_id = ref_frame_;
    goal_msg.target_pose.header.stamp    = now();
    goal_msg.target_pose.pose.position.x = px;
    goal_msg.target_pose.pose.position.y = py;
    goal_msg.target_pose.pose.orientation.w = 1.0;  // facing default

    auto opts = rclcpp_action::Client<MoveTCP>::SendGoalOptions{};
    opts.goal_response_callback = [](auto) { /* ignore */ };

    opts.feedback_callback =
      [](auto, auto fb) {
        RCLCPP_DEBUG(rclcpp::get_logger("ring_path_planner"),
          "remaining distance=%.2f", fb->remaining_distance);
      };

    RCLCPP_INFO(get_logger(),
      "Ring number=%zu local approach", current_goal_idx_ + 1);

    opts.result_callback =
      [this](const LocalGoalH::WrappedResult & res) {
        if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(get_logger(), "Local approach succeeded");
           
          delay_timer_ = create_wall_timer(
          4s,
          [this]() {
            delay_timer_->cancel();
            transitionTo(State::OBJECT_LOCALIZATION);
          });
          
          
          
        } else {
          RCLCPP_ERROR(get_logger(), "Local approach failed");
          transitionTo(State::FINISHED);
        }
      };

    local_client_->async_send_goal(goal_msg, opts);
  }



  void doForward()
  {
    auto [gx, gy] = best_path_[current_goal_idx_];
    double dx  = gx - current_pos_.first;
    double dy  = gy - current_pos_.second;
    double yaw = std::atan2(dy, dx);
    double px = gx - 0.45*std::cos(yaw);
    double py = gy - 0.45*std::sin(yaw);

    if (!local_client_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(get_logger(), "MoveTCP action server unavailable");
      transitionTo(State::FINISHED);
      return;
    }

    MoveTCP::Goal goal_msg;
    goal_msg.target_pose.header.frame_id = ref_frame_;
    goal_msg.target_pose.header.stamp    = now();
    goal_msg.target_pose.pose.position.x = px;
    goal_msg.target_pose.pose.position.y = py;
    goal_msg.target_pose.pose.orientation.w = 1.0;  // facing default

    auto opts = rclcpp_action::Client<MoveTCP>::SendGoalOptions{};
    opts.goal_response_callback = [](auto) { /* ignore */ };

    opts.feedback_callback =
      [](auto, auto fb) {
        RCLCPP_DEBUG(rclcpp::get_logger("ring_path_planner"),
          "remaining distance=%.2f", fb->remaining_distance);
      };

    RCLCPP_INFO(get_logger(),
      "Ring number=%zu local approach", current_goal_idx_ + 1);

    opts.result_callback =
      [this](const LocalGoalH::WrappedResult & res) {
        if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(get_logger(), "Local approach succeeded");
           
          delay_timer_ = create_wall_timer(
          4s,
          [this]() {
            delay_timer_->cancel();
            transitionTo(State::OBJECT_LOCALIZATION);
          });
          
          
          
        } else {
          RCLCPP_ERROR(get_logger(), "Local approach failed");
          transitionTo(State::FINISHED);
        }
      };

    local_client_->async_send_goal(goal_msg, opts);
  }

  // --------------------------------------------------------------------------
  // 3) Local approach: send MoveTCP goal to the ring itself
  // --------------------------------------------------------------------------
  void doLocalApproach()
  {
    auto [gx, gy] = best_path_[current_goal_idx_];

    if (!local_client_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(get_logger(), "MoveTCP action server unavailable");
      transitionTo(State::FINISHED);
      return;
    }

    MoveTCP::Goal goal_msg;
    goal_msg.target_pose.header.frame_id = ref_frame_;
    goal_msg.target_pose.header.stamp    = now();
    goal_msg.target_pose.pose.position.x = gx;
    goal_msg.target_pose.pose.position.y = gy;
    goal_msg.target_pose.pose.orientation.w = 1.0;  // facing default

    auto opts = rclcpp_action::Client<MoveTCP>::SendGoalOptions{};
    opts.goal_response_callback = [](auto) { /* ignore */ };

    opts.feedback_callback =
      [](auto, auto fb) {
        RCLCPP_DEBUG(rclcpp::get_logger("ring_path_planner"),
          "remaining distance=%.2f", fb->remaining_distance);
      };

    RCLCPP_INFO(get_logger(),
      "Ring number=%zu local approach", current_goal_idx_ + 1);

    opts.result_callback =
      [this](const LocalGoalH::WrappedResult & res) {
        if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(get_logger(), "Local approach succeeded");
          ++current_goal_idx_;  
          delay_timer_ = create_wall_timer(
          1s,
          [this]() {
            delay_timer_->cancel();
            transitionTo(State::BACKUP);
          });
          
          
          
        } else {
          RCLCPP_ERROR(get_logger(), "Local approach failed");
          transitionTo(State::FINISHED);
        }
      };

    local_client_->async_send_goal(goal_msg, opts);
  }

  
  void doPutDown()
  {
    RCLCPP_INFO(get_logger(), "Starting PutDown...");

    // Wait for MoveJoint action server
    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "MoveJoint action server unavailable");
      transitionTo(State::FINISHED);
      return;
    }

    // Create and populate the goal
    auto goal_msg = MoveJoint::Goal();
    goal_msg.distance_mm = 157;

    // Define send options with a result callback
    rclcpp_action::Client<MoveJoint>::SendGoalOptions send_opts;
    send_opts.goal_response_callback = [](auto) { /* no-op */ };
    send_opts.feedback_callback      = [](auto, auto) { /* optional */ };


    send_opts.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<MoveJoint>::WrappedResult & wrapped)
    {
      if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result->success) {
        RCLCPP_INFO(get_logger(), "Action succeeded");
        delay_timer_ = create_wall_timer(
          7s,
          [this]() {
            delay_timer_->cancel();
            transitionTo(State::PICK_UP);
          });
      } 
      else {
        RCLCPP_ERROR(get_logger(),
          "Action failed (code %d, success=%s)",
          static_cast<int>(wrapped.code),
          wrapped.result->success ? "true" : "false");
        transitionTo(State::FINISHED);
      }
    };




    // Send the goal asynchronously
    action_client_->async_send_goal(goal_msg, send_opts);
    
    RCLCPP_INFO(get_logger(), "Goal Set...");
    }

  void doPickUp()
{
  RCLCPP_INFO(get_logger(), "Starting PickUp...");

  // Wait for MoveJoint action server
  if (!action_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(get_logger(), "MoveJoint action server unavailable");
    transitionTo(State::FINISHED);
    return;
  }

  // Create and populate the goal
  MoveJoint::Goal goal_msg;
  goal_msg.distance_mm = 0.0;

  // Define send options with a result callback
  rclcpp_action::Client<MoveJoint>::SendGoalOptions send_opts;
  send_opts.goal_response_callback = [](auto) { /* no-op */ };
  send_opts.feedback_callback      = [](auto, auto) { /* optional */ };

  send_opts.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<MoveJoint>::WrappedResult & wrapped)
    {
      if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result->success) {
        RCLCPP_INFO(get_logger(), "Action succeeded");
        delay_timer_ = create_wall_timer(
          2s,
          [this]() {
            delay_timer_->cancel();
            transitionTo(State::BACKUP);
          });
      } 
      else {
        RCLCPP_ERROR(get_logger(),
          "Action failed (code %d, success=%s)",
          static_cast<int>(wrapped.code),
          wrapped.result->success ? "true" : "false");
        transitionTo(State::FINISHED);
      }
    };


  // Send the goal asynchronously
  action_client_->async_send_goal(goal_msg, send_opts);
}

  void doBackup()
  {
    // 1. Wait for the BackUp action server to come up
    if (!back_up_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "BackUp action server not available");
      transitionTo(State::FINISHED);
      return;
   }
  // 2. Fill out the BackUp goal
    using BackUpAction = nav2_msgs::action::BackUp;
    using BackUpGoalHandle = rclcpp_action::ClientGoalHandle<BackUpAction>;

    auto goal_msg = BackUpAction::Goal();
    goal_msg.target.x = 0.45;                                      // 30 cm backward
    goal_msg.speed = 0.1;                                         // 0.1 m/s
    goal_msg.time_allowance = rclcpp::Duration::from_seconds(15.0);
    //goal_msg.disable_collision_checks = false;

    // 3. Set up SendGoalOptions with the proper callback signatures
    rclcpp_action::Client<BackUpAction>::SendGoalOptions options;

    // This callback gets a shared_ptr<GoalHandle>, not a future
    options.goal_response_callback =
      [](std::shared_ptr<BackUpGoalHandle> goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(rclcpp::get_logger("back_up_client"),
                      "BackUp goal was rejected by server");
        } else {
          RCLCPP_INFO(rclcpp::get_logger("back_up_client"),
                      "BackUp goal accepted, waiting for result");
        }
      };
    // The result callback signature stays the same
    options.result_callback =
      [this](const BackUpGoalHandle::WrappedResult & res) {
        if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(get_logger(), "Backup succeeded → transitioning");
          transitionTo(State::GLOBAL_APPROACH);
        } else {
          RCLCPP_ERROR(get_logger(),
                      "Backup failed (code %d), finishing", res.code);
          transitionTo(State::FINISHED);
        }
      };


    // 4. Send the goal
    back_up_client_->async_send_goal(goal_msg, options);
  }


  // --------------------------------------------------------------------------
  // Helper to publish a small arrow at (x,y) with orientation= yaw, in ref_frame_
  // --------------------------------------------------------------------------
  void publishMarker(double x, double y, double yaw)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = ref_frame_;
    m.header.stamp    = now();
    m.ns              = "current_goal";
    m.id              = static_cast<int>(current_goal_idx_);
    m.type            = m.ARROW;
    m.action          = m.ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 0.0;
    m.pose.orientation.z = std::sin(yaw / 2.0);
    m.pose.orientation.w = std::cos(yaw / 2.0);
    m.scale.x = 0.5;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.g = 1.0f;
    m.color.a = 1.0f;
    current_goal_pub_->publish(m);
  }

  // --------------------------------------------------------------------------
  // TSP‐brute‐force helper over a vector of (x,y) points
  // --------------------------------------------------------------------------
  double dist(const std::pair<double,double> & a,
              const std::pair<double,double> & b)
  {
    return std::hypot(a.first - b.first, a.second - b.second);
  }

  std::vector<std::pair<double,double>>findTSP(
    const std::vector<std::pair<double,double>> & pts_in)
  {
    const size_t N = pts_in.size();
    // This will hold the best tour of (x,y) pairs:
    std::vector<std::pair<double,double>> best_path;
    // And here we’ll record, in parallel, the original indices of those points:
    std::vector<size_t> best_idx_seq;
    double best_len = std::numeric_limits<double>::infinity();

    // Build an array of (point, original_index)
    std::vector<std::pair<std::pair<double,double>, size_t>> items;
    items.reserve(N);
    for (size_t i = 0; i < N; ++i) {
      items.emplace_back(pts_in[i], i);
    }

    // Sort lexicographically so that next_permutation will hit all orders:
    std::sort(items.begin(), items.end(),
              [](auto &a, auto &b){ return a.first < b.first; });

    // Brute‐force every permutation
    do {
      // Compute tour length: start → first → … → last → start
      double L = dist(start_pos_, items[0].first);
      for (size_t i = 0; i + 1 < N; ++i) {
        L += dist(items[i].first, items[i+1].first);
      }
      L += dist(items.back().first, start_pos_);

      if (L < best_len) {
        best_len = L;

        // Capture this permutation as the current best
        best_path.clear();
        best_idx_seq.clear();
        for (auto &it : items) {
          best_path.push_back(it.first);
          best_idx_seq.push_back(it.second);
        }
      }
    }
    // Permute based on the point‐coordinates (so items stays in sync)
    while (std::next_permutation(
            items.begin(), items.end(),
            [](auto &a, auto &b){ return a.first < b.first; }));

    // Finally, append the “return home” step
    best_path.emplace_back(start_pos_);
    // Use max() as a sentinel—the last index is “home,” not a ring:
    best_idx_seq.push_back(std::numeric_limits<size_t>::max());

    // Store the index‐sequence for later use:
    rings_index_ = std::move(best_idx_seq);

    return best_path;
  }


  // --------------------------------------------------------------------------
  // Member variables
  // --------------------------------------------------------------------------
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odom_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr       current_goal_pub_;
  rclcpp_action::Client<LocalizeObj>::SharedPtr                localize_client_;
  rclcpp_action::Client<Navigate>::SharedPtr                          nav2_client_;
  rclcpp_action::Client<Spin>::SharedPtr                          nav2_spin_client_;
  rclcpp_action::Client<MoveTCP>::SharedPtr                           local_client_;
  rclcpp_action::Client<UpdateMap>::SharedPtr                         update_map_client_;
  rclcpp::Client<GetState>::SharedPtr                                 bt_client_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::CallbackGroup::SharedPtr joy_cb_group_;   // NEW
  rclcpp_action::Client<MoveJoint>::SharedPtr                         action_client_;
  rclcpp::TimerBase::SharedPtr                                        bt_timer_;
  rclcpp::TimerBase::SharedPtr                                        delay_timer_;
  rclcpp::TimerBase::SharedPtr                                        timer_;
  tf2_ros::Buffer                                                     tf_buffer_;
  tf2_ros::TransformListener                                           tf_listener_;
  State                     state_;
  bool                      got_odom_;
  bool                      start_flag_;
  bool prev_button_state_;
  std::string               ref_frame_;
  std::pair<double,double>  start_pos_, current_pos_;
  std::vector<std::pair<double,double>> ring_positions_, best_path_;
  size_t                    current_goal_idx_;

  std::random_device        rd_;
  std::mt19937              gen_;
  std::uniform_real_distribution<> dist_;  
  double                    start_yaw_;
  double                    current_yaw_;
  double                    yaw_correction_;
  double                    pos_error_;
  double                    pos_error_;
  std::vector<size_t> rings_index_;


  using BackUp = nav2_msgs::action::BackUp;
  rclcpp_action::Client<BackUp>::SharedPtr back_up_client_; 
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RingPathPlanner>();

  rclcpp::executors::MultiThreadedExecutor exec;  // allows true concurrency
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

