#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace laser_uav_hw_api
{

class FastLioPx4Bridge : public rclcpp::Node
{
public:
  FastLioPx4Bridge()
  : Node("fast_lio_px4_bridge")
  {
    const auto uav_name = declare_parameter<std::string>("uav_name", "uav1");
    world_frame_ = uav_name + "/world";
    imu_frame_ = uav_name + "/livox_up/imu";
    fcu_frame_ = uav_name + "/fcu";
    publisher_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
      "fmu/in/vehicle_visual_odometry", 10);
    correction_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "fast_lio/odometry", rclcpp::SensorDataQoS().keep_last(1),
      [this](const nav_msgs::msg::Odometry & msg) {
        const auto stamp = stamp_ns(msg);
        if (valid_sample(msg) && stamp > last_correction_ns_) {
          last_correction_ns_ = stamp;
        }
      });
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "fast_lio/odometry_high_freq", rclcpp::SensorDataQoS().keep_last(1),
      [this](const nav_msgs::msg::Odometry & msg) {on_odometry(msg);});
    RCLCPP_INFO(
      get_logger(), "FAST-LIO bridge ready; requires FCU-transformed pose and body-FLU twist");
  }

private:
  static int64_t stamp_ns(const nav_msgs::msg::Odometry & msg)
  {
    return static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL +
           msg.header.stamp.nanosec;
  }

  bool fresh(int64_t stamp) const
  {
    const auto age = now().nanoseconds() - stamp;
    return stamp > 0 && age >= -50000000LL && age <= 500000000LL;
  }

  static double orientation_norm(const nav_msgs::msg::Odometry & msg)
  {
    const auto & q = msg.pose.pose.orientation;
    return std::hypot(q.w, std::hypot(q.x, q.y, q.z));
  }

  bool valid_sample(const nav_msgs::msg::Odometry & msg) const
  {
    const auto & p = msg.pose.pose.position;
    const double norm = orientation_norm(msg);
    // shortcut: assumes FCU-transformed livox_simulation inputs; recheck frames for other setups.
    return msg.header.frame_id == world_frame_ &&
           (msg.child_frame_id == imu_frame_ || msg.child_frame_id == fcu_frame_) &&
           msg.header.stamp.sec >= 0 && msg.header.stamp.nanosec < 1000000000U &&
           fresh(stamp_ns(msg)) && std::isfinite(static_cast<float>(p.x)) &&
           std::isfinite(static_cast<float>(p.y)) && std::isfinite(static_cast<float>(p.z)) &&
           std::isfinite(norm) && norm > 1e-6;
  }

  static float variance(double value)
  {
    const auto result = static_cast<float>(value);
    return std::isfinite(result) && result > 0.0f ? result :
           std::numeric_limits<float>::quiet_NaN();
  }

  void on_odometry(const nav_msgs::msg::Odometry & msg)
  {
    const auto stamp = stamp_ns(msg);
    if (!valid_sample(msg) || stamp <= last_sample_ns_ || !fresh(last_correction_ns_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Odometry withheld: check frames, timestamps, pose and fresh LiDAR corrections");
      return;
    }

    px4_msgs::msg::VehicleOdometry out;
    out.timestamp = now().nanoseconds() / 1000;
    out.timestamp_sample = stamp / 1000;
    // FAST-LIO's world heading is arbitrary; it must not be declared earth-fixed NED.
    out.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
    out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    const double norm = orientation_norm(msg);
    out.position = {float(p.x), float(-p.y), float(-p.z)};
    out.q = {float(q.w / norm), float(q.x / norm), float(-q.y / norm), float(-q.z / norm)};
    const auto & v = msg.twist.twist.linear;
    const auto & w = msg.twist.twist.angular;
    out.velocity = {float(v.x), float(-v.y), float(-v.z)};
    out.angular_velocity = {float(w.x), float(-w.y), float(-w.z)};
    for (size_t i = 0; i < 3; ++i) {
      out.position_variance[i] = variance(msg.pose.covariance[i * 7]);
      out.orientation_variance[i] = variance(msg.pose.covariance[(i + 3) * 7]);
      out.velocity_variance[i] = variance(msg.twist.covariance[i * 7]);
      if (!std::isfinite(out.velocity[i])) {
        out.velocity.fill(std::numeric_limits<float>::quiet_NaN());
      }
      if (!std::isfinite(out.angular_velocity[i])) {
        out.angular_velocity.fill(std::numeric_limits<float>::quiet_NaN());
      }
    }
    publisher_->publish(out);
    last_sample_ns_ = stamp;
  }

  std::string world_frame_, imu_frame_, fcu_frame_;
  int64_t last_correction_ns_{0};
  int64_t last_sample_ns_{0};
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr correction_subscription_;
};

}  // namespace laser_uav_hw_api

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_uav_hw_api::FastLioPx4Bridge>());
  rclcpp::shutdown();
  return 0;
}
