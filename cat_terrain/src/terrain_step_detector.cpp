#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "cat_terrain/terrain_grid.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/grid_cells.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace cat_terrain
{

class TerrainStepDetector : public rclcpp::Node
{
public:
  TerrainStepDetector()
  : Node("terrain_step_detector")
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/velodyne_points");
    hazards_topic_ = declare_parameter<std::string>("hazards_topic", "/terrain/hazards");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");

    Config config;
    config.min_x = declare_parameter("min_x", config.min_x);
    config.max_x = declare_parameter("max_x", config.max_x);
    config.detection_max_x = declare_parameter("detection_max_x", config.detection_max_x);
    config.half_width = declare_parameter("half_width", config.half_width);
    config.resolution = declare_parameter("resolution", config.resolution);
    config.sensor_x = declare_parameter("sensor_x", config.sensor_x);
    config.angular_resolution_deg = declare_parameter(
      "angular_resolution_deg", config.angular_resolution_deg);
    config.radial_resolution = declare_parameter("radial_resolution", config.radial_resolution);
    config.max_pair_distance = declare_parameter("max_pair_distance", config.max_pair_distance);
    config.step_height = declare_parameter("step_height", config.step_height);
    config.min_residual_slope_deg = declare_parameter(
      "min_residual_slope_deg", config.min_residual_slope_deg);
    config.history_seconds = declare_parameter("history_seconds", config.history_seconds);
    config.history_frames = declare_parameter("history_frames", config.history_frames);
    config.min_confirmations = declare_parameter("min_confirmations", config.min_confirmations);
    config.max_height_mad = declare_parameter("max_height_mad", config.max_height_mad);
    config.min_lateral_support =
      declare_parameter("min_lateral_support", config.min_lateral_support);
    ground_min_z_ = declare_parameter("ground_min_z", -0.5);
    ground_max_z_ = declare_parameter("ground_max_z", 0.35);
    const double tf_timeout_seconds = declare_parameter("tf_timeout_seconds", 0.05);
    input_timeout_seconds_ = declare_parameter("input_timeout_seconds", 0.6);
    if (cloud_topic_.empty() || hazards_topic_.empty() || base_frame_.empty() ||
      fixed_frame_.empty() || hazards_topic_ == "/local_costmap/costmap_raw" ||
      !std::isfinite(ground_min_z_) || !std::isfinite(ground_max_z_) ||
      ground_max_z_ <= ground_min_z_ || !std::isfinite(tf_timeout_seconds) ||
      tf_timeout_seconds <= 0.0 || !std::isfinite(input_timeout_seconds_) ||
      input_timeout_seconds_ <= 0.0)
    {
      throw std::invalid_argument("Invalid terrain detector parameters");
    }
    tf_timeout_ = rclcpp::Duration::from_seconds(tf_timeout_seconds);
    config_ = config;
    grid_ = std::make_unique<TerrainGrid>(config_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    hazards_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(hazards_topic_, 1);
    up_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/terrain/up_steps", 1);
    drop_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/terrain/drop_offs", 1);
    valid_publisher_ = create_publisher<std_msgs::msg::Bool>("/terrain/valid", 1);
    coverage_publisher_ =
      create_publisher<std_msgs::msg::Float32MultiArray>("/terrain/coverage_by_range", 1);
    up_cells_publisher_ = create_publisher<nav_msgs::msg::GridCells>("/terrain/up_cells", 1);
    drop_cells_publisher_ = create_publisher<nav_msgs::msg::GridCells>("/terrain/drop_cells", 1);
    if (declare_parameter("publish_history_cloud", false)) {
      history_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/terrain/retained_points", 1);
    }
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&TerrainStepDetector::onCloud, this, std::placeholders::_1));
    stale_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&TerrainStepDetector::onWatchdog, this));
    RCLCPP_INFO(
      get_logger(), "Terrain detection x=[%.2f, %.2f] m, recording to %.2f m, step=%.3f m; %s -> %s",
      config_.min_x, config_.detection_max_x, config_.max_x, config_.step_height,
      cloud_topic_.c_str(), hazards_topic_.c_str());
  }

private:
  static bool xyzFloatFields(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    int found = 0;
    for (const auto & field : cloud.fields) {
      if (field.name == "x" || field.name == "y" || field.name == "z") {
        if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
          return false;
        }
        ++found;
      }
    }
    return found == 3;
  }

  nav_msgs::msg::OccupancyGrid makeGrid(
    const std::vector<int8_t> & costs, const rclcpp::Time & stamp) const
  {
    nav_msgs::msg::OccupancyGrid message;
    message.header.stamp = stamp;
    message.header.frame_id = base_frame_;
    message.info.map_load_time = stamp;
    message.info.resolution = static_cast<float>(config_.resolution);
    message.info.width = static_cast<uint32_t>(grid_->width());
    message.info.height = static_cast<uint32_t>(grid_->height());
    message.info.origin.position.x = config_.min_x;
    message.info.origin.position.y = -config_.half_width;
    message.info.origin.orientation.w = 1.0;
    message.data = costs;
    return message;
  }

  nav_msgs::msg::GridCells makeCells(
    const std::vector<int8_t> & costs, const rclcpp::Time & stamp) const
  {
    nav_msgs::msg::GridCells message;
    message.header.stamp = stamp;
    message.header.frame_id = base_frame_;
    message.cell_width = static_cast<float>(config_.resolution);
    message.cell_height = static_cast<float>(config_.resolution);
    for (int iy = 0; iy < grid_->height(); ++iy) {
      for (int ix = 0; ix < grid_->width(); ++ix) {
        if (costs[static_cast<std::size_t>(iy * grid_->width() + ix)] != 100) {
          continue;
        }
        geometry_msgs::msg::Point point;
        point.x = config_.min_x + (ix + 0.5) * config_.resolution;
        point.y = -config_.half_width + (iy + 0.5) * config_.resolution;
        point.z = 0.02;
        message.cells.push_back(point);
      }
    }
    return message;
  }

  void publishCoverage(const std::vector<int8_t> & costs)
  {
    const int bands = static_cast<int>(std::ceil(config_.detection_max_x - config_.min_x));
    std::vector<int> observed(static_cast<std::size_t>(bands), 0);
    std::vector<int> total(static_cast<std::size_t>(bands), 0);
    for (int ix = 0; ix < grid_->width(); ++ix) {
      const double distance_from_start = (ix + 0.5) * config_.resolution;
      const int band = std::min(bands - 1, static_cast<int>(distance_from_start));
      for (int iy = 0; iy < grid_->height(); ++iy) {
        const auto index = static_cast<std::size_t>(iy * grid_->width() + ix);
        ++total[static_cast<std::size_t>(band)];
        observed[static_cast<std::size_t>(band)] += costs[index] >= 0;
      }
    }
    std_msgs::msg::Float32MultiArray message;
    message.data.reserve(static_cast<std::size_t>(bands));
    for (int band = 0; band < bands; ++band) {
      message.data.push_back(total[static_cast<std::size_t>(band)] > 0 ?
        static_cast<float>(observed[static_cast<std::size_t>(band)]) /
        total[static_cast<std::size_t>(band)] : 0.0F);
    }
    coverage_publisher_->publish(message);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    const auto start = std::chrono::steady_clock::now();
    if (!xyzFloatFields(*cloud)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "PointCloud2 requires FLOAT32 x/y/z fields");
      return;
    }

    geometry_msgs::msg::TransformStamped base_from_cloud_msg;
    geometry_msgs::msg::TransformStamped world_from_base_msg;
    try {
      base_from_cloud_msg = tf_buffer_->lookupTransform(
        base_frame_, cloud->header.frame_id, cloud->header.stamp, tf_timeout_);
      world_from_base_msg = tf_buffer_->lookupTransform(
        fixed_frame_, base_frame_, cloud->header.stamp, tf_timeout_);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Terrain TF unavailable: %s", error.what());
      return;
    }

    tf2::Transform base_from_cloud;
    tf2::Transform world_from_base;
    tf2::fromMsg(base_from_cloud_msg.transform, base_from_cloud);
    tf2::fromMsg(world_from_base_msg.transform, world_from_base);
    const Pose pose{
      world_from_base.getOrigin().x(), world_from_base.getOrigin().y(),
      world_from_base.getOrigin().z(), tf2::getYaw(world_from_base.getRotation())};

    std::vector<Point> world_points;
    world_points.reserve(static_cast<std::size_t>(cloud->width) * cloud->height / 8);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        const auto body = base_from_cloud * tf2::Vector3(*x, *y, *z);
        if (body.x() < 0.0 || body.x() >= config_.max_x ||
          std::abs(std::atan2(body.y(), body.x() - config_.sensor_x)) >=
          3.141592653589793 / 4.0 ||
          body.z() < ground_min_z_ || body.z() > ground_max_z_)
        {
          continue;
        }
        const auto world = world_from_base * body;
        world_points.push_back(Point{world.x(), world.y(), world.z()});
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Invalid terrain point cloud: %s", error.what());
      return;
    }

    const auto result = grid_->update(
      world_points, pose, rclcpp::Time(cloud->header.stamp).seconds());
    publishHistory(cloud->header.stamp);
    publishResult(
      result.hazards, result.up_steps, result.drop_offs, cloud->header.stamp,
      !world_points.empty() && result.observed_cells > 0);
    last_good_cloud_time_ = std::chrono::steady_clock::now();
    have_good_cloud_ = true;
    stale_reported_ = false;
    const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    if (elapsed > 50.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Terrain processing %.1f ms for %zu ROI points, %d observed cells",
        elapsed, world_points.size(), result.observed_cells);
    }
  }

  void onWatchdog()
  {
    const auto age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_good_cloud_time_).count();
    if (have_good_cloud_ && age <= input_timeout_seconds_) {
      return;
    }
    if (!stale_reported_) {
      grid_->clear();
      stale_reported_ = true;
    }
    const auto count = static_cast<std::size_t>(grid_->width() * grid_->height());
    const std::vector<int8_t> unknown(count, -1);
    publishResult(unknown, unknown, unknown, now(), false);
    publishHistory(now());
  }

  void publishHistory(const rclcpp::Time & stamp)
  {
    if (!history_publisher_ || history_publisher_->get_subscription_count() == 0) {
      return;
    }
    const auto points = grid_->retainedPoints();
    sensor_msgs::msg::PointCloud2 message;
    message.header.stamp = stamp;
    message.header.frame_id = fixed_frame_;
    sensor_msgs::PointCloud2Modifier modifier(message);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    message.is_dense = true;
    if (!points.empty()) {
      sensor_msgs::PointCloud2Iterator<float> x(message, "x");
      sensor_msgs::PointCloud2Iterator<float> y(message, "y");
      sensor_msgs::PointCloud2Iterator<float> z(message, "z");
      for (const auto & point : points) {
        *x = static_cast<float>(point.x);
        *y = static_cast<float>(point.y);
        *z = static_cast<float>(point.z);
        ++x;
        ++y;
        ++z;
      }
    }
    history_publisher_->publish(std::move(message));
  }

  void publishResult(
    const std::vector<int8_t> & hazards,
    const std::vector<int8_t> & up_steps,
    const std::vector<int8_t> & drop_offs,
    const rclcpp::Time & stamp, bool is_valid)
  {
    hazards_publisher_->publish(makeGrid(hazards, stamp));
    up_publisher_->publish(makeGrid(up_steps, stamp));
    drop_publisher_->publish(makeGrid(drop_offs, stamp));
    up_cells_publisher_->publish(makeCells(up_steps, stamp));
    drop_cells_publisher_->publish(makeCells(drop_offs, stamp));
    publishCoverage(hazards);
    std_msgs::msg::Bool valid;
    valid.data = is_valid;
    valid_publisher_->publish(valid);
  }

  std::string cloud_topic_;
  std::string hazards_topic_;
  std::string base_frame_;
  std::string fixed_frame_;
  double ground_min_z_;
  double ground_max_z_;
  double input_timeout_seconds_;
  rclcpp::Duration tf_timeout_{0, 0};
  Config config_;
  std::unique_ptr<TerrainGrid> grid_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr hazards_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr up_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr drop_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr coverage_publisher_;
  rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr up_cells_publisher_;
  rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr drop_cells_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr history_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::TimerBase::SharedPtr stale_timer_;
  std::chrono::steady_clock::time_point last_good_cloud_time_{};
  bool have_good_cloud_{false};
  bool stale_reported_{false};
};

}  // namespace cat_terrain

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cat_terrain::TerrainStepDetector>());
  rclcpp::shutdown();
  return 0;
}
