#include "cat_terrain/terrain_grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cat_terrain
{
namespace
{
constexpr double kPi = 3.141592653589793;
constexpr double kHalfSector = kPi / 4.0;

struct Accumulator
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double min_z{INFINITY};
  double max_z{-INFINITY};
  int count{0};

  void add(const Point & point)
  {
    x += point.x;
    y += point.y;
    z += point.z;
    min_z = std::min(min_z, point.z);
    max_z = std::max(max_z, point.z);
    ++count;
  }
};

struct RaySample
{
  double distance;
  double height;
  int frame;
};

struct ProfilePoint
{
  double distance;
  double height;
  double x;
  double y;
  bool virtual_ground{false};
};

struct Candidate
{
  double x;
  double y;
  int kind;
};

double median(std::vector<double> & values)
{
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  if (values.size() % 2 == 1) {
    return *middle;
  }
  const double upper = *middle;
  const double lower = *std::max_element(values.begin(), middle);
  return 0.5 * (lower + upper);
}
}  // namespace

TerrainGrid::TerrainGrid(const Config & config)
: config_(config),
  width_(static_cast<int>(std::ceil(
      (config.detection_max_x - config.min_x) / config.resolution))),
  height_(static_cast<int>(std::ceil(2.0 * config.half_width / config.resolution))),
  ray_count_(static_cast<int>(std::ceil(90.0 / config.angular_resolution_deg)))
{
  if (!std::isfinite(config.min_x) || !std::isfinite(config.max_x) ||
    !std::isfinite(config.detection_max_x) || !std::isfinite(config.half_width) ||
    !std::isfinite(config.resolution) || !std::isfinite(config.sensor_x) ||
    !std::isfinite(config.angular_resolution_deg) ||
    !std::isfinite(config.radial_resolution) ||
    !std::isfinite(config.max_pair_distance) ||
    !std::isfinite(config.step_height) ||
    !std::isfinite(config.min_residual_slope_deg) ||
    !std::isfinite(config.history_seconds) || !std::isfinite(config.max_height_mad) ||
    config.min_x < 0.0 || config.detection_max_x <= config.min_x ||
    config.max_x < config.detection_max_x || config.half_width <= 0.0 ||
    config.resolution <= 0.0 || config.sensor_x >= 0.0 ||
    config.angular_resolution_deg <= 0.0 || config.angular_resolution_deg > 10.0 ||
    config.radial_resolution <= 0.0 || config.max_pair_distance <= 0.0 ||
    config.step_height <= 0.0 || config.min_residual_slope_deg < 0.0 ||
    config.min_residual_slope_deg >= 89.0 || config.history_seconds <= 0.0 ||
    config.history_frames < 1 || config.min_confirmations < 1 ||
    config.min_confirmations > config.history_frames ||
    config.max_height_mad < 0.0 || config.min_lateral_support < 1 ||
    width_ < 1 || height_ < 1 || ray_count_ < 1)
  {
    throw std::invalid_argument("Invalid terrain profile configuration");
  }
}

Result TerrainGrid::update(
  const std::vector<Point> & world_points, const Pose & robot, double stamp)
{
  if (!std::isfinite(stamp) || !std::isfinite(robot.x) || !std::isfinite(robot.y) ||
    !std::isfinite(robot.z) || !std::isfinite(robot.yaw))
  {
    throw std::invalid_argument("Invalid terrain pose or timestamp");
  }

  if (!history_.empty() && stamp < history_.back().stamp) {
    history_.clear();
  }

  const double cosine = std::cos(robot.yaw);
  const double sine = std::sin(robot.yaw);
  const double sensor_world_x = robot.x + cosine * config_.sensor_x;
  const double sensor_world_y = robot.y + sine * config_.sensor_x;
  const double angle_step = 2.0 * kHalfSector / ray_count_;
  const int radial_bins = static_cast<int>(std::ceil(
      (config_.max_x - config_.sensor_x) /
      (std::cos(kHalfSector) * config_.radial_resolution))) + 1;

  // Compress each scan in sensor-centred polar coordinates before storing it.
  if (!world_points.empty() && (history_.empty() || stamp > history_.back().stamp)) {
    std::unordered_map<int, Accumulator> bins;
    bins.reserve(world_points.size() / 4 + 1);
    for (const auto & point : world_points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const double dx = point.x - sensor_world_x;
      const double dy = point.y - sensor_world_y;
      const double bx = cosine * dx + sine * dy;
      const double by = -sine * dx + cosine * dy;
      const double angle = std::atan2(by, bx);
      const double distance = std::hypot(bx, by);
      if (angle < -kHalfSector || angle >= kHalfSector ||
        distance < -config_.sensor_x || bx + config_.sensor_x > config_.max_x)
      {
        continue;
      }
      const int ray = static_cast<int>((angle + kHalfSector) / angle_step);
      const int radial = static_cast<int>(distance / config_.radial_resolution);
      if (ray >= 0 && ray < ray_count_ && radial >= 0 && radial < radial_bins) {
        bins[ray * radial_bins + radial].add(point);
      }
    }
    Frame frame{stamp, {}};
    frame.points.reserve(bins.size());
    for (const auto & [key, bin] : bins) {
      (void)key;
      if (bin.max_z - bin.min_z > 2.0 * config_.max_height_mad) {
        continue;
      }
      frame.points.push_back({bin.x / bin.count, bin.y / bin.count, bin.z / bin.count});
    }
    history_.push_back(std::move(frame));
  }

  while (!history_.empty() &&
    (static_cast<int>(history_.size()) > config_.history_frames ||
    stamp - history_.front().stamp > config_.history_seconds))
  {
    history_.pop_front();
  }

  Result result;
  result.width = width_;
  result.height = height_;
  const auto cell_count = static_cast<std::size_t>(width_ * height_);
  result.hazards.assign(cell_count, -1);
  result.up_steps.assign(cell_count, -1);
  result.drop_offs.assign(cell_count, -1);

  std::vector<std::vector<RaySample>> rays(static_cast<std::size_t>(ray_count_));
  for (std::size_t frame = 0; frame < history_.size(); ++frame) {
    for (const auto & point : history_[frame].points) {
      const double dx = point.x - sensor_world_x;
      const double dy = point.y - sensor_world_y;
      const double bx = cosine * dx + sine * dy;
      const double by = -sine * dx + cosine * dy;
      const double angle = std::atan2(by, bx);
      if (angle < -kHalfSector || angle >= kHalfSector) {
        continue;
      }
      const double distance = std::hypot(bx, by);
      const double x = config_.sensor_x + bx;
      if (distance < -config_.sensor_x || x > config_.max_x) {
        continue;
      }
      const int ray = static_cast<int>((angle + kHalfSector) / angle_step);
      rays[static_cast<std::size_t>(ray)].push_back({distance, point.z, static_cast<int>(frame)});
    }
  }

  const auto cellIndex = [this](double x, double y) -> int {
      if (x < config_.min_x || x >= config_.detection_max_x ||
        y < -config_.half_width || y >= config_.half_width)
      {
        return -1;
      }
      const int ix = static_cast<int>((x - config_.min_x) / config_.resolution);
      const int iy = static_cast<int>((y + config_.half_width) / config_.resolution);
      return iy * width_ + ix;
    };

  std::vector<std::vector<Candidate>> candidates(static_cast<std::size_t>(ray_count_));
  const double min_break_grade = std::tan(config_.min_residual_slope_deg * kPi / 180.0);
  for (int ray = 0; ray < ray_count_; ++ray) {
    auto & samples = rays[static_cast<std::size_t>(ray)];
    std::sort(samples.begin(), samples.end(), [](const auto & a, const auto & b) {
      return a.distance < b.distance;
    });
    const double angle = -kHalfSector + (ray + 0.5) * angle_step;
    std::vector<ProfilePoint> profile;
    // The already traversed region from the LiDAR to base_link is the local
    // zero-height reference. It is never reported as a measured free cell.
    profile.push_back({-config_.sensor_x, robot.z, 0.0, 0.0, true});
    for (std::size_t begin = 0; begin < samples.size();) {
      std::size_t end = begin + 1;
      while (end < samples.size() &&
        samples[end].distance - samples[begin].distance <= config_.radial_resolution)
      {
        ++end;
      }
      std::vector<double> distances;
      std::vector<double> heights;
      std::vector<uint8_t> seen(history_.size(), 0);
      int confirmations = 0;
      distances.reserve(end - begin);
      heights.reserve(end - begin);
      for (std::size_t i = begin; i < end; ++i) {
        distances.push_back(samples[i].distance);
        heights.push_back(samples[i].height);
        if (!seen[static_cast<std::size_t>(samples[i].frame)]) {
          seen[static_cast<std::size_t>(samples[i].frame)] = 1;
          ++confirmations;
        }
      }
      if (confirmations >= config_.min_confirmations) {
        const double height = median(heights);
        for (auto & value : heights) {
          value = std::abs(value - height);
        }
        if (median(heights) <= config_.max_height_mad) {
          const double distance = median(distances);
          const double x = config_.sensor_x + distance * std::cos(angle);
          const double y = distance * std::sin(angle);
          profile.push_back({distance, height, x, y, false});
          const int cell = cellIndex(x, y);
          if (cell >= 0 && result.hazards[static_cast<std::size_t>(cell)] < 0) {
            result.hazards[static_cast<std::size_t>(cell)] = 0;
            result.up_steps[static_cast<std::size_t>(cell)] = 0;
            result.drop_offs[static_cast<std::size_t>(cell)] = 0;
            ++result.observed_cells;
          }
        }
      }
      begin = end;
    }

    for (std::size_t i = 1; i < profile.size(); ++i) {
      const auto & near = profile[i - 1];
      const auto & far = profile[i];
      const double gap = far.distance - near.distance;
      if (gap < config_.radial_resolution || gap > config_.max_pair_distance) {
        continue;
      }

      // A ramp has a similar grade on both sides; a step has one abrupt
      // height change. Use neighbouring pairs to estimate the local grade.
      double grade_sum = 0.0;
      int grade_count = 0;
      if (i >= 2) {
        const auto & previous = profile[i - 2];
        const double previous_gap = near.distance - previous.distance;
        if (previous_gap >= config_.radial_resolution &&
          previous_gap <= config_.max_pair_distance)
        {
          grade_sum += (near.height - previous.height) / previous_gap;
          ++grade_count;
        }
      }
      if (i + 1 < profile.size()) {
        const auto & next = profile[i + 1];
        const double next_gap = next.distance - far.distance;
        if (next_gap >= config_.radial_resolution &&
          next_gap <= config_.max_pair_distance)
        {
          grade_sum += (next.height - far.height) / next_gap;
          ++grade_count;
        }
      }
      if (grade_count == 0) {
        continue;
      }
      const double change = far.height - near.height;
      const double residual = change - gap * grade_sum / grade_count;
      if (std::abs(residual) + 1.0e-5 < config_.step_height ||
        std::abs(residual) / gap <= min_break_grade ||
        std::abs(change) / gap <= min_break_grade || residual * change <= 0.0)
      {
        continue;
      }
      const auto & edge = near.virtual_ground ? far : near;
      if (cellIndex(edge.x, edge.y) >= 0) {
        candidates[static_cast<std::size_t>(ray)].push_back({
          edge.x, edge.y, residual > 0.0 ? 1 : -1});
      }
    }
  }

  // Require a coherent edge across neighbouring rays, not just one noisy return.
  for (int ray = 0; ray < ray_count_; ++ray) {
    for (const auto & candidate : candidates[static_cast<std::size_t>(ray)]) {
      int support = 1;
      for (int neighbor = std::max(0, ray - 2);
        neighbor <= std::min(ray_count_ - 1, ray + 2); ++neighbor)
      {
        if (neighbor == ray) {
          continue;
        }
        for (const auto & other : candidates[static_cast<std::size_t>(neighbor)]) {
          if (candidate.kind == other.kind &&
            std::abs(candidate.x - other.x) <= 0.25 &&
            std::abs(candidate.y - other.y) <= 0.25)
          {
            ++support;
            break;
          }
        }
      }
      if (support < config_.min_lateral_support) {
        continue;
      }
      const auto cell = static_cast<std::size_t>(cellIndex(candidate.x, candidate.y));
      if (result.hazards[cell] != 100) {
        ++result.hazard_cells;
      }
      result.hazards[cell] = 100;
      (candidate.kind > 0 ? result.up_steps : result.drop_offs)[cell] = 100;
    }
  }
  return result;
}

}  // namespace cat_terrain
