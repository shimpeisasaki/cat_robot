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
struct Key
{
  int x;
  int y;
  bool operator==(const Key & other) const {return x == other.x && y == other.y;}
};

struct KeyHash
{
  std::size_t operator()(const Key & key) const
  {
    const auto x = static_cast<uint32_t>(key.x);
    const auto y = static_cast<uint32_t>(key.y);
    return static_cast<std::size_t>((static_cast<uint64_t>(x) << 32) | y);
  }
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
  width_(static_cast<int>(std::ceil((config.max_x - config.min_x) / config.resolution))),
  height_(static_cast<int>(std::ceil(2.0 * config.half_width / config.resolution)))
{
  if (!std::isfinite(config.min_x) || !std::isfinite(config.max_x) ||
    !std::isfinite(config.half_width) || !std::isfinite(config.resolution) ||
    !std::isfinite(config.step_height) || !std::isfinite(config.max_slope_deg) ||
    !std::isfinite(config.history_seconds) || !std::isfinite(config.max_height_mad) ||
    config.min_x < 0.0 || config.max_x <= config.min_x || config.half_width <= 0.0 ||
    config.resolution <= 0.0 || config.step_height <= 0.0 ||
    config.max_slope_deg < 0.0 || config.max_slope_deg >= 89.0 ||
    config.history_seconds <= 0.0 || config.history_frames < 1 ||
    config.min_confirmations < 1 || config.min_confirmations > config.history_frames ||
    config.max_height_mad < 0.0 || config.max_gap_cells < 1 ||
    config.min_lateral_support < 1 || width_ < 2 || height_ < 1)
  {
    throw std::invalid_argument("Invalid terrain grid configuration");
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
    history_.clear();  // A time jump must not reuse observations from the old timeline.
  }

  if (!world_points.empty() && (history_.empty() || stamp > history_.back().stamp)) {
    std::unordered_map<Key, std::vector<double>, KeyHash> bins;
    bins.reserve(world_points.size() / 4 + 1);
    for (const auto & point : world_points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const Key key{
        static_cast<int>(std::floor(point.x / config_.resolution)),
        static_cast<int>(std::floor(point.y / config_.resolution))};
      bins[key].push_back(point.z);
    }
    Frame frame{stamp, {}};
    frame.cells.reserve(bins.size());
    for (auto & [key, heights] : bins) {
      frame.cells.push_back(CellSample{
        (static_cast<double>(key.x) + 0.5) * config_.resolution,
        (static_cast<double>(key.y) + 0.5) * config_.resolution,
        median(heights)});
    }
    history_.push_back(std::move(frame));
  }

  while (!history_.empty() &&
    (static_cast<int>(history_.size()) > config_.history_frames ||
    stamp - history_.front().stamp > config_.history_seconds))
  {
    history_.pop_front();
  }

  const auto count = static_cast<std::size_t>(width_ * height_);
  Result result;
  result.width = width_;
  result.height = height_;
  result.hazards.assign(count, -1);
  result.up_steps.assign(count, -1);
  result.drop_offs.assign(count, -1);
  std::vector<std::vector<double>> heights_by_cell(count);
  const double cosine = std::cos(robot.yaw);
  const double sine = std::sin(robot.yaw);

  for (const auto & frame : history_) {
    // Count each scan at most once per cell, even if several world cells map to it.
    std::vector<std::vector<double>> frame_heights(count);
    for (const auto & sample : frame.cells) {
      const double dx = sample.x - robot.x;
      const double dy = sample.y - robot.y;
      const double bx = cosine * dx + sine * dy;
      const double by = -sine * dx + cosine * dy;
      if (bx < config_.min_x || bx >= config_.max_x ||
        by < -config_.half_width || by >= config_.half_width)
      {
        continue;
      }
      const int ix = static_cast<int>((bx - config_.min_x) / config_.resolution);
      const int iy = static_cast<int>((by + config_.half_width) / config_.resolution);
      if (ix >= 0 && ix < width_ && iy >= 0 && iy < height_) {
        frame_heights[static_cast<std::size_t>(iy * width_ + ix)].push_back(sample.z);
      }
    }
    for (std::size_t i = 0; i < count; ++i) {
      if (!frame_heights[i].empty()) {
        heights_by_cell[i].push_back(median(frame_heights[i]));
      }
    }
  }

  std::vector<double> ground(count, 0.0);
  std::vector<uint8_t> observed(count, 0);
  for (std::size_t i = 0; i < count; ++i) {
    auto & heights = heights_by_cell[i];
    if (static_cast<int>(heights.size()) < config_.min_confirmations) {
      continue;
    }
    const double center = median(heights);
    for (auto & height : heights) {
      height = std::abs(height - center);
    }
    if (median(heights) > config_.max_height_mad) {
      continue;
    }
    ground[i] = center;
    observed[i] = 1;
    result.hazards[i] = 0;
    result.up_steps[i] = 0;
    result.drop_offs[i] = 0;
    ++result.observed_cells;
  }

  std::vector<int8_t> candidates(count, 0);
  const double max_slope = std::tan(config_.max_slope_deg * 3.141592653589793 / 180.0);
  for (int iy = 0; iy < height_; ++iy) {
    for (int ix = 0; ix < width_ - 1; ++ix) {
      const auto near = static_cast<std::size_t>(iy * width_ + ix);
      if (!observed[near]) {
        continue;
      }
      for (int gap = 1; gap <= config_.max_gap_cells && ix + gap < width_; ++gap) {
        const auto far = static_cast<std::size_t>(iy * width_ + ix + gap);
        if (!observed[far]) {
          continue;
        }
        const double change = ground[far] - ground[near];
        const double distance = gap * config_.resolution;
        if (std::abs(change) + 1.0e-5 >= config_.step_height &&
          std::abs(change) / distance > max_slope)
        {
          candidates[near] = change > 0.0 ? 1 : -1;
        }
        break;  // The nearest visible patch is the only comparison for this cell.
      }
    }
  }

  for (int iy = 0; iy < height_; ++iy) {
    for (int ix = 0; ix < width_; ++ix) {
      const auto index = static_cast<std::size_t>(iy * width_ + ix);
      const auto kind = candidates[index];
      if (kind == 0) {
        continue;
      }
      int support = 1;
      for (int other_y = std::max(0, iy - 2);
        other_y <= std::min(height_ - 1, iy + 2); ++other_y)
      {
        if (other_y == iy) {
          continue;
        }
        for (int other_x = std::max(0, ix - 1);
          other_x <= std::min(width_ - 1, ix + 1); ++other_x)
        {
          if (candidates[static_cast<std::size_t>(other_y * width_ + other_x)] == kind) {
            ++support;
            break;
          }
        }
      }
      if (support < config_.min_lateral_support) {
        continue;
      }
      result.hazards[index] = 100;
      (kind > 0 ? result.up_steps : result.drop_offs)[index] = 100;
      ++result.hazard_cells;
    }
  }
  return result;
}

}  // namespace cat_terrain
