#pragma once

#include <cstdint>
#include <deque>
#include <vector>

namespace cat_terrain
{

struct Point
{
  double x;
  double y;
  double z;
};

struct Pose
{
  double x;
  double y;
  double z;
  double yaw;
};

struct Config
{
  double min_x{0.35};
  double max_x{5.5};
  double detection_max_x{3.0};
  double half_width{1.0};
  double resolution{0.1};
  double sensor_x{-0.347981};
  double angular_resolution_deg{2.0};
  double radial_resolution{0.05};
  double max_pair_distance{1.0};
  double step_height{0.03};
  double min_residual_slope_deg{5.0};
  double history_seconds{2.0};
  int history_frames{24};
  int min_confirmations{3};
  double max_height_mad{0.035};
  int min_lateral_support{2};
};

struct Result
{
  int width{0};
  int height{0};
  std::vector<int8_t> hazards;
  std::vector<int8_t> up_steps;
  std::vector<int8_t> drop_offs;
  int observed_cells{0};
  int hazard_cells{0};
};

// Radial profiles perform detection; the grid is only the output representation.
class TerrainGrid
{
public:
  explicit TerrainGrid(const Config & config);
  Result update(const std::vector<Point> & world_points, const Pose & robot, double stamp);
  void clear() {history_.clear();}
  int width() const {return width_;}
  int height() const {return height_;}

private:
  struct Frame
  {
    double stamp;
    std::vector<Point> points;
  };

  Config config_;
  int width_;
  int height_;
  int ray_count_;
  int radial_bins_;
  std::deque<Frame> history_;
};

}  // namespace cat_terrain
