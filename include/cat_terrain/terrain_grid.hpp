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
  double min_x{0.6};
  double max_x{5.0};
  double half_width{1.0};
  double resolution{0.1};
  double step_height{0.06};
  double max_slope_deg{15.0};
  double history_seconds{1.2};
  int history_frames{6};
  int min_confirmations{3};
  double max_height_mad{0.035};
  int max_gap_cells{2};
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

class TerrainGrid
{
public:
  explicit TerrainGrid(const Config & config);
  Result update(const std::vector<Point> & world_points, const Pose & robot, double stamp);
  void clear() {history_.clear();}
  int width() const {return width_;}
  int height() const {return height_;}

private:
  struct CellSample
  {
    double x;
    double y;
    double z;
  };
  struct Frame
  {
    double stamp;
    std::vector<CellSample> cells;
  };

  Config config_;
  int width_;
  int height_;
  std::deque<Frame> history_;
};

}  // namespace cat_terrain
