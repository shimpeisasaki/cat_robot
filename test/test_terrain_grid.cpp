#include <cmath>
#include <cstdint>
#include <vector>

#include "cat_terrain/terrain_grid.hpp"
#include "gtest/gtest.h"

namespace
{
using cat_terrain::Point;
using cat_terrain::Pose;
using cat_terrain::Result;
using cat_terrain::TerrainGrid;

std::vector<Point> surface(double height_change, bool missing_far = false, bool slope = false)
{
  std::vector<Point> points;
  for (int x = 0; x < 16; ++x) {
    const double px = 0.65 + 0.1 * x;
    if (missing_far && px >= 1.2) {
      continue;
    }
    for (int y = 0; y < 8; ++y) {
      const double py = -0.35 + 0.1 * y;
      const double z = slope ? 0.1 * px : (px >= 1.2 ? height_change : 0.0);
      points.push_back({px, py, z});
    }
  }
  return points;
}

Result confirmed(TerrainGrid & grid, const std::vector<Point> & points, const Pose & pose = {})
{
  grid.update(points, pose, 0.0);
  grid.update(points, pose, 0.2);
  return grid.update(points, pose, 0.4);
}

std::vector<Point> approximateVlp16FlatGround()
{
  // VLP16_B: x=-0.347981 m, z=0.515158 m, roll=160 deg, yaw=-90 deg.
  // Intersect each ideal beam with z=0; this omits reflectivity and range noise.
  constexpr double pi = 3.141592653589793;
  constexpr double roll = 160.0 * pi / 180.0;
  std::vector<Point> points;
  for (int elevation_deg = -15; elevation_deg <= 15; elevation_deg += 2) {
    const double elevation = elevation_deg * pi / 180.0;
    for (int azimuth_step = 0; azimuth_step < 720; ++azimuth_step) {
      const double azimuth = azimuth_step * 0.5 * pi / 180.0;
      const double sy = std::cos(elevation) * std::sin(azimuth);
      const double sz = std::sin(elevation);
      const double dx = std::cos(roll) * sy - std::sin(roll) * sz;
      const double dy = -std::cos(elevation) * std::cos(azimuth);
      const double dz = std::sin(roll) * sy + std::cos(roll) * sz;
      if (dz >= 0.0) {
        continue;
      }
      const double range = 0.515158 / -dz;
      points.push_back({-0.347981 + range * dx, range * dy, 0.0});
    }
  }
  return points;
}

int marked(const std::vector<int8_t> & cells)
{
  int count = 0;
  for (const auto value : cells) {
    count += value == 100;
  }
  return count;
}
}  // namespace

TEST(TerrainGrid, DetectsSupportedUpAndDownSteps)
{
  TerrainGrid up({});
  const auto up_result = confirmed(up, surface(0.1));
  EXPECT_GT(marked(up_result.up_steps), 0);
  EXPECT_EQ(marked(up_result.drop_offs), 0);
  EXPECT_EQ(marked(up_result.hazards), marked(up_result.up_steps));

  TerrainGrid down({});
  const auto down_result = confirmed(down, surface(-0.1));
  EXPECT_GT(marked(down_result.drop_offs), 0);
  EXPECT_EQ(marked(down_result.up_steps), 0);
}

TEST(TerrainGrid, DetectsThreeCentimeterEdgeButRejectsSmallerOrIsolatedPatch)
{
  TerrainGrid edge({});
  const auto three_centimeter = confirmed(edge, surface(-0.03));
  EXPECT_GT(marked(three_centimeter.drop_offs), 0);

  TerrainGrid shallow({});
  const auto two_centimeter = confirmed(shallow, surface(-0.02));
  EXPECT_EQ(marked(two_centimeter.hazards), 0);

  auto isolated = surface(0.0);
  for (auto & point : isolated) {
    if (point.x >= 1.2 && point.y > -0.05 && point.y < 0.05) {
      point.z = -0.1;
    }
  }
  TerrainGrid noise({});
  const auto result = confirmed(noise, isolated);
  EXPECT_EQ(marked(result.hazards), 0);
}

TEST(TerrainGrid, RequiresRepeatedLateralEvidence)
{
  TerrainGrid grid({});
  const auto first = grid.update(surface(0.1), {}, 0.0);
  EXPECT_EQ(marked(first.hazards), 0);
  EXPECT_EQ(first.observed_cells, 0);
  const auto third = confirmed(grid, surface(0.1));
  EXPECT_GT(marked(third.hazards), 0);
}

TEST(TerrainGrid, LeavesFlatSlopeAndMissingGroundUnmarked)
{
  for (const auto & points : {surface(0.0), surface(0.0, false, true),
    surface(0.0, true)})
  {
    TerrainGrid grid({});
    const auto result = confirmed(grid, points);
    EXPECT_EQ(marked(result.hazards), 0);
    EXPECT_GT(result.observed_cells, 0);
    bool has_unknown = false;
    for (const auto value : result.hazards) {
      has_unknown |= value == -1;
    }
    EXPECT_TRUE(has_unknown);
  }
}

TEST(TerrainGrid, RegistersHistoryInFixedFrameAndExpiresIt)
{
  TerrainGrid grid({});
  const auto points = surface(-0.1);
  grid.update(points, {0.0, 0.0, 0.0, 0.0}, 0.0);
  grid.update(points, {0.05, 0.0, 0.0, 0.0}, 0.2);
  const auto moving = grid.update(points, {0.1, 0.0, 0.0, 0.0}, 0.4);
  EXPECT_GT(marked(moving.drop_offs), 0);

  const auto expired = grid.update({}, {0.1, 0.0, 0.0, 0.0}, 2.0);
  EXPECT_EQ(marked(expired.hazards), 0);
  EXPECT_EQ(expired.observed_cells, 0);
}

TEST(TerrainGrid, MountedVlp16HasSparseFarGroundOnFlatSurface)
{
  TerrainGrid grid({});
  const auto result = confirmed(grid, approximateVlp16FlatGround());
  int near_observed = 0;
  int far_observed = 0;
  for (int iy = 0; iy < result.height; ++iy) {
    for (int ix = 0; ix < result.width; ++ix) {
      const auto value = result.hazards[static_cast<std::size_t>(iy * result.width + ix)];
      near_observed += ix < 10 && value >= 0;
      far_observed += ix >= 40 && value >= 0;
    }
  }
  EXPECT_GT(near_observed, 0);
  EXPECT_EQ(far_observed, 0);
  EXPECT_EQ(marked(result.hazards), 0);
}
