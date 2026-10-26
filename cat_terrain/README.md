# cat_terrain

`terrain_step_detector` is a standalone CPU-only ROS 2 node for testing whether
VLP-16 point clouds can identify ground steps in the CAT robot's forward path.
Its internal representation is a set of radial ground profiles centred on the
rear-mounted VLP-16; the 2D grids are output formats for RViz only.

## Simulation

The STL LiDAR simulator is maintained separately in the `cat_robot_sim`
repository under `src/`. Its launch starts this package's detector with
simulated point clouds and isolated TF frames. See that repository's README
for STL coordinates, motion commands, and RViz use.

The node subscribes to `/velodyne_points`. It records points up to `base_link`
`x=5.5 m` in a 90-degree forward sector around the VLP-16. Only points with
`z=-0.5..0.35 m` in `base_link` enter the model. Points are transformed into
`odom` at the cloud timestamp and retained for up to 24 scans or 2 seconds.
Each scan is compressed in 2-degree angular and 5 cm radial bins. Stored
points are placed into current VLP-16-centred rays using odometry, and nearby
returns are merged into a height profile. The LiDAR-to-`base_link` region is
an assumed zero-height reference, never a measured free region.

Adjacent profile points are compared using their actual separation. The
ordinary ground grade is estimated from the neighboring pairs; the remaining
height change must be at least 0.03 m and its grade above 5 degrees. A pair
distance no greater than 1 m, observations from three scans, and support from
a neighboring ray are also required. The evaluated output region is
`base_link` `x=0.35..3.0 m`, `y=-1.0..1.0 m`; straight ahead, 3 m from
`base_link` is approximately 3.35 m from the VLP-16. A gap without a farther
return stays **unknown**. A long gap or a height difference too small for its
separation is not marked as a step.

Outputs are `nav_msgs/msg/OccupancyGrid` in `base_link`:

| Topic | Contents |
|---|---|
| `/terrain/hazards` | Confirmed up and down steps |
| `/terrain/up_steps` | Confirmed rising steps |
| `/terrain/drop_offs` | Confirmed falling steps |
| `/terrain/valid` | `true` when recent clouds produced confirmed ground observations |
| `/terrain/coverage_by_range` | Fraction of observed output cells in successive 1 m bands, starting at `min_x` |
| `/terrain/up_cells`, `/terrain/drop_cells` | Confirmed cells as `GridCells` for clear RViz overlay |

Cells are `100` for a confirmed edge, `0` for observed ground, and `-1` for
unknown. The outputs are for evaluation and RViz. They are **not** connected to
Nav2 or motor commands. If no usable cloud arrives for 0.6 seconds, a watchdog
publishes an all-unknown grid and `valid=false`. Keep
`/local_costmap/costmap_raw` owned by Nav2.

With the default region, `/terrain/coverage_by_range` has three values for
`x=[0.35,1.35), [1.35,2.35), [2.35,3.0)` m. The 5.5 m recording limit keeps
far observations available as the robot approaches them.

Build with `colcon build --symlink-install --packages-select cat_terrain` after
sourcing ROS 2 Humble. When an existing source already publishes the point
cloud and TF, run `ros2 launch cat_terrain terrain_step_detector.launch.py`.
The launch starts only this detector. It does not start LiDAR, motors, or Nav2.
Set `show_rviz:=true` to use the included point-cloud and hazard-grid display.
For a stationary tabletop check without an `odom -> base_link` transform, set
`fixed_frame:=base_link`; this disables meaningful motion registration until
the normal odometry source is running.

To inspect a stationary robot without starting its motors or camera, use two
terminals after sourcing ROS 2 Humble and this workspace:

```bash
ros2 launch cat_bringup bringup.launch.py use_base:=false use_zed:=false use_gnss:=false use_lidar:=true lidar_source:=vlp16 vlp16_mount:=back rviz:=false enable_joystick:=false enable_panorama:=false
ros2 launch cat_terrain terrain_step_detector.launch.py fixed_frame:=base_link show_rviz:=true
```

RViz shows the VLP-16 cloud, yellow rising-step cells, and red falling-step
cells. Toggle the `Terrain hazards` Map display to inspect observed and
unknown cells. Shut down both launches normally to let the VLP-16 power
manager stop the sensor.

For a stationary check with the ZED color cloud as a visual reference, replace
the first command with the following. The dedicated ZED config enables its
registered `rgb` point cloud; the normal sensor config still leaves that
stream off. The same terrain RViz display shows it alongside the VLP-16 points
and step cells. Keep `use_base:=false` to leave motors unstarted.

```bash
ros2 launch cat_bringup bringup.launch.py use_base:=false use_zed:=true odom_source:=wheel use_gnss:=false use_lidar:=true lidar_source:=vlp16 vlp16_mount:=back rviz:=false enable_joystick:=false enable_panorama:=false zed_config:=$HOME/ros2_ws/src/cat_robot/cat_bringup/config/zed_terrain_debug.yaml
```

The ZED cloud is only a visual debug layer; it is not an input to the terrain
detector. Toggle `ZED colored points` in RViz when the additional processing
or screen clutter is unwanted.

This is a first-pass detector. VLP-16 vertical beams are sparse; the 3 cm
threshold is experimental and may produce false positives or miss real steps.
The assumed ground under the robot is not a measured surface. No-return gaps
alone cannot establish a drop-off. The node applies one transform per complete
cloud, so motion within a scan is not deskewed. The `base_link` height relative
to gravity also needs verification on uneven ground.
Record flat ground, slopes, vegetation, curbs, and stairs and compare labels
before using this output to influence navigation. CPU time is logged when a
cloud takes over 50 ms to process.

The 5.5 m recording limit is not a claim that every distance receives ground
returns. On flat ground, the fixed VLP-16 beams leave increasingly wide gaps
with distance. The history uses `odom` to bring successive scans into the
same place; cells without enough returns remain unknown.

## Code organization

- `src/terrain_grid.cpp`: ROS-independent radial terrain evaluation; the grid
  is its output representation.
- `src/terrain_step_detector.cpp`: ROS input transforms and output publishing.
