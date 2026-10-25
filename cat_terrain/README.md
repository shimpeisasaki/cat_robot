# cat_terrain

`terrain_step_detector` is a standalone CPU-only ROS 2 node for testing whether
VLP-16 point clouds can identify ground steps in the CAT robot's forward path.
Its internal representation is a set of radial ground profiles centred on the
rear-mounted VLP-16; the 2D grids are output formats for RViz only.

## Synthetic scenes

`terrain_pointcloud_generator` is a separate offline source of VLP-16-like
`PointCloud2` data. It casts 16 nominal laser channels from the configured
mounting position into YAML-defined horizontal surfaces and boxes. The default
mount is the current `VLP16_B` position and orientation. This generator does
not start any hardware. It publishes `/sim/velodyne_points` and matching
`sim_odom -> sim_base_link -> sim_vlp16` transforms, so the terrain detector
can run with the same time-alignment logic as it uses with real scans. RViz also
shows the full scene as translucent `/sim/scene_markers`, including floor areas
that receive no laser return. The existing CAT URDF is displayed as a robot
model. Its link names are prefixed with `sim_` to keep its TF separate from
any real robot.

Both detector launches load `config/terrain.yaml`. Its wildcard node selector
allows the simulation node name `sim_terrain_step_detector` to use the same
settings; pass this file only to the detector. The simulation launch overrides
input/output topics and TF frames to keep them under the simulation names.

Build and view the default flat floor with two boxes:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cat_terrain
source install/setup.bash
ros2 launch cat_terrain terrain_scene.launch.py show_rviz:=true
```

Choose a supplied scene or an absolute path to your own YAML file:

```bash
ros2 launch cat_terrain terrain_scene.launch.py scene:=$PWD/src/cat_robot/cat_terrain/scenes/drop_6cm.yaml show_rviz:=true
```

Edit a copy of a scene to create a new environment. The YAML uses metres and
degrees:

| Field | Meaning |
|---|---|
| `surfaces[].bounds_xy_m` | `[xmin, xmax, ymin, ymax]` of a horizontal floor patch |
| `surfaces[].height_m` | Height of that patch; `0` is the starting floor |
| `surfaces[].holes` | Rectangles removed from that patch; leave empty for ordinary floor |
| `boxes[].center_m`, `size_m`, `yaw_deg` | Solid box position, dimensions, and optional rotation |
| `robot.pose_xyzyaw_deg` | Initial base pose in `sim_odom` |
| `robot.motion[]` | Timed commands with `duration_s`, `linear_speed_mps`, and `yaw_rate_deg_s` |
| `sensor.range_noise_std_m`, `dropout_probability` | Optional independent range noise and missing returns |

For a 6 cm drop, cut a rectangle out of the original floor and add a
`height_m: -0.06` surface with the same bounds. Without the cutout, the higher
floor hides the lower one. For an open gap, make a cutout and add no lower
surface. For a 6 cm curb, add a box whose bottom is at zero and top at 0.06 m.
The examples are `flat_boxes.yaml`, `drop_6cm.yaml`, and `gap_50cm.yaml` in
`scenes/`. Add more surfaces and boxes to either list to build stairs or other
obstacles. Restart the launch after editing the YAML.

The `drop_6cm.yaml` example waits for 2 seconds, moves forward at 0.3 m/s for
3 seconds, then stops. Each scan is ray-cast from the current pose and the
corresponding odometry transform has the same timestamp. Motion is ideal and
does not include wheel slip, per-point motion during a scan, or body motion
over a drop. The nominal laser calibration is not an individual sensor
calibration. Synthetic detections still need checking against recorded data.

The simulation uses separate `/sim` topics and frames. The output remains an
RViz experiment and is not connected to Nav2 or motor commands.

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

- `cat_terrain/scene.py`: ROS-independent scene geometry, ray casting, and motion.
- `cat_terrain/scene_node.py`: ROS point cloud, marker, and TF publishing.
- `scripts/terrain_pointcloud_generator`: executable entry point.
- `src/terrain_grid.cpp`: ROS-independent radial terrain evaluation; the grid
  is its output representation.
- `src/terrain_step_detector.cpp`: ROS input transforms and output publishing.

The Python modules are installed with `ament_cmake_python`, matching the
other Python packages in this repository. The scene geometry tests import
the package directly without adding the executable directory to `sys.path`.
