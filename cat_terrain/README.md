# cat_terrain

`terrain_step_detector` is a standalone CPU-only ROS 2 node for testing whether
VLP-16 point clouds can identify ground steps in the CAT robot's forward path.
It is informed by the grid-and-height-difference approach in
[kei487/elevation_costmap](https://github.com/kei487/elevation_costmap), but
keeps its own time-aligned terrain observations and step logic.

The node subscribes to `/velodyne_points`. The default region is `base_link`
`x=0.6..5.0 m`, `y=-1.0..1.0 m`. Only points with `z=-0.5..0.35 m` in
`base_link` enter the model. It transforms those points into `odom` at the cloud
timestamp, keeps up to six scans or 1.2 seconds, and compares ground-height
estimates in front-to-back neighboring 0.1 m cells. An edge needs a height
difference of at least 0.06 m, a grade above 15 degrees, three separate scans,
and evidence in at least two nearby lateral cells. A lack of returns stays
**unknown**; it does not by itself prove a drop or safe ground.

Outputs are `nav_msgs/msg/OccupancyGrid` in `base_link`:

| Topic | Contents |
|---|---|
| `/terrain/hazards` | Confirmed up and down steps |
| `/terrain/up_steps` | Confirmed rising steps |
| `/terrain/drop_offs` | Confirmed falling steps |
| `/terrain/valid` | `true` when recent clouds produced confirmed ground observations |
| `/terrain/coverage_by_range` | Fraction of confirmed ground cells in successive 1 m bands, starting at `min_x` |

Cells are `100` for a confirmed edge, `0` for observed ground, and `-1` for
unknown. The outputs are for evaluation and RViz. They are **not** connected to
Nav2 or motor commands. If no usable cloud arrives for 0.6 seconds, a watchdog
publishes an all-unknown grid and `valid=false`. Keep
`/local_costmap/costmap_raw` owned by Nav2.

With the default region, `/terrain/coverage_by_range` has five values for
`x=[0.6,1.6), [1.6,2.6), [2.6,3.6), [3.6,4.6), [4.6,5.0)` m. This makes
the VLP-16's actual ground coverage visible during offline bag evaluation.

Build with `colcon build --symlink-install --packages-select cat_terrain` after
sourcing ROS 2 Humble. When an existing source already publishes the point
cloud and TF, run `ros2 launch cat_terrain terrain_step_detector.launch.py`.
The launch starts only this detector. It does not start LiDAR, motors, or Nav2.

This is a first-pass detector. A VLP-16 has sparse vertical beams and typical
range accuracy of about 3 cm. It cannot guarantee detection of every 6 cm step,
especially if the far surface is unobserved. The node currently applies one
transform per complete cloud, so motion within a scan is not deskewed. The
`base_link` height relative to gravity also needs verification on uneven ground.
Record flat ground, slopes, vegetation, curbs, and stairs and compare labels
before using this output to influence navigation. CPU time is logged when a
cloud takes over 50 ms to process.

The configured 5 m region is an evaluation boundary, not a claim that every
cell receives ground returns. On flat ground, the fixed 2-degree VLP-16 beams
leave increasingly wide gaps with distance. The history uses `odom` to bring
successive scans into the same place, while `/terrain/coverage_by_range` shows
where this actually produces enough observations. Cells without enough returns
remain unknown.
