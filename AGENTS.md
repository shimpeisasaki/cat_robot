# cat_robot maintenance rules

Apply the workspace `../../AGENTS.md` as well as these rules.

- Keep the seven package responsibilities in [docs/README.md](docs/README.md).
  Driver, safety and sensor code must stay independent of exploration, D1 and
  comparison algorithms. External clients consume [docs/control.md](docs/control.md).
- Do not start hardware, launch, replay bags, simulate or send motion commands
  without explicit permission for that operation. An offline test is not permission.
  Never add automatic enabling/rearming or a second motor command path.
- Preserve Horizon clock fault latches, VIO world-pose input and origin handling,
  driver lease handshake, source exclusivity, and GNSS argument propagation.
  Calibration changes require evidence and an adopted calibration record.
- Change the owning specification and user instructions in the same change as
  interfaces, configuration, entry points or behavior. Keep history/evaluation
  records distinct from current procedures; do not rewrite old results as new ones.
- Before tests, inspect fixtures for hardware, ROS graph, processes and external
  access. Use Humble, focused packages, localhost and isolated domains for synthetic
  ROS tests. Leave `CAT_TEST_HORIZON` unset unless hardware testing is authorized.
- Record full tested SHA, dirty diff, OS/ROS/Python and dependencies, inputs,
  exact commands, exit codes, passed/failed/skipped counts and unverified conditions.
  Separate offline checks from simulation and hardware acceptance.
- Read bag/image/cloud/model/log metadata first (paths, sizes, metadata and existing
  summaries). Open payloads only for a named question with bounded files, topics,
  time ranges and sample counts. Do not replay, expand or encode all data.
  Exclude generated outputs, data and models from broad source searches.
- Keep complete failure logs in generated output directories; report bounded
  causal excerpts and their paths, preserving the original command exit code.
  Do not delete data, calibration/evaluation results, licenses or history.
- Follow [docs/maintenance.md](docs/maintenance.md) for verification and provenance.
