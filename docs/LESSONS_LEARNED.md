# Lessons learned

Concrete gotchas from building this cell with RoboDK and its Python API.

- **RoboDK Free license limits the Python API** ("API calls are limited"). A **trial** (or paid)
  license is needed for `build_zones.py` and the other API-heavy scripts.

- **`tool.PoseAbs()` is the tool item's frame, not the TCP.** The TCP is 450 mm further along.
  To read the real TCP, use `robot.Pose()` with the **Car Frame** set as the active reference.

- **Never pass a matrix to `robot.setPoseFrame`.** Passing a pose matrix moved the car; you must
  pass the frame **item** instead.

- **Inverse kinematics is measured from the `Lucas RC3-1Y1R` frame.** `SolveIK` works from that
  frame, and the IK self-test must start from a **bent-elbow** pose — the Home pose has a straight
  elbow, which is a singularity where RoboDK's IK finds no solution.

- **Long API runs need a long timeout.** Set `RDK.TIMEOUT` to ~30 min (1800 s); the API default
  is 10 s and a busy RoboDK (collision checks) then raises "Communication problems".

- **Collision avoidance inside Curve Follow is slow.** Solve **without** it first (pass 1), and
  only fall back to collision avoidance if needed (`--avoid-pass`). On the hood it could not
  attach at all with avoidance on.

- **Each `GunOn` call is a separate Python process.** Too many API calls per call froze the
  animation, which is why `GunOn.py` was reduced to a lightweight **state** script with its
  settings in a single station parameter (`PaintCfg`).

- **Wound-up elbow configurations (|J3| > 190 deg) made transitions impossible.** Programs that
  start and end in a different arm "family" cannot be joined by a safe transit, so such start
  poses are tried **last** (`--allow-wound` to accept them anyway).

- **RoboDK's particle spray would not land on this model.** Its own statistics reported 0 % "in"
  for every normal-size cone, and it was slow — hence the ray-cast **coverage overlay** approach
  (`paint_overlay.py`) instead of physical spray.
