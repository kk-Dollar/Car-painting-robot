"""
amr_station_reach.py (v2) - which stations around the car can reach which surfaces?

Run it with RoboDK open and the AMR test station loaded:
    python3 amr_station_reach.py

What changed from v1
  The first version gave identical results for every station. The diagnostic showed why: the
  robot DOES follow the platform, but RoboDK's "reference frame" stays rigidly attached to the
  robot's base, so my absolute poses were read in a frame that moved with the robot.
  v2 never touches the reference frame. It converts every target to robot-base coordinates
  itself and works out (by a self-test) which SolveIK call convention matches your RoboDK.

What it does
  1. Self-test: checks that SolveIK reproduces the robot's CURRENT joints from its current pose.
     If no convention passes, it stops and tells you (no more silent wrong tables).
  2. Moves the platform (robot on it) to 8 stations around the car, facing the car.
     The platform keeps its current height, so the robot base height stays as you built it.
  3. Checks reach (and collisions, if enabled) for sample spray poses on the car surfaces.
  4. Prints a table and puts the platform back.

NOT YET TESTED against your RoboDK build beyond the diagnostic - send me any error message.
"""
import math
import numpy as np

try:
    from robodk import robolink, robomath
except ImportError:  # old RoboDK installs
    import robolink
    import robomath

# ---------------------------------------------------------------- settings
ROBOT_NAME = "Fanuc P-200 LA HB"
TOOL_NAME = "Paint gun"
PLATFORM_KEY = "amr_platform"     # part of the platform object's name
CAR_KEY = "car_xr_sedan"          # part of the car object's name (the real car, not car_collision)
STANDOFF = 250.0                  # mm, nozzle to surface
CHECK_COLLISIONS = True           # also test the IK solution against the collision map
YAW_OFFSETS = (0,)                # 0 won clearly in the last run (16 OK pairs vs 2-5); use (0, 90, 180, 270) to re-check

# Stations in CAR coordinates (mm): name, x (front +), y (left +)
STATIONS = [
    ("Left",        0.0,  2350.0),
    ("Right",       0.0, -2350.0),
    ("LeftFwd",  1500.0,  2350.0),
    ("RightFwd", 1500.0, -2350.0),
    ("LeftAft", -1500.0,  2350.0),
    ("RightAft", -1500.0, -2350.0),
    ("LeftClose",   0.0,  1900.0),   # closer to the car: tries to reach the roof centre
    ("RightClose",  0.0, -1900.0),
    ("Front",    3800.0,     0.0),
    ("Rear",    -3800.0,     0.0),
]

# Sample points in CAR coordinates (mm): name, position, outward normal. Heights approximate.
POINTS = [
    ("roof centre",     (-300,    0, 1600), (0, 0, 1)),
    ("roof left",       (-300,  600, 1580), (0, 0, 1)),
    ("roof right",      (-300, -600, 1580), (0, 0, 1)),
    ("hood centre",     (1700,    0,  950), (0, 0, 1)),
    ("trunk centre",    (-1950,   0, 1250), (0, 0, 1)),
    ("left door",       (-300,  950,  850), (0, 1, 0)),
    ("right door",      (-300, -950,  850), (0, -1, 0)),
    ("left fender",     (1500,  900,  800), (0, 1, 0)),
    ("right fender",    (1500, -900,  800), (0, -1, 0)),
    ("left rear",       (-1500, 900,  850), (0, 1, 0)),
    ("right rear",      (-1500, -900, 850), (0, -1, 0)),
    ("front face",      (2400,    0,  600), (1, 0, 0)),
    ("rear face",       (-2400,   0,  650), (-1, 0, 0)),
]


# ---------------------------------------------------------------- maths
def unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def look_pose(point, normal, standoff):
    """4x4 pose in car coordinates: `standoff` mm off the surface, tool Z pointing into it."""
    n = unit(normal)
    z = -n
    ref = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(ref, z)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    x = unit(ref - np.dot(ref, z) * z)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z
    T[:3, 3] = np.asarray(point, dtype=float) + n * standoff
    return T


def station_pose(x, y, yaw_offset_deg):
    """4x4 platform pose in car coordinates: at (x, y), +X axis turned toward the car centre, plus an offset."""
    yaw = math.atan2(-y, -x) + math.radians(yaw_offset_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    T[0, 3], T[1, 3] = x, y
    return T


def to_mat(T):
    return robomath.Mat(np.asarray(T, dtype=float).tolist())


def to_np(M):
    return np.array(M.rows, dtype=float)


# ---------------------------------------------------------------- station lookup
def _norm(text):
    return text.lower().replace("\u00d7", "x")


def find_item(RDK, item_type, key, exclude="collision"):
    exact = RDK.Item(key, item_type)
    if exact.Valid():
        return exact
    for item in RDK.ItemList(item_type):
        name = _norm(item.Name())
        if _norm(key) in name and exclude not in name:
            return item
    return exact


# ---------------------------------------------------------------- IK convention self-test
def joints_match(sol, current, n):
    if sol is None:
        return False
    vals = sol.list()
    if len(vals) < n:
        return False
    return all(abs((a - b + 180.0) % 360.0 - 180.0) < 1.0 for a, b in zip(vals[:n], current[:n]))


def calibrate_ik(robot, tool, base_np, tool_np, n):
    """Find which SolveIK call turns a base-relative TCP pose into the robot's current joints."""
    tcp_in_base = np.linalg.inv(base_np) @ to_np(tool.PoseAbs())
    current = robot.Joints().list()
    ident, tool_m = to_mat(np.eye(4)), to_mat(tool_np)
    inv_tool = np.linalg.inv(tool_np)
    candidates = [
        ("explicit tool + base frame", lambda T: robot.SolveIK(to_mat(T), tool=tool_m, reference=ident)),
        ("explicit tool",              lambda T: robot.SolveIK(to_mat(T), tool=tool_m)),
        ("default",                    lambda T: robot.SolveIK(to_mat(T))),
        ("flange pose",                lambda T: robot.SolveIK(to_mat(T @ inv_tool), tool=ident, reference=ident)),
    ]
    log = []
    for name, call in candidates:
        try:
            sol = call(tcp_in_base)
        except Exception as exc:  # keep going, report at the end
            log.append(f"  {name}: error {exc}")
            continue
        if joints_match(sol, current, n):
            return name, call
        got = None if sol is None else [round(v, 1) for v in sol.list()[:n]]
        log.append(f"  {name}: returned {got}")
    print("Self-test FAILED. Current joints:", [round(v, 1) for v in current[:n]])
    print("TCP pose in base frame (mm):", np.round(tcp_in_base[:3, 3], 1).tolist())
    print("\n".join(log))
    return None, None


# ---------------------------------------------------------------- main
def main():
    RDK = robolink.Robolink()
    robot = RDK.Item(ROBOT_NAME, robolink.ITEM_TYPE_ROBOT)
    tool = RDK.Item(TOOL_NAME, robolink.ITEM_TYPE_TOOL)
    platform = find_item(RDK, robolink.ITEM_TYPE_OBJECT, PLATFORM_KEY)
    car = find_item(RDK, robolink.ITEM_TYPE_OBJECT, CAR_KEY)
    for label, item in (("robot", robot), ("tool", tool), ("platform", platform), ("car", car)):
        if not item.Valid():
            names = [i.Name() for i in RDK.ItemList(robolink.ITEM_TYPE_OBJECT)]
            raise SystemExit(f"Could not find the {label}. Objects in the station: {names}")
    base = robot.Parent()                      # the robot's base frame item
    print(f"Using: robot='{robot.Name()}', tool='{tool.Name()}', platform='{platform.Name()}', "
          f"car='{car.Name()}', base frame='{base.Name()}'\n")

    robot.setPoseTool(tool)
    n = len(robot.Joints().list())
    original_platform = platform.PoseAbs()
    original_joints = robot.Joints()
    platform_z = to_np(original_platform)[2, 3]           # keep the height you built
    tool_np = to_np(robot.PoseTool())

    name, ik = calibrate_ik(robot, tool, to_np(base.PoseAbs()), tool_np, n)
    if ik is None:
        raise SystemExit("\nStopping: no SolveIK convention reproduced the current joints. "
                         "Paste the lines above to me.")
    print(f"IK self-test passed with convention: {name}\n")

    if CHECK_COLLISIONS:
        RDK.setCollisionActive(robolink.COLLISION_ON)

    car_T = to_np(car.PoseAbs())
    results = {}
    for off in YAW_OFFSETS:
        res = {}
        for sname, sx, sy in STATIONS:
            T = car_T @ station_pose(sx, sy, off)
            T[2, 3] = platform_z
            platform.setPoseAbs(to_mat(T))
            base_inv = np.linalg.inv(to_np(base.PoseAbs()))   # re-read: the robot moved with the platform
            row = []
            for pname, pos, nrm in POINTS:
                target_in_base = base_inv @ car_T @ look_pose(pos, nrm, STANDOFF)
                try:
                    sol = ik(target_in_base)
                except Exception:
                    sol = None
                if sol is None or len(sol.list()) < n:
                    row.append("--")
                    continue
                status = "OK"
                if CHECK_COLLISIONS:
                    robot.setJoints(sol)
                    if RDK.Collisions() > 0:
                        status = "COL"
                row.append(status)
            res[sname] = row
        results[off] = res

    def score(off):
        return sum(r.count("OK") for r in results[off].values())

    best = max(YAW_OFFSETS, key=score)
    res = results[best]

    platform.setPoseAbs(original_platform)                # put everything back
    robot.setJoints(original_joints)

    print(f"Yaw offsets tried -> OK pairs: " + ", ".join(f"{o} deg: {score(o)}" for o in YAW_OFFSETS))
    print(f"Using {best} deg for the table below.\n")
    names = [s[0] for s in STATIONS]
    print(f"{'surface':14s}" + "".join(f"{n_[:10]:>11s}" for n_ in names))
    print("-" * (14 + 11 * len(names)))
    for i, (pname, _, _) in enumerate(POINTS):
        print(f"{pname:14s}" + "".join(f"{res[n_][i]:>11s}" for n_ in names))
    print("\nOK = reachable and collision-free, COL = reachable but collides, -- = not reachable")
    print("\nStations that cover each surface:")
    for i, (pname, _, _) in enumerate(POINTS):
        ok = [n_ for n_ in names if res[n_][i] == "OK"]
        print(f"  {pname:14s} {', '.join(ok) if ok else 'NO STATION - needs another position'}")


if __name__ == "__main__":
    main()
