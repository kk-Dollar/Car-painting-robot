"""
gantry_reach.py - can a small arm on a 3-axis gantry reach the car? ("gantry carries, arm orients")

Station: a 6-axis arm (UR10) mounted on the flange of a 3-axis gantry and synchronised with it
(Utilities > Synchronize External Axes), so the robot has 9 joints: 6 arm joints + d7, d8, d9 (gantry X, Y, Z).

RoboDK solves the inverse kinematics of the ARM only. The gantry axes are inputs (RoboDK forum: set them first
with robot.setJoints([...6 arm..., d7, d8, d9]) and then call SolveIK; the external values are not changed).
So this script decides the gantry position itself with one rule:

    put the arm's base a distance L BEHIND the gun, along the gun's axis:  base = gun_tip - L * tool_Z
    -> the gantry does the large movement, the arm only has to point the gun.
    If the gantry cannot get there (for example the roof: the base would have to be higher than the gantry's top),
    it also tries sideways shifts of that position (0, 300, 600 mm in the plane across the gun axis).

For every sample point on the car it tries L = 550, 700, 850, 400, 250, 150 mm, converts the wanted base position into
d7, d8, d9 (using the gantry's own axis directions, measured from the station), checks the gantry limits, sets
the joints, solves the arm, and checks collisions. It prints a table and the gantry travel that is actually used.

    python3 gantry_reach.py            # the reach test
    python3 gantry_reach.py --mark     # only drop a target at every sample pose, to SEE where the test points are
    python3 gantry_reach.py --no-align # trust the car object's pose without reading its mesh
Before testing, the script reads the car's mesh from RoboDK and works out where the car really is relative to the
object's origin (a "Move geometry" or a copied object can shift or turn the geometry), so the sample points land
ON the car and not inside it.
Run with RoboDK open on the gantry station. NOT YET TESTED against your RoboDK build (the maths is tested offline).
"""
import argparse
import itertools
from collections import Counter

import numpy as np

try:
    from robodk import robolink, robomath
except ImportError:
    import robolink
    import robomath

# ---------------------------------------------------------------- settings
ROBOT_NAME = "UR10"
TOOL_NAME = "Paint gun"
CAR_KEY = "car_xr_sedan"            # the full car (collisions); the paint-only skin is ignored
STANDOFF = 250.0                    # mm, nozzle to surface
L_CANDIDATES = (550.0, 700.0, 850.0, 400.0, 250.0, 150.0, 0.0)   # mm, base-behind-the-gun distances to try (short ones are for height-limited spots like the roof)
LATERAL = (0.0, 300.0, 600.0, 900.0)                 # mm, sideways shifts of the base (tried if the straight one fails)
CHECK_COLLISIONS = True
AUTO_DISABLE_PERMANENT_PAIRS = True   # switch off collision pairs that collide at EVERY gantry corner (overlapping copies, parts that touch by design)
EXT_LIMITS_FALLBACK = ((0.0, 7750.0), (0.0, 5000.0), (0.0, 2000.0))   # from your screenshot (d7, d8, d9)

EXPECTED_EXTENTS = (4800.0, 1900.0, 1600.0)   # length, width, height of the full car STL (front = +X, tyres on Z = 0)

POINTS = [   # name, position (car coords, mm), outward normal
    ("roof centre",  (-300,    0, 1600), (0, 0, 1)),
    ("roof left",    (-300,  400, 1580), (0, 0, 1)),
    ("roof right",   (-300, -400, 1580), (0, 0, 1)),
    ("hood centre",  (1700,    0,  950), (0, 0, 1)),
    ("trunk centre", (-1850,   0, 1150), (0, 0, 1)),
    ("left door",    (-300,  800,  850), (0, 1, 0)),
    ("right door",   (-300, -800,  850), (0, -1, 0)),
    ("left fender",  (1500,  800,  880), (0, 1, 0)),
    ("right fender", (1500, -800,  880), (0, -1, 0)),
    ("left rear",    (-1500, 800,  880), (0, 1, 0)),
    ("right rear",   (-1500, -800, 880), (0, -1, 0)),
    ("left sill",    (-300,  800,  420), (0, 1, 0)),
    ("right sill",   (-300, -800,  420), (0, -1, 0)),
    ("front face",   (2400,    0,  600), (1, 0, 0)),
    ("rear face",    (-2400,   0,  650), (-1, 0, 0)),
]


# ---------------------------------------------------------------- maths
def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def look_pose(point, normal, standoff):
    """4x4 pose in car coordinates: `standoff` off the surface, tool Z into the surface."""
    n = unit(normal)
    z = -n
    ref = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(ref, z)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    x = unit(ref - np.dot(ref, z) * z)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z
    T[:3, 3] = np.asarray(point, float) + n * standoff
    return T


def base_candidates(tip, zdir, Ls=L_CANDIDATES, laterals=LATERAL):
    """Wanted base positions, nearest-to-the-straight-rule first: (lateral shift, L, position)."""
    z = unit(zdir)
    ref = np.array([0.0, 0.0, 1.0]) if abs(z[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = unit(np.cross(z, ref))
    v = np.cross(z, u)
    out = []
    for lat in laterals:
        dirs = [(0.0, 0.0)] if lat == 0 else [(1, 0), (-1, 0), (0, 1), (0, -1)]
        for a, b in dirs:
            for L in Ls:
                out.append((lat, L, np.asarray(tip, float) - L * z + lat * (a * u + b * v)))
    return out


def chain_frames(robot):
    """Frames above the robot, nearest first (UR10 Base, the gantry mechanism, the gantry base, ...), each as
    (name, function returning its 4x4 absolute pose NOW, the frame item or None). Plus the station origin as a last resort."""
    frames, item = [], robot.Parent()
    for _ in range(5):
        if item is None or not item.Valid():
            break
        try:
            to_np(item.PoseAbs())
            frames.append((item.Name(), (lambda it=item: to_np(it.PoseAbs())), item))
        except Exception:
            pass
        item = item.Parent()
    frames.append(("station origin", lambda: np.eye(4), None))
    return frames


def arm_error(sol, arm_joints, n_arm):
    """Largest joint difference (deg) between a SolveIK result and the expected arm joints, or None."""
    if sol is None or len(sol.list()) < n_arm:
        return None
    return max(abs((a - b + 180.0) % 360.0 - 180.0) for a, b in zip(sol.list()[:n_arm], arm_joints))


def arm_matches(sol, arm_joints, n_arm, tol=3.0):
    err = arm_error(sol, arm_joints, n_arm)
    return err is not None and err < tol


def _wrap_err(a, b):
    return max(abs((x - y + 180.0) % 360.0 - 180.0) for x, y in zip(a, b))


def find_pose_frame(robot, tool, arm_joints, n_arm, tol=3.0):
    """Which frame does SolveIK measure poses from?  For every frame above the robot: make it the robot's ACTIVE reference frame
    (an item, so nothing is moved), give SolveIK / SolveIK_All the gun's pose expressed in that frame, and see whether any
    solution reproduces the current arm joints (within tol deg; the tool pose read back from RoboDK is not exact).
    The explicit call SolveIK(pose, None, tool, identity) is tried too. The winner stays the active reference frame, so plain
    SolveIK calls later on are consistent with it. Returns (getter, name, report_lines); getter.item is the frame item."""
    tcp_abs = to_np(tool.PoseAbs())
    best, report = None, []
    for fname, getter, item in chain_frames(robot):
        if item is not None:
            try:
                robot.setPoseFrame(item)
            except Exception:
                pass
        T_f = to_mat(np.linalg.inv(getter()) @ tcp_abs)
        cands = []                                       # (label, list of arm-joint lists)
        try:
            sol = robot.SolveIK(T_f)
            if sol is not None and len(sol.list()) >= n_arm:
                cands.append(("SolveIK", [sol.list()[:n_arm]]))
        except Exception:
            pass
        try:
            M = robot.SolveIK_All(T_f)
            rows = M.rows
            if len(rows) >= n_arm and rows[0]:
                cands.append(("SolveIK_All", [[float(rows[r][c]) for r in range(n_arm)] for c in range(len(rows[0]))]))
        except Exception:
            pass
        try:
            sol = robot.SolveIK(T_f, None, robot.PoseTool(), to_mat(np.eye(4)))
            if sol is not None and len(sol.list()) >= n_arm:
                cands.append(("explicit SolveIK", [sol.list()[:n_arm]]))
        except Exception:
            pass
        scored = [(min(_wrap_err(s_, arm_joints) for s_ in sols), label, sols) for label, sols in cands if sols]
        if not scored:
            report.append(f"  {fname:28s} -> no solution")
            continue
        err, label, sols = min(scored, key=lambda x: x[0])
        nearest = min(sols, key=lambda s_: _wrap_err(s_, arm_joints))
        report.append(f"  {fname:28s} -> max joint difference {err:.1f} deg ({label}); nearest solution "
                      f"{[round(v) for v in nearest]} vs current {[round(v) for v in arm_joints]}")
        if err < tol and (best is None or err < best[0]):
            getter.item = item
            best = (err, getter, fname, item)
    if best is None:
        return None, None, report
    if best[3] is not None:
        try:
            robot.setPoseFrame(best[3])
        except Exception:
            pass
    return best[1], best[2], report


def find_permanent_pairs(RDK, set_ext, limits):
    """Collision pairs that are in collision with the gantry at ALL 8 corners of its travel box: they do not depend
    on where the gantry is (two overlapping copies of the car, a part that touches its mount, ...)."""
    corners = list(itertools.product(*limits))
    seen, keep = Counter(), {}
    for c in corners:
        set_ext(c)
        if RDK.Collisions() > 0:
            for it1, it2, i1, i2 in RDK.CollisionPairs():
                key = (it1.Name(), i1, it2.Name(), i2)
                seen[key] += 1
                keep[key] = (it1, it2, i1, i2)
    return [keep[k] for k, n in seen.items() if n == len(corners)]


def pair_name(pair):
    it1, it2, i1, i2 = pair
    return f"{it1.Name()}[{i1}] <-> {it2.Name()}[{i2}]"


def excess(ext, limits):
    return sum(max(0.0, lo - v, v - hi) for v, (lo, hi) in zip(ext, limits))


def rot_z(deg):
    c, sn = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    R = np.eye(4)
    R[:2, :2] = [[c, -sn], [sn, c]]
    return R


def translate(t):
    T = np.eye(4)
    T[:3, 3] = t
    return T


def align_from_points(P, expected=EXPECTED_EXTENTS, tol=0.08):
    """P: mesh vertices in the car OBJECT's coordinates. Returns (M, info): M maps the car's own coordinates
    (centred on the footprint, tyres at Z=0, front +X) to object coordinates. None if the shape does not match."""
    P = np.asarray(P, float)[:, :3]
    best = None
    for theta in (0, 90, 180, 270):
        Pp = P @ rot_z(theta)[:3, :3].T
        lo, hi = Pp.min(axis=0), Pp.max(axis=0)
        ext = hi - lo
        if not np.allclose(ext, expected, rtol=tol):
            continue
        top = Pp[Pp[:, 2] > lo[2] + 0.85 * ext[2]]
        cabin_x = top[:, 0].mean() - (lo[0] + hi[0]) / 2.0          # the cabin sits behind the middle: front is +X
        c = np.array([(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, lo[2]])
        M = rot_z(-theta) @ translate(c)
        info = dict(theta=theta, centre=c, extents=ext, cabin_x=cabin_x)
        if cabin_x < 0:
            return M, info
        best = best or (M, info)
    return best if best else (None, None)


def read_mesh_points(car, max_surfaces=2000):
    """All vertices of the car object in its own coordinates. RoboDK splits an object into surfaces and
    GetPoints(FEATURE_OBJECT_MESH, id) returns ONE surface, so loop over the ids until they run out."""
    chunks, misses = [], 0
    for sid in range(max_surfaces):
        try:
            pts, _ = car.GetPoints(robolink.FEATURE_OBJECT_MESH, sid)
            arr = np.array(pts, float)
            if arr.ndim == 2 and arr.shape[0] > 0:
                chunks.append(arr[:, :3])
                misses = 0
                continue
        except Exception:
            pass
        misses += 1
        if misses >= 3:
            break
    if not chunks:
        return None, 0
    return np.vstack(chunks), len(chunks)


def car_alignment(car):
    """Read the car's mesh through the API and align it. Returns (M, P, info) or (None, P_or_None, message)."""
    try:
        P, n_surf = read_mesh_points(car)
    except Exception as exc:
        return None, None, f"could not read the mesh ({exc})"
    if P is None or len(P) < 100:
        return None, None, "the mesh read returned no points"
    M, info = align_from_points(P)
    if M is None:
        lo, hi = P.min(axis=0), P.max(axis=0)
        return None, P, (f"mesh read from {n_surf} surface(s), {len(P)} points; extents {np.round(hi - lo).tolist()} "
                         f"do not match the expected {list(EXPECTED_EXTENTS)}")
    info["surfaces"] = n_surf
    return M, P, info


def nearest_vertex_distance(points, P, chunk=20000):
    pts = np.asarray(points, float)
    out = np.full(len(pts), np.inf)
    for i in range(0, len(P), chunk):
        d = np.linalg.norm(P[i:i + chunk][None, :, :] - pts[:, None, :], axis=2).min(axis=1)
        out = np.minimum(out, d)
    return out


def measure_tool_reach(tool, tcp_np, max_surfaces=300):
    """How far the tool's geometry (the red spray cone) reaches IN FRONT of the TCP along the tool's Z axis, mm.
    Assumes the mesh points are given in the flange frame (the tool's own frame). Returns (beyond_mm, n_points) or (None, 0)."""
    chunks, misses = [], 0
    for sid in range(max_surfaces):
        try:
            pts, _ = tool.GetPoints(robolink.FEATURE_OBJECT_MESH, sid)
            arr = np.array(pts, float)
            if arr.ndim == 2 and arr.shape[0] > 0:
                chunks.append(arr[:, :3])
                misses = 0
                continue
        except Exception:
            pass
        misses += 1
        if misses >= 3:
            break
    if not chunks:
        return None, 0
    P = np.vstack(chunks)
    in_tcp = (np.linalg.inv(np.asarray(tcp_np, float)) @ np.c_[P, np.ones(len(P))].T).T[:, :3]
    return float(in_tcp[:, 2].max()), len(P)


def standoff_for(tool, tcp_np, requested, current):
    """'auto' -> a standoff that keeps the whole tool (cone included) out of the surface; a number -> that number."""
    if requested is None:
        return current
    if str(requested).lower() != "auto":
        return float(requested)
    beyond, n = measure_tool_reach(tool, tcp_np)
    if beyond is None:
        print(f"(could not read the tool mesh; keeping the standoff at {current:.0f} mm)")
        return current
    value = max(current, float(np.ceil((beyond + 40.0) / 10.0) * 10.0))
    print(f"Tool geometry reaches {beyond:.0f} mm in front of the TCP ({n} mesh points) -> standoff {value:.0f} mm.")
    return value


def to_mat(T):
    return robomath.Mat(np.asarray(T, float).tolist())


def to_np(M):
    return np.array(M.rows, float)


def gantry_map(base_positions):
    """base_positions: abs base origin at ext = (0,0,0), (1,0,0)*s, (0,1,0)*s, (0,0,1)*s with s = 1000.
    Returns p0 and the 3x3 matrix A with base = p0 + A @ [d7, d8, d9]."""
    p0 = base_positions[0]
    A = np.column_stack([(base_positions[i] - p0) / 1000.0 for i in (1, 2, 3)])
    return p0, A


def ext_for_base(p0, A, wanted_base):
    """Gantry joint values that put the arm's base at wanted_base."""
    return np.linalg.solve(A, np.asarray(wanted_base, float) - p0)


def within(ext, limits):
    return all(lo - 1e-6 <= v <= hi + 1e-6 for v, (lo, hi) in zip(ext, limits))


# ---------------------------------------------------------------- RoboDK side
def norm(t):
    return t.lower().replace("\u00d7", "x")


def read_limits(robot, n_arm, n_ext):
    try:
        lo, hi = robot.JointLimits()
        lo, hi = lo.list(), hi.list()
        lim = [(float(a), float(b)) for a, b in zip(lo[n_arm:n_arm + n_ext], hi[n_arm:n_arm + n_ext])]
        if len(lim) == n_ext:
            return lim
    except Exception:
        pass
    return list(EXT_LIMITS_FALLBACK)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mark", action="store_true", help="only place a target at each sample pose (to see them) and exit")
    ap.add_argument("--no-align", action="store_true", help="do not read the car mesh; trust the object's pose")
    ap.add_argument("--standoff", help="nozzle-to-surface distance in mm, or 'auto' (= the tool's own reach + 40)")
    args = ap.parse_args()
    RDK = robolink.Robolink()
    robot = RDK.Item(ROBOT_NAME, robolink.ITEM_TYPE_ROBOT)
    tool = RDK.Item(TOOL_NAME, robolink.ITEM_TYPE_TOOL)
    car = None
    for it in RDK.ItemList(robolink.ITEM_TYPE_OBJECT):
        n = norm(it.Name())
        if CAR_KEY in n and "paint_only" not in n and "collision" not in n:
            car = it
    if not robot.Valid() or not tool.Valid() or car is None:
        raise SystemExit("Could not find the robot, the tool or the car object - check the names at the top.")
    base = robot.Parent()
    robot.setPoseTool(tool)
    global STANDOFF
    STANDOFF = standoff_for(tool, to_np(robot.PoseTool()), args.standoff, STANDOFF)
    if args.standoff:
        print(f"Using a standoff of {STANDOFF:.0f} mm.\n")

    # where is the car REALLY? (object pose + any shift/turn baked into the geometry)
    car_T = to_np(car.PoseAbs())
    mesh_pts = None
    if not args.no_align:
        M, mesh_pts, info = car_alignment(car)
        if M is None:
            print(f"WARNING: could not align the car automatically: {info}.\n"
                  f"         Using the object's own pose; run with --mark to check where the test points land.\n")
        else:
            car_T = car_T @ M
            print("Car geometry check: the sample points follow the car's real position.")
            print(f"  mesh read from {info.get('surfaces', '?')} surface(s); turned {info['theta']} deg inside its object, "
                  f"footprint centre in object coordinates {np.round(info['centre']).tolist()}, "
                  f"extents {np.round(info['extents']).tolist()}\n")
    if args.mark:
        for old in RDK.ItemList(robolink.ITEM_TYPE_FRAME):
            if old.Name() == "Reach_test_points":
                old.Delete()
        frame = RDK.AddFrame("Reach_test_points")
        frame.setPoseAbs(to_mat(np.eye(4)))
        for name, pos_c, nrm in POINTS:
            tgt = RDK.AddTarget("pt " + name, frame, robot)
            tgt.setAsCartesianTarget()
            tgt.setPose(to_mat(car_T @ look_pose(pos_c, nrm, STANDOFF)))
        print(f"Placed {len(POINTS)} targets in the frame 'Reach_test_points': each sits {STANDOFF:.0f} mm off the surface "
              f"with its Z axis pointing into the car. If they are inside the car or far from it, tell me where.")
        return
    if mesh_pts is not None:
        surf_abs = [(car_T @ np.append(np.asarray(p, float), 1.0))[:3] for _, p, _ in POINTS]
        obj_T = to_np(car.PoseAbs())
        surf_obj = [(np.linalg.inv(obj_T) @ np.append(q, 1.0))[:3] for q in surf_abs]
        dist = nearest_vertex_distance(surf_obj, mesh_pts[:, :3] if mesh_pts.shape[1] > 3 else mesh_pts)
        far = [n for (n, _, _), d in zip(POINTS, dist) if d > 150.0]
        print(f"Sample points vs the car mesh: nearest mesh vertex within 150 mm for {len(POINTS) - len(far)} of {len(POINTS)} points.")
        if far:
            print("  points that do NOT sit on the car surface: " + ", ".join(far))
        print()

    original = robot.Joints().list()
    n_all = len(original)
    n_arm, n_ext = 6, n_all - 6
    if n_ext != 3:
        raise SystemExit(f"Expected 6 arm joints + 3 gantry axes (9 joints), found {n_all}. "
                         f"Is the gantry synchronised (Utilities > Synchronize External Axes)?")
    arm_home = list(original[:n_arm])
    limits = read_limits(robot, n_arm, n_ext)
    print(f"Robot '{robot.Name()}': {n_arm} arm joints + {n_ext} gantry axes. Gantry limits d7,d8,d9 (mm): {limits}\n")

    def set_ext(ext, arm=None):
        robot.setJoints((list(arm) if arm is not None else arm_home) + [float(v) for v in ext])

    # 1) how do the gantry joints move the arm's base? (measured, so no axis conventions are assumed)
    pos = []
    for e in ((0, 0, 0), (1000, 0, 0), (0, 1000, 0), (0, 0, 1000)):
        set_ext(e)
        pos.append(to_np(base.PoseAbs())[:3, 3])
    p0, A = gantry_map(pos)
    print("Gantry axes in station coordinates (base moves per mm of d7, d8, d9):")
    for name, col in zip(("d7", "d8", "d9"), A.T):
        print(f"  {name}: {np.round(col, 3).tolist()}")
    if abs(np.linalg.det(A)) < 0.5:
        raise SystemExit("The three gantry axes are not independent - cannot invert the mapping.")
    set_ext((0, 0, 0))
    rot0 = to_np(base.PoseAbs())[:3, :3]
    set_ext((1000, 1000, 500))
    if not np.allclose(rot0, to_np(base.PoseAbs())[:3, :3], atol=1e-6):
        print("WARNING: the base orientation changes with the gantry joints; results may be off.")

    # 2) self-test: from which frame does SolveIK measure poses? (for a synchronised gantry it may be the root of
    #    the chain and not the arm's own base frame). Try the frames above the robot until one gives back our joints.
    set_ext((original[n_arm], original[n_arm + 1], original[n_arm + 2]), arm_home)
    pose_frame, pose_frame_name, report = find_pose_frame(robot, tool, arm_home, n_arm)
    if pose_frame is None:
        raise SystemExit("IK self-test failed for every frame I tried:\n" + "\n".join(report) +
                         f"\nexpected arm joints {[round(v, 1) for v in arm_home]}.\nPaste this message to me.")
    print(f"IK self-test passed: SolveIK measures poses from the frame '{pose_frame_name}'.\n")
    if CHECK_COLLISIONS:
        RDK.setCollisionActive(robolink.COLLISION_ON)
        try:
            perm = find_permanent_pairs(RDK, set_ext, limits)
        except Exception as exc:
            perm = []
            print(f"(could not list collision pairs: {exc})")
        if perm:
            print("Collision pairs that collide at EVERY gantry corner (so they are not caused by the gantry position):")
            for pair in perm:
                print("  " + pair_name(pair))
            if AUTO_DISABLE_PERMANENT_PAIRS:
                for it1, it2, i1, i2 in perm:
                    RDK.setCollisionActivePair(robolink.COLLISION_OFF, it1, it2, i1, i2)
                print("  -> switched these pairs OFF in the collision map for this run "
                      "(Tools > Collision map > 'Set default selection' restores the defaults).\n")
        else:
            print("No always-colliding pairs found.\n")
        set_ext(original[n_arm:], original[:n_arm])

    # 3) every sample point
    results, used, hits = [], [], Counter()
    for name, pos_c, nrm in POINTS:
        T_abs = car_T @ look_pose(pos_c, nrm, STANDOFF)
        tip, zdir = T_abs[:3, 3], T_abs[:3, 2]
        verdict = ("--", None, None, "no candidate position worked", 0.0)
        reasons = {"travel": 0, "ik": 0, "col": 0}
        closest = None
        for lat, L, wanted in base_candidates(tip, zdir):
            ext = ext_for_base(p0, A, wanted)
            if not within(ext, limits):
                reasons["travel"] += 1
                if closest is None or excess(ext, limits) < closest[0]:
                    closest = (excess(ext, limits), ext)
                continue
            set_ext(ext)
            T_base = np.linalg.inv(pose_frame()) @ T_abs
            sol = robot.SolveIK(to_mat(T_base))
            if sol is None or len(sol.list()) < n_arm:
                reasons["ik"] += 1
                continue
            arm = [float(v) for v in sol.list()[:n_arm]]
            if CHECK_COLLISIONS:
                set_ext(ext, arm)
                if RDK.Collisions() > 0:
                    reasons["col"] += 1
                    try:
                        for pair in RDK.CollisionPairs():
                            hits[pair_name(pair)] += 1
                    except Exception:
                        pass
                    continue
            verdict = ("OK", L, ext, "", lat)
            break
        else:
            why = max(reasons, key=reasons.get)
            text = {"travel": "gantry travel exceeded for every candidate",
                    "ik": "arm could not reach from any candidate",
                    "col": "every reachable candidate collides"}[why]
            if why == "travel" and closest is not None:
                over = [f"d{7 + k} {v:.0f} (limit {lo:.0f}..{hi:.0f})" for k, (v, (lo, hi)) in enumerate(zip(closest[1], limits))
                        if v < lo - 1e-6 or v > hi + 1e-6]
                text += "; closest needs " + ", ".join(over)
            verdict = ("--", None, None, text, 0.0)
        results.append((name, verdict))
        if verdict[0] == "OK":
            used.append(verdict[2])

    set_ext(original[n_arm:], original[:n_arm])          # put everything back

    print(f"{'surface':14s}{'result':>7s}{'L':>6s}{'shift':>7s}   gantry d7, d8, d9 (mm)")
    print("-" * 70)
    for name, (status, L, ext, why, lat) in results:
        ext_txt = "" if ext is None else "  ".join(f"{v:7.0f}" for v in ext)
        print(f"{name:14s}{status:>7s}{'' if L is None else int(L):>6}{'' if L is None else int(lat):>7}   {ext_txt}   {why}")
    print("\nOK = arm reaches it collision-free (L = base-behind-gun distance, shift = sideways offset, mm); -- = not reachable")
    if hits:
        print("\nPairs most often colliding at otherwise reachable positions:")
        for name_, n_ in hits.most_common(8):
            print(f"  {n_:4d} x  {name_}")
    if used:
        u = np.array(used)
        print("\nGantry travel actually used by the OK points (mm):")
        for k, nm in enumerate(("d7", "d8", "d9")):
            print(f"  {nm}: {u[:, k].min():7.0f} .. {u[:, k].max():7.0f}   (limit {limits[k][0]:.0f} .. {limits[k][1]:.0f})")


if __name__ == "__main__":
    main()
