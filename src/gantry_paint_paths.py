"""
gantry_paint_paths.py - collision-CHECKED spray paths for the gantry + UR10 cell.

Why this exists: clicking Cartesian targets lets RoboDK pick the arm solution closest to wherever the arm happens
to be (that is how you got joints like theta2 = -262 deg) and it moves the gantry in a straight joint line, which
sweeps through the car. Here every pose is solved by ONE rule, and every move is verified before it is written:

  * gantry follows the gun: the arm's base sits at a fixed offset in the TOOL frame (L behind the gun, plus a
    small sideways shift) so the arm only changes orientation along a stroke;
  * the arm is solved with the previous solution as the seed (no sudden flips: jumps > 60 deg are rejected);
  * targets are JOINT targets (6 arm + 3 gantry values) and moves are MoveJ, so the path RoboDK drives is exactly
    the joint-space line this script samples for collisions (every ~3 deg / 40 mm), including the arm against
    itself and the gantry (whatever the collision map says);
  * zones are joined by transit moves: retract -> lift the gantry to its top -> fold the arm -> travel over the car
    -> lower onto the next approach. Each leg is checked and reported.

    python3 gantry_paint_paths.py --paint-stl car_paint_only_4800x1900x1600.stl                # all zones
    python3 gantry_paint_paths.py --paint-stl car_paint_only_4800x1900x1600.stl --zone left_door
    python3 gantry_paint_paths.py --paint-stl ... --dry                                         # check only, create nothing

Needs gantry_reach.py and paint_common.py in the same folder (alignment helpers; zone list, mesh loader, stroke planner).
Spray: calls the GunOn item with a colour per zone (GunOn.py). NOT yet tested against your RoboDK build; the logic is
tested offline on a toy model.
"""
import argparse
import types
import itertools
import math
from collections import Counter

import numpy as np

try:
    from robodk import robolink, robomath
except ImportError:
    import robolink
    import robomath

import gantry_reach as gr
import paint_common as gp

ROBOT_NAME, TOOL_NAME, CAR_KEY = gr.ROBOT_NAME, gr.TOOL_NAME, gr.CAR_KEY
PREFIX = "GP_"
FRAME_NAME = "GP_frame"
OFFSET_LS = (500.0, 700.0, 900.0)                                  # mm, base behind the gun (in the tool frame)
OFFSET_LATERAL = tuple((a, b) for a in (0, 300, -300) for b in (0, 300, 600, 900, 1200))
# mm, shift of the base in the tool X / Y (for a door the tool Y is UP: the carriage cannot go below its Z limit, so the
# base often has to sit several hundred mm above the gun)
APPROACH = 150.0            # mm outward from the surface for approach / retract
MAX_ARM_JUMP = 60.0         # deg between neighbouring points
SEG_STEP_DEG, SEG_STEP_MM = 3.0, 40.0
SPEED_LINEAR, SPEED_JOINT = 300.0, 30.0


# bent-elbow, non-singular arm poses (deg) used only to set up the IK check when the robot's own pose does not work
GENERIC_ARM_POSES = (
    (0.0, -90.0, -90.0, 0.0, 90.0, 0.0),
    (30.0, -100.0, -100.0, -50.0, 70.0, 20.0),
    (-30.0, -80.0, -110.0, -70.0, 80.0, -20.0),
    (60.0, -120.0, -80.0, -30.0, 100.0, 40.0),
    (97.2, -96.0, -60.4, 156.4, -112.8, 180.0),
)


def wrap_jump(a, b):
    return max(abs((x - y + 180.0) % 360.0 - 180.0) for x, y in zip(a, b))


class Cell:
    """Everything needed to solve and check poses of the synchronised arm + gantry."""

    def __init__(self, RDK, robot, tool, p0, A, limits, pose_frame, home_q, car_T, check_collisions=True):
        self.RDK, self.robot, self.tool = RDK, robot, tool
        self.p0, self.A, self.limits, self.pose_frame = p0, A, limits, pose_frame
        self.home_q, self.car_T, self.check = list(home_q), car_T, check_collisions
        self.calls = 0
        self.all_mode = None          # how SolveIK_All wants its pose: "tcp" or "flange" (found by calibrate_all)
        self.tool_inv = np.eye(4)

    def _all(self, T_f, mode):
        """Every arm solution (list of 6 joint values) for a pose, via SolveIK_All (a matrix, one solution per column)."""
        T = T_f if mode == "tcp" else T_f @ self.tool_inv
        try:
            M = self.robot.SolveIK_All(gr.to_mat(T))
            rows = M.rows
            if len(rows) < 6 or not rows[0]:
                return []
            return [[float(rows[r][c]) for r in range(6)] for c in range(len(rows[0]))]
        except Exception:
            return []

    @staticmethod
    def distinct(sols, seed_arm):
        """SolveIK_All lists every +-360 deg variant of each arm configuration (47 'solutions' for about 8 real ones).
        Keep one per configuration: the variant whose joint values are closest to the seed, so there is no 360 deg jump."""
        groups = {}
        for a in sols:
            key = tuple(round(((v + 180.0) % 360.0) - 180.0, 1) for v in a)
            raw = max(abs(x - y) for x, y in zip(a, seed_arm))
            if key not in groups or raw < groups[key][0]:
                groups[key] = (raw, a)
        out = [a for _, a in groups.values()]
        out.sort(key=lambda a: wrap_jump(a, seed_arm))
        return out

    def calibrate_all(self, arm_home, ext_home, tool_np):
        """Does SolveIK_All return the arm joints we are in, for the gun's current pose? (tool pose or flange pose?)"""
        self.tool_inv = np.linalg.inv(tool_np)
        self.set_q(list(arm_home) + list(ext_home))
        T_f = np.linalg.inv(self.pose_frame()) @ gr.to_np(self.tool.PoseAbs())
        for mode in ("tcp", "flange"):
            sols = self._all(T_f, mode)
            if sols and min(wrap_jump(a, arm_home) for a in sols) < 3.0:
                self.all_mode = mode
                return mode, len(self.distinct(sols, arm_home))
        self.all_mode = None
        return None, 0

    def pick_arm(self, T_f, ext, seed_arm):
        """First COLLISION-FREE arm solution, nearest to the seed. Uses every IK branch when SolveIK_All works
        (the arm often has an elbow-up branch that hits the gantry's column and an elbow-down one that does not)."""
        sols = self._all(T_f, self.all_mode) if self.all_mode else []
        if sols:
            sols = self.distinct(sols, seed_arm)
            for a in sols:
                q = a + [float(v) for v in ext]
                if not self.collides(q):
                    return q, ""
            return None, "col"
        try:
            sol = self.robot.SolveIK(gr.to_mat(T_f), joints_approx=list(seed_arm) + list(ext))
        except Exception:
            sol = self.robot.SolveIK(gr.to_mat(T_f))
        if sol is None or len(sol.list()) < 6:
            return None, "ik"
        q = [float(v) for v in sol.list()[:6]] + [float(v) for v in ext]
        return (None, "col") if self.collides(q) else (q, "")

    def set_q(self, q):
        self.robot.setJoints([float(v) for v in q])

    def collides(self, q):
        if not self.check:
            return False
        self.set_q(q)
        self.calls += 1
        return self.RDK.Collisions() > 0

    def solve(self, T_abs, off, seed_q):
        """Joints (6 arm + 3 gantry) that put the gun at T_abs with the base at offset `off` = (L, a, b) in the tool frame."""
        L, a, b = off
        tip, x, y, z = T_abs[:3, 3], T_abs[:3, 0], T_abs[:3, 1], T_abs[:3, 2]
        ext = gr.ext_for_base(self.p0, self.A, tip - L * z + a * x + b * y)
        if not gr.within(ext, self.limits):
            return None, "travel"
        self.set_q(list(seed_q[:6]) + list(ext))
        T_f = np.linalg.inv(self.pose_frame()) @ T_abs
        return self.pick_arm(T_f, ext, seed_q[:6])

    def segment_collides(self, q0, q1):
        """True if the straight JOINT-space move q0 -> q1 (what MoveJ does) hits something."""
        d_arm = max(abs(a - b) for a, b in zip(q0[:6], q1[:6]))
        d_ext = max(abs(a - b) for a, b in zip(q0[6:], q1[6:]))
        n = max(1, int(math.ceil(max(d_arm / SEG_STEP_DEG, d_ext / SEG_STEP_MM))))
        for k in range(1, n):
            t = k / n
            if self.collides([a + (b - a) * t for a, b in zip(q0, q1)]):
                return True
        return False


def sample_points(rows, limit=12):
    pts = []
    for row in rows:
        if row:
            pts += [row[0], row[len(row) // 2], row[-1]]
    return pts[:limit] if len(pts) <= limit else pts[:: max(1, len(pts) // limit)][:limit]


def choose_offset(cell, pts):
    """Offset (L, a, b) that works for the most sample points; ties -> smaller sideways shift."""
    best = None
    for L in OFFSET_LS:
        for a, b in OFFSET_LATERAL:
            seed, ok = cell.home_q, 0
            for p in pts:
                q, _ = cell.solve(p["T_abs"], (L, a, b), seed)
                if q is not None:
                    ok += 1
                    seed = q
            key = (ok, -(abs(a) + abs(b)), L == 700.0)
            if best is None or key > best[0]:
                best = (key, (L, float(a), float(b)))
    return best[1], best[0][0]


def process_zone(cell, mesh, zone, car_T):
    rows = gp.plan_zone(mesh, zone)
    for row in rows:
        for p in row:
            p["T_abs"] = car_T @ p["pose"]
    pts = sample_points(rows)
    st = Counter()
    if not pts:
        return None, st, None
    off, hits = choose_offset(cell, pts)
    seed, prev_q, rows_ok = cell.home_q, None, []
    for row in rows:
        good = []
        for p in row:
            st["total"] += 1
            q, why = cell.solve(p["T_abs"], off, seed)
            if q is None:
                st[why] += 1
                continue
            if prev_q is not None and wrap_jump(q[:6], prev_q[:6]) > MAX_ARM_JUMP:
                st["flip"] += 1
                continue
            p["q"] = q
            good.append(p)
            seed = prev_q = q
            st["ok"] += 1
        rows_ok.append(good)
    # moves between consecutive points (and row to row) must be collision-free too
    prev = None
    for row in rows_ok:
        for p in row:
            if prev is not None and cell.segment_collides(prev["q"], p["q"]):
                st["segment_collision"] += 1
                p["bad_segment"] = True
            prev = p
    return rows_ok, st, off


def approach_pose(T_abs, dist=APPROACH):
    T = T_abs.copy()
    T[:3, 3] = T_abs[:3, 3] - T_abs[:3, 2] * dist       # tool Z points into the surface, so outward is -Z
    return T


def find_safe(cell, car_T):
    """Gantry at its top above the car's middle, arm folded with the gun horizontal: collision-free? -> joint vector."""
    centre = car_T[:3, 3] + np.array([0.0, 0.0, 0.0])
    target_base = np.array([centre[0], centre[1], cell.p0[2]])
    ext = gr.ext_for_base(cell.p0, cell.A, target_base)
    ext[2] = 0.0
    if not gr.within(ext, cell.limits):
        return None
    base_pos = cell.p0 + cell.A @ np.asarray(ext)
    for height, direction in itertools.product((-100.0, -300.0), ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0))):
        z = gr.unit(direction)
        x = np.array([0.0, 0.0, -1.0])
        T = np.eye(4)
        T[:3, 0], T[:3, 1], T[:3, 2] = x, np.cross(z, x), z
        T[:3, 3] = base_pos + 650.0 * z + np.array([0.0, 0.0, height])
        cell.set_q(list(cell.home_q[:6]) + list(ext))
        q, _ = cell.pick_arm(np.linalg.inv(cell.pose_frame()) @ T, ext, cell.home_q[:6])
        if q is not None:
            return q
    return None


def with_ext(q, ext_top):
    return list(q[:6]) + list(ext_top)


def top_of(cell, q):
    e = list(q[6:])
    e[2] = 0.0
    return e


def selftest(cell, car_T):
    """Does RoboDK actually report arm/gantry-versus-car collisions? Put the arm's base INSIDE the car and ask."""
    print("\nCollision self-test (the arm's base is placed inside the car; RoboDK must report a collision):")
    ok_any = False
    for label, local in (("inside the cabin", (0.0, 0.0, 1300.0)), ("inside the car's left side", (0.0, 700.0, 1300.0)),
                         ("inside the hood", (1500.0, 0.0, 1300.0))):
        wanted = (car_T @ np.append(np.asarray(local), 1.0))[:3]
        ext = gr.ext_for_base(cell.p0, cell.A, wanted)
        if not gr.within(ext, cell.limits):
            print(f"  base {label:28s}: gantry cannot go there (travel limits), skipped")
            continue
        cell.set_q(list(cell.home_q[:6]) + list(ext))
        n = cell.RDK.Collisions()
        pairs = []
        if n:
            try:
                pairs = [gr.pair_name(p) for p in cell.RDK.CollisionPairs()][:3]
            except Exception:
                pass
        ok_any |= n > 0
        print(f"  base {label:28s}: " + (f"{n} colliding pair(s), e.g. {'; '.join(pairs)}" if n else "NO COLLISION REPORTED  <-- detection is not working for this"))
    if not ok_any:
        print("  => RoboDK did not report the car at all. Check that Tools > Check collisions is ON and that the full car "
              "('car_xr_sedan...') is checked against the UR10 links, the gantry parts and the gun in Tools > Collision map "
              "(use 'Set default selection', then switch the paint-only skin's pairs OFF).")
    else:
        print("  => collision detection against the car works.")


def setup_cell(RDK, standoff_arg=None, no_align=False):
    """Find robot, tool and car; align the car; measure the gantry map; find the IK frame; build the Cell (collision checks on).
    Shared by this generator and by build_zones.py. Returns a namespace with everything the callers need."""
    robot = RDK.Item(ROBOT_NAME, robolink.ITEM_TYPE_ROBOT)
    tool = RDK.Item(TOOL_NAME, robolink.ITEM_TYPE_TOOL)
    car = None
    for it in RDK.ItemList(robolink.ITEM_TYPE_OBJECT):
        n = gr.norm(it.Name())
        if CAR_KEY in n and "paint_only" not in n and "collision" not in n:
            car = it
    if not robot.Valid() or not tool.Valid() or car is None:
        raise SystemExit("Could not find the robot, tool or car object.")
    base = robot.Parent()
    robot.setPoseTool(tool)
    try:                                   # neutral active reference = the robot's base frame ITEM (earlier builds left 'Car Frame' active).
        robot.setPoseFrame(base)           # Never pass a matrix here: that would MOVE the active frame item.
    except Exception:
        pass
    gp.STANDOFF = gr.standoff_for(tool, gr.to_np(robot.PoseTool()), standoff_arg, gp.STANDOFF)
    print(f"Standoff: {gp.STANDOFF:.0f} mm (nozzle to surface).")

    car_T = gr.to_np(car.PoseAbs())
    if not no_align:
        M, _, info = gr.car_alignment(car)
        if M is None:
            print(f"WARNING: car alignment failed ({info}); using the object's pose.")
        else:
            car_T = car_T @ M
            print(f"Car aligned (geometry turned {info['theta']} deg, footprint centre {np.round(info['centre']).tolist()} in its object).")

    original = robot.Joints().list()
    n_arm = 6
    arm_home, ext_home = list(original[:n_arm]), list(original[n_arm:])
    limits = gr.read_limits(robot, n_arm, 3)

    def set_ext(ext, arm=None):
        robot.setJoints((list(arm) if arm is not None else arm_home) + [float(v) for v in ext])

    pos = []
    for e in ((0, 0, 0), (1000, 0, 0), (0, 1000, 0), (0, 0, 1000)):
        set_ext(e)
        pos.append(gr.to_np(base.PoseAbs())[:3, 3])
    p0, A = gr.gantry_map(pos)

    set_ext(ext_home)
    pose_frame, pose_name, report = gr.find_pose_frame(robot, tool, arm_home, n_arm)
    if pose_frame is None:
        # The pose the robot happens to be in can be awkward for the IK check (a singular or wrist-flipped pose). Try generic,
        # bent, non-singular arm poses instead; the check only needs the solver to reproduce the pose it is given.
        print("IK self-test failed from the current pose:\n" + "\n".join(report) + "\nTrying generic bent poses ...")
        for seed in GENERIC_ARM_POSES:
            arm_home = list(seed)
            robot.setJoints(arm_home + [float(v) for v in ext_home])
            pose_frame, pose_name, rep = gr.find_pose_frame(robot, tool, arm_home, n_arm)
            if pose_frame is not None:
                print(f"IK self-test passed from the generic pose {seed}.")
                break
    if pose_frame is None:
        raise SystemExit("IK self-test failed for every frame and every generic pose I tried:\n" + "\n".join(report) + "\nPaste this message to me.")
    print(f"SolveIK measures poses from '{pose_name}'.")

    RDK.setCollisionActive(robolink.COLLISION_ON)
    cell = Cell(RDK, robot, tool, p0, A, limits, pose_frame, arm_home + ext_home, car_T)
    mode, n_branches = cell.calibrate_all(arm_home, ext_home, gr.to_np(robot.PoseTool()))
    print("IK branches: " + (f"SolveIK_All works ({n_branches} distinct arm configurations at the current pose, pose given as {mode}); the "
                             "generator picks the first collision-free branch." if mode else
                             "SolveIK_All did not reproduce the current joints - using the single closest solution only."))
    try:
        perm = gr.find_permanent_pairs(RDK, set_ext, limits)
        for it1, it2, i1, i2 in perm:
            RDK.setCollisionActivePair(robolink.COLLISION_OFF, it1, it2, i1, i2)
        if perm:
            print("Switched OFF always-colliding pairs: " + "; ".join(gr.pair_name(p) for p in perm))
    except Exception as exc:
        print(f"(could not check always-colliding pairs: {exc})")

    return types.SimpleNamespace(RDK=RDK, robot=robot, tool=tool, car=car, base=base, car_T=car_T, original=original, n_arm=n_arm,
                                 arm_home=arm_home, ext_home=ext_home, limits=limits, set_ext=set_ext, p0=p0, A=A,
                                 pose_frame=pose_frame, cell=cell)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--paint-stl", default="car_paint_only_4800x1900x1600.stl")
    ap.add_argument("--zone")
    ap.add_argument("--dry", action="store_true", help="solve and check everything but create no targets or programs")
    ap.add_argument("--no-align", action="store_true")
    ap.add_argument("--selftest", action="store_true", help="check that RoboDK reports collisions with the car, then exit")
    ap.add_argument("--standoff", help="nozzle-to-surface distance in mm (default 250), or 'auto' (= how far the tool's own geometry, the spray cone included, reaches in front of the TCP, plus 40 mm)")
    args = ap.parse_args()

    RDK = robolink.Robolink()
    ctx = setup_cell(RDK, args.standoff, args.no_align)
    robot, tool, car, base, car_T = ctx.robot, ctx.tool, ctx.car, ctx.base, ctx.car_T
    original, n_arm, arm_home, ext_home = ctx.original, ctx.n_arm, ctx.arm_home, ctx.ext_home
    limits, set_ext, p0, A, pose_frame, cell = ctx.limits, ctx.set_ext, ctx.p0, ctx.A, ctx.pose_frame, ctx.cell

    if args.selftest:
        selftest(cell, car_T)
        set_ext(ext_home, arm_home)
        return

    mesh = gp.load_mesh(args.paint_stl)
    zones = [z for z in gp.ZONES if not args.zone or z["name"] == args.zone]
    safe_q = find_safe(cell, car_T)
    print("Safe (folded, gantry at top) pose:", "found" if safe_q else "NOT found - transit moves cannot be built", "\n")

    print(f"{'zone':14s}{'L':>5s}{'a':>5s}{'b':>5s}{'points':>7s}{'ok':>5s}{'ik':>4s}{'trav':>5s}{'col':>5s}{'flip':>5s}{'seg':>5s}{'cover':>7s}")
    plans = []
    for zone in zones:
        rows_ok, st, off = process_zone(cell, mesh, zone, car_T)
        if rows_ok is None:
            print(f"{zone['name']:14s}  (no surface hits)")
            continue
        total = max(st["total"], 1)
        print(f"{zone['name']:14s}{off[0]:5.0f}{off[1]:5.0f}{off[2]:5.0f}{st['total']:7d}{st['ok']:5d}{st['ik']:4d}{st['travel']:5d}"
              f"{st['col']:5d}{st['flip']:5d}{st['segment_collision']:5d}{100.0 * st['ok'] / total:6.0f}%")
        plans.append((zone, rows_ok, off, st))

    # transit legs between zones (checked, reported)
    def zone_ends(rows_ok, off):
        pts = [p for r in rows_ok for p in r]
        if len(pts) < 2:
            return None
        first, last = pts[0], pts[-1]
        qa, _ = cell.solve(approach_pose(first["T_abs"]), off, first["q"])
        qr, _ = cell.solve(approach_pose(last["T_abs"]), off, last["q"])
        return (qa or first["q"]), (qr or last["q"]), pts

    ends = []
    for zone, rows_ok, off, st in plans:
        e = zone_ends(rows_ok, off)
        if e:
            ends.append((zone, rows_ok, off, e))
    if safe_q and len(ends) >= 1:
        start_ok = not cell.segment_collides(cell.home_q, safe_q)
        print("\nStart leg (the pose the robot is in NOW -> the safe folded pose, the first move of GP_Main): "
              + ("OK" if start_ok else "COLLIDES - put the robot in a clear pose first (the Home button), then run this script again"))
        print("Transit legs between zones (retract -> lift -> fold -> over the car -> lower -> approach):")
        for k in range(len(ends) - 1):
            zi, zj = ends[k][0]["name"], ends[k + 1][0]["name"]
            r_i, a_j = ends[k][3][1], ends[k + 1][3][0]
            legs = [("lift", with_ext(r_i, top_of(cell, r_i))),
                    ("fold", with_ext(safe_q, top_of(cell, r_i))),
                    ("travel", with_ext(safe_q, top_of(cell, a_j))),
                    ("lower", a_j)]
            q_prev, bad = r_i, []
            for name, q in legs:
                if cell.segment_collides(q_prev, q):
                    bad.append(name)
                q_prev = q
            print(f"  {zi:>13s} -> {zj:<13s} {'OK' if not bad else 'COLLIDES in: ' + ', '.join(bad)}")

    if args.dry:
        print("\n--dry: nothing created.")
        set_ext(ext_home, arm_home)
        return

    # ---- build the RoboDK items
    for prog in RDK.ItemList(robolink.ITEM_TYPE_PROGRAM):
        if prog.Name().startswith(PREFIX):
            prog.Delete()
    old = RDK.Item(FRAME_NAME, robolink.ITEM_TYPE_FRAME)
    if old.Valid():
        old.Delete()
    frame = RDK.AddFrame(FRAME_NAME)
    gun = RDK.Item(gp.GUN_ITEM)

    def jt(name, q):
        t = RDK.AddTarget(name, frame, robot)
        t.setAsJointTarget()
        t.setJoints([float(v) for v in q])
        return t

    zone_progs = []
    for zone, rows_ok, off, (qa, qr, pts) in ends:
        prog = RDK.AddProgram(PREFIX + zone["name"], robot)
        prog.setSpeed(SPEED_LINEAR, SPEED_JOINT)
        rgb = gp.ZONE_COLORS.get(zone["name"], (0.85, 0.1, 0.1))
        prog.MoveJ(jt(f"{zone['name']}_approach", qa))
        if gun.Valid():
            prog.RunInstruction(f"GunOn(1,{rgb[0]:.2f},{rgb[1]:.2f},{rgb[2]:.2f},{gp.STANDOFF:.0f})", robolink.INSTRUCTION_CALL_PROGRAM)
        for r, row in enumerate(rows_ok):
            for i, p in enumerate(row):
                prog.MoveJ(jt(f"{zone['name']}_r{r}_p{i}", p["q"]))
        if gun.Valid():
            prog.RunInstruction("GunOn(0)", robolink.INSTRUCTION_CALL_PROGRAM)
        prog.MoveJ(jt(f"{zone['name']}_retract", qr))
        zone_progs.append((zone, prog, qa, qr))

    if safe_q and zone_progs:
        main_prog = RDK.AddProgram(PREFIX + "Main", robot)
        main_prog.setSpeed(SPEED_LINEAR, SPEED_JOINT)
        if gun.Valid():
            main_prog.RunInstruction("GunOn(-1)", robolink.INSTRUCTION_CALL_PROGRAM)
        main_prog.MoveJ(jt("safe_start", safe_q))
        for k, (zone, prog, qa, qr) in enumerate(zone_progs):
            main_prog.MoveJ(jt(f"{zone['name']}_over", with_ext(safe_q, top_of(cell, qa))))
            main_prog.RunInstruction(prog.Name(), robolink.INSTRUCTION_CALL_PROGRAM)
            main_prog.MoveJ(jt(f"{zone['name']}_lift", with_ext(qr, top_of(cell, qr))))
            main_prog.MoveJ(jt(f"{zone['name']}_fold", with_ext(safe_q, top_of(cell, qr))))
        if gun.Valid():
            main_prog.RunInstruction("GunOn(2)", robolink.INSTRUCTION_CALL_PROGRAM)
    set_ext(ext_home, arm_home)
    made_main = bool(safe_q and zone_progs)
    print(f"\nCreated {len(zone_progs)} zone program(s) ({PREFIX}<zone>)" + (f" and {PREFIX}Main." if made_main else
          ". No main program (no zone worked, or no safe folded pose was found)."))
    print("Play GP_Main (or a single zone) and watch it; then right-click > 'Check path and Collisions' (Shift+F5).")


if __name__ == "__main__":
    main()
