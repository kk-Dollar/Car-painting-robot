"""
build_zones.py - build one RoboDK Curve Follow project per CAD zone, from zone_curves.json, cloned from your working template.

For every zone it:
  1. finds a START POSE: spray poses are sampled over the zone; my gantry search (gantry_paint_paths.py) picks where the arm
     base should stand so the most poses are reachable, and solves the first pose -> 9 joints (6 arm + 3 gantry);
  2. creates the curves (points + normals, car frame) as an object under "Car Frame";
  3. creates a project with the template's settings applied BEFORE the first solve (this is what made the solve succeed),
     with the zone's start joints, the path-to-tool offset (180-degree flip + standoff) and the spray events (GunOn);
  4. pass 1: Avoid Collisions OFF (fast), then verifies with Update(collisions ON);
     pass 2, only if pass 1 fails: Avoid Collisions ON with a +/-60 deg spin search, verifies again;
  5. (--main) adds a program Auto_Main that calls the zone programs in order.

    python3 build_zones.py --dry                      # only the start poses (no projects created) - a safe first look
    python3 build_zones.py --zones 3 8                # build only zones 3 and 8 (ids from zone_curves.json)
    python3 build_zones.py --main --skip 0 8          # zones 0 and 8 are already good: reuse them, plan the rest, then Auto_Main
    python3 build_zones.py --zones 0 --salvage        # a zone that fails to attach: find the curves that fail and drop them
    python3 build_zones.py --zones 8                  # defaults: standoff 150 mm, offset Z +150, approach -50, speed 150 mm/s
    python3 build_zones.py --zones 8 --spray-on SprayOn --spray-off SprayOff   # use RoboDK's own spray scripts by name
    python3 build_zones.py --zones 8 --offset-z 250   # standoff along the PATH frame Z (+ = out of the car if the path Z points out)

Not yet tested against your RoboDK (only against a toy cell): run --dry first, then ONE zone, and paste the output.
Zones are never merged with your Path2 project; items it creates are named "Auto ..." and are replaced on every run.
"""
import argparse
import json
import os
import re
import sys
import time

import numpy as np

try:
    from robodk import robolink
    from robodk.robomath import Mat
except ImportError:
    import robolink
    from robomath import Mat

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gantry_paint_paths as gpp  # noqa: E402
import paint_common as pcm  # noqa: E402

COLORS = [(0.85, 0.10, 0.10), (0.10, 0.45, 0.85), (0.10, 0.70, 0.30), (0.90, 0.70, 0.10), (0.60, 0.20, 0.80),
          (0.95, 0.45, 0.10), (0.10, 0.75, 0.75), (0.80, 0.30, 0.55), (0.45, 0.45, 0.45), (0.55, 0.75, 0.15)]


def unit(v):
    v = np.asarray(v, float)
    return v / max(np.linalg.norm(v), 1e-12)


def path_len(P):
    return float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum()) if len(P) > 1 else 0.0


def spray_pose(P, N, d, standoff):
    """Gun pose in the car frame for a path point: tool Z = N (INTO the car), X along the stroke, TCP `standoff` OUT of the skin."""
    z = unit(N)
    x = np.asarray(d, float) - z * np.dot(d, z)
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(z, [0.0, 0.0, 1.0] if abs(z[2]) < 0.9 else [1.0, 0.0, 0.0])
    x = unit(x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = x, np.cross(z, x), z
    T[:3, 3] = np.asarray(P, float) - z * standoff
    return T


def zone_poses(zone, standoff):
    """All spray poses of a zone (car frame), curve by curve."""
    out = []
    for c in zone["curves"]:
        P, N = np.array(c["points"]), np.array(c["normals"])
        for i in range(len(P)):
            j = min(i + 1, len(P) - 1)
            d = P[j] - P[max(i - 1, 0)] if len(P) > 1 else np.array([1.0, 0, 0])
            out.append(spray_pose(P[i], N[i], d, standoff))
    return out


def is_wound(q):
    """Elbow (J3) wound beyond +-190 deg: a different arm 'family' from the usual -5..-135 deg."""
    return q is not None and abs(q[2]) > 190.0


def orientation_class(zone):
    """Coarse class of a zone by the way the gun must point: top / front / rear / left / right. Used to reuse a pose that worked."""
    nx, ny, nz = zone["mean_normal"]
    if nz >= 0.6:
        return "top"
    if nx >= 0.6:
        return "front"
    if nx <= -0.6:
        return "rear"
    return "left" if ny > 0 else "right"


def start_for_zone(cell, car_T, zone, standoff, n_samples=12, Ls=None, seed=None):
    """(joints9, offset, hits, n, k) - where to start: the arm-base offset that works for most sample poses, solved at the first pose.
    Ls restricts the base-behind-the-gun distances tried (default: the generator's 500/700/900 mm)."""
    poses = zone_poses(zone, standoff)
    idx = np.unique(np.linspace(0, len(poses) - 1, min(n_samples, len(poses))).astype(int))
    pts = [{"T_abs": car_T @ poses[i]} for i in idx]
    saved, saved_seed = gpp.OFFSET_LS, list(cell.home_q)
    try:
        if Ls:
            gpp.OFFSET_LS = tuple(Ls)
        if seed is not None:                      # the IK picks the arm configuration nearest to this pose
            cell.home_q = [float(v) for v in seed]
        off, hits = gpp.choose_offset(cell, pts)
        order = [0] + list(range(1, len(poses), max(1, len(poses) // 25)))        # the first pose, then others as a fallback
        for k in order:
            q, _ = cell.solve(car_T @ poses[k], off, cell.home_q)
            if q is not None:
                return [float(v) for v in q], off, hits, len(pts), k
        return None, off, hits, len(pts), None
    finally:
        gpp.OFFSET_LS = saved
        cell.home_q = saved_seed


def read_template(RDK, name):
    tmpl = RDK.Item(name, robolink.ITEM_TYPE_MACHINING)
    if not tmpl.Valid():
        raise SystemExit(f'Template project "{name}" not found.')
    def get(k):                      # "Machining", "OptimAxes", "ProgEvents": no value -> returns the current settings
        v = tmpl.setParam(k)
        return json.loads(v) if isinstance(v, str) else v
    link = lambda t: tmpl.getLink(t)                                                                      # noqa: E731
    return {"item": tmpl, "robot": link(robolink.ITEM_TYPE_ROBOT), "tool": link(robolink.ITEM_TYPE_TOOL),
            "frame": link(robolink.ITEM_TYPE_FRAME), "machining": get("Machining"), "optim": get("OptimAxes"),
            "events": get("ProgEvents"), "pose": np.array([list(r) for r in tmpl.Pose().rows], float)}


def delete_old(RDK, name_proj, name_obj, name_prog):
    for nm, t in ((name_proj, robolink.ITEM_TYPE_MACHINING), (name_obj, robolink.ITEM_TYPE_OBJECT), (name_prog, robolink.ITEM_TYPE_PROGRAM)):
        it = RDK.Item(nm, t)
        if it.Valid():
            it.Delete()


def make_curve_object(RDK, car_frame, zone, name):
    """Curves (points + normals, car frame) as an object under the car frame, or None if RoboDK refuses them."""
    mats = [Mat(np.c_[np.array(c["points"]), np.array(c["normals"])].tolist()).tr() for c in zone["curves"]]
    obj = None
    for candidate in ([mats] if len(mats) > 1 else [mats[0], mats]):         # a single curve: plain matrix first, then a list
        try:
            obj = RDK.AddCurve(candidate, projection_type=robolink.PROJECTION_NONE)
            if obj is not None and obj.Valid():
                break
        except Exception as exc:
            print(f"      (AddCurve failed: {exc})")
            obj = None
    if obj is None or not obj.Valid():
        return None
    obj.setName(name)
    obj.setParent(car_frame)            # curves are expressed in the car frame (like the SPG object under "Frame 4")
    return obj


def clean_msg(text):
    return re.sub(r"\s*\|\s*\|\s*", " | ", re.sub(r"<br\s*/?>", " | ", re.sub(r"</?strong>", "", str(text)))).strip(" |")


def collision_report(RDK, robot, prog, message):
    """RoboDK says 'Collision detected ... Instruction N'. Put the robot at that instruction and ask WHICH items collide."""
    m = re.search(r"Instruction\s+(\d+)", str(message))
    if not m:
        return None
    n = int(m.group(1))
    for k in (n, n - 1, n + 1, n - 2):
        try:
            joints = prog.Instruction(k)[5]
            robot.setJoints(joints)
            if RDK.Collisions() > 0:
                names = []
                for pair in RDK.CollisionPairs():
                    try:
                        names.append(gpp.gr.pair_name(pair))
                    except Exception:
                        names.append(str(pair))
                jl = list(joints.list() if hasattr(joints, "list") else joints)
                return {"instruction": k, "pairs": names[:6], "joints": [round(float(v), 1) for v in jl]}
        except Exception:
            continue
    return None


def measure_standoff(robot, frame, prog, zone_points, n_moves=14):
    """Best effort: smallest distance from the TCP to the zone's own path points over the first moves of the program.
    At the path points this is the standoff (~250 mm); approach/retract points are further away. Unlike a ray test against the
    whole car skin it cannot be fooled by a neighbouring panel."""
    try:
        robot.setPoseFrame(frame)
        best = None
        for i in range(min(prog.InstructionCount(), 60)):
            name, itype, mtype, isjoint, pose, joints = prog.Instruction(i)
            if not joints or len(list(joints.list() if hasattr(joints, "list") else joints)) < 6:
                continue
            robot.setJoints(joints)
            T = np.array([list(r) for r in robot.Pose().rows], float)
            d = float(np.linalg.norm(zone_points - T[:3, 3], axis=1).min())
            best = d if best is None else min(best, d)
            n_moves -= 1
            if n_moves <= 0:
                break
        return best
    except Exception:
        return None


def program_ends(RDK, name):
    """(joints at the first move, joints at the last move) of a program, 9 values each (6 arm + 3 gantry), or (None, None)."""
    prog = RDK.Item(name, robolink.ITEM_TYPE_PROGRAM)
    if not prog.Valid():
        return None, None
    try:
        n = prog.InstructionCount()
    except Exception:
        return None, None

    def joints_at(i):
        try:
            j = prog.Instruction(i)[5]
            lst = [float(v) for v in (j.list() if hasattr(j, "list") else j)]
            return lst if len(lst) >= 9 else None
        except Exception:
            return None
    first = next((j for j in (joints_at(i) for i in range(n)) if j), None)
    last = next((j for j in (joints_at(i) for i in range(n - 1, -1, -1)) if j), None)
    return first, last


def hop_cost(end, start, mm_per_deg=5.0):
    """Cost of going from where one program ends to where the next begins: gantry distance (mm) + arm swing (deg x 5 mm)."""
    e, s_ = np.array(end, float), np.array(start, float)
    return float(np.linalg.norm(e[6:9] - s_[6:9]) + mm_per_deg * np.abs(e[:6] - s_[:6]).sum())


def auto_order(RDK, names):
    """Visiting order of the programs that minimises the total hop cost (exhaustive for up to 9 programs, greedy beyond)."""
    import itertools
    ends = {nm: program_ends(RDK, nm) for nm in names}
    ok = [nm for nm in names if ends[nm][0] is not None and ends[nm][1] is not None]
    bad = [nm for nm in names if nm not in ok]
    if len(ok) < 3:
        return names, None, None
    cost = lambda a, b: hop_cost(ends[a][1], ends[b][0])      # noqa: E731
    total = lambda seq: sum(cost(x, y) for x, y in zip(seq, seq[1:]))   # noqa: E731
    if len(ok) <= 9:
        best = min(itertools.permutations(ok), key=total)
    else:
        best, left = [ok[0]], set(ok[1:])
        while left:
            nxt = min(left, key=lambda b: cost(best[-1], b))
            best.append(nxt)
            left.remove(nxt)
    return list(best) + bad, total(best), total(ok)


def find_safe_all(cell, car_T):
    """Every collision-free safe pose: gantry at its top above the car's middle, arm folded with the gun horizontal, pointing in 4
    directions at 3 heights. (gantry_paint_paths.find_safe returns only the first one.)"""
    import itertools
    gr = gpp.gr
    centre = car_T[:3, 3]
    ext = gr.ext_for_base(cell.p0, cell.A, np.array([centre[0], centre[1], cell.p0[2]]))
    ext[2] = 0.0
    if not gr.within(ext, cell.limits):
        return []
    base_pos = cell.p0 + cell.A @ np.asarray(ext)
    out = []
    for height, direction in itertools.product((-100.0, -300.0, -500.0), ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0))):
        z = gr.unit(direction)
        x = np.array([0.0, 0.0, -1.0])
        T = np.eye(4)
        T[:3, 0], T[:3, 1], T[:3, 2] = x, np.cross(z, x), z
        T[:3, 3] = base_pos + 650.0 * z + np.array([0.0, 0.0, height])
        cell.set_q(list(cell.home_q[:6]) + list(ext))
        q, _ = cell.pick_arm(np.linalg.inv(cell.pose_frame()) @ T, ext, cell.home_q[:6])
        if q is not None and not any(np.allclose(q, o, atol=1.0) for o in out):
            out.append([float(v) for v in q])
    return out


def safe_near(cell, car_T, seed_arm, n=2):
    """Collision-free safe poses (gantry at the top above the car, arm folded, gun horizontal) whose arm configuration is the
    one nearest to seed_arm, so folding from / unfolding to that configuration needs little joint travel. Nearest first."""
    import itertools
    gr = gpp.gr
    centre = car_T[:3, 3]
    ext = gr.ext_for_base(cell.p0, cell.A, np.array([centre[0], centre[1], cell.p0[2]]))
    ext[2] = 0.0
    if not gr.within(ext, cell.limits):
        return []
    base_pos = cell.p0 + cell.A @ np.asarray(ext)
    found = []
    for height, direction in itertools.product((-100.0, -300.0, -500.0), ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0))):
        z = gr.unit(direction)
        x = np.array([0.0, 0.0, -1.0])
        T = np.eye(4)
        T[:3, 0], T[:3, 1], T[:3, 2] = x, np.cross(z, x), z
        T[:3, 3] = base_pos + 650.0 * z + np.array([0.0, 0.0, height])
        cell.set_q(list(seed_arm[:6]) + list(ext))
        q, _ = cell.pick_arm(np.linalg.inv(cell.pose_frame()) @ T, ext, list(seed_arm[:6]))
        if q is not None:
            found.append((float(np.abs(np.array(q[:6]) - np.array(seed_arm[:6])).sum()), [float(v) for v in q]))
    found.sort(key=lambda t: t[0])
    out = []
    for _, q in found:
        if not any(np.allclose(q, o, atol=1.0) for o in out):
            out.append(q)
        if len(out) >= n:
            break
    return out


def staging_exts(cell, car_T, qa, qb, d9=0.0, margin=2200.0, n=3):
    """Gantry positions (d7, d8, d9 = 0 at the top) well clear of the car, on its four sides, nearest to this hop first.
    The arm can be re-configured there without any risk of touching the car."""
    gr = gpp.gr
    c = car_T[:3, 3]
    ax, ay = car_T[:3, 0], car_T[:3, 1]
    cands = []
    for vec, half in ((ax, 2400.0), (-ax, 2400.0), (ay, 950.0), (-ay, 950.0)):
        pos = c + vec * (half + margin)
        pos[2] = cell.p0[2]
        ext = gr.ext_for_base(cell.p0, cell.A, pos)
        ext[2] = d9
        if gr.within(ext, cell.limits):
            cands.append([float(v) for v in ext])
    cost = lambda e: np.linalg.norm(np.array(e[:2]) - np.array(qa[6:8])) + np.linalg.norm(np.array(e[:2]) - np.array(qb[6:8]))   # noqa: E731
    return sorted(cands, key=cost)[:n]


def plan_variants(cell, safe_q, qa, qb, top):
    """Candidate ways from qa (where one program ends) to qb (where the next begins); each is (name, [(leg, joints), ...]).
    top(q) = the gantry position q with the travel height applied."""
    wx = gpp.with_ext
    v = [("lift-travel-unfold-lower", [("lift", wx(qa, top(qa))), ("travel", wx(qa, top(qb))), ("unfold", wx(qb, top(qb))), ("lower", qb)]),
         ("lift-unfold-travel-lower", [("lift", wx(qa, top(qa))), ("unfold", wx(qb, top(qa))), ("travel", wx(qb, top(qb))), ("lower", qb)])]
    if safe_q is not None:
        v = [("lift-fold-travel-lower", [("lift", wx(qa, top(qa))), ("fold", wx(safe_q, top(qa))), ("travel", wx(safe_q, top(qb))), ("lower", qb)]),
             ("lift-fold-travel-unfold-lower", [("lift", wx(qa, top(qa))), ("fold", wx(safe_q, top(qa))), ("travel", wx(safe_q, top(qb))),
                                                 ("unfold", wx(qb, top(qb))), ("lower", qb)]),
             ("liftfold-travel-lower", [("liftfold", wx(safe_q, top(qa))), ("travel", wx(safe_q, top(qb))), ("lower", qb)]),
             ("liftfold-travel-unfold-lower", [("liftfold", wx(safe_q, top(qa))), ("travel", wx(safe_q, top(qb))), ("unfold", wx(qb, top(qb))),
                                                ("lower", qb)])] + v
    return v


def plan_hop(cell, safes, qa, qb, car_T=None, heights=(500.0, 900.0, 0.0)):
    """How to get from joints qa to qb without a collision: a direct joint move, else the first collision-free transit.
    The transit travel height d9 (mm below the gantry's top) is varied: at d9 = 0 the folded arm sits right under the carriage and its Z column
    and tends to hit them, a little lower it has room. For every height: (0) fold near qa, drive to a staging position clear of the car,
    re-configure there, drive back, lower; (1) via safe poses nearest to each end; (2) the plain plan variants with the global safe poses.
    If nothing is clear: the transit with the fewest colliding legs, marked UNRESOLVED. Returns (plan name, legs, colliding leg names)."""
    wx = gpp.with_ext

    def check(legs):
        prev, bad = qa, []
        for ln, q in legs:
            if cell.segment_collides(prev, q):
                bad.append(ln)
            prev = q
        return bad
    if not check([("move", qb)]):
        return "direct", [("move", qb)], []
    tried = []

    def attempt(label, legs):
        bad = check(legs)
        if not bad:
            return label, legs, []
        tried.append((len(bad), label, legs, bad))
        return None

    sA = sB = []
    if car_T is not None:
        sA, sB = safe_near(cell, car_T, qa), safe_near(cell, car_T, qb)
    for h in heights:
        top = lambda q, h=h: list(q[6:8]) + [h]          # noqa: E731
        tag = f" @d9={h:.0f}"
        if sA and sB:
            for si, st in enumerate(staging_exts(cell, car_T, qa, qb, d9=h)):
                legs = [("lift", wx(qa, top(qa))), ("fold", wx(sA[0], top(qa))), ("away", wx(sA[0], st)), ("reconf_far", wx(sB[0], st)),
                        ("back", wx(sB[0], top(qb))), ("lower", qb)]
                r = attempt(f"fold-away-reconf-back-lower [staging {si + 1}]{tag}", legs)
                if r:
                    return r
            for ia, a_ in enumerate(sA):
                for ib, b_ in enumerate(sB):
                    legs = [("lift", wx(qa, top(qa))), ("fold", wx(a_, top(qa))), ("travel", wx(a_, top(qb))), ("reconf", wx(b_, top(qb))), ("lower", qb)]
                    r = attempt(f"lift-fold-travel-reconf-lower [near poses {ia + 1}/{ib + 1}]{tag}", legs)
                    if r:
                        return r
                    legs = [("liftfold", wx(a_, top(qa))), ("travel", wx(a_, top(qb))), ("reconf", wx(b_, top(qb))), ("lower", qb)]
                    r = attempt(f"liftfold-travel-reconf-lower [near poses {ia + 1}/{ib + 1}]{tag}", legs)
                    if r:
                        return r
        for si, safe_q in enumerate((safes[:2] if safes else [None])):
            for name, legs in plan_variants(cell, safe_q, qa, qb, top):
                r = attempt(name + (f" [safe pose {si + 1}]" if si > 0 else "") + tag, legs)
                if r:
                    return r
    n, name, legs, bad = min(tried, key=lambda t: t[0])
    return name + " (UNRESOLVED)", legs, bad


def build_main_with_transits(RDK, ctx, names, step):
    """Auto_Main = [GunOn(-1)] park, then for every program: checked transit, the program; finally back to the park pose."""
    cell, car_T, robot = ctx.cell, ctx.car_T, ctx.robot
    saved = (gpp.SEG_STEP_DEG, gpp.SEG_STEP_MM)
    gpp.SEG_STEP_DEG, gpp.SEG_STEP_MM = step
    try:
        safes = find_safe_all(cell, car_T)
        safe_q = safes[0] if safes else None
        print(f"Safe (folded) poses found: {len(safes)}" + ("" if safes else " - only the plans without a fold are possible"))
        for h in (500.0, 900.0, 0.0):                       # park a little below the very top: there the arm has room around the carriage
            park = next((q for q in ([sq[0], sq[1], sq[2], sq[3], sq[4], sq[5], sq[6], sq[7], h] for sq in safes) if not cell.collides(q)), None)
            if park is not None:
                safe_q = park
                print(f"Park pose: gantry {h:.0f} mm below its top, collision-free.")
                break
        ends = {nm: program_ends(RDK, nm) for nm in names}
        usable = [nm for nm in names if ends[nm][0] is not None and ends[nm][1] is not None]
        for nm in names:
            if nm not in usable:
                print(f"  (cannot read the joints of {nm}: left out of Auto_Main)")
        frame = RDK.Item("Car Frame", robolink.ITEM_TYPE_FRAME)
        old = RDK.Item("Auto_Main", robolink.ITEM_TYPE_PROGRAM)
        if old.Valid():
            old.Delete()
        for it_name in [n for n in RDK.ItemList(list_names=True) if n.startswith("Auto_Main_")]:
            it = RDK.Item(it_name)
            if it.Valid():
                it.Delete()
        main = RDK.AddProgram("Auto_Main", robot)
        main.setSpeed(gpp.SPEED_LINEAR, gpp.SPEED_JOINT)

        def jt(name, q):
            t = RDK.AddTarget(name, frame, robot)
            t.setAsJointTarget()
            t.setJoints([float(v) for v in q])
            return t

        main.RunInstruction("GunOn(-1)", robolink.INSTRUCTION_CALL_PROGRAM)
        rows, prev_end = [], None
        if safe_q is not None:
            start_bad = cell.segment_collides(cell.home_q, safe_q)
            print("Start leg (the pose the robot is in now -> the park pose): " + ("OK" if not start_bad else
                  "COLLIDES - start Auto_Main with the robot at the park pose (the script leaves it there when it finishes)"))
            main.MoveJ(jt("Main_T00_park", safe_q))             # the first move: from wherever the robot is to the park pose
            prev_end = list(safe_q)
        for i, nm in enumerate(usable):
            first, last = ends[nm]
            if prev_end is not None:
                t0 = time.time()
                plan, legs, bad = plan_hop(cell, safes, prev_end, first, car_T)
                rows.append((i, nm, plan, bad, time.time() - t0))
                print(f"  hop {i:2d} -> {nm:32s} {plan}{'  colliding legs: ' + ', '.join(bad) if bad else ''}   [{time.time() - t0:.0f} s]", flush=True)
                for ln, q in (legs if not plan.startswith("direct") or "UNRESOLVED" in plan else []):
                    main.MoveJ(jt(f"Main_T{i:02d}_{ln}", q))
            main.RunInstruction(nm, robolink.INSTRUCTION_CALL_PROGRAM)
            prev_end = list(last)
        if usable and safe_q is not None:                      # finish: lift, fold and park
            plan, legs, bad = plan_hop(cell, safes, prev_end, list(safe_q), car_T)
            rows.append((len(usable), "(park)", plan, bad, 0.0))
            print(f"  end    -> park pose                      {plan}{'  colliding legs: ' + ', '.join(bad) if bad else ''}")
            for ln, q in (legs if not plan.startswith("direct") or "UNRESOLVED" in plan else []):
                main.MoveJ(jt(f"Main_T{len(usable):02d}_{ln}", q))
        main.RunInstruction("GunOn(2)", robolink.INSTRUCTION_CALL_PROGRAM)
    finally:
        gpp.SEG_STEP_DEG, gpp.SEG_STEP_MM = saved
    n_bad = sum(1 for r in rows if r[3])
    print(f"\nAuto_Main built: {len(usable)} programs, {len(rows)} hops, {n_bad} hop(s) with a colliding leg (marked UNRESOLVED).")
    return usable, rows, safe_q


PARK = "__PARK__"
BIG = 1e7                      # cost added to a pair of programs whose hop could not be solved: the order search then avoids it


def best_cycle(nodes, cost):
    """Visiting order that starts and ends at PARK and minimises the total cost: nearest-neighbour start + 2-opt improvement."""
    others = [n for n in nodes if n != PARK]

    def total(order):
        return sum(cost(a, b) for a, b in zip(order, order[1:]))
    seq, left = [PARK], set(others)
    while left:
        nxt = min(left, key=lambda b: cost(seq[-1], b))
        seq.append(nxt)
        left.remove(nxt)
    seq.append(PARK)
    best, improved = total(seq), True
    while improved:
        improved = False
        for i in range(1, len(seq) - 2):
            for j in range(i + 1, len(seq) - 1):
                cand = seq[:i] + seq[i:j + 1][::-1] + seq[j + 1:]
                c = total(cand)
                if c < best - 1e-6:
                    seq, best, improved = cand, c, True
    return seq, best


def build_feasible_main(RDK, ctx, names, step, max_iter, here):
    """Order the programs so that EVERY hop between them can be solved, then build Auto_Main with the planned transits.
    Loop: best order by travel cost -> plan each new hop (results are cached in hop_cache.json) -> forbid the pairs that could not be solved
    -> search again, until all hops of the order are solved (or max_iter rounds)."""
    cell, car_T, robot = ctx.cell, ctx.car_T, ctx.robot
    saved = (gpp.SEG_STEP_DEG, gpp.SEG_STEP_MM)
    gpp.SEG_STEP_DEG, gpp.SEG_STEP_MM = step
    cache_path = os.path.join(here, "hop_cache.json")
    try:
        cache = json.load(open(cache_path))
    except Exception:
        cache = {}
    try:
        safes = find_safe_all(cell, car_T)
        safe_q = safes[0] if safes else None
        for h in (500.0, 900.0, 0.0):
            park = next((q for q in ([sq[0], sq[1], sq[2], sq[3], sq[4], sq[5], sq[6], sq[7], h] for sq in safes) if not cell.collides(q)), None)
            if park is not None:
                safe_q = park
                print(f"Safe poses: {len(safes)}. Park pose: gantry {h:.0f} mm below its top, collision-free.")
                break
        if safe_q is None:
            raise SystemExit("No collision-free park pose found; cannot plan transits.")
        ends = {nm: program_ends(RDK, nm) for nm in names}
        names = [nm for nm in names if ends[nm][0] is not None and ends[nm][1] is not None]
        ends[PARK] = (list(safe_q), list(safe_q))
        penalty = {}

        def cost(a, b):
            return hop_cost(ends[a][1], ends[b][0]) + penalty.get((a, b), 0.0)

        def hop(a, b, heights):
            key = f"{a}->{b}"
            sig = [round(v, 1) for v in ends[a][1]] + [round(v, 1) for v in ends[b][0]]
            e = cache.get(key)
            if e and e.get("sig") == sig and (e["ok"] or len(e.get("heights", [])) >= len(heights)):
                return e, False
            t0 = time.time()
            plan, legs, bad = plan_hop(cell, safes, ends[a][1], ends[b][0], car_T, heights)
            e = {"sig": sig, "plan": plan, "ok": not bad, "bad": bad, "heights": list(heights),
                 "legs": [[n, [float(v) for v in q]] for n, q in legs], "seconds": round(time.time() - t0)}
            cache[key] = e
            json.dump(cache, open(cache_path, "w"))
            return e, True

        quick, full = (500.0, 0.0), (500.0, 900.0, 0.0)
        best_order, best_bad = None, None
        for it in range(1, max_iter + 1):
            order, c = best_cycle(names + [PARK], cost)
            tested, bad_pairs = 0, []
            for x, y in zip(order, order[1:]):
                e, fresh = hop(x, y, quick)
                tested += int(fresh)
                if not e["ok"]:
                    bad_pairs.append((x, y))
                    penalty[(x, y)] = BIG
            print(f"  search round {it}: order cost {c / 1000:.1f}, {tested} new hops planned, {len(bad_pairs)} unsolved in this order", flush=True)
            for x, y in bad_pairs:
                print(f"      unsolved: {x if x != PARK else 'park'} -> {y if y != PARK else 'park'}  (now avoided)")
            if best_bad is None or len(bad_pairs) < best_bad:
                best_order, best_bad = order, len(bad_pairs)
            if not bad_pairs:
                break
        order = best_order
        # a last, thorough attempt (all travel heights) for any hop of the chosen order that is still unsolved
        for x, y in zip(order, order[1:]):
            e = cache[f"{x}->{y}"]
            if not e["ok"]:
                hop(x, y, full)
        frame = RDK.Item("Car Frame", robolink.ITEM_TYPE_FRAME)
        for old_name in [n for n in RDK.ItemList(list_names=True) if n == "Auto_Main" or n.startswith("Auto_Main_")]:
            it_ = RDK.Item(old_name)
            if it_.Valid():
                it_.Delete()
        main = RDK.AddProgram("Auto_Main", robot)
        main.setSpeed(gpp.SPEED_LINEAR, gpp.SPEED_JOINT)

        def jt(name, q):
            t = RDK.AddTarget(name, frame, robot)
            t.setAsJointTarget()
            t.setJoints([float(v) for v in q])
            return t
        main.RunInstruction("GunOn(-1)", robolink.INSTRUCTION_CALL_PROGRAM)
        main.MoveJ(jt("Main_T00_park", safe_q))
        n_bad, called = 0, []
        print("\nFinal order:")
        for i, (x, y) in enumerate(zip(order, order[1:])):
            e = cache[f"{x}->{y}"]
            label = e["plan"] + ("" if e["ok"] else "  <-- UNRESOLVED, colliding legs: " + ", ".join(e["bad"]))
            n_bad += 0 if e["ok"] else 1
            print(f"  {x if x != PARK else 'park':34s} -> {y if y != PARK else 'park':34s} {label}")
            if e["plan"] != "direct" or not e["ok"]:
                for ln, q in e["legs"]:
                    main.MoveJ(jt(f"Main_T{i + 1:02d}_{ln}", q))
            if y != PARK:
                main.RunInstruction(y, robolink.INSTRUCTION_CALL_PROGRAM)
                called.append(y)
        main.RunInstruction("GunOn(2)", robolink.INSTRUCTION_CALL_PROGRAM)
    finally:
        gpp.SEG_STEP_DEG, gpp.SEG_STEP_MM = saved
    print(f"\nAuto_Main built: {len(called)} programs, {len(order) - 1} hops, {n_bad} unresolved.")
    return called, n_bad, safe_q


def zone_group(name):
    n = name.lower()
    if "hood" in n or "roof" in n or "trunk" in n:
        return "top"
    for g in ("rear", "right", "left", "front"):
        if f"_{g}" in n:
            return g
    return "other"


def ring_order(RDK, names):
    """Tour around the car: top (hood, roof, trunk) -> rear -> one side -> front -> the other side. Both directions are costed (gantry distance +
    arm swing between where a program ends and the next begins); inside each group the cheapest order is chosen. Returns (order, cost, reverse cost)."""
    import itertools
    ends = {nm: program_ends(RDK, nm) for nm in names}
    ok = [nm for nm in names if ends[nm][0] is not None and ends[nm][1] is not None]
    groups = {}
    for nm in ok:
        groups.setdefault(zone_group(nm), []).append(nm)
    top = sorted(groups.get("top", []), key=lambda n: ("hood" not in n.lower(), "roof" not in n.lower()))     # hood, roof, trunk

    def chain(group_names, prev_last):
        best = None
        for perm in itertools.permutations(group_names):
            c = (hop_cost(ends[prev_last][1], ends[perm[0]][0]) if prev_last else 0.0) + sum(hop_cost(ends[x][1], ends[y][0]) for x, y in zip(perm, perm[1:]))
            if best is None or c < best[0]:
                best = (c, list(perm))
        return best

    def build(sequence):
        order, total = list(top), sum(hop_cost(ends[x][1], ends[y][0]) for x, y in zip(top, top[1:]))
        for g in sequence:
            if g in groups and groups[g]:
                c, perm = chain(groups[g], order[-1] if order else None)
                order += perm
                total += c
        return order, total
    a1, c1 = build(("rear", "right", "front", "left"))
    a2, c2 = build(("rear", "left", "front", "right"))
    others = groups.get("other", [])
    best, cost, other_cost = (a1, c1, c2) if c1 <= c2 else (a2, c2, c1)
    return best + others + [nm for nm in names if nm not in ok], cost, other_cost


def remove_stray_programs(RDK):
    """A failed attach leaves programs named just 'Auto' behind; delete them."""
    n = 0
    for _ in range(200):
        it = RDK.Item("Auto", robolink.ITEM_TYPE_PROGRAM)
        if not it.Valid():
            break
        it.Delete()
        n += 1
    return n


def make_auto_main(RDK, robot, names):
    old = RDK.Item("Auto_Main", robolink.ITEM_TYPE_PROGRAM)
    if old.Valid():
        old.Delete()
    prog = RDK.AddProgram("Auto_Main", robot)
    prog.RunInstruction("GunOn(-1)", robolink.INSTRUCTION_CALL_PROGRAM)
    called = []
    for nm in names:
        if RDK.Item(nm, robolink.ITEM_TYPE_PROGRAM).Valid():
            prog.RunInstruction(nm, robolink.INSTRUCTION_CALL_PROGRAM)
            called.append(nm)
        else:
            print(f"  (program '{nm}' does not exist - left out of Auto_Main)")
    prog.RunInstruction("GunOn(2)", robolink.INSTRUCTION_CALL_PROGRAM)
    return called


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zones-json", default="zone_curves.json")
    ap.add_argument("--zones", nargs="*", type=int, help="zone ids to build (default: all)")
    ap.add_argument("--template", default="Path2 settings")
    ap.add_argument("--paint-stl", default="car_paint_only_4800x1900x1600.stl")
    ap.add_argument("--standoff", type=float, default=150.0, help="nozzle-to-skin distance (mm); also the default path-to-tool Z")
    ap.add_argument("--offset-z", type=float, help="Z of the path-to-tool offset (mm, path frame). Default: +standoff when the template has "
                    "no translation. If the gun ends up INSIDE the car, use the opposite sign.")
    ap.add_argument("--speed", type=float, default=150.0, help="operation speed (mm/s); cannot be read back from the template, so it is always set")
    ap.add_argument("--salvage", action="store_true", help="if a zone cannot be attached, test its curves one by one and rebuild the zone "
                    "with the curves the robot can follow (the dropped ones are reported)")
    ap.add_argument("--avoid-pass", action="store_true", help="after a failed pass 1 also try RoboDK's collision avoidance (slow: ~5 min, "
                    "and on the hood it could not attach at all)")
    ap.add_argument("--no-retry-starts", action="store_true", help="do not retry a failed zone with the arm base further from the gun (1100, 1300 mm)")
    ap.add_argument("--init-pose", choices=("template", "current"), default="template",
                    help="before the IK set-up, move the robot to the template's start pose (default). The Home pose has a straight elbow, "
                         "a singularity where RoboDK's IK finds no solution; 'current' keeps wherever the robot is")
    ap.add_argument("--skip", nargs="*", type=int, default=[], help="zone ids that are already built and good: reuse their existing "
                    "program (Auto_<id>_<label>) as it is, do not plan them again (they are still called by Auto_Main)")
    ap.add_argument("--resume", action="store_true", help="skip zones that build_report.json already lists as OK and whose program still exists")
    ap.add_argument("--split", type=int, default=0, help="a zone that still fails is cut into chunks of this many curves, each planned "
                    "with its own start pose (e.g. --split 3); parts that work are kept, the rest is reported as uncovered")
    ap.add_argument("--clean", action="store_true", help="first delete every item named 'Auto ...' / 'Auto_...' in the station and the old "
                    "build_report.json (use it when you switch to a different zoning, so old zones are not mixed into Auto_Main)")
    ap.add_argument("--no-autosave", action="store_true", help="do not write <station>_autosave.rdk after every zone")
    ap.add_argument("--main-only", action="store_true", help="plan nothing: only (re)create the program Auto_Main from existing programs "
                    "(automatic ones from build_report.json, or exactly the names given with --order)")
    ap.add_argument("--order", nargs="*", default=None, help="with --main-only: program names in the order Auto_Main should call them, "
                    "e.g. Auto_00_hood Manual_front Auto_01_roof ... (your hand-made programs can be mixed in)")
    ap.add_argument("--extra", nargs="*", default=[], help="with --main-only and no --order: hand-made programs appended at the end")
    ap.add_argument("--auto-order", action="store_true", help="order the programs in Auto_Main to minimise travel between them (gantry distance + "
                    "arm swing, measured from each program's own first and last joint values) instead of by zone number")
    ap.add_argument("--transit", action="store_true", help="with --main-only: put collision-checked TRANSIT moves between the programs "
                    "(lift the gantry, fold the arm, travel over the car, lower) wherever a straight joint move would hit something")
    ap.add_argument("--transit-step", type=float, nargs=2, metavar=("DEG", "MM"), default=[5.0, 80.0],
                    help="collision sampling step along each transit leg (default 5 deg / 80 mm; the library default is finer but slower)")
    ap.add_argument("--ring-order", action="store_true", help="with --main-only (no transits): order the programs as a tour AROUND the car -- "
                    "top (hood, roof, trunk), rear, one side, front, the other side -- choosing the cheaper direction and the best order inside each group")
    ap.add_argument("--prune", action="store_true", help="delete the leftovers of every piece that did not solve (its project, program and curve object, "
                    "per build_report.json) and the stray 'Auto' programs; plans nothing")
    ap.add_argument("--allow-wound", action="store_true", help="accept start poses whose elbow (J3) is wound beyond +-190 deg. By default such poses are "
                    "tried last: programs that start and end in a different arm 'family' cannot be joined by a safe transit")
    ap.add_argument("--drop-programs", nargs="*", default=[], help="with --main-only: program names to leave out of Auto_Main (e.g. Auto_07_left_rail_p2)")
    ap.add_argument("--max-iter", type=int, default=6, help="with --transit --auto-order: how many times the order search may re-plan after "
                    "finding hops that cannot be solved (each round forbids those pairs)")
    ap.add_argument("--exclude", nargs="*", type=int, default=[], help="zone ids to leave OUT of Auto_Main (e.g. --exclude 3 for the front)")
    ap.add_argument("--diagnose", action="store_true", help="after a failed collision check, also ask RoboDK WHICH items collide (informational only; "
                    "it can hang RoboDK when the arm sits deep inside the car, so it is off by default)")
    ap.add_argument("--dry", action="store_true", help="only compute the start poses; create nothing")
    ap.add_argument("--main", action="store_true", help="also create the program Auto_Main calling the zone programs")
    ap.add_argument("--approach", type=float, default=-50.0, help="approach/retract distance along the normal (mm); the template default is 100. "
                    "Written as the RoboDK command 'Approach Normal <mm>'. With the SPG-style normals (pointing INTO the car) a positive value moves "
                    "the approach point TOWARD the car; the default -50 approaches from outside (confirmed on zone 8: tcp-to-path = the offset).")
    ap.add_argument("--no-spray", action="store_true", help="do not add any spray events")
    ap.add_argument("--spray-on", help="name of a spray-ON script in the station to call at each path start (e.g. SprayOn) instead of GunOn(1,r,g,b,standoff)")
    ap.add_argument("--spray-off", help="name of the spray-OFF script (e.g. SprayOff) instead of GunOn(0)")
    a = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    zj = json.load(open(a.zones_json if os.path.isfile(a.zones_json) else os.path.join(here, a.zones_json)))
    zones = [z for z in zj["zones"] if z["curves"] and (not a.zones or z["id"] in a.zones)]
    stl = a.paint_stl if os.path.isfile(a.paint_stl) else os.path.join(here, a.paint_stl)
    mesh = pcm.load_mesh(stl)

    RDK = robolink.Robolink()
    try:                                   # the API's default is 10 s; a busy RoboDK (collision checks) then raises "Communication problems"
        RDK.TIMEOUT = max(getattr(RDK, "TIMEOUT", 10), 1800)
        RDK.COM.settimeout(RDK.TIMEOUT)
    except Exception:
        pass
    if a.prune:
        try:
            rep = json.load(open(os.path.join(here, "build_report.json")))
        except Exception:
            raise SystemExit("build_report.json not found: nothing to prune.")
        removed = []
        for r in rep:
            solved_child = r["status"].startswith("OK") and r.get("program")
            if solved_child:
                continue                       # a program that solved: keep it, its project and its curves
            base = f"Auto {int(r['id']):02d} {r['label']}"
            for nm, kind in ((base + " settings", robolink.ITEM_TYPE_MACHINING), (f"Auto_{int(r['id']):02d}_{r['label']}", robolink.ITEM_TYPE_PROGRAM),
                             (base + " curves", robolink.ITEM_TYPE_OBJECT)):
                it = RDK.Item(nm, kind)
                if it.Valid():
                    it.Delete()
                    removed.append(nm)
        removed_stray = remove_stray_programs(RDK)
        print(f"Pruned {len(removed)} leftover items of pieces that did not solve" + (f" and {removed_stray} stray 'Auto' programs." if removed_stray else "."))
        for nm in removed:
            print("   ", nm)
        return
    if a.main_only and not a.transit:
        robot_m = RDK.Item(gpp.ROBOT_NAME, robolink.ITEM_TYPE_ROBOT)
        if a.order is not None:
            names = list(a.order)
        else:
            try:
                rep = json.load(open(os.path.join(here, "build_report.json")))
            except Exception:
                rep = []
            rep = sorted(rep, key=lambda r: (int(r["id"]), str(r["label"])))
            names = [r["program"] for r in rep if r["status"].startswith("OK") and r.get("program") and r["id"] not in a.exclude] + list(a.extra)
        names = [nm for nm in names if nm not in a.drop_programs]
        if a.ring_order and a.order is None:
            names, c_best, c_other = ring_order(RDK, names)
            print(f"--ring-order: tour cost {c_best / 1000:.1f} (the opposite direction would cost {c_other / 1000:.1f}; units: metres of gantry travel + 5 mm per degree of arm swing)")
            a.auto_order = False
        if a.auto_order and a.order is None:
            before = list(names)
            names, c_new, c_old = auto_order(RDK, names)
            if c_new is not None:
                print(f"--auto-order: travel cost {c_old / 1000:.1f} -> {c_new / 1000:.1f} (units: metres of gantry travel + 5 mm per degree of arm swing)")
        called = make_auto_main(RDK, robot_m, names)
        print(f"Auto_Main created with {len(called)} programs, in this order:")
        for nm in called:
            print("   ", nm)
        return
    if a.init_pose == "template":
        robot0 = RDK.Item(gpp.ROBOT_NAME, robolink.ITEM_TYPE_ROBOT)
        tmpl0 = RDK.Item(a.template, robolink.ITEM_TYPE_MACHINING)
        if robot0.Valid() and tmpl0.Valid():
            try:
                robot0.setJoints(tmpl0.Joints())
                print("Robot moved to the template's start pose (bent elbow, no singularity) before the IK set-up.")
            except Exception as exc:
                print(f"(could not move the robot to the template's start pose: {exc})")
        else:
            print(f'(template "{a.template}" not found: leaving the robot where it is)')
    ctx = gpp.setup_cell(RDK, standoff_arg=str(a.standoff), no_align=False)
    if a.main_only and a.transit:
        if a.order is not None:
            names = list(a.order)
        else:
            try:
                rep = json.load(open(os.path.join(here, "build_report.json")))
            except Exception:
                rep = []
            rep = sorted(rep, key=lambda r: (int(r["id"]), str(r["label"])))
            names = [r["program"] for r in rep if r["status"].startswith("OK") and r.get("program") and r["id"] not in a.exclude] + list(a.extra)
        names = [nm for nm in names if nm not in a.drop_programs]
        if a.auto_order and a.order is None:
            _, _, park_q = build_feasible_main(RDK, ctx, names, tuple(a.transit_step), a.max_iter, here)
            try:
                ctx.robot.setJoints(park_q)
            except Exception:
                pass
            print("The robot is left at the park pose. Play Auto_Main from here (and check it with Shift+F5).")
            return
        _, _, park_q = build_main_with_transits(RDK, ctx, names, tuple(a.transit_step))
        try:                                   # leave the robot at the park pose: Auto_Main then starts without an unchecked first move
            ctx.robot.setJoints(park_q if park_q is not None else ctx.original)
        except Exception:
            pass
        print("The robot is left at the park pose. Play Auto_Main from here (and check it with Shift+F5).")
        return
    cell, car_T, robot, tool = ctx.cell, ctx.car_T, ctx.robot, ctx.tool
    T = None if a.dry else read_template(RDK, a.template)
    frame = None if a.dry else T["frame"]

    offset_pose = None
    if T is not None:
        offset_pose = T["pose"].copy()
        if a.offset_z is not None:
            offset_pose[2, 3] = a.offset_z
            print(f"Path-to-tool offset: Z = {a.offset_z:g} mm (from --offset-z).")
        elif np.linalg.norm(offset_pose[:3, 3]) < 1.0:
            offset_pose[2, 3] = a.standoff
            print(f"Template offset has no translation: using Z = +{a.standoff:g} mm in the path frame "
                  f"(my best guess for 'out of the car'; if the gun ends up inside the car run again with --offset-z -{a.standoff:g}).")
        else:
            print(f"Template offset translation kept as it is: {np.round(offset_pose[:3, 3], 1).tolist()} mm.")

    if a.clean and not a.dry:
        n_del = 0
        for nm in RDK.ItemList(list_names=True):
            if nm.startswith("Auto ") or nm.startswith("Auto_") or nm == "Auto":
                it = RDK.Item(nm)
                if it.Valid():
                    it.Delete()
                    n_del += 1
        rp0 = os.path.join(here, "build_report.json")
        if os.path.isfile(rp0):
            os.remove(rp0)
        print(f"--clean: deleted {n_del} 'Auto' items and the old report.")
    print(f"\nZones to process: {[z['id'] for z in zones]}")
    def autosave():
        """Copy of the station next to the original (<name>_autosave.rdk): if RoboDK dies, the finished zones are not lost."""
        if a.no_autosave or a.dry:
            return
        try:
            path = RDK.getParam("FILE_OPENSTATION")
            if path:
                RDK.Save(os.path.splitext(path)[0] + "_autosave.rdk")
        except Exception as exc:
            print(f"      (autosave failed: {exc})")

    report = []
    good_seeds = {}                        # orientation class -> start pose that worked

    def process_zone(z, allow_split=True):
        """Plan one zone (or one chunk of a zone). Returns the list of records (the zone's own, plus its parts if it was split)."""
        t0 = time.time()
        zid, label = z["id"], z["label"]
        rec = {"id": zid, "label": label, "curves": len(z["curves"]), "status": "", "pass": 0}
        try:                               # building the previous zone left 'Car Frame' as the active reference: neutralise it
            robot.setPoseTool(tool)        # (an ITEM, never a matrix: a matrix would move the active frame item, i.e. the car)
            robot.setPoseFrame(ctx.ik_frame_item if ctx.ik_frame_item is not None else ctx.base)
        except Exception:
            pass
        cls = orientation_class(z)
        print(f"  zone {zid:2d} {label}: searching a start pose (about 1-3 minutes, the terminal stays quiet) ...", flush=True)
        q, off, hits, n, k = start_for_zone(cell, car_T, z, a.standoff, seed=good_seeds.get(cls))
        t_search = time.time() - t0
        if q is None:
            rec["status"] = f"NO START POSE (offset {off}, {hits}/{n} sample poses reachable)"
            print(f"  zone {zid:2d} {label:16s} {rec['status']}")
            return [rec]
        rec.update({"offset_L_a_b": [float(v) for v in off], "reachable_samples": f"{hits}/{n}", "start_q": [round(v, 1) for v in q]})
        if a.dry:
            rec["status"] = "start pose found (dry run)"
            print(f"  zone {zid:2d} {label:16s} start pose OK  offset L={off[0]:.0f} a={off[1]:.0f} b={off[2]:.0f}  "
                  f"samples {hits}/{n}  q={[round(v) for v in q]}")
            return [rec]

        # ---- RoboDK items (names make a re-run replace the previous build)
        base = f"Auto {zid:02d} {label}"
        name_proj, name_obj, name_prog = base + " settings", base + " curves", f"Auto_{zid:02d}_{label}"
        add = getattr(RDK, "AddMillingProject", None) or getattr(RDK, "AddMachiningProject")
        setp_name = "setMillingParameters" if hasattr(robolink.Item, "setMillingParameters") else "setMachiningParameters"
        rgb = COLORS[zid % len(COLORS)]
        state = {"prog": None, "curves": z["curves"], "q": q}

        def build_project(obj, pname, gname, avoid, rotz, quick=False):
            """Create a project on `obj` with the template settings applied first; returns (proj, prog, status, result tuple)."""
            old = RDK.Item(pname, robolink.ITEM_TYPE_MACHINING)              # start every attempt from a clean project
            if old.Valid():
                old_prog = old.getLink(robolink.ITEM_TYPE_PROGRAM)
                old.Delete()
                if old_prog is not None and old_prog.Valid():
                    old_prog.Delete()
            proj = add(pname)
            machining = dict(T["machining"])
            machining["AvoidCollisions"] = 1 if avoid else 0
            if rotz is not None:
                machining["RotZ_Range"] = rotz
            optim = dict(T["optim"])
            qq = state["q"]
            for i in range(len(qq)):
                optim[f"AbsJnt_{i + 1}"] = qq[i]
            events = dict(T["events"])
            if not a.no_spray:
                on_call = a.spray_on or f"GunOn(1,{rgb[0]:g},{rgb[1]:g},{rgb[2]:g},{a.standoff:g})"
                off_call = a.spray_off or "GunOn(0)"
                events.update({"CallPathStart": on_call, "CallPathStartOn": 1, "CallPathFinish": off_call, "CallPathFinishOn": 1})
            robot.setPoseTool(tool)
            robot.setPoseFrame(frame)
            proj.setRobot(robot)
            proj.setPoseTool(tool)
            proj.setPoseFrame(frame)
            proj.setParam("Machining", machining)
            proj.setParam("OptimAxes", optim)
            proj.setParam("ProgEvents", events)
            proj.setJoints(qq)
            proj.setPose(Mat(offset_pose.tolist()))
            if a.speed is not None:
                proj.setParam("OperationSpeed", a.speed)
            if a.approach is not None:
                try:
                    proj.setParam("Approach", f"Normal {a.approach:g}")
                    proj.setParam("Retract", f"Normal {a.approach:g}")
                except Exception as exc:
                    print(f"      (could not set approach/retract: {exc})")
            prog, status = getattr(proj, setp_name)(part=obj, params="ReorderAuto=0")
            if prog is None or not prog.Valid() or status < 1.0 - 1e-6:
                msg = f"the path could not be attached (status {status})"
                if not quick:
                    try:                                     # RoboDK's own explanation, e.g. "Can't find a start point"
                        why = proj.Update(robolink.COLLISION_OFF, timeout_sec=120)[4]
                        if why:
                            msg += " - RoboDK says: " + str(why)
                    except Exception:
                        pass
                return proj, prog, status, (0, 0.0, 0.0, -1.0, msg)
            prog.setName(gname)
            if quick:
                return proj, prog, status, (0, 0.0, 0.0, 1.0, "attached")
            return proj, prog, status, proj.Update(robolink.COLLISION_ON)

        def solve_zone(curves):
            """Pass 1 (avoid off) and, only if the path attached but the check failed, pass 2 (avoid ON). Fills `rec`."""
            delete_old(RDK, name_proj, name_obj, name_prog)
            obj = make_curve_object(RDK, frame, {"curves": curves}, name_obj)
            state["curves"] = curves
            rec["attempts"] = []
            if obj is None:
                rec.update({"status": "FAILED", "pass": 0, "attach_status": -9.0, "valid_ratio": -1.0,
                            "message": "RoboDK could not create the curve object"})
                print("      RoboDK could not create the curve object for this zone.")
                return
            passes = ((False, None), (True, 60.0)) if a.avoid_pass else ((False, None),)
            for pass_no, (avoid, rotz) in enumerate(passes, 1):
                proj, prog, status, res = build_project(obj, name_proj, name_prog, avoid, rotz)
                state["prog"] = prog
                rec["pass"] = pass_no
                rec["attempts"].append({"pass": pass_no, "avoid_collisions": avoid, "attach_status": status,
                                        "valid_ratio": res[3], "message": res[4]})
                print(f"      pass {pass_no} (avoid collisions {'ON' if avoid else 'off'}, spin +/-{rotz or 180:g}): "
                      f"attach status {status}, valid_ratio {res[3]:.2f} - {clean_msg(res[4])}")
                if a.diagnose and status >= 1.0 - 1e-6 and res[3] < 0.999 and "ollision" in str(res[4]):
                    print("      (asking RoboDK which items collide ...)", flush=True)
                    info = collision_report(RDK, robot, prog, res[4])
                    if info:
                        rec["collision"] = info
                        print(f"      colliding items at instruction {info['instruction']}: {'; '.join(info['pairs'])}")
                        print(f"      joints there (arm J1-J6, gantry d7-d9): {info['joints']}")
                rec.update({"attach_status": status, "valid_ratio": res[3], "time_s": round(res[1], 1), "distance_mm": round(res[2]),
                            "message": res[4]})
                if res[3] >= 0.999 and status >= 1.0 - 1e-6:
                    rec["status"] = "OK"
                    return
                rec["status"] = "FAILED"
                if status < 1.0 - 1e-6:      # not attached = unreachable / no start point: collision avoidance cannot fix that
                    print("      (not attached, so a second pass with avoidance would not help - skipped)")
                    return

        wound_fallbacks = []
        if is_wound(q) and not a.allow_wound:
            print(f"      (the first start pose has a wound-up elbow, J3 = {q[2]:.0f} deg: trying other start poses first)", flush=True)
            wound_fallbacks.append((list(q), off, hits, n, "the first start pose"))
            rec["status"] = "SKIPPED (wound elbow)"
        else:
            solve_zone(z["curves"])
        if not rec["status"].startswith("OK") and not a.no_retry_starts:
            retry_Ls = ((1100.0,), (1300.0,)) if abs(z["mean_normal"][2]) >= 0.5 else ((400.0,), (1100.0,))   # top panels: straighter arm; vertical faces: closer
            cls_seed = good_seeds.get(cls)
            tries = [(Ls, cls_seed, f"arm base {Ls[0]:.0f} mm from the gun") for Ls in retry_Ls]
            if cls_seed is not None:        # the first attempt used a seed from an earlier zone: now try the template pose as the seed (and the reverse)
                tries.append((None, None, "the template pose as the IK seed"))
            else:
                other = next(iter(good_seeds.values()), None)
                if other is not None:
                    tries.append((None, other, "the pose of an earlier good zone as the IK seed"))
            for Ls, sd, what in tries:
                print(f"      retry: another start pose ({what}) - searching, about 1-3 minutes ...", flush=True)
                q2, off2, hits2, n2, k2 = start_for_zone(cell, car_T, z, a.standoff, Ls=Ls, seed=sd)
                if q2 is None:
                    print("      (no start pose this way)")
                    continue
                if is_wound(q2) and not a.allow_wound:
                    print(f"      (that start pose also has a wound-up elbow, J3 = {q2[2]:.0f} deg: kept as a last resort)", flush=True)
                    wound_fallbacks.append((list(q2), off2, hits2, n2, what))
                    continue
                state["q"] = q2
                rec.update({"offset_L_a_b": [float(v) for v in off2], "reachable_samples": f"{hits2}/{n2}", "start_q": [round(v, 1) for v in q2],
                            "retry_base_distance": Ls[0] if Ls else None, "retry": what})
                solve_zone(z["curves"])
                if rec["status"].startswith("OK"):
                    break
        if not rec["status"].startswith("OK") and wound_fallbacks:
            fq, foff, fhits, fn, fwhat = wound_fallbacks[0]
            print(f"      last resort: solving with the wound-elbow start pose ({fwhat})", flush=True)
            state["q"] = fq
            rec.update({"offset_L_a_b": [float(v) for v in foff], "reachable_samples": f"{fhits}/{fn}", "start_q": [round(v, 1) for v in fq],
                        "retry": "last resort: " + fwhat, "wound_elbow": True})
            solve_zone(z["curves"])
        if rec["status"].startswith("OK") and state.get("q") is not None and not is_wound(state["q"]):
            good_seeds[cls] = list(state["q"])             # remember it for the next zone of this orientation
        if (rec["status"] != "OK" and a.salvage and rec.get("attach_status", 1.0) < 1.0 - 1e-6 and len(z["curves"]) > 1):
            print("      salvage: attaching each curve on its own to find out which ones the robot can follow ...")
            good, tests = [], []
            for k, c in enumerate(z["curves"]):
                tmp_obj = make_curve_object(RDK, frame, {"curves": [c]}, f"{base} tmp{k}")
                if tmp_obj is None:
                    pj, pg, st, ok = None, None, -9.0, False
                else:
                    pj, pg, st, _ = build_project(tmp_obj, f"{base} tmp{k} settings", f"Auto_tmp_{zid:02d}_{k}", False, None, quick=True)
                    ok = pg is not None and pg.Valid() and st >= 1.0 - 1e-6
                length = path_len(np.array(c["points"]))
                tests.append({"curve": k, "points": len(c["points"]), "length_mm": round(length), "attached": bool(ok), "status": st})
                print(f"        curve {k:2d}: {len(c['points']):4d} points, {length:7.0f} mm -> {'attaches' if ok else f'FAILS (status {st})'}")
                for it in (pj, pg, tmp_obj):
                    try:
                        if it is not None and it.Valid():
                            it.Delete()
                    except Exception:
                        pass
                if ok:
                    good.append(k)
            rec["curve_tests"] = tests
            if good and len(good) < len(z["curves"]):
                lost = sum(t["length_mm"] for t in tests if not t["attached"])
                rec["dropped_curves"] = [t["curve"] for t in tests if not t["attached"]]
                print(f"      salvage: rebuilding with {len(good)}/{len(z['curves'])} curves (dropping {lost} mm of path)")
                solve_zone([z["curves"][k] for k in good])
                if rec["status"] == "OK":
                    rec["status"] = "OK (partial)"
            elif not good:
                print("      salvage: no single curve attaches - the problem is systematic for this zone, not one bad curve.")
        if not rec["status"].startswith("OK"):
            print(f"      -> open the project '{name_proj}' in RoboDK: its panel shows RoboDK's own message "
                  f"(e.g. 'Can't find a start point' / 'cannot reach all targets').")
        else:
            prog = state["prog"]
            rec["program"] = prog.Name()
            dist = measure_standoff(robot, frame, prog, np.array([p for c in state["curves"] for p in c["points"]], float))
            rec["tcp_to_path_mm"] = None if dist is None else round(dist, 1)
        print(f"  zone {zid:2d} {label:16s} {rec['status']:11s} pass {rec['pass']}  valid_ratio={rec.get('valid_ratio', 0):.2f}  "
              f"{rec.get('time_s', 0):6.1f}s of motion  {rec.get('distance_mm', 0):6d} mm   tcp-to-path={rec.get('tcp_to_path_mm')}  "
              f"[{time.time() - t0:.0f} s to build: start-pose search {t_search:.0f} s, solve+check {time.time() - t0 - t_search:.0f} s]")
        out = [rec]
        has_parts = bool(z.get("parts")) and len(z["parts"]) > 1
        if not rec["status"].startswith("OK") and a.split > 0 and allow_split and (has_parts or len(z["curves"]) > a.split):
            if has_parts:               # a grouped zone falls back to the patches it was made of
                subs = [{"id": zid, "label": f"{label}_{p['label']}", "curves": p["curves"], "mean_normal": p["mean_normal"]} for p in z["parts"]]
                print(f"      split: planning the {len(subs)} patches this zone is made of, one by one ...")
            else:                       # a single patch falls back to chunks of --split curves
                subs = []
                for pi, i in enumerate(range(0, len(z["curves"]), a.split), 1):
                    zp = dict(z)
                    zp.update({"label": f"{label}_p{pi}", "curves": z["curves"][i:i + a.split]})
                    subs.append(zp)
                print(f"      split: planning {len(subs)} parts of up to {a.split} curves, each with its own start pose ...")
            direct, everything = [], []
            for zp in subs:
                res = process_zone(zp, allow_split=True)
                direct.append(res[0])
                everything.extend(res)
            n_ok = sum(1 for r in direct if r["status"].startswith("OK"))
            n_part = sum(1 for r in direct if r["status"].startswith("PARTIAL"))
            lost = sum(r.get("uncovered_path_mm", 0) if r["status"].startswith("PARTIAL") else
                       (0 if r["status"].startswith("OK") else sum(path_len(np.array(c["points"])) for c in zp_["curves"]))
                       for r, zp_ in zip(direct, subs))
            rec["status"] = ("OK (split)" if n_ok == len(subs) else f"PARTIAL {n_ok + n_part}/{len(subs)}") if (n_ok or n_part) else "FAILED"
            rec["uncovered_path_mm"] = round(lost)
            print(f"  zone {zid:2d} {label:16s} {rec['status']}  ({n_ok} of {len(subs)} parts fully planned"
                  f"{f', {n_part} partly' if n_part else ''}, {lost / 1000:.1f} m of path uncovered)")
            out += everything
        return out

    def write_report():
        """Cumulative report: this run's records are merged into the file (a partial or crashed run keeps what was built before)."""
        rpath = os.path.join(here, "build_report_dry.json" if a.dry else "build_report.json")
        merged = {}
        if not a.dry and os.path.isfile(rpath):
            try:
                merged = {f"{r['id']}:{r['label']}": r for r in json.load(open(rpath))}
            except Exception:
                merged = {}
        for r in report:
            key = f"{r['id']}:{r['label']}"
            if r["status"] == "OK (kept)" and key in merged:
                continue
            merged[key] = r
        with open(rpath, "w") as f:
            json.dump([merged[k] for k in sorted(merged, key=lambda k: (int(k.split(":")[0]), k))], f, indent=1)
        return rpath, merged

    previous = {}
    if a.resume and not a.dry:
        try:
            previous = {f"{r['id']}:{r['label']}": r for r in json.load(open(os.path.join(here, "build_report.json")))}
        except Exception:
            previous = {}
    aborted = False
    for z in zones:
        zid, label = z["id"], z["label"]
        prog_name = f"Auto_{zid:02d}_{label}"
        prev = previous.get(f"{zid}:{label}")
        reuse = (zid in a.skip) or (prev is not None and (prev["status"].startswith("OK") or prev["status"].startswith("PARTIAL")))
        if reuse and not a.dry:
            if prev is not None and prev["status"].startswith("PARTIAL"):
                print(f"  zone {zid:2d} {label:16s} kept: {prev['status']} from an earlier run")
                continue
            if RDK.Item(prog_name, robolink.ITEM_TYPE_PROGRAM).Valid():
                print(f"  zone {zid:2d} {label:16s} kept: program {prog_name} already exists, not planned again")
                report.append({"id": zid, "label": label, "curves": len(z["curves"]), "status": "OK (kept)", "pass": 0, "program": prog_name})
                write_report()
                continue
            print(f"  zone {zid:2d}: reuse requested but {prog_name} does not exist - planning it")
        try:
            report.extend(process_zone(z))
            remove_stray_programs(RDK)
            autosave()
        except Exception as exc:
            msg = str(exc)
            print(f"  zone {zid:2d} {label:16s} ERROR: {msg}")
            report.append({"id": zid, "label": label, "curves": len(z["curves"]), "status": f"ERROR: {msg[:80]}", "pass": 0})
            if "ommunication" in msg or "Unknown error" in msg:
                print("\nRoboDK stopped answering. Check that it is still open (SAVE THE STATION if it is), then run the same command "
                      "again with --resume to carry on from here.")
                aborted = True
        write_report()
        if aborted:
            break

    rpath, merged = write_report()
    if a.main and not a.dry and not aborted:
        names = []
        for k in sorted(merged, key=lambda k: (int(k.split(":")[0]), k)):
            r = merged[k]
            if r["status"].startswith("OK") and r.get("program") and r["id"] not in a.exclude:
                names.append(r["program"])
        if a.auto_order:
            names, c_new, c_old = auto_order(RDK, names)
            if c_new is not None:
                print(f"--auto-order: travel cost {c_old / 1000:.1f} -> {c_new / 1000:.1f} (metres of gantry travel + 5 mm per degree of arm swing)")
        called = make_auto_main(RDK, robot, names)
        print(f"Created Auto_Main: {len(called)} programs ({'cheapest order' if a.auto_order else 'zone order'}; NO transit moves between them yet).")

    if not a.dry and merged:
        print(f"\n{'id':>3} {'zone':22s} {'status':12s} {'base L':>6s} {'path m':>7s} {'curves':>6s}  note")
        for k in sorted(merged, key=lambda k: (int(k.split(":")[0]), k)):
            r = merged[k]
            note = ""
            if r.get("dropped_curves"):
                note = f"dropped curves {r['dropped_curves']}"
            elif r.get("uncovered_path_mm"):
                note = f"{r['uncovered_path_mm'] / 1000:.1f} m uncovered"
            elif not r["status"].startswith("OK") and not r["status"].startswith("PARTIAL"):
                note = str(r.get("message", ""))[:60]
            Lb = r.get("retry_base_distance") or (r.get("offset_L_a_b") or [0])[0]
            print(f"{r['id']:3d} {r['label']:22s} {r['status']:12s} {Lb:6.0f} {r.get('distance_mm', 0) / 1000:7.1f} {r['curves']:6d}  {note}")
    n_ok = sum(1 for r in merged.values() if r["status"].startswith("OK") or r["status"].startswith("start pose found"))
    print(f"\n{n_ok}/{len(merged)} entries OK   (report: {rpath})")
    try:
        ctx.robot.setJoints(ctx.original)               # put the robot back (not essential)
    except Exception as exc:
        print(f"(could not restore the robot's starting pose: {exc} - press Home in RoboDK)")


if __name__ == "__main__":
    main()
