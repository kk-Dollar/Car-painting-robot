#!/usr/bin/env python3
"""paint_overlay.py - show the paint WITHOUT RoboDK's particle spray.

RoboDK's spray engine could not be aimed reliably (its statistics said 0 % of the particles landed on the car with every normal-sized cone). So this tool
works from the real robot poses instead:

  1. For every stroke (gun ON ... gun OFF) of every program called by Auto_Main it moves the robot through the stroke's targets, shoots a ray along the
     gun's Z axis at the car's paint shell, and collects the hit points.
  2. Every shell triangle within  --width / 2  of those hit points is "painted by that stroke". Those triangles become a thin red surface object
     (lifted --lift mm off the shell), hidden at first, named Paint_<program>_s<k>.
  3. In GunOn's "overlay" mode the end of each stroke (GunOn(0)) makes the next object visible: paint appears behind the gun, stroke by stroke.
  4. The union of all strokes gives the coverage in m2 and % of the shell: printed here, and by GunOn(2) at the end of Auto_Main.

  python3 paint_overlay.py build [--width 120] [--lift 3] [--step 4] [--subdivide 1] [--seg 250]   create the overlay objects from Auto_Main (about 2-4 minutes)
  python3 paint_overlay.py focus main | Auto_00_hood                     which strokes the next run reveals: the whole Auto_Main, or one program (for tests)
  python3 paint_overlay.py resequence                                    re-read the order of the programs in Auto_Main (do this after changing it; live_paint.py does it by itself)
  python3 paint_overlay.py clear                                         delete the overlay objects

Then:  python3 gunon_tools.py mode overlay   and play Auto_Main (or one program after `focus`).
Needs car_paint_only_4800x1900x1600.stl next to this script.
"""
import argparse
import json
import os
import re
import struct
import sys
import numpy as np
from robodk import robolink

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
OVERLAY_DIR = os.path.join(HERE, "overlay")
COLOR = [0.85, 0.10, 0.10, 1.0]


def to_np(m):
    return np.array([[float(v) for v in row] for row in m.rows], float)


def load_triangles(path):
    import paint_common as pc
    mesh = pc.load_mesh(path)
    return np.stack([mesh.v0, mesh.v0 + mesh.e1, mesh.v0 + mesh.e2], axis=1)          # T x 3 x 3


def ray_front(tris, origin, direction):
    """Nearest hit of the ray (Moller-Trumbore) in FRONT of the origin, or None."""
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    e1, e2 = v1 - v0, v2 - v0
    h = np.cross(direction, e2)
    a = np.einsum("ij,ij->i", e1, h)
    ok = np.abs(a) > 1e-12
    f = np.zeros_like(a)
    f[ok] = 1.0 / a[ok]
    s = origin - v0
    u = f * np.einsum("ij,ij->i", s, h)
    q = np.cross(s, e1)
    v = f * (q @ direction)
    t = f * np.einsum("ij,ij->i", e2, q)
    hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-6)
    return float(t[hit].min()) if hit.any() else None


def subdivide(tris, times):
    """Split every triangle into 4 (edge midpoints), `times` times: smoother strip edges where the shell has big triangles."""
    for _ in range(times):
        a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
        ab, bc, ca = (a + b) / 2, (b + c) / 2, (c + a) / 2
        tris = np.concatenate([np.stack([a, ab, ca], 1), np.stack([ab, b, bc], 1), np.stack([ca, bc, c], 1), np.stack([ab, bc, ca], 1)])
    return tris


def write_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            n = np.cross(t[1] - t[0], t[2] - t[0])
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack("<12fH", *n, *t.reshape(-1), 0))


def band(centroids, points, half_width):
    """Indices of the centroids within half_width of the polyline through `points` (resampled every ~10 mm), the resampled polyline, and for every
    selected centroid the index of its nearest polyline point."""
    pts = [points[0]]
    for a, b in zip(points, points[1:]):
        n = max(int(np.linalg.norm(b - a) // 10.0), 1)
        for k in range(1, n + 1):
            pts.append(a + (b - a) * (k / n))
    P = np.array(pts)
    dmin = np.empty(len(centroids))
    jmin = np.empty(len(centroids), dtype=int)
    for i in range(0, len(centroids), 1500):
        c = centroids[i:i + 1500]
        d = np.sqrt(((c[:, None, :] - P[None, :, :]) ** 2).sum(-1))
        j = d.argmin(axis=1)
        jmin[i:i + 1500] = j
        dmin[i:i + 1500] = d[np.arange(len(c)), j]
    sel = np.nonzero(dmin <= half_width)[0]
    return sel, P, jmin[sel]


def called_programs(RDK, main_name="Auto_Main"):
    main = RDK.Item(main_name, robolink.ITEM_TYPE_PROGRAM)
    if not main.Valid():
        raise SystemExit(f"Program {main_name} not found.")
    names = []
    for i in range(main.InstructionCount()):
        raw = (main.Instruction(i)[0] or "").strip()
        nm = re.sub(r"^call\s+", "", raw, flags=re.I).split("(")[0].strip()
        if nm and nm.lower() != "gunon" and RDK.Item(nm, robolink.ITEM_TYPE_PROGRAM).Valid() and nm != main_name:
            names.append(nm)
    return names


def strokes_of(prog):
    """List of strokes; each stroke = list of joint vectors of the moves made while the gun is ON."""
    strokes, cur, on = [], None, False
    for i in range(prog.InstructionCount()):
        ins = prog.Instruction(i)
        nm = (ins[0] or "").lower().replace(" ", "")
        if "gunon(" in nm:
            d = nm.split("gunon(")[1][:2]
            if d.startswith("1"):
                on, cur = True, []
            elif d.startswith("0") and on:
                on = False
                if cur:
                    strokes.append(cur)
                cur = None
            continue
        if on and ins[1] == getattr(robolink, "INS_TYPE_MOVE", 0):
            j = ins[5]
            q = [float(v) for v in (j.list() if hasattr(j, "list") else j)]
            if len(q) >= 6:
                cur.append(q)
    return strokes


def build(RDK, a):
    tool = RDK.Item("Paint gun", robolink.ITEM_TYPE_TOOL)
    carf = RDK.Item("Car Frame", robolink.ITEM_TYPE_FRAME)
    robots = RDK.ItemList(robolink.ITEM_TYPE_ROBOT)
    robot = next((r for r in robots if "ur10" in r.Name().lower()), robots[0] if robots else None)
    shell = next((it for it in RDK.ItemList(robolink.ITEM_TYPE_OBJECT) if "car_paint_only" in it.Name().lower()), None)
    stl = os.path.join(HERE, "car_paint_only_4800x1900x1600.stl")
    if not (tool.Valid() and carf.Valid() and robot is not None and shell is not None and os.path.isfile(stl)):
        raise SystemExit("Need the Paint gun, the Car Frame, the robot, the car_paint_only object and car_paint_only_4800x1900x1600.stl next to this script.")
    tris_ray = load_triangles(stl)                                   # the real shell: used for the rays
    tris = subdivide(tris_ray, a.subdivide)                          # finer copy: used to pick the painted strip (smoother edges)
    centroids = tris.mean(axis=1)
    cross = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    area = 0.5 * np.linalg.norm(cross, axis=1)
    normals = cross / np.maximum(np.linalg.norm(cross, axis=1, keepdims=True), 1e-12)
    programs = called_programs(RDK)
    print(f"Auto_Main calls {len(programs)} programs: {', '.join(programs)}")
    os.makedirs(OVERLAY_DIR, exist_ok=True)
    for old in [it for it in RDK.ItemList(robolink.ITEM_TYPE_OBJECT) if it.Name().startswith("Paint_")]:
        old.Delete()
    for old_frame in [it for it in RDK.ItemList(robolink.ITEM_TYPE_FRAME) if it.Name() == "PaintOverlay"]:
        old_frame.Delete()
    overlay_frame = RDK.AddFrame("PaintOverlay", carf)
    original = list(robot.Joints().list())
    prev_frame = robot.getLink(robolink.ITEM_TYPE_FRAME)
    robot.setPoseTool(tool)
    robot.setPoseFrame(carf)
    inv_shell = np.linalg.inv(to_np(shell.Pose()))
    shell_pose = shell.Pose()
    painted = np.zeros(len(tris), bool)
    strokes_meta, per_prog, n_missed, n_objects = [], {}, 0, 0
    for prog_name in programs:
        prog = RDK.Item(prog_name, robolink.ITEM_TYPE_PROGRAM)
        per_prog[prog_name] = []
        for k, stroke in enumerate(strokes_of(prog), 1):
            picks = stroke[::a.step] + ([stroke[-1]] if (len(stroke) - 1) % a.step else [])
            H, P = [], []
            for q in picks:
                robot.setJoints(q)
                To = inv_shell @ to_np(robot.Pose())
                t = ray_front(tris_ray, To[:3, 3], To[:3, 2])
                if t is not None and t < 600.0:
                    H.append(To[:3, 3] + t * To[:3, 2])
                    P.append(To[:3, 3])
                else:
                    n_missed += 1
            if len(H) < 2:
                print(f"  {prog_name} stroke {k}: the gun does not point at the shell (skipped)")
                continue
            idx, dense, jn = band(centroids, np.array(H), a.width / 2.0)
            if idx.size == 0:
                continue
            Hn = np.array(H)
            nearest = np.argmin(((centroids[idx][:, None, :] - Hn[None, :, :]) ** 2).sum(-1), axis=1)
            to_gun = np.array(P)[nearest] - centroids[idx]
            sign = np.where(np.einsum("ij,ij->i", normals[idx], to_gun) >= 0, 1.0, -1.0)[:, None]
            lifted = tris[idx] + (a.lift * sign * normals[idx])[:, None, :]
            arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))])
            gid = (arc[jn] // a.seg).astype(int)
            segs = []
            for g in sorted(set(gid.tolist())):
                m = gid == g
                name = f"Paint_{prog_name}_s{k}_g{g}"
                path = os.path.join(OVERLAY_DIR, name + ".stl")
                write_stl(path, lifted[m])
                item = RDK.AddFile(path, overlay_frame)
                item.setName(name)
                item.setPose(shell_pose)
                item.setColor(COLOR)
                item.setVisible(False)
                mid = int(np.searchsorted(arc, min((g + 0.5) * a.seg, arc[-1])))
                segs.append({"name": name, "center": [float(v) for v in dense[min(mid, len(dense) - 1)]]})
                n_objects += 1
            painted[idx] = True
            stroke_rec = {"program": prog_name, "k": k, "length_mm": float(arc[-1]), "segments": segs}
            strokes_meta.append(stroke_rec)
            per_prog[prog_name].append(stroke_rec)
        print(f"  {prog_name}: {len(per_prog[prog_name])} strokes, {sum(len(x['segments']) for x in per_prog[prog_name])} segments")
    robot.setJoints(original)
    try:
        if prev_frame.Valid():
            robot.setPoseFrame(prev_frame)
    except Exception:
        pass
    total, covered = float(area.sum()) / 1e6, float(area[painted].sum()) / 1e6
    cov = f"Painted surface {covered:.2f} m2 of {total:.2f} m2 of the paint shell ({100 * covered / total:.0f} %), strip width {a.width:.0f} mm."
    seq = lambda recs: ",".join("|".join(sg["name"] for sg in r["segments"]) for r in recs)     # noqa: E731  strokes by ',', segments of a stroke by '|'
    RDK.setParam("PaintSeq_main", seq(strokes_meta))
    for pn, recs in per_prog.items():
        RDK.setParam("PaintSeq_" + pn, seq(recs))
    RDK.setParam("PaintSeq", seq(strokes_meta))
    RDK.setParam("PaintCoverage", cov)
    with open(os.path.join(HERE, "paint_overlay.json"), "w") as f:
        json.dump({"strokes": strokes_meta, "programs": per_prog, "coverage": cov, "segment_mm": a.seg}, f, indent=1)
    print(cov)
    print(f"{len(strokes_meta)} strokes, {n_objects} segment objects created (hidden, under the frame 'PaintOverlay').")
    print("Next:  python3 gunon_tools.py mode overlay  (a whole stroke appears when it ends)   or   mode live + python3 live_paint.py  (paint follows the gun).")
    if n_missed:
        print(f"({n_missed} sampled targets had no shell surface within 600 mm in front of the gun and were left out.)")


def resequence(RDK, verbose=True):
    """Re-read the order of the programs in Auto_Main and rebuild the stroke sequence from the strokes already computed by `build` (no objects are
    created, takes a second). Link programs between the zones (Smart Motion etc.) have no strokes and are simply skipped."""
    path = os.path.join(HERE, "paint_overlay.json")
    try:
        meta = json.load(open(path))
    except Exception:
        raise SystemExit("paint_overlay.json not found: run `python3 paint_overlay.py build` first.")
    built = meta["programs"]
    order, strokes, notes = [], [], []
    for pn in called_programs(RDK):
        if pn in built:
            strokes += built[pn]
            order.append((pn, len(built[pn])))
        else:
            prog = RDK.Item(pn, robolink.ITEM_TYPE_PROGRAM)
            if strokes_of(prog):
                notes.append(f"'{pn}' paints but has no overlay pieces: run `python3 paint_overlay.py build` again")
    for pn in built:
        if pn not in [o[0] for o in order]:
            notes.append(f"'{pn}' is built but not called by Auto_Main any more (its paint will not appear)")
    seq = ",".join("|".join(sg["name"] for sg in r["segments"]) for r in strokes)
    RDK.setParam("PaintSeq_main", seq)
    RDK.setParam("PaintSeq", seq)
    meta["strokes"] = strokes
    with open(path, "w") as f:
        json.dump(meta, f, indent=1)
    if verbose:
        print(f"Auto_Main order now ({sum(n for _, n in order)} strokes):")
        for pn, n in order:
            print(f"   {pn}: {n} strokes")
        for note in notes:
            print("  NOTE:", note)
    return meta


def focus(RDK, which):
    key = "PaintSeq_main" if which == "main" else "PaintSeq_" + which
    seq = [s for s in str(RDK.getParam(key) or "").split(",") if s]
    if not seq:
        raise SystemExit(f"No overlay sequence stored for '{which}'. Run `build` first (and check the program name).")
    for stroke in [s for s in str(RDK.getParam("PaintSeq_main") or "").split(",") if s]:
        for name in stroke.split("|"):
            it = RDK.Item(name, robolink.ITEM_TYPE_OBJECT)
            if it.Valid():
                it.setVisible(False)
    RDK.setParam("PaintSeq", ",".join(seq))
    import GunOn as G
    cfg = G.read_cfg(RDK)
    cfg["n"], cfg["armed"] = "0", "0"
    G.write_cfg(RDK, cfg)
    print(f"The next run reveals the {len(seq)} strokes of '{which}', one per finished stroke.")


def clear(RDK):
    n = 0
    for fr in [it for it in RDK.ItemList(robolink.ITEM_TYPE_FRAME) if it.Name() == "PaintOverlay"]:
        fr.Delete()
    for it in [it for it in RDK.ItemList(robolink.ITEM_TYPE_OBJECT) if it.Name().startswith("Paint_")]:
        it.Delete()
        n += 1
    print(f"Deleted {n} overlay objects.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["build", "focus", "clear", "resequence"])
    ap.add_argument("which", nargs="?", default="main")
    ap.add_argument("--width", type=float, default=120.0, help="painted strip width in mm")
    ap.add_argument("--lift", type=float, default=3.0, help="how far the red surface sits above the shell, mm")
    ap.add_argument("--step", type=int, default=4, help="use every n-th target of a stroke")
    ap.add_argument("--seg", type=float, default=250.0, help="length in mm of the pieces a stroke is cut into (for live painting)")
    ap.add_argument("--subdivide", type=int, default=1, help="split the shell triangles in 4 this many times before picking the strip (0 = off; 1 = smoother edges)")
    a = ap.parse_args()
    RDK = robolink.Robolink()
    {"build": lambda: build(RDK, a), "focus": lambda: focus(RDK, a.which), "clear": lambda: clear(RDK), "resequence": lambda: resequence(RDK)}[a.cmd]()


if __name__ == "__main__":
    main()
