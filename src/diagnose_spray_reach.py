#!/usr/bin/env python3
"""diagnose_spray_reach.py - does the spray actually REACH the car along the program?   (v3: measures with the car's own STL files)

RoboDK's ProjectPoints call turned out to answer "distance 0" for everything, so this version does not use it. It asks RoboDK only for the
nozzle's pose at each target (robot.setJoints + the Paint gun's absolute pose) and shoots a ray along the gun's Z axis - the direction the
spray goes - at the car's triangles itself (numpy, Moller-Trumbore), for BOTH car meshes:
    car_paint_only_4800x1900x1600.stl   (the paint shell the paths were built on)
    car_xr_sedan_4800x1900x1600.stl     (the full car; GunOn paints this one first)
The STL files must be next to this script (or pass --shell / --full). The spray volume ends at standoff + reach (150 + 250 = 400 mm now, 210 mm before).

  python3 diagnose_spray_reach.py                          hood
  python3 diagnose_spray_reach.py Auto_07_left_rail_p1     left rail, part 1
  python3 diagnose_spray_reach.py Auto_00_hood 2           every 2nd target (default 3)

v4: the nozzle tip is taken TWO ways - (A) the Paint gun item's pose, (B) RoboDK's own TCP pose (robot.Pose() with the Car Frame as reference) - because the
two may differ by the gun's length. Whichever origin reaches the paint shell at about 150 mm along a tool axis is the right one.
"""
import argparse
import os
import sys
import numpy as np
from robodk import robolink

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
FAR_OLD, FAR_NEW = 210.0, 400.0


def to_np(m):
    return np.array([[float(v) for v in row] for row in m.rows], float)


def load_triangles(path):
    import paint_common as pc
    mesh = pc.load_mesh(path)
    return np.stack([mesh.v0, mesh.v0 + mesh.e1, mesh.v0 + mesh.e2], axis=1)     # T x 3 x 3


def ray_hits(tris, origin, direction):
    """Moller-Trumbore against all triangles. Returns (nearest distance in FRONT of the origin or None, nearest distance BEHIND or None)."""
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
    hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1)
    front = t[hit & (t > 1e-6)]
    back = t[hit & (t < -1e-6)]
    return (float(front.min()) if front.size else None), (float(-back.max()) if back.size else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("program", nargs="?", default="Auto_00_hood")
    ap.add_argument("step", nargs="?", type=int, default=3)
    ap.add_argument("--shell", default=os.path.join(HERE, "car_paint_only_4800x1900x1600.stl"))
    ap.add_argument("--full", default=os.path.join(HERE, "car_xr_sedan_4800x1900x1600.stl"))
    a = ap.parse_args()

    RDK = robolink.Robolink()
    prog = RDK.Item(a.program, robolink.ITEM_TYPE_PROGRAM)
    tool = RDK.Item("Paint gun", robolink.ITEM_TYPE_TOOL)
    robots = RDK.ItemList(robolink.ITEM_TYPE_ROBOT)
    robot = next((r for r in robots if "ur10" in r.Name().lower()), robots[0] if robots else None)
    items = {}
    for it in RDK.ItemList(robolink.ITEM_TYPE_OBJECT):
        n = it.Name().lower()
        if "car_paint_only" in n:
            items["paint shell"] = (it, a.shell)
        elif "car_xr_sedan" in n and "collision" not in n:
            items["full car"] = (it, a.full)
    if not (prog.Valid() and tool.Valid() and robot is not None and items):
        raise SystemExit("Could not find the program, the Paint gun, the robot or the car objects in the station.")
    meshes = {}
    for k, (it, path) in items.items():
        if os.path.isfile(path):
            meshes[k] = (it, load_triangles(path))
        else:
            print(f"  ({k}: STL file not found: {path})")
    if not meshes:
        raise SystemExit("No car STL found next to this script. Copy car_paint_only_4800x1900x1600.stl and car_xr_sedan_4800x1900x1600.stl here, or use --shell / --full.")

    carf = RDK.Item("Car Frame", robolink.ITEM_TYPE_FRAME)
    original = list(robot.Joints().list())
    prev_frame = robot.getLink(robolink.ITEM_TYPE_FRAME)
    robot.setPoseTool(tool)                          # items, never matrices (a matrix would move the active frame item, i.e. the car)
    robot.setPoseFrame(carf)                         # so that robot.Pose() is the TCP expressed in the Car Frame
    car_abs = to_np(carf.PoseAbs())
    inv_obj = {k: np.linalg.inv(to_np(it.PoseAbs())) for k, (it, _) in meshes.items()}
    samples, on, n_moves = [], False, 0
    for i in range(prog.InstructionCount()):
        ins = prog.Instruction(i)
        nm = (ins[0] or "").lower().replace(" ", "")
        if "gunon(" in nm:
            on = nm.split("gunon(")[1][:2].startswith("1")
            continue
        if ins[1] != getattr(robolink, "INS_TYPE_MOVE", 0):
            continue
        n_moves += 1
        if n_moves % a.step:
            continue
        j = ins[5]
        q = [float(v) for v in (j.list() if hasattr(j, "list") else j)]
        if len(q) < 6:
            continue
        robot.setJoints(q)
        T_item = to_np(tool.PoseAbs())                                 # (A) the tool item's own pose
        T_tcp = car_abs @ to_np(robot.Pose())                          # (B) RoboDK's TCP pose, brought to absolute coordinates
        samples.append((i, on, T_item, T_tcp))
    robot.setJoints(original)
    try:
        if prev_frame.Valid():
            robot.setPoseFrame(prev_frame)
    except Exception:
        pass

    painting = [x for x in samples if x[1]]
    print(f"{a.program}: {n_moves} moves, {len(samples)} sampled, {len(painting)} of them with the gun ON")
    if not painting:
        raise SystemExit("No sampled target has the gun ON.")
    it_shell, tris = meshes.get("paint shell", next(iter(meshes.values())))
    inv_shell = np.linalg.inv(to_np(it_shell.PoseAbs()))
    lo, hi = tris.reshape(-1, 3).min(axis=0), tris.reshape(-1, 3).max(axis=0)
    print(f"  the STL's bounding box (object coordinates, mm): x {lo[0]:.0f}..{hi[0]:.0f}, y {lo[1]:.0f}..{hi[1]:.0f}, z {lo[2]:.0f}..{hi[2]:.0f}")
    d0 = np.array([np.linalg.norm((x[3][:3, 3] - x[2][:3, 3])) for x in painting])
    off = np.median([(np.linalg.inv(x[2]) @ x[3])[:3, 3] for x in painting], axis=0)
    print(f"  (A) tool item pose vs (B) RoboDK's TCP pose: they differ by {np.median(d0):.0f} mm; B in A's coordinates = ({off[0]:.0f}, {off[1]:.0f}, {off[2]:.0f}) mm")
    verts = tris.reshape(-1, 3)
    axes = {"+X": (1, 0, 0), "-X": (-1, 0, 0), "+Y": (0, 1, 0), "-Y": (0, -1, 0), "+Z": (0, 0, 1), "-Z": (0, 0, -1)}
    best_overall = None
    for label, idx in (("A  tool item pose", 2), ("B  RoboDK TCP pose", 3)):
        table = {}
        near = []
        for x in painting:
            To = inv_shell @ x[idx]
            near.append(float(np.linalg.norm(verts - To[:3, 3], axis=1).min()))
        for nm, vec in axes.items():
            ds = [ray_hits(tris, (inv_shell @ x[idx])[:3, 3], (inv_shell @ x[idx])[:3, :3] @ np.array(vec, float))[0] for x in painting]
            have = np.array([v for v in ds if v is not None])
            table[nm] = (have.size, float(np.median(have)) if have.size else None, int(((have > 90) & (have < 260)).sum()) if have.size else 0)
        print(f"  origin {label}: nearest shell vertex is {np.median(near):.0f} mm away (median)")
        for nm, (n_hit, med, n_ok) in table.items():
            if n_hit:
                print(f"      along tool {nm}: hits at {n_hit:3d} of {len(painting)}, median {med:.0f} mm, {n_ok} within 90-260 mm")
        if not any(v[0] for v in table.values()):
            print("      (no tool axis hits the shell from this origin)")
        best = max(table, key=lambda k: table[k][2])
        if table[best][2] >= 0.5 * len(painting) and (best_overall is None or table[best][2] > best_overall[2]):
            best_overall = (label, best, table[best][2])
    if best_overall:
        print(f"  --> From origin {best_overall[0]}, the spray reaches the shell at about 150 mm along the tool's {best_overall[1]} axis.")
    else:
        print("  --> From neither origin does any tool axis reach the shell at about 150 mm. Send me this whole output.")
    print("  first sample, in the STL's coordinates:")
    x = painting[0]
    for label, idx in (("A", 2), ("B", 3)):
        To = inv_shell @ x[idx]
        print(f"      {label}: position ({To[0,3]:.0f}, {To[1,3]:.0f}, {To[2,3]:.0f}), tool Z axis points ({To[0,2]:.2f}, {To[1,2]:.2f}, {To[2,2]:.2f})")


if __name__ == "__main__":
    main()
