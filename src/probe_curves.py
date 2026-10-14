"""
probe_curves.py - READ-ONLY: how did the Surface Pattern Generator store the door paths (the ones your working Path2 follows)?

Before the automation generates paths for the other zones, they must use the SAME convention as the working door paths:
which way the normals point, how many points per stroke, the stroke spacing, whether the strokes are separate curves or one
zig-zag. This script reads the curves of the path object (default "Frame 4 (SPG)"), converts them to the car frame, and prints
a summary. With --stl it also checks, with rays on the paint-only car mesh, whether the normals point OUT of the car surface.

    python3 probe_curves.py
    python3 probe_curves.py --stl car_paint_only_4800x1900x1600.stl

Writes probe_curves_output.json (the curves in car coordinates, 2 decimals) next to the script. Changes nothing in RoboDK.
"""
import argparse
import json
import os

import numpy as np

try:
    from robodk import robolink
except ImportError:
    import robolink


def to_np(m):
    return np.array([list(r) for r in m.rows], float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--object", default="Frame 4 (SPG)")
    ap.add_argument("--frame", default="Car Frame", help="express the points in this frame")
    ap.add_argument("--stl", help="paint-only car STL (car frame) for the outward-normal check")
    a = ap.parse_args()

    RDK = robolink.Robolink()
    obj = RDK.Item(a.object, robolink.ITEM_TYPE_OBJECT)
    frame = RDK.Item(a.frame, robolink.ITEM_TYPE_FRAME)
    if not obj.Valid() or not frame.Valid():
        raise SystemExit(f'Object "{a.object}" or frame "{a.frame}" not found.')
    T = np.linalg.inv(to_np(frame.PoseAbs())) @ to_np(obj.PoseAbs())

    curves = []
    for i in range(1000):
        try:
            pts, name = obj.GetPoints(robolink.FEATURE_CURVE, i)
        except Exception:
            break
        arr = np.array(pts, float)
        if arr.ndim != 2 or arr.shape[0] == 0:
            break
        P = (T @ np.c_[arr[:, :3], np.ones(len(arr))].T).T[:, :3]
        N = (T[:3, :3] @ arr[:, 3:6].T).T if arr.shape[1] >= 6 else np.zeros_like(P)
        curves.append((P, N))
    print(f'Object "{a.object}": {len(curves)} curve(s), expressed in "{a.frame}".')
    if not curves:
        raise SystemExit("No curves found (the paths may be stored as points: tell me and I will adapt the probe).")

    allP = np.vstack([c[0] for c in curves])
    allN = np.vstack([c[1] for c in curves])
    print(f"Total points: {len(allP)}   bounding box min {np.round(allP.min(0))} max {np.round(allP.max(0))} (mm)")
    steps = np.concatenate([np.linalg.norm(np.diff(c[0], axis=0), axis=1) for c in curves if len(c[0]) > 1])
    print(f"Distance between consecutive points: mean {steps.mean():.1f} mm, min {steps.min():.1f}, max {steps.max():.1f}")
    for k, (P, N) in enumerate(curves[:12]):
        d = P[-1] - P[0]
        print(f"  curve {k:2d}: {len(P):3d} points, length {np.linalg.norm(np.diff(P, axis=0), axis=1).sum():6.0f} mm, "
              f"start {np.round(P[0])}, direction {np.round(d / (np.linalg.norm(d) + 1e-9), 2)}, mean normal {np.round(N.mean(0), 2)}")
    if len(curves) == 1:       # one long curve: count the direction reversals (zig-zag rows)
        st = np.diff(curves[0][0], axis=0)
        st = st / (np.linalg.norm(st, axis=1, keepdims=True) + 1e-9)
        flips = int((np.sum(st[1:] * st[:-1], axis=1) < 0.0).sum())
        print(f"  a single curve with {flips} sharp turns (about {flips // 2 + 1} rows if it is a zig-zag)")
    elif len(curves) > 1:
        mids = np.array([c[0].mean(0) for c in curves])
        print(f"Spacing between consecutive curve centres: mean {np.linalg.norm(np.diff(mids, axis=0), axis=1).mean():.1f} mm")

    out = {"object": a.object, "frame": a.frame, "n_curves": len(curves),
           "curves": [{"points": np.round(P, 2).tolist(), "normals": np.round(N, 3).tolist()} for P, N in curves]}

    if a.stl:
        import sys
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import paint_common as pcm                   # numpy-only mesh + ray casting (same folder)
        mesh = pcm.load_mesh(a.stl)
        idx = np.linspace(0, len(allP) - 1, min(200, len(allP))).astype(int)
        n = allN[idx] / (np.linalg.norm(allN[idx], axis=1, keepdims=True) + 1e-9)
        outward, gaps = 0, []
        for p_, n_ in zip(allP[idx], n):
            # start 3 mm out along the normal and look back at the car: a front face about 3 mm away means "normal points OUT"
            hit = mesh.raycast(p_ + 3.0 * n_, -n_, front_only=True)
            if hit is not None and 0.5 < hit[0] < 12.0:
                outward += 1
                gaps.append(abs(hit[0] - 3.0))
        print(f"\nNormal check on {len(idx)} sampled points: {outward} have the car's outer surface 0.5-12 mm behind them along -normal "
              f"(normal pointing OUT of the car); the rest do not (inward, or the point is not on this mesh). "
              f"Typical distance of the points from the surface: {np.median(gaps) if gaps else float('nan'):.1f} mm.")
        out["normals_outward_of_sampled"] = [outward, len(idx)]

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "probe_curves_output.json")
    with open(path, "w") as f:
        json.dump(out, f)
    print("\nwrote", path)


if __name__ == "__main__":
    main()
