"""
make_zone_curves.py - turn the CAD paint zones into spray paths (zig-zag curves with normals), same convention as the
Surface Pattern Generator door that your working Path2 follows. Pure numpy, no RoboDK needed.

What it does for every zone (the patches from segment_car_patches.py -> car_zones.json):
  1. builds a projection frame: n = the zone's average outward normal, u = the zone's longest direction, v = across;
  2. lays a grid over the zone (row spacing --row-step, point spacing --point-step) and ray-casts every grid point onto
     the zone's own triangles along -n ("remove points not on surface");
  3. links the valid points into a boustrophedon zig-zag (rows alternate direction); a long gap starts a new curve;
  4. stores the NORMALS POINTING INTO THE CAR (the convention of the SPG door: path Z into the surface; your project's
     180-degree path-to-tool flip turns the gun to face the car), smoothed along each curve.

    python3 make_zone_curves.py --validate-door probe_curves_output.json     # reproduce YOUR door and compare, then exit
    python3 make_zone_curves.py                                              # all zones -> zone_curves.json + preview png
    python3 make_zone_curves.py --row-step 80 --point-step 50 --zones 8 13   # other spacing / only some zones

Needs paint_common.py, the paint-only STL and (for zones) car_zones.json in the same folder. matplotlib only for the preview.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import paint_common as pcm  # noqa: E402


def unit(v):
    v = np.asarray(v, float)
    return v / max(np.linalg.norm(v), 1e-12)


def axis_values(lo, hi, step):
    n = int(np.floor((hi - lo) / step + 1e-9))
    vals = lo + step * np.arange(n + 1)
    if hi - vals[-1] > 0.5 * step:
        vals = np.append(vals, hi)
    return vals


def grid_curves(mesh, c, n, u, a_vals, b_vals, normals="surface", max_dev_deg=65.0, smooth=5,
                jump_max=None, start_high=True, reach=3000.0):
    """Zig-zag curves over a grid in the plane (u, v) ray-cast along -n onto `mesh`.
    Returns a list of (points Nx3, normals Nx3 pointing INTO the surface)."""
    n, u = unit(n), unit(u)
    v = unit(np.cross(u, n))
    cos_dev = np.cos(np.radians(max_dev_deg))
    rows = []
    for j, b in enumerate(b_vals):
        row = []
        for a in a_vals:
            origin = c + a * u + b * v + reach * n
            hit = mesh.raycast(origin, -n, front_only=True)
            if hit is None or np.dot(hit[1], n) < cos_dev:
                row.append(None)
            else:
                row.append((origin - n * hit[0], hit[1]))
        rows.append(row)
    step = abs(a_vals[1] - a_vals[0]) if len(a_vals) > 1 else 50.0
    jump_max = jump_max or 2.6 * max(abs(b_vals[1] - b_vals[0]) if len(b_vals) > 1 else step, step)
    curves, cur = [], []

    def close():
        nonlocal cur
        if len(cur) >= 2:
            curves.append(cur)
        cur = []

    for j, row in enumerate(rows):
        runs, run = [], []
        for item in row:
            if item is None:
                if run:
                    runs.append(run)
                run = []
            else:
                run.append(item)
        if run:
            runs.append(run)
        forward = (j % 2 == 1) if start_high else (j % 2 == 0)      # start_high: the first row starts at the high-u end
        runs = [r if forward else r[::-1] for r in (runs if forward else runs[::-1])]
        for r in runs:
            if cur and np.linalg.norm(cur[-1][0] - r[0][0]) > jump_max:
                close()
            cur.extend(r)
    close()

    out = []
    for cv in curves:
        P = np.array([p for p, _ in cv])
        N = np.array([nn for _, nn in cv])
        if normals == "projection":
            N = np.tile(n, (len(P), 1))                                  # SPG door: the projection direction
        elif smooth and smooth > 1 and len(P) >= smooth:
            k = smooth // 2
            Ns = np.array([N[max(0, i - k):i + k + 1].mean(0) for i in range(len(N))])
            N = Ns / np.maximum(np.linalg.norm(Ns, axis=1, keepdims=True), 1e-12)
        out.append((P, -N))                                              # INTO the car, as in the SPG door
    return out


def zone_frame(mesh_tri, normal):
    """u = longest direction of the zone in the plane normal to `normal`; returns centre, u, a/b extents."""
    n = unit(normal)
    pts = mesh_tri.reshape(-1, 3)
    cen = pts.mean(0)
    ref = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = unit(np.cross(n, ref))
    e2 = unit(np.cross(n, e1))
    q = np.c_[(pts - cen) @ e1, (pts - cen) @ e2]
    w, V = np.linalg.eigh(np.cov(q.T))
    u = unit(V[0, 1] * e1 + V[1, 1] * e2)                                # eigenvector of the larger eigenvalue
    if u[0] < -1e-6 or (abs(u[0]) <= 1e-6 and u[1] < 0):
        u = -u
    v = unit(np.cross(u, n))
    a, b = (pts - cen) @ u, (pts - cen) @ v
    return cen, u, (a.min(), a.max()), (b.min(), b.max())


def path_len(P):
    return float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum()) if len(P) > 1 else 0.0


def validate_door(mesh, probe_json):
    spg = json.load(open(probe_json))["curves"][0]
    S = np.array(spg["points"])
    zs = sorted(set(np.round(S[:, 2], 2)))
    a_vals = np.arange(-700.0, 800.1, 50.0)
    b_vals = np.array(zs)
    cur = grid_curves(mesh, np.zeros(3), (0, 1, 0), (1, 0, 0), a_vals, b_vals, normals="projection", start_high=True)
    P = np.vstack([c[0] for c in cur])
    print(f"SPG door: {len(S)} points in 1 curve, rows at z = {zs[0]} .. {zs[-1]} step {zs[1] - zs[0]:.0f}")
    print(f"mine    : {len(P)} points in {len(cur)} curve(s)")
    if len(P) == len(S):
        d = np.linalg.norm(P - S, axis=1)
        print(f"same order, point by point: max distance {d.max():.2f} mm, median {np.median(d):.2f} mm "
              f"(y differs by up to {np.abs(P[:, 1] - S[:, 1]).max():.2f} mm)")
    else:
        from_spg = np.array([np.linalg.norm(P - s, axis=1).min() for s in S])
        print(f"different count; every SPG point has one of mine within {from_spg.max():.2f} mm (median {np.median(from_spg):.2f})")
    print(f"normals: mine {np.round(cur[0][1][0], 3)} vs SPG {np.round(spg['normals'][0], 3)}")
    cs = grid_curves(mesh, np.zeros(3), (0, 1, 0), (1, 0, 0), a_vals, b_vals, normals="surface", start_high=True)
    ang = np.degrees(np.arccos(np.clip(np.vstack([c[1] for c in cs]) @ np.array([0, -1.0, 0]), -1, 1)))
    print(f"true surface normals vs the SPG's constant (0,-1,0): mean {ang.mean():.1f} deg, max {ang.max():.1f} deg")
    print(f"path length: mine {sum(path_len(c[0]) for c in cur):.0f} mm vs SPG {path_len(S):.0f} mm")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl", default="car_paint_only_4800x1900x1600.stl")
    ap.add_argument("--zones", nargs="*", type=int, help="zone ids (default: all in car_zones.json)")
    ap.add_argument("--zones-json", default="car_zones.json")
    ap.add_argument("--row-step", type=float, default=50.0, help="distance between rows (mm); your door used 50")
    ap.add_argument("--point-step", type=float, default=50.0, help="distance between points along a row (mm)")
    ap.add_argument("--smooth", type=int, default=5)
    ap.add_argument("--speed", type=float, default=300.0, help="painting speed (mm/s) for the time estimate")
    ap.add_argument("--out", default=".")
    ap.add_argument("--validate-door", metavar="probe_curves_output.json")
    ap.add_argument("--no-preview", action="store_true")
    a = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    stl = a.stl if os.path.isfile(a.stl) else os.path.join(here, a.stl)
    mesh = pcm.load_mesh(stl)
    if a.validate_door:
        validate_door(mesh, a.validate_door)
        return

    zj = json.load(open(a.zones_json if os.path.isfile(a.zones_json) else os.path.join(here, a.zones_json)))
    tri_all = np.stack([mesh.v0, mesh.v0 + mesh.e1, mesh.v0 + mesh.e2], 1)           # (F,3,3)
    result, rows = [], []
    for z in zj["zones"]:
        if a.zones and z["id"] not in a.zones:
            continue
        idx = np.array(z["face_indices"])
        tri = tri_all[idx]
        sub = pcm.Mesh(tri.reshape(-1, 3), np.arange(len(idx) * 3).reshape(-1, 3))
        n = unit(z["mean_normal"])
        cen, u, (a0, a1), (b0, b1) = zone_frame(tri, n)
        curves = grid_curves(sub, cen, n, u, axis_values(a0, a1, a.point_step), axis_values(b0, b1, a.row_step),
                             normals="surface", smooth=a.smooth)
        npts = sum(len(c[0]) for c in curves)
        length = sum(path_len(c[0]) for c in curves)
        result.append({"id": z["id"], "label": z["label"], "area_m2": z["area_m2"], "mean_normal": z["mean_normal"],
                       "u": np.round(u, 4).tolist(),
                       "stats": {"curves": len(curves), "points": npts, "length_mm": round(length), "est_paint_s": round(length / a.speed)},
                       "curves": [{"points": np.round(P, 2).tolist(), "normals": np.round(N, 4).tolist()} for P, N in curves]})
        rows.append((z["id"], z["label"], z["area_m2"], len(curves), npts, length))
    print(f"{'id':>3} {'label':18s} {'m2':>5} {'curves':>6} {'points':>6} {'length m':>8} {'paint s @%g mm/s' % a.speed:>16}")
    for i, lab, ar, nc, npts, L in rows:
        print(f"{i:3d} {lab:18s} {ar:5.2f} {nc:6d} {npts:6d} {L / 1000:8.1f} {L / a.speed:16.0f}")
    print(f"{'':>3} {'TOTAL':18s} {sum(r[2] for r in rows):5.2f} {sum(r[3] for r in rows):6d} {sum(r[4] for r in rows):6d} "
          f"{sum(r[5] for r in rows) / 1000:8.1f} {sum(r[5] for r in rows) / a.speed:16.0f}")
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "zone_curves.json"), "w") as f:
        json.dump({"convention": "points on the car surface (car frame, mm); normals point INTO the car; one zig-zag curve per run",
                   "row_step": a.row_step, "point_step": a.point_step, "zones": result}, f)
    print("wrote", os.path.join(a.out, "zone_curves.json"))

    if not a.no_preview:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        cmap = plt.get_cmap("tab20")
        fig = plt.figure(figsize=(18, 6.2))
        # matplotlib 3D has no depth test between lines and surfaces, so draw only the curves (the car shape appears from them)
        for k, (title, el, az) in enumerate([("left side (+Y)", 0, 90), ("top", 90, -90), ("front-left oblique", 25, 35)]):
            ax = fig.add_subplot(1, 3, k + 1, projection="3d")
            for r in result:
                nz = r["mean_normal"]
                if (k == 0 and nz[1] < -0.2) or (k == 1 and nz[2] < 0.2) or (k == 2 and nz[1] < -0.2):
                    continue                                   # hide zones facing away from this camera
                for cv in r["curves"]:
                    P = np.array(cv["points"])
                    ax.plot(P[:, 0], P[:, 1], P[:, 2], color=cmap(r["id"] % 20), lw=0.9)
            ax.set_xlim(-2400, 2400); ax.set_ylim(-2400, 2400); ax.set_zlim(-800, 2400); ax.set_box_aspect((1, 1, 0.67))
            ax.view_init(el, az); ax.set_title(title); ax.axis("off")
        plt.tight_layout()
        png = os.path.join(a.out, "zone_curves_preview.png")
        plt.savefig(png, dpi=105)
        print("wrote", png)


if __name__ == "__main__":
    main()
