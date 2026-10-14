"""
group_zones.py - merge the 20 fine CAD patches into a few big paint zones (prototype level: fewer, larger surfaces).

Rules (car frame: front +X, left +Y, up +Z):
  top-facing  (normal z >= 0.6):  hood (x > 600) | trunk (x < -1300) | roof (in between)
  front / rear (|normal x| >= 0.6): front | rear
  side-facing (|normal y| >= 0.6): left_/right_ + doors (below z 900) or rail (the roof-rail / pillar strip above)
Inside a zone the curves are re-ordered greedily (nearest end first) so the gun moves from one stroke to the next without long jumps,
and slivers (< 4 points) are dropped. Every grouped zone keeps its original patches under "parts": build_zones.py falls back to them
when a big zone cannot be solved in one go.

    python3 group_zones.py                 # zone_curves.json + car_zones.json -> zone_curves_grouped.json (+ preview png)
    python3 build_zones.py --zones-json zone_curves_grouped.json --main --split 3
"""
import argparse
import json
import os

import numpy as np


def group_of(n, c):
    nx, ny, nz = n
    if nx >= 0.6:
        return "front"
    if nx <= -0.6:
        return "rear"
    if nz >= 0.6:
        return "hood" if c[0] > 600 else "trunk" if c[0] < -1300 else "roof"
    if abs(ny) >= 0.6:
        return ("left_" if c[1] > 0 else "right_") + ("rail" if c[2] > 900 else "doors")
    return "other"


def length(P):
    P = np.asarray(P)
    return float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum()) if len(P) > 1 else 0.0


def order_curves(curves):
    """Greedy nearest-end ordering; curves may be reversed. Start at the end that is lowest and furthest forward."""
    cs = [(np.array(c["points"]), np.array(c["normals"])) for c in curves]
    ends = lambda P: (P[0], P[-1])      # noqa: E731
    start_key = lambda p: p[2] - 0.2 * p[0]   # noqa: E731
    best = min(((start_key(e), i, k) for i, (P, _) in enumerate(cs) for k, e in enumerate(ends(P))), key=lambda t: t[0])
    used, out = set(), []
    i, k = best[1], best[2]
    while True:
        P, N = cs[i]
        if k == 1:
            P, N = P[::-1], N[::-1]
        out.append({"points": np.round(P, 2).tolist(), "normals": np.round(N, 4).tolist()})
        used.add(i)
        if len(used) == len(cs):
            return out
        cur = P[-1]
        cand = [(np.linalg.norm(cur - e), j, kk) for j, (Q, _) in enumerate(cs) if j not in used for kk, e in enumerate(ends(Q))]
        _, i, k = min(cand)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zones", default="zone_curves.json")
    ap.add_argument("--info", default="car_zones.json")
    ap.add_argument("--out", default="zone_curves_grouped.json")
    ap.add_argument("--min-points", type=int, default=4, help="drop curves with fewer points (slivers)")
    ap.add_argument("--no-preview", action="store_true")
    a = ap.parse_args()
    here = os.path.dirname(os.path.abspath(__file__))
    zc = json.load(open(a.zones if os.path.isfile(a.zones) else os.path.join(here, a.zones)))
    info = {z["id"]: z for z in json.load(open(a.info if os.path.isfile(a.info) else os.path.join(here, a.info)))["zones"]}

    groups = {}
    for z in zc["zones"]:
        groups.setdefault(group_of(z["mean_normal"], info[z["id"]]["centre_mm"]), []).append(z)
    order = ["hood", "roof", "trunk", "front", "rear", "left_doors", "right_doors", "left_rail", "right_rail", "other"]
    out_zones, rows = [], []
    for gi, name in enumerate([g for g in order if g in groups]):
        members = groups[name]
        curves = [c for z in members for c in z["curves"] if len(c["points"]) >= a.min_points]
        area = sum(z["area_m2"] for z in members)
        nrm = np.sum([np.array(z["mean_normal"]) * z["area_m2"] for z in members], axis=0)
        nrm = (nrm / max(np.linalg.norm(nrm), 1e-9)).round(4).tolist()
        ordered = order_curves(curves)
        parts = [{"id": z["id"], "label": z["label"], "area_m2": z["area_m2"], "mean_normal": z["mean_normal"],
                  "curves": [c for c in z["curves"] if len(c["points"]) >= a.min_points]} for z in members]
        L = sum(length(c["points"]) for c in ordered)
        out_zones.append({"id": gi, "label": name, "area_m2": round(area, 3), "mean_normal": nrm, "members": [z["id"] for z in members],
                          "stats": {"curves": len(ordered), "points": sum(len(c["points"]) for c in ordered), "length_mm": round(L)},
                          "curves": ordered, "parts": parts})
        rows.append((gi, name, [z["id"] for z in members], area, len(ordered), sum(len(c["points"]) for c in ordered), L))
    with open(os.path.join(os.path.dirname(os.path.abspath(a.out)) if os.path.dirname(a.out) else here, os.path.basename(a.out)), "w") as f:
        json.dump({"convention": zc.get("convention"), "row_step": zc.get("row_step"), "point_step": zc.get("point_step"),
                   "grouped_from": a.zones, "zones": out_zones}, f)
    print(f"{'id':>2} {'zone':12s} {'from patches':24s} {'m2':>5s} {'curves':>6s} {'points':>6s} {'path m':>7s}")
    for gi, name, mem, ar, nc, npt, L in rows:
        print(f"{gi:2d} {name:12s} {str(mem):24s} {ar:5.2f} {nc:6d} {npt:6d} {L / 1000:7.1f}")
    print(f"{'':>2} {'TOTAL':12s} {'':24s} {sum(r[3] for r in rows):5.2f} {sum(r[4] for r in rows):6d} {sum(r[5] for r in rows):6d} {sum(r[6] for r in rows) / 1000:7.1f}")
    print("wrote", a.out)

    if not a.no_preview:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        cmap = plt.get_cmap("tab10")
        fig = plt.figure(figsize=(18, 6.2))
        for k, (title, el, az) in enumerate([("left side (+Y)", 0, 90), ("top", 90, -90), ("front-left oblique", 25, 35)]):
            ax = fig.add_subplot(1, 3, k + 1, projection="3d")
            for z in out_zones:
                n = z["mean_normal"]
                if (k != 1 and n[1] < -0.2) or (k == 1 and n[2] < 0.2):
                    continue
                for c in z["curves"]:
                    P = np.array(c["points"])
                    ax.plot(P[:, 0], P[:, 1], P[:, 2], color=cmap(z["id"] % 10), lw=0.9)
            ax.set_xlim(-2400, 2400); ax.set_ylim(-2400, 2400); ax.set_zlim(-800, 2400); ax.set_box_aspect((1, 1, 0.67))
            ax.view_init(el, az); ax.set_title(title); ax.axis("off")
        plt.tight_layout()
        png = os.path.splitext(a.out)[0] + "_preview.png"
        plt.savefig(png, dpi=105)
        print("wrote", png)


if __name__ == "__main__":
    main()
