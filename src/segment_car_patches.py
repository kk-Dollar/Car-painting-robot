"""
segment_car_patches.py - split the car's paint-only shell into smooth patches (candidate paint zones) from the CAD geometry.

Idea (from Chen et al., "CAD-based automated robot trajectory planning for spray painting of free-form surfaces", 2002):
neighbouring triangles belong to the same patch while the angle between their normals stays below a threshold, so each
patch is smooth enough to be painted with one gun direction. No hand-drawn rectangles.

    python3 segment_car_patches.py car_paint_only_4800x1900x1600.stl            # defaults: 30 deg, patches > 0.15 m2
    python3 segment_car_patches.py car_paint_only_4800x1900x1600.stl --angle 25 --min-area 0.1 --patch-stls

Outputs (in --out, default "."): cad_patches_<angle>deg.png (3 views, patch ids), car_zones.json (id, auto label, area,
centre, mean normal, bounding box, face indices) and, with --patch-stls, one STL per patch.
Car frame: front +X, left +Y, up +Z, tyres at Z=0 (the frame of our STL). The labels are rule-based guesses - check them.
Needs: pip install trimesh numpy matplotlib
"""
import argparse
import json
import os

import numpy as np
import trimesh


def label_for(centre, normal, k):
    """Very simple auto label from position and normal in the car frame (a guess to be confirmed by eye)."""
    nx, ny, nz = normal
    side = "left" if centre[1] > 300 else "right" if centre[1] < -300 else "centre"
    if nz > 0.6:
        kind = "top"
    elif abs(ny) >= max(abs(nx), 0.5):
        kind = "side"
    elif nx > 0.5:
        kind = "front"
    elif nx < -0.5:
        kind = "rear"
    else:
        kind = "mixed"
    return f"{side}_{kind}_{k}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stl")
    ap.add_argument("--angle", type=float, default=30.0, help="max angle between neighbouring normals inside a patch (deg)")
    ap.add_argument("--min-area", type=float, default=0.15, help="smallest patch kept as a zone (m2)")
    ap.add_argument("--out", default=".")
    ap.add_argument("--patch-stls", action="store_true", help="also write one STL per patch")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    m = trimesh.load(a.stl, process=False)
    m.merge_vertices(digits_vertex=1)
    keep = m.face_adjacency_angles < np.radians(a.angle)
    comps = trimesh.graph.connected_components(m.face_adjacency[keep], nodes=np.arange(len(m.faces)), min_len=1)
    comps = sorted(comps, key=lambda c: -m.area_faces[c].sum())
    big = [c for c in comps if m.area_faces[c].sum() / 1e6 >= a.min_area]
    covered = sum(m.area_faces[c].sum() for c in big) / m.area
    print(f"{len(comps)} patches at {a.angle:g} deg; {len(big)} larger than {a.min_area} m2 cover {covered * 100:.0f}% of the paint area")

    zones, kinds = [], {}
    for i, c in enumerate(big):
        tri = m.triangles[c].reshape(-1, 3)
        centre = m.triangles_center[c].mean(0)
        normal = (m.face_normals[c] * m.area_faces[c][:, None]).sum(0)
        normal /= np.linalg.norm(normal)
        kind_key = label_for(centre, normal, 0).rsplit("_", 1)[0]
        kinds[kind_key] = kinds.get(kind_key, 0) + 1
        zones.append({
            "id": i, "label": f"{kind_key}_{kinds[kind_key]}", "faces": int(len(c)),
            "area_m2": round(float(m.area_faces[c].sum() / 1e6), 3),
            "centre_mm": np.round(centre, 1).tolist(), "mean_normal": np.round(normal, 3).tolist(),
            "bbox_min_mm": np.round(tri.min(0), 1).tolist(), "bbox_max_mm": np.round(tri.max(0), 1).tolist(),
            "face_indices": [int(x) for x in c],
        })
        if a.patch_stls:
            m.submesh([c], append=True).export(os.path.join(a.out, f"patch_{i:02d}_{zones[-1]['label']}.stl"))
    with open(os.path.join(a.out, "car_zones.json"), "w") as f:
        json.dump({"source_stl": os.path.basename(a.stl), "angle_deg": a.angle, "min_area_m2": a.min_area,
                   "frame": "car: front +X, left +Y, up +Z, mm", "zones": zones}, f, indent=1)
    for z in zones:
        print(f"#{z['id']:2d} {z['label']:18s} {z['area_m2']:5.2f} m2  centre {z['centre_mm']}  normal {z['mean_normal']}")

    # picture: left side, top and an oblique view, patch ids at the patch centres
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    lab = np.full(len(m.faces), -1)
    for i, c in enumerate(big):
        lab[c] = i
    cmap = plt.get_cmap("tab20")
    cols = np.array([cmap(l % 20) if l >= 0 else (0.8, 0.8, 0.8, 1) for l in lab])
    fig = plt.figure(figsize=(18, 6.2))
    for k, (title, el, az) in enumerate([("left side (+Y)", 0, 90), ("top", 90, -90), ("front-left oblique", 25, 35)]):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(m.vertices[m.faces], facecolors=cols, edgecolor="none"))
        ax.set_xlim(-2400, 2400); ax.set_ylim(-2400, 2400); ax.set_zlim(-800, 2400); ax.set_box_aspect((1, 1, 0.67))
        ax.view_init(el, az); ax.set_title(title); ax.axis("off")
        if k < 2:
            for z in zones:
                n = z["mean_normal"]
                if (n[1] > 0.25) if k == 0 else (n[2] > 0.25):
                    ax.text(*z["centre_mm"], str(z["id"]), fontsize=11, weight="bold", ha="center")
    plt.tight_layout()
    png = os.path.join(a.out, f"cad_patches_{a.angle:g}deg.png")
    plt.savefig(png, dpi=110)
    print("wrote", png, "and", os.path.join(a.out, "car_zones.json"))


if __name__ == "__main__":
    main()
