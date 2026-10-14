"""
extract_paint_surface.py - split a car model into FULL car (for collisions) and PAINT-ONLY skin (for paths).

Works on any GLB/glTF/OBJ/PLY whose parts carry names or materials (Sketchfab, game and CAD exports usually do).
A plain STL has no part information - see the notes at the bottom.

    python3 extract_paint_surface.py model.glb --list
    python3 extract_paint_surface.py model.glb --out car                       # guesses the painted parts
    python3 extract_paint_surface.py model.glb --out car --keep-mesh Body_Paint_Body_Paint_0

Writes car_full.stl and car_paint_only.stl (mm, Z-up, long axis X, front +X, centred, tyres on Z = 0),
both in the SAME frame, scaled to the brief's 4800 x 1900 x 1600 mm unless --uniform is given.

Default guess for "painted skin": a mesh whose material name contains 'paint' and whose mesh name does not start
with 'details' or 'wheel' (mirror caps, trim and wheel parts are left out). Always check the printed list.
In a real paint shop the body-in-white (shell + doors + hood + lid) is painted BEFORE glass, wheels, lights and
trim are fitted, so the body-only shell is also the realistic input, not a shortcut.
For a plain mesh without parts: select the painted faces in Blender or MeshLab (or the CAD tool) and export them.
"""
import argparse
import numpy as np
import trimesh

TARGET_MM = np.array([4800.0, 1900.0, 1600.0])
GROUND_WORDS = ("concrete", "ground", "floor", "plane", "road", "asphalt", "shadow")


def mat_name(geom):
    return (getattr(getattr(geom.visual, "material", None), "name", "") or "")


def load_parts(path):
    scene = trimesh.load(path)
    if isinstance(scene, trimesh.Trimesh):
        scene = trimesh.Scene(scene)
    parts = []
    for node in scene.graph.nodes_geometry:
        T, gname = scene.graph[node]
        g = scene.geometry[gname].copy()
        g.apply_transform(T)
        parts.append((gname, mat_name(g), g))
    return parts


def is_ground(name, mat, g):
    ext = g.extents                       # glTF is Y-up
    flat = ext[1] < 0.02 * max(ext[0], ext[2], 1e-9)
    return flat or any(w in (name + " " + mat).lower() for w in GROUND_WORDS) and ext[1] < 0.1 * max(ext[0], ext[2])


def guess_paint(name, mat):
    n = name.lower()
    return "paint" in mat.lower() and not n.startswith(("details", "wheel"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model")
    ap.add_argument("--list", action="store_true", help="only list the parts and what would be kept")
    ap.add_argument("--out", default="car", help="output prefix")
    ap.add_argument("--keep-mesh", nargs="+", help="exact mesh names to treat as paintable (overrides the guess)")
    ap.add_argument("--uniform", action="store_true", help="scale length to 4800 and keep proportions")
    a = ap.parse_args()

    parts = [p for p in load_parts(a.model) if not is_ground(*p)]
    keep = [(n, m, g) for n, m, g in parts if (n in a.keep_mesh if a.keep_mesh else guess_paint(n, m))]
    print(f"{'mesh':36s}{'material':22s}{'faces':>7s}  paint?")
    for n, m, g in parts:
        print(f"{n:36s}{m:22s}{len(g.faces):7d}  {'YES' if any(n == k[0] for k in keep) else '-'}")
    if a.list:
        return
    if not keep:
        raise SystemExit("No painted parts found - pass --keep-mesh <names> (see the list above).")

    full = trimesh.util.concatenate([g for _, _, g in parts])
    paint = trimesh.util.concatenate([g for _, _, g in keep])

    def both(op):
        op(full)
        op(paint)

    both(lambda m: m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])))  # Y-up -> Z-up
    both(lambda m: m.apply_scale(1000.0))                                                              # m -> mm
    if full.extents[1] > full.extents[0]:                                                              # long axis on X
        both(lambda m: m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 0, 1])))
    top = full.vertices[full.vertices[:, 2] > full.vertices[:, 2].min() + 0.85 * full.extents[2]]
    if top[:, 0].mean() - full.centroid[0] > 0:                                                        # cabin behind middle
        both(lambda m: m.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [0, 0, 1])))
    native = full.extents.copy()
    scale = TARGET_MM[0] / native[0] if a.uniform else TARGET_MM / native
    both(lambda m: m.apply_scale(scale))
    lo, hi = full.bounds
    shift = [-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]]
    both(lambda m: m.apply_translation(shift))

    full.export(f"{a.out}_full.stl")
    paint.export(f"{a.out}_paint_only.stl")
    print(f"\nfull : {len(full.faces)} faces, size {np.round(full.extents).tolist()} mm -> {a.out}_full.stl")
    print(f"paint: {len(paint.faces)} faces, size {np.round(paint.extents).tolist()} mm -> {a.out}_paint_only.stl")


if __name__ == "__main__":
    main()
