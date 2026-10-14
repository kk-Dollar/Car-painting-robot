"""
paint_common.py - zone list, mesh loader and stroke planning shared by the paint-path scripts.

Self-contained (numpy only): the car mesh is read from a binary STL, strokes are cast onto the real surface,
and each stroke point becomes a spray pose (STANDOFF off the surface, tool Z into it, tool X along the stroke).
gantry_paint_paths.py imports this module, so it does not depend on which version of generate_paint_paths.py
is in your folder. Extracted unchanged from generate_paint_paths.py.
"""
import math

import numpy as np

# ---------------------------------------------------------------- settings
ROBOT_NAME = "Fanuc P-200 LA HB"
TOOL_NAME = "Paint gun"
PLATFORM_KEY = "amr_platform"
CAR_KEY = "car_xr_sedan"
FRAME_NAME = "PP_frame"
PROGRAM_PREFIX = "PP_"
SPRAY_ON_ITEM, SPRAY_OFF_ITEM = "SprayOn", "SprayOff"     # old UR10-example scripts (fallback)
GUN_ITEM = "GunOn"          # spray simulation script from the ABB example, adapted: see GunOn.py
SPRAY_PER_ROW = False       # False: one GunOn on/off per zone (fewer spray objects); True: spray off between rows
# one colour per zone (R, G, B in 0..1) so the demo shows which surface was painted
ZONE_COLORS = {
    "left_door": (0.85, 0.10, 0.10), "right_door": (0.95, 0.45, 0.10),
    "roof_left": (0.15, 0.35, 0.85), "roof_right": (0.20, 0.60, 0.90),
    "hood_left": (0.10, 0.65, 0.25), "hood_right": (0.45, 0.80, 0.20),
    "trunk": (0.60, 0.20, 0.75), "front_face": (0.95, 0.80, 0.10), "rear_face": (0.90, 0.60, 0.10),
    "left_fender": (0.85, 0.20, 0.55), "right_fender": (0.95, 0.40, 0.65),
    "left_rear": (0.10, 0.65, 0.65), "right_rear": (0.30, 0.80, 0.75),
}

STANDOFF = 250.0        # mm, nozzle to surface
FAN_WIDTH = 200.0       # mm, spray fan width at the standoff (a guess - set it from the gun)
OVERLAP = 0.40          # 40 % stroke overlap  -> row pitch = 120 mm
STEP = 100.0            # mm between samples along a stroke
APPROACH_DIST = 150.0   # mm extra standoff for the approach and retract points
SPEED_LINEAR = 300.0    # mm/s while moving
MAX_JOINT_JUMP = 45.0   # deg between neighbouring points; bigger = wrist/elbow flip -> point dropped
MIN_NORMAL_COS = 0.5    # drop hits whose surface turns more than 60 deg away from the zone's normal
ORIGIN_OFFSET = 2500.0  # mm, rays start this far outside the car
CHECK_COLLISIONS = True
PAINT_FRONT_ONLY = True # ignore points where the ray enters an opening and hits the inside of the car

# Stations: name -> (x, y, yaw_deg) in CAR coordinates (mm, deg). yaw = direction the robot faces:
#   -90 = faces -Y (left-side stations look at the car), +90 = faces +Y, 180 = faces -X (front), 0 = faces +X (rear).
# Side stations face the car side straight on. (v1 turned them toward the car centre, which pushed the
# rear-quarter and fender targets past joint 1's +/-105 deg limit.)
STATIONS = {
    "Left": (-350.0, 2350.0, -90.0), "Right": (-350.0, -2350.0, 90.0),
    "LeftFwd": (1500.0, 2350.0, -90.0), "RightFwd": (1500.0, -2350.0, 90.0),
    "LeftAft": (-1400.0, 2350.0, -90.0), "RightAft": (-1400.0, -2350.0, 90.0),
    "LeftClose": (-300.0, 1900.0, -90.0), "RightClose": (-300.0, -1900.0, 90.0),
    "Front": (3800.0, 0.0, 180.0), "Rear": (-3800.0, 0.0, 0.0),
}

# --optimize: try small shifts of each zone's station and keep the one reaching the most points.
SEARCH_ALONG = (-300.0, 0.0, 300.0)     # mm, sideways along the car
SEARCH_DIST = (0.0, 150.0)              # mm, closer to the car (Close stations are not moved closer)
SEARCH_YAW = (-30.0, -15.0, 0.0, 15.0, 30.0)   # deg, turn the robot
PLAN_FILE = "station_plan.json"         # the chosen station pose per zone is saved here

# Zones in CAR coordinates (front = +X, left = +Y, up = +Z; tyres on Z = 0).
#   u = direction the strokes run, v = direction the rows step, normal = outward direction.
ZONES = [
    # --- zones tested so far: 100 % reachable in the last run ---
    dict(name="left_door", station="Left", normal=(0, 1, 0),
         u_axis=(1, 0, 0), u_range=(-900.0, 200.0),
         v_axis=(0, 0, 1), v_range=(550.0, 950.0)),
    dict(name="roof_left", station="LeftClose", normal=(0, 0, 1),
         u_axis=(1, 0, 0), u_range=(-900.0, 300.0),
         v_axis=(0, 1, 0), v_range=(0.0, 480.0)),
    dict(name="roof_right", station="RightClose", normal=(0, 0, 1),
         u_axis=(1, 0, 0), u_range=(-900.0, 300.0),
         v_axis=(0, 1, 0), v_range=(-480.0, 0.0)),
    # --- new zones (surface hits checked offline on the mesh; reach not tested yet) ---
    dict(name="right_door", station="Right", normal=(0, -1, 0),
         u_axis=(1, 0, 0), u_range=(-900.0, 200.0),
         v_axis=(0, 0, 1), v_range=(550.0, 950.0)),
    dict(name="hood_left", station="LeftFwd", normal=(0, 0, 1),
         u_axis=(1, 0, 0), u_range=(1200.0, 2000.0),
         v_axis=(0, 1, 0), v_range=(0.0, 600.0)),
    dict(name="hood_right", station="RightFwd", normal=(0, 0, 1),
         u_axis=(1, 0, 0), u_range=(1200.0, 2000.0),
         v_axis=(0, 1, 0), v_range=(-600.0, 0.0)),
    dict(name="trunk", station="Rear", normal=(0, 0, 1),            # between the rear window and the spoiler
         u_axis=(0, 1, 0), u_range=(-500.0, 500.0),
         v_axis=(1, 0, 0), v_range=(-2000.0, -1700.0)),
    dict(name="front_face", station="Front", normal=(1, 0, 0),
         u_axis=(0, 1, 0), u_range=(-600.0, 600.0),
         v_axis=(0, 0, 1), v_range=(420.0, 750.0)),
    dict(name="rear_face", station="Rear", normal=(-1, 0, 0),
         u_axis=(0, 1, 0), u_range=(-600.0, 600.0),
         v_axis=(0, 0, 1), v_range=(450.0, 750.0)),
    # fenders / rear quarters: strips ABOVE the wheel arches (the arches are open holes in this mesh)
    dict(name="left_fender", station="LeftFwd", normal=(0, 1, 0),
         u_axis=(1, 0, 0), u_range=(1000.0, 1700.0),
         v_axis=(0, 0, 1), v_range=(820.0, 960.0)),
    dict(name="right_fender", station="RightFwd", normal=(0, -1, 0),
         u_axis=(1, 0, 0), u_range=(1000.0, 1700.0),
         v_axis=(0, 0, 1), v_range=(820.0, 960.0)),
    dict(name="left_rear", station="LeftAft", normal=(0, 1, 0),
         u_axis=(1, 0, 0), u_range=(-1800.0, -1000.0),
         v_axis=(0, 0, 1), v_range=(820.0, 960.0)),
    dict(name="right_rear", station="RightAft", normal=(0, -1, 0),
         u_axis=(1, 0, 0), u_range=(-1800.0, -1000.0),
         v_axis=(0, 0, 1), v_range=(820.0, 960.0)),
]


# ================================================================ geometry (pure numpy)
def unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


class Mesh:
    """Triangle mesh with a simple vectorised ray cast (Moller-Trumbore)."""

    def __init__(self, vertices, faces):
        V, F = np.asarray(vertices, float), np.asarray(faces, int)
        self.v0 = V[F[:, 0]]
        self.e1 = V[F[:, 1]] - self.v0
        self.e2 = V[F[:, 2]] - self.v0
        n = np.cross(self.e1, self.e2)
        length = np.linalg.norm(n, axis=1)
        self.normals = n / np.maximum(length, 1e-12)[:, None]
        self.bounds = (V.min(axis=0), V.max(axis=0))

    def raycast(self, origin, direction, front_only=False):
        """Nearest hit along `direction`: returns (distance, outward-facing unit normal) or None.
        front_only=True returns None when the first thing hit is a BACK face, i.e. the ray went through an
        opening (window, wheel arch) and hit the inside of the car - those points are not paintable skin."""
        d = unit(direction)
        pvec = np.cross(d, self.e2)
        det = np.einsum("ij,ij->i", self.e1, pvec)
        ok = np.abs(det) > 1e-12
        inv = np.zeros_like(det)
        inv[ok] = 1.0 / det[ok]
        tvec = np.asarray(origin, float) - self.v0
        u = np.einsum("ij,ij->i", tvec, pvec) * inv
        qvec = np.cross(tvec, self.e1)
        v = (qvec @ d) * inv
        t = np.einsum("ij,ij->i", self.e2, qvec) * inv
        hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-6)
        if not hit.any():
            return None
        idx = np.flatnonzero(hit)
        k = idx[np.argmin(t[idx])]
        n = self.normals[k]
        if np.dot(n, d) > 0:
            if front_only:
                return None
            n = -n                # make the normal face the ray origin
        return float(t[k]), n


def load_mesh(path):
    """Read a binary STL with plain numpy (no extra packages). Falls back to trimesh for other files."""
    import os
    if not os.path.isfile(path):
        raise SystemExit(f"STL file not found: {path}\n"
                         f"Put the car STL next to this script or pass its path with --stl <path>.\n"
                         f"To locate it:  find ~ -name 'car_xr_sedan*.stl'")
    with open(path, "rb") as fh:
        data = fh.read()
    if len(data) >= 84:
        count = int(np.frombuffer(data[80:84], dtype="<u4")[0])
        if len(data) == 84 + 50 * count and count > 0:           # binary STL
            rec = np.dtype([("n", "<f4", 3), ("v", "<f4", 9), ("a", "<u2")])
            tri = np.frombuffer(data, dtype=rec, count=count, offset=84)["v"].astype(float).reshape(-1, 3)
            faces = np.arange(len(tri)).reshape(-1, 3)
            return Mesh(tri, faces)
    import trimesh                                                # ASCII STL, OBJ, ...
    m = trimesh.load(path, force="mesh")
    return Mesh(m.vertices, m.faces)


def smooth_normals(normals, window=5):
    """Moving average of unit normals along a stroke (edge-padded), re-normalised."""
    N = np.asarray(normals, float)
    if len(N) < 3:
        return N
    half = window // 2
    padded = np.vstack([np.repeat(N[:1], half, 0), N, np.repeat(N[-1:], half, 0)])
    kernel = np.ones(window) / window
    out = np.stack([np.convolve(padded[:, i], kernel, mode="valid") for i in range(3)], axis=1)
    return out / np.maximum(np.linalg.norm(out, axis=1), 1e-12)[:, None]


def spray_pose(position, normal, stroke_dir, standoff):
    """4x4 pose (car coords): `standoff` off the surface, tool Z into the surface, tool X along the stroke."""
    n = unit(normal)
    z = -n
    x = np.asarray(stroke_dir, float)
    x = x - np.dot(x, z) * z
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1.0, 0.0, 0.0]) - z[0] * z
    x = unit(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z
    T[:3, 3] = np.asarray(position, float) + n * standoff
    return T


def plan_zone(mesh, zone):
    """Strokes for one zone. Returns a list of rows; each row is a list of dicts
    {pos, normal, dir, pose, u, v} ordered along the stroke (rows alternate direction)."""
    U, V, N = unit(zone["u_axis"]), unit(zone["v_axis"]), unit(zone["normal"])
    pitch = FAN_WIDTH * (1.0 - OVERLAP)
    v0, v1 = zone["v_range"]
    v_vals = list(np.arange(v0, v1 + 1e-6, pitch))
    if v_vals[-1] < v1 - 1.0:
        v_vals.append(v1)
    u0, u1 = zone["u_range"]
    u_vals = list(np.arange(u0, u1 + 1e-6, STEP))
    rows = []
    for k, v in enumerate(v_vals):
        us = u_vals if k % 2 == 0 else u_vals[::-1]
        stroke_dir = U if k % 2 == 0 else -U
        raw = []
        for u in us:
            origin = u * U + v * V + N * ORIGIN_OFFSET
            hit = mesh.raycast(origin, -N, front_only=PAINT_FRONT_ONLY)
            if hit is None:
                continue
            t, n = hit
            if np.dot(n, N) < MIN_NORMAL_COS:
                continue
            raw.append((u, v, origin - N * t, n))
        if not raw:
            rows.append([])
            continue
        smooth = smooth_normals([r[3] for r in raw])
        row = []
        for (u, vv, p, _), n in zip(raw, smooth):
            # tool X stays +U on EVERY row: only the travel direction alternates. Flipping tool X by
            # 180 deg between rows made the wrist roll 180 deg at each row change (v1 dropped those rows).
            row.append(dict(u=u, v=vv, pos=p, normal=n, dir=U,
                            pose=spray_pose(p, n, U, STANDOFF)))
        rows.append(row)
    return rows


