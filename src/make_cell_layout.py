#!/usr/bin/env python3
"""make_cell_layout.py - build the painting-cell building from the assessment's reference layout (12000 x 8000 mm) and put it into the RoboDK station.

The reference plan (section 4 of the assessment PDF): a big hall with a central 8000 x 5000 mm painting bay, a 2000 mm entry zone on the left (car comes in from
the left) and a 2000 mm exit zone on the right, an opening in the left and right wall for the car, a row of three rooms (2500 mm deep) along the top with a door
into the hall each, and columns along the walls. The PDF says the image is "only a general reference", so the bay is ENLARGED when the gantry does not fit.

Everything is generated here (numpy only) as a few STL meshes:
    Layout_Floor      slab, its top 50 mm BELOW the floor level z = 0, so nothing touches it
    Layout_Walls      outer walls with the car openings, the partition wall with 3 doors, the room dividers (semi-transparent)
    Layout_Columns    400 mm columns on the wall lines
    Layout_Markings   yellow outline of the painting bay and a centre line
    Layout_Arrows     green arrows: the car goes in on the left and out on the right
and a top-view drawing  layout_plan.png  (for the report).

  python3 make_cell_layout.py --measure              RoboDK must be running: moves the gantry to its axis limits, prints how big its working area is and the command to use
  python3 make_cell_layout.py                        reference dimensions, writes the STL files + layout_plan.png into ./cell_layout (no RoboDK needed)
  python3 make_cell_layout.py --fit --import         fit the bay around the gantry (measured), write the files AND add them to the station under the frame "Cell Layout"
  python3 make_cell_layout.py --bay-x 12000 --bay-y 9000 --import        or give the bay size yourself
Options: --wall-height 4500  --margin-x 2200 --margin-y 1800  --alpha 0.35  --rot 0 (degrees about Z)

The layout frame is aligned with the "Car Frame": X = the car's front, the car drives through from the left (west) to the right (east). To move the building, move the
frame "Cell Layout" in RoboDK (do not move its objects). After importing, check collisions again (Shift+F5 on Auto_Main): walls must stay clear of the gantry.
"""
import argparse
import os
import struct
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "cell_layout")
COLORS = {"floor": [0.55, 0.55, 0.58, 1.0], "wall": [0.85, 0.86, 0.90, 0.35], "column": [0.42, 0.43, 0.48, 1.0],
          "marking": [0.95, 0.80, 0.10, 1.0], "arrow": [0.10, 0.70, 0.25, 1.0]}
FILES = {"floor": "Layout_Floor", "wall": "Layout_Walls", "column": "Layout_Columns", "marking": "Layout_Markings", "arrow": "Layout_Arrows"}


# ------------------------------------------------------------------ geometry ------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0], [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], float)
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return v[np.array(f)]


def prism(poly, z0, z1):
    """Convex polygon (list of (x, y), counter-clockwise) extruded from z0 to z1."""
    p = np.array(poly, float)
    n = len(p)
    tris = []
    for i in range(1, n - 1):                                  # bottom (down) and top (up) fans
        tris.append([[*p[0], z0], [*p[i + 1], z0], [*p[i], z0]])
        tris.append([[*p[0], z1], [*p[i], z1], [*p[i + 1], z1]])
    for i in range(n):                                         # sides
        a, b = p[i], p[(i + 1) % n]
        tris.append([[*a, z0], [*b, z0], [*b, z1]])
        tris.append([[*a, z0], [*b, z1], [*a, z1]])
    return np.array(tris, float)


def write_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            n = np.cross(t[1] - t[0], t[2] - t[0])
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack("<12fH", *n, *t.reshape(-1), 0))


def wall_x(x0, x1, y0, y1, h, openings):
    """A wall running along X; openings = [(x_centre, width, height)]. Returns rectangles (x0, x1, y0, y1, z0, z1)."""
    out, cur = [], x0
    for xc, w, oh in sorted(openings):
        a, b = xc - w / 2, xc + w / 2
        if a > cur:
            out.append((cur, a, y0, y1, 0.0, h))
        out.append((a, b, y0, y1, oh, h))                       # header above the opening
        cur = b
    if cur < x1:
        out.append((cur, x1, y0, y1, 0.0, h))
    return out


def wall_y(x0, x1, y0, y1, h, openings):
    out, cur = [], y0
    for yc, w, oh in sorted(openings):
        a, b = yc - w / 2, yc + w / 2
        if a > cur:
            out.append((x0, x1, cur, a, 0.0, h))
        out.append((x0, x1, a, b, oh, h))
        cur = b
    if cur < y1:
        out.append((x0, x1, cur, y1, 0.0, h))
    return out


def build_layout(p):
    """Returns (list of elements, dict of key positions). Element = ('box', cat, x0, x1, y0, y1, z0, z1) or ('poly', cat, pts, z0, z1)."""
    L = 2 * p["zone_w"] + p["bay_x"]                            # clear length of the hall
    wt, pt, nt, H = p["wall_t"], p["part_t"], p["north_t"], p["wall_h"]
    y0, y1 = -p["bay_y"] / 2, p["bay_y"] / 2                    # hall: clear south / north faces
    y2 = y1 + pt + p["room_depth"]                              # rooms: clear north face
    xw, xe = -L / 2, L / 2                                      # hall: clear west / east faces
    el = []

    def add(cat, rects):
        for r in rects:
            el.append(("box", cat) + tuple(float(v) for v in r))
    # floor slab (top 50 mm below the floor level)
    add("floor", [(xw - wt, xe + wt, y0 - wt, y2 + nt, -300.0, -50.0)])
    # walls
    add("wall", wall_x(xw - wt, xe + wt, y0 - wt, y0, H, []))                                         # south wall
    add("wall", wall_x(xw - wt, xe + wt, y2, y2 + nt, H, []))                                         # north wall
    add("wall", wall_y(xw - wt, xw, y0 - wt, y2 + nt, H, [(0.0, p["open_w"], p["open_h"])]))          # west wall with the entry opening
    add("wall", wall_y(xe, xe + wt, y0 - wt, y2 + nt, H, [(0.0, p["open_w"], p["open_h"])]))          # east wall with the exit opening
    ratios = (4.0, 3.0, 5.0)
    widths = [L * r / sum(ratios) for r in ratios]
    xd1, xd2 = xw + widths[0], xw + widths[0] + widths[1]                                               # room dividers
    door_x = [xw + widths[0] * 0.80, xd1 + widths[1] * 0.72, xd2 + widths[2] * 0.77]
    add("wall", wall_x(xw, xe, y1, y1 + pt, H, [(x, p["door_w"], p["door_h"]) for x in door_x]))      # partition between hall and rooms, 3 doors
    add("wall", [(xd1 - pt / 2, xd1 + pt / 2, y1 + pt, y2, 0.0, H), (xd2 - pt / 2, xd2 + pt / 2, y1 + pt, y2, 0.0, H)])
    # columns on the wall lines
    c = p["col"]
    cols = []
    ys, yn = y0 - wt / 2, y2 + nt / 2
    for x in [xw - wt / 2, xe + wt / 2, xw + p["zone_w"], xe - p["zone_w"]] + [xw + p["zone_w"] + p["bay_x"] * k / 4 for k in (1, 2, 3)]:
        cols.append((x, ys))
    for x in [xw - wt / 2, xe + wt / 2, xd1, xd2, xw + widths[0] / 2, xd1 + widths[1] / 2, xd2 + widths[2] / 2]:
        cols.append((x, yn))
    for x in (xw - wt / 2, xe + wt / 2):
        for y in (y1 + pt / 2, -p["open_w"] / 2, p["open_w"] / 2):
            cols.append((x, y))
    add("column", [(x - c / 2, x + c / 2, y - c / 2, y + c / 2, 0.0, H) for x, y in cols])
    # floor markings: outline of the painting bay + centre line (5 mm proud of the slab, still below z = 0)
    bx0, bx1 = -p["bay_x"] / 2, p["bay_x"] / 2
    m, z0, z1 = 100.0, -50.0, -45.0
    add("marking", [(bx0, bx1, y0, y0 + m, z0, z1), (bx0, bx1, y1 - m, y1, z0, z1), (bx0, bx0 + m, y0, y1, z0, z1), (bx1 - m, bx1, y0, y1, z0, z1),
                    (xw + 200, xe - 200, -30.0, 30.0, z0, z1)])
    # arrows: shaft + head, pointing east (+X): entry zone and exit zone
    for xc in (xw + p["zone_w"] / 2, xe - p["zone_w"] / 2):
        add("arrow", [(xc - 700, xc + 100, -100.0, 100.0, z0, z1 + 5)])
        el.append(("poly", "arrow", [(xc + 100, -300.0), (xc + 700, 0.0), (xc + 100, 300.0)], z0, z1 + 5))
    info = {"L": L, "x_west": xw, "x_east": xe, "y_south": y0, "y_north_hall": y1, "y_north": y2, "widths": widths, "door_x": door_x,
            "outer": (xw - wt, xe + wt, y0 - wt, y2 + nt)}
    return el, info


def to_meshes(el):
    meshes = {k: [] for k in COLORS}
    for e in el:
        if e[0] == "box":
            meshes[e[1]].append(box(*e[2:]))
        else:
            meshes[e[1]].append(prism(e[2], e[3], e[4]))
    return {k: np.concatenate(v) for k, v in meshes.items() if v}


# ------------------------------------------------------------------ plan drawing ------------------------------------------------------------------
def draw_plan(el, info, p, path, envelope=None, car=(4800.0, 1900.0), car_xy=(0.0, 0.0)):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Polygon
    fig, ax = plt.subplots(figsize=(12, 8.2))
    style = {"floor": dict(fc="#e8e8ea", ec="none"), "wall": dict(fc="#4a4a52", ec="none"), "column": dict(fc="#222", ec="none"),
             "marking": dict(fc="#f2c200", ec="none"), "arrow": dict(fc="#1aa84a", ec="none")}
    order = ["floor", "marking", "arrow", "wall", "column"]
    for cat in order:
        for e in el:
            if e[1] != cat:
                continue
            if e[0] == "box":
                if cat == "wall" and e[6] > 0:                                          # a header above an opening: draw as an open gap outline
                    ax.add_patch(Rectangle((e[2], e[4]), e[3] - e[2], e[5] - e[4], fc="none", ec="#4a4a52", ls="--", lw=0.8))
                    continue
                ax.add_patch(Rectangle((e[2], e[4]), e[3] - e[2], e[5] - e[4], **style[cat], zorder=2))
            else:
                ax.add_patch(Polygon(e[2], closed=True, **style[cat], zorder=3))
    ax.add_patch(Rectangle((car_xy[0] - car[0] / 2, car_xy[1] - car[1] / 2), car[0], car[1], fc="#d33", ec="#900", alpha=0.55, zorder=5))
    ax.text(car_xy[0], car_xy[1], "car 4800 x 1900", ha="center", va="center", color="white", fontsize=9, zorder=6)
    if envelope:
        ex0, ex1, ey0, ey1 = envelope
        ax.add_patch(Rectangle((ex0, ey0), ex1 - ex0, ey1 - ey0, fc="none", ec="#0057b8", ls="-.", lw=1.6, zorder=6))
        ax.text(ex0 + 100, ey1 + 120, "gantry: travel of the robot base (measured)", color="#0057b8", fontsize=9, zorder=6)
    xo0, xo1, yo0, yo1 = info["outer"]
    ax.annotate("", xy=(xo1, yo0 - 700), xytext=(xo0, yo0 - 700), arrowprops=dict(arrowstyle="<->"))
    ax.text((xo0 + xo1) / 2, yo0 - 1100, f"{xo1 - xo0:.0f} mm", ha="center", fontsize=10)
    ax.annotate("", xy=(xo0 - 700, yo1), xytext=(xo0 - 700, yo0), arrowprops=dict(arrowstyle="<->"))
    ax.text(xo0 - 1000, (yo0 + yo1) / 2, f"{yo1 - yo0:.0f} mm", rotation=90, va="center", fontsize=10)
    bx = p["bay_x"] / 2
    ax.text(0, p["bay_y"] / 2 - 300, f"painting bay {p['bay_x']:.0f} x {p['bay_y']:.0f} mm", ha="center", fontsize=10, color="#555")
    ax.text(info["x_west"] + p["zone_w"] / 2, -p["bay_y"] / 2 + 450, f"entry zone\n{p['zone_w']:.0f} mm", ha="center", fontsize=9, color="#555")
    ax.text(info["x_east"] - p["zone_w"] / 2, -p["bay_y"] / 2 + 450, f"exit zone\n{p['zone_w']:.0f} mm", ha="center", fontsize=9, color="#555")
    xs = [info["x_west"], info["x_west"] + info["widths"][0], info["x_west"] + info["widths"][0] + info["widths"][1], info["x_east"]]
    for i, name in enumerate(("room 1", "room 2", "room 3")):
        ax.text((xs[i] + xs[i + 1]) / 2, (info["y_north_hall"] + info["y_north"]) / 2 + 200, f"{name}\n{xs[i + 1] - xs[i]:.0f} x {p['room_depth']:.0f}", ha="center", fontsize=9, color="#555")
    ax.set_aspect("equal")
    ax.set_xlim(xo0 - 1600, xo1 + 1200)
    ax.set_ylim(yo0 - 1800, yo1 + 600)
    ax.set_title("Painting cell layout (top view, floor level) - reference plan adapted to the gantry", fontsize=11)
    ax.set_xlabel("X (mm): the car drives from the entry on the left to the exit on the right")
    ax.set_ylabel("Y (mm)")
    ax.grid(True, alpha=0.2)
    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)


# ------------------------------------------------------------------ RoboDK ------------------------------------------------------------------
def to_np(m):
    return np.array([[float(v) for v in row] for row in m.rows], float)


def measure_gantry(RDK):
    """Move the gantry axes to their limit corners and read where the robot's base frame goes. Returns the base's travel as (x0, x1, y0, y1) in the Car Frame
    (mm), the car frame's absolute pose, and the highest base z above the floor."""
    from robodk import robolink
    carf = RDK.Item("Car Frame", robolink.ITEM_TYPE_FRAME)
    robots = RDK.ItemList(robolink.ITEM_TYPE_ROBOT)
    robot = next((r for r in robots if "ur10" in r.Name().lower()), robots[0] if robots else None)
    if robot is None or not carf.Valid():
        raise SystemExit("Could not find the robot or the Car Frame in the station.")
    lo, hi, _ = robot.JointLimits()
    lo = [float(v) for v in lo.list()]
    hi = [float(v) for v in hi.list()]
    if len(lo) < 9:
        raise SystemExit(f"The robot has {len(lo)} joints; expected 9 (6 arm + 3 gantry axes).")
    base = robot.Parent()
    original = [float(v) for v in robot.Joints().list()]
    inv_car = np.linalg.inv(to_np(carf.PoseAbs()))
    pts = []
    for a in (lo[6], hi[6]):
        for b in (lo[7], hi[7]):
            robot.setJoints(original[:6] + [a, b, original[8]])
            pts.append((inv_car @ to_np(base.PoseAbs()))[:3, 3])
    robot.setJoints(original)
    pts = np.array(pts)
    return (float(pts[:, 0].min()), float(pts[:, 0].max()), float(pts[:, 1].min()), float(pts[:, 1].max())), (lo[6:9], hi[6:9]), to_np(carf.PoseAbs())


def import_to_station(RDK, meshes, shift, rot_deg, alpha):
    from robodk import robolink
    carf = RDK.Item("Car Frame", robolink.ITEM_TYPE_FRAME)
    for old in [it for it in RDK.ItemList(robolink.ITEM_TYPE_FRAME) if it.Name() == "Cell Layout"]:
        old.Delete()
    frame = RDK.AddFrame("Cell Layout")
    c, s = np.cos(np.radians(rot_deg)), np.sin(np.radians(rot_deg))
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    T[:3, 3] = [shift[0], shift[1], 0.0]
    from robodk import robomath
    frame.setPoseAbs(robomath.Mat((to_np(carf.PoseAbs()) @ T).tolist()))
    for cat in meshes:
        item = RDK.AddFile(os.path.join(OUT, FILES[cat] + ".stl"), frame)
        item.setName(FILES[cat])
        col = list(COLORS[cat])
        if cat == "wall":
            col[3] = alpha
        item.setColor(col)
    print(f'Added the frame "Cell Layout" with {len(meshes)} objects. To move the building, move that frame.')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--measure", action="store_true", help="measure the gantry's working area in RoboDK and print the command to use (writes nothing)")
    ap.add_argument("--fit", action="store_true", help="measure the gantry in RoboDK and enlarge the bay around it")
    ap.add_argument("--import", dest="do_import", action="store_true", help="add the layout to the RoboDK station")
    ap.add_argument("--bay-x", type=float, default=8000.0)
    ap.add_argument("--bay-y", type=float, default=5000.0)
    ap.add_argument("--zone-w", type=float, default=2000.0, help="width of the entry and exit zones")
    ap.add_argument("--room-depth", type=float, default=2500.0)
    ap.add_argument("--wall-height", type=float, default=4500.0)
    ap.add_argument("--margin-x", type=float, default=2200.0, help="space between the robot base's travel and the bay's end walls (rails, pillars, clearance)")
    ap.add_argument("--margin-y", type=float, default=1800.0, help="same, across the car")
    ap.add_argument("--alpha", type=float, default=0.35, help="wall transparency in RoboDK (1 = solid)")
    ap.add_argument("--rot", type=float, default=0.0, help="rotate the layout about Z (degrees)")
    a = ap.parse_args()

    p = {"bay_x": a.bay_x, "bay_y": a.bay_y, "zone_w": a.zone_w, "room_depth": a.room_depth, "wall_h": a.wall_height, "wall_t": 300.0, "part_t": 200.0,
         "north_t": 300.0, "open_w": 3000.0, "open_h": 3500.0, "door_w": 1000.0, "door_h": 2200.0, "col": 400.0}
    RDK, envelope, shift = None, None, (0.0, 0.0)
    if a.measure or a.fit or a.do_import:
        from robodk import robolink
        RDK = robolink.Robolink()
    if a.measure or a.fit:
        (bx0, bx1, by0, by1), (lo, hi), _ = measure_gantry(RDK)
        cx0, cx1, cy0, cy1 = -2400.0, 2400.0, -1000.0, 1000.0                 # the car's footprint in the Car Frame
        ux0, ux1, uy0, uy1 = min(bx0, cx0), max(bx1, cx1), min(by0, cy0), max(by1, cy1)
        need_x, need_y = (ux1 - ux0) + 2 * a.margin_x, (uy1 - uy0) + 2 * a.margin_y
        print(f"Gantry axes (d7, d8, d9) limits: {[round(v) for v in lo]} .. {[round(v) for v in hi]} mm")
        print(f"The robot base travels over x {bx0:.0f}..{bx1:.0f}, y {by0:.0f}..{by1:.0f} mm in the Car Frame ({bx1 - bx0:.0f} x {by1 - by0:.0f} mm); the car is 4800 x 1900.")
        print(f"With margins {a.margin_x:.0f} / {a.margin_y:.0f} mm the painting bay needs about {need_x:.0f} x {need_y:.0f} mm (reference bay: 8000 x 5000).")
        envelope = (bx0, bx1, by0, by1)
        shift = ((ux0 + ux1) / 2, (uy0 + uy1) / 2)
        if a.measure and not a.fit:
            print(f"\nRun:  python3 make_cell_layout.py --fit --import     (or choose the sizes: --bay-x {max(8000, need_x):.0f} --bay-y {max(5000, need_y):.0f} --import)")
            return
        p["bay_x"], p["bay_y"] = max(a.bay_x, round(need_x, -2)), max(a.bay_y, round(need_y, -2))
        print(f"Using a bay of {p['bay_x']:.0f} x {p['bay_y']:.0f} mm, centred at x {shift[0]:.0f}, y {shift[1]:.0f} (Car Frame).")
    el, info = build_layout(p)
    os.makedirs(OUT, exist_ok=True)
    meshes = to_meshes(el)
    for cat, tris in meshes.items():
        write_stl(os.path.join(OUT, FILES[cat] + ".stl"), tris)
    outer = info["outer"]
    print(f"Wrote {len(meshes)} STL files to {OUT}: building {outer[1] - outer[0]:.0f} x {outer[3] - outer[2]:.0f} mm (outer), clear hall {info['L']:.0f} x {p['bay_y']:.0f} mm, walls {p['wall_h']:.0f} mm high.")
    try:
        env_plot = None if envelope is None else (envelope[0] - shift[0], envelope[1] - shift[0], envelope[2] - shift[1], envelope[3] - shift[1])
        # the plan is drawn in the layout's own coordinates (origin = centre of the bay); the car sits at -shift
        draw_plan(el, info, p, os.path.join(OUT, "layout_plan.png"), envelope=env_plot, car=(4800.0, 1900.0), car_xy=(-shift[0], -shift[1]))
        print("Wrote", os.path.join(OUT, "layout_plan.png"))
    except Exception as e:                                                      # noqa: BLE001
        print("(could not draw layout_plan.png:", e, ")")
    if a.do_import:
        import_to_station(RDK, meshes, shift, a.rot, a.alpha)


if __name__ == "__main__":
    main()
