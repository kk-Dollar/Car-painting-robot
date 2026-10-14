"""
make_u_track_parts.py - the 3D pieces for a MOVABLE U-track carriage, to feed RoboDK's mechanism builder
(Utilities > Model Mechanism or Robot). All pieces are in CAR coordinates (same frame as the car STL) with the
carriage at its HOME position (default x=-350, y=+1900: beside the doors on the left leg).

Pieces (zero/home pose):
  ut_1_base.stl            fixed U track: two legs at y=+-offset and a front crossbar (open at the rear)
  ut_2_xslide.stl          slider that rides on the track  (link 1 of a 2-linear-axis mechanism)
  ut_3_carriage.stl        900 x 900 x 200 carriage the robot stands on (link 2)
  ut_4_turntable_base.stl  fixed lower disc of the yaw turntable (on the carriage)
  ut_5_turntable_top.stl   rotating upper disc - the robot's base frame sits on top of it

Structure to build in RoboDK:  2-linear-axes mechanism (X, Y) from pieces 1-3, then a one-axis turntable
(pieces 4-5) mounted on its flange, then the robot on the turntable. Axis directions come from the builder's
Frame Base - check them by jogging (see the notes in the chat).
    python3 make_u_track_parts.py [--offset 1900] [--x-start -2200] [--x-end 3800] [--home-x -350]
"""
import argparse
import numpy as np
import trimesh


def box(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0])
    b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
    return b


def disc(cx, cy, r, z0, z1):
    c = trimesh.creation.cylinder(radius=r, height=z1 - z0, sections=48)
    c.apply_translation([cx, cy, (z0 + z1) / 2])
    return c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--offset", type=float, default=1900.0)
    ap.add_argument("--x-start", type=float, default=-2200.0)
    ap.add_argument("--x-end", type=float, default=3800.0)
    ap.add_argument("--home-x", type=float, default=-350.0)
    ap.add_argument("--home-y", type=float, default=None, help="default: the left leg (+offset)")
    a = ap.parse_args()
    o, w, hx = a.offset, 100.0, a.home_x
    hy = a.offset if a.home_y is None else a.home_y

    track_h = 100.0
    parts = {
        "ut_1_base": trimesh.util.concatenate([
            box(a.x_start, a.x_end, o - w, o + w, 0, track_h),
            box(a.x_start, a.x_end, -o - w, -o + w, 0, track_h),
            box(a.x_end - w, a.x_end + w, -o - w, o + w, 0, track_h)]),
        "ut_2_xslide": box(hx - 250, hx + 250, hy - 250, hy + 250, track_h, track_h + 60),
        "ut_3_carriage": box(hx - 450, hx + 450, hy - 450, hy + 450, track_h + 60, track_h + 260),
        "ut_4_turntable_base": disc(hx, hy, 420, track_h + 260, track_h + 320),
        "ut_5_turntable_top": disc(hx, hy, 400, track_h + 320, track_h + 360),
    }
    for name, mesh in parts.items():
        mesh.export(f"{name}.stl")
        lo, hi = mesh.bounds
        print(f"{name}.stl  x[{lo[0]:.0f},{hi[0]:.0f}] y[{lo[1]:.0f},{hi[1]:.0f}] z[{lo[2]:.0f},{hi[2]:.0f}]")
    print(f"\nHome carriage at x={hx:.0f}, y={hy:.0f}; robot base frame goes at z = {track_h + 360:.0f} mm "
          f"(top of the turntable), centred on x={hx:.0f}, y={hy:.0f}.")


if __name__ == "__main__":
    main()
