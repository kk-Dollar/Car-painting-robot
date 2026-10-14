# GunOn.py - spray-paint simulation for the car-painting station.   (v6: few RoboDK calls per GunOn(...) - see below)
#
# Adapted from the Python item "GunOn" inside RoboDK's example "Painting and Spraying with ABB IRB 2600-12/1.85" (RDK.Spray_Add / Spray_SetState).
# Add it to the station:  File > Open (choose GunOn.py), or run  python3 gunon_tools.py install lite
#
#     GunOn(1)                 spray ON, default colour
#     GunOn(1,0.9,0.1,0.1)     spray ON with colour R,G,B (0..1)
#     GunOn(1,0.9,0.1,0.1,150) same, and the nozzle-to-surface distance in mm (sizes the spray fan)
#     GunOn(0)                 spray OFF
#     GunOn(-1)                clear all paint (use once at the start of a main program)
#     GunOn(2)                 print spray statistics (area / coverage) to the output
#     GunOn(5)                 MUTE: paint switched off for everything after this call (the strokes' own ON calls are ignored)
#     GunOn(6)                 UNMUTE: painting back on in lite mode      GunOn(7)   UNMUTE in full mode      GunOn(8)   UNMUTE as overlay (no particles)   GunOn(9)   UNMUTE as live painting
#
# WHY THIS VERSION IS DIFFERENT: every GunOn(...) is a separate Python process, and each of its calls to RoboDK has to wait until RoboDK has finished
# drawing its current frame. The old script made 15-60 such calls per GunOn (it asked for the name of every object in the station, read five settings
# one by one, and created a NEW spray object at every stroke). While RoboDK was busy drawing thousands of particles, every call took seconds, so the
# simulation stood still. Now: all settings live in ONE station parameter ("PaintCfg"), the spray object is created ONCE (after GunOn(-1)) and only
# switched on and off afterwards: GunOn(1) = 2 calls, GunOn(0) = 1 call.
import re
import sys
from robodk import robolink, robomath

TOOL_NAME = "Paint gun"
PAINT_OBJECT_KEYS = ("car_xr_sedan", "car_paint_only")   # first visible match receives the paint
STANDOFF = 250.0        # mm, nozzle to surface (for the fan size)
DEFAULT_RGB = (0.85, 0.10, 0.10)

# Paint modes: off = no paint | lite = flat 60 mm squares on a 50 mm grid (whole car in view) | full = flat 40 mm squares on a 36 mm grid (close look)
# Each entry: particle width mm, thickness mm, grid step mm, fan width mm at the standoff, random scatter (RAND=)
MODES = {"lite": (60.0, 4.0, 50.0, 120.0, 0), "full": (40.0, 8.0, 36.0, 140.0, 1)}
AXES = {"+X": ((1, 0, 0), (0, 1, 0), (0, 0, 1)), "-X": ((-1, 0, 0), (0, 1, 0), (0, 0, 1)),
        "+Y": ((0, 1, 0), (0, 0, 1), (1, 0, 0)), "-Y": ((0, -1, 0), (0, 0, 1), (1, 0, 0)),
        "+Z": ((0, 0, 1), (1, 0, 0), (0, 1, 0)), "-Z": ((0, 0, -1), (1, 0, 0), (0, 1, 0))}   # spray axis, two perpendiculars
DEFAULT_CFG = {"mode": "lite", "reach": "80", "axis": "+Z", "anchor": "item", "spray": "", "tcp": "", "n": "0", "armed": "0"}   # tcp = 16 numbers (4x4); n = strokes shown so far (overlay mode)


# ---- the single station parameter "PaintCfg" = "mode=lite;reach=80;axis=+Z;anchor=item;spray=3" ------------------------------------------------------------
def read_cfg(RDK):
    cfg = dict(DEFAULT_CFG)
    try:
        raw = RDK.getParam("PaintCfg")
    except Exception:
        raw = None
    for part in str(raw or "").split(";"):
        if "=" in part:
            k, v = part.split("=", 1)
            if k.strip() in cfg:
                cfg[k.strip()] = v.strip()
    if cfg["mode"] not in MODES and cfg["mode"] not in ("off", "overlay", "live"):
        cfg["mode"] = DEFAULT_CFG["mode"]
    if cfg["axis"] not in AXES:
        cfg["axis"] = DEFAULT_CFG["axis"]
    if cfg["anchor"] not in ("item", "tcp"):
        cfg["anchor"] = DEFAULT_CFG["anchor"]
    try:
        cfg["reach"] = str(min(max(float(cfg["reach"]), 20.0), 400.0))
    except ValueError:
        cfg["reach"] = DEFAULT_CFG["reach"]
    return cfg


def write_cfg(RDK, cfg):
    RDK.setParam("PaintCfg", ";".join(f"{k}={cfg[k]}" for k in DEFAULT_CFG))


def matrix_from_cfg(cfg, tool):
    """The TCP offset (4x4) stored by `gunon_tools.py calibrate`; if there is none, whatever tool.PoseTool() says."""
    import numpy as np
    try:
        vals = [float(v) for v in cfg.get("tcp", "").split(",") if v.strip() != ""]
        if len(vals) == 16:
            return np.array(vals, float).reshape(4, 4)
    except ValueError:
        pass
    return tcp_matrix(tool)


def spray_options(mode):
    size, thick, grid, _, rand = MODES[mode]
    return "ELLYPSE PROJECT PARTICLE=CUBE(%g,%g,%g) STEP=%gx%g RAND=%g" % (size, size, thick, grid, grid, rand)


def tcp_matrix(tool):
    """4x4 pose of the TCP relative to the tool item's own frame (identity if it cannot be read)."""
    import numpy as np
    try:
        return np.array([[float(v) for v in row] for row in tool.PoseTool().rows], float)
    except Exception:
        return np.eye(4)


def robomath_vec(t):
    import numpy as np
    return np.array(t, float)


def parse_args(argv):
    """Accept GunOn(1,r,g,b[,standoff]) however RoboDK hands the arguments over (separate or comma-joined)."""
    vals = [float(v) for v in re.findall(r"-?\d+(?:\.\d+)?", " ".join(argv[1:]))]
    action = int(vals[0]) if vals else 1
    rgb = tuple(vals[1:4]) if len(vals) >= 4 else DEFAULT_RGB
    standoff = vals[4] if len(vals) >= 5 and vals[4] > 50 else STANDOFF
    return action, rgb, standoff


def cone_volume(rgb, standoff=None, fan_width=120.0, reach=80.0, axis="+Z", tcp=None):
    """Spray volume as in the example: ellipses at a near and a far plane along the chosen tool axis, each given as origin p0, axis points pA and pB, then
    RGBA (opaque). The far plane lies `reach` mm beyond the standoff and is sized so that the footprint is fan_width wide at the standoff. With tcp given
    (4x4) the points are moved from the gun's TCP frame into the tool item's own frame (RoboDK's spray statistics showed that is where the volume is anchored)."""
    standoff = STANDOFF if standoff is None else standoff
    far_plane = standoff + reach
    far_radius = (fan_width / 2.0) * (far_plane / standoff)
    a, u, v = (robomath_vec(x) for x in AXES.get(axis, AXES["+Z"]))
    r, g, b = rgb

    def place(p):
        p = robomath_vec(p)
        return [float(x) for x in (p if tcp is None else tcp[:3, :3] @ p + tcp[:3, 3])]
    centre = far_plane * a
    close_param = place([0, 0, 0]) + place(4 * u) + place(4 * v) + [r, g, b, 1.0]
    far_param = place(centre) + place(centre + far_radius * u) + place(centre + far_radius * v) + [r, g, b, 1.0]
    return robomath.Mat([close_param, far_param])


def reveal_next(RDK, cfg):
    """Overlay mode: one stroke has just finished - show all pieces of the next stroke of the sequence (station parameter "PaintSeq":
    strokes separated by ',', the pieces of a stroke by '|')."""
    seq = [s for s in str(RDK.getParam("PaintSeq") or "").split(",") if s]
    n = int(cfg["n"])
    cfg["armed"] = "0"
    if n < len(seq):
        for name in seq[n].split("|"):
            it = RDK.Item(name, robolink.ITEM_TYPE_OBJECT)
            if it.Valid():
                it.setVisible(True)
        cfg["n"] = str(n + 1)
    write_cfg(RDK, cfg)


def hide_revealed(RDK, cfg):
    seq = [s for s in str(RDK.getParam("PaintSeq") or "").split(",") if s]
    for stroke in seq[:int(cfg["n"])]:
        for name in stroke.split("|"):
            it = RDK.Item(name, robolink.ITEM_TYPE_OBJECT)
            if it.Valid():
                it.setVisible(False)


def find_car_object(RDK):
    for key in PAINT_OBJECT_KEYS:
        for item in RDK.ItemList(robolink.ITEM_TYPE_OBJECT):
            name = item.Name().lower().replace("\u00d7", "x")
            if key in name and "collision" not in name:
                return item
    return None


def main():
    RDK = robolink.Robolink()
    if RDK.RunMode() != robolink.RUNMODE_SIMULATE:      # same as the example: simulation only
        quit()
    action, rgb, standoff = parse_args(sys.argv)

    if action == 0:                                       # OFF
        cfg = read_cfg(RDK)
        if cfg["mode"] == "overlay":
            if cfg["armed"] == "1":                       # only a stroke that was really started shows its paint
                reveal_next(RDK, cfg)
            return
        if cfg["mode"] == "live":                         # live_paint.py shows the paint while the gun moves; here only the stroke counter moves on
            if cfg["armed"] == "1":
                cfg["armed"], cfg["n"] = "0", str(int(cfg["n"]) + 1)
                write_cfg(RDK, cfg)
            return
        RDK.Spray_SetState(robolink.SPRAY_OFF)
        return
    cfg = read_cfg(RDK)                                   # the one parameter read
    if action < 0:
        if cfg["mode"] == "overlay":
            hide_revealed(RDK, cfg)                       # (in live mode live_paint.py notices the reset and hides the paint itself)
        RDK.Spray_Clear()
        cfg["spray"], cfg["n"], cfg["armed"] = "", "0", "0"
        write_cfg(RDK, cfg)
        print("GunOn: cleared. mode = %s, reach = %s mm, axis = %s, anchor = %s" % (cfg["mode"], cfg["reach"], cfg["axis"], cfg["anchor"]))
    elif action == 2 and cfg["mode"] in ("overlay", "live"):
        seq = [s for s in str(RDK.getParam("PaintSeq") or "").split(",") if s]
        print("Paint coverage (overlay): %s of %s strokes shown. %s" % (cfg["n"], len(seq), RDK.getParam("PaintCoverage") or ""))
    elif action == 2:
        info, data = RDK.Spray_GetStats()
        if data.size(1) > 0:
            print("Spray gun statistics:")
            print(info)
            print(data.tr())
        else:
            print("No spray data yet.")
    elif action in (5, 6, 7, 8, 9):                       # mute / unmute the paint from inside a program
        cfg["mode"] = {5: "off", 6: "lite", 7: "full", 8: "overlay", 9: "live"}[action]
        write_cfg(RDK, cfg)
        if cfg["mode"] == "off":
            RDK.Spray_SetState(robolink.SPRAY_OFF)
        print("GunOn: paint mode set to %s" % cfg["mode"])
    elif cfg["mode"] == "off":
        return                                            # no paint: the motion alone is simulated
    elif cfg["mode"] in ("overlay", "live"):
        cfg["armed"] = "1"                                # a stroke starts; its painted surface is shown when it ends (GunOn(0))
        write_cfg(RDK, cfg)
    else:                                                 # ON
        if cfg["spray"] != "":
            RDK.Spray_SetState(robolink.SPRAY_ON, int(cfg["spray"]))      # the spray object exists: just switch it on
            return
        tool = RDK.Item(TOOL_NAME, robolink.ITEM_TYPE_TOOL)               # first ON after a clear: create the spray object once
        obj = find_car_object(RDK)
        if not tool.Valid() or obj is None:
            print("GunOn: tool '%s' or the car object was not found." % TOOL_NAME)
            quit()
        tcp = matrix_from_cfg(cfg, tool) if cfg["anchor"] == "item" else None
        sid = RDK.Spray_Add(tool, obj, spray_options(cfg["mode"]),
                            cone_volume(rgb, standoff, MODES[cfg["mode"]][3], float(cfg["reach"]), cfg["axis"], tcp).tr())
        cfg["spray"] = str(int(sid))
        write_cfg(RDK, cfg)
        RDK.Spray_SetState(robolink.SPRAY_ON, int(sid))


if __name__ == "__main__":
    main()
