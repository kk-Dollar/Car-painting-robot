#!/usr/bin/env python3
"""gunon_tools.py - manage the GunOn spray script WITHOUT RoboDK's built-in text editor.

  python3 gunon_tools.py install [off|lite|full|overlay]   replace the station's GunOn item with the GunOn.py next to this file
                                                   (the programs call it by name, so they keep working) and reset the settings
  python3 gunon_tools.py mode off|lite|full|overlay  paint mode (overlay = no particles: painted surface is shown stroke by stroke, see paint_overlay.py) (takes effect for the next spray object: use `clear` or GunOn(-1) first)
  python3 gunon_tools.py reach MM                  how far past the nozzle-to-surface distance the spray volume extends (default 80)
  python3 gunon_tools.py axis +Z|-Z|+X|-X|+Y|-Y    which axis of the Paint gun's tool frame the spray goes along (default +Z)
  python3 gunon_tools.py anchor item|tcp           item = volume moved from the gun's mounting frame to its TCP (default), tcp = used as is
  python3 gunon_tools.py clear                     clear all paint (same as Esc) and forget the spray object, so the next GunOn(1) creates a fresh one
  python3 gunon_tools.py calibrate [program]       TEST how the spray volume must be placed: sprays at the first painting target of a program (default Auto_00_hood)
                                                   with four different placements, lets RoboDK draw a few frames for each and reads RoboDK's own "percent in";
                                                   keeps the one that lands on the car
  python3 gunon_tools.py status                    show the settings and RoboDK's spray statistics (how many particles landed on the car)
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robodk import robolink
import GunOn as G


def _flat(x):
    if x is None:
        return []
    if hasattr(x, "rows"):
        x = x.rows
    if hasattr(x, "__iter__") and not isinstance(x, (str, bytes)):
        out = []
        for e in x:
            out += _flat(e)
        return out
    return [float(x)]


def calibrate(RDK, cfg, prog_name):
    import time
    import numpy as np

    def to_np(m):
        return np.array([[float(v) for v in row] for row in m.rows], float)
    prog = RDK.Item(prog_name, robolink.ITEM_TYPE_PROGRAM)
    tool = RDK.Item("Paint gun", robolink.ITEM_TYPE_TOOL)
    carf = RDK.Item("Car Frame", robolink.ITEM_TYPE_FRAME)
    robots = RDK.ItemList(robolink.ITEM_TYPE_ROBOT)
    robot = next((r for r in robots if "ur10" in r.Name().lower()), robots[0] if robots else None)
    obj = G.find_car_object(RDK)
    if not (prog.Valid() and tool.Valid() and carf.Valid() and robot is not None and obj is not None):
        raise SystemExit("Could not find the program, the Paint gun, the Car Frame, the robot or the car object.")
    q, on = None, False                                   # the first target where the gun is ON
    for i in range(prog.InstructionCount()):
        ins = prog.Instruction(i)
        nm = (ins[0] or "").lower().replace(" ", "")
        if "gunon(" in nm:
            on = nm.split("gunon(")[1][:2].startswith("1")
            continue
        if on and ins[1] == getattr(robolink, "INS_TYPE_MOVE", 0):
            j = ins[5]
            q = [float(v) for v in (j.list() if hasattr(j, "list") else j)]
            break
    if not q:
        raise SystemExit(f"No painting move found in {prog_name}.")
    original = list(robot.Joints().list())
    prev_frame = robot.getLink(robolink.ITEM_TYPE_FRAME)
    robot.setPoseTool(tool)
    robot.setPoseFrame(carf)
    robot.setJoints(q)
    T_item = to_np(tool.PoseAbs())
    T_tcp = to_np(carf.PoseAbs()) @ to_np(robot.Pose())
    T_rel = np.linalg.inv(T_item) @ T_tcp                 # the TCP expressed in the tool item's own frame
    T_pt = G.tcp_matrix(tool)                             # what RoboDK says tool.PoseTool() is
    print("  TCP relative to the tool item (from the robot's pose): translation", np.round(T_rel[:3, 3], 1), " rotation about Y about",
          round(float(np.degrees(np.arctan2(T_rel[0, 2], T_rel[2, 2]))), 1), "deg")
    print("  tool.PoseTool() says: translation", np.round(T_pt[:3, 3], 1))
    mode = cfg["mode"] if cfg["mode"] in G.MODES else "lite"
    options = G.spray_options(mode)

    def trial(tool_arg, obj_arg, M, axis, fan, reach):
        """One test spray: returns (percent in, particles) as RoboDK reports them."""
        RDK.Spray_Clear()
        vol = G.cone_volume(G.DEFAULT_RGB, 150.0, fan, reach, axis, M).tr()
        sid = int(RDK.Spray_Add(tool_arg, obj_arg, options, vol))
        RDK.Spray_SetState(robolink.SPRAY_ON, sid)
        for _ in range(6):
            RDK.Render(True)
            time.sleep(0.1)
        info, data = RDK.Spray_GetStats(sid)
        vals = _flat(data)
        RDK.Spray_SetState(robolink.SPRAY_OFF, sid)
        RDK.Spray_Clear()
        return (vals[1], vals[3]) if len(vals) >= 4 else (None, None)

    placements = [("as is", None), ("moved by the TCP offset", T_rel)]
    axes = list(G.AXES)
    huge = (800.0, 500.0)                                  # a very big cone (fan 800 mm, reach 500 mm): if even this misses, the problem is not the aim
    results = []
    print("  searching: 2 placements x 6 axes with a huge cone, then the other car object and RoboDK's auto-detection ...")
    for pname, M in placements:
        for ax in axes:
            pi, cnt = trial(tool, obj, M, ax, *huge)
            results.append((pi or 0.0, pname, ax, "car object (" + obj.Name() + ")", "huge cone", M, obj))
    for it in RDK.ItemList(robolink.ITEM_TYPE_OBJECT):     # every object that looks like the car
        n = it.Name().lower()
        if ("car_paint_only" in n or "car_xr_sedan" in n) and it.Name() != obj.Name():
            for pname, M in placements:
                pi, cnt = trial(tool, it, M, "+Z", *huge)
                results.append((pi or 0.0, pname, "+Z", "car object (" + it.Name() + ")", "huge cone", M, it))
    for pname, M in placements:
        pi, cnt = trial(0, 0, M, "+Z", *huge)              # tool = 0, object = 0: let RoboDK choose them, as in its own example
        results.append((pi or 0.0, pname, "+Z", "auto-detected tool and object", "huge cone", M, 0))
    hits = sorted([r for r in results if r[0] > 0.5], key=lambda r: (-round(r[0] / 5.0), r[2] != "+Z", r[1] != "moved by the TCP offset"))   # ties: prefer +Z
    print(f"  tested {len(results)} combinations; {len(hits)} put particles on a car surface")
    for r in hits[:8]:
        print(f"      {r[0]:5.1f}% in | {r[1]:24s} | tool axis {r[2]} | {r[3]} | {r[4]}")
    robot.setJoints(original)
    try:
        if prev_frame.Valid():
            robot.setPoseFrame(prev_frame)
    except Exception:
        pass
    if not hits:
        print("  --> NOTHING reached a car surface, even with a huge cone in every direction. So the aim is not the problem: RoboDK's spray is not projecting"
              " onto this object at all from here. Send me this whole output.")
        return
    best = hits[0]
    pi_norm, _ = (None, None)
    RDK.Spray_Clear()
    robot.setJoints(q)
    pi_norm, _ = trial(tool, best[6], best[5], best[2], G.MODES[mode][3], float(cfg["reach"]))
    robot.setJoints(original)
    print(f"  best combination again with the NORMAL cone (fan {G.MODES[mode][3]:.0f} mm, reach {cfg['reach']} mm): {pi_norm}% in")
    if pi_norm is not None and pi_norm > 0.5:
        cfg["anchor"] = "tcp" if best[5] is None else "item"
        cfg["tcp"] = "" if best[5] is None else ",".join(f"{v:.6f}" for v in np.asarray(best[5]).reshape(-1))
        cfg["axis"] = best[2]
        cfg["spray"] = ""
        G.write_cfg(RDK, cfg)
        print(f"  --> Saved: placement '{best[1]}', tool axis {best[2]}. Press Esc, play the hood and run `status`.")
    else:
        print("  --> Only the huge cone reaches the car: the aim is right but the normal cone is too small or too short. Send me this output.")


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("install", "mode", "clear", "reach", "axis", "anchor", "status", "calibrate"):
        print(__doc__)
        return
    RDK = robolink.Robolink()
    cmd = sys.argv[1]
    cfg = G.read_cfg(RDK)

    def save(note):
        cfg["spray"], cfg["n"], cfg["armed"] = "", "0", "0"          # a changed setting needs a fresh spray object / a restarted overlay
        RDK.Spray_Clear()
        G.write_cfg(RDK, cfg)
        print(note, "- paint cleared; the next GunOn(1) creates a new spray object with the new settings.")

    if cmd == "calibrate":
        calibrate(RDK, cfg, sys.argv[2] if len(sys.argv) > 2 else "Auto_00_hood")
        return
    if cmd == "status":
        print("  settings:", {k: v for k, v in cfg.items()})
        try:
            info, data = RDK.Spray_GetStats()
            if data.size(1) > 0:
                print("Spray statistics from RoboDK:")
                print(info)
                print(data.tr())
            else:
                print("RoboDK has no spray objects yet (no GunOn(1) has created paint since the last clear).")
        except Exception as e:                               # noqa: BLE001
            print("Could not read the spray statistics:", e)
    elif cmd == "clear":
        save("Cleared")
    elif cmd == "mode":
        m = (sys.argv[2] if len(sys.argv) > 2 else "").lower()
        if m not in ("off", "lite", "full", "overlay", "live"):
            raise SystemExit("usage: gunon_tools.py mode off|lite|full|overlay|live")
        cfg["mode"] = m
        save(f"Paint mode = {m}")
    elif cmd == "reach":
        if len(sys.argv) < 3:
            raise SystemExit("usage: gunon_tools.py reach MM")
        cfg["reach"] = str(float(sys.argv[2]))
        save(f"Reach = {cfg['reach']} mm")
    elif cmd == "axis":
        ax = (sys.argv[2] if len(sys.argv) > 2 else "").upper()
        if ax not in G.AXES:
            raise SystemExit("usage: gunon_tools.py axis +Z|-Z|+X|-X|+Y|-Y")
        cfg["axis"] = ax
        save(f"Spray axis = tool {ax}")
    elif cmd == "anchor":
        an = (sys.argv[2] if len(sys.argv) > 2 else "").lower()
        if an not in ("item", "tcp"):
            raise SystemExit("usage: gunon_tools.py anchor item|tcp")
        cfg["anchor"] = an
        save(f"Spray anchor = {an}")
    elif cmd == "install":
        mode = sys.argv[2].lower() if len(sys.argv) > 2 else "lite"
        if mode not in ("off", "lite", "full", "overlay", "live"):
            raise SystemExit("mode must be off, lite, full, overlay or live")
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "GunOn.py")
        old = RDK.Item("GunOn", robolink.ITEM_TYPE_PROGRAM_PYTHON)
        if old.Valid():
            old.Delete()
        new = RDK.AddFile(path)
        if not new.Valid():
            raise SystemExit("RoboDK could not load GunOn.py. Add it with File > Open instead.")
        new.setName("GunOn")
        cfg = dict(G.DEFAULT_CFG)
        cfg["mode"] = mode
        RDK.Spray_Clear()
        G.write_cfg(RDK, cfg)
        print("GunOn replaced with the new version. Settings:", cfg)


if __name__ == "__main__":
    main()
