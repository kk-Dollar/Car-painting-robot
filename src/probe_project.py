"""
probe_project.py - read-only probe of your WORKING Curve Follow project, so the automation can be built on facts.

Run it with RoboDK open and your working project present (default name: "Path2 settings"):
    python3 probe_project.py                       # settings, links, start joints, path-to-tool offset
    python3 probe_project.py --update              # also runs project.Update(collisions ON): valid_ratio + message
    python3 probe_project.py --name "Path2 settings"

It reads only (get-style commands) and changes nothing. It writes probe_output.json next to the script; paste the printed output (or that file) back.
It also looks for the Surface Pattern Generator add-in folder, to see whether its code can be called from a script.
"""
import argparse
import json
import os

try:
    from robodk import robolink
except ImportError:
    import robolink

# READ-ONLY names only. (The first version also asked for OperationSpeed, JoinCurveTol, NormalApproach, SelectAlgorithm,
# OptimRail, OffsetRail, OptimTurntable and OffsetTurntable: those are SETTERS - called with an empty value they answer "OK"
# because they APPLY the empty value. They are removed.) "Machining", "OptimAxes" and "ProgEvents" are get-or-set commands:
# with no value they return the current settings.
PARAMS = ["Machining", "OptimAxes", "ProgEvents", "Visible"]


def parse(v):
    try:
        return json.loads(v)
    except Exception:
        return v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="Path2 settings")
    ap.add_argument("--update", action="store_true")
    a = ap.parse_args()

    RDK = robolink.Robolink()
    projects = RDK.ItemList(robolink.ITEM_TYPE_MACHINING, True)     # with True RoboDK returns the NAMES (strings)
    print("Machining projects in the station:", [p if isinstance(p, str) else p.Name() for p in projects])
    proj = RDK.Item(a.name, robolink.ITEM_TYPE_MACHINING)
    if not proj.Valid():
        raise SystemExit(f'No machining project named "{a.name}". Use --name with one of the names above.')
    out = {"name": proj.Name(), "params": {}, "links": {}}

    for label, t in (("robot", robolink.ITEM_TYPE_ROBOT), ("tool", robolink.ITEM_TYPE_TOOL), ("frame", robolink.ITEM_TYPE_FRAME),
                     ("object", robolink.ITEM_TYPE_OBJECT), ("program", robolink.ITEM_TYPE_PROGRAM)):
        try:
            it = proj.getLink(t)
            out["links"][label] = it.Name() if it.Valid() else None
        except Exception as e:
            out["links"][label] = f"error: {e}"
    try:
        out["preferred_start_joints"] = list(proj.Joints().list())
    except Exception as e:
        out["preferred_start_joints"] = f"error: {e}"
    try:
        out["path_to_tool_offset_xyzwpr"] = [round(x, 4) for x in proj.Pose().Pos()] + ["(pose matrix rows)", [list(map(lambda v: round(v, 4), r)) for r in proj.Pose().rows]]
    except Exception as e:
        out["path_to_tool_offset_xyzwpr"] = f"error: {e}"

    for name in PARAMS:
        try:
            v = proj.setParam(name)          # with no value this returns the current setting
            if v not in ("", None):
                out["params"][name] = parse(v)
        except Exception as e:
            out["params"][name] = f"error: {e}"

    if a.update:
        try:
            r = proj.Update(robolink.COLLISION_ON)
            out["update"] = {"valid_instructions": r[0], "time_s": r[1], "distance_mm": r[2], "valid_ratio": r[3], "message": r[4]}
        except Exception as e:
            out["update"] = f"error: {e}"

    # where is the Surface Pattern Generator add-in? (to see if its code can be imported/called)
    found = []
    for root in (os.path.expanduser("~/RoboDK/Apps"), "/opt/RoboDK/Apps", os.path.expanduser("~/.local/share/RoboDK/Apps"),
                 "/usr/share/RoboDK/Apps", "C:/RoboDK/Apps"):
        if os.path.isdir(root):
            for d in os.listdir(root):
                if "pattern" in d.lower():
                    full = os.path.join(root, d)
                    found.append({"folder": full, "files": sorted(os.listdir(full))[:40]})
    out["surface_pattern_generator_folders"] = found

    text = json.dumps(out, indent=1, default=str)
    print(text)
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "probe_output.json"), "w") as f:
        f.write(text)


if __name__ == "__main__":
    main()
