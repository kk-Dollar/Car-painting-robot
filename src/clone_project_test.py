"""
clone_project_test.py - round-trip test of the automation: re-create your WORKING Curve Follow project from a script.

It reads everything from the template (default "Path2 settings"): robot, tool, reference frame, the path object, the
Machining / OptimAxes / ProgEvents settings, the preferred start joints and the path-to-tool offset. Then it creates a NEW
project "Clone test settings" on the same path object, applies those settings through the API, runs Update with collisions ON
on both projects and compares the results (valid ratio, cycle time, distance). If the clone reproduces the template, the
create -> configure -> validate loop works and the same code can be used for every zone.

    python3 clone_project_test.py                      # exact clone, nothing else changed
    python3 clone_project_test.py --speed 150          # also set the operation speed (mm/s) - not readable through the API
    python3 clone_project_test.py --spray              # also add GunOn events: spray ON at each path start, OFF at each finish

New: every step is timed (to see where the minutes go), the template is only re-checked with --compare, and some switches
help to find what makes it slow:
    python3 clone_project_test.py --settings-first     # apply the settings BEFORE the first solve (avoids the failed default solve)
    python3 clone_project_test.py --steps 50,10        # coarser collision-check steps (mm, deg) for Update
    python3 clone_project_test.py --rotz-range 60      # tool-spin search of +/-60 deg instead of +/-180 (fewer candidates per point)
    python3 clone_project_test.py --no-avoid           # AvoidCollisions off (timing comparison only)

The template is never modified. A previous clone with the same name (and its program) is deleted first, so you can re-run it.
Not yet tested against your RoboDK: paste back the printed output.
"""
import argparse
import json
import time

try:
    from robodk import robolink
    from robodk.robomath import Mat
except ImportError:
    import robolink
    from robomath import Mat


def get_json(item, name):
    v = item.setParam(name)               # "Machining", "OptimAxes", "ProgEvents": no value -> returns the current settings
    return json.loads(v) if isinstance(v, str) else v


def link(item, kind):
    try:
        it = item.getLink(kind)
        return it if it.Valid() else None
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--template", default="Path2 settings")
    ap.add_argument("--name", default="Clone test settings")
    ap.add_argument("--speed", type=float, help="operation speed in mm/s (cannot be read back, so only set when given)")
    ap.add_argument("--join-tol", type=float, help="join curve tolerance in mm (only set when given)")
    ap.add_argument("--spray", action="store_true", help="add GunOn(1,r,g,b,standoff) on path start and GunOn(0) on path finish")
    ap.add_argument("--standoff", type=float, default=250.0, help="value passed to GunOn so the spray fan matches (mm)")
    ap.add_argument("--rgb", default="0.85,0.1,0.1")
    ap.add_argument("--no-update", action="store_true")
    ap.add_argument("--compare", action="store_true", help="also run Update on the template (doubles the checking time)")
    ap.add_argument("--settings-first", action="store_true", help="apply the copied settings before the first solve")
    ap.add_argument("--steps", help="Update collision-check steps 'mm,deg', e.g. 50,10 (default: the Tools-Options-Motion values)")
    ap.add_argument("--rotz-range", type=float, help="override Machining RotZ_Range (deg, 0-180)")
    ap.add_argument("--no-avoid", action="store_true", help="set AvoidCollisions to 0 (timing comparison only)")
    a = ap.parse_args()

    RDK = robolink.Robolink()
    tmpl = RDK.Item(a.template, robolink.ITEM_TYPE_MACHINING)
    if not tmpl.Valid():
        raise SystemExit(f'Template "{a.template}" not found. Projects: {RDK.ItemList(robolink.ITEM_TYPE_MACHINING, True)}')

    robot = link(tmpl, robolink.ITEM_TYPE_ROBOT)
    tool = link(tmpl, robolink.ITEM_TYPE_TOOL)
    frame = link(tmpl, robolink.ITEM_TYPE_FRAME)
    part = link(tmpl, robolink.ITEM_TYPE_OBJECT)
    if not (robot and tool and frame and part):
        raise SystemExit(f"Template links incomplete: robot={robot}, tool={tool}, frame={frame}, object={part}")
    machining, optim, events = get_json(tmpl, "Machining"), get_json(tmpl, "OptimAxes"), get_json(tmpl, "ProgEvents")
    start_joints = tmpl.Joints()
    path_to_tool = tmpl.Pose()
    original_joints = robot.Joints()
    print(f"Template: robot={robot.Name()}, tool={tool.Name()}, frame={frame.Name()}, path object={part.Name()}")

    # idempotent: remove an earlier clone and its program
    old = RDK.Item(a.name, robolink.ITEM_TYPE_MACHINING)
    if old.Valid():
        oldprog = link(old, robolink.ITEM_TYPE_PROGRAM)
        old.Delete()
        if oldprog:
            oldprog.Delete()
        print("Deleted the previous clone.")

    t_start = time.time()
    timings = []

    def timed(label, fn):
        t0 = time.time()
        res = fn()
        timings.append((label, time.time() - t0))
        print(f"  [{timings[-1][1]:6.1f} s] {label}")
        return res

    if a.rotz_range is not None:
        machining["RotZ_Range"] = a.rotz_range
    if a.no_avoid:
        machining["AvoidCollisions"] = 0
    if a.spray:
        r, g, b = [float(x) for x in a.rgb.split(",")]
        events.update({"CallPathStart": f"GunOn(1,{r:g},{g:g},{b:g},{a.standoff:g})", "CallPathStartOn": 1,
                       "CallPathFinish": "GunOn(0)", "CallPathFinishOn": 1})

    def apply_settings(proj):
        proj.setRobot(robot)
        proj.setPoseTool(tool)
        proj.setPoseFrame(frame)
        proj.setParam("Machining", machining)
        proj.setParam("OptimAxes", optim)
        proj.setParam("ProgEvents", events)
        proj.setJoints(start_joints)
        proj.setPose(path_to_tool)
        if a.speed is not None:
            proj.setParam("OperationSpeed", a.speed)
        if a.join_tol is not None:
            proj.setParam("JoinCurveTol", a.join_tol)

    # create the project on the same path object (names differ between RoboDK versions)
    add = getattr(RDK, "AddMillingProject", None) or getattr(RDK, "AddMachiningProject")
    setp_name = "setMillingParameters" if hasattr(robolink.Item, "setMillingParameters") else "setMachiningParameters"
    proj = timed("create the empty project", lambda: add(a.name))
    robot.setPoseTool(tool)
    robot.setPoseFrame(frame)
    if a.settings_first:
        timed("apply the copied settings (before the first solve)", lambda: apply_settings(proj))
    prog, status = timed("attach the path object (this triggers a solve)",
                         lambda: getattr(proj, setp_name)(part=part, params="ReorderAuto=0"))
    print(f"  -> program '{prog.Name() if prog and prog.Valid() else None}', status={status}")
    if not a.settings_first:
        timed("apply the copied settings", lambda: apply_settings(proj))

    results = []
    if not a.no_update:
        kw = {}
        if a.steps:
            mm, deg = [float(x) for x in a.steps.split(",")]
            kw = {"mm_step": mm, "deg_step": deg}
        items = [("clone", proj)] + ([("template", tmpl)] if a.compare else [])
        for label, item in items:
            r = timed(f"Update with collisions ON ({label})", lambda item=item: item.Update(robolink.COLLISION_ON, **kw))
            results.append((label, r))
            print(f"     {label:8s}: valid_instructions={r[0]:.0f}  time={r[1]:.1f}s  distance={r[2]:.0f}mm  valid_ratio={r[3]:.2f}  msg={r[4]}")
        c = results[0][1]
        ok = c[3] >= 0.999
        if a.compare:
            t = results[1][1]
            same = abs(t[2] - c[2]) <= 0.02 * max(t[2], 1) and ok
            print("\nRESULT:", "the clone reproduces the template." if same else "the clone DIFFERS from the template (numbers above).")
        else:
            print("\nRESULT:", "the clone is valid (valid_ratio 1.0, no problems)." if ok else "the clone has problems - see the message above.")
    print(f"\nTotal time: {time.time() - t_start:.0f} s  |  slowest step: {max(timings, key=lambda x: x[1])[0]}")
    robot.setJoints(original_joints)


if __name__ == "__main__":
    main()
