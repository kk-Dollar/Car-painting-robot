"""
diagnose_station.py - find out why every station gave identical reach results.

Run with RoboDK open on the AMR test station:   python3 diagnose_station.py
It only prints numbers and puts everything back afterwards. Paste the whole output.
"""
import math
import numpy as np

try:
    from robodk import robolink, robomath
except ImportError:
    import robolink
    import robomath

ROBOT_NAME = "Fanuc P-200 LA HB"
TOOL_NAME = "Paint gun"
PLATFORM_KEY = "amr_platform"
CAR_KEY = "car_xr_sedan"


def norm(t):
    return t.lower().replace("\u00d7", "x")


def find(RDK, key, exclude="collision"):
    for it in RDK.ItemList(robolink.ITEM_TYPE_OBJECT):
        n = norm(it.Name())
        if norm(key) in n and exclude not in n:
            return it
    raise SystemExit(f"Cannot find an object containing '{key}'")


def xyz(M):
    p = M.Pos()
    return f"({p[0]:9.1f}, {p[1]:9.1f}, {p[2]:9.1f})"


def station_T(x, y, yaw_deg):
    yaw = math.atan2(-y, -x) + math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    return robomath.Mat([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1]])


def snapshot(label, robot, tool, platform, base):
    print(f"[{label}]")
    print(f"  platform  PoseAbs : {xyz(platform.PoseAbs())}")
    print(f"  base frame PoseAbs: {xyz(base.PoseAbs())}   ({base.Name()})")
    print(f"  robot     PoseAbs : {xyz(robot.PoseAbs())}")
    print(f"  tool      PoseAbs : {xyz(tool.PoseAbs())}")
    print(f"  robot.Pose() (TCP): {xyz(robot.Pose())}")


def main():
    RDK = robolink.Robolink()
    robot = RDK.Item(ROBOT_NAME, robolink.ITEM_TYPE_ROBOT)
    tool = RDK.Item(TOOL_NAME, robolink.ITEM_TYPE_TOOL)
    platform = find(RDK, PLATFORM_KEY)
    car = find(RDK, CAR_KEY)
    base = robot.Parent()
    print(f"robot parent (base frame): '{base.Name()}'   platform parent: '{platform.Parent().Name()}'")
    print(f"base frame parent: '{base.Parent().Name()}'\n")

    original = platform.PoseAbs()
    car_T = car.PoseAbs()

    snapshot("before moving", robot, tool, platform, base)

    platform.setPoseAbs(car_T * station_T(3800, 0, 0))     # Front station
    snapshot("after moving platform to the FRONT station", robot, tool, platform, base)

    platform.setPoseAbs(car_T * station_T(-3800, 0, 0))    # Rear station
    snapshot("after moving platform to the REAR station", robot, tool, platform, base)

    # does SolveIK use the same convention as robot.Pose()?
    platform.setPoseAbs(original)
    joints_now = robot.Joints().list()
    P = robot.Pose()
    sol = robot.SolveIK(P)
    sol_list = sol.list() if sol is not None else []
    print("\n[SolveIK sanity check]")
    print(f"  current joints : {[round(j, 1) for j in joints_now]}")
    print(f"  SolveIK(robot.Pose()) : {[round(j, 1) for j in sol_list[:len(joints_now)]]}")

    # same, but after the platform was moved
    platform.setPoseAbs(car_T * station_T(3800, 0, 0))
    P2 = robot.Pose()
    sol2 = robot.SolveIK(P2)
    print(f"  after moving to FRONT: robot.Pose() = {xyz(P2)}  SolveIK ok: {sol2 is not None and len(sol2.list()) >= len(joints_now)}")

    platform.setPoseAbs(original)
    print("\nPlatform restored. Check that the robot is back where it started.")


if __name__ == "__main__":
    main()
