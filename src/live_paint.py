#!/usr/bin/env python3
"""live_paint.py - paint that FOLLOWS the gun while the simulation plays (no particles).

Run it in a second terminal while Auto_Main (or one program) plays in RoboDK:

    python3 live_paint.py                      follow Auto_Main's strokes
    python3 live_paint.py --focus Auto_00_hood follow one program's strokes (use the same name with  paint_overlay.py focus  for the overlay mode)

How it works: GunOn (mode "live") only counts finished strokes and says whether the gun is ON. This script polls RoboDK several times a second, asks
where the gun is, shoots a ray along the gun's Z axis at the paint shell, finds the nearest piece of the current stroke (paint_overlay.py cut every
stroke into pieces of ~250 mm) and makes that piece - and the ones before it - visible. When a stroke ends, all its pieces are shown. When Auto_Main
starts again (GunOn(-1)) everything is hidden again. A piece is only shown AFTER the gun has left it, so the paint trails behind the gun (by at most
one piece, ~250 mm) and never appears ahead of it.

Prepare once:   python3 paint_overlay.py build        python3 gunon_tools.py mode live
When it starts it re-reads the order of the programs in Auto_Main, so reordering Auto_Main (or adding link programs between the zones) needs no rebuild.
Stop with Ctrl+C. Needs paint_overlay.json and car_paint_only_4800x1900x1600.stl next to this script.
"""
import argparse
import json
import os
import sys
import time
import numpy as np
from robodk import robolink

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import GunOn as G
import paint_overlay as po


class LivePainter:
    def __init__(self, RDK, strokes, tris, shell_abs, tool, seg_mm):
        self.RDK, self.strokes, self.tris, self.tool = RDK, strokes, tris, tool
        self.inv_shell = np.linalg.inv(shell_abs)
        self.reach = 0.6 * seg_mm + 30.0              # how close the gun's spot must be to a piece's centre to count as "at that piece"
        self.trel = None
        self.items, self.revealed = {}, set()
        self.last_n, self.last_H = 0, None
        self.hits = self.polls = 0

    def _item(self, name):
        if name not in self.items:
            self.items[name] = self.RDK.Item(name, robolink.ITEM_TYPE_OBJECT)
        return self.items[name]

    def reveal(self, name):
        if name not in self.revealed:
            it = self._item(name)
            if it.Valid():
                it.setVisible(True)
            self.revealed.add(name)

    def hide_all(self):
        for st in self.strokes:
            for sg in st["segments"]:
                it = self._item(sg["name"])
                if it.Valid():
                    it.setVisible(False)
        self.revealed.clear()

    def reveal_stroke(self, k):
        if 0 <= k < len(self.strokes):
            for sg in self.strokes[k]["segments"]:
                self.reveal(sg["name"])

    def step(self):
        """One poll. Returns a short text describing what happened (for the log)."""
        self.polls += 1
        cfg = G.read_cfg(self.RDK)
        if cfg["mode"] != "live":
            return "idle (GunOn is not in live mode)"
        n, armed = int(cfg["n"]), cfg["armed"] == "1"
        if n < self.last_n:                              # GunOn(-1) ran: a new run starts
            self.hide_all()
            self.last_n, self.last_H = n, None
            return "reset: paint hidden"
        if n > self.last_n:                              # strokes finished since the last poll: show them completely
            for k in range(self.last_n, min(n, len(self.strokes))):
                self.reveal_stroke(k)
            self.last_n, self.last_H = n, None
            return f"stroke {n} finished"
        if not armed or n >= len(self.strokes):
            return "gun off"
        if self.trel is None:
            self.trel = G.matrix_from_cfg(cfg, self.tool)
        T = self.inv_shell @ po.to_np(self.tool.PoseAbs()) @ self.trel         # the gun tip in the shell's coordinates
        origin, direction = T[:3, 3], T[:3, 2]
        t = po.ray_front(self.tris, origin, direction)
        if t is None or t > 600.0:
            return "gun ON but not pointing at the shell"
        H = origin + t * direction
        self.hits += 1
        segs = self.strokes[n]["segments"]
        centres = np.array([s["center"] for s in segs])
        dist = np.linalg.norm(centres - H, axis=1)
        j = int(dist.argmin())
        if dist[j] > self.reach:
            return f"gun ON, spot {dist[j]:.0f} mm from the nearest piece (too far)"
        for i in range(j):                               # only pieces the gun has already PASSED: the paint always stays behind the gun
            self.reveal(segs[i]["name"])
        self.last_H = H
        return f"stroke {n + 1}: gun at piece {j + 1} of {len(segs)} ({j} shown behind it)"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--focus", default="main", help="main, or the name of one program")
    ap.add_argument("--poll", type=float, default=0.08, help="seconds between polls")
    ap.add_argument("--no-hide", action="store_true", help="do not hide all paint at the start")
    ap.add_argument("--no-resequence", action="store_true", help="do not re-read the order of the programs in Auto_Main (use the order stored by the last build)")
    a = ap.parse_args()
    RDK = robolink.Robolink()
    if a.focus == "main" and not a.no_resequence:
        meta = po.resequence(RDK)                       # Auto_Main may have been reordered since `build`: follow its CURRENT order
    else:
        meta = json.load(open(os.path.join(HERE, "paint_overlay.json")))
    strokes = meta["strokes"] if a.focus == "main" else meta["programs"].get(a.focus)
    if not strokes:
        raise SystemExit(f"No strokes for '{a.focus}' in paint_overlay.json. Run `python3 paint_overlay.py build` first.")
    tool = RDK.Item("Paint gun", robolink.ITEM_TYPE_TOOL)
    carf = RDK.Item("Car Frame", robolink.ITEM_TYPE_FRAME)
    shell = next((it for it in RDK.ItemList(robolink.ITEM_TYPE_OBJECT) if "car_paint_only" in it.Name().lower()), None)
    stl = os.path.join(HERE, "car_paint_only_4800x1900x1600.stl")
    if not (tool.Valid() and carf.Valid() and shell is not None and os.path.isfile(stl)):
        raise SystemExit("Need the Paint gun, the Car Frame, the car_paint_only object and car_paint_only_4800x1900x1600.stl next to this script.")
    painter = LivePainter(RDK, strokes, po.load_triangles(stl), po.to_np(carf.PoseAbs()) @ po.to_np(shell.Pose()), tool, float(meta.get("segment_mm", 250.0)))
    cfg = G.read_cfg(RDK)
    if cfg["mode"] != "live":
        print("Note: GunOn is in mode '%s'. Run  python3 gunon_tools.py mode live  for this script to do anything." % cfg["mode"])
    if not a.no_hide:
        print("Hiding all paint pieces ...")
        painter.hide_all()
    print(f"Following {len(strokes)} strokes of '{a.focus}'. Play the program in RoboDK. Ctrl+C to stop.")
    last = ""
    try:
        while True:
            msg = painter.step()
            if msg != last:
                print("  ", msg, flush=True)
                last = msg
            time.sleep(a.poll)
    except KeyboardInterrupt:
        print(f"\nStopped after {painter.polls} polls ({painter.hits} with the gun pointing at the shell).")


if __name__ == "__main__":
    main()
