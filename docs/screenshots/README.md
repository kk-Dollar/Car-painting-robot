# Screenshots to capture

Drop the PNGs in this folder (and copy the final set into the Google Drive `Screenshots/`
folder). Suggested filenames in brackets. These are the shots still to take in RoboDK:

- [ ] **Full cell, 3D view** — the gantry + UR10 + car inside the hall. `[01_full_cell.png]`
- [ ] **Layout, top view** — the hall/bay/rooms from above (compare with
      `docs/images/layout_plan.png`). `[02_layout_top.png]`
- [ ] **Car with spray strokes** — the green zig-zag curves on the shell
      (like `docs/images/zone_curves_grouped_preview.png`). `[03_car_strokes.png]`
- [ ] **Each zone painted** — one shot per zone after the overlay reveals it
      (hood, roof, trunk, front face, rear, left doors, right doors, left rail, right rail).
      `[04_zone_<name>.png]`
- [ ] **Coverage line** — the text printed by `GunOn(2)` / `paint_overlay.py`
      ("Painted surface 9.75 m² of 12.73 m² … 77 %"). `[05_coverage.png]`
- [ ] **Collision check "No problems found"** — RoboDK's check result for a program
      (e.g. `Auto_00_hood`). `[06_collision_ok.png]`
- [ ] **Build report** — the results table (from `docs/RESULTS.md` or `build_report.json`).
      `[07_build_report.png]`
- [ ] **Live painting mid-run** — Auto_Main playing with red strips appearing behind the gun
      (`live_paint.py` running). `[08_live_paint.png]`
- [ ] **layout_plan.png** — already generated at `docs/images/layout_plan.png`; include it.

Tip: use a clean background and the same camera angle for the per-zone shots so the "different
surfaces" requirement reads clearly.
