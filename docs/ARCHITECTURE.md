# Architecture & data flow

All scripts live in `src/` (kept flat) and find their data relative to their own folder
(`os.path.dirname(__file__)`), so moving the folder as a unit does not break any path.

## Data flow

```
source car model (GLB/glTF, from Sketchfab)
        │  extract_paint_surface.py            (trimesh)
        ▼
car_xr_sedan_4800x1900x1600.stl   (full car, for collisions)
car_paint_only_4800x1900x1600.stl (paint shell, 8,235 faces)
        │  segment_car_patches.py              (trimesh + numpy + matplotlib)
        ▼
car_zones.json                    (20 smooth patches, > 0.15 m²; 11.70 m² of 12.73 m²)
cad_patches_30deg.png             (3-view preview with patch ids)
        │  make_zone_curves.py                 (paint_common, numpy, matplotlib)
        ▼
zone_curves.json                  (zig-zag spray curves per patch, with surface normals)
zone_curves_preview.png
        │  group_zones.py                      (numpy, matplotlib)
        ▼
zone_curves_grouped.json          (20 patches merged into 9 zones, greedy curve ordering)   <-- FINAL build input
zone_curves_grouped_preview.png
        │  build_zones.py  (RoboDK running)    (gantry_paint_paths -> gantry_reach, paint_common; robodk)
        ▼
RoboDK programs  Auto_XX_<zone> + Auto_Main
build_report.json                 (per-piece: start pose, offsets, collision check, timing)
        │  paint_overlay.py / live_paint.py    (robodk, paint_common)
        ▼
src/overlay/*.stl                 (~900 thin red strips, hidden at first)
paint_overlay.json                (stroke poses + coverage %)

make_cell_layout.py  (independent):  car dimensions -> hall/bay/rooms/columns/markings STLs + layout_plan.png
```

## Module dependencies (import graph)

```
build_zones.py
 ├── gantry_paint_paths.py
 │    ├── gantry_reach.py
 │    └── paint_common.py
 └── paint_common.py
make_zone_curves.py        -> paint_common.py
gunon_tools.py             -> GunOn.py
live_paint.py              -> GunOn.py, paint_overlay.py
```

`gantry_paint_paths.py`, `gantry_reach.py` and `paint_common.py` are **not** stand-alone
pipeline steps — they are libraries imported by `build_zones.py` (and `make_zone_curves.py`).
They stay in `src/` for that reason. (`gantry_reach.ROBOT_NAME = "UR10"`; the `paint_common`
module also carries some unused constants left over from an earlier robot — harmless.)

## What each script reads / writes

| Script | Reads | Writes | Needs RoboDK |
|--------|-------|--------|:---:|
| `extract_paint_surface.py` | source model (CLI arg) | `<out>_full.stl`, `<out>_paint_only.stl` | no |
| `segment_car_patches.py` | `car_paint_only_*.stl` (positional) | `car_zones.json`, `cad_patches_30deg.png` | no |
| `make_zone_curves.py` | `car_paint_only_*.stl`, `car_zones.json`, `probe_curves_output.json` (for `--validate-door`) | `zone_curves.json`, `zone_curves_preview.png` | no |
| `group_zones.py` | `zone_curves.json`, `car_zones.json` | `zone_curves_grouped.json`, `*_preview.png` | no |
| `build_zones.py` | `zone_curves_grouped.json`, `car_paint_only_*.stl`, `build_report.json` (resume/prune), `hop_cache.json` | `Auto_*` programs, `Auto_Main`, `build_report.json`, `<station>_autosave.rdk` | **yes** |
| `GunOn.py` | station parameter `PaintCfg` | paint state in the station | **yes** (runs inside RoboDK) |
| `gunon_tools.py` | `GunOn.py` | installs/configures `GunOn` in the station | **yes** |
| `paint_overlay.py` | `car_paint_only_*.stl`, the `Auto_*` programs' poses | `src/overlay/*.stl`, `paint_overlay.json` | **yes** |
| `live_paint.py` | `paint_overlay.json`, `car_paint_only_*.stl` | reveals overlay strips live | **yes** |
| `make_cell_layout.py` | gantry travel (with `--measure`/`--fit`) | `src/cell_layout/*.stl`, `layout_plan.png` | only for `--measure/--fit/--import` |
| `diagnose_spray_reach.py` | `car_paint_only_*.stl`, `car_xr_sedan_*.stl` | prints a reach report | **yes** |
| `probe_project.py` / `probe_curves.py` | the `Path2` template | `probe_output.json` / `probe_curves_output.json` | **yes** |

## Key parameters

- **Stand-off:** 150 mm (nozzle to surface).
- **Row spacing:** 80 mm (≈ 60 % overlap for a 120-140 mm fan; 50 mm was the first setting,
  80 mm gives ~36 % less path). Point spacing 50 mm.
- **Patch segmentation:** 30 deg max normal angle inside a patch; patches kept if > 0.15 m².
- **TCP:** (50, 0, 450) mm, tilted 30 deg about Y relative to the tool item frame.
- **Gantry base search:** 400-1300 mm behind the gun; wound-up elbow (|J3| > 190 deg) tried last.
- **Build solve:** two passes (collision avoidance off, then optionally on); 30-min API timeout;
  autosave after each zone.
