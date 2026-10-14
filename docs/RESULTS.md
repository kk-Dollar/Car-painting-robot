# Results

All numbers below come straight from `results/build_report.json` (the build run) and
`src/paint_overlay.json` (the coverage overlay). Nothing here is hand-estimated.

## Summary

| Metric | Value | Source |
|--------|-------|--------|
| Path pieces solved | **16** (each passed RoboDK's collision check, valid ratio 1.0, "No problems found") | `build_report.json` |
| Programs called by `Auto_Main` | **14** | `paint_overlay.json` (`programs`) |
| Pieces that failed to attach/pass | **3** (`front_centre_top_5`, `left_rail_p4`, `right_rail_p5`) | `build_report.json` |
| Computed coverage | **9.75 m² of 12.73 m² = 77 %** (120 mm strips) | `paint_overlay.json` (`coverage`) |
| Total solved spray-path length | ≈ **150 m** | sum of `distance_mm` over OK pieces |
| Total solve time (OK pieces) | ≈ **16 min** | sum of `time_s` over OK pieces |

Zones 3, 4, 7 and 8 have a parent summary row (`front`, `rear`, `left_rail`, `right_rail`) that
was split into pieces (`_p1`, `_p2`, …); the parent rows carry the "uncovered" length and are not
programs themselves.

## Per-piece table (from build_report.json)

| id | label | status | curves | reach | offset L,a,b (mm) | valid ratio | path mm | solve s | in Auto_Main |
|---:|-------|--------|---:|:---:|---|:---:|---:|---:|:---:|
| 0 | hood | OK | 7 | 12/12 | 700,300,0 | 1.00 | 21685 | 143.6 | yes |
| 1 | roof | OK | 1 | 12/12 | 700,0,0 | 1.00 | 16150 | 105.8 | yes |
| 2 | trunk | OK | 5 | 7/12 | 1300,300,0 | 1.00 | 14826 | 96.0 | yes |
| 3 | front | PARTIAL 1/2 | 8 | 9/12 | 900,0,300 | — | 0 | 0.0 | - |
| 3 | front_centre_front_1 | OK | 6 | 9/12 | 1100,-300,300 | 1.00 | 10582 | 69.7 | yes |
| 3 | front_centre_top_5 | FAILED | 2 | 12/12 | 900,0,0 | — | 0 | 0.0 | - |
| 4 | rear | OK (split) | 5 | 7/12 | 1100,-300,0 | — | 0 | 0.0 | - |
| 4 | rear_p1 | OK | 3 | 8/12 | 1100,0,300 | 1.00 | 5107 | 33.6 | yes |
| 4 | rear_p2 | OK | 2 | 7/12 | 700,0,600 | 1.00 | 2568 | 12.5 | yes |
| 5 | left_doors | OK | 6 | 8/12 | 1100,0,600 | 1.00 | 23250 | 155.2 | yes |
| 6 | right_doors | OK | 7 | 4/12 | 700,0,0 | 1.00 | 24936 | 162.0 | yes |
| 7 | left_rail | PARTIAL 4/5 | 14 | 11/12 | 1100,0,300 | — | 0 | 0.0 | - |
| 7 | left_rail_p1 | OK | 3 | 12/12 | 700,0,0 | 1.00 | 5291 | 33.7 | yes |
| 7 | left_rail_p2 | OK | 3 | 8/12 | 400,0,900 | 1.00 | 4230 | 25.2 | yes |
| 7 | left_rail_p3 | OK | 3 | 12/12 | 700,0,300 | 1.00 | 4137 | 24.7 | yes |
| 7 | left_rail_p4 | FAILED | 3 | 12/12 | 700,-300,0 | 0.88 | 4801 | 29.1 | - |
| 7 | left_rail_p5 | OK | 2 | 10/12 | 900,-300,0 | 1.00 | 1518 | 9.9 | no |
| 8 | right_rail | PARTIAL 4/5 | 15 | 12/12 | 900,0,300 | — | 0 | 0.0 | - |
| 8 | right_rail_p1 | OK | 3 | 12/12 | 700,300,0 | 1.00 | 4952 | 31.4 | yes |
| 8 | right_rail_p2 | OK | 3 | 7/12 | 400,-300,600 | 1.00 | 3032 | 14.3 | no |
| 8 | right_rail_p3 | OK | 3 | 12/12 | 900,0,0 | 1.00 | 3717 | 22.3 | yes |
| 8 | right_rail_p4 | OK | 3 | 12/12 | 900,0,0 | 1.00 | 3992 | 25.6 | yes |
| 8 | right_rail_p5 | FAILED | 3 | 12/12 | 900,-300,0 | — | 0 | 0.0 | - |

- **valid ratio** is RoboDK's collision-check result for the path (1.00 = no collision over the
  whole path; `left_rail_p4` reached 0.88 before a collision at MoveL 66).
- **reach** is how many of 12 sample poses the start-pose search reached.
- **offset L,a,b** is the chosen path-to-tool offset (gantry base distance behind the gun, plus
  lateral/height tweaks) that made the piece solve.

## Programs in Auto_Main (14, from paint_overlay.json)

```
Auto_00_hood                    Auto_06_right_doors
Auto_01_roof                    Auto_07_left_rail_p1
Auto_02_trunk                   Auto_07_left_rail_p2
Auto_03_front_centre_front_1    Auto_07_left_rail_p3
Auto_04_rear_p1                 Auto_08_right_rail_p1
Auto_04_rear_p2                 Auto_08_right_rail_p3
Auto_05_left_doors              Auto_08_right_rail_p4
```

Two pieces that solved OK — `Auto_07_left_rail_p5` and `Auto_08_right_rail_p2` — are **not** in
Auto_Main (they could not be joined to their neighbours by a safe transit).

## Uncovered surfaces (from the parent rows' `uncovered_path_mm`)

| Zone | Uncovered stroke length |
|------|------------------------:|
| front (upper: headlight/grille openings) | 2045 mm |
| left rail | 3008 mm |
| right rail | 3109 mm |

Plus the failed pieces `front_centre_top_5`, `left_rail_p4`, `right_rail_p5`.

## Notes

The data files above are the source of truth.

- Coverage is **9.75 m² / 77 %** per `paint_overlay.json`.
- `Auto_Main` contains **14** programs per `paint_overlay.json`; `right_rail_p2` and
  `left_rail_p5` solved OK (valid ratio 1.0) but were left out of the master program because they
  could not be joined to their neighbours by a safe transit.
- **Transit (program-to-program) moves:** each zone program is individually collision-free; the
  moves *between* programs are planned by `build_zones.py --transit` plus RoboDK's collision-free
  planner. To regenerate and re-verify the master transit plan, run
  `python3 build_zones.py --main-only --transit --auto-order` with the station open and read the
  console report.
