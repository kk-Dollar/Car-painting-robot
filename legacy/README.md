# Legacy / archived

Earlier iterations and experiments that are **not** part of the final solution. Kept for
reference; nothing in `src/` depends on anything here.

## Earlier AMR / U-track idea
Before the gantry (`Lucas RC3-1Y1R`) was chosen, the plan was a floor-mounted robot on an
AMR / U-shaped track that moved around a static car.

- `generate_paint_paths.py` — the predecessor of `src/gantry_paint_paths.py`. **Note:** it still
  `import`s `gantry_reach` and `paint_common`, which now live in `src/`, so it will not run from
  this folder as-is; it is archived for reference only.
- `amr_station_reach.py` — AMR station reach study.
- `make_u_track_parts.py` — generates the U-track part STLs.
- `ut_1_base.stl`, `ut_2_xslide.stl`, `ut_3_carriage.stl`, `ut_4_turntable_base.stl`,
  `ut_5_turntable_top.stl`, `u_rail_1900.stl` — U-track meshes.
- `amr_platform_900x900x300.stl` — AMR platform mesh.
- `station_plan.json` — chosen station poses per zone (an unused constant `PLAN_FILE` in
  `paint_common.py` still names this file, but nothing opens it).
- `Car Painting Robot.rdk`, `Car Painting Robot_AMR.rdk` — the old stations (gitignored like all
  `.rdk` files).

## Superseded curve experiments
- `zone_curves_50mm.json` — 50 mm row spacing (the final uses 80 mm for ~36 % less path).
- `zone_curves_grouped_80mm.json` — a duplicate grouped file produced from a `/tmp` copy; its
  curves are byte-identical to `src/zone_curves_grouped.json`, only the provenance string differs.

## Misc
- `layout_plan_old_render.png` — an earlier render of the cell layout plan (superseded by
  `results/cell_layout/layout_plan.png`).
