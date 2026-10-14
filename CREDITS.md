# Credits

**Author / copyright holder:** Krishna Pandit.

Developed by Krishna Pandit with the help of an AI assistant (**Claude by Anthropic**).
I ran, tested and verified all results myself.

## Car model

- **"Generic Toyama XR Sedan"** by **mk2design**
- Source: https://skfb.ly/6vENy
- Licence: **Creative Commons Attribution 4.0 International (CC BY 4.0)** —
  https://creativecommons.org/licenses/by/4.0/
- Modifications by Krishna Pandit: extracted a paint-only shell from the full model and
  scaled / re-oriented it to the brief's 4800 x 1900 x 1600 mm (front +X, left +Y, up +Z,
  tyres on Z = 0). The shipped STLs
  (`car_xr_sedan_4800x1900x1600.stl`, `car_paint_only_4800x1900x1600.stl`) are these modified
  versions and remain under CC BY 4.0 with attribution to mk2design.

## Robot, gantry and tool

- **UR10** (6-axis arm), **Lucas RC3-1Y1R** (3-axis gantry) and the paint-gun tool are RoboDK
  library models. They are **not** redistributed in this repository; they are part of the
  RoboDK installation and ship inside the RoboDK station (`Gantry Car Paint.rdk`) delivered via
  Google Drive, under RoboDK's own terms.

## Software

- **RoboDK** and its Python API (robodk / robolink / robomath).
- **Python 3.10**, **numpy**, **matplotlib**, **trimesh**.

## Reference

- Paint-overlap idea (~60 % stroke overlap for a 120-140 mm fan) adapted from
  **Chen et al., 2002** on robotic paint-path and paint-thickness planning.
  Exact title and venue: **TODO(Krishna)** — fill in the full citation (the paper PDF was not in
  the project folder, so it could not be read for the exact title).
