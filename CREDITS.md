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
  scaled / re-oriented it to the target 4800 x 1900 x 1600 mm (front +X, left +Y, up +Z,
  tyres on Z = 0). The shipped STLs
  (`car_xr_sedan_4800x1900x1600.stl`, `car_paint_only_4800x1900x1600.stl`) are these modified
  versions and remain under CC BY 4.0 with attribution to mk2design.

## Robot, gantry and tool

- **UR10** (6-axis arm), **Lucas RC3-1Y1R** (3-axis gantry) and the paint-gun tool are RoboDK
  library models. They are **not** redistributed in this repository; they are part of the
  RoboDK installation and ship inside the RoboDK station (`Gantry Car Paint.rdk`), which is
  distributed separately, under RoboDK's own terms.

## Software

- **RoboDK** and its Python API (robodk / robolink / robomath).
- **Python 3.10**, **numpy**, **matplotlib**, **trimesh**.

## Reference

- Paint-overlap idea (~60 % stroke overlap for a 120-140 mm fan) adapted from robotic
  paint-path / paint-thickness planning work by Chen et al.:
  Chen, H., Sheng, W., Xi, N., Song, M. & Chen, Y. (2002),
  *"CAD-based automated robot trajectory planning for spray painting of free-form surfaces"*,
  **Industrial Robot: An International Journal**, Vol. 29 No. 5, pp. 426-433,
  doi:10.1108/01439910210440237.
  (The same authors published a closely related ICRA 2002 paper, *"Automated robot trajectory
  planning for spray painting of free-form surfaces in automotive manufacturing"* — confirm which
  one you consulted.)
