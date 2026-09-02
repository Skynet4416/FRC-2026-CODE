# Shared assets

`field26.png` is PathPlanner's official 2026 field image, from
[mjansen4857/pathplanner](https://github.com/mjansen4857/pathplanner)
(`images/field26.png`, MIT licensed). It is 3508 x 1814 px at 200 px/m with a
0.5 m margin on every side, which is what `field_view.py` and `render_demo.py`
both assume when they map field metres onto it.

It lives here so the picture the AI looks at and the picture in the demo video
are the same field, and so neither needs to download anything to draw it.
