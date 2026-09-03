# Shared assets

`field26.png` is the official 2026 field render, cropped to the field of play and
laid out the way PathPlanner's field images are: 3508 x 1814 px at 200 px/m with
a 0.5 m margin on every side, blue alliance wall at x = 0. That is what
`field_view.py`, `render_demo.py` and the cockpit all assume when they map field
metres onto it, so a replacement has to keep the same framing.

It is deliberately the render *without* game pieces on it: the only fuel in the
picture is the fuel the simulation is publishing, drawn on top, so the model is
never looking at balls that are not really there.

It lives here so the picture the AI looks at and the picture in the demo video
are the same field, and so neither needs to download anything to draw it.
