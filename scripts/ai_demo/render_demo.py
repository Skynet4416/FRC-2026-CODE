import pickle, math, numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon, Circle
import imageio.v2 as imageio

frames = pickle.load(open("/tmp/demo_frames.pkl","rb"))
img = imageio.imread("/tmp/field26.png")
H, W = img.shape[0], img.shape[1]
# PathPlanner ships field26.png at 200 px/m with a 0.5 m margin; scale if it is resized
PPM, MARGIN = 200.0 * W / 3508.0, 0.5
extent = [-MARGIN, W/PPM - MARGIN, -MARGIN, H/PPM - MARGIN]

BOT = 0.92           # bumper square, metres
FPS = 20

fig = plt.figure(figsize=(12.8, 7.6), dpi=100)
fig.patch.set_facecolor("#0e1116")
ax = fig.add_axes([0.0, 0.0, 1.0, 0.86])
ax.set_facecolor("#0e1116")
ax.imshow(img, extent=extent, origin="upper", zorder=0)
ax.set_xlim(extent[0], extent[1]); ax.set_ylim(extent[2], extent[3])
ax.set_aspect("equal"); ax.axis("off")

hud_prompt = fig.text(0.012, 0.965, "", color="#8ab4ff", fontsize=17, fontweight="bold", va="top", family="DejaVu Sans")
hud_call   = fig.text(0.012, 0.915, "", color="#c9d1d9", fontsize=12, va="top", family="DejaVu Sans Mono")
hud_status = fig.text(0.988, 0.962, "", color="#8b949e", fontsize=11, va="top", ha="right", family="DejaVu Sans Mono")
hud_score  = fig.text(0.988, 0.905, "", color="#ffb000", fontsize=15, fontweight="bold", va="top", ha="right", family="DejaVu Sans Mono")
hud_badges = fig.text(0.012, 0.878, "", color="#3fb950", fontsize=11, va="top", family="DejaVu Sans Mono")
title      = fig.text(0.5, 0.055, "", color="#6e7681", fontsize=11, ha="center", transform=fig.transFigure)

path_line, = ax.plot([], [], "-", color="#58a6ff", lw=2.0, alpha=0.85, zorder=3)
fuel_scat  = ax.scatter([], [], s=26, c="#ffb000", edgecolors="#7a5200", linewidths=0.4, zorder=4)
target_m,  = ax.plot([], [], marker="X", ms=15, color="#3fb950", mew=2, ls="none", zorder=5)
trail,     = ax.plot([], [], "-", color="#f0883e", lw=1.6, alpha=0.55, zorder=3)

bot = Rectangle((0,0), BOT, BOT, angle=0, facecolor="#1f6feb", edgecolor="#c9d1d9", lw=1.6, zorder=6, alpha=0.95)
ax.add_patch(bot)
nose = Polygon([[0,0],[0,0],[0,0]], closed=True, facecolor="#f85149", edgecolor="none", zorder=7)
ax.add_patch(nose)
lock = Circle((0,0), 0.0, facecolor="none", edgecolor="#f85149", lw=2.2, zorder=5, alpha=0.9)
ax.add_patch(lock)

def robot_corner(x, y, th):
    # bottom-left corner of the rotated square whose centre is (x, y)
    dx, dy = -BOT/2, -BOT/2
    return x + dx*math.cos(th) - dy*math.sin(th), y + dx*math.sin(th) + dy*math.cos(th)

writer = imageio.get_writer("/tmp/ai_control_demo.mp4", fps=FPS, codec="libx264",
                            quality=8, macro_block_size=1, ffmpeg_params=["-pix_fmt","yuv420p"])
trail_pts = []
for i, f in enumerate(frames):
    if len(f["pose"]) < 3:
        continue
    x, y, th = f["pose"][0], f["pose"][1], f["pose"][2]
    cx, cy = robot_corner(x, y, th)
    bot.set_xy((cx, cy)); bot.set_angle(math.degrees(th))
    tip = (x + 0.62*math.cos(th), y + 0.62*math.sin(th))
    l = (x + 0.30*math.cos(th+2.4), y + 0.30*math.sin(th+2.4))
    r = (x + 0.30*math.cos(th-2.4), y + 0.30*math.sin(th-2.4))
    nose.set_xy([tip, l, r])

    lock.set_radius(0.75 if f.get("rot") else 0.0)
    lock.set_center((x, y))

    trail_pts.append((x, y))
    trail_pts = trail_pts[-140:]
    trail.set_data([p[0] for p in trail_pts], [p[1] for p in trail_pts])

    tr = f["traj"]
    if tr and len(tr) >= 6:
        path_line.set_data(tr[0::3], tr[1::3])
    else:
        path_line.set_data([], [])

    fl = f["fuel"]
    if fl and len(fl) >= 3:
        pts = np.array([[fl[k], fl[k+1]] for k in range(0, len(fl)-2, 3)])
        fuel_scat.set_offsets(pts)
    else:
        fuel_scat.set_offsets(np.empty((0, 2)))

    tg = f["target"]
    if tg and len(tg) == 3:
        target_m.set_data([tg[0]], [tg[1]])
    else:
        target_m.set_data([], [])

    hud_prompt.set_text(f.get("prompt") or "")
    hud_call.set_text(f.get("caption") or "")
    hud_status.set_text((f.get("status") or "")[:64])
    badges = []
    if f.get("nav"): badges.append("[ PATHFINDING ]")
    if f.get("act"): badges.append(f"[ {f.get('action') or 'ACTION'} ]")
    if f.get("rot"): badges.append("[ SHOOTER OWNS THE DRIVETRAIN ]")
    badges.append("[ ALLIANCE ZONE ]" if f.get("zone") else "[ OUT OF THE SHOOTING ZONE ]")
    hud_badges.set_text("  ".join(badges))
    hud_score.set_text(f"HUB SCORE  {f.get('score') if f.get('score') is not None else 0}")
    title.set_text("Skynet 4416  |  /AIControl NetworkTables bridge  ->  PathPlanner pathfinding  |  MapleSim + PhotonVision sim")

    fig.canvas.draw()
    buf = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
    writer.append_data(buf)

writer.close()
print("wrote /tmp/ai_control_demo.mp4 from", len(frames), "frames")
