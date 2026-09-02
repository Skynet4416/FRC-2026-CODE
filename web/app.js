// The cockpit's front end: poll /state, draw the field, post instructions.
//
// Everything it draws comes from the robot over NetworkTables (relayed by
// serve.py) and everything in the transcript comes from the model, so this file
// only decides how it looks.

const IMAGE_PIXELS_PER_METRE = 200; // PathPlanner ships field26.png at 200 px/m
const IMAGE_MARGIN_M = 0.5;         // ...with half a metre of margin all round

// The canvas is smaller than the image, so metres map through the ratio between
// them rather than the image's own scale. Set once the image is loaded.
let PIXELS_PER_METRE = IMAGE_PIXELS_PER_METRE;
const ROBOT_SIZE_M = 0.9;
const POLL_MS = 120;

const canvas = document.getElementById("canvas");
const context = canvas.getContext("2d");
const statusPill = document.getElementById("status");
const overlay = document.getElementById("overlay");
const transcriptList = document.getElementById("transcript");
const promptBox = document.getElementById("prompt");
const tokenBox = document.getElementById("token");
const hint = document.getElementById("hint");
const sendButton = document.getElementById("send");

const field = new Image();
field.src = "field26.png";

let navgrid = null;
let latest = null;
let shownEvents = 0;

fetch("navgrid.json").then((r) => r.json()).then((g) => { navgrid = g; }).catch(() => {});

// --- geometry -------------------------------------------------------------

// Field metres -> canvas pixels. Y is flipped: the field's Y grows upwards.
const toX = (x) => (x + IMAGE_MARGIN_M) * PIXELS_PER_METRE;
const toY = (y) => canvas.height - (y + IMAGE_MARGIN_M) * PIXELS_PER_METRE;

function draw() {
  const state = latest;
  context.clearRect(0, 0, canvas.width, canvas.height);
  if (field.complete && field.naturalWidth) {
    context.drawImage(field, 0, 0, canvas.width, canvas.height);
  } else {
    context.fillStyle = "#fff";
    context.fillRect(0, 0, canvas.width, canvas.height);
  }
  if (!state) return;

  drawObstacles();
  drawGrid(state.field_size);
  drawLandmarks(state.landmarks);
  drawFuel(state.fuel_all, "rgba(200, 160, 40, 0.55)", null, 5);
  drawFuel(state.fuel, "#ff8c00", "#5a3000", 7);
  drawTrajectory(state.trajectory);
  drawTarget(state.robot.active_target);
  drawRobot(state.robot);
}

function drawObstacles() {
  if (!navgrid) return;
  const cell = navgrid.nodeSizeMeters * PIXELS_PER_METRE;
  context.fillStyle = "rgba(43, 43, 51, 0.16)";
  navgrid.grid.forEach((row, rowIndex) => {
    row.forEach((blocked, columnIndex) => {
      if (!blocked) return;
      const x = toX(columnIndex * navgrid.nodeSizeMeters);
      const y = toY((rowIndex + 1) * navgrid.nodeSizeMeters);
      context.fillRect(x, y, cell, cell);
    });
  });
}

function drawGrid(size) {
  if (!size || size.length !== 2) return;
  context.strokeStyle = "rgba(43, 43, 51, 0.25)";
  context.fillStyle = "#3c3c50";
  context.lineWidth = 1;
  context.font = "16px ui-monospace, Menlo, monospace";
  for (let x = 0; x <= size[0]; x += 1) {
    context.beginPath();
    context.moveTo(toX(x), toY(0));
    context.lineTo(toX(x), toY(size[1]));
    context.stroke();
    context.fillText(String(x), toX(x) + 3, toY(0) - 5);
  }
  for (let y = 0; y <= size[1]; y += 1) {
    context.beginPath();
    context.moveTo(toX(0), toY(y));
    context.lineTo(toX(size[0]), toY(y));
    context.stroke();
    context.fillText(String(y), toX(0) + 4, toY(y) - 5);
  }
}

function drawLandmarks(landmarks) {
  if (!landmarks) return;
  context.fillStyle = "#3c3c50";
  context.font = "15px system-ui, sans-serif";
  context.textAlign = "center";
  for (const [name, pose] of Object.entries(landmarks)) {
    const x = toX(pose[0]);
    const y = toY(pose[1]);
    context.fillRect(x - 5, y - 1, 10, 2);
    context.fillRect(x - 1, y - 5, 2, 10);
    context.fillText(name, x, y - 10);
  }
  context.textAlign = "left";
}

// Two layers: every fuel the physics sim has, then the ones the camera can
// actually see - which is all the model is told about.
function drawFuel(fuel, fill, stroke, radius) {
  if (!fuel) return;
  context.fillStyle = fill;
  context.lineWidth = 1.5;
  for (const [x, y] of fuel) {
    context.beginPath();
    context.arc(toX(x), toY(y), radius, 0, Math.PI * 2);
    context.fill();
    if (stroke) {
      context.strokeStyle = stroke;
      context.stroke();
    }
  }
}

function drawTrajectory(points) {
  if (!points || points.length < 2) return;
  context.strokeStyle = "#0a68d8";
  context.lineWidth = 4;
  context.beginPath();
  points.forEach(([x, y], index) => {
    const method = index === 0 ? "moveTo" : "lineTo";
    context[method](toX(x), toY(y));
  });
  context.stroke();
}

function drawTarget(target) {
  if (!target) return;
  const x = toX(target.x);
  const y = toY(target.y);
  context.strokeStyle = "#0f9d58";
  context.lineWidth = 4;
  context.beginPath();
  context.moveTo(x - 12, y - 12);
  context.lineTo(x + 12, y + 12);
  context.moveTo(x + 12, y - 12);
  context.lineTo(x - 12, y + 12);
  context.stroke();
}

function drawRobot(robot) {
  if (!robot || !robot.pose) return;
  const { x, y, heading_deg: heading } = robot.pose;
  const size = ROBOT_SIZE_M * PIXELS_PER_METRE;

  context.save();
  context.translate(toX(x), toY(y));
  context.rotate((-heading * Math.PI) / 180); // canvas Y is flipped, so is rotation
  context.fillStyle = "rgba(31, 111, 235, 0.9)";
  context.strokeStyle = "#101018";
  context.lineWidth = 2;
  context.fillRect(-size / 2, -size / 2, size, size);
  context.strokeRect(-size / 2, -size / 2, size, size);

  context.fillStyle = "#f85149"; // the nose, so the heading is unmistakable
  context.beginPath();
  context.moveTo(size * 0.68, 0);
  context.lineTo(size * 0.3, -size * 0.26);
  context.lineTo(size * 0.3, size * 0.26);
  context.closePath();
  context.fill();
  context.restore();

  if (robot.rotation_locked) {
    context.strokeStyle = "#e0362c";
    context.lineWidth = 4;
    context.beginPath();
    context.arc(toX(x), toY(y), 0.75 * PIXELS_PER_METRE, 0, Math.PI * 2);
    context.stroke();
  }
}

// --- state ----------------------------------------------------------------

async function poll() {
  try {
    const response = await fetch("state", { cache: "no-store" });
    if (!response.ok) throw new Error(`state: ${response.status}`);
    latest = await response.json();
    render();
  } catch (error) {
    setStatus("down", "no connection to the cockpit - is serve.py running?");
  }
}

function render() {
  const robot = latest.robot;
  if (!latest.connected) {
    setStatus("down", "the robot is not connected - start ./gradlew simulateJava");
  } else if (latest.error) {
    setStatus("down", latest.error);
  } else if (latest.busy) {
    setStatus("busy", robot.status || "working");
  } else {
    setStatus("live", robot.status || "idle");
  }

  sendButton.disabled = latest.busy || !latest.connected;
  sendButton.textContent = latest.busy ? "Working…" : "Send";
  tokenBox.hidden = !latest.needs_token;

  const pose = robot.pose || { x: 0, y: 0, heading_deg: 0 };
  overlay.textContent = [
    `x ${pose.x.toFixed(2)} m   y ${pose.y.toFixed(2)} m   ${pose.heading_deg.toFixed(0)}°`,
    `${robot.in_shooting_zone ? "in the shooting zone" : "outside the shooting zone"}`
      + `${robot.rotation_locked ? "   shooter owns the drivetrain" : ""}`,
    `${latest.fuel.length} of ${(latest.fuel_all || []).length} fuel in sight`
      + `   ${latest.fuel_on_board ?? "?"} on board`
      + (robot.intake_collecting ? "   intaking" : ""),
    latest.model || "",
  ].join("\n");

  const events = latest.transcript || [];
  if (events.length < shownEvents) {
    transcriptList.replaceChildren();
    shownEvents = 0;
  }
  for (const event of events.slice(shownEvents)) {
    const item = document.createElement("li");
    item.className = event.kind;
    const kind = document.createElement("span");
    kind.className = "kind";
    kind.textContent = event.kind.replace("_", " ");
    item.append(kind, document.createTextNode(event.text));
    transcriptList.append(item);
  }
  if (events.length !== shownEvents) {
    shownEvents = events.length;
    transcriptList.lastElementChild?.scrollIntoView({ block: "nearest" });
  }

  draw();
}

function setStatus(kind, text) {
  statusPill.className = `pill ${kind}`;
  statusPill.textContent = text;
}

// --- controls -------------------------------------------------------------

async function post(path, body) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({ ...body, token: tokenBox.value || undefined }),
  });
  return { ok: response.ok, payload: await response.json().catch(() => ({})) };
}

document.getElementById("prompt-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const text = promptBox.value.trim();
  if (!text) return;
  hint.className = "hint";
  hint.textContent = "sending…";
  const { ok, payload } = await post("prompt", { text });
  if (ok) {
    hint.textContent = "";
    promptBox.value = "";
  } else {
    hint.className = "hint bad";
    hint.textContent = payload.error || payload.message || "the cockpit refused that";
  }
  poll();
});

document.getElementById("stop").addEventListener("click", async () => {
  await post("stop", {});
  poll();
});

promptBox.addEventListener("keydown", (event) => {
  if (event.key === "Enter" && !event.shiftKey) {
    event.preventDefault();
    document.getElementById("prompt-form").requestSubmit();
  }
});

field.addEventListener("load", () => {
  PIXELS_PER_METRE = (canvas.width / field.naturalWidth) * IMAGE_PIXELS_PER_METRE;
  draw();
});
poll();
setInterval(poll, POLL_MS);
