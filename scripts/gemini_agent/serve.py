#!/usr/bin/env python3
"""The cockpit: a web page that shows the robot and lets you type instructions.

One process, one port, standard library only - so it forwards cleanly out of a
Codespace, where GitHub's own port authentication is the login.

    ./gradlew simulateJava &                     # the robot
    python3 scripts/gemini_agent/serve.py        # http://localhost:8000

The page draws the field from NetworkTables and shows the model's thoughts and
tool calls as they happen. Watching needs nothing. Typing an instruction runs the
same Gemini agent the CLI runs - and if AGENT_WEB_TOKEN is set, it needs that
token, so the page can be shared with spectators without handing out the API key.
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from agent import DEFAULT_MAX_STEPS, DEFAULT_MODEL, Session, make_client  # noqa: E402
from field_view import NAVGRID  # noqa: E402
from nt4 import RobotConnection  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WEB_DIR = os.path.join(REPO, "web")
FIELD_IMAGE = os.path.join(REPO, "scripts", "assets", "field26.png")

MAX_TRANSCRIPT = 200


class Cockpit:
    """Owns the robot connection, the model session, and what the page sees."""

    def __init__(self, robot: RobotConnection, session_factory, token: str | None):
        self.robot = robot
        self.session_factory = session_factory
        self.token = token
        self.session = None
        self.transcript: list[dict] = []
        self.busy = False
        self.error = ""
        self.lock = threading.Lock()

    # -- what the page reads ----------------------------------------------

    def state(self) -> dict:
        state = self.robot.state()
        return {
            "connected": self.robot.connected,
            "robot": state,
            "fuel": [[round(x, 2), round(y, 2)] for x, y in self.robot.detected_game_pieces()],
            "fuel_all": [[round(x, 2), round(y, 2)] for x, y in self.robot.field_game_pieces()],
            "trajectory": [[round(x, 3), round(y, 3)] for x, y in self.robot.trajectory()]
            if state["navigating"] else [],
            "transcript": self.transcript[-MAX_TRANSCRIPT:],
            "busy": self.busy,
            "error": self.error,
            "actions": self.robot.available_actions(),
            "landmarks": self.robot.landmarks(),
            "field_size": self.robot.field_size(),
            "needs_token": bool(self.token),
            "model": getattr(self.session, "model", None),
            "time": time.time(),
        }

    # -- what the page writes ---------------------------------------------

    def submit(self, instruction: str) -> tuple[int, str]:
        if self.busy:
            return 409, "the robot is already carrying out an instruction"
        with self.lock:
            if self.busy:
                return 409, "the robot is already carrying out an instruction"
            self.busy = True
        threading.Thread(target=self._run, args=(instruction,), daemon=True).start()
        return 202, "accepted"

    def stop(self) -> None:
        self.robot.set_action("STOP")
        self._event("stopped", "STOP - the path and the action were cancelled")

    def _run(self, instruction: str) -> None:
        try:
            if self.session is None:
                self.session = self.session_factory(self._event)
            self.error = ""
            self.session.ask(instruction)
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self._event("error", self.error)
        finally:
            self.busy = False

    def _event(self, kind: str, text: str) -> None:
        self.transcript.append({"t": time.time(), "kind": kind, "text": text})
        del self.transcript[:-MAX_TRANSCRIPT]


def make_handler(cockpit: Cockpit):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def do_GET(self):  # noqa: N802 - BaseHTTPRequestHandler's naming
            path = self.path.split("?")[0]
            if path == "/state":
                return self._json(200, cockpit.state())
            if path == "/navgrid.json":
                return self._file(NAVGRID, "application/json")
            if path == "/field26.png":
                return self._file(FIELD_IMAGE, "image/png")
            name = "index.html" if path in ("/", "") else path.lstrip("/")
            target = os.path.normpath(os.path.join(WEB_DIR, name))
            if not target.startswith(WEB_DIR) or not os.path.isfile(target):
                return self._json(404, {"error": "not found"})
            return self._file(target, mimetypes.guess_type(target)[0] or "text/plain")

        def do_POST(self):  # noqa: N802
            path = self.path.split("?")[0]
            body = self.rfile.read(int(self.headers.get("content-length") or 0))
            try:
                payload = json.loads(body or b"{}")
            except json.JSONDecodeError:
                return self._json(400, {"error": "bad JSON"})

            if cockpit.token and payload.get("token") != cockpit.token:
                return self._json(401, {"error": "wrong or missing token"})

            if path == "/prompt":
                instruction = (payload.get("text") or "").strip()
                if not instruction:
                    return self._json(400, {"error": "no instruction"})
                status, message = cockpit.submit(instruction)
                return self._json(status if status != 202 else 200,
                                  {"ok": status == 202, "message": message})
            if path == "/stop":
                cockpit.stop()
                return self._json(200, {"ok": True})
            return self._json(404, {"error": "not found"})

        def _json(self, status: int, payload: dict):
            self._send(status, json.dumps(payload).encode(), "application/json")

        def _file(self, path: str, content_type: str):
            with open(path, "rb") as handle:
                self._send(200, handle.read(), content_type)

        def _send(self, status: int, body: bytes, content_type: str):
            self.send_response(status)
            self.send_header("content-type", content_type)
            self.send_header("content-length", str(len(body)))
            self.send_header("cache-control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_args):
            pass  # the interesting log is the agent's, not one line per poll

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default=os.environ.get("ROBOT_HOST", "127.0.0.1"),
                        help="robot / simulator address (default 127.0.0.1)")
    parser.add_argument("--port", type=int, default=5810, help="NT4 port (default 5810)")
    parser.add_argument("--web-port", type=int, default=int(os.environ.get("PORT", 8000)),
                        help="port to serve the cockpit on (default 8000)")
    parser.add_argument("--model", default=os.environ.get("GEMINI_MODEL", DEFAULT_MODEL))
    parser.add_argument("--max-steps", type=int, default=DEFAULT_MAX_STEPS)
    args = parser.parse_args()

    print(f"connecting to the robot at {args.host}:{args.port} ...", flush=True)
    try:
        robot = RobotConnection(args.host, args.port, name="gemini-cockpit").connect()
    except Exception as exc:
        print(f"{exc}\nIs the robot code running? Try ./gradlew simulateJava", file=sys.stderr)
        return 2
    print(f"connected. actions: {', '.join(robot.available_actions())}", flush=True)

    api_key = os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")
    if not api_key:
        print("GEMINI_API_KEY is not set - the page will show the robot but cannot drive it.",
              file=sys.stderr)

    def session_factory(on_event):
        if not api_key:
            raise RuntimeError("GEMINI_API_KEY is not set on the machine running the cockpit")
        return Session(make_client(api_key), robot, args.model, args.max_steps,
                       verbose=True, on_event=on_event)

    cockpit = Cockpit(robot, session_factory, os.environ.get("AGENT_WEB_TOKEN") or None)
    server = ThreadingHTTPServer(("0.0.0.0", args.web_port), make_handler(cockpit))
    print(f"cockpit on http://localhost:{args.web_port}"
          + ("  (prompts need AGENT_WEB_TOKEN)" if cockpit.token else ""), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print()
    finally:
        robot.set_action("STOP")
        time.sleep(0.2)
        robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
