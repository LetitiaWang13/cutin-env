"""Loopback-only development HTTP interface; no jobs or live backend sessions."""

import json
import re
from dataclasses import asdict
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
from pathlib import Path
from threading import Lock
from urllib.parse import unquote, urlsplit
from uuid import uuid4

from cutin.config import SimulationConfig
from cutin.core.record import ScenarioRecord
from cutin.core.scenario import ScenarioConfig
from cutin.interfaces.web import inputs, storage
from cutin.interfaces.web.summary import Warnings, summarize

ID = re.compile(r"[A-Za-z0-9_-]+(?: [A-Za-z0-9_-]+)*\Z")
RESERVED_NAME = "default"  # Shown by the UI on load, never saved.
MAX_BODY = 64 * 1024
MAX_ARTIFACT = 16 * 1024 * 1024
MAX_NAME = 40


def clean_name(value) -> str:
    """Run name that doubles as its folder name: letters, digits, space, _ and -."""
    if not isinstance(value, str):
        raise ValueError("name must be text")
    name = " ".join(value.split())
    if not 1 <= len(name) <= MAX_NAME or not ID.fullmatch(name):
        raise ValueError(
            f"name must be 1..{MAX_NAME} characters: letters, digits, spaces, _ or -"
        )
    if name.casefold() == RESERVED_NAME:
        raise ValueError(f'"{RESERVED_NAME}" is reserved for the built-in default run')
    return name


def run_scenario(
    scenario: ScenarioConfig, simulation: SimulationConfig
) -> ScenarioRecord:
    """Default runner. The backend is imported lazily so the page can start fast."""
    from cutin.simulation.adapters.highway_env.adapter import HighwayEnvAdapter

    return HighwayEnvAdapter(simulation).run(scenario)


class WebServer(ThreadingHTTPServer):
    """One physical run at a time; reads/replay do not share simulation state."""

    def __init__(self, directory: Path, port: int = 8000, *, runner=run_scenario):
        self.directory = directory.resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.runner = runner
        self.run_lock = Lock()
        super().__init__(("127.0.0.1", port), Handler)

    def run_directory(self, identifier: str) -> Path:
        if len(identifier) > 64 or not ID.fullmatch(identifier):
            raise ValueError("invalid run id")
        directory = self.directory / identifier
        if not directory.resolve().is_relative_to(self.directory):
            raise ValueError("run must remain inside the artifact root")
        for name in storage.FILES:
            path = directory / name
            if not path.resolve().is_relative_to(self.directory):
                raise ValueError("artifact must remain inside the artifact root")
            if not path.is_file():
                raise FileNotFoundError("run artifacts not found")
        return directory


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(10)

    def log_message(self, format, *args):
        # Keep local experiments quiet; no external logging or analytics.
        pass

    def respond(self, status, data, content_type="application/json"):
        body = (
            json.dumps(data, allow_nan=False).encode()
            if content_type == "application/json"
            else data
        )
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; frame-ancestors 'none'; base-uri 'none'",
        )
        self.end_headers()
        self.wfile.write(body)

    def allowed(self, *, mutation=False):
        port = self.server.server_port
        host = self.headers.get("Host")
        hosts = {f"127.0.0.1:{port}", f"localhost:{port}"}
        if port == 80:
            hosts.update({"127.0.0.1", "localhost"})
        if host not in hosts:
            self.respond(403, {"error": "invalid Host"})
            return False
        origin = self.headers.get("Origin")
        if (origin is not None and origin != f"http://{host}") or (
            mutation and origin is None
        ):
            self.respond(403, {"error": "same-origin requests only"})
            return False
        if mutation and self.headers.get("X-Cutin-Request") != "1":
            self.respond(403, {"error": "missing request header"})
            return False
        return True

    def do_GET(self):
        if not self.allowed():
            return
        path = urlsplit(self.path).path
        assets = {
            "/": ("index.html", "text/html; charset=utf-8"),
            "/app.js": ("app.js", "text/javascript; charset=utf-8"),
            "/style.css": ("style.css", "text/css; charset=utf-8"),
        }
        if path in assets:
            name, kind = assets[path]
            self.respond(
                200,
                files("cutin.interfaces.web").joinpath("assets", name).read_bytes(),
                kind,
            )
        elif path == "/api/defaults":
            self.respond(200, inputs.defaults())
        elif path == "/api/runs":
            ids = []
            for candidate in sorted(self.server.directory.iterdir()):
                try:
                    self.server.run_directory(candidate.name)
                except (OSError, ValueError):
                    continue
                ids.append(candidate.name)
            self.respond(200, {"ids": ids})
        elif path.startswith("/api/runs/"):
            try:
                directory = self.server.run_directory(
                    unquote(path.removeprefix("/api/runs/"))
                )
                for name in storage.FILES:
                    if (directory / name).stat().st_size > MAX_ARTIFACT:
                        raise ValueError("artifact exceeds replay size limit")
                self.respond(200, storage.load_run(directory))
            except FileNotFoundError:
                self.respond(404, {"error": "run not found"})
            except (OSError, ValueError):
                self.respond(400, {"error": "invalid or inaccessible run artifacts"})
        else:
            self.respond(404, {"error": "not found"})

    def do_POST(self):
        if not self.allowed(mutation=True):
            return
        if self.path != "/api/runs":
            self.respond(404, {"error": "not found"})
            return
        if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
            self.respond(415, {"error": "application/json required"})
            return
        if self.headers.get("Transfer-Encoding") is not None:
            self.respond(400, {"error": "chunked requests are not supported"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0
        if length < 1 or length > MAX_BODY:
            self.respond(413, {"error": "request must be 1..65536 bytes"})
            return
        try:
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict) or set(payload) - {
                "scenario",
                "simulation",
                "evaluation",
                "name",
                "save",
                "vehicle",
            }:
                raise ValueError(
                    "request needs scenario and optional simulation/evaluation"
                )
            if (
                payload.get("vehicle", inputs.defaults()["vehicle"])
                != (inputs.defaults()["vehicle"])
            ):
                raise ValueError("vehicle dimensions are fixed by the backend")
            save = payload.get("save", True)
            if not isinstance(save, bool):
                raise ValueError("save must be true or false")
            name = clean_name(payload["name"]) if "name" in payload and save else None
            scenario = inputs.parse_scenario(payload["scenario"])
            simulation = inputs.parse_simulation(
                payload.get("simulation", inputs.defaults()["simulation"])
            )
            warnings = inputs.parse_warnings(
                payload.get("evaluation", inputs.defaults()["evaluation"])
            )
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            self.respond(400, {"error": str(error)})
            return
        except TimeoutError:
            self.respond(408, {"error": "request body timed out"})
            return
        if not self.server.run_lock.acquire(blocking=False):
            self.respond(409, {"error": "another run is in progress"})
            return
        try:
            record = self.server.runner(scenario, simulation)
            metrics = summarize(record, Warnings(**asdict(warnings)))
            if not save:  # Built-in default: replayed in the page, nothing on disk.
                self.respond(200, storage.documents(record, metrics))
                return
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S")  # Local time.
            identifier = (
                f"{stamp}-{name}" if name else f"frontend-{stamp}-{uuid4().hex[:8]}"
            )
            if identifier.casefold() in {
                path.name.casefold() for path in self.server.directory.iterdir()
            }:
                raise ValueError(f'run "{identifier}" already exists; try again')
            storage.save_run(record, metrics, self.server.directory / identifier)
            self.respond(201, {"id": identifier, "name": name, "metrics": metrics})
        except (ValueError, TypeError) as error:
            self.respond(400, {"error": str(error)})
        except Exception:
            self.respond(500, {"error": "run or persistence failed"})
        finally:
            self.server.run_lock.release()


def serve(directory: Path, port: int = 8000) -> None:
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError("port must be 1..65535")
    with WebServer(directory, port) as server:
        print(
            f"Critical Scenario Generator: http://127.0.0.1:{server.server_port}/",
            flush=True,
        )
        print(f"Artifacts: {server.directory}", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
