"""Loopback-only development HTTP interface; no jobs or live backend sessions."""

import json
import re
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
from pathlib import Path
from threading import Lock
from urllib.parse import urlsplit
from uuid import uuid4

from cutin.application import run_scenario, save_run
from cutin.application.storage import load_run_documents
from cutin.core import ScenarioConfig
from cutin.evaluation import EvaluationConfig
from cutin.evaluation.config import validate_evaluation_config
from cutin.simulation import SimulatorConfig
from cutin.simulation.config import validate_simulator_config

ID = re.compile(r"[A-Za-z0-9_-]{1,64}\Z")
MAX_BODY = 64 * 1024
MAX_ARTIFACT = 16 * 1024 * 1024


class WebServer(ThreadingHTTPServer):
    """One physical run at a time; reads/replay do not share simulation state."""

    def __init__(self, directory: Path, port: int = 8000, *, workflow=run_scenario):
        self.directory = directory.resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.workflow = workflow
        self.run_lock = Lock()
        super().__init__(("127.0.0.1", port), Handler)

    def run_directory(self, identifier: str) -> Path:
        if not ID.fullmatch(identifier):
            raise ValueError("invalid run id")
        directory = self.directory / identifier
        if not directory.resolve().is_relative_to(self.directory):
            raise ValueError("run must remain inside the artifact root")
        if not (directory / "log.json").is_file():
            directory = directory / "best"  # Existing CLI search winner.
        for name in ("log.json", "metrics.json"):
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
            self.respond(
                200,
                {
                    "scenario": ScenarioConfig().to_dict(),
                    "simulator": asdict(SimulatorConfig()),
                    "evaluation": asdict(EvaluationConfig()),
                },
            )
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
                directory = self.server.run_directory(path.removeprefix("/api/runs/"))
                for name in ("log", "metrics"):
                    if (directory / f"{name}.json").stat().st_size > MAX_ARTIFACT:
                        raise ValueError("artifact exceeds replay size limit")
                self.respond(200, load_run_documents(directory))
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
                "simulator",
                "evaluation",
            }:
                raise ValueError(
                    "request needs scenario and optional simulator/evaluation"
                )
            scenario = ScenarioConfig.from_dict(payload["scenario"])
            simulator = SimulatorConfig(**payload.get("simulator", {}))
            evaluation = EvaluationConfig(**payload.get("evaluation", {}))
            validate_simulator_config(simulator)
            validate_evaluation_config(evaluation)
            if scenario.duration_s > 60 or simulator.physics_hz > 120:
                raise ValueError(
                    "web runs are limited to 60 seconds and 120 physics Hz"
                )
        except (ValueError, TypeError, KeyError, OverflowError):
            self.respond(
                400, {"error": "invalid scenario/settings or web run limits exceeded"}
            )
            return
        except TimeoutError:
            self.respond(408, {"error": "request body timed out"})
            return
        if not self.server.run_lock.acquire(blocking=False):
            self.respond(409, {"error": "another run is in progress"})
            return
        try:
            result = self.server.workflow(scenario, simulator, evaluation)
            identifier = uuid4().hex
            save_run(result, self.server.directory / identifier)
            self.respond(201, {"id": identifier, "metrics": asdict(result.metrics)})
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
        print(f"Cut-in Web: http://127.0.0.1:{server.server_port}/", flush=True)
        print(f"Artifacts: {server.directory}", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
