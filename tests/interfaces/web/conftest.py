"""Web test fixtures: a fast synthetic runner and a loopback server."""

import json
from http.client import HTTPConnection
from threading import Thread

import pytest

from cutin.core.record import Frame, ScenarioRecord, VehicleState
from cutin.interfaces.web.server import WebServer


def _vehicle(x, y=4.0, speed=25.0, collided=False, first=False):
    return VehicleState(
        x_m=x,
        y_m=y,
        heading_rad=0.0,
        speed_mps=speed,
        length_m=5.0,
        width_m=2.0,
        collided=collided,
        acceleration_mps2=None if first else 0.0,
        steering_rad=None if first else 0.0,
    )


def _record(scenario, simulation, *, collide=False) -> ScenarioRecord:
    """Three ticks of straight driving; optionally ending in an ego collision."""
    frames = []
    for tick in range(3):
        hit = collide and tick == 2
        frames.append(
            Frame(
                tick=tick,
                ego=_vehicle(50 + tick, collided=hit, first=tick == 0),
                front=_vehicle(110 + tick, first=tick == 0),
                cutin=_vehicle(70 + tick, y=0.0, collided=hit, first=tick == 0),
                cutin_phase="warmup",
            )
        )
    return ScenarioRecord(
        scenario=scenario,
        simulation=simulation,
        backend_version="test",
        controller_version="test",
        frames=frames,
        termination_reason="collision" if collide else "horizon",
        prepare_start_time_s=None,
        merge_start_time_s=None,
        merge_completion_time_s=None,
    )


@pytest.fixture
def web(tmp_path):
    server = WebServer(tmp_path, 0, runner=_record)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield server
    server.shutdown()
    server.server_close()
    thread.join()


@pytest.fixture
def defaults(web):
    return json.loads(_request(web, "GET", "/api/defaults")[1])


def _request(server, method, path, data=None, headers=None):
    connection = HTTPConnection("127.0.0.1", server.server_port, timeout=10)
    body = json.dumps(data) if data is not None else None
    default = (
        {
            "Origin": f"http://127.0.0.1:{server.server_port}",
            "Content-Type": "application/json",
            "X-Cutin-Request": "1",
        }
        if method == "POST"
        else {}
    )
    default.update(headers or {})
    connection.request(method, path, body=body, headers=default)
    response = connection.getresponse()
    content = response.read()
    status, response_headers = response.status, dict(response.getheaders())
    connection.close()
    return status, content, response_headers


@pytest.fixture
def http():
    return _request


@pytest.fixture
def make_record():
    return _record


@pytest.fixture
def make_vehicle():
    return _vehicle
