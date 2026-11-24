"""Tests for the operator web app with a fake bridge (no ROS needed)."""

import pytest
from fastapi.testclient import TestClient

from coverage_web.app import COMMANDS, create_app


class FakeBridge:
    def __init__(self):
        self.calls = []
        self.state = "RUNNING"

    def snapshot(self):
        return {"state": self.state, "connected": True, "coverage": 42.5,
                "grid": {"w": 2, "h": 1, "res": 0.5, "cells": "12"},
                "path": [[0.0, 0.25, False], [1.0, 0.25, True]], "pose": [0.5, 0.25]}

    def command(self, name):
        self.calls.append(name)
        if name == "estop":
            self.state = "ESTOP"
        return True, f"accepted, state is now {self.state}"


@pytest.fixture
def client_and_bridge():
    bridge = FakeBridge()
    return TestClient(create_app(bridge, push_period=0.01)), bridge


def test_page_is_served_with_estop_button(client_and_bridge):
    client, _ = client_and_bridge
    r = client.get("/")
    assert r.status_code == 200
    assert "E-STOP" in r.text and "simulation only" in r.text
    assert "<script src=" not in r.text  # self-contained: works without internet on site


def test_status_returns_snapshot(client_and_bridge):
    client, _ = client_and_bridge
    s = client.get("/api/status").json()
    assert s["state"] == "RUNNING" and s["coverage"] == 42.5 and s["grid"]["cells"] == "12"


@pytest.mark.parametrize("name", COMMANDS)
def test_every_command_is_forwarded(client_and_bridge, name):
    client, bridge = client_and_bridge
    r = client.post(f"/api/command/{name}").json()
    assert r["success"] is True and r["command"] == name
    assert bridge.calls == [name]


def test_unknown_command_is_rejected_and_not_forwarded(client_and_bridge):
    client, bridge = client_and_bridge
    assert client.post("/api/command/self_destruct").status_code == 404
    assert bridge.calls == []


def test_websocket_pushes_live_status(client_and_bridge):
    client, bridge = client_and_bridge
    with client.websocket_connect("/ws") as ws:
        assert ws.receive_json()["state"] == "RUNNING"
        client.post("/api/command/estop")
        states = [ws.receive_json()["state"] for _ in range(3)]
        assert states[-1] == "ESTOP"
