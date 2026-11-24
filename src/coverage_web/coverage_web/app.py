"""Operator web app: FastAPI + WebSocket. ROS-free on purpose.

The app only talks to a "bridge" object with two methods:
    snapshot() -> dict            latest robot status (state, coverage, map, pose, path)
    command(name) -> (bool, str)  forward start/pause/resume/estop/reset to the robot
The real bridge (ros_bridge.py) is a ROS 2 node; tests use a fake one.

Routes:
    GET  /                    operator page (static/index.html)
    GET  /api/status          one snapshot as JSON
    POST /api/command/{name}  start | pause | resume | estop | reset
    WS   /ws                  pushes a snapshot every `push_period` seconds
"""

from __future__ import annotations

import asyncio
from importlib import resources
from typing import Protocol, Tuple

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse

COMMANDS = ("start", "pause", "resume", "estop", "reset")


class Bridge(Protocol):
    def snapshot(self) -> dict: ...
    def command(self, name: str) -> Tuple[bool, str]: ...


def create_app(bridge: Bridge, push_period: float = 0.25) -> FastAPI:
    app = FastAPI(title="Coverage robot operator (simulation)")
    page = resources.files("coverage_web").joinpath("static/index.html").read_text("utf-8")

    @app.get("/", response_class=HTMLResponse)
    def index() -> str:
        return page

    @app.get("/api/status")
    def status() -> dict:
        return bridge.snapshot()

    @app.post("/api/command/{name}")
    def command(name: str) -> dict:  # sync def: FastAPI runs it in a worker thread
        if name not in COMMANDS:
            raise HTTPException(status_code=404, detail=f"unknown command '{name}'")
        ok, message = bridge.command(name)
        return {"command": name, "success": ok, "message": message}

    @app.websocket("/ws")
    async def ws(socket: WebSocket) -> None:
        await socket.accept()
        try:
            while True:
                await socket.send_json(bridge.snapshot())
                # Wait for the next push by LISTENING, not sleeping: a closed page
                # arrives as a disconnect message, which receive_text() raises as
                # WebSocketDisconnect. A send-only loop never notices (with older
                # Starlette it spins forever - found when the tests hung in Docker).
                try:
                    await asyncio.wait_for(socket.receive_text(), timeout=push_period)
                except asyncio.TimeoutError:
                    pass  # no message from the browser: time for the next update
        except (WebSocketDisconnect, RuntimeError):
            pass  # browser closed the page

    return app
