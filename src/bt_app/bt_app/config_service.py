#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import signal
from threading import Event
from typing import Any

import yaml
import zmq

from bt_app.app import ConfigRegistry
from bt_app.common import ZMQ_CONFIG_ENDPOINT

CLI_CONFIG_ENDPOINT = "tcp://127.0.0.1:5555"


class ConfigDumpService:
    def __init__(
        self,
        *,
        yaml_path: str,
        endpoint: str = ZMQ_CONFIG_ENDPOINT,
        context: zmq.Context | None = None,
    ) -> None:
        self.endpoint = endpoint
        self.context = context or zmq.Context.instance()
        self.registry = ConfigRegistry(yaml_path)
        self.socket: zmq.Socket | None = None

    def start(self) -> None:
        if self.socket is not None:
            return

        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(self.endpoint)

    def close(self) -> None:
        if self.socket is not None:
            self.socket.close(linger=0)
            self.socket = None

    def _handle_request(self, request: Any) -> Any:
        if isinstance(request, str):
            if request.strip().lower() != "dump":
                return "error: unsupported request"
            return self.registry.dump_yaml()

        if not isinstance(request, dict):
            return {"ok": False, "error": "Unsupported request format"}

        command = request.get("command")
        if command == "dump":
            return {"ok": True, "params": self.registry.dump()}

        if command == "get":
            name = request.get("name")
            if not isinstance(name, str):
                return {"ok": False, "error": "Missing parameter name"}
            param = self.registry.params.get(name)
            if param is None:
                return {"ok": False, "error": f"Unknown parameter {name}"}
            return {"ok": True, "value": param.value}

        if command == "set":
            name = request.get("name")
            if not isinstance(name, str):
                return {"ok": False, "error": "Missing parameter name"}
            value = request.get("value")
            param = self.registry.params.get(name)
            if param is None:
                return {"ok": False, "error": f"Unknown parameter {name}"}

            try:
                parsed_value = self._coerce_value(value, param.type)
                result = self.registry.set(name, parsed_value)
            except Exception as exc:
                return {"ok": False, "error": str(exc)}
            return {"ok": True, "value": result}

        if command == "list":
            return {
                "ok": True,
                "params": {
                    name: {
                        "type": param.type,
                        "value": param.value,
                        "default": param.default,
                        "min": param.min,
                        "max": param.max,
                        "values": param.values,
                    }
                    for name, param in self.registry.params.items()
                },
            }

        if command == "save":
            self.registry.save()
            return {"ok": True}

        return {"ok": False, "error": f"Unsupported command {command}"}

    @staticmethod
    def _coerce_value(value: Any, param_type: str) -> Any:
        if param_type == "int":
            if isinstance(value, bool):
                raise ValueError("bool is not valid for int parameter")
            return int(value)
        if param_type == "float":
            return float(value)
        if param_type == "bool":
            if isinstance(value, bool):
                return value
            if isinstance(value, str):
                lowered = value.strip().lower()
                if lowered in {"true", "1", "yes", "on"}:
                    return True
                if lowered in {"false", "0", "no", "off"}:
                    return False
            raise ValueError("Value must be bool")
        if param_type == "str":
            return str(value)
        if param_type == "enum":
            return value
        return value

    def serve_forever(self, stop_event: Event, timeout_ms: int = 100) -> None:
        if self.socket is None:
            raise RuntimeError("ConfigDumpService.start() must be called before serve_forever")

        poller = zmq.Poller()
        poller.register(self.socket, zmq.POLLIN)

        while not stop_event.is_set():
            events = dict(poller.poll(timeout_ms))
            if self.socket not in events:
                continue

            raw_request = self.socket.recv()
            try:
                request = json.loads(raw_request.decode("utf-8"))
                response = self._handle_request(request)
                self.socket.send_json(response)
                continue
            except (json.JSONDecodeError, UnicodeDecodeError):
                pass

            request = raw_request.decode("utf-8")
            response = self._handle_request(request)
            self.socket.send_string(response)


class ConfigDumpClient:
    def __init__(
        self,
        *,
        endpoint: str = CLI_CONFIG_ENDPOINT,
        context: zmq.Context | None = None,
    ) -> None:
        self.endpoint = endpoint
        self.context = context or zmq.Context.instance()

    def request_dump_yaml(self, timeout_ms: int = 2000) -> str:
        socket = self.context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        socket.connect(self.endpoint)

        try:
            socket.send_string("dump")
            return socket.recv_string()
        finally:
            socket.close(linger=0)

    def request_dump(self, timeout_ms: int = 2000) -> dict[str, Any]:
        payload = self.request_dump_yaml(timeout_ms=timeout_ms)
        data = yaml.safe_load(payload) or {}
        if not isinstance(data, dict):
            raise ValueError("Config dump response is not a mapping")
        return data


def format_dump_table(data: dict[str, Any]) -> str:
    def format_value(value: Any) -> str:
        if isinstance(value, (str, int, float, bool)) or value is None:
            return str(value)
        return yaml.safe_dump(value, default_flow_style=True, sort_keys=True).strip().replace("\n", " ")

    rows = [("parameter", "value")]
    rows.extend((name, format_value(value)) for name, value in data.items())

    name_width = max(len(name) for name, _value in rows)
    value_width = max(len(value) for _name, value in rows)
    border = f"+-{'-' * name_width}-+-{'-' * value_width}-+"

    lines = [border]
    for index, (name, value) in enumerate(rows):
        lines.append(f"| {name.ljust(name_width)} | {value.ljust(value_width)} |")
        if index == 0:
            lines.append(border)
    lines.append(border)
    return "\n".join(lines)


def format_dump_tree(data: dict[str, Any]) -> str:
    grouped: dict[str, list[tuple[str, Any]]] = {}

    for name, value in sorted(data.items()):
        if "." in name:
            group, key = name.split(".", 1)
        else:
            group, key = "config", name
        grouped.setdefault(group, []).append((key, value))

    lines: list[str] = []
    for index, (group, items) in enumerate(grouped.items()):
        lines.append(group)
        for key, value in items:
            lines.append(f"  {key}: {value}")
        if index != len(grouped) - 1:
            lines.append("")

    return "\n".join(lines)


def parse_service_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Serve config commands over a ZMQ REP socket."
    )
    parser.add_argument("--yaml-path", default="/home/user/workspaces/beta_ws/src/bt_bringup/config/app.yaml")
    parser.add_argument("--endpoint", default=CLI_CONFIG_ENDPOINT)
    return parser.parse_args()


def parse_client_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fetch a YAML config dump over ZMQ and print it as a table."
    )
    parser.add_argument("--endpoint", default=CLI_CONFIG_ENDPOINT)
    parser.add_argument("--timeout-ms", type=int, default=2000)
    return parser.parse_args()


def service_main() -> None:
    args = parse_service_args()
    stop_event = Event()

    def stop_handler(signum: int, _frame: object) -> None:
        print(f"Stopping config dump service after signal {signum}")
        stop_event.set()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    service = ConfigDumpService(yaml_path=args.yaml_path, endpoint=args.endpoint)
    service.start()
    print(f"Serving config dump on {args.endpoint} from {args.yaml_path}")

    try:
        service.serve_forever(stop_event)
    finally:
        service.close()





if __name__ == "__main__":
    service_main()
