# bt_cli.py

import json
import shlex
import typer
import zmq

from prompt_toolkit import PromptSession
from prompt_toolkit.key_binding import KeyBindings


app = typer.Typer()
param_app = typer.Typer(invoke_without_command=True)
app.add_typer(param_app, name="param")

ZMQ_ENDPOINT = "tcp://127.0.0.1:5555"


def request(payload: dict) -> dict:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REQ)

    try:
        sock.connect(ZMQ_ENDPOINT)
        sock.send_json(payload)
        reply = sock.recv_json()

        if not reply.get("ok"):
            raise RuntimeError(reply.get("error", "Unknown error"))

        return reply

    finally:
        sock.close()


def print_dump(params: dict):
    for name, value in params.items():
        print(f"{name}: {value}")


def cmd_get(name: str):
    reply = request({
        "command": "get",
        "name": name,
    })
    print(reply["value"])


def cmd_set(name: str, value: str):
    reply = request({
        "command": "set",
        "name": name,
        "value": value,
    })
    print(f"{name} = {reply['value']}")


def cmd_list():
    reply = request({
        "command": "list",
    })

    for name, spec in reply["params"].items():
        value = spec.get("value")
        typ = spec.get("type")
        parts = [f"{name}: {value} ({typ})"]

        min_value = spec.get("min")
        max_value = spec.get("max")
        enum_values = spec.get("values")

        if min_value is not None or max_value is not None:
            parts.append(f"limits=[{min_value}, {max_value}]")

        if enum_values:
            parts.append(f"enums={enum_values}")

        print(", ".join(parts))


def cmd_dump(json_output: bool = False):
    reply = request({
        "command": "dump",
    })

    params = reply["params"]

    if json_output:
        print(json.dumps(params, indent=2))
    else:
        print_dump(params)


def cmd_save():
    request({
        "command": "save",
    })
    print("Saved")


def run_shell():
    bindings = KeyBindings()

    @bindings.add("escape")
    def _(event):
        event.app.exit(exception=KeyboardInterrupt)

    session = PromptSession(
        "bt-param> ",
        key_bindings=bindings,
    )

    print("Parameter shell. Press Esc, Ctrl-C, or type exit to quit.")

    while True:
        try:
            line = session.prompt()
            args = shlex.split(line)

            if not args:
                continue

            command = args[0]

            if command == "get":
                if len(args) != 2:
                    print("Usage: get <name>")
                    continue
                cmd_get(args[1])

            elif command == "set":
                if len(args) != 3:
                    print("Usage: set <name> <value>")
                    continue
                cmd_set(args[1], args[2])

            elif command == "list":
                cmd_list()

            elif command == "dump":
                json_output = "--json" in args
                cmd_dump(json_output=json_output)

            elif command == "save":
                cmd_save()

            elif command in {"exit", "quit"}:
                print("Bye")
                break

            elif command == "help":
                print("""
Commands:
  get <name>
  set <name> <value>
  list
  dump
  dump --json
  save
  exit
""".strip())

            else:
                print(f"Unknown command: {command}")

        except KeyboardInterrupt:
            print("\nBye")
            break

        except EOFError:
            print("\nBye")
            break

        except Exception as e:
            print(f"Error: {e}")


@param_app.callback(invoke_without_command=True)
def param_callback(ctx: typer.Context):
    if ctx.invoked_subcommand is None:
        run_shell()


@param_app.command("get")
def get_param(name: str):
    cmd_get(name)


@param_app.command("set")
def set_param(name: str, value: str):
    cmd_set(name, value)


@param_app.command("list")
def list_params():
    cmd_list()


@param_app.command("dump")
def dump_params(
    json_output: bool = typer.Option(False, "--json", help="Print dump as JSON"),
):
    cmd_dump(json_output=json_output)


@param_app.command("save")
def save_params():
    cmd_save()


if __name__ == "__main__":
    app()
