"""Command line: `cutin web` serves the local page; other commands are pending."""

import argparse
import sys
from pathlib import Path


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="cutin", description="Critical scenario generator"
    )
    commands = parser.add_subparsers(dest="command", required=True)
    web = commands.add_parser("web", help="Local form and run replay")
    web.add_argument("--port", type=int, default=8000)
    web.add_argument("--runs-directory", type=Path, default=Path("runs"))
    args = parser.parse_args(argv)
    try:
        # Imported here so `cutin --help` does not load the Web server.
        from cutin.interfaces.web import serve

        serve(args.runs_directory, args.port)
    except ValueError as error:
        print(f"cutin: {error}", file=sys.stderr)
        raise SystemExit(2) from error
