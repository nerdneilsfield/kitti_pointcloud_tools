#!/usr/bin/env python3
"""Serve KPT Web with headers required by SharedArrayBuffer."""

from __future__ import annotations

import argparse
import http.server
import pathlib
import socketserver


class KptHandler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {
        **http.server.SimpleHTTPRequestHandler.extensions_map,
        ".wasm": "application/wasm",
        ".js": "text/javascript",
    }

    def end_headers(self) -> None:
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
        self.send_header("Cross-Origin-Resource-Policy", "same-origin")
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "directory",
        nargs="?",
        type=pathlib.Path,
        default=pathlib.Path("build/web-release/site"),
    )
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    handler = lambda *a, **kw: KptHandler(  # noqa: E731
        *a, directory=str(args.directory.resolve()), **kw
    )
    with socketserver.ThreadingTCPServer((args.bind, args.port), handler) as server:
        print(f"Serving {args.directory} at http://{args.bind}:{args.port}")
        server.serve_forever()


if __name__ == "__main__":
    main()
