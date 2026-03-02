#!/usr/bin/env python3
"""Standalone HTTP server for the Jugglebot GUI.

Serves static files from this directory on all interfaces.
Completely independent of ROS2 - start once and leave running.

Usage:
    python3 gui_server.py [--port 8081]
"""

import argparse
import functools
import http.server
import os


class CORSHandler(http.server.SimpleHTTPRequestHandler):
    """SimpleHTTPRequestHandler with CORS and correct MIME types."""

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def guess_type(self, path):
        # Ensure .js files are served as ES modules
        if path.endswith(".js"):
            return "application/javascript"
        if path.endswith(".mjs"):
            return "application/javascript"
        # glTF binary format
        if path.endswith(".glb"):
            return "model/gltf-binary"
        if path.endswith(".gltf"):
            return "model/gltf+json"
        return super().guess_type(path)


class ReusableHTTPServer(http.server.HTTPServer):
    """HTTPServer with SO_REUSEADDR enabled to avoid 'Address already in use'."""
    allow_reuse_address = True


def main():
    parser = argparse.ArgumentParser(description="Jugglebot GUI server")
    parser.add_argument("--port", type=int, default=8081, help="Port to serve on")
    args = parser.parse_args()

    directory = os.path.dirname(os.path.abspath(__file__))
    handler = functools.partial(CORSHandler, directory=directory)

    server = ReusableHTTPServer(("0.0.0.0", args.port), handler)
    print("Serving Jugglebot GUI on http://0.0.0.0:{}".format(args.port))
    print("  Directory: {}".format(directory))
    print("  Press Ctrl+C to stop")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.")
        server.server_close()


if __name__ == "__main__":
    main()
