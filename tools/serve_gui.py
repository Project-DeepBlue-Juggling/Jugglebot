"""Serve the GUI preview with correct MIME types, including on Windows.

Usage: python tools/serve_gui.py --port 8081
This serves static files only; it does not start ROS or send robot commands.
"""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class GUIHandler(SimpleHTTPRequestHandler):
    # Windows registry associations can otherwise label .js as text/plain.
    extensions_map = {
        **SimpleHTTPRequestHandler.extensions_map,
        '.js': 'text/javascript',
        '.mjs': 'text/javascript',
        '.glb': 'model/gltf-binary',
    }

    def do_GET(self):
        # A preview should always reflect edits, including corrected MIME types.
        if 'If-Modified-Since' in self.headers:
            del self.headers['If-Modified-Since']
        super().do_GET()

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8081)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1] / 'ros_ws/gui'
    handler = partial(GUIHandler, directory=str(root))
    with ThreadingHTTPServer(('127.0.0.1', args.port), handler) as server:
        print('GUI preview: http://localhost:{}/test_robot_models.html'.format(args.port), flush=True)
        server.serve_forever()
