"""Serve the CCtrl Web Serial debugger on localhost."""
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

ROOT = Path(__file__).resolve().parent

class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT.parent), **kwargs)

    def do_GET(self):
        if self.path == "/":
            self.send_response(302)
            self.send_header("Location", "/usb_robot_debug/")
            self.end_headers()
            return
        super().do_GET()

print("CCtrl USB debugger: http://127.0.0.1:8768")
ThreadingHTTPServer(("127.0.0.1", 8768), Handler).serve_forever()
