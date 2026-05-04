"""
Simple HTTP server to serve the frontend dashboard.
Run this alongside server.py so the WebSocket connection
uses the same hostname (avoids CORS issues).

Usage:
    python serve_frontend.py

This serves files from the 'public' directory on http://localhost:8080
"""

import http.server
import os
import sys

PORT = 8080
FRONTEND_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "public")


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    """Serves frontend files with suppressed request logging."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=FRONTEND_DIR, **kwargs)

    def log_message(self, format, *args):
        # Only log errors, not every GET request
        if args and '404' in str(args):
            super().log_message(format, *args)


def main():
    if not os.path.isdir(FRONTEND_DIR):
        print(f"[HTTP] ERROR: Frontend directory not found: {FRONTEND_DIR}")
        sys.exit(1)

    print(f"╔══════════════════════════════════════════════════════════╗")
    print(f"║  📡 Frontend HTTP Server                                ║")
    print(f"║  Serving: {FRONTEND_DIR:<46} ║")
    print(f"║  URL: http://localhost:{PORT:<40} ║")
    print(f"╚══════════════════════════════════════════════════════════╝")

    with http.server.HTTPServer(("0.0.0.0", PORT), QuietHandler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n[HTTP] Server stopped.")


if __name__ == "__main__":
    main()
