"""EAT-RDK static demo and a bounded, read-only localhost camera proxy."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
from urllib.error import HTTPError, URLError
from urllib.parse import unquote, urlsplit
from urllib.request import ProxyHandler, build_opener

ROOT = Path(__file__).resolve().parent
PROXY_PATH = re.compile(r'/api/rgbd/(?:status|frame|point|depth/[A-Za-z0-9_-]+\.png)\Z')


class Handler(SimpleHTTPRequestHandler):
    extensions_map = {**SimpleHTTPRequestHandler.extensions_map, '.mjs': 'text/javascript',
                      '.js': 'text/javascript', '.wasm': 'application/wasm'}

    def __init__(self, *args, camera_port=5000, **kwargs):
        self.camera_port = camera_port
        super().__init__(*args, **kwargs)

    def reply(self, code, data, mime='application/json', headers=None):
        self.send_response(code)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        for key, value in (headers or {}).items():
            self.send_header(key, value)
        self.end_headers()
        if self.command != 'HEAD':
            self.wfile.write(data)

    def do_HEAD(self):
        self.do_GET()

    def do_GET(self):
        parsed = urlsplit(self.path)
        parts = unquote(parsed.path).split('/')
        if any(p == '..' or p.startswith('.') for p in parts if p):
            return self.send_error(404)
        if parsed.path == '/health':
            return self.reply(200, json.dumps({'ok': True, 'application': 'EAT-RDK',
                'mode': 'simulation_only', 'hardware_execution_enabled': False}).encode())
        if parsed.path in ('/', '/index.html'):
            self.send_response(302)
            self.send_header('Location', '/web/smart-feeding.html')
            self.end_headers()
            return
        if PROXY_PATH.fullmatch(parsed.path) and len(parsed.query) <= 512:
            url = f'http://127.0.0.1:{self.camera_port}'+parsed.path+('?' + parsed.query if parsed.query else '')
            try:
                # Ignore environment proxies for this local camera-only connection.
                with build_opener(ProxyHandler({})).open(url, timeout=2) as r:
                    body = r.read(4*1024*1024+1)
                    if len(body) > 4*1024*1024:
                        raise ValueError('response_too_large')
                    return self.reply(r.status, body, r.headers.get('Content-Type', 'application/json'),
                        {k:r.headers[k] for k in ['X-Depth-Scale','X-Frame-Id'] if k in r.headers})
            except HTTPError as e:
                return self.reply(e.code, e.read(65536))
            except (URLError, TimeoutError, ValueError, OSError):
                return self.reply(503, b'{"ok":false,"error":"local_camera_unavailable"}')
        if not (parsed.path.startswith(('/web/', '/models/')) or parsed.path == '/reports/scooping/scene.json'):
            return self.send_error(404)
        target = Path(self.translate_path(parsed.path)).resolve()
        if not target.is_relative_to(Path(self.directory).resolve()) or not target.is_file():
            return self.send_error(404)
        return super().do_GET()

    def log_message(self, *_):
        pass  # Do not persist mouth pixel queries or camera images.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--camera-port', type=int, default=5000)
    args = parser.parse_args()
    if not all(1 <= p <= 65535 for p in (args.port, args.camera_port)):
        parser.error('ports must be between 1 and 65535')
    server = ThreadingHTTPServer((args.host, args.port), partial(Handler, directory=str(ROOT), camera_port=args.camera_port))
    print(f'EAT-RDK simulation: http://{args.host}:{args.port}/', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
