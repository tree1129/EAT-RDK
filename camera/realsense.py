"""Standalone D455 RGB-D GET API. Run with: python -m camera.realsense.

Camera capture only. Requires a working pyrealsense2 installation in the selected
Python environment. No robot SDK, motor interface, or system service changes.
"""
import argparse
import base64
from collections import OrderedDict
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import threading
import time
from urllib.parse import parse_qs, urlsplit
import uuid


class FrameStore:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.frames = OrderedDict()
        self.lock = threading.Lock()
        self.error = 'camera_starting'

    def put(self, frame):
        with self.lock:
            self.frames[frame['id']] = frame
            while len(self.frames) > 45:
                self.frames.popitem(last=False)
            self.error = ''

    def get(self, frame_id=None):
        with self.lock:
            frame = self.frames.get(frame_id) if frame_id else next(reversed(self.frames.values()), None)
        if frame is None:
            raise ValueError('frame_not_available')
        if self.age(frame) > 500:
            raise ValueError('stale_camera_frame')
        return frame

    def age(self, frame):
        return (self.clock()-frame['received'])*1000


def intrinsics_dict(i):
    return dict(width=i.width, height=i.height, fx=i.fx, fy=i.fy,
                ppx=i.ppx, ppy=i.ppy, model=str(i.model), coeffs=list(i.coeffs))


class Handler(BaseHTTPRequestHandler):
    def __init__(self, *args, store, rs, cv2, **kwargs):
        self.store, self.rs, self.cv2 = store, rs, cv2
        super().__init__(*args, **kwargs)

    def reply(self, code, value, mime='application/json', headers=None):
        data = json.dumps(value, allow_nan=False).encode() if isinstance(value, dict) else value
        self.send_response(code)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        for k, v in (headers or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        from integrations.d455.mouth_depth import sample_depth
        parsed = urlsplit(self.path)
        try:
            if parsed.path == '/api/rgbd/status':
                f = self.store.get()
                return self.reply(200, dict(ok=True, mode='camera_only', aligned_to='color',
                    frame_id=f['id'], age_ms=self.store.age(f), depth_scale=f['scale'],
                    intrinsics=intrinsics_dict(f['intrinsics'])))
            if parsed.path == '/api/rgbd/frame':
                f = self.store.get()
                ok, encoded = self.cv2.imencode('.jpg', f['color'], [self.cv2.IMWRITE_JPEG_QUALITY, 85])
                if not ok:
                    raise ValueError('jpeg_encoding_failed')
                return self.reply(200, dict(ok=True, frame_id=f['id'], frame_number=f['number'],
                    age_ms=self.store.age(f), timestamp_ms=f['timestamp'],
                    width=f['color'].shape[1], height=f['color'].shape[0], aligned_to='color',
                    intrinsics=intrinsics_dict(f['intrinsics']), rgb_jpeg_base64=base64.b64encode(encoded).decode()))
            if parsed.path == '/api/rgbd/point':
                q = parse_qs(parsed.query)
                frame_id = q.get('frame_id', [''])[0]
                if not frame_id:
                    raise ValueError('frame_id_required')
                f = self.store.get(frame_id)
                u, v = float(q['u'][0]), float(q['v'][0])
                quality = sample_depth(f['depth'], f['scale'], u, v)
                xyz = self.rs.rs2_deproject_pixel_to_point(f['intrinsics'], [u, v], quality['depth_m'])
                if not all(math.isfinite(x) for x in xyz):
                    raise ValueError('invalid_deprojection')
                return self.reply(200, dict(ok=True, frame_id=f['id'], pixel=[u,v], xyz_m=xyz,
                    age_ms=self.store.age(f), coordinate_frame='color_camera_optical', **quality))
            if parsed.path.startswith('/api/rgbd/depth/') and parsed.path.endswith('.png'):
                f = self.store.get(parsed.path.rsplit('/',1)[-1][:-4])
                ok, encoded = self.cv2.imencode('.png', f['depth'])
                if not ok:
                    raise ValueError('png_encoding_failed')
                return self.reply(200, encoded.tobytes(), 'image/png',
                    {'X-Depth-Scale':str(f['scale']), 'X-Frame-Id':f['id']})
            return self.reply(404, dict(ok=False, error='unknown_endpoint'))
        except (ValueError, KeyError, IndexError) as e:
            return self.reply(503 if parsed.path.endswith(('/status','/frame')) else 422,
                              dict(ok=False, error=str(e)))

    def log_message(self, *_):
        pass


def capture(pipe, rs, np, store, stop, scale):
    align, session, previous = rs.align(rs.stream.color), uuid.uuid4().hex[:12], None
    while not stop.is_set():
        try:
            raw = pipe.wait_for_frames(timeout_ms=1000)
            received = store.clock()
            frames = align.process(raw)
            depth, color = frames.get_depth_frame(), frames.get_color_frame()
            if not depth or not color or color.get_frame_number() == previous:
                continue
            previous = color.get_frame_number()
            store.put(dict(id=f'{session}-{previous}', number=previous, received=received,
                timestamp=color.get_timestamp(), scale=scale,
                intrinsics=depth.profile.as_video_stream_profile().intrinsics,
                depth=np.asanyarray(depth.get_data()).copy(), color=np.asanyarray(color.get_data()).copy()))
        except Exception as e:
            store.error = str(e)
            stop.wait(.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=5000)
    args = parser.parse_args()
    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
    except ImportError as e:
        parser.exit(1, f'Missing camera dependency: {e}. See docs/RDK_X5.md.\n')
    store, stop = FrameStore(), threading.Event()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), partial(Handler, store=store, rs=rs, cv2=cv2))
    pipe, config, started, thread = rs.pipeline(), rs.config(), False, None
    config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
    config.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
    try:
        profile = pipe.start(config)
        started = True
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        thread = threading.Thread(target=capture,args=(pipe,rs,np,store,stop,scale),daemon=True)
        thread.start()
        print(f'RGB-D camera only: http://127.0.0.1:{args.port}/api/rgbd/status', flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if thread:
            thread.join(timeout=2)
        if started:
            pipe.stop()
        server.server_close()


if __name__ == '__main__':
    main()
