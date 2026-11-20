from functools import partial
from http.server import ThreadingHTTPServer
import json
import threading
from types import SimpleNamespace
import unittest
from urllib.error import HTTPError
from urllib.request import urlopen
import numpy as np
from camera.realsense import FrameStore, Handler


class CameraApiTests(unittest.TestCase):
    def setUp(self):
        self.now = [1.0]
        self.store = FrameStore(lambda:self.now[0])
        intr = SimpleNamespace(width=640,height=480,fx=400.,fy=400.,ppx=320.,ppy=240.,model='test',coeffs=[0]*5)
        self.store.put(dict(id='fixture-1',received=1.,number=1,timestamp=0,scale=.001,
                            depth=np.full((480,640),650,np.uint16),intrinsics=intr))
        self.calls = []
        def deproject(i, uv, z):
            self.calls.append((uv,z))
            return [(uv[0]-i.ppx)*z/i.fx,(uv[1]-i.ppy)*z/i.fy,z]
        rs = SimpleNamespace(rs2_deproject_pixel_to_point=deproject)
        self.server = ThreadingHTTPServer(('127.0.0.1',0), partial(Handler,store=self.store,rs=rs,cv2=None))
        self.thread = threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.base = 'http://127.0.0.1:'+str(self.server.server_port)

    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join()

    def test_same_frame_point_uses_validated_median_and_sdk(self):
        with urlopen(self.base+'/api/rgbd/point?frame_id=fixture-1&u=320&v=240') as r:
            data=json.load(r)
        self.assertEqual(data['xyz_m'],[0.,0.,.65])
        self.assertEqual(data['valid_pixels'],49)
        self.assertEqual(self.calls,[([320.,240.],.65)])

    def test_missing_stale_or_invalid_pixel_rejected(self):
        for suffix in ['?u=320&v=240','?frame_id=unknown&u=320&v=240','?frame_id=fixture-1&u=nan&v=240']:
            with self.assertRaises(HTTPError) as e:
                urlopen(self.base+'/api/rgbd/point'+suffix)
            self.assertEqual(e.exception.code,422)
        self.now[0]=1.6
        with self.assertRaises(HTTPError) as e:
            urlopen(self.base+'/api/rgbd/status')
        self.assertEqual(e.exception.code,503)
        self.assertEqual(self.calls,[])
