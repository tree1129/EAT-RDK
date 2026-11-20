from functools import partial
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import tempfile
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from camera.realsense import FrameStore
from server import Handler


class FrameTests(unittest.TestCase):
    def test_expired_or_unknown_frame_never_falls_back(self):
        now = [1.0]
        store = FrameStore(lambda: now[0])
        store.put({'id':'known','received':1.0})
        self.assertEqual(store.get('known')['id'], 'known')
        with self.assertRaisesRegex(ValueError, 'frame_not_available'):
            store.get('unknown')
        now[0] = 1.501
        with self.assertRaisesRegex(ValueError, 'stale_camera_frame'):
            store.get('known')

    def test_frame_memory_is_bounded(self):
        store = FrameStore(lambda: 1.0)
        for i in range(80):
            store.put({'id':str(i),'received':1.0})
        self.assertEqual(len(store.frames), 45)
        self.assertEqual(store.get()['id'], '79')


class WebTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        (self.root/'web').mkdir()
        (self.root/'web/smart-feeding.html').write_text('EAT-RDK')
        (self.root/'web/core.mjs').write_text('export const mode="simulation_only";')
        (self.root/'web/.private').write_text('secret')
        (self.root/'server.py').write_text('private source')
        self.server = ThreadingHTTPServer(('127.0.0.1',0), partial(Handler,directory=str(self.root)))
        self.thread = threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()
        self.base = 'http://127.0.0.1:'+str(self.server.server_port)

    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join();self.tmp.cleanup()

    def test_entry_health_and_module_mime(self):
        with urlopen(self.base) as r:
            self.assertEqual(r.read(), b'EAT-RDK')
        with urlopen(self.base+'/health') as r:
            self.assertFalse(json.load(r)['hardware_execution_enabled'])
        with urlopen(self.base+'/web/core.mjs') as r:
            self.assertEqual(r.headers.get_content_type(), 'text/javascript')

    def test_no_directory_listing_traversal_or_private_files(self):
        for path in ['/web/','/server.py','/web/%2e%2e/server.py','/web/.private','/api/move']:
            with self.assertRaises(HTTPError) as e:
                urlopen(self.base+path)
            self.assertEqual(e.exception.code,404)

    def test_head_has_no_body_and_post_cannot_actuate(self):
        with urlopen(Request(self.base+'/health',method='HEAD')) as r:
            self.assertEqual(r.read(), b'')
        with self.assertRaises(HTTPError) as e:
            urlopen(Request(self.base+'/api/rgbd/point',method='POST'))
        self.assertEqual(e.exception.code,501)
