import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location('mouth_depth', Path(__file__).resolve().parents[1] / 'integrations/d455/mouth_depth.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class DepthTests(unittest.TestCase):
    def test_valid_depth_and_sparse_outliers(self):
        d = np.full((480, 640), 650, np.uint16)
        d[240, 320] = 0
        d[241, 320] = 1800
        result = module.sample_depth(d, .001, 320, 240)
        self.assertEqual(result['depth_m'], .65)
        self.assertEqual(result['valid_pixels'], 48)

    def test_holes_and_edges_rejected(self):
        for d in [np.zeros((480, 640), np.uint16), np.full((480, 640), 3000, np.uint16)]:
            with self.assertRaises(ValueError):
                module.sample_depth(d, .001, 320, 240)
        d = np.full((480, 640), 650, np.uint16)
        d[:, 320:] = 900
        with self.assertRaises(ValueError):
            module.sample_depth(d, .001, 320, 240)

    def test_invalid_pixel_and_scale_rejected(self):
        d = np.full((480, 640), 650, np.uint16)
        for u, v, scale in [(0, 0, .001), (640, 480, .001), (float('nan'), 240, .001), (320, 240, 0)]:
            with self.assertRaises(ValueError):
                module.sample_depth(d, scale, u, v)
