"""Depth quality gate for a mouth pixel, in aligned color coordinates."""
import numpy as np


def sample_depth(depth, scale, u, v):
    if not np.isfinite([u, v, scale]).all() or scale <= 0:
        raise ValueError('invalid_pixel_or_scale')
    h, w = depth.shape
    if not (3 <= u < w-3 and 3 <= v < h-3):
        raise ValueError('pixel_out_of_bounds')
    x, y = int(round(u)), int(round(v))
    if not (3 <= x < w-3 and 3 <= y < h-3):
        raise ValueError('pixel_out_of_bounds')
    patch = depth[y-3:y+4, x-3:x+4].astype(float) * scale
    values = patch[(patch >= .2) & (patch <= 2.)]
    if values.size < 25:
        raise ValueError('insufficient_depth')
    z = float(np.median(values))
    spread = float(np.quantile(values, .9) - np.quantile(values, .1))
    if spread > .04:
        raise ValueError('depth_edge_or_unstable_surface')
    return dict(depth_m=z, valid_pixels=int(values.size), patch_pixels=int(patch.size), spread_m=spread)
