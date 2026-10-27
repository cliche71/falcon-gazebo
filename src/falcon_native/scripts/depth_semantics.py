"""Depth input adaptation only; no map, pose or timing changes."""
import numpy as np

MODES = ('legacy', 'official_compatible')

def convert_depth(depth, mode, near=0.1, far=500.0):
    if mode not in MODES:
        raise ValueError('Unsupported depth mode: ' + mode)
    finite = np.isfinite(depth)
    if mode == 'legacy':
        valid = finite & (depth > 0)
        values = depth.copy()
        values[~valid] = 0
        return values, {}
    if near != 0.1 or far != 500.0:
        raise ValueError('official_compatible requires actual camera near/far 0.1/500.0')
    # Same contract as OpenNI C++: promote each float VALUE to double, then
    # compare to the double boundary. This is a conversion, never a view.
    numeric = depth.astype(np.float64)
    # Far is supported only at exactly 500; genuine finite hits stay unchanged.
    hit = finite & (numeric > near) & (numeric < far)
    background = finite & (numeric == far)
    categories = {
        'finite_hit': hit, 'finite_far_background': background,
        'nan': np.isnan(depth), 'positive_inf': np.isposinf(depth),
        'negative_inf': np.isneginf(depth), 'zero': finite & (depth == 0),
        'negative': finite & (numeric < 0),
        'positive_at_or_below_near': finite & (numeric > 0) & (numeric <= near),
        'unsupported_above_far': finite & (numeric > far),
    }
    values = depth.copy()
    values[~(hit | background)] = 0
    # The confirmed finite far value is already the official 500 m no-hit code.
    return values, {name: int(mask.sum()) for name, mask in categories.items()}
