"""Measured-pose interpolation and URDF transforms; no ROS dependencies."""
import math
import xml.etree.ElementTree as ET
import numpy as np


def rotation_rpy(values):
    r, p, y = values
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def quaternion(value):
    q = np.asarray(value, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or not 0.99 <= np.linalg.norm(q) <= 1.01:
        raise ValueError('Invalid measured quaternion')
    return q / np.linalg.norm(q)


def quat_matrix(value):
    x, y, z, w = quaternion(value)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def quat_product(a, b):
    x, y, z, w = a
    u, v, t, s = b
    return np.array([w*u+x*s+y*t-z*v, w*v-x*t+y*s+z*u,
                     w*t+x*v-y*u+z*s, w*s-x*u-y*v-z*t])


def slerp(a, b, fraction):
    a, b = quaternion(a), quaternion(b)
    dot = float(a.dot(b))
    if dot < 0:
        b, dot = -b, -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return quaternion(a + fraction*(b-a))
    angle = math.acos(dot)
    return (math.sin((1-fraction)*angle)*a + math.sin(fraction*angle)*b) / math.sin(angle)


def camera_transform(xml, odom_link, optical_link):
    tree = ET.fromstring(xml)
    edges = {}
    links = {el.attrib['name'] for el in tree.findall('link')}
    for joint in tree.findall('joint'):
        parent, child = joint.find('parent').attrib['link'], joint.find('child').attrib['link']
        if child in edges:
            raise ValueError('Multiple URDF parents')
        origin = joint.find('origin')
        xyz = np.fromstring(origin.attrib.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
        rpy = np.fromstring(origin.attrib.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
        if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite([*xyz, *rpy]).all():
            raise ValueError('Invalid URDF joint origin')
        limit = joint.find('limit')
        locked = joint.attrib.get('type') == 'fixed' or (limit is not None and
                 float(limit.attrib.get('lower', 'nan')) == 0 and float(limit.attrib.get('upper', 'nan')) == 0)
        mat = np.eye(4)
        mat[:3, :3], mat[:3, 3] = rotation_rpy(rpy), xyz
        edges[child] = (parent, mat, locked)

    def ancestry(link):
        if link not in links:
            raise ValueError('Running URDF lacks ' + link)
        chain, visited, mat = {}, set(), np.eye(4)
        while True:
            if link in visited:
                raise ValueError('Cyclic URDF')
            visited.add(link)
            chain[link] = mat.copy()
            if link not in edges:
                return link, chain
            parent, transform, locked = edges[link]
            if not locked:
                raise ValueError('Camera/odometry chain includes moving joint')
            mat = transform @ mat
            link = parent
    root_a, chain_a = ancestry(odom_link)
    root_b, chain_b = ancestry(optical_link)
    if root_a != root_b:
        raise ValueError('Camera/odometry have different roots')
    return np.linalg.inv(chain_a[root_a]) @ chain_b[root_b]


def interpolate(samples, stamp, max_gap=0.04):
    for index, sample in enumerate(samples):
        if abs(sample[0]-stamp) < 1e-8:
            return sample[1].copy(), sample[2].copy()
        if sample[0] > stamp:
            if index == 0:
                return None
            before = samples[index-1]
            gap = sample[0]-before[0]
            if gap <= 0 or gap > max_gap:
                return None
            f = (stamp-before[0])/gap
            return before[1]+f*(sample[1]-before[1]), slerp(before[2], sample[2], f)
    return None
