#!/usr/bin/python3
"""Select camera input semantics; legacy without observation returns exact baseline."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

def model(mode='legacy', observe_raw=False, include_self_visuals=False):
    if mode not in ('legacy', 'official_compatible'):
        raise ValueError('Unsupported depth mode: ' + mode)
    original = (Path(__file__).resolve().parents[1] / 'reference/robot.urdf').read_bytes()
    if mode == 'legacy' and not observe_raw:
        return original
    root = ET.fromstring(original)
    sensors = root.findall(".//sensor[@type='depth']")
    if len(sensors) != 1:
        raise ValueError('Expected exactly the original single depth camera')
    sensor = sensors[0]
    if mode == 'official_compatible':
        sensor.find('camera/clip/near').text = '0.1'
        sensor.find('camera/clip/far').text = '500.0'
        sensor.find('camera/horizontal_fov').text = '1.5707963267948966'
        lens = ET.SubElement(sensor.find('camera'), 'lens')
        intrinsics = ET.SubElement(lens, 'intrinsics')
        for name, value in dict(fx=320, fy=320, cx=320, cy=240, s=0).items():
            ET.SubElement(intrinsics, name).text = str(value)
        openni = sensor.find("plugin[@filename='libgazebo_ros_openni_kinect.so']")
        openni.find('pointCloudCutoff').text = '0.1'
        openni.find('pointCloudCutoffMax').text = '500.0'
        for name, value in dict(focalLength=320, Cx=320, CxPrime=320, Cy=240, hackBaseline=0).items():
            ET.SubElement(openni, name).text = str(value)
        if not include_self_visuals:
            guard = ET.SubElement(sensor, 'plugin', filename='libnative_depth_self_filter.so', name='native_depth_self_filter')
            ET.SubElement(guard, 'modelName').text = 'hummingbird'
    observer = ET.SubElement(sensor, 'plugin', filename='libnative_raw_depth_observer.so', name='native_raw_depth_observer')
    ET.SubElement(observer, 'frameName').text = sensor.find("plugin[@filename='libgazebo_ros_openni_kinect.so']/frameName").text
    return ET.tostring(root, encoding='utf-8', xml_declaration=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['legacy', 'official_compatible'], default='legacy')
    parser.add_argument('--observe-raw', choices=['true', 'false'], default='false')
    parser.add_argument('--include-self-visuals', choices=['true', 'false'], default='false')
    args = parser.parse_args()
    import sys
    sys.stdout.buffer.write(model(args.mode, args.observe_raw == 'true', args.include_self_visuals == 'true'))
