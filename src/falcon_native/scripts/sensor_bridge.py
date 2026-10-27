#!/usr/bin/python3
"""Explicit legacy path or aligned same-camera input with official Transformer."""
import rospy
if __name__=='__main__':
    rospy.init_node('native_sensor');rospy.set_param('/native/calibrated',False)
    mode=rospy.get_param('~depth_mode','legacy')
    if mode=='legacy':
        from legacy_sensor_bridge import Bridge
        bridge=Bridge()
    elif mode=='official_compatible':
        from aligned_sensor_bridge import AlignedBridge
        bridge=AlignedBridge()
    else:raise ValueError('Unsupported depth mode: '+mode)
    rospy.spin()
