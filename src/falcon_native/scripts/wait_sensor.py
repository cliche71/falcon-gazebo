#!/usr/bin/python3
import os, sys, time
import rospy
rospy.init_node('wait_native_calibration',anonymous=True,disable_signals=True)
while not rospy.is_shutdown() and not rospy.get_param('/native/calibrated',False): time.sleep(.1)
if not rospy.is_shutdown(): os.execv(sys.argv[1],sys.argv[1:])
