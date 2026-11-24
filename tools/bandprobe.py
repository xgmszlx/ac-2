#!/usr/bin/env python3
import numpy as np, rospy
from sensor_msgs.msg import Image
rospy.init_node('band', anonymous=True)
rgb = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=30)
dep = rospy.wait_for_message('/camera/depth_registered/image_raw', Image, timeout=30)
r = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.width, 3)
d = np.frombuffer(dep.data, dtype=np.float32).reshape(dep.height, dep.width)
print('rgb unique values per band:')
for i in range(0, 480, 80):
    band = r[i:i+80]
    print('  rows %3d-%3d  unique=%4d  mean=%s' % (i, i+80, len(np.unique(band)), np.round(band.reshape(-1,3).mean(0),1)))
print('depth valid fraction per band:')
for i in range(0, 480, 80):
    band = d[i:i+80]
    ok = np.isfinite(band) & (band > 0)
    print('  rows %3d-%3d  valid=%.3f  med=%s' % (i, i+80, ok.mean(), ('%.2f' % np.median(band[ok])) if ok.any() else 'NA'))
