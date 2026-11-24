#!/usr/bin/env python3
import numpy as np, rospy
from sensor_msgs.msg import Image, CameraInfo
rospy.init_node('depth_probe', anonymous=True)
d = rospy.wait_for_message('/camera/depth_registered/image_raw', Image, timeout=30)
ci = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo, timeout=30)
print('rgb_encoding=%s %dx%d' % (d.encoding, d.width, d.height))
arr = np.frombuffer(d.data, dtype=np.float32 if d.encoding == '32FC1' else np.uint16)
arr = arr.astype(np.float64)
nz = arr[arr > 0]
print('depth: n=%d zero_frac=%.3f min=%s median=%s max=%s' % (
    arr.size, 1.0 - nz.size / float(arr.size),
    ('%.4f' % nz.min()) if nz.size else 'NA',
    ('%.4f' % np.median(nz)) if nz.size else 'NA',
    ('%.4f' % nz.max()) if nz.size else 'NA'))
print('K =', list(ci.K))
print('D =', list(ci.D))
rgb = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=30)
print('rgb encoding=%s %dx%d' % (rgb.encoding, rgb.width, rgb.height))
