#!/usr/bin/env python3
import sys, numpy as np, rospy, cv2
from sensor_msgs.msg import Image
rospy.init_node('grab', anonymous=True)
m = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=30)
img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/frame.png'
cv2.imwrite(out, img)
g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
orb = cv2.ORB_create(1000)
kp = orb.detect(g, None)
print('saved %s mean=%.1f std=%.1f orb_keypoints=%d' % (out, g.mean(), g.std(), len(kp)))
