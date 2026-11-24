#!/usr/bin/env python3
"""Exit 0 once the rendered camera image actually contains structure.

The bookstore world streams ~140 models; until they are in the scene the camera sees a
flat background and ORB-SLAM can never initialise. This waits for a frame whose grey
level histogram has enough distinct values and enough ORB keypoints.
"""
import sys
import numpy as np, rospy, cv2
from sensor_msgs.msg import Image

def main():
    min_uniq = int(sys.argv[1]) if len(sys.argv) > 1 else 50
    min_kp = int(sys.argv[2]) if len(sys.argv) > 2 else 200
    timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 90.0
    rospy.init_node('wait_visual', anonymous=True)
    t0 = rospy.get_time()
    orb = cv2.ORB_create(1000)
    while not rospy.is_shutdown():
        try:
            m = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=10)
        except Exception:
            if rospy.get_time() - t0 > timeout:
                print('WAIT_VISUAL timeout (no image)')
                return 1
            continue
        img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
        g = cv2.cvtColor(img[:, :, :3], cv2.COLOR_BGR2GRAY)
        uniq = len(np.unique(g))
        kp = len(orb.detect(g, None))
        el = rospy.get_time() - t0
        print('WAIT_VISUAL t=%.0f uniq=%d kp=%d' % (el, uniq, kp))
        if uniq >= min_uniq and kp >= min_kp:
            return 0
        if el > timeout:
            print('WAIT_VISUAL giving up')
            return 1
    return 1

if __name__ == '__main__':
    sys.exit(main())
