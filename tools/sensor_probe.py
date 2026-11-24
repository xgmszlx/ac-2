#!/usr/bin/env python3
"""Count messages on the sensor topics of every namespace variant, so a probe can
never silently look at the wrong namespace."""
import sys, time
import rospy
from sensor_msgs.msg import Image, LaserScan

def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    try:
        rospy.init_node('sensor_probe', anonymous=True)
    except Exception as exc:
        print('PROBE init failed: %s' % exc)
        return
    topics = []
    for ns in ['', '/robot_1']:
        topics += [(ns + '/camera/rgb/image_raw', 'rgb'),
                   (ns + '/camera/depth_registered/image_raw', 'depth_registered'),
                   (ns + '/camera/depth/image_raw', 'depth'),
                   (ns + '/scan', 'scan'),
                   (ns + '/odom', 'odom')]
    counts = {k: 0 for _, k in topics}
    for t, k in topics:
        if k == 'scan':
            rospy.Subscriber(t, LaserScan, (lambda kk: (lambda m: counts.__setitem__(kk, counts[kk] + 1)))(k), queue_size=3)
        elif k == 'odom':
            from nav_msgs.msg import Odometry
            rospy.Subscriber(t, Odometry, (lambda kk: (lambda m: counts.__setitem__(kk, counts[kk] + 1)))(k), queue_size=3)
        else:
            rospy.Subscriber(t, Image, (lambda kk: (lambda m: counts.__setitem__(kk, counts[kk] + 1)))(k), queue_size=3)
    time.sleep(dur)
    print('PROBE %s' % counts)

if __name__ == '__main__':
    main()
