#!/usr/bin/env python3
"""Try candidate spawn poses and report which one actually gives the SLAM frontend
something to look at (ORB keypoint count on the real rendered image)."""
import sys, time
import numpy as np, rospy, cv2
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

CANDIDATES = [(0.0, 0.0, 0.0), (5.7, 6.0, 0.0), (5.7, 5.0, 1.57), (4.7, 4.0, 3.14),
              (3.0, 6.0, 0.0), (6.0, 2.0, 0.0), (1.0, 5.0, 1.57), (-2.0, 5.0, 0.0),
              (5.7, 6.0, 1.57), (5.7, 6.0, 3.14)]

def grab():
    m = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=20)
    img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create(1000)
    kp = orb.detect(g, None)
    return len(kp), float(g.mean()), float(g.std()), len(np.unique(g))

def main():
    rospy.init_node('pick_spawn', anonymous=True)
    rospy.wait_for_service('/gazebo/set_model_state', timeout=30)
    setstate = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    rows = []
    for (x, y, yaw) in CANDIDATES:
        ms = ModelState()
        ms.model_name = 'robot_1'
        ms.reference_frame = 'world'
        ms.pose.position.x, ms.pose.position.y, ms.pose.position.z = x, y, 0.02
        ms.pose.orientation.z = np.sin(yaw / 2.0)
        ms.pose.orientation.w = np.cos(yaw / 2.0)
        try:
            setstate(ms)
        except Exception as exc:
            print('set failed %s: %s' % ((x, y, yaw), exc)); continue
        time.sleep(2.5)
        try:
            kp, mean, std, uniq = grab()
        except Exception as exc:
            print('grab failed %s: %s' % ((x, y, yaw), exc)); continue
        rows.append((kp, x, y, yaw, mean, std, uniq))
        print('POSE (%5.1f,%5.1f,%.2f) kp=%4d mean=%6.1f std=%6.1f unique_values=%5d'
              % (x, y, yaw, kp, mean, std, uniq))
    rows.sort(reverse=True)
    print('BEST %s' % (rows[:3],))

if __name__ == '__main__':
    main()
