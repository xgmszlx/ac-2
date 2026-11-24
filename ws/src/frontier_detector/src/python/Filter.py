#!/usr/bin/env python3

# jplaced
# 2022, Universidad de Zaragoza

# The filter nodes receives the detected frontier points from all the detectors,
# filters the points, and passes them to the assigner node to command the robots.
# Filtration includes the deletion of old and invalid points, and it also
# discards redundant points.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import os

import rospy
import tf
import dynamic_reconfigure.client

import numpy as np
from sklearn.cluster import MeanShift
from copy import copy, deepcopy
from scipy.spatial.transform import Rotation

from Functions import gridValue, createMarker, informationGain, informationGain_NUMBA, yawBtw2Points

from SimpleRobot import Robot

from frontier_detector.msg import PointArray
from visualization_msgs.msg import Marker

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PointStamped, Pose
from dynamic_reconfigure.server import Server

from frontier_detector.cfg import informationGainConfig

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Callbacks~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
map_data_ = OccupancyGrid()
map_arr_ = np.zeros((0,), dtype=np.int64)   # int64 copy of map_data_.data (see mapCallBack)
frontiers_ = []
f_timestamps_ = []
global_map_ = []
max_t_ = 0

INFORMATION_THRESHOLD_ = 0.5

# ---------------------------------------------------------------------------
# Read-only instrumentation (explorb_line_20261121).
#
# Set EXPLORB_FILTER_DEBUG=1 in the environment of the roslaunch that starts this
# node to get one log line per second with *why* each frontier centroid was thrown
# away, plus the raw numbers that drove the decision. It changes no decision and no
# published message -- it only logs. Used to locate the "filtered_points is empty"
# blocker without guessing at thresholds.
# ---------------------------------------------------------------------------
DEBUG = os.environ.get('EXPLORB_FILTER_DEBUG', '0').lower() in ('1', 'true', 'yes', 'on')
_STATS = {'rx': 0, 'alive': 0, 'n_cent': 0, 'zero': 0, 'near': 0, 'cost': 0, 'ig': 0, 'plan': 0, 'kept': 0}
_LAST_LOG_T = [0.0]
_IG_ERROR_REPORTED = [False]


def _vf(v):
    try:
        return '%.3f' % float(v)
    except Exception:
        return str(v)


def _fdbg(msg, *args):
    if DEBUG:
        rospy.logwarn('[fdbg] ' + (msg % args if args else msg))


def reconfigureCallback(config, level=0):
    global INFORMATION_THRESHOLD_
    rospy.logwarn_throttle(0.5, rospy.get_name() + """: Reconfigure Request! InfoGain threshold changed to: {ig_threshold}""".format(**config))
    INFORMATION_THRESHOLD_ = config["ig_threshold"]
    return config


def frontiersCallBack(data, args):
    global frontiers_, f_timestamps_

    transformed_point = args[0].transformPoint(args[1], data)
    x = np.array([transformed_point.point.x, transformed_point.point.y])
    x_t = data.header.stamp.to_sec()
    _STATS['rx'] += 1
    if DEBUG and _STATS['rx'] in (1, 10, 100) or (DEBUG and _STATS['rx'] % 500 == 0):
        _fdbg('raw frontier #%d  src_frame=%s -> %s  raw=(%.2f,%.2f)  x=(%.2f,%.2f)  stamp=%.3f now=%.3f',
              _STATS['rx'], data.header.frame_id, args[1], data.point.x, data.point.y,
              x[0], x[1], x_t, rospy.get_time())

    # Only add if not already there
    # Use temp variables to avoid errors due to global variables dimension (multiple ROS cb same time)
    temp_black = copy(x.tolist())
    temp_array = np.asarray(copy(frontiers_)).tolist()
    temp_time = copy(f_timestamps_)

    if len(temp_array) == len(temp_time):
        assert (len(temp_array) == len(temp_time))
        if temp_black in temp_array:
            repeated_idx = temp_array.index(temp_black)
            temp_time[repeated_idx] = x_t
        else:  # Otherwise, update timestamp
            temp_array.append(x)
            temp_time.append(x_t)

        # Delete too old points
        original_len = len(temp_array)
        assert original_len == len(temp_time)
        for ip in range(0, original_len):
            i = ip - original_len + len(temp_array)
            t_diff = np.abs(temp_time[i] - rospy.get_time())
            if t_diff > max_t_:
                rospy.logdebug(rospy.get_name() + ': Deleted a frontier with timestamp diff = ' + str(t_diff))
                del temp_array[i]
                del temp_time[i]

        frontiers_ = copy(temp_array)
        f_timestamps_ = copy(temp_time)
        assert (len(frontiers_) == len(f_timestamps_))
    else:
        rospy.logerr(rospy.get_name() + ': Frontier callback failed due to dimension mismatch of ' + str(
            len(temp_array) - len(temp_time)) + '. Skipping callback.')


def mapCallBack(data):
    global map_data_, map_arr_
    map_data_ = data
    # rospy deserialises int8[] as array.array('b'), so np.array(map_data_.data) is an
    # *int8* array while informationGain_NUMBA is compiled for nb.int64[:]. Calling it
    # with int8 raises "No matching definition for argument type(s) ... array(int8, 1d, C)"
    # on every centroid of every cycle -> the node dies and respawns, and filtered_points
    # is never published. Converted once per map message instead of once per centroid
    # (verified offline with tools/selfcheck_filter_ig.py).
    try:
        map_arr_ = np.asarray(data.data, dtype=np.int64)
    except Exception:
        map_arr_ = np.array(data.data)


def globalMapCallback(data):
    global global_map_
    global_map_ = data


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Node~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def node():
    global frontiers_, map_data_, global_map_, max_t_
    rospy.init_node('filter', anonymous=False)

    # Fetch all parameters
    map_topic = rospy.get_param('~map_topic', '/map')
    threshold = rospy.get_param('~costmap_clearing_threshold', 70)
    # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good,
    # info gain won't be accurate
    info_radius = rospy.get_param('~info_radius', 1.0)
    ig_th = rospy.get_param('~information_threshold', 1.0)
    goals_topic = rospy.get_param('~goals_topic', '/detected_points')
    n_robots = rospy.get_param('~n_robots', 1)
    namespace = rospy.get_param('~namespace', '/robot_')
    rate_hz = rospy.get_param('~rate', 1)
    robot_frame = rospy.get_param('~robot_frame', 'base_link')
    global_costmap_topic = rospy.get_param('~global_costmap_topic', '/move_base_node/global_costmap/costmap')
    use_gpu = rospy.get_param('~enable_gpu_comp', True)
    max_t_ = rospy.get_param('~maximum_time_f', 1.0)

    srv = Server(informationGainConfig, reconfigureCallback)

    if DEBUG:
        _fdbg('params: map_topic=%s global_costmap_topic=%s goals_topic=%s cost_thr=%s ig_thr=%s '
              'info_radius=%s rate=%s robot_frame=%s max_t=%s gpu=%s',
              map_topic, global_costmap_topic, goals_topic, threshold, ig_th,
              info_radius, rate_hz, robot_frame, max_t_, use_gpu)

    rate = rospy.Rate(rate_hz)
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)

    global_map_ = OccupancyGrid()

    rospy.Subscriber(global_costmap_topic, OccupancyGrid, globalMapCallback)

    # Robot
    robot_name = namespace + str(n_robots)
    robot = Robot(robot_name)

    # Wait if map map has not been received yet
    while len(map_data_.data) < 1:
        rospy.loginfo(rospy.get_name() + ': Filter is waiting for the map.')
        rospy.sleep(0.5)
        pass

    # Wait if global costmap map has not been received yet
    while len(global_map_.data) < 1:
        rospy.loginfo(rospy.get_name() + ': Filter is waiting for the global costmap.')
        rospy.sleep(0.5)
        pass

    rospy.loginfo(rospy.get_name() + ': Filter received local and global costmaps.')

    global_frame = "/" + map_data_.header.frame_id
    tf_lstnr = tf.TransformListener()
    # platform: this warm-up used to be fatal. tf2 raises TransformException when the
    # two frames are momentarily in different trees (map -> base_footprint is
    # (re)published by ORB-SLAM), and the node has respawn_delay=3s, so one hiccup cost
    # a whole restart *and* the costmap message that had already been received. A
    # missing transform here is not a reason to stop the fusion filter: the historical
    # transformPoint() calls below report their own state.
    try:
        tf_lstnr.waitForTransform(global_frame[1:], robot_frame, rospy.Time(0), rospy.Duration(3))
    except Exception as e:
        rospy.logwarn(rospy.get_name() + ': warm-up transform %s -> %s not available yet (%s); continuing.'
                      % (global_frame[1:], robot_frame, e))

    rospy.Subscriber(goals_topic, PointStamped, callback=frontiersCallBack, callback_args=[tf_lstnr, global_frame[1:]])

    # Publishers
    pub_frontiers = rospy.Publisher(rospy.get_name() + '/frontiers', Marker, queue_size=1)
    pub_centroids = rospy.Publisher(rospy.get_name() + '/centroids', Marker, queue_size=1)
    pub_filt_points = rospy.Publisher(rospy.get_name() + '/filtered_points', PointArray, queue_size=1)

    # Wait if no frontier is received yet
    counter = 0
    while len(frontiers_) < 1 and not rospy.is_shutdown():
        if counter == 0:
            rospy.loginfo(rospy.get_name() + ': Filter is waiting for frontiers.')
            counter = 1
        # platform: the author version spins here without sleeping, which pins a core
        # for as long as the detectors take to produce their first point.
        rospy.sleep(0.1)

    rospy.loginfo(rospy.get_name() + ': Filter received frontiers.')

    points = createMarker(frame=map_data_.header.frame_id, ns="raw_frontiers", colors=[1.0, 1.0, 0.0], scale=0.2,
                          lifetime=1 / rate_hz)
    points_clust = createMarker(frame=map_data_.header.frame_id, ns="filtered_frontiers", colors=[0.0, 1.0, 0.0],
                                lifetime=1 / rate_hz)

    p = Point()
    p.z = 0

    temp_point_stamped = PointStamped()
    temp_point_stamped.header.frame_id = map_data_.header.frame_id
    temp_point_stamped.header.stamp = rospy.Time(0)
    temp_point_stamped.point.z = 0.0

    temp_point_array = PointArray()
    temp_point = Point()
    temp_point.z = 0.0

    client = dynamic_reconfigure.client.Client("filter", timeout=1, config_callback=reconfigureCallback)
    client.update_configuration({"ig_threshold": ig_th})

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    while not rospy.is_shutdown():
        _STATS['alive'] += 1

        # Clustering frontier points
        centroids = []
        front = deepcopy(frontiers_)

        if len(front) > 1:
            ms = MeanShift(bandwidth=1.5)
            ms.fit(front)
            centroids = ms.cluster_centers_
        elif len(front) == 1:  # If there is only one frontier no need for clustering
            centroids = front

        # frontiers_ = deepcopy(centroids)

        # Clearing frontiers
        original_centroids_len = len(centroids)
        _cycle = {'n': int(original_centroids_len), 'zero': 0, 'near': 0, 'cost': 0, 'ig': 0, 'plan': 0}
        _detail = []
        for zp in range(0, original_centroids_len):
            z = zp - original_centroids_len + len(centroids)

            cond = False
            temp_point_stamped.point.x = centroids[z][0]
            temp_point_stamped.point.y = centroids[z][1]
            _why = ''
            _vals = {}

            # Remove frontiers at ~(0,0) // probably errors of the RRT frontier detectors
            if abs(centroids[z][0]) + abs(centroids[z][1]) < .1:
                cond = True
                _why = 'zero'

            # Remove frontiers too close to the robot
            if not cond:
                distance = np.linalg.norm(robot.getPosition() - centroids[z])
                _vals['d'] = distance
                if distance < 0.25:
                    cond = True
                    _why = 'near'
                    # rospy.logwarn(rospy.get_name() + ": Will delete a centroid too close to the robot.")

            # Remove any frontier inside an occupied cell (threshold)
            if not cond:
                transformed_point = tf_lstnr.transformPoint(global_map_.header.frame_id, temp_point_stamped)
                x = np.array([transformed_point.point.x, transformed_point.point.y])
                gval = gridValue(global_map_, x)
                _vals['gval'] = gval
                if gval >= threshold:
                    cond = True
                    _why = 'cost'
                    # rospy.logwarn(rospy.get_name() + ': Will delete a frontier in occupied cell with value: '
                    #               + str(gval))

            # Remove frontiers with low information gain
            if not cond:
                if use_gpu:
                    try:
                        ig = informationGain_NUMBA(map_data_.info.resolution, map_data_.info.width,
                                                   map_data_.info.origin.position.x,
                                                   map_data_.info.origin.position.y, map_arr_,
                                                   centroids[z][0], centroids[z][1], 1.0)
                    except Exception as e:
                        # read-only instrumentation: never let a numba/dispatch problem
                        # silently kill the fusion node (it has respawn=3s, so the visible
                        # symptom would be "filtered_points never published at all").
                        if not _IG_ERROR_REPORTED[0]:
                            _IG_ERROR_REPORTED[0] = True
                            rospy.logerr(rospy.get_name() + ': numba information gain failed (%s: %s); '
                                         'falling back to the python implementation.' % (type(e).__name__, e))
                        ig = informationGain(map_data_, [centroids[z][0], centroids[z][1]], 1.0)
                else:
                    ig = informationGain(map_data_, [centroids[z][0], centroids[z][1]], 1.0)
                _vals['ig'] = ig
                if ig < INFORMATION_THRESHOLD_:
                    cond = True
                    _why = 'ig'
                    # rospy.logwarn(rospy.get_name() + ': Will delete a frontier with information gain = ' + str(ig))

            # Remove frontiers in unreachable locations
            if not cond:
                p_frontier = np.array([centroids[z][0], centroids[z][1]])
                pose_frontier = Pose()
                pose_frontier.position.x = p_frontier[0]
                pose_frontier.position.y = p_frontier[1]
                pose_frontier.position.z = 0
                # Add orientation at frontier's location
                R_frontier = Rotation.from_euler('xyz', [0., 0., yawBtw2Points(robot.getPosition(), p_frontier)],
                                                 degrees=False)
                q_frontier = R_frontier.as_quat()
                pose_frontier.orientation.x = q_frontier[0]
                pose_frontier.orientation.y = q_frontier[1]
                pose_frontier.orientation.z = q_frontier[2]
                pose_frontier.orientation.w = q_frontier[3]
                # platform: a transient failure of the make_plan service must not kill the
                # fusion filter (observed: rospy ServiceException "Connection refused"
                # while move_base was still coming up -> node died, respawned, and the
                # 3 s respawn delay cost the costmap message it had already received).
                # A frontier whose reachability cannot be checked is kept, not deleted.
                try:
                    plan = robot.makePlan(robot.getPoseAsGeometryMsg(), pose_frontier)
                    n_points = int(len(plan))
                except Exception as e:
                    rospy.logwarn_throttle(2.0, rospy.get_name() + ': make_plan failed (%s: %s); '
                                           'keeping the frontier unchecked.' % (type(e).__name__, e))
                    n_points = -1
                _vals['plan'] = n_points
                if n_points == 0:
                    cond = True
                    _why = 'plan'
                    # rospy.logwarn(rospy.get_name() + ': Will delete an unreachable frontier.')

            if cond:
                if _why in _cycle:
                    _cycle[_why] += 1
                if DEBUG and len(_detail) < 6:
                    _detail.append('(%.2f,%.2f) why=%s %s' % (centroids[z][0], centroids[z][1], _why,
                                                              ' '.join('%s=%s' % (k, _vf(v)) for k, v in _vals.items())))
                centroids = np.delete(centroids, z, axis=0)
                # rospy.logwarn(rospy.get_name() + ': Frontier deleted.')
            elif DEBUG and len(_detail) < 6:
                _detail.append('(%.2f,%.2f) KEPT %s' % (centroids[z][0], centroids[z][1],
                                                        ' '.join('%s=%s' % (k, _vf(v)) for k, v in _vals.items())))

        _STATS['n_cent'] += _cycle['n']
        _STATS['kept'] += int(len(centroids))
        for _k in ('zero', 'near', 'cost', 'ig', 'plan'):
            _STATS[_k] += _cycle[_k]
        _now = rospy.get_time()
        if DEBUG and (_now - _LAST_LOG_T[0] > 1.0 or _now < _LAST_LOG_T[0]):
            _LAST_LOG_T[0] = _now
            _fdbg('t=%.2f alive=%d raw_rx=%d stored=%d centroids=%d kept=%d | del zero=%d near=%d cost=%d '
                  'ig=%d plan=%d (cum) | cost_thr=%s ig_thr=%.3f | map=%dx%d res=%.3f origin=(%.2f,%.2f) '
                  'frame=%s | costmap=%dx%d | %s',
                  _now, _STATS['alive'], _STATS['rx'], len(frontiers_), _cycle['n'], len(centroids),
                  _cycle['zero'], _cycle['near'], _cycle['cost'], _cycle['ig'], _cycle['plan'],
                  threshold, INFORMATION_THRESHOLD_,
                  map_data_.info.width, map_data_.info.height, map_data_.info.resolution,
                  map_data_.info.origin.position.x, map_data_.info.origin.position.y,
                  map_data_.header.frame_id,
                  global_map_.info.width, global_map_.info.height,
                  ' ;; '.join(_detail) if _detail else '-')

        rospy.logdebug(rospy.get_name() + ': Frontier centroids len=' + str(len(centroids)) + ', frontiers len=' + str(
            len(front)) + '. Centroids: \n' + str(centroids))

        # Publishing
        temp_point_array.points = []
        pp = []
        if len(centroids) > 0:
            for i in centroids:
                temp_point.x = i[0]
                temp_point.y = i[1]
                temp_point_array.points.append(copy(temp_point))
                pp.append(copy(temp_point))

        points_clust.id += 1
        points_clust.points = pp
        pub_centroids.publish(points_clust)
        pub_filt_points.publish(temp_point_array)

        pp = []
        if len(front) > 0:
            for q in front:
                p.x = q[0]
                p.y = q[1]
                pp.append(copy(p))

        points.id += 1
        points.points = pp
        pub_frontiers.publish(points)

        rate.sleep()


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Main~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
