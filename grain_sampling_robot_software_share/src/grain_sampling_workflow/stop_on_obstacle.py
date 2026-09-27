#!/usr/bin/env python3
from __future__ import annotations
import logging, threading
import numpy as np
try:
    import rospy
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Bool
    import sensor_msgs.point_cloud2 as pc2
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
logger = logging.getLogger('stop_on_obstacle')
class StopOnObstacle:
    def __init__(self):
        self._cloud = None; self._lock = threading.Lock()
        rospy.init_node('stop_on_obstacle', disable_signals=True)
        rospy.Subscriber('/cloud_registered_body', PointCloud2, self._cb, queue_size=1)
        self._obs_pub = rospy.Publisher('/obstacle_detected', Bool, queue_size=5, latch=True)
        self._true_count = 0; self._false_count = 0
        self._debounce = 3  # require 3 consecutive frames
        self._last_published = False
    def _cb(self, msg):
        with self._lock: self._cloud = msg
    def _pts(self):
        with self._lock: c = self._cloud
        if c is None: return np.empty((0,3))
        try: return np.array([(p[0],p[1],p[2]) for p in pc2.read_points(c, field_names=('x','y','z'), skip_nans=True)])
        except: return np.empty((0,3))
    def _blocked(self, pts):
        if not pts.size: return False
        return bool(np.any((pts[:,0]>=0.3)&(pts[:,0]<=1.0)&(pts[:,1]>=-0.3)&(pts[:,1]<=0.3)&(pts[:,2]>=-0.5)&(pts[:,2]<=0.8)))
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            b = self._blocked(self._pts())
            # Debounce: require N consecutive same-value frames
            if b:
                self._true_count += 1; self._false_count = 0
            else:
                self._false_count += 1; self._true_count = 0
            stable = self._true_count >= self._debounce if self._last_published else self._false_count < self._debounce
            if self._true_count >= self._debounce and not self._last_published:
                self._last_published = True
                self._obs_pub.publish(Bool(data=True))
            elif self._false_count >= self._debounce and self._last_published:
                self._last_published = False
                self._obs_pub.publish(Bool(data=False))
            # The goal controller is the sole motor-command owner.  This node
            # only publishes the obstacle state: on True the controller sends
            # its normal hard stop; on False it resumes its saved state.  Never
            # inject a non-zero UDP command when an obstacle clears.
            rate.sleep()
    def shutdown(self):
        pass
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
    n = StopOnObstacle()
    try: n.spin()
    except KeyboardInterrupt: pass
    finally: n.shutdown()
