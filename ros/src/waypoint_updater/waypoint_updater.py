#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
DECELERATION = 2.0 # Absolute value of planned deceleration in m/s^2

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # max velocity, already converted from km/h to m/s
        self.c_max_velocity = rospy.get_param('waypoint_loader/velocity', 40.) / 3.6

        # subscribe to required topics
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # publisher for final waypoints
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # member variables you need below
        self.waypoints_ref = None
        self.cur_wp_ref_idx = 0
        self.traffic_wp_idx = -1
        self.waypoints_with_reduced_velocity = []

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        pass

    def waypoints_cb(self, waypoints):
        self.waypoints_ref = waypoints
        # consider waypoints violates the max-speed condition
        counter = 0
        for wp in self.waypoints_ref.waypoints:
          if self.get_waypoint_velocity(wp) > self.c_max_velocity:
            wp.twist.twist.linear.x = self.c_max_velocity
            counter += 1
        rospy.loginfo('waypoints_cb with {0} wpnts'\
                      ' - total of {1} wpnts with max speed'\
                      .format(len(self.waypoints_ref.waypoints), counter))
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, wp_idx, velocity):
        waypoints[wp_idx].twist.twist.linear.x = velocity
        pass
        
    def next_waypoint(self, wp_idx):
      if self.waypoints_ref is not None:
        return (wp_idx+1) % len(self.waypoints_ref.waypoints)
      return wp_idx

    def prev_waypoint(self, wp_idx):
      if self.waypoints_ref is not None:
        return (wp_idx-1) % len(self.waypoints_ref.waypoints)
      return wp_idx

    def dist_3d(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

    def distance(self, wp_idx_first, wp_idx_last):
        if None == self.waypoints_ref:
          return 0    
        dist = 0
        wp_idx_distance = 0
        #consider overflows in waypoint list as well
        all_wp_length = len(self.waypoints_ref.waypoints)
        if(wp_idx_first < wp_idx_last):
          wp_idx_distance = wp_idx_last-wp_idx_first
        else:
          wp_idx_distance = all_wp_length - wp_idx_first + wp_idx_last
        for i in range(wp_idx_first, (wp_idx_first+wp_idx_distance)):
            idx = i % all_wp_length
            next_idx = (idx + 1) % all_wp_length
            dist += self.dist_3d(self.waypoints_ref.waypoints[idx].pose.pose.position, self.waypoints_ref.waypoints[next_idx].pose.pose.position)
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
