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
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        # publisher for final waypoints
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # member variables you need below
        self.waypoints_base = None
        self.closet_wp_idx = 0

        rospy.spin()

    def pose_cb(self, msg):
        if self.waypoints_base is not None:
            rospy.loginfo('waypoint_updater : pose data (%.2f, %.2f, %.2f)', msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

            # calculate closet_wp_idx
            min_dist = 100000.0
            min_idx  = self.closet_wp_idx

            prev_closet_wp_idx = self.closet_wp_idx
            start_idx = self.closet_wp_idx - 2

            # find closest waypoint
            if (start_idx < 0):
                start_idx = start_idx + len(self.waypoints_base.waypoints)

            for i in range(start_idx, start_idx + len(self.waypoints_base.waypoints)):
                idx = i % len(self.waypoints_base.waypoints)
                cur_dist = self.eucl_dist_3d(msg.pose.position, self.waypoints_base.waypoints[idx].pose.pose.position)

                if (cur_dist < min_dist):
                    min_dist = cur_dist
                    min_idx  = idx

                if (min_dist < 5 and cur_dist > 10 * min_dist):
                    break

            dx = self.waypoints_base.waypoints[min_idx].pose.pose.position.x - msg.pose.position.x
            dy = self.waypoints_base.waypoints[min_idx].pose.pose.position.y - msg.pose.position.y

            heading = np.arctan2(dy, dx)
            (roll, pitch, yaw) = self.get_roll_pitch_yaw(msg.pose.orientation)
            angle = np.abs(yaw - heading)
            angle = np.minimum(angle, 2.0 * np.pi - angle)
            if (angle > np.pi / 4.0):
                self.closet_wp_idx = (min_idx + 1) % len(self.waypoints_base.waypoints)
            else:
                self.closet_wp_idx = min_idx

            # calculate self.waypoints_out only if the waypoint really has changed
            if prev_closet_wp_idx != self.closet_wp_idx:
              self.publish_waypoints()
              wp = self.waypoints_base.waypoints[self.closet_wp_idx]
              waypoint_pos = wp.pose.pose.position
              waypoint_speed = wp.twist.twist.linear.x
              rospy.loginfo('waypoint_updater pub: from index %i: (%.2f, %.2f, %.2f) with speed %.2f...'\
                            , self.closet_wp_idx, waypoint_pos.x, waypoint_pos.y, waypoint_pos.z, waypoint_speed)
        pass

    def publish_waypoints(self):
        if self.waypoints_base is not None:
            lane = Lane()
            lane.header = self.waypoints_base.header

            idx = self.closet_wp_idx
            wp = self.waypoints_base.waypoints

            lane.waypoints = wp[idx: min(idx + LOOKAHEAD_WPS, len(wp))]
            size = len(lane.waypoints)
            if size < LOOKAHEAD_WPS:
                lane.waypoints += wp[:LOOKAHEAD_WPS - size]

            self.final_waypoints_pub.publish(lane)

        pass

    def waypoints_cb(self, waypoints):
        self.waypoints_base = waypoints

        # consider waypoints violates the max-speed condition
        counter = 0
        for wp in self.waypoints_base.waypoints:
            if self.get_waypoint_velocity(wp) > self.c_max_velocity:
                wp.twist.twist.linear.x = self.c_max_velocity
                counter += 1

        rospy.loginfo('waypoint_updater with {0} wpnts'\
                      ' - total of {1} wpnts with max speed'\
                      .format(len(self.waypoints_base.waypoints), counter))
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
        if self.waypoints_base is not None:
            return (wp_idx+1) % len(self.waypoints_base.waypoints)
        return wp_idx

    def prev_waypoint(self, wp_idx):
        if self.waypoints_base is not None:
            return (wp_idx-1) % len(self.waypoints_base.waypoints)
        return wp_idx

    def eucl_dist_3d(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

    def distance(self, wp_idx_first, wp_idx_last):
        if self.waypoints_base is None:
            return 0
        dist = 0

        all_wp_length = len(self.waypoints_base.waypoints)
        if(wp_idx_first < wp_idx_last):
            wp_idx_distance = wp_idx_last-wp_idx_first
        else:
            wp_idx_distance = all_wp_length - wp_idx_first + wp_idx_last

        for i in range(wp_idx_first, (wp_idx_first+wp_idx_distance)):
            idx = i % all_wp_length
            next_idx = (idx + 1) % all_wp_length
            dist += self.dist_3d(self.waypoints_base.waypoints[idx].pose.pose.position, self.waypoints_base.waypoints[next_idx].pose.pose.position)
        return dist

    def get_roll_pitch_yaw(self, ros_quaternion):
        orientation_list = [ros_quaternion.x, ros_quaternion.y, ros_quaternion.z, ros_quaternion.w]
        # returns (roll, pitch, yaw)
        return euler_from_quaternion(orientation_list)


    def get_ros_quaternion(roll, pitch, yaw):
        return Quaternion(*quaternion_from_euler(roll, pitch, yaw))  # returns Quaternion
        

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
