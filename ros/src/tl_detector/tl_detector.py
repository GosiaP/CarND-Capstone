#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
from tf.transformations import euler_from_quaternion
import numpy as np
import os
import re

STATE_COUNT_THRESHOLD = 3
IMAGE_DUMP_FOLDER = "./traffic_light_images/"

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.cur_wp_idx = 0
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''

        # initialize before callback is called
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        # load configuration
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        # Initialization of a bunch of camera-related parameters
        self.camera_z = 1.0
        self.image_width  = self.config['camera_info']['image_width']
        self.image_height = self.config['camera_info']['image_height']
        self.image_scale = 8000
        if ('image_scale' in self.config['camera_info']):
            self.image_scale = self.config['camera_info']['image_scale']
        self.camera_f_x = 2646
        if ('focal_length_x' in self.config['camera_info']):
            self.camera_f_x = self.config['camera_info']['focal_length_x']
        self.camera_f_y = 2647
        if ('focal_length_y' in self.config['camera_info']):
            self.camera_f_y = self.config['camera_info']['focal_length_y']
        self.camera_c_x = 366
        if ('focal_center_x' in self.config['camera_info']):
            self.camera_c_x = self.config['camera_info']['focal_center_x']
        else:
            self.camera_c_x = self.image_width / 2
        self.camera_c_y = 614
        if ('focal_center_y' in self.config['camera_info']):
            self.camera_c_y = self.config['camera_info']['focal_center_y']
        else:
            self.camera_c_y = self.image_height / 2

        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        rospy.loginfo('tldetector got %i waypoints', len(self.waypoints.waypoints))
        pass

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        #initialize the values
        ligth_wp = self.last_wp
        state = self.state
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint_from_pose(self):
        """Identifies the closest path waypoint to the current car position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        if self.pose and self.waypoints:
            # Calculate cur_wp_idx
            min_dist = 100000.
            min_idx = self.cur_wp_idx
            start_idx = self.cur_wp_idx - 2
            if (start_idx < 0):
                start_idx = start_idx + len(self.waypoints.waypoints)
            for i in range(start_idx, start_idx + len(self.waypoints.waypoints)):
                idx = i % len(self.waypoints.waypoints)
                cur_dist = self.dist_3d(self.pose.pose.position, self.waypoints.waypoints[idx].pose.pose.position)
                if (cur_dist < min_dist):
                    min_dist = cur_dist
                    min_idx = idx
                if (min_dist < 5 and cur_dist > 10 * min_dist):
                    break
            dx = self.waypoints.waypoints[min_idx].pose.pose.position.x - self.pose.pose.position.x
            dy = self.waypoints.waypoints[min_idx].pose.pose.position.y - self.pose.pose.position.y
            heading = np.arctan2(dy, dx)
            (roll, pitch, yaw) = self.get_roll_pitch_yaw(self.pose.pose.orientation)
            angle = np.abs(yaw - heading)
            angle = np.minimum(angle, 2.0 * np.pi - angle)
            if (angle > np.pi / 4.0):
                self.cur_wp_idx = (min_idx + 1) % len(self.waypoints.waypoints)
            else:
                self.cur_wp_idx = min_idx

        return self.cur_wp_idx

    def get_closest_waypoint(self, light_idx):
        """Identifies the closest path waypoint to the referenced light's stop line
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_position = self.config['stop_line_positions'][light_idx]

        if self.waypoints:
            min_dist = 100000.
            min_idx  = self.cur_wp_idx
            for i in range(self.cur_wp_idx, self.cur_wp_idx + len(self.waypoints.waypoints)):
                idx = i % len(self.waypoints.waypoints)
                dx = stop_line_position[0] - self.waypoints.waypoints[idx].pose.pose.position.x
                dy = stop_line_position[1] - self.waypoints.waypoints[idx].pose.pose.position.y
                cur_dist = math.sqrt(dx**2 + dy**2)
                if (cur_dist < min_dist):
                    min_dist = cur_dist
                    min_idx  = idx
                if (min_dist < 5 and cur_dist > 10 * min_dist):
                    break

        return min_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light_idx = None

        # Find the closest visible traffic light (if one exists)
        waypoint_idx = self.get_closest_waypoint_from_pose()

        if ( (self.waypoints != None) and (waypoint_idx != None) and
             (self.pose != None) and (self.camera_image != None) ):
            # look ahead through the waypoints along the next 120 meter
            travel_dist = 0
            found_light = False
            for i in range(waypoint_idx, waypoint_idx + len(self.waypoints.waypoints)):
                idx = i % len(self.waypoints.waypoints)
                last_idx = idx - 1
                if (last_idx < 0):
                    last_idx = last_idx + len(self.waypoints.waypoints)
                last_pos = self.waypoints.waypoints[last_idx].pose.pose.position
                this_pos = self.waypoints.waypoints[idx].pose.pose.position
                travel_dist = travel_dist + self.dist_3d(last_pos, this_pos)
                if (travel_dist > 120):
                    break
                # check if a traffic light is in range of 30 meter
                for tli in range(len(self.lights)):
                    wp_light_dist = self.dist_3d(self.lights[tli].pose.pose.position, self.waypoints.waypoints[idx].pose.pose.position)
                    if (wp_light_dist < 30):
                        found_light = True
                        # calculate vector from vehicle to traffic light in vehicle coordinate system
                        pose_light_dist = self.dist_3d(self.lights[tli].pose.pose.position, self.pose.pose.position)
                        light_position = self.lights[tli].pose.pose.position
                        dx_world = light_position.x - self.pose.pose.position.x
                        dy_world = light_position.y - self.pose.pose.position.y
                        dz_world = light_position.z - (self.pose.pose.position.z + self.camera_z)
                        (roll, pitch, yaw) = self.get_roll_pitch_yaw(self.pose.pose.orientation)
                        roll = 0.0  # value seems to be unreliable in bag file
                        pitch = 0.0 # value seems to be unreliable in bag file
                        s_y = math.sin(yaw)
                        c_y = math.cos(yaw)
                        s_p = math.sin(pitch)
                        c_p = math.cos(pitch)
                        s_r = math.sin(roll)
                        c_r = math.cos(roll)
                        rotation_matrix = \
                            [[c_y*c_p, c_y*s_p*s_r - s_y*c_r, c_y*s_p*c_r + s_y*s_r], \
                            [ s_y*c_p, s_y*s_p*s_r + c_y*c_r, s_y*s_p*c_r - c_y*s_r], \
                            [    -s_p,     c_p*s_r,               c_p*c_r]]
                        if np.linalg.matrix_rank(rotation_matrix) == 3:
                            inv_rotation_matrix = np.linalg.inv(rotation_matrix)
                            tmp = np.matmul(inv_rotation_matrix, [[dx_world], [dy_world], [dz_world]])
                            dx_camera = -tmp[1][0]
                            dy_camera = -tmp[2][0]
                            dz_camera =  tmp[0][0]

                            # check if traffic light is visible from vehicle
                            cropped_edge_len = int(round(self.image_scale / dz_camera))
                            cropped_x_center = int(round(self.camera_f_x * (dx_camera / dz_camera) + self.camera_c_x))
                            cropped_y_center = int(round(self.camera_f_y * (dy_camera / dz_camera) + self.camera_c_y))
                            cropped_x_from = cropped_x_center - (cropped_edge_len/2)
                            cropped_y_from = cropped_y_center - (cropped_edge_len/2)
                            cropped_x_to   = cropped_x_from   + cropped_edge_len
                            cropped_y_to   = cropped_y_from   + cropped_edge_len

                            if ( (cropped_x_to - cropped_x_from >= 32) and
                                 (cropped_x_from >= 0) and (cropped_x_to < self.image_width) and
                                 (cropped_y_from >= 0) and (cropped_y_to < self.image_height) ):

                                light_idx = tli

                                # works only in simulator
                                state = self.lights[light_idx].state

                            else:
                                light_pos = self.lights[tli].pose.pose.position

                        break
                if (found_light):
                    #early exit in case that we've detected a light
                    break

        if (light_idx != None):
            light_wp = self.get_closest_waypoint(light_idx)
            if None is state:
              state = self.lights[light_idx].state
            return light_wp, state
        return -1, TrafficLight.UNKNOWN

    def dist_3d(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

    def get_roll_pitch_yaw(self, ros_quaternion):
        orientation_list = [ros_quaternion.x, ros_quaternion.y, ros_quaternion.z, ros_quaternion.w]
        # returns (roll, pitch, yaw)
        return euler_from_quaternion(orientation_list)

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
