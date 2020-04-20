#!/usr/bin/env python
import math

import numpy as np

import rospy

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree



'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

'''

LOOKAHEAD_WPS = 30 # Number of waypoints we will publish. 
MAX_DECEL = 0.49 # Max velocity decrease per cycle
class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscriptions
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # TODO: Add a subscriber for  /obstacle_waypoint 
        
        # Publishers
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # Member Variables
        self.pose = None
        self.base_lane = None
        self.waypoint_tree = None
        self.waypoints_2d = None
        self.stopline_waypoint_ind = -1 
        self.loop()
        
    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints()
            rate.sleep()
            
    def get_closest_waypoint_ind(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_ind = self.waypoint_tree.query([x,y],1)[1]
        
        # Check if the closes waypoint is in front or behind
        closest_coord = self.waypoints_2d[closest_ind]
        prev_coord = self.waypoints_2d[closest_ind-1]
        
        # Hyperplane through closest coordinates
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])
        
        val = np.dot(cl_vect-prev_vect,pos_vect-cl_vect)
        
        if val > 0:
            closest_ind = (closest_ind + 1) % len(self.waypoints_2d)
            
        return closest_ind
        
    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)
                        
    def generate_lane(self):
        lane = Lane()
        closest_ind = self.get_closest_waypoint_ind()
        farthest_ind = closest_ind + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_ind:farthest_ind]
        if (self.stopline_waypoint_ind == -1 or self.stopline_waypoint_ind >= farthest_ind):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_ind)
        return lane
                         
    def decelerate_waypoints(self, waypoints, closest_ind):
        print("decelerating")
        decelerated_waypoints = []
        for ind, wp in enumerate(waypoints):
        
            new_wp = Waypoint()
            new_wp.pose = wp.pose             
            
            stop_ind = max(self.stopline_waypoint_ind - closest_ind - 2, 0) # Center of the car will stop 2 wp behind line
            dist = self.distance(waypoints, ind, stop_ind)
            vel = math.sqrt(2*MAX_DECEL*dist)
            if vel < 1:
                vel = 0 
            new_wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            decelerated_waypoints.append(new_wp)           
          
        return decelerated_waypoints   
                         
    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, lane):
        self.base_lane = lane
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in lane.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
       
    def traffic_cb(self, msg):
        self.stopline_waypoint_ind = msg.data
        
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
