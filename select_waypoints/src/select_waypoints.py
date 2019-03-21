#!/usr/bin/env python
import sys
import rospy
import csv
import math
import time
import threading
import actionlib

from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from std_msgs.msg import String, Empty

class WaypointSelector(object):
    def __init__(self):
        self.frame_id = rospy.get_param('~goal_frame_id','map')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')

        # get csv_files
        f_name = rospy.get_param('/select_waypoints/waypoints_file')
        f = open(f_name)
        self.csv_f = list(csv.reader(f))

        # set up ros stuff
        self.goal_pub = rospy.Publisher('goalpose', PoseWithCovarianceStamped, queue_size=10)
        self.path_ready_pub = rospy.Publisher('path_ready', Empty, queue_size=10)
        self.sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.generateWaypoints)
        rate = rospy.Rate(10) # 10hz
    
    def generateWaypoints(self, pwc):
        min_dist = sys.maxsize
        pose = pwc.pose
        position = pose.pose.position
        orientation = pose.pose.orientation

        idx = 0
        for i, row in enumerate(self.csv_f):
            # x, y, z, qx, qy, qz, qw, is_searching_area, reach_threshold
            row = [float(r) for r in row]
            dist = math.sqrt((position.x - row[0]) ** 2 + (position.y - row[1]) ** 2)
            relative_orient = math.sqrt((orientation.x - row[3]) ** 2 + (orientation.y - row[4]) ** 2 + 
                (orientation.z - row[5]) ** 2 + (orientation.w - row[6]) ** 2)
            if dist < min_dist and relative_orient < 0.5:
                min_dist = dist
                idx = i

        # publish
        waypoints = []        
        for i in range(len(self.csv_f)):
            print('Index of next path', (i + idx) % len(self.csv_f))
            cur = self.csv_f[(i + idx) % len(self.csv_f)]
            new_pose = self.getPoseWithCovariance(cur)
            waypoints.append(new_pose)

        self.execute(waypoints)
    
    def execute(self, waypoints):
        # Execute waypoints each in sequence
        for waypoint in waypoints:
            # Break if preempted
            if waypoints == []:
                rospy.loginfo('The waypoint queue has been reset.')
                break
            # Otherwise publish next waypoint as goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = waypoint.pose.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                    (waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
            rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            self.client.send_goal(goal)
            self.client.wait_for_result()
            rospy.loginfo('Goal arrived')
        return 'success'

    def getPoseWithCovariance(self, row):
        row = [float(r) for r in row]
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = row[0]
        pose.pose.pose.position.y = row[1]
        pose.pose.pose.position.z = row[2]

        pose.pose.pose.orientation.x = row[3]
        pose.pose.pose.orientation.y = row[4]
        pose.pose.pose.orientation.z = row[5]
        pose.pose.pose.orientation.w = row[6]

        return pose

if __name__ == '__main__':
    try:
        rospy.init_node('select_waypoints')
        WaypointSelector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass