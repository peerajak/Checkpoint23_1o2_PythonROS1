#! /usr/bin/python3

from tortoisebot_waypoints.tortoisebot_action_server import WaypointActionClass
from tortoisebot_waypoints.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction,WaypointActionGoal
import rospy
import rosunit
import unittest
import rostest
import sys
import actionlib
import random
import math
from time import sleep
PKG = 'tortoisebot_waypoints'
NAME = 'tortoisebot_waypoints_test'

waypoints = [
{ 'x' : 0.7, 'y':-0.48},                                #1
{ 'x' : 0.680851835461391, 'y':0.5046365559674899},     #2 
{ 'x' : 0.21495551839700774, 'y':0.5368169079826496},   #3
{ 'x' : 0.23105951276897543, 'y':0.04812895919547926},  #4
{ 'x' : -0.13007296166597881, 'y':0.010690406810986703},#5
{ 'x' : -0.18288059001897242, 'y':-0.43215748770886586},#6
{ 'x' : -0.15489504057272618, 'y':0.4629887743221526},  #7
{ 'x' : -0.5495752177522534, 'y':-0.5476146745173234},  #8
{ 'x' : -0.6091414439352052, 'y':0.49241421900139143}   #9
]


class TestTortoisebotWaypoints(unittest.TestCase):

    tolerance_pos = 0.1
    tolerance_yaw = 0.5

    def setUp(self):
        self.rc = WaypointActionClass()

    def done_cb(self,status, result):
        print('done')


    def feedback_cb(self,msg):
        print('Feedback received:', msg)

    # only functions with 'test_'-prefix will be run!
    def test_goal_reached(self):
        # step 1. get current position, and yaw
        self.start_position = self.rc._position
        self.start_yaw = self.rc._yaw
        # step 2. call action server with a list of test params
        self.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        self.client.wait_for_server()
        goal = WaypointActionGoal()
        waypoint_index = random.randint(0, 8)        
        goal.position.x = waypoints[waypoint_index].get('x')
        goal.position.y = waypoints[waypoint_index].get('y')
        while abs(goal.position.x - self.start_position.x) <0.1 and  abs(goal.position.y - self.start_position.y) <0.1:
            waypoint_index = random.randint(0, 8)        
            goal.position.x = waypoints[waypoint_index].get('x')
            goal.position.y = waypoints[waypoint_index].get('y')
            rospy.loginfo('setting goal ({},{})  current position ({},{})'.format(goal.position.x, goal.position.y, \
            self.start_position.x,self.start_position.y))
            sleep(0.5)
            
        
        self.goal_x = goal.position.x 
        self.goal_y = goal.position.y 
        self.client.send_goal(goal, done_cb = self.done_cb, feedback_cb=self.feedback_cb )
        try:
            self.client.wait_for_result()
        except Exception as e_msg:
            print(e_msg)
        try:
            result = self.client.get_result()
            print('result ',result)
        except Exception as e_msg2:
            print(e_msg2)
        # step 3 Check the expected result with assertion
        # Check that the end position [X,Y] of the robot is the expected one based on the goal sent.
        # Check that the end rotation [Yaw] of the robot is the expected one based on the goal sent.
        self.end_position = self.rc._position
        self.end_yaw = self.rc._yaw
        #desired_yaw = math.atan2( self.goal_y -self.start_position.y, self.goal_x - self.start_position.x)
        rospy.loginfo('goal ({},{}, yaw {})  end position ({},{}, yaw {})'.format(self.goal_x, self.goal_y, self.rc._des_yaw,self.end_position.x,self.end_position.y,self.end_yaw))
       
        self.assertTrue(abs(self.goal_x)-self.tolerance_pos <=abs(self.end_position.x) <= abs(self.goal_x)+self.tolerance_pos, "Error X Position not in acceptable range")
        self.assertTrue(abs(self.goal_y)-self.tolerance_pos <=abs(self.end_position.y) <= abs(self.goal_y)+self.tolerance_pos, "Error Y Position not in acceptable range")       
        self.assertTrue(abs(self.rc._des_yaw)-self.tolerance_yaw <=abs(self.end_yaw) <= abs(self.rc._des_yaw)+self.tolerance_yaw, "Error Yaw angle not in acceptable range")
        self.rc.shutdownhook()
        rospy.signal_shutdown('finished')



if __name__ == '__main__':
    rospy.init_node('tortoisebot_ac')
    rosunit.unitrun(PKG, NAME,  TestTortoisebotWaypoints)
    rospy.spin()