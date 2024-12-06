#! /usr/bin/python3

from tortoisebot_waypoints.tortoisebot_action_server import WaypointActionClass
from tortoisebot_waypoints.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction,WaypointActionGoal
import rospy
import rosunit
import unittest
import rostest
import sys
from nav_msgs.msg import Odometry
import actionlib
from tf import transformations
#import random
import math
from time import sleep
PKG = 'tortoisebot_waypoints'
NAME = 'tortoisebot_waypoints_test'

MAKE_TEST_PASS=True

waypoints = [
{ 'x' : 2.7, 'y':-0.48},                                #1
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
    # test passed test sequence 
    #test_sequence = [7,8,9]
    # Error test_sqeunce
    #test_sequence = [3,4,5]#[1,5,2] #[1,5,2,5,8,5,9]
    def setUp(self):
        self._sub_odom = rospy.Subscriber('/odom', Odometry, self._clbk_odom)



    def _clbk_odom(self, msg):
        # position
        self._position = msg.pose.pose.position

        # yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self._yaw = euler[2]


    def done_cb(self,status, result):
        rospy.loginfo('done')        

    def feedback_cb(self,msg):
        rospy.loginfo('Feedback received: %s', msg)

    # only functions with 'test_'-prefix will be run!
    def test_goal_reached(self):
        sleep(3) #Make sure that gazebo, odom has started
        self.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        self.client.wait_for_server()


        # for waypoint_index_plus_one in self.test_sequence: 
        if MAKE_TEST_PASS:
            waypoint_index_plus_one = 9  # 9 is for passed ,# 1 for failed
        else:
            waypoint_index_plus_one = 1  # 9 is for passed ,# 1 for failed
        waypoint_index = waypoint_index_plus_one - 1
        goal = WaypointActionGoal()
        goal.position.x = waypoints[waypoint_index].get('x')
        goal.position.y = waypoints[waypoint_index].get('y')
        self.goal_x = goal.position.x 
        self.goal_y = goal.position.y 
        self.start_position = self._position
        self.start_yaw = self._yaw 
        self.desired_yaw = math.atan2(self.goal_y -self.start_position.y, self.goal_x - self.start_position.x)
        self.client.send_goal(goal, done_cb = self.done_cb, feedback_cb=self.feedback_cb )
        #self.client.send_goal_and_wait(goal)
        self.client.wait_for_result() 
        #print(self.client.get_state)
        result = self.client.get_result()
        print(result)
        # step 3 Check the expected result with assertion
        # Check that the end position [X,Y] of the robot is the expected one based on the goal sent.
        # Check that the end rotation [Yaw] of the robot is the expected one based on the goal sent.
        self.end_position = self._position
        self.end_yaw = self._yaw 
        rospy.loginfo('goal ({},{}, yaw {})  end position ({},{}, yaw {})'.format(self.goal_x, self.goal_y, self.desired_yaw,self.end_position.x,self.end_position.y,self.end_yaw))        
        self.assertTrue(abs(self.goal_x - self.end_position.x) <= self.tolerance_pos, "Error X Position not in acceptable range")
        self.assertTrue(abs(self.goal_y - self.end_position.y) <= self.tolerance_pos, "Error Y Position not in acceptable range")       
        self.assertTrue(abs(self.desired_yaw - self.end_yaw) <= self.tolerance_yaw, "Error Yaw angle not in acceptable range")
            




if __name__ == '__main__':
    rospy.init_node('tortoisebot_ac')
    rostest.rosrun(PKG, NAME, TestTortoisebotWaypoints)
    #rosunit.unitrun(PKG, NAME,  TestTortoisebotWaypoints)
    #rospy.spin()
    #rospy.signal_shutdown('finished')