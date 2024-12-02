#! /usr/bin/python3
import rospy
import time
import actionlib
 
from tortoisebot_waypoints.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction,WaypointActionGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import random
import math
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


def done_cb(status, result):
 print('done')

def feedback_cb(msg):
 print('Feedback received:', msg)

def call_server():
    client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
    client.wait_for_server()
    goal = WaypointActionGoal()
    waypoint_index = random.randint(0, 8)
    goal.position.x = waypoints[waypoint_index].get('x')
    goal.position.y = waypoints[waypoint_index].get('y')

    client.send_goal(goal, done_cb = done_cb, feedback_cb=feedback_cb )

    client.wait_for_result()

    result = client.get_result()

    return result

if __name__ == '__main__':

    try:
        rospy.init_node('tortoisebot_ac')
        result = call_server()
        print( 'The result is:', result)
    except rospy.ROSInterruptException as e:
        print( 'Something went wrong:', e)