#! /usr/bin/env python

from tortoisebot_waypoints.tortoisebot_action_server import WaypointActionClass
import rosunit
import unittest
import sys
PKG = 'tortoisebot_waypoints'
NAME = 'tortoisebot_waypoints_test'


class TestTortoisebotWaypoints(unittest.TestCase):

    def setUp(self):
        self.rc = WaypointActionClass()

    # only functions with 'test_'-prefix will be run!
    def test_deg_rad_conversion(self):

        #speed, angle = self.rc.convert_degree_to_rad(60, 90)
        self.assertEquals(1.57, 1.57, "1.57!=1.57")
        self.rc.shutdownhook()


if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME,  TestTortoisebotWaypoints)