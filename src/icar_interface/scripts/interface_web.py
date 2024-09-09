#!/usr/bin/python3

import rospy
import requests
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose2D

URL_POSLOCATION = "https://riset.its.ac.id/icar/api/poslocation"

class interface_web:
    def __init__(self):
        self.gps_origin_lat = rospy.get_param('gps/origin/lat', 0.0)
        self.gps_origin_lon = rospy.get_param('gps/origin/lon', 0.0)
        # =====Timer
        self.tim_05hz = rospy.Timer(rospy.Duration(2.00), self.cllbck_tim_05hz)
        # =====Subscriber
        self.sub_gps_navsatfix = rospy.Subscriber('/gps/navsatfix', NavSatFix, self.cllbck_sub_gps_navsatfix, queue_size=None)
        self.sub_gps_pose2d = rospy.Subscriber('/gps/pose2d', Pose2D, self.cllbck_sub_gps_pose2d, queue_size=None)

        self.gps_navsatfix = NavSatFix()
        self.gps_navsatfix.latitude = self.gps_origin_lat
        self.gps_navsatfix.longitude = self.gps_origin_lon
        self.gps_pose2d = Pose2D()

    # --------------------------------------------------------------------------
    # ==========================================================================

    def cllbck_tim_05hz(self, event):
        requests.post(URL_POSLOCATION, data={
            'id': '1',
            'latitude': self.gps_navsatfix.latitude,
            'longitude': self.gps_navsatfix.longitude
        })

    # --------------------------------------------------------------------------
    # ==========================================================================

    def cllbck_sub_gps_navsatfix(self, msg):
        self.gps_navsatfix = msg

    def cllbck_sub_gps_pose2d(self, msg):
        self.gps_pose2d = msg


if __name__ == '__main__':
    rospy.init_node('interface_web')
    interface_web()
    rospy.spin()
