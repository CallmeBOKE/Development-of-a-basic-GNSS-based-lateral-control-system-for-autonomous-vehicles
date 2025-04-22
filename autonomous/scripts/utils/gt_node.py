#! /usr/bin/python3
import rospy
from novatel_oem7_msgs.msg import BESTPOS, INSPVAX
from pyproj import Proj
    
class GTNode:
    def __init__(self,file_path):
        self.utm_x = 0
        self.utm_y = 0
        self.heading = 0
        self.utm_proj = Proj(proj='utm', zone=52, ellps='WGS84')
        self.bestpos_sub = rospy.Subscriber('/novatel/oem7/bestpos', BESTPOS, self.bestpos_callback)
       	self.inspvax_sub = rospy.Subscriber('/novatel/oem7/inspvax', INSPVAX, self.inspvax_callback)
        self.file_path = file_path

    def bestpos_callback(self, data):
        lat, lon = data.lat, data.lon
        self.utm_x, self.utm_y = self.convert_to_utm(lat, lon)
    
    def inspvax_callback(self, data):
        self.heading = data.azimuth

    def convert_to_utm(self, lat, lon):
        return self.utm_proj(lon, lat)
    
    def file_write(self):
        with open(self.file_path, "a") as f :
            f.write(f"{self.utm_x},{self.utm_y},{self.heading}\n")
    
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.file_write()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('gt_node')
    file_path = '/home/nvidia/hs_workspace/src/autonomous/scripts/GT/GT_reverse2.csv'
    node = GTNode(file_path)
    node.spin()
