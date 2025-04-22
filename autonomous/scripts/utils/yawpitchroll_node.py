#! /usr/bin/python3
import rospy
from novatel_oem7_msgs.msg import INSPVAX
    
class GTNode:
    def __init__(self,file_path):
        self.heading = 0
        self.pitch = 0
        self.roll = 0
        self.inspvax_sub = rospy.Subscriber('/novatel/oem7/inspvax', INSPVAX, self.inspvax_callback)
        self.file_path = file_path

    def inspvax_callback(self, data):
        self.heading = data.azimuth
        self.pitch = data.pitch
        self.roll = data.roll
    
    def file_write(self):
        with open(self.file_path, "a") as f :
            f.write(f"{self.heading},{self.pitch},{self.roll}\n")
    
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.file_write()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('yawpitchroll_node')
    file_path = '/home/nvidia/hs_workspace/src/autonomous/scripts/record/yawpitchroll/yawpitchroll_reverse.csv'
    node = GTNode(file_path)
    node.spin()
