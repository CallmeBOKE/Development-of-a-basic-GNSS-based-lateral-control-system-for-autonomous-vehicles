#! /usr/bin/python3
import rospy
from novatel_oem7_msgs.msg import BESTPOS, INSPVAX, BESTVEL
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from pyproj import Proj
    
class GnssNode:
    def __init__(self):
        self.utm_x = 0
        self.utm_y = 0
        self.heading = 0
        self.velocity = 0
        self.utm_proj = Proj(proj='utm', zone=52, ellps='WGS84')
        self.bestpos_sub = rospy.Subscriber('/novatel/oem7/bestpos', BESTPOS, self.bestpos_callback)
        self.inspvax_sub = rospy.Subscriber('/novatel/oem7/inspvax', INSPVAX, self.inspvax_callback)
        self.bestvel_sub = rospy.Subscriber('/novatel/oem7/bestvel', BESTVEL, self.bestvel_callback)
        self.position_pub = rospy.Publisher('/cur_position',Point,queue_size=1)
        self.heading_pub = rospy.Publisher("/heading", Float32, queue_size=1)
        self.velocity_pub = rospy.Publisher("/velocity",Float32, queue_size=1)

    def bestpos_callback(self, data):
        lat, lon = data.lat, data.lon
        self.utm_x, self.utm_y = self.convert_to_utm(lat, lon)
    
    def inspvax_callback(self, data):
        self.heading = data.azimuth
    
    def bestvel_callback(self, data):
        self.velocity = data.hor_speed
    
    def convert_to_utm(self, lat, lon):
        return self.utm_proj(lon, lat)
    
    def publish_current_state(self):
        coordinate = Point(x=self.utm_x,y=self.utm_y,z=0)
        heading = Float32(data = self.heading)
        velocity = Float32(data = self.velocity)
        self.position_pub.publish(coordinate)
        self.heading_pub.publish(heading)
        self.velocity_pub.publish(velocity)
        rospy.loginfo(f"\nCurrent State\nx:{coordinate.x}\ty:{coordinate.y}\nHeading:{heading.data}\nVelocity:{velocity.data}")
    
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_current_state()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('gnss_node')
    node = GnssNode()
    node.spin()
