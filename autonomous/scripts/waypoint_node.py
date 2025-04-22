#!/usr/bin/python3
import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Float32 , Int32
from scipy.spatial import KDTree


class TargetPointPublisher:
    def __init__(self, file_path):
        self.target_pub = rospy.Publisher('/target_point', Point, queue_size=1)
        self.index_pub = rospy.Publisher('/target_index',Int32,queue_size = 1)
        self.heading_pub = rospy.Publisher('/target_heading', Float32, queue_size=1)
        self.cte_pub = rospy.Publisher('/cte', Float32, queue_size=1)
        self.front_pub = rospy.Publisher('/position_front',Point,queue_size=1)
        rospy.Subscriber('/cur_position', Point, self.position_callback)
        rospy.Subscriber('/heading', Float32, self.heading_callback)
        self.current_position = None
        self.cur_heading = 0
        self.current_index = 0
        self.waypoints = self.load_waypoints(file_path)
        self.L = 2.7
        waypoint_coords = [(wp['utm_x'], wp['utm_y']) for wp in self.waypoints]
        self.kdtree = KDTree(waypoint_coords)

    def load_waypoints(self, file_path):
        waypoints = []
        with open(file_path, 'r') as f:
            for row in f:
                utm_x, utm_y, heading = row.strip().split(',')
                waypoints.append({'utm_x': float(utm_x), 'utm_y': float(utm_y), 'heading': float(heading)})
        return waypoints

    def position_callback(self, msg):
        self.current_position = (msg.x, msg.y)

    def heading_callback(self, msg):
        self.cur_heading = np.radians(msg.data)

    def publish_target_point(self):
        if self.current_position is None:
            return
        current_x, current_y = self.current_position
        front_x = current_x+self.L*np.cos((np.deg2rad(90)-self.cur_heading)%np.deg2rad(360))
        front_y = current_y+self.L*np.sin((np.deg2rad(90)-self.cur_heading)%np.deg2rad(360))
        _, closest_index = self.kdtree.query((front_x, front_y), k=1)
        self.current_index = closest_index
        target_point = self.waypoints[self.current_index]
        target_point_before = self.waypoints[self.current_index - 1]
        cte = ((target_point['utm_y'] - target_point_before['utm_y']) * front_x +
               (target_point_before['utm_x'] - target_point['utm_x']) * front_y +
               (target_point_before['utm_y'] * target_point['utm_x'] -
                target_point_before['utm_x'] * target_point['utm_y'])) / np.sqrt(
            (target_point['utm_y'] - target_point_before['utm_y'])**2 +
            (target_point_before['utm_x'] - target_point['utm_x'])**2)
        cte_msg = Float32(data=cte)
        target_msg = Point(x=target_point['utm_x'], y=target_point['utm_y'])
        index_msg = Int32(data = self.current_index)
        heading_msg = Float32(data=target_point['heading'])
        front_msg = Point(x=front_x,y= front_y,z=0)
        self.target_pub.publish(target_msg)
        self.index_pub.publish(index_msg)
        self.heading_pub.publish(heading_msg)
        self.cte_pub.publish(cte_msg)
        self.front_pub.publish(front_msg)
        rospy.loginfo(f"\nPublsihed target info\nTarget index:{self.current_index}\tCTE:{cte_msg.data}")
    
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_target_point()
            rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node('waypoint_node')
    file_path = '/home/nvidia/hs_workspace/src/autonomous/scripts/GT/GT_10hz.csv'
    target_point_publisher = TargetPointPublisher(file_path)
    target_point_publisher.spin()
