#! /usr/bin/python3
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Point

class StanleyNode:
    def __init__(self):
        self.current_pos_sub = rospy.Subscriber('/position_front', Point, self.current_pos_callback)
        self.current_vel_sub = rospy.Subscriber('/velocity', Float32, self.velocity_callback)
        self.current_yaw_sub = rospy.Subscriber('/heading', Float32, self.heading_callback)
        self.cte_sub = rospy.Subscriber('/cte', Float32, self.cte_callback)
        self.target_yaw_sub = rospy.Subscriber('/target_heading', Float32, self.target_yaw_callback)
        self.steer_pub = rospy.Publisher('/stanley_steer', Float32, queue_size=1)
        self.cte_pub = rospy.Publisher('/cte_term',Float32,queue_size=1)
        self.yaw_pub = rospy.Publisher('/yaw_term',Float32,queue_size=1)
        self.gt_heading_pub = rospy.Publisher('/gt_heading',Float32,queue_size=1)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_heading = 0
        self.cur_velocity = 0
        self.cte = 0
        self.target_yaw = 0
        self.k = 0.5
        self.L = 2.7
    
    def current_pos_callback(self, data):
        self.cur_x, self.cur_y = data.x, data.y
    
    def velocity_callback(self, data):
        self.cur_velocity = data.data
    
    def heading_callback(self, data):
        self.cur_heading = np.radians(data.data)
    
    def cte_callback(self, data):
        self.cte = data.data
    
    def target_yaw_callback(self, data):
        self.target_yaw = np.radians(data.data)
    
    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
    
    def stanley_control(self):
        yaw_term = self.normalize_angle(self.target_yaw - self.cur_heading)
        if self.cur_velocity <0.1 :
            cte_term = 0
        else :
            cte_term = self.normalize_angle(np.arctan2(self.k * self.cte, self.cur_velocity))*(-1)
        stanley_steer = np.rad2deg(yaw_term+cte_term)
        steering_msg = Float32(data = stanley_steer)
        yaw_term_msg  = Float32(data = yaw_term)
        cte_term_msg = Float32(data = cte_term)
        gt_heading_msg = Float32(data = stanley_steer+np.rad2deg(self.cur_heading))
        self.steer_pub.publish(steering_msg)
        self.cte_pub.publish(cte_term_msg)
        self.yaw_pub.publish(yaw_term_msg)
        self.gt_heading_pub.publish(gt_heading_msg)
        rospy.loginfo(f"\nSteering angle\nTotal:{steering_msg.data}\nYaw term:{np.rad2deg(yaw_term_msg.data)}\nCTE term:{np.rad2deg(cte_term_msg.data)}")
    
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.stanley_control()
            rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node('stanley_node')
    node = StanleyNode()
    node.spin()
