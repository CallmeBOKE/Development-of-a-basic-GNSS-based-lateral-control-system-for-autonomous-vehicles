#!/usr/bin/python3
import rospy
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from collections import deque

class RealTimePlot:
    def __init__(self, file_path):
        self.front_x = 0
        self.front_y = 0
        self.target_x = 0
        self.target_y = 0
        self.steer = 0
        self.yaw_term = 0
        self.cte_term = 0
        self.waypoints = self.load_waypoints(file_path)
        self.position_history = []
        self.steering_angle_history = []
        self.cte_term_history = []
        self.yaw_term_history = []
        self.steering_angles = deque(maxlen=100)
        self.yaw_terms = deque(maxlen=100)
        self.cte_terms = deque(maxlen=100)

        rospy.Subscriber('/cur_position', Point, self.position_callback)
        rospy.Subscriber('/target_point', Point, self.target_callback)
        rospy.Subscriber('/stanley_steer', Float32, self.steer_callback)
        rospy.Subscriber('/cte_term', Float32, self.cte_term_callback)
        rospy.Subscriber('/yaw_term', Float32, self.yaw_term_callback)  
        
        self.fig, self.ax = plt.subplots()
        self.ax.plot(
            [wp['utm_x'] for wp in self.waypoints],
            [wp['utm_y'] for wp in self.waypoints],
            label='Waypoints', linestyle='-', marker='o', markersize=5, linewidth=1, color='grey'
        )
        self.current_position_marker, = self.ax.plot([], [], 'bo', label='Current Position')
        self.target_point_marker, = self.ax.plot([], [], 'ro', label='Target Point')
        self.ax.set_xlabel('UTM X')
        self.ax.set_ylabel('UTM Y')
        self.ax.set_title('Real-Time Waypoint Tracking')
        self.ax.legend()

        self.fig2, self.ax2 = plt.subplots()
        self.ax2.set_ylim(-180, 180)
        self.ax2.set_xlabel('Time (steps)')
        self.ax2.set_ylabel('Steering Angle (degrees)')
        self.ax2.set_title('Real-Time Steering Angle')
        self.steer_line, = self.ax2.plot([], [], 'b-', label='Steering angle')
        self.yaw_line, = self.ax2.plot([], [], 'r-', label='Yaw term')
        self.cte_line, = self.ax2.plot([], [], 'g-', label='CTE term')
        self.ax2.legend()
        plt.ion()
        plt.show()

        self.fig.canvas.mpl_connect('close_event', self.on_close_fig)
        self.fig2.canvas.mpl_connect('close_event', self.on_close_fig2)

    def on_close_fig(self, event):
        plt.close(self.fig)

    def on_close_fig2(self, event):
        plt.close(self.fig2)

    def load_waypoints(self, file_path):
        waypoints = []
        with open(file_path, 'r') as f:
            for row in f:
                utm_x, utm_y, heading = row.strip().split(',')
                waypoints.append({'utm_x': float(utm_x), 'utm_y': float(utm_y), 'heading': float(heading)})
        return waypoints

    def position_callback(self, msg):
        self.front_x, self.front_y = msg.x, msg.y
        self.position_history.append((self.front_x, self.front_y))

    def target_callback(self, msg):
        self.target_x, self.target_y = msg.x, msg.y

    def steer_callback(self, msg):
        self.steer = msg.data
        self.steering_angles.append(self.steer)
        self.steering_angle_history.append(self.steer)

    def yaw_term_callback(self, msg):
        self.yaw_term = np.rad2deg(msg.data)
        self.yaw_terms.append(self.yaw_term)
        self.yaw_term_history.append(self.yaw_term)

    def cte_term_callback(self, msg):
        self.cte_term = np.rad2deg(msg.data)
        self.cte_terms.append(self.cte_term)
        self.cte_term_history.append(self.cte_term)

    def update_plot(self):
        self.current_position_marker.set_data(self.front_x, self.front_y)
        self.target_point_marker.set_data(self.target_x, self.target_y)
        self.ax.relim()
        self.ax.autoscale_view()
        self.steer_line.set_data(range(len(self.steering_angles)), list(self.steering_angles))
        self.yaw_line.set_data(range(len(self.yaw_terms)), list(self.yaw_terms))
        self.cte_line.set_data(range(len(self.cte_terms)), list(self.cte_terms))
        self.ax2.relim()
        self.ax2.autoscale_view()

        plt.draw()
        plt.pause(0.05)

    def save_graphs(self):
        plt.figure(figsize=(10, 8))
        plt.plot(
            [wp['utm_x'] for wp in self.waypoints],
            [wp['utm_y'] for wp in self.waypoints],
            label='Waypoints', linestyle='-', marker='o', markersize=4, linewidth=2, color='grey'
        )
        plt.plot(
            *zip(*self.position_history),
            label='Vehicle Trajectory', linestyle='-', marker='o', markersize=1, linewidth=1, color='blue'
        )
        plt.xlabel('UTM X')
        plt.ylabel('UTM Y')
        plt.title('Vehicle Trajectory')
        plt.legend()
        plt.savefig('/home/nvidia/hs_workspace/src/autonomous/scripts/fig/trajectory_with_waypoints_gunwoo.png', dpi=300)

        # Save Stanley Terms
        plt.figure(figsize=(10, 6))
        plt.plot(self.steering_angle_history, label='Steering Angle', color='blue', linewidth=2)
        plt.plot(self.yaw_term_history, label='Yaw Term', color='red', linewidth=1)
        plt.plot(self.cte_term_history, label='CTE Term', color='green', linewidth=1)
        plt.xlabel('Time (steps)')
        plt.ylabel('Angle (degree)')
        plt.title('Calculated steering angle')
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.savefig('/home/nvidia/hs_workspace/src/autonomous/scripts/fig/steering_angle_gunwoo.png', dpi=300)


    def file_write(self):
        with open('/home/nvidia/hs_workspace/src/autonomous/scripts/record/trajectory/trajectory_gunwoo.csv', 'a') as f:
            f.write(f"{self.front_x},{self.front_y},{self.cte_term},{self.yaw_term},{self.steer}\n")


    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if plt.fignum_exists(self.fig.number) or plt.fignum_exists(self.fig2.number):
                self.update_plot()
            else:
                break
            self.file_write()
            rate.sleep()
        self.save_graphs()


if __name__ == '__main__':
    rospy.init_node('plot_node')
    file_path = '/home/nvidia/hs_workspace/src/autonomous/scripts/GT/GT_10hz.csv'
    node = RealTimePlot(file_path)
    node.spin()

