#! /usr/bin/python3
import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from collections import deque

class HeadingNode:
    def __init__(self, file_path):
        self.cur_heading = 0
        self.gt_heading = 0
        self.error = self.gt_heading - self.cur_heading
        self.file_path = file_path
        self.gt_headings = deque(maxlen=100)  
        self.cur_headings = deque(maxlen=100) 
        self.gt_heading_sub = rospy.Subscriber('/gt_heading', Float32, self.gt_callback)
        self.current_heading_sub = rospy.Subscriber('/heading', Float32, self.heading_callback)

        self.fig, self.ax = plt.subplots()
        self.ax.set_ylim(-450,450)
        self.ax.set_title('Real-time Heading Comparison')
        self.ax.set_xlabel('Time Steps')
        self.ax.set_ylabel('Heading (degrees)')
        self.gt_line, = self.ax.plot([], [], label='GT Heading', color='blue')
        self.cur_line, = self.ax.plot([], [], label='Current Heading', color='red')
        self.ax.legend()
        plt.ion()
        plt.show()

        self.fig.canvas.mpl_connect('close_event', self.on_close_fig)

    def on_close_fig(self, event):
        plt.close(self.fig)

    def gt_callback(self, data):
        self.gt_heading = data.data

    def heading_callback(self, data):
        self.cur_heading = data.data

    def file_write(self):
        with open(self.file_path, "a") as f:
            self.error = self.gt_heading - self.cur_heading
            f.write(f"{self.cur_heading},{self.gt_heading},{self.error}\n")

    def update_plot(self):
        self.gt_headings.append(self.gt_heading)
        self.cur_headings.append(self.cur_heading)
        self.gt_line.set_xdata(range(len(self.gt_headings)))
        self.gt_line.set_ydata(list(self.gt_headings))
        self.cur_line.set_xdata(range(len(self.cur_headings)))
        self.cur_line.set_ydata(list(self.cur_headings))
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.05)

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if plt.fignum_exists(self.fig.number):
                self.update_plot()
            else :
                break
            self.file_write()
            rate.sleep()
            

if __name__ == '__main__':
    rospy.init_node('heading_error_node')
    file_path = '/home/nvidia/hs_workspace/src/autonomous/scripts/record/heading/heading_error_reverse.csv'
    node = HeadingNode(file_path)
    node.spin()
