#! /usr/bin/python3
import rospy
from std_msgs.msg import Float32,Int32
    
class MetricNode:
    def __init__(self,file_path):
        self.cte = 0
        self.index = 1
        self.cte_sub = rospy.Subscriber('/cte', Float32, self.cte_callback)
        self.index_sub = rospy.Subscriber('/target_index',Int32,self.index_callback)
        self.file_path = file_path

    def cte_callback(self, data):
        self.cte = data.data
    
    def index_callback(self, data):
        self.index = data.data

    def file_write(self):
        with open(self.file_path, "a") as f :
            f.write(f"{self.index},{self.cte}\n")
    
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.file_write()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('metric_node')
    file_path = '/home/nvidia/hs_workspace/src/autonomous/scripts/metric/metric_reverse.csv'
    node = MetricNode(file_path)
    node.spin()
