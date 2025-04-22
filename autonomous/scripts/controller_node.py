#! /usr/bin/python3
import cantools
import can
import numpy as np
import threading
import rospy
from std_msgs.msg import Float32


db = cantools.database.load_file("/home/nvidia/hs_workspace/src/autonomous/scripts/DBC.dbc")
can_bus = can.interface.Bus('can0', bustype='socketcan')

class ControllerNode:
    def __init__(self):
        self.override = 1
        self.heartbeat = 0
        self.accel = 700
        self.brake = 0.0
        self.steer = 0.0
        self.gear = 5
        self.steer = 0.0
        self.steer_sub = rospy.Subscriber('/stanley_steer',Float32,self.steer_callback)
        self.control_steer_pub = rospy.Publisher('/control_steer',Float32,queue_size=1)

    def steer_callback(self, data):
        self.steer = np.clip(6*data.data,-450,450)
    
    def Ctrl_CMD(self):
        ctrl_message = db.get_message_by_name('Control_CMD')
        ctrl_data = ctrl_message.encode({'Override': self.override, 'Alive_Count': self.heartbeat, 'Angular_Speed_CMD': 200})
        ctrl_message_send = can.Message(arbitration_id=ctrl_message.frame_id, data=ctrl_data, extended_id=False)
        can_bus.send(ctrl_message_send, timeout=0.001)

    def Driving_CMD(self):
        ctrl_message = db.get_message_by_name('Driving_CMD')
        ctrl_data = ctrl_message.encode({'Accel_CMD': self.accel, 'Brake_CMD': self.brake, 'Steering_CMD': self.steer, 'Gear_Shift_CMD': self.gear})
        ctrl_message_send = can.Message(arbitration_id=ctrl_message.frame_id, data=ctrl_data, extended_id=False)
        can_bus.send(ctrl_message_send, timeout=0.001)

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            print("steer :", self.steer, end='\r')
            control_steer_msg =Float32(data = self.steer)
            self.control_steer_pub.publish(control_steer_msg)
            if self.heartbeat < 255:
                self.Ctrl_CMD()
                self.heartbeat += 1
            else:
                self.Ctrl_CMD()
                self.heartbeat = 0
            self.Driving_CMD()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('controller_node')
    node = ControllerNode()
    node.spin()


