#! /usr/bin/python3
import rospy
import numpy as np
from collections import deque
from std_msgs.msg import Float32
import cantools
import can

db = cantools.database.load_file("/home/nvidia/hs_workspace/src/autonomous/scripts/DBC.dbc")
can_bus = can.interface.Bus('can0', bustype='socketcan',bitrate=500000)

class ControllerNode:
    def __init__(self):
        self.override = 1
        self.heartbeat = 0
        self.accel = 700
        self.brake = 0
        self.gear = 5
        self.steer = 0

        self.stanley_steer = 0
        self.gt_heading = 0
        self.cur_heading = 0

        self.kp = 6.5
        self.ki = 0.2
        self.kd = 0.7
        self.dt = 0.1
        self.errors = deque(maxlen=10)

        self.stanley_steer_sub = rospy.Subscriber('/stanley_steer', Float32, self.stanley_callback)
        self.gt_heading_sub = rospy.Subscriber('/gt_heading', Float32, self.gt_callback)
        self.current_heading_sub = rospy.Subscriber('/heading', Float32, self.heading_callback)
        self.control_steer_pub = rospy.Publisher('/control_steer', Float32, queue_size=1)


    def stanley_callback(self, data):
        self.stanley_steer = data.data

    def gt_callback(self, data):
        self.gt_heading = data.data

    def heading_callback(self, data):
        self.cur_heading = data.data

    def pid_controller(self):
        error = self.gt_heading - self.cur_heading
        rospy.loginfo(f"Error: {self.gt_heading} - {self.cur_heading} = {error}")
        self.errors.append(error)

        if len(self.errors) >= 2:
            de = (self.errors[-1] - self.errors[-2]) / self.dt
            ie = sum(self.errors) * self.dt
        else:
            de, ie = 0, 0

        pid_steer = (self.kp * error) + (self.kd * de) + (self.ki * ie)
        return pid_steer

    def Ctrl_CMD(self):
        ctrl_message = db.get_message_by_name('Control_CMD')
        ctrl_data = ctrl_message.encode({
            'Override': self.override,
            'Alive_Count': self.heartbeat,
            'Angular_Speed_CMD': 100
        })
        ctrl_message_send = can.Message(
            arbitration_id=ctrl_message.frame_id,
            data=ctrl_data,
            extended_id=False
        )
        can_bus.send(ctrl_message_send, timeout=0.001)

    def Driving_CMD(self, steer_value):
        ctrl_message = db.get_message_by_name('Driving_CMD')
        ctrl_data = ctrl_message.encode({
            'Accel_CMD': self.accel,
            'Brake_CMD': self.brake,
            'Steering_CMD': steer_value,
            'Gear_Shift_CMD': self.gear
        })
        ctrl_message_send = can.Message(
            arbitration_id=ctrl_message.frame_id,
            data=ctrl_data,
            extended_id=False
        )
        can_bus.send(ctrl_message_send, timeout=0.001)

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.heartbeat = (self.heartbeat+1)%256
            self.Ctrl_CMD()
            # self.steer = self.stanley_steer
            pid_correction = self.pid_controller()
            self.steer = self.stanley_steer+pid_correction
            self.steer = np.clip(self.steer,-450,450)
            self.Driving_CMD(self.steer)
            self.control_steer_pub.publish(Float32(data=self.steer))
            rospy.loginfo(f"Steering angle : {self.steer}")
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('controller_node')
    node = ControllerNode()
    node.spin()


