#! /usr/bin/python3
import rospy
import cantools
import can
from std_msgs.msg import Float32

db = cantools.database.load_file('/home/nvidia/hs_workspace/src/autonomous/scripts/DBC.dbc')
can_bus = can.interface.Bus('can0',bustype = 'socketcan')
file_path = '/home/nvidia/hs_workspace/src/autonomous/scripts/record/record_reverse.csv'
control = 0

def control_callback(msg):
    global control
    control = msg.data

def can_read(message):
    de_message = db.decode_message(message.arbitration_id,message.data)
    steer = float(de_message['Steering_angle_Feedback'])
    return steer

def record(message):
    global control
    with open(file_path, "a") as f :
            f.write(f"{can_read(message)},{control}\n")

def main():
    rate = rospy.Rate(10)
    rospy.Subscriber("/control_steer",Float32,control_callback,queue_size=1)
    while not rospy.is_shutdown():
        message = can_bus.recv()
        if message.arbitration_id == 0x051:
            try :
                record(message)
                rate.sleep()
            except:
                pass

if __name__ == '__main__':
    rospy.init_node('record_node')
    main()
