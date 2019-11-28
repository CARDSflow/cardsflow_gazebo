import rospy
from sensor_msgs.msg import JointState
from roboy_middleware_msgs.msg import MotorCommand
from math import tanh, sqrt, sin
import numpy as np

def softmax(x):
    return 1.0/(1+np.exp(-x))

rospy.init_node("musctest")

p = rospy.Publisher('/roboy/middleware/MotorCommand', MotorCommand, queue_size=1)
max_cmd = 240
msg = MotorCommand()
msg.motors = [0,1,2,3]

def cb(data):
    msg.set_points = [0,0,0,0]

    if data.position[1]>0:
        msg.set_points[1] = sin(data.position[1])*max_cmd
    else:
        msg.set_points[0] = sin(abs(data.position[1]))*max_cmd

    if data.position[0]>0:
        msg.set_points[2] = sin(data.position[0])*5
    else:
        msg.set_points[3] = sin(abs(data.position[0]))*5
    p.publish(msg)    

s = rospy.Subscriber('/joint_states', JointState, cb)
rospy.spin()
