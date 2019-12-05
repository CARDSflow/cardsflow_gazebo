import rospy
from sensor_msgs.msg import JointState
from roboy_middleware_msgs.msg import MotorCommand, JointStatus
from roboy_middleware_msgs.srv import ControlMode, ControlModeRequest

from math import tanh, sqrt, sin
import numpy as np

def softmax(x):
    return 1.0/(1+np.exp(-x))

rospy.init_node("musctest")

p = rospy.Publisher('/roboy/middleware/MotorCommand', MotorCommand, queue_size=1)
rospy.loginfo("Changing control mode to DISPLACEMENT")
change_control_mode = rospy.ServiceProxy('/roboy/shoulder_left/middleware/ControlMode', ControlMode)
req = ControlModeRequest()
req.motor_id = [0,1,2,3]
req.control_mode = 2 # DISPLACEMENT
change_control_mode(req)
rospy.info("Changed to DISPLACEMENT control mode")
max_cmd = [100,20]

msg = MotorCommand()
msg.id = 3

def encoderticks2degrees(ticks):
    return ticks*(360.0/4096.0)

center = [2673.5, 2539.0]
motors_map = {0: [0,1], 1: [2,3]}

def cb2(data):
    angles = [] # in degrees [lower, upper]
    for i in [0,1]: #range(len(center)):
        encoderticks = data.absAngles[i] - center[i]
        angle = encoderticks2degrees(encoderticks)
        angles.append(angle)
        motors = motors_map[i]
        msg.motors.extend(motors)
        if angle<0:
            msg.set_points.extend([abs(sin(angle)*max_cmd[i]),0])
        else:
            msg.set_points.extend([0, abs(sin(angle)*max_cmd[i])])

    p.publish(msg)
    rospy.loginfo("Current joint angle values: " + str(angles))
    rospy.loginfo("Published MotorCommand: " + str(msg))
    msg.set_points = []
    msg.motors = []


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

s_hw = rospy.Subscriber('/roboy/middleware/JointStatus', JointStatus, cb2) 
s = rospy.Subscriber('/joint_states', JointState, cb)
rospy.spin()
