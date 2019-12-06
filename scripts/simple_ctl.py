import rospy
from roboy_middleware_msgs.msg import MotorCommand, JointStatus
from roboy_middleware_msgs.srv import ControlMode, ControlModeRequest

import numpy as np

rospy.init_node("muscctl")

p = rospy.Publisher('/roboy/middleware/MotorCommand', MotorCommand, queue_size=1)

rospy.loginfo("Changing control mode to DISPLACEMENT")
change_control_mode = rospy.ServiceProxy('/roboy/shoulder_left/middleware/ControlMode', ControlMode)
req = ControlModeRequest()
req.motor_id = [0,1,2,3]
req.control_mode = 1 # VELOCITY
change_control_mode(req)
rospy.loginfo("Changed to DISPLACEMENT control mode")

msg = MotorCommand()
msg.id = 3

msg.motors = [0,1]
msg.set_points = [-100000, 100000]
p.publish(msg)
msg.motors = []
msg.set_points = []

center = [2673.5, 2539.0]
motors_map = {0: [0,1], 1: [2,3]}

def encoderticks2degrees(ticks):
    return ticks*(360.0/4096.0)

def cb(data):
    angles = [] # in degrees [lower, upper]
    for i in [0]: #range(len(center)):
        encoderticks = data.absAngles[i] - center[i]
        angle = encoderticks2degrees(encoderticks)
        angles.append(angle)
        motors = motors_map[i]
        msg.motors.extend(motors)
        if angle <= -50:
            msg.set_points.extend([-100000, 100000])
        elif angle >= 50:
            msg.set_points.extend([-100000, 100000])

    p.publish(msg)
    #rospy.loginfo("Current joint angle values: " + str(angles))
    #rospy.loginfo("Published MotorCommand: " + str(msg))
    msg.set_points = []
    msg.motors = []   

s_hw = rospy.Subscriber('/roboy/middleware/JointStatus', JointStatus, cb2, buff_size=1) 
rospy.spin()
