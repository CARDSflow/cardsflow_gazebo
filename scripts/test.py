import rospy
from roboy_middleware_msgs.msg import MotorCommand
import time

rospy.init_node('musc_test')
pub = rospy.Publisher('/roboy/middleware/MotorCommand', MotorCommand, queue_size=1)

i = 0
msg = MotorCommand()
msg.motors = [0,1,2,3]

for j in range(5):
    while i<100:
        i += 1
        msg.set_points = [0,0,40-i/3, i/3]
        #msg.set_points = [i, 100-i, 40-i/3, i/3]
        rospy.loginfo(msg.set_points)
        pub.publish(msg)
        time.sleep(0.05)

    while i>0:
        i -= 1
        msg.set_points = [0,0,40-i/3,i/3]
        #msg.set_points = [i,100-i, 40-i/3, i/3]
        rospy.loginfo(msg.set_points)
        pub.publish(msg)
        time.sleep(0.05)
