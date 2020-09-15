#!/usr/bin/env python
import rospy,math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import math
def set_throttle_steer(data):
    if data.linear.x>0:
	throttle = data.linear.x * 20 + 1560
    elif data.linear.x<0:
	throttle = data.linear.x * 20 + 1420
    else:
	throttle = data.linear.x +1500
    steer = math.degrees(data.angular.z) * 2.5 + 90

    pub = rospy.Publisher('/car/cmd_vel', Twist, queue_size=1)
    msg = Twist()
    msg.linear.x = throttle
    msg.angular.z = steer
    pub.publish(msg)

def servo_commands():

    rospy.init_node('servo_commands', anonymous=True)

    #rospy.Subscriber("/racecar/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)
    rospy.Subscriber("/cmd_vel", Twist, set_throttle_steer)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
