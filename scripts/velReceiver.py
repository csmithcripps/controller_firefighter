#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from Motors import MotorLib


motors = MotorLib.Motors()

def callback(data):
    rospy.loginfo("Received Command: {0}\n{1}".format(data.linear, data.angular))
    x = data.linear.x
    w = data.angular.z
    if (w == 0):
        if (x>0):
            motors.forward(x*10)
        else:
            motors.backward(-x*10)
    else:
        if (w>0):
            motors.left(w*10)
        else:
            motors.right(-w*10)

def listener():
    rospy.init_node('firefighter_velReceiver', anonymous=True)

    rospy.Subscriber("firefighter/cmd_vel", Twist, callback)
    rospy.loginfo("Initialised Node")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        motors.exit()
        print("Finished Operation")

    

  
