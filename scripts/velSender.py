#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

defaultVelocity = 1
Movement_Keys   = ["w",
                   "a",
                   "s",
                   "d",
                   "_w",
                   "_a",
                   "_s",
                   "_d",
                   "q", 
                   "e", 
                   "_q", 
                   "_e"]


class velSystem():
    def __init__(self):
        self.msg = Twist()
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.z = 0
        self.defaultVelocity = 1

        self.pub = rospy.Publisher('firefighter/cmd_vel', Twist, queue_size=10)

    def changeVelocity(self, key):
        if key == "w":
            self.msg.linear.x = defaultVelocity
        elif key == "a":
            self.msg.linear.y = defaultVelocity
        elif key == "s":
            self.msg.linear.x = -defaultVelocity
        elif key == "d":
            self.msg.linear.y = -defaultVelocity
        elif key == "q":
            self.msg.angular.z = defaultVelocity
        elif key == "e":
            self.msg.angular.z = -defaultVelocity
        if key == "_w":
            self.msg.linear.x = 0
        elif key == "_a":
            self.msg.linear.y = 0
        elif key == "_s":
            self.msg.linear.x = 0
        elif key == "_d":
            self.msg.linear.y = 0
        elif key == "_q":
            self.msg.angular.z = 0
        elif key == "_e":
            self.msg.angular.z = 0
        
    def getTwistMsg(self):
        return self.msg

    def publish(self):
        rospy.loginfo("\nSending Vel Info: \nLinear:\n" + str(self.msg.linear) + 
                                       "\nAngular:\n" + str(self.msg.angular))
        self.pub.publish(self.msg)


VelSystem = velSystem()



def callback(data):
    key = data.data
    if key in Movement_Keys:
        VelSystem.changeVelocity(key)
        VelSystem.publish()
        


def listener():
    rospy.init_node('velSender', anonymous=True)

    rospy.Subscriber("keyboard", String, callback)
    rospy.loginfo("Initialised Node")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass


    

  
