#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import sys
import RPi.GPIO as IO
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008 as MCP3008

pub = rospy.Publisher('firefighter/cmd_vel', Twist, queue_size=10)

class button:
    def __init__(self, pin):
        self.pin = pin
        IO.setmode(IO.BOARD)
        IO.setup(pin, IO.IN)
        
    def read(self):
        return IO.input(self.pin)

class controller:
    def __init__(self):
        self.B = button(31)
        
        self.mcp = MCP3008.MCP3008(spi=SPI.SpiDev(0,0))
        
        self.msg = Twist()
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.z = 0
        
    def update(self):
        if(self.B.read()):
            self.msg.linear.z = 1
        else:
            self.msg.linear.z = 0

        self.msg.linear.x  = 75*(self.mcp.read_adc(0)-516)/516
        self.msg.linear.y  = 75*(self.mcp.read_adc(1)-516)/516
        self.msg.angular.x += 2*(self.mcp.read_adc(2)-503)/503
        self.msg.angular.x = min(max(self.msg.angular.x, 0), 45)
        self.msg.angular.y = self.mcp.read_adc(2)
        self.msg.angular.z = 75*(self.mcp.read_adc(3)-521)/521
        
    

def publisher():
    rospy.init_node('controllerSender', anonymous=True)
    _controller = controller()
    rospy.loginfo("Initialised Node")
    r = rospy.Rate(10)
    while not (rospy.is_shutdown()):
        _controller.update()
        print("ControllerMSG\nLinear:\n{0}\nAngular\n{1}\n".format(_controller.msg.linear, _controller.msg.angular))
        pub.publish(_controller.msg)
        r.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
    finally:
        IO.cleanup()

