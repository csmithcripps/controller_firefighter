#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import sys
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import Gdk

pub = rospy.Publisher('keyboard', String, queue_size=10)




class vel_system():
    KEY_ACTIONS = {
        "w" : 1,
        "s" : -1,
        "d" : -1,
        "a" : 1,
        "e" : -1,
        "q" : 1,
    }
    def __init__(self):
        self.msg = Twist()
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.z = 0
        self.defaultVelocity = 1

        self.pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)

    def changeVelocity(self, keylist):
        x = 0
        y = 0
        w = 0
        for key in keylist:
            if key == "w" or key == "s":
                x += self.KEY_ACTIONS[key]
            if key == "a" or key == "d":
                y += self.KEY_ACTIONS[key]
            if key == "q" or key == "e":
                w += self.KEY_ACTIONS[key]

        self.msg.linear.x = x*self.defaultVelocity
        self.msg.linear.y = y*self.defaultVelocity
        self.msg.angular.z = w*self.defaultVelocity
        self.publish()

        
        
    def getTwistMsg(self):
        return self.msg

    def publish(self):
        rospy.loginfo("\nSending Vel Info: \nLinear:\n" + str(self.msg.linear) + 
                                       "\nAngular:\n" + str(self.msg.angular))
        self.pub.publish(self.msg)

class MyWindow(Gtk.Window):

    keylist = {}
    def __init__(self):
        # init the base class (Gtk.Window)
        super().__init__()

        self.VelSystem = vel_system()

        # state affected by shortcuts
        self.prev_key = ""

        # Tell Gtk what to do when the window is closed (in this case quit the main loop)
        self.connect("delete-event", Gtk.main_quit)

        # connect the key-press event - this will call the keypress
        # handler when any key is pressed
        self.connect("key-press-event",self.on_key_press_event)
        self.connect("key-release-event",self.on_key_release_event)

        # Window content goes in a vertical box
        box = Gtk.VBox()

        # the label that will respond to the event
        self.label = Gtk.Label(label="")
        self.update_label_text()

        # Add the label to the window
        box.add(self.label)

        self.add(box)

    def on_key_press_event(self, widget, event):
        # check the event modifiers (can also use SHIFTMASK, etc)
        ctrl = (event.state & Gdk.ModifierType.CONTROL_MASK)
        key = Gdk.keyval_name(event.keyval)

        if not (key in self.keylist):
            self.keylist[key] = 1
            self.VelSystem.changeVelocity(self.keylist)

        self.prev_key = key
        self.update_label_text()

    
    def on_key_release_event(self, widget, event):

        # check the event modifiers (can also use SHIFTMASK, etc)
        ctrl = (event.state & Gdk.ModifierType.CONTROL_MASK)
        key = Gdk.keyval_name(event.keyval).lower()
        if (key in self.keylist):
            self.keylist.pop(key)
            self.VelSystem.changeVelocity(self.keylist)

        self.prev_key = key
        self.update_label_text()
        

    def update_label_text(self):
        # Update the label based on the state of the hit variable
        self.label.set_text("KEYPRESS MOVEMENT" + 
                            "\nMost Recent Key: %s \n" % self.prev_key + 
                            "\nVELOCITIES" + 
                            "\nLinear:\n" + str(self.VelSystem.msg.linear) + 
                            "\nAngular:\n" + str(self.VelSystem.msg.angular))

if __name__ == "__main__":
    rospy.init_node('keyboardRead', anonymous=True)
    rospy.loginfo("Initialised Node")
    win = MyWindow()
    win.show_all()

    # Start the Gtk main loop
    Gtk.main()