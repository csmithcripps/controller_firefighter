#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import time
import sys

    

  
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import Gdk

pub = rospy.Publisher('keyboard', String, queue_size=10)


class MyWindow(Gtk.Window):

    keyList = {}

    def __init__(self):
        # init the base class (Gtk.Window)
        super().__init__()

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
        if (key) and (key in self.keyList) != True:
            self.keyList[key] = "Pressed"
            rospy.loginfo("Key Pressed: '" + key + "'")
            keyboardMSG = String()
            keyboardMSG.data = key
            pub.publish(keyboardMSG)
            time.sleep(0.01)

        self.prev_key = key
        self.update_label_text()

    
    def on_key_release_event(self, widget, event):

        # check the event modifiers (can also use SHIFTMASK, etc)
        ctrl = (event.state & Gdk.ModifierType.CONTROL_MASK)
        key = Gdk.keyval_name(event.keyval)
        if (key) and (key in self.keyList):
            self.keyList.pop(key)
            rospy.loginfo("Key Released: '" + key + "'")
            keyboardMSG = String()
            keyboardMSG.data = "_" + key
            pub.publish(keyboardMSG)
            time.sleep(0.01)

        self.prev_key = key
        self.update_label_text()
        

    def update_label_text(self):
        # Update the label based on the state of the hit variable
        self.label.set_text("Most Recent Key: %s " % self.prev_key)

if __name__ == "__main__":
    rospy.init_node('keyboardRead', anonymous=True)
    rospy.loginfo("Initialised Node")
    win = MyWindow()
    win.show_all()

    # Start the Gtk main loop
    Gtk.main()