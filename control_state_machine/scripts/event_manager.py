#!/usr/bin/env python
import rospy as rp
import numpy as np
import yaml
import rospkg
import tf
import threading


from control_state_machine.msg import ControlEvent
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import UInt16

class ControlEventBase(object):
    def __init__(self, event_type):
        self._pub = rp.Publisher("behavior_manager/control_events", ControlEvent, queue_size=1)
        self._etype = event_type

    def trigger(self):
        ce = ControlEvent()
        ce.stamp = rp.Time.now()
        ce.type = self._etype
        #print("Trigering event type: %i" % ce.type)
        self._pub.publish(ce)


class LowBatteryEvent(ControlEventBase):
    """
    Triggered whenever the battery level goes below a certain voltage
    """
    def __init__(self):
        super(LowBatteryEvent, self).__init__(ControlEvent.LOW_BATT)
        
        #TODO - CHANGE TO THE CORRESPONDING TOPIC OF THE RAPOSA
        self._sub = rp.Subscriber("ups_battery_voltage", Float32, self.callback, queue_size=1)
        
        self._thr = rp.get_param("event_params/low_battery_event/threshold", 12.0)

    def callback(self, msg):
        if msg.data < self._thr:
            self.trigger()


class ManualControlEvent(ControlEventBase):
    """
    Triggered whenever the user sends a velocity command to the robot
    """
    def __init__(self):
        super(ManualControlEvent, self).__init__(ControlEvent.TELEOPERATION)
        self._sub = rp.Subscriber("manual_cmd_vel", Twist, self.callback, queue_size=1)

    def callback(self, msg):
        #print("manual_cmd_vel received")
        self.trigger()



if __name__ == '__main__':
    rp.init_node('event_manager')

    p = rospkg.RosPack().get_path('control_state_machine')

    direct_events = [LowBatteryEvent(), ManualControlEvent()]

    #personloc_evmanager = PersonLocalizationEventManager()

    rp.loginfo('Event manager initialized.')
    rp.spin()
