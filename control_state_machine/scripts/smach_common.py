#!/usr/bin/env python
import rospy as rp
import smach
import threading

from control_state_machine.msg import ControlEvent


class EventMonitorState(smach.State):
    def __init__(self, outcome_dict):
        smach.State.__init__(self, outcomes=outcome_dict.values() + ['aborted'])
        self._outcome_dict = outcome_dict
        self._ev_type = None
        self._sub = rp.Subscriber("behavior_manager/control_events", ControlEvent, self.callback, queue_size=1)
        self._thread_ev = threading.Event()

    def callback(self, msg):
        if msg.type in self._outcome_dict:
            self._ev_type = msg.type
            self._thread_ev.set()

    def execute(self, userdata):
        while not rp.is_shutdown():
            if self._thread_ev.wait(1):
                self._thread_ev.clear()
                if self._ev_type in self._outcome_dict:
                    return self._outcome_dict[self._ev_type]
        return 'aborted'
