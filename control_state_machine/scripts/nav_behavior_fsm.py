#!/usr/bin/env python
import rospy as rp
import rospkg
import smach
import smach_ros
import threading
import actionlib
import abc

#from tf.transformations import quaternion_from_euler
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from control_state_machine.msg import ControlEvent, StateInfo
from smach_common import EventMonitorState
from indires_macro_actions.msg import *




class NavigationState(smach.State):
    __metaclass__ = abc.ABCMeta

    def __init__(self, macro_name, macro_type):
        smach.State.__init__(self, outcomes=['aborted',
                                             'blocked',
                                             'low battery',
                                             'go home',
                                             'teleoperation',
                                             'nav to waypoint',
                                             'exploration',
                                             'succeeded',
                                             'preempted',
                                             'unknown'])
        self._control_ret = None
        self._ac = actionlib.SimpleActionClient(macro_name, macro_type)
        self._ac_move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self._macro_type = macro_type
        self._macro_name = macro_name
        self._actionlib_status_dict = {GoalStatus.ABORTED: 'aborted',
                                       GoalStatus.PREEMPTED: 'preempted',
                                       GoalStatus.SUCCEEDED: 'succeeded'}
        self._control_ev_dict = {ControlEvent.GO_HOME: 'go home',
                                 ControlEvent.LOW_BATT: 'low battery',
                                 ControlEvent.TELEOPERATION: 'teleoperation',
                                 ControlEvent.NAV_TO_WAYPOINT_REQUESTED: 'nav to waypoint',
                                 ControlEvent.EXPLORATION_REQUESTED: 'exploration',
                                 ControlEvent.CANCEL: 'aborted'}
        while not self._ac.wait_for_server(rp.Duration(1.0)):
            rp.logwarn("Action state '%s' waiting for navigation actionlib server to start..."
                       % macro_name)
        #self._stalkpub = rp.Publisher("/stalk_ref", StalkRef, queue_size=1)
        rp.loginfo("Action state '%s' ready to execute." % macro_name)
        self._stateinfo_pub = rp.Publisher("behavior_manager/state_info", StateInfo, queue_size=1)
        self._events_sub = rp.Subscriber("behavior_manager/control_events",
                                         ControlEvent, self.event_cb)
    @abc.abstractmethod
    def get_goal(self):
        raise NotImplementedError("You can't call this pure virtual method!")

    def event_cb(self, msg):
        if msg.type == ControlEvent.CANCEL:
            rp.loginfo("Cancel command detected")
            #self._ac.cancel_all_goals()  #no hace nada
            self._ac_move_base.cancel_all_goals()
        #rp.loginfo("Received event: ")
        print msg.type	
        if msg.type in self._control_ev_dict:
            self._control_ret = self._control_ev_dict[msg.type]
            self._ac.cancel_all_goals()  # ac already waiting for result

    def execute(self, userdata):
        
        self._control_ret = None
        self.get_goal()
        rp.loginfo("Macro-action '%s': Sending goal." % self._macro_name)
        stateinfo = StateInfo(state_name=self._macro_name, info='started')
        self._stateinfo_pub.publish(stateinfo)
        GoalType = globals()[self._macro_name + "Goal"]
        if self._control_ret is None:
            self._ac.send_goal(GoalType(self._goal))
            self._ac.wait_for_result()
            a = str(self._ac.get_goal_status_text())
            if a.find('blocked') > -1:
                stateinfo = StateInfo(state_name=self._macro_name, info='ended:blocked')
                self._stateinfo_pub.publish(stateinfo)
                return 'blocked'    

            if self._control_ret is not None:
                stateinfo = StateInfo(state_name=self._macro_name, info='ended:interrupted')
                self._stateinfo_pub.publish(stateinfo)
                return self._control_ret

            goal_state = self._ac.get_state()
            try:
                retstr = self._actionlib_status_dict[goal_state]
                stateinfo = StateInfo(state_name=self._macro_name, info='ended:%s' % retstr)
                self._stateinfo_pub.publish(stateinfo)
                return retstr
            except KeyError:
                stateinfo = StateInfo(state_name=self._macro_name, info='ended:unknown')
                self._stateinfo_pub.publish(stateinfo)
                return 'unknown'
        else:
            stateinfo = StateInfo(state_name=self._macro_name, info='ended:interrupted')
            self._stateinfo_pub.publish(stateinfo)
            return self._control_ret






class NavGoalActionState(NavigationState):
    def __init__(self, macro_name, macro_type):
        self._thr_ev = threading.Event()
        super(NavGoalActionState, self).__init__(macro_name, macro_type)

    def get_goal(self):
        rp.Subscriber("behavior_manager/nav_goal",
                      PoseStamped,
                      self.goal_cb)
        self._thr_ev.wait()
        self._thr_ev.clear()

    def goal_cb(self, msg):
        self._goal = msg
        self._thr_ev.set()



class ExplorationActionState(NavigationState):
    def __init__(self, macro_name, macro_type):
        self._thr_ev = threading.Event()
        super(ExplorationActionState, self).__init__(macro_name, macro_type)

    def get_goal(self):
        #rp.Subscriber("behavior_manager/nav_goal",
        #              PoseStamped,
        #              self.goal_cb)
        msg = PoseStamped()
        msg.header.stamp = rp.Time.now()
        msg.header.frame_id = ""
        self._goal = msg
        self._thr_ev.set()
        #self._thr_ev.wait()
        self._thr_ev.clear()

    #def goal_cb(self, msg):
    #    self._goal = msg
    #    self._thr_ev.set()






class HomeGoalActionState(NavigationState):
    def __init__(self, macro_name, macro_type):
		
        home_goal = PoseStamped()
        home_goal.header.stamp = rp.Time.now()
        home_goal.header.frame_id = 'map'
        home_goal.pose.position.x = 0.0
        home_goal.pose.position.y = 0.0
        home_goal.pose.position.z = 0.0
        #quater = (0.0, 0.0, 0.0, 1.0)
        #home_goal.pose.orientation = Quaternion(quater)
        home_goal.pose.orientation.x = 0.0
        home_goal.pose.orientation.y = 0.0
        home_goal.pose.orientation.z = 0.0
        home_goal.pose.orientation.w = 1.0
        self._goal = home_goal
        super(HomeGoalActionState, self).__init__(macro_name, macro_type)

    def get_goal(self):
        return self._goal  # already in self._goal



class TeleoperationActionState(NavigationState):
    def __init__(self, macro_name, macro_type):
		
        self._goal = Twist()
        super(TeleoperationActionState, self).__init__(macro_name, macro_type)

    def get_goal(self):
        return self._goal  # already in self._goal










class NavigationBehaviorTest(smach.StateMachine):
    def __init__(self, conversation_active=False):
        smach.StateMachine.__init__(self,
                                    outcomes=['nav error',
                                              'nav preempted',
                                              'nav succeeded',
                                              'nav exit'],
                                    input_keys=['transition_label'])
        self._initial_state_dict = {'init': 'Wait for Action',
                                    'nav to waypoint': 'Navigate to Waypoint',
                                    'exploration': 'Start Exploration',
                                    'teleoperation': 'Manual Teleoperation',
                                    'go home': 'Navigate Home',
                                    'low batt': 'Navigate Home'}
        
        tran_if_goal_reached = 'Wait for Action' 	

        with self:
            self.add('Wait for Action',
                     EventMonitorState({ControlEvent.NAV_TO_WAYPOINT_REQUESTED: 'nav to waypoint',
                                        ControlEvent.EXPLORATION_REQUESTED: 'exploration',
                                        ControlEvent.TELEOPERATION: 'teleoperation',
                                        ControlEvent.GO_HOME: 'go home',
                                        ControlEvent.LOW_BATT: 'low batt',
                                        ControlEvent.CANCEL:'cancel'}),
                     transitions={'nav to waypoint': 'Navigate to Waypoint',
                                  'exploration': 'Start Exploration',
                                  'teleoperation': 'Manual Teleoperation',
                                  'go home': 'Navigate Home',
                                  'low batt': 'Navigate Home',
                                  'aborted': 'Wait for Action',
                                  'cancel': 'Wait for Action'})
                                  #'aborted': 'nav exit'})  # Warning: Not preemptable - needs concurrency for that
            self.add('Navigate to Waypoint',
                     NavGoalActionState("NavigateWaypoint", NavigateWaypointAction),
                     transitions={'succeeded': tran_if_goal_reached, 
                                  'aborted': 'Wait for Action',    #'Navigate to Waypoint',
                                  'nav to waypoint': 'Navigate to Waypoint',
                                  'exploration': 'Start Exploration',
                                  'blocked': 'Wait for Action',
                                  'teleoperation': 'Manual Teleoperation',
                                  'go home': 'Navigate Home',
                                  'low battery': 'Navigate Home',
                                  #'cancel': 'Wait for Action',
                                  'preempted': 'nav preempted',
                                  'unknown': 'nav error'})
            self.add('Start Exploration',
                     ExplorationActionState("Exploration", ExplorationAction),
                     transitions={'aborted': 'Wait for Action', 
                                  'succeeded': 'Wait for Action',
                                  'nav to waypoint': 'Navigate to Waypoint',
                                  'exploration': 'Start Exploration',
                                  'blocked': 'Wait for Action',
                                  'teleoperation': 'Manual Teleoperation',
                                  'go home': 'Navigate Home',
                                  'low battery': 'Navigate Home',
                                  #'cancel': 'Wait for Action',
                                  'preempted': 'nav preempted',
                                  'unknown': 'nav error'})
            self.add('Navigate Home',
                     HomeGoalActionState("NavigateHome", NavigateHomeAction),
                     transitions={'succeeded': 'Wait for Action',
                                  'aborted': 'Wait for Action',    #'Navigate Home',
                                  'nav to waypoint': 'Navigate Home', 
                                  'exploration': 'Navigate Home', 
                                  'blocked': 'Wait for Action',
                                  'teleoperation': 'Manual Teleoperation',
                                  'go home': 'Navigate Home', 
                                  'low battery': 'Navigate Home',
                                  #'cancel': 'Wait for Action',
                                  'preempted': 'nav preempted',
                                  'unknown': 'nav error'})
            self.add('Manual Teleoperation', 
                     TeleoperationActionState("Teleoperation", TeleoperationAction),
                     transitions={'succeeded': 'Wait for Action', 
                                  'aborted': 'Manual Teleoperation',
                                  'nav to waypoint': 'Navigate to Waypoint',
                                  'exploration': 'Start Exploration',
                                  'blocked': 'nav error',
                                  'teleoperation': 'Manual Teleoperation',
                                  'go home': 'Navigate Home',
                                  'low battery': 'Navigate Home',
                                  'preempted': 'nav preempted',
                                  'unknown': 'nav error'})

    def execute(self, parent_ud=smach.UserData()):
        try:
            l = parent_ud.transition_label
        except KeyError:
            l = 'init'
        s = self._initial_state_dict[l]
        self.set_initial_state([s])
        out = smach.StateMachine.execute(self, parent_ud)
        return out

if __name__ == '__main__':
    rp.init_node("indires_navigation_fsm")

    sm = NavigationBehaviorTest()

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('indires_navigation_fsm', sm, '/SM_ROOT')
    sis.start()

    # Execute state machine
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()
    rp.spin()
    sm.request_preempt()
    smach_thread.join()
    sis.stop()
