#!/usr/bin/python2
import rospy as rp
import rospkg
import yaml
import math


from control_state_machine.msg import ControlEvent   
from geometry_msgs.msg import PoseStamped


class EventPublisher(object):
    def __init__(self):
        

        self._ce_pub = rp.Publisher("behavior_manager/control_events", ControlEvent, queue_size=1)
        self._goal_pub = rp.Publisher("behavior_manager/nav_goal",
                                      PoseStamped,
                                      queue_size=1,
                                      latch=True)


    def publish_waypoint_req(self):
       
        g = PoseStamped()
        g.header.frame_id = 'indires_rover/odom'
        g.header.stamp = rp.Time.now()
        g.pose.orientation.x = 0
        g.pose.orientation.y = 0
        g.pose.orientation.z = 0
        g.pose.orientation.w = 1.0
        
        wayp_selected = False
        while not wayp_selected:
            try:
                print 'Select waypoint coordinate X:'
                x = float(raw_input('> '))
                if x <= 5.0 and x >= -5.0:
					g.pose.position.x = x
					print 'Select waypoint coordinate Y:'
					y = float(raw_input('> '))
					if y <= 5.0 and y >= -5.0:
					    g.pose.position.y = y
					    print 'Select waypoint coordinate Z:'
					    z = float(raw_input('> '))
					    if z <= 5.0 and z >= -5.0:
					        g.pose.position.z = z
					        wayp_selected = True
                    
            except ValueError:
                rp.logwarn("Not a valid input.")
            finally:
                if not wayp_selected:
                    print 'Incorrect waypoint index.'
        
        self._goal_pub.publish(g)

        ce = ControlEvent()
        ce.stamp = rp.Time.now()
        ce.type = ControlEvent.NAV_TO_WAYPOINT_REQUESTED
        self._ce_pub.publish(ce)
        
    
    
    
    def publish_exploration_req(self):
       
        g = PoseStamped()
        g.header.frame_id = ''
        g.header.stamp = rp.Time.now()
        g.pose.orientation.x = 0.0
        g.pose.orientation.y = 0.0
        g.pose.orientation.z = 0.0
        g.pose.orientation.w = 1.0
        self._goal_pub.publish(g)

        ce = ControlEvent()
        ce.stamp = rp.Time.now()
        ce.type = ControlEvent.EXPLORATION_REQUESTED
        self._ce_pub.publish(ce)    
    
    
    def publish_gohome_req(self):
       
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = rp.Time.now()
        g.pose.position.x = 0
        g.pose.position.y = 0
        g.pose.position.z = 0
        g.pose.orientation.x = 0
        g.pose.orientation.y = 0
        g.pose.orientation.z = 0
        g.pose.orientation.w = 1.0
        self._goal_pub.publish(g)

        ce = ControlEvent()
        ce.stamp = rp.Time.now()
        ce.type = ControlEvent.GO_HOME
        self._ce_pub.publish(ce)    
        

    def publish_low_batt(self):
        ce = ControlEvent()
        ce.stamp = rp.Time.now()
        ce.type = ControlEvent.LOW_BATT
        self._ce_pub.publish(ce)

    def publish_cancel(self):
        ce = ControlEvent()
        ce.stamp = rp.Time.now()
        ce.type = ControlEvent.CANCEL
        self._ce_pub.publish(ce)

    
    


if __name__ == '__main__':
    rp.init_node('indires_tester')

    gp = EventPublisher()

    rp.loginfo('test node running')
    option_dict = {'1': gp.publish_waypoint_req,
                   '2': gp.publish_exploration_req,
                   '3': gp.publish_gohome_req,
                   '4': gp.publish_low_batt,
                   '5': gp.publish_cancel}

    while not rp.is_shutdown():
        print 'Select an event type:'
        print '1: Navigate to Waypoint Request'
        print '2: Exploration Request'
        print '3: Go home Request'
        print '4: Low Battery'
        print '5: Cancel Action'

        a = raw_input('> ')
        try:
            option_dict[a]()
        except KeyError:
            print 'Option not recognized. Please select an option from the list'

    rp.spin()
