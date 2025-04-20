#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Wrench, Point
from mavros_msgs.srv import CommandLong
import time

class Initialize(smach.State):
    """
    Initialize the water sampling mission
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized', 'failed'])
        
    def execute(self, userdata):
        rospy.loginfo('Initializing water sampling mission')
        
        # Check if all required systems are available
        try:
            rospy.wait_for_service('/mavros/cmd/command', timeout=5.0)
            rospy.loginfo('MAVROS command service available')
            return 'initialized'
        except rospy.ROSException:
            rospy.logerr('MAVROS command service not available')
            return 'failed'

class MoveToSamplingLocation(smach.State):
    """
    Move the ROV to the water sampling location
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached', 'failed'])
        self.wrench_pub = rospy.Publisher('/desired_wrench', Wrench, queue_size=1)
        self.depth_setpoint_pub = rospy.Publisher('/depth_setpoint', Float64, queue_size=1)
        self.heading_setpoint_pub = rospy.Publisher('/heading_setpoint', Float64, queue_size=1)
        
    def execute(self, userdata):
        rospy.loginfo('Moving to sampling location')
        
        # Set desired depth
        depth_msg = Float64()
        depth_msg.data = 1.0  # 1 meter depth
        self.depth_setpoint_pub.publish(depth_msg)
        rospy.loginfo('Setting depth to %.1f meters', depth_msg.data)
        
        # Set desired heading
        heading_msg = Float64()
        heading_msg.data = 0.0  # 0 degrees (North)
        self.heading_setpoint_pub.publish(heading_msg)
        rospy.loginfo('Setting heading to %.1f degrees', heading_msg.data)
        
        # Move forward
        wrench_msg = Wrench()
        wrench_msg.force.x = 10.0  # Forward force
        self.wrench_pub.publish(wrench_msg)
        rospy.loginfo('Moving forward')
        
        # Wait for a while
        time.sleep(5.0)
        
        # Stop moving
        wrench_msg.force.x = 0.0
        self.wrench_pub.publish(wrench_msg)
        rospy.loginfo('Reached sampling location')
        
        return 'reached'

class ActivateSampler(smach.State):
    """
    Activate the water sampler
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['sampled', 'failed'])
        self.command_client = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        
    def execute(self, userdata):
        rospy.loginfo('Activating water sampler')
        
        try:
            # Send MAVLink command to activate the sampler
            # Using a custom MAVLink command (example)
            response = self.command_client(
                command=31000,  # Custom command ID
                param1=1.0,     # Activate sampler
                param2=0.0,
                param3=0.0,
                param4=0.0,
                param5=0.0,
                param6=0.0,
                param7=0.0
            )
            
            if response.success:
                rospy.loginfo('Water sampler activated successfully')
                
                # Wait for sampling to complete
                time.sleep(3.0)
                
                return 'sampled'
            else:
                rospy.logerr('Failed to activate water sampler: %d', response.result)
                return 'failed'
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)
            return 'failed'

class ReturnToSurface(smach.State):
    """
    Return the ROV to the surface
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['surfaced', 'failed'])
        self.depth_setpoint_pub = rospy.Publisher('/depth_setpoint', Float64, queue_size=1)
        
    def execute(self, userdata):
        rospy.loginfo('Returning to surface')
        
        # Set depth to 0 (surface)
        depth_msg = Float64()
        depth_msg.data = 0.0
        self.depth_setpoint_pub.publish(depth_msg)
        rospy.loginfo('Setting depth to %.1f meters (surface)', depth_msg.data)
        
        # Wait for ROV to reach the surface
        time.sleep(10.0)
        
        rospy.loginfo('Reached surface')
        return 'surfaced'

def main():
    rospy.init_node('water_sampling_mission')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['mission_complete', 'mission_failed'])
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INITIALIZE', Initialize(), 
                               transitions={'initialized':'MOVE_TO_SAMPLING_LOCATION', 
                                            'failed':'mission_failed'})
        
        smach.StateMachine.add('MOVE_TO_SAMPLING_LOCATION', MoveToSamplingLocation(), 
                               transitions={'reached':'ACTIVATE_SAMPLER',
                                            'failed':'mission_failed'})
        
        smach.StateMachine.add('ACTIVATE_SAMPLER', ActivateSampler(), 
                               transitions={'sampled':'RETURN_TO_SURFACE',
                                            'failed':'mission_failed'})
        
        smach.StateMachine.add('RETURN_TO_SURFACE', ReturnToSurface(), 
                               transitions={'surfaced':'mission_complete',
                                            'failed':'mission_failed'})
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('water_sampling_mission', sm, '/WATER_SAMPLING_MISSION')
    sis.start()
    
    # Execute the state machine
    outcome = sm.execute()
    
    rospy.loginfo('Mission completed with outcome: %s', outcome)
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
