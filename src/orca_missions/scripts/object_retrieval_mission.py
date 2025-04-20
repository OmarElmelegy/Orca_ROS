#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import Wrench, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class Initialize(smach.State):
    """
    Initialize the object retrieval mission
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized', 'failed'])
        
    def execute(self, userdata):
        rospy.loginfo('Initializing object retrieval mission')
        
        # Check if all required systems are available
        try:
            # Check if object detection is running
            rospy.wait_for_message('/detected_object', Point, timeout=5.0)
            rospy.loginfo('Object detection is running')
            
            # Check if gripper control is available
            rospy.wait_for_service('/mavros/cmd/command', timeout=5.0)
            rospy.loginfo('MAVROS command service available')
            
            return 'initialized'
        except rospy.ROSException:
            rospy.logerr('Required services or topics not available')
            return 'failed'

class SearchForObject(smach.State):
    """
    Search for the target object
    """
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['object_found', 'search_failed'],
                            output_keys=['object_position'])
        self.wrench_pub = rospy.Publisher('/desired_wrench', Wrench, queue_size=1)
        self.object_sub = rospy.Subscriber('/detected_object', Point, self.object_callback)
        self.object_type_sub = rospy.Subscriber('/object_type', String, self.object_type_callback)
        self.object_position = None
        self.object_type = None
        self.search_timeout = 30.0  # seconds
        
    def object_callback(self, msg):
        self.object_position = msg
        
    def object_type_callback(self, msg):
        self.object_type = msg.data
        
    def execute(self, userdata):
        rospy.loginfo('Searching for object')
        
        # Reset object position
        self.object_position = None
        self.object_type = None
        
        # Start search pattern (rotate slowly)
        wrench_msg = Wrench()
        wrench_msg.torque.z = 2.0  # Rotate clockwise
        self.wrench_pub.publish(wrench_msg)
        
        # Search for a while
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10 Hz
        
        while (rospy.Time.now() - start_time).to_sec() < self.search_timeout:
            if self.object_position is not None and self.object_type is not None:
                # Stop rotating
                wrench_msg.torque.z = 0.0
                self.wrench_pub.publish(wrench_msg)
                
                # Store object position for next state
                userdata.object_position = self.object_position
                
                rospy.loginfo('Found %s at position (%.1f, %.1f)', 
                             self.object_type, 
                             self.object_position.x, 
                             self.object_position.y)
                
                return 'object_found'
            
            rate.sleep()
        
        # Search timeout
        wrench_msg.torque.z = 0.0
        self.wrench_pub.publish(wrench_msg)
        
        rospy.logwarn('Search timeout, no object found')
        return 'search_failed'

class ApproachObject(smach.State):
    """
    Approach the target object
    """
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['approached', 'approach_failed'],
                            input_keys=['object_position'])
        self.wrench_pub = rospy.Publisher('/desired_wrench', Wrench, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/front_camera/image_raw', Image, self.image_callback)
        self.image_width = 640  # Default value
        self.image_height = 480  # Default value
        self.approach_timeout = 30.0  # seconds
        
    def image_callback(self, msg):
        try:
            # Get image dimensions
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_height, self.image_width, _ = cv_image.shape
        except Exception as e:
            rospy.logerr("Error processing image: %s", e)
        
    def execute(self, userdata):
        rospy.loginfo('Approaching object')
        
        # Get object position from userdata
        object_position = userdata.object_position
        
        # Calculate object position relative to image center
        center_x = self.image_width / 2
        center_y = self.image_height / 2
        
        # Start approach
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10 Hz
        
        while (rospy.Time.now() - start_time).to_sec() < self.approach_timeout:
            # Calculate position error
            error_x = object_position.x - center_x
            error_y = object_position.y - center_y
            
            # Create wrench command
            wrench_msg = Wrench()
            
            # Adjust heading to center object horizontally
            wrench_msg.torque.z = -0.05 * error_x
            
            # Move forward
            wrench_msg.force.x = 5.0
            
            # Publish wrench
            self.wrench_pub.publish(wrench_msg)
            
            # Check if we're close enough (object is large in the image)
            # This is a simplified approach - in reality, you'd use depth information
            if abs(error_x) < 50 and object_position.y > (self.image_height * 0.7):
                # Stop moving
                wrench_msg.force.x = 0.0
                wrench_msg.torque.z = 0.0
                self.wrench_pub.publish(wrench_msg)
                
                rospy.loginfo('Approached object')
                return 'approached'
            
            rate.sleep()
        
        # Approach timeout
        wrench_msg = Wrench()
        self.wrench_pub.publish(wrench_msg)
        
        rospy.logwarn('Approach timeout')
        return 'approach_failed'

class GraspObject(smach.State):
    """
    Grasp the target object with the gripper
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['grasped', 'grasp_failed'])
        self.left_gripper_pub = rospy.Publisher('/left_gripper/command', Bool, queue_size=1)
        self.right_gripper_pub = rospy.Publisher('/right_gripper/command', Bool, queue_size=1)
        
    def execute(self, userdata):
        rospy.loginfo('Grasping object')
        
        # Open grippers
        self.left_gripper_pub.publish(True)
        self.right_gripper_pub.publish(True)
        rospy.loginfo('Opening grippers')
        
        # Wait for grippers to open
        time.sleep(2.0)
        
        # Close grippers
        self.left_gripper_pub.publish(False)
        self.right_gripper_pub.publish(False)
        rospy.loginfo('Closing grippers')
        
        # Wait for grippers to close
        time.sleep(2.0)
        
        rospy.loginfo('Object grasped')
        return 'grasped'

class SurfaceWithObject(smach.State):
    """
    Return to the surface with the grasped object
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['surfaced', 'failed'])
        self.depth_setpoint_pub = rospy.Publisher('/depth_setpoint', Float64, queue_size=1)
        
    def execute(self, userdata):
        rospy.loginfo('Returning to surface with object')
        
        # Set depth to 0 (surface)
        depth_msg = Float64()
        depth_msg.data = 0.0
        self.depth_setpoint_pub.publish(depth_msg)
        rospy.loginfo('Setting depth to %.1f meters (surface)', depth_msg.data)
        
        # Wait for ROV to reach the surface
        time.sleep(10.0)
        
        rospy.loginfo('Reached surface with object')
        return 'surfaced'

def main():
    rospy.init_node('object_retrieval_mission')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['mission_complete', 'mission_failed'])
    
    # Create user data
    sm.userdata.object_position = None
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INITIALIZE', Initialize(), 
                               transitions={'initialized':'SEARCH_FOR_OBJECT', 
                                            'failed':'mission_failed'})
        
        smach.StateMachine.add('SEARCH_FOR_OBJECT', SearchForObject(), 
                               transitions={'object_found':'APPROACH_OBJECT',
                                            'search_failed':'mission_failed'})
        
        smach.StateMachine.add('APPROACH_OBJECT', ApproachObject(), 
                               transitions={'approached':'GRASP_OBJECT',
                                            'approach_failed':'SEARCH_FOR_OBJECT'})
        
        smach.StateMachine.add('GRASP_OBJECT', GraspObject(), 
                               transitions={'grasped':'SURFACE_WITH_OBJECT',
                                            'grasp_failed':'APPROACH_OBJECT'})
        
        smach.StateMachine.add('SURFACE_WITH_OBJECT', SurfaceWithObject(), 
                               transitions={'surfaced':'mission_complete',
                                            'failed':'mission_failed'})
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('object_retrieval_mission', sm, '/OBJECT_RETRIEVAL_MISSION')
    sis.start()
    
    # Execute the state machine
    outcome = sm.execute()
    
    rospy.loginfo('Mission completed with outcome: %s', outcome)
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
