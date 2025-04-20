#!/usr/bin/env python

import rospy
import mavros
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Bool, Float64

class GripperControl:
    """
    Control the Newton grippers on the Orca ROV using MAVROS
    """
    
    def __init__(self):
        rospy.init_node('gripper_control')
        
        # Parameters
        self.left_servo_channel = rospy.get_param('~left_servo_channel', 9)
        self.right_servo_channel = rospy.get_param('~right_servo_channel', 10)
        self.open_pwm = rospy.get_param('~open_pwm', 1900)
        self.close_pwm = rospy.get_param('~close_pwm', 1100)
        
        # Initialize MAVROS command service
        self.command_client = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        rospy.loginfo("Waiting for MAVROS command service...")
        self.command_client.wait_for_service()
        rospy.loginfo("MAVROS command service available")
        
        # Subscribers
        rospy.Subscriber('left_gripper/command', Bool, self.left_gripper_callback)
        rospy.Subscriber('right_gripper/command', Bool, self.right_gripper_callback)
        rospy.Subscriber('left_gripper/position', Float64, self.left_position_callback)
        rospy.Subscriber('right_gripper/position', Float64, self.right_position_callback)
        
        rospy.loginfo("Gripper control node initialized")
    
    def left_gripper_callback(self, msg):
        """
        Handle left gripper open/close commands
        """
        pwm = self.open_pwm if msg.data else self.close_pwm
        self.set_servo(self.left_servo_channel, pwm)
        rospy.loginfo(f"Left gripper {'opened' if msg.data else 'closed'}")
    
    def right_gripper_callback(self, msg):
        """
        Handle right gripper open/close commands
        """
        pwm = self.open_pwm if msg.data else self.close_pwm
        self.set_servo(self.right_servo_channel, pwm)
        rospy.loginfo(f"Right gripper {'opened' if msg.data else 'closed'}")
    
    def left_position_callback(self, msg):
        """
        Handle left gripper position commands (0.0-1.0)
        """
        pwm = self.interpolate_pwm(msg.data)
        self.set_servo(self.left_servo_channel, pwm)
        rospy.loginfo(f"Left gripper position set to {msg.data:.2f}")
    
    def right_position_callback(self, msg):
        """
        Handle right gripper position commands (0.0-1.0)
        """
        pwm = self.interpolate_pwm(msg.data)
        self.set_servo(self.right_servo_channel, pwm)
        rospy.loginfo(f"Right gripper position set to {msg.data:.2f}")
    
    def interpolate_pwm(self, position):
        """
        Interpolate between close_pwm and open_pwm based on position (0.0-1.0)
        """
        position = max(0.0, min(1.0, position))  # Clamp to [0, 1]
        return int(self.close_pwm + position * (self.open_pwm - self.close_pwm))
    
    def set_servo(self, channel, pwm):
        """
        Send MAVLink command to set servo PWM
        """
        # MAV_CMD_DO_SET_SERVO (183)
        # param1: Servo number
        # param2: PWM value
        try:
            response = self.command_client(
                command=183,  # MAV_CMD_DO_SET_SERVO
                param1=float(channel),
                param2=float(pwm),
                param3=0,
                param4=0,
                param5=0,
                param6=0,
                param7=0
            )
            if not response.success:
                rospy.logwarn(f"Failed to set servo {channel} to PWM {pwm}: {response.result}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        gripper_control = GripperControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
