#!/usr/bin/env python

import rospy
import yaml
import os
from sensor_msgs.msg import CameraInfo
from camera_info_manager import CameraInfoManager

class CameraConfigNode:
    """
    Node to manage camera configuration and calibration
    """
    
    def __init__(self):
        rospy.init_node('camera_config')
        
        # Get parameters
        self.camera_names = rospy.get_param('~camera_names', ['front_camera', 'rear_camera', 'realsense'])
        self.camera_urls = rospy.get_param('~camera_urls', {})
        self.camera_frames = rospy.get_param('~camera_frames', {})
        
        # Create camera info managers and publishers
        self.managers = {}
        self.publishers = {}
        
        for camera_name in self.camera_names:
            # Get camera URL and frame ID
            url = self.camera_urls.get(camera_name, '')
            frame_id = self.camera_frames.get(camera_name, camera_name + '_optical_frame')
            
            # Create camera info manager
            self.managers[camera_name] = CameraInfoManager(cname=camera_name, url=url)
            self.managers[camera_name].loadCameraInfo()
            
            # Create publisher
            self.publishers[camera_name] = rospy.Publisher(
                '/{}/camera_info'.format(camera_name),
                CameraInfo,
                queue_size=10
            )
            
            rospy.loginfo("Configured camera: {} with URL: {}".format(camera_name, url))
        
        # Create timer to publish camera info
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_camera_info)
        
        rospy.loginfo("Camera configuration node initialized")
    
    def publish_camera_info(self, event):
        """
        Publish camera info for all cameras
        """
        now = rospy.Time.now()
        
        for camera_name, manager in self.managers.items():
            # Get camera info
            camera_info = manager.getCameraInfo()
            
            # Update header
            camera_info.header.stamp = now
            camera_info.header.frame_id = self.camera_frames.get(camera_name, camera_name + '_optical_frame')
            
            # Publish
            self.publishers[camera_name].publish(camera_info)

if __name__ == '__main__':
    try:
        node = CameraConfigNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
