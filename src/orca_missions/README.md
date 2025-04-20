# Orca ROV Missions Package

This package contains mission planning and execution for the Orca ROV.

## Overview

The Orca ROV missions package provides:
- Water sampling mission
- Object retrieval mission
- State machine-based mission execution using SMACH

## Missions

### Water Sampling Mission

The water sampling mission performs the following steps:
1. Initialize the mission
2. Move to the sampling location
3. Activate the water sampler
4. Return to the surface

```bash
rosrun orca_missions water_sampling_mission.py
```

### Object Retrieval Mission

The object retrieval mission performs the following steps:
1. Initialize the mission
2. Search for the target object
3. Approach the object
4. Grasp the object with the gripper
5. Return to the surface with the object

```bash
rosrun orca_missions object_retrieval_mission.py
```

## Launch Files

### water_sampling.launch

Launches the water sampling mission.

```bash
roslaunch orca_missions water_sampling.launch
```

### object_retrieval.launch

Launches the object retrieval mission.

```bash
roslaunch orca_missions object_retrieval.launch
```

## Configuration

The missions are configured via the `missions.yaml` file in the `config` directory. This file includes:
- Water sampling mission parameters
- Object retrieval mission parameters

## State Machine Visualization

The missions use SMACH for state machine-based execution. You can visualize the state machine using the SMACH viewer:

```bash
rosrun smach_viewer smach_viewer.py
```

## Mission States

### Water Sampling Mission States

- **Initialize**: Checks if all required systems are available
- **MoveToSamplingLocation**: Moves the ROV to the sampling location
- **ActivateSampler**: Activates the water sampler
- **ReturnToSurface**: Returns the ROV to the surface

### Object Retrieval Mission States

- **Initialize**: Checks if all required systems are available
- **SearchForObject**: Searches for the target object
- **ApproachObject**: Approaches the detected object
- **GraspObject**: Grasps the object with the gripper
- **SurfaceWithObject**: Returns to the surface with the grasped object

## Custom Mission Development

To create a custom mission:

1. Create a new Python script in the `scripts` directory
2. Define the mission states as SMACH states
3. Create a state machine and add the states
4. Add appropriate transitions between states
5. Create a launch file for the mission

Example:

```python
#!/usr/bin/env python

import rospy
import smach
import smach_ros

class MyMissionState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        
    def execute(self, userdata):
        # Implement state behavior
        return 'success'

def main():
    rospy.init_node('my_mission')
    
    # Create state machine
    sm = smach.StateMachine(outcomes=['mission_complete', 'mission_failed'])
    
    # Add states
    with sm:
        smach.StateMachine.add('MY_STATE', MyMissionState(), 
                               transitions={'success':'mission_complete', 
                                            'failure':'mission_failed'})
    
    # Execute state machine
    outcome = sm.execute()
    
    rospy.spin()

if __name__ == '__main__':
    main()
```
