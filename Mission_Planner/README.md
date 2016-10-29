# Event Handler Readme

## Event Handler TODO - Nathan/Paulo

#### What does the Event handler do?

The event handler will take in information from other topics and interpret them before sending important information to the Mission Planner.

The ultimate goal of the Event Handler is to be able to identify what kind of object lies in the vision of the robot and where 

#### Inputs

* Pose			// Used to determine 
* OccupancyGrid		// Used to find important objects
* CameraHeading
* CameraColor

#### Outputs

* X and Y in Ogrid for location of important object

#### Assumptions

* Lidar will always find objuects even if camera does not
* We only publish locations if: We have a actual occupied area in OGrid that is big enough to be an object, and we can get some kind of color nd heading from camera in object general location We will prioritize and label object depending on size and color of object.
