# simulation_management_ros_tool
Core functionality of the simulation framework.

## Installation
* this package is part of the simulation framework
* see [coincarsim_getting_started](https://github.com/coincar-sim/coincarsim_getting_started) for installation and more details

# localization management
Managing the pose of all objects within the framework.

#### Working Principle
* objects are initialized via an `ObjectInitialization.msg`
* they provide their desired motion via a `DeltaTrajectoryWithID.msg`
* the management keeps track of the current position and interpolates linearly along the desired motion
* the ground truth poses of all objects are published via an `ObjectStateArray.msg`

#### Settings
* topics for the upper messages
* frame_ids for the reference frame and the prefix for the objects
* the frequency with which the ground truth poses are published (dynamically reconfigurable)
* see [cfg/LocalizationMgmt.mrtcfg](cfg/LocalizationMgmt.mrtcfg) for details

#### Usage
* launch the file `launch/simulation_management.launch`

# time management
Providing the simulation time.

#### Working Principle
* publishes the elapsed time since start of simulation
* allows to simulate in real time, slow down or accelerate the simulation

#### Settings
* the acceleration factor (< 1 slows down, 1 is real time, >1 accelerates; dynamically reconfigurable)
* the time resolution with which the simulation time is published (dynamically reconfigurable)
* the time can be paused via `pause_time` in dynamic_reconfigure
* for a synchronized start of all objects set `<param name="/time_mgmt/pause_time" value="true" />` in the launchfile and uncheck it in dynamic_reconfigure
* see [cfg/TimeMgmt.mrtcfg](cfg/TimeMgmt.mrtcfg) for details

#### Usage
* launch the file `launch/simulation_management.launch`

# object initialization
Enabling the initialization of an object.

#### Working Principle
* reads a path, a hull and settings from a launchfile
* creates an `ObjectInitialization.msg` from this information and publishes it (for the localization management)

#### Settings
* object_id, initial (constant) velocity, path, frame in which the path is given, frame of the localization management, start point along path, hull, object type (car, ...), object role (operated agent, dynamic obstacle, ...), spawn time, navsatfix_topic for coordinate transformation, initialization topic
* see [launch/object_initialization.launch](launch/object_initialization.launch) and [scripts/object_initialization.py](scripts/object_initialization.py) for details
* see `simulation_initialization_ros_tool` for a sample configuration

#### Usage
* launch the file `launch/object_initialization.launch`

## Contributors
Pascal Böhmler, Viktoria Braun, Nick Engelhardt, Maximilian Naumann

## License
This package is distributed under the 3-Clause BSD License, see [LICENSE](LICENSE).
