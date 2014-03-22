^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_srvs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.2 (2014-03-20)
------------------
* update CMakeLists
* Contributors: ipa-fxm

0.5.1 (2013-08-16 01:14:35 -0700)
---------------------------------

0.5.0 (2013-08-16 01:14:35 -0700)
---------------------------------
* small change for compiling, descriptions need modifications for gazebo
* added msg generators
* Catkin for cob_common
* Deleted __init__ files
* removed files that should not be in version control
* removed binary file from version control
* Groovy migration
* Revert "removed old cob_utilities files"
  This reverts commit 45e743a0d8d25c3b3ec7d77c73c248949cfb0a51.
* removed old cob_utilities files
* extended GetPoseStampedTransformed.srv
* added dependency on geometry_msgs
* moved GetPoseStampedTransformed.srv to cob_srvs
* merge with ipa320
* removed obsolete dependency
* before merge with 320
* cleanup dependencies
* removed deprecated services
* moved msgs and srvs to according packages in cob3_intern
* deleted object detection services
* Extended Acquire Object image service call to provide transformations/frames of object views
* new service
* merge with ipa320
* moved GetJointState message ro base_drive_chain
* JSF: Added new messages
* moved init test to cob_srvs
* updated service attributes
* new msg/srv definitions for sensor fusion
* use std_msgs/Header instead of Header in cob_msg and cob_srvs due to upcoming deprecation
* added new service
* new services for cob_full_ik_solver
* updated service definition
* added missing dependency
* new service for grasping
* fixed message names and types
* new services for cob_prmce_planner
* fixed message names and types
* preparing rostest
* cleanup in simulation and common
* new services
* update documentation
* service for cartesian movement
* JSF: Integrated image acquisition method for all cameras to calibrate
* Renamed and worked on cob_drive_identification, moved Elmo Recorder services to cob_srvs
* new service SetMaxVel
* added image service to tof node
* clean up in cob_common stack
* new services
* service for env model point cloud
* service for moving the neck
* removed bug
* JSF: Adapted service parameters
* JSF: Added service files for object training
* JSF: Added service files for object training
* msg and srv for getting camera to base transformation
* JSF: Added service and message for object recognition
* new service for Camera2Base transformation
* goa: added service for 2D platform position
* GOA: renamed message in GetColoredPointCloud service
* GOA: changed point cloud service
* build packages again wit cob* instead of cob3*
* renamed to general cob packages
* Contributors: COB3-Manipulation, Georg, Jan Fischer, Mathias Lüdtke, Richard Bormann, abubeck, cob, goa, ipa-cpc, ipa-fmw, ipa-fxm, ipa-goa, ipa-jsf, ipa-nhg, ipa-uhr-fm
