^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.2 (2014-03-20)
------------------
* merged with ipa320
* removed Media folder
* merge with groovy_dev
* fix kinect topics for simulation
* fixes while testing in simulation
* update xacro file format
* merge with groovy_dev
* new structure
* fixed some includes and property definitions
* some missed changes
* merge with groovy_dev_cob4
* fixed gazebo_plugins
* added arm_ee_link
* fixed path to file
* fixed path to file
* renamed tray 3DOF
* Tested on simulation
* cob_description structuration
* cleanup
* update cob4 description
* renamed files
* New struture for cob repositories
* tested on robot
* cob4 integration
* cob4 integration
* bring groovy updates to hydro
* Adapt tray position
* Fixed tray powerball
* Adjust limits for tray and torso
* modify axis on mesh model
* some helper makros for default inertia
* optimize effort and joint limits + use visual mesh as collision for upper neck to give arem some more space
* visual and collision geometry of cameras are now not colliding with head_cover anymore
* update transmission for all components
* remove obsolete files
* use default settings
* update xmlns + beautifying
* fix xacro include tag deprecation
* Merge pull request `#7 <https://github.com/ipa320/cob_common/issues/7>`_ from ipa-fxm/groovy_dev
  bring groovy updates to hydro
* remove obsolete experimental files
* make lookat work with raw
* ur_connector meshes are now assimp conform
* fixed torso joint limits
* adjust limits for ur_connector
* latest changes in lookat component
* simplified lookat component
* new urdf description for lookat
* fixing simulation for hydro. Still wip
* unified torso frames
* unified head frames
* Revert "depth joint for kinect implemented"
  This reverts commit f3449462cd05a5efc8f47252e28366d6a495acb2.
* offset back in lbr.urdf.xacro else wrong calibration
* fixed typo
* Removed safety controller urdf/ur_connector/ur_connector.urdf.xacro
* Renamed ur_connector
* New model descriptions for cob3-7
* offset for lbr set to 0
* Solved xacro Warning in hydro.
* Fixed type error
* changes for hydro gazebo, still not fully working
* depth joint for kinect implemented
* new component base_placement for whole body moveit group
* added fixed links for calibration
* new urdf description for lookat
* Contributors: Alexander Bubeck, Denis Štogl, Jannik, Jannik Abbenseth, abubeck, ipa-cob3-5, ipa-cob3-7, ipa-fmw, ipa-fxm, ipa-nhg

0.5.1 (2013-08-16 01:14:35 -0700)
---------------------------------

0.5.0 (2013-08-16 01:14:35 -0700)
---------------------------------
* added installer stuff
* fixed bug after merging
* merged with upstream changes
* removed generation of mesh files
* changed target name to be specific
* Merge pull request `#41 <https://github.com/ipa320/cob_common/issues/41>`_ from ipa-fxm/mesh_gen_fix
  remove mesh file generation from description packages - they are not nee...
* cleanup deps
* cleanup deps
* name failed test files for urdf check
* adapt urdf_check for groovy
* fix kinect FoV
* set update rate to 20hz again
* Catkin for cob_common
* remove mesh file generation from description packages - they are not needed any longer
* fix meshes and transformation for tray_powerball
* changed field of view of RGB image to be more realistic (from 57 to 62)
* moved all hardcoded offsets to calibration_data
* merge
* added colored collada model for sick s300 scanner
* use collision mesh again
* clean up gazebo files
* major adaptions in gazebo.urdf.xacros according to new gazebo format for sensors - simulated sensor data still not fully correct
* major adaptions in gazebo.urdf.xacros according to new gazebo format for sensors
* major adaptions in gazebo.urdf.xacros according to new gazebo format for controllers
* Merge pull request `#34 <https://github.com/ipa320/cob_common/issues/34>`_ from ipa-fmw/master
  extend urdf test
* extended urdf test
* added ur10 in raw3-1 description
* Redefined collisions in urdf files
* Groovy migration
* Merge branch 'master' of github.com:ipa320/cob_common
* adjust color settings
* rename topic from scan_top to scan_top_raw
* merge
* Deleted texture colors
* Renamed colors
* adjusted params for prosilica
* Merge pull request `#23 <https://github.com/ipa320/cob_common/issues/23>`_ from ipa-goa/master
  changed far clip to 100
* changed far clip to 100
* extended head cover and upper neck meshes
* increased torso_v0 limits for the initialization of cob3-1
* fix colors and powerball tray
* Renamed the colors
* Redefined Care-O-bot colors for Gazebo and Rviz
* Orange color for LBR
* Defined new colors
* Updated phiget sensors position
* Updated joints axis
* Removed stlb as collision mesh files, fuerte does not support this format
* Minor changes in tray_powerball description
* Description for tray_powerball
* Fuerte migration cob_descriptionurdf/base/base.gazebo.xacro
* removed unused reference position for lbr
* final raw-model V2
* update urdf
* Revert "replaced solid with robot in stl"
  This reverts commit 5a415bb7dc12831d2ed8932aa46b8cdcb044d300.
* fixed stl
* use stl
* replaced solid with robot in stl
* undo previous changes in cob_description/urdf/base/base.gazebo.xacro
* add simulated phidgets sensors to tray
* changed stl files not using solid
* Update desire_description
* fix naming for both kinect plugins
* fixed field of view for kinect
* Merge pull request `#12 <https://github.com/ipa320/cob_common/issues/12>`_ from abubeck/master
  fuerte support, compatible with electric
* Merge https://github.com/abubeck/cob_common
* Merge branch 'master' of github.com:abubeck/cob_common
* Merge branch 'electric' of github.com:ipa320/cob_common into release_electric
* changed kinect configuration for fuerte, changed stlb links to stl
* increased upper joint limit and velocity for head_v1
* fixed cam3d topic for head_v1
* finished raw3-1 model --- V1
* limit torso pan and tilt joints
* moved sick_s300 stl to cob_description
* added stl for laser scanner
* substitute 1.57 3.14 6.28 through M_PI
* additional links on tray
* read correct torso stl
* urdf structure change: tray can be calibrated now
* using calibration for laser scanners
* renamed icob to raw and merged and cleaned up lots of things
* Deleted old files and copies
* fix icob urdf
* torso urdf change: made torso middle link longer (as in cad)
* cameras have zero pos/rot offsets in head_v3
* calibrate cam3d to head axis instead of left camera
* setup cob3-4
* don't include urdf files from ros directory
* python urdf test
* merge with ipa320
* added minimum range for kinect
* ..
* add dep
* Merge branch 'master' of github.com:ipa-fmw/cob_common into review-ipa-fmw
* fix collision problem with floor: lift collision base_footprint
* fix names in base urdf
* renamed components
* renamed folders
* moved out of ros dir
* moved out of ros dir
* removed schunk components
* removed calibration for now missing calibration link
* fixed bug with xyz values
* removed calib_joint
* merged with goa
* revert urdf changes because of arm planning collisions
* new calibration for cob3-3 and cob3-4
* temporary fix for urdf collision model
* add configs for cob3-4
* beautify sdh transmissions
* adjust cob3-3 torso calibration
* using now kinect plugin from pr2_gazebo
* fixed origin offset
* Merge branch 'master' of github.com:ipa-goa-wt/cob_common into review-goa-wt
* urdf and default configs for cob3-bosch
* added rgb description for kinect
* added sdh_tip link
* new torso calibration
* merge
* Updated calibration for Kinect sensor
* merge
* added comment
* bumpers measure in the coordinate system of the fingers
* Kinect rgb configuration
* Merge branch 'master' of github.com:ipa-goa/cob_common
* neck calib
* added helper coordinate system for calibration, added calibration values
* Merge branch 'master' of github.com:ipa-fmw/cob_common
* new calibration offset for tray
* Updated camera calibration for cob3-3
* commit from icob
* added urdf for standard schunk lwa3
* merge
* fix head_v3 simulation error
* modifications for fetch and carry
* Merge branch 'master' of github.com:ipa-rmb/cob_common into review-rmb
* update cob3-3
* Merge branch 'master' of github.com:ipa-fmw/cob_common into review-fmw
* fix head orientation for cob3-3
* fix head orientation for cob3-3
* merge
* Merge branch 'master' of github.com:ipa-fmw/cob_common into review-fmw
* fix cob3-3 tf
* calibration for cob3-1
* new arm configurations for faster table manipulation
* head urdf for cob3-1
* changes from b-it-bots
* calib test
* calib test
* Merge branch 'master' of github.com:ipa-taj/cob_common
* corrected calib values
* added calib values for cam to neck
* merge
* Left tp right camera change in urdf
* cob_head_axis set
* corrected the swissranger topics to the unified naming scheme
* cleanup cob3-2 description
* calibration for cob3-3 tray
* fix urdf of cob3-3
* merge
* left camea is now reference camera
* merge
* merge
* update for cob3-3
* Merge branch 'master' of github.com:ipa-fmw/cob_common
* alltest launch file
* torso_v1 added
* update torso for cob3-3
* mimic joint for sdh
* update head description with general tof
* small modification for dashboard
* Fix CRLF
* kinect sensor added
* kinect sensor added
* fix names for multiple tof sensors
* changes in tof.gazebo.xacro
* inserted new urdf files for cob3-3, need to be adapted
* merge
* changed base configuration for cob3-2
* fixed voxelization + now including sdh
* new files for prmce voxelization
* urdf model for voxelization
* merge with ipa320
* update cob3-2 arm
* changed the platform urdf to version 1
* arm planning
* beautifying
* single arm and arm with sdh simulation running
* modifications sensor fusion
* Merge branch 'master' of github.com:ipa-jsf/cob_common into review-jsf
* adjust camrea simulation parameters to real cameras
* renamed cameraone to prosilica
* fixed camera topics for simulation
* reduced mass for simulation
* tuned gazebo controller
* fix safety controller in lbr
* simulation working again after merging
* use stlb files in collision now
* generate stlb files
* included calls to base_v1, but still base_v0 is active
* fixed laser sensor names, version number and visual model
* modified base_collision_model
* Merge branch 'review-320'
* removed falling calibration
* Merge branch 'review-brudder'
* Merge branch 'master' of https://github.com/brudder/cob_common into review-brudder
* fixed error in lbr
* Merge branch 'review-brudder'
* Merge branch 'review-320'
* Merge branch 'master' of https://github.com/ipa320/cob_common into review-320
* added correct calibration
* Merge branch 'master' of github.com:ipa-goa/cob_common into review-goa
* modified base collision model for 2dnav_ipa
* new stl models for collision added and implemented
* update configurations and added grassp link to sdh
* Merge branch 'master' of https://github.com/ipa320/cob_common into review-320
* corrected axes and wheel hubs
* desire robot added
* restructure urdf files and launch files for simulation
* changed urdf files for single components
* changed launch file structure for bringup
* added safety_controller for pr2_kinematics
* simple base collision model
* added swissranger in simulation
* corrected calibration
* cleanup in simulation and common
* added hand-eye-calibration values
* Merge branch 'master' of github.com:ipa-goa/care-o-bot
* added camera calibration
* Head axis working, tested on cob3-1 but adapted parameters (-files)  should work on both robots
* added sick scanner to urdf
* added real scan values to simulation, added scan filters to simulation
* changed mesh files for new transformations
* added hokuyo support to nav
* Merge branch 'review-320'
* added calibration for right camera
* HeadAxis working
* new torso tranfsormation
* update joint limits for lbr
* cob_base
* moved ekf domo publisher to nav; update positions for new urdf trafos; moved controller_manager to cob_controller_configuration_gazebo
* fix for global frame names
* lbr working on cob
* cob_head_axis working
* inserted cob base mesh file
* first version of cob_base urdf
* new trnasformation for base lbr
* new arm transformation for lbr, set_operation_mode with service interface
* tactile sensors in simulation
* cleanup in urdfs
* beautify torso urdf
* changed dimensions of cameras
* preparation for blocklaser
* simulated cameras working
* head axis working in simulation
* removed executable status from files
* preparations for cameras and tof in simulation
* grasp script optimisations
* update urdf to be compatible with ctrutle, add 64bit support for libntcan
* changed transmission and filters to namespaces
* update documentation
* optimized controllers for simulation
* changed angle offset after calibration
* fixed bug with fixed joint
* fixed bug with fixed joint
* changed transformation based on box-style-calibration
* modified urdf and adapted xaml files
* improved simulation for schunk arm and cleanup in 2dnav package
* altered sdh mounting for changed lbr naming
* fixed problem with lbr urdf files occuring on cob3-lbr robot
* update on robot
* grasp from cooler scenarion running
* update for cob3-2
* update script server yaml and lbr urdf description
* dual arm cob3 simulation and modified controllers for schunk simulation
* extended calibration files for camera calibration
* Merge branch 'master' of github.com:abubeck/care-o-bot
* dual arm setup
* modified camera coordinate systems
* added virtual camera support
* updated lbr description, is now correct
* improvements of lbr simulation
* added lbr to simulation
* lbr meshes and simulation
* renamed laser topics
* modified urdf to work with hokuyo simulation
* modified urdf and changes to sdh driver
* changed from cob3-1 to cob3-sim
* small fixes for simulation
* updated simulation files
* clean up in cob_common stack
* added upload file for cob3-1
* changes on powercube chain to accept direct command without actionlib
* missing files for simulation
* new files for navigation, e.g. maps and launch files
* merge
* arm is now on foot block
* arm is now on foot block
* extended limits of joint 1
* rotated arm meshes and tray mesh
* calibration file for sim
* urdf file for cob3-sim
* missing upload file
* new simulation interfaces
* small fix
* separate urdf files for arm and sdh
* separate urdf files for arm and sdh
* missing stl files
* upload files for simulation
* merge
* merge
* big changes to simulation structure
* changed stl files
* modified knoeppkes
* new stl file for tray
* adaptions to urdf for tray
* new stl file for tray
* changed origin of head_cover
* new launch file for cob3-sim
* added sdh controller file
* Merge branch 'fmw-hj'
* modified urdf to have less shaking
* renamed cob launch file
* modified urdf
* inserted new stl files
* new stl file for head cover
* new stl files for torso
* added sdh urdf files
* included calibration files
* modified manifests for documentation
* mesh files for lwa
* included arm
* stl files for base
* missing SR400 files
* missing camera files
* new files for cob_description
* merge
* new urdf desciption
* modifications for cob3-2
* new urdf structure for platform and torso
* Contributors: Alexander Bubeck, COB3-Manipulation, Florian Weißhardt, Georg Arbeiter, Lucian Cucu, Mathias Lüdtke, Richard Bormann, Sven Schneider, abubeck, b-it-bots-secure, brudder, cob, cob3-1-pc1, cpc-pk, fmw-jk, ipa, ipa-bnm, ipa-fmw, ipa-fmw-sh, ipa-fxm, ipa-goa, ipa-goa-wt, ipa-jsf, ipa-mig, ipa-nhg, ipa-rmb, ipa-taj, ipa-taj-dm, ipa-uhr, ipa-uhr-fm, mxcreator, nhg-ipa, robot, root
