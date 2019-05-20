^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package raw_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.12 (2019-05-20)
-------------------

0.6.11 (2019-04-05)
-------------------

0.6.10 (2019-03-14)
-------------------

0.6.9 (2018-07-21)
------------------
* fix syntax
* fix torso urdf for raw3-1 (`#241 <https://github.com/ipa320/cob_common/issues/241>`_)
  * updated raw3-1 configuration
  * Changed back to collision box
  * Using inertia macro
  * Use geometry macro
* Contributors: Felix Messmer, Richard Bormann

0.6.8 (2018-01-07)
------------------
* Merge pull request `#246 <https://github.com/ipa320/cob_common/issues/246>`_ from ipa320/indigo_release_candidate
  Indigo release candidate
* Merge pull request `#239 <https://github.com/ipa320/cob_common/issues/239>`_ from ipa-fxm/indigo_dev_rmb
  corrected torso definition
* corrected torso definition
* Merge pull request `#232 <https://github.com/ipa320/cob_common/issues/232>`_ from bmagyar/simplify_raw_vacuum_gripper_collision_model
  Replace vacuum gripper collision mesh with bounding box
* Replace vacuum gripper collision mesh with bounding box
* Merge pull request `#230 <https://github.com/ipa320/cob_common/issues/230>`_ from ipa-fxm/update_maintainer
  update maintainer
* Merge pull request `#231 <https://github.com/ipa320/cob_common/issues/231>`_ from ipa-fxm/update_torso_raw3-1
  new torso raw3-1
* add new torso for raw3-1
* add powerball urdf
* update maintainer
* Merge pull request `#224 <https://github.com/ipa320/cob_common/issues/224>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* Merge pull request `#229 <https://github.com/ipa320/cob_common/issues/229>`_ from ipa-fxm/move_ur_arm
  move ur_arm to raw_description
* move ur_arm to raw_description
* use license apache 2.0
* Contributors: Bence Magyar, Felix Messmer, Richard Bormann, ipa-fxm, ipa-uhr-mk

0.6.7 (2017-07-17)
------------------
* Current config gripper (`#221 <https://github.com/ipa320/cob_common/issues/221>`_)
  * added vacuum gripper
  * New STL file for the vacuum gripper
  * Vacuum Gripper file description updated
  * torso and gripper updates
  * removed obsolete files
  * orient_gripper
* fix the wheel radius to meet with the actual hardware radii
* separate laser scanner from base
* finalizing
* changes acoording to pull request
* origin of the colision mesh corrected according with visual values
* origin of the colision mesh corrected
* gripper mesh simplified
* stable behavior by tweaking the base mass/inertia achieved
* stable behavior by tweaking the base mass/inertia achieved
* get a stable behavior by tweaking the base mass/inertia
* use scalable primitive meshes
* use the properties at the top for the collision properties
* file name and suffix all small letters.
* gripper macro name changed and prefix removed as argument
* tabs vs spaces solved
* requested changes
* torso and gripper updates
* Vacuum Gripper file description updated
* New STL file for the vacuum gripper
* added vacuum gripper
* move gazebo_ros_control plugin
* use latest xacro syntax
* manually fix changelog
* Contributors: Bruno Brito, Richard Bormann, ipa-bfb-sc, ipa-fxm, ipa-mjp, ipa-raw3-3

0.6.6 (2016-10-10)
------------------
* fixed inertia and mass for raw3 base long
* Contributors: Benjamin Maidel

0.6.5 (2016-04-01)
------------------
* restructure simulated lasers and laser topic names
* fixed copy paste error for base_short laser mounting position
* Contributors: Benjamin Maidel, ipa-fxm

0.6.4 (2015-08-29)
------------------
* fix typo in collision mesh file name
* add explicit exec_depend to xacro
* fix catkin_minimum_required version
* remove trailing whitespaces
* remove trailing whitespaces
* migrate to package format 2
* sort dependencies
* critically review dependencies
* Contributors: ipa-fxm

0.6.3 (2015-06-17)
------------------
* allow laser calibration
* remove unsupported calibration_rising
* separate xacro macro for drive_wheel module used in all bases + significant simplification
* use PositionJointInterface
* Contributors: ipa-fxm

0.6.2 (2014-12-15)
------------------
* use VelocityJointInterface hardware interfaces for simulation of all bases
* Contributors: ipa-fxm

0.6.1 (2014-09-24)
------------------
* 1=true
* fix bumper plugins
* Contributors: ipa-fxm

0.6.0 (2014-09-16)
------------------

0.5.5 (2014-08-27)
------------------

0.5.4 (2014-08-25)
------------------
* update changelog
* consistency changes due to latest gazebo tag format
* consitency changes due to new transmission format
* unify materials
* include gazebo_ros dependendy to export materials
* merge with hydro_dev
* cleanup dependencies
* beautify indentation + cleaning up
* better approximation of inertias
* Merge pull request `#112 <https://github.com/ipa320/cob_common/issues/112>`_ from ipa-cob4-1/hydro_dev
  Rotated sick_s300 mesh file
* use the  macros instead 3.14...
* Merge github.com:ipa-cob4-1/cob_common into hydro_dev
* switch laser scanner orientation
* removed bumpers and changed transmission config to new syntax
* no inertia in base_footprint
* use collada material description
* remove material physic properties of wheels to use default, fixes `#90 <https://github.com/ipa320/cob_common/issues/90>`_
* Contributors: Alexander Bubeck, Felix Messmer, Florian Weisshardt, ipa-bnm, ipa-cob4-1, ipa-fxm, ipa-nhg

0.5.3 (2014-03-31)
------------------

0.5.2 (2014-03-20)
------------------
* merge with groovy_dev_cob4
* fixed gazebo_plugins
* fixed path to file
* update transmission for all components
* update xmlns + beautifying
* fix xacro include tag deprecation
* Merge pull request `#7 <https://github.com/ipa320/cob_common/issues/7>`_ from ipa-fxm/groovy_dev
  bring groovy updates to hydro
* harmonize with cob structure
* upstream changes
* fixing simulation for hydro. Still wip
* Solved xacro Warning in hydro.
* also add urdf include for tf
* small changes for new camera setup
* changes for hydro gazebo, still not fully working
* changed wheel positions to make rotation right, also changed some bugs in the asymetric tower
* changed mesh origin to the center of the base plate
* deleted ur10 description
* Contributors: Denis Štogl, abubeck, ipa-bnm, ipa-fxm, ipa-nhg, raw3-1 administrator

0.5.1 (2013-08-16 01:14:35 -0700)
---------------------------------

0.5.0 (2013-08-16 01:14:35 -0700)
---------------------------------
* added installer stuff
* fixed bug after merging
* merged with upstream changes
* removed generation of mesh files
* Merge pull request `#41 <https://github.com/ipa320/cob_common/issues/41>`_ from ipa-fxm/mesh_gen_fix
  remove mesh file generation from description packages - they are not nee...
* cleanup deps
* Catkin for cob_common
* remove mesh file generation from description packages - they are not needed any longer
* new files for adding universal arms with origin parameter
* new gazebo sensor structure
* merge
* moved tower meshes
* changed path to tower meshes
* added materials
* raw tower descriptions
* added new description for the short raw base
* added new base description for the longer raw base
* remove deprecated ur10 description and meshes from cob_common
* mainly beautifying
* clean up gazebo files
* go back to using mesh for collision instead of big box - box makes robot not movable within gazebo
* re-add kinect to raw-torso
* major adaptions in gazebo.urdf.xacros according to new gazebo format for controllers
* Revert "increase size of boxgripper"
  This reverts commit 2b97071804a7627ca8a41079fbe35cf5c01dc57b.
* increase size of boxgripper
* use boxgripper mesh in urdf
* new mesh for boxgripper
* modified boxgripper
* fixed urdf
* urdf fix
* raw description and meshes for short raw
* adjusted boxgripper collision geometry
* simpler collision geometries
* no stereo cameras attached to raw3-1
* fixed box_gripper position
* added ur10 in raw3-1 description
* Groovy migration
* merge
* Deleted texture colors
* Renamed colors
* fix color
* fix colors and powerball tray
* raw torso calibration
* modified raw3-1 urdf description
* added amadeus boxgripper description for raw3-1
* flipped front to back like on real robot
* fixed typo
* changed names from cob to raw and adapted gazebo and transmission files
* changed limit of torso tilt
* changed torso back to working version from robot, renamed joints
* removed old arm_ur files
* removed old arm_ur meshes
* adapted raw_torso files
* final raw-model V2
* use stl
* new files for raw_description, some fixes
* fixed: all stl file shouldn't start with the word 'solid'. Replace 'solid' with 'robot', see http://ros.org/wiki/cob_description
* merge
* final raw-model
* changed stl files not using solid
* changed kinect configuration for fuerte, changed stlb links to stl
* Merge branch 'review-abubeck'
* finished raw3-1 model --- V1
* small urdf bugfix
* remove swp file
* Merge branch 'master' of github.com:ipa320/cob_common
* deleted swap file
* changes for raw
* delete obsolete files
* added new stls for raw base
* moved sick_s300 stl to cob_description
* added stls and adopted model due to CAD data for raw3-1
* added torso
* substitute 1.57 3.14 6.28 through M_PI
* changed direction of urdf model to new convention
* changed rotation of laser scanner to work on real robot
* renamed icob to raw and merged and cleaned up lots of things
* Contributors: Alexander Bubeck, Florian Weißhardt, Lucian Cucu, abubeck, ipa-bnm, ipa-fmw, ipa-fxm, ipa-nhg, robot
