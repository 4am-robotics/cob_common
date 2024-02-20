^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.7.10 (2024-02-20)
-------------------

2.7.9 (2023-11-06)
------------------
* Merge pull request `#303 <https://github.com/ipa320/cob_common/issues/303>`_ from omar-ihab-ali/update/emergency_stop_state_msg_foxy
  update EmergencyStopState Msg to include other states sent by PLC
* update implementation for EmergencyStopState Msg to include other states sent by PLC
* Contributors: Benjamin Maidel, omar-ihab-ali

2.7.8 (2022-11-18)
------------------
* Merge pull request `#299 <https://github.com/ipa320/cob_common/issues/299>`_ from fmessmer/foxy_port_ros2
  [foxy] port packages
* harmonize packages
* port cob_msgs
* add COLCON_IGNORE
* Contributors: Felix Messmer, fmessmer

0.7.8 (2022-07-29)
------------------
* Merge pull request `#297 <https://github.com/ipa320/cob_common/issues/297>`_ from floweisshardt/feature/power_state_connected
  add explicit power_state.connected
* add explicit power_state.connected
* Contributors: Felix Messmer, floweisshardt

0.7.7 (2021-12-23)
------------------

0.7.6 (2021-08-02)
------------------
* Merge pull request `#292 <https://github.com/ipa320/cob_common/issues/292>`_ from benmaidel/feature/add_ack_needed
  add ack_needed to SafetyControllerState
* add ack_needed to SafetyControllerState
* Merge pull request `#291 <https://github.com/ipa320/cob_common/issues/291>`_ from benmaidel/feature/safety_controller_state
  add SafetyControllerState message
* add SafetyControllerState message
* Contributors: Benjamin Maidel, Felix Messmer

0.7.5 (2021-07-01)
------------------

0.7.4 (2020-11-20)
------------------

0.7.3 (2020-09-26)
------------------
* Merge pull request `#288 <https://github.com/ipa320/cob_common/issues/288>`_ from fmessmer/test_noetic
  test noetic
* Bump CMake version to avoid CMP0048 warning
* Contributors: Felix Messmer, fmessmer

0.7.2 (2020-03-18)
------------------

0.7.1 (2019-11-07)
------------------

0.7.0 (2019-08-06)
------------------

0.6.12 (2019-05-20)
-------------------

0.6.11 (2019-04-05)
-------------------

0.6.10 (2019-03-14)
-------------------

0.6.9 (2018-07-21)
------------------
* update maintainer
* Contributors: fmessmer

0.6.8 (2018-01-07)
------------------
* Merge pull request `#246 <https://github.com/ipa320/cob_common/issues/246>`_ from ipa320/indigo_release_candidate
  Indigo release candidate
* Merge pull request `#230 <https://github.com/ipa320/cob_common/issues/230>`_ from ipa-fxm/update_maintainer
  update maintainer
* update maintainer
* Merge pull request `#224 <https://github.com/ipa320/cob_common/issues/224>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* use license apache 2.0
* Contributors: Felix Messmer, ipa-fxm, ipa-uhr-mk

0.6.7 (2017-07-17)
------------------
* manually fix changelog
* Contributors: ipa-fxm

0.6.6 (2016-10-10)
------------------

0.6.5 (2016-04-01)
------------------
* Update package.xml
* cleanup dashboard message
* rework messages
* remove unused messages
* Contributors: Florian Weisshardt, ipa-fmw

0.6.4 (2015-08-29)
------------------
* remove trailing whitespaces
* remove trailing whitespaces
* migrate to package format 2
* sort dependencies
* critically review dependencies
* Contributors: ipa-fxm

0.6.3 (2015-06-17)
------------------

0.6.2 (2014-12-15)
------------------
* add new msgs
* Contributors: ipa-nhg

0.6.1 (2014-09-24)
------------------

0.6.0 (2014-09-16)
------------------

0.5.5 (2014-08-27)
------------------
* catkin_lint'ing
* move EmergencyStopState.msg to cob_msgs
* Contributors: ipa-fxm

0.5.4 (2014-08-25)
------------------
* add maintainer
* update changelog
* introducing cob_msgs package in order to replace pr2_msgs
* Contributors: Felix Messmer, Florian Weisshardt

0.5.3 (2014-03-31)
------------------

0.5.2 (2014-03-27)
------------------

0.5.1 (2013-08-16 01:14:35 -0700)
---------------------------------

0.5.0 (2013-08-16 01:14:35 -0700)
---------------------------------
* merge with ipa320
* delete cob_msgs
* moved messages
* moved msgs and srvs to according packages in cob3_intern
* merge with ipa320
* dep to actionlib_msgs
* update stacks
* moved ultiple message files out of cob_msgs to their own packages
* moved light message to cob_light
* JSF: Added new messages
* update msg description
* new msg/srv definitions for sensor fusion
* use std_msgs/Header instead of Header in cob_msg and cob_srvs due to upcoming deprecation
* new pause script state
* new services for cob_full_ik_solver
* new Message for GraspPlanner
* cleanup in simulation and common
* grasp script optimisations
* testing cart interface
* bugfix
* restructured script_server, put more functionality to action handle
* live script_viewer is working
* defined script messages
* update documentation
* merge with cpc
* added Person-Msg Types
* Merge branch 'master' into cpc-fm
* addapted messages for person association
* added move action to cob_msgs
* changed actions
* changes on light controller
* new package for lights, not working yet
* clean up in cob_common stack
* Tactile sensors
* merge
* added TactileMatrix message, bugfix on grid view
* removed bug
* JointCommand action for sdh
* removed non ASCII character from Emergency Stop message
* modifications to cob_relayboard
* JSF: Adapted service parameters
* msg and srv for getting camera to base transformation
* JSF: Added service and message for object recognition
* Added EmergencyStop Message containing the current em signals as well as current state (e.g. confirmed after using the key-switch); accordingly adapted the relayboard-node to output the EMState together with EM signals; Last but not least: Fixed a typing error in the platform node
* adapt launch file to new packages names
* new action package
* renamed to general cob packages
* Contributors: Alexander Bubeck, Christian Connette, FM, Georg, Jan Fischer, Winfried Baum, b-it-bots, cob, fmw, ipa-fmw, ipa-goa, ipa-jsf, ipa-uhr-fm
