/****************************************************************
*
* Copyright (c) 2010
*
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Project name: care-o-bot
* ROS stack name: cob_common
* ROS package name: cob_vision_utils
* Description: Basic math utilities
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Author: Jan Fischer, email:jan.fischer@ipa.fhg.de
* Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
*
* Date of creation: Nov 2008
* ToDo:
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/
 
/// @file MathUtils.h
/// Basic math utilities
/// This file has been written by Jan Fischer in 2008.
/// Latest updates: November 2008.

#ifndef __IPA_MATHUTILS_H__
#define __IPA_MATHUTILS_H__

#ifdef __COB_ROS__
	#include "cob_vision_utils/GlobalDefines.h"

	#include <opencv/cv.h>
	#include <libwm4/Wm4Quaternion.h>
	#include <libwm4/Wm4Vector3.h>
	#include <libwm4/Wm4Math.h>
#else
	#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/GlobalDefines.h"

	#include <cv.h>
	#include <Wm4Quaternion.h>
	#include <Wm4Vector3.h>
	#include <Wm4Math.h>
#endif

namespace ipa_Utils {
	
/// Abbreviation for quaternions. Helps also to easily change
/// from WM4 to WM5 namescpace
typedef Wm4::Quaterniond Quaterniond;
typedef Wm4::Vector3d Vector3d;
typedef cv::Vec<double, 7> Vec7d;

/// Point cloud abbreviation
typedef std::vector<ipa_Utils::Vector3d> Pointcloud;

/// A definition for PI.
static const double THE_PI_DEF = Wm4::Mathd::PI;
/// A definition for Epsilon
static const double THE_EPS_DEF = Wm4::Mathd::EPSILON;

/// Apply translation first, then rotate.
/// @param pt The point that is transformed
/// @param t xyz coordinates of translation
/// @param q rotation quaternion
/// @return Return code
unsigned long TranslateRotate(ipa_Utils::Vector3d& pt, 
	const ipa_Utils::Vector3d& t, const ipa_Utils::Quaterniond& q);

/// Apply translation first, then rotate.
/// @param pt The point that is transformed
/// @param transformation Transformation[0]..[2] are xyz coordinates of translation, transformation[3]..[6] specify the quaternion
/// @return Return code
unsigned long TranslateRotate(ipa_Utils::Vector3d& pt, const ipa_Utils::Vec7d& transformation);

/// Apply rotation first, then translate.
/// Former ToWorld with pos angle.
/// @param pt The point that is transformed
/// @param t xyz coordinates of translation
/// @param q rotation quaternion
/// @return Return code
unsigned long RotateTranslate(ipa_Utils::Vector3d& pt, 
	const ipa_Utils::Vector3d& t, const ipa_Utils::Quaterniond& q);

/// Apply rotation first, then translate.
/// Former ToWorld.
/// @param pt The point that is transformed
/// @param transformation Transformation[0]..[2] are xyz coordinates of translation, transformation[3]..[6] specify the quaternion
/// @return Return code
unsigned long RotateTranslate(ipa_Utils::Vector3d& pt, const ipa_Utils::Vec7d& transformation);

/// Converts cartesian coordinates to spherical coordinates.
/// @param pt Point, that is converted
/// @return Return code
unsigned long ToSphere(ipa_Utils::Vector3d& pt);

/// Converts spherical coordinates to cartesian coordinates.
/// @param pt_cartesian Point, that is converted
/// @return Return code
unsigned long ToCartesian(ipa_Utils::Vector3d& pt);

} // end namespace ipa_Utils

#endif // __IPA_MATHUTILS_H__

