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
* ROS stack name: cob3_driver
* ROS package name: cob_camera_sensors
* Description: Basic global defines have to be put in this file within
* the namespace ipa.utils. If they do not fit within this file, consider
* integrating your defines within a class.
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
 

/// @file GlobalDefines.h
/// Basic global defines have to be put in this file within the namespace ipa.utils.
/// If they do not fit within this file, consider integrating your defines within a class.
/// This file has been written by Jan Fischer in 2008.
/// Latest updates: November 2008.

#ifndef __GLOBALDEFINES_H__
#define __GLOBALDEFINES_H__

namespace ipa_Utils {
	
/// An enum for the return values.
/// This enum describes possible return values that are used to return failure or success.
enum {
	RET_OK =									0x00000001UL, ///< Everythings OK.
	RET_FAILED =								0x00000002UL  ///< Something went wrong.
};

/// A definition for PI.
static const double THE_PI_DEF = 3.141593;
/// A definition for Epsilon (very small value).
static const double THE_EPS_DEF = 0.000001;


} // end namespace ipa_Utils

#endif // __GLOBALDEFINES_H__

