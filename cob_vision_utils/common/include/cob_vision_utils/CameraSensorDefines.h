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
* Description: Defines for camera sensors.
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Author: Jan Fischer, email:jan.fischer@ipa.fhg.de
* Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
*
* Date of creation: Sept 2008
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
 

/// @file CameraSensorDefines.h
/// Defines for camera sensors.
/// @author Jan Fischer
/// @date June 2010.

#ifndef __IPA_CAMERASENSORDEFINES_H__
#define __IPA_CAMERASENSORDEFINES_H__

#ifdef __COB_ROS__
	#include <opencv/cv.h>
	#include <opencv/cvaux.h>
	#include <opencv/highgui.h>
#else
	#include <cv.h>
	#include <cvaux.h>
	#include <highgui.h>
#endif

#include <boost/shared_ptr.hpp>
#include <boost/progress.hpp>
#include <boost/timer.hpp>

#include <string>
#include <iostream>
#include <iomanip>
#include <limits.h>
#include <vector>

namespace ipa_CameraSensors {

#if defined _MSC_VER && _MSC_VER >= 1200
    // disable warnings related to inline functions
    #pragma warning( disable: 4251 4275)
#endif

/// Define, if we need to import or export the libraries
#ifdef __LINUX__
	#define __DLL_LIBCAMERASENSORS__ 
	#define APIENTRY
#else
	#ifdef __LIBCAMERASENSORS_EXPORT__
		#define __DLL_LIBCAMERASENSORS__ __declspec(dllexport)
	#else
		#define __DLL_LIBCAMERASENSORS__ __declspec(dllimport)
	#endif
#endif

} // namespace ipa_CameraSensors

#endif // __IPA_CAMERASENSORDEFINES_H__
