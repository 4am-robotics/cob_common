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
* Description:
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Author: Jan Fischer, email:jan.fischer@ipa.fhg.de
* Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
*
* Date of creation: Mai 2008
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

#ifdef __COB_ROS__
#include "cob_vision_utils/CameraSensorToolbox.h"
#else
#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/CameraSensorToolbox.h"
#endif

using namespace ipa_CameraSensors;

__DLL_LIBCAMERASENSORS__ CameraSensorToolboxPtr ipa_CameraSensors::CreateCameraSensorToolbox()
{
	return CameraSensorToolboxPtr(new CameraSensorToolbox());
}

CameraSensorToolbox::CameraSensorToolbox()
{
	m_Initialized = false;
}

CameraSensorToolbox::~CameraSensorToolbox()
{
	Release();
}

unsigned long CameraSensorToolbox::Release()
{
	std::map<std::string, CvMat*>::iterator matrixIterator;
	std::map<std::string, IplImage*>::iterator iplImageIterator;

	m_intrinsicMatrices.clear();
	m_distortionCoeffs.clear();
	m_undistortMapsX.clear();
	m_undistortMapsY.clear();
	m_extrinsicMatrices.clear();
}

CameraSensorToolbox::CameraSensorToolbox(const CameraSensorToolbox& cst)
{
	Release();

	std::map<std::string, cv::Mat>::const_iterator matrixIterator;

	// Clone intrinisc matrices
	for ( matrixIterator=cst.m_intrinsicMatrices.begin() ; matrixIterator != cst.m_intrinsicMatrices.end(); matrixIterator++ )
	{
		m_intrinsicMatrices[matrixIterator->first] = matrixIterator->second.clone();
	}

	// Clone distortion parameters
	for ( matrixIterator=cst.m_distortionCoeffs.begin() ; matrixIterator != cst.m_distortionCoeffs.end(); matrixIterator++ )
	{
		m_distortionCoeffs[matrixIterator->first] = matrixIterator->second.clone();
	}

	// Clone undistortion map X
	for ( matrixIterator=cst.m_undistortMapsX.begin() ; matrixIterator != cst.m_undistortMapsX.end(); matrixIterator++ )
	{
		m_undistortMapsX[matrixIterator->first] = matrixIterator->second.clone();
	}

	// Clone undistortion map Y
	for ( matrixIterator=cst.m_undistortMapsY.begin() ; matrixIterator != cst.m_undistortMapsY.end(); matrixIterator++ )
	{
		m_undistortMapsY[matrixIterator->first] = matrixIterator->second.clone();
	}

	// Clone extrinsic matrix
	for ( matrixIterator=cst.m_extrinsicMatrices.begin() ; matrixIterator != cst.m_extrinsicMatrices.end(); matrixIterator++ )
	{
		m_extrinsicMatrices[matrixIterator->first] = matrixIterator->second.clone();
	}

	m_Initialized = cst.m_Initialized;
}

CameraSensorToolbox& CameraSensorToolbox::operator=(const CameraSensorToolbox& cst)
{
	// Check for self-assignment
	if (this==&cst)
	{
		 return *this;
	}

	Release();

	std::map<std::string, cv::Mat>::const_iterator matrixIterator;

	// Clone intrinisc matrices
	for ( matrixIterator=cst.m_intrinsicMatrices.begin() ; matrixIterator != cst.m_intrinsicMatrices.end(); matrixIterator++ )
	{
		m_intrinsicMatrices[matrixIterator->first] = matrixIterator->second.clone();
	}

	// Clone distortion parameters
	for ( matrixIterator=cst.m_distortionCoeffs.begin() ; matrixIterator != cst.m_distortionCoeffs.end(); matrixIterator++ )
	{
		m_distortionCoeffs[matrixIterator->first] = matrixIterator->second.clone();
	}

	// Clone undistortion map X
	for ( matrixIterator=cst.m_undistortMapsX.begin() ; matrixIterator != cst.m_undistortMapsX.end(); matrixIterator++ )
	{
		m_undistortMapsX[matrixIterator->first] = matrixIterator->second.clone();
	}

	// Clone undistortion map Y
	for ( matrixIterator=cst.m_undistortMapsY.begin() ; matrixIterator != cst.m_undistortMapsY.end(); matrixIterator++ )
	{
		m_undistortMapsY[matrixIterator->first] = matrixIterator->second.clone();
	}

	// Clone extrinsic matrix
	for ( matrixIterator=cst.m_extrinsicMatrices.begin() ; matrixIterator != cst.m_extrinsicMatrices.end(); matrixIterator++ )
	{
		m_extrinsicMatrices[matrixIterator->first] = matrixIterator->second.clone();
	}

	m_Initialized = cst.m_Initialized;

	return *this;
}

unsigned long CameraSensorToolbox::Init(std::string directory, ipa_CameraSensors::t_cameraType cameraType,
										int cameraIndex, const CvSize imageSize)
{
	Release();

	m_ImageSize = imageSize;

	std::string iniFileNameAndPath = directory;
	iniFileNameAndPath += "cameraSensorsIni.xml";
	if (LoadParameters(iniFileNameAndPath.c_str(), cameraType, cameraIndex) & RET_FAILED)
	{
		return (RET_FAILED | RET_INIT_CAMERA_FAILED);
	}

	m_Initialized = true;
	return RET_OK;
}

unsigned long CameraSensorToolbox::Init(const std::map<std::string, cv::Mat>* intrinsicMatrices,
										const std::map<std::string, cv::Mat>* distortionParameters,
										const std::map<std::string, cv::Mat>* extrinsicMatrices,
										const std::map<std::string, cv::Mat>* undistortMapsX,
										const std::map<std::string, cv::Mat>* undistortMapsY,
										const CvSize imageSize)
{
	Release();

	m_ImageSize = imageSize;

	std::map<std::string, cv::Mat>::const_iterator matrixIterator;

	// Clone intrinisc matrices
	for ( matrixIterator=m_intrinsicMatrices.begin() ; matrixIterator != m_intrinsicMatrices.end(); matrixIterator++ )
	{
		m_intrinsicMatrices[matrixIterator->first] = matrixIterator->second.clone();
	}

	// Clone distortion parameters
	for ( matrixIterator=m_distortionCoeffs.begin() ; matrixIterator != m_distortionCoeffs.end(); matrixIterator++ )
	{
		m_distortionCoeffs[matrixIterator->first] = matrixIterator->second.clone();
	}

	// Clone undistortion map X
	for ( matrixIterator=m_undistortMapsX.begin() ; matrixIterator != m_undistortMapsX.end(); matrixIterator++ )
	{
		m_undistortMapsX[matrixIterator->first] = matrixIterator->second.clone();
	}

	// Clone undistortion map Y
	for ( matrixIterator=m_undistortMapsY.begin() ; matrixIterator != m_undistortMapsY.end(); matrixIterator++ )
	{
		m_undistortMapsY[matrixIterator->first] = matrixIterator->second.clone();
	}

	// Clone extrinsic matrix
	for ( matrixIterator=m_extrinsicMatrices.begin() ; matrixIterator != m_extrinsicMatrices.end(); matrixIterator++ )
	{
		m_extrinsicMatrices[matrixIterator->first] = matrixIterator->second.clone();
	}

	m_Initialized = true;
	return RET_OK;
}

unsigned long CameraSensorToolbox::ConvertCameraTypeToString(ipa_CameraSensors::t_cameraType cameraType, std::string &cameraTypeString)
{
	switch (cameraType)
	{
	case CAM_IC:
		cameraTypeString = "ICCam";
		break;
	case CAM_AVTPIKE:
		cameraTypeString = "AVTPikeCam";
		break;
	case CAM_AXIS:
		cameraTypeString = "AxisCam";
		break;
	case CAM_PROSILICA:
		cameraTypeString = "Prosilica";
		break;
	case CAM_VIRTUALCOLOR:
		cameraTypeString = "VirtualColorCam";
		break;
	case CAM_SWISSRANGER:
		cameraTypeString = "Swissranger";
		break;
	case CAM_PMDCAMCUBE:
		cameraTypeString = "PMDCamCube";
		break;
	case CAM_VIRTUALRANGE:
		cameraTypeString = "VirtualRangeCam";
		break;
	default:
		std::cerr << "ERROR - CameraSensorToolbox::ConvertCameraTypeToString:" << std::endl;
		std::cerr << "\t ... Camera type " << cameraType << " unspecified." << std::endl;
		return RET_FAILED;
	}

	return RET_OK;
}

cv::Mat CameraSensorToolbox::GetExtrinsicParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex)
{
	std::stringstream ss;
	std::string extrinsicMapName = "";

	ConvertCameraTypeToString(cameraType, extrinsicMapName);
	ss << extrinsicMapName << "_" << cameraIndex;

	if (m_extrinsicMatrices.find(ss.str()) == m_extrinsicMatrices.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::GetExtrinsicParameters:" << std::endl;
		std::cout << "\t ... Extrinsic matrix to '" << ss.str() << "' not specified\n";
		return cv::Mat();
	}
	else
	{
		return m_extrinsicMatrices[ss.str()];
	}
}

unsigned long CameraSensorToolbox::SetExtrinsicParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex,
														  const cv::Mat& _rotation, const cv::Mat& _translation)
{
	std::stringstream ss;
	std::string extrinsicMapName = "";

	ConvertCameraTypeToString(cameraType, extrinsicMapName);
	ss << extrinsicMapName << "_" << cameraIndex;

	return SetExtrinsicParameters(ss.str(), _rotation, _translation);
}

unsigned long CameraSensorToolbox::SetExtrinsicParameters(std::string key,
														  const cv::Mat& _rotation, const cv::Mat& _translation)
{
	CV_Assert( _rotation.rows == 3 && _rotation.cols == 3 && _rotation.depth() == CV_64FC(1));
	CV_Assert( _translation.rows == 3 && _translation.cols == 1 && _translation.depth() == CV_64FC(1));

	std::map<std::string, cv::Mat>::iterator iterator;
	iterator = m_extrinsicMatrices.find(key);
	if (iterator != m_extrinsicMatrices.end())
	{
		m_extrinsicMatrices.erase(iterator);
	}

	cv::Mat extrinsicMatrix(3, 4, CV_64FC(1), cv::Scalar(0));

	extrinsicMatrix.at<double>(0, 0) = _rotation.at<double>(0, 0);
	extrinsicMatrix.at<double>(0, 1) = _rotation.at<double>(0, 1);
	extrinsicMatrix.at<double>(0, 2) = _rotation.at<double>(0, 2);
	extrinsicMatrix.at<double>(1, 0) = _rotation.at<double>(1, 0);
	extrinsicMatrix.at<double>(1, 1) = _rotation.at<double>(1, 1);
	extrinsicMatrix.at<double>(1, 2) = _rotation.at<double>(1, 2);
	extrinsicMatrix.at<double>(2, 0) = _rotation.at<double>(2, 0);
	extrinsicMatrix.at<double>(2, 1) = _rotation.at<double>(2, 1);
	extrinsicMatrix.at<double>(2, 2) = _rotation.at<double>(2, 2);

	extrinsicMatrix.at<double>(0, 3) = _translation.at<double>(0, 0);
	extrinsicMatrix.at<double>(1, 3) = _translation.at<double>(1, 0);
	extrinsicMatrix.at<double>(2, 3) = _translation.at<double>(2, 0);

	m_extrinsicMatrices[key] = extrinsicMatrix;

	return RET_OK;
}

cv::Mat CameraSensorToolbox::GetIntrinsicMatrix(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex)
{
	std::stringstream ss;
	std::string intrinsicMapName = "";

	ConvertCameraTypeToString(cameraType, intrinsicMapName);
	ss << intrinsicMapName << "_" << cameraIndex;

	if (m_intrinsicMatrices.find(ss.str()) == m_intrinsicMatrices.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::GetIntrinsicMatrix:" << std::endl;
		std::cout << "\t ... Intrinsic matrix related to '" << ss.str() << "' not specified\n";
		return cv::Mat();
	}
	else
	{
		return m_intrinsicMatrices[ss.str()];
	}
}

unsigned long CameraSensorToolbox::SetIntrinsicParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex,
														  const cv::Mat& _intrinsicMatrix, const cv::Mat& _distortionCoeffs)
{
	std::stringstream ss;
	std::string intrinsicMapName = "";

	ConvertCameraTypeToString(cameraType, intrinsicMapName);
	ss << intrinsicMapName << "_" << cameraIndex;

	return SetIntrinsicParameters(ss.str(), _intrinsicMatrix, _distortionCoeffs);
}

unsigned long CameraSensorToolbox::SetIntrinsicParameters(std::string key,
														  const cv::Mat& _intrinsicMatrix, const cv::Mat& _distortionCoeffs)
{
	CV_Assert( _intrinsicMatrix.rows == 4 && _intrinsicMatrix.cols == 1 && _intrinsicMatrix.depth() == CV_64FC(1));
	CV_Assert( _distortionCoeffs.rows == 4 && _distortionCoeffs.cols == 1 && _distortionCoeffs.depth() == CV_64FC(1));

	std::map<std::string, cv::Mat>::iterator matrixIterator;

	// Initialize intrinsic matrix
	// [fx 0 cx; 0 fy cy; 0 0 1]
	matrixIterator = m_intrinsicMatrices.find(key);
	if (matrixIterator != m_intrinsicMatrices.end())
	{
		m_intrinsicMatrices.erase(matrixIterator);
	}

	cv::Mat intrinsicMatrix (3, 3, CV_64FC(1), cv::Scalar(0) );

	intrinsicMatrix.at<double>(0, 0) = _intrinsicMatrix.at<double>(0, 0);
	intrinsicMatrix.at<double>(1, 1) = _intrinsicMatrix.at<double>(1, 0);
	intrinsicMatrix.at<double>(0, 2) = _intrinsicMatrix.at<double>(2, 0);
	intrinsicMatrix.at<double>(1, 2) = _intrinsicMatrix.at<double>(3, 0);
	intrinsicMatrix.at<double>(2, 2) = 1.f;

	m_intrinsicMatrices[key] = intrinsicMatrix;

	// Initialize distortion coeffs
	matrixIterator = m_distortionCoeffs.find(key);
	if (matrixIterator != m_distortionCoeffs.end())
	{
		m_distortionCoeffs.erase(matrixIterator);
	}

	cv::Mat distortionCoeffs(1, 4, CV_64FC(1), cv::Scalar(0) );//Initialisierung

	distortionCoeffs.at<double>(0, 0) = _distortionCoeffs.at<double>(0, 0);
	distortionCoeffs.at<double>(0, 1) = _distortionCoeffs.at<double>(1, 0);
	distortionCoeffs.at<double>(0, 2) = _distortionCoeffs.at<double>(2, 0);
	distortionCoeffs.at<double>(0, 3) = _distortionCoeffs.at<double>(3, 0);

	m_distortionCoeffs[key] = distortionCoeffs;

	// Initialize undistortion matrix X and Y
	matrixIterator = m_undistortMapsX.find(key);
	if (matrixIterator != m_undistortMapsX.end())
	{
		m_undistortMapsX.erase(matrixIterator);
	}

	cv::Mat undistortMapX(m_ImageSize.height, m_ImageSize.width, CV_32FC(1));

	matrixIterator = m_undistortMapsY.find(key);
	if (matrixIterator != m_undistortMapsY.end())
	{
		m_undistortMapsY.erase(matrixIterator);
	}
 
	cv::Mat undistortMapY(m_ImageSize.height, m_ImageSize.width, CV_32FC(1));

	ipa_Utils::InitUndistortMap(intrinsicMatrix, distortionCoeffs, undistortMapX, undistortMapY);

	m_undistortMapsX[key] = undistortMapX;
	m_undistortMapsY[key] = undistortMapY;

	return RET_OK;
}

cv::Mat CameraSensorToolbox::GetDistortionParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex)
{
	std::stringstream ss;
	std::string distortionMapName = "";

	ConvertCameraTypeToString(cameraType, distortionMapName);
	ss << distortionMapName << "_" << cameraIndex;

	if (m_distortionCoeffs.find(ss.str()) == m_distortionCoeffs.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::GetDistortionParameters:" << std::endl;
		std::cout << "\t ... Distortion parameters related to '" << ss.str() << "' not specified\n";
		return cv::Mat();
	}
	else
	{
		return m_distortionCoeffs[ss.str()];
	}
}

cv::Mat CameraSensorToolbox::GetDistortionMapX(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex)
{
	std::stringstream ss;
	std::string distortionMapName = "";

	ConvertCameraTypeToString(cameraType, distortionMapName);
	ss << distortionMapName << "_" << cameraIndex;

	if (m_undistortMapsX.find(ss.str()) == m_undistortMapsX.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::GetDistortionMapX:" << std::endl;
		std::cout << "\t ... Undistortion map X related to '" << ss.str() << "' not specified\n";
		return cv::Mat();
	}
	else
	{
		return m_undistortMapsX[ss.str()];
	}
}

cv::Mat CameraSensorToolbox::GetDistortionMapY(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex)
{
	std::stringstream ss;
	std::string distortionMapName = "";

	ConvertCameraTypeToString(cameraType, distortionMapName);
	ss << distortionMapName << "_" << cameraIndex;

	if (m_undistortMapsY.find(ss.str()) == m_undistortMapsY.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::GetDistortionMapY:" << std::endl;
		std::cout << "\t ... Undistortion map Y related to '" << ss.str() << "' not specified\n";
		return cv::Mat();
	}
	else
	{
		return m_undistortMapsY[ss.str()];
	}
}

unsigned long CameraSensorToolbox::RemoveDistortion(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex, const cv::Mat& src, cv::Mat& dst)
{
	std::stringstream ss;
	std::string distortionMapName = "";

	ConvertCameraTypeToString(cameraType, distortionMapName);
	ss << distortionMapName << "_" << cameraIndex;

	if (m_undistortMapsX.find(ss.str()) == m_undistortMapsX.end() ||
		m_undistortMapsY.find(ss.str()) == m_undistortMapsY.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::RemoveDistortion:" << std::endl;
		std::cout << "\t ... Undistortion map Y related to '" << ss.str() << "' not specified\n";
		return RET_FAILED;
	}
	else
	{
		CV_Assert(src.rows == m_undistortMapsX[ss.str()].rows && src.cols == m_undistortMapsX[ss.str()].cols);

		cv::remap(src, dst, m_undistortMapsX[ss.str()], m_undistortMapsY[ss.str()], cv::INTER_LINEAR);
		return RET_OK;
	}

	return (RET_FAILED | RET_MISSING_INTRINSIC_DISTORTION_PARAMS);
}

unsigned long CameraSensorToolbox::ReprojectXYZ(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex, double x, double y, double z, int& u, int& v)
{
	std::stringstream ss;
	std::string distortionMapName = "";

	ConvertCameraTypeToString(cameraType, distortionMapName);
	ss << distortionMapName << "_" << cameraIndex;

	if (m_intrinsicMatrices.find(ss.str()) == m_intrinsicMatrices.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::ReprojectXYZ:" << std::endl;
		std::cout << "\t ... Intrinsic matrix related to '" << ss.str() << "' not specified\n";
		return RET_FAILED;
	}

	cv::Mat UV1 (3, 1, CV_64FC(1), cv::Scalar(0));
	cv::Mat XYZ (3, 1, CV_64FC(1), cv::Scalar(0));

	x *= 1000;
	y *= 1000;
	z *= 1000;

	x = x/z;
	y = y/z;
	z = 1;

	XYZ.at<double>(0, 0) = x;
	XYZ.at<double>(1, 0) = y;
	XYZ.at<double>(2, 0) = z;

	// Fundamental equation: u = (fx*x)/z + cx
	if (z == 0)
	{
		std::cerr << "ERROR - CameraSensorToolbox::ReprojectXYZ" << std::endl;
		std::cerr << "\t ... z value is 0.\n";
		return RET_FAILED;
	}

	UV1 = m_intrinsicMatrices[ss.str()] * XYZ;

	u = cvRound(UV1.at<double>(0, 0));
	v = cvRound(UV1.at<double>(1, 0));

	return RET_OK;
}


unsigned long CameraSensorToolbox::LoadParameters(const char* filename, ipa_CameraSensors::t_cameraType cameraType, int cameraIndex)
{
	std::stringstream ss;
	std::string xmlTagName = "";

	ConvertCameraTypeToString(cameraType, xmlTagName);
	ss << xmlTagName << "_" << cameraIndex;

	boost::shared_ptr<TiXmlDocument> p_configXmlDocument (new TiXmlDocument( filename ));
	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
		std::cerr << "\t ...  Error while loading xml configuration file (Check filename and syntax of the file):\n" << filename << std::endl;
		return (RET_FAILED | RET_FAILED_OPEN_FILE);
	}
	std::cout << "INFO - CameraSensorsToolbox::LoadParameters:" << std::endl;
	std::cout << "\t ...  Parsing xml configuration file:" << std::endl;
	std::cout << "\t ... '" << filename << "'" << std::endl;

	if ( p_configXmlDocument )
	{

//************************************************************************************
//	BEGIN LibCameraSensors
//************************************************************************************
		// Tag element "LibCameraSensors" of Xml Inifile
		TiXmlElement *p_xmlElement_Root = NULL;
		p_xmlElement_Root = p_configXmlDocument->FirstChildElement( "LibCameraSensors" );
		if ( p_xmlElement_Root )
		{

//************************************************************************************
//	BEGIN LibCameraSensors->XXXCam
//************************************************************************************
			TiXmlElement *p_xmlElement_Root_Cam = NULL;
			p_xmlElement_Root_Cam = p_xmlElement_Root->FirstChildElement( ss.str() );
			if ( p_xmlElement_Root_Cam )
			{

//************************************************************************************
//	BEGIN LibCameraSensors->CameraSensorsToolbox->IntrinsicParameters
//************************************************************************************
				TiXmlElement *p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Cam->FirstChildElement( "IntrinsicParameters" );
				if ( p_xmlElement_Child )
				{
					TiXmlElement *p_xmlElement_Intrinsics = 0;
					TiXmlElement *p_xmlElement_Intrinsics_Child = 0;
					// Iterate all children (intrinsic matrices)
					for( p_xmlElement_Intrinsics = p_xmlElement_Child->FirstChildElement();
						p_xmlElement_Intrinsics;
						p_xmlElement_Intrinsics = p_xmlElement_Intrinsics->NextSiblingElement())
					{
//************************************************************************************
//	BEGIN LibCameraSensors->CameraSensorsToolbox->IntrinsicParameters->IntrinsicMatrix
//************************************************************************************
						// Subtag element "Translation" of Xml Inifile
						cv::Mat intrinsicMatrix (4, 1, CV_64FC(1));
						p_xmlElement_Intrinsics_Child = NULL;
						p_xmlElement_Intrinsics_Child = p_xmlElement_Intrinsics->FirstChildElement( "IntrinsicMatrix" );

						if ( p_xmlElement_Intrinsics_Child )
						{
							double fx, fy, cx, cy;
							// read and save value of attribute
							if ( p_xmlElement_Intrinsics_Child->QueryValueAttribute( "fx", &fx ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'fx' of tag 'IntrinsicMatrix'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Intrinsics_Child->QueryValueAttribute( "fy", &fy ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'fy' of tag 'IntrinsicMatrix'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Intrinsics_Child->QueryValueAttribute( "cx", &cx ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'cx' of tag 'IntrinsicMatrix'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Intrinsics_Child->QueryValueAttribute( "cy", &cy ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'cy' of tag 'IntrinsicMatrix'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}

							intrinsicMatrix.at<double>(0, 0) = fx;
							intrinsicMatrix.at<double>(1, 0) = fy;
							intrinsicMatrix.at<double>(2, 0) = cx;
							intrinsicMatrix.at<double>(3, 0) = cy;
						}
						else
						{
							std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
							std::cerr << "\t ...  Can't find tag 'IntrinsicMatrix'." << std::endl;
							return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
						}

//************************************************************************************
//	BEGIN LibCameraSensors->CameraSensorsToolbox->DistortionCoeffs
//************************************************************************************
						cv::Mat distortionCoeffs (4, 1, CV_64FC(1));
						p_xmlElement_Intrinsics_Child = NULL;
						p_xmlElement_Intrinsics_Child = p_xmlElement_Intrinsics->FirstChildElement( "DistortionCoeffs" );

						if ( p_xmlElement_Child )
						{
							double k1, k2, p1, p2;
							// read and save value of attribute
							if ( p_xmlElement_Intrinsics_Child->QueryValueAttribute( "k1", &k1 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'k1' of tag 'DistortionCoeffs '." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Intrinsics_Child->QueryValueAttribute( "k2", &k2 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'k2' of tag 'DistortionCoeffs '." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Intrinsics_Child->QueryValueAttribute( "p1", &p1 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'p1' of tag 'DistortionCoeffs '." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Intrinsics_Child->QueryValueAttribute( "p2", &p2 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'p2' of tag 'DistortionCoeffs '." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}

							distortionCoeffs.at<double>(0, 0) = k1;
							distortionCoeffs.at<double>(1, 0) = k2;
							distortionCoeffs.at<double>(2, 0) = p1;
							distortionCoeffs.at<double>(3, 0) = p2;
						}
						else
						{
							std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
							std::cerr << "\t ...  Can't find tag 'DistortionCoeffs '." << std::endl;
							return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
						}

						SetIntrinsicParameters(p_xmlElement_Intrinsics->Value(), intrinsicMatrix, distortionCoeffs);
					} // End 'intrinsic' for loop
				}
				else
				{
					std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'IntrinsicParameters'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->CameraSensorsToolbox->ExtrinsicParameters
//************************************************************************************
				// Subtag element "Translation" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Cam->FirstChildElement( "ExtrinsicParameters" );
				if ( p_xmlElement_Child )
				{
					TiXmlElement *p_xmlElement_Extrinsics = 0;
					TiXmlElement *p_xmlElement_Extrinsics_Child = 0;
					// Iterate all children (extrinsic matrices)
					for( p_xmlElement_Extrinsics = p_xmlElement_Child->FirstChildElement();
						p_xmlElement_Extrinsics;
						p_xmlElement_Extrinsics = p_xmlElement_Extrinsics->NextSiblingElement())
					{

//************************************************************************************
//	BEGIN LibCameraSensors->CameraSensorsToolbox->Translation
//************************************************************************************
						// Subtag element "Translation" of Xml Inifile
						cv::Mat extrinsicTranslation (3, 1, CV_64FC(1));
						p_xmlElement_Extrinsics_Child = NULL;
						p_xmlElement_Extrinsics_Child = p_xmlElement_Extrinsics->FirstChildElement( "Translation" );
						if ( p_xmlElement_Extrinsics_Child )
						{
							double x, y, z;
							// read and save value of attribute
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x", &x ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x' of tag 'Translation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "y", &y ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'y' of tag 'Translation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "z", &z ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'z' of tag 'Translation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							extrinsicTranslation.at<double>(0, 0) = x;
							extrinsicTranslation.at<double>(1, 0) = y;
							extrinsicTranslation.at<double>(2, 0) = z;
						}
						else
						{
							std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
							std::cerr << "\t ...  Can't find tag 'Translation'." << std::endl;
							return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
						}

//************************************************************************************
//	BEGIN LibCameraSensors->CameraSensorsToolbox->Rotation
//************************************************************************************
						// Subtag element "Rotation" of Xml Inifile
						cv::Mat extrinsicRotation (3, 3, CV_64FC(1));
						p_xmlElement_Extrinsics_Child = NULL;
						p_xmlElement_Extrinsics_Child = p_xmlElement_Extrinsics->FirstChildElement( "Rotation" );
						if ( p_xmlElement_Extrinsics_Child )
						{
							double x11, x12, x13;
							double x21, x22, x23;
							double x31, x32, x33;
							// read and save value of attribute
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x11", &x11 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:R" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x11' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x12", &x12 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x12' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x13", &x13 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x13' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x21", &x21 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x21' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x22", &x22 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x22' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x23", &x23 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x23' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x31", &x31 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x31' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x32", &x32 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x32' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x33", &x33 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x33' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}

							extrinsicRotation.at<double>(0, 0) = x11;
							extrinsicRotation.at<double>(0, 1) = x12;
							extrinsicRotation.at<double>(0, 2) = x13;
							extrinsicRotation.at<double>(1, 0) = x21;
							extrinsicRotation.at<double>(1, 1) = x22;
							extrinsicRotation.at<double>(1, 2) = x23;
							extrinsicRotation.at<double>(2, 0) = x31;
							extrinsicRotation.at<double>(2, 1) = x32;
							extrinsicRotation.at<double>(2, 2) = x33;
						}
						else
						{
							std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
							std::cerr << "\t ...  Can't find tag 'Rotation'." << std::endl;
							return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
						}

						SetExtrinsicParameters(p_xmlElement_Extrinsics->Value(), extrinsicRotation, extrinsicTranslation);
					} // End 'extrinsic' for loop
				}
				else
				{
					std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'ExtrinsicParameters'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
			}

//************************************************************************************
//	END LibCameraSensors->CameraSensorsToolbox
//************************************************************************************
			else
			{
				std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
				std::cerr << "\t ... Can't find tag '" << ss.str() << "'" << std::endl;
				return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
			}
		}

//************************************************************************************
//	END LibCameraSensors
//************************************************************************************
		else
		{
			std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
			std::cerr << "\t ...  Can't find tag 'LibCameraSensors'." << std::endl;
			return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
		}
	}

	return RET_OK;
}
