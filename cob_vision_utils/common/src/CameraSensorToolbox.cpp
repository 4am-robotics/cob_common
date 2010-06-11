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

#ifdef __cplusplus
extern "C" {
#endif
__DLL_CAMERASENSORTOOLBOX_H__ void APIENTRY ReleaseCameraSensorToolbox(CameraSensorToolbox* toolbox)
{
	delete toolbox;
}
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif
__DLL_CAMERASENSORTOOLBOX_H__ CameraSensorToolbox* APIENTRY CreateCameraSensorToolbox()
{
	return (new CameraSensorToolbox());
}
#ifdef __cplusplus
}
#endif

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

	// Release intrinsic matrix
	if (!m_intrinsicMatrices.empty())
	{
		for (matrixIterator=m_intrinsicMatrices.begin(); matrixIterator!=m_intrinsicMatrices.end(); matrixIterator++)
		{
			cvReleaseMat(&matrixIterator->second);
		}
		m_intrinsicMatrices.erase(m_intrinsicMatrices.begin(), m_intrinsicMatrices.end());
	}

	// Release distortion parameters
	if (!m_distortionCoeffs.empty())
	{
		for (matrixIterator=m_distortionCoeffs.begin(); matrixIterator!=m_distortionCoeffs.end(); matrixIterator++)
		{
			cvReleaseMat(&matrixIterator->second);
		}
		m_distortionCoeffs.erase(m_distortionCoeffs.begin(), m_distortionCoeffs.end());
	}

	// Release undistortion maps X
	if (!m_undistortMapsX.empty())
	{
		for (iplImageIterator=m_undistortMapsX.begin(); iplImageIterator!=m_undistortMapsX.end(); iplImageIterator++)
		{
			cvReleaseImage(&iplImageIterator->second);
		}
		m_undistortMapsX.erase(m_undistortMapsX.begin(), m_undistortMapsX.end());
	}

	// Release undistortion maps Y
	if (!m_undistortMapsY.empty())
	{
		for (iplImageIterator=m_undistortMapsY.begin(); iplImageIterator!=m_undistortMapsY.end(); iplImageIterator++)
		{
			cvReleaseImage(&iplImageIterator->second);
		}
		m_undistortMapsY.erase(m_undistortMapsY.begin(), m_undistortMapsY.end());
	}

	// Release extrinsic matrices
	if (!m_extrinsicMatrices.empty())
	{
		for (matrixIterator=m_extrinsicMatrices.begin(); matrixIterator!=m_extrinsicMatrices.end(); matrixIterator++)
		{
			cvReleaseMat(&matrixIterator->second);
		}
		m_extrinsicMatrices.erase(m_extrinsicMatrices.begin(), m_extrinsicMatrices.end());
	}

	return RET_OK;
}

CameraSensorToolbox::CameraSensorToolbox(const CameraSensorToolbox& cst)
{
	Release();

	std::map<std::string, CvMat*>::const_iterator matrixIterator;
	std::map<std::string, IplImage*>::const_iterator iplImageIterator;

	// Clone intrinisc matrices
	for ( matrixIterator=cst.m_intrinsicMatrices.begin() ; matrixIterator != cst.m_intrinsicMatrices.end(); matrixIterator++ )
	{
		m_intrinsicMatrices[matrixIterator->first] = cvCloneMat(matrixIterator->second);
	}

	// Clone distortion parameters
	for ( matrixIterator=cst.m_distortionCoeffs.begin() ; matrixIterator != cst.m_distortionCoeffs.end(); matrixIterator++ )
	{
		m_distortionCoeffs[matrixIterator->first] = cvCloneMat(matrixIterator->second);
	}

	// Clone undistortion map X
	for ( iplImageIterator=cst.m_undistortMapsX.begin() ; iplImageIterator != cst.m_undistortMapsX.end(); iplImageIterator++ )
	{
		m_undistortMapsX[iplImageIterator->first] = cvCloneImage(iplImageIterator->second);
	}

	// Clone undistortion map Y
	for ( iplImageIterator=cst.m_undistortMapsY.begin() ; iplImageIterator != cst.m_undistortMapsY.end(); iplImageIterator++ )
	{
		m_undistortMapsY[iplImageIterator->first] = cvCloneImage(iplImageIterator->second);
	}

	// Clone extrinsic matrix
	for ( matrixIterator=cst.m_extrinsicMatrices.begin() ; matrixIterator != cst.m_extrinsicMatrices.end(); matrixIterator++ )
	{
		m_extrinsicMatrices[matrixIterator->first] = cvCloneMat(matrixIterator->second);
	}

	m_Initialized = cst.m_Initialized;
}

CameraSensorToolbox& CameraSensorToolbox::operator=(const CameraSensorToolbox& cst)
{
	/// Check for self-assignment
	if (this==&cst)
	{
		 return *this;
	}

	Release();

	std::map<std::string, CvMat*>::const_iterator matrixIterator;
	std::map<std::string, IplImage*>::const_iterator iplImageIterator;

	// Clone intrinisc matrices
	for ( matrixIterator=cst.m_intrinsicMatrices.begin() ; matrixIterator != cst.m_intrinsicMatrices.end(); matrixIterator++ )
	{
		m_intrinsicMatrices[matrixIterator->first] = cvCloneMat(matrixIterator->second);
	}

	// Clone distortion parameters
	for ( matrixIterator=cst.m_distortionCoeffs.begin() ; matrixIterator != cst.m_distortionCoeffs.end(); matrixIterator++ )
	{
		m_distortionCoeffs[matrixIterator->first] = cvCloneMat(matrixIterator->second);
	}

	// Clone undistortion map X
	for ( iplImageIterator=cst.m_undistortMapsX.begin() ; iplImageIterator != cst.m_undistortMapsX.end(); iplImageIterator++ )
	{
		m_undistortMapsX[iplImageIterator->first] = cvCloneImage(iplImageIterator->second);
	}

	// Clone undistortion map Y
	for ( iplImageIterator=cst.m_undistortMapsY.begin() ; iplImageIterator != cst.m_undistortMapsY.end(); iplImageIterator++ )
	{
		m_undistortMapsY[iplImageIterator->first] = cvCloneImage(iplImageIterator->second);
	}

	// Clone extrinsic matrix
	for ( matrixIterator=cst.m_extrinsicMatrices.begin() ; matrixIterator != cst.m_extrinsicMatrices.end(); matrixIterator++ )
	{
		m_extrinsicMatrices[matrixIterator->first] = cvCloneMat(matrixIterator->second);
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

unsigned long CameraSensorToolbox::Init(const std::map<std::string, CvMat*>* intrinsicMatrices,
										const std::map<std::string, CvMat*>* distortionParameters,
										const std::map<std::string, CvMat*>* extrinsicMatrices,
										const std::map<std::string, IplImage*>* undistortMapsX,
										const std::map<std::string, IplImage*>* undistortMapsY,
										const CvSize imageSize)
{
	Release();

	m_ImageSize = imageSize;

	std::map<std::string, CvMat*>::const_iterator matrixIterator;
	std::map<std::string, IplImage*>::const_iterator iplImageIterator;

	// Clone intrinisc matrices
	for ( matrixIterator=intrinsicMatrices->begin() ; matrixIterator != intrinsicMatrices->end(); matrixIterator++ )
	{
		m_intrinsicMatrices[matrixIterator->first] = cvCloneMat(matrixIterator->second);
	}

	// Clone distortion parameters
	for ( matrixIterator=distortionParameters->begin() ; matrixIterator != distortionParameters->end(); matrixIterator++ )
	{
		m_distortionCoeffs[matrixIterator->first] = cvCloneMat(matrixIterator->second);
	}

	// Clone undistortion map X
	for ( iplImageIterator=undistortMapsX->begin() ; iplImageIterator != undistortMapsX->end(); iplImageIterator++ )
	{
		m_undistortMapsX[iplImageIterator->first] = cvCloneImage(iplImageIterator->second);
	}

	// Clone undistortion map Y
	for ( iplImageIterator=undistortMapsY->begin() ; iplImageIterator != undistortMapsY->end(); iplImageIterator++ )
	{
		m_undistortMapsY[iplImageIterator->first] = cvCloneImage(iplImageIterator->second);
	}
	
	// Clone extrinsic matrices
	for ( matrixIterator=extrinsicMatrices->begin() ; matrixIterator != extrinsicMatrices->end(); matrixIterator++ )
	{
		m_extrinsicMatrices[matrixIterator->first] = cvCloneMat(matrixIterator->second);
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

CvMat* CameraSensorToolbox::GetExtrinsicParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex)
{
	std::stringstream ss;
	std::string extrinsicMapName = "";

	ConvertCameraTypeToString(cameraType, extrinsicMapName);
	ss << extrinsicMapName << "_" << cameraIndex;

	if (m_extrinsicMatrices.find(ss.str()) == m_extrinsicMatrices.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::GetExtrinsicParameters:" << std::endl;
		std::cout << "\t ... Extrinsic matrix to '" << ss.str() << "' not specified\n";
		return 0;
	}
	else
	{
		return m_extrinsicMatrices[ss.str()];
	}
}

unsigned long CameraSensorToolbox::GetExtrinsicParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex, CvMat** _extrinsic_matrix)
{
	std::stringstream ss;
	std::string extrinsicMapName = "";

	ConvertCameraTypeToString(cameraType, extrinsicMapName);
	ss << extrinsicMapName << "_" << cameraIndex;

	if (m_extrinsicMatrices.find(ss.str()) == m_extrinsicMatrices.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::GetExtrinsicParameters:" << std::endl;
		std::cout << "\t ... Extrinsic matrix to '" << ss.str() << "' not specified\n";
		return RET_FAILED;
	}
	else
	{
		(*_extrinsic_matrix) = cvCloneMat(m_extrinsicMatrices[ss.str()]);
		return RET_OK;
	}
}

unsigned long CameraSensorToolbox::SetExtrinsicParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex,
														  const CvMat* _rotation, const CvMat* _translation)
{
	std::stringstream ss;
	std::string extrinsicMapName = "";

	ConvertCameraTypeToString(cameraType, extrinsicMapName);
	ss << extrinsicMapName << "_" << cameraIndex;

	return SetExtrinsicParameters(ss.str(), _rotation, _translation);
}

unsigned long CameraSensorToolbox::SetExtrinsicParameters(std::string key,
														  const CvMat* _rotation, const CvMat* _translation)
{
	std::map<std::string, CvMat*>::iterator iterator;
	iterator = m_extrinsicMatrices.find(key);
	if (iterator != m_extrinsicMatrices.end())
	{
		cvReleaseMat(&iterator->second);
		m_extrinsicMatrices.erase(iterator);
	}

	CvMat* extrinsicMatrix = cvCreateMatHeader( 3, 4, CV_64FC1 );
	cvCreateData( extrinsicMatrix );
	cvSet(extrinsicMatrix, cvRealScalar(0), NULL);

	cvmSet(extrinsicMatrix, 0, 0, cvmGet(_rotation, 0, 0));
	cvmSet(extrinsicMatrix, 0, 1, cvmGet(_rotation, 0, 1));
	cvmSet(extrinsicMatrix, 0, 2, cvmGet(_rotation, 0, 2));
	cvmSet(extrinsicMatrix, 1, 0, cvmGet(_rotation, 1, 0));
	cvmSet(extrinsicMatrix, 1, 1, cvmGet(_rotation, 1, 1));
	cvmSet(extrinsicMatrix, 1, 2, cvmGet(_rotation, 1, 2));
	cvmSet(extrinsicMatrix, 2, 0, cvmGet(_rotation, 2, 0));
	cvmSet(extrinsicMatrix, 2, 1, cvmGet(_rotation, 2, 1));
	cvmSet(extrinsicMatrix, 2, 2, cvmGet(_rotation, 2, 2));

	cvmSet(extrinsicMatrix, 0, 3, cvmGet(_translation, 0, 0));
	cvmSet(extrinsicMatrix, 1, 3, cvmGet(_translation, 1, 0));
	cvmSet(extrinsicMatrix, 2, 3, cvmGet(_translation, 2, 0));

	m_extrinsicMatrices[key] = extrinsicMatrix;

	return RET_OK;
}

CvMat* CameraSensorToolbox::GetIntrinsicMatrix(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex)
{
	std::stringstream ss;
	std::string intrinsicMapName = "";

	ConvertCameraTypeToString(cameraType, intrinsicMapName);
	ss << intrinsicMapName << "_" << cameraIndex;

	if (m_intrinsicMatrices.find(ss.str()) == m_intrinsicMatrices.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::GetIntrinsicMatrix:" << std::endl;
		std::cout << "\t ... Intrinsic matrix related to '" << ss.str() << "' not specified\n";
		return 0;
	}
	else
	{
		return m_intrinsicMatrices[ss.str()];
	}
}

unsigned long CameraSensorToolbox::GetIntrinsicMatrix(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex, CvMat** _intrinsic_matrix)
{
	std::stringstream ss;
	std::string intrinsicMapName = "";

	ConvertCameraTypeToString(cameraType, intrinsicMapName);
	ss << intrinsicMapName << "_" << cameraIndex;

	if (m_intrinsicMatrices.find(ss.str()) == m_intrinsicMatrices.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::GetIntrinsicMatrix:" << std::endl;
		std::cout << "\t ... Intrinsic matrix related to '" << ss.str() << "' not specified\n";
		return RET_FAILED;
	}
	else
	{
		(*_intrinsic_matrix) = cvCloneMat(m_intrinsicMatrices[ss.str()]);
		return RET_OK;
	}
}

unsigned long CameraSensorToolbox::SetIntrinsicParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex,
														  const CvMat* _intrinsicMatrix, const CvMat* _distortionCoeffs)
{
	std::stringstream ss;
	std::string intrinsicMapName = "";

	ConvertCameraTypeToString(cameraType, intrinsicMapName);
	ss << intrinsicMapName << "_" << cameraIndex;

	return SetIntrinsicParameters(ss.str(), _intrinsicMatrix, _distortionCoeffs);
}

unsigned long CameraSensorToolbox::SetIntrinsicParameters(std::string key,
														  const CvMat* _intrinsicMatrix, const CvMat* _distortionCoeffs)
{
	std::map<std::string, CvMat*>::iterator matrixIterator;
	std::map<std::string, IplImage*>::iterator iplImageIterator;

	// Initialize intrinsic matrix
	// [fx 0 cx; 0 fy cy; 0 0 1]
	matrixIterator = m_intrinsicMatrices.find(key);
	if (matrixIterator != m_intrinsicMatrices.end())
	{
		cvReleaseMat(&matrixIterator->second);
		m_intrinsicMatrices.erase(matrixIterator);
	}

	CvMat* intrinsicMatrix = cvCreateMatHeader( 3, 3, CV_64FC1 );
	cvCreateData( intrinsicMatrix );
	cvSet(intrinsicMatrix, cvRealScalar(0), NULL);

	cvmSet(intrinsicMatrix,0,0, cvmGet(_intrinsicMatrix, 0, 0));
	cvmSet(intrinsicMatrix,1,1, cvmGet(_intrinsicMatrix, 1, 0));
	cvmSet(intrinsicMatrix,0,2, cvmGet(_intrinsicMatrix, 2, 0));
	cvmSet(intrinsicMatrix,1,2, cvmGet(_intrinsicMatrix, 3, 0));
	cvmSet(intrinsicMatrix,2,2, 1.f);

	m_intrinsicMatrices[key] = intrinsicMatrix;

	// Initialize distortion coeffs
	matrixIterator = m_distortionCoeffs.find(key);
	if (matrixIterator != m_distortionCoeffs.end())
	{
		cvReleaseMat(&matrixIterator->second);
		m_distortionCoeffs.erase(matrixIterator);
	}

	CvMat* distortionCoeffs = cvCreateMatHeader( 1, 4, CV_64FC1 );//Initialisierung
	cvCreateData( distortionCoeffs );
	cvSet(distortionCoeffs,cvRealScalar(0), NULL);

	cvmSet(distortionCoeffs,0,0, cvmGet(_distortionCoeffs, 0, 0));
	cvmSet(distortionCoeffs,0,1, cvmGet(_distortionCoeffs, 1, 0));

	cvmSet(distortionCoeffs,0,2, cvmGet(_distortionCoeffs, 2, 0));
	cvmSet(distortionCoeffs,0,3, cvmGet(_distortionCoeffs, 3, 0));

	m_distortionCoeffs[key] = distortionCoeffs;

	// Initialize undistortion matrix X and Y
	iplImageIterator = m_undistortMapsX.find(key);
	if (iplImageIterator != m_undistortMapsX.end())
	{
		cvReleaseImage(&iplImageIterator->second);
		m_undistortMapsX.erase(iplImageIterator);
	}

	IplImage* undistortMapX = cvCreateImage(m_ImageSize, IPL_DEPTH_32F, 1);

	iplImageIterator = m_undistortMapsY.find(key);
	if (iplImageIterator != m_undistortMapsY.end())
	{
		cvReleaseImage(&iplImageIterator->second);
		m_undistortMapsY.erase(iplImageIterator);
	}
 
	IplImage* undistortMapY = cvCreateImage(m_ImageSize, IPL_DEPTH_32F, 1);

	ipa_Utils::InitUndistortMap(intrinsicMatrix, distortionCoeffs, undistortMapX, undistortMapY);

	m_undistortMapsX[key] = undistortMapX;
	m_undistortMapsY[key] = undistortMapY;

	return RET_OK;
}

CvMat* CameraSensorToolbox::GetDistortionParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex)
{
	std::stringstream ss;
	std::string distortionMapName = "";

	ConvertCameraTypeToString(cameraType, distortionMapName);
	ss << distortionMapName << "_" << cameraIndex;

	if (m_distortionCoeffs.find(ss.str()) == m_distortionCoeffs.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::GetDistortionParameters:" << std::endl;
		std::cout << "\t ... Distortion parameters related to '" << ss.str() << "' not specified\n";
		return 0;
	}
	else
	{
		return m_distortionCoeffs[ss.str()];
	}
}

unsigned long CameraSensorToolbox::GetDistortionParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex, CvMat** _distortion_parameters)
{
	std::stringstream ss;
	std::string distortionMapName = "";

	ConvertCameraTypeToString(cameraType, distortionMapName);
	ss << distortionMapName << "_" << cameraIndex;

	if (m_distortionCoeffs.find(ss.str()) == m_distortionCoeffs.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::GetDistortionParameters:" << std::endl;
		std::cout << "\t ... Distortion parameters related to '" << ss.str() << "' not specified\n";
		return RET_FAILED;
	}
	else
	{
		*_distortion_parameters = cvCloneMat(m_distortionCoeffs[ss.str()]);
		return RET_OK;
	}
}

IplImage* CameraSensorToolbox::GetDistortionMapX(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex)
{
	std::stringstream ss;
	std::string distortionMapName = "";

	ConvertCameraTypeToString(cameraType, distortionMapName);
	ss << distortionMapName << "_" << cameraIndex;

	if (m_undistortMapsX.find(ss.str()) == m_undistortMapsX.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::GetDistortionMapX:" << std::endl;
		std::cout << "\t ... Undistortion map X related to '" << ss.str() << "' not specified\n";
		return 0;
	}
	else
	{
		return m_undistortMapsX[ss.str()];
	}
}

unsigned long CameraSensorToolbox::GetDistortionMapX(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex, IplImage** _undistort_map_X)
{
	std::stringstream ss;
	std::string distortionMapName = "";

	ConvertCameraTypeToString(cameraType, distortionMapName);
	ss << distortionMapName << "_" << cameraIndex;

	if (m_undistortMapsX.find(ss.str()) == m_undistortMapsX.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::GetDistortionMapX:" << std::endl;
		std::cout << "\t ... Undistortion map X related to '" << ss.str() << "' not specified\n";
		return RET_FAILED;
	}
	else
	{
		*_undistort_map_X = cvCloneImage(m_undistortMapsX[ss.str()]);
		return RET_OK;
	}
}

IplImage* CameraSensorToolbox::GetDistortionMapY(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex)
{
	std::stringstream ss;
	std::string distortionMapName = "";

	ConvertCameraTypeToString(cameraType, distortionMapName);
	ss << distortionMapName << "_" << cameraIndex;

	if (m_undistortMapsY.find(ss.str()) == m_undistortMapsY.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::GetDistortionMapY:" << std::endl;
		std::cout << "\t ... Undistortion map Y related to '" << ss.str() << "' not specified\n";
		return 0;
	}
	else
	{
		return m_undistortMapsY[ss.str()];
	}
}

unsigned long CameraSensorToolbox::GetDistortionMapY(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex, IplImage** _undistort_map_Y)
{
	std::stringstream ss;
	std::string distortionMapName = "";

	ConvertCameraTypeToString(cameraType, distortionMapName);
	ss << distortionMapName << "_" << cameraIndex;

	if (m_undistortMapsY.find(ss.str()) == m_undistortMapsY.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::GetDistortionMapY:" << std::endl;
		std::cout << "\t ... Undistortion map Y related to '" << ss.str() << "' not specified\n";
		return RET_FAILED;
	}
	else
	{
		*_undistort_map_Y = cvCloneImage(m_undistortMapsY[ss.str()]);
		return RET_OK;
	}
}

unsigned long CameraSensorToolbox::RemoveDistortion(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex, const CvArr* src, CvArr* dst)
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
		cvRemap(src, dst, m_undistortMapsX[ss.str()], m_undistortMapsY[ss.str()]);
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

	CvMat* UV1 = cvCreateMat(3, 1, CV_64FC1);
	CvMat* XYZ = cvCreateMat(3, 1, CV_64FC1);

	cvSetZero( UV1 );
	cvSetZero( XYZ );

	x *= 1000;
	y *= 1000;
	z *= 1000;

	x = x/z;
	y = y/z;
	z = 1;

	cvmSet(XYZ, 0, 0, x);
	cvmSet(XYZ, 1, 0, y);
	cvmSet(XYZ, 2, 0, z);

	/// Fundamental equation: u = (fx*x)/z + cx
	if (z == 0)
	{
		std::cerr << "ERROR - CameraSensorToolbox::ReprojectXYZ" << std::endl;
		std::cerr << "\t ... z value is 0.\n";
		return RET_FAILED;
	}

	cvMatMulAdd( m_intrinsicMatrices[ss.str()], XYZ, 0, UV1 );

	u = cvRound(cvmGet(UV1, 0, 0));
	v = cvRound(cvmGet(UV1, 1, 0));

	cvReleaseMat(&UV1);
	cvReleaseMat(&XYZ);

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
					/// Iterate all children (intrinsic matrices)
					for( p_xmlElement_Intrinsics = p_xmlElement_Child->FirstChildElement();
						p_xmlElement_Intrinsics;
						p_xmlElement_Intrinsics = p_xmlElement_Intrinsics->NextSiblingElement())
					{
//************************************************************************************
//	BEGIN LibCameraSensors->CameraSensorsToolbox->IntrinsicParameters->IntrinsicMatrix
//************************************************************************************
						// Subtag element "Translation" of Xml Inifile
						CvMat* intrinsicMatrix = cvCreateMat(4, 1, CV_64FC1);
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

							cvmSet(intrinsicMatrix, 0, 0, fx);
							cvmSet(intrinsicMatrix, 1, 0, fy);
							cvmSet(intrinsicMatrix, 2, 0, cx);
							cvmSet(intrinsicMatrix, 3, 0, cy);
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
						CvMat* distortionCoeffs = cvCreateMat(4, 1, CV_64FC1);
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

							cvmSet(distortionCoeffs, 0, 0, k1);
							cvmSet(distortionCoeffs, 1, 0, k2);
							cvmSet(distortionCoeffs, 2, 0, p1);
							cvmSet(distortionCoeffs, 3, 0, p2);
						}
						else
						{
							std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
							std::cerr << "\t ...  Can't find tag 'DistortionCoeffs '." << std::endl;
							return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
						}

						SetIntrinsicParameters(p_xmlElement_Intrinsics->Value(), intrinsicMatrix, distortionCoeffs);
						cvReleaseMat(&intrinsicMatrix);
						cvReleaseMat(&distortionCoeffs);
					} /// End 'intrinsic' for loop
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
					/// Iterate all children (extrinsic matrices)
					for( p_xmlElement_Extrinsics = p_xmlElement_Child->FirstChildElement();
						p_xmlElement_Extrinsics;
						p_xmlElement_Extrinsics = p_xmlElement_Extrinsics->NextSiblingElement())
					{

//************************************************************************************
//	BEGIN LibCameraSensors->CameraSensorsToolbox->Translation
//************************************************************************************
						// Subtag element "Translation" of Xml Inifile
						CvMat* extrinsicTranslation = cvCreateMat(3, 1, CV_64FC1);
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
							cvmSet(extrinsicTranslation, 0, 0, x);
							cvmSet(extrinsicTranslation, 1, 0, y);
							cvmSet(extrinsicTranslation, 2, 0, z);
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
						CvMat* extrinsicRotation = cvCreateMat(3, 3, CV_64FC1);
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
							cvmSet(extrinsicRotation, 0, 0, x11);
							cvmSet(extrinsicRotation, 0, 1, x12);
							cvmSet(extrinsicRotation, 0, 2, x13);
							cvmSet(extrinsicRotation, 1, 0, x21);
							cvmSet(extrinsicRotation, 1, 1, x22);
							cvmSet(extrinsicRotation, 1, 2, x23);
							cvmSet(extrinsicRotation, 2, 0, x31);
							cvmSet(extrinsicRotation, 2, 1, x32);
							cvmSet(extrinsicRotation, 2, 2, x33);
						}
						else
						{
							std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
							std::cerr << "\t ...  Can't find tag 'Rotation'." << std::endl;
							return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
						}

						SetExtrinsicParameters(p_xmlElement_Extrinsics->Value(), extrinsicRotation, extrinsicTranslation);
						cvReleaseMat(&extrinsicTranslation);
						cvReleaseMat(&extrinsicRotation);
					} /// End 'extrinsic' for loop
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
