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
#include "cob_vision_utils/VisionUtils.h"
#else
#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/VisionUtils.h"
#endif

using namespace ipa_Utils;

cv::Mat ipa_Utils::vstack(const std::vector<cv::Mat> &mats)
{
    if (mats.empty())
        return cv::Mat();

    // we need to know the total number of rows to create the stacked matrix
    int nRows = 0;
    int nCols = mats.front().cols;
    int datatype = mats.front().type();
    std::vector<cv::Mat>::const_iterator it;
    for (it = mats.begin(); it != mats.end(); ++it)
    {
        nRows += it->rows;
        // make sure all mats have same num of cols and data type
        CV_Assert(it->cols == nCols);
        CV_Assert(it->type() == datatype);
    }

    // copy data to stacked matrix
    int startRow = 0;
    int endRow = 0;
    cv::Mat stacked(nRows, nCols, datatype);
    for (it = mats.begin(); it != mats.end(); ++it)
    {
        startRow = endRow;
        endRow = startRow + it->rows;
	cv::Mat mat = stacked.rowRange(startRow, endRow);
	it->copyTo(mat);
    }

    return stacked;
}

unsigned long ipa_Utils::EvaluatePolynomial(double x, int degree, double* coefficients, double* y)
{
	(*y) = coefficients[degree];
	for (int i = degree-1; i >= 0; i--)
	{
		(*y) *= x;
		(*y) += coefficients[i];
	}

	return ipa_Utils::RET_OK;
}

void ipa_Utils::InitUndistortMap( const cv::Mat& _A, const cv::Mat& _dist_coeffs,
                    cv::Mat& _mapxarr, cv::Mat& _mapyarr )
{
    uchar* buffer = 0;

	CvMat A = _A;
	CvMat dist_coeffs = _dist_coeffs;
	CvMat mapxarr = _mapxarr;
	CvMat mapyarr = _mapyarr;

    float a[9], k[4];
    int coi1 = 0, coi2 = 0;
    CvMat mapxstub, *_mapx = &mapxarr;
    CvMat mapystub, *_mapy = &mapyarr;
    float *mapx, *mapy;
    CvMat _a = cvMat( 3, 3, CV_32F, a ), _k;
    int mapxstep, mapystep;
    int u, v;
    float u0, v0, fx, fy, _fx, _fy, k1, k2, p1, p2;
    CvSize size;

    _mapx = cvGetMat( _mapx, &mapxstub, &coi1 );
    _mapy = cvGetMat( _mapy, &mapystub, &coi2 );
   
    cvConvert( &A, &_a );
    _k = cvMat( dist_coeffs.rows, dist_coeffs.cols,
                CV_MAKETYPE(CV_32F, CV_MAT_CN(dist_coeffs.type)), k );
    cvConvert( &dist_coeffs, &_k );

    u0 = a[2]; v0 = a[5];
    fx = a[0]; fy = a[4];
    _fx = 1.f/fx; _fy = 1.f/fy;
    k1 = k[0]; k2 = k[1];
    p1 = k[2]; p2 = k[3];

    mapxstep = _mapx->step ? _mapx->step : (1<<30);
    mapystep = _mapy->step ? _mapy->step : (1<<30);
    mapx = _mapx->data.fl;
    mapy = _mapy->data.fl;

    size = cvGetSize(_mapx);
    
    mapxstep /= sizeof(mapx[0]);
    mapystep /= sizeof(mapy[0]);

    for( v = 0; v < size.height; v++, mapx += mapxstep, mapy += mapystep )
    {
        float y = (v - v0)*_fy;
        float y2 = y*y;
        float _2p1y = 2*p1*y;
        float _3p1y2 = 3*p1*y2;
        float p2y2 = p2*y2;

        for( u = 0; u < size.width; u++ )
        {
            float x = (u - u0)*_fx;
            float x2 = x*x;
            float r2 = x2 + y2;
            float d = 1 + (k1 + k2*r2)*r2;
            float _u = fx*(x*(d + _2p1y) + p2y2 + (3*p2)*x2) + u0;
            float _v = fy*(y*(d + (2*p2)*x) + _3p1y2 + p1*x2) + v0;
            mapx[u] = _u;
            mapy[u] = _v;
        }
    }

    cvFree( &buffer );
}

unsigned long ipa_Utils::MaskImage(const cv::Mat& source, cv::Mat& dest, const cv::Mat& mask, cv::Mat& destMask, float minMaskThresh, float maxMaskThresh,
						int sourceChannel, double sourceMin, double sourceMax)
{
        double globalMin = -1;
		double globalMax = -1;

		double maskMin = -1;
		double maskMax = -1;

		dest.create(source.rows, source.cols, CV_8UC3);	

		CV_Assert(sourceChannel >= 1);
		CV_Assert(sourceChannel <= source.channels());

		/// Check if destination image has been initialized correctly
		CV_Assert(destMask.depth() == CV_8U);
		CV_Assert(destMask.channels() == 3);
		CV_Assert(destMask.cols == source.cols);
		CV_Assert(destMask.rows == source.rows);

		/// Check if mask image has been initialized correctly
		CV_Assert(mask.depth() == CV_32F);
		CV_Assert(mask.channels() == 1);
		CV_Assert(mask.cols == source.cols);
		CV_Assert(mask.rows == source.rows);

		/// Calculate minmal and maximal value within the specified image sourceChannel
		/// Channel must be within [1, source->nChannels]
		if (sourceMin == -1 || sourceMax == -1)
		{
			cv::Mat mixImage(source.rows, source.cols, source.depth(), 1);

			// Copy channel 2 of source to channel 0 of zSource
			int from_to[] = {sourceChannel-1, 0};

			cv::mixChannels(&source, 1, &mixImage, 1, from_to, 1);
			cv::minMaxLoc(mixImage, &globalMin, &globalMax);
		}
		else
		{
			std::cerr << "ERROR - OpenCVUtils::MaskImage:" << std::endl;
			std::cerr << "\t ... Parameter sourceChannel ('" << sourceChannel << "') out of range.\n";
			return RET_FAILED;
		}
		
		if (sourceMin == -1) sourceMin = globalMin;
		if (sourceMax == -1) sourceMax = globalMax;

		cv::minMaxLoc(mask, &maskMin, &maskMax);
		double wMask = maskMax-maskMin;

		double w = sourceMax-sourceMin;
		int destIndex = 0;
		int sourceIndex = 0;
		int maskIndex = 0;

		if (source.depth() == CV_32F)
		{
			for(int j=0; j<source.rows; j++)
			{
				const float* f_source_ptr = source.ptr<float>(j);
				const float* f_mask_ptr = mask.ptr<float>(j);
				
				unsigned char* c_dest_ptr = dest.ptr<unsigned char>(j);
				unsigned char* c_destMask_ptr = destMask.ptr<unsigned char>(j);
		
				for(int i=0; i<source.cols; i++)
				{
					unsigned char V = 0;
					unsigned char vMask = 0;
					destIndex = i*3;
					sourceIndex = i*source.channels();
					maskIndex = i*mask.channels();

					double z = (double)f_source_ptr[sourceIndex + sourceChannel - 1];
					float maskVal = f_mask_ptr[maskIndex];
					if (maskVal < maxMaskThresh &&
						maskVal > minMaskThresh)
					{
						if (z < sourceMin) 
						{
							z = sourceMin;
							maskVal = (float)maskMin;
						}
						if (z > sourceMax)
						{
							z = sourceMax;
							maskVal = (float)maskMax;
						}
						V= (unsigned char)(255.0 * ((z-sourceMin)/w));

						vMask= (unsigned char)(255.0 * ((maskVal-globalMin)/wMask));
						
					}
					else
					{
						V = 0;
						vMask = 0;
					}

					c_dest_ptr[destIndex] = V;
					c_dest_ptr[destIndex + 1] = V;
					c_dest_ptr[destIndex + 2] = V;

					c_destMask_ptr[destIndex] = vMask;
					c_destMask_ptr[destIndex + 1] = vMask;
					c_destMask_ptr[destIndex + 2] = vMask;
				}
			}
		}
		else if (source.depth() == CV_32S)
		{
			for(int j=0; j<source.rows; j++)
			{
				const float* f_mask_ptr = mask.ptr<float>(j);
				const int* i_source_ptr = source.ptr<int>(j);
				unsigned char* c_dest_ptr = dest.ptr<unsigned char>(j);

				for(int i=0; i<source.cols; i++)
				{
					int iTimes3 = i*3;
					unsigned char V = 0;
		
					double z = (double)i_source_ptr[i*source.channels() + sourceChannel - 1];
				
					float maskVal = f_mask_ptr[maskIndex];
					if (maskVal < maxMaskThresh &&
						maskVal > minMaskThresh)
					{
						if (z < sourceMin) z = sourceMin;
						if (z > sourceMax) z = sourceMax;
						V = (unsigned char)(255.0 * ((z-sourceMin)/w));
					}
					else
					{
						V = 0;
					}

					c_dest_ptr[iTimes3] = V;
					c_dest_ptr[iTimes3 + 1] = V;
					c_dest_ptr[iTimes3 + 2] = V;
				}
			}
		}
		else if (source.depth() == CV_8U)
		{
			for(int j=0; j<source.rows; j++)
			{
				const float* f_mask_ptr = mask.ptr<float>(j);
				const unsigned char* c_source_ptr = source.ptr<unsigned char>(j);
				unsigned char* c_dest_ptr = dest.ptr<unsigned char>(j);

				for(int i=0; i<source.cols; i++)
				{
					int iTimes3 = i*3;
					unsigned char V = 0;
		
					double z = (double)c_source_ptr[i*source.channels() + sourceChannel - 1];
	
					float maskVal = f_mask_ptr[maskIndex];
					if (maskVal < maxMaskThresh &&
						maskVal > minMaskThresh)
					{
						if (z < sourceMin) z = sourceMin;
						if (z > sourceMax) z = sourceMax;
						V = (unsigned char)(255.0 * ((z-sourceMin)/w));
					}
					else
					{
						V = 0;
					}

					c_dest_ptr[iTimes3] = V;
					c_dest_ptr[iTimes3 + 1] = V;
					c_dest_ptr[iTimes3 + 2] = V;
				}
			}
		}
		else
		{
			std::cout << "ERROR - OpenCVUtils::MaskImage:" << std::endl;
			std::cout << "\t ... Image depth of source not supported.\n";
			return RET_FAILED;
		}
		

		return RET_OK;
}

unsigned long ipa_Utils::ConvertToShowImage(const cv::Mat& source, cv::Mat& dest, int channel, double min, double max)
{
        double globalMin = -1;
		double globalMax = -1;

		CV_Assert(channel >= 1);
		CV_Assert(channel <= source.channels());

		dest.create(source.rows, source.cols, CV_8UC3);	

		/// Calculate minmal and maximal value within the specified image channel
		cv::Mat mixImage(source.rows, source.cols, source.depth(), 1);

		// Copy channel 2 of source to channel 0 of zSource
		int from_to[] = {channel-1, 0};

		cv::mixChannels(&source, 1, &mixImage, 1, from_to, 1);
		cv::minMaxLoc(mixImage, &globalMin, &globalMax);
				
		if (min == -1) min = globalMin;
		if (max == -1) max = globalMax;

		double w = max-min;

		if (source.depth() == CV_32F)
		{
			for(int j=0; j<source.rows; j++)
			{
				const float* f_source_ptr = source.ptr<float>(j);				
				unsigned char* c_dest_ptr = dest.ptr<unsigned char>(j);

				for(int i=0; i<source.cols; i++)
				{
					int iTimes3 = i*3;
		
					double z = (double)f_source_ptr[i*source.channels() + channel - 1];
					
					if (z < min) z = min;
					if (z > max) z = max;

					int V= (int)(255.0 * ((z-min)/w));

					c_dest_ptr[iTimes3] = V;
					c_dest_ptr[iTimes3 + 1] = V;
					c_dest_ptr[iTimes3 + 2] = V;
				}
			}
		}
		else if (source.depth() == CV_32S)
		{
			for(int j=0; j<source.rows; j++)
			{
				const int* i_source_ptr = source.ptr<int>(j);
				unsigned char* c_dest_ptr = dest.ptr<unsigned char>(j);

				for(int i=0; i<source.cols; i++)
				{
					int iTimes3 = i*3;
		
					double z = (double)i_source_ptr[i*source.channels() + channel - 1];
				
					if (z < min) z = min;
					if (z > max) z = max;		

					int V= (int)(255.0 * ((z-min)/w));

					c_dest_ptr[iTimes3] = V;
					c_dest_ptr[iTimes3 + 1] = V;
					c_dest_ptr[iTimes3 + 2] = V;
				}
			}
		}
		else if (source.depth() == CV_8U)
		{
			for(int j=0; j<source.rows; j++)
			{
				const unsigned char* c_source_ptr = source.ptr<unsigned char>(j);
				unsigned char* c_dest_ptr = dest.ptr<unsigned char>(j);

				for(int i=0; i<source.cols; i++)
				{
					int iTimes3 = i*3;
		
					double z = (double)c_source_ptr[i*source.channels() + channel - 1];
	
					if (z < min) z = min;
					if (z > max) z = max;

					int V= (int)(255.0 * ((z-min)/w));

					c_dest_ptr[iTimes3] = V;
					c_dest_ptr[iTimes3 + 1] = V;
					c_dest_ptr[iTimes3 + 2] = V;
				}
			}
		}
		else
		{
			std::cout << "ERROR - OpenCVUtils::ConvertToShowImage:" << std::endl;
			std::cout << "\t ... Image depth of source not supported.\n";
			return RET_FAILED;
		}
		

		return RET_OK;
}
