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
* Description: Implementations of the classes/functions declared and documented in MathUtils.h
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
#include "cob_vision_utils/MathUtils.h"
#else
#include "MathUtils.h"
#endif

/// @file MathUtils.cpp
/// Implementations of the classes/functions declared and documented in MathUtils.h

namespace ipa_Utils {

///*******************************************************
/// IpaVector
///*******************************************************

template<class T>
void IpaVector<T>::AssignCounter(int Size)
{
	this->clear();
	for(int i=0; i<Size; i++)
	{
		T j = (T) i; 
		push_back(j);
	}
}

template<class T>
std::string IpaVector<T>::Str() const
{
	std::stringstream str;
	str << this->size() << " ";
	for(unsigned int i=0; i<this->size(); i++)
		str << " " << (*this)[i] ;
	str << "\n";
	return str.str();
}

template<class T>
unsigned long IpaVector<T>::Load(std::string Name, bool Append)
{
	if(!Append)	this->clear();
	std::ifstream f(Name.c_str());
	if(!f.is_open())
	{
		std::cerr << "IpaVector<T>::Load: Could not load '" << Name << "'\n";
		return RET_FAILED;
	}
	int Size;
	f >> Size;
	for(int i=0; i<Size; i++)
	{
		T val;
		f >> val;
		this->push_back(val);
	}
	return RET_OK;
}

template<class T>
unsigned long IpaVector<T>::Save(std::string Name, bool Append) const
{	
	std::ios_base::openmode m;
	if(Append) m = std::fstream::out | std::fstream::app;
	else m = std::fstream::out;
	std::ofstream f(Name.c_str(), m);
	if(!f.is_open())
	{
		std::cerr << "IpaVector<T>::Save: Could not open '" << Name << "'\n";
		return RET_FAILED;
	}
	f << Str();
	return RET_OK;
}

template<class T>
void IpaVector<T>::WriteRandom(int Min, int Max, unsigned int* Size)
{
	unsigned int s=(unsigned int)this->size();
	if(Size!=NULL) s=(*Size);
	this->clear();
	for(unsigned int i=0; i<s; i++)
		this->push_back((T)RandT(Min, Max));
}


template<class T>
unsigned long IpaVector<T>::MinMax(T& Min, T& Max, int& MinPos, int& MaxPos) const
{
	if (this->size() > 0)
	{
		Min = (*this)[0];
		Max = (*this)[0];
	}
	else
	{
		return RET_FAILED;
	}

	for(unsigned int i=0; i<this->size(); i++)
	{
		T v=(*this)[i];
		if(v<Min) {Min=v; MinPos=i;}
		if(v>Max) {Max=v; MaxPos=i;}
	}
	return RET_OK;
}

template<class T>
bool IpaVector<T>::IsIn(T Val, int* Where) const
{
	for(unsigned int i=0; i<this->size(); i++)
	{
		if(Val==(*this)[i])
		{
			if(Where!=NULL) (*Where) = i;
			return true;
		}
	}
	return false;
}

template<class T>
T IpaVector<T>::Sum()
{
	T s = 0;
	for(unsigned int i=0; i<(this->size()); i++)
		s+=(*this)[i];
	return s;
}

/// This line is necessary to preven tlinker errors with tmplates
/// see for example http://www.parashift.com/c++-faq-lite/templates.html
template class IpaVector<float>;

///*******************************************************
/// IntVector (Deprecated. Use ipa vector instead)
///*******************************************************


void IntVector::AssignCounter(int Size)
{
	clear();
	for(int i=0; i<Size; i++)
		push_back(i);
}

std::string IntVector::Str() const
{
	std::stringstream str;
	str << this->size() << " ";
	for(unsigned int i=0; i<this->size(); i++)
		str << " " << (*this)[i] ;
	str << "\n";
	return str.str();
}

unsigned long IntVector::Load(std::string Name, bool Append)
{
	if(!Append)	this->clear();
	std::ifstream f(Name.c_str());
	if(!f.is_open()){std::cerr << "Could not load: " << Name << "\n"; return RET_FAILED;}
	int Size;
	f >> Size;
	for(int i=0; i<Size; i++)
	{
		int val;
		f >> val;
		this->push_back(val);
	}
	return RET_OK;
}

unsigned long IntVector::Save(std::string Name, bool Append) const
{	
	std::ios_base::openmode m;
	if(Append) m = std::fstream::out | std::fstream::app;
	else m = std::fstream::out;
	std::ofstream f(Name.c_str(), m);
	if(!f.is_open()){std::cerr << "Could not open: " << Name << "\n"; return RET_FAILED;}
	f << Str();
	return RET_OK;
}

void IntVector::WriteRandom(int Min, int Max, unsigned int* Size)
{
	unsigned int s=(unsigned int)this->size();
	if(Size!=NULL) s=*Size;
	this->clear();
	for(unsigned int i=0; i<s; i++)
		this->push_back(RandInt(Min, Max));
}

unsigned long IntVector::WriteRandomSet(int Min, int Max, unsigned int* Size)
{
	if(Max-Min < (int)(*Size-1))
	{
		std::cerr << "Vector too short\n";
		return RET_FAILED;
	}
	else
	{
		unsigned int s=(unsigned int)this->size();
		if(Size!=NULL) s=*Size;
		this->clear();
		std::map<int, int> m;
		for(unsigned int i=0; i<s; i++)
		{
			int r;
			while(true)
			{
				r=RandInt(Min, Max);
				if(m.find(r)==m.end())
				{
					m[r]=1;
					break;
				}
			}
			this->push_back(r);
		}
		std::sort(this->begin(), this->end());
		return RET_OK;
	}
}

void IntVector::MinMax(int& Min, int& Max, int& MinPos, int& MaxPos) const
{
	Min=INT_MAX; Max=-INT_MAX;
	for(unsigned int i=0; i<this->size(); i++)
	{
		int v=(*this)[i];
		if(v<Min) {Min=v; MinPos=i;}
		if(v>Max) {Max=v; MaxPos=i;}
	}
}

bool IntVector::IsIn(int Val, int* Where) const
{
	for(unsigned int i=0; i<this->size(); i++)
	{
		if(Val==(*this)[i])
		{
			if(Where!=NULL) *Where = i;
			return true;
		}
	}
	return false;
}

int IntVector::Sum()
{
	int s=0;
	for(unsigned int i=0; i<(this->size()); i++)
		s+=(*this)[i];
	return s;
}

bool IntVectorGreater(const IntVector& v1, const IntVector& v2)
{
	return v1.size() > v2.size();
}

/*
bool IntVectorSmaller(const IntVector& v1, const IntVector& v2)
{
	return v1.size() > v2.size();
}
*/

///*******************************************************
/// DblVector (Deprecated. Use ipa vector instead)
///*******************************************************


std::string DblVector::Str() const
{
	setlocale(LC_NUMERIC, "C");
	std::stringstream str;
	str << this->size() << " ";
	for(unsigned int i=0; i<this->size(); i++)
		str << " " << (*this)[i];
	str << std::endl;
	return str.str();
}

unsigned long DblVector::Load(std::string Name, bool Append)
{
	setlocale(LC_NUMERIC, "C");
	if(!Append)	this->clear();
	std::ifstream f(Name.c_str(), std::fstream::in);
	if(!f.is_open()){std::cerr << "DblVector::Load: Could not load: " << Name << "\n"; return RET_FAILED;}
	int Size;
	f >> Size;
	for(int i=0; i<Size; i++)
	{
		double val;
		f >> val;
		this->push_back(val);
	}
	return RET_OK;
}

unsigned long DblVector::Save(std::string Name, bool Append) const
{	
	std::ios_base::openmode m;
	if(Append) m = std::fstream::out | std::fstream::app;
	else m = std::fstream::out;
	std::ofstream f(Name.c_str(), m);
	if(!f.is_open()){std::cout << "DblVector::Save: Could not open: " << Name << "\n"; return RET_FAILED;}
	f << Str();
	return RET_OK;
}

void DblVector::WriteRandom(double Min, double Max, unsigned int Size, bool Int)
{
	if(Size>0)
	{
		clear();
		assign(Size, 0.0);
	}
	for(unsigned int i=0; i<size(); i++)
	{
		if(Int==true)
			(*this)[i] = (double)RandInt((int)Min, (int)Max);
		else 
			(*this)[i] = RandDbl(Min, Max);
	}
}

//void DblVector::AppendRandom(double Min, double Max, int Count, bool Int)
//{
//	assert(false); // function not tested
//	for(int i=0; i<Count; i++)
//	{
//		if(Int==true)
//			(*this)[i] = (double)RandInt((int)Min, (int)Max);
//		else (*this)[i] = RandDbl(Min, Max);
//	}
//}

void DblVector::MinMax(double& Min, double& Max, int& MinPos, int& MaxPos) const
{
	Min=DBL_MAX; Max=-DBL_MAX;
	for(unsigned int i=0; i<this->size(); i++)
	{
		double v=(*this)[i];
		if(v<Min) {Min=v; MinPos=i;}
		if(v>Max) {Max=v; MaxPos=i;}
	}
}

void DblVector::AddVec(const DblVector& B, DblVector& Res) const
{
	assert(this->size()==B.size());
	Res.clear();
	for(unsigned int i=0; i<this->size(); i++)
		Res.push_back((*this)[i] + B[i]);
}

void DblVector::AddToVec(const DblVector& B)
{
	assert(this->size()==B.size());
	for(unsigned int i=0; i<this->size(); i++)
		(*this)[i] += B[i];
}

void DblVector::SubVec(const DblVector& B, DblVector& Res) const
{
	assert(this->size()==B.size());
	Res.clear();
	for(unsigned int i=0; i<this->size(); i++)
		Res.push_back((*this)[i] - B[i]);
}

void DblVector::SubFromVec(const DblVector& B)
{
	assert(this->size()==B.size());
	for(unsigned int i=0; i<this->size(); i++)
		(*this)[i] -= B[i];
}

void DblVector::MulElem(const DblVector& B, DblVector& Res) const
{
	assert(this->size()==B.size());
	Res.clear();
	for(unsigned int i=0; i<this->size(); i++)
		Res.push_back((*this)[i]*B[i]);
}

void DblVector::MulElemTo(const DblVector& B)
{
	assert(this->size()==B.size());
	for(unsigned int i=0; i<this->size(); i++)
		(*this)[i] *= B[i];
}

void DblVector::ScalarMul(double s,  DblVector& Res) const
{
	Res.clear();
	for(unsigned int i=0; i<this->size(); i++)
		Res.push_back((*this)[i]*s);
}

void DblVector::ScalarMulTo(double s)
{
	for(unsigned int i=0; i<this->size(); i++)
		(*this)[i]*=s;
}

double DblVector::GetMagnitude() const
{
	return sqrt(GetMagnitudeSqr());
}

double DblVector::GetMagnitudeSqr() const
{
	double Sum=0.0;
	for(unsigned int i=0; i<this->size(); i++)
		Sum+=(*this)[i]*(*this)[i];
	return Sum;
}

//void DblVector::Normalize(DblVector& Res)
//{
	// implement if needed
//	assert(false);
//}

//void DblVector::Normalize()
//{
	// implement if needed
//	assert(false);
//}

double DblVector::EuclDist(const DblVector& B) const
{
	return sqrt(EuclDistSqr(B));
}

double DblVector::EuclDistSqr(const DblVector& B) const
{
	double Sum=0.0;
	for(unsigned int i=0; i<this->size(); i++)
		Sum+=((*this)[i]-B[i]) * ((*this)[i]-B[i]);
	return Sum;
}

std::string DblVector::ExportForPSTricks()
{
	std::stringstream s;
	for(int i=0; i<(int)size(); i++)
		s << "[" << i << " " << (*this)[i] << "]\n";
	return s.str();
}

int DblVector::SavePSTricks(std::string Name)
{
	setlocale(LC_NUMERIC, "C");
	std::ofstream f(Name.c_str());
	if(!f.is_open()) return RET_FAILED;
	f << ExportForPSTricks();
	return RET_OK;
}
//double DblVector::ScalarProduct(const DblVector& B)
//{
//	assert(false);
//	return 0.0;
//}

bool SortStructGreater(const SortStruct& s0, const SortStruct& s1)
{
	return s0.m_V>s1.m_V;
}

SortedVector::SortedVector(const DblVector& v)
{
	clear();
	for(unsigned int i=0; i<v.size(); i++)
		push_back(SortStruct(v[i], i));
	std::sort(begin(), end(), SortStructGreater);
}

void IntMatrix::Assign(int w, int h, int Val)
{
	IntVector v;

	// assign w times Val vector
	v.assign(w, Val);

	// assign h times v to the matrix
	for(int j=0; j<h; j++)
		this->push_back(v);
}

std::string IntMatrix::Str() const
{
	std::stringstream str;
	str << GetHeight() << std::endl;
	for(int i=0; i<GetHeight(); i++)
		str << (*this)[i].Str();
	return str.str();
}

unsigned long IntMatrix::Load(std::string Name, bool Append)
{
	if(!Append)	this->clear();
	std::ifstream f(Name.c_str(), std::fstream::in);
	if(!f.is_open()){std::cout << "Could not load: " << Name << "\n"; return RET_FAILED;}
	int Size;
	f >> Size;
	for(int j=0; j<Size; j++)
	{
		int Size2;
		f >> Size2;
		this->push_back(IntVector());
		for(int i=0; i<Size2; i++)
		{
			int val;
			f >> val;
			(*this)[j].push_back(val);
		}
	}
	return RET_OK;
}

unsigned long IntMatrix::Save(std::string Name, bool Append) const
{
	std::ios_base::openmode m;
	if(Append) m = std::fstream::out | std::fstream::app;
	else m = std::fstream::out;
	std::ofstream f(Name.c_str(), m);
	if(!f.is_open()){std::cout << "Could not open: " << Name << "\n"; return RET_FAILED;}
	f << Str();
	return RET_OK;
}

void IntMatrix::MakeMat(const IntVector& v)
{
	this->clear();
	this->push_back(v);
}

void IntMatrix::MakeVec(IntVector& v) const
{
	v.clear();
	for(int j=0; j<GetHeight(); j++)
		for(int i=0; i<GetWidth(); i++)
			v.push_back((*this)[j][i]);
}

void IntMatrix::WriteRandom(int Min, int Max)
{
	for(int j=0; j<GetHeight(); j++)
		for(int i=0; i<GetWidth(); i++)
			(*this)[j][i] = RandInt(Min, Max);
}

void IntMatrix::MinMax(int& Min, int& Max) const
{
	Min=INT_MAX; Max=-INT_MAX;
	for(int j=0; j<GetHeight(); j++)
		for(int i=0; i<GetWidth(); i++)
		{
			int v = (*this)[j][i];
			if(v<Min) Min=v; 
			if(v>Max) Max=v;
		}
}

void IntMatrix::EraseVector(IntVector& v)
{
	/*
	IntMatrix::iterator It=begin();
	for(;It!=end();)
	{
		if(*It==v) It=erase(v);
		else It++;
	}
	*/
}

void DblMatrix::Assign(int w, int h, double Val)
{
	DblVector v;
	v.assign(w, Val);
	for(int j=0; j<h; j++)
		this->push_back(v);
}

std::string DblMatrix::Str() const
{
	std::stringstream str;
	str << GetHeight() << std::endl;
	for(int i=0; i<GetHeight(); i++)
		str << (*this)[i].Str();
	return str.str();
}

unsigned long DblMatrix::Load(std::string Name, bool Append)
{
	if(!Append)	this->clear();
	std::ifstream f(Name.c_str(), std::fstream::in);
	if(!f.is_open()){std::cout << "DblMatrix::Load: Could not load '" << Name << "'!\n"; return RET_FAILED;}
	int Size;
	f >> Size;
	for(int j=0; j<Size; j++)
	{
		int Size2;
		f >> Size2;
		this->push_back(DblVector());
		for(int i=0; i<Size2; i++)
		{
			double val;
			f >> val;
			(*this)[j].push_back(val);
		}
	}
	return RET_OK;
}

unsigned long DblMatrix::Save(std::string Name, bool Append) const
{
	std::ios_base::openmode m;
	if(Append) m = std::fstream::out | std::fstream::app;
	else m = std::fstream::out;
	std::ofstream f(Name.c_str(), m);
	if(!f.is_open()){std::cout << "DblMatrix::Save: Could not open '" << Name << "'\n"; return RET_FAILED;}
	f << Str();
	return RET_OK;
}

void DblMatrix::MakeMat(const DblVector& v)
{
	this->clear();
	this->push_back(v);
}

void DblMatrix::MakeVec(DblVector& v) const
{
	v.clear();
	for(int j=0; j<GetHeight(); j++)
		for(int i=0; i<GetWidth(); i++)
			v.push_back((*this)[j][i]);
}

void DblMatrix::WriteRandom(double Min, double Max, bool Int)
{
	for(int j=0; j<GetHeight(); j++)
		for(int i=0; i<GetWidth(); i++)
		{
			if(Int) (*this)[j][i] = (double)RandInt((int)Min, (int)Max);
			else (*this)[j][i] = RandDbl(Min, Max);
		}
}

void DblMatrix::MinMax(double& Min, double& Max) const
{
	Min=DBL_MAX; Max=-DBL_MAX;
	for(int j=0; j<GetHeight(); j++)
		for(int i=0; i<GetWidth(); i++)
		{
			double v = (*this)[j][i];
			if(v<Min) Min=v; 
			if(v>Max) Max=v;
		}
}

void DblMatrix::GetLine(int j, DblVector& Line, int Step) const
{
	Line.clear();
	for(int i=0; i<(int)this->size(); i+=Step)
		Line.push_back((*this)[j][i]);
}

void DblMatrix::GetColumn(int i, DblVector& Column, int Step) const
{
	Column.clear();
	for(int j=0; j<(int)this->size(); j+=Step)
		Column.push_back((*this)[j][i]);
}

void DblMatrix::Diagonalize()
{
	assert(this->size() == (*this)[0].size()-1);
	for(int j=0; j<(int)this->size(); j++)
		for(int i=1; i<(int)(*this)[0].size()-1; i++)
			(*this)[i][j]=(*this)[j][i];
}

// determinants
double DblMatrix::GetDeterminant2x2()
{
	assert(GetHeight()==2 && GetWidth()==2); 
	return (*this)[0][0]*(*this)[1][1]-(*this)[0][1]*(*this)[1][0];
}
double DblMatrix::GetDeterminant3x3()
{
	assert(GetHeight()==3 && GetWidth()==3); 
	return (*this)[0][0]*(*this)[1][1]*(*this)[2][2]
			+(*this)[0][1]*(*this)[1][2]*(*this)[2][0]
			+(*this)[0][2]*(*this)[1][0]*(*this)[2][1]
			-(*this)[0][2]*(*this)[1][1]*(*this)[2][0]
			-(*this)[0][1]*(*this)[1][0]*(*this)[2][2]
			-(*this)[0][0]*(*this)[1][2]*(*this)[2][1];
}
double DblMatrix::GetDeterminantNxN()
{
	int w=GetWidth();
	assert(w=GetHeight());
	if(w==1) return (*this)[0][0];
	if(w==2) return GetDeterminant2x2();
	if(w==3) return GetDeterminant3x3();
	double Det=0.0;
	// laplace's method (recursion)
	double s=1.0;
	if(w%2==1) s=-1.0;
	for(int i=0; i<w; i++)
	{
		
		DblMatrix SubDet;
		SubDet.assign(w-1, DblVector());
		for(int j=1; j<w; j++)
		{
			for(int k=0; k<w; k++)
			{
				if(k==i)
				{
					continue;
				}
				else
				{
					SubDet[j-1].push_back((*this)[j][k]);
				}
			}
		}
		double v = (*this)[0][i];
		
		double a = s * v * SubDet.GetDeterminantNxN();
		//std::cout << "s " << s << std::endl;
		Det += a;
		s*=-1.0;
	}
	return Det;
}

// sub-determinant
double DblMatrix::GetHelpDeterminant(int k)
{
	int h=GetHeight();
	assert(h+1==GetWidth());
	DblMatrix HelpDet;
	// replace the
	for(int i=0; i<h; i++)
	{
		HelpDet.push_back(DblVector());
		for(int j=0; j<h; j++)
		{
			if(j==k)
				HelpDet[i].push_back((*this)[i][h]);
			else
				HelpDet[i].push_back((*this)[i][j]);
		}
	}
	return HelpDet.GetDeterminantNxN();
}

// solve linear NxN equation system (cramer)
int DblMatrix::SolveLinearCramer(DblVector& Solution)
{
	// get main determinant
	DblMatrix DetMat;
	double Det=0.0;
	int w = GetWidth();
	int h = GetHeight();
	for(int i=0; i<h; i++)
	{
		DetMat.push_back(DblVector());
		for(int j=0; j<w-1; j++)
			DetMat[i].push_back((*this)[i][j]);
	}
	Det = DetMat.GetDeterminantNxN();
	if(fabs(Det)<THE_EPS_DEF) return 0;
	// get solutions
	Solution.clear();
	for(int k=0; k<h; k++)
		Solution.push_back(this->GetHelpDeterminant(k)/Det);

	return 1;
}


void DblMatrix::SaveAsBMP(std::string Name)
{
	double Min=DBL_MAX, Max=-DBL_MAX;
	MinMax(Min, Max);
	IplImage* Output = cvCreateImage(cvSize(GetWidth(), GetHeight()), IPL_DEPTH_8U, 1); 
	for(int j=0; j<GetHeight(); j++)
		for(int i=0; i<GetWidth(); i++)
		{
			double u = (*this)[j][i];
			double v = 255.0 * (u-Min)/(Max-Min);
			cvSetReal2D(Output, j, i, v);
		}
	cvSaveImage(Name.c_str(), Output);
	cvReleaseImage(&Output);
}

void DblMatrix::SaveHistogram(std::string Name, int NoBuckets)
{
	double Min=DBL_MAX, Max=-DBL_MAX;
	MinMax(Min, Max);
}

std::string DblPtrMat::Str() const
{
	std::string Str;
	for(unsigned int i=0; i<this->size(); i++)
		Str += (*this)[i]->Str();
	return Str;
}

void DblPtrMat::ImportFromDblMatrix(DblMatrix& m)
{
	for(int i=0; i<m.GetHeight(); i++)
		this->push_back(&(m[i]));
}

IntTensor::IntTensor(int Width, int Height, int Depth)
{
	for(int k=0; k<Depth; k++)
		push_back(IntMatrix(Width, Height, 0));
}

/// regression functions

// y = ax + b, x = 0, ..., Points.size()
int FitLine(const DblVector& PointsY, double& a, double& b)
{
	DblVector PointsX;
	for(unsigned int i=0; i<PointsY.size(); i++)
			PointsX.push_back((double)i);
	return FitLine(PointsY, PointsX, a, b);
}

// x given
int FitLine(const DblVector& PointsY, const DblVector& PointsX, double& a, double& b)
{
	assert(PointsX.size()==PointsY.size());
	DblMatrix S;
	S.Assign(3, 2, 0.0);
	for(unsigned int i=0; i<PointsX.size(); i++)
	{
		S[0][0] += PointsX[i] * PointsX[i];
		S[0][1] += PointsX[i];
		S[0][2] += PointsX[i] * PointsY[i];
		S[1][2] += PointsY[i]; 
	}
	S[1][1] = (double)PointsX.size();
	S.Diagonalize();
	DblVector Solutions;
	int Ret = S.SolveLinearCramer(Solutions);
	if(Ret) 
	{
		a = Solutions[0];
		b = Solutions[1];
	}
	else std::cout << "Error in line fitting" << std::endl;
	
	return Ret;
}

int FitLineIt(const DblVector& PointsY, const DblVector& PointsX,
			  double& a, double& b, int It, double ErrWidth)
{
	// base case
	if(It==0) return 0;
	std::cout << "Line fitting no samples: " << PointsY.size() << std::endl;

	// fitting
	FitLine(PointsY, PointsX, a, b);

	double Error = 0.0;
	for(int i=0; i<(int)PointsY.size(); i++)
	{
		double Val0 = a*PointsX[i]+b;
		double Val1 = PointsY[i]; 
		Error += dblsqr(Val1-Val0);
		//std::cout << Val1 << " " << Val0 << std::endl;
	}
	Error /= (double)PointsY.size();
	Error = sqrt(Error);

	DblVector px, py; int Cnt=0;
	for(int k=0; k<(int)PointsX.size(); k++)
	{
		double Val = a*PointsX[k]+b;
		double Dist = sqrt(dblsqr(PointsY[k] - Val));
		if(Dist<Error*ErrWidth)
		{
			px.push_back(PointsX[k]);
			py.push_back(PointsY[k]);
			Cnt++;
		}
	}
	return FitLineIt(py, px, a, b, It-1, ErrWidth);
}

unsigned long FitPolynomial(const DblVector& y, const DblVector& x, int degree, double* coefficients)
{

	if(y.size() != x.size())
	{
		std::cerr << "ipa_Utils::FitPolynomial: Error" << std::endl;
		std::cerr << "\t ... vector 'y' and vector 'x' must have same size." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	/// Compute normalized polynomial coefficients through least squares fitting
	double* c;
	c = Wm4::PolyFit2<double>((int)x.size(), &x[0], &y[0], degree);

	for (int i = 0; i <= degree; i++)
	{
		coefficients[i] = c[i];
	}
	delete[] c;

	return ipa_Utils::RET_OK;
}

unsigned long EvaluatePolynomial(double x, int degree, double* coefficients, double* y)
{
	(*y) = coefficients[degree];
	for (int i = degree-1; i >= 0; i--)
	{
		(*y) *= x;
		(*y) += coefficients[i];
	}

	return ipa_Utils::RET_OK;
}

unsigned long FitNormalizedPolynomial(const DblVector& y, const DblVector& x, int degree, double* coefficients, double* max, double* min)
{

	if(y.size() != x.size())
	{
		std::cerr << "ipa_Utils::FitPolynomial: Error" << std::endl;
		std::cerr << "\t ... vector 'y' and vector 'x' must have same size." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	/// Extract min and max from 32bit distance values
	/// to scale x-values
	(*min)=DBL_MAX;
	(*max)=-DBL_MAX;
	for(unsigned int k=0; k<x.size(); k++)
	{
		double d = x[k];
		if(d < (*min)) (*min)=d;
		if(d > (*max)) (*max)=d;
	}
	double diffMaxMin = (*max) - (*min);

	/// Normalize data points, to assert numeric stability during Matrix inversion
	std::vector<double> y_Normalized;
	std::vector<double> x_Normalized;
	for(unsigned int k=0; k<x.size(); k++)
	{
		x_Normalized.push_back((x[k]-(*min))/diffMaxMin); 
		y_Normalized.push_back(y[k]/diffMaxMin);
	}

	/// Compute normalized polynomial coefficients through least squares fitting
	double* c;
	c = Wm4::PolyFit2<double>((int)x.size(), &x_Normalized[0], &y_Normalized[0], degree);

	for (int i = 0; i <= degree; i++)
	{
		coefficients[i] = c[i];
	}

	delete[] c;

	return ipa_Utils::RET_OK;
}

unsigned long EvaluateNormalizedPolynomial(double x, int degree, double* coefficients, double min, double max, double* y)
{
	double u = (x - min)/(max - min);
	double v = coefficients[degree];
	for (int i = degree-1; i >= 0; i--)
	{
		v *= u;
		v += coefficients[i];
	}
	(*y) = (max - min)*v;

	return ipa_Utils::RET_OK;
}

// y = ax^2+bx+c
int FitParabola(const DblVector& PointsY, double& a, double& b, double& c)
{
	DblVector PointsX;
	for(unsigned int i=0; i<PointsY.size(); i++)
			PointsX.push_back((double)i);
	return FitParabola(PointsY, PointsX, a, b, c);
}

// x given 
int FitParabola(const DblVector& PointsY, const DblVector& PointsX, double& a, double& b, double& c)
{
	// fit parabola
	assert(PointsX.size()==PointsY.size());
	DblMatrix S;
	S.Assign(4, 3, 0.0);
	for(unsigned int i=0; i<PointsX.size(); i++)
	{
		S[0][0] += PointsX[i] * PointsX[i] * PointsX[i] * PointsX[i];
		S[0][1] += PointsX[i] * PointsX[i] * PointsX[i];
		S[0][2] += PointsX[i] * PointsX[i];
		S[0][3] += PointsX[i] * PointsX[i] * PointsY[i];
		S[1][2] += PointsX[i];
		S[1][3] += PointsX[i] * PointsY[i];
		S[2][3] += PointsY[i];
		
	}
	S[1][1] = S[0][2];
	S[2][2] = (double)PointsX.size();
	S.Diagonalize();
	DblVector Solutions;
	int Ret = S.SolveLinearCramer(Solutions);
	if(Ret==-1) return -1;
	else
	{
		a = Solutions[0];
		b = Solutions[1];
		c = Solutions[2];
	}
	return Ret;
}

// fit a two dimensional quadratic surface
int FitQuadraticSurface(const DblVector& PX, const DblVector& PY,
							const DblVector& PZ, DblVector& Parameters)
{
	// fit parabola
	assert(PX.size()==PY.size());
	DblMatrix S;
	S.Assign(7, 6, 0.0);
	for(unsigned int i=0; i<PX.size(); i++)
	{
		double X2 = PX[i] * PX[i];
		double Y2 = PY[i] * PY[i];
		double X3 = X2 * PX[i];
		double Y3 = Y2 * PY[i];
		double X4 = X3 * PX[i];
		double Y4 = Y3 * PY[i];
		
		S[0][1] += PY[i];
		S[1][1] += Y2;
		
		S[0][2] += PX[i];
		S[1][2] += PX[i]*PY[i];
		S[2][2] += X2;

		S[0][3] += PX[i]*PY[i];
		S[1][3] += PX[i]*Y2;
		S[2][3] += X2*PY[i];
		S[3][3] += X2*Y2;

		S[0][4] += Y2;
		S[1][4] += Y3;
		S[2][4] += PX[i]*Y2;
		S[3][4] += PX[i]*Y3;
		S[4][4] += Y4;

		S[0][5] += X2;
		S[1][5] += X2*PY[i];
		S[2][5] += X3;
		S[3][5] += X3*PY[i];
		S[4][5] += X2*Y2;
		S[5][5] += X4;

		// function values
		S[0][6]+=PZ[i];
		S[1][6]+=PY[i]*PZ[i];
		S[2][6]+=PX[i]*PZ[i];
		S[3][6]+=PX[i]*PY[i]*PZ[i];
		S[4][6]+=Y2*PZ[i];
		S[5][6]+=X2*PZ[i];

	}
	S[0][0] = (double)PX.size();
	S.Diagonalize();
	return S.SolveLinearCramer(Parameters);
}

double GetQuadraticSurfaceError(const DblVector& PX, const DblVector& PY,
							const DblVector& PZ, DblVector& Parameters)
{
	double Sum=0.0;
	for(int i=0; i<(int)PX.size(); i++)
	{
		double Val = QuadraticSurfaceVal(PX[i], PY[i], Parameters);
		Sum += (PZ[i] - Val) * (PZ[i] - Val);
	}
	return sqrt(Sum/(double)PX.size());
}

int FitQuadraticSurfaceIt(const DblVector& PX, const DblVector& PY,
							const DblVector& PZ, DblVector& Parameters,
							int It, double ErrWidth)
{

	// base case
	if(It==0) return 0;
	std::cout << "Quad surface fitting no samples: " << PX.size() << std::endl;

	// fitting
	FitQuadraticSurface(PX, PY, PZ, Parameters);
	double Error = GetQuadraticSurfaceError(PX, PY, PZ, Parameters);
	
	DblVector px, py, pz; int Cnt=0;
	for(int k=0; k<(int)PX.size(); k++)
	{
		double Val = QuadraticSurfaceVal(PX[k], PY[k], Parameters);
		double Dist = sqrt((PZ[k] - Val) * (PZ[k] - Val));
		if(Dist<Error*ErrWidth)
		{
			px.push_back(PX[k]);
			py.push_back(PY[k]);
			pz.push_back(PZ[k]);
			Cnt++;
		}
		
	}
	return FitQuadraticSurfaceIt(px, py, pz, Parameters, It-1, ErrWidth);
}

void IntToCol(int i, int& r, int& g, int& b)
{
	int high=255, low0 = 0, low1 = 100;
	if(i==-1) {r=0; g=0; b=255; return;}
	
	int I = i % 11;
	if(I==0) {r=low0; g=high; b=low0; return;}
	if(I==1) {r=high; g=low0; b=low0; return;}
	if(I==2) {r=low0; g=high; b=high; return;}
	if(I==3) {r=high; g=low0; b=high; return;}
	if(I==4) {r=high; g=high; b=low0; return;}
	if(I==5) {r=high; g=high; b=high; return;}

	if(I==6) {r=low1; g=high; b=low1; return;}
	if(I==7) {r=high; g=low1; b=low1; return;}
	if(I==8) {r=low1; g=high; b=high; return;}
	if(I==9) {r=high; g=low1; b=high; return;}
	if(I==10) {r=high; g=high; b=low1; return;}
	//if(I==11) {r=high; g=high; b=high; return;}
}

void HueToCol(int h, int hueVals, int& r, int& g, int& b)
{
	double H = (double)h/(double)hueVals;
	
	double R, G, B;
	GetRGB(H, 0.9, 0.55, R, G, B);
	r = cvRound(R*255);
	g = cvRound(G*255);
	b = cvRound(B*255);
}

void GetHSV(double r, double g, double b, double& h, double& s, double& v)
{
	v=dblmax(r, dblmax(g, b));
	double d=v-dblmin(r, dblmin(g, b));
	if(d==0.0) {s=0.0; h=0.0; /*std::cerr << "Error in HSV conversion\n";*/ return;}
	s=d/v;
	if(r==v)
	{
		if(g>=b)
			h = 0.167 * (g-b)/d;
		else
			h = 0.167 * (g-b)/d + 1.0;
	}
	if(g==v) h = 0.167 * (b-r)/d + 0.333;
	if(b==v) h = 0.167 * (r-g)/d + 0.667;
	v/=255.0;
}

void GetRGB(double h, double s, double v, double& r, double& g, double& b)
{
	/// from Wikipedia
	
	double H = h * 5.99;
	double hi = floor(H);
	double f = H-hi;
	double p = v * (1-s);
	double q = v * (1-s*f);
	double t = v * (1-s * (1-f));
	
	if(hi==0) { r = v; g = t; b = p; }
	if(hi==1) { r = q; g = v; b = p; }
	if(hi==2) { r = p; g = v; b = t; }
	if(hi==3) { r = p; g = q; b = v; }
	if(hi==4) { r = t; g = p; b = v; }
	if(hi==5) { r = v; g = p; b = q; }
	
}

double InterpolateExtr(double y1, double y2, double y3, double x2, double h)
{
	/// generated from MAPLE code generation
	double x_max = ((-h * y3 + y1 * h - 4 * x2 * y2 + 2.0 * x2 * y3 + 2.0 * x2 * y1)
		/ (-2.0 * y2 + y3 + y1)) / 0.2e1;
	return x_max;
}

void GetVectorFromImage(IplImage* Img, DblVector& v, CvScalar* Scale)
{
	for (int c=0; c<Img->nChannels; c++)
	{
		for (int j=0; j<Img->height; j++)
		{
			for (int i=0; i<Img->width; i++)
			{
				CvScalar Val = cvGet2D(Img, j, i);
				if(Scale!=NULL)
					Val.val[c] /= Scale->val[c];
				v.push_back(Val.val[c]);
			}
		}
	}
}

} // end namespace ipa_Utils

