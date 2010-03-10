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
* ROS package name: cob_vision_utils
* Description: Utility functions for 3D operations.
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Author: Jan Fischer, email:jan.fischer@ipa.fhg.de
* Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
*
* Date of creation: July 2008
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

/// @file ThreeDUtils.cpp
/// Implementations of the classes/functions declared and documented in ThreeDUtils.h

#ifdef __COB_ROS__
#include "cob_vision_utils/ThreeDUtils.h"
#else
#include "ThreeDUtils.h"
#endif

namespace ipa_Utils {

void Point3Dbl::ToSphere()
{
	double r=AbsVal();
	double Theta = acos(m_z/r);
	double Phi = atan2(m_y, m_x);
	m_x = r; m_y = Theta; m_z = Phi;
}

void Point3Dbl::ToSphereNormalized()
{
	double Theta = acos(m_z); // Angle between positive z axis and vector
	double Phi = atan2(m_y, m_x); // Angle between positive x axis and vector projected on the xy-plane
	m_x = 1.0; m_y = Theta; m_z = Phi;
}

void Point3Dbl::ToCart()
{
	double SinT = sin(m_y);
	double CosT = cos(m_y);
	double SinP = sin(m_z);
	double CosP = cos(m_z);
	
	m_x = SinT*CosP;
	m_y = SinT*SinP;
	m_z = CosT;

	double r = AbsVal();
	m_x /= r;
	m_y /= r;
	m_z /= r;
}

void Point3Dbl::ToCartNormalized(double Theta, double Phi)
{
	double SinT = sin(Theta);
	double CosT = cos(Theta);
	double SinP = sin(Phi);
	double CosP = cos(Phi);
	
	m_x = SinT*CosP;
	m_y = SinT*SinP;
	m_z = CosT;
}

void Point3Dbl::Projection(const Point3Dbl& B)
{
	double Cos = GetCosine(B);
	(*this)=B;
	ScalarMul(Cos);
}

double Point3Dbl::GetLengthOrthProjection(const Point3Dbl& B)
{
	return AbsVal()*ScalarProd(B)/ScalarProd(*this);
}

void Point3Dbl::OrthProjection(const Point3Dbl& B)
{
	double s = GetLengthOrthProjection(B);
	Normalize();
	ScalarMul(s);
}

void Point3Dbl::OrthProjectionDiffVector(const Point3Dbl& B)
{
	Point3Dbl T=*this;
	T.OrthProjection(B);
	SubVec(B, T);
}


void RotateVector(const Point3Dbl& RotaNormal, double Phi, const Point3Dbl& A, Point3Dbl& Res)
{	
	/// Compute rodrigues' formula
	double CosPhi = cos(Phi);
	double s = A.ScalarProd(RotaNormal);
	Point3Dbl V;
	V.VectorProd(RotaNormal, A);
	Point3Dbl T1, T2, T3;
	T1.ScalarMul(CosPhi, A);
	T2.ScalarMul(s*(1.0-CosPhi), RotaNormal);
	T3.ScalarMul(sin(Phi), V);
	T2.AddVec(T1);
	Res.AddVec(T2, T3);
}

int GetRotation(const Point3Dbl& A1, const Point3Dbl& B1, 
				const Point3Dbl& A2, const Point3Dbl& B2,
				Point3Dbl& RotaNormal, double& Phi)
{
	/// Compute the difference vectors
	Point3Dbl dA, dB;
	dA.SubVec(A2, A1);
	dB.SubVec(B2, B1);
	
	/// Compute the direction of rotation (filter small rotations)
	Point3Dbl R;
	R.VectorProd(dA, dB);
	if(R.AbsVal()<THE_EPS_DEF)
	{
		Point3Dbl U0, U1, D;
		U0.VectorProd(A1, B1);
		U1.VectorProd(A2, B2);
		D.AddVec(U0, U1);
		if(D.AbsVal()>THE_EPS_DEF)
		{
			RotaNormal=Point3Dbl(0,0,1);
			Phi=0;
		}
		else
		{
			RotaNormal.VectorProd(U0,dA);
			RotaNormal.Normalize();
			Phi=THE_PI_DEF-THE_EPS_DEF;
		}
		return RET_OK;
	}
	R.Normalize();

	/// Get the projections of A1, A2 on the plane orthogonal to RotaNormal.
	/// See Rodrigues' formula on en.wikipedia.org
	double A1r = A1.ScalarProd(R);
	double A2r = A2.ScalarProd(R);
	Point3Dbl pA1, pA2, sA1r, sA2r;
	sA1r.ScalarMul(A1r, R);
	sA2r.ScalarMul(A2r, R);
	pA1.SubVec(A1, sA1r);
	pA2.SubVec(A2, sA2r);
	
	/// Get the angle between pA1 and pA2 and correct
	Point3Dbl V;
	V.VectorProd(pA1, pA2);
	double vAbs = V.AbsVal();
	Phi = atan2(vAbs, pA1.ScalarProd(pA2));
	RotaNormal.Normalize(V);
	
	/// Take only positive angles
	if(Phi<0.0)
	{
		Phi = -Phi;
		RotaNormal.Negative();
	}
	return RET_OK;
}

double GetRotationDistance(Point3Dbl& RotaNormal0, double Alpha0, Point3Dbl& RotaNormal1, double Alpha1)
{
	/// Get a point orthogonal to both rotation vectors
	Point3Dbl Orth;
	Orth.VectorProd(RotaNormal0, RotaNormal1);
	Orth.Normalize();

	/// Turn the point according to the two rotations
	Point3Dbl p0, p1;
	RotateVector(RotaNormal0, Alpha0, Orth, p0);
	RotateVector(RotaNormal1, Alpha1, Orth, p1);
	
	/// Return the difference angle between the two rotated points
	return p0.GetAngle(p1);
}

void GetRotationMean(Point3Dbl& RotaNormal0, double Alpha0, Point3Dbl& RotaNormal1, double Alpha1, Point3Dbl& RotaNormal2, double& Alpha2, double w0, double w1)
{
	/// Get a point orthogonal to both rotation vectors
	Point3Dbl Orth;
	Orth.VectorProd(RotaNormal0, RotaNormal1);
	Orth.Normalize();

	/// Turn the point according to the two rotations
	Point3Dbl p0, p1;
	RotateVector(RotaNormal0, Alpha0, Orth, p0);
	RotateVector(RotaNormal1, Alpha1, Orth, p1);

	/// Get the center point
	Point3Dbl c;
	c.SubVec(p1,p0);
	double d = p0.GetDistance(p1);
	double a = 1.0-(w0/(w0+w1));
	c.ScalarMul(d*a);
	c.AddVec(p0);
	c.Normalize();

	/// Find a rotation that moves to the center directly
	RotaNormal2.VectorProd(Orth, c);
	RotaNormal2.Normalize();

	/// Estimate the angle
	Alpha2 = Orth.GetAngle(c);

	/// Check
	//Point3Dbl test;
	//RotateVector(RotaNormal2, Alpha2, Orth, test);
	//assert(test.GetDistance(c)<0.01);
}

void GetBasePoint(const Point3Dbl& S0, const Point3Dbl& S1, const Point3Dbl& Q, Point3Dbl& X)
{
	Point3Dbl S01; S01.SubVec(S1, S0);
	Point3Dbl S0Q; S0Q.SubVec(Q, S0);
	double SProd = S01.ScalarProd(S0Q)/S01.AbsValSqr();
	X.ScalarMul(SProd, S01);
	X.AddVec(S0);
}

double GetLineDistance(const Point3Dbl& S0, const Point3Dbl& S1, const Point3Dbl& Q)
{
	Point3Dbl Base;
	GetBasePoint(S0, S1, Q, Base);
	Point3Dbl F;
	F.SubVec(Base, S0);
	double a = (Base.m_x-S0.m_x)/(S1.m_x-S0.m_x);
	if(a<=0.0)
		return Q.GetDistance(S0);
	else if(a>=1.0)
		return Q.GetDistance(S1);
	else return Q.GetDistance(Base);
}

bool thisGreaterOD2(const CountsStruct& e1, const CountsStruct& e2)
{
	return e1.Counts > e2.Counts;
}

void PointCloud::MakeBase(double Unit)
{
	clear();
	push_back(Point3Dbl(0,0,0));
	push_back(Point3Dbl(Unit,0,0));
	push_back(Point3Dbl(0,Unit,0));
}


void PointCloud::AssignPoints(const Point3Dbl& P, int No)
{
	clear();
	for(int i=0; i<No; i++)
		push_back(P);
}

void PointCloud::AssignRand(double Min, double Max, int No, bool Append, const Point3Dbl& Mask)
{	
	if(!Append) clear();
	for(int i=0; i<No; i++)
	{
		Point3Dbl p;
		p.m_x = Mask.m_x * RandDbl(Min, Max);
		p.m_y = Mask.m_y * RandDbl(Min, Max);
		p.m_z = Mask.m_z * RandDbl(Min, Max);
		push_back(p);
	}
}


void PointCloud::AddRand(double Min, double Max, const Point3Dbl& Mask)
{
	for(int i=0; i<(int)size(); i++)
	{
		Point3Dbl Rand = Point3Dbl(Mask.m_x * RandDbl(Min, Max),
						Mask.m_y * RandDbl(Min, Max),
						Mask.m_z * RandDbl(Min, Max));
		(*this)[i].AddVec(Rand);
	}
}

void PointCloud::ImportCloud(const PointCloud& PC, bool Append)
{
	if(!Append) clear();
	for(unsigned int i=0; i<PC.size(); i++)
		push_back(PC[i]);
}
 
std::string PointCloud::Str() const
{
	setlocale(LC_NUMERIC, "C");
	std::stringstream s;
	s << size() << "\n";
	for(unsigned int i=0; i<size(); i++)
		s << (*this)[i].m_x << " " << (*this)[i].m_y << " " << (*this)[i].m_z << "\n";
	return s.str();
}

int PointCloud::Load(std::string Name)
{
	/// open file
	std::ifstream f(Name.c_str());
	if(!f.is_open())
	{
		std::cout << "Error: could not load " << Name << std::endl;
		return RET_FAILED;
	}
	
	/// loop over the data and read in
	int Size=0;
	f >> Size;
	for(int i=0; i<Size; i++)
	{	
		Point3Dbl p;
		f >> p.m_x;
		f >> p.m_y;
		f >> p.m_z;
		push_back(p);
	}

	return RET_OK;
}

int PointCloud::Save(std::string Name) const
{
	DblMatrix Mat;
	Mat.Assign(3, (int)this->size());
	for(int i=0; i<(int)this->size(); i++)
	{	
		Mat[i][0]=(*this)[i].m_x;
		Mat[i][1]=(*this)[i].m_y;
		Mat[i][2]=(*this)[i].m_z;
	}
	return Mat.Save(Name);
}

void PointCloud::VectorSum(Point3Dbl& Sum) const
{
	Sum = Point3Dbl(0,0,0);
	for(unsigned int i=0; i<size(); i++)
		Sum.AddVec((*this)[i]);
}

void PointCloud::GetCenter(Point3Dbl& Center) const
{
	VectorSum(Center);
	Center.ScalarMul(1.0/(double)size());
}

void PointCloud::GetOrientations(Point3Dbl& OriX, Point3Dbl& OriY) const
{
	/// Get X orientation
	Point3Dbl DX, SumX(0,0,0);
	unsigned int i,j,c=0;
	for(j=0; j<size()-1; j++)
	{
		for(i=j+1; i<size(); i++)
		{
			DX.SubVec((*this)[i], (*this)[j]);
			SumX.AddVec(DX);
			c++;
		}
	}
	SumX.ScalarMul(1.0/(double)c);
	double a = SumX.AbsVal();
	if(a<THE_EPS_DEF) OriX=Point3Dbl(1,0,0);
	else OriX.ScalarMul(1.0/a, SumX);

	/// Get Y orientation
	Point3Dbl DY, SumY(0,0,0);
	for(i=0; i<size(); i++)
	{
		DY=OriX;
		DY.OrthProjectionDiffVector((*this)[i]);
		SumY.AddVec(DY);
	}
	SumY.ScalarMul(1.0/size());
	a = SumY.AbsVal();
	if(a<THE_EPS_DEF) OriY=Point3Dbl(0,1,0);
	else OriY.ScalarMul(1.0/a, SumY);
}

double PClDistSqr(const PointCloud& A, const PointCloud& B)
{
	double Sum =0.0;
	for(unsigned int i=0; i<A.size(); i++)
	{
		double Err = A[i].GetDistance(B[i]); 
		Sum += Err * Err; 
	}
	return Sum/(double)A.size();
}

double PClDist(const PointCloud& A, const PointCloud& B)
{
	return sqrt(PClDistSqr(A, B));
}

Frame::Frame(double tx, double ty, double tz, double rx, double ry, double rz)
{
	this->push_back(tx);
	this->push_back(ty);
	this->push_back(tz);
	this->push_back(rx);
	this->push_back(ry);
	this->push_back(rz);
}

std::string Frame::Str() const
{
	std::stringstream s;
	s << (*this)[0] << " ";
	s << (*this)[1] << " ";
	s << (*this)[2] << " ";
	s << (*this)[3] << " ";
	s << (*this)[4] << " ";
	s << (*this)[5];
	return s.str();
}

void Frame::SetTranslation(const Point3Dbl& Trans)
{
	(*this)[0] = Trans.m_x;
	(*this)[1] = Trans.m_y;
	(*this)[2] = Trans.m_z;
}

void Frame::SetTranslation(double tx, double ty, double tz)
{
	(*this)[0] = tx;
	(*this)[1] = ty;
	(*this)[2] = tz;
}

void Frame::SetRotation(const Point3Dbl& Rot)
{
	(*this)[3] = Rot.m_x;
	(*this)[4] = Rot.m_y;
	(*this)[5] = Rot.m_z;
}

void Frame::SetRotation(double rx, double ry, double rz)
{
	(*this)[3] = rx;
	(*this)[4] = ry;
	(*this)[5] = rz;
}

int Frame::MakeFrameOriginDirectionsXY(const Point3Dbl& O, const Point3Dbl& EX, const Point3Dbl& EY)
{
	/// Apply Gram-Schmidt orthogonalization
	Point3Dbl x0, x1;
	x0.Normalize(EX);
	double s01 = EY.ScalarProd(EX);
	double s0 = EX.ScalarProd(EX);
	if(fabs(s0)<THE_EPS_DEF) return RET_FAILED;
	Point3Dbl n0; n0.ScalarMul(s01/s0, EX);
	x1.SubVec(EY, n0);
	x1.Normalize();
	
	/// Set/get translation and rotation
	SetTranslation(O);
	Point3Dbl RotaNormal; double Alpha=0.0;
	if(GetRotation(Point3Dbl(1,0,0), Point3Dbl(0,1,0), x0, x1, RotaNormal, Alpha) & RET_FAILED)
		return RET_FAILED;
	RotaNormal.ToSphereNormalized();
	(*this)[3] = Alpha;
	(*this)[4] = RotaNormal.m_y; // Theta
	(*this)[5] = RotaNormal.m_z; // Phi
	return RET_OK;
}

int Frame::MakeFrameOriginDirectionsXZ(const Point3Dbl& O, const Point3Dbl& EX, const Point3Dbl& EZ)
{
	/// Apply Gram-Schmidt orthogonalization
	Point3Dbl x0, x1;
	x0.Normalize(EX);
	double s01 = EZ.ScalarProd(EX);
	double s0 = EX.ScalarProd(EX);
	if(fabs(s0)<THE_EPS_DEF) return RET_FAILED;
	Point3Dbl n0; n0.ScalarMul(s01/s0, EX);
	x1.SubVec(EZ, n0);
	x1.Normalize();
	
	/// Set/get translation and rotation
	SetTranslation(O);
	Point3Dbl RotaNormal; double Alpha=0.0;
	if(GetRotation(Point3Dbl(1,0,0), Point3Dbl(0,0,1), x0, x1, RotaNormal, Alpha) & RET_FAILED)
		return RET_FAILED;
	RotaNormal.ToSphereNormalized();
	(*this)[3] = Alpha;
	(*this)[4] = RotaNormal.m_y; // Theta
	(*this)[5] = RotaNormal.m_z; // Phi
	return RET_OK;
}

int Frame::MakeFrameThreePoints(const Point3Dbl& p0, const Point3Dbl& p1, const Point3Dbl& p2)
{
	/// Test for linear independence using the peak angle of p1-p0 and p2-p0
	Point3Dbl v0, v1;
	v0.SubVec(p1, p0); v1.SubVec(p2, p0);
	double phi = v0.GetAngle(v1); 
	if(phi < THE_EPS_DEF) return RET_FAILED; 
	
	/// Call MakeFrameOriginDirections()
	return MakeFrameOriginDirectionsXY(p0, v0, v1);
}

void Frame::SetRandomTranslation(double Bound)
{
	(*this)[0]=RandDbl(-Bound, Bound);
	(*this)[1]=RandDbl(-Bound, Bound);
	(*this)[2]=RandDbl(-Bound, Bound);
}

void Frame::SetRandomRotation(double Factor)
{
	//double Alpha = RandDbl(-THE_PI_DEF, THE_PI_DEF) * Factor;
	double Alpha = RandDbl(0.0, THE_PI_DEF) * Factor;
	double Theta = RandDbl(0.0, THE_PI_DEF) * Factor;
	double Phi = RandDbl(-THE_PI_DEF+THE_EPS_DEF, THE_PI_DEF) * Factor;
	(*this)[3]=Alpha;
	(*this)[4]=Theta;
	(*this)[5]=Phi;
}

void Frame::SetRandomFrame(double Bound, double Factor)
{
	SetRandomTranslation(Bound);
	SetRandomRotation(Factor);
}

void Frame::GetT(Point3Dbl& T) const
{
	T.m_x = (*this)[0];
	T.m_y = (*this)[1];
	T.m_z = (*this)[2];
}

void Frame::GetR(Point3Dbl& R) const
{
	R.m_x = (*this)[3];
	R.m_y = (*this)[4];
	R.m_z = (*this)[5];
}

void Frame::ToWorld(const Point3Dbl& In, Point3Dbl& Out) const
{
	/// Rotate	
	Point3Dbl nr;
	nr.ToCartNormalized((*this)[4], (*this)[5]); 
	RotateVector(nr, (*this)[3], In, Out);

	/// Shift
	Out.m_x += (*this)[0];
	Out.m_y += (*this)[1];
	Out.m_z += (*this)[2];
}


void Frame::ToWorld(Point3Dbl& P) const
{
	Point3Dbl P1;
	ToWorld(P, P1);
	P = P1;
}

void Frame::ToWorld(double& x, double& y, double& z) const
{
	Point3Dbl P = Point3Dbl(x, y, z);
	ToWorld(P);
	x = P.m_x; y = P.m_y; z=P.m_z;
}

void Frame::ToFrame(const Point3Dbl& In, Point3Dbl& Out) const
{
	Out.m_x = In.m_x-(*this)[0];
	Out.m_y = In.m_y-(*this)[1];
	Out.m_z = In.m_z-(*this)[2];

	Point3Dbl nr;
	nr.ToCartNormalized((*this)[4], (*this)[5]); 
	RotateVector(nr, -(*this)[3], Out, Out);
}

void Frame::ToFrame(Point3Dbl& P) const
{
	Point3Dbl P1;
	ToFrame(P, P1);
	P = P1;
}

void Frame::ToFrame(double& x, double& y, double& z) const
{
	Point3Dbl P = Point3Dbl(x, y, z);
	ToFrame(P);
	x = P.m_x; y = P.m_y; z=P.m_z;
}

void Frame::Cloud2World(PointCloud& P) const
{
	for(int i=0; i<(int)P.size(); i++)
		ToWorld(P[i]);
}

void Frame::Cloud2Frame(PointCloud& P) const
{
	for(int i=0; i<(int)P.size(); i++)
		ToFrame(P[i]);
}

void Frame::eX(Point3Dbl& EX) const
{
	Point3Dbl nr;
	nr.ToCartNormalized((*this)[4], (*this)[5]); 
	RotateVector(nr, (*this)[3], Point3Dbl(1,0,0), EX);
}

void Frame::eY(Point3Dbl& EY) const 
{
	Point3Dbl nr;
	nr.ToCartNormalized((*this)[4], (*this)[5]); 
	RotateVector(nr, (*this)[3], Point3Dbl(0,1,0), EY);
}

void Frame::eZ(Point3Dbl& EZ) const 
{	
	Point3Dbl nr;
	nr.ToCartNormalized((*this)[4], (*this)[5]); 
	RotateVector(nr, (*this)[3], Point3Dbl(0,0,1), EZ);
}

int Frame::GetFrameThreeMatches(const Point3Dbl& OA, const Point3Dbl& A1, const Point3Dbl& A2,  
							const Point3Dbl& OB, const Point3Dbl& B1, const Point3Dbl& B2)
{
	/// Get the translation
	Point3Dbl A01, A02, B01, B02;
	Point3Dbl Translation;
	Translation.SubVec(OA, OB); 

	/// Set translation parameters
	(*this)[0] = Translation.m_x;
	(*this)[1] = Translation.m_y;
	(*this)[2] = Translation.m_z;
	
	/// Get the rotation
	Point3Dbl RotaNormal;
	double Alpha;
	A01.SubVec(A1, OA);
	A02.SubVec(A2, OA);
	B01.SubVec(B1, OB);
	B02.SubVec(B2, OB);
	if(GetRotation(B01, B02, A01, A02, RotaNormal, Alpha)==RET_FAILED) return RET_FAILED;
	RotaNormal.ToSphereNormalized();

	(*this)[3] = Alpha;
	(*this)[4] = RotaNormal.m_y; // Theta
	(*this)[5] = RotaNormal.m_z; // Phi 

	/// Debug checks (may be removed if this function is fully tested)
	//assert((*this)[3] >= 0.0 && (*this)[3] < THE_PI_DEF);
	//assert((*this)[4] >= 0.0 && (*this)[4] <= THE_PI_DEF);
	//assert((*this)[5] > -THE_PI_DEF && (*this)[5] <= THE_PI_DEF);
	return RET_OK;
}

int Frame::GetFrameThreeMatchesNoOrigin(const Point3Dbl& A0, const Point3Dbl& A1, const Point3Dbl& A2,  
							const Point3Dbl& B0, const Point3Dbl& B1, const Point3Dbl& B2)
{
	/// Get two frames from the triples
	Frame FB, FA;
	if(FB.MakeFrameThreePoints(B0, B1, B2)==RET_FAILED) return RET_FAILED;
	if(FA.MakeFrameThreePoints(A0, A1, A2)==RET_FAILED) return RET_FAILED;

	Point3Dbl OB, OA;
	FB.ToFrame(Point3Dbl(0,0,0), OB);
	FA.ToWorld(OB, OA);

	/// Get the rotation
	return GetFrameThreeMatches(OA, A1, A2, Point3Dbl(0,0,0), B1, B2);
}

double Frame::GetFrameNMatchesIt(const PointCloud& A, const PointCloud& B, int It, double Delta, double Shrink)
{
	this->SetZero();
	
	/// Get the current approximation
	PointCloud C;
	C.ImportCloud(A);
	Cloud2World(C);
	double Dist = PClDistSqr(B, C);
	//std::cout << "Frame::GetFrameNMatchesIt(" << __LINE__ << "): init distance: " << sqrt(Dist) << "\n";
	
	/// Align center points
	Point3Dbl ac, bc, aox, aoy, box, boy;
	A.GetCenter(ac);
	B.GetCenter(bc);
	(*this)[0] = ac.m_x-bc.m_x;
	(*this)[1] = ac.m_y-bc.m_y;
	(*this)[2] = ac.m_z-bc.m_z;

	C.ImportCloud(A);
	Cloud2Frame(C);
	Dist = PClDistSqr(B, C);
	//std::cout << "GetFrameNMatchesIt: start distance: " << sqrt(Dist) << "\n";

	// loop
	double D = Delta;
	double NewDist = DBL_MAX;
	int ItCnt=0;
	for(int i=0; i<It; ItCnt++)
	{
		int Flag = 0;
		for(int j=0; j<(int)size(); j++)
		{
			for(int s=-1; s<=1; s+=2)
			{
				double Old = (*this)[j];
				double Sign=(double)s;
				if(j==3 || j==4) Sign*=-1;
				double Step=D;
				if(j>2) Step*=2.0*THE_PI_DEF;
				(*this)[j] += Sign*Step;
				C.ImportCloud(A);
				Cloud2Frame(C);
				NewDist = PClDistSqr(B, C);
				if(NewDist<Dist)
				{
					Dist=NewDist;
					Flag = 1;
				}
				else if(NewDist>Dist)
				{
					(*this)[j] = Old;
					NewDist=Dist;
				}	
			}
		}
		if(Flag==0)
		{
			D*=Shrink;
			i++;
		}
	}
	//std::cout << "GetFrameNMatchesIt: final distance: " << sqrt(NewDist) << "\n";
	//std::cout << "GetFrameNMatchesIt: iterations: " << ItCnt << "\n";
	return sqrt(NewDist);
}

int Frame::GetFrameRANSAC(const PointCloud& A, const PointCloud& B, int IterationsK,  double ThreshT, int MinFitPointsD, int MinPointsN)
{
	assert(A.size()==B.size());

	VotingList VL;
	int MinPoints=intmax(MinPointsN, 3);
	double ThreshSqr=ThreshT*ThreshT;
	for(int k=0; k<IterationsK; k++)
	{
		/// Select MinPointsN points
		PointCloud ASmall, BSmall;
		std::set<int> Addr;
		for(int n=0; n<MinPoints;)
		{
			int R = RandInt(0, (int)A.size()-1);
			if(Addr.find(R)==Addr.end())
			{
				Addr.insert(R);
				ASmall.push_back(A[R]);
				BSmall.push_back(B[R]);
				n++;
			}
		}

		Frame F;
		F.GetFrameNMatchesIt(ASmall, BSmall, 3);
		//std::cout << "Trying: " << F.Str();
		Point3Dbl Test; F.ToFrame(A[0], Test);
		//std::cout << "A: " << B[0].Str() << "A' " << Test.Str() << std::endl;

		/// Get close samples in B
		double Err=0.0;
		IntVector GoodFits;
		for(int i=0; i<(int)A.size(); i++)
		{
			//if(Addr.find(i)!=Addr.end()) continue;

			Point3Dbl ADot;
			F.ToFrame(A[i], ADot);
			double dSqr = ADot.GetSqrDistance(B[i]);
			//std::cout << "Dist: " << d << std::endl;
			if(dSqr<ThreshSqr)
			{
				Err+=dSqr;
				GoodFits.push_back(i);
			}
		}
		if(GoodFits.size()>0) Err = sqrt(Err/(double)GoodFits.size());
		else Err=DBL_MAX;
		std::cout << "Err " << Err << " fits " << GoodFits.size() << std::endl;
		
		if((int)GoodFits.size()>=MinFitPointsD)
		{
			/// Refit using the goodies
			PointCloud GoodiesA, GoodiesB;
			for(int l=0; l<(int)GoodFits.size(); l++)
			{
				GoodiesA.push_back(A[GoodFits[l]]);
				GoodiesB.push_back(B[GoodFits[l]]);
			}
			Frame G;
			Err = G.GetFrameNMatchesIt(GoodiesA, GoodiesB, 4);
			VL.AppendVote(FrameStruct(G, Err)); 
		}
	}
	
	if(VL.size()>0)
	{
		std::sort(VL.begin(), VL.end(), ipa_Utils::FrameStructGreater);
		(*this)=VL.back().m_F;
		return RET_OK;
	}
	
	return RET_FAILED;
}

int Frame::GetMeanFrame(Frame& A, Frame& B)
{
	/// Get sum vectors
	Point3Dbl c, ex, ey, C(0), EX(0), EY(0);
	A.GetT(c); A.eX(ex); A.eY(ey);
	C.AddVec(c); EX.AddVec(ex); EY.AddVec(ey);
	B.GetT(c); B.eX(ex); B.eY(ey);
	C.AddVec(c); EX.AddVec(ex); EY.AddVec(ey);
	C.ScalarMul(0.5);

	/// Make a from from all this
	return this->MakeFrameOriginDirectionsXY(C, EX, EY);
}

double Frame::FrameDifferenceSimple(const Frame& A, double RotInfluence)
{
	double T = fabs((*this)[0]-A[0])+fabs((*this)[1]-A[1])+fabs((*this)[2]-A[2]);
	double R = fabs((*this)[3]-A[3])+fabs((*this)[4]-A[4])+fabs((*this)[5]-A[5]);
	return T + RotInfluence * R;
}

double Frame::FrameDifference(const Frame& A, double RotInfluence)
{
	/// Get the translation distance
	double dT = Point3Dbl(A[0],A[1],A[2]).GetDistance(Point3Dbl((*this)[0], (*this)[1], (*this)[2]));

	/// Get rotation difference
	Point3Dbl RA, RB, R;
	RA.ToCartNormalized(A[4], A[5]);
	RB.ToCartNormalized((*this)[4], (*this)[5]);
	double dR = GetRotationDistance(RA, A[3], RB, (*this)[3]); 
	
	return dT+RotInfluence*dR;
}

double Frame::Norm(double RotInfluence)
{
	Frame Zero;
	return FrameDifference(Zero, RotInfluence);
}

void Frame::LimitRotation()
{
		if((*this)[3]<0.0) (*this)[3]=0.0;
		if((*this)[3]>=THE_PI_DEF) (*this)[3]=THE_PI_DEF-THE_EPS_DEF;
		if((*this)[4]<0.0) (*this)[4]=0.0;
		if((*this)[4]>THE_PI_DEF) (*this)[4]=THE_PI_DEF;
		if((*this)[5]<=-THE_PI_DEF || (*this)[5]>THE_PI_DEF) (*this)[5]=THE_PI_DEF;
}

int Frame::GetCentralFrame(const PointCloud& PCl)
{
	/// Get the center and orientations
	Point3Dbl c, ox, oy;
	PCl.GetCenter(c);
	PCl.GetOrientations(ox, oy);

	/// Check orthogonality and set/get translation and rotation
	double sp=0.0;
	sp = ox.ScalarProd(oy);
	if(fabs(sp) > THE_EPS_DEF) return RET_FAILED;
	SetTranslation(c);
	Point3Dbl RotaNormal; double Alpha=0.0;
	if(GetRotation(ox, oy, Point3Dbl(1,0,0), Point3Dbl(0,1,0), RotaNormal, Alpha)==RET_FAILED)
		return RET_FAILED;
	RotaNormal.ToSphereNormalized();
	(*this)[3] = Alpha;
	(*this)[4] = RotaNormal.m_y; // Theta
	(*this)[5] = RotaNormal.m_z; // Phi
	return RET_OK;
}

int Frame::ToWorld(const Frame& A, Frame& B)
{
	/// Convert origin, ex and ey
	Point3Dbl O, EX, EY;
	ToWorld(Point3Dbl(A[0], A[1], A[2]), O);
	A.eX(EX); A.eY(EY);
	EX.AddVec(Point3Dbl(A[0], A[1], A[2]), EX);
	EY.AddVec(Point3Dbl(A[0], A[1], A[2]), EY);
	ToWorld(EX);
	ToWorld(EY);
	EX.SubVec(O);
	EY.SubVec(O);
	
	/// Set frame
	B.SetTranslation(O);
	Point3Dbl RotaNormal; double Alpha=0.0;
	if(GetRotation(Point3Dbl(1,0,0), Point3Dbl(0,1,0), EX, EY, RotaNormal, Alpha)==RET_FAILED)
		return RET_FAILED;
	RotaNormal.ToSphereNormalized();
	B[3] = Alpha;
	B[4] = RotaNormal.m_y; // Theta
	B[5] = RotaNormal.m_z; // Phi
	
	return RET_OK;
}

int Frame::ToFrame(const Frame& A, Frame& B)
{
	/// Convert origin, ex and ey
	Point3Dbl O, EX, EY;
	ToFrame(Point3Dbl(A[0], A[1], A[2]), O);
	A.eX(EX); A.eY(EY);
	EX.AddVec(Point3Dbl(A[0], A[1], A[2]), EX);
	EY.AddVec(Point3Dbl(A[0], A[1], A[2]), EY);
	ToFrame(EX);
	ToFrame(EY);
	EX.SubVec(O);
	EY.SubVec(O);
	
	/// Set frame
	B.SetTranslation(O);
	Point3Dbl RotaNormal; double Alpha=0.0;
	if(GetRotation(Point3Dbl(1,0,0), Point3Dbl(0,1,0), EX, EY, RotaNormal, Alpha) & RET_FAILED)
		return RET_FAILED;
	RotaNormal.ToSphereNormalized();
	B[3] = Alpha;
	B[4] = RotaNormal.m_y; // Theta
	B[5] = RotaNormal.m_z; // Phi
	
	return RET_OK;
}

void Frame::Invert()
{
	Frame Origin, Tmp;
	ToFrame(Origin, Tmp);
	*this = Tmp;
}

std::string FrameList::Str()
{
	std::stringstream s;
	s << size() << "  ";
	FrameList::iterator It;
	for(It=begin(); It!=end(); It++)
		s << It->Str();
	return s.str();
}

void FrameList::QTClustering(double Eps, unsigned int MinN, bool UseMean)
{
	std::cout << "\nClustering ";

	/// 0. Initialize: get a vector of iterators
	std::vector<FrameList::iterator> its;
	FrameList::iterator it=begin();
	for(; it!=end(); it++)
	{
		its.push_back(it);
		std::cout << ".";
	}
	std::cout << "\n";

	/// 1. get for each frame all members in the epsilon vicinity
	
	/// 1.1. Initialize the members
	IntMatrix members;
	members.assign(its.size(), IntVector());
	for(unsigned int i=0; i<its.size(); i++)
	{
		members[i].push_back(i);
		std::cout << ".";
	}
	std::cout << "\n";

	/// 1.2. Get the members
	for(unsigned int j=0; j<its.size()-1; j++)
	{
		for(unsigned int k=j+1; k<its.size(); k++)
		{
			double Difference = its[j]->FrameDifference(*its[k]);
			if(Difference <= Eps)
			{
				members[j].push_back(k);
				members[k].push_back(j);
				std::cout << ".";
			}
		}	
	}
	std::cout << "\n";

	/// 2. Sort and remove
	//std::cout << "\n";

	/// 2.1 Sort and build a map
	DblVector sortVec;
	std::map<int, IntVector> memberMap;
	for(unsigned int i=0; i<members.size(); i++)
		sortVec.push_back((double)members[i].size());
	SortedVector sortedVec(sortVec);
	for(unsigned int i=0; i<sortedVec.size(); i++)
	{
		/// Break if clusters are too small
		if(sortedVec[i].m_V<MinN) break;
		else
		{
			memberMap[sortedVec[i].m_N]=IntVector();
			for(unsigned int j=1; j<sortedVec[i].m_V; j++)
			{
				memberMap[sortedVec[i].m_N].push_back(members[sortedVec[i].m_N][j]);
				std::cout << ".";
			}
		}
	}
	std::cout << "\n";

	/// 2.2 Remove frames contained in other clusters
	for(unsigned int i=0; i<sortedVec.size(); i++)
	{
		/// Access the member map if possible, else continue
		if(memberMap.find(sortedVec[i].m_N)==memberMap.end()) continue;
		else
		{
			/// Loop over all members and delete them
			for(unsigned int j=0; j<memberMap[sortedVec[i].m_N].size(); j++)
			{
				if(memberMap.find(memberMap[sortedVec[i].m_N][j])!=memberMap.end())
					memberMap.erase(memberMap.find(memberMap[sortedVec[i].m_N][j]));

				std::cout << ".";
			}
		}
	}
	std::cout << "\n";

	/// 3. Get the most important frames
	FrameList tmp;
	std::map<int, IntVector>::iterator memberIt=memberMap.begin();
	for(; memberIt!=memberMap.end(); memberIt++)
	{
		tmp.push_back(*(its[memberIt->first]));
		std::cout << ".";
	}
	std::cout << "\n";

	/// 4. Overwrite list
	*this = tmp;
}

std::string FrameStruct::Str()
{
	std::stringstream s;
	s << " Frame: " << m_F.Str();
	s << " Count: " << m_S << "\n";
	//s << " Pos:   " << m_u << " " << m_v << "\n";
	return s.str();
}

bool FrameStructGreater(const FrameStruct& fs1, const FrameStruct& fs2)
{
	return fs1.m_S > fs2.m_S;
}

unsigned long VotingList::GetCenterOfGravity(FrameStruct *center)
{
	VotingList::iterator iterator;
	for(iterator=begin(); iterator!=end(); iterator++)
	{
		(*center).m_F[0] += iterator->m_F[0];
		(*center).m_F[1] += iterator->m_F[1];
		(*center).m_F[2] += iterator->m_F[2];
		(*center).m_F[3] += iterator->m_F[3];
		(*center).m_F[4] += iterator->m_F[4];
		(*center).m_F[5] += iterator->m_F[5];
	}
	(*center).m_F[0] /= this->size();
	(*center).m_F[1] /= this->size();
	(*center).m_F[2] /= this->size();
	(*center).m_F[3] /= this->size();
	(*center).m_F[4] /= this->size();
	(*center).m_F[5] /= this->size();

	return RET_OK;
}

void VotingList::GetCentralFrameStruct(FrameStruct& FS)
{
	/// Get sum vectors
	Point3Dbl c, ex, ey, C(0), EX(0), EY(0);
	double Cnt=0;
	VotingList::iterator It;
	double Sum=0.0;
	for(It=begin(); It!=end(); It++)
	{
		It->m_F.GetT(c);
		It->m_F.eX(ex);
		It->m_F.eY(ey);
		C.AddVec(c);
		EX.AddVec(ex);
		EY.AddVec(ey);
		Cnt++;
		Sum+=It->m_S;
	}
	C.ScalarMul(1.0/Cnt);

	/// Make a frame from all this
	Frame Center;
	FS.m_F.MakeFrameOriginDirectionsXY(C, EX, EY);
	FS.m_S=Sum;
}

void VotingList::GetCentralFrameStructDensity(FrameStruct& FS, double Eps, bool Mean)
{
	/// Count all entries in the close (epsilon) vicinity
	IntMatrix Counts;
	DblVector Densities;
	for(unsigned int i=0; i<size(); i++)
	{
		IntVector Tmp;
		Tmp.push_back(i);
		Counts.push_back(Tmp);
		Densities.push_back((*this)[i].m_S);
	}
	for(unsigned int p0=0; p0<size()-1; p0++)
	{
		for(unsigned int p1=p0+1; p1<size(); p1++)
		{
			double d=(*this)[p0].m_F.FrameDifference((*this)[p1].m_F);
			
			if(d<=Eps)
			{
				Counts[p0].push_back(p1);
				Counts[p1].push_back(p0);
				Densities[p0]+=(*this)[p1].m_S;
				Densities[p1]+=(*this)[p0].m_S;
			}
		}
	}
	
	/// Sorting according to densities
	SortedVector sortedVector(Densities);
	int MostDense = sortedVector[0].m_N;

	if(!Mean)
	{
		FS = (*this)[MostDense];
	}
	else
	{
		/// Make a new list containing the supporters
		VotingList NewList;
		for(unsigned int p=0; p<Counts[MostDense].size(); p++)
			NewList.push_back((*this)[Counts[MostDense][p]]);
		
		/// Take the mean frame of the selected sub set
		NewList.GetCentralFrameStruct(FS);
	}
}

std::string VotingList::Str()
{
	std::stringstream s;
	for(unsigned int i=0; i<this->size(); i++)
		s << (*this)[i].Str() << " ";
	return s.str();
}

void GeometryTestFrames()
{
	int NoTests = 10;
	for(int i=0; i<NoTests; i++)
	{
		PointCloud A, B;
		//A.push_back(Point3Dbl(0,0,0));
		A.push_back(Point3Dbl(1,0,0));
		A.push_back(Point3Dbl(2,2,0));
		A.push_back(Point3Dbl(3,5,0));
		A.push_back(Point3Dbl(4,0,0));
		A.push_back(Point3Dbl(5,1,0));
		//A.AssignRand(-1.0, 1.0, 15, true);

		std::cout << "A:\n" << A.Str();
	
		Frame F;
		//F.SetTranslation(3,3,1);
		//F.SetRotation(THE_PI_DEF,0,0);
		//F.SetRotation(-THE_PI_DEF/2.0, THE_PI_DEF/4.0, -THE_PI_DEF/4.0);
		F.SetRandomFrame(0.5, 0.5);

		std::cout << "F: " << F.Str();
		B.ImportCloud(A);
		F.Cloud2World(B);
		//B.AssignRand(-1.0, 1.0, 40, true);
		//A.AssignRand(-1.0, 1.0, 40, true);
		std::cout << "B:\n" << B.Str();

		//F.GetFrameThreeMatchesNoOrigin(B[0], B[1], B[2], A[0], A[1], A[2]);
		//F.GetFrameThreeMatchesNoOrigin(B[0], B[1], B[2], A[0], A[1], A[2]);
		//F.GetFrameNMatchesIt(B, A);
		F.GetFrameRANSAC(B, A, 100, 0.01, 10, 3);
		std::cout << "F'\n" << F.Str();
		//F.Cloud2Frame(B);
		std::cout << "B'=A:\n" << B.Str();
		std::cout << "Press key: " << std::endl;
		getchar();
	}
}

void GeometryTestMeanNorm()
{
	Point3Dbl r0(0,0,1), r1(0,0.5,1.0), r2(1,1,0), r3(0.5,0,1);
	r0.Normalize();
	r1.Normalize();
	r2.Normalize();
	r3.Normalize();
	double a0=0.3, a1=0.25, a2=1.4, a3=0.6, w0=1.0;
	
	Point3Dbl r4(0,0,0); double a4=0.0;
	GetRotationMean(r0, a0, r1, a1, r4, a4, w0);
	w0+=1.0;
	GetRotationMean(r4, a4, r2, a2, r4, a4, w0);
	w0+=1.0;
	GetRotationMean(r4, a4, r3, a3, r4, a4, w0);
	std::cout << "First estimate:   " << r4.Str() << " " << a4 << "\n";


	w0=1.0;
	Point3Dbl r5(0,0,0); double a5=0.0;
	GetRotationMean(r2, a2, r3, a3, r5, a5, w0);
	w0+=1.0;
	GetRotationMean(r5, a5, r1, a1, r5, a5, w0);
	w0+=1.0;
	GetRotationMean(r5, a5, r0, a0, r5, a5, w0);
	std::cout << "Second estimate:  " << r5.Str() << " " << a5 << "\n";

	w0=1.0;
	Point3Dbl r6(0,0,0); double a6=0.0;
	GetRotationMean(r2, a2, r3, a3, r6, a6, w0);
	w0+=1.0;
	GetRotationMean(r6, a6, r0, a0, r6, a6, w0);
	w0+=1.0;
	GetRotationMean(r6, a6, r1, a1, r6, a6, w0);
	std::cout << "Third estimate: " << r6.Str() << " " << a6 << "\n";
}



} // end namespace ipa_Utils

