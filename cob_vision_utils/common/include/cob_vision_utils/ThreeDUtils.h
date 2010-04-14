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

/// @file ThreeDUtils.h
/// A minimal but "complete" 3D library.
/// This library provides classes for 3D points, 3D point clouds, 3D colored point clouds and a 6D (translation and orientation) frame class.
/// Also a few algorithms for frame reconstruction from point correspondences are provided.
/// This file and the corresponding .cpp file were written by Jens Kubacki in 2005-2008.
/// Latest updates: November 2008.

#ifndef __THREEDUTILS_H__
#define __THREEDUTILS_H__

#ifdef __COB_ROS__
#include "cob_vision_utils/MathUtils.h"
#include "cob_vision_utils/GlobalDefines.h"
#else
#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/MathUtils.h"
#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/GlobalDefines.h"
#endif
#include <list>
#include <map>
#include <set>

namespace ipa_Utils {

static const double ROT_FACTOR = 0.032; // PI * ROT_FACTOR = 0.1 m

/// Basic point struct/class to represent 3 double components.
class Point3Dbl
{
public:

	/// Constructor. 
	Point3Dbl(){};

	/// Copy constructor
	Point3Dbl(const Point3Dbl& p3dbl) {m_x=p3dbl.m_x; m_y=p3dbl.m_y; m_z=p3dbl.m_z;}
	
	/// Constructor with initialization.
	Point3Dbl(double x, double y, double z) {m_x=x; m_y=y; m_z=z;};
	
	/// Constructor with one value initialization
	Point3Dbl(double x) {m_x=x; m_y=x; m_z=x;};

	/// Destructor.
	~Point3Dbl(){};
	
	/// Set function to assign values.
	/// Computes m_x=x, m_y=y, and m_z=z.
	/// @param x x value
	/// @param y y value
	/// @param z z value
	void Set(double x, double y, double z){m_x=x; m_y=y; m_z=z;}
	
	/// Set function to assign values.
	/// Computes m_x=v, m_y=v, and m_z=v.
	/// @param v the value
	void Set(double v){m_x=v; m_y=v; m_z=v;}
	
	/// Set function to assign values.
	/// Computes m_x=p.m_x, m_y=p.m_y, and m_z=p.m_z.
	/// @param p a 3D point
	void Set(const Point3Dbl& p) {m_x=p.m_x; m_y=p.m_y; m_z=p.m_z;}
	
	/// String conversion.
	std::string Str() const {std::stringstream s; s << m_x << " " << m_y << " " << m_z; return s.str();}

	/// x component.
	double m_x;
	
	/// y component.
	double m_y;
	
	/// z component.
	double m_z;

	/// Get square of absolute value.
	/// @return m_x*m_x+m_y*m_y+m_z*m_z.
	double AbsValSqr() const {return m_x*m_x+m_y*m_y+m_z*m_z;}
	
	/// Get the absolute value (magnitude).
	/// @see AbsValSqr()
	/// @return sqrt(AbsValSqr())
	double AbsVal() const {return sqrt(AbsValSqr());}
	
	/// Scalar multiplication.
	/// Computes m_x=s*A.m_x, m_y=s*A.m_y, and m_z=s*A.m_z.
	/// @param s the scalar
	/// @param A the argument vector
	void ScalarMul(const double& s, const Point3Dbl& A) {m_x=s*A.m_x; m_y=s*A.m_y; m_z=s*A.m_z;}
	
	/// Scalar multiplication (in-place).
	/// Computes m_x*=s, m_y*=s, and m_z*=s.
	/// @param s the scalar
	void ScalarMul(const double& s) {m_x*=s; m_y*=s; m_z*=s;}
	
	/// Componentwise multiplication.
	/// @param A the first argument vector
	/// @param B the second argument vector
	void CompMul(const Point3Dbl& A, const Point3Dbl& B) {m_x = A.m_x*B.m_x; m_y = A.m_y*B.m_y; m_z = A.m_z*B.m_z;}

	/// Componentwise in-place multiplication.
	/// Computes m_x *= A.m_x, m_y *= A.m_y, and m_z *= A.m_z.
	/// @param A the argument vector
	void CompMul(const Point3Dbl& A) {m_x *= A.m_x; m_y *= A.m_y; m_z *= A.m_z;}
	
	/// Normalization.
	/// Computes a=A.AbsVal() and ScalarMul(1.0/a, A).
	/// @param A the argument vector
	void Normalize(const Point3Dbl& A) {double a=A.AbsVal(); if(a<THE_EPS_DEF) Set(0,0,1); else ScalarMul(1.0/a, A);}
	
	/// Normalization (in-place).
	void Normalize() {double a=AbsVal(); if(a<THE_EPS_DEF) Set(0,0,1); else {m_x/=a; m_y/=a; m_z/=a;}}

	/// Negative vector.
	/// Computes m_x=-A.m_x, m_y=-A.m_y, and m_z=-A.m_z.
	/// @param A the argument vector
	void Negative(const Point3Dbl& A) {m_x=-A.m_x; m_y=-A.m_y; m_z=-A.m_z;}
	
	/// Negative vector (in-place).
	/// Computes m_x*=-1.0, m_y*=-1.0, and m_z*=-1.0.
	void Negative() {m_x*=-1.0; m_y*=-1.0; m_z*=-1.0;}
	
	/// Scale inversion.
	/// Reciprocal component values.
	/// @param A the argument vector
	void Inverse(const Point3Dbl& A) {m_x=1.0/A.m_x; m_y=1.0/A.m_y; m_z=1.0/A.m_z;}
	
	/// Scale inversion (in-place).
	/// Reciprocal component values.
	/// Computes m_x=1.0/m_x, m_y=1.0/m_y, and m_z=1.0/m_z.
	void Inverse() {m_x=1.0/m_x; m_y=1.0/m_y; m_z=1.0/m_z;}
	
	/// Get the squared euclidian distance between two vectors.
	/// Computes dblsqr(A.m_x-m_x)+dblsqr(A.m_y-m_y)+dblsqr(A.m_z-m_z).
	/// @param A the argument vector
	double GetSqrDistance(const Point3Dbl& A) const {return dblsqr(A.m_x-m_x)+dblsqr(A.m_y-m_y)+dblsqr(A.m_z-m_z);}
	
	/// Get the euclidian distance between two vectors.
	/// Computes sqrt(GetSqrDistance(A)).
	/// @param Arg the argument vector
	double GetDistance(const Point3Dbl& A) const {return sqrt(GetSqrDistance(A));}
	
	/// Vector addition.
	/// Computes m_x=A.m_x+B.m_x, m_y=A.m_y+B.m_y, and m_z=A.m_z+B.m_z.
	/// @param A the first argument vector
	/// @param B the second argument vector
	void AddVec(const Point3Dbl& A, const Point3Dbl& B) {m_x=A.m_x+B.m_x; m_y=A.m_y+B.m_y; m_z=A.m_z+B.m_z;}
	
	/// Vector addition (in-place).
	/// Computes m_x+=A.m_x, m_y+=A.m_y, m_z+=A.m_z.
	/// @param A the argument vector
	void AddVec(const Point3Dbl& A) {m_x+=A.m_x; m_y+=A.m_y; m_z+=A.m_z;}
	
	/// Vector substraction. Computes A - B.
	/// @param A the first argument vector
	/// @param B the second argument vector
	void SubVec(const Point3Dbl& A, const Point3Dbl& B) {m_x=A.m_x-B.m_x; m_y=A.m_y-B.m_y; m_z=A.m_z-B.m_z;}
	
	/// Vector substraction (in-place).
	/// Computes m_x-=A.m_x, m_y-=A.m_y, m_z-=A.m_z.
	/// @param Arg the argument vector
	void SubVec(const Point3Dbl& A) {m_x-=A.m_x; m_y-=A.m_y; m_z-=A.m_z;}

	/// Scalar product (dot product).
	/// Computes m_x*A.m_x+m_y*A.m_y+m_z*A.m_z.
	/// @param Arg the argument vector
	/// @return the scalar prduct
	double ScalarProd(const Point3Dbl& A) const {return m_x*A.m_x+m_y*A.m_y+m_z*A.m_z;}
	
	/// Vector product (cross product).
	/// Computes m_x=A.m_y*B.m_z-A.m_z*B.m_y, m_y=A.m_z*B.m_x-A.m_x*B.m_z, and m_z=A.m_x*B.m_y-A.m_y*B.m_x.
	/// @param A the first argument vector
	/// @param B the second argument vector
	void VectorProd(const Point3Dbl& A, const Point3Dbl& B) {m_x=A.m_y*B.m_z-A.m_z*B.m_y; m_y=A.m_z*B.m_x-A.m_x*B.m_z; m_z=A.m_x*B.m_y-A.m_y*B.m_x;}
	
	/// Get the cosine between two vectors.
	/// Computes ScalarProd(A)/(AbsVal()*A.AbsVal()).
	/// @param A the argument vector
	/// @return the cosine
	double GetCosine(const Point3Dbl& A) const {if(AbsVal()==0.0 || A.AbsVal()==0.0) return 1.0; else return ScalarProd(A)/(AbsVal()*A.AbsVal());}
	
	/// Get the sine between two vectors.
	/// @param A the argument vector 
	/// @return the sine
	double GetSine(const Point3Dbl& A) const {Point3Dbl Tmp; Tmp.VectorProd(*this, A); return Tmp.AbsVal()/(AbsVal()*A.AbsVal());}
	
	/// Get the peak angle between two vectors.
	/// Uses atan2(). 
	/// @param A the argument vector 
	/// @return the angle
	double GetAngle(const Point3Dbl& A) const {Point3Dbl Tmp; Tmp.VectorProd(*this, A); return atan2(Tmp.AbsVal(), ScalarProd(A));} 

	/// Get theta.
	/// Get the theta angle of the sphere coordinates.
	//double GetThetaSphere(const Point3Dbl& Point) {return atan2(sqrt(dblsqr(Point.m_x)+dblsqr(Point.m_y)),Point.m_z);}

	/// Get phi.
	/// Get the theta angle of the sphere coordinates.
	//double GetPhiSphere(const Point3Dbl& Point) {return atan2(Point.m_y, Point.m_x);}

	/// Conversion to sphere coordinates.
	/// Assumption xy-plane is the spherical reference plane
	void ToSphere();
	void ToSphereNormalized();
	
	/// Conversion from sphere to cartesian coordinates.
	void ToCart();

	/// Conversion from sphere to cartesian coordinates only phi and theta (assuming r=1).
	/// @param Phi computed phi
	/// @param Theta computed theta
	void ToCartNormalized(double Theta, double Phi);

	/// Projection of a vector B on this vector.
	void Projection(const Point3Dbl& B);

	/// Orthogonal projection length.
	double GetLengthOrthProjection(const Point3Dbl& B);

	/// Orthogonal component.
	void OrthProjection(const Point3Dbl& B);
	void OrthProjectionDiffVector(const Point3Dbl& B);
};

/// Rotate vector. Rotation of a vector around the axis RotaNormal. The amount of rotation is given by Phi.
/// This function is implemented using Rodrigues' rotation formula (see e.g. http://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula) 
/// @param RotaNormal the rotation normal (axis around which will be rotated).
/// @param Phi The angle (amount) of rotation.
/// @param A The vector that is rotated around RotaNormal with angle Phi.
/// @param Res The rotated result vector.
void RotateVector(const Point3Dbl& RotaNormal, double Phi, const Point3Dbl& A, Point3Dbl& Res);

/// Estimate the orientation of two points around the origin
/// This function is implemented using Rodrigues' rotation formula
/// (see e.g. http://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula).
/// @param RotaNormal the rotation normal (axis around which will be rotated).
/// @param Phi the angle (amount) of rotation.
/// @param A the argument vector.
/// @param Res the rotated result vector.
int GetRotation(const Point3Dbl& A1, const Point3Dbl& B1,
				 const Point3Dbl& A2, const Point3Dbl& B2,
				 Point3Dbl& RotaNormal, double& Phi);

double GetRotationDistance(Point3Dbl& RotaNormal0, double Alpha0, Point3Dbl& RotaNormal1, double Alpha1);
void GetRotationMean(Point3Dbl& RotaNormal0, double Alpha0, Point3Dbl& RotaNormal1, double Alpha1, Point3Dbl& RotaNormal2, double& Alpha2, double w0=1.0, double w1=1.0);

/// Line function: get the base point of a point Q.
/// This function compute the orthogonal projection of Q on the line given by S0 and S1.
void GetBasePoint(const Point3Dbl& S0, const Point3Dbl& S1, const Point3Dbl& Q, Point3Dbl& X);

/// Line function: get the distance to Q.
/// This function compute the distance between  Q and the line given by S0 and S1.
double GetLineDistance(const Point3Dbl& S0, const Point3Dbl& S1, const Point3Dbl& Q);

/// Class to describe a 3D integer point.
/// This class can be used to describe a 3D integer point (integer triple).
class Point3Int
{
public:
	Point3Int(){};
	Point3Int(int x, int y, int z) {m_x=x; m_y=y; m_z=z;};
	~Point3Int(){};
	std::string Str() const {std::stringstream s; s << m_x << " " << m_y << " " << m_z << "\n"; return s.str();}
	int m_x;
	int m_y;
	int m_z;
	Point3Int operator+(const Point3Int& B) const {Point3Int P; P.m_x=m_x+B.m_x; P.m_y=m_y+B.m_y; P.m_z=m_z+B.m_z; return P;}
};

/// Function to compare two Point3Int to use Point3Int as map key.
struct CmpPoint3Int
{
	bool operator()( const Point3Int& p1, const Point3Int& p2 ) const
	{
		if(p1.m_x < p2.m_x) return true;
		if(p1.m_x > p2.m_x) return false;
		if(p1.m_y < p2.m_y) return true;
		if(p1.m_y > p2.m_y) return false;
		if(p1.m_z < p2.m_z) return true;
		if(p1.m_z > p2.m_z) return false;
		return false;
    }
};

/// Class to describe a 5D integer point.
/// This class can be used to describe a 5D integer point.
class Point5Int
{
public:
	Point5Int();
	Point5Int(int a, int b, int c, int d, int e) {m_a=a; m_b=b; m_c=c; m_d=d; m_e=e;};
	~Point5Int(){};
	std::string Str() {std::stringstream s; s << m_a << " " << m_b << " " << m_c << " " << m_d << " " << m_e << "\n"; return s.str();}
	int m_a;
	int m_b;
	int m_c;
	int m_d;
	int m_e;
};

struct CmpPoint5IntKeys
{
	bool operator()( const Point5Int& p1, const Point5Int& p2 ) const
	{
		if(p1.m_a < p2.m_a) return true;
		if(p1.m_a > p2.m_a) return false;
		if(p1.m_b < p2.m_b) return true;
		if(p1.m_b > p2.m_b) return false;
		if(p1.m_c < p2.m_c) return true;
		if(p1.m_c > p2.m_c) return false;
		if(p1.m_d < p2.m_d) return true;
		if(p1.m_d > p2.m_d) return false;
		if(p1.m_e < p2.m_e) return true;
		if(p1.m_e > p2.m_e) return false;
		return false;
	}
};

class CountsStruct
{
public:
	CountsStruct(int p=0, double c=0.0) {Position=p; Counts=c;}
	~CountsStruct(){};
	int Position;
	double Counts;
};

bool thisGreaterOD2(const CountsStruct& e1, const CountsStruct& e2);

typedef std::map<Point5Int, double, CmpPoint5IntKeys> DistanceMap;

/// Class to represent point clouds.
/// This class can be used to handle (create, load, save, maipulate, etc) point clouds.
class PointCloud : public std::vector<Point3Dbl>
{
public:
	PointCloud(){};
	~PointCloud(){};
	
	/// Make a base point cloud (origin, ex and ey) 
	/// This function creates the point cloud (0,0,0), (1,0,0), and (0,1,0).
	void MakeBase(double Unit=1.0);

	/// Assign a set of points to the cloud.
	/// @param P the default value for all points
	/// @param No the number of points
	void AssignPoints(const Point3Dbl& P, int No);
	
	/// Assign a set of random points.
	/// Assign a set of random points to the cloud
	/// @param Min the minimal possible value of a point component
	/// @param Max the maximal possible value of a point component
	/// @param No the number of points to be added
	/// @param Mask a point that can prevent a certain dimension to be set by setting this dimension to zero
	/// @param Append set to true to append PC to the object
	void AssignRand(double Min, double Max, int No,  bool Append=false, const Point3Dbl& Mask=Point3Dbl(1,1,1));
	
	/// Add random noise to each point component.
	/// @param Min the minimal possible value of noise addition
	/// @param Max the maximal possible value of noise addition
	/// @param Mask a point that can prevent a certain dimension to be set by setting this dimension to zero
	void AddRand(double Min, double Max, const Point3Dbl& Mask=Point3Dbl(1,1,1));
	
	/// Imports (and possibly appends) another point cloud.
	/// @param PC the point cloud to be imported
	/// @param Append set to true to append PC to the object
	void ImportCloud(const PointCloud& PC, bool Append=false);

	/// Convert to string.
	/// Convert to string for input/output purposes
	std::string Str() const;

	/// Load.
	int Load(std::string Name);
	
	/// Save.
	int Save(std::string Name) const;

	/// A vector sum function.
	void VectorSum(Point3Dbl& Sum) const;
	
	/// A vector mean function.
	void GetCenter(Point3Dbl& Center) const;

	/// Get orientations
	void GetOrientations(Point3Dbl& OriX, Point3Dbl& OriY) const;

};

/// Gets the squared euclidian distance between two point clouds.
double PClDistSqr(const PointCloud& A, const PointCloud& B);

/// Gets the euclidian distance of two point clouds.
double PClDist(const PointCloud& A, const PointCloud& B);


/// Frame class for coordinate systems.
/// The first three components are the translation offsets.
/// The fourth component is a rotation angle, valid in the interval [0.0,..,PI).
/// Components five and six are the spherical angles describing the rotation axis, Theta and Phi.
/// Theta is valid in the interval [0.0,..,PI] and Phi is valid in the interval (-PI,..,PI].
class Frame : public DblVector
{
public:
	
	Frame(double tx=0.0, double ty=0.0, double tz=0.0, double rx=0.0, double ry=0.0, double rz=0.0); ///< Constructor.
	~Frame(){}; ///< Destructor.
	std::string Str() const;
	void SetZero(){SetTranslation(0,0,0); SetRotation(0,0,0);} ///< Sets to zero (neutral) value.
	void SetTranslation(const Point3Dbl& Trans); ///< Sets translation components.
	void SetTranslation(double tx, double ty, double tz); ///< Sets translation components.
	void SetRotation(const Point3Dbl& Rot); ///< Sets rotation components.
	void SetRotation(double rx, double ry, double rz); ///< Sets rotation components.
	int MakeFrameOriginDirectionsXY(const Point3Dbl& O, const Point3Dbl& EX, const Point3Dbl& EY);
	int MakeFrameOriginDirectionsXZ(const Point3Dbl& O, const Point3Dbl& EX, const Point3Dbl& EZ);
	int MakeFrameThreePoints(const Point3Dbl& p0, const Point3Dbl& p1, const Point3Dbl& p2);
	void SetRandomTranslation(double Bound=0.1);
	void SetRandomRotation(double Factor=0.1); 
	void SetRandomFrame(double Bound=0.1, double Factor=0.1); ///< Sets a random frame.
	void GetT(Point3Dbl& T) const; ///< Get translation vector.
	void GetR(Point3Dbl& R) const; ///< Get rotation vector.
	double GetAlpha() {return (*this)[3];} ///< Get the rotation angle.

	/// ToWorld performs first a right handed rotation around the rotation axis and
	/// then applies the translation.
	/// To Frame performs the inverse operation to ToWorld. This means first the data points
	/// are translated in opposite direction (-t) and then rotated in opposite direction (-alpha)
	/// around the rotation axis.
	/// Translation is given by x = this[0], y = this[1], z = this[2]
	/// Rotation angle alpha is given by alpha = this[3]
	/// Spherical coordinates of rotation axis centered at the origin is given by theta = this[4], phi = this[5].
	/// For an explanation of the spherical coordinate system see http://en.wikipedia.org/wiki/Spherical_coordinate_system
	/// section 'Coordinate system conversion'
	void ToWorld(const Point3Dbl& In, Point3Dbl& Out) const; ///< Convert to world coordinate system.
	void ToWorld(Point3Dbl& P) const;  ///< Convert to world coordinate system.
	void ToWorld(double& x, double& y, double& z) const; ///< Convert to world coordinate system.
	void ToFrame(const Point3Dbl& In, Point3Dbl& Out) const; ///< Convert from world to frame.
	void ToFrame(Point3Dbl& P) const; ///< Convert from world to frame.
	void ToFrame(double& x, double& y, double& z) const; ///< Convert from world to frame.
	void Cloud2World(PointCloud& P) const; ///< Convert a point cloud to world coordinate system.
	void Cloud2Frame(PointCloud& P) const; ///< Convert a point cloud from world to frame coordinate system.

	void eX(Point3Dbl& EX) const;
	void eY(Point3Dbl& EY) const;
	void eZ(Point3Dbl& EZ) const;

	/// Get a conversion frame from three matches.
	/// This function gives a coordinate transformation between two frames.
	/// The first match has to contain the origins of the two systems.
	/// The triple has to be linearly independent.
	/// @param OA the origin of cloud A
	/// @param the second point of A
	/// @param the third point of A
	/// @param OB the origin of cloud B
	/// @param the second point of B
	/// @param the third point of B
	/// todo: generalize such that the first point can be arbitrary
	int GetFrameThreeMatches(const Point3Dbl& OA, const Point3Dbl& A1, const Point3Dbl& A2,  
								const Point3Dbl& OB, const Point3Dbl& B1, const Point3Dbl& B2);
	
	/// Get a conversion frame from three matches. Gets the coordinate transformation without knowing the origín points. 
	/// Not implemented.
	//int GetFrameFromTriple(const Point3Dbl& A0, const Point3Dbl& A1, const Point3Dbl& A2);
	int GetFrameThreeMatchesNoOrigin(const Point3Dbl& a0, const Point3Dbl& a1, const Point3Dbl& a2, 
					const Point3Dbl& b0, const Point3Dbl& b1, const Point3Dbl& b2);

	/// Get a frame conversion from A to B.
	/// This function uses a "greedy" search over the six frame components to estimate a conversion frame between point cloud A and point cloud B.
	/// Here the assumptions about the two point clouds are not strong.
	/// However, two very "dissimilar" clouds can not be matched as good as clouds that are only translated and rotated relative to each other.
	/// @param A the first point cloud A
	/// @param B the second point cloud B
	/// @param It number of iterations
	/// @param Delta step width (at start)
	/// @param Shrink shrinkage factor to reduce the step width Delta (Delta[n]=Delta[n-1]*Shrink)
	double GetFrameNMatchesIt(const PointCloud& A, const PointCloud& B, int It=6, double Delta=0.1, double Shrink=0.1);
	
	/// Get a frame from correspondence points including outliers.
	int GetFrameRANSAC(const PointCloud& A, const PointCloud& B, int IterationsK,  double ThreshT, int MinFitPointsD, int MinPointsN=3);
	
	/// Get a mean frame by summing all origins, exs, and eys and applying Gram-Schmidt
	int GetMeanFrame(Frame& A, Frame& B);

	/// This function estimates a frame difference
	double FrameDifferenceSimple(const Frame& A, double RotInfluence=ROT_FACTOR);
	double FrameDifference(const Frame& A, double RotInfluence=ROT_FACTOR);
	double Norm(double RotInfluence=ROT_FACTOR);


	void LimitRotation();

	int GetCentralFrame(const PointCloud& PCl);

	int ToWorld(const Frame& A, Frame& B);
	inline int ToWorld(Frame& A) {return ToWorld(A, A);};
	int ToFrame(const Frame& A, Frame& B);
	inline int ToFrame(Frame& A) {return ToFrame(A, A);};

	void Invert();

};

/// Class to represent a frame list.
class FrameList : public std::list<Frame>
{
public:
	std::string Str();
	void QTClustering(double Eps, unsigned int MinN=1, bool UseMean=false);
};

/// A list of frames (x, y, z, alpha, beta, gamma) with additional information
class FrameStruct
{
public:
	FrameStruct(){};
	FrameStruct(Frame F, double S){m_F=F; m_S=S;};// m_u=u; m_v=v;}
	~FrameStruct(){};
	Frame m_F; ///< The coordinate frame. 
	double m_S; ///< Scalar that can be used to include a count or probability or error for the frame.
	//double m_u;
	//double m_v;
	std::string Str();
};

bool FrameStructGreater(const FrameStruct& fs1, const FrameStruct& fs2);

/// A list of frame structs, each frame represents a vote
class VotingList : public std::vector<FrameStruct>
{
public:
	VotingList(){m_SumS=0;};
	VotingList(const FrameStruct& FS) {push_back(FS); m_SumS=FS.m_S;};
	~VotingList(){};

	/// Returns the center of gravity of all frames.
	/// @param center The center of gravity
	/// @return Return code
	unsigned long GetCenterOfGravity(FrameStruct *center);

	inline double AppendVote(const FrameStruct& FS){push_back(FS); m_SumS+=FS.m_S; return m_SumS;};
	void GetCentralFrameStruct(FrameStruct& FS);
	void GetCentralFrameStructDensity(FrameStruct& FS, double Eps, bool Mean=false);
	std::string Str();
	double m_SumS;
};

void GeometryTestFrames();
void GeometryTestMeanNorm();

} // end namespace ipa_Utils

#endif // __THREEDUTILS_H__

