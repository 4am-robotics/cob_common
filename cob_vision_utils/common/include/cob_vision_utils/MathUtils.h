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
* Description: Basic utilities. These include double and integer vectors and matrices,
* basic linear algebra tools (solving a linear equation system), and simple
* regression functions.
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
/// Basic utilities. These include double and integer vectors and matrices,
/// basic linear algebra tools (solving a linear equation system),
/// and simple regression functions.
/// This file and the corresponding .cpp file were written by Jens Kubacki and Jan Fischer in 2005-2008.
/// Latest updates: November 2008.

#ifndef __IPA_MATHUTILS_H__
#define __IPA_MATHUTILS_H__

#ifdef __COB_ROS__
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <libwm4/Wm4ApprPolyFit2.h>
 
#include "cob_vision_utils/GlobalDefines.h"
#else
#include <cv.h>
#include <highgui.h>

#include <Wm4ApprPolyFit2.h>

#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/GlobalDefines.h"
#endif

#include <vector>
#include <iostream>
#include <time.h>
#include <limits.h>
#include <fstream>
#include <map>

#include <sstream>
#include <algorithm>

#ifndef __LINUX__
#pragma warning (disable: 4786)
#endif

namespace ipa_Utils {
/// Vector (arbitrary dimensions) with auxillary functions for auxillary types
/// This vector class is derived from public std::vector<T>.
/// Therefore, all standard methods are available and additional member methods as described in this documentation.
template <class T> 
class IpaVector : public std::vector<T>
{
public:

	IpaVector(){};
	~IpaVector(){};
	
	/// Basic initialization.
	/// This is derived from the standard assign() function.
	/// @param Size The number of times <code>Val</code> is added to the vector
	/// @param Val the actual value for all components
	void Assign(int Size, T Val){this->assign(Size, Val);}
	
	/// Initialize with counting upwards.
	/// @param Size The number of times it is couted upwards
	void AssignCounter(int Size);
	
	/// String conversion. This function can be used for console output or serialization.
	/// @return the string itself
	std::string Str() const;
	
	/// Loading function.
	/// @param Name The filename as standard string
	/// @param Append a flag that switches on a consecutive loading behavior
	/// @return RET_OK if file could be loaded, RET_FAILED otherwise
	unsigned long Load(std::string Name, bool Append=false);
	
	/// Saving function.
	/// @param Name the name as stadard string
	/// @param Append a flag that switches on a consecutive saving behavior
	/// @return RET_OK if file could be saved, RET_FAILED otherwise
	unsigned long Save(std::string Name, bool Append=false) const;
	
	/// Random element assignment.
	/// Use this function if you want to set the members randomly between Min and Max (both included).
	///	The original size will be sustained unless Size!=NULL. Then the new size (*Size) will be used.
	/// @param Min minimal possible value
	/// @param Max maximal possible value
	/// @param Size pointer to a new size
	void WriteRandom(int Min, int Max, unsigned int* Size=NULL);

	/// Find the minimum and maximum.
	/// Find the minimum and maximum with optional positions.
	/// @param Min the minimum value in the vector
	/// @param Max the maximum value in the vector
	/// @param MinPos the position of the minimum value in the vector
	/// @param MaxPos the position of the maximum value in the vector
	/// @return RET_FAILED, when the vector is empty. RET_OK otherwise
	unsigned long MinMax(T& Min, T& Max, int& MinPos, int& MaxPos) const;
	
	/// Check if the value is in the vector.
	/// Check if the value is in the vector and return true if so.
	/// @param Val the value to be checked
	/// @param Where optional position of the value (first position in the case of multiple occurences)
	/// @return true if the value is at least once occuring in the vector, false otherwise
	bool IsIn(T Val, int* Where=NULL) const;

	/// Returns the sum of the elements within the vector
	/// @return The sum of all elements within the vector
	T Sum();
};


/// Integer vector (arbitrary dimensions) with auxillary functions.
/// This vector class is derived from public std::vector<int>.
/// Therefore, all standard methods are available and additional member methods as described in this documentation.
class IntVector : public std::vector<int>
{
public: 
	IntVector(){};
	//IntVector(int i0, int i1=0, int i2=0);//, int i3=0, int i4=0);
	~IntVector(){};
	
	/// Basic initialization.
	/// This is derived from the standard assign() function.
	/// @param Size the size in int
	/// @param Val the actual value for all components
	void Assign(int Size, int Val){this->assign(Size, Val);}
	
	/// Initialize with counting upwards.
	void AssignCounter(int Size);
	
	/// String conversion. This function can be used for console output or serialization.
	/// @return the string itself
	std::string Str() const;
	
	/// Loading function.
	/// @param Name the name as stadard string
	/// @param Append a flag that switches on a consecutive loading behavior
	/// @return RET_OK if file could be loaded, RET_FAILED otherwise
	unsigned long Load(std::string Name, bool Append=false);
	
	/// Saving function.
	/// @param Name the name as stadard string
	/// @param Append a flag that switches on a consecutive saving behavior
	/// @return RET_OK if file could be saved, RET_FAILED otherwise
	unsigned long Save(std::string Name, bool Append=false) const;
	
	/// Random element assignment.
	/// Use this function if you want to set the members randomly between Min and Max (both included).
	///	The original size will be sustained unless Size!=NULL. Then the new size (*Size) will be used.
	/// @param Min minimal possible value
	/// @param Max maximal possible value
	/// @param Size pointer to a new size
	void WriteRandom(int Min, int Max, unsigned int* Size=NULL);
	
	/// Random set assignment.
	/// Use this function if you want to set the members randomly between Min and Max (both included).
	///	However each member can only appear once (like in a set).
	/// @param Min minimal possible value
	/// @param Max maximal possible value
	/// @param Size pointer to a new size
	/// @return RET_OK if dimensionality is ok, RET_FAILED otherwise
	unsigned long WriteRandomSet(int Min, int Max, unsigned int* Size=NULL);

	/// Find the minimum and maximum.
	/// Find the minimum and maximum with optional positions.
	/// @param Min the minimum value in the vector
	/// @param Max the maximum value in the vector
	/// @param MinPos the position of the minimum value in the vector
	/// @param MaxPos the position of the maximum value in the vector
	void MinMax(int& Min, int& Max, int& MinPos, int& MaxPos) const;
	
	/// Check if the value is in the vector.
	/// Check if the value is in the vector and return true if so.
	/// @param Val the value to be checked
	/// @param Where optional position of the value (first position in the case of multiple occurences)
	/// @return true if the value is at least once occuring in the vector, false otherwise
	bool IsIn(int Val, int* Where=NULL) const;

	int Sum();
};

/// A "greater" comparison between two IntVectors.
/// The longer (higher dimensional) vector is considered to be larger.
bool IntVectorGreater(const IntVector& v1, const IntVector& v2);
//bool IntVectorSmaller(const IntVector& v1, const IntVector& v2);

/// A function to compare IntVectors.
/// This function can be used to compare IntVectors in order to realize a standard map with IntVector.
/*
struct CompIntVector
{
	bool operator()( const IntVector& v1, const IntVector& v2 ) const
	{
		for(unsigned int i=0; i<v1.size(); i++)
		{
			if(v1[i]<v2[i]) return true;
			else return false;
		}
		return false;
    }
};
*/
/// Double vector (arbitrary dimensions) with auxillary functions.
/// This vector class is derived from public std::vector<double>.
/// Therefore, all standard methods are available and additional member methods as described in this documentation.
class DblVector : public std::vector<double>
{
public:
	DblVector(){};
	~DblVector(){};

	/// Basic initialization.
	/// This is derived from the standard assign() function.
	/// @param Size the size in int
	/// @param Val the actual value for all components
	void Assign(int Size, double Val){this->assign(Size, Val);}
	
	/// String conversion. This function can be used for console output or serialization.
	/// @return the string itself
	std::string Str() const;

	/// Loading function.
	/// @param Name the name as stadard string
	/// @param Append a flag that switches on a consecutive loading behavior
	/// @return RET_OK if file could be loaded, RET_FAILED otherwise
	unsigned long Load(std::string Name, bool Append=false);

	/// Saving function.
	/// @param Name the name as stadard string
	/// @param Append a flag that switches on a consecutive saving behavior
	/// @return RET_OK if file could be saved, RET_FAILED otherwise
	unsigned long Save(std::string Name, bool Append=false) const;
	
	/// Random element assignment.
	/// Use this function if you want to set the members randomly between Min and Max (both included).
	///	The original size will be sustained unless Size!=NULL. Then the new size (*Size) will be used.
	/// @param Min minimal possible value
	/// @param Max maximal possible value
	/// @param Size pointer to a new size
	/// @param Int make the values integer (frac(x)=0.0)
	void WriteRandom(double Min, double Max, unsigned int Size=0, bool Int=false);
	//void AppendRandom(double Min, double Max, int Count, bool Int=false);
	
	/// Find the minimum and maximum.
	/// Find the minimum and maximum with optional positions.
	/// @param Min the minimum value in the vector
	/// @param Max the maximum value in the vector
	/// @param MinPos the position of the minimum value in the vector
	/// @param MaxPos the position of the maximum value in the vector
	void MinMax(double& Min, double& Max, int& MinPos, int& MaxPos) const;

	/// Vector addition.
	/// Res[i] = (*this)[i] + B[i]
	/// @param B argument vector
	/// @param Res result vector
	void AddVec(const DblVector& B, DblVector& Res) const;

	/// Vector addition (in-place).
	/// (*this)[i] += B[i]
	/// @param B argument vector
	void AddToVec(const DblVector& B);

	/// Vector substraction.
	/// Res[i] = (*this)[i] - B[i]
	/// @param B argument vector
	/// @param Res result vector
	void SubVec(const DblVector& B, DblVector& Res) const;

	/// Vector substraction (in-place).
	/// (*this)[i] -= B[i]
	/// @param B argument vector
	void SubFromVec(const DblVector& B);

	/// Elementwise multiplication.
	/// Res[i] = (*this)[i] * B[i]
	/// @param B argument vector
	/// @param Res result vector
	void MulElem(const DblVector& B, DblVector& Res) const; 
	
	/// Elementwise multiplication (in-place).
	/// (*this)[i] *= B[i]
	/// @param B argument vector
	void MulElemTo(const DblVector& B);
	
	/// Multiplication of elements with a scalar.
	/// Res[i] = (*this)[i] * s
	/// @param s the scalar
	/// @param Res result vector
	void ScalarMul(double s,  DblVector& Res) const;

	/// Multiplication of elements with a scalar (in-place).
	/// (*this)[i] * =s
	/// @param s the scalar
	void ScalarMulTo(double s);
	
	/// Compute magnitude.
	/// Compute the magnitude as sqrt(sum(((*this)[i])^2)/N)).
	/// @return magnitude
	double GetMagnitude() const;

	/// Compute squared magnitude.
	/// Compute the squared magnitude as sum(((*this)[i])^2)/N).
	/// @return squared magnitude
	double GetMagnitudeSqr() const;

	// Normalize to sum((*this)[i]
	//void Normalize(DblVector& Res) const;
	//void Normalize();
	
	/// Get the Euclidian distance.
	/// Get the Euclidian distance between two vectors.
	/// @param B the argument vector
	/// @return the Euclidian distance
	double EuclDist(const DblVector& B) const;

	/// Get the squared Euclidian distance.
	/// Get the squared Euclidian distance between two vectors.
	/// @param B the argument vector
	/// @return the squared Euclidian distance
	double EuclDistSqr(const DblVector& B) const;
	
	// scalar product of two vectors
	//double ScalarProduct(const DblVector& B);
	std::string ExportForPSTricks();
	int SavePSTricks(std::string Name);
};


/// Class for sorting a DblVector
class SortStruct
{
public:
	SortStruct(double V=0.0, unsigned int N=0) {m_V=V; m_N=N;}
	double m_V; ///< Scalar value for sorting.
	unsigned int m_N;	///< Position.
};

/// Function to compare two gradient differences
bool SortStructGreater(const SortStruct& s0, const SortStruct& s1);

class SortedVector : public std::vector<SortStruct>
{
public:
	SortedVector(const DblVector& v=DblVector());
	~SortedVector(){};
};

/// Integer matrix with auxillary functions.
/// This matrix class is derived from public std::vector<IntVector>.
/// Therefore, all standard methods are available and additional member methods as described in this documentation.
class IntMatrix : public std::vector<IntVector>
{
public:

	/// Contructor.
	/// Contructor that allocates storage (w=width, h=height, v=value).
	/// @param w width
	/// @param h height
	/// @param v value
	IntMatrix(int w=0, int h=0, int v=0) {Assign(w, h, v);};
	
	~IntMatrix(){};
	
	/// Basic initialization.
	/// This is derived from the standard assign() function.
	/// @param w width
	/// @param h height
	/// @param Val value
	void Assign(int w, int h, int Val=0);
	
	/// Get the width.
	/// Get the width of the matrix.
	/// @return width 
	int GetWidth() const {return (int)((*this)[0].size());}
	
	/// Get the height.
	/// Get the height of the matrix.
	/// @return height
	int GetHeight() const {return (int)(this->size());}
	
	/// String conversion. This function can be used for console output or serialization.
	/// @return the string itself
	std::string Str() const;
	
	/// Loading function.
	/// @param Name the name as stadard string
	/// @param Append a flag that switches on a consecutive loading behavior
	/// @return RET_OK if file could be loaded, RET_FAILED otherwise
	unsigned long Load(std::string Name, bool Append=false);
	
	/// Saving function.
	/// @param Name the name as stadard string
	/// @param Append a flag that switches on a consecutive saving behavior
	/// @return RET_OK if file could be saved, RET_FAILED otherwise
	unsigned long Save(std::string Name, bool Append=false) const;
	
	/// Make a matrix.
	/// Make a matrix from a vector.
	/// @param v vector
	void MakeMat(const IntVector& v);
	
	/// Make a vector.
	/// Make a vector from a matrix.
	/// @param v vector
	void MakeVec(IntVector& v) const;

	/// Random element assignment.
	/// Use this function if you want to set the members randomly between Min and Max (both included).
	///	The original size will be sustained unless Size!=NULL. Then the new size (*Size) will be used.
	/// @param Min minimal possible value
	/// @param Max maximal possible value
	void WriteRandom(int Min, int Max);
	
	/// Find the minimum and maximum.
	/// Find the minimum and maximum with optional positions.
	/// @param Min the minimum value in the vector
	/// @param Max the maximum value in the vector
	void MinMax(int& Min, int& Max) const;

	void EraseVector(IntVector& v);
};

/// Double float matrix with auxillary functions.
/// This matrix class is derived from public std::vector<DblVector>.
/// Therefore, all standard methods are available and additional member methods as described in this documentation.
class DblMatrix : public std::vector<DblVector>
{
public:
	
	/// Basic initialization.
	/// This is derived from the standard assign() function.
	/// @param w width
	/// @param h height
	/// @param Val value
	void Assign(int w, int h, double Val=0.0);
	
	/// Get the width.
	/// Get the width of the matrix.
	/// @return width 
	int GetWidth() const {return (int)((*this)[0].size());}
	
	/// Get the height.
	/// Get the height of the matrix.
	/// @return height
	int GetHeight() const {return (int)(this->size());}
	
	/// String conversion. This function can be used for console output or serialization.
	/// @return the string itself
	std::string Str() const;

	/// Loading function.
	/// @param Name the name as stadard string
	/// @param Append a flag that switches on a consecutive loading behavior
	/// @return RET_OK if file could be loaded, RET_FAILED otherwise
	unsigned long Load(std::string Name, bool Append=false);
	
	/// Saving function.
	/// @param Name the name as stadard string
	/// @param Append a flag that switches on a consecutive saving behavior
	/// @return RET_OK if file could be saved, RET_FAILED otherwise
	unsigned long Save(std::string Name, bool Append=false) const;
	
	/// Make a matrix.
	/// Make a matrix from a vector.
	/// @param v vector
	void MakeMat(const DblVector& v);
	
	/// Make a vector.
	/// Make a vector from a matrix.
	/// @param v vector
	void MakeVec(DblVector& v) const;

	/// Random element assignment.
	/// Use this function if you want to set the members randomly between Min and Max (both included).
	///	The original size will be sustained unless Size!=NULL. Then the new size (*Size) will be used.
	/// @param Min minimal possible value
	/// @param Max maximal possible value
	/// @param Int set to integer values (frac(x)=0.0)
	void WriteRandom(double Min, double Max, bool Int=false);

	/// Find the minimum and maximum.
	/// Find the minimum and maximum with optional positions.
	/// @param Min the minimum value in the vector
	/// @param Max the maximum value in the vector
	void MinMax(double& Min, double& Max) const;
	
	/// Get a selected line.
	/// Get a selected line from a matrix as vector.
	/// @param j line address
	/// @param Line output vector
	/// @param Step filter
	void GetLine(int j, DblVector& Line, int Step=1) const;
	
	/// Get a selected column.
	/// Get a selected column from a matrix as vector.
	/// Find the minimum and maximum with optional positions.
	/// @param j line address
	/// @param Column output vector
	/// @param Step filter
	void GetColumn(int j, DblVector& Column, int Step=1) const;

	/// useful maths functions
	// diagonalization
	void Diagonalize();
	double GetDeterminant2x2(); // sarrus
	double GetDeterminant3x3(); // sarrus
	double GetDeterminantNxN(); // laplace
	// sub-determinant
	double GetHelpDeterminant(int k);
	// solve linear NxN equation system (cramer)
	// matrix
	// a11 .. a1n
	// an1 .. ann
	// [a1n .. ann] containing the offsets
	int SolveLinearCramer(DblVector& Solution);
	void SaveAsBMP(std::string Name);
	void SaveHistogram(std::string Name, int NoBuckets);
};

/// Double float pointer matrix.
/// This matrix class is derived from public std::vector<DblVector*>.
class DblPtrMat : public std::vector<DblVector*>
{
public:
	DblPtrMat(){};
	~DblPtrMat(){};

	/// Get a pointer to a vector.
	DblVector* GetVecPtr(int i) {return (*this)[i];}
	
	/// String conversion.
	std::string Str() const;
	
	/// Import from DblMatrix.
	void ImportFromDblMatrix(DblMatrix& m); 
};

/// Integer tensor (3D cube).
/// This tensor class is derived from public std::vector<IntMatrix>.
class IntTensor : public std::vector<IntMatrix>
{
public:

	/// Constructor.
	/// @param Width width
	/// @param Height height
	/// @param Depth depth
	IntTensor(int Width, int Height, int Depth);
	~IntTensor(){};
};

/// Double minimum function. Returns the minimum of two arguments.
/// @param a first argument
/// @param b second argument
inline double dblmin(double a, double b) {return (a<=b) ? a : b;}

/// Double minimum function. Returns the minimum of three arguments.
/// @param a first argument
/// @param b second argument
/// @param c third argument
inline double dblmin(double a, double b, double c) {return dblmin(a, dblmin(b,c));}

/// Double maximum function. Returns the maximum of two arguments.
/// @param a first argument
/// @param b second argument
inline double dblmax(double a, double b) {return (a>=b) ? a : b;}

/// Double maximum function. Returns the maximum of three arguments.
/// @param a first argument
/// @param b second argument
/// @param c third argument
inline double dblmax(double a, double b, double c) {return dblmax(a, dblmax(b,c));}

/// Integer minimum function. Returns the minimum of two arguments.
/// @param a first argument
/// @param b second argument
inline int intmin(int a, int b) {return (a<=b) ? a : b;}

/// Integer minimum function. Returns the minimum of three arguments.
/// @param a first argument
/// @param b second argument
/// @param c third argument
inline int intmin(int a, int b, int c) {return intmin(a, intmin(b,c));}

/// Integer maximum function. Returns the maximum of two arguments.
/// @param a first argument
/// @param b second argument
inline int intmax(int a, int b) {return (a>=b) ? a : b;}

/// Integer maximum function. Returns the maximum of three arguments.
/// @param a first argument
/// @param b second argument
/// @param c third argument
inline int intmax(int a, int b, int c) {return intmax(a, intmax(b,c));}

/// Double square function. Returns the square of the argument.
/// @param x argument
inline double dblsqr(double x) {return x*x;}

/// Random initialization.
inline void InitRand()
{
	srand((unsigned)time(NULL) );
}

/// Random function double (not tested in Linux).
/// @param Min the minimal possible value
/// @param Max the maximal possible value
template <class T>
inline T RandT(T Min, T Max)
{
	T r = (T)rand()*(Max-Min)/(T)(RAND_MAX)+Min; 
	if (r < Min) r = Min;
	else if (r > Max) r = Max;
	return r;
}

/// Random function double (not tested in Linux).
/// @param Min the minimal possible value
/// @param Max the maximal possible value
inline double RandDbl(double Min, double Max)
{
	double r = (double)rand()*(Max-Min)/(double)(RAND_MAX)+Min; 
	return dblmin(Max, dblmax(Min, r));
}

/// Random function integer (not tested in Linux).
/// @param Min the minimal possible value
/// @param Max the maximal possible value
inline int RandInt(int Min, int Max)
{
	int rnd = rand()*(Max-Min+1)/RAND_MAX+Min;
	return intmin(rnd, Max);
}

/// Gaussian curve computation. Computes the value of a Gaussian for an x given the mean and sigma of the curve.
/// @param x argument
/// @param Mean the mean (position) of the Gaussian
/// @param Sigma the sigma (width) of the Gaussian
/// @return the value of the curve at x 
inline double Gauss(double x, double Mean, double Sigma)
{
	double Fac1=1.0/(sqrt(2*THE_PI_DEF)*Sigma);
	double ex=(x-Mean)/Sigma;
	double Fac2=exp(-0.5*ex*ex);
	return Fac1*Fac2;
}

/// Computes the sign. The sign of a double. Note that it is 1.0 for x >= 0.
/// @param x argument
/// @return the sign
inline double Sign(double x)
{
	if(x<0) return -1.0;
	else return 1.0;
}

/// Casts a small number to THE_EPS_DEF.
/// @param x argument
/// @return x or THE_EPS_DEF with the sign of x
inline double NonZero(double x)
{
	if(fabs(x)<THE_EPS_DEF)
		return Sign(x) * THE_EPS_DEF;
	else return x;
}

/// Compute y=a*x+b.
/// @param a coefficient
/// @param b offset
/// @param x argument
inline double LineVal(double a, double b, double x) {return a*x+b;} 

/// Compute optimal a and b for y[n]=a*x[n]+b.
/// Here the positions are aquidistant.
/// @param PointsY a DblVector of y values
/// @param a output coefficient
/// @param b output offset
int FitLine(const DblVector& PointsY, double& a, double& b);

/// Compute optimal a and b for y[n]=a*x[n]+b.
/// @param PointsY a DblVector of y values (function valus)
/// @param PointsX a DblVector of x values (positions)
/// @param a output coefficient
/// @param b output offset
int FitLine(const DblVector& PointsY, const DblVector& PointsX, double& a, double& b);

/// Compute optimal a and b for y[n]=a*x[n]+b with outlier control.
/// This function works as Fitline() but applies a Sigma-filtering that removes outliers.
/// After the removal of outliers the function is refitted.
/// @param PointsY a DblVector of y values (function valus)
/// @param PointsX a DblVector of x values (positions)
/// @param a output coefficient
/// @param b output offset
/// @param It maximal number of iterations
/// @param ErrWidth parameter to decide if an y value is an outlier or not
int FitLineIt(const DblVector& PointsY, const DblVector& PointsX,
			  double& a, double& b, int It=2, double ErrWidth=2.0);

/// Fits a polynomial to a given sets of data points y = f(x).
/// @param y The 'y' data points
/// @param x The 'x' data points
/// @param degree The degree of the polynomial
/// @param coefficients The resulting coefficients. Assert that you initialized the array to size 'degree+1'
/// @return Return code
unsigned long FitPolynomial(const DblVector& y, const DblVector& x, int degree, double* coefficients);

/// Evaluates a polynomial, calculated with <code>FitPolynomial</code>
/// @param x The function value
/// @param degree The degree of the polynomial
/// @param coefficients Pointer to an array of the <code>degree</code>+1 coefficients fo the polynomial
/// @param y The resulting function value
/// @return Return code
unsigned long EvaluatePolynomial(double x, int degree, double* coefficients, double* y);

/// Fits a polynomial to a given sets of data points y = f(x).
/// Data is normalized before fitting for numerical stability. In order to evaluate the
/// polynomial at position x, call </code>EvaluateNormalizedPolynomial<code>
/// @param y The 'y' data points
/// @param x The 'x' data points
/// @param degree The degree of the polynomial
/// @param coefficients The resulting NORAMLIZED coefficients. Assert that you initialized the array to size 'degree+1'
/// @param min The minimal unnormalized coefficient value.
/// @param max The maximal unnormalized coefficient value.
/// @return Return code
unsigned long FitNormalizedPolynomial(const DblVector& y, const DblVector& x, int degree, double* coefficients, double* min, double* max);

/// Evaluates a polynomial, calculated with <code>FitNormalizedPolynomial</code>
/// @param x The function value
/// @param degree The degree of the polynomial
/// @param coefficients Pointer to an array of the <code>degree</code>+1 coefficients fo the polynomial
/// @param min The minimal value of the original coefficients
/// @param max The maximal value of the original coefficients
/// @param y The resulting function value
/// @return Return code
unsigned long EvaluateNormalizedPolynomial(double x, int degree, double* coefficients, double min, double max, double* y);


/// Compute y = ax^2+bx+c.
/// @param a quadratic coefficient
/// @param b linear coefficient
/// @param c offset
/// @param x argument
inline double ParabolaVal(double a, double b, double c, double x) {return a*x*x+b*x+c;}

/// Compute optimal a, b, and c for y[n]=a*x[n]*x[n]+b*x[n]+c.
/// Here the positions are aquidistant.
/// @param PointsY a DblVector of y values
/// @param a output quadratic coefficient
/// @param b output linear coefficient
/// @param c output offset
int FitParabola(const DblVector& PointsY, double& a, double& b, double& c);

/// Compute optimal a, b, and c for y[n]=a*x[n]*x[n]+b*x[n]+c.
/// @param PointsY a DblVector of y values
/// @param PointsX a DblVector of x values
/// @param a output quadratic coefficient
/// @param b output linear coefficient
/// @param c output offset
int FitParabola(const DblVector& PointsY, const DblVector& PointsX, double& a, double& b, double& c);

/// Compute z=ax^2+by^2+cxy+dx+ey+f
/// @param x the x position
/// @param y the y position
/// @param Params parameter vector [a,b,c,d,e,f]
inline double QuadraticSurfaceVal(double x, double y, const DblVector& Params)
{
	return Params[5]*x*x+Params[4]*y*y+Params[3]*x*y
		+ Params[2]*x+Params[1]*y+Params[0];
}

/// Fit an optimal two dimensional quadratic surface z[n]=ax[n]^2+by[n]^2+cx[n]y[n]+dx[n]+ey[n]+f.
/// @param PX vector of x values
/// @param PY vector of y values
/// @param PZ vector of z values
/// @param Parameters output vector of the estimated parameters
int FitQuadraticSurface(const DblVector& PX, const DblVector& PY,
							const DblVector& PZ, DblVector& Parameters);

/// Compute the quadratic error of z[n]=ax[n]^2+by[n]^2+cx[n]y[n]+dx[n]+ey[n]+f.
/// @param PX vector of x values
/// @param PY vector of y values
/// @param PZ vector of z values
/// @param Parameters vector of the parameters
double GetQuadraticSurfaceError(const DblVector& PX, const DblVector& PY,
							const DblVector& PZ, DblVector& Parameters);

/// Fit an optimal two dimensional quadratic surface z[n]=ax[n]^2+by[n]^2+cx[n]y[n]+dx[n]+ey[n]+f.
/// Compute optimal a and b for z[n]=ax[n]^2+by[n]^2+cx[n]y[n]+dx[n]+ey[n]+f with outlier control.
/// This function works as FitQuadraticSurface() but applies a Sigma-filtering that removes outliers.
/// After the removal of outliers the function is refitted.
/// @param PX vector of x values
/// @param PY vector of y values
/// @param PZ vector of z values
/// @param Parameters output vector of the estimated parameters
/// @param It maximal number of iterations
/// @param ErrWidth parameter to decide if an y value is an outlier or not
int FitQuadraticSurfaceIt(const DblVector& PX, const DblVector& PY,
							const DblVector& PZ, DblVector& Parameters, int It=2, double ErrWidth=2.0);

/// Integer to color conversion.
/// This function computes a color from the eight basic colors for each integer.
/// The function is typically used to annotate different class labels.
/// @param i the integer
/// @param r red component [0..255]
/// @param g green component [0..255]
/// @param b blue component [0..255]
void IntToCol(int i, int& r, int& g, int& b);

void HueToCol(int h, int hueVals, int& r, int& g, int& b); 

/// Computes hue, saturation and value.
/// Computes hue, saturation and value from the red, green, and blue components.
/// Note that hue, saturation and value are between [0.0..1.0].
/// @param r red component [0..255]
/// @param g green component [0..255]
/// @param b blue component [0..255]
/// @param h hue output component [0.0..1.0]
/// @param s saturation output component [0.0..1.0]
/// @param v value (grey) output component [0.0..1.0]
void GetHSV(double r, double g, double b, double& h, double& s, double& v);

/// Computes red, green and blue.
/// Computes hue, saturation and value from the red, green, and blue components.
/// Note that hue, saturation and value are between [0.0..1.0].
/// @param h hue component [0.0..1.0]
/// @param s saturation component [0.0..1.0]
/// @param v value (grey) component [0.0..1.0]
/// @param r red output component [0..255]
/// @param g green output component [0..255]
/// @param b blue output component [0..255]
void GetRGB(double h, double s, double v, double& r, double& g, double& b);

/// Interpolates an extremum of a (1D) function.
/// This function interpolates an extremum of a (1D) function by setting the dervative to zero.
/// The function values have to be given at three adjacent positions:
/// x1=x2-h, x1, and x3=x2+h and the function values are y1, y2, and y3.
/// The actual code is generated with the MATLAB code generation tool.
/// @param y1 the first function value
/// @param y2 the second function value
/// @param y3 the third function value
/// @param x2 the second function values position
/// @param h the width (h=x2-x1=x3-x2) 
double InterpolateExtr(double y1, double y2, double y3, double x2, double h);

/// 2D rotation. Rotates a point on a XY 2D plane.
/// @param x the x component of the position
/// @param y the y component of the position
/// @param phi the phi component of the position
/// @param x_out the x component of the computed position
/// @param y_out the y component of the computed position
inline void Rotate2D(double x, double y, double phi, double& x_out, double& y_out)
{
	double SinPhi = sin(phi);
	double CosPhi = cos(phi);
	x_out = x*CosPhi + y*SinPhi;
	y_out = -x*SinPhi + y*CosPhi;
}

/// Get a vector from an IplImage.
/// @param Img the IPL image pointer
/// @param v the output vector
/// @param Scale optional scaling of the values
void GetVectorFromImage(IplImage* Img, DblVector& v, CvScalar* Scale=NULL);

} // end namespace ipa_Utils

#endif // __IPA_MATHUTILS_H__
