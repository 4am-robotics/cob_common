
#ifdef __COB_ROS__
	#include "cob_vision_math/MathDefines.h"
#else
	#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/MathUtils.h"
#endif

using namespace ipa_Utils;

unsigned long ipa_Utils::TranslateRotate(ipa_Utils::Vector3d& pt, 
	const ipa_Utils::Vector3d& t, const ipa_Utils::Quaterniond& q)
{
	// Translate in oposite direction
	pt[0] -= t[0];
	pt[1] -= t[1];
	pt[2] -= t[2];

	// Rotate in oposite direction	
	Quaterniond qConjugate = q.Conjugate();
	pt = qConjugate.Rotate(pt);

	return RET_OK;
}

unsigned long ipa_Utils::TranslateRotate(ipa_Utils::Vector3d& pt, const ipa_Utils::Vec7d& transformation)
{
	// Convert
	Quaterniond q(transformation[3], transformation[4], transformation[5], transformation[6]);
	ipa_Utils::Vector3d t(transformation[0], transformation[1], transformation[2]);
		
	return TranslateRotate(pt, t, q);
}

unsigned long ipa_Utils::RotateTranslate(ipa_Utils::Vector3d& pt, 
	const ipa_Utils::Vector3d& t, const ipa_Utils::Quaterniond& q)
{
	// Rotate	
	pt = q.Rotate(pt);
	
	// Translate
	pt[0] += t[0];
	pt[1] += t[1];
	pt[2] += t[2];

	return RET_OK;
}

unsigned long ipa_Utils::RotateTranslate(ipa_Utils::Vector3d& pt, const ipa_Utils::Vec7d& transformation)
{
	// Convert
	Quaterniond q(transformation[3], transformation[4], transformation[5], transformation[6]);
	ipa_Utils::Vector3d t(transformation[0], transformation[1], transformation[2]);
		
	return RotateTranslate(pt, t, q);
}

unsigned long ipa_Utils::ToSphere(ipa_Utils::Vector3d& pt)
{
	double r = pt.Length();
	double theta = std::acos(pt[2]/r);
	double phi = std::atan2(pt[1], pt[0]);
	pt[0] = r;
	pt[1] = theta;
	pt[2] = phi;

	return RET_OK;
}

unsigned long ipa_Utils::ToCartesian(ipa_Utils::Vector3d& pt)
{
	double SinT = sin(pt[1]);
	double CosT = cos(pt[1]);
	double SinP = sin(pt[2]);
	double CosP = cos(pt[2]);
	
	pt[0] = SinT*CosP;
	pt[1] = SinT*SinP;
	pt[2] = CosT;

	pt.Normalize();

	return RET_OK;
}


