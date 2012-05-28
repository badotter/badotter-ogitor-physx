////////////////////////////////////////////////////////////////////////////////////////////////////
//  physMaterialCommon
//  Chris Calef
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef	_PHYSMATERIALCOMMON_H_
#define _PHYSMATERIALCOMMON_H_

#include "T3D/physicsBAG/physMaterial.h"


class physMaterialCommon : public physMaterial
{
 public:

	 U32 mIndex;
	 F32 mDynamicFriction;
	 F32 mStaticFriction;
	 F32 mRestitution;
	 F32 mDensity;

	 physMaterialCommon() {};
	 ~physMaterialCommon() {};

	 S32 getIndex() { return mIndex; }
	 void setIndex(S32 m) { mIndex = m; }

	 F32 getDynamicFriction() { return mDynamicFriction; }
	 void setDynamicFriction(F32 m) { mDynamicFriction = m; }

	 F32 getStaticFriction() { return mStaticFriction; }
	 void setStaticFriction(F32 m) { mStaticFriction = m; }

	 F32 getRestitution() { return mRestitution; }
	 void setRestitution(F32 m) { mRestitution = m; }

	 F32 getDensity() { return mDensity; }
	 void setDensity(F32 m) { mDensity = m; }
};

#endif 
