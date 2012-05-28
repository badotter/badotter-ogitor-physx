////////////////////////////////////////////////////////////////////////////////////////////////////
//  nxPhysMaterial.cc
//  Chris Calef
//
//////////////////////////////////////////////////////////////////////////////////////////////////////

#include "T3D/physicsBAG/nxPhysMaterial.h"

nxPhysMaterial::nxPhysMaterial(F32 dF, F32 sF, F32 R, F32 D)
{
  
	mDynamicFriction = dF;
	mStaticFriction = sF;
	mRestitution = R;
	mDensity = D;
	physManagerCommon::getPM()->addPhysMaterial((physMaterial *)this);
  
}

nxPhysMaterial::~nxPhysMaterial()
{

}
