////////////////////////////////////////////////////////////////////////////////////////////////////
//  physMaterial.h
//  Chris Calef
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef	_PHYSMATERIAL_H_
#define _PHYSMATERIAL_H_

#include "T3D/physicsBAG/physManager.h"

class physMaterial
{
 public:
  physMaterial() {};
  virtual ~physMaterial() {};

  virtual S32 getIndex()=0;
  virtual void setIndex(S32)=0;

  virtual F32 getDynamicFriction()=0;
  virtual void setDynamicFriction(F32)=0;

  virtual F32 getStaticFriction()=0;
  virtual void setStaticFriction(F32)=0;

  virtual F32 getRestitution()=0;
  virtual void setRestitution(F32)=0;
  
  virtual F32 getDensity()=0;
  virtual void setDensity(F32)=0;
};

#endif
