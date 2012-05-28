////////////////////////////////////////////////////////////////////////////////////////////////////
//  nxPhysMaterial.h
//  Chris Calef
//
//////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef	_NXPHYSMATERIAL_H_
#define _NXPHYSMATERIAL_H_

//#include "core/stl_fix.h"

#include "T3D/physicsBAG/physManagerCommon.h"
#include "physMaterialCommon.h"


class nxPhysMaterial : public physMaterialCommon
{
 public:

   nxPhysMaterial(F32,F32,F32,F32);
   ~nxPhysMaterial();
};

#endif 
