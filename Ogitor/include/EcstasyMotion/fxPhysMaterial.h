////////////////////////////////////////////////////////////////////////////////////////////////////
//  fxPhysMaterial.h
//  Chris Calef
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _FXPHYSMATERIAL_H_
#define _FXPHYSMATERIAL_H_


#include "T3D/physicsBAG/physManager.h"
#include "platform/platform.h"
#include "T3D/shapeBase.h"
#include "console/simBase.h"

#include "T3D/physicsBAG/nxPhysMaterial.h"
//#include "T3D/physicsBAG/odePhysMaterial.h"

class fxPhysMaterial : public GameBaseData 
{
	typedef GameBaseData Parent;

public: 
	DECLARE_CONOBJECT(fxPhysMaterial);

	U32 mIndex;
	F32 mDynamicFriction;
	F32 mStaticFriction;
	F32 mRestitution;
	F32 mDensity;

	physMaterial *mPhysMaterial;
	StringTableEntry mTextureName;

	fxPhysMaterial();
	~fxPhysMaterial();

	bool onAdd();
	void onRemove();
	static void initPersistFields();
    void packData(BitStream* pBitStream);
    void unpackData(BitStream *pBitStream);
	void getIndex();
	void copy(fxPhysMaterial*);
};


#endif
