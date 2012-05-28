////////////////////////////////////////////////////////////////////////////////////////////////////
//  fxPhysMaterial.cc
//  Chris Calef
//
////////////////////////////////////////////////////////////////////////////////////////////////////
//#include "core/stl_fix.h"

#include "T3D/physicsBAG/physManager.h"
#include "T3D/physicsBAG/nxPhysManager.h"
//#include "T3D/physicsBAG/odePhysManager.h"
#include "T3D/physicsBAG/fxPhysMaterial.h"
#include "console/consoleTypes.h"
#include "core/stream/bitstream.h"

//#include "editor/editor.h"
#include "math/mathio.h"

IMPLEMENT_CO_DATABLOCK_V1(fxPhysMaterial);


fxPhysMaterial::fxPhysMaterial()
{	
	mTextureName = NULL;
	mPhysMaterial = NULL;
	mIndex = 0;
	mDynamicFriction = 0.5;
	mStaticFriction = 0.5;
	mRestitution = 0.5;
	mDensity = 1000.0;
}

fxPhysMaterial::~fxPhysMaterial()
{

}

bool fxPhysMaterial::onAdd()
{
	if(!Parent::onAdd()) return false;
   S32 physType = physManagerCommon::mPM->getType();
	if (physType==PHYS_NX) 
		mPhysMaterial = (physMaterial *) (new nxPhysMaterial(mDynamicFriction,mStaticFriction,mRestitution,mDensity));
	//else if (physType==PHYS_ODE) 
		//mPhysMaterial = (physMaterial *) (new odePhysMaterial(mDynamicFriction,mStaticFriction,mRestitution,mDensity));

	return true;
}

void fxPhysMaterial::onRemove()
{
	Parent::onRemove();

	delete mPhysMaterial;
}


void fxPhysMaterial::getIndex()
{

	mIndex = mPhysMaterial->getIndex();

}

void fxPhysMaterial::initPersistFields()
{
  Parent::initPersistFields();
  addField("TextureName",TypeString,Offset(mTextureName, fxPhysMaterial));
  addField("DynamicFriction", TypeF32,Offset(mDynamicFriction, fxPhysMaterial));
  addField("StaticFriction", TypeF32,Offset(mStaticFriction, fxPhysMaterial));
  addField("Restitution", TypeF32,Offset(mRestitution, fxPhysMaterial));
  addField("Density", TypeF32,Offset(mDensity, fxPhysMaterial));
}

void fxPhysMaterial::packData(BitStream* pBitStream)
{
	Parent::packData(pBitStream);
	pBitStream->writeString(mTextureName);
	pBitStream->write(mDynamicFriction);
	pBitStream->write(mStaticFriction);
	pBitStream->write(mRestitution);
	pBitStream->write(mDensity);
	return;
}

void fxPhysMaterial::unpackData(BitStream *pBitStream)
{
	Parent::unpackData(pBitStream);
	mTextureName = pBitStream->readSTString();
	pBitStream->read(&mDynamicFriction);
	pBitStream->read(&mStaticFriction);
	pBitStream->read(&mRestitution);
	pBitStream->read(&mDensity);
}

void fxPhysMaterial::copy(fxPhysMaterial *other)
{
	mTextureName = other->mTextureName;
	mIndex = other->mIndex;
	mDynamicFriction = other->mDynamicFriction;
	mStaticFriction = other->mStaticFriction;
	mRestitution = other->mRestitution;
	mDensity = other->mDensity;
}
