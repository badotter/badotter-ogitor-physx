//////////////////////////////////////////////////////////////////////////////
// iPhysEvent
// Chris Calef, 2006
//
// Lightweight abstract base class for communication between physics entities
// and higher level classes that use them.  Include a pointer to this class
// in your physics class (esp. rigid body) and then derive the higher level
// classes from this as well as whatever else you're deriving from.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef _IPHYSUSER_H_
#define _IPHYSUSER_H_

//#include "EcstasyMotion/fxFlexBody.h"

enum physEntityType;
enum physEntitySubType;

class fxFlexBody;

class iPhysUser
{
 public:
	 fxFlexBody *mFlexBody;

	 // This is because many physics objects are associated with a flexbody (i.e. player or 
	 // monster/NPC/mob), and they may need to behave differently in interactions with that 
	 // flexbody as opposed to other flexbodies.  Esp. true for weapons.

	 Ogre::Vector3 mTempForce;
	 Ogre::Vector3 mTempPos;

	 float mTempDamage;
	 float mSleepThreshold;

	 bool mHasTempForce;
	 bool mHasTractorBeam;
	 bool mHasTractorSpring;
	 bool mHasSpring;

	 bool mHasWeapon;
	 bool mIsStriking;
	 bool mPositionDirty;

	 bool mIsInflictor;
	 float mInflictMultiplier;

	 physEntityType mEntityType;
	 physEntitySubType mEntitySubType;

	 iPhysUser(){};
	 virtual ~iPhysUser(){};

	 virtual void onWorldStep()=0;

	 virtual void onCollision(iPhysUser *other,int action=0)=0;//action is a user variable that can be defined as needed
	 virtual void onTrigger(iPhysUser *other,int action=0,int id=0)=0;
	 virtual void addForceAtPos(const Ogre::Vector3 &force,const Ogre::Vector3 &pos)=0;
	 virtual void lockTractorBeam(int)=0;

	 //virtual void setupDebugRender()=0;
};

#endif