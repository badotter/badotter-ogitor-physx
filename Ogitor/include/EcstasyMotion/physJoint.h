////////////////////////////////////////////////////////////////////////////////////////////////////
//  physJoint
//  Chris Calef 2006
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _PHYSJOINT_H_
#define _PHYSJOINT_H_

#include "EcstasyMotion/physManager.h"


struct physJointData
{

	enum physJointType mJointType;

	float mTwistLimit;
	float mSwingLimit;
	float mSwingLimit2;

	float mHighLimit;
	float mLowLimit;
	float mHighRestitution;
	float mLowRestitution;

	float mDistanceLimit;
	float mBreakingForce;
	float mBreakingTorque;

	float mMotorForce;
	float mMotorSpeed;

	float mJointSpring;
	float mSwingSpring;
	float mTwistSpring;
	float mSpringDamper;
	float mSpringTargetAngle;

	Ogre::Vector3 mAxisA,   mAxisB;//deprecated
	Ogre::Vector3 mNormalA, mNormalB;//deprecated

	Ogre::Vector3 mLocalAnchor0,   mLocalAnchor1;
	Ogre::Vector3 mLocalAxis0,   mLocalAxis1;
	Ogre::Vector3 mLocalNormal0,   mLocalNormal1;
	Ogre::Vector3 mGlobalAnchor, mGlobalAxis;

	Ogre::Vector3 mLimitPoint;
	Ogre::Vector3 mLimitPlaneAnchor1;
	Ogre::Vector3 mLimitPlaneNormal1;
	Ogre::Vector3 mLimitPlaneAnchor2;
	Ogre::Vector3 mLimitPlaneNormal2;
	Ogre::Vector3 mLimitPlaneAnchor3;
	Ogre::Vector3 mLimitPlaneNormal3;
	Ogre::Vector3 mLimitPlaneAnchor4;
	Ogre::Vector3 mLimitPlaneNormal4;

	bool mCollisionEnabled;
	bool mHW;
};

class physJoint
{

 public:

   physJoint(){};
   virtual ~physJoint(){};

   virtual void setup()=0;
   virtual void	onWorldStep()=0;
   virtual void setMotorVelocity(float)=0;
   virtual Ogre::Quaternion &getMotorTarget()=0;
   virtual void setMotorTarget(Ogre::Quaternion &)=0;
   virtual void setMotorSpring(Ogre::Quaternion &,float)=0;
   virtual Ogre::Quaternion &getLastTarget()=0;
   virtual void setLastTarget(Ogre::Quaternion &)=0;
   virtual Ogre::Quaternion &getNewTarget()=0;
   virtual void setNewTarget(Ogre::Quaternion &)=0;
   virtual void clearMotor()=0;

   virtual void setPM(physManager *)=0;
   //virtual void setJD(physJointData *)=0;

   virtual void setRB_A(physRigidBody *)=0;
   virtual void setRB_B(physRigidBody *)=0;

	virtual void setTwistLimit(float)=0;
	virtual void setSwingLimit(float)=0;
	virtual void setSwingLimit2(float)=0;
	
	virtual void setMaxForce(float)=0;
	virtual void setMaxTorque(float)=0;

   virtual void setAxisA(Ogre::Vector3 &)=0;
   virtual void setAxisB(Ogre::Vector3 &)=0;
   virtual void setNormalA(Ogre::Vector3 &)=0;
   virtual void setNormalB(Ogre::Vector3 &)=0;

   virtual void setLocalAnchor0(Ogre::Vector3 &)=0;
   virtual void setLocalAnchor1(Ogre::Vector3 &)=0;
   virtual void setLocalAxis0(Ogre::Vector3 &)=0;
   virtual void setLocalAxis1(Ogre::Vector3 &)=0;
   virtual void setLocalNormal0(Ogre::Vector3 &)=0;
   virtual void setLocalNormal1(Ogre::Vector3 &)=0;

   virtual void setGlobalAnchor(Ogre::Vector3 &)=0;
   virtual void setGlobalAxis(Ogre::Vector3 &)=0;

   virtual physJointType getJointType()=0;
	
   virtual void setJointType(physJointType)=0;
   virtual bool getHW()=0;
   virtual void setHW(bool b=true)=0;
};
#endif 
