////////////////////////////////////////////////////////////////////////////////////////////////////
//  physJointCommon
//  Chris Calef
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _PHYSJOINTCOMMON_H_
#define _PHYSJOINTCOMMON_H_

#include "EcstasyMotion/physJoint.h"


class physJointCommon : public physJoint
{

public:
	physManager *mPM;
	//physJointData *mJD;

	physRigidBody  *mRB_A;
	physRigidBody  *mRB_B;

	enum physJointType mJointType;

	bool mHW;
	bool mCollisionEnabled;

	float mTwistLimit;
	float mSwingLimit;
	float mSwingLimit2;

	float mLowLimit;
	float mHighLimit;
	float mLowRestitution;
	float mHighRestitution;

	float mDistanceLimit;
	float mMaxForce;
	float mMaxTorque;

	float mMotorForce;
	float mMotorSpeed;

	float mJointSpring;
	float mSwingSpring;
	float mTwistSpring;
	float mSpringDamper;
	float mSpringTargetAngle;


	Ogre::Vector3 mAxisA;//deprecated
	Ogre::Vector3 mAxisB;//deprecated
	Ogre::Vector3 mNormalA;//deprecated
	Ogre::Vector3 mNormalB;//deprecated

	Ogre::Vector3 mLocalAnchor0;
	Ogre::Vector3 mLocalAnchor1;
	Ogre::Vector3 mLocalAxis0;
	Ogre::Vector3 mLocalAxis1;
	Ogre::Vector3 mLocalNormal0;
	Ogre::Vector3 mLocalNormal1;

	Ogre::Vector3 mGlobalAnchor;
	Ogre::Vector3 mGlobalAxis;

	Ogre::Quaternion mMotorTarget;
	Ogre::Quaternion mLastTarget;
	Ogre::Quaternion mNewTarget;

	Ogre::Vector3 mLimitPoint;
	Ogre::Vector3 mLimitPlaneAnchor1;
	Ogre::Vector3 mLimitPlaneNormal1;
	Ogre::Vector3 mLimitPlaneAnchor2;
	Ogre::Vector3 mLimitPlaneNormal2;
	Ogre::Vector3 mLimitPlaneAnchor3;
	Ogre::Vector3 mLimitPlaneNormal3;
	Ogre::Vector3 mLimitPlaneAnchor4;
	Ogre::Vector3 mLimitPlaneNormal4;

	physJointCommon(){};
	~physJointCommon(){};

	virtual void setPM(physManager *ptr) {mPM=ptr;}
	//virtual void setJD(physJointData *JD) {mJD = JD;}

	virtual void setRB_A(physRigidBody *ptr) {mRB_A = ptr;}
	virtual void setRB_B(physRigidBody *ptr) {mRB_B = ptr;}

	virtual void setTwistLimit(float f)  { mTwistLimit = f; }
	virtual void setSwingLimit(float f)  { mSwingLimit = f; }
	virtual void setSwingLimit2(float f) { mSwingLimit2 = f; }

	virtual void setMaxForce(float f)  { mMaxForce = f; }
	virtual void setMaxTorque(float f) { mMaxTorque = f; }

	virtual void setAxisA(Ogre::Vector3 &p)   {mAxisA   = p;}
	virtual void setAxisB(Ogre::Vector3 &p)   {mAxisB   = p;}
	virtual void setNormalA(Ogre::Vector3 &p) {mNormalA = p;}
	virtual void setNormalB(Ogre::Vector3 &p) {mNormalB = p;}

   virtual void setLocalAnchor0(Ogre::Vector3 &p) { mLocalAnchor0 = p;}
   virtual void setLocalAnchor1(Ogre::Vector3 &p) { mLocalAnchor1 = p;}
   virtual void setLocalAxis0(Ogre::Vector3 &p) { mLocalAxis0 = p;}
   virtual void setLocalAxis1(Ogre::Vector3 &p) { mLocalAxis1 = p;}
   virtual void setLocalNormal0(Ogre::Vector3 &p) { mLocalNormal0 = p;}
   virtual void setLocalNormal1(Ogre::Vector3 &p) { mLocalNormal1 = p;}

   virtual void setGlobalAnchor(Ogre::Vector3 &p) { mGlobalAnchor = p;}
   virtual void setGlobalAxis(Ogre::Vector3 &p) { mGlobalAxis = p;}

	virtual physJointType getJointType() {return mJointType;}
	virtual void setJointType(physJointType type) { mJointType = type;}
	virtual bool getHW() {return mHW;}
	virtual void setHW(bool b=true) {mHW=b;}
	virtual Ogre::Quaternion &getMotorTarget() {return mMotorTarget;}
	virtual void setMotorTarget(Ogre::Quaternion &q) {mMotorTarget=q;}
	virtual Ogre::Quaternion& getLastTarget() {return mLastTarget;}
	virtual void setLastTarget(Ogre::Quaternion &q) {mLastTarget=q;}
	virtual Ogre::Quaternion& getNewTarget() {return mNewTarget;}
	virtual void setNewTarget(Ogre::Quaternion &q) {mNewTarget=q;}

};
#endif 
