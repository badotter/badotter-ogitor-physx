////////////////////////////////////////////////////////////////////////////////////////////////////
//  physRigidBodyCommon
//  Chris Calef 2006
//
// adapted from LRGRigidBody.h
//	by Yossi Horowitz
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _PHYSRIGIDBODYCOMMON_H_
#define _PHYSRIGIDBODYCOMMON_H_

#include "EcstasyMotion/physRigidBody.h"


//#include "ts/tsMesh.h"


class physRigidBodyCommon : public physRigidBody
{

 public:
    iPhysUser *mPhysUser;
	 physJoint *mJoint;

	 Ogre::Vector3 mLinearPosition;
	 Ogre::Vector3 mCurrLinearPosition;
	 Ogre::Vector3 mLastLinearPosition;
	 Ogre::Quaternion   mAngularPosition;
	 Ogre::Quaternion   mCurrAngularPosition;
	 Ogre::Quaternion   mLastAngularPosition;
	 Ogre::Vector3 mLinearVelocity;
	 Ogre::Vector3 mAngularVelocity;

	 float      mBodyMass;
	 Ogre::Vector3  mMassCenter;

 	 Ogre::Vector3 mOffset;
	 Ogre::Vector3 mOrientation;
	 Ogre::Vector3 mDimensions;
	 Ogre::Vector3 mScale;

	 Ogre::Vector3 mTriggerOffset;
	 Ogre::Vector3 mTriggerOrientation;
	 Ogre::Vector3 mTriggerDimensions;
	 Ogre::Vector3 mProjectileAxis;

	 float mDynamicFriction;
	 float mStaticFriction;
	 float mRestitution;
	 float mDensity;

	 Ogre::Matrix4  mInitialObjToWorld;
	 Ogre::Quaternion mDefault;

	 Ogre::Vector3  mCurrForce;//store extra forces added by game logic
	 Ogre::Vector3  mCurrTorque;
	 Ogre::Vector3  mGlobalForce;
	 Ogre::Vector3  mGlobalDelayForce;
	 Ogre::Vector3  mGlobalTorque;
	 Ogre::Vector3  mMaxTorque;

	 std::vector<Ogre::Vector3> mVerts;
	 std::vector<unsigned int> mIndices;

	 int mBodyVertLookups[MAX_MESHES_PER_RB];
	 int mStartMesh;
	 int mNumMeshes;
	 int mActorGroup;
	 int mNodeIndex;
	 int mDelayStep;
	 int mLastNumVerts;

	 physEntityType mEntityType;
	 physShapeType mShapeType;
	 physShapeType mTriggerShapeType;

	 bool mIsKinematic;
	 bool mIsNoGravity;
	 bool mIsProjectile;
	 bool mIsInflictor;
	 bool mHW;
	 bool mImpulse;

	 bool mSetup;
	 bool mRemove;

	 physRigidBodyCommon(){};
	 virtual ~physRigidBodyCommon(){};


	 virtual iPhysUser *getPhysUser() {return mPhysUser;}
	 virtual void setPhysUser(iPhysUser *iPU) {mPhysUser=iPU;}

	 virtual physJoint *getJoint() {return mJoint;}
	 virtual void setJoint(physJoint *mJ) {mJoint=mJ;}
  
	 virtual int getActorGroup() {return mActorGroup;}
	 virtual void setActorGroup(int ag) {mActorGroup=ag;}

	 virtual int getNumMeshes() {return mNumMeshes;}
	 virtual void setNumMeshes(int n) {mNumMeshes=n;}

	 virtual int getNodeIndex() {return mNodeIndex;}
	 virtual void setNodeIndex(int ni) {mNodeIndex=ni;}
	 
	 virtual Ogre::Quaternion& getDefaultQuat() {return mDefault;}
	 virtual void setDefaultQuat(Ogre::Quaternion& mD) {mDefault=mD;}

	 virtual physShapeType getShapeType() {return mShapeType;}
	 virtual void setShapeType(physShapeType pST) {mShapeType=pST;}

	 virtual physEntityType getEntityType() {return mEntityType;}
	 virtual void setEntityType(physEntityType pET) {mEntityType=pET;}

	 virtual physShapeType getTriggerShapeType() {return mTriggerShapeType;}
	 virtual void setTriggerShapeType(physShapeType pST) {mTriggerShapeType=pST;}

	 virtual int getStartMesh() {return mStartMesh;}
	 virtual void setStartMesh(int m) {mStartMesh=m;}

	 virtual int getBodyVertLookup(int i) {return mBodyVertLookups[i];}
	 virtual void setBodyVertLookup(int i,int m) {mBodyVertLookups[i]=m;}
	 
	 virtual void setDynamicFriction(float f) {mDynamicFriction=f;}
	 virtual void setStaticFriction(float f)  {mStaticFriction=f;}
	 virtual void setRestitution(float f)     {mRestitution=f;}
	 virtual void setDensity(float f)         {mDensity=f;}
	 virtual float getDensity()               {return mDensity;}


	 virtual bool getIsKinematic() {return mIsKinematic;}
	 virtual void setKinematic(bool b=true){mIsKinematic=b;}
	 virtual bool getIsNoGravity() {return mIsNoGravity;}
	 virtual void setNoGravity(bool b=true){mIsNoGravity=b;}
	 virtual bool getIsProjectile() {return mIsProjectile;}
	 virtual void setProjectile(bool b=true){mIsProjectile=b;}
	 virtual bool getIsInflictor() {return mIsInflictor;}
	 virtual void setInflictor(bool b=true){mIsInflictor=b;}
	 virtual bool getHW() {return mHW;}
	 virtual void setHW(bool b=true){mHW=b;}
	 virtual bool getImpulse() {return mImpulse;}
	 virtual void setImpulse(bool b=true){mImpulse=b;}

	 virtual Ogre::Vector3& getLinearPosition() {return mLinearPosition;}
	 virtual void setLinearPosition(const Ogre::Vector3& kLinearPosition)
	 {mLinearPosition = kLinearPosition;}
	 virtual void setCurrLinearPosition(const Ogre::Vector3& kLinearPosition)
	 {mCurrLinearPosition = kLinearPosition;}
	 virtual void setLastLinearPosition(const Ogre::Vector3& kLinearPosition)
	 {mLastLinearPosition = kLinearPosition;}

	 virtual Ogre::Quaternion& getAngularPosition() {return	mAngularPosition;}
	 virtual void setAngularPosition(const Ogre::Quaternion& kAngularPosition)
	 {mAngularPosition = kAngularPosition;}
	 virtual void  setAngularPositionMatrix(Ogre::Matrix3 &kMat)
	 {mAngularPosition.ToRotationMatrix(kMat);}
	 virtual void setCurrAngularPosition(const Ogre::Quaternion& kAngularPosition)
	 {mCurrAngularPosition = kAngularPosition;}
	 virtual void setLastAngularPosition(const Ogre::Quaternion& kAngularPosition)
	 {mLastAngularPosition = kAngularPosition;}

	 virtual Ogre::Vector3& getLinearVelocity() {return mLinearVelocity;}
	 virtual void setLinearVelocity(const Ogre::Vector3& kLinearVelocity)
	 {mLinearVelocity = kLinearVelocity;}
	 virtual Ogre::Vector3& getAngularVelocity() {return mAngularVelocity;}
	 virtual void setAngularVelocity(const Ogre::Vector3& kAngularVelocity)
	 {mAngularVelocity = kAngularVelocity;}

	 virtual void setCurrForce(const Ogre::Vector3& kForce) {mCurrForce = kForce;}
	 virtual void setCurrTorque(const Ogre::Vector3& kTorque) {mCurrTorque = kTorque;}
	 virtual void setGlobalForce(const Ogre::Vector3& kForce) {mGlobalForce = kForce;}
	 virtual void setGlobalDelayForce(const Ogre::Vector3& kForce) {mGlobalDelayForce = kForce;}
	 virtual void setGlobalTorque(const Ogre::Vector3& kTorque) {mGlobalTorque = kTorque;}
	 virtual void setMaxTorque(const Ogre::Vector3& kTorque) {mMaxTorque = kTorque;}
	 virtual void setDelayStep(int s) {mDelayStep = s;}

	 virtual float getMass() {return mBodyMass;}
	 virtual void setMass(float fMass) {mBodyMass = fMass;}//?? Can this work?  Should have to tell physx...

	 virtual Ogre::Vector3& getMassCenter() {return mMassCenter;}
	 virtual void setMassCenter(Ogre::Vector3& kCenter)	{mMassCenter = kCenter;}
	 virtual void setObjToWorld(Ogre::Matrix4 &m) {mInitialObjToWorld=m;}
	 virtual Ogre::Matrix4& getObjToWorld() {return mInitialObjToWorld;}

	 virtual void setOffset(Ogre::Vector3 &p) {mOffset=p;}
	 virtual void setOrientation(Ogre::Vector3 &p) {mOrientation=p;}
	 virtual void setDimensions(Ogre::Vector3 &p) {mDimensions=p;}
	 virtual void setScale(Ogre::Vector3 &p) {mScale=p;}

	 virtual void setTriggerOffset(Ogre::Vector3 &p) {mTriggerOffset=p;}
	 virtual void setTriggerOrientation(Ogre::Vector3 &p) {mTriggerOrientation=p;}
	 virtual void setTriggerDimensions(Ogre::Vector3 &p) {mTriggerDimensions=p;}
	 virtual void setProjectileAxis(Ogre::Vector3 &p) {mProjectileAxis=p;}

	 virtual void setLastNumVerts(int n) {mLastNumVerts=n;}
};
#endif 
