////////////////////////////////////////////////////////////////////////////////////////////////////
//  fxFlexBodyPart.h
//  Chris Calef
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _FXFLEXBODYPART_H_
#define _FXFLEXBODYPART_H_

#include "EcstasyMotion/physManager.h"
#include "EcstasyMotion/fxFlexBody.h"
#include "EcstasyMotion/nxPhysManager.h"

//#include "platform/platform.h"
//#include "T3D/shapeBase.h"
//#include "gfx/gfxDevice.h"
//#include "T3D/player.h"

//#include "T3D/physicsBAG/physRigidBody.h"
//#include "T3D/physicsBAG/physRigidBodyShape.h"
//#include "T3D/physicsBAG/physJoint.h"

//struct fxFlexBodyData;
class fxFlexBody;
class physRigidBody;
//struct fxJointData;
class physJoint;
class NxConvexMeshDesc;

//struct PlayerData;



class fxFlexBodyPart : public virtual iPhysUser
{// public GameBase, 
  //typedef GameBase Parent;

 public:
  //DECLARE_CONOBJECT(fxFlexBodyPart);
  //fxFlexBodyPartData *mDataBlock;

  //enum MaskBits
  //  {
  //    fxFlexBodyPartStateMask = Parent::NextFreeMask,
  //    NextFreeMask	= Parent::NextFreeMask << 1
  //  };

  //fxFlexBody *mFlexBody;//Now this is in iPhysUser
  physManager *mPM;
  physRigidBody *mRB;
  physJoint *mJoint;

  Ogre::Vector3  mCurrPosition;
  Ogre::Vector3  mLastPosition;
  Ogre::Quaternion  mLastOrientation;
  float      mDeltaPos;

  float mForwardForce;

  Ogre::Vector3  mCurrVelocity;
  Ogre::Vector3  mCurrForce;
  Ogre::Vector3  mCurrTorque;
  Ogre::Vector3  mGlobalForce;
  Ogre::Vector3  mGlobalDelayForce;
  Ogre::Vector3  mGlobalTorque;

  Ogre::Vector3 mInitialPosition;
  Ogre::Vector3 mInitialLinearVelocity;
  Ogre::Vector3 mInitialAngularVelocity;

  Ogre::Quaternion mInitialOrientation;
  Ogre::Quaternion mSDKRot;
  Ogre::Quaternion mSDKInverse;

  int mPartID;//Misnomer, this is only a local index into Bodyparts[], not DB ID
  int mNodeIndex;//This is the index into the shape's full array of nodes, or "bones" in Ogre...
  int mBoneIndex;//Confusing terminology!  This is the index into our flexbody mBodyParts list...

  Ogre::Bone *mBone;
  Ogre::String mFlexBodyPartName;
  Ogre::String mNodeName;//Again, confusing, this is really a "Bone" name in Ogre, "Node" name in Torque.

  fxFlexBodyPart *mParentBodyPart;
  int mParentIndex;
  fxFlexBodyPart *mChildBodyPart;
  int mChildIndex;
  int mMesh;

  int mFlexBodyPartDataID;
  int mJointDataID;

  unsigned int mCurrMS;
  unsigned int mEndMS;
  unsigned int mCurrTick; //This is probably redundant w/ fxFlexBody::mCurrTick, should remove.
  unsigned int mLastCollisionTick;
  unsigned int mLastDeactivateTick;

  bool mIsClientOnly;
  bool mIsKinematic;
  bool mIsNoGravity;
  bool mIsInflictor;
  bool mHasCollisionWaiting;
  bool mHasCollision;
  unsigned int mDelayCollisionStep;
  unsigned int mImpulseForceStep;
  unsigned int mAddClientBody;

  //StringTableEntry mDebugTextureName;
  //GFXTexHandle mDebugTextureHandle;
  //GFXVertexBufferHandle<GFXVertexPNTT> mDebugVerts;
  //GFXPrimitiveBufferHandle           mDebugPrimitives;
  NxConvexMeshDesc mMeshDesc;



  //// From the datablock /// (Most of these are getting removed, use rigid body versions instead.)
  //physShapeType mShapeType;
  physChainType mBodypartChain;
  //Ogre::Vector3 mOffset;
  //Ogre::Vector3 mOrientation;
  //Ogre::Vector3 mDimensions;
  //physShapeType mTriggerShapeType;
  //Ogre::Vector3 mTriggerOffset;
  //Ogre::Vector3 mTriggerOrientation;
  //Ogre::Vector3 mTriggerDimensions;
  //Ogre::Vector3 mTriggerActorOffset;
  float mDamageMultiplier;

  fxFlexBodyPart();
  fxFlexBodyPart(int);
  ~fxFlexBodyPart();

  //virtual bool onAdd();
  //virtual void onRemove();
  //virtual bool onNewDataBlock(GameBaseData* dptr, bool reload);
  //virtual void processTick(const Move* pMove);

  virtual void updatePositionFromRB();
  virtual void updateVelocityFromRB();

  virtual void updatePositionToRB();
  virtual void updateVelocityToRB();

  virtual void updateForcesToRB();

  void setForce(Ogre::Vector3 &kForce) { mCurrForce = kForce;  mImpulseForceStep = ((physManagerCommon *)physManagerCommon::getPM())->mCurrStep + 4;}
  void setTorque(Ogre::Vector3 &kTorque) { mCurrTorque = kTorque; }
  void setGlobalForce(Ogre::Vector3 &kForce) { mGlobalForce = kForce; mImpulseForceStep = ((physManagerCommon *)physManagerCommon::getPM())->mCurrStep + 4; }
  void setGlobalTorque(Ogre::Vector3 &kTorque) { mGlobalTorque = kTorque; }
  void setSDKRot(Ogre::Quaternion &kRot) { mSDKRot = kRot; }//mSDKInverse = kRot;  mSDKInverse.inverse();
  void setSDKInverse(Ogre::Quaternion &kInv) { mSDKInverse = kInv; }
  Ogre::Quaternion& getSDKRot() { return mSDKRot; }

  void setupRigidBody();
  void setupJoint();
  void reconfigureJoint();

  void onWorldStep();
  void onCollision(iPhysUser*,int);	
  void onTrigger(iPhysUser *,int,int);
  void setupDebugRender();
  void renderDebug();

  void addForceAtPos(const Ogre::Vector3 &kForce,const Ogre::Vector3 &kPos);
  void lockTractorBeam(int);
  void reset();
  bool nodeOrChildHasCollision();//See if this node or an immediate child has collision. Recursive.

};


//
//struct fxFlexBodyPartData : public GameBaseData
//{
//  typedef GameBaseData Parent;
//
//  public:
//  DECLARE_CONOBJECT(fxFlexBodyPartData);
//
//  StringTableEntry mBaseNodeName;
//  StringTableEntry mParentNodeName;
//  StringTableEntry mChildNodeName;
//  StringTableEntry mMeshObject;
//
//  fxFlexBodyData       *mFlexBodyData;
//  int                     FlexBodyDataID;
//  SimObject              *mPlayerData;
//  int                     mPlayerDataID;
//  fxJointData             *mJointData;
//  int                     JointDataID;
//
//  float mDynamicFriction;
//  float mStaticFriction;
//  float mRestitution;
//  float mDensity;
//  float mDamageMultiplier;
//  float mForceMultiplier;
//  float mForwardForce;
//  float mRagdollThreshold;//angleBetween limit that triggers ragdolling up the chain.
//    
//  physShapeType mShapeType;
//  physChainType mBodypartChain;
//
//  Ogre::Vector3 mOffset;
//  Ogre::Vector3 mOrientation;
//  Ogre::Vector3 mDimensions;
//
//  physShapeType mTriggerShapeType;
//  Ogre::Vector3 mTriggerOffset;
//  Ogre::Vector3 mTriggerOrientation;
//  Ogre::Vector3 mTriggerDimensions;
//  Ogre::Vector3 mTriggerActorOffset;
//
//  Ogre::Vector3 mBoundsMin;
//  Ogre::Vector3 mBoundsMax;
//
//  Ogre::Vector3 mForceMin;
//  Ogre::Vector3 mForceMax;
//
//  Ogre::Vector3 mTorqueMin;
//  Ogre::Vector3 mTorqueMax;
//
//  Ogre::Vector3 mCenterOfMass;
//
//  bool mIsKinematic;
//  int mNumParentVerts;
//  int mNumChildVerts;
//  int mNumFarVerts;
//  float mWeightThreshold;
//  
//  //iPhysUser
//  bool mIsInflictor;
//  float mInflictMultiplier;
//
//  fxFlexBodyPartData();
//  ~fxFlexBodyPartData();
//  
//  bool preload(bool bServer, String &errorStr);
//  bool onAdd();
//  static void initPersistFields();
//  virtual void packData( BitStream* pStream);
//  virtual void unpackData( BitStream* pStream);
//  
//};
//
////DECLARE_CONSOLETYPE(fxFlexBodyPartData)

#endif