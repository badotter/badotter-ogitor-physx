////////////////////////////////////////////////////////////////////////////////////////////////////
//  fxRigidBody
//  Chris Calef
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _FXRIGIDBODY_H_
#define _FXRIGIDBODY_H_

#include "EcstasyMotion/nxPhysManager.h"
#include "EcstasyMotion/physRigidBody.h"

#include "OgreMovableObject.h"
#include "OgreEntity.h"
//#include "gfx/gfxDevice.h"
//
//#ifndef _BOXCONVEX_H_
//#include "collision/boxConvex.h"
//#endif


class fxRigidBody  :  public virtual iPhysUser
{
	//friend class fxRigidBodyFactory;//Not really necessary ATM - not using any
	//protected data anyway - but might as well put it here for later.

public:
	//SimGroup *mGroup;

   physManager *mPM;
   physRigidBody *mRB;
   //physSpring *mSpring;//often need this for correcting position in world.

   fxFlexBody *mParentBody;
   int mParentMountSlot;

	Ogre::SceneNode *mNode;
	Ogre::Entity *mEntity;

	int mRigidBodyDataID;

   Ogre::Vector3  mCurrPosition;//do I need this, instead of using SceneObject's transform?
   Ogre::Vector3  mCurrVelocity;
   Ogre::Vector3  mCurrForce;
   Ogre::Vector3  mCurrTorque;
   Ogre::Vector3  mLastPosition;
   Ogre::Vector3  mScale;//Q: Is this totally redundant?  physRigidBodyCommon also has it...
   float      mDeltaPos;

   Ogre::Vector3 mInitialPosition;
   Ogre::Vector3 mInitialLinearVelocity;
   Ogre::Vector3 mInitialAngularVelocity;
   Ogre::Quaternion mInitialOrientation;
   Ogre::Quaternion mCurrAngularPosition;
   Ogre::Vector3 mDesiredVelocity;

   Ogre::Vector3 mWeaponPosAdj;
   Ogre::Quaternion mWeaponRotAdjA,mWeaponRotAdjB;

   unsigned int mCurrMS;
   unsigned int mEndMS;
   unsigned int mDelayMS;
   unsigned int mCurrTick;
   unsigned int mLifetimeMS;

   int mMeshBody;
   int mStartMesh;//for storing convex mesh pointers
   int mNumMeshes;//(redundant w/ physRigidBodyCommon?)
   float mSleepThreshold;
	float mDamageMultiplier;
   int mReferenceNumber;

   bool mIsClientOnly;
   bool mIsKinematic;
   bool mIsNoGravity;
   bool mHasTrigger;
   bool mHadCollision;//TEMP - delete after torque collisions is fixed
   bool mReset;//flag to tell us to reset position/velocity, on next worldstep.
   bool mAutoClearKinematic;
   bool mIsMoving;
   bool mHasDesiredVelocity;

   std::vector<int> mCollisionDetails;
   std::vector<int> mLOSDetails;

   //OrthoBoxConvex mConvex;
   //Box3F          mWorkingQueryBox;

	std::string mShapeName;
   std::string mDebugTextureName;

   //GFXTexHandle mDebugTextureHandle;
   //GFXVertexBufferHandle<GFXVertexPNTT> mDebugVerts;
   //GFXPrimitiveBufferHandle           mDebugPrimitives;

   //GFXTexHandle mTriggerTextureHandle;
   //Ogre::Vector3 mTriggerPoints[8];
   //GFXVertexBufferHandle<GFXVertexPNTT> mTriggerVerts;
   //GFXPrimitiveBufferHandle           mTriggerPrimitives;

   NxConvexMeshDesc mMeshDescs[MAX_MESHES_PER_RB];

   fxRigidBody();
   fxRigidBody(Ogre::Entity *,const char*);
   //fxRigidBody(Ogre::SceneNode *);
   ~fxRigidBody();

   //virtual bool onAdd();
   //virtual void onRemove();
   //virtual bool onNewDataBlock(GameBaseData* dptr,bool reload);

   static void initScriptFields();

   //virtual void onEditorEnable();
   //virtual void onEditorDisable();
   //virtual void inspectPostApply();

   virtual void updatePositionFromRB();
   virtual void updateVelocityFromRB();
   virtual void updateForcesToRB();

   //virtual void processTick(const Move* pMove);
   //virtual void advanceTime(float);
   //virtual void interpolateTick(float);

   //virtual void setTransform(const Ogre::Matrix4& kTransformMatrix);

   //void renderDebug();
   //bool prepRenderImage(SceneState* state, const unsigned int stateKey, const unsigned int startZone, const bool modifyBaseZoneState);
   void setupDebugRender();

   //virtual unsigned int packUpdate(NetConnection* pConnection, unsigned int uiMask, BitStream* pBitStream);
   //virtual void unpackUpdate(NetConnection* pConnection, BitStream *pBitStream);

   void setupMeshBody();
   void setupMesh();
   void setupCollisionMesh();
   void setupRigidBody();

   void onWorldStep();
   void onCollision(iPhysUser*,int);
   void onTrigger(iPhysUser *,int,int);

   void addForceAtPos(const Ogre::Vector3 &kForce,const Ogre::Vector3 &kPos);
   void lockTractorBeam(int);
   void getPositionFromParent();

   void setKinematic();
   void clearKinematic();
   void resetPosition();

   void setWeaponPosAdj(Ogre::Vector3 &);
   void setWeaponRotAdjA(Ogre::Vector3 &);
   void setWeaponRotAdjB(Ogre::Vector3 &);

   void setGroup(int groupID);

	void addRef();
	void release();
	int doStuff(int arg);

	bool loadSQL();
	void examineEntity();

	Ogre::Vector3 getNodePosition() const;
	void setNodePosition(const Ogre::Vector3 &pos);

};

// Factory object for creating Entity instances //
//class _OgreExport fxRigidBodyFactory : public Ogre::EntityFactory
//{
//protected:
//	MovableObject* createInstanceImpl( const String& name, const NameValuePairList* params);
//public:
//	fxRigidBodyFactory() {}
//	~fxRigidBodyFactory() {}
//
//	static String FACTORY_TYPE_NAME;
//
//	const String& getType(void) const;
//	void destroyInstance( MovableObject* obj);
//
//};

/*
//class TSShapeInstance;
class fxFlexBody;
class fxFlexBodyPart;
//
//
//struct fxRigidBodyData : public EntityData
//{
//	typedef ShapeBaseData Parent;
//
//public:
//   DECLARE_CONOBJECT(fxRigidBodyData);
//
//   float mDynamicFriction;
//   float mStaticFriction;
//   float mRestitution;
//   float mDensity;
//   float mSleepThreshold;
//
//   physShapeType mShapeType;
//
//   Ogre::Vector3 mOffset;
//   Ogre::Vector3 mOrientation;
//   Ogre::Vector3 mDimensions;
//
//   physShapeType mTriggerShapeType;
//   Ogre::Vector3 mTriggerOffset;
//   Ogre::Vector3 mTriggerOrientation;
//   Ogre::Vector3 mTriggerDimensions;
//   Ogre::Vector3 mTriggerActorOffset;
//   Ogre::Vector3 mProjectileAxis;
//
//   Ogre::Vector3 mWeaponPosAdj;
//   Ogre::Vector3 mWeaponRotAdjA,mWeaponRotAdjB;
//
//   bool mIsKinematic;
//   bool mIsNoGravity;
//   bool mHasTrigger;
//   bool mIsTransient;
//   bool mHasSpring;
//   bool mHW;
//
//   //iPhysUser
//   bool mIsInflictor;
//   float mInflictMultiplier;
//
//   unsigned int mLifetimeMS;
//
//   fxRigidBodyData();
//   ~fxRigidBodyData();
//
//   bool preload(bool bServer, String &errorStr);
//   bool  onAdd();
//   static void initPersistFields();
//   void packData(BitStream*);
//   void unpackData(BitStream*);
//};

//DECLARE_CONSOLETYPE(fxRigidBodyData)
*/


#endif 
