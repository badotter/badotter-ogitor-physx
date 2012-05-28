////////////////////////////////////////////////////////////////////////////////////////////////////
//  nxJoint
//  Chris Calef
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _FXJOINT_H_
#define _FXJOINT_H_

#include "EcstasyMotion/nxPhysManager.h"
#include "EcstasyMotion/physJointCommon.h"
//#include "EcstasyMotion/fxFlexBodyPart.h"

class physRigidBody;
class fxRigidBody;
//class fxFlexBodyPart;

//static EnumTable::Enums gJointEnums[] =
//  {
//     {PHYS_JOINT_PRISMATIC, "JOINT_PRISMATIC"},
//    {PHYS_JOINT_REVOLUTE, "JOINT_REVOLUTE"},
//    {PHYS_JOINT_CYLINDRICAL, "JOINT_CYLINDRICAL"},
//    {PHYS_JOINT_SPHERICAL, "JOINT_SPHERICAL"},
//    {PHYS_JOINT_POINT_ON_LINE, "JOINT_POINT_ON_LINE"},
//    {PHYS_JOINT_POINT_IN_PLANE, "JOINT_POINT_IN_PLANE"},
//    {PHYS_JOINT_DISTANCE, "JOINT_DISTANCE"},
//    {PHYS_JOINT_PULLEY, "JOINT_PULLEY"},
//    {PHYS_JOINT_FIXED, "JOINT_FIXED"},
//    {PHYS_JOINT_D6, "JOINT_D6"}
//  };
//static EnumTable gJointTypeTable(10, &gJointEnums[0]);
//DefineEnumType( physJointType );


//struct fxJointData : public GameBaseData
//{
//   typedef GameBaseData Parent;
//
//public:
//   DECLARE_CONOBJECT(fxJointData);
//
//   enum physJointType mJointType;
//
//   float mTwistLimit;
//   float mSwingLimit;
//   float mSwingLimit2;
//
//   float mHighLimit;
//   float mLowLimit;
//   float mHighRestitution;
//   float mLowRestitution;
//
//   float mDistanceLimit;//maybe use this to create limit planes for 
//   //                   cylindrical/prismatic joints
//
//   float mMaxForce;
//   float mMaxTorque;
//
//   float mMotorForce;
//   float mMotorSpeed;
//
//   float mJointSpring;
//   float mSwingSpring;
//   float mTwistSpring;
//   float mSpringDamper;
//   float mSpringTargetAngle;
//
//   // bring in limit planes, etc. later
//
//   Ogre::Vector3 mAxisA, mAxisB;
//   Ogre::Vector3 mNormalA, mNormalB;
//
//   Ogre::Vector3 mLocalAnchor0, mLocalAnchor1;
//   Ogre::Vector3 mLocalAxis0, mLocalAxis1;
//   Ogre::Vector3 mLocalNormal0, mLocalNormal1;
//
//   Ogre::Vector3 mLimitPoint;
//   Ogre::Vector3 mLimitPlaneAnchor1;
//   Ogre::Vector3 mLimitPlaneNormal1;
//   Ogre::Vector3 mLimitPlaneAnchor2;
//   Ogre::Vector3 mLimitPlaneNormal2;
//   Ogre::Vector3 mLimitPlaneAnchor3;
//   Ogre::Vector3 mLimitPlaneNormal3;
//   Ogre::Vector3 mLimitPlaneAnchor4;
//   Ogre::Vector3 mLimitPlaneNormal4;
//
//   bool mCollisionEnabled;
//   bool mHW;
//
//   physJointData mJD;
//
//   fxJointData();
//
//   bool preload(bool bServer, String &errorStr);
//   bool  onAdd();
//   static void initPersistFields();
//   void packData(BitStream*);
//   void unpackData(BitStream*);
//   //bool preload(bool server, char errorBuffer[256]);
//
//};

//DECLARE_CONSOLETYPE(fxJointData)


class fxJoint //: public GameBase
{
//private:
	//typedef GameBase Parent;
	//fxJointData *mDataBlock;

public:
	//DECLARE_CONOBJECT(fxJoint);

	physManager *mPM;
	physJoint *mJoint;

	physRigidBody *mRB_A;
	physRigidBody *mRB_B;

	fxRigidBody *mBodyA;
	int          mBodyA_ID;
	fxRigidBody *mBodyB;
	int          mBodyB_ID;

	//fxFlexBodyPart *mFlexBodyPartA;
	//int             mFlexBodyPartA_ID;
	//fxFlexBodyPart *mFlexBodyPartB;
	//int             mFlexBodyPartB_ID;

	Ogre::Vector3 mAxisA, mAxisB;
	Ogre::Vector3 mNormalA, mNormalB;

	Ogre::Vector3 mLocalAnchor0, mLocalAnchor1;
	Ogre::Vector3 mLocalAxis0, mLocalAxis1;
	Ogre::Vector3 mLocalNormal0, mLocalNormal1;

	Ogre::Vector3 mGlobalAnchor, mGlobalAxis;
	
	fxJoint();
	~fxJoint();

	//virtual bool onAdd();
	//virtual void onRemove();
	//virtual bool onNewDataBlock(GameBaseData* dptr, bool reload);

	//static void initPersistFields();
	//virtual U32 packUpdate(NetConnection* pConnection, U32 uiMask, BitStream* pBitStream);
	//virtual void unpackUpdate(NetConnection* pConnection, BitStream *pBitStream);

	void setPM(physManager *ptr) {mPM=(nxPhysManager *)ptr;}
};
#endif 
