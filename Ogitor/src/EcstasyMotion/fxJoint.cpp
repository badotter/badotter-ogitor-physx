////////////////////////////////////////////////////////////////////////////////////////////////////
//  fxJoint.cc
//  Chris Calef
//
// 
//  NX_JOINT_PRISMATIC        // Permits a single translational degree of freedom.
//  NX_JOINT_REVOLUTE         // Also known as a hinge joint, permits one rotational degree of freedom.
//  NX_JOINT_CYLINDRICAL      // Formerly known as a sliding joint, permits one translational and one rotational degree of freedom.
//  NX_JOINT_SPHERICAL        // Also known as a ball or ball and socket joint.
//  NX_JOINT_POINT_ON_LINE    // A point on one actor is constrained to stay on a line on another.
//  NX_JOINT_POINT_IN_PLANE   // A point on one actor is constrained to stay on a plane on another.
//  NX_JOINT_DISTANCE         // A point on one actor maintains a certain distance range to another point on another actor.
//  NX_JOINT_PULLEY           // A pulley joint.
//  NX_JOINT_FIXED            // A "fixed" connection.
//  NX_JOINT_D6               // A 6 degree of freedom joint
////////////////////////////////////////////////////////////////////////////////////////////////////
//#include "core/stl_fix.h"

//#include "platform/platform.h"
//#include "math/mathio.h"
//#include "console/consoleTypes.h"
//#include "collision/ConcretePolyList.h"
//#include "core/stream/bitStream.h"
//#include "ts/tsShapeInstance.h"
//#include "EcstasyMotion/SQLiteObject.h"

#include "EcstasyMotion/fxJoint.h"
#include "EcstasyMotion/fxRigidBody.h"

///////////////////////////////////////////////////////////////////////

//IMPLEMENT_CONOBJECT(fxJoint);

fxJoint::fxJoint()
{
  //mDataBlock = NULL;
  mJoint = NULL;

  mRB_A = NULL;
  mRB_B = NULL;

  mBodyA = NULL;
  mBodyA_ID = 0;
  mBodyB = NULL;
  mBodyB_ID = 0;

  //mFlexBodyPartA = NULL;
  //mFlexBodyPartA_ID = 0;
  //mFlexBodyPartB = NULL;
  //mFlexBodyPartB_ID = 0;

  mAxisA = Ogre::Vector3::ZERO;
  mAxisB = Ogre::Vector3::ZERO;
  mNormalA = Ogre::Vector3::ZERO;
  mNormalB = Ogre::Vector3::ZERO;

  mLocalAnchor0 = Ogre::Vector3::ZERO;
  mLocalAnchor1 = Ogre::Vector3::ZERO;
  mLocalAxis0 = Ogre::Vector3::ZERO;
  mLocalAxis1 = Ogre::Vector3::ZERO;
  mLocalNormal0 = Ogre::Vector3::ZERO;
  mLocalNormal1 = Ogre::Vector3::ZERO;

  mGlobalAnchor = Ogre::Vector3::ZERO;
  mGlobalAxis = Ogre::Vector3::ZERO;
}

fxJoint::~fxJoint()
{
 
}

//
//bool fxJoint::onAdd()
//{
//	if(!Parent::onAdd() || !mDataBlock){
//		Con::printf("fxJoint -- No datablock!");
//		return false;
//	}
//	if (!mRB_A) {
//		if (mBodyA_ID) {
//			Con::printf("bodyA ID: %d",mBodyA_ID);
//			if( Sim::findObject( mBodyA_ID, mBodyA ) == false) 
//			{
//				mBodyA = NULL;
//				Con::printf("Joint is connected to world for body A.");
//			} else mRB_A = mBodyA->mRB;
//		} else if (mFlexBodyPartA_ID) {
//			if( Sim::findObject( mFlexBodyPartA_ID, mFlexBodyPartA ) == false) 
//			{
//				mFlexBodyPartA = NULL;
//				Con::printf("Flexbodypart A didn't work.");
//			} else mRB_A = mFlexBodyPartA->mRB;
//		}
//	}
//
//	if (!mRB_B) {
//		if (mBodyB_ID) {
//			Con::printf("bodyB ID: %d",mBodyB_ID);
//			if( Sim::findObject( mBodyB_ID, mBodyB ) == false) 
//			{
//				mBodyB = NULL;
//				Con::printf("Joint is connected to world for body B.");
//			} else mRB_B = mBodyB->mRB;
//		} else if (mFlexBodyPartB_ID) {
//			if( Sim::findObject( mFlexBodyPartB_ID, mFlexBodyPartB ) == false) 
//			{
//				mFlexBodyPartB = NULL;
//				Con::printf("Flexbodypart B didn't work.");
//			} else mRB_B = mFlexBodyPartB->mRB;
//		}
//	}
//
//	if (!mRB_A && !mRB_B) 
//	{
//		Con::printf("No valid bodies defined for joint!");
//		return false;
//	}
//
//	mPM = physManagerCommon::getPM();
//	//HERE: set up mJoint with everything it needs. (First create it: new nxJoint?)
//	mJoint = mPM->createJoint();
//
//	mJoint->setJD(&(mDataBlock->mJD));
//	mJoint->setRB_A(mRB_A);
//	mJoint->setRB_B(mRB_B);
//
//	//q16 = mFlexBody->getShapeInstance()->getShape()->defaultRotations[mNodeIndex];
//	//q16.getOgre::Quaternion(&mDef);
//	//Con::printf("defRotations %d: %3.2f %3.2f %3.2f %3.2f",mNodeIndex,mDef.x,mDef.y,mDef.z,mDef.w);
//
//	MatrixF mA,mB,mDef; //mDef = "default rotation", or local rotation relative to parent. 
//	Ogre::Quaternion qA,qB; 
//	Ogre::Vector3 kAxis,kNormal;
//
//	qA = mRB_A->getAngularPosition();  qA.setMatrix(&mA);
//	qB = mRB_B->getAngularPosition();  qB.setMatrix(&mB);
//	mDef.mul(mB,mA.inverse()); //Multiply child matrix by inverse of parent matrix, to get local matrix.
//
//	//Here: if any axis/normal
//	if ((mDataBlock->mLocalAnchor0.length()>0)&&(mLocalAnchor0.length()<=0))
//		mLocalAnchor0 = mDataBlock->mLocalAnchor0;
//	if ((mDataBlock->mLocalAnchor1.length()>0)&&(mLocalAnchor1.length()<=0))
//		mLocalAnchor1 = mDataBlock->mLocalAnchor1;
//	if ((mDataBlock->mLocalAxis0.length()>0)&&(mLocalAxis0.length()<=0))
//		mLocalAxis0 = mDataBlock->mLocalAxis0;
//	if ((mDataBlock->mLocalAxis1.length()>0)&&(mLocalAxis1.length()<=0))
//		mLocalAxis1 = mDataBlock->mLocalAxis1;
//	if ((mDataBlock->mLocalNormal0.length()>0)&&(mLocalNormal0.length()<=0))
//		mLocalNormal0 = mDataBlock->mLocalNormal0;
//	if ((mDataBlock->mLocalNormal1.length()>0)&&(mLocalNormal1.length()<=0))
//		mLocalNormal1 = mDataBlock->mLocalNormal1;
//
//	//And, have to keep these in case anybody in the first batch of customers is using them.
//	if ((mDataBlock->mAxisA.length()>0)&&(mAxisA.length()<=0))
//		mAxisA = mDataBlock->mAxisA;
//	if ((mDataBlock->mAxisB.length()>0)&&(mAxisB.length()<=0))
//		mAxisB = mDataBlock->mAxisB;
//	if ((mDataBlock->mNormalA.length()>0)&&(mNormalA.length()<=0))
//		mNormalA = mDataBlock->mNormalA;
//	if ((mDataBlock->mNormalB.length()>0)&&(mNormalB.length()<=0))
//		mNormalB = mDataBlock->mNormalB;
//
//	//NOW: don't do any of the testing and special case scenarios here, doing it all in nxJoint::setup, 
//	//since that's where all joints end up in the same place (fxJoints and fxFlexBodyPart joints).
//	//For now, just pass everything on to the physJoint.
//	mJoint->setGlobalAnchor(mGlobalAnchor);	
//	mJoint->setLocalAnchor0(mLocalAnchor0);
//	mJoint->setLocalAnchor1(mLocalAnchor1);
//	mJoint->setGlobalAxis(mGlobalAxis);	
//	mJoint->setLocalAxis0(mLocalAxis0);
//	mJoint->setLocalAxis1(mLocalAxis1);
//	mJoint->setLocalNormal0(mLocalNormal0);
//	mJoint->setLocalNormal1(mLocalNormal1);
//	mJoint->setAxisA(mAxisA);
//	mJoint->setAxisB(mAxisB);
//	mJoint->setNormalA(mNormalA);
//	mJoint->setNormalB(mNormalB);
//
//
//	//IMPORTANT:  This may not be the best place to do this, but it might be.  What we're doing is 
//	//looking at only the child data for axis and normal, so the end user only has to define it once,
//	//and then finding the parent axis and normal by rotating the child versions through the default
//	//rotation of the child bodypart.  (Default rotation: the local difference in orientation between
//	//a child bodypart and its parent.)
//	//if ((mGlobalAxis.length()>0)&&(mLocalNormal1.length()>0))
//	//{
//	//	mJoint->setGlobalAxis(mGlobalAxis);	
//	//	mLocalNormal1.normalize();
//	//	mDef.mulP(mLocalNormal1,&kNormal);
//	//	mJoint->setLocalNormal0(kNormal);
//	//	mJoint->setLocalNormal1(mLocalNormal1);
//	//} else if ((mLocalAxis1.length()>0)&&(mLocalNormal1.length()>0))	{
//	//	mLocalAxis1.normalize();
//	//	mDef.mulP(mLocalAxis1,&kAxis);
//	//	Con::errorf("local axis 1: %3.2f %3.2f %3.2f, parent axis %3.2f %3.2f %3.2f",mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z,kAxis.x,kAxis.y,kAxis.z);
//	//	mLocalNormal1.normalize();
//	//	mDef.mulP(mLocalNormal1,&kNormal);
//	//	mJoint->setLocalAxis0(kAxis);
//	//	mJoint->setLocalAxis1(mLocalAxis1);
//	//	mJoint->setLocalNormal0(kNormal);
//	//	mJoint->setLocalNormal1(mLocalNormal1);
//	//} else if ((mAxisB.length()>0)&&(mNormalB.length()>0)) {//deprecated 
//	//	mAxisB.normalize();
//	//	mDef.mulP(mAxisB,&kAxis);
//	//	mDataBlock->mNormalB.normalize();
//	//	mDef.mulP(mNormalB,&kNormal);
//	//	mJoint->setAxisA(kAxis);//deprecated
//	//	mJoint->setAxisB(mAxisB);//deprecated
//	//	mJoint->setNormalA(kNormal);//deprecated
//	//	mJoint->setNormalB(mNormalB);//deprecated
//	//}
//	//////////////////////////////////////////////////////
//	//if (mGlobalAnchor.length()>0.0)//danger! won't work for joints based at the origin
//	//{
//	//	Con::errorf("fxJoint setting global anchor: %3.2f %3.2f %3.2f",mGlobalAnchor.x,mGlobalAnchor.y,mGlobalAnchor.z);
//	//	mJoint->setGlobalAnchor(mGlobalAnchor);
//	//} 
//	//else if ((mLocalAnchor0.length()>=0)&&(mLocalAnchor1.length()>=0))
//	//{//have to say >= 0, because many times local anchors will be origin.
//	//	Con::errorf("fxJoint setting local anchors: %3.2f %3.2f %3.2f, %3.2f %3.2f %3.2f",
//	//		mLocalAnchor0.x,mLocalAnchor0.y,mLocalAnchor0.z,
//	//		mLocalAnchor1.x,mLocalAnchor1.y,mLocalAnchor1.z);
//	//	mJoint->setLocalAnchor0(mLocalAnchor0);
//	//	mJoint->setLocalAnchor1(mLocalAnchor1);
//	//}
//
//	mJoint->setHW(mDataBlock->mHW);
//	mPM->addJointSetup(mJoint);
//
//	return true;
//}

//void fxJoint::onRemove()
//{
//  if (mJoint) mPM->removeJoint(mJoint);
//
//  scriptOnRemove();
//
//  Parent::onRemove();
//}

//bool fxJoint::onNewDataBlock(GameBaseData *pGameBaseData, bool reload)
//{
//  mDataBlock = dynamic_cast<fxJointData*>(pGameBaseData);
//  if (!mDataBlock || !Parent::onNewDataBlock(pGameBaseData,reload))
//    {
//      return false;
//    }
//
//  // Have parent class do the rest
//  scriptOnNewDataBlock();
//  return true;
//}

//void fxJoint::initPersistFields()
//{
//  Parent::initPersistFields();
//
//  addField("BodyA", TypeS32,
//		Offset(mBodyA_ID, fxJoint));
//  addField("BodyB", TypeS32,
//		Offset(mBodyB_ID, fxJoint));
//
//  //Are these usable, or the result of temporary confusion?
//  addField("FlexBodyPartA", TypeS32,
//		Offset(mFlexBodyPartA_ID, fxJoint));
//  addField("FlexBodyPartB", TypeS32,
//		Offset(mFlexBodyPartB_ID, fxJoint));
//
//  addField("AxisA", TypeOgre::Vector3,
//		Offset(mAxisA, fxJoint));
//  addField("AxisB", TypeOgre::Vector3,
//		Offset(mAxisB, fxJoint));
//  addField("NormalA", TypeOgre::Vector3,
//		Offset(mNormalA, fxJoint));
//  addField("NormalB", TypeOgre::Vector3,
//		Offset(mNormalB, fxJoint));
//
//  addField("LocalAnchor0", TypeOgre::Vector3,
//		Offset(mLocalAnchor0, fxJoint));
//  addField("LocalAnchor1", TypeOgre::Vector3,
//		Offset(mLocalAnchor1, fxJoint));
//  addField("LocalAxis0", TypeOgre::Vector3,
//		Offset(mLocalAxis0, fxJoint));
//  addField("LocalAxis1", TypeOgre::Vector3,
//		Offset(mLocalAxis1, fxJoint));
//  addField("LocalNormal0", TypeOgre::Vector3,
//		Offset(mLocalNormal0, fxJoint));
//  addField("LocalNormal1", TypeOgre::Vector3,
//		Offset(mLocalNormal1, fxJoint));
//
//  addField("GlobalAnchor", TypeOgre::Vector3,
//		Offset(mGlobalAnchor, fxJoint));
//  addField("GlobalAxis", TypeOgre::Vector3,
//		Offset(mGlobalAxis, fxJoint));
//
//}


//U32 fxJoint::packUpdate(NetConnection* pConnection, U32 uiMask, BitStream* pBitStream)
//{
//  //U32 uiRetMask = Parent::packUpdate(pConnection, uiMask, pBitStream);
//  //should not be here at all yet
//  Con::errorf("we're packing an update for fxJoint!!");
//  return 0;
//}

//void fxJoint::unpackUpdate(NetConnection* pConnection, BitStream *pBitStream)
//{
//  //Parent::unpackUpdate(pConnection, pBitStream);
//  //should not be here at all yet
//  Con::errorf("we're unpacking an update for fxJoint!!");
//}






//IMPLEMENT_CO_DATABLOCK_V1(fxJointData);

//IMPLEMENT_CONSOLETYPE(fxJointData)
//IMPLEMENT_SETDATATYPE(fxJointData)
//IMPLEMENT_GETDATATYPE(fxJointData)

//ImplementEnumType( physJointType,
//   "Physics joint types\n" )
//   //"@ingroup JointEnums\n\n")
//	{ PHYS_JOINT_PRISMATIC,     "JOINT_PRISMATIC", "Prismatic Joint"  },
//   { PHYS_JOINT_REVOLUTE,    "JOINT_REVOLUTE", "Revolute Joint" },
//	{ PHYS_JOINT_CYLINDRICAL,      "JOINT_CYLINDRICAL",  "Cylindrical Joint" },
//	{ PHYS_JOINT_SPHERICAL,      "JOINT_SPHERICAL",  "Spherical Joint" },
//	{ PHYS_JOINT_POINT_ON_LINE,      "JOINT_POINT_ON_LINE",  "Point on Line Joint" },
//	{ PHYS_JOINT_POINT_IN_PLANE,      "JOINT_POINT_IN_PLANE",  "Point in Plane Joint" },
//	{ PHYS_JOINT_DISTANCE,      "JOINT_DISTANCE",  "Distance Joint" },
//	{ PHYS_JOINT_PULLEY,      "JOINT_PULLEY",  "Pulley Joint" },
//	{ PHYS_JOINT_FIXED,      "JOINT_FIXED",  "Fixed Joint" },
//	{ PHYS_JOINT_D6,      "JOINT_D6",  "D6 Joint" },
//EndImplementEnumType;
//
//fxJointData::fxJointData()
//{
//  mJointType       = PHYS_JOINT_SPHERICAL;
//  mTwistLimit        = 0.0f;
//  mSwingLimit        = 0.0f;
//  mSwingLimit2       = 0.0f;
//  mHighLimit         = 0.0f;
//  mLowLimit          = 0.0f;
//  mHighRestitution   = 0.0f;
//  mLowRestitution    = 0.0f;
//  mDistanceLimit     = 0.0f;
//  mMaxForce     = 0.0f;
//  mMaxTorque    = 0.0f;
//  mMotorForce        = 0.0f;
//  mMotorSpeed        = 0.0f; 
//  mJointSpring       = 0.0f; 
//  mSwingSpring       = 0.0f; 
//  mTwistSpring       = 0.0f;
//  mSpringDamper      = 0.0f;
//  mSpringTargetAngle = 0.0f;
//  
//  mAxisA = Ogre::Vector3::ZERO;
//  mAxisB = Ogre::Vector3::ZERO;
//  mNormalA = Ogre::Vector3::ZERO;
//  mNormalB = Ogre::Vector3::ZERO;
//
//  mLocalAnchor0 = Ogre::Vector3::ZERO;
//  mLocalAnchor1 = Ogre::Vector3::ZERO;
//  mLocalAxis0 = Ogre::Vector3::ZERO;
//  mLocalAxis1 = Ogre::Vector3::ZERO;
//  mLocalNormal0 = Ogre::Vector3::ZERO;
//  mLocalNormal1 = Ogre::Vector3::ZERO;
//
//  mLimitPoint = Ogre::Vector3::ZERO;
//  mLimitPlaneAnchor1 = Ogre::Vector3::ZERO;
//  mLimitPlaneNormal1 = Ogre::Vector3::ZERO;
//  mLimitPlaneAnchor2 = Ogre::Vector3::ZERO;
//  mLimitPlaneNormal2 = Ogre::Vector3::ZERO;
//  mLimitPlaneAnchor3 = Ogre::Vector3::ZERO;
//  mLimitPlaneNormal3 = Ogre::Vector3::ZERO;
//  mLimitPlaneAnchor4 = Ogre::Vector3::ZERO;
//  mLimitPlaneNormal4 = Ogre::Vector3::ZERO;
//
//  mCollisionEnabled = false;
//  mHW = false;
//}
//
//
//bool fxJointData::preload(bool bServer, String &errorStr)
//{
//  if (!Parent::preload(bServer, errorStr))
//    {
//      return false;
//    }
//
//  return true;
//}
//
//bool fxJointData::onAdd()
//{
//   if(!Parent::onAdd())
//      return false;
//
//   //This is all a little weird, but the reason 
//   //for it is to encapsulate the physics code
//   //as far away from specific Torque details
//   //as possible.  physicsJointData mJD is not
//   //derived from GameBase, it's just a struct.
//   mJD.mJointType = mJointType;
//
//   mJD.mTwistLimit = mTwistLimit;
//   mJD.mSwingLimit = mSwingLimit;
//   mJD.mSwingLimit2 = mSwingLimit2;
//
//   mJD.mHighLimit = mHighLimit;
//   mJD.mLowLimit = mLowLimit;
//   mJD.mHighRestitution = mHighRestitution;
//   mJD.mLowRestitution = mLowRestitution;
//
//   mJD.mDistanceLimit = mDistanceLimit;
//   mJD.mMaxForce = mMaxForce;
//   mJD.mMaxTorque = mMaxTorque;
//
//   mJD.mMotorForce = mMotorForce;
//   mJD.mMotorSpeed = mMotorSpeed;
//
//   mJD.mJointSpring = mJointSpring;
//   mJD.mSwingSpring = mSwingSpring;
//   mJD.mTwistSpring = mTwistSpring;
//   mJD.mSpringDamper = mSpringDamper;
//   mJD.mSpringTargetAngle = mSpringTargetAngle;
//
//   //Normalize all axes/normals
//   if (mAxisA.length()>0) mAxisA.normalize(); 
//   if (mAxisB.length()>0) mAxisB.normalize();
//   if (mNormalA.length()>0) mNormalA.normalize(); 
//   if (mNormalB.length()>0) mNormalB.normalize();
//
//   if (mLocalAnchor0.length()>0) mLocalAnchor0.normalize(); 
//   if (mLocalAnchor1.length()>0) mLocalAnchor1.normalize();
//   if (mLocalAxis0.length()>0) mLocalAxis0.normalize(); 
//   if (mLocalAxis1.length()>0) mLocalAxis1.normalize();
//   if (mLocalNormal0.length()>0) mLocalNormal0.normalize(); 
//   if (mLocalNormal1.length()>0) mLocalNormal1.normalize();
//
//   mJD.mAxisA = mAxisA;
//   mJD.mAxisB = mAxisB;
//   mJD.mNormalA = mNormalA;
//   mJD.mNormalB = mNormalB;
//
//   mJD.mLocalAnchor0 = mLocalAnchor0;
//   mJD.mLocalAnchor1 = mLocalAnchor1;
//   mJD.mLocalAxis0 = mLocalAxis0;
//   mJD.mLocalAxis1 = mLocalAxis1;
//   mJD.mLocalNormal0 = mLocalNormal0;
//   mJD.mLocalNormal1 = mLocalNormal1;
//
//   mJD.mLimitPoint = mLimitPoint;
//   mJD.mLimitPlaneAnchor1 = mLimitPlaneAnchor1;
//   mJD.mLimitPlaneNormal1 = mLimitPlaneNormal1;
//   mJD.mLimitPlaneAnchor2 = mLimitPlaneAnchor2;
//   mJD.mLimitPlaneNormal2 = mLimitPlaneNormal2;
//   mJD.mLimitPlaneAnchor3 = mLimitPlaneAnchor3;
//   mJD.mLimitPlaneNormal3 = mLimitPlaneNormal3;
//   mJD.mLimitPlaneAnchor4 = mLimitPlaneAnchor4;
//   mJD.mLimitPlaneNormal4 = mLimitPlaneNormal4;
//
//   mJD.mCollisionEnabled = mCollisionEnabled;
//   mJD.mHW = mHW;
//
//   //SQLiteObject *sql = dynamic_cast<nxPhysManager*>(physManagerCommon::getPM())->getSQL();
//   SQLiteObject *sql = new SQLiteObject();//WHoops, apparently we haven't created physics scene yet, so getSQL is no good.
//   if (!sql) return true;
//   if (sql->OpenDatabase("EcstasyMotion.db"))
//   {
//	   char id_query[512],insert_query[512];
//	   int body_id,joint_id,result;
//	   sqlite_resultset *resultSet;
//
//	   //sprintf(id_query,"SELECT id FROM fxJointData WHERE name = '%s';",getName());
//	   //result = sql->ExecuteSQL(id_query);
//	   //resultSet = sql->GetResultSet(result);
//	   //if (resultSet->iNumRows == 1)
//	   //{
//		  // delete sql;
//		  // return true;//We already exist
//	   //}
//
//	   //sprintf(insert_query,"INSERT INTO fxJointData (name) VALUES ('%s');",\
//		  // getName());
//	   //result = sql->ExecuteSQL(insert_query);
//	   sql->CloseDatabase();
//   } 
//   delete sql;
//   return true;
//}
//
//void fxJointData::initPersistFields()
//{
//  Parent::initPersistFields();
//
//  addField("JointType", TYPEID< physJointType >(),
//		Offset(mJointType, fxJointData) );
//  addField("TwistLimit", TypeF32,
//		Offset(mTwistLimit, fxJointData));
//  addField("SwingLimit", TypeF32,
//		Offset(mSwingLimit, fxJointData));
//  addField("SwingLimit2", TypeF32,
//		Offset(mSwingLimit2, fxJointData));
//  addField("HighLimit", TypeF32,
//		Offset(mHighLimit, fxJointData));
//  addField("LowLimit", TypeF32,
//		Offset(mLowLimit, fxJointData));
//  addField("HighRestitution", TypeF32,
//		Offset(mHighRestitution, fxJointData));
//  addField("LowRestitution", TypeF32,
//		Offset(mLowRestitution, fxJointData));
//  addField("DistanceLimit", TypeF32,
//		Offset(mDistanceLimit, fxJointData));
//  addField("BreakingForce", TypeF32,
//		Offset(mMaxForce, fxJointData));
//  addField("BreakingTorque", TypeF32,
//		Offset(mMaxTorque, fxJointData));
//  addField("MotorForce", TypeF32,
//		Offset(mMotorForce, fxJointData));
//  addField("MotorSpeed", TypeF32,
//		Offset(mMotorSpeed, fxJointData));
//  addField("JointSpring", TypeF32,
//		Offset(mJointSpring, fxJointData));
//  addField("SwingSpring", TypeF32,
//		Offset(mSwingSpring, fxJointData));
//  addField("TwistSpring", TypeF32,
//		Offset(mTwistSpring, fxJointData));
//  addField("SpringDamper", TypeF32,
//		Offset(mSpringDamper, fxJointData));
//  addField("SpringTargetAngle", TypeF32,
//		Offset(mSpringTargetAngle, fxJointData));
//
//  addField("AxisA", TypeOgre::Vector3,
//		Offset(mAxisA, fxJointData));
//  addField("AxisB", TypeOgre::Vector3,
//		Offset(mAxisB, fxJointData));
//  addField("NormalA", TypeOgre::Vector3,
//		Offset(mNormalA, fxJointData));
//  addField("NormalB", TypeOgre::Vector3,
//		Offset(mNormalB, fxJointData));
//
//  addField("LocalAnchor0", TypeOgre::Vector3,
//		Offset(mLocalAxis0, fxJointData));
//  addField("LocalAnchor1", TypeOgre::Vector3,
//		Offset(mLocalAxis1, fxJointData));
//  addField("LocalAxis0", TypeOgre::Vector3,
//		Offset(mLocalAxis0, fxJointData));
//  addField("LocalAxis1", TypeOgre::Vector3,
//		Offset(mLocalAxis1, fxJointData));
//  addField("LocalNormal0", TypeOgre::Vector3,
//		Offset(mLocalNormal0, fxJointData));
//  addField("LocalNormal1", TypeOgre::Vector3,
//		Offset(mLocalNormal1, fxJointData));
//
//  addField("LimitPoint", TypeOgre::Vector3,
//		Offset(mLimitPoint, fxJointData));
//  addField("LimitPlaneAnchor1", TypeOgre::Vector3,
//		Offset(mLimitPlaneAnchor1, fxJointData));
//  addField("LimitPlaneNormal1", TypeOgre::Vector3,
//		Offset(mLimitPlaneNormal1, fxJointData));
//  addField("LimitPlaneAnchor2", TypeOgre::Vector3,
//		Offset(mLimitPlaneAnchor2, fxJointData));
//  addField("LimitPlaneNormal2", TypeOgre::Vector3,
//		Offset(mLimitPlaneNormal2, fxJointData));
//  addField("LimitPlaneAnchor3", TypeOgre::Vector3,
//		Offset(mLimitPlaneAnchor3, fxJointData));
//  addField("LimitPlaneNormal3", TypeOgre::Vector3,
//		Offset(mLimitPlaneNormal3, fxJointData));
//  addField("LimitPlaneAnchor4", TypeOgre::Vector3,
//		Offset(mLimitPlaneAnchor4, fxJointData));
//  addField("LimitPlaneNormal4", TypeOgre::Vector3,
//		Offset(mLimitPlaneNormal4, fxJointData));
//  addField("CollisionEnabled", TypeBool,
//		Offset(mCollisionEnabled, fxJointData));
//  addField("HW", TypeBool,
//		Offset(mHW, fxJointData));
//  //whew
//}
//
//void fxJointData::packData(BitStream* pBitStream)
//{
//   Parent::packData(pBitStream);
//
//   pBitStream->writeRangedU32(mJointType, 0, 9);
//   pBitStream->write(mTwistLimit);
//   pBitStream->write(mSwingLimit);
//   pBitStream->write(mSwingLimit2);
//   pBitStream->write(mHighLimit);
//   pBitStream->write(mLowLimit);
//   pBitStream->write(mHighRestitution);
//   pBitStream->write(mLowRestitution);
//   pBitStream->write(mDistanceLimit);
//   pBitStream->write(mMaxForce);
//   pBitStream->write(mMaxTorque);
//   pBitStream->write(mMotorForce);
//   pBitStream->write(mMotorSpeed);
//   pBitStream->write(mJointSpring);
//   pBitStream->write(mSwingSpring);
//   pBitStream->write(mTwistSpring);
//   pBitStream->write(mSpringDamper);
//   pBitStream->write(mSpringTargetAngle);
//   mathWrite(*pBitStream, mAxisA);
//   mathWrite(*pBitStream, mAxisB);
//   mathWrite(*pBitStream, mNormalA);
//   mathWrite(*pBitStream, mNormalB);
//
//   mathWrite(*pBitStream, mLocalAnchor0);
//   mathWrite(*pBitStream, mLocalAnchor1);
//   mathWrite(*pBitStream, mLocalAxis0);
//   mathWrite(*pBitStream, mLocalAxis1);
//   mathWrite(*pBitStream, mLocalNormal0);
//   mathWrite(*pBitStream, mLocalNormal1);
//
//   mathWrite(*pBitStream, mLimitPoint);
//   mathWrite(*pBitStream, mLimitPlaneAnchor1);
//   mathWrite(*pBitStream, mLimitPlaneNormal1);
//   mathWrite(*pBitStream, mLimitPlaneAnchor2);
//   mathWrite(*pBitStream, mLimitPlaneNormal2);
//   mathWrite(*pBitStream, mLimitPlaneAnchor3);
//   mathWrite(*pBitStream, mLimitPlaneNormal3);
//   mathWrite(*pBitStream, mLimitPlaneAnchor4);
//   mathWrite(*pBitStream, mLimitPlaneNormal4);
//   pBitStream->write(mCollisionEnabled);
//   pBitStream->write(mHW);
//}
//
//void fxJointData::unpackData(BitStream* pBitStream)
//{
//   Parent::unpackData(pBitStream);
//
//   mJointType = (physJointType)pBitStream->readRangedU32(0, 9);
//   pBitStream->read(&mTwistLimit);
//   pBitStream->read(&mSwingLimit);
//   pBitStream->read(&mSwingLimit2);
//   pBitStream->read(&mHighLimit);
//   pBitStream->read(&mLowLimit);
//   pBitStream->read(&mHighRestitution);
//   pBitStream->read(&mLowRestitution);
//   pBitStream->read(&mDistanceLimit);
//   pBitStream->read(&mMaxForce);
//   pBitStream->read(&mMaxTorque);
//   pBitStream->read(&mMotorForce);
//   pBitStream->read(&mMotorSpeed);
//   pBitStream->read(&mJointSpring);
//   pBitStream->read(&mSwingSpring);
//   pBitStream->read(&mTwistSpring);
//   pBitStream->read(&mSpringDamper);
//   pBitStream->read(&mSpringTargetAngle);
//   mathRead(*pBitStream, &mAxisA);
//   mathRead(*pBitStream, &mAxisB);
//   mathRead(*pBitStream, &mNormalA);
//   mathRead(*pBitStream, &mNormalB);
//   
//   mathRead(*pBitStream, &mLocalAnchor0);
//   mathRead(*pBitStream, &mLocalAnchor1);
//   mathRead(*pBitStream, &mLocalAxis0);
//   mathRead(*pBitStream, &mLocalAxis1);
//   mathRead(*pBitStream, &mLocalNormal0);
//   mathRead(*pBitStream, &mLocalNormal1);
//
//   mathRead(*pBitStream, &mLimitPoint);
//   mathRead(*pBitStream, &mLimitPlaneAnchor1);
//   mathRead(*pBitStream, &mLimitPlaneNormal1);
//   mathRead(*pBitStream, &mLimitPlaneAnchor2);
//   mathRead(*pBitStream, &mLimitPlaneNormal2);
//   mathRead(*pBitStream, &mLimitPlaneAnchor3);
//   mathRead(*pBitStream, &mLimitPlaneNormal3);
//   mathRead(*pBitStream, &mLimitPlaneAnchor4);
//   mathRead(*pBitStream, &mLimitPlaneNormal4);
//   pBitStream->read(&mCollisionEnabled);
//   pBitStream->read(&mHW);
//}
//
