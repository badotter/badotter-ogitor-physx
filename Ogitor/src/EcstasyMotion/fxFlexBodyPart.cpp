////////////////////////////////////////////////////////////////////////////////////////////////////
//  fxFlexBodyPart.cpp
//  Chris Calef
//
////////////////////////////////////////////////////////////////////////////////////////////////////
//#include "core/stl_fix.h"

//#include "platform/platform.h"
////#include "audio/audio.h"
//#include "console/consoleTypes.h"
//#include "core/stream/bitstream.h"
////#include "editor/editor.h"
//#include "math/mathio.h"
//#include "ts/tsShapeInstance.h"
//#include "collision/ConcretePolyList.h"

//#include "T3D/player.h"
//#include "T3D/gameBase/gameConnection.h"
//#include "EcstasyMotion/SQLiteObject.h"

#include "EcstasyMotion/fxFlexBodyPart.h"
#include "EcstasyMotion/physRigidBody.h"
#include "EcstasyMotion/nxRigidBody.h"
#include "EcstasyMotion/physJoint.h"
#include "EcstasyMotion/nxJoint.h"
#include "EcstasyMotion/fxJoint.h"
//#include "T3D/physicsBAG/fxFluid.h"

#include "OgitorsScriptConsole.h"
extern Ogitors::OgitorsScriptConsole *gConsole;
extern SQLiteObject *gSQL;

//#include "T3D/physicsBAG/myStream.h"

extern int mouseValue;

void vertSorter(physVertSort *vs, int numVS);

///////////////////////////////////////////////////////


  void onContactNotify(NxContactPair &pair, NxU32 events)
  {
     if ((pair.actors[0]->userData)&&(((nxRigidBody*)pair.actors[0]->userData)->mPhysUser)) {
        //NxShape *shp = *(pair.actors[0]->getShapes());
        iPhysUser *kPhysUser = ((nxRigidBody*)pair.actors[0]->userData)->mPhysUser;
        //if ((kFlexBody->mIsAnimating)&&(!(kFlexBody->mIsPlayer))) kFlexBody->mStopAnimating = true;
		kPhysUser->onCollision(NULL,0);
        //kFlexBody->stopThread(0);
        //kFlexBody->mIsAnimating = false;
        //if (kFlexBody->mIsKinematic) kFlexBody->clearKinematic();
        //for (unsigned int i = 0; i < kFlexBody->mNumBodyParts; i++) {
        //   kFlexBody->mBodyParts[i]->updatePositionFromRB();
        //}
        //Con::errorf("actor[0]: %s",kFlexBody->getShapeInstance()->getShape()->mSourceResource->name);
     }
     if ((pair.actors[1]->userData)&&(((nxRigidBody*)pair.actors[1]->userData)->mPhysUser)) {
        iPhysUser *kPhysUser = ((nxRigidBody*)pair.actors[1]->userData)->mPhysUser;
        //NxShape *shp = *(pair.actors[1]->getShapes());
        //if ((kFlexBody->mIsAnimating)&&(!(kFlexBody->mIsPlayer))) kFlexBody->mStopAnimating = true;
		kPhysUser->onCollision(NULL,0);
        //if (kFlexBody->mIsKinematic) kFlexBody->clearKinematic();
        //kFlexBody->mIsAnimating = false;
        //kFlexBody->stopThread(0);
        //Con::errorf("actor[1]: %s",kFlexBody->getShapeInstance()->getShape()->mSourceResource->name);
     }
  }
/*
  onTrigger:
  		//if (otherShape.userData) {
          //if (((physShapeData *)otherShape.userData)->mType==PHYS_CONVEX_PART) {
          //   if (((nxRigidBody*)otherShape.getActor().userData)->mPhysUser) {
          //      fxFlexBody *kFlexBody = ((nxRigidBody*)otherShape.getActor().userData)->mFlexBody;
          //      if ((kFlexBody->mIsKinematic)&&(!(kFlexBody->mIsPlayer))) kFlexBody->clearKinematic();
          //   }
          //}
       //}
*/

//IMPLEMENT_CO_NETOBJECT_V1(fxFlexBodyPart);

fxFlexBodyPart::fxFlexBodyPart()
{
  //mNetFlags.clear(Ghostable);
  //mTypeMask |= StaticObjectType;

  //mDataBlock = NULL;
  mFlexBody = NULL;
  mRB = NULL;
  mJoint = NULL;

  //iPhysUser data
  mTempForce = Ogre::Vector3::ZERO;
  mTempPos = Ogre::Vector3::ZERO;
  mHasTempForce = false;
  mHasTractorBeam = false;
  mHasTractorSpring = false;
  mHasSpring = false;
  mIsInflictor = false;
  mInflictMultiplier = 0.0;

  mCurrPosition = Ogre::Vector3::ZERO;
  mLastPosition = Ogre::Vector3::ZERO;
  mDeltaPos = 0.0;
  mCurrVelocity = Ogre::Vector3::ZERO;
  mCurrForce = Ogre::Vector3::ZERO;
  mCurrTorque = Ogre::Vector3::ZERO;
  mGlobalForce = Ogre::Vector3::ZERO;
  mGlobalTorque = Ogre::Vector3::ZERO;

  mInitialOrientation = Ogre::Quaternion::IDENTITY;
  mSDKRot = Ogre::Quaternion::IDENTITY;
  mSDKInverse = Ogre::Quaternion::IDENTITY;

  mCurrMS = 0;
  mEndMS = 0;
  mNodeIndex = 0;//change to int, initialize to -1?
  mBoneIndex = 0;
  mPartID = -1;
  mParentIndex = -1;
  mChildIndex = -1;
  mParentBodyPart = NULL;
  mChildBodyPart = NULL;
  mMesh = -1;
  mCurrTick = 0;
  mLastCollisionTick = 0;
  mLastDeactivateTick = 0;

  mAddClientBody = 0;
  //mIsKinematic = true;
  mIsClientOnly = true;
  
  //mDebugTextureName = NULL;
  //mDebugTextureHandle = NULL;
  //mDebugVerts = 0;
  //mDebugPrimitives = 0;

  mEntityType = PHYS_FLEX_BODY_PART;
}

fxFlexBodyPart::fxFlexBodyPart(int id)
{
   mPartID = id;
   //mNetFlags.clear(Ghostable);

   //mTypeMask |= StaticObjectType;

   //mDataBlock = NULL;
   mFlexBody = NULL;
   mRB = NULL;
   mJoint = NULL;

   //iPhysUser data
   mEntityType = PHYS_FLEX_BODY_PART;
   mEntitySubType = PHYS_SUB_UNDEFINED;
   mTempForce = Ogre::Vector3::ZERO;
   mTempPos = Ogre::Vector3::ZERO;
   mTempDamage = 0.0;
   mHasTempForce = false;
   mHasTractorBeam = false;
   mHasTractorSpring = false;
   mHasWeapon = false;
   mIsStriking = false;
   mHasCollisionWaiting = false;
   mHasCollision = false;
   mDelayCollisionStep = 0;
   mIsInflictor = false;
   mInflictMultiplier = 0.0;

   mCurrPosition = Ogre::Vector3::ZERO;
   mCurrVelocity = Ogre::Vector3::ZERO;
   mCurrForce = Ogre::Vector3::ZERO;
   mCurrTorque = Ogre::Vector3::ZERO;
   mGlobalForce = Ogre::Vector3::ZERO;
   mGlobalTorque = Ogre::Vector3::ZERO;

   mForwardForce = 0.0;

   mInitialPosition = Ogre::Vector3::ZERO;
   mInitialLinearVelocity = Ogre::Vector3::ZERO;
   mInitialAngularVelocity = Ogre::Vector3::ZERO;

   mInitialOrientation = Ogre::Quaternion::IDENTITY;
   mSDKRot = Ogre::Quaternion::IDENTITY;
   mSDKInverse = Ogre::Quaternion::IDENTITY;

   mCurrMS = 0;
   mEndMS = 0;
   mCurrTick = 0;
   mNodeIndex = 0;//change to int, initialize to -1?
   mBoneIndex = 0;
   mParentIndex = -1;
   mChildIndex = -1;
   mParentBodyPart = NULL;
   mChildBodyPart = NULL;
   mMesh = -1;
   mLastCollisionTick = 0;

   mImpulseForceStep = 0xFFFF;

   mAddClientBody = 0;
   mIsClientOnly = true;

   //mDebugTextureName = NULL;
   //mDebugTextureHandle = NULL;
   //mDebugVerts = 0;
   //mDebugPrimitives = 0;
}

fxFlexBodyPart::~fxFlexBodyPart()
{
   //if ((mPM)&&(mRB)) mPM->removeRigidBody(mRB);//this seems to happen
	//after physDestroyScene, which is no good.
	//if ((mPM)&&(mRB)) delete mRB;//onRemove()?
}
//
//bool fxFlexBodyPart::onAdd()
//{
//	//if (mIsClientOnly) mNetFlags.set(IsGhost);
//	mNetFlags.set(IsGhost);//should always be ghost, as long as all physics is only client but
//	//flexbodies can be both.
//	//else {
//	//   mNetFlags.set(Ghostable);
//	//   setScopeAlways();
//	//}
//
//	if(!Parent::onAdd())
//		return false;
//
//	TSShapeInstance *kShapeInstance = mFlexBody->getShapeInstance();
//	TSShape *kShape = kShapeInstance->getShape(); 
//	mNodeName.insert(0,mDataBlock->mBaseNodeName);
//	mNodeIndex = kShape->findNode(mDataBlock->mBaseNodeName);
//	mParentIndex = kShape->nodes[mNodeIndex].parentIndex;
//	//mBoneIndex = mFlexBody->mIndexBones[mNodeIndex];
//	mBoneIndex = mFlexBody->mNumBodyParts;
//	//Con::errorf("flexbodypart %s boneindex: %d",mDataBlock->mBaseNodeName,mBoneIndex);
//
//	mForwardForce = mDataBlock->mForwardForce;
//
//	//Obsolete...
//	if ((!dStrncmp(mFlexBody->mShapeName,"Tree",4)) ||
//		(!dStrncmp(mFlexBody->mShapeName,"tree",4)))
//		mEntitySubType = PHYS_SUB_FLEX_TREE;
//	else 
//		mEntitySubType = PHYS_SUB_FLEX_BIPED;
//
//	bool isParentValid = false;
//	for (unsigned int i=0;i<mFlexBody->mNumBodyParts;i++) {
//		if (mFlexBody->mBodyParts[i]->mNodeIndex==mParentIndex) {
//			isParentValid = true;	
//			if (mFlexBody->mBodyParts[i]->mChildIndex==-1)
//			{	
//				mFlexBody->mBodyParts[i]->mChildIndex = mNodeIndex;
//				mFlexBody->mBodyParts[i]->mChildBodyPart = this;
//		   }
//		}
//	}
//
//	if (!isParentValid) {
//		int tempParent;
//		if (dStrlen(mDataBlock->mParentNodeName)>0) {
//			tempParent = kShape->findNode(mDataBlock->mParentNodeName);
//			if (tempParent>=0) {
//				for (unsigned int i=0;i<mFlexBody->mNumBodyParts;i++) {
//					if (mFlexBody->mBodyParts[i]->mNodeIndex==tempParent) {
//						mParentIndex = tempParent;
//						mFlexBody->mBodyParts[i]->mChildIndex = mNodeIndex;
//						mFlexBody->mBodyParts[i]->mChildBodyPart = this;
//						//Con::errorf("... found parent! %d",mParentIndex);
//						isParentValid = true;	
//					}
//				}
//			}
//		}
//
//		tempParent = mParentIndex;
//		while (!isParentValid) {
//			for (unsigned int i=0;i<mFlexBody->mNumBodyParts;i++) {
//				if (mFlexBody->mBodyParts[i]->mNodeIndex==kShape->nodes[tempParent].parentIndex) {
//					mParentIndex = kShape->nodes[tempParent].parentIndex;
//					mFlexBody->mBodyParts[i]->mChildIndex = mNodeIndex;
//					mFlexBody->mBodyParts[i]->mChildBodyPart = this;
//					isParentValid = true;	
//				} 
//			}
//			while (!isParentValid) {//FIXED: while() loop prevents trouble in 
//				//situation where there is more than one non-bodypart node above 
//				//this one in the hierarchy.
//				if (tempParent==-1) {
//					isParentValid = true;
//					mParentIndex = -1;
//				} else tempParent = kShape->nodes[tempParent].parentIndex;
//			}
//		}//end while
//	}
//
//	for (unsigned int i=0; i < mFlexBody->mNumBodyParts; i++) {
//		if (mFlexBody->mBodyParts[i]->mNodeIndex == mParentIndex) {
//			mParentBodyPart = mFlexBody->mBodyParts[i];
//		}
//	}
//
//	//if (mIsClientOnly) {
//	//mNetFlags.set(IsGhost);
//	mPM = physManagerCommon::getPM();
//	mRB = mPM->createRigidBody();
//	//}
//
//	//Here: dealing with scale in setting rigid bodypart positions.  Node zero has global
//	//position info as well, so we can't scale it by objScale or it would be far away.
//	Ogre::Vector3 nodePos;
//	if (mBoneIndex>0)
//		nodePos = mFlexBody->getShapeInstance()->mNodeTransforms[mNodeIndex].getPosition() * mFlexBody->mObjScale;
//	else
//		nodePos = mFlexBody->getShapeInstance()->mNodeTransforms[mNodeIndex].getPosition();
//	//HERE: start with this in model space, not world space
//	//mCurrPosition.set(mFlexBody->mCurrPosition + nodePos);
//	//mCurrVelocity.set(mFlexBody->mCurrVelocity);
//	mCurrPosition.set(nodePos);
//	mLastPosition.set(nodePos);
//	mCurrVelocity = Ogre::Vector3::ZERO;
//
//	mRB->setEntityType(mEntityType);
//	mRB->setLinearPosition(mCurrPosition);
//
//   mIsInflictor = mDataBlock->mIsInflictor;
//   mRB->setInflictor(mIsInflictor);
//   if (mDataBlock->mTriggerDimensions.length()>0.0)
//   {
//	   //Con::errorf("setting bodypart trigger dimensions: %f %f %f",
//		//   mDataBlock->mTriggerDimensions.x,mDataBlock->mTriggerDimensions.y,mDataBlock->mTriggerDimensions.z);
//	   mRB->setTriggerDimensions(mDataBlock->mTriggerDimensions);
//	   mRB->setTriggerOffset(mDataBlock->mTriggerOffset);
//	   mRB->setTriggerOrientation(mDataBlock->mTriggerOrientation);
//	   mRB->setTriggerShapeType(mDataBlock->mTriggerShapeType);
//   } else {//okay, maybe if all flexbodyparts have triggers, but only inflictors do damage...
//	   mRB->setTriggerDimensions(mDataBlock->mDimensions);
//	   mRB->setTriggerOffset(mDataBlock->mOffset);
//	   mRB->setTriggerOrientation(mDataBlock->mOrientation);
//	   mRB->setTriggerShapeType(mDataBlock->mShapeType);
//   }
//   
//   //mRB->mInitialObjToWorld = mObjToWorld;
//   mPM->addRigidBodySetup(mRB);//FIX
//
//   if ((mDataBlock->mJointData)&&(mParentIndex>=0)) 
//	{
//	   //Ogre::Vector3 kAnchor,kParent,kFinal;
//	   //Ogre::Quaternion mDef,mDefI;
//	   mJoint = mPM->createJoint();
//	   mRB->setJoint(mJoint);
//	   setupJoint();
//	   mPM->addJointSetup(mJoint);
//   }
//
//   addToScene();
//   setPosition(nodePos);
//
//   mRB->setObjToWorld(mObjToWorld);
//
//   return true;
//}

void fxFlexBodyPart::reset()
{
   //Okay, whole other try:  using the mInitalPosition, mInitialOrientation method

	//HERE: come back and fix for Matrix3 funcs
   //Ogre::Matrix3 m,m2;
   //mInitialOrientation.setMatrix(&m);
   //mFlexBody->mInitialOrientation.setMatrix(&m2);
   ////m.mul(m2);
   //Ogre::Vector3 pos;
   //m2.mulP(mInitialPosition,&pos);
   ////mCurrPosition.set(mInitialPosition + mFlexBody->mCurrPosition);
   //mCurrPosition.set(mInitialPosition);
   ////mCurrPosition.set(pos + mFlexBody->mCurrPosition);
   //m.setPosition(mCurrPosition);
   ////m.mul(m2);
   //setTransform(m);
   //Ogre::Quaternion q(m);
   //mRB->setLinearPosition(mCurrPosition);
   //mRB->setAngularPosition(q);//mInitialOrientation
   //mRB->setLinearVelocity(Ogre::Vector3(0,0,0));
   //mRB->setAngularVelocity(Ogre::Vector3(0,0,0));
   //mRB->updatePositionToActor();
   //mRB->updateVelocityToActor();

}

//void fxFlexBodyPart::onRemove()
//{
//	if ((mPM)&&(mRB)) mPM->removeRigidBody(mRB);
//   if ((mPM)&&(mJoint)) mPM->removeJoint(mJoint);
//}
//
//bool fxFlexBodyPart::onNewDataBlock(GameBaseData* pGameBaseData, bool reload)
//{
//  mDataBlock = dynamic_cast<fxFlexBodyPartData*>(pGameBaseData);
//  if (!mDataBlock || !Parent::onNewDataBlock(pGameBaseData,reload))
//    {
//      return false;
//    }
//
//  mIsClientOnly = mFlexBody->mIsClientOnly;
//  mPM = mFlexBody->mPM;
//
//  // Have parent class do the rest
//  scriptOnNewDataBlock();
//
//  return true;
//}

//void fxFlexBodyPart::processTick(const Move* pMove)
//{
//  Parent::processTick(pMove);//before update position?
//  
//  mCurrMS++;//now tracking once per 32 MS tick
//}

void fxFlexBodyPart::updatePositionFromRB()
{
	//HERE: many changes... commenting out entirely for the moment.
	/*
   //Ogre::Matrix3 kTransform;
   //mRB->mAngularPosition.setMatrix(&kTransform);

   //mCurrPosition = mRB->mLinearPosition;
	//kTransform.setPosition(mRB->mLinearPosition);

	///////////////////////////////////////////
	Ogre::Vector3 relPos;
	Ogre::Matrix4 kTransform,pTransform,kAdj1,kAdj2,kAdj3;
	kTransform = Ogre::Matrix4::IDENTITY;
	pTransform = Ogre::Matrix4::IDENTITY;

	//if ((mParentIndex>=0)&&(mParentBodyPart)) {
	if (mRB) 
	{
		Ogre::Vector3 kPos = mFlexBody->getTransform().getPosition();
		Ogre::Matrix4 pTransform = mFlexBody->getTransform();
		pTransform.inverse();
		mLastPosition = mCurrPosition;
		mCurrPosition = mRB->getLinearPosition();
		//mCurrVelocity = mCurrPosition - mLastPosition;//CAREFUL... 

		//if ((mBoneIndex==1)&&(mFlexBody->mCurrTick%10))
		//{
			//Con::printf("updatePosFromRB: currPosition = %f %f %f",mCurrPosition.x,mCurrPosition.y,mCurrPosition.z);
		//	if (mCurrVelocity.length()>0.5)
		//		Con::errorf("crazy velocity here...");
		//}
		mDeltaPos = (mCurrPosition - mLastPosition).length();

		//Note: should eventually work in the possibility here that joints break.  For now, the check for mJointData only filters out bodypart 0.
		if (mDataBlock->mJointData) 
		{
			mRB->setAngularPositionMatrix(kTransform);
			Ogre::Matrix3 m1 = kTransform;
			//m1.mul(pTransform);
			Ogre::Matrix3 temp = pTransform;
			temp.mul(m1);
			m1 = temp;

			//Ogre::Quaternion q1(m1);
			//m1.mul(pTransform);
			//Ogre::Quaternion q2(m1);
			
			if (mFlexBody->getSDK())
			{
				Ogre::Matrix3 sdkTransform,tTransform;
				mSDKInverse.setMatrix(&sdkTransform);
				m1.mul(sdkTransform);
			}
			//relPos = (mRB->getLinearPosition() - kPos);
			Ogre::Vector3 globalPos = mRB->getLinearPosition();
			Ogre::Vector3 adjPos;
			pTransform.mulP(globalPos,&adjPos);
			relPos = adjPos - kPos;
			adjPos /= mFlexBody->mObjScale;
			m1.setPosition(adjPos);//(relPos);

			mFlexBody->getShapeInstance()->mNodeTransforms[mNodeIndex] = m1;
			mObjToWorld = m1;
			mObjToWorld.setPosition(adjPos);//(mRB->getLinearPosition());

			updateForcesToRB();//TEMP turned off for GA

			
			//p = mBodyParts[i]->mParentBodyPart->mRB->getAngularPosition();
			//	p.inverse();
			//	q *= p;
			//} else {
			//	q *= kRot;
			//}
			//nodeRotations[nodeRotations.size()-1].set(q);
			




			//if (mFlexBody->getSDK())
			//{
				//relPos = (mRB->getLinearPosition() - mParentBodyPart->mRB->getLinearPosition());
			//	Ogre::Vector3 sdkPos,testPos;
			//	mParentBodyPart->mSDKInverse.mulP(relPos,&sdkPos);
				//mParentBodyPart->mSDKRot.mulP(relPos,&testPos);
				//Con::errorf("%d sdkInverse: (%3.2f,%3.2f,%3.2f,%3.2f), sdkRot  (%3.2f,%3.2f,%3.2f,%3.2f)",mPartID,
				//	mSDKInverse.x,mSDKInverse.y,mSDKInverse.z,mSDKInverse.w,mSDKRot.x,mSDKRot.y,mSDKRot.z,mSDKRot.w);
				//Con::errorf("before: %3.2f %3.2f %3.2f, after: %3.2f %3.2f %3.2f, test: %3.2f %3.2f %3.2f",
				//	relPos.x,relPos.y,relPos.z,sdkPos.x,sdkPos.y,sdkPos.z,testPos.x,testPos.y,testPos.z);
			//	m1.setPosition(sdkPos);
			//} 
			//else m1.setPosition(relPos);
		} else {
			mRB->setAngularPositionMatrix(kTransform);
			//kTransform = Ogre::Quaternion::IDENTITY;//HERE...?
			kTransform.setPosition(mRB->getLinearPosition());

			if (mFlexBody->getSDK())
			{
				Ogre::Matrix3 sdkTransform;
				mSDKRot.setMatrix(&sdkTransform);
				mSDKInverse.setMatrix(&sdkTransform);
				kTransform.mul(sdkTransform);
			}
			if (mFlexBody->mClearBodyAnimating==false)
			//if (0)
			{
				Ogre::Vector3 kPos,kFloorPos;
				kPos = kTransform.getPosition();
				//Con::printf("setting new flexbody position: %f %f %f kinematic %d RB %d",
				//	kPos.x,kPos.y,kPos.z,mIsKinematic,mRB->getIsKinematic());
				//kFloorPos.set(kPos.x,kPos.y,0.0);//HERE: ghetto fix for floating dudes on getting up from ragdoll...
				//      better fix would be a raycast straight down, looking for (interiors/rigid bodies)?
				kTransform.setPosition(kPos);//(kFloorPos);
				mFlexBody->setTransform(kTransform);
				//mFlexBody->setRenderTransform(kTransform);
				mFlexBody->mCurrPosition = kPos;//kFloorPos;//kTransform.getPosition();

				relPos.set(0,0,0);//kPos - kFloorPos ;
				//Con::printf("flexbody setting pos from bodypart: %f %f %f",
				//	kPos.x,kPos.y,kPos.z);
				kTransform = Ogre::Quaternion::IDENTITY;
				kTransform.setPosition(relPos);//relPos//Now we do need position of hip node relative to floor position.
				mFlexBody->getShapeInstance()->mNodeTransforms[mNodeIndex] = kTransform;
			} else {
				//mRB->setAngularPositionMatrix(kTransform);
				//Ogre::Matrix3 m1 = kTransform;
				////m1.mul(pTransform);
				//Ogre::Matrix3 temp = pTransform;
				//temp.mul(m1);
				//m1 = temp;

				Ogre::Matrix3 m1;
				m1 = Ogre::Quaternion::IDENTITY;
				Ogre::Vector3 globalPos = mRB->getLinearPosition();
				Ogre::Vector3 adjPos;
				pTransform.mulP(globalPos,&adjPos);
				relPos = adjPos - kPos;
				m1.setPosition(adjPos);

				mFlexBody->getShapeInstance()->mNodeTransforms[mNodeIndex] = m1;
				mFlexBody->mClearBodyAnimating = false;

			}

			//Ogre::Vector3 pos = mFlexBody->mCurrPosition;
			//Con::printf("RB base node pos: %f %f %f",pos.x,pos.y,pos.z);

			////mRB->setAngularPositionMatrix(kTransform);
			////mFlexBody->mDefaults[mNodeIndex].setMatrix(&kAdj1);
			////kTransform.mul(kAdj1);

			//set any nodes that are up the hierarchy from the first one we're using
			for (int fn=(mFlexBody->mFirstNode-1);fn>=0;fn--) 
			{
				bool nodeBusy = false;//but deal with weird situations where a "parent" is 
				//actually down the hierarchy from a child node.  (BT fish models)
				for (unsigned int i=0;i<mFlexBody->mNumBodyParts;i++) 
					if (mFlexBody->mBodyParts[i]->mNodeIndex==fn) 
						nodeBusy=true;
				if (nodeBusy==false) {
					mFlexBody->getShapeInstance()->mNodeTransforms[fn] = kTransform;
					//Con::printf("setting weird node: %d",fn);
				}
			}
		}
	}

	relPos = mRB->getLinearPosition();
	setPosition(relPos);

	resetWorldBox();
	*/
}

void fxFlexBodyPart::updatePositionToRB()
{


	//mCurrVelocity = mCurrPosition - mLastPosition;

	/*
   bool kIsServer = isServerObject();
   if (kIsServer) return;//phsyics only done on the client, drawn on the client, no need to keep
   //track of bodyparts on the server,

   Ogre::Matrix3 kTransform,kBodyTransform,kPartTransform;
   Ogre::Quaternion q,q2,q3,kDef,kDefI,kBody;
   Quat16 defRot16;
   Ogre::Vector3 kPos,kPosBase;
   kPos = Ogre::Vector3::ZERO;

   kBodyTransform = mFlexBody->getTransform();
   if ( mIsNaN_F(kBodyTransform[0]) || mIsNaN_F(kBodyTransform[1]) || mIsNaN_F(kBodyTransform[2]) ||
	   mIsNaN_F(kBodyTransform[4]) || mIsNaN_F(kBodyTransform[5]) || mIsNaN_F(kBodyTransform[6]) ||
	   mIsNaN_F(kBodyTransform[8]) || mIsNaN_F(kBodyTransform[9]) || mIsNaN_F(kBodyTransform[10]) )
   {
	   Con::printf("body transform matrix is NaN0ed.");
	   Ogre::Matrix3 mat;  
	   mat = Ogre::Quaternion::IDENTITY;
	   mat.setPosition(mFlexBody->getPosition());
	   mFlexBody->setTransform(mat);
	   kBodyTransform = mat;
   }
   kBody.set(kBodyTransform);
   kTransform = mFlexBody->getShapeInstance()->mNodeTransforms[mNodeIndex];
   kPosBase = kTransform.getPosition();
   kPosBase *= mFlexBody->mObjScale;
   kBody.mulP(kPosBase,&kPos);//if (mBoneIndex>0)
   Ogre::Vector3 fbCurrPos = mFlexBody->mCurrPosition;
   Ogre::Vector3 bodyPos = kBodyTransform.getPosition();
   kPos += bodyPos;

   if (mIsNaN(kPos)) 
   {
	   Con::errorf("fxFlexBodyPart::updatePositionToRB - kPos is NaN0ed out!!!");
	   return;
   }
   setPosition(kPos);
   Ogre::Vector3 kDiff = kPos - mCurrPosition;
   //if ((mFlexBody->mPlaylistTick > 1)&&(kDiff.length() < 3.0))//<3.0: cheap hack, need to reset mPlaylistTick=0; in shapebase
	mLastPosition = mCurrPosition;
   //else
	//   mLastPosition = kPos;
   mCurrPosition = kPos;
	//if ((mBoneIndex==1)&&(mFlexBody->mCurrTick%10))
	//{
	//	Con::warnf("updatePosToRB: currPosition = %f %f %f",mCurrPosition.x,mCurrPosition.y,mCurrPosition.z);
		//	if (mCurrVelocity.length()>0.5)
		//		Con::errorf("crazy velocity here...");
	//}
   //if (mFlexBody->mPlaylistTick>0)//HMMM.. currVelocity getting all f*ed up
   //{
   mCurrVelocity = mCurrPosition - mLastPosition;//UH OH - need this most of the time, to get velocity, but when
	   // Con::errorf("updating position to RB: velocity %f %f %f",mCurrVelocity.x,mCurrVelocity.y,mCurrVelocity.z);
   //}		
   //if (mBoneIndex==0) 
   //{
	  // Con::printf("updatePosToRB: currVelocity = %f %f %f",mCurrVelocity.x,mCurrVelocity.y,mCurrVelocity.z);
	  // if (mCurrVelocity.length()>0.5)
		 //  Con::errorf("crazy velocity here...");
   //}
   kPartTransform.mul(kBodyTransform,kTransform);//it gets down to the transition, it messes things up.
   if (mFlexBody->getSDK())
   {
	   Ogre::Matrix3 sdkTransform;
	   mSDKRot.setMatrix(&sdkTransform);
	   kPartTransform.mul(sdkTransform);
	   kTransform.mul(sdkTransform);
   }
   mObjToWorld = kPartTransform;
   mObjToWorld.setPosition(kPos);

   mRB->setLinearPosition(kPos);
   //if (mBoneIndex==0) Con::errorf("flexbodypart setting position: %3.2f %3.2f %3.2f bodyPos %3.2f %3.2f %3.2f ",
	//   kPos.x,kPos.y,kPos.z,bodyPos.x,bodyPos.y,bodyPos.z);

   
   q.set(kTransform);
   q *= kBody;
   mRB->setAngularPosition(q);

   Ogre::Vector3 kMin = Ogre::Vector3(-0.1,-0.1,-0.1);
   Ogre::Vector3 kMax = Ogre::Vector3(0.1,0.1,0.1);
   mObjBox.minExtents = kMin;//kPos + 
   mObjBox.maxExtents = kMax;//kPos + 
   resetWorldBox();
	*/
}

void fxFlexBodyPart::updateVelocityFromRB()//this is only necessary for the root object, flexbody
{
	mCurrVelocity = mRB->getLinearVelocity();
	//if (mCurrVelocity.length()>12.0)
	//	Con::errorf("gettin crazy velocity here!");
    //setVelocity(mCurrVelocity);
    //setAngularVelocity(mRB->mAngularVelocity);
}

void fxFlexBodyPart::updateVelocityToRB()
{
   //mCurrVelocity = getVelocity();
	if (mCurrVelocity.length()<10.0)
	//	Con::errorf("gettin crazy velocity here!");
	//else
	   mRB->setLinearVelocity(mCurrVelocity);
}

void fxFlexBodyPart::updateForcesToRB()
{
   mRB->setCurrForce(mCurrForce);
   mRB->setCurrTorque(mCurrTorque);
   //if (mGlobalForce.length() > 0.0)
	//   Con::errorf("found a global force: %f %f %f",mGlobalForce.x,mGlobalForce.y,mGlobalForce.z);
   mRB->setGlobalForce(mGlobalForce);
   mRB->setGlobalTorque(mGlobalTorque);

   if (mImpulseForceStep < ((physManagerCommon *)physManagerCommon::getPM())->mCurrStep)
   {
	   mCurrForce = Ogre::Vector3::ZERO;
	   mGlobalForce = Ogre::Vector3::ZERO;
	   mImpulseForceStep = 0xFFFF;
   }

   //mCurrForce = Ogre::Vector3::ZERO;
   //mCurrTorque = Ogre::Vector3::ZERO;
    //setAngularVelocity(mRB->mAngularVelocity);
}

void fxFlexBodyPart::setupRigidBody()
{
	/*
	unsigned int jj,va,vb,vc,ja,jb;
	va = 0; vb = 0; vc = 0;
	Ogre::Vector3 kVertsA[5000];//FIX!! vectorize...
	Ogre::Vector3 kVertsB[5000];//FIX!!
	Ogre::Vector3 kVertsC[5000];//FIX!!

	Ogre::Vector3 avgVertsA; avgVertsA = Ogre::Vector3::ZERO;
	Ogre::Vector3 avgVertsB; avgVertsB = Ogre::Vector3::ZERO;
	physVertSort sortVerts[5000];//FIX!!
	unsigned int numSortVerts;

	mEntityType = PHYS_FLEX_BODY_PART;
	mRB->setPhysUser((iPhysUser *)this);
	Ogre::Vector3 objScale = mFlexBody->mObjScale;
	//mRB->setEntityType(PHYS_FLEX_BODY_PART);

	dynamic_cast<nxRigidBody*>(mRB)->mTriggerActorOffset = mDataBlock->mTriggerActorOffset * objScale;

	//HERE: This needs to have the option of being a primitive (box,capsule,sphere.)
	//mRB->setShapeType(PHYS_SHAPE_CONVEX);
	mRB->setShapeType(mDataBlock->mShapeType);

	mRB->setActorGroup(mFlexBody->mActorGroup);//TEMP: all the individual flexbodies get their own 
	//mRB->setActorGroup(3);//                        collision groups, which may not be necessary.
	mRB->setNodeIndex(mNodeIndex);
	mRB->setDefaultQuat(mFlexBody->mDefaults[mNodeIndex]);

	if (mDataBlock->mDynamicFriction) 
		mRB->setDynamicFriction(mDataBlock->mDynamicFriction);
	else if (mFlexBody->mDataBlock->mDynamicFriction)
		mRB->setDynamicFriction(mFlexBody->mDataBlock->mDynamicFriction);

	if (mDataBlock->mStaticFriction) 
		mRB->setStaticFriction(mDataBlock->mStaticFriction);
	else if (mFlexBody->mDataBlock->mDynamicFriction)
		mRB->setStaticFriction(mFlexBody->mDataBlock->mStaticFriction);

	if (mDataBlock->mRestitution) 
		mRB->setRestitution(mDataBlock->mRestitution);
	else if (mFlexBody->mDataBlock->mRestitution)
		mRB->setRestitution(mFlexBody->mDataBlock->mRestitution);

	if (mDataBlock->mDensity) 
		mRB->setDensity(mDataBlock->mDensity);
	else if (mFlexBody->mDataBlock->mDensity)
		mRB->setDensity(mFlexBody->mDataBlock->mDensity);

	if (!mRB->getDensity()) 
	{
		mRB->setDensity(1.0);//to keep from zero
		Con::errorf("fxFlexBodyPart: don't forget to set density somewhere!");
	} //else Con::errorf("fxFlexBodyPart density: %f!",mRB->getDensity());

	if (mDataBlock->mDimensions)
		mRB->setDimensions(mDataBlock->mDimensions * objScale);
	if (mDataBlock->mOffset)
		mRB->setOffset(mDataBlock->mOffset * objScale);
	if (mDataBlock->mOrientation)
		mRB->setOrientation(mDataBlock->mOrientation);

	Ogre::Vector3 parPos,chiPos,diff,newDim,newOffs;
	parPos = mCurrPosition;
	if ((mChildIndex!=-1)&&(mDataBlock->mShapeType==PHYS_SHAPE_CAPSULE)&&(mDataBlock->mDimensions.z==0))
	{//Fix this for scale, later.
		chiPos = mFlexBody->mBodyParts[mFlexBody->mIndexBones[mChildIndex]]->mCurrPosition;
		diff = chiPos - parPos;
		//Con::printf("SETTING CAPSULE flexbodypart %d  child %d dimensions %f %f %f diff %f",mNodeIndex,mChildIndex,
		//	mDataBlock->mDimensions.x,mDataBlock->mDimensions.y,mDataBlock->mDimensions.z,diff.length());
		newDim.x = mDataBlock->mDimensions.x;
		newDim.z = diff.length() - (newDim.x * 2);//take out radius * 2, to avoid overlapping endcaps
		if (newDim.z < newDim.x) newDim.z = newDim.x; //keep from dropping smaller than a sphere.
		newDim.z *= 0.95;//and then a little smaller yet, to eliminate all contact
		newOffs.set(Ogre::Vector3(diff.length()/2,0,0));
		mRB->setDimensions(newDim);
		mRB->setOffset(newOffs);
	}

	Ogre::Vector3 kStartPos;
	//TEMP: this seems pretty weird, but it is operating under the assumption that it is faster to create the bodyparts
	//ahead of time for a model, and store them somewhere safely off in space, and then merely have to move them to 
	//where you need them when the body becomes physActive, than it would be to have to cook the mesh and do real work
	//while clearing kinematic at runtime..
	//if (0) kStartPos.set(mCurrPosition.x,mCurrPosition.y + ((nxPhysManager *)mPM)->mNumFlexbodies*2,mCurrPosition.z + 30000.0);
	//else kStartPos.set(mCurrPosition.x,mCurrPosition.y ,mCurrPosition.z);//+ ((nxPhysManager *)mPM)->mNumFlexbodies*2
	kStartPos = mCurrPosition;
	mRB->setLinearPosition(kStartPos);
	//}

	mRB->setLinearVelocity(mCurrVelocity);
	mRB->setKinematic(mDataBlock->mIsKinematic);
	mRB->setMaxTorque(mDataBlock->mTorqueMax);//FIX, change flex body data to MaxTorque, MinTorque

	if (mFlexBody->mDataBlock->mHW) mRB->setHW(true);

	TSShape *kShape = mFlexBody->getShapeInstance()->getShape();

	if (mDataBlock->mShapeType!=PHYS_SHAPE_CAPSULE)
	{

		//TEMP!  This is the only novodex-reliant piece of code in here, but
		//I need to figure out how to standardize this part.
		mRB->setStartMesh(mMesh);
		if (((nxPhysManager *)mPM)->mMeshes[mMesh]) {
			//do nothing?
		} else {
			const TSDetail * detail = &kShape->details[0];
			int ss = detail->subShapeNum;
			int od = detail->objectDetailNum;

			int start = kShape->subShapeFirstObject[ss];//hmm, why this?
			int end   = kShape->subShapeNumObjects[ss] + start;//why not 0 to mMeshObjects.size()?

			//Con::printf("%s  ParentVerts: %d, ChildVerts: %d, FarVerts: %d, weight threshold %f",mDataBlock->mBaseNodeName,
			//	mDataBlock->mNumParentVerts,mDataBlock->mNumChildVerts,mDataBlock->mNumFarVerts,mDataBlock->mWeightThreshold);

			for (unsigned int j = start; j < end; j++)
			{

				bool kGoAhead = true;
				//HERE: test j against a list of excluded meshes, by flexbody.
				//if on the list, break and continue to next one.
				//The way it works:  flexbody has mNumMeshExcludes, and mMeshExcludes array.  You loop 
				//through that, and if j is in there, you stop.
				for (unsigned int mX = 0; mX < mFlexBody->mNumMeshExcludes; mX++)
				{
					if (mFlexBody->mMeshExcludes[mX] == j) 
						kGoAhead = false;
				}


				const char *name = kShape->names[mFlexBody->getShapeInstance()->mMeshObjects[j].object->nameIndex];
				//if there's no mMeshObject, go ahead, else check to see that it matches name
				//HERE: Fix this, there may be bodyparts that span more than one mesh.  Go through the list and grab all verts
				//that are connected to a bone, whether or not they are on the same mesh.  Get rid of requirement to name
				//mesh object of interest, at either flexbody or flexbodypart level.  Should be able to just figure it
				//out automatically, reduce manual labor.

				//Q: what if kGoAhead was always true, and we figured it out below?
				//bool kGoAhead = false;
				//if (dStrlen(mDataBlock->mMeshObject)) { 
				//	if (!dStrcmp(name,mDataBlock->mMeshObject))
				//		kGoAhead = true;
				//} else if (dStrlen(mFlexBody->mDataBlock->mMeshObject)) {
				//	if (!dStrcmp(name,mFlexBody->mDataBlock->mMeshObject))
				//		kGoAhead = true;
				//} else kGoAhead = true;


				//if ((!mFlexBody->mDataBlock->mMeshObject)||(!dStrcmp(name,mFlexBody->mDataBlock->mMeshObject))) {
				if (kGoAhead) {

					TSSkinMesh *mesh = (TSSkinMesh *)mFlexBody->getShapeInstance()->mMeshObjects[j].getMesh(od);
					if ((mesh) && (mesh->batchData.nodeIndex.size()))
					{
						int iV = mesh->batchData.nodeIndex.size();
						if ((iV<10000000)&&(iV>-10000000))//crash prevention...
						{
							va = 0; vb = 0;
							//int nodeIndex = kShape->findNode(mFlexBodyPart->mDataBlock->mBaseNodeName);
							//This would give you what mBoneIndex has now, the shape node number for this bodypart.

							//HERE:  this is what we need:  convert to a nodeIndex local to this mesh (often 0,1,2)
							int nodeIndex = -1;
							for (unsigned int jn=0;jn<mesh->batchData.nodeIndex.size();jn++)
							{
								if (mesh->batchData.nodeIndex[jn] == mNodeIndex){ // = mBoneIndex) 
									nodeIndex = jn;
								}
							}

							//Con::errorf("bone index %d, vertex index %d, weight %d",mesh->boneIndex.size(),mesh->vertexIndex.size(),mesh->weight.size());
							for (jj=0;jj<5000;jj++) kVertsA[jj] = Ogre::Vector3::ZERO; //SLOW?

							Ogre::Vector3 sc = objScale;

							//float minX,maxX,minY,maxY,minZ,maxZ;
							//minX = 1000.0; minY = 1000.0; minZ = 1000.0;
							//maxX = -1000.0; maxY = -1000.0; maxZ = -1000.0;

							Ogre::Vector3 min; Ogre::Vector3 max;
							min = Ogre::Vector3::ZERO; max = Ogre::Vector3::ZERO;

							if ((mDataBlock->mBoundsMin.length()>0)||
								(mDataBlock->mBoundsMax.length()>0)) 
							{
									min = mDataBlock->mBoundsMin;
									max = mDataBlock->mBoundsMax;
									//Con::errorf("min %f %f %f, max %f %f %f",min.x,min.y,min.z,max.x,max.y,max.z);
							}

							//Con::errorf("%s %s mCurrPosition %3.2f %3.2f %3.2f verts %d initialVerts %d nodeIndex %d boneIndex %d",
							//	mFlexBody->mShapeName,mDataBlock->mBaseNodeName,mCurrPosition.x,mCurrPosition.y,mCurrPosition.z,
							//	mesh->verts.size(),mesh->initialVerts.size(), mesh->nodeIndex.size(),mesh->boneIndex.size());
							for (unsigned int jj=0; jj < mesh->boneIndex.size();  jj++) 
							{
								if (mesh->boneIndex[jj]==nodeIndex) 
								{
									//Con::errorf("%s weight threshold %f, mesh %f",mDataBlock->mBaseNodeName,mDataBlock->mWeightThreshold,mesh->weight[jj]);
									//if (mDataBlock->mWeightThreshold == 0.1) Con::errorf("mesh vertex %d weight: %f",jj,mesh->weight[jj]);
									if (mesh->weight[jj] > mDataBlock->mWeightThreshold) {
										Ogre::Vector3 p; p = Ogre::Vector3::ZERO;
										Ogre::Vector3 d; d = Ogre::Vector3::ZERO;
										p = mesh->batchData.initialVerts[mesh->vertexIndex[jj]];

										p*=sc;
										//p += (mFlexBodyPart->mCurrPosition - mFlexBody->mCurrPosition);
										//mFlexBody->getTransform().mulP(p,&d);//??
										p -= mCurrPosition;
										if (mFlexBody->getSDK())
										{
											mSDKInverse.mulP(p,&d);
										} else d = p;

										//TEMP: skipping all of that below, it doesn't work.
										kVertsA[va++] = d;
									}
								}
							}
							//find "center of mass" for this body 
							for (ja=0;ja<va;ja++) {
								avgVertsA += kVertsA[ja];
							}
							if (va) avgVertsA /= va;
							//else Con::errorf("trying to divide by 0 in flexbodypart::setupRigidBody!");
							//if (nodeIndex==26) Con::errorf("bounding box: %f to %f, %f to %f, %f to %f",minX,maxX,minY,maxY,minZ,maxZ);


							//HERE: check to see if there's negative ParentVerts or ChildVerts.  
							//If so, begin deleting vertices, based on their distance from the center,
							//in the direction of either the parent or the child center of mass.
							//New plan: if mNumParentVerts or mNumChildVerts < 0, then (for each case) 
							//sort kVertsA by distance from the appropriate point, ordered such that the
							//verts to be deleted are at the end of the list.  Then just reduce va by 
							//the appropriate amount and delete the verts on the end.
							//////////////////////////////////////////////////////////////////////////
							if (mDataBlock->mNumParentVerts < 0) {
								vb=0; vc=0; numSortVerts = 0;
								for (jj=0;jj<5000;jj++) { 
									kVertsB[jj] = Ogre::Vector3::ZERO;
									kVertsC[jj] = Ogre::Vector3::ZERO;
									sortVerts[jj].dist=0; 
									sortVerts[jj].index=0;
								}

								if (mParentIndex>=0) {
									//First, get the whole set.

									int parentNodeIndex = -1;
									for (unsigned int jn=0;jn<mesh->batchData.nodeIndex.size();jn++)
									{
										if (mesh->batchData.nodeIndex[jn] == mParentBodyPart->mNodeIndex)//mBoneIndex) 
											parentNodeIndex = jn;
									}

									for (jj=0; jj < mesh->boneIndex.size();  jj++) {
										//if (mesh->boneIndex[jj]==mParentBodyPart->mBoneIndex) {
										if (mesh->boneIndex[jj]==parentNodeIndex) {
											if (mesh->weight[jj] > mParentBodyPart->mDataBlock->mWeightThreshold) {
												Ogre::Vector3 p; p = Ogre::Vector3::ZERO;
												Ogre::Vector3 d; d = Ogre::Vector3::ZERO;
												p = mesh->batchData.initialVerts[mesh->vertexIndex[jj]];
												p*=sc;
												//mFlexBody->getTransform().mulP(p,&d);
												p -= mCurrPosition;
												if (mFlexBody->getSDK())
												{
													mSDKInverse.mulP(p,&d);
												} else d = p;

												kVertsB[vb++] = d;
											}
										}
									}
									for (jb=0;jb<vb;jb++) {
										avgVertsB += kVertsB[jb];
									}
									avgVertsB /= vb;
									//set up array of sortVerts structs with vert indices & distance
									for (ja=0;ja<va;ja++) {
										Ogre::Vector3 b; b = Ogre::Vector3::ZERO;
										Ogre::Vector3 d; d = Ogre::Vector3::ZERO;
										b = kVertsA[ja];
										d = avgVertsB - b;
										sortVerts[numSortVerts].index = ja;  
										sortVerts[numSortVerts++].dist = -d.length();  //negative, because this time I want
										//the closest vertices to the parent to drop off the list, by going to the end
									}

									//sort the sortVerts array
									vertSorter(sortVerts,numSortVerts);

									for (ja=0;ja<(va + mDataBlock->mNumParentVerts);ja++) {
										kVertsC[vc++] = kVertsA[sortVerts[ja].index];
									}
									va=0;
									for (ja=0;ja<vc;ja++) {
										kVertsA[va++] = kVertsC[ja];
									}
								}
							}

							//////////////////////////////////////////////////////////////////////////
							if (mDataBlock->mNumChildVerts < 0) {
								//HERE: this one is slightly different, in that if there is no child bodypart, then you sort 
								//the primary bodypart's verts by the distance from it's own center of mass rather than the child's.
								vb=0; vc=0; numSortVerts = 0;
								for (jj=0;jj<5000;jj++) {
									kVertsB[jj] = Ogre::Vector3::ZERO;
									kVertsC[jj] = Ogre::Vector3::ZERO;
									sortVerts[jj].dist=0;
									sortVerts[jj].index=0;
								}

								if (mChildIndex>=0) {
									//First, get the whole set.

									int childNodeIndex = -1;
									for (unsigned int jn=0;jn<mesh->batchData.nodeIndex.size();jn++)
									{
										if (mesh->batchData.nodeIndex[jn] == mChildBodyPart->mNodeIndex)//mBoneIndex) 
											childNodeIndex = jn;
									}

									for (jj=0; jj < mesh->boneIndex.size();  jj++) {
										//if (mesh->boneIndex[jj]==mChildBodyPart->mBoneIndex) {
										if (mesh->boneIndex[jj]==childNodeIndex) {
											if (mesh->weight[jj] > mChildBodyPart->mDataBlock->mWeightThreshold) {
												Ogre::Vector3 p; p = Ogre::Vector3::ZERO;
												Ogre::Vector3 d; d = Ogre::Vector3::ZERO;
												p = mesh->batchData.initialVerts[mesh->vertexIndex[jj]];
												p*=sc;
												//mFlexBody->getTransform().mulP(p,&d);
												p -= mCurrPosition;
												if (mFlexBody->getSDK())
												{
													mSDKInverse.mulP(p,&d);
												} else d = p;

												kVertsB[vb++] = d;
											}
										}
									}
									for (jb=0;jb<vb;jb++) {
										avgVertsB += kVertsB[jb];
									}
									if (vb) avgVertsB /= vb;

									//set up array of sortVerts structs with vert indices & distance
									for (ja=0;ja<va;ja++) {
										Ogre::Vector3 b; b = Ogre::Vector3::ZERO;
										Ogre::Vector3 d; d = Ogre::Vector3::ZERO;
										b = kVertsA[ja];
										d = avgVertsB - b;
										sortVerts[numSortVerts].index = ja;  
										sortVerts[numSortVerts++].dist = -d.length();  
									}

									//sort the sortVerts array
									vertSorter(sortVerts,numSortVerts);

									//and then grab the top (NumParentVerts) closest ones
									for (ja=0;ja<(va+mDataBlock->mNumChildVerts);ja++) {
										kVertsC[vc++] = kVertsA[sortVerts[ja].index];
									}
									va=0;
									for (ja=0;ja<vc;ja++) {
										kVertsA[va++] = kVertsC[ja];
									}
								}
							}

							//////////////////////////////////////////////////////////////////////////
							if (mDataBlock->mNumFarVerts < 0) {
								if (va>(-1 * mDataBlock->mNumFarVerts))
								{
									vb=0; vc=0; numSortVerts = 0;
									for (jj=0;jj<5000;jj++) {
										kVertsB[jj] = Ogre::Vector3::ZERO;
										kVertsC[jj] = Ogre::Vector3::ZERO;
										sortVerts[jj].dist=0;
										sortVerts[jj].index=0;
									}
									//set up array of sortVerts structs with vert indices & distance
									for (ja=0;ja<va;ja++) {
										Ogre::Vector3 b; b = Ogre::Vector3::ZERO;
										Ogre::Vector3 d; d = Ogre::Vector3::ZERO;
										b = kVertsA[ja];
										d = avgVertsA - b;
										sortVerts[numSortVerts].index = ja;  
										sortVerts[numSortVerts++].dist = d.length();  //now I do want positive distance, because without a child
										//part, I want to use childverts to subtract the most distant vertices from this bodypart's center.
									}

									//sort the sortVerts array
									vertSorter(sortVerts,numSortVerts);

									//and then grab the top (NumParentVerts) closest ones
									for (ja=0;ja<(va + mDataBlock->mNumFarVerts);ja++) {
										kVertsC[vc++] = kVertsA[sortVerts[ja].index];
									}
									va=0;
									for (ja=0;ja<vc;ja++) {
										kVertsA[va++] = kVertsC[ja];
									}
								}
							}

							//////////////////////////////////////////////////////////////////////////
							if (mDataBlock->mNumParentVerts > 0) {
								vb=0; vc=0; numSortVerts = 0;
								for (jj=0;jj<5000;jj++) { 
									kVertsB[jj] = Ogre::Vector3::ZERO;
									kVertsC[jj] = Ogre::Vector3::ZERO;
									sortVerts[jj].dist=0; 
									sortVerts[jj].index=0;
								}

								if (mParentIndex>=0) {//Now, if we have a parent, grab some verts.
									//First, get the whole set.

									int parentNodeIndex = -1;
									for (unsigned int jn=0;jn<mesh->batchData.nodeIndex.size();jn++)
									{
										if (mesh->batchData.nodeIndex[jn] == mParentBodyPart->mNodeIndex)//mBoneIndex) 
											parentNodeIndex = jn;
									}

									for (jj=0; jj < mesh->boneIndex.size();  jj++) {
										//if (mesh->boneIndex[jj]==mParentBodyPart->mBoneIndex) {
										if (mesh->boneIndex[jj]==parentNodeIndex) {
											if (mesh->weight[jj] > mParentBodyPart->mDataBlock->mWeightThreshold) {
												Ogre::Vector3 p; p = Ogre::Vector3::ZERO;
												Ogre::Vector3 d; d = Ogre::Vector3::ZERO;
												p = mesh->batchData.initialVerts[mesh->vertexIndex[jj]];
												p*=sc;
												//mFlexBody->getTransform().mulP(p,&d);
												p -= mCurrPosition;
												if (mFlexBody->getSDK())
												{
													mSDKInverse.mulP(p,&d);
												} else d = p;

												kVertsB[vb++] = d;
											}
										}
									}

									//set up array of sortVerts structs with vert indices & distance
									for (jb=0;jb<vb;jb++) {
										Ogre::Vector3 b; b = Ogre::Vector3::ZERO;
										Ogre::Vector3 d; d = Ogre::Vector3::ZERO;
										b = kVertsB[jb];
										d = avgVertsA - b;
										sortVerts[numSortVerts].index = jb;  
										sortVerts[numSortVerts++].dist = d.length();  
									}

									//sort the sortVerts array
									vertSorter(sortVerts,numSortVerts);

									//and then grab the top (NumParentVerts) closest ones
									for (ja=0;ja<mDataBlock->mNumParentVerts;ja++) {
										kVertsA[va++] = kVertsB[sortVerts[ja].index];
									}
								}
							}

							//////////////////////////////////////////////////////////////////////////
							if (mDataBlock->mNumChildVerts > 0) {
								vb=0; vc=0; numSortVerts = 0;
								for (jj=0;jj<5000;jj++) {
									kVertsB[jj] = Ogre::Vector3::ZERO;
									kVertsC[jj] = Ogre::Vector3::ZERO;
									sortVerts[jj].dist=0;
									sortVerts[jj].index=0;
								}

								if (mChildIndex>=0) {//Now, if we have a parent, grab some verts.
									//First, get the whole set.

									int childNodeIndex = -1;
									for (unsigned int jn=0;jn<mesh->batchData.nodeIndex.size();jn++)
									{
										if (mesh->batchData.nodeIndex[jn] == mChildBodyPart->mNodeIndex)//mBoneIndex) 
											childNodeIndex = jn;
									}

									for (jj=0; jj < mesh->boneIndex.size();  jj++) {
										//if (mesh->boneIndex[jj]==mChildBodyPart->mBoneIndex) {
										if (mesh->boneIndex[jj]==childNodeIndex) {
											if (mesh->weight[jj] > mChildBodyPart->mDataBlock->mWeightThreshold) {
												Ogre::Vector3 p; p = Ogre::Vector3::ZERO;
												Ogre::Vector3 d; d = Ogre::Vector3::ZERO;
												p = mesh->batchData.initialVerts[mesh->vertexIndex[jj]];
												p*=sc;
												//mFlexBody->getTransform().mulP(p,&d);
												p -= mCurrPosition;
												if (mFlexBody->getSDK())
												{
													mSDKInverse.mulP(p,&d);
												} else d = p;

												kVertsB[vb++] = d;
											}
										}
									}

									//set up array of sortVerts structs with vert indices & distance
									for (jb=0;jb<vb;jb++) {
										Ogre::Vector3 b; b = Ogre::Vector3::ZERO;
										Ogre::Vector3 d; d = Ogre::Vector3::ZERO;
										b = kVertsB[jb];
										d = avgVertsA - b;
										sortVerts[numSortVerts].index = jb;  
										sortVerts[numSortVerts++].dist = d.length();  
									}
									//sort the sortVerts array
									vertSorter(sortVerts,numSortVerts);

									//and then grab the top (NumParentVerts) closest ones
									for (ja=0;ja<mDataBlock->mNumChildVerts;ja++) {
										kVertsA[va++] = kVertsB[sortVerts[ja].index];
									}
								}
							}

							//one more loop through kVertsA, to rotate each vertex by the inverse of 
							//the part's default orientation... 
							for (jj=0;jj<5000;jj++) { 
								kVertsB[jj] = Ogre::Vector3::ZERO;
								kVertsC[jj] = Ogre::Vector3::ZERO;
							}
							Ogre::Vector3 curpos = mCurrPosition;

							Ogre::Quaternion mDefI = mFlexBody->mDefaults[mNodeIndex];
							mDefI.inverse();

							for (ja=0;ja<va;ja++) {
								Ogre::Vector3 a,b;
								a = kVertsA[ja];
								mDefI.mulP(a,&b);
								//kVertsA[ja] = b;
								//and then, stuff it into the RigidBody's mVerts array:
								((nxRigidBody *)mRB)->mVerts.increment(); //HERE: is this going to work? 
								if (mFlexBody->mDataBlock->mSDK==false) 
									((nxRigidBody *)mRB)->mVerts[((nxRigidBody *)mRB)->mVerts.size()-1] = b;//[MAYBE?]
								else 
									((nxRigidBody *)mRB)->mVerts[((nxRigidBody *)mRB)->mVerts.size()-1] = a;//[MAYBE?]
							}
						}
					}
				}
			}
			//if (mRB) Con::errorf("mVerts: %d",((nxRigidBody *)mRB)->mVerts.size());
		}
	}*/
}

void fxFlexBodyPart::setupJoint()
{
	/*
	Ogre::Vector3 kAnchor,kAxis,kNormal,kParent,kFinal;
	Ogre::Vector3 globalAnchor,childAnchor,parentAnchor,parentAxis,parentNormal;
	Ogre::Quaternion mDef,mDefI,mParent;
	//Quat16 q16;

	mJoint->setRB_A(mParentBodyPart->mRB);
	mJoint->setRB_B(mRB);
	mJoint->setJD(&mDataBlock->mJointData->mJD);

	kAnchor = mFlexBody->getShapeInstance()->mNodeTransforms[mNodeIndex].getPosition() * mFlexBody->mObjScale;
	kParent = mFlexBody->getShapeInstance()->mNodeTransforms[mParentIndex].getPosition() * mFlexBody->mObjScale;
	kAnchor -= kParent;
	mDef = mFlexBody->mDefaults[mParentIndex];
	mDefI = mDef.inverse();
	mDefI.mulP(kAnchor,&kFinal);
	if (mFlexBody->getSDK())
	{
		Ogre::Vector3 temp = kFinal;
		kFinal.set(0,0,temp.length());
	}
	//kAnchor = mFlexBody->getShapeInstance()->mNodeTransforms[mNodeIndex].getPosition() * mFlexBody->mObjScale;
	//kAnchor += mFlexBody->getPosition();
	//mJoint->setGlobalAnchor(kAnchor);
	parentAnchor = kFinal;
	childAnchor = Ogre::Vector3::ZERO;//zero relative to child bodypart origin

	mJoint->setLocalAnchor0(parentAnchor);
	mJoint->setLocalAnchor1(childAnchor);

	//mJoint->setGlobalNormal(globalNormal);//Right, so far apparently physx has no setGlobalNormal, ??? Bastards!
	//q16 = mFlexBody->getShapeInstance()->getShape()->defaultRotations[mNodeIndex];
	//q16.getOgre::Quaternion(&mDef);
	//Con::printf("defRotations %d: %3.2f %3.2f %3.2f %3.2f",mNodeIndex,mDef.x,mDef.y,mDef.z,mDef.w);

	//IMPORTANT:  This may not be the best place to do this, but it might be.  What we're doing is 
	//looking at only the child data for axis and normal, so the end user only has to define it once,
	//and then finding the parent axis and normal by rotating the child versions through the default
	//rotation of the child bodypart.  (Default rotation: the local difference in orientation between
	//a child bodypart and its parent.)
	q16 = mFlexBody->getShapeInstance()->getShape()->defaultRotations[mNodeIndex];
	q16.getOgre::Quaternion(&mDef);
	//mDef.inverse();
	if ((mDataBlock->mJointData->mLocalAxis1.length()>0)&&(mDataBlock->mJointData->mLocalNormal1.length()>0))
	{
		mDataBlock->mJointData->mLocalAxis1.normalize();
		mDef.mulP(mDataBlock->mJointData->mLocalAxis1,&kAxis);
		mJoint->setLocalAxis0(kAxis);
		mJoint->setLocalAxis1(mDataBlock->mJointData->mLocalAxis1);
		mDataBlock->mJointData->mLocalNormal1.normalize();
		mDef.mulP(mDataBlock->mJointData->mLocalNormal1,&kNormal);
		mJoint->setLocalNormal0(kNormal);
		mJoint->setLocalNormal1(mDataBlock->mJointData->mLocalNormal1);
	}
	//else if ((mDataBlock->mJointData->mAxisB.length()>0)&&(mDataBlock->mJointData->mNormalB.length()>0))//deprecated
	//{	
	//	mDataBlock->mJointData->mAxisB.normalize();
	//	mDef.mulP(mDataBlock->mJointData->mAxisB,&kAxis);
	//	mDataBlock->mJointData->mNormalB.normalize();
	//	mDef.mulP(mDataBlock->mJointData->mNormalB,&kNormal);
	//	mJoint->setAxisA(kAxis);//deprecated
	//	mJoint->setAxisB(mDataBlock->mJointData->mAxisB);//deprecated
	//	mJoint->setNormalA(kNormal);//deprecated
	//	mJoint->setNormalB(mDataBlock->mJointData->mNormalB);//deprecated
	//}


	if (mFlexBody->mDataBlock->mHW) mJoint->setHW(true);
*/
}

void fxFlexBodyPart::reconfigureJoint()
{

}

void fxFlexBodyPart::onWorldStep()
{
	if (!mRB)
		return;
	

	//std::ostringstream strResult;
	//strResult.str("");
	//strResult << "flexbody bone name  " << mBone->getName() ;
	//gConsole->addOutput(strResult.str());


	//HMMM... doing it wrong.  Doesn't seem to be 
	//Ogre::Quaternion quat;
	//Ogre::Matrix3 mat;
	//if (!strcmp(mBone->getName().c_str(),"rShldr"))
	//{
	//	mat.FromEulerAnglesXYZ(Ogre::Radian(0.0),Ogre::Radian(90.0),Ogre::Radian(0.0));
	//	quat.FromRotationMatrix(mat);
	//	mBone->setOrientation(quat);
	//	//gConsole->addOutput("found my right shoulder!!!!!!!!!!!");
	//} else if (!strcmp(mBone->getName().c_str(),"chest")) {
	//	mat.FromEulerAnglesXYZ(Ogre::Radian(90.0),Ogre::Radian(0.0),Ogre::Radian(0.0));
	//	quat.FromRotationMatrix(mat);
	//	mBone->setOrientation(quat);
	//	//gConsole->addOutput("found my chest!!!!!!!!!!!");
	//}

	//if (mRB->getIsKinematic())
	//{
	//	//if (mBoneIndex==0) Con::printf("bodypart 0 is kinematic, updating to actor");
	//	mRB->updatePositionToActor();   
	//	mRB->updateVelocityToActor();
	//} else {
	//	mRB->updatePositionFromActor();   
	//	mRB->updateVelocityFromActor();

	//	mLastPosition = mCurrPosition;
	//	mCurrPosition = mRB->getLinearPosition();

	//	//HMMM... but skeleton nodes aren't Ogre scene nodes, right?? Hmmm.
	//	//TEMP!!!

	//	//std::ostringstream strResult;
	//	//strResult.str("");
	//	//strResult << "flexbody bone name  " << mBone->getName() ;
	//	//gConsole->addOutput(strResult.str());

	//	if (mFlexBody->mFirstNode == mNodeIndex)
	//	{
	//		mFlexBody->mNode->setPosition(Ogre::Vector3(mCurrPosition.x,mCurrPosition.y,mCurrPosition.z));
	//		Ogre::Quaternion q = mRB->getAngularPosition();
	//		mFlexBody->mNode->setOrientation(q.w,q.x,q.y,q.z);
	//	} else {
	//		//mBone->setPosition(Ogre::Vector3(mCurrPosition.x,mCurrPosition.y,mCurrPosition.z));
	//		Ogre::Quaternion q = mRB->getAngularPosition();
	//		mBone->setOrientation(q.w,q.x,q.y,q.z);
	//	}
	//}

	//Now, call flexbody onWorldStep, if we're the last bodypart to update.
	if (mFlexBody->mNumBodyParts<1000)	//cheezy, checking to see if mFlexBody is valid, not garbage.
		if (this==mFlexBody->mBodyParts[mFlexBody->mNumBodyParts-1])
			mFlexBody->onWorldStep();//do this after all the rigid bodies have updated themselves
	
	
	//std::ostringstream os;
	//os << "flexbodypart ticking!  position " << mCurrPosition.x << ", " << mCurrPosition.y << ", " << mCurrPosition.z;
	//gConsole->addOutput("flexbodypart ticking!!!!");

	/*
   if (mCurrTick==1)
   {
      mInitialPosition = mCurrPosition;
      Ogre::Matrix3 m,m2,m3;
      m = getTransform();
      //Ogre::Quaternion q(m);
      //HERE: multiply or divide by parent's starting transform
      //m2 = mFlexBody->getTransform();
      //m3.mul(m2.inverse(),m);
      //Ogre::Quaternion q(m3);
      Ogre::Quaternion q(m);
      mInitialOrientation = q;
   }

	if (!mFlexBody->mIsPhysActive) {
		if (this==mFlexBody->mBodyParts[mFlexBody->mNumBodyParts-1]) 
			mFlexBody->updateTrigger();
		return;
	}

	if (mPM->getDebugRender())
	{
		if (mMesh>=0) 
		{
			NxConvexMesh *myMesh = ((nxPhysManager *)mPM)->getMesh(mMesh);
			if (myMesh) myMesh->saveToDesc(mMeshDesc);
		}
	}

	if ((mFlexBody->mClearIsAnimating)&&(this==mFlexBody->mBodyParts[0])) 
	{
		mFlexBody->mIsAnimating = false;
		mFlexBody->mClearIsAnimating = false;
	}

	if ((mFlexBody->mStopAnimating)&&(this==mFlexBody->mBodyParts[0])) 
		mFlexBody->stopAnimating();

	if (mHasCollisionWaiting)
	{
		//if (mDelayCollisionStep < ((physManagerCommon *)physManagerCommon::getPM())->mCurrStep)
		//{
		//	mFlexBody->onCollision(NULL);
		//	mFlexBody->mHasCollisionWaiting = false;
		//	mHasCollisionWaiting = false;
		//	mDelayCollisionStep = 0;
		//	Con::printf("Old school collision code: clearing delayCollisionStep");
		//}
	}

	if (mHasTractorBeam)
	{
		float p;  
		Ogre::Matrix3 camTransform;
		Ogre::Vector3 myPos;
		SimGroup *g = Sim::getClientGroup();
		for (SimGroup::iterator itr = g->begin(); itr != g->end(); itr++) 
		{//really, should verify that there's only one client.
			GameConnection *con = (GameConnection *) (*itr);
			ShapeBase *obj = dynamic_cast<ShapeBase *>(con->getControlObject());
			myPos = obj->getPosition();
			obj->getEyeTransform(&camTransform);
			//virtual void getCameraTransform(float* pos,Ogre::Matrix3* mat);
		}
		Ogre::Vector3 newPos; 
		Ogre::Matrix3 newCamTrans;
		newCamTrans = camTransform;
		newCamTrans.setPosition(Ogre::Vector3(0,0,0));
		newCamTrans.mulP(mTempPos,&newPos);
		newPos += myPos;
		mCurrPosition = newPos;
		//mCurrForce = newPos - mRB->getLinearPosition();
		//Ogre::Vector3 actualPos = mRB->getLinearPosition();
		//mCurrForce *= 10000.0;
		mIsKinematic = true;
		mRB->setKinematic(true);
		mRB->setLinearPosition(newPos);
		//Con::errorf("actual pos: %3.2f %3.2f %3.2f, desired pos: %3.2f %3.2f %3.2f, force %3.2f %3.2f %3.2f",actualPos.x,actualPos.y,actualPos.z,newPos.x,newPos.y,newPos.z,mCurrForce.x/10000,mCurrForce.y/10000,mCurrForce.z/10000);
		setPosition(newPos);
		if (mouseValue==0) {
			mHasTractorBeam = false;
			mIsKinematic = false;
			mRB->setKinematic(false);
			//mIsNoGravity = false;
			//mRB->setNoGravity(false);
			mTempPos = Ogre::Vector3::ZERO;
			mTempForce = Ogre::Vector3::ZERO;
		}

		mCurrTick++;
		//return;
	}


	if (mRB->getIsKinematic())
	{
		//if (mBoneIndex==0) Con::printf("bodypart 0 is kinematic, updating to actor");
		mRB->updatePositionToActor();   
		mRB->updateVelocityToActor();

	} else {
		if (mIsKinematic) //meaning this is the FIRST WORLDSTEP since turning non-kinematic
		{
			//if (mBoneIndex==0) Con::printf("bodypart turning off kinematic");
			//mRB->updatePositionToActor();   
			//mRB->updateVelocityToActor();
			mRB->updatePositionFromActor();   
			mRB->updateVelocityFromActor();
			Ogre::Vector3 vel = mRB->getLinearVelocity();
			mIsKinematic = false;
			//if (mBoneIndex==0)
			//{//nope, this didn't work either, damn.  wtf.
			//	Ogre::Vector3 kPos = getPosition();
			//	//Con::printf("hip node on first worldstep after turning kinematic: %f %f %f",
			//	//	kPos.x,kPos.y,kPos.z);
			//	Ogre::Vector3 newPos(kPos.x,kPos.y,0.0);
			//	mFlexBody->setPosition(newPos);//fix: do a raycast down instead!
			//}
			//if (!mFlexBody->mClearBodyAnimating) 
			//	mFlexBody->mClearBodyAnimating = true;
		}
		else 
		{
			//if (mBoneIndex==0) Con::printf("bodypart is not kinematic");
			
			//if (!mFlexBody->mActionUser->mIsResetting)
			//{
			//if (mBoneIndex==0) Con::printf("bodypart 0 is NOT kinematic, updating FROM actor");
			mRB->updatePositionFromActor();   
			mRB->updateVelocityFromActor();
			//} else {
			//	mRB->updatePositionToActor();   
			//	mRB->updateVelocityToActor();
			//if (mBoneIndex==0)
			//{//nope, this didn't work either, damn.  wtf.
			//	Ogre::Vector3 kPos = getPosition();
			//	//Con::printf("hip node non-kinematic: %f %f %f",
			//	//	kPos.x,kPos.y,kPos.z);
			//	Ogre::Vector3 newPos(kPos.x,kPos.y,0.0);
			//	mFlexBody->setPosition(newPos);//fix: do a raycast down instead!
			//}
			//}
			//if (mFlexBody->mClearBodyAnimating) 
			//	mFlexBody->mClearBodyAnimating = false;
			
			//Ogre::Quaternion kRot = mRB->getAngularPosition();
		}
	}

	//if (mHasTempForce)
	//{
	//	addForceAtPos(mTempForce,mTempPos);
	//	Con::errorf("Adding force at pos: %f %f %f",mTempForce.x,mTempForce.y,mTempForce.z);
	//	mHasTempForce = false;
	//}

	if (mFlexBody->mNumBodyParts<1000)	//cheezy, checking to see if mFlexBody is valid, not garbage.
		if (this==mFlexBody->mBodyParts[mFlexBody->mNumBodyParts-1])
			mFlexBody->onWorldStep();//do this after all the rigid bodies have updated themselves

	if ((mHasCollision)&&(mDelayCollisionStep < (mCurrTick-mFlexBody->mCollisionTime))) 
	{
		mLastCollisionTick = mCurrTick;
		mHasCollision = false;
		//Con::printf("turning off hasCollision for shape %s node %d collisionStep %d currTick %d",
		//	mFlexBody->mShapeName,mBoneIndex,mDelayCollisionStep,mCurrTick);
	}
*/
   mCurrTick++;
}

void fxFlexBodyPart::onCollision(iPhysUser *other,int action=0)
{/*
	//HERE:  do the castNxRay logic - what hit me, what is my damage multiplier, am I dead?
	//if ((other->mEntityType!=PHYS_FLEX_BODY_PART)&&(other->mEntityType!=PHYS_TERRAIN)) mFlexBody->doSomeDamage(100.0);//TEMP
	//Con::printf("flexbodypart oncollision! action %d actorname %s",
	//	action,mFlexBody->mActorName);
	if (!other) 
	{
		//Con::printf("no other physUser in collision");
		mFlexBody->onCollision(NULL);
	}
	//if (other)
	//{
	//	if ((other->mIsInflictor)&&(other->mInflictMultiplier>0)) mFlexBody->doSomeDamage(100.0);//TEMP
	//}
	if (mHasTempForce)
	{//Q: should this damage be applied to the mServerObject rather than this client entity? If there is one?

		if (mTempDamage) {
			mFlexBody->doSomeDamage(mTempDamage * mDataBlock->mDamageMultiplier);//new function, go into flexbody (so I have access to 
			//mServerObject) and apply the damage there.
			//For now, ignore damage, we're not making a game, we just want to be able to knock actors over.
			mFlexBody->stopAnimating();			
		}
		mTempForce *= mDataBlock->mForceMultiplier;
	}

	
	//if (mFlexBody->getDamageLevel()==100.0)
	//if (mFlexBody->getDamageValue()==1.0)
	if(!((nxPhysManager*)mPM)->mIsExiting && !((nxPhysManager*)mPM)->mIsReallyExiting)
	{
		if ((mFlexBody->mNumBodyParts<10000)&&(mFlexBody->mNumBodyParts>0)&&(other))//sanity check
		{
			if ((other->mIsInflictor)&&(other->mEntityType==PHYS_FLEX_BODY_PART)&&(mCurrTick>60))
			{
				//NOW, does node have children?  Have to deactivate the chain all the way down.
				//deactivateNodePlusChildNodes recursively calls itself until all nodes from here down
				//are also deactivated.
				//if (mBoneIndex==0)
				//	mFlexBody->stopAnimating();
				//else
				//if (mBoneIndex!=0)
				//{

				fxFlexBody *otherFB = dynamic_cast<fxFlexBodyPart *>(other)->mFlexBody;
				if (!dStrcmp(mFlexBody->mActorName,"DesertSoldier"))
				{
					mFlexBody->mTarget = otherFB;
					mFlexBody->mGoFullRagdoll = false;
					mFlexBody->mNoLegRagdoll = true;
					//mFlexBody->deactivateNodePlusChildNodes(mBoneIndex);
				}
				else if (!dStrcmp(otherFB->mActorName,"DesertSoldier"))
				{
					mFlexBody->onCollision(other,action);
					if (mFlexBody->getDamageLevel()==1.0)
						mFlexBody->mGoFullRagdoll = true;
					else
						mFlexBody->mGoFullRagdoll = false;
					mFlexBody->mNoLegRagdoll = false;
					mFlexBody->mBeenHit = true;
					mFlexBody->mBeenHitTick = mFlexBody->mCurrTick;
					float threadPos = mFlexBody->getThreadPos(0);
					char bodypartnum[20],damageMult[20],sequenceName[255];
					sprintf(bodypartnum,"%d",mBoneIndex);
					sprintf(damageMult,"%f",mDataBlock->mDamageMultiplier);
					sprintf(sequenceName,"%s",mFlexBody->getThreadSequenceName(0));
					if ((!dStrcmp(sequenceName,"zombieAttackL"))&&(threadPos>=0.7)&&
						(mDataBlock->mBodypartChain==PHYS_CHAIN_LEFT_ARM))
					{
						//do nothing here
					}
					else if ((!dStrcmp(sequenceName,"zombieAttackR"))&&(threadPos>=0.7)&&
						(mDataBlock->mBodypartChain==PHYS_CHAIN_RIGHT_ARM))
					{
						//do nothing here
					} else {
						Con::executef(mFlexBody,"onHit",mFlexBody->scriptThis(),bodypartnum,damageMult);
						mFlexBody->deactivateNodePlusChildNodes(mBoneIndex);
					}
				}
				else
				{
					mFlexBody->onCollision(other,action);
					mFlexBody->mGoFullRagdoll = false;
					mFlexBody->mNoLegRagdoll = true;
					mFlexBody->deactivateNodePlusChildNodes(mBoneIndex);
				}

				//}
			} 
		}
		mFlexBody->setDamageLevel(0.0);
		mHasCollision = true;
		mDelayCollisionStep = mCurrTick;
		//Con::printf("Setting delayCollisionStep: %d",mCurrTick);
		if ((mFlexBody->mChainParts[mDataBlock->mBodypartChain]==NULL)&&
			(mCurrTick>30))
		{
			if ((mBoneIndex>0)&&(mParentBodyPart))
			{
				if (mParentBodyPart->mIsKinematic)
				{
					mFlexBody->mChainParts[mDataBlock->mBodypartChain] = this;
					//Con::printf("chain %d setting chainpart: %d, currtick %d",
					//	mDataBlock->mBodypartChain,mBoneIndex,mCurrTick);
				}
			}
		}

	} else Con::errorf("damage value: %f",mFlexBody->getDamageValue());
	*/
}

bool fxFlexBodyPart::nodeOrChildHasCollision()
{//Check to see if this bodypart or any bodypart down the line, to the end of all parent chains,
	//has a collision or did have a collision within the last tick or two.

	if ((mHasCollision) || (mDelayCollisionStep > mCurrTick-mFlexBody->mCollisionTime))//mLastCollisionTick
	{
		//Con::printf("nodeOrChildHasCollision: this node %d has collision.",mBoneIndex);
		return true;
	}
	
	//HERE: something is breaking, because we are able to reactivate the chest node while 
	//I am still in direct contact with the left forearm node.
	for (unsigned int i=0;i<mFlexBody->mNumBodyParts;i++)
	{
		if (mFlexBody->mBodyParts[i]->mParentBodyPart == this)
		{
			//if ((mFlexBody->mBodyParts[i]->mHasCollision)||
			//	(mFlexBody->mBodyParts[i]->mDelayCollisionStep > mFlexBody->mBodyParts[i]->mCurrTick-3))
			if (mFlexBody->mBodyParts[i]->nodeOrChildHasCollision())
			{
				//Con::printf("nodeOrChildHasCollision: child node %d has collision.",i);
				return true;
			}
			//Con::printf("nodeOrChildHasCollision: child node %d does not have collision.",i);
		}
	}
	//Con::printf("nodeOrChildHasCollision: returning false.");
				
	return false;
}

void fxFlexBodyPart::onTrigger(iPhysUser *other,int action=0,int id=0)
{
	//HERE: if you're using bots, comment out or modify the mIsPlayer check.  If you want a player to interact with the 
	//world without going non-kinematic, do it like this.
	//if (!mFlexBody->mIsPlayer) {

	if (other) {
		if (other->mIsInflictor) {
			if ((action == 0)&&(mFlexBody->mNumBodyParts < 1000)) 
			{
				if (other->mEntityType==PHYS_FLEX_BODY_PART)
					mFlexBody->onTrigger((iPhysUser*)this,0,0);
				else if ( (other->mEntityType==PHYS_RIGID_BODY)&&
					(other->mFlexBody != mFlexBody) )//There, now _that_ should solve it OAFA.
					//(other != (iPhysUser*)(mFlexBody->mWeapon))&&// totally didn't work, sigh
					//(other != (iPhysUser*)(mFlexBody->mWeapon2)&&// need to know why
					//(other->mInflictMultiplier != mFlexBody->mInflictMultiplier)) )//Change to a flexbody ID
					mFlexBody->onTrigger();	
			}
			
			if (action == 1)
			{
				if (mFlexBody->mIsAnimating) mFlexBody->mStopAnimating = true;

				///////////////////////////////////
				//TEMP: this is for a flash flood simulation, where water washed boxes down hill.
				//if (dynamic_cast<fxFluid*>(other)->mFluid) {
				//	Ogre::Vector3 vel = dynamic_cast<fxFluid*>(other)->mFluid->getParticleVelocity(id);
				//	//Con::errorf("particle velocity: %3.2f %3.2f %3.2f",vel.x,vel.y,vel.z);
				//	vel *= 250.0;
				//	mRB->setGlobalForce(vel);
				//	//HERE: make a variable to record whether we've already been triggered this frame or not, 
				//	//to make sure we don't get multiple forces added simultaneously. (Results in flinging.)
				//}
				///////////////////////////////////
			} 
			else if (action == 2) 
			{
				mRB->setGlobalForce(Ogre::Vector3(0,0,0));
			}
		}
	}
}


void fxFlexBodyPart::addForceAtPos(const Ogre::Vector3 &kForce,const Ogre::Vector3 &kPos)
{	
	mRB->addForceAtPos(kForce,kPos);
}

void fxFlexBodyPart::lockTractorBeam(int type)
{	
	if (type==0) mHasTractorBeam = true;
	else if (type==1) mHasTractorSpring = true;
	//mIsNoGravity = true;
	//mRB->setNoGravity(true);
	mIsKinematic = true;
	mRB->setKinematic(true);
}

void fxFlexBodyPart::setupDebugRender()
{
	//Had to kill this for 1.8 import because of physx change, no more NxConvexMesh apparently, it's abstract.
	/*
   unsigned int i,j,numVerts,numTris,numIndices;
   numVerts = 0; numTris = 0; numIndices = 0;
   unsigned int kTris[5000];

   if (mMesh>=0) 
   {
      mDebugTextureHandle.set("scriptsAndAssets/data/textures/debug_flexbody.jpg",&GFXDefaultPersistentProfile,"debug_flexbody");
      NxConvexMesh *myMesh = ((nxPhysManager *)mPM)->getMesh(mMesh);
      if (myMesh) 
      {
         NxConvexMeshDesc myDesc;
         myMesh->saveToDesc(myDesc);
         numVerts = myDesc.numVertices;
         numTris = myDesc.numTriangles;
         numIndices = 0;

         U16 *indices = new U16[ numTris * 3 ];

         mDebugVerts.set( GFX, numVerts, GFXBufferTypeDynamic );
         dMemcpy(kTris,myDesc.triangles,numTris * myDesc.triangleStrideBytes);

         for (i=0;i<numTris;i++)
         {
            indices[numIndices++] = kTris[(i*3)+0];
            indices[numIndices++] = kTris[(i*3)+1];
            indices[numIndices++] = kTris[(i*3)+2];
         }

         GFXPrimitive pInfo;
         U16 *ibIndices;
         GFXPrimitive *piInput;

         pInfo.type = GFXTriangleList;
         //pInfo.type = GFXLineList;
         pInfo.numPrimitives = numTris;
         pInfo.startIndex = 0;
         pInfo.minIndex = 0;
         pInfo.numVertices = numVerts;

         mDebugPrimitives.set( GFX, numIndices, 1, GFXBufferTypeStatic );
         mDebugPrimitives.lock( &ibIndices, &piInput );
         dMemcpy( ibIndices, indices, numIndices * sizeof(U16) );
         dMemcpy( piInput, &pInfo, sizeof(GFXPrimitive) );
         mDebugPrimitives.unlock();

         delete [] indices;
      }
   }
   */
}

void fxFlexBodyPart::renderDebug()
{

	//Killing on 1.8 port, bunch of GFX stuff broken.

	//NxConvexMesh *myMesh = ((nxPhysManager *)mPM)->getMesh(mMesh);
	//NxConvexMeshDesc myDesc;
	//myMesh->saveToDesc(myDesc);
	/*
	if (mMesh>=0)
	{
		Ogre::Vector3 p,d;
		NxVec3 kVerts[1000];
		unsigned int kTris[1000];

		unsigned int numVerts,index;
		index = 0;

		PROFILE_START(fxFlexBodyPart_debugRender);
		if (mMeshDesc.isValid()) 
		{		
			numVerts = mMeshDesc.numVertices;
			dMemcpy(kVerts,mMeshDesc.points,numVerts * sizeof(NxVec3));//myDesc.pointStrideBytes
			//dMemcpy(kTris,mMeshDesc.triangles,myDesc.numTriangles * myDesc.triangleStrideBytes);

			Ogre::Quaternion mDef,mDefI;
			mDef = mFlexBody->mDefaults[mNodeIndex];
			mDefI = mDef;
			mDefI.inverse();

			GFXVertexPNTT *verts = new GFXVertexPNTT[ numVerts ];
			GFXVertexPNTT *vert;

			//if (mIsKinematic) 
			//{
			//	for (unsigned int i=0;i<numVerts;i++)
			//	{
			//		Ogre::Vector3 p,d,e;
			//		p.set(kVerts[i].x,kVerts[i].y,kVerts[i].z);
			//mDefI.mulP(p,&d);
			//		mObjToWorld.mulP(d,&e);
			//		vert = &verts[index++];
			//		vert->point.set(e.x,e.y,e.z);
			//		vert->texCoord.set(1.0,1.0);
			//	}
			//} else {
			for (unsigned int i=0;i<numVerts;i++)
			{
				Ogre::Vector3 p,d,e;
				p.set(kVerts[i].x,kVerts[i].y,kVerts[i].z);
				if (mFlexBody->getSDK())
					mSDKRot.mulP(p,&d);
				else d=p;
				//mDefI.mulP(p,&d);
				mObjToWorld.mulP(d,&e);
				vert = &verts[index++];
				vert->point.set(e.x,e.y,e.z);
				vert->texCoord.set(1.0,1.0);
			}
			//}


			GFXVertexPNTT *vbVerts = mDebugVerts.lock();
			dMemcpy( vbVerts, verts, sizeof(GFXVertexPNTT) * numVerts );
			mDebugVerts.unlock();


			GFX->setFillMode(GFXFillWireframe);
			//GFX->setZEnable(false);

			GFX->setTexture(0,mDebugTextureHandle);
			GFX->setTextureStageColorOp(0, GFXTOPModulate);
			GFX->setTextureStageColorOp(1, GFXTOPDisable);

			GFX->setVertexBuffer( mDebugVerts );
			GFX->setPrimitiveBuffer( mDebugPrimitives );
			GFX->drawPrimitives();

			//GFX->setZEnable(true);
			GFX->setFillMode(GFXFillSolid);

			delete [] verts;
			//Con::errorf("%s -- convex verts: %d, triangles: %d",mDataBlock->shapeName,myDesc.numVertices, myDesc.numTriangles);
		}
		PROFILE_END();
	}
	*/
}

/*
ConsoleMethod( fxFlexBodyPart, setTorque, void, 3, 3,"(Ogre::Vector3 kTorque)")
{
	Ogre::Vector3 kTorque;	
	dSscanf(argv[2], "%g %g %g", &kTorque.x, &kTorque.y, &kTorque.z);
	object->setTorque(kTorque);
	return;
}

ConsoleMethod( fxFlexBodyPart, setGlobalTorque, void, 3, 3,"(Ogre::Vector3 kTorque)")
{
	Ogre::Vector3 kTorque;	
	dSscanf(argv[2], "%g %g %g", &kTorque.x, &kTorque.y, &kTorque.z);
	object->setGlobalTorque(kTorque);
	return;
}

ConsoleMethod(fxFlexBodyPart,setForce,void,3,3,"Sets local force.")
{
	Ogre::Vector3 kForce( 0.0f,0.0f,0.0f );
	dSscanf( argv[2], "%f %f %f", &kForce.x, &kForce.y, &kForce.z );
	object->mRB->setCurrForce(kForce);
	return;
}

ConsoleMethod(fxFlexBodyPart,setGlobalForce,void,3,3,"Sets local force.")
{
	Ogre::Vector3 kForce( 0.0f,0.0f,0.0f );
	dSscanf( argv[2], "%f %f %f", &kForce.x, &kForce.y, &kForce.z );
	object->mRB->setGlobalForce(kForce);
	return;
}

ConsoleMethod(fxFlexBodyPart,setGlobalDelayForce,void,3,3,"Sets local force.")
{
	Ogre::Vector3 kForce( 0.0f,0.0f,0.0f );
	dSscanf( argv[2], "%f %f %f", &kForce.x, &kForce.y, &kForce.z );
	object->mRB->setGlobalDelayForce(kForce);
	object->mRB->setDelayStep(((physManagerCommon *)physManagerCommon::getPM())->mCurrStep);
	Con::errorf("setting delay force: %f %f %f step %d",kForce.x, kForce.y, kForce.z,((physManagerCommon *)physManagerCommon::getPM())->mCurrStep );
	return;
}

ConsoleMethod(fxFlexBodyPart,onCollision,void,2,2,"calls onCollision.")
{
	if ((object->mFlexBody->mNumBodyParts>0)&&
		(object->mFlexBody->mNumBodyParts<1000))
        object->mFlexBody->onCollision(NULL);
}

ConsoleMethod(fxFlexBodyPart,onDelayCollision,void,2,2,"onCollision, one tick later.")
{
	if ((object->mFlexBody->mNumBodyParts>0)&&
		(object->mFlexBody->mNumBodyParts<1000)&&
		(!(object->mFlexBody->mHasCollisionWaiting)))
	{
        object->mHasCollisionWaiting = true;
		object->mFlexBody->mHasCollisionWaiting = true;
		object->mDelayCollisionStep = ((physManagerCommon *)physManagerCommon::getPM())->mCurrStep;
	}
}

ConsoleMethod(fxFlexBodyPart,getSubType,int,2,2,"int getSubType()")
{
	return object->mEntitySubType;
}
*/


//////////////////////////////////////////////////////////////////

//
//
////IMPLEMENT_CONSOLETYPE(PlayerData)
////IMPLEMENT_SETDATATYPE(PlayerData)
////IMPLEMENT_GETDATATYPE(PlayerData)
//
//IMPLEMENT_CO_DATABLOCK_V1(fxFlexBodyPartData);
//
////IMPLEMENT_CONSOLETYPE(fxFlexBodyPartData)
////IMPLEMENT_SETDATATYPE(fxFlexBodyPartData)
////IMPLEMENT_GETDATATYPE(fxFlexBodyPartData)
//
//
//fxFlexBodyPartData::fxFlexBodyPartData()
//{
//  mBaseNodeName  = NULL;
//  mParentNodeName  = NULL;
//  mChildNodeName  = NULL;
//  mMeshObject = NULL;
//  mFlexBodyData  = NULL;
//  FlexBodyDataID = 0;
//  mPlayerData  = NULL;
//  mPlayerDataID = 0;
//  mJointData     = NULL;
//  JointDataID    = 0;
//
//  mDynamicFriction = 0.0;
//  mStaticFriction = 0.0;
//  mRestitution = 0.0;
//  mDensity = 0.0;
//  mDamageMultiplier = 1.0;
//  mForceMultiplier = 1.0;
//  mRagdollThreshold = 0.3;
//
//  mShapeType = PHYS_SHAPE_CONVEX;
//  mBodypartChain = PHYS_CHAIN_SPINE;
//
//  mOffset = Ogre::Vector3::ZERO;
//  mOrientation = Ogre::Vector3::ZERO;
//  mDimensions = Ogre::Vector3::ZERO;
//  
//  mTriggerShapeType = PHYS_SHAPE_SPHERE;
//  mTriggerOffset = Ogre::Vector3::ZERO;
//  mTriggerOrientation = Ogre::Vector3::ZERO;
//  mTriggerDimensions = Ogre::Vector3::ZERO;
//  mTriggerActorOffset = Ogre::Vector3::ZERO;
//
//  mBoundsMin = Ogre::Vector3::ZERO;
//  mBoundsMax = Ogre::Vector3::ZERO;
//  mForceMin = Ogre::Vector3::ZERO;
//  mForceMax = Ogre::Vector3::ZERO;
//  mTorqueMin = Ogre::Vector3::ZERO;
//  mTorqueMax = Ogre::Vector3::ZERO;
//  mIsKinematic   = false;//get rid of this, use RB
//  mIsInflictor   = false;
//  mNumParentVerts = 0;
//  mNumChildVerts = 0;
//  mNumFarVerts = 0;
//  mWeightThreshold = 0.0;
//}
//
//fxFlexBodyPartData::~fxFlexBodyPartData()
//{
//}
//
//bool fxFlexBodyPartData::preload(bool bServer, String &errorStr)
//{
//  if (!Parent::preload(bServer, errorStr))
//    {
//      return false;
//    }
//
//   if (!bServer) 
//   {
//      if( !mFlexBodyData && FlexBodyDataID != 0 )
//      {
//         if( Sim::findObject( FlexBodyDataID, mFlexBodyData ) == false)
//         {
//            Con::errorf( ConsoleLogEntry::General, "fxFlexBodyPartData::preload: Invalid packet, bad datablockId(mFlexBodyData): 0x%x", FlexBodyDataID );
//         }
//      }
//	  if( !mPlayerData && mPlayerDataID != 0 )
//      {
//         if( Sim::findObject( mPlayerDataID, mPlayerData ) == false)
//         {
//            Con::errorf( ConsoleLogEntry::General, "fxFlexBodyPartData::preload: Invalid packet, bad datablockId(mPlayerData): 0x%x", mPlayerDataID );
//         }
//      }
//      if( !mJointData && JointDataID != 0 )
//      {
//         if( Sim::findObject( JointDataID, mJointData ) == false)
//         {
//            Con::errorf( ConsoleLogEntry::General, "fxFlexBodyPartData::preload: Invalid packet, bad datablockId(mJointData): 0x%x", JointDataID );
//         }
//      }
//
//      //HERE: find the FlexBodyData, increment its NumBodyParts
//	  if (mFlexBodyData) {
//	    mFlexBodyData->mNumBodyParts++;
//     }
//   }
//
//  return true;
//}
//
//bool fxFlexBodyPartData::onAdd()
//{
//   if(!Parent::onAdd())
//      return false;
//   else
//   {
//	   //WTF, all kinds of shit broke in bodypartdata for some reason.  getName() doesn't work. 
//	   //SQLiteObject *sql = dynamic_cast<nxPhysManager*>(physManagerCommon::getPM())->getSQL();
//	   SQLiteObject *sql = new SQLiteObject();
//	   if (!sql) return true;
//	   if (sql->OpenDatabase("EcstasyMotion.db"))
//	   {
//		   char id_query[512],insert_query[512];
//		   int body_id,joint_id,result;
//		   sqlite_resultset *resultSet;
//
//		  // sprintf(id_query,"SELECT id FROM fxFlexBodyData WHERE name = '%s';",mFlexBodyData->getName());
//		  // result = sql->ExecuteSQL(id_query);
//		  // resultSet = sql->GetResultSet(result);
//		  // if (resultSet->iNumRows == 0)
//		  // {
//			 //  delete sql;	  
//			 //  return true;
//		  // } else if (resultSet->iNumRows == 1)
//			 //  body_id = dAtoi(resultSet->vRows[0]->vColumnValues[0]);
//
//		  // sprintf(id_query,"SELECT id FROM fxJointData WHERE name = '%s';",mJointData->getName());
//		  // result = sql->ExecuteSQL(id_query);
//		  // resultSet = sql->GetResultSet(result);
//		  // if (resultSet->iNumRows == 1)
//			 //  joint_id = dAtoi(resultSet->vRows[0]->vColumnValues[0]);
//		  // else joint_id = -1;
//
//		  // if (body_id)
//		  // {
//			 //  sprintf(insert_query,"INSERT INTO fxFlexBodyPartData (fxFlexBodyData_id,fxJointData_id,name) VALUES (%d,%d,'%s');",\
//				//   body_id,joint_id,"WTF, getName() doesn't work on fxFlexBodyPart.");
//			 //  result = sql->ExecuteSQL(insert_query);
//		  // }
//		   sql->CloseDatabase();
//	   }	   
//	   delete sql;
//	   return true;
//   }
//}
//
//void fxFlexBodyPartData::initPersistFields()
//{
//  Parent::initPersistFields();
//
//  addField("FlexBodyData", TYPEID< fxFlexBodyData >(), 
//        Offset(mFlexBodyData, fxFlexBodyPartData));
//  addField("PlayerData", TYPEID< SimObject >(), 
//        Offset(mPlayerData, fxFlexBodyPartData));
//  addField("JointData", TYPEID< fxJointData >(), 
//        Offset(mJointData, fxFlexBodyPartData));
//  addField("BaseNode",TypeString,
//        Offset(mBaseNodeName, fxFlexBodyPartData));
//  addField("ParentNode",TypeString,
//        Offset(mParentNodeName, fxFlexBodyPartData));
//  addField("ChildNode",TypeString,
//        Offset(mChildNodeName, fxFlexBodyPartData));
//  addField("MeshObject",TypeString,
//        Offset(mMeshObject, fxFlexBodyPartData));
//  addField("BodypartChain",TYPEID< physChainType >(),
//		Offset(mBodypartChain, fxFlexBodyPartData));
//  addField("DynamicFriction", Typefloat, 
//        Offset(mDynamicFriction, fxFlexBodyPartData));
//  addField("StaticFriction", Typefloat, 
//        Offset(mStaticFriction, fxFlexBodyPartData));
//  addField("Restitution", Typefloat, 
//        Offset(mRestitution, fxFlexBodyPartData));
//  addField("Density", Typefloat, 
//        Offset(mDensity, fxFlexBodyPartData));
//  addField("DamageMultiplier", Typefloat, 
//        Offset(mDamageMultiplier, fxFlexBodyPartData));
//  addField("ForceMultiplier", Typefloat, 
//        Offset(mForceMultiplier, fxFlexBodyPartData));
//  addField("ForwardForce", Typefloat, 
//        Offset(mForwardForce, fxFlexBodyPartData));
//  addField("ShapeType", TYPEID< physShapeType >(),
//		Offset(mShapeType, fxFlexBodyPartData));
//  addField("Offset",TypeOgre::Vector3,
//        Offset(mOffset, fxFlexBodyPartData));
//  addField("Orientation",TypeOgre::Vector3,
//        Offset(mOrientation, fxFlexBodyPartData));
//  addField("Dimensions",TypeOgre::Vector3,
//        Offset(mDimensions, fxFlexBodyPartData));
//  addField("TriggerShapeType", TYPEID< physShapeType >(),
//	  Offset(mTriggerShapeType, fxFlexBodyPartData));
//  addField("TriggerOffset", TypeOgre::Vector3,
//	  Offset(mTriggerOffset, fxFlexBodyPartData));
//  addField("TriggerOrientation", TypeOgre::Vector3,
//	  Offset(mTriggerOrientation, fxFlexBodyPartData));
//  addField("TriggerDimensions", TypeOgre::Vector3,
//	  Offset(mTriggerDimensions, fxFlexBodyPartData));
//  addField("TriggerActorOffset", TypeOgre::Vector3,
//	  Offset(mTriggerActorOffset, fxFlexBodyPartData));
//  addField("BoundsMin",TypeOgre::Vector3,
//        Offset(mBoundsMin, fxFlexBodyPartData));
//  addField("BoundsMax",TypeOgre::Vector3,
//        Offset(mBoundsMax, fxFlexBodyPartData));
//  addField("ForceMin",TypeOgre::Vector3,
//        Offset(mForceMin, fxFlexBodyPartData));
//  addField("ForceMax",TypeOgre::Vector3,
//        Offset(mForceMax, fxFlexBodyPartData));
//  addField("TorqueMin",TypeOgre::Vector3,
//        Offset(mTorqueMin, fxFlexBodyPartData));
//  addField("TorqueMax",TypeOgre::Vector3,
//        Offset(mTorqueMax, fxFlexBodyPartData));
//  addField("IsKinematic",TypeBool,
//        Offset(mIsKinematic, fxFlexBodyPartData));
//  addField("IsInflictor",TypeBool,
//        Offset(mIsInflictor, fxFlexBodyPartData));
//  addField("InflictMultiplier",Typefloat,
//        Offset(mInflictMultiplier, fxFlexBodyPartData));
//  addField("ParentVerts", Typeint, 
//        Offset(mNumParentVerts, fxFlexBodyPartData));
//  addField("ChildVerts", Typeint, 
//        Offset(mNumChildVerts, fxFlexBodyPartData));
//  addField("FarVerts", Typeint, 
//        Offset(mNumFarVerts, fxFlexBodyPartData));
//  addField("WeightThreshold", Typefloat, 
//        Offset(mWeightThreshold, fxFlexBodyPartData));
//  addField("RagdollThreshold", Typefloat, 
//        Offset(mRagdollThreshold, fxFlexBodyPartData));
//}
//
//void fxFlexBodyPartData::packData(BitStream* pBitStream)
//{
//   Parent::packData(pBitStream);
//   pBitStream->writeString(mBaseNodeName);
//   pBitStream->writeString(mParentNodeName);
//   pBitStream->writeString(mChildNodeName);
//   pBitStream->writeString(mMeshObject);
//   pBitStream->writeRangedunsigned int(mBodypartChain, 0, MAX_FLEX_CHAINS-1);
//
//   pBitStream->write(mDynamicFriction);
//   pBitStream->write(mStaticFriction);
//   pBitStream->write(mRestitution);
//   pBitStream->write(mDensity);
//   pBitStream->write(mDamageMultiplier);
//   pBitStream->write(mForceMultiplier);
//   pBitStream->write(mForwardForce);
//   pBitStream->writeRangedunsigned int(mShapeType, 0, NUM_SHAPE_TYPES-1);
//
//   pBitStream->write(mNumParentVerts);
//   pBitStream->write(mNumChildVerts);
//   pBitStream->write(mNumFarVerts);
//   pBitStream->write(mWeightThreshold);
//   pBitStream->write(mRagdollThreshold);
//
//   mathWrite(*pBitStream, mOffset);
//   mathWrite(*pBitStream, mOrientation);
//   mathWrite(*pBitStream, mDimensions);
//
//   pBitStream->writeRangedunsigned int(mTriggerShapeType, 0, NUM_SHAPE_TYPES-1);
//   mathWrite(*pBitStream, mTriggerOffset);
//   mathWrite(*pBitStream, mTriggerOrientation);
//   mathWrite(*pBitStream, mTriggerDimensions);
//   mathWrite(*pBitStream, mTriggerActorOffset);
//
//   mathWrite(*pBitStream, mBoundsMin);
//   mathWrite(*pBitStream, mBoundsMax);
//   mathWrite(*pBitStream, mForceMin);
//   mathWrite(*pBitStream, mForceMax);
//   mathWrite(*pBitStream, mTorqueMin);
//   mathWrite(*pBitStream, mTorqueMax);
//
//
//   pBitStream->write(mIsKinematic);
//   pBitStream->write(mIsInflictor);
//   pBitStream->write(mInflictMultiplier);
//
//   if( pBitStream->writeFlag( mFlexBodyData != NULL ) )
//   {
//      pBitStream->writeRangedunsigned int(packed? SimObjectId(mFlexBodyData):
//                             mFlexBodyData->getId(),DataBlockObjectIdFirst,DataBlockObjectIdLast);
//   }
//   if( pBitStream->writeFlag( mPlayerData != NULL ) )
//   {
//      pBitStream->writeRangedunsigned int(packed? SimObjectId(mPlayerData):
//                             mPlayerData->getId(),DataBlockObjectIdFirst,DataBlockObjectIdLast);
//   }
//   if( pBitStream->writeFlag( mJointData != NULL ) )
//   {
//      pBitStream->writeRangedunsigned int(packed? SimObjectId(mJointData):
//                             mJointData->getId(),DataBlockObjectIdFirst,DataBlockObjectIdLast);
//   }
//}
//
//
//void fxFlexBodyPartData::unpackData(BitStream* pBitStream)
//{
//   Parent::unpackData(pBitStream);
//
//   mBaseNodeName = pBitStream->readSTString();
//   mParentNodeName = pBitStream->readSTString();
//   mChildNodeName = pBitStream->readSTString();
//   mMeshObject = pBitStream->readSTString();
//   mBodypartChain = (physChainType)pBitStream->readRangedunsigned int(0, MAX_FLEX_CHAINS-1);
//
//   pBitStream->read(&mDynamicFriction);
//   pBitStream->read(&mStaticFriction);
//   pBitStream->read(&mRestitution);
//   pBitStream->read(&mDensity);
//   pBitStream->read(&mDamageMultiplier);
//   pBitStream->read(&mForceMultiplier);
//   pBitStream->read(&mForwardForce);
//   mShapeType = (physShapeType)pBitStream->readRangedunsigned int(0, NUM_SHAPE_TYPES-1);
//
//   pBitStream->read(&mNumParentVerts);
//   pBitStream->read(&mNumChildVerts);
//   pBitStream->read(&mNumFarVerts);
//   pBitStream->read(&mWeightThreshold);
//   pBitStream->read(&mRagdollThreshold);
//
//   mathRead(*pBitStream, &mOffset);
//   mathRead(*pBitStream, &mOrientation);
//   mathRead(*pBitStream, &mDimensions);
//
//   mTriggerShapeType = (physShapeType)pBitStream->readRangedunsigned int(0, NUM_SHAPE_TYPES-1);
//   mathRead(*pBitStream, &mTriggerOffset);
//   mathRead(*pBitStream, &mTriggerOrientation);
//   mathRead(*pBitStream, &mTriggerDimensions);
//   mathRead(*pBitStream, &mTriggerActorOffset);
//
//   mathRead(*pBitStream, &mBoundsMin);
//   mathRead(*pBitStream, &mBoundsMax);
//   mathRead(*pBitStream, &mForceMin);
//   mathRead(*pBitStream, &mForceMax);
//   mathRead(*pBitStream, &mTorqueMin);
//   mathRead(*pBitStream, &mTorqueMax);
//
//   pBitStream->read(&mIsKinematic);
//
//   pBitStream->read(&mIsInflictor);
//   pBitStream->read(&mInflictMultiplier);
//
//   if( pBitStream->readFlag() )
//   {
//      FlexBodyDataID = pBitStream->readRangedunsigned int( DataBlockObjectIdFirst, DataBlockObjectIdLast );
//   }
//   if( pBitStream->readFlag() )
//   {
//      mPlayerDataID = pBitStream->readRangedunsigned int( DataBlockObjectIdFirst, DataBlockObjectIdLast );
//   }
//   if( pBitStream->readFlag() )
//   {
//      JointDataID = pBitStream->readRangedunsigned int( DataBlockObjectIdFirst, DataBlockObjectIdLast );
//   }
//}
//
