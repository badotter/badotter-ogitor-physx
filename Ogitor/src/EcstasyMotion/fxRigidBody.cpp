////////////////////////////////////////////////////////////////////////////////////////////////////
//  fxRigidBody.cc
//  Chris Calef
//
//  tge implementation of the physRigidBody
////////////////////////////////////////////////////////////////////////////////////////////////////
//#include "core/stl_fix.h"

#include "EcstasyMotion/nxPhysManager.h"
#include "EcstasyMotion/nxRigidBody.h"
#include "EcstasyMotion/fxRigidBody.h"

#include "SceneManagerEditor.h"
#include "OgreHardwareVertexBuffer.h"
#include "OgitorsScriptConsole.h"
extern Ogitors::OgitorsScriptConsole *gConsole;
extern SQLiteObject *gSQL;

#include "DTSShape.h"

fxRigidBody::fxRigidBody()
{


}

fxRigidBody::fxRigidBody(Ogre::Entity *entity,const char *meshFile)
{
	//Ogre::SceneNode *sceneRootNode = Ogitors::OgitorsRoot::getSingleton().GetSceneManager()->getRootSceneNode();

	//_ent->setVisible(true);

	//mSpring = NULL;
	//mParentBody = NULL;
	//mParentMountSlot = -1;

	//iPhysUser data
	//mFlexBody = NULL;
	mEntityType = PHYS_RIGID_BODY;
	mTempForce = Ogre::Vector3::ZERO;
	mTempPos = Ogre::Vector3::ZERO;
	mTempDamage = 0.0;
	mHasTempForce = false;
	mHasTractorBeam = false;
	mHasTractorSpring = false;
	mHasWeapon = false;
	mIsStriking = false;
	mIsInflictor = false;
	mInflictMultiplier = 0.0;
	mReset = false;
	mAutoClearKinematic = false;
	mIsMoving = false;

	mCurrPosition = Ogre::Vector3::ZERO;
	mCurrVelocity = Ogre::Vector3::ZERO;
	mCurrForce = Ogre::Vector3::ZERO;
	mCurrTorque = Ogre::Vector3::ZERO; 
	mLastPosition = Ogre::Vector3::ZERO;
	mDeltaPos = 0.0;

	mInitialPosition = Ogre::Vector3::ZERO;
	mInitialLinearVelocity = Ogre::Vector3::ZERO;
	mInitialAngularVelocity = Ogre::Vector3::ZERO;
	mInitialOrientation = Ogre::Quaternion::IDENTITY;
	mCurrAngularPosition = Ogre::Quaternion::IDENTITY;

	mWeaponPosAdj = Ogre::Vector3::ZERO;
	mWeaponRotAdjA = Ogre::Quaternion::IDENTITY;
	mWeaponRotAdjB = Ogre::Quaternion::IDENTITY;

	mCurrMS = 0;
	mEndMS = 0;
	mDelayMS = 1000;
	mCurrTick = 0;
	mLifetimeMS = 0;
	mSleepThreshold = 0.005;

	mMeshBody = -1;
	mStartMesh = -1;
	mNumMeshes = 0;

	mIsClientOnly = false;
	mIsKinematic = false;
	mIsNoGravity = false;
	mHasTrigger = false;
	mHadCollision = false;

	//mConvex.init(this);
	//mWorkingQueryBox.minExtents.set(-1e9, -1e9, -1e9);
	//mWorkingQueryBox.maxExtents.set(-1e9, -1e9, -1e9);

	mDebugTextureName.clear();
	//mDebugTextureHandle.clear();
	//mDebugVerts = 0;
	//mDebugPrimitives = 0;

	mReferenceNumber = -1;

	mEntityType = PHYS_RIGID_BODY;
	
	//NOW: load our info out of the database, instead of from a datablock!
	//if (!sqlLoad())
	//{
	//	gConsole->addOutput("ERROR, fxRigidBody failed sqlLoad()!");
	//	return;
	//}

	//////////////////////////////////////////

	mPM = physManagerCommon::getPM();

	mEntity = entity;
	mNode = mEntity->getParentSceneNode();
	mShapeName = meshFile;

	mRB = mPM->createRigidBody();
	mRB->setPhysUser((iPhysUser *)this);
	mRB->setLinearPosition(mNode->getPosition());

	if (!loadSQL())
		return;


	setupMeshBody();
	setupMesh();
	setupDebugRender();
	examineEntity();

	//Now, finally set up the actual physics object, if everything is loaded
	mPM->stopPhysics();
	mRB->setup();
	mPM->startPhysics();


	//if (mEntity->getName().find("highlanderhouse")==0)
	//	kNxRB->setKinematic(1);
}

fxRigidBody::~fxRigidBody()
{
	gConsole->addOutput("deleting fxRigidBody");
}

bool fxRigidBody::loadSQL()
{
	std::ostringstream strResult;

	//physRigidBodyCommon *kRB = dynamic_cast<physRigidBodyCommon*>(mRB);
	if (!mRB)
		return false;

	if (gSQL)
	{
		if (gSQL->OpenDatabase("EcstasyMotion.db"))
		{
			std::ostringstream queryRigid;
			sqlite_resultset *resultSet; 
			//strQuery = std::string("");
			queryRigid << "SELECT id, lifetime, sleepThreshold, shapeType, damageMultiplier, " << 
								  "isInflictor, isKinematic, isNoGravity, dimensions_x, dimensions_y, " <<
								  "dimensions_z, orientation_x, orientation_y, orientation_z," <<
								  "offset_x, offset_y, offset_z, scale_x, scale_y, scale_z " <<
								  "FROM fxRigidBodyData WHERE shapeFile = '" << mShapeName.c_str() << "';";

			int result = gSQL->ExecuteSQL(queryRigid.str().c_str());
			if (result)
			{
				resultSet = gSQL->GetResultSet(result);
				if (resultSet<=0)
				{
					strResult << "database opened, resultset null!" ;
				} else {
					if (resultSet->iNumRows == 0)	
					{
						strResult << "database opened, query empty!" ;
					} else {
						if (resultSet->iNumRows > 1)
						{
							strResult << "query returned multiple matches for shapeFile: " <<
								mShapeName.c_str() ;
						} else {
							//Set up the nx or ode rigid body, as a RigidBodyCommon, from database.
							mRigidBodyDataID = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);
							mLifetimeMS = strtol(resultSet->vRows[0]->vColumnValues[1],NULL,10);
							mSleepThreshold = strtod(resultSet->vRows[0]->vColumnValues[2],NULL);
							mRB->setShapeType( (physShapeType)strtol(resultSet->vRows[0]->vColumnValues[3],NULL,10));
							mDamageMultiplier = strtol(resultSet->vRows[0]->vColumnValues[4],NULL,10);
							mRB->setInflictor((bool)strtol(resultSet->vRows[0]->vColumnValues[5],NULL,10));
							mIsKinematic = (bool)strtol(resultSet->vRows[0]->vColumnValues[6],NULL,10);
							mRB->setKinematic(mIsKinematic);//REDUNDANT??
							mIsNoGravity = (bool)strtol(resultSet->vRows[0]->vColumnValues[7],NULL,10);
							mRB->setNoGravity(mIsNoGravity);//REDUNDANT??
							mRB->setDimensions( Ogre::Vector3(
								strtod(resultSet->vRows[0]->vColumnValues[8],NULL),
								strtod(resultSet->vRows[0]->vColumnValues[9],NULL),
								strtod(resultSet->vRows[0]->vColumnValues[10],NULL)) );
							mRB->setOrientation( Ogre::Vector3(
								strtod(resultSet->vRows[0]->vColumnValues[11],NULL),
								strtod(resultSet->vRows[0]->vColumnValues[12],NULL),
								strtod(resultSet->vRows[0]->vColumnValues[13],NULL)) );
							mRB->setOffset( Ogre::Vector3(
								strtod(resultSet->vRows[0]->vColumnValues[14],NULL),
								strtod(resultSet->vRows[0]->vColumnValues[15],NULL),
								strtod(resultSet->vRows[0]->vColumnValues[16],NULL)) );

							Ogre::Vector3 kScale( 
								strtod(resultSet->vRows[0]->vColumnValues[17],NULL),			
								strtod(resultSet->vRows[0]->vColumnValues[18],NULL),
								strtod(resultSet->vRows[0]->vColumnValues[19],NULL));
							if (kScale.length() > 0.0)
								mScale = kScale;
							else
								mScale = Ogre::Vector3(1,1,1);
							mRB->setScale(kScale);
							if (mNode->getScale().length()==Ogre::Vector3(1,1,1).length())
								mNode->scale(mScale);

							gSQL->CloseDatabase();
							return true;
						}
					}
				}
			} else {
				strResult << "database opened, resultset null!" ;
			}
			gSQL->CloseDatabase();
		} else {
			strResult << "database failed!" ;
		}
	}
	gConsole->addOutput(strResult.str());
	return false;
}


//bool fxRigidBody::onAdd()
//{
	//if (mIsClientOnly) mNetFlags.set(IsGhost);
	//else {
	//	mNetFlags.set(Ghostable);
	//	setScopeAlways();
	//}

	//if(!Parent::onAdd())
	//	return false;


	////Q: Should any (all?) of this be in onNewDatablock()?
	//if (mDataBlock->mIsInflictor)
	//	mIsInflictor = mDataBlock->mIsInflictor;
	//if (mDataBlock->mInflictMultiplier)
	//	mInflictMultiplier = mDataBlock->mInflictMultiplier;

	//if (mDataBlock->mWeaponPosAdj.len())
	//	mWeaponPosAdj = mDataBlock->mWeaponPosAdj;
	//if (mDataBlock->mWeaponRotAdjA.len())
	//	mWeaponRotAdjA.set(Ogre::Vector3(
	//	mDataBlock->mWeaponRotAdjA.x,
	//	mDataBlock->mWeaponRotAdjA.y,
	//	mDataBlock->mWeaponRotAdjA.z));
	//if (mDataBlock->mWeaponRotAdjB.len())
	//	mWeaponRotAdjB.set(Ogre::Vector3(
	//	mDataBlock->mWeaponRotAdjB.x,
	//	mDataBlock->mWeaponRotAdjB.y,
	//	mDataBlock->mWeaponRotAdjB.z));

	//if (isClientObject())
	//{

	//	if (mDataBlock->mSleepThreshold)
	//		mSleepThreshold = mDataBlock->mSleepThreshold;

	//	mCurrPosition = getTransform().getPosition();
	//	//Con::printf("creating a rigid body at:  %f %f %f",mCurrPosition.x,mCurrPosition.y,mCurrPosition.z);

	//	mLifetimeMS = mDataBlock->mLifetimeMS;
	//	//need to be able to change lifetime on the fly, so no longer relying
	//	//on the datablock property.
	//
	//	
	//	const String myFileName = mShapeInstance->mShapeResource.getPath().getFileName();
	//	mShapeName = StringTable->insert(myFileName.c_str());// don't have to remove .dts anymore in 1.8
	


	//mPM = physManagerCommon::getPM();
	//mRB = mPM->createRigidBody();
	//mRB->setPhysUser((iPhysUser *)this);

	//if ((mDataBlock->mShapeType==PHYS_SHAPE_CONVEX)||(mDataBlock->mShapeType==PHYS_SHAPE_COLLISION))
	//	setupMeshBody();

	//setupRigidBody();


	//if (mDataBlock->mHasSpring) mSpring = mPM->createSpring();
	//if (mSpring) setupSpring();//Hmm, not sure if this will be useful or not, might be.

	//FIX: we do need a setup() on unpackUpdate, or somewhere that makes 
	//sure the client sets up an object as well as the server.  LATER.

	////Ogre::Vector3 kScale(mObjScale.x,mObjScale.y,mObjScale.z);
	//setScale(mObjScale);
	////}// else {

	//mObjBox = mShapeInstance->mShape->bounds;
	//resetWorldBox();
	//setRenderTransform(mObjToWorld);

	//mObjBox.getCenter(&mConvex.mCenter);
	//mConvex.mSize.x = mObjBox.len_x() / 2.0;
	//mConvex.mSize.y = mObjBox.len_y() / 2.0;
	//mConvex.mSize.z = mObjBox.len_z() / 2.0;
	//mWorkingQueryBox.minExtents.set(-1e9, -1e9, -1e9);
	//mWorkingQueryBox.maxExtents.set(-1e9, -1e9, -1e9);


	
	//for (unsigned int i = 0; i < mShapeInstance->mShape->details.size(); i++)
	//{
	//	if (!mShapeInstance->mShape->details[i].nameIndex)
	//		 continue;//Con::errorf("found a name index: %d",mShapeInstance->mShape->details[i].nameIndex);


	//	String name = mShapeInstance->mShape->getName(mShapeInstance->mShape->details[i].nameIndex);

	//	if (dStrstr(name.c_str(), "collision-"))
	//	{
	//		mCollisionDetails.push_back(i);

	//		// The way LOS works is that it will check to see if there is a LOS detail that matches
	//		// the the collision detail + 1 + MaxCollisionShapes (this variable name should change in
	//		// the future). If it can't find a matching LOS it will simply use the collision instead.
	//		// We check for any "unmatched" LOS's further down
	//		mLOSDetails.increment();

	//		char buff[128];
	//		dSprintf(buff, sizeof(buff), "LOS-%d", i + 1 + 8);
	//		unsigned int los = mShapeInstance->mShape->findDetail(buff);
	//		if (los == -1)
	//			mLOSDetails.last() = i;
	//		else
	//			mLOSDetails.last() = los;
	//	}
	//	//}


	//	// Snag any "unmatched" LOS details
	//	for (unsigned int i = 0; i < mShapeInstance->mShape->details.size(); i++)
	//	{
	//		if (mShapeInstance->mShape->details[i].nameIndex)
	//		{
	//			String name = mShapeInstance->mShape->getName(mShapeInstance->mShape->details[i].nameIndex);

	//			if (dStrstr(name.c_str(), "los-"))
	//			{
	//				// See if we already have this LOS
	//				bool found = false;
	//				for (unsigned int j = 0; j < mLOSDetails.size(); j++)
	//				{
	//					if (mLOSDetails[j] == i)
	//					{
	//						found = true;
	//						break;
	//					}
	//				}

	//				if (!found)
	//					mLOSDetails.push_back(i);
	//			}
	//		}
	//	}

	//	for (unsigned int i = 0; i < mCollisionDetails.size(); i++)
	//		mShapeInstance->getShape()->getAccelerator(mCollisionDetails[i]);
	//}

	//addToScene();
//
//	return true;
//}
//
//void fxRigidBody::onRemove()
//{
//
//	if (mRB) mPM->removeRigidBody(mRB);
//	// Parent classes' methods take care of everything else
//	scriptOnRemove();
//
//	removeFromScene();
//
//	Parent::onRemove();
//}

//bool fxRigidBody::onNewDataBlock(GameBaseData* pGameBaseData,bool reload)
//{
//	mDataBlock = dynamic_cast<fxRigidBodyData*>(pGameBaseData);
//	if (!mDataBlock || !Parent::onNewDataBlock(pGameBaseData,reload))
//	{
//		return false;
//	}
//
//	if (!mIsKinematic) mIsKinematic = mDataBlock->mIsKinematic;
//	if (!mIsNoGravity) mIsNoGravity = mDataBlock->mIsNoGravity;
//	if (!mHasTrigger) mHasTrigger = mDataBlock->mHasTrigger;
//
//
//	// Have parent class do the rest
//	scriptOnNewDataBlock();
//	return true;
//}

//
//void fxRigidBody::initPersistFields()
//{
//	Parent::initPersistFields();
//
//	addGroup("fxRigidBody");
//	//addField("InitialPosition",  TypeOgre::Vector3, 
//	//    Offset(mCurrPosition, fxRigidBody));
//	addField("InitialVelocity", TypeOgre::Vector3, 
//		Offset(mCurrVelocity, fxRigidBody));
//	addField("CurrentForce", TypeOgre::Vector3, 
//		Offset(mCurrForce, fxRigidBody));
//	addField("CurrentTorque", TypeOgre::Vector3, 
//		Offset(mCurrTorque, fxRigidBody));
//	addField("IsClientOnly",TypeBool,
//		Offset(mIsClientOnly, fxRigidBody));
//	addField("IsKinematic",TypeBool,
//		Offset(mIsKinematic, fxRigidBody));
//	addField("IsNoGravity",TypeBool,
//		Offset(mIsNoGravity, fxRigidBody));
//	addField("HasTrigger",TypeBool,
//		Offset(mHasTrigger, fxRigidBody));
//	addField("AutoClearKinematic",TypeBool,
//		Offset(mAutoClearKinematic, fxRigidBody));
//	addField("Lifetime",Typeint,
//		Offset(mLifetimeMS, fxRigidBody));
//	addField("HadCollision",TypeBool,
//		Offset(mHadCollision, fxRigidBody));
//	addField("ReferenceNumber",Typeint,
//		Offset(mReferenceNumber, fxRigidBody));
//	endGroup("fxRigidBody");
//}
//
//unsigned int fxRigidBody::packUpdate(NetConnection* pConnection, unsigned int uiMask, BitStream* pBitStream)
//{
//	unsigned int uiRetMask = Parent::packUpdate(pConnection, uiMask, pBitStream);
//
//	if (pBitStream->writeFlag(uiMask & fxRigidMoveMask))
//	{
//		pBitStream->writeAffineTransform(mObjToWorld);
//		mathWrite(*pBitStream, mCurrPosition);
//		mathWrite(*pBitStream, mCurrVelocity);
//	}
//	if (pBitStream->writeFlag(uiMask & fxRigidStateMask))
//	{
//		mathWrite(*pBitStream, mObjScale);
//		mathWrite(*pBitStream, mCurrForce);
//		mathWrite(*pBitStream, mCurrTorque);
//		mathWrite(*pBitStream, mWeaponPosAdj);
//		mathWrite(*pBitStream, mWeaponRotAdjA);
//		mathWrite(*pBitStream, mWeaponRotAdjB);
//		pBitStream->write(mIsKinematic);
//		pBitStream->write(mIsClientOnly);
//		pBitStream->write(mAutoClearKinematic);
//		pBitStream->write(mReferenceNumber);
//	}
//	return uiRetMask;
//}
//
//void fxRigidBody::unpackUpdate(NetConnection* pConnection, BitStream *pBitStream)
//{
//	Parent::unpackUpdate(pConnection, pBitStream);
//
//	if (pBitStream->readFlag())
//	{
//		Ogre::Matrix4 kTransform;
//		pBitStream->readAffineTransform(&kTransform);
//		setTransform(kTransform);
//		mathRead(*pBitStream, &mCurrPosition);
//		mathRead(*pBitStream, &mCurrVelocity);
//	}
//	if (pBitStream->readFlag())
//	{
//		Ogre::Vector3 kScale;
//		mathRead(*pBitStream, &kScale);
//		setScale(kScale);
//		mathRead(*pBitStream, &mCurrForce);
//		mathRead(*pBitStream, &mCurrTorque);
//		mathRead(*pBitStream, &mWeaponPosAdj);
//		mathRead(*pBitStream, &mWeaponRotAdjA);
//		mathRead(*pBitStream, &mWeaponRotAdjB);
//		pBitStream->read(&mIsKinematic);
//		pBitStream->read(&mIsClientOnly);
//		pBitStream->read(&mAutoClearKinematic);
//		pBitStream->read(&mReferenceNumber);
//	}
//}
//
//void fxRigidBody::onEditorEnable()
//{
//	Parent::onEditorEnable();
//
//	// Force an update so that the client will put objects back where the server thinks they are
//	setMaskBits(fxRigidStateMask);
//}
//
//void fxRigidBody::onEditorDisable()
//{
//	Parent::onEditorDisable();
//}
//
//void fxRigidBody::inspectPostApply()
//{
//	// Tell our parent that apply was pressed.
//	Parent::inspectPostApply();
//
//	// Update our scale and position using the new data from the editor.
//	setScale(getScale());
//	setTransform(mObjToWorld);
//
//	// Set state changed mask.
//	setMaskBits(fxRigidStateMask);
//}
//
//void fxRigidBody::processTick(const Move* pMove)
//{
//	Parent::processTick(pMove);//before update position?
//	if (isClientObject())
//	{
//		//if (((nxRigidBody *)mRB)->mActor) {
//		if (!mIsKinematic) {
//			updatePositionFromRB();
//			updateVelocityFromRB();
//			updateForcesToRB();
//			mCurrAngularPosition = mRB->getAngularPosition();
//		}
//	} else {
//		resetWorldBox();
//	}
//	//Parent::processTick(pMove);//after update position?
//
//	mCurrMS += TickMs;
//
//	if  (( mCurrMS > mLifetimeMS )&&( mLifetimeMS > 0 ))
//	{
//		onRemove();//in onRemove, maybe check for clients that came from this 
//		//object and delete them too.  For now, just let them time out on their own.
//	}
//}
//
//void fxRigidBody::interpolateTick(float delta)
//{
//	//Ogre::Vector3 kDelta = mRB->mCurrLinearPosition - mRB->mLastLinearPosition;
//	//Con::errorf("delta: %f %f %f",kDelta.x,kDelta.y,kDelta.z);
//	//kDelta *= (1.0 - delta);
//	//mRB->mLinearPosition = mRB->mCurrLinearPosition + kDelta;
//	//if (kDelta.len()>0.001) Con::errorf("delta: %f",kDelta.len());
//}
//
//void fxRigidBody::advanceTime(float fTimeDelta)
//{
//	Parent::advanceTime(fTimeDelta);
//}

//
//void fxRigidBody::setTransform(const Ogre::Matrix4& kTransformMatrix)
//{
//	Parent::setTransform(kTransformMatrix);
//
//	if (!isClientObject()) setMaskBits(fxRigidStateMask);
//
//}

//iPhysUser:
void fxRigidBody::onWorldStep()
{
	std::ostringstream os;
	
	//os << "rigid body ticking!  position " << mCurrPosition.x << ", " << mCurrPosition.y << ", " << mCurrPosition.z;
	//gConsole->addOutput("flexbodypart ticking!!!!");
	//if (!mDidTempleActors) {//CHEAT
	//	if (!dStrncmp(mShapeInstance->mShape->mSourceResource->name,"Temple",6)) 
	//        setupTempleActors();		
	//	mDidTempleActors = true;
	//	return;
	//}

	//if (isClientObject())
	//{
		//if (mCurrTick==1)
		//{
		//	mInitialPosition = mCurrPosition;
		//	//Con::printf("rigid body initial position: %f %f %f",mInitialPosition.x,mInitialPosition.y,mInitialPosition.z);
		//	//Ogre::Matrix4 m = getTransform();
		//	//Ogre::Quaternion q(m);
		//	//mInitialOrientation = q;
		//	//mCurrAngularPosition = q;
		//}
		
		//if ((mInitialPosition.len() == 0.0)&&(mCurrPosition.len() != 0.0))
		//{//have to put this here instead of onAdd, because at least when you use Mission Editor to  
		////add bodies, onAdd gets called before dropSelection, which gives us our initial position.
		//mInitialPosition = mCurrPosition;
		//} else {//(This is to make us wait another tick for orientation to catch up.)
		//Ogre::Matrix4 m = getTransform();
		//if ((mInitialOrientation.isIdentity())&&(!m.isIdentity()))
		//{         
		//Ogre::Quaternion q(m);
		//mInitialOrientation = q;
		//}
		//}

		//if (mIsStriking) 
		//{//HERE: check if velocity.length() < threshold, then turn off mIsStriking so we don't kill anybody.
		//	if (mRB->getLinearVelocity().len() < 1.5) 
		//		mIsStriking = false;
		//}

		//if (mHasTractorBeam)
		//{
		//	float p;  Ogre::Matrix4 camTransform;
		//	Ogre::Vector3 myPos;
		//	SimGroup *g = Sim::getClientGroup();
		//	for (SimGroup::iterator itr = g->begin(); itr != g->end(); itr++) 
		//	{//really, should verify that there's only one client, or this is going to get all f**ked up.
		//		GameConnection *con = (GameConnection *) (*itr);
		//		ShapeBase *obj = dynamic_cast<ShapeBase *>(con->getControlObject());
		//		myPos = obj->getPosition();
		//		obj->getEyeTransform(&camTransform);
		//		//virtual void getCameraTransform(float* pos,Ogre::Matrix4* mat);
		//	}
		//	Ogre::Vector3 newPos; Ogre::Matrix4 newCamTrans;
		//	newCamTrans = camTransform;
		//	newCamTrans.makeTrans(Ogre::Vector3(0,0,0));
		//	newCamTrans.mulP(mTempPos,&newPos);
		//	newPos += myPos;
		//	mCurrPosition = newPos;
		//	//mCurrForce = newPos - mRB->getLinearPosition();
		//	Ogre::Vector3 actualPos = mRB->getLinearPosition();
		//	//mCurrForce *= 10000.0;
		//	mRB->setLinearPosition(newPos);
		//	mRB->updatePositionToActor();  //if non-kinematic  
		//	mRB->updateVelocityToActor();
		//	//Con::errorf("actual pos: %3.2f %3.2f %3.2f, desired pos: %3.2f %3.2f %3.2f, force %3.2f %3.2f %3.2f",actualPos.x,actualPos.y,actualPos.z,newPos.x,newPos.y,newPos.z,mCurrForce.x/10000,mCurrForce.y/10000,mCurrForce.z/10000);
		//	mLastPosition = mCurrPosition;
		//	mCurrPosition = mRB->getLinearPosition();
		//	mDeltaPos = mCurrPosition.len() - mLastPosition.len();
		//	setPosition(mCurrPosition);
		//	//NetObject *kServ = getServerObject();
		//	//dynamic_cast<fxFlexBody*>(kServ)->setPosition(mCurrPosition);
		//	//dynamic_cast<fxFlexBody*>(mServerObject)->

		//	if ((mDeltaPos>0.005)&&(!mIsMoving)) 
		//	{
		//		//Con::executef(this,"onMoving");
		//		mIsMoving = true;
		//	}
		//	else if ((mDeltaPos<0.005)&&(mIsMoving)) 
		//	{
		//		//Con::printf("currTick %d, deltaPos %f, isMoving %d",mCurrTick,mDeltaPos,mIsMoving);
		//		//if (mCurrTick>1) Con::executef(this,"onStop");
		//		mIsMoving = false;
		//	}
		//	if (mouseValue==0) {
		//		mHasTractorBeam = false;
		//		mIsKinematic = false;
		//		mRB->setKinematic(false);
		//		mIsNoGravity = false;
		//		mRB->setNoGravity(false);
		//		mTempPos = Ogre::Vector3::ZERO;
		//		mTempForce = Ogre::Vector3::ZERO;
		//		//Con::printf("mouse value = 0, clearing kinematic.");
		//	}
		//	mCurrTick++;
		//	return;
		//}

		//if (mHasTractorSpring)
		//{//HERE:  Throw this all away.  Replace with a real spring.  NxSpringAndDamper.  Have to create
		//	//it during a pause in the sim, so in onWorldStep, meaning have to flag the need for one when
		//	//the mouse event happens and then pick it up next tick.
		//	float p;  Ogre::Matrix4 camTransform;
		//	Ogre::Vector3 myPos;
		//	SimGroup *g = Sim::getClientGroup();
		//	for (SimGroup::iterator itr = g->begin(); itr != g->end(); itr++) 
		//	{//really, should verify that there's only one client, or this is going to get all f**ked up.
		//		GameConnection *con = (GameConnection *) (*itr);
		//		ShapeBase *obj = dynamic_cast<ShapeBase *>(con->getControlObject());
		//		myPos = obj->getPosition();
		//		obj->getEyeTransform(&camTransform);
		//		//virtual void getCameraTransform(float* pos,Ogre::Matrix4* mat);
		//	}
		//	Ogre::Vector3 newPos; Ogre::Matrix4 newCamTrans;
		//	newCamTrans = camTransform;
		//	newCamTrans.makeTrans(Ogre::Vector3(0,0,0));
		//	newCamTrans.mulP(mTempPos,&newPos);
		//	newPos += myPos;
		//	mCurrForce = newPos - mRB->getLinearPosition();

		//	if (mCurrForce.len()<1.0) 
		//	{//first, if we're almost there, just stop
		//		mRB->setLinearVelocity(Ogre::Vector3(0,0,0));
		//		mRB->setAngularVelocity(Ogre::Vector3(0,0,0));
		//		mRB->updateVelocityToActor();
		//	} 
		//	mCurrForce *= 90.0;
		//	if (mCurrForce.len()>20.0)
		//	{
		//		mCurrForce.normalize();
		//		mCurrForce *= 20.0;
		//	}

		//	if (mouseValue==0) 
		//	{
		//		mHasTractorSpring = false;
		//		mIsNoGravity = false;
		//		mRB->setNoGravity(false);
		//		mTempPos = Ogre::Vector3::ZERO;
		//		mTempForce = Ogre::Vector3::ZERO;
		//		mHasTempForce = true;
		//		mTempForce =  mRB->getLinearPosition() - myPos;
		//		mTempForce.normalize();
		//		mTempForce *= 10000000.0;
		//		mTempPos = mRB->getLinearPosition();
		//		Con::errorf("mass center: %f %f %f",mTempPos.x,mTempPos.y,mTempPos.z);
		//		//leave mTempPos at zero, apply force at center of object
		//		//(for objects with origin not at center of mass, we'd need to use
		//		//a novodex getCenterOfMass function)
		//	}
		//	mCurrTick++;
		//	return;
		//}

		if (mIsKinematic) {
		//	if ((mParentBody)&&(mParentMountSlot>-1))
		//		getPositionFromParent();
		//	else {
		//		mCurrPosition = getTransform().getPosition();
		//		mRB->setLinearPosition(mCurrPosition); 
		//		Ogre::Matrix4 M = getTransform();
		//		Ogre::Quaternion q(M);
		//		//Con::errorf("rigid body quat: %3.2f %3.2f %3.2f %3.2f",q.x,q.y,q.z,q.w);
		//		mRB->setAngularPosition(q);
		//	}

			Ogre::Quaternion q = mNode->getOrientation();
			mRB->setAngularPosition(q.Inverse());
			mRB->updatePositionToActor();   
			mRB->updateVelocityToActor();

			//os << "We have a Kinematic RigidBody: Y " << mRB->getLinearPosition().y ;
			//Ogre::String msg1(os.str());
			//gConsole->addOutput(msg1);

		} else {
			mRB->updatePositionFromActor();   
			mRB->updateVelocityFromActor();
			mLastPosition = mCurrPosition;
			mCurrPosition = mRB->getLinearPosition();

			mNode->setPosition(Ogre::Vector3(mCurrPosition.x,mCurrPosition.y,mCurrPosition.z));
			Ogre::Quaternion q = mRB->getAngularPosition();
			mNode->setOrientation(q.w,q.x,q.y,q.z);
			//os << "We have a Non Kinematic RigidBody:  " << mEntityType << ", " << mEntity->getName() ;
			//gConsole->addOutput(os.str());
			//mDeltaPos = mCurrPosition.len() - mLastPosition.len();

		//	if ((mDeltaPos>0.005)&&(!mIsMoving)) 
		//	{
		//		Con::executef(this,"onMoving");
		//		mIsMoving = true;
		//	}
		//	else if ((mDeltaPos<0.005)&&(mIsMoving)) 
		//	{
		//		//Con::printf("currTick %d, deltaPos %f, isMoving %d",mCurrTick,mDeltaPos,mIsMoving);
		//		if (mCurrTick>1) Con::executef(this,"onStop");
		//		mIsMoving = false;
		//	}

		}

		//if (mHasTempForce)
		//{
		//	addForceAtPos(mTempForce,mTempPos);
		//	mHasTempForce = false;
		//}

		//if (mReset)
		//{
		//	if (mInitialPosition.len())
		//	{
		//		mCurrPosition = mInitialPosition;
		//		Ogre::Matrix3 m;
		//		mInitialOrientation.ToRotationMatrix(&m);
		//		Ogre::Quaternion q(m);
		//		m.makeTrans(mCurrPosition);
		//		setTransform(m);
		//		mRB->setLinearPosition(mCurrPosition);
		//		mRB->setAngularPosition(q);
		//		mRB->setLinearVelocity(Ogre::Vector3(0,0,0));
		//		mRB->setAngularVelocity(Ogre::Vector3(0,0,0));
		//		mRB->updatePositionToActor();   
		//		mRB->updateVelocityToActor();
		//	}
		//	mReset = false;
		//}

		//if (mInitialPosition.len())
		//{
		//   mCurrPosition = mInitialPosition;
		//   Ogre::Matrix4 m;
		//   mInitialOrientation.ToRotationMatrix(&m);
		//   m.makeTrans(mCurrPosition);
		//   setTransform(m);
		//setKinematic();
		//}
		//if (mDeltaPos<mSleepThreshold) setKinematic();
		//enableCollision();//ha ha, worth a try anyway

		//mDeltaPos = 0.0;
	//}
	mCurrTick++;

}

void fxRigidBody::onCollision(iPhysUser *other,int action=0)
{
	//if (!dStrcmp(mShapeInstance->mShape->mSourceResource->name,"pendulumTargetBob.dts")||
	//   !dStrcmp(mShapeInstance->mShape->mSourceResource->name,"dropTargetBlock.dts")||
	//   !dStrcmp(mShapeInstance->mShape->mSourceResource->name,"barrelRollBlock.dts")||
	//   !dStrcmp(mShapeInstance->mShape->mSourceResource->name,"teeterTotterTargetLever.dts"))

	//Con::executef(this,"onCollision");

	//if (other->mEntityType == PHYS_FLEX_BODY_PART)
	//{//add an impulse force in direction of collision vector

	//}

	if (mAutoClearKinematic)
		clearKinematic();
}

void fxRigidBody::onTrigger(iPhysUser *other,int action=0,int id=0)
{
	if (action==0)
	{//projectile trigger, only do this in special cases - you want most things to stay kinematic.
		
		if (mAutoClearKinematic)      
			clearKinematic();
	} 
	else if (action==1) 
	{//fluid trigger, apply force based on velocity vector
		//if (dynamic_cast<fxFluid*>(other)->mFluid) {
		//	Ogre::Vector3 vel = dynamic_cast<fxFluid*>(other)->mFluid->getParticleVelocity(id);
		//	Con::errorf("particle velocity: %3.2f %3.2f %3.2f",vel.x,vel.y,vel.z);
		//	vel *= 30000.0;
		//	mRB->setGlobalForce(vel);
		//}
	} else if (action == 2) {
		//mRB->setGlobalForce(Ogre::Vector3(0,0,0));
		//mRB->setLinearVelocity(Ogre::Vector3(0,0,0));
		//if (mIsKinematic==false)
		//{
		//	Con::executef(this,"onGoal");
		//	//setKinematic();
		//}

		//dynamic_cast<fxRigidBody*>(other)->setKinematic();
	}
}


void fxRigidBody::addForceAtPos(const Ogre::Vector3 &kForce,const Ogre::Vector3 &kPos)
{
	if (mRB)
		mRB->addForceAtPos(kForce,kPos);
}


void fxRigidBody::lockTractorBeam(int type)
{
	//DEMO: Don't let buildings ("Temples") get dragged out of the ground
	//if (!dStrncmp(mShapeInstance->mShape->mSourceResource->name,"STATIC",6)) return;
	if (mIsKinematic) return;//TEMP: for ecstasy first pass, only kinematic rigid body is the ground plane. Don't
	//let it get dragged away or everything will fall forever. :-(

	if (type==0) {
		mHasTractorBeam = true;
		mIsKinematic = true;
		mRB->setKinematic(true);
	} else if (type==1) {
		mHasTractorSpring = true;
		mIsNoGravity = true;
		if (mRB)
			mRB->setNoGravity(true);
	}
}


//void fxRigidBody::renderDebug()
//{
	
	//Parent::renderObject(pState, ri);

	//if ((mPM->getDebugRender())&&(mStartMesh>=0))
	//	//	  ((mDataBlock->mShapeType==PHYS_SHAPE_CONVEX)||(mDataBlock->mShapeType==PHYS_SHAPE_COLLISION)))
	//{

	//	PROFILE_START(fxRigidBody_debugRender);
	//	Ogre::Vector3 p,d;
	//	unsigned int numVerts,sumVerts,index,i,j;
	//	ColorF kColor(0.0,1.0,0.0);
	//	NxVec3 kVerts[10000],tempVerts[10000];
	//	index = 0; sumVerts = 0;

	//	for (i=0;i<mNumMeshes;i++)
	//	{
	//		if (mMeshDescs[i].isValid()) {
	//			numVerts = mMeshDescs[i].numVertices;

	//			dMemcpy(tempVerts,mMeshDescs[i].points, numVerts * sizeof(NxVec3));
	//			for (j=sumVerts;j<(sumVerts+numVerts);j++)
	//			{
	//				kVerts[j] =  tempVerts[j-sumVerts];//maybe?
	//			}
	//			sumVerts += numVerts;
	//			dMemset(tempVerts,0,10000);//just in case
	//		}
	//	}

	//	//Hmmm... could skip all this, and just do it once in the beginning, because all that
	//	//changes is mObjToWorld, below.  Would need two mDebugVerts-type arrays, one for permanent
	//	//(model-space) positions, one for temporary (world-space) positions

	//	//dMemcpy(kVerts,mMeshDescs[i].points, numVerts * sizeof(NxVec3));//myDesc.pointStrideBytes
	//	//dMemcpy(kTris,myDesc.triangles,myDesc.numTriangles * myDesc.triangleStrideBytes);

	//	GFXVertexPNTT *verts = new GFXVertexPNTT[ sumVerts ];
	//	GFXVertexPNTT *vert;

	//	for (unsigned int i=0;i<sumVerts;i++)
	//	{
	//		Ogre::Vector3 p,d;
	//		p.set(kVerts[i].x,kVerts[i].y,kVerts[i].z);
	//		mObjToWorld.mulP(p,&d);
	//		vert = &verts[index++];
	//		vert->point.set(d.x,d.y,d.z);
	//		vert->texCoord.set(1.0,1.0);	
	//	}


	//	GFXVertexPNTT *vbVerts = mDebugVerts.lock();
	//	dMemcpy( vbVerts, verts, sizeof(GFXVertexPNTT) * sumVerts );
	//	mDebugVerts.unlock();

	//	GFX->setFillMode(GFXFillWireframe);

	//	GFX->setTexture(0,mDebugTextureHandle);
	//	GFX->setTextureStageColorOp(0, GFXTOPModulate);
	//	GFX->setTextureStageColorOp(1, GFXTOPDisable);

	//	GFX->setVertexBuffer( mDebugVerts );
	//	GFX->setPrimitiveBuffer( mDebugPrimitives );
	//	GFX->drawPrimitives();

	//	//if (mHasTrigger)
	//	if(0)
	//	{	
	//		numVerts = 8;

	//		GFXVertexPNTT *trigverts = new GFXVertexPNTT[ numVerts ];
	//		GFXVertexPNTT *vert;
	//		for (unsigned int i=0;i<numVerts;i++)
	//		{
	//			Ogre::Vector3 p,d;
	//			p.set(mTriggerPoints[i].x,mTriggerPoints[i].y,mTriggerPoints[i].z);
	//			mObjToWorld.mulP(p,&d);
	//			vert = &trigverts[index++];
	//			vert->point.set(d.x,d.y,d.z);
	//			vert->texCoord.set(1.0,1.0);	
	//		}

	//		GFXVertexPNTT *vbVerts = mTriggerVerts.lock();
	//		dMemcpy( vbVerts, trigverts, sizeof(GFXVertexPNTT) * numVerts );
	//		mTriggerVerts.unlock();


	//		//GFX->setTexture(0,mTriggerTextureHandle);
	//		//GFX->setTextureStageColorOp(0, GFXTOPModulate);
	//		//GFX->setTextureStageColorOp(1, GFXTOPDisable);

	//		//GFX->setVertexBuffer( mTriggerVerts );
	//		//GFX->setPrimitiveBuffer( mTriggerPrimitives );
	//		//GFX->drawPrimitives();

	//		//Ogre::Vector3 size = mDataBlock->mTriggerDimensions;
	//		//size.y = size.x;
	//		//Ogre::Vector3 offset,pos;
	//		//offset = mDataBlock->mTriggerOffset;
	//		//mObjToWorld.mulP(offset,&pos);
	//		//GFX->drawWireCube(size, pos, color);
	//		delete [] trigverts;
	//	}

	//	GFX->setFillMode(GFXFillSolid);

	//	delete [] verts;
	//	PROFILE_END();
	//}
	
	
//}

void fxRigidBody::setupDebugRender()
{  //Hmm - need a way to instance this, so we can copy the same object for every instance of this mesh...


	//nxRigidBody *kRB = dynamic_cast<nxRigidBody*>(mRB);


	//Ogre::ManualObject *mo = Ogitors::OgitorsRoot::getSingleton().GetSceneManager()->createManualObject();//Do we
	////need a different name every time?
	//mo->begin(mPM->getWireMat()->getName());
	////mo->setDynamic(true);

	//mo->estimateVertexCount(24);
	//mo->estimateIndexCount(36);


	//Ogre::Vector3 cubeverts[8] = 
	//{
	//	Ogre::Vector3(5, 5, 5),
	//	Ogre::Vector3(-5, 5, 5),
	//	Ogre::Vector3(-5, 5, -5),
	//	Ogre::Vector3(5, 5, -5),
	//	Ogre::Vector3(5, -5, 5), 
	//	Ogre::Vector3(-5, -5, 5),
	//	Ogre::Vector3(-5, -5, -5),
	//	Ogre::Vector3(5, -5, -5)
	//};

	//Ogre::Vector3 normUp(0,1,0);
	//Ogre::Vector3 normDown(0,-1,0);
	//Ogre::Vector3 normBack(0,0,1);
	//Ogre::Vector3 normFront(0,0,-1);
	//Ogre::Vector3 normRight(-1,0,0);
	//Ogre::Vector3 normLeft(1,0,0);

	//Ogre::Vector2 texUpLeft(0.0,0.0);
	//Ogre::Vector2 texUpRight(0.0,1.0);
	//Ogre::Vector2 texLowRight(1.0,1.0);
	//Ogre::Vector2 texLowLeft(1.0,0.0);

	//mo->position(cubeverts[0]); mo->normal(normUp); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texUpLeft);//0
	//mo->position(cubeverts[1]); mo->normal(normUp); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texUpRight);//1
	//mo->position(cubeverts[2]); mo->normal(normUp); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texLowRight);//2
	//mo->position(cubeverts[3]); mo->normal(normUp); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texLowLeft);//3

	//mo->position(cubeverts[4]); mo->normal(normBack); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texUpLeft);//4
	//mo->position(cubeverts[5]); mo->normal(normBack); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texUpRight);//5
	//mo->position(cubeverts[1]); mo->normal(normBack); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texLowRight);//6
	//mo->position(cubeverts[0]); mo->normal(normBack); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texLowLeft);//7

	//mo->position(cubeverts[5]); mo->normal(normRight); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texUpLeft);//8
	//mo->position(cubeverts[6]); mo->normal(normRight); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texUpRight);//9
	//mo->position(cubeverts[2]); mo->normal(normRight); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texLowRight);//10
	//mo->position(cubeverts[1]); mo->normal(normRight); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texLowLeft);//11

	//mo->position(cubeverts[6]); mo->normal(normFront); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texUpLeft);//12
	//mo->position(cubeverts[7]); mo->normal(normFront); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texUpRight);//13
	//mo->position(cubeverts[3]); mo->normal(normFront); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texLowRight);//14
	//mo->position(cubeverts[2]); mo->normal(normFront); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texLowLeft);//15

	//mo->position(cubeverts[7]); mo->normal(normLeft); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texUpLeft);//16
	//mo->position(cubeverts[4]); mo->normal(normLeft); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texUpRight);//17
	//mo->position(cubeverts[0]); mo->normal(normLeft); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texLowRight);//18
	//mo->position(cubeverts[3]); mo->normal(normLeft); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texLowLeft);//19

	//mo->position(cubeverts[5]); mo->normal(normDown); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texUpLeft);//20
	//mo->position(cubeverts[4]); mo->normal(normDown); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texUpRight);//21
	//mo->position(cubeverts[7]); mo->normal(normDown); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texLowRight);//22
	//mo->position(cubeverts[6]); mo->normal(normDown); mo->colour(Ogre::ColourValue::Green); mo->textureCoord(texLowLeft);//23

	//mo->triangle(0,3,1); mo->triangle(1,3,2);//mo->quad(0,3,2,1);//                                //TOP
	//mo->triangle(4,7,6); mo->triangle(5,4,6);//mo->quad(4,7,6,5);//                                //BACK
	//mo->triangle(9,11,10); mo->triangle(9,8,11);//mo->quad(8,11,10,9);//                           //RIGHT
	//mo->triangle(12,14,13); mo->triangle(12,15,14);//mo->quad(12,15,14,13);//                      //FRONT
	//mo->triangle(16,19,17); mo->triangle(17,19,18);//mo->quad(16,19,18,17);//                      //LEFT
	//mo->triangle(20,23,21); mo->triangle(21,23,22);//mo->quad(20,23,22,21);//                      //BOTTOM

	//mo->end();

	//mNode->attachObject(mo);
}




////FIX: if (type = PHYS_NX) {...}
//unsigned int i,j,numVerts,numTris,sumVerts,sumTris,numIndices;
//numVerts = 0; sumVerts = 0; numTris = 0; sumTris = 0; numIndices = 0;
//unsigned int kTris[10000];
//NxConvexMesh *myMesh;

//if (mStartMesh>=0)
	//{
	//	mDebugTextureHandle.set("scriptsAndAssets/data/textures/debug_rigidbody.jpg",&GFXDefaultPersistentProfile,"debug_rigidbody");//fix this, can't we do "~/" 
	//	mTriggerTextureHandle.set("scriptsAndAssets/data/textures/debug_trigger.jpg",&GFXDefaultPersistentProfile,"debug_trigger");//or something instead?

	//	//HERE: do a loop 0 - mNumMeshes
	//	for (i=0;i<mNumMeshes;i++)
	//	{
	//		myMesh = ((nxPhysManager *)mPM)->getMesh(mStartMesh+i);
	//		if (myMesh)
	//		{
	//			myMesh->saveToDesc(mMeshDescs[i]);
	//			sumVerts += mMeshDescs[i].numVertices;
	//			sumTris += mMeshDescs[i].numTriangles;
	//		}
	//	}


	//	mDebugVerts.set( GFX, sumVerts, GFXBufferTypeDynamic );
	//	U16 *indices = new U16[ sumTris * 3 ];
	//	numIndices = 0;  

	//	for (i=0;i<mNumMeshes;i++)
	//	{
	//		numTris = mMeshDescs[i].numTriangles;
	//		dMemcpy(kTris,mMeshDescs[i].triangles,numTris * mMeshDescs[i].triangleStrideBytes);

	//		//for (j=sumTris;j<(sumTris+numTris);j++)
	//		for (j=0;j<numTris;j++)
	//		{
	//			indices[numIndices++] = kTris[(j*3)+0];
	//			indices[numIndices++] = kTris[(j*3)+1];
	//			indices[numIndices++] = kTris[(j*3)+2];
	//		}
	//	}

	//	GFXPrimitive pInfo;
	//	U16 *ibIndices;
	//	GFXPrimitive *piInput;

	//	pInfo.type = GFXTriangleList;
	//	pInfo.numPrimitives = sumTris;
	//	pInfo.startIndex = 0;
	//	pInfo.minIndex = 0;
	//	pInfo.numVertices = sumVerts;

	//	mDebugPrimitives.set( GFX, numIndices, 1, GFXBufferTypeStatic );
	//	mDebugPrimitives.lock( &ibIndices, &piInput );
	//	dMemcpy( ibIndices, indices, numIndices * sizeof(U16) );
	//	dMemcpy( piInput, &pInfo, sizeof(GFXPrimitive) );
	//	mDebugPrimitives.unlock();

	//	delete [] indices;
	//}
	
//}






//
//void fxRigidBody::setupDebugRender()
//{
//unsigned int i,j,numVerts,numTris,numIndices;
//numVerts = 0; numTris = 0; numIndices = 0;
//unsigned int kTris[1000];
//
//if (mStartMesh>=0)
//{
//
//mDebugTextureHandle.set("terrain_water_demo/data/textures/debug_rigidbody.jpg",&GFXDefaultPersistentProfile);
//mTriggerTextureHandle.set("terrain_water_demo/data/textures/debug_trigger.jpg",&GFXDefaultPersistentProfile);
//
//
//NxConvexMesh *myMesh = ((nxPhysManager *)mPM)->getMesh(mStartMesh);
//if (myMesh) 
//{
//NxConvexMeshDesc myDesc;
//myMesh->saveToDesc(myDesc);
//numVerts = myDesc.numVertices;
//numTris = myDesc.numTriangles;
//numIndices = 0;
//
//U16 *indices = new U16[ numTris * 3 ];
//
//mDebugVerts.set( GFX, numVerts, GFXBufferTypeDynamic );
//dMemcpy(kTris,myDesc.triangles,numTris * myDesc.triangleStrideBytes);
//
//for (i=0;i<numTris;i++)
//{
//indices[numIndices++] = kTris[(i*3)+0];
//indices[numIndices++] = kTris[(i*3)+1];
//indices[numIndices++] = kTris[(i*3)+2];
//}
//
//GFXPrimitive pInfo;
//U16 *ibIndices;
//GFXPrimitive *piInput;
//
//pInfo.type = GFXTriangleList;
//pInfo.numPrimitives = numTris;
//pInfo.startIndex = 0;
//pInfo.minIndex = 0;
//pInfo.numVertices = numVerts;
//
////damn, ouch... crashes here
//mDebugPrimitives.set( GFX, numIndices, 1, GFXBufferTypeStatic );
//mDebugPrimitives.lock( &ibIndices, &piInput );
//dMemcpy( ibIndices, indices, numIndices * sizeof(U16) );
//dMemcpy( piInput, &pInfo, sizeof(GFXPrimitive) );
//mDebugPrimitives.unlock();
//
//delete [] indices;
//
//}	
//...
//}



//
//bool fxRigidBody::prepRenderImage(SceneState* state, const unsigned int stateKey,
//											 const unsigned int startZone, const bool modifyBaseState)
//{
//	RenderInst *ri;
//	if (mPM->getDebugRender()) 
//	{
//		//ri = gRenderInstManager.allocInst();
//		//ri->obj = this;
//		//ri->state = state;
//		//ri->type = RenderInstManager::RIT_NxDebug;
//		//gRenderInstManager.addInst( ri );
//		renderDebug();//timing is bad, this will render before object render
//	}
//
//	if(1)// && (dynamic_cast<ShapeBase*>(getDataBlock())->shadowEnable)
//	{
//		//ri = gRenderInstManager.allocInst();         
//		//ri->obj = this;
//		//ri->state = state;
//		//ri->type = RenderInstManager::RIT_Shadow;
//		//ri->transFlags = 2;
//		//gRenderInstManager.addInst(ri);
//	}
//
//	if (Parent::prepRenderImage(state,stateKey,startZone,modifyBaseState))
//		return true;
//	else return false;
//
//}

void fxRigidBody::updatePositionFromRB()
{
	//Ogre::Matrix4 kTransform;
	//mRB->setAngularPositionMatrix(kTransform);
	//kTransform.makeTrans(mRB->getLinearPosition());
	//Ogre::Vector3 diff = mCurrPosition - mRB->getLinearPosition();
	//if (diff.len() > 0.02) {
	//	setMaskBits(fxRigidStateMask);
	//}
	//mCurrPosition = mRB->getLinearPosition();
	//setTransform(kTransform);
}

void fxRigidBody::updateVelocityFromRB()
{//whoops, how do you set angular velocity in Torque again?
	//setVelocity(mRB->getLinearVelocity());
	//setAngularVelocity(mRB->mAngularVelocity);    
}

void fxRigidBody::updateForcesToRB()
{//whoops, how do you set angular velocity in Torque again?
	//mRB->setCurrForce(mCurrForce);
	//if (mRB)
	//{
	//	mRB->setCurrTorque(mCurrTorque);
	//	mRB->setGlobalForce(mCurrForce);
	//	mCurrForce = Ogre::Vector3::ZERO;
	//	mCurrTorque = Ogre::Vector3::ZERO;
	//	//setAngularVelocity(mRB->mAngularVelocity);    
	//}
}

void fxRigidBody::getPositionFromParent()
{
	//if (mRB)
	//{
		//T3D 1.1 beta 2 - getMountTransform now requires a mount transform as an argument??
		//Ogre::Matrix4 kTransform(true);
		//mParentBody->getMountTransform(mParentMountSlot,&kTransform);
		//setTransform(kTransform);
		//mRB->setLinearPosition(kTransform.getPosition());
		//Ogre::Quaternion q;
		//q.set(kTransform);
		//mRB->setAngularPosition(q);
		//Ogre::Vector3 curPos = kTransform.getPosition();
	//}
}

void fxRigidBody::setupMeshBody()
{
	std::ostringstream os;

	//HERE: Ogretize the shape name/path!
	//const String myPath = getShapeInstance()->mShapeResource.getPath().getPath();
	//const String myFileName = getShapeInstance()->mShapeResource.getPath().getFileName();
	Ogre::String myPath,myMeshName,fullPath;
	Ogre::MeshPtr meshPtr = mEntity->getMesh();
	myMeshName = meshPtr->getName();
	//Ogre::ResourceHandle resHandle = meshPtr->getHandle();
	//Ogre::Resource myResource = Ogre::ResourceGroupManager::getSingleton().getResourceLocationList(

	//HOly SH!T.... :-{  Hell with the filename, sorting by mesh names, you can only have one per name.
	//Ogre::ResourceGroupManager::ResourceGroup myGroup = Ogre::ResourceGroupManager::getSingleton().getResourceGroup("ProjectResources");
	//Ogre::MeshManager myMgr = Ogre::
	//Ogre::ResourcePtr myRsrc = myMgr.getByHandle(resHandle);
	//fullPath = myRsrc.getPointer()->getOrigin();
	//
	//os.str("");
	//os << " my resource origin: " << fullPath.c_str() ;
	//gConsole->addOutput(os.str());

	if (1)//(mPM->getType()==PHYS_NX) //HERE: only for PhysX, because we're storing a list of NxConvexMesh objects.
	{
		nxPhysManager *kPM = (nxPhysManager*)mPM;

		//NOTE: this will break if you try to use the same model twice and use different
		//SHAPE_TYPES, like convex for one and collision for the other.  This will grab
		//whatever was first.  Shouldn't happen often, if ever -- ignoring for now.
		for (unsigned int i=0;i<kPM->mNumMeshBodies;i++) 
		{
			if (!strcmp(kPM->mMeshBodies[i]->shapeName.c_str(),myMeshName.c_str())) 
			{ 
				mMeshBody = i;
				mStartMesh = kPM->mMeshBodies[i]->startMesh;
				mNumMeshes = kPM->mMeshBodies[i]->numMeshes;
				gConsole->addOutput("Found my mesh by name!");
			}
		}

		if (mStartMesh==-1)  
		{
			//HERE: count the collision meshes
			//if (mDataBlock->mShapeType==PHYS_SHAPE_COLLISION)
			//{
			//	const char *name;
			//	TSShape *kShape = mShapeInstance->mShape;
			//	for (unsigned int i = 0; i < kShape->details.size(); i++)
			//	{
			//		const TSDetail *detail = &kShape->details[i];
			//		name = kShape->names[kShape->details[i].nameIndex].c_str();
			//		if (!dStrnicmp(name,"collision-",10))
			//		{
			//			int ss = detail->subShapeNum;
			//			int od = detail->objectDetailNum;
			//			int start = kShape->subShapeFirstObject[ss];
			//			int end   = kShape->subShapeNumObjects[ss] + start;
			//			Con::errorf("found a collision mesh! %s subshape %d objdet %d start %d end %d",
			//				name,ss,od,start,end);
			//			if (start < end) 
			//			{
			//				for (unsigned int j = start; j < end; j++)   
			//				{
			//					TSShapeInstance::MeshObjectInstance * meshObject = &mShapeInstance->mMeshObjects[j];
			//					if (od >= meshObject->object->numMeshes) 
			//						continue;
			//					mNumMeshes++;
			//					Con::errorf("got a mesh!");
			//				}
			//			}
			//		}
			//	}
			//} else mNumMeshes = 1;
			//HERE: count all subentities/submeshes, not just collision.
			mNumMeshes = 1;

			kPM->mMeshBodies[kPM->mNumMeshBodies] = new physMeshBody;
			//HERE: Ogretize it!//kPM->mMeshBodies[kPM->mNumMeshBodies]->shapeName = StringTable->insert(fullpath);
			kPM->mMeshBodies[kPM->mNumMeshBodies]->shapeName = mEntity->getMesh()->getName();
			kPM->mMeshBodies[kPM->mNumMeshBodies]->startMesh = kPM->mNumMeshes;
			kPM->mMeshBodies[kPM->mNumMeshBodies]->numMeshes = mNumMeshes;
			mMeshBody = kPM->mNumMeshBodies;
			mStartMesh = kPM->mNumMeshes;
			kPM->mNumMeshes += mNumMeshes;
			kPM->mNumMeshBodies++;
		}
		mRB->setStartMesh(mStartMesh);
	}
}

void fxRigidBody::setupMesh()
{
	Ogre::Vector3 p;
	Ogre::Vector3 vertex,d;
	Ogre::Quaternion qF;
	Ogre::Matrix4 kTransform;
	
	Ogre::VertexDeclaration* decl;// = meshPtr->sharedVertexData->vertexDeclaration;
	const Ogre::VertexElement *elem;// = decl->findElementBySemantic(Ogre::VES_POSITION);
	Ogre::VertexData* vertex_data;// = meshPtr->sharedVertexData;

	int nodeIndex;

	std::ostringstream os;
	Ogre::String msg;
	//if (mPM->getType()==PHYS_NX) {
	nxPhysManager *kPM = dynamic_cast<nxPhysManager*>(mPM);

	//os << "mStartMesh: " << mStartMesh;
	//msg = os.str();
	//gConsole->addOutput(msg);
	if ((mStartMesh>-1)&&(kPM->mMeshes[mStartMesh])) 
	{
		mRB->setStartMesh(mStartMesh);
		gConsole->addOutput("found the mesh, it's been loaded!");
	} 
	else 
	{
		Ogre::MeshPtr meshPtr = mEntity->getMesh();
		int numBoneAssignments = 0;//meshPtr->getSubMesh(0)->getBoneAssignments().size();
		int numIndices = meshPtr->getSubMesh(0)->indexData->indexCount;
		bool sharedVertexData = meshPtr->getSubMesh(0)->useSharedVertices;
		if (sharedVertexData)
		{
			vertex_data = meshPtr->sharedVertexData;
		} else {
			//HERE: deal with multiple submeshes!
			vertex_data = meshPtr->getSubMesh(0)->vertexData;
		}

		int numVerts = vertex_data->vertexCount;
		decl = vertex_data->vertexDeclaration;
		elem = decl->findElementBySemantic(Ogre::VES_POSITION);

		Ogre::HardwareVertexBufferSharedPtr vbuf= vertex_data->vertexBufferBinding->getBuffer(elem->getSource());
		const unsigned int vSize = (unsigned int)vbuf->getVertexSize();
		unsigned char* vertex =static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
		physRigidBodyCommon *kRB = (physRigidBodyCommon*)mRB;

		float* pReal;
		for (int i=0;i<numVerts;i++)
		{
			elem->baseVertexPointerToElement(vertex, &pReal);
			Ogre::Vector3 vertex3 = Ogre::Vector3(pReal[0],pReal[1],pReal[2]);
			vertex3 *= mScale;

			//Whoops!  Can't do this now, have to rotate it when you make the actual rigid body 
			//instance!  All instances can be forced to have the same scale, but not the same rotation.
			//(Even scale would be better if we could do it individually.)
			//Ogre::Quaternion quat = mNode->getOrientation();
			//Ogre::Matrix3 mat;
			//quat.ToRotationMatrix(mat);
			//Ogre::Vector3 finalVert = mat * vertex3;

			//kRB->mVerts.increment();
			//kRB->mVerts[kRB->mVerts.size()-1]=vertex3;
			
			kRB->mVerts.push_back(vertex3);
			
			//if (kTransform) kTransform.mulP(vertex,&d);
			//else d = vertex;
			//kRB->mVerts.increment();//HERE: is this going to work? 
			//kRB->mVerts[kRB->mVerts.size()-1]=d;//vertex[MAYBE?]
			os.str("");
			os << "Added a RB Vert " << i << ": " << vertex3.x << " " << vertex3.y << " " << vertex3.z << ", size " << kRB->mVerts.size() ;
			gConsole->addOutput(os.str());
			vertex += vSize;
		}
		//TSShape *kShape = mShapeInstance->mShape;
		//const TSDetail *detail = &kShape->details[0];

		//HERE: Ogre LODS?
		//int ss = detail->subShapeNum;
		//int od = detail->objectDetailNum;

		//int vc = 0;//vertex count
		//int mc = 0;//mesh count

		////HERE: Ogre SubMeshes.
		////int start = kShape->subShapeFirstObject[ss];
		////int end   = kShape->subShapeNumObjects[ss] + start;

		//Ogre::Vector3 vertex3,d;
		//Ogre::Matrix4  kTransform;
		//Ogre::Matrix3  kMat3;

		//gConsole->addOutput("Starting entity/mesh research...");

		//int numSubEnts = mEntity->getNumSubEntities();
		//int numManualLODs = mEntity->getNumManualLodLevels();
		//Ogre::MeshPtr meshPtr = mEntity->getMesh();
		//int numAnims = meshPtr->getNumAnimations();
		//int numLODMeshLevels = meshPtr->getNumLodLevels();
		//int numSubMeshes = meshPtr->getNumSubMeshes();

		//os << "SubEntities " << numSubEnts << ", manualLODS " << numManualLODs << ", anims " <<
		//	numAnims << ", Mesh LODS " << numLODMeshLevels << ", SubMeshes " << numSubMeshes ;
		//msg = os.str();
		//gConsole->addOutput(msg);

		//os.str("");
		//os << "material: " << meshPtr->getSubMesh(0)->getMaterialName() ;
		//gConsole->addOutput(os.str());

		////Very good... making sense so far.  Don't worry about Entity->getNumManualLODs right now, 
		////looks like meshPtr->getNumLodLevels() is the one to watch.
		//Ogre::Mesh::SubMeshIterator subMeshIter = meshPtr->getSubMeshIterator();
		////meshPtr->getSubMesh(0)->vertexData->vertexCount;
		//int numBoneAssignments = 0;//meshPtr->getSubMesh(0)->getBoneAssignments().size();
		//int numIndices = meshPtr->getSubMesh(0)->indexData->indexCount;
		//bool sharedVertexData = meshPtr->getSubMesh(0)->useSharedVertices;
		//int numVerts = meshPtr->sharedVertexData->vertexCount;

		//os.str("");
		//os << "NumVerts: " << numVerts << ", num indices: " << numIndices << ", shared vertices: " <<
		//	sharedVertexData << ", num bone assignments " << numBoneAssignments ;
		//msg = os.str();
		//gConsole->addOutput(msg);

		//Ogre::VertexDeclaration* decl = meshPtr->sharedVertexData->vertexDeclaration;
		//const Ogre::VertexElement *elem = decl->findElementBySemantic(Ogre::VES_POSITION);
		//Ogre::VertexData* vertex_data = meshPtr->sharedVertexData;

		//Ogre::HardwareVertexBufferSharedPtr vbuf= vertex_data->vertexBufferBinding->getBuffer(elem->getSource());
		//const unsigned int vSize = (unsigned int)vbuf->getVertexSize();
		//unsigned char* vertex =static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
		//size_t vertSize = declaration->getVertexSize(0);

		//mVecCtlPoints.push_back(Vector3(pFloat[0], pFloat[1], pFloat[2]));
		//pVert += vertSize;

		//float* pReal;
		//for (int i=0;i<numVerts;i++)
		//{
		//	elem->baseVertexPointerToElement(vertex, &pReal);
		//	vertex3 = Ogre::Vector3(pReal[0],pReal[1],pReal[2]);
		//	os.str("");
		//	os << "Vert " << i << ": " << vertex3.x << " " << vertex3.y << " " << vertex3.z ;
		//	msg = os.str();
		//	gConsole->addOutput(msg);
		//	vertex += vSize;
		//}
		//vbuf->unlock();

		//DTS::Shape dts_shape;
		//OUCH... COMPLETE FAIL.
		//int version = 0;
		//std::ifstream infile ("BigBarrel.dts",  std::ios::in | std::ios::binary  ) ;
		//if (infile.is_open())
		//{
			//CRASH
			//dts_shape.read(infile);
			//infile.close();
		//}

		// TESTING .. FAIL!  ////////////////////
		//infile.read ((char *) &version, 2) ;
		//int size;
		//char memblock[12];
		//memblock = new char [12];
		//size = (int) infile.tellg();
		//infile.seekg (0, std::ios::beg);
		//infile.read (memblock, 12);
		//////////////////////////////////////


		//std::ostringstream os("Loaded a dts!");
		//os << "Opened up a dts?  version:  " << dts_shape.DT;
		//os << "Opened up a dts?  file contents:  " << memblock;
		//Ogre::String msg(os.str());
		//gConsole->addOutput(msg);

		//std::ofstream outfile ("testobject.dts", std::ios::binary | std::ios::trunc | std::ios::out) ;
		//if (!outfile) {
		//	printf("didn't work!\n");
		//} else {
		//	dts_shape.save(outfile);
		//	printf("worked!\n");
		//}
		//outfile.close();
		//for (unsigned int j = start; j < end; j++)   
		//{
		//	name = kShape->names[mShapeInstance->mMeshObjects[j].object->nameIndex];
		//	//TSSkinMesh *mesh = (TSSkinMesh *)mShapeInstance->mMeshObjects[j].getMesh(od);
		//	TSMesh *mesh = mShapeInstance->mMeshObjects[j].getMesh(od);
		//	physRigidBodyCommon *kRB = (physRigidBodyCommon*)mRB;
		//	kRB->mBodyVertLookups[mc] = vc;

		//	nodeIndex = kShape->objects[j].nodeIndex;
		//	//HERE is the meat of the issue:  need to start talking to Ogre::Entity
		//	// and find out where the lists of quaternions for default pose and 
		//	// animations are stored.
		//	//kShape->defaultRotations[nodeIndex].getQuaternion(&qF);
		//	qF.FromAxes(0.0,0.0,0.0);// (Placeholder)
		//	qF.ToRotationMatrix(&kMat3);	
		//	kTransform = kMat3;	
		//	kTransform.makeTrans(kShape->defaultTranslations[nodeIndex]);
		//	if (mesh) 
		//	{
		//		Ogre::Vector3 kCenter = mesh->getCenter();
		//		Ogre::Vector3 sc = getScale();
		//		TSMesh::TSMeshVertexArray kVertArray = mesh->mVertexData;
		//		//mi->SetNbVertices( mVertexData.isReady() ? mesh->mNumVerts : verts.size() );
		//		//for (unsigned int k=0;k<mesh->verts.size();k++) //Uh oh, major changes to TSMesh
		//		for (unsigned int k=0;k<mesh->mNumVerts;k++)
		//		{
		//			//vertex = mesh->verts[k];
		//			vertex.set(kVertArray[k].vert().x,kVertArray[k].vert().y,kVertArray[k].vert().z);
		//			vertex *= sc;
		//			if (kTransform) kTransform.mulP(vertex,&d);
		//			else d = vertex;
		//			kRB->mVerts.increment();//HERE: is this going to work? 
		//			kRB->mVerts[kRB->mVerts.size()-1]=d;//vertex[MAYBE?]
		//			vc++;
		//		}
		//	}
		//	mc++;
		//}
	}
	//}
}


void fxRigidBody::examineEntity()
{
		Ogre::Vector3 vertex3,d;
		Ogre::Matrix4  kTransform;
		Ogre::Matrix3  kMat3;

		std::ostringstream os;

		gConsole->addOutput("Starting rigid body entity/mesh research...");

		int numSubEnts = mEntity->getNumSubEntities();
		int numManualLODs = mEntity->getNumManualLodLevels();
		Ogre::MeshPtr meshPtr = mEntity->getMesh();
		Ogre::ResourceHandle resHandle = meshPtr->getHandle();
		
		int numAnims = meshPtr->getNumAnimations();
		int numLODMeshLevels = meshPtr->getNumLodLevels();
		int numSubMeshes = meshPtr->getNumSubMeshes();
		int numVerts = -1;
		Ogre::String myPath;
		//Ogre::Resource myResource = mEntity->getR
		//Ogre::ResourceGroupManager::getSingleton().find..?(meshPtr->getName());
		Ogre::StringVector groupList = Ogre::ResourceGroupManager::getSingleton().getResourceGroups();

		//os.str("");
		//os << "Resource Groups: " ;
		//gConsole->addOutput(os.str());
		//for (int i=0;i<groupList.size();i++)
		//{
		//	os.str("");
		//	os <<  groupList[i].c_str();
		//	gConsole->addOutput(os.str());
		//}

		//meshPtr->getName()


		os << mEntity->getName()  << ",  SubEntities " << numSubEnts << ", manualLODS " << numManualLODs << ", anims " <<
			numAnims << ", Mesh LODS " << numLODMeshLevels << ", SubMeshes " << numSubMeshes ;
		gConsole->addOutput(os.str());

		//Very good... making sense so far.  Don't worry about Entity->getNumManualLODs right now, 
		//looks like meshPtr->getNumLodLevels() is the one to watch.
		Ogre::Mesh::SubMeshIterator subMeshIter = meshPtr->getSubMeshIterator();
		//meshPtr->getSubMesh(0)->vertexData->vertexCount;
		int numBoneAssignments = 0;//meshPtr->getSubMesh(0)->getBoneAssignments().size();
		int numIndices = meshPtr->getSubMesh(0)->indexData->indexCount;
		bool sharedVertexData = meshPtr->getSubMesh(0)->useSharedVertices;

		if (sharedVertexData)
		{
			numVerts = meshPtr->sharedVertexData->vertexCount;
		} else {
			for (int i=0;i<numSubMeshes;i++)
			{
				numVerts = meshPtr->getSubMesh(i)->vertexData->vertexCount;
				numIndices = meshPtr->getSubMesh(i)->indexData->indexCount;
				int numBoneAssignments = meshPtr->getSubMesh(i)->getBoneAssignments().size();
				os.str("");
				os << "submesh material: " << meshPtr->getSubMesh(i)->getMaterialName() << ", verts " << numVerts <<
					", indices " << numIndices << ", bone assignments " << numBoneAssignments ;
				gConsole->addOutput(os.str());
			}
		}

		os.str("");
		os << "NumVerts: " << numVerts << ", num indices: " << numIndices << ", shared vertices: " <<
			sharedVertexData << ", num bone assignments " << numBoneAssignments ;
		gConsole->addOutput(os.str());

		
		Ogre::VertexDeclaration* decl;// = meshPtr->sharedVertexData->vertexDeclaration;
		const Ogre::VertexElement *elem;// = decl->findElementBySemantic(Ogre::VES_POSITION);
		Ogre::VertexData* vertex_data;// = meshPtr->sharedVertexData;

		if (sharedVertexData)
		{
			vertex_data = meshPtr->sharedVertexData;
			decl = vertex_data->vertexDeclaration;
			elem = decl->findElementBySemantic(Ogre::VES_POSITION);
		} else {
			vertex_data = meshPtr->getSubMesh(0)->vertexData;
			decl = vertex_data->vertexDeclaration;
			elem = decl->findElementBySemantic(Ogre::VES_POSITION);
		}


		Ogre::HardwareVertexBufferSharedPtr vbuf= vertex_data->vertexBufferBinding->getBuffer(elem->getSource());
		const unsigned int vSize = (unsigned int)vbuf->getVertexSize();
		unsigned char* vertex =static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
		//size_t vertSize = declaration->getVertexSize(0);

		//mVecCtlPoints.push_back(Vector3(pFloat[0], pFloat[1], pFloat[2]));
		//pVert += vertSize;

		//int numBones = 0;
		//if (mEntity->getSkeleton())
		//	numBones = mEntity->getSkeleton()->getNumBones();
		//else return;

		//os.str("");
		//os << "Bones: " << numBones << ", root node: " << mEntity->getSkeleton()->getRootBone()->getName();
		//gConsole->addOutput(os.str());
		//for (int i=0;i<numBones;i++)
		//{
		//	os.str("");
		//	os << mEntity->getSkeleton()->getBone(i)->getName() ;
		//	gConsole->addOutput(os.str());
		//}

		//float* pReal;
		//for (int i=0;i<numVerts;i++)
		//{
		//	elem->baseVertexPointerToElement(vertex, &pReal);
		//	vertex3 = Ogre::Vector3(pReal[0],pReal[1],pReal[2]);
		//	os.str("");
		//	os << "Vert " << i << ": " << vertex3.x << " " << vertex3.y << " " << vertex3.z ;
		//	gConsole->addOutput(os.str());
		//	vertex += vSize;
		//}
		//vbuf->unlock();
}

void fxRigidBody::setupCollisionMesh()
{
	//Broken on 1.8 port -- buildPolyList changed arguments, requires material list (?)
	//
	//Ogre::Vector3 p;
	//Ogre::Vector3 vertex,d;
	//Ogre::Quaternion qF;
	//Ogre::Matrix4  kTransform,kTrans2;
	//int nodeIndex,thisNode;
	//physRigidBodyCommon *kRB = (physRigidBodyCommon*)mRB;

	//unsigned int cols = 0;
	//unsigned int vsum = 0;

	//if (mPM->getType()==PHYS_NX) 
	//{
	//	nxPhysManager *kPM = (nxPhysManager*)mPM;

	//	if (kPM->mMeshes[mStartMesh]) 
	//	{
	//		mRB->setStartMesh(mStartMesh);
	//	} 
	//	else 
	//	{			
	//		TSShape *kShape = mShapeInstance->mShape;
	//		///////////////////////////////////////////////
	//		// Now grab the collision meshes
	//		for (unsigned int i = 0; i < kShape->details.size(); i++)
	//		{

	//			const TSDetail *detail = &kShape->details[i];
	//			char* name = (char *)kShape->names[kShape->details[i].nameIndex].c_str();

	//			//if (dStrstr(dStrlwr(name), "collision-"))
	//			if (!dStrncmp(name,"collision-",10))
	//			{
	//				mShapeInstance->animate(i);
	//				mShapeInstance->setCurrentDetail(i);
	//				//mShapeInstance->setStatics(i);

	//				int ss = detail->subShapeNum;
	//				int od = detail->objectDetailNum;

	//				int start = kShape->subShapeFirstObject[ss];
	//				int end   = kShape->subShapeNumObjects[ss] + start;
	//				const char *myName;
	//				if (start < end) 
	//				{
	//					for (unsigned int j = start; j < end; j++)   
	//					{
	//						myName = kShape->names[mShapeInstance->mMeshObjects[j].object->nameIndex];
	//						TSShapeInstance::MeshObjectInstance * meshObject = &mShapeInstance->mMeshObjects[j];

	//						if (od >= meshObject->object->numMeshes) 
	//							continue;

	//						// Get a valid transform for the polylist
	//						Ogre::Matrix4 mat = meshObject->getTransform();

	//						Ogre::Matrix4 m;
	//						if(mat)
	//							m = mat;
	//						else
	//							m.identity();


	//						//Here: checking to see if we're in a node hierarchy, if so accumulate transforms from parents
	//						nodeIndex = kShape->objects[j].nodeIndex;
	//						kShape->defaultRotations[nodeIndex].getOgre::Quaternion(&qF);
	//						qF.ToRotationMatrix(&kTransform);	
	//						Ogre::Vector3 pos1 = kShape->defaultTranslations[nodeIndex];
	//						//kTransform.makeTrans(pos1);

	//						thisNode = nodeIndex;
	//						while (kShape->nodes[thisNode].parentIndex>=0) 
	//						{
	//							thisNode = kShape->nodes[thisNode].parentIndex;
	//							pos1 +=  kShape->defaultTranslations[thisNode];
	//							//pos1 *= getScale();//?
	//							kShape->defaultRotations[thisNode].getOgre::Quaternion(&qF);
	//							qF.ToRotationMatrix(&kTrans2);	
	//							kTransform *= kTrans2;//Q: does it work this way, or do you have to reverse kTrans2 * kTransform
	//						}
	//						kTransform.makeTrans(pos1);

	//						//Now get verts from the poly list
	//						//polyList.setTransform(&m, Ogre::Vector3(1.0f, 1.0f, 1.0f));

	//						unsigned int surfaceKey = 0;
	//						ConcretePolyList *polyList = new ConcretePolyList();
	//						//meshObject->buildPolyList(od, &polyList, surfaceKey);
	//						mShapeInstance->buildPolyList( polyList, od);

	//						Ogre::Vector3 sc = getScale();
	//						//Con::errorf("polylist vert count: %d",polyList->mVertexList.size());
	//						for (unsigned int j = 0; j < polyList->mVertexList.size(); j++)
	//						{
	//							vertex = polyList->mVertexList[j];
	//							vertex*=sc;
	//							//Con::errorf("polylist vertex: %f %f %f",vertex.x,vertex.y,vertex.z);
	//							if (kTransform) kTransform.mulP(vertex,&d);
	//							else d = vertex;
	//							kRB->mVerts.increment();//HERE: is this going to work? 
	//							kRB->mVerts[kRB->mVerts.size()-1]=d;
	//						}

	//						kRB->setBodyVertLookup(cols++,vsum);
	//						vsum += polyList->mVertexList.size();
	//						kRB->setLastNumVerts(polyList->mVertexList.size());
	//						//mNumMeshes++;
	//					}
	//				}
	//			}
	//		}
	//		//mShapeInstance->clearStatics();
	//	}
	//}
}
//////////////////////////////////////////////
//
//for (unsigned int i = 0; i < mDataBlock->collisionDetails.size(); i++)
//{
//const TSDetail *detail = &kShape->details[mDataBlock->collisionDetails[i]];
//int ss = detail->subShapeNum;
//int od = detail->objectDetailNum;
//int vc = 0;//vertex count
//
//const char *name;
//int start = kShape->subShapeFirstObject[ss];
//int end   = kShape->subShapeNumObjects[ss] + start;
//unsigned int cols = 0;
//int vsum = 0;
//unsigned int maxMeshes = 200;
//TSMesh *mesh;
//
//for (unsigned int j = start; j < end; j++)   {
//name = kShape->names[mShapeInstance->mMeshObjects[j].object->nameIndex];
////if ((dStrncmp(name,"Col",3))&&(dStrncmp(name,"col",3))) continue;
//
//for (unsigned int m=0;m<maxMeshes;m++)
//{
//mesh = mShapeInstance->mMeshObjects[j].getMesh(m);
//if (mesh) {
////Con::errorf("collision mesh %s size %d",name,mesh->verts.size());
//kRB->setBodyVertLookup(cols++,vsum);
//vsum += mesh->verts.size();
//kRB->setLastNumVerts(mesh->verts.size());
//}
//}
//
////mNumMeshes = mShapeInstance->mMeshObjects[j].object->numMeshes;//done in setupMeshBody now
//nodeIndex = kShape->objects[j].nodeIndex;
//kShape->defaultRotations[nodeIndex].getOgre::Quaternion(&qF);
//qF.ToRotationMatrix(&kTransform);	
//Ogre::Vector3 pos1 = kShape->defaultTranslations[nodeIndex];
//thisNode = nodeIndex;
//while (kShape->nodes[thisNode].parentIndex>=0) {
//thisNode = kShape->nodes[thisNode].parentIndex;
//pos1 +=  kShape->defaultTranslations[thisNode];
//kShape->defaultRotations[thisNode].getOgre::Quaternion(&qF);
//qF.ToRotationMatrix(&kTrans2);	
//kTransform *= kTrans2;//Q: does it work this way, or do you have to reverse kTrans2 * kTransform
//}
//kTransform.makeTrans(pos1);
//}
//}
//
//unsigned int jj = 0;
//int ds = mDataBlock->collisionDetails.size();
//
//
//for (unsigned int i = 0; i < mDataBlock->collisionDetails.size(); i++)
//{
//TSShape* shape = mShapeInstance->getShape();
//
//TSShape::ConvexHullAccelerator* pAccel =
//shape->getAccelerator(mDataBlock->collisionDetails[i]);
//if (!pAccel || !pAccel->numVerts)
//continue;
//
//Ogre::Vector3 sc = getScale();
////kRB->setBodyVertLookup(cols++,kRB->mVerts.size());
//for (jj=0; jj < pAccel->numVerts;  jj++) {
//vertex = pAccel->vertexList[jj];
//vertex*=sc;
//if (kTransform) kTransform.mulP(vertex,&d);
//else d = vertex;
//Con::printf("detail %d: %f %f %f",i,d.x,d.y,d.z);
//kRB->mVerts.increment();//HERE: is this going to work? 
//kRB->mVerts[kRB->mVerts.size()-1]=d;
//}
//}




//void fxRigidBody::setupSpring()
//{
//	if (mSpring)
//	{
//
//	}
//}

void fxRigidBody::setupRigidBody()
{
	if (mRB)
		mRB->setup();

	//if (mRB)
	//{
	//	mRB->setEntityType(PHYS_RIGID_BODY);
	//	mRB->setActorGroup(1);//all loose rigid bodies are group #1, for now.

	//	//Q: does this have any effect?? physMaterial handles these properties.
	//	if (mDataBlock->mDynamicFriction)
	//		mRB->setDynamicFriction(mDataBlock->mDynamicFriction);
	//	if (mDataBlock->mStaticFriction)
	//		mRB->setStaticFriction(mDataBlock->mStaticFriction);
	//	if (mDataBlock->mRestitution)
	//		mRB->setRestitution(mDataBlock->mRestitution);
	//	if (mDataBlock->mDensity)
	//		mRB->setDensity(mDataBlock->mDensity);

	//	mRB->setShapeType(mDataBlock->mShapeType);
	//	mRB->setTriggerShapeType(mDataBlock->mTriggerShapeType);
	//	
	//	mRB->setInflictor(mDataBlock->mIsInflictor);

	//	//seems like rigid body should really just have its own pointer to my datablock, but this works.
	//	mRB->setTriggerOffset(mDataBlock->mTriggerOffset);
	//	mRB->setTriggerOrientation(mDataBlock->mTriggerOrientation);
	//	mRB->setTriggerDimensions(mDataBlock->mTriggerDimensions * getScale());
	//	mRB->setProjectileAxis(mDataBlock->mProjectileAxis);

	//	mRB->setOffset(mDataBlock->mOffset * getScale());
	//	mRB->setOrientation(mDataBlock->mOrientation);
	//	mRB->setDimensions(mDataBlock->mDimensions * getScale());
	//	dynamic_cast<nxRigidBody*>(mRB)->mTriggerActorOffset = mDataBlock->mTriggerActorOffset;
	//	if (mDataBlock->mDensity) mRB->setDensity(mDataBlock->mDensity);

	//	mRB->setLinearPosition(mCurrPosition);
	//	mRB->setCurrLinearPosition(mCurrPosition);
	//	mRB->setLastLinearPosition(mCurrPosition);
	//	mRB->setLinearVelocity(mCurrVelocity);

	//	mRB->setCurrForce(mCurrForce);
	//	mRB->setCurrTorque(mCurrTorque);

	//	mRB->setKinematic(mIsKinematic);
	//	mRB->setNoGravity(mIsNoGravity);
	//	mRB->setProjectile(mHasTrigger);
	//	mRB->setHW(mDataBlock->mHW);

	//	mRB->setObjToWorld(mObjToWorld);

	//	if (mPM->getType()==PHYS_NX)
	//	{
	//		if (mDataBlock->mShapeType==PHYS_SHAPE_CONVEX)
	//		{
	//			if (!((nxPhysManager *)mPM)->mMeshes[mStartMesh]) 
	//				setupMesh();
	//			mRB->setNumMeshes(1);
	//		}
	//		else if (mDataBlock->mShapeType==PHYS_SHAPE_COLLISION)
	//		{
	//			if (!((nxPhysManager *)mPM)->mMeshes[mStartMesh]) 
	//				setupCollisionMesh();
	//			mRB->setNumMeshes(mNumMeshes);
	//		}

	//		mRB->setStartMesh(mStartMesh);
	//	}


	//	mPM->addRigidBodySetup(mRB);
	//}
}

void fxRigidBody::setKinematic()
{
	if (mRB)
	{
		mRB->setKinematic(true);
		mIsKinematic = true;
	}
}

void fxRigidBody::clearKinematic()
{
	if (mRB)
	{
		mRB->setKinematic(false);
		mIsKinematic = false;
	}
}

void fxRigidBody::resetPosition()
{
	mReset = true;
}


void fxRigidBody::setWeaponPosAdj(Ogre::Vector3 &pos)
{
	mWeaponPosAdj = pos;
	//Con::errorf("setting weapon pos adj: %3.2f %3.2f %3.2f",mWeaponPosAdj.x,mWeaponPosAdj.y,mWeaponPosAdj.z);
	
	return;
}

void fxRigidBody::setWeaponRotAdjA(Ogre::Vector3 &rot)
{
	//mWeaponRotAdjA.FromAxes(Ogre::Vector3(mDegToRad(rot.x),mDegToRad(rot.y),mDegToRad(rot.z)));
	//Con::errorf("setting weapon rot adj: %3.2f %3.2f %3.2f %3.2f",mWeaponRotAdjA.x,mWeaponRotAdjA.y,mWeaponRotAdjA.z,mWeaponRotAdjA.w);
	return;
}

void fxRigidBody::setWeaponRotAdjB(Ogre::Vector3 &rot)
{
	//mWeaponRotAdjB.FromAxes(mDegToRad(rot.x),mDegToRad(rot.y),mDegToRad(rot.z));
	//Con::errorf("setting weapon rot adj: %3.2f %3.2f %3.2f %3.2f",mWeaponRotAdjB.x,mWeaponRotAdjB.y,mWeaponRotAdjB.z,mWeaponRotAdjB.w);
	return;
}

//void fxRigidBody::setGroup(int groupID)
//{
//	if( Sim::findObject( groupID, mGroup ) == false) Con::errorf("couldn't find the group!");
//	else Con::errorf("group I found has %d members",
//		mGroup->getC);
//
//	return;
//}




void fxRigidBody::addRef()
{
	//dynamic_cast<nxPhysManager*>(mPM)->mConsole->addOutput("DOING STUFF OVER HERE!!!!!!!!!!!!!!!!!!!");
}


void fxRigidBody::release()
{
	//dynamic_cast<nxPhysManager*>(mPM)->mConsole->addOutput("DOING STUFF OVER HERE!!!!!!!!!!!!!!!!!!!");
}


int fxRigidBody::doStuff(int arg)
{

	int i = arg * 4;
	return i;
}
	//physmanager testing
	//nxPhysManager *kPM = dynamic_cast<nxPhysManager*>(mPM);
	//kPM->doStuff();

Ogre::Vector3 fxRigidBody::getNodePosition() const
{
	//const Ogre::Vector3 kPos;
	//Ogre::SceneNode *myNode = dynamic_cast<Ogre::SceneNode *>(_node);
	if (mNode)
	{
		return mNode->getPosition();
	} else {
		return Ogre::Vector3(-999999,-999999,-999999);
	}

}

void fxRigidBody::setNodePosition(const Ogre::Vector3 &pos)
{
	//if (mNode)
	//	mNode->setPosition(pos);

	return;
}




////-----------------------------------------------------------------------
////-----------------------------------------------------------------------
//String fxRigidBodyFactory::FACTORY_TYPE_NAME = "fxRigidBody";
////-----------------------------------------------------------------------
//const String& fxRigidBodyFactory::getType(void) const
//{
//	return FACTORY_TYPE_NAME;
//}
////-----------------------------------------------------------------------
//MovableObject* fxRigidBodyFactory::createInstanceImpl( const String& name,
//																 const NameValuePairList* params)
//{
//
//	// must have mesh parameter
//	//MeshPtr pMesh;
//	//if (params != 0)
//	//{
//	//	String groupName = ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME;
//
//	//	NameValuePairList::const_iterator ni;
//
//	//	ni = params->find("resourceGroup");
//	//	if (ni != params->end())
//	//	{
//	//		groupName = ni->second;
//	//	}
//
//	//	ni = params->find("mesh");
//	//	if (ni != params->end())
//	//	{
//	//		// Get mesh (load if required)
//	//		pMesh = MeshManager::getSingleton().load(
//	//			ni->second,
//	//			// autodetect group location
//	//			groupName );
//	//	}
//
//	//}
//	//if (pMesh.isNull())
//	//{
//	//	OGRE_EXCEPT(Exception::ERR_INVALIDPARAMS,
//	//		"'mesh' parameter required when constructing an Entity.",
//	//		"EntityFactory::createInstance");
//	//}
//
//	return OGRE_NEW Ogre::Entity(name, pMesh);//HERE: need to figure out how to call the 
//	//EntityFactory with all the normal procedures, but then do our own additional logic
//	//by calling fxRigidyBody constructor afterwards.
//	//return OGRE_NEW fxRigidyBody(...?);
//
//}
//-----------------------------------------------------------------------
//void EntityFactory::destroyInstance( MovableObject* obj)
//{
//	OGRE_DELETE obj;
//}


































/////////////////////////////////////////////////////////////////////////////
/*

#include "platform/platform.h"
#include "math/mathio.h"
#include "core/stream/bitStream.h"
#include "console/consoleTypes.h"
#include "ts/tsShapeInstance.h"
#include "collision/ConcretePolyList.h"

#include "gfx/gfxDevice.h"
#include "core/color.h"

#include "T3D/player.h"
#include "T3D/gameBase/gameConnection.h"

//#include "T3D/physicsBAG/fxFluid.h"
#include "T3D/physicsBAG/fxFlexBody.h"
#include "T3D/physicsBAG/myStream.h"

extern int mouseValue;

//class nxRaycastReport;
//extern nxRaycastReport gRaycastReport;

struct physMeshBody;
class fxFlexBody;


IMPLEMENT_CO_DATABLOCK_V1(fxRigidBodyData);

//IMPLEMENT_CONSOLETYPE(fxRigidBodyData)
//IMPLEMENT_SETDATATYPE(fxRigidBodyData)
//IMPLEMENT_GETDATATYPE(fxRigidBodyData)


//had to move these from physRigidBodyManager.cc 
//     because this compiles first. (??)
//IMPLEMENT_CONSOLETYPE(nxRigidBodyPhysicsData)
//IMPLEMENT_SETDATATYPE(nxRigidBodyPhysicsData)
//IMPLEMENT_GETDATATYPE(nxRigidBodyPhysicsData)
//whoops, had to move it to fxFlexBody

fxRigidBodyData::fxRigidBodyData()
{
	shapeName          = "";
	mShapeType         = PHYS_SHAPE_BOX;
	mTriggerShapeType  = PHYS_SHAPE_CAPSULE;
	mLifetimeMS        = 0;

	mDynamicFriction = 0.0;
	mStaticFriction = 0.0;
	mRestitution = 0.0;
	mDensity = 1.0;
	mSleepThreshold = 0.0;

	mOffset = Ogre::Vector3::ZERO;
	mOrientation = Ogre::Vector3::ZERO;
	mDimensions = Ogre::Vector3::ZERO;

	mTriggerOffset = Ogre::Vector3::ZERO;
	mTriggerOrientation = Ogre::Vector3::ZERO;
	mTriggerDimensions = Ogre::Vector3::ZERO;
	mTriggerActorOffset = Ogre::Vector3::ZERO;
	mProjectileAxis = Ogre::Vector3::ZERO;

	mWeaponPosAdj = Ogre::Vector3::ZERO;
	mWeaponRotAdjA = Ogre::Vector3::ZERO;
	mWeaponRotAdjB = Ogre::Vector3::ZERO;

	mIsKinematic = false;
	mIsNoGravity = false;
	mHasTrigger = false;
	mIsTransient = false;
	mHasSpring = false;
	mIsInflictor = false;
	mInflictMultiplier = 0.0;
	mHW = false;

	shadowEnable = false;
	//shadowCanMove = true;
	//shadowCanAnimate = false;
	//shadowSelfShadow = false;
}

fxRigidBodyData::~fxRigidBodyData()
{

}

bool fxRigidBodyData::preload(bool bServer, String &errorStr)
{
	if (!Parent::preload(bServer, errorStr))
	{
		return false;
	}

	return true;
}

bool fxRigidBodyData::onAdd()
{
	if(!Parent::onAdd())
		return false;

	Con::errorf("rigidbody data adding: %s",this->shapeName);
	return true;
}


void fxRigidBodyData::initPersistFields()
{
	Parent::initPersistFields();

	addField("DynamicFriction", Typefloat,
		Offset(mDynamicFriction, fxRigidBodyData));
	addField("StaticFriction", Typefloat,
		Offset(mStaticFriction, fxRigidBodyData));
	addField("Restitution", Typefloat,
		Offset(mRestitution, fxRigidBodyData));
	addField("myDensity", Typefloat,
		Offset(mDensity, fxRigidBodyData));
	addField("SleepThreshold", Typefloat,
		Offset(mSleepThreshold, fxRigidBodyData));
	addField("ShapeType", TYPEID< physShapeType >(),
		Offset(mShapeType, fxRigidBodyData));
	addField("TriggerShapeType", TYPEID< physShapeType >(),
		Offset(mTriggerShapeType, fxRigidBodyData));
	addField("Offset", TypeOgre::Vector3,
		Offset(mOffset, fxRigidBodyData));
	addField("Orientation", TypeOgre::Vector3,
		Offset(mOrientation, fxRigidBodyData));
	addField("Dimensions", TypeOgre::Vector3,
		Offset(mDimensions, fxRigidBodyData));
	addField("TriggerOffset", TypeOgre::Vector3,
		Offset(mTriggerOffset, fxRigidBodyData));
	addField("TriggerOrientation", TypeOgre::Vector3,
		Offset(mTriggerOrientation, fxRigidBodyData));
	addField("TriggerDimensions", TypeOgre::Vector3,
		Offset(mTriggerDimensions, fxRigidBodyData));
	addField("TriggerActorOffset", TypeOgre::Vector3,
		Offset(mTriggerActorOffset, fxRigidBodyData));
	addField("ProjectileAxis", TypeOgre::Vector3,
		Offset(mProjectileAxis, fxRigidBodyData));
	addField("WeaponPosAdj", TypeOgre::Vector3,
		Offset(mWeaponPosAdj, fxRigidBodyData));
	addField("WeaponRotAdjA", TypeOgre::Vector3,
		Offset(mWeaponRotAdjA, fxRigidBodyData));
	addField("WeaponRotAdjB", TypeOgre::Vector3,
		Offset(mWeaponRotAdjB, fxRigidBodyData));
	addField("IsKinematic", TypeBool,
		Offset(mIsKinematic, fxRigidBodyData));
	addField("IsNoGravity", TypeBool,
		Offset(mIsNoGravity, fxRigidBodyData));
	addField("HasTrigger", TypeBool,
		Offset(mHasTrigger, fxRigidBodyData));
	addField("IsTransient", TypeBool,
		Offset(mIsTransient, fxRigidBodyData));
	addField("HasSpring", TypeBool,
		Offset(mHasSpring, fxRigidBodyData));
	addField("HW", TypeBool,
		Offset(mHW, fxRigidBodyData));
	addField("Lifetime", Typeint, 
		Offset(mLifetimeMS, fxRigidBodyData));
	addField("IsInflictor", TypeBool,
		Offset(mIsInflictor, fxRigidBodyData));
	addField("InflictMultiplier", Typefloat,
		Offset(mInflictMultiplier, fxRigidBodyData));

}

void fxRigidBodyData::packData(BitStream* pBitStream)
{
	Parent::packData(pBitStream);

	pBitStream->write(mDynamicFriction);
	pBitStream->write(mStaticFriction);
	pBitStream->write(mRestitution);
	pBitStream->write(mDensity);
	pBitStream->write(mSleepThreshold);
	pBitStream->writeRangedunsigned int(mShapeType, 0, NUM_SHAPE_TYPES-1);
	pBitStream->writeRangedunsigned int(mTriggerShapeType, 0, NUM_SHAPE_TYPES-1);
	mathWrite(*pBitStream, mOffset);
	mathWrite(*pBitStream, mOrientation);
	mathWrite(*pBitStream, mDimensions); 
	mathWrite(*pBitStream, mTriggerOffset);
	mathWrite(*pBitStream, mTriggerOrientation);
	mathWrite(*pBitStream, mTriggerDimensions);
	mathWrite(*pBitStream, mTriggerActorOffset);
	mathWrite(*pBitStream, mProjectileAxis);
	mathWrite(*pBitStream, mWeaponPosAdj);
	mathWrite(*pBitStream, mWeaponRotAdjA);
	mathWrite(*pBitStream, mWeaponRotAdjB);
	pBitStream->write(mIsKinematic);
	pBitStream->write(mIsNoGravity);
	pBitStream->write(mHasTrigger);
	pBitStream->write(mIsTransient);
	pBitStream->write(mHasSpring);
	pBitStream->write(mHW);
	pBitStream->write(mLifetimeMS);
	pBitStream->write(mIsInflictor);
	pBitStream->write(mInflictMultiplier);
}

void fxRigidBodyData::unpackData(BitStream* pBitStream)
{
	Parent::unpackData(pBitStream);

	pBitStream->read(&mDynamicFriction);
	pBitStream->read(&mStaticFriction);
	pBitStream->read(&mRestitution);
	pBitStream->read(&mDensity);
	pBitStream->read(&mSleepThreshold);
	mShapeType = (physShapeType)pBitStream->readRangedunsigned int(0, NUM_SHAPE_TYPES-1);
	mTriggerShapeType = (physShapeType)pBitStream->readRangedunsigned int(0, NUM_SHAPE_TYPES-1);
	mathRead(*pBitStream, &mOffset);
	mathRead(*pBitStream, &mOrientation);
	mathRead(*pBitStream, &mDimensions);
	mathRead(*pBitStream, &mTriggerOffset);
	mathRead(*pBitStream, &mTriggerOrientation);
	mathRead(*pBitStream, &mTriggerDimensions);
	mathRead(*pBitStream, &mTriggerActorOffset);
	mathRead(*pBitStream, &mProjectileAxis);
	mathRead(*pBitStream, &mWeaponPosAdj);
	mathRead(*pBitStream, &mWeaponRotAdjA);
	mathRead(*pBitStream, &mWeaponRotAdjB);
	pBitStream->read(&mIsKinematic);
	pBitStream->read(&mIsNoGravity);
	pBitStream->read(&mHasTrigger);
	pBitStream->read(&mIsTransient);
	pBitStream->read(&mHasSpring);
	pBitStream->read(&mHW);
	pBitStream->read(&mLifetimeMS);
	pBitStream->read(&mIsInflictor);
	pBitStream->read(&mInflictMultiplier);
}

////////////////////////////////////////////////

IMPLEMENT_CO_NETOBJECT_V1(fxRigidBody);

////////////////////////////////////////////////////////////////

ConsoleMethod(fxRigidBody,setLifetime,void,3,3,"Sets lifetime in MS.")
{
	unsigned int lifetime;
	dSscanf(argv[2],"%d",&lifetime);
	object->mLifetimeMS = lifetime;
	object->mCurrMS = 0;
	return;
}

ConsoleMethod( fxRigidBody, setTorque, void, 3, 3,"(Ogre::Vector3 kTorque)")
{
	Ogre::Vector3 kTorque;	
	dSscanf(argv[2], "%g %g %g", &kTorque.x, &kTorque.y, &kTorque.z);
	object->mRB->setCurrTorque(kTorque);
	return;
}

ConsoleMethod( fxRigidBody, setGlobalTorque, void, 3, 3,"(Ogre::Vector3 kTorque)")
{
	Ogre::Vector3 kTorque;	
	dSscanf(argv[2], "%g %g %g", &kTorque.x, &kTorque.y, &kTorque.z);
	object->mRB->setGlobalTorque(kTorque);
	return;
}

ConsoleMethod(fxRigidBody,setForce,void,3,3,"Sets local force.")
{
	Ogre::Vector3 kForce( 0.0f,0.0f,0.0f );
	dSscanf( argv[2], "%f %f %f", &kForce.x, &kForce.y, &kForce.z );
	object->mRB->setCurrForce(kForce);
	return;
}

ConsoleMethod(fxRigidBody,setGlobalForce,void,3,3,"Sets global force.")
{
	Ogre::Vector3 kForce( 0.0f,0.0f,0.0f );
	dSscanf( argv[2], "%f %f %f", &kForce.x, &kForce.y, &kForce.z );
	object->mRB->setGlobalForce(kForce);
	return;
}

ConsoleMethod(fxRigidBody,setGlobalDelayForce,void,3,3,"Sets delayed global force.")
{
	Ogre::Vector3 kForce( 0.0f,0.0f,0.0f );
	dSscanf( argv[2], "%f %f %f", &kForce.x, &kForce.y, &kForce.z );
	object->mRB->setGlobalDelayForce(kForce);
	return;
}

ConsoleMethod(fxRigidBody,setNoGravity,void,3,3,"Sets gravity (true/false).")
{
	int grav;
	dSscanf( argv[2], "%d", &grav );
	object->mIsNoGravity = grav;
	object->mRB->setNoGravity(grav);
	return;
}

ConsoleMethod(fxRigidBody,setLinearVelocity,void,3,3,"setLinearVelocity(Ogre::Vector3)")
{
	Ogre::Vector3 kForce( 0.0f,0.0f,0.0f );
	dSscanf( argv[2], "%f %f %f", &kForce.x, &kForce.y, &kForce.z );
	object->mRB->setLinearVelocity(kForce);
	return;
}

ConsoleMethod(fxRigidBody,setAngularVelocity,void,3,3,"setAngularVelocity(Ogre::Vector3)")
{
	Ogre::Vector3 kForce( 0.0f,0.0f,0.0f );
	dSscanf( argv[2], "%f %f %f", &kForce.x, &kForce.y, &kForce.z );
	object->mRB->setAngularVelocity(kForce);
	return;
}

ConsoleMethod(fxRigidBody,setIsStriking,void,3,3,"(bool isStriking)")
{
	object->mIsStriking = dAtoi(argv[2]);
}

ConsoleMethod(fxRigidBody,setKinematic,void,2,2,"()")
{
	object->setKinematic();
}

ConsoleMethod(fxRigidBody,clearKinematic,void,2,2,"()")
{
	object->clearKinematic();
}

ConsoleMethod(fxRigidBody,resetPosition,void,2,2,"()")
{
	object->resetPosition();
}

ConsoleMethod(fxRigidBody,setWeaponPosAdj,void,3,3,"setWeaponPosAdj(Ogre::Vector3)")
{
	Ogre::Vector3 kPos( 0.0f,0.0f,0.0f );
	dSscanf( argv[2], "%f %f %f", &kPos.x, &kPos.y, &kPos.z );
	object->setWeaponPosAdj(kPos);
	return;
}

ConsoleMethod(fxRigidBody,getWeaponPosAdj,void,2,2,"getWeaponPosAdj()")
{
	Con::printf("WeaponPosAdj: %3.2f %3.2f %3.2f",
		object->mWeaponPosAdj.x,object->mWeaponPosAdj.y,object->mWeaponPosAdj.z);
	return;
}

ConsoleMethod(fxRigidBody,setWeaponRotAdjA,void,3,3,"setWeaponRotAdjA(Ogre::Vector3)")
{
	Ogre::Vector3 kRot( 0.0f,0.0f,0.0f );
	dSscanf( argv[2], "%f %f %f", &kRot.x, &kRot.y, &kRot.z );
	object->setWeaponRotAdjA(kRot);
	return;
}

ConsoleMethod(fxRigidBody,setWeaponRotAdjB,void,3,3,"setWeaponRotAdjB(Ogre::Vector3)")
{
	Ogre::Vector3 kRot( 0.0f,0.0f,0.0f );
	dSscanf( argv[2], "%f %f %f", &kRot.x, &kRot.y, &kRot.z );
	object->setWeaponRotAdjB(kRot);
	return;
}

ConsoleMethod(fxRigidBody,getPhysVelocity,const char *,2,2,"getPhysVelocity()")
{
	char* buff = Con::getReturnBuffer(100);
	
	NetConnection *toServer = NetConnection::getConnectionToServer();
	NetConnection *toClient = NetConnection::getLocalClientConnection();
	
	fxRigidBody *clientObject = NULL;
	Ogre::Vector3 kVel;// = NULL;
	int objectID;
	if (object) {
		objectID = toClient->getGhostIndex(object);
		clientObject = (fxRigidBody *)(toServer->resolveGhost(objectID));
	}
	if (clientObject->mRB)
	{
		kVel = clientObject->mRB->getLinearVelocity();
		if (kVel)
			dSprintf(buff,100,"%g %g %g",kVel.x,kVel.y,kVel.z);
	}
	return buff;
}

ConsoleMethod(fxRigidBody,getPhysSpeed,float,2,2,"getPhysSpeed()")
{
	float kSpeed = 0.0;

	NetConnection *toServer = NetConnection::getConnectionToServer();
	NetConnection *toClient = NetConnection::getLocalClientConnection();
	
	fxRigidBody *clientObject = NULL;
	int objectID;
	
	if (object)
	{
		objectID = toClient->getGhostIndex(object);
		clientObject = (fxRigidBody *)(toServer->resolveGhost(objectID));
	}
	
	if(objectID == -1) return kSpeed;  //Sanity Check

	if (clientObject->mRB)
	{
		kSpeed = clientObject->mRB->getLinearVelocity().len();
	}

	return kSpeed;
}

ConsoleMethod(fxRigidBody,remove,void,2,2,"remove()")
{
	object->onRemove();
}
//ConsoleMethod(fxRigidBody,setGroup,void,3,3,"setGroup(int groupID)");
//{
//	//%group = ?
//	int groupID = dAtoi(argv[2]);
//	object->setGroup(groupID);
//	return;
//}
*/