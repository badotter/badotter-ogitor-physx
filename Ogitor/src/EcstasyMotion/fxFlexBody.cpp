////////////////////////////////////////////////////////////////////////////////////////////////////
//  fxFlexBody.cc
//  Chris Calef
//
////////////////////////////////////////////////////////////////////////////////////////////////////
//#include "core/stl_fix.h"
//
//#include "platform/platform.h"
////#include "audio/audio.h"
//#include "console/consoleTypes.h"
//#include "core/stream/bitstream.h"
////#include "editor/editor.h"
//#include "math/mathio.h"
//#include "ts/tsShapeInstance.h"
//#include "ts/tsShapeConstruct.h"
//#include "collision/ConcretePolyList.h"
//#include "math/mathUtils.h"
//#include "math/mRandom.h"
//#include "sim/netConnection.h"
//#include "core/stream/fileStream.h"
////#include "T3D/aiGuard.h"
//#include "console/SimXMLDocument.h"
//#include "console/SQLiteObject.h"

#include "EcstasyMotion/fxFlexBody.h"
#include "EcstasyMotion/fxRigidBody.h"
#include "EcstasyMotion/fxFlexBodyPart.h"
#include "EcstasyMotion/nxPhysManager.h"
#include "EcstasyMotion/physJoint.h"
#include "EcstasyMotion/physRigidBody.h"
#include "EcstasyMotion/fxRigidBody.h"
#include "EcstasyMotion/nxRigidBody.h"
#include "EcstasyMotion/fxJoint.h"
#include "EcstasyMotion/gaAction.h"
#include "EcstasyMotion/nxJoint.h"
#include "EcstasyMotion/EulerAngles.h"

#include <typeinfo>

#include "OgitorsScriptConsole.h"
extern Ogitors::OgitorsScriptConsole *gConsole;
extern SQLiteObject *gSQL;


//#include "natnettypes.h"
//#include "NatNetClient.h"
//#include "NatNetServer.h"
////#include "NatNetHelper.h"

struct physMeshBody;

//Silly, get rid of this, defines whether we put the word "groundframes" into groundframes dsqs.
#define ECSTASY_LABEL_GROUNDFRAMES 1

int gGetUpState = -1;//TEMP  This is for simple Get Up logic, standing in for entire AI class, gaActionUser etc.

fxFlexBody *gTweakerOne = NULL;
fxFlexBody *gTweakerTwo = NULL;
fxFlexBody *gArenaBot = NULL;
unsigned int gLastArenaTick = 0;



//IMPLEMENT_CO_DATABLOCK_V1(physGroundSequenceData);

//IMPLEMENT_CONSOLETYPE(physGroundSequenceData)
//IMPLEMENT_SETDATATYPE(physGroundSequenceData)
//IMPLEMENT_GETDATATYPE(physGroundSequenceData)

// FBX Function Prototypes.
//bool CreateScene(KFbxSdkManager* pSdkManager, KFbxScene* pScene, fxFlexBody *,int seq);
//
//KFbxNode* CreatePatch(KFbxSdkManager* pSdkManager, char* pName, fxFlexBody *);
//KFbxNode* CreateSkeleton(KFbxSdkManager* pSdkManager, char* pName, fxFlexBody *);
//
//void LinkPatchToSkeleton(KFbxSdkManager* pSdkManager, KFbxNode* pPatch, KFbxNode* pSkeletonRoot, fxFlexBody *);
//void StoreBindPose(KFbxSdkManager* pSdkManager, KFbxScene* pScene, KFbxNode* pPatch, KFbxNode* pSkeletonRoot, fxFlexBody *);
//void StoreRestPose(KFbxSdkManager* pSdkManager, KFbxScene* pScene, KFbxNode* pSkeletonRoot, fxFlexBody *);
//void AnimateSkeleton(KFbxSdkManager* pSdkManager, KFbxScene* pScene, KFbxNode* pSkeletonRoot, fxFlexBody *,int seq);
//void AddThumbnailToScene(KFbxSdkManager* pSdkManager, KFbxScene* pScene);
//void AddThumbnailToTake(KFbxSdkManager* pSdkManager, KFbxScene* pScene, KString& pTakeName, int pThumbnailIndex);
//void AddNodeRecursively(KArrayTemplate<KFbxNode*>& pNodeArray, KFbxNode* pNode);


//ARENA Streaming
//NatNetClient natnetClient;
//
//
//void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData);			// receives data from the server
//void __cdecl MessageHandler(int msgType, char* msg);		// receives NatNet error mesages
//bool startArenaStreaming();
//bool startArenaStreaming(const char *);
//void stopArenaStreaming();

//IMPLEMENT_CO_NETOBJECT_V1(fxFlexBody);


fxFlexBody::fxFlexBody()
{
	//Don't use this.  Might make it create a default flexbody in the future.
}

fxFlexBody::fxFlexBody(Ogre::Entity *entity,const char *meshFile)
{
	//mNetFlags.clear(Ghostable);

	mFlexBodyDataID = 0;// not assigned by database yet
	mPersonaID = 1;//personaDefault = 1
	mPlaylistID = 0;//personaDefault = 1
	mPlaylistName = "";//Is this legal with StringTableEntry?  Does it need insert?
	mSkeletonID = 0;//ACK = 1 - FIX!  When fxFlexBodyData is in the DB it will be easy.  Hacking it for now in onAdd.
	mSkeletonName = "";
	mPersonaName = "";
	mActorID = 0;
	mActorName = "";

	mFollowEvent = NULL;
	mFirstFollowEvent = NULL;
	mFollowEventDone = false;
	mLoadedKeyframeSets = false;

	//iPhysUser data
	mFlexBody = this;
	mTempForce = Ogre::Vector3::ZERO;
	mTempPos = Ogre::Vector3::ZERO;
	mHasTempForce = false;
	mHasTractorBeam = false;
	mHasTractorSpring = false;
	mHasSpring = false;
	mIsInflictor = false;
	mInflictMultiplier = 0.0;

	mMoveThreshold = 1.0;
	mMoveTarget = Ogre::Vector3::ZERO;
	mMoveSequence = "";//"walkfast"

	//mTypeMask |= StaticObjectType | ShadowCasterObjectType | ShapeBaseObjectType;
	//mDataBlock	    = NULL;

	mCurrPosition = Ogre::Vector3::ZERO;
	mCurrVelocity = Ogre::Vector3::ZERO;

	mInitialPosition = Ogre::Vector3::ZERO;
	mInitialLinearVelocity = Ogre::Vector3::ZERO;
	mInitialAngularVelocity = Ogre::Vector3::ZERO;
	mInitialOrientation = Ogre::Quaternion::IDENTITY;
	mRecordInitialPosition = Ogre::Vector3::ZERO;
	mRecordInitialOrientation = Ogre::Quaternion::IDENTITY;

	mCurrTick = 0;
	mCurrMS = 0;
	mLifetimeMS = 0;
	mTriggerTimeMS = 0;
	mRagdollStep = 0;

	mPM = NULL;

	mGroundStep = 0;
	mIsGroundAnimating = false;
	mIsMoveTargeting = false;
	//mGroundPart = NULL;
	mGroundNode = 0;
	//mGroundSequenceData = NULL;
	mGroundVector = Ogre::Vector3::ZERO;
	mCurrThrTime = 0.0;
	mLastThrTime = 0.0;
	mSleepThreshold = 0.00;//0.001
	mDeltaVector = Ogre::Vector3::ZERO;

	mWeapon = NULL;
	mWeaponID = -1;
	mWeaponNodeName = "";
	mWeaponJoint = NULL;
	mWeaponBodypart = NULL;
	mWeaponMountNode = -1;

	mWeapon2 = NULL;
	mWeapon2ID = -1;
	mWeapon2NodeName = "";
	mWeapon2Joint = NULL;
	mWeapon2Bodypart = NULL;
	mWeapon2MountNode = -1;

	mWeaponPos = Ogre::Vector3::ZERO; mWeapon2Pos = Ogre::Vector3::ZERO;
	mWeaponPosAdj = Ogre::Vector3::ZERO; mWeapon2PosAdj = Ogre::Vector3::ZERO;
	mWeaponRot = Ogre::Quaternion::IDENTITY; mWeapon2Rot = Ogre::Quaternion::IDENTITY;
	mWeaponRotAdjA = Ogre::Quaternion::IDENTITY; mWeaponRotAdjB = Ogre::Quaternion::IDENTITY;
	mWeapon2RotAdjA = Ogre::Quaternion::IDENTITY;mWeapon2RotAdjB = Ogre::Quaternion::IDENTITY;
	mWeaponTriggerRotAdjA = Ogre::Quaternion::IDENTITY; mWeaponTriggerRotAdjB = Ogre::Quaternion::IDENTITY;

	mTriggerActor = NULL;
	mEntityType = PHYS_FLEX_BODY;
	mEntitySubType = PHYS_SUB_UNDEFINED;
	mShapeName = "";

	mKeyframesFile = "";
	mPlaylistFile = "";
	mSpawnScript = "";
	mPlaylistDelay = 0;
	mCurrSeq = -1;
	mSequenceStartStep = 0;
	mSequenceEndStep = 0;

	mIsClientOnly = false;
	mIsAnimating = false;
	mIsKinematic = false;
	mIsNoGravity = false;
	mStopAnimating = false;
	mClearIsAnimating = false;
	mClearBodyAnimating = false;
	mIsThinking = false;
	mIsPlayer = false;
	mIsStriking = false;
	mIsCorpse = false;
	mIsPhysActive = false;
	mHasCollisionWaiting = false;
	mReset = false;
	mIsRecording = false;
	mIsRendering = true;
	mIsReturnToZero = false;
	mIsStreaming = false;
	mArenaStreamWeap = false;
	mArenaStreamWeap2 = false;

	mRecordSampleRate = 1;
	mRecordCount = 0;

	mNumBodyParts = 0;
	mFirstNode = 0;
	mActorGroup = -1;

	mMeshBody = -1;
	mStartMesh = -1;
	mNumMeshes = -1;

	mHeadIndex = -1;
	mNeckIndex = -1;
	mBodyIndex = -1;
	mRightFrontIndex = -1;//for biped, arms
	mLeftFrontIndex = -1;//for quadruped, front legs
	mRightBackIndex = -1;//biped legs or quadruped back legs
	mLeftBackIndex = -1;

	//overrideOptions = false;

	for (unsigned int i=0;i<MAX_FLEX_PARTS;i++) {
		mBodyParts[i] = NULL;
		mBaseNodeNames[i] = "";
		mIndexBones[i] = -1;
	}

	for (unsigned int i=0;i<MAX_FLEX_CHAINS;i++) {
		mChainParts[i] = NULL;
	}

	mNumMeshExcludes = 0;
	for (unsigned int i=0;i<MAX_MESH_EXCLUDES;i++) {
		mMeshExcludes[i] = -1;
	}

	for (unsigned int i=0;i<MAX_FLEX_NODES;i++) {
		mDefaults[i] = Ogre::Quaternion::IDENTITY;
		//mResets[i] = Ogre::Quaternion::IDENTITY;
	}
	mResetPoint = Ogre::Vector3::ZERO;

	mPlaylistFile.clear();
	mKeyframesFile.clear();

	//mHeadHThread = mHeadVThread = 0;

	//Ecstasy Motion
	mActionUser = NULL;
   mAnimationFreeze = false;
   mShapeSize = 2.4;//2.4 to compare orc/ack=2.4 to normal human=2.0
   
   mRunningPlaylist = false;
   mBeenTweakerBot = false;
   mBvhCfgUsingNames = false;
   mPlaylistTick = 0;

   mNoLegRagdoll = true;//Expose to gui!
   mGoFullRagdoll = false;
   mBeenHit = false;
   mBeenHitTick = 0;
   mPhysicsDamage = 0.0;

   mPlaylistRepeats = 1;
   mCurrentPlaylistSeq = 0;
   mLastFrameChecked = 0;//-1?
   mBaseNodeSetPos = Ogre::Vector3::ZERO;
   mBaseNodeAdjustPos = Ogre::Vector3::ZERO;
   mOrigin = Ogre::Vector3::ZERO;
   mImportSampleRate = 1;

   //mShapeBaseMount = NULL;
   mTarget = NULL;
   mCollisionTime = 15;//15 ticks, 1/2 second, to wait after last collision before trying to reanimate.
   //End Ecstasy Motion
	mVerbose = false;

	mPM = physManagerCommon::getPM();

	mEntity = entity;
	mNode = mEntity->getParentSceneNode();
	mShapeName = meshFile;

	//Now: loadSQL is where we will create all the bodyparts, and load anything else
	//that's defined in the database.

	examineEntity();

	//loadAnim("walkFull");//TEMP - keep default starting anim in DB.

	if (!loadSQL())
	{
		gConsole->addOutput("Failed to load flexbody SQL!!");
		return;
	}




	//mEntity->setDisplaySkeleton(true);

	gConsole->addOutput("Made a flexbody!!");
}


fxFlexBody::~fxFlexBody()
{
	//HERE: delete flexbodyparts
	//for (unsigned int i=0;i<mNumBodyParts;i++) mBodyParts[i]->onRemove();
}


bool fxFlexBody::loadSQL()
{
	std::ostringstream strResult;

	//physRigidBodyCommon *kRB = dynamic_cast<physRigidBodyCommon*>(mRB);
	//if (!kRB)
	//	return false;

	if (gSQL)
	{
		if (gSQL->OpenDatabase("EcstasyMotion.db"))
		{
			std::ostringstream queryFlexBody,queryFlexBodyPart;
			sqlite_resultset *resultSet; 
			//strQuery = std::string("");
			queryFlexBody << "SELECT id, skeleton_id, name, lifetime, sleepThreshold, " <<
				"relaxType, isKinematic, isNoGravity, headNode, neckNode, bodyNode, " <<
				"rightFrontNode, leftFrontNode, rightBackNode, leftBackNode, tailNode, " <<
				"scale_x, scale_y, scale_z " <<
				"FROM fxFlexBodyData WHERE shapeFile = '" << mShapeName.c_str() << "';" ;

			int result = gSQL->ExecuteSQL(queryFlexBody.str().c_str());
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
							strResult << "query returned multiple matches for shapeFile: " << mShapeName.c_str() ;
						} else {

							mFlexBodyDataID = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);
							mSkeletonID = strtol(resultSet->vRows[0]->vColumnValues[1],NULL,10);
							//mShapeName = resultSet->vRows[0]->vColumnValues[2];
							mLifetimeMS = strtod(resultSet->vRows[0]->vColumnValues[3],NULL);
							mSleepThreshold = strtod(resultSet->vRows[0]->vColumnValues[4],NULL);
							mRelaxType = strtol(resultSet->vRows[0]->vColumnValues[5],NULL,10);
							mIsKinematic = (bool)strtol(resultSet->vRows[0]->vColumnValues[6],NULL,10);
							mIsNoGravity = (bool)strtol(resultSet->vRows[0]->vColumnValues[7],NULL,10);

							//Option: make these Ogre::Bone* instead of int?
							//Ogre::Bone *kBone;
							//kBone = mEntity->getSkeleton()->getBone( Ogre::String(resultSet->vRows[0]->vColumnValues[8]) );
							//if (kBone) mHeadNode = kBone->getHandle();
							//mNeckNode = mEntity->getSkeleton()->getBone(Ogre::String(resultSet->vRows[0]->vColumnValues[9]))->getHandle();
							//mBodyNode = mEntity->getSkeleton()->getBone(Ogre::String(resultSet->vRows[0]->vColumnValues[10]))->getHandle();
							//mRightFrontNode = mEntity->getSkeleton()->getBone(Ogre::String(resultSet->vRows[0]->vColumnValues[11]))->getHandle();
							//mLeftFrontNode = mEntity->getSkeleton()->getBone(Ogre::String(resultSet->vRows[0]->vColumnValues[12]))->getHandle();
							//mRightBackNode = mEntity->getSkeleton()->getBone(Ogre::String(resultSet->vRows[0]->vColumnValues[13]))->getHandle();
							//mLeftBackNode = mEntity->getSkeleton()->getBone(Ogre::String(resultSet->vRows[0]->vColumnValues[14]))->getHandle();
							//mTailNode = mEntity->getSkeleton()->getBone(Ogre::String(resultSet->vRows[0]->vColumnValues[15]))->getHandle();

							Ogre::Vector3 kScale( strtod(resultSet->vRows[0]->vColumnValues[16],NULL),
											strtod(resultSet->vRows[0]->vColumnValues[17],NULL),strtod(resultSet->vRows[0]->vColumnValues[18],NULL));
							if (kScale.length() > 0.0)
								mScale = kScale;
							else
								mScale = Ogre::Vector3(1,1,1);
							if (mNode->getScale().length()==Ogre::Vector3(1,1,1).length())
								mNode->scale(mScale);


							//mNode->setInheritScale // bones need to inherit scale from scene node?

							//Ogre::Vector3 nScale = mNode->getScale();

							//Now, go ahead and set up all the bodyparts with DB data as well, so we don't have  
							//to do queries one at a time for each part.
							queryFlexBodyPart << "SELECT p.id, p.fxJointData_id, p.name, p.baseNode, p.shapeType, " <<
								"p.dimensions_x, p.dimensions_y, p.dimensions_z, p.offset_x, p.offset_y, p.offset_z, " << 
								"p.orientation_x, p.orientation_y, p.orientation_z, p.damageMultiplier, p.isInflictor, p.density," <<
								"p.isNoGravity, p.isKinematic, j.jointType, j.twistLimit, j.swingLimit, j.swingLimit2, j.localAxis_x, j.localAxis_y, " <<
								"j.localAxis_z, j.localNormal_x, j.localNormal_y, j.localNormal_z, j.maxForce, j.maxTorque " <<
								"FROM fxFlexBodyPartData p LEFT JOIN fxJointData j ON p.fxJointData_id = j.id " <<
								"WHERE fxFlexBodyData_id = " << mFlexBodyDataID << ";" ;
							result = gSQL->ExecuteSQL(queryFlexBodyPart.str().c_str());

							if (result)
							{
								resultSet = gSQL->GetResultSet(result);
								if (resultSet<=0)
								{
									gConsole->addOutput("no bodyparts found");
								} else {
									mPM->stopPhysics();
									for (int i=0;i<resultSet->iNumRows;i++) 
									{

										mBodyParts[i] = new fxFlexBodyPart(i);
										mBodyParts[i]->mFlexBody = this;

										//mBodyParts[k]->mMesh = mStartMesh + k;
										mBodyParts[i]->mFlexBodyPartDataID = strtol(resultSet->vRows[i]->vColumnValues[0],NULL,10);
										mBodyParts[i]->mJointDataID = strtol(resultSet->vRows[i]->vColumnValues[1],NULL,10);
										mBodyParts[i]->mFlexBodyPartName = resultSet->vRows[i]->vColumnValues[2];
										mBodyParts[i]->mNodeName = resultSet->vRows[i]->vColumnValues[3];

										//NOW: find this node in my list of bones, and assign my mBone pointer
										mBodyParts[i]->mBone = mEntity->getSkeleton()->getBone(mBodyParts[i]->mNodeName);
										mBodyParts[i]->mNodeIndex = mBodyParts[i]->mBone->getHandle();

										mBodyParts[i]->mRB = mPM->createRigidBody();
										mBodyParts[i]->mRB->setPhysUser((iPhysUser *)mBodyParts[i]);
										mBodyParts[i]->mRB->setLinearPosition(mNode->getPosition());

										//RigidBody parameters...
										mBodyParts[i]->mRB->setShapeType((physShapeType)strtol(resultSet->vRows[i]->vColumnValues[4],NULL,10));
										mBodyParts[i]->mRB->setDimensions(Ogre::Vector3(strtod(resultSet->vRows[i]->vColumnValues[5],NULL),
											strtod(resultSet->vRows[i]->vColumnValues[6],NULL),strtod(resultSet->vRows[i]->vColumnValues[7],NULL)));
										mBodyParts[i]->mRB->setOffset(Ogre::Vector3(strtod(resultSet->vRows[i]->vColumnValues[8],NULL),
											strtod(resultSet->vRows[i]->vColumnValues[9],NULL),strtod(resultSet->vRows[i]->vColumnValues[10],NULL)));
										mBodyParts[i]->mRB->setOrientation(Ogre::Vector3(strtod(resultSet->vRows[i]->vColumnValues[11],NULL),
											strtod(resultSet->vRows[i]->vColumnValues[12],NULL),strtod(resultSet->vRows[i]->vColumnValues[13],NULL)));

										mBodyParts[i]->mDamageMultiplier = strtod(resultSet->vRows[i]->vColumnValues[14],NULL);
										mBodyParts[i]->mIsInflictor = (bool)strtol(resultSet->vRows[i]->vColumnValues[15],NULL,10);
										float density = strtod(resultSet->vRows[i]->vColumnValues[16],NULL);
										if (density==0.0) density = 1.0;
										mBodyParts[i]->mRB->setDensity(density);

										//strResult.str("");
										//strResult << "flexbody bone inheritScale  " << mBodyParts[i]->mBone->getInheritScale();
										//gConsole->addOutput(strResult.str());

										if (mIsNoGravity) 
											mBodyParts[i]->mIsNoGravity = true;
										else
											mBodyParts[i]->mIsNoGravity = (bool)strtol(resultSet->vRows[i]->vColumnValues[17],NULL,10);
										mBodyParts[i]->mRB->setNoGravity(mBodyParts[i]->mIsNoGravity);

										if (mIsKinematic) 
											mBodyParts[i]->mIsKinematic = true;
										else
											mBodyParts[i]->mIsKinematic = (bool)strtol(resultSet->vRows[i]->vColumnValues[18],NULL,10);
										mBodyParts[i]->mRB->setKinematic(mBodyParts[i]->mIsKinematic);

										//HERE: get the world position for the rigid body, from the skeleton.
										//Ogre::Vector3 bpPos = (mBodyParts[i]->mBone->getPosition() * kScale ) + mNode->getPosition();
										Ogre::Vector3 globalPos = mBodyParts[i]->mBone->convertLocalToWorldPosition(Ogre::Vector3::ZERO);
										Ogre::Quaternion globalQuat = mBodyParts[i]->mBone->convertLocalToWorldOrientation(Ogre::Quaternion::IDENTITY);
										//Ogre::Quaternion globalQuatAlt = Ogre::Quaternion(globalQuat.w,globalQuat.x,globalQuat.y,globalQuat.z);

										//strResult.str("");
										//strResult << "Bone " << mBodyParts[i]->mNodeName << " quat: x " <<
										//	globalQuat.x << " y " << globalQuat.y << " z " << globalQuat.z << " w " << globalQuat.w ;
										//gConsole->addOutput(strResult.str());

										globalPos *= kScale;
										Ogre::Quaternion nodeQuat = mNode->getOrientation();
										//Ogre::Quaternion finalQuat = globalQuat * nodeQuat; 
										Ogre::Quaternion finalQuat = nodeQuat * globalQuat; 

										Ogre::Matrix3 mat;
										nodeQuat.ToRotationMatrix(mat);
										Ogre::Vector3 finalPos = (mat * globalPos) + mNode->getPosition();
										
										mBodyParts[i]->mRB->setLinearPosition(finalPos);
										mBodyParts[i]->mRB->setAngularPosition(finalQuat);



										mBodyParts[i]->mRB->setup();


										//strResult.str("");
										//strResult << mBodyParts[i]->mNodeName  << " mass: " << mBodyParts[i]->mRB->getMass() * 1000;
										//gConsole->addOutput(strResult.str());

										if (mBodyParts[i]->mJointDataID )
										{
											mBodyParts[i]->mJoint = mPM->createJoint();
											mBodyParts[i]->mJoint->setJointType((physJointType)(strtol(resultSet->vRows[i]->vColumnValues[19],NULL,10)));
											mBodyParts[i]->mJoint->setTwistLimit(strtod(resultSet->vRows[i]->vColumnValues[20],NULL));
											mBodyParts[i]->mJoint->setSwingLimit(strtod(resultSet->vRows[i]->vColumnValues[21],NULL));
											mBodyParts[i]->mJoint->setSwingLimit2(strtod(resultSet->vRows[i]->vColumnValues[22],NULL));
											mBodyParts[i]->mJoint->setLocalAxis1(Ogre::Vector3(strtod(resultSet->vRows[i]->vColumnValues[23],NULL),
												strtod(resultSet->vRows[i]->vColumnValues[24],NULL),strtod(resultSet->vRows[i]->vColumnValues[25],NULL)));
											mBodyParts[i]->mJoint->setLocalNormal1(Ogre::Vector3(strtod(resultSet->vRows[i]->vColumnValues[26],NULL),
												strtod(resultSet->vRows[i]->vColumnValues[27],NULL),strtod(resultSet->vRows[i]->vColumnValues[28],NULL)));

											//TEMP: for the moment, set axes and normals to same values - or better yet use global axes.
											mBodyParts[i]->mJoint->setLocalAxis0(Ogre::Vector3(strtod(resultSet->vRows[i]->vColumnValues[23],NULL),
												strtod(resultSet->vRows[i]->vColumnValues[24],NULL),strtod(resultSet->vRows[i]->vColumnValues[25],NULL)));
											mBodyParts[i]->mJoint->setLocalNormal0(Ogre::Vector3(strtod(resultSet->vRows[i]->vColumnValues[26],NULL),
												strtod(resultSet->vRows[i]->vColumnValues[27],NULL),strtod(resultSet->vRows[i]->vColumnValues[28],NULL)));

											mBodyParts[i]->mJoint->setMaxForce(strtod(resultSet->vRows[i]->vColumnValues[29],NULL));
											mBodyParts[i]->mJoint->setMaxTorque(strtod(resultSet->vRows[i]->vColumnValues[30],NULL));

											//NOW, find parent bodypart...  should keep that info in flexbodypartdata, so if the actual skeleton
											//has other nodes in between this node and its physics parent node, we can track the right one here.
											//int possibleParent = mBodyParts[i]->mBone->getp  //...FFUUUU.... turns out getParent gives me a Node,
											//not a Bone... HMMM.
											Ogre::Node *possibleParent = mBodyParts[i]->mBone->getParent();
											int possibleParentIndex = dynamic_cast<Ogre::Bone*>(possibleParent)->getHandle();
											while (mIndexBones[possibleParentIndex] == -1)
											{
												possibleParent = possibleParent->getParent();
												possibleParentIndex = dynamic_cast<Ogre::Bone*>(possibleParent)->getHandle();
											}  //That just might do it...
											mBodyParts[i]->mParentBodyPart = mBodyParts[mIndexBones[possibleParentIndex]];

											//WARNING: this code is dependent on parents coming before children in the database query!  We should have
											//an index property we sort by to guarantee that.
											mBodyParts[i]->mJoint->setRB_B(mBodyParts[i]->mRB);
											if (mBodyParts[i]->mParentBodyPart)
												mBodyParts[i]->mJoint->setRB_A(mBodyParts[i]->mParentBodyPart->mRB);

											mBodyParts[i]->mJoint->setup();
											mBodyParts[i]->mRB->setJoint(mBodyParts[i]->mJoint);
											
											//strResult.str("");
											//strResult << "fxFlexBodyPartData found: " << mBodyParts[i]->mNodeName.c_str() << " joint type: " << 
											//	mBodyParts[i]->mJoint->getJointType() << ", id " << mBodyParts[i]->mFlexBodyPartDataID << 
											//	", local axis: " <<  strtod(resultSet->vRows[i]->vColumnValues[22],NULL) << ", " <<
											//	strtod(resultSet->vRows[i]->vColumnValues[23],NULL) << ", " << strtod(resultSet->vRows[i]->vColumnValues[24],NULL) <<
											//	" local normal: " << strtod(resultSet->vRows[i]->vColumnValues[25],NULL) << ", " <<
											//	strtod(resultSet->vRows[i]->vColumnValues[26],NULL) << ", " << strtod(resultSet->vRows[i]->vColumnValues[27],NULL) ;
											//	
											//gConsole->addOutput(strResult.str());
										}

										mNumBodyParts++;


										if (i==0) {
											mFirstNode = mBodyParts[i]->mNodeIndex;
										}
										mIndexBones[mBodyParts[i]->mNodeIndex] = i;

									}
									mPM->startPhysics();
								}
							}
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

void fxFlexBody::loadAnim(const char *anim)
{
		Ogre::AnimationState *as;
		Ogre::Skeleton *kSkeleton =  mEntity->getSkeleton();
		if (kSkeleton->hasAnimation(anim))//HERE: grab default anim out of flexbodydata
		{//list all available anims in sequences dropdown
			as = mEntity->getAnimationState(anim);
			as->setLoop(true);
			as->setEnabled(true);
			as->setTimePosition(0.0);
		}
		//Ogre::Animation *anim = kSkeleton->getAnimation("walkFull");
		//Animation::NodeTrackIterator tracks = anim->getNodeTrackIterator();
		//while (tracks.hasMoreElements())   // for every node track...
		//{
		//}

}

void fxFlexBody::examineEntity()
{
		Ogre::Vector3 vertex3,d;
		Ogre::Matrix4  kTransform;
		Ogre::Matrix3  kMat3;

		Ogre::Skeleton *kSkeleton =  mEntity->getSkeleton();
		gConsole->addOutput("Starting FLEXBODY entity/mesh research...");

		int numSubEnts = mEntity->getNumSubEntities();
		int numManualLODs = mEntity->getNumManualLodLevels();
		Ogre::MeshPtr meshPtr = mEntity->getMesh();
		//int numAnims = meshPtr->getNumAnimations();
		int numAnims = kSkeleton->getNumAnimations();
		int numLODMeshLevels = meshPtr->getNumLodLevels();
		int numSubMeshes = meshPtr->getNumSubMeshes();
		int numVerts = -1;

		
		//Ogre::AnimationState *animState = mEntity->getAnimationState("root");
		//if (animState)
		//	animState->setEnabled(true);

		std::ostringstream os;
		os << "SubEntities " << numSubEnts << ", manualLODS " << numManualLODs << ", anims " <<
			numAnims << ", Mesh LODS " << numLODMeshLevels << ", SubMeshes " << numSubMeshes ;
		gConsole->addOutput(os.str());

		for (int i=0;i<numAnims;i++)
		{
			Ogre::Animation *anim = kSkeleton->getAnimation(i);
			gConsole->addOutput(anim->getName().c_str());
		}

		for (int i=0;i<mNumBodyParts;i++)
		{
			//mEntity->getSkeleton()->sc
			Ogre::Vector3 kPos = mBodyParts[i]->mBone->getPosition() * 0.01;
			Ogre::Vector3 globalPos = mBodyParts[i]->mBone->convertLocalToWorldPosition(Ogre::Vector3::ZERO);
			Ogre::Quaternion globalQuat = mBodyParts[i]->mBone->convertLocalToWorldOrientation(Ogre::Quaternion::IDENTITY);

			Ogre::Quaternion nodeQuat = mNode->getOrientation();
			Ogre::Quaternion finalQuat = globalQuat * nodeQuat; 
			//globalPos *= kScale;
			os.str("");
			os << mBodyParts[i]->mBone->getName().c_str() << " localPos:  " << kPos.x << ", " << kPos.y << ", " << kPos.z ;
			gConsole->addOutput(os.str());
		}

		//Very good... making sense so far.  Don't worry about Entity->getNumManualLODs right now, 
		//looks like meshPtr->getNumLodLevels() is the one to watch.
		Ogre::Mesh::SubMeshIterator subMeshIter = meshPtr->getSubMeshIterator();
		//meshPtr->getSubMesh(0)->vertexData->vertexCount;
		int numBoneAssignments = 0;//meshPtr->getSubMesh(0)->getBoneAssignments().size();
		int numIndices = meshPtr->getSubMesh(0)->indexData->indexCount;
		bool sharedVertexData = meshPtr->getSubMesh(0)->useSharedVertices;


		//os.str("");
		//for (int i=0;i<numSubMeshes;i++)
		//{
		//	numVerts = meshPtr->getSubMesh(i)->vertexData->vertexCount;
		//	numIndices = meshPtr->getSubMesh(i)->indexData->indexCount;
		//	int numBoneAssignments = meshPtr->getSubMesh(i)->getBoneAssignments().size();
		//	os.str("");
		//	os << "submesh material: " << meshPtr->getSubMesh(i)->getMaterialName() << ", verts " << numVerts <<
		//		 ", indices " << numIndices << ", bone assignments " << numBoneAssignments ;
		//	gConsole->addOutput(os.str());
		//}

		if (sharedVertexData)
		{
			numVerts = meshPtr->sharedVertexData->vertexCount;
		}// else {
		//	numVerts = meshPtr->getSubMesh(0)->vertexData->vertexCount;
		//}

		//os.str("");
		//os << "NumVerts: " << numVerts << ", num indices: " << numIndices << ", shared vertices: " <<
		//	sharedVertexData << ", num bone assignments " << numBoneAssignments ;
		//gConsole->addOutput(os.str());

		
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


		//Ogre::HardwareVertexBufferSharedPtr vbuf= vertex_data->vertexBufferBinding->getBuffer(elem->getSource());
		//const unsigned int vSize = (unsigned int)vbuf->getVertexSize();
		//unsigned char* vertex =static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
		//size_t vertSize = declaration->getVertexSize(0);

		//mVecCtlPoints.push_back(Vector3(pFloat[0], pFloat[1], pFloat[2]));
		//pVert += vertSize;


		int numBones = 0;
		if (mEntity->getSkeleton())
			numBones = mEntity->getSkeleton()->getNumBones();
		else return;

		//os.str("");
		//os << "Bones: " << numBones << ", root node: " << mEntity->getSkeleton()->getRootBone()->getName();
		//gConsole->addOutput(os.str());
		//for (int i=0;i<numBones;i++)
		//{
		//	os.str("");
		//	os << mEntity->getSkeleton()->getBone(i)->getName() << ", handle " <<
		//		mEntity->getSkeleton()->getBone(i)->getHandle() << ", parent " ;
		//	if ( mEntity->getSkeleton()->getBone(i)->getParent() )
		//		os << mEntity->getSkeleton()->getBone(i)->getParent()->getName();
		//	Ogre::Vector3 kPos = mEntity->getSkeleton()->getBone(i)->getPosition() * 0.01;
		//	os << ", local pos (" << kPos.x << ", " << kPos.y << ", " << kPos.z ;
		//	gConsole->addOutput(os.str());
		//}
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
}


//
//void fxFlexBody::initPersistFields()
//{
//  Parent::initPersistFields();
//
//  //addField("InitialPosition",  TypeOgre::Vector3, 
//  //    Offset(mCurrPosition, fxFlexBody));
//  addField("InitialVelocity", TypeOgre::Vector3, 
//      Offset(mCurrVelocity, fxFlexBody));
//  addField("IsClientOnly",TypeBool,
//      Offset(mIsClientOnly, fxFlexBody));
//  addField("IsNoGravity",TypeBool,
//      Offset(mIsNoGravity, fxFlexBody));
//  addField("IsReturnToZero",TypeBool,
//      Offset(mIsReturnToZero, fxFlexBody));
//  addField("IsRendering",TypeBool,
//      Offset(mIsRendering, fxFlexBody));
//  addField("KeyframesFile",TypeStringFilename,
//      Offset(mKeyframesFile, fxFlexBody));
//  addField("PlaylistFile",TypeStringFilename,
//      Offset(mPlaylistFile, fxFlexBody));
//  addField("SpawnScript",TypeCommand,
//      Offset(mSpawnScript, fxFlexBody));
//  addField("PlaylistDelay",Typeint,
//      Offset(mPlaylistDelay, fxFlexBody));
//  addField("SkeletonName",TypeString,
//      Offset(mSkeletonName, fxFlexBody));
//  addField("Persona",TypeString,
//      Offset(mPersonaName, fxFlexBody));
//  addField("ActorName",TypeString,
//      Offset(mActorName, fxFlexBody));
//}
//
//unsigned int fxFlexBody::packUpdate(NetConnection* pConnection, unsigned int uiMask, BitStream* pBitStream)
//{
//	////Con::printf("flex body packing update %f %f %f",mCurrPosition.x,mCurrPosition.y,mCurrPosition.z);
//	unsigned int uiRetMask = Parent::packUpdate(pConnection, uiMask, pBitStream);
//	//if (pBitStream->writeFlag(uiMask & fxFlexBodyStateMask))
//	//  {
//	//    mathWrite(*pBitStream, mCurrPosition);
//	//    mathWrite(*pBitStream, mCurrVelocity);
//	//  }
//	if (pBitStream->writeFlag(uiMask & MoveMask))
//	{
//		if (!mIsPlayer)
//		{
//			Ogre::Vector3 pos;
//			Ogre::Matrix3 m = getTransform();
//			m.getColumn(3,&pos);
//			pBitStream->writeCompressedPoint(pos);
//
//			Ogre::Quaternion q(m);//Um, no stream functions for quats, only floats and vectors?
//			pBitStream->write(q.x);
//			pBitStream->write(q.y);
//			pBitStream->write(q.z);
//			pBitStream->write(q.w);
//		}
//	}
//	if (pBitStream->writeFlag(uiMask & fxFlexBodyMountMask))
//	{
//		pBitStream->write(mIsPhysActive);
//		pBitStream->write(mIsRecording);
//		pBitStream->write(mIsRendering);
//		pBitStream->write(mIsReturnToZero);
//		pBitStream->writeString(mShapeName);
//		pBitStream->write(mWeaponID);
//		pBitStream->write(mWeaponMountNode);
//		pBitStream->writeString(mWeaponNodeName);
//		pBitStream->write(mWeapon2ID);
//		pBitStream->write(mWeapon2MountNode);
//		pBitStream->writeString(mWeapon2NodeName);
//		pBitStream->write(mKeyframesFile);
//		pBitStream->write(mPlaylistFile);
//		pBitStream->write(mSpawnScript);
//		pBitStream->write(mPlaylistDelay);
//		pBitStream->write(mSkeletonID);
//		pBitStream->writeString(mSkeletonName);//Wait, why can I use write(string) above but feel
//		pBitStream->writeString(mPersonaName);//the need to use StringTableEntry here?
//		pBitStream->writeString(mActorName);
//		////Con::printf("flexbody packing actor name: %s",mActorName);
//		//mathWrite(*pBitStream, mCurrPosition);
//		//mathWrite(*pBitStream, mCurrVelocity);
//	}
//	return uiRetMask;
//}
//
//void fxFlexBody::unpackUpdate(NetConnection* pConnection, BitStream *pBitStream)
//{
//	Parent::unpackUpdate(pConnection, pBitStream);
//	//if (pBitStream->readFlag())
//	//  {
//	//    mathRead(*pBitStream, &mCurrPosition);
//	//    mathRead(*pBitStream, &mCurrVelocity);
//	//  }
//	if (pBitStream->readFlag())
//	{
//		if (!mIsPlayer)
//		{
//			Ogre::Vector3 pos,rot;
//			rot = Ogre::Vector3::ZERO;
//			pBitStream->readCompressedPoint(&pos);
//			//setPosition(pos);
//			Ogre::Quaternion q;//Um, no stream functions for quats, only floats and vectors?
//			pBitStream->read(&q.x);
//			pBitStream->read(&q.y);
//			pBitStream->read(&q.z);
//			pBitStream->read(&q.w);
//			Ogre::Matrix3 m;
//			q.setMatrix(&m);
//			m.setPosition(pos);
//			setTransform(m);
//		}
//	}
//	if (pBitStream->readFlag())
//	{
//		pBitStream->read(&mIsPhysActive);
//		pBitStream->read(&mIsRecording);
//		pBitStream->read(&mIsRendering);
//		pBitStream->read(&mIsReturnToZero);
//		mShapeName = pBitStream->readSTString();
//		pBitStream->read(&mWeaponID);
//		pBitStream->read(&mWeaponMountNode);
//		mWeaponNodeName = pBitStream->readSTString();
//		//if( Sim::findObject( mWeaponID, mWeapon ))
//		//	mountWeapon();
//		pBitStream->read(&mWeapon2ID);
//		pBitStream->read(&mWeapon2MountNode);
//		mWeaponNodeName = pBitStream->readSTString();
//		pBitStream->read(&mKeyframesFile);
//		pBitStream->read(&mPlaylistFile);
//		pBitStream->read(&mSpawnScript);
//		pBitStream->read(&mPlaylistDelay);
//		pBitStream->read(&mSkeletonID);
//		mSkeletonName = pBitStream->readSTString();
//		mPersonaName = pBitStream->readSTString();
//		mActorName = pBitStream->readSTString();
//		////Con::printf("client flexbody unpacking actor name: %s, skeleton id %d",mActorName,mSkeletonID);
//		//if( Sim::findObject( mWeapon2ID, mWeapon2 ))
//		//	mountWeapon2();
//		//mathRead(*pBitStream, &mCurrPosition);
//		//mathRead(*pBitStream, &mCurrVelocity);
//		//Ogre::Matrix3 kTransform = getTransform();
//		//kTransform.setPosition(Ogre::Vector3(mCurrPosition.x,mCurrPosition.y,mCurrPosition.z));
//		//setTransform(kTransform);
//	}
//	////Con::printf("flex body unpacking update.  %f %f %f",mCurrPosition.x,mCurrPosition.y,mCurrPosition.z);
//}
//
//bool fxFlexBody::onAdd()
//{
//	bool kIsServer = isServerObject();
//
//	if (!mIsPlayer) {
//		if (mIsClientOnly) mNetFlags.set(IsGhost);
//		else {
//			mNetFlags.set(Ghostable);
//			setScopeAlways();
//		}
//	} else mNetFlags.set(Ghostable);
//
//	if(!Parent::onAdd() || !mDataBlock)
//		return false;
//
//	mLifetimeMS = mDataBlock->mLifetimeMS;
//
//	if (mDataBlock->mSleepThreshold)
//		mSleepThreshold = mDataBlock->mSleepThreshold;
//
//	if (mDataBlock->mMeshExcludes)
//	{
//		if (strlen(mDataBlock->mMeshExcludes)>0)
//		{
//			char *bufp;
//			char buf[255];
//			unsigned int m;
//
//			//Con::errorf("my mesh excludes: %s",mDataBlock->mMeshExcludes);
//			strcpy(buf,mDataBlock->mMeshExcludes);
//			bufp = strtok(buf,",");
//			sscanf(bufp,"%d",&m);
//			mMeshExcludes[mNumMeshExcludes++] = m;
//			bufp = strtok(NULL,",");
//			while (bufp)
//			{
//				sscanf(bufp,"%d",&m);
//				mMeshExcludes[mNumMeshExcludes++] = m;
//				bufp = strtok(NULL,",");
//			}
//		}
//	}
//
//	const String myPath = mShapeInstance->mShapeResource.getPath().getPath();
//	const String myFileName = mShapeInstance->mShapeResource.getPath().getFileName();
//
//	//char shortname[255];
//	//strncpy(shortname,myFileName.c_str(),
//	//	strlen(mShapeInstance->getShape()->mSourceResource->name)-4);//remove ".dts"
//	//shortname[strlen(mShapeInstance->getShape()->mSourceResource->name)-4] = '\0';
//
//	mShapeName = StringTable->insert(myFileName.c_str());
//	
//	//Obsolete, handle with relaxType now.
//	if ((!strncmp(mShapeName,"Tree",4)) ||
//		(!strncmp(mShapeName,"tree",4)))
//		mEntitySubType = PHYS_SUB_FLEX_TREE; //TREE is special because first bodypart is kinematic, locked to ground.
//	else 
//		mEntitySubType = PHYS_SUB_FLEX_BIPED;//or PHYS_SUB_QUADRUPED, MULTIPED, whatever, count legs later.
//
//
//	if (!mIsPlayer) 
//		addToScene();
//
//	mCurrPosition = getTransform().getPosition();
//
//	if (kIsServer) 
//		//setMaskBits(MoveMask);
//
//	//Con::errorf("making a flexbody at: %f %f %f, isServer %d",mCurrPosition.x,mCurrPosition.y,mCurrPosition.z,kIsServer);
//	
//	mResetPoint = mCurrPosition;
//	mInitialPosition = mCurrPosition;//FIX: is one of these redundant?
//
//	Ogre::Matrix3 m = getTransform();
//	Ogre::Quaternion q(m);
//	mInitialOrientation = q;
//
//	if (kIsServer) return true;
//
//	mPM = physManagerCommon::getPM();
//
//	dynamic_cast<nxPhysManager*>(mPM)->mNumFlexbodies++;
//	//if (mDataBlock->mDensity) mDensity = mDataBlock->mDensity;
//	mActorGroup = mPM->getNextActorGroup();
//	getBaseRot();
//	getMesh();
//	getChildNodes();//have to do this after getMesh, so that all parts exist.
//	getNamedNodes();
//	setupOrderNodes();//set up mOrderNodes, to translate from bodypart order to shape node order
//
//	if (mDataBlock->mGA && mDataBlock->mActionUserData) 
//		setupGA();
//
//	if (!mIsClientOnly) {
//		if (isClientObject())
//			//Con::errorf("Adding client fxFlexBody: ID %d",getId());
//		else 
//			//Con::errorf("Adding server fxFlexBody: ID %d",getId());
//	}
//	mIsAnimating = true;//(call startAnimating() to set this.)
//	//saveResets();
//	setKinematic();
//	setPhysActive(true);
//
//	//////////  DB stuff /////////////
//	loadDatabaseIds();
//	//loadkeyframeSets(int scene_id);//(Making this function a lot smarter, to get this logic out of onAdd().)
//	//loadPlaylists(int scene_id);//whoops, do these later from script in selectScene, not at init time.
//	//backupSequenceData();
//	//////////////////////////////////
//
//	//Con::executef(this, "onAdd", scriptThis());
//	if (strlen(mSpawnScript.c_str()))
//		//Con::executef(this, mSpawnScript.c_str(), scriptThis());
//	
//	getShapeInstance()->mFlexBody = this;
//
//	return true;
//}

	//EXAMPLE CODE:  This is how you create a thread (for a subset of bodyparts in this case.)
	//int headSeq = mShapeInstance->getShape()->findSequence("head");
	//if (headSeq != -1) {
	//   mHeadVThread = mShapeInstance->addThread();
	//   mShapeInstance->setSequence(mHeadVThread,headSeq,0);
	//   mShapeInstance->setTimeScale(mHeadVThread,0);
	//}
	//else
	//   mHeadVThread = 0;

	////////  Debug zone

	//Ogre::Quaternion q1(0,0,0,1);
	//Ogre::Quaternion q2(0.5,0.5,0.5,0.5);
	//Ogre::Matrix3 m1,m2;
	//q1.setMatrix(&m1);
	//q2.setMatrix(&m2);
	//m1.mul(m2);
	////Con::errorf("resulting q1: %f %f %f %f",q1.x,q1.y,q1.z,q1.w);

	//  TSShape *kShape = mShapeInstance->getShape(); 
	//for (unsigned int i=0;i<kShape->nodes.size();i++)
	//{	
	//	//Con::errorf("%s node %d: %3.2f %3.2f %3.2f",mShapeName,i,
	//		kShape->defaultTranslations[i].x,kShape->defaultTranslations[i].y,kShape->defaultTranslations[i].z);
	//}
	//for (unsigned int i=0;i<mNumBodyParts;i++)
	//{	
	//	//Con::errorf("%s kbodypart %d: %3.2f %3.2f %3.2f",mShapeName,i,
	//		mBodyParts[i]->getPosition().x,mBodyParts[i]->getPosition().y,mBodyParts[i]->getPosition().z);
	//}


void fxFlexBody::setupOrderNodes()
{
	unsigned int sortList[MAX_FLEX_NODES];

	for (unsigned int i = 0; i < mNumBodyParts; i++)
	{
		sortList[i] = mBodyParts[i]->mNodeIndex;
	}

	for (unsigned int i = 0; i < mNumBodyParts; i++)
	{
		for (unsigned int j = 0; j < mNumBodyParts-1; j++)
		{
			int temp = -1;
			if (sortList[j] > sortList[j+1]) 
			{
				temp = sortList[j];
				sortList[j] = sortList[j+1];
				sortList[j+1] = temp;
				
			}
		}
	}
	
	for (unsigned int i=0; i<mNumBodyParts; i++)
		for (unsigned int j=0; j<mNumBodyParts; j++)
			if (mBodyParts[j]->mNodeIndex == sortList[i]) 
			{
				mOrderNodes[i] = j;
			}

}

void fxFlexBody::loadDatabaseIds()
{
	//SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(mPM)->getSQL();
	//if (sql)
	//{
	//	if (sql->OpenDatabase("EcstasyMotion.db"))
	//	{
	//		char id_query[512],insert_query[512];
	//		int result,fxFlexBodyData_id;
	//		sqlite_resultset *resultSet;

	//		if (strlen(mDataBlock->mSkeletonName) > 0)
	//		{
	//			sprintf(id_query,"SELECT id FROM skeleton WHERE name = \"%s\";",mDataBlock->mSkeletonName);
	//			result = sql->ExecuteSQL(id_query);
	//			resultSet = sql->GetResultSet(result);
	//			if (resultSet->iNumRows == 1)
	//				mSkeletonID = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);
	//		}
	//		if (strlen(mPersonaName.c_str()) > 0)
	//		{
	//			sprintf(id_query,"SELECT id FROM persona WHERE name = \"%s\";",mPersonaName);
	//			result = sql->ExecuteSQL(id_query);
	//			resultSet = sql->GetResultSet(result);
	//			if (resultSet->iNumRows == 1)
	//				mPersonaID = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);
	//		}
	//		if ((mSkeletonID>0)&&(mPersonaID>0))
	//			//Con::printf("fxFlexBody found a persona %d %s and a skeleton %d",mPersonaID,mPersonaName,mSkeletonID);

	//		if (strlen(mActorName.c_str()) > 0)
	//		{
	//			sprintf(id_query,"SELECT id FROM actor WHERE name = \"%s\";",mActorName);
	//			result = sql->ExecuteSQL(id_query);
	//			resultSet = sql->GetResultSet(result);
	//			if (resultSet->iNumRows == 1)
	//				mActorID = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);
	//			else if (resultSet->iNumRows == 0)
	//			{
	//				sprintf(insert_query,"INSERT INTO actor (name,fxFlexBodyData_id,persona_id) values ('%s',%d,%d);",mActorName,mDataBlock->mDBId,mPersonaID);
	//				result = sql->ExecuteSQL(insert_query);
	//				result = sql->ExecuteSQL(id_query);
	//				resultSet = sql->GetResultSet(result);
	//				if (resultSet->iNumRows == 1)
	//					mActorID = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);
	//				else //Con::errorf("Failed to insert new actor %s",mActorName);
	//			}
	//		} else if (mDataBlock->mDBId) {
	//			sprintf(id_query,"SELECT id,name FROM actor WHERE name LIKE 'Actor_%%';");
	//			result = sql->ExecuteSQL(id_query);
	//			resultSet = sql->GetResultSet(result);
	//			int maxActorNameNum = 0;
	//			for (unsigned int i=0;i<resultSet->iNumRows;i++)
	//			{
	//				String actorName(resultSet->vRows[i]->vColumnValues[1]);
	//				String actorName_number = actorName.substr(actorName.find('_')+1,actorName.length()-6);
	//				int actorNameNum = strtol(actorName_number.c_str(),NULL,10);
	//				//Con::printf("found actor name: %s",actorName.c_str());
	//				if (actorNameNum > maxActorNameNum)
	//					maxActorNameNum = actorNameNum;
	//			}
	//			String newActorName("Actor_");
	//			newActorName += newActorName.ToString(maxActorNameNum+1);
	//			//Con::printf("isServer %d new actor name: %s",isServerObject(),newActorName.c_str());
	//			sprintf(insert_query,"INSERT INTO actor (name,fxFlexBodyData_id,persona_id) values ('%s',%d,%d);",newActorName.c_str(),mDataBlock->mDBId,mPersonaID);
	//			result = sql->ExecuteSQL(insert_query);
	//			sprintf(id_query,"SELECT id,name FROM actor WHERE name = '%s';",newActorName.c_str());
	//			
	//			result = sql->ExecuteSQL(id_query);
	//			resultSet = sql->GetResultSet(result);																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																															
	//			if (resultSet->iNumRows == 1) {
	//				mActorID = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);
	//				mActorName = newActorName.c_str();
	//				//setNameChangeAllowed(true);//this probably doesn't help - we're on the client.
	//				////Con::executef(this, "setNewActorName", scriptThis(), mActorName);
	//				//Con::executef(this,"schedule","400","setNewActorName", mActorName);
	//				////setMaskBits(fxFlexBodyMountMask);//this probably doesn't help - we're on the client.
	//			} else //Con::errorf("Failed to insert new actor %s",newActorName.c_str());
	//		}
	//		sql->CloseDatabase();
	//		delete sql;
	//	}
	//}
}

void fxFlexBody::loadKeyframeSets(int scene_id)
{
	
	////HERE: first check for existence of database keyframes, then if you don't find any for this character and scene,
	////look for a preferred .keyframes file in the flexbody definition, and if that also fails, look for a default.keyframes file.
	//TSShape *kShape = getShapeInstance()->getShape();
	//const Ogre::String myFullPath = getShapeInstance()->mShapeResource.getPath().getFullPath();
	//TSShapeConstructor* ctor = TSShapeConstructor::findShapeConstructor( myFullPath );

	////First, before anything else, we need to drop and reload all our sequences and drop all ultraframesets.
	//restoreSequences();
	////while (mUltraframeSets.size()>0)//HERE: don't we need to delete mShape->sequenceBackups as well?
	////	mUltraframeSets.erase((unsigned int)0);
	////while (kShape->sequenceBackups.size()>0)//but wait, don't we need to copy them back to the sequence first??
	////	kShape->sequenceBackups.erase((unsigned int)0);

	////Hmm, maybe only do this IF we are not doing this for the first time, on load.  Experiencing
	////if (mLoadedKeyframeSets)// some serious lag.
	////{
	//	////Con::printf("actor %d reloading sequences",mActorID); 
	////	while (kShape->sequences.size()>1) 
	////		kShape->dropSequence(0);
	////	kShape->dropSequence(0);//dropSequence hangs if you do above loop all the way down to zero.
	////}//Hm, we were never setting mLoadedKeyframeSets to true anyway, so this is always false.

	////reloadSequences(); 

	////if (ctor)//PROBLEM:  this clears the ChangeSet, and we are losing our earlier addSequences.
	////	ctor->_onLoad( getShapeInstance()->mShapeResource );
	////backupSequenceData();
	////With any luck at all, that should do it.  From here on we can safely add ultraframesets for each sequence that needs them.
	////(Ignoring problem of backupRotations etc. for now - backing up entire set whether or not they have morph frames, ugly, expensive.)

	//mPM = physManagerCommon::getPM();
	//Ogre::String sceneName = dynamic_cast<nxPhysManager*>(mPM)->mSceneName;
	//SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(mPM)->getSQL();
	//bool foundKeyframeSet = false;
	//if (sql)
	//{
	//	if (sql->OpenDatabase("EcstasyMotion.db"))
	//	{
	//		char sequence_id_query[512],keyframe_set_id_query[512],keyframes_query[512],insert_query[512];
	//		int result,result2;
	//		sqlite_resultset *resultSet,*resultSet2;

	//		//Okay, first SQL optimization: instead of doing a query into actorKeyframeSet for every actor and every sequence on that shape, 
	//		//let's do a query by actor that returns all the sequences for this scene.

	//		int sequence_id=0,keyframe_set_id=0;
	//		sprintf(keyframe_set_id_query,"SELECT id FROM keyframeSet WHERE actor_id=%d AND scene_id=%d;",
	//			mActorID,scene_id);
	//		result = sql->ExecuteSQL(keyframe_set_id_query);
	//		if (result==0)
	//		{
	//			//Con::errorf("loadKeyframeSets could not find keyframe set for actor %d and scene %d",mActorID,scene_id);
	//			sql->CloseDatabase();
	//			return;
	//		}
	//		resultSet = sql->GetResultSet(result);
	//		//Con::printf("%s, results %d",keyframe_set_id_query,resultSet->iNumRows);
	//		for (unsigned int i=0;i<resultSet->iNumRows;i++)
	//		{
	//			int type,frame,node,seq;
	//			Ogre::Vector3 value;
	//			keyframe_set_id = strtol(resultSet->vRows[i]->vColumnValues[0]);
	//			//HERE: should probably get all keyframes for all keyframesets affecting this actor/scene,
	//			//in one query up above instead of one per sequence here.
	//			sprintf(keyframes_query,"SELECT k.type,k.frame,k.node,k.value_x,k.value_y,k.value_z,s.name FROM keyframe k JOIN keyframeSet m ON m.id = k.keyframe_set_id JOIN sequence s ON s.id = m.sequence_id WHERE keyframe_set_id=%d;",
	//				keyframe_set_id);
	//			result2 = sql->ExecuteSQL(keyframes_query);
	//			resultSet2 = sql->GetResultSet(result2);
	//			////Con::printf("query %s, results %d",keyframes_query,resultSet2->iNumRows);
	//			for (unsigned int j=0;j<resultSet2->iNumRows;j++)
	//			{
	//				type = strtol(resultSet2->vRows[j]->vColumnValues[0],NULL,10);
	//				frame = strtol(resultSet2->vRows[j]->vColumnValues[1],NULL,10);
	//				node = strtol(resultSet2->vRows[j]->vColumnValues[2],NULL,10);
	//				value.x = strtod(resultSet2->vRows[j]->vColumnValues[3],NULL);
	//				value.y = strtod(resultSet2->vRows[j]->vColumnValues[4],NULL);
	//				value.z = strtod(resultSet2->vRows[j]->vColumnValues[5],NULL);
	//				seq = kShape->findSequence(resultSet2->vRows[j]->vColumnValues[6]);
	//				if (seq >= 0)
	//					addUltraframeNoInsert(seq,frame,node,type,0,value);
	//				else
	//					//Con::errorf("Sequence %s not loaded on actor %d, cannot add ultraframe",resultSet2->vRows[j]->vColumnValues[6],mActorID);
	//			}		
	//			foundKeyframeSet = true;
	//		}
	//		sql->CloseDatabase();
	//		delete sql;
	//	}
	//}

	//if (foundKeyframeSet)
	//	return;
}

	//Here and below: this is only to grab up the obsolete .keyframes files in case any current users
	//have them.  Remove this in later versions.
	//if (strlen(mKeyframesFile.c_str()))
	//{
	//	loadUltraframes(mKeyframesFile.c_str());
	//} else {
	//	String ultraframesFile;
	//	ultraframesFile = getShapeInstance()->getShapeResource()->getPath().getPath();
	//	String myFileName = getShapeInstance()->getShapeResource()->getPath().getFileName();
	//	myFileName.insert(0,"/");
	//	myFileName.insert(myFileName.length(),".default.keyframes");
	//	ultraframesFile.insert(ultraframesFile.length(),myFileName.c_str());
	//	loadUltraframes(ultraframesFile.c_str());
	//}


			/*
//OLD WAY: this was tremendously slow, because I was looping through all sequences for all actors, and querying the database up to four times 
//per sequence.  Now I'm querying once to get a list of active morphs (actorKeyframeSet), for each of those active morphs querying once to
//get the keyframes, and I'm done.  Giving up on looking for default morphs for this actor, scene, sequence in keyframeSets table, because
//that's what actorKeyframeSets are for, trust it to be there.

			int sequence_id=0,keyframe_set_id=0;
			for (unsigned int i=0;i<kShape->sequences.size();i++)
			{
				TSShape::Sequence *kSeq = &(kShape->sequences[i]);
				String seqName = kShape->getName(kSeq->nameIndex); 

				//FIX: save this for each tsshape when you add the sequence.  For now looking it up from db every time we change scenes, slower.
				sprintf(sequence_id_query,"SELECT id FROM sequence WHERE name='%s' AND skeleton_id=%d",seqName.c_str(),mSkeletonID);
				result = sql->ExecuteSQL(sequence_id_query);
				resultSet = sql->GetResultSet(result);
				if (resultSet->iNumRows == 1)
				{
					sequence_id = strtol(resultSet->vRows[0]->vColumnValues[0]);
					//Now, we need to find out if we have a current sequence morph for this sequence, actor, scene.
					keyframe_set_id = 0;
					sprintf(keyframe_set_id_query,"SELECT keyframe_set_id FROM actorKeyframeSet WHERE sequence_id=%d AND actor_id=%d AND scene_id=%d",
						sequence_id,mActorID,scene_id);
					result = sql->ExecuteSQL(keyframe_set_id_query);
					resultSet = sql->GetResultSet(result);
					if (resultSet->iNumRows == 1)
						keyframe_set_id = strtol(resultSet->vRows[0]->vColumnValues[0]);
					else if (resultSet->iNumRows == 0) {					
						char morphName[512];
						//Failing that, check to see if we have a default sequence morph for this sequence, actor, scene.
						sprintf(morphName,"%s.%s.%s",sceneName.c_str(),mActorName,kShape->getName(kSeq->nameIndex).c_str());
						//Now, see if we have a default sequence morph for this scene, this actor, this sequence.
						sprintf(keyframe_set_id_query,"SELECT id FROM keyframeSet WHERE actor_id=%d AND sequence_id=%d AND scene_id=%d AND name='%s'",
							mActorID,sequence_id,scene_id,morphName);
						result = sql->ExecuteSQL(keyframe_set_id_query);
						resultSet2 = sql->GetResultSet(result);
						if (resultSet2->iNumRows == 1)
						{
							keyframe_set_id = strtol(resultSet2->vRows[0]->vColumnValues[0]);
							sprintf(insert_query,"INSERT INTO actorKeyframeSet (sequence_id,actor_id,scene_id,keyframe_set_id) VALUES (%d,%d,%d,%d);",
								sequence_id,mActorID,scene_id,keyframe_set_id);
							result = sql->ExecuteSQL(insert_query);
						}
					}
					if (keyframe_set_id)
					{
						int type,frame,node;
						Ogre::Vector3 value;

						sprintf(keyframes_query,"SELECT type,frame,node,value_x,value_y,value_z FROM keyframeSetKeyframe WHERE keyframe_set_id=%d;",
							keyframe_set_id);
						result = sql->ExecuteSQL(keyframes_query);
						resultSet = sql->GetResultSet(result);
						//if (resultSet->iNumRows > 0)
						//{
						//First, add an ultraframeset for this sequence.  Wait, unnecessary, addultraframe does that if it doesn't find one.
						//}
						for (unsigned int j=0;j<resultSet->iNumRows;j++)
						{
							type = strtol(resultSet->vRows[j]->vColumnValues[0]);
							frame = strtol(resultSet->vRows[j]->vColumnValues[1]);
							node = strtol(resultSet->vRows[j]->vColumnValues[2]);
							value.x = strtod(resultSet->vRows[j]->vColumnValues[3]);
							value.y = strtod(resultSet->vRows[j]->vColumnValues[4]);
							value.z = strtod(resultSet->vRows[j]->vColumnValues[5]);
							addUltraframeNoInsert(i,frame,node,type,0,value);
						}		
						foundDBMorph = true;
						//Con::errorf("Added sequence morph frames for sequence %d  %s",sequence_id,seqName.c_str());
					}
				}
			}*/
			//sql->ClearResultSet(result);

void fxFlexBody::loadPlaylist(int scene_id)
{
	//TSShape *kShape = getShapeInstance()->getShape();
	//mPM = physManagerCommon::getPM();
	//String sceneName = dynamic_cast<nxPhysManager*>(mPM)->mSceneName;
	//SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(mPM)->getSQL();
	//bool foundDBPlaylist = false;

	////HERE: Possible optimization would be to move queries up to level above this function, 
	////so you could query all actor_playlists and all playlistSequences for all bots in this scene.
	////Then loop through all the playlistSequences you get back, adding them by bot.
	//clearPlaylist();
	//if (sql)
	//{
	//	if (sql->OpenDatabase("EcstasyMotion.db"))
	//	{
	//		char playlist_id_query[512],playlist_sequence_query[512],sequence_name_query[512];
	//		int result,seq,repeats,sequence_id=0,playlist_id=0;
	//		float speed;
	//		sqlite_resultset *resultSet,*resultSet2;

	//		sprintf(playlist_id_query,"SELECT playlist_id FROM actorPlaylist WHERE actor_id=%d AND scene_id=%d;",
	//			mActorID,scene_id);
	//		result = sql->ExecuteSQL(playlist_id_query);
	//		resultSet = sql->GetResultSet(result);
	//		//Con::printf("looking for playlist...");
	//		if (resultSet->iNumRows == 1)//don't allow multiple actorPlaylists for same actor, same scene
	//		{
	//			//Con::printf("actor:  %d  playlist %d ",mActorID,strtol(resultSet->vRows[0]->vColumnValues[0]));
	//			playlist_id = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);
	//			setPlaylist(playlist_id);
	//			//HERE: add a playlist
	//			sprintf(playlist_sequence_query,"%s %s %s %s %d %s",
	//								"SELECT pS.sequence_id,pS.repeats,pS.speed,s.name",
	//								"FROM playlistSequence pS",
	//								"JOIN sequence s ON s.id=pS.sequence_id",
	//								"WHERE playlist_id =",playlist_id,
	//								"ORDER BY sequence_order;");
	//			result = sql->ExecuteSQL(playlist_sequence_query);
	//			resultSet = sql->GetResultSet(result);

	//			//Con::printf("Found one playlist and %d sequences for actor %d.",resultSet->iNumRows,mActorID);
	//			for (unsigned int i=0;i<resultSet->iNumRows;i++)
	//			{
	//				//HERE: add a playlist sequence
	//				sequence_id = strtol(resultSet->vRows[i]->vColumnValues[0],NULL,10);
	//				repeats = strtol(resultSet->vRows[i]->vColumnValues[1],NULL,10);
	//				speed = strtod(resultSet->vRows[i]->vColumnValues[2],NULL);
	//				String seqName = resultSet->vRows[i]->vColumnValues[3];
	//				seq = kShape->findSequence(seqName.c_str());

	//				//sprintf(sequence_name_query,"SELECT name FROM sequence WHERE id=%d;",sequence_id);
	//				//result = sql->ExecuteSQL(sequence_name_query);
	//				//resultSet2 = sql->GetResultSet(result);
	//				//if (resultSet2->iNumRows == 1)
	//				//{
	//					
	//					
	//				//}
	//				if ((seq>=0)&&(seq<kShape->sequences.size())&&(repeats>0)&&(speed!=0))
	//					addPlaylistSeq(seq,repeats,speed);

	//				foundDBPlaylist = true;
	//			}
	//		}
	//		sql->CloseDatabase();
	//		delete sql;
	//	}
	//}

	//if (foundDBPlaylist) 
	//{

	//	TSShapeInstance *kSI = getShapeInstance();
	//	playSeq(0,mPlaylist[0].seq);
	//	TSThread *th = kSI->getThread(0);
	//	kSI->setTimeScale(th,0.0);
	//	return;
	//}

	////HERE: check for playlist items in database, for this actor and this scene, and failing that look for files.
	//if (strlen(mPlaylistFile.c_str()))
	//{
	//	loadPlaylist(mPlaylistFile.c_str());
	//} else {
	//	String playlistFile;
	//	playlistFile = getShapeInstance()->getShapeResource()->getPath().getPath();
	//	String myFileName = getShapeInstance()->getShapeResource()->getPath().getFileName();
	//	myFileName.insert(0,"/");
	//	myFileName.insert(myFileName.length(),".default.playlist");
	//	playlistFile.insert(playlistFile.length(),myFileName.c_str());
	//	loadPlaylist(playlistFile.c_str());
	//}
}

void fxFlexBody::loadPlaylistById(int playlist_id)
{
	//TSShape *kShape = getShapeInstance()->getShape();
	//mPM = physManagerCommon::getPM();
	//SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(mPM)->getSQL();

	//clearPlaylist();
	//if (sql)
	//{
	//	if (sql->OpenDatabase("EcstasyMotion.db"))
	//	{
	//		char playlist_sequence_query[512],sequence_name_query[512];
	//		int result,seq,repeats,sequence_id=0;
	//		float speed;
	//		sqlite_resultset *resultSet,*resultSet2,*result3;

	//		sprintf(playlist_sequence_query,"SELECT sequence_id,repeats,speed,sequence_order FROM playlistSequence WHERE playlist_id=%d ORDER BY sequence_order ASC;",
	//			playlist_id);
	//		result = sql->ExecuteSQL(playlist_sequence_query);
	//		resultSet = sql->GetResultSet(result);
	//		for (unsigned int i=0;i<resultSet->iNumRows;i++)
	//		{
	//			//HERE: add a playlist sequence
	//			sequence_id = strtol(resultSet->vRows[i]->vColumnValues[0],NULL,10);
	//			repeats = strtol(resultSet->vRows[i]->vColumnValues[1],NULL,10);
	//			speed = strtod(resultSet->vRows[i]->vColumnValues[2],NULL);

	//			sprintf(sequence_name_query,"SELECT name,filename FROM sequence WHERE id=%d;",sequence_id);
	//			result = sql->ExecuteSQL(sequence_name_query);
	//			resultSet2 = sql->GetResultSet(result);
	//			if (resultSet2->iNumRows == 1)
	//			{
	//				String seqName = resultSet2->vRows[0]->vColumnValues[0];
	//				String fileName = resultSet2->vRows[0]->vColumnValues[1];
	//				seq = kShape->findSequence(seqName.c_str());
	//				if ((seq<0)||(seq>=kShape->sequences.size()))
	//				{
	//					loadDsq(fileName.c_str());
	//					seq = kShape->findSequence(seqName.c_str());
	//					if (seq<0)
	//					{//Looks like we need to add the name to TSShape names array as well.
	//						seq = kShape->sequences.size()-1;
	//						if (seq>=0)
	//						{
	//							int nameIndex = kShape->names.size();
	//							kShape->names.increment();
	//							kShape->names.last() = seqName;
	//							kShape->sequences[seq].nameIndex = nameIndex;
	//						}
	//					}
	//				}
	//			}
	//			if ((seq>=0)&&(seq<kShape->sequences.size()))
	//				addPlaylistSeq(seq,repeats,speed);
	//		}
	//		
	//		mPlaylistID = playlist_id;
	//		//Con::printf("%s Setting playlist id: %d",mActorName,mPlaylistID);
	//		sql->CloseDatabase();
	//		delete sql;
	//	}
	//}
}

//void fxFlexBody::onRemove()
//{
//	
//	//Con::executef(this, "onRemove", scriptThis());
//
//	if (mTriggerActor) { 
//		//mTriggerActor = NULL;
//		//mPM->removeFlexBody(this);
//	}//just in case it didn't get done in clearKinematic
//
//	//if (isServerObject()) gServerProcessList.getRBM()->removeRigidBody(this);
//	//else physManagerCommon::getRBM()->removeRigidBody(this);
//	for (unsigned int i=0;i<mNumBodyParts;i++) mBodyParts[i]->onRemove();
//
//	//FIX! (move to physManager somewhere)
//	//((nxPhysManager *)mPM)->getScene()->releaseActor(*mTriggerActor);
//	//mTriggerActor = NULL;
//
//	// Parent classes' methods take care of everything else
//	scriptOnRemove();
//
//	removeFromScene();
//
//	//Parent::onRemove();
//}

//bool fxFlexBody::onNewDataBlock(GameBaseData* pGameBaseData, bool reload)
//{
//  mDataBlock = dynamic_cast<fxFlexBodyData*>(pGameBaseData);
//  if (!mDataBlock || !Parent::onNewDataBlock(pGameBaseData,reload))
//    {
//      return false;
//    }
//
//  // Have parent class do the rest
//  scriptOnNewDataBlock();
//  
//  return true;
//}

//void fxFlexBody::advanceTime(float fTimeDelta)
//{
	//if(0) {
	////if ((mClearIsAnimating)||(mClearBodyAnimating)) {
	//	for (unsigned int i=0;i<mNumBodyParts;i++)
	//		if (!(mBodyParts[i]->mIsKinematic))
	//			mBodyParts[i]->updatePositionFromRB();

	//	updateNodes();
	//}

	//const String myFileName = mShapeInstance->mShapeResource.getPath().getFileName();
	//if (strstr(myFileName.c_str(),"ninja"))
	//{
	//	Ogre::Vector3 rootpos = mShapeInstance->mNodeTransforms[1].getPosition();
	//	Ogre::Vector3 spinepos = mShapeInstance->mNodeTransforms[2].getPosition();
	//	//Con::printf("rootpos %3.2f %3.2f %3.2f, spinepos  %3.2f %3.2f %3.2f",
	//		rootpos.x,rootpos.y,rootpos.z,spinepos.x,spinepos.y,spinepos.z);
	//	if (rootpos.length() == 0.0)
	//		//Con::errorf("uh oh, root node frozen again!");
	//}

	//Parent::advanceTime(fTimeDelta);
//}

//void fxFlexBody::setTransform(const Ogre::Matrix3& kTransformMatrix)
//{
//  Parent::setTransform(kTransformMatrix);
//}

//void fxFlexBody::renderDebug()//(SceneState* pState, RenderInst *ri)
//{
	//Parent::renderObject(pState, ri); 

	//if (mPM->getDebugRender())
	//	for (unsigned int i=0;i<mNumBodyParts;i++) mBodyParts[i]->renderDebug();

//}

//bool fxFlexBody::prepRenderImage(SceneState* state, const unsigned int stateKey,
//                                const unsigned int startZone, const bool modifyBaseState)
//{
//	RenderInst *ri;
//	if (mPM->getDebugRender()) 
//	{
//		//ri = gRenderInstManager.allocInst();
//		//ri->obj = this;
//		//ri->state = state;
//		//ri->type = RenderInstManager::RIT_NxDebug;
//		//gRenderInstManager.addInst( ri );
//		renderDebug();
//
//	}
//
//	//if(!mIsPlayer)// && getDataBlock()->shadowEnable)
//	//{
//	//	ri = gRenderInstManager.allocInst();         
//	//	ri->obj = this;
//	//	ri->state = state;
//	//	ri->type = RenderInstManager::RIT_Shadow;
//	//	ri->transFlags = 2;
//	//	gRenderInstManager.addInst(ri);
//	//}
//	
//	if ((mIsRendering)&&(Parent::prepRenderImage(state,stateKey,startZone,modifyBaseState)))
//		return true;
//	else return false;
//}
//
//void fxFlexBody::processTick(const Move* pMove)
//{
//	
//   bool kIsServer = isServerObject();
//   //if (kIsServer) //setMaskBits(MoveMask);//HERE:  NO!! DO NOT do this, this was source of a full day's confusion.
//   //This forces (0,0,0) over as the position, since the client is handling all of the motion when we're a 
//   //clientside flexbody.  
//
//   Parent::processTick(pMove);//before update position?
//
//   //if (mShapeInstance->getShape()->nodes.size()==29) {
//      //Ogre::Vector3 pos = mBodyParts[0]->mCurrPosition;
//      ////Con::printf("position %3.2f %3.2f %3.2f",pos.x,pos.y,pos.z);
//   //}
//   //mCurrMS += TickMs;
//   //mCurrTick++;
//
//   if (mIsRecording)
//	   recordTick();
//
//   if  (( mCurrMS > mLifetimeMS )&&( mLifetimeMS > 0 ))
//   {
//	  //for (unsigned int i=0;i<mNumBodyParts;i++) mBodyParts[i]->onRemove();
//      onRemove();
//   }
//
//	//in case we died on the server, but didn't notice
//	if ((getDamageLevel()==1.0)&&(!mIsCorpse)) 
//		stopAnimating();
//
//   //if (mIsPlayer) {
////	mCurrPosition = getPosition();
//   //}
//
//	if (mIsGroundAnimating) {
//
//		Ogre::Matrix3 kTransform,kBodyTransform;
//		Ogre::Quaternion kBody;
//		Ogre::Vector3 pos,scale,base,ground,newpos,lastpos,kDV,kGV;
//		scale.set(1,1,0);
//
//		kBody.set(getTransform());
//		kBody.mulP(mGroundSequenceData->mDeltaVector,&kDV);//OH... i remember now.  This is to take a 
//		//Y vector like 0.0 0.3 0.0, representing forward motion, and rotate it around the Z axis so
//		//it represents forward motion for the character, whichever direction we're pointing in.
//
//		mLastThrTime = mCurrThrTime;//CurrThrTime is 0.0 to 1.0, from shapeInstance getPos().
//		mCurrThrTime = mShapeInstance->getPos(mScriptThread[0].thread);
//		
//		if (mCurrThrTime < mLastThrTime) 
//		{//(meaning we looped back to beginning)
//			mGroundStep = 0;
//			mGroundNode = mGroundSequenceData->mNodes[mGroundStep];
//
//			if (mGroundNode>=0) {
//				pos = mShapeInstance->mNodeTransforms[mGroundNode].getPosition();
//				lastpos = mGroundVector;
//				mGroundVector = (pos * scale);
//				kBody.mulP(mGroundVector,&kGV);
//				kGV += getPosition();
//				mGroundVector = kGV;
//				////Con::printf("Looping: my groundVector: %f %f %f.  getPosition %f %f %f  pos %f %f %f",kGV.x,kGV.y,kGV.z,getPosition().x,getPosition().y,getPosition().z,pos.x,pos.y,pos.z);
//				//mGroundVector = (pos * scale) + getPosition();            
//			}
//		}
//		else if ((mGroundStep < (mGroundSequenceData->mNumSteps-1)) && 
//			(mCurrThrTime > mGroundSequenceData->mTimes[mGroundStep+1])) 
//		{
//			mGroundStep++;
//			mGroundNode = mGroundSequenceData->mNodes[mGroundStep];
//			if (mGroundNode>=0) {            
//				pos = mShapeInstance->mNodeTransforms[mGroundNode].getPosition();
//				lastpos = mGroundVector;
//				mGroundVector = (pos * scale);
//				kBody.mulP(mGroundVector,&kGV);
//				kGV += getPosition();
//				mGroundVector = kGV;
//				////Con::printf("Changing ground nodes: GV: %f %f %f.  getPosition %f %f %f  pos %f %f %f",kGV.x,kGV.y,kGV.z,getPosition().x,getPosition().y,getPosition().z,pos.x,pos.y,pos.z);
//				//mGroundVector = (pos * scale) + getPosition();
//				//mDeltaVector = mGroundVector - lastpos;
//				////Con::errorf("changed feet: delta = %3.2f %3.2f %3.2f",mDeltaVector.x,mDeltaVector.y,mDeltaVector.z);
//			}
//		}
//
//		if (mGroundNode>=0) {
//			////Con::errorf("ground animating: node = %d",mGroundNode);
//			Ogre::Vector3 ground2;
//			ground = mShapeInstance->mNodeTransforms[mGroundNode].getPosition();
//			kBody.mulP(ground,&ground2);
//
//			////Con::errorf("ground animating: node = %d, ground %f %f %f, ground2 %f %f %f",mGroundNode,
//			//	ground.x,ground.y,ground.z,ground2.x,ground2.y,ground2.z);
//			ground2 *= scale;
//			newpos = mGroundVector - ground2;
//			Point2F lamePos(newpos.x,newpos.y);
//			float z = mPM->getTerrHeight(lamePos);
//			newpos.z = z;
//			lastpos = getPosition();
//			setPosition(newpos);
//			mCurrPosition = newpos;
//			////Con::errorf("new pos %f %f %f",newpos.x,newpos.y,newpos.z);
//			//mDeltaVector = newpos - lastpos;
//			////Con::errorf("moving: newpos = %3.2f %3.2f %3.2f",newpos.x,newpos.y,newpos.z,);
//			//ground = mShapeInstance->mNodeTransforms[mGroundNode].getPosition();
//			//ground2 = ground + getPosition();
//			////Con::errorf("     ground after = %3.2f %3.2f %3.2f",ground2.x,ground2.y,ground2.z);
//		} else {
//			Ogre::Vector3 lastpos = getPosition();
//			//newpos = lastpos + mDeltaVector;
//			newpos = lastpos + kDV;
//			setPosition(newpos);
//			mCurrPosition = newpos;
//		}
//	}
//	////Con::printf("flexbody processed tick.");
//}

void fxFlexBody::recordTick()
{
	//mRecordCount++;
	//if (!(mRecordCount % mRecordSampleRate)&&(mBodyParts[0]))//&&(isClientObject())//mBodyParts[0] - crashes if we fire off too early?
	//{
	//	nodeTranslations.increment();
	//	//nodeTranslations[nodeTranslations.size()-1] = mCurrPosition;
	//	//nodeTranslations[nodeTranslations.size()-1] = mBodyParts[0]->mRB->getLinearPosition();
	//	
	//	Ogre::Vector3 kDiff,kPos;
	//	Ogre::Quaternion kRot = mRecordInitialOrientation;
	//	kRot.inverse();
	//	//bool kRelativePosition = true;//false;//
	//	bool kRelativePosition = dynamic_cast<nxPhysManager*>(mPM)->mSceneRecordLocal;
	//	if (kRelativePosition)
	//	{//TEMP - kRelativePosition: trying to figure out what will work fpr iClone.  Doesn't rotate  
	//		//bvh position when you rotate actor.
	//		kDiff = mBodyParts[0]->mRB->getLinearPosition() - mRecordInitialPosition;
	//		kRot.mulP(kDiff,&kPos);
	//		////Con::printf("kPos recording relative: %3.2f %3.2f %3.2f, initial %3.2f %3.2f %3.2f",kPos.x,kPos.y,kPos.z,
	//		//	mRecordInitialPosition.x,mRecordInitialPosition.y,mRecordInitialPosition.z);
	//	} else {
	//		kPos = mBodyParts[0]->mRB->getLinearPosition();// - mRecordInitialPosition;
	//		////Con::printf("kPos recording: %3.2f %3.2f %3.2f",kPos.x,kPos.y,kPos.z);
	//	}
	//	kPos /= mObjScale;
	//	nodeTranslations[nodeTranslations.size()-1] = kPos;


	//	//TEMP: checking for error condition resulting in saved sequences flickering between local origin
	//	//and desired ground translation (animated position, away from local origin)
	//	////Con::printf("flexbody recording trans!!! %3.2f %3.2f %3.2f isServer %d",nodeTranslations[nodeTranslations.size()-1].x,
	//	//		nodeTranslations[nodeTranslations.size()-1].y,nodeTranslations[nodeTranslations.size()-1].z,isServerObject());

	//	//unsigned int kTotalParts = mNumBodyParts;
	//	unsigned int kNumWeapons = 0;
	//	//if ((mWeapon)&&(mWeaponBodypart->mPartID>=0)) kTotalParts++;
	//	//if ((mWeapon2)&&(mWeapon2Bodypart->mPartID>=0)) kTotalParts++;
	//	////Con::errorf("recording tick:  kTotalParts %d, weapon part %d, weapon2 part %d",
	//	//	kTotalParts,mWeaponBodypart->mPartID,mWeapon2Bodypart->mPartID);

	//	Quat16 tempRots[MAX_FLEX_NODES];
	//	//for (unsigned int i=0;i<kTotalParts;i++)
	//	
	//	for (unsigned int i=0;i<mNumBodyParts;i++)
	//	{			
	//		Ogre::Quaternion q,p;
	//		//q = mBodyParts[i-kNumWeapons]->mRB->getAngularPosition();
	//		q = mBodyParts[i]->mRB->getAngularPosition();
	//		if (i>0) {
	//			//p = mBodyParts[i-kNumWeapons]->mParentBodyPart->mRB->getAngularPosition();
	//			
	//			p = mBodyParts[i]->mParentBodyPart->mRB->getAngularPosition();
	//			p.inverse();
	//			q *= p;
	//		} else {
	//			if (kRelativePosition)
	//				q *= kRot;
	//		}

	//		tempRots[i].set(q);

	//		if ((mWeapon)&&(i==mWeaponBodypart->mPartID))
	//		{
	//			nodeRotations.increment();
	//			q = mWeapon->mRB->getAngularPosition();
	//			p = mWeaponBodypart->mRB->getAngularPosition();
	//			p.inverse();
	//			q *= p;
	//			nodeRotations[nodeRotations.size()-1].set(q);
	//			//kNumWeapons++;
	//			//Con::printf("adding weapon %d node %d, mount node %d",
	//				mWeaponBodypart->mPartID,mWeaponBodypart->mNodeIndex,mWeaponMountNode);
	//		}
	//		
	//		if ((mWeapon2)&&(i==mWeapon2Bodypart->mPartID))
	//		{
	//			nodeRotations.increment();
	//			q = mWeapon2->mRB->getAngularPosition();
	//			p = mWeapon2Bodypart->mRB->getAngularPosition();
	//			p.inverse();
	//			q *= p;
	//			nodeRotations[nodeRotations.size()-1].set(q);
	//			//kNumWeapons++;
	//			//Con::printf("adding weapon2 %d",mWeapon2Bodypart->mPartID);
	//		}
	//	}
	//	for (unsigned int i=0;i<mNumBodyParts;i++)
	//	{//Now, copy them all to nodeRotations in the right order.
	//		nodeRotations.increment();
	//		//nodeRotations[nodeRotations.size()-1] = tempRots[i];//same as old way, just testing first step
	//		nodeRotations[nodeRotations.size()-1] = tempRots[mOrderNodes[i]];
	//		Ogre::Quaternion qF = tempRots[mOrderNodes[i]].getOgre::Quaternion();
	//	}
	//}
}

void fxFlexBody::makeSequence(const char *dsqName)//,bool importGround
{
	//unsigned int importGround = 0;
	//int numRealKeyframes;
	//TSShape *kShape = getShapeInstance()->getShape();
	//////Con::errorf("sequences: %d, rotations: %d, translations %d",kShape->sequences.size(),kShape->nodeRotations.size(),kShape->nodeTranslations.size());

	//// kRelativePosition = true;//false;//
	//bool kRelativePosition = dynamic_cast<nxPhysManager*>(mPM)->mSceneRecordLocal;
	//	
	//Thread& st = mScriptThread[0];
	//if (st.thread )
	//{
	//	st.sequence = 0;
	//	st.thread->setSequence(0,0.0);
	//	st.state = fxFlexBody::Thread::Stop;
	//	updateThread(st);
	//}

	//const Ogre::String dsqExt(".dsq");
	//String seqDir(dsqName);
	////remove ".dsq" from filename if it's there
	////if (strlen(seqDir.find(dsqExt.c_str(),0))) 
	////	seqDir.erase(seqDir.length()-dsqExt.length(),dsqExt.length());
	//if (strstr((const char *)seqDir.c_str(),".dsq"))
	//	seqDir.erase(seqDir.length()-dsqExt.length(),dsqExt.length());
	////if (strpos(seqDir.c_str(),dsqExt.c_str())>-1)         // seqDir.find(dsqExt.c_str(),0,String::Case|String::Left) > -1) 
	////	seqDir.erase(seqDir.length()-dsqExt.length(),dsqExt.length());

	////then separate the sequence name, to get it by itself and the path by itself.
	//unsigned int nameLength = strlen(strrchr(seqDir.c_str(),'/'))-1;
	//String seqName(seqDir);
	//seqName.erase(0,seqDir.length()-nameLength);
	//seqDir.erase(seqDir.length()-nameLength,nameLength);

	////and make sure there's no spaces
	//seqName.replace(' ','_');

	////Now, make the new sequence.
	//kShape->sequences.increment();
	//TSShape::Sequence & seq = kShape->sequences.last();
	//constructInPlace(&seq);

	//if (kRelativePosition)
	//	seq.numKeyframes = nodeTranslations.size();//safe because only base node has translations, one per keyframe.
	//else//NOW - for iClone, adding ten frames at the beginning to interpolate out from (0,0,0) to starting position.
	//	seq.numKeyframes = nodeTranslations.size() + 10;
	//seq.duration = (float)seq.numKeyframes * (TickSec*mRecordSampleRate);
	//seq.baseRotation = kShape->nodeRotations.size();
	//seq.baseTranslation = kShape->nodeTranslations.size();
	//seq.baseScale = 0;
	//seq.baseObjectState = 1;
	//seq.baseDecalState = 0;
	//seq.firstGroundFrame = kShape->groundTranslations.size();
	////if (importGround) seq.numGroundFrames = numSamples;
	////else seq.numGroundFrames = 0;//1;?
	//seq.numGroundFrames = 0;//TEMP, groundRotations.size();
	//seq.firstTrigger = kShape->triggers.size();
	//seq.numTriggers = 0;
	//seq.toolBegin = 0.0;
	//seq.flags = 0;//TSShape::Cyclic;// | TSShape::Blend;// | TSShape::MakePath;
	//seq.priority = 5;

	////Con::errorf("New sequence!  numKeyframes %d, duration %f, baseRotation %d",seq.numKeyframes,seq.duration,seq.baseRotation);
	//
	//seq.rotationMatters.clearAll();
	////seq.rotationMatters.set(0);
	////seq.rotationMatters.set(1);
	////seq.rotationMatters.set(2);
	////seq.rotationMatters.set(3);
	////for (unsigned int i=0;i<rc;i++) seq.rotationMatters.set(dtsNodes[i]);//numJoints
	//for (unsigned int i=0;i<mNumBodyParts;i++) { 
	//	seq.rotationMatters.set(mBodyParts[i]->mNodeIndex); 
	//	//Con::errorf("rotation matters: %d",mBodyParts[i]->mNodeIndex);
	//}

	//if (mWeapon) {
	//	seq.rotationMatters.set(mWeaponMountNode);
	//	//Con::printf("weapon node matters: %d",mWeaponMountNode);
	//}
	//if (mWeapon2) {
	//	seq.rotationMatters.set(mWeapon2MountNode);
	//	//Con::printf("weapon2 node matters: %d",mWeapon2MountNode);
	//}

	//seq.translationMatters.clearAll();
	//seq.translationMatters.set(0);//ASSUMPTION: only root node has position data

	//seq.scaleMatters.clearAll();
	//seq.visMatters.clearAll();
	//seq.frameMatters.clearAll();
	//seq.matFrameMatters.clearAll();
	////seq.decalMatters.clearAll();
	////seq.iflMatters.clearAll();

	//kShape->names.increment();
	//kShape->names.last() = StringTable->insert(seqName);

	//seq.nameIndex = kShape->findName(seqName);

	//if ((!kRelativePosition)&&(!importGround))
	//{
	//	for (unsigned int i=0;i<10;i++)
	//	{
	//		kShape->nodeTranslations.increment();
	//		//nodeTranslations[0].z = 0.0;//For iClone specifically - vertical axis has to be zero, iClone keeps 
	//		//it on ground as long as there is no Z (actually Y in bvh/iClone world) value.
	//		kShape->nodeTranslations[kShape->nodeTranslations.size()-1] = nodeTranslations[0] * ((float)i/10.0);
	//	}
	//	numRealKeyframes = seq.numKeyframes - 10;
	//} else numRealKeyframes = seq.numKeyframes;

	//for (unsigned int i=0;i<numRealKeyframes;i++)
	//{
	//	if (importGround)
	//	{
	//		kShape->nodeTranslations.increment();
	//		kShape->nodeTranslations[kShape->nodeTranslations.size()-1].x = 0.0;
	//		kShape->nodeTranslations[kShape->nodeTranslations.size()-1].y = 0.0;
	//		kShape->nodeTranslations[kShape->nodeTranslations.size()-1].z = nodeTranslations[i].z;

	//		kShape->groundRotations.increment();
	//		kShape->groundRotations[kShape->groundRotations.size()-1] = Ogre::Quaternion::IDENTITY;

	//		kShape->groundTranslations.increment();
	//		kShape->groundTranslations[kShape->groundTranslations.size()-1].x = nodeTranslations[i].x;
	//		kShape->groundTranslations[kShape->groundTranslations.size()-1].y = nodeTranslations[i].y;
	//		kShape->groundTranslations[kShape->groundTranslations.size()-1].z = 0.0;
	//	} else {
	//		kShape->nodeTranslations.increment();
	//		//nodeTranslations[i].z = 0.0;//iClone, see above. [Nope - didn't work either.]
	//		kShape->nodeTranslations[kShape->nodeTranslations.size()-1] = nodeTranslations[i];
	//		//Con::errorf("nodeTranslations: %f %f %f",nodeTranslations[i].x,nodeTranslations[i].y,nodeTranslations[i].z);
	//	}
	//}

	//unsigned int kTotalParts = mNumBodyParts;
	//if (mWeapon) kTotalParts++;//((mWeapon)&&(mWeaponBodypart->mPartID>=0))
	//if (mWeapon2) kTotalParts++;//((mWeapon2)&&(mWeapon2Bodypart->mPartID>=0))

	//for(unsigned int j=0;j<kTotalParts;j++)
	////for(unsigned int j=0;j<mNumBodyParts;j++)
	//{
	//	if ((!kRelativePosition)&&(!importGround))
	//	{
	//		for (unsigned int i=0;i<10;i++)
	//		{
	//			kShape->nodeRotations.increment();
	//			kShape->nodeRotations[kShape->nodeRotations.size()-1] = nodeRotations[j];//Should duplicate 
	//		}                                                             //first frame rotations ten times.
	//	}
	//	for (unsigned int i=0;i<numRealKeyframes;i++)
	//	{
	//		//q16.set(rots[(i*numJoints)+orderNodes[j]]);
	//		//q16.set(nodeRotations[(i*mNumBodyParts)+j]);

	//		//TEMP
	//		kShape->nodeRotations.increment();
	//		kShape->nodeRotations[kShape->nodeRotations.size()-1] = nodeRotations[(i*kTotalParts)+j];
	//		
	//		Ogre::Quaternion qF = nodeRotations[(i*kTotalParts)+j].getOgre::Quaternion();
	//		//Con::printf("  final nodeRots, bodypart %d:  %f %f %f %f",j,qF.x,qF.y,qF.z,qF.w);
	//		//kShape->nodeRotations[kShape->nodeRotations.size()-1] = nodeRotations[(i*mNumBodyParts)+j];
	//	}
	//}

	//nodeTranslations.clear();
	//nodeRotations.clear();
	//mRecordCount = 0;

	//////Con::printf("BVH -- nodes %d nodeTranslations %d nodeRotations %d sequences: %d",kShape->nodes.size(),
	////	kShape->nodeTranslations.size(),kShape->nodeRotations.size(),kShape->sequences.size());
	////for (unsigned int i=0;i<kShape->sequences.size();i++)
	////{
	////	TSShape::Sequence & seq = kShape->sequences[i];

	////	//Con::printf("Seq[%d] %s frames: %d duration %3.2f baseObjectState %d baseScale %d baseDecalState %d toolbegin %f",
	////		i,kShape->getName(seq.nameIndex).c_str(),seq.numKeyframes,
	////		seq.duration,kShape->sequences[i].baseObjectState,kShape->sequences[i].baseScale,
	////		kShape->sequences[i].baseDecalState,seq.toolBegin);
	////	//Con::printf("   groundFrames %d isBlend %d isCyclic %d flags %d",
	////		seq.numGroundFrames,seq.isBlend(),seq.isCyclic(),seq.flags);
	////}

	////HA!  Yay, finally T3D has its own exportSequence (singular) function, don't 
	////kShape->dropAllButOneSeq(kShape->sequences.size()-1); // have to do this anymore.

	//FileStream *outstream;
	//String dsqPath(dsqName);
	//if (!strstr(dsqPath.c_str(),".dsq")) dsqPath += dsqExt;
	////if (!gResourceManager->openFileForWrite(outstream,dsqPath.c_str())) {
	//if ((outstream = FileStream::createAndOpen( dsqPath.c_str(), Torque::FS::File::Write))==NULL) {
	//	//Con::printf("whoops, name no good: %s!",dsqPath.c_str()); 
	//} else {
	//	//kShape->exportSequences((Stream *)outstream);
	//	kShape->exportSequence((Stream *)outstream,seq,1);//1 = save in old format (v24) for show tool
	//	outstream->close();
	//}

	////Yay, don't have to do this anymore!
	////Now, load the sequence again, and drop the one we have... we hope this works.
	//
	//////loadDsq(dsqPath.c_str());
	//////kShape->dropAllButOneSeq(kShape->sequences.size()-1);
	//////HERE: using my existing shapeconstructor to reload all sequences, would be the win.
	////const String myFullPath = getShapeInstance()->mShapeResource.getPath().getFullPath();
	////TSShapeConstructor* ctor = TSShapeConstructor::findShapeConstructor( myFullPath );
	////if (ctor)
	////{
	////	//HERE: reloadSequences() instead.
	//////	//Con::errorf("found my shape constructor!  sequences %d",ctor->mSeqData.size());
	////	//ctor->??
	////	ctor->_onLoad( getShapeInstance()->mShapeResource );//see what happens if we pretend we're reloading...

	////	kShape->dropSequence(0);

	////	//String pathPlusName(dsqPath);
	////	//pathPlusName.insert(pathPlusName.length()," ");
	////	//pathPlusName.insert(pathPlusName.length(),kShape->getName(kShape->sequences[0].nameIndex));
	////	////Con::executef(ctor,"addSequence",ctor->scriptThis(), dsqPath.c_str(),kShape->getName(kShape->sequences[0].nameIndex));
	////	//Except, so far this seems much easier to do from script.  Going back out to cropCrop() in script to do this.
	////}

	////setThreadSequence(0,0,true);
}

void fxFlexBody::restoreSequences()
{//This is for putting the original data back into a sequence after it's been modified by ultraframes.
	//
	//TSShape *kShape = getShapeInstance()->getShape();

	//////Con::printf("trying to restore sequences!  ultraframeSets %d, sequencebackups %d",
	////	mUltraframeSets.size(),kShape->sequenceBackups.size());
	//if ((mUltraframeSets.size()>0) && (mUltraframeSets.size()==kShape->sequenceBackups.size()))
	//{
	//	//Con::printf("restoring %d sequences",mUltraframeSets.size());
	//	TSShape::sequenceBackup *seqBackup = NULL;
	//	for (unsigned int i=0;i<mUltraframeSets.size();i++)
	//	{
	//		unsigned int seq = kShape->findSequence(kShape->sequenceBackups[i].name);
	//		TSShape::Sequence *kSeq = &(kShape->sequences[seq]);
	//		seqBackup = &(kShape->sequenceBackups[i]);
	//		//Con::printf("sequence: %d  %s",seq,seqBackup->name.c_str());
	//		
	//		unsigned int rot_matters_count = 0;
	//		for (unsigned int j=0;j<kSeq->numKeyframes;j++)
	//		{
	//			if (kSeq->rotationMatters.test(j)) 
	//				rot_matters_count++;
	//		}
	//		for (unsigned int j=0;j<kSeq->numKeyframes;j++)
	//		{
	//			 kShape->nodeTranslations[kSeq->baseTranslation+j] = seqBackup->nodeTranslations[j];
	//		}
	//		for (unsigned int j=0;j<rot_matters_count;j++)
	//		{
	//			for (unsigned int k=0;k<kSeq->numKeyframes;k++)
	//			{
	//				kShape->nodeRotations[kSeq->baseRotation+(j*kSeq->numKeyframes)+k] = seqBackup->nodeRotations[(j*kSeq->numKeyframes)+k];
	//			}
	//		}
	//	}
	//}
	////Now, assuming all is restored, go ahead and delete the ultraframesets and seqbackups...
	//while (mUltraframeSets.size()>0)//HERE: don't we need to delete mShape->sequenceBackups as well?
	//	mUltraframeSets.erase((unsigned int)0);
	//while (kShape->sequenceBackups.size()>0)//but wait, don't we need to copy them back to the sequence first??
	//	kShape->sequenceBackups.erase((unsigned int)0);
}

void fxFlexBody::reloadSequences()
{
	//const String myFullPath = getShapeInstance()->mShapeResource.getPath().getFullPath();
	//TSShapeConstructor* ctor = TSShapeConstructor::findShapeConstructor( myFullPath );
	//if (ctor)
	//{
	//	ctor->_onLoad( getShapeInstance()->mShapeResource );//see what happens if we pretend we're reloading...
	//	//Con::printf("Trying to reload sequences... commands %d",ctor->mChangeSet.mCommands.size());
	//	for (unsigned int i = 0; i< ctor->mChangeSet.mCommands.size(); i++)
	//	{
	//		//Con::printf("command 1: type %d argv[0] %s",ctor->mChangeSet.mCommands[i].type);

	//	}
	//}
}

void fxFlexBody::onWorldStep()
{
	std::ostringstream os;

	if ((mIsKinematic)&&(mCurrTick>1))
	{
		//HERE: find this actor's default anim and start it. (EM only)
		if (mEntity->getSkeleton()->hasAnimation("walkFull"))
		{
			Ogre::AnimationState *as = mEntity->getAnimationState("walkFull");
			if (!(as->getEnabled()))
			{
				as->setLoop(true);
				as->setEnabled(true);
				as->setTimePosition(0.0);
			}
			as->addTime(0.032);//FIX - add time since last here, not just 0.032 no matter what.
		}
	}

	if (mCurrTick>130)   //TEMP... testing...
	{
		if (mEntity->getSkeleton()->hasAnimation("walkFull"))
		{
			//HERE: find a way to turn off all active animation, not just this one.
			Ogre::AnimationState *as = mEntity->getAnimationState("walkFull");
			if (as->getEnabled())
				as->setEnabled(false);
		}
		clearKinematic();
	}


	//bool kIsServer = isServerObject();
	//if (kIsServer) 
	//{
		//if (mReset) //setMaskBits(MoveMask);
		//return;
	//}
   //if (mCurrTick==1)
   //{
   //   mInitialPosition = mCurrPosition;
   //   Ogre::Matrix3 m = getTransform();
   //   Ogre::Quaternion q(m);
   //   mInitialOrientation = q;
   //}
   /*
   if ((mInitialPosition.length() == 0.0)&&(mCurrPosition.length() != 0.0))
   {
      mInitialPosition = mCurrPosition;//getTransform().getPosition? Make sure this is always up to date.
   } 
   else 
   {//(This is to make us wait another tick for orientation to catch up.)
      Ogre::Matrix3 m = getTransform();
      if ((mInitialOrientation.isIdentity())&&(!m.isIdentity()))
      {         
         Ogre::Quaternion q(m);
         mInitialOrientation = q;
      }
   }*/

	//if (!mIsPhysActive) 
	//{	
	//	return;
	//}

	//if (!strcmp(mActorName,"zombie_2") && getThreadSequence(0)>-1)
	//{
	//	//Con::printf("flexbody on world step, threadState %d, threadAtEnd %d, threadDir %d, threadPos %f",
	//		getThreadState(0),isThreadAtEnd(0),getThreadDir(0),getThreadPos(0));
	//}
	
	//Here: deal with the triggered-but-not-collided bug.  If no collision within 200 MS, go back to kinematic.
	//if ((mIsAnimating)&&(!mIsKinematic))
	//{
	//	unsigned int curTimeMS = Platform::getVirtualMilliseconds();
	//	if (((curTimeMS - mTriggerTimeMS)>200)&&(mTriggerTimeMS>0))
	//	{
	//		setKinematic();
	//		mClearIsAnimating = false;
	//		mStopAnimating = false;
	//		mTriggerTimeMS = 0;
	//		////Con::errorf("flexbody triggered-not-colliding");
	//	}
	//}

	//if ((mBeenHit)&&(mBeenHitTick < (mCurrTick - 30)))
	//{
	//	mBeenHit = false;
	//	mBeenHitTick = 0;
	//	if ( hasRagdollBodyparts() && mIsKinematic )
	//		setKinematic();
	//}

	//if ((mIsAnimating)&&(mIsCorpse))
	//{//can't remember why I was doing this, but it's causing render bug now.
		//if (getThreadPos(0)==1.0) 
		//{
		//	startAnimating(0);//mIdleAnim
		//	mIsCorpse = false;
		//	//Con::errorf("flexbody isCorpse");
		//}
	//}

	float avgDelta = 0.0;

	//Sigh, again, feel like this whole thing should be rewritten, but for now, another bandaid.
	//This is for moving our clearBodypart() logic into the next safe worldStep instead of trying
	//to do it any old time on demand.
	//for (unsigned int i = 0; i < mNumBodyParts; i++) 
	//{
	//	if ((mBodyParts[i]->mRB->getIsKinematic())&&(mBodyParts[i]->mIsKinematic==false))
	//		clearBodypart(i);
	//}

	//NOW:  Time for a major rewrite.  New rules: 1) we do a single ultraframes search right here 
	//at the beginning, and test by bodypart whether we're kinematic or not.
	//We need a bool mIsActing at the bodypart level, as opposed to just mIsThinking at the body level.
	//char actorName[255];
	//strcpy(actorName,mActorName.c_str());

	//if (mVerbose)
	//{
		//Ogre::Vector3 myPos = getPosition();
		//Ogre::Vector3 myPartPos = mBodyParts[0]->getPosition();
		//Con::printf("flexbody ticking %d pos %3.2f %3.2f %3.2f , bodypart[0] pos %3.2f %3.2f %3.2f ",
			//mCurrTick,myPos.x,myPos.y,myPos.z,myPartPos.x,myPartPos.y,myPartPos.z);
	//	for (unsigned int i=0;i<mNumBodyParts;i++)
	//		//Con::printf("part %d: %d",i,mBodyParts[i]->mIsKinematic);
	//}

	//if ((mIsThinking)&&(mActionUser)) mActionUser->think();//run any GA or action user functions, act()

	//////////////////////////////////////////////////////////////////////////////////////////////
	///// if (we are full ragdoll)  //////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	//if ((!mIsAnimating))
	if ((!mIsKinematic))
	{
		//if (mScriptThread[0].state != Thread::Stop)
		//	stopThread(0);

		for (unsigned int i = 0; i < mNumBodyParts; i++) 
		{
			//if (!mBodyParts[i]->mIsKinematic)
			//{
			if (!mIsThinking) 
			{
				if ((mBodyParts[i]->mIsKinematic==false)&&(mBodyParts[i]->mRB->getIsKinematic()))
					mBodyParts[i]->mRB->setKinematic(false);


				mBodyParts[i]->mRB->updatePositionFromActor();
				mBodyParts[i]->mRB->updateVelocityFromActor();

				mBodyParts[i]->updatePositionFromRB();
				mBodyParts[i]->updateVelocityFromRB();

				//Ogre::Vector3 globalPos = mBodyParts[i]->mRB->getLinearPosition() - mNode->getPosition();
				//Ogre::Vector3 localPos = mBodyParts[i]->mBone->convertWorldToLocalPosition(globalPos);
				//mBodyParts[i]->mBone->setPosition(localPos * 100.0);//Okay, this should be utterly fucked up... but 
				//moving the bones at all would be preferable to nothing.  What I need to do is set the bone 
				//rotations ONLY, there should be no need to set the positions if all the rotations work.
				//(Child's position should be determined by rotation of parent.)

				mBodyParts[i]->mBone->setManuallyControlled(true);

				if (i==0)
				{
					mNode->setPosition(mBodyParts[i]->mRB->getLinearPosition());
					//mNode->setOrientation(mBodyParts[i]->mRB->getAngularPosition());
					mNode->setOrientation(Ogre::Quaternion::IDENTITY);
					mBodyParts[i]->mBone->setPosition(Ogre::Vector3::ZERO);
					Ogre::Quaternion quat = mBodyParts[i]->mRB->getAngularPosition();
					mBodyParts[i]->mBone->_setDerivedOrientation(quat);
					
					//if (mCurrTick < 30) {
					//	//mEntity->getSkeleton()->getBone(0)->setOrientation(Ogre::Quaternion::IDENTITY);
					//	//mEntity->getSkeleton()->getBone(1)->setOrientation(Ogre::Quaternion::IDENTITY);
					//	//mEntity->getSkeleton()->getBone(2)->setOrientation(Ogre::Quaternion::IDENTITY);
					//	Ogre::Quaternion node_quat = mNode->getOrientation();
					//	Ogre::Quaternion bone0_quat = mEntity->getSkeleton()->getBone(0)->getOrientation();
					//	Ogre::Quaternion bone1_quat = mEntity->getSkeleton()->getBone(1)->getOrientation();
					//	Ogre::Quaternion bone2_quat = mEntity->getSkeleton()->getBone(2)->getOrientation();
					//	Ogre::Quaternion bone3_quat = mEntity->getSkeleton()->getBone(3)->getOrientation();
					//	os.str("");
					//	os << " node quat: w " << node_quat.w << " x " << node_quat.x << " y " << node_quat.y << " z " << node_quat.z <<
					//		  " bone 0 quat: w " << bone0_quat.w << " x " << bone0_quat.x << " y " << bone0_quat.y << " z " << bone0_quat.z <<
					//		  " bone 1 quat: w " << bone1_quat.w << " x " << bone1_quat.x << " y " << bone1_quat.y << " z " << bone1_quat.z <<
					//		  " bone 2 quat: w " << bone2_quat.w << " x " << bone2_quat.x << " y " << bone2_quat.y << " z " << bone2_quat.z <<
					//		  " bone 3 quat: w " << bone3_quat.w << " x " << bone3_quat.x << " y " << bone3_quat.y << " z " << bone3_quat.z ;
					//	gConsole->addOutput(os.str());
					//}
				} else {
					Ogre::Quaternion quat = mBodyParts[i]->mRB->getAngularPosition();
					mBodyParts[i]->mBone->_setDerivedOrientation(quat);
				}
			} else {
				if (mActionUser->mIsResetting)
				{
					mBodyParts[i]->reset();
					mBodyParts[i]->updatePositionToRB();
					mBodyParts[i]->updateVelocityToRB();
				} else {
					mBodyParts[i]->updatePositionFromRB();
					mBodyParts[i]->updateVelocityFromRB();
				}
			}
			//if (i==8) {
			//	Ogre::Vector3 pos = mBodyParts[i]->mCurrPosition;
			//	//Con::errorf("right hand pos: %f %f %f",pos.x,pos.y,pos.z);
			//}
			//if (i==0)
			//{
			// Ogre::Vector3 pos = mBodyParts[i]->getPosition();
			// //Con::printf("base node pos: %f %f %f",pos.x,pos.y,pos.z);
			//}
			//////Con::errorf("updating velocity: %f %f %f",mBodyParts[i]->mCurrVelocity.x,mBodyParts[i]->mCurrVelocity.y,mBodyParts[i]->mCurrVelocity.z);
			mBodyParts[i]->updateForcesToRB();//watch out here, this might mess up GA
			avgDelta += mBodyParts[i]->mDeltaPos;
			//}// else {
			//mBodyParts[i]->updatePositionToRB();
			//}
		}
		avgDelta /= mNumBodyParts;

		//////////////////////////////////
		//turns out this is unnecessary
		//Ogre::Matrix3 kTransform = getTransform();
		//const Ogre::Vector3 curPos = mBodyParts[0]->mRB->getLinearPosition();
		//kTransform.setPosition(curPos);
		//setTransform(kTransform);

		int currStep = ((physManagerCommon *)physManagerCommon::getPM())->mCurrStep;

		//if ( //(mSequenceEndStep && (mCurrMS>=mSequenceEndStep) ) ||//Either stop ragdolling because we reached end of sequence time,
		//	((avgDelta)&&(avgDelta < mSleepThreshold) &&  //(!mSequenceRagdoll)// or stop because our average movement dropped below sleep threshold,
		//	currStep > (mRagdollStep+10)) )//&&(!mIsPlayer))     // meaning interesting ragdoll has finished, time to get up or go to sleep.
		//	//Ragdoll step gives every ragdoll action at least a third of a second to avoid 
		//	//erroneously "waking up" before we even start to fall.
		//	//FIX -- Still need a method for deciding if we should get up or stay down, i.e. are we dead yet?
		//{
		//	//NOW: make this a script callback, get all logic out of here except for setting kinematic.
		//	setKinematic();
		//	//mIsAnimating = true;
		//	//mClearIsAnimating = false;

		//	//Con::printf("below sleep threshold, setting kinematic!!!!!!!!!!!!!!!!!!!");
		//	mIsCorpse = true;//cheesy, but need to avoid torque reanimating
		//	//resetPosition();


		//	//FIX: Ogre version of everything.
		//	//Ogre::Vector3 kPos = getPosition();
		//	//Ogre::Matrix3 m;  m = Ogre::Matrix3::IDENTITY;

		//	//if (!mIsReturnToZero) 
		//	//{
		//	//	m.setPosition(kPos);
		//	//	setTransform(m);

		//	//} else {//m = Ogre::Quaternion::IDENTITY;//else set to initial orientation and position
		//	//	mInitialOrientation.setMatrix(&m);
		//	//	m.setPosition(mInitialPosition);
		//	//	setTransform(m);
		//	//	mPlaylistTick = 0;

		//	//	//runPlaylist();//let onRagdollStop in script worry about this
		//	//}
		//	//Con::executef(this,"onRagdollStop",scriptThis());
		//}
		for (unsigned int i = 0; i < mNumBodyParts; i++) 
		{
			if (mBodyParts[i]->mHasTempForce)
			{
				mBodyParts[i]->addForceAtPos(mBodyParts[i]->mTempForce,mBodyParts[i]->mTempPos);
				//Con::errorf("Adding force at pos: %f %f %f",
					//mBodyParts[i]->mTempForce.x,mBodyParts[i]->mTempForce.y,mBodyParts[i]->mTempForce.z);
				mBodyParts[i]->mHasTempForce = false;
			}
		}
		//////////////////////////////////////////////////////////////////////////////////////////////
		///// else we are animating  /////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////////////////
	} else  {

		//CAUTION:  this breaks optitrack streaming, because it animates the character in between arena frames.
	   //if (!mIsStreaming) getShapeInstance()->animateNodeSubtrees();//in case this hasn't been done yet for this frame

	   //Ogre::Vector3 nDiff = getPosition() - mInitialPosition;
	   //if ((nDiff.x==0.0)&&(nDiff.y==0.0))
		   ////Con::errorf("body resetting? nDiff %3.2f %3.2f %3.2f",nDiff.x,nDiff.y,nDiff.z);
   
	   //if ((mFollowEvent)&&(!mFollowEventDone))
	   //{
		  // char strValue[255];
		  // sprintf(strValue,"%f %f %f",mFollowEvent->value.x,mFollowEvent->value.y,mFollowEvent->value.z);
		  // //if ((!mFollowEvent->action.isEmpty())&&(mFollowEvent->eventType != SE_FOLLOW_MOVE_TO_POSITION))
			 //  //Con::executef(this, mFollowEvent->action.c_str(), scriptThis(), strValue);
			 //  ////Con::executef(this, "firstFollowFunction", scriptThis());
	   //}
	   //if (mIsMoveTargeting) 
	   //{//Hmm possibly move this out to script, above in follow event
		  // Ogre::Vector3 diff,moveDiff,dir;
		  // Ogre::Quaternion arc,final;
		  // Ogre::Matrix3 finalMat;
		  // if (mTarget)
		  // {
			 //  //mMoveTarget = mTarget->getPosition();
			 //  mMoveTarget = mTarget->mBodyParts[0]->getPosition();
			 //  mMoveTarget.z = 0.0;
			 //  diff = mMoveTarget - getPosition();
			 //  if ((diff.length()>0.3))//??mMoveThreshold?//&&(strcmp(mActorName,"DesertSoldier")))//if NOT desert soldier
			 //  {
				//   ////Con::errorf("setting move target to target body position: %f %f %f",
				////	   mMoveTarget.x,mMoveTarget.y,mMoveTarget.z);

				//   Ogre::Vector3 diffNorm = diff;
				//   diffNorm.z = 0.0;
				//   diffNorm.normalize();

				//   Ogre::Matrix3 mat;// = getTransform();
				//   //Ogre::Quaternion q(mat);
				//   //q.mulP(Ogre::Vector3(0,1,0),&dir);//dir should now be Y vector multiplied by our world transform, i.e. the direction we are facing.
				//   dir.set(0,1,0);
				//   //Now, rotate around the Z axis so that our Y axis points at the target.
				//   dir.normalize(); //just to make sure, should already be.
				//   arc.shortestArc(diffNorm,dir);
				//   if (!(mIsNaN_F(arc.x)||mIsNaN_F(arc.y)||mIsNaN_F(arc.z)||mIsNaN_F(arc.w)))
				//   {
				//	   arc.setMatrix(&mat);
				//	   //final.mul(q,arc);
				//	   //q.setMatrix(&finalMat);
				//	   //mat.mul(finalMat);
				//	   Ogre::Vector3 myPos = getPosition();
				//	   setTransform(mat);
				//	   setPosition(myPos);
				//   }
				//   ////Con::printf("%s transforming to target: %f %f %f",
				//	 //  mActorName,mMoveTarget.x,mMoveTarget.y,mMoveTarget.z);

			 //  }
		  // } else {
			 //  diff =  mMoveTarget - getPosition();//mBodyParts[0]->getPosition()//can't use actual flexbody position because it
			 //  //only changes once per walk cycle.  Next, add a mPositionNode, to track desired position node if it isn't root node.
			 //  //realign to target
			 //  Ogre::Vector3 diffNorm = diff;
			 //  diffNorm.z = 0.0;
			 //  diffNorm.normalize();
			 //  Ogre::Matrix3 mat;
			 //  Ogre::Quaternion arc;
			 //  Ogre::Vector3 dir;
			 //  dir.set(0,1,0);
			 //  arc.shortestArc(diffNorm,dir);
			 //  arc.setMatrix(&mat);
			 //  Ogre::Vector3 myPos = getPosition();
			 //  setTransform(mat);
			 //  setPosition(myPos);

		  // }
		  // if (diff.length() < mMoveThreshold) 
		  // {
			 //  //stopThread(0);// w/o this, ground transforms get applied from partway through the 
			 //  //Con::executef(this, "onReachTarget", scriptThis());
			 //  //AAARRGH, must move all this to flexbody!!
			 //  if (mIsGroundAnimating) mIsGroundAnimating = false;
			 //  if (mFollowEvent)
				//   if (mFollowEvent->eventType==SE_FOLLOW_MOVE_TO_POSITION)
				//	   finishFollowEvent();
			 //  if (mFollowEvent)//If new follow event assigned in finishFollowEvent...
			 //  {
				//   if (mFollowEvent->eventType==SE_FOLLOW_MOVE_TO_POSITION)
				//   {
				//	   mMoveTarget = mFollowEvent->value;
				//	   moveToPosition(mMoveTarget,mFollowEvent->action);
				//	   mFollowEventDone = false;
				//   }
			 //  }
		  // }
	   //}
	   for (unsigned int i = 0; i < mNumBodyParts; i++) 
	   {
		   //HERE: instead of updating position and velocity directly, we're gonna try
		   //giving each joint a goal orientation.
		   //int keyNum1,keyNum2;
		   //float keyPos;
		   //if (mScriptThread[0].thread) {
		   //mCurrThrTime = mShapeInstance->getPos(mScriptThread[0].thread);
		   //mScriptThread[0].thread->selectKeyframes(mCurrThrTime,mScriptThread[0].thread->sequence,&keyNum1,&keyNum2,&keyPos);
		   ////Con::errorf("between keyframes %d and %d by %f",keyNum1,keyNum2,keyPos);
		   //}
		   //NEXT: get the proper nodeRotations from mShape->nodeRotations.  Get this one and the next one.  
		   //interpolate between them with slerp.  Take the final result and set it as your driveOrientation.
		   //Then, if all that works, and it's a little laggy, try jumping forward one tick.

		   //HERE: do this only if THIS BODYPART is animating.  Allow us to keep parts of the body animating 
			//while other parts are ragdolling.

			Ogre::Quaternion globalQuat = mBodyParts[i]->mBone->convertLocalToWorldOrientation(Ogre::Quaternion::IDENTITY);
			Ogre::Quaternion nodeQuat = mNode->getOrientation();
			//Ogre::Quaternion finalQuat = globalQuat * nodeQuat; 
			Ogre::Quaternion finalQuat =   globalQuat.Inverse() * nodeQuat.Inverse();//Okay, gold star for anyone who can tell me:  WHY INVERSES??
			//Ogre::Quaternion finalQuat = Ogre::Quaternion(globalQuat.w,globalQuat.x,globalQuat.y,globalQuat.z); //

			Ogre::Matrix3 mat;			
			Ogre::Vector3 globalPos = mBodyParts[i]->mBone->convertLocalToWorldPosition(Ogre::Vector3::ZERO);
			globalPos *= mScale;
			nodeQuat.ToRotationMatrix(mat);
			Ogre::Vector3 finalPos = (mat * globalPos) + mNode->getPosition();

			if (mCurrTick > 1) {
				mBodyParts[i]->mLastPosition = mBodyParts[i]->mRB->getLinearPosition();
				Ogre::Vector3 curVel = finalPos - mBodyParts[i]->mLastPosition;
				curVel *= 32;//distance times 32 ticks per second...
				//if (i==7)
				//{
				//	os.str("");
				//	os << "curVelocity: " << curVel.x << " " <<  curVel.y << " " <<  curVel.z ;
				//	gConsole->addOutput(os.str());
				//}
				mBodyParts[i]->mRB->setLinearVelocity(curVel);
				mBodyParts[i]->mRB->setLinearPosition(finalPos);

				mBodyParts[i]->mLastOrientation = mBodyParts[i]->mRB->getAngularPosition();
				mBodyParts[i]->mRB->setAngularPosition(finalQuat);
				Ogre::Quaternion lastOrient = mBodyParts[i]->mLastOrientation;


				//Now, the sticky part: converting differences in quaternions into XYZ vector  
				// for nxActor::setAngularVelocity().
				Ogre::Radian yaw,pitch,roll;
				Ogre::Matrix3 angMat;
				Ogre::Vector3 angVel;
				Ogre::Quaternion finalAngleStep(Ogre::Quaternion::IDENTITY);
				Ogre::Quaternion angStep = lastOrient.Inverse() *  finalQuat;//Right??
				for (int i=0;i<32;i++) //Right??
					finalAngleStep = finalAngleStep * angStep;
				finalAngleStep.ToRotationMatrix(angMat);
				angMat.ToEulerAnglesXYZ(yaw,pitch,roll);
				angVel.x = pitch.valueRadians();  angVel.y = yaw.valueRadians();  angVel.z = roll.valueRadians(); //Right??
				mBodyParts[i]->mRB->setAngularVelocity(angVel);


				//TEMP: experimenting with EulerAngles code.
				//Ogre::Vector3 col0,col1,col2;
				//col0 = mat.GetColumn(0);
				//col1 = mat.GetColumn(1);
				//col2 = mat.GetColumn(2);

				//HMatrix	hMatrix;
				//hMatrix[0][0] = col0.x; hMatrix[1][0] = col1.x; hMatrix[2][0] = col2.x; hMatrix[3][0] = 0.0;
				//hMatrix[0][1] = col0.y; hMatrix[1][1] = col1.y; hMatrix[2][1] = col2.y; hMatrix[3][1] = 0.0;
				//hMatrix[0][2] = col0.z; hMatrix[1][2] = col1.z; hMatrix[2][2] = col2.z; hMatrix[3][2] = 0.0;
				//hMatrix[0][3] = 0.0; hMatrix[1][3] = 0.0; hMatrix[2][3] = 0.0; hMatrix[3][3] = 1.0;

				//EulerAngles eulQ;
				//eulQ = Eul_FromHMatrix( hMatrix,EulOrdXYZs);
				


				mBodyParts[i]->mRB->updatePositionToActor();
				mBodyParts[i]->mRB->updateVelocityToActor();
			}
			//if (mCurrTick < 30) {
			//	//mEntity->getSkeleton()->getBone(0)->setOrientation(Ogre::Quaternion::IDENTITY);
			//	//mEntity->getSkeleton()->getBone(1)->setOrientation(Ogre::Quaternion::IDENTITY);
			//	//mEntity->getSkeleton()->getBone(2)->setOrientation(Ogre::Quaternion::IDENTITY);
			//	Ogre::Quaternion node_quat = mNode->getOrientation();
			//	Ogre::Quaternion bone0_quat = mEntity->getSkeleton()->getBone(0)->getOrientation();
			//	Ogre::Quaternion bone1_quat = mEntity->getSkeleton()->getBone(1)->getOrientation();
			//	Ogre::Quaternion bone2_quat = mEntity->getSkeleton()->getBone(2)->getOrientation();
			//	Ogre::Quaternion bone3_quat = mEntity->getSkeleton()->getBone(3)->getOrientation();
			//	os.str("");
			//	os << " node quat: w " << node_quat.w << " x " << node_quat.x << " y " << node_quat.y << " z " << node_quat.z <<
			//		os << " bone 0 quat: w " << bone0_quat.w << " x " << bone0_quat.x << " y " << bone0_quat.y << " z " << bone0_quat.z <<
			//		os << " bone 1 quat: w " << bone1_quat.w << " x " << bone1_quat.x << " y " << bone1_quat.y << " z " << bone1_quat.z <<
			//		os << " bone 2 quat: w " << bone2_quat.w << " x " << bone2_quat.x << " y " << bone2_quat.y << " z " << bone2_quat.z <<
			//		os << " bone 3 quat: w " << bone3_quat.w << " x " << bone3_quat.x << " y " << bone3_quat.y << " z " << bone3_quat.z ;
			//	gConsole->addOutput(os.str());
			//}
	   }

	   //Ogre::Matrix3 mountTransform,matWeapon,matWeaponAdj,matWeaponAdjA,matWeaponAdjB,matWeaponAdjInv,matAdjX,matAdjZ,matAdjFinal;
	   //Ogre::Quaternion quatAdjFinal,Ogre::QuaternioninalInverse,quatWeapon,temp,temp2;
	   //Ogre::Vector3 adjPos,weapPos,finalPos;

	   //if ((mWeapon)&&(mWeaponMountNode>=0))
	   //{
		  // mountTransform = Ogre::Quaternion::IDENTITY;
		  // matWeaponAdj = Ogre::Quaternion::IDENTITY;

		  // mWeaponPosAdj = mWeapon->mWeaponPosAdj;
		  // mWeaponRotAdjA = mWeapon->mWeaponRotAdjA;
		  // mWeaponRotAdjB = mWeapon->mWeaponRotAdjB;
		  // mWeaponRotAdjA.setMatrix(&matWeaponAdjA);
		  // mWeaponRotAdjB.setMatrix(&matWeaponAdjB);

		  // matWeaponAdj.mul(matWeaponAdjA,matWeaponAdjB);

		  // if (mIsStreaming)
		  // {
			 //  mWeaponRot.setMatrix(&matWeapon);

			 //  mountTransform.mul(matWeapon);
			 //  mountTransform.mul(matWeaponAdj);

			 //  mountTransform.mulP(mWeaponPosAdj,&adjPos);
			 //  if (0)//arena-defined global pos way
			 //  {
				//   weapPos = mWeaponPos + adjPos;
			 //  }
			 //  else //dts-defined mount node way
			 //  {
				//   Ogre::Matrix3 kTransform = mShapeInstance->mNodeTransforms[mWeaponMountNode];
				//   weapPos = kTransform.getPosition() + adjPos;
				//   //weapPos = mWeaponPos + adjPos;
			 //  }
			 //  finalPos = weapPos + getPosition();
			 //  mountTransform.setPosition(finalPos);

		  // } else {
			 //  mountTransform = mShapeInstance->mNodeTransforms[mWeaponMountNode];
			 //  Ogre::Quaternion mount(mountTransform);
			 //  //mWeapon->mCurrAngularPosition = mount;
			 //  ////Con::printf("current mount transform: %3.2f %3.2f %3.2f %3.2f",mount.x,mount.y,mount.z,mount.w);
			 //  finalPos = mountTransform.getPosition() + getPosition();//FIX: rotate mountTransform.getPosition by flexbody orientation!
			 //  mountTransform.setPosition(finalPos);
		  // }
		  // mWeapon->setTransform(mountTransform);
		  // mWeapon->setRenderTransform(mountTransform);
		  // mWeapon->//setMaskBits(fxRigidBody::fxRigidMoveMask);
	   //}
	   //if ((mWeapon2)&&(mWeapon2MountNode>=0))
	   //{
		  // //Ogre::Matrix3 mountTransform = mShapeInstance->mNodeTransforms[mWeapon2MountNode];

		  // //Ogre::Matrix3 mountTransform,matWeapon,matWeaponAdj,matWeaponAdjA,matWeaponAdjB,matWeaponAdjInv;

		  // mountTransform = Ogre::Quaternion::IDENTITY;
		  // matWeaponAdj = Ogre::Quaternion::IDENTITY;

		  // mWeapon2PosAdj = mWeapon2->mWeaponPosAdj;
		  // mWeapon2RotAdjA = mWeapon2->mWeaponRotAdjA;
		  // mWeapon2RotAdjB = mWeapon2->mWeaponRotAdjB;
		  // mWeapon2RotAdjA.setMatrix(&matWeaponAdjA);
		  // mWeapon2RotAdjB.setMatrix(&matWeaponAdjB);

		  // matWeaponAdj.mul(matWeaponAdjA,matWeaponAdjB);

		  // if (mIsStreaming)
		  // {
			 //  mWeapon2Rot.setMatrix(&matWeapon);

			 //  //mountTransform.mul(matWeaponAdj);
			 //  mountTransform.mul(matWeapon);
			 //  mountTransform.mul(matWeaponAdj);

			 //  mountTransform.mulP(mWeapon2PosAdj,&adjPos);
			 //  if (0)//arena-defined global pos way
			 //  {
				//   weapPos = mWeapon2Pos + adjPos;
			 //  }
			 //  else //dts-defined mount node way
			 //  {
				//   Ogre::Matrix3 kTransform = mShapeInstance->mNodeTransforms[mWeapon2MountNode];
				//   weapPos = kTransform.getPosition() + adjPos;
				//   //weapPos = mWeaponPos + adjPos;
			 //  }
			 //  //HERE: set my mWeaponBodypart's trigger body position, through joint spring or motor force.
			 //  //Ogre::Quaternion mountQuat(mountTransform);
			 //  //((nxRigidBody *)(mWeaponBodypart->mRB))->setTriggerJointMotorTarget(mountQuat);
			 //  //dynamic_cast<nxRigidBody *>(mWeapon2Bodypart->mRB)->setTriggerJointMotorTarget(mWeaponRot);//(mountQuat);

			 //  finalPos = weapPos + getPosition();
			 //  mountTransform.setPosition(finalPos);
		  // } else {
			 //  mountTransform = mShapeInstance->mNodeTransforms[mWeapon2MountNode];	
			 //  finalPos = mountTransform.getPosition() + getPosition();
			 //  mountTransform.setPosition(finalPos);
		  // }
		  // mWeapon2->setTransform(mountTransform);
		  // mWeapon2->//setMaskBits(fxRigidBody::fxRigidMoveMask);
	   //}

	   //Thread& st = mScriptThread[0];
	   //int pos = 0;


	}
	//////////////////////////////////////////////////////////////////////////////////

	//updateNodes();

	//if (mReset)
	//{//HERE: need to run through bodyparts and reset them all, as well as changing overall pos.

	//	if (mInitialPosition.length())
	//	{
	//		mCurrPosition = mInitialPosition;
	//		Ogre::Matrix3 m;
	//		mInitialOrientation.setMatrix(&m);

	//		Ogre::Quaternion q(m);
	//		m.setPosition(mCurrPosition);
	//		setTransform(m);
	//		//mIsAnimating = true;
	//		mClearIsAnimating = false;
	//		mStopAnimating = false;
	//		mIsAnimating = true;
	//		stopThread(0);
	//		for (unsigned int j=0; j< mNumBodyParts; j++) 
	//		{
	//			mBodyParts[j]->reset();
	//			// ? mBodyParts[j]->updatePositionFromRB();
	//		}
	//		setKinematic();
	//	}
	//	mReset = false;
	//}
	//mPlaylistTick++;
	mCurrTick++;
	//mCurrMS += TickMs;
}
				//Let onRagdollStop in script worry about everything else.
				////HERE: need an angleBetween, or maybe a dot product or cross product, to tell me whether I'm closer to facing
				////right or left, then do the appropriate one.  Later add on back or on front as well.
				//m = mShapeInstance->mNodeTransforms[mBodyParts[2]->mNodeIndex];
				//Ogre::Vector3 eul = m.toEuler();
				//Ogre::Matrix3 mR, mL;
				//Ogre::Vector3 eulR,eulL;
				//eulR.set(0.0,mDegToRad(-90.0),0.0); eulL.set(0.0,mDegToRad(90.0),0.0);
				//mR.set(eulR);  mL.set(eulL);
				//Ogre::Vector3 test,testL,testR,testC,chest; 
				//test.set(0.0,0.0,1.0);
				//chest.set(0.0,1.0,0.0);
				//mR.mulP(test,&testR); mL.mulP(test,&testL);
				//m.mulP(chest,&testC);
				//float dotR,dotL;
				//dotR = mDot(testR,testC); dotL = mDot(testL,testC);

				//float rAngle,lAngle;
				//Ogre::Quaternion qC(m);
				//Ogre::Quaternion qR(mR);
				//Ogre::Quaternion qL(mL);
				//rAngle = qC.angleBetween(qR);
				//lAngle = qC.angleBetween(qL);

				//TEMP, this doesn't really work.  At least it sort of randomizes them.
				//if (getDamageLevel()<100.0)
				//ARGh, mDamage gets zeroed out somewhere...
				//if (0)
				//{
				//if ( (mCurrSeq>-1) &&
				//	(mCurrSeq < getShapeInstance()->getShape()->sequences.size()))
				//{
				//	mSequenceEndStep = mCurrMS + (getShape()->sequences[mCurrSeq].duration * 1000);
				//	mSequenceStartStep = mCurrMS;
				//	startAnimating(mCurrSeq);

				//} else {
				//	if (getShapeInstance()->getShape()->sequences.size()>1)
				//	{
				//		if ((dotR>dotL)||(getShapeInstance()->getShape()->sequences.size()==2)) 
				//			startAnimating(1);//rSideGetup
				//		else 
				//			startAnimating(2);//lSideGetup
				//	} else startAnimating(0);
				//}
				//}



////////REMOVED:  physics ultraFrames, this functionality is now handled by scene events.
/*
		if  ( mSequenceStartStep && 
			  mSequenceEndStep &&
			  (mCurrMS < mSequenceEndStep) )//run the physics ultraframes
		{//get pos from position
			float pos = (float)(mCurrMS - mSequenceStartStep) / (float)(mSequenceEndStep - mSequenceStartStep);
			int frame = (int)(pos * getShape()->sequences[mCurrSeq].numKeyframes);
			int matterscount = getShapeInstance()->getShape()->getNumMattersNodes(mCurrSeq);
			if (mUltraframeSets.size())
			{
				if (mUltraframeSets[mCurrSeq].frames.size())//First, bail on this if we have no keyframes
					//at all for this sequence.
				{
					for (unsigned int i=0;i<matterscount;i++)
					{//FIX!! Time to start filtering by keyframe type, and possibly node.
						//anyway rewrite this stuff into a for loop...
						Ogre::Vector3 value;
						int target;
						value = Ogre::Vector3::ZERO;
						getUltraframe(mCurrSeq,frame,i,IMPULSE_FORCE,&target,&value);
						if (value.length()>0.001) 
						{
							if (mIsAnimating) stopAnimating();
							setBodypartForce(getShapeInstance()->getShape()->getMattersNodeIndex(mCurrSeq,i),value);
						}

						getUltraframe(mCurrSeq,pos,i,CONSTANT_FORCE,&target,&value);
						if (value.length()>0.001) 
						{
							if (mIsAnimating) stopAnimating();
							setBodypartGlobalForce(getShapeInstance()->getShape()->getMattersNodeIndex(mCurrSeq,i),value);
						}

						getUltraframe(mCurrSeq,pos,i,LOCAL_TORQUE,&target,&value);
						if (value.length()>0.001) 
						{
							if (mIsAnimating) stopAnimating();
							setBodypartTorque(getShapeInstance()->getShape()->getMattersNodeIndex(mCurrSeq,i),value);
						}

						getUltraframe(mCurrSeq,pos,i,GLOBAL_TORQUE,&target,&value);
						if (value.length()>0.001) 
						{
							if (mIsAnimating) stopAnimating();
							setBodypartGlobalTorque(getShapeInstance()->getShape()->getMattersNodeIndex(mCurrSeq,i),value);
						}

						getUltraframe(mCurrSeq,pos,i,MOTOR_TARGET,&target,&value);
						if (value.length()>0.001) 
						{
							if (mIsAnimating) stopAnimating();
							setBodypartMotorTarget(getShapeInstance()->getShape()->getMattersNodeIndex(mCurrSeq,i),value);
						}

						getUltraframe(mCurrSeq,pos,i,SPRING_TARGET,&target,&value);
						if (value.length()>0.001) 
						{
							if (mIsAnimating) stopAnimating();
							setBodypartMotorSpring(getShapeInstance()->getShape()->getMattersNodeIndex(mCurrSeq,i),value,1.0);
						}

						getUltraframe(mCurrSeq,pos,i,IMPULSE_WEAPON_FORCE,&target,&value);
						if (value.length()>0.001) 
						{
							//Con::errorf("FOUND an impulse weapon force!!! target %d force %f %f %f",
								target,value.x,value.y,value.z);
							Ogre::Vector3 muzzlePoint,targetPos,diff;

							//Now:  Found my impulse weapon force and I know my target number.  Next step
							//is to find the playbot associated with that number. 
							SimObject *simObj = Sim::findObject("PlayBotGroup");
							SimGroup *g = (SimGroup *)simObj;
							if (target<g->size()&&(target>0))
							{
								SimObject *simTarget = (SimObject*)(*g)[target-1];//target-1 because 
								mTarget = (fxFlexBody *)simTarget;//users deal with a list that 
								//starts with one, not zero.
							}

							if (!mTarget) return;

							int part = getShapeInstance()->getShape()->getMattersNodeIndex(mCurrSeq,i);
							targetPos = mTarget->mBodyParts[part]->getTransform().getPosition();
							muzzlePoint = mBodyParts[2]->getTransform().getPosition();//mBodyParts[2] = Good enough for now.
							diff = targetPos - muzzlePoint;
							diff.normalize();
							diff *= value.length();
							if (mTarget->mIsAnimating) mTarget->stopAnimating();
							mTarget->setBodypartGlobalForce(part,diff);//danger!  part is not necessarily flexbodypart
						}
					}
				}
			}
		}

		///////////////////////////////////////////

		if (st.thread) 
		{

			int seqnum = getShapeInstance()->getThread(0)->getSequenceNum();
			if ((seqnum < getShapeInstance()->getShape()->sequences.size())&&
				(seqnum < mUltraframeSets.size()))
			{
				pos = getShapeInstance()->getKeyframeNumber(st.thread);

				int matterscount = getShapeInstance()->getShape()->getNumMattersNodes(seqnum);//(0)??
				if (mUltraframeSets.size())
				{
					if (mUltraframeSets[seqnum].frames.size())//First, bail on this if we have no keyframes
						//at all for this sequence.
					{
						for (unsigned int i=0;i<matterscount;i++)
						{//FIX!! Time to start filtering by keyframe type, and possibly node.
							Ogre::Vector3 value;
							int target;
							value = Ogre::Vector3::ZERO;
							//if (
							getUltraframe(seqnum,pos,i,IMPULSE_FORCE,&target,&value);
							if (value.length()>0.001) 
							{
								////Con::errorf("FOUND an impulse force!!! %f %f %f",value.x,value.y,value.z);
								if (mIsAnimating) stopAnimating();
								//setBodypartGlobalForce(getShapeInstance()->getShape()->getMattersNodeIndex(seqnum,i),value);
								setBodypartForce(getShapeInstance()->getShape()->getMattersNodeIndex(seqnum,i),value);
								//setDamageLevel(100.0);//TEMP: get the amount from the gui.  Except, this doesn't work, mDamage gets zeroed out somewhere else. :-(
								//mRagdollStep = ((physManagerCommon *)physManagerCommon::getPM())->mCurrStep;
							}

							getUltraframe(seqnum,pos,i,CONSTANT_FORCE,&target,&value);
							if (value.length()>0.001) 
							{
								////Con::errorf("FOUND a constant force!!! %f %f %f",value.x,value.y,value.z);
								if (mIsAnimating) stopAnimating();
								setBodypartGlobalForce(getShapeInstance()->getShape()->getMattersNodeIndex(seqnum,i),value);
								//mRagdollStep = ((physManagerCommon *)physManagerCommon::getPM())->mCurrStep;
								//setBodypartForce(getShapeInstance()->getShape()->getMattersNodeIndex(seqnum,i),value);
							}//Need global/local options for both of these.

							getUltraframe(seqnum,pos,i,LOCAL_TORQUE,&target,&value);
							if (value.length()>0.001) 
							{
								////Con::errorf("FOUND a bodypart torque!!! %f %f %f",value.x,value.y,value.z);
								if (mIsAnimating) stopAnimating();
								setBodypartTorque(getShapeInstance()->getShape()->getMattersNodeIndex(seqnum,i),value);
								//mRagdollStep = ((physManagerCommon *)physManagerCommon::getPM())->mCurrStep;
							}//Need global/local options for both of these.

							getUltraframe(seqnum,pos,i,GLOBAL_TORQUE,&target,&value);
							if (value.length()>0.001) 
							{
								////Con::errorf("FOUND a bodypart global torque!!! %f %f %f",value.x,value.y,value.z);
								if (mIsAnimating) stopAnimating();
								setBodypartGlobalTorque(getShapeInstance()->getShape()->getMattersNodeIndex(seqnum,i),value);
								//mRagdollStep = ((physManagerCommon *)physManagerCommon::getPM())->mCurrStep;
							}//Need global/local options for both of these.

							getUltraframe(seqnum,pos,i,MOTOR_TARGET,&target,&value);
							if (value.length()>0.001) 
							{
								////Con::errorf("FOUND a motor target!!! %f %f %f",value.x,value.y,value.z);
								if (mIsAnimating) stopAnimating();
								setBodypartMotorTarget(getShapeInstance()->getShape()->getMattersNodeIndex(seqnum,i),value);
								//mRagdollStep = ((physManagerCommon *)physManagerCommon::getPM())->mCurrStep;
							}//Need global/local options for both of these.

							getUltraframe(seqnum,pos,i,SPRING_TARGET,&target,&value);
							if (value.length()>0.001) 
							{
								////Con::errorf("FOUND a spring target!!! %f %f %f",value.x,value.y,value.z);
								if (mIsAnimating) stopAnimating();
								setBodypartMotorSpring(getShapeInstance()->getShape()->getMattersNodeIndex(seqnum,i),value,1.0);
								//mRagdollStep = ((physManagerCommon *)physManagerCommon::getPM())->mCurrStep;
							}//Need global/local options for both of these.

							getUltraframe(seqnum,pos,i,IMPULSE_WEAPON_FORCE,&target,&value);
							if (value.length()>0.001) 
							{
								//Con::errorf("FOUND an impulse weapon force!!! target %d force %f %f %f",
									target,value.x,value.y,value.z);
								Ogre::Vector3 muzzlePoint,targetPos,diff;

								//Now:  Found my impulse weapon force and I know my target number.  Next step
								//is to find the playbot associated with that number. 
								SimObject *simObj = Sim::findObject("PlayBotGroup");
								SimGroup *g = (SimGroup *)simObj;
								////Con::errorf("PlayBotGroup count: %d",g->size());
								if (target<g->size()&&(target>0))
								{
									SimObject *simTarget = (SimObject*)(*g)[target-1];//target-1 because 
									mTarget = (fxFlexBody *)simTarget;//users deal with a list that 
									//starts with one, not zero.
								}

								if (!mTarget) return;

								//getMuzzlePoint(0,&muzzlePoint);//Deal with this later... I need muzzlePoint
								//for my mWeapon, not a regular mounted torque weapon.  For distance shots in
								//first pass, my local origin to target bot's bodypart should be fine.
								//aRgh, this is using matters node index, NOT bodypart index, need to convert.
								//OOPS!  Need to look on myself, not my target, it's MY sequence! (:-P)
								//int part = ((fxFlexBody*)mTarget)->getShapeInstance()->getShape()->getMattersNodeIndex(seqnum,i);
								int part = getShapeInstance()->getShape()->getMattersNodeIndex(seqnum,i);

								//FIX!
								targetPos = mTarget->mBodyParts[part]->getTransform().getPosition();
								muzzlePoint = mBodyParts[2]->getTransform().getPosition();//mBodyParts[2] = Good enough for now.
								diff = targetPos - muzzlePoint;
								diff.normalize();
								//muzzlePoint += diff;//(to get out of our own chest)
								diff *= value.length();
								if (mTarget->mIsAnimating) mTarget->stopAnimating();
								mTarget->setBodypartGlobalForce(part,diff);//danger!  part is not necessarily flexbodypart
								//((nxPhysManager*)mPM)->nxCastRay(muzzlePoint,diff,value.length(),1.0,"","","","");
								//stopAnimating();
								//setBodypartMotorSpring(getShapeInstance()->getShape()->getMattersNodeIndex(seqnum,i),value,1.0);
							}//Need global/local options for both of these.

							getUltraframe(seqnum,pos,i,BODYPART_ACTION,&target,&value);
							if (value.length()>0.001) 
							{//hmm.. this doesn't fit the mold... might have to change the mold.  Instead of 
								//Ogre::Vector3 "value" I need to have the name of an action.

							}
						}
					}
				}
			}
		}
*/

void fxFlexBody::updateTrigger()
{
	//HERE: first need to define trigger dimensions, in FlexBodyData
	//then, down here, whenever we're !mPhysActive, we need to update the position of this trigger

	//bool serv = isServerObject();
	if (mTriggerActor) 
	{
		if (mIsPhysActive)
		{
			NxVec3 nxPos(0,0,0);
			mTriggerActor->setGlobalPosition(nxPos);

			NxMat33 mat;
			//if (mDataBlock->mTriggerOrientation.x) mat.rotX(mDataBlock->mTriggerOrientation.x * (NxPi/180.0));
			//else if (mDataBlock->mTriggerOrientation.y) mat.rotY(mDataBlock->mTriggerOrientation.y * (NxPi/180.0));
			//else if (mDataBlock->mTriggerOrientation.z) mat.rotZ(mDataBlock->mTriggerOrientation.z * (NxPi/180.0));
			//else mat.id();
			mat.id();
			mTriggerActor->setGlobalOrientation(mat);
		}
		else
		{
         NxVec3 nxPos(mCurrPosition.x,mCurrPosition.y,mCurrPosition.z+0.2);//0.2??
			mTriggerActor->setGlobalPosition(nxPos);

			NxMat33 mat;
			//if (mDataBlock->mTriggerOrientation.x) mat.rotX(mDataBlock->mTriggerOrientation.x * (NxPi/180.0));
			//else if (mDataBlock->mTriggerOrientation.y) mat.rotY(mDataBlock->mTriggerOrientation.y * (NxPi/180.0));
			//else if (mDataBlock->mTriggerOrientation.z) mat.rotZ(mDataBlock->mTriggerOrientation.z * (NxPi/180.0));
			//else mat.id();
			mat.id();
			mTriggerActor->setGlobalOrientation(mat);
			//mTriggerActor->setGlobalOrientation();
		}
	}
}

void fxFlexBody::onTrigger()
{
	////Con::errorf("triggering flexbody");
	//if (!mIsPlayer) if (mIsKinematic) clearKinematic();
	//mTriggerTimeMS = Platform::getVirtualMilliseconds();
}

void fxFlexBody::onTrigger(iPhysUser *other,int action,int id)
{
	//So... here is where we need to do the complicated trigger logic, NOT down in nxPhysManager.  If we're an rts unit, 
	//we might even want to have an onTrigger function down there that this one calls, because only rts units need to fake 
	//their torque collision.

	switch (other->mEntityType)
	{
	case PHYS_STATIC:
		////Con::errorf("flexbody trigger colliding with PHYS_STATIC");
		break;
	case PHYS_RIGID_BODY:
		////Con::errorf("flexbody trigger colliding with PHYS_RIGID_BODY");
		break;
	case PHYS_FLEX_BODY:
		//if (!strcmp(typeid(*this).name(),"class AIGuard")) 
		//{
		//	Ogre::Vector3 otherPos = dynamic_cast<fxFlexBody*>(other)->mCurrPosition;
		//	Ogre::Vector3 diff = mCurrPosition - otherPos;
		//	diff.normalize();
		//	diff *= 0.2;
		//	//if (dynamic_cast<RTSUnit*>(this)->mPushVector.length()==0.0)
		//	//	dynamic_cast<RTSUnit*>(this)->mPushVector = diff;

		//	//Con::errorf("flexbody trigger colliding with PHYS_FLEX_BODY: away vector %f %f %f",diff.x,diff.y,diff.z);
		//}
		break;
	case PHYS_FLEX_BODY_PART:
		if (!mIsPlayer) 
		{
			clearKinematic();
			mStopAnimating = true;
		}
		break;
	}
}
/*
enum physEntityType 
{
   PHYS_TERRAIN = 0,
   PHYS_INTERIOR,
   PHYS_STATIC,
   PHYS_RIGID_BODY,
   PHYS_FLEX_BODY,
   PHYS_FLUID,
   PHYS_CLOTH,
   PHYS_TRIGGER
};
*/
void fxFlexBody::addForceAtPos(const Ogre::Vector3 &force,const Ogre::Vector3 &pos)
{

}

void fxFlexBody::lockTractorBeam(int type)
{

}

void fxFlexBody::onCollision(iPhysUser *other,int action)
{
	//if (action==1) //Special case for ecstasy motion, action==1 means we're selecting the body, not applying force.
	//{
	//	//Con::executef("setTweakerBotOne",scriptThis());
	//	gTweakerOne = (fxFlexBody *)this;
	//	return;
	//}

	//physEntityType otherType;
	//if (other) otherType = other->mEntityType;
	//else otherType = PHYS_TYPE_UNDEFINED;

	////either this is a non-other-body collision (grav gun, bullet, etc)
	//// or this is a collision with something besides my weapon, so react.
	////if (otherType==PHYS_RIGID_BODY)||(otherType==PHYS_FLEX_BODY)
	//NetConnection *toServer = NetConnection::getConnectionToServer();
	//NetConnection *toClient = NetConnection::getLocalClientConnection();
	//int weaponID = -1;
	//fxRigidBody *clientWeapon = NULL;
	//if (mWeapon) {
	//	weaponID = toClient->getGhostIndex(mWeapon);
	//	clientWeapon = (fxRigidBody *)(toServer->resolveGhost(weaponID));
	//}
	//if ((!other)||(((otherType==PHYS_RIGID_BODY)||(otherType==PHYS_FLEX_BODY_PART))&&
	//	(other != clientWeapon)))//&&(other->mIsStriking)//(other != mWeapon)
	//{
	//	mTriggerTimeMS = 0;
	//	//if (mIsAnimating) { //((mIsAnimating)&&(!mIsPlayer)) {

	//	//HERE: need logic about whether or not this is a damage-inflicting type of object or not.
	//	//  Also, what mass category we are dealing with, of three: 
	//	//    a) much huger than me, so I do what it does and don't affect it at all
	//	//    b) much smaller than me, so it does what I do and doesn't affect me at all
	//	//    c) in the same range as me, so we affect each other.
	//	// Initially, this can be specified manually, until I figure out how to get mass out of a 
	//	// physx rigid body.

	//	if (!mIsPlayer)
	//	{
	//		if (other)
	//		{
	//			if ((other->mIsInflictor)&&(mCurrTick>60)) 
	//			{
	//				//NOW - instead of global stopAnimating, we need local stopAnimating.
	//				//mStopAnimating = true;//Doing it in flexbodypart instead.

	//				//killing everything in the level on the first tick.
	//				////Con::printf("inflictor collision with flexbody");
	//				//Con::executef(this,"onCollisionInflictor",scriptThis());
	//			} else {
	//				////Con::printf("non-inflictor collision with flexbody");
	//				//Con::executef(this,"onCollisionNonInflictor",scriptThis());
	//			}
	//			
	//		} //else mStopAnimating = true;//Here: have to have this on for force gun to work, but will cause 
	//		//spontaneous death where you don't always want it.  This is where we need much smarter collision logic.
	//	}
	//	//Q: is there a reason to do mStopAnimating = true instead of calling stopAnimating()?
	//	mRagdollStep = ((physManagerCommon *)physManagerCommon::getPM())->mCurrStep;

	//	//}//TEMP! Had to get rid of this test in order to reset flexbodies.
	//	//If mIsAnimating is not true, then mStopAnimating doesn't get set, and the guy stays kinematic when there is a collision.
	//	//However, if I set mIsAnimating, then somehow the reset gets borked and he resets to his last known flying-through-the-air 
	//	//position, instead of his initial position/orientation.  (??)  
	//	//The drawback to commenting this out is when we actually are animating, i.e. walking around in kinematic mode.  Every time
	//	//a bot touches something now, he will collapse.
	//}
	////if ((otherType==PHYS_FLEX_BODY)&&(mDamageState!=Enabled))
	////{//we're a corpse, and somebody kicked us.
	////mTriggerTimeMS = 0;
	////if (mIsAnimating) { //((mIsAnimating)&&(!mIsPlayer)) {
	////	mStopAnimating = true;
	////}
	////} //hmm, not working, we die anyway.

	//if ((mIsPlayer)&&(mStopAnimating==true))
	////if(0)
	//{
	//	//NetConnection *toServer = NetConnection::getConnectionToServer();
	//	//NetConnection *toClient = NetConnection::getLocalClientConnection();

	//	//int myId = getId();

	//	//AIGuard *mySB;

	//	//mySB = ((AIGuard *)(toServer->resolveObjectFromGhostIndex(getId())));
	//	//int sID = mySB->getId();
	//	/*
	//	AIGuard * serverParent = NULL;
	//	if (bool(mServerObject))
	//	{
	//		serverParent = dynamic_cast<AIGuard* >((NetObject *)mServerObject);
	//		
	//		
	//		//{//HERE: limit this to amount of damage carried by whatever collided with us, or some other determination
	//		//so we can not always kill, and instead phase back into animation after partial ragdoll on limited set of bodyparts.
	//		//For now, though, we are restricting onCollision elsewhere, so it only gets called when we are actually being killed.
	//		if (serverParent)
	//		{
	//			if (serverParent->getDamageValue()!=1.0) 
	//			{
	//				////Con::executef(serverParent,2,"onKill",scriptThis());
	//				//Con::errorf("applying server damage from onCollision, step %d",((physManagerCommon *)physManagerCommon::getPM())->mCurrStep);
	//				serverParent->applyDamage(1000);
	//				//serverParent->stopMove();
	//				
	//			}
	//		}
	//	} 
	//	else if (getDamageValue()!=1.0) 
	//	{
	//		applyDamage(1000);
	//		//Con::errorf("applying client damage from onCollision, step %d",((physManagerCommon *)physManagerCommon::getPM())->mCurrStep);
	//	}
	//	*/
	//}
}

void fxFlexBody::updateParts()
{
	for (unsigned int i = 0; i < mNumBodyParts; i++) 
	{
		if (mBodyParts[i]->mIsKinematic)
		{
			mBodyParts[i]->updatePositionToRB();
			mBodyParts[i]->updateVelocityToRB();
		}
	}
}

void fxFlexBody::updateForces()
{
	for (unsigned int i = 0; i < mNumBodyParts; i++) 
	{
        mBodyParts[i]->updateForcesToRB();
	}
}

/*void fxFlexBody::fixRigidBodies()
{
   NxQuat q;
   Ogre::Quaternion mDefI;
 
   mPM->mScene->fetchResults(NX_RIGID_BODY_FINISHED,true);

   for (unsigned int i=0;i<mNumBodyParts;i++) {
      mDefI = mDefaults[i];
      mDefI.inverse();
      q.setXYZW(mDefI.x,mDefI.y,mDefI.z,-mDefI.w);
      NxMat33 nxMat(q);
      if (mBodyParts[i]->mRB->mActor)
         mBodyParts[i]->mRB->mActor->moveGlobalOrientation(nxMat);
      else //Con::errorf("no actor!!");
   }
   mPM->mScene->simulate(mPM->mStepTime);
}*/

void fxFlexBody::startAnimating(int seq)
{//Move to shapebase?  need flexbody stuff here though.
	//setThreadSequence(0,seq);//??  

	//mIsAnimating = true;
	//setKinematic();
	//mClearIsAnimating = false;
	//mStopAnimating = false;

	////mShapeInstance->mIsAnimating = true;//ECSTASY -- TESTING...
	//const TSShape *kShape = getShape();
	//String seqName = kShape->getName(kShape->sequences[seq].nameIndex);

	//SimDataBlockGroup *g = Sim::getDataBlockGroup();
	//for (unsigned int c=0;c<g->size();c++) {
	//	if (!strcmp(( (SimDataBlock *)(*g)[c])->getClassName(),"physGroundSequenceData")) 
	//	{
	//		physGroundSequenceData *pd = (physGroundSequenceData *)(*g)[c];
	//		if ((pd->mFlexBodyData==mDataBlock)&&(strcmp(pd->mSeqName,seqName.c_str()))) 
	//		{
	//			mIsGroundAnimating = true;
	//			mGroundSequenceData = pd;
	//			mGroundNode = pd->mNodes[0];
	//			//Con::errorf("ground node: %d",mGroundNode);
	//			mGroundStep = 0;
	//			if (mGroundNode>=0) 
	//			{
	//				mGroundVector = mShapeInstance->mNodeTransforms[mGroundNode].getPosition() + getPosition();
	//				Point2F twoDPos(mGroundVector.x,mGroundVector.y);
	//				float z = mPM->getTerrHeight(twoDPos);//DANGER you can't stand on interiors if you do this.
	//				mGroundVector.z = z;//better to do a raycast straight down that collides with everything. 
	//			}
	//		}
	//	}
	//}
}

void fxFlexBody::startAnimatingAtPos(int seq,float pos)
{
	//if (seq>-1)
	//{
	//	int oldSeq = getThreadSequence(0);
	//	float oldPos = 0.0;
	//	if (oldSeq==seq)
	//	{
	//		TSThread *thr;
	//		thr = getScriptThread(0)->thread;
	//		oldPos = getThreadPos(0);
	//		fxFlexBody *servFB = dynamic_cast<fxFlexBody* >((NetObject *)mServerObject);
	//		Ogre::Vector3 servPos =  servFB->getPosition();
	//		Ogre::Matrix3 oldGroundMat,newGroundMat;
	//		thr->getGround(oldPos,&oldGroundMat);
	//		thr->getGround(pos,&newGroundMat);
	//		Ogre::Matrix3 oldGroundMatInv = oldGroundMat;
	//		oldGroundMatInv.inverse();
	//		Ogre::Matrix3 groundMatDiff = newGroundMat * oldGroundMatInv;
	//		Ogre::Vector3 eul1,eul2,eul3;
	//		eul1 = newGroundMat.toEuler();
	//		eul2 = oldGroundMatInv.toEuler();
	//		eul3 = groundMatDiff.toEuler();
	//		//Ogre::Matrix3 groundMatDiff = oldGroundMatInv * newGroundMat;
	//		Ogre::Matrix3 newTransform = servFB->getTransform() * groundMatDiff;//_maybe_...??
	//		//Ogre::Matrix3 newTransform = groundMatDiff * servFB->getTransform();
	//		servFB->setTransform(newTransform);

	//		Ogre::Vector3 oldPoint = oldGroundMat.getPosition();
	//		Ogre::Vector3 newPoint = newGroundMat.getPosition();
	//		Ogre::Vector3 groundDiff = newPoint - oldPoint;
	//		Ogre::Vector3 rotGroundDiff;
	//		//Ogre::Matrix3 rotOnlyTransform = newTransform;
	//		//Ogre::Quaternion rotOnlyQuat(newTransform);
	//		//Ogre::Quaternion rotOnlyQuat(mGroundStartXform);
	//		//rotOnlyQuat.mulP(groundDiff,&rotGroundDiff);
	//		
	//		//HERE: CAREFUL! Need to maintain mInitialOrientation now, every time you start or repeat a sequence.
	//		//Only got it covered now when you select a sequence from sequences list, not other ways.
	//		mInitialOrientation.mulP(groundDiff,&rotGroundDiff);
	//		
	//		//rotOnlyTransform.setPosition(Ogre::Vector3(0,0,0));
	//		//rotOnlyTransform.mulP(groundDiff,&rotGroundDiff);//oh no, whoops
	//		////Con::printf("servPos %3.2f %3.2f %3.2f, groundDiff %3.2f %3.2f %3.2f, rotGroundDiff %3.2f %3.2f %3.2f",
	//		//	servPos.x,servPos.y,servPos.z,groundDiff.x,groundDiff.y,groundDiff.z,rotGroundDiff.x,rotGroundDiff.y,rotGroundDiff.z);
	//		servPos += rotGroundDiff;
	//		//servPos += groundDiff;
	//		servFB->setPosition(servPos);
	//		//HERE: need to rotate groundDiff, and also apply Z rotation to actor.

	//		servFB->//setMaskBits(MoveMask);
	//		////Con::printf("serverPos %3.2f %3.2f %3.2f, new ground %f, old ground inverse %f, diff %f",
	//		//	servPos.x,servPos.y,servPos.z,mRadToDeg(eul1.z),mRadToDeg(eul2.z),mRadToDeg(eul3.z));
	//	}
	//	stopThread(0);
	//	setThreadPos(0,pos);
	//	setThreadSequence(0,seq,true,pos);//??  

	//	//HERE: if you are already in this sequence, then compare your current position to the new
	//	//position, and get a GT difference, apply it to your position

	//	mIsAnimating = true;
	//	setKinematic();
	//	mClearIsAnimating = false;
	//	mStopAnimating = false;

	//	//Here do ground sequence stuff
	//}
}

void fxFlexBody::stopAnimating()
{
 //  //stopThread(0);//ECSTASY -- TESTING...
 //  //if (isServerObject()) return;

 //  for (unsigned int i=1;i<mNumBodyParts;i++)//Should this include part 0?  Apparently not, 
 //  {//this seems to have been creating the most recent "ragdoll render glitch".
	//   mBodyParts[i]->updatePositionFromRB();
	//	//mBodyParts[i]->clearKinematic();//?
 //  }

 //  mRagdollStep = ((physManagerCommon *)physManagerCommon::getPM())->mCurrStep;

	//mIsKinematic = false;
	//clearKinematic();
	////Con::printf("cleared kinematic!!");

 //  mIsGroundAnimating = false;
 //  mClearIsAnimating = true;
 //  mClearBodyAnimating = true;
 //  mStopAnimating = false;
	//
	////mIsThinking = false;//HERE: 

	//clearJointMotors();

 //  //mShapeInstance->mIsAnimating = false;//ECSTASY -- TESTING

 //  //startThinking();
 //  //if (mDataBlock->mGA && !mIsThinking)
 //  //{
	//   ////Con::errorf("GA starting to think!");
	//   //mIsThinking = true;
	//   //mActionUser->mActionState = 0;//GA_ACTION_NONE;
	//   //saveResets();
	//   ////Con::executef(this,"schedule","30","startThinking");
 //  //}
}

void fxFlexBody::getMesh()
{
   //if (isServerObject()) return;

//   fxFlexBodyPartData *partData[MAX_FLEX_PARTS];
//   unsigned int numPartData = 0;
//   // HERE: have to add flexbodypart, with the datablock set by partData[k] 
//   unsigned int c;
//
//   SimDataBlockGroup *g = Sim::getDataBlockGroup();
//   for (c=0;c<g->size();c++) {
//	   if (!strcmp(( (SimDataBlock *)(*g)[c])->getClassName(),"fxFlexBodyPartData")) {
//		   fxFlexBodyPartData *pd = (fxFlexBodyPartData *)(*g)[c];
//		   if (!mIsPlayer) {
//			   if (pd->mFlexBodyData == mDataBlock) {
//				   partData[numPartData++] = (fxFlexBodyPartData *)(*g)[c];
//			   }
//		   } else {
//			   if (pd->mPlayerData == (SimObject *)mDataBlock) {
//				   partData[numPartData++] = (fxFlexBodyPartData *)(*g)[c];
//			   }   
//		   }
//	   }
//   }
//
//   //TSSkinMesh *mesh = (TSSkinMesh *)mShapeInstance->mMeshObjects[0].getMesh(0);
//	unsigned int numMeshes = mShapeInstance->getShape()->meshes.size();//25 meshes in brokeass human.
//	int v,iV,bI,nI;
//	for (unsigned int m = 0;m<numMeshes;m++)
//	{
//		TSSkinMesh *kMesh = (TSSkinMesh *)mShapeInstance->getShape()->meshes[m];
//		if (kMesh)
//		{
//			v = kMesh->verts.size();
//			iV = kMesh->batchData.initialVerts.size();
//			bI = kMesh->boneIndex.size();
//			nI = kMesh->batchData.nodeIndex.size();
//			//if ((iV<10000000)&&(iV>-10000000)) //weak, sanity checking for skinmesh
//			//	//Con::errorf("onAdd: initialVerts size = %d, vertIndex size = %d, boneIndex size = %d, nodeIndex size = %d, nodes %d",
//			//	iV,kMesh->vertexIndex.size(),bI,nI,mShapeInstance->getShape()->nodes.size());
//		}
//	}
//   //TSSkinMesh *mesh = (TSSkinMesh *)mShapeInstance->getShape()->meshes[0];
//	//TSSkinMesh *mesh = (TSSkinMesh *)mShapeInstance->getShape()->meshes[1];//TEMP!!
//	
//   ////Con::errorf("%s rigid body meshes: %d",mDataBlock->mMeshObject,mShapeInstance->getShape()->meshes.size());
//   if ((numPartData)) {
//	   ////Con::errorf("onAdd: initialVerts size = %d, boneIndex size = %d, nodeIndex size = %d, nodes %d",mesh->initialVerts.size(),mesh->boneIndex.size(),mesh->nodeIndex.size(),mShapeInstance->getShape()->nodes.size());
//	   //Ogre::Vector3 basePos = mShapeInstance->getShape()->defaultTranslations[0] + mShapeInstance->getShape()->defaultTranslations[1];
//	   //for (unsigned int jj=0;jj<mesh->initialVerts.size();jj++) {
//		  //didn't work at all
//		  ////Con::errorf("old initialVert: %f %f %f",mesh->initialVerts[jj].x,mesh->initialVerts[jj].y,mesh->initialVerts[jj].z);
//		  //mesh->initialVerts[jj] -= basePos;
//		  ////Con::errorf("new initialVert: %f %f %f",mesh->initialVerts[jj].x,mesh->initialVerts[jj].y,mesh->initialVerts[jj].z);
//	   //}
//
//		int shapeNodes = mShapeInstance->getShape()->nodes.size();
//
//	   for (unsigned int jj=0;jj<MAX_FLEX_PARTS;jj++) {
//		   mIndexBones[jj] = -1;
//		   //? for (unsigned int jk=0;jk<mesh->nodeIndex.size();jk++) {
//			//?   if (mesh->nodeIndex[jk]==jj) mIndexBones[jj] = jk;	
//		   //? }
//			/*
//			for (unsigned int jm = 0;jm<numMeshes;jm++)
//			{
//				TSSkinMesh *mesh = (TSSkinMesh *)mShapeInstance->getShape()->meshes[jm];
//				if (mesh)
//				{
//					iV = mesh->initialVerts.size();
//					if ((iV<1000000)&&(iV>-1000000))//WEAK, this is just to make sure it's a valid skinmesh
//					{
//						for (unsigned int jk=0;jk<mesh->nodeIndex.size();jk++) 
//						{
//							if (mesh->nodeIndex[jk]==jj) mIndexBones[jj] = mesh->nodeIndex[jk];	// = jk;
//						}// WAIT, now we're just saying "mIndexBones[jj] = jj;" ... ?
//						//What we need is for that to be the bodypart number, NOT the local mesh node number.
//						//for (unsigned int jk=0;jk<mesh->boneIndex.size();jk++) {
//						//	//Con::errorf("boneIndex[%d] = %d",jk,mesh->boneIndex[jk]);
//						//}
//					}
//				}
//			} */        
//	   }
//
//   }
//
//
//   if ((mPM->getType() == PHYS_NX)&&(numPartData))
//   {//HERE: fix this to work with trimesh if type==ode
//
//	   	//The new, 1.8 way to do it:
//	   const String myPath = mShapeInstance->mShapeResource.getPath().getPath();
//	   const String myFileName = mShapeInstance->mShapeResource.getPath().getFileName();
//	
//	   char fullpath[255];
//	   //sprintf(fullpath,255,"%s/%s",mShapeInstance->getShape()->mSourceResource->path,mShapeInstance->getShape()->mSourceResource->name);
//	   sprintf(fullpath,255,"%s/%s",myPath.c_str(),myFileName.c_str());
//
//	   for (unsigned int i=0;i<((nxPhysManager *)mPM)->mNumMeshBodies;i++) {
//		   if (!strcmp(((nxPhysManager *)mPM)->mMeshBodies[i]->shapeName,fullpath)) {
//			   mMeshBody = i;
//			   mStartMesh = ((nxPhysManager *)mPM)->mMeshBodies[i]->startMesh;
//			   mNumMeshes = ((nxPhysManager *)mPM)->mMeshBodies[i]->numMeshes;
//		   }
//	   }
//
//	   if (mStartMesh==-1)  {
//		   ((nxPhysManager *)mPM)->mMeshBodies[((nxPhysManager *)mPM)->mNumMeshBodies] = new physMeshBody;
//		   ((nxPhysManager *)mPM)->mMeshBodies[((nxPhysManager *)mPM)->mNumMeshBodies]->shapeName = StringTable->insert(fullpath);
//		   ((nxPhysManager *)mPM)->mMeshBodies[((nxPhysManager *)mPM)->mNumMeshBodies]->startMesh = ((nxPhysManager *)mPM)->mNumMeshes;
//		   ((nxPhysManager *)mPM)->mMeshBodies[((nxPhysManager *)mPM)->mNumMeshBodies]->numMeshes = numPartData;
//		   mMeshBody = ((nxPhysManager *)mPM)->mNumMeshBodies;
//		   mStartMesh = ((nxPhysManager *)mPM)->mMeshBodies[((nxPhysManager *)mPM)->mNumMeshBodies]->startMesh;
//		   mNumMeshes = ((nxPhysManager *)mPM)->mMeshBodies[((nxPhysManager *)mPM)->mNumMeshBodies]->numMeshes;
//		   ((nxPhysManager *)mPM)->mNumMeshes += mNumMeshes;
//		   ((nxPhysManager *)mPM)->mNumMeshBodies++;
//	   }
//   }
//
//   //Con::errorf("Made a fxFlexBody ID = %d",getId());
//
//   for (int k = 0; k < numPartData; k++)
//   {
//	   mBodyParts[k] = new fxFlexBodyPart(k);
//	   mBodyParts[k]->mFlexBody = this;
//	   mBodyParts[k]->mMesh = mStartMesh + k;
//	   mBodyParts[k]->onNewDataBlock(partData[k],false);//true?
//	   //mBodyParts[k]->onAdd();
//	   if (mBodyParts[k]->registerObject()==false) //Con::errorf("failed to register a fxFlexBodyPart");
//	   //else //Con::errorf("sucessfully registered fxFlexBodyPart");
//
//	   mNumBodyParts++;
//
//	   if (k==0) {
//		   mFirstNode = mBodyParts[k]->mNodeIndex;
//	   }
//		mIndexBones[mBodyParts[k]->mNodeIndex] = k;
//		////Con::errorf("mIndexBones[%d] = %d",mBodyParts[k]->mNodeIndex,k);
//   }
///*
////NOPE!!  Instead of a separate array, make a physTrigger object be a component of 
////all fxFlexBodyParts, turned on by IsInflictor property.  
//   //HERE: check for triggerParts, set those up hardcode style for now in flexbody onAdd before we get here.
//	for (int k=0;k<mNumTriggerParts;k++)
//	{
//		//make the trigger actors, and start them out using position
//		//Con::errorf("I would be making triggers for the hands and feet right now, but I have to schedule it");
//		//Con::errorf("through the physManager to avoid interrupting the simulation.  triggerPart ID: %d",mTriggerParts[k].ID);
//
//	}
//*/
//	//NOW: this is the place to read the sdkRot values.  After bodyparts have been created, but before they've created their convexes.   
//	if (mDataBlock->mSDK)
//	{//load up default rotations from dtsSDK project that made the model
//		const String myPath = mShapeInstance->mShapeResource.getPath().getPath();
//		const String myFileName = mShapeInstance->mShapeResource.getPath().getFileName();
//	
//		////Con::errorf("my path %s, my name %s",mShapeInstance->getShape()->mSourceResource->path,mShapeInstance->getShape()->mSourceResource->name);
//		//Con::errorf("my path %s, my name %s",myPath.c_str(),myFileName.c_str());
//
//		FILE *fp = NULL;
//		char filename[255],buf[255];
//		unsigned int i = 0;
//		Ogre::Quaternion rot;
//
//		sprintf(filename,"%s/%s.rot",myPath.c_str(),myFileName.c_str(),mShapeName);
//		fp = fopen(filename,"r");
//		if (fp==NULL) //Con::errorf("ERROR: can't open rotation file '%s' for an SDK/LParser model!",filename);
//		else
//		{
//			while (fgets(buf,255,fp))
//			{
//				sscanf(buf,"%d: (%g,%g,%g,%g);",&i,&rot.x,&rot.y,&rot.z,&rot.w);
//				mBodyParts[i]->setSDKRot(rot);
//
//				Ogre::Matrix3 kMat;
//				mBodyParts[i]->mSDKRot.setMatrix(&kMat);
//				kMat.inverse();
//				Ogre::Quaternion kMatInv(kMat);
//				
//				//This way gets all fucked up, shear.  Values above 1.0.
//				//Ogre::Quaternion kMatInv = rot;
//				//kMatInv.inverse();
//				
//				mBodyParts[i]->setSDKInverse(kMatInv);
//
//
//				mBodyParts[0]->mSDKInverse.setMatrix((Ogre::Matrix3 *)&(mBodyParts[i]->getTransform()));
//				Ogre::Quaternion obj2w(mBodyParts[i]->getTransform());
//				//Con::errorf("starting obj2world: %3.2f %3.2f %3.2f %3.2f",obj2w.x,
//					obj2w.y,obj2w.z,obj2w.w);
//
//			}
//			fclose(fp);
//		}
//	}
//
//   for (int k = 0; k < mNumBodyParts; k++)
//   {
//	   mBodyParts[k]->setupRigidBody();
//   }
}

void fxFlexBody::getChildNodes()
{
   //if (isServerObject()) return;

   //for (unsigned int k = 0; k < mNumBodyParts; k++)
   //{
   //   if (strlen(mBodyParts[k]->mDataBlock->mChildNodeName)>0) {
   //      int kChild = mShapeInstance->getShape()->findNode(mBodyParts[k]->mDataBlock->mChildNodeName);
   //      if (kChild>-1) {
   //         mBodyParts[k]->mChildIndex = kChild;
   //         mBodyParts[k]->mChildBodyPart = getBodyPart(kChild);
   //      }
   //   }
   //}
}

void fxFlexBody::getNamedNodes()
{
   //if (isServerObject()) return;

	////Two passes:
	////First get the node index
	////then run through and grab the bodypart index.

	//if (strlen(mDataBlock->mHeadNodeName)>0) 
	//	mHeadIndex = mShapeInstance->getShape()->findNode(mDataBlock->mHeadNodeName);
	//if (strlen(mDataBlock->mNeckNodeName)>0) 
	//	mNeckIndex = mShapeInstance->getShape()->findNode(mDataBlock->mNeckNodeName);
	//if (strlen(mDataBlock->mBodyNodeName)>0) 
	//	mBodyIndex = mShapeInstance->getShape()->findNode(mDataBlock->mBodyNodeName);
	//if (strlen(mDataBlock->mRightFrontNodeName)>0) 
	//	mRightFrontIndex = mShapeInstance->getShape()->findNode(mDataBlock->mRightFrontNodeName);
	//if (strlen(mDataBlock->mLeftFrontNodeName)>0) 
	//	mLeftFrontIndex = mShapeInstance->getShape()->findNode(mDataBlock->mLeftFrontNodeName);
	//if (strlen(mDataBlock->mRightBackNodeName)>0) 
	//	mRightBackIndex = mShapeInstance->getShape()->findNode(mDataBlock->mRightBackNodeName);
	//if (strlen(mDataBlock->mLeftBackNodeName)>0) 
	//	mLeftBackIndex = mShapeInstance->getShape()->findNode(mDataBlock->mLeftBackNodeName);

	////Now, run through and get the BodyParts[] index, which is more useful.
	//for (unsigned int k = 0; k < mNumBodyParts; k++)
	//{
	//	if (mBodyParts[k]->mNodeIndex==mHeadIndex)  	 mHeadIndex = k;
	//	if (mBodyParts[k]->mNodeIndex==mNeckIndex)       mNeckIndex = k;
	//	if (mBodyParts[k]->mNodeIndex==mBodyIndex)       mBodyIndex = k;
	//	if (mBodyParts[k]->mNodeIndex==mRightFrontIndex) mRightFrontIndex = k;
	//	if (mBodyParts[k]->mNodeIndex==mLeftFrontIndex)  mLeftFrontIndex = k;
	//	if (mBodyParts[k]->mNodeIndex==mRightBackIndex)  mRightBackIndex = k;
	//	if (mBodyParts[k]->mNodeIndex==mLeftBackIndex)   mLeftBackIndex = k;
	//}
}

void fxFlexBody::setupGA()
{
   //if (isServerObject()) return;

	//int num_competitors = 10;
	//int num_slices = 6;
	//int total_MS = 12000;

	//mActionUser = new gaActionUser();
	//mActionUser->mFlexBody = this;
	//mActionUser->onNewDataBlock((GameBaseData *)(mDataBlock->mActionUserData),false);

	//mActionUser->setup();
	//mActionUser->onAdd();

	////for (int i=1;i<mNumBodyParts;i++) {
	//for (int i=0;i<mActionUser->mNumActiveBodies;i++) {
	//	mActionUser->addBody(mBodyParts[mActionUser->mActiveBodyIndices[i]]->mRB);
	//	//Danger - fix named bodypart indices below if it's broken now, might work as is. 
	//	if (mActionUser->mActiveBodyIndices[i]==mHeadIndex) mActionUser->mHeadIndex = (mActionUser->mNumBodies-1); //mNumActiveBodies
	//	if (mActionUser->mActiveBodyIndices[i]==mBodyIndex) mActionUser->mBodyIndex = (mActionUser->mNumBodies-1); 
	//	if (mActionUser->mActiveBodyIndices[i]==mRightFrontIndex) mActionUser->mRightFrontIndex = (mActionUser->mNumBodies-1); 
	//	if (mActionUser->mActiveBodyIndices[i]==mLeftFrontIndex) mActionUser->mLeftFrontIndex = (mActionUser->mNumBodies-1); 
	//	if (mActionUser->mActiveBodyIndices[i]==mRightBackIndex) mActionUser->mRightBackIndex = (mActionUser->mNumBodies-1); 
	//	if (mActionUser->mActiveBodyIndices[i]==mLeftBackIndex) mActionUser->mLeftBackIndex = (mActionUser->mNumBodies-1); 
	//}	
	//
	////Con::errorf("done setting up GA for %s",mShapeName);
}

fxFlexBodyPart *fxFlexBody::getBodyPart(int childIndex)
{
   //if (isServerObject()) return NULL;

   for (unsigned int k = 0; k < mNumBodyParts; k++)
   {
      if (mBodyParts[k]->mNodeIndex == childIndex) return mBodyParts[k];
   }
   return NULL;
}

fxFlexBodyPart *fxFlexBody::getBodyPart(const char *basenode)
{
   //if (isServerObject()) return NULL;

 //  for (unsigned int k = 0; k < mNumBodyParts; k++)
 //  {
	//   if (!strcmp(mBodyParts[k]->mDataBlock->mBaseNodeName,basenode)) 
	//		return mBodyParts[k];
 //  }

	return NULL;
}

int fxFlexBody::getBodyPartID(const char *basenode)
{
   //if (isServerObject()) return NULL;

   //for (unsigned int k = 0; k < mNumBodyParts; k++)
   //{
	  // if (!strcmp(mBodyParts[k]->mDataBlock->mBaseNodeName,basenode)) 
			//return k;
   //}
   return -1;
}


void fxFlexBody::getBaseRot()
{
  //for (unsigned int i = mFirstNode; i < mShapeInstance->getShape()->nodes.size(); i++) 
  //{
  //   unsigned int rc = 0;
  //   Quat16 defRot16[100];//this allows up to 30 levels in the hierarchy.
  //   Ogre::Quaternion defRot,qTemp,qTemp2; 

  //   defRot = Ogre::Quaternion::IDENTITY;
  //   for (unsigned int j=0;j<30;j++) defRot16[j] = Ogre::Quaternion::IDENTITY;

  //   int kParentID = mShapeInstance->getShape()->nodes[i].parentIndex;
  //   defRot16[rc++] = mShapeInstance->getShape()->defaultRotations[i];

  //   while (kParentID!=-1) {
  //      defRot16[rc++] = mShapeInstance->getShape()->defaultRotations[kParentID];
  //      kParentID = mShapeInstance->getShape()->nodes[kParentID].parentIndex;
  //   }
  //   
  //   for (int j=0;j<rc;j++) {
  //      defRot16[j].getOgre::Quaternion(&qTemp);
  //      defRot *= qTemp;
  //   }
  //   mDefaults[i] = defRot;
  //}
}


void fxFlexBody::zeroForces()
{
   //if (isServerObject()) return;

	for (unsigned int i=0;i<mNumBodyParts;i++) 
	{
		mBodyParts[i]->mCurrForce = Ogre::Vector3::ZERO;
		mBodyParts[i]->mCurrTorque = Ogre::Vector3::ZERO;
		mBodyParts[i]->mGlobalForce = Ogre::Vector3::ZERO;
		mBodyParts[i]->mGlobalTorque = Ogre::Vector3::ZERO;
      if (mBodyParts[i]->mJoint) mBodyParts[i]->mJoint->clearMotor();
      mBodyParts[i]->updateForcesToRB();
	}
}

int fxFlexBody::headCheck()
{
	//Find the Z (height) of a (some length) vector aligned with the head.  
	//Compare the end of this vector with the average height of the body.  
	//If body is too thick for head to get above the center no matter what, 
	//use a longer vector.

	//Then apply force to achieve desired result, but that's on the script side.
	
	//All kinds of ways I could do this, but it's starting to look like I want some 
	//data members of fxFlexBody for common bodypart types, like the head.  Almost 
	//everything alive has a head, and something that could be called a main body. 
	//Soon this kind of thing should move over to a class derived from fxFlexBody,
	//but it can start here.
	
	//Also, rather than a vector, you can just compare the average height of the head 
	//with its own height from last frame, if you are going up, return 1, if equal or 
	//zero, return zero.

	//Con::errorf("doing head check!");

	return 1;
}

void fxFlexBody::headUp()
{
	//Ogre::Vector3 kForce(0,0,800);
	//if (mShapeInstance->getShape()->nodes.size()==55) mBodyParts[mHeadIndex]->setGlobalForce(kForce*1.15);//soldier
 //  else if (mShapeInstance->getShape()->nodes.size()==28) {
 //     mBodyParts[mHeadIndex]->setGlobalForce(kForce*10);//kork
 //     mBodyParts[mHeadIndex]->updateForcesToRB();
 //     //Con::errorf("trying to get kork's head up!");
 //  } else if (mShapeInstance->getShape()->nodes.size()==29){
 //     //mBodyParts[mHeadIndex-1]->setGlobalForce(kForce*25);//giraffe
 //     Ogre::Quaternion q;
 //     Ogre::Matrix3 mat;
 //     mat.set(Ogre::Vector3(0,0,90));
 //     q.set(mat);
 //     //mPM->mScene->fetchResults(NX_RIGID_BODY_FINISHED,true);
	//  mPM->stopPhysics();

 //     mBodyParts[mHeadIndex]->mJoint->setMotorTarget(q);
 //     mBodyParts[mHeadIndex-1]->mJoint->setMotorTarget(q);
 //     mBodyParts[mHeadIndex-2]->mJoint->setMotorTarget(q);

 //     //mPM->mScene->simulate(mPM->mStepTime);
	//  mPM->startPhysics();
 //     //mBodyParts[mHeadIndex-3]->mJoint->setSlerpMotorTarget(q);
 //  }
}


void fxFlexBody::headClear()
{
  //if (mShapeInstance->getShape()->nodes.size()==29){
  //    //mBodyParts[mHeadIndex-1]->setGlobalForce(kForce*25);//giraffe
  //    //Con::errorf("clearing slerp target");

  //    //mPM->mScene->fetchResults(NX_RIGID_BODY_FINISHED,true);
	 // mPM->stopPhysics();

  //    mBodyParts[mHeadIndex]->mJoint->clearMotor();
  //    mBodyParts[mHeadIndex-1]->mJoint->clearMotor();
  //    mBodyParts[mHeadIndex-2]->mJoint->clearMotor();

  //    //mPM->mScene->simulate(mPM->mStepTime);
	 // mPM->startPhysics();
  // }
}

void fxFlexBody::setBodypart(int i)
{
	if (i<0)
		return;

	mBodyParts[i]->mRB->setKinematic(true);
	mBodyParts[i]->mIsKinematic = true;

	//getShapeInstance()->mHandsOffNodes.clear(mBodyParts[i]->mNodeIndex);
}

void fxFlexBody::setKinematic()
{
   for (unsigned int i=0; i < mNumBodyParts; i++) 
   {
	   setBodypart(i);
	   //mBodyParts[i]->mRB->setKinematic(true);
	   //mBodyParts[i]->mIsKinematic = true;// THIS CAUSES BODYPART RENDER GLITCH
		//  But is necessary for carrying inertia forward. (?) Fix later. 11-18-07
		//  Meanwhile, adding bool version below -- inertia not necessary if we're
		//  leaving bodyparts kinematic anyway.
   }
   mIsKinematic = true;
}

void fxFlexBody::clearBodypart(int i)
{	
	//if (i<0) 
	//	return;

	//float timeDelta = (float)(mPM->getTimeDelta());
	//float timeMult;

	//if (timeDelta) timeMult = 1.0 / (timeDelta/1000.0);
	//else timeMult = 32.0;

	//mBodyParts[i]->mRB->setKinematic(false);

	//mBodyParts[i]->mCurrVelocity *= 6.0;//????  This is related to my "conservation of momentum" process.
	////First attempt at velocity in terms of change of position over time did not give visually pleasing results.
	////Speeding it up a bit manually here looked better, but this is awkward, find better solution.
	//
	//mBodyParts[i]->updateVelocityToRB();
	//mBodyParts[i]->updatePositionToRB();
	//
	//if (mBodyParts[i]->mJoint)
	//{
	//	mBodyParts[i]->mJoint->clearMotor();
	//	//mBodyParts[i]->setTorque(Ogre::Vector3(0.0,0.0,0.0));
	//	//mBodyParts[i]->setForce(Ogre::Vector3(0.0,0.0,0.0));
	//	//mBodyParts[i]->setGlobalTorque(Ogre::Vector3(0.0,0.0,0.0));
	//	//mBodyParts[i]->setGlobalForce(Ogre::Vector3(0.0,0.0,0.0));
	//}

	////getShapeInstance()->mHandsOffNodes.set(mBodyParts[i]->mNodeIndex);
	////Ogre::Quaternion q;  q = Ogre::Quaternion::IDENTITY;//Just to initialize them...
	////getShapeInstance()->mNodeHandsOffRotations[mBodyParts[i]->mNodeIndex] = q;
}

void fxFlexBody::deactivateNodePlusChildNodes(int boneIndex)
{   
	////This clears this node and one child, but it calls itself recursively to
	////clear all the children in the whole chain.

	////TEMP: for now, eliminate hip node going ragdoll, this needs a different state
	////where we stop trying to get back to our anim and instead make decisions about 
	////what to do next.  For now just block it.
	//if (boneIndex==0)
	//{
	//	//Here: add a boolean, give it a checkbox, if (mGoFullRagdoll)
	//	//if (mGoFullRagdoll)
	//	if (mPhysicsDamage==1.0)
	//	{
	//		stopAnimating();
	//		clearKinematic();
	//		mIsThinking = false;
	//		//Con::executef(this, "onRagdoll", scriptThis());
	//	} else {
	//		return;
	//	}
	//}
	////if ( (mNoLegRagdoll) && //NoLegRagdoll helps us ignore trivial collisions
	////	((mBodyParts[boneIndex]->mDataBlock->mBodypartChain == PHYS_CHAIN_RIGHT_LEG) ||
	////	(mBodyParts[boneIndex]->mDataBlock->mBodypartChain == PHYS_CHAIN_LEFT_LEG)))
	////{ //Provides a way to keep legs from accidentally popping out of animation because of 
	////	//otherwise unimportant collisions, not related to opponent's attack.
	////	return;
	////}

	//if (mBodyParts[boneIndex]->mRB->getIsKinematic())
	//{
	//	////Con::printf("Deactivating node: %d",boneIndex);
	//	mBodyParts[boneIndex]->mLastDeactivateTick = mBodyParts[boneIndex]->mCurrTick;

	//	mBodyParts[boneIndex]->mRB->setKinematic(false);
	//	getShapeInstance()->mHandsOffNodes.set(mBodyParts[boneIndex]->mNodeIndex);

	//	for (unsigned int i=boneIndex;i<mNumBodyParts;i++)//don't need to start at zero, because 
	//		//parents always come before children in the hierarchy.
	//	{
	//		if (mBodyParts[i]->mParentBodyPart==mBodyParts[boneIndex])
	//			deactivateNodePlusChildNodes(i);
	//	}

	//	//HERE: this part is wrong.  ChainParts is ending up with the wrong part.
	//	//This gets done in tsAnimate, remove it from here, doesn't want to
	//	//be in recursive function anyway.
	//	//int chainIndex = mBodyParts[boneIndex]->mDataBlock->mBodypartChain;
	//	//if (mChainParts[chainIndex])
	//	//{
	//	//	if (boneIndex < mChainParts[chainIndex]->mBoneIndex)
	//	//	{
	//	//		mChainParts[chainIndex] = mBodyParts[boneIndex];
	//	//		//Con::printf("new chainpart for chain %d:  node %d",
	//	//			chainIndex,boneIndex);
	//	//	}
	//	//} else {
	//	//	mChainParts[chainIndex] = mBodyParts[boneIndex];
	//	//	//Con::printf("new chainpart for chain %d:  node %d",
	//	//		chainIndex,boneIndex);
	//	//}
	//}
}

void fxFlexBody::activateNode(int node)//Here node is in 
{//Here: have to put the node back into the sequence, and if it's the current chain part, 
 //pass that on to the child.  If multiple children exist, pass it on to the other chains as well.


	////We don't need the following tests anymore, because we're checking this before we call this function.
	////if (mChainParts[mBodyParts[node]->mDataBlock->mBodypartChain])
	////{
	////	if (mBodyParts[node] == mChainParts[mBodyParts[node]->mDataBlock->mBodypartChain])
	////	{
	//fxFlexBodyPart *kFBP = mFlexBody->mBodyParts[node];
	//int chainIndex = kFBP->mDataBlock->mBodypartChain;
	//
	//if (kFBP->mParentBodyPart->mIsKinematic==false)
	//{
	//	fxFlexBodyPart *kPBP = kFBP->mParentBodyPart;
	//	int parentChain = kPBP->mDataBlock->mBodypartChain;
	//	mFlexBody->mChainParts[parentChain] = kPBP;
	//	if (parentChain != chainIndex)
	//		mFlexBody->mChainParts[chainIndex] = NULL;
	//	////Con::errorf("tried to activate a node whose parent was not kinematic. setting chain %d to %d",
	//	//	parentChain,kPBP->mBoneIndex);
	//	return;
	//}

	//mFlexBody->setBodypart(node);
	//////Con::errorf("setting chainpart from node %d to NULL.",mFlexBody->mChainParts[chainIndex]->mBoneIndex);
	//mFlexBody->mChainParts[chainIndex] = NULL;
	//for (unsigned int i=0;i<mFlexBody->mNumBodyParts;i++)
	//{
	//	fxFlexBodyPart *kCBP = mFlexBody->mBodyParts[i];//ChildBodyPart
	//	if (kCBP->mParentBodyPart == kFBP)
	//	{//this is ONE of this node's direct children, so see if it's on the same chain.
	//		if (kCBP->mDataBlock->mBodypartChain == chainIndex)
	//		{
	//			mFlexBody->mChainParts[chainIndex] = kCBP;
	//			////Con::errorf("new part %d boneIndex %d in the same chain %d",i,kCBP->mBoneIndex,chainIndex);
	//		} else {
	//			mFlexBody->mChainParts[kCBP->mDataBlock->mBodypartChain] = kCBP;
	//			////Con::errorf("firing up part %d in a new chain %d",i,kCBP->mDataBlock->mBodypartChain);
	//		}
	//	}
	//}
	////if (!mFlexBody->mChainParts[chainIndex])
	//	////Con::errorf("closing out chain %d!",chainIndex);

}

void fxFlexBody::clearKinematic()
{
	//physManager *myPM = physManagerCommon::getPM();
	//mPM = myPM;
	if (!mPM) return;

	int isTree = 0;
	//bool client = isClientObject();

	/// HERE: following section being obsoleted by FlexBodyData::mRelaxType
	/////////////////////////////////////////////////
	//Warning: Arbitrary naming convention.
	//if ((!strncmp(mShapeName,"Tree",4)) ||
	//	(!strncmp(mShapeName,"tree",4)))
	//{
	//	isTree = 1;
	//	mSleepThreshold = 0.0;
	//}//might be used elsewhere..?

	//int relaxType = 0;

	//if (!strcmp(mShapeName,"Player")||!strcmp(mShapeName,"player")
	//	||!strcmp(mShapeName,"female")||!strcmp(mShapeName,"male")) 
	//	relaxType = 1;
	//else if (!strcmp(mShapeName,"Wolf")) 
	//	relaxType = 0;
	/////////////////////////////////////////////////


	//float timeDelta = (float)(mPM->getTimeDelta());
	//float timeMult;

	//if (timeDelta) timeMult = 1.0 / (timeDelta/1000.0);
	//else timeMult = 32.0;

	for (unsigned int i=0; i < mNumBodyParts; i++) 
	{		
		//trees lock the base segment
		//if (isTree==1)
		//{
		//	relaxType = 6;
			//if (i==0) continue;
		//}

		//REWRITE ALL - need to make this come from the database 
		switch (mRelaxType)
		{
		case 0: //everything
			//clearBodypart(i);//OOPS!! You shouldn't ever call this except from onWorldStep, 
			//because we can't change properties on the actual RBs until sim is paused!
			mBodyParts[i]->mIsKinematic = false;
			break;

		case 1: //everything but the head
			if (i!=mHeadIndex) mBodyParts[i]->mIsKinematic = false;//clearBodypart(i);
			break;

		case 2:	//everything but the spine -- wolf  (i!=mBodyIndex)&&
			if ((i!=mBodyIndex+5)&&(i!=mBodyIndex+6)&&(i!=mBodyIndex+7)&&
				(i!=mBodyIndex+8))//			if ((i!=mBodyIndex+9))
				mBodyParts[i]->mIsKinematic = false;//clearBodypart(i);	
			break;


		case 3: //arms
			if ((i==mRightFrontIndex)||(i==mRightFrontIndex+1)||(i==mRightFrontIndex+2)||(i==mRightFrontIndex+3)
				||(i==mLeftFrontIndex)||(i==mLeftFrontIndex+1)||(i==mLeftFrontIndex+2)||(i==mLeftFrontIndex+3))
				mBodyParts[i]->mIsKinematic = false;//clearBodypart(i);		
			break;
			
		case 4: //feet
			if ((i==mRightBackIndex+2)
				||(i==mLeftBackIndex+2))
				mBodyParts[i]->mIsKinematic = false;//clearBodypart(i);		
			break;
			
		case 5: //everything but chest
			if ((i!=mBodyIndex)&&(i!=mBodyIndex+1)&&(i!=mBodyIndex+2))	
				mBodyParts[i]->mIsKinematic = false;//clearBodypart(i);
			break;

		case 6: //everything but base
			if ((i!=0))	mBodyParts[i]->mIsKinematic = false;//clearBodypart(i);
			break;
		case 7: //legs
			if ((i==mRightBackIndex)||(i==mRightBackIndex+1)||(i==mRightBackIndex+2)
				||(i==mLeftBackIndex)||(i==mLeftBackIndex+1)||(i==mLeftBackIndex+2))
				mBodyParts[i]->mIsKinematic = false;//clearBodypart(i);
			break;
		case 8: //legs plus hips and abdomen
			if (
				(i==mRightBackIndex)||(i==mRightBackIndex+1)||(i==mRightBackIndex+2)||
				(i==mLeftBackIndex)||(i==mLeftBackIndex+1)||(i==mLeftBackIndex+2)||
				(i==mBodyIndex+1) )//(i==mBodyIndex)||
				mBodyParts[i]->mIsKinematic = false;//clearBodypart(i);
			break;
		case 9: //forearms and shins
			if (
				(i==mRightFrontIndex+2)||(i==mRightFrontIndex+3)||
				(i==mLeftFrontIndex+2)||(i==mLeftFrontIndex+3)||
				(i==mRightBackIndex)||(i==mRightBackIndex+1)||(i==mRightBackIndex+2)||
				(i==mLeftBackIndex)||(i==mLeftBackIndex+1)||(i==mLeftBackIndex+2) )
				mBodyParts[i]->mIsKinematic = false;//clearBodypart(i);
		}
	}
	//if (mWeapon) {
	//	NetConnection *toServer = NetConnection::getConnectionToServer();
	//	NetConnection *toClient = NetConnection::getLocalClientConnection();
	//	//mWeaponID = toClient->getGhostIndex(mWeapon);
	//	int weaponID = toClient->getGhostIndex(mWeapon);
	//	fxRigidBody *clientWeapon = (fxRigidBody *)(toServer->resolveGhost(weaponID));
	//	clientWeapon->mIsKinematic = false;
	//	clientWeapon->mRB->setKinematic(false);//HERE: same problem, do this step in onWorldStep!!
	//}

	//if (mWeaponJoint) delete mWeaponJoint;

	//if (mTriggerActor) { //?
	//	//mTriggerActor = NULL;
	//	mPM->removeFlexBody(this);
	//}

	mIsKinematic = false;
}

void fxFlexBody::setNoGravity()
{
   for (unsigned int i=0; i < mNumBodyParts; i++) 
   {
      mBodyParts[i]->mRB->setNoGravity(true);
   }
   mIsNoGravity = true;
   //setMaskBits(fxFlexBodyMountMask);
}

void fxFlexBody::clearNoGravity()
{
   for (unsigned int i=0; i < mNumBodyParts; i++) 
   {
      mBodyParts[i]->mRB->setNoGravity(false);
   }
   mIsNoGravity = false;
   //setMaskBits(fxFlexBodyMountMask);
}

void fxFlexBody::clearJointMotors()
{
   for (unsigned int i=1; i < mNumBodyParts; i++) 
   {
		if (mBodyParts[i]->mJoint)
			dynamic_cast<nxJoint*>(mBodyParts[i]->mJoint)->clearMotor();
   }
}

//void fxFlexBody::saveResets()
//{
//   for (int i=0;i<mShapeInstance->getShape()->nodes.size();i++) 
//   {
//      mResets[i] = mShapeInstance->mNodeTransforms[i];
//   }
//}

void fxFlexBody::splay()
{
   //Con::errorf("testing drive orientations");
   //mPM->stopPhysics();
   ////mPM->mScene->fetchResults(NX_RIGID_BODY_FINISHED,true);
   //for (unsigned int i=0;i<mNumBodyParts;i++) {
   //   if ((mBodyParts[i]->mJoint)) {
   //      if (mBodyParts[i]->mJoint->getJointType()==PHYS_JOINT_D6) {
   //         Quat16 mD = mShapeInstance->getShape()->defaultRotations[mBodyParts[i]->mNodeIndex];
   //         Ogre::Quaternion mRot;
   //         mD.getOgre::Quaternion(&mRot);
   //         mRot = Ogre::Quaternion::IDENTITY;
   //         //hm, do I want mDefaults or defaultRotations? global or local?
   //         mBodyParts[i]->mJoint->setMotorTarget(mRot);
   //      }
   //   }
   //}
   ////mPM->mScene->simulate(mPM->mStepTime);
   //mPM->startPhysics();
}

void fxFlexBody::setBodypartForce(int i, Ogre::Vector3 &f)
{
	if (i<0)
		return;

	Ogre::Vector3 adjForce;

	adjForce.x = f.x;
	adjForce.y = f.y;
	adjForce.z = f.z;

	mBodyParts[i]->setForce(adjForce);
}

void fxFlexBody::setBodypartGlobalForce(int i, Ogre::Vector3 &f)
{
	//Con::printf("setting bodypart global force! node %d, force %f %f %f, bodyparts %d, actor %s",
	//	i,f.x,f.y,f.z,mNumBodyParts,mBodyParts[i]->mFlexBody->mActorName);
	//return;
	if (i>=0)
	{
		Ogre::Vector3 adjForce;

		adjForce.x = f.x;
		adjForce.y = f.y;
		adjForce.z = f.z;

		mBodyParts[i]->setGlobalForce(adjForce);
	}
	//Con::printf("set bodypart global force.");
}

void fxFlexBody::setBodypartDelayForce(int i, Ogre::Vector3 force)
{
	//mBodyParts[i]->setGlobalForce(force);
	if (i>=0)
	{
		mBodyParts[i]->mRB->setGlobalDelayForce(force);
		//Con::errorf("setting bodypart %d delay force: %f %f %f",i,force.x,force.y,force.z);
	}
	//mBodyParts[i]->mRB->setDelayStep(((physManagerCommon *)physManagerCommon::getPM())->mCurrStep);
}

void fxFlexBody::setBodypartDelayTorque(int i, Ogre::Vector3 force)
{
	//mBodyParts[i]->setGlobalTorque(force);
	//mBodyParts[i]->mRB->setGlobalDelayTorque(force);
	//mBodyParts[i]->mRB->setDelayStep(((physManagerCommon *)physManagerCommon::getPM())->mCurrStep);
}

void fxFlexBody::setBodypartTorque(int i, Ogre::Vector3 &kTorque)
{
	//if (i>=0)
	//{
	//	Ogre::Vector3 maxTorque,minTorque,adjTorque;
	//	maxTorque = mBodyParts[i]->mDataBlock->mTorqueMax;
	//	minTorque = mBodyParts[i]->mDataBlock->mTorqueMin;//Temp: getting rid of minTorque, unnecessary.
	//	if (kTorque.x>=0) adjTorque.x = maxTorque.x * kTorque.x;
	//	else              adjTorque.x = maxTorque.x * kTorque.x;
	//	//else              adjTorque.x = minTorque.x * kTorque.x * -1;
	//	if (kTorque.y>=0) adjTorque.y = maxTorque.y * kTorque.y;
	//	else              adjTorque.y = maxTorque.y * kTorque.y;
	//	//else              adjTorque.y = minTorque.y * kTorque.y * -1;
	//	if (kTorque.z>=0) adjTorque.z = maxTorque.z * kTorque.z;
	//	else              adjTorque.z = maxTorque.z * kTorque.z;
	//	//else              adjTorque.z = minTorque.z * kTorque.z * -1;

	//	//Con::errorf("mBodyPart %d setting torque: %f %f %f",i,adjTorque.x,adjTorque.y,adjTorque.z);
	//	mBodyParts[i]->setTorque(adjTorque);
	//}
}

void fxFlexBody::setBodypartGlobalTorque(int i, Ogre::Vector3 &kTorque)
{
	//if (i>=0)
	//{
	//	Ogre::Vector3 maxTorque,minTorque,adjTorque;
	//	maxTorque = mBodyParts[i]->mDataBlock->mTorqueMax;
	//	minTorque = mBodyParts[i]->mDataBlock->mTorqueMin;//Temp: getting rid of minTorque, unnecessary.
	//	if (kTorque.x>=0) adjTorque.x = maxTorque.x * kTorque.x;
	//	else              adjTorque.x = maxTorque.x * kTorque.x;
	//	if (kTorque.y>=0) adjTorque.y = maxTorque.y * kTorque.y;
	//	else              adjTorque.y = maxTorque.y * kTorque.y;
	//	if (kTorque.z>=0) adjTorque.z = maxTorque.z * kTorque.z;
	//	else              adjTorque.z = maxTorque.z * kTorque.z;

	//	//Con::errorf("mBodyPart %d setting torque: %f %f %f",i,adjTorque.x,adjTorque.y,adjTorque.z);
	//	mBodyParts[i]->setGlobalTorque(adjTorque);
	//}
}

void fxFlexBody::setBodypartMotorTarget(int i, Ogre::Vector3 &kTorque)
{
	//if (i>=0)
	//{
	//	Ogre::Quaternion q;
	//	Ogre::Matrix3 mat;
	//	mat.set(Ogre::Vector3(mDegToRad(kTorque.x),mDegToRad(kTorque.y),mDegToRad(kTorque.z)));
	//	q.set(mat);

	//	if (mBodyParts[i]->mJoint){
	//		mBodyParts[i]->mJoint->setMotorTarget(q);
	//	}
	//}
	//return;	
}

void fxFlexBody::setBodypartMotorSpring(int i, Ogre::Vector3 &kTorque,float force)
{
	//if (i>=0)
	//{
	//	Ogre::Quaternion q;
	//	Ogre::Matrix3 mat;
	//	mat.set(Ogre::Vector3(mDegToRad(kTorque.x),mDegToRad(kTorque.y),mDegToRad(kTorque.z)));
	//	q.set(mat);
	//	if (mBodyParts[i]->mJoint){
	//		//Con::errorf("got a joint!  force %f",force);
	//		mBodyParts[i]->mJoint->setMotorSpring(q,force);
	//	}
	//}
	//return;
}

void fxFlexBody::dropWeapon()
{
	//if (mWeapon)
	//{
	//	mWeapon->resetPosition();
	//	mWeapon = NULL;
	//}
	//return;
}

void fxFlexBody::addWeapon(int weaponID,int mountNode)//const char *bodypartNodeName
{
	//if (mWeapon) dropWeapon();

	////Con::errorf("Adding a weapon, int mount node %d",mountNode);
	//if( Sim::findObject( weaponID, mWeapon ) == false) {
	//	//Con::errorf("couldn't find the weapon!");
	//	return;
	//}
	//else //Con::errorf("weapon I found is located at: %f %f %f, mount node %d",
	//	mWeapon->mCurrPosition.x,mWeapon->mCurrPosition.y,mWeapon->mCurrPosition.z,mountNode);

	//mWeaponMountNode = mountNode;
	//mWeaponID = weaponID;
	////mWeapon->mRB->setPhysUser((iPhysUser*)mWeapon);//this should have been done, but doesn't seem 
	////to be working... my iPhysUser in setupRigidBody & onTrigger is not the fxRigidBody weapon.
	//mWeapon->mFlexBody = this;//There, try THAT! Now everything associated with a flexbody player/NPC/monster 
	////can know it already, whether it be clothing, weapon(s), backpack/bag, helmet... and trigger and
	////collision for each can have their own rules specific to this flexbody vs. other flexbodies.


	////mWeaponPosAdj = mWeapon->mDataBlock->mWeaponPosAdj;
	////Ogre::Vector3 rotAdj = mWeapon->mDataBlock->mWeaponRotAdj;
	//mWeaponPosAdj = mWeapon->mWeaponPosAdj;
	//mWeaponRotAdjA = mWeapon->mWeaponRotAdjA;
	//mWeaponRotAdjB = mWeapon->mWeaponRotAdjB;
	////mWeaponRotAdj.set(Ogre::Vector3(mDegToRad(rotAdj.x),mDegToRad(rotAdj.y),mDegToRad(rotAdj.z)));

	//if (mountNode==39) mWeaponBodypart =  mBodyParts[mRightFrontIndex+3];//??  No, set this by mount node for now
	//else mWeaponBodypart =  mBodyParts[mLeftFrontIndex+3];//later make it accessible on the fly from script.

	////Ogre::Vector3 myPos = mWeaponBodypart->getPosition();
	////Actually, this doesn't even matter here, because onWorldStep starts taking over weapon position 
	////immediately after this.

	//return;
	
}

void fxFlexBody::addWeapon(int weaponID,const char *mountNode)//const char *bodypartNodeName
{
	////Con::errorf("Adding a weapon, char mount node %s",mountNode);
	//TSShape *kShape = mShapeInstance->getShape();
	//if( Sim::findObject( weaponID, mWeapon ) == false) {
	//	//Con::errorf("couldn't find the weapon!");
	//	return;
	//}
	//else //Con::errorf("weapon I found is located at: %f %f %f, mount node %s",
	//	mWeapon->mCurrPosition.x,mWeapon->mCurrPosition.y,mWeapon->mCurrPosition.z,mountNode);

	//mWeaponMountNode = kShape->findNode(mountNode);
	//if (mWeaponMountNode == -1)
	//{
	//	//Con::errorf("AddWeapon can't find mount node: %s",mountNode);
	//	return;
	//}
	//mWeaponID = weaponID;
	////mWeapon->mRB->setPhysUser((iPhysUser*)mWeapon);//this should have been done, but doesn't seem 
	////to be working... my iPhysUser in setupRigidBody & onTrigger is not the fxRigidBody weapon.
	//mWeapon->mFlexBody = this;//There, try THAT! Now everything associated with a flexbody player/NPC/monster 
	////can know it already, whether it be clothing, weapon(s), backpack/bag, helmet... and trigger and
	////collision for each can have their own rules specific to this flexbody vs. other flexbodies.


	////mWeaponPosAdj = mWeapon->mDataBlock->mWeaponPosAdj;
	////Ogre::Vector3 rotAdj = mWeapon->mDataBlock->mWeaponRotAdj;
	//mWeaponPosAdj = mWeapon->mWeaponPosAdj;
	//mWeaponRotAdjA = mWeapon->mWeaponRotAdjA;
	//mWeaponRotAdjB = mWeapon->mWeaponRotAdjB;
	////mWeaponRotAdj.set(Ogre::Vector3(mDegToRad(rotAdj.x),mDegToRad(rotAdj.y),mDegToRad(rotAdj.z)));

	//if (!strcmp(mountNode,"mount0")) mWeaponBodypart =  mBodyParts[mRightFrontIndex+3];//??  No, set this by mount node for now
	//else mWeaponBodypart =  mBodyParts[mLeftFrontIndex+3];//later make it accessible on the fly from script.

	////Ogre::Vector3 myPos = mWeaponBodypart->getPosition();
	//	
	//return;	
}
	//mWeaponBodypart->mInflictMultiplier = mWeapon->mInflictMultiplier;
	//mInflictMultiplier = mWeapon->mInflictMultiplier;//Have to store it relative to flexbody, not bodypart,
	//so we don't go limp when we trigger any other bodypart besides the hand.  FIX!  Change this to a
	//flexbody ID value stored in iPhysUser, so we can leave inflict multiplier alone.
	//Also this won't work when dual wielding different weapons unless they have same inflict multiplier,
	//and won't work against another flexbody carrying same weapon.  FIX!

	////Con::errorf("Found bodyparts[%d]: position %f %f %f",mRightFrontIndex+3,myPos.x,myPos.y,myPos.z);
	//Ogre::Vector3 handPos = kBodypart->mCurrPosition;
	//Ogre::Vector3 mountNodePos = 
	//fxJoint *joint = new fxJoint(...);
	//mWeaponNodeName = StringTable->insert(bodypartNodeName);
	////setMaskBits(fxFlexBodyMountMask);

	//if (kBodypart) {
	//if (0) { //old way

	//	////Con::errorf("hand position: %f %f %f",handPos.x,handPos.y,handPos.z);
	//	fxJointData *jd;
	//	SimDataBlockGroup *g = Sim::getDataBlockGroup();
	//	for (unsigned int c=0;c<g->size();c++) {
	//		if (!strcmp(( (SimDataBlock *)(*g)[c])->getName(),"WeaponJointData")) {
	//			jd = (fxJointData *)(*g)[c];
	//		}
	//	}
	//	if (jd) //Con::errorf("found a weapon joint: %f",jd->mBreakingForce);

	//	Ogre::Matrix3 mountTransform = mShapeInstance->mNodeTransforms[mWeaponMountNode];
	//	//Ogre::Matrix3 trialAndErrorAdjust1(Ogre::Vector3(0,90,0));//Hm, figure this out later, is it because of how hammer or mount node is set up?
	//	Ogre::Matrix3 trialAndErrorAdjust2(Ogre::Vector3(90,0,0));
	//	//mountTransform.mul(trialAndErrorAdjust1);
	//	mountTransform.mul(trialAndErrorAdjust2);

	//	Ogre::Vector3 mountPos = mountTransform.getPosition();
	//	Ogre::Vector3 finalPos = mountPos + getPosition();
	//	//Con::errorf("mountPos %f %f %f, finalPos %f %f %f",mountPos.x,mountPos.y,mountPos.z,finalPos.x,finalPos.y,finalPos.z);
	//	Ogre::Quaternion mountQuat(mountTransform);

	//	mountTransform.setPosition(finalPos);
	//	//mWeapon->setPosition(mountPos);
	//	mWeapon->setTransform(mountTransform);
	//	mWeapon->mRB->setLinearPosition(finalPos);
	//	mWeapon->mRB->setAngularPosition(mountQuat);
	//	mWeapon->setKinematic();

	//	//Finally, create fxJoint, using WeaponJointData for the datablock.
	//	mWeaponJoint = new fxJoint();
	//	mWeaponJoint->onNewDataBlock((GameBaseData *)jd);
	//	mWeaponJoint->mRB_A = mWeaponBodypart->mRB;
	//	mWeaponJoint->mRB_B = mWeapon->mRB;
	//	mWeaponJoint->onAdd();
	//	mWeaponJoint->mJoint->setMotorTarget(Ogre::Quaternion(0,0,1,0));
	//}


void fxFlexBody::setWeaponMotorTarget(Ogre::Vector3 &pRot)
{
	//Ogre::Quaternion qRot(pRot);
	//mWeaponJoint->mJoint->setMotorTarget(qRot);
	//return;	
}

void fxFlexBody::setWeaponMotorTarget(Ogre::Quaternion &qRot)
{
	//mWeaponJoint->mJoint->setMotorTarget(qRot);
	//return;	
}


void fxFlexBody::setWeaponTriggerMotorTarget(Ogre::Vector3 &pRot)
{
	//Ogre::Quaternion qRot(pRot);
	////mWeapon2Bodypart->mRB->setMotorTarget(qRot);
	//((nxRigidBody *)(mWeaponBodypart->mRB))->setTriggerJointMotorTarget(qRot);
	//return;	
}

void fxFlexBody::setWeaponTriggerRotAdjA(Ogre::Vector3 &pRot)
{
	//Ogre::Quaternion qRot(pRot);
	//mWeaponTriggerRotAdjA = qRot;
	//return;
}

void fxFlexBody::setWeaponTriggerRotAdjB(Ogre::Vector3 &pRot)
{
	//Ogre::Quaternion qRot(pRot);
	//mWeaponTriggerRotAdjB = qRot;
	//return;
}

//void fxFlexBody::mountWeapon()
//{//OBSOLETE
//	//Con::errorf("now we're on the client, mounting a weapon!");
//	//Con::errorf("weapID = %d,  weap Position = %f %f %f",mWeaponID,mWeapon->mCurrPosition.x,mWeapon->mCurrPosition.y,mWeapon->mCurrPosition.z);
//
//	//fxFlexBodyPart *kBodypart = NULL;
//	//kBodypart = getBodyPart(mWeaponNodeName);
//	//this will be NECESSARY if ever we return to creating a joint between the bodypart and the weapon.
//
//	//Tell the weapon about this, so it can keep position updated.
//	mWeapon->mParentBody = this;
//	mWeapon->mParentMountSlot = mWeaponMountNode;
//	//mWeapon->mIsKinematic = true;
//	//mWeapon->mRB->setKinematic();
//
//	//HERE: move the weapon to the mount node position, just to be safe?
//	//Ogre::Matrix3 xf(true);
//	//if (mWeaponMountSlot >= 0 && mWeaponMountSlot < fxFlexBody::MaxMountedImages)
//	//{
//	//	getMountTransform(mWeaponMountSlot,&xf);
//	//	mWeapon->mCurrPosition = xf.getPosition();
//	//	mWeapon->mRB->setLinearPosition(mWeapon->mCurrPosition);
//	//}
//		
//	//if (kBodypart) {
//		//[Not necessary if the guy's going to drop his weapon when he dies anyway.]
//		//Now, find the weapon joint data (possibly add more options here later)
//		//fxJointData *jd;
//		//SimDataBlockGroup *g = Sim::getDataBlockGroup();
//		//for (unsigned int c=0;c<g->size();c++) {
//		//	if (!strcmp(( (SimDataBlock *)(*g)[c])->getName(),"WeaponJointData")) {
//		//		jd = (fxJointData *)(*g)[c];
//		//	}
//		//}
//
//		//Finally, create fxJoint, using WeaponJointData for the datablock.
//		//mWeaponJoint = new fxJoint();
//		//mWeaponJoint->onNewDataBlock((GameBaseData *)jd);
//		//mWeaponJoint->mRB_A = kBodypart->mRB;
//		//mWeaponJoint->mRB_B = mWeapon->mRB;
//		//mWeaponJoint->onAdd();
//	//}
//}
////////////////////////////////////////////////////
void fxFlexBody::addWeapon2(int weaponID,int mountNode)//const char *bodypartNodeName
{
	
	////Con::errorf("Adding a weapon2, int mount node %d",mountNode);
	//if( Sim::findObject( weaponID, mWeapon2 ) == false) //Con::errorf("couldn't find the weapon!");
	//else //Con::errorf("weapon I found is located at: %f %f %f, mount slot %d",
	//	mWeapon2->mCurrPosition.x,mWeapon2->mCurrPosition.y,mWeapon2->mCurrPosition.z,mountNode);

	//mWeapon2MountNode = mountNode;
	//mWeapon2ID = weaponID;
	//mWeapon2->mFlexBody = this;

	////mWeapon2PosAdj = mWeapon2->mDataBlock->mWeaponPosAdj;
	////Ogre::Vector3 rotAdj = mWeapon2->mDataBlock->mWeaponRotAdj;
	//mWeapon2PosAdj = mWeapon2->mWeaponPosAdj;
	//mWeapon2RotAdjA = mWeapon2->mWeaponRotAdjA;
	//mWeapon2RotAdjB = mWeapon2->mWeaponRotAdjB;
	////mWeapon2RotAdj.set(Ogre::Vector3(mDegToRad(rotAdj.x),mDegToRad(rotAdj.y),mDegToRad(rotAdj.z)));

	//if (mountNode==39) mWeapon2Bodypart =  mBodyParts[mRightFrontIndex+3];//??  No, set this by mount node for now
	//else mWeapon2Bodypart =  mBodyParts[mLeftFrontIndex+3];//later make it accessible on the fly from script.
	////Ogre::Vector3 myPos = mWeapon2Bodypart->getPosition();
	//
	//mWeapon2->mFlexBody = this;

}
void fxFlexBody::addWeapon2(int weaponID,const char *mountNode)
{
	
	////Con::errorf("Adding a weapon2, char mount node %s",mountNode);
	//TSShape *kShape = mShapeInstance->getShape();
	//if( Sim::findObject( weaponID, mWeapon2 ) == false) //Con::errorf("couldn't find the weapon!");
	//else //Con::errorf("weapon I found is located at: %f %f %f, mount slot %d",
	//	mWeapon2->mCurrPosition.x,mWeapon2->mCurrPosition.y,mWeapon2->mCurrPosition.z,mountNode);

	//mWeapon2MountNode = kShape->findNode(mountNode);
	//if (mWeapon2MountNode == -1)
	//{
	//	//Con::errorf("AddWeapon can't find mount node: %s",mountNode);
	//	return;
	//}
	//mWeapon2ID = weaponID;
	//mWeapon2->mFlexBody = this;

	////mWeapon2PosAdj = mWeapon2->mDataBlock->mWeaponPosAdj;
	////Ogre::Vector3 rotAdj = mWeapon2->mDataBlock->mWeaponRotAdj;
	//mWeapon2PosAdj = mWeapon2->mWeaponPosAdj;
	//mWeapon2RotAdjA = mWeapon2->mWeaponRotAdjA;
	//mWeapon2RotAdjB = mWeapon2->mWeaponRotAdjB;
	////mWeapon2RotAdj.set(Ogre::Vector3(mDegToRad(rotAdj.x),mDegToRad(rotAdj.y),mDegToRad(rotAdj.z)));

	//if (!strcmp(mountNode,"mount0")) mWeapon2Bodypart =  mBodyParts[mRightFrontIndex+3];//??  No, set this by mount node for now
	//else mWeapon2Bodypart =  mBodyParts[mLeftFrontIndex+3];//later make it accessible on the fly from script.
	////Ogre::Vector3 myPos = mWeapon2Bodypart->getPosition();
	//
	//mWeapon2->mFlexBody = this;

}
	//if (kBodypart) {
	//if (0) { //old way

	//	////Con::errorf("hand position: %f %f %f",handPos.x,handPos.y,handPos.z);
	//	fxJointData *jd;
	//	SimDataBlockGroup *g = Sim::getDataBlockGroup();
	//	for (unsigned int c=0;c<g->size();c++) {
	//		if (!strcmp(( (SimDataBlock *)(*g)[c])->getName(),"WeaponJointData")) {
	//			jd = (fxJointData *)(*g)[c];
	//		}
	//	}
	//	if (jd) //Con::errorf("found a weapon joint: %f",jd->mBreakingForce);

	//	Ogre::Matrix3 mountTransform = mShapeInstance->mNodeTransforms[mWeapon2MountNode];
	//	//Ogre::Matrix3 trialAndErrorAdjust1(Ogre::Vector3(0,90,0));//Hm, figure this out later, is it because of how hammer or mount node is set up?
	//	Ogre::Matrix3 trialAndErrorAdjust2(Ogre::Vector3(90,0,0));
	//	//mountTransform.mul(trialAndErrorAdjust1);
	//	mountTransform.mul(trialAndErrorAdjust2);

	//	Ogre::Vector3 mountPos = mountTransform.getPosition();
	//	Ogre::Vector3 finalPos = mountPos + getPosition();
	//	//Con::errorf("mountPos %f %f %f, finalPos %f %f %f",mountPos.x,mountPos.y,mountPos.z,finalPos.x,finalPos.y,finalPos.z);
	//	Ogre::Quaternion mountQuat(mountTransform);

	//	mountTransform.setPosition(finalPos);
	//	//mWeapon->setPosition(mountPos);
	//	mWeapon2->setTransform(mountTransform);
	//	mWeapon2->mRB->setLinearPosition(finalPos);
	//	mWeapon2->mRB->setAngularPosition(mountQuat);
	//	mWeapon2->setKinematic();

	//	//Finally, create fxJoint, using WeaponJointData for the datablock.
	//	mWeapon2Joint = new fxJoint();
	//	mWeapon2Joint->onNewDataBlock((GameBaseData *)jd);
	//	mWeapon2Joint->mRB_A = mWeapon2Bodypart->mRB;
	//	mWeapon2Joint->mRB_B = mWeapon2->mRB;
	//	mWeapon2Joint->onAdd();
	//	mWeapon2Joint->mJoint->setMotorTarget(Ogre::Quaternion(0,0,1,0));
	//}
void fxFlexBody::setWeapon2MotorTarget(Ogre::Vector3 &pRot)
{
	//Ogre::Quaternion qRot(pRot);
	//mWeapon2Joint->mJoint->setMotorTarget(qRot);
	//return;	
}

void fxFlexBody::setWeapon2MotorTarget(Ogre::Quaternion &qRot)
{
	mWeapon2Joint->mJoint->setMotorTarget(qRot);
	return;	
}


void fxFlexBody::setWeapon2TriggerMotorTarget(Ogre::Vector3 &pRot)
{
	//Ogre::Quaternion qRot(pRot);
	////mWeapon2Bodypart->mRB->setMotorTarget(qRot);
	//((nxRigidBody *)(mWeapon2Bodypart->mRB))->setTriggerJointMotorTarget(qRot);
	//return;	
}

//void fxFlexBody::mountWeapon2()
//{
//	//Con::errorf("now we're on the client, mounting a weapon!");
//	//Con::errorf("weapID = %d,  weap Position = %f %f %f",mWeapon2ID,mWeapon2->mCurrPosition.x,mWeapon2->mCurrPosition.y,mWeapon2->mCurrPosition.z);
//
//	//Tell the weapon about this, so it can keep position updated.
//	mWeapon2->mParentBody = this;
//	mWeapon2->mParentMountSlot = mWeapon2MountNode;
//	return;	
//}


////////////////////////////////////////////////////


void fxFlexBody::clientRemoveBodyparts()
{
	//if (isClientObject())
	//{
		//Con::errorf("client is removing bodyparts?");
		//for (unsigned int i=0;i<mNumBodyParts;i++) mBodyParts[i]->onRemove();
	//}
}


void fxFlexBody::setPhysActive(bool b)
{
	if (b)
		mIsPhysActive = true;
	else
		mIsPhysActive = false;

	//setMaskBits(fxFlexBodyMountMask);
}

void fxFlexBody::setIsRecording(bool b)
{
	//if (b)
	//{
	//	mIsRecording = true;
	//	//HERE: get initial position
	//	mRecordInitialPosition = getPosition();
	//	Ogre::Matrix3 kTransform = getTransform();
	//	mRecordInitialOrientation.set(kTransform);
	//	mRecordCount = 0;
	//	nodeTranslations.clear();
	//	nodeRotations.clear();
	//}
	//else
	//	mIsRecording = false;

 	//setMaskBits(fxFlexBodyMountMask);
}

void fxFlexBody::setIsRendering(bool b)
{
	if (b)
		mIsRendering = true;
	else
		mIsRendering = false;

	//setMaskBits(fxFlexBodyMountMask);
}

void fxFlexBody::setIsReturnToZero(bool b)
{
	if (b)
		mIsReturnToZero = true;
	else
		mIsReturnToZero = false;

	//setMaskBits(fxFlexBodyMountMask);
}

void fxFlexBody::setInitialPosition(Ogre::Vector3 &pos)
{
	mInitialPosition = pos;
	//setMaskBits(fxFlexBodyMountMask);
}

void fxFlexBody::setInitialOrientation(Ogre::Vector3 &rot)
{
	//rot *= (180.0 / M_PI);
	//mInitialOrientation.set(rot);
	//setMaskBits(fxFlexBodyMountMask);
}

void fxFlexBody::doSomeDamage(float damage)
{
	/*
	AIGuard * serverParent = NULL;
	serverParent = dynamic_cast<AIGuard* >((NetObject *)mServerObject);
	if (serverParent)
	{
		serverParent->applyDamage(damage);
	}
	applyDamage(damage);
	*/
}

void fxFlexBody::setup()
{//Half-formed plans... was thinking of a trigger around the whole flexbody that signified impending 
	//collision possibility.  Causes problems, not using.
	//NxPhysicsSDK *kPhysicsSDK = ((nxPhysManager *)mPM)->getPhysicsSDK();
	//NxScene *kScene = NULL;
	//NxScene *kHWScene = NULL;

	//kScene = ((nxPhysManager *)mPM)->getScene();
	//kHWScene = ((nxPhysManager *)mPM)->getHWScene();

	//if ((!mTriggerActor)&&(mDataBlock->mTriggerDimensions.length()>0.0))
	//{
	//	NxActorDesc actorDesc;
	//	NxBodyDesc bodyDesc;
	//	NxBoxShapeDesc boxDesc;
	//	NxSphereShapeDesc sphereDesc,baseSphereDesc;
	//	NxCapsuleShapeDesc capsDesc;

	//	//base sphere, 
	//	baseSphereDesc.radius = 0.2;
	//	baseSphereDesc.localPose.t = NxVec3(0,0,0);
	//	baseSphereDesc.materialIndex = 0;
	//	actorDesc.shapes.pushBack(&baseSphereDesc);

	//	actorDesc.density = 1.0;

	//	Ogre::Vector3 offset,orient,dim;
	//	if (mDataBlock->mTriggerShapeType==PHYS_SHAPE_BOX) {
	//		boxDesc.dimensions.set(mDataBlock->mTriggerDimensions.x/2.0,mDataBlock->mTriggerDimensions.y/2.0,mDataBlock->mTriggerDimensions.z/2.0);
	//		boxDesc.localPose.t = NxVec3(mDataBlock->mTriggerOffset.x,mDataBlock->mTriggerOffset.y,mDataBlock->mTriggerOffset.z);
	//		boxDesc.shapeFlags |= NX_TRIGGER_ENABLE;
	//		boxDesc.materialIndex = 0;
	//		actorDesc.shapes.pushBack(&boxDesc);
	//		/////////////////////////////////////////////////////////////////////////////////
	//	} else if (mDataBlock->mTriggerShapeType==PHYS_SHAPE_SPHERE) { 
	//		sphereDesc.radius = mDataBlock->mTriggerDimensions.x;
	//		sphereDesc.localPose.t = NxVec3(mDataBlock->mTriggerOffset.x,mDataBlock->mTriggerOffset.y,mDataBlock->mTriggerOffset.z);
	//		sphereDesc.shapeFlags |= NX_TRIGGER_ENABLE;
	//		sphereDesc.materialIndex = 0;
	//		actorDesc.shapes.pushBack(&sphereDesc);
	//		/////////////////////////////////////////////////////////////////////////////////
	//	} else if (mDataBlock->mTriggerShapeType==PHYS_SHAPE_CAPSULE) { 
	//		capsDesc.radius = mDataBlock->mTriggerDimensions.x;
	//		capsDesc.height = mDataBlock->mTriggerDimensions.z;
	//		capsDesc.localPose.t = NxVec3(mDataBlock->mTriggerOffset.x,mDataBlock->mTriggerOffset.y,mDataBlock->mTriggerOffset.z);
	//		capsDesc.shapeFlags |= NX_TRIGGER_ENABLE;
	//		capsDesc.materialIndex = 0;
	//		actorDesc.shapes.pushBack(&capsDesc);
	//		/////////////////////////////////////////////////////////////////////////////////
	//	}
	//	actorDesc.body = &bodyDesc;
	//	//actorDesc.body->mass = 10.0;
	//	//actorDesc.body->massSpaceInertia.set(NxVec3(0.0,0.0,100.0);//(?)
	//	actorDesc.globalPose.t = NxVec3(mCurrPosition.x,mCurrPosition.y,mCurrPosition.z);

	//	//(body->mass < 0 || body->massSpaceInertia.isZero())

	//	if ((!mDataBlock->mHW)||(!kHWScene)) {

	//		if (actorDesc.isValid())
	//			mTriggerActor = kScene->createActor(actorDesc);
	//		if (!mTriggerActor) return;

	//		//Con::errorf("flexbody creating trigger actor!!!!");
	//		mTriggerActor->setGroup(0);//non-colliding group
	//		NxMat33 mat;
	//		//mTriggerActor->raiseBodyFlag(NX_BF_KINEMATIC);
	//		mTriggerActor->raiseBodyFlag(NX_BF_DISABLE_GRAVITY);

	//		NxShape * const * shapes = mTriggerActor->getShapes();
	//		Nxunsigned int nShapes    = mTriggerActor->getNbShapes();
	//		while (nShapes--) {

	//			if (mDataBlock->mTriggerOrientation.x) mat.rotX(mDataBlock->mTriggerOrientation.x * (NxPi/180.0));
	//			else if (mDataBlock->mTriggerOrientation.y) mat.rotY(mDataBlock->mTriggerOrientation.y * (NxPi/180.0));
	//			else if (mDataBlock->mTriggerOrientation.z) mat.rotZ(mDataBlock->mTriggerOrientation.z * (NxPi/180.0));
	//			else mat.id();
	//			shapes[nShapes]->setGlobalOrientation(mat);
	//		}
	//		physShapeData *kUserData = new physShapeData;
	//		kUserData->mEntityType = PHYS_FLEX_BODY;//PHYS_TRIGGER;
	//		kUserData->mPhysUser = this;
	//		mTriggerActor->userData = (void *)kUserData;
	//	}
	//}
}

void fxFlexBody::releaseActors()
{
	NxScene *kScene = ((nxPhysManager*)mPM)->getScene();
	
	if ((kScene)&&(mTriggerActor)) kScene->releaseActor(*mTriggerActor);
	
	mTriggerActor = NULL;
}

void fxFlexBody::setBodypartDelayForces(Ogre::Vector3 force)
{
	for (unsigned int i=0;i<mNumBodyParts;i++)
	{
		mBodyParts[i]->mRB->setGlobalDelayForce(force);
		mBodyParts[i]->mRB->setDelayStep(((physManagerCommon *)physManagerCommon::getPM())->mCurrStep);
	}
}

void fxFlexBody::resetPosition()
{
 //  mReset = true;
 //  mPlaylistTick = 0;
	////HERE: not sure this is best overall way to handle this, but least changes to the 
	////current system would be to set mInitialPosition right now from the database.  
	////Drawback is it's one query per actor, instead of one query for all of them.
	//char actor_scene_query[512];
	//sqlite_resultset *resultSet;
	//int result;		

	//SQLiteObject *sql = new SQLiteObject();
	//if (!sql) return;

	//if (sql->OpenDatabase("EcstasyMotion.db"))//FIX: replace all tese with global variable
	//{
	//	sprintf(actor_scene_query,"SELECT id,start_x,start_y,start_z,start_rot FROM actorScene WHERE actor_id=%d AND scene_id=%d;",
	//			mActorID,dynamic_cast<nxPhysManager*>(mPM)->mSceneId);
	//	result = sql->ExecuteSQL(actor_scene_query);
	//	resultSet = sql->GetResultSet(result);
	//	if (resultSet->iNumRows == 1)//danger, make sure there can never be two or more.
	//	{
	//		int actor_scene_id = strtol(resultSet->vRows[0]->vColumnValues[0]);
	//		float x = strtod(resultSet->vRows[0]->vColumnValues[1],NULL);
	//		float y = strtod(resultSet->vRows[0]->vColumnValues[2],NULL);
	//		float z = strtod(resultSet->vRows[0]->vColumnValues[3],NULL);
	//		float rot = strtod(resultSet->vRows[0]->vColumnValues[4],NULL);
	//		mInitialPosition.set(x,y,z);
	//		//Con::errorf("setting initial position: %3.3f %3.3f %3.3f, rotation %3.2f",x,y,z,rot);
	//		mInitialOrientation.set(Ogre::Vector3(0,0,mDegToRad(rot)));
	//	}
	//	sql->CloseDatabase();
	//	delete sql;
	//}
	////Now, put yourself in frame 0 of your first anim...
	//if (mPlaylist.size()>0)
	//{
	//	TSShapeInstance *kSI = getShapeInstance();
	//	playSeq(0,mPlaylist[0].seq);
	//	TSThread *th = kSI->getThread(0);
	//	kSI->setTimeScale(th,0.0);
	//}
	//setKinematic();
}

void fxFlexBody::resetSequence(int seq)
{
	//TSShapeInstance *kSI = getShapeInstance();
	////mGroundStartXform = getTransform();
	//mInitialOrientation.set(getTransform());//Do this every time you start or repeat a sequence!
	//playSeq(0,seq);
	//TSThread *th = kSI->getThread(0);
	//kSI->setTimeScale(th,0.0);
}

//ConsoleMethod( fxFlexBody, resetSequence, void,  3, 3,"(int seq)")
//{
//	object->resetSequence(strtol(argv[2]));
//}

//void fxFlexBody::snakeForward(float force)
//{	
//	if (mActionUser)
//	{
//		mActionUser->setGoalForward();
//		mActionUser->setForwardForce(force);
//	}
//}

//void fxFlexBody::snakeStop()
//{
//	Ogre::Vector3 forwardVec;
//	forwardVec.set(0.0,0.0,0.0);
//
//	for (unsigned int i=0;i<mNumBodyParts;i++)		
//	{
//		mBodyParts[i]->setForce(forwardVec);
//		mBodyParts[i]->setTorque(forwardVec);
//	}
//}

void fxFlexBody::giveDetails()
{
	for (unsigned int i=0;i<mNumBodyParts;i++)
	{
		Ogre::Vector3 pos = mBodyParts[i]->mCurrPosition;
		//Con::printf("%d %s: %f %f %f",i,mBodyParts[i]->mDataBlock->mBaseNodeName,pos.x,pos.y,pos.z);
	}
}


//bool fxFlexBody::castRay(const Ogre::Vector3 &start, const Ogre::Vector3 &end, RayInfo* info)
//{
//	info->object = this;
//	return true;
//}

int fxFlexBody::getCurrSeqNum()
{
	//Thread& st = mScriptThread[0];
	//int seqnum;
	//if (st.thread) 
	//{
	//	seqnum = getShapeInstance()->getThread(0)->getSequenceNum();
	//	return seqnum;
	//}
	//else 
	return 0;
}

void fxFlexBody::showNodeTransform(int index)
{
	//Ogre::Matrix3 m = mShapeInstance->mNodeTransforms[index];
	//Ogre::Quaternion q(m);
	//Ogre::Vector3 eul = m.toEuler();
	//Ogre::Vector3 pos = m.getPosition();
	//Ogre::Matrix3 pTransform = getTransform();
	//Ogre::Vector3 eu2 = pTransform.toEuler();
	//pTransform.inverse();
	//Ogre::Vector3 eu3 = pTransform.toEuler();
	//Con::errorf("node %d transform: quat %f %f %f %f, euler %f %f %f, pos %f %f %f",index,q.x,q.y,q.z,q.w,eul.x,eul.y,eul.z,pos.x,pos.y,pos.z);
}


//if (mScriptThread[0].state != Thread::Stop)
		//stopThread(0);
//		setThreadPos(0,pos);
//		setThreadSequence(0,seq,true,pos);


// > pg thread, overridden so we can set mSequenceEndStep
//bool fxFlexBody::playThread(unsigned int slot, const char *name)
//{
//	int seq = getShape()->findSequence(name);
//	char actionSequence[255];
//	if (!mIsAnimating)
//	{
//		mIsAnimating = true;
//		if (!(mDataBlock->mGA)) setKinematic();
//		mClearIsAnimating = false;
//		mStopAnimating = false;
//	}
//	sprintf(actionSequence,"sequence.%s",name);
//	loadAction(actionSequence);//HERE: decide more carefully whether to do this or not...
//	if (!mIsThinking)
//		mIsThinking = true;
//
//	//Figure out groundSequences... these advance us while animating, based on foot positions 
//	// and/or constant velocity vector.
//	SimDataBlockGroup *g = Sim::getDataBlockGroup();
//	for (unsigned int c=0;c<g->size();c++) 
//	{
//		if (!strcmp(( (SimDataBlock *)(*g)[c])->getClassName(),"physGroundSequenceData")) 
//		{
//			physGroundSequenceData *pd = (physGroundSequenceData *)(*g)[c];
//			if ((pd->mFlexBodyData==mDataBlock)&&(strcmp(pd->mSeqName,name)))
//			{
//				mIsGroundAnimating = true;
//				mGroundSequenceData = pd;
//				mGroundNode = pd->mNodes[0];
//				//Con::errorf("ground node: %d",mGroundNode);
//				mGroundStep = 0;
//				if (mGroundNode>=0) 
//				{
//					mGroundVector = mShapeInstance->mNodeTransforms[mGroundNode].getPosition() + getPosition();
//					Point2F twoDPos(mGroundVector.x,mGroundVector.y);
//					float z = mPM->getTerrHeight(twoDPos);//DANGER you can't stand on interiors if you do this.
//					mGroundVector.z = z;//better to do a raycast straight down that collides with everything. 
//				}
//			}
//		}
//	}
//
//	
//	//stopThread(0);
//	//setThreadSequence(0,seq,true,0.0);
//	//Thread& st = mScriptThread[0];
//	//if (st.sequence>=0)
//	//{
//		//TSThread *tsT = st.thread;
//
//		////Con::printf("about to playThread: pos %f seq %d forward %d speed %f startPos %f state %d",
//		//st.position,st.sequence,st.forward,st.speed,st.startPos,st.state);//, tsT->getPos());
//	//}
//
//	if (Parent::playThread(slot, name))
//	{
//		//mSequenceEndStep = ((physManagerCommon *)physManagerCommon::getPM())->mCurrStep + (getShape()->sequences[seq].duration * 1000);
//		mSequenceEndStep = mCurrMS + (getShape()->sequences[seq].duration * 1000);
//		mSequenceStartStep = mCurrMS;
//		mCurrSeq = seq;
//		setThreadPos(0,0.0);
//		Thread& st = mScriptThread[0];
//		if (st.atEnd)
//		{
//			//Con::errorf("thread at end is true!!!");
//			st.atEnd = false;
//		}
//		st.thread->setTimeScale(1.0);
//
//	//TSShapeInstance *kSI = getShapeInstance();
//	//playSeq(0,seq);
//	//TSThread *th = kSI->getThread(0);
//	//kSI->setTimeScale(th,0.0);
//
//		 //st = mScriptThread[0];
//		 //if (st.sequence>=0)
//		 //{
//			 //TSThread *tsT = st.thread;
//
//			 
//			// //Con::printf("just called playThread: pos %f seq %d forward %d speed %f startPos %f state %d",
//			//	 st.position,st.sequence,st.forward,st.speed,st.startPos,st.state);//, tsT->getPos());
//		 //}
//		return true;
//	}
//	else return false;
//
//}
//bool fxFlexBody::playThread(unsigned int slot)
//{	
	//if (Parent::playThread(slot))
	//{
		//mSequenceEndStep = ((physManagerCommon *)physManagerCommon::getPM())->mCurrStep + (getShape()->sequences[seq].duration * 1000);
		//return true;
	//}
	//else return false;
//}

//bool fxFlexBody::playSeq(unsigned int slot, int seq)
//{
//	if (Parent::playSeq(slot, seq))
//	{
//		//mSequenceEndStep = ((physManagerCommon *)physManagerCommon::getPM())->mCurrStep + (getShape()->sequences[seq].duration * 1000);
//		mSequenceEndStep = mCurrMS + (getShape()->sequences[seq].duration * 1000);
//		mSequenceStartStep = mCurrMS;
//		mCurrSeq = seq;
//		return true;
//	}
//	else return false;
//}

void fxFlexBody::runPlaylist()
{
	//if (mPlaylist.size())
	//{
	//	mRunningPlaylist = true;
	//	mPlaylistRepeats = 0;//mPlaylist[0].repeats;
	//	mPlaylistTick = 0;
	//	mCurrentPlaylistSeq = 0;

	//	playSeq(0,mPlaylist[0].seq);

	//	TSShape *kShape = mShapeInstance->getShape();
	//	if (mShapeInstance->mThreadList.size())
	//	{
	//		TSThread *thread = mShapeInstance->getThread(0);
	//		if (thread)
	//		{
	//			//Con::printf("running playlist, thread pos: %f, timescale %f",thread->getPos(),thread->timeScale);
	//			//HERE: Somehow, for no apparent reason, bots are randomly having their timescale set to -1 when 
	//			//running playlists.  No idea why or how that happens, but it's extremely counterproductive.
	//			if (thread->timeScale == -1) 
	//			{
	//				thread->timeScale = 1;//NOPE, doesn't work.  This must be a symptom, not the cause.
	//				//Con::printf("Error condition: runPlaylist has returned with timescale = -1.");
	//			}
	//		}
	//	}
	//	//playThread(0,kShape->getName(kShape->sequences[mPlaylist[0].seq].nameIndex));//Hmm, something going
	//	//wrong somewhere in playSeq or somewhere else... all bots but first one have playback bug, stuck 
	//	//on first frame, not playing, but playthread works on them in other cases... wtf.
	//	TSShape::Sequence kSeq = kShape->sequences[mPlaylist[0].seq];
	//	if (kSeq.numGroundFrames>1) mGroundStartXform = getTransform();
	//	////setMaskBits(ThreadMask);//ARgh, doesn't WORK... still can't get a value for mRunningPlaylist, down in tsThread where I need it.

	//}
 //  return;
}
	//For now, hard code our particular skeleton from Capoeira demo to ACK rig.
	//kArenaIndex[0] = 1;//Will have to add more shapebase bloat in order
	//kArenaIndex[1] = 2;//to allow kork & other models to have their own mappings.
	//kArenaIndex[2] = 3;//Or else, do specific cases here based on shapename.
	//kArenaIndex[3] = 4;
	//kArenaIndex[4] = 5;
	//kArenaIndex[5] = 12;
	//kArenaIndex[6] = 13;
	//kArenaIndex[7] = 14;
	//kArenaIndex[8] = 15;
	//kArenaIndex[9] = 7;
	//kArenaIndex[10] = 8;
	//kArenaIndex[11] = 9;
	//kArenaIndex[12] = 10;
	//kArenaIndex[13] = 21;
	//kArenaIndex[14] = 22;
	//kArenaIndex[15] = 23;
	//kArenaIndex[16] = 17;
	//kArenaIndex[17] = 18;
	//kArenaIndex[18] = 19;

void fxFlexBody::startArenaFrame()
{
	mArenaStreamWeap = false;
	mArenaStreamWeap2 = false;

}

bool gSetupArena = false;
int kArenaIndex[MAX_FLEX_PARTS];
int kBodyIndex[MAX_FLEX_PARTS];

void fxFlexBody::setBodypartArena(int ID, Ogre::Vector3 &pos, Ogre::Quaternion &quat)
{
	//if (gSetupArena == false)
	//{
	//	const String myPath = mShapeInstance->mShapeResource.getPath().getPath();
	//	if ((strstr(myPath.c_str(),"ACK"))||(strstr(myPath.c_str(),"Soccer"))||(strstr(myPath.c_str(),"Daz3D")))
	//	{    
	//		TSShape *kShape = mShapeInstance->getShape();
	//		kArenaIndex[1] = kShape->findNode("hip");
	//		kArenaIndex[2] = kShape->findNode("abdomen");
	//		kArenaIndex[3] = kShape->findNode("chest");
	//		kArenaIndex[4] = kShape->findNode("neck");
	//		kArenaIndex[5] = kShape->findNode("head");
	//		kArenaIndex[7] = kShape->findNode("lCollar");
	//		kArenaIndex[8] = kShape->findNode("lShldr");
	//		kArenaIndex[9] = kShape->findNode("lForeArm");
	//		kArenaIndex[10] = kShape->findNode("lHand");
	//		kArenaIndex[12] = kShape->findNode("rCollar");
	//		kArenaIndex[13] = kShape->findNode("rShldr");
	//		kArenaIndex[14] = kShape->findNode("rForeArm");
	//		kArenaIndex[15] = kShape->findNode("rHand");
	//		kArenaIndex[17] = kShape->findNode("lThigh");
	//		kArenaIndex[18] = kShape->findNode("lShin");
	//		kArenaIndex[19] = kShape->findNode("lFoot");
	//		kArenaIndex[21] = kShape->findNode("rThigh");
	//		kArenaIndex[22] = kShape->findNode("rShin");
	//		kArenaIndex[23] = kShape->findNode("rFoot");
	//		kArenaIndex[24] = kShape->findNode("mount0");
	//		kArenaIndex[25] = kShape->findNode("mount1");

	//		kBodyIndex[1] = getBodyPartID("hip");
	//		kBodyIndex[2] = getBodyPartID("abdomen");
	//		kBodyIndex[3] = getBodyPartID("chest");
	//		kBodyIndex[4] = getBodyPartID("neck");
	//		kBodyIndex[5] = getBodyPartID("head");
	//		kBodyIndex[7] = getBodyPartID("lCollar");
	//		kBodyIndex[8] = getBodyPartID("lShldr");
	//		kBodyIndex[9] = getBodyPartID("lForeArm");
	//		kBodyIndex[10] = getBodyPartID("lHand");
	//		kBodyIndex[12] = getBodyPartID("rCollar");
	//		kBodyIndex[13] = getBodyPartID("rShldr");
	//		kBodyIndex[14] = getBodyPartID("rForeArm");
	//		kBodyIndex[15] = getBodyPartID("rHand");
	//		kBodyIndex[17] = getBodyPartID("lThigh");
	//		kBodyIndex[18] = getBodyPartID("lShin");
	//		kBodyIndex[19] = getBodyPartID("lFoot");
	//		kBodyIndex[21] = getBodyPartID("rThigh");
	//		kBodyIndex[22] = getBodyPartID("rShin");
	//		kBodyIndex[23] = getBodyPartID("rFoot");
	//		
	//		gSetupArena = true;
	//	} else return;
	//}

	//Ogre::Matrix3 m1,m2;
	//quat.setMatrix(&m1);

	////m1.setPosition(pos);//hm, at mNodeTransforms level, are we local or global?
	//TSShape *kShape = mShapeInstance->getShape();

	//if (ID>23) //weapon
	//{
	//	float globalScale = 1.0;//0.95;//trial and error, unit confusion. :-(
	//	//First valid body = weapon1, second = weapon2.  Third and beyond are ignored. 
	//	//non-active weapons in same scene still stream but return identity quats.
	//	//So, test first whether incoming quat is identity.
	//	if ((mWeapon)&&(!mArenaStreamWeap)&&(pos.length()>0)){
	//		mWeaponPos = pos * globalScale; 
	//		mWeaponRot = quat; 
	//		mArenaStreamWeap = true;
	//		
	//		Ogre::Vector3 defaultPos,newPos;
	//		defaultPos = kShape->defaultTranslations[kArenaIndex[ID]];
	//		m2 = mShapeInstance->mNodeTransforms[kShape->nodes[kArenaIndex[ID]].parentIndex];
	//		Ogre::Vector3 eul = m2.toEuler();
	//		m2.mulP(defaultPos,&newPos);
	//		m1.setPosition(newPos);
	//		mShapeInstance->mNodeTransforms[kArenaIndex[ID]] = m1;

	//	} else if ((mWeapon2)&&(mArenaStreamWeap)&&(!mArenaStreamWeap2)&&(pos.length()>0)) {
	//		mWeapon2Pos = pos * globalScale; 
	//		mWeapon2Rot = quat; 
	//		mArenaStreamWeap2 = true;

	//		Ogre::Vector3 defaultPos,newPos;
	//		defaultPos = kShape->defaultTranslations[kArenaIndex[ID]];
	//		m2 = mShapeInstance->mNodeTransforms[kShape->nodes[kArenaIndex[ID]].parentIndex];
	//		Ogre::Vector3 eul = m2.toEuler();
	//		m2.mulP(defaultPos,&newPos);
	//		m1.setPosition(newPos);
	//		mShapeInstance->mNodeTransforms[kArenaIndex[ID]] = m1;
	//	}
	//}
	//else if (kShape->nodes[kArenaIndex[ID]].parentIndex>0)//>=0, hacked for soccer
	//{
	//	Ogre::Vector3 defaultPos,newPos;
	//	defaultPos = kShape->defaultTranslations[kArenaIndex[ID]];
	//	m2 = mShapeInstance->mNodeTransforms[kShape->nodes[kArenaIndex[ID]].parentIndex];
	//	Ogre::Vector3 eul = m2.toEuler();
	//	m2.mulP(defaultPos,&newPos);
	//	m1.setPosition(newPos);
	//	//HERE: this can be motor target, instead of directly node transforms.
	//	if (mBodyParts[kBodyIndex[ID]]->mIsKinematic)
	//		mShapeInstance->mNodeTransforms[kArenaIndex[ID]] = m1;
	//	else 
	//		if (mBodyParts[kBodyIndex[ID]]->mJoint)
	//		{
	//			//Ogre::Quaternion q(m1);
	//			mBodyParts[kBodyIndex[ID]]->mJoint->setMotorTarget(quat);
	//		}

	//}
	//else 
	//{
	//	m1.setPosition(pos);
	//	mShapeInstance->mNodeTransforms[kArenaIndex[ID]] = m1;
	//	//Ogre::Vector3 arenaPos = pos + mInitialPosition;
	//	//arenaPos.z = 0.0;
	//	//arenaPos.z = -0.287;//HACK, hard coding feet to floor w/ our arena setup.
	//	////Con::printf("setting arena bot position: %f %f %f",arenaPos.x,arenaPos.y,arenaPos.z);
	//	//setPosition(arenaPos);
	//}
}

void fxFlexBody::updateNodes()
{
	////THEN: run through mNodeTransforms, find all the ones that don't have bodyparts associated with 
	////them, and fix them with the position/orientation of their parent.  [NOTE: why position? come back to this]
	//for (unsigned int i = mFirstNode; i < mShapeInstance->getShape()->nodes.size(); i++) 
	//{
	//	bool kFound = false;
	//	//speed this up? store an array of [MAX_FLEX_PARTS] or max nodes, if different, 
	//	//check in one step
	//	unsigned int j;
	//	for (j=0; j< mNumBodyParts; j++) 
	//	{
	//		if (mBodyParts[j]->mNodeIndex==i) 
	//		{
	//			kFound = true;
	//			break;
	//		}
	//	}
	//	if (!kFound)
	//	{
	//		Ogre::Vector3 relPos,outPos,parPos,z;  z = Ogre::Vector3::ZERO;
	//		Ogre::Matrix3 kTransform;

	//		if (mShapeInstance->getShape()->nodes[i].parentIndex > -1)
	//		{
	//			kTransform = mShapeInstance->mNodeTransforms[mShapeInstance->getShape()->nodes[i].parentIndex];
	//			parPos = kTransform.getPosition();
	//			relPos = mShapeInstance->getShape()->defaultTranslations[i];
	//			kTransform.setPosition(z);
	//			kTransform.mulP(relPos,&outPos);
	//			outPos += parPos;
	//			kTransform.setPosition(outPos);
	//			//if (i==31) //Con::errorf("relPos %3.2f %3.2f %3.2f, outPos %3.2f %3.2f %3.2f",relPos.x,relPos.y,relPos.z,outPos.x,outPos.y,outPos.z);
	//			mShapeInstance->mNodeTransforms[i] = kTransform;
	//		}
	//	}
	//	//1. find out if we have a bodypart with this nodeIndex -- if so, skip
	//	//2. if not, then grab the transform of my parentIndex node.  (it should already be defined,
	//	//   since we're going through in order of the node hierarchy.)
	//	//3. for first cut, adjust position by minus basePos, where basePos is defaultTrans[0] + [1]
	//}
}

void fxFlexBody::resetParts()
{
	for (unsigned int i=0;i<mNumBodyParts;i++)
	{
		if (!mBodyParts[i]->mIsKinematic)
			mBodyParts[i]->reset();
	}
}

//const char* fxFlexBody::getFlexBodyName()
//{
//	return mDataBlock->getName();
//}

const char* fxFlexBody::getPersonaName()
{
	return mPersonaName.c_str();
}

void fxFlexBody::motorize()
{
	for (unsigned int i=0;i<mNumBodyParts;i++)
		if (!mBodyParts[i]->mIsKinematic) 
			setBodypartMotorTarget(i,Ogre::Vector3(0,0,0));
}

bool fxFlexBody::loadAction(const char *name)
{
	if (mActionUser)
	{
		return mActionUser->loadAction(name);
	}
	else return false;
}

bool fxFlexBody::loadAction(const char *name, float pos)
{
	if (mActionUser)
	{
		return mActionUser->loadAction(name,pos);
	}
	else return false;
}
//////////// Ecstasy Motion ////////////////////////////////
//int fxFlexBody::getKeyFrame(unsigned int slot)
//{
//   Thread& st = mScriptThread[slot];
//	int pos = 0;
//	if (st.thread) {
//		pos = getShapeInstance()->getKeyframeNumber(st.thread);
//	}
//   return pos;
//}

void fxFlexBody::cleanupDir(const char *bvhDir)
{
	char dos_command[255],filepath[255],listfile[255],buf[255],bvhName[255];
	char *bufp;
	//sprintf(filepath,"%s",mShapeInstance->getShape()->mSourceResource->path);

	strcpy(filepath,bvhDir);
	//HERE: probably have to replace "/" with "\\" first.

	sprintf(dos_command,"dir \"%s\\*.bvh\" /B > \"%s\\bvhfiles.txt\"",filepath,filepath);
	system(dos_command);

	
	sprintf(listfile,"%s/bvhfiles.txt",bvhDir);
	FILE *fplist = fopen(listfile,"r");
	if (fplist==NULL) 
	{
		//Con::errorf("ERROR: can't open bvh list");
		return;
	}
	//Con::errorf("opened list file: %s",listfile);
	while (fgets(buf,255,fplist)) 
	{
		////Con::errorf("buffer: %s",buf);
		bufp = strtok(buf,".");//get name w/o ".bvh", got a weird character at the end
		//sprintf(bvhName,"%s.bvh",bufp);
		sprintf(bvhName,"%s/%s.bvh",bvhDir,bufp);

		//Con::errorf("trying to open bvh file: %s",bvhName);
		cleanupBvh(bvhName);
	}
	fclose(fplist);
}

void fxFlexBody::nullBvh(const char *bvhFile)
{//TODO: update to kCfg (loadBvhCfg/loadBvhSkeleton) system.
	//HERE:  make a null bvh -- just like cleanup, except stop after base offsets, write out one frame full of zeroes. 
	//unsigned int pc,rc,jc,jloop,loopDepth;
	//pc = 0; rc = 0; jc = 0; jloop = 0; loopDepth = 0;
	//int newParent = -1;
	//Ogre::Quaternion rot;
	//Ogre::Quaternion rots[20000];//FIX!
	//						
	//Ogre::Vector3 trans[1000];//FIX!

	//Ogre::Vector3 p;
	//Ogre::Vector3 r;
	//Ogre::Vector3 pos3[200];
	//Ogre::Vector3 rot3[200];
	//bvhJoint joints[200];
	//bool keepGoing,loadingJoints;
	//char filename[255],writename[255],buf[2500];//FIX!
	//char line[2500], rotation[40], tempc[40],name[40];
	//char *bufp;
	//FILE *fp = NULL;
	//FILE *fpw = NULL;

	//strcpy(filename,bvhFile);
	//fp = fopen(filename,"r");
	//
	//if (fp==NULL) 
	//{
	//	//Con::errorf("ERROR: can't open bvh file: %s",filename);
	//	return;
	//}

	//
	//strcpy(writename,bvhFile);
	//strcat(writename,".null");
	//fpw = fopen(writename,"w");

	//char chan1[10], chan2[10], chan3[10];
	//unsigned int numChannels;

	//fgets(buf,250,fp); fprintf(fpw,"%s",buf);// HIERARCHY
	//fgets(buf,250,fp); fprintf(fpw,"%s",buf);// ROOT
	//sscanf(buf,"%s %s",&tempc,&name);

	//joints[0].parent = -1;
	//strcpy(joints[0].name,name);
	//fgets(buf,250,fp); fprintf(fpw,"%s",buf);// {
	//fgets(buf,250,fp); fprintf(fpw,"%s",buf);// OFFSET x y z
	//sscanf(buf,"  OFFSET %f %f %f",&p.x,&p.y,&p.z);
	////fprintf(fpw,"  OFFSET %f %f %f",p.x,p.y,p.z);
	//joints[0].offset = p;
	//fgets(buf,250,fp); fprintf(fpw,"%s",buf);// CHANNELS n ...
	//sscanf(buf,"  CHANNELS %d ",&numChannels);
	//if (numChannels==6)
	//	sscanf(buf,"  CHANNELS %d %s %s %s %s %s %s",&numChannels,&tempc,&tempc,&tempc,&chan1,&chan2,&chan3);
	//else if (numChannels==3)
	//	sscanf(buf,"  CHANNELS %d %s %s %s",&numChannels,&chan1,&chan2,&chan3);

	//joints[0].channels = numChannels;
	////Gotta sort out what order the rotations come in, PER NODE.
	//if (chan1[0]=='X') joints[0].chanrots[0] = 0;
	//else if (chan1[0]=='Y') joints[0].chanrots[0] = 1;
	//else if (chan1[0]=='Z')	joints[0].chanrots[0] = 2;
	//if (chan2[0]=='X') joints[0].chanrots[1] = 0;
	//else if (chan2[0]=='Y') joints[0].chanrots[1] = 1;
	//else if (chan2[0]=='Z') joints[0].chanrots[1] = 2;
	//if (chan3[0]=='X') joints[0].chanrots[2] = 0;
	//else if (chan3[0]=='Y') joints[0].chanrots[2] = 1;
	//else if (chan3[0]=='Z') joints[0].chanrots[2] = 2;

	//loopDepth = 0;

	//fgets(buf,250,fp); fprintf(fpw,"%s",buf);
	//bufp = strtok(buf," \t\n");

	//loadingJoints = true;
	//while (loadingJoints)
	//{
	//	keepGoing = true;
	//	while (keepGoing)
	//	{
	//		sprintf(tempc,"JOINT");
	//		if (!strstr(bufp,tempc)) 
	//		{ 
	//			keepGoing = false; //End Site
	//			fgets(buf,250,fp); fprintf(fpw,"%s",buf);//{
	//			fgets(buf,250,fp); fprintf(fpw,"%s",buf);//terminal OFFSET x y z, ignore it
	//			fgets(buf,250,fp); fprintf(fpw,"%s",buf);//}
	//			break; 
	//		}
	//		jc++;  loopDepth++;

	//		if (newParent>=0) {
	//			joints[jc].parent = newParent;
	//			newParent = -1;
	//		} else joints[jc].parent = jc-1;

	//		bufp = strtok(NULL, " \t\n");
	//		sscanf(bufp,"%s",name);				
	//		strcpy(joints[jc].name,name);
	//		fgets(buf,250,fp); fprintf(fpw,"%s",buf);//{
	//		fgets(buf,250,fp); fprintf(fpw,"%s",buf);//OFFSET x y z
	//		bufp = strtok(buf," \t\n");//"OFFSET"
	//		bufp = strtok(NULL, " \t\n"); sscanf(bufp,"%f",&joints[jc].offset.x);
	//		bufp = strtok(NULL, " \t\n"); sscanf(bufp,"%f",&joints[jc].offset.y);
	//		bufp = strtok(NULL, " \t\n"); sscanf(bufp,"%f",&joints[jc].offset.z);
	//		

	//		fgets(buf,250,fp);//CHANNELS n s s s s s s
	//		bufp = strtok(buf," \t\n");//"CHANNELS"
	//		bufp = strtok(NULL, " \t\n"); sscanf(bufp,"%d",&joints[jc].channels);
	//		if (joints[jc].channels==6) {
	//			bufp = strtok(NULL," \t\n"); 
	//			bufp = strtok(NULL," \t\n"); 
	//			bufp = strtok(NULL," \t\n"); 
	//		}
	//		bufp = strtok(NULL," \t\n"); sscanf(bufp,"%s",&chan1);
	//		bufp = strtok(NULL," \t\n"); sscanf(bufp,"%s",&chan2);
	//		bufp = strtok(NULL," \t\n"); sscanf(bufp,"%s",&chan3);
	//		//Gotta sort out what order the rotations come in, PER NODE.
	//		if (chan1[0]=='X') joints[jc].chanrots[0] = 0;
	//		else if (chan1[0]=='Y') joints[jc].chanrots[0] = 1;
	//		else if (chan1[0]=='Z') joints[jc].chanrots[0] = 2; 
	//		if (chan2[0]=='X') joints[jc].chanrots[1] = 0;
	//		else if (chan2[0]=='Y') joints[jc].chanrots[1] = 1;
	//		else if (chan2[0]=='Z') joints[jc].chanrots[1] = 2;
	//		if (chan3[0]=='X') joints[jc].chanrots[2] = 0;
	//		else if (chan3[0]=='Y') joints[jc].chanrots[2] = 1;
	//		else if (chan3[0]=='Z') joints[jc].chanrots[2] = 2;
	//		//FIX: here we need to write out "CHANNELS 3 Zrotation Xrotation Yrotation"  no matter what it said before.
	//		//Problem is we need to indent it properly, which is a PITA.  Have to track level of nesting and add 
	//		//correct number of tabs, use strcat in a loop.
	//		//ALSO need to track whether the original file was made with tabs or spaces.  This could get ridiculous, though.

	//		strcpy(line,"\t");
	//		for (unsigned int k=0;k<loopDepth;k++) strcat(line,"\t");
	//		//strcat(line,"CHANNELS 3 Zrotation Xrotation Yrotation\n");
	//		sprintf(tempc,80,"CHANNELS 3 %s %s %s\n",chan1,chan2,chan3);
	//		strcat(line,tempc);

	//		fprintf(fpw,"%s",line);


	//		//Con::printf("JOINT %d: %s %d %3.2f %3.2f %3.2f",jc,joints[jc].name,joints[jc].channels,
	//			joints[jc].offset.x,joints[jc].offset.y,joints[jc].offset.z);
	//		
	//		fgets(buf,250,fp); fprintf(fpw,"%s",buf);
	//		bufp = strtok(buf," \t\n");
	//	}
	//	//fgets(buf,250,fp);// JOINT?


	//	//HERE: back out of the nested curly braces, checking for new JOINTs and decrementing jloop each time.

	//	keepGoing = true;
	//	while (keepGoing)
	//	{
	//		fgets(buf,250,fp); fprintf(fpw,"%s",buf);
	//		bufp = strtok(buf," \t\n");
	//		char tempOne[40],tempTwo[40],tempThree[40];
	//		sprintf(tempOne,"}");
	//		sprintf(tempTwo,"JOINT");
	//		sprintf(tempThree,"MOTION");
	//		if (strstr(bufp,tempOne))//"}"
	//		{
	//			jloop++; loopDepth--;
	//		} else if (strstr(bufp,tempTwo)) {//"JOINT"
	//			newParent = joints[jc-(jloop-1)].parent;
	//			jloop = 0;
	//			keepGoing = false;
	//		} else if (strstr(bufp,tempThree)) {//"MOTION"
	//			//Con::printf("BVH -- loaded schema, proceeding to motion frames.");
	//			keepGoing = false;
	//			loadingJoints = false;
	//		} else {
	//			//Con::errorf("BVH -- problem, found no frames.");
	//			fclose(fp);
	//			fclose(fpw);
	//			return;
	//		}
	//	}
	//}
	//unsigned int numJoints = jc+1;
	////Con::printf("num joints: %d",numJoints);
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	////NOW: all done loading joints.  Array is filled up and available.  Next step is loading all the motion
	////frames, after picking up the numFrames and frameLength variables.
	//		
	//			
	//fgets(buf,250,fp);  fprintf(fpw,"Frames: 1\n");//fprintf(fpw,"%s",buf);//Frames: n

	////sscanf(bufp,"Frames:\t%d",&numFrames);
	////sscanf(bufp,"Frames:\t%d",1);
	////bufp = strtok(buf," \t\n");//"Frames:"
	////bufp = strtok(NULL, " \t\n"); sscanf(bufp,"%d",&numFrames);

	//fgets(buf,250,fp);  fprintf(fpw,"Frame Time: 1.0\n");//fprintf(fpw,"%s",buf);//Frame Time: f
	////sscanf(bufp,"Frame Time:\t%f",&frameTime);

	//fgets(buf,250,fp);  
	//bufp = strtok(buf," \t\n");
	////HERE: read the initial base node position, and then write out just one null frame.
	//sscanf(bufp,"%f",&p.x);
	//bufp = strtok(NULL, " \t\n");
	//sscanf(bufp,"%f",&p.y);
	//bufp = strtok(NULL, " \t\n");
	//sscanf(bufp,"%f",&p.z);
	//pos3[0] = p;
	//sprintf(rotation,40,"%3.2f %3.2f %3.2f ",pos3[0].x,pos3[0].y,pos3[0].z);

	////sprintf(rotation,40,"0.0 0.0 0.0 ");

	////sprintf(line,strlen(rotation),rotation);
	//sprintf(line,40,rotation);
	////Con::printf("printing joints");
	//for (unsigned int j=0; j<numJoints;j++)
	//{
	//	sprintf(rotation,40,"0.0 0.0 0.0 ");
	//	strcat(line,rotation);
	//}
	//fprintf(fpw,"%s\n",line);
	////all done.

	//fclose(fp);
	//fclose(fpw);
	
}

void fxFlexBody::cleanupBvh(const char *bvhFile)
{
	//unsigned int pc,rc,jc,jloop,loopDepth;
	//pc = 0; rc = 0; jc = 0; jloop = 0; loopDepth = 0;
	//int newParent = -1;
	//int numFrames = 0;
	//float frameTime = 0.0;
	//Ogre::Quaternion rot;
	//Ogre::Quaternion rots[20000];
	//Ogre::Vector3 trans[1000];
	////Ogre::Vector3 rotsByPart[20][300];//while we're temporarily hogging memory...

	//Ogre::Vector3 p;
	//Ogre::Vector3 r;
	//Ogre::Vector3 pos3[200];
	//Ogre::Vector3 rot3[200];
	//bvhJoint joints[200];
	//bool keepGoing,loadingJoints;
	//char filename[255],writename[255],buf[2500];
	//char line[2500], rotation[40], tempc[40],name[40];
	//char *bufp;
	//FILE *fpr = NULL;
	//FILE *fpw = NULL;

	//strcpy(filename,bvhFile);
	//fpr = fopen(filename,"r");
	//
	//if (fpr==NULL) 
	//{
	//	//Con::errorf("ERROR: can't open bvh file: %s",filename);
	//	return;
	//}

	//strcpy(writename,bvhFile);
	//String cleanName(writename);
	//int ext = cleanName.find(".bvh");
	//if (ext > -1)
	//{
	//	cleanName.erase(ext,4);
	//	cleanName += ".clean.bvh";
	//}

	//fpw = fopen(cleanName.c_str(),"w");

	//char chan1[10], chan2[10], chan3[10];
	//unsigned int numChannels;

	//fgets(buf,250,fpr); fprintf(fpw,"%s",buf);// HIERARCHY
	//fgets(buf,250,fpr); fprintf(fpw,"%s",buf);// ROOT
	//sscanf(buf,"%s %s",&tempc,&name);

	//joints[0].parent = -1;
	//strcpy(joints[0].name,name);
	//fgets(buf,250,fpr); fprintf(fpw,"%s",buf);// {
	//fgets(buf,250,fpr); fprintf(fpw,"%s",buf);// OFFSET x y z
	//sscanf(buf,"  OFFSET %f %f %f",&p.x,&p.y,&p.z);
	////fprintf(fpw,"  OFFSET %f %f %f",p.x,p.y,p.z);
	//joints[0].offset = p;
	//fgets(buf,250,fpr); fprintf(fpw,"%s",buf);// CHANNELS n ...
	//sscanf(buf,"  CHANNELS %d ",&numChannels);
	//if (numChannels==6)
	//	sscanf(buf,"  CHANNELS %d %s %s %s %s %s %s",&numChannels,&tempc,&tempc,&tempc,&chan1,&chan2,&chan3);
	//else if (numChannels==3)
	//	sscanf(buf,"  CHANNELS %d %s %s %s",&numChannels,&chan1,&chan2,&chan3);

	//joints[0].channels = numChannels;
	////Gotta sort out what order the rotations come in, PER NODE.
	//if (chan1[0]=='X') joints[0].chanrots[0] = 0;
	//else if (chan1[0]=='Y') joints[0].chanrots[0] = 1;
	//else if (chan1[0]=='Z')	joints[0].chanrots[0] = 2;
	//if (chan2[0]=='X') joints[0].chanrots[1] = 0;
	//else if (chan2[0]=='Y') joints[0].chanrots[1] = 1;
	//else if (chan2[0]=='Z') joints[0].chanrots[1] = 2;
	//if (chan3[0]=='X') joints[0].chanrots[2] = 0;
	//else if (chan3[0]=='Y') joints[0].chanrots[2] = 1;
	//else if (chan3[0]=='Z') joints[0].chanrots[2] = 2;

	//loopDepth = 0;

	//fgets(buf,250,fpr); fprintf(fpw,"%s",buf);
	//bufp = strtok(buf," \t\n");

	//loadingJoints = true;
	//while (loadingJoints)
	//{
	//	keepGoing = true;
	//	while (keepGoing)
	//	{
	//		sprintf(tempc,"JOINT");
	//		if (!strstr(bufp,tempc)) 
	//		{ 
	//			keepGoing = false; //End Site
	//			fgets(buf,250,fpr); fprintf(fpw,"%s",buf);//{
	//			fgets(buf,250,fpr); fprintf(fpw,"%s",buf);//terminal OFFSET x y z, ignore it
	//			fgets(buf,250,fpr); fprintf(fpw,"%s",buf);//}
	//			break; 
	//		}
	//		jc++;  loopDepth++;

	//		if (newParent>=0) {
	//			joints[jc].parent = newParent;
	//			newParent = -1;
	//		} else joints[jc].parent = jc-1;

	//		bufp = strtok(NULL, " \t\n");
	//		sscanf(bufp,"%s",name);				
	//		strcpy(joints[jc].name,name);
	//		fgets(buf,250,fpr); fprintf(fpw,"%s",buf);//{
	//		fgets(buf,250,fpr); fprintf(fpw,"%s",buf);//OFFSET x y z
	//		bufp = strtok(buf," \t\n");//"OFFSET"
	//		bufp = strtok(NULL, " \t\n"); sscanf(bufp,"%f",&joints[jc].offset.x);
	//		bufp = strtok(NULL, " \t\n"); sscanf(bufp,"%f",&joints[jc].offset.y);
	//		bufp = strtok(NULL, " \t\n"); sscanf(bufp,"%f",&joints[jc].offset.z);
	//		

	//		fgets(buf,250,fpr);//CHANNELS n s s s s s s
	//		bufp = strtok(buf," \t\n");//"CHANNELS"
	//		bufp = strtok(NULL, " \t\n"); sscanf(bufp,"%d",&joints[jc].channels);
	//		if (joints[jc].channels==6) {
	//			bufp = strtok(NULL," \t\n"); 
	//			bufp = strtok(NULL," \t\n"); 
	//			bufp = strtok(NULL," \t\n"); 
	//		}
	//		bufp = strtok(NULL," \t\n"); sscanf(bufp,"%s",&chan1);
	//		bufp = strtok(NULL," \t\n"); sscanf(bufp,"%s",&chan2);
	//		bufp = strtok(NULL," \t\n"); sscanf(bufp,"%s",&chan3);
	//		//Gotta sort out what order the rotations come in, PER NODE.
	//		if (chan1[0]=='X') joints[jc].chanrots[0] = 0;
	//		else if (chan1[0]=='Y') joints[jc].chanrots[0] = 1;
	//		else if (chan1[0]=='Z') joints[jc].chanrots[0] = 2; 
	//		if (chan2[0]=='X') joints[jc].chanrots[1] = 0;
	//		else if (chan2[0]=='Y') joints[jc].chanrots[1] = 1;
	//		else if (chan2[0]=='Z') joints[jc].chanrots[1] = 2;
	//		if (chan3[0]=='X') joints[jc].chanrots[2] = 0;
	//		else if (chan3[0]=='Y') joints[jc].chanrots[2] = 1;
	//		else if (chan3[0]=='Z') joints[jc].chanrots[2] = 2;
	//		//FIX: here we need to write out "CHANNELS 3 Zrotation Xrotation Yrotation"  no matter what it said before.
	//		//Problem is we need to indent it properly, which is a PITA.  Have to track level of nesting and add 
	//		//correct number of tabs, use strcat in a loop.
	//		//ALSO need to track whether the original file was made with tabs or spaces.  This could get ridiculous, though.

	//		strcpy(line,"\t");
	//		for (unsigned int k=0;k<loopDepth;k++) strcat(line,"\t");
	//		//strcat(line,"CHANNELS 3 Zrotation Xrotation Yrotation\n");
	//		sprintf(tempc,80,"CHANNELS 3 %s %s %s\n",chan1,chan2,chan3);
	//		strcat(line,tempc);

	//		fprintf(fpw,"%s",line);


	//		//Con::printf("JOINT %d: %s %d %3.2f %3.2f %3.2f",jc,joints[jc].name,joints[jc].channels,
	//			joints[jc].offset.x,joints[jc].offset.y,joints[jc].offset.z);
	//		
	//		fgets(buf,250,fpr); fprintf(fpw,"%s",buf);
	//		bufp = strtok(buf," \t\n");
	//	}
	//	//fgets(buf,250,fp);// JOINT?


	//	//HERE: back out of the nested curly braces, checking for new JOINTs and decrementing jloop each time.

	//	keepGoing = true;
	//	while (keepGoing)
	//	{
	//		fgets(buf,250,fpr); fprintf(fpw,"%s",buf);
	//		bufp = strtok(buf," \t\n");
	//		char tempOne[40],tempTwo[40],tempThree[40];
	//		sprintf(tempOne,"}");
	//		sprintf(tempTwo,"JOINT");
	//		sprintf(tempThree,"MOTION");
	//		if (strstr(bufp,tempOne))//"}"
	//		{
	//			jloop++; loopDepth--;
	//		} else if (strstr(bufp,tempTwo)) {//"JOINT"
	//			newParent = joints[jc-(jloop-1)].parent;
	//			keepGoing = false;
	//		} else if (strstr(bufp,tempThree)) {//"MOTION"
	//			//Con::printf("BVH -- loaded schema, proceeding to motion frames.");
	//			keepGoing = false;
	//			loadingJoints = false;
	//		} else {
	//			//Con::errorf("BVH -- problem, found no frames.");
	//			fclose(fpr);
	//			fclose(fpw);
	//			return;
	//		}
	//	}
	//}
	//unsigned int numJoints = jc+1;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	////NOW: all done loading joints.  Array is filled up and available.  Next step is loading all the motion
	////frames, after picking up the numFrames and frameLength variables.

	//fgets(buf,250,fpr); fprintf(fpw,"%s",buf);//Frames: n

	//sscanf(bufp,"Frames:\t%d",&numFrames);
	////bufp = strtok(buf," \t\n");//"Frames:"
	////bufp = strtok(NULL, " \t\n"); sscanf(bufp,"%d",&numFrames);

	//fgets(buf,250,fpr); fprintf(fpw,"%s",buf);//Frame Time: f
	//sscanf(bufp,"Frame Time:\t%f",&frameTime);
	////bufp = strtok(buf," \t\n");//"Frame Time:"
	////bufp = strtok(NULL, " \t\n"); sscanf(bufp,"%f",&frameTime);

	//if (!numFrames || (numFrames<=0)) 
	//{
	//	//Con::errorf("BVH -- couldn't read numFrames, bailing.");
	//	fclose(fpr);
	//	fclose(fpw);
	//	return;
	//}

	//for (unsigned int c=0;c<numFrames;c++)
	//{
	//	if (!fgets(buf,2500,fpr)) 
	//	{
	//		//Con::errorf("BVH -- not enough frames, bailing out.");
	//		fclose(fpr);
	//		fclose(fpw);
	//		return;
	//	}

	//	//fprintf(fpw,"%s",buf);

	//	pc=0;

	//	for (unsigned int d=0; d < numJoints;d++)
	//	{
	//		if (d==0) 
	//			bufp = strtok(buf," \t\n");
	//		else 
	//			bufp = strtok(NULL, " \t\n");

	//		if (joints[d].channels == 3)
	//		{//NOTE: this is NOT necessarily x,y,z, just using them for convenience.  Order is whatever it is in bvh.
	//			sscanf(bufp,"%f",&p.x);
	//			bufp = strtok(NULL, " \t\n");
	//			sscanf(bufp,"%f",&p.y);
	//			bufp = strtok(NULL, " \t\n");
	//			sscanf(bufp,"%f",&p.z);
	//			rot3[d] = p;
	//		}
	//		else
	//		{
	//			sscanf(bufp,"%f",&p.x);
	//			bufp = strtok(NULL, " \t\n");
	//			sscanf(bufp,"%f",&p.y);
	//			bufp = strtok(NULL, " \t\n");
	//			sscanf(bufp,"%f",&p.z);
	//			pos3[d] = p;

	//			bufp = strtok(NULL, " \t\n");
	//			sscanf(bufp,"%f",&p.x);
	//			bufp = strtok(NULL, " \t\n");
	//			sscanf(bufp,"%f",&p.y);
	//			bufp = strtok(NULL, " \t\n");
	//			sscanf(bufp,"%f",&p.z);
	//			rot3[d] = p;
	//		}
	//	}
	//	

	//	sprintf(rotation,40,"%3.2f %3.2f %3.2f ",pos3[0].x,pos3[0].y,pos3[0].z);
	//	sprintf(line,40,rotation);
	//	for (unsigned int j=0; j<numJoints;j++)
	//	{
	//		sprintf(rotation,40,"%3.2f %3.2f %3.2f ",rot3[j].x,rot3[j].y,rot3[j].z);
	//		//sprintf(line,strlen(rotation),rotation);
	//		//sprintf(line,40,rotation);
	//		strcat(line,rotation);
	//	}

	//	fprintf(fpw,"%s\n",line);
	//}

	//fclose(fpr);
	//fclose(fpw);
	

}

void fxFlexBody::importDir(bool importGround,const char *bvh_dir,const char *dsq_dir,const char *bvhProfile)
{
	//FileStream *outstream;
	//char dos_command[512],filename[255],listfile[255],bvh_name[255];
	//char buf[255];
	//char *bufp;
	//bool isCyclic;
	////sprintf(filepath,"%s",mShapeInstance->getShape()->mSourceResource->path);

	////HERE: need to do a strtok on "/", grab each intervening string and pad it with "\\" instead.
	//String bvhDir(bvh_dir);
	//String dsqDir(dsq_dir);
	//String bvhPath(bvh_dir);
	//String bvhPathDir;
	//String dosDir;
	//unsigned int slashPos;
	//while (strchr(bvhPath.c_str(),'/'))
	//{
	//	slashPos = bvhPath.length() - strlen(strchr(bvhPath.c_str(),'/'));
	//	bvhPathDir = bvhPath;
	//	bvhPath.erase(0,slashPos+1);//erase up to and including the slash
	//	bvhPathDir.erase(slashPos,strlen(bvhPathDir)-slashPos);
	//	//Con::printf("directory: %s, remaining path: %s",bvhPathDir.c_str(),bvhPath.c_str());
	//	dosDir += bvhPathDir;
	//	dosDir += '\\';		
	//}
	//dosDir += bvhPath;
	////Con::printf("final dos dir: %s",dosDir.c_str());

	//sprintf(dos_command,"dir \"%s\\*.bvh\" /B > \"%s\\bvhfiles.txt\"",dosDir.c_str(),dosDir.c_str());
	////Con::errorf("dos command: %s",dos_command);
	//system(dos_command);

	//sprintf(listfile,"%s/bvhfiles.txt",bvhDir.c_str());
	//	

	////for (unsigned int i=0;i<10000000;i++);//pause a bit
	////Con::errorf("list file: %s",listfile);
	//FILE *fplist = fopen(listfile,"r");
	//if (fplist==NULL) 
	//{
	//	//Con::errorf("ERROR: can't open bvh list");
	//	return;
	//}
	//TSShape *kShape = getShapeInstance()->getShape();
	////Con::errorf("opened list file: %s",listfile);
	//while (fgets(buf,255,fplist)) 
	//{//HERE: for each entry, you need to importBvh it, saveOut it, and dropSeq it.  See if it works just like that.
	//	//Con::errorf("buffer: %s",buf);
	//	if (strstr(buf,".cyclic."))
	//		isCyclic = true;
	//	else isCyclic = false;
	//	String bvhName(buf);
	//	bufp = strtok(buf,".");//get name w/o ".bvh" -- IMPORTANT: this cuts off at the first '.',
	//	//so you can't use periods in the name of the sequence itself.  Probably good, given how Torque can be.
	//	sprintf(bvh_name,"%s",bufp);

	//		
	//	//if (strlen(bvhDir))
	//	//	//Con::errorf("trying to open bvh file: %s, dir: %s",bvh_name,bvhDir.c_str());
	//	//else
	//	bvhName.erase(bvhName.length()-1,1);//(get rid of trailing carriage return)
	//	//Con::errorf("bvhName: %s, length %d",bvhName.c_str(),bvhName.length());

	//	String bvhFile(bvhDir);
	//	bvhFile += '/';
	//	bvhFile += bvhName;
	//	//bvhFile += ".bvh";
	//	//Con::errorf("trying to open bvh file: %s",bvhFile.c_str());

	//	importBvh(importGround,bvhFile.c_str(),bvhProfile);

	//	if (kShape->sequences.size()>1)
	//	{
	//		kShape->dropSequence(0);
	//		if (mUltraframeSets.size()>0)
	//			mUltraframeSets.erase((unsigned int)0);

	//		//and, to get rid of the rest of them...
	//		while (kShape->sequences.size()>1){
	//			//Con::errorf("dropping another sequence");
	//			kShape->dropSequence(0);
	//		}
	//		while (mUltraframeSets.size()>0)
	//			mUltraframeSets.erase((unsigned int)0);

	//		if (isCyclic) kShape->sequences[0].flags = TSShape::Cyclic;
	//		else kShape->sequences[0].flags = 0;//NOTE: this will have to simply subtract cyclic, once
	//		//we start including blend as an option, and/or other flags.

	//		//Con::errorf("dropped sequence (0), size %d",getShapeInstance()->getShape()->sequences.size());

	//		sprintf(filename,"%s/%s.dsq",dsqDir.c_str(),bufp);	

	//		//if (!gResourceManager->openFileForWrite(outstream,filename)) {
	//		if ( (outstream = FileStream::createAndOpen( filename, Torque::FS::File::Write) ) == NULL) {
	//			//Con::printf("whoops, name no good!"); 
	//		} else {
	//			getShapeInstance()->getShape()->exportSequences((Stream *)outstream);
	//			outstream->close();
	//		}
	//	}
	//}
	//fclose(fplist);
	
}

void fxFlexBody::saveBvhDir(const char *inDir, const char *format)
{

	//FileStream outstream;
	//char dos_command[512],filename[255],listfile[255],dsq_name[255];
	//char buf[255];
	//char *bufp;
	//bool isCyclic;

	//String dsqDir(inDir);
	//String bvhOutDir(inDir);//(outDir);
	//String dsqPath(inDir);
	//String dsqPathDir;
	//String dosDir;
	//unsigned int slashPos;
	//while (strchr(dsqPath.c_str(),'/'))
	//{
	//	slashPos = dsqPath.length() - strlen(strchr(dsqPath.c_str(),'/'));
	//	dsqPathDir = dsqPath;
	//	dsqPath.erase(0,slashPos+1);//erase up to and including the slash
	//	dsqPathDir.erase(slashPos,strlen(dsqPathDir)-slashPos);
	//	//Con::printf("directory: %s, remaining path: %s",dsqPathDir.c_str(),dsqPath.c_str());
	//	dosDir += dsqPathDir;
	//	dosDir += '\\';		
	//}
	//dosDir += dsqPath;
	////Con::printf("final dos dir: %s",dosDir.c_str());

	//sprintf(dos_command,"dir \"%s\\*.dsq\" /B > \"%s\\dsqfiles.txt\"",dosDir.c_str(),dosDir.c_str());
	////Con::errorf("dos command: %s",dos_command);
	//system(dos_command);

	//sprintf(listfile,"%s/dsqfiles.txt",dsqDir.c_str());
	//	

	////for (unsigned int i=0;i<10000000;i++);//pause a bit
	////Con::errorf("list file: %s",listfile);
	//FILE *fplist = fopen(listfile,"r");
	//if (fplist==NULL) 
	//{
	//	//Con::errorf("ERROR: can't open dsq list");
	//	return;
	//}
	//TSShape *kShape = getShapeInstance()->getShape();
	//const String myPath = getShapeInstance()->mShapeResource.getPath().getPath();
	////Con::errorf("opened list file: %s",listfile);
	//while (fgets(buf,255,fplist)) 
	//{//HERE: for each entry, you need to importBvh it, saveOut it, and dropSeq it.  See if it works just like that.
	//	//Con::errorf("buffer: %s",buf);
	//	String dsqName(buf);
	//	unsigned int extPos = dsqName.find(".dsq",0,String::NoCase|String::Right);//dsqName.length()
	//	//Note re: StrFind - you don't have to specify starting at the end if you're using Right.
	//	//bufp = strtok(buf,".");//get name w/o extension//WHOOPS!  Need to find this from the RIGHT!
	//	//or else we chop everything after the first "." - which could be "military.1.dsq" for all we know.
	//	//sprintf(dsq_name,"%s",bufp);
	//	sprintf(dsq_name,"%s",dsqName.substr(0,extPos).c_str());
	//	//if (strlen(bvhDir))
	//	//	//Con::errorf("trying to open bvh file: %s, dir: %s",bvh_name,bvhDir.c_str());
	//	//else
	//	dsqName.erase(dsqName.length()-1,1);//(get rid of trailing carriage return)

	//	String dsqFile(dsqDir);
	//	dsqFile += '/';
	//	dsqFile += dsqName;
	//	//bvhFile += ".bvh";
	//	//Con::errorf("converting dsq to bvh: %s",dsqFile.c_str());
	//	
	//	//importBvh(importGround,bvhFile.c_str());
	//	loadDsq(dsqFile.c_str());
	//	
	//	//HERE!  Need to remove hands, if they exist, because other player models don't have hands.  Make this optional, later, for more
	//	//complex applications of the bvh (poser, etc.)
	//	if (strstr(myPath.c_str(),"ACK")) 
	//		kShape->removeAckHands(kShape->sequences.size()-1);


	//	if (kShape->sequences[kShape->sequences.size()-1].isCyclic()) 
	//		isCyclic = true;
	//	else isCyclic = false;

	//	if (isCyclic)
	//		sprintf(filename,"%s/%s.cyclic.bvh",bvhOutDir.c_str(),dsq_name);//bufp);
	//	else 
	//		sprintf(filename,"%s/%s.bvh",bvhOutDir.c_str(),dsq_name);//bufp);

	//	saveBvh(kShape->sequences.size()-1,filename,format);//HERE: need format, not "native"

	//	//if (kShape->sequences.size()>1)
	//	//{
	//	//	kShape->dropSequence(0);
	//	//	mUltraframeSets.erase((unsigned int)0);

	//	//	//and, to get rid of the rest of them...
	//	//	while (kShape->sequences.size()>1)
	//	//		kShape->dropSequence(0);

	//	//	while (mUltraframeSets.size()>0)
	//	//		mUltraframeSets.erase((unsigned int)0);

	//	//}
	//}
	//fclose(fplist);
}

void fxFlexBody::groundCaptureDir(const char *inDir, const char *outDir)
{//HERE: go through inDir and convert all, save to outDir.  If same, overwrite.

	//FileStream *outstream;
	//char dos_command[512],filename[255],listfile[255],dsq_name[255];
	//char buf[255];
	//char *bufp;
	////sprintf(filepath,"%s",mShapeInstance->getShape()->mSourceResource->path);
	//////Con::errorf("dsq dir: %s",dsq_dir);

	////HERE: need to do a strtok on "/", grab each intervening string and pad it with "\\" instead.
	//String dsqDir(inDir);
	//String dsqOutDir(outDir);
	//String dsqPath(inDir);
	//String dsqPathDir;
	//String dosDir;
	//unsigned int slashPos;
	//while (strchr(dsqPath.c_str(),'/'))
	//{
	//	slashPos = dsqPath.length() - strlen(strchr(dsqPath.c_str(),'/'));
	//	dsqPathDir = dsqPath;
	//	dsqPath.erase(0,slashPos+1);//erase up to and including the slash
	//	dsqPathDir.erase(slashPos,strlen(dsqPathDir)-slashPos);
	//	//Con::printf("directory: %s, remaining path: %s",dsqPathDir.c_str(),dsqPath.c_str());
	//	dosDir += dsqPathDir;
	//	dosDir += '\\';		
	//}
	//dosDir += dsqPath;
	////Con::printf("final dos dir: %s",dosDir.c_str());

	//sprintf(dos_command,"dir \"%s\\*.dsq\" /B > \"%s\\dsqfiles.txt\"",dosDir.c_str(),dosDir.c_str());
	////Con::errorf("dos command: %s",dos_command);
	//system(dos_command);

	//sprintf(listfile,"%s/dsqfiles.txt",dsqDir.c_str());
	//	

	////for (unsigned int i=0;i<10000000;i++);//pause a bit
	////Con::errorf("list file: %s",listfile);
	//FILE *fplist = fopen(listfile,"r");
	//if (fplist==NULL) 
	//{
	//	//Con::errorf("ERROR: can't open dsq list");
	//	return;
	//}
	//TSShape *kShape = getShapeInstance()->getShape();
	////Con::errorf("opened list file: %s",listfile);
	//while (fgets(buf,255,fplist)) 
	//{//HERE: for each entry, you need to importBvh it, saveOut it, and dropSeq it.  See if it works just like that.
	//	//Con::errorf("buffer: %s",buf);
	//	String dsqName(buf);
	//	bufp = strtok(buf,".");//get name w/o extension
	//	sprintf(dsq_name,"%s",bufp);
	//	//if (strlen(bvhDir))
	//	//	//Con::errorf("trying to open bvh file: %s, dir: %s",bvh_name,bvhDir.c_str());
	//	//else
	//	dsqName.erase(dsqName.length()-1,1);//(get rid of trailing carriage return)
	//	//Con::errorf("dsqName: %s, length %d",dsqName.c_str(),dsqName.length());

	//	String dsqFile(dsqDir);
	//	dsqFile += '/';
	//	dsqFile += dsqName;
	//	//bvhFile += ".bvh";
	//	//Con::errorf("trying to groundCapture dsq file: %s",dsqFile.c_str());
	//	
	//	//importBvh(importGround,bvhFile.c_str());
	//	loadDsq(dsqFile.c_str());
	//	kShape->groundCaptureSeq(kShape->sequences.size()-1);//kShape->findSequence(kShape->getName(dsqFile.c_str()
	//	
	//	if (kShape->sequences.size()>1)
	//	{
	//		kShape->dropSequence(0);

	//		//and, to get rid of the rest of them...
	//		while (kShape->sequences.size()>1)
	//			kShape->dropSequence(0);

	//		while (mUltraframeSets.size()>0)
	//			mUltraframeSets.erase((unsigned int)0);

	//		if (ECSTASY_LABEL_GROUNDFRAMES)
	//			sprintf(filename,"%s/%s.groundframes.dsq",dsqOutDir.c_str(),bufp);	
	//		else 
	//			sprintf(filename,"%s/%s.dsq",dsqOutDir.c_str(),bufp);	

	//		//Con::errorf("writing to file: %s",filename);

	//		//if (!gResourceManager->openFileForWrite(outstream,filename)) {
	//		if ((outstream = FileStream::createAndOpen( filename, Torque::FS::File::Write))==NULL) {
	//			//Con::printf("whoops, name no good!"); 
	//		} else {
	//			getShapeInstance()->getShape()->exportSequences((Stream *)outstream);
	//			outstream->close();
	//		}
	//	}
	//}
	//fclose(fplist);
}

void fxFlexBody::saveBvhCfg(FILE *fpc,bvhCfgData *cfg)
{
	fprintf(fpc,"SAMPLE RATE = 1;\n");
	fprintf(fpc,"SCALE = INCHES;\n");

	for (unsigned int i=0;i<cfg->numBvhNodes;i++)
	{
		Ogre::Vector3 posA,posB,fixA,fixB;
		posA = cfg->bvhPoseRotsA[i];
		posB = cfg->bvhPoseRotsB[i];
		fixA = cfg->axesFixRotsA[i];
		fixB = cfg->axesFixRotsB[i];
		fprintf(fpc,"%d;%d;%d;(%3.2f,%3.2f,%3.2f);(%3.2f,%3.2f,%3.2f);(%3.2f,%3.2f,%3.2f);(%3.2f,%3.2f,%3.2f);\n",
			cfg->bvhNodes[i],cfg->dtsNodes[i],cfg->nodeGroups[i],posA.x,posA.y,posA.z,posB.x,posB.y,posB.z,
			fixA.x,fixA.y,fixA.z,fixB.x,fixB.y,fixB.z);//mRadToDeg(posA.x)?
	}
}

unsigned int fxFlexBody::loadBvhCfgDB(bvhCfgData *cfg,unsigned int profile_id)
{
	//unsigned int rc=0;
	//char select_query[512];
	//String bvhName,dtsName;
	//TSShape *kShape = getShapeInstance()->getShape();
	//
	//SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(mPM)->getSQL();
	//if (!sql) return 0;
	//if (!profile_id) return 0;
	//if (sql->OpenDatabase("EcstasyMotion.db"))
	//{
	//	char id_query[512],insert_query[512];
	//	int result;
	//	sqlite_resultset *resultSet;
	//	if (profile_id)
	//	{
	//		sprintf(select_query,"SELECT scale FROM bvhProfile WHERE id = %d;",profile_id);
	//		result = sql->ExecuteSQL(select_query);
	//		resultSet = sql->GetResultSet(result);
	//		if (resultSet->iNumRows == 1) 
	//			cfg->bvhScale = strtod(resultSet->vRows[0]->vColumnValues[0]);
	//		if (mShapeSize) cfg->bvhScale *= mShapeSize/2.0;

	//		sprintf(select_query,"SELECT bvhNodeName, dtsNodeName,\
	//								 nodeGroup, poseRotA_x, poseRotA_y, poseRotA_z,\
	//								 poseRotB_x, poseRotB_y, poseRotB_z,\
	//								 fixRotA_x, fixRotA_y, fixRotA_z,\
	//								 fixRotB_x, fixRotB_y, fixRotB_z \
	//						 FROM bvhProfileNode WHERE bvh_profile_id = %d;",profile_id);
	//		result = sql->ExecuteSQL(select_query);
	//		resultSet = sql->GetResultSet(result);
	//		rc = resultSet->iNumRows;
	//		for (unsigned int i=0;i<resultSet->iNumRows;i++)
	//		{
	//			cfg->bvhNames[i] = String(resultSet->vRows[i]->vColumnValues[0]);
	//			cfg->bvhNodes[i] = i;//(right?) wrong!  Need cfg to be able to skip finger nodes in bvh, for example.  Need a findNode or getID function for BVH.
	//			//Actually, if we want real node ids we need to load the skeleton FIRST, not after this.  Then query db for this node name. 
	//			int kID = kShape->findNode(resultSet->vRows[i]->vColumnValues[1]);
	//			cfg->dtsNodes[i] = kID;
	//			cfg->nodeGroups[i] = strtol(resultSet->vRows[i]->vColumnValues[2]);//HERE: count up distinct 
	//			cfg->bvhPoseRotsA[i].x = mDegToRad(strtod(resultSet->vRows[i]->vColumnValues[3]));
	//			cfg->bvhPoseRotsA[i].y = mDegToRad(strtod(resultSet->vRows[i]->vColumnValues[4]));
	//			cfg->bvhPoseRotsA[i].z = mDegToRad(strtod(resultSet->vRows[i]->vColumnValues[5]));
	//			cfg->bvhPoseRotsB[i].x = mDegToRad(strtod(resultSet->vRows[i]->vColumnValues[6]));
	//			cfg->bvhPoseRotsB[i].y = mDegToRad(strtod(resultSet->vRows[i]->vColumnValues[7]));
	//			cfg->bvhPoseRotsB[i].z = mDegToRad(strtod(resultSet->vRows[i]->vColumnValues[8]));
	//			cfg->axesFixRotsA[i].x = mDegToRad(strtod(resultSet->vRows[i]->vColumnValues[9]));
	//			cfg->axesFixRotsA[i].y = mDegToRad(strtod(resultSet->vRows[i]->vColumnValues[10]));
	//			cfg->axesFixRotsA[i].z = mDegToRad(strtod(resultSet->vRows[i]->vColumnValues[11]));
	//			cfg->axesFixRotsB[i].x = mDegToRad(strtod(resultSet->vRows[i]->vColumnValues[12]));
	//			cfg->axesFixRotsB[i].y = mDegToRad(strtod(resultSet->vRows[i]->vColumnValues[13]));
	//			cfg->axesFixRotsB[i].z = mDegToRad(strtod(resultSet->vRows[i]->vColumnValues[14]));
	//		}
	//		if (rc>0) //Con::printf("Found %d bvh profile nodes in the database.",rc);
	//	}
	//	sql->CloseDatabase();
	//	delete sql;
	//}
	//
	////Now, sort: inefficient bubble sort, but who cares, it's trivial anyway.
	//int sortNodes[MAX_BVH_NODES];
	//for (unsigned int i=0; i<rc; i++) sortNodes[i] = cfg->dtsNodes[i];

	//for (unsigned int i=0; i<rc; i++)
	//{
	//	for (unsigned int j=0; j<rc-1; j++)
	//	{
	//		int temp;
	//		if (sortNodes[j] > sortNodes[j+1]) 
	//		{
	//			temp = sortNodes[j];
	//			sortNodes[j] = sortNodes[j+1];
	//			sortNodes[j+1] = temp;
	//		}
	//	}
	//}

	//for (unsigned int i=0; i<rc; i++)
	//	for (unsigned int j=0; j<rc; j++)
	//		if (cfg->dtsNodes[j] == sortNodes[i]) cfg->orderNodes[i] = j;

	//cfg->numBvhNodes = rc;
	//cfg->numDtsNodes = rc;
	//cfg->numNodeGroups = 5; //FIX, need to 
	//return rc;
	return 0;
}

unsigned int fxFlexBody::loadBvhCfg(FILE *fpc,bvhCfgData *cfg,unsigned int profile_id)
{
	//HERE: loop through cfg file, and load up bvhNodes[], bvhPoseRots[], and axesFixRots[]
	//unsigned int rc,bc;
	//float PAx,PAy,PAz,PBx,PBy,PBz,FAx,FAy,FAz,FBx,FBy,FBz;
	//bool keepGoing,isRelevant,loadingJoints;
	//Ogre::Vector3 worldTrans;
	//int sortNodes[MAX_BVH_NODES];
	//int lastNodeGroup = -1;
	//char buf[2500],scaleUnits[10],bvh_name[255];
	//char *bufp;

	//TSShape *kShape = getShapeInstance()->getShape();

	//rc = bc = 0;
	//cfg->numBvhNodes = 0;
	//cfg->numDtsNodes = 0;
	//for (unsigned int i=0;i<MAX_BVH_NODES;i++)
	//{
	//	cfg->bvhNodes[i] = -1;
	//	cfg->dtsNodes[i] = -1;
	//}
	//SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(mPM)->getSQL();

	//if (!sql) return 0;
	//if (!profile_id) return 0;
	//if (!sql->OpenDatabase("EcstasyMotion.db")) { delete sql; return 0; }
	//
	//char update_query[512],insert_query[512];
	//int result;
	//sqlite_resultset *resultSet;

	//result = sql->ExecuteSQL("BEGIN TRANSACTION;");

	//while  (fgets(buf,2500,fpc))
	//{
	//	//if (buf[0]=='#') continue;

	//	if (!strncmp(buf,"SAMPLE",6))
	//		continue;
	//	else if (!strncmp(buf,"SCALE",5))
	//	{
	//		sscanf(buf,"SCALE = %s;",&scaleUnits);
	//		if (strstr(scaleUnits,"INCHES"))
	//			cfg->bvhScale = 1.0 / 39.0;
	//		else
	//			cfg->bvhScale = 1.0;

	//		sprintf(update_query,"UPDATE bvhProfile SET scale=%f WHERE id=%d;",cfg->bvhScale,profile_id);
	//		result = sql->ExecuteSQL(update_query);
	//		//Con::printf("Scale %s = %f",scaleUnits,cfg->bvhScale);
	//		if (mShapeSize) cfg->bvhScale *= mShapeSize/2.0; //2.0 because it's assumed that our mocap actor is two meters tall... 
	//		//This is not safe either.  Better fix would be exposing it as a variable on the gui too.
	//	} 
	//	else
	//	{
	//		//HERE!!  analyze the first two inputs.  If you have two integers, then treat them as
	//		//node indices, otherwise consider them node name strings.

	//		String kArg1,kArg2;
	//		String kBuffer(buf); 
	//		char name1[255],name2[255];

	//		int currPos = kBuffer.find(';',0);
	//		kArg1 = kBuffer;
	//		kArg1.erase(currPos,kBuffer.length() - currPos);
	//		kArg2 = kBuffer;
	//		kArg2.erase(0,currPos+1);//+1 to eat up semicolon as well.
	//		currPos = kArg2.find(';',0);
	//		kArg2.erase(currPos,kArg2.length() - currPos);

	//		int bvhID,dtsID,nodeGroup;
	//		if (dIsAllNumeric(kArg1.c_str())&&dIsAllNumeric(kArg2.c_str())) 
	//		{
	//			sscanf(buf,"%d;%d;%d;(%f,%f,%f);(%f,%f,%f);(%f,%f,%f);(%f,%f,%f);",
	//				&bvhID,&dtsID,&nodeGroup,
	//				&PAx,&PAy,&PAz,&PBx,&PBy,&PBz,&FAx,&FAy,&FAz,&FBx,&FBy,&FBz);
	//			cfg->bvhNodes[rc] = bvhID;
	//			cfg->dtsNodes[rc] = dtsID;
	//			cfg->nodeGroups[rc] = nodeGroup;
	//			cfg->usingNames = false;
	//			if (nodeGroup != lastNodeGroup)
	//			{
	//				cfg->numNodeGroups++;
	//				lastNodeGroup = nodeGroup;
	//			}
	//			sprintf(name1,"%d",bvhID);//Unfortunately, we don't have the skeleton loaded yet, so we don't know the names.
	//			sprintf(name2,"%s",kShape->getName(kShape->nodes[dtsID].nameIndex).c_str());
	//			sprintf(insert_query,"INSERT INTO bvhProfileNode (bvh_profile_id,bvhNodeName,dtsNodeName,\
	//								 nodeGroup,poseRotA_x,poseRotA_y,poseRotA_z,\
	//								 poseRotB_x,poseRotB_y,poseRotB_z,\
	//								 fixRotA_x,fixRotA_y,fixRotA_z,\
	//								 fixRotB_x,fixRotB_y,fixRotB_z)\
	//								 VALUES (%d,%d,'%s',%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f);",
	//								 profile_id,bvhID,name2,nodeGroup,PAx,PAy,PAz,PBx,PBy,PBz,FAx,FAy,FAz,FBx,FBy,FBz);
	//			result = sql->ExecuteSQL(insert_query);
	//		} else {
	//			sscanf(buf,"%s ; %s ;%d;(%f,%f,%f);(%f,%f,%f);(%f,%f,%f);(%f,%f,%f);",
	//				&name1,&name2,&nodeGroup,
	//				&PAx,&PAy,&PAz,&PBx,&PBy,&PBz,&FAx,&FAy,&FAz,&FBx,&FBy,&FBz);
	//			cfg->bvhNames[rc] = String(name1);
	//			cfg->usingNames = true;
	//			int kID = kShape->findNode(name2);
	//			if (kID>=0)
	//			{
	//				cfg->dtsNodes[rc] = kID;//rc = count of total nodes in bvh file
	//				cfg->bvhNodes[bc++] = rc;//bc = count of bvh nodes linked to shape nodes.
	//			} else cfg->dtsNodes[rc] = -1;
	//			cfg->nodeGroups[rc] = nodeGroup;
	//			
	//			//HERE: insert into bvhProfileNode.
	//			sprintf(insert_query,"INSERT INTO bvhProfileNode (bvh_profile_id,bvhNodeName,dtsNodeName,\
	//								 nodeGroup,poseRotA_x,poseRotA_y,poseRotA_z,\
	//								 poseRotB_x,poseRotB_y,poseRotB_z,\
	//								 fixRotA_x,fixRotA_y,fixRotA_z,\
	//								 fixRotB_x,fixRotB_y,fixRotB_z)\
	//								 VALUES (%d,'%s','%s',%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f);",
	//								 profile_id,name1,name2,nodeGroup,PAx,PAy,PAz,PBx,PBy,PBz,FAx,FAy,FAz,FBx,FBy,FBz);
	//			result = sql->ExecuteSQL(insert_query);

	//			//bvhNodes[rc] = ??;//HERE: I don't know if I'm skipping any nodes in the bvh.
	//			//I should not be allowed to skip any nodes. (NOT!  FIX!) that means I need to assign 
	//			//-1 to the dts side for any nodes that don't count.
	//			//bvhNodes[rc] = 0;//HERE: need a name search function for the bvh file, 
	//			//have to open that previously and grab the node names.
	//		}

	//		cfg->bvhPoseRotsA[rc].x = mDegToRad(PAx); cfg->bvhPoseRotsB[rc].x = mDegToRad(PBx);
	//		cfg->bvhPoseRotsA[rc].y = mDegToRad(PAy); cfg->bvhPoseRotsB[rc].y = mDegToRad(PBy);
	//		cfg->bvhPoseRotsA[rc].z = mDegToRad(PAz); cfg->bvhPoseRotsB[rc].z = mDegToRad(PBz);
	//		cfg->axesFixRotsA[rc].x = mDegToRad(FAx); cfg->axesFixRotsB[rc].x = mDegToRad(FBx);
	//		cfg->axesFixRotsA[rc].y = mDegToRad(FAy); cfg->axesFixRotsB[rc].y = mDegToRad(FBy);
	//		cfg->axesFixRotsA[rc].z = mDegToRad(FAz); cfg->axesFixRotsB[rc].z = mDegToRad(FBz);
	//		rc++;//rc = relevant count -- the ones that are in the cfg file are the only ones that count.
	//	}
	//}

	////Now, sort: inefficient bubble sort, but who cares, it's trivial anyway.
	//for (unsigned int i=0; i<rc; i++) sortNodes[i] = cfg->dtsNodes[i];

	//for (unsigned int i=0; i<rc; i++)
	//{
	//	for (unsigned int j=0; j<rc-1; j++)
	//	{
	//		int temp;
	//		if (sortNodes[j] > sortNodes[j+1]) 
	//		{
	//			temp = sortNodes[j];
	//			sortNodes[j] = sortNodes[j+1];
	//			sortNodes[j+1] = temp;
	//		}
	//	}
	//}
	//for (unsigned int i=0; i<rc; i++)
	//	for (unsigned int j=0; j<rc; j++)
	//		if (cfg->dtsNodes[j] == sortNodes[i]) cfg->orderNodes[i] = j;
	//for (unsigned int i=0; i<rc; i++)
	//{
	//	//Con::printf("%d  bvh %d  dts %d  ordered dts %d",i,cfg->bvhNodes[i],cfg->dtsNodes[i],cfg->orderNodes[i]);
	//}

	//cfg->numBvhNodes = rc;
	//cfg->numDtsNodes = bc;
	//
	////saveBvhCfgXml("art/shapes/bvhCfg/testBvhCfg.xml",cfg);
	////saveBvhCfgSql(cfg);

	//result = sql->ExecuteSQL("END TRANSACTION;");

	//sql->CloseDatabase();
	//delete sql;
	//return rc;
	return 0;
}

void fxFlexBody::saveBvhCfgSql(bvhCfgData *cfg)
{
	//SQLiteObject *sql = new SQLiteObject();
	////Con::errorf("trying to open database.");
	//if (sql->OpenDatabase("Ecstasy.db"))
	//{
	//	sql->ExecuteSQL("INSERT INTO model (filename) VALUES ('./art/shapes/ACK/male/kungfu.dts');");
	//	int result = sql->ExecuteSQL("SELECT * from model;");
	//	sqlite_resultset *resultSet = sql->GetResultSet(result);
	//	//Con::errorf("-------------------opened database. %d results, %d columns.",resultSet->iNumRows,resultSet->iNumCols);
	//	for (unsigned int i=0; i<resultSet->iNumRows; i++)
	//	{
	//		//Con::errorf("%s; %s;",resultSet->vRows[i]->vColumnNames[0],resultSet->vRows[i]->vColumnValues[0]);
	//		//Con::errorf("%s; %s;",resultSet->vRows[i]->vColumnNames[1],resultSet->vRows[i]->vColumnValues[1]);
	//	}
	//}
	//sql->CloseDatabase();
	//delete sql;

}

unsigned int fxFlexBody::loadBvhCfgXml(const char *xml_file,bvhCfgData *cfg)
{
	/////// grabbed straight from old loadBvhCfg - need to weed out unused vars /////////
	//unsigned int nodeCount = 0;
	////float PAx,PAy,PAz,PBx,PBy,PBz,FAx,FAy,FAz,FBx,FBy,FBz;
	////bool keepGoing,isRelevant,loadingJoints;
	////Ogre::Vector3 worldTrans;
	////int sortNodes[MAX_BVH_NODES];
	////int lastNodeGroup = -1;
	////char buf[2500],scaleUnits[10],bvh_name[255];
	////char *bufp;
	//float x,y,z;
	//TSShape *kShape = getShapeInstance()->getShape();

	//cfg->numBvhNodes = 0;
	//cfg->numDtsNodes = 0;
	//for (unsigned int i=0;i<MAX_BVH_NODES;i++)
	//{
	//	cfg->bvhNodes[i] = -1;
	//	cfg->dtsNodes[i] = -1;
	//}

	////rc = bc = 0;
	////cfg->numBvhNodes = 0;
	////cfg->numDtsNodes = 0;
	////for (unsigned int i=0;i<MAX_BVH_NODES;i++)
	////{
	////	cfg->bvhNodes[i] = -1;
	////	cfg->dtsNodes[i] = -1;
	////}

	///////// end old loadBvhCfg /////////
	//
	//SimXMLDocument *doc = new SimXMLDocument();
	//doc->registerObject();

	//int loaded = doc->loadFile(xml_file);
	//if (loaded) 
	//{
	//	////Con::errorf("loaded xml file!!!!!");
	//	doc->pushFirstChildElement("bvhCfgData");
	//	doc->pushFirstChildElement("SampleRate");
	//	//int sampleRate = strtol(doc->getData());

	//	doc->nextSiblingElement("Scale");
	//	if (strstr(doc->getData(),"INCHES"))
	//		cfg->bvhScale = 1.0 / 39.0;
	//	else
	//		cfg->bvhScale = 1.0;

	//	while(doc->nextSiblingElement("Node"))
	//	{
	//		doc->pushFirstChildElement("ModelNodeName");
	//		cfg->dtsNodes[nodeCount] = kShape->findNode(doc->getData());
	//		doc->nextSiblingElement("BvhNodeName");
	//		cfg->bvhNames[nodeCount] = doc->getData();
	//		//cfg->bvhNodes[nodeCount] = ?;
	//		doc->nextSiblingElement("NodeGroup");
	//		cfg->nodeGroups[nodeCount] = strtol(doc->getData());
	//		////Con::errorf("NodeGroup: %d",strtol(doc->getData()));
	//		doc->nextSiblingElement("bvhPoseRotA");
	//		sscanf(doc->getData(),"%f %f %f",&x,&y,&z);
	//		cfg->bvhPoseRotsA[nodeCount] = Ogre::Vector3(mDegToRad(x),mDegToRad(y),mDegToRad(z));
	//		////Con::errorf("bvhPoseRotA: %f %f %f",x,y,z);
	//		doc->nextSiblingElement("bvhPoseRotB");
	//		sscanf(doc->getData(),"%f %f %f",&x,&y,&z);
	//		cfg->bvhPoseRotsB[nodeCount] = Ogre::Vector3(mDegToRad(x),mDegToRad(y),mDegToRad(z));
	//		////Con::errorf("bvhPoseRotB: %s",doc->getData());
	//		doc->nextSiblingElement("axesFixRotA");
	//		sscanf(doc->getData(),"%f %f %f",&x,&y,&z);
	//		cfg->axesFixRotsA[nodeCount] = Ogre::Vector3(mDegToRad(x),mDegToRad(y),mDegToRad(z));
	//		////Con::errorf("axesFixRotA: %s",doc->getData());
	//		doc->nextSiblingElement("axesFixRotB");
	//		sscanf(doc->getData(),"%f %f %f",&x,&y,&z);
	//		cfg->axesFixRotsB[nodeCount] = Ogre::Vector3(mDegToRad(x),mDegToRad(y),mDegToRad(z));
	//		////Con::errorf("axesFixRotB: %s",doc->getData());
	//		nodeCount++;
	//		doc->popElement();
	//	}
	//} else //Con::errorf("Failed to load bvh cfg xml file: %s",xml_file);

	//cfg->numBvhNodes = nodeCount;
	//cfg->numDtsNodes = nodeCount;//Do I still need both of these?

	//doc->deleteObject();

	//return nodeCount;//success;
	return 0;
}

void fxFlexBody::saveBvhCfgXml(const char *xml_file,bvhCfgData *cfg)
{
	//char temp[255];
	//TSShape *kShape = getShapeInstance()->getShape();
	//SimXMLDocument *doc = new SimXMLDocument();
	//doc->registerObject();
	//doc->addHeader();

	//doc->pushNewElement("bvhCfgData");
	//doc->setAttribute("name","ACK_TrueBones");
	//doc->pushNewElement("SampleRate");
	//doc->addData("1");
	//doc->addNewElement("Scale");
	//doc->addData("INCHES");

	//for (unsigned int i=0;i<cfg->numDtsNodes;i++) 
	//{
	//	doc->addNewElement("Node");

	//	doc->pushNewElement("ModelNodeName");
	//	doc->addData(kShape->names[kShape->nodes[cfg->dtsNodes[i]].nameIndex].c_str());
	//	//doc->addData(mBodyParts[]);
	//	doc->addNewElement("BvhNodeName");
	//	doc->addData(cfg->bvhNames[i].c_str());
	//	doc->addNewElement("NodeGroup");
	//	sprintf(temp,"%d",cfg->nodeGroups[i]);
	//	doc->addData(temp);
	//	doc->addNewElement("bvhPoseRotA");
	//	sprintf(temp,"%3.2f %3.2f %3.2f",cfg->bvhPoseRotsA[i].x,cfg->bvhPoseRotsA[i].y,cfg->bvhPoseRotsA[i].z);
	//	doc->addData(temp);
	//	doc->addNewElement("bvhPoseRotB");
	//	sprintf(temp,"%3.2f %3.2f %3.2f",cfg->bvhPoseRotsB[i].x,cfg->bvhPoseRotsB[i].y,cfg->bvhPoseRotsB[i].z);
	//	doc->addData(temp);
	//	doc->addNewElement("axesFixRotA");
	//	sprintf(temp,"%3.2f %3.2f %3.2f",cfg->axesFixRotsA[i].x,cfg->axesFixRotsA[i].y,cfg->axesFixRotsA[i].z);
	//	doc->addData(temp);
	//	doc->addNewElement("axesFixRotB");
	//	sprintf(temp,"%3.2f %3.2f %3.2f",cfg->axesFixRotsB[i].x,cfg->axesFixRotsB[i].y,cfg->axesFixRotsB[i].z);
	//	doc->addData(temp);

	//	doc->popElement();
	//}
	//bool saved = doc->saveFile(xml_file);
	//doc->deleteObject();
}

//void fxFlexBody::addPersonaSequence()
//{
//}
//
//void fxFlexBody::dropPersonaSequence()
//{
//}
//
//void fxFlexBody::addPersonaSequenceEvent()
//{
//}
//
//void fxFlexBody::dropPersonaSequenceEvent()
//{
//}

unsigned int fxFlexBody::loadBvhSkeletonDB(bvhCfgData *cfg,unsigned int profile_id)
{
	unsigned int jc=0;
	//char select_query[512];

	//SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(mPM)->getSQL();
	//if (!sql) return 0;
	//if (!profile_id) return 0;
	//if (sql->OpenDatabase("EcstasyMotion.db"))
	//{
	//	char id_query[512],insert_query[512];
	//	int result;
	//	sqlite_resultset *resultSet;
	//	if (profile_id)
	//	{

	//		sprintf(select_query,"SELECT name,parent_id,\
	//							 offset_x,offset_y,offset_z,channels, \
	//							 channelRots_0,channelRots_1,channelRots_2 \
	//						 FROM bvhProfileJoint WHERE bvh_profile_id = %d;",profile_id);
	//		result = sql->ExecuteSQL(select_query);
	//		resultSet = sql->GetResultSet(result);
	//		jc = resultSet->iNumRows;
	//		for (unsigned int i=0;i<resultSet->iNumRows;i++)
	//		{
	//			sprintf(cfg->joints[i].name,"%s",resultSet->vRows[i]->vColumnValues[0]);
	//			cfg->joints[i].parent = strtol(resultSet->vRows[i]->vColumnValues[1]);
	//			cfg->joints[i].offset.x = strtod(resultSet->vRows[i]->vColumnValues[2]);
	//			cfg->joints[i].offset.y = strtod(resultSet->vRows[i]->vColumnValues[3]);
	//			cfg->joints[i].offset.z = strtod(resultSet->vRows[i]->vColumnValues[4]);
	//			cfg->joints[i].channels = strtol(resultSet->vRows[i]->vColumnValues[5]);
	//			cfg->joints[i].chanrots[0] = strtol(resultSet->vRows[i]->vColumnValues[6]);
	//			cfg->joints[i].chanrots[1] = strtol(resultSet->vRows[i]->vColumnValues[7]);
	//			cfg->joints[i].chanrots[2] = strtol(resultSet->vRows[i]->vColumnValues[8]);
	//			//Con::printf("Joint %d offset: (%f, %f, %f)",i,cfg->joints[i].offset.x,cfg->joints[i].offset.y,cfg->joints[i].offset.z);
	//		}
	//	}
	//	sql->CloseDatabase();
	//	delete sql;
	//}
	//
	return jc;
}

unsigned int fxFlexBody::loadBvhSkeleton(FILE *fp,bvhCfgData *cfg,unsigned int profile_id)
{
	unsigned int jc,currBvhNode,numChannels,jloop;
	jc = jloop = 0;
	//int newParent = -1;
	//char name[40], tempc[40];
	//char chan1[10], chan2[10], chan3[10];
	//char buf[2500];
	//char *bufp;
	//bool keepGoing,isRelevant,loadingJoints;

	//Ogre::Vector3 p,r;
	//Ogre::Vector3 pos3[MAX_BVH_NODES];
	//Ogre::Vector3 rot3[MAX_BVH_NODES];

	////Load BVH
	//fgets(buf,250,fp); // HIERARCHY
	//fgets(buf,250,fp); // ROOT
	//sscanf(buf,"%s %s",&tempc,&name);
	//if (cfg->usingNames)
	//{
	//	if (!strcmp(name,cfg->bvhNames[0].c_str()))
	//		cfg->bvhNodes[0] = 0;//Still necessary if we use node indices in the cfg.
	//	currBvhNode = 1;//Use this to track our next possible active bvh node index.	
	//}
	//
	//cfg->joints[0].parent = -1;
	//strcpy(cfg->joints[0].name,name);
	//fgets(buf,250,fp); // {
	//fgets(buf,250,fp);// OFFSET x y z
	//sscanf(buf,"  OFFSET %f %f %f",&p.x,&p.y,&p.z);
	//cfg->joints[0].offset = p;
	//fgets(buf,250,fp);// CHANNELS n ...
	//sscanf(buf,"  CHANNELS %d",&numChannels);

	//if (numChannels==6)
	//	sscanf(buf,"  CHANNELS %d %s %s %s %s %s %s",&numChannels,&tempc,&tempc,&tempc,&chan1,&chan2,&chan3);
	//else if (numChannels==3)
	//	sscanf(buf,"  CHANNELS %d %s %s %s",&numChannels,&chan1,&chan2,&chan3);

	//cfg->joints[0].channels = numChannels;

	////Gotta sort out what order the rotations come in, PER NODE.
	//if (chan1[0]=='X') cfg->joints[0].chanrots[0] = 0;
	//else if (chan1[0]=='Y') cfg->joints[0].chanrots[0] = 1;
	//else if (chan1[0]=='Z') cfg->joints[0].chanrots[0] = 2;
	//if (chan2[0]=='X') cfg->joints[0].chanrots[1] = 0;
	//else if (chan2[0]=='Y') cfg->joints[0].chanrots[1] = 1;
	//else if (chan2[0]=='Z') cfg->joints[0].chanrots[1] = 2;
	//if (chan3[0]=='X') cfg->joints[0].chanrots[2] = 0;
	//else if (chan3[0]=='Y') cfg->joints[0].chanrots[2] = 1;
	//else if (chan3[0]=='Z') cfg->joints[0].chanrots[2] = 2;

	//////Con::printf("JOINT %d: %s chanrots %d %d %d",0,joints[0].name,joints[0].chanrots[0],joints[0].chanrots[1],joints[0].chanrots[2]);
	//fgets(buf,250,fp); 
	//bufp = strtok(buf," \t\n");

	//loadingJoints = true;
	//while (loadingJoints)
	//{
	//	keepGoing = true;
	//	while (keepGoing)
	//	{
	//		char tempOne[40];
	//		sprintf(tempOne,"JOINT");
	//		if (!strstr(bufp,tempOne)) 
	//		{ 
	//			keepGoing = false; //End Site
	//			fgets(buf,250,fp);//{
	//			fgets(buf,250,fp);//terminal OFFSET x y z, DON'T ignore it
	//			bufp = strtok(buf," \t\n");//"OFFSET"
	//			float X,Y,Z;
	//			bufp = strtok(NULL," \t\n"); sscanf(bufp,"%f",&X);
	//			bufp = strtok(NULL, " \t\n"); sscanf(bufp,"%f",&Y);
	//			bufp = strtok(NULL, " \t\n"); sscanf(bufp,"%f",&Z);
	//			cfg->endSiteOffsets[cfg->nodeGroups[jc]].set(X,Y,Z);
	//			fgets(buf,250,fp);//}
	//			break; 
	//		}
	//		jc++;
	//		if (newParent>=0) {
	//			cfg->joints[jc].parent = newParent;
	//			newParent = -1;
	//		} else cfg->joints[jc].parent = jc-1;

	//		bufp = strtok(NULL," \t\n");
	//		sscanf(bufp,"%s",name);				
	//		strcpy(cfg->joints[jc].name,name);
	//		if (cfg->usingNames)
	//		{
	//			if (!strcmp(name,cfg->bvhNames[currBvhNode]))
	//			{
	//				cfg->bvhNodes[currBvhNode] = jc;
	//				currBvhNode++;
	//			}
	//		}
	//		fgets(buf,250,fp);//{
	//		fgets(buf,250,fp);//OFFSET x y z
	//		bufp = strtok(buf," \t\n");//"OFFSET"
	//		bufp = strtok(NULL," \t\n"); sscanf(bufp,"%f",&(cfg->joints[jc]).offset.x);
	//		bufp = strtok(NULL, " \t\n"); sscanf(bufp,"%f",&(cfg->joints[jc]).offset.y);
	//		bufp = strtok(NULL, " \t\n"); sscanf(bufp,"%f",&(cfg->joints[jc]).offset.z);

	//		fgets(buf,250,fp);//CHANNELS n s s s s s s
	//		bufp = strtok(buf," \t\n");//"CHANNELS"
	//		bufp = strtok(NULL," \t\n"); sscanf(bufp,"%d",&(cfg->joints[jc]).channels);
	//		if (cfg->joints[jc].channels==6) {
	//			bufp = strtok(NULL," \t\n"); 
	//			bufp = strtok(NULL," \t\n"); 
	//			bufp = strtok(NULL," \t\n"); 
	//		}
	//		bufp = strtok(NULL," \t\n"); sscanf(bufp,"%s",&chan1);
	//		bufp = strtok(NULL," \t\n"); sscanf(bufp,"%s",&chan2);
	//		bufp = strtok(NULL," \t\n"); sscanf(bufp,"%s",&chan3);
	//		//This is ugly but it's not worth rewriting chan1 etc. as 2d array.
	//		//Gotta sort out what order the rotations come in, PER NODE.
	//		if (chan1[0]=='X') cfg->joints[jc].chanrots[0] = 0;
	//		else if (chan1[0]=='Y') cfg->joints[jc].chanrots[0] = 1;
	//		else if (chan1[0]=='Z') cfg->joints[jc].chanrots[0] = 2; 
	//		if (chan2[0]=='X') cfg->joints[jc].chanrots[1] = 0;
	//		else if (chan2[0]=='Y') cfg->joints[jc].chanrots[1] = 1;
	//		else if (chan2[0]=='Z') cfg->joints[jc].chanrots[1] = 2;
	//		if (chan3[0]=='X') cfg->joints[jc].chanrots[2] = 0;
	//		else if (chan3[0]=='Y') cfg->joints[jc].chanrots[2] = 1;
	//		else if (chan3[0]=='Z') cfg->joints[jc].chanrots[2] = 2;

	//		if (!fgets(buf,250,fp)) break;
	//		bufp = strtok(buf," \t\n");
	//	}
	//	//fgets(buf,250,fp);// JOINT?

	//	//HERE: back out of the nested curly braces, checking for new JOINTs and decrementing jloop each time.

	//	keepGoing = true;
	//	newParent = jc;//cfg->joints[jc].parent;
	//	while (keepGoing)
	//	{
	//		if (!fgets(buf,250,fp)) 
	//		{
	//			keepGoing = false;
	//			loadingJoints = false;
	//			break;
	//		}
	//		
	//		bufp = strtok(buf," \n\t");
	//		char tempOne[40],tempTwo[40],tempThree[40];
	//		sprintf(tempOne,"}");
	//		sprintf(tempTwo,"JOINT");
	//		sprintf(tempThree,"MOTION");
	//		if (strstr(bufp,tempOne))
	//		{
	//			//jloop++;
	//			newParent = cfg->joints[newParent].parent;
	//		} else if (strstr(bufp,tempTwo)) {
	//			keepGoing = false;
	//		} else if (strstr(bufp,tempThree)) {
	//			//Con::printf("BVH -- loaded schema, proceeding to motion frames.");
	//			keepGoing = false;
	//			loadingJoints = false;
	//		} else {
	//			//Con::errorf("BVH -- problem, found no frames.");
	//			fclose(fp);
	//			return jc;
	//		}
	//	}
	//}
	////NOW: add all this to the database, if we have no joints for this profile
	//SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(mPM)->getSQL();
	//if (!sql) return 0;
	//if (!sql->OpenDatabase("EcstasyMotion.db")) { delete sql; return 0; }
	//
	//char id_query[512],insert_query[512];
	//int result;
	//sqlite_resultset *resultSet;

	//sprintf(id_query,"SELECT id FROM bvhProfileJoint WHERE bvh_profile_id = %d;",profile_id);
	//result = sql->ExecuteSQL(id_query);
	//resultSet = sql->GetResultSet(result);
	//if (resultSet->iNumRows == 0)//For now, we are loading the skeleton from the actual bvh file always, instead of
	//{//from the database.  Later it might be helpful to let people make adjustments to their bvh skeleton for some reason
	//	//Con::printf("loadBvhSkeleton: loading %d joints into the database, profile_id %d",jc,profile_id);
	//	for (unsigned int i=0;i<=jc;i++)//and save it to the DB, and then read from that for use in-program and for saving out bvhs.
	//	{
	//		sprintf(insert_query,"INSERT INTO bvhProfileJoint (bvh_profile_id,parent_id,name,offset_x,offset_y,offset_z,channels,channelRots_0,channelRots_1,channelRots_2) VALUES (%d,%d,'%s',%f,%f,%f,%d,%d,%d,%d);",
	//			profile_id,cfg->joints[i].parent,cfg->joints[i].name,cfg->joints[i].offset.x,cfg->joints[i].offset.y,cfg->joints[i].offset.z,cfg->joints[i].channels,cfg->joints[i].chanrots[0],cfg->joints[i].chanrots[1],cfg->joints[i].chanrots[2]);
	//		//FIX: parent will be wrong, need parent_id from DB stored earlier
	//		result = sql->ExecuteSQL(insert_query);
	//	}
	//}
	//sql->CloseDatabase();

	return jc;
}

// EulerAngles code ///////////////////
//HMMMM... unresolved externals, can't include the .c files like I'd think I could... 
//sigh, sticking what I need in here for the moment.

//EulerAngles.c
//
//EulerAngles Eul_(float ai, float aj, float ah, int order)
//{
//    EulerAngles ea;
//    ea.x = ai; ea.y = aj; ea.z = ah;
//    ea.w = order;
//    return (ea);
//}
///* Construct quaternion from Euler angles (in radians). */
//Quat Eul_ToQuat(EulerAngles ea)
//{
//    Quat qu;
//    double a[3], ti, tj, th, ci, cj, ch, si, sj, sh, cc, cs, sc, ss;
//    int i,j,k,h,n,s,f;
//    EulGetOrd(ea.w,i,j,k,h,n,s,f);
//    if (f==EulFrmR) {float t = ea.x; ea.x = ea.z; ea.z = t;}
//    if (n==EulParOdd) ea.y = -ea.y;
//    ti = ea.x*0.5; tj = ea.y*0.5; th = ea.z*0.5;
//    ci = cos(ti);  cj = cos(tj);  ch = cos(th);
//    si = sin(ti);  sj = sin(tj);  sh = sin(th);
//    cc = ci*ch; cs = ci*sh; sc = si*ch; ss = si*sh;
//    if (s==EulRepYes) {
//	a[i] = cj*(cs + sc);	/* Could speed up with */
//	a[j] = sj*(cc + ss);	/* trig identities. */
//	a[k] = sj*(cs - sc);
//	qu.w = cj*(cc - ss);
//    } else {
//	a[i] = cj*sc - sj*cs;
//	a[j] = cj*ss + sj*cc;
//	a[k] = cj*cs - sj*sc;
//	qu.w = cj*cc + sj*ss;
//    }
//    if (n==EulParOdd) a[j] = -a[j];
//    qu.x = a[X]; qu.y = a[Y]; qu.z = a[Z];
//    return (qu);
//}
//
///* Construct matrix from Euler angles (in radians). */
//void Eul_ToHMatrix(EulerAngles ea, HMatrix M)
//{
//    double ti, tj, th, ci, cj, ch, si, sj, sh, cc, cs, sc, ss;
//    int i,j,k,h,n,s,f;
//    EulGetOrd(ea.w,i,j,k,h,n,s,f);
//    if (f==EulFrmR) {float t = ea.x; ea.x = ea.z; ea.z = t;}
//    if (n==EulParOdd) {ea.x = -ea.x; ea.y = -ea.y; ea.z = -ea.z;}
//    ti = ea.x;	  tj = ea.y;	th = ea.z;
//    ci = cos(ti); cj = cos(tj); ch = cos(th);
//    si = sin(ti); sj = sin(tj); sh = sin(th);
//    cc = ci*ch; cs = ci*sh; sc = si*ch; ss = si*sh;
//    if (s==EulRepYes) {
//	M[i][i] = cj;	  M[i][j] =  sj*si;    M[i][k] =  sj*ci;
//	M[j][i] = sj*sh;  M[j][j] = -cj*ss+cc; M[j][k] = -cj*cs-sc;
//	M[k][i] = -sj*ch; M[k][j] =  cj*sc+cs; M[k][k] =  cj*cc-ss;
//    } else {
//	M[i][i] = cj*ch; M[i][j] = sj*sc-cs; M[i][k] = sj*cc+ss;
//	M[j][i] = cj*sh; M[j][j] = sj*ss+cc; M[j][k] = sj*cs-sc;
//	M[k][i] = -sj;	 M[k][j] = cj*si;    M[k][k] = cj*ci;
//    }
//    M[W][X]=M[W][Y]=M[W][Z]=M[X][W]=M[Y][W]=M[Z][W]=0.0; M[W][W]=1.0;
//}
//
///* Convert matrix to Euler angles (in radians). */
//EulerAngles Eul_FromHMatrix(HMatrix M, int order)
//{
//    EulerAngles ea;
//    int i,j,k,h,n,s,f;
//    EulGetOrd(order,i,j,k,h,n,s,f);
//    if (s==EulRepYes) {
//	double sy = sqrt(M[i][j]*M[i][j] + M[i][k]*M[i][k]);
//	if (sy > 16*FLT_EPSILON) {
//	    ea.x = atan2((float)M[i][j], (float)M[i][k]);
//	    ea.y = atan2((float)sy, (float)M[i][i]);
//	    ea.z = atan2((float)M[j][i], (float)-M[k][i]);
//	} else {
//	    ea.x = atan2((float)-M[j][k], (float)M[j][j]);
//	    ea.y = atan2((float)sy, (float)M[i][i]);
//	    ea.z = 0;
//	}
//    } else {
//	double cy = sqrt(M[i][i]*M[i][i] + M[j][i]*M[j][i]);
//	if (cy > 16*FLT_EPSILON) {
//	    ea.x = atan2((float)M[k][j], (float)M[k][k]);
//	    ea.y = atan2((float)-M[k][i], (float)cy);
//	    ea.z = atan2((float)M[j][i], (float)M[i][i]);
//	} else {
//	    ea.x = atan2((float)-M[j][k], (float)M[j][j]);
//	    ea.y = atan2((float)-M[k][i], (float)cy);
//	    ea.z = 0;
//	}
//    }
//    if (n==EulParOdd) {ea.x = -ea.x; ea.y = - ea.y; ea.z = -ea.z;}
//    if (f==EulFrmR) {float t = ea.x; ea.x = ea.z; ea.z = t;}
//    ea.w = order;
//    return (ea);
//}
//
///* Convert quaternion to Euler angles (in radians). */
//EulerAngles Eul_FromQuat(Quat q, int order)
//{
//    HMatrix M;
//    double Nq = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
//    double s = (Nq > 0.0) ? (2.0 / Nq) : 0.0;
//    double xs = q.x*s,	  ys = q.y*s,	 zs = q.z*s;
//    double wx = q.w*xs,	  wy = q.w*ys,	 wz = q.w*zs;
//    double xx = q.x*xs,	  xy = q.x*ys,	 xz = q.x*zs;
//    double yy = q.y*ys,	  yz = q.y*zs,	 zz = q.z*zs;
//    M[X][X] = 1.0 - (yy + zz); M[X][Y] = xy - wz; M[X][Z] = xz + wy;
//    M[Y][X] = xy + wz; M[Y][Y] = 1.0 - (xx + zz); M[Y][Z] = yz - wx;
//    M[Z][X] = xz - wy; M[Z][Y] = yz + wx; M[Z][Z] = 1.0 - (xx + yy);
//    M[W][X]=M[W][Y]=M[W][Z]=M[X][W]=M[Y][W]=M[Z][W]=0.0; M[W][W]=1.0;
//    return (Eul_FromHMatrix(M, order));
//}
///////// End EulerAngles code //////////////


void fxFlexBody::importBvh(bool importGround,const char *bvhFile,const char *bvhProfile)
{
	//unsigned int pc,rc,jc,jloop,currBvhNode,profile_id;
	//pc = 0; rc = 0; jc = 0; jloop = 0;
	//unsigned int sampleRate = 1;//3
	//int newParent = -1;
	//int numTrans = 0;
	//int numFrames = 0;
	//int numSamples = 0;
	//float frameTime = 0.0;
	//Ogre::Quaternion rot,q;
	//Quat16 q16;
	//Ogre::Quaternion rots[30000];//FIX - make variable! 
	//unsigned int numRots = 0; 							
	//Ogre::Vector3 trans[2000];
	//Ogre::Vector3 p,r;

	//Ogre::Vector3 pos3[MAX_BVH_NODES];
	//Ogre::Vector3 rot3[MAX_BVH_NODES];
	////bvhJoint joints[MAX_BVH_NODES];
	////int bvhNodes[MAX_BVH_NODES];
	////int dtsNodes[MAX_BVH_NODES];
	////int sortNodes[MAX_BVH_NODES];
	////int orderNodes[MAX_BVH_NODES];//from initial order to shape order
	////Ogre::Vector3 bvhPoseRotsA[MAX_BVH_NODES],bvhPoseRotsB[MAX_BVH_NODES];//allow for two later, if we need it
	////Ogre::Vector3 axesFixRotsA[MAX_BVH_NODES],axesFixRotsB[MAX_BVH_NODES];
	////String bvhNames[MAX_BVH_NODES];

	//bool keepGoing,isRelevant,loadingJoints,loadingDB;
	//char buf[2500],scaleUnits[10],bvh_name[255];
	//char *bufp;
	//bvhCfgData kCfg;
	////bvhCfgData kCfg2;
	//FILE *fp = NULL;
	//FILE *fpc = NULL;
	//String seqDir,seqName,dsqFile;
	//	
	////The new, 1.8 way to do it:
	//const String myPath = getShapeInstance()->mShapeResource.getPath().getPath();
	//const String myFileName = getShapeInstance()->mShapeResource.getPath().getFileName();
	//
	//TSShape *kShape = getShapeInstance()->getShape();

	//
	//seqDir = bvhFile;
	//unsigned int nameLength = strlen(strrchr(seqDir.c_str(),'/'))-1;
	//seqName = seqDir;
	//seqName.erase(0,seqDir.length()-nameLength);
	//seqDir.erase(seqDir.length()-nameLength,nameLength);

	//seqName.replace(' ','_');
	//seqName.replace(".BVH","");
	//seqName.replace(".bvh","");//Another way to do it...

	//fp = fopen(bvhFile,"r");
	//if (fp==NULL) 
	//{
	//	//Con::errorf("ERROR: can't open bvh file: %s",bvhFile);
	//	return;
	//} else //Con::errorf("opened bvhFile: %s",bvhFile);

	////HERE: check in the database for any bvhProfileNodes attached to this bvhProfile.  If there are none, then this 
	////is a new profile, so look for a default.cfg to load the data from.  Otherwise, load it from the DB and go on.
	//SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(mPM)->getSQL();
	//if (!sql) return;
	//if (sql->OpenDatabase("EcstasyMotion.db"))
	//{
	//	char id_query[512],insert_query[512];
	//	int result;
	//	sqlite_resultset *resultSet;
	//	sprintf(id_query,"SELECT id FROM bvhProfile WHERE name = '%s';",bvhProfile);
	//	result = sql->ExecuteSQL(id_query);
	//	resultSet = sql->GetResultSet(result);
	//	if (resultSet->iNumRows == 1)
	//		profile_id = strtol(resultSet->vRows[0]->vColumnValues[0]);

	//	loadingDB = false;
	//	if (profile_id)
	//	{
	//		sprintf(id_query,"SELECT id FROM bvhProfileNode WHERE bvh_profile_id = %d;",profile_id);
	//		result = sql->ExecuteSQL(id_query);
	//		resultSet = sql->GetResultSet(result);
	//		if (resultSet->iNumRows > 0)
	//			loadingDB = true;
	//	}
	//	sql->CloseDatabase();
	//	delete sql;
	//}
	////HERE: strip all but the filename out of the chosen path, then add default.cfg to that.  
	//
	//if (loadingDB) 
	//{
	//	rc = loadBvhCfgDB(&kCfg,profile_id);

	//} else {

	//	String bvhExt;
	//	if (strstr(bvhFile,".cyclic."))
	//		bvhExt.insert(0,".cyclic.bvh");
	//	else 
	//		bvhExt.insert(0,".bvh");

	//	//seqDir.ToLower();
	//	String bvhExtUpp;
	//	bvhExtUpp = String::ToUpper(bvhExt);
	//	//if (seqDir.find(bvhExt) > -1) seqDir.erase(seqDir.length()-bvhExt.length(),bvhExt.length());
	//	if ( (strstr((const char *)seqDir.c_str(),(const char *)bvhExt.c_str())) ||
	//		(strstr((const char *)seqDir.c_str(),(const char *)bvhExtUpp.c_str())) )
	//		seqDir.erase(seqDir.length()-bvhExt.length(),bvhExt.length());



	//	String configPath(seqDir);
	//	String configName("default.cfg");
	//	configPath += configName;

	//	fpc = fopen(configPath.c_str(),"r");

	//	if (fpc==NULL)
	//	{
	//		//Con::errorf("ERROR: can't open config file");
	//		return;
	//	}
	//	//void loadBvhCfg(FILE *,int *,int *,int *,int *,Ogre::Vector3 *,Ogre::Vector3 *,Ogre::Vector3 *,Ogre::Vector3 *);
	//	//rc = loadBvhCfg(fpc,bvhNodes,dtsNodes,sortNodes,orderNodes,bvhPoseRotsA,bvhPoseRotsB,
	//	//	axesFixRotsA,axesFixRotsB,bvhNames);

	//	rc = loadBvhCfg(fpc,&kCfg,profile_id);
	//	
	//	fclose(fpc);
	//}
	////unsigned int tc = loadBvhCfgXml("art/shapes/bvhCfg/ACK_TrueBones.cfg",&kCfg2);

	//jc = loadBvhSkeleton(fp,&kCfg,profile_id);

	////Now, with skeleton loaded, go back and figure out the right bvh nodes based on names from cfg.
	//for (unsigned int i=0;i<rc;i++)
	//	for (unsigned int j=0;j<jc;j++)
	//		if (!strcmp(kCfg.bvhNames[i],kCfg.joints[jc].name))
	//			kCfg.bvhNodes[i] = j;

	//	


	//////Con::errorf("------------------bvhScale = %f, xml bvh scale %f, numNodes %d ---------------",kCfg.bvhScale,kCfg2.bvhScale,kCfg2.numDtsNodes);


	////<snip> whole bunch of code removed, moved to loadBvhSkeleton above.</snip>

	////unsigned int numJoints = jc+1;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	////NOW: all done loading joints.  Array is filled up and available.  Next step is loading all the motion
	////frames, after picking up the numFrames and frameLength variables.

	//if (kCfg.bvhNodes[rc-1]>jc) {//First, sanity check.
	//			//Con::errorf("BVH -- problem, nodes referenced in cfg that are not in bvh file.");
	//			fclose(fp);
	//			return;
	//}
	//float pr[3];
	//
	//fgets(buf,250,fp);//Frames: n
	//bufp = strtok(buf," \n\t");//"Frames:"
	//bufp = strtok(NULL," \t\n");
	//sscanf(bufp,"%d",&numFrames);

	//fgets(buf,250,fp);//Frame Time: f
	//bufp = strtok(buf," \n\t");//"Frame"
	//bufp = strtok(NULL," \t\n");//"Time:"
	//bufp = strtok(NULL," \t\n");
	//sscanf(bufp,"%f",&frameTime);

	////Con::printf("Frames %d, time %f",numFrames,frameTime);

	//if (!numFrames || (numFrames<=0)) 
	//{
	//	//Con::errorf("BVH -- couldn't read numFrames, bailing.");
	//	fclose(fp);
	//	return;
	//}

	//for (unsigned int c=0;c<numFrames;c++)
	//{
	//	if (!fgets(buf,3500,fp)) 
	//	{
	//		//Con::errorf("BVH -- not enough frames, bailing out.");
	//		fclose(fp);
	//		return;
	//	}

	//	pc=0;
	//	//for (unsigned int d=0; d < numJoints;d++)
	//	for (unsigned int d=0; d < jc; d++)//rc
	//	{
	//		if (d==0) 
	//			bufp = strtok(buf," \t\n");
	//		else 
	//			bufp = strtok(NULL, " \t\n");

	//		if (kCfg.joints[d].channels == 3)
	//		{
	//			sscanf(bufp,"%f",&pr[0]);
	//			bufp = strtok(NULL, " \t\n");
	//			sscanf(bufp,"%f",&pr[1]);
	//			bufp = strtok(NULL, " \t\n");
	//			sscanf(bufp,"%f",&pr[2]);

	//			isRelevant = 0;
	//			//HERE: only do this if j is one of the relevant nodes.
	//			for (unsigned int k=0;k<rc;k++) {
	//				if (kCfg.bvhNodes[k]==d) isRelevant = 1;
	//			}
	//			if (isRelevant) 
	//			{
	//				if (kCfg.joints[d].chanrots[0]==0) p.x = pr[0];
	//				else if (kCfg.joints[d].chanrots[0]==1) p.y = pr[0];
	//				else if (kCfg.joints[d].chanrots[0]==2) p.z = pr[0];
	//				if (kCfg.joints[d].chanrots[1]==0) p.x = pr[1];
	//				else if (kCfg.joints[d].chanrots[1]==1) p.y = pr[1];
	//				else if (kCfg.joints[d].chanrots[1]==2) p.z = pr[1];
	//				if (kCfg.joints[d].chanrots[2]==0) p.x = pr[2];
	//				else if (kCfg.joints[d].chanrots[2]==1) p.y = pr[2];
	//				else if (kCfg.joints[d].chanrots[2]==2) p.z = pr[2];
	//				rot3[pc++] = p;
	//			}
	//		}
	//		else
	//		{//ignore isRelevant, assuming this will only happen on root node, do cleanup otherwise.
	//			sscanf(bufp,"%f",&p.x);
	//			bufp = strtok(NULL, " \t\n");
	//			sscanf(bufp,"%f",&p.y);
	//			bufp = strtok(NULL, " \t\n");
	//			sscanf(bufp,"%f",&p.z);
	//			pos3[pc] = p;

	//			//HERE: note that this is loading z,x,y order, bvh rule ... ??
	//			bufp = strtok(NULL, " \t\n");
	//			sscanf(bufp,"%f",&pr[0]);
	//			bufp = strtok(NULL, " \t\n");
	//			sscanf(bufp,"%f",&pr[1]);
	//			bufp = strtok(NULL, " \t\n");
	//			sscanf(bufp,"%f",&pr[2]);
	//			if (kCfg.joints[d].chanrots[0]==0) p.x = pr[0];
	//			else if (kCfg.joints[d].chanrots[0]==1) p.y = pr[0];
	//			else if (kCfg.joints[d].chanrots[0]==2) p.z = pr[0];
	//			if (kCfg.joints[d].chanrots[1]==0) p.x = pr[1];
	//			else if (kCfg.joints[d].chanrots[1]==1) p.y = pr[1];
	//			else if (kCfg.joints[d].chanrots[1]==2) p.z = pr[1];
	//			if (kCfg.joints[d].chanrots[2]==0) p.x = pr[2];
	//			else if (kCfg.joints[d].chanrots[2]==1) p.y = pr[2];
	//			else if (kCfg.joints[d].chanrots[2]==2) p.z = pr[2];
	//			rot3[pc++] = p;
	//		}
	//	}
	//	//if (c % sampleRate == 0)
	//	if (c % mImportSampleRate == 0)
	//	{
	//		numSamples++;
	//		Ogre::Matrix3 m,matX,matY,matZ,mat1,mat2,mat3,matDef,matBvhPose,matBvhPoseA,matBvhPoseB;
	//		Ogre::Matrix3 matAxesFix,matAxesFixA,matAxesFixB;
	//		Ogre::Matrix3 matAxesUnfix,matWorldFix,matWorldUnfix;
	//		Ogre::Vector3 eulX,eulY,eulZ,eulFix,eulWorldFix;

	//		//ASSUMPTION: only root node has translation data.  
	//		trans[numTrans].x = -pos3[0].x;//do some sign flipping and coord swapping to 
	//		trans[numTrans].y = pos3[0].z;//get from right-handed to left-handed system.
	//		trans[numTrans].z = pos3[0].y;

	//		//And then deal with the scale difference between bvh translations and dts, this can be huge.
	//		//float transScale = kShape->defaultTranslations[0].z / pos3[0].y;
	//		////Con::errorf("transScale: %f",transScale);

	//		//OOH... source of weird base node translation errors!  When we're not in an upright position, this does NOT WORK.  Need 
	//		//to get one multiplier from abdomen height in a t-pose, or else find some other way to measure it, or do something else entirely.


	//		trans[numTrans] *= kCfg.bvhScale;

	//		numTrans++;//Any reason not to just use numSamples here?

	//		//for (unsigned int j=0; j<numJoints;j++)
	//		for (unsigned int j=0; j<rc;j++)
	//		{
	//			//isRelevant = 0;
	//			//HERE: only do this if j is one of the relevant nodes.
	//			//for (unsigned int k=0;k<rc;k++) {
	//			//	if (bvhNodes[k]==j) isRelevant = 1;
	//			//}
	//			//if (isRelevant)
	//			//{
	//				matBvhPoseA.set(kCfg.bvhPoseRotsA[j]);
	//				matBvhPoseB.set(kCfg.bvhPoseRotsB[j]);
	//				matBvhPose.mul(matBvhPoseA,matBvhPoseB);

	//				matAxesFixA.set(kCfg.axesFixRotsA[j]);
	//				matAxesFixB.set(kCfg.axesFixRotsB[j]);
	//				matAxesFix.mul(matAxesFixA,matAxesFixB);

	//				matAxesUnfix = matAxesFix;
	//				matAxesUnfix.inverse();

	//				//Swap from lefthanded bvh to righthanded Torque.
	//				eulX.set(mDegToRad(rot3[j].x),0.0,0.0);
	//				matX.set(eulX);
	//				eulY.set(0.0,mDegToRad(-rot3[j].z),0.0);
	//				matY.set(eulY);
	//				eulZ.set(0.0,0.0,mDegToRad(-rot3[j].y));
	//				matZ.set(eulZ);

	//				m = Ogre::Quaternion::IDENTITY;
	//				m.mul(matBvhPose);
	//				m.mul(matAxesFix);

	//				//m.mul(matY);//(matZ);
	//				//m.mul(matX);
	//				//m.mul(matZ);//(matY);

	//				//Even though I've changed the names of the axes, I still need to do the rotations in the order they came.
	//				if (kCfg.joints[kCfg.bvhNodes[j]].chanrots[0]==0) mat1 = matX;
	//				else if (kCfg.joints[kCfg.bvhNodes[j]].chanrots[0]==1) mat1 = matZ;
	//				else if (kCfg.joints[kCfg.bvhNodes[j]].chanrots[0]==2) mat1 = matY;
	//				if (kCfg.joints[kCfg.bvhNodes[j]].chanrots[1]==0) mat2 = matX;
	//				else if (kCfg.joints[kCfg.bvhNodes[j]].chanrots[1]==1) mat2 = matZ;
	//				else if (kCfg.joints[kCfg.bvhNodes[j]].chanrots[1]==2) mat2 = matY;
	//				if (kCfg.joints[kCfg.bvhNodes[j]].chanrots[2]==0) mat3 = matX;
	//				else if (kCfg.joints[kCfg.bvhNodes[j]].chanrots[2]==1) mat3 = matZ;
	//				else if (kCfg.joints[kCfg.bvhNodes[j]].chanrots[2]==2) mat3 = matY;

	//				m.mul(mat1);
	//				Ogre::Vector3 eul1 = m.toEuler();
	//				m.mul(mat2);
	//				Ogre::Vector3 eul2 = m.toEuler();
	//				m.mul(mat3);
	//				Ogre::Vector3 eul3 = m.toEuler();

	//				m.mul(matAxesUnfix);

	//				//BLEND WAY:
	//				//q.set(m);

	//				//NON-BLEND WAY:
	//				q16 = kShape->defaultRotations[kCfg.dtsNodes[j]];
	//				q16.getOgre::Quaternion(&q);

	//				q.setMatrix(&matDef);
	//				matDef.mul(m);
	//				Ogre::Quaternion qFinal(matDef);
	//				rots[numRots++] = qFinal;
	//			//}
	//		}
	//	}
	//	

	//	//sprintf(rotation,40,"%3.2f %3.2f %3.2f ",pos3[0].x,pos3[0].y,pos3[0].z);
	//	//unsigned int len = strlen(rotation);
	//	//sprintf(line,40,rotation);
	//	//for (unsigned int j=0; j<numJoints;j++)
	//	//{
	//	//	sprintf(rotation,40,"%3.2f %3.2f %3.2f ",rot3[j].z,rot3[j].x,rot3[j].y);//HERE: 
	//	//	strcat(line,rotation);
	//	//}
	//}

	//fclose(fp);

	//
	////NOW, time to make a new sequence,
	////and add it to the stack!

	////for (unsigned int i=0;i<200;i++) dtsNodes[i] = -1;
	////... and the reverse mapping, just in case I need it.
	////for (unsigned int i=0;i<numJoints;i++) dtsNodes[dtsNodes[i]] = i;

	//
	////DANGER: can you do this while you have an existing shapeInstance?  Change your underlying mShape in realtime?
	////I bet not.  Maybe so, though.  
	////[Actually, seems to mostly work, although there are occasional crashes, don't know if this is why.]
	//kShape->sequences.increment();
	//TSShape::Sequence & seq = kShape->sequences.last();
	//constructInPlace(&seq);

	//seq.numKeyframes = numSamples;
	//seq.duration = (float)numFrames * frameTime;
	//seq.baseRotation = kShape->nodeRotations.size();
	//seq.baseTranslation = kShape->nodeTranslations.size();
	//seq.baseScale = 0;
	//seq.baseObjectState = 1;
	//seq.baseDecalState = 0;
	//seq.firstGroundFrame = kShape->groundTranslations.size();
	//if (importGround) seq.numGroundFrames = numSamples;
	//else seq.numGroundFrames = 0;//1;?
	//seq.firstTrigger = kShape->triggers.size();
	//seq.numTriggers = 0;
	//seq.toolBegin = 0.0;
	//seq.flags = TSShape::Cyclic;// | TSShape::Blend;// | TSShape::MakePath;
	//seq.priority = 5;

	////Con::errorf("New sequence!  numKeyframes %d, duration %f, baseRotation %d, baseTranslation %d",seq.numKeyframes,seq.duration,seq.baseRotation,seq.baseTranslation);
	//
	//seq.rotationMatters.clearAll();
	//for (unsigned int i=0;i<rc;i++) seq.rotationMatters.set(kCfg.dtsNodes[i]);//numJoints

	//seq.translationMatters.clearAll();
	//seq.translationMatters.set(0);//ASSUMPTION: only root node has position data

	//seq.scaleMatters.clearAll();
	//seq.visMatters.clearAll();
	//seq.frameMatters.clearAll();
	//seq.matFrameMatters.clearAll();
	////seq.decalMatters.clearAll();
	////seq.iflMatters.clearAll();

 //  kShape->names.increment();
 //  kShape->names.last() = StringTable->insert(seqName.c_str());
 //  seq.nameIndex = kShape->findName(seqName.c_str());

	//for (unsigned int i=0;i<numSamples;i++)
	//{
	//	//NOW, rotate this vector by the "fix" rotation of the base bodypart node.
	//	Ogre::Matrix3 matAxesFixA,matAxesFixB,matAxesFix;
	//	Ogre::Vector3 temp = trans[i];

	//	if ((kCfg.bvhPoseRotsA[0].isZero())&&(!kCfg.axesFixRotsA[0].isZero()))
	//	{//Hmm, needed this for Victoria (collada import) but it breaks Kork.  Making special case test here.
	//		matAxesFixA.set(kCfg.axesFixRotsA[0]);
	//		matAxesFixB.set(kCfg.axesFixRotsB[0]);
	//		matAxesFix.mul(matAxesFixA,matAxesFixB);
	//		matAxesFix.mulP(temp,&trans[i]);
	//	}
	//	if (importGround)
	//	{
	//		kShape->nodeTranslations.increment();
	//		kShape->nodeTranslations[kShape->nodeTranslations.size()-1].x = 0.0;
	//		kShape->nodeTranslations[kShape->nodeTranslations.size()-1].y = 0.0;
	//		kShape->nodeTranslations[kShape->nodeTranslations.size()-1].z = trans[i].z;

	//		kShape->groundRotations.increment();
	//		kShape->groundRotations[kShape->groundRotations.size()-1] = Ogre::Quaternion::IDENTITY;

	//		kShape->groundTranslations.increment();
	//		kShape->groundTranslations[kShape->groundTranslations.size()-1].x = trans[i].x;
	//		kShape->groundTranslations[kShape->groundTranslations.size()-1].y = trans[i].y;
	//		kShape->groundTranslations[kShape->groundTranslations.size()-1].z = 0.0;
	//	} else {
	//		kShape->nodeTranslations.increment();
	//		kShape->nodeTranslations[kShape->nodeTranslations.size()-1].x = trans[i].x;
	//		kShape->nodeTranslations[kShape->nodeTranslations.size()-1].y = trans[i].y;
	//		kShape->nodeTranslations[kShape->nodeTranslations.size()-1].z = trans[i].z;
	//	}
	//}

	////for(unsigned int j=0;j<numJoints;j++)
	//for(unsigned int j=0;j<rc;j++)
	//{
	//	for (unsigned int i=0;i<numSamples;i++)
	//	{
	//		//q16.set(rots[(i*numJoints)+orderNodes[j]]);
	//		q16.set(rots[(i*rc)+kCfg.orderNodes[j]]);
	//		kShape->nodeRotations.increment();
	//		
	//		if ((j==0)&&(importGround)) 
	//		{//HERE: make nodeRotations hold on to X and Y components, and save only Z component to groundRotations.
	//			kShape->groundRotations[i] = q16;
	//			kShape->nodeRotations[kShape->nodeRotations.size()-1] = Ogre::Quaternion::IDENTITY;
	//		}
	//		else
	//			kShape->nodeRotations[kShape->nodeRotations.size()-1] = q16;
	//	}
	//}

	////Add "Permanent" adjustments.
	//if (mBaseNodeSetPos.length()>0) setBaseNodePosRegion(kShape->sequences.size()-1,mBaseNodeSetPos,0.0,1.0);
	//if (mBaseNodeAdjustPos.length()>0) adjustBaseNodePosRegion(kShape->sequences.size()-1,mBaseNodeAdjustPos,0.0,1.0);
	//for (unsigned int i=0;i<mNodeSetRots.size();i++)
	//	if (mNodeSetRots[i].rot.length()>0)
	//		setNodeRotRegion(kShape->sequences.size()-1,mNodeSetRots[i].node,mNodeSetRots[i].rot,0.0,1.0);
	//for (unsigned int i=0;i<mNodeAdjustRots.size();i++)
	//	if (mNodeAdjustRots[i].rot.length()>0)
	//		adjustNodeRotRegion(kShape->sequences.size()-1,mNodeAdjustRots[i].node,mNodeAdjustRots[i].rot,0.0,1.0);


	////Yay! Don't have to do this anymore!
	////NOW: imported the bvh, and it should be working as a new sequence on my model.  However, 
	////due to the frequent First-Time Playback Bug, odds are high that it doesn't play back 
	////correctly.  And anyway, it would be nice to automatically save this dsq on import, just so 
	////we have it and don't have to remember to save it later.  Do that now.
	////kShape->dropAllButOneSeq(kShape->sequences.size()-1);

	//String dsqPath;
	//if (strlen(dsqFile)==0)
	//{
	//	//dsqPath = myPath + '/' + seqName;
	//	dsqPath = seqDir + seqName;//seqDir + '/' + seqName;
	//} else 
	//	dsqPath.insert(0,dsqFile);

	//FileStream *outstream;
	//String dsqExt(".dsq");
	//if (!strstr(dsqPath.c_str(),".dsq")) dsqPath += dsqExt;
	////if (!gResourceManager->openFileForWrite(outstream,dsqPath.c_str())) {
	//if ((outstream = FileStream::createAndOpen( dsqPath.c_str(), Torque::FS::File::Write))==NULL) {
	//	//Con::printf("whoops, name no good: %s!",dsqPath.c_str()); 
	//} else {
	//	//kShape->exportSequences((Stream *)outstream);
	//	kShape->exportSequence((Stream *)outstream,seq,1);//1 = save in old format (v24) for show tool
	//	
	//	outstream->close();
	//}

	////Now, load the sequence again, and drop the one we have... we hope this works.
	////10/11/10 - we may get stuck back on the first frame import bug and have to drop and 
	////reload the sequence, but we do not have to dropAllButOne in order to save it out anymore.
	//kShape->dropSequence(kShape->sequences.size()-1);
	//loadDsq(dsqPath.c_str());


	//////Con::errorf("loading sequence: %s",dsqPath.c_str());
	////loadDsq(dsqPath.c_str());
	////kShape->dropAllButOneSeq(kShape->sequences.size()-1);

	//////HERE: using my existing shapeconstructor to reload all sequences, would be the win.
	////const String myFullPath = getShapeInstance()->mShapeResource.getPath().getFullPath();
	////TSShapeConstructor* ctor = TSShapeConstructor::findShapeConstructor( myFullPath );
	////if (ctor)
	////{
	//////	//Con::errorf("found my shape constructor!  sequences %d",ctor->mSeqData.size());
	////	//ctor->??
	////	ctor->_onLoad( getShapeInstance()->mShapeResource );//see what happens if we pretend we're reloading...

	////	kShape->dropSequence(0);
	////	//String pathPlusName(dsqPath);
	////	//pathPlusName.insert(pathPlusName.length()," ");
	////	//pathPlusName.insert(pathPlusName.length(),kShape->getName(kShape->sequences[0].nameIndex));
	////	////Con::executef(ctor,"addSequence",ctor->scriptThis(), dsqPath.c_str(),kShape->getName(kShape->sequences[0].nameIndex));
	////	//Except, so far this seems much easier to do from script.  Going back out to cropCrop() in script to do this.
	////}

	//////Con::printf("BVH -- nodes %d nodeTranslations %d nodeRotations %d sequences: %d",kShape->nodes.size(),
	////	kShape->nodeTranslations.size(),kShape->nodeRotations.size(),kShape->sequences.size());
	////for (unsigned int i=0;i<kShape->sequences.size();i++)
	////{
	////	TSShape::Sequence & seq = kShape->sequences[i];

	////	//Con::printf("Seq[%d] %s frames: %d duration %3.2f baseObjectState %d baseScale %d baseDecalState %d toolbegin %f",
	////		i,kShape->getName(seq.nameIndex).c_str(),seq.numKeyframes,
	////		seq.duration,kShape->sequences[i].baseObjectState,kShape->sequences[i].baseScale,
	////		kShape->sequences[i].baseDecalState,seq.toolBegin);
	////	//Con::printf("   groundFrames %d isBlend %d isCyclic %d flags %d",
	////		seq.numGroundFrames,seq.isBlend(),seq.isCyclic(),seq.flags);
	////}
}

	//<snip>
	///////////// CUT from here down, loadBvhSkeleton //////////
	//jc=0;
	//char name[40], tempc[40];
	//char chan1[10], chan2[10], chan3[10];
	//unsigned int numChannels;

	////Load BVH
	//fgets(buf,250,fp); // HIERARCHY
	//fgets(buf,250,fp); // ROOT
	//sscanf(buf,"%s %s",&tempc,&name);
	//if (kCfg.usingNames)
	//{
	//	if (!strcmp(name,kCfg.bvhNames[0].c_str()))
	//		kCfg.bvhNodes[0] = 0;//Still necessary if we use node indices in the cfg.
	//	currBvhNode = 1;//Use this to track our next possible active bvh node index.	
	//}
	//
	//joints[0].parent = -1;
	//strcpy(joints[0].name,name);
	//fgets(buf,250,fp); // {
	//fgets(buf,250,fp);// OFFSET x y z
	//sscanf(buf,"  OFFSET %f %f %f",&p.x,&p.y,&p.z);
	//joints[0].offset = p;
	//fgets(buf,250,fp);// CHANNELS n ...
	//sscanf(buf,"  CHANNELS %d",&numChannels);

	//if (numChannels==6)
	//	sscanf(buf,"  CHANNELS %d %s %s %s %s %s %s",&numChannels,&tempc,&tempc,&tempc,&chan1,&chan2,&chan3);
	//else if (numChannels==3)
	//	sscanf(buf,"  CHANNELS %d %s %s %s",&numChannels,&chan1,&chan2,&chan3);

	//joints[0].channels = numChannels;

	////Gotta sort out what order the rotations come in, PER NODE.
	//if (chan1[0]=='X') joints[0].chanrots[0] = 0;
	//else if (chan1[0]=='Y') joints[0].chanrots[0] = 1;
	//else if (chan1[0]=='Z') joints[0].chanrots[0] = 2;
	//if (chan2[0]=='X') joints[0].chanrots[1] = 0;
	//else if (chan2[0]=='Y') joints[0].chanrots[1] = 1;
	//else if (chan2[0]=='Z') joints[0].chanrots[1] = 2;
	//if (chan3[0]=='X') joints[0].chanrots[2] = 0;
	//else if (chan3[0]=='Y') joints[0].chanrots[2] = 1;
	//else if (chan3[0]=='Z') joints[0].chanrots[2] = 2;

	//////Con::printf("JOINT %d: %s chanrots %d %d %d",0,joints[0].name,joints[0].chanrots[0],joints[0].chanrots[1],joints[0].chanrots[2]);
	//fgets(buf,250,fp); 
	//bufp = strtok(buf," \t\n");

	//loadingJoints = true;
	//while (loadingJoints)
	//{
	//	keepGoing = true;
	//	while (keepGoing)
	//	{
	//		char tempOne[40];
	//		sprintf(tempOne,"JOINT");
	//		if (!strstr(bufp,tempOne)) 
	//		{ 
	//			keepGoing = false; //End Site
	//			fgets(buf,250,fp);//{
	//			fgets(buf,250,fp);//terminal OFFSET x y z, ignore it
	//			fgets(buf,250,fp);//}
	//			break; 
	//		}
	//		jc++;
	//		if (newParent>=0) {
	//			joints[jc].parent = newParent;
	//			newParent = -1;
	//		} else joints[jc].parent = jc-1;

	//		bufp = strtok(NULL," \t\n");
	//		sscanf(bufp,"%s",name);				
	//		strcpy(joints[jc].name,name);
	//		if (kCfg.usingNames)
	//		{
	//			if (!strcmp(name,kCfg.bvhNames[currBvhNode]))
	//			{
	//				kCfg.bvhNodes[currBvhNode] = jc;
	//				currBvhNode++;
	//			}
	//		}
	//		fgets(buf,250,fp);//{
	//		fgets(buf,250,fp);//OFFSET x y z
	//		bufp = strtok(buf," \t\n");//"OFFSET"
	//		bufp = strtok(NULL," \t\n"); sscanf(bufp,"%f",&joints[jc].offset.x);
	//		bufp = strtok(NULL, " \t\n"); sscanf(bufp,"%f",&joints[jc].offset.y);
	//		bufp = strtok(NULL, " \t\n"); sscanf(bufp,"%f",&joints[jc].offset.z);

	//		fgets(buf,250,fp);//CHANNELS n s s s s s s
	//		bufp = strtok(buf," \t\n");//"CHANNELS"
	//		bufp = strtok(NULL," \t\n"); sscanf(bufp,"%d",&joints[jc].channels);
	//		if (joints[jc].channels==6) {
	//			bufp = strtok(NULL," \t\n"); 
	//			bufp = strtok(NULL," \t\n"); 
	//			bufp = strtok(NULL," \t\n"); 
	//		}
	//		bufp = strtok(NULL," \t\n"); sscanf(bufp,"%s",&chan1);
	//		bufp = strtok(NULL," \t\n"); sscanf(bufp,"%s",&chan2);
	//		bufp = strtok(NULL," \t\n"); sscanf(bufp,"%s",&chan3);
	//		//This is ugly but it's not worth rewriting chan1 etc. as 2d array.
	//		//Gotta sort out what order the rotations come in, PER NODE.
	//		if (chan1[0]=='X') joints[jc].chanrots[0] = 0;
	//		else if (chan1[0]=='Y') joints[jc].chanrots[0] = 1;
	//		else if (chan1[0]=='Z') joints[jc].chanrots[0] = 2; 
	//		if (chan2[0]=='X') joints[jc].chanrots[1] = 0;
	//		else if (chan2[0]=='Y') joints[jc].chanrots[1] = 1;
	//		else if (chan2[0]=='Z') joints[jc].chanrots[1] = 2;
	//		if (chan3[0]=='X') joints[jc].chanrots[2] = 0;
	//		else if (chan3[0]=='Y') joints[jc].chanrots[2] = 1;
	//		else if (chan3[0]=='Z') joints[jc].chanrots[2] = 2;

	//		////Con::printf("JOINT %d: %s %d %3.2f %3.2f %3.2f",jc,joints[jc].name,joints[jc].channels,
	//		//	joints[jc].offset.x,joints[jc].offset.y,joints[jc].offset.z);
	//		////Con::printf("JOINT %d: %s chanrots %d %d %d",jc,joints[jc].name,joints[jc].chanrots[0],joints[jc].chanrots[1],joints[jc].chanrots[2]);
	//		//if (bvhNodes[jc]==scaleNode)
	//		//{
	//		//	//float bvhForearmLen = joints[jc].offset.length();
	//		//	////Con::errorf("jc %d forearm length: %f",jc,bvhForearmLen);
	//		//	//float dtsForearmLen = getShape()->defaultTranslations[dtsNodes[jc]].length();
	//		//	//transScale = dtsForearmLen / bvhForearmLen;
	//		//	if (1) //"INCHES"
	//		//		transScale = 1.0 / 39.0;
	//		//	else
	//		//		transScale = 1.0;
	//		//}

	//		fgets(buf,250,fp);
	//		bufp = strtok(buf," \t\n");
	//	}
	//	//fgets(buf,250,fp);// JOINT?

	//	//HERE: back out of the nested curly braces, checking for new JOINTs and decrementing jloop each time.

	//	keepGoing = true;
	//	while (keepGoing)
	//	{
	//		fgets(buf,250,fp);
	//		
	//		bufp = strtok(buf," \n\t");
	//		char tempOne[40],tempTwo[40],tempThree[40];
	//		sprintf(tempOne,"}");
	//		sprintf(tempTwo,"JOINT");
	//		sprintf(tempThree,"MOTION");
	//		if (strstr(bufp,tempOne))
	//		{
	//			jloop++;
	//		} else if (strstr(bufp,tempTwo)) {
	//			newParent = joints[jc-(jloop-1)].parent;
	//			keepGoing = false;
	//		} else if (strstr(bufp,tempThree)) {
	//			//Con::printf("BVH -- loaded schema, proceeding to motion frames.");
	//			keepGoing = false;
	//			loadingJoints = false;
	//		} else {
	//			//Con::errorf("BVH -- problem, found no frames.");
	//			fclose(fp);
	//			return;
	//		}
	//	}
	//}
	/////////// END CUT  loadBvhSkeleton  ///////////////////////////
	//</snip>

/*
Hmmm... some quick notes.  Main issue is I need to check the parentage of nodes as I go along.  
Each dts node has a parent index, but this won't correlate to the bvh indices, because there are
more dts nodes than bvh nodes.  Need to detect the end of a limb by finding out, when I get to a 
new node, whether the new node is descended from the last node or not.
This might be as easy as:  Is the new node's parentindex equal to or lower than the last node's index?
We know we might skip nodes, but we can't jump below us on the list -- nodes are always parented to nodes
somewhere ABOVE them in the hierarchy.  So if we get to the right hand node, and the next node is the left
collar node, the parent for left collar will be a spine node.  Which has an index much lower than right hand.
	So, the next question is how we back up and find out how many tabs to remove before we get to the parent
we want.  Need to keep track of indices all the way down, so we can find the first one where the parent we're 
looking for is NOT less than this node's index.
*/
void fxFlexBody::saveBvh(unsigned int seqNum, const char *bvh_file)
{
	saveBvh(seqNum,bvh_file,"native");
	return;
}

void fxFlexBody::saveBvh(unsigned int seqNum, const char *bvh_file, const char *bvh_format)
{
	//int rot_matters_count,profile_id;
	//unsigned int node_matters[MAX_BVH_NODES], parent_chain[MAX_BVH_NODES];//, node_indices[MAX_BVH_NODES];
	//int bvhNodeMatters[MAX_BVH_NODES], dtsNodeMatters[MAX_BVH_NODES];
	//unsigned int start_rot, start_trans, first_ground, tab_count,parent_count,num_keyframes,rc,jc;
	//char tabs[255],rotOrder[255];
	//FILE *fpw,*fpc,*fps;//write file, config file, skeleton file
	//fpw = fpc = fps = NULL;
	//bvhCfgData kCfg;
	//Ogre::Vector3 p,r;
	//float scale_factor = 39.0;
	//rc = 0; jc = 0;
	//profile_id = 0;
	////Moved these all to bvhCfgFileData struct
	////int bvhNodes[MAX_BVH_NODES];
	////int dtsNodes[MAX_BVH_NODES];
	////int sortNodes[MAX_BVH_NODES];
	////int orderNodes[MAX_BVH_NODES];//from initial order to shape order
	////Ogre::Vector3 bvhPoseRotsA[MAX_BVH_NODES],bvhPoseRotsB[MAX_BVH_NODES];//allow for two later, if we need it
	////Ogre::Vector3 axesFixRotsA[MAX_BVH_NODES],axesFixRotsB[MAX_BVH_NODES];
	////String bvhNames[MAX_BVH_NODES];

	//TSShape *kShape = getShapeInstance()->getShape();
	//TSShape::Sequence *kSeq = &(kShape->sequences[seqNum]);

	////The new, 1.8 way to do it:
	//String myPath = getShapeInstance()->mShapeResource.getPath().getPath();
	//String configName,configPath,skeletonName,skeletonPath;
	//configName.clear();
	//skeletonName.clear();

	//float pi_over_180,pi_under_180;

	//pi_over_180 = M_PI/180.0;
	//pi_under_180 = 180.0/M_PI;

	//unsigned int bvhFormat = 0;
	//SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(mPM)->getSQL();
	//if (!sql->OpenDatabase("EcstasyMotion.db"))
	//{
	//	//Con::errorf("saveBvh: couldn't open database EcstasyMotion.db");
	//	delete sql;
	//	return;
	//}

	//char id_query[512],insert_query[512];
	//int result;
	//sqlite_resultset *resultSet;
	//sprintf(id_query,"SELECT id FROM bvhProfile WHERE name = '%s';",bvh_format);
	//result = sql->ExecuteSQL(id_query);
	//resultSet = sql->GetResultSet(result);
	//if (resultSet->iNumRows == 1)
	//	profile_id = strtol(resultSet->vRows[0]->vColumnValues[0]);

	//if (profile_id==0)
	//{
	//	//Con::errorf("couldn't find profile_id: %s",bvh_format);
	//	delete sql;
	//	return;
	//}

	//sprintf(id_query,"SELECT id FROM bvhProfileJoint WHERE bvh_profile_id = %d;",profile_id);
	//result = sql->ExecuteSQL(id_query);
	//resultSet = sql->GetResultSet(result);
	////WAIT:  rc is node count form bvhProfileNode, not bvhProfileJoint...
	//jc = resultSet->iNumRows;//rc = node count,   jc = joint count in bvh file

	//if (strcmp(bvh_format,"native")) //if NOT "native"
	//{//CHANGE:  now we are reading bvh cfg data from the db instead of a file, using BvhProfileList for profile name
	//	char myNormalString[255];
	//	bvhFormat = 1;
	//	//SQLiteObject *sql = dynamic_cast<nxPhysManager*>(mPM)->getSQL();
	//	//if (!sql) return;
	//	
	//	//myNormalString = NULL; // Now we don't need this anymore
	//	//sprintf(myNormalString,"/%s.cfg",bvh_format);
	//	////configName = '/' + bvh_format;
	//	////configName += ".cfg";
	//	//configName.insert(0,myNormalString);
	//	
	//	//HERE: this is to be used only on the first time you use the profile.  It would be better
	//	//if we had a bvh type by itself, ie "Truebones" rather than having to do this once per 
	//	//bvhProfile, "Truebones2ACK".  But this is how it is right now.  The least you should do
	//	//is make it ask for a sample bvh if it doesn't find one called "_.skeleton.bvh".
	//	//Had to change name of sample skeleton file to: Truebones2ACK.skeleton.bvh for this part to work. 
	//	//(Load this into DB next, however - bvhProfileJoint table, for holding skeleton data.)
	//	sprintf(myNormalString,"/%s.skeleton.bvh",bvh_format);//used first time only
	//	skeletonName.insert(0,myNormalString);
	//}

	//if (rc==0)
	//{
	//	//configPath = myPath + configName;
	//	//fpc = fopen(configPath.c_str(),"r");
	//	//if (fpc==NULL)
	//	//{
	//	//	//Con::errorf("ERROR: can't open config file %s",myPath.c_str());
	//	//	return;
	//	//}

	//	//rc = loadBvhCfg(fpc,bvhNodes,dtsNodes,sortNodes,orderNodes,bvhPoseRotsA,bvhPoseRotsB,
	//	//	axesFixRotsA,axesFixRotsB,bvhNames);
	//	rc = loadBvhCfgDB(&kCfg,profile_id);
	//	
	//	//HERE: load data from db instead of opening a file, if you have it.
	//}
	//if (jc == 0)
	//{
	//	skeletonPath = myPath + skeletonName;
	//	fps = fopen(skeletonPath.c_str(),"r");
	//	if (fps==NULL)
	//	{
	//		//Con::errorf("ERROR: can't open skeleton file %s",myPath.c_str());
	//		return;
	//	}

	//	jc = loadBvhSkeleton(fps,&kCfg,profile_id);//FIX!  get profile_id
	//} else {
	//	loadBvhSkeletonDB(&kCfg,profile_id);//FIX!  get profile_id
	//}

	//String bvhFile(bvh_file);
	//String bvhExt(".bvh");
	//if (!strstr(bvhFile.c_str(),".bvh")) bvhFile += bvhExt;
	//
	////Con::errorf("opening filename: %s, original %s",bvhFile.c_str(),bvh_file);

	//fpw = fopen(bvhFile.c_str(),"w");

	////////////////////////////////////////

	//rot_matters_count = 0;
	//for (unsigned int i=0;i<kShape->nodes.size();i++) 
	//{
	//	if (kSeq->rotationMatters.test(i)) 
	//	{
	//		node_matters[rot_matters_count] = i;
	//		dtsNodeMatters[i] = rot_matters_count;
	//		rot_matters_count++;
	//	}
	//}

	//tab_count = 1;
	//int node_marker = 0;
	//for (unsigned int i=0;i<MAX_BVH_NODES;i++) { parent_chain[i] = -1; }
	//parent_chain[0] = 0;

	//if (bvhFormat==0)
	//{//Using "Native" node names and only exporting active nodes.
	//	fprintf(fpw,"HIERARCHY\n");
	//	fprintf(fpw,"ROOT %s\n",kShape->getName(kShape->nodes[0].nameIndex).c_str());

	//	fprintf(fpw,"{\n");
	//	//fprintf(fpw,"\tOFFSET %f %f %f\n",kShape->defaultTranslations[0].x,kShape->defaultTranslations[0].y,kShape->defaultTranslations[0].z);
	//	fprintf(fpw,"\tOFFSET 0.00 0.00 0.00\n");

	//	//fprintf(fpw,"\tCHANNELS 6  Xposition Yposition Zposition Zrotation Xrotation Yrotation\n");
	//	//fprintf(fpw,"\tCHANNELS 6  Xposition Yposition Zposition Yrotation Xrotation Zrotation\n");
	//	//fprintf(fpw,"\tCHANNELS 6  Xposition Yposition Zposition Zrotation Yrotation Xrotation\n");
	//	//fprintf(fpw,"\tCHANNELS 6  Xposition Yposition Zposition Yrotation Zrotation Xrotation\n");
	//	fprintf(fpw,"\tCHANNELS 6  Xposition Yposition Zposition Xrotation Yrotation Zrotation\n");
	//	//fprintf(fpw,"\tCHANNELS 6  Xposition Yposition Zposition Xrotation Zrotation Yrotation\n");
	//	//Number six of six choices... never forget the pain...

	//	for (unsigned int i=1;i<rot_matters_count;i++)
	//	{
	//		//HERE: have to make a method of tab-multiplying, to keep track of indents.  Did this before somewhere, resurrect.
	//		//Has to involve joint parent nodes.  Need "End Site" tag when we reach the end of a (limb).  Keep checking to see if bodypart
	//		//in front is parent, if not go to bodypart in front of that, sooner or later the next bodypart will find it's parent, unless
	//		//it is a new root node in which case start a new body.

	//		int back_up = 0;
	//		int nodeParentIndex = kShape->nodes[node_matters[i]].parentIndex;
	//		//Con::errorf("i: %d, node: %d, tab_count %d, parentIndex %d",i,node_matters[i],tab_count,nodeParentIndex);
	//		//while ((i>1)&&(nodeParentIndex < node_matters[i-(back_up+1)])) 
	//		while ((i>1)&&(nodeParentIndex < parent_chain[tab_count - (back_up+1)])) 
	//		{//HERE: we're at the end of a limb!   First, find out how far back up the loop we have to go,
	//			//then do the END SITE line and back out of the curly braces and decrement tab_count.
	//			//Con::printf("backing up... parentIndex = %d, parent_chain[%d] = %d",nodeParentIndex,tab_count - (back_up+1),parent_chain[tab_count - (back_up+1)]);
	//			back_up++;
	//		}

	//		if (!back_up) {//back_up==0, meaning we're still moving down the limb.
	//			parent_chain[tab_count] = node_matters[i];
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//			fprintf(fpw,"JOINT %s\n",kShape->getName(kShape->nodes[node_matters[i]].nameIndex).c_str());
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//			fprintf(fpw,"{\n");
	//			tab_count++;
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//			Ogre::Vector3 p = kShape->defaultTranslations[node_matters[i]];
	//			fprintf(fpw,"OFFSET %f %f %f\n",-p.x*scale_factor,p.z*scale_factor,p.y*scale_factor);
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");

	//			//fprintf(fpw,"CHANNELS 3 Zrotation Xrotation Yrotation\n");
	//			//fprintf(fpw,"CHANNELS 3 Yrotation Xrotation Zrotation\n");
	//			//fprintf(fpw,"CHANNELS 3 Zrotation Yrotation Xrotation\n");
	//			//fprintf(fpw,"CHANNELS 3 Yrotation Zrotation Xrotation\n");
	//			fprintf(fpw,"CHANNELS 3 Xrotation Yrotation Zrotation\n");
	//			//fprintf(fpw,"CHANNELS 3 Xrotation Zrotation Yrotation\n");
	//		} else {
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//			fprintf(fpw,"End Site\n");
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//			fprintf(fpw,"{\n");
	//			tab_count++;
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//			fprintf(fpw,"OFFSET 0.0 0.0 0.1\n");//FIX: find a way to get a reasonable offset from toe or finger nodes... varies by model, 
	//			//but at least we could hard code something for the ACK model for now.
	//			tab_count--;
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//			fprintf(fpw,"}\n");
	//			while (back_up-- > 0) 
	//			{
	//				tab_count--;
	//				for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//				fprintf(fpw,"}\n");
	//			}
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");

	//			fprintf(fpw,"JOINT %s\n",kShape->getName(kShape->nodes[node_matters[i]].nameIndex).c_str());
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//			fprintf(fpw,"{\n");
	//			tab_count++;
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//			Ogre::Vector3 p = kShape->defaultTranslations[node_matters[i]];
	//			fprintf(fpw,"OFFSET %f %f %f\n",-p.x*scale_factor,p.z*scale_factor,p.y*scale_factor);//TEMP: Have to get things to "normal" BVH scale somehow.
	//			//Also have to switch z and y and reverse x, for righthandedness.
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");

	//			//fprintf(fpw,"CHANNELS 3 Zrotation Xrotation Yrotation\n");
	//			//fprintf(fpw,"CHANNELS 3 Yrotation Xrotation Zrotation\n");
	//			//fprintf(fpw,"CHANNELS 3 Zrotation Yrotation Xrotation\n");
	//			//fprintf(fpw,"CHANNELS 3 Yrotation Zrotation Xrotation\n");
	//			fprintf(fpw,"CHANNELS 3 Xrotation Yrotation Zrotation\n");
	//			//fprintf(fpw,"CHANNELS 3 Xrotation Zrotation Yrotation\n");
	//		}
	//	} 
	//	for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//	fprintf(fpw,"End Site\n");
	//	for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//	fprintf(fpw,"{\n");
	//	tab_count++;
	//	for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//	fprintf(fpw,"OFFSET 0.0 0.0 0.1\n");//FIX: find a way to get a reasonable offset from toe or finger nodes... varies by model, 
	//	//but at least we could hard code something for the ACK model for now.
	//	tab_count--;
	//	for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//	fprintf(fpw,"}\n");
	//	while (tab_count > 0) 
	//	{
	//		tab_count--;
	//		for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//		fprintf(fpw,"}\n");
	//	}

	//	//HERE: then, do the motion frames...
	//	//fprintf(fpw,"}\n");
	//	fprintf(fpw,"MOTION\n");
	//	fprintf(fpw,"Frames: %d\n",kSeq->numKeyframes);
	//	fprintf(fpw,"Frame Time: %f\n",kSeq->duration/((float)kSeq->numKeyframes));
	//	//Don't forget:  if there are ground frames, then copy those into the root node positions in the bvh.

	//	start_rot = kSeq->baseRotation;
	//	start_trans = kSeq->baseTranslation;
	//	first_ground = kSeq->firstGroundFrame;
	//	num_keyframes = kSeq->numKeyframes;


	//	for (unsigned int i=0;i<num_keyframes;i++)
	//	{//FIX: go through trans_matters, don't assume only 0
	//		fprintf(fpw,"%f %f %f ",-kShape->nodeTranslations[start_trans+i].x*scale_factor,kShape->nodeTranslations[start_trans+i].z*scale_factor,kShape->nodeTranslations[start_trans+i].y*scale_factor);
	//		//Then, go through all the nodes, convert all rotations to euler, write out.
	//		for (unsigned int j=0;j<rot_matters_count;j++) 
	//		{
	//			Quat16 q16 = kShape->nodeRotations[start_rot + (j * num_keyframes) + i];
	//			Ogre::Quaternion q;
	//			Ogre::Vector3 eul;
	//			Ogre::Matrix3 mat;

	//			q16.getOgre::Quaternion(&q);
	//			q.setMatrix(&mat);
	//			eul = mat.toEuler();

	//			Ogre::Vector3 row0,row1,row2;
	//			mat.getRow(0,&row0);
	//			mat.getRow(1,&row1);
	//			mat.getRow(2,&row2);

	//			HMatrix	hMatrix;
	//			hMatrix[0][0] = row0.x; hMatrix[1][0] = row0.y; hMatrix[2][0] = row0.z; hMatrix[3][0] = 0.0;
	//			hMatrix[0][1] = row1.x; hMatrix[1][1] = row1.y; hMatrix[2][1] = row1.z; hMatrix[3][1] = 0.0;
	//			hMatrix[0][2] = row2.x; hMatrix[1][2] = row2.y; hMatrix[2][2] = row2.z; hMatrix[3][2] = 0.0;
	//			hMatrix[0][3] = 0.0; hMatrix[1][3] = 0.0; hMatrix[2][3] = 0.0; hMatrix[3][3] = 1.0;

	//			EulerAngles eulQ;
	//			eulQ = Eul_FromHMatrix( hMatrix,EulOrdXYZs);
	//			//Con::errorf("XYZ Euler: %f %f %f - %f",eulQ.x * pi_under_180,eulQ.y * pi_under_180,eulQ.z * pi_under_180,eulQ.w);
	//			//eulQ = Eul_FromHMatrix( hMatrix,EulOrdZXYs);
	//			////Con::errorf("ZXY Euler: %f %f %f - %f",eulQ.x * pi_under_180,eulQ.y * pi_under_180,eulQ.z * pi_under_180,eulQ.w);

	//			fprintf(fpw,"%3.2f %3.2f %3.2f ",mRadToDeg(eulQ.x),-mRadToDeg(eulQ.z),-mRadToDeg(eulQ.y));
	//			//fprintf(fpw,"%3.2f %3.2f %3.2f ",-mRadToDeg(eul.y),mRadToDeg(eul.x),-mRadToDeg(eul.z));
	//			//fprintf(fpw,"%3.2f %3.2f %3.2f ",-mRadToDeg(eul.z),mRadToDeg(eul.x),-mRadToDeg(eul.y));
	//			//fprintf(fpw,"%3.2f %3.2f %3.2f ",-mRadToDeg(eul.y),-mRadToDeg(eul.z),mRadToDeg(eul.x));
	//			//fprintf(fpw,"%3.2f %3.2f %3.2f ",-mRadToDeg(eul.z),-mRadToDeg(eul.y),mRadToDeg(eul.x));
	//			//fprintf(fpw,"%3.2f %3.2f %3.2f ",mRadToDeg(eul.x),-mRadToDeg(eul.z),-mRadToDeg(eul.y));
	//			//fprintf(fpw,"%3.2f %3.2f %3.2f ",mRadToDeg(eul.x),-mRadToDeg(eul.y),-mRadToDeg(eul.z));
	//		}
	//		fprintf(fpw,"\n");
	//	}
	//	//////////////////////////////////////////////////////////////
	//} else { //Using bvhFormat that requires exporting all nodes, whether active or not.
	//	//////////////////////////////////////////////////////////////
	//	//First, get index from bvh node order to corresponding place in nodeRotations, via rotationMatters. 
	//	for (unsigned int i=0;i<rc;i++)
	//	{
	//		if (kCfg.dtsNodes[i] > -1)
	//		{
	//			if (kSeq->rotationMatters.test(kCfg.dtsNodes[i]))
	//				bvhNodeMatters[i] = dtsNodeMatters[kCfg.dtsNodes[i]];
	//			else 
	//				bvhNodeMatters[i] = -1;
	//		} else 
	//			bvhNodeMatters[i] = -1;
	//	}

	//	//NOW, we've loaded the joints[] array, which has parentage data.

	//	fprintf(fpw,"HIERARCHY\n");
	//	fprintf(fpw,"ROOT %s\n",kCfg.bvhNames[0].c_str());

	//	fprintf(fpw,"{\n");
	//	//fprintf(fpw,"\tOFFSET %f %f %f\n",kShape->defaultTranslations[0].x,kShape->defaultTranslations[0].y,kShape->defaultTranslations[0].z);
	//	fprintf(fpw,"\tOFFSET 0.00 0.00 0.00\n");//Is this okay?

	//	if ((kCfg.joints[0].chanrots[0]==0)&&(kCfg.joints[0].chanrots[1]==1)&&(kCfg.joints[0].chanrots[2]==2))
	//		sprintf(rotOrder,"Xrotation Yrotation Zrotation");
	//	else if ((kCfg.joints[0].chanrots[0]==0)&&(kCfg.joints[0].chanrots[1]==2)&&(kCfg.joints[0].chanrots[2]==1))
	//		sprintf(rotOrder,"Xrotation Zrotation Yrotation");
	//	else if ((kCfg.joints[0].chanrots[0]==1)&&(kCfg.joints[0].chanrots[1]==0)&&(kCfg.joints[0].chanrots[2]==2))
	//		sprintf(rotOrder,"Yrotation Xrotation Zrotation");
	//	else if ((kCfg.joints[0].chanrots[0]==1)&&(kCfg.joints[0].chanrots[1]==2)&&(kCfg.joints[0].chanrots[2]==0))
	//		sprintf(rotOrder,"Yrotation Zrotation Xrotation");
	//	else if ((kCfg.joints[0].chanrots[0]==2)&&(kCfg.joints[0].chanrots[1]==0)&&(kCfg.joints[0].chanrots[2]==1))
	//		sprintf(rotOrder,"Zrotation Xrotation Yrotation");
	//	else if ((kCfg.joints[0].chanrots[0]==2)&&(kCfg.joints[0].chanrots[1]==1)&&(kCfg.joints[0].chanrots[2]==0))
	//		sprintf(rotOrder,"Zrotation Yrotation Xrotation");

	//	//fprintf(fpw,"\tCHANNELS 6  Xposition Yposition Zposition Zrotation Xrotation Yrotation\n");fprintf(fpw,"\tCHANNELS 6  Xposition Yposition Zposition Zrotation Xrotation Yrotation\n");
	//	fprintf(fpw,"\tCHANNELS 6  Xposition Yposition Zposition %s\n",rotOrder);


	//	//Damn, another parsing section - need to load the skeleton from another bvh file, so I can get the proper
	//	//hierarchy.  The dts hierarchy won't work, because some bvh nodes might not be in the dts model at all,
	//	//and some might have slightly different parentage.
	//	for (unsigned int i=1;i<rc;i++)
	//	{
	//		int back_up = 0;
	//		int nodeParentIndex = -1;
	//		if (kCfg.joints[i].parent > -1)
	//			nodeParentIndex = kCfg.joints[i].parent;//orderNodes[i]
	//		

	//		////Con::errorf("i: %d, node: %d, tab_count %d, parentIndex %d",i,kCfg.dtsNodes[i],tab_count,nodeParentIndex);//orderNodes[i]
	//		//while ((i>1)&&(nodeParentIndex < node_matters[i-(back_up+1)])) 
	//		while ((i>1)&&(nodeParentIndex < parent_chain[tab_count - (back_up+1)])) 
	//		{//HERE: we're at the end of a limb!   First, find out how far back up the loop we have to go,
	//			//then do the END SITE line and back out of the curly braces and decrement tab_count.
	//			////Con::printf("backing up... parentIndex = %d, parent_chain[%d] = %d",nodeParentIndex,tab_count - (back_up+1),parent_chain[tab_count - (back_up+1)]);
	//			back_up++;
	//		}

	//		if (!back_up) {//back_up==0, meaning we're still moving down the limb.
	//			parent_chain[tab_count] = i;
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");

	//			fprintf(fpw,"JOINT %s\n",kCfg.bvhNames[i].c_str());
	//		
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//			fprintf(fpw,"{\n");
	//			tab_count++;
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");

	//			//HERE: this actually needs to be the offset from the original skeleton, not my dts skeleton.
	//			//if (kCfg.dtsNodes[i] > -1)
	//			//	p = kShape->defaultTranslations[kCfg.dtsNodes[i]];//kCfg.orderNodes[i]
	//			//else
	//			//	p = Ogre::Vector3::ZERO;
	//			//fprintf(fpw,"OFFSET %f %f %f\n",-p.x*scale_factor,p.z*scale_factor,p.y*scale_factor);
	//			
	//			p = kCfg.joints[i].offset;
	//			fprintf(fpw,"OFFSET %f %f %f\n",p.x,p.y,p.z);
	//			//But now we're going to have to convert the rotations from native to whatever arm position 
	//			//we had in the original bvh. (i.e. arms out in T-Pose, or down at sides?)
	//			
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");

	//			//fprintf(fpw,"CHANNELS 3 Zrotation Xrotation Yrotation\n");
	//			if ((kCfg.joints[i].chanrots[0]==0)&&(kCfg.joints[i].chanrots[1]==1)&&(kCfg.joints[i].chanrots[2]==2))
	//				sprintf(rotOrder,"Xrotation Yrotation Zrotation");
	//			else if ((kCfg.joints[i].chanrots[0]==0)&&(kCfg.joints[i].chanrots[1]==2)&&(kCfg.joints[i].chanrots[2]==1))
	//				sprintf(rotOrder,"Xrotation Zrotation Yrotation");
	//			else if ((kCfg.joints[i].chanrots[0]==1)&&(kCfg.joints[i].chanrots[1]==0)&&(kCfg.joints[i].chanrots[2]==2))
	//				sprintf(rotOrder,"Yrotation Xrotation Zrotation");
	//			else if ((kCfg.joints[i].chanrots[0]==1)&&(kCfg.joints[i].chanrots[1]==2)&&(kCfg.joints[i].chanrots[2]==0))
	//				sprintf(rotOrder,"Yrotation Zrotation Xrotation");
	//			else if ((kCfg.joints[i].chanrots[0]==2)&&(kCfg.joints[i].chanrots[1]==0)&&(kCfg.joints[i].chanrots[2]==1))
	//				sprintf(rotOrder,"Zrotation Xrotation Yrotation");
	//			else if ((kCfg.joints[i].chanrots[0]==2)&&(kCfg.joints[i].chanrots[1]==1)&&(kCfg.joints[i].chanrots[2]==0))
	//				sprintf(rotOrder,"Zrotation Yrotation Xrotation");

	//			//fprintf(fpw,"\tCHANNELS 6  Xposition Yposition Zposition Zrotation Xrotation Yrotation\n");fprintf(fpw,"\tCHANNELS 6  Xposition Yposition Zposition Zrotation Xrotation Yrotation\n");
	//			fprintf(fpw,"\tCHANNELS 3 %s\n",rotOrder);
	//			//fprintf(fpw,"CHANNELS 3 Xrotation Yrotation Zrotation\n");
	//		} else {
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//			fprintf(fpw,"End Site\n");
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//			fprintf(fpw,"{\n");
	//			tab_count++;
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//			//fprintf(fpw,"OFFSET 0.0 0.0 0.1\n");//HERE:  Take offset from bvh!!
	//			Ogre::Vector3 offset = kCfg.endSiteOffsets[kCfg.nodeGroups[i-1]];
	//			fprintf(fpw,"OFFSET %f %f %f\n",offset.x,offset.y,offset.z);
	//			tab_count--;
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//			fprintf(fpw,"}\n");
	//			while (back_up-- > 0) 
	//			{
	//				tab_count--;
	//				for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//				fprintf(fpw,"}\n");
	//			}
	//			
	//			parent_chain[tab_count] = i;//kCfg.dtsNodes[kCfg.orderNodes[i]];

	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");

	//			fprintf(fpw,"JOINT %s\n",kCfg.bvhNames[i].c_str());
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//			fprintf(fpw,"{\n");
	//			tab_count++;
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");

	//			//if (kCfg.dtsNodes[i] > -1)
	//			//	p = kShape->defaultTranslations[kCfg.dtsNodes[i]];
	//			//else
	//			//	p = Ogre::Vector3::ZERO;
	//			//fprintf(fpw,"OFFSET %f %f %f\n",-p.x*scale_factor,p.z*scale_factor,p.y*scale_factor);//TEMP: Have to get things to "normal" BVH scale somehow.
	//			
	//			p = kCfg.joints[i].offset;
	//			fprintf(fpw,"OFFSET %f %f %f\n",p.x,p.y,p.z);
	//			//Also have to switch z and y and reverse x, for righthandedness.
	//			for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");

	//			//HERE:  Make this whatever way required by the bvh flavor.  NOT always ZXY.
	//			//fprintf(fpw,"CHANNELS 3 Zrotation Xrotation Yrotation\n");
	//			if ((kCfg.joints[i].chanrots[0]==0)&&(kCfg.joints[i].chanrots[1]==1)&&(kCfg.joints[i].chanrots[2]==2))
	//				sprintf(rotOrder,"Xrotation Yrotation Zrotation");
	//			else if ((kCfg.joints[i].chanrots[0]==0)&&(kCfg.joints[i].chanrots[1]==2)&&(kCfg.joints[i].chanrots[2]==1))
	//				sprintf(rotOrder,"Xrotation Zrotation Yrotation");
	//			else if ((kCfg.joints[i].chanrots[0]==1)&&(kCfg.joints[i].chanrots[1]==0)&&(kCfg.joints[i].chanrots[2]==2))
	//				sprintf(rotOrder,"Yrotation Xrotation Zrotation");
	//			else if ((kCfg.joints[i].chanrots[0]==1)&&(kCfg.joints[i].chanrots[1]==2)&&(kCfg.joints[i].chanrots[2]==0))
	//				sprintf(rotOrder,"Yrotation Zrotation Xrotation");
	//			else if ((kCfg.joints[i].chanrots[0]==2)&&(kCfg.joints[i].chanrots[1]==0)&&(kCfg.joints[i].chanrots[2]==1))
	//				sprintf(rotOrder,"Zrotation Xrotation Yrotation");
	//			else if ((kCfg.joints[i].chanrots[0]==2)&&(kCfg.joints[i].chanrots[1]==1)&&(kCfg.joints[i].chanrots[2]==0))
	//				sprintf(rotOrder,"Zrotation Yrotation Xrotation");

	//			//fprintf(fpw,"\tCHANNELS 6  Xposition Yposition Zposition Zrotation Xrotation Yrotation\n");fprintf(fpw,"\tCHANNELS 6  Xposition Yposition Zposition Zrotation Xrotation Yrotation\n");
	//			fprintf(fpw,"\tCHANNELS 3 %s\n",rotOrder);
	//			//fprintf(fpw,"CHANNELS 3 Xrotation Yrotation Zrotation\n");
	//		}
	//	}
	//	for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//	fprintf(fpw,"End Site\n");
	//	for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//	fprintf(fpw,"{\n");
	//	tab_count++;
	//	for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//	Ogre::Vector3 offset = kCfg.endSiteOffsets[kCfg.nodeGroups[rc-1]];
	//	fprintf(fpw,"OFFSET %f %f %f\n",offset.x,offset.y,offset.z);
	//	//fprintf(fpw,"OFFSET 0.0 0.0 0.1\n");//FIX: find a way to get a reasonable offset from toe or finger nodes... varies by model, 
	//	//but at least we could hard code something for the ACK model for now.
	//	tab_count--;
	//	for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//	fprintf(fpw,"}\n");
	//	while (tab_count > 0) 
	//	{
	//		tab_count--;
	//		for (unsigned int j=0;j<tab_count;j++) fprintf(fpw,"\t");
	//		fprintf(fpw,"}\n");
	//	}


	//	//HERE: then, do the motion frames...
	//	//fprintf(fpw,"}\n");
	//	fprintf(fpw,"MOTION\n");
	//	fprintf(fpw,"Frames: %d\n",kSeq->numKeyframes);
	//	fprintf(fpw,"Frame Time: %f\n",kSeq->duration/((float)kSeq->numKeyframes));
	//	//Don't forget:  if there are ground frames, then copy those into the root node positions in the bvh.

	//	start_rot = kSeq->baseRotation;
	//	start_trans = kSeq->baseTranslation;
	//	first_ground = kSeq->firstGroundFrame;
	//	num_keyframes = kSeq->numKeyframes;


	//	for (unsigned int i=0;i<num_keyframes;i++)
	//	{//FIX: go through trans_matters, don't assume only 0
	//		float new_scale_factor = scale_factor * (2.0/mShapeSize);
	//		fprintf(fpw,"%f %f %f ",-kShape->nodeTranslations[start_trans+i].x*new_scale_factor,
	//			kShape->nodeTranslations[start_trans+i].z*new_scale_factor,
	//			kShape->nodeTranslations[start_trans+i].y*new_scale_factor);
	//		//Then, go through all the nodes, convert all rotations to euler, write out.
	//		for (unsigned int j=0;j<rc;j++) 
	//		{
	//			Quat16 q16;
	//			if (bvhNodeMatters[j]>=0)
	//				q16 = kShape->nodeRotations[start_rot + (bvhNodeMatters[j] * num_keyframes) + i];
	//			else 
	//				q16 = Ogre::Quaternion::IDENTITY;

	//			Ogre::Quaternion q;
	//			Ogre::Vector3 eul;
	//			Ogre::Matrix3 m,mat,matBvhPose,matBvhPoseA,matBvhPoseB,matAxesFixA,matAxesFixB,matAxesFix,matAxesUnfix;


	//			//HERE: do necessary rotations from config file!  Need this now because we have to use 
	//			//skeleton from original bvh, not our ACK skeleton, and bvh might have arms-down root pose.
	//			//EXCEPT: temporarily putting all this back, didn't get it finished and got a different way
	//			//to get into iClone anyway.  Still need to do this anyway, though, to fix shoulder problems,
	//			//and handle export to formats where arms are down.

	//			//matBvhPoseA.set(kCfg.bvhPoseRotsA[j]);
	//			//matBvhPoseB.set(kCfg.bvhPoseRotsB[j]);
	//			//matBvhPose.mul(matBvhPoseA,matBvhPoseB);

	//			//matAxesFixA.set(kCfg.axesFixRotsA[j]);
	//			//matAxesFixB.set(kCfg.axesFixRotsB[j]);
	//			//matAxesFix.mul(matAxesFixA,matAxesFixB);

	//			//matAxesUnfix = matAxesFix;
	//			//matAxesUnfix.inverse();

	//			//m = Ogre::Quaternion::IDENTITY;	
	//			//m.mul(matBvhPose);	
	//			//m.mul(matAxesFix);
	//			//m.mul(mat);
	//			//m.mul(matAxesUnfix);
	//			//Here: end of cfg adjustment, putting things back the way they were.

	//			Ogre::Vector3 p,row0,row1,row2;
	//			HMatrix	hMatrix;
	//			EulerAngles eulQ;

	//			q16.getOgre::Quaternion(&q);
	//			q.setMatrix(&mat);

	//			m = mat;
	//			//m.rightToLeftHanded(mat);//wait a minute...

	//			mat.getRow(0,&row0);
	//			mat.getRow(1,&row1);
	//			mat.getRow(2,&row2);

	//			//Column major interpretation of hMatrix:
	//			hMatrix[0][0] = row0.x; hMatrix[1][0] = row0.y; hMatrix[2][0] = row0.z; hMatrix[3][0] = 0.0;
	//			hMatrix[0][1] = row1.x; hMatrix[1][1] = row1.y; hMatrix[2][1] = row1.z; hMatrix[3][1] = 0.0;
	//			hMatrix[0][2] = row2.x; hMatrix[1][2] = row2.y; hMatrix[2][2] = row2.z; hMatrix[3][2] = 0.0;
	//			hMatrix[0][3] = 0.0;    hMatrix[1][3] = 0.0;    hMatrix[2][3] = 0.0;    hMatrix[3][3] = 1.0;

	//			//Just in case, had to try a row-major interpretation as well.
	//			//hMatrix[0][0] = row0.x; hMatrix[1][0] = row1.x; hMatrix[2][0] = row2.x; hMatrix[3][0] = 0.0;
	//			//hMatrix[0][1] = row0.y; hMatrix[1][1] = row1.y; hMatrix[2][1] = row2.y; hMatrix[3][1] = 0.0;
	//			//hMatrix[0][2] = row0.z; hMatrix[1][2] = row1.z; hMatrix[2][2] = row2.z; hMatrix[3][2] = 0.0;
	//			//hMatrix[0][3] = 0.0; hMatrix[1][3] = 0.0; hMatrix[2][3] = 0.0; hMatrix[3][3] = 1.0;
 //
	//			int kOrder;

	//			if ((kCfg.joints[j].chanrots[0]==0)&&(kCfg.joints[j].chanrots[1]==1)&&(kCfg.joints[j].chanrots[2]==2))
	//				kOrder = EulOrdXZYs;//kOrder = EulOrdXYZs;//
	//			else if ((kCfg.joints[j].chanrots[0]==0)&&(kCfg.joints[j].chanrots[1]==2)&&(kCfg.joints[j].chanrots[2]==1))
	//				kOrder = EulOrdXYZs;//kOrder = EulOrdXZYs;//
	//			else if ((kCfg.joints[j].chanrots[0]==1)&&(kCfg.joints[j].chanrots[1]==0)&&(kCfg.joints[j].chanrots[2]==2))
	//				kOrder = EulOrdZXYs;//kOrder = EulOrdYXZs;//
	//			else if ((kCfg.joints[j].chanrots[0]==1)&&(kCfg.joints[j].chanrots[1]==2)&&(kCfg.joints[j].chanrots[2]==0))
	//				kOrder = EulOrdZYXs;//kOrder = EulOrdYZXs;//
	//			else if ((kCfg.joints[j].chanrots[0]==2)&&(kCfg.joints[j].chanrots[1]==0)&&(kCfg.joints[j].chanrots[2]==1))
	//				kOrder = EulOrdYXZs;//kOrder = EulOrdZXYs;//
	//			else if ((kCfg.joints[j].chanrots[0]==2)&&(kCfg.joints[j].chanrots[1]==1)&&(kCfg.joints[j].chanrots[2]==0))
	//				kOrder = EulOrdYZXs;//kOrder = EulOrdZYXs;//

	//			eulQ = Eul_FromHMatrix( hMatrix,kOrder);

	//			//NOW, since the bvh needs the angles written IN THE ORDER specified here, we will use 
	//			//Ogre::Vector3 p to store them.  Doing the right-hand/left-hand swap at the same time, so Y
	//			//in bvh file means -Z here, and vice versa.
	//			//if (kCfg.joints[j].chanrots[0]==0) p.x = eulQ.x;
	//			//else if (kCfg.joints[j].chanrots[0]==1) p.x = eulQ.y;//eulQ.y;
	//			//else if (kCfg.joints[j].chanrots[0]==2) p.x = eulQ.z;//eulQ.z;
	//			//if (kCfg.joints[j].chanrots[1]==0) p.y = eulQ.x;
	//			//else if (kCfg.joints[j].chanrots[1]==1) p.y = eulQ.y;//eulQ.y;
	//			//else if (kCfg.joints[j].chanrots[1]==2) p.y = eulQ.z;//eulQ.z;
	//			//if (kCfg.joints[j].chanrots[2]==0) p.z = eulQ.x;
	//			//else if (kCfg.joints[j].chanrots[2]==1) p.z = eulQ.y;//eulQ.y;
	//			//else if (kCfg.joints[j].chanrots[2]==2) p.z = eulQ.z;//eulQ.z;

	//			if ((kCfg.joints[j].chanrots[0]==0)&&(kCfg.joints[j].chanrots[1]==1)&&(kCfg.joints[j].chanrots[2]==2))
	//				p.set(eulQ.x,-eulQ.y,-eulQ.z);//p.set(eulQ.x,-eulQ.z,-eulQ.y);
	//			else if ((kCfg.joints[j].chanrots[0]==0)&&(kCfg.joints[j].chanrots[1]==2)&&(kCfg.joints[j].chanrots[2]==1))
	//				p.set(eulQ.x,-eulQ.y,-eulQ.z);//p.set(eulQ.x,-eulQ.z,-eulQ.y);
	//			else if ((kCfg.joints[j].chanrots[0]==1)&&(kCfg.joints[j].chanrots[1]==0)&&(kCfg.joints[j].chanrots[2]==2))
	//				p.set(-eulQ.x,eulQ.y,-eulQ.z);//p.set(-eulQ.z,eulQ.y,-eulQ.x);
	//			else if ((kCfg.joints[j].chanrots[0]==1)&&(kCfg.joints[j].chanrots[1]==2)&&(kCfg.joints[j].chanrots[2]==0))
	//				p.set(-eulQ.x,-eulQ.y,eulQ.z);//p.set(-eulQ.y,-eulQ.x,eulQ.z);
	//			else if ((kCfg.joints[j].chanrots[0]==2)&&(kCfg.joints[j].chanrots[1]==0)&&(kCfg.joints[j].chanrots[2]==1))
	//				p.set(-eulQ.x,eulQ.y,-eulQ.z);//p.set(-eulQ.z,eulQ.y,-eulQ.x);
	//			else if ((kCfg.joints[j].chanrots[0]==2)&&(kCfg.joints[j].chanrots[1]==1)&&(kCfg.joints[j].chanrots[2]==0))
	//				p.set(-eulQ.x,-eulQ.y,eulQ.z);//p.set(-eulQ.y,-eulQ.x,eulQ.z);
	//			fprintf(fpw,"%3.2f %3.2f %3.2f ",mRadToDeg(p.x),mRadToDeg(p.y),mRadToDeg(p.z));
	//			//fprintf(fpw,"%3.2f %3.2f %3.2f ",mRadToDeg(eulQ.x),mRadToDeg(eulQ.y),mRadToDeg(eulQ.z));
	//			
	//			
	//			//eul = mat.toEuler();
	//			////Con::errorf("chanrots %d %d, %d, %d",i,kCfg.joints[j].chanrots[0],kCfg.joints[j].chanrots[1],kCfg.joints[j].chanrots[2]);
	//			//if (kCfg.joints[j].chanrots[0]==0) p.x = eul.x;
	//			//else if (kCfg.joints[j].chanrots[0]==1) p.x = -eul.z;
	//			//else if (kCfg.joints[j].chanrots[0]==2) p.x = -eul.y;
	//			//if (kCfg.joints[j].chanrots[1]==0) p.y = eul.x;
	//			//else if (kCfg.joints[j].chanrots[1]==1) p.y = -eul.z;
	//			//else if (kCfg.joints[j].chanrots[1]==2) p.y = -eul.y;
	//			//if (kCfg.joints[j].chanrots[2]==0) p.z = eul.x;
	//			//else if (kCfg.joints[j].chanrots[2]==1) p.z = -eul.z;
	//			//else if (kCfg.joints[j].chanrots[2]==2) p.z = -eul.y;
	//			//fprintf(fpw,"%3.2f %3.2f %3.2f ",mRadToDeg(p.x),mRadToDeg(p.y),mRadToDeg(p.z));


	//			//m = mat;
	//			//fprintf(fpw,"%3.2f %3.2f %3.2f ",-mRadToDeg(eul.z),mRadToDeg(eul.x),-mRadToDeg(eul.y));
	//			//fprintf(fpw,"%3.2f %3.2f %3.2f ",-mRadToDeg(eul.y),-mRadToDeg(eul.z),mRadToDeg(eul.x));
	//			//fprintf(fpw,"%3.2f %3.2f %3.2f ",-mRadToDeg(eul.z),-mRadToDeg(eul.y),mRadToDeg(eul.x));
	//			//fprintf(fpw,"%3.2f %3.2f %3.2f ",mRadToDeg(eul.x),-mRadToDeg(eul.z),-mRadToDeg(eul.y));
	//			//fprintf(fpw,"%3.2f %3.2f %3.2f ",mRadToDeg(eul.x),-mRadToDeg(eul.y),-mRadToDeg(eul.z));
	//		}
	//		fprintf(fpw,"\n");
	//	}
	//}
	//fclose(fpw);


	//sql->CloseDatabase();
	//delete sql;
}

		////HERE: with known output skeletons, we don't need to do any conversions from our skeleton
		////to theirs for the header:  just copy the entire skeleton, with default offsets and rotations,
		////from a known bvh file.  We have examples (frame data removed) called (name).skeleton.bvh.
		//String skeletonPath = myPath + skeletonName;
		//fps = fopen(skeletonPath.c_str(),"r");
		//if (fps==NULL)
		//{
		//	//Con::errorf("ERROR: can't open skeleton file %s",skeletonPath.c_str());
		//	return;
		//}
		////Now, load the whole file, up to but not including MOTION tag if it's in there.
		//char buf[512];
		//while (fgets(buf,512,fps))
		//{
		//	if (!strncmp(buf,"MOTION",6))
		//		break;

		//	fprintf(fpw,buf);
		//}
//Whoops... doesn't work like that after all, no default pose, the skeleton offsets == first frame of anim.

void fxFlexBody::loadDsq(const char *dsqFile)
{
	////Here:  open the file, and give the shape->importSequences function a filestream, and you're done.
	//FileStream  fileStream;

	//const String myPath = getShapeInstance()->mShapeResource.getPath().getPath();
	//const String myFileName = getShapeInstance()->mShapeResource.getPath().getFileName();
	//const String myFullPath = getShapeInstance()->mShapeResource.getPath().getFullPath();
	//
	//TSShape *kShape = getShapeInstance()->getShape();

	//fileStream.open(dsqFile, Torque::FS::File::Read);

	//if (fileStream.getStatus() != Stream::Ok)
	//{
	//	//Con::errorf("Missing sequence %s",dsqFile);
	//	return;
	//}
	//if (!kShape->importSequences(&fileStream,myPath) || fileStream.getStatus()!= Stream::Ok)
	//{
	//	fileStream.close();
	//	//Con::errorf("Load sequence %s failed",dsqFile);
	//	return;
	//}
	//fileStream.close();

	////TSShapeConstructor::ChangeSet::add( TSShapeConstructor::ChangeSet::Command& cmd )
	//TSShapeConstructor* ctor = TSShapeConstructor::findShapeConstructor( myFullPath );
	//
	////cmd.addArgs( seqName );
	//if (ctor)
	//{
	//	TSShapeConstructor::ChangeSet::Command cmd("addSequence");
	////	//Con::errorf("found my shape constructor!  sequences %d",ctor->mSeqData.size());
	////	ctor->mSequences.increment();//NOPE!  Only for old style sequence0 = "..." syntax
	////	ctor->mSequences[ctor->mSequences.size()-1] = myFileName;//Maybe??
	//	String seqname = kShape->getName(kShape->sequences.last().nameIndex);
	//	String dsqFilename = dsqFile;
	//	String relativePath;
	//	int gamePos = dsqFilename.find("game/art",0);//?? Arbitrary, FIX!
	//	if (gamePos > -1)
	//		relativePath = dsqFilename.substr(gamePos + 5);
	//	else
	//		relativePath = dsqFilename;

	//	//Con::printf("shape constructor, gamePos: %d, adding sequence: %s",gamePos,relativePath.c_str()); 
	//	//relativePath += "\t";
	//	//relativePath += seqname;
	//	//const char relPath[512];
	//	//sprintf( relPath, "%s", StrFind( dsqFile, "game/art",0,0));
	//	
	//	//HERE: figure out how to chop out the main engine path from the full dsqFile path, to
	//	//get only the relative path starting with art/....


	//	//THEN: add " " + seqname to that, and make it argv[0].
	//	cmd.argc = 4;
	//	cmd.argv[0] = relativePath;
	//	cmd.argv[1] = seqname;
	//	cmd.argv[2] = "0";
	//	cmd.argv[3] = "-1";
	//	//cmd.addArgs( "filepath",  "newname" ,  "0",  "-1" );//SYNTAX??
	//	ctor->mChangeSet.add( cmd );
	//}
}

void fxFlexBody::setTarget(fxFlexBody *target)
{
	mTarget = target;
	return;
}


int fxFlexBody::getShapeConstructor()
{
	//const String myFullPath = getShapeInstance()->mShapeResource.getPath().getFullPath();
	//TSShapeConstructor* ctor = TSShapeConstructor::findShapeConstructor( myFullPath );
	//if (ctor)
	//{
	//	return ctor->getId();//server/client? ghost id?
	//} 
	//else 
		return 0;
}

void fxFlexBody::threadInfo(int index)
{
	//Thread& st = mScriptThread[index];
	//TSThread *tsthread = st.thread;
	//Con::errorf("seq: %d, state %d, atEnd %d, forward %d, pos %f, keyNum %d",index,st.state,st.atEnd,st.forward,tsthread->getPos(),tsthread->getKeyframeNumber());
}



void fxFlexBody::fixBvhCfg(const char *cfg_filename,const char *bvh_filename)
{
	//bvhCfgData kCfg;
	//char buf[255];
	//String oldFile,newFile,bvhFile;
	//int profile_id = 1;//FIX!  But later, this function is broken anyway

	//Ogre::Quaternion finalRots[MAX_BVH_NODES],finalTPoseRots[MAX_BVH_NODES],qF,invQF;
	//int bvhParents[MAX_BVH_NODES];
	//Ogre::Vector3 bvhGoalAxes[5],bvhGlobalAxes[6],bvhGlobalNormals[6];//FIX - MAX_BVH_NODE_GROUPS?
	//Ogre::Vector3 finalAxis,finalNorm,defaultFinalAxis,defaultFinalNorm;
	//int bvhGroupAxes[5];

	//TSShape *kShape = getShapeInstance()->getShape();

	//oldFile.insert(0,cfg_filename);
	//newFile.insert(0,cfg_filename);
	//bvhFile.insert(0,bvh_filename);
	////newFile.insert(newFile.length()-4,".new");

	//FILE *cfg = NULL;
	////cfg = fopen(oldFile.c_str(),"r");cfg_filename
	//cfg = fopen(cfg_filename,"r");
	//if (!cfg) { //Con::errorf("couldn't open cfg file: %s",cfg_filename); return; }
	//loadBvhCfg(cfg,&kCfg,profile_id);//FIX!!  Need bvhProfile.
	//fclose(cfg);

	//FILE *bvh = NULL;
	//bvh = fopen(bvhFile.c_str(),"r");
	//if (!bvh) { //Con::errorf("couldn't open bvh skeleton file: %s",bvh_filename); return; }
	//loadBvhSkeleton(bvh,&kCfg,profile_id);//FIX!  profile_id
	//fclose(bvh);

	//int lastGroup;

	//bvhGoalAxes[0].set(0.0,0.0,1.0);//main body
	//bvhGoalAxes[1].set(-1.0,0.0,0.0);//left arm
	//bvhGoalAxes[2].set(1.0,0.0,0.0);//right arm
	//bvhGoalAxes[3].set(0.0,0.0,-1.0);//left leg
	//bvhGoalAxes[4].set(0.0,0.0,-1.0);//right leg

	////X,-X,Y,-Y,Z,-Z
	//bvhGlobalAxes[0].set(1.0,0.0,0.0);
	//bvhGlobalAxes[1].set(-1.0,0.0,0.0);
	//bvhGlobalAxes[2].set(0.0,1.0,0.0);
	//bvhGlobalAxes[3].set(0.0,-1.0,0.0);
	//bvhGlobalAxes[4].set(0.0,0.0,1.0);
	//bvhGlobalAxes[5].set(0.0,0.0,-1.0);

	////Now, for one axis at ninety degrees to our main axis - picked these arbitrarily.
	//bvhGlobalNormals[0].set(0.0,0.0,1.0);
	//bvhGlobalNormals[1].set(0.0,0.0,1.0);
	//bvhGlobalNormals[2].set(1.0,0.0,0.0);
	//bvhGlobalNormals[3].set(1.0,0.0,0.0);
	//bvhGlobalNormals[4].set(0.0,1.0,0.0);
	//bvhGlobalNormals[5].set(0.0,1.0,0.0);

	////bvhPoseRotsA, "kCfg.axesFixRotsA[rc].x = mDegToRad(FAx);"
	////Con::printf("bvh num nodes: %d",kCfg.numBvhNodes);

	///////////////////////////////////////////////////////////
	////First, do a scanning pass, to establish whether this model is all global or not.
	//bool allGlobal = true;
	//lastGroup = -1;
	//for (unsigned int j=0;j<kCfg.numBvhNodes;j++)
	//{
	//	Ogre::Vector3 childPos,childFinal,goalPos;
	//	Ogre::Matrix3 matF;
	//	Ogre::Vector3 eul;

	//	int childID = kShape->nodes[kCfg.dtsNodes[j]].firstChild;
	//	//NOTE: this parentID is from the dts skeleton, it may not point us at the node
	//	//immediately above us in the bvh chain.  In other words, the bvh may have skipped
	//	//some nodes.  But they might have their own rotations which need to be included.
	//	int parentID = kShape->nodes[kCfg.dtsNodes[j]].parentIndex;
	//	int nodeGroup = kCfg.nodeGroups[j];
	//	Quat16 rot = kShape->defaultRotations[kCfg.dtsNodes[j]];
	//	Ogre::Quaternion rotF = rot.getOgre::Quaternion();
	//	if (!(rotF.isIdentity()))
	//		allGlobal = false;

	//	if (lastGroup !=  nodeGroup)
	//	{//starting a new group.
	//		switch (nodeGroup)
	//		{//Pick a reasonable child node to grab a position from, to clue us in on which way
	//			//the main mesh is pointing.
	//		case 0:
	//			childPos = kShape->defaultTranslations[childID];
	//			break;
	//		case 1:
	//			childPos = kShape->defaultTranslations[kShape->nodes[childID].firstChild];
	//			break;
	//		case 2:
	//			childPos = kShape->defaultTranslations[kShape->nodes[childID].firstChild];
	//			break;
	//		case 3:
	//			childPos = kShape->defaultTranslations[childID];
	//			break;
	//		case 4:
	//			childPos = kShape->defaultTranslations[childID];
	//		}
	//		childPos.normalize();
	//		Ogre::Vector3 goalAxis = bvhGoalAxes[nodeGroup];
	//		Ogre::Vector3 crossProd = mCross(childPos,goalAxis);
	//		float dotProd = mDot(childPos,goalAxis);
	//		if (dotProd<0.95)
	//			allGlobal = false;

	//		//HERE, figure out which node is my parent (as 0 - numBvhNodes) and figure out my final rotation
	//		if (j>0)
	//		{
	//			for (unsigned int jj=0; jj<j; jj++)
	//			{
	//				if (kCfg.dtsNodes[jj]==parentID)
	//					bvhParents[j] = jj;
	//			}
	//			if (bvhParents[j]>=0)
	//			{
	//				//HERE: DEAL WITH SKIPPED PARENTS
	//				//If nodes in the dts hierarchy are not reflected in the bvh hierarchy
	//				//but they do have non-identity rotations, then you need those rotations.
	//				int tempParent = parentID;
	//				int count = 0;
	//				Ogre::Quaternion quats[20];//for now limit skipped nodes to twenty, should be plenty
	//				while (tempParent != kCfg.dtsNodes[kCfg.joints[j].parent]) 
	//				{
	//					rot = kShape->defaultRotations[tempParent];
	//					quats[count] = rot.getOgre::Quaternion();
	//					tempParent = kShape->nodes[tempParent].parentIndex;
	//					count++;
	//				}
	//				finalRots[j] = Ogre::Quaternion::IDENTITY;
	//				finalRots[j] *= rotF;
	//				for (int k = (count-1); k>=0; k--)
	//				{
	//					finalRots[j] *= quats[k];
	//				}
	//				finalRots[j] *= finalRots[bvhParents[j]];
	//			}
	//			else 
	//			{
	//				//Con::errorf("fix Cfg PROBLEM: a new group can not find its parent node!");
	//				finalRots[j] = rotF;
	//			}
	//		} else {
	//			if (parentID == -1)
	//				finalRots[j] = rotF;//root node, finalRots[0]
	//			else 
	//			{
	//				int tempParent = parentID;
	//				int count = 0;
	//				Ogre::Quaternion quats[20];//for now limit skipped nodes to twenty, should be plenty
	//				while (tempParent != -1) 
	//				{
	//					rot = kShape->defaultRotations[tempParent];
	//					quats[count] = rot.getOgre::Quaternion();
	//					tempParent = kShape->nodes[tempParent].parentIndex;
	//					count++;
	//				}
	//				finalRots[j] = Ogre::Quaternion::IDENTITY;
	//				finalRots[j] *= rotF;
	//				for (int k = (count-1); k>=0; k--)
	//				{
	//					finalRots[j] *= quats[k];
	//				}

	//			}
	//		}
	//	} else {
	//		bvhParents[j] = j-1;
	//		//HERE: DEAL WITH SKIPPED PARENTS
	//		int tempParent = parentID;
	//		int count = 0;
	//		Ogre::Quaternion quats[20];//for now limit skipped nodes to twenty, should be plenty
	//		while (tempParent != kCfg.dtsNodes[kCfg.joints[j].parent]) 
	//		{
	//			rot = kShape->defaultRotations[tempParent];
	//			quats[count] = rot.getOgre::Quaternion();
	//			tempParent = kShape->nodes[tempParent].parentIndex;
	//			count++;
	//		}
	//		finalRots[j] = Ogre::Quaternion::IDENTITY;
	//		finalRots[j] *= rotF;
	//		for (int k = (count-1); k>=0; k--)
	//		{
	//			finalRots[j] *= quats[k];
	//		}
	//		finalRots[j] *= finalRots[bvhParents[j]];
	//		//finalRots[j].mul( rotF, finalRots[bvhParents[j]]);//finalRots[bvhParents[j]],rotF
	//	}
	//	Ogre::Vector3 p;
	//	finalRots[j].mulP(Ogre::Vector3(1,0,0),&p);
	//	//Con::errorf("(1,0,0) through finalRots[%d]: %3.2f %3.2f %3.2f",j,p.x,p.y,p.z);
	//	lastGroup = nodeGroup;
	//}
	//if (allGlobal) {
	//	//Con::printf("Congratulations, your model is All Global, no changes need to be made to the default.cfg file!");
	//	return;
	//}
	//else //Con::printf("model is not All Global!");


	///////////////////////////////////////////////////////////
	////Now, do the real pass where we fix the cfg file.
	//FILE *newCfg = fopen(newFile.c_str(),"w");
	//lastGroup = -1;
	//for (unsigned int j=0;j<kCfg.numBvhNodes;j++)
	//{
	//	Ogre::Vector3 childPos,childPosNorm,childFinal,goalPos,goalAxis,globalAxis,globalNormal;
	//	int childID = kShape->nodes[kCfg.dtsNodes[j]].firstChild;
	//	int parentID = kShape->nodes[kCfg.dtsNodes[j]].parentIndex;
	//	int nodeGroup = kCfg.nodeGroups[j];
	//	Quat16 rot = kShape->defaultRotations[kCfg.dtsNodes[j]];
	//	Ogre::Quaternion q1,q2,rotF;

	//	rotF = rot.getOgre::Quaternion();
	//	//HERE if we have more parents, wrap them into rotF before we take the inverse
	//	int tempParent = parentID;
	//	int count = 0;
	//	Ogre::Quaternion quats[20];//for now limit skipped nodes to twenty, should be plenty
	//	while (tempParent != kCfg.dtsNodes[kCfg.joints[j].parent]) 
	//	{
	//		rot = kShape->defaultRotations[tempParent];
	//		quats[count] = rot.getOgre::Quaternion();
	//		tempParent = kShape->nodes[tempParent].parentIndex;
	//		count++;
	//	}

	//	for (int k = (count-1); k>=0; k--)
	//	{
	//		rotF *= quats[k];
	//	}
	//	

	//	Ogre::Quaternion invRotF = rotF;
	//	invRotF.inverse();

	//	Ogre::Matrix3 invMatF;
	//	invRotF.setMatrix(&invMatF);
	//	Ogre::Vector3 eul = invMatF.toEuler();
	//	
	//	eul *= 180.0/M_PI;
	//	//First, reverse whatever local transforms currently exist.
	//	kCfg.bvhPoseRotsA[j] = eul;

	//	if (lastGroup !=  nodeGroup)
	//	{//starting a new group.
	//		switch (nodeGroup)
	//		{//Do this again.
	//		case 0:
	//			childPos = kShape->defaultTranslations[childID];
	//			break;
	//		case 1:
	//			childPos = kShape->defaultTranslations[kShape->nodes[childID].firstChild];
	//			break;
	//		case 2:
	//			childPos = kShape->defaultTranslations[kShape->nodes[childID].firstChild];
	//			break;
	//		case 3:
	//			childPos = kShape->defaultTranslations[childID];
	//			break;
	//		case 4:
	//			childPos = kShape->defaultTranslations[childID];
	//		}
	//		childPos.normalize();

	//		goalAxis = bvhGoalAxes[nodeGroup];

	//		bvhGroupAxes[nodeGroup] = -1;
	//		for (unsigned int k=0;k<6;k++)
	//		{
	//			if (mDot(childPos,bvhGlobalAxes[k])>0.9)
	//				bvhGroupAxes[nodeGroup]=k;
	//			else
	//			{
	//				float wrongDot = mDot(childPos,bvhGlobalAxes[k]);
	//				float t = 0.0;
	//			}
	//		}
	//		
	//		if (bvhGroupAxes[nodeGroup]>-1)
	//		{
	//			globalAxis = bvhGlobalAxes[bvhGroupAxes[nodeGroup]];
	//			globalNormal = bvhGlobalNormals[bvhGroupAxes[nodeGroup]];

	//			if (j==0)//root node
	//			{
	//				qF.rotationArc(goalAxis,globalAxis);
	//				if (!qF.isIdentity())
	//				{
	//					Ogre::Matrix3 mF;
	//					qF.setMatrix(&mF);
	//					Ogre::Vector3 eulF = mF.toEuler();
	//					eulF *= 180.0/M_PI;
	//					kCfg.bvhPoseRotsB[j] = eulF;
	//				} else {//directly opposite,
	//					//Just hard code it depending on axis.
	//					if (mDot(finalAxis,goalAxis)<-0.95)
	//					{
	//						Ogre::Vector3 eulF,eulRadF;
	//						if ((bvhGroupAxes[nodeGroup]==0)||(bvhGroupAxes[nodeGroup]==1))
	//							eulF.set(0.0,180.0,0.0);
	//						else if ((bvhGroupAxes[nodeGroup]==2)||(bvhGroupAxes[nodeGroup]==3))
	//							eulF.set(180.0,0.0,0.0);
	//						else if ((bvhGroupAxes[nodeGroup]==4)||(bvhGroupAxes[nodeGroup]==5))
	//							eulF.set(0.0,180.0,0.0);
	//						eulRadF = eulF * M_PI/180.0;
	//						qF.set(eulRadF);
	//						kCfg.bvhPoseRotsB[j] = eulF;
	//					}
	//				}
	//			} else {//the rest of the body
	//				finalRots[j].mulP(globalAxis,&defaultFinalAxis);
	//				finalRots[j].mulP(globalNormal,&defaultFinalNorm);
	//				finalTPoseRots[bvhParents[j]].mulP(globalAxis,&finalAxis);
	//				finalTPoseRots[bvhParents[j]].mulP(globalNormal,&finalNorm);

	//				//Con::errorf("%d dts %d,new group mDot: %f  finalAxis %3.2f %3.2f %3.2f  finalNorm %3.2f %3.2f %3.2f parentspace final axis  %3.2f %3.2f %3.2f parentspace final norm %3.2f %3.2f %3.2f ",
	//					j,kCfg.dtsNodes[j],mDot(defaultFinalAxis,goalAxis),
	//					defaultFinalAxis.x,defaultFinalAxis.y,defaultFinalAxis.z,
	//					defaultFinalNorm.x,defaultFinalNorm.y,defaultFinalNorm.z,
	//					finalAxis.x,finalAxis.y,finalAxis.z,
	//					finalNorm.x,finalNorm.y,finalNorm.z);
	//				qF.rotationArc(goalAxis,finalAxis);
	//				if (!qF.isIdentity())
	//				{
	//					qF;
	//					Ogre::Matrix3 mF;
	//					qF.setMatrix(&mF);
	//					Ogre::Vector3 eulF = mF.toEuler();
	//					eulF *= 180.0/M_PI;
	//					kCfg.bvhPoseRotsB[j] = eulF;
	//				} else {//directly opposite, dot product at or near -1.0, no solution from rotationArc
	//					//Just hard code it depending on axis.
	//					if (mDot(finalAxis,goalAxis)<-0.95)
	//					{
	//						Ogre::Vector3 eulF,eulRadF;
	//						if ((bvhGroupAxes[nodeGroup]==0)||(bvhGroupAxes[nodeGroup]==1))
	//							eulF.set(0.0,180.0,0.0);
	//						else if ((bvhGroupAxes[nodeGroup]==2)||(bvhGroupAxes[nodeGroup]==3))
	//							eulF.set(180.0,0.0,0.0);
	//						else if ((bvhGroupAxes[nodeGroup]==4)||(bvhGroupAxes[nodeGroup]==5))
	//							eulF.set(0.0,180.0,0.0);
	//						eulRadF = eulF * M_PI/180.0;
	//						qF.set(eulRadF);
	//						kCfg.bvhPoseRotsB[j] = eulF;
	//					}
	//				}
	//			}
	//			//HERE:  now we need to decide whether to take advice from the defaultRots
	//			//or not.  If mDot is above a certain threshold, then do it.
	//			if (mDot(defaultFinalAxis,goalAxis)>0.75)
	//			{//Now, we need to use our new bvhGlobalNormals array, to get an idea which way
	//				//we need to rotate around our central axis.
	//				Ogre::Vector3 finalNorm;
	//				Ogre::Quaternion q1,q2;
	//				q1.mul(qF,finalTPoseRots[bvhParents[j]]);
	//				q1.mulP(globalNormal,&finalNorm);
	//				q1.mulP(globalAxis,&finalAxis);
	//				//Con::printf("final axis %3.2f %3.2f %3.2f, final norm %3.2f %3.2f %3.2f, def final norm %3.2f %3.2f %3.2f",
	//					finalAxis.x,finalAxis.y,finalAxis.z,
	//					finalNorm.x,finalNorm.y,finalNorm.z,defaultFinalNorm.x,defaultFinalNorm.y,defaultFinalNorm.z);

	//				//HERE: compare (mDot) between the defaultFinalNorm and four ideal positions, rotated around our 
	//				//central bodypart axis.  Take one unit vector and rotate it three times.
	//				if (mDot(finalNorm,defaultFinalNorm)<0.75)
	//				{//	otherwise skip this step, we're already perfect.
	//					Ogre::Vector3 testNorms[4];
	//					testNorms[0] = globalNormal;
	//					Ogre::Quaternion rot90(globalAxis,M_PI/2.0);
	//					Ogre::Quaternion rotBodyAxisFix;
	//					Ogre::Vector3 temp;
	//					for (unsigned int k=1; k<4; k++)
	//					{//Start by doing the rotations in global space.
	//						rot90.mulP(testNorms[k-1],&temp);
	//						testNorms[k] = temp;
	//					}
	//					for (unsigned int k=0; k<4; k++)
	//					{//Now move the results to final positions.
	//						q1.mulP(testNorms[k],&temp);
	//						testNorms[k] = temp;
	//						//Con::printf("testNorms %d: %f %f %f",k,temp.x,temp.y,temp.z);
	//					}
	//					for (unsigned int k=0; k<4; k++)
	//					{
	//						if (mDot(defaultFinalNorm,testNorms[k])>0.75)
	//						{
	//							rotBodyAxisFix.set(globalAxis,(M_PI/2.0)*k);
	//						}
	//					}
	//					Ogre::Quaternion tempQ;
	//					tempQ.mul(rotBodyAxisFix,qF);
	//					qF = tempQ;
	//					//qF *= rotBodyAxisFix;
	//					Ogre::Matrix3 mF;
	//					qF.setMatrix(&mF);
	//					Ogre::Vector3 eulF = mF.toEuler();
	//					eulF *= 180.0/M_PI;
	//					kCfg.bvhPoseRotsB[j] = eulF;
	//				} else //Con::printf("T Pose Rot comes out close enough to default rot, not modifying.");
	//			}
	//		}
	//	} else {//just a normal node, not starting a new group.
	//		qF = Ogre::Quaternion::IDENTITY;
	//		bvhParents[j] = j-1;
	//		Ogre::Vector3 globalNormal = bvhGlobalNormals[bvhGroupAxes[nodeGroup]];

	//		finalRots[j].mulP(globalAxis,&defaultFinalAxis);
	//		finalRots[j].mulP(globalNormal,&defaultFinalNorm);
	//		finalTPoseRots[bvhParents[j]].mulP(globalAxis,&finalAxis);
	//		finalTPoseRots[bvhParents[j]].mulP(globalNormal,&finalNorm);

	//		//Con::errorf("%d dts %d, normal node. mDot: %f  finalAxis %3.2f %3.2f %3.2f  finalNorm %3.2f %3.2f %3.2f parentspace final axis  %3.2f %3.2f %3.2f parentspace final norm %3.2f %3.2f %3.2f ",
	//			j,kCfg.dtsNodes[j],mDot(defaultFinalAxis,goalAxis),
	//			defaultFinalAxis.x,defaultFinalAxis.y,defaultFinalAxis.z,
	//			defaultFinalNorm.x,defaultFinalNorm.y,defaultFinalNorm.z,
	//			finalAxis.x,finalAxis.y,finalAxis.z,
	//			finalNorm.x,finalNorm.y,finalNorm.z);
	//		
	//		kCfg.bvhPoseRotsB[j] = Ogre::Vector3::ZERO;
	//		//HERE:  now we need to decide whether to take advice from the defaultRots
	//		//or not.  If mDot is above a certain threshold, then do it.
	//		if (mDot(defaultFinalAxis,goalAxis)>0.75)
	//		{//Now, we need to use our new bvhGlobalNormals array, to get an idea which way
	//			//we need to rotate around our central axis.
	//			Ogre::Vector3 finalNorm;
	//			Ogre::Quaternion q1,q2;
	//			q1.mul(qF,finalTPoseRots[bvhParents[j]]);
	//			q1.mulP(globalNormal,&finalNorm);
	//			q1.mulP(globalAxis,&finalAxis);
	//			//Con::printf("mDot(normals) %f final axis %3.2f %3.2f %3.2f, final norm %3.2f %3.2f %3.2f, def final axis %3.2f %3.2f %3.2f def final norm %3.2f %3.2f %3.2f",
	//				mDot(finalNorm,defaultFinalNorm),finalAxis.x,finalAxis.y,finalAxis.z,finalNorm.x,finalNorm.y,finalNorm.z,
	//				defaultFinalAxis.x,defaultFinalAxis.y,defaultFinalAxis.z,defaultFinalNorm.x,defaultFinalNorm.y,defaultFinalNorm.z);

	//			//HERE: compare (mDot) between the defaultFinalNorm and four ideal positions, based around our 
	//			//central axis.  Take one unit vector and rotate it three times.
	//			if (mDot(finalNorm,defaultFinalNorm)<0.75)
	//			{//	if mDot>0.75 then skip this whole step, we're already close enough.
	//				//Con::printf("we're going to try to get some help from default rotations.");
	//				Ogre::Vector3 testNorms[4];
	//				testNorms[0] = globalNormal;
	//				Ogre::Quaternion rot90(globalAxis,M_PI/2.0);
	//				Ogre::Quaternion rotBodyAxisFix;
	//				Ogre::Vector3 temp;
	//				for (unsigned int k=1; k<4; k++)
	//				{//Start by doing the rotations in global space.
	//					rot90.mulP(testNorms[k-1],&temp);
	//					testNorms[k] = temp;
	//				}
	//				for (unsigned int k=0; k<4; k++)
	//				{//Now move the results to final positions.
	//					q1.mulP(testNorms[k],&temp);
	//					testNorms[k] = temp;
	//					//Con::printf("testNorms %d: %f %f %f",k,temp.x,temp.y,temp.z);
	//				}
	//				for (unsigned int k=0; k<4; k++)
	//				{
	//					if (mDot(defaultFinalNorm,testNorms[k])>0.75)
	//					{
	//						rotBodyAxisFix.set(globalAxis,(M_PI/2.0)*k);
	//					}
	//				}
	//				Ogre::Quaternion tempQ;
	//				tempQ.mul(rotBodyAxisFix,qF);
	//				qF = tempQ;
	//				//qF *= rotBodyAxisFix;
	//				Ogre::Matrix3 mF;
	//				qF.setMatrix(&mF);
	//				Ogre::Vector3 eulF = mF.toEuler();
	//				eulF *= 180.0/M_PI;
	//				kCfg.bvhPoseRotsB[j] = eulF;
	//			}
	//		} else //Con::printf("T Pose Rot comes out close enough to default rot, not modifying.");
	//	} 

	//	if (j>0)
	//		finalTPoseRots[j].mul(qF,finalTPoseRots[bvhParents[j]]);//invQF,finalTPoseRots?/
	//	else
	//		finalTPoseRots[j] = qF;

	//	Ogre::Quaternion invFinal = finalTPoseRots[j];
	//	Ogre::Matrix3 matInvFinal;
	//	invFinal.setMatrix(&matInvFinal);
	//	matInvFinal.affineInverse();
	//	Ogre::Vector3 eulInvFinal = matInvFinal.toEuler();
	//	eulInvFinal *= 180.0/M_PI;

	//	kCfg.axesFixRotsA[j] = eulInvFinal;
	//	kCfg.axesFixRotsB[j] = Ogre::Vector3::ZERO;

	//	lastGroup = nodeGroup;		
	//}

	//saveBvhCfg(newCfg,&kCfg);

	//fclose(newCfg);
}

void fxFlexBody::adjustBaseNodePosRegion(unsigned int seq, Ogre::Vector3 &pos, float start, float stop)
{
	//int startFrame,stopFrame;

	//TSShape *kShape = getShapeInstance()->getShape();
	//TSShape::Sequence *kSeq = &(kShape->sequences[seq]);

	//startFrame = (int)((float)kSeq->numKeyframes * start);
	//stopFrame = (int)((float)kSeq->numKeyframes * stop);

	//for (unsigned int i=startFrame;i<stopFrame;i++)
	//{
	//	Ogre::Vector3 oldpos = kShape->nodeTranslations[i+kSeq->baseTranslation];//+node	
	//	Ogre::Vector3 newpos = oldpos + pos;
	//	kShape->nodeTranslations[i+kSeq->baseTranslation] = newpos;
	//}
}

void fxFlexBody::setBaseNodePosRegion(unsigned int seq, Ogre::Vector3 &pos, float start, float stop)
{
	//////Con::errorf("setting base node pos region: %f %f %f, start %f stop %f",pos.x,pos.y,pos.z,start,stop);
	//int startFrame,stopFrame;

	//TSShape *kShape = getShapeInstance()->getShape();
	//TSShape::Sequence *kSeq = &(kShape->sequences[seq]);

	//startFrame = (int)((float)kSeq->numKeyframes * start);
	//stopFrame = (int)((float)kSeq->numKeyframes * stop);

	//for (unsigned int i=startFrame;i<stopFrame;i++)
	//{
	//	Ogre::Vector3 oldpos = kShape->nodeTranslations[i+kSeq->baseTranslation];//+node	
	//	kShape->nodeTranslations[i+kSeq->baseTranslation] = pos;
	//}

}

void fxFlexBody::adjustNodeRotRegion(unsigned int seq, unsigned int node, Ogre::Vector3 &rot, float start, float stop)
{

	//int rot_matters_count,node_matter_index;
	//Quat16 q16,nq16;
	//int startFrame,stopFrame;

	//rot_matters_count = 0;

	//rot.x = mDegToRad(rot.x);
	//rot.y = mDegToRad(rot.y);
	//rot.z = mDegToRad(rot.z);

	//Ogre::Quaternion q(rot);
	////q16.set(q);

	//TSShape *kShape = getShapeInstance()->getShape();
	//TSShape::Sequence *kSeq = &(kShape->sequences[seq]);

	//startFrame = (int)((float)kSeq->numKeyframes * start);
	//stopFrame = (int)((float)kSeq->numKeyframes * stop);

	//
	//if (mBodyParts[0]->mNodeIndex == node)
	//{
	//	//Con::printf("adjusting BASE node %d rotation from %d to %d",node,startFrame,stopFrame);
	//	Ogre::Vector3 oldPos,newPos;
	//	for (unsigned int i=startFrame;i<stopFrame;i++)
	//	{
	//		oldPos = kShape->nodeTranslations[i+kSeq->baseTranslation];
	//		q.mulP(oldPos,&newPos);
	//		kShape->nodeTranslations[i+kSeq->baseTranslation] = newPos;
	//	}
	//}

	//if (!kSeq->rotationMatters.test(node)) { 
	//	//Con::errorf("Node %d doesn't matter!",node); 
	//	return; 
	//}

	//for (unsigned int j=0;j<kShape->nodes.size();j++) 
	//{
	//	if (j==node) node_matter_index = rot_matters_count;
	//	if (kSeq->rotationMatters.test(j))
	//		rot_matters_count++;
	//}
	//for (unsigned int i=startFrame;i<stopFrame;i++)
	//{
	//	q16 = kShape->nodeRotations[kSeq->baseRotation+(node_matter_index*kSeq->numKeyframes)+i];
	//	Ogre::Quaternion temp;
	//	q16.getOgre::Quaternion(&temp);
	//	temp *= q;
	//	nq16.set(temp);
	//	kShape->nodeRotations[kSeq->baseRotation+(node_matter_index*kSeq->numKeyframes)+i] = nq16;
	//}
}

void fxFlexBody::setNodeRotRegion(unsigned int seq, unsigned int node, Ogre::Vector3 &rot, float start, float stop)
{
	//int rot_matters_count,node_matter_index;
	//int startFrame,stopFrame;

	//rot_matters_count = 0;

	//rot.x = mRadToDeg(rot.x);
	//rot.y = mRadToDeg(rot.y);
	//rot.z = mRadToDeg(rot.z);

	//Ogre::Quaternion q(rot);
	//Quat16 q16;
	//q16.set(q);

	//TSShape *kShape = getShapeInstance()->getShape();
	//TSShape::Sequence *kSeq = &(kShape->sequences[seq]);

	//startFrame = (int)((float)kSeq->numKeyframes * start);
	//stopFrame = (int)((float)kSeq->numKeyframes * stop);

	//if (!kSeq->rotationMatters.test(node)) { 
	//	//Con::errorf("Node %d doesn't matter!",node); 
	//	return; 
	//}
	//for (unsigned int j=0;j<kShape->nodes.size();j++) 
	//{
	//	if (j==node) node_matter_index = rot_matters_count;
	//	if (kSeq->rotationMatters.test(j)) 
	//		rot_matters_count++;
	//}
	////for (unsigned int i=0;i<kShape->sequences[seq].numKeyframes;i++)
	//for (unsigned int i=startFrame;i<stopFrame;i++)
	//{
	//	kShape->nodeRotations[kSeq->baseRotation+(node_matter_index*kSeq->numKeyframes)+i] = q16;
	//}
}

void fxFlexBody::addNodeSetRot(unsigned int node, Ogre::Vector3 &rot)
{
	//unsigned int done = 0;
	//if (mNodeSetRots.size())
	//{
	//	for (unsigned int i=0;i<mNodeSetRots.size();i++)
	//	{
	//		if (mNodeSetRots[i].node==node)
	//		{
	//			mNodeSetRots[i].rot = rot;
	//			done = 1;
	//			break;
	//		}
	//	}
	//}
	//if (!done) 
	//{
	//	mNodeSetRots.increment();
	//	mNodeSetRots[mNodeSetRots.size()].node = node;
	//	mNodeSetRots[mNodeSetRots.size()].rot = rot;
	//	//Con::errorf("added node set rot: node %d rot %f %f %f",node,rot.x,rot.y,rot.z);
	//}
}

void fxFlexBody::addNodeAdjustRot(unsigned int node, Ogre::Vector3 &rot)
{
	//unsigned int done = 0;
	//if (mNodeAdjustRots.size())
	//{
	//	for (unsigned int i=0;i<mNodeAdjustRots.size();i++)
	//	{
	//		if (mNodeAdjustRots[i].node==node)
	//		{
	//			mNodeAdjustRots[i].rot = rot;
	//			done = 1;
	//			break;
	//		}
	//	}
	//}
	//if (!done) 
	//{
	//	mNodeAdjustRots.increment();
	//	mNodeAdjustRots[mNodeAdjustRots.size()].node = node;
	//	mNodeAdjustRots[mNodeAdjustRots.size()].rot = rot;

	//	//Con::errorf("added node adjust rot: node %d rot %f %f %f",node,rot.x,rot.y,rot.z); 
	//}
}


void fxFlexBody::doMatrixFix(unsigned int seq, Ogre::Vector3 &eul1,Ogre::Vector3 &eul2)
{
	
	//TSShape *kShape = getShapeInstance()->getShape();
	//TSShape::Sequence *kSeq = &(kShape->sequences[seq]);
	//Ogre::Matrix3 m,mQ,matAxesFixA,matAxesFixB,matAxesFix,matAxesUnfix;
	//Ogre::Quaternion q,qM;
	//Quat16 q16,q16Final;
	//int rot_matters_count = 0;

	//for (unsigned int j=0;j<kShape->nodes.size();j++) 
	//	if (kSeq->rotationMatters.test(j)) 
	//		rot_matters_count++;

	//matAxesFixA.set(eul1);
	//matAxesFixB.set(eul2);
	////matAxesFix.set(Ogre::Vector3(90,0,180));
	//matAxesFix.mul(matAxesFixA,matAxesFixB);//(matAxesFixB,matAxesFixA);//
	//matAxesUnfix = matAxesFix;
	//matAxesUnfix.inverse();

	//for (unsigned int i=0;i<kSeq->numKeyframes;i++)
	//{
	//	for (unsigned int j=0;j<rot_matters_count;j++)
	//	{
	//		m = Ogre::Quaternion::IDENTITY;

	//		m.mul(matAxesFix);
	//		q16 = kShape->nodeRotations[kSeq->baseRotation+(j*kSeq->numKeyframes)+i];
	//		q = q16.getOgre::Quaternion();
	//		q.setMatrix(&mQ);
	//		m.mul(mQ);
	//		m.mul(matAxesUnfix);

	//		Ogre::Quaternion qM(m);
	//		q16Final.set(qM);

	//		kShape->nodeRotations[kSeq->baseRotation+(j*kSeq->numKeyframes)+i] = q16Final;
	//	}
	//}
}
//void fxFlexBody::addUltraframeSet(int seq)
//{
//	for (unsigned int i=0;i<mUltraframeSets.size();i++)
//		if (mUltraframeSets[i].seq==seq)
//			return;//Just a sanity check, shouldn't ever happen.
//
//	TSShape *kShape = getShapeInstance()->getShape();
//
//	mUltraframeSets.increment();
//	int set = mUltraframeSets.size()-1;
//
//	mUltraframeSets[set].seq = seq;
//
//	mUltraframeSets[set].frames.increment();
//	mUltraframeSets[set].frames[0].frame = 0;
//	mUltraframeSets[set].frames[0].adjustBaseNodePos = Ogre::Vector3::ZERO;
//
//	mUltraframeSets[set].frames.increment();
//	mUltraframeSets[set].frames[1].frame = kShape->sequences[seq].numKeyframes-1;
//	mUltraframeSets[set].frames[1].adjustBaseNodePos = Ogre::Vector3::ZERO;
//	//There, two frames automatically created, book-ending the entire sequence.
//	//Now, the trick is to call this function for all the sequences present on the shape, since we won't 
//	//know we're a shapebase way down in TSShapeConstructor.  Do the setup in onWake for the gui, but don't 
//	//erase them when you leave the bot to go to another -- leave it set up.
//	//Con::errorf("added UltraframeSet %d, keyframes %d!",seq,mUltraframeSets[set].frames.size());
//
//}

void fxFlexBody::backupSequenceData()
{
	//TSShape *kShape = getShapeInstance()->getShape();

	//backupTranslations.clear();
	//backupTranslations.setSize(kShape->nodeTranslations.size());	
	//for (unsigned int i=0;i<kShape->nodeTranslations.size();i++)
	//	backupTranslations[i] = kShape->nodeTranslations[i];
	//
	//backupRotations.clear();
	//backupRotations.setSize(kShape->nodeRotations.size());
	////HERE:  STOP!!! This is not only duplicating all sequence data whether or not it has any morphs on it.  It is
	////also doing this PER INSTANCE of the shape, even though they share the same sequence data.
	//for (unsigned int i=0;i<kShape->nodeRotations.size();i++)
	//	backupRotations[i] = kShape->nodeRotations[i];
}

void fxFlexBody::addUltraframeSet(int seq)
{//going obsolete, rely on seqName versions below.
	//for (unsigned int i=0;i<mUltraframeSets.size();i++)
	//	if (mUltraframeSets[i].seq==seq)
	//		return;//Just a sanity check, shouldn't ever happen.

	//mUltraframeSets.increment();
	//int set = mUltraframeSets.size()-1;

	//mUltraframeSets[set].seq = seq;

	////Fill array of beginnings of ultraframe lists with null.
	//for (unsigned int i=0;i<ULTRAFRAME_TYPES;i++)
	//	for (unsigned int j=0;j<MAX_ULTRAFRAME_NODES;j++)
	//		mUltraframeSets[set].lists[i][j] = NULL;

	////clear the bool arrays (used for early out, to speed up searching)
	//for (unsigned int i=0;i<ULTRAFRAME_TYPES;i++)
	//	mUltraframeSets[set].types[i] = false;
	//for (unsigned int j=0;j<MAX_ULTRAFRAME_NODES;j++)
	//	mUltraframeSets[set].nodes[j] = false;
}

//void fxFlexBody::dropUltraframeSet(int seq)
//{//HERE: now, WITHOUT assuming that all sequences have ultraframesets, delete only if necessary.
	//int set = -1;	
	//for (unsigned int i=0;i<mUltraframeSets.size();i++)
	//{
	//	if (mUltraframeSets[i].seq == seq)
	//		set = i;
	//}
	//if (set>=0)
	//	mUltraframeSets.erase(set);
//}

void fxFlexBody::dropUltraframeSet(Ogre::String &seqName)
{
	//int set = -1;	
	//for (unsigned int i=0;i<mUltraframeSets.size();i++)
	//	if (seqName.equal(mUltraframeSets[i].seqName))
	//		set = i;

	//if (set>=0)
	//	mUltraframeSets.erase(set);
}

void fxFlexBody::addUltraframeSet(Ogre::String &seqName)
{
	//for (unsigned int i=0;i<mUltraframeSets.size();i++)
	//	if (seqName.equal(mUltraframeSets[i].seqName))
	//		return;//bail if we've already been here for this sequence.

	//int seqIndex = -1;


	//TSShape *kShape = getShapeInstance()->getShape();
	//for (unsigned int i=0;i<kShape->sequences.size();i++)
	//{
	//	if (seqName.equal(kShape->getName(kShape->sequences[i].nameIndex)))
	//		seqIndex = i;
	//}

	//if (seqIndex < 0)
	//	return;//We don't have this sequence loaded, something wrong happened, bail.

	////Else, go ahead and add the ultraframeset.
	//mUltraframeSets.increment();
	//int set = mUltraframeSets.size()-1;
	//mUltraframeSets[set].seqName = seqName;
	//mUltraframeSets[set].seq = seqIndex;

	////Fill array of beginnings of ultraframe lists with null.
	//for (unsigned int i=0;i<ULTRAFRAME_TYPES;i++)
	//	for (unsigned int j=0;j<MAX_ULTRAFRAME_NODES;j++)
	//		mUltraframeSets[set].lists[i][j] = NULL;

	////clear the bool arrays (used for early out, to speed up searching)
	//for (unsigned int i=0;i<ULTRAFRAME_TYPES;i++)
	//	mUltraframeSets[set].types[i] = false;
	//for (unsigned int j=0;j<MAX_ULTRAFRAME_NODES;j++)
	//	mUltraframeSets[set].nodes[j] = false;
}


bool fxFlexBody::hasUltraframe(int seq, int frame, int node, int type)
{
	//TSShape *kShape = getShapeInstance()->getShape();
	//for (unsigned int i=0;i<mUltraframeSets.size();i++)
	//{
	//	if (mUltraframeSets[i].seq==seq)
	//	{
	//		int mattersNode = kShape->getNodeMattersIndex(seq,node);
	//		if ((mUltraframeSets[i].types[type]==false)||(mUltraframeSets[i].nodes[mattersNode]==false))
	//			return false;

	//		ultraframe *uf = NULL;
	//		uf = mUltraframeSets[i].lists[type][node];
	//		if (!uf) 
	//			return false;
	//		else if (uf->frame==frame)
	//			return true;
	//		else
	//		{
	//			while (uf) 
	//			{
	//				if (uf->frame == frame)
	//					return true;
	//				else 
	//					uf = uf->next;
	//			}
	//		}
	//		break;
	//	}
	//}
	return false;
}

void fxFlexBody::addUltraframe(int seq,int frame,int node,int type,int target,Ogre::Vector3 &value)
{
	////HERE: insert into sqlite database!  Need to know what keyframeSet we are a part of, however.
	////Made a new table, actorKeyframeSet, with which you can look up a keyframeSet by means of 
	////an actor_id and a sequence_id.

	////One way: store sequence id in shape constructor?  Skipping that, going with straight SQL for now.
	////const String myFullPath = getShapeInstance()->mShapeResource.getPath().getFullPath();
	////TSShapeConstructor* ctor = TSShapeConstructor::findShapeConstructor( myFullPath );
	////if (ctor)
	////{
	////	ctor->...?
	////}
	//
	//TSShape *kShape = getShapeInstance()->getShape();
	//TSShape::Sequence *kSeq = &(kShape->sequences[seq]);
	//String seqName = kShape->getName(kSeq->nameIndex); 

	//int framesetIndex = -1;
	//for (unsigned int i=0;i<mUltraframeSets.size();i++)
	//	if (seqName.equal(mUltraframeSets[i].seqName))
	//		framesetIndex = i;

	//if (framesetIndex < 0) {
	//	addUltraframeSet(seqName);
	//	framesetIndex = mUltraframeSets.size()-1;
	//}

	//int sequence_id=0,keyframe_set_id=0,scene_id=0;
	//scene_id = dynamic_cast<nxPhysManager*>(mPM)->mSceneId;
	//String sceneName = dynamic_cast<nxPhysManager*>(mPM)->mSceneName;

	////First, we have to find our current keyframeSet, and create one if necessary.
	//SQLiteObject *sql = new SQLiteObject();
	//if (sql)
	//{
	//	if (sql->OpenDatabase("EcstasyMotion.db"))
	//	{
	//		char actor_keyframe_set_id_query[512],keyframe_set_id_query[512],insert_query[512],sequence_id_query[512];
	//		int result;
	//		sqlite_resultset *resultSet;

	//		result = sql->ExecuteSQL("BEGIN TRANSACTION;");

	//		sprintf(sequence_id_query,"SELECT id FROM sequence WHERE name='%s' AND skeleton_id=%d",kShape->getName(kSeq->nameIndex).c_str(),mSkeletonID);
	//		result = sql->ExecuteSQL(sequence_id_query);//Should really keep sequence ids around somewhere, but just looking them up for now.
	//		if (result==0)
	//		{
	//			//Con::errorf("addUltraframe: could not find sequence where name = %s",kShape->getName(kSeq->nameIndex).c_str());
	//			sql->CloseDatabase();
	//			return;
	//		}
	//		resultSet = sql->GetResultSet(result);
	//		if (resultSet->iNumRows == 1)
	//			sequence_id = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);
	//		
	//		sprintf(keyframe_set_id_query,"SELECT id FROM keyframeSet WHERE sequence_id=%d AND actor_id=%d AND scene_id=%d",
	//			sequence_id,mActorID,scene_id);
	//		result = sql->ExecuteSQL(keyframe_set_id_query);			
	//		if (result==0)
	//		{
	//			//Con::errorf("addUltraframe: could not find keyframe_set_id where sequence_id = %d",sequence_id);
	//			sql->CloseDatabase();
	//			return;
	//		}
	//		resultSet = sql->GetResultSet(result);
	//		if (resultSet->iNumRows == 1)
	//			keyframe_set_id = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);
	//		else if (resultSet->iNumRows == 0) {					
	//			char morphName[512];
	//			sprintf(morphName,"%s.%s.%s",sceneName.c_str(),mActorName,kShape->getName(kSeq->nameIndex).c_str());
	//			////Now, see if we have a default sequence morph for this scene, this actor, this sequence.
	//			//sprintf(keyframe_set_id_query,"SELECT id FROM keyframeSet WHERE actor_id=%d AND sequence_id=%d AND scene_id=%d AND name='%s'",
	//			//	mActorID,sequence_id,scene_id,morphName);
	//			//result = sql->ExecuteSQL(keyframe_set_id_query);
	//			//resultSet = sql->GetResultSet(result);
	//			//if (resultSet->iNumRows == 1)
	//			//	keyframe_set_id = strtol(resultSet->vRows[0]->vColumnValues[0]);

	//			//if (keyframe_set_id == 0)
	//			//{//Now, didn't have a keyframeSet for this actor-sequence-scene, so make one.
	//			sprintf(insert_query,"INSERT INTO keyframeSet (sequence_id,skeleton_id,actor_id,scene_id,name) VALUES (%d,%d,%d,%d,'%s');",
	//				sequence_id,mSkeletonID,mActorID,scene_id,morphName);
	//			result = sql->ExecuteSQL(insert_query);
	//			result = sql->ExecuteSQL(keyframe_set_id_query);
	//			resultSet = sql->GetResultSet(result);
	//			if (resultSet->iNumRows == 1)
	//				keyframe_set_id = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);

	//		}			
	//		result = sql->ExecuteSQL("END TRANSACTION;");
	//		sql->CloseDatabase();
	//	}
	//}
	////Now: we should have a valid keyframe_set_id.  Use that to insert or update as needed.  (Repeat this logic in saveUltraframe(), because
	////that can be called from many other places.)


	//unsigned int set = framesetIndex;
	////Con::errorf("addUltraframe( seq %d frame %d node %d type %d force %f %f %f)",seq,frame,node,type,value.x,value.y,value.z);
	////for (unsigned int i=0;i<mUltraframeSets.size();i++)
	////{
	//	//if (mUltraframeSets[i].seq==seq)
	//	//{
	//		//set=i;

	//if (frame > kSeq->numKeyframes-1) 
	//{
	//	//Con::errorf("Ultraframe out of bounds: seq %d frame %d, numKeyframes %d",seq,frame,kSeq->numKeyframes);
	//}
	//if (!kSeq->rotationMatters.test(node))
	//{
	//	//Con::errorf("Ultraframe node appears not to matter to this sequence.");
	//	//return;//check this first
	//}

	//int mattersNode = kShape->getNodeMattersIndex(seq,node);
	//mUltraframeSets[set].types[type] = true;
	//mUltraframeSets[set].nodes[mattersNode] = true;

	//ultraframe *uf,*lf;
	//lf = NULL;
	//uf = mUltraframeSets[set].lists[type][node];
	//while (uf) 
	//{
	//	if (uf->frame < frame)
	//	{
	//		lf = uf;
	//		uf = uf->next;
	//	} else break;
	//}
	//if (uf)
	//	if (uf->frame==frame)
	//	{//Hit a frame exactly, which shouldn't happen, we are supposed to have checked for that by now and be calling save
	//		//if that is the case.  But pop out a warning and call save anyway.
	//		//Con::warnf("Warning: trying to add an existing ultraframe! Frame: %d",frame);
	//		saveUltraframe(seq,frame,node,type,target,value);//saveUltraframe does its own update to DB					
	//		return;
	//	}
	//if (!uf) 
	//{//HERE: have to create a new list, by instantiating 0 and 1.0 ultraframes, and then inserting 
	//	//this one between them if it isn't at an endpoint.
	//	mUltraframeSets[set].frames.increment();//first one
	//	int start = mUltraframeSets[set].frames.size()-1;
	//	mUltraframeSets[set].frames.increment();//last one
	//	int end = mUltraframeSets[set].frames.size()-1;

	//	mUltraframeSets[set].frames[start].frame = 0;
	//	mUltraframeSets[set].frames[start].node = node;
	//	mUltraframeSets[set].frames[start].type = type;
	//	mUltraframeSets[set].frames[start].target = target;
	//	mUltraframeSets[set].frames[start].value = Ogre::Vector3(0,0,0);
	//	mUltraframeSets[set].frames[start].next = &(mUltraframeSets[set].frames[end]);

	//	mUltraframeSets[set].frames[end].frame = kSeq->numKeyframes-1;
	//	mUltraframeSets[set].frames[end].node = node;
	//	mUltraframeSets[set].frames[end].type = type;
	//	mUltraframeSets[set].frames[end].target = target;
	//	mUltraframeSets[set].frames[end].value = Ogre::Vector3(0,0,0);
	//	mUltraframeSets[set].frames[end].next = NULL;
	//	
	//	lf = &(mUltraframeSets[set].frames[start]);
	//	uf = &(mUltraframeSets[set].frames[end]);

	//	mUltraframeSets[set].lists[type][node] = &(mUltraframeSets[set].frames[start]);
	//	//Here: insert into DB
	//	if ((keyframe_set_id)&&(sql))
	//	{
	//		if (sql->OpenDatabase("EcstasyMotion.db"))
	//		{
	//			char id_query[512],insert_query[512];
	//			int result,keyframe_set_frame_id=0;
	//			sqlite_resultset *resultSet;

	//			sprintf(id_query,"SELECT id FROM keyframe WHERE keyframe_set_id=%d \
	//							 AND type=%d AND frame=%d AND node=%d;",
	//							 keyframe_set_id,type,frame,node);//time is not supported yet
	//			result = sql->ExecuteSQL(id_query);
	//			resultSet = sql->GetResultSet(result);
	//			if (resultSet->iNumRows == 1)
	//			{//This shouldn't ever happen...
	//				keyframe_set_frame_id = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);
	//				sprintf(insert_query,"UPDATE keyframe SET value_x=%f,value_y=%f,value_z=%f WHERE id=%d;",
	//					value.x,value.y,value.z,keyframe_set_frame_id);
	//				result = sql->ExecuteSQL(insert_query);
	//			} else {// ... because this should be what's going on.  Need to add start and end frames too, though. 
	//				sprintf(insert_query,"INSERT INTO keyframe (keyframe_set_id,type,\
	//									 frame,node,value_x,value_y,value_z) VALUES (%d,%d,%d,%d,%f,%f,%f);",
	//									 keyframe_set_id,type,0,node,0.0,0.0,0.0);
	//				result = sql->ExecuteSQL(insert_query);//Start frame bookend
	//				sprintf(insert_query,"INSERT INTO keyframe (keyframe_set_id,type,\
	//									 frame,node,value_x,value_y,value_z) VALUES (%d,%d,%d,%d,%f,%f,%f);",
	//									 keyframe_set_id,type,frame,node,value.x,value.y,value.z);
	//				result = sql->ExecuteSQL(insert_query);//Actual new keyframe data
	//				sprintf(insert_query,"INSERT INTO keyframe (keyframe_set_id,type,\
	//									 frame,node,value_x,value_y,value_z) VALUES (%d,%d,%d,%d,%f,%f,%f);",
	//									 keyframe_set_id,type,kSeq->numKeyframes-1,node,0.0,0.0,0.0);
	//				result = sql->ExecuteSQL(insert_query);//End frame bookend.
	//			}
	//			sql->CloseDatabase();
	//		}
	//	}
	//	
	//	//Con::errorf("added base ultraframes! start=%d, end=%d",lf->frame,uf->frame);
	//}
	//if ( uf && lf )
	//{//Now, whether by creating the list or because I already had frames around this point
	//	if ((frame>0) && (frame<(kSeq->numKeyframes-1)))
	//	{
	//		mUltraframeSets[set].frames.increment();
	//		int id = mUltraframeSets[set].frames.size()-1;
	//		mUltraframeSets[set].frames[id].frame = frame;//How many times can we use the word f-r-a-m-e in one line of code?
	//		mUltraframeSets[set].frames[id].node = node;
	//		mUltraframeSets[set].frames[id].type = type;
	//		mUltraframeSets[set].frames[id].target = target;
	//		mUltraframeSets[set].frames[id].value = value;
	//		mUltraframeSets[set].frames[id].next = uf;
	//		lf->next = &(mUltraframeSets[set].frames[id]);

	//		//Here: insert into DB
	//		if ((keyframe_set_id)&&(sql))
	//		{
	//			if (sql->OpenDatabase("EcstasyMotion.db"))
	//			{
	//				char id_query[512],insert_query[512];
	//				int result,keyframe_set_frame_id=0;
	//				sqlite_resultset *resultSet;

	//				sprintf(id_query,"SELECT id FROM keyframe WHERE keyframe_set_id=%d \
	//								 AND type=%d AND frame=%d AND node=%d;",
	//								 keyframe_set_id,type,frame,node);//time is not supported yet
	//				result = sql->ExecuteSQL(id_query);
	//				resultSet = sql->GetResultSet(result);
	//				if (resultSet->iNumRows == 1)
	//				{
	//					keyframe_set_frame_id = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);
	//					sprintf(insert_query,"UPDATE keyframe SET value_x=%f,value_y=%f,value_z=%f WHERE id=%d;",
	//										 value.x,value.y,value.z,keyframe_set_frame_id);
	//					result = sql->ExecuteSQL(insert_query);
	//				} else {
	//					sprintf(insert_query,"INSERT INTO keyframe (keyframe_set_id,type,\
	//										 frame,node,value_x,value_y,value_z) VALUES (%d,%d,%d,%d,%f,%f,%f);",
	//										 keyframe_set_id,type,frame,node,value.x,value.y,value.z);
	//					result = sql->ExecuteSQL(insert_query);
	//				}
	//				sql->CloseDatabase();
	//			}
	//		}
	//		//Con::errorf("added ultraframe! frame=%d",frame);
	//	}
	//}

	//delete sql;

	//if (set<0) return;//Error, no ultraframeSet exists for this sequence, skipped all of above.
	//else//Else go ahead and do a save, even though you already set the value above, in order to modify all the node data if necessary.
	//{
	//	saveUltraframe(seq,frame,node,type,target,value);//NoInsert
	//}
}

void fxFlexBody::addUltraframeNoInsert(int seq,int frame,int node,int type,int target,Ogre::Vector3 &value)
{
	
	//TSShape *kShape = getShapeInstance()->getShape();
	//TSShape::Sequence *kSeq = &(kShape->sequences[seq]);
	//String seqName = kShape->getName(kSeq->nameIndex); 

	//int framesetIndex = -1;
	//for (unsigned int i=0;i<mUltraframeSets.size();i++)
	//	if (seqName.equal(mUltraframeSets[i].seqName))
	//		framesetIndex = i;

	//if (framesetIndex < 0) {
	//	addUltraframeSet(seqName);
	//	framesetIndex = mUltraframeSets.size()-1;
	//}

	//int sequence_id=0,keyframe_set_id=0,scene_id=0;
	//scene_id = dynamic_cast<nxPhysManager*>(mPM)->mSceneId;
	//String sceneName = dynamic_cast<nxPhysManager*>(mPM)->mSceneName;

	////Con::printf("adding ultraframe, scene id %d seq %d node %d value %3.2f %3.2f %3.2f ",
	//	scene_id,seq,node,value.x,value.y,value.z);

	//unsigned int set = framesetIndex;

	//if (frame > kSeq->numKeyframes-1) 
	//{
	//	//Con::errorf("Ultraframe out of bounds: seq %d frame %d, numKeyframes %d",seq,frame,kSeq->numKeyframes);
	//}
	//if (!kSeq->rotationMatters.test(node))
	//{
	//	//Con::errorf("Ultraframe node appears not to matter to this sequence.");
	//	//return;//check this first
	//}

	//int mattersNode = kShape->getNodeMattersIndex(seq,node);
	//mUltraframeSets[set].types[type] = true;
	//mUltraframeSets[set].nodes[mattersNode] = true;

	//ultraframe *uf,*lf;
	//lf = NULL;
	//uf = mUltraframeSets[set].lists[type][node];
	//while (uf) 
	//{
	//	if (uf->frame < frame)
	//	{
	//		lf = uf;
	//		uf = uf->next;
	//	} else break;
	//}
	//if (uf)
	//	if (uf->frame==frame)
	//	{//Hit a frame exactly, which shouldn't happen, we are supposed to have checked for that by now and be calling save
	//		//if that is the case.  But pop out a warning and call save anyway.
	//		//Con::warnf("Warning: trying to add an existing ultraframe! Frame: %d",frame);
	//		saveUltraframe(seq,frame,node,type,target,value);//saveUltraframe does its own update to DB					
	//		return;
	//	}
	//if (!uf) 
	//{//HERE: have to create a new list, by instantiating 0 and 1.0 ultraframes, and then inserting 
	//	//this one between them if it isn't at an endpoint.
	//	mUltraframeSets[set].frames.increment();//first one
	//	int start = mUltraframeSets[set].frames.size()-1;
	//	mUltraframeSets[set].frames.increment();//last one
	//	int end = mUltraframeSets[set].frames.size()-1;

	//	mUltraframeSets[set].frames[start].frame = 0;
	//	mUltraframeSets[set].frames[start].node = node;
	//	mUltraframeSets[set].frames[start].type = type;
	//	mUltraframeSets[set].frames[start].target = target;
	//	mUltraframeSets[set].frames[start].value = Ogre::Vector3(0,0,0);
	//	mUltraframeSets[set].frames[start].next = &(mUltraframeSets[set].frames[end]);

	//	mUltraframeSets[set].frames[end].frame = kSeq->numKeyframes-1;
	//	mUltraframeSets[set].frames[end].node = node;
	//	mUltraframeSets[set].frames[end].type = type;
	//	mUltraframeSets[set].frames[end].target = target;
	//	mUltraframeSets[set].frames[end].value = Ogre::Vector3(0,0,0);
	//	mUltraframeSets[set].frames[end].next = NULL;
	//	
	//	lf = &(mUltraframeSets[set].frames[start]);
	//	uf = &(mUltraframeSets[set].frames[end]);

	//	mUltraframeSets[set].lists[type][node] = &(mUltraframeSets[set].frames[start]);
	//	
	//	//Con::errorf("added base ultraframes! start=%d, end=%d",lf->frame,uf->frame);
	//}
	//if ( uf && lf )
	//{//Now, whether by creating the list or because I already had frames around this point
	//	if ((frame>0) && (frame<(kSeq->numKeyframes-1)))
	//	{
	//		mUltraframeSets[set].frames.increment();
	//		int id = mUltraframeSets[set].frames.size()-1;
	//		mUltraframeSets[set].frames[id].frame = frame;//How many times can we use the word f-r-a-m-e in one line of code?
	//		mUltraframeSets[set].frames[id].node = node;
	//		mUltraframeSets[set].frames[id].type = type;
	//		mUltraframeSets[set].frames[id].target = target;
	//		mUltraframeSets[set].frames[id].value = value;
	//		mUltraframeSets[set].frames[id].next = uf;
	//		lf->next = &(mUltraframeSets[set].frames[id]);

	//	}
	//}

	//if (set<0) return;//Error, no ultraframeSet exists for this sequence, skipped all of above.
	//else//Else go ahead and do a save, even though you already set the value above, in order to modify all the node data if necessary.
	//{
	//	saveUltraframeNoInsert(seq,frame,node,type,target,value);
	//}
}

void fxFlexBody::addUltraframeFromFile(int seq, int frame, int node, int type, int target, Ogre::Vector3 &value)
{
	//Hmm, maybe don't need this after all... existing functions should work.  It would be 
	//more efficient to load from file as a separate process though, because at end of list
	//we could recalculate all node rotations and translations in one step instead of modifying 
	//the whole tail of list on each add. 
}

void fxFlexBody::addUltraframeSingle(int seq,int frame,int node,int type,int target,Ogre::Vector3 &value)
{//VERY MISLEADING: this is changing the arrays directly, not an ultraframe at all, not reversible.
	//HERE: change just the one frame.  Really this needs to be another struct, or maybe we can bend ultraframe into 
	//serving this function as well with a flag, isSingle.  For now, don't even bother w/ saving, just get the work done.
	//int rot_matters_count,node_matter_index;
	//rot_matters_count = 0;

	//TSShape *kShape = getShapeInstance()->getShape();
	//TSShape::Sequence *kSeq = &(kShape->sequences[seq]);

	//if ((type==ADJUST_NODE_ROT) || (type==SET_NODE_ROT))
	//{
	//	if (!kSeq->rotationMatters.test(node)) { 
	//		//Con::errorf("Node %d doesn't matter!",node); 
	//		return; 
	//	}
	//	for (unsigned int j=0;j<kShape->nodes.size();j++) 
	//	{
	//		if (j==node) node_matter_index = rot_matters_count;
	//		if (kSeq->rotationMatters.test(j)) 
	//			rot_matters_count++;
	//	}
	//}
	//if ((type==ADJUST_NODE_POS) || (type==SET_NODE_POS))
	//{
	//	Ogre::Vector3 newPos,basePos;
	//	basePos = kShape->nodeTranslations[kSeq->baseTranslation+frame];//+node
	//	if (type==ADJUST_NODE_POS)
	//		newPos = basePos + value;
	//	else if (type==SET_NODE_POS)
	//		newPos = value;
	//	kShape->nodeTranslations[kSeq->baseTranslation+frame] = newPos;
	//} 
	//else if ((type==ADJUST_NODE_ROT) || (type==SET_NODE_ROT))
	//{
	//	Ogre::Quaternion q((Ogre::Vector3)value);
	//	Quat16 newQuat,baseQuat;
	//	//Con::errorf("adjusting node %d frame %d rot %f %f %f",node,frame,value.x,value.y,value.z);
	//	if (type==SET_NODE_ROT)
	//	{
	//		newQuat.set(q);
	//	} 
	//	else if (type==ADJUST_NODE_ROT)
	//	{
	//		Ogre::Quaternion temp;
	//		baseQuat = kShape->nodeRotations[kSeq->baseRotation+(node_matter_index*kSeq->numKeyframes)+frame];
	//		baseQuat.getOgre::Quaternion(&temp);
	//		temp *= q;
	//		newQuat.set(temp);
	//	}
	//	kShape->nodeRotations[kSeq->baseRotation+(node_matter_index*kSeq->numKeyframes)+frame] = newQuat;
	//}
}

void fxFlexBody::saveUltraframe(int seq, int frame, int node, int type, int target, Ogre::Vector3 &value)
{
	//Con::errorf("saving ultraframe:  seq %d frame %d node %d type %d value %f %f %f",seq,frame,node,type,value.x,value.y,value.z);
	
	//Ogre::Vector3 curVal,lastVal,nextVal;
	//curVal = Ogre::Vector3::ZERO; lastVal = Ogre::Vector3::ZERO; nextVal = Ogre::Vector3::ZERO;
	//int lastFrame,nextFrame;
	//lastFrame = nextFrame = 0;
	//int rot_matters_count,node_matter_index;
	//rot_matters_count = 0;

	//TSShape *kShape = getShapeInstance()->getShape();
	//TSShape::Sequence *kSeq = &(kShape->sequences[seq]);
	//String seqName = kShape->getName(kShape->sequences[seq].nameIndex);

	//int sequence_id=0,keyframe_set_id=0,scene_id=0,keyframe_set_frame_id=0;
	//scene_id = dynamic_cast<nxPhysManager*>(mPM)->mSceneId;
	//String sceneName = dynamic_cast<nxPhysManager*>(mPM)->mSceneName;

	//for (unsigned int j=0;j<kShape->nodes.size();j++) 
	//{
	//	if (j==node) node_matter_index = rot_matters_count;
	//	if (kSeq->rotationMatters.test(j)) 
	//		rot_matters_count++;
	//}
	////Old way.
	////if (backupRotations.size()==0)
	////	backupSequenceData();

	////New way:
	////bool foundBackup = false;
	////int seqBackup = -1;
	//TSShape::sequenceBackup *seqBackup = NULL;
	//for (unsigned int i=0;i<kShape->sequenceBackups.size();i++)
	//{
	//	if (kShape->sequenceBackups[i].name.equal(seqName))
	//	{
	//		seqBackup = &(kShape->sequenceBackups[i]);
	//	}
	//}
	//if (seqBackup==NULL)
	//{
	//	//add a new sequenceBackup
	//	kShape->sequenceBackups.increment();
	//	seqBackup = &(kShape->sequenceBackups[kShape->sequenceBackups.size()-1]);
	//	seqBackup->name = seqName;
	//	for (unsigned int j=0;j<kSeq->numKeyframes;j++)
	//	{
	//		seqBackup->nodeTranslations.increment();
	//		seqBackup->nodeTranslations[seqBackup->nodeTranslations.size()-1] = kShape->nodeTranslations[kSeq->baseTranslation+j];
	//	}
	//	for (unsigned int j=0;j<rot_matters_count;j++)
	//	{

	//		for (unsigned int k=0;k<kSeq->numKeyframes;k++)
	//		{
	//			seqBackup->nodeRotations.increment();
	//			seqBackup->nodeRotations[seqBackup->nodeRotations.size()-1] = kShape->nodeRotations[kSeq->baseRotation+(j*kSeq->numKeyframes)+k];
	//		}
	//	}
	//}

	////First, we have to find our current KeyframeSet, and create one if necessary.
	//SQLiteObject *sql = new SQLiteObject();
	//if (sql)
	//{
	//	if (sql->OpenDatabase("EcstasyMotion.db"))
	//	{
	//		char actor_keyframe_set_id_query[512],keyframe_set_id_query[512],id_query[512],insert_query[512];
	//		int result;
	//		sqlite_resultset *resultSet;

	//		result = sql->ExecuteSQL("BEGIN TRANSACTION;");

	//		sprintf(actor_keyframe_set_id_query,"SELECT id FROM sequence WHERE name='%s' AND skeleton_id=%d;",kShape->getName(kSeq->nameIndex).c_str(),mSkeletonID);
	//		result = sql->ExecuteSQL(actor_keyframe_set_id_query);//Should really keep sequence ids around somewhere, but just looking them up for now.
	//		resultSet = sql->GetResultSet(result);
	//		if (resultSet->iNumRows == 1)
	//			sequence_id = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);
	//		
	//		sprintf(keyframe_set_id_query,"SELECT id FROM keyframeSet WHERE sequence_id=%d AND actor_id=%d AND scene_id=%d;",sequence_id,mActorID,scene_id);
	//		result = sql->ExecuteSQL(keyframe_set_id_query);
	//		resultSet = sql->GetResultSet(result);
	//		if (resultSet->iNumRows == 1)
	//			keyframe_set_id = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);
	//		else if (resultSet->iNumRows == 0) {					
	//			char morphName[512];
	//			sprintf(morphName,"%s.%s.%s",sceneName.c_str(),mActorName,kShape->getName(kSeq->nameIndex).c_str());
	//			//Now, see if we have a default sequence morph for this scene, this actor, this sequence.
	//			//sprintf(keyframe_set_id_query,"SELECT id FROM keyframeSet WHERE actor_id=%d AND sequence_id=%d AND scene_id=%d AND name='%s'",mActorID,sequence_id,scene_id,morphName);
	//			//result = sql->ExecuteSQL(keyframe_set_id_query);
	//			//resultSet = sql->GetResultSet(result);
	//			//if (resultSet->iNumRows == 1)
	//			//	keyframe_set_id = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);

	//			//if (keyframe_set_id == 0)
	//			//{//Now, didn't have a sequence morph for this actor, sequence, scene, so make one.
	//			sprintf(insert_query,"INSERT INTO keyframeSet (sequence_id,skeleton_id,actor_id,scene_id,name) VALUES (%d,%d,%d,%d,'%s');",
	//				sequence_id,mSkeletonID,mActorID,scene_id,morphName);
	//			result = sql->ExecuteSQL(insert_query);
	//			result = sql->ExecuteSQL(keyframe_set_id_query);
	//			resultSet = sql->GetResultSet(result);
	//			if (resultSet->iNumRows == 1)
	//				keyframe_set_id = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);
	//			//}
	//			//Now, either way, we need to assign this as current one in actorKeyframeSet. [NOT! removing this table, redundant.]
	//			//if (keyframe_set_id)
	//			//{
	//			//	sprintf(insert_query,"INSERT INTO actorKeyframeSet (sequence_id,actor_id,keyframe_set_id) VALUES (%d,%d,%d);",
	//			//		sequence_id,mActorID,keyframe_set_id);
	//			//	result = sql->ExecuteSQL(insert_query);
	//			//}
	//		}
	//		//Now, we should have a valid keyframe_set_id no matter what.	
	//		sprintf(id_query,"SELECT id FROM keyframe WHERE keyframe_set_id=%d \
	//						 AND type=%d AND frame=%d AND node=%d;",
	//						 keyframe_set_id,type,frame,node);//time is not supported yet
	//		result = sql->ExecuteSQL(id_query);
	//		resultSet = sql->GetResultSet(result);
	//		if (resultSet->iNumRows == 1)
	//		{
	//			keyframe_set_frame_id = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);
	//			sprintf(insert_query,"UPDATE keyframe SET value_x=%f,value_y=%f,value_z=%f WHERE id=%d;",
	//				value.x,value.y,value.z,keyframe_set_frame_id);
	//			result = sql->ExecuteSQL(insert_query);
	//		} 
	//		result = sql->ExecuteSQL("END TRANSACTION;");
	//		sql->CloseDatabase();
	//	}
	//	delete sql;
	//}

	//if ((type==ADJUST_NODE_ROT) || (type==SET_NODE_ROT))
	//{
	//	if (!kSeq->rotationMatters.test(node)) { 
	//		//Con::errorf("Node %d doesn't matter!",node); 
	//		return; 
	//	}

	//}

	//for (unsigned int i=0;i<mUltraframeSets.size();i++)
	//{
	//	if (mUltraframeSets[i].seq==seq)
	//	{
	//		
	//		int mattersNode = kShape->getNodeMattersIndex(seq,node);
	//		mUltraframeSets[i].types[type] = true;
	//		mUltraframeSets[i].nodes[mattersNode] = true;

	//		ultraframe *uf,*lf;
	//		lf = NULL;
	//		uf = mUltraframeSets[i].lists[type][node];
	//		while (uf) 
	//		{
	//			if (uf->frame < frame)
	//			{
	//				lf = uf;
	//				uf = uf->next;
	//			} else break;
	//		}
	//		if (uf->frame == frame)
	//		{//sanity check, if we don't hit the frame, then we bail.  Should have added it first.
	//			
	//			uf->value = value;
	//			uf->target = target;

	//			if ((type==ADJUST_NODE_POS) || (type==SET_NODE_POS) ||
	//				(type==ADJUST_NODE_ROT) || (type==SET_NODE_ROT) )
	//			{
	//				Ogre::Vector3 newPos,basePos;
	//				Quat16 newQuat,baseQuat;

	//				if (lf)
	//				{
	//					lastFrame = lf->frame;
	//					lastVal = lf->value;
	//				}
	//				if (uf->next) 
	//				{
	//					nextFrame = uf->next->frame;
	//					nextVal = uf->next->value;
	//				}

	//				if (lf)
	//				{
	//					for (unsigned int k=lastFrame+1;k<=frame;k++)
	//					{
	//						curVal = lastVal + ((value - lastVal)*((float)(k-lastFrame)/(float)(frame-lastFrame)));
	//						if ((type==ADJUST_NODE_POS) || (type==SET_NODE_POS))
	//						{
	//							//basePos = backupTranslations[k+kSeq->baseTranslation];//+node
	//							basePos = seqBackup->nodeTranslations[k];
	//							if (type==ADJUST_NODE_POS)
	//								newPos = basePos + curVal;
	//							else if (type==SET_NODE_POS)
	//								newPos = curVal;
	//							kShape->nodeTranslations[k+kSeq->baseTranslation] = newPos;
	//						}
	//						else if ((type==ADJUST_NODE_ROT) || (type==SET_NODE_ROT))
	//						{
	//							curVal.x = mDegToRad(curVal.x);
	//							curVal.y = mDegToRad(curVal.y);
	//							curVal.z = mDegToRad(curVal.z);

	//							Ogre::Quaternion q((Ogre::Vector3)curVal);
	//							if (type==SET_NODE_ROT)
	//							{
	//								newQuat.set(q);

	//							} else if (type==ADJUST_NODE_ROT) {
	//								Ogre::Quaternion temp;
	//								//baseQuat = backupRotations[kSeq->baseRotation+(node_matter_index*kSeq->numKeyframes)+k];
	//								baseQuat = seqBackup->nodeRotations[(node_matter_index*kSeq->numKeyframes)+k];
	//								baseQuat.getOgre::Quaternion(&temp);
	//								temp *= q;
	//								newQuat.set(temp);
	//							}
	//							kShape->nodeRotations[kSeq->baseRotation+(node_matter_index*kSeq->numKeyframes)+k] = newQuat;
	//						}
	//					}
	//				}
	//				if (uf->next) 
	//				{
	//					for (unsigned int k=frame;k<nextFrame;k++)
	//					{//NOTE: including k=frame on both sides, so it always gets done at least once even if I'm at begin or end.
	//						curVal = value + ((nextVal - value)*((float)(k-frame)/(float)(nextFrame-frame)));

	//						if ((type==ADJUST_NODE_POS) || (type==SET_NODE_POS))
	//						{
	//							//basePos = backupTranslations[k+kSeq->baseTranslation];//+node
	//							basePos = seqBackup->nodeTranslations[k];
	//							if (type==ADJUST_NODE_POS)
	//								newPos = basePos + curVal;
	//							else if (type==SET_NODE_POS)
	//								newPos = curVal;
	//							kShape->nodeTranslations[k+kSeq->baseTranslation] = newPos;
	//						} 
	//						else if ((type==ADJUST_NODE_ROT) || (type==SET_NODE_ROT))
	//						{
	//							curVal.x = mDegToRad(curVal.x);
	//							curVal.y = mDegToRad(curVal.y);
	//							curVal.z = mDegToRad(curVal.z);

	//							Ogre::Quaternion q((Ogre::Vector3)curVal);
	//							if (type==SET_NODE_ROT)
	//							{
	//								newQuat.set(q);
	//							} 
	//							else if (type==ADJUST_NODE_ROT)
	//							{
	//								Ogre::Quaternion temp;
	//								//baseQuat = backupRotations[kSeq->baseRotation+(node_matter_index*kSeq->numKeyframes)+i];
	//								baseQuat = seqBackup->nodeRotations[(node_matter_index*kSeq->numKeyframes)+k];
	//								baseQuat.getOgre::Quaternion(&temp);
	//								temp *= q;
	//								newQuat.set(temp);
	//							}
	//							kShape->nodeRotations[kSeq->baseRotation+(node_matter_index*kSeq->numKeyframes)+k] = newQuat;
	//						}
	//					}
	//				}
	//			}
	//		}
	//		break;
	//	}
	//}
}

void fxFlexBody::saveUltraframeNoInsert(int seq, int frame, int node, int type, int target, Ogre::Vector3 &value)
{
	//Con::errorf("saving ultraframe:  seq %d frame %d node %d type %d value %f %f %f",seq,frame,node,type,value.x,value.y,value.z);
	
	//Ogre::Vector3 curVal,lastVal,nextVal;
	//curVal = Ogre::Vector3::ZERO; lastVal = Ogre::Vector3::ZERO; nextVal = Ogre::Vector3::ZERO;
	//int lastFrame,nextFrame;
	//lastFrame = nextFrame = 0;
	//int rot_matters_count,node_matter_index;

	//TSShape *kShape = getShapeInstance()->getShape();
	//TSShape::Sequence *kSeq = &(kShape->sequences[seq]);
	//String seqName = kShape->getName(kSeq->nameIndex);

	//int sequence_id=0,keyframe_set_id=0,scene_id=0,keyframe_set_frame_id=0;
	//scene_id = dynamic_cast<nxPhysManager*>(mPM)->mSceneId;
	//String sceneName = dynamic_cast<nxPhysManager*>(mPM)->mSceneName;

	////Old way.
	////if (backupRotations.size()==0)
	////	backupSequenceData();

	////New way:
	////bool foundBackup = false;
	////int seqBackup = -1;

	//rot_matters_count = 0;
	//for (unsigned int i=0;i<kShape->nodes.size();i++) 
	//	if (kSeq->rotationMatters.test(i)) 
	//		rot_matters_count++;

	//TSShape::sequenceBackup *seqBackup = NULL;
	//for (unsigned int i=0;i<kShape->sequenceBackups.size();i++)
	//{
	//	if (kShape->sequenceBackups[i].name.equal(seqName))
	//	{
	//		seqBackup = &(kShape->sequenceBackups[i]);
	//	}
	//}
	//if (seqBackup==NULL)
	//{
	//	//add a new sequenceBackup
	//	kShape->sequenceBackups.increment();
	//	seqBackup = &(kShape->sequenceBackups[kShape->sequenceBackups.size()-1]);
	//	seqBackup->name = seqName;
	//	for (unsigned int j=0;j<kSeq->numKeyframes;j++)
	//	{
	//		seqBackup->nodeTranslations.increment();
	//		seqBackup->nodeTranslations[seqBackup->nodeTranslations.size()-1] = kShape->nodeTranslations[kSeq->baseTranslation+j];
	//	}
	//	for (unsigned int j=0;j<rot_matters_count;j++)
	//	{
	//		for (unsigned int k=0;k<kSeq->numKeyframes;k++)
	//		{
	//			seqBackup->nodeRotations.increment();
	//			seqBackup->nodeRotations[seqBackup->nodeRotations.size()-1] = kShape->nodeRotations[kSeq->baseRotation+(j*kSeq->numKeyframes)+k];
	//		}
	//	}
	//}

	//if ((type==ADJUST_NODE_ROT) || (type==SET_NODE_ROT))
	//{
	//	if (!kSeq->rotationMatters.test(node)) { 
	//		//Con::errorf("Node %d doesn't matter!",node); 
	//		return; 
	//	}
	//	//for (unsigned int j=0;j<kShape->nodes.size();j++) 
	//	//{
	//	//	if (j==node) node_matter_index = rot_matters_count;
	//	//	if (kSeq->rotationMatters.test(j)) 
	//	//		rot_matters_count++;
	//	//}
	//}

	//for (unsigned int i=0;i<mUltraframeSets.size();i++)
	//{
	//	if (mUltraframeSets[i].seq==seq)
	//	{
	//		
	//		int mattersNode = kShape->getNodeMattersIndex(seq,node);
	//		mUltraframeSets[i].types[type] = true;
	//		mUltraframeSets[i].nodes[mattersNode] = true;

	//		ultraframe *uf,*lf;
	//		lf = NULL;
	//		uf = mUltraframeSets[i].lists[type][node];
	//		while (uf) 
	//		{
	//			if (uf->frame < frame)
	//			{
	//				lf = uf;
	//				uf = uf->next;
	//			} else break;
	//		}
	//		if (uf)
	//		{
	//			if (uf->frame == frame)
	//			{//sanity check, if we don't hit the frame, then we bail.  Should have added it first.

	//				uf->value = value;
	//				uf->target = target;

	//				if ((type==ADJUST_NODE_POS) || (type==SET_NODE_POS) ||
	//					(type==ADJUST_NODE_ROT) || (type==SET_NODE_ROT) )
	//				{
	//					Ogre::Vector3 newPos,basePos;
	//					Quat16 newQuat,baseQuat;

	//					if (lf)
	//					{
	//						lastFrame = lf->frame;
	//						lastVal = lf->value;
	//					}
	//					if (uf->next) 
	//					{
	//						nextFrame = uf->next->frame;
	//						nextVal = uf->next->value;
	//					}

	//					if (lf)
	//					{
	//						for (unsigned int k=lastFrame+1;k<=frame;k++)
	//						{
	//							curVal = lastVal + ((value - lastVal)*((float)(k-lastFrame)/(float)(frame-lastFrame)));
	//							if ((type==ADJUST_NODE_POS) || (type==SET_NODE_POS))
	//							{
	//								//basePos = backupTranslations[k+kSeq->baseTranslation];//+node
	//								basePos = seqBackup->nodeTranslations[k];

	//								if (type==ADJUST_NODE_POS)
	//									newPos = basePos + curVal;
	//								else if (type==SET_NODE_POS)
	//									newPos = curVal;
	//								kShape->nodeTranslations[k+kSeq->baseTranslation] = newPos;
	//							}
	//							else if ((type==ADJUST_NODE_ROT) || (type==SET_NODE_ROT))
	//							{
	//								curVal.x = mDegToRad(curVal.x);
	//								curVal.y = mDegToRad(curVal.y);
	//								curVal.z = mDegToRad(curVal.z);

	//								Ogre::Quaternion q((Ogre::Vector3)curVal);
	//								if (type==SET_NODE_ROT)
	//								{
	//									newQuat.set(q);

	//								} else if (type==ADJUST_NODE_ROT) {
	//									Ogre::Quaternion temp;
	//									//baseQuat = backupRotations[kSeq->baseRotation+(node_matter_index*kSeq->numKeyframes)+k];
	//									baseQuat = seqBackup->nodeRotations[(mattersNode*kSeq->numKeyframes)+k];
	//									baseQuat.getOgre::Quaternion(&temp);
	//									temp *= q;
	//									newQuat.set(temp);
	//								}
	//								kShape->nodeRotations[kSeq->baseRotation+(mattersNode*kSeq->numKeyframes)+k] = newQuat;
	//							}
	//						}
	//					}
	//					if (uf->next) 
	//					{
	//						for (unsigned int k=frame;k<nextFrame;k++)
	//						{//NOTE: including k=frame on both sides, so it always gets done at least once even if I'm at begin or end.
	//							curVal = value + ((nextVal - value)*((float)(k-frame)/(float)(nextFrame-frame)));

	//							if ((type==ADJUST_NODE_POS) || (type==SET_NODE_POS))
	//							{
	//								//basePos = backupTranslations[k+kSeq->baseTranslation];//+node
	//								basePos = seqBackup->nodeTranslations[k];
	//								if (type==ADJUST_NODE_POS)
	//									newPos = basePos + curVal;
	//								else if (type==SET_NODE_POS)
	//									newPos = curVal;
	//								kShape->nodeTranslations[k+kSeq->baseTranslation] = newPos;
	//							} 
	//							else if ((type==ADJUST_NODE_ROT) || (type==SET_NODE_ROT))
	//							{
	//								curVal.x = mDegToRad(curVal.x);
	//								curVal.y = mDegToRad(curVal.y);
	//								curVal.z = mDegToRad(curVal.z);

	//								Ogre::Quaternion q((Ogre::Vector3)curVal);
	//								if (type==SET_NODE_ROT)
	//								{
	//									newQuat.set(q);
	//								} 
	//								else if (type==ADJUST_NODE_ROT)
	//								{
	//									Ogre::Quaternion temp;
	//									//baseQuat = backupRotations[kSeq->baseRotation+(node_matter_index*kSeq->numKeyframes)+i];
	//									baseQuat = seqBackup->nodeRotations[(mattersNode*kSeq->numKeyframes)+k];
	//									baseQuat.getOgre::Quaternion(&temp);
	//									temp *= q;
	//									newQuat.set(temp);
	//								}
	//								kShape->nodeRotations[kSeq->baseRotation+(mattersNode*kSeq->numKeyframes)+k] = newQuat;
	//							}
	//						}
	//					}
	//				}
	//			}
	//		}
	//		break;
	//	}
	//}
}

void fxFlexBody::getUltraframe(int seq, int frame, int node, int type,int *target,Ogre::Vector3 *outVal)
{
	//Ogre::Vector3 curVal,lastVal,nextVal;
	//int lastFrame,nextFrame;
	//int curLastFrameChecked = mLastFrameChecked;
	//mLastFrameChecked = frame;
	//lastFrame = nextFrame = 0;
	//outVal->zero();
	//TSShape *kShape = getShapeInstance()->getShape();

	//for (unsigned int i=0;i<mUltraframeSets.size();i++)
	//{
	//	if (mUltraframeSets[i].seq==seq)
	//	{

	//		int mattersNode = kShape->getNodeMattersIndex(seq,node);
	//		if ((mUltraframeSets[i].types[type]==false)||(mUltraframeSets[i].nodes[mattersNode]==false))
	//			return;

	//		ultraframe *uf,*lf;
	//		lf = NULL;
	//		uf = mUltraframeSets[i].lists[type][node];
	//		if (!uf) 
	//			return;
	//		while (uf) 
	//		{
	//			if (uf->frame < frame)
	//			{
	//				lf = uf;
	//				uf = uf->next;
	//			} else break;
	//		}
	//		if (uf->frame == frame)
	//		{
	//			outVal->set(uf->value.x,uf->value.y,uf->value.z);
	//			//if (type==IMPULSE_WEAPON_FORCE)
	//			//	*target = uf->target;
	//			//if (outVal->len()) //Con::printf("ultraframe exact hit! %f %f %f",outVal->x,outVal->y,outVal->z);
	//			return;
	//		} 
	//		else if (uf->frame > frame) 
	//		{
	//			if (!lf) 
	//				return;//Whoops!  My delete frame logic breaks here, removing frames from the array
	//			//so my pointers are no longer valid.  FIX!!
	//			lastFrame = lf->frame;
	//			lastVal = lf->value;
	//			nextFrame = uf->frame;
	//			nextVal = uf->value;
	//			////Con::printf("looking for missed frame: current %d lastframe %d nextframe %d",frame,curLastFrameChecked

	//			//if ((type==IMPULSE_FORCE)&&(curLastFrameChecked > lf->frame))
	//			//{//meaning we missed our force frame, so go back and grab it.
	//			//	outVal->set(lastVal.x,lastVal.y,lastVal.z);
	//			//	return;
	//			//} else if ((type==IMPULSE_WEAPON_FORCE)&&(curLastFrameChecked > lf->frame))
	//			//{
	//			//	*target = lf->target;
	//			//	outVal->set(lastVal.x,lastVal.y,lastVal.z);
	//			//	return;
	//			//}
	//			break;
	//		}
	//	}
	//}
	//if (nextFrame>lastFrame)
	//{
	//	curVal = lastVal + ((nextVal - lastVal)*((float)(frame-lastFrame)/(float)(nextFrame-lastFrame)));
	//	outVal->set(curVal.x,curVal.y,curVal.z);
	//	if (outVal->len()) //Con::errorf("last frame: %d, next frame %d, type %d, curVal %f %f %f",lastFrame,nextFrame,type,outVal->x,outVal->y,outVal->z);
	//	return;
	//} else return;
}

void fxFlexBody::clearUltraframe(int seq, int frame, int node)
{
	//Con::errorf("clearing ultraframe! seq %d frame %d node %d",seq,frame,node);

	//TSShape *kShape = getShapeInstance()->getShape();
	//for (unsigned int i=0;i<mUltraframeSets.size();i++)
	//{
	//	if (mUltraframeSets[i].seq==seq)
	//	{
	//		for (unsigned int type=0;type<ULTRAFRAME_TYPES;type++)
	//		{
	//			if (!mUltraframeSets[i].types[type]) continue;

	//			ultraframe *uf,*lf;
	//			lf = NULL;
	//			if (node>-1)
	//			{
	//				uf = mUltraframeSets[i].lists[type][node];
	//				//Con::errorf("checking type %d",type);
	//				if (!uf) 
	//					continue;
	//				while (uf) 
	//				{
	//					if (uf->frame < frame)
	//					{
	//						lf = uf;
	//						uf = uf->next;
	//					} else break;
	//				}
	//				if (uf->frame == frame)
	//				{
	//					saveUltraframe(seq,frame,node,type,0,Ogre::Vector3(0,0,0));
	//				}
	//			} else {
	//				for (unsigned int j=0;j<kShape->getNumMattersNodes(seq);j++)
	//				{
	//					if (!mUltraframeSets[i].nodes[j]) continue;

	//					uf = mUltraframeSets[i].lists[type][j];
	//					if (!uf) 
	//						continue;
	//					while (uf) 
	//					{
	//						if (uf->frame < frame)
	//						{
	//							lf = uf;
	//							uf = uf->next;
	//						} else break;
	//					}
	//					if (uf->frame == frame)
	//					{
	//						saveUltraframe(seq,frame,node,type,0,Ogre::Vector3(0,0,0));
	//					}
	//				}
	//			}
	//		}
	//	}
	//}
}

void fxFlexBody::dropUltraframe(int seq,int frame, int node, int type)
{//HERE: drop from database as well - (done in script)
	//TSShape *kShape = getShapeInstance()->getShape();
	//if ((seq>=0)&&(seq<mUltraframeSets.size()))
	//{
	//	ultraframe *uf,*lf;
	//	lf = NULL;
	//	if (node>-1)
	//	{
	//		uf = mUltraframeSets[seq].lists[type][node];
	//		if (!uf) 
	//			return;
	//		while (uf) 
	//		{
	//			if (uf->frame < frame)
	//			{
	//				lf = uf;
	//				uf = uf->next;
	//			} else break;
	//		}
	//		if (uf->frame == frame)
	//		{
	//			if ((lf!=uf)&&(uf->next))
	//			{
	//				lf->next = uf->next;//This would work even if we're the end frame,
	//				//since lf->next would then equal NULL, but we need to see if we're
	//				//an "area" or "region" or a "point" ultraframe.  Not done yet.
	//				mUltraframeSets[seq].frames.erase(uf);
	//				uf = lf->next;
	//				saveUltraframe(seq,frame,node,type,0,uf->value);
	//			}
	//			//saveUltraframe(seq,frame,node,type,0,Ogre::Vector3(0,0,0));
	//		}
	//	} else {
	//		for (unsigned int j=0;j<kShape->getNumMattersNodes(seq);j++)
	//		{
	//			if (!mUltraframeSets[seq].nodes[j]) continue;

	//			uf = mUltraframeSets[seq].lists[type][j];
	//			if (!uf) 
	//				continue;
	//			while (uf) 
	//			{
	//				if (uf->frame < frame)
	//				{
	//					////Con::errorf("frame: %d",uf->frame);
	//					lf = uf;
	//					uf = uf->next;
	//				} else break;
	//			}
	//			if (uf->frame == frame)
	//			{
	//				////Con::errorf("not dropping it yet, though.");
	//				lf->next = uf->next;//This would work even if we're the end frame,
	//				//since lf->next would then equal NULL, but we need to see if we're
	//				//an "area" or "region" or a "point" ultraframe.  Not done yet.
	//				mUltraframeSets[seq].frames.erase(uf);
	//				uf = lf->next;
	//				saveUltraframe(seq,uf->frame,j,type,0,uf->value);
	//				//saveUltraframe(seq,frame,node,type,0,Ogre::Vector3(0,0,0));
	//			}
	//		}
	//	}
	//}
}

void fxFlexBody::dropUltraframe(int seq, int frame, int node)
{

	//TSShape *kShape = getShapeInstance()->getShape();
	//if ((seq>=0)&&(seq<mUltraframeSets.size()))
	//{
	//	for (unsigned int type=0;type<ULTRAFRAME_TYPES;type++)
	//	{
	//		ultraframe *uf,*lf;
	//		lf = NULL;
	//		if (node>-1)
	//		{
	//			uf = mUltraframeSets[seq].lists[type][node];
	//			if (!uf) 
	//				continue;
	//			while (uf) 
	//			{
	//				if (uf->frame < frame)
	//				{
	//					lf = uf;
	//					uf = uf->next;
	//				} else break;
	//			}
	//			if (uf->frame == frame)
	//			{
	//				if ((lf!=uf)&&(uf->next))
	//				{
	//					lf->next = uf->next;//This would work even if we're the end frame,
	//					//since lf->next would then equal NULL, but we need to see if we're
	//					//an "area" or "region" or a "point" ultraframe.  Not done yet.
	//					mUltraframeSets[seq].frames.erase(uf);
	//					uf = lf->next;
	//					saveUltraframe(seq,frame,node,type,0,uf->value);
	//				}
	//				//saveUltraframe(seq,frame,node,type,0,Ogre::Vector3(0,0,0));
	//			}
	//		} else {
	//			for (unsigned int j=0;j<kShape->getNumMattersNodes(seq);j++)
	//			{
	//				if (!mUltraframeSets[seq].nodes[j]) continue;

	//				uf = mUltraframeSets[seq].lists[type][j];
	//				if (!uf) 
	//					continue;
	//				while (uf) 
	//				{
	//					if (uf->frame < frame)
	//					{
	//						////Con::errorf("frame: %d",uf->frame);
	//						lf = uf;
	//						uf = uf->next;
	//					} else break;
	//				}
	//				if (uf->frame == frame)
	//				{
	//					////Con::errorf("not dropping it yet, though.");
	//					lf->next = uf->next;//This would work even if we're the end frame,
	//					//since lf->next would then equal NULL, but we need to see if we're
	//					//an "area" or "region" or a "point" ultraframe.  Not done yet.
	//					//mUltraframeSets[seq].frames.erase(uf);
	//					//WHOOPS: erasing the actual frame data throws off all the rest of your pointers.
	//					//DOH!!  For now, just leave the frame data there, but skip it with ->next. 
	//					uf = lf->next;
	//					saveUltraframe(seq,uf->frame,j,type,0,uf->value);
	//					//saveUltraframe(seq,frame,node,type,0,Ogre::Vector3(0,0,0));
	//				}
	//			}
	//		}
	//	}
	//}
}

void fxFlexBody::saveUltraframes(int seq,const char *filename,bool append)
{
	//HERE, make an arbitrary custom text file format for now.  Save keyframe data with collada or bvh or fbx(?) later, 
	//possibly in addition to the original custom format.
	//NOW:  get name of keyframeSet, and save keyframes in database instead. 
	//EXCEPT:  database is kept up to date all the time, so there is no need for a special save function.  Commenting all.

	//if (strlen(filename)==0) { 
	//	//Con::errorf("saveUltraframes called with zero length filename."); 
	//	return;
	//}

	//FILE *fpw = NULL;
	//String keyframesFile(filename);
	//String keyframesExt(".keyframes");
	//if (!strstr(keyframesFile.c_str(),".keyframes")) keyframesFile += keyframesExt;

	//if (append)
	//	fpw = fopen(keyframesFile.c_str(),"a");
	//else 
	//	fpw = fopen(keyframesFile.c_str(),"w");                                                                                                                                                                                                                                                            

	//for (unsigned int i=0;i<mUltraframeSets.size();i++)
	//{
	//	if (mUltraframeSets[i].seq==seq)
	//	{
	//		ultraframe *uf;
	//		TSShape *kShape = getShapeInstance()->getShape();
	//		TSShape::Sequence kSeq = kShape->sequences[seq];
	//		fprintf(fpw,"Sequence: %s\n",kShape->getName(kSeq.nameIndex).c_str());
	//		
	//		////Con::errorf("found my sequence: %s  matters nodes %d",kShape->getName(kSeq.nameIndex).c_str(),kShape->getNumMattersNodes(seq));
	//		for (unsigned int type=0;type<ULTRAFRAME_TYPES;type++)
	//		{
	//			for (unsigned int node=0;node<kShape->getNumMattersNodes(seq);node++)
	//			{
	//				unsigned int nodeNum = kShape->getMattersNodeIndex(seq,node);
	//				uf = mUltraframeSets[i].lists[type][node];
	//				while (uf)
	//				{
	//					//Con::errorf("found a keyframe for node %d! %f %f %f",uf->node,uf->value.x,uf->value.y,uf->value.z);
	//					fprintf(fpw,"%d;%d;%d;%d;(%3.3f %3.3f %3.3f);\n",uf->frame,uf->node,uf->type,uf->target,uf->value.x,uf->value.y,uf->value.z);
	//					uf = uf->next;
	//				}
	//			}
	//		}
	//	}
	//}

	//fclose(fpw);
	//return;
}


void fxFlexBody::loadUltraframes(const char *filename)
{
	//if (strlen(filename)==0) { 
	//	//Con::errorf("loadUltraframes called with zero length filename."); 
	//	return;
	//}

	//char buf[255],seqname[255];
	//int fileSeq = -1;
	//		
	//int frame;
	//int node;
	//int type;
	//int target;
	//float x,y,z;
	//Ogre::Vector3 value;

	//TSShape *kShape = getShapeInstance()->getShape();
	//
	//FILE *fp = fopen(filename,"r");
	//if (fp==NULL) 
	//	return;

	////if (mUltraframeSets.size()==0)
	////	for (unsigned int i=0;i<kShape->sequences.size();i++)
	////		addUltraframeSet(i);

	////if (backupRotations.size()==0)
	////	backupSequenceData();

	//while (fgets(buf,255,fp))
	//{
	//	if (strncmp(buf,"Sequence",8)==0)
	//	{
	//		sscanf(buf,"Sequence: %s",&seqname);
	//		fileSeq = kShape->findSequence(seqname);
	//	} else {
	//		//for (unsigned int i=0;i<mUltraframeSets.size();i++)
	//		//	if (mUltraframeSets[i].seq==fileSeq)
	//		//		set=i;
	//		sscanf(buf,"%d;%d;%d;%d;(%f %f %f);",&frame,&node,&type,&target,&x,&y,&z);
	//		value.set(x,y,z);
	//		addUltraframe(fileSeq,frame,node,type,target,value);
	//	}
	//}

	////if (fileSeq!=seq) 
	////	//Con::warnf("Loading keyframes from a different sequence: %s",seqname);

	////if (fileSeq==-1)
	////{
	////	//Con::errorf("Could not find sequence %d.",fileSeq);
	////	fclose(fp);
	////	return;
	////}
	//
	////Hmm, should actually clear the whole ultraframeset before doing this, in case we already had some.
	////unsigned int set = -1;
	////if (set==-1) addUltraframeSet(fileSeq);

	//fclose(fp);
	//return;
}

int fxFlexBody::getNumUltraframes(int seq)
{

	//for (unsigned int i=0;i<mUltraframeSets.size();i++) 
	//{
	//	if (mUltraframeSets[i].seq==seq)
	//		return mUltraframeSets[i].frames.size();
	//}
	return 0;		
}

bool fxFlexBody::hasUltraframesForNode(int seq,int node)
{	
	//for (unsigned int i=0;i<mUltraframeSets.size();i++) 
	//{
	//	if (mUltraframeSets[i].seq==seq)
	//	{
	//		TSShape *kShape = getShapeInstance()->getShape();
	//		int mattersNode = kShape->getNodeMattersIndex(seq,node);
	//		return mUltraframeSets[i].nodes[mattersNode];
	//	}
	//} 
	return false;
}

bool fxFlexBody::hasUltraframesForType(int seq,int type)
{
	//for (unsigned int i=0;i<mUltraframeSets.size();i++) 
	//{
	//	if (mUltraframeSets[i].seq==seq)
	//	{
	//		return mUltraframeSets[i].types[type];
	//	}
	//} 
	return false;
}


void fxFlexBody::addPlaylistSeq(int seq,int repeats,float speed)
{
	////HERE: stop calling this, instead need to refresh whole list out of DB, we already
	////inserted into db anyway, and now they do not always add to the end of the list.
	//mPlaylist.increment();
	//mPlaylist[mPlaylist.size()-1].seq = seq;
	//mPlaylist[mPlaylist.size()-1].repeats = repeats;
	//mPlaylist[mPlaylist.size()-1].speed = speed;
	//setSeqCyclic(seq,false);//cyclic will kill onEndSequence, which we need to switch to next playlist sequence or repeat this one.
}

void fxFlexBody::dropPlaylistSeq(int playlist_sequence_id)
{
	//HERE: loop through mPlaylist from seq onward, moving each one up one, and then go mPlaylist.decrement();
	//ALSO, remove from database.  If last one on playlist, remove playlist as well.
	//NEW: seq is now sequence_id
	//FIX: do SQL only on the script side for stuff like this!  
	//SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(mPM)->getSQL();
	//if (!sql) return;
	//if (sql->OpenDatabase("EcstasyMotion.db"))
	//{
	//	char delete_query[512];
	//	int result;

	//	sprintf(delete_query,"DELETE FROM playlistSequence WHERE id=%d",playlist_sequence_id);
	//	result = sql->ExecuteSQL(delete_query);
	//	sql->CloseDatabase();
	//	delete sql;
	//}
	//clearPlaylist();

	//if (mPlaylistID) loadPlaylistById(mPlaylistID);
	//else loadPlaylist(dynamic_cast<nxPhysManager*>(mPM)->mSceneId);


	////if ((seq<0)||(seq>=mPlaylist.size()))
	////{
	////	//Con::errorf("trying to delete playlist sequence out of range.  Playlist size: %d",mPlaylist.size());
	////	return;
	////}
	////if (seq<mPlaylist.size()-1)
	////{
	////	for (unsigned int i=seq;i<mPlaylist.size()-1;i++)		
	////	{
	////		mPlaylist[i].seq = mPlaylist[i+1].seq;
	////		mPlaylist[i].repeats = mPlaylist[i+1].repeats;
	////		mPlaylist[i].speed = mPlaylist[i+1].speed;
	////	}
	////}
	////mPlaylist.decrement();
	//return;
}

void fxFlexBody::savePlaylistSeq(int index,int seq,int repeats,float speed)
{
	//if ((index>=0)&&(index<mPlaylist.size()))
	//{
	//	mPlaylist[index].seq = seq;
	//	mPlaylist[index].repeats = repeats;
	//	mPlaylist[index].speed = speed;
	//	setSeqCyclic(seq,false);
	//}
}

int fxFlexBody::getNumPlaylistSeqs()
{
	return mPlaylist.size();
}

int fxFlexBody::getPlaylistNum(int seq)
{
	//Con::errorf("looking for playlist with seq = %d",seq);
	for (int i=0;i<mPlaylist.size();i++)
	{
		if (mPlaylist[i].seq==seq)
			return i;
	}
	return -1;
}

int fxFlexBody::getPlaylistSeq(int index)
{
	if ((index>=0)&&(index<mPlaylist.size()))
		return mPlaylist[index].seq;
	else return -1;
}

int fxFlexBody::getPlaylistRepeats(int index)
{
	if ((index>=0)&&(index<mPlaylist.size()))
		return mPlaylist[index].repeats;
	else return 0;
}

float fxFlexBody::getPlaylistSpeed(int index)
{
	if ((index>=0)&&(index<mPlaylist.size()))
		return mPlaylist[index].speed;
	else return -1;
}


void fxFlexBody::clearPlaylist()
{
	if (mPlaylist.size())
	{
		mRunningPlaylist = false;
		mPlaylist.clear();
	}
}

void fxFlexBody::savePlaylist(const char *playlist_file)
{
	//FILE *fpw = NULL;

	//if (mPlaylist.size())
	//{
	//	TSShape *kShape = getShapeInstance()->getShape();
	//	TSShape::Sequence *kSeq;

	//	String playlistFile(playlist_file);
	//	String playlistExt(".playlist");
	//	if (!strstr(playlistFile.c_str(),".playlist")) playlistFile += playlistExt;
	//
	//	fpw = fopen(playlistFile.c_str(),"w");
	//	for (unsigned int i=0;i<mPlaylist.size();i++)
	//	{
	//		kSeq = &(kShape->sequences[mPlaylist[i].seq]);
	//		fprintf(fpw,"%s ;%d;%3.3f;\n",kShape->getName(kSeq->nameIndex).c_str(),mPlaylist[i].repeats,mPlaylist[i].speed);
	//	}
	//	fclose(fpw);
	//	//fxFlexBody *kFB = dynamic_cast<fxFlexBody*>(this);
	//	mPlaylistFile.clear();
	//	mPlaylistFile.insert(0,playlist_file);
	//} else {
	//	//Con::errorf("Error -- fxFlexBody::savePlaylist() --- No playlist loaded.");
	//}

	//return;
}

void fxFlexBody::loadPlaylist(const char *filename)
{
	////OBSOLETE: this is only for loading old .playlist files, remove as soon as you know no one is using them anymore.
	////Update this for actorPlaylist, if anyone does need it.
	//if (strlen(filename)==0) { 
	//	//Con::errorf("loadPlaylist called with zero length string."); 
	//	return;
	//}

	//TSShape *kShape = getShapeInstance()->getShape();
	//FILE *fp = fopen(filename,"r");
	//if (fp==NULL) return;

	//int seqCount = 0;
	//char seqName[255],buf[255];
	//int seq;
	//int repeats;
	//float speed;

	////First, create a playlist in the database, so we don't have to come here again.
	//mPM = physManagerCommon::getPM();
	//int scene_id = dynamic_cast<nxPhysManager*>(mPM)->mSceneId;
	//String scene_name = dynamic_cast<nxPhysManager*>(mPM)->mSceneName;
	//SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(mPM)->getSQL();

	//if (sql)
	//{
	//	if (sql->OpenDatabase("EcstasyMotion.db"))
	//	{
	//		char insert_query[512],playlist_id_query[512],sequence_id_query[512],playlistName[512];
	//		int result,sequence_id=0,playlist_id=0;
	//		sqlite_resultset *resultSet;

	//		result = sql->ExecuteSQL("BEGIN TRANSACTION;");

	//		sprintf(playlistName,"%s.%s.default",scene_name.c_str(),mActorName);
	//		
	//		sprintf(insert_query,"INSERT INTO playlist (skeleton_id,name) VALUES (%d,'%s');",
	//			mSkeletonID,playlistName);
	//		result = sql->ExecuteSQL(insert_query);
	//		
	//		sprintf(playlist_id_query,"SELECT id FROM playlist WHERE name='%s';",playlistName);
	//		result = sql->ExecuteSQL(playlist_id_query);
	//		resultSet = sql->GetResultSet(result);
	//		if (resultSet->iNumRows == 1)
	//			playlist_id = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);

	//		mPlaylist.clear();
	//		while (fgets(buf,255,fp))
	//		{
	//			sscanf(buf,"%s ;%d;%f;",&seqName,&repeats,&speed);
	//			seq = kShape->findSequence(seqName);
	//			sprintf(sequence_id_query,"SELECT id FROM sequence WHERE name='%s';",seqName);
	//			result = sql->ExecuteSQL(sequence_id_query);
	//			resultSet = sql->GetResultSet(result);
	//			if (resultSet->iNumRows == 1)
	//				sequence_id = strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10);

	//			if (sequence_id && playlist_id)
	//			{
	//				sprintf(insert_query,"INSERT INTO playlistSequence (playlist_id,sequence_id,sequence_order,repeats,speed) VALUES (%d,%d,%f,%d,%f);",
	//					playlist_id,sequence_id,(float)seqCount,repeats,speed);
	//				result = sql->ExecuteSQL(insert_query);
	//			}
	//			if ((seq>=0)&&(repeats>0)&&(speed!=0))
	//			{//really this is where the sql insert should go, in addPlaylistSeq, so it doesn't get repeated.
	//				addPlaylistSeq(seq,repeats,speed);
	//				seqCount++;
	//			}
	//		}
	//		fclose(fp);
	//		//NOW: we might have inserted an empty playlist, which we don't want.  If no playlistSequences exist, go back and delete the new playlist.
	//		sprintf(playlist_id_query,"SELECT id FROM playlistSequence WHERE playlist_id=%d;",playlist_id);
	//		result = sql->ExecuteSQL(playlist_id_query);
	//		resultSet = sql->GetResultSet(result);
	//		if (resultSet->iNumRows == 0)
	//		{
	//			sprintf(playlist_id_query,"DELETE FROM playlist WHERE id=%d;",playlist_id);
	//			result = sql->ExecuteSQL(playlist_id_query);
	//		}
	//		result = sql->ExecuteSQL("END TRANSACTION;");
	//		sql->CloseDatabase();
	//	}
	//	delete sql;
	//}

	//clearPlaylist();
	//mPlaylistFile.clear();
	//mPlaylistFile.insert(0,filename);
	////Con::printf("Finished loading playlist %s.",filename);
	//return;
}


void fxFlexBody::setPlaylist(int playlist_id)
{
	//if (mPlaylistID != playlist_id)
	//{
	//	SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(physManagerCommon::getPM())->getSQL();
	//	if (!sql) return;
	//	if (sql->OpenDatabase("EcstasyMotion.db"))
	//	{
	//		char name_query[512];
	//		int result;
	//		sqlite_resultset *resultSet;

	//		sprintf(name_query,"SELECT name FROM playlist WHERE id = %d;",playlist_id);
	//		result = sql->ExecuteSQL(name_query);
	//		resultSet = sql->GetResultSet(result);
	//		if (resultSet->iNumRows == 1)
	//		{
	//			mPlaylistName = resultSet->vRows[0]->vColumnValues[0];
	//			mPlaylistID = playlist_id;
	//			sql->CloseDatabase();//(Could move this sql call into loadPlaylistById below.)
	//			loadPlaylistById(playlist_id);
	//		}
	//		else 
	//		{	
	//			//Con::errorf("Error - found %d playlists with id = %d",resultSet->iNumRows,playlist_id);
	//		}
	//		//Con::printf("%s set new playlist: id %d name %s",mActorName,mPlaylistID,mPlaylistName);
	//	}		
	//	sql->CloseDatabase();
	//	delete sql;
	//}
}

///////////////////////////////////////////////////////////

void fxFlexBody::orientToPosition(Ogre::Vector3 target)
{
	//if (!(mIsNaN_F(target.x) || mIsNaN_F(target.y) || mIsNaN_F(target.z) ))
	//{
	//	TSShape *kShape = getShapeInstance()->getShape();
	//	Ogre::Vector3 groundPos,diff;
	//	Ogre::Matrix3 mat;
	//	Ogre::Quaternion arc;
	//	Ogre::Vector3 dir;

	//	//Con::printf("server %d orienting to position: %f %f %f",isServerObject(),target.x,target.y,target.z);
	//	diff = target - getPosition();
	//	diff.normalize();
	//	dir.set(0,1,0);
	//	arc.shortestArc(diff,dir);
	//	arc.setMatrix(&mat);
	//	Ogre::Vector3 myPos = getPosition();
	//	setTransform(mat);
	//	setPosition(myPos);
	//}
}


void fxFlexBody::moveToPosition(Ogre::Vector3 target,Ogre::String action)
{
	//Con::errorf("flexbody trying to move to pos: (%3.2f %3.2f %3.2f) using action: %s",target.x,target.y,target.z,action.c_str());
	//TSShape *kShape = getShapeInstance()->getShape();
	//Ogre::Vector3 groundPos,diff;
	//int seq = -1;
	//String seqAction;

	////First, look up our move function.  See if "action" is one of our PersonaActions, 
	////if so grab name from there.  Otherwise, just look for a sequence with this
	////name.  If that isn't there either then ditch.
	//SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(mPM)->getSQL();
	//if (sql)
	//{
	//	if (sql->OpenDatabase("EcstasyMotion.db"))
	//	{
	//		char sequence_name_query[512],seqName[255];
	//		int result,sequence_id=0;
	//		sqlite_resultset *resultSet;
	//		sprintf(sequence_name_query,
	//			"SELECT s.name FROM sequence s, personaAction pA, personaActionSequence pAS \
	//			WHERE s.id = pAS.sequence_id \
	//			AND pA.id = pAS.persona_action_id \
	//			AND pA.name='%s' AND pA.persona_id=%d;",
	//			action.c_str(),mPersonaID);
	//		result = sql->ExecuteSQL(sequence_name_query);
	//		if (result)
	//		{
	//			resultSet = sql->GetResultSet(result);
	//			////Con::printf("query: %s, results %d",sequence_name_query,resultSet->iNumRows);
	//			if (resultSet->iNumRows==1)
	//			{
	//				seqAction = resultSet->vRows[0]->vColumnValues[0];
	//				seq = kShape->findSequence(seqAction);
	//				//Con::printf("%s moveToPosition found a persona action sequence %s calling sequence %s %d",
	//					mActorName,action.c_str(),resultSet->vRows[0]->vColumnValues[0],seq);
	//			}
	//		}
	//		sql->CloseDatabase();
	//		delete sql;
	//	}
	//}

	//if (seq<0)
	//{
	//	seq = kShape->findSequence(action);
	//	seqAction = action;
	//}
	//if (seq < 0) return;

	////First, see if we are already within our targetThreshold, if so do nothing.
	////Except, whoops!  We need a raycast to figure out our groundPos.  But a physX raycast can't happen until after next worldstep, 
	////cuz we need to stop the physx thread in order to ask it, and then stop it again to find out what it found out, I think.  At 
	////least we need to stop it once.  So, possibly a torque raycast would be better.  For now, just widen the threshold to more than 
	////hip height, so we don't walk forever trying to get our hips to the ground, and move on.
	//diff =  target - getPosition();//mBodyParts[0]->getPosition()//can't use actual flexbody position because it
	////only changes once per walk cycle.  Next, add a mPositionNode, to track desired position node if it isn't root node.
	//if (diff.length() < mMoveThreshold) 
	//{
	//	////Con::printf("diff %f is already within moveThreshold %f , exiting",diff.length(),mMoveThreshold);	
	//	//Con::executef(this, "onReachTarget", scriptThis());
	//	return;
	//}

	//mMoveTarget = target;
	//mIsMoveTargeting = true;
	//mMoveSequence = seqAction;

	////Then, turn to point at the target (do it instantly for the first pass, later play turning animation and turn with the sequence.)
	//diff.z = 0.0;
	//diff.normalize();
	////Ogre::Matrix3 mat = getTransform();
	////Ogre::Quaternion q(mat);
	////Ogre::Vector3 dir;
	////q.mulP(Ogre::Vector3(0,1,0),&dir);//dir should now be Y vector multiplied by our world transform, i.e. the direction we are facing.

	//////Now, rotate around the Z axis so that our Y axis points at the target.
	////dir.normalize(); //just to make sure, should already be.
	////Ogre::Quaternion arc = q.shortestArc(diff,dir);
	////Ogre::Quaternion final;
	////final.mul(q,arc);

	////Ogre::Matrix3 finalMat;
	////q.setMatrix(&finalMat);
	////mat.mul(finalMat);
	////setTransform(mat);
	//Ogre::Matrix3 mat;
	//Ogre::Quaternion arc;
	//Ogre::Vector3 dir;
	//dir.set(0,1,0);
	//arc.shortestArc(diff,dir);
	//arc.setMatrix(&mat);
	//Ogre::Vector3 myPos = getPosition();
	//setTransform(mat);
	//setPosition(myPos);

	////Con::printf("Calling playthread from moveToPosition, seq %d",seq);
	//playThread(0,seqAction);
	//
	////Wow, well, hope that works.  Later, we will either apply the arc gradually over time while playing a turning animation that doesn't 
	////actually turn the body itself, or better yet we will play an anim that does turn itself, and then go back and measure over time to see
	////if we've hit our target arc yet. 
	////This function still only starts the move action, we also need to keep checking to see if we've reached it and then stopped, that will
	////have to be in onWorldStep.  And we will need to store our target position as a flexbody member as well.
}

void fxFlexBody::attackPosition(Ogre::Vector3 target,Ogre::String action)
{
	////Con::errorf("flexbody trying to attack pos: (%3.2f %3.2f %3.2f) using action: %s",target.x,target.y,target.z,action.c_str());
	//TSShape *kShape = getShapeInstance()->getShape();
	//Ogre::Vector3 groundPos,diff;
	//int seq = -1;
	//String seqAction;
	////First, look up our move function.  See if "action" is one of our PersonaActions, 
	////if so grab name from there.  Otherwise, just look for a sequence with this
	////name.  If that isn't there either then ditch.
	//SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(mPM)->getSQL();
	//if (sql)
	//{
	//	if (sql->OpenDatabase("EcstasyMotion.db"))
	//	{
	//		char sequence_name_query[512],seqName[255];
	//		int result,sequence_id=0;
	//		sqlite_resultset *resultSet;
	//		sprintf(sequence_name_query,
	//			"SELECT s.name FROM sequence s, personaAction pA, personaActionSequence pAS \
	//			WHERE s.id = pAS.sequence_id \
	//			AND pA.id = pAS.persona_action_id \
	//			AND pA.name='%s' AND pA.persona_id=%d;",
	//			action.c_str(),mPersonaID);
	//		result = sql->ExecuteSQL(sequence_name_query);
	//		resultSet = sql->GetResultSet(result);
	//		////Con::printf("query: %s, results %d, value %s",
	//		//	sequence_name_query,resultSet->iNumRows,resultSet->vRows[0]->vColumnValues[0]);
	//		if (resultSet->iNumRows==1)
	//		{
	//			seqAction = resultSet->vRows[0]->vColumnValues[0];
	//			seq = kShape->findSequence(seqAction);
	//			//Con::printf("%s attackPosition found a persona action sequence %s calling sequence %s",
	//				mActorName,action.c_str(),resultSet->vRows[0]->vColumnValues[0]);
	//		}
	//		sql->CloseDatabase();
	//		delete sql;
	//	}

	//}

	//if (seq<0)
	//{
	//	seq = kShape->findSequence(action);
	//	seqAction = action;
	//}
	//if (seq < 0) return;

	////First, see if we are already within our targetThreshold, if so do nothing.
	////Except, whoops!  We need a raycast to figure out our groundPos.  But a physX raycast can't happen until after next worldstep, 
	////cuz we need to stop the physx thread in order to ask it, and then stop it again to find out what it found out, I think.  At 
	////least we need to stop it once.  So, possibly a torque raycast would be better.  For now, just widen the threshold to more than 
	////hip height, so we don't walk forever trying to get our hips to the ground, and move on.
	//diff =  target - mBodyParts[0]->getPosition();//can't use actual flexbody position because it
	////only changes once per walk cycle.  Next, add a mPositionNode, to track desired position node if it isn't root node.
	//if (diff.length() < mMoveThreshold) return;

	//mMoveTarget = target;
	//mIsMoveTargeting = false;//turn this off so we just play the anim once, and don't turn 
	////it off immediately because we're within moveThreshold of the target...
	//mMoveSequence = seqAction;

	////Then, turn to point at the target (do it instantly for the first pass, later play turning animation and turn with the sequence.)
	//diff.z = 0.0;
	//diff.normalize();
	//Ogre::Matrix3 mat = this->getTransform();
	//Ogre::Quaternion q(mat);
	//Ogre::Vector3 dir;
	//q.mulP(Ogre::Vector3(0,1,0),&dir);//dir should now be Y vector multiplied by our world transform, i.e. the direction we are facing.

	////Now, rotate around the Z axis so that our Y axis points at the target.
	//dir.normalize(); //just to make sure, should already be.
	//Ogre::Quaternion arc = q.shortestArc(diff,dir);
	//Ogre::Quaternion final;
	//final.mul(q,arc);
	//Ogre::Matrix3 finalMat;
	//q.setMatrix(&finalMat);
	//mat.mul(finalMat);
	//this->setTransform(mat);

	//playThread(0,seqAction);
	//////Con::printf("attackPosition called playThread: %s, thread state: %d, atEnd %d  dir  %d, speed %f",
	////	seqAction.c_str(),getThreadState(0),isThreadAtEnd(0),getThreadDir(0),getThreadSpeed(0));
	//

	////Wow, well, hope that works.  Later, we will either apply the arc gradually over time while playing a turning animation that doesn't 
	////actually turn the body itself, or better yet we will play an anim that does turn itself, and then go back and measure over time to see
	////if we've hit our target arc yet. 
	////This function still only starts the move action, we also need to keep checking to see if we've reached it and then stopped, that will
	////have to be in onWorldStep.  And we will need to store our target position as a flexbody member as well.
}

void fxFlexBody::saveShapeConstructor(const char *filename)
{
	//TSShape *kShape = getShapeInstance()->getShape();
	//const String myFullPath = getShapeInstance()->mShapeResource.getPath().getFullPath();
	//TSShapeConstructor* ctor = TSShapeConstructor::findShapeConstructor( myFullPath );

	//if (ctor)
	//{
	//	if (strlen(filename)>0)
	//		ctor->writeChangeSet();//(filename)//T3D 1.1 beta 2
	//	else
	//		ctor->writeChangeSet();
	//}
}

void fxFlexBody::cropSequence(unsigned int seq,float start,float stop,const char *name)
{
	//Thread& st = mScriptThread[0];
 //   st.sequence = 0;
	//st.thread->setSequence(0,0.0);
	//st.state = fxFlexBody::Thread::Stop;
	//updateThread(st);

	//TSShape *kShape = getShapeInstance()->getShape();
	//////Con::errorf("cropping sequence");
	//kShape->cropSequence(seq,start,stop,name);
	//////Con::errorf("done cropping sequence");

	//FileStream *outstream;
	//String dsqPath(name);
	//String dsqExt(".dsq");
	//if (!strstr(dsqPath.c_str(),dsqExt.c_str())) dsqPath += dsqExt;
	////if (!gResourceManager->openFileForWrite(outstream,dsqPath.c_str())) {
	//if ((outstream = FileStream::createAndOpen( dsqPath.c_str(), Torque::FS::File::Write))==NULL) {
	//	//Con::printf("whoops, name no good: %s!",dsqPath.c_str()); 
	//} else {
	//	//kShape->exportSequences((Stream *)outstream);
	//	
	//	TSShape::Sequence & seq = kShape->sequences.last();
	//	kShape->exportSequence((Stream *)outstream,seq,1);
	//	outstream->close();
	//}

	////Now, load the sequence again, and drop the one we have... we hope this works.
	////loadDsq(dsqPath.c_str());
	////kShape->dropAllButOneSeq(kShape->sequences.size()-1);

	//////HERE: using my existing shapeconstructor to reload all sequences, would be the win.
	////const String myFullPath = getShapeInstance()->mShapeResource.getPath().getFullPath();
	////TSShapeConstructor* ctor = TSShapeConstructor::findShapeConstructor( myFullPath );
	////if (ctor)
	////{
	//////	//Con::errorf("found my shape constructor!  sequences %d",ctor->mSeqData.size());
	////	//ctor->??
	////	ctor->_onLoad( getShapeInstance()->mShapeResource );//see what happens if we pretend we're reloading...

	////	kShape->dropSequence(0);

	////	String pathPlusName(dsqPath);
	////	pathPlusName.insert(pathPlusName.length()," ");
	////	pathPlusName.insert(pathPlusName.length(),kShape->getName(kShape->sequences[0].nameIndex));
	////	////Con::executef(ctor,"addSequence",ctor->scriptThis(), dsqPath.c_str(),kShape->getName(kShape->sequences[0].nameIndex));
	////	//Except, so far this seems much easier to do from script.  Going back out to cropCrop() in script to do this.
	////}
}

void fxFlexBody::setSequenceFrames(unsigned int seq,unsigned int frames)
{
	//Con::errorf("setting sequence %d frames to %d",seq,frames);
	//TSShape *kShape = getShapeInstance()->getShape();
	//kShape->setSequenceFrames(seq,frames);
}

void fxFlexBody::convertAckToKork(int ackID, int seq)
{
	//SimObject *other;
	//other = Sim::findObject(ackID);
	//ShapeBase *otherShapeBase = dynamic_cast<ShapeBase*>(other);
	//
	//if (otherShapeBase)
	//{
	//	TSShape *ACK = otherShapeBase->getShapeInstance()->getShape();
	//	const String otherPath = otherShapeBase->getShapeInstance()->getShapeResource()->getPath().getPath();
	//	const String myPath = mShapeInstance->getShapeResource()->getPath().getPath();
	//	//Con::errorf("trying to convert sequence -- Kork path %s",myPath.c_str());
	//	//Con::errorf("other path: %s",otherPath.c_str());
	//	mShapeInstance->getShape()->convertSequence(ACK,"ack2kork",seq,myPath.c_str());
	//}
	//return;
}

void fxFlexBody::convertKorkDefault(int ackID)
{
	//SimObject *other;
	//other = Sim::findObject(ackID);
	//ShapeBase *otherShapeBase = dynamic_cast<ShapeBase*>(other);
	//
	//if (otherShapeBase)
	//{
	//	TSShape *ACK = otherShapeBase->getShapeInstance()->getShape();
	//	mShapeInstance->getShape()->convertDefaultPose(ACK,"ack2kork");
	//}
	//return;
}

void fxFlexBody::renameSequence(const char *oldName, const char *newName)
{
	//TSShape *kShape = getShapeInstance()->getShape();
	//int seqnum = kShape->findSequence(oldName);
	//int newNameIndex = kShape->addName(newName);
	//kShape->sequences[seqnum].nameIndex = newNameIndex;
}

bool fxFlexBody::hasRagdollBodyparts()
{
	bool hasRB = false;
	for (unsigned int i=0; i<mNumBodyParts; i++)
		if (!(mBodyParts[i]->mRB->getIsKinematic()))
			hasRB = true;

	return hasRB;
}


///////////////////////////////////////////////////////////////////////////
///// ARENA Streaming ///////////////////////////


//bool startArenaStreaming()
//{

	//// print version info
	//unsigned char ver[4];
	//natnetClient.NatNetVersion(ver);
	////Con::printf("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

	//natnetClient.SetMessageCallback(MessageHandler);
	//natnetClient.SetVerbosityLevel(Verbosity_Debug);
	//natnetClient.SetDataCallback( DataHandler, &natnetClient );	// this function will receive data from the server

	//int retCode;
	//retCode = natnetClient.Initialize("192.168.1.169", "192.168.1.169");
	//if (retCode != ErrorCode_OK)
	//{
	//	//Con::errorf("Unable to connect to server.  Error code: %d. Exiting", retCode);
	//	return 1;
	//}
	//else
	//{
	//	// print server info
	//	sServerDescription ServerDescription;
	//	memset(&ServerDescription, 0, sizeof(ServerDescription));
	//	natnetClient.GetServerDescription(&ServerDescription);
	//	if(!ServerDescription.HostPresent)
	//	{
	//		//Con::printf("Unable to connect to server. Host not present. Exiting.");
	//		return 1;
	//	}
	//	//Con::printf("[SampleClient] Server application info:\n");
	//	//Con::printf("Application: %s (ver. %d.%d.%d.%d)\n", ServerDescription.szHostApp, ServerDescription.HostAppVersion[0],
	//			ServerDescription.HostAppVersion[1],ServerDescription.HostAppVersion[2],ServerDescription.HostAppVersion[3]);
	//	//Con::printf("NatNet Version: %d.%d.%d.%d\n", ServerDescription.NatNetVersion[0], ServerDescription.NatNetVersion[1],
	//			ServerDescription.NatNetVersion[2], ServerDescription.NatNetVersion[3]);
	//	////Con::printf("Client IP:%s\n", szMyIPAddress);
	//	////Con::printf("Server IP:%s\n", szServerIPAddress);
	//	//Con::printf("Server Name:%s\n\n", ServerDescription.szHostComputerName);
	//}
	//////Con::printf(
	//return true;
//}

//bool startArenaStreaming(const char *localIP,const char *sourceIP)
//{
	// print version info
	//unsigned char ver[4];
	//char myIP[15],myLocalIP[15];

	//strcpy(myIP,sourceIP);
	//strcpy(myLocalIP,localIP);
	//natnetClient.NatNetVersion(ver);
	////Con::printf("IP VERSION... NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

	////Con::printf("source IP: %s",sourceIP);
	////Con::printf("local IP: %s",myLocalIP);
	//natnetClient.SetMessageCallback(MessageHandler);
	//natnetClient.SetVerbosityLevel(Verbosity_Debug);
	//natnetClient.SetDataCallback( DataHandler, &natnetClient );	// this function will receive data from the server

	//int retCode;
	//retCode = natnetClient.Initialize(myLocalIP, myIP);
	//if (retCode != ErrorCode_OK)
	//{
	//	//Con::errorf("Unable to connect to server.  Error code: %d. Exiting", retCode);
	//	return 1;
	//}
	//else
	//{		
	//	// print server info
	//	sServerDescription ServerDescription;
	//	memset(&ServerDescription, 0, sizeof(ServerDescription));
	//	natnetClient.GetServerDescription(&ServerDescription);
	//	if(!ServerDescription.HostPresent)
	//	{
	//		//Con::printf("Unable to connect to server. Host not present. Exiting.");
	//		return 1;
	//	}
	//	//Con::printf("[SampleClient] Server application info:\n");
	//	//Con::printf("Application: %s (ver. %d.%d.%d.%d)\n", ServerDescription.szHostApp, ServerDescription.HostAppVersion[0],
	//			ServerDescription.HostAppVersion[1],ServerDescription.HostAppVersion[2],ServerDescription.HostAppVersion[3]);
	//	//Con::printf("NatNet Version: %d.%d.%d.%d\n", ServerDescription.NatNetVersion[0], ServerDescription.NatNetVersion[1],
	//			ServerDescription.NatNetVersion[2], ServerDescription.NatNetVersion[3]);
	//	////Con::printf("Client IP:%s\n", szMyIPAddress);
	//	////Con::printf("Server IP:%s\n", szServerIPAddress);
	//	//Con::printf("Server Name:%s\n\n", ServerDescription.szHostComputerName);

	//	// Retrieve MarkerSets from server
	//	//Con::printf("\n\n[SampleClient] Requesting MarkerSet...");
	//	sDataDescriptions* pDataDefs = NULL;
	//	int nBodies = natnetClient.GetDataDescriptions(&pDataDefs);
	//	if(!pDataDefs)
	//	{
	//		//Con::printf("[SampleClient] Unable to retrieve MarkerSet.");
	//		//return 1;
	//	}
	//	else
	//	{
	//		//Con::printf("[SampleClient] Received MarkerSet:\n");
	//		sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[0].Data.MarkerSetDescription;
	//		//sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[1].Data.RigidBodyDescription;
	//		sSkeletonDescription* pSk = pDataDefs->arrDataDescriptions[1].Data.SkeletonDescription;
	//		////Con::printf("MarkerSet Name : %s, numbodies %d RigidBody ID : %d  parentID %d\n", 
	//		//	pMS->szName, nBodies, pRB->ID,pRB->parentID);
	//		if (pMS->szName) 
	//			for (int i=0; i < pMS->nMarkers; i++)
	//				//Con::printf("%s   szName  %s \n", pMS->szMarkerNames[i],pMS->szName);

	//		if (pSk->szName)
	//		{
	//			//Con::printf("skeleton name %s, rigid bodies %d",pSk->szName,pSk->nRigidBodies);
	//			for (int i=0; i < pSk->nRigidBodies; i++)
	//			{
	//				sRigidBodyDescription* pRB = &(pSk->RigidBodies[i]);
	//				//Con::printf("body %d ID %d  parentID %d \n",i,pRB->ID, pRB->parentID );
	//			}
	//		}
	//		//if (pRB->ID)
	//		//	//Con::printf("ParentID %d, offset %f %f %f\n", pRB->parentID,pRB->offsetx,pRB->offsety,pRB->offsetz);

	//	}
	//}
	//////Con::printf(
	//return true;
//}

//void stopArenaStreaming()
//{
	//natnetClient.Uninitialize();
	//return;
//}

// DataHandler receives data from the server
//void __cdecl fxFlexBody::ArenaDataHandler(sFrameOfMocapData* data, void* pUserData)
//void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData)
//{
		
	//////Con::printf("arena frame! \n");
	//nxPhysManager *kPM = ((nxPhysManager *)physManagerCommon::getPM());
	//if ((kPM->mIsExiting)||(kPM->mIsReallyExiting))
	//	return;
	////So, this isn't the most useful callback format for me, given that it can't be a ShapeBase function and it has to take 
	////those two arguments and return void -- limits the options somewhat.  But, that's why we have a gArenaBot ShapeBase *.
	//NatNetClient* pClient = (NatNetClient*) pUserData;

	//if (gArenaBot) //Avoid piled up arena frames.
	//{

	//	unsigned int currTick = ((fxFlexBody *)(gArenaBot))->mCurrTick;
	//	if (currTick == gLastArenaTick) 
	//		return;
	//	gLastArenaTick = currTick;
	//	float inch2m = 1.0/1000.0;// actually millimeters to meters
	//	inch2m *= 1.526; //TRIAL AND ERROR ... UNITS??
	//	TSShape *kShape = gArenaBot->getShapeInstance()->getShape();
	//	if (data->nSkeletons)
	//	{
	//		if (data->Skeletons[0].nRigidBodies)
	//		{
	//			int i=0;
	//			dynamic_cast<fxFlexBody *>(gArenaBot)->startArenaFrame();
	//			//gArenaBot->setPosition( data->RigidBodies[i].x * inch2m, data->RigidBodies[i].y * inch2m, data->RigidBodies[i].z * inch2m);
	//			Ogre::Matrix3 xform, matAdjX, matAdjY, matAdjZ, matAdjFinal, xformNew;
	//			Ogre::Quaternion quat,quatAdjX,quatAdjY,quatAdjZ,quatAdjFinal,Ogre::QuaternioninalInverse,quatAdjXInverse;
	//			Ogre::Vector3 pos;			
	//			quat.set(data->Skeletons[0].RigidBodyData[i].qx,data->Skeletons[0].RigidBodyData[i].qy, data->Skeletons[0].RigidBodyData[i].qz, data->Skeletons[0].RigidBodyData[i].qw);			
	//			quat.setMatrix(&xform);	

	//			matAdjX.set(Ogre::Vector3((M_PI/2.0),0,0));     
	//			quatAdjX.set(matAdjX);	
	//			quatAdjXInverse.set(matAdjX);
	//			quatAdjXInverse.inverse();
	//			matAdjZ.set(Ogre::Vector3(0,0,M_PI));
	//			quatAdjZ.set(matAdjZ);	

	//			//matAdjY.set(Ogre::Vector3(0,M_PI,0));
	//			//quatAdjY.set(matAdjY);

	//			matAdjFinal.mul(matAdjX,matAdjZ); 
	//			quatAdjFinal.set(matAdjFinal);

	//			xformNew.mul(matAdjFinal);
	//			xformNew.mul(xform);			
	//			xformNew.mul(matAdjFinal.inverse());  
	//			Ogre::QuaternioninalInverse.set(matAdjFinal);

	//			//xform.mul(xformAdj);			
	//			//pos.set(data->RigidBodies[i].x  * inch2m, data->RigidBodies[i].z * inch2m, data->RigidBodies[i].y * inch2m);			
	//			//swapping Z and Y for handedness... plus flip sign on X, hope that's correct.			

	//			//gArenaBot->setTransform(matAdjZ);			
	//			//gArenaBot->setPosition(pos);			

	//			////Con::printf("flexbody setting bodyparts from arena.");
	//			for(int i = 0; i < data->Skeletons[0].nRigidBodies; i++)
	//			{
	//				Ogre::Quaternion temp,temp2;
	//				Ogre::Vector3 newPos,testPos;
	//				//temp *= quatAdjFinal;

	//				pos.set(data->Skeletons[0].RigidBodyData[i].x * inch2m, data->Skeletons[0].RigidBodyData[i].z * -1.0 * inch2m, data->Skeletons[0].RigidBodyData[i].y * inch2m);
	//				testPos.set(data->Skeletons[0].RigidBodyData[i].x * inch2m, data->Skeletons[0].RigidBodyData[i].y * inch2m, data->Skeletons[0].RigidBodyData[i].z * inch2m);
	//				quat.set(data->Skeletons[0].RigidBodyData[i].qx,data->Skeletons[0].RigidBodyData[i].qy, data->Skeletons[0].RigidBodyData[i].qz, data->Skeletons[0].RigidBodyData[i].qw);
	//				matAdjZ.mulP(pos,&newPos);

	//				//temp *= quat;
	//				//temp *= Ogre::QuaternioninalInverse;
	//				//quat *= quatAdjZ;
	//				//quatAdjFinal.mul(quatAdjZ,quatAdjX);
	//				//quatAdjFinal.mul(quatAdjX,quatAdjZ);

	//				//temp.mul(quatAdjFinal,quat);
	//				//temp2.mul(temp,Ogre::QuaternioninalInverse);
	//				//temp.mul(Ogre::QuaternioninalInverse,quat);

	//				temp.mul(quatAdjFinal,quat);
	//				temp2.mul(temp,Ogre::QuaternioninalInverse);
	//				////Con::errorf("body %d q = %3.2f %3.2f %3.2f %3.2f",data->RigidBodies[i].ID,temp2.x,temp2.y,temp2.z,temp2.w);

	//				//temp.mul(quatAdjX,quat);
	//				//temp2.mul(temp,quatAdjXInverse);
	//				////Con::errorf("ID %d pos %3.2f %3.2f %3.2f q = %3.2f %3.2f %3.2f %3.2f",data->RigidBodies[i].ID,
	//				//	newPos.x,newPos.y,newPos.z,temp2.x,temp2.y,temp2.z,temp2.w);
	//				if ((data->Skeletons[0].RigidBodyData[i].ID >= 0)&&(gArenaBot))//filter out end marker bodies.
	//				{  // if (gArenaBot) won't work - it's a quick attempt to test for whether we're exiting
	//					//and have deleted our fxFlexBody already.  Need better test, nothing sets gArenaBot 
	//					//to null on the way out, it will be garbage and still pass the test.
	//					if ((data->Skeletons[0].RigidBodyData[i].ID==1)&&(i>23))//this is a weapon, ID = 1.
	//					{
	//						//"ID + i" : ID is 1 for weapons, but with "ID + i" I can get it up to 25, 26, 27, 28 etc.
	//						//which is more useful.
	//						dynamic_cast<fxFlexBody *>(gArenaBot)->setBodypartArena(data->Skeletons[0].RigidBodyData[i].ID + i, newPos, temp2);	
	//					} else 
	//						dynamic_cast<fxFlexBody *>(gArenaBot)->setBodypartArena(data->Skeletons[0].RigidBodyData[i].ID, newPos, temp2);//quat
	//				}
	//				dynamic_cast<fxFlexBody *>(gArenaBot)->updateNodes();
	//			}
	//		}
	//	}
	//}
//}


// MessageHandler receives NatNet error/debug messages
//void __cdecl MessageHandler(int msgType, char* msg)
//{
//	////Con::printf("\n%s\n", msg);
//}


//KFbxNode *gSkeletonLimbNodes[MAX_FLEX_NODES];

//KFbxNode* fxFlexBody::createFbxSkeleton( KFbxScene* pScene, const char* pName)
//{

	//TSShape *kShape = getShapeInstance()->getShape();

	//// Create skeleton root. 
	//KString lRootName(pName);
	//lRootName += "ShapeRootNode";
	//KFbxSkeleton* lSkeletonRootAttribute,*lSkeletonLimbNodeAttributes[MAX_FLEX_NODES];
	//KFbxNode* lSkeletonRoot;//,*gSkeletonLimbNodes[MAX_FLEX_NODES];

	//float fX,fY,fZ;
	//Ogre::Vector3 rootpos,row0,row1,row2;
	//Ogre::Quaternion q,qFinal;
	//Quat16 q16,q16final;
	//Ogre::Vector3 eul;
	//Ogre::Matrix3 mat;
	//HMatrix	hMatrix;
	//EulerAngles eulQ;

	//lSkeletonRootAttribute = KFbxSkeleton::Create(pScene, pName);
	//lSkeletonRootAttribute->SetSkeletonType(KFbxSkeleton::eROOT);

	//lSkeletonRoot = KFbxNode::Create(pScene,lRootName.Buffer());
	//lSkeletonRoot->SetNodeAttribute(lSkeletonRootAttribute);    
	//lSkeletonRoot->SetDefaultT(KFbxVector4(0.0, 0.0, 0.0));  
	////lSkeletonRoot->SetDefaultR(KFbxVector4(0.0, 0.0, 0.0));


	//// Create skeleton first limb node. 
	//KString lLimbNodeName1(pName);

	//////lLimbNodeName1 += kShape->getName(kShape->nodes[mBodyParts[0]->mNodeIndex].nameIndex).c_str();//"Hips";
	//lLimbNodeName1 += mBodyParts[0]->mNodeName.c_str();
	//lSkeletonLimbNodeAttributes[0] = KFbxSkeleton::Create(pScene,lLimbNodeName1);
	//////Con::printf("made limb node attributes 0.");

	//lSkeletonLimbNodeAttributes[0]->SetSkeletonType(KFbxSkeleton::eLIMB_NODE);
	//lSkeletonLimbNodeAttributes[0]->Size.Set(1.0);

 //  gSkeletonLimbNodes[0] = KFbxNode::Create(pScene,lLimbNodeName1.Buffer());
	////Con::printf("making first limb node name 1: %s",lLimbNodeName1.Buffer());

	//gSkeletonLimbNodes[0]->SetNodeAttribute(lSkeletonLimbNodeAttributes[0]);    
	//rootpos = kShape->defaultTranslations[mBodyParts[0]->mNodeIndex];//Base bodypart, NOT first node, if it's not a bodypart
	//rootpos *= 39.0;//Scale from meters to inches.
	//gSkeletonLimbNodes[0]->SetDefaultT(KFbxVector4(-rootpos.x, rootpos.z, rootpos.y));  

	////NOW, another big conversion from Quat16 to Matrix to functional Eulers...
	//q16 = getShapeInstance()->getShape()->defaultRotations[0];
	//q16.getOgre::Quaternion(&q);
	//q.setMatrix(&mat);
	//mat.getRow(0,&row0); mat.getRow(1,&row1); mat.getRow(2,&row2);
	//hMatrix[0][0] = row0.x; hMatrix[1][0] = row0.y; hMatrix[2][0] = row0.z; hMatrix[3][0] = 0.0;
	//hMatrix[0][1] = row1.x; hMatrix[1][1] = row1.y; hMatrix[2][1] = row1.z; hMatrix[3][1] = 0.0;
	//hMatrix[0][2] = row2.x; hMatrix[1][2] = row2.y; hMatrix[2][2] = row2.z; hMatrix[3][2] = 0.0;
	//hMatrix[0][3] = 0.0; hMatrix[1][3] = 0.0; hMatrix[2][3] = 0.0; hMatrix[3][3] = 1.0;
	//eulQ = Eul_FromHMatrix( hMatrix,EulOrdXYZs);
	//fX = eulQ.x * (180.0 / M_PI);
	//fY = -eulQ.z * (180.0 / M_PI);
	//fZ = -eulQ.y * (180.0 / M_PI);
	//gSkeletonLimbNodes[0]->SetDefaultR(KFbxVector4(fX, fY, fZ));

	//lSkeletonRoot->AddChild(gSkeletonLimbNodes[0]);
	////for (unsigned int i=1;i<mNumBodyParts;i++)
	//for (unsigned int i=1;i<kShape->nodes.size();i++)
	//{
	//	//int parentID = mBodyParts[i]->mParentBodyPart->mPartID;
	//	int parentID = kShape->nodes[i].parentIndex;
	//	KString lLimbNodeName(pName);
	//	//String nodeName;
	//	//char nodeName[255];
	//	//sprintf(nodeName,255,"LimbNode%d",i);
	//	//lLimbNodeName += mBodyParts[i]->mNodeName.c_str();
	//	lLimbNodeName += kShape->names[kShape->nodes[i].nameIndex];
	//	//lLimbNodeName += "LimbNode";
	//	lSkeletonLimbNodeAttributes[i] = KFbxSkeleton::Create(pScene,lLimbNodeName);
	//	lSkeletonLimbNodeAttributes[i]->SetSkeletonType(KFbxSkeleton::eLIMB_NODE);
	//	lSkeletonLimbNodeAttributes[i]->Size.Set(1.0);

	//	gSkeletonLimbNodes[i] = KFbxNode::Create(pScene,lLimbNodeName.Buffer());
	//	gSkeletonLimbNodes[i]->SetNodeAttribute(lSkeletonLimbNodeAttributes[i]);

	//	//Ogre::Vector3 pos = mBodyParts[i]->mCurrPosition;
	//	Ogre::Vector3 pos = getShapeInstance()->getShape()->defaultTranslations[i];
	//	pos *= 39.0;//Scale from meters to inches.
	//	//pos.set(0,40,0);
	//	gSkeletonLimbNodes[i]->SetDefaultT(KFbxVector4(-pos.x, pos.z, pos.y));
	//	
	//	q16 = getShapeInstance()->getShape()->defaultRotations[i];
	//	q16.getOgre::Quaternion(&q);
	//	q.setMatrix(&mat);
	//	mat.getRow(0,&row0); mat.getRow(1,&row1); mat.getRow(2,&row2);
	//	hMatrix[0][0] = row0.x; hMatrix[1][0] = row0.y; hMatrix[2][0] = row0.z; hMatrix[3][0] = 0.0;
	//	hMatrix[0][1] = row1.x; hMatrix[1][1] = row1.y; hMatrix[2][1] = row1.z; hMatrix[3][1] = 0.0;
	//	hMatrix[0][2] = row2.x; hMatrix[1][2] = row2.y; hMatrix[2][2] = row2.z; hMatrix[3][2] = 0.0;
	//	hMatrix[0][3] = 0.0; hMatrix[1][3] = 0.0; hMatrix[2][3] = 0.0; hMatrix[3][3] = 1.0;
	//	eulQ = Eul_FromHMatrix( hMatrix,EulOrdXYZs);
	//	fX = eulQ.x * (180.0 / M_PI);
	//	fY = -eulQ.z * (180.0 / M_PI);
	//	fZ = -eulQ.y * (180.0 / M_PI);
	//	gSkeletonLimbNodes[i]->SetDefaultR(KFbxVector4(fX, fY, fZ));

	//	gSkeletonLimbNodes[i]->SetDefaultR(KFbxVector4(0.0, 0.0, 0.0));//FIX! defaultRotations
	//	//Con::printf("flexbody node: %d, parent %d pos %f %f %f",i,parentID,pos.x, pos.y, pos.z);

	//	if (parentID > -1)
	//		gSkeletonLimbNodes[parentID]->AddChild(gSkeletonLimbNodes[i]);
	//	else 
	//		pScene->GetRootNode()->AddChild(gSkeletonLimbNodes[i]);
	//}

	//return lSkeletonRoot;
	//return NULL;
//}

//void fxFlexBody::animateFbxSkeleton(KFbxSdkManager* pSdkManager, KFbxScene* pScene, KFbxNode* pSkeletonRoot,int seq)
//{
	//KString kAnimName;
	////KFCurve *lCurves_X[MAX_FLEX_NODES],*lCurves_Y[MAX_FLEX_NODES],*lCurves_Z[MAX_FLEX_NODES],*lCurves_W[MAX_FLEX_NODES];// = NULL;
	////KFCurve *lCurve_T_X,*lCurve_T_Y,*lCurve_T_Z;
	//KTime kTime;
	//KFbxAnimCurve *kRotCurve_X,*kRotCurve_Y,*kRotCurve_Z;
	//KFbxAnimCurve *kTransCurve_X,*kTransCurve_Y,*kTransCurve_Z;
	//KFbxAnimCurveNode* kCurveNode;  
	//KFbxAnimStack *kAnimStack;
	//KFbxAnimLayer *kAnimLayer;

	//KFbxVector4 kT, kR;
	//KFbxXMatrix kM;

	//int kKeyIndex = 0;

	//KFbxNode *kRoot = pSkeletonRoot;
	//KFbxNode *kLimbNode;

	//Ogre::Quaternion q,qGround,qFinal;
	//Quat16 q16,q16ground,q16final;
	//Ogre::Vector3 eul;
	//Ogre::Vector3 rootpos,row0,row1,row2;

	//Ogre::Matrix3 mat;
	//float fX,fY,fZ;
	//HMatrix	hMatrix;
	//EulerAngles eulQ;

	//int rot_matters_count;
	//unsigned int node_matters[MAX_BVH_NODES];
	//int dtsNodeMatters[MAX_BVH_NODES];
	//int start_rot, start_trans, first_ground, num_keyframes;
	//TSShape *kShape = getShapeInstance()->getShape();
	//TSShape::Sequence *kSeq = &(kShape->sequences[seq]);

	//rot_matters_count = 0;
	//for (unsigned int i=0;i<kShape->nodes.size();i++) 
	//{
	//	if (kSeq->rotationMatters.test(i)) 
	//	{
	//		node_matters[rot_matters_count] = i;
	//		dtsNodeMatters[i] = rot_matters_count;
	//		rot_matters_count++;
	//		//Con::errorf("node %d matters!",i);
	//	}
	//}
	////Con::errorf("trying to save sequence: %d nameIndex %d name %s rotMattersCount %d",
	//	seq,kSeq->nameIndex,kShape->getName(kSeq->nameIndex).c_str(),rot_matters_count);


	//kAnimName = kShape->getName(kSeq->nameIndex).c_str();

	//kAnimStack = KFbxAnimStack::Create(pScene, kAnimName);
	//kAnimLayer = KFbxAnimLayer::Create(pScene, "Base Layer");
	//kAnimStack->AddMember(kAnimLayer);

	//float frameTime = kSeq->duration / kSeq->numKeyframes;
	//float currentTime;
	//float scale_factor = 1.0;

	//start_rot = kSeq->baseRotation;
	//start_trans = kSeq->baseTranslation;
	//first_ground = kSeq->firstGroundFrame;
	//num_keyframes = kSeq->numKeyframes;

	////if (rot_matters_count<mNumBodyParts) 
	////{
	////	//Con::errorf("FAIL! current bodyparts: %d, rot matters count: %d",mNumBodyParts,rot_matters_count);
	////} 
	////else
	////{

	////for (unsigned int i=0;i<kShape->nodes.size();i++)//
	//for (unsigned int i=0;i<rot_matters_count;i++)
	//{
	//	//if (!kSeq->rotationMatters.test(i))
	//	//{
	//	//	continue;
	//	//}
	//	currentTime = 0.0;
	//	kLimbNode = gSkeletonLimbNodes[node_matters[i]];
	//	//Con::printf("%d limb node: %d name %s limbNode %d",i,node_matters[i],kShape->getName(kShape->nodes[node_matters[i]].nameIndex).c_str(),kLimbNode);
	//	//HERE: need to take translation and rotation from base shape as well as base node of skeleton, for ground transform anims
	//	//(For first draft, ignoring shape transform.)
	//	if (i==0) 
	//	{
	//		kTransCurve_X = kLimbNode->LclTranslation.GetCurve<KFbxAnimCurve>(kAnimLayer, KFCURVENODE_T_X, true);
	//		kTransCurve_Y = kLimbNode->LclTranslation.GetCurve<KFbxAnimCurve>(kAnimLayer, KFCURVENODE_T_Y, true);
	//		kTransCurve_Z = kLimbNode->LclTranslation.GetCurve<KFbxAnimCurve>(kAnimLayer, KFCURVENODE_T_Z, true);
	//		//Con::printf("TransCurves %d limb node: %d name %s limbNode %d",i,node_matters[i],kShape->getName(kShape->nodes[node_matters[i]].nameIndex).c_str(),kLimbNode);
	//		//IMPORTANT: we are assuming a particular up axis (Y) and handedness (left) for FBX here 
	//		//             "native" (Maya?) layout.

	//		//// Translation X //////////////////////////////
	//		currentTime = 0.0;
	//		kTransCurve_X->KeyModifyBegin();
	//		for (unsigned int j=0;j<kSeq->numKeyframes;j++)
	//		{			
	//			if (kSeq->numGroundFrames==kSeq->numKeyframes) //Meaning it's one of ours, ground frames imported from sequence data.
	//			{  //FIX also grab groundRotations here & combine w/ base node
	//				rootpos = kShape->groundTranslations[kSeq->firstGroundFrame+j];
	//			} else { //Meaning either it doesn't have groundFrames or there is an odd number of them, figure out later.
	//				rootpos = kShape->nodeTranslations[kSeq->baseTranslation+j];
	//			}
	//			rootpos *= 39.0;//Scale from meters to inches.
	//			kTime.SetSecondDouble(currentTime);
	//			kKeyIndex = kTransCurve_X->KeyAdd(kTime);
	//			kTransCurve_X->KeySetValue(kKeyIndex, -rootpos.x);//HERE: FBX assumed Y up, left handed
	//			kTransCurve_X->KeySetInterpolation(kKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_LINEAR);
	//			currentTime += frameTime;
	//		}
	//		kTransCurve_X->KeyModifyEnd();

	//		//// Translation Y //////////////////////////////
	//		currentTime = 0.0;
	//		kTransCurve_Y->KeyModifyBegin();
	//		for (unsigned int j=0;j<kSeq->numKeyframes;j++)
	//		{			
	//			if (kSeq->numGroundFrames==kSeq->numKeyframes) //Meaning it's one of ours, ground frames imported from sequence data.
	//			{  //FIX also grab groundRotations here & combine w/ base node
	//				rootpos = kShape->groundTranslations[kSeq->firstGroundFrame+j];
	//			} else { //Meaning either it doesn't have groundFrames or there is an odd number of them, figure out later.
	//				rootpos = kShape->nodeTranslations[kSeq->baseTranslation+j];
	//			}
	//			rootpos *= 39.0;//Scale from meters to inches.
	//			kTime.SetSecondDouble(currentTime);
	//			kKeyIndex = kTransCurve_Y->KeyAdd(kTime);
	//			kTransCurve_Y->KeySetValue(kKeyIndex, rootpos.z);//HERE: FBX assumed Y up, left handed
	//			kTransCurve_Y->KeySetInterpolation(kKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_LINEAR);
	//			currentTime += frameTime;
	//		}
	//		kTransCurve_Y->KeyModifyEnd();

	//		//// Translation Z //////////////////////////////
	//		currentTime = 0.0;
	//		kTransCurve_Z->KeyModifyBegin();
	//		for (unsigned int j=0;j<kSeq->numKeyframes;j++)
	//		{			
	//			if (kSeq->numGroundFrames==kSeq->numKeyframes) //Meaning it's one of ours, ground frames imported from sequence data.
	//			{  //FIX also grab groundRotations here & combine w/ base node
	//				rootpos = kShape->groundTranslations[kSeq->firstGroundFrame+j];
	//			} else { //Meaning either it doesn't have groundFrames or there is an odd number of them, figure out later.
	//				rootpos = kShape->nodeTranslations[kSeq->baseTranslation+j];
	//			}
	//			rootpos *= 39.0;//Scale from meters to inches.
	//			kTime.SetSecondDouble(currentTime);
	//			kKeyIndex = kTransCurve_Z->KeyAdd(kTime);
	//			kTransCurve_Z->KeySetValue(kKeyIndex, rootpos.y);//HERE: FBX assumed Y up, left handed
	//			kTransCurve_Z->KeySetInterpolation(kKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_LINEAR);
	//			currentTime += frameTime;
	//		}
	//		kTransCurve_Z->KeyModifyEnd();
	//		//}
	//	}
	//	//lLimbNode->GetRotationOrder(kPivotSet,kRotOrder);
	//	// \enum ERotationOrder Rotation order flags.
	//	// - \e eEULER_XYZ
	//	// - \e eEULER_XZY
	//	// - \e eEULER_YZX 
	//	// - \e eEULER_YXZ 
	//	// - \e eEULER_ZXY 
	//	// - \e eEULER_ZYX
	//	// - \e eSPHERIC_XYZ
	//	//kLimbNode->SetRotationOrder(KFbxNode::eDESTINATION_SET,eSPHERIC_XYZ);//Maybe??//(KFbxNode::eSOURCE_SET,eEULER_XYZ)
	//	///////////////////////////////////////////////////////////

	//	kRotCurve_X = kLimbNode->LclRotation.GetCurve<KFbxAnimCurve>(kAnimLayer, KFCURVENODE_R_X, true);
	//	kRotCurve_Y = kLimbNode->LclRotation.GetCurve<KFbxAnimCurve>(kAnimLayer, KFCURVENODE_R_Y, true);
	//	kRotCurve_Z = kLimbNode->LclRotation.GetCurve<KFbxAnimCurve>(kAnimLayer, KFCURVENODE_R_Z, true);

	//	//// Rotation X //////////////////////////////
	//	currentTime = 0.0;
	//	kRotCurve_X->KeyModifyBegin();
	//	for (unsigned int j=0;j<kSeq->numKeyframes;j++)
	//	{
	//		q16ground = Ogre::Quaternion::IDENTITY;
	//		if (i==0)
	//		{
	//			if (kSeq->numGroundFrames==kSeq->numKeyframes) //Meaning it's one of ours, ground frames imported from sequence data.
	//			{  //FIX also grab groundRotations here & combine w/ base node
	//				q16ground = kShape->groundRotations[kSeq->firstGroundFrame+j];
	//			}
	//		}

	//		q16ground.getOgre::Quaternion(&qGround);
	//		q16 = kShape->nodeRotations[start_rot + (i * num_keyframes) + j];
	//		q16.getOgre::Quaternion(&q);
	//		qFinal.mul(qGround,q);
	//		qFinal.setMatrix(&mat);

	//		//eul = mat.toEuler();//NOPE, can't trust torque's toEuler to give you what you need...
	//		mat.getRow(0,&row0); mat.getRow(1,&row1); mat.getRow(2,&row2);
	//		hMatrix[0][0] = row0.x; hMatrix[1][0] = row0.y; hMatrix[2][0] = row0.z; hMatrix[3][0] = 0.0;
	//		hMatrix[0][1] = row1.x; hMatrix[1][1] = row1.y; hMatrix[2][1] = row1.z; hMatrix[3][1] = 0.0;
	//		hMatrix[0][2] = row2.x; hMatrix[1][2] = row2.y; hMatrix[2][2] = row2.z; hMatrix[3][2] = 0.0;
	//		hMatrix[0][3] = 0.0; hMatrix[1][3] = 0.0; hMatrix[2][3] = 0.0; hMatrix[3][3] = 1.0;
	//		eulQ = Eul_FromHMatrix( hMatrix,EulOrdXYZs);

	//		fX = eulQ.x * (180.0 / M_PI);

	//		kTime.SetSecondDouble(currentTime);
	//		kKeyIndex = kRotCurve_X->KeyAdd(kTime);
	//		kRotCurve_X->KeySetValue(kKeyIndex, fX);//HERE: FBX assumed Y up, left handed
	//		kRotCurve_X->KeySetInterpolation(kKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_CUBIC);
	//		currentTime += frameTime;
	//	}
	//	kRotCurve_X->KeyModifyEnd();

	//	//// Rotation Y //////////////////////////////
	//	currentTime = 0.0;
	//	kRotCurve_Y->KeyModifyBegin();
	//	for (unsigned int j=0;j<kSeq->numKeyframes;j++)
	//	{
	//		q16ground = Ogre::Quaternion::IDENTITY;
	//		if (i==0)
	//		{
	//			if (kSeq->numGroundFrames==kSeq->numKeyframes) //Meaning it's one of ours, ground frames imported from sequence data.
	//			{  //FIX also grab groundRotations here & combine w/ base node
	//				q16ground = kShape->groundRotations[kSeq->firstGroundFrame+j];
	//			}
	//		}
	//		q16ground.getOgre::Quaternion(&qGround);
	//		q16 = kShape->nodeRotations[start_rot + (i * num_keyframes) + j];
	//		q16.getOgre::Quaternion(&q);
	//		qFinal.mul(qGround,q);
	//		qFinal.setMatrix(&mat);
	//		//eul = mat.toEuler();//NOPE, can't trust torque's toEuler to give you what you need...
	//		mat.getRow(0,&row0); mat.getRow(1,&row1); mat.getRow(2,&row2);
	//		hMatrix[0][0] = row0.x; hMatrix[1][0] = row0.y; hMatrix[2][0] = row0.z; hMatrix[3][0] = 0.0;
	//		hMatrix[0][1] = row1.x; hMatrix[1][1] = row1.y; hMatrix[2][1] = row1.z; hMatrix[3][1] = 0.0;
	//		hMatrix[0][2] = row2.x; hMatrix[1][2] = row2.y; hMatrix[2][2] = row2.z; hMatrix[3][2] = 0.0;
	//		hMatrix[0][3] = 0.0; hMatrix[1][3] = 0.0; hMatrix[2][3] = 0.0; hMatrix[3][3] = 1.0;
	//		eulQ = Eul_FromHMatrix( hMatrix,EulOrdXYZs);

	//		fY = -eulQ.z * (180.0 / M_PI);

	//		kTime.SetSecondDouble(currentTime);
	//		kKeyIndex = kRotCurve_Y->KeyAdd(kTime);
	//		kRotCurve_Y->KeySetValue(kKeyIndex, fY);//HERE: FBX assumed Y up, left handed
	//		kRotCurve_Y->KeySetInterpolation(kKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_CUBIC);
	//		currentTime += frameTime;
	//	}
	//	kRotCurve_Y->KeyModifyEnd();

	//	//// Rotation Z //////////////////////////////////
	//	currentTime = 0.0;
	//	kRotCurve_Z->KeyModifyBegin();
	//	for (unsigned int j=0;j<kSeq->numKeyframes;j++)
	//	{
	//		q16ground = Ogre::Quaternion::IDENTITY;
	//		if (i==0)
	//		{
	//			if (kSeq->numGroundFrames==kSeq->numKeyframes) //Meaning it's one of ours, ground frames imported from sequence data.
	//			{  //FIX also grab groundRotations here & combine w/ base node
	//				q16ground = kShape->groundRotations[kSeq->firstGroundFrame+j];
	//			}
	//		}
	//		q16ground.getOgre::Quaternion(&qGround);
	//		q16 = kShape->nodeRotations[start_rot + (i * num_keyframes) + j];
	//		q16.getOgre::Quaternion(&q);
	//		qFinal.mul(qGround,q);
	//		qFinal.setMatrix(&mat);
	//		//eul = mat.toEuler();//NOPE, can't trust torque's toEuler to give you what you need...
	//		mat.getRow(0,&row0); mat.getRow(1,&row1); mat.getRow(2,&row2);
	//		hMatrix[0][0] = row0.x; hMatrix[1][0] = row0.y; hMatrix[2][0] = row0.z; hMatrix[3][0] = 0.0;
	//		hMatrix[0][1] = row1.x; hMatrix[1][1] = row1.y; hMatrix[2][1] = row1.z; hMatrix[3][1] = 0.0;
	//		hMatrix[0][2] = row2.x; hMatrix[1][2] = row2.y; hMatrix[2][2] = row2.z; hMatrix[3][2] = 0.0;
	//		hMatrix[0][3] = 0.0; hMatrix[1][3] = 0.0; hMatrix[2][3] = 0.0; hMatrix[3][3] = 1.0;
	//		eulQ = Eul_FromHMatrix( hMatrix,EulOrdXYZs);

	//		fZ = -eulQ.y * (180.0 / M_PI);

	//		kTime.SetSecondDouble(currentTime);
	//		kKeyIndex = kRotCurve_Z->KeyAdd(kTime);
	//		kRotCurve_Z->KeySetValue(kKeyIndex, fZ);//HERE: FBX assumed Y up, left handed
	//		kRotCurve_Z->KeySetInterpolation(kKeyIndex, KFbxAnimCurveDef::eINTERPOLATION_CUBIC);
	//		currentTime += frameTime;
	//	}
	//	kRotCurve_Z->KeyModifyEnd();
	//}
//}

//KFbxNode* fxFlexBody::createFbxMesh( KFbxSdkManager* kSdkManager, KFbxScene* pScene, KFbxNode* kRoot, const char* pName)
//{	
	//Ogre::Vector3 p;
	//Ogre::Vector3 vertex,d;
	//Ogre::Quaternion qF;
	//Ogre::Matrix3  kTransform;
	//int nodeIndex;

	////HERE: for multiple-object models, we may need to create a node to serve as the root for the 
	////whole model, and then add another node for each mesh object.
	//KFbxNode* kNode = KFbxNode::Create(pScene,pName);

	////Each object will need its own KFbxMesh, layers, etc.
	//KFbxMesh* kMesh = KFbxMesh::Create(pScene, pName);
	//kNode->SetNodeAttribute(kMesh);

	//kNode->SetShadingMode(KFbxNode::eTEXTURE_SHADING);
	//
	//KFbxLayer* kLayer = kMesh->GetLayer(0);
	//if (kLayer == NULL)
	//{
	//	kMesh->CreateLayer();
	//	kLayer = kMesh->GetLayer(0);
	//}

	////// Set texture mapping for diffuse channel.
	//KFbxLayerElementTexture* kTextureDiffuseLayer=KFbxLayerElementTexture::Create(kMesh, "base.powderkeg.png");
	//kTextureDiffuseLayer->SetMappingMode(KFbxLayerElement::eALL_SAME);//KFbxLayerElement::eALL_SAME);
	//kTextureDiffuseLayer->SetReferenceMode(KFbxLayerElement::eDIRECT);
	//kLayer->SetTextures(KFbxLayerElement::eDIFFUSE_TEXTURES, kTextureDiffuseLayer);

	//KFbxLayerElementNormal* kLayerElementNormal= KFbxLayerElementNormal::Create(kMesh, "");
	//kLayerElementNormal->SetMappingMode(KFbxLayerElement::eBY_CONTROL_POINT);
	//kLayerElementNormal->SetReferenceMode(KFbxLayerElement::eDIRECT);

	//KFbxLayerElementUV* kUVDiffuseLayer = KFbxLayerElementUV::Create(kMesh, "DiffuseUV");
	//kUVDiffuseLayer->SetMappingMode(KFbxLayerElement::eBY_CONTROL_POINT);
	//kUVDiffuseLayer->SetReferenceMode(KFbxLayerElement::eDIRECT);
	//kLayer->SetUVs(kUVDiffuseLayer, KFbxLayerElement::eDIFFUSE_TEXTURES);

	//// Set material mapping.
	//KFbxLayerElementMaterial* lMaterialLayer=KFbxLayerElementMaterial::Create(kMesh, "");
	//lMaterialLayer->SetMappingMode(KFbxLayerElement::eALL_SAME);
	//lMaterialLayer->SetReferenceMode(KFbxLayerElement::eDIRECT);
	//kLayer->SetMaterials(lMaterialLayer);

	//TSShape *kShape = mShapeInstance->mShape;
	//const TSDetail *detail = &kShape->details[0];
	//int ss = detail->subShapeNum;
	//int od = detail->objectDetailNum;
	//int vc = 0;//vertex count
	//int mc = 0;//mesh count

	//FILE *debug = fopen("debug.txt","w");

	//const char *name;
	//int start = kShape->subShapeFirstObject[ss];
	//int end   = kShape->subShapeNumObjects[ss] + start;

	//TSSkinMesh *mesh;
	////Have to tell fbx how many points in advance, so first a pass just to count vertices...
	////bool hasSkinMesh = false;
	//for (unsigned int j = start; j < end; j++)   
	//{
	//	mesh = (TSSkinMesh *)mShapeInstance->mMeshObjects[j].getMesh(od);
	//	if (mesh) //HMMM, seems like we can treat a mesh as a skinmesh, at least for purposes
	//	{// of extracting the verts and indices?  In which case it would be simpler to write it once...
	//		
	//		vc += mesh->mNumVerts;
	//		//hasSkinMesh = true;
	//	}// else {
	//	//	TSMesh *mesh = mShapeInstance->mMeshObjects[j].getMesh(od);
	//	//	if (mesh) {
	//	//		//Con::errorf("Found a TSMesh!!! type: %d, verts %d",
	//	//		mesh->mNumVerts,mesh->mNumVerts);
	//	//		vc += mesh->mNumVerts;
	//	//	}
	//	//}
	//}
	////if (mesh)//(!hasSkinMesh)
	////{
	////Con::errorf("Initializing control points:  %d",vc);
	//kMesh->InitControlPoints(vc);
	//KFbxVector4* kControlPoints = kMesh->GetControlPoints();

	////then the real pass in which we assign the coordinates from the mesh vertices.
	//vc = 0;
	//for (unsigned int j = start; j < end; j++)   
	//{
	//	name = kShape->names[mShapeInstance->mMeshObjects[j].object->nameIndex];

	//	//First pass - try this on a rigid body with a mesh, rather than a skinmesh.
	//	mesh = NULL;
	//	mesh = (TSSkinMesh *)mShapeInstance->mMeshObjects[j].getMesh(od);
	//	if (!mesh)
	//		continue;

	//	nodeIndex = kShape->objects[j].nodeIndex;
	//	//if (skinmesh) 
	//	//{
	//	////Con::errorf("Wowzers, we found a real live skin mesh!!! type: %d, verts %d",
	//	//	skinmesh->SkinMeshType,skinmesh->mNumVerts,skinmesh->mNumVerts);
	//	//} else {
	//	
	//	Ogre::Vector3 kCenter = mesh->getCenter();
	//	Ogre::Vector3 sc = getScale();

	//	 ////////////////////////  TSMesh  ///////////////////////////////////
	//	if (nodeIndex>-1)  // Placeholder, still not convinced I'm not going to need different sections
	//	{ // for skinmesh and mesh types. For now treating everything as skinmesh...

	//		//TSMesh *mesh = mShapeInstance->mMeshObjects[j].getMesh(od);
	//		//unsigned int *kIndices;

	//		kShape->defaultRotations[nodeIndex].getOgre::Quaternion(&qF);
	//		qF.setMatrix(&kTransform);	
	//		kTransform.setPosition(kShape->defaultTranslations[nodeIndex]);

	//		//Con::printf("Found a TSMesh %d:  %s numVerts %d indices %d",
	//			j,name,mesh->mNumVerts,mesh->indices.size());

	//		TSMesh::TSMeshVertexArray kVertArray = mesh->mVertexData;
	//		//mesh->indices;
	//		//mi->SetNbVertices( mVertexData.isReady() ? mesh->mNumVerts : verts.size() );
	//		//for (unsigned int k=0;k<mesh->verts.size();k++) //Uh oh, major changes to TSMesh
	//		//vc = 0;
	//		for (unsigned int k=0;k<mesh->mNumVerts;k++)
	//		{
	//			//vertex = mesh->verts[k];
	//			vertex.set(kVertArray[k].vert().x,kVertArray[k].vert().y,kVertArray[k].vert().z);
	//			vertex *= sc;//kVertArray[k].tangent, .normal, .color, .tvert, ...
	//			if (kTransform) kTransform.mulP(vertex,&d);
	//			else d = vertex;
	//			//kRB->mVerts.increment();//HERE: is this going to work? 
	//			//kRB->mVerts[kRB->mVerts.size()-1]=d;//vertex[MAYBE?]
	//			d *= 39.0;//HERE: inches/meters radio button in FBX rollout?

	//			kControlPoints[vc] = KFbxVector4(-d.x,d.z,d.y);//Assuming Y up left-handedness in FBX.

	//			Point2F tempTVert = kVertArray[k].tvert();
	//			Point2F tempTVert2 = kVertArray[k].tvert2();
	//			if (tempTVert.x<0) tempTVert.x=0; // we are getting very small negative
	//			if (tempTVert.y<0) tempTVert.y=0; // numbers in a field that should be 0-1
	//			//if (tempTVert2.x<0) tempTVert2.x=0; // we are getting very small negative
	//			//if (tempTVert2.y<0) tempTVert2.y=0; // numbers in a field that should be 0-1
	//			//if (tempTVert2.x>1.0) tempTVert2.x=1.0; // we are getting very small negative
	//			//if (tempTVert2.y>1.0) tempTVert2.y=1.0; // numbers in a field that should be 0-1
	//			KFbxVector2 kTVert(tempTVert.x,1-tempTVert.y);
	//			kUVDiffuseLayer->GetDirectArray().Add(kTVert);

	//			fprintf(debug,"vertices %d: %f %f %f tvert %f %f tvert2 %f %f \n",
	//				vc,-d.x,d.z,d.y,kTVert.mData[0],kTVert.mData[1],tempTVert2.x,tempTVert2.y);

	//			KFbxVector4 kNormal(-kVertArray[k].normal().x,kVertArray[k].normal().z,kVertArray[k].normal().y);
	//			kLayerElementNormal->GetDirectArray().Add(kNormal);

	//			vc++;
	//		}
	//		//Now, put together the polygons:
	//		if (mesh->smUseOneStrip)
	//		{
	//			for (unsigned int k=2;k<mesh->indices.size();k++)//
	//			{
	//				////Con::printf("Tri strip, indices %d %d %d %d",k,mesh->indices[k-2],mesh->indices[k-1],mesh->indices[k]);
	//				if (k%2==0)
	//				{//Winding apparently flips every time...
	//					kMesh->BeginPolygon();
	//					kMesh->AddPolygon(mesh->indices[k]); // Control point 0
	//					kMesh->AddPolygon(mesh->indices[k-1]); // Control point 1
	//					kMesh->AddPolygon(mesh->indices[k-2]); // Control point 2
	//					kMesh->EndPolygon();
	//				} else {
	//					kMesh->BeginPolygon();
	//					kMesh->AddPolygon(mesh->indices[k-2]); // Control point 0
	//					kMesh->AddPolygon(mesh->indices[k-1]); // Control point 1
	//					kMesh->AddPolygon(mesh->indices[k]); // Control point 2
	//					kMesh->EndPolygon();
	//				}
	//			}
	//			//for (unsigned int k=mesh->indices.size()-1;k>1;k--)//
	//			//{
	//			//	kMesh->BeginPolygon();
	//			//	kMesh->AddPolygon(mesh->indices[k]); // Control point 0
	//			//	kMesh->AddPolygon(mesh->indices[k-1]); // Control point 1
	//			//	kMesh->AddPolygon(mesh->indices[k-2]); // Control point 2
	//			//	kMesh->EndPolygon();
	//			//}
	//		} else {
	//			//Con::errorf("WARNING: this is not a tri-strip model!  Time to deal with smUseTriangles.");
	//		}
	//	} else { ////////////////////////  TSSkinMesh  ///////////////////////////////////

	//		TSMesh::TSMeshVertexArray kVertArray = mesh->mVertexData;

	//		//Con::printf("Found a TSSkinMesh %d: %s nodes %d verts %d norms %d weights %d vertArray verts %d",
	//			j,name,mesh->batchData.nodeIndex.size(),mesh->batchData.initialVerts.size(),
	//		  mesh->batchData.initialNorms.size(),mesh->weight.size(),kVertArray.size());

	//		//Hmm, multiply verts by initial transform of base node?  Trying w/o first.
	//		//vc = 0;
	//		for (unsigned int k=0;k<mesh->batchData.initialVerts.size();k++)
	//		{
	//		//	//Con::printf("skinmesh vert: %f %f %f",mesh->batchData.initialVerts[k].x,
	//		//		mesh->batchData.initialVerts[k].y,mesh->batchData.initialVerts[k].z);
	//			vertex.set(mesh->batchData.initialVerts[k].x,mesh->batchData.initialVerts[k].y,mesh->batchData.initialVerts[k].z);
	//			vertex *= sc;
	//			d = vertex;
	//			d *= 39.0;//HERE: inches/meters radio button in FBX rollout?
	//			kControlPoints[vc] = KFbxVector4(-d.x,d.z,d.y);

	//			Point2F tempTVert = kVertArray[k].tvert();
	//			Point2F tempTVert2 = kVertArray[k].tvert2();
	//			if (tempTVert.x<0) tempTVert.x=0; // we are getting very small negative
	//			if (tempTVert.y<0) tempTVert.y=0; // numbers in a field that should be 0-1
	//			KFbxVector2 kTVert(tempTVert.x,1-tempTVert.y);
	//			kUVDiffuseLayer->GetDirectArray().Add(kTVert);

	//			KFbxVector4 kNormal(-kVertArray[k].normal().x,kVertArray[k].normal().z,kVertArray[k].normal().y);
	//			kLayerElementNormal->GetDirectArray().Add(kNormal);
	//			
	//			fprintf(debug,"vertices %d: %f %f %f tvert %f %f tvert2 %f %f \n",
	//				vc,-d.x,d.z,d.y,kTVert.mData[0],kTVert.mData[1],tempTVert2.x,tempTVert2.y);

	//			vc++;
	//		}

	//		//Now, put together the polygons:
	//		if (mesh->smUseOneStrip)
	//		{
	//			for (unsigned int k=2;k<mesh->indices.size();k++)//
	//			{
	//				////Con::printf("Tri strip, indices %d %d %d %d",k,mesh->indices[k-2],mesh->indices[k-1],mesh->indices[k]);
	//				if (k%2==0)
	//				{//Winding apparently flips every time...
	//					kMesh->BeginPolygon();
	//					kMesh->AddPolygon(mesh->indices[k]); // Control point 0
	//					kMesh->AddPolygon(mesh->indices[k-1]); // Control point 1
	//					kMesh->AddPolygon(mesh->indices[k-2]); // Control point 2
	//					kMesh->EndPolygon();
	//				} else {
	//					kMesh->BeginPolygon();
	//					kMesh->AddPolygon(mesh->indices[k-2]); // Control point 0
	//					kMesh->AddPolygon(mesh->indices[k-1]); // Control point 1
	//					kMesh->AddPolygon(mesh->indices[k]); // Control point 2
	//					kMesh->EndPolygon();
	//				}
	//			}
	//		} else {
	//			//Con::errorf("WARNING: this is not a tri-strip model!  Time to deal with smUseTriangles.");
	//		}
	//		kMesh->RemoveBadPolygons();

	//		//Attach verts to bones
	//		KFbxCluster *kClusters[500];//FIX - make Vector
	//		int nodeIndices[500];
	//		int clusterIndices[500];
	//		int clusterCount = 0;
	//		
	//		for(unsigned int i=0; i < kShape->nodes.size(); i++) 
	//		{
	//			for (unsigned int jn=0;jn<mesh->batchData.nodeIndex.size();jn++)
	//			{
	//				if (mesh->batchData.nodeIndex[jn] == i)
	//				{ 
	//					kClusters[clusterCount] = KFbxCluster::Create(kSdkManager,"");
	//					kClusters[clusterCount]->SetLink(gSkeletonLimbNodes[i]);
	//					kClusters[clusterCount]->SetLinkMode(KFbxCluster::eTOTAL1);//eADDITIVE);//eNORMALIZE);//
	//					nodeIndices[jn] = i;
	//					clusterIndices[jn] = clusterCount;
	//					fprintf(debug,"node %d cluster %d for %s \n",i,clusterCount,kShape->names[kShape->nodes[i].nameIndex].c_str());
	//					clusterCount++;
	//				}
	//			}
	//		}

	//		for(unsigned int j=0; j < mesh->weight.size(); j++) 
	//		{
	//			kClusters[clusterIndices[mesh->boneIndex[j]]]->AddControlPointIndex(mesh->vertexIndex[j],mesh->weight[j]);
	//			//if (j<500)
	//			fprintf(debug,"cluster control point: bone %d vert %d weight %f  \n",nodeIndices[mesh->boneIndex[j]],
	//				mesh->vertexIndex[j],mesh->weight[j]);//,mesh->batchData.nodeIndex[j]);
	//		}

	//		KFbxSkin* lSkin = KFbxSkin::Create(kSdkManager, "");
	//		for(unsigned int i=0; i < clusterCount; i++) 
	//		{//Here (or above): should be checking for nodes that don't influence any verts, don't 
	//			//   create clusters for them.
	//			lSkin->AddCluster(kClusters[i]);
	//		}
	//		
	//		kMesh->AddDeformer(lSkin);


	//		//KArrayTemplate<KFbxNode*> lClusteredFbxNodes;
	//		//
	//		//lClusterCount=lSkin->GetClusterCount();
	//		//for (j=0; j<lClusterCount; ++j)
	//		//{
	//		//	KFbxNode* lClusterNode = lSkin->GetCluster(j)->GetLink();
	//		//	AddNodeRecursively(lClusteredFbxNodes, lClusterNode);
	//		//}
	//		//  lClusteredFbxNodes.Add(kMesh);

	//		//Set Bind Pose
	//		KFbxNode*  lKFbxNode;
	//		KFbxMatrix lBindMatrix;
	//		KFbxVector4 nodePos,row;
	//		Ogre::Vector3 nodePosF;
	//		KFbxPose* lPose = KFbxPose::Create(kSdkManager,kMesh->GetName());
	//		for (unsigned int i=0; i < kShape->nodes.size(); i++)
	//		{
	//			lKFbxNode   = gSkeletonLimbNodes[i];
	//			//nodePosF = getShapeInstance()->mNodeTransforms[i].getPosition();
	//			////Con::printf("nodeTrans pos %d  %f %f %f",i,nodePosF.x,nodePosF.y,nodePosF.z);
	//			//nodePos = KFbxVector4(-nodePosF.x,nodePosF.z,nodePosF.y);
	//			
	//			//lBindMatrix.SetIdentity();//lKFbxNode->GetGlobalFromDefault(KFbxNode::eSOURCE_SET);
	//			lBindMatrix = pScene->GetEvaluator()->GetNodeLocalTransform(lKFbxNode);
	//			lPose->Add(lKFbxNode, lBindMatrix,true);
	//		}

	//		// Add the pose to the scene
	//		pScene->AddPose(lPose);

	//		//Set Rest Pose?


	//	}
	//	mc++;
	//}

	///////////////////////////////////////////
	////HERE: find the name of the textures (all of them) and attach them to their respective objects.
	//KFbxTexture* lTexture = KFbxTexture::Create(pScene,"base.powderkeg.png");

	//// Set texture properties.
	//lTexture->SetFileName("base.powderkeg.png"); // Resource file is in current directory.
	//lTexture->SetTextureUse(KFbxTexture::eSTANDARD);
	//lTexture->SetMappingType(KFbxTexture::eUV);
	//lTexture->SetMaterialUse(KFbxTexture::eMODEL_MATERIAL);
	//lTexture->SetSwapUV(false);
	//lTexture->SetTranslation(0,0);//0.45, -0.05);
	//lTexture->SetScale(1.0, 1.0);
	//lTexture->SetRotation(0.0, 0.0);

	//kTextureDiffuseLayer->GetDirectArray().Add(lTexture);

	//KFbxSurfacePhong* lMaterial = NULL;
	//lMaterial = KFbxSurfacePhong::Create(pScene, "base.powderkeg.png");
	//if (lMaterial)
	//	lMaterial->GetDiffuseColor().ConnectSrcObject(lTexture);


	//lMaterialLayer->GetDirectArray().Add(lMaterial);

	////CreateTexture(pScene,kMesh);
	////MapTexture(pScene,kNode);
	//
	//fclose(debug);


	//return kNode;
	//return NULL;
//}
//
//void fxFlexBody::linkMeshToSkeleton(KFbxSdkManager* kSdkManager, KFbxNode* kMesh, KFbxNode* kRoot)
//{
//
//}

//bool fxFlexBody::createFbxScene(KFbxSdkManager* pSdkManager, KFbxScene* pScene, int seq)
//{
	// create scene info
	//KFbxDocumentInfo* sceneInfo = KFbxDocumentInfo::Create(pSdkManager,"SceneInfo");
	//sceneInfo->mTitle = "FBX Output";
	//sceneInfo->mSubject = "Ecstasy Motion";
	//sceneInfo->mAuthor = "BrokeAss Games";
	//sceneInfo->mRevision = "rev. 1.0";
	//sceneInfo->mKeywords = "";
	//sceneInfo->mComment = "";

	////// we need to add the sceneInfo before calling AddThumbNailToScene because
	////// that function is asking the scene for the sceneInfo.
 //  pScene->SetSceneInfo(sceneInfo);

	////AddThumbnailToScene(pSdkManager, pScene);

	//KFbxNode* lSkeletonRoot = createFbxSkeleton( pScene, "Skeleton");
	//pScene->GetRootNode()->AddChild(lSkeletonRoot);

	//KFbxNode* lMeshRoot = createFbxMesh(pSdkManager, pScene, lSkeletonRoot, "Mesh");
	//pScene->GetRootNode()->AddChild(lMeshRoot);

	////linkMeshToSkeleton(pSdkManager,lMeshRoot,lSkeletonRoot);

	////KFbxNode* lCubeRoot = CreateCubeWithTexture(pScene, "Cuuube");
	////pScene->GetRootNode()->AddChild(lCubeRoot);

	////KFbxNode* lTriangleRoot = CreateTriangle(pScene, "triiangle");
	////pScene->GetRootNode()->AddChild(lTriangleRoot);

	//////LinkPatchToSkeleton(pSdkManager, lPatch, lSkeletonRoot,kFB);
	//////StoreBindPose(pSdkManager, pScene, lPatch, lSkeletonRoot,kFB);
	//////StoreRestPose(pSdkManager, pScene, lSkeletonRoot,kFB);
	//animateFbxSkeleton(pSdkManager, pScene, lSkeletonRoot,seq);

	////// Build the node tree.

	//return true;
//}

//void fxFlexBody::exportFBX(int seq)
//{
	//KFbxSdkManager* lSdkManager = NULL;
	//KFbxScene* lScene = NULL;
	//bool lResult = false;

	//// Prepare the FBX SDK.
	//InitializeSdkObjects(lSdkManager, lScene);

	//// Create the scene.
	//lResult = createFbxScene(lSdkManager, lScene, seq);

	//if(lResult == false)
	//{
	//    printf("\n\nAn error occurred while creating the scene...\n");
	//    DestroySdkObjects(lSdkManager);
	//    return;
	//}

	////// Save the scene.

	////// The example can take an output file name as an argument.
	//////if(argc > 2)
	//////{
	//////    lResult = SaveScene(lSdkManager, lScene, argv[2]);
	//////}
	//////// A default output file name is given otherwise.
	//////else
	//////{

	//lResult = SaveScene(lSdkManager, lScene, "humanoid.fbx");

	//////}

	//if(lResult == false)
	//{
	//	//Con::printf("\n\nAn error occurred while saving the scene...\n");
	//	DestroySdkObjects(lSdkManager);
	//	return;
	//} else {
	//	//Con::printf("\n\nFbx scene created!\n");
	//}

	//// Destroy all objects created by the FBX SDK.
	//DestroySdkObjects(lSdkManager);


	//return;
//}


///////////////////////////////////////////////

//KFbxNode* CreateTriangle(KFbxScene* pScene, char* pName)
//{
    //KFbxMesh* lMesh = KFbxMesh::Create(pScene, pName);

    //// The three vertices
    //KFbxVector4 lControlPoint0(-50, 0, 50);
    //KFbxVector4 lControlPoint1(50, 0, 50);
    //KFbxVector4 lControlPoint2(0, 50, -50);

    //// Create control points.
    //lMesh->InitControlPoints(3);
    //KFbxVector4* lControlPoints = lMesh->GetControlPoints();

    //lControlPoints[0] = lControlPoint0;
    //lControlPoints[1] = lControlPoint1;
    //lControlPoints[2] = lControlPoint2;

    //// Create the triangle's polygon
    //lMesh->BeginPolygon();
    //lMesh->AddPolygon(0); // Control point 0
    //lMesh->AddPolygon(1); // Control point 1
    //lMesh->AddPolygon(2); // Control point 2
    //lMesh->EndPolygon();

    //KFbxNode* lNode = KFbxNode::Create(pScene,pName);
    //lNode->SetNodeAttribute(lMesh);

    //return lNode;
//}


//struct PlayerData;

//ImplementEnumType( physChainType,
//   "Bodypart chain types\n" )
//   //"@ingroup ChainEnums\n\n")
//	{ PHYS_CHAIN_SPINE,     "CHAIN_SPINE", "Spine, Neck, Head"  },
//   { PHYS_CHAIN_RIGHT_ARM,    "CHAIN_RIGHT_ARM", "Right Arm" },
//	{ PHYS_CHAIN_LEFT_ARM,      "CHAIN_LEFT_ARM",  "Left Arm" },
//	{ PHYS_CHAIN_RIGHT_LEG,      "CHAIN_RIGHT_LEG",  "Right Leg" },
//	{ PHYS_CHAIN_LEFT_LEG,      "CHAIN_LEFT_LEG",  "Left Leg" },
//	{ PHYS_CHAIN_RIGHT_WING,      "CHAIN_RIGHT_WING",  "Right Wing" },
//	{ PHYS_CHAIN_LEFT_WING,      "CHAIN_LEFT_WING",  "Left Wing" },
//	{ PHYS_CHAIN_TAIL,      "CHAIN_TAIL",  "Tail" },
//	{ PHYS_CHAIN_TONGUE,      "CHAIN_TONGUE",  "Tongue" },
//EndImplementEnumType;

//physGroundSequenceData::physGroundSequenceData()
//{
//   //mFlexBodyData  = NULL;
//   //FlexBodyDataID = 0;
//   mSeqNum = -1;
//   mSeqName = "";
//   mGroundNode1 = -1;
//   mTime1 = -1.0;
//   mGroundNode2 = -1;
//   mTime2 = -1.0;
//   mGroundNode3 = -1;
//   mTime3 = -1.0;
//   mGroundNode4 = -1;
//   mTime4 = -1.0;
//   mGroundNode5 = -1;
//   mTime5 = -1.0;
//   mGroundNode6 = -1;
//   mTime6 = -1.0;
//   mGroundNode7 = -1;
//   mTime7 = -1.0;
//   mGroundNode8 = -1;
//   mTime8 = -1.0;
//   mTimeScale = 1.0;
//   mDeltaVector.set(0,0.5,0);
//}
//
////physGroundSequenceData::~physGroundSequenceData()
////{
////}
//
//bool physGroundSequenceData::preload(bool bServer, String &errorStr)
//{
//  if (!Parent::preload(bServer, errorStr)) return false;
//
//  if (!bServer) {
//     if( !mFlexBodyData && FlexBodyDataID != 0 )
//     {
//        if( Sim::findObject( FlexBodyDataID, mFlexBodyData ) == false)
//        {
//           //Con::errorf( ConsoleLogEntry::General, "physGroundSequenceData::preload: Invalid packet, bad datablockId(mFlexBodyData): 0x%x", FlexBodyDataID );
//        }
//     }
//  }
//
//  return true;
//}
//
//bool physGroundSequenceData::onAdd()
//{
//   if(!Parent::onAdd()) return false;
//
//   //well, this is ugly but here goes:
//   if (mTime1>=0.0) {
//      mNodes[0]=mGroundNode1;
//      mTimes[0]=mTime1;
//      mNumSteps=1;
//   }
//   if (mTime2>=0.0) {
//      mNodes[1]=mGroundNode2;
//      mTimes[1]=mTime2;
//      mNumSteps=2;
//   }
//   if (mTime3>=0.0) {
//      mNodes[2]=mGroundNode3;
//      mTimes[2]=mTime3;
//      mNumSteps=3;
//   }
//   if (mTime4>=0.0) {
//      mNodes[3]=mGroundNode4;
//      mTimes[3]=mTime4;
//      mNumSteps=4;
//   }
//   if (mTime5>=0.0) {
//      mNodes[4]=mGroundNode5;
//      mTimes[4]=mTime5;
//      mNumSteps=5;
//   }
//   if (mTime6>=0.0) {
//      mNodes[5]=mGroundNode6;
//      mTimes[5]=mTime6;
//      mNumSteps=6;
//   }
//   if (mTime7>=0.0) {
//      mNodes[6]=mGroundNode7;
//      mTimes[6]=mTime7;
//      mNumSteps=7;
//   }
//   if (mTime8>=0.0) {
//      mNodes[7]=mGroundNode8;
//      mTimes[7]=mTime8;
//      mNumSteps=8;
//   }//... yuck.  Improvements welcome. :-)
//
//   //Con::errorf("physGroundSequenceData loaded: num steps %d sequence %s",mNumSteps,mSeqName);
//   return true;
//}
//
//
//void physGroundSequenceData::initPersistFields()
//{
//  Parent::initPersistFields();
//  addField("FlexBodyData", TYPEID< fxFlexBodyData >(), 
//        Offset(mFlexBodyData, physGroundSequenceData));
//  addField("SequenceNum", Typeint, 
//        Offset(mSeqNum, physGroundSequenceData));
//  addField("SequenceName", TypeString, 
//        Offset(mSeqName, physGroundSequenceData));
//  addField("GroundNode1", Typeint, 
//        Offset(mGroundNode1, physGroundSequenceData));
//  addField("Time1", Typefloat, 
//        Offset(mTime1, physGroundSequenceData));
//  addField("GroundNode2", Typeint, 
//        Offset(mGroundNode2, physGroundSequenceData));
//  addField("Time2", Typefloat, 
//        Offset(mTime2, physGroundSequenceData));
//  addField("GroundNode3", Typeint, 
//        Offset(mGroundNode3, physGroundSequenceData));
//  addField("Time3", Typefloat, 
//        Offset(mTime3, physGroundSequenceData));
//  addField("GroundNode4", Typeint, 
//        Offset(mGroundNode4, physGroundSequenceData));
//  addField("Time4", Typefloat, 
//        Offset(mTime4, physGroundSequenceData));
//  addField("GroundNode5", Typeint, 
//        Offset(mGroundNode5, physGroundSequenceData));
//  addField("Time5", Typefloat, 
//        Offset(mTime5, physGroundSequenceData));
//  addField("GroundNode6", Typeint, 
//        Offset(mGroundNode6, physGroundSequenceData));
//  addField("Time6", Typefloat, 
//        Offset(mTime6, physGroundSequenceData));
//  addField("GroundNode7", Typeint, 
//        Offset(mGroundNode7, physGroundSequenceData));
//  addField("Time7", Typefloat, 
//        Offset(mTime7, physGroundSequenceData));
//  addField("GroundNode8", Typeint, 
//        Offset(mGroundNode8, physGroundSequenceData));
//  addField("Time8", Typefloat, 
//        Offset(mTime8, physGroundSequenceData));
//  addField("TimeScale", Typefloat, 
//        Offset(mTimeScale, physGroundSequenceData));
//  addField("DeltaVector", TypeOgre::Vector3, 
//        Offset(mDeltaVector, physGroundSequenceData));
//
//}
//
//void physGroundSequenceData::packData(BitStream* pBitStream)
//{
//   Parent::packData(pBitStream);
//
//      if( pBitStream->writeFlag( mFlexBodyData != NULL ) )
//   {
//      pBitStream->writeRangedunsigned int(packed? SimObjectId(mFlexBodyData):
//            mFlexBodyData->getId(),DataBlockObjectIdFirst,DataBlockObjectIdLast);
//   }
//   pBitStream->write(mSeqNum);
//   pBitStream->writeString(mSeqName);
//   pBitStream->write(mGroundNode1);
//   pBitStream->write(mTime1);
//   pBitStream->write(mGroundNode2);
//   pBitStream->write(mTime2);
//   pBitStream->write(mGroundNode3);
//   pBitStream->write(mTime3);
//   pBitStream->write(mGroundNode4);
//   pBitStream->write(mTime4);
//   pBitStream->write(mGroundNode5);
//   pBitStream->write(mTime5);
//   pBitStream->write(mGroundNode6);
//   pBitStream->write(mTime6);
//   pBitStream->write(mGroundNode7);
//   pBitStream->write(mTime7);
//   pBitStream->write(mGroundNode8);
//   pBitStream->write(mTime8);
//   pBitStream->write(mTimeScale);
//   mathWrite(*pBitStream, mDeltaVector);
//}
//
//void physGroundSequenceData::unpackData(BitStream* pBitStream)
//{
//   Parent::unpackData(pBitStream); 
//   if( pBitStream->readFlag() )
//   {
//      FlexBodyDataID = pBitStream->readRangedunsigned int( DataBlockObjectIdFirst, DataBlockObjectIdLast );
//   }
//   pBitStream->read(&mSeqNum);
//   mSeqName = pBitStream->readSTString();
//   pBitStream->read(&mGroundNode1);
//   pBitStream->read(&mTime1);
//   pBitStream->read(&mGroundNode2);
//   pBitStream->read(&mTime2);
//   pBitStream->read(&mGroundNode3);
//   pBitStream->read(&mTime3);
//   pBitStream->read(&mGroundNode4);
//   pBitStream->read(&mTime4);
//   pBitStream->read(&mGroundNode5);
//   pBitStream->read(&mTime5);
//   pBitStream->read(&mGroundNode6);
//   pBitStream->read(&mTime6);
//   pBitStream->read(&mGroundNode7);
//   pBitStream->read(&mTime7);
//   pBitStream->read(&mGroundNode8);
//   pBitStream->read(&mTime8);
//   pBitStream->read(&mTimeScale);
//   mathRead(*pBitStream, &mDeltaVector);
//}
//
/////////////////////////////////////////////////////////
//
//IMPLEMENT_CO_DATABLOCK_V1(fxFlexBodyData);


//IMPLEMENT_CONSOLETYPE(fxFlexBodyData)
//IMPLEMENT_SETDATATYPE(fxFlexBodyData)
//IMPLEMENT_GETDATATYPE(fxFlexBodyData)

//
//fxFlexBodyData::fxFlexBodyData()
//{
//  mDBId = 0;
//  mActionUserData = NULL;
//  mActionUserDataID = 0;
//  mLifetimeMS = 0;
//  mNumBodyParts = 0;
//  mSDK = false;
//  mHW = false;
//  mGA = false;
//  mDynamicFriction = 0.0;
//  mStaticFriction = 0.0;
//  mRestitution = 0.0;
//  mDensity = 1.0;
//  mSleepThreshold = 0.0;
//  
//  mTriggerShapeType  = PHYS_SHAPE_CAPSULE;
//  mTriggerOffset = Ogre::Vector3::ZERO;
//  mTriggerOrientation = Ogre::Vector3::ZERO;
//  mTriggerDimensions = Ogre::Vector3::ZERO;
//
//  mMeshObject = NULL;
//  mHeadNodeName = NULL;
//  mNeckNodeName = NULL;
//  mBodyNodeName = NULL;
//  mRightFrontNodeName = NULL;
//  mLeftFrontNodeName = NULL;
//  mRightBackNodeName = NULL;
//  mLeftBackNodeName = NULL;
//  mMeshExcludes = NULL;
//  mSkeletonName = NULL;//Just can't get this to work as another datablock member exposed to script - crashes on Boombot.cs,  
//  //or any other player datablock (not flexbody datablock.)
//
//  mRelaxType = 0;
//
//  shadowEnable = true;
//  //shadowCanMove = true;
//  //shadowCanAnimate = true;
//  //shadowSelfShadow = false;
//  shadowSize = 128;
//  //shadowAnimationFrameSkip = 5;
//  shadowMaxVisibleDistance = 80.0f;
//  shadowProjectionDistance = 14.0f;
//  shadowSphereAdjust = 1.0;
//  //shadowBias = 0.0005f;
//  //shadowDTSShadow = false;
//  //shadowIntensity = 1.0f;
//
//}
//
//fxFlexBodyData::~fxFlexBodyData()
//{
//}
//
//bool fxFlexBodyData::preload(bool bServer, String &errorStr)
//{
//  if (!Parent::preload(bServer, errorStr)) return false;
//
//  
//   if (!bServer) 
//   {
//      if( !mActionUserData && mActionUserDataID != 0 )
//      {
//         if( Sim::findObject( mActionUserDataID, mActionUserData ) == false)
//         {
//            //Con::errorf( ConsoleLogEntry::General, "fxFlexBodyPartData::preload: Invalid packet, bad datablockId(mActionUserData): 0x%x", mActionUserDataID );
//         }
//      }
//   }
//	  
//   return true;
//}
//
//bool fxFlexBodyData::onAdd()
//{
//   if(!Parent::onAdd()) return false;
//   else if (!strcmp(category,"Actors"))
//   {
//	   SQLiteObject *sql = new SQLiteObject();
//	   if (!sql) return true;//Important:  this is one of the times that we have to 
//	   //create a new sqlite object, because physManager hasn't started the main one up yet.
//	   //Don't forget to DELETE IT. :-)
//	   if (sql->OpenDatabase("EcstasyMotion.db"))
//	   {
//		   char id_query[512],insert_query[512];
//		   int flexbodydata_id,skeleton_id,result;
//		   sqlite_resultset *resultSet;
//
//		   result = sql->ExecuteSQL("BEGIN TRANSACTION;");
//
//		   //SEARCH FIRST - don't insert every time!
//		   sprintf(id_query,"SELECT id FROM skeleton WHERE name = '%s';",mSkeletonName);
//		   result = sql->ExecuteSQL(id_query);
//		   resultSet = sql->GetResultSet(result);
//		   if (resultSet->iNumRows == 0)
//		   {
//			   sprintf(insert_query,"INSERT INTO skeleton (name) VALUES ('%s');",mSkeletonName);
//			   result = sql->ExecuteSQL(insert_query);
//			   result = sql->ExecuteSQL(id_query);
//			   resultSet = sql->GetResultSet(result);
//		   }
//		   if (resultSet->iNumRows == 1)
//			   skeleton_id = strtol(resultSet->vRows[0]->vColumnValues[0]);
//
//		   sprintf(id_query,"SELECT id FROM fxFlexBodyData WHERE name = '%s';",getName());
//		   result = sql->ExecuteSQL(id_query);
//		   resultSet = sql->GetResultSet(result);
//		   if (resultSet->iNumRows == 1)
//			   flexbodydata_id = strtol(resultSet->vRows[0]->vColumnValues[0]);
//		   else if (resultSet->iNumRows == 0)
//		   {//also: action_user_id
//			   sprintf(insert_query,"INSERT INTO fxFlexBodyData (skeleton_id,name,shapeFile,category,SleepThreshold,HeadNode,NeckNode,BodyNode,RightFrontNode,LeftFrontNode,RightBackNode,LeftBackNode) VALUES (%d,'%s','%s','%s',%f,'%s','%s','%s','%s','%s','%s','%s');",\
//				   skeleton_id,getName(),shapeName,category,\
//				   mSleepThreshold,mHeadNodeName,mNeckNodeName,\
//				   mBodyNodeName,mRightFrontNodeName,mLeftFrontNodeName,\
//				   mRightBackNodeName,mLeftBackNodeName);
//			   result = sql->ExecuteSQL(insert_query);
//			   result = sql->ExecuteSQL(id_query);
//			   resultSet = sql->GetResultSet(result);
//			   if (resultSet->iNumRows == 1)
//				   flexbodydata_id = strtol(resultSet->vRows[0]->vColumnValues[0]);
//		   }
//			if (flexbodydata_id>0) mDBId = flexbodydata_id;
//			else mDBId = 0;
//		   
//		   result = sql->ExecuteSQL("END TRANSACTION;");
//		   sql->CloseDatabase();
//	   }
//	   delete sql;
//	   return true;
//   }
//}
//
//void fxFlexBodyData::initPersistFields()
//{
//  Parent::initPersistFields();
//
//  addField("ActionUserData", TYPEID< SimObject >(), 
//        Offset(mActionUserData, fxFlexBodyData));	
//  addField("Lifetime", Typeint, 
//        Offset(mLifetimeMS, fxFlexBodyData));	
//  addField("SDK", TypeBool, 
//        Offset(mSDK, fxFlexBodyData));
//  addField("HW", TypeBool, 
//        Offset(mHW, fxFlexBodyData));
//  addField("GA", TypeBool, 
//        Offset(mGA, fxFlexBodyData));
//  addField("DynamicFriction", Typefloat, 
//        Offset(mDynamicFriction, fxFlexBodyData));
//  addField("StaticFriction", Typefloat, 
//        Offset(mStaticFriction, fxFlexBodyData));
//  addField("Restitution", Typefloat, 
//        Offset(mRestitution, fxFlexBodyData));
//  addField("myDensity", Typefloat, 
//        Offset(mDensity, fxFlexBodyData));
//  addField("SleepThreshold", Typefloat, 
//        Offset(mSleepThreshold, fxFlexBodyData));
//  addField("MeshObject",TypeString,
//        Offset(mMeshObject, fxFlexBodyData));
//
//  addField( "upAxis", TYPEID< domUpAxisType >(), Offset(mOptions.upAxis, TSShapeConstructor),
//      "Override the <up_axis> element in the COLLADA (.dae) file. No effect for DTS files.\n"
//      "Set to one of the following values:\n"
//      "<dl><dt>X_AXIS</dt><dd>Positive X points up. Model will be rotated into Torque's coordinate system (Z up).</dd>"
//      "<dt>Y_AXIS</dt><dd>Positive Y points up. Model will be rotated into Torque's coordinate system (Z up).</dd>"
//      "<dt>Z_AXIS</dt><dd>Positive Z points up. No rotation will be applied to the model.</dd>"
//      "<dt>DEFAULT</dt><dd>The default value. Use the value in the .dae file (defaults to Z_AXIS if the <up_axis> element is not present).</dd></dl>" );
//
//  addField("TriggerShapeType", TYPEID< physShapeType >(),
//		Offset(mTriggerShapeType, fxFlexBodyData) );
//  addField("TriggerOffset", TypeOgre::Vector3,
//		Offset(mTriggerOffset, fxFlexBodyData));
//  addField("TriggerOrientation", TypeOgre::Vector3,
//		Offset(mTriggerOrientation, fxFlexBodyData));
//  addField("TriggerDimensions", TypeOgre::Vector3,
//		Offset(mTriggerDimensions, fxFlexBodyData));
//  addField("HeadNode",TypeString,
//        Offset(mHeadNodeName, fxFlexBodyData));
//  addField("NeckNode",TypeString,
//        Offset(mNeckNodeName, fxFlexBodyData));
//  addField("BodyNode",TypeString,
//        Offset(mBodyNodeName, fxFlexBodyData));
//  addField("RightFrontNode",TypeString,
//        Offset(mRightFrontNodeName, fxFlexBodyData));
//  addField("LeftFrontNode",TypeString,
//        Offset(mLeftFrontNodeName, fxFlexBodyData));
//  addField("RightBackNode",TypeString,
//        Offset(mRightBackNodeName, fxFlexBodyData));
//  addField("LeftBackNode",TypeString,
//        Offset(mLeftBackNodeName, fxFlexBodyData));  
//  addField("MeshExcludes",TypeString,
//        Offset(mMeshExcludes, fxFlexBodyData));  
//  addField("RelaxType", Typeint, 
//        Offset(mRelaxType, fxFlexBodyData));
//  addField("SkeletonName",TypeString,
//        Offset(mSkeletonName, fxFlexBodyData));
//}
//
//void fxFlexBodyData::packData(BitStream* pBitStream)
//{
//   Parent::packData(pBitStream);
//
//   if( pBitStream->writeFlag( mActionUserData != NULL ) )
//   {
//      pBitStream->writeRangedunsigned int(packed? SimObjectId(mActionUserData):
//                             mActionUserData->getId(),DataBlockObjectIdFirst,DataBlockObjectIdLast);
//   }
//   pBitStream->write(mLifetimeMS);
//   pBitStream->write(mSDK);
//   pBitStream->write(mHW);
//   pBitStream->write(mGA);
//   pBitStream->write(mDynamicFriction);
//   pBitStream->write(mStaticFriction);
//   pBitStream->write(mRestitution);
//   pBitStream->write(mDensity);
//   pBitStream->write(mSleepThreshold);
//   pBitStream->writeString(mMeshObject);
//   pBitStream->writeRangedunsigned int(mTriggerShapeType, 0, NUM_SHAPE_TYPES-1);
//   mathWrite(*pBitStream, mTriggerOffset);
//   mathWrite(*pBitStream, mTriggerOrientation);
//   mathWrite(*pBitStream, mTriggerDimensions);
//   pBitStream->writeString(mHeadNodeName);
//   pBitStream->writeString(mNeckNodeName);
//   pBitStream->writeString(mBodyNodeName);
//   pBitStream->writeString(mRightFrontNodeName);
//   pBitStream->writeString(mLeftFrontNodeName);
//   pBitStream->writeString(mRightBackNodeName);
//   pBitStream->writeString(mLeftBackNodeName);
//   pBitStream->writeString(mMeshExcludes);
//   pBitStream->write(mNumBodyParts);
//   pBitStream->write(mRelaxType);
//   pBitStream->writeString(mSkeletonName);
//}
//
//
//void fxFlexBodyData::unpackData(BitStream* pBitStream)
//{
//   Parent::unpackData(pBitStream); 
//
//   if( pBitStream->readFlag() )
//   {
//      mActionUserDataID = pBitStream->readRangedunsigned int( DataBlockObjectIdFirst, DataBlockObjectIdLast );
//   }
//   pBitStream->read(&mLifetimeMS);
//   pBitStream->read(&mSDK);  
//   pBitStream->read(&mHW);  
//   pBitStream->read(&mGA);
//   pBitStream->read(&mDynamicFriction);
//   pBitStream->read(&mStaticFriction);
//   pBitStream->read(&mRestitution);
//   pBitStream->read(&mDensity);
//   pBitStream->read(&mSleepThreshold);
//   mMeshObject = pBitStream->readSTString();
//   mTriggerShapeType = (physShapeType)pBitStream->readRangedunsigned int(0, NUM_SHAPE_TYPES-1);
//   mathRead(*pBitStream, &mTriggerOffset);
//   mathRead(*pBitStream, &mTriggerOrientation);
//   mathRead(*pBitStream, &mTriggerDimensions);
//   mHeadNodeName = pBitStream->readSTString();
//   mNeckNodeName = pBitStream->readSTString();
//   mBodyNodeName = pBitStream->readSTString();
//   mRightFrontNodeName = pBitStream->readSTString();
//   mLeftFrontNodeName = pBitStream->readSTString();
//   mRightBackNodeName = pBitStream->readSTString();
//   mLeftBackNodeName = pBitStream->readSTString();
//   mMeshExcludes = pBitStream->readSTString();
//   pBitStream->read(&mNumBodyParts);
//   pBitStream->read(&mRelaxType);
//   mSkeletonName = pBitStream->readSTString();
//}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
/*
ConsoleMethod( fxFlexBody, startAnimatingAtPos, void,  3, 4,"(char sequence,float pos)")
{

	int seq = object->getShape()->findSequence(argv[2]);
	float pos = 0.0;
	if (argc==4) pos = strtod(argv[3]);

	if ((seq>-1)&&(pos>=0.0)&&(pos<=1.0))
		object->startAnimatingAtPos(seq,pos);

}

ConsoleMethod( fxFlexBody, play, void,  3, 3,"(char seqname)")
{
	int seq = object->getShape()->findSequence(argv[2]);
	if (seq<0) 
	{
		//Con::errorf("can't find sequence %s",argv[2]);
		return;
	}
	////Con::errorf("playing sequence: %d  %s",seq,argv[2]);
	//object->setThreadSequence(0,seq);
	object->playThread(0,argv[2]);
	return;
}

ConsoleMethod( fxFlexBody, playThread, bool, 3, 4, "(int slot, string sequenceName)")
{
   unsigned int slot = strtol(argv[2]);
   if (object->isLockedError("playThread",slot))
      return false;
   if (slot >= 0 && slot < fxFlexBody::MaxScriptThreads) {
      if (argc == 4) {
         if (object->playThread(slot,argv[3]))
            return true;
      }
      //else
      //   if (object->playThread(slot))
      //      return true;
   }
   return false;
}

ConsoleMethod( fxFlexBody, zeroForces, void, 2, 2,"")
{	
	object->zeroForces();
	return;
}

ConsoleMethod( fxFlexBody, setBodypartForce, void, 4, 4,"(index i, Ogre::Vector3 force)")
{
	int i;
	Ogre::Vector3 force;	
	sscanf(argv[2], "%d", &i);
	sscanf(argv[3], "%g %g %g", &force.x, &force.y, &force.z);
	object->setBodypartForce(i,force);
	return;
}

ConsoleMethod( fxFlexBody, setBodypartGlobalForce, void, 4, 4,"setBodypartGlobalForce(index i, Ogre::Vector3 force)")
{
	int i;
	Ogre::Vector3 force;	
	sscanf(argv[2], "%d", &i);
	sscanf(argv[3], "%g %g %g", &force.x, &force.y, &force.z);
	object->setBodypartGlobalForce(i,force);
	return;
}

ConsoleMethod( fxFlexBody, setBodypartTorque, void, 4, 4,"(index i, Ogre::Vector3 torque)")
{
	int i;
	Ogre::Vector3 torque;	
	sscanf(argv[2], "%d", &i);
	sscanf(argv[3], "%g %g %g", &torque.x, &torque.y, &torque.z);
	object->setBodypartTorque(i,torque);
	return;
}
ConsoleMethod( fxFlexBody, setBodypartGlobalTorque, void, 4, 4,"(index i, Ogre::Vector3 torque)")
{
	int i;
	Ogre::Vector3 torque;	
	sscanf(argv[2], "%d", &i);
	sscanf(argv[3], "%g %g %g", &torque.x, &torque.y, &torque.z);
	object->setBodypartGlobalTorque(i,torque);
	return;
}
ConsoleMethod( fxFlexBody, setBodypartImpulseForce, void, 4, 4,"(index i, Ogre::Vector3 force)")
{
	int i;
	Ogre::Vector3 force;	
	sscanf(argv[2], "%d", &i);
	sscanf(argv[3], "%g %g %g", &force.x, &force.y, &force.z);
	object->setBodypartDelayForce(i,force);
	return;
}

ConsoleMethod( fxFlexBody, setBodypartImpulseTorque, void, 4, 4,"(index i, Ogre::Vector3 torque)")
{
	int i;
	Ogre::Vector3 torque;	
	sscanf(argv[2], "%d", &i);
	sscanf(argv[3], "%g %g %g", &torque.x, &torque.y, &torque.z);
	object->setBodypartDelayTorque(i,torque);
	return;
}

ConsoleMethod( fxFlexBody, setBodypartMotorTarget, void, 3, 4,"(index i, Ogre::Vector3 target)")
{
	int i;
	Ogre::Vector3 target;	
	sscanf(argv[2], "%d", &i);
	if (argc==4)
		sscanf(argv[3], "%g %g %g", &target.x, &target.y, &target.z);
	else
		target = Ogre::Vector3::ZERO;
	object->setBodypartMotorTarget(i,target);
	return;
}

ConsoleMethod( fxFlexBody, setBodypartMotorSpring, void, 3, 5,"(index i, Ogre::Vector3 target,float force)")
{
	int i;
	float force;
	Ogre::Vector3 target;	
	sscanf(argv[2], "%d", &i);
	if (argc>3)
		sscanf(argv[3], "%g %g %g", &target.x, &target.y, &target.z);
	else target = Ogre::Vector3::ZERO;

	if (argc>4)
		sscanf(argv[3], "%g", &force);
	else
		force = 1.0;

	//Con::errorf("setting bodypart motor spring, consolemethod %f %f %f!",target.x,target.y,target.z);
	object->setBodypartMotorSpring(i,target,force);
	return;
}

ConsoleMethod( fxFlexBody, headUp, void, 2, 2,"")
{	
	object->headUp();
	return;
}

ConsoleMethod( fxFlexBody, headClear, void, 2, 2,"")
{	
	object->headClear();
	return;
}

ConsoleMethod( fxFlexBody, headCheck, int, 2, 2,"")
{	
	return object->headCheck();
}

ConsoleMethod( fxFlexBody, splay, void, 2, 2,"")
{	
	object->splay();
	return;
}

ConsoleMethod( fxFlexBody, setKinematic, void, 2, 2,"")
{	
	object->setKinematic();
	return;
}

ConsoleMethod( fxFlexBody, clearKinematic, void, 2, 2,"")
{	
	object->clearKinematic();
	return;
}

ConsoleMethod( fxFlexBody, getKinematic, bool, 2, 2,"")
{	
	return object->mIsKinematic;
}

ConsoleMethod( fxFlexBody, setBodypart, void, 3, 3,"setBodypart(unsigned int i)")
{	
	object->setBodypart(strtol(argv[2]));
	return;
}

ConsoleMethod( fxFlexBody, clearBodypart, void, 3, 3,"clearBodypart(unsigned int i)")
{	
	object->clearBodypart(strtol(argv[2]));
	return;
}

ConsoleMethod( fxFlexBody, setNoGravity, void, 2, 2,"")
{	
	object->setNoGravity();
	return;
}

ConsoleMethod( fxFlexBody, clearNoGravity, void, 2, 2,"")
{	
	object->clearNoGravity();
	return;
}

ConsoleMethod( fxFlexBody, startThinking, void, 2, 2,"")
{	
	object->mIsThinking = true;
	//Con::executef(object,"onStartThinking", //Con::getIntArg(object->getId()));
	return;
}

ConsoleMethod( fxFlexBody, stopThinking, void, 2, 2,"")
{	
	object->zeroForces();
	object->mIsThinking = false;
	//Con::executef(object,"onStopThinking", //Con::getIntArg(object->getId()));
	return;
}

ConsoleMethod( fxFlexBody, startAnimating, void, 3, 3,"(int sequence)")
{	
   int seq;
   sscanf(argv[2],"%d",&seq);
   object->startAnimating(seq);
   //Con::executef(object,"onStartAnimating", //Con::getIntArg(object->getId()));
   return;
}

ConsoleMethod( fxFlexBody, stopAnimating, void, 2, 2,"")
{
   object->stopAnimating();
   //Con::executef(object,"onStopAnimating", //Con::getIntArg(object->getId()));
   return;
}


ConsoleMethod(fxFlexBody,setLifetime,void,3,3,"Sets lifetime in MS")
{
	unsigned int lifetime;
	sscanf(argv[2],"%d",&lifetime);
	object->mLifetimeMS = lifetime;
	object->mCurrMS = 0;
	return;//hmm, setting properties on the datablock is the wrong way to go.
}

ConsoleMethod(fxFlexBody,onCollision,void,2,2,"calls onCollision")
{
	object->onCollision(NULL);
	return;
}

ConsoleMethod(fxFlexBody,motorize,void,2,2,"activates all joint motors")
{
	object->motorize();
	return;
}

ConsoleMethod(fxFlexBody,setBodypartForces,void,5,5,"applies explosion forces to bodyparts.")
{
	Ogre::Vector3 kPosition(0,0,0);
	Ogre::Vector3 kImpulse(0,0,0);
	Ogre::Vector3 kImpulseVec(0,0,0);
	float kScale;

	sscanf(argv[2],"%f %f %f",&kPosition.x,&kPosition.y,&kPosition.z);
	sscanf(argv[3],"%f %f %f",&kImpulse.x,&kImpulse.y,&kImpulse.z);
	sscanf(argv[4],"%f",&kScale);


	for (unsigned int i=0; i < object->mNumBodyParts; i++) {
		kImpulseVec = object->mBodyParts[i]->mCurrPosition - kPosition;
		kImpulseVec.normalize();
		kImpulseVec *= kScale;

		object->mBodyParts[i]->setGlobalForce(kImpulseVec);
		////Con::errorf("applying force! %f %f %f",kImpulse.x,kImpulse.y,kImpulse.z);
	}
	return;
}

ConsoleMethod(fxFlexBody,getBodyPart,int,3,3,"getbodypart(const char *) - returns flexbodypart ID for bodypart")
{
	fxFlexBodyPart *myPart = object->getBodyPart(argv[2]);
	////Con::errorf("trying to find bodypart %s! isserver = %d",argv[2],object->isServerObject());

	if (myPart) {
		////Con::errorf("part exists! bone index: %d",myPart->mBoneIndex);
        //return myPart->mNodeIndex;
		return myPart->mBoneIndex;
	}
	return -1;
}

ConsoleMethod(fxFlexBody,getNumBodyparts,int,2,2,"returns bodypart count")
{
	return object->mNumBodyParts;
}

ConsoleMethod(fxFlexBody,addWeapon,void,4,4,"addWeapon(fxRigidBody weaponID,int mount_slot")//,char *bodypart_node)
{
	if (dIsAllNumeric(argv[3])) //did I expose that as console function?
		object->addWeapon(strtol(argv[2]),strtol(argv[3]));
	else
		object->addWeapon(strtol(argv[2]),argv[3]);
}

ConsoleMethod(fxFlexBody,addWeapon2,void,4,4,"addWeapon2(fxRigidBody weaponID,int mount_slot")//,char *bodypart_node)
{
	if (dIsAllNumeric(argv[3])) //did I expose that as console function?
		object->addWeapon2(strtol(argv[2]),strtol(argv[3]));
	else
		object->addWeapon2(strtol(argv[2]),argv[3]);
}

ConsoleMethod(fxFlexBody,setIsStriking,void,3,3,"(bool isStriking)")
{
	object->mIsStriking = strtol(argv[2]);
}

ConsoleMethod(fxFlexBody,removeBodyparts,void,2,2,"removeBodyparts()")
{
	object->clientRemoveBodyparts();
	//Con::errorf("trying to delete bodyparts");
}

ConsoleMethod(fxFlexBody,setPhysActive,void,3,3,"setPhysActive(bool)")
{
	object->setPhysActive(strtol(argv[2]));
}

ConsoleMethod(fxFlexBody,getPhysActive,int,2,2,"getPhysActive()")
{
	return(object->mIsPhysActive);
}

ConsoleMethod(fxFlexBody,getSubType,int,2,2,"int getSubType()")
{
	return object->mEntitySubType;
}

ConsoleMethod(fxFlexBody,setBodypartDelayForces,void,3,3,"setBodyPartDelayForces(Ogre::Vector3 force)")
{
	Ogre::Vector3 kForce(0,0,0);
	sscanf(argv[2],"%f %f %f",&kForce.x,&kForce.y,&kForce.z);
	object->setBodypartDelayForces(kForce);
}

ConsoleMethod(fxFlexBody,resetPosition,void,2,2,"reset position")
{
   object->resetPosition();
}


ConsoleMethod(fxFlexBody, getFlexBodyName, const char*, 2, 2, "")
{
   return object->getFlexBodyName();
}


//ConsoleMethod( fxFlexBody, addWeapon, void, 4, 4,"bodypart, weapon")
//{
//	int bodypart,weapon;
//	sscanf(argv[2],"%d",&bodypart);
//	sscanf(argv[3],"%d",&weapon);
//	object->addWeapon(bodypart,weapon);
//	return;
//}

ConsoleMethod( fxFlexBody, saveBest, void, 3, 3,"(char filename)")
{
	object->mActionUser->saveAction(argv[2]);
	return;
}

ConsoleMethod( fxFlexBody, loadBest, void,  3, 3,"(char filename)")
{
	object->mActionUser->loadAction(argv[2]);
	return;
}

ConsoleMethod( fxFlexBody, saveAll, void, 2, 2,"")
{
	object->mActionUser->saveAll();
	return;
}

ConsoleMethod( fxFlexBody, getActionUser, int, 2, 2,"")
{
	return object->mActionUser->getId();
}

ConsoleMethod( fxFlexBody, act, void, 3, 3,"")
{

	object->mActionUser->loadAction(argv[2]);
	//if (!strncmp(argv[2],"dynam",5))
	//	object->mActionUser->setGoalDynamic();
	//else
	//	object->mActionUser->setGoalSingleAction();
	
}

//ConsoleMethod( fxFlexBody, snakeForward, void, 3, 3,"")
//{	
//	float f;
//	sscanf(argv[2], "%g",&f); 
//	object->snakeForward(f);
//	return;
//}

//ConsoleMethod( fxFlexBody, snakeStop, void, 2, 2,"")
//{	
//	object->snakeStop();
//	return;
//}

ConsoleMethod( fxFlexBody, printPos, void, 2, 2,"")
{
	for (unsigned int i=0;i<object->mNumBodyParts;i++)
	{
		Ogre::Vector3 kPos = object->getShapeInstance()->mNodeTransforms[object->mBodyParts[i]->mNodeIndex].getPosition();
		Ogre::Quaternion kNodeTrans(object->getShapeInstance()->mNodeTransforms[object->mBodyParts[i]->mNodeIndex]);

		Ogre::Vector3 kObj = object->mBodyParts[i]->getTransform().getPosition();
		Ogre::Matrix3 obj2w = object->mBodyParts[i]->getTransform();
		Ogre::Quaternion ang = object->mBodyParts[i]->mRB->getAngularPosition();
		Ogre::Quaternion sdkInv = object->mBodyParts[i]->mSDKInverse;

		Ogre::Matrix3 kMat;
		object->mBodyParts[i]->mSDKRot.setMatrix(&kMat);
		kMat.inverse();
		Ogre::Quaternion kMatInv(kMat);

		//Con::errorf("mNodeTransforms %d pos = %3.2f %3.2f %3.2f, trans =  %3.2f %3.2f %3.2f %3.2f",
			object->mBodyParts[i]->mNodeIndex,
			kPos.x,kPos.y,kPos.z,
			kNodeTrans.x,kNodeTrans.y,kNodeTrans.z,kNodeTrans.w);
			//kMatInv.x,kMatInv.y,kMatInv.z,kMatInv.w);
			//sdkInv.x,sdkInv.y,sdkInv.z,sdkInv.w);

		//if ((i>3) && (i<8) && obj2w)
		if(0)
		{
			Ogre::Vector3 one, two, three, four, pos;
			obj2w.getRow(0, &one);
			obj2w.getRow(1, &two);
			obj2w.getRow(2, &three);
			obj2w.getRow(3, &four);
			obj2w.getColumn(3, &pos);
			
			//Con::printf("obj2w pos: %3.2f %3.2f %3.2f, shear: %3.2f %3.2f %3.2f" ,pos.x,pos.y,pos.z,four.x,four.y,four.z);
		}
		//if ((i==10) && obj2w) obj2w.
	}
	Ogre::Quaternion kRot = object->mBodyParts[0]->mRB->getAngularPosition();
	Ogre::Vector3 test1,test2;
	test1.set(0,0,1);
	kRot.mulP(test1,&test2);
	//Con::errorf("kRot %3.2f %3.2f %3.2f %3.2f, (0,0,1) turns to (%3.2f,%3.2f,%3.2f)",
		kRot.x,kRot.y,kRot.z,kRot.w,test2.x,test2.y,test2.z);
	kRot.inverse();
	kRot.mulP(test1,&test2);
	//Con::errorf("kRot Inverse %3.2f %3.2f %3.2f %3.2f, (0,0,1) turns to (%3.2f,%3.2f,%3.2f)",
		kRot.x,kRot.y,kRot.z,kRot.w,test2.x,test2.y,test2.z);
}


ConsoleMethod(fxFlexBody, getBodypartPos, const char *,2,3,"getBodypartPos(int partID)")
{
	char* buff = //Con::getReturnBuffer(100);
	Ogre::Vector3 pos;

	if ((argc==3)&&(strlen(buff)>0))
		pos = object->mBodyParts[strtol(argv[2])]->getTransform().getPosition();
	else
		pos = object->mBodyParts[0]->getTransform().getPosition();

	sprintf(buff,100,"%g %g %g",pos.x,pos.y,pos.z);
	return buff;
}

ConsoleMethod(fxFlexBody, getBodypartKinematic, bool,3,3,"getBodypartKinematic(int partID)")
{
	return object->mBodyParts[strtol(argv[2])]->mIsKinematic;
}

ConsoleMethod(fxFlexBody, getBodypartsKinematic, void,2,2,"getBodypartsKinematic()")
{
	//Con::printf("getBodypartsKinematic:"); 
	//Con::printf("Main body:  %d",object->mIsKinematic);
	for (unsigned int i=0;i<object->mNumBodyParts;i++)
	{
		//Con::printf("part %d:  %d",i,object->mBodyParts[i]->mIsKinematic);
	}
}

ConsoleMethod(fxFlexBody, getBodypartName, const char *,2,3,"getBodypartName(int partID)")
{
	char* buff = //Con::getReturnBuffer(100);

	int bodypartID; 
	if (argc==3)
		bodypartID = strtol(argv[2]);
	else 
		bodypartID = 0;
	
	TSShape *kShape = object->getShapeInstance()->getShape();
	const char *name = kShape->names[kShape->nodes[object->mBodyParts[bodypartID]->mNodeIndex].nameIndex];
	sprintf(buff,255,"%s",name);
	return buff;
}

ConsoleMethod(fxFlexBody, getBodypartParent,int,3,3,"getBodypartParent(int partID)")
{
	int bodypartID = strtol(argv[2]);
	int parentID = -1;
	for (unsigned int i=0;i<object->mNumBodyParts;i++)
	{
		if (object->mBodyParts[i]->mNodeIndex==object->mBodyParts[bodypartID]->mParentIndex)
		{
			parentID = i;
			break;
		}
	}
	//object->mBodyParts[bodypartID]->mParentIndex
	return parentID;
}

ConsoleMethod( fxFlexBody, giveDetails, void,  2, 2,"(char filename)")
{
	object->giveDetails();
	return;
}

ConsoleMethod(fxFlexBody,setIsRecording,void,3,3,"setIsRecording(bool)")
{
	object->setIsRecording(strtol(argv[2]));
}

ConsoleMethod(fxFlexBody,toggleIsRecording,void,2,2,"toggleIsRecording(bool)")
{
	if (object->mIsRecording)
		object->setIsRecording(0);
	else
		object->setIsRecording(1);
}

ConsoleMethod(fxFlexBody,setIsRendering,void,3,3,"setIsRendering(bool)")
{
	object->setIsRendering(strtol(argv[2]));
}

ConsoleMethod(fxFlexBody,setIsReturnToZero,void,3,3,"setIsReturnToZero(bool)")
{
	object->setIsReturnToZero(strtol(argv[2]));
}

ConsoleMethod(fxFlexBody,setInitialPosition,void,3,3,"setInitialPosition(pos)")
{
	Ogre::Vector3 pos;
	sscanf(argv[2],"%g %g %g",&pos.x,&pos.y,&pos.z);
	object->setInitialPosition(pos);
}

ConsoleMethod(fxFlexBody,setInitialOrientation,void,3,3,"setInitialOrientation(rot)")
{
	Ogre::Vector3 rot;
	sscanf(argv[2],"%g %g %g",&rot.x,&rot.y,&rot.z);
	object->setInitialOrientation(rot);
}

ConsoleMethod(fxFlexBody,makeSequence,void,3,3,"makeSequence(char seqName)")
{
	if (strlen(argv[2])) object->makeSequence(argv[2]);
	else object->makeSequence("sequence");
}

ConsoleMethod(fxFlexBody,showNodeTransform,void,3,3,"showNodeTransform(int nodeIndex)")
{
	object->showNodeTransform(strtol(argv[2]));
}

ConsoleMethod(fxFlexBody,showNodeTransforms,void,2,2,"showNodeTransforms()")
{
	for (unsigned int i=0;i<object->mNumBodyParts;i++)
	{
		object->showNodeTransform(object->mBodyParts[i]->mNodeIndex);
	}
}

ConsoleMethod( fxFlexBody, setSleepThreshold, void, 3, 3, "setSleepThreshold(float)")
{
	object->mSleepThreshold = strtod(argv[2]);
}

ConsoleMethod( fxFlexBody, getSleepThreshold, float, 2, 2, "getSleepThreshold()")
{
	return object->mSleepThreshold;
}

ConsoleMethod( fxFlexBody, setWeaponMotorTarget, void, 3, 3, "setWeaponMotorTarget(Ogre::Vector3 rot)")
{
	Ogre::Vector3 rot(0,0,0);
	sscanf(argv[2],"%f %f %f",&rot.x,&rot.y,&rot.z);
	object->setWeaponMotorTarget(rot);
	return; 
}

ConsoleMethod( fxFlexBody, setWeapon2MotorTarget, void, 3, 3, "setWeapon2MotorTarget(Ogre::Vector3 rot)")
{
	Ogre::Vector3 rot(0,0,0);
	sscanf(argv[2],"%f %f %f",&rot.x,&rot.y,&rot.z);
	object->setWeapon2MotorTarget(rot);
	return; 
}

ConsoleMethod( fxFlexBody, setWeaponTriggerMotorTarget, void, 3, 3, "setWeaponTriggerMotorTarget(Ogre::Vector3 rot)")
{
	Ogre::Vector3 rot(0,0,0);
	sscanf(argv[2],"%f %f %f",&rot.x,&rot.y,&rot.z);
	object->setWeaponTriggerMotorTarget(rot);
	return; 
}

ConsoleMethod( fxFlexBody, setWeapon2TriggerMotorTarget, void, 3, 3, "setWeapon2TriggerMotorTarget(Ogre::Vector3 rot)")
{
	Ogre::Vector3 rot(0,0,0);
	sscanf(argv[2],"%f %f %f",&rot.x,&rot.y,&rot.z);
	object->setWeapon2TriggerMotorTarget(rot);
	return; 
}


ConsoleMethod(fxFlexBody,setWeaponTriggerRotAdjA,void,3,3,"setWeaponTriggerRotAdjA(Ogre::Vector3)")
{
	Ogre::Vector3 kRot( 0.0f,0.0f,0.0f );
	sscanf( argv[2], "%f %f %f", &kRot.x, &kRot.y, &kRot.z );
	object->setWeaponTriggerRotAdjA(kRot);
	return;
}

ConsoleMethod(fxFlexBody,setWeaponTriggerRotAdjB,void,3,3,"setWeaponTriggerRotAdjB(Ogre::Vector3)")
{
	Ogre::Vector3 kRot( 0.0f,0.0f,0.0f );
	sscanf( argv[2], "%f %f %f", &kRot.x, &kRot.y, &kRot.z );
	object->setWeaponTriggerRotAdjB(kRot);
	return;
}

ConsoleMethod(fxFlexBody,getCurrSeqNum,int,2,2,"getSeqNum()")
{
	return object->getCurrSeqNum();
}


ConsoleMethod(fxFlexBody,loadAction,bool,3,3,"loadAction(char *name)")
{
	return object->loadAction(argv[2]);
}

//(getShapeName() is already being used for something else in shapeBase.)
ConsoleMethod( fxFlexBody, getModelName, const char*, 2, 2, "")
{
   return object->mShapeName;
}



//////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////

ConsoleMethod( fxFlexBody, getShapeConstructor, int,  2, 2, "getShapeConstructor()" )
{
	return object->getShapeConstructor();
}

ConsoleMethod( fxFlexBody, loadDsq, void,  3, 3,"(char filename)")
{

	object->loadDsq(argv[2]);
	return;
}


//ConsoleMethod( fxFlexBody, play, void,  3, 3,"(char seqname)")
//{
//	int seq = object->getShape()->findSequence(argv[2]);
//	if (seq<0) 
//	{
//		//Con::errorf("can't find sequence %s",argv[2]);
//		return;
//	}
//	////Con::errorf("playing sequence: %d  %s",seq,argv[2]);
//	//object->setThreadSequence(0,seq);
//	object->playThread(0,argv[2]);
//	return;
//}

ConsoleMethod( fxFlexBody, playAtPos, void,  3, 4,"(char filename)")
{

	int seq = object->getShape()->findSequence(argv[2]);
	float pos = 0.0;
	if (argc==4) pos = strtod(argv[3]);

	if (seq<0) 
	{
		//Con::errorf("can't find sequence %s",argv[2]);
		return;
	}
	//FIX: everywhere I'm assuming slot (0), change to an argument, and use a global $slot hooked to the gui.  
	//Maybe a dropdown that constrains it to (0-3).
	object->stopThread(0);
	//object->setThreadSequence(0,seq,true);
	object->setThreadPos(0,pos);
	object->setThreadSequence(0,seq,true,pos);

	return;
}

//ConsoleMethod( fxFlexBody, saveSingleSeq, void,  3, 4,"(char filename,char seqname)")
////ConsoleMethod( fxFlexBody, saveOut, void,  2, 2,"")
//{
//	FileStream outstream;
//	char filename[255];
//	char filename2[255];
//
//
//}

ConsoleMethod( fxFlexBody, saveOut, void,  3, 3,"(char filename)")
//ConsoleMethod( fxFlexBody, saveOut, void,  2, 2,"")
{
	FileStream *outstream;
	char filename[255];
	char filename2[255];

	const String myPath = object->getShapeInstance()->getShapeResource()->getPath().getPath();
	strcpy(filename,argv[2]);
	if (!strstr(filename,".dsq")) strcat(filename,".dsq");

	sprintf(filename2,255,"\"%s\"",filename);
	//HERE: need a function to save out only one sequence without dropping all the rest of them first.
	//if ((argc==3)&&(argv[2]))
	//Con::printf("Saving dsq file: %s",filename);
	//if (!gResourceManager->openFileForWrite(outstream,filename)) {
	if ((outstream = FileStream::createAndOpen( filename, Torque::FS::File::Write))==NULL) {
		//Con::errorf("whoops, name no good!"); 
	} else {
		object->getShapeInstance()->getShape()->exportSequences((Stream *)outstream);
		outstream->close();
	}	
}

ConsoleMethod( fxFlexBody, saveSequence, void,  3, 4,"(char filename,char seqname)")
{
	//HERE: now we need to use Chris R's official save sequence function.


	FileStream *outstream;
	char filename[255];
	int seq = -1;

	TSShape *kShape = object->getShapeInstance()->getShape();
	if (argc==4)
	{
		seq = kShape->findSequence(argv[3]);
		if (seq == -1) {
			//Con::errorf("Whoops, can't find sequence %s",argv[3]);
			return;
		}
	} else {// else assume the last one.
		seq = kShape->sequences.size() - 1;
	}
	
	const String myPath = object->getShapeInstance()->getShapeResource()->getPath().getPath();
	strcpy(filename,argv[2]);
	if (!strstr(filename,".dsq")) 
		strcat(filename,".dsq");

	if ((outstream = FileStream::createAndOpen( filename, Torque::FS::File::Write))==NULL) 
	{
		//Con::printf("whoops, name no good!"); 
	} else {
		kShape->exportSequence((Stream *)outstream,kShape->sequences[seq],1);
		outstream->close();
	}
}


ConsoleMethod( fxFlexBody, write, void,  3, 3,"(char filename)")
{
	FileStream *outstream;
	TSShape *kShape = object->getShapeInstance()->getShape();
	if ((outstream = FileStream::createAndOpen( argv[2], Torque::FS::File::Write))==NULL) {
		//Con::printf("whoops, name no good!"); 
	} else {
		kShape->write(outstream);
		outstream->close();
	}	
}

ConsoleMethod( fxFlexBody, showSeq, void, 2, 2, "")
{	
	TSShape *kShape = object->getShapeInstance()->getShape();

	//Con::errorf("ground Rotations: %d, translations %d",kShape->groundRotations.size(),kShape->groundTranslations.size());
	for (unsigned int i=0;i<kShape->sequences.size();i++)
	{
		TSShape::Sequence & seq = kShape->sequences[i];

		//Con::printf("Seq[%d] %s frames: %d duration %3.2f baseObjectState %d baseScale %d baseDecalState %d toolbegin %f",
			i,kShape->getName(seq.nameIndex).c_str(),seq.numKeyframes,
			seq.duration,kShape->sequences[i].baseObjectState,kShape->sequences[i].baseScale,
			kShape->sequences[i].baseDecalState,seq.toolBegin);
		//Con::printf("   groundFrames %d first %d isBlend %d isCyclic %d flags %d",
			seq.numGroundFrames,seq.firstGroundFrame,seq.isBlend(),seq.isCyclic(),seq.flags);
	}
}


ConsoleMethod( fxFlexBody, threadInfo, void, 2, 3, "threadInfo(int index")
{
	//Con::errorf("--------  Shape Thread Info -----------");
	TSShapeInstance *kShapeInst = object->getShapeInstance();
	unsigned int index = 0;
	////Con::errorf("thread list size: %d",kShapeInst->mThreadList.size());

	if (argc==3) index = strtol(argv[2]);
	object->threadInfo(index);

	//Con::errorf("--------   End Thread Info  -----------");
}


ConsoleMethod( fxFlexBody, setAnimationFreeze, void, 3, 3, "setAnimationFreeze(bool)")
{
	int f = strtol(argv[2]);
	if (f) object->mAnimationFreeze = true;
	else object->mAnimationFreeze = false;

}

ConsoleMethod( fxFlexBody, getNumSeqs, int, 2, 2, "getNumSeqs()")
{
	TSShape *kShape = object->getShapeInstance()->getShape();
	return kShape->sequences.size();
}

ConsoleMethod( fxFlexBody, getSeqName, const char *, 3, 3, "getSeqName(unsigned int index)")
{	
	TSShape *kShape = object->getShapeInstance()->getShape();

   char *returnBuffer = //Con::getReturnBuffer(256);
	if (argc>2) 
	{
		unsigned int index = strtol(argv[2]);
		if ((index>=0)&&(index<kShape->sequences.size()))  
		{
			TSShape::Sequence & seq = kShape->sequences[index];

			sprintf(returnBuffer,256,"%s",kShape->getName(seq.nameIndex).c_str());
		}
	}
	return returnBuffer;
}

ConsoleMethod( fxFlexBody, getSeqFilename, const char *, 3, 3, "getSeqFilename(const char *name)")
{	
	TSShape *kShape = object->getShapeInstance()->getShape();

   char *returnBuffer = //Con::getReturnBuffer(256);
	if (argc>2) 
	{
		unsigned int index = kShape->findSequence(argv[2]);
		if ((index>=0)&&(index<kShape->sequences.size()))  
		{
			TSShape::Sequence & seq = kShape->sequences[index];
			sprintf(returnBuffer,256,"%s",seq.sourceData.from.c_str());
		}
	}
	return returnBuffer;
}


ConsoleMethod( fxFlexBody, getSeqNum, int, 3, 3, "getSeqNum(const char *name)")
{	
	return object->getShape()->findSequence(argv[2]);
}

ConsoleMethod( fxFlexBody, getPlaylistNum, int, 3, 3, "getPlaylistNum(int seqnum)")
{	
	int seq;
	seq = strtol(argv[2]);
	return object->getPlaylistNum(seq);
}



ConsoleMethod( fxFlexBody, groundCaptureSeq, void, 3, 3, "groundCaptureSeq(unsigned int seq)")
{	
	int seq = strtol(argv[2]);
	TSShape *kShape = object->getShapeInstance()->getShape();
	kShape->groundCaptureSeq(seq);
	return;
}

ConsoleMethod( fxFlexBody, groundCaptureDir, void, 3, 4, "groundCaptureDir(char *inDir, char *outDir)")
{	
	if (argc==4)
		object->groundCaptureDir(argv[2],argv[3]);
	else 
		object->groundCaptureDir(argv[2],argv[2]);

	return;
}

ConsoleMethod( fxFlexBody, saveBvhDir, void, 4, 4, "saveBvhDir(char *inDir, char *outDir)")
{	
	if (argc==4)
		object->saveBvhDir(argv[2],argv[3]);
	//else 
	//	object->saveBvhDir(argv[2],argv[2]);

	return;
}

ConsoleMethod( fxFlexBody, getSeqCyclic, int, 3, 3, "getSeqCyclic(unsigned int index)")
{	
	TSShape *kShape = object->getShapeInstance()->getShape();
	if (argc>2) 
	{
		unsigned int index = strtol(argv[2]);
		if ((index>=0)&&(index<kShape->sequences.size())) 
		{
			TSShape::Sequence & seq = kShape->sequences[index];
			return seq.isCyclic();
		}
	}
	return 0;
}

void fxFlexBody::setSeqCyclic(int seq,bool cyclic)
{
	TSShape *kShape = getShapeInstance()->getShape();

	if ((seq>=0)&&(seq<kShape->sequences.size())) 
	{
		TSShape::Sequence & kSeq = kShape->sequences[seq];
		if (cyclic) kSeq.flags = kSeq.flags | TSShape::Cyclic;
		else {
			if (kSeq.isCyclic())
				kSeq.flags = kSeq.flags ^ TSShape::Cyclic;
		}
	}
	return;
}

ConsoleMethod( fxFlexBody, setSeqCyclic, void, 4, 4, "setSeqCyclic(unsigned int index, bool cyclic)")
{	
	int seq = strtol(argv[2]);
	bool cyclic = dAtob(argv[3]);
	object->setSeqCyclic(seq,cyclic);
	return;
}

ConsoleMethod( fxFlexBody, getSeqBlend, int, 3, 3, "getSeqBlend(unsigned int index)")
{	
	TSShape *kShape = object->getShapeInstance()->getShape();
	if (argc>2) 
	{
		unsigned int index = strtol(argv[2]);
		if ((index>=0)&&(index<kShape->sequences.size())) 
		{
			TSShape::Sequence & seq = kShape->sequences[index];
			return seq.isBlend();
		}
	}
	return 0;
}

ConsoleMethod( fxFlexBody, setSeqBlend, void, 4, 4, "setSeqBlend(unsigned int index, bool blend)")
{	
	TSShape *kShape = object->getShapeInstance()->getShape();
	if (argc>2) 
	{
		unsigned int index = strtol(argv[2]);
		if ((index>=0)&&(index<kShape->sequences.size()))  
		{
			TSShape::Sequence & seq = kShape->sequences[index];
			if (dAtob(argv[3])) seq.flags = seq.flags | TSShape::Blend;
			else {
				if (seq.isBlend())
					seq.flags = seq.flags ^ TSShape::Blend;
			}
		}
	}
	return;
}

int fxFlexBody::getSeqNumKeyframes(int index)
{
	TSShape *kShape = getShapeInstance()->getShape();
	if ((index>=0)&&(index<kShape->sequences.size()))  
	{
		TSShape::Sequence & seq = kShape->sequences[index];
		return seq.numKeyframes;
	}
	return 0;
}

ConsoleMethod( fxFlexBody, getSeqNumKeyframes, int, 3, 3, "getSeqNumKeyframes(unsigned int index)")
{	
	return object->getSeqNumKeyframes(strtol(argv[2]));
}

ConsoleMethod( fxFlexBody, getSeqNumGroundFrames, int, 3, 3, "getSeqNumGroundframes(unsigned int index)")
{	
	TSShape *kShape = object->getShapeInstance()->getShape();
	if (argc>2) 
	{
		unsigned int index = strtol(argv[2]);
		if ((index>=0)&&(index<kShape->sequences.size()))  
		{
			TSShape::Sequence & seq = kShape->sequences[index];
			return seq.numGroundFrames;
		}
	}
	return 0;
}

ConsoleMethod( fxFlexBody, getSeqNumTriggers, int, 3, 3, "getSeqNumTriggers(unsigned int index)")
{	
	TSShape *kShape = object->getShapeInstance()->getShape();
	if (argc>2) 
	{
		unsigned int index = strtol(argv[2]);
		if ((index>=0)&&(index<kShape->sequences.size())) 
		{
			TSShape::Sequence & seq = kShape->sequences[index];
			return seq.numTriggers;
		}
	}
	return 0;
}

ConsoleMethod( fxFlexBody, getSeqDuration, float, 3, 3, "getSeqDuration(unsigned int index)")
{	
	TSShape *kShape = object->getShapeInstance()->getShape();
	if (argc>2) 
	{
		unsigned int index = strtol(argv[2]);
		if ((index>=0)&&(index<kShape->sequences.size())) 
		{
			TSShape::Sequence & seq = kShape->sequences[index];
			float dur = seq.duration;
			int durRounded = (int)(dur * 1000.0);
			dur = (float)(durRounded)/1000.0;
			return dur;
		}
	}
	return 0.0;
}

ConsoleMethod( fxFlexBody, setSeqDuration, void, 4, 4, "getSeqDuration(unsigned int index, float duration)")
{	
	TSShape *kShape = object->getShapeInstance()->getShape();
	if (argc>2) 
	{
		unsigned int index = strtol(argv[2]);
		if ((index>=0)&&(index<kShape->sequences.size())) 
		{
			TSShape::Sequence & seq = kShape->sequences[index];
			seq.duration = strtod(argv[3]);
		}
	}
	return;
}

ConsoleMethod( fxFlexBody, getSeqPos, float, 2, 3, "getSeqPos(int slot)")
{	
	//Shit, this isn't what I need, need TSThread not Thread.
	//Thread& st = object->mScriptThread[0];
	////Con::errorf("found a thread, isAlive = %d",st->isAlive());
	//if (st->isAlive() return 1.0;
	//else return 0.5;

	return 0.0;
}

ConsoleMethod( fxFlexBody, getNumNodes, int, 2, 2, "getNumNodes()")
{
	TSShape *kShape = object->getShapeInstance()->getShape();
	return kShape->nodes.size();
}

ConsoleMethod( fxFlexBody, getNodeName, const char *, 3, 3, "getNodeName(unsigned int index)")
{	
	TSShape *kShape = object->getShapeInstance()->getShape();
	char *returnBuffer = //Con::getReturnBuffer(256);
	if (argc>2) 
	{
		unsigned int index = strtol(argv[2]);
		if ((index>=0)&&(index<kShape->nodes.size())) 
		{
			//TSShape::Sequence & seq = kShape->sequences[index];
			sprintf(returnBuffer,256,"%s",kShape->getName(kShape->nodes[index].nameIndex).c_str());
		}
	}
	return returnBuffer;
}

ConsoleMethod( fxFlexBody, getNodeNum, int, 3, 3, "getNodeNum(const char *name)")
{	
	return object->getShape()->findNode(argv[2]);
}

ConsoleMethod( fxFlexBody, showNodes, void, 2, 2, "")
{
	TSShape *kShape = object->getShapeInstance()->getShape();
	Ogre::Quaternion q;
	for (unsigned int i=0;i<kShape->nodes.size();i++)
	{
		q = kShape->defaultRotations[i].getOgre::Quaternion();
		//Con::printf("nodes[%d] %s  %f %f %f  rot %f %f %f %f len %f",i,kShape->getName(kShape->nodes[i].nameIndex).c_str(),kShape->defaultTranslations[i].x,
			kShape->defaultTranslations[i].y,kShape->defaultTranslations[i].z,
			q.x,q.y,q.z,q.w,kShape->defaultTranslations[i].length());
	}
}

ConsoleMethod( fxFlexBody, showNodeRots, void, 3, 3, "showNodeRots(seq)")
{
	int seq,node;
	sscanf(argv[2],"%d",&seq);
	TSShape *kShape = object->getShapeInstance()->getShape();
	for (unsigned int i=0;i<kShape->sequences[seq].numKeyframes;i++)
	{
		node=0;
		//Con::errorf("////////////////////////////////////////////");
		for (unsigned int j=0;j<kShape->nodes.size();j++)
		{
			if (kShape->sequences[seq].rotationMatters.test(j))
			{
				Quat16 q16 = kShape->nodeRotations[i+kShape->sequences[seq].baseRotation+node];
				Ogre::Quaternion q;
				q16.getOgre::Quaternion(&q);
				//Con::printf("frame %d nodes[%d] %3.2f %3.2f %3.2f %3.2f",i,j,q.x,q.y,q.z,q.w);
				node++;
			}
		}
	}
}

ConsoleMethod( fxFlexBody, getNodeFrameQuat, const char *, 5, 5, "getNodeFrameQuat(seq,frame,node)")
{
	char *returnBuffer = //Con::getReturnBuffer(256);
	int seq = strtol(argv[2]);
	int frame = strtol(argv[3]);
	int mattersNode = strtol(argv[4]);

	TSShape *kShape = object->getShapeInstance()->getShape();
	int num_frames = kShape->sequences[seq].numKeyframes;
	Quat16 q16 = kShape->nodeRotations[kShape->sequences[seq].baseRotation+(mattersNode*num_frames)+frame];
	Ogre::Quaternion q;
	q = q16.getOgre::Quaternion();
	sprintf(returnBuffer,256,"%1.3g %1.3g %1.3g %1.3g",q.x,q.y,q.z,q.w);
	return returnBuffer;
}

ConsoleMethod( fxFlexBody, setNodeFrameQuat, void, 6, 6, "setNodeFrameQuat(seq,frame,node,quat)")
{
	char *returnBuffer = //Con::getReturnBuffer(256);
	int seq = strtol(argv[2]);
	int frame = strtol(argv[3]);
	int mattersNode = strtol(argv[4]);

	TSShape *kShape = object->getShapeInstance()->getShape();
	int num_frames = kShape->sequences[seq].numKeyframes;

	Ogre::Quaternion q;
	sscanf(argv[5],"%g %g %g %g",&q.x,&q.y,&q.z,&q.w);

	Quat16 q16;
	q16.set(q);
	kShape->nodeRotations[kShape->sequences[seq].baseRotation+(mattersNode*num_frames)+frame] = q16;
	
	return;
}


ConsoleMethod( fxFlexBody, getNodeFrameEuler, const char *, 5, 5, "getNodeFrameEuler(seq,frame,node)")
{
	char *returnBuffer = //Con::getReturnBuffer(256);
	int seq = strtol(argv[2]);
	int frame = strtol(argv[3]);
	int mattersNode = strtol(argv[4]);

	TSShape *kShape = object->getShapeInstance()->getShape();
	int num_frames = kShape->sequences[seq].numKeyframes;


	Quat16 q16 = kShape->nodeRotations[kShape->sequences[seq].baseRotation+(mattersNode*num_frames)+frame];
	Ogre::Quaternion q;
	q = q16.getOgre::Quaternion();
	Ogre::Matrix3 mat;
	q.setMatrix(&mat);
	Ogre::Vector3 eul = mat.toEuler();
	eul *= 180.0/M_PI;
	sprintf(returnBuffer,256,"%3.3g %3.3g %3.3g",eul.x,eul.y,eul.z);
	return returnBuffer;
}

ConsoleMethod( fxFlexBody, setNodeFrameEuler, void, 6, 6, "setNodeFrameEuler(seq,frame,node,euler)")
{
	char *returnBuffer = //Con::getReturnBuffer(256);
	int seq = strtol(argv[2]);
	int frame = strtol(argv[3]);
	int mattersNode = strtol(argv[4]);

	TSShape *kShape = object->getShapeInstance()->getShape();
	int num_frames = kShape->sequences[seq].numKeyframes;
	
	Ogre::Vector3 eul;
	sscanf(argv[5],"%g %g %g",&eul.x,&eul.y,&eul.z);
	//Con::errorf("setting node frame euler: %g %g %g",eul.x,eul.y,eul.z);
	eul *= M_PI/180.0;
	Ogre::Quaternion q(eul);
	Quat16 q16;
	q16.set(q);
	kShape->nodeRotations[kShape->sequences[seq].baseRotation+(mattersNode*num_frames)+frame] = q16;
	
	return;
}

ConsoleMethod( fxFlexBody, showNodeTrans, void, 3, 3, "showNodeTrans(seq)")
{
	int seq;
	sscanf(argv[2],"%d",&seq);
	TSShape *kShape = object->getShapeInstance()->getShape();
	for (unsigned int i=0;i<kShape->sequences[seq].numKeyframes;i++)
	{
		Ogre::Vector3 p = kShape->nodeTranslations[i+kShape->sequences[seq].baseTranslation];
		//Con::printf("frame %d %3.2f %3.2f %3.2f",i,p.x,p.y,p.z);
	}
}

ConsoleMethod( fxFlexBody, showGroundRots, void, 3, 3, "showGroundRots(seq)")
{
	int seq;
	sscanf(argv[2],"%d",&seq);
	TSShape *kShape = object->getShapeInstance()->getShape();
	for (unsigned int i=kShape->sequences[seq].firstGroundFrame;i<kShape->sequences[seq].numGroundFrames+kShape->sequences[seq].firstGroundFrame;i++)
	{
		Quat16 q16 = kShape->groundRotations[i];
		Ogre::Quaternion q;
		q16.getOgre::Quaternion(&q);
		//Con::printf("frame %d %3.2f %3.2f %3.2f %3.2f",i,q.x,q.y,q.z,q.w);
	}
}

ConsoleMethod( fxFlexBody, showGroundTrans, void, 3, 3, "showGroundTrans(seq)")
{
	int seq;
	sscanf(argv[2],"%d",&seq);
	TSShape *kShape = object->getShapeInstance()->getShape();
	for (unsigned int i=kShape->sequences[seq].firstGroundFrame;i<kShape->sequences[seq].numGroundFrames+kShape->sequences[seq].firstGroundFrame;i++)
	{
		Ogre::Vector3 gt = kShape->groundTranslations[i];
		//Con::printf("frame %d %3.2f %3.2f %3.2f",i,gt.x,gt.y,gt.z);
	}
}

ConsoleMethod( fxFlexBody, showDefRots, void, 2, 2, "showDefRots()")
{
	TSShape *kShape = object->getShapeInstance()->getShape();
	for (unsigned int j=0;j<kShape->nodes.size();j++)
	{
		//if (kShape->sequences[13].rotationMatters.test(j))
		if (1)
		{
			Ogre::Quaternion q; Ogre::Matrix3 m; Ogre::Vector3 e, ei;
			kShape->defaultRotations[j].getOgre::Quaternion(&q);
			q.setMatrix(&m);
			e = m.toEuler();
			e.x = mRadToDeg(e.x); e.y = mRadToDeg(e.y); e.z = mRadToDeg(e.z); 
			m.inverse();
			ei = m.toEuler();
			ei.x = mRadToDeg(ei.x); ei.y = mRadToDeg(ei.y); ei.z = mRadToDeg(ei.z); 
			//Con::printf("defRots[%d] %3.2f %3.2f %3.2f %3.2f, euler %3.2f %3.2f %3.2f,inverse %3.2f %3.2f %3.2f",
				j,q.x,q.y,q.z,q.w,e.x,e.y,e.z,ei.x,ei.y,ei.z);
		}
	}
}

ConsoleMethod( fxFlexBody, fixCfg, void, 4, 4, "fixCfg(cfg_filename,bvh_filename)")
{
	object->fixBvhCfg(argv[2],argv[3]);
}




ConsoleMethod( fxFlexBody, hasUltraframesForNode, bool, 4, 4, "hasUltraframesForNode(seq,node)")
{
	int seq = strtol(argv[2]);
	int node = strtol(argv[3]);
	return object->hasUltraframesForNode(seq,node);
}

ConsoleMethod( fxFlexBody, hasUltraframesForType, bool, 4, 4, "hasUltraframesForType(seq,type)")
{
	int seq = strtol(argv[2]);
	int type = strtol(argv[3]);
	return object->hasUltraframesForType(seq,type);
}


///////////////////////////////////////////////////////////

ConsoleMethod( fxFlexBody, orientToPosition, void, 3, 3, "orientToPosition(Ogre::Vector3 pos)")
{
	Ogre::Vector3 pos;
	sscanf(argv[2],"%g %g %g",&pos.x,&pos.y,&pos.z);
	object->orientToPosition(pos);
}


ConsoleMethod( fxFlexBody, moveToPosition, void, 4, 4, "moveToPosition(Ogre::Vector3 pos,String action)")
{
	Ogre::Vector3 pos;
	sscanf(argv[2],"%g %g %g",&pos.x,&pos.y,&pos.z);
	object->moveToPosition(pos,argv[3]);
}

ConsoleMethod( fxFlexBody, setMoveThreshold, void, 3, 3, "setMoveThreshold(float value)")
{
	float value;
	sscanf(argv[2],"%f",&value);
	object->mMoveThreshold = value;
}

ConsoleMethod( fxFlexBody, getMoveThreshold, float, 2, 2, "getMoveThreshold()")
{
	return object->mMoveThreshold;
}

ConsoleMethod( fxFlexBody, setMoveSequence, void, 3, 3, "setMoveSequence(String action)")
{
	object->mMoveSequence = argv[2];
}


ConsoleMethod( fxFlexBody, attackPosition, void, 4, 4, "attackPosition(Ogre::Vector3 pos,String action)")
{
	Ogre::Vector3 pos;
	sscanf(argv[2],"%g %g %g",&pos.x,&pos.y,&pos.z);
	object->attackPosition(pos,argv[3]);
}

ConsoleMethod( fxFlexBody, saveShapeConstructor, void, 2, 3, "saveShapeConstructor(const char *filename)")
{
	if (argc>2)
		object->saveShapeConstructor(argv[2]);
	else
		object->saveShapeConstructor("");

}

////////////////////////////////////////////////////////////////////////////////////

ConsoleMethod( fxFlexBody, addPlaylistSeq, void, 6, 6, "addPlaylistSeq()")
{//HERE: also need to insert into database if we add playlist sequence through the gui.
	int seq, repeats, playlist_id;
	float speed;

	sscanf(argv[2],"%d",&seq);
	sscanf(argv[3],"%d",&repeats);
	sscanf(argv[4],"%g",&speed);
	sscanf(argv[5],"%d",&playlist_id);
	
	if (playlist_id <=0 )
		return;

	////First, create a playlist in the database, so we don't have to come here again.
	//int scene_id = dynamic_cast<nxPhysManager*>(object->mPM)->mSceneId;
	//String scene_name = dynamic_cast<nxPhysManager*>(object->mPM)->mSceneName;
	//SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(object->mPM)->getSQL();
	//TSShape *kShape = object->getShapeInstance()->getShape();

	//if (sql)
	//{
	//	if (sql->OpenDatabase("EcstasyMotion.db"))
	//	{
	//		char insert_query[512],sequence_id_query[512],seq_order_query[512],playlistName[512],seqName[255];//playlist_id_query[512],
	//		int result,sequence_id=0;
	//		float lastOrder=0.0,nextOrder=0.0;
	//		sqlite_resultset *resultSet,*resultSet2;

	//		result = sql->ExecuteSQL("BEGIN TRANSACTION;");

	//		sprintf(seqName,"%s",kShape->getName(kShape->sequences[seq].nameIndex).c_str());
	//		sprintf(sequence_id_query,"SELECT id FROM sequence WHERE name='%s' AND skeleton_id=%d;",seqName,object->mSkeletonID);
	//		result = sql->ExecuteSQL(sequence_id_query);
	//		resultSet = sql->GetResultSet(result);
	//		if (resultSet->iNumRows >= 1)
	//			sequence_id = strtol(resultSet->vRows[0]->vColumnValues[0]);

	//		//sprintf(playlist_id_query,"SELECT id FROM playlist WHERE actor_id=%d AND scene_id=%d;",
	//		//	object->mActorID,scene_id);
	//		//result = sql->ExecuteSQL(playlist_id_query);
	//		//resultSet = sql->GetResultSet(result);
	//		//if (resultSet->iNumRows >= 1)
	//		//{//Danger, if there is more than one then we just grab the first one.  Should make there be only one.
	//		//	playlist_id = strtol(resultSet->vRows[0]->vColumnValues[0]);
	//		//} else {
	//		//	sprintf(playlistName,"%s.%s.default",scene_name.c_str(),object->mActorName);
	//		//	sprintf(insert_query,"INSERT INTO playlist (skeleton_id,actor_id,scene_id,name) VALUES (%d,%d,%d,'%s');",
	//		//		object->mSkeletonID,object->mActorID,scene_id,playlistName);
	//		//	result = sql->ExecuteSQL(insert_query);
	//		//	result = sql->ExecuteSQL(playlist_id_query);
	//		//	resultSet = sql->GetResultSet(result);
	//		//	if (resultSet->iNumRows == 1)
	//		//		playlist_id = strtol(resultSet->vRows[0]->vColumnValues[0]);
	//		//}
	//		//FIX: need to be able to insert, this will only append.
	//		sprintf(seq_order_query,"SELECT sequence_order FROM playlistSequence WHERE playlist_id = %d ORDER BY sequence_order DESC;",
	//			playlist_id);//order descending so the highest order number will be on top, and we can safely add 1.0 to that.
	//		result = sql->ExecuteSQL(seq_order_query);//Later, need to expose this variable and make it the same kind of interface as scene events, so we can insert.
	//		resultSet2 = sql->GetResultSet(result);
	//		if (resultSet2->iNumRows > 0)
	//			lastOrder = strtod(resultSet2->vRows[0]->vColumnValues[0]);
	//		if (lastOrder>=0.0)
	//			nextOrder = (int)(lastOrder + 1.0);

	//		sprintf(insert_query,"INSERT INTO playlistSequence (playlist_id,sequence_id,sequence_order,repeats,speed) VALUES (%d,%d,%f,%d,%f);",
	//			playlist_id,sequence_id,nextOrder,repeats,speed);
	//		result = sql->ExecuteSQL(insert_query);

	//		result = sql->ExecuteSQL("END TRANSACTION;");
	//		sql->CloseDatabase();
	//	}
	//	delete sql;
	//}
	object->addPlaylistSeq(seq,repeats,speed);
}

ConsoleMethod( fxFlexBody, dropPlaylistSeq, void, 3, 3, "dropPlaylistSeq()")
{
	int index;
	sscanf(argv[2],"%d",&index);
	object->dropPlaylistSeq(index);	
}

ConsoleMethod( fxFlexBody, savePlaylistSeq, void, 7, 7, "savePlaylistSeq()")
{
	int id,index,seq,repeats;
	float speed,order;

	sscanf(argv[2],"%d",&id);
	sscanf(argv[3],"%d",&seq);
	sscanf(argv[4],"%d",&repeats);
	sscanf(argv[5],"%g",&speed);
	sscanf(argv[6],"%g",&order);

	//Following is not necessary here - doing it in script instead before we call this function.
	//int scene_id = dynamic_cast<nxPhysManager*>(object->mPM)->mSceneId;
	//String scene_name = dynamic_cast<nxPhysManager*>(object->mPM)->mSceneName;
	//SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(object->mPM)->getSQL();
	//TSShape *kShape = object->getShapeInstance()->getShape();

	//if (sql)
	//{
	//	if (sql->OpenDatabase("EcstasyMotion.db"))
	//	{
	//		char insert_query[512],playlist_id_query[512],sequence_id_query[512],seq_order_query[512],playlistName[512],seqName[255];
	//		int result,sequence_id=0,playlist_id=0;
	//		float lastOrder=0.0,nextOrder=0.0;
	//		sqlite_resultset *resultSet,*resultSet2;

	//		sprintf(seqName,"%s",kShape->getName(kShape->sequences[seq].nameIndex).c_str());
	//		sprintf(sequence_id_query,"SELECT id FROM sequence WHERE name='%s' AND skeleton_id=%d;",seqName,object->mSkeletonID);
	//		result = sql->ExecuteSQL(sequence_id_query);
	//		resultSet = sql->GetResultSet(result);
	//		if (resultSet->iNumRows == 1)
	//			sequence_id = strtol(resultSet->vRows[0]->vColumnValues[0]);

	//		sprintf(playlist_id_query,"SELECT id FROM playlist WHERE actor_id=%d AND scene_id=%d;",
	//			object->mActorID,scene_id);
	//		result = sql->ExecuteSQL(playlist_id_query);
	//		resultSet = sql->GetResultSet(result);
	//		if (resultSet->iNumRows == 1)
	//		{
	//			playlist_id = strtol(resultSet->vRows[0]->vColumnValues[0]);
	//		}
	//		if (playlist_id)
	//		{//FIX: need to be able to insert, this will only append.


	//			sprintf(seq_order_query,"SELECT id FROM playlistSequence WHERE playlist_id = %d ORDER BY sequence_order ASC;",playlist_id);
	//			result = sql->ExecuteSQL(seq_order_query);
	//			resultSet = sql->GetResultSet(result);
	//			index = 0;
	//			for (unsigned int i=0;i<resultSet->iNumRows;i++)
	//			{
	//				if (id == strtol(resultSet->vRows[0]->vColumnValues[0]))
	//				{
	//					index = id;
	//					break;
	//				}
	//			}

	//			sprintf(seq_order_query,"UPDATE playlistSequence SET repeats = %d, speed = %f, order = %f WHERE id = %d;",
	//				repeats,speed,order,id);//order descending so the highest order number will be on top, and we can safely add 1.0 to that.
	//			result = sql->ExecuteSQL(seq_order_query);//Later, need to expose this variable and make it the same kind of interface as scene events, so we can insert.
	//		}
	//		sql->CloseDatabase();
	//		delete sql;
	//	}
	//}

	//This is the only necessary engine-side work here - to set the values in the local mPlaylist array.
	object->savePlaylistSeq(index,seq,repeats,speed);
}

ConsoleMethod( fxFlexBody, getNumPlaylistSeqs, int, 2, 2, "getNumPlaylistSeqs()")
{
	return object->getNumPlaylistSeqs();
}

ConsoleMethod( fxFlexBody, getPlaylistSeq, int, 3, 3, "getPlaylistSeq()")
{
	int index;
	sscanf(argv[2],"%d",&index);
	return object->getPlaylistSeq(index);
}

ConsoleMethod( fxFlexBody, getPlaylistRepeats, int, 3, 3, "getPlaylistRepeats()")
{
	int index;
	sscanf(argv[2],"%d",&index);
	return object->getPlaylistRepeats(index);
}

ConsoleMethod( fxFlexBody, getPlaylistSpeed, float, 3, 3, "getPlaylistSpeed()")
{
	int index;
	sscanf(argv[2],"%d",&index);
	return object->getPlaylistSpeed(index);
}

ConsoleMethod( fxFlexBody, runPlaylist, void, 2, 2, "runPlaylist()")
{
	object->runPlaylist();
	return;
}

ConsoleMethod( fxFlexBody, savePlaylist, void, 3, 3, "savePlaylist(filename)")
{
	object->savePlaylist(argv[2]);
	return;
}


ConsoleMethod( fxFlexBody, loadPlaylist, void, 2, 3, "loadPlaylist(int scene_id)")
{
	if (argc==3)
		object->loadPlaylist(strtol(argv[2]));//scene id
	else
		object->loadPlaylist(dynamic_cast<nxPhysManager*>(object->mPM)->mSceneId);

	return;
}

ConsoleMethod( fxFlexBody, loadPlaylistById, void, 3, 3, "loadPlaylist(int playlist_id)")
{
	object->loadPlaylistById(strtol(argv[2]));

	return;
}

ConsoleMethod( fxFlexBody, clearPlaylist, void, 2, 2, "dropPlaylist")
{
	object->clearPlaylist();
}

///////////////////////////////////////////////////////////////////////
ConsoleMethod( fxFlexBody, adjustBaseNodePosRegion, void, 6, 6, "adjustBaseNodePosRegion(seq,pos,start,stop)")
{
	int seq;
	Ogre::Vector3 pos;
	float start,stop;

	sscanf(argv[2],"%d",&seq);
	sscanf(argv[3],"%g %g %g",&pos.x,&pos.y,&pos.z);
	start = strtod(argv[4]);
	stop = strtod(argv[5]);

	object->adjustBaseNodePosRegion(seq,pos,start,stop);

}

ConsoleMethod( fxFlexBody, adjustBaseNodePosPermanent, void, 4, 4, "adjustBaseNodePosPermanent(seq,pos)")
{
	int seq;
	Ogre::Vector3 pos;

	sscanf(argv[2],"%d",&seq);
	sscanf(argv[3],"%g %g %g",&pos.x,&pos.y,&pos.z);

	object->adjustBaseNodePosRegion(seq,pos,0.0,1.0);

	object->mBaseNodeAdjustPos = pos;
	object->mBaseNodeSetPos = Ogre::Vector3::ZERO;

}

//No need for anything other than base node, because other nodes only store rotations.
ConsoleMethod( fxFlexBody, setBaseNodePosRegion, void, 6, 6, "setBaseNodePosRegion(seq,pos,start,stop)")
{
	int seq;
	Ogre::Vector3 pos;
	float start,stop;

	sscanf(argv[2],"%d",&seq);
	sscanf(argv[3],"%g %g %g",&pos.x,&pos.y,&pos.z);
	start = strtod(argv[4]);
	stop = strtod(argv[5]);
	
	object->setBaseNodePosRegion(seq,pos,start,stop);
}

//No need for anything other than base node, because other nodes only store rotations.
ConsoleMethod( fxFlexBody, setBaseNodePosPermanent, void, 4, 4, "setBaseNodePosPermanent(seq,pos)")
{
	int seq;
	Ogre::Vector3 pos;

	sscanf(argv[2],"%d",&seq);
	sscanf(argv[3],"%g %g %g",&pos.x,&pos.y,&pos.z);

	object->setBaseNodePosRegion(seq,pos,0.0,1.0);

	object->mBaseNodeSetPos = pos;
	object->mBaseNodeAdjustPos = Ogre::Vector3::ZERO;
}

ConsoleMethod( fxFlexBody, setNodeRotRegion, void, 7, 7, "setNodeRotRegion(seq,node,rot,start,stop)")
{
	int seq,node;
	Ogre::Vector3 rot;
	float start,stop;

	sscanf(argv[2],"%d",&seq);
	sscanf(argv[3],"%d",&node);
	sscanf(argv[4],"%g %g %g",&rot.x,&rot.y,&rot.z);
	start = strtod(argv[5]);
	stop = strtod(argv[6]);

	object->setNodeRotRegion(seq,node,rot,start,stop);
}

ConsoleMethod( fxFlexBody, setNodeRotPermanent, void, 4, 4,"setNodeRotPermanent(unsigned int node,Ogre::Vector3 &rot)")
{
	int seq,node;
	Ogre::Vector3 rot;

	sscanf(argv[2],"%d",&seq);
	sscanf(argv[3],"%d",&node);
	sscanf(argv[4],"%g %g %g",&rot.x,&rot.y,&rot.z);

	object->setNodeRotRegion(seq,node,rot, 0.0, 1.0);
	object->addNodeSetRot(node,rot);
	
}

ConsoleMethod( fxFlexBody, adjustNodeRotRegion, void, 7, 7, "adjustNodeRotRegion(seq,node,rot,start,stop)")
{
	int seq,node;
	Ogre::Vector3 rot;
	float start,stop;

	sscanf(argv[2],"%d",&seq);
	sscanf(argv[3],"%d",&node);
	sscanf(argv[4],"%g %g %g",&rot.x,&rot.y,&rot.z);
	start = strtod(argv[5]);
	stop = strtod(argv[6]);

	object->adjustNodeRotRegion(seq,node,rot,start,stop);
}

ConsoleMethod( fxFlexBody, adjustNodeRotPermanent, void, 4, 4,"adjustNodeRotPermanent(unsigned int node,Ogre::Vector3 &rot)")
{
	int seq,node;
	Ogre::Vector3 rot;

	sscanf(argv[2],"%d",&seq);
	sscanf(argv[3],"%d",&node);
	sscanf(argv[4],"%g %g %g",&rot.x,&rot.y,&rot.z);

	object->adjustNodeRotRegion(seq,node,rot,0.0,1.0);
	object->addNodeAdjustRot(node,rot);
}

ConsoleMethod( fxFlexBody, doOgre::Matrix3ix, void, 5, 5, "doOgre::Matrix3ix(seq,Ogre::Vector3,Ogre::Vector3)")
{
	int seq;
	Ogre::Vector3 euler1,euler2;
	sscanf(argv[2],"%d",&seq);
	sscanf(argv[3],"%g %g %g",&euler1.x,&euler1.y,&euler1.z);
	sscanf(argv[4],"%g %g %g",&euler2.x,&euler2.y,&euler2.z);
	object->doOgre::Matrix3ix(seq,euler1,euler2);
}

ConsoleMethod( fxFlexBody, grabSeqByName, void, 5, 5, "grabSequence(other,char sequence,char cfg)")
{
	SimObject *other;
	other = Sim::findObject(argv[2]);
	ShapeBase *otherShapeBase = dynamic_cast<ShapeBase*>(other);

	const String myPath = object->getShapeInstance()->getShapeResource()->getPath().getPath();
	const char *path_str = myPath.c_str();

	if (otherShapeBase) {
		TSShape *otherShape = otherShapeBase->getShapeInstance()->getShape();
		TSShape *kShape = object->getShapeInstance()->getShape();
		int seq = otherShape->findSequence(argv[3]);
		kShape->convertSequence(otherShape,argv[4],seq,path_str);
	}
}

ConsoleMethod( fxFlexBody, grabSeqByNum, void, 5, 5, "grabSeqNum(other,unsigned int sequence,char cfg)")
{
	SimObject *other;
	other = Sim::findObject(argv[2]);
	int seq;
	
	const String myPath = object->getShapeInstance()->getShapeResource()->getPath().getPath();
	const char *path_str = myPath.c_str();

	sscanf(argv[3],"%d",&seq);
	ShapeBase *otherShapeBase = dynamic_cast<ShapeBase*>(other);
	if (otherShapeBase) {
		TSShape *otherShape = otherShapeBase->getShapeInstance()->getShape();
		TSShape *kShape = object->getShapeInstance()->getShape();
		kShape->convertSequence(otherShape,argv[4],seq,path_str);
	}
}

ConsoleMethod( fxFlexBody, dropSeqByName, void, 3, 3, "dropSequence(unsigned int sequence)")
{
	int seq;
	TSShape *kShape = object->getShapeInstance()->getShape();
	seq = kShape->findSequence(argv[2]);
	if (object->getThreadSequence(0) == seq)
		object->setThreadSequence(0,0);
	kShape->dropSequence(seq);
	String seqName = argv[2];
	object->dropUltraframeSet(seqName);
}

ConsoleMethod( fxFlexBody, dropSeqByNum, void, 3, 3, "dropSequence(int sequence)")
{
	int seq;
	sscanf(argv[2],"%d",&seq);
	TSShape *kShape = object->getShapeInstance()->getShape();
	if (object->getThreadSequence(0) == seq)
		object->setThreadSequence(0,0);
	kShape->dropSequence(seq);
	String seqName = kShape->getName(kShape->sequences[seq].nameIndex);
	object->dropUltraframeSet(seqName);
}

ConsoleMethod( fxFlexBody, dropSeq, void, 3, 3, "dropSeq(char sequence)")
{
	int seq;
	sscanf(argv[2],"%d",&seq);
	TSShape *kShape = object->getShapeInstance()->getShape();
	if (object->getThreadSequence(0) == seq)
		object->setThreadSequence(0,0);
	kShape->dropSequence(seq);
	String seqName = kShape->getName(kShape->sequences[seq].nameIndex);
	object->dropUltraframeSet(seqName);
}


ConsoleMethod( fxFlexBody, cropSequence, void, 6, 6, "cropSequence(unsigned int seq, float start, float stop, char name)")
{
	int seq;
	float start,stop;
	sscanf(argv[2],"%d",&seq);
	sscanf(argv[3],"%f",&start);
	sscanf(argv[4],"%f",&stop);

	object->cropSequence(seq,start,stop,argv[5]);
	//need to stop or clear threads, because it deletes all sequences but one and then tries to keep 
	//playing the sequence number it was using before.
	//starting with slot 0 only, since that's all we're using so far, but come back here when we get multi-threads.
}



ConsoleMethod( fxFlexBody, setSequenceFrames, void, 4, 4, "setSequenceFrames(unsigned int seq, unsigned int frames)")
{
	unsigned int seq;
	unsigned int frames;
	sscanf(argv[2],"%d",&seq);
	sscanf(argv[3],"%d",&frames);

	object->setSequenceFrames(seq,frames);
}

ConsoleMethod( fxFlexBody, findStop, float, 4, 4, "findStop(unsigned int slot, float start)")
{
	int seq;
	float start,stop;
	sscanf(argv[2],"%d",&seq);
	sscanf(argv[3],"%f",&start);
	//Con::errorf("finding stop point: seq %d, start %f",seq,start);
	TSShape *kShape = object->getShapeInstance()->getShape();
	stop = kShape->findStop(seq,start);
	if (stop) return stop;
	else return 0.0;
}

ConsoleMethod( fxFlexBody, getNumMattersNodes, int, 3, 3, "getNumMattersNodes(int seqnum)")
{
	int seqnum = strtol(argv[2]);
	TSShape *kShape = object->getShapeInstance()->getShape();
	return kShape->getNumMattersNodes(seqnum);
}


ConsoleMethod( fxFlexBody, getMattersNodeIndex, int, 4, 4, "getMattersNodeIndex(int seqnum,int nodenum)")
{
	int seqnum = strtol(argv[2]);
	int nodenum = strtol(argv[3]);
	
	TSShape *kShape = object->getShapeInstance()->getShape();
	return kShape->getMattersNodeIndex(seqnum,nodenum);
}

ConsoleMethod( fxFlexBody, getNodeMattersIndex, int, 4, 4, "getNodeMattersIndex(int seqnum,int nodenum)")
{
	int seqnum = strtol(argv[2]);
	int nodenum = strtol(argv[3]);
	
	TSShape *kShape = object->getShapeInstance()->getShape();
	return kShape->getNodeMattersIndex(seqnum,nodenum);
}

ConsoleMethod( fxFlexBody, addMattersNode, void, 4, 4, "addMattersNode(int seqnum,int nodenum)")
{
	int seqnum = strtol(argv[2]);
	int nodenum = strtol(argv[3]);
	
	TSShape *kShape = object->getShapeInstance()->getShape();
	kShape->addMattersNode(seqnum,nodenum);
}

ConsoleMethod( fxFlexBody, dropMattersNode, void, 4, 4, "dropMattersNode(int seqnum,int nodenum)")
{
	int seqnum = strtol(argv[2]);
	int nodenum = strtol(argv[3]);
	
	TSShape *kShape = object->getShapeInstance()->getShape();
	kShape->dropMattersNode(seqnum,nodenum);
}

ConsoleMethod( fxFlexBody, info, void, 2, 2, "info()")
{
	TSShape *myShape = object->getShapeInstance()->getShape();
	//Con::errorf("my shape has: nodeTrans %d, nodeRot %d, groundFrames %d, triggers %d.",
		myShape->nodeTranslations.size(),myShape->nodeRotations.size(),myShape->groundRotations.size(),
		myShape->groundRotations.size(),myShape->triggers.size());

}

ConsoleMethod( fxFlexBody, importBvh, void,  5, 5,"(bool importGround,char bvhname,char bvhProfile)")
{
	object->importBvh((bool)strtol(argv[2]),argv[3],argv[4]);
	return;
}

ConsoleMethod( fxFlexBody, bvh, void,  5, 5,"(bool importGround,char bvhname,char bvhProfile)")
{
	object->importBvh(argv[2],argv[3],argv[4]);
	return;
}

ConsoleMethod( fxFlexBody, cleanupBvh, void,  3, 3,"(char filename)")
{	
	object->cleanupBvh(argv[2]);
	return;
}

ConsoleMethod( fxFlexBody, clean, void,  3, 3,"(char filename)")
{
	object->cleanupBvh(argv[2]);
	return;
}

ConsoleMethod( fxFlexBody, nullBvh, void,  3, 3,"(char filename)")
{	
	object->nullBvh(argv[2]);
	return;
}

ConsoleMethod( fxFlexBody, null, void,  3, 3,"(char filename)")
{
	object->nullBvh(argv[2]);
	return;
}

ConsoleMethod( fxFlexBody, saveBvh, void, 4, 5, "(char sequence, char outputname, char outputformat)")
{
	if (argc==4)
		object->saveBvh(strtol(argv[2]),argv[3]);
	else if (argc==5)
		object->saveBvh(strtol(argv[2]),argv[3],argv[4]);
	return;
}


ConsoleMethod( fxFlexBody, importDir, void, 5, 5, "importDir(bool importGround,char bvhDir,char dsqDir)")
{

	object->importDir((bool)strtol(argv[2]),argv[3],argv[3],argv[4]);//argv[3] is doubled because there used
	//to be two directory names, one for the bvh files and one for the dsq files, now it's the same.

	return;
}

ConsoleMethod( fxFlexBody, cleanDir, void, 3, 3, "cleanDir(char dir)")
{
	object->cleanupDir(argv[2]);
	return;
}

ConsoleMethod( fxFlexBody, getPath, const char*, 2, 2, "")
{
	char *returnBuffer = //Con::getReturnBuffer(256);
	const String myPath = object->getShapeInstance()->getShapeResource()->getPath().getPath();
	sprintf(returnBuffer,256,"%s", myPath.c_str());
	return returnBuffer;
}

ConsoleMethod( fxFlexBody, getName, const char *, 2, 2, "getName()")
{
	char *returnBuffer = //Con::getReturnBuffer(256);
	const String myFileName = object->getShapeInstance()->getShapeResource()->getPath().getFileName();
	//const String myFilePath = object->getShapeInstance()->getShapeResource()->getPath();
	////Con::errorf("found a filename: %s %s",myFilePath.c_str(),myFileName.c_str());
	sprintf(returnBuffer,256,"%s", myFileName.c_str());
	return returnBuffer;
}


ConsoleMethod( fxFlexBody, getNodeParent, int, 3, 3, "getNodeName(int)")
{
	int nodeID = strtol(argv[2]);
	const TSShape *kShape = object->getShape();
	
	return kShape->nodes[nodeID].parentIndex;
}


//ConsoleMethod( fxFlexBody, setThreadPos, void, 4, 4, "setThreadPos(int slot, float pos)")
//{
//	int slot = strtol(argv[2]);
//	float pos = strtod(argv[3]);
//	object->setThreadPos(slot,pos);
//	return;
//}


//ConsoleMethod( fxFlexBody, getThreadPos, float, 3, 3, "getThreadPos(int slot)")
//{
//	int slot = strtol(argv[2]);
//	float pos = object->getThreadPos(slot);
//	return pos;
//}

ConsoleMethod( fxFlexBody, getKeyFrame, int, 3, 3, "getKeyFrame(int slot)")
{
	int slot = strtol(argv[2]);
	int pos = object->getKeyFrame(slot);
	return pos;
}

ConsoleMethod( fxFlexBody, pauseThreads, void, 2, 2, "pauseThreads()")
{
	TSShapeInstance *kSI = object->getShapeInstance();
	unsigned int count = kSI->threadCount();
	for (unsigned int i=0;i<count;i++) {
		TSThread *th = kSI->getThread(i);
		//Con::errorf("timescale %d before: %f...",i,kSI->getTimeScale(th));
		kSI->setTimeScale(th,0.0);
		//Con::errorf("... and after: %f",kSI->getTimeScale(th));
	}
	return;
}

ConsoleMethod( fxFlexBody, getThreadTimescale, float, 3, 3, "getThreadTimescale(int slot)")
{
	int i = strtol(argv[2]);
	if (i>=0)
	{
		TSShapeInstance *kSI = object->getShapeInstance();
		TSThread *th = kSI->getThread(i);
		return kSI->getTimeScale(th);
	}
	return 0.0;
}

ConsoleMethod( fxFlexBody, setShapeSize, void, 3, 3, "setShapeSize(float size)")
{	
	object->setShapeSize(strtod(argv[2]));
	return;
}

ConsoleMethod( fxFlexBody, getShapeSize, float, 2, 2, "getShapeSize()")
{	
	return object->getShapeSize();
}


ConsoleMethod( fxFlexBody, backupSequenceData, void, 2, 2, "backupSequenceData()")
{
	object->backupSequenceData();
}

ConsoleMethod( fxFlexBody, hasUltraframeSets, bool, 2, 2, "hasUltraframeSets()")
{
	if (object->mUltraframeSets.size())
		return true;
	else 
		return false;
}

ConsoleMethod( fxFlexBody, hasUltraframe, bool, 6, 6, "hasUltraframe(int seq, int frame, int node, int type)")
{
	int seq = strtol(argv[2]);
	int frame = strtol(argv[3]);
	int node = strtol(argv[4]);
	int type = strtol(argv[5]);
	return object->hasUltraframe(seq,frame,node,type);
}

ConsoleMethod( fxFlexBody, addUltraframe, void, 7, 8, "addUltraframe(int seq, int frame, int node, int type, int target, Ogre::Vector3 value)")
{
	Ogre::Vector3 value;
	int target=0;
	int seq = strtol(argv[2]);
	int frame = strtol(argv[3]);
	int node = strtol(argv[4]);
	int type = strtol(argv[5]);
	if (argc==7){
		sscanf(argv[6],"%g %g %g",&value.x,&value.y,&value.z);
	} else {
		target = strtol(argv[6]);
		sscanf(argv[7],"%g %g %g",&value.x,&value.y,&value.z);
	}
	object->addUltraframe(seq,frame,node,type,target,value);
}

ConsoleMethod( fxFlexBody, addUltraframeNoInsert, void, 7, 8, "addUltraframeNoInsert(int seq, int frame, int node, int type, int target, Ogre::Vector3 value)")
{
	Ogre::Vector3 value;
	int target=0;
	int seq = strtol(argv[2]);
	int frame = strtol(argv[3]);
	int node = strtol(argv[4]);
	int type = strtol(argv[5]);
	if (argc==7){
		sscanf(argv[6],"%g %g %g",&value.x,&value.y,&value.z);
	} else {
		target = strtol(argv[6]);
		sscanf(argv[7],"%g %g %g",&value.x,&value.y,&value.z);
	}
	object->addUltraframeNoInsert(seq,frame,node,type,target,value);
}

ConsoleMethod( fxFlexBody, addUltraframeSingle, void, 7, 8, "addUltraframeSingle(int seq, int frame, int node, int type, Ogre::Vector3 value)")
{
	Ogre::Vector3 value;
	int target=0;
	int seq = strtol(argv[2]);
	int frame = strtol(argv[3]);
	int node = strtol(argv[4]);
	int type = strtol(argv[5]);
	if (argc==7)
	{
		sscanf(argv[6],"%g %g %g",&value.x,&value.y,&value.z);
	} else {
		target = strtol(argv[6]);
		sscanf(argv[7],"%g %g %g",&value.x,&value.y,&value.z);
	}  
	object->addUltraframeSingle(seq,frame,node,type,target,value);
}
ConsoleMethod( fxFlexBody, addUltraframeSet, void, 3, 3, "addUltraframeSet(int seq)")
{   //  (OBSOLETE)
	int seq = strtol(argv[2]);
	object->addUltraframeSet(seq);
}

ConsoleMethod( fxFlexBody, saveUltraframe, void, 7, 8, "saveUltraframe(int seq, int frame, int node, int type, Ogre::Vector3 value)") 
{
	Ogre::Vector3 value;
	int target=0;
	int seq = strtol(argv[2]);
	int frame = strtol(argv[3]);
	int node = strtol(argv[4]);
	int type = strtol(argv[5]);
	if (argc==7)
	{
		sscanf(argv[6],"%g %g %g",&value.x,&value.y,&value.z);
	} else {
		target = strtol(argv[6]);
		sscanf(argv[7],"%g %g %g",&value.x,&value.y,&value.z);
	}
	object->saveUltraframe(seq,frame,node,type,target,value);
}

ConsoleMethod( fxFlexBody, getUltraframe, const char*, 6, 6, "getUltraframe(int seq, int frame, int node, int type)") 
{
	int seq = strtol(argv[2]);
	int frame = strtol(argv[3]);
	int node = strtol(argv[4]);
	int type = strtol(argv[5]);
	char* buf = //Con::getReturnBuffer(100);
	Ogre::Vector3 pos;
	int target;
	object->getUltraframe(seq,frame,node,type,&target,&pos);
	//if (pos.length()) //Con::errorf("returning frame %d new value: %3.2f %3.2f %3.2f",frame,pos.x,pos.y,pos.z);
	sprintf(buf,"%3.2f %3.2f %3.2f",pos.x,pos.y,pos.z);
	return buf;
}

ConsoleMethod( fxFlexBody, clearUltraframe, void, 5, 5, "clearUltraframe(int seq, int frame, int node)") 
{
	int seq = strtol(argv[2]);
	int frame = strtol(argv[3]);
	int node = strtol(argv[4]);
	object->clearUltraframe(seq,frame,node);
	return;
}

ConsoleMethod( fxFlexBody, dropUltraframe, void, 5, 5, "dropUltraframe(int seq, int frame, int node)") 
{
	int seq = strtol(argv[2]);
	int frame = strtol(argv[3]);
	int node = strtol(argv[4]);
	object->dropUltraframe(seq,frame,node);
	return;
}

ConsoleMethod( fxFlexBody, saveUltraframes, void, 4, 5, "saveUltraframes(int seq,char *filename)")
{
	int seq = strtol(argv[2]);
	if ((seq>=0) && strlen(argv[3]))
	{
		bool append = false;
		if (argc>4)
			append = dAtob(argv[4]);
		object->saveUltraframes(seq,argv[3],append);
	}
	return;
}

//ConsoleMethod( fxFlexBody, loadUltraframes, void, 4, 4, "loadUltraframes(int seq,char *filename)")
//{
//	int seq = strtol(argv[2]);
//	if ((seq>=0) && strlen(argv[3]))
//		object->loadUltraframes(seq,argv[3]);
//	return;
//}

ConsoleMethod( fxFlexBody, loadUltraframes, void, 3, 3, "loadUltraframes(char *filename)")
{
	if (strlen(argv[2]))
		object->loadUltraframes(argv[2]);
	return;
}

ConsoleMethod( fxFlexBody, getNumUltraframes, int, 3, 3, "getNumUltraframes(int seq);")
{
	int seq = strtol(argv[2]);
	return object->getNumUltraframes(seq);
}


ConsoleMethod( fxFlexBody, convertAckToKork, void, 4, 4, "convertAckToKork(int ackID,int seq)")
{
	//First:  find my $kork character, from script, somehow, pass the ID to here.
	//Second: remember name of config file right here:  "ack2kork", in either ack or kork directory, wherever.
	//Third: pass the sequence as usual.
	//TSShape *other = ?;
	//object->convertSequence(TSShape *other, const char *cfg, int sequence,const char *shapepath);
	int ackID = strtol(argv[2]);
	int seq = strtol(argv[3]);
	object->convertAckToKork(ackID,seq);
	return;
}
ConsoleMethod( fxFlexBody, convertKorkDefault, void, 3, 3, "convertKorkDefault(int ackID)")
{
	//First:  find my $kork character, from script, somehow, pass the ID to here.
	//Second: remember name of config file right here:  "ack2kork", in either ack or kork directory, wherever.
	//Third: pass the sequence as usual.
	//TSShape *other = ?;
	//object->convertSequence(TSShape *other, const char *cfg, int sequence,const char *shapepath);
	int ackID = strtol(argv[2]);
	object->convertKorkDefault(ackID);
	return;
}

ConsoleMethod( fxFlexBody, getSampleRate, int, 2, 2, "getSampleRate()")
{
	return object->mImportSampleRate;
}

ConsoleMethod( fxFlexBody, setSampleRate, void, 3, 3, "setSampleRate(int rate)")
{
	int rate;
	rate = strtol(argv[2]);
	if (rate>0)
		object->mImportSampleRate = rate;
	else
		//Con::errorf("Set ShapeBase sample rate to an integer above zero.");
	return;
}

ConsoleMethod( fxFlexBody, isEcstasyFirstTime, bool, 2, 2, "isEcstasyFirstTime()")
{
	if (object->mBeenTweakerBot)
		return false;
	else 
		return true;
}


ConsoleMethod( fxFlexBody, setPlaylist, void, 3, 3, "setPlaylist()")
{
	int playlist_id = strtol(argv[2]);
	//Con::errorf("setting playlist id: %d",playlist_id);
	object->setPlaylist(playlist_id);

}

ConsoleMethod( fxFlexBody, setPlaylistName, void, 3, 3, "setPlaylistName()")
{
	//Con::errorf("setting playlist name: %d",argv[2]);
	object->mPlaylistName = argv[2];

	//here: get  id from database.
	SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(physManagerCommon::getPM())->getSQL();
	if (!sql) return;
	if (sql->OpenDatabase("EcstasyMotion.db"))
	{
		char id_query[512];
		int result;
		sqlite_resultset *resultSet;

		sprintf(id_query,"SELECT id FROM playlist WHERE name = \"%s\";",object->mPlaylistName);
		result = sql->ExecuteSQL(id_query);
		resultSet = sql->GetResultSet(result);

		if (resultSet->iNumRows == 1)
		{
			object->mPlaylistID = strtol(resultSet->vRows[0]->vColumnValues[0]);
			object->loadPlaylistById( object->mPlaylistID );
		}
		else //Con::errorf("Error - found %d playlist with name = %s",resultSet->iNumRows,object->mPlaylistName);

		//Con::printf("set new playlist: id %d name %s",object->mPlaylistID,object->mPlaylistName);
		sql->CloseDatabase();
		delete sql;
	}
}

ConsoleMethod( fxFlexBody, getPlaylistId, int, 2, 2, "getPlaylistId()")
{
	return object->mPlaylistID;
}

ConsoleMethod( fxFlexBody, getPlaylistName, const char *, 2, 2, "getPlaylistName()")
{
	return object->mPlaylistName;
}



ConsoleMethod( fxFlexBody, setPersona, void, 3, 3, "setPersona()")
{
	//Con::errorf("setting persona id: %d",strtol(argv[2]));
	object->mPersonaID = strtol(argv[2]);

	//here: get persona name from database.
	SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(physManagerCommon::getPM())->getSQL();
	if (!sql) return;
	if (sql->OpenDatabase("EcstasyMotion.db"))
	{
		char name_query[512];
		int result;
		sqlite_resultset *resultSet;

		sprintf(name_query,"SELECT name FROM persona WHERE id = %d;",object->mPersonaID);
		result = sql->ExecuteSQL(name_query);
		resultSet = sql->GetResultSet(result);
		if (resultSet->iNumRows == 1)
			object->mPersonaName = resultSet->vRows[0]->vColumnValues[0];
		else //Con::errorf("Error - found %d persona with id = %d",resultSet->iNumRows,object->mPersonaID);

		//Con::printf("set new persona: id %d name %s",object->mPersonaID,object->mPersonaName);
		sql->CloseDatabase();
		delete sql;
	}
}

ConsoleMethod( fxFlexBody, setPersonaName, void, 3, 3, "setPersonaName()")
{
	//Con::errorf("setting persona name: %d",argv[2]);
	object->mPersonaName = StringTable->insert( argv[2] );

	//here: get persona id from database.
	SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(physManagerCommon::getPM())->getSQL();
	if (!sql) return;
	if (sql->OpenDatabase("EcstasyMotion.db"))
	{
		char id_query[512];
		int result;
		sqlite_resultset *resultSet;

		sprintf(id_query,"SELECT id FROM persona WHERE name = \"%s\";",object->mPersonaName);
		result = sql->ExecuteSQL(id_query);
		resultSet = sql->GetResultSet(result);

		if (resultSet->iNumRows == 1)
			object->mPersonaID = strtol(resultSet->vRows[0]->vColumnValues[0]);
		else //Con::errorf("Error - found %d persona with name = %s",resultSet->iNumRows,object->mPersonaName);

		//Con::printf("set new persona: id %d name %s",object->mPersonaID,object->mPersonaName);
		sql->CloseDatabase();
		delete sql;
	}
}

ConsoleMethod( fxFlexBody, getPersonaId, int, 2, 2, "getPersonaId()")
{
	return object->mPersonaID;
}

ConsoleMethod( fxFlexBody, getPersonaName, const char *, 2, 2, "getPersonaName()")
{
	return object->getPersonaName();
}

ConsoleMethod( fxFlexBody, setActorName, void, 3, 3, "setActorName(const char*)")
{
	////Con::errorf("setting actor name: %s",argv[2]);
	
	object->mActorName = StringTable->insert( argv[2] );

	SQLiteObject *sql = new SQLiteObject();//dynamic_cast<nxPhysManager*>(physManagerCommon::getPM())->getSQL();
	if (!sql) return;
	if (sql->OpenDatabase("EcstasyMotion.db"))
	{
		char id_query[512],insert_query[512];
		int result;
		sqlite_resultset *resultSet;

		sprintf(id_query,"SELECT id FROM actor WHERE name = \"%s\";",object->mActorName);
		result = sql->ExecuteSQL(id_query);
		resultSet = sql->GetResultSet(result);
		if (resultSet->iNumRows == 1)
			object->mActorID = strtol(resultSet->vRows[0]->vColumnValues[0]);
		else if (resultSet->iNumRows == 0)
		{//Fix: when flexbodydata is moved into db, get an id for it too.
			sprintf(insert_query,"INSERT INTO actor (name,persona_id) VALUES ('%s',%d);",object->mActorName,object->mPersonaID);
			result = sql->ExecuteSQL(insert_query);
			result = sql->ExecuteSQL(id_query);
			resultSet = sql->GetResultSet(result);
			if (resultSet->iNumRows == 1)
				object->mActorID = strtol(resultSet->vRows[0]->vColumnValues[0]);
			else //Con::errorf("Failed to insert new actor %s",object->mActorName);
		}
		else //Con::errorf("Error - found %d actors with name = %s",resultSet->iNumRows,object->mActorName);

		sql->CloseDatabase();
		delete sql;
	}
    object->//setMaskBits(fxFlexBody::fxFlexBodyMountMask);
}

ConsoleMethod( fxFlexBody, getActorId, int, 2, 2, "getActorId()")
{
	return object->mActorID;
}

ConsoleMethod( fxFlexBody, getActorName, const char *, 2, 2, "getActorName()")
{
	char actorName[512];
	sprintf(actorName,"%s",object->mActorName);
	return actorName;
}

ConsoleMethod( fxFlexBody, getSkeletonName, const char *, 2, 2, "getSkeletonName()")
{
	if (strlen(object->mSkeletonName)>0)
		return object->mSkeletonName;

	if (object->mSkeletonID<=0)
		return "";

	SQLiteObject *sql = new SQLiteObject();
	if (!sql) return "";

	if (!sql->OpenDatabase("EcstasyMotion.db"))
	{
		delete sql;
		return "";
	}

	char name_query[512],resultname[255];
	int result;
	sqlite_resultset *resultSet;

	sprintf(name_query,"SELECT name FROM skeleton WHERE id = %d;",object->mSkeletonID);
	result = sql->ExecuteSQL(name_query);
	resultSet = sql->GetResultSet(result);
	sprintf(resultname,"%s",resultSet->vRows[0]->vColumnValues[0]);
	if (resultSet->iNumRows == 1)
	{
		sql->CloseDatabase();
		delete sql;
		return resultname;
	}
	else
	{
		sql->CloseDatabase();
		delete sql;
		return "";
	}
	//return "";
}

ConsoleMethod( fxFlexBody, setSkeletonName, void, 3, 3, "setSkeletonName(const char*)")
{
	//Con::errorf("setting Skeleton name: %s",argv[2]);
	object->mSkeletonName = StringTable->insert( argv[2] );

	//Now set skeleton id to match, and make sure it goes across network to client(s).
	
	SQLiteObject *sql = new SQLiteObject();
	if (!sql) return ;

	if (!sql->OpenDatabase("EcstasyMotion.db"))
	{
		delete sql;
		return;
	}

	char id_query[512],insert_query[512];
	int result;
	sqlite_resultset *resultSet;

	sprintf(id_query,"SELECT id FROM skeleton WHERE name = '%s';",object->mSkeletonName);
	result = sql->ExecuteSQL(id_query);
	resultSet = sql->GetResultSet(result);
	if (resultSet->iNumRows == 1)
	{
		object->mSkeletonID = strtol(resultSet->vRows[0]->vColumnValues[0]);

	} else {
		sprintf(insert_query,"INSERT (name) INTO skeleton VALUES ('%s');",object->mSkeletonName);
		result = sql->ExecuteSQL(insert_query);
		result = sql->ExecuteSQL(id_query);
		resultSet = sql->GetResultSet(result);
		if (resultSet->iNumRows == 1)
		{
			object->mSkeletonID = strtol(resultSet->vRows[0]->vColumnValues[0]);
		}
	}

	sql->CloseDatabase();
	delete sql;
	return;
	
}

ConsoleMethod( fxFlexBody, getSkeletonId, int, 2, 2, "getSkeletonId()")
{
	return object->mSkeletonID;
}

ConsoleMethod( fxFlexBody, setSkeletonId, void, 3, 3, "setSkeletonId(unsigned int)")
{
	object->mSkeletonID = strtol(argv[2]);

	SQLiteObject *sql = new SQLiteObject();
	if (!sql) return ;

	if (!sql->OpenDatabase("EcstasyMotion.db"))
	{
		delete sql;
		return;
	}

	char id_query[512],resultname[255];
	int result;
	sqlite_resultset *resultSet;

	sprintf(id_query,"SELECT name FROM skeleton WHERE id = %d;",object->mSkeletonID);
	result = sql->ExecuteSQL(id_query);
	resultSet = sql->GetResultSet(result);
	sprintf(resultname,"%s",resultSet->vRows[0]->vColumnValues[0]);
	if (resultSet->iNumRows == 1)
	{
		object->mSkeletonName = StringTable->insert(resultname);
	} else {
		//Con::errorf("Error: Trying to set skeleton id %d which isn't in the database!",object->mSkeletonID);
	}
    object->//setMaskBits(fxFlexBody::fxFlexBodyMountMask);
	sql->CloseDatabase();
	delete sql;
	return;
}

ConsoleMethod( fxFlexBody, setTweakerDone, void, 2, 2, "setTweakerDone();")
{
	//gTweakerOne = dynamic_cast<ShapeBase *>(Sim::findObject(argv[1]));
	object->mBeenTweakerBot = true;
	object->//setMaskBits(fxFlexBody::NoWarpMask);//Ecstasy:  stole "NoWarp" mask bit 
		//to avoid wasting another one for a one-time use.
}

ConsoleMethod( fxFlexBody, setMaskBit, void, 2, 2, "setMaskBit)")
{
    object->//setMaskBits(fxFlexBody::fxFlexBodyMountMask);
}

ConsoleMethod( fxFlexBody, setTarget, void, 3, 3, "setTarget(shapebase_id);")
{
	//gTweakerOne = dynamic_cast<ShapeBase *>(Sim::findObject(argv[1]));
	if (Sim::findObject(argv[2])) 
	{
		object->setTarget((fxFlexBody *) Sim::findObject(argv[2]));
	}
	else object->setTarget(NULL);
}
ConsoleMethod( fxFlexBody, getTarget, int, 2, 2, "getTarget();")
{
	//gTweakerOne = dynamic_cast<ShapeBase *>(Sim::findObject(argv[1]));
	if (object->mTarget)
		return object->mTarget->getId();
	else 
		return 0;
}

ConsoleMethod( fxFlexBody, getTargetPosition, const char *, 2, 2, "getTargetPosition()")
{
	char *returnBuffer = //Con::getReturnBuffer(256);
	if (object->mTarget)
	{
		//Ogre::Vector3 kTargetPos = object->mTarget->getPosition();
		Ogre::Vector3 kTargetPos = object->mTarget->mBodyParts[0]->mCurrPosition;
		kTargetPos.z = 0.0;

		sprintf(returnBuffer,256,"%f %f %f",kTargetPos.x,kTargetPos.y,kTargetPos.z);
	}
	return returnBuffer;
}

void fxFlexBody::finishFollowEvent()
{
	if (mFollowEvent)
	{
		if (mFollowEvent->next)
			mFollowEvent = mFollowEvent->next;
		else 
			mFollowEvent = NULL;
	}
}

ConsoleMethod( fxFlexBody, finishFollowEvent, void, 2, 2, "finishFollowEvent();")
{
	object->finishFollowEvent();
}

ConsoleMethod( fxFlexBody, setPosition, void, 3, 3, "setPosition(Ogre::Vector3 pos);")
{
	//gTweakerOne = dynamic_cast<ShapeBase *>(Sim::findObject(argv[1]));
	Ogre::Vector3 pos;
	sscanf(argv[2],"%g %g %g",&pos.x,&pos.y,&pos.z);
	object->setPosition(pos);
}


ConsoleMethod( fxFlexBody, renameSequence, void, 4, 4, "renameSequence(char *old_name, char *new_name);")
{
	object->renameSequence(argv[2],argv[3]);
}

ConsoleMethod( fxFlexBody, dropAllSequences, void, 2, 2, "dropAllSequences();")
{
	TSShape *kShape = object->getShapeInstance()->getShape();
	kShape->dropAllSequences();
}

ConsoleMethod( fxFlexBody, reloadSequences, void, 2, 2, "reloadSequences();")
{
	object->reloadSequences();
}

//WAIT!  I think I can skip this whole step, if I just add ultraframesets as needed in addUltraframe.  If you take sequence name instead of sequence number in 
//addUltraFrame

//void fxFlexBody::loadKeyframeSets()
//{  //HERE: we need to create ultraframesets if there are any active sequence morphs - meaning default (keyframeSet) for this actor and this scene, 
//   //or current (actorKeyframeSet) for this actor and this scene
//
//	int sequence_id=0,keyframe_set_id=0,scene_id=0;
//	scene_id = dynamic_cast<nxPhysManager*>(mPM)->mSceneId;
//	String sceneName = dynamic_cast<nxPhysManager*>(mPM)->mSceneName;
//	TSShape *kShape = getShapeInstance()->getShape();
//
//	//First, check for default sequence morph for this actor and this scene.
//	for (unsigned int i=0;i<kShape->sequences.size();i++)
//	{
//
//
//	}
//
//
//}

ConsoleMethod( fxFlexBody, loadKeyframeSets, void, 3, 3, "loadKeyframeSets(int scene_id);")
{
	object->loadKeyframeSets(strtol(argv[2]));
}

//Temp
ConsoleMethod( fxFlexBody, showChainParts, void, 2, 2, "showChainParts")
{
	for (unsigned int i=0;i<MAX_FLEX_CHAINS;i++)
	{
		if (object->mChainParts[i])
			//Con::printf("chainparts[%d]: %d",i,object->mChainParts[i]->mBoneIndex);
		else 
			//Con::printf("chainparts[%d]: NULL",i);
	}
}

ConsoleMethod( fxFlexBody, clearChainParts, void, 2, 2, "clearChainParts")
{
	for (unsigned int i=0;i<MAX_FLEX_CHAINS;i++)
	{
		object->mChainParts[i] = NULL;
	}
}

ConsoleMethod( fxFlexBody, showKinematicNodes, void, 2, 2, "showKinematicNodes")
{
	for (unsigned int i=0;i<object->mNumBodyParts;i++)
	{
		if (object->mBodyParts[i]->mIsKinematic)
			//Con::printf("[%d] : TRUE",i);
		else 
			//Con::printf("[%d] : FALSE",i);
	}
}

ConsoleMethod( fxFlexBody, getCurrentTick, int, 2, 2, "getCurrentTick")
{// This should be nxPhysManager current tick, not by flexbody or bodypart.
	return object->mCurrTick;
}

ConsoleMethod( fxFlexBody, getBeenHitTick, int, 2, 2, "getBeenHitTick")
{// This should be nxPhysManager current tick, not by flexbody or bodypart.
	return object->mBeenHitTick;
}

ConsoleMethod( fxFlexBody, zeroHeight, void, 2, 2, "zeroHeight")
{// This should be nxPhysManager current tick, not by flexbody or bodypart.
	
	Ogre::Vector3 kPos = object->getPosition();
	kPos.z = 0.0;
	object->setPosition(kPos);
	return;
}

ConsoleMethod( fxFlexBody, getPhysicsDamage, float, 2, 2, "getPhysicsDamage")
{// This should be nxPhysManager current tick, not by flexbody or bodypart.
	return object->mPhysicsDamage;
}

ConsoleMethod( fxFlexBody, setPhysicsDamage, void, 3, 3, "setPhysicsDamage(float damage)")
{// This should be nxPhysManager current tick, not by flexbody or bodypart.
	object->mPhysicsDamage = strtod(argv[2]);
	return;
}


ConsoleMethod( fxFlexBody, getLinearVelocity, const char *, 2, 2, "getLinearVelocity")
{// This should be nxPhysManager current tick, not by flexbody or bodypart.
	char* buff = //Con::getReturnBuffer(100);
	Ogre::Vector3 vel = object->mBodyParts[0]->mRB->getLinearVelocity();
	sprintf(buff,100,"%g %g %g",vel.x,vel.y,vel.z);
	return buff;
}


ConsoleMethod( fxFlexBody, hasRagdollBodyparts, bool, 2, 2, "hasRagdollBodyparts")
{
	return object->hasRagdollBodyparts();
}

///////////////////////////////////////////////////////

ConsoleFunction( castRayShape, int, 3, 3, "castRayShape(start,end);")
{
   RayInfo ri;
   bool hit = false;
	Ogre::Vector3 start,end;
	int myID;

	sscanf(argv[1],"%g %g %g",&start.x,&start.y,&start.z);
	sscanf(argv[2],"%g %g %g",&end.x,&end.y,&end.z);

	hit = gServerContainer.castRay(start, end, ShapeBaseObjectType, &ri);
	//hit = gServerContainer.castRay(start, end, 0, &ri);
	//Con::errorf("casting ray from %g %g %g to %g %g %g",start.x,start.y,start.z,end.x,end.y,end.z);
   if(hit)
   {
		SceneObject *myObject = ri.object;
		//Con::errorf("got a hit! %s",myObject->getClassName());
		myID = myObject->getId();
		return myID;
	} else {
		hit = gClientContainer.castRay(start, end, ShapeBaseObjectType, &ri);
		//hit = gClientContainer.castRay(start, end, StaticObjectType, &ri);
		
		if(hit)
		{
			SceneObject *myObject = ri.object;
			//Con::errorf("got a hit! %s",myObject->getClassName());
			myID = myObject->getId();
			return myID;
		}
	}
	return 0;
}


ConsoleFunction( setTweakerOne, void, 2, 2, "setTweakerOne(shapebase_id);")
{
	//gTweakerOne = dynamic_cast<ShapeBase *>(Sim::findObject(argv[1]));
	if (Sim::findObject(argv[1])) 
	{
		gTweakerOne = (fxFlexBody *) Sim::findObject(argv[1]);
	}
	else gTweakerOne = NULL;
}

ConsoleFunction( setTweakerTwo, void, 2, 2, "setTweakerTwo(shapebase_id);")
{
	//gTweakerTwo = dynamic_cast<ShapeBase *>(Sim::findObject(argv[1]));//strtol(argv[1])?
	if (Sim::findObject(argv[1])) gTweakerTwo = (fxFlexBody *) Sim::findObject(argv[1]);
	else gTweakerTwo = NULL;
}

ConsoleFunction( setArenaBot, void, 2, 2, "setArenaBot(shapebase_id);")
{
	//gTweakerTwo = dynamic_cast<ShapeBase *>(Sim::findObject(argv[1]));//strtol(argv[1])?
	if (Sim::findObject(argv[1])) 
	{
		gArenaBot = (fxFlexBody *) Sim::findObject(argv[1]);
		if (gArenaBot)
			(gArenaBot)->mIsStreaming = true;
	}
	else 
	{
		if (gArenaBot)
			(gArenaBot)->mIsStreaming = false;
		gArenaBot = NULL;
	}
}



ConsoleFunction(  startArenaStreaming, bool, 1, 3, "startArenaStreaming(char localIP,char sourceIP)")
{
	if (argc==3)
		return startArenaStreaming(argv[1],argv[2]);
	else
		return startArenaStreaming();
}

ConsoleFunction(  stopArenaStreaming, void, 1, 1, "stopArenaStreaming()")
{
	stopArenaStreaming();
}

ConsoleMethod(fxFlexBody,exportFBX,void,2,3,"exportFBX()")
{
	int seq = 0;
	if (argc==3)
		seq = strtol(argv[2]);

	object->exportFBX(seq);

	return;
}

ConsoleMethod(fxFlexBody,setVerbose,void,2,3,"setVerbose(bool)")
{
	if (argc>2)
		object->mVerbose = dAtob(argv[2]);
	else 
		object->mVerbose = true;
}

*/


/*
	///TEMP MATRIX TESTING ///////////////////

	Ogre::Matrix3 m1,m2,m3,mf,mn;
	Ogre::Vector3 p1,p2,pForward,pRight,pUp,row0,row1,row2;
	Ogre::Vector3 e1,e2,e3,ef;
	HMatrix	hMatrix;
	EulerAngles eulQ;

	hMatrix[0][0] = 1.0; hMatrix[1][0] = 0.0; hMatrix[2][0] = 0.0; hMatrix[3][0] = 0.0;
	hMatrix[0][1] = 0.0; hMatrix[1][1] = 1.0; hMatrix[2][1] = 0.0; hMatrix[3][1] = 0.0;
	hMatrix[0][2] = 0.0; hMatrix[1][2] = 0.0; hMatrix[2][2] = 1.0; hMatrix[3][2] = 0.0;
	hMatrix[0][3] = 0.0; hMatrix[1][3] = 0.0; hMatrix[2][3] = 0.0; hMatrix[3][3] = 1.0;

	//spine: XZY, lCollar: YZX, lShldr ZYX, lForearm YZX

	//lCollar -3.326983 -9.505730 -2.443470 YZX
	e1.set(0,mDegToRad(-3.326983),0); m1.set(e1);
	e2.set(0,0,mDegToRad(-9.505730)); m2.set(e2);
	e3.set(mDegToRad(-2.443470),0,0); m3.set(e3);

	mf.mul(m1,m2); mf.mul(m3);
	pUp = mf.getUpVector(); //Con::errorf("Up (Z): %f %f %f",pUp.x,pUp.y,pUp.z);
	ef = mf.toEuler(); ////Con::errorf("Torque Euler: %f %f %f",mRadToDeg(ef.x),mRadToDeg(e2.y),mRadToDeg(e2.z));

	mf.getRow(0,&row0); mf.getRow(1,&row1); mf.getRow(2,&row2);
	hMatrix[0][0] = row0.x; hMatrix[1][0] = row0.y; hMatrix[2][0] = row0.z; 
	hMatrix[0][1] = row1.x; hMatrix[1][1] = row1.y; hMatrix[2][1] = row1.z; 
	hMatrix[0][2] = row2.x; hMatrix[1][2] = row2.y; hMatrix[2][2] = row2.z; 

	eulQ = Eul_FromHMatrix( hMatrix,EulOrdYZXs);
	//Con::errorf("lCollar YZX Euler s: %f %f %f",mRadToDeg(eulQ.x),mRadToDeg(eulQ.y),mRadToDeg(eulQ.z));

	//lShldr -57.099701 4.886140 42.675098 ZYX
	e1.set(0,0,mDegToRad(-57.099701)); m1.set(e1);
	e2.set(0,mDegToRad(4.886140),0); m2.set(e2);
	e3.set(mDegToRad(42.675098),0,0); m3.set(e3);

	mf.mul(m1,m2); mf.mul(m3);
	pUp = mf.getUpVector(); //Con::errorf("Up (Z): %f %f %f",pUp.x,pUp.y,pUp.z);
	ef = mf.toEuler(); ////Con::errorf("Torque Euler: %f %f %f",mRadToDeg(ef.x),mRadToDeg(e2.y),mRadToDeg(e2.z));

	mf.getRow(0,&row0); mf.getRow(1,&row1); mf.getRow(2,&row2);
	hMatrix[0][0] = row0.x; hMatrix[1][0] = row0.y; hMatrix[2][0] = row0.z; 
	hMatrix[0][1] = row1.x; hMatrix[1][1] = row1.y; hMatrix[2][1] = row1.z; 
	hMatrix[0][2] = row2.x; hMatrix[1][2] = row2.y; hMatrix[2][2] = row2.z; 

	eulQ = Eul_FromHMatrix( hMatrix,EulOrdZYXs);
	//Con::errorf("lShldr ZYX Euler s: %f %f %f",mRadToDeg(eulQ.x),mRadToDeg(eulQ.y),mRadToDeg(eulQ.z));

	//lForearm -1.600510 -1.877220 -6.124470 YZX
	e1.set(0,mDegToRad(-1.600510),0); m1.set(e1);
	e2.set(0,0,mDegToRad(-1.877220)); m2.set(e2);
	e3.set(mDegToRad(-6.124470),0,0); m3.set(e3);

	mf.mul(m1,m2); mf.mul(m3);
	pUp = mf.getUpVector(); //Con::errorf("Up (Z): %f %f %f",pUp.x,pUp.y,pUp.z);
	ef = mf.toEuler(); ////Con::errorf("Torque Euler: %f %f %f",mRadToDeg(ef.x),mRadToDeg(e2.y),mRadToDeg(e2.z));

	mf.getRow(0,&row0); mf.getRow(1,&row1); mf.getRow(2,&row2);
	hMatrix[0][0] = row0.x; hMatrix[1][0] = row0.y; hMatrix[2][0] = row0.z; 
	hMatrix[0][1] = row1.x; hMatrix[1][1] = row1.y; hMatrix[2][1] = row1.z; 
	hMatrix[0][2] = row2.x; hMatrix[1][2] = row2.y; hMatrix[2][2] = row2.z; 

	eulQ = Eul_FromHMatrix( hMatrix,EulOrdYZXs);
	//Con::errorf("lForearm YZX Euler s: %f %f %f",mRadToDeg(eulQ.x),mRadToDeg(eulQ.y),mRadToDeg(eulQ.z));

	//lHand -2.833100 39.653198 -9.023950 ZYX
	e1.set(0,0,mDegToRad(-2.833100)); m1.set(e1);
	e2.set(0,mDegToRad(39.653198),0); m2.set(e2);
	e3.set(mDegToRad(-9.023950),0,0); m3.set(e3);

	mf.mul(m1,m2); mf.mul(m3);
	pUp = mf.getUpVector(); //Con::errorf("Up (Z): %f %f %f",pUp.x,pUp.y,pUp.z);
	ef = mf.toEuler(); ////Con::errorf("Torque Euler: %f %f %f",mRadToDeg(ef.x),mRadToDeg(e2.y),mRadToDeg(e2.z));

	mf.getRow(0,&row0); mf.getRow(1,&row1); mf.getRow(2,&row2);
	hMatrix[0][0] = row0.x; hMatrix[1][0] = row0.y; hMatrix[2][0] = row0.z; 
	hMatrix[0][1] = row1.x; hMatrix[1][1] = row1.y; hMatrix[2][1] = row1.z; 
	hMatrix[0][2] = row2.x; hMatrix[1][2] = row2.y; hMatrix[2][2] = row2.z; 

	eulQ = Eul_FromHMatrix( hMatrix,EulOrdZYXs);
	//Con::errorf("lHand ZYX Euler s: %f %f %f",mRadToDeg(eulQ.x),mRadToDeg(eulQ.y),mRadToDeg(eulQ.z));


	//mn.set(Ogre::Vector3(eulQ.x,eulQ.y,eulQ.z));
	//pUp = mn.getUpVector();
	////Con::errorf("New Up (Z): %f %f %f",pUp.x,pUp.y,pUp.z);
	
	//eulQ = Eul_FromHMatrix( hMatrix,EulOrdZYXr);
	////Con::errorf("ZYX Euler r: %f %f %f",mRadToDeg(eulQ.x),mRadToDeg(eulQ.y),mRadToDeg(eulQ.z));
	//eulQ = Eul_FromHMatrix( hMatrix,EulOrdXYZr);
	////Con::errorf("XYZ Euler r: %f %f %f",mRadToDeg(eulQ.x),mRadToDeg(eulQ.y),mRadToDeg(eulQ.z));
	//eulQ = Eul_FromHMatrix( hMatrix,EulOrdXYZs);
	////Con::errorf("XYZ Euler s: %f %f %f",mRadToDeg(eulQ.x),mRadToDeg(eulQ.y),mRadToDeg(eulQ.z));

	//pForward = mf.getForwardVector();
	//pRight = mf.getRightVector();
	////Con::errorf("Forward (Y): %f %f %f",pForward.x,pForward.y,pForward.z);
	////Con::errorf("Right (X): %f %f %f",pRight.x,pRight.y,pRight.z);
	///END TEMP///////////////////
*/


//////////////////////////////////////////////
/*
void fxFlexBody::exportFBX(int seq)
{
	KFbxSdkManager *pSdkManager = KFbxSdkManager::Create();

	if (!pSdkManager)
	{
		printf("Unable to create the FBX SDK manager\n");
		exit(0);
	}

	// create an IOSettings object
	KFbxIOSettings * ios = KFbxIOSettings::Create(pSdkManager, IOSROOT );
	pSdkManager->SetIOSettings(ios);

	
    // Create an exporter.
    KFbxExporter* lExporter = KFbxExporter::Create(pSdkManager, "");


	// Load plugins from the executable directory
	KString lPath = KFbxGetApplicationDirectory();
#if defined(KARCH_ENV_WIN)
	KString lExtension = "dll";
#elif defined(KARCH_ENV_MACOSX)
	KString lExtension = "dylib";
#elif defined(KARCH_ENV_LINUX)
	KString lExtension = "so";
#endif
	pSdkManager->LoadPluginsDirectory(lPath.Buffer(), lExtension.Buffer());

	// Create the entity that will hold the scene.
	KFbxScene* pScene = NULL;
	pScene = KFbxScene::Create(pSdkManager,"");

	if (pScene)//Con::printf("Made an fbx scene!!! character count %d",pScene->GetCharacterCount());

	lExporter->Destroy();
	pSdkManager->Destroy();

}
*/









////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
/////  FBX  ////////////////////////////////////////////////////////


/*
void AddNodeRecursively(KArrayTemplate<KFbxNode*>& pNodeArray, KFbxNode* pNode)
{
    if (pNode)
    {
        AddNodeRecursively(pNodeArray, pNode->GetParent());

        if (pNodeArray.Find(pNode) == -1)
        {
            // Node not in the list, add it
            pNodeArray.Add(pNode);
        }
    }
}

// Create a cylinder centered on the Z axis. 
KFbxNode* CreatePatch(KFbxSdkManager* pSdkManager, char* pName, fxFlexBody *kFB)
{
    KFbxPatch* lPatch = KFbxPatch::Create(pSdkManager,pName);

    // Set patch properties.
    lPatch->InitControlPoints(4, KFbxPatch::eBSPLINE, 7, KFbxPatch::eBSPLINE);
    lPatch->SetStep(4, 4);
    lPatch->SetClosed(true, false);

    KFbxVector4* lVector4 = lPatch->GetControlPoints();
    int i;

    for (i = 0; i < 7; i++)
    {
        double lRadius = 15.0;
        double lSegmentLength = 20.0;
        lVector4[4*i + 0].Set(lRadius, 0.0, (i-3)*lSegmentLength);
        lVector4[4*i + 1].Set(0.0, -lRadius, (i-3)*lSegmentLength);
        lVector4[4*i + 2].Set(-lRadius, 0.0, (i-3)*lSegmentLength);
        lVector4[4*i + 3].Set(0.0, lRadius, (i-3)*lSegmentLength);
    }

    KFbxNode* lNode = KFbxNode::Create(pSdkManager,pName);

    // Rotate the cylinder along the X axis so the axis
    // of the cylinder is the same as the bone axis (Y axis)
    KFbxVector4 lR(-90.0, 0.0, 0.0);
    lNode->SetDefaultR(lR);

    lNode->SetNodeAttribute(lPatch);

    return lNode;
}

KFbxNode *gSkeletonLimbNodes[MAX_FLEX_NODES];//TEMP, just hit an awkward point where I needed this array
//to animate the skeleton, but didn't want to deal with it properly.

	//
	//for (unsigned int i=0;i<rot_matters_count;i++)
	///
	//for (unsigned int i=0;i<5;i++)
	//{
	//	currentTime = 0.0;

	//	lLimbNode->CreateTakeNode(lTakeName.Buffer());
	//	lLimbNode->SetCurrentTakeNode(lTakeName.Buffer());
	//	lLimbNode->LclRotation.GetKFCurveNode(true, lTakeName.Buffer());

	//	//X Curve
	//	lCurves_X[i] = lLimbNode->LclRotation.GetKFCurve(KFCURVENODE_R_X, lTakeName.Buffer());
	//	lCurves_X[i]->KeyModifyBegin();
	//	for (unsigned int j=0;j<kSeq->numKeyframes;j++)
	//	{
	//		q16 = kShape->nodeRotations[start_rot + (i * num_keyframes) + j];

	//		q16.getOgre::Quaternion(&q);
	//		q.setMatrix(&mat);
	//		eul = mat.toEuler();

	//		fX = -eul.y;//It doesn't make any sense that I can figure out,
	//		fY = eul.x;// but this is what worked for bvh...
	//		fZ = -eul.z;

	//		lTime.SetSecondDouble(currentTime);

	//		lKeyIndex = lCurves_X[i]->KeyAdd(lTime);
	//		lCurves_X[i]->KeySetValue(lKeyIndex, fX);
	//		lCurves_X[i]->KeySetInterpolation(lKeyIndex, KFCURVE_INTERPOLATION_CUBIC);

	//		currentTime += frameTime;
	//	}
	//	lCurves_X[i]->KeyModifyEnd();

	//	//Y Curve
	//	currentTime = 0.0;
	//	//lLimbNode->LclRotation.GetKFCurveNode(true, lTakeName.Buffer());
	//	lCurves_Y[i] = lLimbNode->LclRotation.GetKFCurve(KFCURVENODE_R_Y, lTakeName.Buffer());
	//	lCurves_Y[i]->KeyModifyBegin();
	//	for (unsigned int j=0;j<kSeq->numKeyframes;j++)
	//	{
	//		q16 = kShape->nodeRotations[start_rot + (i * num_keyframes) + j];

	//		q16.getOgre::Quaternion(&q);
	//		q.setMatrix(&mat);
	//		eul = mat.toEuler();

	//		fX = -eul.x;
	//		fY = eul.z;
	//		fZ = eul.y;
	//		//fX = -eul.y;//It doesn't make any sense that I can figure out,
	//		//fY = eul.x;// but this is what worked for bvh...
	//		//fZ = -eul.z;

	//		lTime.SetSecondDouble(currentTime);

	//		lKeyIndex = lCurves_Y[i]->KeyAdd(lTime);
	//		lCurves_Y[i]->KeySetValue(lKeyIndex, fY);
	//		lCurves_Y[i]->KeySetInterpolation(lKeyIndex, KFCURVE_INTERPOLATION_CUBIC);

	//		currentTime += frameTime;
	//	}
	//	lCurves_Y[i]->KeyModifyEnd();

	//	//Z Curve
	//	currentTime = 0.0;
	//	//lLimbNode->LclRotation.GetKFCurveNode(true, lTakeName.Buffer());
	//	lCurves_Z[i] = lLimbNode->LclRotation.GetKFCurve(KFCURVENODE_R_Z, lTakeName.Buffer());
	//	lCurves_Z[i]->KeyModifyBegin();
	//	for (unsigned int j=0;j<kSeq->numKeyframes;j++)
	//	{
	//		q16 = kShape->nodeRotations[start_rot + (i * num_keyframes) + j];

	//		q16.getOgre::Quaternion(&q);
	//		q.setMatrix(&mat);
	//		eul = mat.toEuler();

	//		fX = -eul.y;//It doesn't make any sense that I can figure out,
	//		fY = eul.x;// but this is what worked for bvh...
	//		fZ = -eul.z;

	//		lTime.SetSecondDouble(currentTime);

	//		lKeyIndex = lCurves_Z[i]->KeyAdd(lTime);
	//		lCurves_Z[i]->KeySetValue(lKeyIndex, fZ);
	//		lCurves_Z[i]->KeySetInterpolation(lKeyIndex, KFCURVE_INTERPOLATION_CUBIC);

	//		currentTime += frameTime;
	//	}
	//	lCurves_Z[i]->KeyModifyEnd();

	//	lLimbNode = lLimbNode->GetChild(0);//HERE: need to cycle through the array, NOT by grabbing
	//	//first child every time, this is a biped not a snake.
	//}
	//


// Set the influence of the skeleton segments over the cylinder.
// The link mode is KFbxLink::eTOTAL1 which means the total
// of the weights assigned to a given control point must equal 1.
void LinkPatchToSkeleton(KFbxSdkManager* pSdkManager, KFbxNode* pPatch, KFbxNode* pSkeletonRoot,fxFlexBody *kFB)
{
    int i, j;
    KFbxXMatrix lXMatrix;

    KFbxNode* lRoot = pSkeletonRoot;
    KFbxNode* lLimbNode1 = pSkeletonRoot->GetChild(0);
    KFbxNode* lLimbNode2 = lLimbNode1->GetChild(0);
	//HERE: need a loop through whole body.

    // Bottom section of cylinder is clustered to skeleton root.
    KFbxCluster *lClusterToRoot = KFbxCluster::Create(pSdkManager,"");
    lClusterToRoot->SetLink(lRoot);
    lClusterToRoot->SetLinkMode(KFbxCluster::eTOTAL1);
    for(i=0; i<4; ++i)
        for(j=0; j<4; ++j)
            lClusterToRoot->AddControlPointIndex(4*i + j, 1.0 - 0.25*i);

    // Center section of cylinder is clustered to skeleton limb node.
    KFbxCluster* lClusterToLimbNode1 = KFbxCluster::Create(pSdkManager, "");
    lClusterToLimbNode1->SetLink(lLimbNode1);
    lClusterToLimbNode1->SetLinkMode(KFbxCluster::eTOTAL1);

    for (i =1; i<6; ++i)
        for (j=0; j<4; ++j)
            lClusterToLimbNode1->AddControlPointIndex(4*i + j, (i == 1 || i == 5 ? 0.25 : 0.50));


    // Top section of cylinder is clustered to skeleton limb.

    KFbxCluster * lClusterToLimbNode2 = KFbxCluster::Create(pSdkManager,"");
    lClusterToLimbNode2->SetLink(lLimbNode2);
    lClusterToLimbNode2->SetLinkMode(KFbxCluster::eTOTAL1);

    for (i=3; i<7; ++i)
        for (j=0; j<4; ++j)
            lClusterToLimbNode2->AddControlPointIndex(4*i + j, 0.25*(i - 2));

    // Now we have the Patch and the skeleton correctly positioned,
    // set the Transform and TransformLink matrix accordingly.
    lXMatrix = pPatch->GetGlobalFromDefault(KFbxNode::eSOURCE_SET);

    lClusterToRoot->SetTransformMatrix(lXMatrix);
    lClusterToLimbNode1->SetTransformMatrix(lXMatrix);
    lClusterToLimbNode2->SetTransformMatrix(lXMatrix);



    lXMatrix = lRoot->GetGlobalFromDefault(KFbxNode::eSOURCE_SET);
    lClusterToRoot->SetTransformLinkMatrix(lXMatrix);


    lXMatrix = lLimbNode1->GetGlobalFromDefault(KFbxNode::eSOURCE_SET);
    lClusterToLimbNode1->SetTransformLinkMatrix(lXMatrix);


    lXMatrix = lLimbNode2->GetGlobalFromDefault(KFbxNode::eSOURCE_SET);
    lClusterToLimbNode2->SetTransformLinkMatrix(lXMatrix);


    // Add the clusters to the patch by creating a skin and adding those clusters to that skin.
    // After add that skin.

    KFbxGeometry* lPatchAttribute = (KFbxGeometry*) pPatch->GetNodeAttribute();
    KFbxSkin* lSkin = KFbxSkin::Create(pSdkManager, "");
    lSkin->AddCluster(lClusterToRoot);
    lSkin->AddCluster(lClusterToLimbNode1);
    lSkin->AddCluster(lClusterToLimbNode2);
    lPatchAttribute->AddDeformer(lSkin);

}




// Store the Bind Pose
void StoreBindPose(KFbxSdkManager* pSdkManager, KFbxScene* pScene, KFbxNode* pPatch, KFbxNode* pSkeletonRoot,fxFlexBody *kFB)
{
    // In the bind pose, we must store all the link's global matrix at the time of the bind.
    // Plus, we must store all the parent(s) global matrix of a link, even if they are not
    // themselves deforming any model.


    // Now list the all the link involve in the patch deformation
    KArrayTemplate<KFbxNode*> lClusteredFbxNodes;
    int                       i, j;

    if (pPatch && pPatch->GetNodeAttribute())
    {
        int lSkinCount=0;
        int lClusterCount=0;
        switch (pPatch->GetNodeAttribute()->GetAttributeType())
        {
        case KFbxNodeAttribute::eMESH:
        case KFbxNodeAttribute::eNURB:
        case KFbxNodeAttribute::ePATCH:

            lSkinCount = ((KFbxGeometry*)pPatch->GetNodeAttribute())->GetDeformerCount(KFbxDeformer::eSKIN);
            //Go through all the skins and count them
            //then go through each skin and get their cluster count
            for(i=0; i<lSkinCount; ++i)
            {
                KFbxSkin *lSkin=(KFbxSkin*)((KFbxGeometry*)pPatch->GetNodeAttribute())->GetDeformer(i, KFbxDeformer::eSKIN);
                lClusterCount+=lSkin->GetClusterCount();
            }
            break;
        }
        //if we found some clusters we must add the node
        if (lClusterCount)
        {
            //Again, go through all the skins get each cluster link and add them
            for (i=0; i<lSkinCount; ++i)
            {
                KFbxSkin *lSkin=(KFbxSkin*)((KFbxGeometry*)pPatch->GetNodeAttribute())->GetDeformer(i, KFbxDeformer::eSKIN);
                lClusterCount=lSkin->GetClusterCount();
                for (j=0; j<lClusterCount; ++j)
                {
                    KFbxNode* lClusterNode = lSkin->GetCluster(j)->GetLink();
                    AddNodeRecursively(lClusteredFbxNodes, lClusterNode);
                }

            }

            // Add the patch to the pose
            lClusteredFbxNodes.Add(pPatch);
        }
    }

    // Now create a bind pose with the link list
    if (lClusteredFbxNodes.GetCount())
    {
        // A pose must be named. Arbitrarily use the name of the patch node.
        KFbxPose* lPose = KFbxPose::Create(pSdkManager,pPatch->GetName());

        for (i=0; i<lClusteredFbxNodes.GetCount(); i++)
        {
            KFbxNode*  lKFbxNode   = lClusteredFbxNodes.GetAt(i);
            KFbxMatrix lBindMatrix = lKFbxNode->GetGlobalFromDefault(KFbxNode::eSOURCE_SET);

            lPose->Add(lKFbxNode, lBindMatrix);
        }

        // Add the pose to the scene
        pScene->AddPose(lPose);
    }
}

// Store a Rest Pose
void StoreRestPose(KFbxSdkManager* pSdkManager, KFbxScene* pScene, KFbxNode* pSkeletonRoot,fxFlexBody *kFB)
{
    // This example show an arbitrary rest pose assignment.
    // This rest pose will set the bone rotation to the same value 
    // as time 1 second in the first take of animation, but the 
    // position of the bone will be set elsewhere in the scene.
    KString     lNodeName;
    KFbxNode*   lKFbxNode;
    KFbxMatrix  lTransformMatrix;
    KFbxVector4 lT,lR,lS(1.0, 1.0, 1.0);

    // Create the rest pose
    KFbxPose* lPose = KFbxPose::Create(pSdkManager,"A Bind Pose");

    // Set the skeleton root node to the global position (10, 10, 10)
    // and global rotation of 45deg along the Z axis.
    lT.Set(10.0, 10.0, 10.0);
    lR.Set( 0.0,  0.0, 45.0);

    lTransformMatrix.SetTRS(lT, lR, lS);

    // Add the skeleton root node to the pose
    lKFbxNode = pSkeletonRoot;
    lPose->Add(lKFbxNode, lTransformMatrix, false );

    // Set the lLimbNode1 node to the local position of (0, 40, 0)
    // and local rotation of -90deg along the Z axis. This show that
    // you can mix local and global coordinates in a rest pose.
    lT.Set(0.0, 40.0,   0.0);
    lR.Set(0.0,  0.0, -90.0);

    lTransformMatrix.SetTRS(lT, lR, lS);

    // Add the skeleton second node to the pose
    lKFbxNode = lKFbxNode->GetChild(0);
    lPose->Add(lKFbxNode, lTransformMatrix, true );

    // Set the lLimbNode2 node to the local position of (0, 40, 0)
    // and local rotation of 45deg along the Z axis.
    lT.Set(0.0, 40.0, 0.0);
    lR.Set(0.0,  0.0, 45.0);

    lTransformMatrix.SetTRS(lT, lR, lS);

    // Add the skeleton second node to the pose
    lKFbxNode = lKFbxNode->GetChild(0);
    lNodeName = lKFbxNode->GetName();
    lPose->Add(lKFbxNode, lTransformMatrix, true);

    // Now add the pose to the scene
    pScene->AddPose(lPose);
}
*/



/*
KFbxNode* fxFlexBody::createFbxMesh( KFbxScene* pScene, const char* pName)
{	
	KFbxNode* kRootNode = pScene->GetRootNode();
	KFbxNode* kMeshRoot = KFbxNode::Create(pScene, pName);
	KFbxMesh* kMesh = KFbxMesh::Create(pScene, "Mesh");

	kRootNode->AddChild( kMeshRoot );
	kMeshRoot->SetNodeAttribute( kMesh );
	kMeshRoot->SetShadingMode(KFbxNode::eWIRE_FRAME);//eTEXTURE_SHADING
	kMeshRoot->LclScaling.Set(KFbxVector4(39.0, 39.0, 39.0));

	KFbxVector4 lControlPoint0(-1, 0, 1);
	KFbxVector4 lControlPoint1(1, 0, 1);
	KFbxVector4 lControlPoint2(1, 2, 1);
	KFbxVector4 lControlPoint3(-1, 2, 1);
	KFbxVector4 lControlPoint4(-1, 0, -1);
	KFbxVector4 lControlPoint5(1, 0, -1);
	KFbxVector4 lControlPoint6(1, 2, -1);
	KFbxVector4 lControlPoint7(-1, 2, -1);

	KFbxVector4 lNormalXPos(1, 0, 0);
	KFbxVector4 lNormalXNeg(-1, 0, 0);
	KFbxVector4 lNormalYPos(0, 1, 0);
	KFbxVector4 lNormalYNeg(0, -1, 0);
	KFbxVector4 lNormalZPos(0, 0, 1);
	KFbxVector4 lNormalZNeg(0, 0, -1);

	// Create control points.
	kMesh->InitControlPoints(24);

	KFbxVector4* lControlPoints = kMesh->GetControlPoints();

	lControlPoints[0] = lControlPoint0;
	lControlPoints[1] = lControlPoint1;
	lControlPoints[2] = lControlPoint2;
	lControlPoints[3] = lControlPoint3;
	lControlPoints[4] = lControlPoint1;
	lControlPoints[5] = lControlPoint5;
	lControlPoints[6] = lControlPoint6;
	lControlPoints[7] = lControlPoint2;
	lControlPoints[8] = lControlPoint5;
	lControlPoints[9] = lControlPoint4;
	lControlPoints[10] = lControlPoint7;
	lControlPoints[11] = lControlPoint6;
	lControlPoints[12] = lControlPoint4;
	lControlPoints[13] = lControlPoint0;
	lControlPoints[14] = lControlPoint3;
	lControlPoints[15] = lControlPoint7;
	lControlPoints[16] = lControlPoint3;
	lControlPoints[17] = lControlPoint2;
	lControlPoints[18] = lControlPoint6;
	lControlPoints[19] = lControlPoint7;
	lControlPoints[20] = lControlPoint1;
	lControlPoints[21] = lControlPoint0;
	lControlPoints[22] = lControlPoint4;
	lControlPoints[23] = lControlPoint5;

	// Set the normals on Layer 0.
	KFbxLayer* kLayer = kMesh->GetLayer(0);
	if (kLayer == NULL)
	{
		kMesh->CreateLayer();
		kLayer = kMesh->GetLayer(0);
	}


	// We want to have one normal for each control point,
	// so we set the mapping mode to eBY_CONTROL_POINT.
	KFbxLayerElementNormal* lLayerElementNormal= KFbxLayerElementNormal::Create(kMesh, "");
	lLayerElementNormal->SetMappingMode(KFbxLayerElement::eBY_CONTROL_POINT);
	// Set the normal values for every control point.
	lLayerElementNormal->SetReferenceMode(KFbxLayerElement::eDIRECT);


	lLayerElementNormal->GetDirectArray().Add(lNormalZPos);
	lLayerElementNormal->GetDirectArray().Add(lNormalZPos);
	lLayerElementNormal->GetDirectArray().Add(lNormalZPos);
	lLayerElementNormal->GetDirectArray().Add(lNormalZPos);
	lLayerElementNormal->GetDirectArray().Add(lNormalXPos);
	lLayerElementNormal->GetDirectArray().Add(lNormalXPos);
	lLayerElementNormal->GetDirectArray().Add(lNormalXPos);
	lLayerElementNormal->GetDirectArray().Add(lNormalXPos);
	lLayerElementNormal->GetDirectArray().Add(lNormalZNeg);
	lLayerElementNormal->GetDirectArray().Add(lNormalZNeg);
	lLayerElementNormal->GetDirectArray().Add(lNormalZNeg);
	lLayerElementNormal->GetDirectArray().Add(lNormalZNeg);
	lLayerElementNormal->GetDirectArray().Add(lNormalXNeg);
	lLayerElementNormal->GetDirectArray().Add(lNormalXNeg);
	lLayerElementNormal->GetDirectArray().Add(lNormalXNeg);
	lLayerElementNormal->GetDirectArray().Add(lNormalXNeg);
	lLayerElementNormal->GetDirectArray().Add(lNormalYPos);
	lLayerElementNormal->GetDirectArray().Add(lNormalYPos);
	lLayerElementNormal->GetDirectArray().Add(lNormalYPos);
	lLayerElementNormal->GetDirectArray().Add(lNormalYPos);
	lLayerElementNormal->GetDirectArray().Add(lNormalYNeg);
	lLayerElementNormal->GetDirectArray().Add(lNormalYNeg);
	lLayerElementNormal->GetDirectArray().Add(lNormalYNeg);
	lLayerElementNormal->GetDirectArray().Add(lNormalYNeg);

	kLayer->SetNormals(lLayerElementNormal);


	return kMeshRoot;
}

// Create texture for cube.
void CreateTexture(KFbxScene* pScene, KFbxMesh* pMesh)
{
    // A texture need to be connected to a property on the material,
    // so let's use the material (if it exists) or create a new one
    KFbxSurfacePhong* lMaterial = NULL;

    //get the node of mesh, add material for it.
    KFbxNode* lNode = pMesh->GetNode();
    if(lNode)
    {
        lMaterial = lNode->GetSrcObject<KFbxSurfacePhong>(0);
        if (lMaterial == NULL)
        {
            KString lMaterialName = "toto";
            KString lShadingName  = "Phong";
            fbxDouble3 lBlack(0.0, 0.0, 0.0);
            fbxDouble3 lRed(1.0, 0.0, 0.0);
            fbxDouble3 lDiffuseColor(0.75, 0.75, 0.0);
            lMaterial = KFbxSurfacePhong::Create(pScene, lMaterialName.Buffer());

            // Generate primary and secondary colors.
            lMaterial->GetEmissiveColor()       .Set(lBlack);
            lMaterial->GetAmbientColor()        .Set(lRed);
            lMaterial->GetAmbientFactor()       .Set(1.);
            // Add texture for diffuse channel
            lMaterial->GetDiffuseColor()        .Set(lDiffuseColor);
            lMaterial->GetDiffuseFactor()       .Set(1.);
            lMaterial->GetTransparencyFactor()  .Set(40);
            lMaterial->GetShadingModel()        .Set(lShadingName);
            lMaterial->GetShininess()           .Set(0.5);
            lMaterial->GetSpecularColor()       .Set(lBlack);
            lMaterial->GetSpecularFactor()      .Set(0.3);

            lNode->AddMaterial(lMaterial);
        }
    }

    KFbxTexture* lTexture = KFbxTexture::Create(pScene,"Diffuse Texture");

    // Set texture properties.
    lTexture->SetFileName("scene03.jpg"); // Resource file is in current directory.
    lTexture->SetTextureUse(KFbxTexture::eSTANDARD);
    lTexture->SetMappingType(KFbxTexture::eUV);
    lTexture->SetMaterialUse(KFbxTexture::eMODEL_MATERIAL);
    lTexture->SetSwapUV(false);
    lTexture->SetTranslation(0.0, 0.0);
    lTexture->SetScale(1.0, 1.0);
    lTexture->SetRotation(0.0, 0.0);

    // don't forget to connect the texture to the corresponding property of the material
    if (lMaterial)
        lMaterial->GetDiffuseColor().ConnectSrcObject(lTexture);

    lTexture = KFbxTexture::Create(pScene,"Ambient Texture");

    // Set texture properties.
    lTexture->SetFileName("gradient.jpg"); // Resource file is in current directory.
    lTexture->SetTextureUse(KFbxTexture::eSTANDARD);
    lTexture->SetMappingType(KFbxTexture::eUV);
    lTexture->SetMaterialUse(KFbxTexture::eMODEL_MATERIAL);
    lTexture->SetSwapUV(false);
    lTexture->SetTranslation(0.0, 0.0);
    lTexture->SetScale(1.0, 1.0);
    lTexture->SetRotation(0.0, 0.0);

    // don't forget to connect the texture to the corresponding property of the material
    if (lMaterial)
        lMaterial->GetAmbientColor().ConnectSrcObject(lTexture);

    lTexture = KFbxTexture::Create(pScene,"Emissive Texture");

    // Set texture properties.
    lTexture->SetFileName("spotty.jpg"); // Resource file is in current directory.
    lTexture->SetTextureUse(KFbxTexture::eSTANDARD);
    lTexture->SetMappingType(KFbxTexture::eUV);
    lTexture->SetMaterialUse(KFbxTexture::eMODEL_MATERIAL);
    lTexture->SetSwapUV(false);
    lTexture->SetTranslation(0.0, 0.0);
    lTexture->SetScale(1.0, 1.0);
    lTexture->SetRotation(0.0, 0.0);

    // don't forget to connect the texture to the corresponding property of the material
    if (lMaterial)
        lMaterial->GetEmissiveColor().ConnectSrcObject(lTexture);
}*/



//// Create texture for cube.
//void CreateTexture(KFbxScene* pScene, KFbxMesh* pMesh)
//{
//    // A texture need to be connected to a property on the material,
//    // so let's use the material (if it exists) or create a new one
//    KFbxSurfacePhong* lMaterial = NULL;
//
//    //get the node of mesh, add material for it.
//    KFbxNode* lNode = pMesh->GetNode();
//    if(lNode)
//    {
//        lMaterial = NULL;//lNode->GetSrcObject<KFbxSurfacePhong>(0);//linker error!  ClassId
//        if (lMaterial == NULL)
//        {
//            KString lMaterialName = "toto";
//            KString lShadingName  = "Phong";
//            fbxDouble3 lBlack(0.0, 0.0, 0.0);
//            fbxDouble3 lRed(1.0, 0.0, 0.0);
//            fbxDouble3 lDiffuseColor(0.75, 0.75, 0.0);
//            lMaterial = KFbxSurfacePhong::Create(pScene, lMaterialName.Buffer());
//
//            // Generate primary and secondary colors.
//            lMaterial->GetEmissiveColor()       .Set(lBlack);
//            lMaterial->GetAmbientColor()        .Set(lRed);
//            lMaterial->GetAmbientFactor()       .Set(1.);
//            // Add texture for diffuse channel
//            lMaterial->GetDiffuseColor()        .Set(lDiffuseColor);
//            lMaterial->GetDiffuseFactor()       .Set(1.);
//            lMaterial->GetTransparencyFactor()  .Set(40);
//            lMaterial->GetShadingModel()        .Set(lShadingName);
//            lMaterial->GetShininess()           .Set(0.5);
//            lMaterial->GetSpecularColor()       .Set(lBlack);
//            lMaterial->GetSpecularFactor()      .Set(0.3);
//
//            lNode->AddMaterial(lMaterial);
//        }
//    }
//
//    KFbxTexture* lTexture = KFbxTexture::Create(pScene,"Diffuse Texture");
//
//    // Set texture properties.
//    lTexture->SetFileName("scene03.jpg"); // Resource file is in current directory.
//    lTexture->SetTextureUse(KFbxTexture::eSTANDARD);
//    lTexture->SetMappingType(KFbxTexture::eUV);
//    lTexture->SetMaterialUse(KFbxTexture::eMODEL_MATERIAL);
//    lTexture->SetSwapUV(false);
//    lTexture->SetTranslation(0.0, 0.0);
//    lTexture->SetScale(1.0, 1.0);
//    lTexture->SetRotation(0.0, 0.0);
//
//    // don't forget to connect the texture to the corresponding property of the material
//    if (lMaterial)
//        lMaterial->GetDiffuseColor().ConnectSrcObject(lTexture);
//
//    lTexture = KFbxTexture::Create(pScene,"Ambient Texture");
//
//    // Set texture properties.
//    lTexture->SetFileName("gradient.jpg"); // Resource file is in current directory.
//    lTexture->SetTextureUse(KFbxTexture::eSTANDARD);
//    lTexture->SetMappingType(KFbxTexture::eUV);
//    lTexture->SetMaterialUse(KFbxTexture::eMODEL_MATERIAL);
//    lTexture->SetSwapUV(false);
//    lTexture->SetTranslation(0.0, 0.0);
//    lTexture->SetScale(1.0, 1.0);
//    lTexture->SetRotation(0.0, 0.0);
//
//    // don't forget to connect the texture to the corresponding property of the material
//    if (lMaterial)
//        lMaterial->GetAmbientColor().ConnectSrcObject(lTexture);
//
//    lTexture = KFbxTexture::Create(pScene,"Emissive Texture");
//
//    // Set texture properties.
//    lTexture->SetFileName("spotty.jpg"); // Resource file is in current directory.
//    lTexture->SetTextureUse(KFbxTexture::eSTANDARD);
//    lTexture->SetMappingType(KFbxTexture::eUV);
//    lTexture->SetMaterialUse(KFbxTexture::eMODEL_MATERIAL);
//    lTexture->SetSwapUV(false);
//    lTexture->SetTranslation(0.0, 0.0);
//    lTexture->SetScale(1.0, 1.0);
//    lTexture->SetRotation(0.0, 0.0);
//
//    // don't forget to connect the texture to the corresponding property of the material
//    if (lMaterial)
//        lMaterial->GetEmissiveColor().ConnectSrcObject(lTexture);
//}




///////  Ogre/AngelScript/SQLite
void fxFlexBody::addRef()
{
	//dynamic_cast<nxPhysManager*>(mPM)->mConsole->addOutput("DOING STUFF OVER HERE!!!!!!!!!!!!!!!!!!!");
}


void fxFlexBody::release()
{
	//dynamic_cast<nxPhysManager*>(mPM)->mConsole->addOutput("DOING STUFF OVER HERE!!!!!!!!!!!!!!!!!!!");
}


int fxFlexBody::doStuff(int arg)
{
	Ogre::Skeleton *kSkeleton =  mEntity->getSkeleton();
	Ogre::AnimationState *as;
	if (kSkeleton->hasAnimation("side"))
	{
		as = mEntity->getAnimationState("side");
		as->setLoop(true);
		as->setEnabled(true);
	}

	int i = arg * 4;
	return i;
}


Ogre::Vector3 fxFlexBody::getNodePosition() const
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

void fxFlexBody::setNodePosition(const Ogre::Vector3 &pos)
{
	//if (mNode)
	//	mNode->setPosition(pos);

	return;
}

