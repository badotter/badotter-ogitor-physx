////////////////////////////////////////////////////////////////////////////////////////////////////
//  nxPhysManager.cc
//  Chris Calef
//
//  adapted from LRGRigidBodyManager.cc
//  by Yossi Horowitz
//
//  Manages all of the Rigid Bodies in the scene, and serves as an interface between 
//  Torque and Novodex
////////////////////////////////////////////////////////////////////////////////////////////////////


#include "EcstasyMotion/nxPhysManager.h"
#include "EcstasyMotion/nxRigidBody.h"
#include "EcstasyMotion/fxRigidBody.h"
#include "EcstasyMotion/nxJoint.h"
#include "EcstasyMotion/fxJoint.h"
#include "EcstasyMotion/fxFlexBody.h"
#include "EcstasyMotion/myStream.h" 

#include "OgitorsScriptInterpreter.h"
#include "OgitorsScriptConsole.h"
#include "SceneManagerEditor.h"

extern Ogitors::OgitorsScriptConsole *gConsole;
extern SQLiteObject *gSQL;
extern fxFlexBody *gTweakerBot;
//#include "EcstasyMotion/nxPhysMaterial.h"
//#include "EcstasyMotion/fxPhysMaterial.h"

//using namespace Ogre;
//using namespace Ogitors;


const NxDebugRenderable *gDebugRenderer;

//class nxRaycastReport gRaycastReport;
//class nxTriggerReport gTriggerReport;
//class nxContactReport gContactReport;

//class fxRigidBody;
//class fxFlexBody;
//class fxFlexBodyPart;

////////////////////////////////////////////////////////////////////////////

class nxContactReport : public NxUserContactReport
{
public:
	void onContactNotify(NxContactPair &pair, NxU32 events)
	{//NxContactCallbackData ::localpos1
		nxPhysManager *kPM = ((nxPhysManager *)physManagerCommon::getPM());
		
		if ((kPM->mIsExiting)||(kPM->mIsReallyExiting))
			return;

		NxContactStreamIterator streamIter(pair.stream);
		//NxVec3 contactPos = streamIter.getPoint();
		//NxU32 numPoints = streamIter.getNumPoints();
		//Con::errorf("nxContactReport - points %d contact %f %f %f",
		//	numPoints,contactPos.x,contactPos.y,contactPos.z);

		if ((pair.actors[0]->userData)&&(pair.actors[1]->userData)&&(!kPM->mIsExiting)&&(!kPM->mIsReallyExiting)) 
		{
			iPhysUser *kPhysUser0 = ((iPhysUser *)pair.actors[0]->userData);
			iPhysUser *kPhysUser1 = ((iPhysUser *)pair.actors[1]->userData);

			physEntityType kEntType0 = kPhysUser0->mEntityType;
			physEntityType kEntType1 = kPhysUser1->mEntityType;

			//Con::errorf("nxContactReport, entity 0 type %d, entity 1 type %d",kEntType0,kEntType1);
			//First a little sanity check, wish I had a better way to check for validity of iPhysUsers.
			if ((kEntType0<0)||(kEntType0>1000)||(kEntType1<0)||(kEntType1>1000))
				return;//When I don't do this, a null gives me garbage, not null.

			//if ((kEntType0==PHYS_RIGID_BODY)&&(kEntType1==PHYS_RIGID_BODY))
			//{//didn't work... ?
			//  kPhysUser0->onCollision(kPhysUser1,0);
			//  kPhysUser1->onCollision(kPhysUser0,0);
			//}
			//So, for now, this is a very limited test.  The only time we need to call onCollision is when  flexbody 
			//collides with anything other than another flexbody.  This could use all kinds of more advanced logic, such 
			//as hitting flexbodies with other flexbodies that are flying at a certain speed threshold.  For now I'm just
			//trying to keep the orcs from falling down when they bump into other orcs on the ground.

			//TEMP: removed until flexbody onCollision is defined.
			//if (kEntType0==PHYS_FLEX_BODY_PART)
			//{
			//	kPhysUser0->onCollision(kPhysUser1,0);
			//} 

			//if (kEntType1==PHYS_FLEX_BODY_PART)//&&(kEntType0!=PHYS_FLEX_BODY))
			//{
			//	kPhysUser1->onCollision(kPhysUser0,0);
			//}
			//			
			//if ((kEntType0==PHYS_FLEX_BODY_PART)&&(kEntType1==PHYS_FLEX_BODY_PART))
			//{
			//	//Con::printf("flexbodypart colliding with flexbodypart, events %d",events);
			//} 
		}
	}
};
nxContactReport gContactReport;


class nxTriggerReport : public NxUserTriggerReport
{
public:
	virtual void onTrigger(NxShape& triggerShape, NxShape& otherShape, NxTriggerFlag status)
	{
		nxPhysManager *kPM = ((nxPhysManager *)physManagerCommon::getPM());

		if ((kPM->mIsExiting)||(kPM->mIsReallyExiting))
			return;

	//	if((status & NX_TRIGGER_ON_ENTER)||(status & NX_TRIGGER_ON_STAY)) 
	//	{
	//		if ((otherShape.getActor().userData)&&(triggerShape.getActor().userData)) 
	//		{

	//			iPhysUser *triggerPU,*otherPU;
	//			triggerPU = otherPU = NULL;

	//			triggerPU = ((iPhysUser *)triggerShape.getActor().userData);
	//			otherPU = ((iPhysUser *)otherShape.getActor().userData);

	//			physEntityType triggerET = triggerPU->mEntityType;
	//			physEntityType otherET = otherPU->mEntityType;

	//			//if ((triggerET==PHYS_RIGID_BODY)&&(otherET==PHYS_FLEX_BODY_PART)) Con::errorf("rigid body %d triggering flexbody %d",triggerPU,otherPU);
	//			if ((triggerET==PHYS_RIGID_BODY)&&(otherET==PHYS_FLEX_BODY_PART))//&&(0) TEMP addWeapon debugging
	//			{
	//				//otherPU->onTrigger(triggerPU,0,0);
	//				fxRigidBody* triggerRB = dynamic_cast<fxRigidBody*>(triggerPU);
	//				fxFlexBodyPart* otherFBP = dynamic_cast<fxFlexBodyPart*>(otherPU);
	//				//Con::printf("rigid body %s was triggered by a node %d!",triggerRB->mShapeName,otherFBP->mNodeIndex);

	//				if (dStrcmp(triggerRB->mShapeName,"SmallBall")==0 || dStrcmp(triggerRB->mShapeName,"ballreset")==0 || dStrcmp(triggerRB->mShapeName,"changegender")==0 || dStrcmp(triggerRB->mShapeName,"changeuniform")==0 || dStrcmp(triggerRB->mShapeName,"changemode")==0 || dStrcmp(triggerRB->mShapeName,"penaltytarget")==0 )
	//				{

	//					TSShape *kShape = otherFBP->mFlexBody->getShapeInstance()->getShape();

	//					int rHandIndex = kShape->findNode("rHand");
	//					int rCollarIndex = kShape->findNode("rCollar");
	//					int lHandIndex = kShape->findNode("lHand");
	//					int lCollarIndex = kShape->findNode("lCollar");
	//					int rFootIndex = kShape->findNode("rFoot");
	//					int rShinIndex = kShape->findNode("rShin");
	//					int lFootIndex = kShape->findNode("lFoot");
	//					int lShinIndex = kShape->findNode("lShin");
	//					int headIndex = kShape->findNode("head");


	//					if (otherFBP->mNodeIndex==headIndex)
	//					{
	//						if(dStrcmp(triggerRB->mShapeName,"SmallBall")==0)
	//							Con::executef(triggerRB,"BallHeadCollision");
	//						if(dStrcmp(triggerRB->mShapeName,"penaltytarget")==0)
	//							Con::executef(triggerRB,"onPenaltyTargetCollision");
	//					} 
	//					else if (((otherFBP->mNodeIndex>=lCollarIndex)&&(otherFBP->mNodeIndex<=lHandIndex))||
	//						((otherFBP->mNodeIndex>=rCollarIndex)&&(otherFBP->mNodeIndex<=rHandIndex)))
	//					{
	//						if(dStrcmp(triggerRB->mShapeName,"ballreset")==0)
	//							Con::executef(triggerRB,"onBallResetCollision");
	//						if(dStrcmp(triggerRB->mShapeName,"changegender")==0)
	//							Con::executef(triggerRB,"onChangeGenderCollision");
	//						if(dStrcmp(triggerRB->mShapeName,"changeuniform")==0)
	//							Con::executef(triggerRB,"onChangeUniformCollision");
	//						if(dStrcmp(triggerRB->mShapeName,"changemode")==0)
	//							Con::executef(triggerRB,"onChangeModeCollision");
	//						if(dStrcmp(triggerRB->mShapeName,"penaltytarget")==0)
	//							Con::executef(triggerRB,"onPenaltyTargetCollision");
	//						if(dStrcmp(triggerRB->mShapeName,"SmallBall")==0)
	//							Con::executef(triggerRB,"BallArmCollision");
	//					} 
	//					else if ((otherFBP->mNodeIndex==rFootIndex)||(otherFBP->mNodeIndex==lFootIndex))
	//					{
	//						if(dStrcmp(triggerRB->mShapeName,"SmallBall")==0)
	//							Con::executef(triggerRB,"BallFootCollision");
	//						if(dStrcmp(triggerRB->mShapeName,"penaltytarget")==0)
	//							Con::executef(triggerRB,"onPenaltyTargetCollision");
	//					} else {
	//						if(dStrcmp(triggerRB->mShapeName,"SmallBall")==0)
	//							Con::executef(triggerRB,"BallBodyCollision");
	//						if(dStrcmp(triggerRB->mShapeName,"penaltytarget")==0)
	//							Con::executef(triggerRB,"onPenaltyTargetCollision");
	//					}
	//				} 
	//				else if (dStrcmp(triggerRB->mShapeName,"WayPoint")==0 ) 
	//				{
	//					Con::executef(triggerRB,"BodyWaypointCollision");
	//				}
	//			}
	//			else if ((triggerET==PHYS_FLEX_BODY_PART)&&(otherET==PHYS_RIGID_BODY))
	//			{
	//				otherPU->onTrigger(triggerPU,0,0);
	//			}
	//			else if ((triggerET==PHYS_FLEX_BODY_PART)&&(otherET==PHYS_FLEX_BODY_PART))
	//			{
	//				Con::printf("flexbodypart from %s triggering flexbodypart from %s!",
	//					dynamic_cast<fxFlexBodyPart*>(otherPU)->mFlexBody->mShapeName,
	//					dynamic_cast<fxFlexBodyPart*>(triggerPU)->mFlexBody->mShapeName);
	//				if (dynamic_cast<fxFlexBodyPart*>(triggerPU)->mFlexBody != dynamic_cast<fxFlexBodyPart*>(otherPU)->mFlexBody)
	//				{
	//					otherPU->onTrigger(triggerPU,0,0);
	//				}
	//			}
	//			else if ((triggerET==PHYS_RIGID_BODY)&&(otherET==PHYS_RIGID_BODY))
	//			{//TEMP: code specific to Telibrahma
	//				fxRigidBody* triggerRB = dynamic_cast<fxRigidBody*>(triggerPU);
	//				fxRigidBody* otherRB = dynamic_cast<fxRigidBody*>(otherPU);
	//				if (dStrcmp(triggerRB->mShapeName,"SmallBall")==0)
	//				{
	//					if (triggerRB->mHadCollision)
	//						return;
	//					triggerRB->mHadCollision = true;
	//					if (dStrcmp(otherRB->mShapeName,"goal")==0)
	//					{
	//						Con::executef(triggerRB,"onGoalBoxCollision");
	//					} 
	//					else if (dStrcmp(otherRB->mShapeName,"wall")==0)
	//					{
	//						Con::executef(triggerRB,"onBallWallCollision");
	//					}
	//					else if (dStrcmp(otherRB->mShapeName,"balltarget")==0)
	//					{
	//						//ballTargetMS = Platform::getVirtualMilliseconds();
	//						//Con::printf("ball target milliseconds = %d, last time = %d, diff %d",
	//						//	ballTargetMS,lastBallTargetMS,ballTargetMS-lastBallTargetMS);
	//						//if ((ballTargetMS-lastBallTargetMS)>3000)
	//						Con::executef(triggerRB,"onBallTargetCollision");
	//						//lastBallTargetMS = ballTargetMS;
	//					}
	//				}
	//				else if (dStrcmp(triggerRB->mShapeName,"HealthKit")==0) 
	//				{
	//					if (dStrcmp(otherRB->mShapeName,"SmallBall")==0)
	//					{
	//						Con::executef(otherRB,"onGoalCollision");							
	//					}
	//				}
	//			}
	//			//else if ((triggerET==PHYS_FLEX_BODY)&&(otherET==PHYS_FLEX_BODY))
	//			//else if ((triggerET==PHYS_FLEX_BODY)&&(otherPU))//just pass the other entity to flexbody::onTrigger, deal with it there.
	//			//{
	//			//triggerPU->onTrigger(otherPU,0,0);
	//			//otherPU->onTrigger(triggerPU,0,0);
	//			//}//probably we'll get a callback for both entities, so only need to call onTrigger for one at a time.
	//			//question is, how often do we get these callbacks, if we're still overlapping?  once per worldstep? once per frame? other?
	//		}
	//	} 
	}
};  
nxTriggerReport gTriggerReport;

//////////////////////////////

class nxRaycastReport : public NxUserRaycastReport
{
public:
	nxRaycastReport(){};
	virtual ~nxRaycastReport(){};

	virtual bool onHit(const NxRaycastHit& rayhit)
	{
		//record information here
		//fxFlexBody *kFlexBody = ((nxRigidBody*)rayhit.shape->getActor().userData)->mFlexBody;
		if (rayhit.distance < 12.0) {
			//if (((physShapeData *)rayhit.shape.userData)->) {
			if (rayhit.shape->userData) {
				//if (((physShapeData *)rayhit.shape->userData)->mType==PHYS_FLEX_BODY_PART) {
				//if (((nxRigidBody*)rayhit.shape->getActor().userData)->mFlexBody) {
				//  fxFlexBody *kFlexBody = ((nxRigidBody*)rayhit.shape->getActor().userData)->mFlexBody;
				////kFlexBody->mIsAnimating = false;
				////kFlexBody->stopThread(0);
				//kFlexBody->clearKinematic();
				////Con::errorf("actor[1]: %s",kFlexBody->mShapeInstance->mShape->mSourceResource->name);
				//}
				//}
			}
		}
		return true; //or false to stop the raycast
	}
};
nxRaycastReport gRaycastReport;

///////////////////////////////////////////////////////////////////////

nxPhysManager::nxPhysManager()
{
	mPM = this; 

	Ogitors::OgitorsScriptInterpreter *kInterpreter = Ogitors::OgitorsRoot::getSingleton().GetScriptInterpreter();
	mScriptEngine = dynamic_cast<Ogitors::AngelScriptInterpreter*>(kInterpreter)->mEngine;
	gConsole = &(Ogitors::OgitorsScriptConsole::getSingleton());

	gSQL = new SQLiteObject();//Q:  Should I open the database here once, and keep it open for the whole time  
	//the program is running, or should I open it and close it every time I use it?
	mDR = NULL;
	mTerrain = NULL;

	mMissionId = 0;
	mSceneId = 0;
	mPhysicsSDK = NULL;
	mScene = NULL;
	mHWScene = NULL;
	mFluidScene = NULL;
	mDebugRender = false;// true;//    
	mDefaultGravity = Ogre::Vector3(0.0,-4.8,0.0);//-9.8 //FIX!  Expose this to script/DB
	//mDefaultGravity = Ogre::Vector3(0.0,0.0,0.0);
	mCurrStep = 0;

	mBaseDynamicFriction = 0.2;//0.75;
	mBaseStaticFriction = 0.2;//0.25;
	mBaseRestitution = 0.5;

	mCurrStepTime = 0;
	mLastStepTime = 0;
	mStepTime = 0.032f;//TEMP //0.032f;//0.064f;//32 millisecond ticks?
	mTimeFactor = 1.6;
	mNextActorGroup = 2;//save 0 for world, 1 for unconnected rigid bodies.

	mNumMeshBodies = 0;
	mNumMeshes = 0;
	mNumMaterials = 0;
	mNumRaycasts = 0;
	mNumFlexbodies = 0;

	for (unsigned int i=0;i<MAX_MESH_BODIES;i++) mMeshBodies[i]=NULL;
	for (unsigned int i=0;i<MAX_MESHES;i++) mMeshes[i]=NULL;
	//for (unsigned int i=0;i<MAX_MATERIALS;i++) mPhysMaterials[i]=NULL;
	for (unsigned int i=0;i<MAX_RAYCASTS;i++)
	{
		mRaycasts[i].start = Ogre::Vector3::ZERO;
		mRaycasts[i].dir = Ogre::Vector3::ZERO;
		mRaycasts[i].force = 0.0;
		mRaycasts[i].damage = 0.0;
		mRaycasts[i].Dirt = "";
		mRaycasts[i].Brick = "";
		mRaycasts[i].Water = "";
		mRaycasts[i].Blood = "";
	}   
	//pRandom.setSeed(111);

	mIsExiting = false;
	mIsReallyExiting = false;

	mSceneEventCounter = 1;//Can't start at 0, or first ID will always be confused with NULL from select box.
	mSceneStartStep = 0;;//in milliseconds
	mSceneStartTime = 0.0;
	mSceneDuration = 0.0;//in seconds
	mSceneRecordLocal = true;

	mLastImpulseEvent = NULL;
	mLastDurationEvent = NULL;
	mLastInterpolationEvent = NULL;
	mLastFollowEvent = NULL;

	mMissionName.clear();
	mSceneName.clear();

	//gPhysRaycastStart.zero();
	//gPhysRaycastEnd.zero();

	init();

}

nxPhysManager::~nxPhysManager()
{
	delete gSQL;

	if (mScene)
	{
		mPhysicsSDK->releaseScene(*mScene);
		mScene = NULL;
	}
}


/////////////////////////
int g_numEnemies = 20;//Hehe, trivial script testing here...
void incrementEnemies()
{
	g_numEnemies++;
}
/////////////////////////

void numActors();
void addActor();
int getTweakerBot();

void nxPhysManager::init()
{
	// Initialize PhysicsSDK
	mPhysicsSDK = NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION);

 	if(!mPhysicsSDK)
		return;

	createScene();

	scriptInit();
}


void nxPhysManager::createScene()
{

	if (!mPhysicsSDK)
		return;

	// Create a scene
	NxSceneDesc sceneDesc;
	sceneDesc.gravity				= NxVec3(mDefaultGravity.x,mDefaultGravity.y,mDefaultGravity.z);
	//sceneDesc.broadPhase			= NX_BROADPHASE_COHERENT;
	sceneDesc.userTriggerReport = &gTriggerReport;
	sceneDesc.userContactReport = &gContactReport;
	sceneDesc.maxIter = 8;
	//sceneDesc.collisionDetection	= true;
	mScene = NULL;
	mScene = mPhysicsSDK->createScene(sceneDesc);
	//if (!mScene)
	//	Con::errorf("ERROR: Failed to create physics scene!");

	//mScene->setTiming(1.0f/32.0f,1,NX_TIMESTEP_FIXED);
	//mScene->setUserContactReport(&gContactReport);
	//mScene->setUserTriggerReport(&gTriggerReport);

#ifdef USE_HARDWARE
	//sceneDesc.simType                = NX_SIMULATION_HW;
	////sceneDesc.hwSceneType            = NX_HW_SCENE_TYPE_RB;
	//mHWScene = mPhysicsSDK->createScene(sceneDesc);
	////sceneDesc.hwSceneType            = NX_HW_SCENE_TYPE_FLUID;
	////mFluidScene = mPhysicsSDK->createScene(sceneDesc);
#endif

	NxMaterial *testMaterial = mScene->getMaterialFromIndex(0);
	NxMaterialDesc m;
	m.dynamicFriction = mBaseDynamicFriction;
	m.staticFriction  = mBaseStaticFriction;
	m.restitution     = mBaseRestitution;
	testMaterial->loadFromDesc(m);

	if (mHWScene)
	{
		testMaterial = mHWScene->getMaterialFromIndex(0);
		testMaterial->loadFromDesc(m);
	}

	if (mFluidScene)
	{
		testMaterial = mFluidScene->getMaterialFromIndex(0);
		testMaterial->loadFromDesc(m);
	}


	//////////////////// TEMP - take out till we get physMaterial back in.
	//for (unsigned int c=0;c<mNumMaterials;c++) {
	//	m.dynamicFriction = mPhysMaterials[c]->getDynamicFriction();
	//	m.staticFriction = mPhysMaterials[c]->getStaticFriction();
	//	m.restitution = mPhysMaterials[c]->getRestitution();
	//	mPhysMaterials[c]->setIndex(mScene->createMaterial(m)->getMaterialIndex());
	//	if (mHWScene) mHWScene->createMaterial(m);
	//	if (mFluidScene) mFluidScene->createMaterial(m);
	//}
	//for (SimSetIterator obj(Sim::getRootGroup()); *obj; ++obj)
	//{
	//	ConsoleObject* nobj = dynamic_cast<ConsoleObject*>(*obj);
	//	if (nobj) 
	//	{
	//		if (!dStrcmp(nobj->getClassName(),"fxPhysMaterial"))
	//		{
	//			((fxPhysMaterial *)nobj)->getIndex();
	//		}
	//	}
	//}
	//////////////////////////////////////////////

	for (unsigned int i=0;i<MAX_MESHES;i++) mMeshes[i] = NULL;


	mPhysicsSDK->setParameter(NX_SKIN_WIDTH,0.001f);

	mPhysicsSDK->setParameter(NX_VISUALIZE_COLLISION_SHAPES,1);
	mPhysicsSDK->setParameter(NX_TRIGGER_TRIGGER_CALLBACK,true);
	//mPhysicsSDK->setParameter(NX_VISUALIZE_WORLD_AXES,1);
	mPhysicsSDK->setParameter(NX_VISUALIZATION_SCALE, 1.0f);
	//mPhysicsSDK->setParameter(NX_VISUALIZE_JOINT_LIMITS,1);
	//mPhysicsSDK->setParameter(NX_VISUALIZE_JOINT_LOCAL_AXES,1);
	//mPhysicsSDK->setParameter(NX_VISUALIZE_ACTOR_AXES,1);
	//mPhysicsSDK->setParameter(NX_VISUALIZE_BODY_AXES,1);

	mIsExiting = false;
	
	NxInitCooking();


	gConsole->addOutput("created physics scene");
}



void nxPhysManager::destroyScene()
{

	//if (mSQL)
	//{
	//	mSQL->CloseDatabase();
	//	delete mSQL;
	//}

	if (mScene)
	{
		stopPhysics();
		for (int i=0;i<mJointList.size();i++)
		{
			mScene->releaseJoint(*(mJointList[i]->mJoint));
			delete mJointList[i];
			gConsole->addOutput("deleted joint");
		}
		for (int i=0;i<mBodyList.size();i++)
		{
			mScene->releaseActor(*(mBodyList[i]->mActor));
			delete mBodyList[i]->mPhysUser;
			delete mBodyList[i];
			gConsole->addOutput("deleted body");
		}

		mBodyList.clear();
		mJointList.clear();
		gConsole->addOutput("cleared bodylist and jointlist");


		//mImpulseEventList.reset();//Danger: are we deleting the actual objects somewhere?
		//mDurationEventList.reset();
		//mInterpolationEventList.reset();
		//mFollowEventList.reset();

		mCurrDurationEvents.clear();
		mCurrInterpolationEvents.clear();

		mSceneEventCounter = 0;
		mSceneStartStep = 0;
		mSceneStartTime = 0.0;
		mSceneDuration = 0.0;
		mLastImpulseEvent = NULL;
		mLastDurationEvent = NULL;
		mLastInterpolationEvent = NULL;
		mLastFollowEvent = NULL;

		mNumMaterials = 0;


		//mIsExiting = true;

		mPhysicsSDK->releaseScene(*mScene);
		mScene = NULL;

		gConsole->addOutput("cleared the scene");

	}
	//mIsExiting = false;

}

void nxPhysManager::stopPhysics()
{
	mScene->fetchResults(NX_RIGID_BODY_FINISHED,true);
}

void nxPhysManager::startPhysics()
{
	mScene->simulate(mStepTime);
}

//void nxPhysManager::addPhysMaterial(physMaterial *mat)
//{
//	mPhysMaterials[mNumMaterials++] = dynamic_cast<nxPhysMaterial *>(mat);
//}

//void nxPhysManager::addRigidBody(physRigidBody *pRB)
//{
//	mBodyList.push_back(dynamic_cast<nxRigidBody*> (pRB));
//}


physRigidBody *nxPhysManager::createRigidBody()
{
	nxRigidBody *p = new nxRigidBody();
	p->mPM = this;
	mBodyList.push_back(p);
	//p->setup();
	return (physRigidBody *)p;
}

void nxPhysManager::removeRigidBody(physRigidBody *pRB)
{
	dynamic_cast<nxRigidBody*>(pRB)->mRemove = true;
}

physJoint *nxPhysManager::createJoint()
{
	nxJoint *p = new nxJoint();
	p->mPM = this;
	mJointList.push_back(p);
	return (physJoint *)p;
}

void nxPhysManager::removeJoint(physJoint *pJoint)
{
	//release joint here
}

void nxPhysManager::stepPhysics()
{
	if (!mPhysicsSDK)
		return;

	//HERE: find system time in milliseconds, see if 32 have gone by, if not then wait.
	char printstring[255];
	//SYSTEMTIME st;
	//GetSystemTime(&st);
	mCurrStepTime = clock();//st.wMilliseconds;
	int kTimeDiff = mCurrStepTime - mLastStepTime;

	if (kTimeDiff < (mStepTime * 1000))//1000(mStepTime in seconds, clock() in MS)
		return;

	mLastStepTime = mCurrStepTime;
	//Ogitors::OgitorsScriptConsole *console = &(Ogitors::OgitorsScriptConsole::getSingleton());
	//sprintf(printstring,"world step, time: %d",mCurrStepTime);
	//gConsole->addOutput(printstring);

	//PROFILE_START(nxPhysManager_stepPhysics);
	std::vector<nxRigidBody*>::iterator pBodyIterator;
	//nxJoint* pJointIterator	 = NULL;
	//nxSpringAndDamper* pSpringIterator = NULL;
	////nxFluid* pFluidIterator	 = NULL;
	//RenderClothExample* pClothIterator	 = NULL;
	//fxFlexBody* pFlexBodyIterator = NULL;


	//PROFILE_START(nxPhysManager_stepPhysics_PauseSWSim);
	//mScene->fetchResults(NX_RIGID_BODY_FINISHED,true);//true
	stopPhysics();
	//PROFILE_END();
	//PROFILE_START(nxPhysManager_stepPhysics_PauseHWSim);
	//if (mHWScene) mHWScene->fetchResults(NX_RIGID_BODY_FINISHED,true);
	//PROFILE_END();
	//if (mFluidScene) mFluidScene->fetchResults(NX_RIGID_BODY_FINISHED,true);

	//SceneGraph::getPostRenderSignal().notify( &nxPhysManager::renderRaycast );

	//mLastStepTime = mCurrStepTime;
	//mCurrStepTime = Sim::getCurrentTime();


	///////////  Ecstasy Scene Events  //////////////////////
	//ecstasySceneEvent *pEventIterator;
	////HERE:  now that we have scene events, let's step through them.
	////Con::printf("stepping physics, impulse %d duration %d interp %d",mImpulseEventList.size(),
	////	mDurationEventList.size(),mInterpolationEventList.size());

	//if (mImpulseEventList.size())
	//{
	//	if (mLastImpulseEvent) pEventIterator = mLastImpulseEvent;
	//	else pEventIterator = NULL;
	//	for (pEventIterator = mImpulseEventList.next(pEventIterator); pEventIterator;
	//		pEventIterator = mImpulseEventList.next(pEventIterator))
	//	{
	//		//For impulse events, check the next one and each one after that to see if it  
	//		//_just_ passed its start time.
	//		if ( ((mCurrStepTime-mSceneStartStep)>((int)(pEventIterator->time*1000.0))) &&
	//			((mLastStepTime-mSceneStartStep)<(int)(pEventIterator->time*1000.0)) )
	//		{ //Simple impulse event, so just act on it right here.
	//			//Con::printf("handling impulse event!!!!!!!!!!!!!!");
	//			handleSceneEvent(pEventIterator);
	//			mLastImpulseEvent = pEventIterator;
	//		}
	//	}
	//}
	//if (mDurationEventList.size())
	//{
	//	if (mLastDurationEvent) pEventIterator = mLastDurationEvent;
	//	else pEventIterator = NULL;
	//	for (pEventIterator = mDurationEventList.next(pEventIterator); pEventIterator;
	//		pEventIterator = mDurationEventList.next(pEventIterator))
	//	{
	//		if ( ((mCurrStepTime-mSceneStartStep)>((int)(pEventIterator->time*1000.0))) &&
	//			((mLastStepTime-mSceneStartStep)<(int)(pEventIterator->time*1000.0)) )
	//		{
	//			mCurrDurationEvents.push_back(pEventIterator);
	//			mLastDurationEvent = pEventIterator;
	//		}
	//	}
	//	for (unsigned int i=0; i<mCurrDurationEvents.size(); i++)
	//	{
	//		if ((mCurrStepTime-mSceneStartStep)>((int)(mCurrDurationEvents[i]->time*1000.0)+(int)(mCurrDurationEvents[i]->duration*1000.0)))
	//		{
	//			mCurrDurationEvents.erase(i);
	//			i--;
	//		} else {
	//			handleSceneEvent(mCurrDurationEvents[i]);
	//		}
	//	}
	//}
	//if (mInterpolationEventList.size())
	//{
	//	if (mLastInterpolationEvent) pEventIterator = mLastInterpolationEvent;
	//	else pEventIterator = NULL;
	//	for (pEventIterator = mInterpolationEventList.next(pEventIterator); pEventIterator;
	//		pEventIterator = mInterpolationEventList.next(pEventIterator))
	//	{
	//		if ( ((mCurrStepTime-mSceneStartStep)>((int)(pEventIterator->time*1000.0))) &&
	//			((mLastStepTime-mSceneStartStep)<(int)(pEventIterator->time*1000.0)) )
	//		{ 
	//			mCurrInterpolationEvents.push_back(pEventIterator);
	//			mLastInterpolationEvent = pEventIterator;
	//		}
	//	}
	//	//   if ( ((currentTime-startStep)>(int)(nextEvent.time*1000.0)) &&
	//       //((currentTime)<(startStep+(int)(nextEvent.duration*1000.0)) )
	//	for (unsigned int i=0; i<mCurrInterpolationEvents.size(); i++)
	//	{
	//		ecstasySceneEvent *kSE = mCurrInterpolationEvents[i];
	//		if (!(kSE->next))
	//		{//We reached the end of this chain.
	//			mCurrInterpolationEvents.erase(i);
	//			i--;
	//			continue;
	//		}
	//		else if ((mCurrStepTime-mSceneStartStep)>((int)(kSE->next->time*1000.0))) 
	//		{//We reached the next event in the chain.
	//			mCurrInterpolationEvents.erase(i);
	//			i--;
	//		} else {//We're still on the chain, so interpolate.
	//			float top = (float)( mCurrStepTime - (mSceneStartStep + (kSE->time * 1000.0)));
	//			float bottom = ( (kSE->next->time - kSE->time) * 1000.0 );
	//			float delta = top/bottom;
	//			//float delta = (float)( mCurrStepTime - (mSceneStartStep + (kSE->time * 1000.0)) /
	//			//	( (kSE->next->time - kSE->time) * 1000.0 ));///(current time - this event start time)/(next event start time - this event start time);
	//			handleInterpolationSceneEvent(mCurrInterpolationEvents[i],delta);
	//		}
	//	}
	//}
	//if (mFollowEventList.size())
	//{
	//	if (mLastFollowEvent) pEventIterator = mLastFollowEvent;
	//	else pEventIterator = NULL;
	//	for (pEventIterator = mFollowEventList.next(pEventIterator); pEventIterator;
	//		pEventIterator = mFollowEventList.next(pEventIterator))
	//	{
	//		if ( ((mCurrStepTime-mSceneStartStep)>((int)(pEventIterator->time*1000.0))) &&
	//			((mLastStepTime-mSceneStartStep)<(int)(pEventIterator->time*1000.0)) )
	//		{ 
	//			//Hm, maybe no need for currFollowEvents after all.  Just handle it like an impulse, if it's a First Follow event, which it has to
	//			//be to get a positive start time.  After this, keep track of it by switching to next event, no more time searching.
	//			handleFollowSceneEvent(pEventIterator);
	//			mLastFollowEvent = pEventIterator;
	//		}
	//	}
	//}
	/////////  End Ecstasy Scene Events  ////////////////////
	//sprintf(printstring,"bodyList size:  %d",mBodyList.size());
	//gConsole->addOutput(printstring);

	for (int i=0;i<mBodyList.size();i++)
	{
		//sprintf(printstring,"bodyList %d: Actor %d  setup %d  remove %d",
		//	i,(int)mBodyList[i]->mActor,(int)mBodyList[i]->mSetup,(int)mBodyList[i]->mRemove);
		//gConsole->addOutput(printstring);
		if (mBodyList[i]->mActor)// && mBodyList[i]->mSetup && !mBodyList[i]->mRemove)			
			mBodyList[i]->onWorldStep();
	}


	////PROFILE_START(nxPhysManager_stepPhysics_FlexBody);
	//if (mFlexBodyList.size()>0) { 
	//	pFlexBodyIterator = NULL;
	//	for(	pFlexBodyIterator = mFlexBodyList.next(pFlexBodyIterator); pFlexBodyIterator;
	//		pFlexBodyIterator = mFlexBodyList.next(pFlexBodyIterator))
	//	{
	//		pFlexBodyIterator->updateTrigger();//onWorldStep();//flexbody::onWorldStep gets called at the end of flexbodypart::onworldstep
	//	}
	//}
	////PROFILE_END();

	//PROFILE_START(nxPhysManager_stepPhysics_Joints);
	//if (mJointList.size()>0) { 
	//	pJointIterator = NULL;
	//	for(	pJointIterator = mJointList.next(pJointIterator); pJointIterator;
	//		pJointIterator = mJointList.next(pJointIterator))
	//	{
	//		if (pJointIterator->mJoint) pJointIterator->onWorldStep();
	//	}
	//}
	//PROFILE_END();

	//PROFILE_START(nxPhysManager_stepPhysics_Springs);
	//if (mSpringList.size()>0) { 
	//	pSpringIterator = NULL;
	//	for(	pSpringIterator = mSpringList.next(pSpringIterator); pSpringIterator;
	//		pSpringIterator = mSpringList.next(pSpringIterator))
	//	{
	//		if (pSpringIterator->mEffector) pSpringIterator->onWorldStep();
	//	}
	//}
	//PROFILE_END();

	//PROFILE_START(nxPhysManager_stepPhysics_Fluids);
	//if (mFluidList.size()>0) { 
	//   pFluidIterator = NULL;
	//   for(	pFluidIterator = mFluidList.next(pFluidIterator); pFluidIterator;
	//      pFluidIterator = mFluidList.next(pFluidIterator))
	//   {
	//      if (pFluidIterator->mFluid) pFluidIterator->onWorldStep();
	//   }
	//}
	//PROFILE_END();

	//PROFILE_START(nxPhysManager_stepPhysics_Cloth);
	//if (mClothList.size()>0) { 
	//	pClothIterator = NULL;
	//	for(	pClothIterator = mClothList.next(pClothIterator); pClothIterator;
	//		pClothIterator = mClothList.next(pClothIterator))
	//	{
	//		if (pClothIterator->mCloth) pClothIterator->onWorldStep();
	//	}
	//}
	//PROFILE_END();


	//Setup lists
	/////////////////////////////////////////////////////////////////////////////////////////////////
	//PROFILE_START(nxPhysManager_stepPhysics_Body_Setup);
	//Setup lists: add bodies to sim only during pause
	//Cancel: trying a new way, stop physics whenever you need to add a body.
	//for (int i=0;i<mBodyList.size();i++)
	//{//A little wasteful to run through the whole list checking for mSetup or mRemove every tick...
	//	// but seeing how it goes.
	//	if (!mBodyList[i]->mSetup)
	//	{
	//		mBodyList[i]->setup();	
	//		mBodyList[i]->mSetup = true;	
	//	}
	//}
	//if (mBodySetupList.size()>0) {
	//	pBodyIterator = NULL;
	//	for(	pBodyIterator = mBodySetupList.next(pBodyIterator); pBodyIterator;
	//		pBodyIterator = mBodySetupList.next(pBodyIterator))
	//	{
	//		pBodyIterator->setup();
	//		addRigidBody(pBodyIterator);
	//	}
	//	mBodySetupList.reset();
	//}
	//PROFILE_END();

	//PROFILE_START(nxPhysManager_stepPhysics_Joint_Setup);
	//if (mJointSetupList.size()>0) {
	//	pJointIterator = NULL;
	//	for(	pJointIterator = mJointSetupList.next(pJointIterator); pJointIterator;
	//		pJointIterator = mJointSetupList.next(pJointIterator))
	//	{
	//		pJointIterator->setup();
	//		addJoint(pJointIterator);
	//	}
	//	mJointSetupList.reset();
	//}
	//PROFILE_END();

	//if (mFlexBodySetupList.size()>0) {
	//	pFlexBodyIterator = NULL;
	//	for(	pFlexBodyIterator = mFlexBodySetupList.next(pFlexBodyIterator); pFlexBodyIterator;
	//		pFlexBodyIterator = mFlexBodySetupList.next(pFlexBodyIterator))
	//	{
	//		pFlexBodyIterator->setup();
	//		addFlexBody(pFlexBodyIterator);
	//	}
	//	mFlexBodySetupList.reset();
	//}

	//if (mSpringSetupList.size()>0) {
	//	pSpringIterator = NULL;
	//	for(	pSpringIterator = mSpringSetupList.next(pSpringIterator); pSpringIterator;
	//		pSpringIterator = mSpringSetupList.next(pSpringIterator))
	//	{
	//		pSpringIterator->setup();
	//		addSpring(pSpringIterator);
	//	}
	//	mSpringSetupList.reset();
	//}

	//if (mFluidSetupList.size()>0) {
	//   pFluidIterator = NULL;
	//   for(	pFluidIterator = mFluidSetupList.next(pFluidIterator); pFluidIterator;
	//      pFluidIterator = mFluidSetupList.next(pFluidIterator))
	//   {
	//      pFluidIterator->setup();
	//      addFluid(pFluidIterator);
	//   }
	//   mFluidSetupList.reset();
	//}

	//if (mClothSetupList.size()>0) {
	//	pClothIterator = NULL;
	//	for(	pClothIterator = mClothSetupList.next(pClothIterator); pClothIterator;
	//		pClothIterator = mClothSetupList.next(pClothIterator))
	//	{
	//		pClothIterator->setup();
	//		addCloth(pClothIterator);
	//	}
	//	mClothSetupList.reset();
	//}




	/////////////////////////////////////////////////////////////////////////////////////////////////
	//Release lists:  remove geometry from the sim only during a pause.
	//New way: stop physics when you're ready to delete everything.
	//pBodyIterator = mBodyList.begin();
	//for (int i=0; i<mBodyList.size(); i++ )
	//{
	//	if (mBodyList[i]->mActor && mBodyList[i]->mRemove)
	//	{
	//		mScene->releaseActor(*mBodyList[i]->mActor);
	//		delete mBodyList[i];
	//		mBodyList.erase(pBodyIterator);
	//		i--;//MAYBE?  Trying to figure out how to avoid skipping one.
	//	}
	//	pBodyIterator++;
	//}
	//if (mBodyReleaseList.size()>0) {
		//pBodyIterator = NULL;
		//for(	pBodyIterator = mBodyReleaseList.next(pBodyIterator); pBodyIterator;
		//	pBodyIterator = mBodyReleaseList.next(pBodyIterator))
		//{
		//	if (pBodyIterator->mActor) mScene->releaseActor(*pBodyIterator->mActor);
		//}
		//mBodyReleaseList.reset();
	//}

	//if (mJointReleaseList.size()>0) {
	//	pJointIterator = NULL;
	//	for(	pJointIterator = mJointReleaseList.next(pJointIterator); pJointIterator;
	//		pJointIterator = mJointReleaseList.next(pJointIterator))
	//	{
	//		if (pJointIterator->mJoint) mScene->releaseJoint(*pJointIterator->mJoint);
	//	}
	//	mJointReleaseList.reset();
	//}

	//if (mSpringReleaseList.size()>0) {
	//	pSpringIterator = NULL;
	//	for(	pSpringIterator = mSpringReleaseList.next(pSpringIterator); pSpringIterator;
	//		pSpringIterator = mSpringReleaseList.next(pSpringIterator))
	//	{
	//		mScene->releaseEffector(*pSpringIterator->mEffector);
	//	}
	//	mSpringReleaseList.reset();
	//}

	//if (mFluidReleaseList.size()>0) {
	//   pFluidIterator = NULL;
	//   for(	pFluidIterator = mFluidReleaseList.next(pFluidIterator); pFluidIterator;
	//      pFluidIterator = mFluidReleaseList.next(pFluidIterator))
	//   {
	//      //if (pFluidIterator->mFluid) mScene->releaseFluid(*pFluidIterator->mFluid);
	// if (pFluidIterator->mFluid) mHWScene->releaseFluid(*pFluidIterator->mFluid);
	//   }
	//   mFluidReleaseList.reset();
	//}

	//if (mClothReleaseList.size()>0) {
	//	pClothIterator = NULL;
	//	for(	pClothIterator = mClothReleaseList.next(pClothIterator); pClothIterator;
	//		pClothIterator = mClothReleaseList.next(pClothIterator))
	//	{
	//		if (pClothIterator->mCloth) 
	//			mScene->releaseCloth(*pClothIterator->mCloth);
	//		
	//		if ( pClothIterator->mClothMesh  )
	//			mPhysicsSDK->releaseClothMesh( *(pClothIterator->mClothMesh) );
 //     
	//		pClothIterator->mCloth = NULL;
	//		pClothIterator->mClothMesh = NULL;
	//	}
	//	mClothReleaseList.reset();
	//}

	//if (mFlexBodyReleaseList.size()>0) {
	//	pFlexBodyIterator = NULL;
	//	for(	pFlexBodyIterator = mFlexBodyReleaseList.next(pFlexBodyIterator); pFlexBodyIterator;
	//		pFlexBodyIterator = mFlexBodyReleaseList.next(pFlexBodyIterator))
	//	{
	//		pFlexBodyIterator->releaseActors();
	//	}
	//	mFlexBodyReleaseList.reset();
	//}


	//now, the delayed raycasts need to happen when the simulation is RUNNING, not when it's paused.
	//(Right?)
	//PROFILE_START(nxPhysManager_stepPhysics_Raycasts);
	//if (mNumRaycasts==1)
	//{
	//	if (mRaycasts[0].createStep < mCurrStep-1) 
	//	{
	//		nxCastRay(mRaycasts[0]);
	//		mNumRaycasts = 0;
	//	}
	//} 
	//else if (mNumRaycasts>1)
	//{
	//	float tempNumRaycasts = mNumRaycasts;
	//	for (unsigned int i=0;i<mNumRaycasts;i++)
	//	{
	//		if (mRaycasts[i].createStep < mCurrStep-1) 
	//		{
	//			nxCastRay(mRaycasts[i]);//hitPos = nxCastRay(mRaycasts[i], could send hitPos back if I had a pointer 
	//			for (unsigned int j=i;j<(tempNumRaycasts-1);j++)
	//			{//move everybody else up one, shouldn't have to do more than one or two passes unless
	//				mRaycasts[j].start      = mRaycasts[j+1].start;//bot counts get really up there.
	//				mRaycasts[j].dir        = mRaycasts[j+1].dir;
	//				mRaycasts[j].force      = mRaycasts[j+1].force;
	//				mRaycasts[j].damage     = mRaycasts[j+1].damage;
	//				mRaycasts[j].Dirt       = mRaycasts[j+1].Dirt;
	//				mRaycasts[j].Brick      = mRaycasts[j+1].Brick;
	//				mRaycasts[j].Water      = mRaycasts[j+1].Water;
	//				mRaycasts[j].Blood      = mRaycasts[j+1].Blood;
	//				mRaycasts[j].createStep = mRaycasts[j+1].createStep;
	//			}
	//			tempNumRaycasts--;
	//			i--;
	//		}//This is unnecessarily complicated; it allows raycasts anywhere in the stack to "time out" in any order.
	//	}//In reality, they should pile up in chronological order, so once you find one that isn't ready to go, you should be done.
	//	mNumRaycasts = tempNumRaycasts;
	//}
	//PROFILE_END();

	if (mDebugRender) 
	{
		//SceneGraph::getPostRenderSignal().notify( &nxPhysManager::renderDebug );
		if (mDR)
			mDR->setVisible(true);
		debugRender();
		//gDebugRenderer = mScene->getDebugRenderable();//>visualize();
		//gDebugRenderer->
		//mPhysicsSDK->  >visualize(gDebugRenderer); 
	} else {
		if (mDR)
			mDR->setVisible(false);
	}
	if (mLastStepTime)
	{
		//PROFILE_START(nxPhysManager_stepPhysics_StartSWSim);
		//float kStep = ((float)(Platform::getRealMilliseconds() - mLastStepTime)) * 0.001 * mTimeFactor;
		//mScene->simulate(kStep);
		//mScene->simulate(mStepTime);
		startPhysics();
		//PROFILE_END();	  
		//PROFILE_START(nxPhysManager_stepPhysics_StartHWSim);
		//if (mHWScene) mHWScene->simulate(kStep);
		//if (mHWScene) mHWScene->simulate(mStepTime);
		//PROFILE_END();
		//if (mFluidScene) mFluidScene->simulate(kStep);
	} else {
		startPhysics();
		//mScene->simulate(mStepTime);
		//if (mHWScene) mHWScene->simulate(mStepTime);
		//if (mFluidScene) mFluidScene->simulate(mStepTime);

	}
	//int curStep = Platform::getRealMilliseconds();
	//mTimeDelta = curStep - mLastStepTime;
	//mLastStepTime = curStep;



	//FIX: have to delete this one raycast and rearrange the list, in case there's more than one.
	//
	//mScene->flushStream();

	mCurrStep++;

	//if (mIsExiting)//(mIsReallyExiting) 
	//{
	//	if (mScene) {
	//		mScene->fetchResults(NX_RIGID_BODY_FINISHED,true);
	//		mPhysicsSDK->releaseScene(*mScene);
	//		mScene = NULL;
	//	}

	//	if (mFluidScene) {
	//		mFluidScene->fetchResults(NX_RIGID_BODY_FINISHED,true);
	//		mPhysicsSDK->releaseScene(*mFluidScene);
	//		mFluidScene = NULL;
	//	}

	//	if (mHWScene) {
	//		mHWScene->fetchResults(NX_RIGID_BODY_FINISHED,true);
	//		mPhysicsSDK->releaseScene(*mHWScene);
	//		mHWScene = NULL;
	//	}
	//	mIsExiting = false;

	//	Con::printf("!!!!!!!!!!!!!!!!!!!!!!! Releasing physx SDK!!!!!!!");
	//	mPhysicsSDK->release();
	//	mPhysicsSDK = NULL;
	//	//preTickListServer.remove( dynamic_cast<IProcessManager *>(this) );

	//}

	//if (mIsExiting){
	//	mIsReallyExiting = true;
	//	mIsExiting = false;
	//}
	//ODE
	//dWorldStep(mOdeWorld,mStepTime);
	//const dReal *p = dBodyGetPosition(mTestBody);
	//Con::errorf("stepping physics!!");

	//PROFILE_END();

}

//void nxPhysManager::destroy()
//{
	//FILE *fpw = fopen("debug.txt","w");
	//fprintf(fpw,"releasing physX!");
	//fclose(fpw);
	//Con::printf("!!!!!!!!!!!!!!!!!!!!!! Releasing PhysX SDK!!!!!!!!!!!!!!!!!!!!");

	//mPhysicsSDK->release();
//}
///////////////////////////////////////////////////////////////////////


fxRigidBody *global_fxRigidBodyFactory()
{
	fxRigidBody *fxRB = new fxRigidBody();
	return fxRB;
}


fxFlexBody *global_fxFlexBodyFactory()
{
	fxFlexBody *fxFB = new fxFlexBody();
	return fxFB;
}



void nxPhysManager::scriptInit()
{  //Here: for each script-exposed class, have a global function like this that gets
	//called at the end of physics manager init.  Seems like this one should be a phys
	//manager common or physManager thing though - or else a static function belonging
	//to fxRigidBody class.

	
	////////// script testing...
	int r;
	char msg[255];
	r = mScriptEngine->RegisterGlobalFunction("void numActors()",asFUNCTION(numActors), asCALL_CDECL); assert(r>=0);
	//sprintf(msg,"numActors, r: %d",r);  gConsole->addOutput(msg);
	r = mScriptEngine->RegisterGlobalFunction("void addActor()",asFUNCTION(addActor), asCALL_CDECL); assert(r>=0);
	//sprintf(msg,"AddActors, r: %d, engine: %d,  global function count %d",r, mScriptEngine,mScriptEngine->GetGlobalFunctionCount());
	//if (mScriptEngine==0)
	//	gConsole->addOutput("Script engine null!");
	//else 
	//	gConsole->addOutput(msg);
	//////// end script testing

	/////////////////////////
	//nxPhysManager
	r = mScriptEngine->RegisterObjectType("nxPhysManager", 0, asOBJ_REF | asOBJ_NOHANDLE); assert( r >= 0 );
	r = mScriptEngine->RegisterGlobalProperty("nxPhysManager pm", (void*)this); assert( r >= 0 );
	r = mScriptEngine->RegisterObjectMethod("nxPhysManager", "void dostuff()", asMETHOD(nxPhysManager, doStuff), asCALL_THISCALL); assert( r >= 0 );
	r = mScriptEngine->RegisterObjectMethod("nxPhysManager", "void setDR(bool)", asMETHOD(nxPhysManager, setDebugRender), asCALL_THISCALL); assert( r >= 0 );
	//r = mScriptEngine->RegisterObjectMethod("nxPhysManager", "void addTerrain(const Vector3 &pos, const Vector3 &extent)", asMETHOD(nxPhysManager, addTerrain), asCALL_THISCALL); assert( r >= 0 );
	r = mScriptEngine->RegisterObjectMethod("nxPhysManager", "void addTerrain()", asMETHOD(nxPhysManager, addTerrain), asCALL_THISCALL); assert( r >= 0 );

	/////////////////////////
	//fxRigidBody
	r = mScriptEngine->RegisterObjectType("fxRigidBody", 0, asOBJ_REF ); assert( r >= 0 );// | asOBJ_NOHANDLE  // 
	r = mScriptEngine->RegisterObjectBehaviour("fxRigidBody", asBEHAVE_FACTORY, "fxRigidBody @f()", asFUNCTION(global_fxRigidBodyFactory), asCALL_CDECL); assert( r >= 0 );
	r = mScriptEngine->RegisterObjectBehaviour("fxRigidBody", asBEHAVE_ADDREF, "void f()", asMETHOD(fxRigidBody, addRef),  asCALL_THISCALL); assert( r >= 0 );
	r = mScriptEngine->RegisterObjectBehaviour("fxRigidBody", asBEHAVE_RELEASE, "void f()", asMETHOD(fxRigidBody, release),  asCALL_THISCALL); assert( r >= 0 );
	r = mScriptEngine->RegisterObjectMethod("fxRigidBody", "int doStuff(int arg)", asMETHOD(fxRigidBody, doStuff), asCALL_THISCALL); assert( r >= 0 );
	r = mScriptEngine->RegisterObjectMethod("fxRigidBody", "Vector3 getPosition()", asMETHOD(fxRigidBody, getNodePosition), asCALL_THISCALL); assert( r >= 0 );
	r = mScriptEngine->RegisterObjectMethod("fxRigidBody", "void setPosition(const Vector3 &in)", asMETHOD(fxRigidBody, setNodePosition), asCALL_THISCALL); assert( r >= 0 );
			


	/////////////////////////
	//fxFlexBody
	r = mScriptEngine->RegisterObjectType("fxFlexBody", 0, asOBJ_REF ); assert( r >= 0 );// | asOBJ_NOHANDLE  // 
	r = mScriptEngine->RegisterObjectBehaviour("fxFlexBody", asBEHAVE_FACTORY, "fxFlexBody @f()", asFUNCTION(global_fxFlexBodyFactory), asCALL_CDECL); assert( r >= 0 );
	r = mScriptEngine->RegisterObjectBehaviour("fxFlexBody", asBEHAVE_ADDREF, "void f()", asMETHOD(fxFlexBody, addRef),  asCALL_THISCALL); assert( r >= 0 );
	r = mScriptEngine->RegisterObjectBehaviour("fxFlexBody", asBEHAVE_RELEASE, "void f()", asMETHOD(fxFlexBody, release),  asCALL_THISCALL); assert( r >= 0 );
	r = mScriptEngine->RegisterObjectMethod("fxFlexBody", "int dostuff(int arg)", asMETHOD(fxFlexBody, doStuff), asCALL_THISCALL); assert( r >= 0 );
	r = mScriptEngine->RegisterObjectMethod("fxFlexBody", "Vector3 getPosition()", asMETHOD(fxFlexBody, getNodePosition), asCALL_THISCALL); assert( r >= 0 );
	r = mScriptEngine->RegisterObjectMethod("fxFlexBody", "void setPosition(const Vector3 &in)", asMETHOD(fxFlexBody, setNodePosition), asCALL_THISCALL); assert( r >= 0 );
	r = mScriptEngine->RegisterGlobalProperty("fxFlexBody tweakerBot",&gTweakerBot); assert(r>=0);
	r = mScriptEngine->RegisterGlobalFunction("int getTweakerBot()",asFUNCTION(getTweakerBot), asCALL_CDECL); assert(r>=0);
	
	
	/////////////////////////
	//fxJoint

	/////////////////////////
	//fxFlexBodyPart

	/////////////////////////
	//fxFlexBody

	//Now, this might want to move somewhere else later, but we need to set up a material for my wireframe debug renders:

	Ogre::String matName = "EcstasyMotion/PhysicsWireFrame";
	mWireMat = Ogre::MaterialManager::getSingleton().getByName(matName);
	if (mWireMat.isNull())
	{
		mWireMat = Ogre::MaterialManager::getSingleton().create(matName, Ogre::ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME);
		Ogre::Pass* p = mWireMat->getTechnique(0)->getPass(0);
		p->setLightingEnabled(false );//Want my wireframe to be fullbright, not shaded
		//p->setPolygonModeOverrideable(false);
		//p->setVertexColourTracking(Ogre::TVC_AMBIENT);
		p->setPolygonMode(Ogre::PM_WIREFRAME);
		//p->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);//defaults to SET_REPLACE, (no transparency).
		//Other options: SBT_TRANSPARENT_COLOUR (darker = more transparent), SBT_ADD (add colors), SBT_MODULATE (multiply colors)
		p->setCullingMode(Ogre::CULL_NONE);//double sided 
		//p->setDepthWriteEnabled(false);
		//p->setDepthCheckEnabled(false);
		p->setDepthFunction(Ogre::CMPF_ALWAYS_PASS);
		p->setTransparentSortingForced(true);
	}

}


void nxPhysManager::doStuff()
{

	std::ostringstream os;
	os << "bodyList size:  " << mBodyList.size();
	gConsole->addOutput(os.str());


	//mDebugRender = true;

	//stopPhysics();

 //  mPhysicsSDK->setParameter( NX_VISUALIZATION_SCALE, 1.0f );
 //  ////gPhysicsSDK->setParameter( NX_VISUALIZE_BODY_MASS_AXES, 0.0f );
 //  //kPM->mPhysicsSDK->setParameter( NX_VISUALIZE_BODY_AXES, 1.0f );   
 //  mPhysicsSDK->setParameter( NX_VISUALIZE_COLLISION_SHAPES, 1.0f );

 //  const NxDebugRenderable *data = mScene->getDebugRenderable();  
 //  if ( !data )
 //     return;

	//NxU32 numPoints = data->getNbPoints();
	//NxU32 numLines = data->getNbLines();
	//const NxDebugLine *lines = data->getLines();
	//Ogre::Vector3 p0,p1;

	//os.str("");
	//os << "Num Lines: " << numLines;
	//gConsole->addOutput(os.str());

	////Okay, here we go:  make this into your mDebugRenderObject.
	//if (numLines)
	//{
	//	Ogre::ManualObject *mDR = Ogitors::OgitorsRoot::getSingleton().GetSceneManager()->createManualObject("DebugRenderObject");

	//	mDR->begin(mWireMat->getName());
	//	mDR->setDynamic(true);

	//	mDR->estimateVertexCount(numLines*2);
	//	mDR->estimateIndexCount(numLines*2);

	//	int v = 0;
	//	while ( numLines-- )
	//	{
	//		p0 = Ogre::Vector3(lines->p0.x,lines->p0.y,lines->p0.z);
	//		p1 = Ogre::Vector3(lines->p1.x,lines->p1.y,lines->p1.z);
	//		mDR->position(p0); mDR->colour(Ogre::ColourValue::Green); 
	//		mDR->position(p1); mDR->colour(Ogre::ColourValue::Green); 
	//		mDR->index(v); mDR->index(v+1); mDR->index(v);//three, to make triangles out of every three indices.  
	//		//Q: How do you just make points/lines?
	//		v+=2;
	//		lines++;
	//	}
	//	mDR->end();

	//	Ogitors::OgitorsRoot::getSingleton().GetSceneManager()->getRootSceneNode()->attachObject(mDR);
	//	
	//	Ogre::Vector3 pos = dynamic_cast<fxRigidBody*>(mBodyList[0]->getPhysUser())->mNode->getPosition();
	//	os << " body 0 node position: " << pos.x << ", " << pos.y << ", " << pos.z << 
	//		"mo name: " << mDR->getName() << ", scale x: " << mDR->getParentNode()->getScale().x;
	//	gConsole->addOutput(os.str());
	//	//mMeshPtr = mDR->convertToMesh(meshName, Ogre::ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME);


	//	//}
	//}

	//startPhysics();

	gConsole->addOutput("nxPhysManager did stuff!");
}

void nxPhysManager::debugRender()
{//Note: this ONLY gets called from within stepPhysics (make sure of that) so it doesn't need to stop/startPhysics.
   const NxDebugRenderable *data = mScene->getDebugRenderable();  
   if ( !data )
      return;

	NxU32 numLines = data->getNbLines();
	const NxDebugLine *lines = data->getLines();
	if (numLines==0)
		return;

	Ogre::Vector3 p0,p1;
	bool firstTime = false;

	if (!mDR) 
	{
		//mPhysicsSDK->setParameter( NX_VISUALIZATION_SCALE, 1.0f );
		//mPhysicsSDK->setParameter( NX_VISUALIZE_COLLISION_SHAPES, 1.0f );
		//gPhysicsSDK->setParameter( NX_VISUALIZE_BODY_MASS_AXES, 0.0f );
		//kPM->mPhysicsSDK->setParameter( NX_VISUALIZE_BODY_AXES, 1.0f );   

		mDR = Ogitors::OgitorsRoot::getSingleton().GetSceneManager()->createManualObject("DebugRenderObject");
		mDR->begin(mWireMat->getName());
		mDR->setDynamic(true);
		mDR->estimateVertexCount(numLines*3);//numLines * 2 is actual number of points, * 3 for a fifty percent growth buffer.
		mDR->estimateIndexCount(numLines*3);
		firstTime = true;
	} 	else {
		mDR->beginUpdate(0);
	}

	int v = 0;
	while ( numLines-- )
	{
		p0 = Ogre::Vector3(lines->p0.x,lines->p0.y,lines->p0.z);
		p1 = Ogre::Vector3(lines->p1.x,lines->p1.y,lines->p1.z);
		Ogre::ColourValue kColor;
		kColor.setAsARGB(lines->color);
		mDR->position(p0); mDR->colour(kColor);
		mDR->position(p1); mDR->colour(kColor);
		mDR->index(v); mDR->index(v+1); mDR->index(v);//three, to make triangles out of every three indices.  
		//Q: How do you just make points/lines?
		v+=2;
		lines++;
	}
	mDR->end();


	if (firstTime)
		Ogitors::OgitorsRoot::getSingleton().GetSceneManager()->getRootSceneNode()->attachObject(mDR);

}

//FIX!!  Use vetors instead!
#define MAX_TERRAIN_VERTS 40000
#define MAX_TERRAIN_FACES 120000
#define MAX_TERRAIN_MATERIALS 40000

//AngelScript Note:  WHY THE FUCK CAN I NOT ADD A FUCKING VECTOR ARGUMENT THIS TIME???? EXACTLY
//the same syntax as was used successfully in fxRigidBody::setPosition(const Ogre::Vector3 &pos) 
//FAILS here, script engine dead on arrival, WHAT THE FUCK?????

//void nxPhysManager::addTerrain(const Ogre::Vector2 &pos,const Ogre::Vector2 &extent)//FIX: this should be moved to physTerrainBlock.cpp
//void nxPhysManager::addTerrain(const Ogre::Vector3 &pos)//FIX: this should be moved to physTerrainBlock.cpp
void nxPhysManager::addTerrain()
{
	std::ostringstream os;

	float worldSize,squareSize;
	int terrainSize,stepX,stepY;

	Ogre::Vector3 pos(-100.0,0.0,-150.0);
	Ogre::Vector3 startPos,endPos,worldPos;
	Ogre::Vector2 extent(120.0,120.0);
	Ogre::Vector3 terrainStartPosF,terrainEndPosF;
	Ogre::Vector2 terrainStartPos,terrainEndPos;

	if (!mTerrain)
	{
		gConsole->addOutput("No terrain found.");
		return;
	}

	gConsole->addOutput("we have a terrain pointer!!");
	//myHeight = mTerrain->getHeightAtWorldPosition(pos.x,pos.y,pos.z);
	terrainSize = mTerrain->getSize();
	worldSize = mTerrain->getWorldSize();
	squareSize = worldSize / (terrainSize-1);

	if (extent.length() < squareSize) 
	{
		gConsole->addOutput("Active area is smaller than one terrain square.");
		return;
	}

	startPos = Ogre::Vector3(pos.x - (extent.x/2.0),0.0,pos.z - (extent.y/2.0));
	endPos = Ogre::Vector3(pos.x + (extent.x/2.0),0.0,pos.z + (extent.y/2.0));
	mTerrain->getTerrainPosition(Ogre::Vector3(startPos.x,0.0,startPos.z),&terrainStartPosF);
	mTerrain->getTerrainPosition(Ogre::Vector3(endPos.x,0.0,endPos.z),&terrainEndPosF);
	terrainStartPos = Ogre::Vector2((int)(terrainStartPosF.x * (float)terrainSize),(int)(terrainStartPosF.y * (float)terrainSize));
	terrainEndPos = Ogre::Vector2((int)(terrainEndPosF.x * (float)terrainSize),(int)(terrainEndPosF.y * (float)terrainSize));
	//myHeightStart = mTerrain->getHeightAtPoint((long)terrainStartPos.x,(long)terrainStartPos.y);
	//myHeightEnd = mTerrain->getHeightAtPoint((long)terrainEndPos.x,(long)terrainEndPos.y);

	os << "Adding Terrain,  " << ", squareSize " << squareSize << ", world size " << worldSize << ", terrain size " << terrainSize <<
		", terrain start pos: " << terrainStartPos.x << ", " << terrainStartPos.y << 
		", end pos: " << terrainEndPos.x << ", " << terrainEndPos.y ;
	gConsole->addOutput(os.str());

	if (terrainStartPos.x < terrainEndPos.x) 
		stepX = 1;
	else 
		stepX = -1;
	if (terrainStartPos.y < terrainEndPos.y)
		stepY = 1;
	else 
		stepY = -1;

	NxVec3* gTerrainVerts = new NxVec3[MAX_TERRAIN_VERTS];
	//NxU32* gTerrainFaces = new NxU32[(((x_blocks-1)*(y_blocks-1))*2)*3];//(num_tris * 3)
	NxU32* gTerrainFaces = new NxU32[MAX_TERRAIN_FACES];
	//NxMaterialIndex gTerrainMaterials[(((x_blocks-1)*(y_blocks-1))*2)];
	NxMaterialIndex gTerrainMaterials[MAX_TERRAIN_MATERIALS];

	int x_count=0;  int y_count=0; int v_count = 0;
	int y_blocks = (int)(fabs(float(terrainEndPos.y - terrainStartPos.y))) + 1;
	for (int i=terrainStartPos.x;i!=(terrainEndPos.x+stepX);i+=stepX)
	{
		y_count = 0;
		for (int j=terrainStartPos.y;j!=(terrainEndPos.y+stepY);j+=stepY)
		{
			//float height = mTerrain->getHeightAtPoint(i,j);
			mTerrain->getPoint(i,j,&worldPos);
			//if (v_count < 2) worldPos.y += 20.0;
			gTerrainVerts[(x_count*y_blocks)+(y_count)].set(worldPos.x,worldPos.y,worldPos.z);
			//os.str("");
			//os << i << " " << j << " world pos: " << worldPos.x << ", " << worldPos.y << ", " << 
			//	worldPos.z  ;
			//gConsole->addOutput(os.str());
			y_count++;
			v_count++;
		}
		x_count++;
	}

	os.str("");
	os << "x count: " << x_count << ", y count: " << y_count << ", y blocks: " << y_blocks ;
	gConsole->addOutput(os.str());

	//HERE:  This section is entirely different for Torque terrains - winding order alternates for each 
	//strip there, here it is the same for each strip.  Should have a switch for that.
	NxU32 k = 0;
	NxU32 m = 0;
	//int initialSplit = 1;
	for (int i=0;i<(x_count-1);i++) {
		for (int j=0;j<(y_blocks-1);j++) {
			if (((int)(terrainStartPos.y)+j)%2==0) {
				//upper left to lower right diagonal
				gTerrainFaces[k++] = (i * y_blocks) + j;
				gTerrainFaces[k++] = (i * y_blocks) + (j+1);
				gTerrainFaces[k++] = ((i+1) * y_blocks) + (j+1);

				gTerrainFaces[k++] = (i * y_blocks) + j;
				gTerrainFaces[k++] = ((i+1) * y_blocks) + (j+1);
				gTerrainFaces[k++] = ((i+1) * y_blocks) + j;

			} else {
				//lower left to upper right diagonal
				gTerrainFaces[k++] = (i * y_blocks) + j;
				gTerrainFaces[k++] = (i * y_blocks) + (j+1);
				gTerrainFaces[k++] = ((i+1) * y_blocks) + j;

				gTerrainFaces[k++] = (i * y_blocks) + (j+1);
				gTerrainFaces[k++] = ((i+1) * y_blocks) + (j+1);
				gTerrainFaces[k++] = ((i+1) * y_blocks) + j;
			}
		}
	}

	int num_verts = x_count * y_count;
	int num_tris = ((x_count-1) * (y_count-1) * 2);

	stopPhysics();


	NxTriangleMeshDesc terrainDesc;
	terrainDesc.numVertices = num_verts;
	terrainDesc.numTriangles = num_tris;
	terrainDesc.pointStrideBytes = sizeof(NxVec3);
	terrainDesc.triangleStrideBytes = 3 * sizeof(NxU32);
	terrainDesc.points = gTerrainVerts;
	terrainDesc.triangles = gTerrainFaces;

	if (mHWScene) terrainDesc.flags = NX_MF_HARDWARE_MESH ;
	else terrainDesc.flags = 0;

	terrainDesc.heightFieldVerticalAxis = NX_Y;
	terrainDesc.heightFieldVerticalExtent = -1000;
	//terrainDesc.materialIndexStride = sizeof(NxMaterialIndex);
	//terrainDesc.materialIndices = gTerrainMaterials;

	NxTriangleMeshShapeDesc terrainShapeDesc;

	bool status = NxCookTriangleMesh(terrainDesc, myUserStream("terrain.bin", false));
	terrainShapeDesc.meshData = mPhysicsSDK->createTriangleMesh(myUserStream("terrain.bin", true));		

	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&terrainShapeDesc);

	//physShapeData *userData = new physShapeData;
	//userData->mType = PHYS_TERRAIN;

	NxActor* actor;
	NxShape *shp;

	//if (mHWScene) {
	//	actor = mHWScene->createActor(actorDesc);
	//	actor->userData = (void*)-1;
	//	shp = *actor->getShapes();
	//	//shp->userData = userData;
	//	unsigned int pages = terrainShapeDesc.meshData->getPageCount();
	//	for (unsigned int p=0;p<pages;p++) ((NxTriangleMeshShape*)shp)->mapPageInstance(p);
	//}

	//That does it for hardware scene, next do fluid scene:
	//if (mFluidScene) {//TEMP -- problem here on restart mission.
	//if (0) {
		//actor = mFluidScene->createActor(actorDesc);
		//NxArray<NxTriangleMeshShape*> shapes;
		//NxTriangleMeshShape* mesh = shp->isTriangleMesh();
		//if (mesh) shapes.pushBack(mesh);
		//status = NxCookFluidHardwareMesh(REST_PARTICLES_PER_METER, KERNEL_RADIUS_MULTIPLIER, MOTION_LIMIT_MULTIPLIER, PACKET_SIZE_MULTIPLIER, shapes,myUserStream("terrain.bin", false));
		//mFluidScene->createFluidHardwareTriangleMesh(myUserStream("terrain.bin", true));
	//}

	//Then, do regular SW scene (could do with a bigger terrain mesh here).
	terrainDesc.flags = 0;
	status = NxCookTriangleMesh(terrainDesc, myUserStream("terrain.bin", false));
	terrainShapeDesc.meshData = mPhysicsSDK->createTriangleMesh(myUserStream("terrain.bin", true));		
	terrainShapeDesc.shapeFlags = NX_SF_FEATURE_INDICES | NX_SF_VISUALIZATION;


	NxActorDesc actorDesc2;
	actorDesc2.shapes.pushBack(&terrainShapeDesc);
	actor = mScene->createActor(actorDesc2);
	//actor->userData = (void*)-1;
	//shp = *actor->getShapes();
	//shp->userData = userData;

	//physShapeData *kUserData = new physShapeData;
	//kUserData->mEntityType = PHYS_TERRAIN;
	//kUserData->mPhysUser = NULL;
	//actor->userData = (void *)kUserData;

	startPhysics();


}


	//std::ostringstream os;
	//os.str("");
	//os << "color: " << lines->color;
	//gConsole->addOutput(os.str());


//void nxPhysManager::renderDebug()//SceneGraph *graph, const SceneState *state)
//{//well, here we go, have to make this a static function for the scenegraph thing to work.
//	 
//	//
//	//nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
//   if ( !(mScene) || !(mDebugRender) )
//      return;
//
//   //// We need the write lock to be able to request 
//   //// the NxDebugRenderable object.
//   //releaseWriteLock();
//
//   //// TODO: We need to expose the different types of visualization
//   //// options to script and add a GUI for toggling them!
//
//   mPhysicsSDK->setParameter( NX_VISUALIZATION_SCALE, 1.0f );
//   ////gPhysicsSDK->setParameter( NX_VISUALIZE_BODY_MASS_AXES, 0.0f );
//   //kPM->mPhysicsSDK->setParameter( NX_VISUALIZE_BODY_AXES, 1.0f );   
//   mPhysicsSDK->setParameter( NX_VISUALIZE_COLLISION_SHAPES, 1.0f );
//
//   const NxDebugRenderable *data = mScene->getDebugRenderable();  
//   if ( !data )
//      return;
//
//   //// Render points
//  // {
//  //    NxU32 numPoints = data->getNbPoints();
//  //    const NxDebugPoint *points = data->getPoints();
//
//  //    PrimBuild::begin( GFXPointList, numPoints );
//  //    
//  //    while ( numPoints-- )
//  //    {
//  //       PrimBuild::color( getDebugColor(points->color) );
//  //       PrimBuild::vertex3fv( &points->p.x );
//  //       points++;
//  //    }
//
//  //    PrimBuild::end();
//  // }
//
//   //// Render lines
//   {
//      NxU32 numLines = data->getNbLines();
//		NxU32 numPoints = data->getNbPoints();
//      const NxDebugLine *lines = data->getLines();
//
//		//std::ostringstream os;
//		//os << "Debug rendering, num lines: " << numLines << ", num points " << numPoints ;
//		//gConsole->addOutput(os.str());
//
//		//Ogre::ManualObject manObj;
//
//      //PrimBuild::begin( GFXLineList, numLines * 2 );
//
//      //while ( numLines-- )
//      //{
//      //   PrimBuild::color( getDebugColor( lines->color ) );
//      //   PrimBuild::vertex3fv( &lines->p0.x );
//      //   PrimBuild::vertex3fv( &lines->p1.x );
//      //   lines++;
//      //}
//
//      //PrimBuild::end();
//   }
//
//   //// Render triangles
//  // {
//  //    NxU32 numTris = data->getNbTriangles();
//  //    const NxDebugTriangle *triangles = data->getTriangles();
//
//  //    PrimBuild::begin( GFXTriangleList, numTris * 3 );
//  //    
//  //    while ( numTris-- )
//  //    {
//  //       PrimBuild::color( getDebugColor( triangles->color ) );
//  //       PrimBuild::vertex3fv( &triangles->p0.x );
//  //       PrimBuild::vertex3fv( &triangles->p1.x );
//  //       PrimBuild::vertex3fv( &triangles->p2.x );
//  //       triangles++;
//  //    }
//
//  //    PrimBuild::end();
//  // }
//}







///////////////////////////////////////////////////////////////////////

int getTweakerBot()
{
	return 4;
}


NxActor *gActor = 0;

void numActors()
{
	char engineresult[255];
	nxPhysManager *kPM = dynamic_cast<nxPhysManager*>(physManagerCommon::getPM());
	
	
	if (kPM->mScene)
	{
		NxScene *kScene = kPM->mScene;

		kScene->fetchResults(NX_RIGID_BODY_FINISHED,true);

		if (gActor)
		{
			NxVec3 nxPos = gActor->getGlobalPosition();
			//Ogre::Vector3 actorPos(nxPos.x,nxPos.y,nxPos.z);
			sprintf(engineresult,"Actor position: %f %f %f, numActors: %d, numRigidBodies %d",
				nxPos.x,nxPos.y,nxPos.z,kScene->getNbActors(),kPM->mBodyList.size());
		} else {
			sprintf(engineresult,"numActors: %d, numRigidBodies %d",kScene->getNbActors(),kPM->mBodyList.size());
		}
		kScene->simulate(kPM->mStepTime);

		//sprintf(engineresult,"At least I have a physics scene!  %d actors!",kPM->mScene->getNbActors());

	}
	else
		sprintf(engineresult,"I don't even have a physics scene.");

	if (gConsole) 
		gConsole->addOutput(engineresult);

	
	//mOgitorsRoot = &OgitorsRoot::getSingleton();
	if (gSQL)
	{
		if (gSQL->OpenDatabase("EcstasyMotion.db"))
		{
			char id_query[512];
			sqlite_resultset *resultSet;
			sprintf(id_query,"SELECT id, shapeFile, dimensions_x, dimensions_y, dimensions_z FROM fxRigidBodyData WHERE name = 'crate';");

			int result = gSQL->ExecuteSQL(id_query);
			if (result)
			{
				resultSet = gSQL->GetResultSet(result);
				if (resultSet<=0)
				{
					sprintf(engineresult,"database opened, resultset null!");
				} else {
					if (resultSet->iNumRows == 0)	
						sprintf(engineresult,"database opened, query failed!");
					else
						sprintf(engineresult,"database opened, crate: shape file %s, dimensions %f %f %f!!!!",
						resultSet->vRows[0]->vColumnValues[1],
						strtod(resultSet->vRows[0]->vColumnValues[2],NULL),
						strtod(resultSet->vRows[0]->vColumnValues[3],NULL),
						strtod(resultSet->vRows[0]->vColumnValues[4],NULL));//Rewrite as atoi(str)
				}
			} else {
				sprintf(engineresult,"database opened, resultset null!");
			}
			gSQL->CloseDatabase();
		} else {
			sprintf(engineresult,"database failed!");
		}
	} else {
		sprintf(engineresult,"don't even have gSQL!");
	}
	gConsole->addOutput(engineresult);

}


void addActor()
{
	
	NxActorDesc actorDesc;
	NxBodyDesc bodyDesc;
	NxBoxShapeDesc boxDesc;

	char msg[255];

	nxPhysManager *kPM = dynamic_cast<nxPhysManager*>(physManagerCommon::getPM());
	
	Ogitors::OgitorsScriptConsole *console = &(Ogitors::OgitorsScriptConsole::getSingleton());
	
	if (kPM->mScene)
	{
		nxRigidBody *box = new nxRigidBody();


		//NxScene *kScene = kPM->mScene;
		//kScene->fetchResults(NX_RIGID_BODY_FINISHED,true);

		//boxDesc.dimensions.set(NxVec3(1.0,1.0,1.0));
		//boxDesc.localPose.t = NxVec3(0.0,0.0,0.0);
		//actorDesc.shapes.pushBack(&boxDesc);

		//actorDesc.body = &bodyDesc;
		//actorDesc.density = 1.0;
		//actorDesc.globalPose.t = NxVec3(-100.0,200.0,-150.0);
		

		//if (actorDesc.isValid())
		//	gActor = kScene->createActor(actorDesc);

		sprintf(msg,"At least I have a physics scene!  %d actors!",kPM->mScene->getNbActors());
		//kScene->simulate(kPM->mStepTime);

	}
	else
		sprintf(msg,"I don't even have a physics scene.");

	//console->addOutput(msg);
}


///////////////////////////////////////////////////////////////////////















//////////////////////////////

/*
#define USE_HARDWARE 0


#include "console/simBase.h"
#include "T3D/gameBase/gameBase.h"
#include "T3D/gameBase/gameConnection.h"
#include "console/consoleTypes.h"

#include "T3D/physicsBAG/nxRigidBody.h"
#include "T3D/physicsBAG/nxJoint.h"
//#include "T3D/physicsBAG/nxFluid.h"
#include "T3D/physicsBAG/nxCloth.h"
#include "T3D/physicsBAG/nxSpringAndDamper.h"
#include "T3D/physicsBAG/fxFlexBody.h"
#include "T3D/physicsBAG/fxFlexBodyPart.h"
#include "T3D/physicsBAG/fxRigidBody.h"

#include "T3D/physicsBAG/physSpring.h"
#include "T3D/physicsBAG/iPhysUser.h"

#include "T3D/fx/explosion.h"

#include "ts/tsShape.h"
#include "ts/tsShapeInstance.h"

//#include "terrain/terrData.h"
#include "T3D/physicsBAG/physTerrainBlock.h"
#include "terrain/terrRender.h"
#include "gfx/gfxDrawUtil.h"
#include "gfx/sim/debugDraw.h"
#include "console/SQLiteObject.h"
#include "T3D/gameBase/gameProcess.h"

//#include "T3D/physicsBAG/physAtlasInstance2.h"
#include "T3D/physicsBAG/RenderClothExample.h"


#include "platform/profiler.h"

//#include "editor/terrainEditor.h"
#include "collision/ConcretePolyList.h"
//#include "console/simXMLDocument.h"
//#include "core/iProcessManager.h"

//class iProcessManager;

MRandomLCG pRandom;
int mouseValue = 0;
bool waitForReset = false;

extern float gTimeScale;
extern fxFlexBody *gTweakerOne;

extern Vector<IProcessManager *> preTickListClient;
extern Vector<IProcessManager *> preTickListServer;
extern Vector<IProcessManager *> postTickListClient;
extern Vector<IProcessManager *> postTickListServer;



//Ogre::Vector3 gPhysRaycastStart;
//Ogre::Vector3 gPhysRaycastEnd;

//HERE: this would be how you make a more complex struct for actor->userData
//struct actorData {
//   nxRigidBody *mRB;
//   fxFlexBody *mFB;
//   fxFlexBodyPart *mFBP;
//};
//First, though, see if you can just use the nxRigidBody * as itself.

////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
unsigned int ballTargetMS = 0;
unsigned int lastBallTargetMS = 0;

////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////



nxPhysManager::~nxPhysManager()
{
	//destroy();
}

void nxPhysManager::init()
{
	// Initialize PhysicsSDK
	mPhysicsSDK = NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION);
	if(!mPhysicsSDK) return;

	NxInitCooking();//Should this be done only once?

	//HMMM:  This should be updated to take into account ClientProcessList, ServerProcessList
	//preTickListClient.push_back( dynamic_cast<IProcessManager *>(this) );
	//if (!preTickListServer.contains(dynamic_cast<IProcessManager *>(this))
	if (preTickListServer.size()==0)//DANGER if I ever use preTickListServer for anything else,
	{ //this will break.
		preTickListServer.push_back( dynamic_cast<IProcessManager *>(this) );
		//if (!isServerObject()) 
		//ClientProcessList::get()->addObject(this);
		//else //WEIRD - this is always a serverObject, no matter which "side" I create it from.
		ServerProcessList::get()->addObject(this);
	}
	//mPhysicsSDK->setParameter(NX_MIN_SEPARATION_FOR_PENALTY, -0.05f);

	mPhysicsSDK->setParameter(NX_SKIN_WIDTH,0.01f);
	mPhysicsSDK->setParameter(NX_TRIGGER_TRIGGER_CALLBACK,true);

}


	////////////ODE TESTING  //////////////////
	//mOdeWorld = dWorldCreate();
	//if (mOdeWorld) Con::errorf("got an ode world!!");
	//dSimpleSpaceCreate(mOdeSpace);//(0, kCenter, kExtent, 6);
	//dWorldSetGravity (mOdeWorld, mDefaultGravity.x,mDefaultGravity.y,mDefaultGravity.z);
	//// Set up the contact joint group
	//mOdeContactGroup = dJointGroupCreate(0);
	//mTestBody = dBodyCreate(mOdeWorld);

	//if (mTestBody) {
	//	  Con::errorf("got a test body!!");
	//      dBodySetPosition (mTestBody, -36.0, -144.0, 278.0);
	//} else Con::errorf("body failed");


void nxPhysManager::destroyScene()
{

	//if (mSQL)
	//{
	//	mSQL->CloseDatabase();
	//	delete mSQL;
	//}

	if (mScene)
	{
		mScene->fetchResults(NX_RIGID_BODY_FINISHED,true);

		nxRigidBody* pBodyIterator = NULL;
		nxJoint* pJointIterator	 = NULL;
		nxSpringAndDamper* pSpringIterator = NULL;
		//nxFluid* pFluidIterator	 = NULL;
		RenderClothExample* pClothIterator	 = NULL;
		fxFlexBody* pFlexBodyIterator = NULL;

		while (mBodyList.size())
			removeRigidBody(mBodyList.first());
		while (mJointList.size())
			removeJoint(mJointList.first());
		while (mSpringList.size())
			removeSpring(mSpringList.first());
		while (mClothList.size())
			removeCloth(mClothList.first());
		while (mFlexBodyList.size())
			removeFlexBody(mFlexBodyList.first());

		if (mBodyReleaseList.size()>0) {
			pBodyIterator = NULL;
			for(	pBodyIterator = mBodyReleaseList.next(pBodyIterator); pBodyIterator;
				pBodyIterator = mBodyReleaseList.next(pBodyIterator))
			{
				if (pBodyIterator->mActor) mScene->releaseActor(*pBodyIterator->mActor);
			}
			mBodyReleaseList.reset();
		}

		if (mJointReleaseList.size()>0) {
			pJointIterator = NULL;
			for(	pJointIterator = mJointReleaseList.next(pJointIterator); pJointIterator;
				pJointIterator = mJointReleaseList.next(pJointIterator))
			{
				if (pJointIterator->mJoint) mScene->releaseJoint(*pJointIterator->mJoint);
			}
			mJointReleaseList.reset();
		}

		if (mSpringReleaseList.size()>0) {
			pSpringIterator = NULL;
			for(	pSpringIterator = mSpringReleaseList.next(pSpringIterator); pSpringIterator;
				pSpringIterator = mSpringReleaseList.next(pSpringIterator))
			{
				mScene->releaseEffector(*pSpringIterator->mEffector);
			}
			mSpringReleaseList.reset();
		}

		//if (mFluidReleaseList.size()>0) {
		//   pFluidIterator = NULL;
		//   for(	pFluidIterator = mFluidReleaseList.next(pFluidIterator); pFluidIterator;
		//      pFluidIterator = mFluidReleaseList.next(pFluidIterator))
		//   {
		//      //if (pFluidIterator->mFluid) mScene->releaseFluid(*pFluidIterator->mFluid);
		// if (pFluidIterator->mFluid) mHWScene->releaseFluid(*pFluidIterator->mFluid);
		//   }
		//   mFluidReleaseList.reset();
		//}

		if (mClothReleaseList.size()>0) {
			pClothIterator = NULL;
			for(	pClothIterator = mClothReleaseList.next(pClothIterator); pClothIterator;
				pClothIterator = mClothReleaseList.next(pClothIterator))
			{
				if (pClothIterator->mCloth) 
					mScene->releaseCloth(*pClothIterator->mCloth);

				if ( pClothIterator->mClothMesh  )
					mPhysicsSDK->releaseClothMesh( *(pClothIterator->mClothMesh) );

				pClothIterator->mCloth = NULL;
				pClothIterator->mClothMesh = NULL;
			}
			mClothReleaseList.reset();
		}

		if (mFlexBodyReleaseList.size()>0) {
			pFlexBodyIterator = NULL;
			for(	pFlexBodyIterator = mFlexBodyReleaseList.next(pFlexBodyIterator); pFlexBodyIterator;
				pFlexBodyIterator = mFlexBodyReleaseList.next(pFlexBodyIterator))
			{
				pFlexBodyIterator->releaseActors();
			}
			mFlexBodyReleaseList.reset();
		}

		mBodyList.reset();
		mJointList.reset();
		mSpringList.reset();
		//mFluidList.reset();
		mClothList.reset();
		mFlexBodyList.reset();

		mBodySetupList.reset();
		mJointSetupList.reset();
		mSpringSetupList.reset();
		//mFluidSetupList.reset();
		mClothSetupList.reset();
		mFlexBodySetupList.reset();


		mImpulseEventList.reset();//Danger: are we deleting the actual objects somewhere?
		mDurationEventList.reset();
		mInterpolationEventList.reset();
		mFollowEventList.reset();

		mCurrDurationEvents.clear();
		mCurrInterpolationEvents.clear();

		mSceneEventCounter = 0;
		mSceneStartStep = 0;
		mSceneStartTime = 0.0;
		mSceneDuration = 0.0;
		mLastImpulseEvent = NULL;
		mLastDurationEvent = NULL;
		mLastInterpolationEvent = NULL;
		mLastFollowEvent = NULL;


		mNumMaterials = 0;


		//mIsExiting = true;

		////Fuck it, going back to immediate release of everything, does this still wreck?

		mPhysicsSDK->releaseScene(*mScene);
		mScene = NULL;
	}
	mIsExiting = false;

	Con::printf("!!!!!!!!!!!!!!!!!!!!!!! Releasing physx SDK!!!!!!!");
	mPhysicsSDK->release();
	mPhysicsSDK = NULL;
}

void nxPhysManager::destroy()
{
	//FILE *fpw = fopen("debug.txt","w");
	//fprintf(fpw,"releasing physX!");
	//fclose(fpw);
	//Con::printf("!!!!!!!!!!!!!!!!!!!!!! Releasing PhysX SDK!!!!!!!!!!!!!!!!!!!!");

	//mPhysicsSDK->release();
}

bool nxPhysManager::onAdd()
{
	return true;
}


void nxPhysManager::processTick()
{
	if (mScene) stepPhysics();
}

void nxPhysManager::stopPhysics()
{
	mScene->fetchResults(NX_RIGID_BODY_FINISHED,true);
}

void nxPhysManager::startPhysics()
{
	mScene->simulate(mStepTime);
}

///////////////////////////////////////////////////////


void nxPhysManager::addRaycast(nxRaycastData rc)
{
	mRaycasts[mNumRaycasts].start = rc.start;
	mRaycasts[mNumRaycasts].dir = rc.dir;
	mRaycasts[mNumRaycasts].force = rc.force;
	mRaycasts[mNumRaycasts].damage = rc.damage;
	mRaycasts[mNumRaycasts].Dirt = rc.Dirt;
	mRaycasts[mNumRaycasts].Brick = rc.Brick;
	mRaycasts[mNumRaycasts].Water = rc.Water;
	mRaycasts[mNumRaycasts].Blood = rc.Blood;
	mRaycasts[mNumRaycasts].createStep = mCurrStep;

	mNumRaycasts++;
}




void nxPhysManager::removeRigidBodySetup(physRigidBody *pRB)
{
	mBodySetupList.unlink(dynamic_cast<nxRigidBody*> (pRB));
}

///////////////////////////////////////////////////////
void nxPhysManager::addJoint(physJoint *pJoint)
{
	mJointList.link(dynamic_cast<nxJoint*> (pJoint));
}



void nxPhysManager::addJointSetup(physJoint *pJoint)
{
	mJointSetupList.link(dynamic_cast<nxJoint*> (pJoint));
}

void nxPhysManager::removeJointSetup(physJoint *pJoint)
{
	mJointSetupList.unlink(dynamic_cast<nxJoint*> (pJoint));
}

///////////////////////////////////////////////////////
void nxPhysManager::addSpring(physSpring *pSpring)
{
	mSpringList.link(dynamic_cast<nxSpringAndDamper*> (pSpring));
}

void nxPhysManager::removeSpring(physSpring *pSpring)
{
	mSpringReleaseList.link(dynamic_cast<nxSpringAndDamper*> (pSpring));
	mSpringList.unlink(dynamic_cast<nxSpringAndDamper*> (pSpring));
}


void nxPhysManager::addSpringSetup(physSpring *pSpring)
{
	mSpringSetupList.link(dynamic_cast<nxSpringAndDamper*> (pSpring));
}

void nxPhysManager::removeSpringSetup(physSpring *pSpring)
{
	mSpringSetupList.unlink(dynamic_cast<nxSpringAndDamper*> (pSpring));
}

///////////////////////////////////////////////////////
//void nxPhysManager::addFluid(physFluid *p)
//{
//  mFluidList.link(dynamic_cast<nxFluid*> (p));
//}
//
//void nxPhysManager::removeFluid(physFluid *p)
//{
//  mFluidReleaseList.link(dynamic_cast<nxFluid*> (p));
//  mFluidList.unlink(dynamic_cast<nxFluid*> (p));
//}
//
//
//void nxPhysManager::addFluidSetup(physFluid *p)
//{
//  mFluidSetupList.link(dynamic_cast<nxFluid*> (p));
//}
//
//void nxPhysManager::removeFluidSetup(physFluid *p)
//{
//  mFluidSetupList.unlink(dynamic_cast<nxFluid*> (p));
//}

///////////////////////////////////////////////////////
void nxPhysManager::addCloth(RenderClothExample *p)
{
	mClothList.link(p);
}

void nxPhysManager::removeCloth(RenderClothExample *p)
{
	mClothReleaseList.link(p);
	mClothList.unlink(p);
}

void nxPhysManager::addClothSetup(RenderClothExample *p)
{
	mClothSetupList.link(p);
}

void nxPhysManager::removeClothSetup(RenderClothExample *p)
{
	mClothSetupList.unlink(p);
}
///////////////////////////////////////////////////////
void nxPhysManager::addFlexBody(fxFlexBody *p)
{
	mFlexBodyList.link(dynamic_cast<fxFlexBody*> (p));
}

void nxPhysManager::removeFlexBody(fxFlexBody *p)
{
	mFlexBodyReleaseList.link(dynamic_cast<fxFlexBody*> (p));
	mFlexBodyList.unlink(dynamic_cast<fxFlexBody*> (p));
}

void nxPhysManager::addFlexBodySetup(fxFlexBody *p)
{
	mFlexBodySetupList.link(dynamic_cast<fxFlexBody*> (p));
}

void nxPhysManager::removeFlexBodySetup(fxFlexBody *p)
{
	mFlexBodySetupList.unlink(dynamic_cast<fxFlexBody*> (p));
}
///////////////////////////////////////////////////////




physSpring *nxPhysManager::createSpring()
{
	nxSpringAndDamper *p = new nxSpringAndDamper();
	p->mPM = this;
	return (physSpring *)p;
}


//physFluid *nxPhysManager::createFluid()
//{
//   nxFluid *p = new nxFluid();
//   p->mPM = this;
//   p->mFluidScene = getHWScene();//getFluidScene()
//   return (physFluid *)p;
//}

physCloth *nxPhysManager::createCloth()
{
	nxCloth *p = new nxCloth();
	p->mPM = this;
	return (physCloth *)p;
}

///////////////////////////////////////////////////////


static ColorI getDebugColor( NxU32 packed )
{
   ColorI col;
   col.blue = (packed)&0xff;
   col.green = (packed>>8)&0xff;
   col.red = (packed>>16)&0xff;
   col.alpha = 255;

   return col;
}


//#define	SE_IMP_FORCE         1001 // Impulse force.
//#define	SE_IMP_TORQUE        1002 // Impulse torque.
//#define	SE_IMP_MOTOR_TARGET  1003 // Impulse motor target (pulse in this direction then ragdoll?)
//#define	SE_IMP_SET_FORCE     1004 // Constant force (until instructed otherwise).
//#define	SE_IMP_SET_TORQUE    1005 // Constant torque (until instructed otherwise).
//#define	SE_IMP_SET_MOTOR_TARGET  1006 // Set new motor target (until instructed otherwise).
//
//
////DURATION EVENTS start at 2000
//#define	SE_DUR_NULL          2000
//#define	SE_DUR_FORCE         2001 // Constant force for duration.
//#define	SE_DUR_TORQUE        2002 // Constant torque for duration.
//#define	SE_DUR_MOTOR_TARGET  2003 // Motor target for duration. 
//
//
////INTERPOLATION EVENTS start at 3000
//#define	SE_INTERP_NULL          3000
//#define	SE_INTERP_FORCE         3001 // Force interpolation. 
//#define	SE_INTERP_TORQUE        3002 // Torque interpolation.
//#define	SE_INTERP_MOTOR_TARGET  3003 // Motor target interpolation.


//void nxPhysManager::renderRaycast(SceneGraph *graph, const SceneState *state)
//{
//	PrimBuild::begin(GFXLineList,2);
//	PrimBuild::color( ColorI(0,255,0,255) );
//	PrimBuild::vertex3fv( gPhysRaycastStart );//Ogre::Vector3(gPhysRaycastStart.x,gPhysRaycastStart.y,gRaycastStart.z) );
//	PrimBuild::vertex3fv( gPhysRaycastEnd );// Ogre::Vector3(gPhysRaycastEnd.x,gPhysRaycastEnd.y,gRaycastEnd.z ));
//	PrimBuild::end();
//}

bool nxPhysManager::handleSceneEvent(ecstasySceneEvent *sceneEvent)
{
	//int eventID;//For looking up this event later from script	
	//int dbID;//database ID for this event
	//int eventType;
	//iPhysUser *physUser;//iPhysUser so we can have events for rigid bodies, flexbodies, cloth etc.

	//float time;//Time in seconds, from scene start.
	//float duration;//Duration in seconds.
	//int node;//Q: Index into bodyparts list on flexbody, or shape node index? Or depends on eventType?

	//ecstasySceneEvent *next;//Next event of this type, for this node on this body.
	//ecstasySceneEvent *prev;//In case we ever need to play a scene backwards.
	//ecstasySceneEvent *cause;//In case another event is the direct cause of this one, use this to determine force direction, magnitude etc.
	////Used only for types that require interpolation between events, not for impulses. 

	//Ogre::Vector3 value;
	//String action;

	fxFlexBody *kFB = dynamic_cast<fxFlexBody *>(sceneEvent->physUser);

	//Con::printf("thinking about handling scene event! dbID %d, eventID %d, actor %s",
	//	sceneEvent->dbID,sceneEvent->eventID,kFB->mActorName);

	if (!kFB) return false;//TEMP!  Need to be able to have truly global scene events, but right 
	//now we're getting a null flexbody for some reason when we shouldn't be.
	//NEXT: kFB needs to have a property telling me whether I am in the selection group or not.  
	//Only play scenes on selected bots.  Oh, but there's already isSelected() and isSelectedRecursive().
	if (!kFB->isSelectedRecursive()) return false;

	bool kExecAction = true;

	if (sceneEvent->eventType == SE_IMP_FORCE) {
		kFB->setBodypartForce(sceneEvent->node,sceneEvent->value);
	} else if (sceneEvent->eventType == SE_IMP_TORQUE) {
		kFB->setBodypartTorque(sceneEvent->node,sceneEvent->value);
	} else if (sceneEvent->eventType == SE_IMP_MOTOR_TARGET) {
		kFB->setBodypartMotorTarget(sceneEvent->node,sceneEvent->value);
	} else if ((sceneEvent->eventType == SE_IMP_SET_FORCE)||(sceneEvent->eventType == SE_DUR_FORCE)) {
		kFB->setBodypartForce(sceneEvent->node,sceneEvent->value);
	} else if ((sceneEvent->eventType == SE_IMP_SET_GLOBAL_FORCE)||(sceneEvent->eventType == SE_DUR_GLOBAL_FORCE)) {
		if (kFB->mIsAnimating) kFB->stopAnimating();//NEXT: get rid of this for local bodypart forces that don't cause full ragdoll.
		kFB->setBodypartGlobalForce(sceneEvent->node,sceneEvent->value);
	} else if ((sceneEvent->eventType == SE_IMP_SET_TORQUE)||(sceneEvent->eventType == SE_DUR_TORQUE)) {
		kFB->setBodypartTorque(sceneEvent->node,sceneEvent->value);
	} else if ((sceneEvent->eventType == SE_IMP_GLOBAL_TORQUE)||(sceneEvent->eventType == SE_DUR_GLOBAL_TORQUE)) {
		kFB->setBodypartGlobalTorque(sceneEvent->node,sceneEvent->value);
	} else if ((sceneEvent->eventType == SE_IMP_SET_MOTOR_TARGET)||(sceneEvent->eventType == SE_DUR_MOTOR_TARGET)) {
		kFB->setBodypartMotorTarget(sceneEvent->node,sceneEvent->value);
	} else if (sceneEvent->eventType == SE_IMP_RAGDOLL_FORCE) {
		Con::printf("handling ragdoll force event! id %d node %d force %f %f %f",
			sceneEvent->dbID,sceneEvent->node,sceneEvent->value.x,sceneEvent->value.y,sceneEvent->value.z);
		//return false;
		if (kFB->mIsAnimating) kFB->stopAnimating();
		kFB->setBodypartGlobalForce(sceneEvent->node,sceneEvent->value);
		
		return false;
	} else if (sceneEvent->eventType == SE_IMP_RAGDOLL) {
		if ((sceneEvent->node >= 0)&&(kFB->mBodyParts[sceneEvent->node]->mIsKinematic))
			kFB->clearBodypart(sceneEvent->node);
		else {
			if (kFB->mIsAnimating) kFB->stopAnimating();
			Con::errorf("Found a ragdoll scene event! Stopped animating.");
		}
	} else if ((sceneEvent->eventType == SE_IMP_KINEMATIC)||(sceneEvent->eventType == SE_DUR_KINEMATIC)) {
		if ((sceneEvent->node >= 0)&&(!kFB->mBodyParts[sceneEvent->node]->mIsKinematic))
			kFB->setBodypart(sceneEvent->node);
		else
			if (!kFB->mIsKinematic) kFB->setKinematic();	
	} else if (sceneEvent->eventType == SE_IMP_MOVE) {
		kFB->setPosition(sceneEvent->value);//setTransform, interpolate in advanceTime

	} else if (sceneEvent->eventType == SE_IMP_TURN) {
		//kFB->setRotation// setTransform, interpolate in advanceTime

	} else if (sceneEvent->eventType == SE_IMP_EXPLOSION_CAUSE) {
		char strValue1[255],strValue2[25];
		sprintf(strValue1,"%f %f %f",kFB->mCurrPosition.x,kFB->mCurrPosition.y,kFB->mCurrPosition.z);
		sprintf(strValue2,"%f",sceneEvent->value.len());
		Con::executef("playbotExplosion",strValue1,strValue2);

	} else if (sceneEvent->eventType == SE_IMP_WEAPON_CAUSE) {
		

		
	} else if (sceneEvent->eventType == SE_IMP_EXPLOSION_EFFECT) {
		
	} else if (sceneEvent->eventType == SE_IMP_WEAPON_EFFECT) {
		

	} else if ((sceneEvent->eventType == SE_IMP_MOTORIZE)||(sceneEvent->eventType == SE_DUR_MOTORIZE)) {
		kFB->motorize();

	} else if (sceneEvent->eventType == SE_IMP_MOVE_TO_POSITION) {
		Con::printf("found a scene event impulse moveToPosition: action %s, position %f %f %f",sceneEvent->action.c_str(),
			sceneEvent->value.x,sceneEvent->value.y,sceneEvent->value.z);
		String seqName;

		mSQL = new SQLiteObject();
		if (!mSQL) return true;
		if (mSQL->OpenDatabase("EcstasyMotion.db"))
		{
			char id_query[512],personaAction_id_query[512],insert_query[512];
			int body_id,joint_id,personaAction_id,result;
			sqlite_resultset *resultSet;

			//SELECT Customers.FirstName, Customers.LastName, SUM(Sales.SaleAmount) 
			//AS SalesPerCustomer
			//FROM Customers JOIN Sales ON Customers.CustomerID = Sales.CustomerID
			sprintf(personaAction_id_query,"SELECT id FROM personaAction \
										   WHERE persona_id = %d AND name = '%s';",
										   kFB->mPersonaId,sceneEvent->action.c_str());
			result = mSQL->ExecuteSQL(personaAction_id_query);
			resultSet = mSQL->GetResultSet(result);
			if (resultSet->iNumRows == 0)  //FIX: Instead of exiting here, check to see if action is a 
			{//simple sequence name that we have loaded.
				int seq = kFB->getShape()->findSequence(sceneEvent->action.c_str());
				if (seq == -1)
				{
					mSQL->CloseDatabase();
					return false; 
				} else {
					seqName = kFB->getShape()->getName(kFB->getShape()->sequences[seq].nameIndex);
				}
			} else {
				personaAction_id = dAtoi(resultSet->vRows[0]->vColumnValues[0]);
				sprintf(id_query,"SELECT name FROM sequence s \
								 JOIN personaActionSequence p \
								 ON p.sequence_id = s.id  \
								 WHERE p.persona_id = %d \
								 AND p.persona_action_id = %d \
								 AND p.skeleton_id = %d;",
								 kFB->mPersonaId,personaAction_id,kFB->mSkeletonId);
				//kFB->persona_id, kFB->skeleton_id, JOIN personaAction a ON personaAction.persona_id = kFB->persona_id ... AND a.
				result = mSQL->ExecuteSQL(id_query);
				resultSet = mSQL->GetResultSet(result);
				if (resultSet->iNumRows == 0) 
				{
					mSQL->CloseDatabase();
					return false;
				}
				seqName = resultSet->vRows[0]->vColumnValues[0];
			}

			kFB->setTarget(NULL);
			kFB->moveToPosition(sceneEvent->value,seqName);
			
			mSQL->CloseDatabase();
			delete mSQL;
			kExecAction = false;
		}
		///////////////////////////////////////////////////////////////
	}  else if (sceneEvent->eventType == SE_DUR_PLAY_SEQ) {
		kFB->setKinematic();
		kFB->playThread(0,sceneEvent->action);
	}	else if (sceneEvent->eventType == SE_DUR_ACTION_SEQ) { 
		//if (!kFB->mIsThinking)
		//{
			Con::errorf("found an action sequence: %s",sceneEvent->action);
			kFB->mActionUser->loadAction(sceneEvent->action);
			kFB->mIsThinking = true;
			kExecAction = false;
		//}
	}	else if (sceneEvent->eventType == SE_DUR_ACTION) {
		kFB->mActionUser->loadAction(sceneEvent->action);
		kFB->mIsThinking = true;
		kExecAction = false;
	} 

	char strValue[255];
	sprintf(strValue,"%f %f %f",sceneEvent->value.x,sceneEvent->value.y,sceneEvent->value.z);
	if (!sceneEvent->action.isEmpty()) 
		if (kExecAction)
			Con::executef(kFB,sceneEvent->action.c_str(),strValue);

	return true;
}

bool nxPhysManager::handleInterpolationSceneEvent(ecstasySceneEvent *sceneEvent,float delta)
{
	Ogre::Vector3 diff,currVal;
	fxFlexBody *kFB = dynamic_cast<fxFlexBody *>(sceneEvent->physUser);

	if ((!kFB)||(!sceneEvent->next)||(delta<0.0)||(delta>1.0))
		return false;

	if (!kFB->isSelectedRecursive()) return false;

	bool kExecAction = true;

	diff = sceneEvent->next->value - sceneEvent->value;
	currVal = sceneEvent->value + (diff * delta);
	//if (kFB->mIsAnimating) kFB->stopAnimating();//NEXT: get rid of this for local bodypart forces that don't cause full ragdoll.

	if (sceneEvent->eventType == SE_INTERP_FORCE) {
		kFB->setBodypartForce(sceneEvent->node,currVal);
	} else if (sceneEvent->eventType == SE_INTERP_TORQUE) {
		kFB->setBodypartTorque(sceneEvent->node,currVal);
	} else if (sceneEvent->eventType == SE_INTERP_MOTOR_TARGET) {
		kFB->setBodypartMotorTarget(sceneEvent->node,currVal);
	} else if (sceneEvent->eventType == SE_INTERP_GLOBAL_FORCE) {
		kFB->setBodypartGlobalForce(sceneEvent->node,currVal);
	} else if (sceneEvent->eventType == SE_INTERP_GLOBAL_TORQUE) {
		kFB->setBodypartGlobalTorque(sceneEvent->node,currVal);
	} else if (sceneEvent->eventType == SE_INTERP_MOVE) {
		//This is for just setting the position, not calling an action (move) animation
	} 
	
	//	if (!sceneEvent->action.isEmpty()) 
	//		Con::executef(sceneEvent->action.c_str(),currVal.x,currVal.y,currVal.z);
	//}

	char strValue[255];
	sprintf(strValue,"%f %f %f",currVal.x,currVal.y,currVal.z);
	if (!sceneEvent->action.isEmpty()) 
		if (kExecAction)
			Con::executef(sceneEvent->action.c_str(),strValue);
	
	return true;
}

bool nxPhysManager::handleFollowSceneEvent(ecstasySceneEvent *sceneEvent)
{
	fxFlexBody *kFB = dynamic_cast<fxFlexBody *>(sceneEvent->physUser);
	bool kExecAction = true;
	
	if (!kFB->isSelectedRecursive()) return false;

	if (sceneEvent->eventType == SE_FOLLOW_MOVE_TO_POSITION) 
	{
		//this is for getting to a position by calling a move animation
		Con::printf("found a scene event follow moveToPosition: action %s, position %f %f %f",sceneEvent->action.c_str(),
			sceneEvent->value.x,sceneEvent->value.y,sceneEvent->value.z);
		String seqName;
		mSQL = new SQLiteObject();
		if (!mSQL) return false;
		if (mSQL->OpenDatabase("EcstasyMotion.db"))
		{
			char id_query[512],personaAction_id_query[512],insert_query[512];
			int body_id,joint_id,personaAction_id,result;
			sqlite_resultset *resultSet;

			//SELECT Customers.FirstName, Customers.LastName, SUM(Sales.SaleAmount) 
			//AS SalesPerCustomer
			//FROM Customers JOIN Sales ON Customers.CustomerID = Sales.CustomerID
			sprintf(personaAction_id_query,"SELECT id FROM personaAction \
										   WHERE persona_id = %d AND name = '%s';",
										   kFB->mPersonaId,sceneEvent->action.c_str());
			result = mSQL->ExecuteSQL(personaAction_id_query);
			resultSet = mSQL->GetResultSet(result);
			if (resultSet->iNumRows == 0)  //FIX: Instead of exiting here, check to see if action is a 
			{//simple sequence name that we have loaded.
				int seq = kFB->getShape()->findSequence(sceneEvent->action.c_str());
				if (seq == -1)
				{
					mSQL->CloseDatabase();
					return false; 
				} else {
					seqName = kFB->getShape()->getName(kFB->getShape()->sequences[seq].nameIndex);
				}
			} else {
				personaAction_id = dAtoi(resultSet->vRows[0]->vColumnValues[0]);
				sprintf(id_query,"SELECT name FROM sequence s \
								 JOIN personaActionSequence p \
								 ON p.sequence_id = s.id  \
								 WHERE p.persona_id = %d \
								 AND p.persona_action_id = %d \
								 AND p.skeleton_id = %d;",
								 kFB->mPersonaId,personaAction_id,kFB->mSkeletonId);
				//kFB->persona_id, kFB->skeleton_id, JOIN personaAction a ON personaAction.persona_id = kFB->persona_id ... AND a.
				result = mSQL->ExecuteSQL(id_query);
				resultSet = mSQL->GetResultSet(result);
				if (resultSet->iNumRows == 0) 
				{
					Con::errorf("couldn't find sequence for persona action, exiting moveToPosition scene event");
					mSQL->CloseDatabase();
					return false;
				}
				seqName = resultSet->vRows[0]->vColumnValues[0];
			}

			kFB->moveToPosition(sceneEvent->value,seqName);

			mSQL->CloseDatabase();
			delete mSQL;
			kExecAction = false;
		}
	}
	char strValue[255];
	sprintf(strValue,"%f %f %f",sceneEvent->value.x,sceneEvent->value.y,sceneEvent->value.z);
	if (!sceneEvent->action.isEmpty()) 
		if (kExecAction)
			Con::executef(kFB,sceneEvent->action.c_str(),kFB->scriptThis(),strValue);
	
	kFB->mFollowEvent = sceneEvent;
	
}

//#define	SE_INTERP_FORCE         3001 // Force interpolation. 
//#define	SE_INTERP_TORQUE        3002 // Torque interpolation.
//#define	SE_INTERP_MOTOR_TARGET  3003 // Motor target interpolation.
//#define	SE_INTERP_MOVE          3004 // Interpolated move.
//#define	SE_INTERP_TURN          3005 // Interpolated turn.
//#define	SE_INTERP_GLOBAL_FORCE  3006 
//#define	SE_INTERP_GLOBAL_TORQUE 3007 
//#define	SE_INTERP_SCRIPT        3100 



#define MAX_TERRAIN_VERTS 40000
#define MAX_TERRAIN_FACES 120000
#define MAX_TERRAIN_MATERIALS 40000

float nxPhysManager::getTerrHeight(Point2F lamePos, Ogre::Vector3 *normal)
{
	TerrainBlock* pBlock = dynamic_cast<TerrainBlock*>(Sim::findObject("Terrain"));//FIX!! This requires you to name
	//the terrainblock "Terrain" or else it won't work. >:-\

	Ogre::Vector3 pos(lamePos.x, lamePos.y, 0);

	if (pBlock != NULL) 
	{
		Ogre::Vector3 terrPos = pos;
		pBlock->getWorldTransform().mulP(terrPos);
		terrPos.convolveInverse(pBlock->getScale());

		float height;
		bool res;

		if(normal)
			res = pBlock->getNormalAndHeight(Point2F(terrPos.x, terrPos.y), normal, &height);
		else
			res = pBlock->getHeight(Point2F(terrPos.x, terrPos.y), &height);

		if (res)
		{
			terrPos.z = height;
			terrPos.convolve(pBlock->getScale());
			pBlock->getTransform().mulP(terrPos);
		}

		return height;
	}
	else
		return 0;
}

bool nxPhysManager::gridToWorld(const Point2I & gPos, Ogre::Vector3 & wPos)
{
	TerrainBlock * terrain = dynamic_cast<TerrainBlock*>(Sim::findObject("Terrain"));
	if(terrain)
	{
		const Ogre::Matrix4 & mat = terrain->getTransform();
		Ogre::Vector3 origin;
		mat.getColumn(3, &origin);
		wPos.x = gPos.x * (float)terrain->getSquareSize() + origin.x;
		wPos.y = gPos.y * (float)terrain->getSquareSize() + origin.y;
		wPos.z = getTerrHeight(Point2F(wPos.x, wPos.y));

		return true;
	}

	return false;
}

//
//void nxPhysManager::debugRender()
//{
//
//	const NxDebugRenderable *dbgData = mScene->getDebugRenderable();
//
//
//	if (dbgData) {
//
//		//GFX->pushWorldMatrix();//?
//		//GFX->setWorldMatrix( Ogre::Matrix4::Identity );//?
//		//Ogre::Matrix4 world = GFX->getWorldMatrix();
//
//		GFXTexHandle mDebugTextureHandle;
//		GFXVertexBufferHandle<GFXVertexPT> mDebugVerts;
//		GFXPrimitiveBufferHandle           mDebugPrimitives;
//
//		mDebugTextureHandle.set("art/textures/debug_rigidbody.jpg",&GFXDefaultPersistentProfile,"debug_rigidbody");//fix this, can't we do "~/" 
//
//		NxU32 NbPoints = dbgData->getNbPoints();
//		NxU32 NbLines = dbgData->getNbLines();
//		NxU32 NbTris = dbgData->getNbTriangles();
//
//		unsigned int numVerts, numIndices;
//
//		unsigned int index = 0;
//		//unsigned int indices_index = 0;
//
//		numVerts = NbLines * 2;
//		numIndices = NbLines * 2;
//
//		mDebugVerts.set( GFX, numVerts, GFXBufferTypeStatic );
//		U16 *indices = new U16[ numIndices ];
//		GFXVertexPT *verts = new GFXVertexPT[ numVerts ];
//		GFXVertexPT *vert;
//
//		const NxDebugLine* Lines = dbgData->getLines();
//		while(NbLines--)
//		{
//			vert =  &verts[index];
//			vert->point.set(Lines->p0.x,Lines->p0.y,Lines->p0.z);
//			vert->texCoord.set(1.0,1.0);
//			indices[index] = index;
//			index++;
//
//			vert =  &verts[index];
//			vert->point.set(Lines->p1.x,Lines->p1.y,Lines->p1.z);
//			vert->texCoord.set(1.0,1.0);
//			indices[index] = index;
//			index++;
//
//			Lines++;
//		}
//
//
//		GFXVertexPT *vbVerts = mDebugVerts.lock();
//		dMemcpy( vbVerts, verts, sizeof(GFXVertexPT) * numVerts );
//		mDebugVerts.unlock();
//
//
//		GFXPrimitive pInfo;
//		U16 *ibIndices;
//		GFXPrimitive *piInput;
//
//		pInfo.type = GFXLineList;
//		pInfo.numPrimitives = dbgData->getNbLines();
//		pInfo.startIndex = 0;
//		pInfo.minIndex = 0;
//		pInfo.numVertices = numVerts;
//		//GFX->setFillMode(GFXFillWireframe);
//
//		mDebugPrimitives.set( GFX, numIndices, 1, GFXBufferTypeStatic );
//		mDebugPrimitives.lock( &ibIndices, &piInput );
//		dMemcpy( ibIndices, indices, numIndices * sizeof(U16) );
//		dMemcpy( piInput, &pInfo, sizeof(GFXPrimitive) );
//		mDebugPrimitives.unlock();
//
//		delete [] verts;
//		delete [] indices;
//
//		//GFX->pushWorldMatrix();
//		GFX->setTexture(0,mDebugTextureHandle);
//		//GFX->setTextureStageColorOp(0, GFXTOPModulate);
//		//GFX->setTextureStageColorOp(1, GFXTOPDisable);
//
//		GFX->setVertexBuffer( mDebugVerts );
//		GFX->setPrimitiveBuffer( mDebugPrimitives );
//		GFX->drawPrimitives();
//
//		//GFX->popWorldMatrix();//?
//
//		//GFX->setFillMode(GFXFillSolid);
//
//
//	}
//}

void nxPhysManager::addTerrain()//FIX: this should be moved to physTerrainBlock.cpp
{
	unsigned int s,num_verts,num_tris,x_blocks,y_blocks,x_start,y_start;
	int i,j;

	//TerrainBlock *terrBlock = dynamic_cast<TerrainBlock*>(Sim::findObject("Terrain"));
	physTerrainBlock *terrBlock;
	terrBlock = NULL;
	for (SimSetIterator obj(Sim::getRootGroup()); *obj; ++obj)
	{
		ConsoleObject* nobj = dynamic_cast<ConsoleObject*>(*obj);
		if (nobj) 
		{
			if (!dStrcmp(nobj->getClassName(),"physTerrainBlock"))
			{
				terrBlock = dynamic_cast<physTerrainBlock*>(nobj);
			}
		}
	}

	if (!terrBlock) {
		Con::errorf("Can't find a terrain block!");
		return;
	}

	float terrHeight;

	if (terrBlock->mPhysExtent.len()==0.0) return;
	terrBlock->setupDebugRender();

	int squareSize = terrBlock->getSquareSize();
	x_start = (int)(terrBlock->mPhysStart.x/(float)squareSize) + 128; 
	y_start = (int)(terrBlock->mPhysStart.y/(float)squareSize) + 128;
	x_blocks = (int)(terrBlock->mPhysExtent.x/(float)squareSize); 
	y_blocks = (int)(terrBlock->mPhysExtent.y/(float)squareSize);

	num_verts = x_blocks * y_blocks;
	num_tris = ((x_blocks-1) * (y_blocks-1) * 2);

	//NxVec3* gTerrainVerts = new NxVec3[x_blocks * y_blocks];
	NxVec3* gTerrainVerts = new NxVec3[MAX_TERRAIN_VERTS];
	//NxU32* gTerrainFaces = new NxU32[(((x_blocks-1)*(y_blocks-1))*2)*3];//(num_tris * 3)
	NxU32* gTerrainFaces = new NxU32[MAX_TERRAIN_FACES];
	//NxMaterialIndex gTerrainMaterials[(((x_blocks-1)*(y_blocks-1))*2)];
	NxMaterialIndex gTerrainMaterials[MAX_TERRAIN_MATERIALS];

	Point2F xy(0,0);


	//Picking an arbitrary subset of the terrain points: (344.0,0.0)-(520.0,224.0)
	//total of 23 points on the X axis, 29 on the Y axis.
	//in 2D integers, these points range from (171,128)-(193,156)
	//Con::printf("Terrain block size: %d",s);
	//Point2I terrStart(134,128);
	//Point2I terrStart(80,80);
	//Point2I terrStart(x_start,y_start);

	//Well, this is cheap but I'm in a hurry.. -- but, DAMN, they removed GridSquare in T3D... ?
	//GridSquare *gs;//need to make sure tesselation starts on the 
	unsigned int initialSplit = 0;//right foot, after that you can alternate.
	//gs = terrBlock->findSquare(0, Point2I(x_start,y_start));
	//if(gs->flags & GridSquare::Split45) initialSplit = 0;
	//else initialSplit = 1;

	for (i=x_start;i<(x_start+x_blocks);i++) {
		for (j=y_start;j<(y_start+y_blocks);j++) {
			xy.set((i*8.0)-1024.0,(j*8.0)-1024.0);//8.0 = terrain square size

			Ogre::Vector3 worldCoords;
			gridToWorld(Point2I(i,j),worldCoords);
			terrHeight = worldCoords.z + terrBlock->getPosition().z;

			gTerrainVerts[((i-x_start)*y_blocks)+(j-y_start)].set(xy.x,xy.y,terrHeight);
		}
	}

	 // whoops, terrain materials done all different in T3D, get this part later 05-30-09
	//U8 LR_Alphas[terrBlock->MaterialGroups];//lower right
	//U8 LL_Alphas[terrBlock->MaterialGroups];//lower left
	//U8 UL_Alphas[terrBlock->MaterialGroups];//upper left
	//U8 UR_Alphas[terrBlock->MaterialGroups];//upper right
	//U16 A_Alphas[terrBlock->MaterialGroups];//triangle A
	//U16 B_Alphas[terrBlock->MaterialGroups];//triangle B
	//U16 maxA, maxB;
	//U8 indexA,indexB;
	//U8 matA, matB;
	//char *matNames[terrBlock->MaterialGroups];
	//S16 matIndices[terrBlock->MaterialGroups];

	//for (unsigned int i=0;i<terrBlock->MaterialGroups;i++) 
	//{
	//matNames[i] = (char *)terrBlock->mMaterialFileName[i].c_str();
	//char *noPath;
	//if (matNames[i]) 
	//{
	//noPath = dStrrchr(matNames[i],'/');
	//noPath++;
	//matNames[i] = noPath;
	//}
	//}

	//SimSet physMats;
	//fxPhysMaterial tempPhysMats[MAX_MATERIALS];
	//unsigned int numMats = 0;
	//for (SimSetIterator obj(Sim::getRootGroup()); *obj; ++obj)
	//{
	//ConsoleObject* nobj = dynamic_cast<ConsoleObject*>(*obj);
	//if (nobj) 
	//{
	//if (!dStrcmp(nobj->getClassName(),"fxPhysMaterial"))
	//{
	//tempPhysMats[numMats].copy((fxPhysMaterial *)nobj);
	//physMats.addObject((SimObject *)&tempPhysMats[numMats]);
	//numMats++;
	//}
	//}
	//}

	//for (unsigned int i=0;i<terrBlock->MaterialGroups;i++) 
	//{
	//if (matNames[i]) {
	//SimSet::iterator m;
	//for (m=physMats.begin();m!=physMats.end();m++) 
	//{
	//fxPhysMaterial *myMat = (fxPhysMaterial *)(*m);
	//if (!dStrcmp(matNames[i],myMat->mTextureName))
	//{
	//matIndices[i] = myMat->mIndex;
	//break;
	//} else matIndices[i] = 1;
	//}
	//}
	//}
	
	NxU32 k = 0;
	NxU32 m = 0;
	for (i=0;i<(x_blocks-1);i++) {
		for (j=0;j<(y_blocks-1);j++) {
			
			//for (unsigned int a=0;a<terrBlock->MaterialGroups;a++)
			//{
			//UL_Alphas[a] = 0;
			//UR_Alphas[a] = 0;
			//LL_Alphas[a] = 0;
			//LR_Alphas[a] = 0;
			//A_Alphas[a] = 0;
			//B_Alphas[a] = 0;
			//}

			//terrBlock->getMaterialAlpha(Point2I(i+x_start,j+y_start+1),UL_Alphas);
			//terrBlock->getMaterialAlpha(Point2I(i+x_start+1,j+y_start+1),UR_Alphas);
			//terrBlock->getMaterialAlpha(Point2I(i+x_start,j+y_start),LL_Alphas);
			//terrBlock->getMaterialAlpha(Point2I(i+x_start+1,j+y_start),LR_Alphas);
			

			//FIX:  this is entirely dependent on where you start the section.  Need to look at the terrain block 
			//property that tells you which way we are tesselated.  Variable name? GridSquare::split45
			if (initialSplit==0)
			{//FIX
				if (((i%2==0)&&(j%2==0))||((i%2==1)&&(j%2==1))) {
					//upper left to lower right diagonal

					gTerrainFaces[k++] = (i * y_blocks) + j;
					gTerrainFaces[k++] = ((i+1) * y_blocks) + (j+1);
					gTerrainFaces[k++] = (i * y_blocks) + (j+1);

					gTerrainFaces[k++] = (i * y_blocks) + j;
					gTerrainFaces[k++] = ((i+1) * y_blocks) + j;
					gTerrainFaces[k++] = ((i+1) * y_blocks) + (j+1);

					//HERE: look up the textures involved, if foundMat, then use an average of 
					//the values for whatever materials are blended in this triangle.


					//for (unsigned int k=0;k<terrBlock->MaterialGroups;k++)
					//{
					//	A_Alphas[k] = UL_Alphas[k] + LR_Alphas[k] + LL_Alphas[k];
					//	B_Alphas[k] = UL_Alphas[k] + UR_Alphas[k] + LR_Alphas[k];
					//}

					//maxA = 0; maxB = 0;

					//for (unsigned int k=0;k<terrBlock->MaterialGroups;k++)
					//{
					//	if (A_Alphas[k]>maxA) { maxA = A_Alphas[k]; indexA = k; }
					//	if (B_Alphas[k]>maxB) { maxB = B_Alphas[k]; indexB = k; }
					//}
					//so, that should do it... except now I need to find the right material for whatever 
					//textures are in slots "indexA" and "indexB".  Need to get the texture filename,
					//and then find the PhysMaterial datablock that refers to that.  Tomorrow.

					//gTerrainMaterials[m++] = matIndices[indexA];
					//gTerrainMaterials[m++] = matIndices[indexB];


				} else {
					//lower left to upper right diagonal

					gTerrainFaces[k++] = (i * y_blocks) + j;
					gTerrainFaces[k++] = ((i+1) * y_blocks) + j;
					gTerrainFaces[k++] = (i * y_blocks) + (j+1);

					gTerrainFaces[k++] = (i * y_blocks) + (j+1);
					gTerrainFaces[k++] = ((i+1) * y_blocks) + j;
					gTerrainFaces[k++] = ((i+1) * y_blocks) + (j+1);

					//materials section			  

					//for (unsigned int k=0;k<terrBlock->MaterialGroups;k++)
					//{
					//	A_Alphas[k] = UL_Alphas[k] + UR_Alphas[k] + LL_Alphas[k];
					//	B_Alphas[k] = UR_Alphas[k] + LR_Alphas[k] + LL_Alphas[k];
					//}

					//maxA = 0; maxB = 0;

					//for (unsigned int k=0;k<terrBlock->MaterialGroups;k++)
					//{
					//	if (A_Alphas[k]>maxA) { maxA = A_Alphas[k]; indexA = k; }
					//	if (B_Alphas[k]>maxB) { maxB = B_Alphas[k]; indexB = k; }
					//}

					//gTerrainMaterials[m++] = matIndices[indexA];
					//gTerrainMaterials[m++] = matIndices[indexB];

				}
			} else {//initialSplit==1
				if (((i%2==0)&&(j%2==0))||((i%2==1)&&(j%2==1))) {
					//lower left to upper right diagonal

					gTerrainFaces[k++] = (i * y_blocks) + j;
					gTerrainFaces[k++] = ((i+1) * y_blocks) + j;
					gTerrainFaces[k++] = (i * y_blocks) + (j+1);

					gTerrainFaces[k++] = (i * y_blocks) + (j+1);
					gTerrainFaces[k++] = ((i+1) * y_blocks) + j;
					gTerrainFaces[k++] = ((i+1) * y_blocks) + (j+1);

					//materials section			  

					//for (unsigned int k=0;k<terrBlock->MaterialGroups;k++)
					//{
					//	A_Alphas[k] = UL_Alphas[k] + UR_Alphas[k] + LL_Alphas[k];
					//	B_Alphas[k] = UR_Alphas[k] + LR_Alphas[k] + LL_Alphas[k];
					//}

					//maxA = 0; maxB = 0;

					//for (unsigned int k=0;k<terrBlock->MaterialGroups;k++)
					//{
					//	if (A_Alphas[k]>maxA) { maxA = A_Alphas[k]; indexA = k; }
					//	if (B_Alphas[k]>maxB) { maxB = B_Alphas[k]; indexB = k; }
					//}

					//gTerrainMaterials[m++] = matIndices[indexA];
					//gTerrainMaterials[m++] = matIndices[indexB];

				} else {

					//upper left to lower right diagonal

					gTerrainFaces[k++] = (i * y_blocks) + j;
					gTerrainFaces[k++] = ((i+1) * y_blocks) + (j+1);
					gTerrainFaces[k++] = (i * y_blocks) + (j+1);

					gTerrainFaces[k++] = (i * y_blocks) + j;
					gTerrainFaces[k++] = ((i+1) * y_blocks) + j;
					gTerrainFaces[k++] = ((i+1) * y_blocks) + (j+1);

					//HERE: look up the textures involved, if foundMat, then use an average of 
					//the values for whatever materials are blended in this triangle.


					//for (unsigned int k=0;k<terrBlock->MaterialGroups;k++)
					//{
					//	A_Alphas[k] = UL_Alphas[k] + LR_Alphas[k] + LL_Alphas[k];
					//	B_Alphas[k] = UL_Alphas[k] + UR_Alphas[k] + LR_Alphas[k];
					//}

					//maxA = 0; maxB = 0;

					//for (unsigned int k=0;k<terrBlock->MaterialGroups;k++)
					//{
					//	if (A_Alphas[k]>maxA) { maxA = A_Alphas[k]; indexA = k; }
					//	if (B_Alphas[k]>maxB) { maxB = B_Alphas[k]; indexB = k; }
					//}
					//so, that should do it... except now I need to find the right material for whatever 
					//textures are in slots "indexA" and "indexB".  Need to get the texture filename,
					//and then find the PhysMaterial datablock that refers to that.  Tomorrow.

					//gTerrainMaterials[m++] = matIndices[indexA];
					//gTerrainMaterials[m++] = matIndices[indexB];

				}

			}
		}
	}
	//Con::printf("num terrain face indices: %d  num tris: %d, num_verts: %d",k,num_tris,num_verts);
	//for (i=0;i<k;i++) Con::printf("terrain faces[%d] = %d",i,gTerrainFaces[i]);
	//for (i=0;i<num_verts;i++) Con::printf("terrain verts[%d] = (%f,%f,%f)",i,gTerrainVerts[i].x,gTerrainVerts[i].y,gTerrainVerts[i].z);

	NxTriangleMeshDesc terrainDesc;
	terrainDesc.numVertices = num_verts;
	terrainDesc.numTriangles = num_tris;
	terrainDesc.pointStrideBytes = sizeof(NxVec3);
	terrainDesc.triangleStrideBytes = 3 * sizeof(NxU32);
	terrainDesc.points = gTerrainVerts;
	terrainDesc.triangles = gTerrainFaces;

	if (mHWScene) terrainDesc.flags = NX_MF_HARDWARE_MESH ;
	else terrainDesc.flags = 0;

	terrainDesc.heightFieldVerticalAxis = NX_Z;
	terrainDesc.heightFieldVerticalExtent = -1000;
	terrainDesc.materialIndexStride = sizeof(NxMaterialIndex);
	terrainDesc.materialIndices = gTerrainMaterials;

	mScene->fetchResults(NX_RIGID_BODY_FINISHED,true);
	if (mHWScene) mHWScene->fetchResults(NX_RIGID_BODY_FINISHED,true);
	//if (mFluidScene) mFluidScene->fetchResults(NX_RIGID_BODY_FINISHED,true);

	NxTriangleMeshShapeDesc terrainShapeDesc;
	//
	//  Wuh-oh, user stream is borked, 1.8 port -- FIXED - 05-30-09
	NxInitCooking();

	bool status = NxCookTriangleMesh(terrainDesc, myUserStream("terrain.bin", false));
	terrainShapeDesc.meshData = mPhysicsSDK->createTriangleMesh(myUserStream("terrain.bin", true));		

	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&terrainShapeDesc);

	//physShapeData *userData = new physShapeData;
	//userData->mType = PHYS_TERRAIN;

	NxActor* actor;
	NxShape *shp;

	if (mHWScene) {
		actor = mHWScene->createActor(actorDesc);
		actor->userData = (void*)-1;
		shp = *actor->getShapes();
		//shp->userData = userData;
		unsigned int pages = terrainShapeDesc.meshData->getPageCount();
		for (unsigned int p=0;p<pages;p++) ((NxTriangleMeshShape*)shp)->mapPageInstance(p);
	}

	//That does it for hardware scene, next do fluid scene:
	//if (mFluidScene) {//TEMP -- problem here on restart mission.
	if (0) {
		//actor = mFluidScene->createActor(actorDesc);
		//NxArray<NxTriangleMeshShape*> shapes;
		//NxTriangleMeshShape* mesh = shp->isTriangleMesh();
		//if (mesh) shapes.pushBack(mesh);
		//status = NxCookFluidHardwareMesh(REST_PARTICLES_PER_METER, KERNEL_RADIUS_MULTIPLIER, MOTION_LIMIT_MULTIPLIER, PACKET_SIZE_MULTIPLIER, shapes,myUserStream("terrain.bin", false));
		//mFluidScene->createFluidHardwareTriangleMesh(myUserStream("terrain.bin", true));
	}

	//Then, do regular SW scene (could do with a bigger terrain mesh here).
	terrainDesc.flags = 0;
	status = NxCookTriangleMesh(terrainDesc, myUserStream("terrain.bin", false));
	terrainShapeDesc.meshData = mPhysicsSDK->createTriangleMesh(myUserStream("terrain.bin", true));		
	terrainShapeDesc.shapeFlags = NX_SF_FEATURE_INDICES | NX_SF_VISUALIZATION;


	NxActorDesc actorDesc2;
	actorDesc2.shapes.pushBack(&terrainShapeDesc);
	actor = mScene->createActor(actorDesc2);
	//actor->userData = (void*)-1;
	//shp = *actor->getShapes();
	//shp->userData = userData;

	physShapeData *kUserData = new physShapeData;
	kUserData->mEntityType = PHYS_TERRAIN;
	kUserData->mPhysUser = NULL;
	actor->userData = (void *)kUserData;


	mScene->simulate(mStepTime);
	if (mHWScene) mHWScene->simulate(mStepTime);
	//if (mFluidScene) mFluidScene->simulate(mStepTime);

	//while (physMats.size() > 0)
	//	physMats.removeObject(*(physMats.begin()));
	//
}
//
//void nxPhysManager::addAtlasTerrain()
//{
//	unsigned int s,num_verts,num_indices,num_tris;
//	int i,j,vc,fc,x_start,y_start,x_blocks,y_blocks;
//	//s = TerrainBlock::BlockSize;
//	vc = 0; fc = 0;
//
//	physAtlasInstance2 *atlasInstance;// = dynamic_cast<AtlasInstance2*>(Sim::findObject("NewTerrain"));//WHOOPS! 
//	// (Can't be naming the particular terrain instance here -- need to find any atlasInstance2 instead.)
//	atlasInstance = NULL;
//	for (SimSetIterator obj(Sim::getRootGroup()); *obj; ++obj)
//	{
//		ConsoleObject* nobj = dynamic_cast<ConsoleObject*>(*obj);
//		if (nobj) 
//		{
//			if (!dStrcmp(nobj->getClassName(),"physAtlasInstance2"))
//			{
//				//Con::errorf("found my atlas terrain block!");
//				atlasInstance = dynamic_cast<physAtlasInstance2*>(nobj);
//			}
//		}
//	}
//
//	if (!atlasInstance) return;
//
//	if (atlasInstance->mPhysExtent.len()==0.0) return;
//	atlasInstance->setupDebugRender();//ERROR: this is a server object.  I need to set up the client object.
//	//atlasInstance->getGhostId
//
//	x_start = atlasInstance->mPhysStart.x; 
//	y_start = atlasInstance->mPhysStart.y;
//	x_blocks = atlasInstance->mPhysExtent.x; 
//	y_blocks = atlasInstance->mPhysExtent.y;
//
//	ConcretePolyList*	kPolyList	= new ConcretePolyList;
//	SphereF kBoundingSphere(Ogre::Vector3(0.0f, 0.0f, 0.0f), 2500.0f);
//	Box3F kBBox;
//	kBBox.minExtents.set(x_start,y_start,-500);
//	kBBox.maxExtents.set(x_start + x_blocks,y_start + y_blocks,500);
//	atlasInstance->buildPolyList(kPolyList, kBBox, kBoundingSphere);
//
//
//	num_verts = kPolyList->mVertexList.size();
//	num_indices = kPolyList->mIndexList.size();
//	num_tris = num_indices/3;
//
//
//	NxVec3* gTerrainVerts = new NxVec3[num_verts];
//	NxVec3* tempVerts = new NxVec3[num_verts];
//	NxU32* gTerrainFaces = new NxU32[num_indices];//(num_tris * 3)
//	NxU32* tempFaces = new NxU32[num_indices];//(num_tris * 3)
//
//	for (unsigned int i = 0; i < num_verts; i++)
//		gTerrainVerts[vc++] = NxVec3(kPolyList->mVertexList[i].x,kPolyList->mVertexList[i].y,kPolyList->mVertexList[i].z);
//
//	for (unsigned int i = 0; i < num_indices; i++)
//		gTerrainFaces[fc++] = kPolyList->mIndexList[num_indices-(i+1)];
//	//backwards winding, I believe -- try this the normal way if this doesn't work.
//
//	NxMaterialIndex gTerrainMaterials[10000];
//	for (unsigned int i=0;i<num_tris;i++) gTerrainMaterials[i] = 1;
//
//
//
//	NxTriangleMeshDesc terrainDesc;
//	terrainDesc.numVertices = num_verts;
//	terrainDesc.numTriangles = num_tris;
//	terrainDesc.pointStrideBytes = sizeof(NxVec3);
//	terrainDesc.triangleStrideBytes = 3 * sizeof(NxU32);
//	terrainDesc.points = gTerrainVerts;
//	terrainDesc.triangles = gTerrainFaces;
//
//	if (mHWScene) terrainDesc.flags = NX_MF_HARDWARE_MESH; 
//	else terrainDesc.flags = 0;
//
//	terrainDesc.heightFieldVerticalAxis = NX_Z;
//	terrainDesc.heightFieldVerticalExtent = -1000;
//	terrainDesc.materialIndexStride = sizeof(NxMaterialIndex);
//	terrainDesc.materialIndices = gTerrainMaterials;
//
//	mScene->fetchResults(NX_RIGID_BODY_FINISHED,true);
//	if (mHWScene) mHWScene->fetchResults(NX_RIGID_BODY_FINISHED,true);
//	if (mFluidScene) mFluidScene->fetchResults(NX_RIGID_BODY_FINISHED,true);
//
//	NxTriangleMeshShapeDesc terrainShapeDesc;
//
//	NxInitCooking();
//
//	bool status = NxCookTriangleMesh(terrainDesc, myUserStream("terrain.bin", false));
//	terrainShapeDesc.meshData = mPhysicsSDK->createTriangleMesh(myUserStream("terrain.bin", true));		
//
//	NxActorDesc actorDesc;
//	actorDesc.shapes.pushBack(&terrainShapeDesc);
//
//	//physShapeData *userData = new physShapeData;
//	//userData->mType = PHYS_TERRAIN;
//
//	NxActor* actor;
//	NxShape *shp;
//
//	if (mHWScene) {
//		actor = mHWScene->createActor(actorDesc);
//		actor->userData = (void*)-1;
//		shp = *actor->getShapes();
//		//shp->userData = userData;
//		((NxTriangleMeshShape*)shp)->mapPageInstance(0);
//	}
//
//	//That does it for hardware scene, next do fluid scene:
//	//if (mFluidScene) {
//	//  actor = mFluidScene->createActor(actorDesc);
//	//  NxArray<NxTriangleMeshShape*> shapes;
//	//  NxTriangleMeshShape* mesh = shp->isTriangleMesh();
//	//  if (mesh) shapes.pushBack(mesh);
//	//  status = NxCookFluidHardwareMesh(REST_PARTICLES_PER_METER, KERNEL_RADIUS_MULTIPLIER, MOTION_LIMIT_MULTIPLIER, PACKET_SIZE_MULTIPLIER, shapes,myUserStream("terrain.bin", false));
//	//  mFluidScene->createFluidHardwareTriangleMesh(myUserStream("terrain.bin", true));
//	//}
//	//Then, do regular SW scene (could do with a bigger terrain mesh here).
//	terrainDesc.flags = 0;
//	status = NxCookTriangleMesh(terrainDesc, myUserStream("terrain.bin", false));
//	terrainShapeDesc.meshData = mPhysicsSDK->createTriangleMesh(myUserStream("terrain.bin", true));		
//	terrainShapeDesc.shapeFlags = NX_SF_FEATURE_INDICES | NX_SF_VISUALIZATION;
//
//	NxActorDesc actorDesc2;
//	actorDesc2.shapes.pushBack(&terrainShapeDesc);
//	actor = mScene->createActor(actorDesc2);
//	//actor->userData = (void*)-1;
//	shp = *actor->getShapes();
//	//shp->userData = userData;
//
//	mScene->simulate(mStepTime);
//	if (mHWScene) mHWScene->simulate(mStepTime);
//	if (mFluidScene) mFluidScene->simulate(mStepTime);
//
//	//Con::errorf("num terrain verts: %d",num_verts);
//
//	unsigned int c=0;
//	tempVerts[0] = gTerrainVerts[0];
//	for (unsigned int i=1;i<num_verts;i++) 
//	{
//		U8 found=0;
//		for (unsigned int j=0;j<c;j++)
//		{
//			if ((gTerrainVerts[i]==tempVerts[j])&&(i!=j))
//			{	  found=1;  break;  }
//		}	
//		if (!found) tempVerts[c++] = gTerrainVerts[i];
//	}
//	//Con::errorf("distinct verts: %d",c);
//
//	c=0;
//	tempFaces[0] = gTerrainFaces[0];
//	for (unsigned int i=1;i<num_indices;i++) 
//	{
//		U8 found=0;
//		for (unsigned int j=0;j<c;j++)
//		{
//			if ((gTerrainFaces[i]==tempFaces[j])&&(i!=j))
//			{	  found=1;  break;  }
//		}	
//		if (!found) tempFaces[c++] = gTerrainFaces[i];
//	}
//	//Con::errorf("distinct faces: %d",c);
//
//}

ConsoleFunction( nxNum, void, 1, 1, "")
{
	NxScene *kScene = ((nxPhysManager *)physManagerCommon::getPM())->getScene();
	unsigned int num = kScene->getNbActors();
	Con::printf("num actors: %d",num);
}

//ConsoleFunction( addAtlasTerrain, void, 1, 1, "Add Atlas Terrain")
//{
//	physManagerCommon::getPM()->addAtlasTerrain();
//}
//
//ConsoleFunction( setDR, void, 2, 2, "")
//{
//	bool dr = dAtoi(argv[1]);
//	((nxPhysManager *)physManagerCommon::getPM())->setDebugRender(dr);
//	return;
//}


Ogre::Vector3 nxPhysManager::nxCastRay(nxRaycastData rc)
{
	Ogre::Vector3 hitPos = nxCastRay(rc.start,rc.dir,rc.force,rc.damage,rc.Dirt,rc.Brick,rc.Water,rc.Blood);
	return hitPos;//BUMMER - how do I return the hitPos to a function that went on its merry way half a tick ago?
	//ANSWER: store it somewhere and look it up there when you come back.
}

Ogre::Vector3 nxPhysManager::nxCastRay(Ogre::Vector3 start, Ogre::Vector3 dir, float force, float damage, const char *Dirt, const char *Brick, const char *Water, const char *Blood)
{
	//Ogre::Vector3 start,dir;   
	NxVec3 cstart,cdir;
	//float force,damage;

	NxScene *kScene = ((nxPhysManager *)physManagerCommon::getPM())->getScene();

	ExplosionData *dirtDB = NULL;
	ExplosionData *brickDB = NULL;
	ExplosionData *waterDB = NULL;
	ExplosionData *bloodDB = NULL;

	SimObject *simObj;

	SimDataBlockGroup *g = Sim::getDataBlockGroup();
	simObj = Sim::findObject(Dirt);//optimization?  can we find these  
	dirtDB = dynamic_cast<ExplosionData*> (simObj);//once and hold on to them?
	simObj = Sim::findObject(Brick);
	brickDB = dynamic_cast<ExplosionData*> (simObj);
	simObj = Sim::findObject(Water);
	waterDB = dynamic_cast<ExplosionData*> (simObj);
	simObj = Sim::findObject(Blood);
	bloodDB = dynamic_cast<ExplosionData*> (simObj);

	cstart.set(start.x,start.y,start.z);
	cdir.set(dir.x,dir.y,dir.z);
	cdir.normalize();

	NxRay worldRay;
	NxRaycastHit rHit;
	worldRay.orig = cstart;
	worldRay.dir = cdir;

	//gPhysRaycastStart = start;
	//gPhysRaycastEnd = start + (dir * 100);
	//Con::printf("Setting raycastStart: %f %f %f",start.x,start.y,start.z);

	//kScene->fetchResults(NX_RIGID_BODY_FINISHED,true);
	//NxShape* closestShape = kScene->raycastClosestShape(worldRay, NX_DYNAMIC_SHAPES, rHit);
	NxShape* closestShape = kScene->raycastClosestShape(worldRay, NX_ALL_SHAPES, rHit,3);//group 3?  I hope?	
	//unsigned int numShapes = kScene->raycastAllShapes(worldRay, NX_ALL_SHAPES, rHit);	

	Ogre::Vector3 kNorm(0.0,0.0,1.0);

	if (closestShape) {
		//if (rHit.shape) {
		NxActor *kActor = &(closestShape->getActor());
		iPhysUser *kPU = ((iPhysUser *)kActor->userData);
		if (kPU)
		{

			Con::errorf("castray got a hit: %f %f %f, type %d force %f damage %f",
				rHit.worldImpact.x,rHit.worldImpact.y,rHit.worldImpact.z,kPU->mEntityType,force,damage);
			if (kPU->mEntityType == PHYS_INTERIOR) {

				Ogre::Vector3 kPos(rHit.worldImpact.x,rHit.worldImpact.y,rHit.worldImpact.z);
				Ogre::Vector3 kNorm(0.0,0.0,1.0);//FIX

				if (force==-2.0) {//Another special case, give TweakerOne a moveToPosition command at this location.
					gTweakerOne->moveToPosition(kPos,gTweakerOne->mMoveSequence);
					char kPosString[255];
					sprintf(kPosString,"%g %g %g",kPos.x,kPos.y,kPos.z);
					Con::executef("setMoveToPositionXYZ",kPosString);
				}

				if (brickDB) 
				{
					Explosion* pExplosion = NULL;
					pExplosion = new Explosion;
					pExplosion->onNewDataBlock((GameBaseData *)brickDB,false);

					if( pExplosion )
					{
						Ogre::Matrix4 xform(true);
						xform.setPosition(kPos);
						pExplosion->setTransform(xform);
						pExplosion->setInitialState(kPos, kNorm);
						pExplosion->setCollideType( 0 );//collideType?
						if (pExplosion->registerObject() == false)
						{
							Con::errorf(ConsoleLogEntry::General, "Projectile(%s)::explode: couldn't register explosion",
								brickDB->getName() );
							delete pExplosion;
							pExplosion = NULL;
						}
					}
				}
			}  else if (kPU->mEntityType == PHYS_TERRAIN) {
				//HERE: make a particle effect!  (BulletDirtExplosion)
				Ogre::Vector3 kPos(rHit.worldImpact.x,rHit.worldImpact.y,rHit.worldImpact.z);
				Ogre::Vector3 kNorm(0.0,0.0,1.0);
				if (force==-2.0) {//Another special case, give TweakerOne a moveToPosition command at this location.
					gTweakerOne->moveToPosition(kPos,gTweakerOne->mMoveSequence);
				}

				if (dirtDB) 
				{
					Explosion* pExplosion = NULL;
					pExplosion = new Explosion;
					pExplosion->onNewDataBlock((GameBaseData *)dirtDB,false);

					if( pExplosion )
					{
						Ogre::Matrix4 xform(true);
						xform.setPosition(kPos);
						pExplosion->setTransform(xform);
						pExplosion->setInitialState(kPos, kNorm);
						pExplosion->setCollideType( 0 );//collideType?
						if (pExplosion->registerObject() == false)
						{
							Con::errorf(ConsoleLogEntry::General, "Projectile(%s)::explode: couldn't register explosion",
								dirtDB->getName() );
							delete pExplosion;
							pExplosion = NULL;
						}
					}
				}

			} else if (kPU->mEntityType == PHYS_RIGID_BODY) {
				if (force>0) 
				{
					Ogre::Vector3 kForce(dir.x * force,dir.y * force,dir.z * force);
					Ogre::Vector3 kPos(rHit.worldImpact.x,rHit.worldImpact.y,rHit.worldImpact.z);

					if (force==-2.0) 
					{//Another special case, give TweakerOne a moveToPosition command at this location.
						gTweakerOne->moveToPosition(kPos,gTweakerOne->mMoveSequence);
					}

					kPU->mTempForce = kForce;
					kPU->mTempPos = kPos;
					kPU->mHasTempForce = true;
					//kPU->addForceAtPos(kForce,kPos);


					if (brickDB) 
					{
						Explosion* pExplosion = NULL;
						pExplosion = new Explosion;
						pExplosion->onNewDataBlock((GameBaseData *)brickDB,false);

						if( pExplosion )
						{
							Ogre::Matrix4 xform(true);
							xform.setPosition(kPos);
							pExplosion->setTransform(xform);
							pExplosion->setInitialState(kPos, kNorm);
							pExplosion->setCollideType( 0 );//collideType?
							if (pExplosion->registerObject() == false)
							{
								Con::errorf(ConsoleLogEntry::General, "Projectile(%s)::explode: couldn't register explosion",
									brickDB->getName() );
								delete pExplosion;
								pExplosion = NULL;
							}
						}
					}
				}
			} else if (kPU->mEntityType == PHYS_FLEX_BODY_PART) {

				if (force>0) 
				{
					//HERE: do not call onCollision, hasTempForce, etc. unless the victim is DEAD
					//First, do some reasonable amount of damage, and see where we're at.
					//Furthermore, use the bodypart to determine the amount of damage (head shots).
					//NEGATIVE:  Do none of that here, instead pass all this information to mPhysUser,
					//and _then_ call onCollision

					//shapeData->mPhysUser->onCollision(NULL);
					Ogre::Vector3 kForce(dir.x * force,dir.y * force,dir.z * force);
					Ogre::Vector3 kPos(rHit.worldImpact.x,rHit.worldImpact.y,rHit.worldImpact.z);
					//iPhysUser *kPU = shapeData->mPhysUser;

					if (kPU->mEntitySubType == PHYS_SUB_FLEX_BIPED)
					{
						kPU->mTempDamage = damage;
						kPU->mTempForce = kForce;
						kPU->mTempPos = kPos;
						kPU->mHasTempForce = true;//any reason for this? can just reset mTempForce every time.
						//shapeData->mPhysUser->addForceAtPos(kForce,kPos);
						if (force==-1.0) {
							kPU->onCollision(NULL,1);//Ecstasy: force==-1 is code for tap the character 
							//with a castray, don't do any damage, but do something else with it. (eg select for gui)
							Con::errorf("selecting bot!");
							return Ogre::Vector3(0,0,0);
						} 
						kPU->onCollision(NULL);
						if (bloodDB) 
						{
							Explosion* pExplosion = NULL;
							pExplosion = new Explosion;
							pExplosion->onNewDataBlock((GameBaseData *)bloodDB,false);

							if( pExplosion )
							{
								Ogre::Matrix4 xform(true);
								xform.setPosition(kPos);
								pExplosion->setTransform(xform);
								pExplosion->setInitialState(kPos, kNorm);
								pExplosion->setCollideType( 0 );//collideType?
								if (pExplosion->registerObject() == false)
								{
									Con::errorf(ConsoleLogEntry::General, "Projectile(%s)::explode: couldn't register explosion",
										bloodDB->getName() );
									delete pExplosion;
									pExplosion = NULL;
								}
							}
						}
					}
					else if (kPU->mEntitySubType == PHYS_SUB_FLEX_TREE)
					{
						kPU->mTempDamage = damage;
						kPU->mTempForce = kForce;
						kPU->mTempPos = kPos;
						kPU->mHasTempForce = true;//any reason for this? can just reset mTempForce every time.
						//shapeData->mPhysUser->addForceAtPos(kForce,kPos);
						kPU->onCollision(NULL);
						if (dirtDB) 
						{
							Explosion* pExplosion = NULL;
							pExplosion = new Explosion;
							pExplosion->onNewDataBlock((GameBaseData *)dirtDB,false);

							if( pExplosion )
							{
								Ogre::Matrix4 xform(true);
								xform.setPosition(kPos);
								pExplosion->setTransform(xform);
								pExplosion->setInitialState(kPos, kNorm);
								pExplosion->setCollideType( 0 );//collideType?
								if (pExplosion->registerObject() == false)
								{
									Con::errorf(ConsoleLogEntry::General, "Projectile(%s)::explode: couldn't register explosion",
										dirtDB->getName() );
									delete pExplosion;
									pExplosion = NULL;
								}
							}
						}
					}
				}
			}
		}
	}
	Ogre::Vector3 hitPos(rHit.worldImpact.x,rHit.worldImpact.y,rHit.worldImpact.z);
	return hitPos;
}

ConsoleFunction( nxCastRay, const char*, 9, 9, "Ogre::Vector3 start, Ogre::Vector3 dir, float force, float damage, DirtExplosion, BrickExplosion, WaterExplosion,BloodExplosion")
{
	Ogre::Vector3 start,dir;
	float force,damage;
	dSscanf(argv[1],"%g %g %g",&start.x,&start.y,&start.z);
	dSscanf(argv[2],"%g %g %g",&dir.x,&dir.y,&dir.z);
	force = dAtoi(argv[3]);
	damage = dAtoi(argv[4]);

	//gPhysRaycastStart = start;
	//gPhysRaycastEnd = start + (dir * 100);
	//Con::printf("Setting raycastStart: %f %f %f",start.x,start.y,start.z);

	Ogre::Vector3 hitPos = ((nxPhysManager *)physManagerCommon::getPM())->nxCastRay(start,dir,force,damage,argv[5],argv[6],argv[7],argv[8]);

	char* ret = Con::getReturnBuffer(256);
	dSprintf(ret, 255, "%g %g %g",hitPos.x,hitPos.y,hitPos.z);
	return ret;
}

ConsoleFunction( nxDelayCastRay, void, 9, 9, "Ogre::Vector3 start, Ogre::Vector3 dir, float force, float damage, DirtExplosion, BrickExplosion, WaterExplosion,BloodExplosion")
{
	nxRaycastData rc;

	dSscanf(argv[1],"%g %g %g",&rc.start.x,&rc.start.y,&rc.start.z);
	dSscanf(argv[2],"%g %g %g",&rc.dir.x,&rc.dir.y,&rc.dir.z);
	rc.force = dAtoi(argv[3]);
	rc.damage = dAtoi(argv[4]);
	rc.Dirt = argv[5];
	rc.Brick = argv[6];
	rc.Water = argv[7];
	rc.Blood = argv[8];

	((nxPhysManager *)physManagerCommon::getPM())->addRaycast(rc);
}

ConsoleFunction( castNxTractorBeam, void, 4, 4, "Ogre::Vector3 start, Ogre::Vector3 dir, int type")
{
	Ogre::Vector3 start,dir;
	int type;
	dSscanf(argv[1],"%g %g %g",&start.x,&start.y,&start.z);
	dSscanf(argv[2],"%g %g %g",&dir.x,&dir.y,&dir.z);
	dSscanf(argv[3],"%d",&type);

	NxVec3 cstart,cdir;
	cstart.set(start.x,start.y,start.z);
	cdir.set(dir.x,dir.y,dir.z);
	cdir.normalize();

	NxRay worldRay;
	NxRaycastHit rHit;
	worldRay.orig = cstart;
	worldRay.dir = cdir;
	NxScene *kScene = ((nxPhysManager *)physManagerCommon::getPM())->getScene();

	//gPhysRaycastStart = start;
	//gPhysRaycastEnd = start + (dir * 20);
	//Con::printf("Setting raycastStart: %f %f %f",start.x,start.y,start.z);

	NxShape* closestShape = kScene->raycastClosestShape(worldRay, NX_DYNAMIC_SHAPES, rHit);

	if (rHit.shape) {
		NxActor *kActor = &(closestShape->getActor());
		//physShapeData *shapeData = ((physShapeData *)kActor->userData);
		iPhysUser *physUser = ((iPhysUser *)kActor->userData);

		if (physUser)
			Con::printf("tractor beam hit something! type %d hasTractor %d",physUser->mEntityType,physUser->mHasTractorBeam);
		else 
			return;

		if ((physUser->mEntityType==PHYS_FLEX_BODY_PART)||(physUser->mEntityType==PHYS_RIGID_BODY)) 
		{
			//if (physUser->mEntityType==PHYS_FLEX_BODY_PART)
				//physUser->onCollision(NULL);
			NxVec3 nxPos = kActor->getGlobalPosition();
			//Ogre::Vector3 kPos(rHit.worldImpact.x,rHit.worldImpact.y,rHit.worldImpact.z);
			Ogre::Vector3 kPos(nxPos.x,nxPos.y,nxPos.z);
			Ogre::Vector3 kInvPos, objPos;
			//kPos -= start;//start is muzzle position, which is NOT the same as player position

			SimGroup *g = Sim::getClientGroup();
			Ogre::Matrix4 eyeTransform;
			for (SimGroup::iterator itr = g->begin(); itr != g->end(); itr++) 
			{//really, should verify that there's only one client, or this is going to get all f**ked up.
				GameConnection *con = (GameConnection *) (*itr);
				ShapeBase *obj = dynamic_cast<ShapeBase *>(con->getControlObject());
				obj->getEyeTransform(&eyeTransform);
				objPos = obj->getPosition();
			}
			kPos -= objPos;
			Ogre::Matrix4 newCamTrans = eyeTransform;
			newCamTrans.setPosition(Ogre::Vector3(0,0,0));
			newCamTrans.inverse();
			newCamTrans.mulP(kPos);
			//kPos.normalize();
			//kPos *= 6.0;
			physUser->mTempPos = kPos;
			physUser->lockTractorBeam(type);
		}
	}
	return;
}

ConsoleFunction( setMouseValue, void, 2,2,"int value")
{
	int val;
	dSscanf(argv[1],"%d",&val);
	mouseValue = val;
	return;
}

ConsoleFunction( getNumActors, int, 1,1,"")
{
	return ((nxPhysManager *)physManagerCommon::getPM())->getScene()->getNbActors();
}

///////////////////////////////////////


NxFixedJoint* nxPhysManager::CreateFixedJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxFixedJointDesc fixedDesc;
	fixedDesc.actor[0] = a0;
	fixedDesc.actor[1] = a1;
	fixedDesc.setGlobalAnchor(globalAnchor);
	fixedDesc.setGlobalAxis(globalAxis);

	Con::printf("create fixed joint! Anchor %3.2f %3.2f %3.2f   --  Axis %3.2f %3.2f %3.2f",globalAnchor.x,globalAnchor.y,globalAnchor.z,globalAxis.x,globalAxis.y,globalAxis.z);

	return (NxFixedJoint*)mScene->createJoint(fixedDesc);
}

NxRevoluteJoint* nxPhysManager::CreateRevoluteJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxRevoluteJointDesc revDesc;
	revDesc.actor[0] = a0;
	revDesc.actor[1] = a1;
	revDesc.setGlobalAnchor(globalAnchor);
	revDesc.setGlobalAxis(globalAxis);

	return (NxRevoluteJoint*)mScene->createJoint(revDesc);
}

NxRevoluteJoint* nxPhysManager::CreateRevoluteJoint2(NxActor* a0, NxActor* a1, const NxVec3& localAnchor0,const NxVec3& localAnchor1, const NxVec3& localAxis0, const NxVec3& localAxis1)
{
	NxRevoluteJointDesc revDesc;
	revDesc.actor[0] = a0;
	revDesc.actor[1] = a1;
	//revDesc.setGlobalAnchor(globalAnchor);
	//revDesc.setGlobalAxis(globalAxis);
	revDesc.localAnchor[0]=localAnchor0;
	revDesc.localAnchor[1]=localAnchor1;
	revDesc.localAxis[0]  =localAxis0;
	revDesc.localAxis[1]  =localAxis1;
	revDesc.projectionMode = NX_JPM_POINT_MINDIST;
	return (NxRevoluteJoint*)mScene->createJoint(revDesc);
}


NxSphericalJoint* nxPhysManager::CreateSphericalJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxSphericalJointDesc sphericalDesc;
	sphericalDesc.actor[0] = a0;
	sphericalDesc.actor[1] = a1;
	sphericalDesc.setGlobalAnchor(globalAnchor);
	sphericalDesc.setGlobalAxis(globalAxis);

	return (NxSphericalJoint*)mScene->createJoint(sphericalDesc);
}

NxPrismaticJoint* nxPhysManager::CreatePrismaticJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxPrismaticJointDesc prismaticDesc;
	prismaticDesc.actor[0] = a0;
	prismaticDesc.actor[1] = a1;
	prismaticDesc.setGlobalAnchor(globalAnchor);
	prismaticDesc.setGlobalAxis(globalAxis);

	NxJoint* joint = mScene->createJoint(prismaticDesc);

	//	joint->setLimitPoint(globalAnchor);
	//	joint->addLimitPlane(-globalAxis, globalAnchor + 1.5*globalAxis);
	//	joint->addLimitPlane(globalAxis, globalAnchor - 1.5*globalAxis);

	return (NxPrismaticJoint*)joint;
}

NxCylindricalJoint* nxPhysManager::CreateCylindricalJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxCylindricalJointDesc cylDesc;
	cylDesc.actor[0] = a0;
	cylDesc.actor[1] = a1;
	cylDesc.setGlobalAnchor(globalAnchor);
	cylDesc.setGlobalAxis(globalAxis);

	NxJoint* joint = mScene->createJoint(cylDesc);

	return (NxCylindricalJoint*)joint;
}

NxPointOnLineJoint* nxPhysManager::CreatePointOnLineJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxPointOnLineJointDesc polDesc;
	polDesc.actor[0] = a0;
	polDesc.actor[1] = a1;
	polDesc.setGlobalAnchor(globalAnchor);
	polDesc.setGlobalAxis(globalAxis);
	polDesc.jointFlags |= NX_JF_COLLISION_ENABLED;

	NxJoint* joint = mScene->createJoint(polDesc);

	return (NxPointOnLineJoint*)joint;
}

NxPointInPlaneJoint* nxPhysManager::CreatePointInPlaneJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxPointInPlaneJointDesc pipDesc;
	pipDesc.actor[0] = a0;
	pipDesc.actor[1] = a1;
	pipDesc.setGlobalAnchor(globalAnchor);
	pipDesc.setGlobalAxis(globalAxis);
	pipDesc.jointFlags |= NX_JF_COLLISION_ENABLED;

	NxJoint* joint = mScene->createJoint(pipDesc);

	return (NxPointInPlaneJoint*)joint;
}

NxPulleyJoint* nxPhysManager::CreatePulleyJoint(NxActor* a0, NxActor* a1, const NxVec3& pulley0, const NxVec3& pulley1, const NxVec3& globalAxis, NxReal distance, NxReal ratio, const NxMotorDesc& motorDesc)
{
	NxPulleyJointDesc pulleyDesc;
	pulleyDesc.actor[0] = a0;
	pulleyDesc.actor[1] = a1;
	pulleyDesc.localAnchor[0] = NxVec3(0,2,0);
	pulleyDesc.localAnchor[1] = NxVec3(0,2,0);
	pulleyDesc.setGlobalAxis(globalAxis);

	pulleyDesc.pulley[0] = pulley0; 	// suspension points of two bodies in world space.
	pulleyDesc.pulley[1] = pulley1; 	// suspension points of two bodies in world space.
	pulleyDesc.distance = distance;		// the rest length of the rope connecting the two objects.  The distance is computed as ||(pulley0 - anchor0)|| +  ||(pulley1 - anchor1)|| * ratio.
	pulleyDesc.ratio = ratio;			// transmission ratio
	pulleyDesc.flags = NX_PJF_IS_RIGID;	// this is a combination of the bits defined by ::NxPulleyJointFlag. 
	pulleyDesc.motor = motorDesc;
	pulleyDesc.stiffness = 1;		    // how stiff the constraint is, between 0 and 1 (stiffest)

	//	pulleyDesc.projectionMode = NX_JPM_NONE;
	//	pulleyDesc.projectionMode = NX_JPM_POINT_MINDIST;

	pulleyDesc.jointFlags |= NX_JF_COLLISION_ENABLED;

	return (NxPulleyJoint*)mScene->createJoint(pulleyDesc);
}

NxDistanceJoint* nxPhysManager::CreateDistanceJoint(NxActor* a0, NxActor* a1, const NxVec3& anchor0, const NxVec3& anchor1, const NxVec3& globalAxis)
{
	NxDistanceJointDesc distanceDesc;
	distanceDesc.actor[0] = a0;
	distanceDesc.actor[1] = a1;
	distanceDesc.localAnchor[0] = anchor0;
	distanceDesc.localAnchor[1] = anchor1;
	distanceDesc.setGlobalAxis(globalAxis);

	NxVec3 dist = a1->getGlobalPose()*anchor1 - a0->getGlobalPose()*anchor0;
	distanceDesc.maxDistance = dist.magnitude()*1.5f;  // maximum rest length of the rope or rod between the two anchor points
	distanceDesc.minDistance = dist.magnitude()*0.5f;  // minimum rest length of the rope or rod between the two anchor points
	NxSpringDesc spring;
	spring.spring = 100;
	spring.damper = 0.5;
	distanceDesc.spring = spring;  // makes the joint springy. The spring.targetValue field is not used.
	distanceDesc.flags = (NX_DJF_MIN_DISTANCE_ENABLED | NX_DJF_MAX_DISTANCE_ENABLED);  // combination of the bits defined by ::NxDistanceJointFlag
	distanceDesc.flags |= NX_DJF_SPRING_ENABLED;

	//    distanceDesc.projectionMode = NX_JPM_NONE;
	//    distanceDesc.projectionMode = NX_JPM_POINT_MINDIST;

	distanceDesc.jointFlags |= NX_JF_COLLISION_ENABLED;

	return (NxDistanceJoint*)mScene->createJoint(distanceDesc);
}

NxSphericalJoint* nxPhysManager::CreateRopeSphericalJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxSphericalJointDesc sphericalDesc;
	sphericalDesc.actor[0] = a0;
	sphericalDesc.actor[1] = a1;
	sphericalDesc.setGlobalAnchor(globalAnchor);
	sphericalDesc.setGlobalAxis(globalAxis);

	sphericalDesc.flags |= NX_SJF_TWIST_LIMIT_ENABLED;
	sphericalDesc.twistLimit.low.value = -(NxReal)0.1*NxPi;
	sphericalDesc.twistLimit.high.value = (NxReal)0.1*NxPi;

	sphericalDesc.flags |= NX_SJF_TWIST_SPRING_ENABLED;
	NxSpringDesc ts;
	ts.spring = 500;
	ts.damper = 0.5;
	ts.targetValue = 0;
	sphericalDesc.twistSpring = ts;

	sphericalDesc.flags |= NX_SJF_SWING_LIMIT_ENABLED;
	sphericalDesc.swingLimit.value = (NxReal)0.3*NxPi;

	sphericalDesc.flags |= NX_SJF_SWING_SPRING_ENABLED;
	NxSpringDesc ss;
	ss.spring = 500;
	ss.damper = 0.5;
	ss.targetValue = 0;
	sphericalDesc.swingSpring = ss;

	return (NxSphericalJoint*)mScene->createJoint(sphericalDesc);
}

NxSphericalJoint* nxPhysManager::CreateClothSphericalJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxSphericalJointDesc sphericalDesc;
	sphericalDesc.actor[0] = a0;
	sphericalDesc.actor[1] = a1;
	sphericalDesc.setGlobalAnchor(globalAnchor);
	sphericalDesc.setGlobalAxis(globalAxis);

	sphericalDesc.flags |= NX_SJF_TWIST_LIMIT_ENABLED;
	sphericalDesc.twistLimit.low.value = -(NxReal)0.025*NxPi;
	sphericalDesc.twistLimit.low.hardness = 0.5;
	sphericalDesc.twistLimit.low.restitution = 0.5;
	sphericalDesc.twistLimit.high.value = (NxReal)0.025*NxPi;
	sphericalDesc.twistLimit.high.hardness = 0.5;
	sphericalDesc.twistLimit.high.restitution = 0.5;

	sphericalDesc.flags |= NX_SJF_SWING_LIMIT_ENABLED;
	sphericalDesc.swingLimit.value = (NxReal)0.25*NxPi;
	sphericalDesc.swingLimit.hardness = 0.5;
	sphericalDesc.swingLimit.restitution = 0.5;

	sphericalDesc.flags |= NX_SJF_TWIST_SPRING_ENABLED;
	sphericalDesc.twistSpring.spring = 0.5;
	sphericalDesc.twistSpring.damper = 1;

	sphericalDesc.flags |= NX_SJF_SWING_SPRING_ENABLED;
	sphericalDesc.swingSpring.spring = 0.5;
	sphericalDesc.swingSpring.damper = 1;

	//	sphericalDesc.flags |= NX_SJF_JOINT_SPRING_ENABLED;
	//	sphericalDesc.jointSpring.spring = 0.5;
	//	sphericalDesc.jointSpring.damper = 1;

	sphericalDesc.projectionDistance = (NxReal)0.15;
	sphericalDesc.projectionMode = NX_JPM_POINT_MINDIST;

	return (NxSphericalJoint*)mScene->createJoint(sphericalDesc);
}

NxSphericalJoint* nxPhysManager::CreateBodySphericalJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxSphericalJointDesc sphericalDesc;
	sphericalDesc.actor[0] = a0;
	sphericalDesc.actor[1] = a1;
	sphericalDesc.setGlobalAnchor(globalAnchor);
	sphericalDesc.setGlobalAxis(globalAxis);

	sphericalDesc.flags |= NX_SJF_TWIST_LIMIT_ENABLED;
	sphericalDesc.twistLimit.low.value = -(NxReal)0.025*NxPi;
	sphericalDesc.twistLimit.low.hardness = 0.5;
	sphericalDesc.twistLimit.low.restitution = 0.5;
	sphericalDesc.twistLimit.high.value = (NxReal)0.025*NxPi;
	sphericalDesc.twistLimit.high.hardness = 0.5;
	sphericalDesc.twistLimit.high.restitution = 0.5;

	sphericalDesc.flags |= NX_SJF_SWING_LIMIT_ENABLED;
	sphericalDesc.swingLimit.value = (NxReal)0.25*NxPi;
	sphericalDesc.swingLimit.hardness = 0.5;
	sphericalDesc.swingLimit.restitution = 0.5;

	sphericalDesc.flags |= NX_SJF_TWIST_SPRING_ENABLED;
	sphericalDesc.twistSpring.spring = 0.5;
	sphericalDesc.twistSpring.damper = 1;

	sphericalDesc.flags |= NX_SJF_SWING_SPRING_ENABLED;
	sphericalDesc.swingSpring.spring = 0.5;
	sphericalDesc.swingSpring.damper = 1;

	//	sphericalDesc.flags |= NX_SJF_JOINT_SPRING_ENABLED;
	//	sphericalDesc.jointSpring.spring = 0.5;
	//	sphericalDesc.jointSpring.damper = 1;

	sphericalDesc.projectionDistance = (NxReal)0.15;
	sphericalDesc.projectionMode = NX_JPM_POINT_MINDIST;

	return (NxSphericalJoint*)mScene->createJoint(sphericalDesc);
}

NxRevoluteJoint* nxPhysManager::CreateWheelJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxRevoluteJointDesc revDesc;
	revDesc.actor[0] = a0;
	revDesc.actor[1] = a1;
	revDesc.setGlobalAnchor(globalAnchor);
	revDesc.setGlobalAxis(globalAxis);

	return (NxRevoluteJoint*)mScene->createJoint(revDesc);
}

NxRevoluteJoint* nxPhysManager::CreateStepJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxRevoluteJointDesc revDesc;
	revDesc.actor[0] = a0;
	revDesc.actor[1] = a1;
	revDesc.setGlobalAnchor(globalAnchor);
	revDesc.setGlobalAxis(globalAxis);

	return (NxRevoluteJoint*)mScene->createJoint(revDesc);
}

NxRevoluteJoint* nxPhysManager::CreateChassisJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxRevoluteJointDesc revDesc;
	revDesc.actor[0] = a0;
	revDesc.actor[1] = a1;
	revDesc.setGlobalAnchor(globalAnchor);
	revDesc.setGlobalAxis(globalAxis);

	revDesc.flags |= NX_RJF_LIMIT_ENABLED;

	NxJointLimitPairDesc limitDesc;
	limitDesc.high.value = (NxReal)0.01*NxPi;
	limitDesc.low.value = -(NxReal)0.01*NxPi;;

	revDesc.limit = limitDesc;

	revDesc.flags |= NX_RJF_SPRING_ENABLED;
	NxSpringDesc springDesc;
	springDesc.targetValue = 0;
	springDesc.spring = 5000;

	//	motorDesc.maxForce = 100;
	//	motorDesc.freeSpin = 0;
	//
	//	revDesc.motor = motorDesc;

	return (NxRevoluteJoint*)mScene->createJoint(revDesc);
}

NxFixedJoint* nxPhysManager::CreateCannonJoint(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxFixedJointDesc fixDesc;

	fixDesc.actor[0] = a0;
	fixDesc.actor[1] = a1;
	fixDesc.setGlobalAnchor(globalAnchor);
	fixDesc.setGlobalAxis(globalAxis);

	return (NxFixedJoint*)mScene->createJoint(fixDesc);
}

NxSphericalJoint* nxPhysManager::CreateBladeLink(NxActor* a0, NxActor* a1, const NxVec3& globalAnchor, const NxVec3& globalAxis)
{
	NxSphericalJointDesc sphericalDesc;
	sphericalDesc.actor[0] = a0;
	sphericalDesc.actor[1] = a1;
	sphericalDesc.setGlobalAnchor(globalAnchor);
	sphericalDesc.setGlobalAxis(globalAxis);

	sphericalDesc.flags |= NX_SJF_SWING_LIMIT_ENABLED;
	sphericalDesc.swingLimit.value = (NxReal)0.05*NxPi;
	sphericalDesc.swingLimit.restitution = 0.75;
	sphericalDesc.swingLimit.hardness = 0.5;

	sphericalDesc.flags |= NX_SJF_SWING_SPRING_ENABLED;
	sphericalDesc.swingSpring.spring = 0.75;
	sphericalDesc.swingSpring.damper = 1;

	sphericalDesc.flags |= NX_SJF_TWIST_LIMIT_ENABLED;
	sphericalDesc.twistLimit.low.value = -(NxReal)0.05*NxPi;
	sphericalDesc.twistLimit.low.restitution = 0.75;
	sphericalDesc.twistLimit.low.hardness = 0.5;
	sphericalDesc.twistLimit.high.value = (NxReal)0.05*NxPi;
	sphericalDesc.twistLimit.high.restitution = 0.75;
	sphericalDesc.twistLimit.high.hardness = 0.5;

	sphericalDesc.flags |= NX_SJF_TWIST_SPRING_ENABLED;
	sphericalDesc.twistSpring.spring = 0.75;
	sphericalDesc.twistSpring.damper = 1;

	return (NxSphericalJoint*)mScene->createJoint(sphericalDesc);
}


NxActor* nxPhysManager::CreateGroundPlane()
{
	// Create a plane with default descriptor
	NxPlaneShapeDesc planeDesc;
	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&planeDesc);
	return mScene->createActor(actorDesc);
}

NxActor* nxPhysManager::CreateBox(const NxVec3& pos, const NxVec3& boxDim, const NxReal density)
{

	NxBoxShapeDesc boxDesc;
	boxDesc.dimensions.set(boxDim.x, boxDim.y, boxDim.z);
	boxDesc.localPose.t = NxVec3(0, boxDim.y, 0);

	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&boxDesc);
	actorDesc.globalPose.t = pos;

	NxBodyDesc bodyDesc;
	if (density)
	{
		actorDesc.body = &bodyDesc;
		actorDesc.density = density;
	}
	else
	{
		actorDesc.body = 0;
	}
	return mScene->createActor(actorDesc);	
}

NxActor* nxPhysManager::CreateSphere(const NxVec3& pos, const NxReal radius, const NxReal density)
{

	NxSphereShapeDesc sphereDesc;
	sphereDesc.radius = radius;
	sphereDesc.localPose.t = NxVec3(0, radius, 0);

	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&sphereDesc);
	actorDesc.globalPose.t = pos;

	NxBodyDesc bodyDesc;
	if (density)
	{
		actorDesc.body = &bodyDesc;
		actorDesc.density = density;
	}
	else
	{
		actorDesc.body = 0;
	}
	return mScene->createActor(actorDesc);	
}

NxActor* nxPhysManager::CreateCapsule(const NxVec3& pos, const NxReal height, const NxReal radius, const NxReal density)
{

	NxCapsuleShapeDesc capsuleDesc;
	capsuleDesc.height = height;
	capsuleDesc.radius = radius;
	capsuleDesc.localPose.t = NxVec3(0, radius + 0.5f * height, 0);

	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&capsuleDesc);
	actorDesc.globalPose.t = pos;

	NxBodyDesc bodyDesc;
	if (density)
	{
		actorDesc.body = &bodyDesc;
		actorDesc.density = density;
	}
	else
	{
		actorDesc.body = 0;
	}
	return mScene->createActor(actorDesc);	
}


class Ragdoll
{
public:
	Ragdoll(const NxVec3& pos)
	{
		NxQuat qRotLeft, qRotRight, qRotAround;
		qRotLeft.fromAngleAxis(90, NxVec3(0,0,1));
		qRotRight.fromAngleAxis(-90, NxVec3(0,0,1));
		qRotAround.fromAngleAxis(180, NxVec3(0,0,1));

		NxMat33 mRotLeft, mRotRight, mRotAround;
		mRotLeft.fromQuat(qRotLeft);
		mRotRight.fromQuat(qRotRight);
		mRotAround.fromQuat(qRotAround);

		nxPhysManager *kPM = ((nxPhysManager *)physManagerCommon::getPM());
		// Create body parts
		head = kPM->CreateSphere(NxVec3(0,8.8,0), 0.5, 10);
		torso = kPM->CreateSphere(NxVec3(0,7,0), 0.95, 10);
		pelvis = kPM->CreateSphere(NxVec3(0,5.8,0), 0.7, 10);

		leftUpperArm = kPM->CreateCapsule(NxVec3(0.5,8.5,0), 1, 0.4, 10);
		leftUpperArm->setGlobalOrientationQuat(qRotRight);
		leftForeArm = kPM->CreateCapsule(NxVec3(2,8.5,0), 1, 0.3, 10);
		leftForeArm->setGlobalOrientationQuat(qRotRight);
		leftHand = kPM->CreateBox(NxVec3(3.5,8.5,0), NxVec3(0.3,0.3,0.1), 10);
		leftHand->setGlobalOrientationQuat(qRotRight);

		rightUpperArm = kPM->CreateCapsule(NxVec3(-0.5,8.5,0), 1, 0.4, 10);
		rightUpperArm->setGlobalOrientationQuat(qRotLeft);
		rightForeArm = kPM->CreateCapsule(NxVec3(-2,8.5,0), 1, 0.3, 10);
		rightForeArm->setGlobalOrientationQuat(qRotLeft);
		rightHand = kPM->CreateBox(NxVec3(-3.5,8.5,0), NxVec3(0.3,0.3,0.1), 10);
		rightHand->setGlobalOrientationQuat(qRotLeft);

		leftThigh = kPM->CreateCapsule(NxVec3(0.6,6,0), 1.5, 0.5, 10);
		leftThigh->setGlobalOrientationQuat(qRotAround);
		leftCalf = kPM->CreateCapsule(NxVec3(0.6,3.5,0), 1.5, 0.35, 10);
		leftCalf->setGlobalOrientationQuat(qRotAround);
		leftFoot = kPM->CreateBox(NxVec3(0.6,1.5,0.2), NxVec3(0.4,0.2,0.75), 10);
		leftFoot->setGlobalOrientationQuat(qRotAround);

		rightThigh = kPM->CreateCapsule(NxVec3(-0.6,6,0), 1.5, 0.5, 10);
		rightThigh->setGlobalOrientationQuat(qRotAround);
		rightCalf = kPM->CreateCapsule(NxVec3(-0.6,3.5,0), 1.5, 0.35, 10);
		rightCalf->setGlobalOrientationQuat(qRotAround);
		rightFoot = kPM->CreateBox(NxVec3(-0.6,1.5,0.2), NxVec3(0.4,0.2,0.75), 10);
		rightFoot->setGlobalOrientationQuat(qRotAround);

		// Joint body parts together
		neck = kPM->CreateFixedJoint(head,torso,NxVec3(0,8.8,0),NxVec3(0,1,0));
		leftShoulder = kPM->CreateFixedJoint(leftUpperArm,torso,NxVec3(0.5,8.5,0),NxVec3(1,0,0));
		rightShoulder = kPM->CreateFixedJoint(rightUpperArm,torso,NxVec3(-0.5,8.5,0),NxVec3(-1,0,0));
		spine = kPM->CreateFixedJoint(torso,pelvis,NxVec3(0,7,0),NxVec3(0,-1,0));
		leftHip = kPM->CreateFixedJoint(leftThigh,pelvis,NxVec3(0.6,6,0),NxVec3(0,-1,0));
		rightHip = kPM->CreateFixedJoint(rightThigh,pelvis,NxVec3(-0.6,6,0),NxVec3(0,-1,0));
		leftElbow = kPM->CreateFixedJoint(leftForeArm,leftUpperArm,NxVec3(2,8.5,0),NxVec3(0,0,-1));
		rightElbow = kPM->CreateFixedJoint(rightForeArm,rightUpperArm,NxVec3(-2,8.5,0),NxVec3(0,0,-1));
		leftWrist = kPM->CreateFixedJoint(leftHand,leftForeArm,NxVec3(3.5,8.5,0),NxVec3(0,0,-1));
		rightWrist = kPM->CreateFixedJoint(rightHand,rightForeArm,NxVec3(-3.5,8.5,0),NxVec3(0,0,-1));
		leftKnee = kPM->CreateFixedJoint(leftCalf,leftThigh,NxVec3(0.6,3.5,0),NxVec3(1,0,0));
		rightKnee = kPM->CreateFixedJoint(rightCalf,rightThigh,NxVec3(-0.6,3.5,0),NxVec3(-1,0,0));
		leftAnkle = kPM->CreateFixedJoint(leftFoot,leftCalf,NxVec3(0.6,1.3,0),NxVec3(1,0,0));
		rightAnkle = kPM->CreateFixedJoint(rightFoot,rightCalf,NxVec3(-0.6,1.3,0),NxVec3(-1,0,0));

		//neck = kPM->CreateBodySphericalJoint(head,torso,NxVec3(0,8.8,0),NxVec3(0,1,0));
		//leftShoulder = kPM->CreateBodySphericalJoint(leftUpperArm,torso,NxVec3(0.5,8.5,0),NxVec3(1,0,0));
		//rightShoulder = kPM->CreateBodySphericalJoint(rightUpperArm,torso,NxVec3(-0.5,8.5,0),NxVec3(-1,0,0));
		//spine = kPM->CreateBodySphericalJoint(torso,pelvis,NxVec3(0,7,0),NxVec3(0,-1,0));
		//leftHip = kPM->CreateBodySphericalJoint(leftThigh,pelvis,NxVec3(0.6,6,0),NxVec3(0,-1,0));
		//rightHip = kPM->CreateBodySphericalJoint(rightThigh,pelvis,NxVec3(-0.6,6,0),NxVec3(0,-1,0));
		//leftElbow = kPM->CreateRevoluteJoint(leftForeArm,leftUpperArm,NxVec3(2,8.5,0),NxVec3(0,0,-1));
		//rightElbow = kPM->CreateRevoluteJoint(rightForeArm,rightUpperArm,NxVec3(-2,8.5,0),NxVec3(0,0,-1));
		//leftWrist = kPM->CreateRevoluteJoint2(leftHand,leftForeArm,NxVec3(0,-0.15,0),NxVec3(0,1.3,0),NxVec3(0,1,0),NxVec3(0,1,0));
		//rightWrist = kPM->CreateRevoluteJoint2(rightHand,rightForeArm,NxVec3(0,-0.15,0),NxVec3(0,1.3,0),NxVec3(0,1,0),NxVec3(0,1,0));
		//leftKnee = kPM->CreateRevoluteJoint(leftCalf,leftThigh,NxVec3(0.6,3.5,0),NxVec3(1,0,0));
		//rightKnee = kPM->CreateRevoluteJoint(rightCalf,rightThigh,NxVec3(-0.6,3.5,0),NxVec3(-1,0,0));
		//leftAnkle = kPM->CreateRevoluteJoint(leftFoot,leftCalf,NxVec3(0.6,1.3,0),NxVec3(1,0,0));
		//rightAnkle = kPM->CreateRevoluteJoint(rightFoot,rightCalf,NxVec3(-0.6,1.3,0),NxVec3(-1,0,0));


	};

	NxActor* head;
	NxActor* torso;
	NxActor* pelvis;
	NxActor* leftUpperArm;
	NxActor* rightUpperArm;
	NxActor* leftForeArm;
	NxActor* rightForeArm;
	NxActor* leftHand;
	NxActor* rightHand;
	NxActor* leftThigh;
	NxActor* rightThigh;
	NxActor* leftCalf;
	NxActor* rightCalf;
	NxActor* leftFoot;
	NxActor* rightFoot;

	NxFixedJoint* neck;
	NxFixedJoint* leftShoulder;
	NxFixedJoint* rightShoulder;
	NxFixedJoint* spine;
	NxFixedJoint* leftHip;
	NxFixedJoint* rightHip;
	NxFixedJoint* leftElbow;
	NxFixedJoint* rightElbow;
	NxFixedJoint* leftWrist;
	NxFixedJoint* rightWrist;
	NxFixedJoint* leftKnee;
	NxFixedJoint* rightKnee;
	NxFixedJoint* leftAnkle;
	NxFixedJoint* rightAnkle;

	//NxSphericalJoint* neck;
	//NxSphericalJoint* leftShoulder;
	//NxSphericalJoint* rightShoulder;
	//NxSphericalJoint* spine;
	//NxSphericalJoint* leftHip;
	//NxSphericalJoint* rightHip;
	//NxRevoluteJoint* leftElbow;
	//NxRevoluteJoint* rightElbow;
	//NxRevoluteJoint* leftWrist;
	//NxRevoluteJoint* rightWrist;
	//NxRevoluteJoint* leftKnee;
	//NxRevoluteJoint* rightKnee;
	//NxRevoluteJoint* leftAnkle;
	//NxRevoluteJoint* rightAnkle;
};

// Actor globals
//extern NxActor* groundPlane;

Ragdoll* guy = NULL;

// Focus actor
extern NxActor* gSelectedActor;


void nxPhysManager::nxSampleRagdoll()
{
	Con::errorf("making sample ragdoll");

	guy = new Ragdoll(NxVec3(0,0,0));

}

ConsoleFunction( nxSampleRagdoll, void, 1,1, "")
{
	((nxPhysManager *)physManagerCommon::getPM())->nxSampleRagdoll();
	return;
}

//////////////////////////////////////

int nxPhysManager::addSceneEvent(int eventType,iPhysUser *physUser,float time,float duration,int node,Ogre::Vector3 &value,const char *action,int dbID)
{
	ecstasySceneEvent *kEvent = NULL;
	ecstasySceneEvent *p;
	ecstasySceneEvent *pEventIterator = NULL;

	//FIX: need to test for iterator->time == time, (and other fields where relevant) to determing NOT to alloc a new event, because we are going to just update values
	if ( (eventType > SE_IMP_NULL) && (eventType < SE_DUR_NULL) )
	{
		for (pEventIterator = mImpulseEventList.next(pEventIterator); pEventIterator;
			pEventIterator = mImpulseEventList.next(pEventIterator))
			if ((pEventIterator->time > time)&&(!kEvent))
				kEvent = mImpulseEventList.alloc(pEventIterator);
		if (!kEvent) kEvent = mImpulseEventList.alloc();
	}
	else if ( (eventType > SE_DUR_NULL) && (eventType < SE_INTERP_NULL) )
	{
		for (pEventIterator = mDurationEventList.next(pEventIterator); pEventIterator;
			pEventIterator = mDurationEventList.next(pEventIterator))
			if ((pEventIterator->time > time)&&(!kEvent))
				kEvent = mDurationEventList.alloc(pEventIterator);
		if (!kEvent) kEvent = mDurationEventList.alloc();
	}
	else if ( (eventType > SE_INTERP_NULL) && (eventType < SE_FOLLOW_NULL) )
	{
		for (pEventIterator = mInterpolationEventList.next(pEventIterator); pEventIterator;
			pEventIterator = mInterpolationEventList.next(pEventIterator))
			if ((pEventIterator->time > time)&&(!kEvent))
				kEvent = mInterpolationEventList.alloc(pEventIterator);
		if (!kEvent) kEvent = mInterpolationEventList.alloc();
		kEvent->prev = NULL;  kEvent->next = NULL;
	}
	else if (eventType > SE_FOLLOW_NULL)
	{
		//With follow events, most of them will all pile up at the beginning if we don't put them at the end explicitly
		if (time > 0) 
		{
			for (pEventIterator = mFollowEventList.next(pEventIterator); pEventIterator;
				pEventIterator = mFollowEventList.next(pEventIterator))
				if ((pEventIterator->time > time)&&(!kEvent))
					kEvent = mFollowEventList.alloc(pEventIterator);
		}
		if (!kEvent) kEvent = mFollowEventList.alloc();
		kEvent->prev = NULL;  kEvent->next = NULL;
	}

	if (kEvent)
	{
		kEvent->eventID = mSceneEventCounter++;//Only one counter though, because it's just for reference.

		kEvent->eventType = eventType;
		kEvent->physUser = physUser;
		kEvent->dbID = dbID;
		kEvent->time = time;
		kEvent->duration = duration;
		kEvent->node = node;
		kEvent->value = value;
		kEvent->action.clear();
		kEvent->action.insert(0,action);
		fxFlexBody *kFB = dynamic_cast<fxFlexBody *>(kEvent->physUser);

		if ( (eventType > SE_INTERP_NULL) && (eventType < SE_FOLLOW_NULL) )
		{
			//HERE: Have to hook up prev and next pointers, do a search for all events of this type 
			//with this physUser and this node number, and find the last one with time before this time.
			//If it has duration == -1.0, it is our previous pointer and we are its next pointer.
			//If it has duration == 0, it has no next pointer, it is the end of a chain.
			//If it has duration == -2.0, it is our first follow event
			//Meanwhile, if WE have start time == -1 and duration > 0.0, we are a not-first follow event, so we need to 
			//skip the start time sort and set up our prev-next pointers in a separate sort based on duration.
			//(I hope this isn't a ridiculously overcomplicated solution.  Could just add a new list, mFollowEventList.)
			if (mInterpolationEventList.size())
			{
				ecstasySceneEvent *pEventIterator = NULL;
				ecstasySceneEvent *lastEvent = NULL;
				ecstasySceneEvent *nextEvent = NULL;
				int done=0;
				for (pEventIterator = mInterpolationEventList.next(pEventIterator); pEventIterator;
					pEventIterator = mInterpolationEventList.next(pEventIterator))
				{
					if ((pEventIterator->eventType == eventType) &&
						(pEventIterator->physUser == physUser) &&
						(pEventIterator->node == node)&&
						(pEventIterator!=kEvent)&&(!done))
					{
						if (pEventIterator->time == time)
						{//HERE: overwrite existing event, but keep going since we might still need to 
							//find our next event (can't use this one, because it might be null).
							pEventIterator->value = kEvent->value;
							//pEventIterator->action = kEvent->action;
						} 
						else if (pEventIterator->time > time)
						{
							nextEvent = pEventIterator;
							done=1;
						} else lastEvent = pEventIterator;
					}
				}
				if (lastEvent)
				{
					if (lastEvent->duration == -1.0)
					{
						lastEvent->next = kEvent;
						kEvent->prev = lastEvent;
					}
				}
				if (nextEvent)
				{
					if (kEvent->duration == -1.0)
					{
						kEvent->next = nextEvent;
						nextEvent->prev = kEvent;
					}
				}
			}
		} else if (eventType > SE_FOLLOW_NULL) {
			if (kEvent->time >= 0.0)
			{//We have a first follow event
				if (mFollowEventList.size())
				{
					ecstasySceneEvent *pEventIterator = NULL;
					ecstasySceneEvent *lastEvent = NULL;
					ecstasySceneEvent *nextEvent = NULL;
					int done=0;
					for (pEventIterator = mFollowEventList.next(pEventIterator); pEventIterator;
						pEventIterator = mFollowEventList.next(pEventIterator))
					{//HERE: follow nodes can only have one per physUser, not one per node and type.  All types in same list, and node is usually -1.
						if ((pEventIterator->physUser == physUser) &&
							(pEventIterator!=kEvent)&&(!done))
						{
							if (pEventIterator->time == time)
							{//HERE: overwrite existing event, but keep going since we might still need to 
								//find our next event (can't use this one, because it might be null).
								pEventIterator->value = kEvent->value;
								pEventIterator->action = kEvent->action;
								pEventIterator->eventType = kEvent->eventType;
								pEventIterator->node = kEvent->node;
							} 
						}
					}
					kFB->mFirstFollowEvent = kEvent;
				}
			} else {//else we have a regular follow event, time==-1.0, so sort by duration (order)
				if (kFB->mFirstFollowEvent)
				{
					int numFollowEvents = 1;
					ecstasySceneEvent *pEventIterator = kFB->mFirstFollowEvent;
					while (pEventIterator->next)
					{//HERE: follow nodes can only have one per physUser, not one per node and type.  All types in same list, and node is usually -1.
						numFollowEvents++;
						pEventIterator = pEventIterator->next;
						if (pEventIterator->duration > kEvent->duration)
						{//we found someone to insert ourselves in front of.
							if (pEventIterator->prev)
								kEvent->prev = pEventIterator->prev;
							pEventIterator->prev = kEvent;
							kEvent->next = pEventIterator;
						}
					}
					//if (kEvent->prev == NULL)//meaning we are the second follow event added for this flexbody.
					if (numFollowEvents == 1)
					{
						kEvent->prev = kFB->mFirstFollowEvent;
						kFB->mFirstFollowEvent->next = kEvent;
					} else {//else we're just adding it on to the end of the list.
						pEventIterator->next = kEvent;
						kEvent->prev = pEventIterator;
					}
				}
			}
		}
		return kEvent->eventID;
	}
	return -1;	
}


ConsoleFunction( addSceneEvent, int, 2,9, "addSceneEvent(int eventType,int physUser,float time,float duration,int node,Ogre::Vector3 value,const char *action,int dbID)")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	iPhysUser *kPU;// = (get this from ghost ID)
	fxFlexBody *kFB;
	int ghostID,cause,node,type,index;
	float time,duration;
	Ogre::Vector3 value;
	String action;
	int dbID = 0;

	char sceneEvent_id_query[512],scene_id_query[512],insert_query[512];
	int actor_id,body_id,joint_id,sceneEvent_id,scene_id,result;
	sqlite_resultset *resultSet;

	//Only requirement: there must be a type!
	type = dAtoi(argv[1]);
	
	//HERE: find iPhysUser * from ghostID
	if (argc>=3) {
		ghostID = dAtoi(argv[2]);

		NetConnection *toServer =  NetConnection::getConnectionToServer();
		NetConnection *toClient = NetConnection::getLocalClientConnection();

		NetObject *netObj = toClient->resolveObjectFromGhostIndex(ghostID);
		index = toClient->getGhostIndex(netObj);
		NetObject *clientNetObj = toServer->resolveGhost(index);
		kPU = dynamic_cast<iPhysUser *>(clientNetObj);
		kFB = dynamic_cast<fxFlexBody *>(kPU);
	} else kPU = NULL;
	
	if (argc>=4) time = dAtof(argv[3]);
	else time = 0.0;
	if (argc>=5) duration = dAtof(argv[4]);
	else duration = 0.0;
	if (argc>=6) node = dAtoi(argv[5]);
	else node = -1;
	if (argc>=7) dSscanf(argv[6],"%g %g %g",&value.x,&value.y,&value.z);
	else value.zero();
	if (argc>=8) action.insert(0,argv[7]);
	else action.clear();
	if (argc==9) dbID = dAtoi(argv[8]);
	else dbID = -1;
	
	Con::errorf("adding scene event: type %d value %f %f %f",type,value.x,value.y,value.z);
	kPM->mSQL = new SQLiteObject();
	if (kPM->mSQL) 
	{
		kFB = dynamic_cast<fxFlexBody *>(kPU);
		if (kPM->mSQL->OpenDatabase("EcstasyMotion.db"))
		{
			scene_id = 0;
			sprintf(scene_id_query,"SELECT id FROM scene WHERE name='%s';",kPM->mSceneName.c_str());
			result = kPM->mSQL->ExecuteSQL(scene_id_query);
			resultSet = kPM->mSQL->GetResultSet(result);
			if (resultSet->iNumRows==1) 
			{
				scene_id = dAtoi(resultSet->vRows[0]->vColumnValues[0]);
			}
			if (scene_id>0)
			{
				sprintf(sceneEvent_id_query,"SELECT id FROM sceneEvent WHERE actor_id = %d AND scene_id = %d AND type = %d AND node = %d AND time = %f AND action = '%s';",
					kFB->mActorId,scene_id,type,node,time,action);
				result = kPM->mSQL->ExecuteSQL(sceneEvent_id_query);
				resultSet = kPM->mSQL->GetResultSet(result);
				if (resultSet->iNumRows == 0)  
				{
					sprintf(insert_query,"INSERT INTO sceneEvent (actor_id,scene_id,type,node,time,duration,value_x,value_y,value_z,action) VALUES (%d,%d,%d,%d,%f,%f,%f,%f,%f,'%s');",
						kFB->mActorId,scene_id,type,node,time,duration,value.x,value.y,value.z,action.c_str());

					//Con::errorf("scene event query: %s",insert_query);
					result = kPM->mSQL->ExecuteSQL(insert_query);
					result = kPM->mSQL->ExecuteSQL(sceneEvent_id_query);
					resultSet = kPM->mSQL->GetResultSet(result);
					if (resultSet->iNumRows == 1)
						dbID = dAtoi(resultSet->vRows[0]->vColumnValues[0]);

				} else {//shouldn't get here, but just in case...
					dbID = dAtoi(resultSet->vRows[0]->vColumnValues[0]);
					sprintf(insert_query,"UPDATE sceneEvent SET duration=%f, value_x=%f, value_y=%f, value_z=%f,action='%s') WHERE id=%d;",
						duration,value.x,value.y,value.z,action.c_str(),dbID);
					result = kPM->mSQL->ExecuteSQL(insert_query);
				}
			}
			kPM->mSQL->CloseDatabase();
			delete kPM->mSQL;
		}
	}
	return kPM->addSceneEvent(type,kPU,time,duration,node,value,action.c_str(),dbID);	
}

ConsoleFunction( saveSceneEvent, bool, 2, 9, "saveSceneEvent(int eventID,int eventType,int physUser,float time,float duration,int node,Ogre::Vector3 value,const char *action)")
{

	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	iPhysUser *kPU;// = (get this from ghost ID)
	fxFlexBody *kFB;
	int ghostID,cause,node,type,index;
	float time,duration;
	Ogre::Vector3 value;
	String action;

	char sceneEvent_id_query[512],scene_id_query[512],insert_query[512];
	int actor_id,body_id,joint_id,sceneEvent_id,scene_id,result;
	sqlite_resultset *resultSet;

	//Only requirement: there must be a type!
	int dbEventID = dAtoi(argv[1]);
	type = dAtoi(argv[2]);
	
	//HERE: find iPhysUser * from ghostID
	if (argc>=4) {
		ghostID = dAtoi(argv[3]);

		NetConnection *toServer =  NetConnection::getConnectionToServer();
		NetConnection *toClient = NetConnection::getLocalClientConnection();

		NetObject *netObj = toClient->resolveObjectFromGhostIndex(ghostID);
		index = toClient->getGhostIndex(netObj);
		NetObject *clientNetObj = toServer->resolveGhost(index);
		kPU = dynamic_cast<iPhysUser *>(clientNetObj);

		// Yup, this works.  Hm, wow, didn't know you could do that.  iPhysUser
		// isn't even a netobject, but everybody who inherits from it is, so I guess it's okay.
		kFB = dynamic_cast<fxFlexBody *>(kPU);
		//if (kFB) kFB->stopAnimating();

	} else kPU = NULL;
	
	if (argc>=5) time = dAtof(argv[4]);
	else time = 0.0;
	if (argc>=6) duration = dAtof(argv[5]);
	else duration = 0.0;
	if (argc>=7) node = dAtoi(argv[6]);
	else node = -1;
	if (argc>=8) dSscanf(argv[7],"%g %g %g",&value.x,&value.y,&value.z);
	else value.zero();
	if (argc>=9) action.insert(0,argv[8]);
	else action.clear();
	if (argc==10) cause = dAtoi(argv[9]);
	else cause = -1;
	
	Con::errorf("saving scene event: type %d value %f %f %f",type,value.x,value.y,value.z);
	kPM->mSQL = new SQLiteObject();
	if (kPM->mSQL) 
	{
		kFB = dynamic_cast<fxFlexBody *>(kPU);
		if (kPM->mSQL->OpenDatabase("EcstasyMotion.db"))
		{
			scene_id = 0;
			sprintf(scene_id_query,"SELECT id FROM scene WHERE name='%s';",kPM->mSceneName.c_str());
			result = kPM->mSQL->ExecuteSQL(scene_id_query);
			resultSet = kPM->mSQL->GetResultSet(result);
			if (resultSet->iNumRows==1) 
			{
				scene_id = dAtoi(resultSet->vRows[0]->vColumnValues[0]);
			}
			if (scene_id>0)
			{
				sprintf(insert_query,"UPDATE sceneEvent SET actor_id=%d,type=%d,time=%f,duration=%f, value_x=%f, value_y=%f, value_z=%f,action='%s' WHERE id=%d;",
					kFB->mActorId,type,time,duration,value.x,value.y,value.z,action.c_str(),dbEventID);
				result = kPM->mSQL->ExecuteSQL(insert_query);
				
			}
			kPM->mSQL->CloseDatabase();
			delete kPM->mSQL;
		}
	}
	return true;
}

ConsoleFunction( dropSceneEvent, bool, 3, 3, "dropSceneEvent(int eventID, int DatabaseID)")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	ecstasySceneEvent *pEventIterator = NULL;
	int result;
	char delete_query[512];
	
	int eventID = dAtoi(argv[1]);
	int dbEventID = dAtoi(argv[2]);
	kPM->mSQL = new SQLiteObject();
	if (kPM->mSQL) 
	{
		if (kPM->mSQL->OpenDatabase("EcstasyMotion.db"))
		{
			sprintf(delete_query,"DELETE FROM sceneEvent WHERE id=%d;",dbEventID);
			result = kPM->mSQL->ExecuteSQL(delete_query);
			
			kPM->mSQL->CloseDatabase();
			delete kPM->mSQL;
		}
	}

	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
		if (pEventIterator->eventID == eventID)
		{
			kPM->mImpulseEventList.unlink(pEventIterator);
			return true;
		}

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
		if (pEventIterator->eventID == eventID)
		{
			kPM->mDurationEventList.unlink(pEventIterator);
			return true;
		}

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
		if (pEventIterator->eventID == eventID)
		{
			kPM->mInterpolationEventList.unlink(pEventIterator);
			return true;
		}

	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
		if (pEventIterator->eventID == eventID)
		{
			kPM->mFollowEventList.unlink(pEventIterator);
			return true;
		}

	//ecstasySceneEvent *pEventIterator = NULL;
	//for (pEventIterator = kPM->mSceneEventList.next(pEventIterator); pEventIterator;
	//	pEventIterator = kPM->mSceneEventList.next(pEventIterator))
	//{
	//	if (pEventIterator->eventID == dAtoi(argv[1]))
	//	{
	//		kPM->mSceneEventList.free(pEventIterator);
	//	}
	//}
	return false;
}

ConsoleFunction( startSceneEvents, bool, 1, 2, "startSceneEvents(float time)")
{
	Con::printf("starting scene events!");
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	if (kPM->mImpulseEventList.size() ||
		kPM->mDurationEventList.size() ||
		kPM->mInterpolationEventList.size() ||
		kPM->mFollowEventList.size() )
	{
		kPM->mSceneStartStep = Sim::getCurrentTime();
		kPM->mSceneStartTime = 0.0;
		kPM->mLastStepTime = 0.0;
		kPM->mLastImpulseEvent = NULL;
		kPM->mLastDurationEvent = NULL;
		kPM->mLastInterpolationEvent = NULL;
		kPM->mLastFollowEvent = NULL;
		//if (argc>1) kPM->mSceneStartTime = dAtof(argv[1]);
		//else kPM->mSceneStartTime = 0.0;//LATER, we might try to let them start
		//a scene in the middle.  For now, only the beginning.
		return true;
	}
	return false;
}

int nxPhysManager::getNumSceneEvents(int eventType,iPhysUser *physUser,int node)
{
	int numEvents = 0;
	ecstasySceneEvent *p = NULL;
	if ( (eventType > SE_IMP_NULL) && (eventType < SE_DUR_NULL) )
	{
		for (p = mImpulseEventList.next(p); p; p = mImpulseEventList.next(p))
			if ((p->eventType == eventType)&&
				((p->physUser == physUser)||(physUser == NULL))&&
				((p->node == node)||(node == -1)))
				numEvents++;
	}
	else if ( (eventType > SE_DUR_NULL) && (eventType < SE_INTERP_NULL) )
	{
		for (p = mDurationEventList.next(p); p;	p = mDurationEventList.next(p))
			if ((p->eventType == eventType)&&
				((p->physUser == physUser)||(physUser == NULL))&&
				((p->node == node)||(node == -1)))
				numEvents++;
	}
	else if ( (eventType > SE_INTERP_NULL) && (eventType < SE_FOLLOW_NULL)  )
	{
		for (p = mInterpolationEventList.next(p); p; p = mInterpolationEventList.next(p))
			if ((p->eventType == eventType)&&
				((p->physUser == physUser)||(physUser == NULL))&&
				((p->node == node)||(node == -1)))
				numEvents++;
	}
	else if (eventType > SE_FOLLOW_NULL)
	{
		for (p = mFollowEventList.next(p); p; p = mFollowEventList.next(p))
			if ((p->eventType == eventType)&&
				((p->physUser == physUser)||(physUser == NULL))&&
				((p->node == node)||(node == -1)))
				numEvents++;
	}
	else 
	{//Grab all of them for this physUser if we have one, or else just all of them. 
		for (p = mImpulseEventList.next(p); p; p = mImpulseEventList.next(p))
			if ((p->physUser == physUser)||(physUser == NULL))
				numEvents++;
		for (p = mDurationEventList.next(p); p;	p = mDurationEventList.next(p))
			if ((p->physUser == physUser)||(physUser == NULL))
				numEvents++;
		for (p = mInterpolationEventList.next(p); p; p = mInterpolationEventList.next(p))
			if ((p->physUser == physUser)||(physUser == NULL))
				numEvents++;
		for (p = mFollowEventList.next(p); p; p = mFollowEventList.next(p))
			if ((p->physUser == physUser)||(physUser == NULL))
				numEvents++;
	}

	return numEvents;
}



//Okay, this needs to get done.  You have to have a type, but you don't have to have an actor
//or a node.  If you have no tweaker_bot selected, then actor and node will be null, but you can
//still create global events, esp. script events.
ConsoleFunction( getNumSceneEvents, int, 2, 4, "getNumSceneEvents(int type,int actorID,int node)")
{
	int type = dAtoi(argv[1]);
	int actorID = -1;
	int nodeID = -1;
	iPhysUser *kPU = NULL;

	if (argc>2)
		actorID = dAtoi(argv[2]);
	if (argc>3)
		nodeID = dAtoi(argv[3]);

	if (actorID>0)
	{
		NetConnection *toServer =  NetConnection::getConnectionToServer();
		NetConnection *toClient = NetConnection::getLocalClientConnection();
		NetObject *netObj = toClient->resolveObjectFromGhostIndex(actorID);
		int index = toClient->getGhostIndex(netObj);
		NetObject *clientNetObj = toServer->resolveGhost(index);
		kPU = dynamic_cast<iPhysUser *>(clientNetObj);
	}
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	return kPM->getNumSceneEvents(type,kPU,nodeID);
}


int nxPhysManager::getEventIdByNum(int eventType,iPhysUser *physUser,int node,int eventNum)
{
	int numEvents = 0;
	int eventID = -1;
	ecstasySceneEvent *p = NULL;
	if ( (eventType > SE_IMP_NULL) && (eventType < SE_DUR_NULL) )
	{
		for (p = mImpulseEventList.next(p); p; p = mImpulseEventList.next(p))
		{
			if ((p->eventType == eventType)&&
				((p->physUser == physUser)||(physUser == NULL))&&
				((p->node == node)||(node == -1)))
			{
				if (numEvents == eventNum) 
					eventID = p->eventID;
				numEvents++;

			}
		}
	}
	else if ( (eventType > SE_DUR_NULL) && (eventType < SE_INTERP_NULL) )
	{
		for (p = mDurationEventList.next(p); p;	p = mDurationEventList.next(p))
		{
			if ((p->eventType == eventType)&&
				((p->physUser == physUser)||(physUser == NULL))&&
				((p->node == node)||(node == -1)))
			{
				if (numEvents == eventNum) 
					eventID = p->eventID;
				numEvents++;
			}
		}
	}
	else if ( (eventType > SE_INTERP_NULL) && (eventType < SE_FOLLOW_NULL) )
	{
		for (p = mInterpolationEventList.next(p); p; p = mInterpolationEventList.next(p))
		{
			if ((p->eventType == eventType)&&
				((p->physUser == physUser)||(physUser == NULL))&&
				((p->node == node)||(node == -1)))
			{
				if (numEvents == eventNum) 
					eventID = p->eventID;
				numEvents++;
			}
		}
	}
	else if (eventType > SE_FOLLOW_NULL)
	{
		for (p = mFollowEventList.next(p); p; p = mFollowEventList.next(p))
		{
			if ((p->eventType == eventType)&&
				((p->physUser == physUser)||(physUser == NULL))&&
				((p->node == node)||(node == -1)))
			{
				if (numEvents == eventNum) 
					eventID = p->eventID;
				numEvents++;
			}
		}
	}
	else //If type==0, grab whole list for physUser,
	{//  or for all if physUser is also null.
		for (p = mImpulseEventList.next(p); p; p = mImpulseEventList.next(p))
		{
			if ((p->physUser == physUser)||(physUser == NULL))
			{
				if (numEvents == eventNum) 
					eventID = p->eventID;
				numEvents++;
			}
		}
		for (p = mDurationEventList.next(p); p;	p = mDurationEventList.next(p))
		{
			if ((p->physUser == physUser)||(physUser == NULL))
			{
				if (numEvents == eventNum) 
					eventID = p->eventID;
				numEvents++;
			}
		}
		for (p = mInterpolationEventList.next(p); p; p = mInterpolationEventList.next(p))
		{
			if ((p->physUser == physUser)||(physUser == NULL))
			{
				if (numEvents == eventNum) 
					eventID = p->eventID;
				numEvents++;
			}
		}
		for (p = mFollowEventList.next(p); p; p = mFollowEventList.next(p))
		{
			if ((p->physUser == physUser)||(physUser == NULL))
			{
				if (numEvents == eventNum) 
					eventID = p->eventID;
				numEvents++;
			}
		}
	}
	return eventID;
}

ConsoleFunction( getEventIdByNum, int, 5, 5, "getEventIdByNum(int type,int actorID,int node,int eventNum)")
{
	int type = 0;
	int actorID = -1;
	int nodeID = -1;
	int eventNum = -1;
	iPhysUser *kPU = NULL;

	if (argc>1)
		type = dAtoi(argv[1]);
	if (argc>2)
		actorID = dAtoi(argv[2]);
	if (argc>3)
		nodeID = dAtoi(argv[3]);
	if (argc>4)
		eventNum = dAtoi(argv[4]);

	if (actorID>0)
	{
		NetConnection *toServer =  NetConnection::getConnectionToServer();
		NetConnection *toClient = NetConnection::getLocalClientConnection();
		NetObject *netObj = toClient->resolveObjectFromGhostIndex(actorID);
		int index = toClient->getGhostIndex(netObj);
		NetObject *clientNetObj = toServer->resolveGhost(index);
		kPU = dynamic_cast<iPhysUser *>(clientNetObj);
	}
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	return kPM->getEventIdByNum(type,kPU,nodeID,eventNum);
}


int nxPhysManager::getEventIdByTime(int eventType,iPhysUser *physUser,int node,float startTime)
{
	int eventID = -1;
	ecstasySceneEvent *p = NULL;
	
	if ((physUser==NULL)||(eventType<SE_IMP_NULL)||(node==-1)||(startTime<0))
		return -1;

	if ( (eventType > SE_IMP_NULL) && (eventType < SE_DUR_NULL) )
	{
		for (p = mImpulseEventList.next(p); p; p = mImpulseEventList.next(p))
		{
			if ((p->eventType == eventType)&&
				(p->physUser == physUser)&&
				(p->node == node))
				if (p->time == startTime) 
					eventID = p->eventID;
		}
	}
	else if ( (eventType > SE_DUR_NULL) && (eventType < SE_INTERP_NULL) )
	{
		for (p = mDurationEventList.next(p); p;	p = mDurationEventList.next(p))
		{
			if ((p->eventType == eventType)&&
				(p->physUser == physUser)&&
				(p->node == node))
				if (p->time == startTime) 
					eventID = p->eventID;
		}
	}
	else if ( (eventType > SE_INTERP_NULL) && (eventType < SE_FOLLOW_NULL) )
	{
		for (p = mInterpolationEventList.next(p); p; p = mInterpolationEventList.next(p))
		{
			if ((p->eventType == eventType)&&
				(p->physUser == physUser)&&
				(p->node == node))
				if (p->time == startTime) 
					eventID = p->eventID;
		}
	}
	else if (eventType > SE_FOLLOW_NULL)
	{
		for (p = mFollowEventList.next(p); p; p = mFollowEventList.next(p))
		{
			if ((p->eventType == eventType)&&
				(p->physUser == physUser)&&
				(p->node == node))
				if (p->time == startTime) 
					eventID = p->eventID;
		}
	}
	return eventID;
}

ConsoleFunction( getEventIdByTime, int, 5, 5, "getEventIdByTime(int type,int actorID,int node,float startTime)")
{
	int type = dAtoi(argv[1]);
	int actorID = -1;
	int nodeID = -1;
	float startTime = 0.0;
	iPhysUser *kPU = NULL;

	if (argc>2)
		actorID = dAtoi(argv[2]);
	if (argc>3)
		nodeID = dAtoi(argv[3]);
	if (argc>4)
		startTime = dAtof(argv[4]);

	if (actorID>0)
	{
		NetConnection *toServer =  NetConnection::getConnectionToServer();
		NetConnection *toClient = NetConnection::getLocalClientConnection();
		NetObject *netObj = toClient->resolveObjectFromGhostIndex(actorID);
		int index = toClient->getGhostIndex(netObj);
		NetObject *clientNetObj = toServer->resolveGhost(index);
		kPU = dynamic_cast<iPhysUser *>(clientNetObj);
	}
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	return kPM->getEventIdByTime(type,kPU,nodeID,startTime);
}
//NOW, after above works, also need getEventTime, getEventDuration, getEventValue, getEventAction 
//for (type, actorID, node, eventNum) arguments.


void nxPhysManager::clearSceneEvents()
{
	mImpulseEventList.reset();
	mDurationEventList.reset();
	mInterpolationEventList.reset();
	mCurrDurationEvents.clear();
	mCurrInterpolationEvents.clear();

	mSceneEventCounter = 0;
	mSceneStartStep = 0;
	mSceneStartTime = 0.0;
	mSceneDuration = 0.0;
	mLastImpulseEvent = NULL;
	mLastDurationEvent = NULL;
	mLastInterpolationEvent = NULL;
	mLastFollowEvent = NULL;

}

void nxPhysManager::saveSceneEventsToFile(const char *filename)
{
	//So... just need to iterate through all three lists, and write it all out to a file.
	if (dStrlen(filename)==0) { 
		Con::errorf("saveSceneEvents called with zero length filename."); 
		return;
	}

	int botNum;
	FILE *fpw = NULL;
	String eventsFile(filename);
	String eventsExt(".events");
	if (!dStrstr(eventsFile.c_str(),".events")) eventsFile += eventsExt;

	fpw = fopen(eventsFile.c_str(),"w");
	ecstasySceneEvent *ev = NULL;

	for (ev = mImpulseEventList.next(ev); ev;
		ev = mImpulseEventList.next(ev))
	{
		if (ev->physUser)
			botNum = dynamic_cast<fxFlexBody *>(ev->physUser)->mBotNumber;
		else botNum = -1;
		fprintf(fpw,"%d;%f;%f;%d;%d;%d;(%f %f %f); %s ;\n",ev->eventType,ev->time,ev->duration,
			botNum,ev->cause,ev->node,ev->value.x,ev->value.y,ev->value.z,ev->action.c_str());
	}

	for (ev = mDurationEventList.next(ev); ev;
		ev = mDurationEventList.next(ev))
	{
		if (ev->physUser)
			botNum = dynamic_cast<fxFlexBody *>(ev->physUser)->mBotNumber;
		else botNum = -1;
		fprintf(fpw,"%d;%f;%f;%d;%d;%d;(%f %f %f); %s ;\n",ev->eventType,ev->time,ev->duration,
			botNum,ev->cause,ev->node,ev->value.x,ev->value.y,ev->value.z,ev->action.c_str());
	}

	for (ev = mInterpolationEventList.next(ev); ev;
		ev = mInterpolationEventList.next(ev))
	{
		if (ev->physUser)
			botNum = dynamic_cast<fxFlexBody *>(ev->physUser)->mBotNumber;
		else botNum = -1;
		fprintf(fpw,"%d;%f;%f;%d;%d;%d;(%f %f %f); %s ;\n",ev->eventType,ev->time,ev->duration,
			botNum,ev->cause,ev->node,ev->value.x,ev->value.y,ev->value.z,ev->action.c_str());
	}

	for (ev = mFollowEventList.next(ev); ev;
		ev = mFollowEventList.next(ev))
	{
		if (ev->physUser)
			botNum = dynamic_cast<fxFlexBody *>(ev->physUser)->mBotNumber;
		else botNum = -1;
		fprintf(fpw,"%d;%f;%f;%d;%d;%d;(%f %f %f); %s ;\n",ev->eventType,ev->time,ev->duration,
			botNum,ev->cause,ev->node,ev->value.x,ev->value.y,ev->value.z,ev->action.c_str());
	}
	fclose(fpw);
	return;
}

void nxPhysManager::loadSceneEvents(int scene_id)//(const char *scenename)
{
	float time,duration,x,y,z;
	int botNum,cause,type,node;
	char action[255];	
	char buf[1000];
	char *bufp;
	
	char sceneEvent_id_query[512],scene_name_query[512],mission_id_query[512],insert_query[512];
	int actor_id,body_id,joint_id,sceneEvent_id,result,db_id;
	fxFlexBody *kFB;
	sqlite_resultset *resultSet;

	FILE *fp = NULL;
	iPhysUser *kPU = NULL;

	if (scene_id<=0)
	{
		Con::errorf("nxPhysManager trying to load scene events, invalid scene_id!");
		return;
	} else 
		Con::printf("nxPhysManager loading scene events...");

	mSQL = new SQLiteObject();
	if (mSQL) 
	{
		mSQL->OpenDatabase("EcstasyMotion.db");
		clearSceneEvents();
		mSceneId = scene_id;

		SimSet *playBotGroup = NULL;
		playBotGroup = dynamic_cast<SimSet *>(Sim::findObject("PlayBotGroup"));
		if (playBotGroup) Con::printf("playbotgroup size: %d",playBotGroup->size());
		else return;

		sprintf(scene_name_query,"SELECT name FROM scene WHERE id=%d;",mSceneId);
		result = mSQL->ExecuteSQL(scene_name_query);
		resultSet = mSQL->GetResultSet(result);
		if (resultSet->iNumRows==1) 
			mSceneName = resultSet->vRows[0]->vColumnValues[0];
				
		sprintf(sceneEvent_id_query,"SELECT id,actor_id,type,node,time,duration,value_x,value_y,value_z,action FROM sceneEvent WHERE scene_id = %d;",mSceneId);
		result = mSQL->ExecuteSQL(sceneEvent_id_query);
		resultSet = mSQL->GetResultSet(result);
		for (unsigned int i=0;i<resultSet->iNumRows;i++)
		{
			db_id = dAtoi(resultSet->vRows[i]->vColumnValues[0]);
			actor_id = dAtoi(resultSet->vRows[i]->vColumnValues[1]);
			//HERE: need to get a physUser kPU out of actor_id somehow.
			//Could run through botgroup to find one where actor_id == this actor_id.
			for (unsigned int j=0;j<playBotGroup->size();j++)
			{
				int botActorId = dynamic_cast<fxFlexBody *>(playBotGroup->at(j))->mActorId;
				if (botActorId == actor_id)
					kPU = dynamic_cast<iPhysUser *>(playBotGroup->at(j));
			}
			type = dAtoi(resultSet->vRows[i]->vColumnValues[2]);
			node = dAtoi(resultSet->vRows[i]->vColumnValues[3]);
			time = dAtof(resultSet->vRows[i]->vColumnValues[4]);
			duration = dAtof(resultSet->vRows[i]->vColumnValues[5]);
			x = dAtof(resultSet->vRows[i]->vColumnValues[6]);
			y = dAtof(resultSet->vRows[i]->vColumnValues[7]);
			z = dAtof(resultSet->vRows[i]->vColumnValues[8]);
			sprintf(action,"%s",resultSet->vRows[i]->vColumnValues[9]);

			if ((type)&&(kPU)) 
			{
				addSceneEvent(type,kPU,time,duration,node,Ogre::Vector3(x,y,z),action,db_id);
				Con::printf("adding a scene event!  type %d, force %f %f %f",type,x,y,z);
			}
			else 
				Con::errorf("problem with scene event, type %d or actor %d physUser %d is null!",type,actor_id,kPU);

			kPU = NULL;
		}
		mSQL->CloseDatabase();
		delete mSQL;
	}
}

void nxPhysManager::loadSceneEventsFromFile(const char *filename)
{
	float time,duration,x,y,z;
	int botNum,cause,type,node;
	char action[255];	
	char buf[1000];
	char *bufp;
	
	char sceneEvent_id_query[512],scene_id_query[512],mission_id_query[512],insert_query[512];
	int actor_id,body_id,joint_id,sceneEvent_id,result;
	fxFlexBody *kFB;
	sqlite_resultset *resultSet;

	FILE *fp = NULL;
	iPhysUser *kPU = NULL;

	mSQL = new SQLiteObject();
	if (mSQL) 
	{
		if (mSQL->OpenDatabase("EcstasyMotion.db"))
		{

			String eventsFilename(filename);
			if (dStrstr(eventsFilename.c_str(),".mis"))
			{
				String::SizeType begin = eventsFilename.find("/",eventsFilename.length(),String::Right) + 1;
				String::SizeType end = eventsFilename.find(".mis");
				mMissionName = eventsFilename.substr(begin,end - begin);
				Con::printf("Mission name in physManager: %s",mMissionName.c_str());
				sprintf(sceneEvent_id_query,"SELECT id FROM mission WHERE name = '%s';",
					mMissionName.c_str());
				result = mSQL->ExecuteSQL(sceneEvent_id_query);
				resultSet = mSQL->GetResultSet(result);
				if (resultSet->iNumRows==1) 
					mMissionId = dAtoi(resultSet->vRows[0]->vColumnValues[0]);
			}

			String sceneName(mMissionName);
			sceneName += "_default";

			String eventsFile(filename);
			String eventsExt(".events");
			if (!dStrstr(eventsFile.c_str(),".events")) eventsFile += eventsExt;
			fp = fopen(eventsFile.c_str(),"r");

			SimSet *playBotGroup = NULL;
			playBotGroup = dynamic_cast<SimSet *>(Sim::findObject("PlayBotGroup"));
			if (playBotGroup) Con::printf("playbotgroup size: %d",playBotGroup->size());
			else return;

			if (fp) 
			{
				//NOW: with sqlite in place, see if the mission exists, if not add it, and then see if 
				//any scene events for this mission exist.

				while (fgets(buf,255,fp)) 
				{
					//sscanf(buf,"%d;%f;%f;%d;%d;(%f %f %f); %s ;",&type,&time,&duration,&botNum,&node,&x,&y,&z,&action);
					if (dStrlen(buf)<10) return;//Filter out empty events file with maybe some whitespace.
					sprintf(action,"");
					bufp = strtok(buf,";");
					sscanf(bufp,"%d",&type);     bufp = strtok(NULL,";");
					sscanf(bufp,"%f",&time);     bufp = strtok(NULL,";");
					sscanf(bufp,"%f",&duration); bufp = strtok(NULL,";");
					sscanf(bufp,"%d",&botNum);   bufp = strtok(NULL,";");
					sscanf(bufp,"%d",&cause);   bufp = strtok(NULL,";");
					sscanf(bufp,"%d",&node);     bufp = strtok(NULL,";");
					sscanf(bufp,"(%f %f %f)",&x,&y,&z); bufp = strtok(NULL,";");
					sscanf(bufp,"%s",&action);  

					if (botNum>0)
						if (botNum<playBotGroup->size()+1)
							kPU = dynamic_cast<iPhysUser *>(playBotGroup->at(botNum-1));

					kFB = dynamic_cast<fxFlexBody *>(kPU);

					sprintf(scene_id_query,"SELECT id FROM scene WHERE name='%s';",sceneName.c_str());
					result = mSQL->ExecuteSQL(scene_id_query);
					resultSet = mSQL->GetResultSet(result);
					if (resultSet->iNumRows==1) 
						mSceneId = dAtoi(resultSet->vRows[0]->vColumnValues[0]);

					if (mSceneId>0)
					{
						sprintf(sceneEvent_id_query,"SELECT id FROM sceneEvent WHERE actor_id = %d AND scene_id = %d AND type = %d AND node = %d AND time = %f AND action = '%s';",
							kFB->mActorId,mSceneId,type,node,time,action);
						result = mSQL->ExecuteSQL(sceneEvent_id_query);
						resultSet = mSQL->GetResultSet(result);
						if (resultSet->iNumRows == 0)  
						{
							if (dStrlen(action)==0)
								sprintf(insert_query,"INSERT INTO sceneEvent (actor_id,scene_id,type,node,time,duration,value_x,value_y,value_z) VALUES (%d,%d,%d,%d,%f,%f,%f,%f,%f);",
								kFB->mActorId,mSceneId,type,node,time,duration,x,y,z);
							else
								sprintf(insert_query,"INSERT INTO sceneEvent (actor_id,scene_id,type,node,time,duration,value_x,value_y,value_z,action) VALUES (%d,%d,%d,%d,%f,%f,%f,%f,%f,'%s');",
								kFB->mActorId,mSceneId,type,node,time,duration,x,y,z,action);

							//Con::errorf("scene event query: %s",insert_query);
							result = mSQL->ExecuteSQL(insert_query);
						}
					}
				}
			}

			//In case we didn't have a .events file, we need to do the scene id query again here.
			sprintf(scene_id_query,"SELECT id FROM scene WHERE name='%s';",sceneName.c_str());
			result = mSQL->ExecuteSQL(scene_id_query);
			resultSet = mSQL->GetResultSet(result);
			if (resultSet->iNumRows==1) 
				mSceneId = dAtoi(resultSet->vRows[0]->vColumnValues[0]);

			sprintf(sceneEvent_id_query,"SELECT actor_id,type,node,time,duration,value_x,value_y,value_z,action FROM sceneEvent WHERE scene_id = %d;",mSceneId);
			result = mSQL->ExecuteSQL(sceneEvent_id_query);
			resultSet = mSQL->GetResultSet(result);
			for (unsigned int i=0;i<resultSet->iNumRows;i++)
			{
				actor_id = dAtoi(resultSet->vRows[i]->vColumnValues[0]);
				//HERE: need to get a physUser kPU out of actor_id somehow.
				//Could run through botgroup to find one where actor_id == this actor_id.
				for (unsigned int j=0;j<playBotGroup->size();j++)
				{
					if (dynamic_cast<fxFlexBody *>(playBotGroup->at(j))->mActorId == actor_id)
						kPU = dynamic_cast<iPhysUser *>(playBotGroup->at(j));
				}
				type = dAtoi(resultSet->vRows[i]->vColumnValues[1]);
				node = dAtoi(resultSet->vRows[i]->vColumnValues[2]);
				time = dAtoi(resultSet->vRows[i]->vColumnValues[3]);
				duration = dAtoi(resultSet->vRows[i]->vColumnValues[4]);
				x = dAtoi(resultSet->vRows[i]->vColumnValues[5]);
				y = dAtoi(resultSet->vRows[i]->vColumnValues[6]);
				z = dAtoi(resultSet->vRows[i]->vColumnValues[7]);
				sprintf(action,"%s",resultSet->vRows[i]->vColumnValues[8]);
				if ((type)&&(kPU)) addSceneEvent(type,kPU,time,duration,node,Ogre::Vector3(x,y,z),action,cause);
				else Con::errorf("problem with scene event, type or physUser is null!");
			}
			mSQL->CloseDatabase();
			delete mSQL;
		}
	}
}

void nxPhysManager::saveScene()
{  
	//Here: save scene button has been pressed.  Most parts of the scene (scene events, etc) 
	//are saved already because they go straight to the database.  But actor positions are
	//the reason for this function, and it will also be a good place to put any other data  
	//that isn't already stored somewhere else.
	
	char actor_scene_query[512],update_query[512],insert_query[512];
	sqlite_resultset *resultSet;
	int result,result2;

	SimSet *playBotGroup = NULL;
	playBotGroup = dynamic_cast<SimSet *>(Sim::findObject("PlayBotGroup"));
	if (playBotGroup) Con::printf("playbotgroup size: %d",playBotGroup->size());
	else return;

	mSQL = new SQLiteObject();
	if (mSQL) 
	{
		if (mSQL->OpenDatabase("EcstasyMotion.db"))
		{
			for (unsigned int j=0;j<playBotGroup->size();j++)
			{//HERE: another place where a real intelligent query could do the whole set in one
				//line, and one DB call, instead of two queries per actor.  But this is not called
				//very often, leaving it the simple slow way.  Massive crowd scenes could justify
				//fixing it someday.
				fxFlexBody *kFB = dynamic_cast<fxFlexBody *>(playBotGroup->at(j));
				//kFB->mActorId;

				sprintf(actor_scene_query,"SELECT id,start_x,start_y,start_z,start_rot FROM actorScene WHERE actor_id=%d AND scene_id=%d;",
					kFB->mActorId,mSceneId);
				result = mSQL->ExecuteSQL(actor_scene_query);
				resultSet = mSQL->GetResultSet(result);
				Ogre::Vector3 pos = kFB->getPosition();
				Ogre::Matrix4 mat = kFB->getTransform();
				EulerF euler = mat.toEuler();
				float euler_z = mRadToDeg(euler.z);
				if (euler_z == -360.0)
					euler_z = 0.0;

				if (resultSet->iNumRows == 1)//danger, make sure there can never be two or more.
				{
					int actor_scene_id = dAtoi(resultSet->vRows[0]->vColumnValues[0]);
					float x = dAtof(resultSet->vRows[0]->vColumnValues[1]);
					float y = dAtof(resultSet->vRows[0]->vColumnValues[2]);
					float z = dAtof(resultSet->vRows[0]->vColumnValues[3]);
					float rot = dAtof(resultSet->vRows[0]->vColumnValues[4]);
					if ((pos.x!=x)||(pos.y!=y)||(pos.z!=z)||(euler_z!=rot))//deal with rot later
					{					
						sprintf(update_query,"UPDATE actorScene SET start_x=%f,start_y=%f,start_z=%f,start_rot=%f where id=%d;",
							pos.x,pos.y,pos.z,euler_z,actor_scene_id);
						result2 = mSQL->ExecuteSQL(update_query);
					}
				} else if (resultSet->iNumRows == 0) {
						sprintf(update_query,"INSERT INTO actorScene (actor_id,scene_id,start_x,start_y,start_z,start_rot) VALUES (%d,%d,%f,%f,%f,%f);",
							kFB->mActorId,mSceneId,pos.x,pos.y,pos.z,euler_z);
						result2 = mSQL->ExecuteSQL(update_query);
				} else {
					Con::errorf("Warning: there are multiple actorScene rows with actor_id %d and scene_id %d",
						kFB->mActorId,mSceneId);
				}
			}
			mSQL->CloseDatabase();
			delete mSQL;
		}
	}

}

ecstasySceneEvent *nxPhysManager::getEvent(int eventID)
{
	ecstasySceneEvent *kSE = NULL;
	ecstasySceneEvent *pEventIterator = NULL;
	
	for (pEventIterator = mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = mImpulseEventList.next(pEventIterator))
		if (pEventIterator->eventID == eventID)
		{
			kSE = pEventIterator;
		}

	for (pEventIterator = mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = mDurationEventList.next(pEventIterator))
		if (pEventIterator->eventID == eventID)
		{
			kSE = pEventIterator;
		}

	for (pEventIterator = mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = mInterpolationEventList.next(pEventIterator))
		if (pEventIterator->eventID == eventID)
		{
			kSE = pEventIterator;
		}

	for (pEventIterator = mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = mFollowEventList.next(pEventIterator))
		if (pEventIterator->eventID == eventID)
		{
			kSE = pEventIterator;
		}
	return kSE;
}

ConsoleFunction( clearSceneEvents, void, 1, 1, "clearSceneEvents()")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	kPM->clearSceneEvents();
}

ConsoleFunction( saveSceneEvents, void, 2, 2, "saveSceneEvents(const char *filename)")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	kPM->saveSceneEventsToFile(argv[1]);
}

ConsoleFunction( loadSceneEvents, void, 1, 2, "loadSceneEvents(int scene_id || const char *filename)")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();

	if (argc==2)
	{
		if (dStrstr(argv[1],".mis.events"))
			kPM->loadSceneEventsFromFile(argv[1]);//filename, for backwards compatibility
		else
			kPM->loadSceneEvents(dAtoi(argv[1]));//scene id
	}
	else kPM->loadSceneEvents(kPM->mSceneId);

}

ConsoleFunction( saveScene, void, 1, 1, "saveScene()")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	kPM->saveScene();
}


//ARgh... should go through and change ALL of these from console functions to nxPhysManager
//functions called from console functions.

ConsoleFunction( getEventType, int, 2, 2, "getEventType(int eventID)")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	ecstasySceneEvent *pEventIterator = NULL;

	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->eventType;

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->eventType;

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->eventType;

	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->eventType;

	return -1;
}

ConsoleFunction( setEventType, bool, 3, 3, "setEventType(int eventID,int type)")
{
	if (argc<3) return false;

	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	ecstasySceneEvent *pEventIterator = NULL;

	//HERE: be careful here!  Check to see if we're moving to a different list first.
	//For now, solving this on the script side by dropping and adding, as with time changes.
	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->eventType = dAtoi(argv[2]);
			return true;
		}

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->eventType = dAtoi(argv[2]);
			return true;
		}

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->eventType = dAtoi(argv[2]);
			return true;
		}

	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->eventType = dAtoi(argv[2]);
			return true;
		}

	return false;
}

ConsoleFunction( getEventIDDB, int, 2, 2, "getEventIDDB(int dbID)")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	float time = 0.0;

	ecstasySceneEvent *pEventIterator = NULL;

	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
		if (pEventIterator->dbID == dAtoi(argv[1]))
			return pEventIterator->eventID;

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
		if (pEventIterator->dbID == dAtoi(argv[1]))
			return pEventIterator->eventID;

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
		if (pEventIterator->dbID == dAtoi(argv[1]))
			return pEventIterator->eventID;

	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
		if (pEventIterator->dbID == dAtoi(argv[1]))
			return pEventIterator->eventID;

	return -1;
}

ConsoleFunction( getEventDBID, int, 2, 2, "getEventDBID(int eventID)")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	float time = 0.0;

	ecstasySceneEvent *pEventIterator = NULL;

	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->dbID;

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->dbID;

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->dbID;

	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->dbID;

	return -1;
}

ConsoleFunction( getEventCause, int, 2, 2, "getEventCause(int eventID)")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	ecstasySceneEvent *pEventIterator = NULL;

	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->cause->eventID;

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->cause->eventID;

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->cause->eventID;

	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->cause->eventID;

	return -1;
}

ConsoleFunction( setEventCause, bool, 3, 3, "setEventCause(int eventID,int causeID)")
{
	if (argc<3) return false;

	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	ecstasySceneEvent *pEventIterator = NULL;
	int eventID = -1;

	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			//HERE: have to find a scene event pointer by searching the lists till I find this ID.
			pEventIterator->cause = kPM->getEvent(dAtoi(argv[2]));
			return true;
		}

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->cause = kPM->getEvent(dAtoi(argv[2]));
			return true;
		}

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->cause = kPM->getEvent(dAtoi(argv[2]));
			return true;
		}

	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->cause = kPM->getEvent(dAtoi(argv[2]));
			return true;
		}

	return false;
}

ConsoleFunction( getEventPhysUser, int, 2, 2, "getEventPhysUser(int eventID)")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	iPhysUser *kPU = NULL;
	ecstasySceneEvent *pEventIterator = NULL;
		
	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			kPU = pEventIterator->physUser;

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			kPU = pEventIterator->physUser;

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			kPU = pEventIterator->physUser;

	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			kPU = pEventIterator->physUser;

	///HERE: convert this to a ghost ID and return that.
	//else return 0;
	if (kPU)
	{
		//NOW: have to figure this out again, going the other way this time. 
		int ghostID = dynamic_cast<SimObject *>(kPU)->getId();//Maybe??  Yup!
		return ghostID;
	} else return 0;
}

ConsoleFunction( setEventPhysUser, bool, 3, 3, "setEventPhysUser(int eventID,int actorID)")
{
	if (argc<3) return false;

	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	iPhysUser *kPU = NULL;
	ecstasySceneEvent *pEventIterator = NULL;

	int ghostID = dAtoi(argv[2]);//this is ServerConnection.getGhostID($tweaker_bot)
	//Have to resolve that into a physUser pointer.
	NetConnection *toServer =  NetConnection::getConnectionToServer();
	NetConnection *toClient = NetConnection::getLocalClientConnection();
	NetObject *netObj = toClient->resolveObjectFromGhostIndex(ghostID);
	int index = toClient->getGhostIndex(netObj);
	NetObject *clientNetObj = toServer->resolveGhost(index);
	kPU = dynamic_cast<iPhysUser *>(clientNetObj);


	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->physUser = kPU;
			return true;
		}

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->physUser = kPU;
			return true;
		}

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->physUser = kPU;
			return true;
		}

	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->physUser = kPU;
			return true;
		}

	return false;
}

ConsoleFunction( getEventTime, float, 2, 2, "getEventTime(int eventID)")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	float time = 0.0;

	ecstasySceneEvent *pEventIterator = NULL;

	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->time;

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->time;

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->time;

	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->time;

	return 0.0;
}

ConsoleFunction( setEventTime, float, 3, 3, "setEventTime(int eventID,float time)")
{
	if (argc<3) return false;

	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	ecstasySceneEvent *pEventIterator = NULL;

	//HERE: need to have a care when changing the time - may have to move it in the list.
	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->time = dAtof(argv[2]);
			return true;
		}
	}

	//HERE: may have to move it in the list.
	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->time = dAtof(argv[2]);
			return true;
		}
	}

	//HERE: for interpolation events may also have to change prev and next.
	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->time = dAtof(argv[2]);
			return true;
		}
	}

	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->time = dAtof(argv[2]);
			return true;
		}
	}

	return false;
}

ConsoleFunction( getEventDuration, float, 2, 2, "getEventDuration(int eventID)")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	float duration = 0.0;

	ecstasySceneEvent *pEventIterator = NULL;
	
	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->duration;

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->duration;

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->duration;

	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->duration;

	return 0.0;
}

ConsoleFunction( setEventDuration, bool, 3, 3, "setEventDuration(int eventID,float duration)")
{
	if (argc<3) return false;

	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	ecstasySceneEvent *pEventIterator = NULL;
	
	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->duration = dAtof(argv[2]);
			return true;
		}
	}

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->duration = dAtof(argv[2]);
			return true;
		}
	}

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->duration = dAtof(argv[2]);
			return true;
		}
	}

	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->duration = dAtof(argv[2]);
			return true;
		}
	}

	return false;
}

ConsoleFunction( getEventNode, int, 2, 2, "getEventNode(int eventID)")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	int node = -1;
	ecstasySceneEvent *pEventIterator = NULL;

	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->node;

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->node;

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->node;

	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			return pEventIterator->node;

	return -1;
}

ConsoleFunction( setEventNode, bool, 3, 3, "setEventNode(int eventID,int node)")
{
	if (argc<3) return false;

	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	ecstasySceneEvent *pEventIterator = NULL;

	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->node = dAtoi(argv[2]);
			return true;
		}
	}

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->node = dAtoi(argv[2]);
			return true;
		}
	}

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->node = dAtoi(argv[2]);
			return true;
		}
	}

	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->node = dAtoi(argv[2]);
			return true;
		}
	}

	return false;
}

ConsoleFunction( getEventValue, const char *, 2, 2, "getEventValue(int eventID)")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	char* buff = Con::getReturnBuffer(100);
	Ogre::Vector3 value;
	value.zero();
	ecstasySceneEvent *pEventIterator = NULL;
			
	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			value = pEventIterator->value;

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			value = pEventIterator->value;

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			value = pEventIterator->value;

	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			value = pEventIterator->value;


	dSprintf(buff,100,"%g %g %g",value.x,value.y,value.z);
	return buff;
}

ConsoleFunction( setEventValue, bool, 3, 3, "setEventValue(int eventID, Ogre::Vector3 value)")
{
	if (argc<3) return false;
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	Ogre::Vector3 value;
	ecstasySceneEvent *pEventIterator = NULL;

	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			dSscanf(argv[2],"%g %g %g",&value.x,&value.y,&value.z);
			pEventIterator->value = value;
			return true;
		}
	}

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			dSscanf(argv[2],"%g %g %g",&value.x,&value.y,&value.z);
			pEventIterator->value = value;
			return true;
		}
	}

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			dSscanf(argv[2],"%g %g %g",&value.x,&value.y,&value.z);
			pEventIterator->value = value;
			return true;
		}
	}
	
	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			dSscanf(argv[2],"%g %g %g",&value.x,&value.y,&value.z);
			pEventIterator->value = value;
			return true;
		}
	}

	return false;
}

ConsoleFunction( getEventAction, const char *, 2, 2, "getEventAction(int eventID)")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	char* buff = Con::getReturnBuffer(100);
	ecstasySceneEvent *pEventIterator = NULL;

	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			dSprintf(buff,100,"%s",pEventIterator->action.c_str());

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			dSprintf(buff,100,"%s",pEventIterator->action.c_str());

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			dSprintf(buff,100,"%s",pEventIterator->action.c_str());

	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
		if (pEventIterator->eventID == dAtoi(argv[1]))
			dSprintf(buff,100,"%s",pEventIterator->action.c_str());

	return buff;
}

ConsoleFunction( setEventAction, bool, 3, 3, "setEventAction(int eventID, const char *action)")
{
	if (argc<3) return false;

	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	ecstasySceneEvent *pEventIterator = NULL;

	for (pEventIterator = kPM->mImpulseEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mImpulseEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->action.clear();
			pEventIterator->action.insert(0,argv[2]);
			return true;
		}
	}

	for (pEventIterator = kPM->mDurationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mDurationEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->action.clear();
			pEventIterator->action.insert(0,argv[2]);
			return true;
		}
	}

	for (pEventIterator = kPM->mInterpolationEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mInterpolationEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->action.clear();
			pEventIterator->action.insert(0,argv[2]);
			return true;
		}
	}
	
	for (pEventIterator = kPM->mFollowEventList.next(pEventIterator); pEventIterator;
		pEventIterator = kPM->mFollowEventList.next(pEventIterator))
	{
		if (pEventIterator->eventID == dAtoi(argv[1]))
		{
			pEventIterator->action.clear();
			pEventIterator->action.insert(0,argv[2]);
			return true;
		}
	}

	return false;
}


ConsoleFunction( setEcstasyProject, bool, 2, 2, "setEcstasyProject(const char *filename)")
{
	FILE *fpw = fopen(argv[1],"w");
	if (fpw == NULL)
		return false;
	else 
	{
		fprintf(fpw,"///////////////Ecstasy Project File/////////////////\n");
		//fprintf(fpw,"echo(\"Executing my project startup script!!!!!!!!!!!!!\");\n");


		fclose(fpw);
		return true;
	}
}


ConsoleFunction( setSceneRecordLocal, void, 1, 1, "setSceneRecordLocal()")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	kPM->mSceneRecordLocal = true;
}

ConsoleFunction( setSceneRecordGlobal, void, 1, 1, "setSceneRecordGlobal()")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	kPM->mSceneRecordLocal = false;
}

ConsoleFunction( setMissionName, void, 2, 2, "setMissionName(char *name)")
{
	
	int result,scene_id=0;
	char scene_id_query[512];
	sqlite_resultset *resultSet;

	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	kPM->mMissionName = argv[1];
	if ((dStrstr(kPM->mMissionName.c_str(),".mis"))||(dStrstr(kPM->mMissionName.c_str(),".MIS")))
		kPM->mMissionName.erase(kPM->mMissionName.length()-4,4);
	String::SizeType slashPos = kPM->mMissionName.find('/',kPM->mMissionName.length(),String::Right);
	kPM->mMissionName.erase(0,slashPos+1);
	kPM->mSceneName = kPM->mMissionName;
	kPM->mSceneName += "_default";
	
	kPM->mSQL = new SQLiteObject();
	if (kPM->mSQL) 
	{
		if (kPM->mSQL->OpenDatabase("EcstasyMotion.db"))
		{
			scene_id = 0;
			sprintf(scene_id_query,"SELECT id FROM scene WHERE name='%s';",kPM->mSceneName.c_str());
			result = kPM->mSQL->ExecuteSQL(scene_id_query);
			resultSet = kPM->mSQL->GetResultSet(result);
			if (resultSet->iNumRows==1) 
			{
				kPM->mSceneId = dAtoi(resultSet->vRows[0]->vColumnValues[0]);
			}
			kPM->mSQL->CloseDatabase();
			delete kPM->mSQL;
		}
	}
	Con::printf("trimmed mission name: %s, scene name %s, scene id %d",kPM->mMissionName.c_str(),kPM->mSceneName.c_str(),kPM->mSceneId);
}

ConsoleFunction( setSceneName, int, 2, 2, "setSceneName(char *name)")
{	
	int result,scene_id=0;
	char scene_id_query[512];
	sqlite_resultset *resultSet;

	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	kPM->mSceneName = argv[1];

	kPM->mSQL = new SQLiteObject();
	if (kPM->mSQL) 
	{
		if (kPM->mSQL->OpenDatabase("EcstasyMotion.db"))
		{
			scene_id = 0;
			sprintf(scene_id_query,"SELECT id FROM scene WHERE name='%s';",kPM->mSceneName.c_str());
			result = kPM->mSQL->ExecuteSQL(scene_id_query);
			resultSet = kPM->mSQL->GetResultSet(result);
			if (resultSet->iNumRows==1) 
			{
				kPM->mSceneId = dAtoi(resultSet->vRows[0]->vColumnValues[0]);
			}
			kPM->mSQL->CloseDatabase();
			delete kPM->mSQL;
		}
	}
	return kPM->mSceneId;
}

ConsoleFunction( getSceneID, int, 1, 1, "getSceneID()")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();

	return kPM->mSceneId;
}

ConsoleFunction( getSceneName,  const char *, 1, 1, "getSceneName()")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();

	return kPM->mSceneName.c_str();
}

ConsoleFunction( getEventDatabaseID, int, 7, 7,"getEventDatabaseID(int eventType,int physUser,float time,float duration,int node,const char *action)")
{
	nxPhysManager *kPM = (nxPhysManager *)physManagerCommon::getPM();
	iPhysUser *kPU;// = (get this from ghost ID)
	fxFlexBody *kFB;
	int ghostID,cause,node,type,index;
	float time,duration;
	String action;
	char sceneEvent_id_query[512],scene_id_query[512];
	int sceneEvent_id,scene_id,result;
	sqlite_resultset *resultSet;

	type = dAtoi(argv[1]);
	ghostID = dAtoi(argv[2]);

	NetConnection *toServer =  NetConnection::getConnectionToServer();
	NetConnection *toClient = NetConnection::getLocalClientConnection();

	NetObject *netObj = toClient->resolveObjectFromGhostIndex(ghostID);
	index = toClient->getGhostIndex(netObj);
	NetObject *clientNetObj = toServer->resolveGhost(index);
	kPU = dynamic_cast<iPhysUser *>(clientNetObj);

	// Yup, this works.  Hm, wow, didn't know you could do that.  iPhysUser
	// isn't even a netobject, but everybody who inherits from it is, so I guess it's okay.
	kFB = dynamic_cast<fxFlexBody *>(kPU);

	time = dAtof(argv[3]);
	duration = dAtof(argv[4]);
	node = dAtoi(argv[5]);
	action.clear();
	action.insert(0,argv[6]);

	kPM->mSQL = new SQLiteObject();
	if (kPM->mSQL) 
	{
		if (kPM->mSQL->OpenDatabase("EcstasyMotion.db"))
		{

			//WTF? time is coming in here as 1.0, where it shoudl be 1.3 for the action I'm selecting.  Query breaking.
			sprintf(sceneEvent_id_query,"SELECT id FROM sceneEvent WHERE actor_id = %d AND scene_id = %d AND type = %d AND node = %d AND time = %f AND action = '%s';",
				kFB->mActorId,kPM->mSceneId,type,node,time,action.c_str());
			result = kPM->mSQL->ExecuteSQL(sceneEvent_id_query);
			resultSet = kPM->mSQL->GetResultSet(result);
	
			if (resultSet->iNumRows > 0) 
				sceneEvent_id = dAtoi(resultSet->vRows[0]->vColumnValues[0]);
			else 
				sceneEvent_id = 0;	

			kPM->mSQL->CloseDatabase();
			delete kPM->mSQL;
		}
	}

	return sceneEvent_id;
}

ConsoleFunction( getTime, int ,1,1,"getTime()")
{
	return Platform::getTime();
}

ConsoleFunction( clearWaitForReset, void,1,1,"clearWaitForReset()")
{
	waitForReset = false;
}


ConsoleFunction( getEventTypeDescription, const char*, 2, 2,"getEventTypeDescription(int eventType)")
{

	int type = dAtoi(argv[1]);
	//Sigh, smallest memory footprint and shortest path appears to be a switch, here goes...
	switch (type)
	{
			
//IMPULSE EVENTS start at 1000
	case SE_IMP_NULL :
		return "IMPULSE_NULL";
	case SE_IMP_FORCE :
		return "IMPULSE FORCE";
	case	SE_IMP_TORQUE :
		return "IMPULSE_TORQUE";
	case	SE_IMP_MOTOR_TARGET :
		return "IMPULSE_";
	case	SE_IMP_SET_FORCE :
		return "IMPULSE_MOTOR_TARGET";
	case	SE_IMP_SET_TORQUE :
		return "IMPULSE_SET_TORQUE";
	case	SE_IMP_SET_MOTOR_TARGET :
		return "IMPULSE_SET_MOTOR_TARGET";
	case	SE_IMP_SET_GLOBAL_FORCE :
		return "IMPULSE_SET_GLOBAL_FORCE";
	case	SE_IMP_MOVE :
		return "IMPULSE_MOVE";
	case	SE_IMP_TURN :
		return "IMPULSE_TURN";
	case	SE_IMP_RAGDOLL_FORCE :
		return "IMPULSE_RAGDOLL_FORCE";
	case	SE_IMP_RAGDOLL :
		return "IMPULSE_RAGDOLL";
	case	SE_IMP_KINEMATIC :
		return "IMPULSE_KINEMATIC";
	case	SE_IMP_GLOBAL_TORQUE :
		return "IMPULSE_GLOBAL_TORQUE";
	case	SE_IMP_MOTORIZE :
		return "IMPULSE_MOTORIZE";
	case	SE_IMP_CLEAR_MOTOR :
		return "IMPULSE_CLEAR_MOTOR";
	case	SE_IMP_EXPLOSION_CAUSE :
		return "IMPULSE_EXPLOSION_CAUSE";
	case	SE_IMP_EXPLOSION_EFFECT :
		return "IMPULSE_EXPLOSION_EFFECT";
	case	SE_IMP_WEAPON_CAUSE :
		return "IMPULSE_WEAPON_CAUSE";
	case	SE_IMP_WEAPON_EFFECT :
		return "IMPULSE_WEAPON_EFFECT";
	case	SE_IMP_MOVE_TO_POSITION :
		return "IMPULSE_MOVE_TO_POSITION";
	case	SE_IMP_MOVE_TO_TARGET  :
		return "IMPULSE_MOVE_TO_TARGET";
	case	SE_IMP_SCRIPT :
		return "IMPULSE_SCRIPT";

//DURATION EVENTS start at 2000
	case	SE_DUR_NULL :
		return "DUR_NULL";
	case	SE_DUR_FORCE :
		return "DUR_FORCE";
	case	SE_DUR_TORQUE :
		return "DUR_TORQUE";
	case	SE_DUR_MOTOR_TARGET  :
		return "DUR_MOTOR_TARGET";
	case	SE_DUR_PLAY_SEQ :
		return "DUR_PLAY_SEQ";
	case	SE_DUR_ACTION_SEQ  :
		return "DUR_ACTION_SEQ";
	case	SE_DUR_ACTION :
		return "DUR_ACTION";
	case	SE_DUR_GLOBAL_FORCE :
		return "DUR_GLOBAL_FORCE";
	case	SE_DUR_GLOBAL_TORQUE  :
		return "DUR_GLOBAL_TORQUE";
	case	SE_DUR_RAGDOLL :
		return "DUR_RAGDOLL";
	case	SE_DUR_KINEMATIC  :
		return "DUR_KINEMATIC";
	case	SE_DUR_MOTORIZE  :
		return "DUR_MOTORIZE";
	case	SE_DUR_SCRIPT  :
		return "DUR_SCRIPT";

//INTERPOLATION EVENTS start at 3000
	case	SE_INTERP_NULL  :
		return "INTERP_NULL";
	case	SE_INTERP_FORCE  :
		return "INTERP_FORCE";
	case	SE_INTERP_TORQUE :
		return "INTERP_TORQUE";
	case	SE_INTERP_MOTOR_TARGET :
		return "INTERP_MOTOR_TARGET";
	case	SE_INTERP_MOVE  :
		return "INTERP_MOVE";
	case	SE_INTERP_TURN  :
		return "INTERP_TURN";
	case	SE_INTERP_GLOBAL_FORCE  :
		return "INTERP_GLOBAL_FORCE";
	case	SE_INTERP_GLOBAL_TORQUE :
		return "INTERP_GLOBAL_TORQUE";
	case	SE_INTERP_SCRIPT   :
		return "INTERP_SCRIPT";


//FOLLOW EVENTS start at 4000
	case	SE_FOLLOW_NULL  :
		return "FOLLOW_NULL";
	case	SE_FOLLOW_MOVE_TO_POSITION :
		return "FOLLOW_MOVE_TO_POS";
	case	SE_FOLLOW_SCRIPT  :
		return "FOLLOW_SCRIPT";
//////// END ECSTASY SCENE EVENT TYPES ///////////
	}
	return "(undefined type)";
}


// Create body parts
head = kPM->CreateSphere(NxVec3(0,0.88,0), 0.05, 10);
torso = kPM->CreateSphere(NxVec3(0,0.7,0), 0.095, 10);
pelvis = kPM->CreateSphere(NxVec3(0,0.58,0), 0.07, 10);

leftUpperArm = kPM->CreateCapsule(NxVec3(0.05,0.85,0), 0.1, 0.04, 10);
leftUpperArm->setGlobalOrientationQuat(qRotRight);
leftForeArm = kPM->CreateCapsule(NxVec3(0.2,0.85,0), 0.1, 0.03, 10);
leftForeArm->setGlobalOrientationQuat(qRotRight);
leftHand = kPM->CreateBox(NxVec3(0.35,0.85,0), NxVec3(0.03,0.03,0.01), 10);
leftHand->setGlobalOrientationQuat(qRotRight);

rightUpperArm = kPM->CreateCapsule(NxVec3(-0.05,0.85,0), 0.1, 0.04, 10);
rightUpperArm->setGlobalOrientationQuat(qRotLeft);
rightForeArm = kPM->CreateCapsule(NxVec3(-0.2,0.85,0), 0.1, 0.03, 10);
rightForeArm->setGlobalOrientationQuat(qRotLeft);
rightHand = kPM->CreateBox(NxVec3(-0.35,0.85,0), NxVec3(0.03,0.03,0.01), 10);
rightHand->setGlobalOrientationQuat(qRotLeft);

leftThigh = kPM->CreateCapsule(NxVec3(0.06,0.6,0), 0.15, 0.05, 10);
leftThigh->setGlobalOrientationQuat(qRotAround);
leftCalf = kPM->CreateCapsule(NxVec3(0.06,0.35,0), 0.15, 0.035, 10);
leftCalf->setGlobalOrientationQuat(qRotAround);
leftFoot = kPM->CreateBox(NxVec3(0.06,0.15,0.02), NxVec3(0.04,0.02,0.075), 10);
leftFoot->setGlobalOrientationQuat(qRotAround);

rightThigh = kPM->CreateCapsule(NxVec3(-0.06,0.6,0), 0.15, 0.05, 10);
rightThigh->setGlobalOrientationQuat(qRotAround);
rightCalf = kPM->CreateCapsule(NxVec3(-0.06,0.35,0), 0.15, 0.035, 10);
rightCalf->setGlobalOrientationQuat(qRotAround);
rightFoot = kPM->CreateBox(NxVec3(-0.06,0.15,0.02), NxVec3(0.04,0.02,0.075), 10);
rightFoot->setGlobalOrientationQuat(qRotAround);

// Joint body parts together
neck = kPM->CreateFixedJoint(head,torso,NxVec3(0,0.88,0),NxVec3(0,1,0));
leftShoulder = kPM->CreateFixedJoint(leftUpperArm,torso,NxVec3(0.05,0.85,0),NxVec3(1,0,0));
rightShoulder = kPM->CreateFixedJoint(rightUpperArm,torso,NxVec3(-0.05,0.85,0),NxVec3(-1,0,0));
spine = kPM->CreateFixedJoint(torso,pelvis,NxVec3(0,0.7,0),NxVec3(0,-1,0));
leftHip = kPM->CreateFixedJoint(leftThigh,pelvis,NxVec3(0.06,0.6,0),NxVec3(0,-1,0));
rightHip = kPM->CreateFixedJoint(rightThigh,pelvis,NxVec3(-0.06,0.6,0),NxVec3(0,-1,0));
leftElbow = kPM->CreateFixedJoint(leftForeArm,leftUpperArm,NxVec3(0.2,0.85,0),NxVec3(0,0,-1));
rightElbow = kPM->CreateFixedJoint(rightForeArm,rightUpperArm,NxVec3(-0.2,0.85,0),NxVec3(0,0,-1));
leftWrist = kPM->CreateFixedJoint(leftHand,leftForeArm,NxVec3(0.35,0.85,0),NxVec3(0,0,-1));
rightWrist = kPM->CreateFixedJoint(rightHand,rightForeArm,NxVec3(-0.35,0.85,0),NxVec3(0,0,-1));
leftKnee = kPM->CreateFixedJoint(leftCalf,leftThigh,NxVec3(0.06,0.35,0),NxVec3(1,0,0));
rightKnee = kPM->CreateFixedJoint(rightCalf,rightThigh,NxVec3(-0.06,0.35,0),NxVec3(-1,0,0));
leftAnkle = kPM->CreateFixedJoint(leftFoot,leftCalf,NxVec3(0.06,0.13,0),NxVec3(1,0,0));
rightAnkle = kPM->CreateFixedJoint(rightFoot,rightCalf,NxVec3(-0.06,0.13,0),NxVec3(-1,0,0));

//neck = kPM->CreateBodySphericalJoint(head,torso,NxVec3(0,8.8,0),NxVec3(0,1,0));
//leftShoulder = kPM->CreateBodySphericalJoint(leftUpperArm,torso,NxVec3(0.5,8.5,0),NxVec3(1,0,0));
//rightShoulder = kPM->CreateBodySphericalJoint(rightUpperArm,torso,NxVec3(-0.5,8.5,0),NxVec3(-1,0,0));
//spine = kPM->CreateBodySphericalJoint(torso,pelvis,NxVec3(0,7,0),NxVec3(0,-1,0));
//leftHip = kPM->CreateBodySphericalJoint(leftThigh,pelvis,NxVec3(0.6,6,0),NxVec3(0,-1,0));
//rightHip = kPM->CreateBodySphericalJoint(rightThigh,pelvis,NxVec3(-0.6,6,0),NxVec3(0,-1,0));
//leftElbow = kPM->CreateRevoluteJoint(leftForeArm,leftUpperArm,NxVec3(2,8.5,0),NxVec3(0,0,-1));
//rightElbow = kPM->CreateRevoluteJoint(rightForeArm,rightUpperArm,NxVec3(-2,8.5,0),NxVec3(0,0,-1));
//leftWrist = kPM->CreateRevoluteJoint2(leftHand,leftForeArm,NxVec3(0,-0.15,0),NxVec3(0,1.3,0),NxVec3(0,1,0),NxVec3(0,1,0));
//rightWrist = kPM->CreateRevoluteJoint2(rightHand,rightForeArm,NxVec3(0,-0.15,0),NxVec3(0,1.3,0),NxVec3(0,1,0),NxVec3(0,1,0));
//leftKnee = kPM->CreateRevoluteJoint(leftCalf,leftThigh,NxVec3(0.6,3.5,0),NxVec3(1,0,0));
//rightKnee = kPM->CreateRevoluteJoint(rightCalf,rightThigh,NxVec3(-0.6,3.5,0),NxVec3(-1,0,0));
//leftAnkle = kPM->CreateRevoluteJoint(leftFoot,leftCalf,NxVec3(0.6,1.3,0),NxVec3(1,0,0));
//rightAnkle = kPM->CreateRevoluteJoint(rightFoot,rightCalf,NxVec3(-0.6,1.3,0),NxVec3(-1,0,0));
*/
