////////////////////////////////////////////////////////////////////////////////////////////////////
//  fxFlexBody.h
//  Chris Calef
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _FXFLEXBODY_H_
#define _FXFLEXBODY_H_


#include "EcstasyMotion/physManager.h"

//#include "platform/platform.h"
//#include "T3D/shapeBase.h"

#include "EcstasyMotion/gaAction.h"
#include "EcstasyMotion/gaObservation.h"

//class SimXMLDocument;
struct ecstasySceneEvent;

#define MAX_FLEX_PARTS 100 //How many possible physics bodyparts
#define MAX_FLEX_NODES 200 //How many possible nodes in the dts
#define MAX_FLEX_CHAINS 8 //How many possible limbs
#define MAX_MESH_EXCLUDES 40 //How many meshes can we skip
//#define MAX_TRIGGER_PARTS 20

#include "NxPhysics.h"//FIX: need physTrigger class instead
#include "NxCooking.h"
#include "Nxp.h"


//#include "EcstasyMotion/fbx/myCommon.h"


class fxFlexBodyPart;
class fxRigidBody;
class fxJoint;
class gaActionUser;
struct gaActionUserData;
class NxActor;

//static EnumTable::Enums gChainEnums[] =
//  {
//    {PHYS_CHAIN_SPINE, "CHAIN_SPINE"},
//    {PHYS_CHAIN_RIGHT_ARM, "CHAIN_RIGHT_ARM"},
//    {PHYS_CHAIN_LEFT_ARM, "CHAIN_LEFT_ARM"},
//    {PHYS_CHAIN_RIGHT_LEG, "CHAIN_RIGHT_LEG"},
//    {PHYS_CHAIN_LEFT_LEG, "CHAIN_LEFT_LEG"},
//    {PHYS_CHAIN_RIGHT_WING, "CHAIN_RIGHT_WING"},
//    {PHYS_CHAIN_LEFT_WING, "CHAIN_LEFT_WING"},
//    {PHYS_CHAIN_TAIL, "CHAIN_TAIL"}
//  };
//static EnumTable gChainTypeTable(MAX_FLEX_CHAINS, &gChainEnums[0]);
//DefineEnumType( physChainType );


/////////////////////////////////////////////////////
//
//struct fxFlexBodyData : public ShapeBaseData
//{
//  typedef ShapeBaseData Parent;
//
//  public:
//  
//  DECLARE_CONOBJECT(fxFlexBodyData);
//
//  SimObject               *mActionUserData;
//  int                     mActionUserDataID;
//
//  //database keys
//  int mDBId;//Do we need this here?
//  int mSkeletonID;
//  Ogre::String mSkeletonName;
//
//  unsigned int mLifetimeMS;
//  Ogre::String mMeshObject;
//  unsigned int mNumBodyParts;
//
//  bool mSDK;
//  bool mHW;
//  bool mGA;
//
//  float mDynamicFriction;
//  float mStaticFriction;
//  float mRestitution;
//  float mDensity;
//  float mSleepThreshold;
//  
//  physShapeType mTriggerShapeType;
//  Ogre::Vector3 mTriggerOffset;
//  Ogre::Vector3 mTriggerOrientation;
//  Ogre::Vector3 mTriggerDimensions;
//
//
//  Ogre::String mHeadNodeName;
//  Ogre::String mNeckNodeName;
//  Ogre::String mBodyNodeName;
//  Ogre::String mRightFrontNodeName;
//  Ogre::String mLeftFrontNodeName;
//  Ogre::String mRightBackNodeName;
//  Ogre::String mLeftBackNodeName;
//  Ogre::String mMeshExcludes;
//
//  unsigned int mRelaxType;
//
//  fxFlexBodyData();
//  ~fxFlexBodyData();
//  
//  bool preload(bool bServer, Ogre::String &errorStr);
//  bool onAdd();
//  static void initPersistFields();
//  virtual void packData( BitStream* pStream);
//  virtual void unpackData( BitStream* pStream);
//};

//DECLARE_CONSOLETYPE(fxFlexBodyData)

//
//struct physGroundSequenceData : public GameBaseData
//{
//   typedef GameBaseData Parent;
//
//public:
//   fxFlexBodyData       *mFlexBodyData;
//   int                     FlexBodyDataID;
//
//   int mSeqNum;
//   Ogre::String mSeqName;
//   int mGroundNode1;
//   float mTime1;
//   int mGroundNode2;
//   float mTime2;
//   int mGroundNode3;
//   float mTime3;
//   int mGroundNode4;
//   float mTime4;
//   int mGroundNode5;
//   float mTime5;
//   int mGroundNode6;
//   float mTime6;
//   int mGroundNode7;
//   float mTime7;
//   int mGroundNode8;//?
//   float mTime8;//?
//
//   int mNodes[8];
//   float mTimes[8];
//
//   int mNumSteps;
//   float mTimeScale;//not implemented yet
//
//   Ogre::Vector3 mDeltaVector;//temp, till I work out something better
//
//   physGroundSequenceData();
//
//   bool preload(bool bServer, Ogre::String &errorStr);
//   bool  onAdd();
//   static	void initPersistFields();
//   virtual void packData(BitStream*);
//   virtual void unpackData(BitStream*);
//
//   DECLARE_CONOBJECT(physGroundSequenceData);
//};

//DECLARE_CONSOLETYPE(physGroundSequenceData)

//struct triggerPart 
//{
//	int ID;//index into mBodyParts[]
//	int triggerShape;//0=box,1=capsule,2=sphere -- PHYS_SHAPE_BOX,PHYS_SHAPE_CAPSULE,PHYS_SHAPE_SPHERE
//	Ogre::Vector3 offset;//relative to bodypart origin
//	Ogre::Vector3 dimensions;// X Y Z = dimensions for box, X = diam for sphere, X = diam & Y = length for capsule.
//};

//from ecstasy shapebase
 struct bvhJoint
{
	char name[40];
	Ogre::Vector3 offset;
	unsigned int channels;
	unsigned int chanrots[3];//keep track of rotation order
	int parent;
};

struct nodeRot
{
	int node;
	Ogre::Vector3 rot;
};

#define MAX_BVH_NODES 400
#define MAX_ULTRAFRAME_NODES 200
#define MAX_BVH_NODE_GROUPS 20

#define ULTRAFRAME_TYPES 4
//enum ultraframeType
//{
#define	ADJUST_NODE_POS    0
#define	SET_NODE_POS       1
#define	ADJUST_NODE_ROT    2
#define	SET_NODE_ROT       3

//#define	IMPULSE_FORCE      4
//#define	CONSTANT_FORCE     5
//#define	LOCAL_TORQUE       6
//#define	GLOBAL_TORQUE      7
//#define	MOTOR_TARGET       8
//#define	SPRING_TARGET      9
//#define	IMPULSE_WEAPON_FORCE 10
//#define	BODYPART_ACTION    11

//};

struct ultraframe
{
	ultraframe *next;
	float time;
	int frame;
	int node;
	int type;
	int target;
	Ogre::Vector3 value;
};

struct ultraframeSet
{ 
	int seq;
	Ogre::String seqName;
	std::vector<ultraframe> frames;
	ultraframe *lists[ULTRAFRAME_TYPES][MAX_ULTRAFRAME_NODES];
	bool nodes[MAX_ULTRAFRAME_NODES];
	bool types[ULTRAFRAME_TYPES];
};

//rc = loadBvhCfg(fpc,bvhNodes,dtsNodes,sortNodes,orderNodes,bvhPoseRotsA,bvhPoseRotsB,
//	axesFixRotsA,axesFixRotsB,bvhNames);
struct bvhCfgData
{
	int bvhNodes[MAX_BVH_NODES];//this is index into bvh nodes, since we might skip some.
	int dtsNodes[MAX_BVH_NODES];//this is the index into the dts chain
	int nodeGroups[MAX_BVH_NODES];//these are the group id, 0-4 for (spine neck head-RL arms-RL legs)
	//int sortNodes[MAX_BVH_NODES];//these should be okay to instantiate temporarily, not keep around.
	int orderNodes[MAX_BVH_NODES];
	Ogre::Vector3 bvhPoseRotsA[MAX_BVH_NODES];
	Ogre::Vector3 bvhPoseRotsB[MAX_BVH_NODES];
	Ogre::Vector3 axesFixRotsA[MAX_BVH_NODES];
	Ogre::Vector3 axesFixRotsB[MAX_BVH_NODES];

	bvhJoint joints[MAX_BVH_NODES];

	Ogre::Vector3 endSiteOffsets[MAX_BVH_NODE_GROUPS];

	bool usingNames;
	Ogre::String bvhNames[MAX_BVH_NODES];

	unsigned int numBvhNodes;//(rc)
	unsigned int numDtsNodes;
	unsigned int numNodeGroups;
	float bvhScale;
};
//end from shapebase

struct playlistSeq
{
	int seq;
	int repeats;
	float speed;
	Ogre::String seqFile;
	//F32 duration;
	//bool forward;
};

class fxFlexBody:  public virtual iPhysUser
{ //public ShapeBase,
   //typedef ShapeBase Parent;

public:
	//DECLARE_CONOBJECT(fxFlexBody);

	//fxFlexBodyData* mDataBlock;

	//enum MaskBits
	//{
	//	fxFlexBodyStateMask = Parent::NextFreeMask,
	//	MoveMask     = Parent::NextFreeMask << 1,//Temp?  This has to go back into player if you turn off
	//	fxFlexBodyMountMask	= Parent::NextFreeMask << 2,// player physics and go back to inheriting shapebase. 
	//	NextFreeMask	    = Parent::NextFreeMask << 3// 
	//};

	physManager* mPM;
	gaActionUser *mActionUser;

	Ogre::SceneNode *mNode;
	Ogre::Entity *mEntity;

	int mFlexBodyDataID;
	Ogre::String mFlexBodyDataName;
	int mSkeletonID;
	Ogre::String mSkeletonName;
	int mPersonaID;
	Ogre::String mPersonaName;
	int mPlaylistID;
	Ogre::String mPlaylistName;
	int mActorID;
	Ogre::String mActorName;

	ecstasySceneEvent *mFollowEvent;
	ecstasySceneEvent *mFirstFollowEvent;//this is last follow event ADDED, unless it turns out we also need last follow event executed.
	bool mFollowEventDone;
	bool mLoadedKeyframeSets;

	unsigned int mNumBodyParts;
	unsigned int mFirstNode;
	int mOrderNodes[MAX_FLEX_NODES];

	bool mIsGroundAnimating;
	//bool mIsMoveTargeting;//in shapebase for now

	int mGroundStep;
	Ogre::Vector3 mGroundVector;
	Ogre::Vector3 mDeltaVector;
	int mGroundNode; //not sure which of these is going to 
	//fxFlexBodyPart *mGroundPart;//be more useful.
	//physGroundSequenceData *mGroundSequenceData;
	float mCurrThrTime;
	float mLastThrTime;

	// weapon mount section -- experimental!
	fxRigidBody *mWeapon,*mWeapon2;// this stuff should all be arrays, to allow
	fxJoint *mWeaponJoint,*mWeapon2Joint;// links to multiple rigid bodies & flexbodies.
	fxFlexBodyPart *mWeaponBodypart,*mWeapon2Bodypart;
	int  mWeaponID,mWeapon2ID;
	Ogre::String mWeaponNodeName,mWeapon2NodeName;//TO DO -- change to Ogre::String
	int mWeaponMountNode,mWeapon2MountNode;
	Ogre::Vector3 mWeaponPos,mWeapon2Pos;
	Ogre::Vector3 mWeaponPosAdj,mWeapon2PosAdj;//per-weapon-model adjustments
	Ogre::Quaternion mWeaponRot,mWeapon2Rot;
	Ogre::Quaternion mWeaponRotAdjA,mWeaponRotAdjB,mWeapon2RotAdjA,mWeapon2RotAdjB;//per-weapon-model adjustments
	Ogre::Quaternion mWeaponTriggerRotAdjA,mWeaponTriggerRotAdjB;

	int mHeadNode;
	int mNeckNode;
	int mBodyNode;
	int mRightFrontNode;
	int mLeftFrontNode;
	int mRightBackNode;
	int mLeftBackNode;
	int mTailNode;

	fxFlexBodyPart *mBodyParts[MAX_FLEX_PARTS];
	fxFlexBodyPart *mChainParts[MAX_FLEX_CHAINS];
	Ogre::String mBaseNodeNames[MAX_FLEX_PARTS];
	int mIndexBones[MAX_FLEX_PARTS];
	Ogre::Quaternion mDefaults[MAX_FLEX_NODES];
	// Ogre::Matrix4 mResets[MAX_FLEX_NODES];
	Ogre::Vector3 mResetPoint;
	int mActorGroup;//collision group
	NxActor *mTriggerActor;//FIX! Need a physTrigger class, to be consistent.
	//triggerPart mTriggerParts[MAX_TRIGGER_PARTS];//NOPE!
	//int mNumTriggerParts;

	unsigned int mNumMeshExcludes;
	unsigned int mMeshExcludes[MAX_MESH_EXCLUDES];

	int mMeshBody;
	int mStartMesh;//for storing convex mesh pointers
	int mNumMeshes;// gMeshBodies[], gMeshes[]
	//mNumMeshes is currently redundant, because it equals mNumBodyParts
	//that will change when bodyparts can have more than one convex mesh
	//(ex: elephant's tusks, should be two extra convexes attached to head)

	Ogre::String mShapeName;//Need to change all above from Ogre::String
	Ogre::String mKeyframesFile,mPlaylistFile;//to Ogre::String or Filename, now that they exist.
	Ogre::String mSpawnScript;
	int mPlaylistDelay;
	int mSequenceStartStep;
	int mSequenceEndStep;
	int mCurrSeq;
	int mBotNumber;

	Ogre::Vector3  mCurrPosition;
	Ogre::Vector3  mCurrVelocity;

	Ogre::Vector3 mInitialPosition;
	Ogre::Vector3 mInitialLinearVelocity;
	Ogre::Vector3 mInitialAngularVelocity;
	Ogre::Quaternion mInitialOrientation;

	Ogre::Vector3 mRecordInitialPosition;//?
	Ogre::Quaternion mRecordInitialOrientation;//?

	Ogre::Vector3 mScale;

	//std::vector<DTS::Quat16>   nodeRotations;
	std::vector<Ogre::Vector3>  nodeTranslations;
	//std::vector<DTS::Quat16>   groundRotations;
	std::vector<Ogre::Vector3>  groundTranslations;
	//std::vector<Trigger>  triggers;

	unsigned int mCurrTick;
	unsigned int mCurrMS;
	unsigned int mLifetimeMS;
	unsigned int mTriggerTimeMS;
	unsigned int mRagdollStep;

	bool mIsClientOnly;
	bool mIsAnimating;
	bool mStopAnimating;
	bool mClearIsAnimating;//argh, see notes 02-01-06.log
	bool mClearBodyAnimating;//AAAARgh, see notes 07-28-09.log
	bool mIsKinematic;
	bool mIsNoGravity;
	bool mIsThinking;
	bool mIsPlayer;
	bool mIsStriking;
	bool mIsCorpse;
	bool mIsPhysActive;
	bool mHasCollisionWaiting;
	bool mHasMoved;
	bool mReset;
	bool mIsRecording;
	bool mIsRendering;
	bool mIsReturnToZero;
	bool mIsStreaming;
	bool mArenaStreamWeap;
	bool mArenaStreamWeap2;
	//int mIdleAnim;
	//int mGetUpAnim;
	//int mRunAnim;
	//int mWalkAnim;
	//int mCrawlAnim;
	//int mAttackAnimRight;
	//int mAttackAnimLeft;
	//int mFlungAnim;

	unsigned int mRecordSampleRate;
	unsigned int mRecordCount;
	unsigned int mCollisionTime;

	//////from FlexBodyData//////
	unsigned int mRelaxType;
	
	int mHeadIndex;
	int mNeckIndex;
	int mBodyIndex;
	int mRightFrontIndex;//for biped, arms
	int mLeftFrontIndex;//for quadruped, front legs
	int mRightBackIndex;//biped legs, quad back legs


	int mLeftBackIndex;
	//TSThread* mHeadVThread;
	//TSThread* mHeadHThread;
	//ShadowCaster mShadowCaster;

	fxFlexBody();
	fxFlexBody(Ogre::Entity *entity,const char *meshFile);
	~fxFlexBody();

	// ShapeBase functionality
	//virtual bool onAdd();
	//virtual void onRemove();
	//virtual bool onNewDataBlock(GameBaseData* dptr, bool reload);

	//static void initPersistFields();

	//virtual void processTick(const Move* pMove);
	//virtual void advanceTime(float fTimeDelta);

	virtual void onWorldStep();
	virtual void onTrigger();
	virtual void onTrigger(iPhysUser *,int action=0,int id=0);
	virtual void onCollision(iPhysUser*,int action=0);
	virtual void addForceAtPos(const Ogre::Vector3 &force,const Ogre::Vector3 &pos);
	virtual void lockTractorBeam(int);

	//virtual void setTransform(const Ogre::Matrix4& kTransformMatrix);

	//void renderDebug();//virtual void renderObject(SceneState* pState, RenderInst *ri);
	//bool prepRenderImage(SceneState* state, const unsigned int stateKey, const unsigned int startZone, const bool modifyBaseZoneState);
	//void setupDebugRender(){};

	//virtual unsigned int packUpdate(NetConnection* pConnection, unsigned int uiMask, BitStream* pBitStream);
	//virtual void unpackUpdate(NetConnection* pConnection, BitStream *pBitStream);

	void getMesh();
	void getBaseRot();
	void getChildNodes();
	void getNamedNodes();
	void setupGA();
	void startAnimating(int seq);
	void startAnimatingAtPos(int seq,float pos);
	void stopAnimating();
	//void saveResets();
	void updateParts();
	void updateForces();
	void updateTrigger();
	//bool getSDK() { return mDataBlock->mSDK; }

	void addWeapon(int,int);
	void addWeapon2(int,int);
	void addWeapon(int,const char *);
	void addWeapon2(int,const char *);
	//void mountWeapon();
	//void mountWeapon2();
	fxFlexBodyPart *getBodyPart(int);
	fxFlexBodyPart *getBodyPart(const char *);
	int getBodyPartID(const char *);

	void zeroForces();

	virtual void setBodypartForce(int, Ogre::Vector3 &);
	virtual void setBodypartGlobalForce(int, Ogre::Vector3 &);
	virtual void setBodypartTorque(int, Ogre::Vector3 &);
	virtual void setBodypartGlobalTorque(int, Ogre::Vector3 &);
	virtual void setBodypartMotorTarget(int, Ogre::Vector3 &);
	virtual void setBodypartMotorSpring(int, Ogre::Vector3 &,float);

	virtual void setWeaponMotorTarget(Ogre::Vector3 &);//TO DO: when weapons change to arrays of attached rigid bodies, add int item
	virtual void setWeaponMotorTarget(Ogre::Quaternion &);
	virtual void setWeapon2MotorTarget(Ogre::Vector3 &);
	virtual void setWeapon2MotorTarget(Ogre::Quaternion &);
	virtual void setWeaponTriggerMotorTarget(Ogre::Vector3 &);
	virtual void setWeapon2TriggerMotorTarget(Ogre::Vector3 &);
	virtual void setWeaponTriggerRotAdjA(Ogre::Vector3 &);
	virtual void setWeaponTriggerRotAdjB(Ogre::Vector3 &);


	virtual void headUp();
	virtual void headClear();
	virtual int headCheck();
	void splay();
	void setBodypart(int);
	void setKinematic();
	//void setKinematic(bool);
	void clearBodypart(int);
	void clearKinematic();
	void setNoGravity();
	void clearNoGravity();
	void serverRemoveClientBodyparts();
	void clientRemoveBodyparts();
	void setPhysActive(bool);
	void doSomeDamage(float);
	void setup();
	void releaseActors();
	void setBodypartDelayForces(Ogre::Vector3);
	void setBodypartDelayForce(int, Ogre::Vector3 );
	void setBodypartDelayTorque(int, Ogre::Vector3 );
	void resetPosition();
	void resetSequence(int);
	//void setBodypartsPosition();
	void saveBest();
	void saveAll();

	//void snakeForward(float);
	//void snakeStop();

	void giveDetails();
	void recordTick();

	//bool castRay(const Ogre::Vector3 &start, const Ogre::Vector3 &end, RayInfo* info);

	void setIsRecording(bool);
	void setIsRendering(bool);
	void setIsReturnToZero(bool);

	void setInitialPosition(Ogre::Vector3 &pos);
	void setInitialOrientation(Ogre::Vector3 &rot);
	void makeSequence(const char *seqName);
	void showNodeTransform(int index);

	void setBodypartArena(int index, Ogre::Vector3 &pos, Ogre::Quaternion &rot);
	void startArenaFrame();

	//void exportFBX(int seq);
	int getCurrSeqNum();
	void updateNodes();//This is for updating nodes that are not affiliated
	//with flexbodyparts, esp. for Arena streaming.
	void dropWeapon();
	const char *getFlexBodyName();
	const char *getPersonaName();

	void motorize();
	void resetParts();
	
	bool loadAction(const char *name);
	bool loadAction(const char *name, float pos);

	//overriding a Paul Dana functions, to set my EndSequenceStep if I'm a flexbody.
	//bool playThread(unsigned int slot);
	bool playThread(unsigned int slot, const char *name);
	bool playSeq(unsigned int slot, int seq);
	void runPlaylist();

	////////Ecstasy Motion from ShapeBase //////////////////////
    fxFlexBody *mTarget;
	//bvhCfgData *mCfgData;

	//bool setThreadPos(unsigned int slot,float pos);
	//float getThreadPos(unsigned int slot);
	int getKeyFrame(unsigned int slot);

	//bvhCfgData = (int bvhNodes[],dtsNodes[],sortNodes[],orderNodes[],Ogre::Vector3 bvhPoseRotsA[],B[],axesFixRotsA[],B[]) 
	unsigned int loadBvhCfg(FILE *,bvhCfgData*,unsigned int profile_id);
	unsigned int loadBvhCfgDB(bvhCfgData*,unsigned int profile_id);
	void fixBvhCfg(const char *cfg_filename,const char *bvh_filename);
	void saveBvhCfg(FILE *,bvhCfgData*);
	unsigned int loadBvhCfgXml(const char *xml_filename,bvhCfgData*);
	void saveBvhCfgXml(const char *xml_filename,bvhCfgData*);
	void saveBvhCfgSql(bvhCfgData *);
	unsigned int loadBvhSkeleton(FILE *,bvhCfgData*,unsigned int profile_id);
	unsigned int loadBvhSkeletonDB(bvhCfgData*,unsigned int profile_id);

	void nullBvh(const char *bvhFile);
	void nullBvh(const char *bvhFile, const char *bvhDir);
	void cleanupBvh(const char *bvhFile);
	void cleanupDir(const char *bvhDir);
	void importBvh(bool,const char *bvhFile,const char *bvhProfile);
	void importDir(bool,const char *bvhDir,const char *dsqDir,const char *bvhProfile);
	void loadDsq(const char *dsqFile);
	void saveBvh(unsigned int seqNum, const char *bvhFile);
	void saveBvh(unsigned int seqNum, const char *bvhFile, const char *bvhFormat);
	void threadInfo(int index);
	void setSeqCyclic(int seq,bool cyclic);
	void groundCaptureDir(const char *inDir, const char *outDir);
	void saveBvhDir(const char *inDir, const char *format);

	int mAnimationFreeze;

	float mShapeSize;
	Ogre::Vector3 mBaseNodeSetPos;
	Ogre::Vector3 mBaseNodeAdjustPos;
	Ogre::Vector3 mOrigin;

	bool mBvhCfgUsingNames;
	bool mBeenTweakerBot;

	
	bool mGoFullRagdoll;
	bool mBeenHit;
	bool mNoLegRagdoll;
	int mBeenHitTick;
	float mPhysicsDamage;//getDamageLevel etc. too hard to work with

	//int mPlaylistRepeats;//Moved back to shapebase.
	//int mCurrentPlaylistSeq;
	//int mPlaylistTick;
	int mLastFrameChecked;
	int mImportSampleRate;

	std::vector<nodeRot> mNodeSetRots;
	std::vector<nodeRot> mNodeAdjustRots;
	std::vector<ultraframeSet> mUltraframeSets;
	//std::vector<playlistSeq> mPlaylist;//Moved back to shapebase.

	//std::vector<Quat16>  backupRotations;//NAY! do NOT keep this as a shape INSTANCE thing, this is a SHAPE thing.
	//std::vector<Ogre::Vector3> backupTranslations;//Also, storing it by sequence would be better than storing them all in one vector.
	//Instead we're going to use TSShape::sequenceBackups vector, once per shape, one set per morphed sequence.

	//backupGroundTranslations? Rotations?
	//Ogre::Vector3 mGroundStartPos;
	// Ogre::Matrix4 mGroundStartXform;//moved back up to shapebase

	void setBaseNodePosRegion(unsigned int seq,Ogre::Vector3 &pos,float start,float stop);
	void adjustBaseNodePosRegion(unsigned int seq,Ogre::Vector3 &pos,float start,float stop);
	void setNodeRotRegion(unsigned int seq, unsigned int node, Ogre::Vector3 &rot,float start,float stop);
	void adjustNodeRotRegion(unsigned int seq, unsigned int node, Ogre::Vector3 &rot,float start,float stop);

	void addNodeSetRot(unsigned int node, Ogre::Vector3 &rot);
	void addNodeAdjustRot(unsigned int node, Ogre::Vector3 &rot);


	void backupSequenceData();
	void addUltraframeSet(int seq);
	//void dropUltraframeSet(int seq);
	void addUltraframeSet(Ogre::String &seqName);
	void dropUltraframeSet(Ogre::String &seqName);

	bool hasUltraframe(int seq,int frame,int node,int type);
	void addUltraframe(int seq,int frame,int node,int type,int target,Ogre::Vector3 &value);
	void addUltraframeFromFile(int seq,int frame,int node,int type,int target,Ogre::Vector3 &value);
	void addUltraframeSingle(int seq,int frame,int node,int type,int target,Ogre::Vector3 &value);
	void addUltraframeNoInsert(int seq,int frame,int node,int type,int target,Ogre::Vector3 &value);
	void saveUltraframe(int seq,int frame,int node,int type,int target,Ogre::Vector3 &value);
	void saveUltraframeNoInsert(int seq,int frame,int node,int type,int target,Ogre::Vector3 &value);
	void getUltraframe(int seq, int frame, int node, int type,int *target,Ogre::Vector3 *outValue);
	void clearUltraframe(int seq, int frame, int node);
	void dropUltraframe(int seq, int frame, int node);
	void dropUltraframe(int seq,int frame,int node,int type);

	void saveUltraframes(int seq, const char *filename,bool append = false);
	//void loadUltraframes(int seq, const char *filename);
	void loadUltraframes(const char *filename);

	void addPlaylistSeq(int seq,int repeats,float speed);//this seq is index to whole sequences list on shape
	void dropPlaylistSeq(int playseq);//this one is index into playlist
	void savePlaylistSeq(int index,int seq,int repeats,float speed);
	int getNumPlaylistSeqs();
	int getPlaylistNum(int seq);//convert from sequence number to playlist index, cuz guiPopUpCtrl apparently has no way of 
	//getting an index of the selection, only the text of the selection, unless I'm missing it.
	int getPlaylistSeq(int index);
	int getPlaylistRepeats(int index);
	float getPlaylistSpeed(int index);
	void setPlaylist(int playlist_id);

	void savePlaylist(const char *filename);
	void loadPlaylist(const char *filename);
	//void runPlaylist();
	void clearPlaylist();

	void setShapeSize(float size) { mShapeSize = size; }
	float getShapeSize() { return mShapeSize; }

	void convertAckToKork(int ackID,int seq); 
	void convertKorkDefault(int ackID); 

	void setTarget(fxFlexBody *);
	int getNumUltraframes(int seq);
	bool hasUltraframesForNode(int seq,int node);
	bool hasUltraframesForType(int seq,int type);
	int getSeqNumKeyframes(int seqnum);
	void cropSequence(unsigned int seq,float start,float stop,const char *name);
	void setSequenceFrames(unsigned int seq,unsigned int frames);
	int getShapeConstructor();
	void saveShapeConstructor(const char *filename);
	void doMatrixFix(unsigned int seq,Ogre::Vector3 &eul1,Ogre::Vector3 &eul2);
	void moveToPosition(Ogre::Vector3,Ogre::String);
	void attackPosition(Ogre::Vector3,Ogre::String);
	void orientToPosition(Ogre::Vector3 target);
	void renameSequence(const char *oldName, const char *newName);
	void finishFollowEvent();
	void reloadSequences();
	void restoreSequences();

	void loadDatabaseIds();
	void loadKeyframeSets(int scene_id);
	void loadPlaylist(int scene_id);
	void loadPlaylistById(int playlist_id);

	void setupOrderNodes();
	void deactivateNodePlusChildNodes(int node);
	void activateNode(int node);
	bool hasRagdollBodyparts();

	//bool createFbxScene(KFbxSdkManager* pSdkManager, KFbxScene* pScene, int seq);
	//KFbxNode* createFbxSkeleton(KFbxScene* pScene, const char* pName);
	//KFbxNode* createFbxMesh( KFbxSdkManager* pSdkManager, KFbxScene* pScene, KFbxNode* kRoot, const  char* pName);
	//void animateFbxSkeleton(KFbxSdkManager* pSdkManager, KFbxScene* pScene, KFbxNode* pSkeletonRoot,int seq);
	//void linkMeshToSkeleton(KFbxSdkManager* kSdkManager, KFbxNode* kMesh, KFbxNode* kRoot);

	void clearJointMotors();
	//void addPersonaSequence();
	//void dropPersonaSequence();
	//void addPersonaSequenceEvent();
	//void dropPersonaSequenceEvent();

	//from shapebase in Torque EM
	Ogre::Matrix4 mGroundStartXform;
	std::vector<playlistSeq> mPlaylist;

	bool mRunningPlaylist;
	bool mIsMoveTargeting;
	float mMoveThreshold;
	Ogre::Vector3 mMoveTarget;
	Ogre::String mMoveSequence;

	int mPlaylistRepeats;
	int mCurrentPlaylistSeq;
	int mPlaylistTick;
	//////// End Ecstasy Motion from ShapeBase ////////////////

	bool mVerbose;


	/////////////// Ogre/AngelScript/SQLite  /////////////
	bool loadSQL();

	void addRef();
	void release();
	int doStuff(int arg);
	void examineEntity();
	void loadAnim(const char*);

	Ogre::Vector3 getNodePosition() const;
	void setNodePosition(const Ogre::Vector3 &pos);


};

#endif 
