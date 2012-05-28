////////////////////////////////////////////////////////////////////////////////////////////////////
//  nxPhysManager.h
//  Chris Calef
//
//  adapted from LRGRigidBodyManager.h
//	by Yossi Horowitz
//////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef	_NXPHYSMANAGER_H_
#define _NXPHYSMANAGER_H_


#include "EcstasyMotion/physManagerCommon.h"
//#include "EcstasyMotion/myStream.h"


#include "NxPhysics.h"
#include "NxCooking.h"
#include "Nxp.h"

//#include "aiseek.h"

// Forward declares
class physRigidBody;
class nxRigidBody;
class physJoint;
class nxJoint;
//class physSpring;
//class nxSpringAndDamper;
//class physFluid;
//class nxFluid;
//class physCloth;
//class nxCloth;
//class RenderClothExample;
//class fxFlexBody;
class SQLiteObject;

#define MAX_RAYCASTS 10

////////// ECSTASY SCENE EVENT TYPES /////////////
//IMPULSE EVENTS start at 1000
#define	SE_IMP_NULL             1000
#define	SE_IMP_FORCE            1001 // Impulse force.
#define	SE_IMP_TORQUE           1002 // Impulse torque.
#define	SE_IMP_MOTOR_TARGET     1003 // Impulse motor target (pulse in this direction then ragdoll?)
#define	SE_IMP_SET_FORCE        1004 // Constant force (until instructed otherwise).
#define	SE_IMP_SET_TORQUE       1005 // Constant torque (until instructed otherwise).
#define	SE_IMP_SET_MOTOR_TARGET 1006 // Set new motor target (until instructed otherwise).
#define	SE_IMP_SET_GLOBAL_FORCE 1007
#define	SE_IMP_MOVE             1008 // Instantaneous move
#define	SE_IMP_TURN             1009 // Instantaneous turn
#define	SE_IMP_RAGDOLL_FORCE    1010 // causes whole body ragdoll, not just local bodypart force.
#define	SE_IMP_RAGDOLL          1011 // cause whole body or bodypart to go ragdoll until told otherwise.
#define	SE_IMP_KINEMATIC        1012 // ... as above but opposite.  If node is -1 then whole body.
#define	SE_IMP_GLOBAL_TORQUE    1013 
#define	SE_IMP_MOTORIZE         1014
#define	SE_IMP_CLEAR_MOTOR      1015
#define	SE_IMP_EXPLOSION_CAUSE  1016
#define	SE_IMP_EXPLOSION_EFFECT 1017 // these have to have an ID of an EXPLOSION_CAUSE
#define	SE_IMP_WEAPON_CAUSE     1018
#define	SE_IMP_WEAPON_EFFECT    1019 // these have to have an ID of a WEAPON_CAUSE
#define	SE_IMP_MOVE_TO_POSITION 1020 // starts a moveTo operation, using action as personaAction name and value vector as target.
#define	SE_IMP_MOVE_TO_TARGET   1021 // starts a moveTo operation, using an actor (or other entity?) as target.
#define	SE_IMP_SET_POSITION     1022 // instant set position
#define	SE_IMP_SCRIPT           1100 // Call out to script function specified by action.

//DURATION EVENTS start at 2000
#define	SE_DUR_NULL             2000
#define	SE_DUR_FORCE            2001 // Constant force for duration.
#define	SE_DUR_TORQUE           2002 // Constant torque for duration.
#define	SE_DUR_MOTOR_TARGET     2003 // Motor target for duration. 
#define	SE_DUR_PLAY_SEQ         2004 // Play a sequence, kinematic physics.
#define	SE_DUR_ACTION_SEQ       2005 // Play a sequence non-kinematic, with joint motors.
#define	SE_DUR_ACTION           2006 // Run an .action file
#define	SE_DUR_GLOBAL_FORCE     2007
#define	SE_DUR_GLOBAL_TORQUE    2008
#define	SE_DUR_RAGDOLL          2009 
#define	SE_DUR_KINEMATIC        2010
#define	SE_DUR_MOTORIZE         2011
#define	SE_DUR_SCRIPT           2100 // Call out to script function specified by action.

//INTERPOLATION EVENTS start at 3000
#define	SE_INTERP_NULL          3000
#define	SE_INTERP_FORCE         3001 // Force interpolation. 
#define	SE_INTERP_TORQUE        3002 // Torque interpolation.
#define	SE_INTERP_MOTOR_TARGET  3003 // Motor target interpolation.
#define	SE_INTERP_MOVE          3004 // Interpolated move.
#define	SE_INTERP_TURN          3005 // Interpolated turn.
#define	SE_INTERP_GLOBAL_FORCE  3006 
#define	SE_INTERP_GLOBAL_TORQUE 3007 
#define	SE_INTERP_SCRIPT        3100 // Call out to script function specified by action.

//FOLLOW EVENTS start at 4000
#define	SE_FOLLOW_NULL             4000
#define	SE_FOLLOW_MOVE_TO_POSITION 4001
#define	SE_FOLLOW_SCRIPT           4100

//REALTIME EVENTS start at 5000



struct nxRaycastData
{
	Ogre::Vector3 start;
	Ogre::Vector3 dir;
	float force;
	float damage;
	const char *Dirt;
	const char *Brick;
	const char *Water;
	const char *Blood;
	int createStep;
};

struct ecstasySceneEvent
{
	int eventID;//For looking up this event later from script	
	int dbID;//database ID for this event
	int eventType;
	iPhysUser *physUser;//iPhysUser so we can have events for rigid bodies, flexbodies, cloth etc.

	float time;//Time in seconds, from scene start.
	float duration;//Duration in seconds.
	int node;//Q: Index into bodyparts list on flexbody, or shape node index? Or depends on eventType?

	ecstasySceneEvent *next;//Next event of this type, for this node on this body.
	ecstasySceneEvent *prev;//In case we ever need to play a scene backwards.
	ecstasySceneEvent *cause;//In case another event is the direct cause of this one, use this to determine force direction, magnitude etc.
	//Used only for types that require interpolation between events, not for impulses. 

	Ogre::Vector3 value;
	std::string action;
};

//First, we need to track, for each actor, what his sequence names are, for each persona action.
//
//struct ecstasyPersonaSequence
//{
//	iPhysUser *physUser;
//	std::string action;//Run, Walk, SitDown, GetUpFromSitting, GetUpFromProneLSide, GetUpFromProneRSide, GetUpFromProneFront, ... 
//	std::string sequence;//lSideGetup.dsq, rSideGetup.dsq, ...
//	float timescale;//speed?
//};


class nxPhysManager : public physManagerCommon
{
 public:
  NxPhysicsSDK *mPhysicsSDK;
  NxScene *mScene;
  NxScene *mHWScene;
  NxScene *mFluidScene;

  //Ogitors::OgitorsScriptInterpreter *mInterpreter;

  unsigned int mNumMeshBodies;
  unsigned int mNumMeshes;
  unsigned int mNumRaycasts;
  unsigned int mNumFlexbodies;


  bool mIsExiting;
  bool mIsReallyExiting;// Is this unnecessary yet?? 

  NxConvexMesh *mMeshes[MAX_MESHES];

  nxRaycastData mRaycasts[MAX_RAYCASTS];

  // A list of the rigid body objects currently in the world.
  std::vector<nxRigidBody*>       mBodyList;
  //std::vector<nxRigidBody*>       mBodySetupList;
  //std::vector<nxRigidBody*>       mBodyReleaseList;

  std::vector<nxJoint*>           mJointList;
  //std::list<nxJoint>           mJointSetupList;
  //std::list<nxJoint>           mJointReleaseList;

  //std::list<fxFlexBody>        mFlexBodyList;
  //std::list<fxFlexBody>        mFlexBodySetupList;
  //std::list<fxFlexBody>        mFlexBodyReleaseList;

  //std::list<nxSpringAndDamper> mSpringList;
  //std::list<nxSpringAndDamper> mSpringSetupList;
  //std::list<nxSpringAndDamper> mSpringReleaseList;

  //std::list<nxFluid>           mFluidList;
  //std::list<nxFluid>           mFluidSetupList;
  //std::list<nxFluid>           mFluidReleaseList;

  //std::list<nxCloth>           mClothList;
  //std::list<nxCloth>           mClothSetupList;
  //std::list<nxCloth>           mClothReleaseList;

  //std::list<RenderClothExample>  mClothList;//TEMP, testing...
  //std::list<RenderClothExample>  mClothSetupList;
  //std::list<RenderClothExample>  mClothReleaseList;

  //std::list<nxRaycastData>     mRaycastList;

  //std::list<ecstasySceneEvent> mSceneEventList;//All the events in the current scene.
  std::list<ecstasySceneEvent> mImpulseEventList;
  std::list<ecstasySceneEvent> mDurationEventList;
  std::list<ecstasySceneEvent> mInterpolationEventList;
  std::list<ecstasySceneEvent> mFollowEventList;

  std::vector<ecstasySceneEvent *> mCurrDurationEvents;
  std::vector<ecstasySceneEvent *> mCurrInterpolationEvents;
  //std::vector<ecstasySceneEvent *> mCurrFollowEvents;//All the events that are currently being played,
  //ie have passed their start time but not hit their start+duration time yet, or not hit next event
  //yet for interpolation or follow events.  Do not allocate or free events from here!  Main list does that.

  ecstasySceneEvent *mLastImpulseEvent;//Use these to speed up the search by storing the last
  ecstasySceneEvent *mLastDurationEvent;//event found for each type.
  ecstasySceneEvent *mLastInterpolationEvent;
  ecstasySceneEvent *mLastFollowEvent;

  unsigned int mSceneEventCounter;
  unsigned int mSceneStartStep;//in milliseconds - this is global time when we started playing this scene.
  float mSceneStartTime;//in seconds - this is the time into the scene that we started playing it. (later!)
  float mSceneDuration;//in seconds
  bool mSceneRecordLocal;//for how we record scenes, to output to bvh for different apps.  For now
	//we only have two choices, local sequences (local to actor position/orientation) for Ecstasy,
	//and global sequences (where position data is global) for iClone, which also adds ten frame 
	//interpolation buffer to get each character from origin to their starting position.

  //SQLiteObject *mSQL;

  std::string mMissionName;
  int mMissionId;
  std::string mSceneName;
  int mSceneId;

  nxPhysManager();
  ~nxPhysManager();

  void init();
  void createScene();
  void destroyScene();
  //void destroy();
  //bool onAdd();

  //SQLiteObject *getSQL() { return mSQL; }

  NxPhysicsSDK *getPhysicsSDK() { return mPhysicsSDK; }
  NxScene *getScene() { return mScene; }
  NxScene *getHWScene() { return mHWScene; }
  NxScene *getFluidScene() { return mFluidScene; }
  NxConvexMesh *getMesh(unsigned int i) { return mMeshes[i]; }

  void stepPhysics();
  void stopPhysics();
  void startPhysics();

  //void addTerrain(const Ogre::Vector3 &pos,const Ogre::Vector3 &extent);
  //void addTerrain(const Ogre::Vector3 &pos);
  void addTerrain();

  physRigidBody *createRigidBody();
  physJoint *createJoint();
  //physSpring *createSpring();
  ////physFluid *createFluid();
  //physCloth *createCloth();

  //void addRigidBody(physRigidBody*);
  void removeRigidBody(physRigidBody*);
  //void addRigidBodySetup(physRigidBody*);
  //void removeRigidBodySetup(physRigidBody*);

  //void addJoint(physJoint*);
  void removeJoint(physJoint*);
  //void addJointSetup(physJoint*);
  //void removeJointSetup(physJoint*);

  //void addFlexBody(fxFlexBody*);
  //void removeFlexBody(fxFlexBody*);
  //void addFlexBodySetup(fxFlexBody*);
  //void removeFlexBodySetup(fxFlexBody*);

  //void addSpring(physSpring*);
  //void removeSpring(physSpring*);
  //void addSpringSetup(physSpring*);
  //void removeSpringSetup(physSpring*);

  //void addFluid(physFluid*);
  //void removeFluid(physFluid*);
  //void addFluidSetup(physFluid*);
  //void removeFluidSetup(physFluid*);

  //void addCloth(physCloth*);
  //void removeCloth(physCloth*);
  //void addClothSetup(physCloth*);
  //void removeClothSetup(physCloth*);

  //void addCloth(RenderClothExample*);
  //void removeCloth(RenderClothExample*);
  //void addClothSetup(RenderClothExample*);
  //void removeClothSetup(RenderClothExample*);

  //void addPhysMaterial(physMaterial *);
  //void addRaycast(nxRaycastData);

  //bool gridToWorld(const Point2I & gPos, Ogre::Vector3 & wPos);
  //float getTerrHeight(Point2F pos, Ogre::Vector3 *normal=NULL);
  bool getDebugRender() { return mDebugRender; }
  void setDebugRender(bool dr) { mDebugRender = dr; }
  void debugRender();
  ////static void renderRaycast(SceneGraph *graph, const SceneState *state);
  //Ogre::Vector3 nxCastRay(Ogre::Vector3,Ogre::Vector3,float,float,const char *,const char *,const char *,const char *);
  //Ogre::Vector3 nxCastRay(nxRaycastData);
 
  //void debugRender(SceneGraph *graph, const SceneState *state);
  //void addAtlasTerrain();

  //int addSceneEvent(int eventType,iPhysUser *physUser,float time,float duration,int node,Ogre::Vector3 &value,const char *actionName,int dbID = -1);
  //bool handleSceneEvent(ecstasySceneEvent *sceneEvent);
  //bool handleInterpolationSceneEvent(ecstasySceneEvent *sceneEvent,float delta);
  //bool handleFollowSceneEvent(ecstasySceneEvent *sceneEvent);
  //
  //void clearSceneEvents();
  //void saveSceneEventsToFile(const char *filename);
  //void loadSceneEventsFromFile(const char *filename);
  //void loadSceneEvents(int scene_id);
  //void saveScene();

  //int getNumSceneEvents(int eventType,iPhysUser *physUser,int node);
  //int getEventIdByNum(int eventType,iPhysUser *physUser,int node,int eventNum);
  //int getEventIdByTime(int eventType,iPhysUser *physUser,int node,float time);
  //ecstasySceneEvent *getEvent(int eventID);

  void scriptInit();
  void doStuff();

};


#endif 
