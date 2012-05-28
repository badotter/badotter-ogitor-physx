#ifndef GA_ACTION_H
#define GA_ACTION_H 
//#include <ode/ode.h>
//#include <drawstuff/drawstuff.h>
//#include "vect3.h"


//#include "platform/platform.h"
//#include "console/consoleTypes.h"
//#include "math/mathio.h"
//#include "math/mathUtils.h"
//#include "math/mRandom.h"
//#include "console/simBase.h"
//#include "T3D/gameBase/gameBase.h"
//#include "core/stream/bitStream.h"

#include "EcstasyMotion/physRigidBody.h"
#include "EcstasyMotion/physRigidBodyCommon.h"
#include "EcstasyMotion/physJoint.h"
#include "EcstasyMotion/fxFlexBody.h"
#include "EcstasyMotion/fxFlexBodyPart.h"
#include "EcstasyMotion/gaObservation.h"

#define GA_MAX_ACTIONS              2000
#define GA_MAX_ACTION_SETS          100
#define GA_MAX_FILE_ACTIONS         50
#define GA_MAX_FILE_ACTIONS_PER     20
#define GA_MAX_CROSSOVER_ACTIONS    1000
#define GA_MAX_BODIES               200
#define GA_MAX_OBS_SETS             60 // This is higher than populations, because it was expecting to be used at bodypart-group level.
#define GA_MAX_BODYGROUPS           24
#define GA_MAX_PARTS_PER_BODYGROUP  16
#define GA_MAX_POPS                 8
#define GA_MAX_FITNESS_DATA			6

//goal set is the generation by which you hope to have discovered a reasonable solution sequence
//use it to measure progress (difference between score of best sequence and goal score)

class fxFlexBody;
class gaActionUser;

//struct gaFileAction {
//  int part;
//  int numActions;
//  float times[GA_MAX_FILE_ACTIONS_PER];
//  Ogre::Vector3 forces[GA_MAX_FILE_ACTIONS_PER];
//};

struct gaBodyGroup
{
	unsigned int mNumBodyParts;
	unsigned int mBodies[GA_MAX_PARTS_PER_BODYGROUP];
	unsigned int mActionGoal;
};

class gaAction
{//for arbitrary duration
 public:
  int mBodyIndex;
  Ogre::Vector3 mForces;//-1.0 to 1.0 on X and Y, no Z yet
  Ogre::Quaternion mQuat;//HERE: when you are starting with a quat, as with sequence actions,  why turn it 
  //into an Euler and then change it back later???

  //(0.0 - 1.0) values referencing this action in the timespace of the sequence. Saved values.
  float mStart;
  float mDuration;

  //per sequence, depending on gaActionSequence.numSteps
  int mStartStep;
  int mEndStep;
  int mNumSteps;

  bool mIsTraining;

  gaAction();
  gaAction(gaAction *);
  gaAction(int,int,float,float);
  gaAction(int,int,float,float,Ogre::Vector3 &);
  ~gaAction();

  void set(gaAction *);
  void set(int,int,float,float,Ogre::Vector3 &);
  void setSteps(int);
  void clear();
  //void save(FILE *);
  //void load(FILE *);
};


class gaActionSet 
{
public:	  
  gaActionUser *mAU;

  int mNumBodies;

  int mNumActions;
  int mCurrAction;

  int mNumSlices; //just in case we're making a timeslice-style action set, see below.
  int mNumSteps;

  float mCurrTime;
  float mDuration;

  float mScore;
  float mSOA;//score_over_avg
  float mSMA;//score_minus_avg

  float mWeight;//lower weight goes to the top
  int mBodyGroup;//group id, as in whole body=1, legs=2, arms=3, head+neck=4, torso+arms=5
  //nxRigidBody *mBodies[GA_MAX_BODIES];
  //int *mActiveBodyIndices;
  int parts[GA_MAX_BODIES];
  Ogre::String mSequenceName;

  //these relate to blended actions, i.e. "shoot gun" arm movement given higher 
  //weight than whole body running motion, so it can take precedence.
  gaAction *mActions[GA_MAX_ACTIONS];

  gaActionSet();
  gaActionSet(int,gaActionUser *);
  gaActionSet(gaActionSet *);
  ~gaActionSet();

  void init(int);
  void clear();
  void save(const char *);
  void load(const char *);
  void saveAsText(const char *);
  bool loadAsText(const char *, bool dynamic=false, bool exclusive=true, bool repeat=false);
  void loadActionSetAsText(const char *);
  bool loadSequence(const char *);
  void addAction(gaAction *);
  void removeAction(int);
  void addRandomActions();
  void setRandomActions();
  void addRotateActions(int,int);
  void setRotateActions(int,int);
  void addZeroActions(int,physRigidBody *);
  void setZeroActions();
  void set(gaActionSet *);
  void mateSets(gaActionSet *,gaActionSet *,int);
  void mutate();
  void mutate(float);
  void setScore(float);
  float avgForces();
  float sumForces();
};

/////////////////////////////////////////////////////////
//     gaActionSetTimeslice
//the timeslice version of gaActionSet
//consists of a certain duration filled all the way up with 
//actions, such that every muscle is doing something all the time.
//divided into equal-length time "slices".
//inefficient, because numActions = numMuscles * numSlices
//however, makes it easier to write a GA mating/crossover technique.


class gaActionGroup
{
public:
  gaActionUser *mAU;

  int mNumBodies;
  int mNumSets;
  int mCurrSet;
  //int mNumPops;
  //int mCurrPop;
  int mNumIntermediates;
 
  gaActionSet *mBestSet;
  gaActionSet *mActionSets[GA_MAX_ACTION_SETS];
  gaActionSet *mIntermediates[GA_MAX_ACTION_SETS];
  int *mActiveBodyIndices;

  gaActionGroup();
  gaActionGroup(int);
  gaActionGroup(int,int,gaActionUser *);
  //gaActionGroup(char *,int);
  gaActionGroup(char *);
  ~gaActionGroup();

  void init(int);
  void init(int,int);
  //void save(char *);
  //void load(char *);
  //void loadAction(char *,int,Entity *);
  //void loadActionFile(char *);
  //void addSet(gaActionSet *);
  void clear();
  void sort();
  void repopulate();//(char *);
  void repopulateFromOne();//(char *);
  void randomize();
  void randomize(int);
  void setNumBodies(int);
  void immigrate(gaActionGroup *);
  //void loadBest();
  //void repopulate(int,int);
};


//struct gaFitnessData : public GameBaseData
//{
//  typedef GameBaseData Parent;
//
//  public:
//  DECLARE_CONOBJECT(gaFitnessData);
//
//  Ogre::Vector3 mPositionGoal;
//  Point3I mPositionGoalType;
//  float mRotationGoal;
//  int mRotationGoalType;
//
//  Ogre::String mBodypartName;
//
//  gaFitnessData();
//  ~gaFitnessData();
//  
//  bool preload(bool bServer, String &errorStr);
//  bool  onAdd();
//  static void initPersistFields();
//  virtual void packData(BitStream*);
//  virtual void unpackData(BitStream*);
//};

//struct gaActionUserData : public GameBaseData
//{
//  typedef GameBaseData Parent;
//
//  public:
//  DECLARE_CONOBJECT(gaActionUserData);
//  
//  float mMutationChance;
//  float mMutationAmount;
//  int mNumPopulations;
//  float mMigrateChance;
//  //int mNumSequenceSteps;
//  int mNumRestSteps;
//  int mObserveInterval;
//  int mNumActionSets;
//  //int mNumSlices;
//  int mNumSequenceReps;
//
//  StringTableEntry mActionName;
//  StringTableEntry mFitnessFunction;
//
//  gaFitnessData* mFitnessData[GA_MAX_FITNESS_DATA];
//  unsigned int mFitnessDataID[GA_MAX_FITNESS_DATA];
//
//  gaActionUserData();
//  ~gaActionUserData();
//  
//  bool preload(bool bServer, String &errorStr);
//  bool  onAdd();
//  static void initPersistFields();
//  virtual void packData(BitStream*);
//  virtual void unpackData(BitStream*);
//
//};

class gaActionUser //: public GameBase
{
private:
	//typedef GameBase Parent;
	//gaActionUserData *mDataBlock;

public:
		
  //DECLARE_CONOBJECT(gaActionUser);

  int mNumBodies;
  int mNumActiveBodies;
  int mNumBodyGroups;
  int mNumSets;
  int mNumPops;
  int mCurrPop;

  int mSimStep;
  int mCurrGen;
  int mDone;

  bool mIsResetting;

  int mActionState;
  int mHeadActionState;//not used yet
  int mTailActionState;//not used yet
  int mGoalState;
  
  int mRandSeed;
  int mForwardSteps;//TEMP

  float mMutationChance;
  float mMutationAmount;

  //all below replaced by datablock
  //int mNumSequenceSteps;
  //int mNumRestSteps;
  //int mObserveInterval;
  //int mNumCompetitors;
  //int mNumTimeSlices;
  //int mNumSeqReps;

  int mCurrSeqRep;
  int mNumObsSets;

  float mResetForce;
  float mActionForce;
  float mForwardForce;//TEMP?

  //named nodes
  int mHeadIndex;
  int mNeckIndex;
  int mBodyIndex;
  int mRightFrontIndex;//for biped, arms
  int mLeftFrontIndex;//for quadruped, front legs
  int mRightBackIndex;//biped legs, quad back legs
  int mLeftBackIndex;

  fxFlexBody *mFlexBody;
  physRigidBody *mBodies[GA_MAX_BODIES];
  physRigidBody *mActiveBodies[GA_MAX_BODIES];
  int mActiveBodyIndices[GA_MAX_BODIES];

  Ogre::Vector3 mBodyBasePosition;
  Ogre::Quaternion mBodyBaseQuat;

  //Ogre::Vector3 mBodyStartPositions[GA_MAX_BODIES];
  //Ogre::Quaternion mBodyStartQuats[GA_MAX_BODIES];
  Ogre::Vector3 mBodyInitialPositions[GA_MAX_BODIES];
  Ogre::Quaternion mBodyInitialQuats[GA_MAX_BODIES];

  Ogre::String mShapeName;
  Ogre::String mCurrAction;

  gaActionSet *mActionSet;//s[GA_MAX_BODYGROUPS];
  gaActionGroup *mActionGroups[GA_MAX_POPS];

  gaObservationUnit *mObsUnit;//s[GA_MAX_BODYGROUPS];
  gaObservationSequence *mObsSeq;//s[GA_MAX_BODYGROUPS];

  gaObservationSequenceSet *mObsSets[GA_MAX_OBS_SETS];

  gaBodyGroup *mBodyGroups[GA_MAX_BODYGROUPS];

  gaActionUser();//fxFlexBody *
  ~gaActionUser();

  //////////////////
  //bool onAdd();
  //void onRemove();
  //bool onNewDataBlock(GameBaseData* dptr, bool reload);

  //unsigned int packUpdate(NetConnection *, unsigned int, BitStream *);
  //void unpackUpdate(NetConnection *, BitStream *);

  //static void initPersistFields();
	/////////////////
  void setup();
  void addBody(physRigidBody *);
  void addActiveBody(physRigidBody *,int);

  void addBodyGroup(const char *);
  void addBodyGroupBody(const char *,int);

  void think();
  void thinkAction();
  void thinkSequence();
  //void thinkForward();
  //void thinkDynamic();
  void act();

  void setAction(const char *);

  // observeBodypart(unsigned int kBodyId, Ogre::Vector3 kPosScale, Point3I kPosDiffFromStart, float kRotScale, int kRotDiffFromStart)
  void observeBodypart(unsigned int,Ogre::Vector3,Ogre::Vector3,float,int);

  void saveAction(const char *);
  bool loadAction(const char *);
  bool loadAction(const char *,float);
  void loadActionSetOnly(const char *);//rename

  void saveAll();
  void loadAll();//const char *

  //void setGoalSingleAction();
  //void setGoalForward();
  //void setGoalDynamic();
  void setForwardForce(float);

  void motorize();
};

#endif //GA_ACTION_H
