////////////////////////////////////////////////////////////////////////////////////////////////////
//  physManagerCommon
//  Chris Calef
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef	_PHYSMANAGERCOMMON_H_
#define  _PHYSMANAGERCOMMON_H_

#include "OgreTerrain.h"
#include "EcstasyMotion/physManager.h"

///////////////////////////////////////////////

class physManagerCommon : public physManager
{
 public:

  static physManager *mPM;
  static asIScriptEngine *mScriptEngine;  
  //static SQLiteObject *mSQL;

  Ogre::MaterialPtr mWireMat;
  Ogre::ManualObject *mDR;//mDebugRenderObject
  Ogre::Terrain *mTerrain;

  Ogre::Vector3 mDefaultGravity;
  float mStepTime;
  float mTimeFactor;
  int mTimeDelta;
  int mCurrStep;//Confusing syntax - mCurrStep is incremented once per step.
  unsigned int mCurrStepTime;//CurrStepTime and LastStepTime are actual game times in MS.
  unsigned int mLastStepTime;
  int mType;
  int mNumMaterials;
  
  bool mDebugRender;

  float mBaseDynamicFriction;
  float mBaseStaticFriction;
  float mBaseRestitution;
  int mNextActorGroup;


  physMeshBody *mMeshBodies[MAX_MESH_BODIES];
  //physMaterial *mPhysMaterials[MAX_MATERIALS];

  physManagerCommon() {};
  ~physManagerCommon() {};

  static physManager *getPM() { return mPM; }
  static asIScriptEngine *getScriptEngine() { return mScriptEngine; }
  Ogre::ManualObject *getDebugRenderObject() { return mDR; }
  
  float getStepTime() { return mStepTime; }
  int getTimeDelta() { return mTimeDelta; }
  float getTimeFactor() { return mTimeFactor; }
  void setTimeFactor(float s) { mTimeFactor = s; }
  int getType() { return mType; }
  void setType(int t) { mType = t; }
  int getLastStepTime() { return mLastStepTime; }
  void setLastStepTime(int s) { mLastStepTime = s; }
  int getNextActorGroup() { return mNextActorGroup++; }
  int getCurrentActorGroup() { return mNextActorGroup; }
  bool getDebugRender() { return mDebugRender; }

  Ogre::MaterialPtr getWireMat() { return mWireMat; }
  //void renderObject(ObjectRenderInst *ri, BaseMatInstance* );
};


#endif 
