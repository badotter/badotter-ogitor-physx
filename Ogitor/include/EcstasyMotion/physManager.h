////////////////////////////////////////////////////////////////////////////////////////////////////
//  physManager.h
//  Chris Calef
//  Brokeass Games 2010
//
//  adapted from LRGRigidBodyManager.h
//	 by Yossi Horowitz
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef	_PHYSMANAGER_H_
#define  _PHYSMANAGER_H_


//#include "T3D/physicsBAG/iPhysUser.h"
//#include "T3D/physicsBAG/physMaterial.h"

//#include <stdio.h>

#include <Ogre.h>
#include "Ogitors.h"
#include "OgitorsRoot.h"
#include "BaseEditor.h"
#include "angelscript.h"
//#include "OgitorsScriptInterpreter.h"
//#include "OgitorsScriptConsole.h"

#include "EcstasyMotion/iPhysUser.h"
#include "EcstasyMotion/SQLiteObject.h"
#include "../../Plugins/OgAngelScript/AngelScriptInterpreter.h"

#define PHYS_NX 1
#define PHYS_ODE 2
#define PHYS_BULLET 2

//FIX - vectorize all these things!
#define MAX_MESH_BODIES 300
#define MAX_MESHES      600
#define MAX_MESHES_PER_RB 200
#define MAX_MATERIALS   200

#ifndef NULL
#  define NULL 0
#endif

///////////////////////////////////////////////


class physRigidBody;
class physJoint;
//class physSpring;
//class physFluid;
//class physCloth;
//class physMaterial;
//class fxFlexBody;
//class RenderClothExample;

class SQLiteObject;

enum physShapeType
{//FIX 0 should be PHYS_SHAPE_NULL?
	PHYS_SHAPE_BOX = 0,
	PHYS_SHAPE_CAPSULE,
	PHYS_SHAPE_SPHERE,
	PHYS_SHAPE_CONVEX,
	PHYS_SHAPE_COLLISION,
	PHYS_SHAPE_TRIMESH
};

enum physChainType
{//PHYS_CHAIN_NULL = 0?
	PHYS_CHAIN_SPINE = 0,
	PHYS_CHAIN_RIGHT_ARM,
	PHYS_CHAIN_LEFT_ARM,
	PHYS_CHAIN_RIGHT_LEG,
	PHYS_CHAIN_LEFT_LEG,
	PHYS_CHAIN_RIGHT_WING,
	PHYS_CHAIN_LEFT_WING,
	PHYS_CHAIN_TAIL,
	PHYS_CHAIN_TONGUE
};

enum physJointType
{//PHYS_JOINT_NULL = 0
	PHYS_JOINT_PRISMATIC = 0,
    PHYS_JOINT_REVOLUTE,
    PHYS_JOINT_CYLINDRICAL,
    PHYS_JOINT_SPHERICAL,
    PHYS_JOINT_POINT_ON_LINE,
    PHYS_JOINT_POINT_IN_PLANE,
    PHYS_JOINT_DISTANCE,
    PHYS_JOINT_PULLEY,
    PHYS_JOINT_FIXED,
    PHYS_JOINT_D6
};

enum physEntityType 
{
   PHYS_TYPE_UNDEFINED = 0,
   PHYS_TERRAIN,
   PHYS_INTERIOR,
   PHYS_STATIC,
   PHYS_RIGID_BODY,
   PHYS_FLEX_BODY,
   PHYS_FLEX_BODY_PART,
   PHYS_FLUID,
   PHYS_CLOTH,
   PHYS_TRIGGER
};

enum physEntitySubType 
{
	PHYS_SUB_UNDEFINED = 0,

	//PHYS_SUB_RIGID_STATIC,
	//PHYS_SUB_RIGID_PROJECTILE,
	//PHYS_SUB_RIGID_INFLICTOR,

	PHYS_SUB_FLEX_BIPED,
	PHYS_SUB_FLEX_QUADRUPED,
	PHYS_SUB_FLEX_MULTIPED,
	PHYS_SUB_FLEX_TREE//, ...
	
	//PHYS_SUB_CLOTH_TEARABLE,
	//PHYS_SUB_FLUID_,
	//PHYS_SUB_INTERIOR_BREAKABLE,
	//PHYS_SUB_TERRAIN_DIGGABLE,

	// Danger -- the line between flags on the object and different subtypes is thin.
	// I think BIPED/QUADRUPED/TREE are helpful, PROJECTILE/INFLICTOR/TEARABLE etc. maybe not so much.
};

/////////////////////////////

struct physMeshBody
{
	std::string shapeName;//full path to dts
   unsigned int startMesh;
   unsigned int numMeshes;
};

struct physShapeData 
{
   physEntityType mEntityType;
   iPhysUser *mPhysUser;
   int index;
};

struct physVertSort {
   unsigned int index;
   float dist;
};

/////////////////////////////
//Move the following to a general Ecstasy Motion utilities/math file.

//Is this really not done anywhere in the entire Ogre/Ogitor project?? Can't find...
#define M_PI 3.1415927

inline float mDegToRad(float d)
{
   return((d * M_PI) / 180.0f);
}

inline float mRadToDeg(float d)
{
   return((d * 180.0f) / M_PI);
}

/////////////////////////////

class physManager// : public virtual IProcessManager
{
 public:
  physManager();
  virtual ~physManager();//virtual

  ////init() = whatever you do before creating the scene/world
  virtual void init()=0;

  ////createScene() = creating the space/world/scene, whatever you need
  ////to have done before you start creating bodies.
  //virtual void createScene() = 0;
  //virtual void destroyScene() = 0;

  //virtual void processTick()=0;
  //virtual void interpolateTick( F32 delta )=0;
  //virtual void advanceTime( F32 timeDelta )=0;

  virtual void stepPhysics()=0;
  //virtual F32 getStepTime()=0;
  //virtual F32 getTimeFactor()=0;
  //virtual S32 getTimeDelta()=0;
  //virtual void setTimeFactor(F32)=0;

  virtual void debugRender()=0;

  //virtual S32 getLastStepTime() = 0;
  //virtual void setLastStepTime(S32) = 0;

  //virtual S32 getType() = 0;
  //virtual void setType(S32) = 0;
  //
  //virtual bool getDebugRender() = 0;

  //virtual S32 getNextActorGroup()=0;
  //virtual S32 getCurrentActorGroup()=0;

  virtual Ogre::MaterialPtr getWireMat()=0;
  virtual Ogre::ManualObject *getDebugRenderObject()=0;

  ////start/stopPhysics() = for threaded physics applications, stop the 
  ////thread before making changes to the simulation.
  virtual void startPhysics() = 0;
  virtual void stopPhysics() = 0;
  //
  virtual physRigidBody *createRigidBody() = 0;
  virtual physJoint *createJoint() = 0;
  //virtual physSpring *createSpring() = 0;
  ////virtual physFluid *createFluid() = 0;
  //virtual physCloth *createCloth() = 0;

  //virtual void addRigidBody(physRigidBody*) = 0;
  virtual void removeRigidBody(physRigidBody*) = 0;
  //virtual void addRigidBodySetup(physRigidBody*) = 0;
  //virtual void removeRigidBodySetup(physRigidBody*) = 0;

  //virtual void addJoint(physJoint*) = 0;
  virtual void removeJoint(physJoint*) = 0;
  //virtual void addJointSetup(physJoint*) = 0;
  //virtual void removeJointSetup(physJoint*) = 0;

  //virtual void addSpring(physSpring*) = 0;
  //virtual void removeSpring(physSpring*) = 0;
  //virtual void addSpringSetup(physSpring*) = 0;
  //virtual void removeSpringSetup(physSpring*) = 0;

  ////virtual void addFluid(physFluid*) = 0;
  ////virtual void removeFluid(physFluid*) = 0;
  ////virtual void addFluidSetup(physFluid*) = 0;
  ////virtual void removeFluidSetup(physFluid*) = 0;

  ////virtual void addCloth(physCloth*) = 0;
  ////virtual void removeCloth(physCloth*) = 0;
  ////virtual void addClothSetup(physCloth*) = 0;
  ////virtual void removeClothSetup(physCloth*) = 0;

  //virtual void addCloth(RenderClothExample*) = 0;
  //virtual void removeCloth(RenderClothExample*) = 0;
  //virtual void addClothSetup(RenderClothExample*) = 0;
  //virtual void removeClothSetup(RenderClothExample*) = 0;

  //virtual void addFlexBody(fxFlexBody*) = 0;
  //virtual void removeFlexBody(fxFlexBody*) = 0;
  //virtual void addFlexBodySetup(fxFlexBody*) = 0;
  //virtual void removeFlexBodySetup(fxFlexBody*) = 0;

  //virtual void addPhysMaterial(physMaterial *)=0;

  //virtual bool gridToWorld(const Point2I & gPos, Point3F & wPos)=0;
  //virtual F32 getTerrHeight(Point2F pos, Point3F *normal=NULL)=0;
  //virtual void addTerrain()=0;
  //
  ////virtual void addAtlasTerrain()=0;
};

physManager* createPhysicsManager(int type);
void destroyPhysicsManager();

#endif 
