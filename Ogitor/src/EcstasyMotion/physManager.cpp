//////////////////////////////
//
//
//
//////////////////////////////

#include "EcstasyMotion/physManager.h"
#include "EcstasyMotion/physManagerCommon.h"
#include "EcstasyMotion/nxPhysManager.h"
////#include "T3D/physicsBAG/odePhysManager.h"

//struct physVertSort;
class fxFlexBody;

physManager *physManagerCommon::mPM = 0;
asIScriptEngine *physManagerCommon::mScriptEngine = 0;
//SQLiteObject *physManagerCommon::mSQL = 0;

SQLiteObject *gSQL;
Ogitors::OgitorsScriptConsole *gConsole;
fxFlexBody *gTweakerBot;
//class odePhysManager;

physManager::physManager()
{

}

physManager::~physManager()
{

}

physManager* createPhysicsManager(int type)
{
	if (type==PHYS_NX) {
		nxPhysManager *PM = new nxPhysManager();
		PM->setType(type);
		// Con::printf("created nxPhysManager");
		return (physManager*)PM;
	} else if (type==PHYS_ODE) {
		//odePhysManager *PM = new odePhysManager();
		//PM->setType(type);
		//return (physManager*)PM;
	} else return NULL;
	return 0;
}

void destroyPhysicsManager()
{
	physManagerCommon *PM = (nxPhysManager *)physManagerCommon::getPM();
	if (PM->mType==PHYS_NX)
	{
		nxPhysManager *kPM = dynamic_cast<nxPhysManager *>(PM);
		kPM->destroyScene();
		delete kPM;
	}
}

//
//void vertSorter(physVertSort *vs, S32 numVS)
//{
//   S32 ja,jb;
//   for (ja=0;ja<numVS;ja++) {
//      for (jb=0;jb<numVS-1;jb++) {
//         physVertSort temp;
//         if (vs[jb].dist>vs[jb+1].dist) {
//            temp.dist = vs[jb].dist;
//            temp.index = vs[jb].index;
//            vs[jb].dist = vs[jb+1].dist;
//            vs[jb].index = vs[jb+1].index;
//            vs[jb+1].dist = temp.dist;
//            vs[jb+1].index = temp.index;
//         }
//      }
//   }
//}
//
//ConsoleFunction( createPhysicsManager, void, 1, 2, "createPhysicsManager(int type) 1 = PhysX, 2 = ODE")
//{
//   S32 physType = dAtoi(argv[1]);
//   if (!physType) physType = 1;
//   if (!createPhysicsManager(physType)) 
//      Con::errorf("Physics Manager FAILED.");
//}
//ConsoleFunction( destroyPhysicsManager, void, 1, 1, "destroyPhysicsManager()")
//{
//	destroyPhysicsManager();
//}
//
//ConsoleFunction( addTerrain, void, 1, 1, "Add Terrain")
//{
//  physManagerCommon::getPM()->addTerrain();
//}
//
//
//ConsoleFunction( physCreateScene, void, 1, 1, "")
//{
//  physManagerCommon::getPM()->createScene();
//  Con::errorf("creating physics scene!!");
//  return;
//}
//
//ConsoleFunction( physEndingMission, void, 1, 1, "")
//{
//  if (physManagerCommon::getPM()->getType() == PHYS_NX) 
//	  ((nxPhysManager *)physManagerCommon::getPM())->mIsExiting = true;
//
//  return;
//}
//
//
//ConsoleFunction( physDestroyScene, void, 1, 1, "")
//{
//  physManagerCommon::getPM()->destroyScene();
//  return;
//}
//
//ConsoleFunction( setDebugRender, void, 2, 2, "")
//{
//  bool dr = dAtoi(argv[1]);
//  if (physManagerCommon::getPM()->getType() == PHYS_NX)
//  ((nxPhysManager *)physManagerCommon::getPM())->setDebugRender(dr);
//  return;
//}
//
//ConsoleFunction( setDR, void, 1, 1, "")
//{
//  if (physManagerCommon::getPM()->getType() == PHYS_NX)
//  ((nxPhysManager *)physManagerCommon::getPM())->setDebugRender(1);
//  return;
//}
//
//ConsoleFunction( clearDR, void, 1, 1, "")
//{
//  if (physManagerCommon::getPM()->getType() == PHYS_NX)
//  ((nxPhysManager *)physManagerCommon::getPM())->setDebugRender(0);
//  return;
//}
//
//ConsoleFunction( setVISJointLimits, void, 2, 2, "bool")
//{
//  bool value = dAtoi(argv[1]);
//  if (physManagerCommon::getPM()->getType() == PHYS_NX)
//  ((nxPhysManager *)physManagerCommon::getPM())->mPhysicsSDK->setParameter(NX_VISUALIZE_JOINT_LIMITS,value);
//  return;
//}
//ConsoleFunction( setVISJointAxes, void, 2, 2, "bool")
//{
//  bool value = dAtoi(argv[1]);
//  if (physManagerCommon::getPM()->getType() == PHYS_NX)
//  ((nxPhysManager *)physManagerCommon::getPM())->mPhysicsSDK->setParameter(NX_VISUALIZE_JOINT_LOCAL_AXES,value);
//  return;
//}
//ConsoleFunction( setVISBodyAxes, void, 2, 2, "bool")
//{
//  bool value = dAtoi(argv[1]);
//  if (physManagerCommon::getPM()->getType() == PHYS_NX)
//  ((nxPhysManager *)physManagerCommon::getPM())->mPhysicsSDK->setParameter(NX_VISUALIZE_BODY_AXES,value);
//  return;
//}
//ConsoleFunction( setVISCollisionShapes, void, 2, 2, "bool")
//{
//  bool value = dAtoi(argv[1]);
//  if (physManagerCommon::getPM()->getType() == PHYS_NX)
//  ((nxPhysManager *)physManagerCommon::getPM())->mPhysicsSDK->setParameter(NX_VISUALIZE_COLLISION_SHAPES,value);
//  return;
//}
///*  
//  mPhysicsSDK->setParameter(NX_VISUALIZE_COLLISION_SHAPES,1);
//  //mPhysicsSDK->setParameter(NX_VISUALIZE_WORLD_AXES,1);
//  mPhysicsSDK->setParameter(NX_VISUALIZATION_SCALE, 1.0f);
//  //mPhysicsSDK->setParameter(NX_VISUALIZE_JOINT_LIMITS,1);
//  //mPhysicsSDK->setParameter(NX_VISUALIZE_JOINT_LOCAL_AXES,1);
//  //mPhysicsSDK->setParameter(NX_VISUALIZE_ACTOR_AXES,1);
//  //mPhysicsSDK->setParameter(NX_VISUALIZE_BODY_AXES,1);
//*/
//
//ConsoleFunction( getDebugRender, S32, 1, 1, "")
//{
//  return ((nxPhysManager *)physManagerCommon::getPM())->getDebugRender();
//}
//
//ConsoleFunction( initClientContainerRadiusSearch, void, 4, 4, "(Point3F pos, float radius, bitset mask)"
//				"Start a search for items within radius of pos, filtering by bitset mask.")
//{
//	F32 x, y, z;
//	dSscanf(argv[1], "%g %g %g", &x, &y, &z);
//	F32 r = dAtof(argv[2]);
//	U32 mask = dAtoi(argv[3]);
//
//	gClientContainer.initRadiusSearch(Point3F(x, y, z), r, mask);
//}
//
//ConsoleFunction( clientContainerSearchNext, S32, 1, 1, "Get next item from a search started with initContainerRadiusSearch.")
//{
//	return gClientContainer.containerSearchNext();
//}
//
//ConsoleFunction( clientContainerSearchCurrDist, F32, 1, 1, "Get distance of the center of the current item from the center of the current initContainerRadiusSearch.")
//{
//	return gClientContainer.containerSearchCurrDist();
//}
//
//ConsoleFunction( clientContainerSearchCurrRadiusDist, F32, 1, 1, "Get the distance of the closest point of the current item from the center of the current initContainerRadiusSearch.")
//{
//	return gClientContainer.containerSearchCurrRadiusDist();
//}

