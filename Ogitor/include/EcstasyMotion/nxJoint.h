////////////////////////////////////////////////////////////////////////////////////////////////////
//  nxJoint
//  Chris Calef
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _NXJOINT_H_
#define _NXJOINT_H_

#include "EcstasyMotion/nxPhysManager.h"
#include "EcstasyMotion/physJointCommon.h"

class physRigidBody;


class nxJoint : public physJointCommon
{
 public:

  nxPhysManager *mPM;
  NxJoint        *mJoint;

  nxJoint();
  ~nxJoint();

  void setup();
  void onWorldStep();
  void setMotorVelocity(float);
  Ogre::Quaternion& getMotorTarget();
  void setMotorTarget(Ogre::Quaternion &);
  void setLastTarget(Ogre::Quaternion &);
  void setNewTarget(Ogre::Quaternion &);
  void setMotorSpring(Ogre::Quaternion &,float);
  void clearMotor();

};
#endif 
