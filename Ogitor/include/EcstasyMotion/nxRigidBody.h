////////////////////////////////////////////////////////////////////////////////////////////////////
//  nxRigidBody
//  Chris Calef
//
// adapted from nxRigidBody.h
//	by Yossi Horowitz
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _NXRIGIDBODY_H_
#define _NXRIGIDBODY_H_

#include "EcstasyMotion/nxPhysManager.h"
#include "EcstasyMotion/physRigidBodyCommon.h"



class nxRigidBody : public physRigidBodyCommon
{
 public:

	 nxPhysManager *mPM;
	 NxActor *mActor;
	 NxActor *mTriggerActor;//Putting these in out here in Nx world because I hope I
	 NxJoint *mTriggerJoint;//don't need them anywhere else, or here for much longer.
	 Ogre::Vector3 mTriggerActorOffset;
	 float mMass;

	 nxRigidBody();
	 ~nxRigidBody();
 
	 void setPM(physManager *pm);
	 void onWorldStep();
	 void setup();

	 //physJoint *getJoint();
	 //void setJoint(physJoint *);

	 void updatePositionFromActor();
	 void updateVelocityFromActor();
	 void updatePositionToActor();
	 void updateVelocityToActor();
	 void addForcesToActor();
	 void addForceAtPos(const Ogre::Vector3&,const Ogre::Vector3&);
	 void setTriggerActorPos(Ogre::Vector3 &pos);
	 void setTriggerActorRot(Ogre::Quaternion &q);
	 void setTriggerJointMotorTarget(Ogre::Quaternion &q);
	 void setTriggerJointSpringTarget(Ogre::Quaternion &q);

};
#endif 
