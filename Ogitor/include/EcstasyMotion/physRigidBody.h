////////////////////////////////////////////////////////////////////////////////////////////////////
//  physRigidBody
//  Chris Calef 2006
//
// adapted from LRGRigidBody.h
//	by Yossi Horowitz
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _PHYSRIGIDBODY_H_
#define _PHYSRIGIDBODY_H_

#include "EcstasyMotion/physManager.h"

#define NUM_SHAPE_TYPES 6

//static EnumTable::Enums gShapeEnums[] =
//  {
//    {PHYS_SHAPE_BOX, "SHAPE_BOX"},
//    {PHYS_SHAPE_CAPSULE, "SHAPE_CAPSULE"},
//    {PHYS_SHAPE_SPHERE, "SHAPE_SPHERE"},
//    {PHYS_SHAPE_CONVEX, "SHAPE_CONVEX"},
//    {PHYS_SHAPE_COLLISION, "SHAPE_COLLISION"},
//    {PHYS_SHAPE_TRIMESH, "SHAPE_TRIMESH"}
//  };
//static EnumTable gShapeTypeTable(NUM_SHAPE_TYPES, &gShapeEnums[0]);
//DefineEnumType( physShapeType );


class physRigidBody
{
 public:
 
  physRigidBody() {};
  virtual ~physRigidBody() {};
  
  virtual void	onWorldStep()=0;

  virtual void setup()=0;
  
  virtual void setPM(physManager *pm)=0;

  virtual iPhysUser *getPhysUser()=0;
  virtual void setPhysUser(iPhysUser *)=0;

  virtual physJoint *getJoint()=0;
  virtual void setJoint(physJoint *)=0;

  virtual physShapeType getShapeType()=0;
  virtual void setShapeType(physShapeType)=0;

  virtual physEntityType getEntityType()=0;
  virtual void setEntityType(physEntityType)=0;

  virtual physShapeType getTriggerShapeType()=0;
  virtual void setTriggerShapeType(physShapeType)=0;

  virtual int getActorGroup()=0;
  virtual void setActorGroup(int)=0;

  virtual int getNumMeshes()=0;
  virtual void setNumMeshes(int)=0;

  virtual int getStartMesh()=0;
  virtual void setStartMesh(int)=0;

  virtual int getBodyVertLookup(int)=0;
  virtual void setBodyVertLookup(int,int)=0;

  virtual int getNodeIndex()=0;
  virtual void setNodeIndex(int)=0;

  virtual Ogre::Quaternion& getDefaultQuat()=0;
  virtual void setDefaultQuat(Ogre::Quaternion&)=0;

  virtual void updatePositionFromActor()=0;
  virtual void updateVelocityFromActor()=0;
  virtual void updatePositionToActor()=0;
  virtual void updateVelocityToActor()=0;
  virtual void addForcesToActor()=0;
  virtual void addForceAtPos(const Ogre::Vector3&,const Ogre::Vector3&)=0;

  virtual void setDynamicFriction(float f)=0;
  virtual void setStaticFriction(float f)=0;
  virtual void setRestitution(float f)=0;
  virtual void setDensity(float f)=0;
  virtual float getDensity()=0;

  virtual Ogre::Vector3& getLinearPosition()=0;
  virtual void	setLinearPosition(const Ogre::Vector3&)=0;
  virtual void	setCurrLinearPosition(const Ogre::Vector3&)=0;
  virtual void	setLastLinearPosition(const Ogre::Vector3&)=0;
  
  virtual Ogre::Quaternion& getAngularPosition()=0;
  virtual void	setAngularPosition(const Ogre::Quaternion&)=0;
  virtual void  setAngularPositionMatrix(Ogre::Matrix3&)=0;
  virtual void	setCurrAngularPosition(const Ogre::Quaternion&)=0;
  virtual void	setLastAngularPosition(const Ogre::Quaternion&)=0;
  
  virtual Ogre::Vector3& getLinearVelocity()=0;
  virtual void	setLinearVelocity(const Ogre::Vector3&)=0;
  virtual Ogre::Vector3& getAngularVelocity()=0;
  virtual void	setAngularVelocity(const Ogre::Vector3&)=0;
  
  virtual void	setCurrForce(const Ogre::Vector3&)=0;
  virtual void	setCurrTorque(const Ogre::Vector3&)=0;
  virtual void	setGlobalForce(const Ogre::Vector3&)=0;
  virtual void	setGlobalDelayForce(const Ogre::Vector3&)=0;
  virtual void	setGlobalTorque(const Ogre::Vector3&)=0;
  virtual void	setMaxTorque(const Ogre::Vector3&)=0;
  virtual void setDelayStep(int)=0;

  virtual float getMass()=0;
  virtual void setMass(float fMass)=0;
  
  virtual Ogre::Vector3& getMassCenter()=0;
  virtual void setMassCenter(Ogre::Vector3&)=0;
  virtual void setObjToWorld(Ogre::Matrix4&)=0;
  virtual Ogre::Matrix4& getObjToWorld()=0;

  virtual void setOffset(Ogre::Vector3 &)=0;
  virtual void setOrientation(Ogre::Vector3 &)=0;
  virtual void setDimensions(Ogre::Vector3 &)=0;
  virtual void setScale(Ogre::Vector3 &)=0;

  virtual void setTriggerOffset(Ogre::Vector3 &)=0;
  virtual void setTriggerOrientation(Ogre::Vector3 &)=0;
  virtual void setTriggerDimensions(Ogre::Vector3 &)=0;
  virtual void setProjectileAxis(Ogre::Vector3 &)=0;

  virtual bool getIsKinematic()=0;
  virtual void setKinematic(bool b=true)=0;
  virtual bool getIsNoGravity()=0;
  virtual void setNoGravity(bool b=true)=0;
  virtual bool getIsProjectile()=0;
  virtual void setProjectile(bool b=true)=0;
  virtual bool getIsInflictor()=0;
  virtual void setInflictor(bool b=true)=0;
  virtual bool getHW()=0;
  virtual void setHW(bool b=true)=0;
  virtual bool getImpulse()=0;
  virtual void setImpulse(bool b=true)=0;

};
#endif 
