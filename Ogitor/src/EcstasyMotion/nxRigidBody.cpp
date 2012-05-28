////////////////////////////////////////////////////////////////////////////////////////////////////
//  nxRigidBody.cc
//  Chris Calef
//
//  adapted from nxRigidBody.cc
//  by Yossi Horowitz
//
//  An object in the scene to which rigid body physics can be applied.
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "EcstasyMotion/nxRigidBody.h"
#include "EcstasyMotion/fxRigidBody.h"
#include "EcstasyMotion/myStream.h" 


#include "OgitorsScriptConsole.h"
extern Ogitors::OgitorsScriptConsole *gConsole;
extern SQLiteObject *gSQL;

//class fxRigidBody;

struct physVertSort;
extern void vertSorter(physVertSort *vs,int numVS);


nxRigidBody::nxRigidBody()
{

	mPM              = NULL;
	//mJoint           = NULL;
	mActor           = NULL;
	mTriggerActor    = NULL;
	//mTriggerJoint  = NULL;
	mPhysUser        = NULL;

	mTriggerActorOffset = Ogre::Vector3::ZERO;

	for (unsigned int i=0;i<MAX_MESHES_PER_RB;i++) {
		mBodyVertLookups[i] = -1;
	}

	mLinearPosition = Ogre::Vector3::ZERO;
	mCurrLinearPosition = Ogre::Vector3::ZERO;
	mLastLinearPosition = Ogre::Vector3::ZERO;
	mAngularPosition = Ogre::Quaternion::IDENTITY;
	mCurrAngularPosition = Ogre::Quaternion::IDENTITY;
	mLastAngularPosition = Ogre::Quaternion::IDENTITY;
	mLinearVelocity = Ogre::Vector3::ZERO;
	mAngularVelocity = Ogre::Vector3::ZERO;
	mBodyMass		= 1.0f;
	mMassCenter = Ogre::Vector3::ZERO;

	mOffset = Ogre::Vector3::ZERO;
	mOrientation = Ogre::Vector3::ZERO;
	mDimensions = Ogre::Vector3::ZERO;

	mTriggerOffset = Ogre::Vector3::ZERO;
	mTriggerOrientation = Ogre::Vector3::ZERO;
	mTriggerDimensions = Ogre::Vector3::ZERO;
	mTriggerActorOffset = Ogre::Vector3::ZERO;

	mProjectileAxis = Ogre::Vector3(0,0,1);

	mDynamicFriction = 0.5;
	mStaticFriction = 0.5;
	mRestitution = 0.5;
	mDensity = 1.0;
	mInitialObjToWorld = Ogre::Matrix4::IDENTITY;
	//mInitialObjToWorld.identity();
	mDefault = Ogre::Quaternion::IDENTITY;

	mCurrForce = Ogre::Vector3::ZERO;
	mCurrTorque = Ogre::Vector3::ZERO;
	mGlobalForce = Ogre::Vector3::ZERO;
	mGlobalDelayForce = Ogre::Vector3::ZERO;
	mGlobalTorque = Ogre::Vector3::ZERO;
	mMaxTorque = Ogre::Vector3::ZERO;
	mDelayStep = 0;

	mStartMesh = -1;
	mNumMeshes = 0;
	mActorGroup = -1;
	mNodeIndex = -1;

	mIsKinematic = false;
	mIsNoGravity = false;
	mIsProjectile = false;
	mIsInflictor = false;
	mHW           = false;
	mBodyMass = 1.0;

	
	mSetup = false;//Flag to tell us when we're ready to start doing physics.
	mRemove = false;//Flag to tell us we're removing ourselves, so stop doing physics.

	mLastNumVerts = 0;

	mVerts.clear();
	//mIndices.clear();

}

nxRigidBody::~nxRigidBody()
{
	//mRemove = true;//NO, you need to do this BEFORE you call the destructor, cuz the nxRigidBody won't 
	//exist at all after this point.  Having mRemove set is what CALLS this destructor.
   //mPM->removeRigidBody(this); 
}

void nxRigidBody::setPM(physManager *pm)
{
	mPM = (nxPhysManager *)pm;
}


////FIX: should put this into a new file physRigidBody.cpp or PhysRigidBodyCommon.cpp... right?
//ImplementEnumType( physShapeType,
//   "Physics primitive shape types\n" )
//   //"@ingroup ShapeEnums\n\n")//Maybe group is optional?
//	{ PHYS_SHAPE_BOX,     "SHAPE_BOX", "Box"  },
//	{ PHYS_SHAPE_CAPSULE,    "SHAPE_CAPSULE", "Capsule" },
//	{ PHYS_SHAPE_SPHERE,      "SHAPE_SPHERE",  "Sphere" },
//	{ PHYS_SHAPE_CONVEX,      "SHAPE_CONVEX",  "Convex Mesh" },
//	{ PHYS_SHAPE_COLLISION,      "SHAPE_COLLISION",  "DTS Collision Mesh (Convex)" },
//	{ PHYS_SHAPE_TRIMESH,      "SHAPE_TRIMESH",  "Triangle Mesh" },
//EndImplementEnumType;

void nxRigidBody::onWorldStep()
{   

	if (0) return;//here, check for mFlexBody->mIsPhysActive (however I get that info)
	//could just set local mIsPhysActive properties for all the rigid bodies whenever you switch the flexbody on or off.

	//mActor->userData = (void *)mPhysUser;//WTF? Not getting the one I want, trying to make sure here.
	//if (mTriggerActor) mTriggerActor->userData = (void *)mPhysUser;

  //PROFILE_START(nxRigidBody_onWorldStep);
   //if (mIsKinematic != mActor->readBodyFlag(NX_BF_KINEMATIC)) {
   //   if (mIsKinematic) mActor->raiseBodyFlag(NX_BF_KINEMATIC);
   //   else mActor->clearBodyFlag(NX_BF_KINEMATIC);
   //}
	
   //if (mIsNoGravity != mActor->readBodyFlag(NX_BF_DISABLE_GRAVITY)) {
   //   if (mIsNoGravity) mActor->raiseBodyFlag(NX_BF_DISABLE_GRAVITY);
   //   else mActor->clearBodyFlag(NX_BF_DISABLE_GRAVITY);
   //}

   if (mIsKinematic != mActor->readBodyFlag(NX_BF_KINEMATIC)) {
	   //for (unsigned int i=0;i<mNumActors;i++) {
	   if (mIsKinematic) {
		   mActor->raiseBodyFlag(NX_BF_KINEMATIC);//mActors[i]
		   //mActor->raiseActorFlag(NX_AF_DISABLE_COLLISION);
	   }
      else 
	  {
		  mActor->clearBodyFlag(NX_BF_KINEMATIC);//mActors[i]
		  updateVelocityToActor();
		  //mActor->clearActorFlag(NX_AF_DISABLE_COLLISION);
	  }
   }
	
   if (mIsNoGravity != mActor->readBodyFlag(NX_BF_DISABLE_GRAVITY)) {
	   //for (unsigned int i=0;i<mNumActors;i++) {
      if (mIsNoGravity) mActor->raiseBodyFlag(NX_BF_DISABLE_GRAVITY);//mActors[i]
      else mActor->clearBodyFlag(NX_BF_DISABLE_GRAVITY);//mActors[i]
		//}
   }

   //PROFILE_START(nxRigidBody_onWorldStep_addForcesToActor);
   addForcesToActor();//HERE: add mCurrForce and mCurrTorque to actor
   //PROFILE_END();

   mCurrForce = Ogre::Vector3::ZERO;
   mGlobalForce = Ogre::Vector3::ZERO;
   mCurrTorque = Ogre::Vector3::ZERO;
   mGlobalTorque = Ogre::Vector3::ZERO;

   //PROFILE_START(nxRigidBody_onWorldStep_PhysUser);
	if (mPhysUser) 
	{
	   mPhysUser->onWorldStep();
	}
   //else if (mImpulse) 
   //{
   //   mImpulse = false;
   //   addForcesToActor();
   //} //oops
   else {
	   updatePositionFromActor();
	   updateVelocityFromActor();
   }
   //PROFILE_END();

   if (mGlobalDelayForce.length())
   {
	   if (mDelayStep < mPM->mCurrStep)
	   {
		   //Con::errorf("adding delay force! %f %f %f",mGlobalDelayForce.x,mGlobalDelayForce.y,mGlobalDelayForce.z);
			mGlobalForce = mGlobalDelayForce;
			mGlobalDelayForce = Ogre::Vector3::ZERO;
			mDelayStep = 0;
	   }
   }
   //PROFILE_END();
}


void nxRigidBody::addForcesToActor()
{
   if (mActor) 
   {
	   //if (mCurrForce.length()>0) mActor->addLocalForce(NxVec3(mCurrForce.x,mCurrForce.y,mCurrForce.z));
	   //if (mCurrTorque.length()>0) mActor->addLocalTorque(NxVec3(mCurrTorque.x,mCurrTorque.y,mCurrTorque.z));
	   //if (mGlobalForce.length()>0) mActor->addForce(NxVec3(mCurrForce.x,mCurrForce.y,mCurrForce.z));
	   //if (mGlobalTorque.length()>0) mActor->addTorque(NxVec3(mCurrTorque.x,mCurrTorque.y,mCurrTorque.z));if (mCurrForce.length()>0) mActor->addLocalForce(NxVec3(mCurrForce.x,mCurrForce.y,mCurrForce.z));
	   if ((mGlobalForce.length()>0)||(mGlobalTorque.length()>0)) 
	   {
		   mPhysUser->onCollision(NULL);
	   }
	   mActor->addForce(NxVec3(mGlobalForce.x,mGlobalForce.y,mGlobalForce.z));
	   //if (mGlobalForce.length()>0) Con::warnf("added global force: %f %f %f",mGlobalForce.x,mGlobalForce.y,mGlobalForce.z);
	   mActor->addTorque(NxVec3(mGlobalTorque.x,mGlobalTorque.y,mGlobalTorque.z));
	   mActor->addLocalForce(NxVec3(mCurrForce.x,mCurrForce.y,mCurrForce.z));
	   mActor->addLocalTorque(NxVec3(mCurrTorque.x,mCurrTorque.y,mCurrTorque.z));
	   mCurrForce = Ogre::Vector3::ZERO;
	   mGlobalForce = Ogre::Vector3::ZERO;
   }
}

void nxRigidBody::addForceAtPos(const Ogre::Vector3& kForce,const Ogre::Vector3& kPos)
{
	mActor->addForceAtPos(NxVec3(kForce.x,kForce.y,kForce.z),NxVec3(kPos.x,kPos.y,kPos.z));
}

void nxRigidBody::updatePositionFromActor()
{
	if (mActor) 
	{
		NxVec3 nxPos = mActor->getGlobalPosition();

		Ogre::Vector3 newPos(nxPos.x,nxPos.y,nxPos.z);

		//mLastLinearPosition = mCurrLinearPosition;
		mLinearPosition = Ogre::Vector3(nxPos.x,nxPos.y,nxPos.z);
		//mCurrLinearPosition = mLinearPosition;

		NxQuat q;
		NxF32 qvalues[4];

		q = mActor->getGlobalOrientationQuat();
		q.getWXYZ(qvalues);

		//mLastAngularPosition = mCurrAngularPosition;
		mAngularPosition.w	= qvalues[0];
		mAngularPosition.x	= qvalues[1];
		mAngularPosition.y	= qvalues[2];
		mAngularPosition.z	= qvalues[3];
		//mCurrAngularPosition = mAngularPosition;
	}
}

void nxRigidBody::updateVelocityFromActor()
{
   if (mActor) 
	{	 
	   NxVec3 nxVel = mActor->getLinearVelocity();
	   mLinearVelocity = Ogre::Vector3(nxVel.x,nxVel.y,nxVel.z);

	   nxVel = mActor->getAngularVelocity();
	   mAngularVelocity = Ogre::Vector3(nxVel.x,nxVel.y,nxVel.z);    
   }
}

void nxRigidBody::updatePositionToActor()
{//FIX: figure out whether to keep mCurrPos or mLinearPosition, NOT BOTH
	if (mActor) {
		std::ostringstream os;
		//os << " physUser type: " << mPhysUser->mEntityType;
		//gConsole->addOutput(os.str());
		NxVec3 nxPos(mLinearPosition.x,mLinearPosition.y,mLinearPosition.z);
		//mActor->setGlobalPosition(nxPos);
		mActor->moveGlobalPosition(nxPos);
		//mActor->moveGlobalPosition(nxPos);
		if (mTriggerActor)
		{
			NxVec3 triggerAdj(mTriggerActorOffset.x,mTriggerActorOffset.y,mTriggerActorOffset.z);
			//WAIT - first rotate trigger actor offset by rigid body transform.  For now just try zero.
			//NxVec3 triggerAdj(0,0,0);
			mActor->getGlobalOrientationQuat().rotate(triggerAdj);	
			mTriggerActor->setGlobalPosition(nxPos + triggerAdj);
			mTriggerActor->setGlobalOrientationQuat(mActor->getGlobalOrientationQuat());
			//mTriggerActor->moveGlobalOrientationQuat(mActor->getGlobalOrientationQuat());


		}


		NxQuat q;
		//if (mPhysUser
		q.setXYZW(mAngularPosition.x,mAngularPosition.y,mAngularPosition.z,-mAngularPosition.w);
		NxMat33 nxMat(q);
		//mActor->setGlobalOrientation(nxMat);
		mActor->moveGlobalOrientation(nxMat);
		//mActor->moveGlobalOrientation(nxMat);
	}
}

void nxRigidBody::updateVelocityToActor()
{
   if (mActor) 
	{
	   NxVec3 nxVel; 
	   nxVel.set(mLinearVelocity.x,mLinearVelocity.y,mLinearVelocity.z);
	   mActor->setLinearVelocity(nxVel);
	   if (mTriggerActor) mTriggerActor->setLinearVelocity(nxVel);

	   nxVel.set(mAngularVelocity.x,mAngularVelocity.y,mAngularVelocity.z);
	   mActor->setAngularVelocity(nxVel);
	   if (mTriggerActor) mTriggerActor->setAngularVelocity(nxVel);
   }
}

void nxRigidBody::setTriggerActorPos(Ogre::Vector3 &pos)
{
	if (mTriggerActor)
		mTriggerActor->setGlobalPosition(NxVec3(pos.x,pos.y,pos.z));
	return;	
}

void nxRigidBody::setTriggerActorRot(Ogre::Quaternion &quat)
{      
	
	if (mTriggerActor)
	{
		NxQuat q;
		q.setXYZW(quat.x,quat.y,quat.z,-quat.w);
		NxMat33 nxMat(q);
		mTriggerActor->setGlobalOrientation(nxMat);
	}
	return;
}
/*
void nxRigidBody::setTriggerJointMotorTarget(Ogre::Quaternion &q)
{	//nxRigidBody *me = this;
	//mTriggerJoint->setMotorTarget(quat);//FIX 1000.0 is spring force, expose it.
	//Whoops, we're using straight NxJoints here, not my nxJoint class... stick with it for now.
	if (mTriggerJoint)
	{
		if (mTriggerJoint->isD6Joint()) {
			NxD6JointDesc d6Desc;
			((NxD6Joint *)mTriggerJoint)->saveToDesc(d6Desc);

			NxActor *actorA,*actorB;
			mTriggerJoint->getActors(&actorA,&actorB);
			NxMat34 matA = actorA->getGlobalPose();
			NxMat34 matB = actorB->getGlobalPose();
			NxQuat qA(matA.M); qA.invert();
			NxQuat qB(matB.M); qB.invert();
			//NxVec3 normalA = qA.rot(NxVec3(0,0,1));
			//NxVec3 normalB = qB.rot(NxVec3(0,0,1));

			//d6Desc.setGlobalAxis(NxVec3(0,1,0));
			//d6Desc.localNormal[0] = normalA;
			//d6Desc.localNormal[1] = normalB;


			NxQuat nxq;
			nxq.setXYZW(q.x,q.y,q.z,-q.w);
			//nxq.fromAngleAxis(90,NxVec3(0,1,0));
			d6Desc.driveOrientation = qB * nxq; 
			d6Desc.flags |= NX_D6JOINT_SLERP_DRIVE;
			d6Desc.twistMotion = NX_D6JOINT_MOTION_FREE;
			d6Desc.swing1Motion = NX_D6JOINT_MOTION_FREE;
			d6Desc.swing2Motion = NX_D6JOINT_MOTION_FREE;
			d6Desc.swingDrive.spring = 10000;
			d6Desc.twistDrive.spring = 10000;
			d6Desc.slerpDrive.spring = 10000;
			d6Desc.slerpDrive.driveType = NX_D6JOINT_DRIVE_POSITION;
			//d6Desc.slerpDrive.forceLimit = FLT_MAX;
			//Con::printf("setting weapon trigger motor target!");

			((NxD6Joint *)mTriggerJoint)->loadFromDesc(d6Desc);
		}
	}
	return;
}

void nxRigidBody::setTriggerJointSpringTarget(Ogre::Quaternion &quat)
{	
	//if (mTriggerJoint)
	//   mTriggerJoint->setMotorSpring(quat,1000.0);
	return;
}
*/
void nxRigidBody::setup()
{
	if (!mPM->getPhysicsSDK())
		return;

	std::ostringstream os;

	NxPhysicsSDK *kPhysicsSDK = mPM->getPhysicsSDK();
	NxScene *kScene = NULL;
	NxScene *kHWScene = NULL;

	kScene = mPM->getScene();
	kHWScene = mPM->getHWScene();

	//physShapeData *kUserData;

	if (!kScene)
	{
		gConsole->addOutput("nxRigidBody couldn't find scene.");
		return;
	}

	if (!mActor) 
	{
		NxActorDesc actorDesc;
		NxBodyDesc bodyDesc;
		NxBoxShapeDesc boxDesc,boxDescTrigger;
		NxSphereShapeDesc sphereDesc,sphereDescTrigger;
		NxCapsuleShapeDesc capsDesc,capsDescTrigger;
		Ogre::Vector3 offset,orient,dim;
		NxVec3 kVerts[10000];
		int va,vb,vc;
		va = 0; vb = 0; vc = 0;

		/////////////////////////////////////////////////////////////////////////////////
		if (mShapeType==PHYS_SHAPE_BOX) {
			boxDesc.dimensions.set(mDimensions.x/2.0,mDimensions.y/2.0,mDimensions.z/2.0);
			boxDesc.localPose.t = NxVec3(mOffset.x,mOffset.y,mOffset.z);
			boxDesc.materialIndex = 0;
			//boxDesc.shapeFlags |= NX_TRIGGER_ENABLE;
			actorDesc.shapes.pushBack(&boxDesc);
			/////////////////////////////////////////////////////////////////////////////////
		} else if (mShapeType==PHYS_SHAPE_SPHERE) { 
			sphereDesc.radius = mDimensions.x;
			sphereDesc.localPose.t = NxVec3(mOffset.x,mOffset.y,mOffset.z);
			sphereDesc.materialIndex = 0;
			//sphereDesc.shapeFlags |= NX_TRIGGER_ENABLE;
			actorDesc.shapes.pushBack(&sphereDesc);
			/////////////////////////////////////////////////////////////////////////////////
		} else if (mShapeType==PHYS_SHAPE_CAPSULE) {
			capsDesc.radius = mDimensions.x;
			capsDesc.height = mDimensions.z;//HERE: if height==0, then figure out how far to the next bodypart.
			capsDesc.localPose.t = NxVec3(mOffset.x,mOffset.y,mOffset.z);
			capsDesc.materialIndex = 0;
			//capsDesc.shapeFlags |= NX_TRIGGER_ENABLE;
			actorDesc.shapes.pushBack(&capsDesc);
			/////////////////////////////////////////////////////////////////////////////////
		} else if (mShapeType==PHYS_SHAPE_CONVEX) { 

			//NxInitCooking();

			// Cooking from memory
			//MyMemoryWriteBuffer buf;
			//if(CookConvexMesh(convexDesc, buf)){
			//}

			//NxCloseCooking();

			if (mPM->mMeshes[mStartMesh]) {
				NxConvexShapeDesc convexShapeDesc;
				convexShapeDesc.localPose.t = NxVec3(mOffset.x,mOffset.y,mOffset.z);//NxVec3(0,0,0);
				//if (mPhysUser->mEntityType == PHYS_RIGID_BODY)
				//{
				//	fxRigidBody *fxRB = dynamic_cast<fxRigidBody*>(mPhysUser);
				//	Ogre::String myName = fxRB->mEntity->getName();
				//	Ogre::Matrix3 mat;
				//	fxRB->mNode->getOrientation().ToRotationMatrix(mat);
				//	Ogre::Vector3 myPos = fxRB->mNode->getPosition();

				//	
				//	NxF32 M44[16];
				//	Ogre::Vector3 vec0,vec1,vec2;
				//	vec0 = mat.GetColumn(0); 
				//	vec1 = mat.GetColumn(1);
				//	vec2 = mat.GetColumn(2);

				//	//convexShapeDesc.localPose.M.setColumn(0,NxVec3(vec0.x,vec0.y,vec0.z));
				//	//convexShapeDesc.localPose.M.setColumn(1,NxVec3(vec1.x,vec1.y,vec1.z));
				//	//convexShapeDesc.localPose.M.setColumn(2,NxVec3(vec2.x,vec2.y,vec2.z));

				//	M44[0] = vec0.x; M44[4] = vec0.y; M44[8] = vec0.z; M44[12] = 0.0; 
				//	M44[1] = vec1.x; M44[5] = vec1.y; M44[9] = vec1.z; M44[13] = 0.0; 
				//	M44[2] = vec2.x; M44[6] = vec2.y; M44[10] = vec2.z; M44[14] = 0.0; 
				//	M44[3] =    0.0; M44[7] =    0.0; M44[11] =    0.0; M44[15] = 1.0; 
				//	convexShapeDesc.localPose.setColumnMajor44(M44);

				//	os.str("");
				//	os << "nxRigidBody convex " << myName.c_str() << ", Matrix: " ;	 
				//	gConsole->addOutput(os.str());
				//	
				//	os.str("");
				//	os << M44[0] << " " << M44[1] << " " << M44[2] << " " << M44[3] ;
				//	gConsole->addOutput(os.str());					
				//	os.str("");
				//	os << M44[4] << " " << M44[5] << " " << M44[6] << " " << M44[7] ;
				//	gConsole->addOutput(os.str());
				//	os.str("");
				//	os << M44[8] << " " << M44[9] << " " << M44[10] << " " << M44[11] ;
				//	gConsole->addOutput(os.str());
				//	os.str("");
				//	os << M44[12] << " " << M44[13] << " " << M44[14] << " " << M44[15] ;
				//	gConsole->addOutput(os.str());

				//	Ogre::Quaternion q = fxRB->mNode->getOrientation();

				//	//os.str("");
				//	//os << "nxRigidBody trying to set orientation... " << myName.c_str() << " position " <<
				//	//	myPos.x << ", " << myPos.y << ", " << myPos.z << 
				//	//	",    -- quat " << q.w << ", " << 
				//	//	q.x << ", " << q.y << ", " << q.z ;
				//	//gConsole->addOutput(os.str());
				//}
				convexShapeDesc.meshData = mPM->mMeshes[mStartMesh];
				actorDesc.shapes.pushBack(&convexShapeDesc);
			} else {
				Ogre::Vector3 avgVert;  
				avgVert = Ogre::Vector3::ZERO;
				for (unsigned int k=0;k<mVerts.size();k++) 
				{
					Ogre::Vector3 vertex = mVerts[k];
					kVerts[vc++] = NxVec3(vertex.x,vertex.y,vertex.z);
					avgVert += vertex;
				}
				avgVert /= mVerts.size();
				//mOffset = avgVert;
				NxConvexMeshDesc convexDesc;
				convexDesc.numVertices = vc;
				convexDesc.pointStrideBytes = sizeof(NxVec3);
				convexDesc.points = kVerts;
				convexDesc.flags = NX_CF_COMPUTE_CONVEX;// | NX_TRIGGER_ENABLE;
				NxConvexShapeDesc convexShapeDesc;
				convexShapeDesc.localPose.t = NxVec3(mOffset.x,mOffset.y,mOffset.z);//NxVec3(0,0,0);

				//if (mPhysUser->mEntityType == PHYS_RIGID_BODY)
				//{
				//	fxRigidBody *fxRB = dynamic_cast<fxRigidBody*>(mPhysUser);
				//	Ogre::String myName = fxRB->mEntity->getName();
				//	Ogre::Matrix3 mat;
				//	fxRB->mNode->getOrientation().ToRotationMatrix(mat);
				//	Ogre::Vector3 myPos = fxRB->mNode->getPosition();

				//	NxF32 M44[16];
				//	Ogre::Vector3 vec0,vec1,vec2;
				//	vec0 = mat.GetColumn(0); 
				//	vec1 = mat.GetColumn(1);
				//	vec2 = mat.GetColumn(2);

				//	//convexShapeDesc.localPose.M.setColumn(0,NxVec3(vec0.x,vec0.y,vec0.z));
				//	//convexShapeDesc.localPose.M.setColumn(1,NxVec3(vec1.x,vec1.y,vec1.z));
				//	//convexShapeDesc.localPose.M.setColumn(2,NxVec3(vec2.x,vec2.y,vec2.z));

				//	M44[0] = vec0.x; M44[4] = vec0.y; M44[8] = vec0.z; M44[12] = 0.0; 
				//	M44[1] = vec1.x; M44[5] = vec1.y; M44[9] = vec1.z; M44[13] = 0.0; 
				//	M44[2] = vec2.x; M44[6] = vec2.y; M44[10] = vec2.z; M44[14] = 0.0; 
				//	M44[3] =    0.0; M44[7] =    0.0; M44[11] =    0.0; M44[15] = 1.0; 
				//	convexShapeDesc.localPose.setColumnMajor44(M44);

				//	os.str("");
				//	os << "nxRigidBody convex " << myName.c_str() << ", Matrix: " ;	 
				//	gConsole->addOutput(os.str());
				//	
				//	os.str("");
				//	os << M44[0] << " " << M44[1] << " " << M44[2] << " " << M44[3] ;
				//	gConsole->addOutput(os.str());					
				//	os.str("");
				//	os << M44[4] << " " << M44[5] << " " << M44[6] << " " << M44[7] ;
				//	gConsole->addOutput(os.str());
				//	os.str("");
				//	os << M44[8] << " " << M44[9] << " " << M44[10] << " " << M44[11] ;
				//	gConsole->addOutput(os.str());
				//	os.str("");
				//	os << M44[12] << " " << M44[13] << " " << M44[14] << " " << M44[15] ;
				//	gConsole->addOutput(os.str());

				//	Ogre::Quaternion q = fxRB->mNode->getOrientation();

				//	//os.str("");
				//	//os << "nxRigidBody trying to set orientation... " << myName.c_str() << " position " <<
				//	//	myPos.x << ", " << myPos.y << ", " << myPos.z << 
				//	//	",    -- quat " << q.w << ", " << 
				//	//	q.x << ", " << q.y << ", " << q.z ;
				//	//gConsole->addOutput(os.str());
				//}


				myUserStream my_stream("shape.bin", false);
				bool status = NxCookConvexMesh(convexDesc, myUserStream("shape.bin",false));// myUserStream("shape.bin", false));
				if (status) {
					mPM->mMeshes[mStartMesh] = kPhysicsSDK->createConvexMesh(myUserStream("shape.bin", true)); 
					convexShapeDesc.meshData = mPM->mMeshes[mStartMesh];
					actorDesc.shapes.pushBack(&convexShapeDesc);
				//	//Con::errorf("created convex hull: vertices %d",vc);
				} //else Con::errorf("FAILED to create convex hull ");
			}
			//mPhysUser->setupDebugRender();	//  Con::errorf("FAILED to create convex hull ");
		} else if (mShapeType==PHYS_SHAPE_COLLISION) {
			//NxConvexMeshDesc convexDescs[MAX_MESHES_PER_RB];
			NxConvexShapeDesc convexShapeDescs[MAX_MESHES_PER_RB];
			//Con::errorf("making  rigid body collision: numMeshes %d startMesh %d",mNumMeshes,mStartMesh);
			for (unsigned int i=0;i<mNumMeshes;i++) {
				vc = 0;
				NxVec3 kVerts[10000];//redeclaring this to make it local, new every time.
				if (mPM->mMeshes[mStartMesh+i]) {
					//NxConvexShapeDesc convexShapeDesc;
					convexShapeDescs[i].localPose.t = NxVec3(mOffset.x,mOffset.y,mOffset.z);//NxVec3(0,0,0);
					convexShapeDescs[i].meshData = mPM->mMeshes[mStartMesh+i];
					actorDesc.shapes.pushBack(&convexShapeDescs[i]);//
				} else {
					int start,end;
					//Con::errorf("making Mesh %d",i);
					//if (i==0) {
					//	start = 0;
					//	end = mBodyVertLookups[i];
					//} else {
					//start = mBodyVertLookups[i];
					//end = mBodyVertLookups[i+1];
					//}
					if (i<(mNumMeshes-1)) {
						start = mBodyVertLookups[i];
						end = mBodyVertLookups[i+1];
					} else {
						start = mBodyVertLookups[i];
						end = start + mLastNumVerts;
					}
					
					//Con::errorf("start %d end %d",start,end);
					if (end<=start) 
					{
						continue;
					}

					Ogre::Vector3 avgVert;  
					avgVert = Ogre::Vector3::ZERO;
					for (unsigned int k=start;k<end;k++) {
						Ogre::Vector3 vertex = mVerts[k];
						kVerts[vc++] = NxVec3(vertex.x,vertex.y,vertex.z);
						avgVert += vertex;
						//Con::errorf("collision mesh vertex %f %f %f",vertex.x,vertex.y,vertex.z);
					}
					avgVert /= (end-start);
					NxConvexMeshDesc convexDesc;
					convexDesc.numVertices = vc;
					convexDesc.pointStrideBytes = sizeof(NxVec3);
					convexDesc.points = kVerts;
					convexDesc.flags = NX_CF_COMPUTE_CONVEX;// | NX_TRIGGER_ENABLE;
					//NxConvexShapeDesc convexShapeDesc;
					convexShapeDescs[i].localPose.t = NxVec3(mOffset.x,mOffset.y,mOffset.z);//NxVec3(0,0,0);

					//FIX - NxStream broken again.
					myUserStream my_stream("shape.bin", false);
					bool status = NxCookConvexMesh(convexDesc, my_stream);
					if (status) {
						mPM->mMeshes[mStartMesh+i] = kPhysicsSDK->createConvexMesh(myUserStream("shape.bin", true)); 
						convexShapeDescs[i].meshData = mPM->mMeshes[mStartMesh+i];
						actorDesc.shapes.pushBack(&convexShapeDescs[i]);
					////	//Con::errorf("created collision mesh! vertices %d",vc);
					} //else Con::errorf("FAILED to create convex hull ");
				}
			}
			//mPhysUser->setupDebugRender();//FIX!  This will need to be updated as well, for multiple actors.
			//  Con::errorf("FAILED to create convex hull ");
		}


		NxActorDesc triggerActorDesc;
		NxActor *triggerActor;
		NxBodyDesc triggerBodyDesc;
		//NxFixedJointDesc triggerJointDesc;
		NxD6JointDesc triggerJointDesc;
		//NxSphericalJointDesc triggerJointDesc;
		NxJoint *triggerJoint;

		if ((mIsProjectile)&&(!mHW))
		{
			if (mTriggerShapeType==PHYS_SHAPE_CAPSULE) {
				capsDescTrigger.radius = mTriggerDimensions.x;
				capsDescTrigger.height = mTriggerDimensions.z;
				capsDescTrigger.shapeFlags |= NX_TRIGGER_ENABLE;
				actorDesc.shapes.pushBack(&capsDescTrigger); 		    
			} else if (mTriggerShapeType==PHYS_SHAPE_BOX) {
				boxDescTrigger.dimensions.set(mTriggerDimensions.x,mTriggerDimensions.y,mTriggerDimensions.z);
				boxDescTrigger.shapeFlags |= NX_TRIGGER_ENABLE;
				actorDesc.shapes.pushBack(&boxDescTrigger);
			} else if (mTriggerShapeType==PHYS_SHAPE_SPHERE) {
				sphereDescTrigger.radius = mTriggerDimensions.x;
				sphereDescTrigger.shapeFlags |= NX_TRIGGER_ENABLE;
				actorDesc.shapes.pushBack(&sphereDescTrigger);
			}
		} else {
			if ((mIsInflictor)&&(!mHW)) 
				//if ((mTriggerDimensions.length()>0)&&(!mHW))
			{//HERE: if mTriggerShapeType is not defined, we will not create a trigger.
				//And... WHY?  WHY?  WHY?  Seems that there is literally no way at all to make physx
				//recognize kinematic-kinematic trigger collisions, even with triggers on every bodypart
				//and NX_TRIGGER_TRIGGER_CALLBACK turned on.  Just doesn't work, period.
				//So, PITA fallback solution: create a different actor, trigger only, NON-kinematic,
				//and connected to our regular bodypart rigid body by a fixed joint.  ARgh. >:-\
				//Note: see if this works as well as the old way, if not we will need to leave 
				//projectile rigid bodies the way they were, and just do this for flexbodyparts.

				//NxMat33 mat;
				//if (mTriggerOrientation.x) mat.rotX(mTriggerOrientation.x * (NxPi/180.0));
				//else if (mTriggerOrientation.y) mat.rotY(mTriggerOrientation.y * (NxPi/180.0));
				//else if (mTriggerOrientation.z) mat.rotZ(mTriggerOrientation.z * (NxPi/180.0));
				//else mat.id();
					
				//triggerActorDesc.shapes[0]->setLocalOrientation(mat);
				if (mShapeType==PHYS_SHAPE_CAPSULE) {//mTriggerShapeType
					capsDescTrigger.radius = mDimensions.x;//mTriggerDimensions.x;
					capsDescTrigger.height = mDimensions.z;//mTriggerDimensions.z;
					//capsDescTrigger.shapeFlags |= NX_TRIGGER_ENABLE;
					capsDescTrigger.shapeFlags |= NX_SF_DISABLE_RESPONSE;
					capsDescTrigger.localPose.t =  NxVec3(mOffset.x,mOffset.y,mOffset.z);//NxVec3(mTriggerOffset.x,mTriggerOffset.y,mTriggerOffset.z);
					triggerActorDesc.shapes.pushBack(&capsDescTrigger); 		    
				} else if (mShapeType==PHYS_SHAPE_BOX) {//mTriggerShapeType
					//boxDescTrigger.dimensions.set(mTriggerDimensions.x,mTriggerDimensions.y,mTriggerDimensions.z);
					//boxDescTrigger.localPose.t =  NxVec3(mTriggerOffset.x,mTriggerOffset.y,mTriggerOffset.z);
					boxDescTrigger.dimensions.set(mDimensions.x/2.0,mDimensions.y/2.0,mDimensions.z/2.0);
					boxDescTrigger.localPose.t =  NxVec3(mOffset.x,mOffset.y,mOffset.z);
					//boxDescTrigger.shapeFlags |= NX_TRIGGER_ENABLE;
					boxDescTrigger.shapeFlags |= NX_SF_DISABLE_RESPONSE;
					triggerActorDesc.shapes.pushBack(&boxDescTrigger);
				} else if (mShapeType==PHYS_SHAPE_SPHERE) {//mTriggerShapeType
					//sphereDescTrigger.radius = mTriggerDimensions.x;
					//sphereDescTrigger.localPose.t = NxVec3(mTriggerOffset.x,mTriggerOffset.y,mTriggerOffset.z);
					sphereDescTrigger.radius = mDimensions.x;
					sphereDescTrigger.localPose.t = NxVec3(mOffset.x,mOffset.y,mOffset.z);
					//sphereDescTrigger.shapeFlags |= NX_TRIGGER_ENABLE;
					sphereDescTrigger.shapeFlags |= NX_SF_DISABLE_RESPONSE;
					triggerActorDesc.shapes.pushBack(&sphereDescTrigger);
				}
				//HERE: Do it the Ross Way:  make stupidSphereDesc into caps, box or sphere desc, trigger 
				//dimensions, but turn off collision.
				//NxSphereShapeDesc stupidSphereDesc;//Okay, ONE more try... apparently can't get anywhere
				//stupidSphereDesc.radius = 0.02;//by tying a trigger-only actor to a joint, so adding a 
				//triggerActorDesc.shapes.pushBack(&stupidSphereDesc);//trivial actual actor to hold it together.
				triggerActorDesc.density = 1.0;//do I want extra density here?  Should I make it 0.0 or 0.001?
				triggerActorDesc.body = &triggerBodyDesc;
				Ogre::Vector3 finalTriggerPos = mLinearPosition + mTriggerActorOffset;
				triggerActorDesc.globalPose.t = NxVec3(finalTriggerPos.x,finalTriggerPos.y,finalTriggerPos.z);
			}
		}

		//////////////////////////////////////////////////////////////////////////
		// Now, no matter what the type of body -- using the actorDesc you just made, create the actor.
		actorDesc.body = &bodyDesc;
		actorDesc.density = mDensity;
		actorDesc.globalPose.t = NxVec3(mLinearPosition.x,mLinearPosition.y,mLinearPosition.z);
		//actorDesc.globalPose
		

		//if ((kScene)&&((mEntityType==PHYS_FLEX_BODY)||(mIsProjectile))) {

		//if (kScene) {
		if ((!mHW)||(!kHWScene)) 
		{
			NxMat33 mat,mat2;

			if (actorDesc.isValid())
				mActor = kScene->createActor(actorDesc);
			if (!mActor) 
			{
				//Con::errorf("rigid body failed to create actor!!!!!!");
				gConsole->addOutput("rigid body failed to create actor!!!!!!");
				return;
			} 
			NxShape * const * shapes = mActor->getShapes();
			NxU32 nShapes    = mActor->getNbShapes();

			//TRYING to get a trigger that can interact with kinematic bodies...
			if (triggerActorDesc.shapes.size()>0)
			{
				triggerActor =  kScene->createActor(triggerActorDesc);
				triggerActor->clearBodyFlag(NX_BF_KINEMATIC);
				triggerActor->raiseBodyFlag(NX_BF_DISABLE_GRAVITY);
				//Con::errorf("cleared kinematic flag for trigger actor!");
				mTriggerActor = triggerActor;

				triggerJointDesc.actor[0] = mActor;
				triggerJointDesc.actor[1] = mTriggerActor;
				triggerJointDesc.setGlobalAnchor(NxVec3(mLinearPosition.x,mLinearPosition.y,mLinearPosition.z));	
				//triggerJointDesc.setGlobalAxis(NxVec3(0,0,1));
				triggerJointDesc.setGlobalAxis(NxVec3(0,1,0));
				triggerJointDesc.localNormal[0] = NxVec3(0,0,1);
				triggerJointDesc.localNormal[1] = NxVec3(0,0,1);//might have to change if 
				//trigger orientation changes from "0 0 0"
				triggerJointDesc.twistMotion = NX_D6JOINT_MOTION_LOCKED;
				triggerJointDesc.swing1Motion = NX_D6JOINT_MOTION_LOCKED;
				triggerJointDesc.swing2Motion = NX_D6JOINT_MOTION_LOCKED;
				triggerJointDesc.xMotion = NX_D6JOINT_MOTION_LOCKED;
				triggerJointDesc.yMotion = NX_D6JOINT_MOTION_LOCKED;
				triggerJointDesc.zMotion = NX_D6JOINT_MOTION_LOCKED;

				triggerJointDesc.maxForce = 100000;
				triggerJointDesc.maxTorque = 100000;
					
				if (triggerJointDesc.isValid()) {
					triggerJoint = kScene->createJoint(triggerJointDesc);
					mTriggerJoint = triggerJoint;
					//nxRigidBody *me = this;
					//Con::printf("***********created a trigger joint!!*** shapes %d ***** pos %f %f %f ***",
					//	triggerActorDesc.shapes.size(),mLinearPosition.x,mLinearPosition.y,mLinearPosition.z);
				}// else Con::errorf("trigger joint desc is invalid!");

				NxShape * const * tShapes = triggerActor->getShapes();
				NxU32 ntShapes    = triggerActor->getNbShapes();
				
				while (ntShapes--) {
					if (tShapes[ntShapes]->getFlag(NX_TRIGGER_ENABLE)) 
					{
						if (mTriggerOrientation.x) mat.rotX(mTriggerOrientation.x * (NxPi/180.0));
						else if (mTriggerOrientation.y) mat.rotY(mTriggerOrientation.y * (NxPi/180.0));
						else if (mTriggerOrientation.z) mat.rotZ(mTriggerOrientation.z * (NxPi/180.0));
						else mat.id();
						tShapes[ntShapes]->setLocalOrientation(mat);
					} else {
						if (mOrientation.x) mat.rotX(mOrientation.x * (NxPi/180.0));
						else if (mOrientation.y) mat.rotY(mOrientation.y * (NxPi/180.0));
						else if (mOrientation.z) mat.rotZ(mOrientation.z * (NxPi/180.0));
						else mat.id();
						tShapes[ntShapes]->setLocalOrientation(mat);
						//NxVec3 off(mOffset.x,mOffset.y,mOffset.z);
						//tShapes[ntShapes]->setLocalPosition(off);
					}
				}
			}

			mBodyMass = mActor->getMass();
			//char printstring[255];
			//sprintf(printstring,"actor created, mass :  %f ",mBodyMass);
			//gConsole->addOutput(printstring);
			//Con::errorf("actor mass: %f",mBodyMass);
			//kUserData = new physShapeData;
			//kUserData->mEntityType = mEntityType;
			//kUserData->mPhysUser = mPhysUser;
			//mActor->userData = (void *)kUserData;
			//mPhysUser->mEntityType = mEntityType;
			//mActor->userData = (void *)mPhysUser;

			while (nShapes--) {

				if ((offset.length())&&(nShapes>0)) {//Hmm?  what was local "offset" for?  doesn't get used.
					NxVec3 off(mOffset.x,mOffset.y,mOffset.z);//And we already have mOffset applied to desc.localPose.t
					shapes[nShapes]->setLocalPosition(off);
				}

				if (shapes[nShapes]->getFlag(NX_TRIGGER_ENABLE)) 
				{//(This will be involved in 

					if (mTriggerOrientation.x) mat.rotX(mTriggerOrientation.x * (NxPi/180.0));
					else if (mTriggerOrientation.y) mat.rotY(mTriggerOrientation.y * (NxPi/180.0));
					else if (mTriggerOrientation.z) mat.rotZ(mTriggerOrientation.z * (NxPi/180.0));
					else mat.id();
					//Con::printf("setting trigger orientation! %f %f %f",
					//	mTriggerOrientation.x,mTriggerOrientation.y,mTriggerOrientation.z);
					shapes[nShapes]->setLocalOrientation(mat);
					shapes[nShapes]->setLocalPosition(NxVec3(mTriggerOffset.x,mTriggerOffset.y,mTriggerOffset.z));

				} else {//HERE: are we setting barrel orientation?
					//Con::errorf("setting local orientation: %f %f %f",
					//	mOrientation.x,mOrientation.y,mOrientation.z);
					if (mOrientation.x) mat.rotX(mOrientation.x * (NxPi/180.0));
					else if (mOrientation.y) mat.rotY(mOrientation.y * (NxPi/180.0));
					else if (mOrientation.z) mat.rotZ(mOrientation.z * (NxPi/180.0));
					else mat.id();
					shapes[nShapes]->setLocalOrientation(mat);

				}
			}
			
			//NOPE: need to bring in my mega Euler math library anyway - Ogre matrices don't
			//have toEuler, and in Torque it didn't really work anyway.
			//Ogre::Vector3 eul = mInitialObjToWorld. .toEuler();
			//if (eul.length()) {//FIX - only works with first valid axis
			//	if (eul.x) mat2.rotX(-eul.x);
			//	else if (eul.y) mat2.rotY(-eul.y);
			//	else if (eul.z) mat2.rotZ(-eul.z);//NOTE: is this universal?  Works for constructor interiors anyway. CEC 11/13/08

			//} else
			//if ((mLinearVelocity.length())&&(mIsProjectile)) {
			//	Ogre::Quaternion qDir;
			//	qDir.rotationArc(mProjectileAxis,mLinearVelocity);
			//	NxQuat nxq; 
			//	nxq.setXYZW(qDir.x,qDir.y,qDir.z,qDir.w);//NOT -qDir.w, not sure why.
			//	mat2.fromQuat(nxq);

			//} else if (getNodeIndex()>=0) {
			//	Ogre::Quaternion mDef = getDefaultQuat();
			//	NxQuat q;
			//	q.setXYZW(mDef.x,mDef.y,mDef.z,-mDef.w);
			//	NxMat33 nxMat(q);
			//	mat2 = nxMat;
			//} else {
			//	mat2.id();
			//}
			mat2.id();
			NxQuat nxQuat;
			nxQuat.setXYZW(mAngularPosition.x,mAngularPosition.y,mAngularPosition.z,mAngularPosition.w);
			mat2.fromQuat(nxQuat);
			mActor->setGlobalOrientation(mat2);

			//HERE: more complicated logic could allow different groups to not collide.
			//For now, group 0 is the only non-colliding group.
			mActor->setGroup(mActorGroup);
			if (mTriggerActor)
				mTriggerActor->setGroup(mActorGroup);
			//Con::errorf("setting actors to group: %d",mActorGroup);
			for (unsigned int k=1; k < mPM->getCurrentActorGroup(); k++) {
				if (k!=mActorGroup) {
					kScene->setActorGroupPairFlags(k,mActorGroup,NX_NOTIFY_ON_START_TOUCH | 
					NX_NOTIFY_ON_END_TOUCH | NX_NOTIFY_ON_TOUCH);// | NX_NOTIFY_ON_IMPACT );
				}
			}

			if (mIsKinematic) mActor->raiseBodyFlag(NX_BF_KINEMATIC);
			else mActor->setLinearVelocity(NxVec3(mLinearVelocity.x,mLinearVelocity.y,mLinearVelocity.z));

			//Con::errorf("nx actor mass: %f",mActor->getMass());
			//kUserData = new physShapeData;
			//kUserData->mEntityType = mEntityType;
			//kUserData->mPhysUser = mPhysUser;
			//mActor->userData = (void *)kUserData;
			mActor->userData = (void *)mPhysUser;
			if (mTriggerActor) 
				mTriggerActor->userData = (void *)mPhysUser;

		}

		////if ((kHWScene)&&((mEntityType==PHYS_RIGID_BODY)&&(!mIsProjectile))) {

		//if ((mHW)&&(kHWScene)) {
		//	NxMat33 mat,mat2;
		//	//NxActor *actor;
		//	mActor = kHWScene->createActor(actorDesc);
		//	NxShape * const * shapes = mActor->getShapes();
		//	NxU32 nShapes    = mActor->getNbShapes();
		//	while (nShapes--) {

		//		if ((offset.length())&&(nShapes>0)) {
		//			NxVec3 off(mOffset.x,mOffset.y,mOffset.z);
		//			shapes[nShapes]->setLocalPosition(off);
		//		}

		//		if (shapes[nShapes]->getFlag(NX_TRIGGER_ENABLE)) {
		//			if (mTriggerOrientation.x) mat.rotX(mTriggerOrientation.x * (NxPi/180.0));
		//			else if (mTriggerOrientation.y) mat.rotY(mTriggerOrientation.y * (NxPi/180.0));
		//			else if (mTriggerOrientation.z) mat.rotZ(mTriggerOrientation.z * (NxPi/180.0));
		//			else mat.id();
		//			shapes[nShapes]->setLocalOrientation(mat);
		//		} else {
		//			if (mOrientation.x) mat.rotX(mOrientation.x * (NxPi/180.0));
		//			else if (mOrientation.y) mat.rotY(mOrientation.y * (NxPi/180.0));
		//			else if (mOrientation.z) mat.rotZ(mOrientation.z * (NxPi/180.0));
		//			else mat.id();
		//			shapes[nShapes]->setLocalOrientation(mat);
		//		}
		//	}
			//HERE: after we're aligned to match the shape in default position, now we need to 
			//deal with actual orientation (defined by "rotation" (mObjToWorld) Shape member.)

			//EulerF eul = mInitialObjToWorld.toEuler();
			////NOTE: this only has value for nxRigidBodyShape objects.
			//if (eul.length()) {//FIX - only works with first valid axis
			//	if (eul.x) mat2.rotX(eul.x);
			//	else if (eul.y) mat2.rotY(eul.y);
			//	else if (eul.z) mat2.rotZ(eul.z);
			//} else if ((mLinearVelocity.length())&&(mIsProjectile)) {//projectiles
			//	Ogre::Quaternion qDir;
			//	qDir.rotationArc(mProjectileAxis,mLinearVelocity);
			//	NxQuat nxq; 
			//	nxq.setXYZW(qDir.x,qDir.y,qDir.z,qDir.w);
			//	mat2.fromQuat(nxq);
			//} else if (getNodeIndex()>=0) {
			//	Ogre::Quaternion mDef = getDefaultQuat();
			//	NxQuat q;
			//	q.setXYZW(mDef.x,mDef.y,mDef.z,-mDef.w);
			//	NxMat33 nxMat(q);
			//	mat2 = nxMat;
			//} else mat2.id();

			//mActor->setGlobalOrientation(mat2);

			//mActor->setGroup(mActorGroup);
			//triggerActor->setGroup(mActorGroup);
			//for (unsigned int k=1; k < mPM->getCurrentActorGroup(); k++) {
			//	if (k!=mActorGroup) {
			//		kHWScene->setActorGroupPairFlags(k,mActorGroup,NX_NOTIFY_ON_START_TOUCH//);// | 
			//		//NX_NOTIFY_ON_END_TOUCH | NX_NOTIFY_ON_TOUCH | NX_NOTIFY_ON_IMPACT );
			//		//Con::errorf("set up collisions between group %d and group %d",k,mActorGroup);
			//	}
			//}
		//	if (mIsKinematic) mActor->raiseBodyFlag(NX_BF_KINEMATIC);
		//	else mActor->setLinearVelocity(NxVec3(mLinearVelocity.x,mLinearVelocity.y,mLinearVelocity.z));


		//	//mActor->userData = this;		   
		//	//physShapeData *kUserData = new physShapeData;
		//	//kUserData->mEntityType = mEntityType;
		//	//kUserData->mPhysUser = mPhysUser;
		//	//mActor->userData = (void *)kUserData;
		//	mActor->userData = (void *)mPhysUser;

		//}
	}
}

