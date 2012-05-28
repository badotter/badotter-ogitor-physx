////////////////////////////////////////////////////////////////////////////////////////////////////
//  nxJoint.cc
//  Chris Calef
//
// 
//  NX_JOINT_PRISMATIC        // Permits a single translational degree of freedom.
//  NX_JOINT_REVOLUTE         // Also known as a hinge joint, permits one rotational degree of freedom.
//  NX_JOINT_CYLINDRICAL      // Formerly known as a sliding joint, permits one translational and one rotational degree of freedom.
//  NX_JOINT_SPHERICAL        // Also known as a ball or ball and socket joint.
//  NX_JOINT_POINT_ON_LINE    // A point on one actor is constrained to stay on a line on another.
//  NX_JOINT_POINT_IN_PLANE   // A point on one actor is constrained to stay on a plane on another.
//  NX_JOINT_DISTANCE         // A point on one actor maintains a certain distance range to another point on another actor.
//  NX_JOINT_PULLEY           // A pulley joint.
//  NX_JOINT_FIXED            // A "fixed" connection.
//  NX_JOINT_D6               // A 6 degree of freedom joint
////////////////////////////////////////////////////////////////////////////////////////////////////
//#include "core/stl_fix.h"

//#include "platform/platform.h"
//#include "math/mathio.h"
//#include "console/consoleTypes.h"
//#include "collision/ConcretePolyList.h"
//#include "core/stream/bitStream.h"
//#include "ts/tsShapeInstance.h"

//#include "T3D/shapeBase.h"
#include "EcstasyMotion/nxJoint.h"
#include "EcstasyMotion/nxRigidBody.h"

#include "OgitorsScriptConsole.h"
extern Ogitors::OgitorsScriptConsole *gConsole;
extern SQLiteObject *gSQL;

nxJoint::nxJoint()
{
	mPM = NULL;
	//mJD = NULL;

	mJoint = NULL;
	mRB_A = NULL;
	mRB_B = NULL;

	mAxisA = Ogre::Vector3::ZERO;
	mAxisB = Ogre::Vector3::ZERO;
	mNormalA = Ogre::Vector3::ZERO;
	mNormalB = Ogre::Vector3::ZERO;

	mLocalAnchor0 = Ogre::Vector3::ZERO;
	mLocalAnchor1 = Ogre::Vector3::ZERO;
	mLocalAxis0 = Ogre::Vector3::ZERO;
	mLocalAxis1 = Ogre::Vector3::ZERO;
	mLocalNormal0 = Ogre::Vector3::ZERO;
	mLocalNormal1 = Ogre::Vector3::ZERO;

	mGlobalAnchor = Ogre::Vector3::ZERO;
	mGlobalAxis = Ogre::Vector3::ZERO;

	mSwingSpring = 0.0;
	mTwistSpring = 0.0;
	mJointSpring = 0.0;

	mHW = false;

    mMotorTarget = Ogre::Quaternion::IDENTITY;
    mLastTarget = Ogre::Quaternion::IDENTITY;
    mNewTarget = Ogre::Quaternion::IDENTITY;

}

nxJoint::~nxJoint()
{
 
}

void nxJoint::onWorldStep()
{
  //nothing to do
}

Ogre::Quaternion& nxJoint::getMotorTarget()
{
	Ogre::Quaternion q;
	q = Ogre::Quaternion::IDENTITY;
	if (mJoint) {
		if (mJoint->isD6Joint()) {
			NxD6JointDesc d6Desc;
			((NxD6Joint *)mJoint)->saveToDesc(d6Desc);

			NxQuat nxq;
			nxq = d6Desc.driveOrientation;

			if (fabs(nxq.x)<10.0) {
				q.x = nxq.x; q.y = nxq.y; q.z = nxq.z; q.w = -nxq.w;  
			} else q = Ogre::Quaternion::IDENTITY;
		}
	}
	mMotorTarget = q;
	return q;
}

void nxJoint::setMotorTarget(Ogre::Quaternion &q)
{
	NxQuat nxq;


	if (mJoint) {
		if (mJoint->isD6Joint()) {
			NxD6JointDesc d6Desc;
			((NxD6Joint *)mJoint)->saveToDesc(d6Desc);

			//HERE: need to switch on axis.  Need to figure out general solution based on axis rotation.
			if (mLocalAxis1 == Ogre::Vector3(0,0,1))
			{//SPINE
				mMotorTarget = Ogre::Quaternion(-q.z,-q.x,-q.y,q.w);
			}
			else if (mLocalAxis1 == Ogre::Vector3(-1,0,0))
			{//LEFT ARM
				mMotorTarget = Ogre::Quaternion(q.x,q.z,q.y,q.w);
			}
			else if (mLocalAxis1 == Ogre::Vector3(1,0,0))
			{//RIGHT ARM
				mMotorTarget = Ogre::Quaternion(-q.x,-q.z,q.y,q.w);
			}
			else if (mLocalAxis1 == Ogre::Vector3(0,0,-1))
			{//LEGS
				mMotorTarget = Ogre::Quaternion(q.z,q.x,-q.y,q.w);
			} 
			else 
			{//(other)
				mMotorTarget = Ogre::Quaternion(-q.z,-q.x,-q.y,q.w);
			}
			nxq.setXYZW(mMotorTarget.x,mMotorTarget.y,mMotorTarget.z,mMotorTarget.w);

			d6Desc.driveOrientation = nxq; 
			d6Desc.flags = NX_D6JOINT_SLERP_DRIVE;
			d6Desc.swingDrive.spring = mSwingSpring;
			//d6Desc.twistDrive.spring = mTwistSpring;
			d6Desc.slerpDrive.spring = mSwingSpring;	
			d6Desc.slerpDrive.damping = mSpringDamper;
			d6Desc.slerpDrive.driveType = NX_D6JOINT_DRIVE_POSITION;//NX_D6JOINT_SLERP_DRIVE
			//d6Desc.slerpDrive.driveType = NX_D6JOINT_SLERP_DRIVE;
			d6Desc.slerpDrive.forceLimit = FLT_MAX;

			((NxD6Joint *)mJoint)->loadFromDesc(d6Desc);
		}//HERE: support spherical, revolute.
	}
}
// Here... sample of the hell I almost got stuck in here.  
// Good example of how to use Eul_FromHMatrix() though.
//HMatrix M;
//EulerAngles ea;
//Ogre::Vector3 row0,row1,row2;
//mat.getRow(0,&row0); mat.getRow(1,&row1); mat.getRow(2,&row2);
//M[0][0] = row0.x; M[1][0] = row0.y; M[2][0] = row0.z; M[3][0] = 0.0;
//M[0][1] = row1.x; M[1][1] = row1.y; M[2][1] = row1.z; M[3][1] = 0.0;
//M[0][2] = row2.x; M[1][2] = row2.y; M[2][2] = row2.z; M[3][2] = 0.0;
//M[0][3] = 0.0; M[1][3] = 0.0; M[2][3] = 0.0; M[3][3] = 1.0;
//ea = Eul_FromHMatrix( M,EulOrdXYZs);
//eul2.x = ea.z;
//eul2.y = ea.x;
//eul2.z = ea.y;
//Ogre::Quaternion q2(eul2);

//Ogre::Quaternion qX(EulerF(mDegToRad(20.0),0.0,0.0));
//Ogre::Quaternion qY(EulerF(0.0,mDegToRad(20.0),0.0));
//Ogre::Quaternion qZ(EulerF(0.0,0.0,mDegToRad(-20.0)));


void nxJoint::setMotorSpring(Ogre::Quaternion &q,float f)
{
   //get Desc from joint
   //set new target
   //reload joint from Desc

   mMotorTarget = q;

   if (mJoint) {
	   if (mJoint->isD6Joint()) {
		   NxD6JointDesc desc;
		   ((NxD6Joint *)mJoint)->saveToDesc(desc);

		   NxQuat nxq;
		   nxq.setXYZW(q.x,q.y,q.z,-q.w);
		   //nxq.fromAngleAxis(90,NxVec3(0,1,0));
		   desc.driveOrientation = nxq; 
		   //d6Desc.flags |= NX_D6JOINT_SLERP_DRIVE;
		   if (f==0.0) f = 1.0;
		   desc.swingDrive.spring = mSwingSpring * f;
		   desc.twistDrive.spring = mTwistSpring * f;
		   //d6Desc.slerpDrive.spring = mSpringForce;
		   //jointDesc.slerpDrive.driveType = NX_D6JOINT_DRIVE_POSITION;
		   //jointDesc.slerpDrive.forceLimit = FLT_MAX;

		   ((NxD6Joint *)mJoint)->loadFromDesc(desc);
		   //Con::errorf("nxJoint set motor spring: %f %f %f %f",q.x,q.y,q.z,q.w);
	   } else if (mJoint->isSphericalJoint()) {
		   NxSphericalJointDesc desc;
		   ((NxSphericalJoint *)mJoint)->saveToDesc(desc);

		   NxQuat nxq;
		   nxq.setXYZW(q.x,q.y,q.z,-q.w);

		   //desc.twistSpring.targetValue = ??;
		   //desc.driveOrientation = nxq; 

		   if (f==0.0) f = 1.0;
		   desc.swingSpring.spring = mSwingSpring * f;
		   //desc.swingDrive.spring = mSwingSpring * f;
		   desc.twistSpring.spring = mTwistSpring * f;
		   //desc.twistDrive.spring = mTwistSpring * f;

		   ((NxSphericalJoint *)mJoint)->loadFromDesc(desc);
		   //Con::errorf("nxJoint set motor spring: %f %f %f %f",q.x,q.y,q.z,q.w);
	   } else if (mJoint->isRevoluteJoint()) {
		   NxRevoluteJointDesc desc;
		   ((NxRevoluteJoint *)mJoint)->saveToDesc(desc);

		   NxQuat nxq;
		   nxq.setXYZW(q.x,q.y,q.z,-q.w);

		   //desc.spring.targetValue = ??;
		   //desc.driveOrientation = nxq; 

		   if (f==0.0) f = 1.0;
		   desc.spring = mSwingSpring * f;
		   //desc.swingDrive.spring = mSwingSpring * f;
		   //desc.twistDrive.spring = mTwistSpring * f;

		   ((NxRevoluteJoint *)mJoint)->loadFromDesc(desc);
		   //Con::errorf("nxJoint set motor spring: %f %f %f %f",q.x,q.y,q.z,q.w);

	   }
   }
}

void nxJoint::setLastTarget(Ogre::Quaternion &q)
{
	mLastTarget = q;
}

void nxJoint::setNewTarget(Ogre::Quaternion &q)
{
	mNewTarget = q;
}

void nxJoint::clearMotor()
{
	//get Desc from joint
	//set new target
	//reload joint from Desc

	if (mJoint)
	{
		if (mJoint->isD6Joint()) 
		{
			NxD6JointDesc d6Desc;
			((NxD6Joint *)mJoint)->saveToDesc(d6Desc);

			NxQuat nxq;
			nxq.id();

			//d6Desc.flags |= NX_D6JOINT_SLERP_DRIVE;
			d6Desc.driveOrientation = nxq; 
			d6Desc.slerpDrive.spring = 0;
			d6Desc.swingDrive.spring = 0;
			d6Desc.twistDrive.spring = 0;
			((NxD6Joint *)mJoint)->loadFromDesc(d6Desc);

		}
	}
}

void nxJoint::setMotorVelocity(float vel)
{
   if (mJointType==PHYS_JOINT_REVOLUTE) {
      NxRevoluteJointDesc revDesc;
      NxRevoluteJoint *rev = (NxRevoluteJoint*)mJoint;
      rev->saveToDesc(revDesc);
      revDesc.motor.velTarget = vel;
      rev->loadFromDesc(revDesc);
   }//else if (mJointType==PHYS_JOINT_SPHERICAL) {..}

}

void nxJoint::setup()
{
	Ogre::Vector3 p,globalAnchor,parentAxis,parentNormal,globalAxis,globalNormal,globalPos; 
	//NxVec3 globalAnchor,childAnchor,parentAnchor,parentAxis,parentNormal;
	Ogre::Quaternion qA,qB,mDef,mDefI,childQuat;
	//Quat16 q16;
	Ogre::Matrix3 mA,mB,childMatrix;

	NxScene *kScene = NULL;
	NxScene *kHWScene = NULL;

	kScene = ((nxPhysManager *)mPM)->getScene();
	kHWScene = ((nxPhysManager *)mPM)->getHWScene();


	//FIX:  use setGlobalAnchor, ditch mAnchorA and B.
	//Shortcut: if mAnchorA is left empty, grab the point between the two bodies.
	//if ((mLocalAnchor0.length()==0)||(!mLocalAnchor0)) {
	//   if ((mRB_A)&&(mRB_B)) {
	//      p = (mRB_A->getLinearPosition() + mRB_B->getLinearPosition()) / 2;
	//   } else if (mRB_A) {
	//      p = mRB_A->getLinearPosition();
	//   } else if (mRB_B) {
	//      p = mRB_B->getLinearPosition();
	//   }
	//mLocalAnchor0 = p - mRB_A->getLinearPosition();
	//   mLocalAnchor1 = p - mRB_B->getLinearPosition();
	//} //else assume it's all good.


	//verify that my rigid bodies are HW if I am HW.
	if (mHW) {
		if (((mRB_A)&&(!mRB_A->getHW()))||((mRB_B)&&(!mRB_B->getHW())))
			mHW = false;
	}

	//First, normalize (normalise) everything that should be normaliz(s)ed.
	if (mGlobalAxis.length()>0) mGlobalAxis.normalise();
	if (mLocalAxis0.length()>0) mLocalAxis0.normalise();
	if (mLocalAxis1.length()>0) mLocalAxis1.normalise();
	if (mLocalNormal0.length()>0) mLocalNormal0.normalise();
	if (mLocalNormal1.length()>0) mLocalNormal1.normalise();
	if (mAxisA.length()>0) mAxisA.normalise();
	if (mAxisB.length()>0) mAxisB.normalise();
	if (mNormalA.length()>0) mNormalA.normalise();
	if (mNormalB.length()>0) mNormalB.normalise();


	//HERE: fix for matrix functions, grab toEuler from the mondo-Euler math library.
	//qA = mRB_A->getObjToWorld();  qA.setMatrix(&mA);  EulerF eulA = mA.toEuler();
	//qB = mRB_B->getObjToWorld();  qB.setMatrix(&mB);  EulerF eulB = mB.toEuler();
	//childMatrix = mA.inverse() * mB; //Multiply child matrix by inverse of parent matrix, to get local matrix.
	//EulerF eulC = childMatrix.toEuler();

	//Con::errorf("eulA %3.2f %3.2f %3.2f, eulB %3.2f %3.2f %3.2f, eulC %3.2f %3.2f %3.2f",
	//	 eulA.x,eulA.y,eulA.z,eulB.x,eulB.y,eulB.z,eulC.x,eulC.y,eulC.z);
	//mB.inverse();

	std::ostringstream strResult;
	//strResult << "Making a joint, type " << mJointType ;
	//gConsole->addOutput(strResult.str());


	/////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////
	if (mJointType == PHYS_JOINT_D6) {
		NxD6JointDesc jointDesc;
		if (mRB_A) jointDesc.actor[0] = ((nxRigidBody *)mRB_A)->mActor;
		if (mRB_B) jointDesc.actor[1] = ((nxRigidBody *)mRB_B)->mActor;

		if (mGlobalAnchor.length()>0)
		{
			jointDesc.setGlobalAnchor(NxVec3(mGlobalAnchor.x,mGlobalAnchor.y,mGlobalAnchor.z));
		} else if ((mLocalAnchor0.length()>0)||(mLocalAnchor1.length()>0)) {//HERE: implement localAnchor1!
			jointDesc.localAnchor[0] = NxVec3(mLocalAnchor0.x,mLocalAnchor0.y,mLocalAnchor0.z);
			jointDesc.localAnchor[1] = NxVec3(mLocalAnchor1.x,mLocalAnchor1.y,mLocalAnchor1.z);
		} else {
			globalPos = mRB_B->getLinearPosition();
			jointDesc.setGlobalAnchor(NxVec3(globalPos.x,globalPos.y,globalPos.z));	
		}

		if (mGlobalAxis.length()>0) {
			jointDesc.setGlobalAxis(NxVec3(mGlobalAxis.x,mGlobalAxis.y,mGlobalAxis.z));
		} else if ((mLocalAxis0.length()>0)&&(mLocalAxis1.length()>0)) {
			jointDesc.localAxis[0] = NxVec3(mLocalAxis0.x,mLocalAxis0.y,mLocalAxis0.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		} else if ((mLocalAxis0.length()==0.0)&&(mLocalAxis1.length()>0.0)) { //we have only defined local axis 1, child bodypart axis, so derive parent axis from it.
			parentAxis = childMatrix * mLocalAxis1;//(??) (Does this work?)

			//HERE: fix for Ogre matrix functions
			// mB.mulP(mLocalAxis1,&parentAxis);
			jointDesc.localAxis[0] = NxVec3(parentAxis.x,parentAxis.y,parentAxis.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		}

		if ((mLocalNormal0.length()>0)&&(mLocalNormal1.length()>0)) {
			jointDesc.localNormal[0] = NxVec3(mLocalNormal0.x,mLocalNormal0.y,mLocalNormal0.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		} else if ((mLocalNormal0.length()==0)&&(mLocalNormal1.length()>0)) {//we have only defined local normal 1, child bodypart normal, so derive parent normal from it.
			//HERE: fix for Ogre matrix functions
			parentNormal =  childMatrix * mLocalNormal1;
			// //mB.mulP(mLocalNormal1,&parentNormal);
			// jointDesc.localNormal[0] = NxVec3(parentNormal.x,parentNormal.y,parentNormal.z);
			// jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		}
		//End ridiculous repeated section
		////////////////////////////////////////////////////////////////

		if (mCollisionEnabled) jointDesc.jointFlags |= NX_JF_COLLISION_ENABLED;

		if (mMaxForce)  jointDesc.maxForce = mMaxForce;
		if (mMaxTorque) jointDesc.maxTorque = mMaxTorque;


		if (mTwistLimit) { 
			if (mTwistLimit < 0.02) {
				jointDesc.twistMotion = NX_D6JOINT_MOTION_LOCKED;
			} else {
				jointDesc.twistMotion = NX_D6JOINT_MOTION_LIMITED;
				jointDesc.twistLimit.low.value =  -mTwistLimit * (NxPi/180.0);
				jointDesc.twistLimit.high.value =  mTwistLimit * (NxPi/180.0);
				//jointDesc.twistLimit.low.damping = 0.05f;
				//jointDesc.twistLimit.high.damping = 0.05f;
			}
		} else {
			jointDesc.twistMotion = NX_D6JOINT_MOTION_FREE;
		}

		if (mSwingLimit) { 
			if (mSwingLimit < 0.02) {
				jointDesc.swing1Motion = NX_D6JOINT_MOTION_LOCKED;
			} else {
				jointDesc.swing1Motion = NX_D6JOINT_MOTION_LIMITED;
				jointDesc.swing1Limit.value = mSwingLimit * (NxPi/180.0);
				//jointDesc.swing1Limit.damping = 0.05f;
			}
		} else {
			jointDesc.swing1Motion = NX_D6JOINT_MOTION_FREE;
		}


		if (mSwingLimit2) { 
			if (mSwingLimit2 < 0.02) {
				jointDesc.swing2Motion = NX_D6JOINT_MOTION_LOCKED;
			} else {
				jointDesc.swing2Motion = NX_D6JOINT_MOTION_LIMITED;
				jointDesc.swing2Limit.value = mSwingLimit2 * (NxPi/180.0);
				//jointDesc.swing2Limit.damping = 0.05f;
			}
		} else {
			jointDesc.swing2Motion = NX_D6JOINT_MOTION_FREE;
		}



		//jointDesc.flags |= NX_SJF_JOINT_SPRING_ENABLED;
		//jointDesc.flags |= NX_SJF_SWING_SPRING_ENABLED;
		//jointDesc.flags |= NX_SJF_TWIST_SPRING_ENABLED;

		//jointDesc.jointSpring.spring = mJointSpring;
		//jointDesc.swingSpring.spring = mSwingSpring;
		//jointDesc.twistSpring.spring = mTwistSpring;
		//jointDesc.swingSpring.damper = mSpringDamper;
		//jointDesc.swingSpring.targetValue = mSpringTargetAngle;

		//jointDesc.twistMotion = NX_D6JOINT_MOTION_LOCKED;
		//jointDesc.swing1Motion = NX_D6JOINT_MOTION_LOCKED;
		//jointDesc.swing2Motion = NX_D6JOINT_MOTION_LOCKED;

		jointDesc.xMotion = NX_D6JOINT_MOTION_LOCKED;
		jointDesc.yMotion = NX_D6JOINT_MOTION_LOCKED;
		jointDesc.zMotion = NX_D6JOINT_MOTION_LOCKED;

		//jointDesc.projectionMode = NX_JPM_NONE;
		jointDesc.projectionMode = NX_JPM_POINT_MINDIST;
		jointDesc.projectionDistance = 0.1f;
		jointDesc.projectionAngle = 0.0872f;


		if (1) {
			//jointDesc.jointFlags |= NX_D6JOINT_SLERP_DRIVE;
			jointDesc.flags |= NX_D6JOINT_SLERP_DRIVE;//out of date docs?
			//jointDesc.driveMode = NX_D6_SLERP_MODE;//out of date docs?
			//jointDesc.slerpDrive.driveType = NX_D6JOINT_DRIVE_POSITION;
			jointDesc.slerpDrive.forceLimit = FLT_MAX;
			jointDesc.slerpDrive.spring = 0.0f;//HERE: if this has value, it will "motorize" all the time.
			jointDesc.slerpDrive.damping = 0.02f;//Have to set it when we need it.
			jointDesc.slerpDrive.driveType.raiseFlagMask(NX_D6JOINT_SLERP_DRIVE);
		} else {
			//   jointDesc.swingDrive.driveType = NX_D6JOINT_DRIVE_POSITION;
			//   jointDesc.swingDrive.forceLimit = FLT_MAX;
			//   jointDesc.swingDrive.spring = 0;
			//   jointDesc.swingDrive.damping = 0;
			//   jointDesc.twistDrive.driveType = NX_D6JOINT_DRIVE_POSITION;
			//   jointDesc.twistDrive.forceLimit = FLT_MAX;
			//   jointDesc.twistDrive.spring = 0;
			//   jointDesc.twistDrive.damping = 0;
		}

		NxQuat q;
		////q.fromAngleAxis(90,NxVec3(0,0,1));
		////q.fromAngleAxis(180,NxVec3(0,0,1));
		////jointDesc.driveAngularVelocity = NxVec3(0,3,0);

		//q.fromAngleAxis(90,NxVec3(1,0,0));
		//jointDesc.driveOrientation = q; 


		if (jointDesc.isValid()) {
			if ((mHW)&&(kHWScene)) mJoint = kHWScene->createJoint(jointDesc);
			else mJoint = kScene->createJoint(jointDesc);
		} //else Con::printf("D6 joint desc is invalid! %f",mSwingLimit2);

		
	/////////////////////////////////////////////////////////////////////////////////////////////////
	} else if (mJointType == PHYS_JOINT_SPHERICAL) {

		NxSphericalJointDesc jointDesc;

		if (mRB_A) jointDesc.actor[0] = ((nxRigidBody *)mRB_A)->mActor;
		if (mRB_B) jointDesc.actor[1] = ((nxRigidBody *)mRB_B)->mActor;

		if (!jointDesc.actor[0])
			gConsole->addOutput("Body A has no actor!");
		if (!jointDesc.actor[1])
			gConsole->addOutput("Body B has no actor!");

		if (mGlobalAnchor.length()>0)
		{
			jointDesc.setGlobalAnchor(NxVec3(mGlobalAnchor.x,mGlobalAnchor.y,mGlobalAnchor.z));
		} else if ((mLocalAnchor0.length()>0)||(mLocalAnchor1.length()>0)) {//HERE: implement localAnchor1!
			jointDesc.localAnchor[0] = NxVec3(mLocalAnchor0.x,mLocalAnchor0.y,mLocalAnchor0.z);
			jointDesc.localAnchor[1] = NxVec3(mLocalAnchor1.x,mLocalAnchor1.y,mLocalAnchor1.z);
		} else {
			globalPos = mRB_B->getLinearPosition();
			jointDesc.setGlobalAnchor(NxVec3(globalPos.x,globalPos.y,globalPos.z));	
		}

		if (mGlobalAxis.length()>0) {
			jointDesc.setGlobalAxis(NxVec3(mGlobalAxis.x,mGlobalAxis.y,mGlobalAxis.z));
		} else if ((mLocalAxis0.length()>0)&&(mLocalAxis1.length()>0)) {
			jointDesc.localAxis[0] = NxVec3(mLocalAxis0.x,mLocalAxis0.y,mLocalAxis0.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		} else if ((mLocalAxis0.length()==0.0)&&(mLocalAxis1.length()>0.0)) { //we have only defined local axis 1, child bodypart axis, so derive parent axis from it.
			parentAxis = childMatrix * mLocalAxis1;
			jointDesc.localAxis[0] = NxVec3(parentAxis.x,parentAxis.y,parentAxis.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		} 

		if ((mLocalNormal0.length()>0)&&(mLocalNormal1.length()>0)) {
			jointDesc.localNormal[0] = NxVec3(mLocalNormal0.x,mLocalNormal0.y,mLocalNormal0.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		} else if ((mLocalNormal0.length()==0)&&(mLocalNormal1.length()>0)) {//we have only defined local normal 1, child bodypart normal, so derive parent normal from it.
			parentNormal = childMatrix * mLocalNormal1;
			jointDesc.localNormal[0] = NxVec3(parentNormal.x,parentNormal.y,parentNormal.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		}
		//End ridiculous repeated section
		////////////////////////////////////////////////////////////////

		jointDesc.projectionMode = NX_JPM_POINT_MINDIST;
		//if (mCollisionEnabled) jointDesc.jointFlags |= NX_JF_COLLISION_ENABLED;

		if (mTwistLimit) 
		{ 
			jointDesc.flags |= NX_SJF_TWIST_LIMIT_ENABLED;
			jointDesc.twistLimit.low.value =  -mTwistLimit * (NxPi/180.0);
			jointDesc.twistLimit.high.value =  mTwistLimit * (NxPi/180.0);
		}

		if (mSwingLimit) 
		{
			jointDesc.flags |= NX_SJF_SWING_LIMIT_ENABLED;
			jointDesc.swingLimit.value = mSwingLimit * (NxPi/180.0);
			jointDesc.swingLimit.hardness = 1;//0.5;//FIX!! expose to script/DB
			jointDesc.swingLimit.restitution = 0;//0.5;
		}

		//if (mTwistSpring) {
		//jointDesc.flags |= NX_SJF_TWIST_SPRING_ENABLED;
		//jointDesc.twistLimit.value = mTwistSpring;
		//}

		if (mSwingSpring) 
		{
			jointDesc.flags |= NX_SJF_SWING_SPRING_ENABLED;
			jointDesc.swingSpring.spring = mSwingSpring;
			jointDesc.swingSpring.damper = mSpringDamper;
			jointDesc.swingSpring.targetValue = mSpringTargetAngle;

			//NxSpringDesc springDesc;
			//springDesc.spring = (NxReal)mSwingSpring;
			//springDesc.damper = (NxReal)mSpringDamper;
			//springDesc.targetValue = (NxReal)mSpringTargetAngle * (NxPi/180.0);
			//jointDesc.jointSpring = springDesc;
			//jointDesc.flags |= NX_SJF_JOINT_SPRING_ENABLED;
			//jointDesc.swingSpring = springDesc;
			//jointDesc.flags |= NX_SJF_SWING_SPRING_ENABLED;
		}

		if (mJointSpring) 
		{
			jointDesc.flags |= NX_SJF_JOINT_SPRING_ENABLED;
			jointDesc.jointSpring.spring = mJointSpring;
			jointDesc.jointSpring.damper = mSpringDamper;
		}
		if (mTwistSpring) 
		{
			jointDesc.flags |= NX_SJF_TWIST_SPRING_ENABLED;
			jointDesc.twistSpring.spring = mTwistSpring;
			jointDesc.twistSpring.damper = mSpringDamper;
		}

		jointDesc.maxForce = mMaxForce;// ;
		jointDesc.maxTorque = mMaxTorque;//;

		//   if (1) {
		//   jointDesc.swingDrive.driveType = NX_D6JOINT_DRIVE_POSITION;//Spherical (swing) motor drive?
		//   jointDesc.swingDrive.forceLimit = FLT_MAX;
		//   jointDesc.swingDrive.spring = 0;
		//   jointDesc.swingDrive.damping = 0;
		//   jointDesc.twistDrive.driveType = NX_D6JOINT_DRIVE_POSITION;
		//   jointDesc.twistDrive.forceLimit = FLT_MAX;
		//   jointDesc.twistDrive.spring = 0;
		//   jointDesc.twistDrive.damping = 0;
		//}

		//strResult.str("");
		//strResult << "local axis 1 " << jointDesc.localAxis[1].x << ", " << jointDesc.localAxis[1].y << ", " <<
		//	jointDesc.localAxis[1].z << ", " << "  local normal 1 " <<  jointDesc.localNormal[1].x << ", " <<
		//	jointDesc.localNormal[1].y << ", " << jointDesc.localNormal[1].z ;
		//gConsole->addOutput(strResult.str());

		if (jointDesc.isValid()) { 
			if ((mHW)&&(kHWScene)) mJoint = kHWScene->createJoint(jointDesc);
			else mJoint = kScene->createJoint(jointDesc);
		} else gConsole->addOutput("spherical joint desc is invalid!");

	
	/////////////////////////////////////////////////////////////////////////////////////////////////
	} else if (mJointType == PHYS_JOINT_REVOLUTE) {

		NxRevoluteJointDesc jointDesc;

		if (mRB_A) jointDesc.actor[0] = ((nxRigidBody *)mRB_A)->mActor;
		if (mRB_B) jointDesc.actor[1] = ((nxRigidBody *)mRB_B)->mActor;

		if (mGlobalAnchor.length()>0)
		{
			jointDesc.setGlobalAnchor(NxVec3(mGlobalAnchor.x,mGlobalAnchor.y,mGlobalAnchor.z));
		} else if ((mLocalAnchor0.length()>0)||(mLocalAnchor1.length()>0)) {//HERE: implement localAnchor1!
			jointDesc.localAnchor[0] = NxVec3(mLocalAnchor0.x,mLocalAnchor0.y,mLocalAnchor0.z);
			jointDesc.localAnchor[1] = NxVec3(mLocalAnchor1.x,mLocalAnchor1.y,mLocalAnchor1.z);
		} else {
			globalPos = mRB_B->getLinearPosition();
			jointDesc.setGlobalAnchor(NxVec3(globalPos.x,globalPos.y,globalPos.z));	
		}

		if (mGlobalAxis.length()>0) {
			jointDesc.setGlobalAxis(NxVec3(mGlobalAxis.x,mGlobalAxis.y,mGlobalAxis.z));
		} else if ((mLocalAxis0.length()>0)&&(mLocalAxis1.length()>0)) {
			jointDesc.localAxis[0] = NxVec3(mLocalAxis0.x,mLocalAxis0.y,mLocalAxis0.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		} else if ((mLocalAxis0.length()==0.0)&&(mLocalAxis1.length()>0.0)) { //we have only defined local axis 1, child bodypart axis, so derive parent axis from it.
			parentAxis = childMatrix * mLocalAxis1;
			jointDesc.localAxis[0] = NxVec3(parentAxis.x,parentAxis.y,parentAxis.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		}

		if ((mLocalNormal0.length()>0)&&(mLocalNormal1.length()>0)) {
			jointDesc.localNormal[0] = NxVec3(mLocalNormal0.x,mLocalNormal0.y,mLocalNormal0.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		} else if ((mLocalNormal0.length()==0)&&(mLocalNormal1.length()>0)) {//we have only defined local normal 1, child bodypart normal, so derive parent normal from it.
			parentNormal = childMatrix * mLocalNormal1;
			jointDesc.localNormal[0] = NxVec3(parentNormal.x,parentNormal.y,parentNormal.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		}
		//End ridiculous repeated section
		////////////////////////////////////////////////////////////////

		jointDesc.projectionMode = NX_JPM_POINT_MINDIST;
		//if (mCollisionEnabled) jointDesc.jointFlags |= NX_JF_COLLISION_ENABLED;

		jointDesc.maxForce = mMaxForce;
		jointDesc.maxTorque = mMaxTorque;

		//When you pass the two actors into NxScene::createJoint(), the 
		//limit values are defined against the local axes of the first actor
		if (0) {//(mHighLimit)||(mLowLimit)) {
			jointDesc.flags |= NX_RJF_LIMIT_ENABLED;
			jointDesc.limit.high.value = mHighLimit * (NxPi/180.0);
			jointDesc.limit.high.restitution = mHighRestitution;
			jointDesc.limit.low.value = mLowLimit * (NxPi/180.0);
			jointDesc.limit.low.restitution = mLowRestitution;
			//jointDesc.limit.restitution = 0.0;
			//jointDesc.limit.hardness = 0.0;
		} else {
			jointDesc.flags |= NX_RJF_LIMIT_ENABLED;
			jointDesc.limit.low.value = mSwingLimit * NxPi/180.0;//-(mSwingLimit) * (NxPi/180.0);
			jointDesc.limit.low.restitution = 0;//mLowRestitution;
			jointDesc.limit.high.value = mSwingLimit2 * (NxPi/180.0);
			jointDesc.limit.high.restitution = 0;//mHighRestitution;
			//jointDesc.limit.restitution = 0.0;
			//jointDesc.limit.hardness = 0.0;
		}

		//if (mSwingSpring) {
		//	NxSpringDesc springDesc;
		//	springDesc.spring = mSwingSpring;
		//	springDesc.damper = mSpringDamper;
		//	springDesc.targetValue = mSpringTargetAngle * (NxPi/180.0);
		//	jointDesc.spring = springDesc;
		//	jointDesc.flags |= NX_RJF_SPRING_ENABLED;
		//}

		//if (mMotorForce) {
		//	NxMotorDesc motorDesc;
		//	motorDesc.velTarget = mMotorSpeed;
		//	motorDesc.maxForce = mMotorForce;
		//	motorDesc.freeSpin = true;
		//	jointDesc.motor = motorDesc;
		//	jointDesc.flags |= NX_RJF_MOTOR_ENABLED;
		//}

		if (jointDesc.isValid()) {
			if ((mHW)&&(kHWScene)) mJoint = kHWScene->createJoint(jointDesc);
			else mJoint = kScene->createJoint(jointDesc);
		} //else Con::printf("revolute joint desc is invalid!");

	
	/////////////////////////////////////////////////////////////////////////////////////////////////
	} else if (mJointType == PHYS_JOINT_PRISMATIC) {

		NxPrismaticJointDesc jointDesc;
		if (mRB_A) jointDesc.actor[0] = ((nxRigidBody *)mRB_A)->mActor;
		if (mRB_B) jointDesc.actor[1] = ((nxRigidBody *)mRB_B)->mActor;

		////////////////////////////////////////////////////////////////
		//HERE: For now this whole huge section is going to have to be repeated for every single joint type (booo!) 
		//because we're defining a different type of jointDesc each time.  Would be much cleaner to 
		//convert jointDesc pointer to its common parent class and only do this once.  FIX.
		//childQuat = mRB_B->getAngularPosition();
		//childQuat.setMatrix(&childMatrix);

		if (mGlobalAnchor.length()>0)
		{
			jointDesc.setGlobalAnchor(NxVec3(mGlobalAnchor.x,mGlobalAnchor.y,mGlobalAnchor.z));
		} else if ((mLocalAnchor0.length()>0)||(mLocalAnchor1.length()>0)) {//HERE: implement localAnchor1!
			jointDesc.localAnchor[0] = NxVec3(mLocalAnchor0.x,mLocalAnchor0.y,mLocalAnchor0.z);
			jointDesc.localAnchor[1] = NxVec3(mLocalAnchor1.x,mLocalAnchor1.y,mLocalAnchor1.z);
		} else {
			globalPos = mRB_B->getLinearPosition();
			jointDesc.setGlobalAnchor(NxVec3(globalPos.x,globalPos.y,globalPos.z));	
		}

		if (mGlobalAxis.length()>0) {
			jointDesc.setGlobalAxis(NxVec3(mGlobalAxis.x,mGlobalAxis.y,mGlobalAxis.z));
		} else if ((mLocalAxis0.length()>0)&&(mLocalAxis1.length()>0)) {
			jointDesc.localAxis[0] = NxVec3(mLocalAxis0.x,mLocalAxis0.y,mLocalAxis0.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		} else if ((mLocalAxis0.length()==0.0)&&(mLocalAxis1.length()>0.0)) { //we have only defined local axis 1, child bodypart axis, so derive parent axis from it.
			parentAxis = childMatrix * mLocalAxis1;
			jointDesc.localAxis[0] = NxVec3(parentAxis.x,parentAxis.y,parentAxis.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		}

		if ((mLocalNormal0.length()>0)&&(mLocalNormal1.length()>0)) {
			jointDesc.localNormal[0] = NxVec3(mLocalNormal0.x,mLocalNormal0.y,mLocalNormal0.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		} else if ((mLocalNormal0.length()==0)&&(mLocalNormal1.length()>0)) {//we have only defined local normal 1, child bodypart normal, so derive parent normal from it.
			parentNormal = childMatrix * mLocalNormal1;
			jointDesc.localNormal[0] = NxVec3(parentNormal.x,parentNormal.y,parentNormal.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		}
		//End ridiculous repeated section
		////////////////////////////////////////////////////////////////

		if (mCollisionEnabled) jointDesc.jointFlags |= NX_JF_COLLISION_ENABLED;

		jointDesc.maxForce = mMaxForce;
		jointDesc.maxTorque = mMaxTorque;

		if (jointDesc.isValid()) {
			if ((mHW)&&(kHWScene)) mJoint = kHWScene->createJoint(jointDesc);
			else mJoint = kScene->createJoint(jointDesc);
		} //else Con::printf("prismatic joint desc is invalid!");
		//mJoint->setLimitPoint(globalAnchor);

	
	/////////////////////////////////////////////////////////////////////////////////////////////////
	} else if (mJointType == PHYS_JOINT_CYLINDRICAL) {

		NxCylindricalJointDesc jointDesc;
		if (mRB_A) jointDesc.actor[0] = ((nxRigidBody *)mRB_A)->mActor;
		if (mRB_B) jointDesc.actor[1] = ((nxRigidBody *)mRB_B)->mActor;

		////////////////////////////////////////////////////////////////
		//HERE: For now this whole huge section is going to have to be repeated for every single joint type (booo!) 
		//because we're defining a different type of jointDesc each time.  Would be much cleaner to 
		//convert jointDesc pointer to its common parent class and only do this once.  FIX.
		//childQuat = mRB_B->getAngularPosition();
		//childQuat.setMatrix(&childMatrix);

		if (mGlobalAnchor.length()>0)
		{
			jointDesc.setGlobalAnchor(NxVec3(mGlobalAnchor.x,mGlobalAnchor.y,mGlobalAnchor.z));
		} else if ((mLocalAnchor0.length()>0)||(mLocalAnchor1.length()>0)) {//HERE: implement localAnchor1!
			jointDesc.localAnchor[0] = NxVec3(mLocalAnchor0.x,mLocalAnchor0.y,mLocalAnchor0.z);
			jointDesc.localAnchor[1] = NxVec3(mLocalAnchor1.x,mLocalAnchor1.y,mLocalAnchor1.z);
		} else {
			globalPos = mRB_B->getLinearPosition();
			jointDesc.setGlobalAnchor(NxVec3(globalPos.x,globalPos.y,globalPos.z));	
		}

		if (mGlobalAxis.length()>0) {
			//Con::printf("Cylindrical setting global axis: %3.2f %3.2f %3.2f",mGlobalAxis.x,mGlobalAxis.y,mGlobalAxis.z);
			jointDesc.setGlobalAxis(NxVec3(mGlobalAxis.x,mGlobalAxis.y,mGlobalAxis.z));
		} else if ((mLocalAxis0.length()>0)&&(mLocalAxis1.length()>0)) {
			//Con::printf("Cylindrical joint setting 0 & 1 axes: %3.2f %3.2f %3.2f, %3.2f %3.2f %3.2f",mLocalAxis0.x,mLocalAxis0.y,mLocalAxis0.z,mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
			jointDesc.localAxis[0] = NxVec3(mLocalAxis0.x,mLocalAxis0.y,mLocalAxis0.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		} else if ((mLocalAxis0.length()==0.0)&&(mLocalAxis1.length()>0.0)) { //we have only defined local axis 1, child bodypart axis, so derive parent axis from it.
			//Con::printf("Cylindrical joint got a local axis 1 only: %f %f %f",mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
			parentAxis = childMatrix * mLocalAxis1;
			jointDesc.localAxis[0] = NxVec3(parentAxis.x,parentAxis.y,parentAxis.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		}

		if ((mLocalNormal0.length()>0)&&(mLocalNormal1.length()>0)) {
			jointDesc.localNormal[0] = NxVec3(mLocalNormal0.x,mLocalNormal0.y,mLocalNormal0.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		} else if ((mLocalNormal0.length()==0)&&(mLocalNormal1.length()>0)) {//we have only defined local normal 1, child bodypart normal, so derive parent normal from it.
			//Con::printf("Cylindrical joint got a local normal 1 only: %f %f %f",mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
			parentNormal = childMatrix * mLocalNormal1;
			jointDesc.localNormal[0] = NxVec3(parentNormal.x,parentNormal.y,parentNormal.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		}
		//End ridiculous repeated section
		////////////////////////////////////////////////////////////////

		if (mCollisionEnabled) jointDesc.jointFlags |= NX_JF_COLLISION_ENABLED;

		jointDesc.maxForce = mMaxForce;
		jointDesc.maxTorque = mMaxTorque;

		if (jointDesc.isValid()) {
			if ((mHW)&&(kHWScene)) mJoint = kHWScene->createJoint(jointDesc);
			else mJoint = kScene->createJoint(jointDesc);
		}// else Con::printf("cylindrical joint desc is invalid!");
		//mJoint->setLimitPoint(globalAnchor);

	
	/////////////////////////////////////////////////////////////////////////////////////////////////
	} else if (mJointType == PHYS_JOINT_POINT_ON_LINE) {

		NxPointOnLineJointDesc jointDesc;
		if (mRB_A) jointDesc.actor[0] = ((nxRigidBody *)mRB_A)->mActor;
		if (mRB_B) jointDesc.actor[1] = ((nxRigidBody *)mRB_B)->mActor;

		////////////////////////////////////////////////////////////////
		//HERE: For now this whole huge section is going to have to be repeated for every single joint type (booo!) 
		//because we're defining a different type of jointDesc each time.  Would be much cleaner to 
		//convert jointDesc pointer to its common parent class and only do this once.  FIX.
		//childQuat = mRB_B->getAngularPosition();
		//childQuat.setMatrix(&childMatrix);

		if (mGlobalAnchor.length()>0)
		{
			jointDesc.setGlobalAnchor(NxVec3(mGlobalAnchor.x,mGlobalAnchor.y,mGlobalAnchor.z));
		} else if ((mLocalAnchor0.length()>0)||(mLocalAnchor1.length()>0)) {//HERE: implement localAnchor1!
			jointDesc.localAnchor[0] = NxVec3(mLocalAnchor0.x,mLocalAnchor0.y,mLocalAnchor0.z);
			jointDesc.localAnchor[1] = NxVec3(mLocalAnchor1.x,mLocalAnchor1.y,mLocalAnchor1.z);
		} else {
			globalPos = mRB_B->getLinearPosition();
			jointDesc.setGlobalAnchor(NxVec3(globalPos.x,globalPos.y,globalPos.z));	
		}

		if (mGlobalAxis.length()>0) {
			//Con::printf("PointOnLine joint setting global axis: %3.2f %3.2f %3.2f",mGlobalAxis.x,mGlobalAxis.y,mGlobalAxis.z);
			jointDesc.setGlobalAxis(NxVec3(mGlobalAxis.x,mGlobalAxis.y,mGlobalAxis.z));
		} else if ((mLocalAxis0.length()>0)&&(mLocalAxis1.length()>0)) {
			//Con::printf("PointOnLine joint setting 0 & 1 axes: %3.2f %3.2f %3.2f, %3.2f %3.2f %3.2f",mLocalAxis0.x,mLocalAxis0.y,mLocalAxis0.z,mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
			jointDesc.localAxis[0] = NxVec3(mLocalAxis0.x,mLocalAxis0.y,mLocalAxis0.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		} else if ((mLocalAxis0.length()==0.0)&&(mLocalAxis1.length()>0.0)) { //we have only defined local axis 1, child bodypart axis, so derive parent axis from it.
			//Con::printf("PointOnLine joint got a local axis 1 only: %f %f %f",mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
			parentAxis = childMatrix * mLocalAxis1;
			jointDesc.localAxis[0] = NxVec3(parentAxis.x,parentAxis.y,parentAxis.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		}

		if ((mLocalNormal0.length()>0)&&(mLocalNormal1.length()>0)) {
			jointDesc.localNormal[0] = NxVec3(mLocalNormal0.x,mLocalNormal0.y,mLocalNormal0.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		} else if ((mLocalNormal0.length()==0)&&(mLocalNormal1.length()>0)) {//we have only defined local normal 1, child bodypart normal, so derive parent normal from it.
			//Con::printf("PointOnLine joint got a local normal 1 only: %f %f %f",mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
			parentNormal = childMatrix * mLocalNormal1;
			jointDesc.localNormal[0] = NxVec3(parentNormal.x,parentNormal.y,parentNormal.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		}
		//End ridiculous repeated section
		////////////////////////////////////////////////////////////////

		if (jointDesc.isValid())
			mJoint = kScene->createJoint(jointDesc);
		//else Con::printf("point on line joint desc is invalid!");

	
	/////////////////////////////////////////////////////////////////////////////////////////////////
	} else if (mJointType == PHYS_JOINT_POINT_IN_PLANE) {

		NxPointInPlaneJointDesc jointDesc;
		if (mRB_A) jointDesc.actor[0] = ((nxRigidBody *)mRB_A)->mActor;
		if (mRB_B) jointDesc.actor[1] = ((nxRigidBody *)mRB_B)->mActor;

		////////////////////////////////////////////////////////////////
		//HERE: For now this whole huge section is going to have to be repeated for every single joint type (booo!) 
		//because we're defining a different type of jointDesc each time.  Would be much cleaner to 
		//convert jointDesc pointer to its common parent class and only do this once.  FIX.
		//childQuat = mRB_B->getAngularPosition();
		//childQuat.setMatrix(&childMatrix);

		if (mGlobalAnchor.length()>0)
		{
			jointDesc.setGlobalAnchor(NxVec3(mGlobalAnchor.x,mGlobalAnchor.y,mGlobalAnchor.z));
		} else if ((mLocalAnchor0.length()>0)||(mLocalAnchor1.length()>0)) {//HERE: implement localAnchor1!
			jointDesc.localAnchor[0] = NxVec3(mLocalAnchor0.x,mLocalAnchor0.y,mLocalAnchor0.z);
			jointDesc.localAnchor[1] = NxVec3(mLocalAnchor1.x,mLocalAnchor1.y,mLocalAnchor1.z);
		} else {
			globalPos = mRB_B->getLinearPosition();
			jointDesc.setGlobalAnchor(NxVec3(globalPos.x,globalPos.y,globalPos.z));	
		}

		if (mGlobalAxis.length()>0) {
			//Con::printf("PointInPlane joint setting global axis: %3.2f %3.2f %3.2f",mGlobalAxis.x,mGlobalAxis.y,mGlobalAxis.z);
			jointDesc.setGlobalAxis(NxVec3(mGlobalAxis.x,mGlobalAxis.y,mGlobalAxis.z));
		} else if ((mLocalAxis0.length()>0)&&(mLocalAxis1.length()>0)) {
			//Con::printf("PointInPlane joint setting 0 & 1 axes: %3.2f %3.2f %3.2f, %3.2f %3.2f %3.2f",mLocalAxis0.x,mLocalAxis0.y,mLocalAxis0.z,mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
			jointDesc.localAxis[0] = NxVec3(mLocalAxis0.x,mLocalAxis0.y,mLocalAxis0.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		} else if ((mLocalAxis0.length()==0.0)&&(mLocalAxis1.length()>0.0)) { //we have only defined local axis 1, child bodypart axis, so derive parent axis from it.
			//Con::printf("PointInPlane joint got a local axis 1 only: %f %f %f",mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
			parentAxis = childMatrix * mLocalAxis1;
			jointDesc.localAxis[0] = NxVec3(parentAxis.x,parentAxis.y,parentAxis.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		}

		if ((mLocalNormal0.length()>0)&&(mLocalNormal1.length()>0)) {
			jointDesc.localNormal[0] = NxVec3(mLocalNormal0.x,mLocalNormal0.y,mLocalNormal0.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		} else if ((mLocalNormal0.length()==0)&&(mLocalNormal1.length()>0)) {//we have only defined local normal 1, child bodypart normal, so derive parent normal from it.
			//Con::printf("PointInPlane joint got a local normal 1 only: %f %f %f",mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
			parentNormal = childMatrix * mLocalNormal1;
			jointDesc.localNormal[0] = NxVec3(parentNormal.x,parentNormal.y,parentNormal.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		}
		//End ridiculous repeated section
		////////////////////////////////////////////////////////////////

		if (jointDesc.isValid())
			mJoint = kScene->createJoint(jointDesc);
		//else Con::printf("point in plane joint desc is invalid!");

	
	/////////////////////////////////////////////////////////////////////////////////////////////////
	} else if (mJointType == PHYS_JOINT_DISTANCE) {

		NxDistanceJointDesc jointDesc;
		if (mRB_A) jointDesc.actor[0] = ((nxRigidBody *)mRB_A)->mActor;
		if (mRB_B) jointDesc.actor[1] = ((nxRigidBody *)mRB_B)->mActor;

		////////////////////////////////////////////////////////////////
		//HERE: For now this whole huge section is going to have to be repeated for every single joint type (booo!) 
		//because we're defining a different type of jointDesc each time.  Would be much cleaner to 
		//convert jointDesc pointer to its common parent class and only do this once.  FIX.
		//childQuat = mRB_B->getAngularPosition();
		//childQuat.setMatrix(&childMatrix);

		if (mGlobalAnchor.length()>0)
		{
			jointDesc.setGlobalAnchor(NxVec3(mGlobalAnchor.x,mGlobalAnchor.y,mGlobalAnchor.z));
		} else if ((mLocalAnchor0.length()>0)||(mLocalAnchor1.length()>0)) {//HERE: implement localAnchor1!
			jointDesc.localAnchor[0] = NxVec3(mLocalAnchor0.x,mLocalAnchor0.y,mLocalAnchor0.z);
			jointDesc.localAnchor[1] = NxVec3(mLocalAnchor1.x,mLocalAnchor1.y,mLocalAnchor1.z);
		} else {
			globalPos = mRB_B->getLinearPosition();
			jointDesc.setGlobalAnchor(NxVec3(globalPos.x,globalPos.y,globalPos.z));	
		}

		if (mGlobalAxis.length()>0) {
			//Con::printf("Distance joint setting global axis: %3.2f %3.2f %3.2f",mGlobalAxis.x,mGlobalAxis.y,mGlobalAxis.z);
			jointDesc.setGlobalAxis(NxVec3(mGlobalAxis.x,mGlobalAxis.y,mGlobalAxis.z));
		} else if ((mLocalAxis0.length()>0)&&(mLocalAxis1.length()>0)) {
			//Con::printf("Distance joint setting 0 & 1 axes: %3.2f %3.2f %3.2f, %3.2f %3.2f %3.2f",mLocalAxis0.x,mLocalAxis0.y,mLocalAxis0.z,mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
			jointDesc.localAxis[0] = NxVec3(mLocalAxis0.x,mLocalAxis0.y,mLocalAxis0.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		} else if ((mLocalAxis0.length()==0.0)&&(mLocalAxis1.length()>0.0)) { //we have only defined local axis 1, child bodypart axis, so derive parent axis from it.
			//Con::printf("Distance joint got a local axis 1 only: %f %f %f",mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
			parentAxis = childMatrix * mLocalAxis1;
			jointDesc.localAxis[0] = NxVec3(parentAxis.x,parentAxis.y,parentAxis.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		}

		if ((mLocalNormal0.length()>0)&&(mLocalNormal1.length()>0)) {
			jointDesc.localNormal[0] = NxVec3(mLocalNormal0.x,mLocalNormal0.y,mLocalNormal0.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		} else if ((mLocalNormal0.length()==0)&&(mLocalNormal1.length()>0)) {//we have only defined local normal 1, child bodypart normal, so derive parent normal from it.
			//Con::printf("Distance joint got a local normal 1 only: %f %f %f",mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
			parentNormal = childMatrix * mLocalNormal1;
			jointDesc.localNormal[0] = NxVec3(parentNormal.x,parentNormal.y,parentNormal.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		}
		//End ridiculous repeated section
		////////////////////////////////////////////////////////////////

		//HERE: deal with jointDesc.maxDistance, .minDistance

		if (jointDesc.isValid()) 
			mJoint = kScene->createJoint(jointDesc);
		// else Con::printf("distance joint desc is invalid!");

	
	/////////////////////////////////////////////////////////////////////////////////////////////////
	} else if (mJointType == PHYS_JOINT_PULLEY) {

		NxPulleyJointDesc jointDesc;
		if (mRB_A) jointDesc.actor[0] = ((nxRigidBody *)mRB_A)->mActor;
		if (mRB_B) jointDesc.actor[1] = ((nxRigidBody *)mRB_B)->mActor;

		////////////////////////////////////////////////////////////////
		//HERE: For now this whole huge section is going to have to be repeated for every single joint type (booo!) 
		//because we're defining a different type of jointDesc each time.  Would be much cleaner to 
		//convert jointDesc pointer to its common parent class and only do this once.  FIX.
		//childQuat = mRB_B->getAngularPosition();
		//childQuat.setMatrix(&childMatrix);

		if (mGlobalAnchor.length()>0)
		{
			jointDesc.setGlobalAnchor(NxVec3(mGlobalAnchor.x,mGlobalAnchor.y,mGlobalAnchor.z));
		} else if ((mLocalAnchor0.length()>0)||(mLocalAnchor1.length()>0)) {//HERE: implement localAnchor1!
			jointDesc.localAnchor[0] = NxVec3(mLocalAnchor0.x,mLocalAnchor0.y,mLocalAnchor0.z);
			jointDesc.localAnchor[1] = NxVec3(mLocalAnchor1.x,mLocalAnchor1.y,mLocalAnchor1.z);
		} else {
			globalPos = mRB_B->getLinearPosition();
			jointDesc.setGlobalAnchor(NxVec3(globalPos.x,globalPos.y,globalPos.z));	
		}

		if (mGlobalAxis.length()>0) {
			//Con::printf("Pulley joint setting global axis: %3.2f %3.2f %3.2f",mGlobalAxis.x,mGlobalAxis.y,mGlobalAxis.z);
			jointDesc.setGlobalAxis(NxVec3(mGlobalAxis.x,mGlobalAxis.y,mGlobalAxis.z));
		} else if ((mLocalAxis0.length()>0)&&(mLocalAxis1.length()>0)) {
			//Con::printf("Pulley joint setting 0 & 1 axes: %3.2f %3.2f %3.2f, %3.2f %3.2f %3.2f",mLocalAxis0.x,mLocalAxis0.y,mLocalAxis0.z,mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
			jointDesc.localAxis[0] = NxVec3(mLocalAxis0.x,mLocalAxis0.y,mLocalAxis0.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		} else if ((mLocalAxis0.length()==0.0)&&(mLocalAxis1.length()>0.0)) { //we have only defined local axis 1, child bodypart axis, so derive parent axis from it.
			//Con::printf("Pulley joint got a local axis 1 only: %f %f %f",mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
			parentAxis = childMatrix * mLocalAxis1;
			jointDesc.localAxis[0] = NxVec3(parentAxis.x,parentAxis.y,parentAxis.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		}

		if ((mLocalNormal0.length()>0)&&(mLocalNormal1.length()>0)) {
			jointDesc.localNormal[0] = NxVec3(mLocalNormal0.x,mLocalNormal0.y,mLocalNormal0.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		} else if ((mLocalNormal0.length()==0)&&(mLocalNormal1.length()>0)) {//we have only defined local normal 1, child bodypart normal, so derive parent normal from it.
			//Con::printf("Pulley joint got a local normal 1 only: %f %f %f",mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
			parentNormal = childMatrix * mLocalNormal1;
			jointDesc.localNormal[0] = NxVec3(parentNormal.x,parentNormal.y,parentNormal.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		}
		//End ridiculous repeated section
		////////////////////////////////////////////////////////////////

		//Here: deal with jointDesc.pulley[0]/[1], .distance, .stiffness, .ratio
		if (jointDesc.isValid()) 
			mJoint = kScene->createJoint(jointDesc);
		//else Con::printf("pulley joint desc is invalid!");

	
	/////////////////////////////////////////////////////////////////////////////////////////////////
	} else if (mJointType == PHYS_JOINT_FIXED) {

		NxFixedJointDesc jointDesc;
		if (mRB_A) jointDesc.actor[0] = ((nxRigidBody *)mRB_A)->mActor;
		if (mRB_B) jointDesc.actor[1] = ((nxRigidBody *)mRB_B)->mActor;

		////////////////////////////////////////////////////////////////
		//HERE: For now this whole huge section is going to have to be repeated for every single joint type (booo!) 
		//because we're defining a different type of jointDesc each time.  Would be much cleaner to 
		//convert jointDesc pointer to its common parent class and only do this once.  FIX.
		//childQuat = mRB_B->getAngularPosition();
		//childQuat.setMatrix(&childMatrix);

		jointDesc.maxForce = mMaxForce;
		jointDesc.maxTorque = mMaxTorque;

		if (mGlobalAnchor.length()>0)
		{
			jointDesc.setGlobalAnchor(NxVec3(mGlobalAnchor.x,mGlobalAnchor.y,mGlobalAnchor.z));
		} else if ((mLocalAnchor0.length()>0)||(mLocalAnchor1.length()>0)) {//HERE: implement localAnchor1!
			jointDesc.localAnchor[0] = NxVec3(mLocalAnchor0.x,mLocalAnchor0.y,mLocalAnchor0.z);
			jointDesc.localAnchor[1] = NxVec3(mLocalAnchor1.x,mLocalAnchor1.y,mLocalAnchor1.z);
		} else {
			globalPos = mRB_B->getLinearPosition();
			jointDesc.setGlobalAnchor(NxVec3(globalPos.x,globalPos.y,globalPos.z));	
		}

		if (mGlobalAxis.length()>0) {
			//Con::printf("Fixed joint setting global axis: %3.2f %3.2f %3.2f",mGlobalAxis.x,mGlobalAxis.y,mGlobalAxis.z);
			jointDesc.setGlobalAxis(NxVec3(mGlobalAxis.x,mGlobalAxis.y,mGlobalAxis.z));
		} else if ((mLocalAxis0.length()>0)&&(mLocalAxis1.length()>0)) {
			//Con::printf("Fixed joint setting 0 & 1 axes: %3.2f %3.2f %3.2f, %3.2f %3.2f %3.2f",mLocalAxis0.x,mLocalAxis0.y,mLocalAxis0.z,mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
			jointDesc.localAxis[0] = NxVec3(mLocalAxis0.x,mLocalAxis0.y,mLocalAxis0.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		} else if ((mLocalAxis0.length()==0.0)&&(mLocalAxis1.length()>0.0)) { //we have only defined local axis 1, child bodypart axis, so derive parent axis from it.
			//Con::printf("Fixed joint got a local axis 1 only: %f %f %f",mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
			parentAxis = childMatrix * mLocalAxis1;
			jointDesc.localAxis[0] = NxVec3(parentAxis.x,parentAxis.y,parentAxis.z);
			jointDesc.localAxis[1] = NxVec3(mLocalAxis1.x,mLocalAxis1.y,mLocalAxis1.z);
		}

		if ((mLocalNormal0.length()>0)&&(mLocalNormal1.length()>0)) {
			jointDesc.localNormal[0] = NxVec3(mLocalNormal0.x,mLocalNormal0.y,mLocalNormal0.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		} else if ((mLocalNormal0.length()==0)&&(mLocalNormal1.length()>0)) {//we have only defined local normal 1, child bodypart normal, so derive parent normal from it.
			//Con::printf("Fixed joint got a local normal 1 only: %f %f %f",mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
			parentNormal = childMatrix * mLocalNormal1;
			jointDesc.localNormal[0] = NxVec3(parentNormal.x,parentNormal.y,parentNormal.z);
			jointDesc.localNormal[1] = NxVec3(mLocalNormal1.x,mLocalNormal1.y,mLocalNormal1.z);
		}
		//End ridiculous repeated section
		////////////////////////////////////////////////////////////////

		if (mMaxForce)  jointDesc.maxForce = mMaxForce;
		if (mMaxTorque) jointDesc.maxTorque = mMaxTorque;

		if (jointDesc.isValid()) {
			if ((mHW)&&(kHWScene)) mJoint = kHWScene->createJoint(jointDesc);
			mJoint = kScene->createJoint(jointDesc);
		} //else Con::printf("fixed joint desc is invalid!");
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////

	if ((mLimitPoint.length())&&(mJoint)) {
		Ogre::Vector3 limitP,limitA,limitN;
		limitP = mLimitPoint;
		limitN = mLimitPlaneNormal1;
		limitA = mLimitPlaneAnchor1;

		p = mRB_B->getLinearPosition();
		NxVec3 kAnchor(p.x,p.y,p.z);

		mJoint->setLimitPoint(NxVec3(limitP.x + kAnchor.x,limitP.y+ kAnchor.y,limitP.z + kAnchor.z));

		mJoint->addLimitPlane(NxVec3(limitN.x,limitN.y,limitN.z), 
			NxVec3(limitA.x + kAnchor.x,limitA.y + kAnchor.y,limitA.z + kAnchor.z));

		if (mLimitPlaneAnchor2.length()) {
			limitN = mLimitPlaneNormal2;
			limitA = mLimitPlaneAnchor2;
			mJoint->addLimitPlane(NxVec3(limitN.x,limitN.y,limitN.z),
				NxVec3(limitA.x + kAnchor.x,limitA.y + kAnchor.y,limitA.z + kAnchor.z));
		}
		if (mLimitPlaneAnchor3.length()) { 
			limitN = mLimitPlaneNormal3;
			limitA = mLimitPlaneAnchor3;              
			mJoint->addLimitPlane(NxVec3(limitN.x,limitN.y,limitN.z),
				NxVec3(limitA.x + kAnchor.x,limitA.y + kAnchor.y,limitA.z + kAnchor.z));
		}
		if (mLimitPlaneAnchor4.length()) { 
			limitN = mLimitPlaneNormal4;
			limitA = mLimitPlaneAnchor4;        
			mJoint->addLimitPlane(NxVec3(limitN.x,limitN.y,limitN.z),
				NxVec3(limitA.x + kAnchor.x,limitA.y + kAnchor.y,limitA.z + kAnchor.z));
		}
	}
}

