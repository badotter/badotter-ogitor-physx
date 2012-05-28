#include "EcstasyMotion/gaAction.h"
#include "EcstasyMotion/gaObservation.h"
//#include "mathutil.h"
//#include "entity.h"
#include "EcstasyMotion/nxRigidBody.h"
//#include "ts/tsShapeInstance.h"
//#include "core/stream/fileStream.h"
//#include "core/resourceManager.h"

//#define NUM_STEPS 500

#define GA_MUTATE_CHANCE            0.25
#define GA_MUTATE_AMOUNT            0.25

#define GA_NUM_ISLANDS              1
#define GA_MIGRATE_CHANCE           0.0

#define GA_NUM_SEQUENCE_STEPS       60
#define GA_NUM_REST_STEPS           20
#define GA_OBSERVE_INTERVAL         6

#define GA_NUM_ACTION_SETS          6
#define GA_NUM_SLICES               4
#define GA_NUM_SEQ_REPS             1

#define GA_GOAL_NONE                0
#define GA_GOAL_SINGLE_ACTION       1
#define GA_GOAL_PLAY_SEQUENCE       2
#define GA_GOAL_FITNESS_TRAINING    3

//////////////////////////////////////////

#define GA_ACTION_NONE              0
#define GA_ACTION_START             1
#define GA_ACTION_ACT               2
#define GA_ACTION_RESET             3
#define GA_ACTION_END_SET           4

//////////////////////////////////////////

//MRandomLCG gRandom;
//extern int simStep;
//extern int sort_list[100];
//extern int stepLength;
//int simStep = 0;
int sort_list[100];

int sequence_count;
int total_sequence_count=0;


class physJoint;
class nxRigidBody;

//(action forces in sets of three, corresponding to torque on joint for x,y,z axes)
gaAction::gaAction()
{

	mBodyIndex = NULL;
	mForces = Ogre::Vector3::ZERO;
	mQuat = Ogre::Quaternion::IDENTITY;

	mStart =     0.0;
	mDuration =  0.0;

	mStartStep = 0;
	mNumSteps = 0;
	mEndStep = 0;

	mIsTraining = false;
}

gaAction::gaAction(gaAction *other) 
{
	mBodyIndex = other->mBodyIndex;
	mForces = other->mForces;
	mQuat = other->mQuat;
	mStart =     other->mStart;
	mDuration =  other->mDuration;
	mIsTraining = other->mIsTraining;

	setSteps(GA_NUM_SEQUENCE_STEPS);
	//startStep = (int)(start * (float)steps);
	//numSteps = (int)(duration * (float)steps);
	//endStep = (int)((start + duration) * (float)steps);
}

gaAction::gaAction(int rb,int steps,float s,float d) 
{//"blank" action
	mBodyIndex = rb;
	mStart = s;
	mDuration = d;
	mForces = Ogre::Vector3::ZERO;
	mQuat = Ogre::Quaternion::IDENTITY;

	mStartStep = (int)(mStart * (float)steps);
	mNumSteps = (int)(mDuration * (float)steps);
	mEndStep = (int)((mStart + mDuration) * (float)steps);
}

gaAction::gaAction(int rb,int steps,float s,float d,Ogre::Vector3 &force) 
{
	mBodyIndex = rb;
	mStart = s;
	mDuration = d;
	mForces = force;
	setSteps(steps);
}

gaAction::~gaAction() 
{
	//do nothing yet
}

void gaAction::set(gaAction *A) 
{
	mBodyIndex     =    A->mBodyIndex;
	mForces =    A->mForces;
	mQuat =    A->mQuat;
	mStart =     A->mStart;
	mDuration =  A->mDuration;

	mStartStep = A->mStartStep;
	mNumSteps =  A->mNumSteps;
	mEndStep =   A->mEndStep;
}

void gaAction::set(int rb,int steps,float s,float d,Ogre::Vector3 &force)
{
	mBodyIndex = rb;
	mStart = s;
	mDuration = d;
	mForces = force;

	mStartStep = (int)(mStart * (float)steps);
	mNumSteps = (int)(mDuration * (float)steps);
	mEndStep = (int)((mStart + mDuration) * (float)steps);
}

void gaAction::setSteps(int steps) 
{
	mStartStep = (int)(mStart * (float)steps);
	mNumSteps = (int)(mDuration * (float)steps);
	mEndStep = (int)((mStart + mDuration) * (float)steps);
}

void gaAction::clear()
{
	mBodyIndex = NULL;
	mForces = Ogre::Vector3::ZERO;
	mQuat = Ogre::Quaternion::IDENTITY;
	mStart =     0.0;
	mDuration =  0.0;

	mStartStep = 0;
	mNumSteps = 0;
	mEndStep = 0;

	mIsTraining = false;
}
//void gaAction::save(FILE *fp)
//{
//  fprintf(fp,"%d;%1.3f;%1.3f;%1.3f;%1.3f;%1.3f;\n",bodyID,start,duration,forces.X,forces.Y,forces.Z);
//}

//void gaAction::load(FILE *fp)
//{
//  float x,y,z;
//  fscanf(fp,"%d;%f;%f;%f;%f;%f;\n",&bodyID,&start,&duration,&x,&y,&z);
//  forces = Ogre::Vector3(x,y,z);
//}


//////////////////////////////////////////////////
/////////   g a A c t i o n S e t        /////////

gaActionSet::gaActionSet()
{
	init(0);//GA_NUM_SEQUENCE_STEPS
}

gaActionSet::gaActionSet(int num_bodies,gaActionUser *au)
{
	mAU = au;
	init(num_bodies);
}

gaActionSet::gaActionSet(gaActionSet *A)
{
	init(0);
	set(A);
}

gaActionSet::~gaActionSet()
{

}

void gaActionSet::init(int num_bodies)
{
	mNumBodies = num_bodies;
	mNumSlices = 0;
	mNumActions = 0;
	mNumSteps = 0;

	mCurrTime = 0.0;
	mCurrAction = 0;
	mDuration = 0.0;

	mScore = 0.0;
	mSOA = 0.0;
	mSMA = 0.0;

	mWeight = 0.0;
	mBodyGroup = 0;
	
	//mRandSeed = 1;
	//gRandom.setSeed(mRandSeed);

	for (unsigned int i=0;i<GA_MAX_ACTIONS;i++) mActions[i]=NULL;

}

void gaActionSet::clear()
{
	if (mNumActions>0)
		for (unsigned int i=0; i<mNumActions; i++)
			if (mActions[i]) delete mActions[i];

	mNumBodies = 0;
	mNumActions = 0;
	
	mCurrTime = 0.0;
	mDuration = 0.0;

}

void gaActionSet::save(const char *filename)
{
	//int i;
	////Con::printf("ActionSet: %s",filename);
	//FileStream outstream; 
	//if (!outstream.open(filename,Torque::FS::File::Write)) {
	//	//Con::printf("actionSet::save - bad path or filename!"); 
	//} else {
	//	for (i=0;i<mNumActions;i++) {
	//		outstream.write(mActions[i]->mBodyIndex);
	//		outstream.write(mActions[i]->mStart);
	//		outstream.write(mActions[i]->mDuration);
	//		mathWrite(outstream, mActions[i]->mForces);
	//	}
	//	outstream.close();
	//}
	
}

void gaActionSet::load(const char *filename)
{
	
	//FileStream instream;
 // //fscanf(fp,"ActionSet: %d;%f;%d;\n",&numActions,&duration,&numSteps);
	//if (!instream.open(filename,Torque::FS::File::Read)) {
	//	//Con::errorf("actionSet::load - bad path or filename!"); 
	//} else {
	//	//HERE: get numActions from the file
	//	for (int i=0;i<mNumActions;i++) {
	//		mActions[i] = new gaAction();
	//		instream.read(&mActions[i]->mBodyIndex);
	//		instream.read(&mActions[i]->mStart);
	//		instream.read(&mActions[i]->mDuration);
	//		mathRead(instream,&mActions[i]->mForces);
	//		//Con::printf("loaded action %d! %f %f \n",i,mActions[i]->mStart,mActions[i]->mDuration);
	//	}
	//}
}

void gaActionSet::saveAsText(const char *filename)
{
	FILE *fp = fopen(filename,"w");
		
	for (int j=0;j<mNumActions;j++) {
		if (j==0) 
		{
			//fprintf(fp,"numActions: %d;\n",mNumActions);
			fprintf(fp,"numSlices: %d;\n",mNumSlices);
		}
		fprintf(fp,"%d;%d;%3.2f;%3.2f;(%3.2f,%3.2f,%3.2f);\n",mActions[j]->mBodyIndex,mActions[j]->mIsTraining,mActions[j]->mStart,mActions[j]->mDuration,
			mActions[j]->mForces.x,mActions[j]->mForces.y,mActions[j]->mForces.z);	
	}
	fclose(fp);
}

bool gaActionSet::loadSequence(const char *name)
{
	/*
	int mattersNodes[200],mattersBodies[200],activeNodeIndices[200];

	clear();

	const TSShape *kShape = mAU->mFlexBody->getShape();
	int seqNum = kShape->findSequence(name);
	if (seqNum<0) return false;

	TSSequence seq = kShape->sequences[seqNum];
	mSequenceName = Ogre::String(name);

	int rot_matters_count = 0;
	int node_matters_count = 0;
	unsigned int baseNode = 0;//Meant to store the location in the dts node hierarchy of our flexbody's base node.

	//for (unsigned int j=0;j<mAU->mNumBodies;j++) 
	//for (unsigned int j=1;j<mAU->mFlexBody->mNumBodyParts;j++) 
	for (unsigned int j=0;j<kShape->nodes.size();j++) // First, find and skip base node, as it doesn't have a joint
	{//Can't assume that it is nodes[0], could be farther down the chain.
		//if (seq.rotationMatters.test(mAU->mFlexBody->mBodyParts[j]->mNodeIndex))
		if (seq.rotationMatters.test(j)) node_matters_count++;
		fxFlexBodyPart *kPart = mAU->mFlexBody->getBodyPart(kShape->getName(kShape->nodes[j].nameIndex));
		if (seq.rotationMatters.test(j)&&(kPart)) 
		{
			unsigned int bodyID = kPart->mBoneIndex;
 			if (bodyID<=0)
				baseNode++;//WAIT... how can this even work at all, if it only gets here when kPart exists?
			//Okay, this is awkward... but this is how many nodes we need to skip before we get to our first
			//movable bodypart.  Hoping that we don't get nodes later that matter to the sequence but not to the model, but
			//that will totally happen.  Solution is keeping an array of all mattersnodes in the sequence, and storing
			//where our bodyparts fit into them.
			else if (bodyID>0)
			{//Here, actually eliminate my flexbody base node.  Later, save base node rotation too, and apply it kinematically or otherwise.
				mattersNodes[rot_matters_count] = j;//(j-1) because we're not dealing with base node rotations, no joint no motor. 
				//Deal with base node rotations as the one kinematic part, play the animation from there the normal way.
				mattersBodies[rot_matters_count] = bodyID;
				//Con::errorf("body %s active: node %d, bone %d",
				//	mAU->mFlexBody->mBodyParts[mattersBodies[rot_matters_count]]->mDataBlock->mBaseNodeName,
				//	j,mattersBodies[rot_matters_count]);
				mAU->mActiveBodyIndices[rot_matters_count] = rot_matters_count;//?
				activeNodeIndices[rot_matters_count] = node_matters_count-1;
				rot_matters_count++;
			} 
		}
	}

	mAU->mNumActiveBodies = rot_matters_count;
	//mAU->mNumBodies = rot_matters_count;//for this pass at least, numBodies and numActiveBodies will be the same.
	//We are not training, only copying, and there is no need to include the bodyparts that aren't in the anim.
	//AH... but what about nodes in the anim that are NOT BODYPARTS?  Is this handled?
	mNumSlices = seq.numKeyframes;
	mDuration = seq.duration;// * 10.0;//TEMP: debug, slowing down the run sequence
	mNumSteps = (int)(mDuration * 32.0);
	mNumActions = 0;

	for (unsigned int j=0; j<mAU->mNumActiveBodies; j++)
	{
		for (unsigned int i=0;i < seq.numKeyframes; i++)
		{
			mActions[mNumActions] = new gaAction();
			mActions[mNumActions]->mBodyIndex = mattersBodies[j];//mattersNodes[j];
			mActions[mNumActions]->mIsTraining = 0;//training;
			mActions[mNumActions]->mStart = (float)i / (float)mNumSlices ;
			mActions[mNumActions]->mDuration = 1.0 / (float)mNumSlices ;

			//HERE: this was breaking whenever there were nodes in the sequence that didn't exist as bodyparts.
			//Have to use activeNodeIndices to track the actual position in the nodeRotations of the ones we care about.
			Quat16 q16 = kShape->nodeRotations[seq.baseRotation+((activeNodeIndices[j])*seq.numKeyframes)+i];
			//Quat16 q16 = kShape->nodeRotations[seq.baseRotation+((j+baseNode)*seq.numKeyframes)+i];
			//(j+baseNode): we need to skip base node, which is in nodeRotations but we're not doing
			//anything with it in actionUser, because it doesn't have a joint.  Instead, it's position  
			//is going to be locked down as a groundTransform.

			Ogre::Quaternion q;
			Ogre::Matrix3 m;
			q16.getOgre::Quaternion(&q);

			//q.setMatrix(&m);
			//Ogre::Vector3 eul = m.toEuler();//OR NOT...
			////mActions[mNumActions]->mForces = Ogre::Vector3( 0.0, 0.0, 0.60 * ((float)i/(float)seq.numKeyframes) );
			//mActions[mNumActions]->mForces = Ogre::Vector3(eul.x,eul.y,eul.z);
			//q.set(mActions[mNumActions]->mForces);
			mActions[mNumActions]->mQuat = q;
			mActions[mNumActions]->setSteps(mNumSteps);
			//Con::errorf("loading quat %f %f %f %f for bodypart %d activeNodeIndices %d",
			//	q.x,q.y,q.z,q.w,j+1,activeNodeIndices[j]);

			mNumActions++;
		}
	}
	if (rot_matters_count) return true;
	else return false;
	*/
	return false;
}

bool gaActionSet::loadAsText(const char *filename, bool dynamic, bool exclusive, bool repeat)
//dynamic=false,exclusive=true, repeat=false
{
	int i,j,id,training,numParts,found;
	
	float start,duration,argval,x,y,z;
	char buf[255];	
	char argname[80];

	//for (i=0;i<GA_MAX_ACTIONS;i++) if (mActions[i]) delete mActions[i];

	FILE *fp = fopen(filename,"r");
	if (fp==NULL) return false;

	//numParts = mAU->mNumBodies; 
	//numActions = mNumActions;

	mNumBodies = 0;

	//fgets(buf,255,fp);
	//sscanf(buf,"numActions: %d;",&temp);//number doesn't matter, count lines instead

	if ((exclusive)&&(mNumActions>0))
	{
		for (i=0;i<mNumActions;i++)
			if (mActions[i]) delete mActions[i];
		mNumActions = 0;
	}

	while (fgets(buf,255,fp)) {
		if ((buf[0]=='#')||(buf[0]==' ')||(buf[0]=='\n')||(buf[0]=='\t')) continue;
		else if (buf[0]=='$') 
		{

			//strcpy(buf1,buf[1]);
			//bufp = strtok(buf,"=");
			sscanf(&buf[1],"%s = %f;",&argname,&argval);  

			//Important: make sure whether or not we need ".0" to make it read float
			//can't read %d because we might need a float next.

			if (!strcmp(argname,"numSteps"))
			{
				mNumSteps = (int)argval;
				//Con::errorf("action set num steps: %d",mNumSteps);
			}
			if (!strcmp(argname,"numSlices"))
			{
				mNumSlices = (int)argval;
				//Con::errorf("action set num slices: %d",mNumSlices);
			}

			if (!strcmp(argname,"dynamic"))
				if (argval) dynamic = true;

			if (!strcmp(argname,"exclusive"))
				if (argval) exclusive = true;

			if (!strcmp(argname,"repeat"))
				if (argval) repeat = true;

			continue;	
		}
		else
		{
			sscanf(buf,"%d;%d;%f;%f;(%f,%f,%f);",&id,&training,&start,&duration,&x,&y,&z);
			mActions[mNumActions] = new gaAction();

			found=0;
			for (j=0;j<mNumBodies;j++) if (parts[j]==id) found=1;
			if (!found) { //count each active bodypart only once
				//mAU->mActiveBodyIndices[numParts]=id;
				mAU->mActiveBodyIndices[mNumBodies]=id;
				parts[mNumBodies]=id;
				mNumBodies++;
			}

			mActions[mNumActions]->mBodyIndex = id;
			mActions[mNumActions]->mIsTraining = training;
			mActions[mNumActions]->mStart = start;
			mActions[mNumActions]->mDuration = duration;
			mActions[mNumActions]->mForces = Ogre::Vector3(mDegToRad(x),mDegToRad(y),mDegToRad(z));
			mActions[mNumActions]->setSteps(mNumSteps);

			//Con::printf("action forces: %f %f %f",x,y,z);
			mNumActions++;
		}
	}
	mAU->mNumActiveBodies = mNumBodies;
	//mNumBodies = numParts;
	//mNumActions = numActions;

	//all bodies are potentially active with dynamic actions
	//if (dynamic)//(but do we want this?  might want some "dead" parts that don't move.)
	//{
		//mAU->mNumActiveBodies = mAU->mNumBodies;
		//mAU->setGoalDynamic();
	//}// if (repeat)?... if (exclusive)?...
	//else//or else we count them as we load actions
	//{
		//mAU->setGoalSingleAction();
	//}
	//Con::errorf("loaded action: numactions %d  numBodies %d",mNumActions,mNumBodies);
	if (mNumBodies) return true;
	else return false;
}

void gaActionSet::loadActionSetAsText(const char *filename)
{
	int i,j,id,training,numActions,numParts,found,temp,slices;
	
	float start,duration,x,y,z;
	char buf[255];	

	//for (i=0;i<GA_MAX_ACTIONS;i++) if (mActions[i]) delete mActions[i];

	FILE *fp = fopen(filename,"r");

	numParts = mNumBodies; 
	//numActions = mNumActions;

	fgets(buf,255,fp);
	sscanf(buf,"numActions: %d;",&temp);//number doesn't matter, count lines instead
	fgets(buf,255,fp);
	sscanf(buf,"numSlices: %d;",&slices);//number will matter, set mNumTimeSlices here
	mNumSlices = slices;

	while (fgets(buf,255,fp)) {
		if ((buf[0]=='#')||(buf[0]==' ')||(buf[0]=='\n')||(buf[0]=='\t')) continue;
		sscanf(buf,"%d;%d;%f;%f;(%f,%f,%f);",&id,&training,&start,&duration,&x,&y,&z);
		mActions[mNumActions] = new gaAction();
		//found=0;
		//for (j=0;j<numParts;j++) if (parts[j]==id) found=1;
		//if (!found) {
		//	mAU->mActiveBodyIndices[numParts]=id;//?
		//	parts[numParts++]=id;
		//	mNumBodies++;
		//}

		mActions[mNumActions]->mBodyIndex = id;
		mActions[mNumActions]->mIsTraining = training;
		mActions[mNumActions]->mStart = start;
		mActions[mNumActions]->mDuration = duration;
		mActions[mNumActions]->mForces = Ogre::Vector3(x,y,z);
		mActions[mNumActions]->setSteps(mNumSteps);

		//numActions++;
		mNumActions++;
	}
	//mNumBodies = numParts;
	//mNumActions = numActions;
	//mAU->mNumActiveBodies = mNumBodies;
}


void gaActionSet::set(gaActionSet *A)
{
	int i;

	mAU = A->mAU;

	mNumBodies = A->mNumBodies;
	mNumSlices = A->mNumSlices;
	mDuration = A->mDuration;
	mNumSteps = A->mNumSteps;

	mCurrAction = 0;
	mCurrTime = 0.0;
	mDuration = A->mDuration;

	//save these in case we're sorting the list and still need them (?)
	//mScore = A->mScore;
	//mSOA = A->mSOA;
	//mSMA = A->mSMA;

	mWeight = A->mWeight;
	mBodyGroup = A->mBodyGroup;

	//now copy all the actions
	if (mNumActions>0) {
		for (i=0;i<mNumActions;i++) delete mActions[i];
		mNumActions=0;
	}

	for (i=0;i<A->mNumActions;i++) addAction(A->mActions[i]);
	
}

void gaActionSet::addAction(gaAction *new_action)
{
	mActions[mNumActions++] = new gaAction(new_action);
}

void gaActionSet::removeAction(int id)
{
	int i;
	for (i=id;i<(mNumActions-1);i++) {
		mActions[i]->set(mActions[i+1]);
	}
	delete mActions[mNumActions--];
}

void gaActionSet::addRandomActions()
{
	//HERE: replicate the timeslice system.
	//add num_slices * num_bodyparts actions

	int i,j,k;
	//printf("adding random actions! parts %d slices %d\n",numBodyparts,numSlices);
	Ogre::Vector3 work; work = Ogre::Vector3::ZERO;

	for (i=0;i<mNumBodies;i++) {
		for (j=0;j<mNumSlices;j++) {
			//int id;

			//if (gRandom.randF()<GA_MUTATE_CHANCE) work = Ogre::Vector3(gRandom.randF(-1.0,1.0), gRandom.randF(-1.0,1.0), 0.0);
			//else work = Ogre::Vector3(0,0,0);
			work = Ogre::Vector3(0,0,0);//Ogre::Vector3(gRandom.randF(-1.0,1.0), gRandom.randF(-1.0,1.0), 0.0);
			//Con::errorf("set up random action: %f %f",work.x,work.y);

			//HERE: instead of i, you need mActiveBodyIndices[i], which stores the index of each active bodypart
			//since you are no longer dealing with all the bodyparts in the body.
			//mActions[mNumActions] = new gaAction(i,GA_NUM_SEQUENCE_STEPS,(float)j/(float)mNumSlices,1.0/(float)mNumSlices,work);
			mActions[mNumActions] = new gaAction(mAU->mActiveBodyIndices[i],mNumSteps,
				(float)j/(float)mNumSlices,1.0/(float)mNumSlices,work);
			mActions[mNumActions++]->setSteps(mNumSteps);
			//Con::printf("added random action: %3.2f %3.2f %3.2f",work.x,work.y,work.z);
		}
	}
}

void gaActionSet::setRandomActions()
{
	int i,j;
	//printf("setting random actions!\n");
	Ogre::Vector3 work; work = Ogre::Vector3::ZERO;
	for (i=0;i<mNumBodies;i++) {
		for (j=0;j<mNumSlices;j++) {
			work = Ogre::Vector3(0,0,0);//(gRandom.randF(-1.0,1.0), gRandom.randF(-1.0,1.0),0);//gRandom.randF(-1.0,1.0));
			mActions[(j*mNumBodies)+i]->set(i,mNumSteps,(float)j/(float)mNumSlices,1.0/(float)mNumSlices,work);
		}
	}
}

void gaActionSet::addRotateActions(int curSet,int numSets)
{
	////HERE: replicate the timeslice system.
	////add num_slices * num_bodyparts actions

	//int i,j,k;
	////printf("adding random actions! parts %d slices %d\n",numBodyparts,numSlices);
	//float angle;
	//Ogre::Vector3 start,work; 
	//work = Ogre::Vector3::ZERO;
	//start = Ogre::Vector3(1,0,0);

	//angle = (((float)curSet/(float)numSets)*(360.0));
	//Ogre::Matrix3 mat;
	//mat.set(Ogre::Vector3(0,0,angle));
	//mat.mulP(start,&work);
 //   //Con::printf("adding rotate actions! set %d, forces %f %f %f",curSet,work.x,work.y,work.z);

	//for (i=0;i<mNumBodies;i++) {
	//	for (j=0;j<mNumSlices;j++) {
	//		//work.set(gRandom.randF(-1.0,1.0), gRandom.randF(-1.0,1.0), 0.0);

	//		mActions[mNumActions] = new gaAction(i,mNumSteps,(float)j/(float)mNumSlices,1.0/(float)mNumSlices,work);
	//		mActions[mNumActions++]->setSteps(mNumSteps);
	//		//Con::printf("added random action: %3.2f %3.2f %3.2f",work.x,work.y,work.z);
	//	}
	//}
}

void gaActionSet::setRotateActions(int curSet,int numSets)
{
	////Here: take a stock action force (1,0,0) and rotate it around (0,0,1) 
	////by ((curSet/numSets)*(M_PI*2)), i.e. full circle.
	//int i,j;
	//float angle;
	//Ogre::Vector3 start,work;
	//work = Ogre::Vector3::ZERO;
	//start = Ogre::Vector3(1,0,0);

	//angle = (((float)curSet/(float)numSets)*(360.0));
	//Ogre::Matrix3 mat;
	//mat.set(Ogre::Vector3(0,0,angle));
	//mat.mulP(start,&work);
	//
	////Con::printf("setting rotate actions! set %d, forces %f %f %f",curSet,work.x,work.y,work.z);
	//for (i=0;i<mNumBodies;i++) {
	//	for (j=0;j<mNumSlices;j++) {
	//		//work = Ogre::Vector3(gRandom.randF(-1.0,1.0), gRandom.randF(-1.0,1.0),0.0);
	//		mActions[(j*mNumBodies)+i]->set(i,mNumSteps,(float)j/(float)mNumSlices,1.0/(float)mNumSlices,work);
	//	}
	//}
}


void gaActionSet::addZeroActions(int num_parts,physRigidBody *bodies_)//const
{
	int i,j;
	Ogre::Vector3 work; work = Ogre::Vector3::ZERO;


	mNumBodies = num_parts;
	//for (i=0;i<mNumBodies;i++) {
	//	*mBodies[i] = bodies_[i];
	//}

	for (i=0;i<mNumBodies;i++) {
		for (j=0;j<mNumSlices;j++) {
			mActions[mNumActions++] = new gaAction(i,mNumSteps,(float)j/(float)mNumSlices,1.0/(float)mNumSlices,work);
		}
	}

	//delete work;
}

void gaActionSet::setZeroActions()//const?
{
	int i,j;
	Ogre::Vector3 work; work = Ogre::Vector3::ZERO;

	if (mNumActions>0) {
		for (i=0;i<mNumBodies;i++) {
			for (j=0;j<mNumSlices;j++) {
				mActions[(i*mNumSlices)+j]->set(i,mNumSteps,(float)j/(float)mNumSlices,1.0/(float)mNumSlices,work);	
			}
		} 
	}// else printf("****  OOPS -- trying to set empty actions!!! ***\n");
}

//void gaActionSet::setZeroActionForces();

void gaActionSet::mateSets(gaActionSet *A,gaActionSet *B,int setID)
{//HERE given two sets, make this one a random combination of the two
	//go through the whole action set, and each time flip a coin for 
	//which parent to grab it from.
	int i,j;

	Ogre::Vector3 work; work = Ogre::Vector3::ZERO;

	if (!A->mNumActions) {
		set(B);
		return;
	} else if (!B->mNumActions) {
		set(A);
		return;
	} else {
		for (i=0;i<mNumBodies;i++) {
			for (j=0;j<mNumSlices;j++) {
				if (1)//(gRandom.randF()>0.5) 
					work = A->mActions[(i*mNumSlices)+j]->mForces;
				else 
					work = B->mActions[(i*mNumSlices)+j]->mForces;
				mActions[(i*mNumSlices)+j]->set(mAU->mActiveBodyIndices[i],mNumSteps,(float)j/(float)mNumSlices,1.0/(float)mNumSlices,work);
			}
		}
	}
}

void gaActionSet::mutate()
{
	//int i,j;
	//float x,y,z,xm,ym,zm,mutate_chance,mutate_amount;
	//Ogre::Vector3 work; work = Ogre::Vector3::ZERO;
	//if (mAU->mCurrGen==0) 
	//{
	//	mutate_chance = 1.0;//increase chance & amount of mutation on first gen.
	//	mutate_amount = mAU->mMutationAmount * 2.0;
	//} else {
	//	mutate_chance = mAU->mMutationChance;
	//	mutate_amount = mAU->mMutationAmount;
	//}
	//for (i=0;i<mNumBodies;i++) {
	//	for (j=0;j<mNumSlices;j++) {
	//		if (mActions[(j*mNumBodies)+i]->mIsTraining) 
	//		{
	//			if (gRandom.randF()<mutate_chance) 
	//			{	//Ogre::Vector3 before = mActions[(j*mNumBodies)+i]->mForces;		
	//				//x = 0.0;
	//				xm = (gRandom.randF(0.0,mutate_amount) + gRandom.randF(0.0,mutate_amount)) - mutate_amount;
	//				x = xm + mActions[(j*mNumBodies)+i]->mForces.x;
	//				if (x > M_PI) x = M_PI;
	//				else if (x < -M_PI) x = -M_PI;

	//				ym = (gRandom.randF(0.0,mutate_amount) + gRandom.randF(0.0,mutate_amount)) - mutate_amount;
	//				y = ym + mActions[(j*mNumBodies)+i]->mForces.y;
	//				if (y > M_PI) y = M_PI;
	//				else if (y < -M_PI) y = -M_PI;

	//				zm = (gRandom.randF(0.0,mutate_amount) + gRandom.randF(0.0,mutate_amount)) - mutate_amount;
	//				z = zm + mActions[(j*mNumBodies)+i]->mForces.z;
	//				if (z > M_PI) z = M_PI;//1.0;//arbitrary! should go to M_PI, 180 degrees
	//				else if (z < -M_PI) z = -M_PI;
	//				
	//				work = Ogre::Vector3(x,y,z);//(x,y,0.0) - UH OH - originally designed this in a system where Z was
	//				//always twist axis.  Can't do it that way anymore.  Would be nice to be able to specify 
	//				//for each bodypart which axis was twist and whether or not to include it in GA.
	//				mActions[(j*mNumBodies)+i]->mForces = work;
	//				//Con::printf("mutating action for bodypart %d: X: %f, Y: %f, Z: %f",mActions[(j*mNumBodies)+i]->mBodyIndex,work.x,work.y,work.z);
	//			}
	//		}
	//	}
	//}
}

void gaActionSet::mutate(float scale)
{
	//int i,j;
	//float x,y,z,xm,ym,zm;
	//Ogre::Vector3 work; work = Ogre::Vector3::ZERO;

	//for (i=0;i<mNumBodies;i++) {
	//	for (j=0;j<mNumSlices;j++) {
	//		if (mActions[(j*mNumBodies)+i]->mIsTraining) 
	//		{
	//			if (gRandom.randF()<(mAU->mMutationChance)) {

	//				xm = (gRandom.randF(0.0,mAU->mMutationAmount) + gRandom.randF(0.0,mAU->mMutationAmount)) - mAU->mMutationAmount;
	//				xm *= scale;
	//				x = xm + mActions[(j*mNumBodies)+i]->mForces.x;
	//				if (x > 1.0) x = 1.0;
	//				else if (x < -1.0) x = -1.0;

	//				//ym = gRandom.randF(0.0,mAU->mMutationAmount*2)-mAU->mMutationAmount;
	//				ym = (gRandom.randF(0.0,mAU->mMutationAmount)+gRandom.randF(0.0,mAU->mMutationAmount)) - mAU->mMutationAmount;
	//				ym *= scale;
	//				y = ym + mActions[(j*mNumBodies)+i]->mForces.y;
	//				if (y > 1.0) y = 1.0;
	//				else if (y < -1.0) y = -1.0;

	//				work = Ogre::Vector3(x,y,0.0);

	//				mActions[(j*mNumBodies)+i]->mForces = work;

	//			}
	//		}
	//	}
	//}
}

void gaActionSet::setScore(float s)
{
	mScore = s;
}


float gaActionSet::avgForces()
{
	int i;

	return sumForces()/mNumActions;
}

float gaActionSet::sumForces()
{
	int i;
	float f = 0.0;

	for (i=0;i<mNumActions;i++) {
		f += mActions[i]->mForces.length();
	}
	return f;
}

//////////////////////////////////////////////////
////   g a A c t i o n S e t G r o u p       /////

gaActionGroup::gaActionGroup()
{
	init(10);
}

gaActionGroup::gaActionGroup(int num_sets)
{
	init(num_sets);
}

gaActionGroup::gaActionGroup(int num_bodies,int num_sets,gaActionUser *au)
{
	mAU = au;
	init(num_bodies,num_sets);
}

gaActionGroup::gaActionGroup(char *filename)
{
	//load(filename);
}

gaActionGroup::~gaActionGroup()
{

}

void gaActionGroup::init(int num_sets)
{
	int i;
	mNumSets = num_sets;
	mCurrSet = 0;
	mBestSet = NULL;

	for (i=0;i<GA_MAX_ACTION_SETS;i++) 
		mActionSets[i] = NULL; 
	
}

void gaActionGroup::init(int num_bodies,int num_sets)
{
	int i;
	mNumBodies = num_bodies;
	mNumSets = num_sets;
	mNumIntermediates = 0;
	mCurrSet = 0;
	mBestSet = new gaActionSet(mNumBodies,mAU);

	for (i=0;i<GA_MAX_ACTION_SETS;i++) mActionSets[i] = NULL;

	for (i=0;i<num_sets;i++) 
		mActionSets[i] = new gaActionSet(mNumBodies,mAU); 
	
}


void gaActionGroup::clear()
{
	
	mNumSets = 0;
	mNumBodies = 0;
	mNumIntermediates = 0;
	mCurrSet = 0;

	for (int i=0;i<mNumSets;i++) 
	{
		if (mActionSets[i]) delete mActionSets[i];//mActionSets[i]->clear();
	}

	delete mBestSet;
}


void gaActionGroup::sort()
{//IMPORTANT: This function assumes gaObservationSequenceSet::sort() has already been 
	//called for this crop of competitors.  It uses global sort_list[] array from there.
	int i,j;//NOTE: does this get called at all, now?  11-24-07

	gaActionSet *action_set = new gaActionSet();
	gaActionSet *set_copies[GA_MAX_ACTION_SETS];

	for (i=0;i<mNumSets;i++) {
		set_copies[i] = new gaActionSet(mActionSets[i]);
	}
	for (i=0;i<mNumSets;i++) {
		mActionSets[i]->set(set_copies[sort_list[i]]);
	}

	//delete seq;
	for (i=0;i<mNumSets;i++) delete set_copies[i];

	//printf("finished gactionGroup->sort.  Best action: %d \n",sort_list[0]);
}

void gaActionGroup::repopulate()//(char *config)
{
	//int i,j,k,best,p1,p2;
	//float sum,avg,max;
	////char filename[50];
	//gaActionSet *a_set = new gaActionSet();

	//sum = 0.0; max = -999999.0;

	////save(filename);

	//// A) find best score, and the average.
	//for (i=0;i<mNumSets;i++) {
	//	sum += mActionSets[i]->mScore;
	//	if (mActionSets[i]->mScore > max) { 
	//		best = i;
	//		max = mActionSets[i]->mScore;
	//	}
	//}
	//avg = sum/mNumSets;
	////Con::errorf("                                 avg %3.2f \n\n",avg);
	//mNumIntermediates = 0;

	//// B) set score-over-average for everybody.
	//for (i=0;i<mNumSets;i++)
	//{
	//	if (avg>0) 
	//		mActionSets[i]->mSOA = mActionSets[i]->mScore/avg;
	//	else if (avg<0) 
	//	{
	//		if (mActionSets[i]->mScore>0)
	//			mActionSets[i]->mSOA = 3.0;//Arbitrary but in this case we just need  
	//									//to encourage anyone who's clearing zero.
	//		else mActionSets[i]->mSOA = 0.01;//and highly discourage the rest.
	//	}

	//	else mActionSets[i]->mSOA = 1.0;
	//	//Con::errorf("actionset %d Score Over Average: %f",i,mActionSets[i]->mSOA);
	//}

	//// C) fill up mIntermediates, using score-over-average to determine frequency 
	//// for each competitor in this round of actionSets.
	//unsigned int lastInter;
	//for (i=0;i<mNumSets;i++)
	//{
	//	//(First, add one intermediate for every full point in the SOA. 
	//	// 2.7 equals at least two intermediates.)
	//	if (mActionSets[i]->mSOA >= 1.0) 
	//	{      
	//		for (j=0;j<(int)(mActionSets[i]->mSOA);j++) 
	//		{
	//			//Con::errorf("adding an intermediate: %d, set %d,actions %d",mNumIntermediates,i,mActionSets[i]->mNumActions);
	//			if (mActionSets[i]->mNumActions == 0)
	//			{
	//				//Con::errorf("got zero actions!!");
	//				//PROBLEM: somehow my numActions and my mActions for the first actionset 
	//				//got fragged. Have to keep looking till I find a valid actionset, till 
	//				//we can properly fix this.  [Except now this is totally broken too. Rewrite.]
	//				//unsigned int count;
	//				//count = i+1;
	//				//if (count>=mNumSets) return;//? Temp, debugging 06-25-09
	//				//while (mActionSets[count]->mNumActions == 0) count++;	
	//				//mIntermediates[mNumIntermediates++] = new gaActionSet(mActionSets[count]);
	//				////(hope that works.)
	//			} else {
	//				mIntermediates[mNumIntermediates++] = new gaActionSet(mActionSets[i]);
	//			}
	//			lastInter = i;
	//			//Con::errorf("grabbing %d for an intermediate, soa = %f",i,mActionSets[i]->mSOA);
	//			//mIntermediates[mNumIntermediates++] = new gaActionSet(mActionSets[i]);
	//			//HACK: padding with one extra here, to ensure that there are ALWAYS 
	//			//at least two intermediates.  Crashes when there is only one.
	//		}
	//	}
	//	//(Second, have the chance of one more intermediate equal to the remainder.
	//	// 2.7 equals two for sure, plus 0.7 chance of a third.)
	//	if (1)//(gRandom.randF() < (mActionSets[i]->mSOA - (int)(mActionSets[i]->mSOA))) 
	//		mIntermediates[mNumIntermediates++] = new gaActionSet(mActionSets[i]);
	//}

	//if (mNumIntermediates==0) 
	//{//solve the problem of one intermediate, ONLY if it comes up.
	//	mIntermediates[mNumIntermediates++] = new gaActionSet(mActionSets[lastInter]);
	//	//Con::errorf("creating an intermediate, had none!");
	//}
	//if (mNumIntermediates==1) 
	//{//solve the problem of one intermediate, ONLY if it comes up.
	//	mIntermediates[mNumIntermediates++] = new gaActionSet(mActionSets[lastInter]);
	//	//Con::errorf("duplicating an intermediate, had only one!");
	//}


	//sum = 0.0;
	//for (i=0;i<mNumSets;i++) {
	//	mActionSets[i]->mSMA = fabs(mActionSets[i]->mScore - avg);
	//	sum += mActionSets[i]->mSMA;
	//}//hmm, what was I using this for?
	//if (mIntermediates[0]->mNumActions==0)	
	//{
	//	//Con::printf("got a null intermediate!");
	//	mIntermediates[0]->set(mIntermediates[1]);//seems to only happen to [0]?
	//}
	//for (i=0;i<mNumSets;i++) {
	//	if (i==0) {//keep the best one from last time
	//		//if (best > 0) actionSets[i]->set(actionSets[best]);
	//		if (sort_list[0] != 0)//not happy to copy from itself.
	//			mActionSets[i]->set(mActionSets[sort_list[0]]);
	//		//printf("copying actionSets[%d]\n",sort_list[0]);
	//		//actionSets[i]->set(actionSets[i]);
	//		//Con::printf("grabbing the best one: %d score %1.2f",sort_list[0],mActionSets[sort_list[0]]->mScore);
	//	} else {
	//		//first, select two parents
	//		if (mNumIntermediates==2)
	//		{
	//			p1 = 0;
	//			p2 = 1;
	//		} else {
	//			p1 = (int)(gRandom.randI(0,mNumIntermediates-1));
	//			p2 = (int)(gRandom.randI(0,mNumIntermediates-1));
	//		//HERE is why you have to guarantee at least two intermediates
	//			while (p2==p1) p2 = (int)(gRandom.randI(0,mNumIntermediates-2));
	//		}
	//		//printf("%d-%d, p1->numActions %d,p2->numActions %d\n ",p1,p2,intermediates[p1]->numActions,intermediates[p2]->numActions);
	//		//just to make sure you don't pick the same one twice.

	//		if ((mIntermediates[p1]->mNumActions)&&(mIntermediates[p2]->mNumActions))
	//		{
	//			//Con::errorf("grabbing %d and %d for parents.",p1,p2);
	//			mActionSets[i]->mateSets(mIntermediates[p1],mIntermediates[p2],i);
	//			mActionSets[i]->mutate();
	//		} 
	//		else
	//		{
	//			mActionSets[i]->set(mActionSets[0]);
	//			mActionSets[i]->mutate();
	//		}
	//		
	//		for (unsigned int k=0;k<mActionSets[i]->mNumActions;k++)
	//		{
	//			//if (mActionSets[i]->mActions[k]->mForces.length()>0)
	//			//{
	//			//	Ogre::Vector3 kForces = mActionSets[i]->mActions[k]->mForces;
	//			//	Con::errorf("Bodypart %d motor target %f %f %f",
	//			//		mActionSets[i]->mActions[k]->mBodyIndex,
	//			//		kForces.x,kForces.y,kForces.z);
	//			//}
	//		}
	//	}
	//}
	//for (i=0;i<mNumIntermediates;i++) delete mIntermediates[i];

	//mNumIntermediates = 0;
	//sequence_count = 0;

}


void gaActionGroup::repopulateFromOne()
{
	//mActionSets[0] is the best or only one, from here I fill the rest of the sets with 
	//random mutations off that one.
	int i;
	//Con::errorf("repopulateFromOne: currGen %d",mAU->mCurrGen);
	for (i=1;i<mNumSets;i++) {
		//mActionSets[i]->set(mActionSets[0]);
		mActionSets[i]->set(mAU->mActionSet);
		mActionSets[i]->mutate();
		//Con::printf("repopulating actionset %d",i);
	}
}

//if ((actionSets[best]->score_minus_avg > (avg * 3))&&(actionSets[best]->dominant == 1)) {
//  if (actionSets[best]->dominant == 1) {
//printf("RESET!  Top score was dominating.\n");
//actionSets[i]->set(actionSets[best]);
//if (i>0) actionSets[i]->mutate(1.5);
//actionSets[best]->dominant = 0;
//  }
//} else if (actionSets[best]->score_minus_avg > (avg * 3)) {
//actionSets[best]->dominant = 1;
// } else if ((actionSets[best]->score_minus_avg - avg) < 2.4) {
//printf("RESET!  Scores are in a rut.\n");
//FIX -- need to mark a startpoint and observe for a few steps before taking radical action
//actionSets[i]->set(actionSets[i]);
//actionSets[i]->mutate(10.0);
//  } else {


void gaActionGroup::randomize()
{
	int i,j;

	for (i=0;i<mNumSets;i++) {
		//mActionSets[i]->mNumBodies = mNumBodies;//unnecessary?
		mActionSets[i]->addRandomActions();
		//mActionSets[i]->addRotateActions(i,mNumSets);
	}
}

void gaActionGroup::randomize(int num_bodies)
{
	int i,j,num_active_parts=0;
	//printf("randomizing: numSets %d\n",numSets);

	//assume that when we call randomize(), we want all parts 
	//except part[0] to be active.
	//this gets called from entity, like randomize(numBodyparts-1)

	for (i=0;i<mNumSets;i++) {
		mActionSets[i]->mNumBodies = num_bodies;
		//for (j=0;j<num_bodies;j++) {
			//mActionSets[i]->mBodies[j] = j+1;
		//}
		mActionSets[i]->addRandomActions();
	}

	//for (i=0;i<numSets;i++) {
	//  if (actionSets[i]->numActions==0) {
	//    actionSets[i]->numBodyparts = num_parts;
	//    for (j=0;j<num_parts;j++) {
	//	actionSets[i]->bodyparts[j] = j+1;
	//    }
	//    actionSets[i]->addRandomActions();
	//  } else actionSets[i]->setRandomActions();
	//}
}
void gaActionGroup::setNumBodies(int num_bodies)
{
	mNumBodies = num_bodies;
	for (unsigned int i=0;i<mNumSets;i++)
	{
		mActionSets[i]->mNumBodies = num_bodies;
	}
}


void gaActionGroup::immigrate(gaActionGroup *otherGroup)
{
	mActionSets[1]->set(otherGroup->mActionSets[0]);
}
///////////////////////////////////////////////////////////

//IMPLEMENT_CO_DATABLOCK_V1(gaFitnessData);


//IMPLEMENT_CONSOLETYPE(gaFitnessData)
//IMPLEMENT_SETDATATYPE(gaFitnessData)
//IMPLEMENT_GETDATATYPE(gaFitnessData)

//gaFitnessData::gaFitnessData()
//{
//	mPositionGoal = Ogre::Vector3::ZERO;
//	mPositionGoalType = Ogre::Vector3(0,0,0);
//	mRotationGoal = 0.0;
//	mRotationGoalType = 0;
//	mBodypartName = NULL;
//}
//
//gaFitnessData::~gaFitnessData()
//{
//}
//
//bool gaFitnessData::preload(bool bServer, Ogre::String &errorStr)
//{
//  if (!Parent::preload(bServer, errorStr))
//      return false;
//
//  return true;
//}
//
//bool gaFitnessData::onAdd()
//{
//   if(!Parent::onAdd())
//      return false;
//
//   return true;
//}
//
//void gaFitnessData::initPersistFields()
//{
//  Parent::initPersistFields();
//  
//  addField("PositionGoal", TypeOgre::Vector3,Offset(mPositionGoal, gaFitnessData));
//  addField("PositionGoalType", TypeOgre::Vector3,Offset(mPositionGoalType, gaFitnessData));
//  addField("RotationGoal", Typefloat,Offset(mRotationGoal, gaFitnessData));
//  addField("RotationGoalType", Typeint,Offset(mRotationGoalType, gaFitnessData));
//  addField("BodypartName", TypeString,Offset(mBodypartName, gaFitnessData));
//}
//
//void gaFitnessData::packData(BitStream* pBitStream)
//{
//   Parent::packData(pBitStream);
//
//   mathWrite(*pBitStream, mPositionGoal);
//   mathWrite(*pBitStream, mPositionGoalType);
//   pBitStream->write(mRotationGoal);
//   pBitStream->write(mRotationGoalType);
//   pBitStream->writeString(mBodypartName);
//
//}
//
//void gaFitnessData::unpackData(BitStream* pBitStream)
//{
//   Parent::unpackData(pBitStream);   
//
//   mathRead(*pBitStream, &mPositionGoal);
//   mathRead(*pBitStream, &mPositionGoalType);
//   pBitStream->read(&mRotationGoal);
//   pBitStream->read(&mRotationGoalType);
//   mBodypartName = pBitStream->readSTString();
//}
//
////////////////////////////////////////////////////////////
//IMPLEMENT_CO_DATABLOCK_V1(gaActionUserData);
//
//gaActionUserData::gaActionUserData()
//{
//	mMutationChance = 0.0;
//	mMutationAmount = 0.0;
//	mNumPopulations = 0;
//	mMigrateChance = 0.0;
//	//mNumSequenceSteps = 0;
//	mNumRestSteps = 0;
//	mObserveInterval = 0;
//	mNumActionSets = 0;
//	//mNumSlices = 0;
//	mNumSequenceReps = 0;
//	mActionName = NULL;
//
//	for (unsigned int i=0;i<GA_MAX_FITNESS_DATA;i++)
//	{
//		mFitnessDataID[i] = 0;
//		mFitnessData[i] = NULL;
//	}
//}
//
//gaActionUserData::~gaActionUserData()
//{
//}
//
//bool gaActionUserData::preload(bool bServer, Ogre::String &errorStr)
//{
//  if (!Parent::preload(bServer, errorStr))
//      return false;
//
//  if (!bServer) 
//   {
//	   for (unsigned int i=0;i<GA_MAX_FITNESS_DATA;i++)
//	   {
//		   if( !mFitnessData[i] && mFitnessDataID[i] != 0 )
//		   {
//			   if( Sim::findObject( mFitnessDataID[i], mFitnessData[i] ) == false)
//			   {
//				   //Con::errorf( ConsoleLogEntry::General, "fxFitnessData::preload: Invalid packet, bad datablockId(mFitnessData%d): 0x%x", i+1,mFitnessDataID[i] );
//			   }
//		   }
//	   }
//  }
//  return true;
//}
//
//bool gaActionUserData::onAdd()
//{
//   if(!Parent::onAdd())
//      return false;
//
//   return true;
//}
//
//void gaActionUserData::initPersistFields()
//{
//  Parent::initPersistFields();
//  
//  addField("MutationChance", Typefloat,Offset(mMutationChance, gaActionUserData));
//  addField("MutationAmount", Typefloat,Offset(mMutationAmount, gaActionUserData));
//  addField("NumPopulations", Typeint,Offset(mNumPopulations, gaActionUserData));
//  addField("MigrateChance", Typefloat,Offset(mMigrateChance, gaActionUserData));
//  //addField("NumSequenceSteps", Typeint,Offset(mNumSequenceSteps, gaActionUserData));
//  addField("NumRestSteps", Typeint,Offset(mNumRestSteps, gaActionUserData));
//  addField("ObserveInterval", Typeint,Offset(mObserveInterval, gaActionUserData));
//  addField("NumActionSets", Typeint,Offset(mNumActionSets, gaActionUserData));
//  //addField("NumSlices", Typeint,Offset(mNumSlices, gaActionUserData));
//  addField("NumSequenceReps", Typeint,Offset(mNumSequenceReps, gaActionUserData));
//  addField("ActionName", TypeString,Offset(mActionName, gaActionUserData));
//  addField("FitnessData1", TYPEID< gaFitnessData >(),Offset(mFitnessData[0], gaActionUserData));
//  addField("FitnessData2", TYPEID< gaFitnessData >(),Offset(mFitnessData[1], gaActionUserData));
//  addField("FitnessData3", TYPEID< gaFitnessData >(),Offset(mFitnessData[2], gaActionUserData));
//  addField("FitnessData4", TYPEID< gaFitnessData >(),Offset(mFitnessData[3], gaActionUserData));
//  addField("FitnessData5", TYPEID< gaFitnessData >(),Offset(mFitnessData[4], gaActionUserData));
//  addField("FitnessData6", TYPEID< gaFitnessData >(),Offset(mFitnessData[5], gaActionUserData));
//
//}
//
//void gaActionUserData::packData(BitStream* pBitStream)
//{
//   Parent::packData(pBitStream);
//
//   pBitStream->write(mMutationChance);
//   pBitStream->write(mMutationAmount);
//   pBitStream->write(mNumPopulations);
//   pBitStream->write(mMigrateChance);
//   //pBitStream->write(mNumSequenceSteps);
//   pBitStream->write(mNumRestSteps);
//   pBitStream->write(mObserveInterval);
//   pBitStream->write(mNumActionSets);
//   //pBitStream->write(mNumSlices);
//   pBitStream->write(mNumSequenceReps);
//   pBitStream->writeString(mActionName);
//
//   for (unsigned int i=0;i<GA_MAX_FITNESS_DATA;i++)
//   {
//	   if( pBitStream->writeFlag( mFitnessData[i] != NULL ) )
//	   {
//		   pBitStream->writeRangedunsigned int(packed? SimObjectId(mFitnessData[i]):
//			   mFitnessData[i]->getId(),DataBlockObjectIdFirst,DataBlockObjectIdLast);
//	   }
//   }
//}
//
//void gaActionUserData::unpackData(BitStream* pBitStream)
//{
//   Parent::unpackData(pBitStream);
//
//   pBitStream->read(&mMutationChance);
//   pBitStream->read(&mMutationAmount);
//   pBitStream->read(&mNumPopulations);
//   pBitStream->read(&mMigrateChance);
//   //pBitStream->read(&mNumSequenceSteps);
//   pBitStream->read(&mNumRestSteps);
//   pBitStream->read(&mObserveInterval);
//   pBitStream->read(&mNumActionSets);
//   //pBitStream->read(&mNumSlices);
//   pBitStream->read(&mNumSequenceReps);
//   mActionName = pBitStream->readSTString();
//   for (unsigned int i=0;i<GA_MAX_FITNESS_DATA;i++)
//   {   
//	   if( pBitStream->readFlag() )
//	   {
//		   mFitnessDataID[i] = pBitStream->readRangedunsigned int( DataBlockObjectIdFirst, DataBlockObjectIdLast );
//	   }
//   }
//}
//
////////////////////////////////////////////////////////////
//IMPLEMENT_CO_NETOBJECT_V1(gaActionUser);

gaActionUser::gaActionUser()//fxFlexBody *fb
{
	mNumBodies = 0;
	mNumActiveBodies = 0;
	mNumBodyGroups = 0;
	mNumSets = GA_NUM_ACTION_SETS;
	mNumPops = 1;
	mCurrPop = 0;
	mSimStep = 0;
	mCurrGen = 0;
	mDone = 0;
	
	mIsResetting = false;

	mActionState     = GA_ACTION_NONE;
	mHeadActionState = GA_ACTION_NONE;
	mTailActionState = GA_ACTION_NONE;
	mGoalState       = GA_GOAL_NONE;

	mHeadIndex = -1;
	mNeckIndex = -1;
	mBodyIndex = -1;
	mRightFrontIndex = -1;
	mLeftFrontIndex = -1;
	mRightBackIndex = -1;
	mLeftBackIndex = -1;

	mMutationChance = 0.0;//GA_MUTATE_CHANCE;
	mMutationAmount = 0.0;//GA_MUTATE_AMOUNT;

	//mNumSequenceSteps  = GA_NUM_SEQUENCE_STEPS;
	//mNumRestSteps      = GA_NUM_REST_STEPS;
	//mObserveInterval   = GA_OBSERVE_INTERVAL;
	//mNumTimeSlices     = GA_NUM_SLICES;
	//mNumSeqReps        = GA_NUM_SEQ_REPS;
	mCurrSeqRep        = 0;
	mNumObsSets        = 1;

	mResetForce = 2.0;
	mActionForce = 2.0;
	mForwardForce = 0.0;

	mRandSeed = 98;//FIX -- expose this to script
	mForwardSteps = 100;//TEMP

	mFlexBody = NULL;
	mShapeName = "";
	mCurrAction = "";

	for (int i=0;i<GA_MAX_BODIES;i++)
	{
		mBodies[i] = NULL; 
		mActiveBodies[i] = NULL;
		mActiveBodyIndices[i] = -1;
		mBodyInitialPositions[i] = Ogre::Vector3::ZERO;
		mBodyInitialQuats[i] = Ogre::Quaternion::IDENTITY;
	}

	mActionSet = NULL;
	mObsUnit = NULL;
	mObsSeq = NULL;
	for (int i=0;i<GA_MAX_OBS_SETS;i++)
		mObsSets[i] = NULL;
	for (int i=0;i<GA_MAX_POPS;i++)
		mActionGroups[i] = NULL;
	
	//gRandom.setSeed(mRandSeed);
}

gaActionUser::~gaActionUser()
{

}

/////////////////////////////////////


//void gaActionUser::initPersistFields()
//{
//	Parent::initPersistFields();
//    //addField("NumParticlesX", Typeint, Offset(m, gaActionUser));
//
//}

//------------------------------------------------------------------------------
//
//bool gaActionUser::onAdd()
//{
//	if (Parent::onAdd())
//		return true;
//
//	addToScene();
//
//	return false;
//}
//
//void gaActionUser::onRemove()
//{
//	Parent::onRemove();
//}

//------------------------------------------------------------------------------
//bool gaActionUser::onNewDataBlock(GameBaseData *pGameBaseData, bool reload)
//{
//  mDataBlock = dynamic_cast<gaActionUserData*>(pGameBaseData);
//  if (!mDataBlock || !Parent::onNewDataBlock(pGameBaseData, reload))
//    {
//      return false;
//    }
//
//  // Have parent class do the rest
//  scriptOnNewDataBlock();
//
//  //misuse of datablock?  I want to have the value local to actionuser,
//  //so it can be changed over time.
//  mMutationAmount = mDataBlock->mMutationAmount;
//  mMutationChance = mDataBlock->mMutationChance;
//  mNumPops = mDataBlock->mNumPopulations;//any reason to keep this local to gaActionUser?
//
//  return true;
//}


//unsigned int gaActionUser::packUpdate(NetConnection * con, unsigned int mask, BitStream * stream)
//{
//	// Pack Parent.
//	unsigned int retMask = Parent::packUpdate(con, mask, stream);
//
//	return(retMask);
//}
//
////------------------------------------------------------------------------------
//
//void gaActionUser::unpackUpdate(NetConnection * con, BitStream * stream)
//{
//	// Unpack Parent.
//	Parent::unpackUpdate(con, stream);
//
//}
/////////////////////////////////////

void gaActionUser::setup()
{
	//IMPORTANT: can't call setup() until AFTER you call addBody for all the relevant bodies, 
	//else it won't know how many bodies it has to deal with.

	//mShapeName = Ogre::StringTable->insert(strrchr(mFlexBody->mShapeName,'.'));

	//if (mFlexBody) mShapeName = mFlexBody->mShapeName;//StringTable->insert(mFlexBody->mShapeName);
	////Con::errorf("action user %s setting up! numPops %d",mShapeName,mDataBlock->mNumPopulations);

	////mActionSet = new gaActionSet(mDataBlock->mNumSlices,mDataBlock->mNumSequenceStepsmNumActiveBodies,this);
	//mActionSet = new gaActionSet(mNumActiveBodies,this);

	//mObsUnit = new gaObservationUnit();
	//mObsSeq = new gaObservationSequence(mDataBlock->mObserveInterval);
	//
	//for (unsigned int i=0;i<mDataBlock->mNumPopulations;i++)
	//{
	//	mActionGroups[i] = new gaActionGroup(mNumActiveBodies,mDataBlock->mNumActionSets,this);
	//	//mActionGroups[i]->randomize();//FIX: Do this if you didn't load an action in think().
	//	mObsSets[i] = new gaObservationSequenceSet(mDataBlock->mNumActionSets,mDataBlock->mObserveInterval);
	//}
	//
	//if (mDataBlock->mFitnessData[0]!=NULL)
	//	mGoalState = GA_GOAL_FITNESS_TRAINING;
	//else if (!strncmp(mDataBlock->mActionName,"sequence",8))
	//	mGoalState = GA_GOAL_PLAY_SEQUENCE;
	//else
	//	mGoalState = GA_GOAL_SINGLE_ACTION;
	//	

	//if (strlen(mDataBlock->mActionName)>0)
	//	loadAction(mDataBlock->mActionName);
	//else // can't do anything without an action, even all bodyparts random needs an action file.
	//	mGoalState = GA_GOAL_NONE;

	//return;
}

void gaActionUser::addBody(physRigidBody *rb)
{
	mBodies[mNumBodies++] = rb;
	return;
}

void gaActionUser::addActiveBody(physRigidBody *rb, int id)
{
	mActiveBodies[mNumActiveBodies] = rb;
	mActiveBodyIndices[mNumActiveBodies++] = id;
	return;
}

////////////////////////////////////////////////  THINK  //////////////
void gaActionUser::think()
{
	//char filename[255];
	//int i,m,s_m,id;//done=0;
	//float f;
	//Ogre::Vector3 work; work = Ogre::Vector3::ZERO;

	////////////////
	////Con::executef(mFlexBody,"onThink", Con::getIntArg(mFlexBody->getId()));
	////  Okay, cool, executef to onThink in script is working... now taking it out until 
	////we need it.  Should have a script think rate property of flexbody, exposed to script,
	////which determines how often we call out, probably don't want to do this once per tick
	////for everybody.  And don't do it at all if you don't have a script think process running.
	////////////////

	////Next step:  have actionState and goalState determine the reset mode and the 
	////choice of actions.  
	////if (actionstate==none) then start action state resetting, goal standing
	////if state resetting is done then do action state standing
	////if state standing is done then do action holding, goal standing is met
	////then change goals - walking forward, turning, rearing, jumping forward, ...
	////Note: change moveTargets gradually over time, rather than instantly.
	////if (mSimStep%30==0) Con::errorf("GA thinking: goal state %d, action state %d",mGoalState,mActionState);
	////FIRST: opt out if we have no goal
	//if (mGoalState==GA_GOAL_NONE) 
	//	return;
	////SECOND: opt out if we are asking the entity to do one action indefinitely
	//else if (mGoalState==GA_GOAL_SINGLE_ACTION) 
	//{
	//	thinkAction();
	//	return;
	//} 
	//else if (mGoalState==GA_GOAL_PLAY_SEQUENCE) 
	//{
	//	thinkSequence();
	//	return;
	//} 
	//else if (mGoalState==GA_GOAL_FITNESS_TRAINING) 
	//{ 
	//	if (mActionState==GA_ACTION_NONE) ///////////////////////////////////////////////////////
	//	{
	//		Ogre::Quaternion q;
	//		q = Ogre::Quaternion::IDENTITY;

	//		motorize();

	//		//Con::printf("GA starting fitness training.");
	//		for (i=1;i<mNumBodies;i++) {//0 has no joint
	//			physJoint *kJoint = mBodies[i]->getJoint();
	//			//Here: this is where you set the "rubber-man" spring-to-default-pose effect.
	//			//if (kJoint) kJoint->setMotorSpring(q,mResetForce);
	//			//Comment it out if you don't want that.
	//		}

	//		for (i=0;i<mNumBodies;i++) {
	//			mBodyInitialPositions[i] = mBodies[i]->getLinearPosition();
	//			mBodyInitialQuats[i] = mBodies[i]->getAngularPosition();	
	//			//Con::errorf("initial pos: %f %f %f",mBodyInitialPositions[i].x,mBodyInitialPositions[i].y,mBodyInitialPositions[i].z);
	//		}

	//		mActionState = GA_ACTION_START;
	//		//mActionState = GA_ACTION_BODY_STAND_UP;
	//		//mGoalState = GA_GOAL_STARTING;
	//		mSimStep = 0;//Gotta take a break here and let the world do a step, while we 
	//	}		
	//	else if (mActionState==GA_ACTION_START) //////////////////////////////////////////////
	//	{
	//		if (mSimStep < mDataBlock->mNumRestSteps) 
	//		{//First, do a reset to identity pose, and pause for bouncing to die down.
	//			Ogre::Quaternion q;
	//			q = Ogre::Quaternion::IDENTITY;
	//			for (i=1;i<mNumBodies;i++) //(0 has no joint)
	//			{
	//				physJoint *kJoint = mBodies[i]->getJoint();
	//				//More rubber-man effect here, comment out if undesirable.
	//				if (kJoint) kJoint->setMotorTarget(q);//,mResetForce);
	//			}
	//		}
	//		else if (mSimStep == mDataBlock->mNumRestSteps) 
	//		{//Then, record initial positions and start action.
	//			mActionState = GA_ACTION_ACT;
	//			mSimStep = 0;

	//			//for (i=0;i<mNumBodies;i++)
	//			//{
	//			//	mBodyStartPositions[i] = mBodies[i]->getLinearPosition();	
	//			//	mBodyStartQuats[i] = mBodies[i]->getAngularPosition();	
	//			//}

	//		}
	//	}
	//	else if (mActionState==GA_ACTION_RESET) //////////////////////////////////////////////
	//	{
	//		mIsResetting = true;
	//		if ((mSimStep==1)&&(mBodyInitialPositions[0].length()))//mBodyStartPositions//
	//		{
	//			//Okay, **** this, try setting whole body kinematic.
	//			//Except, whoops, now taking this back out, because of partial body ragdoll.
	//			//Some of our bodyparts might 
	//			//mFlexBody->setKinematic();

	//			//mFlexBody->setPosition(mBodyInitialPositions[0]);
	//			for (i=0;i<mNumBodies;i++) 
	//			{
	//				//  TEMP, having inertia problems, reset is flinging me exponentially farther every time...
	//				mBodies[i]->setLinearPosition(mBodyInitialPositions[i]);
	//				mBodies[i]->setAngularPosition(mBodyInitialQuats[i]);
	//				mBodies[i]->updatePositionToActor();
	//				//mBodies[i]->setLinearVelocity(Ogre::Vector3(0,0,0));
	//				//mBodies[i]->setAngularVelocity(Ogre::Vector3(0,0,0));
	//				//mBodies[i]->updateVelocityToActor();//TEMP
	//				Ogre::Quaternion q; q = Ogre::Quaternion::IDENTITY;
	//				mBodies[i]->getJoint()->setNewTarget(q);
	//			}
	//			mIsResetting = true;
	//			//Ecstasy - trying to restore reset, rigid body actors are not resetting... ?
	//			//mFlexBody->mReset = true;
	//			//mFlexBody->mCurrPosition = mBodyInitialPositions[0];//mFlexBody->mBodyParts[0]->mCurrPosition;
	//			//for (unsigned int i=0;i<mFlexBody->mNumBodyParts;i++)
	//			//	mFlexBody->mBodyParts[i]->reset();
	//	
	//			//mFlexBody->updateNodes();
	//		}
	//		else if (mSimStep < mDataBlock->mNumRestSteps) 
	//		{
	//			//Con::errorf("flexbody position %f %f %f",mFlexBody->mCurrPosition.x,mFlexBody->mCurrPosition.y,mFlexBody->mCurrPosition.z);
	//			Ogre::Quaternion q;
	//			q = Ogre::Quaternion::IDENTITY;
	//			for (i=0;i<mNumBodies;i++)
	//			{
	//				physJoint *kJoint = mBodies[i]->getJoint();
	//				//More rubber-man effect here, comment out if undesirable.
	//				if (kJoint) kJoint->setMotorTarget(q);//,mResetForce);
	//			}
	//			if (mSimStep == mDataBlock->mNumRestSteps - 40)
	//			{
	//				for (i=0;i<mNumBodies;i++) 
	//				{
	//					//  TEMP, having inertia problems, reset is flinging me exponentially farther every time...
	//					mBodies[i]->setLinearPosition(mBodyInitialPositions[i]);
	//					mFlexBody->mBodyParts[mActiveBodyIndices[i]]->mRB->setLinearPosition(mBodyInitialPositions[i]);
	//					mBodies[i]->setAngularPosition(mBodyInitialQuats[i]);
	//					mFlexBody->mBodyParts[mActiveBodyIndices[i]]->mRB->setAngularPosition(mBodyInitialQuats[i]);
	//					mBodies[i]->updatePositionToActor();
	//					mFlexBody->mBodyParts[mActiveBodyIndices[i]]->mRB->updatePositionToActor();
	//					mBodies[i]->setLinearVelocity(Ogre::Vector3(0,0,0));
	//					mFlexBody->mBodyParts[mActiveBodyIndices[i]]->mRB->setLinearVelocity(Ogre::Vector3(0,0,0));
	//					mBodies[i]->setAngularVelocity(Ogre::Vector3(0,0,0));
	//					mFlexBody->mBodyParts[mActiveBodyIndices[i]]->mRB->setAngularVelocity(Ogre::Vector3(0,0,0));
	//					mBodies[i]->updateVelocityToActor();//TEMP
	//					mFlexBody->mBodyParts[mActiveBodyIndices[i]]->mRB->updateVelocityToActor();
	//					//mBodies[i]->getJoint()->setNewTarget(q);
	//				}
	//				//mIsResetting = false;
	//			} else if (mSimStep == mDataBlock->mNumRestSteps - 20) 
	//			{					
	//				//Had to remove this, need to record which bodyparts are supposed to be ragdoll
	//				//and which ones have to stay kinematic now.
	//				//mFlexBody->clearKinematic();
	//			}
	//		}
	//		else if (mSimStep == mDataBlock->mNumRestSteps) 
	//		{
	//			mActionState = GA_ACTION_ACT;
	//			mIsResetting = false;
	//			//mFlexBody->clearKinematic();
	//			mSimStep = 0;
	//		}
	//	}
	//	if (mActionState==GA_ACTION_ACT) 
	//	{//Move through the action set, applying whatever actions are currently valid.
	//		if (mSimStep < mActionSet->mNumSteps) 
	//		{
	//			//if (mFlexBody->mIsKinematic) mFlexBody->clearKinematic();

	//			act();

	//			if (mObsSeq->mObsInterval) {
	//				if (((int)(mSimStep/mObsSeq->mObsInterval)*mObsSeq->mObsInterval) == mSimStep) 
	//				{
	//					for (unsigned int i=0;i<GA_MAX_FITNESS_DATA;i++)
	//					{
	//						if (mDataBlock->mFitnessData[i]!=NULL)
	//						{
	//							int bodypartID = mFlexBody->getBodyPart(mDataBlock->mFitnessData[i]->mBodypartName)->mPartID;
	//							//TEMP!  Need array to index from bodypart IDs on the flexbody into bodies on the actionUser
	//							//for now, bodypartID-1 works, because we are using all the bodyparts except the root.
	//							//This will probably change.
	//							observeBodypart(bodypartID-1,
	//								mDataBlock->mFitnessData[i]->mPositionGoal,
	//								mDataBlock->mFitnessData[i]->mPositionGoalType,
	//								mDataBlock->mFitnessData[i]->mRotationGoal,
	//								mDataBlock->mFitnessData[i]->mRotationGoalType
	//								);
	//						}
	//					}
	//				}
	//			}
	//		}
	//		else
	//		{
	//			if (++mCurrSeqRep==mDataBlock->mNumSequenceReps) 
	//			{
	//				mSimStep = 0;
	//				mActionSet->clear();
	//				mCurrSeqRep = 0;
	//				mActionState = GA_ACTION_END_SET;
	//			} 
	//			else
	//				mSimStep = 0;
	//		}
	//	}


	//	if (mActionState==GA_ACTION_END_SET) ////////////////////////////////////////////////////////
	//	{
	//		//HERE: run through the list of observations, score each of them
	//		//then sort each subset, so the best is first, and record the relative
	//		//rating compared to the others of that subset in that generation.
	//		mObsSeq->scoreSequence();	
	//		//Con::errorf("%s gen %d pop %d seq %d score: %1.2f",mShapeName,mActionGroups[mCurrPop]->mAU->mCurrGen,mCurrPop,mActionGroups[mCurrPop]->mCurrSet,mObsSeq->mScore);
	//		mActionGroups[mCurrPop]->mActionSets[mActionGroups[mCurrPop]->mCurrSet]->setScore(mObsSeq->mScore);
	//		//HERE: mObsSets[0] is in place of mObsSets[mCurrPop], for current population, or island.
	//		mObsSets[0]->mSequences[mActionGroups[mCurrPop]->mCurrSet]->mScore = mObsSeq->mScore;
	//		mObsSeq->clear();
	//		mActionGroups[mCurrPop]->mCurrSet++;

	//		//Con::errorf("curr set: %d, num sets: %d",mActionGroup->mCurrSet,mActionGroup->mNumSets);
	//		if (mActionGroups[mCurrPop]->mCurrSet < mActionGroups[mCurrPop]->mNumSets)
	//			mActionSet->set(mActionGroups[mCurrPop]->mActionSets[mActionGroups[mCurrPop]->mCurrSet]);
	//		else 
	//		{
	//			mObsSets[mCurrPop]->sort();
	//			saveAll();
	//			mActionGroups[mCurrPop]->repopulate();//config
	//			if (gRandom.randF() < mDataBlock->mMigrateChance)
	//			{
	//				int otherPop = gRandom.randI(0,mNumPops-2);
	//				if (otherPop>=mCurrPop) otherPop++;
	//				mActionGroups[mCurrPop]->immigrate(mActionGroups[otherPop]);
	//				//Con::errorf("successful immigration to %d from %d!",mCurrPop,otherPop);
	//			}
	//			//HERE: test for migrate chance, and then randomly pick another population to grab ActionSet[0] from, migrate in.
	//			//mActionGroups[mCurrPop]->migrate();//something with random(mNumPops-1) and then get around mCurrPop, to give it one that
	//			//is NOT this one.  Simple thing, but gotta go now.
	//			//sprintf(filename,"%s.%s.%d",mShapeName,mDataBlock->mActionName,mCurrGen);
	//			//saveAction(filename);
	//			mActionGroups[mCurrPop]->mCurrSet = 0;
	//			mCurrPop++;
	//			if (mCurrPop == mNumPops)
	//			{
	//				mCurrGen++;
	//				mCurrPop = 0;
	//			}
	//			mActionSet->set(mActionGroups[mCurrPop]->mActionSets[0]);
	//		} 

	//		mActionState = GA_ACTION_RESET;
	//		mSimStep=0;
	//	}
	//}
	//mSimStep++;
	//return;
}


//////////////////////////////////////////////////  ACT  ////////////////
void gaActionUser::act() 
{
	//unsigned int i,j;
	//Ogre::Vector3 work; work = Ogre::Vector3::ZERO;
	//Ogre::Quaternion q;  q = Ogre::Quaternion::IDENTITY;

	////Con::errorf("ga acting, numActions %d mSimStep %d",mActionSet->mNumActions,mSimStep);

	//for (i=0;i < mActionSet->mNumActions;i++) {
	//	if ((mSimStep >= mActionSet->mActions[i]->mStartStep)
	//		&&(mSimStep <= mActionSet->mActions[i]->mEndStep)) 
	//	{
	//		//parts[mActionSet->mActions[i]->bodyID]->applyForces(&mActionSet->mActions[i]->mForces);

	//		//HERE:  use Ogre::Quaternion->interpolate(quatA,quatB,time) to distribute motion over numSteps;
	//		//t = (mSimStep - mActionSet->mActions[i]->mStartStep) / mActionSet->mActions[i]->mNumSteps;
	//		//quatA = joint last motor target
	//		//quatB = joint's new motor target (action "mForces")

	//		int rb = mActionSet->mActions[i]->mBodyIndex;
	//		//Con::errorf("rb = %d",rb);
	//		
	//		//nxRigidBody *kRB = dynamic_cast<nxRigidBody *>(mBodies[rb]);
	//		//nxRigidBody *kRB = dynamic_cast<nxRigidBody *>(mBodies[rb]);
	//		nxRigidBody *kRB = dynamic_cast<nxRigidBody *>(mFlexBody->mBodyParts[rb]->mRB);
	//		
	//		
	//		if (kRB)
	//		{
	//			if (mSimStep == mActionSet->mActions[i]->mStartStep)
	//			{//  on first time, set last target, and set new target
	//				//then interpolate between them on every other round. (???)
	//				if ((kRB->getJoint())&&(!kRB->getIsKinematic())) 
	//				{
	//					q = kRB->getJoint()->getMotorTarget();
	//					
	//					//mActiveBodies[rb]->getJoint()->setLastTarget(q);
	//					kRB->getJoint()->setLastTarget(q);

	//					//if (mActionSet->mActions[i]->mForces.length()>0)
	//					//{
	//					//	work = mActionSet->mActions[i]->mForces;
	//					//	//if (work.length()&&(rb==14))
	//					//	//	Con::errorf("acting shin motor target: %f %f %f",work.x,work.y,work.z);
	//					//	Ogre::Quaternion q2;
	//					//	//q2 = Ogre::Quaternion::IDENTITY;
	//					//	//q2.set(Ogre::Vector3(work.x * M_PI/1.0,work.y * M_PI/1.0,work.z * M_PI/1.0));
	//					//	//q2.set(Ogre::Vector3(work.x * M_PI/1.0,work.y * M_PI/1.0,work.z * M_PI/1.0));
	//					//	//if (work.length()>0) Con::printf("act: body %d, target %f %f %f",
	//					//	//	rb,mRadToDeg(work.x),mRadToDeg(work.y),mRadToDeg(work.z));
	//					//	q2.set(Ogre::Vector3(work.x,work.y,work.z));
	//					//	kRB->getJoint()->setNewTarget(q2);//?
	//					//	kRB->getJoint()->setMotorTarget(q2);//,mActionForce*1000.0);
	//					//} else {
	//					kRB->getJoint()->setNewTarget(mActionSet->mActions[i]->mQuat);//?
	//					kRB->getJoint()->setMotorTarget(mActionSet->mActions[i]->mQuat);

	//					//q = mActionSet->mActions[i]->mQuat;
	//					//Con::errorf("setting body %d to motor target: %f %f %f %f",rb,q.x,q.y,q.z,q.w);

	//					//}
	//					//mBodies[rb]->getJoint()->setMotorSpring(q2,mActionForce);

	//					//Whoops, this makes no sense: no point in interpolating on a thing we do only at startStep for each action.
	//					//need to interpolate on every step for it to make any sense.
	//					//float t = (mSimStep - mActionSet->mActions[i]->mStartStep) / (float)(mActionSet->mActions[i]->mNumSteps);
	//					//Ogre::Quaternion q,q1;
	//					//q1 = kRB->getJoint()->getLastTarget();
	//					//q2 = kRB->getJoint()->getNewTarget();
	//					//q.interpolate(q1,q2,t);

	//					//HERE:  I did try to interpolate motor targets, but is this what's making 
	//					//me freak out?
	//					//kRB->getJoint()->setMotorTarget(q);
	//				}
	//			}
	//		}
	//	}
	//}
}

void gaActionUser::observeBodypart(unsigned int kBodyId, Ogre::Vector3 kPosScale, Ogre::Vector3 kPosDiffFromStart, float kRotScale, int kRotDiffFromStart)
{
	//int numParams = 0;
	//float angB = 0.0;
	//float params[GA_MAX_OBSERV_PARAMS];
	//Ogre::Vector3 pos;
	//Ogre::Quaternion q;

	//pos = Ogre::Vector3::ZERO;

	//q = mBodies[kBodyId]->getAngularPosition();
	//angB = q.angleBetween(mBodyInitialQuats[kBodyId]);
	//if (angB>(M_PI/2)) angB = M_PI - angB;

	////Sigh, cheap hack, but tired of dinking around with this.  Got a first frame undefined angB problem
	////that is causing a crash.  Rather than figure out how to test for an float being undefined, or figure out
	////what's wrong with the start quat, I'm just going to force frame 0 to report 0.0 for now, and get on with life.
	//if (mObsSeq->mNumUnits==0) angB=0.0;

	//pos = mBodies[kBodyId]->getLinearPosition();
	//pos -= mBodyInitialPositions[kBodyId];

	//if (kPosScale.x) 	
	//{  
	//	if (kPosDiffFromStart.x) params[numParams] = fabs(pos.x) * kPosScale.x;  
	//	else params[numParams] = pos.x * kPosScale.x; 
	//	numParams++; 
	//}
	//if (kPosScale.y) 	
	//{  
	//	if (kPosDiffFromStart.y) params[numParams] = fabs(pos.y) * kPosScale.y;  
	//	else params[numParams] = pos.y * kPosScale.y; 
	//	numParams++; 
	//}
	//if (kPosScale.z) 	
	//{  
	//	if (kPosDiffFromStart.z) params[numParams] = fabs(pos.z) * kPosScale.z;  
	//	else params[numParams] = pos.z * kPosScale.z; 
	//	numParams++; 
	//}	
	//if (kRotScale)
	//{
	//	if (kRotDiffFromStart) params[numParams] = fabs(angB) * kRotScale;
	//	else params[numParams] = angB * kRotScale;//(this one not too useful)
	//	numParams++;
	//}

	//mObsUnit->setObsData(numParams,params);
	//mObsSeq->addUnit(mObsUnit);
}




void gaActionUser::saveAction(const char *name)
{
	char filename[255];
	char buf[255],part[50];

	sprintf(filename,"actions/%s.action",name);
	mActionGroups[mCurrPop]->mActionSets[0]->saveAsText(filename);
}

bool gaActionUser::loadAction(const char *name)
{
	//char filename[255];
	//bool dynamic, exclusive, repeat;
	////if (!mFlexBody->getShapeInstance()->mShapeResource)
	//// return false;
 //   const Ogre::String myPath = mFlexBody->getShapeInstance()->mShapeResource.getPath().getPath();
	////Con::errorf("loading action %s/actions/%s.%s.action",myPath.c_str(),mShapeName,name);
	//sprintf(filename,"%s/actions/%s.%s.action",myPath.c_str(),mShapeName,name);//player.move_forward, snake_1.curl_right, etc

	//dynamic = false;
	//exclusive = true;
	//repeat = false;

	////if (!strncmp(name,"dynam",5)) 
	////{//FIXED - this is stored as an argument in the action now, "$dynamic = 1;".
	////	Con::errorf("loading a dynamic action!");
	////	dynamic = true;//
	////}
	////CHANGE: Now I'm going to use mActionSet to store (or add) any loaded action, and 
	////only modify mActionGroup if I intend to train this action.  Do that elsewhere.
	//////mActionGroup->clear();
	////mActionGroup->mActionSets[0]->loadAsText(filename);//this sets my mNumActiveBodies 
	//                                               //and mActiveBodyIndices as it goes
	////mActionGroup->mNumBodies = mActionGroup->mActionSets[0]->mNumBodies;
	////mActionGroup->mCurrSet = 0;
	//mActionSet->clear();

	//if (!strncmp(name,"sequence",8)) {
	//	char seqName[255];
	//	sscanf(name,"sequence.%s",&seqName);
	//	//Con::errorf("loading sequence %s",seqName);
	//	mActionSet->loadSequence(seqName);
	//	mGoalState = GA_GOAL_PLAY_SEQUENCE;
	//	Ogre::String actionSeqName(seqName);
	//	TSShape *kShape = mFlexBody->getShapeInstance()->getShape();
	//	int seq = mFlexBody->getThreadSequence(0);
	//	if ((seq>=0)&&(seq<kShape->sequences.size()))
	//	{
	//		String threadSeqName(kShape->names[kShape->sequences[seq].nameIndex]);
	//		if (actionSeqName == threadSeqName) 
	//		{
	//			float pos = mFlexBody->getThreadPos(0);
	//			mSimStep = (int)(pos * (float)(mActionSet->mNumSteps));
	//		}
	//	}
	//	//Con::errorf("finished loading sequence.");
	//} else if (!strncmp(name,"all",3)){
	//	loadAll();
	//} else {
	//	mActionSet->loadAsText(filename);//,dynamic=true
	//	if (mDataBlock->mNumPopulations<1) 
	//		mDataBlock->mNumPopulations = 1;//safety check
	//	for (unsigned int i=0;i<mDataBlock->mNumPopulations;i++)
	//	{
	//		mActionGroups[i]->mActionSets[0]->set(mActionSet);//this sets my mNumActiveBodies 
	//		mActionGroups[i]->mNumBodies = mActionSet->mNumBodies;
	//		mActionGroups[i]->mCurrSet = 0;            //and mActiveBodyIndices as it goes
	//		mActionGroups[i]->repopulateFromOne();
	//	}
	//}
	////Con::errorf("mNumActiveBodies: %d",mNumActiveBodies);
	//
	////mActionSet->loadAsText(filename,false,true,false);//dynamic=false, exclusive=true, repeat=false
	////"dynamic" means actions that move down the body, involving all parts eventually.
	////"exclusive" means all other actions get cleared, body 

	////if (mDataBlock->mNumSlices != mActionSet->mNumSlices)
	////	Con::errorf("ActionUser::loadAction -- wrong number of slices!");
	////FIX!  Should set my own numSlices to the number from the actionset, not be locked down.

	////UH OH... crash and burn.  Don't have mRB for everything, apparently
	////for (unsigned int i=1;i<mNumActiveBodies;i++) {
	////	mActiveBodies[i] = mFlexBody->mBodyParts[mActiveBodyIndices[i]]->mRB;
	////}

	////mActionGroup->repopulateFromOne();
	////mActionGroup->mCurrSet = 0;
	//mDone = 0;
	//mSimStep = 0;
	//if (mNumActiveBodies) return true;
	//else 
	return false;
}


bool gaActionUser::loadAction(const char *name, float pos)
{
	//char filename[255];
	//bool dynamic, exclusive, repeat;
	////if (!mFlexBody->getShapeInstance()->mShapeResource)
	//// return false;
	//const Ogre::String myPath = mFlexBody->getShapeInstance()->mShapeResource.getPath().getPath();
	////Con::errorf("loading action %s/actions/%s.%s.action",myPath.c_str(),mShapeName,name);
	//sprintf(filename,"%s/actions/%s.%s.action",myPath.c_str(),mShapeName,name);//player.move_forward, snake_1.curl_right, etc

	//dynamic = false;
	//exclusive = true;
	//repeat = false;

	////if (!strncmp(name,"dynam",5)) 
	////{//FIXED - this is stored as an argument in the action now, "$dynamic = 1;".
	////	Con::errorf("loading a dynamic action!");
	////	dynamic = true;//
	////}
	////CHANGE: Now I'm going to use mActionSet to store (or add) any loaded action, and 
	////only modify mActionGroup if I intend to train this action.  Do that elsewhere.
	//////mActionGroup->clear();
	////mActionGroup->mActionSets[0]->loadAsText(filename);//this sets my mNumActiveBodies 
	//                                               //and mActiveBodyIndices as it goes
	////mActionGroup->mNumBodies = mActionGroup->mActionSets[0]->mNumBodies;
	////mActionGroup->mCurrSet = 0;
	//mActionSet->clear();

	//if (!strncmp(name,"sequence",8)) {
	//	char seqName[255];
	//	sscanf(name,"sequence.%s",&seqName);
	//	//Con::errorf("loading sequence %s",seqName);
	//	mActionSet->loadSequence(seqName);
	//	mGoalState = GA_GOAL_PLAY_SEQUENCE;
	//	mSimStep = (int)(pos * (float)(mActionSet->mNumSteps));
	//	//Con::errorf("finished loading sequence.");
	//} else if (!strncmp(name,"all",3)){
	//	loadAll();
	//} else {
	//	mActionSet->loadAsText(filename);//,dynamic=true
	//	if (mDataBlock->mNumPopulations<1) 
	//		mDataBlock->mNumPopulations = 1;//safety check
	//	for (unsigned int i=0;i<mDataBlock->mNumPopulations;i++)
	//	{
	//		mActionGroups[i]->mActionSets[0]->set(mActionSet);//this sets my mNumActiveBodies 
	//		mActionGroups[i]->mNumBodies = mActionSet->mNumBodies;
	//		mActionGroups[i]->mCurrSet = 0;            //and mActiveBodyIndices as it goes
	//		mActionGroups[i]->repopulateFromOne();
	//	}
	//}
	////Con::errorf("mNumActiveBodies: %d",mNumActiveBodies);
	//
	////mActionSet->loadAsText(filename,false,true,false);//dynamic=false, exclusive=true, repeat=false
	////"dynamic" means actions that move down the body, involving all parts eventually.
	////"exclusive" means all other actions get cleared, body 

	////if (mDataBlock->mNumSlices != mActionSet->mNumSlices)
	////	Con::errorf("ActionUser::loadAction -- wrong number of slices!");
	////FIX!  Should set my own numSlices to the number from the actionset, not be locked down.

	////UH OH... crash and burn.  Don't have mRB for everything, apparently
	////for (unsigned int i=1;i<mNumActiveBodies;i++) {
	////	mActiveBodies[i] = mFlexBody->mBodyParts[mActiveBodyIndices[i]]->mRB;
	////}

	////mActionGroup->repopulateFromOne();
	////mActionGroup->mCurrSet = 0;
	//mDone = 0;
	//mSimStep = 0;
	//if (mNumActiveBodies) 
	//	return true;
	//else 
		return false;
}

void gaActionUser::loadActionSetOnly(const char *name)
{//RENAME:  (quick & dirty) - this is to call loadActionSetAsText, which is also q&d attempt to 
	//avoid messing with mActionUser->mActiveBodyIndices, etc. for an action that is merely setting
	//up for another action.
	//char filename[255];
	//const Ogre::String myPath = mFlexBody->getShapeInstance()->mShapeResource.getPath().getPath();
	//sprintf(filename,"%s/actions/%s.action",myPath.c_str(),name);

	//mActionSet->loadActionSetAsText(filename);
	////if (mDataBlock->mNumSlices != mActionSet->mNumSlices)
	////	Con::errorf("ActionUser::loadActionSetOnly -- wrong number of slices! %d, actionset %d",
	////	mDataBlock->mNumSlices,mActionSet->mNumSlices);

	////mNumTimeSlices = mActionSet->mNumSlices;

	////for (unsigned int i=0;i<mNumActiveBodies;i++) {
	////	mActiveBodies[i] = mFlexBody->mBodyParts[mActiveBodyIndices[i]]->mRB;
	////}

	////mActionGroup->repopulateFromOne();
	////mActionGroup->mCurrSet = 0;
	//mDone = 0;
	//mSimStep = 0;

}

	/*
	int i,j,kNumActions,numActions,id;
	float start,duration,x,y,z;
	Ogre::Vector3 work;
	char filename[255],buf[255];

	kNumActions = 0;
	mNumBodies = 0;
	//mCurrSet = 0;

	//for (i=0;i<GA_MAX_ACTIONS;i++) 
	//sprintf(filename,"%s_%d.action",mFlexBody->(name),..);
	sprintf(filename,"actions/dragon_3_best.action");
	FILE *fp = fopen(filename,"r");

	sscanf(buf,"numActions: %d;",&numActions);


	//Okay, lost the thread, gotta stop.  HERE: clear the way for the rest of the action sets
	while (fgets(buf,255,fp) != NULL) {
		sscanf(buf,"%d;%f;%f;(%f,%f,%f);",&id,&start,&duration,&x,&y,&z);
		work = Ogre::Vector3(x,y,z);
		//mActionGroup->mActionSets[0]->mActions[kNumActions] = new gaAction(id,GA_NUM_SEQUENCE_STEPS,start,duration,work);	
		//mActionGroup->mActionSets[0]->mActions[kNumActions++]->setSteps(mNumSteps);
	}

	//if (mNumActions==numActions) Con::errorf("loaded %d actions!",numActions);

	fclose(fp);
	*/
	

void gaActionUser::saveAll()
{	
	//char filename[255];

	//const Ogre::String myPath = mFlexBody->getShapeInstance()->mShapeResource.getPath().getPath();
	//	
	////sprintf(filename,"%s_%d.action",mFlexBody->(name)
	////sprintf(filename,"actions/dragon_3.stand_up.all.action");
	////sprintf(filename,"actions/spider_1.all.%d.action",mCurrGen);
	//sprintf(filename,"%s/actions/archive/%s.all.%d.action",myPath.c_str(),mShapeName,mCurrGen);
	//FILE *out = fopen(filename,"w");
	//if (!out) 
	//{
	//	//Con::printf("whoops, no archive directory! %s",filename); 
	//	return;
	//}
	//	
	//for (int i=0;i<mActionGroups[mCurrPop]->mNumSets;i++) {	
	//	if	(i==0) {
	//		fprintf(out,"$numSets = %d;\n",mActionGroups[mCurrPop]->mNumSets);
	//		fprintf(out,"$numSlices = %d;\n",mActionGroups[mCurrPop]->mActionSets[0]->mNumSlices);
	//		fprintf(out,"$numSteps = %d;\n",mActionGroups[mCurrPop]->mActionSets[0]->mNumSteps);
	//		fprintf(out,"$currGen = %d;\n",mCurrGen);
	//	}
	//	for (int j=0;j<mActionGroups[mCurrPop]->mActionSets[i]->mNumActions;j++) {
	//		if (j==0) fprintf(out,"$numActions = %d;  Score %f;\n",mActionGroups[mCurrPop]->mActionSets[i]->mNumActions,
	//			mActionGroups[mCurrPop]->mActionSets[i]->mScore);
	//		gaAction *kAction = mActionGroups[mCurrPop]->mActionSets[i]->mActions[j];	
	//		fprintf(out,"%d;%d;%3.2f;%3.2f;(%3.2f,%3.2f,%3.2f);\n",kAction->mBodyIndex,kAction->mIsTraining,kAction->mStart,kAction->mDuration,
	//			mRadToDeg(kAction->mForces.x),mRadToDeg(kAction->mForces.y),mRadToDeg(kAction->mForces.z));	
	//	}
	//}
	//fclose(out);
}

void gaActionUser::loadAll()
{
	//int i,j,id,training,numSets,numSlices,numSteps,numParts,numActions,found;
	//float start,duration,x,y,z;
	//char filename[255],buf[255];		
	//int parts[GA_MAX_BODIES];

	//const Ogre::String myPath = mFlexBody->getShapeInstance()->mShapeResource.getPath().getPath();
	//	
	//sprintf(filename,"%s/actions/%s.all.action",myPath.c_str(),mShapeName);
	//FILE *fp = fopen(filename,"r");

	////mActionGroup->clear();
	//for (i=0;i<mNumActiveBodies;i++) mActiveBodyIndices[i] = -1;
	//mNumActiveBodies = 0;

	//fgets(buf,255,fp);
	//sscanf(buf,"$numSets = %d;",&numSets);
	//fgets(buf,255,fp);
	//sscanf(buf,"$numSlices = %d;",&numSlices);
	//fgets(buf,255,fp);
	//sscanf(buf,"$numSteps = %d;",&numSteps);
	//fgets(buf,255,fp);
	//sscanf(buf,"$currGen = %d;",&mCurrGen);

	//fgets(buf,255,fp);//first "numActions=n;"
	//for (i=0;i<numSets;i++) 
	//{
	//	gaActionSet *kActionSet;
	//	kActionSet = mActionGroups[mCurrPop]->mActionSets[i];
	//	kActionSet->mNumSteps = numSteps;
	//	numParts=0;
	//	numActions = 0;
	//	for (j=0;j<GA_MAX_BODIES;j++) parts[j] = -1;

	//	while (fgets(buf,255,fp)) {
	//		if ((buf[0]=='#')||(buf[0]==' ')||(buf[0]=='\n')||(buf[0]=='\t')) continue;
	//		if (!strncmp(buf,"$numActions",11)) break;

	//		sscanf(buf,"%d;%d;%f;%f;(%f,%f,%f);",&id,&training,&start,&duration,&x,&y,&z);

	//		found=0;
	//		for (j=0;j<numParts;j++) if (parts[j]==id) found=1;
	//		if (!found) {
	//			mActiveBodyIndices[numParts]=id;
	//			parts[numParts++]=id;
	//		}

	//		kActionSet->mActions[numActions] = new gaAction();
	//		kActionSet->mActions[numActions]->mBodyIndex = id;
	//		kActionSet->mActions[numActions]->mIsTraining = training;
	//		kActionSet->mActions[numActions]->mStart = start;
	//		kActionSet->mActions[numActions]->mDuration = duration;
	//		kActionSet->mActions[numActions]->mForces = Ogre::Vector3(mDegToRad(x),mDegToRad(y),mDegToRad(z));
	//		kActionSet->mActions[numActions]->setSteps(numSteps);

	//		numActions++;
	//	}
	//	kActionSet->mNumBodies = numParts;
	//	kActionSet->mNumActions = numActions;
	//	kActionSet->mNumSlices = numSlices;
	//}
	//mActionGroups[mCurrPop]->mNumSets = numSets;
	//mActionGroups[mCurrPop]->mNumBodies = numParts;
	//mActionGroups[mCurrPop]->mCurrSet = 0;

	//mActionSet->set(mActionGroups[mCurrPop]->mActionSets[0]);
	////setGoalDynamic();
	//fclose(fp);
}

//void gaActionUser::setGoalSingleAction()
//{
//	mGoalState = GA_GOAL_SINGLE_ACTION;
//}
//
//void gaActionUser::setGoalForward()
//{
//	mGoalState = GA_GOAL_FORWARD;
//}

void gaActionUser::setForwardForce(float force)
{
	mForwardForce = force;
}

void gaActionUser::motorize()
{
	//Con::errorf("action user motorizing: %d",mFlexBody->mNumBodyParts);
	mFlexBody->motorize();
	//for (unsigned int i=0;i<mFlexBody->mNumBodyParts;i++)
	//	mFlexBody->setBodypartMotorTarget(i,Ogre::Vector3(0,0,0));
}



void gaActionUser::thinkAction()
{
	//if (mActionState==GA_ACTION_NONE) ///////////////////////////////////////////////////////
	//{
	//	motorize();
	//	mSimStep = -1;//start step is 0, and we simStep++ at end of this function, so...

	//	mActionState = GA_ACTION_ACT;
	//	//mActionSet->clear();//wtf? somehow we have numActions==99 right here... ??
	//	//Con::errorf("thinkAction about to load action: numactions %d",mActionSet->mNumActions);
	//	//if (mActionSet->mNumActions==0) loadAction(mDataBlock->mActionName);

	//	//In case we're still moving forward:
	//	for (unsigned int i=0;i<mNumBodies;i++) 
	//	{
	//		mFlexBody->mBodyParts[i]->setForce(Ogre::Vector3(0,0,0));
	//		mBodyInitialPositions[i] = mBodies[i]->getLinearPosition();
	//		mBodyInitialQuats[i] = mBodies[i]->getAngularPosition();	
	//	}
	//	//This might be interesting... to stiffen up the body in whatever pose it is in.
	//	//for (i=1;i<mNumBodies;i++) {
	//	//	physJoint *kJoint = mBodies[i]->getJoint();
	//	//	if (kJoint) kJoint->setMotorSpring(mBodyInitialQuats[i],mResetForce);
	//	//}
	//} 
	//else if (mActionState==GA_ACTION_RESET) //////////////////////////////////////////////
	//{
	//	if ((mSimStep==1)&&(mBodyInitialPositions[0].length()))//mBodyStartPositions
	//	{
	//		//mFlexBody->mReset = true;
	//		for (unsigned int i=0;i<mNumBodies;i++) 
	//		{
	//			//  TEMP, having inertia problems, reset is flinging me exponentially farther every time...
	//			mBodies[i]->setLinearPosition(mBodyInitialPositions[i]);
	//			mBodies[i]->setAngularPosition(mBodyInitialQuats[i]);
	//			mBodies[i]->updatePositionToActor();
	//			mBodies[i]->setLinearVelocity(Ogre::Vector3(0,0,0));
	//			mBodies[i]->setAngularVelocity(Ogre::Vector3(0,0,0));
	//			mBodies[i]->updateVelocityToActor();//TEMP
	//			//mBodies[i]->getJoint()->setNewTarget(q);
	//		}
	//		//for (unsigned int i=0;i<mFlexBody->mNumBodyParts;i++)
	//		//	mFlexBody->mBodyParts[i]->updatePositionFromRB();
	//		//mFlexBody->updateNodes();
	//	}
	//	else if (mSimStep < mDataBlock->mNumRestSteps) 
	//	{
	//		Ogre::Quaternion q;
	//		q = Ogre::Quaternion::IDENTITY;
	//		for (unsigned int i=1;i<mNumBodies;i++) //(0 has no joint)
	//		{
	//			physJoint *kJoint = mBodies[i]->getJoint();
	//			//More rubber-man effect here, comment out if undesirable.
	//			if (kJoint) kJoint->setMotorTarget(q);//,mResetForce);
	//		}
	//	}
	//	else if (mSimStep == mDataBlock->mNumRestSteps) 
	//	{
	//		mActionState = GA_ACTION_ACT;
	//		mSimStep = 0;
	//	}
	//}
	//if (mActionState==GA_ACTION_ACT)
	//{
	//	if (mSimStep < mActionSet->mNumSteps) 
	//	{
	//		if (mFlexBody->mIsKinematic) mFlexBody->clearKinematic();
	//		act();
	//	}
	//	else
	//	{
	//		//mActionState = GA_ACTION_END_SET;//HERE: don't do this, cuz I'd like to repeat this action indefinitely.
	//		mSimStep = 0;
	//		mActionState=GA_ACTION_RESET;
	//	}
	//}
	//else if (mActionState==GA_ACTION_END_SET)
	//{
	//	mSimStep = 0;
	//	mGoalState = GA_GOAL_NONE;

	//	for (unsigned int i=1;i<mNumBodies;i++) {
	//		mBodies[i]->getJoint()->clearMotor();
	//	}
	//	mActionSet->clear();
	//}
	//
	//mSimStep++;
	//return;
}

void gaActionUser::thinkSequence()
{
	//if (mActionState==GA_ACTION_NONE) ///////////////////////////////////////////////////////
	//{
	//	//motorize();
	//	if (mSimStep >= mActionSet->mNumSteps) mSimStep = -1;//start step is 0, and we simStep++ at end of this function, so...

	//	mActionState = GA_ACTION_ACT;
	//	//mActionSet->clear();//wtf? somehow we have numActions==99 right here... ??
	//	//Con::errorf("thinkAction about to load action: numactions %d",mActionSet->mNumActions);
	//	//if (mActionSet->mNumActions==0) loadAction(mDataBlock->mActionName);

	//	//In case we're still moving forward://(Is this our lag problem?)
	//	for (unsigned int i=1;i<mNumBodies;i++) 
	//	{
	//		mFlexBody->mBodyParts[i]->setForce(Ogre::Vector3(0,0,0));
	//		mBodyInitialPositions[i] = mBodies[i]->getLinearPosition();
	//		mBodyInitialQuats[i] = mBodies[i]->getAngularPosition();	
	//	}

	//	act();//Is this our lag problem?

	//	//This might be interesting... to stiffen up the body in whatever pose it is in.
	//	//for (i=1;i<mNumBodies;i++) {
	//	//	physJoint *kJoint = mBodies[i]->getJoint();
	//	//	if (kJoint) kJoint->setMotorSpring(mBodyInitialQuats[i],mResetForce);
	//	//}
	//} 
	//else if (mActionState==GA_ACTION_RESET) //////////////////////////////////////////////
	//{
	//	if (0)//((mSimStep==1)&&(mBodyInitialPositions[0].length()))//mBodyStartPositions
	//	{//Temp: maybe this doesn't want to happen at all anymore.
	//		for (unsigned int i=0;i<mNumBodies;i++) 
	//		{
	//			//  TEMP, having inertia problems, reset is flinging me exponentially farther every time...
	//			mBodies[i]->setLinearPosition(mBodyInitialPositions[i]);
	//			mBodies[i]->setAngularPosition(mBodyInitialQuats[i]);
	//			mBodies[i]->updatePositionToActor();
	//			mBodies[i]->setLinearVelocity(Ogre::Vector3(0,0,0));
	//			mBodies[i]->setAngularVelocity(Ogre::Vector3(0,0,0));
	//			mBodies[i]->updateVelocityToActor();//TEMP
	//			//mBodies[i]->getJoint()->setNewTarget(q);
	//		}
	//	}
	//	else if (mSimStep < mDataBlock->mNumRestSteps) 
	//	{
	//		Ogre::Quaternion q;
	//		q = Ogre::Quaternion::IDENTITY;
	//		for (unsigned int i=1;i<mNumBodies;i++) //(0 has no joint)
	//		{
	//			physJoint *kJoint = mBodies[i]->getJoint();
	//			//More rubber-man effect here, comment out if undesirable.
	//			if (kJoint) kJoint->setMotorTarget(q);//,mResetForce);
	//		}
	//	}
	//	else if (mSimStep == mDataBlock->mNumRestSteps) 
	//	{
	//		mActionState = GA_ACTION_ACT;
	//		//mSimStep = 0;
	//		TSShape *kShape = mFlexBody->getShapeInstance()->getShape();
	//		int seq = mFlexBody->getThreadSequence(0);
	//		if ((seq>=0)&&(seq<kShape->sequences.size()))
	//		{
	//			String threadSeqName(kShape->names[kShape->sequences[seq].nameIndex]);
	//			if (mActionSet->mSequenceName == threadSeqName) 
	//			{//Temp: still need to make sure we're on this sequence.
	//				float pos = mFlexBody->getThreadPos(0);
	//				mSimStep = (int)(pos * (float)(mActionSet->mNumSteps));
	//			}
	//		}

	//	}
	//}
	//if (mActionState==GA_ACTION_ACT)
	//{
	//	if (mSimStep < mActionSet->mNumSteps) 
	//	{
	//		//if (mFlexBody->mIsKinematic) mFlexBody->clearKinematic();
	//		act();
	//	}
	//	else
	//	{
	//		//mActionState = GA_ACTION_END_SET;//HERE: don't do this, cuz I'd like to repeat this action indefinitely.
	//		mSimStep = 0;
	//		mActionState=GA_ACTION_RESET;
	//	}
	//}
	//else if (mActionState==GA_ACTION_END_SET)
	//{
	//	mSimStep = 0;
	//	mGoalState = GA_GOAL_NONE;

	//	//for (unsigned int i=1;i<mNumBodies;i++) {
	//	//	mBodies[i]->getJoint()->clearMotor();
	//	//}
	//	mActionSet->clear();
	//}
	//
	//mSimStep++;
	//return;

}
//
//void gaActionUser::thinkDynamic()//THIS IS ONLY FOR SNAKE MODEL
//{//HERE: this needs to pass a curve down the body over time
//	//there should be a numSteps that it should take to get all the way down the body
//	//that over numParts will be how many steps it spends with each bodypart
//	//the action should be defined without bodyparts named, or go ahead and start them with the
//	//head bodypart, but make a special load function that just loads the actions and doesn't 
//	//hook them to a bodypart, keep bodypartID = -1.
//	//Then, actually, it might work quite well to keep the current act() system in place,
//	//and just change the bodyparts here.  Hmm.
//
//
//	if (mActionState==GA_ACTION_NONE) //////////////// start ///////////////////////////////////////
//	{
//		mSimStep = -1;
//		mActionState = GA_ACTION_ONE;
//		//HERE: put the first action in the set onto the first bodypart
//		//important: the actions are going to run in reverse order from the way they look
//		//if you have (0,-0.1,0), (0,-0.2,0), (0,-0.3,0), then the top segment will go -0.1, -0.2, -0.3
//		//even though it looks like the opposite in a normal action.
//		mActionSet->mActions[0]->mBodyIndex = 0;//1 being first neck segment, 
//		//because head being root doesn't have a joint
//	}
//	else if (mActionState==GA_ACTION_END_SET)//////////////// end /////////////////////////////////
//	{
//		mSimStep=0;
//		mGoalState = GA_GOAL_NONE;
//	}
//	else ///////////////////  run the action  ///////////////////////////////////////////
//	{
//		if (mSimStep < mActionSet->mNumSteps)//BendSteps... really it needs to be by action, in the .action file 
//		{
//			if (mFlexBody->mIsKinematic) mFlexBody->clearKinematic();
//			
//			if (mSimStep % 4 == 0) mActionSet->mActions[0]->mBodyIndex++;
//			if (mActionSet->mActions[0]->mBodyIndex > mFlexBody->mNumBodyParts) 
//			{
//				mActionSet->mActions[0]->mBodyIndex--;
//				mActionState = GA_ACTION_END_SET;
//			}
//			else
//				act();
//			//set forward forces
//		}
//		else
//		{
//			mActionState = GA_ACTION_END_SET;
//		}
//	}
//	mSimStep++;
//
//}


/*
				//HERE: this part is FUBAR, need to be calling loadAction "head_up" instead.
				if ((i>1)&&(i<4))
				{
					//if (mSimStep%3==0)
					if (0)//oh well
					{
						//reverse neck direction, start with only one bodypart
						work = Ogre::Vector3(0.0,0.0,0.1);
						q2.set(Ogre::Vector3(work.x * M_PI/1.0,work.y * M_PI/1.0,work.z * M_PI/1.0));
						mBodies[i]->getJoint()->setMotorTarget(q2);
					}
					else
					{
						//q2 = Ogre::Quaternion::IDENTITY;
						//mBodies[i]->getJoint()->setNewTarget(q2);
					}
				}
				//HERE:  work in one or two joint's worth of side to side motion, somewhere in the neck, 
				//and then do some constant head raising as well.
				//HERE: check this bodypart's velocity, based on this position and last position.  If below a certain amount,
				//add force, otherwise don't.
				//mass = ?  ... I can get density, but not volume from nxActor
*/







//////////////////////////////////////////////////
/////////   g a A c t i o n S e t        /////////
//
// The new evolution of the Genetic Algorithm Thing.
// This time it's based on gaActions which are each 
// linked to exactly one rigid body, and which each 
// contain a start time and a duration.  
// The original method, with equal length time slices
// in which all limbs are acting all the time, can be 
// duplicated by creating a set of actions that covers
// all the bodies all the time, then run through them
// just like they were a sparser set with only some 
// muscles in action at any given time. 
// The reason for doing it that way is for ease of 
// mating.  A reasonable crossover algorithm is a lot 
// easier to figure out for a full array of constant
// action.
//
//////////////////////////////////////////////////
/*
void gaActionGroup::loadAction(char *creature,int start_action,Entity *ent)
{
int i,j,k,s,part;
int num_parts,num_slices,num_actions,num_file_parts,start_act;
float t,x,y,z,tn;
//float actions[5][40];
char filename[255];
char partname[255];
char buf[1000];
char *bufp;
int bodyparts[100];
Ogre::Vector3 work; work = Ogre::Vector3::ZERO;

gaFileAction fa[GA_MAX_FILE_ACTIONS];

//num_parts = actionSets[0]->numBodyparts;
num_slices = actionSets[0]->numSlices;

num_file_parts = 0;

//sprintf(filename,"%s.action_%d.cfg",creature,start_action);
//FILE *fp = fopen(filename,"r");


//for (s=0;s<num_sets;s++) {

while (fgets(buf, 255, fp) != NULL) {
if ((buf[0]=='#')||(buf[0]==' ')||(buf[0]=='\n')||(buf[0]=='\t')) continue;
else {//if not a comment or whitespace, this is an action line.
num_actions = 0;
bufp = strtok(buf,";");
sscanf(bufp,"%s",&partname);    bufp = strtok(NULL,";");
sscanf(bufp," t(%f)",&t);    bufp = strtok(NULL,";");
sscanf(bufp," f(%f,%f,%f)",&x,&y,&z); bufp = strtok(NULL,";");
fa[num_file_parts].part = ent->partFind(partname);
ent->parts[fa[num_file_parts].part]->active = 1;
bodyparts[num_file_parts] = fa[num_file_parts].part;
//printf("activating part %d, %s!\n",bodyparts[num_file_parts],partname);

fa[num_file_parts].times[num_actions] = t; fa[num_file_parts].forces[num_actions].Set(x,y,z);
num_actions++;

while (strlen(bufp)>3) {
sscanf(bufp," t(%f)",&t);    bufp = strtok(NULL,";");
sscanf(bufp," f(%f,%f,%f)",&x,&y,&z); bufp = strtok(NULL,";");
fa[num_file_parts].part = ent->partFind(partname);
fa[num_file_parts].times[num_actions] = t; fa[num_file_parts].forces[num_actions].Set(x,y,z);
num_actions++;
//printf("%s : %f, (%f,%f,%f)\n",partname,t,x,y,z);
}
fa[num_file_parts].numActions = num_actions;
num_file_parts++; 
}
} 
fclose(fp);

//HERE: now that you have actions[][] loaded up, and num_actions, you just have to loop through 
//num_bodyparts * num_slices and make actions.  First for each actionSet, do setZeroActions to
//fill an array with zeros.  Then change the affected ones to forces. 

ent->activeParts = num_file_parts;
//numBodyparts = num_file_parts;
//I think the above will not be necessary

//HERE: fill up just the desired slots with zero actions, not the whole creature.
for (s=0;s<numSets;s++) {
actionSets[s]->addZeroActions(num_file_parts,mBodies); 
}
//confusion exists!  between the different initialization procedures, and which ones are 
//called when, which ones create new actionSets and which ones assume existing actionSets.

for (s=0;s<numSets;s++) {
for (i=0;i<num_file_parts;i++) {
//start_act = fa[i].part * num_slices;
start_act = i * num_slices;
int ca = 0;//current action

while (ca < fa[i].numActions) {
t = fa[i].times[ca];

if (ca < (fa[i].numActions-1)) tn = fa[i].times[ca+1];
else tn = 1.0;

work.Set(&fa[i].forces[ca]);
for (j=0;j<num_slices;j++) {
float slice_step = j*(1.0/(float)num_slices);
if ((t<=slice_step)&&(tn>=slice_step)) {
actionSets[s]->actions[start_act+j]->forces.Set(work);
}
}
ca++;
}
}
actionSets[s]->mutate();
}

//delete work;
}
*/

//void gaActionGroup::save(char *filename)
//{
//  int i;
//  FILE *fp = fopen(filename,"w");
//  fprintf(fp,"numSets: %d\n",numSets);
//  for (i=0;i<numSets;i++) {
//    actionSets[i]->save(fp);
//  }
//  fclose(fp);
//}

//void gaActionGroup::load(char *filename)
//{
//  int i;
//  FILE *fp = fopen(filename,"r");
//  fscanf(fp,"numSets: %d\n",&numSets);
//  for (i=0;i<numSets;i++) {
//    actionSets[i] = new gaActionSet();
//    actionSets[i]->load(fp);
//  }
//  fclose(fp);
//  currentSet = 0;
//}

/*
	////////////////////////////////////////
	if ((mSimStep>mDataBlock->mNumRestSteps)&&(mDone==1)) {
		mDone = 0;
		mSimStep = 0;
	} else if ((mSimStep<rest_time)&&(mDone==1)) {
		//resetPosition();	
		Ogre::Quaternion q;	
		q = Ogre::Quaternion::IDENTITY;
		for (i=1;i<mNumBodies;i++) {//0 has no joint
			physJoint *kJoint = mBodies[i]->getJoint();
			if (kJoint) kJoint->setMotorSpring(q,10.0);// 4X power
		}
	}
	if (!mDone) {
		unsigned int kSet = mActionGroup->mCurrSet;
		for (i=0;i < mActionGroup->mActionSets[kSet]->mNumActions;i++) {
			if ((mActionGroup->mActionSets[kSet]->mActions[i]->mStartStep <= mSimStep)
				&&(mActionGroup->mActionSets[kSet]->mActions[i]->mEndStep >= mSimStep)) 
			{
				//parts[mActionSet->mActions[i]->bodyID]->applyForces(&mActionSet->mActions[i]->mForces);
				work = mActionGroup->mActionSets[kSet]->mActions[i]->mForces;
				int rb = mActionGroup->mActionSets[kSet]->mActions[i]->mBodyIndex;	
				if (0) {
					work *= mBodies[rb]->getMaxTorque();
					mBodies[rb]->setCurrTorque(work);
				} else {
					Ogre::Quaternion q;
					q.set(Ogre::Vector3(work.x * M_PI/2.0, work.y * M_PI/2.0, 0.0));//TEMP
					mBodies[rb]->getJoint()->setMotorSpring(q,2.5);//(nxq,20000);
				}
			}   
		}
	}

	if ((mSimStep == mActionGroup->mActionSets[mActionGroup->mCurrSet]->mNumSteps)&&(!mDone)) {
		mSimStep=0;
		mDone = 1;
		mObsSeq->scoreSequence();
		mActionGroup->mActionSets[mActionGroup->mCurrSet]->setScore(mObsSeq->mScore);
		mObsSet->mSequences[mActionGroup->mCurrSet]->mScore = mObsSeq->mScore;
		mObsSeq->clear();

		//sprintf(filename,"neck_%d.action",total_sequence_count++);
		//mActionGroup->mActionSets[mActionGroup->mCurrSet]->save(filename);

		mActionGroup->mCurrSet++;

		if (mActionGroup->mCurrSet >= mActionGroup->mNumSets) {
			mObsSet->sort();
			saveAll();
			mActionGroup->repopulate();//config
			Con::errorf("repopulating!!!");
			mActionGroup->mCurrSet = 0;
			mCurrGen++;
		}
	}
	*/


/*
void gaActionUser::think()
{
	//HERE: actually apply the forces to the rigid bodies.  mRB->mCurrForce = actions[]->force;
	char filename[255];
	int i,m,s_m,o_i,id;//done=0;
	int rest_time = GA_NUM_REST_STEPS;
	float f;

	//Next step:  have actionState and goalState determine the reset mode and the 
	//choice of actions.  
	//if (actionstate==none) then start action state resetting, goal standing
	//if state resetting is done then do action state standing
	//if state standing is done then do action holding, goal standing is met
	//then change goals - walking forward, turning, rearing, jumping forward, ...

	mSimStep++;

	Ogre::Vector3 work; work = Ogre::Vector3::ZERO;

	///  1) OBSERVE
	o_i = mObsSeq->mObsInterval;
	if (((int)(mSimStep/o_i)*o_i) == mSimStep) observeBodyUp();

	if ((mSimStep>rest_time)&&(mDone==1)) {
		mDone = 0;
		mSimStep = 0;
	} else if ((mSimStep<rest_time)&&(mDone==1)) {
		//resetPosition();	
		Ogre::Quaternion q;	
		q = Ogre::Quaternion::IDENTITY;
		for (i=1;i<mNumBodies;i++) {//0 has no joint
			physJoint *kJoint = mBodies[i]->getJoint();
			if (kJoint) kJoint->setMotorSpring(q,10.0);// 4X power
		}
	}
	if (!mDone) {
		unsigned int kSet = mActionGroup->mCurrSet;
		for (i=0;i < mActionGroup->mActionSets[kSet]->mNumActions;i++) {
			if ((mActionGroup->mActionSets[kSet]->mActions[i]->mStartStep <= mSimStep)
				&&(mActionGroup->mActionSets[kSet]->mActions[i]->mEndStep >= mSimStep)) 
			{
				//parts[mActionSet->mActions[i]->bodyID]->applyForces(&mActionSet->mActions[i]->mForces);
				work = mActionGroup->mActionSets[kSet]->mActions[i]->mForces;
				int rb = mActionGroup->mActionSets[kSet]->mActions[i]->mBodyIndex;	
				if (0) {
					work *= mBodies[rb]->getMaxTorque();
					mBodies[rb]->setCurrTorque(work);
				} else {
					Ogre::Quaternion q;
					q.set(Ogre::Vector3(work.x * M_PI/2.0, work.y * M_PI/2.0, 0.0));//TEMP
					mBodies[rb]->getJoint()->setMotorSpring(q,2.5);//(nxq,20000);
				}
			}   
		}
	}

	if ((mSimStep == mActionGroup->mActionSets[mActionGroup->mCurrSet]->mNumSteps)&&(!mDone)) {
		mSimStep=0;
		mDone = 1;
		mObsSeq->scoreSequence();
		mActionGroup->mActionSets[mActionGroup->mCurrSet]->setScore(mObsSeq->mScore);
		mObsSet->mSequences[mActionGroup->mCurrSet]->mScore = mObsSeq->mScore;
		mObsSeq->clear();

		//sprintf(filename,"neck_%d.action",total_sequence_count++);
		//mActionGroup->mActionSets[mActionGroup->mCurrSet]->save(filename);

		mActionGroup->mCurrSet++;

		if (mActionGroup->mCurrSet >= mActionGroup->mNumSets) {
			mObsSet->sort();
			saveAll();
			mActionGroup->repopulate();//config
			Con::errorf("repopulating!!!");
			mActionGroup->mCurrSet = 0;
			mCurrGen++;
		}
	}
	return;
}*/
/*
void gaActionUser::observeHeadUp()
{
	if (mHeadIndex<0) return;

	int numParams = 0, numParts = 0;

	float sum;
	float params[GA_MAX_OBSERV_PARAMS];
	Ogre::Vector3 head; head = Ogre::Vector3::ZERO;

	sum = 0; numParts = 0;

	head = mBodies[mHeadIndex]->getLinearPosition();
	head -= mBodyStartPositions[mHeadIndex];
	sum += head.z; numParts++;

	sum = sum/(float)numParts;
	params[numParams] = sum;
	numParams++;

	mObsUnit->setObsData(numParams,params);
	mObsSeq->addUnit(mObsUnit);
}

void gaActionUser::observeBodyForward()
{
	int numParams = 0, numParts = 0;

	float sumA,sumX,sumY,sumZ;
	float params[GA_MAX_OBSERV_PARAMS];
	Ogre::Vector3 bodies[4],work;
	Ogre::Quaternion q;

	sumA = 0; sumX = 0; sumY = 0; sumZ = 0;  numParts = 0;

	q = mBodies[mBodyIndex]->getAngularPosition();
	float angB = q.angleBetween(mBodyStartQuats[mBodyIndex]);
	if (angB>(M_PI/2)) angB = M_PI - angB;

	q.inverse();

	for (int i=0;i<1;i++) 
	{
		bodies[i] = mBodies[mBodyIndex+i]->getLinearPosition();
		bodies[i] -= mBodyStartPositions[mBodyIndex+i];
		work = bodies[i];
		//q.mulP(bodies[i],&work);
		sumX -= fabs(work.x/3);
		sumY += work.y; 
		sumZ += work.z; 
		sumA -= fabs(angB);
		numParts++;
	}
	
	sumY = sumY/(float)numParts;
	params[numParams] = sumY;
	numParams++;

	sumX = sumX/(float)numParts;
	params[numParams] = sumX;
	numParams++;

	sumA = sumA/(float)numParts;
	params[numParams] = sumA;
	numParams++;

	//temp: got problems w/ undefined values for angB, trying to sanity check
	//if ((sumA<0)&&(sumA>-1000)) params[numParams] = sumA;
	//else params[numParams] = 0.0;
	//numParams++;

	mObsUnit->setObsData(numParams,params);
	mObsSeq->addUnit(mObsUnit);
}

void gaActionUser::observeDogChestForward()
{
	int numParams = 0, numParts = 0;

	float sumX,sumY,sumZ,sumA;
	float params[GA_MAX_OBSERV_PARAMS];
	Ogre::Vector3 bodies[4],work;
	Ogre::Quaternion q;

	sumZ = 0; sumX = 0; sumY = 0; sumA = 0; numParts = 0;

	//q = mBodies[mBodyIndex]->getAngularPosition();
	q = mBodyStartQuats[mBodyIndex];
	float angB = q.angleBetween(mBodyStartQuats[mBodyIndex]);
	if (angB>(M_PI/2)) angB = M_PI - angB;
	//angB *= -10.0;
	//Con::errorf("angleBetween: %f",angB);


	for (int i=1;i<2;i++) 
	{
		q = mBodyStartQuats[mBodyIndex+i];
		q.inverse();
		bodies[i] = mBodies[mBodyIndex+i]->getLinearPosition();
		bodies[i] -= mBodyStartPositions[mBodyIndex+i];
		work = bodies[i];
		sumX += work.x;//-=fabs(work.x);
		sumZ += work.z;
		sumY += work.y; 
		numParts++;
	}
	
	sumZ = sumZ/(float)numParts;
	params[numParams] = sumZ;
	numParams++;

	sumX = sumX/(float)numParts;
	sumY = sumY/(float)numParts;

	mObsUnit->setObsData(numParams,params);
	mObsSeq->addUnit(mObsUnit);
}

void gaActionUser::observeBodyUp()
{
	int numParams = 0, numParts = 0;

	float sum;
	float params[GA_MAX_OBSERV_PARAMS];
	Ogre::Vector3 res; res = Ogre::Vector3::ZERO;
	Ogre::Vector3 chest; chest = Ogre::Vector3::ZERO;
	Ogre::Vector3 body1; body1 = Ogre::Vector3::ZERO;
	Ogre::Vector3 body2; body2 = Ogre::Vector3::ZERO;
	Ogre::Vector3 body3; body3 = Ogre::Vector3::ZERO;

	sum = 0; numParts = 0;

	body1 = mBodies[mBodyIndex]->getLinearPosition();
	body1 -= mBodyStartPositions[mBodyIndex];
	body2 = mBodies[mBodyIndex+1]->getLinearPosition();
	body2 -= mBodyStartPositions[mBodyIndex+1];

	sum += body1.z; numParts++;
	sum += body2.z; numParts++;

	sum = sum/(float)numParts;
	params[numParams] = sum;
	numParams++;

	mObsUnit->setObsData(numParams,params);
	mObsSeq->addUnit(mObsUnit);
}



void gaActionUser::observeBodyBackward()
{
	int numParams = 0, numParts = 0;

	float sum,sumA,sumY;
	float params[GA_MAX_OBSERV_PARAMS];
	Ogre::Vector3 bodies[4],work;
	Ogre::Quaternion q;

	sum = 0.0; sumA = 0.0; sumY = 0.0; numParts = 0;

	q = mBodies[mBodyIndex]->getAngularPosition();
	float angB = q.angleBetween(mBodyStartQuats[mBodyIndex]);
	if (angB>(M_PI/2)) angB = M_PI - angB;
	angB *= -10.0;
	
	for (int i=0;i<1;i++) 
	{
		bodies[i] = mBodies[mBodyIndex+i]->getLinearPosition();
		bodies[i] -= mBodyStartPositions[mBodyIndex+i];
		sumY -= fabs(bodies[i].y)/2;//FIX make these relative to mObjToWorld, or mWorldToObj
		sum -= bodies[i].x;	// + sumY	
		sumA += angB/2;
		//sumA += angB;//try to stay pointing forward
		numParts++;
	}
	
	sum = sum/(float)numParts;
	params[numParams++] = sum;

	sumA = sumA/(float)numParts;
	params[numParams++] = sumA;

	sumY = sumY/(float)numParts;
	params[numParams++] = sumY;

	//Con::errorf("sum: %f, sumA: %f, sumY: %f",sum,sumA,sumY);

	mObsUnit->setObsData(numParams,params);
	mObsSeq->addUnit(mObsUnit);
}







*/