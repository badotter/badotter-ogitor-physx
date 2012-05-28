#include "EcstasyMotion/gaObservation.h"
#include "EcstasyMotion/gaAction.h"
//#include "mathutil.h"


//extern int simStep;
//int simStep = 0;
//extern int numCollisions;

extern int sort_list[100];//FIX FIX FIX 


gaObservationUnit::gaObservationUnit() 
{
	mNumParams = 0;
}

gaObservationUnit::gaObservationUnit(gaObservationUnit *other) 
{
	int i;
	mNumParams = other->mNumParams;
	for (i=0;i<mNumParams;i++) {
		mObservParams[i] = other->mObservParams[i];
	}
}

gaObservationUnit::gaObservationUnit(int num,float *params) 
{
	int i;
	mNumParams = num;
	for (i=0;i<num;i++) {
		mObservParams[i] = params[i];
	}
}

gaObservationUnit::~gaObservationUnit() 
{

}

void gaObservationUnit::setObsData(gaObservationUnit *other)
{
	int i;
	mNumParams = other->mNumParams;
	for (i=0;i<GA_MAX_OBSERV_PARAMS;i++) {
		mObservParams[i] = other->mObservParams[i];
	}
}

void gaObservationUnit::setObsData(int num,float *params) 
{
	int i;
	mNumParams = num;
	for (i=0;i<num;i++) {
		mObservParams[i] = params[i];
	}
}
///////////////////////////////////

gaObservationSequence::gaObservationSequence()
{
	mStartStep = 0;
	mEndStep = 0;
	mNumSteps = 0;
	mNumUnits = 0;
	mCurrUnit = 0;
	mObsInterval = 5;
	mScore = 0.0;
}

gaObservationSequence::gaObservationSequence(int obs_interval)
{
	mStartStep = 0;
	mEndStep = 0;
	mNumSteps = 0;
	mNumUnits = 0;
	mCurrUnit = 0;
	mObsInterval = obs_interval;
	mScore = 0.0;
}

gaObservationSequence::gaObservationSequence(gaObservationSequence *other)
{
	int i;
	//FILE *debug = fopen("debug.txt","a");
	//fprintf(debug,"numUnits = %d, score = %f\n",other->numUnits,other->score);
	//for (i=0;i<other->numUnits;i++) {
	//fprintf(debug,"unit %d: %f\n",i,other->units[i]);
	//}
	// 
	//startStep = other->startStep;
	//endStep = other->endStep;
	//numSteps = other->numSteps;
	mNumUnits = other->mNumUnits;
	mObsInterval = other->mObsInterval;
	//currentUnit = other->currentUnit;
	mScore = other->mScore;
	//printf("new sequence: score %f, units %d\n",score,numUnits);
	//fprintf(debug,"new sequence: score %f, units %d\n",score,numUnits);
	//for (i=0;i<mNumUnits;i++) {
	//fprintf(debug,"units[%d] %f\n",i,other->units[i]->observParams[0]);
	//addUnit(other->units[i]);

	//once I have the score, and this sequence is rated, I don't care about
	//the array of units anymore.
	//but, better be careful, because you are going to try to delete them.
	//}
	//fclose(debug);
}

gaObservationSequence::~gaObservationSequence()
{
	int i;
	for (i=0;i<mNumUnits;i++) {
		delete mUnits[i];
	}
}

void gaObservationSequence::addUnit(gaObservationUnit *new_unit) 
{
	mUnits[mNumUnits++] = new gaObservationUnit(new_unit);

}

void gaObservationSequence::scoreSequence()
{
	int i,j;
	float avg,sum;
	float paramSum[GA_MAX_OBSERV_PARAMS];

	sum = 0.0;

	for (i=0;i<GA_MAX_OBSERV_PARAMS;i++) {
		paramSum[i] = 0.0;
	}

	if (mNumUnits > 0) 
	{
		for (i=0;i<mNumUnits;i++) 
		{
			for (j=0;j<mUnits[i]->mNumParams;j++) 
			{
				sum += mUnits[i]->mObservParams[j];
				paramSum[j] += mUnits[i]->mObservParams[j];
			}

			//fourLegged forward			
			//paramSum[0] += mUnits[i]->mObservParams[0];//forward
			//paramSum[1] += mUnits[i]->mObservParams[1];//angle
			//paramSum[2] += units[i]->observParams[2];//front feet
			//paramSum[3] += units[i]->observParams[3];//head
			//paramSum[4] += units[i]->observParams[4];//energy

			//dragon_1 walk
			//body_dist += units[i]->observParams[0];
			//feet += units[i]->observParams[1];
			//head += units[i]->observParams[2];
			//body_height += units[i]->observParams[3];
			//collisions += units[i]->observParams[4];

		}


		avg = (sum / mNumUnits);
		mScore = avg;

		for (j=0;j<mUnits[0]->mNumParams;j++) 
			paramSum[j] /= mNumUnits;

		//traveling /= numUnits;
		//Con::errorf("X: %3.2f, Y: %3.2f, Z %3.2f, A %3.2f",paramSum[0],paramSum[1],paramSum[2],paramSum[3]);

		mNumUnits = 0;
	}
}

//FILE *debug = fopen("debug.txt","a");
//fprintf(debug,"score - numUnits = %d, avg = %f\n",numUnits,score);
//fclose(debug);


//height_sum += units[i]->observParams[0];
//planar_dist_sum += units[i]->observParams[1];
////legs_sum += units[i]->observParams[2]; //don't really have problem of legs flinging up in the air anymore
//contacts_sum += units[i]->observParams[3];
//HERE add in other factors: 
//  * distance from starting point in x/y plane, to discourage behaviors that jump up and away,
//  * number of points in contact with the ground, from two to four acceptable, one or zero penalized heavily
//  * angle of body "up" vector in global units
//  (These will be in different units, so factor them all over a 0.0 - 1.0 range and let them add up.  Since 
//  the sum will include maybe five or six or more separate components, a low score in one of them won't 
//  necessarily disqualify it.)
//sum = height_sum + planar_dist_sum + legs_sum + contacts_sum;

//void gaObservationSequence::swap (gaObservationSequence *other) 
//{
//  int i;
//  gaObservationSequence *temp = new gaObservationSequence(this);
//  startStep = 0;
//  endStep = 0;
//  numSteps = 0;
//  for (i=0;i<numUnits;i++) { delete units[i]; }
//  numUnits = 0;
//  currentUnit = 0;
//  score = 0.0;
//  }

void gaObservationSequence::clear() 
{
	int i;
	mStartStep = 0;
	mEndStep = 0;
	mNumSteps = 0;
	for (i=0;i<mNumUnits;i++) { delete mUnits[i]; }
	mNumUnits = 0;
	mCurrUnit = 0;
	mScore = 0.0;

}
//for (j=0;j<numParams;j++) {
//}

///////////////////////////////////

gaObservationSequenceSet::gaObservationSequenceSet(int num_action_sets,int obs_int)
{
	int i;

	mNumSequences = num_action_sets;
	mCurrSequence = 0;

	for (i=0;i<mNumSequences;i++) {
		mSequences[i] = new gaObservationSequence(obs_int);
	}//Hmm, is this necessary to do in advance like this?
}

gaObservationSequenceSet::~gaObservationSequenceSet()
{
	int i;
	for (i=0;i<mNumSequences;i++) {
		delete mSequences[i];
	}
}

void gaObservationSequenceSet::addSequence(gaObservationSequence *seq)
{
	mSequences[mNumSequences++] = new gaObservationSequence(seq);
}

//int sort_list[MAX_ACTION_SEQUENCES_PER_SET];//whoops, needs to be global so it 
//doesn't disappear when sort is finished. Do something better with this.

void gaObservationSequenceSet::sort() 
{
	int i,j,temp;//sort_list[10];//FIX 

	//printf("starting sort ",numSequences);

	//printf("++++++++++++ starting sort: numsequences %d, %1.2f %1.2f %1.2f %1.2f +++++++++++\n",numSequences,sequences[0]->score,sequences[1]->score,sequences[2]->score,sequences[3]->score);
	//first set up the list as sort_list[0] = 0,[1] = 1, etc.
	for (i=0;i<mNumSequences;i++) sort_list[i] = i;

	//numSequences?

	for (i=0;i<mNumSequences;i++) {
		for (j=0;j<(mNumSequences-1);j++) {
			if (mSequences[sort_list[j]]->mScore < mSequences[sort_list[j+1]]->mScore) {
				//printf("switching %d (%f) for %d (%f)\n",j,sequences[j]->score,j+1,sequences[j+1]->score);
				temp = sort_list[j];
				sort_list[j] = sort_list[j+1];
				sort_list[j+1] = temp;
			}
		}
	}
	//printf("\n");
	//Con::printf("sorted list, winners are: %d and %d",sort_list[0],sort_list[1]);
	return;// sort_list;
}

void gaObservationSequenceSet::clear()
{//after scoring a whole generation and repopulating, delete all to prepare for next set of observations
	int i;
	for (i=0;i<mNumSequences;i++) {
		delete mSequences[i];
	}
	mNumSequences = 0;

	//for (i=0;i<numSequences;i++) {
	//delete sequences[i];
	//}
	//numSequences = 0;
}


//////////////////////

