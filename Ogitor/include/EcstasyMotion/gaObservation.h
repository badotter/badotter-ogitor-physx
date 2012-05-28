#ifndef GA_OBSERVATION_H
#define GA_OBSERVATION_H 
//#include <ode/ode.h>
//#include <drawstuff/drawstuff.h>
//#include "fourLeggedTable.h"
//#include "Vect3.h"


//#include "platform/platform.h"
//#include "console/consoleTypes.h"
//#include "math/mathio.h"
//#include "math/mathUtils.h"
//#include "math/mRandom.h"

//NUM_OBSERVE_PARAMS is floats, not vectors, so doesn't
//need to be divisible by three       (???)

#define GA_MAX_OBSERV_PARAMS            25
#define GA_MAX_OBSERV_SEQUENCE_UNITS    500
#define GA_MAX_OBSERV_SET_SEQUENCES     150

class gaObservationUnit
{
public:
  int mNumParams;
  float mObservParams[GA_MAX_OBSERV_PARAMS];

  gaObservationUnit();
  gaObservationUnit(gaObservationUnit *);
  gaObservationUnit(int,float *);
  ~gaObservationUnit();

  void setObsData(gaObservationUnit *);
  void setObsData(int, float *);

};

class gaObservationSequence 
{
public:
  int mStartStep;
  int mEndStep;
  int mNumSteps;

  int mNumUnits;
  int mCurrUnit;

  int mObsInterval;

  float mScore;

  gaObservationUnit *mUnits[GA_MAX_OBSERV_SEQUENCE_UNITS];

  gaObservationSequence();
  gaObservationSequence(int);
  gaObservationSequence(gaObservationSequence *);
  ~gaObservationSequence();
  void addUnit(gaObservationUnit *);
  void scoreSequence();
  //void copy(gaObservationSequence *other);
  //void swap(gaObservationSequence *other);
  void clear(); 
};

class gaObservationSequenceSet
{
public:
  int mNumSequences;
  int mCurrSequence;

  gaObservationSequence *mSequences[GA_MAX_OBSERV_SET_SEQUENCES];
  
  gaObservationSequenceSet(int,int);
  ~gaObservationSequenceSet();  
  void addSequence(gaObservationSequence *);
  void sort();
  void clear();
};


#endif //GA_OBSERVATION_H
