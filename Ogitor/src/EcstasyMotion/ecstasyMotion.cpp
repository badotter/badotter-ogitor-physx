////////////////////////////////////////////////////////////////////////////////////////////////////
//  nxPhysManager.cc
//  Chris Calef
//
//  adapted from LRGRigidBodyManager.cc
//  by Yossi Horowitz
//
//  Manages all of the Rigid Bodies in the scene, and serves as an interface between 
//  Torque and Novodex
////////////////////////////////////////////////////////////////////////////////////////////////////


#include "OgitorsPrerequisites.h"
#include "BaseEditor.h"
#include "OgitorsRoot.h"
#include "NodeEditor.h"
#include "EntityEditor.h"

#include "EcstasyMotion/ecstasyMotion.h"
#include "EcstasyMotion/nxPhysManager.h"
#include "EcstasyMotion/nxRigidBody.h"
#include "EcstasyMotion/fxRigidBody.h"
#include "EcstasyMotion/fxFlexBody.h"
#include "EcstasyMotion/SQLiteObject.h"

#include "OgitorsScriptInterpreter.h"
#include "OgitorsScriptConsole.h"

extern Ogitors::OgitorsScriptConsole *gConsole;
extern SQLiteObject *gSQL;


namespace EM {

	void sqlFindBodies(Ogitors::CEntityEditor *editorHandle)
	{
		std::ostringstream strResult;
		if (gSQL)
		{
			if (gSQL->OpenDatabase("EcstasyMotion.db"))
			{
				sqlite_resultset *resultSet;
				std::ostringstream queryRigid,queryFlex;
				int result;

				queryRigid << "SELECT id FROM fxFlexBodyData WHERE shapeFile = '" << 
					editorHandle->mMeshFile->get().c_str() << "';" ;
				result = gSQL->ExecuteSQL(queryRigid.str().c_str());
				if (result)
				{
					resultSet = gSQL->GetResultSet(result);
					if (resultSet<=0)
					{
						strResult << "database opened, resultset null!" ;
					} else {
						if (resultSet->iNumRows == 0)	
						{
							strResult << "database opened, query empty!" ;
						} else {
							if (resultSet->iNumRows > 1)
							{
								strResult << "query returned multiple flexbody matches for one shapeFile: "  
									<< editorHandle->mMeshFile->get().c_str() ;
							} else {
								strResult << "One fxFlexBodyData shape found for mesh " << 
									editorHandle->mMeshFile->get().c_str() << " id: " << strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10) ;
								//resultSet->vRows[0]->vColumnValues[1]);
								//strtod(resultSet->vRows[0]->vColumnValues[2],NULL));
								//here: call new fxRigidBody()
								fxFlexBody  *fxFB = new fxFlexBody(editorHandle->mEntityHandle,editorHandle->mMeshFile->get().c_str());
								//Next: store the pointer somewhere so it gets deleted when we clear the scene.
								//Maybe make physManager find all "fx" level objects when it deletes its lists?
								gConsole->addOutput(strResult.str());
								return;//If you find a flexbody, bail, otherwise keep looking for rigid bodies.
							}
						}
					}
				} else {
					strResult << "database opened, result null!" ;
				}
				gConsole->addOutput(strResult.str());

				//dangit, how do you set an ostringstream to empty?  can't assign  = ""
				//and I think neither flush nor clear does this.
				queryFlex << "SELECT id FROM fxRigidBodyData WHERE shapeFile = '" << 
					editorHandle->mMeshFile->get().c_str() << "';" ;
				result = gSQL->ExecuteSQL(queryFlex.str().c_str());
				if (result)
				{
					resultSet = gSQL->GetResultSet(result);
					if (resultSet<=0)
					{
						strResult << "database opened, resultset null!" ;
					} else {
						if (resultSet->iNumRows == 0)	
						{
							strResult << "database opened, query empty!" ;
						} else {
							if (resultSet->iNumRows > 1)
							{
								strResult << "query returned multiple rigidbody matches for one shapeFile: "  
									<< editorHandle->mMeshFile->get().c_str() ;
							} else {
								strResult << "One fxRigidBodyData shape found for mesh " << 
									editorHandle->mMeshFile->get().c_str() << " id:  " << strtol(resultSet->vRows[0]->vColumnValues[0],NULL,10) ;
								//resultSet->vRows[0]->vColumnValues[1]);
								//strtod(resultSet->vRows[0]->vColumnValues[2],NULL));
								//here: call new fxRigidBody()
								fxRigidBody  *fxRB = new fxRigidBody(editorHandle->mEntityHandle,editorHandle->mMeshFile->get().c_str());
								//Next: store the pointer somewhere so it gets deleted when we clear the scene.
							}
						}
					}
				} else {
					strResult << "database opened, result null!" ;
				}
				gSQL->CloseDatabase();

			} else {
				strResult << "database failed!" ;
			}
		} else {
			strResult << "don't even have gSQL!" ;
		}
		gConsole->addOutput(strResult.str());
	}





}