///Modular Zone Plugin
///
/// Copyright (c) 2009 Gary Mclean
//
//This program is free software; you can redistribute it and/or modify it under
//the terms of the GNU Lesser General Public License as published by the Free Software
//Foundation; either version 2 of the License, or (at your option) any later
//version.
//
//This program is distributed in the hope that it will be useful, but WITHOUT
//ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
//FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
//
//You should have received a copy of the GNU Lesser General Public License along with
//this program; if not, write to the Free Software Foundation, Inc., 59 Temple
//Place - Suite 330, Boston, MA 02111-1307, USA, or go to
//http://www.gnu.org/copyleft/lesser.txt.
////////////////////////////////////////////////////////////////////////////////*/

#include "OgitorsPrerequisites.h"
#include "OgitorsSystem.h"
#include "BaseEditor.h"
#include "OgitorsRoot.h"
#include "NodeEditor.h"
#include "ModularZoneEditor.h"
#include "ModularZoneFactory.h"
#include "PortalOutlineRenderable.h"
#include "PortalEditor.h"
#include "PortalFactory.h"
#include "ZoneInfo.h"
#include "Ogre.h"
#include "tinyxml.h"
#include "exportzonedialog.hxx"
#include "portalsizedialog.hxx"
#include <QtGui/QFileDialog>
#include <QtCore/QString>
#include <numeric>



using namespace Ogitors;

namespace MZP
{

//-----------------------------------------------------------------------------------------
ModularZoneEditor::ModularZoneEditor(Ogitors::CBaseEditorFactory *factory) :	
																				Ogitors::CNodeEditor(factory),
																				mZoneMesh(0),
																				mPortalLocked(false),
																				mDesignInfo(0),
																				mDragging(false)
{
    mHandle = 0;
    mUsesGizmos = true;
    mUsesHelper = false;

}

void ModularZoneEditor::showBoundingBox(bool bShow) 
{
    if(!mBoxParentNode)
        createBoundingBox();
	
    mBBoxNode->setVisible(bShow);
}
//-----------------------------------------------------------------------------------------

bool ModularZoneEditor::_setZoneTemplate(OgitorsPropertyBase* property, const int& value)
{
	mZoneDescription->set(value);
	return true;
}

//-----------------------------------------------------------------------------------------
bool ModularZoneEditor::_setMeshName(OgitorsPropertyBase* property, const Ogre::String& value)
{
	mMeshName->set(value);
	return true;
}
//-----------------------------------------------------------------------------------------
bool ModularZoneEditor::_setPortalCount(OgitorsPropertyBase* property, const int& value)
{
	mPortalCount->set(value);
	return true;
}

bool ModularZoneEditor::_setPortal(OgitorsPropertyBase* property, const Ogre::String& value)
{

	PortalEditor* portal;
	portal = dynamic_cast<PortalEditor*>(OgitorsRoot::getSingletonPtr()->FindObject(value));
	if(portal)
	{
		Ogre::Any test = property->getValue();
		mPortals.push_back(portal);
	}
	return true;
}
//-----------------------------------------------------------------------------------------
void ModularZoneEditor::createProperties(Ogitors::OgitorsPropertyValueMap &params)
{
	PROPERTY_PTR(mZoneDescription, "zonetemplate", int,-1, 0, SETTER(int, ModularZoneEditor, _setZoneTemplate));
	PROPERTY_PTR(mMeshName, "meshname", Ogre::String, "", 0, SETTER(Ogre::String, ModularZoneEditor, _setMeshName));

    PROPERTY_PTR(mPortalCount, "portalcount", int , -1  ,0, SETTER(int,ModularZoneEditor,_setPortalCount));

	//set up portal properties
	int count = 0;
	OgitorsPropertyValueMap::const_iterator it = params.find("portalcount");
	if(it != params.end())
		count = Ogre::any_cast<int>(it->second.val);

	for(int ix = 0;ix < count;ix++)
    {

		Ogre::String propname = "portal " + Ogre::StringConverter::toString(ix);
		mFactory->AddPropertyDefinition(propname,"Zone::Portals::"+propname,"List of the attached portals",PROP_STRING,true,false,true);
		PROPERTY(propname, Ogre::String, "" , ix, SETTER(Ogre::String, ModularZoneEditor, _setPortal)); 

    }

	mProperties.initValueMap(params);
}
//-----------------------------------------------------------------------------------------
bool ModularZoneEditor::load(bool async)
{
    if(mLoaded->get())
        return true;


    
    if(Ogitors::CNodeEditor::load())
    {

		if(mPortalCount->get() == -1)//means this is new, not loaded from ogscene file
			_loadZoneDescription(mZoneDescription->get());


		mZoneMesh = Ogitors::OgitorsRoot::getSingletonPtr()->GetSceneManager()->createEntity(mName->get(),mMeshName->get());
        mHandle->attachObject(mZoneMesh);

		mZoneMesh->setQueryFlags(QUERYFLAG_MOVABLE);
        mZoneMesh->setCastShadows(false);


    }
    else
        return false;

    return true;
}
//-----------------------------------------------------------------------------------------
bool ModularZoneEditor::unLoad()
{
    if(!mLoaded->get())
        return true;

    if(mZoneMesh)
    {
		mZoneMesh->detachFromParent();
        mZoneMesh->_getManager()->destroyEntity(mZoneMesh);
        mZoneMesh = 0;
		mPortals.clear();
    }
	

	    
    return Ogitors::CNodeEditor::unLoad();
}
//----------------------------------------------------------------------------------------
bool ModularZoneEditor::_loadZoneDescription(int key)
{
	ZoneInfo* info = dynamic_cast<ModularZoneFactory*> (mFactory)->getZoneTemplate(key);

	mMeshName->set(info->mMesh);
	mPortalCount->set(info->mPortalCount);

	//get the portal factory:
	CBaseEditorFactory* portalFactory = OgitorsRoot::getSingletonPtr()->GetEditorObjectFactory("MZ Portal Object");
	int count = 0;
	int i;
	for( i = 0; i<info->mPortalCount;i++)
	{
		PortalEditor* portal = 0;
		Ogre::Vector3 position(info->mPortals[i].mPosition);
		Ogre::Quaternion orientation(info->mPortals[i].mOrientation);
		Ogre::Real width = info->mPortals[i].mWidth;
		Ogre::Real height = info->mPortals[i].mHeight;
		
		OgitorsPropertyValueMap params;
		OgitorsPropertyValue propValue;
		params["init"] = EMPTY_PROPERTY_VALUE;

		propValue.propType = PROP_VECTOR3;
		propValue.val = Ogre::Any(position);
		params["position"] = propValue;

		propValue.propType = PROP_QUATERNION;
		propValue.val = Ogre::Any(orientation);
		params["orientation"] = propValue;

		propValue.propType = PROP_REAL;
		propValue.val = Ogre::Any(width);
		params["width"] = propValue;

		propValue.propType = PROP_REAL;
		propValue.val = Ogre::Any(height);
		params["height"] = propValue;

		CBaseEditor *parent = this;


		if(!mDragging)//This is a hack to stop it crashing during drag'n'drop
		{
			//don't create portals when the zone is being dragged - they will cause a crash - I don't know why
			portal = dynamic_cast<PortalEditor*>(OgitorsRoot::getSingletonPtr()->CreateEditorObject(parent,"MZ Portal Object",params,true,true));
		}
		if(portal)
		{
			Ogre::String propname = "portal " + Ogre::StringConverter::toString(count);
			mFactory->AddPropertyDefinition(propname,"Zone::Portals::"+propname,"List of the attached portals",PROP_STRING,true,false,true);
			PROPERTY(propname, Ogre::String, portal->getName(), count, SETTER(Ogre::String, ModularZoneEditor, _setPortal)); 
			++count;

		}
	}

	return false;
}
//----------------------------------------------------------------------------------------
bool ModularZoneEditor::update(float timePassed)
{
	//TODO: check for proximity to other portals
	//trigger snap-to when close enough.
	bool bModified = false;//return value for Ogitor
	bool bAllowMove = true;//once we snapto one portal, just check connections for others

	if (getSelected())
	{
		//for each of this zone's portal, do proximity check
		//std::for_each(mPortals.begin(),mPortals.end(),std::mem_fun(&PortalEditor::connectNearPortals) );

		std::vector<PortalEditor*>::iterator itr;
		for (itr = mPortals.begin();itr != mPortals.end();++itr)
		{
			if((*itr)->connectNearPortals(bAllowMove))bAllowMove = false;
		}
		bModified = true;
		
	}

	return bModified;
}
//----------------------------------------------------------------------------------------
bool ModularZoneEditor::getObjectContextMenu(UTFStringVector &menuitems)
{
	menuitems.clear();

	if(mDesignInfo) //if there is a potential destination...
	{
		menuitems.push_back(OTR("Add Portal"));
		menuitems.push_back(OTR("Update")); //sends any changes to the zone selection widget
		menuitems.push_back(OTR("Export..."));//export as a .zone file
	}
	
    return true;
}
//----------------------------------------------------------------------------------------
void ModularZoneEditor::onObjectContextMenu(int menuresult)
{

	switch(menuresult)
	{
	case 0: addPortal();break;//"Add Portal"
	case 1: updateDesignInfo();break;//"Save Design"
	case 2: exportZone();break;//"Export Design"
	}
   
}
//----------------------------------------------------------------------------------------
void ModularZoneEditor::setPortalLocked(bool lock)
{
	mPortalLocked= lock;
	this->setLocked(lock);
	//PROBLEM: really it should only unlock the NodeEditor
	//if it was locked by locking the Zone.
	//if the user locked it for whatever reason before
	//the portal lock, then when the portal lock
	//is removed, the node lock should remain.

}
//----------------------------------------------------------------------------------------
void ModularZoneEditor::addPortal(PortalEditor* portal)
{
	mPortals.push_back(portal);
}
//----------------------------------------------------------------------------------------
void ModularZoneEditor::addPortal(void)
{
	//Add a new portal to the zone
	//it will appear at the centre of the zone - the user needs to position it
	if(mDesignInfo)
	{
					//open resize menu
		PortalSizeDialog dlg;
		Ogre::Real x,y;
		if(dlg.exec())//OK
		{
			//retrieve new portal dimensions
			x = dlg.getWidth();
			if(x == 0)x = 2.0;//TODO: should move validation to dialog class
			y = dlg.getHeight();
			if(y == 0)y = 2.0; 
							
		}
		else
		{
			x = PortalEditor::DEFAULT_WIDTH; 
			y = PortalEditor::DEFAULT_HEIGHT;
		}
		//set up properties for the new portal
		Ogitors::OgitorsPropertyValueMap params,zoneparams;
		OgitorsPropertyValue propValue;
		this->getPropertyMap(zoneparams);
		//set the init param to create a new object
		params["init"] = EMPTY_PROPERTY_VALUE;
		params["parentnode"] = zoneparams["name"];
		propValue.propType = PROP_STRING;
		propValue.val = Ogre::Any(Ogre::String(""));
		params["destination"] = propValue;
		propValue.propType = PROP_REAL;
		propValue.val = Ogre::Any(x);
		params["width"] = propValue;
		propValue.val = Ogre::Any(y);
		params["height"]= propValue;
		propValue.propType = PROP_VECTOR3;
		propValue.val = Ogre::Any(Ogre::Vector3(0,0,0));
		params["position"]= propValue;
		propValue.propType = PROP_QUATERNION;
		propValue.val = Ogre::Any(Ogre::Quaternion(1,0,0,0));
		params["orientation"]= propValue;
		//create a portal
		//Because the portal is a created as child to this zone, no need to call AddPortal(portal)
		//portal will add itself automatically
		OgitorsRoot* root = OgitorsRoot::getSingletonPtr();
		PortalEditor* portal = dynamic_cast<PortalEditor*>(root->CreateEditorObject(0, "MZ Portal Object",params,true,true));
		portal->setLocked(false);//we need to be able to move new portal into position

		//add the PortalInfo to the ZoneInfo
		PortalInfo portalInfo;
		portalInfo.mHeight = Ogre::any_cast<Ogre::Real>(params["height"].val);
		portalInfo.mWidth = Ogre::any_cast<Ogre::Real>(params["width"].val);
		portalInfo.mPosition = Ogre::any_cast<Ogre::Vector3>(params["position"].val);
		portalInfo.mOrientation = Ogre::any_cast<Ogre::Quaternion>(params["orientation"].val);
		mDesignInfo->mPortalCount++;
		mDesignInfo->mPortals.push_back(portalInfo);
	}
}
//----------------------------------------------------------------------------------------
void ModularZoneEditor::updateDesignInfo(void)
{
	//
	if(mDesignInfo)
	{
		Ogitors::OgitorsPropertyValueMap params;
		OgitorsPropertyValue propValue;
		//clear the old portal info
		mDesignInfo->mPortals.clear();
		mDesignInfo->mPortalCount = 0;
		//get each portals and update the info
		std::vector<PortalEditor*>::iterator itr;
		for(itr = mPortals.begin();itr!=mPortals.end();++itr)
		{
			(*itr)->getPropertyMap(params);
			PortalInfo portalInfo;
			portalInfo.mHeight = Ogre::any_cast<Ogre::Real>(params["height"].val);
			portalInfo.mWidth = Ogre::any_cast<Ogre::Real>(params["width"].val);
			portalInfo.mPosition = Ogre::any_cast<Ogre::Vector3>(params["position"].val);
			portalInfo.mOrientation = Ogre::any_cast<Ogre::Quaternion>(params["orientation"].val);
			mDesignInfo->mPortalCount++;
			mDesignInfo->mPortals.push_back(portalInfo);
		}

		//get the zone widget to update the ToolTip of the zone item 
		ModularZoneFactory* factory = dynamic_cast<ModularZoneFactory*>(mFactory);//get the  ModularZoneFactory
		getPropertyMap(params);
		int key = Ogre::any_cast<int>(params["zonetemplate"].val);
		factory->updateZoneListWidget(key);
	}
}
//----------------------------------------------------------------------------------------
void ModularZoneEditor::exportZone(void)
{
	//get name, short description & long description
	//get destination & (if destination is not in current projects resources) add to project

	//update the ZoneInfo
	updateDesignInfo();

	//write the .zone file
	ExportZoneDialog dlg;
	//get the zone description info
	if(dlg.exec())
	{
		//get file name 
		Ogre::String fileName = QFileDialog::getSaveFileName(0,"Save ZONE file",".","ZONE files (*.zone)").toStdString();
		
		if(!fileName.empty())
		{
			//store the info
			mDesignInfo->mName = fileName;
			mDesignInfo->mShortDesc = dlg.getShortDescription();
			mDesignInfo->mLongDesc = dlg.getLongDescription();

			//export the file
			std::ofstream outfile(fileName.c_str());
			//XML header
			outfile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
			//Zone Description 
			outfile << "<zonedescription " 
					<< "mesh=\"" 
					<< mDesignInfo->mMesh 
					<<"\" portals=\""
					<<mDesignInfo->mPortalCount 
					<<"\" short=\""
					<<mDesignInfo->mShortDesc
					<<"\" long=\""
					<<mDesignInfo->mLongDesc
					<<"\">\n";
			//Portal descriptions
			std::vector<PortalInfo>::iterator itr;
			for(itr = mDesignInfo->mPortals.begin();itr != mDesignInfo->mPortals.end();++itr)
			{
				outfile << "<portal width = \"" << (*itr).mWidth << "\" height=\"" << (*itr).mHeight << "\">\n";
				outfile << "<position x=\""<< (*itr).mPosition.x <<"\" y=\"" << (*itr).mPosition.y << "\" z=\"" << (*itr).mPosition.z << "\"/>\n";
				outfile << "<orientation x=\""<< (*itr).mOrientation.x <<"\" y=\"" << (*itr).mOrientation.y << "\" z=\"" << (*itr).mOrientation.z << "\" w = \"" << (*itr).mOrientation.w << "\"/>\n";
				outfile << "</portal>\n";
			}
			outfile << "</zonedescription>\n";
			outfile.close();
		}
	
	}
}
//----------------------------------------------------------------------------------------
void ModularZoneEditor::setSelectedImpl(bool bSelected)
{
	
	if (bSelected)
	{
		
		//register for updates
		OgitorsRoot::getSingletonPtr()->RegisterForUpdates(this);
	}
	else
	{
		//unregister
		OgitorsRoot::getSingletonPtr()->UnRegisterForUpdates(this);
	}
	
	CBaseEditor::setSelectedImpl(bSelected);

}
//----------------------------------------------------------------------------------------
void ModularZoneEditor::setDesignMode(void)
{
	if(!mDesignInfo)
	{
		//get the  ModularZoneFactory
		ModularZoneFactory* factory = dynamic_cast<ModularZoneFactory*>(mFactory);
		//create a blank ZoneInfo
		ZoneInfo zone;
		
		//add to to the end of the factory's Zone templates map
		int key = factory->addZoneTemplate(zone);
		//retrieve a pointer to the ZoneInfo so we can edit it
		mDesignInfo = factory->getZoneTemplate(key);
		//get this zones property map
		Ogitors::OgitorsPropertyValueMap params;
		this->getPropertyMap(params);
		//add the zone template key
		mZoneDescription->set(key);
		//set the mesh name in the zone info
		mDesignInfo->mMesh = Ogre::any_cast<Ogre::String>(params["meshname"].val);
		//display the mesh in wireframe
		this->mZoneMesh->setMaterialName("scbMATWIREFRAME");
		this->mDesignTools = new ZoneDesignTools;
		enableMeshHolesTool(true);

	}
}
//----------------------------------------------------------------------------------------
void ModularZoneEditor::enableMeshHolesTool(bool enable)
{
	Ogre::EdgeData* edgedata = mZoneMesh->getEdgeList();
    Ogre::Vector3* vertices;

	if(!edgedata->isClosed)
	{
		//look for the holes and add helpers
		Ogre::EdgeData::EdgeGroupList::iterator group; 
		
		/*std::vector<Ogre::EdgeData::EdgeList>::iterator */
		std::vector<std::pair<Ogre::Vector3,Ogre::Vector3> > degenEdges;
		std::vector<std::vector<std::pair<Ogre::Vector3,Ogre::Vector3> > > loops;
		std::vector<std::vector<std::pair<Ogre::Vector3,Ogre::Vector3> > >::iterator itrLoop;
		for(group = edgedata->edgeGroups.begin();group != edgedata->edgeGroups.end();++group)
		{
			//collect all the degenerate edges in a nice container
			Ogre::EdgeData::EdgeList::iterator edge;
			getVertexData((*group).vertexData,vertices);//get the verts for this group
			for(edge = (*group).edges.begin();edge != (*group).edges.end();++edge)
			{
				if((*edge).degenerate)
				{
					//store as a pair of verts in degenEdges
					degenEdges.push_back(std::pair<Ogre::Vector3,Ogre::Vector3>(vertices[(*edge).vertIndex[0]],vertices[(*edge).vertIndex[1]]));
				}
			}
			delete[] vertices;//clean up
		}
		
		while(!degenEdges.empty())//while we still have edges
		{
			std::vector<std::pair<Ogre::Vector3,Ogre::Vector3> > loopEdges;
			std::pair<Ogre::Vector3,Ogre::Vector3> start = (*(--degenEdges.end()));//get last edge from list
			Ogre::Vector3 endVertex = start.second;
			loopEdges.push_back(start);
			degenEdges.pop_back();//remove it from list
			bool finished = false;
			while(!finished)
			{
				std::vector<std::pair<Ogre::Vector3,Ogre::Vector3> >::iterator match;
				match = std::find_if(degenEdges.begin(),degenEdges.end(),std::bind1st(MatchEdge(),endVertex));
				if(match != degenEdges.end())
				{

					loopEdges.push_back((*match));
					
					Ogre::Vector3 nextVert; //which of the 2 verts was matched?
					if((*match).first == endVertex)
					{
						nextVert = (*match).second;
					}
					else
					{
						nextVert = (*match).first;
					}

					degenEdges.erase(match);//remove from list

					//test if loop is closed
					if(start.first == nextVert)
					{
						bool coplanar = true;
						//loop is closed
						if(loopEdges.size()>3)//we only need to do plane test for 4 or more vertices
						{
							//test that verts are coplanar
							
							Ogre::Plane plane(loopEdges[0].second,loopEdges[1].second,loopEdges[2].second);
							plane.normalise();//needed for the distance test
							const Ogre::Real DIST_EPSILON(0.0001f);//fudge factor

							for(unsigned int i = 3;i<loopEdges.size()&&coplanar;++i)
							{
								//test the remaining vertices against the plane:
								//if(plane.getSide(loopEdges[i].second)!=Ogre::Plane::NO_SIDE)//this test is TOO strict
								if(plane.getDistance(loopEdges[i].second)>DIST_EPSILON)
								{
									coplanar = false;
								}
							}


						}

						if(coplanar)
						{
							loops.push_back(loopEdges);
						}

					}
					else
					{
						//loop is open - continue looking for edges
						endVertex = nextVert;
					}
				}
				else
				{
					//no more matches found
					finished = true;
				}
			}
			

		}
			
		//iterate thru edge loops and add snap-to points
		for(itrLoop = loops.begin();itrLoop != loops.end();++itrLoop)
		{
			//work out centre of loop
			//(for the moment only do quads - my brain has stopped working) ;)
			if((*itrLoop).size()==4)//TODO: handle non quads
			{	
				Ogre::Plane loopPlane((*itrLoop)[0].first,(*itrLoop)[1].first,(*itrLoop)[2].first);
				loopPlane.normalise();

				Ogre::Vector3 position;

				//hacky solution until I get centroid to work for all cases
				Ogre::Real d1 = ((*itrLoop)[0].first.distance((*itrLoop)[2].first));
				Ogre::Real d2 = ((*itrLoop)[1].first.distance((*itrLoop)[3].first));
				if(d1 > d2)
				{
					position = (*itrLoop)[0].first.midPoint((*itrLoop)[2].first);
				}
				else
				{   
					position = (*itrLoop)[1].first.midPoint((*itrLoop)[3].first);
				}
					
				//Ogre::Vector3 position = getCentroid((*itrLoop));

				//calc orientation
				Ogre::Quaternion orientation = Ogre::Vector3::UNIT_Z.getRotationTo(loopPlane.normal);

				HoleSnapPoint snapto;
				snapto.first = position;
				snapto.second = orientation;
				mDesignTools->mHoles.push_back(snapto);
			}
		}

			
	}
	
}

//----------------------------------------------------------------------------------------
void getVertexData(const Ogre::VertexData* vertex_data,Ogre::Vector3* &vertices)
{
	//utility function for retrieving vertex info from an EdgeGroup
	//(doesn't retrieve indices, assumes a single submesh)
	vertices = new Ogre::Vector3[vertex_data->vertexCount];
	const Ogre::VertexElement* posElem = vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
	Ogre::HardwareVertexBufferSharedPtr vbuf = vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());
	unsigned char* vertex = static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

	float* pReal;
	for( size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
    {
		posElem->baseVertexPointerToElement(vertex, &pReal);
		Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);
		vertices[j] = pt;
	}
 
	vbuf->unlock();
}
//----------------------------------------------------------------------------------------
Ogre::Vector3 getCentroid(std::vector<std::pair<Ogre::Vector3,Ogre::Vector3> > polygon)
{
	/*for some reason this doen't work for all cases...*/

	//Triangulate the poly 
	std::vector<std::pair<Ogre::Vector3,Ogre::Vector3> >::iterator i;
	std::vector<Ogre::Real> areas;
	std::vector<Ogre::Vector3> centres;
	
	bool first = true;
	Ogre::Vector3 Vx0;
	Ogre::Vector3 v0 = (*polygon.begin()).first;
	for(i = (++polygon.begin());i!=polygon.end()&&i!=(--polygon.end());++i)
	{
		//Calculate the Area of each subtriangle
		Ogre::Vector3 v1 = (*i).first;
		Ogre::Vector3 v2 = (*(i+1)).first;
		Ogre::Vector3 Vxn = (v1-v0).crossProduct((v2-v0));//use this for test later
		Ogre::Real A = Vxn.length()/2;
		if(!first)
		{
			Ogre::Real result = Vxn.dotProduct(Vx0);
			if((result<0)==(A<0))
			{
				//convex
				areas.push_back(std::abs(A));
			}
			else
			{
				//concave
				areas.push_back(-(std::abs(A)));
				
			}

		}
		else
		{
			Vx0 = Vxn;
			first = false;
			areas.push_back(std::abs(A));
		}
		
		
		//calculate centroid of each triangle
		Ogre::Vector3 C = (v0+v1+v2)/3;
		centres.push_back(C);

	}

	//get the area of the polygon
	Ogre::Real TotalArea = std::accumulate(areas.begin(),areas.end(),0);

	//then sum of centroid x area for each tri
	Ogre::Vector3 TotalCentroid = Ogre::Vector3::ZERO;
	for(unsigned int i = 0;i<areas.size();++i)
	{
		TotalCentroid+=(centres[i]*areas[i]);
	}

	//calc the centroid of the polygon
	return (1/TotalArea)*TotalCentroid;
}



}//MZP namespace