/*/////////////////////////////////////////////////////////////////////////////////
/// An
///    ___   ____ ___ _____ ___  ____
///   / _ \ / ___|_ _|_   _/ _ \|  _ \
///  | | | | |  _ | |  | || | | | |_) |
///  | |_| | |_| || |  | || |_| |  _ <
///   \___/ \____|___| |_| \___/|_| \_\
///                              File
///
/// Copyright (c) 2008-2010 Ismail TARIM <ismail@royalspor.com> and the Ogitor Team
//
/// The MIT License
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
////////////////////////////////////////////////////////////////////////////////*/

#include "OgitorsPrerequisites.h"
#include "tinyxml.h"
#include "BaseEditor.h"
#include "BaseSerializer.h"
#include "OgitorsRoot.h"
#include "OgitorsSystem.h"
#include "OgitorsSceneSerializer.h"

using namespace Ogitors;

int COgitorsSceneSerializer::Import(Ogre::String importfile)
{
    OgitorsRoot *ogRoot = OgitorsRoot::getSingletonPtr();
    OgitorsSystem *mSystem = OgitorsSystem::getSingletonPtr();

    if(importfile == "")
    {
        UTFStringVector extlist;
        extlist.push_back(OTR("Ogitor Scene File"));
        extlist.push_back("*.ogscene");
        importfile = mSystem->DisplayOpenDialog(OTR("Open"),extlist);
        if(importfile == "") 
            return SCF_CANCEL;
    }

    Ogre::String filePath = OgitorsUtils::ExtractFilePath(importfile);
    Ogre::String fileName = OgitorsUtils::ExtractFileName(importfile);

    PROJECTOPTIONS *pOpt = ogRoot->GetProjectOptions();
    pOpt->CreatedIn = "";

    if(filePath.find(".") == 0)
    {
        filePath = OgitorsUtils::GetExePath() + filePath;
        filePath = OgitorsUtils::QualifyPath(filePath);
    }

    pOpt->ProjectDir = filePath;
    int typepos = fileName.find_last_of(".");
    pOpt->ProjectName = fileName;
    if(typepos != -1)
        pOpt->ProjectName.erase(typepos,pOpt->ProjectName.length() - typepos);

    bool testpassed = false;
    try
    {
        std::ofstream test((filePath + "test.dat").c_str());
        if(test.is_open())
            testpassed = true;
        test.close();
        mSystem->DeleteFile(filePath + "test.dat");
    }
    catch(...)
    {
        testpassed = false;
    }

    if(!testpassed)
    {
        mSystem->DisplayMessageDialog("The path is Read-Only. Ogitor can not work with Read-Only Project Paths!", DLGTYPE_OK);
        return SCF_CANCEL;
    }

    TiXmlDocument docImport((filePath + fileName).c_str());

    Ogre::UTFString loadmsg = mSystem->Translate("Parsing Scene File");
    mSystem->UpdateLoadProgress(1, loadmsg);

    if(!docImport.LoadFile()) 
        return SCF_ERRFILE;

    TiXmlNode* node = 0;
    TiXmlElement* element = 0;
    node = docImport.FirstChild("OGITORSCENE");
    
    if(!node)
        return SCF_ERRPARSE;

    element = node->ToElement();
    int version = Ogre::StringConverter::parseInt(ValidAttr(element->Attribute("version"),"1"));
    if(version == 1)
        return ImportV1(element);

    node = node->FirstChild("PROJECT");

    if(node)
    {
        loadmsg = mSystem->Translate("Parsing project options");
        mSystem->UpdateLoadProgress(5, loadmsg);
        ogRoot->LoadProjectOptions(node->ToElement());
        ogRoot->PrepareProjectResources();
    }

    node = docImport.FirstChild("OGITORSCENE");
    if(!node) 
        return SCF_ERRPARSE;
    element = node->FirstChildElement();

    loadmsg = mSystem->Translate("Creating scene objects");
    mSystem->UpdateLoadProgress(10, loadmsg);

    unsigned int obj_count = 0;
    Ogre::String objecttype;
    OgitorsPropertyValueMap params;
    OgitorsPropertyValue tmpPropVal;
    do
    {
        // Make sure its NON-ZERO
        if(pOpt->ObjectCount)
        {
            ++obj_count;
            mSystem->UpdateLoadProgress(10 + ((obj_count * 70) / pOpt->ObjectCount), loadmsg);
        }
        
        params.clear();

        Ogre::String objAttValue;

        objAttValue = ValidAttr(element->Attribute("object_id"), "");
        if(objAttValue != "")
        {
            tmpPropVal.propType = PROP_UNSIGNED_INT;
            tmpPropVal.val = Ogre::Any(Ogre::StringConverter::parseUnsignedInt(objAttValue));
            params.insert(OgitorsPropertyValueMap::value_type("object_id", tmpPropVal));
        }

        objAttValue = ValidAttr(element->Attribute("parentnode"),"");
        if(objAttValue != "")
        {
            tmpPropVal.propType = PROP_STRING;
            tmpPropVal.val = Ogre::Any(objAttValue);
            params.insert(OgitorsPropertyValueMap::value_type("parentnode", tmpPropVal));
        }

        objAttValue = ValidAttr(element->Attribute("name"),"");
        if(objAttValue != "")
        {
            tmpPropVal.propType = PROP_STRING;
            tmpPropVal.val = Ogre::Any(objAttValue);
            params.insert(OgitorsPropertyValueMap::value_type("name", tmpPropVal));
        }
        else
            continue;

        objAttValue = ValidAttr(element->Attribute("typename"),"");
        if(objAttValue != "")
        {
            tmpPropVal.propType = PROP_STRING;
            tmpPropVal.val = Ogre::Any(objAttValue);
            params.insert(OgitorsPropertyValueMap::value_type("typename", tmpPropVal));
        }
        else
            continue;

        TiXmlElement *properties = element->FirstChildElement();
        if(properties)
        {
            Ogre::String elementName;
            do
            {
                elementName = properties->Value();
                if(elementName != "PROPERTY")
                    continue;

                Ogre::String attID = ValidAttr(properties->Attribute("id"),"");
                int attType = Ogre::StringConverter::parseInt(ValidAttr(properties->Attribute("type"),""));
                Ogre::String attValue = ValidAttr(properties->Attribute("value"),"");

                params.insert(OgitorsPropertyValueMap::value_type(attID, OgitorsPropertyValue::createFromString((OgitorsPropertyType)attType, attValue)));
            } while(properties = properties->NextSiblingElement());
        }

        objecttype = Ogre::any_cast<Ogre::String>(params["typename"].val);
        CBaseEditor *result = ogRoot->CreateEditorObject(0, objecttype, params, false, false);
        if(result)
        {
            TiXmlElement *customprop = element->FirstChildElement("CUSTOMPROPERTIES");
            if(customprop) 
            {
                OgitorsUtils::ReadCustomPropertySet(customprop, result->getCustomProperties());
            }
        }
    } while(element = element->NextSiblingElement());

    ogRoot->AfterLoadScene();

    return SCF_OK;
}
//-----------------------------------------------------------------------------
int COgitorsSceneSerializer::Export(bool SaveAs)
{
    OgitorsRoot *ogRoot = OgitorsRoot::getSingletonPtr();
    OgitorsSystem *mSystem = OgitorsSystem::getSingletonPtr();

    PROJECTOPTIONS *pOpt = ogRoot->GetProjectOptions();
    Ogre::String fileName = pOpt->ProjectDir + pOpt->ProjectName + ".ogscene";

    bool forceSave = false;
    // If SaveAs is TRUE, use the OgitorsSystem Functions to retrieve
    // a FileName and also copy the contents of current scene to the new location
    if(SaveAs)
    {
        UTFStringVector extlist;
        extlist.push_back(OTR("Ogitor Scene File"));
        extlist.push_back("*.ogscene");
        fileName = mSystem->DisplaySaveDialog(OTR("Save As"),extlist);
        if(fileName == "") 
            return SCF_CANCEL;

        Ogre::String oldProjDir = pOpt->ProjectDir;
        Ogre::String oldProjName = pOpt->ProjectName;

        pOpt->ProjectName = OgitorsUtils::ExtractFileName(fileName);
        int typepos = pOpt->ProjectName.find_last_of(".");
        if(typepos != -1)
            pOpt->ProjectName.erase(typepos,pOpt->ProjectName.length() - typepos);

        pOpt->ProjectDir = OgitorsUtils::ExtractFilePath(fileName);

        ogRoot->AdjustUserResourceDirectories(oldProjDir);

        Ogre::String newDir = pOpt->ProjectDir;

        mSystem->MakeDirectory(newDir);
        mSystem->CopyFilesEx(oldProjDir + "*", newDir);

        Ogre::String delfilestr = OgitorsUtils::QualifyPath(pOpt->ProjectDir + "/" + oldProjName + ".ogscene");
        mSystem->DeleteFile(delfilestr);

        forceSave = true;
    }

    if(!SaveAs)
        fileName += ".tmp";

    // Open a stream to output our XML Content and write the general header
    std::ofstream outfile(fileName.c_str());
    outfile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    outfile << "<OGITORSCENE version=\"2\">\n";
    
    //TODO: write this out to a separate file fileName + "_priv" maybe?
    ogRoot->WriteProjectOptions(outfile, false);

    ObjectVector ObjectList;
    OgitorsPropertyValueMap theList;
    OgitorsPropertyValueMap::iterator ni;

    // Start from 1, since 0 means all objects
    for(unsigned int i = 1;i < LAST_EDITOR;i++)
    {
        ogRoot->GetObjectList(i, ObjectList);
        for(unsigned int ob = 0;ob < ObjectList.size();ob++)
        {
            /// If Object does not have a parent, then it is not part of the scene
            if(ObjectList[ob]->getParent())
            {
                ObjectList[ob]->onSave(forceSave);
                if(ObjectList[ob]->isSerializable())
                {
                    outfile << OgitorsUtils::GetObjectSaveStringV2(ObjectList[ob], 2, true, true).c_str();
                    outfile << "\n";
                }
            }
        }
    }
    outfile << "</OGITORSCENE>\n";
    outfile.close();

    ogRoot->SetSceneModified(false);

    if(SaveAs)
    {
        ogRoot->TerminateScene();
        ogRoot->LoadScene(fileName);
    }
    else
    {
        Ogre::String delfile = fileName.substr(0, fileName.length() - 4);
        OgitorsSystem::getSingletonPtr()->DeleteFile(delfile);
        OgitorsSystem::getSingletonPtr()->RenameFile(fileName, delfile);
    }
    return SCF_OK;
}
//-----------------------------------------------------------------------------
int COgitorsSceneSerializer::ImportV1(TiXmlElement *baseelement)
{
    return SCF_ERRVERSION;
 //   Ogre::UTFString loadmsg;

    //OgitorsRoot *ogRoot = OgitorsRoot::getSingletonPtr();
 //   OgitorsSystem *mSystem = OgitorsSystem::getSingletonPtr();

 //   TiXmlElement* element = baseelement;
    //TiXmlElement* project = element->FirstChildElement("PROJECT");

    //if(project)
    //{
 //       loadmsg = mSystem->Translate("Parsing project options");
 //       mSystem->UpdateLoadProgress(15, loadmsg);
    //    ogRoot->LoadProjectOptions(project);
 //       ogRoot->PrepareProjectResources();
    //}

    //ogRoot->LoadCustomProperties();

 //   element = element->FirstChildElement();

 //   loadmsg = mSystem->Translate("Creating scene objects");
 //   mSystem->UpdateLoadProgress(30, loadmsg);

 //   Ogre::String objecttype;
 //   Ogre::NameValuePairList params;
 //   do
 //   {
    //    objecttype = ValidAttr(element->Attribute("type"));
    //    if(objecttype == "") 
 //           continue;
    //    TiXmlAttribute *att = element->FirstAttribute();
    //    if(!att) 
 //           continue;
    //    params.clear();
    //    do
    //    {
    //        Ogre::String attName = att->Name();
    //        Ogre::String attValue = att->Value();
    //        params.insert(Ogre::NameValuePairList::value_type(attName, attValue));
    //    } while(att = att->Next());

 //       CBaseEditor *result = ogRoot->CreateEditorObject(0, objecttype, params, false, false);
    //    if(result)
    //    {
    //        TiXmlElement *customprop = element->FirstChildElement();
    //        if(customprop) 
    //        {
    //            do
    //            {
    //                Ogre::String eType = customprop->Value();
    //                if(eType != "CUSTOMPROPERTY") 
 //                       continue;
    //                result->ReadCustomProperty(customprop);
    //            } while(customprop = customprop->NextSiblingElement());
    //        }
    //    }
 //   } while(element = element->NextSiblingElement());

 //   ogRoot->AfterLoadScene();

 //   return SCF_OK;
}
//-----------------------------------------------------------------------------

