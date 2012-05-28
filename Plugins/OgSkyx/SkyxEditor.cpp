///////////////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////////////

#include "Ogitors.h"
#include "SkyxEditor.h"

using namespace Ogitors;
using namespace Ogre;

//---------------------------------------------------------------------------------
CSkyxEditor::CSkyxEditor(CBaseEditorFactory *factory) : CBaseEditor(factory)
{
    mHandle = 0;
    mName->init("SkyX");

    mUsesGizmos = false;
    mUsesHelper = false;
}
//---------------------------------------------------------------------------------
CSkyxEditor::~CSkyxEditor()
{
}
//---------------------------------------------------------------------------------
bool CSkyxEditor::update(float timePassed)
{
    mHandle->update(timePassed);
    return false;
}
//---------------------------------------------------------------------------------
Ogre::SceneManager *CSkyxEditor::getSceneManager()
{
    return mHandle->getSceneManager();
}
//---------------------------------------------------------------------------------
void CSkyxEditor::refresh()
{
    if(!mHandle) return;
}
//---------------------------------------------------------------------------------
void CSkyxEditor::_restoreState()
{
    mHandle->setTimeMultiplier(mTimeMultiplier->get());
    
    SkyX::AtmosphereManager::Options opt = mHandle->getAtmosphereManager()->getOptions();
    
    opt.RayleighMultiplier = mRayleighMultiplier->get();
    opt.MieMultiplier = mMieMultiplier->get();
    opt.NumberOfSamples = mSampleCount->get();
    opt.HeightPosition = mHeight->get();
    opt.Exposure = mExposure->get();
    opt.InnerRadius = mInnerRadius->get();
    opt.OuterRadius = mOuterRadius->get();
    opt.Time = mTime->get();
    opt.EastPosition = mEastPosition->get();
    opt.SunIntensity = mSunIntensity->get();
    opt.WaveLength = mWaveLength->get();
    opt.G = mG->get();

    mHandle->getAtmosphereManager()->setOptions(opt);

    mHandle->getVCloudsManager()->getVClouds()->setNoiseScale(mNoiseScale->get());
    mHandle->getVCloudsManager()->getVClouds()->setWindSpeed(mWindSpeed->get());
    mHandle->getVCloudsManager()->getVClouds()->setWindDirection(Ogre::Degree(mWindDirection->get()));
}
//---------------------------------------------------------------------------------
void CSkyxEditor::onSave(bool forced)
{
}
//---------------------------------------------------------------------------------
bool CSkyxEditor::load(bool async)
{
    if(mLoaded->get())
        return true;

    Ogre::ResourceGroupManager *mngr = Ogre::ResourceGroupManager::getSingletonPtr();
    Ogre::String value = OgitorsUtils::QualifyPath(mOgitorsRoot->GetProjectOptions()->ProjectDir + "/SkyX");
    mngr->addResourceLocation(value,"FileSystem","SkyX");
    mngr->initialiseResourceGroup("SkyX");

	// Create SkyX
    mHandle = new SkyX::SkyX(mOgitorsRoot->GetSceneManager(), mOgitorsRoot->GetViewport()->getCameraEditor()->getCamera());
	mHandle->create();

    mHandle->getVCloudsManager()->create();

    _restoreState();

    registerForUpdates();

    mLoaded->set(true);
    return true;
}
//-----------------------------------------------------------------------------------------
bool CSkyxEditor::unLoad()
{
    if(!mLoaded->get())
        return true;

    unRegisterForUpdates();

    if(mHandle)
    {
        if(mHandle->isCreated()) mHandle->remove();
        delete mHandle;
    }

    Ogre::ResourceGroupManager::getSingletonPtr()->destroyResourceGroup("SkyX");

    mLoaded->set(false);
    return true;
}
//-----------------------------------------------------------------------------------------
void CSkyxEditor::createProperties(OgitorsPropertyValueMap &params)
{
    PROPERTY_PTR(mTimeMultiplier    , "timemultiplier"             , Ogre::Real, 0.1f, 0, SETTER(Ogre::Real, CSkyxEditor, _setTimeMultiplier));
    PROPERTY_PTR(mSampleCount       , "options::samplecount"       , int       , 4, 0, SETTER(int, CSkyxEditor, _setOptionsSampleCount));
    PROPERTY_PTR(mHeight            , "options::height"            , Ogre::Real, 0.01f, 0, SETTER(Ogre::Real, CSkyxEditor, _setOptionsHeight));
    PROPERTY_PTR(mTime              , "options::time"              , Ogre::Vector3, Ogre::Vector3(14.0f, 7.50f, 20.50f), 0, SETTER(Ogre::Vector3, CSkyxEditor, _setOptionsTime));
    PROPERTY_PTR(mEastPosition      , "options::eastposition"      , Ogre::Vector2, Ogre::Vector2(0, 1), 0, SETTER(Ogre::Vector2, CSkyxEditor, _setOptionsEastPosition));
    PROPERTY_PTR(mWaveLength        , "options::wavelength"        , Ogre::Vector3, Ogre::Vector3(0.57f, 0.54f, 0.44f), 0, SETTER(Ogre::Vector3, CSkyxEditor, _setOptionsWaveLength));
    PROPERTY_PTR(mExposure          , "options::exposure"          , Ogre::Real, 2.0f, 0, SETTER(Ogre::Real, CSkyxEditor, _setOptionsExposure));
    PROPERTY_PTR(mG                 , "options::g"                 , Ogre::Real, -0.991f, 0, SETTER(Ogre::Real, CSkyxEditor, _setOptionsG));
    PROPERTY_PTR(mRayleighMultiplier, "options::rayleighmultiplier", Ogre::Real, 0.0022f, 0, SETTER(Ogre::Real, CSkyxEditor, _setOptionsRayleighMultiplier));
    PROPERTY_PTR(mMieMultiplier     , "options::miemultiplier"     , Ogre::Real, 0.000675f, 0, SETTER(Ogre::Real, CSkyxEditor, _setOptionsMieMultiplier));
    PROPERTY_PTR(mInnerRadius       , "options::innerradius"       , Ogre::Real, 9.77501f, 0, SETTER(Ogre::Real, CSkyxEditor, _setOptionsInnerRadius));
    PROPERTY_PTR(mOuterRadius       , "options::outerradius"       , Ogre::Real, 10.2963f, 0, SETTER(Ogre::Real, CSkyxEditor, _setOptionsOuterRadius));
    PROPERTY_PTR(mSunIntensity      , "options::sunintensity"      , Ogre::Real, 30, 0, SETTER(Ogre::Real, CSkyxEditor, _setOptionsSunIntensity));
    PROPERTY_PTR(mNoiseScale        , "vclouds::noisescale"         , Ogre::Real, 4.2f, 0, SETTER(Ogre::Real, CSkyxEditor, _setNoiseScale));
    PROPERTY_PTR(mWindSpeed         , "vclouds::windspeed"          , Ogre::Real, 80, 0, SETTER(Ogre::Real, CSkyxEditor, _setWindSpeed));
    PROPERTY_PTR(mWindDirection     , "vclouds::winddirection"      ,Ogre::Real, 0, 0, SETTER(Ogre::Real, CSkyxEditor, _setWindDirection));

    mProperties.initValueMap(params);
}
//-----------------------------------------------------------------------------------------
bool CSkyxEditor::_setTimeMultiplier(OgitorsPropertyBase* property, const Ogre::Real& value)
{
    mHandle->setTimeMultiplier(value);
    return true;
}
//-----------------------------------------------------------------------------------------
bool CSkyxEditor::_setOptionsRayleighMultiplier(OgitorsPropertyBase* property, const Ogre::Real& value)
{
    SkyX::AtmosphereManager::Options opt = mHandle->getAtmosphereManager()->getOptions();
    opt.RayleighMultiplier = value;
    mHandle->getAtmosphereManager()->setOptions(opt);
    return true;
}
//-----------------------------------------------------------------------------------------
bool CSkyxEditor::_setOptionsMieMultiplier(OgitorsPropertyBase* property, const Ogre::Real& value)
{
    SkyX::AtmosphereManager::Options opt = mHandle->getAtmosphereManager()->getOptions();
    opt.MieMultiplier = value;
    mHandle->getAtmosphereManager()->setOptions(opt);
    return true;
}
//-----------------------------------------------------------------------------------------
bool CSkyxEditor::_setOptionsSampleCount(OgitorsPropertyBase* property, const int& value)
{
    if(value < 1)
        return false;
    
    SkyX::AtmosphereManager::Options opt = mHandle->getAtmosphereManager()->getOptions();
    opt.NumberOfSamples = value;
    mHandle->getAtmosphereManager()->setOptions(opt);
    return true;
}
//-----------------------------------------------------------------------------------------
bool CSkyxEditor::_setOptionsHeight(OgitorsPropertyBase* property, const Ogre::Real& value)
{
    SkyX::AtmosphereManager::Options opt = mHandle->getAtmosphereManager()->getOptions();
    opt.HeightPosition = value;
    mHandle->getAtmosphereManager()->setOptions(opt);
    return true;
}
//-----------------------------------------------------------------------------------------
bool CSkyxEditor::_setOptionsExposure(OgitorsPropertyBase* property, const Ogre::Real& value)
{
    SkyX::AtmosphereManager::Options opt = mHandle->getAtmosphereManager()->getOptions();
    opt.Exposure = value;
    mHandle->getAtmosphereManager()->setOptions(opt);
    return true;
}
//-----------------------------------------------------------------------------------------
bool CSkyxEditor::_setOptionsInnerRadius(OgitorsPropertyBase* property, const Ogre::Real& value)
{
    if(value >= mOuterRadius->get())
        return false;

    SkyX::AtmosphereManager::Options opt = mHandle->getAtmosphereManager()->getOptions();
    opt.InnerRadius = value;
    mHandle->getAtmosphereManager()->setOptions(opt);
    return true;
}
//-----------------------------------------------------------------------------------------
bool CSkyxEditor::_setOptionsOuterRadius(OgitorsPropertyBase* property, const Ogre::Real& value)
{
    if(value <= mInnerRadius->get())
        return false;

    SkyX::AtmosphereManager::Options opt = mHandle->getAtmosphereManager()->getOptions();
    opt.OuterRadius = value;
    mHandle->getAtmosphereManager()->setOptions(opt);
    return true;
}
//-----------------------------------------------------------------------------------------
bool CSkyxEditor::_setOptionsTime(OgitorsPropertyBase* property, const Ogre::Vector3& value)
{
    Ogre::Vector3 checkvalue = value;
    if(checkvalue.x < 0.0f)
        checkvalue.x = 0.0f;
    else if(checkvalue.x > 24.0f)
        checkvalue.x = 24.0f;

    if(checkvalue.y < 0.0f)
        checkvalue.y = 0.0f;
    else if(checkvalue.y > 24.0f)
        checkvalue.y = 24.0f;

    if(checkvalue.z < 0.0f)
        checkvalue.z = 0.0f;
    else if(checkvalue.z > 24.0f)
        checkvalue.z = 24.0f;

    mTime->init(checkvalue);

    SkyX::AtmosphereManager::Options opt = mHandle->getAtmosphereManager()->getOptions();
    opt.Time = checkvalue;
    mHandle->getAtmosphereManager()->setOptions(opt);
    return true;
}
//-----------------------------------------------------------------------------------------
bool CSkyxEditor::_setOptionsEastPosition(OgitorsPropertyBase* property, const Ogre::Vector2& value)
{
    SkyX::AtmosphereManager::Options opt = mHandle->getAtmosphereManager()->getOptions();
    opt.EastPosition = value;
    mHandle->getAtmosphereManager()->setOptions(opt);
    return true;
}
//-----------------------------------------------------------------------------------------
bool CSkyxEditor::_setOptionsSunIntensity(OgitorsPropertyBase* property, const Ogre::Real& value)
{
    SkyX::AtmosphereManager::Options opt = mHandle->getAtmosphereManager()->getOptions();
    opt.SunIntensity = value;
    mHandle->getAtmosphereManager()->setOptions(opt);
    return true;
}
//-----------------------------------------------------------------------------------------
bool CSkyxEditor::_setOptionsWaveLength(OgitorsPropertyBase* property, const Ogre::Vector3& value)
{
    SkyX::AtmosphereManager::Options opt = mHandle->getAtmosphereManager()->getOptions();
    opt.WaveLength = value;
    mHandle->getAtmosphereManager()->setOptions(opt);
    return true;
}
//-----------------------------------------------------------------------------------------
bool CSkyxEditor::_setOptionsG(OgitorsPropertyBase* property, const Ogre::Real& value)
{
    SkyX::AtmosphereManager::Options opt = mHandle->getAtmosphereManager()->getOptions();
    opt.G = value;
    mHandle->getAtmosphereManager()->setOptions(opt);
    return true;
}
//-----------------------------------------------------------------------------------------
bool CSkyxEditor::_setNoiseScale(OgitorsPropertyBase* property, const Ogre::Real& value)
{
    mHandle->getVCloudsManager()->getVClouds()->setNoiseScale(value);
    return true;
}
//-----------------------------------------------------------------------------------------
bool CSkyxEditor::_setWindSpeed(OgitorsPropertyBase* property, const Ogre::Real& value)
{
    mHandle->getVCloudsManager()->getVClouds()->setWindSpeed(value);
    return true;
}
//-----------------------------------------------------------------------------------------
bool CSkyxEditor::_setWindDirection(OgitorsPropertyBase* property, const Ogre::Real& value)
{
    mHandle->getVCloudsManager()->getVClouds()->setWindDirection(Ogre::Degree(value));
    return true;
}
//-----------------------------------------------------------------------------------------

//----------------------------------------------------------------------------
//----HYDRAXEDITORFACTORY-----------------------------------------------------
//----------------------------------------------------------------------------
CSkyxEditorFactory::CSkyxEditorFactory(OgitorsView *view) : CBaseEditorFactory(view)
{
    mTypeName = "Skyx Object";
    mEditorType = ETYPE_SKY_MANAGER;
    mAddToObjectList = true;
    mIcon = "Icons/caelum.svg";
    mCapabilities = CAN_DELETE;

    AddPropertyDefinition("timemultiplier","Time Mult.","",PROP_REAL);
    AddPropertyDefinition("options::rayleighmultiplier","Options::Rayleigh Mult.","",PROP_REAL);
    AddPropertyDefinition("options::miemultiplier","Options::Mie Mult.","",PROP_REAL);
    AddPropertyDefinition("options::samplecount","Options::Sample Count","",PROP_INT);
    AddPropertyDefinition("options::height","Options::Height","",PROP_REAL);
    AddPropertyDefinition("options::exposure","Options::Exposure","",PROP_REAL);
    AddPropertyDefinition("options::innerradius","Options::Inner Radius","",PROP_REAL);
    AddPropertyDefinition("options::outerradius","Options::Outer Radius","",PROP_REAL);

    OgitorsPropertyDef * definition = AddPropertyDefinition("options::time","Options::Time","",PROP_VECTOR3);
    definition->setFieldNames("Current", "Sun Rise", "Sun Set");
    AddPropertyDefinition("options::eastposition","Options::East Position","",PROP_VECTOR2);
    AddPropertyDefinition("options::sunintensity","Options::Sun Intensity","",PROP_REAL);
    definition = AddPropertyDefinition("options::wavelength","Options::Wave Length","",PROP_VECTOR3);
    definition->setFieldNames("R Mult.", "G Mult.", "B Mult.");
    AddPropertyDefinition("options::g","Options::G","",PROP_REAL);


    AddPropertyDefinition("vclouds::noisescale","VClouds::Noise Scale","",PROP_REAL);
    AddPropertyDefinition("vclouds::windspeed","VClouds::Wind Speed","",PROP_REAL);
    AddPropertyDefinition("vclouds::winddirection","VClouds::Wind Direction","",PROP_REAL);


    OgitorsPropertyDefMap::iterator it = mPropertyDefs.find("name");
    it->second.setAccess(true, false);

    it = mPropertyDefs.find("layer");
    it->second.setAccess(false, false);
}
//----------------------------------------------------------------------------
CBaseEditorFactory *CSkyxEditorFactory::duplicate(OgitorsView *view)
{
    CBaseEditorFactory *ret = OGRE_NEW CSkyxEditorFactory(view);
    ret->mTypeID = mTypeID;

    return ret;
}
//-----------------------------------------------------------------------------------------
CBaseEditor *CSkyxEditorFactory::CreateObject(CBaseEditor **parent, OgitorsPropertyValueMap &params)
{
  OgitorsRoot *ogroot = OgitorsRoot::getSingletonPtr();
  Ogre::ResourceGroupManager *mngr = Ogre::ResourceGroupManager::getSingletonPtr();
  Ogre::String value = ogroot->GetProjectOptions()->ProjectDir + "/SkyX/";

  CSkyxEditor *object = OGRE_NEW CSkyxEditor(this);

  if(params.find("init") != params.end())
  {
    Ogre::String dirname = OgitorsUtils::QualifyPath(value);
    OgitorsSystem::getSingletonPtr()->MakeDirectory(dirname);
    Ogre::String copydir = OgitorsUtils::GetEditorResourcesPath() + "/SKYX/*";
    OgitorsSystem::getSingletonPtr()->CopyFilesEx(copydir,dirname);

    params.erase(params.find("init"));
  }

  object->createProperties(params);
  object->mParentEditor->init(*parent);
  object->load();
  object->update(0);

  mInstanceCount++;
  return object;
}
//----------------------------------------------------------------------------
void CSkyxEditorFactory::DestroyObject(CBaseEditor *object)
{
    CSkyxEditor *SKYXOBJECT = static_cast<CSkyxEditor*>(object);

    SKYXOBJECT->unLoad();
    SKYXOBJECT->destroyAllChildren();
    if(SKYXOBJECT->getName() != "")
        OgitorsRoot::getSingletonPtr()->UnRegisterObjectName(SKYXOBJECT->getName(), SKYXOBJECT);

    OGRE_DELETE SKYXOBJECT;
    mInstanceCount--;
}

//----------------------------------------------------------------------------
bool dllStartPlugin(void *identifier, Ogre::String& name)
{
    name = "Skyx Plugin";
    OgitorsRoot::getSingletonPtr()->RegisterEditorFactory(identifier, OGRE_NEW CSkyxEditorFactory());
    return true;
}
//----------------------------------------------------------------------------
bool dllGetPluginName(Ogre::String& name)
{
    name = "Skyx Plugin";
    return true;
}
//----------------------------------------------------------------------------
bool dllStopPlugin(void)
{
    return true;
}
//----------------------------------------------------------------------------