/**************************************************************************************

 Copyright (C) 2001 - 2009 Autodesk, Inc. and/or its licensors.
 All Rights Reserved.

 The coded instructions, statements, computer programs, and/or related material 
 (collectively the "Data") in these files contain unpublished information 
 proprietary to Autodesk, Inc. and/or its licensors, which is protected by 
 Canada and United States of America federal copyright law and by international 
 treaties. 
 
 The Data may not be disclosed or distributed to third parties, in whole or in
 part, without the prior written consent of Autodesk, Inc. ("Autodesk").

 THE DATA IS PROVIDED "AS IS" AND WITHOUT WARRANTY.
 ALL WARRANTIES ARE EXPRESSLY EXCLUDED AND DISCLAIMED. AUTODESK MAKES NO
 WARRANTY OF ANY KIND WITH RESPECT TO THE DATA, EXPRESS, IMPLIED OR ARISING
 BY CUSTOM OR TRADE USAGE, AND DISCLAIMS ANY IMPLIED WARRANTIES OF TITLE, 
 NON-INFRINGEMENT, MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE OR USE. 
 WITHOUT LIMITING THE FOREGOING, AUTODESK DOES NOT WARRANT THAT THE OPERATION
 OF THE DATA WILL BE UNINTERRUPTED OR ERROR FREE. 
 
 IN NO EVENT SHALL AUTODESK, ITS AFFILIATES, PARENT COMPANIES, LICENSORS
 OR SUPPLIERS ("AUTODESK GROUP") BE LIABLE FOR ANY LOSSES, DAMAGES OR EXPENSES
 OF ANY KIND (INCLUDING WITHOUT LIMITATION PUNITIVE OR MULTIPLE DAMAGES OR OTHER
 SPECIAL, DIRECT, INDIRECT, EXEMPLARY, INCIDENTAL, LOSS OF PROFITS, REVENUE
 OR DATA, COST OF COVER OR CONSEQUENTIAL LOSSES OR DAMAGES OF ANY KIND),
 HOWEVER CAUSED, AND REGARDLESS OF THE THEORY OF LIABILITY, WHETHER DERIVED
 FROM CONTRACT, TORT (INCLUDING, BUT NOT LIMITED TO, NEGLIGENCE), OR OTHERWISE,
 ARISING OUT OF OR RELATING TO THE DATA OR ITS USE OR ANY OTHER PERFORMANCE,
 WHETHER OR NOT AUTODESK HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH LOSS
 OR DAMAGE. 

**************************************************************************************/

/////////////////////////////////////////////////////////////////////////
//
// The scene created in this example is a cylinder linked to a skeleton
// made of 2 segments. Two takes of animation show the influence of the
// skeleton segments over the cylinder.
//
// The example illustrates how to:
//        1) create a patch
//        2) create a skeleton segment
//        3) create a link
//        4) store the bind pose
//        5) store one arbitrary rest pose
//        6) create multiple takes of animation
//        7) create meta-data and add a thumbnail
//        8) export a scene in a .FBX file (ASCII mode)
//
/////////////////////////////////////////////////////////////////////////

#include <fbxsdk.h>

#include "../Common/Common.h"
#include "Thumbnail.h"

#define SAMPLE_FILENAME "ExportScene01.fbx"


// Function prototypes.
bool CreateScene(KFbxSdkManager* pSdkManager, KFbxScene* pScene);

KFbxNode* CreatePatch(KFbxSdkManager* pSdkManager, char* pName);
KFbxNode* CreateSkeleton(KFbxSdkManager* pSdkManager, char* pName);

void LinkPatchToSkeleton(KFbxSdkManager* pSdkManager, KFbxNode* pPatch, KFbxNode* pSkeletonRoot);
void StoreBindPose(KFbxSdkManager* pSdkManager, KFbxScene* pScene, KFbxNode* pPatch, KFbxNode* pSkeletonRoot);
void StoreRestPose(KFbxSdkManager* pSdkManager, KFbxScene* pScene, KFbxNode* pSkeletonRoot);
void AnimateSkeleton(KFbxSdkManager* pSdkManager, KFbxScene* pScene, KFbxNode* pSkeletonRoot);
void AddThumbnailToScene(KFbxSdkManager* pSdkManager, KFbxScene* pScene);
void AddThumbnailToTake(KFbxSdkManager* pSdkManager, KFbxScene* pScene, KString& pTakeName, int pThumbnailIndex);
void AddNodeRecursively(KArrayTemplate<KFbxNode*>& pNodeArray, KFbxNode* pNode);

void SetXMatrix(KFbxXMatrix& pXMatrix, const KFbxMatrix& pMatrix);

int main(int argc, char** argv)
{
    KFbxSdkManager* lSdkManager = NULL;
    KFbxScene* lScene = NULL;
    bool lResult;

    // Prepare the FBX SDK.
    InitializeSdkObjects(lSdkManager, lScene);

    // Create the scene.
    lResult = CreateScene(lSdkManager, lScene);

    if(lResult == false)
    {
        printf("\n\nAn error occurred while creating the scene...\n");
        DestroySdkObjects(lSdkManager);
        return 0;
    }

    // Save the scene.

    // The example can take an output file name as an argument.
    if(argc > 1)
    {
        lResult = SaveScene(lSdkManager, lScene, argv[1]);
    }
    // A default output file name is given otherwise.
    else
    {
        lResult = SaveScene(lSdkManager, lScene, SAMPLE_FILENAME);
    }

    if(lResult == false)
    {
        printf("\n\nAn error occurred while saving the scene...\n");
        DestroySdkObjects(lSdkManager);
        return 0;
    }

    // Destroy all objects created by the FBX SDK.
    DestroySdkObjects(lSdkManager);

    return 0;
}

bool CreateScene(KFbxSdkManager* pSdkManager, KFbxScene* pScene)
{
    // create scene info
    KFbxDocumentInfo* sceneInfo = KFbxDocumentInfo::Create(pSdkManager,"SceneInfo");
    sceneInfo->mTitle = "Example scene";
    sceneInfo->mSubject = "Illustrates the creation and animation of a deformed cylinder.";
    sceneInfo->mAuthor = "ExportScene01.exe sample program.";
    sceneInfo->mRevision = "rev. 1.0";
    sceneInfo->mKeywords = "deformed cylinder";
    sceneInfo->mComment = "no particular comments required.";

    // we need to add the sceneInfo before calling AddThumbNailToScene because
    // that function is asking the scene for the sceneInfo.
    pScene->SetSceneInfo(sceneInfo);

    AddThumbnailToScene(pSdkManager, pScene);

    KFbxNode* lPatch = CreatePatch(pSdkManager, "Patch");
    KFbxNode* lSkeletonRoot = CreateSkeleton(pSdkManager, "Skeleton");

    LinkPatchToSkeleton(pSdkManager, lPatch, lSkeletonRoot);
    StoreBindPose(pSdkManager, pScene, lPatch, lSkeletonRoot);
    StoreRestPose(pSdkManager, pScene, lSkeletonRoot);
    AnimateSkeleton(pSdkManager, pScene, lSkeletonRoot);

    // Build the node tree.
    KFbxNode* lRootNode = pScene->GetRootNode();
    lRootNode->AddChild(lPatch);
    lRootNode->AddChild(lSkeletonRoot);

    // Identify current take when file is loaded.
    pScene->SetCurrentTake("Bend on 2 sides");

    return true;
}

// Create a cylinder centered on the Z axis. 
KFbxNode* CreatePatch(KFbxSdkManager* pSdkManager, char* pName)
{
    KFbxPatch* lPatch = KFbxPatch::Create(pSdkManager,pName);

    // Set patch properties.
    lPatch->InitControlPoints(4, KFbxPatch::eBSPLINE, 7, KFbxPatch::eBSPLINE);
    lPatch->SetStep(4, 4);
    lPatch->SetClosed(true, false);

    KFbxVector4* lVector4 = lPatch->GetControlPoints();
    int i;

    for (i = 0; i < 7; i++) 
    {
        double lRadius = 15.0;
        double lSegmentLength = 20.0;
        lVector4[4*i + 0].Set(lRadius, 0.0, (i-3)*lSegmentLength);
        lVector4[4*i + 1].Set(0.0, -lRadius, (i-3)*lSegmentLength);
        lVector4[4*i + 2].Set(-lRadius, 0.0, (i-3)*lSegmentLength);
        lVector4[4*i + 3].Set(0.0, lRadius, (i-3)*lSegmentLength);
    }

    KFbxNode* lNode = KFbxNode::Create(pSdkManager,pName);

    // Rotate the cylinder along the X axis so the axis
    // of the cylinder is the same as the bone axis (Y axis)
    KFbxVector4 lR(-90.0, 0.0, 0.0);
    lNode->SetDefaultR(lR);

    lNode->SetNodeAttribute(lPatch);

    return lNode;
}

// Create a skeleton with 2 segments.
KFbxNode* CreateSkeleton(KFbxSdkManager* pSdkManager, char* pName)
{
    // Create skeleton root. 
    KString lRootName(pName);
    lRootName += "Root";
    KFbxSkeleton* lSkeletonRootAttribute = KFbxSkeleton::Create(pSdkManager, pName);
    lSkeletonRootAttribute->SetSkeletonType(KFbxSkeleton::eROOT);
    KFbxNode* lSkeletonRoot = KFbxNode::Create(pSdkManager,lRootName.Buffer());
    lSkeletonRoot->SetNodeAttribute(lSkeletonRootAttribute);    
    lSkeletonRoot->SetDefaultT(KFbxVector4(0.0, -40.0, 0.0));

    // Create skeleton first limb node. 
    KString lLimbNodeName1(pName);
    lLimbNodeName1 += "LimbNode1";
    KFbxSkeleton* lSkeletonLimbNodeAttribute1 = KFbxSkeleton::Create(pSdkManager,lLimbNodeName1);
    lSkeletonLimbNodeAttribute1->SetSkeletonType(KFbxSkeleton::eLIMB_NODE);
    lSkeletonLimbNodeAttribute1->Size.Set(1.0);
    KFbxNode* lSkeletonLimbNode1 = KFbxNode::Create(pSdkManager,lLimbNodeName1.Buffer());
    lSkeletonLimbNode1->SetNodeAttribute(lSkeletonLimbNodeAttribute1);    
    lSkeletonLimbNode1->SetDefaultT(KFbxVector4(0.0, 40.0, 0.0));

    // Create skeleton second limb node. 
    KString lLimbNodeName2(pName);
    lLimbNodeName2 += "LimbNode2";
    KFbxSkeleton* lSkeletonLimbNodeAttribute2 = KFbxSkeleton::Create(pSdkManager,lLimbNodeName2);
    lSkeletonLimbNodeAttribute2->SetSkeletonType(KFbxSkeleton::eLIMB_NODE);
    lSkeletonLimbNodeAttribute2->Size.Set(1.0);
    KFbxNode* lSkeletonLimbNode2 = KFbxNode::Create(pSdkManager,lLimbNodeName2.Buffer());
    lSkeletonLimbNode2->SetNodeAttribute(lSkeletonLimbNodeAttribute2);    
    lSkeletonLimbNode2->SetDefaultT(KFbxVector4(0.0, 40.0, 0.0));

    // Build skeleton node hierarchy. 
    lSkeletonRoot->AddChild(lSkeletonLimbNode1);
    lSkeletonLimbNode1->AddChild(lSkeletonLimbNode2);

    return lSkeletonRoot;
}

// Set the influence of the skeleton segments over the cylinder.
// The link mode is KFbxLink::eTOTAL1 which means the total
// of the weights assigned to a given control point must equal 1.
void LinkPatchToSkeleton(KFbxSdkManager* pSdkManager, KFbxNode* pPatch, KFbxNode* pSkeletonRoot)
{
    int i, j;
    KFbxXMatrix lXMatrix;

    KFbxNode* lRoot = pSkeletonRoot;
    KFbxNode* lLimbNode1 = pSkeletonRoot->GetChild(0);
    KFbxNode* lLimbNode2 = lLimbNode1->GetChild(0);

    // Bottom section of cylinder is clustered to skeleton root.
    KFbxCluster *lClusterToRoot = KFbxCluster::Create(pSdkManager,"");
    lClusterToRoot->SetLink(lRoot);
    lClusterToRoot->SetLinkMode(KFbxCluster::eTOTAL1);
    for(i=0; i<4; ++i)
        for(j=0; j<4; ++j)
            lClusterToRoot->AddControlPointIndex(4*i + j, 1.0 - 0.25*i);

    // Center section of cylinder is clustered to skeleton limb node.
    KFbxCluster* lClusterToLimbNode1 = KFbxCluster::Create(pSdkManager, "");
    lClusterToLimbNode1->SetLink(lLimbNode1);
    lClusterToLimbNode1->SetLinkMode(KFbxCluster::eTOTAL1);

    for (i =1; i<6; ++i)
        for (j=0; j<4; ++j)
            lClusterToLimbNode1->AddControlPointIndex(4*i + j, (i == 1 || i == 5 ? 0.25 : 0.50));


    // Top section of cylinder is clustered to skeleton limb.

    KFbxCluster * lClusterToLimbNode2 = KFbxCluster::Create(pSdkManager,"");
    lClusterToLimbNode2->SetLink(lLimbNode2);
    lClusterToLimbNode2->SetLinkMode(KFbxCluster::eTOTAL1);

    for (i=3; i<7; ++i)
        for (j=0; j<4; ++j)
            lClusterToLimbNode2->AddControlPointIndex(4*i + j, 0.25*(i - 2));

    // Now we have the Patch and the skeleton correctly positioned,
    // set the Transform and TransformLink matrix accordingly.
    lXMatrix = pPatch->GetGlobalFromDefaultTake(KFbxNode::eSOURCE_SET);

    lClusterToRoot->SetTransformMatrix(lXMatrix);
    lClusterToLimbNode1->SetTransformMatrix(lXMatrix);
    lClusterToLimbNode2->SetTransformMatrix(lXMatrix);



    lXMatrix = lRoot->GetGlobalFromDefaultTake(KFbxNode::eSOURCE_SET);
    lClusterToRoot->SetTransformLinkMatrix(lXMatrix);


    lXMatrix = lLimbNode1->GetGlobalFromDefaultTake(KFbxNode::eSOURCE_SET);
    lClusterToLimbNode1->SetTransformLinkMatrix(lXMatrix);


    lXMatrix = lLimbNode2->GetGlobalFromDefaultTake(KFbxNode::eSOURCE_SET);
    lClusterToLimbNode2->SetTransformLinkMatrix(lXMatrix);


    // Add the clusters to the patch by creating a skin and adding those clusters to that skin.
    // After add that skin.

    KFbxGeometry* lPatchAttribute = (KFbxGeometry*) pPatch->GetNodeAttribute();
    KFbxSkin* lSkin = KFbxSkin::Create(pSdkManager, "");
    lSkin->AddCluster(lClusterToRoot);
    lSkin->AddCluster(lClusterToLimbNode1);
    lSkin->AddCluster(lClusterToLimbNode2);
    lPatchAttribute->AddDeformer(lSkin);

}

// Create two animation takes.
void AnimateSkeleton(KFbxSdkManager* pSdkManager, KFbxScene* pScene, KFbxNode* pSkeletonRoot)
{
    KString lTakeName;
    KFCurve* lCurve = NULL;
    KTime lTime;
    int lKeyIndex = 0;

    KFbxNode* lRoot = pSkeletonRoot;
    KFbxNode* lLimbNode1 = pSkeletonRoot->GetChild(0);

    // First take.
    lTakeName = "Bend on 2 sides";
    pScene->CreateTake(lTakeName.Buffer());

    lRoot->CreateTakeNode(lTakeName.Buffer());
    lRoot->SetCurrentTakeNode(lTakeName.Buffer());
    lRoot->LclRotation.GetKFCurveNode(true, lTakeName.Buffer());

    lCurve = lRoot->LclRotation.GetKFCurve(KFCURVENODE_R_Z, lTakeName.Buffer());
    if (lCurve)
    {
        lCurve->KeyModifyBegin();

        lTime.SetSecondDouble(0.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 0.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFCURVE_INTERPOLATION_CUBIC);

        lTime.SetSecondDouble(1.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 45.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFCURVE_INTERPOLATION_CUBIC);

        lTime.SetSecondDouble(2.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, -45.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFCURVE_INTERPOLATION_CUBIC);

        lTime.SetSecondDouble(3.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 0.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFCURVE_INTERPOLATION_CUBIC);
        lCurve->KeyModifyEnd();
    }


    lLimbNode1->CreateTakeNode(lTakeName.Buffer());
    lLimbNode1->SetCurrentTakeNode(lTakeName.Buffer());
    lLimbNode1->LclRotation.GetKFCurveNode(true, lTakeName.Buffer());

    lCurve = lLimbNode1->LclRotation.GetKFCurve(KFCURVENODE_R_Z, lTakeName.Buffer());
    if (lCurve)
    {
        lCurve->KeyModifyBegin();

        lTime.SetSecondDouble(0.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 0.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFCURVE_INTERPOLATION_CUBIC);

        lTime.SetSecondDouble(1.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, -90.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFCURVE_INTERPOLATION_CUBIC);

        lTime.SetSecondDouble(2.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 90.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFCURVE_INTERPOLATION_CUBIC);

        lTime.SetSecondDouble(3.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 0.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFCURVE_INTERPOLATION_CUBIC);

        lCurve->KeyModifyEnd();
    }

    AddThumbnailToTake(pSdkManager, pScene, lTakeName, 0);

    // Second take.
    lTakeName = "Bend and turn around";
    pScene->CreateTake(lTakeName.Buffer());

    lRoot->CreateTakeNode(lTakeName.Buffer());
    lRoot->SetCurrentTakeNode(lTakeName.Buffer());
    lRoot->LclRotation.GetKFCurveNode(true, lTakeName.Buffer());

    lCurve = lRoot->LclRotation.GetKFCurve(KFCURVENODE_R_Y, lTakeName.Buffer());
    if (lCurve)
    {
        lCurve->KeyModifyBegin();

        lTime.SetSecondDouble(0.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 0.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFCURVE_INTERPOLATION_CUBIC);

        lTime.SetSecondDouble(2.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 720.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFCURVE_INTERPOLATION_CUBIC);

        lCurve->KeyModifyEnd();
    }

    lLimbNode1->CreateTakeNode(lTakeName.Buffer());
    lLimbNode1->SetCurrentTakeNode(lTakeName.Buffer());
    lLimbNode1->LclRotation.GetKFCurveNode(true, lTakeName.Buffer());

    lCurve = lLimbNode1->LclRotation.GetKFCurve(KFCURVENODE_R_Z, lTakeName.Buffer());
    if (lCurve)
    {
        lCurve->KeyModifyBegin();

        lTime.SetSecondDouble(0.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 0.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFCURVE_INTERPOLATION_CUBIC);

        lTime.SetSecondDouble(1.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 90.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFCURVE_INTERPOLATION_CUBIC);

        lTime.SetSecondDouble(2.0);
        lKeyIndex = lCurve->KeyAdd(lTime);
        lCurve->KeySetValue(lKeyIndex, 0.0);
        lCurve->KeySetInterpolation(lKeyIndex, KFCURVE_INTERPOLATION_CUBIC);

        lCurve->KeyModifyEnd();
    }

    AddThumbnailToTake(pSdkManager, pScene, lTakeName, 1);
}

// Add a thumbnail to the scene
void AddThumbnailToScene(KFbxSdkManager* pSdkManager, KFbxScene* pScene)
{
    KFbxThumbnail* lThumbnail = KFbxThumbnail::Create(pSdkManager,"");

    lThumbnail->SetDataFormat(KFbxThumbnail::eRGB_24);
    lThumbnail->SetSize(KFbxThumbnail::e64x64);
    lThumbnail->SetThumbnailImage(cSceneThumbnail);

    if (pScene->GetSceneInfo())
    {
        pScene->GetSceneInfo()->SetSceneThumbnail(lThumbnail);
    }
}

// Add a thumbnail to the take
void AddThumbnailToTake(KFbxSdkManager* pSdkManager, KFbxScene* pScene, KString& pTakeName, int pThumbnailIndex)
{
    KFbxTakeInfo lTakeInfo;
    KFbxThumbnail* lThumbnail = KFbxThumbnail::Create(pSdkManager,"");

    lThumbnail->SetDataFormat(KFbxThumbnail::eRGB_24);
    lThumbnail->SetSize(KFbxThumbnail::e64x64);
    lThumbnail->SetThumbnailImage(pThumbnailIndex == 0 ? cTakeOneThumbnail : cTakeTwoThumbnail);

    lTakeInfo.mName = pTakeName;
    lTakeInfo.mDescription = KString("Take at index ") + pThumbnailIndex;
    lTakeInfo.SetTakeThumbnail(lThumbnail);

    pScene->SetTakeInfo(lTakeInfo);
}

// Store the Bind Pose
void StoreBindPose(KFbxSdkManager* pSdkManager, KFbxScene* pScene, KFbxNode* pPatch, KFbxNode* pSkeletonRoot)
{
    // In the bind pose, we must store all the link's global matrix at the time of the bind.
    // Plus, we must store all the parent(s) global matrix of a link, even if they are not
    // themselves deforming any model.

    // In this example, since there is only one model deformed, we don't need walk through 
    // the scene
    //

    // Now list the all the link involve in the patch deformation
    KArrayTemplate<KFbxNode*> lClusteredFbxNodes;
    int                       i, j;

    if (pPatch && pPatch->GetNodeAttribute())
    {
        int lSkinCount=0;
        int lClusterCount=0;
        switch (pPatch->GetNodeAttribute()->GetAttributeType())
        {
        case KFbxNodeAttribute::eMESH:
        case KFbxNodeAttribute::eNURB:
        case KFbxNodeAttribute::ePATCH:

            lSkinCount = ((KFbxGeometry*)pPatch->GetNodeAttribute())->GetDeformerCount(KFbxDeformer::eSKIN);
            //Go through all the skins and count them
            //then go through each skin and get their cluster count
            for(i=0; i<lSkinCount; ++i)
            {
                KFbxSkin *lSkin=(KFbxSkin*)((KFbxGeometry*)pPatch->GetNodeAttribute())->GetDeformer(i, KFbxDeformer::eSKIN);
                lClusterCount+=lSkin->GetClusterCount();
            }
            break;
        }
        //if we found some clusters we must add the node
        if (lClusterCount)
        {
            //Again, go through all the skins get each cluster link and add them
            for (i=0; i<lSkinCount; ++i)
            {
                KFbxSkin *lSkin=(KFbxSkin*)((KFbxGeometry*)pPatch->GetNodeAttribute())->GetDeformer(i, KFbxDeformer::eSKIN);
                lClusterCount=lSkin->GetClusterCount();
                for (j=0; j<lClusterCount; ++j)
                {
                    KFbxNode* lClusterNode = lSkin->GetCluster(j)->GetLink();
                    AddNodeRecursively(lClusteredFbxNodes, lClusterNode);
                }

            }

            // Add the patch to the pose
            lClusteredFbxNodes.Add(pPatch);
        }
    }

    // Now create a bind pose with the link list
    if (lClusteredFbxNodes.GetCount())
    {
        // A pose must be named. Arbitrarily use the name of the patch node.
        KFbxPose* lPose = KFbxPose::Create(pSdkManager,pPatch->GetName());

        for (i=0; i<lClusteredFbxNodes.GetCount(); i++)
        {
            KFbxNode*  lKFbxNode   = lClusteredFbxNodes.GetAt(i);
            KFbxMatrix lBindMatrix = lKFbxNode->GetGlobalFromDefaultTake(KFbxNode::eSOURCE_SET);

            lPose->Add(lKFbxNode, lBindMatrix);
        }

        // Add the pose to the scene
        pScene->AddPose(lPose);
    }
}

// Store a Rest Pose
void StoreRestPose(KFbxSdkManager* pSdkManager, KFbxScene* pScene, KFbxNode* pSkeletonRoot)
{
    // This example show an arbitrary rest pose assignment.
    // This rest pose will set the bone rotation to the same value 
    // as time 1 second in the first take of animation, but the 
    // position of the bone will be set elsewhere in the scene.
    KString     lNodeName;
    KFbxNode*   lKFbxNode;
    KFbxMatrix  lTransformMatrix;
    KFbxVector4 lT,lR,lS(1.0, 1.0, 1.0);

    // Create the rest pose
    KFbxPose* lPose = KFbxPose::Create(pSdkManager,"A Bind Pose");

    // Set the skeleton root node to the global position (10, 10, 10)
    // and global rotation of 45deg along the Z axis.
    lT.Set(10.0, 10.0, 10.0);
    lR.Set( 0.0,  0.0, 45.0);

    lTransformMatrix.SetTRS(lT, lR, lS);

    // Add the skeleton root node to the pose
    lKFbxNode = pSkeletonRoot;
    lPose->Add(lKFbxNode, lTransformMatrix, false /*it's a global matrix*/);

    // Set the lLimbNode1 node to the local position of (0, 40, 0)
    // and local rotation of -90deg along the Z axis. This show that
    // you can mix local and global coordinates in a rest pose.
    lT.Set(0.0, 40.0,   0.0);
    lR.Set(0.0,  0.0, -90.0);

    lTransformMatrix.SetTRS(lT, lR, lS);

    // Add the skeleton second node to the pose
    lKFbxNode = lKFbxNode->GetChild(0);
    lPose->Add(lKFbxNode, lTransformMatrix, true /*it's a local matrix*/);

    // Set the lLimbNode2 node to the local position of (0, 40, 0)
    // and local rotation of 45deg along the Z axis.
    lT.Set(0.0, 40.0, 0.0);
    lR.Set(0.0,  0.0, 45.0);

    lTransformMatrix.SetTRS(lT, lR, lS);

    // Add the skeleton second node to the pose
    lKFbxNode = lKFbxNode->GetChild(0);
    lNodeName = lKFbxNode->GetName();
    lPose->Add(lKFbxNode, lTransformMatrix, true /*it's a local matrix*/);

    // Now add the pose to the scene
    pScene->AddPose(lPose);
}

// Add the specified node to the node array. Also, add recursively
// all the parent node of the specified node to the array.
void AddNodeRecursively(KArrayTemplate<KFbxNode*>& pNodeArray, KFbxNode* pNode)
{
    if (pNode)
    {
        AddNodeRecursively(pNodeArray, pNode->GetParent());

        if (pNodeArray.Find(pNode) == -1)
        {
            // Node not in the list, add it
            pNodeArray.Add(pNode);
        }
    }
}

void SetXMatrix(KFbxXMatrix& pXMatrix, const KFbxMatrix& pMatrix)
{
    memcpy((double*)pXMatrix, &pMatrix.mData[0][0], sizeof(pMatrix.mData));
}
