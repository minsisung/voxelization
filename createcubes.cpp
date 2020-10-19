#include "createcubes.h"
#include <QDebug>
#include<iostream>
#include <QElapsedTimer>
#include <math.h>



CreateCubes::CreateCubes():
    m_totalCount(0)
{
}

void CreateCubes::setupInitialTransformation(MachineTool &MT)
{
    //setup transformation matrix for each component  (X,Y,Z,A,B,C)
    //translational unit: meter
    //rotary unit: degree
    voxelizer.setupInitialTransformationMatrix(MT, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void CreateCubes::setupTransformation(MachineTool &MT, QChar linkType, float amount)
{
    voxelizer.setTransformationMatrix(MT, linkType, amount);
}

void CreateCubes::createMTVoxelspace(float vSize, QVector<stl_reader::StlMesh <float, unsigned int>>& STLMeshVector)
{
    //setup voxelsize
    voxelSize = vSize;

    //initialize voxel space and voxel size
    voxelizer.setupSize(voxelSize, STLMeshVector);
}

QString CreateCubes::createMTVoxelspace(float vSize, QVector<component> &componentVector)
{
    //setup voxelsize
    voxelSize = vSize;

    //initialize voxel space and voxel size and return the name of the lowest component
    QString lowestCompName = voxelizer.setupSize(voxelSize, componentVector);

    return lowestCompName;
}

QVector<contactComponentsPair> CreateCubes::findContactComponentsPairs(QVector<component>& componentVector)
{
    QVector<contactComponentsPair> ccpVector = voxelizer.collisionDetectionForComponents(componentVector);

    //Initialize CCPs for intial grouping usage
    for(int ind_ccp = 0; ind_ccp < ccpVector.size(); ind_ccp++){
        //build CCPs
        QVector<QString> ccp;
        ccp.append(ccpVector[ind_ccp].getFirstMeshName());
        ccp.append(ccpVector[ind_ccp].getSecondMeshName());
        CCPs.append(ccp);
    }

    qDebug()<<"---------------------The number of components:"<<componentVector.size()<<"----------------------"<<endl;
    qDebug()<<"---------------------The number of CCPs:"<<ccpVector.size()<<"----------------------"<<endl;
    voxelizer.updateCCPVector(ccpVector);

    //print all motion relationship between components of CCPs
    for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
        qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
        qDebug()<<"Collision occurs when move in positive X:"<<ccpVector[ccp_ind].isCollided_Positive_X();
        qDebug()<<"Collision occurs when move in negative X:"<<ccpVector[ccp_ind].isCollided_Negative_X();
        qDebug()<<"Collision occurs when move in positive Y:"<<ccpVector[ccp_ind].isCollided_Positive_Y();
        qDebug()<<"Collision occurs when move in negative Y:"<<ccpVector[ccp_ind].isCollided_Negative_Y();
        qDebug()<<"Collision occurs when move in positive Z:"<<ccpVector[ccp_ind].isCollided_Positive_Z();
        qDebug()<<"Collision occurs when move in negative Z:"<<ccpVector[ccp_ind].isCollided_Negative_Z();

        if(ccpVector[ccp_ind].containsCommonRotaryAxis1()){
            qDebug()<<"Collision occurs when rotate along"<<ccpVector[ccp_ind].firstAxis<<":"
                   <<ccpVector[ccp_ind].isCollided_FirstAxis();
        }else{
            qDebug()<<"This CCP doesn't have common rotational axis in"<<ccpVector[ccp_ind].firstAxis;
        }

        if(ccpVector[ccp_ind].containsCommonRotaryAxis2()){
            qDebug()<<"Collision occurs when rotate along"<<ccpVector[ccp_ind].secondAxis<<":"
                   <<ccpVector[ccp_ind].isCollided_SecondAxis();
        }else{
            qDebug()<<"This CCP doesn't have common rotational axis in"<<ccpVector[ccp_ind].secondAxis;
        }
        qDebug()<<endl;
    }

    return ccpVector;
}


int CreateCubes::findLIPCandidates(QVector<contactComponentsPair> &ccpVector)
{
    for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){

        //Rotational LIP candidate
        //first common axis
        if(ccpVector[ccp_ind].containsCommonRotaryAxis1() && !ccpVector[ccp_ind].isCollided_FirstAxis()){
            //both components contain offset mesh
            if(ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() && ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()){
                //A axis
                if(ccpVector[ccp_ind].firstAxis == "A"){
                    if((ccpVector[ccp_ind].isCollided_Positive_Y() | ccpVector[ccp_ind].isCollided_Negative_Y()) &&
                            (ccpVector[ccp_ind].isCollided_Positive_Z() | ccpVector[ccp_ind].isCollided_Negative_Z())){
                        qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                        qDebug()<<"is a candidate of A-LIP"<<endl;

                        //fill LIPs
                        QVector<QString> lip;
                        QString axis = "A";
                        lip.append(ccpVector[ccp_ind].getFirstMeshName());
                        lip.append(ccpVector[ccp_ind].getSecondMeshName());
                        LIPs.append(qMakePair(axis, lip));
                        continue;
                    }
                }
                //B axis
                if(ccpVector[ccp_ind].firstAxis == "B"){
                    if((ccpVector[ccp_ind].isCollided_Positive_X() | ccpVector[ccp_ind].isCollided_Negative_X()) &&
                            (ccpVector[ccp_ind].isCollided_Positive_Z() | ccpVector[ccp_ind].isCollided_Negative_Z())){
                        qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                        qDebug()<<"is a candidate of B-LIP"<<endl;

                        //fill LIPs
                        QVector<QString> lip;
                        QString axis = "B";
                        lip.append(ccpVector[ccp_ind].getFirstMeshName());
                        lip.append(ccpVector[ccp_ind].getSecondMeshName());
                        LIPs.append(qMakePair(axis, lip));
                        continue;
                    }
                }
                //C axis
                if(ccpVector[ccp_ind].firstAxis == "C"){
                    if((ccpVector[ccp_ind].isCollided_Positive_X() | ccpVector[ccp_ind].isCollided_Negative_X()) &&
                            (ccpVector[ccp_ind].isCollided_Positive_Y() | ccpVector[ccp_ind].isCollided_Negative_Y())){
                        qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                        qDebug()<<"is a candidate of C-LIP"<<endl;

                        //fill LIPs
                        QVector<QString> lip;
                        QString axis = "C";
                        lip.append(ccpVector[ccp_ind].getFirstMeshName());
                        lip.append(ccpVector[ccp_ind].getSecondMeshName());
                        LIPs.append(qMakePair(axis, lip));
                        continue;
                    }
                }
            }
        }

        //second common axis
        if(ccpVector[ccp_ind].containsCommonRotaryAxis2() && !ccpVector[ccp_ind].isCollided_SecondAxis()){
            //both components contain offset mesh
            if(ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() && ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()){
                //A axis
                if(ccpVector[ccp_ind].secondAxis == "A"){
                    if((ccpVector[ccp_ind].isCollided_Positive_Y() | ccpVector[ccp_ind].isCollided_Negative_Y()) &&
                            (ccpVector[ccp_ind].isCollided_Positive_Z() | ccpVector[ccp_ind].isCollided_Negative_Z())){
                        qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                        qDebug()<<"is a candidate of A-LIP"<<endl;

                        //fill LIPs
                        QVector<QString> lip;
                        QString axis = "A";
                        lip.append(ccpVector[ccp_ind].getFirstMeshName());
                        lip.append(ccpVector[ccp_ind].getSecondMeshName());
                        LIPs.append(qMakePair(axis, lip));
                        continue;
                    }
                }
                //B axis
                if(ccpVector[ccp_ind].secondAxis == "B"){
                    if((ccpVector[ccp_ind].isCollided_Positive_X() | ccpVector[ccp_ind].isCollided_Negative_X()) &&
                            (ccpVector[ccp_ind].isCollided_Positive_Z() | ccpVector[ccp_ind].isCollided_Negative_Z())){
                        qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                        qDebug()<<"is a candidate of B-LIP"<<endl;

                        //fill LIPs
                        QVector<QString> lip;
                        QString axis = "B";
                        lip.append(ccpVector[ccp_ind].getFirstMeshName());
                        lip.append(ccpVector[ccp_ind].getSecondMeshName());
                        LIPs.append(qMakePair(axis, lip));
                        continue;
                    }
                }
                //C axis
                if(ccpVector[ccp_ind].secondAxis == "C"){
                    if((ccpVector[ccp_ind].isCollided_Positive_X() | ccpVector[ccp_ind].isCollided_Negative_X()) &&
                            (ccpVector[ccp_ind].isCollided_Positive_Y() | ccpVector[ccp_ind].isCollided_Negative_Y())){
                        qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                        qDebug()<<"is a candidate of C-LIP"<<endl;

                        //fill LIPs
                        QVector<QString> lip;
                        QString axis = "C";
                        lip.append(ccpVector[ccp_ind].getFirstMeshName());
                        lip.append(ccpVector[ccp_ind].getSecondMeshName());
                        LIPs.append(qMakePair(axis, lip));
                        continue;
                    }
                }
            }
        }

        //X-LIP candidate
        if(!ccpVector[ccp_ind].isCollided_Positive_X() && !ccpVector[ccp_ind].isCollided_Negative_X()){
            if(((ccpVector[ccp_ind].isCollided_Positive_Y() | ccpVector[ccp_ind].isCollided_Negative_Y()) &&
                (ccpVector[ccp_ind].isCollided_Positive_Z() && ccpVector[ccp_ind].isCollided_Negative_Z())) |
                    ((ccpVector[ccp_ind].isCollided_Positive_Y() && ccpVector[ccp_ind].isCollided_Negative_Y()) &&
                     (ccpVector[ccp_ind].isCollided_Positive_Z() | ccpVector[ccp_ind].isCollided_Negative_Z()))){
                //both components contain offset mesh
                if(ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() && ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()){
                    qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                    qDebug()<<"is a candidate of X-LIP"<<endl;

                    //fill LIPs
                    QVector<QString> lip;
                    QString axis = "X";
                    lip.append(ccpVector[ccp_ind].getFirstMeshName());
                    lip.append(ccpVector[ccp_ind].getSecondMeshName());
                    LIPs.append(qMakePair(axis, lip));
                    continue;
                }
            }
        }

        //Y-LIP candidate
        if(!ccpVector[ccp_ind].isCollided_Positive_Y() && !ccpVector[ccp_ind].isCollided_Negative_Y()){
            if(((ccpVector[ccp_ind].isCollided_Positive_X() | ccpVector[ccp_ind].isCollided_Negative_X()) &&
                (ccpVector[ccp_ind].isCollided_Positive_Z() && ccpVector[ccp_ind].isCollided_Negative_Z())) |
                    ((ccpVector[ccp_ind].isCollided_Positive_X() && ccpVector[ccp_ind].isCollided_Negative_X()) &&
                     (ccpVector[ccp_ind].isCollided_Positive_Z() | ccpVector[ccp_ind].isCollided_Negative_Z()))){
                //both components contain offset mesh
                if(ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() && ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()){
                    qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                    qDebug()<<"is a candidate of Y-LIP"<<endl;

                    //fill LIPs
                    QVector<QString> lip;
                    QString axis = "Y";
                    lip.append(ccpVector[ccp_ind].getFirstMeshName());
                    lip.append(ccpVector[ccp_ind].getSecondMeshName());
                    LIPs.append(qMakePair(axis, lip));
                    continue;
                }
            }
        }

        //Z-LIP candidate
        if(!ccpVector[ccp_ind].isCollided_Positive_Z() && !ccpVector[ccp_ind].isCollided_Negative_Z()){
            if(((ccpVector[ccp_ind].isCollided_Positive_Y() | ccpVector[ccp_ind].isCollided_Negative_Y()) &&
                (ccpVector[ccp_ind].isCollided_Positive_X() && ccpVector[ccp_ind].isCollided_Negative_X())) |
                    ((ccpVector[ccp_ind].isCollided_Positive_Y() && ccpVector[ccp_ind].isCollided_Negative_Y()) &&
                     (ccpVector[ccp_ind].isCollided_Positive_X() | ccpVector[ccp_ind].isCollided_Negative_X()))){
                //both components contain offset mesh
                if(ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() && ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()){
                    qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                    qDebug()<<"is a candidate of Z-LIP"<<endl;

                    //fill LIPs
                    QVector<QString> lip;
                    QString axis = "Z";
                    lip.append(ccpVector[ccp_ind].getFirstMeshName());
                    lip.append(ccpVector[ccp_ind].getSecondMeshName());
                    LIPs.append(qMakePair(axis, lip));
                    continue;
                }
            }
        }
    }

    //draw selected CCP with relative movement
    drawVoxelforCCP(voxelizer.compVoxelIndicesList1, voxelizer.compVoxelIndicesList2);

    return LIPs.size();
}


void CreateCubes::collisionDetectionForConfigurations(MachineTool& MT, bool needVisualization)
{
    //create voxel space to check collisoin for all configurations
    voxelizer.createVoxelSapce();

    //setup transformation matrix for each component
    setupInitialTransformation(MT);
    qDebug()<<"Finished setup initial transformation matrix for links"<<endl;

    //translational unit: meter
    //rotary unit: degree
    //    voxelizer.setTransformationMatrix(MT, 'Y', 0.6f);

    //initialize if visualization is necessary
    ifNeedVisualization = needVisualization;

    // create voxel models for every link------------------------------------------------

        for (QVector<Link>::iterator loop = MT.LinkVector.begin();loop != MT.LinkVector.end(); loop++){
            voxelizer.parentModelVoxelization(*loop);
        }
    // create voxel models for every link------------------------------------------------


    // Start creating parent voxel models  ----------------------------------------------------------------------

    //set joint limit

//    for (QVector<Joint>::iterator loop = MT.JointVector.begin(); loop != MT.JointVector.end(); loop++){
//        QChar linkType = loop->getChildLink()->getLinkType();

//        switch (linkType.toLatin1()) {
//        case 'X':
//            loop->setLowerLimit(-0.1f);
//            loop->setUpperLimit(0.1f);
//            break;
//        case 'Y':
//            loop->setLowerLimit(-0.1f);
//            loop->setUpperLimit(0.1f);
//            break;
//        case 'Z':
//            loop->setLowerLimit(-0.1f);
//            loop->setUpperLimit(0.1f);
//            break;
//        case 'A':
//            loop->setLowerLimit(-45.0f);
//            loop->setUpperLimit(45.0f);
//            break;
//        case 'B':
//            loop->setLowerLimit(-45.0f);
//            loop->setUpperLimit(45.0f);
//            break;
//        case 'C':
//            loop->setLowerLimit(0.0f);
//            loop->setUpperLimit(45.0f);
//            break;
//        }
//    }

//    qDebug()<<endl<<"Start creating parent voxel models ----------------------------------------------"<<endl<<endl;

//    //find and voxelize base link
//    Link* baseLink = MT.baseLink;
//    voxelizer.parentModelVoxelization(*baseLink);

//    //timer
//    QElapsedTimer parentModelstimer;
//    parentModelstimer.start();

    int samplingNumber = 1;

//    for(int Number = 0; Number < baseLink->ChildLink.size(); Number++){
//        Link* currentLink = baseLink->ChildLink[Number];

//        while(currentLink != nullptr){

//            if(currentLink->isRotaitonal){
//                // voxelize rotary links

//                for(int i = 0; i < samplingNumber; ++i){
//                    QChar linkType1 = currentLink->getLinkType();
//                    float lowerLimit1 = currentLink->getLowerLimit();
//                    float upperLimit1 = currentLink->getUpperLimit();
//                    float motionRange1 = 0;
//                    if(samplingNumber !=1){
//                        motionRange1 = (upperLimit1 - lowerLimit1)/(samplingNumber - 1)* i + lowerLimit1;
//                    }else{
//                        motionRange1 = lowerLimit1;
//                    }

//                    //translational unit: meter
//                    //rotary unit: degree
//                    voxelizer.setTransformationMatrix(MT, linkType1 ,motionRange1);
//                    voxelizer.parentModelVoxelization(*currentLink);

//                    if(!currentLink->ChildLink.isEmpty()){
//                        currentLink = currentLink->ChildLink[0];
//                        for(int j = 0; j < samplingNumber; ++j){
//                            QChar linkType2 = currentLink->getLinkType();
//                            float lowerLimit2 = currentLink->getLowerLimit();
//                            float upperLimit2 = currentLink->getUpperLimit();
//                            float motionamount2 = 0;
//                            if(samplingNumber !=1){
//                                motionamount2 = (upperLimit2 - lowerLimit2)/(samplingNumber - 1) * j + lowerLimit2;
//                            }else{
//                                motionamount2 = lowerLimit2;
//                            }

//                            //translational unit: meter
//                            //rotary unit: degree
//                            voxelizer.setTransformationMatrix(MT, linkType2, motionamount2);
//                            voxelizer.parentModelVoxelization(*currentLink);

//                            if(j == samplingNumber-1)
//                                currentLink = currentLink->ParentLink;
//                        }
//                    }
//                    if(i == samplingNumber-1)
//                        break;
//                }
//                break;
//            }else{
//                // voxelize translational links
//                voxelizer.parentModelVoxelization(*currentLink);

//                if(!currentLink->ChildLink.isEmpty()){
//                    currentLink = currentLink->ChildLink[0];
//                }else{
//                    break;
//                }
//            }
//        }
//    }

//    qDebug() << "Creating parent models took"<< parentModelstimer.elapsed() << "milliseconds"<<endl;
//    qDebug()<<endl<<"Finish creating parent voxel models ------------------------------------"<<endl<<endl;

    //     Finish creating parent voxel models  ---------------------------------------------------------------------


    //    Checking all configuration for collision-----------------------------------------------

    //    QSet<QString> totalCollisionSet;

    //    //timer
    //    QElapsedTimer collisionDetectiontimer;
    //    collisionDetectiontimer.start();

    //    float shiftX = 0;
    //    float shiftY = 0;
    //    float shiftZ = 0;
    //    float rotateFirst =0;
    //    float rotateSecond = 0;

    //    for(int samplingX = 0; samplingX < samplingNumber; samplingX++){
    //        for(int samplingY = 0; samplingY < samplingNumber; samplingY++){
    //            for(int samplingZ = 0; samplingZ < samplingNumber; samplingZ++){
    //                if(samplingNumber !=1){
    //                    shiftX = (MT.xLink->getUpperLimit() - MT.xLink->getLowerLimit())/(samplingNumber - 1)* samplingX + MT.xLink->getLowerLimit();
    //                    shiftY = (MT.yLink->getUpperLimit() - MT.yLink->getLowerLimit())/(samplingNumber - 1)* samplingY + MT.yLink->getLowerLimit();
    //                    shiftZ = (MT.zLink->getUpperLimit() - MT.zLink->getLowerLimit())/(samplingNumber - 1)* samplingZ + MT.zLink->getLowerLimit();
    //                }else{
    //                    shiftX = MT.xLink->getLowerLimit();
    //                    shiftY = MT.yLink->getLowerLimit();
    //                    shiftZ = MT.zLink->getLowerLimit();
    //                }

    //                voxelizer.shiftVoxelModel(MT, shiftX, shiftY, shiftZ);

    //                for(int firstRotarySamplingNumber = 0; firstRotarySamplingNumber < samplingNumber; firstRotarySamplingNumber++){
    //                    for(int secondRotarySamplingNumber = 0; secondRotarySamplingNumber < samplingNumber; secondRotarySamplingNumber++){
    //                        if(samplingNumber !=1){
    //                            rotateFirst =(MT.firstRotaryLink->getUpperLimit() - MT.firstRotaryLink->getLowerLimit())/(samplingNumber - 1)* firstRotarySamplingNumber + MT.firstRotaryLink->getLowerLimit();
    //                            rotateSecond =(MT.secondRotaryLink->getUpperLimit() - MT.secondRotaryLink->getLowerLimit())/(samplingNumber - 1)* secondRotarySamplingNumber + MT.secondRotaryLink->getLowerLimit();
    //                        }else{
    //                            rotateFirst = MT.firstRotaryLink->getLowerLimit();
    //                            rotateSecond = MT.secondRotaryLink->getLowerLimit();
    //                        }
    //                        totalCollisionSet +=
    //                                voxelizer.collisionDetection(MT, firstRotarySamplingNumber,
    //                                                             firstRotarySamplingNumber * samplingNumber + secondRotarySamplingNumber);

    //                        if(totalCollisionSet.empty()){
    //                            qDebug()<<"No collision occurs at X shift:"<<shiftX<<" Y shift:"<<shiftY
    //                                   <<" Z shift:"<<shiftZ<<MT.firstRotaryLink->getLinkType()<<"rotayte:"<<rotateFirst<<
    //                                     MT.secondRotaryLink->getLinkType()<<"rotate:"<<rotateSecond<<endl;
    //                        }else{
    //                            qDebug()<<"Collision pairs at X shift:"<<shiftX<<" Y shift:"<<shiftY
    //                                   <<" Z shift:"<<shiftZ<<MT.firstRotaryLink->getLinkType()<<"rotayte:"<<rotateFirst<<
    //                                     MT.secondRotaryLink->getLinkType()<<"rotate:"<<rotateSecond<<":"
    //                                  <<totalCollisionSet<<endl;
    //                        }
    //                        totalCollisionSet.clear();
    //                    }
    //                }
    //            }
    //        }
    //    }
    //    qDebug() << "Collision detection with"<<samplingNumber<<"sampling point for each axis took"
    //             << collisionDetectiontimer.elapsed() << "milliseconds"<<endl;

    //    Checking all configuration for collision-----------------------------------------------



    //Checking only one configuration for collision-----------------------------------------------
    voxelizer.shiftVoxelModel(MT, 0.0 * 1, 0.0 *1, 0.0 *1);
    QSet<QString> totalCollisionSet;
    totalCollisionSet +=
            voxelizer.collisionDetectionForGroups(MT, 0, 0 * samplingNumber + 0);

    if(totalCollisionSet.empty()){
        qDebug()<<"No collision occurs at X shift:"<<0<<" Y shift:"<<0
               <<" Z shift:"<<0<<MT.firstRotaryLink->getLinkType()<<"rotayte:"<<0<<
                 MT.secondRotaryLink->getLinkType()<<"rotate:"<<0<<endl;
    }else{
        qDebug()<<"Collision pairs at X shift:"<<0<<" Y shift:"<<0
               <<" Z shift:"<<0<<MT.firstRotaryLink->getLinkType()<<"rotayte:"<<0<<
                 MT.secondRotaryLink->getLinkType()<<"rotate:"<<0<<":"
              <<totalCollisionSet<<endl;
    }
    totalCollisionSet.clear();

    //Checking only one configuration for collision-----------------------------------------------


    // check if visualization is necessary     ===========================================================
    if(ifNeedVisualization){
        for (QVector<Link>::iterator loop = MT.LinkVector.begin();loop != MT.LinkVector.end(); loop++){

            //timer
            QElapsedTimer timer;
            timer.start();
            drawVoxelforMT(*loop,0,0);
            qDebug() << "The creation of cubes for"<<loop->getLinkType()<< "took" << timer.elapsed() << "milliseconds"<<endl;
        }
    }
}


void CreateCubes::drawVoxelforMT(Link& link, int ind1, int ind2)
{
    float voxelSpace_X_min = voxelizer.voxelStarting_X;
    float voxelSpace_Y_min = voxelizer.voxelStarting_Y;
    float voxelSpace_Z_min = voxelizer.voxelStarting_Z;

    int index = 0;

    if(link.isFirstRotational)
        index = ind1;

    if(link.isSecondRotational)
        index = ind2;

    for(int mesh_ind = 0; mesh_ind < link.MTVoxelIndicesListVector.size(); ++mesh_ind){
        for (QList<QVector3D>::iterator i = link.MTVoxelIndicesListVectorUpdate[mesh_ind][index].begin();
             i != link.MTVoxelIndicesListVectorUpdate[mesh_ind][index].end(); ++i){

            int number_x = i->x();
            int number_y = i->y();
            int number_z = i->z();

            GLfloat offset_y = voxelSize * number_y;
            GLfloat offset_x = voxelSize * number_x;
            GLfloat offset_z = voxelSize * number_z;
            GLfloat x_right = voxelSpace_X_min + voxelSize + offset_x;
            GLfloat x_left = voxelSpace_X_min + offset_x;
            GLfloat y_up = voxelSpace_Y_min + voxelSize + offset_y;
            GLfloat y_down = voxelSpace_Y_min + offset_y;
            GLfloat z_futher = voxelSpace_Z_min + offset_z;
            GLfloat z_closer = voxelSpace_Z_min + voxelSize + offset_z;

            //coordinates of triangles forming faces of cubes
            //6 faces for each cube, 4 vertices for each face, 3 coordinates for each vertex
            GLfloat coords[6][4][3] = {
                { { x_right, y_down, z_futher}, { x_left, y_down, z_futher}, { x_left, y_up, z_futher}, { x_right, y_up, z_futher} },
                { { x_right, y_up, z_futher}, { x_left, y_up, z_futher}, { x_left, y_up, z_closer}, { x_right, y_up, z_closer} },
                { { x_right, y_down, z_closer}, { x_right, y_down, z_futher}, { x_right, y_up, z_futher}, { x_right, y_up, z_closer} },
                { { x_left,y_down, z_futher}, { x_left, y_down, z_closer}, { x_left, y_up, z_closer}, { x_left, y_up, z_futher} },
                { { x_right,y_down, z_closer}, { x_left, y_down, z_closer}, { x_left, y_down, z_futher}, { x_right, y_down, z_futher} },
                { { x_left, y_down, z_closer}, { x_right, y_down, z_closer}, { x_right, y_up, z_closer}, { x_left, y_up, z_closer} }
            };

            QVector3D normal;
            bool ifDuplicate = false;

            // Draw6 different normal direction and avoid overlapping face for each face
            for (int i = 0; i < 6; ++i) {
                normal = setNormal(i);

                //                ifDuplicate = checkDuplicateFace(i, number_x, number_y, number_z);
                //                if(ifDuplicate)
                //                    continue;

                // insert vertex position into m_data for creating VBO

                //first triangle in the face
                m_data.resize(m_totalCount+36);
                GLfloat *p = m_data.data() + m_totalCount;
                *p++ = coords[i][0][0];
                *p++ = coords[i][0][1];
                *p++ = coords[i][0][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];
                //---------------------------------
                *p++ = coords[i][1][0];
                *p++ = coords[i][1][1];
                *p++ = coords[i][1][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];
                //---------------------------------
                *p++ = coords[i][2][0];
                *p++ = coords[i][2][1];
                *p++ = coords[i][2][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];

                /////////////////////////////////////////////
                //second triangle in the face
                *p++ = coords[i][0][0];
                *p++ = coords[i][0][1];
                *p++ = coords[i][0][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];
                //---------------------------------
                *p++ = coords[i][2][0];
                *p++ = coords[i][2][1];
                *p++ = coords[i][2][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];
                //---------------------------------
                *p++ = coords[i][3][0];
                *p++ = coords[i][3][1];
                *p++ = coords[i][3][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];
                m_totalCount += 36;

                //update number of vertices
                link.numberOfVertex += 6;
            }
        }
    }
}

void CreateCubes::drawVoxelforCCP(QList<QVector3D> &compVoxelIndicesList1, QList<QVector3D> &compVoxelIndicesList2)
{
    float voxelSpace_X_min = voxelizer.voxelStarting_X;
    float voxelSpace_Y_min = voxelizer.voxelStarting_Y;
    float voxelSpace_Z_min = voxelizer.voxelStarting_Z;

    for(int comp_ind = 0; comp_ind < 2; ++comp_ind){

        QList<QVector3D> compVoxelIndciesList;
        int *numberOfVertex;

        //first component (only first component being transform)
        if(comp_ind == 0){
            compVoxelIndciesList = compVoxelIndicesList1;
            numberOfVertex = &numberOfVertex_comp1;
        }else{
            compVoxelIndciesList = compVoxelIndicesList2;
            numberOfVertex = &numberOfVertex_comp2;
        }

        for (QList<QVector3D>::iterator i = compVoxelIndciesList.begin();
             i != compVoxelIndciesList.end(); ++i){

            int number_x = i->x();
            int number_y = i->y();
            int number_z = i->z();

            GLfloat offset_y = voxelSize * number_y;
            GLfloat offset_x = voxelSize * number_x;
            GLfloat offset_z = voxelSize * number_z;
            GLfloat x_right = voxelSpace_X_min + voxelSize + offset_x;
            GLfloat x_left = voxelSpace_X_min + offset_x;
            GLfloat y_up = voxelSpace_Y_min + voxelSize + offset_y;
            GLfloat y_down = voxelSpace_Y_min + offset_y;
            GLfloat z_futher = voxelSpace_Z_min + offset_z;
            GLfloat z_closer = voxelSpace_Z_min + voxelSize + offset_z;

            //coordinates of triangles forming faces of cubes
            //6 faces for each cube, 4 vertices for each face, 3 coordinates for each vertex
            GLfloat coords[6][4][3] = {
                { { x_right, y_down, z_futher}, { x_left, y_down, z_futher}, { x_left, y_up, z_futher}, { x_right, y_up, z_futher} },
                { { x_right, y_up, z_futher}, { x_left, y_up, z_futher}, { x_left, y_up, z_closer}, { x_right, y_up, z_closer} },
                { { x_right, y_down, z_closer}, { x_right, y_down, z_futher}, { x_right, y_up, z_futher}, { x_right, y_up, z_closer} },
                { { x_left,y_down, z_futher}, { x_left, y_down, z_closer}, { x_left, y_up, z_closer}, { x_left, y_up, z_futher} },
                { { x_right,y_down, z_closer}, { x_left, y_down, z_closer}, { x_left, y_down, z_futher}, { x_right, y_down, z_futher} },
                { { x_left, y_down, z_closer}, { x_right, y_down, z_closer}, { x_right, y_up, z_closer}, { x_left, y_up, z_closer} }
            };

            QVector3D normal;
            bool ifDuplicate = false;

            // Draw6 different normal direction and avoid overlapping face for each face
            for (int i = 0; i < 6; ++i) {
                normal = setNormal(i);

                //                ifDuplicate = checkDuplicateFace(i, number_x, number_y, number_z);
                //                if(ifDuplicate)
                //                    continue;

                // insert vertex position into m_data for creating VBO

                //first triangle in the face
                m_data.resize(m_totalCount+36);
                GLfloat *p = m_data.data() + m_totalCount;
                *p++ = coords[i][0][0];
                *p++ = coords[i][0][1];
                *p++ = coords[i][0][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];
                //---------------------------------
                *p++ = coords[i][1][0];
                *p++ = coords[i][1][1];
                *p++ = coords[i][1][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];
                //---------------------------------
                *p++ = coords[i][2][0];
                *p++ = coords[i][2][1];
                *p++ = coords[i][2][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];

                /////////////////////////////////////////////
                //second triangle in the face
                *p++ = coords[i][0][0];
                *p++ = coords[i][0][1];
                *p++ = coords[i][0][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];
                //---------------------------------
                *p++ = coords[i][2][0];
                *p++ = coords[i][2][1];
                *p++ = coords[i][2][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];
                //---------------------------------
                *p++ = coords[i][3][0];
                *p++ = coords[i][3][1];
                *p++ = coords[i][3][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];
                m_totalCount += 36;

                //update number of vertices
                *numberOfVertex += 6;
            }
        }
    }
}


void CreateCubes::createCollisionVoxelspace(float vSize, MachineTool& MT ,bool needVisualization)
{
    //initialize if visualization is necessary
    ifNeedVisualization = needVisualization;

    //setup voxelsize
    voxelSize = vSize;

    //    voxelizer.setupSize(voxelSize, MT);

    //setup transformation matrix for each component
    setupInitialTransformation(MT);

    //translational unit: meter
    //rotary unit: degree
    //    voxelizer.setTransformationMatrix(MT, 'Y', 0.6f);


    // create voxel models for every link------------------------------------------------

    //    for (QVector<Link>::iterator loop = MT.LinkVector.begin();loop != MT.LinkVector.end(); loop++){
    //        voxelizer.Voxelize(*loop, true);

    //        //reset bounding index for next component
    //        voxelizer.reset_bounding_index();
    //    }
    // create voxel models for every link------------------------------------------------


    // create parent voxel models  ----------------------------------------------------------------------

    //set joint limit

    //    for (QVector<Joint>::iterator loop = MT.JointVector.begin(); loop != MT.JointVector.end(); loop++){
    //        QChar linkType = loop->getChildLink()->getLinkType();

    //        switch (linkType.toLatin1()) {
    //        case 'X':
    //            loop->setLowerLimit(-0.1f);
    //            loop->setUpperLimit(0.1f);
    //            break;
    //        case 'Y':
    //            loop->setLowerLimit(-0.1f);
    //            loop->setUpperLimit(0.1f);
    //            break;
    //        case 'Z':
    //            loop->setLowerLimit(-0.1f);
    //            loop->setUpperLimit(0.1f);
    //            break;
    //        case 'A':
    //            loop->setLowerLimit(-90.0f);
    //            loop->setUpperLimit(0.0f);
    //            break;
    //        case 'C':
    //            loop->setLowerLimit(0.0f);
    //            loop->setUpperLimit(90.0f);
    //            break;
    //        }
    //    }

    //    //find and voxelize base link
    //    Link* baseLink = nullptr;

    //    for (QVector<Link>::iterator loop = MT.LinkVector.begin(); loop != MT.LinkVector.end(); loop++){
    //        if(loop->ParentLink == nullptr)
    //        {
    //            baseLink = loop;
    //            voxelizer.Voxelize(*baseLink);

    //            break;
    //        }
    //    }

    //    // only works for 50 configuration type
    //    Link* currentLink = baseLink->ChildLink[0];
    //    int samplingNumber = 3;
    //    while(currentLink->ChildLink[0] != nullptr){
    //        QChar linkType = currentLink->getLinkType();

    //        if(linkType == 'A' |linkType == 'B' | linkType == 'C'){
    //            for(int i = 0; i < samplingNumber; ++i){
    //                QChar linkType1 = currentLink->getLinkType();
    //                float lowerLimit1 = currentLink->getLowerLimit();
    //                float upperLimit1 = currentLink->getUpperLimit();
    //                float motionRange1 = (upperLimit1 - lowerLimit1)/samplingNumber * i + lowerLimit1;

    //                //translational unit: meter
    //                //rotary unit: degree
    //                voxelizer.setTransformationMatrix(MT, linkType1,motionRange1);
    //                voxelizer.Voxelize(*currentLink);


    //                currentLink = currentLink->ChildLink[0];

    //                for(int j = 0; j < samplingNumber; ++j){
    //                    QChar linkType2 = currentLink->getLinkType();
    //                    float lowerLimit2 = currentLink->getLowerLimit();
    //                    float upperLimit2 = currentLink->getUpperLimit();
    //                    float motionamount2 = (upperLimit2 - lowerLimit2)/samplingNumber * j + lowerLimit2;

    //                    //translational unit: meter
    //                    //rotary unit: degree
    //                    voxelizer.setTransformationMatrix(MT, linkType2, motionamount2);
    //                    voxelizer.Voxelize(*currentLink);


    //                    if(j == samplingNumber-1)
    //                        currentLink = currentLink->ParentLink;
    //                }
    //                if(i == samplingNumber-1)
    //                    break;
    //            }
    //            break;

    //        }else{
    //            voxelizer.Voxelize(*currentLink);
    //            currentLink = currentLink->ChildLink[0];
    //        }
    //    }

    // create parent voxel models  ----------------------------------------------------------------------

    //    Link* link1 = baseLink->ChildLink[0];
    //    Link* link2 = link1->ChildLink[0];
    //    Link* link3 = link2->ChildLink[0];
    //    Link* link4  = link3->ChildLink[0];
    //    Link* link5 = link4->ChildLink[0];

    //    QChar linkType1 = link1->getLinkType();
    //    QChar linkType2 = link2->getLinkType();
    //    QChar linkType3 = link3->getLinkType();
    //    QChar linkType4 = link4->getLinkType();
    //    QChar linkType5 = link5->getLinkType();

    //    float motion1 = link1->getLowerLimit()+
    //            (link1->getUpperLimit() - link1->getLowerLimit()) * 0 / samplingNumber;
    //    float motion2 = link2->getLowerLimit()+
    //            (link2->getUpperLimit() - link2->getLowerLimit()) * 0 / samplingNumber;
    //    float motion3 = link3->getLowerLimit()+
    //            (link3->getUpperLimit() - link3->getLowerLimit()) * 0 / samplingNumber;
    //    float motion4 = link4->getLowerLimit()+
    //            (link4->getUpperLimit() - link4->getLowerLimit()) * 0 / samplingNumber;
    //    float motion5 = link5->getLowerLimit()+
    //            (link5->getUpperLimit() - link5->getLowerLimit()) * 0 / samplingNumber;


    //        for (int ind_1 = 0; ind_1 < samplingNumber + 1; ind_1++ ){
    //            motion1 = link1->getLowerLimit() +
    //                    (link1->getUpperLimit() - link1->getLowerLimit()) * ind_1 / samplingNumber;

    //            QSet<QString> totalCollisionSet = voxelizer.translateVoxelModel(MT, linkType1, motion1, 0, 0);
    //            if(totalCollisionSet.empty()){
    //                qDebug()<<"No collision at"<<linkType1<<"="<<motion1<<linkType2<<"="<<motion2<<linkType3<<"="<<motion3
    //                       <<linkType4<<"="<<motion4<<linkType5<<"="<<motion5<<endl;
    //            }else{
    //                qDebug()<<"Collision pairs at"<<linkType1<<"="<<motion1<<linkType2<<"="<<motion2<<linkType3<<"="<<motion3
    //                       <<linkType4<<"="<<motion4<<linkType5<<"="<<motion5<<":"<<totalCollisionSet<<endl;
    //            }

    //            for (int ind_2 = 1; ind_2 < samplingNumber + 1; ind_2++ ){
    //                motion2 = link2->getLowerLimit() +
    //                        (link2->getUpperLimit() - link2->getLowerLimit()) * ind_2 / samplingNumber;

    //                QSet<QString> totalCollisionSet = voxelizer.translateVoxelModel(MT, linkType2, motion2, 0, 0);
    //                if(totalCollisionSet.empty()){
    //                    qDebug()<<"No collision at"<<linkType1<<"="<<motion1<<linkType2<<"="<<motion2<<linkType3<<"="<<motion3
    //                           <<linkType4<<"="<<motion4<<linkType5<<"="<<motion5<<endl;
    //                }else{
    //                    qDebug()<<"Collision pairs at"<<linkType1<<"="<<motion1<<linkType2<<"="<<motion2<<linkType3<<"="<<motion3
    //                           <<linkType4<<"="<<motion4<<linkType5<<"="<<motion5<<":"<<totalCollisionSet<<endl;
    //                }

    //                for (int ind_3 = 1; ind_3 < samplingNumber + 1; ind_3++ ){
    //                    motion3 = link3->getLowerLimit() +
    //                            (link3->getUpperLimit() - link3->getLowerLimit()) * ind_3 / samplingNumber;

    //                    QSet<QString> totalCollisionSet = voxelizer.translateVoxelModel(MT, linkType3, motion3, 0, 0);
    //                    if(totalCollisionSet.empty()){
    //                        qDebug()<<"No collision at"<<linkType1<<"="<<motion1<<linkType2<<"="<<motion2<<linkType3<<"="<<motion3
    //                               <<linkType4<<"="<<motion4<<linkType5<<"="<<motion5<<endl;
    //                    }else{
    //                        qDebug()<<"Collision pairs at"<<linkType1<<"="<<motion1<<linkType2<<"="<<motion2<<linkType3<<"="<<motion3
    //                               <<linkType4<<"="<<motion4<<linkType5<<"="<<motion5<<":"<<totalCollisionSet<<endl;
    //                    }

    //                    for (int ind_4 = 1; ind_4<samplingNumber + 1; ind_4++ ){
    //                        motion4 = link4->getLowerLimit() +
    //                                (link4->getUpperLimit() - link4->getLowerLimit()) * ind_4 / samplingNumber;

    //                        QSet<QString> totalCollisionSet = voxelizer.translateVoxelModel(MT, linkType4, motion4, ind_4, 0);
    //                        if(totalCollisionSet.empty()){
    //                            qDebug()<<"No collision at"<<linkType1<<"="<<motion1<<linkType2<<"="<<motion2<<linkType3<<"="<<motion3
    //                                   <<linkType4<<"="<<motion4<<linkType5<<"="<<motion5<<endl;
    //                        }else{
    //                            qDebug()<<"Collision pairs at"<<linkType1<<"="<<motion1<<linkType2<<"="<<motion2<<linkType3<<"="<<motion3
    //                                   <<linkType4<<"="<<motion4<<linkType5<<"="<<motion5<<":"<<totalCollisionSet<<endl;
    //                        }

    //                        for (int ind_5 = 1; ind_5<samplingNumber + 1; ind_5++ ){
    //                            motion5 = link5->getLowerLimit() +
    //                                    (link5->getUpperLimit() - link5->getLowerLimit()) * ind_5 / samplingNumber;

    //                            QSet<QString> totalCollisionSet = voxelizer.translateVoxelModel(MT, linkType5, motion5, ind_4, ind_4 * samplingNumber + ind_5);
    //                            if(totalCollisionSet.empty()){
    //                                qDebug()<<"No collision at"<<linkType1<<"="<<motion1<<linkType2<<"="<<motion2<<linkType3<<"="<<motion3
    //                                       <<linkType4<<"="<<motion4<<linkType5<<"="<<motion5<<endl;
    //                            }else{
    //                                qDebug()<<"Collision pairs at"<<linkType1<<"="<<motion1<<linkType2<<"="<<motion2<<linkType3<<"="<<motion3
    //                                       <<linkType4<<"="<<motion4<<linkType5<<"="<<motion5<<":"<<totalCollisionSet<<endl;
    //                            }
    //                        }
    //                    }
    //                }
    //            }
    //        }


    //    QSet<QString> totalCollisionSet = voxelizer.translateVoxelModel(MT, linkType5, motion5, 0, 0 * samplingNumber + 0);
    //    if(totalCollisionSet.empty()){
    //        qDebug()<<"No collision at"<<linkType1<<"="<<motion1<<linkType2<<"="<<motion2<<linkType3<<"="<<motion3
    //               <<linkType4<<"="<<motion4<<linkType5<<"="<<motion5<<endl;
    //    }else{
    //        qDebug()<<"Collision pairs at"<<linkType1<<"="<<motion1<<linkType2<<"="<<motion2<<linkType3<<"="<<motion3
    //               <<linkType4<<"="<<motion4<<linkType5<<"="<<motion5<<":"<<totalCollisionSet<<endl;
    //    }


    //    //timer
    //    QElapsedTimer timer1;
    //    timer1.start();
    //    //    QSet<QString> totalCollisionSet = voxelizer.translateVoxelModel(MT, 'Z', -0.2f, 0,0);
    //    QSet<QString> totalCollisionSet = voxelizer.translateVoxelModel(MT, 'Y', 0.52f, 0,0);

    //    qDebug() << "Translate voxels took" << timer1.elapsed() << "milliseconds"<<endl;

    //    if(totalCollisionSet.empty()){
    //        qDebug()<<"No collision occurs"<<endl;
    //    }else{
    //        qDebug()<<"Collision pairs for this configuration"<<totalCollisionSet<<endl;
    //    }

    // check if visualization is necessary     ===========================================================
    if(ifNeedVisualization){
        for (QVector<Link>::iterator loop = MT.LinkVector.begin();loop != MT.LinkVector.end(); loop++){

            //timer
            QElapsedTimer timer;
            timer.start();
            drawVoxelforCollision(*loop);
            qDebug() << "The creation of cubes for"<<loop->getLinkType()<< "took" << timer.elapsed() << "milliseconds"<<endl;
        }
    }
    //check if visualization is necessary     =======================================================
}

void CreateCubes::drawVoxelforCollision(Link& link)
{
    float voxelSpace_X_min = voxelizer.voxelStarting_X;
    float voxelSpace_Y_min = voxelizer.voxelStarting_Y;
    float voxelSpace_Z_min = voxelizer.voxelStarting_Z;

    for (QList<QVector3D>::iterator i = link.MTCollidedVoxelIndicesList.begin();
         i != link.MTCollidedVoxelIndicesList.end(); ++i){
        int number_x = i->x();
        int number_y = i->y();
        int number_z = i->z();

        GLfloat offset_y = voxelSize * number_y;
        GLfloat offset_x = voxelSize * number_x;
        GLfloat offset_z = voxelSize * number_z;
        GLfloat x_right = voxelSpace_X_min + voxelSize + offset_x;
        GLfloat x_left = voxelSpace_X_min + offset_x;
        GLfloat y_up = voxelSpace_Y_min + voxelSize + offset_y;
        GLfloat y_down = voxelSpace_Y_min + offset_y;
        GLfloat z_futher = voxelSpace_Z_min + offset_z;
        GLfloat z_closer = voxelSpace_Z_min + voxelSize + offset_z;




        //coordinates of triangles forming faces of cubes
        //6 faces for each cube, 4 vertices for each face, 3 coordinates for each vertex
        GLfloat coords[6][4][3] = {
            { { x_right, y_down, z_futher}, { x_left, y_down, z_futher}, { x_left, y_up, z_futher}, { x_right, y_up, z_futher} },
            { { x_right, y_up, z_futher}, { x_left, y_up, z_futher}, { x_left, y_up, z_closer}, { x_right, y_up, z_closer} },
            { { x_right, y_down, z_closer}, { x_right, y_down, z_futher}, { x_right, y_up, z_futher}, { x_right, y_up, z_closer} },
            { { x_left,y_down, z_futher}, { x_left, y_down, z_closer}, { x_left, y_up, z_closer}, { x_left, y_up, z_futher} },
            { { x_right,y_down, z_closer}, { x_left, y_down, z_closer}, { x_left, y_down, z_futher}, { x_right, y_down, z_futher} },
            { { x_left, y_down, z_closer}, { x_right, y_down, z_closer}, { x_right, y_up, z_closer}, { x_left, y_up, z_closer} }
        };

        QVector3D normal;
        bool ifDuplicate = false;

        // Draw6 different normal direction and avoid overlapping face for each face
        for (int i = 0; i < 6; ++i) {
            normal = setNormal(i);

            ifDuplicate = checkDuplicateFaceforCollision(i, number_x, number_y, number_z);
            if(ifDuplicate)
                continue;

            // insert vertex position into m_data for creating VBO

            //first triangle in the face
            m_data.resize(m_totalCount+36);
            GLfloat *p = m_data.data() + m_totalCount;
            *p++ = coords[i][0][0];
            *p++ = coords[i][0][1];
            *p++ = coords[i][0][2];
            *p++ = normal[0];
            *p++ = normal[1];
            *p++ = normal[2];
            //---------------------------------
            *p++ = coords[i][1][0];
            *p++ = coords[i][1][1];
            *p++ = coords[i][1][2];
            *p++ = normal[0];
            *p++ = normal[1];
            *p++ = normal[2];
            //---------------------------------
            *p++ = coords[i][2][0];
            *p++ = coords[i][2][1];
            *p++ = coords[i][2][2];
            *p++ = normal[0];
            *p++ = normal[1];
            *p++ = normal[2];

            /////////////////////////////////////////////
            //second triangle in the face
            *p++ = coords[i][0][0];
            *p++ = coords[i][0][1];
            *p++ = coords[i][0][2];
            *p++ = normal[0];
            *p++ = normal[1];
            *p++ = normal[2];
            //---------------------------------
            *p++ = coords[i][2][0];
            *p++ = coords[i][2][1];
            *p++ = coords[i][2][2];
            *p++ = normal[0];
            *p++ = normal[1];
            *p++ = normal[2];
            //---------------------------------
            *p++ = coords[i][3][0];
            *p++ = coords[i][3][1];
            *p++ = coords[i][3][2];
            *p++ = normal[0];
            *p++ = normal[1];
            *p++ = normal[2];
            m_totalCount += 36;

            //update number of vertices
            link.numberOfVertex += 6;
        }
    }
}

bool CreateCubes::checkDuplicateFace(int i, int number_x, int number_y, int number_z)
{
    // number_z - 1 has overlapping face
    if (i==0 && number_z > 0){
        if(voxelizer.voxelspace[number_x][number_y][number_z - 1].getVoxelLinkType() != 'E'){
            return true;
        }
    }
    // number_y + 1 has overlapping face
    else if (i==1 && number_y < voxelizer.voxelSpaceSize_Y - 1){
        if(voxelizer.voxelspace[number_x][number_y + 1][number_z].getVoxelLinkType()!= 'E'){
            return true;
        }
    }
    // number_x + 1 has overlapping face
    else if (i==2 && number_x < voxelizer.voxelSpaceSize_X - 1){
        if(voxelizer.voxelspace[number_x + 1][number_y][number_z].getVoxelLinkType()!= 'E'){
            return true;
        }
    }
    // number_x - 1 has overlapping face
    else if (i==3 && number_x > 0){
        if(voxelizer.voxelspace[number_x - 1][number_y][number_z].getVoxelLinkType()!= 'E'){
            return true;
        }
    }
    // number_y - 1 has overlapping face
    else if (i==4 && number_y > 0){
        if(voxelizer.voxelspace[number_x][number_y - 1][number_z].getVoxelLinkType()!= 'E'){
            return true;
        }
    }
    // number_z + 1 has overlapping face
    else if (i==5 && number_z < voxelizer.voxelSpaceSize_Z - 1){
        if(voxelizer.voxelspace[number_x][number_y][number_z + 1].getVoxelLinkType()!= 'E'){
            return true;
        }
    }
    //if no overlapping face
    return false;
}

bool CreateCubes::checkDuplicateFaceforCollision(int i, int number_x, int number_y, int number_z)
{
    // number_z - 1 has overlapping face
    if (i==0 && number_z > 0){
        if(voxelizer.voxelspace[number_x][number_y][number_z - 1].isCollide()){
            return true;
        }
    }
    // number_y + 1 has overlapping face
    else if (i==1 && number_y < voxelizer.voxelSpaceSize_Y - 1){
        if(voxelizer.voxelspace[number_x][number_y + 1][number_z].isCollide()){
            return true;
        }
    }
    // number_x + 1 has overlapping face
    else if (i==2 && number_x < voxelizer.voxelSpaceSize_Y - 1){
        if(voxelizer.voxelspace[number_x + 1][number_y][number_z].isCollide()){
            return true;
        }
    }
    // number_x - 1 has overlapping face
    else if (i==3 && number_x > 0){
        if(voxelizer.voxelspace[number_x - 1][number_y][number_z].isCollide()){
            return true;
        }
    }
    // number_y - 1 has overlapping face
    else if (i==4 && number_y > 0){
        if(voxelizer.voxelspace[number_x][number_y - 1][number_z].isCollide()){
            return true;
        }
    }
    // number_z + 1 has overlapping face
    else if (i==5 && number_z < voxelizer.voxelSpaceSize_Z - 1){
        if(voxelizer.voxelspace[number_x][number_y][number_z + 1].isCollide()){
            return true;
        }
    }
    //if no overlapping face
    return false;
}


QVector3D CreateCubes::setNormal(int i)
{
    if (i==0)
        return { 0.0,0.0,-1.0};
    else if (i==1)
        return { 0.0,1.0,0.0};
    else if (i==2)
        return { 1.0,0.0,0.0};
    else if (i==3)
        return { -1.0,0.0,0.0};
    else if (i==4)
        return { 0.0,-1.0,0.0};
    else
        return { 0.0,0.0,1.0};
}
