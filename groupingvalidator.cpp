#include "groupingvalidator.h"

GroupingValidator::GroupingValidator()
{ 
}

void GroupingValidator::createMTVoxelspace(float vSize, QVector<component> &componentVector)
{
    //setup voxelsize
    voxelSize = vSize;

    //initialize voxel space and voxel size and return the name of the lowest component
    QString lowestCompName = voxelizer.setupSize(voxelSize, componentVector);
}

void GroupingValidator::collisionDetectionForConfigurations(MachineTool &MT, bool needVisualization)
{
    //create voxel space to check collisoin for all configurations
    voxelizer.createVoxelSapce();

    //setup transformation matrix for each component
    setupInitialTransformation(MT);

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

    //    int samplingNumber = 1;

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
    //    voxelizer.shiftVoxelModel(MT, 0.0 * 1, 0.0 *1, 0.0 *1);
    //    QSet<QString> totalCollisionSet;
    //    totalCollisionSet +=
    //            voxelizer.collisionDetectionForGroups(MT, 0, 0 * samplingNumber + 0);

    //    if(totalCollisionSet.empty()){
    //        qDebug()<<"No collision occurs at X shift:"<<0<<" Y shift:"<<0
    //               <<" Z shift:"<<0<<MT.firstRotaryLink->getLinkType()<<"rotayte:"<<0<<
    //                 MT.secondRotaryLink->getLinkType()<<"rotate:"<<0<<endl;
    //    }else{
    //        qDebug()<<"Collision pairs at X shift:"<<0<<" Y shift:"<<0
    //               <<" Z shift:"<<0<<MT.firstRotaryLink->getLinkType()<<"rotayte:"<<0<<
    //                 MT.secondRotaryLink->getLinkType()<<"rotate:"<<0<<":"
    //              <<totalCollisionSet<<endl;
    //    }
    //    totalCollisionSet.clear();

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

void GroupingValidator::setupInitialTransformation(MachineTool &MT)
{
    //setup transformation matrix for each component  (X,Y,Z,A,B,C)
    //translational unit: meter
    //rotary unit: degree
    voxelizer.setupInitialTransformationMatrix(MT, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void GroupingValidator::drawVoxelforMT(Link &link, int ind1, int ind2)
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
            // Draw6 different normal direction and avoid overlapping face for each face
            for (int i = 0; i < 6; ++i) {
                normal = setNormal(i);

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

QVector3D GroupingValidator::setNormal(int i)
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