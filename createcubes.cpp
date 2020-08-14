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

void CreateCubes::createMTVoxelspace(float vSize, MachineTool& MT, bool needVisualization)
{
    //initialize if visualization is necessary
    ifNeedVisualization = needVisualization;

    //setup voxelsize
    voxelSize = vSize;

    //    //  make zero point in the center of voxel
    //    if(static_cast<int>(spaceLength/vSize) %2 == 0)
    //        spaceLength += vSize;

    //initialize voxel space and voxel size
    voxelizer.setupSize(voxelSize, MT);

    //setup transformation matrix for each component
    setupInitialTransformation(MT);

    //translational unit: meter
    //rotary unit: degree
    //    voxelizer.setTransformationMatrix(MT, 'Y', 0.6f);

    voxelSpace_X_min = voxelizer.voxelStarting_X;
    voxelSpace_Y_min = voxelizer.voxelStarting_Y;
    voxelSpace_Z_min = voxelizer.voxelStarting_Z;


    // create voxel models for every link------------------------------------------------

    //        for (QVector<Link>::iterator loop = MT.LinkVector.begin();loop != MT.LinkVector.end(); loop++){
    //            voxelizer.Voxelize(*loop);
    //        }
    // create voxel models for every link------------------------------------------------


    // create parent voxel models  ----------------------------------------------------------------------

    //set joint limit

    for (QVector<Joint>::iterator loop = MT.JointVector.begin(); loop != MT.JointVector.end(); loop++){
        QChar linkType = loop->getChildLink()->getLinkType();

        switch (linkType.toLatin1()) {
        case 'X':
            loop->setLowerLimit(-0.1f);
            loop->setUpperLimit(0.1f);
            break;
        case 'Y':
            loop->setLowerLimit(-0.1f);
            loop->setUpperLimit(0.1f);
            break;
        case 'Z':
            loop->setLowerLimit(-0.1f);
            loop->setUpperLimit(0.1f);
            break;
        case 'A':
            loop->setLowerLimit(-45.0f);
            loop->setUpperLimit(45.0f);
            break;
        case 'B':
            loop->setLowerLimit(-45.0f);
            loop->setUpperLimit(45.0f);
            break;
        case 'C':
            loop->setLowerLimit(-45.0f);
            loop->setUpperLimit(45.0f);
            break;
        }
    }

    qDebug()<<endl<<"Start creating parent voxel models ----------------------------------------------"<<endl<<endl;

    //find and voxelize base link
    Link* baseLink = MT.baseLink;
    voxelizer.Voxelize(*baseLink);

    //timer
    QElapsedTimer parentModelstimer;
    parentModelstimer.start();

    int samplingNumber = 2;

    for(int Number = 0; Number < baseLink->ChildLink.size(); Number++){
        Link* currentLink = baseLink->ChildLink[Number];

        while(currentLink != nullptr){

            if(currentLink->isRotaitonal){
                // voxelize rotary links

                for(int i = 0; i < samplingNumber; ++i){
                    QChar linkType1 = currentLink->getLinkType();
                    float lowerLimit1 = currentLink->getLowerLimit();
                    float upperLimit1 = currentLink->getUpperLimit();
                    float motionRange1 = (upperLimit1 - lowerLimit1)/(samplingNumber - 1)* i + lowerLimit1;

                    //translational unit: meter
                    //rotary unit: degree
                    voxelizer.setTransformationMatrix(MT, linkType1,motionRange1);
                    voxelizer.Voxelize(*currentLink);

                    if(!currentLink->ChildLink.isEmpty()){
                        currentLink = currentLink->ChildLink[0];
                        for(int j = 0; j < samplingNumber; ++j){
                            QChar linkType2 = currentLink->getLinkType();
                            float lowerLimit2 = currentLink->getLowerLimit();
                            float upperLimit2 = currentLink->getUpperLimit();
                            float motionamount2 = (upperLimit2 - lowerLimit2)/(samplingNumber - 1) * j + lowerLimit2;

                            //translational unit: meter
                            //rotary unit: degree
                            voxelizer.setTransformationMatrix(MT, linkType2, motionamount2);
                            voxelizer.Voxelize(*currentLink);

                            if(j == samplingNumber-1)
                                currentLink = currentLink->ParentLink;
                        }
                    }
                    if(i == samplingNumber-1)
                        break;
                }
                break;
            }else{
                // voxelize translational links
                voxelizer.Voxelize(*currentLink);

                if(!currentLink->ChildLink.isEmpty()){
                    currentLink = currentLink->ChildLink[0];
                }else{
                    break;
                }
            }
        }
    }

    qDebug() << "Creating parent models took"<< parentModelstimer.elapsed() << "milliseconds"<<endl;
    qDebug()<<endl<<"Finish creating parent voxel models ------------------------------------"<<endl<<endl;

    // create parent voxel models  ---------------------------------------------------------------------

//        QSet<QString> totalCollisionSet;

//        //timer
//        QElapsedTimer collisionDetectiontimer;
//        collisionDetectiontimer.start();

//        for(int samplingX = 0; samplingX < samplingNumber; samplingX++){
//            for(int samplingY = 0; samplingY < samplingNumber; samplingY++){
//                for(int samplingZ = 0; samplingZ < samplingNumber; samplingZ++){

//                    voxelizer.shiftVoxelModel(MT, 0.05 * samplingX, 0.03 *samplingY, -0.05 *samplingZ);

//                    for(int firstRotarySamplingNumber = 0; firstRotarySamplingNumber < samplingNumber; firstRotarySamplingNumber++){
//                        for(int secondRotarySamplingNumber = 0; secondRotarySamplingNumber < samplingNumber; secondRotarySamplingNumber++){
//                            totalCollisionSet +=
//                                    voxelizer.collisionDetection(MT, firstRotarySamplingNumber,
//                                                                 firstRotarySamplingNumber * samplingNumber + secondRotarySamplingNumber);

//                            if(totalCollisionSet.empty()){
//                                qDebug()<<"No collision occurs at X:"<<samplingX + 1<<" Y:"<<samplingY + 1
//                                       <<" Z:"<<samplingZ + 1<<" C:"<<firstRotarySamplingNumber + 1<<
//                                         " A:"<<firstRotarySamplingNumber * samplingNumber + secondRotarySamplingNumber + 1<<endl;
//                            }else{
//                                qDebug()<<"Collision pairs at X:"<<samplingX + 1<<" Y:"<<samplingY + 1
//                                       <<" Z:"<<samplingZ + 1<<" C:"<<firstRotarySamplingNumber + 1<<
//                                         " A:"<<firstRotarySamplingNumber * samplingNumber + secondRotarySamplingNumber + 1<<":"
//                                      <<totalCollisionSet<<endl;
//                            }
//                            totalCollisionSet.clear();
//                        }
//                    }
//                }
//            }
//        }
//        qDebug() << "Collision detection with"<<samplingNumber<<"sampling point for each axis took"
//                 << collisionDetectiontimer.elapsed() << "milliseconds"<<endl;

//    voxelizer.shiftVoxelModel(MT, 0.3 * 1, 0.3 *1, -0.3 *1);
//    QSet<QString> totalCollisionSet;
//    totalCollisionSet +=
//            voxelizer.collisionDetection(MT, 0,
//                                         0 * samplingNumber + 0);

//    if(totalCollisionSet.empty()){
//        qDebug()<<"No collision occurs at X:"<<1 + 1<<" Y:"<<1 + 1
//               <<" Z:"<<1 + 1<<" C:"<<0 + 1<<
//                 " A:"<<0 * samplingNumber + 0 + 1<<endl;
//    }else{
//        qDebug()<<"Collision pairs at X:"<<1 + 1<<" Y:"<<1 + 1
//               <<" Z:"<<1 + 1<<" C:"<<0 + 1<<
//                 " A:"<<0 * samplingNumber + 2 + 0<<":"
//              <<totalCollisionSet<<endl;
//    }
//    totalCollisionSet.clear();

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
    int index = 0;

    if(link.getLinkType() == 'B')
        index = ind1;

    if(link.getLinkType() == 'C')
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

                //ifDuplicate = checkDuplicateFace(i, number_x, number_y, number_z, link);
                // if(ifDuplicate)
                //     continue;

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


void CreateCubes::createCollisionVoxelspace(float vSize, MachineTool& MT ,bool needVisualization)
{
    //initialize if visualization is necessary
    ifNeedVisualization = needVisualization;

    //setup voxelsize
    voxelSize = vSize;

    voxelizer.setupSize(voxelSize, MT);

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

    for (QVector<Joint>::iterator loop = MT.JointVector.begin(); loop != MT.JointVector.end(); loop++){
        QChar linkType = loop->getChildLink()->getLinkType();

        switch (linkType.toLatin1()) {
        case 'X':
            loop->setLowerLimit(-0.1f);
            loop->setUpperLimit(0.1f);
            break;
        case 'Y':
            loop->setLowerLimit(-0.1f);
            loop->setUpperLimit(0.1f);
            break;
        case 'Z':
            loop->setLowerLimit(-0.1f);
            loop->setUpperLimit(0.1f);
            break;
        case 'A':
            loop->setLowerLimit(-90.0f);
            loop->setUpperLimit(0.0f);
            break;
        case 'C':
            loop->setLowerLimit(0.0f);
            loop->setUpperLimit(90.0f);
            break;
        }
    }

    //find and voxelize base link
    Link* baseLink = nullptr;

    for (QVector<Link>::iterator loop = MT.LinkVector.begin(); loop != MT.LinkVector.end(); loop++){
        if(loop->ParentLink == nullptr)
        {
            baseLink = loop;
            voxelizer.Voxelize(*baseLink);

            break;
        }
    }

    // only works for 50 configuration type
    Link* currentLink = baseLink->ChildLink[0];
    int samplingNumber = 3;
    while(currentLink->ChildLink[0] != nullptr){
        QChar linkType = currentLink->getLinkType();

        if(linkType == 'A' |linkType == 'B' | linkType == 'C'){
            for(int i = 0; i < samplingNumber; ++i){
                QChar linkType1 = currentLink->getLinkType();
                float lowerLimit1 = currentLink->getLowerLimit();
                float upperLimit1 = currentLink->getUpperLimit();
                float motionRange1 = (upperLimit1 - lowerLimit1)/samplingNumber * i + lowerLimit1;

                //translational unit: meter
                //rotary unit: degree
                voxelizer.setTransformationMatrix(MT, linkType1,motionRange1);
                voxelizer.Voxelize(*currentLink);


                currentLink = currentLink->ChildLink[0];

                for(int j = 0; j < samplingNumber; ++j){
                    QChar linkType2 = currentLink->getLinkType();
                    float lowerLimit2 = currentLink->getLowerLimit();
                    float upperLimit2 = currentLink->getUpperLimit();
                    float motionamount2 = (upperLimit2 - lowerLimit2)/samplingNumber * j + lowerLimit2;

                    //translational unit: meter
                    //rotary unit: degree
                    voxelizer.setTransformationMatrix(MT, linkType2, motionamount2);
                    voxelizer.Voxelize(*currentLink);


                    if(j == samplingNumber-1)
                        currentLink = currentLink->ParentLink;
                }
                if(i == samplingNumber-1)
                    break;
            }
            break;

        }else{
            voxelizer.Voxelize(*currentLink);
            currentLink = currentLink->ChildLink[0];
        }
    }

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
    for (QList<QVector3D>::iterator i = link.MTCollidedVoxelIndicesList.begin();
         i != link.MTCollidedVoxelIndicesList.end(); ++i){
        int number_x = i->x();
        int number_y = i->y();
        int number_z = i->z();

        GLfloat offset_y = voxelSize * number_y;
        GLfloat offset_x = voxelSize * number_x;
        GLfloat offset_z = voxelSize * number_z;
        GLfloat x_right = mostLeftBottom + voxelSize/2 + offset_x;
        GLfloat x_left = mostLeftBottom - voxelSize/2 + offset_x;
        GLfloat y_up = mostLeftBottom + voxelSize/2 + offset_y;
        GLfloat y_down = mostLeftBottom - voxelSize/2 + offset_y;
        GLfloat z_futher = mostLeftBottom - voxelSize/2 + offset_z;
        GLfloat z_closer = mostLeftBottom + voxelSize/2 + offset_z;

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

bool CreateCubes::checkDuplicateFace(int i, int number_x, int number_y, int number_z, Link& link)
{
    QVector < QVector < QVector< Voxel > > > &linkVS = link.linkVoxelspace;

    // number_z - 1 has overlapping face
    if (i==0 && number_z > 0){
        if(linkVS[number_x][number_y][number_z - 1].getVoxelLinkType() != 'E'){
            return true;
        }
    }
    // number_y + 1 has overlapping face
    else if (i==1 && number_y < voxelizer.voxelSpaceSize_Y - 1){
        if(linkVS[number_x][number_y + 1][number_z].getVoxelLinkType()!= 'E'){
            return true;
        }
    }
    // number_x + 1 has overlapping face
    else if (i==2 && number_x < voxelizer.voxelSpaceSize_X - 1){
        if(linkVS[number_x + 1][number_y][number_z].getVoxelLinkType()!= 'E'){
            return true;
        }
    }
    // number_x - 1 has overlapping face
    else if (i==3 && number_x > 0){
        if(linkVS[number_x - 1][number_y][number_z].getVoxelLinkType()!= 'E'){
            return true;
        }
    }
    // number_y - 1 has overlapping face
    else if (i==4 && number_y > 0){
        if(linkVS[number_x][number_y - 1][number_z].getVoxelLinkType()!= 'E'){
            return true;
        }
    }
    // number_z + 1 has overlapping face
    else if (i==5 && number_z < voxelizer.voxelSpaceSize_Z - 1){
        if(linkVS[number_x][number_y][number_z + 1].getVoxelLinkType()!= 'E'){
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
