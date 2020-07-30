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

void CreateCubes::setupTransformation(MachineTool &MT, char linkType, float amount)
{
    voxelizer.setTransformationMatrix(MT, linkType, amount);
}

void CreateCubes::createMTVoxelspace(float spaceLength, float vSize, MachineTool& MT, bool needVisualization)
{
    //initialize if visualization is necessary
    ifNeedVisualization = needVisualization;

    //setup voxelsize
    voxelSize = vSize;

    // (space length should be xx times of voxel size)
    Q_ASSERT_X(fmod(spaceLength,voxelSize) == 0.0f, "createVoxelspace", "spaceLength % voxelSize should be zero");

    //  make zero point in the center of voxel
    if(static_cast<int>(spaceLength/vSize) %2 == 0)
        spaceLength += vSize;

    //initialize voxel space and voxel size
    voxelizer.setupSize(spaceLength, voxelSize);

    //setup transformation matrix for each component
    setupInitialTransformation(MT);

    //translational unit: meter
    //rotary unit: degree
    //    voxelizer.setTransformationMatrix(MT, 'Y', 0.6f);

    n_voxel_in_axis = static_cast<int>(spaceLength / voxelSize);
    mostLeftBottom = -spaceLength/2.0f;

    for (QVector<Link>::iterator loop = MT.LinkVector.begin();loop != MT.LinkVector.end(); loop++){
        voxelizer.Voxelize(*loop, true);

        //reset bounding index for next component
        voxelizer.reset_bounding_index();
    }


    //    // create parent voxel models  ----------------------------------------------------------------------

    //    //set joint limit

    //    for (QVector<Joint>::iterator loop = MT.JointVector.begin(); loop != MT.JointVector.end(); loop++){
    //        char linkType = loop->getChildLink()->getLinkType();

    //        switch (linkType) {
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
    //            loop->setLowerLimit(0.0f);
    //            loop->setUpperLimit(90.0f);
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
    //            voxelizer.Voxelize(*baseLink, true);

    //            //reset bounding index for next component
    //            voxelizer.reset_bounding_index();

    //            break;
    //        }
    //    }

    //    // only works for 50 configuration type
    //    Link* currentLink = baseLink->ChildLink;

    //    while(currentLink->ChildLink != nullptr){
    //        char linkType = currentLink->getLinkType();

    //        if(linkType == 'A' |linkType == 'B' | linkType == 'C'){
    //            int samplingNumber = 3;

    //            for(int i = 0; i < samplingNumber; ++i){
    //                char linkType1 = currentLink->getLinkType();
    //                float lowerLimit1 = currentLink->getLowerLimit();
    //                float upperLimit1 = currentLink->getUpperLimit();
    //                float motionRange1 = (upperLimit1 - lowerLimit1)/samplingNumber * i + lowerLimit1;

    //                //translational unit: meter
    //                //rotary unit: degree
    //                voxelizer.setTransformationMatrix(MT, linkType1,motionRange1);
    //                voxelizer.Voxelize(*currentLink, true);

    //                //reset bounding index for next component
    //                voxelizer.reset_bounding_index();

    //                currentLink = currentLink->ChildLink;

    //                for(int j = 0; j < samplingNumber; ++j){
    //                    char linkType2 = currentLink->getLinkType();
    //                    float lowerLimit2 = currentLink->getLowerLimit();
    //                    float upperLimit2 = currentLink->getUpperLimit();
    //                    float motionamount2 = (upperLimit2 - lowerLimit2)/samplingNumber * j + lowerLimit2;

    //                    //translational unit: meter
    //                    //rotary unit: degree
    //                    voxelizer.setTransformationMatrix(MT, linkType2, motionamount2);
    //                    voxelizer.Voxelize(*currentLink, true);

    //                    //reset bounding index for next component
    //                    voxelizer.reset_bounding_index();

    //                    if(j == samplingNumber-1)
    //                        currentLink = currentLink->ParentLink;
    //                }
    //                if(i == samplingNumber-1)
    //                    break;
    //            }
    //            break;

    //        }else{
    //            voxelizer.Voxelize(*currentLink, true);

    //            //reset bounding index for next component
    //            voxelizer.reset_bounding_index();
    //            currentLink = currentLink->ChildLink;
    //        }
    //    }


    // create parent voxel models  ----------------------------------------------------------------------

    //timer
    QElapsedTimer timer1;
    timer1.start();
    //    voxelizer.translateVoxelModel(MT, 'Z', -0.1f, 0,0);
    voxelizer.translateVoxelModel(MT, 'Y', 0.135f, 0,0);
    //    voxelizer.translateVoxelModel(MT, 'Z', -0.1f, 1, 4);
    //    voxelizer.translateVoxelModel(MT, 'X', 0.1f, 1, 4);

    qDebug() << "Translate voxels took" << timer1.elapsed() << "milliseconds"<<endl;

    // check if visualization is necessary     ===========================================================
    if(ifNeedVisualization){
        for (QVector<Link>::iterator loop = MT.LinkVector.begin();loop != MT.LinkVector.end(); loop++){

            //timer
            QElapsedTimer timer;
            timer.start();
            drawVoxelforMT(*loop,1,4);
            qDebug() << "The creation of cubes for"<<loop->getLinkType()<< "took" << timer.elapsed() << "milliseconds"<<endl;
        }
    }
}



void CreateCubes::drawVoxelforMT(Link& link, int ind1, int ind2)
{
    char linkType = link.getLinkType();

    //    loop through voxel space to plot occupied voxels (only loop through the bounding voxel space from voxelizer)
    //        for (int number_x = link.get_x_min_index(); number_x < link.get_x_max_index() + 1; ++number_x) {
    //            for (int number_y = link.get_y_min_index(); number_y < link.get_y_max_index() + 1; ++number_y) {
    //                for (int number_z = link.get_z_min_index(); number_z < link.get_z_max_index() + 1; ++number_z) {

    for (QList<QVector3D>::iterator i = link.MTInnerVoxelIndicesList.begin(); i != link.MTInnerVoxelIndicesList.end(); ++i){
        int number_x = i->x();
        int number_y = i->y();
        int number_z = i->z();
        int index = 0;

        if(link.getLinkType() == 'C')
            index = ind1;

        if(link.getLinkType() == 'A')
            index = ind2;

        //    for (QList<QVector3D>::iterator i = link.MTOutterVoxelIndicesListVector[index].begin();
        //         i != link.MTOutterVoxelIndicesListVector[index].end(); ++i){
        //        int number_x = i->x();
        //        int number_y = i->y();
        //        int number_z = i->z();

        //        if voxel is empty, jump to next iteration
        //                if(link.linkVoxelspace[number_x][number_y][number_z].getOutterShellLinkType() != linkType)
        //                continue;


        //        if(link.linkVoxelspace[number_x][number_y][number_z].getInnerShellLinkType() != linkType)
        //            continue;

        GLfloat offset_y = voxelSize * number_y;
        GLfloat offset_x = voxelSize * number_x;
        GLfloat offset_z = voxelSize * number_z;
        GLfloat x_right = mostLeftBottom + voxelSize + offset_x;
        GLfloat x_left = mostLeftBottom + offset_x;
        GLfloat y_up = mostLeftBottom + voxelSize + offset_y;
        GLfloat y_down = mostLeftBottom + offset_y;
        GLfloat z_futher = mostLeftBottom + offset_z;
        GLfloat z_closer = mostLeftBottom + voxelSize + offset_z;

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

            //            ifDuplicate = checkDuplicateFace(i, number_x, number_y, number_z, link);
            //            if(ifDuplicate)
            //                continue;

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
//    }
//}


void CreateCubes::createCollisionVoxelspace(float spaceLength, float vSize, MachineTool& MT ,bool needVisualization)
{
    //initialize if visualization is necessary
    ifNeedVisualization = needVisualization;

    //setup voxelsize
    voxelSize = vSize;

    //initialize voxel space and voxel size (space length should be xx times of voxel size)
    Q_ASSERT_X(fmod(spaceLength,voxelSize) == 0.0f, "createVoxelspace", "spaceLength % voxelSize should be zero");
    voxelizer.setupSize(spaceLength, voxelSize);

    //setup transformation matrix for each component
    setupInitialTransformation(MT);

    //translational unit: meter
    //rotary unit: degree
    //    voxelizer.setTransformationMatrix(MT, 'Y', 0.6f);

    n_voxel_in_axis = static_cast<int>(spaceLength / voxelSize);
    mostLeftBottom = -spaceLength/2.0f;


    for (QVector<Link>::iterator loop = MT.LinkVector.begin();loop != MT.LinkVector.end(); loop++){
        voxelizer.Voxelize(*loop, true);

        //reset bounding index for next component
        voxelizer.reset_bounding_index();
    }

    // create parent voxel models  ----------------------------------------------------------------------

    //set joint limit

    //    for (QVector<Joint>::iterator loop = MT.JointVector.begin(); loop != MT.JointVector.end(); loop++){
    //        char linkType = loop->getChildLink()->getLinkType();

    //        switch (linkType) {
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
    //            loop->setLowerLimit(0.0f);
    //            loop->setUpperLimit(90.0f);
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
    //            voxelizer.Voxelize(*baseLink, true);

    //            //reset bounding index for next component
    //            voxelizer.reset_bounding_index();

    //            break;
    //        }
    //    }

    //    // only works for 50 configuration type
    //    Link* currentLink = baseLink->ChildLink;
    //    int samplingNumber = 3;

    //    while(currentLink->ChildLink != nullptr){
    //        char linkType = currentLink->getLinkType();

    //        if(linkType == 'A' |linkType == 'B' | linkType == 'C'){


    //            for(int i = 0; i < samplingNumber; ++i){
    //                char linkType1 = currentLink->getLinkType();
    //                float lowerLimit1 = currentLink->getLowerLimit();
    //                float upperLimit1 = currentLink->getUpperLimit();
    //                float motionRange1 = (upperLimit1 - lowerLimit1)/samplingNumber * i + lowerLimit1;

    //                //translational unit: meter
    //                //rotary unit: degree
    //                voxelizer.setTransformationMatrix(MT, linkType1,motionRange1);
    //                voxelizer.Voxelize(*currentLink, true);

    //                //reset bounding index for next component
    //                voxelizer.reset_bounding_index();

    //                currentLink = currentLink->ChildLink;

    //                for(int j = 0; j < samplingNumber; ++j){
    //                    char linkType2 = currentLink->getLinkType();
    //                    float lowerLimit2 = currentLink->getLowerLimit();
    //                    float upperLimit2 = currentLink->getUpperLimit();
    //                    float motionamount2 = (upperLimit2 - lowerLimit2)/samplingNumber * j + lowerLimit2;

    //                    //translational unit: meter
    //                    //rotary unit: degree
    //                    voxelizer.setTransformationMatrix(MT, linkType2, motionamount2);
    //                    voxelizer.Voxelize(*currentLink, true);

    //                    //reset bounding index for next component
    //                    voxelizer.reset_bounding_index();

    //                    if(j == samplingNumber-1)
    //                        currentLink = currentLink->ParentLink;
    //                }
    //                if(i == samplingNumber-1)
    //                    break;
    //            }
    //            break;

    //        }else{
    //            voxelizer.Voxelize(*currentLink, true);

    //            //reset bounding index for next component
    //            voxelizer.reset_bounding_index();
    //            currentLink = currentLink->ChildLink;
    //        }
    //    }


    //    // create parent voxel models  ----------------------------------------------------------------------

    //    Link* link1 = baseLink;

    //    for (int ind_1 = 1; ind_1 < samplingNumber + 1; ind_1++ ){
    //        float motion1 = link1->getLowerLimit() +
    //                (link1->getUpperLimit() - link1->getLowerLimit()) * ind_1 / samplingNumber;
    //        voxelizer.translateVoxelModel(MT, baseLink->getLinkType(), motion1, 0, 0);
    //        Link* link2 = link1->ChildLink;

    //        for (int ind_2 = 1; ind_2 < samplingNumber + 1; ind_2++ ){
    //            float motion2 = link2->getLowerLimit() +
    //                    (link2->getUpperLimit() - link2->getLowerLimit()) * ind_2 / samplingNumber;
    //            voxelizer.translateVoxelModel(MT, baseLink->getLinkType(), motion2, 0, 0);
    //            Link* link3 = link2->ChildLink;

    //            for (int ind_3 = 1; ind_3 < samplingNumber + 1; ind_3++ ){
    //                float motion3 = link3->getLowerLimit() +
    //                        (link3->getUpperLimit() - link3->getLowerLimit()) * ind_3 / samplingNumber;
    //                voxelizer.translateVoxelModel(MT, baseLink->getLinkType(), motion3, 0, 0);
    //                Link* link4  = link3->ChildLink;

    //                for (int ind_4 = 0; ind_4<samplingNumber + 1; ind_4++ ){
    //                    float motion4 = link4->getLowerLimit() +
    //                            (link4->getUpperLimit() - link4->getLowerLimit()) * ind_4 / samplingNumber;
    //                    voxelizer.translateVoxelModel(MT, baseLink->getLinkType(), motion4, ind_4, 0);
    //                    Link* link5 = link4->ChildLink;

    //                    for (int ind_5 = 1; ind_5<samplingNumber + 1; ind_5++ ){
    //                        float motion5 = link5->getLowerLimit() +
    //                                (link5->getUpperLimit() - link5->getLowerLimit()) * ind_5 / samplingNumber;
    //                        voxelizer.translateVoxelModel(MT, baseLink->getLinkType(), motion5, ind_4, ind_4 * samplingNumber + ind_5);
    //                    }
    //                }
    //            }
    //        }
    //    }


    //timer
    QElapsedTimer timer1;
    timer1.start();
    //    voxelizer.translateVoxelModel(MT, 'Z', -0.1f, 0,0);
    QSet<QString> totalCollisionSet = voxelizer.translateVoxelModel(MT, 'Y', 0.135f, 0,0);

    qDebug()<<"Collision pairs for this configuration"<<totalCollisionSet<<endl;


    qDebug() << "Translate voxels took" << timer1.elapsed() << "milliseconds"<<endl;

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
    //loop through voxel space to plot occupied voxels (only loop through the bounding voxel space from voxelizer)
    for (int number_x = link.get_x_min_index(); number_x < link.get_x_max_index() +1; ++number_x) {
        for (int number_y = link.get_y_min_index(); number_y < link.get_y_max_index()+1; ++number_y) {
            for (int number_z = link.get_z_min_index(); number_z < link.get_z_max_index()+1; ++number_z) {

                //if voxel is empty, jump to next iteration
                if(!link.linkVoxelspace[number_x][number_y][number_z].isCollide())
                    continue;

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
    }
}


bool CreateCubes::checkDuplicateFace(int i, int number_x, int number_y, int number_z, Link& link)
{
    QVector < QVector < QVector< Voxel > > > &linkVS = link.linkVoxelspace;

    // number_z - 1 has overlapping face
    if (i==0 && number_z > 0){
        if(linkVS[number_x][number_y][number_z - 1].getOutterShellLinkType() != 'E'){
            return true;
        }
    }
    // number_y + 1 has overlapping face
    else if (i==1 && number_y < n_voxel_in_axis - 1){
        if(linkVS[number_x][number_y + 1][number_z].getOutterShellLinkType()!= 'E'){
            return true;
        }
    }
    // number_x + 1 has overlapping face
    else if (i==2 && number_x < n_voxel_in_axis - 1){
        if(linkVS[number_x + 1][number_y][number_z].getOutterShellLinkType()!= 'E'){
            return true;
        }
    }
    // number_x - 1 has overlapping face
    else if (i==3 && number_x > 0){
        if(linkVS[number_x - 1][number_y][number_z].getOutterShellLinkType()!= 'E'){
            return true;
        }
    }
    // number_y - 1 has overlapping face
    else if (i==4 && number_y > 0){
        if(linkVS[number_x][number_y - 1][number_z].getOutterShellLinkType()!= 'E'){
            return true;
        }
    }
    // number_z + 1 has overlapping face
    else if (i==5 && number_z < n_voxel_in_axis - 1){
        if(linkVS[number_x][number_y][number_z + 1].getOutterShellLinkType()!= 'E'){
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
    else if (i==1 && number_y < n_voxel_in_axis - 1){
        if(voxelizer.voxelspace[number_x][number_y + 1][number_z].isCollide()){
            return true;
        }
    }
    // number_x + 1 has overlapping face
    else if (i==2 && number_x < n_voxel_in_axis - 1){
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
    else if (i==5 && number_z < n_voxel_in_axis - 1){
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
