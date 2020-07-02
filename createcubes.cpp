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

    //make zero point in the center of voxel
    if(static_cast<int>(spaceLength/vSize) %2 == 1)
        spaceLength += vSize;

    //initialize voxel space and voxel size
    voxelizer.setupSize(spaceLength, voxelSize);

    //setup transformation matrix for each component
    setupInitialTransformation(MT);

    //        setupTransformation(MT, 'Y', -0.7f);

    n_voxel_in_axis = static_cast<int>(spaceLength / voxelSize);
    mostLeftBottom = -spaceLength/2.0f;

    for (QVector<Link>::iterator loop = MT.LinkVector.begin();loop != MT.LinkVector.end(); loop++){
        voxelizer.VoxelizeForMT(*loop, true);

        m_totalVoxelCount += loop->MTVoxelIndicesList.size();

        //reset bounding index for next component
        voxelizer.reset_bounding_index();
    }

    //resize m_data (*216 is the number to draw all voxel face but it's too large, so I
    //only use 80 here to approximate the total number of voxel faces)
    //sometime crash because size 80 is too small
    //    m_data.resize(m_totalVoxelCount * 80);

    //timer
    QElapsedTimer timer1;
    timer1.start();

    //    voxelizer.translateVoxelModel(MT, 'Y', -0.5f);
    voxelizer.translateVoxelModel(MT, 'X', 0.6f);

    qDebug() << "Translate voxels took" << timer1.elapsed() << "milliseconds"<<endl;

    // check if visualization is necessary     ===========================================================
    if(ifNeedVisualization){
        for (QVector<Link>::iterator loop = MT.LinkVector.begin();loop != MT.LinkVector.end(); loop++){

            //timer
            QElapsedTimer timer;
            timer.start();

            drawVoxelforMT(*loop);

            qDebug() << "The creation of cubes for"<<loop->getLinkType()<< "took" << timer.elapsed() << "milliseconds"<<endl;
        }
    }

    for(int i = 0; i < MT.LinkVector.size(); i++){
        for(int j = i+1; j < MT.LinkVector.size(); j++){
            voxelizer.checkCollision(MT.LinkVector[i], MT.LinkVector[j]);
        }
    }
}

void CreateCubes::drawVoxelforMT(Link& link)
{
    for (int i = 0; i < link.MTVoxelIndicesList.size(); ++i){
        double index = link.MTVoxelIndicesList[i];

        int number_x = int(floor(index));
        int number_y = int(index*10000 - (floor(index))*10000);
        int number_z = int(round((index*10000 - floor(index*10000))*10000));


        GLfloat offset_x = voxelSize * number_x;
        GLfloat offset_y = voxelSize * number_y;
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

            ifDuplicate = checkDuplicateFace(i, number_x, number_y, number_z);
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


void CreateCubes::createCoincidentVoxelspace(float spaceLength, float vSize, MachineTool& MT ,bool needVisualization)
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

    //    setupTransformation(MT, 'B', 30.0f);

    n_voxel_in_axis = static_cast<int>(spaceLength / voxelSize);
    mostLeftBottom = -spaceLength/2.0f;

    for (QVector<Link>::iterator loop = MT.LinkVector.begin();loop != MT.LinkVector.end(); loop++){
        voxelizer.VoxelizeForInterference(*loop, true);

        // check if visualization is necessary     ===========================================================
        if(ifNeedVisualization){

            //timer
            QElapsedTimer timer;
            timer.start();

            drawVoxelforCoincident(*loop);

            //reset bounding index for next component
            voxelizer.reset_bounding_index();

            qDebug() << "The creation of cubes took" << timer.elapsed() << "milliseconds"<<endl;
        }
    }


    for(int i = 0; i < MT.LinkVector.size(); i++){
        for(int j = i+1; j < MT.LinkVector.size(); j++){
            voxelizer.checkCollision(MT.LinkVector[i], MT.LinkVector[j]);
        }
    }
}


void CreateCubes::drawVoxelforCoincident(Link& link)
{
    //loop through voxel space to plot occupied voxels (only loop through the bounding voxel space from voxelizer)
    for (int number_x = link.get_x_min_index(); number_x < (link.get_x_max_index())+1; ++number_x) {
        for (int number_y = link.get_y_min_index(); number_y < link.get_y_max_index()+1; ++number_y) {
            for (int number_z = link.get_z_min_index(); number_z < link.get_z_max_index()+1; ++number_z) {

                //if voxel is empty, jump to next iteration
                if(!link.linkVoxelspace[number_x][number_y][number_z].isCollide())
                    continue;

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

                // Draw 6 different normal direction and avoid overlapping face for each face
                for (int i = 0; i < 6; ++i) {
                    normal = setNormal(i);

                    ifDuplicate = checkDuplicateFaceforCoincident(i, number_x, number_y, number_z);
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


bool CreateCubes::checkDuplicateFace(int i, int number_x, int number_y, int number_z)
{
    // number_z - 1 has overlapping face
    if (i==0 && number_z > 0){
        if(voxelizer.voxelspace[number_x][number_y][number_z - 1].getLinkType() != 'E'){
            return true;
        }
    }
    // number_y + 1 has overlapping face
    else if (i==1 && number_y < n_voxel_in_axis - 1){
        if(voxelizer.voxelspace[number_x][number_y + 1][number_z].getLinkType()!= 'E'){
            return true;
        }
    }
    // number_x + 1 has overlapping face
    else if (i==2 && number_x < n_voxel_in_axis - 1){
        if(voxelizer.voxelspace[number_x + 1][number_y][number_z].getLinkType()!= 'E'){
            return true;
        }
    }
    // number_x - 1 has overlapping face
    else if (i==3 && number_x > 0){
        if(voxelizer.voxelspace[number_x - 1][number_y][number_z].getLinkType()!= 'E'){
            return true;
        }
    }
    // number_y - 1 has overlapping face
    else if (i==4 && number_y > 0){
        if(voxelizer.voxelspace[number_x][number_y - 1][number_z].getLinkType()!= 'E'){
            return true;
        }
    }
    // number_z + 1 has overlapping face
    else if (i==5 && number_z < n_voxel_in_axis - 1){
        if(voxelizer.voxelspace[number_x][number_y][number_z + 1].getLinkType()!= 'E'){
            return true;
        }
    }
    //if no overlapping face
    return false;
}

bool CreateCubes::checkDuplicateFaceforCoincident(int i, int number_x, int number_y, int number_z)
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

