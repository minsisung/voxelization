#include "createcubes.h"
#include <QDebug>
#include<iostream>
#include <QElapsedTimer>

CreateCubes::CreateCubes():
    m_totalCount(0)
{

}

void CreateCubes::createVoxelspace(float spaceLength, float voxelSize,QString filepath1, QString filepath2)
{    
    //initialize voxel space and voxel size
    voxelizer.setupSize(spaceLength, voxelSize);

    try {
        //read STL file
        stl_reader::StlMesh <float, unsigned int> mesh1(filepath1.toStdString());
        voxelizer.Voxelize(mesh1);

        stl_reader::StlMesh <float, unsigned int> mesh2(filepath2.toStdString());
        voxelizer.Voxelize(mesh2);
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }

    n_voxel_in_axis = static_cast<int>(spaceLength / voxelSize);
    float mostLeftBottom = -spaceLength/2.0f;

    QElapsedTimer timer;
    timer.start();

    //loop through voxel space to plot occupied voxels
    for (int number_x = 0; number_x < n_voxel_in_axis; ++number_x) {
        for (int number_y = 0; number_y < n_voxel_in_axis; ++number_y) {
            for (int number_z = 0; number_z < n_voxel_in_axis; ++number_z) {

                //if voxel is empty, jump to next iteration
                if(voxelizer.voxelspace[number_x][number_y][number_z].getStatus()==0)
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

                //6 different normal direction and overlapping face for each face
                for (int i = 0; i < 6; ++i) {

                    ifDuplicate = checkDuplicateFace(i, number_x, number_y, number_z);
                    normal = setNormal(i);

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
                }
            }
        }
    }
    qDebug() << "Total count" << m_totalCount << endl;
    qDebug() << "The creation of cubes took" << timer.elapsed() << "milliseconds"<<endl;
}

bool CreateCubes::checkDuplicateFace(int i, int number_x, int number_y, int number_z)
{
    // number_z - 1 has overlapping face
    if (i==0 && number_z > 0){
        if(voxelizer.voxelspace[number_x][number_y][number_z - 1].getStatus()==1){
            return true;
        }
    }
    // number_y + 1 has overlapping face
    else if (i==1 && number_y < n_voxel_in_axis - 1){
        if(voxelizer.voxelspace[number_x][number_y + 1][number_z].getStatus()==1){
            return true;
        }
    }
    // number_x + 1 has overlapping face
    else if (i==2 && number_x < n_voxel_in_axis - 1){
        if(voxelizer.voxelspace[number_x + 1][number_y][number_z].getStatus()==1){
            return true;
        }
    }
    // number_x - 1 has overlapping face
    else if (i==3 && number_x > 0){
        if(voxelizer.voxelspace[number_x - 1][number_y][number_z].getStatus()==1){
            return true;
        }
    }
    // number_y - 1 has overlapping face
    else if (i==4 && number_y > 0){
        if(voxelizer.voxelspace[number_x][number_y - 1][number_z].getStatus()==1){
            return true;
        }
    }
    // number_z + 1 has overlapping face
    else if (i==5 && number_z < n_voxel_in_axis - 1){
        if(voxelizer.voxelspace[number_x][number_y][number_z + 1].getStatus()==1){
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
