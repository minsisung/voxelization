#include "createcubes.h"
#include <qvector3d.h>
#include <QDebug>
#include<iostream>

CreateCubes::CreateCubes():
    m_totalCount(0)
{

}

void CreateCubes::createVoxelspace(float spaceLength, float voxelSize,QString filepath)
{
    Voxelizer voxelizer{spaceLength, voxelSize};
    try {
        stl_reader::StlMesh <float, unsigned int> mesh(filepath.toStdString());
        voxelizer.Voxelize(mesh);
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }



    int n_voxel_in_axis = static_cast<int>(spaceLength / voxelSize);
    float mostLeftBottom = -spaceLength/2.0f;

    for (int number_x = 0; number_x < n_voxel_in_axis; ++number_x) {
        for (int number_y = 0; number_y < n_voxel_in_axis; ++number_y) {
            for (int number_z = 0; number_z < n_voxel_in_axis; ++number_z) {
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


                GLfloat coords[6][4][3] = {
                    { { x_right, y_down, z_futher}, { x_left, y_down, z_futher}, { x_left, y_up, z_futher}, { x_right, y_up, z_futher} },
                    { { x_right, y_up, z_futher}, { x_left, y_up, z_futher}, { x_left, y_up, z_closer}, { x_right, y_up, z_closer} },
                    { { x_right, y_down, z_closer}, { x_right, y_down, z_futher}, { x_right, y_up, z_futher}, { x_right, y_up, z_closer} },
                    { { x_left,y_down, z_futher}, { x_left, y_down, z_closer}, { x_left, y_up, z_closer}, { x_left, y_up, z_futher} },
                    { { x_right,y_down, z_closer}, { x_left, y_down, z_closer}, { x_left, y_down, z_futher}, { x_right, y_down, z_futher} },
                    { { x_left, y_down, z_closer}, { x_right, y_down, z_closer}, { x_right, y_up, z_closer}, { x_left, y_up, z_closer} }
                };

                QVector3D normal;
                for (int i = 0; i < 6; ++i) {
                    // vertex position
                    if (i==0)
                        normal = { 0.0,0.0,-1.0};
                    else if(i==1)
                        normal = { 0.0,1.0,0.0};
                    else if(i==2)
                        normal = { 1.0,0.0,0.0};
                    else if(i==3)
                        normal = { -1.0,0.0,0.0};
                    else if(i==4)
                        normal = { 0.0,-1.0,0.0};
                    else if(i==5)
                        normal = { 0.0,0.0,1.0};


                    m_data.resize(m_totalCount+6);
                    GLfloat *p = m_data.data() + m_totalCount;
                    *p++ = coords[i][0][0];
                    *p++ = coords[i][0][1];
                    *p++ = coords[i][0][2];
                    *p++ = normal[0];
                    *p++ = normal[1];
                    *p++ = normal[2];
                    m_totalCount += 6;

                    m_data.resize(m_totalCount+6);
                    p = m_data.data() + m_totalCount;
                    *p++ = coords[i][1][0];
                    *p++ = coords[i][1][1];
                    *p++ = coords[i][1][2];
                    *p++ = normal[0];
                    *p++ = normal[1];
                    *p++ = normal[2];
                    m_totalCount += 6;

                    m_data.resize(m_totalCount+6);
                    p = m_data.data() + m_totalCount;
                    *p++ = coords[i][2][0];
                    *p++ = coords[i][2][1];
                    *p++ = coords[i][2][2];
                    *p++ = normal[0];
                    *p++ = normal[1];
                    *p++ = normal[2];
                    m_totalCount += 6;

                    /////////////////////////////////////////////
                    m_data.resize(m_totalCount+6);
                    p = m_data.data() + m_totalCount;
                    *p++ = coords[i][0][0];
                    *p++ = coords[i][0][1];
                    *p++ = coords[i][0][2];
                    *p++ = normal[0];
                    *p++ = normal[1];
                    *p++ = normal[2];
                    m_totalCount += 6;

                    m_data.resize(m_totalCount+6);
                    p = m_data.data() + m_totalCount;
                    *p++ = coords[i][2][0];
                    *p++ = coords[i][2][1];
                    *p++ = coords[i][2][2];
                    *p++ = normal[0];
                    *p++ = normal[1];
                    *p++ = normal[2];
                    m_totalCount += 6;

                    m_data.resize(m_totalCount+6);
                    p = m_data.data() + m_totalCount;
                    *p++ = coords[i][3][0];
                    *p++ = coords[i][3][1];
                    *p++ = coords[i][3][2];
                    *p++ = normal[0];
                    *p++ = normal[1];
                    *p++ = normal[2];
                    m_totalCount += 6;

                }

            }
        }
    }
}
