#include "creategeometry.h"
#include <qfile.h>
#include<qtextstream.h>
#include<stl_reader.h>
#include<iostream>
#include<QDebug>

CreateGeometry::CreateGeometry():
    m_totalCount(0),

    min_x(1000000),max_x(-1000000),
    min_y(1000000),max_y(-1000000),
    min_z(1000000),max_z(-1000000){}

void CreateGeometry::readSTL(QString filepath)
{
    try {
        stl_reader::StlMesh <float, unsigned int> mesh(filepath.toStdString());

        for (size_t itri = 0; itri < mesh.num_tris(); ++itri) {
            const float* c = nullptr;
            const float* n = mesh.tri_normal(itri);
            for (size_t icorner = 0; icorner < 3; ++icorner) {
                c = mesh.vrt_coords(mesh.tri_corner_ind(itri, icorner));

                m_data.resize(m_totalCount+6);
                GLfloat *p = m_data.data() + m_totalCount;
                *p++ = 1000*c[0];
                *p++ = 1000*c[1];
                *p++ = 1000*c[2];
                *p++ = n[0];
                *p++ = n[1];
                *p++ = n[2];
                m_totalCount += 6;
            }
        }
        qDebug()<<"totalCount "<<m_totalCount<<endl;
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }
}

