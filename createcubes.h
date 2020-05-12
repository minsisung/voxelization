#ifndef CREATECUBES_H
#define CREATECUBES_H

#include <qopengl.h>
#include <QVector>
#include "voxelizer.h"
#include <qvector3d.h>

class CreateCubes
{
public:
    CreateCubes();
    const GLfloat *constData() const { return m_data.constData(); }
    int totalCount() const { return m_totalCount; }
    int totocalVertexCount() const { return m_totalCount / 6; }
    void createVoxelspace(float spaceLength, float voxelSize,QStringList m_filepathes);
    bool checkDuplicateFace(int i, int number_x, int number_y, int number_z);
    QVector3D setNormal(int i);
    QVector<int> get_vertices_numbers(){return vertices_number_vector;}

private:
    QVector<GLfloat> m_data;
    int m_totalCount;
    Voxelizer voxelizer;
    int n_voxel_in_axis;
    QVector<int> vertices_number_vector;
};


void findBoundingBox(const float* c);

#endif // CREATECUBES_H
