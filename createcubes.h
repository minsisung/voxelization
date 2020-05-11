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
    void createVoxelspace(float spaceLength, float voxelSize,QString filepath1, QString filepath2);
    bool checkDuplicateFace(int i, int number_x, int number_y, int number_z);
    QVector3D setNormal(int i);
private:
    QVector<GLfloat> m_data;
    int m_totalCount;
    Voxelizer voxelizer;
    int n_voxel_in_axis;
};


void findBoundingBox(const float* c);

#endif // CREATECUBES_H
