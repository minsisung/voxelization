#ifndef CREATECUBES_H
#define CREATECUBES_H

#include <qopengl.h>
#include <QVector>
#include "voxelizer.h"

class CreateCubes
{
public:
    CreateCubes();
    const GLfloat *constData() const { return m_data.constData(); }
    int totalCount() const { return m_totalCount; }
    int totocalVertexCount() const { return m_totalCount / 6; }
    void createVoxelspace(float spaceLength, float voxelSize,QString filepath);
private:
    QVector<GLfloat> m_data;
    int m_totalCount;
};


void findBoundingBox(const float* c);

#endif // CREATECUBES_H
