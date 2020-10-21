#ifndef CREATECUBES_H
#define CREATECUBES_H

#include <qopengl.h>
#include <QVector>
#include "voxelizer.h"
#include <qvector3d.h>
#include "machinetool.h"

class CreateCubes
{
public:
    CreateCubes();
    const GLfloat *constData() const { return m_data.constData(); }
    int totalCount() const { return m_totalCount; }
    void createMTVoxelspace(float voxelSize, QVector<stl_reader::StlMesh <float, unsigned int>>& STLMeshVector);
    void createCollisionVoxelspace(float vSize, MachineTool& MT, bool needVisualization);
    bool checkDuplicateFace(int i, int number_x, int number_y, int number_z);
    bool checkDuplicateFaceforCollision(int i, int number_x, int number_y, int number_z);
    void setData(QVector<GLfloat> data){m_data = data;}
    void setTotalCount(int totalCount){m_totalCount = totalCount;}
    QVector3D setNormal(int i);
    bool ifNeedVisualization = true;
    int numberOfVertex_comp1 = 0;
    int numberOfVertex_comp2 = 0;
    QString m_paintMode;

private:
    QVector<GLfloat> m_data;
    int m_totalCount;
    Voxelizer voxelizer;
    float voxelSize;
    void drawVoxelforCollision(Link& link);
    QVector<stl_reader::StlMesh <float, unsigned int>> meshVector;
    void setupInitialTransformation(MachineTool& MT);
    void setupTransformation(MachineTool& MT, QChar linkType, float amount);
    QVector<stl_reader::StlMesh <float, unsigned int>> m_STLMeshVector;
};


void findBoundingBox(const float* c);

#endif // CREATECUBES_H
