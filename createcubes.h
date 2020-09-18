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
    int totocalVertexCount() const { return m_totalCount / 6; }
    void createMTVoxelspace(float voxelSize, QVector<stl_reader::StlMesh <float, unsigned int>>& STLMeshVector);
    void createMTVoxelspace(float voxelSize, QVector<component>& componentVector);
    void createCollisionVoxelspace(float vSize, MachineTool& MT, bool needVisualization);
    void findContactComponentsPairs(QVector<component>& componentVector);
    void findContactComponentsPairsFromURDF(MachineTool& MT);

    void collisionDetectionForConfigurations(MachineTool& MT, bool needVisualization);
    bool checkDuplicateFace(int i, int number_x, int number_y, int number_z);
    bool checkDuplicateFaceforCollision(int i, int number_x, int number_y, int number_z);
    QVector3D setNormal(int i);
    QVector<int> get_vertices_numbers(){return vertices_number_vector;}
    bool ifNeedVisualization = true;
    int m_totalVoxelCount = 0;

private:
    QVector<GLfloat> m_data;
    int m_totalCount;
    Voxelizer voxelizer;
    QVector<int> vertices_number_vector;
    float voxelSize;
    void drawVoxelforCollision(Link& link);
    void drawVoxelforMT(Link& link, int ind1, int ind2);
    void drawVoxelforCCP(QVector<contactComponentsPair>& CCPVector);
    QVector<stl_reader::StlMesh <float, unsigned int>> meshVector;
    void setupInitialTransformation(MachineTool& MT);
    void setupTransformation(MachineTool& MT, QChar linkType, float amount);
    QVector<stl_reader::StlMesh <float, unsigned int>> m_STLMeshVector;
};


void findBoundingBox(const float* c);

#endif // CREATECUBES_H
