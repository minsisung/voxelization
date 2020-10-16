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
    QString createMTVoxelspace(float voxelSize, QVector<component>& componentVector);
    void createCollisionVoxelspace(float vSize, MachineTool& MT, bool needVisualization);
    QVector<contactComponentsPair> findContactComponentsPairs(QVector<component>& componentVector);
    int findLIPCandidates(QVector<contactComponentsPair>& ccpVector);
    void collisionDetectionForConfigurations(MachineTool& MT, bool needVisualization);
    bool checkDuplicateFace(int i, int number_x, int number_y, int number_z);
    bool checkDuplicateFaceforCollision(int i, int number_x, int number_y, int number_z);
    QVector3D setNormal(int i);
    QVector<int> get_vertices_numbers(){return vertices_number_vector;}
    bool ifNeedVisualization = true;
    int m_totalVoxelCount = 0;
    int numberOfVertex_comp1 = 0;
    int numberOfVertex_comp2 = 0;
    QVector<QVector<QString>> CCPs;
    QVector<QPair<QString,QVector<QString>>> LIPs;

private:
    QVector<GLfloat> m_data;
    int m_totalCount;
    Voxelizer voxelizer;
    QVector<int> vertices_number_vector;
    float voxelSize;
    void drawVoxelforCollision(Link& link);
    void drawVoxelforMT(Link& link, int ind1, int ind2);
    void drawVoxelforCCP(QList<QVector3D> &compVoxelIndicesList1, QList<QVector3D> &compVoxelIndicesList2);
    QVector<stl_reader::StlMesh <float, unsigned int>> meshVector;
    void setupInitialTransformation(MachineTool& MT);
    void setupTransformation(MachineTool& MT, QChar linkType, float amount);
    QVector<stl_reader::StlMesh <float, unsigned int>> m_STLMeshVector;
};


void findBoundingBox(const float* c);

#endif // CREATECUBES_H
