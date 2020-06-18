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
    void createMTVoxelspace(float spaceLength, float voxelSize, MachineTool& MT, bool needVisualization);
    void createCollisionVoxelspace(float spaceLength, float vSize, MachineTool& MT, bool needVisualization);
    bool checkDuplicateFace(int i, int number_x, int number_y, int number_z);
    bool checkDuplicateFaceforCoincident(int i, int number_x, int number_y, int number_z);
    QVector3D setNormal(int i);
    QVector<int> get_vertices_numbers(){return vertices_number_vector;}
    bool ifNeedVisualization = true;

private:
    QVector<GLfloat> m_data;
    int m_totalCount;
    Voxelizer voxelizer;
    int n_voxel_in_axis;
    QVector<int> vertices_number_vector;
    float voxelSize;
    float mostLeftBottom;
    void drawVoxelforCoincident(Link& link);
    void drawVoxelforMT(Link& link);
    QVector<stl_reader::StlMesh <float, unsigned int>> meshVector;
    void setupInitialTransformation(MachineTool& MT);
    void setupTransformation(MachineTool& MT, char linkType, float amount);
};


void findBoundingBox(const float* c);

#endif // CREATECUBES_H
