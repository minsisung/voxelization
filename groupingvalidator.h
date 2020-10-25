#ifndef GROUPINGVALIDATOR_H
#define GROUPINGVALIDATOR_H
#include"component.h"
#include "voxelizer.h"
#include <qopengl.h>
#include <QElapsedTimer>

class GroupingValidator
{
public:
    GroupingValidator();
    void createMTVoxelspace(float voxelSize, QVector<component>& componentVector);
    void collisionDetectionForConfigurations(MachineTool& MT, bool needVisualization);
    QVector<GLfloat> getData(){return m_data;}
    int getTotalCount(){return m_totalCount;}
    bool getIfNeedVisualization(){return ifNeedVisualization;}

private:
    void setupInitialTransformation(MachineTool& MT);
    void drawVoxelforMT(Link& link, int ind1, int ind2);
    bool checkDuplicateFace(int i, int number_x, int number_y, int number_z);
    QVector3D setNormal(int i);
    Voxelizer voxelizer;
    QVector<GLfloat> m_data;
    int m_totalCount;
    float voxelSize;
    bool ifNeedVisualization = true;
    QSet<QVector<int>>voxelSet;
};

#endif // GROUPINGVALIDATOR_H
