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
    QVector<QPair<QString,QString>> collisionDetectionForConfigurations(MachineTool& MT, bool needVisualization);
    QVector<GLfloat> getData(){return m_data;}
    int getTotalCount(){return m_totalCount;}
    bool getIfNeedVisualization(){return ifNeedVisualization;}
    void drawVoxelforMT(MachineTool& MT, int ind1, int ind2);
    void clear();

private:
    void setupInitialTransformation(MachineTool& MT);
    bool checkDuplicateFace(int i, int number_x, int number_y, int number_z);
    QVector3D setNormal(int i);
    QPair<QString, QString> getCollidedPairName(MachineTool& MT, QString linkName1, QString linkName2);
    Voxelizer voxelizer;
    QVector<GLfloat> m_data;
    int m_totalCount;
    float voxelSize;
    bool ifNeedVisualization = true;
    QSet<QVector<int>>voxelSet;
};

#endif // GROUPINGVALIDATOR_H
