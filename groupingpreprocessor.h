#ifndef GROUPINGPREPROCESSOR_H
#define GROUPINGPREPROCESSOR_H
#include "component.h"
#include "voxelizer.h"
#include <qopengl.h>

class GroupingPreProcessor
{
public:
    GroupingPreProcessor();
    ~GroupingPreProcessor();
    void createMTVoxelspace(float voxelSize, QVector<component>& componentVector);
    QVector<contactComponentsPair> findContactComponentsPairs(QVector<component>& componentVector);
    int findLIPCandidates(QVector<contactComponentsPair>& ccpVector);
    QVector<GLfloat> getData(){return m_data;}
    int getTotalCount(){return m_totalCount;}

    QVector<QVector<QString>> CCPs;
    QVector<QPair<QString,QVector<QString>>> LIPs;
    int numberOfVertex_comp1 = 0;
    int numberOfVertex_comp2 = 0;

private:
    void drawVoxelforCCP(QList<QVector3D> &compVoxelIndicesList1, QList<QVector3D> &compVoxelIndicesList2);
    void updateCCPVector(QVector<contactComponentsPair>& ccpVector);

    QVector3D setNormal(int i);
    Voxelizer voxelizer;
    QVector<GLfloat> m_data;
    int m_totalCount;
    float voxelSize;
    QString drawingCCPName;
};

#endif // GROUPINGPREPROCESSOR_H
