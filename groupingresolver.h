#ifndef GROUPINGRESOLVER_H
#define GROUPINGRESOLVER_H
#include "voxelizer.h"
#include "initialgrouper.h"

class GroupingResolver
{
public:
    GroupingResolver();
    GroupingResolver(QVector<QVector<QString>> CCPs, QVector<QString> overlappingCompsVector, QVector<QPair<QString,QString>> collisionPairsVector,
                     QVector<QPair<QString,QVector<QString>>> group_axisVector);

    QVector<QPair<QString, QVector<QString> > > regroup();
    MachineTool createMT(QVector<QPair<QString,QVector<QString>>>& group_axisVector,
                         QVector<component>& compVector, QVector<JointString>& jointStringVector);
private:
    QVector<QString> getContactCompsVector(QVector<QVector<QString>>& CCPs, QString& compName);
    QVector<QString> getGroupsForRegrouping();
    QString getJointType(QString jointName);
    Vector3 getJointAxis(QString jointName);
    Vector3 getJointXYZ(QString lipCompName,QString jointName, QVector<component>& compVector);

    QVector<QPair<QString,QVector<QString>>> updatedGroup_axisVector;
    QVector<QPair<QString,QVector<QString>>> group_axisVector;
    QVector<QString> GroupsForRegrouping;
    QString targetComp;
    QVector<QString> contactCompsVector;
};

#endif // GROUPINGRESOLVER_H
