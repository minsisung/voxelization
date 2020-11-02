#ifndef INITIALGROUPER_H
#define INITIALGROUPER_H
#include<QVector>
#include<qdebug.h>
#include<machinetool.h>
#include<component.h>

struct JointString
{
    QString name;
    QString link_parent;
    QString link_child;
    QString compInChildLink;
//    Vector3 xyz;
};

class InitialGrouper
{
public:
    InitialGrouper(QVector<QVector<QString>> CCPs_input,
                   QVector<QPair<QString,QVector<QString>>>LIPs_input,
                   int num_group,QVector<component> &compnent_Mesh_Vector);
    QVector<QPair<QString,QVector<QString>>> startGrouping();
    MachineTool createMT(QVector<QPair<QString,QVector<QString>>>& group_axisVector,
                         QVector<component>& compVector);
    QVector<QString> getOverlappingCompsVector(){return m_overlappingCompsVector;}
    QVector<JointString> getjointStringVector(){return jointStringVector;}

private:
    QVector<QVector<QString>> CCPs;
    QVector<QPair<QString,QVector<QString>>> LIPs;
    QVector<QString> compVector;
    int num_group;
    QString lowestCompName;
    QVector<QString> m_overlappingCompsVector;
    void mergeGroups(QVector<QVector<QString>>& subgroupVector, int ind_group1, int ind_group2);
    void deleteOverlappingComps(QVector<QString> overlappingCompsVector, QVector<QVector<QString>>& subgroupVector,
                                int ind_group1, int ind_group2);
    bool containsLIIfMerge(QVector<QString>& group1, QVector<QString>& group2,
                           QVector<QPair<QString,QVector<QString>>>& LIPs);
    QVector<QString> overlappingComps(QVector<QVector<QString>>& subgroupVector, int ind_comp_targetGroup, int ind_comp_CheckingGroup);
    void growsGroup(QVector<QVector<QString>>& subgroupVector, QVector<QString>& compVector,
                    QVector<QVector<QString>>& CCPs);
    QVector<QPair<QString,QVector<QString>>> assignAxisToGroups(
            QVector<QVector<QString>>& subgroupVector);
    QString getJointType(QString jointName);
    Vector3 getJointAxis(QString jointName);
    Vector3 getJointXYZ(QString lipCompName,QString jointName, QVector<component>& compVector);
    void setLowestComponent(QVector<component> &compnent_Mesh_Vector);
    QVector<JointString> jointStringVector;
};



#endif // INITIALGROUPER_H
