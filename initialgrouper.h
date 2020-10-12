#ifndef INITIALGROUPER_H
#define INITIALGROUPER_H
#include<QVector>
#include<qdebug.h>

class InitialGrouper
{
public:
    InitialGrouper(QVector<QVector<QString>> CCPs_input,
                   QVector<QPair<QString,QVector<QString>>>LIPs_input,
                   int num_group, QString lowestCompName);
    QVector<QVector<QString>> startGrouping();

private:
    QVector<QVector<QString>> CCPs;
    QVector<QPair<QString,QVector<QString>>> LIPs;
    QVector<QString> compVector;
    int num_group;
    QString lowestCompName;
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
};

#endif // INITIALGROUPER_H
