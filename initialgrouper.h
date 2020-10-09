#ifndef INITIALGROUPER_H
#define INITIALGROUPER_H
#include<QVector>
#include<qdebug.h>

class InitialGrouper
{
public:
    InitialGrouper(QVector<QVector<QString>> CCPs_input,
                   QVector<QVector<QString>>LIPs_input, int num_group);
    QVector<QVector<QString>> startGrouping();

private:
    QVector<QVector<QString>> CCPs;
    QVector<QVector<QString>>LIPs;
    QVector<QString> compVector;
    int num_group;
    void mergeGroups(QVector<QVector<QString>>& subgroupVector, int ind_group1, int ind_group2);
    void deleteOverlappingComps(QVector<QString> overlappingCompsVector, QVector<QVector<QString>>& subgroupVector,
                                int ind_group1, int ind_group2);
    bool containsLIIfMerge(QVector<QString>& group1, QVector<QString>& group2, QVector<QVector<QString>>& LIPs);
    QVector<QString> overlappingComps(QVector<QVector<QString>>& subgroupVector, int ind_comp_targetGroup, int ind_comp_CheckingGroup);
    void growsGroup(QVector<QVector<QString>>& subgroupVector, QVector<QString>& compVector,
                    QVector<QVector<QString>>& CCPs);
};

#endif // INITIALGROUPER_H
