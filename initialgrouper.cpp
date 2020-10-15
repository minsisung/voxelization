#include "initialgrouper.h"

InitialGrouper::InitialGrouper(QVector<QVector<QString>> CCPs_input,
                               QVector<QPair<QString,QVector<QString>>> LIPs_input,
                               int num_group_input, QString lowestCompName_input)
    :CCPs(CCPs_input), LIPs(LIPs_input),
      num_group(num_group_input), lowestCompName(lowestCompName_input){

    //delete components if there is no any connection to other component
    QVector<QString> compVectorUpdate;
    for(int ind_ccp = 0; ind_ccp < CCPs.size(); ind_ccp++){
        if(!compVectorUpdate.contains(CCPs[ind_ccp][0]))
            compVectorUpdate.append(CCPs[ind_ccp][0]);

        if(!compVectorUpdate.contains(CCPs[ind_ccp][1]))
            compVectorUpdate.append(CCPs[ind_ccp][1]);
    }
    compVector = compVectorUpdate;

    //delete LIPs from CCPs
    for(int ind_lip = 0; ind_lip < LIPs.size(); ind_lip++){
        int ind = -1;

        for(int ind_ccp = 0; ind_ccp < CCPs.size(); ind_ccp++){
            if(LIPs[ind_lip].second[0] == CCPs[ind_ccp][0] &&
                    LIPs[ind_lip].second[1] == CCPs[ind_ccp][1])
                ind = ind_ccp;
        }
        if(ind != -1)
            CCPs.remove(ind);
    }


}

void InitialGrouper::mergeGroups(QVector<QVector<QString> > &subgroupVector, int ind_group1, int ind_group2)
{
    // put non-overlapping components in group2 to group1
    for(int ind_comp = 0; ind_comp < subgroupVector[ind_group2].size(); ind_comp++){
        if(!subgroupVector[ind_group1].contains(subgroupVector[ind_group2][ind_comp]))
            subgroupVector[ind_group1].append(subgroupVector[ind_group2][ind_comp]);
    }

    //remove the group2
    subgroupVector.remove(ind_group2);
}

void InitialGrouper::deleteOverlappingComps(QVector<QString> overlappingCompsVector, QVector<QVector<QString> > &subgroupVector, int ind_group1, int ind_group2)
{
    // move overlapping component to the subgroup with less components
    if(subgroupVector[ind_group2].size() <= subgroupVector[ind_group1].size()){
        qDebug()<<"The overlapping components are deleted from group"<< ind_group1;
        for(int ind_overlappingComp = 0; ind_overlappingComp < overlappingCompsVector.size(); ind_overlappingComp++){
            subgroupVector[ind_group1].remove(subgroupVector[ind_group1].indexOf(overlappingCompsVector[ind_overlappingComp]));
        }
    }else{
        qDebug()<<"The overlapping components are deleted from group"<< ind_group2<<endl;
        for(int ind_overlappingComp = 0; ind_overlappingComp < overlappingCompsVector.size(); ind_overlappingComp++){
            subgroupVector[ind_group2].remove(subgroupVector[ind_group2].indexOf(overlappingCompsVector[ind_overlappingComp]));
        }
    }
}

bool InitialGrouper::containsLIIfMerge(QVector<QString> &group1, QVector<QString> &group2,
                                       QVector<QPair<QString,QVector<QString>>>& LIPs)
{
    bool contains = false;
    for(int ind_lip = 0; ind_lip < LIPs.size();ind_lip++){
        if((group1.contains(LIPs[ind_lip].second[0]) | group2.contains(LIPs[ind_lip].second[0])) &&
                (group1.contains(LIPs[ind_lip].second[1]) | group2.contains(LIPs[ind_lip].second[1]))){
            qDebug()<<"There are link interface in these two subgroups if merge which is:";
            qDebug()<<LIPs[ind_lip].second[0]<<LIPs[ind_lip].second[1]<<endl;
            contains = true;
            break;
        }
    }
    return contains;
}

QVector<QString> InitialGrouper::overlappingComps(QVector<QVector<QString> > &subgroupVector, int ind_comp_targetGroup, int ind_comp_CheckingGroup)
{
    QVector<QString> overlappingComps;
    QVector<QString>& targetGroup = subgroupVector[ind_comp_targetGroup];
    QVector<QString>& checkingGroup = subgroupVector[ind_comp_CheckingGroup];

    for(int ind_comp = 1; ind_comp < targetGroup.size(); ind_comp++){
        //if there is component overlapping
        if(checkingGroup.contains(targetGroup[ind_comp])){
            QString overlapping_comp = targetGroup[ind_comp];
            overlappingComps.append(overlapping_comp);
            qDebug()<<"subgroup"<<ind_comp_CheckingGroup + 1<<"and subgroup"<<ind_comp_targetGroup + 1<<"both contains"<<overlapping_comp;
        }
    }
    return overlappingComps;
}

void InitialGrouper::growsGroup(QVector<QVector<QString> > &subgroupVector, QVector<QString> &compVector,
                                QVector<QVector<QString> > &CCPs)
{
    QVector<QString> compsNeedDelete;
    for(int ind_comp = 0; ind_comp < compVector.size(); ind_comp++){
        for(int ind_CCP = 0; ind_CCP < CCPs.size(); ind_CCP++){

            //find the contact component of this component
            QString contactComp;
            if(CCPs[ind_CCP][0] == compVector[ind_comp])
                contactComp = CCPs[ind_CCP][1];

            if(CCPs[ind_CCP][1] == compVector[ind_comp])
                contactComp = CCPs[ind_CCP][0];

            if(contactComp != ""){
                for(int ind_subgroup = 0; ind_subgroup < subgroupVector.size(); ind_subgroup++){

                    //find which subgroup contains this contact component, if it contains, append this component into this group
                    if(subgroupVector[ind_subgroup].contains(contactComp) && !subgroupVector[ind_subgroup].contains(compVector[ind_comp]) &&
                            !compVector.contains(contactComp)){
                        subgroupVector[ind_subgroup].append(compVector[ind_comp]);
                        compsNeedDelete.append(compVector[ind_comp]);
                    }
                }
            }
        }
    }

    //delete the components that are assigned to groups from compVector
    for(int ind_comp = 0; ind_comp < compsNeedDelete.size(); ind_comp++){
        compVector.remove(compVector.indexOf(compsNeedDelete[ind_comp]));
    }
}

QVector<QPair<QString,QVector<QString>>> InitialGrouper::assignAxisToGroups(
        QVector<QVector<QString>>& subgroupVector)
{
    //Vector of pair which first element is the axis and second one is the components in the group
    QVector<QPair<QString,QVector<QString>>> group_axisVector;

    //assign Base to the corresponding group
    int ind_base = -1;
    for(int ind_group = 0; ind_group < subgroupVector.size(); ind_group++){
        QString axisName;
        if(subgroupVector[ind_group].contains(lowestCompName)){
            axisName = "Base";
            ind_base = ind_group;
        }else{
            axisName = "";
        }
        group_axisVector.append(qMakePair(axisName, subgroupVector[ind_group]));
    }

    //assign axis type to the corresponding group
    QVector<QPair<QString,QVector<QString>>> LIPsCopy = LIPs;
    while(!LIPsCopy.isEmpty()){
        for(int ind_lip = 0; ind_lip < LIPsCopy.size(); ind_lip++){
            for(int ind1 = 0; ind1 < group_axisVector.size() - 1; ind1++){
                for(int ind2 = ind1 + 1; ind2 < group_axisVector.size(); ind2++){

                    //find the groups that are connected by the lip
                    if((group_axisVector[ind1].second.contains(LIPsCopy[ind_lip].second[0]) &&
                        group_axisVector[ind2].second.contains(LIPsCopy[ind_lip].second[1])) |
                            (group_axisVector[ind1].second.contains(LIPsCopy[ind_lip].second[1]) &&
                             group_axisVector[ind2].second.contains(LIPsCopy[ind_lip].second[0]))){

                        //find the condition that one of the group connected by the lip hasn't be assigned axis
                        //and the other one has, then assign the no assigned group and delete the lip
                        if(group_axisVector[ind1].first == "" & group_axisVector[ind2].first != ""){
                            group_axisVector[ind1].first = LIPsCopy[ind_lip].first;
                            LIPsCopy.remove(ind_lip);
                            continue;
                        }
                        if(group_axisVector[ind2].first == "" & group_axisVector[ind1].first != ""){
                            group_axisVector[ind2].first = LIPsCopy[ind_lip].first;
                            LIPsCopy.remove(ind_lip);
                            continue;
                        }
                    }
                }
            }
        }
    }

    for(int ind_group = 0; ind_group < group_axisVector.size(); ind_group++){
        qDebug()<<"Group"<<group_axisVector[ind_group].first<<"contains"<<
                  group_axisVector[ind_group].second<<endl;
    }
    return group_axisVector;
}

MachineTool InitialGrouper::createMT(QVector<QPair<QString, QVector<QString> > > group_axisVector)
{
    //Create machine tool object
    MachineTool MT;
    Link link_reading("Base");
    MT.LinkVector.push_back(link_reading);


    return MT;
}

QVector<QVector<QString>> InitialGrouper::startGrouping()
{
    //print out inputs
    qDebug()<<"All components:"<<compVector<<endl;
    qDebug()<<"All CCPs";
    for(int i = 0 ; i < CCPs.size(); i++){
        qDebug()<<CCPs[i][0]<<" "<< CCPs[i][1];
    }
    qDebug()<<endl;

    qDebug()<<"All LIPs";
    for(int i = 0 ; i < LIPs.size(); i++){
        qDebug()<<"Axis type of lip:"<<LIPs[i].first;
        qDebug()<<LIPs[i].second[0]<<" "<<LIPs[i].second[1];
    }
    qDebug()<<endl;

    if(LIPs.size() != num_group - 1){
        qDebug()<<"LIPs number != what this machine should have which is"<<num_group - 1<<endl;
        QVector<QVector<QString>> fault;
        return fault;
    }

    //build LIP component vector to store all lip component without repeating
    QVector<QString >LIP_Com_Vector;
    for(int ind_LIP = 0; ind_LIP < LIPs.size(); ind_LIP++){
        for(int i = 0 ; i < 2; i++){
            if(!LIP_Com_Vector.contains(LIPs[ind_LIP].second[i]))
                LIP_Com_Vector.append(LIPs[ind_LIP].second[i]);
        }
    }

    qDebug() <<"LIP Components";
    for(int ind_LIP = 0; ind_LIP < LIP_Com_Vector.size(); ind_LIP++)
        qDebug()<< LIP_Com_Vector[ind_LIP];
    qDebug()<<endl;

    //Start doing grouping------------------------------------------------
    QVector<QVector<QString>> subgroupVector;

    //create subgroup according to the number of lip components
    for(int ind_LIP_Comp = 0; ind_LIP_Comp < LIP_Com_Vector.size(); ind_LIP_Comp++){
        QVector<QString> subgroup;
        subgroup.append(LIP_Com_Vector[ind_LIP_Comp]);

        //delete this component from the component vector
        int ind = compVector.indexOf(LIP_Com_Vector[ind_LIP_Comp]);

        //if this component contains in compVector, remove it
        if(ind != -1)
            compVector.remove(ind);

        for(int ind_CCP = 0 ; ind_CCP < CCPs.size(); ind_CCP++){
            if(CCPs[ind_CCP][0] == LIP_Com_Vector[ind_LIP_Comp]){
                subgroup.append(CCPs[ind_CCP][1]);
                int ind = compVector.indexOf(CCPs[ind_CCP][1]);

                //if this component contains in compVector, remove it
                if(ind != -1)
                    compVector.remove(ind);
            }

            if(CCPs[ind_CCP][1] == LIP_Com_Vector[ind_LIP_Comp]){
                subgroup.append(CCPs[ind_CCP][0]);
                int ind = compVector.indexOf(CCPs[ind_CCP][0]);

                //if this component contains in compVector, remove it
                if(ind != -1)
                    compVector.remove(ind);
            }
        }
        subgroupVector.append(subgroup);
    }

    for(int ind_subgroup = 0; ind_subgroup < subgroupVector.size(); ind_subgroup++){
        qDebug()<<"subgroups"<<ind_subgroup + 1;
        for(int ind_comp = 0; ind_comp < subgroupVector[ind_subgroup].size(); ind_comp++){
            qDebug()<<subgroupVector[ind_subgroup][ind_comp]<<" ";
        }
        qDebug()<<endl;
    }

    //keep doing iteration if there is component left being assigned to any group
    //or the number of subgroup is not number of axis + 1
    while(compVector.size()!= 0 | subgroupVector.size() != num_group){

        for(int ind_subgroup1 = 0; ind_subgroup1 < subgroupVector.size(); ind_subgroup1++){
            //merge subgroups if there is overlapping components in two subgroups
            //loop all other subgroups
            for(int ind_subgroup2 = ind_subgroup1 + 1; ind_subgroup2 < subgroupVector.size(); ind_subgroup2++){

                //vector to store overlapping components if exist
                QVector<QString> overlappingCompsVector = overlappingComps(subgroupVector, ind_subgroup1, ind_subgroup2);

                //if there is component overlapping
                if(!overlappingCompsVector.empty()){
                    qDebug()<<"There are overlapping component";
                    if(containsLIIfMerge(subgroupVector[ind_subgroup1], subgroupVector[ind_subgroup2], LIPs)){
                        deleteOverlappingComps(overlappingCompsVector, subgroupVector,ind_subgroup1, ind_subgroup2);
                        qDebug()<<"-------------------------------------------------";
                    }else{
                        qDebug()<<"No LI contain after merging so merge group"<<ind_subgroup1 + 1 <<"and group"<<ind_subgroup2 + 1;
                        mergeGroups(subgroupVector, ind_subgroup1, ind_subgroup2);
                        qDebug()<<"-------------------------------------------------";
                    }
                }
            }
        }

        //after merge operation
        qDebug()<<endl<<"after merge operation"<<endl;
        for(int ind_subgroup = 0; ind_subgroup < subgroupVector.size(); ind_subgroup++){
            qDebug()<<"subgroups"<<ind_subgroup + 1;
            for(int ind_comp = 0; ind_comp < subgroupVector[ind_subgroup].size(); ind_comp++){
                qDebug()<<subgroupVector[ind_subgroup][ind_comp]<<" ";
            }
            qDebug()<<endl;
        }

        qDebug()<<"All components after merging:"<<compVector<<endl;

        qDebug()<<"All CCPs after merging:"<<CCPs<<endl;

        growsGroup(subgroupVector, compVector, CCPs);

        qDebug()<<endl<<"after growing operation"<<endl;
        for(int ind_subgroup = 0; ind_subgroup < subgroupVector.size(); ind_subgroup++){
            qDebug()<<"subgroups"<<ind_subgroup + 1;
            for(int ind_comp = 0; ind_comp < subgroupVector[ind_subgroup].size(); ind_comp++){
                qDebug()<<subgroupVector[ind_subgroup][ind_comp]<<" ";
            }
            qDebug()<<endl;
        }

        qDebug()<<"All components after growing:"<<compVector<<endl;
        qDebug()<<"There are "<<subgroupVector.size()<<"subgroups"<<endl;
    }

    //Assign axis to each group
    QVector<QPair<QString,QVector<QString>>>group_axisVector = assignAxisToGroups(subgroupVector);

    return subgroupVector;
}
