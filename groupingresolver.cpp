#include "groupingresolver.h"

GroupingResolver::GroupingResolver()
{}

GroupingResolver::GroupingResolver(QVector<QVector<QString> > CCPs, QVector<QString> overlappingCompsVector, QVector<QPair<QString,QString>> collisionPairsVector,
                                   QVector<QPair<QString,QVector<QString>>> group_axisVector_original):
    group_axisVector(group_axisVector_original)
{    
    //find the CCP partner of the overlapping comp that occurs collision
    for(int ind_collisionPair = 0; ind_collisionPair < collisionPairsVector.size(); ind_collisionPair++){

        //if the first component of the collision pair is in the overlapping comp vector
        if(overlappingCompsVector.contains(collisionPairsVector[ind_collisionPair].first)){
            targetComp = collisionPairsVector[ind_collisionPair].first;
            contactCompsVector = getContactCompsVector(CCPs, collisionPairsVector[ind_collisionPair].first);
        }

        //if the second component of the collision pair is in the overlapping comp vector
        if(overlappingCompsVector.contains(collisionPairsVector[ind_collisionPair].second)){
            targetComp = collisionPairsVector[ind_collisionPair].second;
            contactCompsVector = getContactCompsVector(CCPs, collisionPairsVector[ind_collisionPair].second);
        }
    }

    //get regrouping options
    GroupsForRegrouping = getGroupsForRegrouping();
}

QVector<QString> GroupingResolver::getContactCompsVector(QVector<QVector<QString> >& CCPs, QString& compName)
{
    QVector<QString> contactCompsVector;
    for(int ind_CCP = 0; ind_CCP < CCPs.size(); ind_CCP++){
        if(CCPs[ind_CCP][0] == compName){
            contactCompsVector.append(CCPs[ind_CCP][1]);
        }

        if(CCPs[ind_CCP][1] == compName){
            contactCompsVector.append(CCPs[ind_CCP][0]);
        }
    }
    return contactCompsVector;
}

QVector<QString> GroupingResolver::getGroupsForRegrouping()
{
    QVector<QString> groups;
    QString groupContainsTargetComp;

    for(int ind_group = 0; ind_group < group_axisVector.size(); ind_group++){

        //find the group that contains targetComp
        if(group_axisVector[ind_group].second.contains(targetComp)){
            groupContainsTargetComp = group_axisVector[ind_group].first;
            qDebug()<<"Target component is in the group:"<<groupContainsTargetComp;
        }

        for(int ind_cc = 0; ind_cc < contactCompsVector.size(); ind_cc++){
            if(group_axisVector[ind_group].second.contains(contactCompsVector[ind_cc]) &&
                    !groups.contains(group_axisVector[ind_group].first))
                groups.append(group_axisVector[ind_group].first);
        }
    }
    groups.removeAt(groups.indexOf(groupContainsTargetComp));
    qDebug()<<"Target componentis in the group:"<<groupContainsTargetComp<<targetComp<<endl;
    qDebug()<<"regroup options:"<<groups;

    return groups;

}

QString GroupingResolver::getJointType(QString jointName)
{
    if(jointName == "X" | jointName == "Y" | jointName == "Z")
        return "prismatic";
    return "revolute";
}

Vector3 GroupingResolver::getJointAxis(QString jointName)
{
    if(jointName == "X" | jointName == "A"){
        Vector3 axis(1,0,0);
        return axis;
    }
    if(jointName == "Y" | jointName == "B"){
        Vector3 axis(0,1,0);
        return axis;
    }
    Vector3 axis(0,0,1);
    return axis;
}

Vector3 GroupingResolver::getJointXYZ(QString lipCompName, QString jointName, QVector<component> &compVector)
{
    component& component = compVector[compVector.indexOf(lipCompName)];

    //AB machine tool
    if(component.m_mtRotaryAxes == QVector3D(1.0,1.0,0.0)){
        QVector3D axisVector;
        if(jointName == "A"){
            axisVector = component.getRotaryAxisPoint1();
            Vector3 axis(axisVector.x(),axisVector.y(),axisVector.z());
            return axis;
        }
        if(jointName == "B"){
            axisVector = component.getRotaryAxisPoint2();
            Vector3 axis(axisVector.x(),axisVector.y(),axisVector.z());
            return axis;
        }
    }

    //AC machine tool
    if(component.m_mtRotaryAxes == QVector3D(1.0,0.0,1.0)){
        QVector3D axisVector;
        if(jointName == "A"){
            axisVector = component.getRotaryAxisPoint1();
            Vector3 axis(axisVector.x(),axisVector.y(),axisVector.z());
            return axis;
        }
        if(jointName == "C"){
            axisVector = component.getRotaryAxisPoint2();
            Vector3 axis(axisVector.x(),axisVector.y(),axisVector.z());
            return axis;
        }
    }

    //BC machine tool
    if(component.m_mtRotaryAxes == QVector3D(0.0,1.0,1.0)){
        QVector3D axisVector;
        if(jointName == "B"){
            axisVector = component.getRotaryAxisPoint1();
            Vector3 axis(axisVector.x(),axisVector.y(),axisVector.z());
            return axis;
        }
        if(jointName == "C"){
            axisVector = component.getRotaryAxisPoint2();
            Vector3 axis(axisVector.x(),axisVector.y(),axisVector.z());
            return axis;
        }
    }
    Vector3 axis_prismatic(0,0,0);
    return axis_prismatic;
}

QVector<QPair<QString, QVector<QString> > > GroupingResolver::regroup()
{
    for(int ind_group = 0; ind_group < group_axisVector.size(); ind_group++){

        //find the group that contains targetComp
        int ind_comp = group_axisVector[ind_group].second.indexOf(targetComp);

        //remove the target component from the original group
        if(ind_comp!= -1){
            group_axisVector[ind_group].second.removeAt(ind_comp);
        }

        //insert targetComp to the new group
        if(group_axisVector[ind_group].first == GroupsForRegrouping[0])
            group_axisVector[ind_group].second.append(targetComp);
    }
    return group_axisVector;
}

MachineTool GroupingResolver::createMT(QVector<QPair<QString, QVector<QString> > > &group_axisVector, QVector<component> &compVector,
                                       QVector<JointString>& jointStringVector)
{
    //Create machine tool object
    MachineTool MT;

    //initialize links
    for(int ind_group = 0; ind_group < group_axisVector.size(); ind_group++){

        //constructor with link name
        Link link_input(group_axisVector[ind_group].first.toStdString());

        //set STL files for the link
        link_input.setSTLMesh(group_axisVector[ind_group].second, compVector);
        qDebug()<<"Link"<<QString::fromStdString(link_input.getName())<<"contains"<<
                  link_input.m_STLMeshVector.size()<<"components"<<endl;

        //set color for link
        VectorRGBA rgba(0.37647 * (ind_group + 1),0.75294 / (ind_group + 1),
                        0.2 * (ind_group + 1), 1.0);
        link_input.setRGBA(rgba);

        MT.LinkVector.push_back(link_input);
    }

    qDebug()<<"MT contains # links:"<<MT.LinkVector.size()<<endl;


    //initialize joints
    for(int ind_joint = 0; ind_joint < jointStringVector.size(); ind_joint++){
        QString jointName = jointStringVector[ind_joint].name;
        QString parentLink = jointStringVector[ind_joint].link_parent;
        QString childLink = jointStringVector[ind_joint].link_child;
        QString compInChildLink = jointStringVector[ind_joint].compInChildLink;
        Vector3 rpy(0,0,0);
        Joint joint_new(jointName.toStdString(), getJointType(childLink).toStdString(),
                        getJointXYZ(compInChildLink,childLink, compVector), rpy, getJointAxis(childLink),
                        MT.find_link(parentLink.toStdString(), MT.LinkVector),
                        MT.find_link(childLink.toStdString(), MT.LinkVector));
        MT.JointVector.push_back(joint_new);
        joint_new.getParentLink()->ChildLink.append(joint_new.getChildLink());
        //assign child link to the parent link

        joint_new.getChildLink()->ParentLink = joint_new.getParentLink();
        //assign parent link to the child link

        MT.assignLinkType(joint_new);
        //assign Link type to child link of joint

        qDebug()<<"Joint name:"<< QString::fromStdString(joint_new.getName())<<
                  "Type:"<<QString::fromStdString(joint_new.getType())<<
                  "parent link:"<<QString::fromStdString(joint_new.getParentLink()->getName())<<
                  "child link:"<<QString::fromStdString(joint_new.getChildLink()->getName())<<
                  "xyz:"<<joint_new.getOrigin_xyz().x<<joint_new.getOrigin_xyz().y<<
                  joint_new.getOrigin_xyz().z<<endl;
    }

    //create hash table for components
    for(int link_ind = 0; link_ind < MT.LinkVector.size(); ++link_ind){
        for(int mesh_ind = 0; mesh_ind < MT.LinkVector[link_ind].m_STLMeshVector.size(); ++mesh_ind){
            QString componentName = QString(QChar::fromLatin1(MT.LinkVector[link_ind].getLinkType())) + QString::number(mesh_ind + 1);
            MT.componentsHash[componentName] = MT.LinkVector[link_ind].m_STLMeshVector[mesh_ind];
        }
    }

    //find and voxelize base link

    for (QVector<Link>::iterator loop = MT.LinkVector.begin(); loop != MT.LinkVector.end(); loop++){
        if(loop->ParentLink == nullptr)
        {
            MT.baseLink = loop;
            break;
        }
    }

    return MT;
}

