#include "machinetool.h"
#include "tinyxml2.h"
#include <QVector>
#include <cassert>


using namespace tinyxml2;
using namespace std;

MachineTool::MachineTool(){}


Link* MachineTool::find_link(std::string linkName, QVector<Link> &myVector)
{               //The function to find the link for joint and return a pointer

    for (QVector<Link>::iterator loop = myVector.begin();loop != myVector.end(); loop++)
    {
        if(loop->getName()== linkName)
            return loop;

        if(loop == myVector.end() && myVector.end()->getName() != linkName)//assert  can not find the name
            assert (false && "Can not find this link in the link vector");
    }

    return nullptr;
}

void MachineTool::assignLinkType(Joint &joint)
{
    if(joint.getType() == "revolute"){
        if(abs(joint.getAxis().x - 1.0) < 0.0001)
            joint.getChildLink()->setLinkType('A');
        if(abs(joint.getAxis().y - 1.0) < 0.0001)
            joint.getChildLink()->setLinkType('B');
        if(abs(joint.getAxis().z - 1.0) < 0.0001)
            joint.getChildLink()->setLinkType('C');
    }

    if(joint.getType() == "prismatic"){
        if(joint.getAxis().x == 1.0)
            joint.getChildLink()->setLinkType('X');
        if(joint.getAxis().y == 1.0)
            joint.getChildLink()->setLinkType('Y');
        if(joint.getAxis().z == 1.0)
            joint.getChildLink()->setLinkType('Z');
    }

}

int MachineTool::readURDF(const char* filename){
    XMLDocument doc;
    int res = doc.LoadFile(filename);

    if (res != 0)
    {
        qDebug()<< "load xml file failed" << endl;
        return res;
    }
    XMLElement *MT = doc.RootElement();
    const XMLAttribute *nameOfMT = MT->FirstAttribute();
    qDebug() << "Machine Tool: "<< nameOfMT->Name() << ":" << nameOfMT->Value() << endl << endl;
    m_name = nameOfMT->Value();

    XMLElement *link_count = MT->FirstChildElement("link");

    QList<double> mtBoundingBox_X_min;
    QList<double> mtBoundingBox_X_max;
    QList<double> mtBoundingBox_Y_min;
    QList<double> mtBoundingBox_Y_max;
    QList<double> mtBoundingBox_Z_min;
    QList<double> mtBoundingBox_Z_max;


    while (link_count)
    {
        const XMLAttribute *nameOfLink = link_count->FirstAttribute();
        XMLElement *visual = link_count->FirstChildElement("visual");

        if (visual == nullptr)                                          //if there is no visual element
        {
            Link link_reading(nameOfLink->Value());
            LinkVector.push_back(link_reading);                         //push into the link vector

            link_count = link_count->NextSiblingElement("link");        //move to next link
            continue;
        }

        XMLElement *origin = visual->FirstChildElement("origin");
        const XMLAttribute *xyz = origin->FirstAttribute();
        const XMLAttribute *rpy = xyz->Next();

        Vector3 xyz_input;
        xyz_input.init(xyz->Value());

        Vector3 rpy_input;
        rpy_input.init(rpy->Value());

        XMLElement *geometry = visual->FirstChildElement("geometry");
        XMLElement *mesh = geometry->FirstChildElement("mesh");
        const XMLAttribute *fileName = mesh->FirstAttribute();

        XMLElement *material = visual->FirstChildElement("material");
        XMLElement *color = material->FirstChildElement("color");
        const XMLAttribute *rgba = color->FirstAttribute();

        VectorRGBA rgba_input;
        rgba_input.init(rgba->Value());

        Link link_reading(nameOfLink->Value(), xyz_input, rpy_input,
                          fileName->Value(), rgba_input);

        // read STL and store in each link
        link_reading.setSTLMesh();
        LinkVector.push_back(link_reading);
        //push link into the vector

        //append all bounding box coordinates of each link
        QVector<stl_reader::StlMesh <float, unsigned int>>& STLMeshVector = link_reading.m_STLMeshVector;
        for(int i = 0; i < STLMeshVector.size(); i++){
        mtBoundingBox_X_min.append(STLMeshVector[i].getBoundingBox_X_min());
        mtBoundingBox_X_max.append(STLMeshVector[i].getBoundingBox_X_max());
        mtBoundingBox_Y_min.append(STLMeshVector[i].getBoundingBox_Y_min());
        mtBoundingBox_Y_max.append(STLMeshVector[i].getBoundingBox_Y_max());
        mtBoundingBox_Z_min.append(STLMeshVector[i].getBoundingBox_Z_min());
        mtBoundingBox_Z_max.append(STLMeshVector[i].getBoundingBox_Z_max());
        }

        link_count = link_count->NextSiblingElement("link");                  //move to next sibling element
    }

    //Get bounding box coordinates for whole machine tool
    qSort(mtBoundingBox_X_min.begin(), mtBoundingBox_X_min.end());
    qSort(mtBoundingBox_X_max.begin(), mtBoundingBox_X_max.end());
    qSort(mtBoundingBox_Y_min.begin(), mtBoundingBox_Y_min.end());
    qSort(mtBoundingBox_Y_max.begin(), mtBoundingBox_Y_max.end());
    qSort(mtBoundingBox_Z_min.begin(), mtBoundingBox_Z_min.end());
    qSort(mtBoundingBox_Z_max.begin(), mtBoundingBox_Z_max.end());
    bounding_x_min = mtBoundingBox_X_min.first();
    bounding_x_max = mtBoundingBox_X_max.last();
    bounding_y_min = mtBoundingBox_Y_min.first();
    bounding_y_max = mtBoundingBox_Y_max.last();
    bounding_z_min = mtBoundingBox_Z_min.first();
    bounding_z_max = mtBoundingBox_Z_max.last();

    XMLElement *joint = MT->FirstChildElement("joint");

    while (joint)
    {
        const XMLAttribute *nameOfJoint = joint->FirstAttribute();
        const XMLAttribute *typeOfJoint = joint->FirstAttribute()->Next();
        XMLElement *parent_link = joint->FirstChildElement("parent");  //parent link
        XMLElement *child_link = joint->FirstChildElement("child");    //child link
        const XMLAttribute *parent = parent_link->FirstAttribute();
        const XMLAttribute *child = child_link->FirstAttribute();

        XMLElement *origin = joint->FirstChildElement("origin");
        if (origin == nullptr)                                              //for some joint without origin element
        {
            Joint joint_reading(nameOfJoint->Value(), typeOfJoint->Value(),
                                find_link(parent->Value(),LinkVector),find_link(child->Value(),LinkVector));
            JointVector.push_back(joint_reading);                           //push into the joint vector

            joint = joint->NextSiblingElement("joint");
            continue;
        }
        const XMLAttribute *xyz = origin->FirstAttribute();
        const XMLAttribute *rpy = xyz->Next();

        Vector3 xyz_input;
        xyz_input.init(xyz->Value());

        xyz_input.x = xyz_input.x;
        xyz_input.y = xyz_input.y;
        xyz_input.z = xyz_input.z;

        Vector3 rpy_input;
        rpy_input.init(rpy->Value());

        XMLElement *axis = joint->FirstChildElement("axis");           //axis
        const XMLAttribute *axisAttribute = axis->FirstAttribute();
        //        XMLElement *limit = joint->FirstChildElement("limit");
        //        const XMLAttribute *limit_lower = limit->FirstAttribute();   //lower limit do it later
        //        const XMLAttribute *limit_upper = limit_lower->Next();       //upper limit


        //        double double_limit_lower = strToDouble(limit_lower->Value());
        //        double double_limit_upper = strToDouble(limit_upper->Value());

        Vector3 axis_input;
        axis_input.init(axisAttribute->Value());

        Joint joint_reading(nameOfJoint->Value(), typeOfJoint->Value(), xyz_input, rpy_input, axis_input,
                            find_link(parent->Value(),LinkVector),find_link(child->Value(),LinkVector));
        JointVector.push_back(joint_reading);                           //push into the joint vector

        joint_reading.getParentLink()->ChildLink = joint_reading.getChildLink();
        //assign child link to the parent link

        joint_reading.getChildLink()->ParentLink = joint_reading.getParentLink();
        //assign parent link to the child link

        assignLinkType(joint_reading);
        //assign Link type to child link of joint

        joint = joint->NextSiblingElement("joint");  //move to next sibling element
    }

    qDebug()<<"Finish creating kinematic chain by reading URDF"<<endl;
    return 0;
}

QDebug operator<<(QDebug stream, const MachineTool &MT)
{
    stream<< "Machine Tool: "<< MT.m_name<<endl<<endl;

    for (QVector<const Link>::iterator loop = MT.LinkVector.begin();loop != MT.LinkVector.end(); loop++)        //print out all links information
    {
        stream<<"Name of the link: "<<QString::fromStdString(loop->getName())<<endl;
        stream<<"xyz:  "<<loop->getOrigin_xyz().x<<" "<<loop->getOrigin_xyz().y<<" "<<loop->getOrigin_xyz().z<<endl;
        stream<<"rpy:  "<<loop->getOrigin_rpy().x<<" "<<loop->getOrigin_rpy().y<<" "<<loop->getOrigin_rpy().z<<endl;
        stream<<"STL file Names:"<<loop->getMeshFile()<<endl<<endl;
    }
    return stream;
}
