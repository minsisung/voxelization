#include "urdfexporter.h"

UrdfExporter::UrdfExporter(MachineTool& MT):
    m_MT(MT)
{
    m_MTName = m_MT.m_name;
}

void UrdfExporter::start()
{
    exportXML();
    exportSTL();
}

void UrdfExporter::exportSTL()
{    
    for(int ind_link = 0; ind_link < m_MT.LinkVector.size(); ind_link++){
        Link& link = m_MT.LinkVector[ind_link];
        QString filename = QString::fromStdString(link.getName()) + ".STL";
        QString directoryName = QDir::current().path() + "/" + m_MTName+ "/mesh";
        QDir dir(directoryName);
        if (!dir.exists()){
            dir.mkpath(directoryName);
        }
        QString filePath = directoryName + '/' + filename;
        QFile file(filePath);
        file.open(QIODevice::WriteOnly | QIODevice::Text);
        QTextStream out(&file);
        out.setRealNumberNotation(QTextStream::ScientificNotation);
        out << "solid "<<QString::fromStdString(link.getName())<< endl;

        //default offset vector are x = 0, y = 0, z = 0
        Vector3 offsetVector(0.0, 0.0, 0.0);

        //if the link has parent, update offsetvector by the joint that connect parent link and the current link
        if(link.ParentLink!= nullptr)
            offsetVector = findJoint(QString::fromStdString(link.ParentLink->getName()),
                                     QString::fromStdString(link.getName()), m_MT.JointVector)->getOrigin_xyz();

        qDebug()<<"Link name:"<<QString::fromStdString(link.getName());
        qDebug()<<"offsetVector"<<offsetVector.x<<" "<<offsetVector.y<<" "<<offsetVector.z;

        //write all components into the link
        for(int ind_mesh = 0; ind_mesh < link.m_MeshNameVector.size(); ind_mesh++){
            QString path = QDir::current().path() + "/" + m_MTName + "/groupingValidation";
            stl_reader::StlMesh <float, unsigned int> componentMesh;
            componentMesh.read_file((path + "/" + link.getMeshNameAt(ind_mesh)
                                     + ".STL").toStdString());

            STLFacetOut(out, componentMesh, offsetVector);
        }
        out << "endsolid" << endl;
        file.close();
        qDebug()<<QString::fromStdString(link.getName())<<"Link contains:"<<link.m_MeshNameVector;
        qDebug()<<"Finished exporter stl file for"<<QString::fromStdString(link.getName());
    }
}

void UrdfExporter::STLFacetOut(QTextStream &out, stl_reader::StlMesh <float, unsigned int>& mesh, Vector3& offsetVector)
{
    for(size_t itri = 0; itri < mesh.num_tris(); ++itri) {
        const float* n = mesh.tri_normal(itri);
        out << "facet normal " << n[0] << " " << n[1] << " " << n[2] << endl;
        out << "   outer loop" << endl;

        for(size_t icorner = 0; icorner < 3; ++icorner) {
            const float* c = mesh.tri_corner_coords (itri, icorner);
            out << "      vertex " << c[0] - offsetVector.x<< " " << c[1] - offsetVector.y<< " " << c[2] - offsetVector.z<< endl;
        }
        out << "   endloop" << endl;
        out << "endfacet" << endl;
    }
}

void UrdfExporter::exportXML()
{
    //open file for writing xml
    QString filename = m_MTName + ".urdf";
    QString directoryName = QDir::current().path() + "/" + m_MTName+ "/urdf";
    QDir dir(directoryName);
    if (!dir.exists()){
        dir.mkpath(directoryName);
    }
    QString filePath = directoryName + '/' + filename;
    QFile file(filePath);
    file.open(QIODevice::WriteOnly);
    QXmlStreamWriter xmlWriter(&file);
    xmlWriter.setAutoFormatting(true);
    xmlWriter.writeStartDocument();
    xmlWriter.writeStartElement("robot");
    xmlWriter.writeAttribute("name" ,m_MTName);

    //export base link
    writeLink(xmlWriter, *m_MT.baseLink);

    for(int Number = 0; Number < m_MT.baseLink->ChildLink.size(); Number++){
        Link* currentLink = m_MT.baseLink->ChildLink[Number];
        Joint* prev_joint = nullptr;
        while(currentLink != nullptr){
            qDebug()<<"Current Link:"<<QString::fromStdString(currentLink->getName());
            writeLink(xmlWriter, *currentLink);
            Link* parentLink = currentLink->ParentLink;
            Joint* joint = findJoint(QString::fromStdString(parentLink->getName()),
                                     QString::fromStdString(currentLink->getName()), m_MT.JointVector);

            writeJoint(xmlWriter, *joint, prev_joint);

            //update prev_joint
            prev_joint = joint;

            if(!currentLink->ChildLink.isEmpty()){
                currentLink = currentLink->ChildLink[0];
            }else{
                break;
            }
        }
    }

    xmlWriter.writeEndElement();//robot

    file.close();
}

void UrdfExporter::writeLink(QXmlStreamWriter& xmlWriter, Link &link)
{
    QString linkName = QString::fromStdString(link.getName());

    xmlWriter.writeStartElement("link");
    xmlWriter.writeAttribute("name" ,linkName);
    xmlWriter.writeStartElement("visual");
    xmlWriter.writeStartElement("origin");
    xmlWriter.writeAttribute("xyz" ,QString::number(link.getOrigin_xyz().x) + " " +  QString::number(link.getOrigin_xyz().y)
                             + " " +  QString::number(link.getOrigin_xyz().z));
    xmlWriter.writeAttribute("rpy" ,QString::number(link.getOrigin_rpy().x) + " " +  QString::number(link.getOrigin_rpy().y)
                             + " " +  QString::number(link.getOrigin_rpy().z));
    xmlWriter.writeEndElement();//origin
    xmlWriter.writeStartElement("geometry");
    xmlWriter.writeStartElement("mesh");
    xmlWriter.writeAttribute("filename" ,"package://" + m_MTName +
                             "/meshes/" + linkName + ".STL");
    xmlWriter.writeEndElement();//mesh
    xmlWriter.writeEndElement();//geometry
    xmlWriter.writeStartElement("material");
    xmlWriter.writeAttribute("name", "");
    xmlWriter.writeStartElement("color");
    xmlWriter.writeAttribute("rgba" ,"0.752941176470588 0.752941176470588 0.752941176470588 1");
    xmlWriter.writeEndElement();//color
    xmlWriter.writeEndElement();//material
    xmlWriter.writeEndElement();//visual
    xmlWriter.writeEndElement();//link
}

void UrdfExporter::writeJoint(QXmlStreamWriter& xmlWriter, Joint &joint, Joint* prev_joint)
{
    xmlWriter.writeStartElement("joint");
    xmlWriter.writeAttribute("name", QString::fromStdString(joint.getName()));
    xmlWriter.writeAttribute("type", QString::fromStdString(joint.getType()));
    xmlWriter.writeStartElement("origin");


    //if previous joint is a rotary joint, xyz of the current joint need to
    //minus by the xyz of prvious joint
    if(prev_joint!=nullptr && prev_joint->getType() == "revolute"){
        xmlWriter.writeAttribute("xyz", QString::number(joint.getOrigin_xyz().x - prev_joint->getOrigin_xyz().x) + " "
                                 +  QString::number(joint.getOrigin_xyz().y - prev_joint->getOrigin_xyz().y)
                                 + " " +  QString::number(joint.getOrigin_xyz().z - prev_joint->getOrigin_xyz().z));
    }else{
        xmlWriter.writeAttribute("xyz", QString::number(joint.getOrigin_xyz().x) + " " +  QString::number(joint.getOrigin_xyz().y)
                                 + " " +  QString::number(joint.getOrigin_xyz().z));
    }

    xmlWriter.writeAttribute("rpy", QString::number(joint.getOrigin_rpy().x) + " " +  QString::number(joint.getOrigin_rpy().y)
                             + " " +  QString::number(joint.getOrigin_rpy().z));
    xmlWriter.writeEndElement();//origin
    xmlWriter.writeStartElement("parent");
    xmlWriter.writeAttribute("link", QString::fromStdString(joint.getParentLink()->getName()));
    xmlWriter.writeEndElement();//parent
    xmlWriter.writeStartElement("child");
    xmlWriter.writeAttribute("link", QString::fromStdString(joint.getChildLink()->getName()));
    xmlWriter.writeEndElement();//child
    xmlWriter.writeStartElement("axis");
    xmlWriter.writeAttribute("xyz" ,QString::number(joint.getAxis().x) + " " +  QString::number(joint.getAxis().y)
                             + " " +  QString::number(joint.getAxis().z));
    xmlWriter.writeEndElement();//axis
    xmlWriter.writeStartElement("limit");
    xmlWriter.writeAttribute("lower" ,QString::number(joint.getLowerLimit()));
    xmlWriter.writeAttribute("upper" ,QString::number(joint.getUpperLimit()));
    xmlWriter.writeAttribute("effort" ,QString::number(1));
    xmlWriter.writeAttribute("velocity" ,QString::number(1));
    xmlWriter.writeEndElement();//limit
    xmlWriter.writeEndElement();//joint
}

Joint* UrdfExporter::findJoint(QString parentLinkName, QString childLinkName, QVector<Joint>& JointVector)
{
    for(int ind_joint = 0; ind_joint < JointVector.size(); ind_joint++){
        if(QString::fromStdString(JointVector[ind_joint].getParentLink()->getName()) == parentLinkName &&
                QString::fromStdString(JointVector[ind_joint].getChildLink()->getName()) == childLinkName)
            return &JointVector[ind_joint];
    }
    return nullptr;
}


