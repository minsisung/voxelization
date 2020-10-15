#include "link.h"
#include<iostream>
#include <QDir>

Link::Link(){}
Link::Link(std::string name):m_name(name){}
//constructor
Link::Link(std::string name, Vector3 origin_xyz,Vector3 origin_rpy, std::string meshfile
           ,VectorRGBA rgba)
    : m_name(name),m_origin_xyz(origin_xyz), m_origin_rpy(origin_rpy),m_meshfile(meshfile), m_rgba(rgba){

    m_TransformMatrix.setToIdentity();
}

QStringList Link::getMeshFile()const {

    std::size_t pos = m_meshfile.find("meshes/");
    std::size_t pos2 = m_meshfile.find(".STL");
    std::string short_meshfile = m_meshfile.substr(pos+7 , pos2-(pos+7));   //only return name of the stl file

    //store all .stl with the same first name
    QDir directory = QDir::current();
    QStringList STLList = directory.entryList(QStringList() << "*.STL" << "*.stl",QDir::Files);
    QStringList meshList = STLList.filter(QString::fromStdString(short_meshfile), Qt::CaseInsensitive);

    return meshList;
}

void Link::setSTLMesh()
{
    //store all mesh in a vector for this link from meshlist
    QStringList meshList = this->getMeshFile();

    for(int i = 0; i< meshList.size(); i++)
        try {
        //read STL file for each file
        stl_reader::StlMesh <float, unsigned int> mesh(meshList[i].toStdString());

        m_STLMeshVector.append(mesh);
        qDebug() << "Finish setting mesh for " <<meshList[i]<< endl;
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }

    //setup MTVoxelIndicesListVector depends on the size of meshlist
    QVector<QList<QVector3D>> correctsize_MTVoxelIndicesList;
    QVector<QVector<QList<QVector3D>>> correctsize_MTVoxelIndicesListVector(meshList.size(), correctsize_MTVoxelIndicesList);
    MTVoxelIndicesListVector = correctsize_MTVoxelIndicesListVector;
}

void Link::setBoundingBoxIndex(int x_min_index, int x_max_index, int y_min_index, int y_max_index,
                               int z_min_index, int z_max_index)
{
    bounding_x_min_index = x_min_index;
    bounding_x_max_index = x_max_index;
    bounding_y_min_index = y_min_index;
    bounding_y_max_index = y_max_index;
    bounding_z_min_index = z_min_index;
    bounding_z_max_index = z_max_index;
}
