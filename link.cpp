#include "link.h"
#include<iostream>

Link::Link(){}
Link::Link(std::string name):m_name(name){}
//constructor
Link::Link(std::string name, Vector3 origin_xyz,Vector3 origin_rpy, std::string meshfile
           ,VectorRGBA rgba)
    : m_name(name),m_origin_xyz(origin_xyz), m_origin_rpy(origin_rpy),m_meshfile(meshfile), m_rgba(rgba){

    m_TransformMatrix.setToIdentity();
}

std::string Link::getMeshFile()const {

    std::size_t pos = m_meshfile.find("meshes/");
    std::string short_meshfile = m_meshfile.substr(pos+7);   //only return name of the stl file

    return short_meshfile;
}

void Link::setSTLMesh()
{
    try {
        //read STL file for each file
        stl_reader::StlMesh <float, unsigned int> mesh(this->getMeshFile());
        m_STLMesh = mesh;
        std::cout << "Finish setting mesh for " <<this->m_name << std::endl;
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }
}
