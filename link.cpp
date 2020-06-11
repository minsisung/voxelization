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
