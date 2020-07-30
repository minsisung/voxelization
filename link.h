#ifndef LINK_H
#define LINK_H
#include <string>
#include <Vector3.h>
#include <QMatrix4x4>
#include <stl_reader.h>
#include "voxel.h"

class Link
{
private:
    std::string m_name;
    Vector3 m_origin_xyz;
    Vector3 m_origin_rpy;
    std::string m_meshfile;
    VectorRGBA m_rgba;
    stl_reader::StlMesh <float, unsigned int> m_STLMesh;
    char linkType = 'b';

    int bounding_x_min_index;
    int bounding_x_max_index;
    int bounding_y_min_index;
    int bounding_y_max_index;
    int bounding_z_min_index;
    int bounding_z_max_index;

    float m_lower_limit;
    float m_upper_limit;


public:
    Link();             //constructor
    Link(std::string name);
    Link(std::string name, Vector3 origin_xyz,Vector3 origin_rpy, std::string meshfile
         ,VectorRGBA rgba);
    ~Link(){}

    Link *ParentLink = nullptr;
    Link *ChildLink = nullptr;

    Vector3 getOrigin_xyz() const{return m_origin_xyz;}
    Vector3 getOrigin_rpy() const{return m_origin_rpy;}
    VectorRGBA getRGBA() const{return m_rgba;}
    std::string getMeshFile()const;
    std::string getName() const{return m_name;}
    int numberOfVertex = 0;
    QMatrix4x4 m_TransformMatrix;
    Link *m_parent_link;
    Link *m_child_link;
    void setSTLMesh();
    stl_reader::StlMesh <float, unsigned int>  getSTLMesh(){return m_STLMesh;}
    char getLinkType(){return linkType;}
    void setLinkType(char setLinkType){linkType = setLinkType;}

    void setLowerLimit(float lowerLimit){m_lower_limit = lowerLimit;}
    void setUpperLimit(float upperLimit){m_upper_limit = upperLimit;}
    float getLowerLimit(){return m_lower_limit;}
    float getUpperLimit(){return m_upper_limit;}
    int get_x_min_index(){return bounding_x_min_index;}
    int get_x_max_index(){return bounding_x_max_index;}
    int get_y_min_index(){return bounding_y_min_index;}
    int get_y_max_index(){return bounding_y_max_index;}
    int get_z_min_index(){return bounding_z_min_index;}
    int get_z_max_index(){return bounding_z_max_index;}
    void setBoundingBoxIndex(int x_min_index, int x_max_index, int y_min_index, int y_max_index,
                             int z_min_index, int z_max_index);

    QVector < QVector < QVector< Voxel > > > linkVoxelspace;
    QList<QVector3D> MTOutterVoxelIndicesList;
    QList<QVector3D> MTInnerVoxelIndicesList;

    QVector<QList<QVector3D>> MTOutterVoxelIndicesListVector;
};

#endif // LINK_H
