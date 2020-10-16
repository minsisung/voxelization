#include "component.h"

component::component(){}
component::component(QString componentName):m_name(componentName){}
component::component(QString componentName, stl_reader::StlMesh<float, unsigned int>mesh):
    m_name(componentName),m_nonOffestMesh(mesh){}
component::component(QString componentName, stl_reader::StlMesh<float, unsigned int>mesh,
                     QMatrix4x4 transformMatrix): m_name(componentName),
    m_nonOffestMesh(mesh), m_transformMatrix(transformMatrix){}

bool operator== (const component &c1, const component &c2)
{
    return (c1.m_name == c2.m_name);
}
