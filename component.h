#ifndef COMPONENT_H
#define COMPONENT_H
#include <stl_reader.h>
#include <QString>
#include <QMatrix4x4>


class component
{
public:
    component();
    component(QString componentName);
    component(QString componentName, stl_reader::StlMesh<float, unsigned int>mesh);
    component(QString componentName, stl_reader::StlMesh<float, unsigned int>mesh, QMatrix4x4 transformMatrix);
    QString getName(){return m_name;}
    void setName(QString componentName){m_name = componentName;}
    stl_reader::StlMesh<float, unsigned int>& getNonOffsetMesh(){return m_nonOffestMesh;}
    stl_reader::StlMesh<float, unsigned int>& getOffsetMesh(){return m_offestMesh;}
    void setOffsetMesh(stl_reader::StlMesh<float, unsigned int> offsetMesh){m_offestMesh = offsetMesh;}
    void setContainsOffsetMesh(){m_containsOffsetMesh = true;}
    bool containsOffsetMesh(){return m_containsOffsetMesh;}

private:
    QString m_name;
    stl_reader::StlMesh <float, unsigned int> m_nonOffestMesh;
    stl_reader::StlMesh <float, unsigned int> m_offestMesh;
    bool m_containsOffsetMesh = false;
    QMatrix4x4 m_transformMatrix;
};

#endif // COMPONENT_H
