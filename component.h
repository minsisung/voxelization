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
    // what rotary axis machine tool contain
    QVector3D m_mtRotaryAxes;

    QString getName(){return m_name;}
    void setName(QString componentName){m_name = componentName;}
    stl_reader::StlMesh<float, unsigned int>& getNonOffsetMesh(){return m_nonOffestMesh;}
    stl_reader::StlMesh<float, unsigned int>& getOffsetMesh(){return m_offestMesh;}
    void setOffsetMesh(stl_reader::StlMesh<float, unsigned int> offsetMesh){m_offestMesh = offsetMesh;}
    void setContainsOffsetMesh(){m_containsOffsetMesh = true;}
    bool containsOffsetMesh(){return m_containsOffsetMesh;}
    void setRotaryAxisPoint1(float x, float y, float z){rotaryAxisPoint1.setX(x);
                                                        rotaryAxisPoint1.setY(y);
                                                                                                        rotaryAxisPoint1.setZ(z);}
    void setRotaryAxisPoint2(float x, float y, float z){rotaryAxisPoint2.setX(x);
                                                        rotaryAxisPoint2.setY(y);
                                                                                                        rotaryAxisPoint2.setZ(z);}

    QVector3D getRotaryAxisPoint1(){return rotaryAxisPoint1;}
    QVector3D getRotaryAxisPoint2(){return rotaryAxisPoint2;}

    void setContainsRotaryAxis1(){m_containsRotaryAxis1 = true;}
    bool containsRotaryAxis1(){return m_containsRotaryAxis1;}
    void setContainsRotaryAxis2(){m_containsRotaryAxis2 = true;}
    bool containsRotaryAxis2(){return m_containsRotaryAxis2;}



private:
    QString m_name;
    stl_reader::StlMesh <float, unsigned int> m_nonOffestMesh;
    stl_reader::StlMesh <float, unsigned int> m_offestMesh;
    bool m_containsOffsetMesh = false;
    QMatrix4x4 m_transformMatrix;
    QVector3D rotaryAxisPoint1;
    QVector3D rotaryAxisPoint2;
    bool m_containsRotaryAxis1 = false;
    bool m_containsRotaryAxis2 = false;
};

#endif // COMPONENT_H
