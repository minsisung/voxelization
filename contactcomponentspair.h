#ifndef CONTACTCOMPONENTSPAIR_H
#define CONTACTCOMPONENTSPAIR_H
#include <stl_reader.h>
#include <QVector>
#include <QMatrix4x4>
#include "component.h"


class contactComponentsPair
{
public:
    contactComponentsPair();
    QString getName(){return name;}
    void setName(QString name_input){name =name_input;}
    void collided_Positive_X(){positive_X = true;}
    void collided_Negative_X(){negative_X = true;}
    void collided_Positive_Y(){positive_Y = true;}
    void collided_Negative_Y(){negative_Y = true;}
    void collided_Positive_Z(){positive_Z = true;}
    void collided_Negative_Z(){negative_Z = true;}
    void not_collided_FirstAxis(){rotary_first = false;}
    void not_collided_SecondAxis(){rotary_second = false;}
    bool isCollided_Positive_X(){return positive_X;}
    bool isCollided_Negative_X(){return negative_X;}
    bool isCollided_Positive_Y(){return positive_Y;}
    bool isCollided_Negative_Y(){return negative_Y;}
    bool isCollided_Positive_Z(){return positive_Z;}
    bool isCollided_Negative_Z(){return negative_Z;}
    bool isCollided_FirstAxis(){return rotary_first;}
    bool isCollided_SecondAxis(){return rotary_second;}

    stl_reader::StlMesh <float, unsigned int> getFirstMesh(){return firstMesh;}
    void setFirstMesh(stl_reader::StlMesh <float, unsigned int> mesh){firstMesh = mesh;}
    stl_reader::StlMesh <float, unsigned int> getSecondMesh(){return secondMesh;}
    void setSecondMesh(stl_reader::StlMesh <float, unsigned int> mesh){secondMesh = mesh;}
    void setFirstMeshName(QString name){firstMeshName = name;}
    QString getFirstMeshName(){return firstMeshName;}
    void setSecondMeshName(QString name){secondMeshName = name;}
    QString getSecondMeshName(){return secondMeshName;}

    void setFirstComp(component comp){firstComp = comp;}
    void setSecondComp(component comp){secondComp = comp;}
    component getFirstComp(){return firstComp;}
    component getSecondComp(){return secondComp;}
    QMatrix4x4 firstTransformMatrix;
    QMatrix4x4 secondTransformMatrix;
    void setContainsCommonRotaryAxis1(){m_containsCommonRotaryAxis1 = true;}
    bool containsCommonRotaryAxis1(){return m_containsCommonRotaryAxis1;}
    void setContainsCommonRotaryAxis2(){m_containsCommonRotaryAxis2 = true;}
    bool containsCommonRotaryAxis2(){return m_containsCommonRotaryAxis2;}

    QString firstAxis;
    QString secondAxis;

private:
    QString name;
    //    QVector<stl_reader::StlMesh <float, unsigned int>> CCP;
    stl_reader::StlMesh <float, unsigned int> firstMesh;
    stl_reader::StlMesh <float, unsigned int> secondMesh;

    component firstComp;
    component secondComp;
    //Relative motion between components
    bool positive_X = false;
    bool negative_X = false;
    bool positive_Y = false;
    bool negative_Y = false;
    bool positive_Z = false;
    bool negative_Z = false;
    bool rotary_first = true;
    bool rotary_second = true;
    QString firstMeshName;
    QString secondMeshName;
    bool m_containsCommonRotaryAxis1 = false;
    bool m_containsCommonRotaryAxis2 = false;
};

#endif // CONTACTCOMPONENTSPAIR_H
