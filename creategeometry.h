#ifndef CREATEGEOMETRY_H
#define CREATEGEOMETRY_H

#include <qopengl.h>
#include <QVector>

class CreateGeometry
{
public:
    CreateGeometry();

    const GLfloat *constData() const { return m_data.constData(); }
    int totalCount() const { return m_totalCount; }
    int totocalVertexCount() const { return m_totalCount / 6; }

    float getX_avg() const {return (max_x+min_x)/2;}
    float getY_avg() const {return (max_y+min_y)/2;}
    float getZ_avg() const {return (max_z+min_z)/2;}


    void readSTL(QString filepath);
private:
    QVector<GLfloat> m_data;
    int m_totalCount;
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
};

#endif // CREATEGEOMETRY_H
