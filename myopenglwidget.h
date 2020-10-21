#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include <creategeometry.h>
#include <createcubes.h>
#include <voxelizer.h>
#include "machinetool.h"
#include <QDir>
#include <TopExp_Explorer.hxx>
#include <TDF_Label.hxx>
#include <Geom_Circle.hxx>
#include <gp_Circ.hxx>
#include <gp_Pnt.hxx>
#include <gp_Ax1.hxx>
#include <BRep_Tool.hxx>
#include <TopoDS_Edge.hxx>
#include <IGESCAFControl_Reader.hxx>
#include <QXmlStreamWriter>
#include "initialgrouper.h"
#include "groupingpreprocessor.h"
#include "groupingvalidator.h"

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram)

class MyOpenGLWidget:public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    MyOpenGLWidget(QWidget *parent = 0);
    ~MyOpenGLWidget();

    static bool isTransparent() { return m_transparent; }
    static void setTransparent(bool t) { m_transparent = t; }

    QSize minimumSizeHint() const override;
    QSize sizeHint() const override;

public slots:
    void setXRotation(int angle);
    void setZRotation(int angle);
    void setXTranslation(int distance);
    void setYTranslation(int distance);
    void setZoom(int distance);
    void cleanup();

signals:
    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    void setupVertexAttribs();
    void drawMTComponents();
    void drawCCPComponents();
    QVector<stl_reader::StlMesh <float, unsigned int>> readSTLFiles(QString mtName);
    QVector<component> readCompSTL(QString mtName, QVector3D mtRotaryAxes);
    QVector<component> getAxisForComp(QVector<component>& compVector, QVector3D& mtRotaryAxes);
    QVector3D findCommonAxis(TopoDS_Shape ashape, QString componentAxis);
    TopoDS_Shape readBRep(QString compName);
    QVector<component> readAxisForComp(QVector<component>& compVector, QXmlStreamReader& Rxml, QVector3D& mtRotaryAxes);
    int indexOfComponent(QVector<component> &compVector,QString compName);
    bool m_core;
    int m_xRot;
    int m_yRot;
    int m_zRot;
    int m_xTran;
    int m_yTran;
    int m_zTran;
    float fov;
    QPoint m_lastPos;
    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_geometryVbo;      //Vertex Buffer Objects
    QOpenGLShaderProgram *m_program;
    int m_projMatrixLoc;
    int m_mvMatrixLoc;
    int m_normalMatrixLoc;
    int m_lightPosLoc;
    int m_colorLoc;
    int m_alphaLoc;
    QMatrix4x4 m_proj;
    QMatrix4x4 m_camera;
    QMatrix4x4 m_world;
    QVector3D m_color;
    static bool m_transparent;

    QStringList m_filepathes;
    CreateGeometry m_geometry;
    CreateCubes m_cubeGemoetry;
    GroupingPreProcessor m_groupingPreProcessor;
    GroupingValidator m_groupingValidator;

    MachineTool MT;
    QString machineToolName;

    bool IsEqual(const double& dX, const double& dY){
        const double dEpsilon = 0.000001;
        return fabs(dX - dY) <= dEpsilon;
    }
};

#endif
