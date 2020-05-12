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
    void drawComponents();

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
};

#endif
