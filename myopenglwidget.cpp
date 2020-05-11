#include "myopenglwidget.h"
#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <math.h>
#include <shaders.h>
#include <QFileDialog>

bool MyOpenGLWidget::m_transparent = true;

MyOpenGLWidget::MyOpenGLWidget(QWidget *parent)
    : QOpenGLWidget(parent),
      m_xRot(0),m_yRot(0),m_zRot(0),
      m_xTran(0), m_yTran(0), m_zTran(-4500), //original camera position
      fov(45.0f),
      m_program(nullptr)
{
    m_core = QSurfaceFormat::defaultFormat().profile() == QSurfaceFormat::CoreProfile;
    m_filepath1 = QFileDialog::getOpenFileName(this,
                                              tr("Open first 3D Model"), "/home/",tr("3D Model Files (*.stl)"));

    m_filepath2 = QFileDialog::getOpenFileName(this,
                                              tr("Open second 3D Model"), "/home/",tr("3D Model Files (*.stl)"));
}

MyOpenGLWidget::~MyOpenGLWidget()
{
    cleanup();
}

QSize MyOpenGLWidget::minimumSizeHint() const
{
    return QSize(100, 100);
}

QSize MyOpenGLWidget::sizeHint() const
{
    return QSize(400, 400);
}

static void qNormalizeAngle(int &angle)
{
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360 * 16)
        angle -= 360 * 16;
}

void MyOpenGLWidget::setXRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != m_xRot) {
        m_xRot = angle;
        emit xRotationChanged(angle);
        update();
    }
}

void MyOpenGLWidget::setZRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != m_zRot) {
        m_zRot = angle;
        emit zRotationChanged(angle);
        update();
    }
}

void MyOpenGLWidget::setXTranslation(int distance)
{
    if (distance != m_xTran) {
        m_xTran = distance;
        update();
    }
}

void MyOpenGLWidget::setYTranslation(int distance)
{
    if (distance != m_yTran) {
        m_yTran = distance;
        update();
    }
}

void MyOpenGLWidget::setZoom(int yoffset)
{
    if(fov >= 1.0f && fov <= 90.0f)
        fov -= yoffset;
    if(fov <= 1.0f)
        fov = 1.0f;
    if(fov > 90.0f)
        fov = 90.0f;

    resizeGL(MyOpenGLWidget::width(),MyOpenGLWidget::height());
    update();
}

void MyOpenGLWidget::cleanup()
{
    if (m_program == nullptr)
        return;
    makeCurrent();
    m_geometryVbo.destroy();
    delete m_program;
    m_program = nullptr;
    doneCurrent();
}

void MyOpenGLWidget::initializeGL()
{
    connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &MyOpenGLWidget::cleanup);

    initializeOpenGLFunctions();
    glClearColor(0.9f, 0.9f, 0.9f, m_transparent ? 0 : 1);

    //    m_geometry.readSTL(m_filepath);
    m_cubeGemoetry.createVoxelspace(6000.0f,5.0f,m_filepath1, m_filepath2);

    m_program = new QOpenGLShaderProgram;
    m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, m_core ? vertexShaderSourceCore : vertexShaderSource);
    m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, m_core ? fragmentShaderSourceCore : fragmentShaderSource);
    m_program->bindAttributeLocation("vertex", 0);
    m_program->bindAttributeLocation("normal", 1);
    m_program->link();

    m_program->bind();
    m_projMatrixLoc = m_program->uniformLocation("projMatrix");
    m_mvMatrixLoc = m_program->uniformLocation("mvMatrix");
    m_normalMatrixLoc = m_program->uniformLocation("normalMatrix");
    m_lightPosLoc = m_program->uniformLocation("lightPos");
    m_colorLoc = m_program->uniformLocation("color");
    m_alphaLoc = m_program->uniformLocation("alpha");

    //    m_vao.create();
    //    QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);
    //To use a VAO, all you have to do is bind the VAO

    // Setup our vertex buffer object.
    m_geometryVbo.create();
    //Create a buffer for dynamic allocation later

    m_geometryVbo.bind();
    //Bind the buffer so that it is the current active buffer

    m_geometryVbo.allocate(m_cubeGemoetry.constData(), m_cubeGemoetry.totalCount() * sizeof(GLfloat));
    //Allocate and initialize the information

    m_geometryVbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    //set usage pattern for the data is changed a lot and used many times.

    // Store the vertex attribute bindings for the program.
    setupVertexAttribs();

    m_camera.setToIdentity();
    m_camera.translate(m_xTran,m_yTran, m_zTran);

    // Light position is fixed.
    m_program->setUniformValue(m_lightPosLoc, QVector3D(-2000, 2000, 2000));
    m_program->setUniformValue(m_colorLoc, m_color);
    m_program->setUniformValue(m_alphaLoc, 1.0f);

    m_program->release();
}

void MyOpenGLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    //    glEnable (GL_BLEND);
    //    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    m_world.setToIdentity();
    m_world.rotate(-90.0f, 1, 0, 0);        //Transform to intuitive machine tool coordinate
    m_world.rotate(m_xRot / 16.0f, 1, 0, 0);
    m_world.rotate(m_zRot / 16.0f, 0, 0, 1);

    m_camera.setToIdentity();
    m_camera.translate(m_xTran,m_yTran,m_zTran) ;

    //    QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);
    m_program->bind();
    m_program->setUniformValue(m_projMatrixLoc, m_proj);
    m_program->setUniformValue(m_mvMatrixLoc, m_camera * m_world);
    QMatrix3x3 normalMatrix = m_world.normalMatrix();
    m_program->setUniformValue(m_normalMatrixLoc, normalMatrix);

    //set color for model
    m_color = QVector3D(0.752941176470588f, 0.752941176470588f, 0.752941176470588f);
    m_program->setUniformValue(m_colorLoc, m_color);

    //  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDrawArrays(GL_TRIANGLES, 0, m_cubeGemoetry.totalCount());

    m_program->setUniformValue(m_mvMatrixLoc, m_camera * m_world);

    //Creating coordinate system
    //x axis
    glBegin( GL_LINES );
    glVertex3f( 0., 0., 0. );
    glVertex3f( 600, 0., 0. );

    // arrow
    glVertex3f(600, 0.0f, 0.0f);
    glVertex3f(500, 100, 0.0f);

    glVertex3f(600, 0.0f, 0.0f);
    glVertex3f(500, -100, 0.0f);
    glEnd();
    glFlush();

    //y axis
    glBegin( GL_LINES );
    glVertex3f( 0., 0., 0. );
    glVertex3f( 0., 400., 0. );

    // arrow
    glVertex3f(0, 400.0f, 0.0f);
    glVertex3f(0, 300, 100.0f);

    glVertex3f(0, 400.0f, 0.0f);
    glVertex3f(0, 300, -100.0f);
    glEnd();
    glFlush();

    //z axis
    glBegin( GL_LINES );
    glVertex3f( 0., 0., 0. );
    glVertex3f( 0, 0., 400. );

    // arrow
    glVertex3f(0, 0.0f, 400.0f);
    glVertex3f(0, 100, 300.0f);

    glVertex3f(0, 0.0f, 400.0f);
    glVertex3f(0, -100, 300.0f);
    glEnd();
    glFlush();

    m_program->release();
}

void MyOpenGLWidget::resizeGL(int w, int h)
{
    m_proj.setToIdentity();
    m_proj.perspective(fov, GLfloat(w) / h, 1000.0f, -1000.0f);
}

//specified how OpenGL should interpret the vertex data
void MyOpenGLWidget::setupVertexAttribs()
{
    m_geometryVbo.bind();
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    f->glEnableVertexAttribArray(0);
    f->glEnableVertexAttribArray(1);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), nullptr);
    f->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat),
                             reinterpret_cast<void *>(3 * sizeof(GLfloat)));
    m_geometryVbo.release();
}

void MyOpenGLWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastPos = event->pos();
}

void MyOpenGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - m_lastPos.x();
    int dy = event->y() - m_lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRotation(m_xRot + 8 * dy);
        setZRotation(m_zRot + 8 * dx);
    }
    else if (event->buttons() & Qt::RightButton) {
        setXTranslation(m_xTran + 1 * dx);
        setYTranslation(m_yTran - 1 * dy);
    }
    m_lastPos = event->pos();
}

void MyOpenGLWidget::wheelEvent(QWheelEvent *event)
{
    setZoom(event->delta()/30);

}
