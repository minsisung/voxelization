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
      m_xTran(0), m_yTran(0), m_zTran(-5000), //original camera position
      fov(45.0f),
      m_program(nullptr)
{
    m_core = QSurfaceFormat::defaultFormat().profile() == QSurfaceFormat::CoreProfile;
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

    //m_geometry.readSTL(m_filepath);

    //Select which one to be painted  1. machinetool  2. CCP
    QString paintMode = "machinetool";
    //timer
    QElapsedTimer timer_total;
    QElapsedTimer timer_step1;
    long long step1_time;
    QVector<long long> step2_times;
    QVector<long long> step3_times;

    //**Step 1: Find contact-components pairs
    timer_total.start();
    timer_step1.start();

    //read stl files -------------------------
    //    machineToolName = "UMC-500";   //----------------
    //    //rotary axes of machine tool (A,B,C)
    //    QVector3D mtRotaryAxes(0,1,1);

    machineToolName = "UMC-750"; //----------------
    //rotary axes of machine tool (A,B,C)
    QVector3D mtRotaryAxes(0,1,1);

    //    machineToolName = "UMC-1600H";   //----------------
    //    //rotary axes of machine tool (A,B,C)
    //    QVector3D mtRotaryAxes(1,0,1);

    //    machineToolName = "VR-8";   //----------------
    //    //rotary axes of machine tool (A,B,C)
    //    QVector3D mtRotaryAxes(1,0,1);

    //    machineToolName = "VF-2";   //----------------
    //    //rotary axes of machine tool (A,B,C)
    //    QVector3D mtRotaryAxes(1,0,1);

    //set up component vector for initial grouping using small voxel size
    float voxelsize_initialGrouping = 3.0f;
    qDebug()<<"Initial grouping voxel size:" <<voxelsize_initialGrouping<<"mm";
    QVector<component> compVector_initialGrouping = readCompSTL(machineToolName, mtRotaryAxes, "initialGrouping");

    //dynamically allocate the class
    GroupingPreProcessor* m_groupingPreProcessor = new GroupingPreProcessor();

    //setup voxel space
    m_groupingPreProcessor->createMTVoxelspace(voxelsize_initialGrouping, compVector_initialGrouping);
    int num_LIPs = 3 + static_cast<int>(mtRotaryAxes.x()) +
            static_cast<int>(mtRotaryAxes.y()) + static_cast<int>(mtRotaryAxes.z());

    //find CCPs using collision detection and also update CCPVector by doing relative movement for
    //the following LIPs checking
    QVector<contactComponentsPair> ccpVector = m_groupingPreProcessor->findContactComponentsPairs(compVector_initialGrouping);

    //Find LIPs using the information from above
    int LIPs_Number = m_groupingPreProcessor->findLIPCandidates(ccpVector);

    CCPs = m_groupingPreProcessor->CCPs;
    LIPs = m_groupingPreProcessor->LIPs;

    //    //get information for drawing CCP
    if(paintMode == "CCP"){
        m_cubeGemoetry.m_paintMode = paintMode;
        m_cubeGemoetry.setData(m_groupingPreProcessor->getData());
        m_cubeGemoetry.setTotalCount(m_groupingPreProcessor->getTotalCount());
        num_Vertex_CCP_comp1 = m_groupingPreProcessor->numberOfVertex_comp1;
        num_Vertex_CCP_comp2 = m_groupingPreProcessor->numberOfVertex_comp2;
    }

    //deallocate the class and compVector_initialGrouping
    delete m_groupingPreProcessor;
    m_groupingPreProcessor = nullptr;
    compVector_initialGrouping.clear();

    //if number of LIPs is small than the number of groups - 1, which means there might have missing components,
    //then stop processing.
    if(LIPs_Number < num_LIPs)
        return;

    //Initial grouping using LIPs and CCPs
    //dynamically allocate the class
    InitialGrouper* initialGrouper = new InitialGrouper(CCPs, LIPs, num_LIPs + 1, compVector_initialGrouping);
    QVector<QPair<QString,QVector<QString>>> group_axisVector = initialGrouper->startGrouping();

    //If LIPs number != number of group - 1, stop processing
    if(group_axisVector.isEmpty())
        return;

    //set up component vector for grouping validation using big voxel size
    float voxelsize_groupingValidation = 6.0f;
    qDebug()<<"Initial grouping voxel size:" <<voxelsize_groupingValidation<<"mm";
    QVector<component> compVector_groupingValidation = readCompSTL(machineToolName, mtRotaryAxes, "groupingValidation");

    //create machine tool object by initial grouping
    MT = initialGrouper->createMT(group_axisVector, compVector_groupingValidation, machineToolName);
    QVector<JointString> jointStringVector = initialGrouper->getjointStringVector();

    OverlappingCompsVector = initialGrouper->getOverlappingCompsVector();

    //deallocate the class
    delete initialGrouper;
    initialGrouper = nullptr;

    qDebug()<<"overlappingCompVector"<<OverlappingCompsVector;
    step1_time = timer_step1.elapsed()/1000;

    int num_interation = 0;
    while(true){
        qDebug()<<"Grouping iteration no."<<num_interation + 1;
        num_interation++;

        //**Step 2: Check collision for all configurations--------------
        QElapsedTimer* timer_step2 = new QElapsedTimer;
        timer_step2->start();
        m_groupingValidator.createMTVoxelspace(voxelsize_groupingValidation, compVector_groupingValidation);
        qDebug()<<"GenerateMTVoxelspace";
        QVector<QPair<QString,QString>> collisionPairsVector = m_groupingValidator.collisionDetectionForConfigurations(MT, true);
        qDebug()<<"Collision Pairs for all configurations:"<<collisionPairsVector<<endl;
        step2_times.append(timer_step2->elapsed()/1000);
        delete timer_step2;
        timer_step2 = nullptr;

        //        //**Step 3: if collision occurs, resolve it--------------------

        if(!collisionPairsVector.isEmpty()){
            QElapsedTimer* timer_step3 = new QElapsedTimer;
            timer_step3->start();
            GroupingResolver m_groupingResolver(CCPs, OverlappingCompsVector,
                                                collisionPairsVector, group_axisVector);
            group_axisVector = m_groupingResolver.regroup();

            for(int ind_group = 0; ind_group < group_axisVector.size(); ind_group++){
                qDebug()<<"Link"<<group_axisVector[ind_group].first<<"contains"
                       <<group_axisVector[ind_group].second;
            }

            //Update MT
            MT = m_groupingResolver.createMT(group_axisVector, compVector_groupingValidation, jointStringVector, machineToolName);
            m_groupingValidator.clear();
            step3_times.append(timer_step3->elapsed()/1000);
            delete timer_step3;
            timer_step3 = nullptr;
        }else{
            qDebug()<<"Grouping Done!";

            //print out times for calculation
            qDebug()<<"Initial Grouping takes:"<<step1_time << "seconds";
            for(int i = 0; i < num_interation; i ++){
                if(i < num_interation - 1){
                    qDebug()<<"Iteration no."<<i + 1<<"time";
                    qDebug()<<"Grouping validation takes:"<<step2_times[i] << "seconds";
                    qDebug()<<"Grouping resolving takes:"<<step3_times[i] << "seconds";
                }else{
                    qDebug()<<"Iteration no."<<i + 1<<"time";
                    qDebug()<<"Grouping validation takes:"<<step2_times[i] << "seconds";
                }
            }
            break;
        }
    }
    qDebug()<<"Total process takes:"<<timer_total.elapsed()/1000 << "seconds";

    //    //get information for drawing the machine tool
    //    if(paintMode == "machinetool"){
    //        m_groupingValidator.drawVoxelforMT(MT, 0, 0);
    //        m_cubeGemoetry.m_paintMode = paintMode;
    //        m_cubeGemoetry.setData(m_groupingValidator.getData());
    //        m_cubeGemoetry.setTotalCount(m_groupingValidator.getTotalCount());
    //    }

    //export urdf
    UrdfExporter urdfExporter(MT);
    urdfExporter.start();

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
    //    m_alphaLoc = m_program->uniformLocation("alpha");

    //    m_vao.create();
    //    QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);
    //    //To use a VAO, all you have to do is bind the VAO

    // Setup our vertex buffer object.
    m_geometryVbo.create();
    //Create a buffer for dynamic allocation later

    m_geometryVbo.bind();
    //Bind the buffer so that it is the current active buffer

    m_geometryVbo.allocate(m_cubeGemoetry.constData(), m_cubeGemoetry.totalCount() * sizeof(GLfloat));
    //Allocate and initialize the information

    m_geometryVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    //The data will be set once and used many times for drawing operations.

    // Store the vertex attribute bindings for the program.
    setupVertexAttribs();

    m_camera.setToIdentity();
    m_camera.translate(m_xTran,m_yTran, m_zTran);

    // Light position is fixed.
    m_program->setUniformValue(m_lightPosLoc, QVector3D(1000, 100, 2000));
    //    m_program->setUniformValue(m_colorLoc, m_color);
    //    m_program->setUniformValue(m_alphaLoc, 1.0f);

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

    //check if visualization is necessary
    if(m_cubeGemoetry.ifNeedVisualization){
        if(m_cubeGemoetry.m_paintMode == "machinetool"){
            drawMTComponents();
        }else{
            drawCCPComponents();
        }
    }
    m_program->setUniformValue(m_mvMatrixLoc, m_camera * m_world);

    //Creating coordinate system================================================================================
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

    //================================================================================================
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

void MyOpenGLWidget::drawMTComponents()
{
    //QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);
    m_program->bind();
    m_program->setUniformValue(m_projMatrixLoc, m_proj);
    m_program->setUniformValue(m_mvMatrixLoc, m_camera * m_world);
    QMatrix3x3 normalMatrix = m_world.normalMatrix();
    m_program->setUniformValue(m_normalMatrixLoc, normalMatrix);
    int startNumber = 0;

    for (int ind_link = 0; ind_link < MT.LinkVector.size(); ind_link++){
        // only draw skeleton of each triangle
        //                                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        float r, g, b;
        if(ind_link == 0){
            r = 0.8f; g = 0.7f;b = 0.7f;
        }else if(ind_link == 1){r = 0.6f; g = 0.4f; b = 0.3987f;
        }else if(ind_link == 2){r = 0.5143f; g = 0.143f; b = 0.456f;
        }else if(ind_link == 3){r = 0.125f; g = 0.643f; b = 0.619f;
        }else if(ind_link == 4){r = 0.32f; g = 0.2f; b = 0.8f;
        }else if(ind_link == 5){r = 0.37647f ; g = 0.75294f; b = 0.37647f ;
        }else{
            r = 0.1f; g = 0.7f; b = 0.8f;
        }
        m_program->setUniformValue(m_colorLoc, QVector3D(r,g,b));
        //        qDebug()<<QString::fromStdString(MT.LinkVector[ind_link].getName())<<"RGB:"<<r<<g<<b;
        //draw triangles
        glDrawArrays(GL_TRIANGLES, startNumber, MT.LinkVector[ind_link].numberOfVertex);
        //update starting number of each component
        startNumber += MT.LinkVector[ind_link].numberOfVertex;
    }
}

void MyOpenGLWidget::drawCCPComponents()
{
    //QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);
    m_program->bind();
    m_program->setUniformValue(m_projMatrixLoc, m_proj);
    m_program->setUniformValue(m_mvMatrixLoc, m_camera * m_world);
    QMatrix3x3 normalMatrix = m_world.normalMatrix();
    m_program->setUniformValue(m_normalMatrixLoc, normalMatrix);
    int startNumber = 0;
    for (int comp_ind = 0; comp_ind < 2; comp_ind++){

        // only draw skeleton of each triangle
        // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        m_program->setUniformValue(m_colorLoc, QVector3D(static_cast<float>(0.37647f * (comp_ind + 1)),
                                                         static_cast<float>(0.75294f / (comp_ind + 1)),
                                                         static_cast<float>(0.37647f * (comp_ind + 1))));
        if(comp_ind ==0){
            //draw triangles
            glDrawArrays(GL_TRIANGLES, startNumber, num_Vertex_CCP_comp1);
            //update starting number of each component
            startNumber += num_Vertex_CCP_comp1;
        }else{
            //draw triangles
            glDrawArrays(GL_TRIANGLES, startNumber, num_Vertex_CCP_comp2);
            //update starting number of each component
            startNumber += num_Vertex_CCP_comp2;
        }
    }
}

QVector<stl_reader::StlMesh <float, unsigned int>> MyOpenGLWidget::readSTLFiles(QString mtName)
{
    QVector<stl_reader::StlMesh <float, unsigned int>> m_STLMeshVector;

    //store all .stl that contain the name of machine tool
    QDir directory = QDir::current();
    QStringList STLList = directory.entryList(QStringList() << "*.STL" << "*.stl",QDir::Files);
    QStringList mtSTLList = STLList.filter(mtName, Qt::CaseInsensitive);

    for(int i = 0; i< mtSTLList.size(); i++)
        try {
        //read STL file for each file
        stl_reader::StlMesh <float, unsigned int> mesh(mtSTLList[i].toStdString());

        m_STLMeshVector.append(mesh);
        qDebug() << "Finish setting mesh for" <<mtSTLList[i]<< endl;
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }
    return m_STLMeshVector;
}

QVector<component> MyOpenGLWidget::readCompSTL(QString mtName, QVector3D mtRotaryAxes, QString mode)
{
    QVector<component> compVector;

    //store all .stl that contain the name of machine tool
    QString path;
    if(mode == "initialGrouping"){
        path = QDir::current().path() + "/" + mtName + "/initialGrouping";
        qDebug()<<"create compVector for initial grouping";
    }
    if(mode == "groupingValidation"){
        path = QDir::current().path() + "/" + mtName+ "/groupingValidation";
        qDebug()<<"create compVector for grouping validation";
    }

    QDir directory(path);

    QStringList STLList = directory.entryList(QStringList() << "*.STL" << "*.stl",QDir::Files);
    QStringList nonOffsetSTLList;
    foreach (const QString &str, STLList) {
        if (!str.contains("_offset"))
            nonOffsetSTLList += str;
    }

    if(nonOffsetSTLList.empty())
        qDebug()<<"No STL files detected"<<endl;

    for(int i = 0; i< nonOffsetSTLList.size(); i++){
        stl_reader::StlMesh <float, unsigned int> nonOffestMesh;
        try {
            //read STL file for each file
            nonOffestMesh.read_file((path + "/" + nonOffsetSTLList[i]).toStdString());
            qDebug() << "Finish setting mesh for" <<nonOffsetSTLList[i];
        }
        catch (std::exception& e) {
            std::cout << e.what() << std::endl;
        }

        //create component
        QString componentName = nonOffsetSTLList[i].left(nonOffsetSTLList[i].size() - 4);
        component comp(componentName, nonOffestMesh);

        QString offsetMeshName = nonOffsetSTLList[i].insert(nonOffsetSTLList[i].size() - 4,"_offset");
        //if offset mesh exists
        if(STLList.contains(offsetMeshName)){
            stl_reader::StlMesh <float, unsigned int> offestMesh;
            try {
                //read offset STL file
                offestMesh.read_file((path + "/" + offsetMeshName).toStdString());
                qDebug() << "Finish setting mesh for" <<offsetMeshName;
            }
            catch (std::exception& e) {
                std::cout << e.what() << std::endl;
            }

            comp.setOffsetMesh(offestMesh);
            comp.setContainsOffsetMesh();
        }
        qDebug()<<endl;

        compVector.append(comp);
    }

    QXmlStreamReader Rxml;
    QString filename = machineToolName + ".xml";
    QString filePath_Name = QDir::current().path() + "/" + mtName + "/" + filename;
    QFile file(filePath_Name);
    //if there xml doesn't exist
    if (!file.open(QFile::ReadOnly | QFile::Text))
    {
        qDebug()<<"There is no"<<filename<<"exists, so it's going to create one"<<endl;
        return getAxisForComp(compVector, mtRotaryAxes);
    }else{
        qDebug()<<"Read .xml for axes of components";
        Rxml.setDevice(&file);
        return readAxisForComp(compVector, Rxml , mtRotaryAxes);
    }
}

QVector<component> MyOpenGLWidget::getAxisForComp(QVector<component>& compVector, QVector3D& mtRotaryAxes)
{
    //open file for writing xml
    QString filename = machineToolName + ".xml";
    QString directoryName = QDir::current().path() + "/" + machineToolName;
    QString filePath = directoryName + '/' + filename;
    QFile file(filePath);
    file.open(QIODevice::WriteOnly);

    QXmlStreamWriter xmlWriter(&file);
    xmlWriter.setAutoFormatting(true);
    xmlWriter.writeStartDocument();
    xmlWriter.writeStartElement(machineToolName);

    // Assign rotary axis
    for(int i = 0; i < compVector.size(); i++){
        component comp = compVector[i];

        xmlWriter.writeStartElement("Component");
        xmlWriter.writeAttribute("Name" ,comp.getName());

        // 3-axis Machine
        if(mtRotaryAxes == QVector3D(0,0,0)){
            comp.m_mtRotaryAxes = QVector3D(0,0,0);
        }else{
            //read B-Rep model
            TopoDS_Shape aShape = readBRep(comp.getName());

            //AB 5-axis Machine
            if(mtRotaryAxes == QVector3D(1,1,0)){
                QVector3D rotaryAxisPoint1;
                QVector3D rotaryAxisPoint2;

                rotaryAxisPoint1 = findCommonAxis(aShape, "A");
                //if rotary axis 1 exist
                if(!(abs(rotaryAxisPoint1.x() - 111111.0f) < 0.0001f)){
                    comp.setRotaryAxisPoint1(rotaryAxisPoint1.x(),rotaryAxisPoint1.y(),rotaryAxisPoint1.z());
                    xmlWriter.writeStartElement("rotaryAxisPoint1");
                    xmlWriter.writeAttribute("XYZ" ,QString::number(rotaryAxisPoint1.x()) + " " + QString::number(rotaryAxisPoint1.y()) + " " +  QString::number(rotaryAxisPoint1.z()));
                    xmlWriter.writeEndElement();
                }

                rotaryAxisPoint2 = findCommonAxis(aShape, "B");
                //if rotary axis 2 exist
                if(!(abs(rotaryAxisPoint2.x() - 111111.0f) < 0.0001f)){
                    comp.setRotaryAxisPoint2(rotaryAxisPoint2.x(),rotaryAxisPoint2.y(),rotaryAxisPoint2.z());
                    xmlWriter.writeStartElement("rotaryAxisPoint2");
                    xmlWriter.writeAttribute("XYZ" ,QString::number(rotaryAxisPoint2.x()) + " " +  QString::number(rotaryAxisPoint2.y()) + " " +  QString::number(rotaryAxisPoint2.z()));
                    xmlWriter.writeEndElement();
                }
            }

            //BC 5-axis Machine
            if(mtRotaryAxes == QVector3D(0,1,1)){
                QVector3D rotaryAxisPoint1;
                QVector3D rotaryAxisPoint2;

                rotaryAxisPoint1 = findCommonAxis(aShape, "B");
                //if rotary axis 1 exist
                if(!(abs(rotaryAxisPoint1.x() - 111111.0f) < 0.0001f)){
                    comp.setRotaryAxisPoint1(rotaryAxisPoint1.x(),rotaryAxisPoint1.y(),rotaryAxisPoint1.z());
                    xmlWriter.writeStartElement("rotaryAxisPoint1");
                    xmlWriter.writeAttribute("XYZ" ,QString::number(rotaryAxisPoint1.x()) + " " +  QString::number(rotaryAxisPoint1.y()) + " " +  QString::number(rotaryAxisPoint1.z()));
                    xmlWriter.writeEndElement();
                }


                rotaryAxisPoint2 = findCommonAxis(aShape, "C");
                //if rotary axis 2 exist
                if(!(abs(rotaryAxisPoint2.x() - 111111.0f) < 0.0001f)){
                    comp.setRotaryAxisPoint2(rotaryAxisPoint2.x(),rotaryAxisPoint2.y(),rotaryAxisPoint2.z());
                    xmlWriter.writeStartElement("rotaryAxisPoint2");
                    xmlWriter.writeAttribute("XYZ" ,QString::number(rotaryAxisPoint2.x()) + " " +  QString::number(rotaryAxisPoint2.y()) + " " +  QString::number(rotaryAxisPoint2.z()));
                    xmlWriter.writeEndElement();
                }
            }

            //AC 5-axis Machine
            if(mtRotaryAxes == QVector3D(1,0,1)){
                QVector3D rotaryAxisPoint1;
                QVector3D rotaryAxisPoint2;

                rotaryAxisPoint1 = findCommonAxis(aShape, "A");
                //if rotary axis 1 exist
                qDebug()<<"rotary axis 1:"<<rotaryAxisPoint1<<endl;
                if(!(abs(rotaryAxisPoint1.x() - 111111.0f) < 0.0001f)){
                    comp.setRotaryAxisPoint1(rotaryAxisPoint1.x(),rotaryAxisPoint1.y(),rotaryAxisPoint1.z());
                    xmlWriter.writeStartElement("rotaryAxisPoint1");
                    xmlWriter.writeAttribute("XYZ" ,QString::number(rotaryAxisPoint1.x()) + " " +  QString::number(rotaryAxisPoint1.y()) + " " +  QString::number(rotaryAxisPoint1.z()));
                    xmlWriter.writeEndElement();
                }

                rotaryAxisPoint2 = findCommonAxis(aShape, "C");
                //if rotary axis 2 exist
                qDebug()<<"rotary axis 2:"<<rotaryAxisPoint2<<endl;
                if(!(abs(rotaryAxisPoint2.x() - 111111.0f) < 0.0001f)){
                    comp.setRotaryAxisPoint2(rotaryAxisPoint2.x(),rotaryAxisPoint2.y(),rotaryAxisPoint2.z());
                    xmlWriter.writeStartElement("rotaryAxisPoint2");
                    xmlWriter.writeAttribute("XYZ" ,QString::number(rotaryAxisPoint2.x()) + " " +  QString::number(rotaryAxisPoint2.y()) + " " +  QString::number(rotaryAxisPoint2.z()));
                    xmlWriter.writeEndElement();
                }
            }

            //A 4-axis Machine
            if(mtRotaryAxes == QVector3D(1,0,0)){
                QVector3D rotaryAxisPoint1;

                rotaryAxisPoint1 = findCommonAxis(aShape, "A");
                //if rotary axis 1 exist
                if(!(abs(rotaryAxisPoint1.x() - 111111.0f) < 0.0001f)){
                    comp.setRotaryAxisPoint1(rotaryAxisPoint1.x(),rotaryAxisPoint1.y(),rotaryAxisPoint1.z());
                    xmlWriter.writeStartElement("rotaryAxisPoint1");
                    xmlWriter.writeAttribute("XYZ" ,QString::number(rotaryAxisPoint1.x()) + " " +  QString::number(rotaryAxisPoint1.y()) + " " +  QString::number(rotaryAxisPoint1.z()));
                    xmlWriter.writeEndElement();
                }
            }

            //B 4-axis Machine
            if(mtRotaryAxes == QVector3D(0,1,0)){
                QVector3D rotaryAxisPoint1;

                rotaryAxisPoint1 = findCommonAxis(aShape, "B");
                //if rotary axis 1 exist
                if(!(abs(rotaryAxisPoint1.x() - 111111.0f) < 0.0001f)){
                    comp.setRotaryAxisPoint1(rotaryAxisPoint1.x(),rotaryAxisPoint1.y(),rotaryAxisPoint1.z());
                    xmlWriter.writeStartElement("rotaryAxisPoint1");
                    xmlWriter.writeAttribute("XYZ" ,QString::number(rotaryAxisPoint1.x()) + " " +  QString::number(rotaryAxisPoint1.y()) + " " +  QString::number(rotaryAxisPoint1.z()));
                    xmlWriter.writeEndElement();
                }
            }

            //C 4-axis Machine
            if(mtRotaryAxes == QVector3D(0,0,1)){
                QVector3D rotaryAxisPoint1;

                rotaryAxisPoint1 = findCommonAxis(aShape, "C");
                //if rotary axis 1 exist
                if(!(abs(rotaryAxisPoint1.x() - 111111.0f) < 0.0001f)){
                    comp.setRotaryAxisPoint1(rotaryAxisPoint1.x(),rotaryAxisPoint1.y(),rotaryAxisPoint1.z());
                    xmlWriter.writeStartElement("rotaryAxisPoint1");
                    xmlWriter.writeAttribute("XYZ" ,QString::number(rotaryAxisPoint1.x()) + " " +  QString::number(rotaryAxisPoint1.y()) + " " +  QString::number(rotaryAxisPoint1.z()));
                    xmlWriter.writeEndElement();
                }
            }
            comp.m_mtRotaryAxes = mtRotaryAxes;
        }
        xmlWriter.writeEndElement();
    }
    xmlWriter.writeEndElement();

    file.close();


    return compVector;
}

QVector3D MyOpenGLWidget::findCommonAxis(TopoDS_Shape aShape, QString componentAxis)
{
    QHash<QString, int> locationCollector;

    //Transverse all edge
    for (TopExp_Explorer fExpl(aShape, TopAbs_EDGE); fExpl.More(); fExpl.Next())
    {
        const TopoDS_Edge &curEdge =
                static_cast<const TopoDS_Edge &>(fExpl.Current());

        Standard_Real c_start, c_end;
        Handle_Geom_Curve curve= BRep_Tool::Curve(curEdge, c_start, c_end);

        if(!curve.IsNull()){
            if(curve->DynamicType()==STANDARD_TYPE(Geom_Circle))
            {
                Handle_Geom_Circle circle = Handle_Geom_Circle::DownCast(curve);
                gp_Circ gpCirc = circle->Circ();
                const gp_Ax1 axis = gpCirc.Axis();

                //                qDebug()<<"axis direction:"<<QString::number(axis.Direction().X(), 'f', 8)<<QString::number(axis.Direction().Y(), 'f', 8)<<QString::number(axis.Direction().Z(), 'f', 8);
                //                qDebug()<<"axis location:"<<QString::number(axis.Location().X(), 'f', 8)<<QString::number(axis.Location().Y(), 'f', 8)<<QString::number(axis.Location().Z(), 'f', 8)<<endl;

                //-------------------------------------------------------------------
                if(componentAxis =="C")
                {
                    if(IsEqual(axis.Direction().Y(),0)& IsEqual(axis.Direction().X(),0) &
                            (IsEqual(axis.Direction().Z(),1) | IsEqual(axis.Direction().Z(),-1)))
                    {
                        QString locationQString = QString("%1 %2").arg(QString::number(axis.Location().X(), 'f', 8))
                                .arg(QString::number(axis.Location().Y(), 'f', 8));

                        if(locationCollector.contains(locationQString))
                        {
                            locationCollector.insert(locationQString,
                                                     locationCollector.value(locationQString)+1);
                        }
                        else
                        {
                            locationCollector.insert(locationQString,1);
                        }
                    }
                }
                //-------------------------------------------------------------------
                if(componentAxis =="B")
                {
                    if((IsEqual(axis.Direction().Y(),1) | (IsEqual(axis.Direction().Y(),-1))) & IsEqual(axis.Direction().X(),0) &
                            IsEqual(axis.Direction().Z(),0))
                    {
                        QString locationQString = QString("%1 %2").arg(QString::number(axis.Location().X(), 'f', 8))
                                .arg(QString::number(axis.Location().Z(), 'f', 8));

                        if(locationCollector.contains(locationQString))
                        {
                            locationCollector.insert(locationQString,
                                                     locationCollector.value(locationQString)+1);
                        }
                        else
                        {
                            locationCollector.insert(locationQString,1);
                        }
                    }
                }
                //-------------------------------------------------------------------
                if(componentAxis =="A")
                {
                    if(IsEqual(axis.Direction().Y(),0)& (IsEqual(axis.Direction().X(),1) | IsEqual(axis.Direction().X(),-1)) &
                            IsEqual(axis.Direction().Z(),0))
                    {
                        QString locationQString = QString("%1 %2").arg(QString::number(axis.Location().X(), 'f', 8))
                                .arg(QString::number(axis.Location().Y(), 'f', 8));

                        if(locationCollector.contains(locationQString))
                        {
                            locationCollector.insert(locationQString,
                                                     locationCollector.value(locationQString)+1);
                        }
                        else
                        {
                            locationCollector.insert(locationQString,1);
                        }
                    }
                }
                //-------------------------------------------------------------------
            }
        }
    }

    QVector3D commonAxis;

    if(!locationCollector.empty()){
        int largestNumber = 0;
        QString largestString;
        QHashIterator<QString, int> i(locationCollector);

        // find the most common axis
        while (i.hasNext()) {
            i.next();
            if(i.value()>largestNumber)
            {
                largestNumber = i.value();
                largestString = i.key();
            }
        }
        qDebug()<<"Largest Number:"<<largestNumber<<endl;
        //only the axis that repeats twice or more need to be considered
        if(largestNumber > 2){

            QStringList splitList = largestString.split(" ");
            float value1 = splitList.at(0).toFloat();
            float value2 = splitList.at(1).toFloat();

            // for C axis, unit conversion
            if(componentAxis =="C")
            {
                commonAxis.setX(value1/1000.0f);
                commonAxis.setY(value2/1000.0f);
                commonAxis.setZ(0);

                qDebug()<<"Component C: "<<commonAxis.x()<<" "
                       << commonAxis.y()<<" "<<commonAxis.z();
            }
            // for B axis, unit conversion
            if(componentAxis =="B")
            {
                commonAxis.setX(value1/1000.0f);
                commonAxis.setY(0);
                commonAxis.setZ(value2/1000.0f);

                qDebug()<<"Component B: "<<commonAxis.x()<<" "
                       << commonAxis.y()<<" "<<commonAxis.z();
            }
            // for A axis, unit conversion
            if(componentAxis =="A")
            {
                commonAxis.setX(0);
                commonAxis.setY(value1/1000.0f);
                commonAxis.setZ(value2/1000.0f);

                qDebug()<<"Component A: "<<commonAxis.x()<<" "
                       << commonAxis.y()<<" "<<commonAxis.z();
            }
            qDebug()<<"Largest Number: "<<largestNumber<<endl;
        }else{
            //stupid way to check if commonAxis is empty
            commonAxis.setX(111111.0f);
        }
    }else{
        //stupid way to check if commonAxis is empty
        commonAxis.setX(111111.0f);
    }

    return commonAxis;
}

TopoDS_Shape MyOpenGLWidget::readBRep(QString compName)
{
    qDebug()<<"Read B-Rep model for"<<compName;
    QString fileName = compName + ".IGS";

    QString path = QDir::current().path() + "/" + machineToolName;
    IGESControl_Reader Reader;
    std::string fileName_str = (path + "/" + fileName).toStdString();
    Standard_Integer status = Reader.ReadFile((Standard_CString)fileName_str.c_str());

    if (status != IFSelect_RetDone)
    {
        qDebug()<<"error from reading IGES";
    }
    Reader.TransferRoots();

    return Reader.OneShape();
}

QVector<component> MyOpenGLWidget::readAxisForComp(QVector<component> &compVector, QXmlStreamReader& Rxml, QVector3D& mtRotaryAxes)
{
    int num_comp_inXML = 0;
    while(!Rxml.atEnd() && !Rxml.hasError()) {
        // Read next element
        QXmlStreamReader::TokenType token = Rxml.readNext();
        //If token is just StartDocument - go to next
        if(token == QXmlStreamReader::StartDocument) {
            continue;
        }
        //If token is StartElement - read it
        if(token == QXmlStreamReader::StartElement) {
            if(Rxml.name() == "Component") {
                num_comp_inXML++;
                QString name = Rxml.attributes().value("Name").toString();
                int index = indexOfComponent(compVector ,name);
                if(index != -1){
                    compVector[index].m_mtRotaryAxes = mtRotaryAxes;
                    qDebug()<<compVector[index].getName()<<"'s mtRotaryAxes is:"<<mtRotaryAxes;
                }
                while(Rxml.readNextStartElement()){
                    if(Rxml.name() == "rotaryAxisPoint1"){
                        QString xyz = Rxml.attributes().value("XYZ").toString();
                        QStringList splitList = xyz.split(" ");
                        float X = splitList.at(0).toFloat();
                        float Y = splitList.at(1).toFloat();
                        float Z = splitList.at(2).toFloat();
                        if(index != -1){
                            compVector[index].setRotaryAxisPoint1(X,Y,Z);
                        }
                        Rxml.readNext();
                    }
                    if(Rxml.name() == "rotaryAxisPoint2"){
                        QString xyz = Rxml.attributes().value("XYZ").toString();
                        QStringList splitList = xyz.split(" ");
                        float X = splitList.at(0).toFloat();
                        float Y = splitList.at(1).toFloat();
                        float Z = splitList.at(2).toFloat();
                        int index = indexOfComponent(compVector ,name);
                        if(index != -1){
                            compVector[index].setRotaryAxisPoint2(X,Y,Z);
                        }
                    }
                }
            }
        }
    }

    if(num_comp_inXML != compVector.size())
        qDebug()<<"Number of components in .xml doesn't match with the number of component 3d files"<<endl;

    return compVector;
}

int MyOpenGLWidget::indexOfComponent(QVector<component> &compVector, QString compName)
{
    for(int i = 0; i < compVector.size(); i++){
        if(compVector[i].getName() == compName)
            return i;
    }
    qDebug()<<"No"<<compName<<"in .IGES file"<<endl;
    return -1;
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
    setZoom(event->delta()/45);

}
