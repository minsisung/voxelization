QT       += core gui opengl
LIBS += -lopengl32

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    component.cpp \
    contactcomponentspair.cpp \
    createcubes.cpp \
    creategeometry.cpp \
    groupingpreprocessor.cpp \
    groupingresolver.cpp \
    groupingvalidator.cpp \
    initialgrouper.cpp \
    joint.cpp \
    link.cpp \
    machinetool.cpp \
    main.cpp \
    mainwindow.cpp \
    myopenglwidget.cpp \
    tinyxml2.cpp \
    urdfexporter.cpp \
    voxel.cpp \
    voxelforccp.cpp \
    voxelizer.cpp

HEADERS += \
    Vector3.h \
    component.h \
    contactcomponentspair.h \
    createcubes.h \
    creategeometry.h \
    groupingpreprocessor.h \
    groupingresolver.h \
    groupingvalidator.h \
    initialgrouper.h \
    joint.h \
    link.h \
    machinetool.h \
    mainwindow.h \
    myopenglwidget.h \
    shaders.h \
    stl_reader.h \
    tinyxml2.h \
    urdfexporter.h \
    voxel.h \
    voxelforccp.h \
    voxelizer.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


## OpenCascade
DEFINES +=  \
    WNT
INCLUDEPATH +=  C:/OpenCASCADE-7.3.0-vc14-64/build/inc
LIBS += -LC:/OpenCASCADE-7.3.0-vc14-64/build/win64/vc14/libd
LIBS += -lTKernel -lTKMath -lTKTopAlgo -lTKV3d -lTKOpenGl -lTKService
LIBS += -lTKG2d
LIBS += -lTKBRep -lTKSTL
LIBS += -lTKXSBase -lTKIGES -lTKSTEP -lTKXDESTEP -lTKXDEIGES
LIBS += -lTKMeshVS -lTKXSDRAW
LIBS += -lTKLCAF -lTKXCAF -lTKCAF
LIBS += -lTKG3d
LIBS += -lTKGeomBase
