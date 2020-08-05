#ifndef VOXELIZER_H
#define VOXELIZER_H
#include <QVector>
#include <stl_reader.h>
#include <voxel.h>
#include <limits>
#include <QtGlobal>
#include <QMatrix4x4>
#include "machinetool.h"

typedef struct vx_vertex {
    union {
        float v[3];
        struct {
            float x;
            float y;
            float z;
        };
    };
} vx_vertex_t;

typedef vx_vertex_t vx_vec3_t;


typedef struct vx_triangle {
    union {
        vx_vertex_t vertices[3];
        struct {
            vx_vertex_t p1;
            vx_vertex_t p2;
            vx_vertex_t p3;
        };
    };
} vx_triangle_t;


class Voxelizer
{
public:
    Voxelizer();
    QVector < QVector < QVector< Voxel > > > voxelspace;


    float voxelStarting_X;
    float voxelStarting_Y;
    float voxelStarting_Z;

    int voxelSpaceSize_X;
    int voxelSpaceSize_Y;
    int voxelSpaceSize_Z;

    //Component       QChar
    //    Base         'b'
    //    X            'X'
    //    Y            'Y'
    //    Z            'Z'
    //    A            'A'
    //    B            'B'
    //    C            'C'

    void Voxelize(Link& link);

    void setupSize(float voxelSize, MachineTool& MT);
    void loadAndTransform(size_t itri, stl_reader::StlMesh <float, unsigned int>& mesh, QMatrix4x4 TransformMatrix);
    void setupInitialTransformationMatrix(MachineTool& MT, float x, float y, float z, float a, float b, float c);
    void setTransformationMatrix(MachineTool& MT, QChar linkType, float amount);
    QSet<QString> translateVoxelModel(MachineTool &MT, QChar linkType, float amount, int ind1, int ind2);

private:
    void parentModelVoxelization(Link& link);
    QSet<QString> translateVoxels(Link *link, QChar linkType, int voxelNumberDistance, int ind1, int ind2);
    float spaceLength_X;
    float spaceLength_Y;
    float spaceLength_Z;

    float voxelSize;

    float VSEnglargeRatio;

    vx_vertex_t boxcenter;
    vx_vertex_t halfboxsize;
    vx_triangle_t triangle;
    vx_vertex_t p1;
    vx_vertex_t p2;
    vx_vertex_t p3;
};

#endif // VOXELIZER_H
