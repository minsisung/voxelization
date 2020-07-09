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

    //Component       Char
    //    Base         'b'
    //    X            'X'
    //    Y            'Y'
    //    Z            'Z'
    //    A            'A'
    //    B            'B'
    //    C            'C'

    void Voxelize(Link& link, bool needVisualization);

    void setupSize(float spaceLength, float voxelSize);
    int get_x_min_index(){return bounding_x_min_index;}
    int get_x_max_index(){return bounding_x_max_index;}
    int get_y_min_index(){return bounding_y_min_index;}
    int get_y_max_index(){return bounding_y_max_index;}
    int get_z_min_index(){return bounding_z_min_index;}
    int get_z_max_index(){return bounding_z_max_index;}
    void set_bounding_voxel_index(int index_x_min, int index_x_max, int index_y_min, int index_y_max, int index_z_min, int index_z_max);
    void reset_bounding_index();
    void loadAndTransform(size_t itri, stl_reader::StlMesh <float, unsigned int>& mesh, QMatrix4x4 TransformMatrix);
    void setupInitialTransformationMatrix(MachineTool& MT, float x, float y, float z, float a, float b, float c);
    void setTransformationMatrix(MachineTool& MT, char linkType, float amount);
    void translateVoxelModel(MachineTool &MT, char linkType, float amount);


private:
    void fillInnerShell(char linkType, int indexX, int indexY, int indexZ, const float* normalArray);
    void normalVoxelization(Link& link, bool needVisualization);
    void translateVoxels(Link *link, char linkType, int voxelNumberDistance);
    float spaceLength;
    float voxelSize;
    int voxelSpaceSize;

    vx_vertex_t boxcenter;
    vx_vertex_t halfboxsize;
    vx_triangle_t triangle;
    vx_vertex_t p1;
    vx_vertex_t p2;
    vx_vertex_t p3;
    QVector < QVector < QVector<int> > > shellMap;

    int bounding_x_min_index = std::numeric_limits<int>::max();
    int bounding_x_max_index = 0;
    int bounding_y_min_index = std::numeric_limits<int>::max();
    int bounding_y_max_index = 0;
    int bounding_z_min_index = std::numeric_limits<int>::max();
    int bounding_z_max_index = 0;
};

#endif // VOXELIZER_H
