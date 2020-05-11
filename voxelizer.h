#ifndef VOXELIZER_H
#define VOXELIZER_H
#include <QVector>
#include <stl_reader.h>
#include <voxel.h>


typedef struct vx_vertex {
    union {
        float v[3];
        struct {
            float x;
            float y;
            float z;
        };
        struct {
            float r;
            float g;
            float b;
        };
    };
} vx_vertex_t;

typedef vx_vertex_t vx_vec3_t;
typedef vx_vertex_t vx_color_t;

typedef struct vx_mesh {
    vx_vertex_t* vertices;          // Contiguous mesh vertices
    vx_color_t* colors;             // Contiguous vertices colors
    vx_vec3_t* normals;             // Contiguous mesh normals
    unsigned int* indices;          // Mesh indices
    unsigned int* normalindices;    // Mesh normal indices
    size_t nindices;                // The number of normal indices
    size_t nvertices;               // The number of vertices
    size_t nnormals;                // The number of normals
} vx_mesh_t;

typedef struct vx_triangle {
    union {
        vx_vertex_t vertices[3];
        struct {
            vx_vertex_t p1;
            vx_vertex_t p2;
            vx_vertex_t p3;
        };
    };
    vx_color_t colors[3];
} vx_triangle_t;


class Voxelizer
{
public:
    Voxelizer();
    QVector < QVector < QVector< Voxel > > > voxelspace;
    void Voxelize(stl_reader::StlMesh <float, unsigned int> mesh);
    void resize(int size);
    void setupSize(float spaceLength, float voxelSize);

private:
    float spaceLength;
    float voxelSize;
    vx_vertex_t boxcenter;
    vx_vertex_t halfboxsize;
    vx_triangle_t triangle;


};

#endif // VOXELIZER_H
