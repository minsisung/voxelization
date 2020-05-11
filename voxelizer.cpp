#include "voxelizer.h"
#include <QDebug>
#include <math.h>
#include <QElapsedTimer>

//====================================================================================================================
// ** Functions and variable types for calculating AABB-triangle intersection **
//====================================================================================================================

#define VX_FINDMINMAX(x0, x1, x2, min, max) \
    min = max = x0;                         \
    if (x1 < min) min = x1;                 \
    if (x1 > max) max = x1;                 \
    if (x2 < min) min = x2;                 \
    if (x2 > max) max = x2;

vx_vec3_t vx__vec3_cross(vx_vec3_t* v1, vx_vec3_t* v2)
{
    vx_vec3_t cross;
    cross.x = v1->y * v2->z - v1->z * v2->y;
    cross.y = v1->z * v2->x - v1->x * v2->z;
    cross.z = v1->x * v2->y - v1->y * v2->x;
    return cross;
}

void vx__vec3_sub(vx_vec3_t* a, vx_vec3_t* b)
{
    a->x -= b->x;
    a->y -= b->y;
    a->z -= b->z;
}

float vx__vec3_dot(vx_vec3_t* v1, vx_vec3_t* v2)
{
    return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}
bool vx__plane_box_overlap(vx_vec3_t* normal,
                           float d,
                           vx_vertex_t* halfboxsize)
{
    vx_vertex_t vmin, vmax;

    for (int dim = 0; dim <= 2; dim++) {
        if (normal->v[dim] > 0.0f) {
            vmin.v[dim] = -halfboxsize->v[dim];
            vmax.v[dim] = halfboxsize->v[dim];
        } else {
            vmin.v[dim] = halfboxsize->v[dim];
            vmax.v[dim] = -halfboxsize->v[dim];
        }
    }

    if (vx__vec3_dot(normal, &vmin) + d > 0.0f) {
        return false;
    }

    if (vx__vec3_dot(normal, &vmax) + d >= 0.0f) {
        return true;
    }

    return false;
}

#define AXISTEST_X01(a, b, fa, fb)                 \
    p1 = a * v1.y - b * v1.z;                      \
    p3 = a * v3.y - b * v3.z;                      \
    if (p1 < p3) {                                 \
    min = p1; max = p3;                        \
    } else {                                       \
    min = p3; max = p1;                        \
    }                                              \
    rad = fa * halfboxsize.y + fb * halfboxsize.z; \
    if (min > rad || max < -rad) {                 \
    return false;                              \
    }                                              \

#define AXISTEST_X2(a, b, fa, fb)                  \
    p1 = a * v1.y - b * v1.z;                      \
    p2 = a * v2.y - b * v2.z;                      \
    if (p1 < p2) {                                 \
    min = p1; max = p2;                        \
    } else {                                       \
    min = p2; max = p1;                        \
    }                                              \
    rad = fa * halfboxsize.y + fb * halfboxsize.z; \
    if (min > rad || max < -rad) {                 \
    return false;                              \
    }                                              \

#define AXISTEST_Y02(a, b, fa, fb)                 \
    p1 = -a * v1.x + b * v1.z;                     \
    p3 = -a * v3.x + b * v3.z;                     \
    if (p1 < p3) {                                 \
    min = p1; max = p3;                        \
    } else {                                       \
    min = p3; max = p1;                        \
    }                                              \
    rad = fa * halfboxsize.x + fb * halfboxsize.z; \
    if (min > rad || max < -rad) {                 \
    return false;                              \
    }                                              \

#define AXISTEST_Y1(a, b, fa, fb)                  \
    p1 = -a * v1.x + b * v1.z;                     \
    p2 = -a * v2.x + b * v2.z;                     \
    if (p1 < p2) {                                 \
    min = p1; max = p2;                        \
    } else {                                       \
    min = p2; max = p1;                        \
    }                                              \
    rad = fa * halfboxsize.x + fb * halfboxsize.z; \
    if (min > rad || max < -rad) {                 \
    return false;                              \
    }

#define AXISTEST_Z12(a, b, fa, fb)                 \
    p2 = a * v2.x - b * v2.y;                      \
    p3 = a * v3.x - b * v3.y;                      \
    if (p3 < p2) {                                 \
    min = p3; max = p2;                        \
    } else {                                       \
    min = p2; max = p3;                        \
    }                                              \
    rad = fa * halfboxsize.x + fb * halfboxsize.y; \
    if (min > rad || max < -rad) {                 \
    return false;                              \
    }

#define AXISTEST_Z0(a, b, fa, fb)                  \
    p1 = a * v1.x - b * v1.y;                      \
    p2 = a * v2.x - b * v2.y;                      \
    if (p1 < p2) {                                 \
    min = p1; max = p2;                        \
    } else {                                       \
    min = p2; max = p1;                        \
    }                                              \
    rad = fa * halfboxsize.x + fb * halfboxsize.y; \
    if (min > rad || max < -rad) {                 \
    return false;                              \
    }

bool vx__triangle_box_overlap(vx_vertex_t boxcenter,
                              vx_vertex_t halfboxsize,
                              vx_triangle_t triangle)
{
    vx_vec3_t v1, v2, v3, normal, e1, e2, e3;
    float min, max, d, p1, p2, p3, rad, fex, fey, fez;

    v1 = triangle.p1;
    v2 = triangle.p2;
    v3 = triangle.p3;

    vx__vec3_sub(&v1, &boxcenter);
    vx__vec3_sub(&v2, &boxcenter);
    vx__vec3_sub(&v3, &boxcenter);

    e1 = v2;
    e2 = v3;
    e3 = v1;

    vx__vec3_sub(&e1, &v1);
    vx__vec3_sub(&e2, &v2);
    vx__vec3_sub(&e3, &v3);

    fex = fabs(e1.x);
    fey = fabs(e1.y);
    fez = fabs(e1.z);

    AXISTEST_X01(e1.z, e1.y, fez, fey)
            AXISTEST_Y02(e1.z, e1.x, fez, fex)
            AXISTEST_Z12(e1.y, e1.x, fey, fex)

            fex = fabs(e2.x);
    fey = fabs(e2.y);
    fez = fabs(e2.z);

    AXISTEST_X01(e2.z, e2.y, fez, fey)
            AXISTEST_Y02(e2.z, e2.x, fez, fex)
            AXISTEST_Z0(e2.y, e2.x, fey, fex)

            fex = fabs(e3.x);
    fey = fabs(e3.y);
    fez = fabs(e3.z);

    AXISTEST_X2(e3.z, e3.y, fez, fey)
            AXISTEST_Y1(e3.z, e3.x, fez, fex)
            AXISTEST_Z12(e3.y, e3.x, fey, fex)

            VX_FINDMINMAX(v1.x, v2.x, v3.x, min, max)
            if (min > halfboxsize.x || max < -halfboxsize.x) {
        return false;
    }

    VX_FINDMINMAX(v1.y, v2.y, v3.y, min, max)
            if (min > halfboxsize.y || max < -halfboxsize.y) {
        return false;
    }

    VX_FINDMINMAX(v1.z, v2.z, v3.z, min, max)
            if (min > halfboxsize.z || max < -halfboxsize.z) {
        return false;
    }

    normal = vx__vec3_cross(&e1, &e2);
    d = -vx__vec3_dot(&normal, &v1);

    if (!vx__plane_box_overlap(&normal, d, &halfboxsize)) {
        return false;
    }

    return true;
}

#undef AXISTEST_X2
#undef AXISTEST_X01
#undef AXISTEST_Y1
#undef AXISTEST_Y02
#undef AXISTEST_Z0
#undef AXISTEST_Z12


//====================================================================================================================



//constructor
Voxelizer::Voxelizer()
{

}


void Voxelizer::Voxelize(stl_reader::StlMesh <float, unsigned int> mesh)
{
    vx_vertex_t p1;
    vx_vertex_t p2;
    vx_vertex_t p3;
    float min_x, max_x, min_y, max_y, min_z, max_z;

    QElapsedTimer timer;
    timer.start();

    for (size_t itri = 0; itri < mesh.num_tris(); ++itri) {

        //Load triangles from mesh
        p1.x = 1000 * mesh.vrt_coords(mesh.tri_corner_ind(itri, 0))[0];
        p1.y = 1000 * mesh.vrt_coords(mesh.tri_corner_ind(itri, 0))[1];
        p1.z = 1000 * mesh.vrt_coords(mesh.tri_corner_ind(itri, 0))[2];


        p2.x = 1000 * mesh.vrt_coords(mesh.tri_corner_ind(itri, 1))[0];
        p2.y = 1000 * mesh.vrt_coords(mesh.tri_corner_ind(itri, 1))[1];
        p2.z = 1000 * mesh.vrt_coords(mesh.tri_corner_ind(itri, 1))[2];

        p3.x = 1000 * mesh.vrt_coords(mesh.tri_corner_ind(itri, 2))[0];
        p3.y = 1000 * mesh.vrt_coords(mesh.tri_corner_ind(itri, 2))[1];
        p3.z = 1000 * mesh.vrt_coords(mesh.tri_corner_ind(itri, 2))[2];

        triangle.p1 = p1;
        triangle.p2 = p2;
        triangle.p3 = p3;

        //find bounding box of triangle
        VX_FINDMINMAX(triangle.p1.x, triangle.p2.x, triangle.p3.x, min_x, max_x)
                VX_FINDMINMAX(triangle.p1.y, triangle.p2.y, triangle.p3.y, min_y, max_y)
                VX_FINDMINMAX(triangle.p1.z, triangle.p2.z, triangle.p3.z, min_z, max_z)

                //get voxel indices of bounding box of triangle
                int index_x_min = static_cast<int>(floor((min_x - (-spaceLength/2))/voxelSize));
        int index_x_max = static_cast<int>(floor((max_x - (-spaceLength/2))/voxelSize));
        int index_y_min = static_cast<int>(floor((min_y - (-spaceLength/2))/voxelSize));
        int index_y_max = static_cast<int>(floor((max_y - (-spaceLength/2))/voxelSize));
        int index_z_min = static_cast<int>(floor((min_z - (-spaceLength/2))/voxelSize));
        int index_z_max = static_cast<int>(floor((max_z - (-spaceLength/2))/voxelSize));

        //Check intersection between triangle and voxels in the bounding boxes of triangle
        for (int ind_x = index_x_min; ind_x<index_x_max + 1; ind_x++ ){
            for (int ind_y = index_y_min; ind_y<index_y_max + 1; ind_y++ ){
                for (int ind_z = index_z_min; ind_z<index_z_max + 1; ind_z++ ){
                    boxcenter.x = (-spaceLength/2) + (voxelSize/2) + voxelSize*ind_x;
                    boxcenter.y = (-spaceLength/2) + (voxelSize/2) + voxelSize*ind_y;
                    boxcenter.z = (-spaceLength/2) + (voxelSize/2) + voxelSize*ind_z;
                    if(vx__triangle_box_overlap(boxcenter, halfboxsize, triangle))
                        voxelspace[ind_x][ind_y][ind_z].setStatus(1);
                }
            }
        }
    }
    qDebug() << "The mesh contains " << mesh.num_tris() << " triangles."<<endl;
    qDebug() << "The mesh voxelization took" << timer.elapsed() << "milliseconds"<<endl;
}

void Voxelizer::resize(int size)
{
    voxelspace.resize(size);
    for (int i = 0; i < size; i++)
    {
        voxelspace[i].resize(size);
        for (int j = 0; j < size; j++)
        {
            voxelspace[i][j].resize(size);
        }
    }

}

void Voxelizer::setupSize(float s_Length, float v_Size)
{
    voxelSize = v_Size;
    spaceLength = s_Length;
    halfboxsize.x = voxelSize/2;
    halfboxsize.y = voxelSize/2;
    halfboxsize.z = voxelSize/2;

    //resize voxelspace
    resize(static_cast<int>(spaceLength/voxelSize));
}
