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

bool vx_triangle_box_overlap(vx_vertex_t boxcenter,
                             vx_vertex_t halfboxsize,
                             vx_triangle_t& triangle)
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

void Voxelizer::setupSize(float v_Size, QVector<stl_reader::StlMesh <float, unsigned int>>& STLMeshVector)
{
    //get bounding box for each STL
    QList<double> mtBoundingBox_X_min;
    QList<double> mtBoundingBox_X_max;
    QList<double> mtBoundingBox_Y_min;
    QList<double> mtBoundingBox_Y_max;
    QList<double> mtBoundingBox_Z_min;
    QList<double> mtBoundingBox_Z_max;
    for(int i = 0; i < STLMeshVector.size(); i++){
        mtBoundingBox_X_min.append(STLMeshVector[i].getBoundingBox_X_min());
        mtBoundingBox_X_max.append(STLMeshVector[i].getBoundingBox_X_max());
        mtBoundingBox_Y_min.append(STLMeshVector[i].getBoundingBox_Y_min());
        mtBoundingBox_Y_max.append(STLMeshVector[i].getBoundingBox_Y_max());
        mtBoundingBox_Z_min.append(STLMeshVector[i].getBoundingBox_Z_min());
        mtBoundingBox_Z_max.append(STLMeshVector[i].getBoundingBox_Z_max());
    }

    //Get bounding box coordinates for whole machine tool
    qSort(mtBoundingBox_X_min.begin(), mtBoundingBox_X_min.end());
    qSort(mtBoundingBox_X_max.begin(), mtBoundingBox_X_max.end());
    qSort(mtBoundingBox_Y_min.begin(), mtBoundingBox_Y_min.end());
    qSort(mtBoundingBox_Y_max.begin(), mtBoundingBox_Y_max.end());
    qSort(mtBoundingBox_Z_min.begin(), mtBoundingBox_Z_min.end());
    qSort(mtBoundingBox_Z_max.begin(), mtBoundingBox_Z_max.end());

    double bounding_x_min = mtBoundingBox_X_min.first();
    double bounding_x_max = mtBoundingBox_X_max.last();
    double bounding_y_min = mtBoundingBox_Y_min.first();
    double bounding_y_max = mtBoundingBox_Y_max.last();
    double bounding_z_min = mtBoundingBox_Z_min.first();
    double bounding_z_max = mtBoundingBox_Z_max.last();



    //setup voxel size from the bounding box of whole machine tool
    voxelSize = v_Size;
    VSEnglargeRatio = 1.4f;
    spaceLength_X = (bounding_x_max - bounding_x_min) * VSEnglargeRatio * 1000;
    spaceLength_Y = (bounding_y_max - bounding_y_min) * VSEnglargeRatio* 1000;
    spaceLength_Z = (bounding_z_max - bounding_z_min) * VSEnglargeRatio * 1000;

    halfboxsize.x = voxelSize/2;
    halfboxsize.y = voxelSize/2;
    halfboxsize.z = voxelSize/2;
    voxelSpaceSize_X = static_cast<int>(spaceLength_X/voxelSize);
    voxelSpaceSize_Y = static_cast<int>(spaceLength_Y/voxelSize);
    voxelSpaceSize_Z = static_cast<int>(spaceLength_Z/voxelSize);

    qDebug()<<"Voxel number of Voxel Space in X:"<<voxelSpaceSize_X
           <<"Voxel number of Voxel Space in Y:"<<voxelSpaceSize_Y
          <<"Voxel number of Voxel Space in Z:"<<voxelSpaceSize_Z<<endl;

    voxelStarting_X = (bounding_x_min - (bounding_x_max - bounding_x_min) * (VSEnglargeRatio-1)/2) * 1000;
    voxelStarting_Y = (bounding_y_min - (bounding_y_max - bounding_y_min) * (VSEnglargeRatio-1)/2) * 1000;
    voxelStarting_Z = (bounding_z_min - (bounding_z_max - bounding_z_min) * (VSEnglargeRatio-1)/2) * 1000;
}

void Voxelizer::setupSize(float v_Size, QVector<component> &componentVector)
{
    //get bounding box for each STL
    QList<double> mtBoundingBox_X_min;
    QList<double> mtBoundingBox_X_max;
    QList<double> mtBoundingBox_Y_min;
    QList<double> mtBoundingBox_Y_max;
    QList<double> mtBoundingBox_Z_min;
    QList<double> mtBoundingBox_Z_max;
    for(int i = 0; i < componentVector.size(); i++){
        mtBoundingBox_X_min.append(componentVector[i].getNonOffsetMesh().getBoundingBox_X_min());
        mtBoundingBox_X_max.append(componentVector[i].getNonOffsetMesh().getBoundingBox_X_max());
        mtBoundingBox_Y_min.append(componentVector[i].getNonOffsetMesh().getBoundingBox_Y_min());
        mtBoundingBox_Y_max.append(componentVector[i].getNonOffsetMesh().getBoundingBox_Y_max());
        mtBoundingBox_Z_min.append(componentVector[i].getNonOffsetMesh().getBoundingBox_Z_min());
        mtBoundingBox_Z_max.append(componentVector[i].getNonOffsetMesh().getBoundingBox_Z_max());
    }

    //Get bounding box coordinates for whole machine tool
    qSort(mtBoundingBox_X_min.begin(), mtBoundingBox_X_min.end());
    qSort(mtBoundingBox_X_max.begin(), mtBoundingBox_X_max.end());
    qSort(mtBoundingBox_Y_min.begin(), mtBoundingBox_Y_min.end());
    qSort(mtBoundingBox_Y_max.begin(), mtBoundingBox_Y_max.end());
    qSort(mtBoundingBox_Z_min.begin(), mtBoundingBox_Z_min.end());
    qSort(mtBoundingBox_Z_max.begin(), mtBoundingBox_Z_max.end());

    double bounding_x_min = mtBoundingBox_X_min.first();
    double bounding_x_max = mtBoundingBox_X_max.last();
    double bounding_y_min = mtBoundingBox_Y_min.first();
    double bounding_y_max = mtBoundingBox_Y_max.last();
    double bounding_z_min = mtBoundingBox_Z_min.first();
    double bounding_z_max = mtBoundingBox_Z_max.last();



    //setup voxel size from the bounding box of whole machine tool
    voxelSize = v_Size;
    VSEnglargeRatio = 1.4f;
    spaceLength_X = (bounding_x_max - bounding_x_min) * VSEnglargeRatio * 1000;
    spaceLength_Y = (bounding_y_max - bounding_y_min) * VSEnglargeRatio* 1000;
    spaceLength_Z = (bounding_z_max - bounding_z_min) * VSEnglargeRatio * 1000;

    halfboxsize.x = voxelSize/2;
    halfboxsize.y = voxelSize/2;
    halfboxsize.z = voxelSize/2;
    voxelSpaceSize_X = static_cast<int>(spaceLength_X/voxelSize);
    voxelSpaceSize_Y = static_cast<int>(spaceLength_Y/voxelSize);
    voxelSpaceSize_Z = static_cast<int>(spaceLength_Z/voxelSize);

    qDebug()<<"Voxel number of Voxel Space in X:"<<voxelSpaceSize_X
           <<"Voxel number of Voxel Space in Y:"<<voxelSpaceSize_Y
          <<"Voxel number of Voxel Space in Z:"<<voxelSpaceSize_Z<<endl;

    voxelStarting_X = (bounding_x_min - (bounding_x_max - bounding_x_min) * (VSEnglargeRatio-1)/2) * 1000;
    voxelStarting_Y = (bounding_y_min - (bounding_y_max - bounding_y_min) * (VSEnglargeRatio-1)/2) * 1000;
    voxelStarting_Z = (bounding_z_min - (bounding_z_max - bounding_z_min) * (VSEnglargeRatio-1)/2) * 1000;
}

void Voxelizer::createVoxelSapce()
{
    //setup size for voxelspace
    QVector < QVector < QVector< Voxel > > > correctSizeVS
            (voxelSpaceSize_X, QVector < QVector< Voxel > >(voxelSpaceSize_Y,QVector<Voxel>(voxelSpaceSize_Z)));
    voxelspace = correctSizeVS;
}

void Voxelizer::parentModelVoxelization(Link& link)
{
    float min_x, max_x, min_y, max_y, min_z, max_z;
    QChar linkType = link.getLinkType();

    QVector<stl_reader::StlMesh <float, unsigned int>> meshVector = link.getSTLMesh();
    QMatrix4x4 TransformMatrix = link.m_TransformMatrix;

    QElapsedTimer timer;
    timer.start();

    //go through all components of this link
    for(int mesh_ind = 0; mesh_ind < meshVector.size(); ++mesh_ind) {
        //create empty voxelIndicesList to store parent voxel model of this link
        QList<QVector3D> newParentMTVoxelIndicesList;

        qDebug() <<"Number"<< mesh_ind + 1 << "of the mesh of "<<link.getLinkType()<< "link contains"
                << meshVector[mesh_ind].num_tris() << " triangles."<<endl;

        for (size_t itri = 0; itri < meshVector[mesh_ind].num_tris(); ++itri){

            //Load and transform triangles from mesh
            loadAndTransform(itri, meshVector[mesh_ind], TransformMatrix);

            //find bounding box of triangle
            VX_FINDMINMAX(triangle.p1.x, triangle.p2.x, triangle.p3.x, min_x, max_x)
                    VX_FINDMINMAX(triangle.p1.y, triangle.p2.y, triangle.p3.y, min_y, max_y)
                    VX_FINDMINMAX(triangle.p1.z, triangle.p2.z, triangle.p3.z, min_z, max_z)

                    //get voxel indices of bounding box of triangle
                    int index_x_min = floor((min_x - (voxelStarting_X))/voxelSize);
            int index_x_max = floor((max_x - (voxelStarting_X))/voxelSize);
            int index_y_min = floor((min_y - (voxelStarting_Y))/voxelSize);
            int index_y_max = floor((max_y - (voxelStarting_Y))/voxelSize);
            int index_z_min = floor((min_z - (voxelStarting_Z))/voxelSize);
            int index_z_max = floor((max_z - (voxelStarting_Z))/voxelSize);

            // bounding box of triangle can't be outside of voxelspace
            Q_ASSERT_X((max_x < voxelStarting_X + voxelSpaceSize_X * voxelSize &&
                        max_y < voxelStarting_Y + voxelSpaceSize_Y * voxelSize &&
                        max_z < voxelStarting_Z + voxelSpaceSize_Z * voxelSize &&
                        min_x > voxelStarting_X && min_y > voxelStarting_Y && min_z > voxelStarting_Z), "voxelizer", "part of geometry is outside of voxelspace");

            //Check intersection between triangle and voxels in the bounding boxes of triangle
            for (int ind_x = index_x_min; ind_x<index_x_max + 1; ind_x++){
                for (int ind_y = index_y_min; ind_y<index_y_max + 1; ind_y++){
                    for (int ind_z = index_z_min; ind_z<index_z_max + 1; ind_z++){
                        Voxel& voxel = voxelspace[ind_x][ind_y][ind_z];
                        //                        QChar voxelLinkType = voxel.getVoxelLinkType();

                        boxcenter.x = voxelStarting_X + (voxelSize/2) + voxelSize*ind_x;
                        boxcenter.y = voxelStarting_Y + (voxelSize/2) + voxelSize*ind_y;
                        boxcenter.z = voxelStarting_Z + (voxelSize/2) + voxelSize*ind_z;
                        if(vx_triangle_box_overlap(boxcenter, halfboxsize, triangle)){

                            //                            if(voxelLinkType != 'E' && voxelLinkType != linkType){
                            //                                voxel.collide();
                            //                                link.MTCollidedVoxelIndicesList.append(QVector3D(ind_x, ind_y, ind_z));
                            //                            }

                            voxel.setVoxelLinkType(linkType);
                            voxel.setComponentNumber(mesh_ind + 1);
                            newParentMTVoxelIndicesList.append(QVector3D(ind_x, ind_y, ind_z));
                        }
                    }
                }
            }
        }
        link.MTVoxelIndicesListVector[mesh_ind].append(newParentMTVoxelIndicesList);
        qDebug() << "The mesh voxelization for Number"<<mesh_ind + 1<<"component of" <<link.getLinkType()
                 << "took" << timer.elapsed() << "milliseconds"<<endl;
    }
    link.MTVoxelIndicesListVectorUpdate = link.MTVoxelIndicesListVector;

    //update voxel space assigned to voxel space

    if(link.getLinkType() == 'b')
        link.linkVoxelspace = voxelspace;
}

void Voxelizer::loadAndTransform(size_t itri, stl_reader::StlMesh <float, unsigned int> &mesh, QMatrix4x4 TransformMatrix)
{
    QVector3D vertex1(mesh.vrt_coords(mesh.tri_corner_ind(itri, 0))[0],
            mesh.vrt_coords(mesh.tri_corner_ind(itri, 0))[1],
            mesh.vrt_coords(mesh.tri_corner_ind(itri, 0))[2]);
    vertex1 = TransformMatrix * vertex1;

    QVector3D vertex2(mesh.vrt_coords(mesh.tri_corner_ind(itri, 1))[0],
            mesh.vrt_coords(mesh.tri_corner_ind(itri, 1))[1],
            mesh.vrt_coords(mesh.tri_corner_ind(itri, 1))[2]);
    vertex2 = TransformMatrix * vertex2;

    QVector3D vertex3(mesh.vrt_coords(mesh.tri_corner_ind(itri, 2))[0],
            mesh.vrt_coords(mesh.tri_corner_ind(itri, 2))[1],
            mesh.vrt_coords(mesh.tri_corner_ind(itri, 2))[2]);
    vertex3 = TransformMatrix * vertex3;

    p1.x = 1000 * vertex1.x();
    p1.y = 1000 * vertex1.y();
    p1.z = 1000 * vertex1.z();

    p2.x = 1000 * vertex2.x();
    p2.y = 1000 * vertex2.y();
    p2.z = 1000 * vertex2.z();

    p3.x = 1000 * vertex3.x();
    p3.y = 1000 * vertex3.y();
    p3.z = 1000 * vertex3.z();

    triangle.p1 = p1;
    triangle.p2 = p2;
    triangle.p3 = p3;
}

void Voxelizer::setupInitialTransformationMatrix(MachineTool& MT, float x, float y, float z, float a, float b, float c)
{
    //Transform according to each component
    for (QVector<Joint>::iterator loop = MT.JointVector.begin();loop != MT.JointVector.end(); loop++)
    {
        Link *ParentLink = loop->getParentLink();
        Link *ChildLink = loop->getChildLink();

        switch(ChildLink->getLinkType()) {
        case 'A':
            loop->rotational_motion = a;
            break;
        case 'B':
            loop->rotational_motion = b;
            break;
        case 'C':
            loop->rotational_motion = c;
            break;
        case 'X':
            loop->translational_motion = x;
            break;
        case 'Y':
            loop->translational_motion = y;
            break;
        case 'Z':
            loop->translational_motion = z;
            break;
        }

        if (loop->getType() == "prismatic")
        {
            ChildLink->m_TransformMatrix = ParentLink->m_TransformMatrix;
            ChildLink->m_TransformMatrix.translate(static_cast<float>(loop->getOrigin_xyz().x),static_cast<float>(loop->getOrigin_xyz().y),
                                                   static_cast<float>(loop->getOrigin_xyz().z));
            ChildLink->m_TransformMatrix.translate(static_cast<float>(loop->getAxis().x) * static_cast<float>(loop->translational_motion)
                                                   ,static_cast<float>(loop->getAxis().y) * static_cast<float>(loop->translational_motion),
                                                   static_cast<float>(loop->getAxis().z) * static_cast<float>(loop->translational_motion));
        }

        if (loop->getType() == "revolute")
        {
            ChildLink->m_TransformMatrix = ParentLink->m_TransformMatrix;
            ChildLink->m_TransformMatrix.translate(static_cast<float>(loop->getOrigin_xyz().x),static_cast<float>(loop->getOrigin_xyz().y),
                                                   static_cast<float>(loop->getOrigin_xyz().z));
            ChildLink->m_TransformMatrix.rotate(static_cast<float>(loop->rotational_motion), static_cast<float>(loop->getAxis().x),
                                                static_cast<float>(loop->getAxis().y),static_cast<float>(loop->getAxis().z));
        }
    }


    //setup transformation matrix hastable for machine tool
    //create hash table for components
    for(int link_ind = 0; link_ind < MT.LinkVector.size(); ++link_ind){
        for(int mesh_ind = 0; mesh_ind < MT.LinkVector[link_ind].m_STLMeshVector.size(); ++mesh_ind){
            QString componentName = QString(QChar::fromLatin1(MT.LinkVector[link_ind].getLinkType())) + QString::number(mesh_ind + 1);
            MT.tranMatrixHash[componentName] = MT.LinkVector[link_ind].m_TransformMatrix;
        }
    }
}

//transform triangle mesh
void Voxelizer::setTransformationMatrix(MachineTool &MT, QChar linkType, float amount)
{
    //Transform according to each component
    for (QVector<Joint>::iterator loop = MT.JointVector.begin(); loop != MT.JointVector.end(); loop++)
    {
        Link *ParentLink = loop->getParentLink();
        Link *ChildLink = loop->getChildLink();

        if(ChildLink->getLinkType() == linkType){
            if(ChildLink->isRotaitonal){
                loop->rotational_motion = amount;
            }
            else{
                loop->translational_motion = amount;
            }
        }

        if (loop->getType() == "prismatic")
        {
            ChildLink->m_TransformMatrix = ParentLink->m_TransformMatrix;
            ChildLink->m_TransformMatrix.translate(static_cast<float>(loop->getOrigin_xyz().x),static_cast<float>(loop->getOrigin_xyz().y),
                                                   static_cast<float>(loop->getOrigin_xyz().z));
            ChildLink->m_TransformMatrix.translate(static_cast<float>(loop->getAxis().x) * static_cast<float>(loop->translational_motion)
                                                   ,static_cast<float>(loop->getAxis().y) * static_cast<float>(loop->translational_motion),
                                                   static_cast<float>(loop->getAxis().z) * static_cast<float>(loop->translational_motion));
        }

        if (loop->getType() == "revolute")
        {
            ChildLink->m_TransformMatrix = ParentLink->m_TransformMatrix;
            ChildLink->m_TransformMatrix.translate(static_cast<float>(loop->getOrigin_xyz().x),static_cast<float>(loop->getOrigin_xyz().y),
                                                   static_cast<float>(loop->getOrigin_xyz().z));
            ChildLink->m_TransformMatrix.rotate(static_cast<float>(loop->rotational_motion), static_cast<float>(loop->getAxis().x),
                                                static_cast<float>(loop->getAxis().y),static_cast<float>(loop->getAxis().z));
        }
    }
}

QSet<QString> Voxelizer::translateVoxelModel(MachineTool &MT, QChar movingLinkType, float amount, int samplingNumber)
{
    QSet<QString> totalCollisionSet;

    if((movingLinkType == 'X') | (movingLinkType == 'Y') | (movingLinkType == 'Z')){

        int voxelNumberDistance = amount * 1000.0f / voxelSize;
        float realMovement = voxelNumberDistance * voxelSize;

        //updateTransformationMatrix
        //        setTransformationMatrix(MT, movingLinkType, realMovement / 1000.0f);

        //Transform according to each component
        for (QVector<Joint>::iterator loop = MT.JointVector.begin(); loop != MT.JointVector.end(); loop++)
        {
            Link *ChildLink = loop->getChildLink();

            if(ChildLink->getLinkType() == movingLinkType){
                Link *tralatedLink = ChildLink;
                bool ifEnd = false;

                //loop until no child link
                //                while(!ifEnd){
                while(tralatedLink != nullptr){

                    //timer
                    QElapsedTimer timer;
                    timer.start();

                    totalCollisionSet.unite(translateVoxels(tralatedLink, movingLinkType, voxelNumberDistance,samplingNumber, ifEnd));

                    qDebug() << "Translating"<<tralatedLink->getLinkType()<< "took" << timer.elapsed() << "milliseconds"<<endl;
                    // qDebug() <<tralatedLink->getLinkType()  << " link contains" << tralatedLink->MTVoxelIndicesList.size() << " shell voxels"<<endl;

                    //move pointer to childLink
                    tralatedLink = tralatedLink->ChildLink[0];

                    //                    ifEnd = ((tralatedLink->getLinkType() == 'A') | (tralatedLink->getLinkType() == 'B') | (tralatedLink->getLinkType() == 'C'));
                }
            }
        }
    }
    return totalCollisionSet;
}


QSet<QString> Voxelizer::translateVoxels(Link *link, QChar movingLinkType, int voxelNumberDistance, int samplingNumber, bool ifEnd)
{
    QSet<QString> collisionSet;
    link->MTCollidedVoxelIndicesList.clear();

    QChar currentLinkType = link->getLinkType();

    int voxelNumberDistanceX = 0;
    int voxelNumberDistanceY = 0;
    int voxelNumberDistanceZ = 0;

    if(movingLinkType == 'X'){
        voxelNumberDistanceX = voxelNumberDistance;
    }else if(movingLinkType == 'Y'){
        voxelNumberDistanceY = voxelNumberDistance;
    }else{
        voxelNumberDistanceZ = voxelNumberDistance;
    }
    QVector < QVector < QVector< Voxel > > > newVS = link->ParentLink->linkVoxelspace;

    //traslate indicesListVector---------------------------------------------------------------------

    for(int mesh_ind = 0; mesh_ind < link->MTVoxelIndicesListVector.size(); ++mesh_ind){

        for (QList<QVector3D>::iterator i = link->MTVoxelIndicesListVectorUpdate[mesh_ind][0].begin();
             i != link->MTVoxelIndicesListVectorUpdate[mesh_ind][0].end(); ++i){
            int number_x = i->x();
            int number_y = i->y();
            int number_z = i->z();

            Voxel& voxel = newVS[number_x + voxelNumberDistanceX][number_y + voxelNumberDistanceY]
                    [number_z + voxelNumberDistanceZ];

            QChar ShellType = voxel.getVoxelLinkType();
            QString componentNumber = QString::number(voxel.getComponentNumber());

            voxel.setVoxelLinkType(currentLinkType);
            voxel.setComponentNumber(mesh_ind + 1);

            if(ShellType != 'E' && ShellType != currentLinkType){
                voxel.collide();
                link->MTCollidedVoxelIndicesList.append(QVector3D(number_x, number_y, number_z));

                QString collisionPair;

                collisionPair.append(currentLinkType + QString::number(voxel.getComponentNumber()) +
                                     ShellType + componentNumber);

                if(!collisionSet.contains(collisionPair))
                    collisionSet.insert(collisionPair);
            }

            //update MTVoxelIndiciesList of the link
            i->setX( number_x + voxelNumberDistanceX);
            i->setY( number_y + voxelNumberDistanceY);
            i->setZ( number_z + voxelNumberDistanceZ);
        }
    }

    //traslate indicesListVector---------------------------------------------------------------------

    // update link voxel model bounding box
    link->setBoundingBoxIndex(link->get_x_min_index() + voxelNumberDistanceX, link->get_x_max_index() + voxelNumberDistanceX,
                              link->get_y_min_index() + voxelNumberDistanceY, link->get_y_max_index() + voxelNumberDistanceY,
                              link->get_z_min_index() + voxelNumberDistanceZ, link->get_z_max_index() + voxelNumberDistanceZ);
    //update link voxel space
    link->linkVoxelspace = newVS;
    //    voxelspace = newVS;

    return collisionSet;
}

int Voxelizer::indexOfCompVector(QString compName, QVector<component> &compVector)
{
    int index = -1;
    for(int comp_ind = 0; comp_ind < compVector.size(); ++comp_ind){
        if(compVector[comp_ind].getName() == compName)
            index = comp_ind;
    }
    return index;
}

void Voxelizer::shiftVoxelModel(MachineTool &MT,float amountX, float amountY, float amountZ)
{
    int voxelNumberDistanceX = amountX * 1000.0f / voxelSize;
    int voxelNumberDistanceY = amountY * 1000.0f / voxelSize;
    int voxelNumberDistanceZ = amountZ * 1000.0f / voxelSize;

    for(int Number = 0; Number < MT.baseLink->ChildLink.size(); Number++){
        bool switchX = false;
        bool switchY = false;
        bool switchZ = false;
        Link* currentLink = MT.baseLink->ChildLink[Number];

        while(currentLink != nullptr){
            QSet<QString> collisionSet;
            currentLink->MTCollidedVoxelIndicesList.clear();
            QChar currentLinkType = currentLink->getLinkType();
            if (currentLinkType == 'X')
                switchX = true;
            if (currentLinkType == 'Y')
                switchY = true;
            if (currentLinkType == 'Z')
                switchZ = true;

            //reset MTVoxelIndicesListVectorUpdate to original position
            currentLink->MTVoxelIndicesListVectorUpdate = currentLink->MTVoxelIndicesListVector;

            for(int mesh_ind = 0; mesh_ind < currentLink->MTVoxelIndicesListVector.size(); ++mesh_ind){
                for(int parentVoxel_ind = 0; parentVoxel_ind < currentLink->MTVoxelIndicesListVector[mesh_ind].size(); ++parentVoxel_ind){

                    for (QList<QVector3D>::iterator i = currentLink->MTVoxelIndicesListVectorUpdate[mesh_ind][parentVoxel_ind].begin();
                         i != currentLink->MTVoxelIndicesListVectorUpdate[mesh_ind][parentVoxel_ind].end(); ++i){

                        //update MTVoxelIndiciesList of the link
                        if(switchX)
                            i->setX(i->x() + voxelNumberDistanceX);
                        if(switchY)
                            i->setY(i->y() + voxelNumberDistanceY);
                        if(switchZ)
                            i->setZ(i->z() + voxelNumberDistanceZ);
                    }
                }
            }
            if(!currentLink->ChildLink.isEmpty()){
                currentLink = currentLink->ChildLink[0];
            }else{
                break;
            }
        }
    }
}

QSet<QString> Voxelizer::collisionDetectionForGroups(MachineTool &MT, int ind1, int ind2)
{
    QSet<QString> totalCollisionSet;

    voxelspace = MT.baseLink->linkVoxelspace;


    for(int Number = 0; Number < MT.baseLink->ChildLink.size(); Number++){
        Link *currentLink = MT.baseLink->ChildLink[Number];

        while(currentLink != nullptr){
            QSet<QString> collisionSet;
            currentLink->MTCollidedVoxelIndicesList.clear();
            QChar currentLinkType = currentLink->getLinkType();
            int index = 0;

            if(currentLink->isFirstRotational)
                index = ind1;

            if(currentLink->isSecondRotational)
                index = ind2;

            for(int mesh_ind = 0; mesh_ind < currentLink->MTVoxelIndicesListVector.size(); ++mesh_ind){

                for (QList<QVector3D>::iterator i = currentLink->MTVoxelIndicesListVectorUpdate[mesh_ind][index].begin();
                     i != currentLink->MTVoxelIndicesListVectorUpdate[mesh_ind][index].end(); ++i){
                    int number_x = i->x();
                    int number_y = i->y();
                    int number_z = i->z();

                    Voxel& voxel = voxelspace[number_x][number_y][number_z];

                    QChar ShellType = voxel.getVoxelLinkType();
                    QString componentNumber = QString::number(voxel.getComponentNumber());

                    voxel.setVoxelLinkType(currentLinkType);
                    voxel.setComponentNumber(mesh_ind + 1);

                    if(ShellType != 'E' && ShellType != currentLinkType){
                        voxel.collide();
                        currentLink->MTCollidedVoxelIndicesList.append(QVector3D(number_x, number_y, number_z));

                        QString collisionPair;

                        collisionPair.append(currentLinkType + QString::number(voxel.getComponentNumber()) +
                                             ShellType + componentNumber);

                        if(!collisionSet.contains(collisionPair))
                            collisionSet.insert(collisionPair);
                    }
                }
            }
            totalCollisionSet.unite(collisionSet);

            if(!currentLink->ChildLink.isEmpty()){
                currentLink = currentLink->ChildLink[0];
            }else{
                break;
            }
        }
    }
    return totalCollisionSet;
}

QVector<contactComponentsPair> Voxelizer::collisionDetectionForComponents(QVector<component>& componentVector)
{
    qDebug()<<"collisionDetectionForComponents-------------------------------"<<endl;

    QSet<QString> contactComponentsPairsSet;
    QVector<contactComponentsPair> ccpVector;

    //setup size for voxelspace for finding contact-components pairs
    QVector < QVector < QVector< voxelForCCP > > > contactComponentsPairsVS
            (voxelSpaceSize_X, QVector < QVector< voxelForCCP > >(voxelSpaceSize_Y,QVector<voxelForCCP>(voxelSpaceSize_Z)));
    float min_x, max_x, min_y, max_y, min_z, max_z;

    for(int comp_ind = 0; comp_ind < componentVector.size(); ++comp_ind){
        QString componentName = componentVector[comp_ind].getName();
        stl_reader::StlMesh <float, unsigned int>& nonOffestMesh = componentVector[comp_ind].getNonOffsetMesh();

        for (size_t itri = 0; itri < nonOffestMesh.num_tris(); ++itri){

            //get triangl information for voxel transformation
            QVector3D vertex1(nonOffestMesh.vrt_coords(nonOffestMesh.tri_corner_ind(itri, 0))[0],
                    nonOffestMesh.vrt_coords(nonOffestMesh.tri_corner_ind(itri, 0))[1],
                    nonOffestMesh.vrt_coords(nonOffestMesh.tri_corner_ind(itri, 0))[2]);

            QVector3D vertex2(nonOffestMesh.vrt_coords(nonOffestMesh.tri_corner_ind(itri, 1))[0],
                    nonOffestMesh.vrt_coords(nonOffestMesh.tri_corner_ind(itri, 1))[1],
                    nonOffestMesh.vrt_coords(nonOffestMesh.tri_corner_ind(itri, 1))[2]);

            QVector3D vertex3(nonOffestMesh.vrt_coords(nonOffestMesh.tri_corner_ind(itri, 2))[0],
                    nonOffestMesh.vrt_coords(nonOffestMesh.tri_corner_ind(itri, 2))[1],
                    nonOffestMesh.vrt_coords(nonOffestMesh.tri_corner_ind(itri, 2))[2]);

            p1.x = 1000 * vertex1.x();
            p1.y = 1000 * vertex1.y();
            p1.z = 1000 * vertex1.z();

            p2.x = 1000 * vertex2.x();
            p2.y = 1000 * vertex2.y();
            p2.z = 1000 * vertex2.z();

            p3.x = 1000 * vertex3.x();
            p3.y = 1000 * vertex3.y();
            p3.z = 1000 * vertex3.z();

            triangle.p1 = p1;
            triangle.p2 = p2;
            triangle.p3 = p3;

            //find bounding box of triangle
            VX_FINDMINMAX(triangle.p1.x, triangle.p2.x, triangle.p3.x, min_x, max_x)
                    VX_FINDMINMAX(triangle.p1.y, triangle.p2.y, triangle.p3.y, min_y, max_y)
                    VX_FINDMINMAX(triangle.p1.z, triangle.p2.z, triangle.p3.z, min_z, max_z)

                    //get voxel indices of bounding box of triangle
                    int index_x_min = floor((min_x - (voxelStarting_X))/voxelSize);
            int index_x_max = floor((max_x - (voxelStarting_X))/voxelSize);
            int index_y_min = floor((min_y - (voxelStarting_Y))/voxelSize);
            int index_y_max = floor((max_y - (voxelStarting_Y))/voxelSize);
            int index_z_min = floor((min_z - (voxelStarting_Z))/voxelSize);
            int index_z_max = floor((max_z - (voxelStarting_Z))/voxelSize);

            // bounding box of triangle can't be outside of voxelspace
            Q_ASSERT_X((max_x < voxelStarting_X + voxelSpaceSize_X * voxelSize &&
                        max_y < voxelStarting_Y + voxelSpaceSize_Y * voxelSize &&
                        max_z < voxelStarting_Z + voxelSpaceSize_Z * voxelSize &&
                        min_x > voxelStarting_X && min_y > voxelStarting_Y && min_z > voxelStarting_Z), "voxelizer", "part of geometry is outside of voxelspace");

            //Check intersection between triangle and voxels in the bounding boxes of triangle
            for (int ind_x = index_x_min; ind_x<index_x_max + 1; ind_x++){
                for (int ind_y = index_y_min; ind_y<index_y_max + 1; ind_y++){
                    for (int ind_z = index_z_min; ind_z<index_z_max + 1; ind_z++){
                        voxelForCCP& voxelForCpp = contactComponentsPairsVS[ind_x][ind_y][ind_z];
                        QVector<QString>& voxelCompVector = voxelForCpp.componentsVector;

                        boxcenter.x = voxelStarting_X + (voxelSize/2) + voxelSize*ind_x;
                        boxcenter.y = voxelStarting_Y + (voxelSize/2) + voxelSize*ind_y;
                        boxcenter.z = voxelStarting_Z + (voxelSize/2) + voxelSize*ind_z;

                        // for voxel that intersect with 3d model
                        if(vx_triangle_box_overlap(boxcenter, halfboxsize, triangle)){
                            // newParentMTVoxelIndicesList.append(QVector3D(ind_x, ind_y, ind_z));
                            if(voxelCompVector.isEmpty()){
                                //if there is no component assigned to this voxel
                                voxelCompVector.push_back(componentName);
                            }else{
                                if(!voxelCompVector.contains(componentName)){
                                    //if this component hasn't assigned to this voxel
                                    for (int component_ind = 0; component_ind < voxelCompVector.size(); component_ind++){
                                        QString ccpName;
                                        ccpName.append(componentName + "==" + voxelCompVector[component_ind]);

                                        if(!contactComponentsPairsSet.contains(ccpName)){
                                            contactComponentsPairsSet.insert(ccpName);
                                            //create an empty ccp
                                            contactComponentsPair ccp;
                                            ccp.setFirstMeshName(componentName);
                                            ccp.setSecondMeshName(voxelCompVector[component_ind]);
                                            ccp.setName(ccpName);
                                            ccpVector.append(ccp);
                                            qDebug()<<ccp.getName();
                                        }
                                    }
                                    voxelCompVector.push_back(componentName);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    for(QVector<contactComponentsPair>::iterator loop = ccpVector.begin();loop != ccpVector.end(); loop++){
        qDebug()<<loop->getName();
        loop->setFirstComp(componentVector[indexOfCompVector(loop->getFirstMeshName(),componentVector)]);
        loop->setSecondComp(componentVector[indexOfCompVector(loop->getSecondMeshName(),componentVector)]);
        qDebug()<<"First mesh contain"<<loop->getFirstComp().getNonOffsetMesh().num_tris()<<"triangles";
        qDebug()<<"Second mesh contain"<<loop->getSecondComp().getNonOffsetMesh().num_tris()<<"triangles"<<endl;
    }

    qDebug()<<"All contact-components pairs:";
    for(QVector<contactComponentsPair>::iterator loop = ccpVector.begin();loop != ccpVector.end(); loop++){
        qDebug()<<loop->getName();
    }
    qDebug()<<"There are"<<ccpVector.size()<<"ccp"<<endl;

    return ccpVector;
}

QVector<contactComponentsPair> Voxelizer::collisionDetectionForComponentsFromURDF(MachineTool &MT)
{

    qDebug()<<"collisionDetectionForComponentsFromURDF-------------------------------"<<endl;

    QSet<QString> contactComponentsPairsSet;
    QVector<contactComponentsPair> ccpVector;
    //setup size for voxelspace for finding contact-components pairs
    QVector < QVector < QVector< voxelForCCP > > > contactComponentsPairsVS
            (voxelSpaceSize_X, QVector < QVector< voxelForCCP > >(voxelSpaceSize_Y,QVector<voxelForCCP>(voxelSpaceSize_Z)));
    float min_x, max_x, min_y, max_y, min_z, max_z;

    //collision detection for each link
    for(int link_ind = 0; link_ind < MT.LinkVector.size(); ++link_ind){
        Link& link = MT.LinkVector[link_ind];
        QVector<stl_reader::StlMesh <float, unsigned int>> meshVector = link.getSTLMesh();
        QMatrix4x4 TransformMatrix = link.m_TransformMatrix;

        //go through all components of this link
        for(int mesh_ind = 0; mesh_ind < meshVector.size(); ++mesh_ind) {

            //            //create empty voxelIndicesList to store parent voxel model of this link
            //            QList<QVector3D> newParentMTVoxelIndicesList;

            QString componentName = QString(QChar::fromLatin1(link.getLinkType())) + QString::number(mesh_ind + 1);

            qDebug()<<componentName<<endl;

            qDebug() <<"Number"<< mesh_ind + 1 << "of the mesh of "<<link.getLinkType()<< "link contains"
                    << meshVector[mesh_ind].num_tris() << " triangles."<<endl;

            for (size_t itri = 0; itri < meshVector[mesh_ind].num_tris(); ++itri){

                //Load and transform triangles from mesh
                loadAndTransform(itri, meshVector[mesh_ind], TransformMatrix);

                //find bounding box of triangle
                VX_FINDMINMAX(triangle.p1.x, triangle.p2.x, triangle.p3.x, min_x, max_x)
                        VX_FINDMINMAX(triangle.p1.y, triangle.p2.y, triangle.p3.y, min_y, max_y)
                        VX_FINDMINMAX(triangle.p1.z, triangle.p2.z, triangle.p3.z, min_z, max_z)

                        //get voxel indices of bounding box of triangle
                        int index_x_min = floor((min_x - (voxelStarting_X))/voxelSize);
                int index_x_max = floor((max_x - (voxelStarting_X))/voxelSize);
                int index_y_min = floor((min_y - (voxelStarting_Y))/voxelSize);
                int index_y_max = floor((max_y - (voxelStarting_Y))/voxelSize);
                int index_z_min = floor((min_z - (voxelStarting_Z))/voxelSize);
                int index_z_max = floor((max_z - (voxelStarting_Z))/voxelSize);

                // bounding box of triangle can't be outside of voxelspace
                Q_ASSERT_X((max_x < voxelStarting_X + voxelSpaceSize_X * voxelSize &&
                            max_y < voxelStarting_Y + voxelSpaceSize_Y * voxelSize &&
                            max_z < voxelStarting_Z + voxelSpaceSize_Z * voxelSize &&
                            min_x > voxelStarting_X && min_y > voxelStarting_Y && min_z > voxelStarting_Z), "voxelizer", "part of geometry is outside of voxelspace");

                //Check intersection between triangle and voxels in the bounding boxes of triangle
                for (int ind_x = index_x_min; ind_x<index_x_max + 1; ind_x++){
                    for (int ind_y = index_y_min; ind_y<index_y_max + 1; ind_y++){
                        for (int ind_z = index_z_min; ind_z<index_z_max + 1; ind_z++){
                            voxelForCCP& voxelForCpp = contactComponentsPairsVS[ind_x][ind_y][ind_z];
                            QVector<QString>& componentsVector = voxelForCpp.componentsVector;

                            boxcenter.x = voxelStarting_X + (voxelSize/2) + voxelSize*ind_x;
                            boxcenter.y = voxelStarting_Y + (voxelSize/2) + voxelSize*ind_y;
                            boxcenter.z = voxelStarting_Z + (voxelSize/2) + voxelSize*ind_z;

                            // for voxel that intersect with 3d model
                            if(vx_triangle_box_overlap(boxcenter, halfboxsize, triangle)){

                                //                                newParentMTVoxelIndicesList.append(QVector3D(ind_x, ind_y, ind_z));

                                if(componentsVector.isEmpty()){
                                    //if there is no component assigned to this voxel

                                    componentsVector.push_back(componentName);
                                }else{
                                    if(!componentsVector.contains(componentName)){
                                        //if this component hasn't assigned to this voxel
                                        for (int component_ind = 0; component_ind < componentsVector.size(); component_ind++){

                                            QString ccpName;
                                            ccpName.append(componentName + componentsVector[component_ind]);

                                            if(!contactComponentsPairsSet.contains(ccpName)){
                                                contactComponentsPairsSet.insert(ccpName);
                                                //create an empty ccp
                                                contactComponentsPair ccp;

                                                ccp.setFirstMeshName(componentName);
                                                ccp.setSecondMeshName(componentsVector[component_ind]);
                                                ccp.setName(ccpName);

                                                qDebug()<<ccp.getName();
                                                //append to cppVector
                                                ccpVector.append(ccp);
                                            }
                                        }
                                        componentsVector.push_back(componentName);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            //            link.MTVoxelIndicesListVector[mesh_ind].append(newParentMTVoxelIndicesList);
        }
        //        link.MTVoxelIndicesListVectorUpdate = link.MTVoxelIndicesListVector;
    }

    qDebug()<<"There are"<<ccpVector.size()<<"ccp"<<endl;
    for(QVector<contactComponentsPair>::iterator loop = ccpVector.begin();loop != ccpVector.end(); loop++){
        qDebug()<<loop->getName();
        loop->setFirstMesh(MT.componentsHash[loop->getFirstMeshName()]);
        loop->setSecondMesh(MT.componentsHash[loop->getSecondMeshName()]);
        loop->firstTransformMatrix = MT.tranMatrixHash[loop->getFirstMeshName()];
        loop->secondTransformMatrix = MT.tranMatrixHash[loop->getSecondMeshName()];
        qDebug()<<"First mesh contain"<<loop->getFirstMesh().num_tris()<<"triangles";
        qDebug()<<"Second mesh contain"<<loop->getSecondMesh().num_tris()<<"triangles"<<endl;
    }

    QList<QString> CCPList = contactComponentsPairsSet.toList();
    qSort(CCPList.begin() , CCPList.end());

    qDebug()<<"All contact-components pairs:"<<contactComponentsPairsSet<<endl;
    qDebug()<<"CCP has"<<CCPList.size()<<"pairs"<<endl;
    return ccpVector;
}

void Voxelizer::updateCCPVector(QVector<contactComponentsPair>& ccpVector)
{
    //check x positive
    QMatrix4x4 transformMatrix_X_Positive;
    transformMatrix_X_Positive.setToIdentity();
    transformMatrix_X_Positive.translate(0.05f,0.0f,0.0f);

    for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
        qDebug()<<"colliion detection in X positive for"<<ccpVector[ccp_ind].getName();
        bool isCollided = collisionDetectionForCCP(ccpVector[ccp_ind], transformMatrix_X_Positive);

        if(isCollided)
            ccpVector[ccp_ind].collided_Positive_X();
    }

    //check x negative
    QMatrix4x4 transformMatrix_X_Negative;
    transformMatrix_X_Negative.setToIdentity();
    transformMatrix_X_Negative.translate(-0.05f,0.0f,0.0f);

    for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
        qDebug()<<"colliion detection in X negative for"<<ccpVector[ccp_ind].getName();
        bool isCollided = collisionDetectionForCCP(ccpVector[ccp_ind], transformMatrix_X_Negative);

        if(isCollided)
            ccpVector[ccp_ind].collided_Negative_X();
    }

    //check y positive
    QMatrix4x4 transformMatrix_Y_Positive;
    transformMatrix_Y_Positive.setToIdentity();
    transformMatrix_Y_Positive.translate(0.0f,0.05f,0.0f);

    for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
        qDebug()<<"colliion detection in Y positive for"<<ccpVector[ccp_ind].getName();
        bool isCollided = collisionDetectionForCCP(ccpVector[ccp_ind], transformMatrix_Y_Positive);

        if(isCollided)
            ccpVector[ccp_ind].collided_Positive_Y();
    }

    //check y negative
    QMatrix4x4 transformMatrix_Y_Negative;
    transformMatrix_Y_Negative.setToIdentity();
    transformMatrix_Y_Negative.translate(0.0f,-0.05f,0.0f);

    for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
        qDebug()<<"colliion detection in Y negative for"<<ccpVector[ccp_ind].getName();
        bool isCollided = collisionDetectionForCCP(ccpVector[ccp_ind], transformMatrix_Y_Negative);

        if(isCollided)
            ccpVector[ccp_ind].collided_Negative_Y();
    }

    //check z positive
    QMatrix4x4 transformMatrix_Z_Positive;
    transformMatrix_Z_Positive.setToIdentity();
    transformMatrix_Z_Positive.translate(0.0f,0.0f,0.05f);

    for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
        qDebug()<<"colliion detection in Z positive for"<<ccpVector[ccp_ind].getName();
        bool isCollided = collisionDetectionForCCP(ccpVector[ccp_ind], transformMatrix_Z_Positive);

        if(isCollided)
            ccpVector[ccp_ind].collided_Positive_Z();
    }


    //check z negative
    QMatrix4x4 transformMatrix_Z_Negative;
    transformMatrix_Z_Negative.setToIdentity();
    transformMatrix_Z_Negative.translate(0.0f,0.0f,-0.05f);

    for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
        qDebug()<<"colliion detection in Z negative for"<<ccpVector[ccp_ind].getName();
        bool isCollided = collisionDetectionForCCP(ccpVector[ccp_ind], transformMatrix_Z_Negative);

        if(isCollided)
            ccpVector[ccp_ind].collided_Negative_Z();
    }

    for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
        qDebug()<<"Name:"<<ccpVector[ccp_ind].getName();
        qDebug()<<"Collision occurs when move in positive X direction:"<<ccpVector[ccp_ind].isCollided_Positive_X();
        qDebug()<<"Collision occurs when move in negative X:"<<ccpVector[ccp_ind].isCollided_Negative_X();
        qDebug()<<"Collision occurs when move in positive Y:"<<ccpVector[ccp_ind].isCollided_Positive_Y();
        qDebug()<<"Collision occurs when move in negative Y:"<<ccpVector[ccp_ind].isCollided_Negative_Y();
        qDebug()<<"Collision occurs when move in positive Z:"<<ccpVector[ccp_ind].isCollided_Positive_Z();
        qDebug()<<"Collision occurs when move in negative Z:"<<ccpVector[ccp_ind].isCollided_Negative_Z();
    }

}

bool Voxelizer::collisionDetectionForCCP(contactComponentsPair &ccp, QMatrix4x4 movingtransformMatrix)
{
    //    qDebug()<<"collision detection for CCP:"<<ccp.getName()<<endl;

    bool isCollided = false;

    QVector < QVector < QVector< voxelForCCP > > > contactComponentsPairsVS
            (voxelSpaceSize_X, QVector < QVector< voxelForCCP > >(voxelSpaceSize_Y,QVector<voxelForCCP>(voxelSpaceSize_Z)));
    float min_x, max_x, min_y, max_y, min_z, max_z;

    for (int component_ind = 0; component_ind < 2; ++component_ind){
        stl_reader::StlMesh <float, unsigned int> componentMesh;
        QMatrix4x4 transformMatrix;
        transformMatrix.setToIdentity();
        QString componentName;

        //first component
        if(component_ind == 0){
            if(ccp.getFirstComp().containsOffsetMesh()){
                componentMesh = ccp.getFirstComp().getOffsetMesh();
                componentName = ccp.getFirstComp().getName() + "_offset";
                transformMatrix = transformMatrix* movingtransformMatrix;
            }else{
                componentMesh = ccp.getFirstComp().getNonOffsetMesh();
                componentName = ccp.getFirstComp().getName();
                transformMatrix = transformMatrix* movingtransformMatrix;
            }
        }else{
            //second component
            if(ccp.getSecondComp().containsOffsetMesh()){
                componentMesh = ccp.getSecondComp().getOffsetMesh();
                componentName = ccp.getSecondComp().getName() + "_offset";
            }else{
                componentMesh = ccp.getSecondComp().getNonOffsetMesh();
                componentName = ccp.getSecondComp().getName();
            }
        }

        for (size_t itri = 0; itri < componentMesh.num_tris(); ++itri){

            //Load and transform triangles from mesh
            loadAndTransform(itri, componentMesh, transformMatrix);

            //find bounding box of triangle
            VX_FINDMINMAX(triangle.p1.x, triangle.p2.x, triangle.p3.x, min_x, max_x)
                    VX_FINDMINMAX(triangle.p1.y, triangle.p2.y, triangle.p3.y, min_y, max_y)
                    VX_FINDMINMAX(triangle.p1.z, triangle.p2.z, triangle.p3.z, min_z, max_z)

                    //get voxel indices of bounding box of triangle
                    int index_x_min = floor((min_x - (voxelStarting_X))/voxelSize);
            int index_x_max = floor((max_x - (voxelStarting_X))/voxelSize);
            int index_y_min = floor((min_y - (voxelStarting_Y))/voxelSize);
            int index_y_max = floor((max_y - (voxelStarting_Y))/voxelSize);
            int index_z_min = floor((min_z - (voxelStarting_Z))/voxelSize);
            int index_z_max = floor((max_z - (voxelStarting_Z))/voxelSize);

            // bounding box of triangle can't be outside of voxelspace
            Q_ASSERT_X((max_x < voxelStarting_X + voxelSpaceSize_X * voxelSize &&
                        max_y < voxelStarting_Y + voxelSpaceSize_Y * voxelSize &&
                        max_z < voxelStarting_Z + voxelSpaceSize_Z * voxelSize &&
                        min_x > voxelStarting_X && min_y > voxelStarting_Y && min_z > voxelStarting_Z), "voxelizer", "part of geometry is outside of voxelspace");

            //Check intersection between triangle and voxels in the bounding boxes of triangle
            for (int ind_x = index_x_min; ind_x<index_x_max + 1; ind_x++){
                for (int ind_y = index_y_min; ind_y<index_y_max + 1; ind_y++){
                    for (int ind_z = index_z_min; ind_z<index_z_max + 1; ind_z++){
                        voxelForCCP& voxelForCpp = contactComponentsPairsVS[ind_x][ind_y][ind_z];
                        QVector<QString>& componentsVector = voxelForCpp.componentsVector;

                        boxcenter.x = voxelStarting_X + (voxelSize/2) + voxelSize*ind_x;
                        boxcenter.y = voxelStarting_Y + (voxelSize/2) + voxelSize*ind_y;
                        boxcenter.z = voxelStarting_Z + (voxelSize/2) + voxelSize*ind_z;

                        // for voxel that intersect with 3d model
                        if(vx_triangle_box_overlap(boxcenter, halfboxsize, triangle)){

                            //if there is no component assigned to this voxel
                            if(componentsVector.isEmpty()){
                                componentsVector.push_back(componentName);
                            }else{
                                //if this component hasn't assigned to this voxel
                                if(!componentsVector.contains(componentName)){
                                    //collision occurs
                                    isCollided = true;
//                                    goto endOfLoop;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
//endOfLoop:
    return isCollided;
}



