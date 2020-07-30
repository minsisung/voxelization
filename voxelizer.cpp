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

void Voxelizer::setupSize(float s_Length, float v_Size)
{
    voxelSize = v_Size;
    spaceLength = s_Length;
    halfboxsize.x = voxelSize/2;
    halfboxsize.y = voxelSize/2;
    halfboxsize.z = voxelSize/2;
    voxelSpaceSize = static_cast<int>(spaceLength/voxelSize);

    //setup size for voxelspace
    QVector < QVector < QVector< Voxel > > > correctSizeVS(voxelSpaceSize, QVector < QVector< Voxel > >(voxelSpaceSize, QVector<Voxel>(voxelSpaceSize)));
    voxelspace = correctSizeVS;
}

void Voxelizer::Voxelize(Link& link, bool needVisualization)
{
    qDebug() << "The mesh of "<<link.getLinkType()<< "contains" << link.getSTLMesh().num_tris() << " triangles."<<endl;

    if(link.ParentLink != nullptr)
        voxelspace = link.ParentLink->linkVoxelspace;

    QElapsedTimer timer;
    timer.start();

    parentModelVoxelization(link, needVisualization);

    qDebug() << "The mesh voxelization for" <<link.getLinkType()<< "took" << timer.elapsed() << "milliseconds"<<endl;

}

void Voxelizer::parentModelVoxelization(Link& link, bool needVisualization)
{
    //create empty voxelIndicesList to store parent voxel model of this link
    QList<QVector3D> newMotherMTOutterVoxelIndicesList;

    //    QList<QVector3D>& currentMTOutterVoxelIndicesList = link.MTOutterVoxelIndicesListVector.last();

    float min_x, max_x, min_y, max_y, min_z, max_z;
    char linkType = link.getLinkType();
    float voxelStartingCenter = (-spaceLength/2) + (voxelSize/2);
    stl_reader::StlMesh <float, unsigned int> mesh = link.getSTLMesh();
    QMatrix4x4 TransformMatrix = link.m_TransformMatrix;

    for (size_t itri = 0; itri < mesh.num_tris(); ++itri){

        //Load and transform triangles from mesh
        loadAndTransform(itri, mesh, TransformMatrix);

        //find bounding box of triangle
        VX_FINDMINMAX(triangle.p1.x, triangle.p2.x, triangle.p3.x, min_x, max_x)
                VX_FINDMINMAX(triangle.p1.y, triangle.p2.y, triangle.p3.y, min_y, max_y)
                VX_FINDMINMAX(triangle.p1.z, triangle.p2.z, triangle.p3.z, min_z, max_z)

                //get voxel indices of bounding box of triangle
                int index_x_min = floor((min_x - (-spaceLength/2))/voxelSize);
        int index_x_max = floor((max_x - (-spaceLength/2))/voxelSize);
        int index_y_min = floor((min_y - (-spaceLength/2))/voxelSize);
        int index_y_max = floor((max_y - (-spaceLength/2))/voxelSize);
        int index_z_min = floor((min_z - (-spaceLength/2))/voxelSize);
        int index_z_max = floor((max_z - (-spaceLength/2))/voxelSize);

        // bounding box of triangle can't be outside of voxelspace
        Q_ASSERT_X((max_x < spaceLength/2 && max_y < spaceLength/2 && max_z < spaceLength/2 &&
                    min_x > -spaceLength/2 && min_y > -spaceLength/2 && min_z > -spaceLength/2), "voxelizer", "part of geometry is outside of voxelspace");

        //setup the bounding voxel index of the geometry to speed up cubes creation when visualization is necessary
        //(index_z_min for z component is smaller for swept volume)
        if(needVisualization)
            set_bounding_voxel_index(index_x_min, index_x_max, index_y_min, index_y_max, index_z_min, index_z_max);

        //Check intersection between triangle and voxels in the bounding boxes of triangle
        for (int ind_x = index_x_min; ind_x<index_x_max + 1; ind_x++ ){
            for (int ind_y = index_y_min; ind_y<index_y_max + 1; ind_y++ ){
                for (int ind_z = index_z_min; ind_z<index_z_max + 1; ind_z++ ){
                    Voxel& voxel = voxelspace[ind_x][ind_y][ind_z];
                    //char outterShellType = voxel.getOutterShellLinkType();
                    char innerShellType = voxel.getInnerShellLinkType();

                    boxcenter.x = voxelStartingCenter + voxelSize*ind_x;
                    boxcenter.y = voxelStartingCenter + voxelSize*ind_y;
                    boxcenter.z = voxelStartingCenter + voxelSize*ind_z;
                    if(vx_triangle_box_overlap(boxcenter, halfboxsize, triangle)){

                        //                        if(outterShellType != 'E' && outterShellType != linkType){
                        //                            voxel.coincident();
                        //                        }

                        //                        if(innerShellType != 'E' && innerShellType != linkType){
                        //                            voxel.collide();
                        //                        }

                        if(innerShellType == linkType)
                            voxel.setInnerShellLinkType('E');

                        voxel.setOutterShellLinkType(linkType);
                        link.MTOutterVoxelIndicesList.append(QVector3D(ind_x, ind_y, ind_z));

                        newMotherMTOutterVoxelIndicesList.append(QVector3D(ind_x, ind_y, ind_z));

                        //fill inner shell
                        fillInnerShell(linkType, ind_x, ind_y, ind_z, mesh.tri_normal(itri));
                    }
                }
            }
        }
    }


    link.MTOutterVoxelIndicesListVector.append(newMotherMTOutterVoxelIndicesList);

    //set bounding box for link
    link.setBoundingBoxIndex(bounding_x_min_index, bounding_x_max_index, bounding_y_min_index,bounding_y_max_index,
                             bounding_z_min_index, bounding_z_max_index);

    //update voxel space assigned to voxel space
    link.linkVoxelspace = voxelspace;

    for (int number_x = link.get_x_min_index(); number_x < link.get_x_max_index() + 1; ++number_x) {
        for (int number_y = link.get_y_min_index(); number_y < link.get_y_max_index() + 1; ++number_y) {
            for (int number_z = link.get_z_min_index(); number_z < link.get_z_max_index() + 1; ++number_z) {
                if(link.linkVoxelspace[number_x][number_y][number_z].getInnerShellLinkType() != linkType)
                    continue;

                link.MTInnerVoxelIndicesList.append(QVector3D(number_x, number_y, number_z));
            }
        }
    }
}

void Voxelizer::set_bounding_voxel_index(int index_x_min, int index_x_max, int index_y_min, int index_y_max, int index_z_min, int index_z_max)
{
    if(index_x_min < bounding_x_min_index)
        bounding_x_min_index = index_x_min;

    if(index_x_max > bounding_x_max_index)
        bounding_x_max_index = index_x_max;

    if(index_y_min < bounding_y_min_index)
        bounding_y_min_index = index_y_min;

    if(index_y_max > bounding_y_max_index)
        bounding_y_max_index = index_y_max;

    if(index_z_min < bounding_z_min_index)
        bounding_z_min_index = index_z_min;

    if(index_z_max > bounding_z_max_index)
        bounding_z_max_index = index_z_max;
}

void Voxelizer::reset_bounding_index()
{
    bounding_x_min_index = std::numeric_limits<int>::max();
    bounding_x_max_index = 0;
    bounding_y_min_index = std::numeric_limits<int>::max();
    bounding_y_max_index = 0;
    bounding_z_min_index = std::numeric_limits<int>::max();
    bounding_z_max_index = 0;
}

void Voxelizer::loadAndTransform(size_t itri, stl_reader::StlMesh <float, unsigned int>& mesh, QMatrix4x4 TransformMatrix)
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
}

//transform triangle mesh
void Voxelizer::setTransformationMatrix(MachineTool &MT, char linkType, float amount)
{
    //Transform according to each component
    for (QVector<Joint>::iterator loop = MT.JointVector.begin(); loop != MT.JointVector.end(); loop++)
    {
        Link *ParentLink = loop->getParentLink();
        Link *ChildLink = loop->getChildLink();

        if(ChildLink->getLinkType() == linkType){
            if(linkType == 'A' | linkType == 'B' | linkType == 'C'){
                loop->rotational_motion = amount;
            }
            else if(linkType == 'X' | linkType == 'Y' | linkType == 'Z'){
                loop->translational_motion = amount;
            }else{
                qDebug()<<"There is no such linkType"<<endl;
                break;
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

void Voxelizer::translateVoxelModel(MachineTool &MT, char movingLinkType, float amount, int ind1, int ind2)
{
    if((movingLinkType == 'X') | (movingLinkType == 'Y') | (movingLinkType == 'Z')){

        int voxelNumberDistance = amount * 1000.0f / voxelSize;
        float realMovement = voxelNumberDistance * voxelSize;

        //updateTransformationMatrix
        setTransformationMatrix(MT, movingLinkType, realMovement / 1000.0f);

        //Transform according to each component
        for (QVector<Joint>::iterator loop = MT.JointVector.begin(); loop != MT.JointVector.end(); loop++)
        {
            Link *ChildLink = loop->getChildLink();

            if(ChildLink->getLinkType() == movingLinkType){
                Link *tralatedLink = ChildLink;

                //loop until no child link
                while(tralatedLink != nullptr){

                    //timer
                    QElapsedTimer timer;
                    timer.start();

                    translateVoxels(tralatedLink, movingLinkType, voxelNumberDistance,ind1, ind2);

                    qDebug() << "Translating"<<tralatedLink->getLinkType()<< "took" << timer.elapsed() << "milliseconds"<<endl;
                    //                    qDebug() <<tralatedLink->getLinkType()  << " link contains" << tralatedLink->MTOutterVoxelIndicesList.size() << " shell voxels"<<endl;
                    qDebug() <<tralatedLink->getLinkType()  << " link contains" << tralatedLink->MTInnerVoxelIndicesList.size() << " inner shell voxels"<<endl;

                    //move pointer to childLink
                    tralatedLink = tralatedLink->ChildLink;
                }
            }
        }
    }
}

void Voxelizer::translateVoxels(Link *link, char movingLinkType, int voxelNumberDistance, int ind1, int ind2)
{
    char currentLinkType = link->getLinkType();

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

    // can not translate to outside of voxelspace
    Q_ASSERT_X(((link->get_x_min_index() + voxelNumberDistanceX) > 0) && ((link->get_x_max_index() + voxelNumberDistanceX) < voxelSpaceSize) &&
               ((link->get_y_min_index() + voxelNumberDistanceY) > 0) && ((link->get_y_max_index() + voxelNumberDistanceY) < voxelSpaceSize) &&
               ((link->get_z_min_index() + voxelNumberDistanceZ) > 0) && ((link->get_z_max_index() + voxelNumberDistanceZ) < voxelSpaceSize),
               "translateVoxels", "translate to outside of voxelspace");

    QVector < QVector < QVector< Voxel > > > newVS = link->ParentLink->linkVoxelspace;

    //    for (QList<QVector3D>::iterator i = link->MTOutterVoxelIndicesList.begin(); i != link->MTOutterVoxelIndicesList.end(); ++i){
    //        int number_x = i->x();
    //        int number_y = i->y();
    //        int number_z = i->z();

    //        Voxel& voxel = newVS[number_x + voxelNumberDistanceX][number_y + voxelNumberDistanceY]
    //                [number_z + voxelNumberDistanceZ];
    //        char outterShellType = voxel.getOutterShellLinkType();
    //        char innerShellType = voxel.getInnerShellLinkType();

    //        voxel.setOutterShellLinkType(currentLinkType);

    //        if(outterShellType != 'E' && outterShellType != currentLinkType){
    //            voxel.coincident();
    //        }

    //        if(innerShellType != 'E' && innerShellType != currentLinkType){
    //            voxel.collide();
    //        }

    //        //update MTVoxelIndiciesList of the link
    //        i->setX( number_x + voxelNumberDistanceX);
    //        i->setY( number_y + voxelNumberDistanceY);
    //        i->setZ( number_z + voxelNumberDistanceZ);
    //    }

    //    for (QList<QVector3D>::iterator i = link->MTInnerVoxelIndicesList.begin(); i != link->MTInnerVoxelIndicesList.end(); ++i){

    int index = 0;

    if(link->getLinkType() == 'C')
        index = ind1;

    if(link->getLinkType() == 'A')
        index = ind2;

    for (QList<QVector3D>::iterator i = link->MTOutterVoxelIndicesListVector[index].begin();
         i != link->MTOutterVoxelIndicesListVector[index].end(); ++i){
        int number_x = i->x();
        int number_y = i->y();
        int number_z = i->z();

        Voxel& voxel = newVS[number_x + voxelNumberDistanceX][number_y + voxelNumberDistanceY]
                [number_z + voxelNumberDistanceZ];
        //        char outterShellType = voxel.getOutterShellLinkType();
        char innerShellType = voxel.getInnerShellLinkType();

        voxel.setInnerShellLinkType(currentLinkType);

        //        if(outterShellType != 'E' && outterShellType != currentLinkType){
        //            voxel.collide();
        //        }

        if(innerShellType != 'E' && innerShellType != currentLinkType){
            voxel.collide();
        }

        //update MTVoxelIndiciesList of the link
        i->setX( number_x + voxelNumberDistanceX);
        i->setY( number_y + voxelNumberDistanceY);
        i->setZ( number_z + voxelNumberDistanceZ);
    }


    // update link voxel model bounding box
    link->setBoundingBoxIndex(link->get_x_min_index() + voxelNumberDistanceX, link->get_x_max_index() + voxelNumberDistanceX,
                              link->get_y_min_index() + voxelNumberDistanceY, link->get_y_max_index() + voxelNumberDistanceY,
                              link->get_z_min_index() + voxelNumberDistanceZ, link->get_z_max_index() + voxelNumberDistanceZ);
    //update link voxel space
    link->linkVoxelspace = newVS;
    //    voxelspace = newVS;
}

void Voxelizer::fillInnerShell(char linkType, int index_X, int index_Y, int index_Z, const float* normalArray)
{
    Voxel* voxelBottom = &voxelspace[index_X][index_Y][index_Z - 1];
    Voxel* voxelTop = &voxelspace[index_X][index_Y][index_Z + 1];
    Voxel* voxelFront = &voxelspace[index_X][index_Y - 1][index_Z];
    Voxel* voxelBack = &voxelspace[index_X][index_Y + 1][index_Z];
    Voxel* voxelLeft = &voxelspace[index_X - 1][index_Y][index_Z];
    Voxel* voxelRight = &voxelspace[index_X + 1][index_Y][index_Z];

    //    Voxel* voxelTopFront = &voxelspace[index_X][index_Y - 1][index_Z + 1];
    //    Voxel* voxelTopBack = &voxelspace[index_X][index_Y + 1][index_Z + 1];
    //    Voxel* voxelTopLeft = &voxelspace[index_X - 1][index_Y][index_Z + 1];
    //    Voxel* voxelTopRight = &voxelspace[index_X + 1][index_Y][index_Z + 1];

    //    Voxel* voxelMiddleFrontRight = &voxelspace[index_X][index_Y - 1][index_Z];
    //    Voxel* voxelMiddleFrontLeft = &voxelspace[index_X][index_Y + 1][index_Z];
    //    Voxel* voxelMiddleBackRight = &voxelspace[index_X - 1][index_Y][index_Z];
    //    Voxel* voxelMiddleBackLeft = &voxelspace[index_X + 1][index_Y][index_Z];

    //    Voxel* voxelBottomFront = &voxelspace[index_X][index_Y - 1][index_Z - 1];
    //    Voxel* voxelBottomBack = &voxelspace[index_X][index_Y + 1][index_Z - 1];
    //    Voxel* voxelBottomLeft = &voxelspace[index_X - 1][index_Y][index_Z - 1];
    //    Voxel* voxelBottomRight = &voxelspace[index_X + 1][index_Y][index_Z - 1];


    //face-connected voxels-------------------------------------------------------------------
    //if normal in z is larger than 0.8, bottom voxel is not innerShell of this link and
    // botom voxel is not outterShell of this link
    if(normalArray[2] > 0.75f  &&
            voxelBottom->getOutterShellLinkType() != linkType){

        //set top voxel to outside voxel of this link
        voxelTop->setOutsideVoxelLinkType(linkType);
        voxelTop->setInnerShellLinkType('E');

        // if bottom voxel is not outside voxel of any voxel in this link
        if(voxelBottom->getOutsideVoxelLinkType() != linkType &&
                voxelBottom->getInnerShellLinkType() != linkType){

            if(voxelBottom->getInnerShellLinkType() != 'E')
                voxelBottom->collide();

            //set bottom voxel to the inner shell of this link
            voxelBottom->setInnerShellLinkType(linkType);

            return;
        }

        return;
    }

    if(normalArray[2] < -0.75f &&
            voxelTop->getOutterShellLinkType() != linkType){

        //set bottom voxel to outside voxel of this link
        voxelBottom->setOutsideVoxelLinkType(linkType);
        voxelBottom->setInnerShellLinkType('E');


        if(voxelTop->getOutsideVoxelLinkType() != linkType &&
                voxelTop->getInnerShellLinkType() != linkType){
            if(voxelTop->getInnerShellLinkType() != 'E')
                voxelTop->collide();

            voxelTop->setInnerShellLinkType(linkType);

            return;
        }
        return;
    }


    if(normalArray[1] > 0.75f &&
            voxelFront->getOutterShellLinkType() != linkType){

        //set back voxel to outside voxel of this link
        voxelBack->setOutsideVoxelLinkType(linkType);
        voxelBack->setInnerShellLinkType('E');


        if(voxelFront->getOutsideVoxelLinkType() != linkType  &&
                voxelFront->getInnerShellLinkType() != linkType){
            if(voxelFront->getInnerShellLinkType() != 'E')
                voxelFront->collide();

            voxelFront->setInnerShellLinkType(linkType);
            return;
        }
        return;
    }

    if(normalArray[1] < -0.75f &&
            voxelBack->getOutterShellLinkType() != linkType){

        //set front voxel to outside voxel of this link
        voxelFront->setOutsideVoxelLinkType(linkType);
        voxelFront->setInnerShellLinkType('E');

        if(voxelBack->getOutsideVoxelLinkType() != linkType  &&
                voxelBack->getInnerShellLinkType() != linkType){

            if(voxelBack->getInnerShellLinkType() != 'E')
                voxelBack->collide();

            voxelBack->setInnerShellLinkType(linkType);
            return;
        }
        return;
    }

    if(normalArray[0] > 0.75f &&
            voxelLeft->getOutterShellLinkType() != linkType){

        //set right voxel to outside voxel of this link
        voxelRight->setOutsideVoxelLinkType(linkType);
        voxelRight->setInnerShellLinkType('E');

        if(voxelLeft->getOutsideVoxelLinkType() != linkType  &&
                voxelLeft->getInnerShellLinkType() != linkType){
            if(voxelLeft->getInnerShellLinkType() != 'E')
                voxelLeft->collide();

            voxelLeft->setInnerShellLinkType(linkType);
            return;
        }
        return;
    }

    if(normalArray[0] < -0.75f &&
            voxelRight->getOutterShellLinkType() != linkType){

        //set right voxel to outside voxel of this link
        voxelLeft->setOutsideVoxelLinkType(linkType);
        voxelLeft->setInnerShellLinkType('E');

        if(voxelRight->getOutsideVoxelLinkType() != linkType  &&
                voxelRight->getInnerShellLinkType() != linkType){
            if(voxelRight->getInnerShellLinkType() != 'E')
                voxelRight->collide();

            voxelRight->setInnerShellLinkType(linkType);

            return;
        }
        return;
    }
    //face-connected voxels-------------------------------------------------------------------


    //edge-connected voxels-------------------------------------------------------------------

    //TopRight
    //        if(fabs(normalArray[0] + 0.5774f)  < 0.2f &&
    //                fabs(normalArray[1]) < 0.2f &&
    //                fabs(normalArray[2] + 0.5774f)  < 0.2f &&
    //                voxelTopRight->getOutterShellLinkType() != linkType){

    //            //set right voxel to outside voxel of this link
    //            voxelBottomLeft->setOutsideVoxelLinkType(linkType);
    //            voxelBottomLeft->setInnerShellLinkType('E');

    //            if(voxelTopRight->getOutsideVoxelLinkType() != linkType  &&
    //                    voxelTopRight->getInnerShellLinkType() != linkType){
    //                if(voxelTopRight->getInnerShellLinkType() != 'E')
    //                    voxelTopRight->collide();

    //                voxelTopRight->setInnerShellLinkType(linkType);

    //                return;
    //            }
    //            return;
    //        }

    //TopLeft
    //        if(fabs(normalArray[0] - 0.5774f)  < 0.2f &&
    //                fabs(normalArray[1]) < 0.2f &&
    //                fabs(normalArray[2] + 0.5774f)  < 0.2f &&
    //                voxelTopLeft->getOutterShellLinkType() != linkType){

    //            //set right voxel to outside voxel of this link
    //            voxelBottomRight->setOutsideVoxelLinkType(linkType);
    //            voxelBottomRight->setInnerShellLinkType('E');

    //            if(voxelTopLeft->getOutsideVoxelLinkType() != linkType  &&
    //                    voxelTopLeft->getInnerShellLinkType() != linkType){
    //                if(voxelTopLeft->getInnerShellLinkType() != 'E')
    //                    voxelTopLeft->collide();

    //                voxelTopLeft->setInnerShellLinkType(linkType);

    //                return;
    //            }
    //            return;
    //        }

    //    //TopBack
    //        if(fabs(normalArray[0]) < 0.2f &&
    //                fabs(normalArray[1] + 0.5774f)  < 0.2f &&
    //                fabs(normalArray[2] + 0.5774f)  < 0.2f &&
    //                voxelTopBack->getOutterShellLinkType() != linkType){

    //            //set right voxel to outside voxel of this link
    //            voxelBottomFront->setOutsideVoxelLinkType(linkType);
    //            voxelBottomFront->setInnerShellLinkType('E');

    //            if(voxelTopBack->getOutsideVoxelLinkType() != linkType  &&
    //                    voxelTopBack->getInnerShellLinkType() != linkType){
    //                if(voxelTopBack->getInnerShellLinkType() != 'E')
    //                    voxelTopBack->collide();

    //                voxelTopBack->setInnerShellLinkType(linkType);

    //                return;
    //            }
    //            return;
    //        }

    //    //TopFront
    //    if(fabs(normalArray[0]) < 0.2f &&
    //            fabs(normalArray[1] - 0.5774f)  < 0.2f &&
    //            fabs(normalArray[2] + 0.5774f)  < 0.2f &&
    //            voxelTopFront->getOutterShellLinkType() != linkType){

    //        //set right voxel to outside voxel of this link
    //        voxelBottomBack->setOutsideVoxelLinkType(linkType);
    //        voxelBottomBack->setInnerShellLinkType('E');

    //        if(voxelTopFront->getOutsideVoxelLinkType() != linkType  &&
    //                voxelTopFront->getInnerShellLinkType() != linkType){
    //            if(voxelTopFront->getInnerShellLinkType() != 'E')
    //                voxelTopFront->collide();

    //            voxelTopFront->setInnerShellLinkType(linkType);

    //            return;
    //        }
    //        return;
    //    }

    //    //middlefrontleft
    //    if(fabs(normalArray[0] - 0.5774f) < 0.2f &&
    //            fabs(normalArray[1] - 0.5774f) < 0.2f &&
    //            fabs(normalArray[2]) < 0.2f &&
    //            voxelMiddleFrontLeft->getOutterShellLinkType() != linkType){

    //        //set right voxel to outside voxel of this link
    //        voxelMiddleBackRight->setOutsideVoxelLinkType(linkType);
    //        voxelMiddleBackRight->setInnerShellLinkType('E');

    //        if(voxelMiddleFrontLeft->getOutsideVoxelLinkType() != linkType  &&
    //                voxelMiddleFrontLeft->getInnerShellLinkType() != linkType){
    //            if(voxelMiddleFrontLeft->getInnerShellLinkType() != 'E')
    //                voxelMiddleFrontLeft->collide();

    //            voxelMiddleFrontLeft->setInnerShellLinkType(linkType);

    //            return;
    //        }
    //        return;
    //    }

    //    //middlefrontright
    //    if(fabs(normalArray[0] + 0.5774f) < 0.2f &&
    //            fabs(normalArray[1] - 0.5774f) < 0.2f &&
    //            fabs(normalArray[2]) < 0.2f &&
    //            voxelMiddleFrontRight->getOutterShellLinkType() != linkType){

    //        //set right voxel to outside voxel of this link
    //        voxelMiddleBackLeft->setOutsideVoxelLinkType(linkType);
    //        voxelMiddleBackLeft->setInnerShellLinkType('E');

    //        if(voxelMiddleFrontRight->getOutsideVoxelLinkType() != linkType  &&
    //                voxelMiddleFrontRight->getInnerShellLinkType() != linkType){
    //            if(voxelMiddleFrontRight->getInnerShellLinkType() != 'E')
    //                voxelMiddleFrontRight->collide();

    //            voxelMiddleFrontRight->setInnerShellLinkType(linkType);

    //            return;
    //        }
    //        return;
    //    }

    //    //middlebackleft
    //    if(fabs(normalArray[0] - 0.5774f) < 0.2f &&
    //            fabs(normalArray[1] + 0.5774f) < 0.2f &&
    //            fabs(normalArray[2]) < 0.2f &&
    //            voxelMiddleBackLeft->getOutterShellLinkType() != linkType){

    //        //set right voxel to outside voxel of this link
    //        voxelMiddleFrontRight->setOutsideVoxelLinkType(linkType);
    //        voxelMiddleFrontRight->setInnerShellLinkType('E');

    //        if(voxelMiddleBackLeft->getOutsideVoxelLinkType() != linkType  &&
    //                voxelMiddleBackLeft->getInnerShellLinkType() != linkType){
    //            if(voxelMiddleBackLeft->getInnerShellLinkType() != 'E')
    //                voxelMiddleBackLeft->collide();

    //            voxelMiddleBackLeft->setInnerShellLinkType(linkType);

    //            return;
    //        }
    //        return;
    //    }

    //    //middlebackright
    //    if(fabs(normalArray[0] + 0.5774f) < 0.2f &&
    //            fabs(normalArray[1] + 0.5774f) < 0.2f &&
    //            fabs(normalArray[2]) < 0.2f &&
    //            voxelMiddleBackRight->getOutterShellLinkType() != linkType){

    //        //set right voxel to outside voxel of this link
    //        voxelMiddleFrontLeft->setOutsideVoxelLinkType(linkType);
    //        voxelMiddleFrontLeft->setInnerShellLinkType('E');

    //        if(voxelMiddleBackRight->getOutsideVoxelLinkType() != linkType  &&
    //                voxelMiddleBackRight->getInnerShellLinkType() != linkType){
    //            if(voxelMiddleBackRight->getInnerShellLinkType() != 'E')
    //                voxelMiddleBackRight->collide();

    //            voxelMiddleBackRight->setInnerShellLinkType(linkType);

    //            return;
    //        }
    //        return;
    //    }

    //    //BottomRight
    //    if(fabs(normalArray[0] - 0.5774f)  < 0.15f &&
    //            fabs(normalArray[1]) < 0.15f &&
    //            fabs(normalArray[2] - 0.5774f)  < 0.15f &&
    //            voxelBottomRight->getOutterShellLinkType() != linkType){

    //        //set right voxel to outside voxel of this link
    //        voxelTopLeft->setOutsideVoxelLinkType(linkType);
    //        voxelTopLeft->setInnerShellLinkType('E');

    //        if(voxelBottomRight->getOutsideVoxelLinkType() != linkType  &&
    //                voxelBottomRight->getInnerShellLinkType() != linkType){
    //            if(voxelBottomRight->getInnerShellLinkType() != 'E')
    //                voxelBottomRight->collide();

    //            voxelBottomRight->setInnerShellLinkType(linkType);

    //            return;
    //        }
    //        return;
    //    }

    //    //BottomLeft
    //    if(fabs(normalArray[0] + 0.5774f)  < 0.15f &&
    //            fabs(normalArray[1]) < 0.15f &&
    //            fabs(normalArray[2] - 0.5774f)  < 0.15f &&
    //            voxelBottomLeft->getOutterShellLinkType() != linkType){

    //        //set right voxel to outside voxel of this link
    //        voxelTopRight->setOutsideVoxelLinkType(linkType);
    //        voxelTopRight->setInnerShellLinkType('E');

    //        if(voxelBottomLeft->getOutsideVoxelLinkType() != linkType  &&
    //                voxelBottomLeft->getInnerShellLinkType() != linkType){
    //            if(voxelBottomLeft->getInnerShellLinkType() != 'E')
    //                voxelBottomLeft->collide();

    //            voxelBottomLeft->setInnerShellLinkType(linkType);

    //            return;
    //        }
    //        return;
    //    }

    //    //BottomBack
    //        if(fabs(normalArray[0]) < 0.2f &&
    //                fabs(normalArray[1] + 0.5774f)  < 0.2f &&
    //                fabs(normalArray[2] - 0.5774f)  < 0.2f &&
    //                voxelBottomBack->getOutterShellLinkType() != linkType){

    //            //set right voxel to outside voxel of this link
    //            voxelTopFront->setOutsideVoxelLinkType(linkType);
    //            voxelTopFront->setInnerShellLinkType('E');

    //            if(voxelBottomBack->getOutsideVoxelLinkType() != linkType  &&
    //                    voxelBottomBack->getInnerShellLinkType() != linkType){
    //                if(voxelBottomBack->getInnerShellLinkType() != 'E')
    //                    voxelBottomBack->collide();

    //                voxelBottomBack->setInnerShellLinkType(linkType);

    //                return;
    //            }
    //            return;
    //        }

    //    //BottomFront
    //        if(fabs(normalArray[0]) < 0.2f &&
    //                fabs(normalArray[1] - 0.5774f)  < 0.2f &&
    //                fabs(normalArray[2] - 0.5774f)  < 0.2f &&
    //                voxelBottomFront->getOutterShellLinkType() != linkType){

    //            //set right voxel to outside voxel of this link
    //            voxelTopBack->setOutsideVoxelLinkType(linkType);
    //            voxelTopBack->setInnerShellLinkType('E');

    //            if(voxelBottomFront->getOutsideVoxelLinkType() != linkType  &&
    //                    voxelBottomFront->getInnerShellLinkType() != linkType){
    //                if(voxelBottomFront->getInnerShellLinkType() != 'E')
    //                    voxelBottomFront->collide();

    //                voxelBottomFront->setInnerShellLinkType(linkType);

    //                return;
    //            }
    //            return;
    //        }

    //edge-connected voxels-------------------------------------------------------------------
}
