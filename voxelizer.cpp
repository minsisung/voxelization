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
    if(link.ParentLink != nullptr)
        voxelspace = link.ParentLink->linkVoxelspace;

    parentModelVoxelization(link, needVisualization);
}

void Voxelizer::parentModelVoxelization(Link& link, bool needVisualization)
{
    //create empty voxelIndicesList to store parent voxel model of this link
    QList<QVector3D> newParentMTVoxelIndicesList;

    float min_x, max_x, min_y, max_y, min_z, max_z;
    QChar linkType = link.getLinkType();
    float voxelStartingCenter = (-spaceLength/2) + (voxelSize/2);
    QVector<stl_reader::StlMesh <float, unsigned int>> meshVector = link.getSTLMesh();
    QMatrix4x4 TransformMatrix = link.m_TransformMatrix;

    QElapsedTimer timer;
    timer.start();

    //go through all components of this link
    for(int mesh_ind = 0; mesh_ind < meshVector.size(); ++mesh_ind) {
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
            for (int ind_x = index_x_min; ind_x<index_x_max + 1; ind_x++){
                for (int ind_y = index_y_min; ind_y<index_y_max + 1; ind_y++){
                    for (int ind_z = index_z_min; ind_z<index_z_max + 1; ind_z++){
                        Voxel& voxel = voxelspace[ind_x][ind_y][ind_z];
                        QChar voxelLinkType = voxel.getVoxelLinkType();

                        boxcenter.x = voxelStartingCenter + voxelSize*ind_x;
                        boxcenter.y = voxelStartingCenter + voxelSize*ind_y;
                        boxcenter.z = voxelStartingCenter + voxelSize*ind_z;
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

    //set bounding box for link
    link.setBoundingBoxIndex(bounding_x_min_index, bounding_x_max_index, bounding_y_min_index,bounding_y_max_index,
                             bounding_z_min_index, bounding_z_max_index);

    //update voxel space assigned to voxel space
    link.linkVoxelspace = voxelspace;
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
void Voxelizer::setTransformationMatrix(MachineTool &MT, QChar linkType, float amount)
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

QSet<QString> Voxelizer::translateVoxelModel(MachineTool &MT, QChar movingLinkType, float amount, int ind1, int ind2)
{
    QSet<QString> totalCollisionSet;

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

                    totalCollisionSet.unite(translateVoxels(tralatedLink, movingLinkType, voxelNumberDistance,ind1, ind2));

                    qDebug() << "Translating"<<tralatedLink->getLinkType()<< "took" << timer.elapsed() << "milliseconds"<<endl;
                    //                    qDebug() <<tralatedLink->getLinkType()  << " link contains" << tralatedLink->MTOutterVoxelIndicesList.size() << " shell voxels"<<endl;
                    //                    qDebug() <<tralatedLink->getLinkType()  << " link contains" << tralatedLink->MTInnerVoxelIndicesList.size() << " inner shell voxels"<<endl;

                    //move pointer to childLink
                    tralatedLink = tralatedLink->ChildLink;
                }
            }
        }
    }
    return totalCollisionSet;
}

QSet<QString> Voxelizer::translateVoxels(Link *link, QChar movingLinkType, int voxelNumberDistance, int ind1, int ind2)
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

    // can not translate to outside of voxelspace
    Q_ASSERT_X(((link->get_x_min_index() + voxelNumberDistanceX) > 0) && ((link->get_x_max_index() + voxelNumberDistanceX) < voxelSpaceSize) &&
               ((link->get_y_min_index() + voxelNumberDistanceY) > 0) && ((link->get_y_max_index() + voxelNumberDistanceY) < voxelSpaceSize) &&
               ((link->get_z_min_index() + voxelNumberDistanceZ) > 0) && ((link->get_z_max_index() + voxelNumberDistanceZ) < voxelSpaceSize),
               "translateVoxels", "translate to outside of voxelspace");

    QVector < QVector < QVector< Voxel > > > newVS = link->ParentLink->linkVoxelspace;

    //traslate indicesListVector---------------------------------------------------------------------
    int index = 0;

    if(link->getLinkType() == 'C')
        index = ind1;

    if(link->getLinkType() == 'A')
        index = ind2;


    for(int mesh_ind = 0; mesh_ind < link->MTVoxelIndicesListVector.size(); ++mesh_ind){
        for (QList<QVector3D>::iterator i = link->MTVoxelIndicesListVector[mesh_ind][index].begin();
             i != link->MTVoxelIndicesListVector[mesh_ind][index].end(); ++i){
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

                collisionPair.append(currentLinkType);
                collisionPair.append(QString::number(voxel.getComponentNumber()));
                collisionPair.append(ShellType);
                collisionPair.append(componentNumber);

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

