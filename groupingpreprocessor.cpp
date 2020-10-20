#include "groupingpreprocessor.h"

GroupingPreProcessor::GroupingPreProcessor():
    m_totalCount(0)
{
}

QString GroupingPreProcessor::createMTVoxelspace(float vSize, QVector<component> &componentVector)
{
    //setup voxelsize
    voxelSize = vSize;

    //initialize voxel space and voxel size and return the name of the lowest component
    QString lowestCompName = voxelizer.setupSize(voxelSize, componentVector);

    return lowestCompName;
}

QVector<contactComponentsPair> GroupingPreProcessor::findContactComponentsPairs(QVector<component> &componentVector)
{
    QVector<contactComponentsPair> ccpVector = voxelizer.collisionDetectionForComponents(componentVector);

    //Initialize CCPs for intial grouping usage
    for(int ind_ccp = 0; ind_ccp < ccpVector.size(); ind_ccp++){
        //build CCPs
        QVector<QString> ccp;
        ccp.append(ccpVector[ind_ccp].getFirstMeshName());
        ccp.append(ccpVector[ind_ccp].getSecondMeshName());
        CCPs.append(ccp);
    }

    qDebug()<<"---------------------The number of components:"<<componentVector.size()<<"----------------------"<<endl;
    qDebug()<<"---------------------The number of CCPs:"<<ccpVector.size()<<"----------------------"<<endl;
    updateCCPVector(ccpVector);

    //print all motion relationship between components of CCPs
    for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
        qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
        qDebug()<<"Collision occurs when move in positive X:"<<ccpVector[ccp_ind].isCollided_Positive_X();
        qDebug()<<"Collision occurs when move in negative X:"<<ccpVector[ccp_ind].isCollided_Negative_X();
        qDebug()<<"Collision occurs when move in positive Y:"<<ccpVector[ccp_ind].isCollided_Positive_Y();
        qDebug()<<"Collision occurs when move in negative Y:"<<ccpVector[ccp_ind].isCollided_Negative_Y();
        qDebug()<<"Collision occurs when move in positive Z:"<<ccpVector[ccp_ind].isCollided_Positive_Z();
        qDebug()<<"Collision occurs when move in negative Z:"<<ccpVector[ccp_ind].isCollided_Negative_Z();

        if(ccpVector[ccp_ind].containsCommonRotaryAxis1()){
            qDebug()<<"Collision occurs when rotate along"<<ccpVector[ccp_ind].firstAxis<<":"
                   <<ccpVector[ccp_ind].isCollided_FirstAxis();
        }else{
            qDebug()<<"This CCP doesn't have common rotational axis in"<<ccpVector[ccp_ind].firstAxis;
        }

        if(ccpVector[ccp_ind].containsCommonRotaryAxis2()){
            qDebug()<<"Collision occurs when rotate along"<<ccpVector[ccp_ind].secondAxis<<":"
                   <<ccpVector[ccp_ind].isCollided_SecondAxis();
        }else{
            qDebug()<<"This CCP doesn't have common rotational axis in"<<ccpVector[ccp_ind].secondAxis;
        }
        qDebug()<<endl;
    }

    return ccpVector;
}

int GroupingPreProcessor::findLIPCandidates(QVector<contactComponentsPair> &ccpVector)
{
    for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){

        //Rotational LIP candidate
        //first common axis
        if(ccpVector[ccp_ind].containsCommonRotaryAxis1() && !ccpVector[ccp_ind].isCollided_FirstAxis()){
            //both components contain offset mesh
            if(ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() && ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()){
                //A axis
                if(ccpVector[ccp_ind].firstAxis == "A"){
                    if((ccpVector[ccp_ind].isCollided_Positive_Y() | ccpVector[ccp_ind].isCollided_Negative_Y()) &&
                            (ccpVector[ccp_ind].isCollided_Positive_Z() | ccpVector[ccp_ind].isCollided_Negative_Z())){
                        qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                        qDebug()<<"is a candidate of A-LIP"<<endl;

                        //fill LIPs
                        QVector<QString> lip;
                        QString axis = "A";
                        lip.append(ccpVector[ccp_ind].getFirstMeshName());
                        lip.append(ccpVector[ccp_ind].getSecondMeshName());
                        LIPs.append(qMakePair(axis, lip));
                        continue;
                    }
                }
                //B axis
                if(ccpVector[ccp_ind].firstAxis == "B"){
                    if((ccpVector[ccp_ind].isCollided_Positive_X() | ccpVector[ccp_ind].isCollided_Negative_X()) &&
                            (ccpVector[ccp_ind].isCollided_Positive_Z() | ccpVector[ccp_ind].isCollided_Negative_Z())){
                        qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                        qDebug()<<"is a candidate of B-LIP"<<endl;

                        //fill LIPs
                        QVector<QString> lip;
                        QString axis = "B";
                        lip.append(ccpVector[ccp_ind].getFirstMeshName());
                        lip.append(ccpVector[ccp_ind].getSecondMeshName());
                        LIPs.append(qMakePair(axis, lip));
                        continue;
                    }
                }
                //C axis
                if(ccpVector[ccp_ind].firstAxis == "C"){
                    if((ccpVector[ccp_ind].isCollided_Positive_X() | ccpVector[ccp_ind].isCollided_Negative_X()) &&
                            (ccpVector[ccp_ind].isCollided_Positive_Y() | ccpVector[ccp_ind].isCollided_Negative_Y())){
                        qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                        qDebug()<<"is a candidate of C-LIP"<<endl;

                        //fill LIPs
                        QVector<QString> lip;
                        QString axis = "C";
                        lip.append(ccpVector[ccp_ind].getFirstMeshName());
                        lip.append(ccpVector[ccp_ind].getSecondMeshName());
                        LIPs.append(qMakePair(axis, lip));
                        continue;
                    }
                }
            }
        }

        //second common axis
        if(ccpVector[ccp_ind].containsCommonRotaryAxis2() && !ccpVector[ccp_ind].isCollided_SecondAxis()){
            //both components contain offset mesh
            if(ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() && ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()){
                //A axis
                if(ccpVector[ccp_ind].secondAxis == "A"){
                    if((ccpVector[ccp_ind].isCollided_Positive_Y() | ccpVector[ccp_ind].isCollided_Negative_Y()) &&
                            (ccpVector[ccp_ind].isCollided_Positive_Z() | ccpVector[ccp_ind].isCollided_Negative_Z())){
                        qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                        qDebug()<<"is a candidate of A-LIP"<<endl;

                        //fill LIPs
                        QVector<QString> lip;
                        QString axis = "A";
                        lip.append(ccpVector[ccp_ind].getFirstMeshName());
                        lip.append(ccpVector[ccp_ind].getSecondMeshName());
                        LIPs.append(qMakePair(axis, lip));
                        continue;
                    }
                }
                //B axis
                if(ccpVector[ccp_ind].secondAxis == "B"){
                    if((ccpVector[ccp_ind].isCollided_Positive_X() | ccpVector[ccp_ind].isCollided_Negative_X()) &&
                            (ccpVector[ccp_ind].isCollided_Positive_Z() | ccpVector[ccp_ind].isCollided_Negative_Z())){
                        qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                        qDebug()<<"is a candidate of B-LIP"<<endl;

                        //fill LIPs
                        QVector<QString> lip;
                        QString axis = "B";
                        lip.append(ccpVector[ccp_ind].getFirstMeshName());
                        lip.append(ccpVector[ccp_ind].getSecondMeshName());
                        LIPs.append(qMakePair(axis, lip));
                        continue;
                    }
                }
                //C axis
                if(ccpVector[ccp_ind].secondAxis == "C"){
                    if((ccpVector[ccp_ind].isCollided_Positive_X() | ccpVector[ccp_ind].isCollided_Negative_X()) &&
                            (ccpVector[ccp_ind].isCollided_Positive_Y() | ccpVector[ccp_ind].isCollided_Negative_Y())){
                        qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                        qDebug()<<"is a candidate of C-LIP"<<endl;

                        //fill LIPs
                        QVector<QString> lip;
                        QString axis = "C";
                        lip.append(ccpVector[ccp_ind].getFirstMeshName());
                        lip.append(ccpVector[ccp_ind].getSecondMeshName());
                        LIPs.append(qMakePair(axis, lip));
                        continue;
                    }
                }
            }
        }

        //X-LIP candidate
        if(!ccpVector[ccp_ind].isCollided_Positive_X() && !ccpVector[ccp_ind].isCollided_Negative_X()){
            if(((ccpVector[ccp_ind].isCollided_Positive_Y() | ccpVector[ccp_ind].isCollided_Negative_Y()) &&
                (ccpVector[ccp_ind].isCollided_Positive_Z() && ccpVector[ccp_ind].isCollided_Negative_Z())) |
                    ((ccpVector[ccp_ind].isCollided_Positive_Y() && ccpVector[ccp_ind].isCollided_Negative_Y()) &&
                     (ccpVector[ccp_ind].isCollided_Positive_Z() | ccpVector[ccp_ind].isCollided_Negative_Z()))){
                //both components contain offset mesh
                if(ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() && ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()){
                    qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                    qDebug()<<"is a candidate of X-LIP"<<endl;

                    //fill LIPs
                    QVector<QString> lip;
                    QString axis = "X";
                    lip.append(ccpVector[ccp_ind].getFirstMeshName());
                    lip.append(ccpVector[ccp_ind].getSecondMeshName());
                    LIPs.append(qMakePair(axis, lip));
                    continue;
                }
            }
        }

        //Y-LIP candidate
        if(!ccpVector[ccp_ind].isCollided_Positive_Y() && !ccpVector[ccp_ind].isCollided_Negative_Y()){
            if(((ccpVector[ccp_ind].isCollided_Positive_X() | ccpVector[ccp_ind].isCollided_Negative_X()) &&
                (ccpVector[ccp_ind].isCollided_Positive_Z() && ccpVector[ccp_ind].isCollided_Negative_Z())) |
                    ((ccpVector[ccp_ind].isCollided_Positive_X() && ccpVector[ccp_ind].isCollided_Negative_X()) &&
                     (ccpVector[ccp_ind].isCollided_Positive_Z() | ccpVector[ccp_ind].isCollided_Negative_Z()))){
                //both components contain offset mesh
                if(ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() && ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()){
                    qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                    qDebug()<<"is a candidate of Y-LIP"<<endl;

                    //fill LIPs
                    QVector<QString> lip;
                    QString axis = "Y";
                    lip.append(ccpVector[ccp_ind].getFirstMeshName());
                    lip.append(ccpVector[ccp_ind].getSecondMeshName());
                    LIPs.append(qMakePair(axis, lip));
                    continue;
                }
            }
        }

        //Z-LIP candidate
        if(!ccpVector[ccp_ind].isCollided_Positive_Z() && !ccpVector[ccp_ind].isCollided_Negative_Z()){
            if(((ccpVector[ccp_ind].isCollided_Positive_Y() | ccpVector[ccp_ind].isCollided_Negative_Y()) &&
                (ccpVector[ccp_ind].isCollided_Positive_X() && ccpVector[ccp_ind].isCollided_Negative_X())) |
                    ((ccpVector[ccp_ind].isCollided_Positive_Y() && ccpVector[ccp_ind].isCollided_Negative_Y()) &&
                     (ccpVector[ccp_ind].isCollided_Positive_X() | ccpVector[ccp_ind].isCollided_Negative_X()))){
                //both components contain offset mesh
                if(ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() && ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()){
                    qDebug()<<"CCP name:"<<ccpVector[ccp_ind].getName();
                    qDebug()<<"is a candidate of Z-LIP"<<endl;

                    //fill LIPs
                    QVector<QString> lip;
                    QString axis = "Z";
                    lip.append(ccpVector[ccp_ind].getFirstMeshName());
                    lip.append(ccpVector[ccp_ind].getSecondMeshName());
                    LIPs.append(qMakePair(axis, lip));
                    continue;
                }
            }
        }
    }

    //draw selected CCP with relative movement
    drawVoxelforCCP(voxelizer.compVoxelIndicesList1, voxelizer.compVoxelIndicesList2);

    return LIPs.size();
}

void GroupingPreProcessor::drawVoxelforCCP(QList<QVector3D> &compVoxelIndicesList1, QList<QVector3D> &compVoxelIndicesList2)
{
    float voxelSpace_X_min = voxelizer.voxelStarting_X;
    float voxelSpace_Y_min = voxelizer.voxelStarting_Y;
    float voxelSpace_Z_min = voxelizer.voxelStarting_Z;

    for(int comp_ind = 0; comp_ind < 2; ++comp_ind){

        QList<QVector3D> compVoxelIndciesList;
        int *numberOfVertex;

        //first component (only first component being transform)
        if(comp_ind == 0){
            compVoxelIndciesList = compVoxelIndicesList1;
            numberOfVertex = &numberOfVertex_comp1;
        }else{
            compVoxelIndciesList = compVoxelIndicesList2;
            numberOfVertex = &numberOfVertex_comp2;
        }

        for (QList<QVector3D>::iterator i = compVoxelIndciesList.begin();
             i != compVoxelIndciesList.end(); ++i){

            int number_x = i->x();
            int number_y = i->y();
            int number_z = i->z();

            GLfloat offset_y = voxelSize * number_y;
            GLfloat offset_x = voxelSize * number_x;
            GLfloat offset_z = voxelSize * number_z;
            GLfloat x_right = voxelSpace_X_min + voxelSize + offset_x;
            GLfloat x_left = voxelSpace_X_min + offset_x;
            GLfloat y_up = voxelSpace_Y_min + voxelSize + offset_y;
            GLfloat y_down = voxelSpace_Y_min + offset_y;
            GLfloat z_futher = voxelSpace_Z_min + offset_z;
            GLfloat z_closer = voxelSpace_Z_min + voxelSize + offset_z;

            //coordinates of triangles forming faces of cubes
            //6 faces for each cube, 4 vertices for each face, 3 coordinates for each vertex
            GLfloat coords[6][4][3] = {
                { { x_right, y_down, z_futher}, { x_left, y_down, z_futher}, { x_left, y_up, z_futher}, { x_right, y_up, z_futher} },
                { { x_right, y_up, z_futher}, { x_left, y_up, z_futher}, { x_left, y_up, z_closer}, { x_right, y_up, z_closer} },
                { { x_right, y_down, z_closer}, { x_right, y_down, z_futher}, { x_right, y_up, z_futher}, { x_right, y_up, z_closer} },
                { { x_left,y_down, z_futher}, { x_left, y_down, z_closer}, { x_left, y_up, z_closer}, { x_left, y_up, z_futher} },
                { { x_right,y_down, z_closer}, { x_left, y_down, z_closer}, { x_left, y_down, z_futher}, { x_right, y_down, z_futher} },
                { { x_left, y_down, z_closer}, { x_right, y_down, z_closer}, { x_right, y_up, z_closer}, { x_left, y_up, z_closer} }
            };

            QVector3D normal;
            bool ifDuplicate = false;

            // Draw6 different normal direction and avoid overlapping face for each face
            for (int i = 0; i < 6; ++i) {
                normal = setNormal(i);

                //                ifDuplicate = checkDuplicateFace(i, number_x, number_y, number_z);
                //                if(ifDuplicate)
                //                    continue;

                // insert vertex position into m_data for creating VBO

                //first triangle in the face
                m_data.resize(m_totalCount+36);
                GLfloat *p = m_data.data() + m_totalCount;
                *p++ = coords[i][0][0];
                *p++ = coords[i][0][1];
                *p++ = coords[i][0][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];
                //---------------------------------
                *p++ = coords[i][1][0];
                *p++ = coords[i][1][1];
                *p++ = coords[i][1][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];
                //---------------------------------
                *p++ = coords[i][2][0];
                *p++ = coords[i][2][1];
                *p++ = coords[i][2][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];

                /////////////////////////////////////////////
                //second triangle in the face
                *p++ = coords[i][0][0];
                *p++ = coords[i][0][1];
                *p++ = coords[i][0][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];
                //---------------------------------
                *p++ = coords[i][2][0];
                *p++ = coords[i][2][1];
                *p++ = coords[i][2][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];
                //---------------------------------
                *p++ = coords[i][3][0];
                *p++ = coords[i][3][1];
                *p++ = coords[i][3][2];
                *p++ = normal[0];
                *p++ = normal[1];
                *p++ = normal[2];
                m_totalCount += 36;

                //update number of vertices
                *numberOfVertex += 6;
            }
        }
    }
}

void GroupingPreProcessor::updateCCPVector(QVector<contactComponentsPair> &ccpVector)
{
    float CD_distance_offoff = 0.016f;
    float CD_distance_offnon = 0.007f;
    drawingCCPName = "UMC-1600H - X Axis Table UMC-1600 5-AX-1==UMC-1600H - B Axis UMC-1600H-1";

    //check x positive
    QMatrix4x4 transformMatrix_X_Positive;

    for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
        bool isCollided = false;
        transformMatrix_X_Positive.setToIdentity();
        qDebug()<<"colliion detection in X positive for"<<ccpVector[ccp_ind].getName();

        //offset offset CD
        if(ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()){
            transformMatrix_X_Positive.translate(CD_distance_offoff,0.0f,0.0f);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_X_Positive);

            //non-offset offset CD
        }else if((ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                  !ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()) |
                 (!ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                  ccpVector[ccp_ind].getSecondComp().containsOffsetMesh())){
            transformMatrix_X_Positive.translate(CD_distance_offnon,0.0f,0.0f);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_X_Positive);

            //non-offset non-offset CD
        }else{
            transformMatrix_X_Positive.translate(CD_distance_offnon,0.0f,0.0f);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_X_Positive);
        }

        if(isCollided)
            ccpVector[ccp_ind].collided_Positive_X();
    }

    //check x negative
    QMatrix4x4 transformMatrix_X_Negative;

    for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
        bool isCollided = false;
        transformMatrix_X_Negative.setToIdentity();
        qDebug()<<"colliion detection in X negative for"<<ccpVector[ccp_ind].getName();

        //offset offset CD
        if(ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()){
            transformMatrix_X_Negative.translate(-CD_distance_offoff,0.0f,0.0f);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_X_Negative);

            //non-offset offset CD
        }else if((ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                  !ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()) |
                 (!ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                  ccpVector[ccp_ind].getSecondComp().containsOffsetMesh())){
            transformMatrix_X_Negative.translate(-CD_distance_offnon,0.0f,0.0f);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_X_Negative);

            //non-offset non-offset CD
        }else{
            transformMatrix_X_Negative.translate(-CD_distance_offnon,0.0f,0.0f);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_X_Negative);
        }
        if(isCollided)
            ccpVector[ccp_ind].collided_Negative_X();
    }

    //check y positive
    QMatrix4x4 transformMatrix_Y_Positive;

    for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
        bool isCollided = false;
        transformMatrix_Y_Positive.setToIdentity();
        qDebug()<<"colliion detection in Y positive for"<<ccpVector[ccp_ind].getName();

        //offset offset CD
        if(ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()){
            transformMatrix_Y_Positive.translate(0.0f,CD_distance_offoff,0.0f);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_Y_Positive);

            //non-offset offset CD
        }else if((ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                  !ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()) |
                 (!ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                  ccpVector[ccp_ind].getSecondComp().containsOffsetMesh())){
            transformMatrix_Y_Positive.translate(0.0f,CD_distance_offnon,0.0f);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_Y_Positive);

            //non-offset non-offset CD
        }else{
            transformMatrix_Y_Positive.translate(0.0f,CD_distance_offnon,0.0f);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_Y_Positive);
        }
        if(isCollided)
            ccpVector[ccp_ind].collided_Positive_Y();
    }

    //check y negative
    QMatrix4x4 transformMatrix_Y_Negative;

    for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
        bool isCollided = false;
        transformMatrix_Y_Negative.setToIdentity();
        qDebug()<<"colliion detection in Y negative for"<<ccpVector[ccp_ind].getName();

        //offset offset CD
        if(ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()){
            transformMatrix_Y_Negative.translate(0.0f,-CD_distance_offoff,0.0f);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_Y_Negative);

            //non-offset offset CD
        }else if((ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                  !ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()) |
                 (!ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                  ccpVector[ccp_ind].getSecondComp().containsOffsetMesh())){
            transformMatrix_Y_Negative.translate(0.0f,-CD_distance_offnon,0.0f);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_Y_Negative);

            //non-offset non-offset CD
        }else{
            transformMatrix_Y_Negative.translate(0.0f,-CD_distance_offnon,0.0f);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_Y_Negative);
            qDebug()<<"non-offset non-offset CD"<<endl;
        }
        if(isCollided)
            ccpVector[ccp_ind].collided_Negative_Y();
    }

    //check z positive
    QMatrix4x4 transformMatrix_Z_Positive;

    for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
        bool isCollided = false;
        transformMatrix_Z_Positive.setToIdentity();
        qDebug()<<"colliion detection in Z positive for"<<ccpVector[ccp_ind].getName();

        //offset offset CD
        if(ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()){
            transformMatrix_Z_Positive.translate(0.0f,0.0f,CD_distance_offoff);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_Z_Positive);

            //non-offset offset CD
        }else if((ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                  !ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()) |
                 (!ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                  ccpVector[ccp_ind].getSecondComp().containsOffsetMesh())){
            transformMatrix_Z_Positive.translate(0.0f,0.0f,CD_distance_offnon);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_Z_Positive);

            //non-offset non-offset CD
        }else{
            transformMatrix_Z_Positive.translate(0.0f,0.0f,CD_distance_offnon);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_Z_Positive);
        }
        if(isCollided)
            ccpVector[ccp_ind].collided_Positive_Z();
    }

    //check z negative
    QMatrix4x4 transformMatrix_Z_Negative;

    for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
        bool isCollided = false;
        transformMatrix_Z_Negative.setToIdentity();
        qDebug()<<"colliion detection in Z negative for"<<ccpVector[ccp_ind].getName();

        //offset offset CD
        if(ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()){
            transformMatrix_Z_Negative.translate(0.0f,0.0f,-CD_distance_offoff);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_Z_Negative);

            //non-offset offset CD
        }else if((ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                  !ccpVector[ccp_ind].getSecondComp().containsOffsetMesh()) |
                 (!ccpVector[ccp_ind].getFirstComp().containsOffsetMesh() &&
                  ccpVector[ccp_ind].getSecondComp().containsOffsetMesh())){
            transformMatrix_Z_Negative.translate(0.0f,0.0f,-CD_distance_offnon);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_Z_Negative);

            //non-offset non-offset CD
        }else{
            transformMatrix_Z_Negative.translate(0.0f,0.0f,-CD_distance_offnon);
            isCollided = voxelizer.translationalCDForCCP(ccpVector[ccp_ind], transformMatrix_Z_Negative);
        }
        if(isCollided)
            ccpVector[ccp_ind].collided_Negative_Z();
    }

    //Collision detection for rotary axis

    //Transformation Matrix in A axis
    QMatrix4x4 transformMatrix_A;
    transformMatrix_A.setToIdentity();
    transformMatrix_A.rotate(3, 1.0, 0.0, 0.0);

    //Transformation Matrix in B axis
    QMatrix4x4 transformMatrix_B;
    transformMatrix_B.setToIdentity();
    transformMatrix_B.rotate(3, 0.0, 1.0, 0.0);

    //Transformation Matrix in C axis
    QMatrix4x4 transformMatrix_C;
    transformMatrix_C.setToIdentity();
    transformMatrix_C.rotate(3, 0.0, 0.0, 1.0);

    //get machine tool type
    QVector3D mtRotaryAxes = ccpVector[0].getFirstComp().m_mtRotaryAxes;

    //AB 5-axis Machine
    if(mtRotaryAxes == QVector3D(1,1,0)){
        for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){

            //if components has common rotary axis in A, then check collision
            if(ccpVector[ccp_ind].containsCommonRotaryAxis1()){
                qDebug()<<"colliion detection for rotating in A for"<<ccpVector[ccp_ind].getName();
                bool isCollided = voxelizer.rotationalCDForCCP(ccpVector[ccp_ind], transformMatrix_A, 1);

                if(!isCollided)
                    ccpVector[ccp_ind].not_collided_FirstAxis();
            }

            //if components has common rotary axis in B, then check collision
            if(ccpVector[ccp_ind].containsCommonRotaryAxis2()){
                qDebug()<<"colliion detection for rotating in B for"<<ccpVector[ccp_ind].getName();
                bool isCollided = voxelizer.rotationalCDForCCP(ccpVector[ccp_ind], transformMatrix_B, 2);

                if(!isCollided)
                    ccpVector[ccp_ind].not_collided_SecondAxis();
            }
        }
    }

    //BC 5-axis Machine
    if(mtRotaryAxes == QVector3D(0,1,1)){
        for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
            //if components has common rotary axis in B, then check collision
            if(ccpVector[ccp_ind].containsCommonRotaryAxis1()){
                qDebug()<<"colliion detection for rotating in B for"<<ccpVector[ccp_ind].getName();
                bool isCollided = voxelizer.rotationalCDForCCP(ccpVector[ccp_ind], transformMatrix_B, 1);

                if(!isCollided)
                    ccpVector[ccp_ind].not_collided_FirstAxis();
            }

            //if components has common rotary axis in C, then check collision
            if(ccpVector[ccp_ind].containsCommonRotaryAxis2()){
                qDebug()<<"colliion detection for rotating in C for"<<ccpVector[ccp_ind].getName();
                bool isCollided = voxelizer.rotationalCDForCCP(ccpVector[ccp_ind], transformMatrix_C, 2,true);

                if(!isCollided)
                    ccpVector[ccp_ind].not_collided_SecondAxis();
            }
        }
    }

    //AC 5-axis Machine
    if(mtRotaryAxes == QVector3D(1,0,1)){
        for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
            //if components has common rotary axis in A, then check collision
            if(ccpVector[ccp_ind].containsCommonRotaryAxis1()){
                qDebug()<<"colliion detection for rotating in A for"<<ccpVector[ccp_ind].getName();
                bool isCollided = voxelizer.rotationalCDForCCP(ccpVector[ccp_ind], transformMatrix_A, 1);

                if(!isCollided)
                    ccpVector[ccp_ind].not_collided_FirstAxis();
            }

            //if components has common rotary axis in C, then check collision
            if(ccpVector[ccp_ind].containsCommonRotaryAxis2()){
                qDebug()<<"colliion detection for rotating in C for"<<ccpVector[ccp_ind].getName();
                bool isCollided = voxelizer.rotationalCDForCCP(ccpVector[ccp_ind], transformMatrix_C, 2,true);

                if(!isCollided)
                    ccpVector[ccp_ind].not_collided_SecondAxis();
            }
        }
    }

    //A 4-axis Machine
    if(mtRotaryAxes == QVector3D(1,0,0)){
        for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
            //if components has common rotary axis in A, then check collision
            if(ccpVector[ccp_ind].containsCommonRotaryAxis1()){
                qDebug()<<"colliion detection for rotating in A for"<<ccpVector[ccp_ind].getName();
                bool isCollided = voxelizer.rotationalCDForCCP(ccpVector[ccp_ind], transformMatrix_A, 1);

                if(!isCollided)
                    ccpVector[ccp_ind].not_collided_FirstAxis();
            }
        }
    }

    //B 4-axis Machine
    if(mtRotaryAxes == QVector3D(0,1,0)){
        for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
            //if components has common rotary axis in B, then check collision
            if(ccpVector[ccp_ind].containsCommonRotaryAxis1()){
                qDebug()<<"colliion detection for rotating in B for"<<ccpVector[ccp_ind].getName();
                bool isCollided = voxelizer.rotationalCDForCCP(ccpVector[ccp_ind], transformMatrix_B, 1);

                if(!isCollided)
                    ccpVector[ccp_ind].not_collided_FirstAxis();
            }
        }
    }

    //C 4-axis Machine
    if(mtRotaryAxes == QVector3D(0,0,1)){
        for(int ccp_ind = 0; ccp_ind < ccpVector.size(); ++ccp_ind){
            //if components has common rotary axis in C, then check collision
            if(ccpVector[ccp_ind].containsCommonRotaryAxis1()){
                qDebug()<<"colliion detection for rotating in C for"<<ccpVector[ccp_ind].getName();
                bool isCollided = voxelizer.rotationalCDForCCP(ccpVector[ccp_ind], transformMatrix_C, 1);

                if(!isCollided)
                    ccpVector[ccp_ind].not_collided_FirstAxis();
            }
        }
    }
    qDebug()<<endl;
}

QVector3D GroupingPreProcessor::setNormal(int i)
{
    if (i==0)
        return { 0.0,0.0,-1.0};
    else if (i==1)
        return { 0.0,1.0,0.0};
    else if (i==2)
        return { 1.0,0.0,0.0};
    else if (i==3)
        return { -1.0,0.0,0.0};
    else if (i==4)
        return { 0.0,-1.0,0.0};
    else
        return { 0.0,0.0,1.0};
}
