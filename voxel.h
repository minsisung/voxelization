#ifndef VOXEL_H
#define VOXEL_H
#include <QChar>
#include <QSet>

class Voxel
{
public:
    Voxel();
    void setVoxelLinkType(QChar type){VoxelLinkType = type;}
    QChar getVoxelLinkType(){return VoxelLinkType;}

    void setComponentNumber(int number){componentNumber = number;}
    int getComponentNumber(){return componentNumber;}

    void collide(){collisionStatus = true;}
    bool isCollide(){return collisionStatus;}

private:
    QChar VoxelLinkType = 'E';   //E: empty
    bool collisionStatus = false;
    int componentNumber = 0;
};


#endif // VOXEL_H
