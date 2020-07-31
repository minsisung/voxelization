#ifndef VOXEL_H
#define VOXEL_H

class Voxel
{
public:
    Voxel();
    void setVoxelLinkType(char type){VoxelLinkType = type;}
    char getVoxelLinkType(){return VoxelLinkType;}

    void collide(){collisionStatus = true;}
    bool isCollide(){return collisionStatus;}

private:
    char VoxelLinkType = 'E';   //E: empty
    bool collisionStatus = false;

};
#endif // VOXEL_H
