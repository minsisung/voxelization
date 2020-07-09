#ifndef VOXEL_H
#define VOXEL_H

class Voxel
{
public:
    Voxel();
    void setOutterShellLinkType(char type){outterShellLinkType = type;}
    char getOutterShellLinkType(){return outterShellLinkType;}

    void collide(){collisionStatus = true;}
    bool isCollide(){return collisionStatus;}


    void coincident(){coincidentStatus = true;}
    bool isCoincident(){return coincidentStatus;}

    void setInnerShellLinkType(char type){innerShellType = type;}
    char getInnerShellLinkType(){return innerShellType;}

    void setOutsideVoxelLinkType(char linkType){outsideVoxelLinkType = linkType;}
    char getOutsideVoxelLinkType(){return outsideVoxelLinkType;}

private:
    char outterShellLinkType = 'E';   //E: empty
    bool collisionStatus = false;
    bool coincidentStatus = false;
    char outsideVoxelLinkType = 'E';

    //E: empty
    char innerShellType = 'E';

};
#endif // VOXEL_H
