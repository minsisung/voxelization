#ifndef VOXEL_H
#define VOXEL_H

class Voxel
{
public:
    Voxel();
    void setLinkType(char type){linkType = type;}
    char getLinkType(){return linkType;}

    void collide(){collisionStatus = true;}
    void notCollide(){collisionStatus = false;}
    bool isCollide(){return collisionStatus;}


    void coincident(){coincidentStatus = true;}
    void notCoincident(){coincidentStatus = false;}
    bool isCoincident(){return coincidentStatus;}

    void setShellType(char type){shellType = type;}
    char getShellType(){return shellType;}

private:
    char linkType = 'E';   //E: empty
    bool collisionStatus = false;
    bool coincidentStatus = false;

    //E: empty  //O: outer shell  //I: inner shell //N: not inside of model
    char shellType = 'E';
};
#endif // VOXEL_H
