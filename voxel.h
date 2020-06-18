#ifndef VOXEL_H
#define VOXEL_H

class Voxel
{
public:
    Voxel();
    void setStatus(char status){Status = status;}
    char getStatus(){return Status;}

    void collide(){collisionStatus = true;}
    bool isCollide(){return collisionStatus;}


    void coincident(){coincidentStatus = true;}
    bool isCoincident(){return coincidentStatus;}

    void setNormalPointToMinus(){normalPointToXMinus = true;}
    bool isNormalPointMinus(){return normalPointToXMinus;}

    void setShellType(char type){shellType = type;}
    char getShellType(){return shellType;}

private:
    char Status = 'E';   //E: empty  O: occupied  C: collided
    bool collisionStatus = false;
    bool coincidentStatus = false;
    bool normalPointToXMinus = false;

    //E: empty  //O: outer shell  //I: inner shell //N: not inside of model
    char shellType = 'E';
};
#endif // VOXEL_H
