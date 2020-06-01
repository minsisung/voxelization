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

    void setNormalPointToMinus(){normalPointToXMinus = true;}
    bool isNormalPointMinus(){return normalPointToXMinus;}
private:
    char Status = 'E';   //E: empty  O: occupied  C: collided
    bool collisionStatus = false;
    bool normalPointToXMinus = false;
};
#endif // VOXEL_H
