#ifndef VOXEL_H
#define VOXEL_H

class Voxel
{
public:
    Voxel();
    void setStatus(char status){Status = status;}
    int getStatus(){return Status;}

    void collide(){collisionStatus = true;}
    bool isCollide(){return collisionStatus;}
private:
    char Status = 'E';   //E: empty  O: occupied  C: collided
    bool collisionStatus = false;
};
#endif // VOXEL_H
