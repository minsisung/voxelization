#ifndef VOXEL_H
#define VOXEL_H

class Voxel
{
public:
    Voxel();
    void setStatus(int status){Status = status;}
    int getStatus(){return Status;}
private:
    int Status = 0;   //0: empty  1: occupied  2: collided
};
#endif // VOXEL_H
