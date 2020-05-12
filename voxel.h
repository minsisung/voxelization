#ifndef VOXEL_H
#define VOXEL_H

class Voxel
{
public:
    Voxel();
    void setStatus(char status){Status = status;}
    int getStatus(){return Status;}
private:
    char Status = 'E';   //E: empty  O: occupied  C: collided
};
#endif // VOXEL_H
