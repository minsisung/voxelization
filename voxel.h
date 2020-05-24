#ifndef VOXEL_H
#define VOXEL_H
#include<QVector>

class Voxel
{
public:
    Voxel();
    void addCollide(char component){collideList.append(component);}
    bool isCoolide(char component){return collideList.contains(component);}
    int listSize(){return collideList.size();}
private:
    QVector<char> collideList;   //E: empty  O: occupied  C: collided
};
#endif // VOXEL_H
