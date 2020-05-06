#ifndef VOXELIZER_H
#define VOXELIZER_H
#include <QVector>
#include <stl_reader.h>
#include <voxel.h>

class Voxelizer
{
public:
    Voxelizer();
    QVector < QVector < QVector< Voxel > > > voxelspace;
    void Voxelize(float spaceLength, float voxelSize);
    void resize(int size);

};

#endif // VOXELIZER_H
