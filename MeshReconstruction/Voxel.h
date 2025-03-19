#pragma once
#include "CGALTypes.h"

#include <openvdb/openvdb.h>
#include <openvdb/math/Transform.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/io/File.h>
#include <openvdb/tools/VolumeToMesh.h>

using GridType = openvdb::FloatGrid;

class CGALMeshAdapter 
{
public:
    explicit CGALMeshAdapter(const Mesh* mesh, const openvdb::math::Transform::Ptr transform)
        : mesh(mesh), transform(transform) {}

    // 面的数量（多边形数）
    size_t polygonCount() const;

    // 顶点的数量
    size_t pointCount() const;

    // 某个面的顶点数量
    size_t vertexCount(size_t n) const;

    // 获取索引空间中的顶点坐标
    void getIndexSpacePoint(size_t n, size_t v, openvdb::Vec3d& pos) const;

private:
    const Mesh* mesh;
    const openvdb::math::Transform::Ptr transform;
};

class Voxel
{

};