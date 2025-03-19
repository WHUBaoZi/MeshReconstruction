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

    // ������������������
    size_t polygonCount() const;

    // ���������
    size_t pointCount() const;

    // ĳ����Ķ�������
    size_t vertexCount(size_t n) const;

    // ��ȡ�����ռ��еĶ�������
    void getIndexSpacePoint(size_t n, size_t v, openvdb::Vec3d& pos) const;

private:
    const Mesh* mesh;
    const openvdb::math::Transform::Ptr transform;
};

class Voxel
{

};