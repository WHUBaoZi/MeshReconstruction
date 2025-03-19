#include "Voxel.h"

size_t CGALMeshAdapter::polygonCount() const
{
    return std::distance(mesh->faces_begin(), mesh->faces_end());
}

size_t CGALMeshAdapter::pointCount() const
{
    return std::distance(mesh->vertices_begin(), mesh->vertices_end());
}

size_t CGALMeshAdapter::vertexCount(size_t n) const
{
    auto f = std::next(mesh->faces_begin(), n);
    return std::distance(mesh->halfedges_around_face(mesh->halfedge(*f)).begin(),
        mesh->halfedges_around_face(mesh->halfedge(*f)).end());
}

void CGALMeshAdapter::getIndexSpacePoint(size_t n, size_t v, openvdb::Vec3d& pos) const
{
    auto f = std::next(mesh->faces_begin(), n);
    auto h = mesh->halfedge(*f);
    auto v_it = mesh->halfedges_around_face(h).begin();
    std::advance(v_it, v);

    Point_3 p = mesh->point(mesh->target(*v_it));

    // 转换到 OpenVDB 索引空间
    pos = transform->worldToIndex(openvdb::Vec3d(p.x(), p.y(), p.z()));
}
