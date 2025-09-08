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

openvdb::FloatGrid::Ptr Voxel::sharpen_vdb(openvdb::FloatGrid::Ptr sdfGrid)
{
    // 计算梯度场
    openvdb::tools::Gradient<openvdb::FloatGrid> gradient(*sdfGrid);
    auto gradGrid = gradient.process();  // 计算梯度

    // 通过梯度来锐化
    openvdb::tools::LevelSetFilter<openvdb::FloatGrid> filter(*sdfGrid);
    filter.meanCurvature();

    return sdfGrid;  // 返回修改后的网格
}

Mesh Voxel::volumeToMesh(openvdb::FloatGrid::Ptr sdfGrid)
{
    std::vector<openvdb::Vec3s> points;
    std::vector<openvdb::Vec3I> triangles;
    std::vector<openvdb::Vec4I> quads;
    openvdb::tools::volumeToMesh(*sdfGrid, points, triangles, quads);
    Mesh voxelMesh;
    std::vector<vertex_descriptor> vertices;
    for (const auto& p : points)
    {
        vertices.push_back(voxelMesh.add_vertex(Point_3(p[0], p[1], p[2])));
    }
    for (const auto& quad : quads)
    {
        voxelMesh.add_face(vertices[quad[3]], vertices[quad[2]], vertices[quad[1]], vertices[quad[0]]);
    }
    return voxelMesh;
}

size_t Voxel::countInteriorVoxels(const openvdb::FloatGrid::Ptr& grid)
{
    size_t count = 0;
    for (auto it = grid->cbeginValueOn(); it; ++it) 
    {
        if (it.getValue() <= 0.0f) ++count;
    }
    return count;
}

openvdb::Vec3f Voxel::sampleGradient(
    const openvdb::tools::GridSampler<openvdb::FloatTree, openvdb::tools::BoxSampler>& sampler,
    const openvdb::Vec3d& worldPos)
{
    // 获取体素单位长度作为差分步长
    const double eps = sampler.transform().voxelSize()[0];

    // 中心差分方向向量
    const openvdb::Vec3d dx(eps, 0.0, 0.0);
    const openvdb::Vec3d dy(0.0, eps, 0.0);
    const openvdb::Vec3d dz(0.0, 0.0, eps);

    const float gx = static_cast<float>(
        (sampler.wsSample(worldPos + dx) - sampler.wsSample(worldPos - dx)) / (2.0 * eps));
    const float gy = static_cast<float>(
        (sampler.wsSample(worldPos + dy) - sampler.wsSample(worldPos - dy)) / (2.0 * eps));
    const float gz = static_cast<float>(
        (sampler.wsSample(worldPos + dz) - sampler.wsSample(worldPos - dz)) / (2.0 * eps));
    return openvdb::Vec3f(gx, gy, gz);
}