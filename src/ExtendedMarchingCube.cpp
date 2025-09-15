#include "ExtendedMarchingCube.h"

//int main(int argc, char* argv[])
//{	
//	Mesh mesh;
//	CGAL::Polygon_mesh_processing::IO::read_polygon_mesh("D:/DATA/AcademicRelevance/MeshReconstruction/MeshReconstruction/MeshReconstruction/Data/Output/test6_HoleFill/MergedMesh.obj", mesh);
//	//mesh = UtilLib::CreateCube(Point_3(-10.0, -10.0, -10.0), Point_3(10.0, 10.0, 10.0));
//	openvdb::util::NullInterrupter interrupter;
//	float voxelSize = 0.3;
//	openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(voxelSize);
//	double diag = std::sqrt(3.0) * voxelSize;
//	CGALMeshAdapter adapter(&mesh, transform);
//	openvdb::FloatGrid::Ptr grid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
//		interrupter,
//		adapter,
//		*transform,
//		3.f,
//		3.f
//	);
//	ExtendedMarchingCube::ApplyExtendedMarchingCube(grid);
//	std::cout << "test";
//}


//void MeshToVolume(const Mesh& mesh, openvdb::math::Transform::Ptr transform, float exteriorWidth, float interiorWidth)
//{
//	constexpr float floatMax = std::numeric_limits<float>::max();
//	auto vec3dMax = openvdb::Vec3d(floatMax, floatMax, floatMax);
//	openvdb::Vec3dGrid::Ptr vectorGrid(new openvdb::Vec3dGrid(vec3dMax));
//	vectorGrid->setTransform(transform->copy());
//	float voxelSize = transform->voxelSize()[0];
//
//	// Convert narrow band width from voxel units to world space units.
//	exteriorWidth *= voxelSize;
//	// Avoid the unit conversion if the interior band width is set to
//	// inf or std::numeric_limits<float>::max().
//	if (interiorWidth < floatMax)
//	{
//		interiorWidth *= voxelSize;
//	}
//	auto tree = Tree(mesh.faces().begin(), mesh.faces().end(), mesh);
//
//
//	auto accessor = vectorGrid->getAccessor();
//
//	for (const auto& face : mesh.faces())
//	{
//		std::deque<openvdb::Coord> coordList;
//		Point_3 center = UtilLib::GetFaceCenter(face, mesh);
//		openvdb::Vec3d pos = transform->worldToIndex(openvdb::Vec3d(center.x(), center.y(), center.z()));
//		openvdb::Coord ijk = openvdb::Coord::floor(pos);
//		coordList.push_back(ijk);
//
//	}
//	
//}

Mesh ExtendedMarchingCube::ApplyExtendedMarchingCube(openvdb::FloatGrid::Ptr grid)
{
	openvdb::FloatGrid::Accessor accessor = grid->getAccessor();
	openvdb::math::Transform::Ptr transform = grid->transformPtr();
	openvdb::util::NullInterrupter interrupter;
	float voxelSize = 0.3;
	double diag = std::sqrt(3.0) * voxelSize;
	Mesh gridMesh;
	std::unordered_map<openvdb::Coord, GridCell> gridCells;
	for (auto iter = grid->cbeginValueOn(); iter; ++iter)
	{
		openvdb::Coord coord = iter.getCoord();
		gridCells.emplace(coord, GridCell());
		openvdb::Vec3f op = transform->indexToWorld(coord);
		Point_3 p(op.x(), op.y(), op.z());
		gridMesh.add_vertex(p);
	}
	CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "GridMesh.obj", gridMesh);

	std::vector<openvdb::Vec3f> vertices;
	std::unordered_set<int> features;
	std::vector<std::array<int, 3>> triangles;
	std::unordered_map<EdgeKey, int> edgeVertexIndexMap;
	int debugTmpNum = 0;
	openvdb::tools::GridSampler<openvdb::FloatTree, openvdb::tools::BoxSampler> sampler(grid->tree(), grid->transform());
	float minThresholdDot = std::cos(30.f * UtilLib::DEG_TO_RAD);
	float maxThresholdDot = std::cos(45.f * UtilLib::DEG_TO_RAD);

	for (auto iter = grid->cbeginValueOn(); iter; ++iter)
	{
		debugTmpNum++;
		openvdb::Coord coord = iter.getCoord();
		auto& gridCell = gridCells[coord];
		for (int i = 0; i < 8; i++)
		{
			gridCell.cellCoords[i] = cellOffsets[i] + coord;
			gridCell.sdf[i] = accessor.getValue(gridCell.cellCoords[i]);
			if (gridCell.sdf[i] < 0.f)
			{
				gridCell.cubeIndex |= (1 << i);
			}
		}
		int edgeMask = edgeTable[gridCell.cubeIndex];
		if (edgeMask == 0)
		{
			continue;
		}
		else
		{
			std::array<int, 12> edgeVertexIndices;
			edgeVertexIndices.fill(-1);
			for (int edge = 0; edge < 12; ++edge)
			{
				if (edgeMask & (1 << edge))
				{
					int v0 = edgeVertexPairs[edge][0];
					int v1 = edgeVertexPairs[edge][1];
					openvdb::Coord c0 = gridCell.cellCoords[v0];
					openvdb::Coord c1 = gridCell.cellCoords[v1];
					EdgeKey key(c0, c1);
					auto it = edgeVertexIndexMap.find(key);
					if (it != edgeVertexIndexMap.end())
					{
						edgeVertexIndices[edge] = it->second;
					}
					else
					{
						openvdb::Vec3f p0 = transform->indexToWorld(c0);
						openvdb::Vec3f p1 = transform->indexToWorld(c1);
						float val0 = gridCell.sdf[v0];
						float val1 = gridCell.sdf[v1];
						float t = val0 / (val0 - val1);
						openvdb::Vec3f vertex = p0 + t * (p1 - p0);
						int index = static_cast<int>(vertices.size());
						vertices.push_back(vertex);
						edgeVertexIndexMap[key] = index;
						edgeVertexIndices[edge] = index;
					}
				}
			}

			// 连通分量检测
			std::unordered_map<int, std::unordered_set<int>> adjMap;
			const int* triEdges = triTable[gridCell.cubeIndex];
			std::vector<std::array<int, 3>> cellTriangles;
			for (int i = 0; triEdges[i] != -1; i += 3)
			{
				int i0 = edgeVertexIndices[triEdges[i]];
				int i1 = edgeVertexIndices[triEdges[i + 1]];
				int i2 = edgeVertexIndices[triEdges[i + 2]];

				// 记录连接部分
				adjMap[i0].insert(i1);
				adjMap[i0].insert(i2);
				adjMap[i1].insert(i0);
				adjMap[i1].insert(i2);
				adjMap[i2].insert(i0);
				adjMap[i2].insert(i1);
				cellTriangles.push_back({ i0, i1, i2 });
			}

			std::unordered_set<int> visitedVertices;	// vertices index
			std::vector<std::vector<int>> connectedComponents;	// vertices index
			for (const auto& pair : adjMap)
			{
				const int& i = pair.first;
				if (visitedVertices.count(i))
				{
					continue;
				}
				connectedComponents.emplace_back(std::vector<int>({ i }));
				auto& component = *connectedComponents.rbegin();
				visitedVertices.insert(i);
				std::queue<int> q;
				q.push(i);
				while (!q.empty())
				{
					int front = q.front(); q.pop();
					for (const auto& connection : adjMap[front])
					{
						if (visitedVertices.count(connection)) continue;
						visitedVertices.insert(connection);
						component.push_back(connection);
						q.push(connection);
					}
				}
			}
			std::vector<std::vector<std::array<int, 3>>> componentTriangles(connectedComponents.size());
			for (int i = 0; i < connectedComponents.size(); i++)
			{
				const auto& connectedComponent = connectedComponents[i];
				std::unordered_set<int> points;
				points.insert(connectedComponent.begin(), connectedComponent.end());
				for (const auto& triangle : cellTriangles)
				{
					for (int j = 0; j < 3; j++)
					{
						if (points.count(triangle[j]))
						{
							componentTriangles[i].push_back(triangle);
							break;
						}
					}
				}
			}
			for (int i = 0; i < connectedComponents.size(); i++)
			{
				auto& connectedComponent = connectedComponents[i];
				std::vector<openvdb::Vec3f> normals;
				std::vector<openvdb::Vec3f> samplePoints;
				for (int j = 0; j < connectedComponent.size(); j++)
				{
					openvdb::Vec3f normal = Voxel::sampleGradient(sampler, vertices[connectedComponent[j]]);
					normals.push_back(normal.unit());
					samplePoints.push_back(vertices[connectedComponent[j]]);
				}

				// 1. 计算质心
				openvdb::Vec3f centroid(0.0f);
				for (const auto& sp : samplePoints)
					centroid += sp;
				centroid /= float(samplePoints.size());

				// 2. 坐标中心化
				std::vector<openvdb::Vec3f> centeredPoints;
				for (const auto& sp : samplePoints)
					centeredPoints.push_back(sp - centroid);

				// 3. 构造 N 和 d
				Eigen::MatrixXd N(centeredPoints.size(), 3);
				Eigen::VectorXd d(centeredPoints.size());
				for (size_t k = 0; k < centeredPoints.size(); k++)
				{
					const auto& si = centeredPoints[k];  // 使用平移后的点
					const auto& ni = normals[k];
					N.row(k) << ni.x(), ni.y(), ni.z();
					d.row(k) << ni.dot(si);  // 使用平移后的坐标
				}

				// 4. SVD 分解
				Eigen::JacobiSVD<Eigen::MatrixXd> svd(N, Eigen::ComputeFullU | Eigen::ComputeFullV);
				Eigen::Vector3d singularValues = svd.singularValues();
				Eigen::MatrixXd U = svd.matrixU();
				Eigen::MatrixXd V = svd.matrixV();
				Eigen::Vector3d p_centered;

				double r2 = singularValues(1) / singularValues(0);
				double r3 = singularValues(2) / singularValues(0);

				if (r2 < 0.15 && r3 < 0.05) // No feature
				{
					triangles.insert(triangles.end(), componentTriangles[i].begin(), componentTriangles[i].end());
					continue;
				}
				else if (r2 > 0.15 && r3 < 0.05) // Sharp edge feature
				{
					Eigen::Matrix3d S_inv = Eigen::Matrix3d::Zero();
					for (int k = 0; k < 2; ++k)
					{
						S_inv(k, k) = 1.0 / singularValues(k);
					}
					Eigen::MatrixXd N_pinv = V * S_inv * U.transpose();
					p_centered = N_pinv * d;

					// Check Valid
					Eigen::Vector3d p = p_centered + Eigen::Vector3d(centroid.x(), centroid.y(), centroid.z());
					bool invalid = false;

					double err = 0.0;
					for (size_t k = 0; k < samplePoints.size(); k++) {
						Eigen::Vector3d si(samplePoints[k].x(), samplePoints[k].y(), samplePoints[k].z());
						Eigen::Vector3d ni(normals[k].x(), normals[k].y(), normals[k].z());
						double dist = std::abs((p - si).dot(ni));
						err += dist;
					}
					err /= samplePoints.size();
					if (err > 0.25 * diag)
					{
						invalid = true;
					}
					if (invalid)
					{
						triangles.insert(triangles.end(), componentTriangles[i].begin(), componentTriangles[i].end());
						continue;
					}
				}
				else // Corner feature
				{
					//p_centered = svd.solve(d);
					triangles.insert(triangles.end(), componentTriangles[i].begin(), componentTriangles[i].end());
					continue;
				}

				// 5. 平移回原坐标系
				Eigen::Vector3d p = p_centered + Eigen::Vector3d(centroid.x(), centroid.y(), centroid.z());
				Point_3 cgalP(p.x(), p.y(), p.z());

				// 6. 添加顶点并构造三角形扇
				int indexP = static_cast<int>(vertices.size());
				vertices.push_back(openvdb::Vec3f(p.x(), p.y(), p.z()));
				features.insert(indexP);
				int size = connectedComponent.size();
				std::vector<Point_3> cgalPoints;
				for (int j = 0; j < size; j++)
				{
					const auto& vertex = vertices[connectedComponent[j]];
					cgalPoints.push_back(Point_3(vertex.x(), vertex.y(), vertex.z()));
				}
				Plane_3 plane;
				CGAL::linear_least_squares_fitting_3(cgalPoints.begin(), cgalPoints.end(), plane, CGAL::Dimension_tag<0>());
				Vector_3 normal = plane.orthogonal_vector();
				normal = normal / std::sqrt(normal.squared_length());
				{
					const auto& v0 = vertices[componentTriangles[i][0][0]];
					const auto& v1 = vertices[componentTriangles[i][0][1]];
					const auto& v2 = vertices[componentTriangles[i][0][2]];
					Point_3 p0(v0.x(), v0.y(), v0.z());
					Point_3 p1(v1.x(), v1.y(), v1.z());
					Point_3 p2(v2.x(), v2.y(), v2.z());

					Vector_3 triN = CGAL::cross_product(p1 - p0, p2 - p0);
					if (normal * triN < 0)  // 点积为负，方向相反
					{
						normal = -normal;
					}
				}
				Vector_3 ref = (std::abs(normal.x()) < 0.9) ? Vector_3(1, 0, 0) : Vector_3(0, 1, 0);
				Vector_3 u = ref - (ref * normal) * normal;
				u = u / std::sqrt(u.squared_length());
				Vector_3 v = CGAL::cross_product(normal, u);
				v = v / std::sqrt(v.squared_length());
				std::unordered_map<int, double> angleMap;
				for (int j = 0; j < cgalPoints.size(); j++)
				{
					const auto& point = cgalPoints[j];
					Vector_3 vector = point - cgalP;
					double x = vector * u;
					double y = vector * v;
					double angle = std::atan2(y, x);
					angleMap[connectedComponent[j]] = angle;
				}
				std::sort(connectedComponent.begin(), connectedComponent.end(), [&](const int& a, const int& b) {return angleMap[a] < angleMap[b]; });
				std::vector<std::array<int, 3>> fanTriangles;
				fanTriangles.reserve(size);
				for (int j = 0; j < size; ++j) {
					fanTriangles.push_back({ indexP, connectedComponent[j], connectedComponent[(j + 1) % size] });
				}
				triangles.insert(triangles.end(), fanTriangles.begin(), fanTriangles.end());
			}
		}
	}

	Mesh marchingCubesTestMesh;
	std::vector<vertex_descriptor> cgalVertices;
	std::unordered_set<vertex_descriptor> cgalFeatures;
	for (int i = 0; i < vertices.size(); i++)
	{
		openvdb::Vec3f op = vertices[i];
		Point_3 p(op.x(), op.y(), op.z());
		auto vertex = marchingCubesTestMesh.add_vertex(p);
		int id = vertex.id();
		int idx = vertex.idx();
		cgalVertices.push_back(vertex);
		if (features.count(i))
		{
			cgalFeatures.insert(vertex);
		}
	}
	for (auto& triangle : triangles)
	{
		for (int i = 0; i < 3; i++)
		{
			marchingCubesTestMesh.add_face(cgalVertices[triangle[0]], cgalVertices[triangle[2]], cgalVertices[triangle[1]]);
		}
	}

	CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "ExtendedMarchingCubesTestMesh_BeforeFlip.obj", marchingCubesTestMesh);
	// Flip edges
	for (auto edge : marchingCubesTestMesh.edges())
	{
		halfedge_descriptor h1 = marchingCubesTestMesh.halfedge(edge);
		if (h1 == Mesh::null_halfedge())
		{
			continue;
		}
		halfedge_descriptor h2 = marchingCubesTestMesh.opposite(h1);
		face_descriptor f1 = marchingCubesTestMesh.face(h1);
		face_descriptor f2 = marchingCubesTestMesh.face(h2);
		if (f1 == Mesh::null_face() || f2 == Mesh::null_face())
		{
			continue;
		}
		auto v1 = marchingCubesTestMesh.target(marchingCubesTestMesh.next(h1));
		auto v2 = marchingCubesTestMesh.target(marchingCubesTestMesh.next(h2));
		if (cgalFeatures.count(v1) && cgalFeatures.count(v2))
		{
			CGAL::Euler::flip_edge(h1, marchingCubesTestMesh);
		}
	}
	CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "ExtendedMarchingCubesTestMesh.obj", marchingCubesTestMesh);

	CGAL::Polygon_mesh_processing::stitch_borders(marchingCubesTestMesh);
	std::vector<std::pair<face_descriptor, face_descriptor>> intersectedTris;
	CGAL::Polygon_mesh_processing::self_intersections(marchingCubesTestMesh, std::back_inserter(intersectedTris));
	for (auto& p : intersectedTris)
	{
		marchingCubesTestMesh.remove_face(p.first);
		marchingCubesTestMesh.remove_face(p.second);
	}
	CGAL::Polygon_mesh_processing::remove_isolated_vertices(marchingCubesTestMesh);
	CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "ExtendedMarchingCubesTestMesh_RemoveIntersections.obj", marchingCubesTestMesh);
	return marchingCubesTestMesh;
}