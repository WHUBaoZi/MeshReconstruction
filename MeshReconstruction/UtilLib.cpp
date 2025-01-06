#include "UtilLib.h"

void UtilLib::MeshFiltering(Mesh& mesh, int iterCount)
{
	auto fNormalMap = mesh.property_map<face_descriptor, Vector_3>("f:normal").first;
	// 设置距离权重公式的σ,角度权重公式的θ
	double sigma = 0, sita = 20 * PI / 180;	// θ初始化为20°
	double standardDot = std::cos(sita);
	{
		double sum = 0;
		int edgeCount = 0;
		for (auto edge : mesh.edges())
		{
			Point_3 p1 = mesh.point(mesh.vertex(edge, 0));
			Point_3 p2 = mesh.point(mesh.vertex(edge, 1));
			sum += std::sqrt(CGAL::squared_distance(p1, p2));
			edgeCount++;
		}
		sigma = sum / edgeCount;
	}
	// 迭代滤波
	for (size_t round = 0; round < iterCount; round++)
	{
		// 计算滤波后的法向量
		std::map<face_descriptor, Vector_3> filteredNormalMap;
		int idx = 0;
		for (auto face : mesh.faces())
		{
			idx++;
			filteredNormalMap[face] = Vector_3(0, 0, 0);
			Point_3 centerPi = GetFaceCenter(face, mesh);
			Vector_3 normalFi = fNormalMap[face];
			auto oneRingFaces = GetTriFacesKRing(face, 1, mesh);
			for (auto neighborFace : oneRingFaces)
			{
				double alpha = 0, beta = 0;		// 权重
				Vector_3 normalFj = fNormalMap[neighborFace];
				double dot = normalFj * normalFi;
				if (dot < standardDot)	// 夹角大于阈值跳过
				{
					continue;
				}
				Point_3 centerPj = GetFaceCenter(neighborFace, mesh);
				alpha = std::exp(-(CGAL::squared_distance(centerPi, centerPj) / (2 * sigma * sigma)));
				double tmp1 = std::pow(1 - std::fabs(dot), 2);
				double tmp2 = std::pow(1 - std::cos(sita), 2);
				beta = std::exp(-(tmp1 / tmp2));
				double faceArea = CGAL::Polygon_mesh_processing::face_area(neighborFace, mesh);
				Vector_3 filteredNormal = faceArea * alpha * beta * normalFj;
				filteredNormalMap[face] += filteredNormal;
			}
			filteredNormalMap[face] /= std::sqrt(filteredNormalMap[face].squared_length());
		}
		// 根据滤波法向量调整点位置
		idx = 0;
		for (auto vertex : mesh.vertices())
		{
			idx++;
			int facesCount = 0;
			Point_3 point = mesh.point(vertex);
			Vector_3 deltaVector(0, 0, 0);
			for (auto face : CGAL::faces_around_target(mesh.halfedge(vertex), mesh))
			{
				if (face == mesh.null_face())
				{
					continue;
				}
				facesCount++;
				Point_3 centerPointFk = GetFaceCenter(face, mesh);
				Vector_3 vectorVC = centerPointFk - point;
				deltaVector += CGAL::scalar_product(vectorVC, filteredNormalMap[face]) * fNormalMap[face];
				deltaVector;
			}
			if (facesCount != 0)
			{
				deltaVector /= facesCount;
			}
			mesh.point(vertex) += deltaVector;
		}
		CGAL::Polygon_mesh_processing::compute_face_normals(mesh, fNormalMap);
	}
}

Point_3 UtilLib::GetFaceCenter(const face_descriptor& face, const Mesh& mesh)
{
	int size = 0;
	Vector_3 sum(0, 0, 0);
	for (auto vertex : CGAL::vertices_around_face(mesh.halfedge(face), mesh))
	{
		Point_3 point = mesh.point(vertex);
		sum += Vector_3(point.x(), point.y(), point.z());
		size++;
	}
	sum /= size;
	Point_3 centerPoint = Point_3(sum.x(), sum.y(), sum.z());
	return centerPoint;
}

std::set<face_descriptor> UtilLib::GetTriFacesKRing(const face_descriptor& face, int kRing, const Mesh& mesh)
{
	std::set<face_descriptor> faces;
	faces.insert(face);
	std::set<vertex_descriptor> vertices;
	std::queue<std::pair<face_descriptor, int>> faceQueue;
	faceQueue.push(std::make_pair(face, 0));

	while (!faceQueue.empty())
	{
		auto front = faceQueue.front();
		faceQueue.pop();
		face_descriptor currFace = front.first;
		int currKRing = front.second;

		if (currKRing < kRing)
		{
			for (halfedge_descriptor halfedge : CGAL::halfedges_around_face(mesh.halfedge(currFace), mesh))
			{
				vertex_descriptor vertex = mesh.target(halfedge);
				if (vertices.find(vertex) == vertices.end())
				{
					vertices.insert(vertex);
					for (face_descriptor neighborFace : CGAL::faces_around_target(halfedge, mesh))
					{
						if (neighborFace != mesh.null_face() && faces.find(neighborFace) == faces.end())
						{
							faces.insert(neighborFace);
							faceQueue.push(std::make_pair(neighborFace, currKRing + 1));
						}
					}
				}

			}
		}
	}
	return faces;
}


double UtilLib::ComputeVPlanarityOfKRing(const vertex_descriptor& vertex, int kRing, const Mesh& mesh)
{
	std::set<vertex_descriptor> neighbors = GetVerticesKRing(vertex, kRing, mesh);
	std::vector<Point_3> points;
	for (auto vertex : neighbors)
	{
		points.push_back(mesh.point(vertex));
	}
	Plane_3 plane;
	double planarity = linear_least_squares_fitting_3(points.begin(), points.end(), plane, CGAL::Dimension_tag<0>());
	return planarity;
}

std::set<vertex_descriptor> UtilLib::GetVerticesKRing(const vertex_descriptor& vertex, int kRing, const Mesh& mesh)
{
	std::set<vertex_descriptor> vertices;
	vertices.insert(vertex);
	std::queue<std::pair<vertex_descriptor, int>> verticesQueue;
	verticesQueue.push(std::make_pair(vertex, 0));
	while (!verticesQueue.empty())
	{
		auto front = verticesQueue.front();
		vertex_descriptor currVertex = front.first;
		int currKRing = front.second;
		verticesQueue.pop();
		if (currKRing < kRing)
		{
			for (auto neighbor : CGAL::vertices_around_target(currVertex, mesh))
			{
				if (vertices.find(neighbor) == vertices.end())
				{
					vertices.insert(neighbor);
					verticesQueue.push(std::make_pair(neighbor, currKRing + 1));
				}
			}
		}
	}
	return vertices;
}

std::vector<vertex_descriptor> UtilLib::GetVerticesAroundFace(const CGAL::SM_Face_index& face, const Mesh& mesh)
{
	std::vector<vertex_descriptor> vertices;
	for (halfedge_descriptor halfedge : CGAL::halfedges_around_face(mesh.halfedge(face), mesh))
	{
		vertices.push_back(mesh.target(halfedge));
	}
	return vertices;
}

CGAL::Color UtilLib::GetRandomColor()
{
	int R = -1; int G = -1; int B = -1;
	while (R < 150) { R = rand() % 256; }
	while (G < 150) { G = rand() % 256; }
	while (B < 150) { B = rand() % 256; }
	return CGAL::Color(R, G, B);
}

std::set<int> UtilLib::GetPartitionNeighbors(int id, const std::map<int, std::set<face_descriptor>>& partitionFacesMap, const Mesh& mesh)
{
	auto fChartMap = mesh.property_map<face_descriptor, int>("f:chart").first;
	std::set<int> neighbors;
	for (auto face : partitionFacesMap.at(id))
	{
		for (const auto& neighbor : CGAL::faces_around_face(mesh.halfedge(face), mesh))
		{
			if (neighbor != Mesh::null_face())
			{
				if (fChartMap[neighbor] != id)
				{
					neighbors.insert(fChartMap[neighbor]);
				}
			}
		}
	}
	return neighbors;
}

Vector_3 UtilLib::GetPartitionAverageNormal(int id, const std::map<int, std::set<face_descriptor>>& partitionFacesMap, const Mesh& mesh)
{
	auto fNormalMap = mesh.property_map<face_descriptor, Vector_3>("f:normal").first;
	Vector_3 normal(0.0, 0.0, 0.0);
	for (const auto& face : partitionFacesMap.at(id))
	{
		normal += fNormalMap[face];
	}
	normal /= partitionFacesMap.at(id).size();
	return normal;
}

Mesh UtilLib::MakeCube(const Point_3& minPoint, const Point_3& maxPoint)
{
	Mesh cubeMesh;
	vertex_descriptor v0 = cubeMesh.add_vertex(minPoint);
	vertex_descriptor v1 = cubeMesh.add_vertex(maxPoint);
	vertex_descriptor v2 = cubeMesh.add_vertex(Point_3(maxPoint.x(), minPoint.y(), minPoint.z()));
	vertex_descriptor v3 = cubeMesh.add_vertex(Point_3(maxPoint.x(), minPoint.y(), maxPoint.z()));
	vertex_descriptor v4 = cubeMesh.add_vertex(Point_3(minPoint.x(), maxPoint.y(), maxPoint.z()));
	vertex_descriptor v5 = cubeMesh.add_vertex(Point_3(minPoint.x(), maxPoint.y(), minPoint.z()));
	vertex_descriptor v6 = cubeMesh.add_vertex(Point_3(minPoint.x(), minPoint.y(), maxPoint.z()));
	vertex_descriptor v7 = cubeMesh.add_vertex(Point_3(maxPoint.x(), maxPoint.y(), minPoint.z()));

	cubeMesh.add_face(v0, v2, v3);
	cubeMesh.add_face(v0, v3, v6);
	cubeMesh.add_face(v0, v6, v4);
	cubeMesh.add_face(v0, v4, v5);
	cubeMesh.add_face(v0, v5, v7);
	cubeMesh.add_face(v0, v7, v2);
	cubeMesh.add_face(v1, v6, v3);
	cubeMesh.add_face(v1, v4, v6);
	cubeMesh.add_face(v1, v5, v4);
	cubeMesh.add_face(v1, v7, v5);
	cubeMesh.add_face(v1, v2, v7);
	cubeMesh.add_face(v1, v3, v2);
	
	return cubeMesh;
}


bool UtilLib::IsPolyhedronValid(const Mesh& mesh)
{
	bool bIsValid = false;
	// Mesh has vertex that actually exists
	for (const auto& vertex : mesh.vertices())
	{
		bIsValid = true;
		break;
	}
	// Mesh has only one component
	std::map<Mesh::Face_index, std::size_t> face_component_map;
	boost::associative_property_map<std::map<Mesh::Face_index, std::size_t>> fcm(face_component_map);
	if (CGAL::Polygon_mesh_processing::connected_components(mesh, fcm) > 1)
	{
		bIsValid = false;
	}

	return bIsValid;
}

Vector_3 compute_orthogonal_vector(const Vector_3& normal)
{
	if (normal.x() != 0 || normal.y() != 0) {
		// 法向量不平行于 z 轴时，返回 (0, c, -b)
		return Kernel::Vector_3(0, normal.z(), -normal.y());
	}
	else {
		// 法向量平行于 z 轴时，返回 (1, 0, 0)
		return Kernel::Vector_3(1, 0, 0);
	}
}

void UtilLib::CreatePlaneMesh(const Plane_3& plane, const Point_3& centerPoint, Mesh& mesh, double size)
{
	// 法向量
	Kernel::Vector_3 normal(plane.a(), plane.b(), plane.c());

	// 确定平面上的中心点
	Point_3 center = plane.projection(centerPoint);

	// 创建两个任意正交向量 (u, v) 来定义平面
	Kernel::Vector_3 u = compute_orthogonal_vector(normal);
	Kernel::Vector_3 v = CGAL::cross_product(normal, u);

	// 归一化并放大到指定大小
	u = size * u / std::sqrt(u.squared_length());
	v = size * v / std::sqrt(v.squared_length());

	// 平面的四个顶点
	Point_3 p1 = center + u + v;
	Point_3 p2 = center + u - v;
	Point_3 p3 = center - u - v;
	Point_3 p4 = center - u + v;

	// 添加顶点到网格
	auto v1 = mesh.add_vertex(p1);
	auto v2 = mesh.add_vertex(p2);
	auto v3 = mesh.add_vertex(p3);
	auto v4 = mesh.add_vertex(p4);

	// 添加两组三角形形成矩形
	mesh.add_face(v1, v2, v3);
	mesh.add_face(v1, v3, v4);
}

Point_3 UtilLib::GetMeshCenterPoint(const Mesh& mesh)
{
	double x = 0.0, y = 0.0, z = 0.0;
	int num = 0;
	for (const auto& point : mesh.points())
	{
		x += point.x();
		y += point.y();
		z += point.z();
		++num;
	}
	x /= num;
	y /= num;
	z /= num;
	return Point_3(x, y, z);
}

void UtilLib::CentralizeMesh(Mesh& mesh)
{
	// 1. 初始化边界
	Point_3 boxMin(DBL_MAX, DBL_MAX, DBL_MAX);
	Point_3 boxMax(-DBL_MAX, -DBL_MAX, -DBL_MAX);

	// 2. 遍历所有顶点，计算包围盒
	for (const auto& p : mesh.points()) 
	{
		boxMin = Point_3(std::min(boxMin.x(), p.x()),
			std::min(boxMin.y(), p.y()),
			std::min(boxMin.z(), p.z()));
		boxMax = Point_3(std::max(boxMax.x(), p.x()),
			std::max(boxMax.y(), p.y()),
			std::max(boxMax.z(), p.z()));
	}

	// 3. 计算中心点
	Point_3 center((boxMin.x() + boxMax.x()) / 2.0,
		(boxMin.y() + boxMax.y()) / 2.0,
		(boxMin.z() + boxMax.z()) / 2.0);

	// 4. 平移所有顶点
	for (const auto& v : mesh.vertices()) 
	{
		Point_3 p = mesh.point(v);
		mesh.point(v) = Point_3(p.x() - center.x(), p.y() - center.y(), p.z() - center.z());
	}
}