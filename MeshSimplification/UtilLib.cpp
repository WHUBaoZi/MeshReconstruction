#include "UtilLib.h"

void UtilLib::meshFiltering(Mesh& mesh, int iterCount)
{
	auto fNormalMap = mesh.property_map<face_descriptor, Vector_3>("f:normal").first;
	// 设置距离权重公式的σ,角度权重公式的θ
	double sigma = 0, sita = 20 * M_PI / 180;	// θ初始化为20°
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
			Point_3 centerPi = getFaceCenter(face, mesh);
			Vector_3 normalFi = fNormalMap[face];
			auto oneRingFaces = getTriFacesKRing(face, 1, mesh);
			for (auto neighborFace : oneRingFaces)
			{
				double alpha = 0, beta = 0;		// 权重
				Vector_3 normalFj = fNormalMap[neighborFace];
				double dot = normalFj * normalFi;
				if (dot < standardDot)	// 夹角大于阈值跳过
				{
					continue;
				}
				Point_3 centerPj = getFaceCenter(neighborFace, mesh);
				alpha = std::exp(-(CGAL::squared_distance(centerPi, centerPj) / (2 * sigma * sigma)));
				double tmp1 = std::pow(1 - std::fabs(dot), 2);
				double tmp2 = std::pow(1 - std::cos(sita), 2);
				beta = std::exp(-(tmp1 / tmp2));
				double faceArea = getTriFaceArea(neighborFace, mesh);
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
				Point_3 centerPointFk = getFaceCenter(face, mesh);
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

Point_3 UtilLib::getFaceCenter(const face_descriptor& face, const Mesh& mesh)
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

std::set<face_descriptor> UtilLib::getTriFacesKRing(const face_descriptor& face, int kRing, const Mesh& mesh)
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

double UtilLib::getTriFaceArea(const face_descriptor& face, const Mesh& mesh)
{
	int size = 0;
	double faceArea = 0;
	for (auto vertex : CGAL::vertices_around_face(mesh.halfedge(face), mesh))
	{
		size++;
	}
	if (size == 3)
	{
		Point_3 points[3];
		int i = 0;
		for (auto vertex : CGAL::vertices_around_face(mesh.halfedge(face), mesh))
		{
			points[i] = mesh.point(vertex);
			i++;
		}
		Vector_3 vectorA = points[1] - points[0];
		Vector_3 vectorB = points[2] - points[0];
		faceArea = 0.5 * std::sqrt(CGAL::cross_product(vectorA, vectorB).squared_length());
	}
	return faceArea;
}


double UtilLib::computeVPlanarityOfKRing(const vertex_descriptor& vertex, int kRing, const Mesh& mesh)
{
	std::set<vertex_descriptor> neighbors = getVerticesKRing(vertex, kRing, mesh);
	std::vector<Point_3> points;
	for (auto vertex : neighbors)
	{
		points.push_back(mesh.point(vertex));
	}
	Plane_3 plane;
	double planarity = linear_least_squares_fitting_3(points.begin(), points.end(), plane, CGAL::Dimension_tag<0>());
	return planarity;
}

std::set<vertex_descriptor> UtilLib::getVerticesKRing(const vertex_descriptor& vertex, int kRing, const Mesh& mesh)
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

std::vector<vertex_descriptor> UtilLib::getVerticesAroundFace(const CGAL::SM_Face_index& face, const Mesh& mesh)
{
	std::vector<vertex_descriptor> vertices;
	for (halfedge_descriptor halfedge : CGAL::halfedges_around_face(mesh.halfedge(face), mesh))
	{
		vertices.push_back(mesh.target(halfedge));
	}
	return vertices;
}

CGAL::Color UtilLib::getRandomColor()
{
	int R = -1; int G = -1; int B = -1;
	while (R < 150) { R = rand() % 256; }
	while (G < 150) { G = rand() % 256; }
	while (B < 150) { B = rand() % 256; }
	return CGAL::Color(R, G, B);
}

std::set<int> UtilLib::getPartitionNeighbors(int id, const std::map<int, std::set<face_descriptor>>& partitionFacesMap, const Mesh& mesh)
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

Vector_3 UtilLib::getPartitionAverageNormal(int id, const std::map<int, std::set<face_descriptor>>& partitionFacesMap, const Mesh& mesh)
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

Mesh UtilLib::makeCube(const Point_3& minPoint, const Point_3& maxPoint)
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

bool UtilLib::intersection(const Plane_3& plane, const Mesh& mesh)
{
	const double epsilon = 1e-2; // 容差值，可根据精度需求调整

	bool bHasPositive = false; // 是否存在严格正值点
	bool bHasNegative = false; // 是否存在严格负值点

	for (const auto& point : mesh.points())
	{
		// 计算点到平面的距离
		double value = (plane.a() * point.x() + plane.b() * point.y() + plane.c() * point.z() + plane.d()) / std::sqrt(plane.a() * plane.a() + plane.b() * plane.b() + plane.c() * plane.c());

		// 使用 epsilon 判断
		if (value > epsilon)
		{
			bHasPositive = true;
		}
		else if (value < -epsilon)
		{
			bHasNegative = true;
		}

		// 提前退出条件：网格既有正侧点又有负侧点
		if (bHasPositive && bHasNegative)
		{
			return true;
		}
	}

	// 所有点都在平面同侧或平面上，不相交
	return false;
}

bool UtilLib::intersection(const Line_3& line, const face_descriptor& triangleFace, const Mesh& mesh)
{
	const double EPSILON = 1e-8;

	std::vector<Point_3> V;
	// 获取三角形顶点
	for (const auto& vertex : CGAL::vertices_around_face(mesh.halfedge(triangleFace), mesh))
	{
		V.push_back(mesh.point(vertex));
	}

	// 计算三角形边
	Vector_3 e1 = V[1] - V[0];
	Vector_3 e2 = V[2] - V[0];

	// 获取直线起点和方向向量
	Point_3 P0 = line.point(0); // 直线上的一点
	Vector_3 dir = line.to_vector(); // 直线方向向量

	// 计算方向向量与 e2 的叉积
	Vector_3 h = CGAL::cross_product(dir, e2);
	double a = e1 * h; // 点积

	// 如果 a 近似为 0，直线与三角形平面平行
	if (std::abs(a) < EPSILON) return false;

	double f = 1.0 / a;
	Vector_3 s = P0 - V[0];
	double u = f * (s * h); // 点积计算 u

	// 检查 u 是否在 [0, 1] 范围内
	if (u < 0.0 || u > 1.0) return false;

	Vector_3 q = CGAL::cross_product(s, e1);
	double v = f * (dir * q); // 点积计算 v

	// 检查 v 是否在 [0, 1]，且 u + v 不超过 1
	if (v < 0.0 || u + v > 1.0) return false;

	// 如果能通过所有检查，说明直线与三角形相交
	return true;
}


bool UtilLib::isPolyhedronValid(const Mesh& mesh)
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

void UtilLib::createPlaneMesh(const Plane_3& plane, const Point_3& centerPoint, Mesh& mesh, double size)
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

Point_3 UtilLib::getMeshCenterPoint(const Mesh& mesh)
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

//Point_3 UtilLib::getPolyhedronCentroid(const Mesh& polyhedron)
//{
//	std::vector<Point_3> centroidPoints;
//	std::vector<double> facesArea;
//	for (const auto& face : polyhedron.faces())
//	{
//		facesArea.push_back(CGAL::Polygon_mesh_processing::face_area(face, polyhedron));
//
//		for (const auto& vertex : polyhedron.vertices_around_face(polyhedron.halfedge(face)))
//		{
//
//		}
//	}
//}