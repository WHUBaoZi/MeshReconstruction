#include "UtilLib.h"

void UtilLib::FilterMesh(Mesh& mesh, int iterCount)
{
	auto fNormalMapOpt = mesh.property_map<face_descriptor, Vector_3>("f:normal");
	Mesh::Property_map<face_descriptor, Vector_3> fNormalMap;
	if (!fNormalMapOpt)
	{
		fNormalMap = mesh.add_property_map<face_descriptor, Vector_3>("f:normal").first;
		CGAL::Polygon_mesh_processing::compute_face_normals(mesh, fNormalMap);
	}
	else
	{
		fNormalMap = *fNormalMapOpt;
	}

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
			auto oneRingFaces = GetKRingFaces(face, 1, mesh);
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
			}
			if (facesCount != 0)
			{
				deltaVector /= facesCount;
			}
			mesh.point(vertex) += deltaVector;
		}
		CGAL::IO::write_OBJ("tmpFilteredMesh.obj", mesh);
		CGAL::Polygon_mesh_processing::compute_face_normals(mesh, fNormalMap);
	}
}

Point_3 UtilLib::GetFaceCenter(const face_descriptor& face, const Mesh& mesh)
{
	std::vector<Point_3> points;
	for (auto vertex : CGAL::vertices_around_face(mesh.halfedge(face), mesh)) {
		points.push_back(mesh.point(vertex));
	}
	return CGAL::centroid(points.begin(), points.end(), CGAL::Dimension_tag<0>());
}

std::unordered_set<face_descriptor> UtilLib::GetKRingFaces(const face_descriptor& face, int kRing, const Mesh& mesh)
{
	std::unordered_set<face_descriptor> faces;
	faces.insert(face); // contain self
	std::unordered_set<vertex_descriptor> vertices;
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

std::set<vertex_descriptor> UtilLib::GetKRingVertices(const vertex_descriptor& vertex, int kRing, const Mesh& mesh)
{
	std::set<vertex_descriptor> vertices;
	vertices.insert(vertex); // contain self
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

std::vector<vertex_descriptor> UtilLib::GetFaceVertices(const CGAL::SM_Face_index& face, const Mesh& mesh)
{
	std::vector<vertex_descriptor> vertices;
	for (halfedge_descriptor halfedge : CGAL::halfedges_around_face(mesh.halfedge(face), mesh))
	{
		vertices.push_back(mesh.target(halfedge));
	}
	return vertices;
}

Plane_3 UtilLib::FitPlaneFromFaces(const std::unordered_set<face_descriptor>& faces, const Mesh& mesh)
{
	std::set<vertex_descriptor> vertices;
	for (const auto& face : faces)
	{
		auto verticesOfFace = GetFaceVertices(face, mesh);
		vertices.insert(verticesOfFace.begin(), verticesOfFace.end());
	}
	std::vector<Point_3> points;
	for (const auto& vertex : vertices)
	{
		points.emplace_back(mesh.point(vertex));
	}
	Plane_3 plane;
	CGAL::linear_least_squares_fitting_3(points.begin(), points.end(), plane, CGAL::Dimension_tag<0>());
	return plane;
}

CGAL::Color UtilLib::GenerateRandomColor()
{
	int R = -1; int G = -1; int B = -1;
	while (R < 150) { R = rand() % 256; }
	while (G < 150) { G = rand() % 256; }
	while (B < 150) { B = rand() % 256; }
	return CGAL::Color(R, G, B);
}

Vector_3 UtilLib::GetPartitionAverageNormal(int partitionId, const std::map<int, std::set<face_descriptor>>& partitionFacesMap, const Mesh& mesh)
{
	auto fNormalMap = *mesh.property_map<face_descriptor, Vector_3>("f:normal");
	Vector_3 normal(0.0, 0.0, 0.0);
	for (const auto& face : partitionFacesMap.at(partitionId))
	{
		normal += fNormalMap[face];
	}
	normal /= partitionFacesMap.at(partitionId).size();
	return normal;
}

Mesh UtilLib::CreateCube(const Point_3& minPoint, const Point_3& maxPoint)
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

Mesh UtilLib::CreatePlaneMesh(const Plane_3& plane, const Point_3& centerPoint, double size)
{
	Mesh mesh;
	CreatePlaneMesh(plane, centerPoint, mesh, size);
	return mesh;
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

halfedge_descriptor UtilLib::GetHalfedge(vertex_descriptor source, vertex_descriptor target, const Mesh& mesh)
{
	for (halfedge_descriptor h : halfedges_around_target(target, mesh)) {
		if (mesh.source(h) == source)
			return h;
	}
	std::cerr << "Target halfedge undefined" << std::endl;
	return halfedge_descriptor();
}

//std::map<int, std::set<face_descriptor>> UtilLib::PartitionByNormal(Mesh& mesh, double threshold, double thresholdAngle)
//{
//	auto fNormalMapOpt = mesh.property_map<face_descriptor, Vector_3>("f:normal");
//	Mesh::Property_map<face_descriptor, Vector_3> fNormalMap;
//	if (!fNormalMapOpt) 
//	{
//		fNormalMap = mesh.add_property_map<face_descriptor, Vector_3>("f:normal", CGAL::NULL_VECTOR).first;
//		CGAL::Polygon_mesh_processing::compute_face_normals(mesh, fNormalMap);
//	}
//	else
//	{
//		fNormalMap = *fNormalMapOpt;
//	}
//	auto fChartMapOpt = mesh.property_map<face_descriptor, int>("f:chart");
//	Mesh::Property_map<face_descriptor, int> fChartMap;
//	if (!fChartMapOpt)
//	{
//		fChartMap = mesh.add_property_map<face_descriptor, int>("f:chart", -1).first;
//	}
//	else
//	{
//		fChartMap = *fChartMapOpt;
//	}
//	auto fColorMapOpt = mesh.property_map<face_descriptor, CGAL::Color>("f:color");
//	Mesh::Property_map<face_descriptor, CGAL::Color> fColorMap;
//	if (!fColorMapOpt)
//	{
//		fColorMap = mesh.add_property_map<face_descriptor, CGAL::Color>("f:color", CGAL::Color(0, 0, 0)).first;
//	}
//	else
//	{
//		fColorMap = *fColorMapOpt;
//	}
//
//	std::map<size_t, std::set<face_descriptor>> partitionsMap;
//	std::map<size_t, Vector_3> partitionsNormal;
//	size_t currenId = 0;
//	CGAL::Color currentColor = GenerateRandomColor();
//	const double cosThreshold = std::cos(thresholdAngle * DEG_TO_RAD);
//
//	for (const auto& seed : mesh.faces())
//	{
//		if (fChartMap[seed] != size_t(-1))
//			continue;
//
//		std::queue<face_descriptor> facesQueue;
//		fChartMap[seed] = currenId;
//		fColorMap[seed] = currentColor;
//		partitionsMap[currenId] = std::set<face_descriptor>();
//		partitionsMap[currenId].insert(seed);
//		facesQueue.push(seed);
//		Vector_3 avgNormal = fNormalMap[seed];
//
//		while (!facesQueue.empty())
//		{
//			face_descriptor face = facesQueue.front();
//			facesQueue.pop();
//
//			for (const auto& neighborFace : mesh.faces_around_face(mesh.halfedge(face)))
//			{
//				if (fChartMap[neighborFace] != size_t(-1))
//				{
//					continue;
//				}
//				Vector_3 neighborNormal = fNormalMap[neighborFace];
//				double dot = (avgNormal * neighborNormal);
//				double lenProduct = std::sqrt(avgNormal.squared_length() * neighborNormal.squared_length());
//				if (lenProduct == 0) continue;
//
//				double cosAngle = dot / lenProduct;
//				if (cosAngle > cosThreshold)
//				{
//					fChartMap[neighborFace] = currenId;
//					fColorMap[neighborFace] = currentColor;
//					partitionsMap[currenId].insert(neighborFace);
//					facesQueue.push(neighborFace);
//					avgNormal = avgNormal + neighborNormal;
//				}
//			}
//		}
//		partitionsNormal[currenId] = avgNormal / std::sqrt(avgNormal.squared_length());
//		currenId++;
//		currentColor = GenerateRandomColor();
//	}
//	CGAL::IO::write_PLY(TEST_OUTPUT_PATH + "OriginalClassifyMesh.ply", mesh, CGAL::parameters::face_color_map(fColorMap).use_binary_mode(false));
//
//	if (threshold == 0.0)
//	{
//		return partitionsMap;
//	}
//
//	std::vector<size_t> smallPartitions;
//	std::vector<size_t> partitions;
//	std::map<size_t, double> partitionsArea;
//	std::map<size_t, double> partitionsPerimeter;
//	double totalArea = 0;
//	for (auto& pair : partitionsMap)
//	{
//		size_t partitionId = pair.first;
//		auto& partition = pair.second;
//		std::set<halfedge_descriptor> boundaries;
//		double perimeter = 0.0;
//		double area = 0.0;
//		for (auto face : partition)
//		{
//			area += CGAL::Polygon_mesh_processing::face_area(face, mesh);
//			for (auto halfedge : CGAL::halfedges_around_face(mesh.halfedge(face), mesh))
//			{
//				halfedge_descriptor oppositeHalfedge = mesh.opposite(halfedge);
//				face_descriptor face = mesh.face(oppositeHalfedge);
//				if (mesh.is_valid(face))
//				{
//					size_t neighborId = fChartMap[face];
//					if (partitionId != neighborId)
//					{
//						boundaries.insert(halfedge);
//						break;
//					}
//				}
//				else
//				{
//					boundaries.insert(halfedge);
//					break;
//				}
//			}
//		}
//		for (auto halfedge : boundaries)
//		{
//			perimeter += UtilLib::EdgeLength(halfedge, mesh);
//		}
//		partitions.push_back(partitionId);
//		partitionsArea[partitionId] = area;
//		partitionsPerimeter[partitionId] = perimeter;
//		totalArea += area;
//	}
//	
//	std::sort(partitions.begin(), partitions.end(), [&](const int& indexA, const int& indexB) {return partitionsArea.at(indexA) > partitionsArea.at(indexB); });
//	double thresholdArea = threshold * totalArea;
//	double sumArea = 0;
//	for (size_t i = 0; i < partitions.size(); i++)
//	{
//		size_t partitionId = partitions[i];
//		double area = partitionsArea[partitionId];
//		if (area < thresholdArea)
//		{
//			smallPartitions.push_back(partitionId);
//		}
//		sumArea += area;
//	}
//
//	for (size_t partitionId : smallPartitions)
//	{
//		// neighbor partition ID -> appear count
//		std::map<size_t, size_t> neighborCountMap;
//		for (const auto& face : partitionsMap.at(partitionId))
//		{
//			for (const auto& neighbor : CGAL::faces_around_face(mesh.halfedge(face), mesh))
//			{
//				size_t neighborId = fChartMap[neighbor];
//				if (neighborId == partitionId)
//				{
//					continue;
//				}
//				neighborCountMap[neighborId]++;
//			}
//		}
//		size_t maxCountId = size_t(-1);
//		size_t maxCount = 0;
//		for (auto& pair : neighborCountMap)
//		{
//			size_t neighborId = pair.first;
//			size_t count = pair.second;
//			if (count > maxCount)
//			{
//				maxCountId = neighborId;
//				maxCount = count;
//			}
//		}
//		if (maxCountId != size_t(-1))
//		{
//			CGAL::Color newColor = fColorMap[*partitionsMap[maxCountId].begin()];
//			for (auto& face : partitionsMap.at(partitionId))
//			{
//				partitionsMap[maxCountId].insert(face);
//				fChartMap[face] = maxCountId;
//				fColorMap[face] = newColor;
//			}
//			partitionsMap.erase(partitionId);
//			partitionsNormal.erase(partitionId);
//		}
//	}
//	return partitionsMap;
//}

Mesh UtilLib::ConstructWireframeMesh(const std::map<size_t, std::vector<vertex_descriptor>>& boundaryMap, const Mesh& baseMesh)
{
	Mesh wireframeMesh;
	std::map<vertex_descriptor, vertex_descriptor> v2v;
	for (auto& pair : boundaryMap)
	{
		auto& vertices = pair.second;
		for (size_t i = 0; i < vertices.size(); i++)
		{
			const vertex_descriptor& vertex = vertices[i];
			if (v2v.find(vertex) == v2v.end())
			{
				v2v[vertex] = wireframeMesh.add_vertex(baseMesh.point(vertex));
			}
		}
		for (size_t i = 0; i < vertices.size(); i++)
		{
			vertex_descriptor currV = vertices[i];
			vertex_descriptor nextV;
			if (i == vertices.size() - 1)
			{
				nextV = vertices[0];
			}
			else
			{
				nextV = vertices[i + 1];
			}
			wireframeMesh.add_edge(v2v[currV], v2v[nextV]);
		}
	}
	return wireframeMesh;
}

void UtilLib::BuildLocalBasis(const Vector_3& normal, Vector_3& u, Vector_3& v)
{
	Vector_3 temp;
	if (std::abs(normal.x()) <= std::abs(normal.y()) && std::abs(normal.x()) <= std::abs(normal.z()))
		temp = Vector_3(1, 0, 0);
	else if (std::abs(normal.y()) <= std::abs(normal.x()) && std::abs(normal.y()) <= std::abs(normal.z()))
		temp = Vector_3(0, 1, 0);
	else
		temp = Vector_3(0, 0, 1);

	u = CGAL::cross_product(temp, normal);
	u = u / std::sqrt(u.squared_length());
	v = CGAL::cross_product(normal, u);
	v = v / std::sqrt(v.squared_length());
}

void UtilLib::WriteWireframeOBJ(std::string outputFileName, const Mesh& mesh)
{
	std::ofstream os(outputFileName);
	size_t verticesCount = mesh.number_of_vertices();
	size_t edgesCount = mesh.number_of_edges();

	os << "# file written from a CGAL tool in Wavefront obj format" << std::endl;
	os << "# This is a wireframe model" << std::endl;
	os << "# " << verticesCount << " vertices" << std::endl;
	os << "# " << edgesCount << " lines" << std::endl;
	os << std::endl;
	os << std::endl;

	os << "# " << verticesCount << " vertices" << std::endl;
	os << "# ------------------------------------------" << std::endl;
	os << std::endl;
	for (vertex_descriptor vertex : mesh.vertices())
	{
		Point_3 point = mesh.point(vertex);
		os << "v " << point.x() << " " << point.y() << " " << point.z() << std::endl;
	}
	os << std::endl;
	os << "# " << edgesCount << " lines" << std::endl;
	os << "# ------------------------------------------" << std::endl;
	for (edge_descriptor edge : mesh.edges())
	{
		vertex_descriptor v1 = mesh.vertex(edge, 1);
		vertex_descriptor v2 = mesh.vertex(edge, 0);
		os << "l " << v1.idx() + 1 << " " << v2.idx() + 1 << " " << std::endl;
	}
	os << std::endl;
	os << "# End of Wavefront obj format #" << std::endl;
}


double UtilLib::EdgeLength(halfedge_descriptor h, const Mesh& mesh)
{
	vertex_descriptor v1 = mesh.source(h);
	vertex_descriptor v2 = mesh.target(h);

	const Point_3& p1 = mesh.point(v1);
	const Point_3& p2 = mesh.point(v2);

	return std::sqrt(CGAL::squared_distance(p1, p2));
}

void UtilLib::FillHoles(Mesh& mesh)
{
	std::vector<halfedge_descriptor> border_cycles;
	CGAL::Polygon_mesh_processing::extract_boundary_cycles(mesh, std::back_inserter(border_cycles));

	for (halfedge_descriptor h : border_cycles)
	{
		std::vector<face_descriptor>  patch_facets;
		std::vector<vertex_descriptor> patch_vertices;

		CGAL::Polygon_mesh_processing::triangulate_hole(
			mesh,
			h,
			std::back_inserter(patch_facets),
			CGAL::Polygon_mesh_processing::parameters::use_delaunay_triangulation(false));
	}
}

std::vector<std::vector<vertex_descriptor>> UtilLib::ExtractBoundaries(Mesh& mesh, const std::unordered_set<face_descriptor>& faces)
{
	std::vector<std::vector<vertex_descriptor>> boundaries;
	std::unordered_set<halfedge_descriptor> visited;
	for (const auto& f : faces)
	{
		for (const auto& h : CGAL::halfedges_around_face(mesh.halfedge(f), mesh))
		{
			face_descriptor oppF = mesh.face(mesh.opposite(h));
			if (!faces.count(oppF))
			{
				if (visited.count(h)) continue;
				std::vector<vertex_descriptor> boundary;
				halfedge_descriptor start = h;
				halfedge_descriptor curr = h;
				do 
				{
					boundary.push_back(mesh.source(curr));
					visited.insert(curr);
					curr = mesh.next(curr);
					while (faces.count(mesh.face(mesh.opposite(curr))))
					{
						curr = mesh.next(mesh.opposite(curr));
					}
				} while (curr != start);
				boundaries.push_back(boundary);
			}
		}
	}
	return boundaries;
}

std::vector<std::vector<vertex_descriptor>> UtilLib::ExtractCornerBoundaries(Mesh& mesh, const std::unordered_set<face_descriptor>& faces)
{
	auto fChartMap = *mesh.property_map<face_descriptor, int>("f:chart");
	int currentType = fChartMap[*faces.begin()];
	std::vector<std::vector<vertex_descriptor>> boundaries = ExtractBoundaries(mesh, faces);
	std::vector<std::vector<vertex_descriptor>> cornerBoundaries;
	for (const auto& boundary : boundaries)
	{
		std::vector<vertex_descriptor> cornerBoundary;
		for (const auto& vertex : boundary)
		{
			halfedge_descriptor h = mesh.halfedge(vertex);
			std::unordered_set<int> adjacentCharts;
			for (const auto& face : CGAL::faces_around_target(h, mesh))
			{
				if (face != Mesh::null_face())
				{
					adjacentCharts.insert(fChartMap[face]);
				}
				else
				{
					adjacentCharts.insert(-1);
				}
				if (adjacentCharts.size() >= 3)
				{
					cornerBoundary.push_back(vertex);
					break;
				}
			}
		}
		cornerBoundaries.push_back(cornerBoundary);
	}
	return cornerBoundaries;
}

Mesh UtilLib::CreateWireframeMesh(const std::unordered_map<int, std::vector<std::vector<vertex_descriptor>>>& BoundariesMap, const Mesh& mesh)
{
	Mesh wireframeMesh;
	std::unordered_map<vertex_descriptor, vertex_descriptor> v2v;
	for (const auto& pair : BoundariesMap)
	{
		const auto& boundaries = pair.second;
		for (const auto& boundary : boundaries)
		{
			int size = boundary.size();
			for (int i = 0; i < boundary.size(); i++)
			{
				vertex_descriptor v = boundary[i];
				vertex_descriptor nextV = boundary[(i + 1) % size];
				if (!v2v.count(v)) v2v[v] = wireframeMesh.add_vertex(mesh.point(v));
				if (!v2v.count(nextV)) v2v[nextV] = wireframeMesh.add_vertex(mesh.point(nextV));
				wireframeMesh.add_edge(v2v[v], v2v[nextV]);
			}
		}
	}
	return wireframeMesh;
}

bool UtilLib::Intersection(const Mesh& mesh, const Plane_3& plane)
{
	bool has_pos = false;
	bool has_neg = false;
	for (auto p : mesh.points())
	{
		double signed_distance =
			plane.a() * p.x() +
			plane.b() * p.y() +
			plane.c() * p.z() +
			plane.d();
		if (signed_distance > 1e-8)
			has_pos = true;
		else if (signed_distance < -1e-8)
			has_neg = true;

		if (has_pos && has_neg)
			return true;
	}
	return false;
}

