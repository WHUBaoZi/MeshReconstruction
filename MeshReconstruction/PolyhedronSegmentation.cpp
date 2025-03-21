#include "PolyhedronSegmentation.h"


Polyhedron::Polyhedron(Mesh polyhedronMesh, std::vector<PartitionSet*> parentPartitions): polyhedronMesh(polyhedronMesh)
{
	centroidPoint = CGAL::Polygon_mesh_processing::centroid(polyhedronMesh);
	polyhedronTree = Tree(polyhedronMesh.faces().begin(), polyhedronMesh.faces().end(), polyhedronMesh);
	for (auto& partitionSet : parentPartitions)
	{
		for (const auto& point: partitionSet->GetCoveredPoints())
		{
			Ray questRay(point, centroidPoint);
			std::size_t intersections = polyhedronTree.number_of_intersected_primitives(questRay);
			if (intersections % 2 != 0)
			{
				partitions.emplace_back(partitionSet);
				break;
			}
		}
	}
	planeIntersectionNums.assign(partitions.size(), 0);
	for (int i = 0; i < partitions.size(); i++)
	{
		for (int j = i + 1; j < partitions.size(); j++)
		{
			CGAL::Object result = CGAL::intersection(partitions[i]->clipPlane, partitions[j]->clipPlane);
			const Line_3* intersectionLine = CGAL::object_cast<Line_3>(&result);
			if (intersectionLine)
			{
				Point_3 point = intersectionLine->point();
				Direction_3 direction = intersectionLine->direction();
				Ray questRay1(point, direction);
				Ray questRay2(point, -direction);
				if (polyhedronTree.do_intersect(questRay1) || polyhedronTree.do_intersect(questRay2))
				{
					planeIntersectionNums[i]++;
					planeIntersectionNums[j]++;
				} 
			}
		}
	}
}

Mesh Polyhedron::DrawPlanesMesh()
{
	Mesh mesh;
	for (const auto& partition : partitions)
	{
		UtilLib::CreatePlaneMesh(partition->clipPlane, partition->averageCenterPoint, mesh);
	}
	return mesh;
}


PolyhedronSegmentation::PolyhedronSegmentation(PartitionManager* partitionManager, Mesh* mesh) : partitionManager(partitionManager), mesh(mesh)
{
#pragma region Make Cube Mesh
	Point_3 minPoint(UtilLib::INF, UtilLib::INF, UtilLib::INF);
	Point_3 maxPoint(-UtilLib::INF, -UtilLib::INF, -UtilLib::INF);
	for (const auto& vertex : mesh->vertices())
	{
		Point_3 point = mesh->point(vertex);
		minPoint = Point_3(std::min(minPoint.x(), point.x()), std::min(minPoint.y(), point.y()), std::min(minPoint.z(), point.z()));
		maxPoint = Point_3(std::max(maxPoint.x(), point.x()), std::max(maxPoint.y(), point.y()), std::max(maxPoint.z(), point.z()));
	}
	cubeMesh = UtilLib::CreateCube(minPoint, maxPoint);
#pragma endregion
}

Mesh PolyhedronSegmentation::Run(std::string outputPath)
{
	Point_3 centerPoint = UtilLib::GetMeshCenterPoint(cubeMesh);
	{
		std::vector<PartitionSet*> partitions;
		std::vector<PartitionSet*> allPartitions;
		for (auto& pair : partitionManager->partitionSetMap)
		{
			if (pair.second.bIsValid)
			{
				partitions.push_back(&pair.second);
			}
			allPartitions.push_back(&pair.second);
		}
		auto polyhedron = std::make_shared<Polyhedron>(cubeMesh, partitions);
		polyhedrons.push(polyhedron);
		boost::filesystem::create_directories(outputPath + "ClipPlanes");
		CGAL::IO::write_OBJ(outputPath + "ClipPlanes/Original_PolyhedronPlanes.obj", Polyhedron(cubeMesh, allPartitions).DrawPlanesMesh());
		for (const auto& partition : partitions)
		{
			Mesh partitionSetMesh;
			UtilLib::CreatePlaneMesh(partition->clipPlane, partition->averageCenterPoint, partitionSetMesh);
			CGAL::IO::write_OBJ(outputPath + "ClipPlanes/PartitionSet_" + std::to_string(partition->partitionSetIndex) + ".obj", partitionSetMesh);
		}
	}

	int loopNum = 1;
	std::cout << "Clipping..." << std::endl;
	while (!polyhedrons.empty())
	{
		std::cout << "Clipping Num is " << loopNum << std::endl;
		std::shared_ptr<Polyhedron> polyhedron = polyhedrons.front();
		int clipPlaneIndex = polyhedron->GetMinIntersectionIndex();
		Plane_3 clipPlane = polyhedron->partitions[clipPlaneIndex]->clipPlane;
		CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Polyhedron.obj", polyhedron->polyhedronMesh);
		CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_PolyhedronPlanes.obj", polyhedron->DrawPlanesMesh());

		bool bSelfIntersection = CGAL::Polygon_mesh_processing::does_self_intersect(polyhedron->polyhedronMesh);

		Mesh belowMesh = polyhedron->polyhedronMesh;
		Mesh aboveMesh = polyhedron->polyhedronMesh;
		CGAL::Polygon_mesh_processing::clip(belowMesh, clipPlane, CGAL::parameters::clip_volume(true));
		CGAL::Polygon_mesh_processing::clip(aboveMesh, UtilLib::ReversePlane(clipPlane), CGAL::parameters::clip_volume(true));

		//Mesh remeshedBelowMesh, remeshedAboveMesh;
		//CGAL::Polygon_mesh_processing::remesh_planar_patches(belowMesh, remeshedBelowMesh);
		//CGAL::Polygon_mesh_processing::remesh_planar_patches(aboveMesh, remeshedAboveMesh);
		//belowMesh = remeshedBelowMesh;
		//aboveMesh = remeshedAboveMesh;

		CGAL::Surface_mesh_simplification::Edge_length_stop_predicate<double> stop(0.5);
		CGAL::Surface_mesh_simplification::edge_collapse(belowMesh, stop);
		CGAL::Surface_mesh_simplification::edge_collapse(aboveMesh, stop);

		//if (CGAL::Polygon_mesh_processing::does_self_intersect(belowMesh) || 
		//	CGAL::Polygon_mesh_processing::does_self_intersect(aboveMesh))	// 产生自相交，回滚切割
		//{
		//	bool cut_successful = false;
		//	const int max_attempts = 20; // 最大尝试次数
		//	double adjustment_step = 0.01; // 每次调整的步长
		//	double random_tilt = (rand() % 100 - 50) * 0.0001;
		//	clipPlane = Kernel::Plane_3(
		//		clipPlane.a() + random_tilt,
		//		clipPlane.b() + random_tilt,
		//		clipPlane.c(),
		//		clipPlane.d() + adjustment_step);
		//	for (int attempt = 0; attempt < max_attempts; ++attempt) 
		//	{
		//		belowMesh = polyhedron->polyhedronMesh;
		//		aboveMesh = polyhedron->polyhedronMesh;
		//		// 尝试切割
		//		CGAL::Polygon_mesh_processing::clip(belowMesh, clipPlane, CGAL::parameters::clip_volume(true));
		//		CGAL::Polygon_mesh_processing::clip(aboveMesh, UtilLib::ReversePlane(clipPlane), CGAL::parameters::clip_volume(true));
		//		// 检测自相交
		//		if (!CGAL::Polygon_mesh_processing::does_self_intersect(belowMesh) &&
		//			!CGAL::Polygon_mesh_processing::does_self_intersect(aboveMesh)) 
		//		{
		//			cut_successful = true;
		//			break;
		//		}
		//		// 如果自相交，回滚并调整平面
		//		clipPlane = Kernel::Plane_3(
		//			clipPlane.a() + random_tilt,
		//			clipPlane.b() + random_tilt,
		//			clipPlane.c(),
		//			clipPlane.d() + adjustment_step);
		//	}
		//}
		//CGAL::Polygon_mesh_processing::stitch_borders(belowMesh);
		//CGAL::Polygon_mesh_processing::stitch_borders(aboveMesh);

		std::vector<PartitionSet*> parentPartitions = polyhedron->partitions;
		parentPartitions.erase(parentPartitions.begin() + clipPlaneIndex);

		std::map<Mesh::Face_index, std::size_t> face_component_map;
		boost::associative_property_map<std::map<Mesh::Face_index, std::size_t>> fcm(face_component_map);
		if (CGAL::Polygon_mesh_processing::connected_components(belowMesh, fcm) == 1)
		{
			CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Below.obj", belowMesh);
			auto belowPolyhedron = std::make_shared<Polyhedron>(belowMesh, parentPartitions);
			if (!belowPolyhedron->partitions.empty())
			{
				polyhedrons.push(belowPolyhedron);
			}
			else
			{
				indivisiblePolyhedrons.push_back(belowPolyhedron);
			}
		}
		if (CGAL::Polygon_mesh_processing::connected_components(aboveMesh, fcm) == 1)
		{
			CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Above.obj", aboveMesh);
			auto abovePolyhedron = std::make_shared<Polyhedron>(aboveMesh, parentPartitions);
			if (!abovePolyhedron->partitions.empty())
			{
				polyhedrons.push(abovePolyhedron);
			}
			else
			{
				indivisiblePolyhedrons.push_back(abovePolyhedron);
			}
		}
		polyhedrons.pop();
		loopNum++;
	}

	Tree tree(mesh->faces().begin(), mesh->faces().end(), *mesh);
	boost::filesystem::create_directories(outputPath + "IndivisiblePolyhedrons/Useless/");
	boost::filesystem::create_directories(outputPath + "IndivisiblePolyhedrons/Useful/");
	std::vector<int> usefulPolyhedrons;
	for (int i = 0; i < indivisiblePolyhedrons.size(); i++)
	{
		Point_3 centroid = CGAL::Polygon_mesh_processing::centroid(indivisiblePolyhedrons[i]->polyhedronMesh);
		Ray questRay(centroid, Point_3(centroid.x() + 1, centroid.y(), centroid.z()));
		std::size_t intersections = tree.number_of_intersected_primitives(questRay);
		if (intersections % 2 == 0)
		{
			CGAL::IO::write_OBJ(outputPath + "IndivisiblePolyhedrons/Useless/" + std::to_string(i) + "_IndivisiblePolyhedron.obj", indivisiblePolyhedrons[i]->polyhedronMesh);
		}
		else 
		{
			CGAL::IO::write_OBJ(outputPath + "IndivisiblePolyhedrons/Useful/" + std::to_string(i) + "_IndivisiblePolyhedron.obj", indivisiblePolyhedrons[i]->polyhedronMesh);
			usefulPolyhedrons.push_back(i);
		}
	}


	Mesh mergedMesh;
	for (size_t i = 0; i < usefulPolyhedrons.size(); i++)
	{
		int polyhedronsIndex = usefulPolyhedrons[i];
		Mesh& polyhedronMesh = indivisiblePolyhedrons[polyhedronsIndex]->polyhedronMesh;
		std::unordered_map<vertex_descriptor, vertex_descriptor> localVertexMap;
		// 复制顶点
		for (vertex_descriptor v : polyhedronMesh.vertices())
		{
			Point_3 p = polyhedronMesh.point(v);
			vertex_descriptor newV = mergedMesh.add_vertex(p);
			localVertexMap[v] = newV;
		}

		// 复制面
		for (face_descriptor f : polyhedronMesh.faces())
		{
			std::vector<vertex_descriptor> newFace;
			for (vertex_descriptor v : CGAL::vertices_around_face(polyhedronMesh.halfedge(f), polyhedronMesh))
			{
				newFace.push_back(localVertexMap[v]);
			}
			mergedMesh.add_face(newFace);
		}
	}
	CGAL::IO::write_OBJ("MergedMesh.obj", mergedMesh);


#pragma region Merge polyhedrons
	openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(0.1);
	CGALMeshAdapter adapter(&mergedMesh, transform);
	openvdb::util::NullInterrupter interrupter;
	GridType::Ptr sdfGrid = openvdb::tools::meshToVolume<GridType>(
		interrupter,
		adapter,                  // CGALMeshAdapter
		*transform,               // 传递 Transform（解引用 shared_ptr）
		1.0f,                     // exteriorBandWidth：外部窄带宽度（单位：体素）
		std::numeric_limits<float>::max() // interiorBandWidth：最大值，填充内部
	);
	std::vector<openvdb::Vec3s> points;
	std::vector<openvdb::Vec3I> triangles;
	std::vector<openvdb::Vec4I> quads;
	openvdb::tools::volumeToMesh(*sdfGrid, points, triangles, quads);
	std::ofstream objFile("output.obj");
	for (const auto& p : points) {
		objFile << "v " << p[0] << " " << p[1] << " " << p[2] << "\n";
	}
	for (const auto& quad : quads) {
		objFile << "f " << quad[0] + 1 << " " << quad[1] + 1 << " " << quad[2] + 1 << " "<< quad[3] + 1 << "\n";
	}
	objFile.close();
#pragma endregion


    return Mesh();
}
