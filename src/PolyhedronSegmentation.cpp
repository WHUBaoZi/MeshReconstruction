#include "PolyhedronSegmentation.h"

Polyhedron::Polyhedron(Mesh polyhedronMesh, std::vector<std::shared_ptr<Partition>> parentPartitions): polyhedronMesh(polyhedronMesh)
{
	for (auto partition : parentPartitions)
	{
		CGAL::Side_of_triangle_mesh<Mesh, Kernel> insideTester(polyhedronMesh);
		CGAL::Bounded_side boundResult = insideTester(partition->centerPoint);
		if (boundResult == CGAL::ON_UNBOUNDED_SIDE)
		{
			if (CGAL::Polygon_mesh_processing::do_intersect(polyhedronMesh, partition->partitionMesh))
			{
				partitions.push_back(partition);
			}
		}
		else
		{
			partitions.push_back(partition);
		}
	}
}


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
				partitionSets.emplace_back(partitionSet);
				break;
			}
		}
	}
	planeIntersectionNums.assign(partitionSets.size(), 0);
	for (int i = 0; i < partitionSets.size(); i++)
	{
		for (int j = i + 1; j < partitionSets.size(); j++)
		{
			CGAL::Object result = CGAL::intersection(partitionSets[i]->clipPlane, partitionSets[j]->clipPlane);
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
	for (const auto& partition : partitionSets)
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
	cubeCenter = UtilLib::GetMeshCenterPoint(cubeMesh);
	std::vector<std::shared_ptr<Partition>> initialPartitions;
	boost::filesystem::create_directories(outputPath + "ClipPartitions");
	Mesh originalPartitions;
	for (auto& pair : partitionManager->partitionMap)
	{
		auto& partition = pair.second;
		if (partition->bIsValid)
		{
			initialPartitions.push_back(partition);
			CGAL::IO::write_OBJ(outputPath + "ClipPartitions/ClipPartition_" + std::to_string(partition->partitionIndex) + ".obj", UtilLib::CreatePlaneMesh(partition->fitPlane, partition->centerPoint));
		}
		UtilLib::CreatePlaneMesh(partition->fitPlane, partition->centerPoint, originalPartitions);
	}
	CGAL::IO::write_OBJ(outputPath + "ClipPartitions/OriginalClipPartitions.obj", originalPartitions);

	polyhedrons.push(std::shared_ptr<Polyhedron>(new Polyhedron(cubeMesh, initialPartitions)));
	int loopNum = 1;
	std::cout << "Clipping..." << std::endl;
	while (!polyhedrons.empty())
	{
		std::cout << "Clipping Num is " << loopNum << std::endl;
		std::shared_ptr<Polyhedron> polyhedron = polyhedrons.front();
		polyhedrons.pop();
		auto partition = polyhedron->partitions[0];
		Plane_3 clipPlane = partition->fitPlane;
		CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Polyhedron.obj", polyhedron->polyhedronMesh);
		CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_ClipPartition_" + std::to_string(partition->partitionIndex) + ".obj", UtilLib::CreatePlaneMesh(partition->fitPlane, partition->centerPoint));

		bool bSelfIntersection = CGAL::Polygon_mesh_processing::does_self_intersect(polyhedron->polyhedronMesh);	//For Test

		Mesh belowMesh = polyhedron->polyhedronMesh;
		Mesh aboveMesh = polyhedron->polyhedronMesh;
		CGAL::Polygon_mesh_processing::clip(belowMesh, clipPlane, CGAL::parameters::clip_volume(true));
		CGAL::Polygon_mesh_processing::clip(aboveMesh, UtilLib::ReversePlane(clipPlane), CGAL::parameters::clip_volume(true));
		CGAL::Surface_mesh_simplification::Edge_length_stop_predicate<double> stop(0.5);
		CGAL::Surface_mesh_simplification::edge_collapse(belowMesh, stop);
		CGAL::Surface_mesh_simplification::edge_collapse(aboveMesh, stop);

		std::vector<std::shared_ptr<Partition>> parentPartitions = polyhedron->partitions;
		parentPartitions.erase(parentPartitions.begin());

		CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Below.obj", belowMesh);
		auto belowPolyhedron = std::shared_ptr<Polyhedron>(new Polyhedron(belowMesh, parentPartitions));
		if (belowPolyhedron->partitions.empty())
		{
			indivisiblePolyhedrons.push_back(belowPolyhedron);
		}
		else
		{
			polyhedrons.push(belowPolyhedron);
		}
		CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Above.obj", aboveMesh);
		auto abovePolyhedron = std::shared_ptr<Polyhedron>(new Polyhedron(aboveMesh, parentPartitions));
		if (abovePolyhedron->partitions.empty())
		{
			indivisiblePolyhedrons.push_back(abovePolyhedron);
		}
		else
		{
			polyhedrons.push(abovePolyhedron);
		}
		loopNum++;
	}



	Tree tree(mesh->faces().begin(), mesh->faces().end(), *mesh);
	boost::filesystem::create_directories(outputPath + "IndivisiblePolyhedrons/Useless/");
	boost::filesystem::create_directories(outputPath + "IndivisiblePolyhedrons/Useful/");
	std::vector<int> usefulPolyhedrons;
	for (int i = 0; i < indivisiblePolyhedrons.size(); i++)
	{
		Point_3 centroid = CGAL::Polygon_mesh_processing::centroid(indivisiblePolyhedrons[i]->polyhedronMesh);
		// Ray questRay(centroid, Point_3(centroid.x() + 1, centroid.y(), centroid.z()));
		Ray questRay(centroid, cubeCenter);
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
	int flags = openvdb::tools::DISABLE_RENORMALIZATION;
	CGALMeshAdapter adapter(&mergedMesh, transform);
	openvdb::util::NullInterrupter interrupter;
	GridType::Ptr sdfGrid = openvdb::tools::meshToVolume<GridType>(
		interrupter,
		adapter,
		*transform,
		1.0f,
		std::numeric_limits<float>::max()
	);
	std::vector<openvdb::Vec3s> points;
	std::vector<openvdb::Vec3I> triangles;
	std::vector<openvdb::Vec4I> quads;
	openvdb::tools::volumeToMesh(*sdfGrid, points, triangles, quads);
	Mesh voxelMesh, remeshedMesh;
	std::vector<vertex_descriptor> vertices;
	for (const auto& p : points)
	{
		vertices.push_back(voxelMesh.add_vertex(Point_3(p[0], p[1], p[2])));
	}
	for (const auto& quad : quads)
	{
		voxelMesh.add_face(vertices[quad[3]], vertices[quad[2]], vertices[quad[1]], vertices[quad[0]]);
	}
	CGAL::IO::write_OBJ(outputPath + "VoxelMesh.obj", voxelMesh);
	CGAL::Polygon_mesh_processing::triangulate_faces(voxelMesh);
	CGAL::IO::write_OBJ(outputPath + "VoxelMesh_Triangulate.obj", voxelMesh);
#pragma endregion


    return voxelMesh;
}
