#include "PolyhedronSegmentation.h"

Polyhedron::Polyhedron(Mesh polyhedronMesh, std::vector<std::shared_ptr<Partition>> parentPartitions): polyhedronMesh(polyhedronMesh)
{
	for (auto partition : parentPartitions)
	{
		CGAL::Side_of_triangle_mesh<Mesh, Kernel> insideTester(polyhedronMesh);
		CGAL::Bounded_side boundResult = insideTester(partition->centerPoint);
		if (boundResult == CGAL::ON_UNBOUNDED_SIDE)
		{
			if (CGAL::Polygon_mesh_processing::do_intersect(polyhedronMesh, partition->partitionPlaneMesh))
			{
				partitions.push_back(partition);
			}
		}
		else
		{
			partitions.push_back(partition);
		}
	}
	Remesh();
}

void Polyhedron::Remesh()
{
	auto partitionsMap = UtilLib::PartitionByNormal(polyhedronMesh, 0.0, 0.01);
	auto fChartMap = polyhedronMesh.property_map<face_descriptor, size_t>("f:chart").first;
	std::map<size_t, std::vector<vertex_descriptor>> boundaryVerticesMap;
	std::map<size_t, std::vector<vertex_descriptor>> simpBoundaryVerticesMap;
	// partitions traverse
	for (const auto& pair : partitionsMap)
	{
		size_t partitionIndex = pair.first;
		auto& faces = pair.second;
		std::map<vertex_descriptor, halfedge_descriptor> boundaryHalfedges;
		// faces traverse
		for (auto& f : faces)
		{
			// face's halfedge traverse
			for (auto& h : CGAL::halfedges_around_face(polyhedronMesh.halfedge(f), polyhedronMesh))
			{
				halfedge_descriptor oppositeH = polyhedronMesh.opposite(h);
				face_descriptor f1 = polyhedronMesh.face(h);
				face_descriptor f2 = polyhedronMesh.face(oppositeH);
				if (f1 != Mesh::null_face() && f2 != Mesh::null_face())
				{
					if (fChartMap[f1] != fChartMap[f2])
					{
						boundaryHalfedges[polyhedronMesh.source(h)] = h;
					}
				}
			}
		}

		std::vector<vertex_descriptor> boundary;
		std::vector<vertex_descriptor> simpBoundary;
		vertex_descriptor seed = polyhedronMesh.source(boundaryHalfedges.begin()->second);
		vertex_descriptor currV = seed;
		std::set<vertex_descriptor> visited;
		// boundary halfedges traverse. Only for hole-free partition
		do
		{
			if (!visited.insert(currV).second) {
				std::cerr << "[ERROR] Vertex revisited before loop closed. Invalid boundary loop.\n";
				break;
			}
			boundary.push_back(currV);
			currV = polyhedronMesh.target(boundaryHalfedges[currV]);
			// traverse faces around current vertex
			std::set<size_t> charts;
			for (auto& f : CGAL::faces_around_target(polyhedronMesh.halfedge(currV), polyhedronMesh))
			{
				if (f != Mesh::null_face())
				{
					charts.insert(fChartMap[f]);
				}
				else
				{
					std::cerr << "[ERROR] Face does not exist.\n";
				}
			}
			if (charts.size() > 2)
			{
				simpBoundary.push_back(currV);
			}
		} while (currV != seed);
		boundaryVerticesMap[partitionIndex] = boundary;
		simpBoundaryVerticesMap[partitionIndex] = simpBoundary;
	}

	Mesh wireframeMesh = UtilLib::ConstructWirframeMesh(boundaryVerticesMap, polyhedronMesh);
	Mesh simpWireframeMesh = UtilLib::ConstructWirframeMesh(simpBoundaryVerticesMap, polyhedronMesh);
	
	UtilLib::WriteWireframeOBJ(TEST_OUTPUT_PATH + "WireframeMesh.obj", wireframeMesh);
	UtilLib::WriteWireframeOBJ(TEST_OUTPUT_PATH + "SimpWireframeMesh.obj", simpWireframeMesh);

	typedef CGAL::Triple<int, int, int> Triangle_int;

	std::map<vertex_descriptor, vertex_descriptor> v2v;
	Mesh newMesh;
	for (auto& pair : simpBoundaryVerticesMap)
	{
		auto& boundary = pair.second;
		std::vector<Point_3> polyline;
		for (auto& vertex : boundary)
		{
			polyline.push_back(polyhedronMesh.point(vertex));
		}
		std::vector<Triangle_int> patch;
		patch.reserve(polyline.size() - 2);
		CGAL::Polygon_mesh_processing::triangulate_hole_polyline(polyline, std::back_inserter(patch));
		for (size_t i = 0; i < patch.size(); ++i)
		{
			vertex_descriptor v1, v2, v3;
			if (v2v.find(boundary[patch[i].first]) == v2v.end())
			{
				v2v[boundary[patch[i].first]] = newMesh.add_vertex(polyline[patch[i].first]);
			}
			if (v2v.find(boundary[patch[i].second]) == v2v.end())
			{
				v2v[boundary[patch[i].second]] = newMesh.add_vertex(polyline[patch[i].second]);
			}
			if (v2v.find(boundary[patch[i].third]) == v2v.end())
			{
				v2v[boundary[patch[i].third]] = newMesh.add_vertex(polyline[patch[i].third]);
			}
			v1 = v2v[boundary[patch[i].first]];
			v2 = v2v[boundary[patch[i].second]];
			v3 = v2v[boundary[patch[i].third]];
			newMesh.add_face(v1, v2, v3);
		}
	}
	CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "Remesh.obj", newMesh);
	polyhedronMesh = newMesh;
}


Polyhedron::Polyhedron(Mesh polyhedronMesh, std::vector<PartitionSet*> parentPartitions) : polyhedronMesh(polyhedronMesh)
{
	centroidPoint = CGAL::Polygon_mesh_processing::centroid(polyhedronMesh);
	polyhedronTree = Tree(polyhedronMesh.faces().begin(), polyhedronMesh.faces().end(), polyhedronMesh);
	for (auto& partitionSet : parentPartitions)
	{
		for (const auto& point : partitionSet->GetCoveredPoints())
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
	//openvdb::util::NullInterrupter interrupter;
	//openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(1.0);
	//CGALMeshAdapter adapter(&cubeMesh, transform);
	//openvdb::FloatGrid::Ptr grid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
	//	interrupter,
	//	adapter,
	//	*transform,
	//	1.0f,
	//	1.0f
	//);
	//cubeMesh = Voxel::volumeToMesh(grid);
	//CGAL::Polygon_mesh_processing::triangulate_faces(cubeMesh);

#pragma endregion
}

Mesh PolyhedronSegmentation::Run(std::string outputPath)
{
	cubeCenter = UtilLib::GetMeshCenterPoint(cubeMesh);
	std::vector<std::shared_ptr<Partition>> initialPartitions;
	boost::filesystem::create_directories(outputPath + "ClipPartitions");
	boost::filesystem::create_directories(outputPath + "ClipPartitions/OriginalClipPartitions");
	for (auto& pair : partitionManager->partitionMap)
	{
		auto& partition = pair.second;
		Mesh planeMesh = UtilLib::CreatePlaneMesh(partition->fitPlane, partition->centerPoint);
		if (partition->bIsValid)
		{
			initialPartitions.push_back(partition);
			CGAL::IO::write_OBJ(outputPath + "ClipPartitions/ClipPartition_" + std::to_string(partition->partitionIndex) + ".obj", planeMesh);
		}
		CGAL::IO::write_OBJ(outputPath + "ClipPartitions/OriginalClipPartitions/" + std::to_string(partition->partitionIndex) + ".obj", planeMesh);
	}
	

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
		bool bSelfIntersect = CGAL::Polygon_mesh_processing::does_self_intersect(polyhedron->polyhedronMesh);	// For test

		Mesh belowMesh, aboveMesh;
		CGAL::copy_face_graph(polyhedron->polyhedronMesh, belowMesh);
		CGAL::copy_face_graph(polyhedron->polyhedronMesh, aboveMesh);

		bool bClipBelow = CGAL::Polygon_mesh_processing::clip(belowMesh, clipPlane, CGAL::parameters::clip_volume(true));
		bool bClipAbove = CGAL::Polygon_mesh_processing::clip(aboveMesh, UtilLib::ReversePlane(clipPlane), CGAL::parameters::clip_volume(true));
		//CGAL::Surface_mesh_simplification::Edge_length_stop_predicate<double> stop(0.5);
		//CGAL::Surface_mesh_simplification::edge_collapse(belowMesh, stop);
		//CGAL::Surface_mesh_simplification::edge_collapse(aboveMesh, stop);
		std::vector<std::shared_ptr<Partition>> parentPartitions = polyhedron->partitions;
		parentPartitions.erase(parentPartitions.begin());
		auto belowPolyhedron = std::shared_ptr<Polyhedron>(new Polyhedron(belowMesh, parentPartitions));
		CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Below.obj", belowMesh);
		CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Below_Remeshed.obj", belowPolyhedron->polyhedronMesh);
		if (belowPolyhedron->partitions.empty())
		{
			indivisiblePolyhedrons.push_back(belowPolyhedron);
		}
		else
		{
			polyhedrons.push(belowPolyhedron);
		}
		auto abovePolyhedron = std::shared_ptr<Polyhedron>(new Polyhedron(aboveMesh, parentPartitions));
		CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Above.obj", aboveMesh);
		CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Above_Remeshed.obj", abovePolyhedron->polyhedronMesh);
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




	boost::filesystem::create_directories(outputPath + "IndivisiblePolyhedrons/Useless/");
	boost::filesystem::create_directories(outputPath + "IndivisiblePolyhedrons/Useful/");
	openvdb::util::NullInterrupter interrupter;
	openvdb::math::Transform::Ptr transformCheck = openvdb::math::Transform::createLinearTransform(0.1f);
	CGALMeshAdapter adapterMesh(mesh, transformCheck);
	openvdb::FloatGrid::Ptr meshGrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
		interrupter,
		adapterMesh,
		*transformCheck,
		0.1f,
		std::numeric_limits<float>::max()
	);

	Mesh testVolumeMesh = Voxel::volumeToMesh(meshGrid);
	CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "originalVolumeMesh.obj", testVolumeMesh);

	std::vector<int> usefulPolyhedrons;
	for (int i = 0; i < indivisiblePolyhedrons.size(); i++)
	{
		auto polyhedron = indivisiblePolyhedrons[i];
		CGALMeshAdapter adapterPolyhedron(&polyhedron->polyhedronMesh, transformCheck);
		openvdb::FloatGrid::Ptr polyhedronGrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
			interrupter,
			adapterPolyhedron,
			*transformCheck,
			0.1f,
			1.f
		);
		openvdb::FloatGrid::Ptr intersectionGrid = openvdb::tools::csgIntersectionCopy(*polyhedronGrid, *meshGrid);

		CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "PolyhedronMesh.obj", polyhedron->polyhedronMesh);
		Mesh volumePolyhedronMesh = Voxel::volumeToMesh(polyhedronGrid);
		CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "VolumePolyhedronMesh.obj", volumePolyhedronMesh);
		Mesh intersectionPolyhedronMesh = Voxel::volumeToMesh(intersectionGrid);
		CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "IntersectionPolyhedronMesh.obj", intersectionPolyhedronMesh);

		if (!intersectionGrid || intersectionGrid->empty())
		{
			CGAL::IO::write_OBJ(outputPath + "IndivisiblePolyhedrons/Useless/" + std::to_string(i) + "_IndivisiblePolyhedron.obj", indivisiblePolyhedrons[i]->polyhedronMesh);
			continue;
		}

		//openvdb::tools::LevelSetMeasure<openvdb::FloatGrid>
		//	polyhedronMeasure(*polyhedronGrid);
		//openvdb::tools::LevelSetMeasure<openvdb::FloatGrid>
		//	intersectionMeasure(*intersectionGrid);

		//double polyhedronVolume = polyhedronMeasure.volume(true);
		//double intersectionVolume = intersectionMeasure.volume(true);
		size_t polyhedronVoxelCount = Voxel::countInteriorVoxels(polyhedronGrid);
		if (polyhedronVoxelCount == 0)
		{
			std::cout << "Voxel error" << std::endl;
			CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "ErrorPolyhedronMesh_" + std::to_string(i) + ".obj", polyhedron->polyhedronMesh);
		}
		size_t intersectionVoxelCount = Voxel::countInteriorVoxels(intersectionGrid);

		double ratio = static_cast<float>(intersectionVoxelCount) / polyhedronVoxelCount;


		//size_t polyhedronVoxelCount = polyhedronGrid->tree().activeLeafVoxelCount();
		//size_t intersectionVoxelCount = intersectionGrid->tree().activeLeafVoxelCount();
		//float ratio = static_cast<float>(intersectionVoxelCount) / polyhedronVoxelCount;
		if (ratio > 0.7)
		{
			CGAL::IO::write_OBJ(outputPath + "IndivisiblePolyhedrons/Useful/" + std::to_string(i) + "_IndivisiblePolyhedron.obj", indivisiblePolyhedrons[i]->polyhedronMesh);
			usefulPolyhedrons.push_back(i);
		}
		else
		{
			CGAL::IO::write_OBJ(outputPath + "IndivisiblePolyhedrons/Useless/" + std::to_string(i) + "_IndivisiblePolyhedron.obj", indivisiblePolyhedrons[i]->polyhedronMesh);
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
	CGAL::IO::write_OBJ(outputPath + "../MergedMesh.obj", mergedMesh);


#pragma region Merge polyhedrons
	openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(0.1);
	CGALMeshAdapter adapter(&mergedMesh, transform);
	openvdb::FloatGrid::Ptr sdfGrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
		interrupter,
		adapter,
		*transform,
		1.0f,
		std::numeric_limits<float>::max()
	);
	Mesh voxelMesh = Voxel::volumeToMesh(sdfGrid);
	size_t removed = CGAL::Polygon_mesh_processing::keep_largest_connected_components(voxelMesh, 1);
	CGAL::IO::write_OBJ(outputPath + "../VoxelMesh.obj", voxelMesh);
	CGAL::Polygon_mesh_processing::triangulate_faces(voxelMesh);
	CGAL::IO::write_OBJ(outputPath + "../VoxelMesh_Triangulate.obj", voxelMesh);
#pragma endregion


    return voxelMesh;
}
