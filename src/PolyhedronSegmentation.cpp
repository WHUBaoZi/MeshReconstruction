#include "PolyhedronSegmentation.h"

#include "ExtendedMarchingCube.h"
#include "Remesh.h"

#include "AlgoDebugIO.h"

#include <windows.h>

//Polyhedron::Polyhedron(Mesh polyhedronMesh, std::vector<std::shared_ptr<Partition>> parentPartitions): polyhedronMesh(polyhedronMesh)
//{
//	for (auto partition : parentPartitions)
//	{
//		CGAL::Side_of_triangle_mesh<Mesh, Kernel> insideTester(polyhedronMesh);
//		CGAL::Bounded_side boundResult = insideTester(partition->centerPoint);
//		if (boundResult == CGAL::ON_UNBOUNDED_SIDE)
//		{
//			if (CGAL::Polygon_mesh_processing::do_intersect(polyhedronMesh, partition->partitionPlaneMesh))
//			{
//				partitions.push_back(partition);
//			}
//		}
//		else
//		{
//			partitions.push_back(partition);
//		}
//	}
//	Remesh();
//}
//PolyhedronSegmentation::PolyhedronSegmentation(PartitionManager* partitionManager, Mesh* mesh) : partitionManager(partitionManager), mesh(mesh)
//{
//#pragma region Make Cube Mesh
//	Point_3 minPoint(UtilLib::INF, UtilLib::INF, UtilLib::INF);
//	Point_3 maxPoint(-UtilLib::INF, -UtilLib::INF, -UtilLib::INF);
//	for (const auto& vertex : mesh->vertices())
//	{
//		Point_3 point = mesh->point(vertex);
//		minPoint = Point_3(std::min(minPoint.x(), point.x()), std::min(minPoint.y(), point.y()), std::min(minPoint.z(), point.z()));
//		maxPoint = Point_3(std::max(maxPoint.x(), point.x()), std::max(maxPoint.y(), point.y()), std::max(maxPoint.z(), point.z()));
//	}
//	cubeMesh = UtilLib::CreateCube(minPoint, maxPoint);
//	//openvdb::util::NullInterrupter interrupter;
//	//openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(1.0);
//	//CGALMeshAdapter adapter(&cubeMesh, transform);
//	//openvdb::FloatGrid::Ptr grid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
//	//	interrupter,
//	//	adapter,
//	//	*transform,
//	//	1.0f,
//	//	1.0f
//	//);
//	//cubeMesh = Voxel::volumeToMesh(grid);
//	//CGAL::Polygon_mesh_processing::triangulate_faces(cubeMesh);
//
//#pragma endregion
//}
//Mesh PolyhedronSegmentation::Run(std::string outputPath)
//{
//	cubeCenter = UtilLib::GetMeshCenterPoint(cubeMesh);
//	std::vector<std::shared_ptr<Partition>> initialPartitions;
//	boost::filesystem::create_directories(outputPath + "ClipPartitions");
//	boost::filesystem::create_directories(outputPath + "ClipPartitions/OriginalClipPartitions");
//	for (auto& pair : partitionManager->partitionMap)
//	{
//		auto& partition = pair.second;
//		Mesh planeMesh = UtilLib::CreatePlaneMesh(partition->fitPlane, partition->centerPoint);
//		if (partition->bIsValid)
//		{
//			initialPartitions.push_back(partition);
//			CGAL::IO::write_OBJ(outputPath + "ClipPartitions/ClipPartition_" + std::to_string(partition->partitionIndex) + ".obj", planeMesh);
//		}
//		CGAL::IO::write_OBJ(outputPath + "ClipPartitions/OriginalClipPartitions/" + std::to_string(partition->partitionIndex) + ".obj", planeMesh);
//	}
//	
//
//	polyhedrons.push(std::shared_ptr<Polyhedron>(new Polyhedron(cubeMesh, initialPartitions)));
//	int loopNum = 1;
//	std::cout << "Clipping..." << std::endl;
//
//	while (!polyhedrons.empty())
//	{
//		std::cout << "Clipping Num is " << loopNum << std::endl;
//		std::shared_ptr<Polyhedron> polyhedron = polyhedrons.front();
//		polyhedrons.pop();
//		auto partition = polyhedron->partitions[0];
//		Plane_3 clipPlane = partition->fitPlane;
//		CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Polyhedron.obj", polyhedron->polyhedronMesh);
//		CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_ClipPartition_" + std::to_string(partition->partitionIndex) + ".obj", UtilLib::CreatePlaneMesh(partition->fitPlane, partition->centerPoint));
//		bool bSelfIntersect = CGAL::Polygon_mesh_processing::does_self_intersect(polyhedron->polyhedronMesh);	// For test
//
//		Mesh belowMesh, aboveMesh;
//		CGAL::copy_face_graph(polyhedron->polyhedronMesh, belowMesh);
//		CGAL::copy_face_graph(polyhedron->polyhedronMesh, aboveMesh);
//
//		bool bClipBelow = CGAL::Polygon_mesh_processing::clip(belowMesh, clipPlane, CGAL::parameters::clip_volume(true));
//		bool bClipAbove = CGAL::Polygon_mesh_processing::clip(aboveMesh, UtilLib::ReversePlane(clipPlane), CGAL::parameters::clip_volume(true));
//		//CGAL::Surface_mesh_simplification::Edge_length_stop_predicate<double> stop(0.5);
//		//CGAL::Surface_mesh_simplification::edge_collapse(belowMesh, stop);
//		//CGAL::Surface_mesh_simplification::edge_collapse(aboveMesh, stop);
//		std::vector<std::shared_ptr<Partition>> parentPartitions = polyhedron->partitions;
//		parentPartitions.erase(parentPartitions.begin());
//		auto belowPolyhedron = std::shared_ptr<Polyhedron>(new Polyhedron(belowMesh, parentPartitions));
//		CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Below.obj", belowMesh);
//		CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Below_Remeshed.obj", belowPolyhedron->polyhedronMesh);
//		if (belowPolyhedron->partitions.empty())
//		{
//			indivisiblePolyhedrons.push_back(belowPolyhedron);
//		}
//		else
//		{
//			polyhedrons.push(belowPolyhedron);
//		}
//		auto abovePolyhedron = std::shared_ptr<Polyhedron>(new Polyhedron(aboveMesh, parentPartitions));
//		CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Above.obj", aboveMesh);
//		CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Above_Remeshed.obj", abovePolyhedron->polyhedronMesh);
//		if (abovePolyhedron->partitions.empty())
//		{
//			indivisiblePolyhedrons.push_back(abovePolyhedron);
//		}
//		else
//		{
//			polyhedrons.push(abovePolyhedron);
//		}
//		loopNum++;
//	}
//
//
//
//
//	boost::filesystem::create_directories(outputPath + "IndivisiblePolyhedrons/Useless/");
//	boost::filesystem::create_directories(outputPath + "IndivisiblePolyhedrons/Useful/");
//	openvdb::util::NullInterrupter interrupter;
//	openvdb::math::Transform::Ptr transformCheck = openvdb::math::Transform::createLinearTransform(0.1f);
//	CGALMeshAdapter adapterMesh(mesh, transformCheck);
//	openvdb::FloatGrid::Ptr meshGrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
//		interrupter,
//		adapterMesh,
//		*transformCheck,
//		0.1f,
//		std::numeric_limits<float>::max()
//	);
//
//	Mesh testVolumeMesh = Voxel::volumeToMesh(meshGrid);
//	CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "originalVolumeMesh.obj", testVolumeMesh);
//
//	std::vector<int> usefulPolyhedrons;
//	for (int i = 0; i < indivisiblePolyhedrons.size(); i++)
//	{
//		auto polyhedron = indivisiblePolyhedrons[i];
//		CGALMeshAdapter adapterPolyhedron(&polyhedron->polyhedronMesh, transformCheck);
//		openvdb::FloatGrid::Ptr polyhedronGrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
//			interrupter,
//			adapterPolyhedron,
//			*transformCheck,
//			0.1f,
//			1.f
//		);
//		openvdb::FloatGrid::Ptr intersectionGrid = openvdb::tools::csgIntersectionCopy(*polyhedronGrid, *meshGrid);
//
//		CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "PolyhedronMesh.obj", polyhedron->polyhedronMesh);
//		Mesh volumePolyhedronMesh = Voxel::volumeToMesh(polyhedronGrid);
//		CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "VolumePolyhedronMesh.obj", volumePolyhedronMesh);
//		Mesh intersectionPolyhedronMesh = Voxel::volumeToMesh(intersectionGrid);
//		CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "IntersectionPolyhedronMesh.obj", intersectionPolyhedronMesh);
//
//		if (!intersectionGrid || intersectionGrid->empty())
//		{
//			CGAL::IO::write_OBJ(outputPath + "IndivisiblePolyhedrons/Useless/" + std::to_string(i) + "_IndivisiblePolyhedron.obj", indivisiblePolyhedrons[i]->polyhedronMesh);
//			continue;
//		}
//
//		//openvdb::tools::LevelSetMeasure<openvdb::FloatGrid>
//		//	polyhedronMeasure(*polyhedronGrid);
//		//openvdb::tools::LevelSetMeasure<openvdb::FloatGrid>
//		//	intersectionMeasure(*intersectionGrid);
//
//		//double polyhedronVolume = polyhedronMeasure.volume(true);
//		//double intersectionVolume = intersectionMeasure.volume(true);
//		size_t polyhedronVoxelCount = Voxel::countInteriorVoxels(polyhedronGrid);
//		if (polyhedronVoxelCount == 0)
//		{
//			std::cout << "Voxel error" << std::endl;
//			CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "ErrorPolyhedronMesh_" + std::to_string(i) + ".obj", polyhedron->polyhedronMesh);
//		}
//		size_t intersectionVoxelCount = Voxel::countInteriorVoxels(intersectionGrid);
//
//		double ratio = static_cast<float>(intersectionVoxelCount) / polyhedronVoxelCount;
//
//
//		//size_t polyhedronVoxelCount = polyhedronGrid->tree().activeLeafVoxelCount();
//		//size_t intersectionVoxelCount = intersectionGrid->tree().activeLeafVoxelCount();
//		//float ratio = static_cast<float>(intersectionVoxelCount) / polyhedronVoxelCount;
//		if (ratio > 0.7)
//		{
//			CGAL::IO::write_OBJ(outputPath + "IndivisiblePolyhedrons/Useful/" + std::to_string(i) + "_IndivisiblePolyhedron.obj", indivisiblePolyhedrons[i]->polyhedronMesh);
//			usefulPolyhedrons.push_back(i);
//		}
//		else
//		{
//			CGAL::IO::write_OBJ(outputPath + "IndivisiblePolyhedrons/Useless/" + std::to_string(i) + "_IndivisiblePolyhedron.obj", indivisiblePolyhedrons[i]->polyhedronMesh);
//		}
//	}
//
//
//	Mesh mergedMesh;
//	for (size_t i = 0; i < usefulPolyhedrons.size(); i++)
//	{
//		int polyhedronsIndex = usefulPolyhedrons[i];
//		Mesh& polyhedronMesh = indivisiblePolyhedrons[polyhedronsIndex]->polyhedronMesh;
//		std::unordered_map<vertex_descriptor, vertex_descriptor> localVertexMap;
//		// 复制顶点
//		for (vertex_descriptor v : polyhedronMesh.vertices())
//		{
//			Point_3 p = polyhedronMesh.point(v);
//			vertex_descriptor newV = mergedMesh.add_vertex(p);
//			localVertexMap[v] = newV;
//		}
//
//		// 复制面
//		for (face_descriptor f : polyhedronMesh.faces())
//		{
//			std::vector<vertex_descriptor> newFace;
//			for (vertex_descriptor v : CGAL::vertices_around_face(polyhedronMesh.halfedge(f), polyhedronMesh))
//			{
//				newFace.push_back(localVertexMap[v]);
//			}
//			mergedMesh.add_face(newFace);
//		}
//	}
//	CGAL::IO::write_OBJ(outputPath + "../MergedMesh.obj", mergedMesh);
//
//
//#pragma region Merge polyhedrons
//	Mesh voxelMesh, fixedMesh;
//	if (false)
//	{
//		openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(0.1);
//		CGALMeshAdapter adapter(&mergedMesh, transform);
//		openvdb::FloatGrid::Ptr sdfGrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
//			interrupter,
//			adapter,
//			*transform,
//			1.0f,
//			std::numeric_limits<float>::max()
//		);
//		Mesh voxelMesh = Voxel::volumeToMesh(sdfGrid);
//		size_t removed = CGAL::Polygon_mesh_processing::keep_largest_connected_components(voxelMesh, 1);
//		CGAL::IO::write_OBJ(outputPath + "../VoxelMesh.obj", voxelMesh);
//		CGAL::Polygon_mesh_processing::triangulate_faces(voxelMesh);
//		CGAL::IO::write_OBJ(outputPath + "../VoxelMesh_Triangulate.obj", voxelMesh);
//	}
//	else
//	{
//
//
//		openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(0.3);
//		CGALMeshAdapter adapter(&mergedMesh, transform);
//		openvdb::FloatGrid::Ptr sdfGrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
//			interrupter,
//			adapter,
//			*transform,
//			3.f,
//			3.f
//		);
//		voxelMesh = ExtendedMarchingCube::ApplyExtendedMarchingCube(sdfGrid);
//
//		STARTUPINFO si = { 0 };
//		si.cb = sizeof(si);
//		PROCESS_INFORMATION pi;
//
//		std::string inputMeshPath = outputPath + "../ExtendedMarchingCubeMesh.off";
//		std::string outputMeshPath = outputPath + "../ExtendedMarchingCubeMesh_fixed.off";
//		std::string cmd = "MeshFix.exe \"" + inputMeshPath + "\" \"" + outputMeshPath + "\"";
//		CGAL::IO::write_OFF(inputMeshPath, voxelMesh);
//
//		
//		char cmdLine[1024];
//		strcpy_s(cmdLine, cmd.c_str());
//		if (CreateProcess(
//			NULL,        // lpApplicationName 可以为 NULL
//			cmdLine,     // lpCommandLine
//			NULL, NULL, FALSE, 0, NULL, NULL,
//			&si, &pi))
//		{
//			WaitForSingleObject(pi.hProcess, INFINITE);
//			CloseHandle(pi.hProcess);
//			CloseHandle(pi.hThread);
//			std::cout << "MeshFix executed successfully" << std::endl;
//		}
//		else
//		{
//			DWORD error = GetLastError();
//			std::cerr << "MeshFix activate fail, error code: " << error << std::endl;
//		}
//		CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(outputMeshPath, fixedMesh);
//	}
//#pragma endregion
//
//
//    return fixedMesh;
//}

Mesh PolyhedronSegmentationFunctions::DoSegmentation(const Mesh& mesh, const std::unordered_map<int, std::unordered_set<face_descriptor>>& partitions)
{
	// Make Cube Mesh
	#pragma region Make Cube Mesh
	Point_3 minPoint(UtilLib::INF, UtilLib::INF, UtilLib::INF);
	Point_3 maxPoint(-UtilLib::INF, -UtilLib::INF, -UtilLib::INF);
	for (const auto& vertex : mesh.vertices())
	{
		Point_3 point = mesh.point(vertex);
		minPoint = Point_3(std::min(minPoint.x(), point.x()), std::min(minPoint.y(), point.y()), std::min(minPoint.z(), point.z()));
		maxPoint = Point_3(std::max(maxPoint.x(), point.x()), std::max(maxPoint.y(), point.y()), std::max(maxPoint.z(), point.z()));
	}
	Mesh cubeMesh = UtilLib::CreateCube(minPoint, maxPoint);
	#pragma endregion

	// Create Dividing Surfaces
	#pragma region Create Dividing Surfaces
	SegmentationManager seg(mesh, partitions);
	for (const auto& pair : partitions)
	{
		int partitionID = seg.CreateDividingSurface(pair);
		const auto& ds = seg.dividingSurfaces[partitionID];

		Mesh planeMesh = UtilLib::CreatePlaneMesh(ds.plane, ds.centerPoint);
		//CGAL::IO::write_OBJ(outputPath + "ClipPartitions/ClipPartition_" + std::to_string(partitionID) + ".obj", planeMesh);
	}
	#pragma endregion

	// Segmentation
	#pragma region Segmentation
	std::unordered_set<int> initialDividingSurfaces;
	for (const auto pair : seg.dividingSurfaces)
	{
		initialDividingSurfaces.insert(pair.first);
	}

	int initialPolyhedron = seg.CreatePolyhedron(cubeMesh, initialDividingSurfaces);
	std::queue<int> polyhedrons;
	polyhedrons.push(initialPolyhedron);
	std::vector<int> indivisiblePolyhedrons;
	int loopNum = 1;
	while (!polyhedrons.empty())
	{
		std::cout << "Clipping Num is " << loopNum << std::endl;
		int polyhedronID = polyhedrons.front();
		polyhedrons.pop();
		const PolyhedronFromPartition& polyhedron = seg.polyhedrons.at(polyhedronID);
		int dividingSurfaceID = *polyhedron.dividingSurfaces.begin();
		const DividingSurface& dividingSurface = seg.dividingSurfaces[dividingSurfaceID];
		const Plane_3& dividingPlane = dividingSurface.plane;
		//CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Polyhedron.obj", polyhedron.polyhedronMesh);
		//CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_ClipPartition_" + std::to_string(dividingSurface.partitionID) + ".obj", UtilLib::CreatePlaneMesh(dividingSurface.plane, dividingSurface.centerPoint));
		bool bSelfIntersect = CGAL::Polygon_mesh_processing::does_self_intersect(polyhedron.polyhedronMesh);	// Check intersect
		Mesh belowMesh, aboveMesh;
		CGAL::copy_face_graph(polyhedron.polyhedronMesh, belowMesh);
		CGAL::copy_face_graph(polyhedron.polyhedronMesh, aboveMesh);
		bool bClipBelow = CGAL::Polygon_mesh_processing::clip(belowMesh, dividingPlane, CGAL::parameters::clip_volume(true));
		bool bClipAbove = CGAL::Polygon_mesh_processing::clip(aboveMesh, UtilLib::ReversePlane(dividingPlane), CGAL::parameters::clip_volume(true));
		std::unordered_set<int> parentDividingSurfaces = polyhedron.dividingSurfaces;
		parentDividingSurfaces.erase(dividingSurfaceID);
		int belowPolyhedronID = seg.CreatePolyhedron(belowMesh, parentDividingSurfaces);
		const PolyhedronFromPartition& belowPolyhedron = seg.polyhedrons[belowPolyhedronID];
		//CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Below.obj", belowMesh);
		//CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Below_Remeshed.obj", belowPolyhedron.polyhedronMesh);
		if (belowPolyhedron.dividingSurfaces.empty())
		{
			indivisiblePolyhedrons.push_back(belowPolyhedronID);
		}
		else
		{
			polyhedrons.push(belowPolyhedronID);
		}
		int abovePolyhedronID = seg.CreatePolyhedron(aboveMesh, parentDividingSurfaces);
		const PolyhedronFromPartition& abovePolyhedron = seg.polyhedrons[abovePolyhedronID];
		//CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Above.obj", aboveMesh);
		//CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Above_Remeshed.obj", abovePolyhedron.polyhedronMesh);
		if (abovePolyhedron.dividingSurfaces.empty())
		{
			indivisiblePolyhedrons.push_back(abovePolyhedronID);
		}
		else
		{
			polyhedrons.push(abovePolyhedronID);
		}
		loopNum++;
	}
	#pragma endregion


	openvdb::util::NullInterrupter interrupter;
	openvdb::math::Transform::Ptr transformCheck = openvdb::math::Transform::createLinearTransform(0.1f);
	CGALMeshAdapter adapterMesh(&seg.mesh, transformCheck);
	openvdb::FloatGrid::Ptr meshGrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
		interrupter,
		adapterMesh,
		*transformCheck,
		0.1f,
		std::numeric_limits<float>::max()
	);

	Mesh testVolumeMesh = Voxel::volumeToMesh(meshGrid);
	//CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "originalVolumeMesh.obj", testVolumeMesh);

	std::vector<int> usefulPolyhedrons;
	for (int i = 0; i < indivisiblePolyhedrons.size(); i++)
	{
		int polyhedronID = indivisiblePolyhedrons[i];
		PolyhedronFromPartition& polyhedron = seg.polyhedrons[polyhedronID];
		CGALMeshAdapter adapterPolyhedron(&polyhedron.polyhedronMesh, transformCheck);
		openvdb::FloatGrid::Ptr polyhedronGrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
			interrupter,
			adapterPolyhedron,
			*transformCheck,
			1.f,
			1.f
		);
		openvdb::FloatGrid::Ptr intersectionGrid = openvdb::tools::csgIntersectionCopy(*polyhedronGrid, *meshGrid);

		//CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "PolyhedronMesh.obj", polyhedron.polyhedronMesh);
		Mesh volumePolyhedronMesh = Voxel::volumeToMesh(polyhedronGrid);
		//CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "VolumePolyhedronMesh.obj", volumePolyhedronMesh);
		Mesh intersectionPolyhedronMesh = Voxel::volumeToMesh(intersectionGrid);
		//CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "IntersectionPolyhedronMesh.obj", intersectionPolyhedronMesh);

		if (!intersectionGrid || intersectionGrid->empty())
		{
			//CGAL::IO::write_OBJ(outputPath + "IndivisiblePolyhedrons/Useless/" + std::to_string(i) + "_IndivisiblePolyhedron.obj", polyhedron.polyhedronMesh);
			continue;
		}
		size_t polyhedronVoxelCount = Voxel::countInteriorVoxels(polyhedronGrid);
		if (polyhedronVoxelCount == 0)
		{
			std::cout << "Voxel error" << std::endl;
			//CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "ErrorPolyhedronMesh_" + std::to_string(i) + ".obj", polyhedron.polyhedronMesh);
		}
		size_t intersectionVoxelCount = Voxel::countInteriorVoxels(intersectionGrid);

		double ratio = static_cast<float>(intersectionVoxelCount) / polyhedronVoxelCount;
		if (ratio > 0.7)
		{
			//CGAL::IO::write_OBJ(outputPath + "IndivisiblePolyhedrons/Useful/" + std::to_string(i) + "_IndivisiblePolyhedron.obj", polyhedron.polyhedronMesh);
			usefulPolyhedrons.push_back(polyhedronID);
		}
		else
		{
			//CGAL::IO::write_OBJ(outputPath + "IndivisiblePolyhedrons/Useless/" + std::to_string(i) + "_IndivisiblePolyhedron.obj", polyhedron.polyhedronMesh);
		}
	}


	Mesh mergedMesh;
	for (size_t i = 0; i < usefulPolyhedrons.size(); i++)
	{
		int polyhedronsID = usefulPolyhedrons[i];
		PolyhedronFromPartition& polyhedron = seg.polyhedrons[polyhedronsID];
		Mesh& polyhedronMesh = polyhedron.polyhedronMesh;
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
	//CGAL::IO::write_OBJ(outputPath + "../MergedMesh.obj", mergedMesh);

	// Merge polyhedrons
	#pragma region Merge polyhedrons
	Mesh voxelMesh, fixedMesh;
	if (false)
	{
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
		//CGAL::IO::write_OBJ(outputPath + "../VoxelMesh.obj", voxelMesh);
		CGAL::Polygon_mesh_processing::triangulate_faces(voxelMesh);
		//CGAL::IO::write_OBJ(outputPath + "../VoxelMesh_Triangulate.obj", voxelMesh);
	}
	else
	{
		//openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(0.3);
		//CGALMeshAdapter adapter(&mergedMesh, transform);
		//openvdb::FloatGrid::Ptr sdfGrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
		//	interrupter,
		//	adapter,
		//	*transform,
		//	3.f,
		//	3.f
		//);
		//voxelMesh = ExtendedMarchingCube::ApplyExtendedMarchingCube(sdfGrid);

		//STARTUPINFO si = { 0 };
		//si.cb = sizeof(si);
		//PROCESS_INFORMATION pi;

		//std::string inputMeshPath = outputPath + "../ExtendedMarchingCubeMesh.off";
		//std::string outputMeshPath = outputPath + "../ExtendedMarchingCubeMesh_fixed.off";
		//std::string cmd = "MeshFix.exe \"" + inputMeshPath + "\" \"" + outputMeshPath + "\"";
		////CGAL::IO::write_OFF(inputMeshPath, voxelMesh);


		//char cmdLine[1024];
		//strcpy_s(cmdLine, cmd.c_str());
		//if (CreateProcess(
		//	NULL,        // lpApplicationName 可以为 NULL
		//	cmdLine,     // lpCommandLine
		//	NULL, NULL, FALSE, 0, NULL, NULL,
		//	&si, &pi))
		//{
		//	WaitForSingleObject(pi.hProcess, INFINITE);
		//	CloseHandle(pi.hProcess);
		//	CloseHandle(pi.hThread);
		//	std::cout << "MeshFix executed successfully" << std::endl;
		//}
		//else
		//{
		//	DWORD error = GetLastError();
		//	std::cerr << "MeshFix activate fail, error code: " << error << std::endl;
		//}
		//CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(outputMeshPath, fixedMesh);
	}
	#pragma endregion

	return fixedMesh;
}

Mesh PolyhedronSegmentationFunctions::DoSegmentation(const Mesh& mesh, const std::vector<RansacPlane>& planes, const std::vector<std::vector<Point_3>>& planePoints)
{
	// Make Cube Mesh
#pragma region Make Cube Mesh
	Point_3 minPoint(UtilLib::INF, UtilLib::INF, UtilLib::INF);
	Point_3 maxPoint(-UtilLib::INF, -UtilLib::INF, -UtilLib::INF);
	for (const auto& vertex : mesh.vertices())
	{
		Point_3 point = mesh.point(vertex);
		minPoint = Point_3(std::min(minPoint.x(), point.x()), std::min(minPoint.y(), point.y()), std::min(minPoint.z(), point.z()));
		maxPoint = Point_3(std::max(maxPoint.x(), point.x()), std::max(maxPoint.y(), point.y()), std::max(maxPoint.z(), point.z()));
	}
	Mesh cubeMesh = UtilLib::CreateCube(minPoint, maxPoint);
#pragma endregion

	std::vector<PolyhedronFromPlane> polyhedrons;
	std::vector<int> indivisiblePolyhedrons;
	std::vector<int> initialPlanes(planes.size());
	std::iota(initialPlanes.begin(), initialPlanes.end(), 0);

	polyhedrons.emplace_back(PolyhedronFromPlane(cubeMesh, initialPlanes, planes, planePoints));
	std::queue<int> polyhedronsQueue;
	polyhedronsQueue.push(0);
	while (!polyhedronsQueue.empty())
	{
		int index = polyhedronsQueue.front();
		polyhedronsQueue.pop();
		auto& polyhedron = polyhedrons[index];
		int planeIndex = polyhedron.planeIndices[0];
		const auto& plane = planes[planeIndex];

#ifdef ENABLE_ALGO_DEBUG
		CGAL::IO::write_OBJ(GAlgoDebugOutputDir + "SegmentationResults/Progress/" + std::to_string(index) + "_Polyhedron.obj", polyhedron.polyhedronMesh);
		CGAL::IO::write_OBJ(GAlgoDebugOutputDir + "SegmentationResults/Progress/" + std::to_string(index) + "_ClipPartition_" + std::to_string(planeIndex) + ".obj", UtilLib::CreatePlaneMesh(plane, CGAL::Polygon_mesh_processing::centroid(polyhedron.polyhedronMesh)));
#endif // ENABLE_ALGO_DEBUG

		bool bSelfIntersect = CGAL::Polygon_mesh_processing::does_self_intersect(polyhedron.polyhedronMesh);	// Check intersect
		Mesh belowMesh, aboveMesh;
		CGAL::copy_face_graph(polyhedron.polyhedronMesh, belowMesh);
		CGAL::copy_face_graph(polyhedron.polyhedronMesh, aboveMesh);
		bool bClipBelow = CGAL::Polygon_mesh_processing::clip(belowMesh, plane, CGAL::parameters::clip_volume(true));
		bool bClipAbove = CGAL::Polygon_mesh_processing::clip(aboveMesh, UtilLib::ReversePlane(plane), CGAL::parameters::clip_volume(true));
		std::vector<int> planeIndices = polyhedron.planeIndices;
		planeIndices.erase(planeIndices.begin());
		polyhedrons.emplace_back(PolyhedronFromPlane(belowMesh, planeIndices, planes, planePoints));
		polyhedrons.emplace_back(PolyhedronFromPlane(aboveMesh, planeIndices, planes, planePoints));
		int aboveIndex = polyhedrons.size() - 1;
		int belowIndex = aboveIndex - 1;
		auto& belowPolyhedron = polyhedrons[belowIndex];
		auto& abovePolyhedron = polyhedrons[aboveIndex];
		if (belowPolyhedron.planeIndices.empty()) indivisiblePolyhedrons.push_back(belowIndex);
		else polyhedronsQueue.push(belowIndex);
		if (abovePolyhedron.planeIndices.empty()) indivisiblePolyhedrons.push_back(aboveIndex);
		else polyhedronsQueue.push(aboveIndex);
	}

	openvdb::util::NullInterrupter interrupter;
	openvdb::math::Transform::Ptr transformCheck = openvdb::math::Transform::createLinearTransform(0.1f);
	CGALMeshAdapter adapterMesh(&mesh, transformCheck);
	openvdb::FloatGrid::Ptr meshGrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
		interrupter,
		adapterMesh,
		*transformCheck,
		0.1f,
		std::numeric_limits<float>::max()
	);

	Mesh testVolumeMesh = Voxel::volumeToMesh(meshGrid);
	//CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "originalVolumeMesh.obj", testVolumeMesh);

	std::vector<int> usefulPolyhedrons;
	for (int i = 0; i < indivisiblePolyhedrons.size(); i++)
	{
		int index = indivisiblePolyhedrons[i];
		auto& polyhedron = polyhedrons[index];
		CGALMeshAdapter adapterPolyhedron(&polyhedron.polyhedronMesh, transformCheck);
		openvdb::FloatGrid::Ptr polyhedronGrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
			interrupter,
			adapterPolyhedron,
			*transformCheck,
			1.f,
			1.f
		);
		openvdb::FloatGrid::Ptr intersectionGrid = openvdb::tools::csgIntersectionCopy(*polyhedronGrid, *meshGrid);

		//CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "PolyhedronMesh.obj", polyhedron.polyhedronMesh);
		Mesh volumePolyhedronMesh = Voxel::volumeToMesh(polyhedronGrid);
		//CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "VolumePolyhedronMesh.obj", volumePolyhedronMesh);
		Mesh intersectionPolyhedronMesh = Voxel::volumeToMesh(intersectionGrid);
		//::IO::write_OBJ(TEST_OUTPUT_PATH + "IntersectionPolyhedronMesh.obj", intersectionPolyhedronMesh);

		if (!intersectionGrid || intersectionGrid->empty())
		{
#ifdef ENABLE_ALGO_DEBUG
			CGAL::IO::write_OBJ(GAlgoDebugOutputDir + "SegmentationResults/IndivisiblePolyhedrons/Useless/" + std::to_string(i) + "_IndivisiblePolyhedron.obj", polyhedron.polyhedronMesh);
#endif // ENABLE_ALGO_DEBUG

			continue;
		}
		size_t polyhedronVoxelCount = Voxel::countInteriorVoxels(polyhedronGrid);
		if (polyhedronVoxelCount == 0)
		{
			std::cout << "Voxel error" << std::endl;
			//CGAL::IO::write_OBJ(TEST_OUTPUT_PATH + "ErrorPolyhedronMesh_" + std::to_string(i) + ".obj", polyhedron.polyhedronMesh);
		}
		size_t intersectionVoxelCount = Voxel::countInteriorVoxels(intersectionGrid);

		double ratio = static_cast<float>(intersectionVoxelCount) / polyhedronVoxelCount;
		if (ratio > 0.7)
		{
#ifdef ENABLE_ALGO_DEBUG
				CGAL::IO::write_OBJ(GAlgoDebugOutputDir + "SegmentationResults/IndivisiblePolyhedrons/Useful/" + std::to_string(i) + "_IndivisiblePolyhedron.obj", polyhedron.polyhedronMesh);
#endif // ENABLE_ALGO_DEBUG

			usefulPolyhedrons.push_back(index);
		}
		else
		{
#ifdef ENABLE_ALGO_DEBUG
			CGAL::IO::write_OBJ(GAlgoDebugOutputDir + "SegmentationResults/IndivisiblePolyhedrons/Useless/" + std::to_string(i) + "_IndivisiblePolyhedron.obj", polyhedron.polyhedronMesh);
#endif // ENABLE_ALGO_DEBUG
		}
	}

	Mesh mergedMesh;
	for (size_t i = 0; i < usefulPolyhedrons.size(); i++)
	{
		int index = usefulPolyhedrons[i];
		auto& polyhedron = polyhedrons[index];
		Mesh& polyhedronMesh = polyhedron.polyhedronMesh;
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

#ifdef ENABLE_ALGO_DEBUG
	CGAL::IO::write_OBJ(GAlgoDebugOutputDir + "SegmentationResults/MergedMesh.obj", mergedMesh);
#endif // ENABLE_ALGO_DEBUG

	// Merge polyhedrons
#pragma region Merge polyhedrons
	Mesh voxelMesh;

	openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(0.3);
	CGALMeshAdapter adapter(&mergedMesh, transform);
	openvdb::FloatGrid::Ptr sdfGrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(
		interrupter,
		adapter,
		*transform,
		3.f,
		3.f
	);

	voxelMesh = Voxel::volumeToMesh(sdfGrid);
	size_t removed = CGAL::Polygon_mesh_processing::keep_largest_connected_components(voxelMesh, 1);

#ifdef ENABLE_ALGO_DEBUG
	CGAL::IO::write_OBJ(GAlgoDebugOutputDir + "SegmentationResults/VoxelMesh.obj", voxelMesh);
#endif // ENABLE_ALGO_DEBUG

	CGAL::Polygon_mesh_processing::triangulate_faces(voxelMesh);

#ifdef ENABLE_ALGO_DEBUG
	CGAL::IO::write_OBJ(GAlgoDebugOutputDir + "SegmentationResults/VoxelMesh_Triangulate.obj", voxelMesh);
#endif // ENABLE_ALGO_DEBUG
	
#pragma endregion

	return voxelMesh;
}

int SegmentationManager::CreateDividingSurface(const std::pair<int, std::unordered_set<face_descriptor>>& pair)
{
	DividingSurface ds;
	ds.partitionID = pair.first;
	const auto& faces = pair.second;

	// Calculate Center Point
	#pragma region
	std::unordered_set<Point_3> points;
	for (const auto& face : faces)
	{
		for (const auto& vertex : CGAL::vertices_around_face(mesh.halfedge(face), mesh))
		{
			points.insert(mesh.point(vertex));
		}
	}
	Vector_3 sumVector(0.0, 0.0, 0.0);
	for (const auto& point : points)
	{
		sumVector = sumVector + Vector_3(point.x(), point.y(), point.z());
	}
	sumVector /= points.size();
	ds.centerPoint = Point_3(sumVector.x(), sumVector.y(), sumVector.z());
	#pragma endregion

	// Calculate Dividing Plane
	#pragma region
	auto fNormalMap = *mesh.property_map<face_descriptor, Vector_3>("f:normal");
	ds.averageNormal = Vector_3(0.0, 0.0, 0.0);
	for (const auto& face : faces)
	{
		ds.averageNormal += fNormalMap[face];
	}
	ds.averageNormal /= faces.size();

	Plane_3 plane;
	CGAL::linear_least_squares_fitting_3(points.begin(), points.end(), plane, CGAL::Dimension_tag<0>());
	if (plane.orthogonal_vector() * ds.averageNormal < 0)
	{
		ds.plane = UtilLib::ReversePlane(plane);
	}
	else
	{
		ds.plane = plane;
	}
	ds.normal = ds.plane.orthogonal_vector();
	ds.normal /= std::sqrt(ds.normal.squared_length());

	// Calculate planeMesh
	Mesh chMesh;
	CGAL::convex_hull_3(points.begin(), points.end(), chMesh);
	std::vector<Point_3> convexHullPoints;
	float maxDistSq = 0.0;
	for (auto it1 = chMesh.points().begin(); it1 != chMesh.points().end(); ++it1)
	{
		for (auto it2 = std::next(it1); it2 != chMesh.points().end(); ++it2)
		{
			float d = CGAL::squared_distance(*it1, *it2);
			if (d > maxDistSq)
			{
				maxDistSq = d;
			}
		}
	}
	ds.planeMesh = UtilLib::CreatePlaneMesh(ds.plane, ds.centerPoint, std::sqrt(maxDistSq) * 0.7);
	#pragma endregion

	this->dividingSurfaces.insert(std::make_pair(ds.partitionID, ds));
	return ds.partitionID;
}

int SegmentationManager::CreatePolyhedron(Mesh polyhedronMesh, std::unordered_set<int> parentDividingSurfaces)
{
	PolyhedronFromPartition polyhedron;
	polyhedron.polyhedronID = GetNewPolyhedronID();
	for (int partitionID : parentDividingSurfaces)
	{
		const auto& ds = dividingSurfaces.at(partitionID);
		CGAL::Side_of_triangle_mesh<Mesh, Kernel> insideTester(polyhedronMesh);
		CGAL::Bounded_side boundResult = insideTester(ds.centerPoint);
		if (boundResult == CGAL::ON_UNBOUNDED_SIDE)
		{
			if (CGAL::Polygon_mesh_processing::do_intersect(polyhedronMesh, ds.planeMesh))
			{
				polyhedron.dividingSurfaces.insert(partitionID);
			}
		}
		else
		{
			polyhedron.dividingSurfaces.insert(partitionID);
		}
	}

	auto polyhedronPartitions = PartitionFunctions::PartitionByNormal(polyhedronMesh, 0.01);
	polyhedron.polyhedronMesh = Remesh::DoRemesh(polyhedronMesh, polyhedronPartitions);

	this->polyhedrons.insert(std::make_pair(polyhedron.polyhedronID, polyhedron));
	return polyhedron.polyhedronID;
}

PolyhedronFromPlane::PolyhedronFromPlane(Mesh inPolyhedronMesh, std::vector<int> inPlaneIndices, const std::vector<RansacPlane>& planes, const std::vector<std::vector<Point_3>>& planePoints) : polyhedronMesh(inPolyhedronMesh)
{
	auto polyhedronPartitions = PartitionFunctions::PartitionByNormal(polyhedronMesh, 0.01);
	polyhedronMesh = Remesh::DoRemesh(polyhedronMesh, polyhedronPartitions);
	CGAL::Side_of_triangle_mesh<Mesh, Kernel> checkPoint(polyhedronMesh);
	for (int index : inPlaneIndices)
	{
		const auto& plane = planes[index];
		const auto& points = planePoints[index];
		for (const auto& point : points)
		{
			if (checkPoint(point) == CGAL::ON_BOUNDED_SIDE)
			{
				planeIndices.push_back(index);
				break;
			}
		}
	}
}
