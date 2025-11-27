#include "MeshReconstruction.h"

#include "CGALTypes.h"
#include "Ransac.h"
#include "Partition.h"
#include "Remesh.h"
#include "PolyhedronSegmentation.h"
#include "UtilLib.h"

#include "AlgoDebugIO.h"

CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3> MeshReconstruction::DoReconstruction(CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3>& mesh)
{
	// Centralize Mesh
	UtilLib::CentralizeMesh(mesh);

#ifdef ENABLE_ALGO_DEBUG
	CGAL::IO::write_OBJ(GAlgoDebugOutputDir + "CentralizeMesh.obj", mesh);
#endif

	// Mesh Ransac
#ifdef ENABLE_ALGO_DEBUG
	std::cout << "Start Mesh Ransac..." << std::endl;
	start = std::chrono::high_resolution_clock::now();
#endif // ENABLE_ALGO_DEBUG
	std::vector<std::vector<Point_3>> planePoints;
	std::vector<RansacPlane> planes = Ransac::RansacPlanes(mesh, planePoints);
#ifdef ENABLE_ALGO_DEBUG
	end = std::chrono::high_resolution_clock::now();
	timings["MeshPartition"] = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
	std::cout << "Mesh Ransac finished. Time taken: " << timings["MeshPartition"] << " seconds" << std::endl;
#endif // ENABLE_ALGO_DEBUG

	// Segmentation
#ifdef ENABLE_ALGO_DEBUG
	std::cout << "Start Segmentation..." << std::endl;
	start = std::chrono::high_resolution_clock::now();
#endif // ENABLE_ALGO_DEBUG
	Mesh voxelMesh = PolyhedronSegmentationFunctions::DoSegmentation(mesh, planes, planePoints);
#ifdef ENABLE_ALGO_DEBUG
	end = std::chrono::high_resolution_clock::now();
	timings["Segmentation"] = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
	std::cout << "Segmentation finished. Time taken: " << timings["Segmentation"] << " seconds" << std::endl;
#endif // ENABLE_ALGO_DEBUG

	// Remesh
#ifdef ENABLE_ALGO_DEBUG
	std::cout << "Start Remesh..." << std::endl;
	start = std::chrono::high_resolution_clock::now();
#endif // ENABLE_ALGO_DEBUG
	auto partitions = PartitionFunctions::PartitionByRegionGrowing(voxelMesh);
	Mesh result = Remesh::DoRemesh(voxelMesh, partitions);
#ifdef ENABLE_ALGO_DEBUG
	end = std::chrono::high_resolution_clock::now();
	timings["Remesh"] = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
	std::cout << "Remesh finished. Time taken: " << timings["Remesh"] << " seconds" << std::endl;
#endif // ENABLE_ALGO_DEBUG

	// Total time cost
#ifdef ENABLE_ALGO_DEBUG
	std::cout << "All done. Time taken: " << GetTotalTime() << " seconds" << std::endl;
#endif // ENABLE_ALGO_DEBUG


	return result;
}