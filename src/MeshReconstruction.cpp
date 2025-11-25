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
	UtilLib::CentralizeMesh(mesh);

	std::vector<std::vector<Point_3>> planePoints;
	std::vector<RansacPlane> planes = Ransac::RansacPlanes(mesh, planePoints);

	Mesh voxelMesh = PolyhedronSegmentationFunctions::DoSegmentation(mesh, planes, planePoints);

	auto partitions = PartitionFunctions::PartitionByRegionGrowing(voxelMesh);

	Mesh result = Remesh::DoRemesh(voxelMesh, partitions);
	return result;
}