#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#ifdef MESHRECONSTRUCTION_EXPORTS
#define MESH_API __declspec(dllexport)
#else
#define MESH_API __declspec(dllimport)
#endif


struct RansacParams
{
	double epsilon = 0.1;

	double normal_threshold = 0.95;

	double min_points_percent = 0.003;

	double cluster_epsilon_percent = 0.01;
};

namespace MeshReconstruction
{
	extern RansacParams ransacParams;

	MESH_API CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3> DoReconstruction(CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3>& mesh);
}