#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#ifdef MESHRECONSTRUCTION_EXPORTS
#define MESH_API __declspec(dllexport)
#else
#define MESH_API __declspec(dllimport)
#endif


namespace MeshReconstruction
{
	MESH_API CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3> DoReconstruction(CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3>& mesh);
}