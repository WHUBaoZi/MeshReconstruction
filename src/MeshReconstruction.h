#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>


namespace MeshReconstruction
{
	Mesh DoReconstruction(const CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3>& mesh);
}