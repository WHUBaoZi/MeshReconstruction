#include "Partition.h"

Partition::Partition()
{
}

Partition::Partition(std::set<face_descriptor> faces, const Mesh* mesh) : faces(faces), mesh(mesh)
{
	CalculateAverageNormal();
	area = CGAL::Polygon_mesh_processing::area(faces, *mesh);
	for (const auto& face : faces)
	{
		for (const auto& vertex : CGAL::vertices_around_face((*mesh).halfedge(face), *mesh))
		{
			vertices.insert(vertex);
			points.insert((*mesh).point(vertex));
		}
	}
	Vector_3 sumVector(0.0, 0.0, 0.0);
	for (const auto& point : points)
	{
		sumVector = sumVector + Vector_3(point.x(), point.y(), point.z());
	}
	sumVector /= points.size();
	centerPoint = Point_3(sumVector.x(), sumVector.y(), sumVector.z());

	Plane_3 plane;
	CGAL::linear_least_squares_fitting_3(points.begin(), points.end(), plane, CGAL::Dimension_tag<0>());
	if (plane.orthogonal_vector() * facesAverageNormal < 0)
	{
		fitPlane = UtilLib::ReversePlane(plane);
	}
	else
	{
		fitPlane = plane;
	}
	fitPlaneNormal = fitPlane.orthogonal_vector() / std::sqrt(fitPlane.orthogonal_vector().squared_length());
}


void Partition::CalculateAverageNormal()
{
	auto fNormalMap = (*mesh).property_map<face_descriptor, Vector_3>("f:normal").first;
	for (const auto& face : faces)
	{
		facesAverageNormal += fNormalMap[face];
	}
	facesAverageNormal /= faces.size();
}

bool Partition::CheckPartitionProximity(const Partition& partitionA, const Partition& partitionB)
{
	double dot = partitionA.fitPlaneNormal * partitionB.fitPlaneNormal;
	if (std::abs(dot) > std::cos(10 * UtilLib::DEG_TO_RAD))
	{
		double distanceA = CGAL::squared_distance(partitionA.centerPoint, partitionB.fitPlane);
		double distanceB = CGAL::squared_distance(partitionB.centerPoint, partitionA.fitPlane);
		if (distanceA < 2 || distanceB < 2)
		{
			return true;
		}
	}
	return false;
}
