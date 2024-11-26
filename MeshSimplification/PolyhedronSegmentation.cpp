#include "PolyhedronSegmentation.h"


Polyhedron::Polyhedron(Mesh polyhedronMesh, std::vector<Plane_3> parentPlanes): polyhedronMesh(polyhedronMesh)
{
	for (const auto& plane : parentPlanes)
	{
		if (UtilLib::intersection(plane, polyhedronMesh))
		{
			planes.push_back(plane);
		}
	}
	planeIntersectionNums.assign(planes.size(), 0);
	for (int i = 0; i < planes.size(); i++)
	{
		for (int j = i + 1; j < planes.size(); j++)
		{
			CGAL::Object result = CGAL::intersection(planes[i], planes[j]);
			const Line_3* intersectionLine = CGAL::object_cast<Line_3>(&result);
			if (intersectionLine)
			{
				for (const auto& face : polyhedronMesh.faces())
				{
					bool bIntersectionResult = UtilLib::intersection(*intersectionLine, face, polyhedronMesh);
					if (bIntersectionResult)
					{
						planeIntersectionNums[i]++;
						planeIntersectionNums[j]++;
						break;
					}
				}
			}
		}
	}
}

Mesh Polyhedron::drawPlanesMesh(const Point_3& centerPoint)
{
	Mesh mesh;
	for (const auto& plane : planes)
	{
		UtilLib::createPlaneMesh(plane, centerPoint, mesh);
	}
	return mesh;
}


PolyhedronSegmentation::PolyhedronSegmentation(Mesh mesh, std::map<int, Plane_3> partitionPlaneMap): mesh(mesh), partitionPlaneMap(partitionPlaneMap)
{
#pragma region Make Cube Mesh
	Point_3 minPoint(UtilLib::INF, UtilLib::INF, UtilLib::INF);
	Point_3 maxPoint(-UtilLib::INF, -UtilLib::INF, -UtilLib::INF);
	for (const auto& vertex : mesh.vertices())
	{
		Point_3 point = mesh.point(vertex);
		minPoint = Point_3(std::min(minPoint.x(), point.x()), std::min(minPoint.y(), point.y()), std::min(minPoint.z(), point.z()));
		maxPoint = Point_3(std::max(maxPoint.x(), point.x()), std::max(maxPoint.y(), point.y()), std::max(maxPoint.z(), point.z()));
	}
	cubeMesh = UtilLib::makeCube(minPoint, maxPoint);
#pragma endregion
}

Mesh PolyhedronSegmentation::run()
{
	///////////////////////////////////////////
	// for test
	std::string outputPath = "../Data/Output/ClipTest/";
	///////////////////////////////////////////
	Point_3 centerPoint = UtilLib::getMeshCenterPoint(cubeMesh);
	{
		std::vector<Plane_3> planes;
		for (const auto& pair : partitionPlaneMap)
		{
			planes.push_back(pair.second);
		}
		polyhedrons.push(Polyhedron(cubeMesh, planes));
	}

	int loopNum = 0;
	while (!polyhedrons.empty())
	{
		Polyhedron& polyhedron = polyhedrons.front();
		int clipPlaneIndex = polyhedron.GetMinIntersectionIndex();
		Plane_3 clipPlane = polyhedron.planes[clipPlaneIndex];
		Mesh belowMesh = polyhedron.polyhedronMesh;
		Mesh aboveMesh = polyhedron.polyhedronMesh;
		//CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Polyhedron.obj", polyhedron.polyhedronMesh);
		//CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_PolyhedronPlanes.obj", polyhedron.drawPlanesMesh(centerPoint));

		CGAL::Polygon_mesh_processing::clip(belowMesh, clipPlane, CGAL::parameters::clip_volume(true));
		CGAL::Polygon_mesh_processing::clip(aboveMesh, UtilLib::reversePlane(clipPlane), CGAL::parameters::clip_volume(true));

		bool belowValid = UtilLib::isPolyhedronValid(belowMesh);
		bool aboveValid = UtilLib::isPolyhedronValid(aboveMesh);

		std::vector<Plane_3> parentPlanes = polyhedron.planes;
		parentPlanes.erase(parentPlanes.begin() + clipPlaneIndex);
		if (belowValid)
		{
			//CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Below.obj", belowMesh);
			
			Polyhedron belowPolyhedron(belowMesh, parentPlanes);
			if (!belowPolyhedron.planes.empty())
			{
				polyhedrons.push(belowPolyhedron);
			}
			else
			{
				indivisiblePolyhedrons.push_back(belowPolyhedron);
			}
		}
		if (aboveValid)
		{
			//CGAL::IO::write_OBJ(outputPath + std::to_string(loopNum) + "_Above.obj", aboveMesh);
			Polyhedron abovePolyhedron(aboveMesh, parentPlanes);
			if (!abovePolyhedron.planes.empty())
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

	Tree tree(mesh.faces().begin(), mesh.faces().end(), mesh);
	for (int i = 0; i < indivisiblePolyhedrons.size(); i++)
	{
		Point_3 centroid = CGAL::Polygon_mesh_processing::centroid(indivisiblePolyhedrons[i].polyhedronMesh);
		Ray questRay(centroid, Point_3(centroid.x() + 1, centroid.y(), centroid.z()));
		std::size_t intersections = tree.number_of_intersected_primitives(questRay);
		for (auto v : indivisiblePolyhedrons[i].polyhedronMesh.vertices())
		{
			Point_3 oldPoint = indivisiblePolyhedrons[i].polyhedronMesh.point(v);
			indivisiblePolyhedrons[i].polyhedronMesh.point(v) = Point_3(oldPoint.x() - centerPoint.x(), oldPoint.y() - centerPoint.y(), oldPoint.z() - centerPoint.z());
		}

		if (intersections % 2 == 0)
		{
			CGAL::IO::write_OBJ(outputPath + "IndivisiblePolyhedrons/Useless/" + std::to_string(i) + "_IndivisiblePolyhedron.obj", indivisiblePolyhedrons[i].polyhedronMesh);
		}
		else 
		{
			CGAL::IO::write_OBJ(outputPath + "IndivisiblePolyhedrons/Useful/" + std::to_string(i) + "_IndivisiblePolyhedron.obj", indivisiblePolyhedrons[i].polyhedronMesh);
		}
	}
	printf("done");
    return Mesh();
}
