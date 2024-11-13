#include <string>
#include <omp.h>
#include <vector>

#include "CGALTypes.h"

void writeWireframeOBJ(std::string outputFileName, const Mesh& mesh)
{
	std::ofstream os(outputFileName);
	size_t verticesCount = mesh.number_of_vertices();
	size_t edgesCount = mesh.number_of_edges();

	os << "# file written from a CGAL tool in Wavefront obj format" << std::endl;
	os << "# This is a wireframe model" << std::endl;
	os << "# " << verticesCount << " vertices" << std::endl;
	os << "# " << edgesCount << " lines" << std::endl;
	os << std::endl;
	os << std::endl;

	os << "# " << verticesCount << " vertices" << std::endl;
	os << "# ------------------------------------------" << std::endl;
	os << std::endl;
	for (vertex_descriptor vertex : mesh.vertices())
	{
		Point_3 point = mesh.point(vertex);
		os << "v " << point.x() << " " << point.y() << " " << point.z() << std::endl;
	}
	os << std::endl;
	os << "# " << edgesCount << " lines" << std::endl;
	os << "# ------------------------------------------" << std::endl;
	for (edge_descriptor edge : mesh.edges())
	{
		vertex_descriptor v1 = mesh.vertex(edge, 1);
		vertex_descriptor v2 = mesh.vertex(edge, 0);
		os << "l " << v1.idx() + 1 << " " << v2.idx() + 1 << " " << std::endl;
	}
	os << std::endl;
	os << "# End of Wavefront obj format #" << std::endl;
}

std::vector<Point_2> polygonToVector(Polygon_2 polygon)
{
	std::vector<Point_2> pointsVector;
	for (const auto& point : polygon)
	{
		pointsVector.push_back(point);
	}
	return pointsVector;
}

int main(int argc, char* argv[])
{
	int slicesNum = 15;
	std::string inputFile = "../Data/chosenTestData/test6.obj";
	std::string outputPath = "../Data/Output/";
	if (argc > 1)
	{
		int slicesNum = std::stoi(argv[1]);
		std::string inputFile = argv[2];
		std::string outputPath = argv[3];
	}
	size_t lastSlashPos = inputFile.find_last_of("/\\");
	size_t lastDotPos = inputFile.find_last_of('.');
	std::string fileName = inputFile.substr(lastSlashPos + 1, lastDotPos - lastSlashPos - 1);

	Mesh mesh;
	CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(inputFile, mesh);
	CGAL::Polygon_mesh_processing::remove_degenerate_faces(mesh);
	CGAL::Polygon_mesh_processing::remove_isolated_vertices(mesh);

#pragma region Get maxZ and minZ of mesh
	double maxZ = -std::numeric_limits<double>::infinity();
	double minZ = std::numeric_limits<double>::infinity();
	for (const auto& vertex : mesh.vertices())
	{
		double pointZ = mesh.point(vertex).z();
		if (pointZ > maxZ)
		{
			maxZ = pointZ;
		}
		if (pointZ < minZ)
		{
			minZ = pointZ;
		}
	}
#pragma endregion

	std::vector<Mesh> sliceMeshes(slicesNum);
	std::vector<std::vector<Polygon_2>> polygons(slicesNum);
	std::vector<double> sliceHeight(slicesNum);
	double sliceSpace = (maxZ - minZ) / (slicesNum + 1);

#pragma omp parallel for
	for (int i = 0; i < slicesNum; i++)
	{
		float planeHeight = minZ + sliceSpace * (i + 1);
		Mesh meshCopy = mesh;

		bool success = false;
		while (!success) 
		{
			try 
			{
				// 尝试进行切割操作
				CGAL::Polygon_mesh_processing::clip(meshCopy, Plane_3(Point_3(0, 0, planeHeight), Vector_3(0, 0, -1)));
				sliceHeight[i] = planeHeight;
				success = true; // 如果没有异常，切割成功
			}
			catch (const std::exception& e) 
			{
				// 处理异常，调整切割高度并重试
				std::cerr << "Skipping slice at height " << planeHeight << " due to exception: " << e.what() << std::endl;
				planeHeight += 0.01;  // 调整切割高度
				meshCopy = mesh;
			}
		}

		std::set<vertex_descriptor> visitedVertices;
		std::map<vertex_descriptor, vertex_descriptor> v2v;
		std::set<halfedge_descriptor> visitedEdges;

		for (const auto& halfedge : meshCopy.halfedges())
		{

			if (meshCopy.is_border(halfedge) && visitedEdges.find(halfedge) == visitedEdges.end())	// 当前半边未访问过，且是边界半边
			{
				Polygon_2 polygon;
				halfedge_descriptor start = halfedge;
				halfedge_descriptor currentHalfedge = halfedge;
				do
				{
					vertex_descriptor vertex = meshCopy.target(currentHalfedge);
					Point_3 point3 = meshCopy.point(vertex);
					polygon.push_back(Point_2(point3.x(), point3.y()));
					visitedEdges.insert(currentHalfedge);

					vertex_descriptor source = meshCopy.source(currentHalfedge);
					vertex_descriptor target = meshCopy.target(currentHalfedge);
					if (visitedVertices.insert(source).second)
					{
						v2v.insert(std::make_pair(source, sliceMeshes[i].add_vertex(meshCopy.point(source))));
					}
					if (visitedVertices.insert(target).second)
					{
						v2v.insert(std::make_pair(target, sliceMeshes[i].add_vertex(meshCopy.point(target))));
					}
					sliceMeshes[i].add_edge(v2v[source], v2v[target]);
					visitedVertices.insert(source);
					visitedVertices.insert(target);

					for (const auto h : CGAL::halfedges_around_target(currentHalfedge, meshCopy))
					{
						halfedge_descriptor oh = meshCopy.opposite(h);
						if (meshCopy.is_border(oh))
						{
							currentHalfedge = oh;
							break;
						}
					}
				} while (currentHalfedge != start);
				polygons[i].push_back(polygon);
			}
		}
	}

	// 合并sliceMeshes
	Mesh sliceMesh;
	for (const auto& sMesh : sliceMeshes)
	{
		std::set<vertex_descriptor> verticesUsed;
		std::map<vertex_descriptor, vertex_descriptor> v2v;
		for (const auto& halfedge : sMesh.halfedges())
		{
			vertex_descriptor source = sMesh.source(halfedge);
			vertex_descriptor target = sMesh.target(halfedge);
			if (verticesUsed.insert(source).second)
			{
				v2v.insert(std::make_pair(source, sliceMesh.add_vertex(sMesh.point(source))));
			}
			if (verticesUsed.insert(target).second)
			{
				v2v.insert(std::make_pair(target, sliceMesh.add_vertex(sMesh.point(target))));
			}
			sliceMesh.add_edge(v2v[source], v2v[target]);
		}
	}
	writeWireframeOBJ(outputPath + fileName + "_Slice.obj", sliceMesh);


#pragma region Simplify Polygons
	Mesh simpSliceMesh;
	std::vector<std::vector<Polygon_2>> simpPolygons(slicesNum);
	CGAL::Polyline_simplification_2::Stop_above_cost_threshold Stop(0.5);
	CGAL::Polyline_simplification_2::Squared_distance_cost Cost;
	for (int i = 0; i < slicesNum; i++)
	{
		for (int j = 0; j < polygons[i].size(); j++)
		{
			simpPolygons[i].push_back(CGAL::Polyline_simplification_2::simplify(polygons[i][j], Cost, Stop));
			std::vector<vertex_descriptor> vertices;
			for (const auto& point2 : simpPolygons[i][j])
			{
				vertices.push_back(simpSliceMesh.add_vertex(Point_3(point2.x(), point2.y(), sliceHeight[i])));
			}
			for (int k = 0; k < vertices.size(); k++)
			{
				simpSliceMesh.add_edge(vertices[k], vertices[(k + 1) % vertices.size()]);
			}
		}
	}
	writeWireframeOBJ(outputPath + fileName + "_SimpSlice.obj", simpSliceMesh);
#pragma endregion

	
#pragma region Regularize Polygons
	Mesh regSimpSliceMesh;
	std::vector<std::vector<Polygon_2>> regSimpPolygons(slicesNum);
	for (int i = 0; i < slicesNum; i++)
	{
		regSimpPolygons[i].resize(simpPolygons[i].size());
		for (int j = 0; j < polygons[i].size(); j++)
		{
			std::vector<Point_2> pVector = polygonToVector(simpPolygons[i][j]);
			std::vector<Point_2> outVector;
			CGAL::Shape_regularization::Contours::regularize_closed_contour(pVector, std::back_inserter(outVector));
			std::vector<vertex_descriptor> vertices;
			for (const auto& point2 : outVector)
			{
				regSimpPolygons[i][j].push_back(point2);
				vertices.push_back(regSimpSliceMesh.add_vertex(Point_3(point2.x(), point2.y(), sliceHeight[i])));
			}
			for (int k = 0; k < vertices.size(); k++)
			{
				regSimpSliceMesh.add_edge(vertices[k], vertices[(k + 1) % vertices.size()]);
			}
		}
	}
	writeWireframeOBJ(outputPath + fileName + "_RegSimpSlice.obj", regSimpSliceMesh);
#pragma endregion


	printf("done");
}