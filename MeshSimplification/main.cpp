#include <string>
#include <omp.h>

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
		Point3 point = mesh.point(vertex);
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
				// ���Խ����и����
				CGAL::Polygon_mesh_processing::clip(meshCopy, Plane3(Point3(0, 0, planeHeight), Vector3(0, 0, -1)));
				success = true; // ���û���쳣���и�ɹ�
			}
			catch (const std::exception& e) 
			{
				// �����쳣�������и�߶Ȳ�����
				std::cerr << "Skipping slice at height " << planeHeight << " due to exception: " << e.what() << std::endl;
				planeHeight += 0.01;  // �����и�߶�
				meshCopy = mesh;
			}
		}

		std::set<vertex_descriptor> visitedVertices;
		std::map<vertex_descriptor, vertex_descriptor> v2v;
		std::vector<std::vector<Polygon2>> polygons(slicesNum);
		std::set<halfedge_descriptor> visitedEdges;

		for (const auto& halfedge : meshCopy.halfedges())
		{

			if (meshCopy.is_border(halfedge) && visitedEdges.find(halfedge) == visitedEdges.end())	// ��ǰ���δ���ʹ������Ǳ߽���
			{
				Polygon2 polygon;
				halfedge_descriptor start = halfedge;
				halfedge_descriptor currentHalfedge = halfedge;
				do
				{
					vertex_descriptor vertex = meshCopy.target(halfedge);
					Point3 point3 = meshCopy.point(vertex);
					polygon.push_back(Point2(point3.x(), point3.y()));
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

					for (const auto h : CGAL::halfedges_around_target(halfedge, meshCopy))
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

	// �ϲ�sliceMeshes
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

	printf("done");
}