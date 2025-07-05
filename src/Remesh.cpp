#include "Remesh.h"

RemeshManager::RemeshManager(Mesh* inMesh) : mesh(inMesh)
{
	partitionMap = UtilLib::PartitionByNormal(*mesh, 0.00005);
	fChartMap = mesh->property_map<face_descriptor, size_t>("f:chart").first;
	vCornerMap = mesh->add_property_map<vertex_descriptor, bool>("v:corner", false).first;
	for (auto vertex : mesh->vertices())
	{
		std::set<size_t> chartSet;
		for (const auto& halfedge : CGAL::halfedges_around_target(mesh->halfedge(vertex), *mesh))
		{
			face_descriptor face = mesh->face(halfedge);
			if (face != Mesh::null_face())
			{
				chartSet.insert(fChartMap[face]);
			}
		}
		if (chartSet.size() > 2)
		{
			vCornerMap[vertex] = true;
		}
	}
}

Mesh* RemeshManager::Run(std::string outputPath)
{
	if (!outputPath.empty() && !boost::filesystem::exists(outputPath))
	{
		boost::filesystem::create_directories(outputPath);
	}
	auto fColorMap = mesh->property_map<face_descriptor, CGAL::Color>("f:color").first;
	CGAL::IO::write_PLY(outputPath + "RemeshClassifyMesh.ply", *mesh, CGAL::parameters::face_color_map(fColorMap).use_binary_mode(false));

	for (const auto& pair : partitionMap)
	{
		size_t id = pair.first;
		remeshPartitions[id] = std::make_unique<RemeshPartition>(id, this, pair.second);
	}
	Mesh wireframeMesh, originalWireframeMesh;
	for (const auto& pair : remeshPartitions)
	{
		auto& partition = pair.second;
		if (partition->bIsValid)
		{
			for (size_t i = 0; i < partition->simpBoundaries.size(); i++)
			{
				const auto& boundary = partition->simpBoundaries[i];
				std::vector<vertex_descriptor> vertices;
				for (const auto& vertex : boundary)
				{
					vertices.push_back(wireframeMesh.add_vertex(mesh->point(vertex)));
				}
				for (size_t j = 0; j < vertices.size(); j++)
				{
					size_t next = j + 1;
					if (j == vertices.size() - 1)
					{
						next = 0;
					}
					wireframeMesh.add_edge(vertices[j], vertices[next]);
				}
			}

			for (size_t i = 0; i < partition->boundaries.size(); i++)
			{
				const auto& testBoundary = partition->boundaries[i];
				std::vector<vertex_descriptor> testVertices;
				for (const auto& vertex : testBoundary)
				{
					testVertices.push_back(originalWireframeMesh.add_vertex(mesh->point(vertex)));
				}
				for (size_t j = 0; j < testVertices.size(); j++)
				{
					size_t next = j + 1;
					if (j == testVertices.size() - 1)
					{
						next = 0;
					}
					originalWireframeMesh.add_edge(testVertices[j], testVertices[next]);
				}
			}
		}
	}
	UtilLib::WriteWireframeOBJ(outputPath + "WireframeOBJ.obj", wireframeMesh);
	UtilLib::WriteWireframeOBJ(outputPath + "OriginWireframeOBJ.obj", originalWireframeMesh);

	std::map<vertex_descriptor, vertex_descriptor> v2NewV;
	for (const auto& pair : remeshPartitions)
	{
		size_t id = pair.first;
		auto& partition = pair.second;
		if (partition->bIsValid)
		{
			CDT cdt;
			std::vector<CDT::Vertex_handle> vhs;
			std::map<CDT::Vertex_handle, vertex_descriptor> v2v;
			for (size_t i = 0; i < partition->simpBoundaries.size(); i++)
			{
				const auto& boundary = partition->simpBoundaries[i];
				for (const auto& vertex : boundary)
				{
					Point_2 p2 = partition->ProjectTo2D(mesh->point(vertex));
					auto cdtV = cdt.insert(p2);
					v2v[cdtV] = vertex;
					vhs.push_back(cdtV);
				}
			}
			for (size_t i = 0; i < vhs.size(); i++)
			{
				cdt.insert_constraint(vhs[i], vhs[(i + 1) % vhs.size()]);
			}
			std::unordered_map<Face_handle, bool> in_domain_map;
			boost::associative_property_map<std::unordered_map<Face_handle, bool>>in_domain(in_domain_map);
			CGAL::mark_domain_in_triangulation(cdt, in_domain);
			for (Face_handle f : cdt.finite_face_handles())
			{
				if (!get(in_domain, f))
				{
					continue;
				}
				bool faceValid = true;
				std::vector<CDT::Vertex_handle> faceVhs;
				for (int j = 0; j < 3; ++j)
				{
					CDT::Vertex_handle vh = f->vertex(j);
					faceVhs.push_back(vh);
					if (v2v.find(vh) == v2v.end())
					{
						faceValid = false;
						break;
					}
				}
				if (!faceValid)
				{
					std::cerr << "Warning: Face has unmapped vertex, skipping." << std::endl;
					continue;
				}
				std::vector<vertex_descriptor> newVertices;
				newVertices.reserve(3);
				for (size_t i = 0; i < faceVhs.size(); i++)
				{
					auto oldVertex = v2v.at(faceVhs[i]);
					if (v2NewV.find(oldVertex) == v2NewV.end())
					{
						v2NewV[oldVertex] = remeshedMesh.add_vertex(mesh->point(oldVertex));
					}
					newVertices.push_back(v2NewV[oldVertex]);
				}

				remeshedMesh.add_face(newVertices[0], newVertices[1], newVertices[2]);
			}
		}
	}
	CGAL::IO::write_OBJ(outputPath + "RemeshedResult.obj", remeshedMesh);

	return &remeshedMesh;
}

RemeshPartition::RemeshPartition(size_t inId, RemeshManager* inRemeshManager, const std::set<face_descriptor>& inFaces) : id(inId), remeshManager(inRemeshManager), mesh(inRemeshManager->mesh), faces(inFaces)
{
	auto fNormalMap = mesh->property_map<face_descriptor, Vector_3>("f:normal").first;
	auto fColorMap = mesh->property_map <face_descriptor, CGAL::Color>("f:color").first;
	Vector_3 checkNormal(0.0, 0.0, 0.0);
	std::vector<halfedge_descriptor> boundaryHalfedges;
	for (const auto& f : faces)
	{
		for (const halfedge_descriptor& halfedge : CGAL::halfedges_around_face(mesh->halfedge(f), *mesh))
		{
			halfedge_descriptor oppositeHalfedge = mesh->opposite(halfedge);
			if (faces.find(mesh->face(oppositeHalfedge)) == faces.end())
			{
				boundaryHalfedges.push_back(halfedge);
			}
		}
		checkNormal += fNormalMap[f];
	}

	std::map<vertex_descriptor, std::set<halfedge_descriptor>> sourceHalfdegesMap;
	for (const auto& halfedge : boundaryHalfedges)
	{
		vertex_descriptor source = mesh->source(halfedge);
		sourceHalfdegesMap[source].insert(halfedge);
	}

	std::set<halfedge_descriptor> visitedHalfedges;
	for (const auto& halfedge : boundaryHalfedges)
	{
		if (!visitedHalfedges.insert(halfedge).second)
		{
			continue;
		}
		std::vector<std::vector<halfedge_descriptor>> halfedgeBoundaries = ExtractBoundaries(halfedge, sourceHalfdegesMap);
		for (const auto& halfedgeBoundary : halfedgeBoundaries)
		{
			std::vector<vertex_descriptor> boundary;
			visitedHalfedges.insert(halfedgeBoundary.begin(), halfedgeBoundary.end());
			for (const halfedge_descriptor he : halfedgeBoundary)
			{
				boundary.push_back(mesh->source(he));
			}
			boundaries.push_back(boundary);
		}
	}
	for (const auto& boundary : boundaries)
	{
		std::vector<vertex_descriptor> simpBoundary;
		for (const auto& vertex : boundary)
		{
			if (remeshManager->vCornerMap[vertex])
			{
				simpBoundary.push_back(vertex);
			}
		}
		if (simpBoundary.size() > 2)
		{
			simpBoundaries.push_back(simpBoundary);
		}
	}
	if (simpBoundaries.size() > 0)
	{
		for (const auto& face : faces)
		{
			for (const auto& vertex : CGAL::vertices_around_face(mesh->halfedge(face), *mesh))
			{
				points.insert(mesh->point(vertex));
			}
		}
		CGAL::linear_least_squares_fitting_3(points.begin(), points.end(), fitPlane, CGAL::Dimension_tag<0>());
		fitPlaneNormal = fitPlane.orthogonal_vector() / std::sqrt(fitPlane.orthogonal_vector().squared_length());
		checkNormal /= std::sqrt(checkNormal.squared_length());
		if (fitPlaneNormal * checkNormal < 0)
		{
			fitPlane = UtilLib::ReversePlane(fitPlane);
			fitPlaneNormal = fitPlane.orthogonal_vector() / std::sqrt(fitPlane.orthogonal_vector().squared_length());
		}
		UtilLib::BuildLocalBasis(fitPlaneNormal, u, v);
	}
	else
	{
		bIsValid = false;
	}
}

std::vector<std::vector<halfedge_descriptor>> RemeshPartition::ExtractBoundaries(const halfedge_descriptor& startHalfedge, std::map<vertex_descriptor, std::set<halfedge_descriptor>>& sourceHalfdegesMap)
{
	size_t maxLoop = sourceHalfdegesMap.size() * 2;
	std::vector<std::vector<halfedge_descriptor>> boundaries;
	vertex_descriptor startVertex = mesh->source(startHalfedge);
	halfedge_descriptor currentHalfedge = startHalfedge;
	std::vector<halfedge_descriptor> boundary;
	size_t loopNum = 0;
	while (true)
	{
		loopNum++;
		if (loopNum > maxLoop)
		{
			std::cerr << "Border search error!" << std::endl;
			break;
		}
		boundary.push_back(currentHalfedge);
		vertex_descriptor targetVertex = mesh->target(currentHalfedge);
		if (targetVertex == startVertex)
		{
			break;
		}
		const auto& halfedges = sourceHalfdegesMap[targetVertex];
		if (halfedges.size() != 1)
		{
			// Prevent self-intersecting polygons
			boundary.clear();
			for (const auto& halfedge : halfedges)
			{
				auto currentBoundaries = ExtractBoundaries(halfedge, sourceHalfdegesMap);
				boundaries.insert(boundaries.end(), currentBoundaries.begin(), currentBoundaries.end());
			}
			break;
		}
		else
		{
			currentHalfedge = *halfedges.begin();
		}
	}
	if (!boundary.empty())
	{
		boundaries.push_back(boundary);
	}
	return boundaries;
}

Point_2 RemeshPartition::ProjectTo2D(Point_3 point3)
{
	Vector_3 vec = point3 - fitPlane.point();
	return Point_2(vec * u, vec * v);
}
