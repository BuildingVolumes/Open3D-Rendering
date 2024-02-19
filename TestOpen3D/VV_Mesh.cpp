#include "VV_Mesh.h"
#include <fstream>
#include <iostream>

#include "AdditionalUtilities.h"

//bool VV_Mesh::DecimateMesh(int vertex_target, int subdivision_level)
//{
//	if (vertex_target <= 0 || subdivision_level <= 0)
//	{
//		std::cout << "Invalid mesh decimation parameters" << std::endl;
//		return false;
//	}
//
//	if (!ValidVertices())
//	{
//		std::cout << "Invalid mesh vertices" << std::endl;
//		return false;
//	}
//	if (!ValidTriangles())
//	{
//		std::cout << "Invalid mesh vertices" << std::endl;
//		return false;
//	}
//
//	return true;
//}

bool VV_Mesh::WriteOBJ(std::string filename)
{
	std::ofstream savefile;

	savefile.open(filename);

	if (!savefile.is_open())
	{
		std::cout << "Couldn't open file" << std::endl;
		return false;
	}

	WriteTriangles(savefile);

	savefile.close();

	return true;
}

bool VV_Mesh::ReadOBJ(std::string filename)
{
	std::ifstream savefile;

	savefile.open(filename);

	if (!savefile.is_open())
	{
		std::cout << "Couldn't open file" << std::endl;
		return false;
	}

	std::string line;
	std::vector<std::string> attribute_components;

	std::vector<std::string> triangle_components;

	std::vector<std::vector<std::string>> face_components;

	int line_num = 0;

	while (getline(savefile, line))
	{
		++line_num;
		SplitString(line, attribute_components, " ");

		if (line_num == 0)
		{
			std::cout << line << std::endl;
		}

		if (attribute_components[0] == "v")
		{
			vertices.push_back(std::stod(attribute_components[1]));
			vertices.push_back(std::stod(attribute_components[2]));
			vertices.push_back(std::stod(attribute_components[3]));
		}
		else if (attribute_components[0] == "vt")
		{
			uvs.push_back(std::stod(attribute_components[1]));
			uvs.push_back(std::stod(attribute_components[2]));
		}
		else if (attribute_components[0] == "vn")
		{
			normals.push_back(std::stod(attribute_components[1]));
			normals.push_back(std::stod(attribute_components[2]));
			normals.push_back(std::stod(attribute_components[3]));
		}
		else if (attribute_components[0] == "f")
		{
			for (int i = 1; i < attribute_components.size(); ++i)
			{
				SplitString(attribute_components[i], triangle_components, "\\/", "", false);
				if (triangle_components.size() <= 0)
				{
					std::cout << "ERROR - invalid face component structure on line " << line_num << "!" << std::endl;
					savefile.close();
					return false;
				}

				face_components.push_back(triangle_components);
				triangle_components.clear();
			}

			for (int i = 0; i < face_components.size() - 2; ++i)
			{
				triangles_vertices.push_back(std::stoi(face_components[0][0]) - 1);
				triangles_vertices.push_back(std::stoi(face_components[i + 1][0]) - 1);
				triangles_vertices.push_back(std::stoi(face_components[i + 2][0]) - 1);

				if (uvs.size() > 0)
				{
					triangles_uvs.push_back(std::stoi(face_components[0][1]) - 1);
					triangles_uvs.push_back(std::stoi(face_components[i + 1][1]) - 1);
					triangles_uvs.push_back(std::stoi(face_components[i + 2][1]) - 1);
				}

				if (normals.size() > 0)
				{
					triangles_normals.push_back(std::stoi(face_components[0][2]) - 1);
					triangles_normals.push_back(std::stoi(face_components[i + 1][2]) - 1);
					triangles_normals.push_back(std::stoi(face_components[i + 2][2]) - 1);
				}
			}

			face_components.clear();
		}

		attribute_components.clear();
	}

	std::cout << "Lines read: " << line_num << std::endl;

	savefile.close();
	return true;
}

std::shared_ptr<VV_Mesh> VV_Mesh::DecimateEdges(double decimation_ratio)
{
	auto to_return = std::make_shared<VV_Mesh>();

	CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3> sm;

	std::vector<CGAL::Simple_cartesian<double>::Point_3> verts;
	verts.resize(vertices.size() / 3);

	std::vector<Eigen::Vector3i> triangle_faces;
	triangle_faces.resize(triangles_vertices.size() / 3);

	auto verts_mem_size = vertices.size() * sizeof(double);
	auto tri_verts_mem_size = triangles_vertices.size() * sizeof(int);

	memcpy(verts.data(), vertices.data(), verts_mem_size);
	memcpy(triangle_faces.data(), triangles_vertices.data(), tri_verts_mem_size);

	CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(verts, triangle_faces, sm);

	std::cout << "Num faces: " << sm.number_of_faces() << ", should be " << (triangles_vertices.size() / 3) << " (" << triangles_vertices.size() << ")" << std::endl;
	std::cout << "Num verts: " << sm.number_of_vertices() << ", should be " << (vertices.size() / 3) << " (" << vertices.size() << ")" << std::endl;

	CGAL::Surface_mesh_simplification::Edge_count_ratio_stop_predicate<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>> stop(decimation_ratio);

	int r = CGAL::Surface_mesh_simplification::edge_collapse(sm, stop);

	sm.collect_garbage();

	to_return->vertices.resize(sm.number_of_vertices() * 3);
	to_return->triangles_vertices.resize(sm.number_of_faces() * 3);

	for (int i = 0; i < sm.number_of_vertices(); ++i)
	{
		auto vert = sm.point(CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>::Vertex_index(i));
		to_return->vertices[3 * i] = vert.x();
		to_return->vertices[3 * i + 1] = vert.y();
		to_return->vertices[3 * i + 2] = vert.z();
	}

	std::cout << "Decimated faces: " << sm.number_of_faces() << std::endl;

	for (int i = 0; i < sm.number_of_faces(); ++i)
	{
		auto sm_verts = sm.vertices_around_face(sm.halfedge(CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>::Face_index(i)));
		int t = 0;

		for (auto v_index : sm_verts)
		{
			if ((3 * i + t) >= to_return->triangles_vertices.size())
			{
				std::cout << "ERROR! index oob at " << (3 * i + t) << std::endl;
			}

			to_return->triangles_vertices[3 * i + t] = v_index.id();
			++t;
		}
	}

	return to_return;
}

void VV_Mesh::WriteVertices(std::ofstream& savefile)
{
	int v_count = vertices.size();

	std::string to_write;

	for (int i = 0; i < v_count; i += 3)
	{
		to_write = "v " + 
			std::to_string(vertices[i]) + " " + 
			std::to_string(vertices[i + 1]) + " " + 
			std::to_string(vertices[i + 2]) + "\n";

		savefile.write(to_write.c_str(), to_write.size());
	}
}

void VV_Mesh::WriteUVs(std::ofstream& savefile)
{
	int vt_count = uvs.size();

	std::string to_write;

	for (int i = 0; i < vt_count; i += 2)
	{
		to_write = "vt " +
			std::to_string(uvs[i]) + " " +
			std::to_string(uvs[i + 1]) + "\n";

		savefile.write(to_write.c_str(), to_write.size());
	}
}

void VV_Mesh::WriteNormals(std::ofstream& savefile)
{
	int vn_count = normals.size();

	std::string to_write;

	for (int i = 0; i < vn_count; i += 3)
	{
		to_write = "vn " +
			std::to_string(normals[i]) + " " +
			std::to_string(normals[i + 1]) + " " +
			std::to_string(normals[i + 2]) + "\n";

		savefile.write(to_write.c_str(), to_write.size());
	}
}

void VV_Mesh::WriteTriangles(std::ofstream& savefile)
{
	std::vector<bool> validations(3, false);

	validations[0] = (ValidTrianglesVertices() && ValidVertices());
	validations[1] = (ValidTrianglesUVs() && ValidUVs());
	validations[2] = (ValidTrianglesNormals() && ValidNormals());

	std::string to_write;

	int elem_max = 1;

	if (!validations[0])
	{
		std::cout << "ERROR - No triangle vertices!" << std::endl;
		return;
	}

	WriteVertices(savefile);

	if (validations[1])
	{
		elem_max = 2;
		WriteUVs(savefile);
	}

	if (validations[2])
	{
		elem_max = 3;
		WriteNormals(savefile);
	}

	//to_write = "f ";
	for (int i = 0; i < triangles_vertices.size(); i += 3)
	{
		to_write = "f " + std::to_string(triangles_vertices[i] + 1);
		to_write += (validations[1] ? ("/" + std::to_string(triangles_uvs[i] + 1)) : ((elem_max >= 2 ? "/" : "")));
		to_write += (validations[2] ? ("/" + std::to_string(triangles_normals[i] + 1) + " ") : " ");

		to_write += std::to_string(triangles_vertices[i + 1] + 1);
		to_write += (validations[1] ? ("/" + std::to_string(triangles_uvs[i + 1] + 1)) : ((elem_max >= 2 ? "/" : "")));
		to_write += (validations[2] ? ("/" + std::to_string(triangles_normals[i + 1] + 1) + " ") : " ");

		to_write += std::to_string(triangles_vertices[i + 2] + 1);
		to_write += (validations[1] ? ("/" + std::to_string(triangles_uvs[i + 2] + 1)) : ((elem_max >= 2 ? "/" : "")));
		to_write += (validations[2] ? ("/" + std::to_string(triangles_normals[i + 2] + 1) + "\n") : "\n");

		savefile.write(to_write.c_str(), to_write.size());
	}

	savefile.close();
}

//void VV_Mesh::GetQuadricErrorMetric(int index, int* presizedQuadric)
//{
//	auto v0 = triangles_vertices[index];
//	auto v1 = triangles_vertices[index + 1];
//	auto v2 = triangles_vertices[index + 2];
//
//	auto v0x = vertices[v0];
//	auto v0y = vertices[v0 + 1];
//	auto v0z = vertices[v0 + 2];
//
//	auto v1x = vertices[v1];
//	auto v1y = vertices[v1 + 1];
//	auto v1z = vertices[v1 + 2];
//
//	auto v2x = vertices[v2];
//	auto v2y = vertices[v2 + 1];
//	auto v2z = vertices[v2 + 2];
//
//	
//}
