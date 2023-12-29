#include "VV_Mesh.h"
#include <fstream>
#include <iostream>

bool VV_Mesh::DecimateMesh(int vertex_target, int subdivision_level)
{
	if (vertex_target <= 0 || subdivision_level <= 0)
	{
		std::cout << "Invalid mesh decimation parameters" << std::endl;
		return false;
	}

	if (!ValidVertices())
	{
		std::cout << "Invalid mesh vertices" << std::endl;
		return false;
	}
	if (!ValidTriangles())
	{
		std::cout << "Invalid mesh vertices" << std::endl;
		return false;
	}

	return true;
}

bool VV_Mesh::WriteOBJ(std::string filename)
{
	if (!ValidVertices())
	{
		std::cout << "Invalid mesh vertices" << std::endl;
		return false;
	}
	if (!ValidTriangles())
	{
		std::cout << "Invalid mesh vertices" << std::endl;
		return false;
	}

	std::ofstream savefile;

	savefile.open(filename);

	if (!savefile.is_open())
	{
		std::cout << "Couldn't open file" << std::endl;
		return false;
	}
	
	WriteVertices(savefile);
	
	if (ValidUVs())
	{
		WriteUVs(savefile);
	}

	if (ValidNormals())
	{
		WriteNormals(savefile);
	}

	WriteTriangles(savefile);

	savefile.close();

	return true;
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
	int i_count = triangles.size();

	std::string to_write;

	for (int i = 0; i < i_count; i += 9)
	{
		to_write = "f " +
			std::to_string(triangles[i]) + "\\" +
			std::to_string(triangles[i + 1]) + "\\" +
			std::to_string(triangles[i + 2]) + " " +
			std::to_string(triangles[i + 3]) + "\\" +
			std::to_string(triangles[i + 4]) + "\\" +
			std::to_string(triangles[i + 5]) + " " +
			std::to_string(triangles[i + 6]) + "\\" +
			std::to_string(triangles[i + 7]) + "\\" +
			std::to_string(triangles[i + 8]) + "\n";

		savefile.write(to_write.c_str(), to_write.size());
	}
}

void VV_Mesh::GetQuadricErrorMetric(int index, int* presizedQuadric)
{
	auto v0 = triangles[index];
	auto v1 = triangles[index + 1];
	auto v2 = triangles[index + 2];

	auto v0x = vertices[v0];
	auto v0y = vertices[v0 + 1];
	auto v0z = vertices[v0 + 2];

	auto v1x = vertices[v1];
	auto v1y = vertices[v1 + 1];
	auto v1z = vertices[v1 + 2];

	auto v2x = vertices[v2];
	auto v2y = vertices[v2 + 1];
	auto v2z = vertices[v2 + 2];

	
}
