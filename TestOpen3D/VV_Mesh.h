#pragma once

#include <vector>
#include <string>

class VV_Mesh
{
public:
	std::vector<float> vertices;
	std::vector<float> uvs;
	std::vector<float> normals;

	std::vector<int> triangles;

	bool ValidVertices() { return (vertices.size() % 3 == 0) && (vertices.size() > 0); }
	bool ValidUVs() { return (uvs.size() % 2 == 0) && (vertices.size() > 0); }
	bool ValidNormals() { return (normals.size() % 3 == 0) && (normals.size() > 0); }

	bool ValidTriangles() { return (triangles.size() % 3 == 0) && (triangles.size() > 0); }

	bool DecimateMesh(int vertex_target, int subdivision_level);

	bool WriteOBJ(std::string filename);

private:
	void WriteVertices(std::ofstream &savefile);
	void WriteUVs(std::ofstream &savefile);
	void WriteNormals(std::ofstream &savefile);

	void WriteTriangles(std::ofstream &savefile);

	void GetQuadricErrorMetric(int index, int* presizedQuadric);
};