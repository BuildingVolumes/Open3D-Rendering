#include "PolygonalMeshReader.h"
#include "AdditionalUtilities.h"

PolygonalMeshReader::PolygonalMeshReader()
{
}

//bool PolygonalMeshReader::ReadFromFile(std::string obj_file, open3d::geometry::TriangleMesh* existing_mesh)
//{
//	std::ifstream reader;
//
//	reader.open(obj_file);
//
//	if (!reader.is_open())
//	{
//		std::cout << "Reader couldn't open file!" << std::endl;
//		return false;
//	}
//
//	std::vector<double> verts;
//	std::vector<double> uvs;
//	std::vector<double> normals;
//	std::vector<int> vert_faces;
//	std::vector<int> uv_faces;
//	std::vector<int> normal_faces;
//
//	//existing_mesh->tri
//
//	std::string readline;
//
//	std::vector<std::string> by_parts;
//	std::vector<std::vector<std::string>> face_components;
//
//	while (std::getline(reader, readline))
//	{
//		SplitString(readline, by_parts, " ");
//
//		if (by_parts[0] == "v")
//		{
//			verts.push_back(std::stod(by_parts[1]));
//			verts.push_back(std::stod(by_parts[2]));
//			verts.push_back(std::stod(by_parts[3]));
//		}
//		else if (by_parts[0] == "vt")
//		{
//			uvs.push_back(std::stod(by_parts[1]));
//			uvs.push_back(std::stod(by_parts[2]));
//		}
//		else if (by_parts[0] == "vn")
//		{
//			normals.push_back(std::stod(by_parts[1]));
//			normals.push_back(std::stod(by_parts[2]));
//			normals.push_back(std::stod(by_parts[3]));
//		}
//		else if (by_parts[0] == "f")
//		{
//			for (int i = 1; i < by_parts.size(); ++i)
//			{
//				face_components.push_back(std::vector<std::string>());
//
//				SplitString(by_parts[i], face_components[i - 1], "/", "", false);
//			}
//
//			for (int i = 0; i < face_components.size() - 2; ++i)
//			{
//				vert_faces.push_back(std::stoi(face_components[i][0]));
//
//				if (face_components[i][1] != "")
//				{
//					uv_faces.push_back(std::stoi(face_components[i][1]));
//				}
//
//				if (face_components[i][2] != "")
//				{
//					normal_faces.push_back(std::stoi(face_components[i][2]));
//				}
//			}
//
//			face_components.clear();
//		}
//
//		by_parts.clear();
//	}
//
//	return true;
//}

bool PolygonalMeshReader::TriangulateQuadOBJ(std::string input_file, std::string output_file)
{
	std::ifstream reader;

	reader.open(input_file);

	if (!reader.is_open())
	{
		std::cout << "Reader couldn't open input file!" << std::endl;
		return false;
	}

	std::ofstream writer;

	writer.open(output_file);

	if (!writer.is_open())
	{
		reader.close();
		std::cout << "Writer couldn't open output file!" << std::endl;
		return false;
	}

	std::string line;
	std::vector<std::string> by_parts;

	while (std::getline(reader, line))
	{
		if (line.substr(0, 2) == "f ")
		{
			SplitString(line, by_parts, " ");

			int tris = by_parts.size() - 2;

			for (int i = 1; i < tris; ++i)
			{
				writer << by_parts[0] << " " << by_parts[1] << " " << by_parts[i + 1] << " " << by_parts[i + 2] << std::endl;
			}

			by_parts.clear();
		}
		else
		{
			writer << line << std::endl;
		}
	}

	writer.close();
	reader.close();
	return true;
}
