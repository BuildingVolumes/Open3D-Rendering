#pragma once
#include <Alembic/Abc/All.h>
#include <Alembic/AbcGeom/All.h>
#include <Alembic/AbcCoreOgawa/All.h>
#include <math.h>
#include <Alembic/AbcMaterial/MaterialAssignment.h>


struct AlembicMeshData {
	size_t numVerts;
	std::vector<Alembic::Abc::float32_t> vertices;

	size_t numIndicies;
	std::vector<Alembic::Abc::int32_t> indicies;

	size_t numCounts;
	std::vector<Alembic::Abc::int32_t> counts;

	size_t numNormals;
	std::vector<Alembic::Abc::float32_t> normals;

	std::vector<Alembic::AbcGeom::C3f> vertexColours;

};

class AlembicWriter {
private:
	Alembic::AbcGeom::chrono_t startTime; //Start time of the frames
	Alembic::AbcGeom::chrono_t deltaTime; //Time between each frame

	Alembic::AbcGeom::TimeSamplingPtr g_ts;

	Alembic::Abc::OArchive archive; //The Archive is Alebmic's name for the file
	std::string fileName;

	Alembic::Abc::OObject top; //Alembic uses hierarchical modeling and needs a top object
	std::string topName;

	//The mesh is contained in the MeshNode
	Alembic::AbcGeom::v12::OPolyMesh meshNode;
	Alembic::AbcGeom::OPolyMeshSchema mesh;

	Alembic::Abc::OObject materialsNode;
	Alembic::AbcMaterial::OMaterial material;

	Alembic::AbcGeom::OC3fGeomParam colourProps;

	/*
	void setFloatParameter(
		Mat::OMaterialSchema& schema,
		const std::string& target,
		const std::string& shaderType,
		const std::string& paramName, float value) */
	void setFloatParameter(Alembic::AbcMaterial::OMaterialSchema schema, const std::string& target,
		const std::string& shaderType, const std::string& paramName, float value);
public:
	AlembicWriter(std::string fileName, std::string topName, float start, float delta);

	//Call saveFrame and pass in the data for the frame to save it
	void saveFrame(struct AlembicMeshData meshData);
};