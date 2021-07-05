#pragma once
#include "AlembicWriter.h"

void AlembicWriter::setFloatParameter(Alembic::AbcMaterial::OMaterialSchema schema, const std::string& target,
	const std::string& shaderType, const std::string& paramName, float value)
{
	Alembic::Abc::OFloatProperty prop(
		schema.getShaderParameters(target, shaderType),
		paramName);
	prop.set(value);
}


void AlembicWriter::setTimeSampling(float start, float delta) {
	g_ts = Alembic::AbcGeom::TimeSamplingPtr(new Alembic::AbcGeom::TimeSampling(delta, start));
	mesh.setTimeSampling(g_ts);
}

AlembicWriter::AlembicWriter(std::string fileName, std::string topName, float start, float delta) {
	startTime = start;
	deltaTime = delta;
	
	//g_ts = Alembic::AbcGeom::TimeSamplingPtr(new Alembic::AbcGeom::TimeSampling(deltaTime, startTime));
	


	this->fileName = fileName;
	this->topName = topName;

	archive = Alembic::Abc::OArchive(Alembic::AbcCoreOgawa::WriteArchive(), this->fileName);
	std::cout << archive.getName() << std::endl;
	top = archive.getTop();
	//Alembic::AbcGeom::v12::OXform xform(top, this->topName, g_ts);

	


	meshNode = Alembic::AbcGeom::v12::OPolyMesh(top, "meshy");
	
	mesh = meshNode.getSchema();
	
	/*
	Alembic::AbcMaterial::OMaterial material(meshNode, "material");
	std::cout << material.getFullName() << std::endl;
	material.getSchema().setShader("abc", "surface", "bsdf");

	Alembic::AbcMaterial::addMaterialAssignment(meshNode, "meshy/material");
	materialsNode = Alembic::Abc::OObject(meshNode, "materialTest");

	setFloatParameter(material.getSchema(), "abc", "surface", "specular", 1.0f);*/
	//meshNode = Alembic::AbcGeom::v12::OPolyMesh(xform, "meshy", g_ts);


	mesh.setUVSourceName("yeet");

	colourProps = Alembic::AbcGeom::OC3fGeomParam(mesh.getArbGeomParams(), "Test Colour",
		true, Alembic::AbcGeom::v12::GeometryScope::kVertexScope, 1);
	

	
	/*
	materialsNode = Alembic::Abc::OObject(top, "materials");
	Alembic::AbcMaterial::OMaterial materialA(materialsNode, "materialA");
	materialA.getSchema().setShader("unity", "surface", "standard");
	//setFloatParameter(materialA.getSchema(), "opengl", "surface", "roughness", 0.1f);
	
	
	//Alembic::AbcMaterial::addMaterialAssignment(meshNode, "/materials/material");
	*/
	
}

void AlembicWriter::saveFrame(struct AlembicMeshData meshData) {
	Alembic::AbcGeom::v12::OV2fGeomParam::Sample UVs(Alembic::AbcGeom::v12::V2fArraySample((const Alembic::AbcGeom::v12::V2f*)meshData.uvs.data(),
		meshData.numUvs), Alembic::AbcGeom::v12::kFacevaryingScope);


	Alembic::AbcGeom::v12::ON3fGeomParam::Sample normals(Alembic::AbcGeom::v12::N3fArraySample((const Alembic::AbcGeom::v12::N3f*)meshData.normals.data(),
		meshData.numNormals),
		Alembic::AbcGeom::v12::kFacevaryingScope);


	Alembic::AbcGeom::OPolyMeshSchema::Sample g_meshsamp = Alembic::AbcGeom::OPolyMeshSchema::Sample(
		Alembic::AbcGeom::v12::V3fArraySample((const Alembic::AbcGeom::v12::V3f*)meshData.vertices.data(), meshData.numVerts),
		Alembic::AbcGeom::v12::Int32ArraySample(meshData.indicies.data(), meshData.numIndicies),
		Alembic::AbcGeom::v12::Int32ArraySample(meshData.counts.data(), meshData.numCounts));

	
	/*
	Alembic::AbcGeom::OC3fGeomParam::Sample colourParam(Alembic::AbcGeom::C3fArraySample(meshData.vertexColours),
		Alembic::AbcGeom::v12::GeometryScope::kVertexScope);
	*/

	

	//Alembic::AbcMaterial::addMaterialAssignment(mesh, "/materials/materialA");
	//colourProps.set(colourParam);
	
	g_meshsamp.setNormals(normals);

	g_meshsamp.setUVs(UVs);
	
	mesh.set(g_meshsamp);

	
	
}