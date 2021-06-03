#pragma once
#include "AlembicWriter.h"

AlembicWriter::AlembicWriter(std::string fileName, std::string topName, float start, float delta) {
	startTime = start;
	deltaTime = delta;
	g_ts = Alembic::AbcGeom::TimeSamplingPtr(new Alembic::AbcGeom::TimeSampling(deltaTime, startTime));

	this->fileName = fileName;
	this->topName = topName;

	archive = Alembic::Abc::OArchive(Alembic::AbcCoreOgawa::WriteArchive(), this->fileName);
	top = archive.getTop();
	Alembic::AbcGeom::v12::OXform xform(top, this->topName, g_ts);
	meshNode = Alembic::AbcGeom::v12::OPolyMesh(xform, "meshy", g_ts);
	mesh = meshNode.getSchema();
}

void AlembicWriter::saveFrame(struct AlembicMeshData meshData) {
	Alembic::AbcGeom::OPolyMeshSchema::Sample g_meshsamp = Alembic::AbcGeom::OPolyMeshSchema::Sample(
		Alembic::AbcGeom::v12::V3fArraySample((const Alembic::AbcGeom::v12::V3f*)meshData.vertices.data(), meshData.numVerts),
		Alembic::AbcGeom::v12::Int32ArraySample(meshData.indicies.data(), meshData.numIndicies),
		Alembic::AbcGeom::v12::Int32ArraySample(meshData.counts.data(), meshData.numCounts));

	Alembic::AbcGeom::v12::ON3fGeomParam::Sample normals(Alembic::AbcGeom::v12::N3fArraySample((const Alembic::AbcGeom::v12::N3f*)meshData.normals.data(),
		meshData.numNormals),
		Alembic::AbcGeom::v12::kFacevaryingScope);
	g_meshsamp.setNormals(normals);

	mesh.set(g_meshsamp);
}