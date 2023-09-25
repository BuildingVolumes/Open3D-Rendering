#include "AlembicExportSuite.h"

#include "AdditionalUtilities.h"

#include <Alembic/Abc/All.h>
#include <Alembic/AbcGeom/All.h>
#include <Alembic/AbcCoreOgawa/All.h>
#include <math.h>
#include <Alembic/AbcMaterial/MaterialAssignment.h>

float* AlembicExportSuite::DoubleArrayToFloatArray(double* elems, size_t count)
{
	float* float_arr = new float[count];

#pragma omp parallel
#pragma omp for
	for (int i = 0; i < count; ++i)
	{
		float_arr[i] = elems[i];
	}

	return float_arr;
}

AlembicExportSuite::AlembicExportSuite()
{

}

void AlembicExportSuite::run(int argc, char** argv)
{
	root = "D:\\_VV_DATASETS\\RAFA\\Rafa_Approves_hd_4k";
	tex_handle = "";
	mesh_handle = "";

	SynthesizeAlembic(root, tex_handle, mesh_handle);
}

void AlembicExportSuite::SynthesizeAlembic(std::string root_folder, std::string texture_names, std::string mesh_names)
{
	std::string alembic_file = "TEST_ALEMBIC.abc";

	auto filenames = GetFiles(root_folder);

	mesh_files.clear();
	texture_files.clear();

	std::vector<std::string> split_string;

	for (int i = 0; i < filenames.size(); ++i)
	{
		SplitString(filenames[i], split_string, ".", "", true);

		std::string parser = split_string.back();

		if (parser == std::string("obj"))
		{
			mesh_files.push_back(filenames[i]);
		}
		else if (parser == std::string("jpg") || parser == std::string("png"))
		{
			texture_files.push_back(filenames[i]);
		}
	}


	Alembic::AbcCoreOgawa::WriteArchive archiveWriter;

	auto archive = Alembic::Abc::OArchive(archiveWriter, "TestAlembic.abc");

	open3d::geometry::TriangleMesh single_frame;
	open3d::io::ReadTriangleMeshOptions ops;

	//ops.enable_post_processing = false;
	//ops.print_progress = false;
	//ops.update_progress = false;

	uint64_t time_sampling = 200;

	for (const auto& mesh : mesh_files)
	{
		open3d::io::ReadTriangleMeshFromOBJ(mesh, single_frame, ops);

		// Create an object for the mesh
		Alembic::AbcGeom::OPolyMesh mesh_obj(archive.getTop(), "SomeMesh");// , Alembic::AbcGeom::OObject::kWrapExisting);

		// Create a mesh schema for the mesh object
		Alembic::AbcGeom::OPolyMeshSchema mesh_schema = mesh_obj.getSchema();

		// Set the time sampling for the mesh schema
		mesh_schema.setTimeSampling(time_sampling);

		// Set the vertex positions for the mesh schema
		Alembic::AbcGeom::OPolyMeshSchema::Sample mesh_sample;

		auto position_array = DoubleArrayToFloatArray(reinterpret_cast<double*>(single_frame.vertices_.data()), single_frame.vertices_.size());

		mesh_sample.setPositions(Alembic::AbcGeom::P3fArraySample(
			reinterpret_cast<const Alembic::AbcGeom::V3f*>(position_array), single_frame.vertices_.size()
		));

		delete[] position_array;

		mesh_sample.setFaceIndices(Alembic::AbcGeom::Int32ArraySample(
			reinterpret_cast<int*>(single_frame.triangles_.data()),
			single_frame.triangles_.size()
		));

		mesh_schema.set(mesh_sample);
	}
}

