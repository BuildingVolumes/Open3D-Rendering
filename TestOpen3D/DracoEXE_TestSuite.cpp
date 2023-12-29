#include "DracoEXE_TestSuite.h"

void DracoEXE_TestSuite::PrecursorWrapperTest()
{
	std::string primary_meshes_folder = "D:\\VsprojectsOnD\\Open3D\\TestOpen3D\\TestOpen3D\\TestOpen3D\\_AUXILIARY_TEST_DATA\\VolumetricSOAR\\AB-all\\AB-1punch";
	std::string encoded_meshes_folder = "D:\\VsprojectsOnD\\Open3D\\TestOpen3D\\TestOpen3D\\TestOpen3D\\_MeshDump\\_CompressionDump";
	std::string decoded_meshes_folder = "D:\\VsprojectsOnD\\Open3D\\TestOpen3D\\TestOpen3D\\TestOpen3D\\_MeshDump\\_DecompressionDump";

	std::string primary_ext = ".obj";
	std::string comp_ext = ".drc";

	std::string comp_name = "DracoEnc";
	std::string decomp_name = "DracoDec";

	dw.MassCompressMeshes(primary_meshes_folder, encoded_meshes_folder, primary_ext, comp_name, comp_ext);

	dw.MassDecompressMeshes(encoded_meshes_folder, decoded_meshes_folder, comp_ext, decomp_name, primary_ext);
}

void DracoEXE_TestSuite::run(int argc, char** argv)
{
	PrecursorWrapperTest();
}
