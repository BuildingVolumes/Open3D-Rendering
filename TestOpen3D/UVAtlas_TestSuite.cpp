#include "UVAtlas_TestSuite.h"

void UVAtlasTestSuite::run(int argc, char** argv)
{
	//DirectX::UVAtlasCreate();

	if (!vv_m.ReadOBJ(mesh_path))
	{
		return;
	}

	int max_chart_number = 10;
	size_t width = 1024;
	size_t height = 1024;
	float gutter = 2.0f;

	//DirectX::UVAtlasCreate();
}
