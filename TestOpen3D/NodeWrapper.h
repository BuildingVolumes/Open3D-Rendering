#pragma once

#include "CameraManager.h"

using namespace::MKV_Rendering;

class NodeWrapper
{
	//TODO: mild change to camera manager, make it allow taking it folders after it's been created
	//TODO: make a command line based system to allow us to efficiently perform any functionality implemented thus far

	CameraManager *cm;

	VoxelGridData* vgd;

	int specsLength = 0;

	void WriteOBJ(std::string filename, std::string filepath, open3d::geometry::TriangleMesh* mesh);

public:
	NodeWrapper();
	~NodeWrapper();

	void PrintHelp();

	void PerformOperations(int maxSpecs, char** specs);

protected:
	int MakeOBJ(char** specs, int startingLoc);

	int TextureOBJ(char** specs, int startingLoc);

	int LoadDataLivescan(char** specs, int startingLoc, bool useMattes);

	int LoadDataStructure(char** specs, int startingLoc);

	int Unload(char** specs, int startingLoc);

	int SetVoxelGridData(char** specs, int startingLoc);

	int EnableCamera(char** specs, int startingLoc, bool enable);
};