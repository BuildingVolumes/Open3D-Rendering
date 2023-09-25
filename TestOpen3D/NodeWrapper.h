#pragma once

#include "EncodeDecodeManager.h"
#include "CameraManager.h"

using namespace::MKV_Rendering;

class NodeWrapper
{
	//TODO: mild change to camera manager, make it allow taking it folders after it's been created
	//TODO: make a command line based system to allow us to efficiently perform any functionality implemented thus far

	CameraManager *cm;

	EncDec::EncodeDecodeManager* edm;

	VoxelGridData* vgd;

	int specsLength = 0;

	std::vector<std::string> pseudoSpecs;

	void WriteOBJ(std::string filename, std::string filepath, open3d::geometry::TriangleMesh* mesh);

public:
	NodeWrapper();
	~NodeWrapper();

	void PrintHelp();

	void PerformOperations(int maxSpecs, char** specs);

protected:
	int MakeEmptyTXT(int currentSpec);

	void DebugLine(std::string text);

	int MakeOBJ(int startingLoc);

	int TextureOBJ(int startingLoc);

	int LoadDataLivescan(int startingLoc);

	int AddCameraLivescan(int startingLoc);

	int LoadDataStructure(int startingLoc);

	int Unload(int startingLoc);

	int SetVoxelGridData(int startingLoc);

	int EnableCamera(int startingLoc, bool enable);

	int CleanupMeshPoisson(int startingLoc);

	int EncodeMesh(int startingLoc);

	int DecodeMesh(int startingLoc);

	int MortonEncodeMesh(int startingLoc);

	int MortonDecodeMesh(int startingLoc);
};