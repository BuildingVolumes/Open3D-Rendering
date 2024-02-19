#pragma once

//#define PLAYGROUND_CODE_ENABLED 1
#define PLAYGROUND_CODE_ENABLED 0

#include "CGAL_TestSuite.h"
#include "DracoEXE_TestSuite.h"
#include "TestFFT.h"
#include "PointCloudSuite.h"
#include "TestZIP.h"
#include "TestMultiMesh.h"
#include "ErrorMetricSuite.h"
#include "VoxelEncodeDecodeSuite.h"
#include "VDMC_TestSuite.h"
#include "CleanupSuite.h"
#include "VV_MeshSuite.h"

#if PLAYGROUND_CODE_ENABLED == 1
#include "CGAL_HoleFilling.h"
#endif

class SuiteMaster
{
public:
	void run(int argc, char** argv)
	{
#if PLAYGROUND_CODE_ENABLED == 1
        TestPlaygroundCode();
        return;
#elif PLAYGROUND_CODE_ENABLED == 0
        TestSuite* suite = nullptr;

        //suite = new TestFFT();
        //suite = new TestMultiMesh();
        //suite = new AlembicExportSuite();
        //suite = new PointCloudSuite();
        //suite = new TestZIP();
        //suite = new CGAL_TestSuite();
        //suite = new DracoEXE_TestSuite();
        //suite = new ErrorMetricSuite();
        //suite = new VoxelEncodeDecodeSuite();
        //suite = new VDMC_TestSuite();
        //suite = new CleanupSuite();
        suite = new VV_MeshSuite();

        suite->run(argc, argv);

        delete suite;
#endif
	}

#if PLAYGROUND_CODE_ENABLED == 1
    void TestPlaygroundCode()
    {
        CGAL_HoleFilling cgalholefilling;

        cgalholefilling.PseudoMain();
    }
#endif
};