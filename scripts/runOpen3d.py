import sys, os
from dataclasses import dataclass


@dataclass
class VoxelGridData:
    blocks : int
    voxel_size : float
    depth_scale : float
    depth_max : float
    signed_distance_field_trucation : float

    device_code : str
    


def createAlembic(inputDir, structFileName, outputDir, binDir):
    print("Running Alembic...")

    binName = binDir + "\\TestOpen3D.exe"

    commandLine = binName

    os.system(commandLine)


def main():
    print("Running Open3D")
    print(sys.argv)
    if(len(sys.argv) != 7): #Remeber that runOpen3d.py is the 0th parameter

        print("Usage: python runOpen3d.py <inputDir> <useImage> <structureFileName> <step> <outputDir> <binDir>")
        print("Error: Must have 6 parameters")
        sys.exit(0)

    inputDir = sys.argv[1] #Where are the images or mkvs are located?
    useImage = sys.argv[2] # Are we using images or MKVs?
    structFileName = sys.argv[3] #Name of the sturcture files
    step = sys.argv[4] #What part of the open3d code do we want to run?
    outputDir = sys.argv[5] #Where do we want to output the data?
    binDir = sys.argv[6] #Where is the Open3D executeable?

    print("Input Dir: ", inputDir)
    print("Use Image: ", useImage)
    print("Structure File name: ", structFileName)
    print("Step: ", step)
    print("Output Dir: ", outputDir)
    print("bin Dir:", binDir)

    createAlembic(inputDir, structFileName, outputDir, binDir)

main()