import sys, os
import argparse


def createAlembic(inputDir, imageDir, structFileName,liveScanFolder, outputDir, binDir, vgData):
    print("Running Alembic...")

    binName = binDir + "\\TestOpen3D.exe"
    
    commandLine = binName
    commandLine += " \""
    commandLine += inputDir
    commandLine += "\" \""
    commandLine += imageDir
    commandLine += "\" \""
    commandLine += structFileName
    commandLine += "\" \""
    commandLine += liveScanFolder
    commandLine += "\" \""
    commandLine += outputDir
    commandLine += "\""
    for key in vgData:
        commandLine += " "
        commandLine += key 
        commandLine += " " 
        commandLine += str(vgData[key])
        
    
    print("\n")
    print(commandLine)
    
    os.system(commandLine)
    

def parseCommandLine():
    parser = argparse.ArgumentParser(description="An Volumetric Renderer software")
    parser.add_argument("inDir", metavar="inDir", type=str, help="The folder that contains the input data")
    parser.add_argument("imageDir", metavar="imageDir", type=str, help="The folder that contains the images")
    parser.add_argument("structName", metavar="Structure Name", type=str, help="name of the structure file")
    parser.add_argument("liveScan", metavar="Live Scan Folder", type=str, help="Location of the live scan data")
    parser.add_argument("outDir", metavar="Output Dir", type=str, help="Where to output the data")
    parser.add_argument("binDir", metavar="Bin Dir", type=str, help="Where the open3d.exe file is")
    parser.add_argument("pipeLineStep", metavar="Pipe Line Step", type=str, help="What you're wanting to use for the pipe line")
    parser.add_argument("-t", default=1999999, metavar="timestamp", type=int, help="Where to start the video")
    parser.add_argument("-vgBlock",default=1000, metavar="Voxel Grid Block", type=int, help="How large are the blocks of the voxel grid")
    parser.add_argument("-vgSize",default=0.00586, metavar="Voxel Grid Size", type=float, help="How large are the voxels")
    parser.add_argument("-vgScale", default=1000.0, metavar="Voxel Grid Depth Scale", type=float, help="What is the depth scale")
    parser.add_argument("-vgMax",default=3.0, metavar="Voxel Grid Depth Max", type=float, help="What is the max depth")
    parser.add_argument("-vgTrunc", default=0.04, metavar="Voxel Grid Truncation", type=float, help="What is the field truncation")
    parser.add_argument("-vgDVCode", default="CPU:0", metavar="Voxel Grid Device Code", type=str, help="What is the device code")
    args = parser.parse_args()
    return args

def main():
    args = parseCommandLine()
    
    print(args.vgDVCode)

    vgData = {"-vgBlock": args.vgBlock, "-vgSize": args.vgSize, "-vgScale": args.vgScale, "-vgMax": args.vgMax, "-vgTrunc": args.vgTrunc, "-vgDVCode": args.vgDVCode}
    print("Running Open3D")
    if(len(sys.argv) != 8): #Remeber that runOpen3d.py is the 0th parameter

        print("Usage: python runOpen3d.py <inputDir> <imageFolder> <structureFileName> <live_scan_folder> <outputDir> <binDir> <pipeLineStep>")
        print("Error: Must have 7 parameters. Number of paremeters currently: ", len(sys.argv))
        #print("arguments: ", sys.argv)
        sys.exit(0)

    inputDir = args.inDir #Where are the images or mkvs are located?
    imageFolder = args.imageDir # Are we using images or MKVs?
    structFileName = args.structName #Name of the sturcture files
    liveScanFolder = args.liveScan #What part of the open3d code do we want to run?
    outputDir = args.outDir #Where do we want to output the data?
    binDir = args.binDir #Where is the Open3D executeable?
    pipeLineStep = args.pipeLineStep
    
    #optional paremeters
    
    


    print("Input Dir: ", inputDir)
    print("Image Folder: ", imageFolder)
    print("Structure File name: ", structFileName)
    print("Live Scan Folder: ", liveScanFolder)
    print("Output Dir: ", outputDir)
    print("bin Dir:", binDir)
    print("Pipe Line Step:", pipeLineStep)
    if(pipeLineStep == 'a'):
        createAlembic(inputDir, imageFolder, structFileName, liveScanFolder, outputDir, binDir, vgData)

main()