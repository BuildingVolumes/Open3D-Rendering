import os
from PIL import Image #https://pillow.readthedocs.io/en/stable/
import argparse


def parseCommandLine():
    parser = argparse.ArgumentParser(description="Converts a folder of pictures into another file type")
    parser.add_argument("inDir", metavar="inDir", type=str, help="The folder that contains the pictures you want to convert")
    parser.add_argument("outDir", metavar="outDir", type=str, help="The folder to save the converted files")
    parser.add_argument("type", metavar="type", type=str, help="The file type to convert to")

    args = parser.parse_args()
    return args

def main():
    arguments = parseCommandLine()

    print("Input Dir is ", arguments.inDir)
    print("Output Dir is ", arguments.outDir)
    print("File Type is ", arguments.type)
    for filename in filter(os.path.isfile, os.listdir(arguments.inDir)):
        
        fileParts = filename.split('.')
        print(fileParts)
        if(fileParts[1] == "tiff"):
            tiff = Image.open(filename)
            tiff.save(arguments.outDir + fileParts[0] + "." + arguments.type, arguments.type)

main()


