# -*- coding: utf-8 -*-
from optparse import OptionParser
import os
import os.path
import glob
import json
import codecs
import matplotlib.pyplot as plt
import numpy as np

def parse_arguments():

    parser = OptionParser()
    parser.description = \
        "This program takes the smartEye test datas"

    parser.add_option("-i", "--input", dest="input_path",
                      metavar="PATH", type="string",
                      help="path to the "
                      "wissen dataset folder")

    parser.add_option("-o", "--output", dest="output_path",
                      metavar="DIRECTORY", type="string",
                      help="path to a non existing directory"
                      "where the new training dataset will be created")

    (options, args) = parser.parse_args()
    #print (options, args)

    if options.input_path:
        if not os.path.exists(options.input_path):
            parser.error("Could not find the input file")
        else:
            # we normalize the path
            options.input_path = os.path.normpath(options.input_path)
    else:
        parser.error("'input' option is required to run this program")

    return options

def computeEvalution(inputPath):
    truthObjectCount = 0
    detectObjectCount = 0
    truePositives = 0
    falseNegative = 0
    try:
        with open(inputPath, 'r') as textFile:
            for line in textFile:
                jsonData = json.loads(line)
                fcwResultData = jsonData["FCW"]
                truthObjectCount += int(fcwResultData["truthCount"])
                detectObjectCount += int(fcwResultData["detectCount"])
                truePositives += int(fcwResultData["TP"])
                falseNegative += int(fcwResultData["FN"])
        pecision = 0
        recall = 0
        if detectObjectCount != 0:
            pecision = float(truePositives) / detectObjectCount
        if truePositives != 0:
            recall = 1 - float(falseNegative) / truthObjectCount
        result = {}
        result["pecision"] = pecision
        result["recall"] = recall
        with codecs.open('totalResult.txt', 'a+', 'utf-8') as f:
            f.write(json.dumps(result, ensure_ascii=False))
            f.write("\n")
    except IOError:
        return
    return

def main():
    print("start...")
    options = parse_arguments()
    computeEvalution(options.input_path)
    print("process end!")

if __name__ == "__main__":
    main()