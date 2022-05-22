# -*- coding: utf-8 -*-
from optparse import OptionParser
import os
import os.path
import glob
import codecs
import matplotlib.pyplot as plt
import numpy as np
from smartEyeTest.roc import *

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

def drawROC(inputPath):
    pecision, recall= readEvalutionResult(inputPath)
    drawLine = [(0, 1), (0, 1)]

    plt.title('ROC')
    plt.plot(recall, pecision, linewidth=2, color='r', marker='o')
    plt.plot(drawLine[0], drawLine[1], linewidth=1, color='b')
    plt.xlabel('recall')
    plt.ylabel('pecision')
    plt.xlim(0.0, 1.0)  # set axis limits
    plt.ylim(0.0, 1.0)

    plt.savefig('./ROC.jpg')
    plt.show()

def main():
    print("start...")
    options = parse_arguments()
    drawROC(options.input_path)
    print("process end!")

if __name__ == "__main__":
    main()