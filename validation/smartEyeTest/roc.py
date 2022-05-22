# -*- coding: utf-8 -*-
import os
import os.path
import glob
import json
import codecs

def readEvalutionResult(inputPath):
    recall = []
    pecision = []
    try:
        with open(inputPath, 'r') as textFile:
            for line in textFile:
                jsonData = json.loads(line)
                pecision.append(float(jsonData["pecision"]))
                recall.append(float(jsonData["recall"]))
    except IOError:
        return pecision, recall
    return pecision, recall

def readStatDetection(inputPath):
    width = []
    confidence = []
    try:
        with open(inputPath, 'r') as textFile:
            for line in textFile:
                jsonData = json.loads(line)
                width.append(int(jsonData["width"]))
                confidence.append(int(jsonData["confidence"]))
    except IOError:
        return width, confidence
    return width, confidence