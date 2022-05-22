# -*- coding: utf-8 -*-
from optparse import OptionParser
import os
import os.path
import glob
import cv2
import json
import sys
import codecs
import caffe
import numpy as np
from PIL import Image
import scipy.misc as sci
from smartEyeTest.evalution import *
import smartEyeEvalution
reload(sys)
sys.setdefaultencoding('utf-8')

def saveLDResult(outputFile, resultLD):
    count = len(resultLD)
    if count > 0:
        outputFile.write("{\"LD\":{\"ldCount\":%d,\"alarm\":%d,\"ldResult\":[" % (count, 0))
        for index, point in enumerate(resultLD):
            if index != 0:
                outputFile.write(",")
            outputFile.write("{\"x\":%d,\"y\":%d,\"position\":%d}" % (point.x, point.y, 0))
        outputFile.write("]}}\n")
    else:
        outputFile.write("{\"LD\":{\"ldCount\":0,\"alarm\":0,\"ldResult\":[]}}\n")

def saveFCResult(outputFile, resultFCW):
    count = len(resultFCW)
    if count > 0:
        outputFile.write("{\"FC\":{\"fcCount\":%d,\"fcResult\":[" % count)
        for index, box in enumerate(resultFCW):
            if index != 0:
                outputFile.write(",")
            outputFile.write("{\"x\":%d,\"y\":%d,\"wid\":%d,\"hig\":%d,\"confidence\":%d}"
                             % (box.min_corner.x, box.min_corner.y, box.width(), box.height(), box.confidence))
        outputFile.write("]}}\n")
    else:
        outputFile.write("{\"FC\":{\"fcCount\":0,\"fcResult\":[]}}\n")

def saveCNNResult(resultPath, resultDatas):
    with open(resultPath, 'w') as outputFile:
        for resultNumber, resultLD, resultFCW in resultDatas:
            outputFile.write("freameNo:%d\n" % resultNumber)
            saveLDResult(outputFile, resultLD)
            saveFCResult(outputFile, resultFCW)

def caffeInit(netPath, modelPath, meanFile):
    net = caffe.Net(netPath, modelPath, caffe.TEST)
    caffe.set_device(0)
    caffe.set_mode_gpu()
    mean_blob = caffe.proto.caffe_pb2.BlobProto()
    mean_blob.ParseFromString(open(meanFile, 'rb').read())
    caffeMean = caffe.io.blobproto_to_array(mean_blob)
    return net, caffeMean

def testCaffeModel(videoPath, resultDatas, netPath, modelPath, meanFile):
    result = []
    caffeNet, caffeMean = caffeInit(netPath, modelPath, meanFile)
    #transformer = caffe.io.Transformer({'data': caffeNet.blobs['data'].data.shape})
    #transformer.set_transpose('data', (2, 0, 1))
    #transformer.set_mean('data', caffeMean[0, :, 0, 0])
    #transformer.set_raw_scale('data', 255)
    #transformer.set_channel_swap('data', (2,1,0))
    capture = cv2.VideoCapture(videoPath)
    if capture.isOpened():
        for resultNumber, resultLD, resultFCW in resultDatas:
            #print("frame number:", resultNumber)
            capture.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, resultNumber - 1)
            ret, frame = capture.read()
            grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            inputFrame = cv2.resize(grayFrame, (smartEyeEvalution.IMAGE_WIDTH, smartEyeEvalution.IMAGE_HEIGHT))
            if ret == False:
                print("video read end!")
                break
            else:
                resultBox = []
                for box in resultFCW:
                    image = inputFrame[box.min_corner.y:box.max_corner.y+1, box.min_corner.x:box.max_corner.x+1]
                    resizeImage = cv2.resize(image, (25, 25))
                    tempImage = np.array(resizeImage, dtype=np.float32)
                    tempImage -= np.array(caffeMean[0, :, 0, 0])
                    #tempImage = tempImage.transpose((2,0,1))
                    caffeNet.blobs['data'].data[...] = tempImage
                    caffeNet.forward()
                    prob = caffeNet.blobs['prob'].data[0]
                    if prob.argmax() == 1:
                        resultBox.append(box)
                result.append((resultNumber, resultLD, resultBox))
    return result

def smartEyeCNNEvalution(videoPath):

    path, file_name_and_post = os.path.split(videoPath)
    videoName, post = os.path.splitext(file_name_and_post)
    resultName = videoName + "_smartEye.txt"
    resultDir = os.path.join(path, "../result/")
    resultPath = os.path.join(resultDir, resultName)

    resultDatas = list(readSmartEyeResult(resultPath, \
                        smartEyeEvalution.MIN_WIDTH, smartEyeEvalution.MIN_HEIGHT, 0))

    resultDatas = testCaffeModel(videoPath, resultDatas, "/home/lipj/testCNN/car_model/deploy.prototxt", "/home/lipj/testCNN/car_model/fc-car_iter_200000_Day5.caffemodel", "/home/lipj/caffe-ssd/data/ilsvrc12/imagenet_mean.binaryproto")

    saveCNNResult(resultPath, resultDatas)

def main():
    print("start...")
    options = smartEyeEvalution.parse_arguments()
    smartEyeCNNEvalution(options.input_path)
    print("process end!")

if __name__ == "__main__":
    main()
