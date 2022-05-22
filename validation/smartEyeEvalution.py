# -*- coding: utf-8 -*-
from optparse import OptionParser
import os
import os.path
import glob
import cv2
import json
import sys
import codecs
from smartEyeTest.evalution import *
reload(sys)
sys.setdefaultencoding('utf-8')

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
VIDEO_RATIO = 1.5

MIN_WIDTH = 40
MIN_HEIGHT = 40

LD_MIN_Y = 384 * VIDEO_RATIO
LD_MAX_Y = 650 * VIDEO_RATIO

def parse_arguments():

    parser = OptionParser()
    parser.description = \
        "This program takes the smartEye test datas"

    parser.add_option("-i", "--input", dest="input_path",
                      metavar="PATH", type="string",
                      help="path to the "
                      "wissen dataset folder")

    parser.add_option("-f", "--threshold", dest="threshold",
                      type="int", default=0,
                      help="detecttion threshold")

    parser.add_option("-p", "--post", action="store", dest="video_post",
                      type="string", default="*.avi", help="video post")

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

def readGt1(videoPath):
    resultDatas = []
    dataPath = unicode(videoPath, "utf-8")
    path, file_name_and_post = os.path.split(dataPath)
    videoName, post = os.path.splitext(file_name_and_post)

    #gt.txt
    gtFileName = videoName + "_gt1.txt"
    gtPath = os.path.join(path, gtFileName)
    gtDatas = readGtValue(gtPath)

    if gtDatas:
        resultDatas = list(gtDatas)

    return resultDatas

def filterFCWTruthValue(FCWTruthValueDatas):
    for frameNumber, boxes in FCWTruthValueDatas:
        result = []
        for box in boxes:
            box.min_corner.x = int(box.min_corner.x / VIDEO_RATIO)
            box.min_corner.y = int(box.min_corner.y / VIDEO_RATIO)
            box.max_corner.x = int(box.max_corner.x/ VIDEO_RATIO)
            box.max_corner.y = int(box.max_corner.y / VIDEO_RATIO)
            width = box.max_corner.x - box.min_corner.x
            height = box.max_corner.y - box.min_corner.y
            if width >= MIN_WIDTH and height >= MIN_HEIGHT:
                result.append(box)
        if frameNumber >= 0:
            yield (frameNumber, result)
    return

def filterLDTruthValue(LDTruthValueDatas):
    for frameNumber, points in LDTruthValueDatas:
        result = []
        for point in points:
            point.x = int(point.x / VIDEO_RATIO)
            point.y = int(point.y / VIDEO_RATIO)
            result.append(point)
        if frameNumber >= 0:
            yield (frameNumber, result)
    return

def testSmartEyeResult(videoPath, resultPath):
    capture = cv2.VideoCapture(videoPath)
    if capture.isOpened():
        for frameNumber, points, boxes in readSmartEyeResult(resultPath, MIN_WIDTH, MIN_HEIGHT):
            capture.set(cv2.CAP_PROP_POS_FRAMES, frameNumber - 1)
            ret, frame = capture.read()
            if ret == False:
                print("video read end!")
                break
            else:
                #frame = frame[0:1080, 240: 240 + 1440]
                resized_image = cv2.resize(frame, (IMAGE_WIDTH, IMAGE_HEIGHT))
                showPoints(points, resized_image, 8, (0, 255, 0))
                showObjects(boxes, resized_image, (0, 0, 255), 2)
                cv2.imshow("video", resized_image)
                if cv2.waitKey(0) & 0xFF == 27:
                    break
            print(frameNumber)
    else:
        print("open video:%s fail!" % videoPath)
    capture.release()
    cv2.destroyAllWindows()

def compareSmartEyeResult(videoPath, resultPath1, resultPath2, flag):
    index = 0
    capture = cv2.VideoCapture(videoPath)
    resultData2 = []
    if flag == 0:
        resultData2 = list(readSmartEyeDetSlaverResult(resultPath2, MIN_WIDTH, MIN_HEIGHT, 0))
    elif flag == 1:
        resultData2 = list(readSmartEyeDetSlaverResult(resultPath2, MIN_WIDTH, MIN_HEIGHT))
    if capture.isOpened():
        for frameNumber, points, boxes in readSmartEyeResult(resultPath1, MIN_WIDTH, MIN_HEIGHT):
            result2Number, points2, detBoxes2, boxes2 = resultData2[index]
            if result2Number == frameNumber:
                print(frameNumber)
                capture.set(cv2.CAP_PROP_POS_FRAMES, frameNumber - 1)
                ret, frame = capture.read()
                if ret == False:
                    print("video read end!")
                    break
                else:
                    #frame = frame[0:1080, 240: 240 + 1440]
                    resized_image = cv2.resize(frame, (IMAGE_WIDTH, IMAGE_HEIGHT))
                    showPoints(points, resized_image, 12, (0, 255, 0))
                    showObjects(boxes, resized_image, (0, 0, 255), 2)
                    showPoints(points2, resized_image, 8, (255, 0, 0))
                    #showObjects(detBoxes2, resized_image, (0, 255, 255), 1)
                    showObjects(boxes2, resized_image, (0, 255, 255), 1)
                    cv2.imshow("video", resized_image)
                    if cv2.waitKey(0) & 0xFF == 27:
                        break
                index += 1
    else:
        print("open video:%s fail!" % videoPath)
    capture.release()
    cv2.destroyAllWindows()

def smartEyeEvalution(videoPath, threshold, isSaveVideo):

    path, file_name_and_post = os.path.split(videoPath)
    videoName, post = os.path.splitext(file_name_and_post)
    ldPath = os.path.join(path , "%s_ld.txt" % videoName)
    resultName = videoName + "_smartEye.txt"
    resultDir = os.path.join(path, "../result/")
    resultPath = os.path.join(resultDir, resultName)

    FCWTruthValueDatas = readGt1(videoPath)
    #FCWTruthValueDatas = readFCWTruthValue(videoPath)
    LDTruthValueDatas = readLDTruthValue(ldPath, LD_MIN_Y, LD_MAX_Y)
    FCWDatas = list(filterFCWTruthValue(FCWTruthValueDatas))
    LDDatas = list(filterLDTruthValue(LDTruthValueDatas))
    truthDatas = map(mergeLDAndFCW, LDDatas, FCWDatas)
    resultDatas = list(readSmartEyeResult(resultPath, MIN_WIDTH, MIN_HEIGHT, threshold))

    videoEvalution(videoPath, truthDatas, resultDatas, IMAGE_WIDTH, IMAGE_HEIGHT, isSaveVideo)

def slaversmartEyeEvalution(videoPath, isSaveVideo):

    path, file_name_and_post = os.path.split(videoPath)
    videoName, post = os.path.splitext(file_name_and_post)
    ldPath = os.path.join(path, "%s_ld.txt" % videoName)
    resultDir = os.path.join(path, "../result/")
    resultPath = os.path.join(resultDir, "%s.txt" % videoName)

    FCWTruthValueDatas = readGt1(videoPath)
    # FCWTruthValueDatas = readFCWTruthValue(videoPath)
    LDTruthValueDatas = readLDTruthValue(ldPath, LD_MIN_Y, LD_MAX_Y)
    FCWDatas = list(filterFCWTruthValue(FCWTruthValueDatas))
    LDDatas = list(filterLDTruthValue(LDTruthValueDatas))
    truthDatas = map(mergeLDAndFCW, LDDatas, FCWDatas)
    resultDatas = list(readSmartEyeSlaverResult(resultPath, MIN_WIDTH, MIN_HEIGHT))

    videoEvalution(videoPath, truthDatas, resultDatas, IMAGE_WIDTH, IMAGE_HEIGHT, isSaveVideo)

def main():
    print("start...")
    options = parse_arguments()
    #testSmartEyeResult("E:\\QtProject\\DetectionProposals\\video\\20180201152018_0.avi",
    #                 "C:\\Users\\Administrator\\Desktop\\result.txt")
    #compareSmartEyeResult("E:\\QtProject\\DetectionProposals\\video\\20180201152018_0.avi",
    #                    "E:\\VS2013Project\\testSmartEye\\testSmartEye\\result\\20180201152018_0_smartEye.txt",
    #                   "C:\\Users\\Administrator\\Desktop\\result.txt")
    smartEyeEvalution(options.input_path, options.threshold, 0)
    #slaversmartEyeEvalution(options.input_path, 1)
    print("process end!")

if __name__ == "__main__":
    main()