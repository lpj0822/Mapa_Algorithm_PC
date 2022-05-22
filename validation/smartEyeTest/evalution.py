# -*- coding: utf-8 -*-
import os
import os.path
import glob
import cv2
import json
import sys
import codecs
reload(sys)
sys.setdefaultencoding('utf-8')

MINIMAL_OVERLAP_THRESHOLD = 0.45

class Point2d:
    """
    Helper class to define Box
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y
        return

class Box:
    """
        Helper Box class
    """
    def __init__(self):
        self.min_corner = Point2d(0, 0)
        self.max_corner = Point2d(0, 0)
        self.confidence = 0
        return

    def copy(self):
        b = Box()
        b.min_corner = self.min_corner
        b.max_corner = self.max_corner
        return b

    def width(self):
        return self.max_corner.x - self.min_corner.x

    def height(self):
        return self.max_corner.y - self.min_corner.y

    def __eq__(self, other):
        return self.min_corner.x == other.min_corner.x and \
                self.min_corner.y == other.min_corner.y and \
               self.max_corner.x == other.max_corner.x and \
               self.max_corner.y == other.max_corner.y

def showObjects(boxes, image, color, lineWidth):
    for box in boxes:
        p1 = (int(box.min_corner.x), int(box.min_corner.y))
        p2 = (int(box.max_corner.x), int(box.max_corner.y))
        cv2.rectangle(image, p1, p2, color, lineWidth)

def showPoints(points, image, length, color):
    for point in points:
        p1 = (point.x - length, point.y)
        p2 = (point.x + length, point.y)
        cv2.line(image, p1, p2, color, 2)

def showLines(points, image, color):
    lenght = len(points)
    if lenght % 2 == 0:
        for index in range(0, lenght, 2):
            point1 = points[index]
            point2 = points[index + 1]
            cv2.line(image, (point1.x, point1.y), (point2.x, point2.y), color, 2)

def overlapping_area(box1, box2):
    width = min(box1.max_corner.x, box2.max_corner.x) - max(box1.min_corner.x, box2.min_corner.x)
    height = min(box1.max_corner.y, box2.max_corner.y) - max(box1.min_corner.y, box2.min_corner.y);
    if width < 0 or height < 0:
        return 0
    else:
        return width * height;

def compare_box(box1, box2):
    #print("box1:%s" % box1)
    #print("box2:%s" % box2)
    intersection_area = overlapping_area(box1, box2)
    area1 = box1.width() * box1.height()
    area2 = box2.width() * box2.height()

    rate = float(intersection_area) / (area1 + area2 - intersection_area)
    if rate >= MINIMAL_OVERLAP_THRESHOLD:
        return True
    else:
        return False

def hasInboxs(box, boxList):
    hasIn = False
    for detectionBox in boxList:
        if compare_box(box, detectionBox):
            hasIn = True
            break
    return hasIn

def mergeTruthValue(value1, value2):
    objects1 = value1[1]
    objects2 = value2[1]
    if value1[0] == value2[0]:
        objects1.extend(objects2)
    return value1

def mergeLDAndFCW(value1, value2):
    objects1 = value1[1]
    objects2 = value2[1]
    if value1[0] == value2[0]:
        return (value1[0], objects1, objects2)

def filterTruthValue(value1, value2):
    result = []
    if value1[0] == value2[0]:
        for object1 in value1[1]:
            for object2 in value2[1]:
                if object1 == object2:
                    break
            else:
                result.append(object1)
    return (value1[0], result)

def readGtValue(fileName):
    frameNumber = -1
    try:
        with open(fileName, 'r') as text_file:
            while 1:
                line = text_file.readline().strip()
                if not line:
                    break
                datas = [x.strip() for x in line.split(',') if x.strip()]
                if len(datas) == 2:
                    frameNumber = int(datas[0])
                    objectsCount = int(datas[1])
                    objects = []
                    for _ in xrange(objectsCount):
                        objectLine = text_file.readline()
                        objectDatas = [x.strip() for x in objectLine.split(',') if x.strip()]
                        if len(objectDatas) >= 4:
                            box = Box()
                            box.min_corner.x = int(objectDatas[0])
                            box.min_corner.y = int(objectDatas[1])
                            width = int(objectDatas[2])
                            height = int(objectDatas[3])
                            box.max_corner.x = box.min_corner.x + int(objectDatas[2])
                            box.max_corner.y = box.min_corner.y + int(objectDatas[3])
                            objects.append(box)
                    if frameNumber >= 0:
                        yield (frameNumber, objects)
    except IOError:
        return
    return

def readLDTruthValue(fileName, LDMinY, LDMaxY):
    frameNumber = -1
    try:
        with open(fileName, 'r') as text_file:
            while 1:
                line = text_file.readline().strip()
                if not line:
                    break
                datas = [x.strip() for x in line.split(',') if x.strip()]
                if len(datas) == 1:
                    points = []
                    frameNumber = int(datas[0])
                    ldLine = text_file.readline()
                    ldDatas = [x.strip() for x in ldLine.split(',') if x.strip()]
                    try:
                        if len(ldDatas) >= 6:
                            leftX = int(ldDatas[0])
                            leftY = int(ldDatas[1])
                            leftSlope = float(ldDatas[2])
                            rightX = int(ldDatas[3])
                            rightY = int(ldDatas[4])
                            rightSlope = float(ldDatas[5])
                            leftB = leftY - leftX * leftSlope
                            rightB = rightY - rightX * rightSlope
                            leftPoint1 = Point2d(0, 0)
                            leftPoint2 = Point2d(0, 0)
                            if leftSlope > 0:
                                leftPoint1.x = int((LDMinY - leftB) / leftSlope)
                                leftPoint1.y = LDMinY
                                leftPoint2.x = int((LDMaxY - leftB) / leftSlope)
                                leftPoint2.y = LDMaxY

                            rightPoint1 = Point2d(0, 0)
                            rightPoint2 = Point2d(0, 0)
                            if rightSlope > 0:
                                rightPoint1.x = int((LDMinY - rightB) / rightSlope)
                                rightPoint1.y = LDMinY
                                rightPoint2.x = int((LDMaxY - rightB) / rightSlope)
                                rightPoint2.y = LDMaxY
                            points.append(leftPoint1)
                            points.append(leftPoint2)
                            points.append(rightPoint1)
                            points.append(rightPoint2)
                        if frameNumber >= 0:
                            yield (frameNumber, points)
                    except ValueError:
                        yield (frameNumber, [])
    except IOError:
        return
    return

def readFCWTruthValue(videoPath):
    resultDatas = []
    dataPath = unicode(videoPath, "utf-8")
    path, file_name_and_post = os.path.split(dataPath)
    videoName, post = os.path.splitext(file_name_and_post)

    #gt.txt
    gtFileName = videoName + "_gt.txt"
    gtPath = os.path.join(path, gtFileName)
    gtDatas = readGtValue(gtPath)
    # dt.txt
    dtFileName = videoName + "_dt.txt"
    dtPath = os.path.join(path, dtFileName)
    dtDatas = readGtValue(dtPath)
    # et.txt
    etFileName = videoName + "_et.txt"
    etPath = os.path.join(path, etFileName)
    etDatas = readGtValue(etPath)

    #dt - et + gt
    if gtDatas or dtDatas or etDatas:
        if dtDatas and etDatas and gtDatas:
            tempDatas = map(filterTruthValue, dtDatas, etDatas)
            resultDatas = map(mergeTruthValue, tempDatas, gtDatas)
        elif gtDatas:
            resultDatas = list(gtDatas)

    return resultDatas

def readSmartEyeResult(resultFileName, minWidth, minHeight, threshold=0):
    index = 1
    frameNumber = -1
    ldResult = []
    fcResult = []
    try:
        with open(resultFileName, 'r') as text_file:
            for line in text_file:
                if index % 3 == 1:
                    ldResult = []
                    fcResult = []
                    datas = [x.strip() for x in line.split(':') if x.strip()]
                    frameNumber = -1
                    if len(datas) == 2:
                        frameNumber = int(datas[1])
                elif index % 3 == 2:
                    jsonData = json.loads(line)
                    #print(jsonData["LD"]["ldCount"])
                    pointsData = jsonData["LD"]["ldResult"]
                    for data in pointsData:
                        point = Point2d(int(data["x"]), int(data["y"]))
                        ldResult.append(point)
                elif index % 3 == 0:
                    jsonData = json.loads(line)
                    #print(jsonData["FC"]["fcCount"])
                    boxesData = jsonData["FC"]["fcResult"]
                    for data in boxesData:
                        confidence = data.get("confidence", None)
                        if confidence:
                            confidence = int(confidence)
                        else:
                            confidence = 100
                        if confidence >= threshold:
                            box = Box()
                            width = int(data["wid"])
                            height = int(data["hig"])
                            if width >= minWidth and height >= minHeight:
                                box.confidence = confidence
                                box.min_corner.x = int(data["x"])
                                box.min_corner.y = int(data["y"])
                                box.max_corner.x = box.min_corner.x + width
                                box.max_corner.y = box.min_corner.y + height
                                fcResult.append(box)
                    if frameNumber >= 0:
                        yield (frameNumber, ldResult, fcResult)
                index += 1
    except IOError:
        return
    return

def readSmartEyeSlaverResult(resultFileName, minWidth, minHeight):
    index = 1
    frameNumber = -1
    ldResult = []
    fcResult = []
    try:
        with open(resultFileName, 'r') as text_file:
            for line in text_file:
                if index % 3 == 1:
                    ldResult = []
                    fcResult = []
                    datas = [x.strip() for x in line.split(':') if x.strip()]
                    frameNumber = -1
                    if len(datas) == 2:
                        frameNumber = int(datas[1])
                elif index % 3 == 2:
                    jsonData = json.loads(line)
                    #print(jsonData["LD"]["ldCount"])
                    pointsData = jsonData["LD"]["ldResult"]
                    for data in pointsData:
                        status = int(data["status"])
                        if status == 16:
                            break
                        point = Point2d(int(data["x"]), int(data["y"]))
                        ldResult.append(point)
                elif index % 3 == 0:
                    jsonData = json.loads(line)
                    #print(jsonData["FC"]["fcCount"])
                    boxesData = jsonData["FC"]["fcResult"]
                    for data in boxesData:
                        box = Box()
                        width = int(data["wid"])
                        height = int(data["hig"])
                        if width >= minWidth and height >= minHeight:
                            box.min_corner.x = int(data["x"])
                            box.min_corner.y = int(data["y"])
                            box.max_corner.x = box.min_corner.x + width
                            box.max_corner.y = box.min_corner.y + height
                            fcResult.append(box)
                    if frameNumber >= 0:
                        yield (frameNumber, ldResult, fcResult)
                index += 1
    except IOError:
        return
    return

def readSmartEyeDetSlaverResult(resultFileName, minWidth, minHeight, threshold):
    index = 1
    frameNumber = -1
    ldResult = []
    detResult = []
    fcResult = []
    try:
        with open(resultFileName, 'r') as text_file:
            for line in text_file:
                if index % 4 == 1:
                    ldResult = []
                    detResult = []
                    fcResult = []
                    datas = [x.strip() for x in line.split(':') if x.strip()]
                    frameNumber = -1
                    if len(datas) == 2:
                        frameNumber = int(datas[1])
                elif index % 4 == 2:
                    jsonData = json.loads(line)
                    # print(jsonData["LD"]["ldCount"])
                    pointsData = jsonData["LD"]["ldResult"]
                    for data in pointsData:
                        status = int(data["status"])
                        if status == 16:
                            break
                        point = Point2d(int(data["x"]), int(data["y"]))
                        ldResult.append(point)
                elif index % 4 == 3:
                    jsonData = json.loads(line)
                    # print(jsonData["DET"]["detCount"])
                    boxesData = jsonData["DET"]["detResult"]
                    for data in boxesData:
                        box = Box()
                        width = int(data["wid"])
                        height = int(data["hig"])
                        if width >= minWidth and height >= minHeight:
                            box.min_corner.x = int(data["x"])
                            box.min_corner.y = int(data["y"])
                            box.max_corner.x = box.min_corner.x + width
                            box.max_corner.y = box.min_corner.y + height
                            detResult.append(box)
                elif index % 4 == 0:
                    jsonData = json.loads(line)
                    # print(jsonData["FC"]["fcCount"])
                    boxesData = jsonData["FC"]["fcResult"]
                    for data in boxesData:
                        box = Box()
                        width = int(data["wid"])
                        height = int(data["hig"])
                        if width >= minWidth and height >= minHeight:
                            box.min_corner.x = int(data["x"])
                            box.min_corner.y = int(data["y"])
                            box.max_corner.x = box.min_corner.x + width
                            box.max_corner.y = box.min_corner.y + height
                            fcResult.append(box)
                    if frameNumber >= 0:
                        yield (frameNumber, ldResult, detResult, fcResult)
                index += 1
    except IOError:
        return
    return

def evalutionFCW(truthBoxes, detectionBoxes):
    TP = 0
    FN = 0
    for detectionBox in detectionBoxes:
        if hasInboxs(detectionBox, truthBoxes):
            TP += 1

    for truthBox in truthBoxes:
        if not hasInboxs(truthBox, detectionBoxes):
            FN += 1

    return TP, FN

def videoEvalution(videoPath, truthDatas, resultDatas, imageWidth, imageHeight, isSaveVideo):
    path, file_name_and_post = os.path.split(videoPath)
    videoName, post = os.path.splitext(file_name_and_post)
    truthObjectCount = 0
    detectObjectCount = 0
    truePositives = 0
    falseNegative = 0
    pecision = 0
    recall = 0
    index = 0
    if isSaveVideo == 0:
        for frameNumber, points, boxes in truthDatas:
            resultNumber, resultLD, resultFCW = resultDatas[index]
            if resultNumber == frameNumber:
                truthObjectCount += len(boxes)
                detectObjectCount += len(resultFCW)
                TP, FN = evalutionFCW(boxes, resultFCW)
                truePositives += TP
                falseNegative += FN
                index += 1
    elif isSaveVideo == 1:
        capture = cv2.VideoCapture(videoPath)
        if capture.isOpened():
            saveVideoName = videoName + "_result.avi"
            resultDir = os.path.join(path, "../result/")
            saveVideoPath = os.path.join(resultDir, saveVideoName)
            fps = capture.get(cv2.CAP_PROP_FPS)
            videoWriter = cv2.VideoWriter(saveVideoPath, cv2.VideoWriter_fourcc('D', 'I', 'V', 'X'), fps,
                                          (imageWidth, imageHeight))
            for frameNumber, points, boxes in truthDatas:
                resultNumber, resultLD, resultFCW = resultDatas[index]
                if resultNumber == frameNumber:
                    capture.set(cv2.CAP_PROP_POS_FRAMES, frameNumber - 1)
                    ret, frame = capture.read()
                    if ret == False:
                        print("video read end!")
                        break
                    else:
                        resized_image = cv2.resize(frame, (imageWidth, imageHeight))
                        showObjects(boxes, resized_image, (0, 0, 255), 2)
                        showObjects(resultFCW, resized_image, (0, 255, 255), 2)
                        showLines(points, resized_image, (0, 0, 255))
                        showPoints(resultLD, resized_image, 8, (0, 255, 255))
                        videoWriter.write(resized_image)
                        truthObjectCount += len(boxes)
                        detectObjectCount += len(resultFCW)
                        TP, FN = evalutionFCW(boxes, resultFCW)
                        truePositives += TP
                        falseNegative += FN
                    index += 1
            videoWriter.release()
        else:
            print("open video:%s fail!" % videoPath)

    if detectObjectCount != 0:
        pecision = float(truePositives) / detectObjectCount
    if truePositives != 0:
        recall = 1 - float(falseNegative) / truthObjectCount
    resultFCW = {}
    resultFCW["videoName"] = file_name_and_post
    resultFCW["truthCount"] = truthObjectCount
    resultFCW["detectCount"] = detectObjectCount
    resultFCW["TP"] = truePositives
    resultFCW["FN"] = falseNegative
    resultFCW["pecision"] = pecision
    resultFCW["recall"] = recall
    with codecs.open('FCWResult.txt', 'a+', 'utf-8') as f:
        f.write(json.dumps({"FCW": resultFCW}, ensure_ascii=False))
        f.write("\n")