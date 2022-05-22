#include "IdentifyNet.hpp"
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <cstdlib>

FClassifier::FClassifier()
{
    init();
    std::cout << "FClassifier()" << std::endl;
}

FClassifier::~FClassifier()
{
    std::cout << "~FClassifier()"<< std::endl;
}

int FClassifier::initClassifier(std::string modelTxt, std::string modelBin)
{
    cv::dnn::initModule();          //Required if OpenCV is built as static libs
    //! [Create the importer of Caffe model]
    try
    {
        cv::Ptr<cv::dnn::Importer> importer;
        importer = cv::dnn::createCaffeImporter(modelTxt, modelBin);
        //! [Initialize network]
        importer->populateNet(net);
        importer.release();                     //We don't need importer anymore
        //! [Initialize network]
    }
    catch (const cv::Exception &err)        //Importer can throw errors, we will catch them
    {
        std::cerr << "Can't load network by using the following files: " << std::endl;
        std::cerr << "prototxt:   " << modelTxt << std::endl;
        std::cerr << "caffemodel: " << modelBin << std::endl;
        return -1;
    }
    //! [Create the importer of Caffe model]
    return 0;
}

int FClassifier::identifyNet(cv::Mat img)
{
    if (img.empty())
    {
        std::cerr << "Something wrong with image. " << std::endl;
        return -1;
    }

    cv::Mat resizeMat, newMat;

    img.convertTo(newMat, CV_16S);

	cv::add(resizeMat, cv::Scalar(-99), resizeMat);

    cv::resize(newMat, resizeMat, cv::Size(25, 25)); 

    cv::dnn::Blob inputBlob = cv::dnn::Blob::fromImages(resizeMat);   //Convert Mat to dnn::Blob batch of images

    //! [Set input blob]
    net.setBlob(".data", inputBlob);        //set the network input

    //! [Make forward pass]
    net.forward();

    //! [Gather output]
    cv::dnn::Blob score = net.getBlob("prob");

    int result = argmax(score);
    return result;
}

int FClassifier::argmax(cv::dnn::Blob &score)
{
    int index=0;
    float maxScore = 0.0f;

    const float *ptrScore = score.ptrf(0, 0);
    //std::cout << ptrScore[0] << " " << ptrScore[1] << std::endl;
    if(ptrScore[1] > ptrScore[0])
    {
        maxScore = ptrScore[1];
        index = 1;
    }
    else
    {
        maxScore = ptrScore[0];
        index = 0;
    }
    if(index == 1 && maxScore > 0.6f)
    {
       return 1;
    }
    else if(index == 0 && maxScore > 0.6f)
    {
       return 0;
    }
    else
    {
       return 0;
    }
}

void FClassifier::init()
{

}
