#ifndef IDENTIFYNET_H
#define IDENTIFYNET_H

#include "opencv2/core.hpp"
#include "opencv2/dnn.hpp"

class FClassifier
{
public:

    FClassifier();
    ~FClassifier();
    
    int initClassifier(std::string modelTxt, std::string modelBin);
    
    int identifyNet(cv::Mat img);

private:

     int argmax(cv::dnn::Blob &score);
     void init();

private:

    cv::dnn::Net net;
};

#endif // IDENTIFYNET
