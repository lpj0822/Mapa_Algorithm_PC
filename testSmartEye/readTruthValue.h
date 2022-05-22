#ifndef READ_TRUTH_VALUE_H
#define READ_TRUTH_VALUE_H

#include <fstream>
#include <string>
#include <vector>
#include <iostream>

extern "C"
{
#include "LDWS_Interface.h"
#include "FCWSD_Interface.h"
}

typedef struct CarTruthValue
{
	int x;
	int y;
	int width;
	int height;
}CarTruthValue;

//read gt
int readGtValue(const std::string videoPathName, std::vector< std::vector<CarTruthValue> > &videoTruthValues);

//read dt - et + gt
int readTruthValue(const std::string videoPathName, std::vector< std::vector<CarTruthValue> > &videoTruthValues);

//LD Filter
std::vector<CarTruthValue> truthValueFilter(const WissenImage *GrayImg, const LDWS_Output *pLDWS, const std::vector<CarTruthValue> &truthValue);

//save gt1
int saveGtResultToTxt(const std::string videoPathName, const int allCount, const std::vector< std::vector<CarTruthValue> > &resultTruthValues);

#endif // READ_TRUTH_VALUE_H