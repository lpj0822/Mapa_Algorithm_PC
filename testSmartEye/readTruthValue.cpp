#include "readTruthValue.h"

static const int videoHeighty = 0;
static const int videoWidthx = 0;
static const float videoRatio = 1.0f;
static const int minObjectWidth = 20;
static const int minObjectHeight = 20;

static int compareCarTruthValue(const CarTruthValue object1, const CarTruthValue objects2)
{
	if ((object1.x == objects2.x) && (object1.y == objects2.y) && (object1.width == objects2.width) && (object1.height == objects2.height))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

//read et txt
static int readEtValue(std::string videoPathName, std::vector< std::vector<CarTruthValue> > &videoTruthValues)
{
	int result = 0;
	int frameno;
	char c;
	int num;
	int objectCount;
	int i = 0;
	std::vector<CarTruthValue> carObjects;

	std::string etPathName(videoPathName);
	std::string savefromat(".txt");

	etPathName.insert((etPathName.find_last_of('.')), "_et");
	etPathName.replace(etPathName.find_last_of('.'), 4, savefromat);

	std::ifstream in(etPathName, std::ios::in);

	videoTruthValues.clear();

	if (in.is_open())
	{
		while (!in.eof())
		{
			in >> frameno >> c >> objectCount;

			if (frameno == i)
			{
				carObjects.clear();
				for (int j = 0; j < objectCount; j++)
				{
					CarTruthValue value;
					in >> value.x >> c;
					in >> value.y >> c;
					in >> value.width >> c;
					in >> value.height >> c >> num;
					carObjects.push_back(value);
				}
				videoTruthValues.push_back(carObjects);
			}
			i++;
		}
	}
	else
	{
		std::cout << "can not open file:" << etPathName << std::endl;
		result = -1;
	}
	in.close();
	return result;
}

//read dt txt
static int readDtValue(std::string videoPathName, std::vector< std::vector<CarTruthValue> > &videoTruthValues)
{
	int result = 0;
	int frameno;
	char c;
	int num;
	int objectCount;
	int i = 0;
	std::vector<CarTruthValue> carObjects;

	std::string dtPathName(videoPathName);
	std::string savefromat(".txt");

	dtPathName.insert((dtPathName.find_last_of('.')), "_dt");
	dtPathName.replace(dtPathName.find_last_of('.'), 4, savefromat);

	std::ifstream in(dtPathName, std::ios::in);

	videoTruthValues.clear();

	if (in.is_open())
	{
		while (!in.eof())
		{
			in >> frameno >> c >> objectCount;

			if (frameno == i)
			{
				carObjects.clear();
				for (int j = 0; j < objectCount; j++)
				{
					CarTruthValue value;
					in >> value.x >> c;
					in >> value.y >> c;
					in >> value.width >> c;
					in >> value.height;
					carObjects.push_back(value);
				}
				videoTruthValues.push_back(carObjects);
			}
			i++;
		}
	}
	else
	{
		std::cout << "can not open file:" << dtPathName << std::endl;
		result = -1;
	}
	in.close();
	return result;
}

//read dt txt
static int readDtValue1(std::string videoPathName, std::vector< std::vector<CarTruthValue> > &videoTruthValues)
{
	int result = 0;
	int frameno;
	char c;
	int num;
	int objectCount;
	int i = 0;
	std::vector<CarTruthValue> carObjects;

	std::string dtPathName(videoPathName);
	std::string savefromat(".txt");

	dtPathName.insert((dtPathName.find_last_of('.')), "_dt");
	dtPathName.replace(dtPathName.find_last_of('.'), 4, savefromat);

	std::ifstream in(dtPathName, std::ios::in);

	videoTruthValues.clear();

	if (in.is_open())
	{
		int i = 0;
		while (!in.eof())
		{
			if (i == 0)
			{
				in >> frameno;
			}
			else
			{
				in >> frameno >> c >> objectCount;

				if (frameno == i - 1)
				{
					carObjects.clear();
					for (int j = 0; j < objectCount; j++)
					{
						CarTruthValue value;
						in >> value.x >> c;
						in >> value.y >> c;
						in >> value.width >> c;
						in >> value.height >> c >> num;
						carObjects.push_back(value);
					}
					videoTruthValues.push_back(carObjects);
				}
			}
			i++;
		}
	}
	else
	{
		std::cout << "can not open file:" << dtPathName << std::endl;
		result = -1;
	}
	in.close();
	return result;
}

//read gt txt
int readGtValue(const std::string videoPathName, std::vector< std::vector<CarTruthValue> > &videoTruthValues)
{
	int result = 0;
	int frameno;
	char c;
	int num;
	int objectCount = 0;
	std::vector<CarTruthValue> carObjects;
	std::string gtPathName(videoPathName);
	std::string savefromat(".txt");

	gtPathName.insert((gtPathName.find_last_of('.')), "_gt");
	gtPathName.replace(gtPathName.find_last_of('.'), 4, savefromat);

	std::ifstream in(gtPathName, std::ios::in);

	videoTruthValues.clear();

	if (in.is_open())
	{
		int i = 0;
		while (!in.eof())
		{
			if (i == 0)
			{
				in >> frameno;
			}
			else
			{
				in >> frameno >> c >> objectCount;

				if (frameno == i - 1)
				{
					carObjects.clear();
					for (int j = 0; j < objectCount; j++)
					{
						CarTruthValue value;
						in >> value.x >> c;
						in >> value.y >> c;
						in >> value.width >> c;
						in >> value.height >> c >> num;
						carObjects.push_back(value);
					}
					videoTruthValues.push_back(carObjects);
				}
			}
			i++;
		}
	}
	else
	{
		std::cout << "can not open file:" << gtPathName << std::endl;
		result = -1;
	}
	in.close();
	return result;
}

//read dt - et + gt
int readTruthValue(const std::string videoPathName, std::vector< std::vector<CarTruthValue> > &videoTruthValues)
{
	int result = 0;
	std::vector< std::vector<CarTruthValue> > gtValues;
	std::vector< std::vector<CarTruthValue> > etValues;
	std::vector< std::vector<CarTruthValue> > dtValues;
	std::vector<CarTruthValue> carObjects;

	gtValues.clear();
	etValues.clear();
	dtValues.clear();

	readGtValue(videoPathName, gtValues);
	readEtValue(videoPathName, etValues);
	readDtValue1(videoPathName, dtValues);

	if (dtValues.size() == etValues.size() && dtValues.size() == gtValues.size())
	{
		for (int i = 0; i < dtValues.size(); i++)
		{
			carObjects.clear();
			for (int loop = 0; loop < dtValues[i].size(); loop++)
			{
				int loop2 = 0;
				for (loop2 = 0; loop2 < etValues[i].size(); loop2++)
				{
					if (compareCarTruthValue(dtValues[i][loop], etValues[i][loop2]))
					{
						break;
					}
				}

				if (loop2 >= etValues[i].size())
					carObjects.push_back(dtValues[i][loop]);
			}
			videoTruthValues.push_back(carObjects);
		}

		for (int i = 0; i < gtValues.size(); i++)
		{
			for (int loop = 0; loop < gtValues[i].size(); loop++)
			{
				videoTruthValues[i].push_back(gtValues[i][loop]);
			}
		}
	}
	else
	{
		for (int i = 0; i < dtValues.size(); i++)
		{
			carObjects.clear();
			videoTruthValues.push_back(carObjects);
		}
		std::cout << videoPathName << ": truth value error!" << std::endl;
		result = -1;
	}
	return result;
}

//LD Filter
std::vector<CarTruthValue> truthValueFilter(const WissenImage *GrayImg, const LDWS_Output *pLDWS, const std::vector<CarTruthValue> &truthValue)
{
	LDWS_Point roiPoint[64];
	int laneWidth;
	int i = 0;
	int flag = 0;
	std::vector<CarTruthValue> result;
	result.clear();

	if (pLDWS != NULL)
	{
		for (i = 0; i < pLDWS->NB_INTERVALLES; i++)
		{
			roiPoint[i].y = pLDWS->pCaPoint[i].y;
			if (roiPoint[i].y < 0)
			{
				roiPoint[i].y = 0;
			}
			roiPoint[i + pLDWS->NB_INTERVALLES].y = roiPoint[i].y;

			/* Extended the ROI region for vehicle detection */
			laneWidth = (pLDWS->pCaPoint[i + pLDWS->NB_INTERVALLES].x - pLDWS->pCaPoint[i].x + 1);// << 1;

			roiPoint[i + pLDWS->NB_INTERVALLES].x = pLDWS->pCaPoint[i + pLDWS->NB_INTERVALLES].x + laneWidth - 1;
			roiPoint[i].x = pLDWS->pCaPoint[i].x - laneWidth + 1;

			if (roiPoint[i + pLDWS->NB_INTERVALLES].x >= GrayImg->nWid)
			{
				roiPoint[i + pLDWS->NB_INTERVALLES].x = GrayImg->nWid - 1;
			}
			else if (roiPoint[i + pLDWS->NB_INTERVALLES].x < 0)
			{
				roiPoint[i + pLDWS->NB_INTERVALLES].x = 0;
			}

			if (roiPoint[i].x >= GrayImg->nWid)
			{
				roiPoint[i].x = GrayImg->nWid - 1;
			}
			else if (roiPoint[i].x < 0)
			{
				roiPoint[i].x = 0;
			}

		}

		for (auto it = truthValue.begin(); it != truthValue.end(); it++)
		{
			CarTruthValue truthValue;
			int objectX = (int)((it->x - videoWidthx) / videoRatio);
			int objectY = (int)(it->y / videoRatio);
			int objectWidth = (int)(it->width / videoRatio);
			int objectHeight = (int)(it->height / videoRatio);

			truthValue.x = objectX;
			truthValue.y = objectY;
			truthValue.width = objectWidth;
			truthValue.height = objectHeight;

			if (objectWidth < minObjectWidth || objectHeight < minObjectHeight)
				continue;

			flag = 0;
			for (i = 0; i < pLDWS->NB_INTERVALLES; ++i)
			{
				if (objectY + objectHeight < roiPoint[i].y)
				{
					if (objectX > roiPoint[i].x && objectX < roiPoint[i + pLDWS->NB_INTERVALLES].x)
					{
						flag = 1;
					}
					break;
				}
				else
				{
					if (i == pLDWS->NB_INTERVALLES - 1)
					{
						if (objectY + (objectHeight >> 1) < roiPoint[i].y)
						{
							if (objectX > roiPoint[i].x && objectX < roiPoint[i + pLDWS->NB_INTERVALLES].x)
							{
								flag = 1;
							}
							break;
						}
					}
				}
			}
			if (flag == 1)
			{
				result.push_back(*it);
			}
		}

	}
	else
	{
		for (auto it = truthValue.begin(); it != truthValue.end(); it++)
		{
			CarTruthValue truthValue;
			int objectX = (int)((it->x - videoWidthx) / videoRatio);
			int objectY = (int)(it->y / videoRatio);
			int objectWidth = (int)(it->width / videoRatio);
			int objectHeight = (int)(it->height / videoRatio);

			truthValue.x = objectX;
			truthValue.y = objectY;
			truthValue.width = objectWidth;
			truthValue.height = objectHeight;

			if (objectWidth < minObjectWidth || objectHeight < minObjectHeight)
				continue;

			result.push_back(*it);
		}
	}
	return result;
}

//save gt1
int saveGtResultToTxt(const std::string videoPathName, const int allCount, const std::vector< std::vector<CarTruthValue> > &resultTruthValues)
{
	int result = 0;
	std::string gtPathName(videoPathName);
	std::string savefromat(".txt");
	gtPathName.insert((gtPathName.find_last_of('.')), "_gt1");
	gtPathName.replace(gtPathName.find_last_of('.'), 4, savefromat);

	std::ofstream writeFile(gtPathName, std::ios::out);

	if (writeFile.is_open())
	{
		writeFile << allCount << std::endl;
		for (int i = 0; i < resultTruthValues.size(); i++)
		{
			writeFile << i << ',' << resultTruthValues[i].size() << std::endl;
			for (int j = 0; j < resultTruthValues[i].size(); j++)
			{
				writeFile << resultTruthValues[i][j].x << ',' << resultTruthValues[i][j].y << ',' << resultTruthValues[i][j].width << ',' << resultTruthValues[i][j].height << ',' << j << std::endl;
			}
		}
	}
	else
	{
		std::cout << "can not write gt1 file!" << std::endl;
		result = -1;
	}
	writeFile.close();
	return result;
}