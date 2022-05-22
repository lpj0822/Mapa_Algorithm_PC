#include "getCameraRefer.h"

const int Tx = 1920;
const int Ty = 1080;
int hist[(Tx >> 2) * (Ty >> 2)] = { 0 };
frameinfo labelSet[MAX_FRAME_NUMBER];


static int ReadLabel(string path)
{
	string labelname(path, 0, path.length() - 4);
	labelname.append("_ld.txt");
	ifstream file(labelname);
	if (!file.is_open())
	{
		return 0;
	}

	int i = 0, frameno;
	char c;
	while (!file.eof())
	{
		file >> frameno;
		if (frameno == i)
			file >> labelSet[i].leftx >> c >> labelSet[i].lefty >> c >> labelSet[i].leftSlope >> c >> labelSet[i].rightx >> c >> labelSet[i].righty >> c >> labelSet[i].rightSlope;
		else
		{
			break;
		}
		i++;
	}
	file.close();
	return 1;
}

void getCameraRefer(string sPathName, int* dx, int* dy, int* sy, int* deltx)
{
	int vinishx, vinishy;
	if (!ReadLabel(sPathName))
	{
		printf("Can not load LD file.txt! \n");
		return;
	}

	int maxNum = 0;
	bool flag = true;
	for (int i = 0; i < MAX_FRAME_NUMBER; i++)
	{
		if (labelSet[i].leftx == 0 || labelSet[i].lefty == 0 || labelSet[i].leftSlope == 0 ||
			labelSet[i].rightx == 0 || labelSet[i].righty == 0 || labelSet[i].rightSlope == 0)
		{
			continue;
		}

		double bl = labelSet[i].lefty - labelSet[i].leftSlope * labelSet[i].leftx;
		double br = labelSet[i].righty - labelSet[i].rightSlope * labelSet[i].rightx;

		int x = (int)((bl - br) / (labelSet[i].rightSlope - labelSet[i].leftSlope + 1e-10) + 0.5);
		int y = (int)(x * labelSet[i].leftSlope + bl + 0.5);

		if (x > 0 && x < Tx && y > 0 && y < Ty)
		{
			if (++hist[y * (Tx >> 4) + (x >> 2)] >= maxNum)
			{
				maxNum = hist[y * (Tx >> 4) + (x >> 2)];
				*dx = x;
				*dy = y;
			}
		}


		if (flag)
		{
			flag = false;
			*sy = (*dy >> 1) + 540;
			*deltx = (int)((*sy - br) / labelSet[i].rightSlope - (*sy - bl) / labelSet[i].leftSlope + 0.5);
		}
	}

	if (maxNum == 0)
	{
		*dx = 960;
		*dy = 540;
	}


	vinishx = ((double)(*dx)) * 1280 / 1920;
	vinishy = ((double)(*dy)) * 720 / 1080;
	int firstLine = vinishy + (double)(720 - vinishy)*0.16;
	double Z0 = (*sy - *dy)*3.63 / (*deltx);
	ofstream fileout("CarCalibration_test.txt");
	fileout << "CarWidth " << 180 << endl;
	fileout << "LeftDeviation " << 10 << endl;
	if (maxNum == 0)
		fileout << "CameraHigh " << 130 << endl;
	else
		fileout << "CameraHigh " << (int)(Z0 * 100) << endl;

	fileout << "RightDeviation " << 10 << endl;
	fileout << "RoadWidth " << 363 << endl;
	fileout << "VehicleWarning " << 4 << endl;
	fileout << "CameraPosx " << 186 << endl;
	fileout << "FirstLineY " << firstLine << endl;
	fileout << "SecondLineY " << 660 << endl;
	fileout << "VerticalLineX " << vinishx << endl;
	fileout << "HorizontalLineY " << vinishy << endl;
	fileout << "FCWSD_th" << 6 << endl;
	fileout.close();
}