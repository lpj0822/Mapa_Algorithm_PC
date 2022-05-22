#include "interpolation_algorithm.h"

/*
Function process:
+ Grayscale image resize and use neighbor interpolation algorithm.
*/
void grayImageResizeOfNeighborInterpolation(const WissenImage srcImage, const WissenRect roi, int *tempX, int *tempY, WissenImage *dstRoiImage)
{
	const int imageWidth = srcImage.nWid;
	const int resizeRoiWidth = dstRoiImage->nWid;
	const int resizeRoiHeight = dstRoiImage->nHig;
	int row = 0;
	int col = 0;
	float factorWidth = (float)roi.width / resizeRoiWidth;
	float factorHeight = (float)roi.height / resizeRoiHeight;
	const unsigned char* grayImage = srcImage.data + imageWidth * roi.y;
	const unsigned char* tempSrc = NULL;
	unsigned char* tempDst = NULL;
	for (col = 0; col < resizeRoiWidth; col++)
	{
		tempX[col] = (int)(col * factorWidth);
	}
	for (row = 0; row < resizeRoiHeight; row++)
	{
		tempY[row] = (int)(row * factorHeight);
	}
	for (row = 0; row < resizeRoiHeight; row++)
	{
		//grayImage data is positioned to ROI
		tempSrc = grayImage + roi.x + imageWidth * tempY[row];
		tempDst = dstRoiImage->data + resizeRoiWidth * row;
		for (col = 0; col < resizeRoiWidth; col++)
		{
			tempDst[col] = *(tempSrc + tempX[col]);
		}
	}
}