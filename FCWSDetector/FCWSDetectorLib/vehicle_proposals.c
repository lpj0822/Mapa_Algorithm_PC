#include "vehicle_proposals.h"
#include "LDWS_Interface.h"

#include <stdio.h>
#include <stdlib.h>

//day
#define OFS (1024) // 256*4
#define SOBEL_TABLE (2304) // OFS * 2 + 256
#define ftzero (80)
#define BLOCK_SIZE_X (6)
#define BLOCK_SIZE_Y (2)

//dark
#define MIN_HIST (170)
#define MAX_HIST (220)
#define HIST_COUNT (256 - MIN_HIST)
static unsigned char imageHist[HIST_COUNT] = { 0 };

static ProposalsGlobalPara proposalsPara = { 0 };

static void computeLightImage(const WissenImage *pOriGrayImg, const WissenRect *proposalsRoi, const int relativeRoiY, unsigned char *imageHist, unsigned char *darkImage);

static void initSobelTable(unsigned char *sobleTable);

static void prefilterYSobel(const WissenImage *pOriGrayImg, const WissenRect *proposalsRoi, const int relativeRoiY, const unsigned char *sobleTable, unsigned char *dyImage);

static void computeDyLine(const WissenRect *proposalsRoi, const int relativeRoiY, const unsigned char *dyImage, unsigned char *dyLineImage);

static void computeIntegrate(const WissenRect *proposalsRoi, const int relativeRoiY, const unsigned char *image, unsigned int *integralImage);

int initProposals(const int srcWidth, const int srcHeight, const float srcROIYFactor)
{
	float proposalsRoiYFactor = 0.0f;
	int allMallocSize = 0;
	int count = 0;
	int srcRoiY = 0;
	int srcRoiWidth = 0;
	int srcRoiHeight = 0;
	int stopY = 0;
	LDWS_InitGuid *pLDWSInit = NULL;
	unsigned char* copyMallPtr = NULL;
	if (srcROIYFactor > 0.45f)
	{
		proposalsRoiYFactor = srcROIYFactor;
	}
	else
	{
		proposalsRoiYFactor = 0.45f;
	}

	srcRoiY = (int)(srcHeight * srcROIYFactor);
	srcRoiWidth = srcWidth;
	srcRoiHeight = srcHeight - srcRoiY;
	count = srcRoiWidth * srcRoiHeight;

	LDWS_Getinit(&pLDWSInit);
	stopY = WS_MAX(srcRoiY, pLDWSInit->pBoundPoint[2].y) + 1;
	stopY = WS_MIN(stopY, srcHeight);

	proposalsPara.proposalsRoi.x = 0;
	proposalsPara.proposalsRoi.y = (int)(srcHeight * proposalsRoiYFactor);
	proposalsPara.proposalsRoi.width = srcWidth;
	proposalsPara.proposalsRoi.height = stopY - proposalsPara.proposalsRoi.y;

	proposalsPara.integrateRoi.x = 0;
	proposalsPara.integrateRoi.y = (int)(srcHeight * proposalsRoiYFactor);
	proposalsPara.integrateRoi.width = srcWidth;
	proposalsPara.integrateRoi.height = srcHeight - proposalsPara.integrateRoi.y;

	proposalsPara.relativeRoiY = proposalsPara.proposalsRoi.y - srcRoiY;

	allMallocSize = SOBEL_TABLE * sizeof(unsigned char)+count * 2 * sizeof(unsigned char)+count * sizeof(unsigned int)+8 * sizeof(unsigned char);

	proposalsPara.allMemory = (unsigned char *)malloc(allMallocSize);

	if (proposalsPara.allMemory == NULL)
		return 0;

	memset(proposalsPara.allMemory, 0, allMallocSize * sizeof(unsigned char));

	copyMallPtr = proposalsPara.allMemory;
	proposalsPara.sobleTable = (unsigned char*)copyMallPtr;

	copyMallPtr = ALIGN_16BYTE(proposalsPara.sobleTable + SOBEL_TABLE);
	proposalsPara.dyImage = (unsigned char*)copyMallPtr;

	copyMallPtr = ALIGN_16BYTE(proposalsPara.dyImage + count);
	proposalsPara.dyLineImage = (unsigned char*)copyMallPtr;

	copyMallPtr = ALIGN_16BYTE(proposalsPara.dyLineImage + count);
	proposalsPara.dyIntegralImage = (unsigned int*)copyMallPtr;

	initSobelTable(proposalsPara.sobleTable);
	
	if (pLDWSInit != NULL)
	{
		LDWS_Freeinit(&pLDWSInit);
		pLDWSInit = NULL;
	}
	return 1;
}

void computeProposals(const unsigned char flag, const WissenImage *pOriGrayImg)
{
	if (flag == 0)//day
	{
		prefilterYSobel(pOriGrayImg, &proposalsPara.proposalsRoi, proposalsPara.relativeRoiY, proposalsPara.sobleTable, proposalsPara.dyImage);
		computeDyLine(&proposalsPara.proposalsRoi, proposalsPara.relativeRoiY, proposalsPara.dyImage, proposalsPara.dyLineImage);
		computeIntegrate(&proposalsPara.integrateRoi, proposalsPara.relativeRoiY, proposalsPara.dyLineImage, proposalsPara.dyIntegralImage);
	}
	else if(flag == 1)// night
	{
		computeLightImage(pOriGrayImg, &proposalsPara.proposalsRoi, proposalsPara.relativeRoiY, imageHist, proposalsPara.dyImage);
		computeIntegrate(&proposalsPara.integrateRoi, proposalsPara.relativeRoiY, proposalsPara.dyImage, proposalsPara.dyIntegralImage);
	}

}

int filterCarTask(const unsigned char flag, const WissenPoint* task, const FCWSDetectorGlobalPara *pVehicleDetor, const float factor)
{
	const int height = (int)(pVehicleDetor->pdetorModel->l->data.featureHeight * factor);
	int minX = (int)(task->x * factor);
	int minY = (int)(task->y * factor);
	int maxX = minX + height;
	int maxY = minY + height;
	unsigned int a, b, c, d;
	float taskFactor = 0;

	if (proposalsPara.dyIntegralImage == NULL)
		return 1;

	a = *(proposalsPara.dyIntegralImage + minY * proposalsPara.proposalsRoi.width + minX);
	b = *(proposalsPara.dyIntegralImage + minY * proposalsPara.proposalsRoi.width + maxX);
	c = *(proposalsPara.dyIntegralImage + maxY * proposalsPara.proposalsRoi.width + minX);
	d = *(proposalsPara.dyIntegralImage + maxY * proposalsPara.proposalsRoi.width + maxX);
	taskFactor = (float)(a + d - b - c) / (height * height);

	if (flag == 0)//day
	{
		if (height < 100)
		{
			if (taskFactor >= 0.04f)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else if (height < 300)
		{
			if (taskFactor >= 0.02f)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			if (taskFactor >= 0.001f)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}

	}
	else if (flag == 1)// night
	{
		if (taskFactor > 0.9f)
		{
			return 0;
		}
		else if (height < 100)
		{
			if (taskFactor >= 0.02f)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else if (height < 300)
		{
			if (taskFactor >= 0.005f)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			if (taskFactor >= 0.001f)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
	}
	else
	{
		return 0;
	}
}

int freeProposals()
{
	if (proposalsPara.allMemory != NULL)
	{
		free(proposalsPara.allMemory);
		proposalsPara.allMemory = NULL;
	}
	proposalsPara.sobleTable = NULL;
	proposalsPara.dyImage = NULL;
	proposalsPara.dyLineImage = NULL;
	proposalsPara.dyIntegralImage = NULL;

	return 1;
}

static void computeLightImage(const WissenImage *pOriGrayImg, const WissenRect *proposalsRoi, const int relativeRoiY, unsigned char *imageHist, unsigned char *darkImage)
{
	int row = 0;
	int col = 0;
	int x = 0;
	int stopRow = proposalsRoi->height + proposalsRoi->y;
	int stopCol = proposalsRoi->width + proposalsRoi->x;
	int index = 0;
	int maxValue = 0;
	int threshold = 0;
	unsigned char *srcData = pOriGrayImg->data + proposalsRoi->y * pOriGrayImg->nWid;
	unsigned char *dstData = darkImage + relativeRoiY * proposalsRoi->width;

	memset(imageHist, 0, HIST_COUNT * sizeof(unsigned char));

	for (row = proposalsRoi->y; row < stopRow; row++)
	{
		for (col = proposalsRoi->x; col < stopCol; col++)
		{
			if (srcData[col] >= MIN_HIST)
			{
				index = srcData[col] - MIN_HIST;
				imageHist[index]++;
			}
		}
		srcData += pOriGrayImg->nWid;
	}

	index = 0;
	for (x = 0; x < HIST_COUNT; x++)
	{
		if (imageHist[x] > maxValue)
		{
			index = x;
		}
	}
	index += MIN_HIST;
	if (index >= MIN_HIST && index <= MAX_HIST)
	{
		threshold = MIN_HIST;
	}
	else
	{
		threshold = (int)(index * 0.8f);
	}

	srcData = pOriGrayImg->data + proposalsRoi->y * pOriGrayImg->nWid;
	for (row = proposalsRoi->y; row < stopRow; row++)
	{
		for (x = 0, col = proposalsRoi->x; col < stopCol; col++, x++)
		{
			if (srcData[col] > threshold)
			{
				dstData[x] = 1;
			}
			else
			{
				dstData[x] = 0;
			}
		}
		srcData += pOriGrayImg->nWid;
		dstData += proposalsRoi->width;
	}
}

static void initSobelTable(unsigned char *sobleTable)
{
	int x = 0;
	for (x = 0; x < SOBEL_TABLE; x++)
		sobleTable[x] = (unsigned char)(x - OFS < -ftzero ? 0 : x - OFS > ftzero ? ftzero * 2 : x - OFS + ftzero);
}

static void  prefilterYSobel(const WissenImage *pOriGrayImg, const WissenRect *proposalsRoi,
	const int relativeRoiY, const unsigned char *sobleTable, unsigned char *dyImage)
{
	int col = 0;
	int row = 0;
	int startX = proposalsRoi->x + 1;
	int startY = proposalsRoi->y + 1;
	int stopY = proposalsRoi->height + proposalsRoi->y - 1;
	int stopX = proposalsRoi->width + proposalsRoi->x - 2;
	unsigned char *srcData0 = pOriGrayImg->data + proposalsRoi->y * pOriGrayImg->nWid;
	unsigned char *srcData1 = srcData0 + pOriGrayImg->nWid;
	unsigned char *srcData2 = srcData1 + pOriGrayImg->nWid;
	unsigned char *dstData = dyImage + relativeRoiY * proposalsRoi->width;
	short d0, d1, d2, d3, v0, v1;
	int index1, index2, index3;

	for (row = startY; row < stopY; row++)
	{
		for (col = startX; col < stopX; col += 2)
		{
			index1 = col - 1;
			index2 = col + 1;
			index3 = col + 2;
			d0 = srcData0[index1] - srcData2[index1];
			d1 = srcData0[col] - srcData2[col];
			d2 = srcData0[index2] - srcData2[index2];
			d3 = srcData0[index3] - srcData2[index3];

			v0 = sobleTable[d0 + (d1 << 1) + d2 + OFS];
			v1 = sobleTable[d1 + (d2 << 1) + d3 + OFS];

			if (v0 < 50)
			{
				dstData[col] = 255;
			}
			else
			{
				dstData[col] = 0;
			}

			if (v1 < 50)
			{
				dstData[index2] = 255;
			}
			else
			{
				dstData[index2] = 0;
			}
		}
		srcData0 += pOriGrayImg->nWid;
		srcData1 += pOriGrayImg->nWid;
		srcData2 += pOriGrayImg->nWid;
		dstData += proposalsRoi->width;
	}
}

static void computeDyLine(const WissenRect *proposalsRoi, const int relativeRoiY, const unsigned char *dyImage, unsigned char *dyLineImage)
{
	int row = 0;
	int col = 0;
	int y = 0;
	int x = 0;
	float factor = 0.0f;
	int stopCol = proposalsRoi->width;
	int stopRow = proposalsRoi->height - BLOCK_SIZE_Y;
	int stepX = 1;
	int stepY = BLOCK_SIZE_Y;
	int rowWidth = BLOCK_SIZE_Y * proposalsRoi->width;
	int stopX = 0;
	int stopY = 0;
	int hCount = 0;
	int count = 0;
	const unsigned char *dyData = NULL;
	const unsigned char *dyRow = dyImage + relativeRoiY * proposalsRoi->width;
	unsigned char * dyLineImageData = dyLineImage + relativeRoiY * proposalsRoi->width;
	int carWidth = 0;

	memset(dyLineImageData, 0, proposalsRoi->width * proposalsRoi->height * sizeof(unsigned char));

	for (row = 0; row < stopRow; row += stepY)
	{
		carWidth = (int)LDWS_GetXLengthofImage(1.3, row + proposalsRoi->y);
		if (carWidth < BLOCK_SIZE_X)
		{
			carWidth = BLOCK_SIZE_X;
		}
		for (col = 0; col < stopCol; col += stepX)
		{
			stepX = 1;
			if (dyRow[col] != 0)
			{
				hCount = 0;
				count = 0;
				stopX = WS_MIN(col + carWidth, proposalsRoi->width);
				stopY = WS_MIN(row + BLOCK_SIZE_Y, proposalsRoi->height);

				dyData = dyRow;

				for (y = row; y < stopY; y++)
				{
					for (x = col; x < stopX; x++)
					{
						if (dyData[x] != 0)
						{
							hCount++;
						}
						count++;
					}
					dyData += proposalsRoi->width;
				}
				factor = (float)hCount / count;
				if (factor >= 0.4f)
				{
					for (x = col; x < stopX; x++)
					{
						dyLineImageData[x] = 1;
					}
				}
				stepX = carWidth >> 1;
			}
		}
		dyRow += rowWidth;
		dyLineImageData += rowWidth;
	}
}


// Helper function that integrates an image
static void computeIntegrate(const WissenRect *proposalsRoi, const int relativeRoiY, const unsigned char *image, unsigned int *integralImage)
{
	int row = 0;
	int col = 0;
	int sum = 0;
	int width = proposalsRoi->width;
	int height = proposalsRoi->height;
	unsigned int  *integralChannelPreviousRow = NULL;
	unsigned int  *integralChannelRow = NULL;
	const unsigned char *srcData = image + relativeRoiY * proposalsRoi->width;

	integralImage += relativeRoiY * proposalsRoi->width;

	for (col = 0; col < width; col++)
	{
		sum += srcData[col];
		integralImage[col] = sum;
	}

	integralChannelPreviousRow = integralImage;
	integralChannelRow = integralImage + proposalsRoi->width;
	srcData += proposalsRoi->width;

	for (row = 1; row < height; row += 1)
	{
		sum = 0;
		for (col = 0; col < width; col += 1)
		{
			sum += srcData[col];
			integralChannelRow[col] = integralChannelPreviousRow[col] + sum;
		}
		integralChannelPreviousRow += proposalsRoi->width;
		integralChannelRow += proposalsRoi->width;
		srcData += proposalsRoi->width;
	}
}
