#include "surf_feature.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

extern unsigned char uArrGaussRegion[36];
extern unsigned char uArrGaussSeed[16];
extern unsigned char sqrtData[255 * 255 + 1];
extern unsigned char stdDataSurf[511 * 160];


/*
I/O:	    Name		          Type	     		  Content

[in]	    srcImage		      const WissenImage*	  input image
[in]	    pPoint		          const AdasCorner*   input fast points
[in/out]	pSurfFeature		  unsigned char*		      surf feature.

Realized function:
+ caculate the surf feature of input point
*/
static void mvComputeSingleSurfDescriptor(const WissenImage* srcImage, const AdasCorner *pPoint, unsigned char *pSurfFeature)
{
	const unsigned char* imageData = srcImage->data;
	int i;
	int j;
	int nRow;
	int nCol;
	int nGaussIndex;
	int nSurFeaIndex = 0;
	int nSeedIndex = 0;
	int nArrDx[15][15];
	int nArrDy[15][15];
	int nPixelDx;
	int nPixelDy;
	int nSeedDx;
	int nSeedDy;
	int nSeedAbsDx;
	int nSeedAbsDy;
	int nArrSurfFea[SURF_DESC_DIMENTION];
	int nSquareSum = 0;
	int nSqrtSum;
	int nShift = 0;
	int nFeaThresh;
	int nMathSymbol;
	const unsigned char *pRowIndex1 = 0;
	const unsigned char *pRowIndex2 = 0;
	int pty0 = pPoint->point.y - 7;
	int pty1 = pPoint->point.y + 7;
	int ptx0 = pPoint->point.x - 7;
	int ptx1 = pPoint->point.x + 7;

	memset(nArrDx, 0, sizeof(int)* 15 * 15);
	memset(nArrDy, 0, sizeof(int)* 15 * 15);
	for (nRow = pty0, j = 0; nRow <= pty1; ++nRow, ++j)
	{
		if (nRow < 0 || nRow > srcImage->nHig - 2)
		{
			continue;
		}

		pRowIndex1 = imageData + srcImage->nWid * nRow;
		pRowIndex2 = imageData + srcImage->nWid * (nRow + 1);

		for (nCol = ptx0, i = 0; nCol <= ptx1; ++nCol, ++i)
		{
			if (nCol < 0 || nCol > srcImage->nWid - 2)
			{
				continue;
			}

			nArrDx[j][i] = *(pRowIndex1 + nCol + 1) - *(pRowIndex1 + nCol) + *(pRowIndex2 + nCol + 1) - *(pRowIndex2 + nCol);
			nArrDy[j][i] = *(pRowIndex2 + nCol) - *(pRowIndex1 + nCol) + *(pRowIndex2 + nCol + 1) - *(pRowIndex1 + nCol + 1);
		}
	}

	for (j = 0; j < 4; ++j)
	{
		for (i = 0; i < 4; ++i)
		{
			int j0 = j * 3;
			int j1 = j0 + 6;
			int i0 = i * 3;
			int i1 = i0 + 6;

			nSeedDx = nSeedDy = nSeedAbsDx = nSeedAbsDy = 0;
			nGaussIndex = 0;

			for (nRow = j0; nRow < j1; ++nRow)
			{
				for (nCol = i0; nCol < i1; ++nCol)
				{
					nPixelDx = uArrGaussRegion[nGaussIndex] * nArrDx[nRow][nCol];
					nPixelDy = uArrGaussRegion[nGaussIndex] * nArrDy[nRow][nCol];

					nSeedDx += nPixelDx;
					nSeedDy += nPixelDy;

					nPixelDx = (nPixelDx > 0) ? nPixelDx : -nPixelDx;
					nPixelDy = (nPixelDy > 0) ? nPixelDy : -nPixelDy;

					nSeedAbsDx += nPixelDx;
					nSeedAbsDy += nPixelDy;

					nGaussIndex++;
				}
			}

			nArrSurfFea[nSurFeaIndex++] = uArrGaussSeed[nSeedIndex] * nSeedDx;
			nArrSurfFea[nSurFeaIndex++] = uArrGaussSeed[nSeedIndex] * nSeedDy;
			nArrSurfFea[nSurFeaIndex++] = uArrGaussSeed[nSeedIndex] * nSeedAbsDx;
			nArrSurfFea[nSurFeaIndex++] = uArrGaussSeed[nSeedIndex] * nSeedAbsDy;

			nSeedIndex++;
		}
	}

	for (i = 0; i < SURF_DESC_DIMENTION; ++i)
	{
		nArrSurfFea[i] = nArrSurfFea[i] >> 18;

		if (nArrSurfFea[i] > 510)
		{
			nArrSurfFea[i] = 510;
		}
		else if (nArrSurfFea[i] < -510)
		{
			nArrSurfFea[i] = -510;
		}

		nSquareSum += nArrSurfFea[i] * nArrSurfFea[i];
	}

	for (i = 0; i <= 8; ++i)
	{
		if ((nSquareSum >> i) <= 65025)
		{
			nShift = i;
			break;
		}
	}

	nSquareSum = nSquareSum >> nShift;
	nSqrtSum = sqrtData[nSquareSum];

	if (1 == nShift)
	{
		nSqrtSum = (nSqrtSum * 362) >> 8;
	}
	else if (2 == nShift)
	{
		nSqrtSum = nSqrtSum << 1;
	}
	else if (3 == nShift)
	{
		nSqrtSum = (nSqrtSum * 362) >> 7;
	}
	else if (4 == nShift)
	{
		nSqrtSum = nSqrtSum << 2;
	}
	else if (5 == nShift)
	{
		nSqrtSum = (nSqrtSum * 362) >> 6;
	}
	else if (6 == nShift)
	{
		nSqrtSum = nSqrtSum << 3;
	}
	else if (7 == nShift)
	{
		nSqrtSum = (nSqrtSum * 362) >> 5;
	}
	else if (8 == nShift)
	{
		nSqrtSum = nSqrtSum << 4;
	}

	nFeaThresh = (nSqrtSum * 76) >> 8;

	for (i = 0; i <= 3; ++i)
	{
		if ((nSqrtSum >> i) <= 510)
		{
			nShift = i;
			break;
		}
	}

	nSqrtSum = nSqrtSum >> nShift;
	for (i = 0; i < SURF_DESC_DIMENTION; ++i)
	{
		nMathSymbol = 1;
		if (nArrSurfFea[i] < 0)
		{
			nArrSurfFea[i] = -nArrSurfFea[i];
			nMathSymbol = -1;
		}

		if (nArrSurfFea[i] > nFeaThresh)
		{
			nArrSurfFea[i] = nFeaThresh;
		}

		nArrSurfFea[i] = nArrSurfFea[i] >> nShift;

		pSurfFeature[i] = nMathSymbol * stdDataSurf[160 * nSqrtSum + nArrSurfFea[i]] + 64;
	}
}

/*
Function process:
+ caculate the surf feature of input point
Fan-in :
+ mvFeatrueDescribe()
Fan-out:
+ mvComputeSingleSurfDescriptor()
ATTENTION: __________
*/
void mvComputeSurfDescriptor(const WissenImage* srcImage, const AdasCorner *pPoint, int nPtCnt, unsigned char *pSurfFeature)
{
	int i = 0;
	for (i = 0; i < nPtCnt; ++i)
	{
		mvComputeSingleSurfDescriptor(srcImage, pPoint + i, pSurfFeature + (i << 6));
	}
}

/*
Function process:
+ caculate the distance between surf features
Fan-in :
+ ()
Fan-out:
+ N/A
ATTENTION: __________
*/
int mvSurfDist(const unsigned char *p1, const unsigned char *p2)
{
	int i;
	int dif;
	int distsq;

	distsq = 0;
	for (i = 0; i < SURF_DESC_DIMENTION; ++i)
	{
		dif = *p1++ - *p2++;
		distsq += dif * dif;
	}

	return distsq;
}
