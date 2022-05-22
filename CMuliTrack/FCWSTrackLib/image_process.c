#include "image_process.h"
#include "declare.h"
#include "utility_function.h"

/*
I/O:	    Name		    Type	     		  Content

[in]        srcimg	        const WissenImage*	      input image buffer
[in]	    pointRio		WissenPoint		      Left and up ROI point.
[in/out]	IntelImg		integral WissenImage*	  output integral image

Realized function:
+  caculate the integral image of ROI of srcimg
*/
void mvInterlimg(const WissenImage* srcimg, const WissenPoint pointRio, WissenImage* IntelImg)
{
	int i, j;
	int nColSum;
	int * pInterlTr = 0;
	unsigned char * pTr = 0;

	for (j = 0; j < IntelImg->nHig - 1; j++) 
	{
		nColSum = 0;

		pTr = srcimg->data + srcimg->nWid * (j + pointRio.y);

		pInterlTr = (int*)IntelImg->data + IntelImg->nWid * (j + 1);

		for (i = 0; i < IntelImg->nWid - 1; i++) 
		{
			nColSum += pTr[i + pointRio.x];
			pInterlTr[i + 1] = nColSum;
		}
	}

	for (i = 1; i < IntelImg->nWid; i++) 
	{
		for (j = 2; j < IntelImg->nHig; j++) 
		{
			pInterlTr = (int*)IntelImg->data + IntelImg->nWid * j;
			pInterlTr[i] += pInterlTr[i - IntelImg->nWid];
		}
	}
}

/*
I/O:	    Name		    Type	     		  Content

[in]	    IntelImg		integral WissenImage*	  input integral image
[in]	    lt		        Wissen16SPoint		      Left and up point of ROI.
[in]	    br		        Wissen16SPoint		      Right and bottom point of ROI.

Realized function:
+  Get the inteval value of ROI based on the interval image
*/
int mvSumImgRio(WissenImage* IntelImg, Wissen16SPoint lt, Wissen16SPoint br) 
{
	int * pInterlTr = (int*)IntelImg->data;

	int nSumVal = *(pInterlTr + IntelImg->nWid * lt.y + lt.x)
		+ *(pInterlTr + IntelImg->nWid * br.y + br.x)
		- *(pInterlTr + IntelImg->nWid * lt.y + br.x)
		- *(pInterlTr + IntelImg->nWid * br.y + lt.x);

	return nSumVal;
}


/*
I/O:	    Name		          Type	     		  Content

[in]	    pSrcImg		          const WissenImage*		  input image buffer.
[in/out]	pDstImg		          WissenImage*		      output resized image buffer.

Realized function:
+ Get the resized image buffer;
*/
void mvResizeImg(const WissenImage *pSrcImg, int *arr_y, int *arr_x, WissenImage *pDstImg)
{
	int i, j;
	float fScale;
	float fSum;
	unsigned char *Ptr = 0;
	unsigned char *PSrc = 0;
	int width, height;

	width = pDstImg->nWid;
	height = pDstImg->nHig;

	fScale = pSrcImg->nWid / (width + 0.001f);

	fSum = -fScale;
	for (j = 0; j < height; j++)
	{
		fSum += fScale;

		arr_y[j] = (int)(fSum + 0.5f);

		if (j < width)
		{
			arr_x[j] = arr_y[j];
		}
	}

	if (width > height)
	{
		for (i = height; i < width; i++)
		{
			fSum += fScale;

			arr_x[i] = (int)(fSum + 0.5f);
		}
	}

	for (j = 0; j < height; j++)
	{
		Ptr = pDstImg->data + width * j;

		PSrc = pSrcImg->data + pSrcImg->nWid * arr_y[j];

		for (i = 0; i < width; i++)
		{
			Ptr[i] = PSrc[arr_x[i]];
		}
	}
}

int findSymmetryAxisX(unsigned char *src, int width, int height)
{
	int axisX = -1;
	int secaxisX = -1;
	int half_width, x;
	float minHs = 100000;
	float secMinHS = 100000;

	for (x = width * 1 / 4; x < width * 3 / 4; x++)
	{
		float HS = 0;
		int count = 0;
		int step, y;
		half_width = WS_MIN(x, width - x);
		for (step = 1; step < half_width; step++)
		{
			for (y = height / 4; y < height; y += 2)
			{
				unsigned char* ptr = src + y * width;
				int neg = x - step;
				int pos = x + step;
				unsigned char Gneg = ptr[neg];
				unsigned char Gpos = ptr[pos];
				HS += abs(Gneg - Gpos);
				count++;
			}
		}
		HS /= (count + 1e-10f);

		if (HS < minHs)
		{
			secMinHS = minHs;
			minHs = HS;
			secaxisX = axisX;
			axisX = x;
		}
	}

	if (secMinHS >= 1.2 * minHs)
	{
		return axisX;
	}
	else
	{
		if (abs(secaxisX - axisX) < width * 0.1)
		{
			return axisX;
		}
		else
		{
			return 0;
		}
	}
}

/*
I/O:	    Name		        Type	     		  Content

[in]	    img		            const WissenImage	      input image
[in/out]	pTempimg		    WissenImage*	              Temple image
[in]	    pTempRec		    AdasRect*	          Temple rect acorrding to img.

Realized function:
+ Get the temple image from img
*/
void mvSelcTemplatByCen(const WissenImage img, WissenImage *pTempimg, WissenObjectRect *pTempRec)
{
	int j;
	Wissen16SPoint lt;
	Wissen16SPoint imgCen;
	unsigned char *pTr = 0;
	unsigned char *pSrcTr = 0;

	pTempimg->nWid = WS_MAX(2, (int)(0.4 * WS_MIN(img.nWid, img.nHig)));
	pTempimg->nHig = pTempimg->nWid;

	imgCen.x = img.nWid >> 1;
	imgCen.y = img.nHig >> 1;

	lt.x = imgCen.x - (pTempimg->nWid >> 1);
	lt.y = imgCen.y - (pTempimg->nWid >> 1);

	pTempRec->x = lt.x;
	pTempRec->y = lt.y;
	pTempRec->width = pTempimg->nWid;
	pTempRec->height = pTempimg->nHig;

	for (j = 0; j < pTempimg->nHig; j++) {
		pTr = pTempimg->data + pTempimg->nWid * j;

		pSrcTr = img.data + img.nWid * (j + lt.y);

		memcpy(pTr, pSrcTr + lt.x, pTempimg->nWid);

	}
}

unsigned char matchByTemple(const WissenImage *pimg, const WissenImage *pTempimg,
	WissenObjectRect SearcRec, WissenObjectRect *pMatchRec, int nPubliBuffOff, int *tempSpace)
{
	WissenImage IntelImg;
	int m, n, i, j;
	int addresoff = 0;
	unsigned char* pTemplateptr = 0;
	int* pdata = 0;
	float* pSimlarTr = 0;
	WissenImage tempSubAvg;
	WissenImage MathchSimilar;
	int nTemplateAvg = 0;
	unsigned char* ptr = (unsigned char*)tempSpace + nPubliBuffOff;
	int powT = 0;
	int ntemplateSum = pTempimg->nHig * pTempimg->nWid;
	Wissen16SPoint lt, br;
	WissenPoint tempPoint;
	int RioSubAvgSum = 0;
	float fMatchVal = 0.0f;
	int nAvg;
	unsigned char *pMatchRio = 0;
	float fMaxMatchSim = 0.0f;
	//float fSecdMaxMatchSim = 0.0f;
	//float fMinMatchSim = 1.0f;
	int nMaxExternByte = MAX_EXTERN_SPACE_SIZE * sizeof(int);

	IntelImg.data = ptr;

	if (SearcRec.width < pTempimg->nWid || SearcRec.height < pTempimg->nHig) {
		return 0;
	}

	IntelImg.nWid = SearcRec.width + 1;
	IntelImg.nHig = SearcRec.height + 1;
	memset(IntelImg.data, 0, IntelImg.nWid * IntelImg.nHig * sizeof(int));

	addresoff +=
		ADAS_ALIGN_16BYTE_SIZE(IntelImg.nWid * IntelImg.nHig * sizeof(int));
	ptr += addresoff;

	if (nPubliBuffOff + addresoff > nMaxExternByte) {
		return 0;
	}

	tempPoint.x = SearcRec.x;
	tempPoint.y = SearcRec.y;
	mvInterlimg(pimg, tempPoint, &IntelImg);

	tempSubAvg.data = ptr;
	tempSubAvg.nWid = pTempimg->nWid;
	tempSubAvg.nHig = pTempimg->nHig;
	addresoff +=
		ADAS_ALIGN_16BYTE_SIZE(pTempimg->nWid * pTempimg->nHig * sizeof(int));
	ptr += addresoff;

	//
	MathchSimilar.data = ptr;
	MathchSimilar.nWid = SearcRec.width - pTempimg->nWid;
	MathchSimilar.nHig = SearcRec.height - pTempimg->nHig;
	addresoff +=
		ADAS_ALIGN_16BYTE_SIZE(MathchSimilar.nWid * MathchSimilar.nHig * sizeof(float));

	if (nPubliBuffOff + addresoff > nMaxExternByte) {
		return 0;
	}

	for (m = 0; m < pTempimg->nHig; m++) {
		pTemplateptr = pTempimg->data + pTempimg->nWid * m;

		for (n = 0; n < pTempimg->nWid; n++) {
			nTemplateAvg += pTemplateptr[n];
		}
	}

	nTemplateAvg /= ntemplateSum;

	for (m = 0; m < tempSubAvg.nHig; m++) {
		pdata = (int*)tempSubAvg.data + tempSubAvg.nWid * m;

		pTemplateptr = pTempimg->data + pTempimg->nWid * m;

		for (n = 0; n < tempSubAvg.nWid; n++) {
			pdata[n] = pTemplateptr[n] - nTemplateAvg;
			powT += pdata[n] * pdata[n];
		}
	}

	//match
	pMatchRec->width = pTempimg->nWid;
	pMatchRec->height = pTempimg->nHig;

	for (m = SearcRec.y; m < SearcRec.y + SearcRec.height - pTempimg->nHig;
		m++) {
		for (n = SearcRec.x; n < SearcRec.x + SearcRec.width - pTempimg->nWid;
			n++) {
			lt.x = n - SearcRec.x;
			lt.y = m - SearcRec.y;
			br.x = n - SearcRec.x + pTempimg->nWid; //-1 + 1
			br.y = m - SearcRec.y + pTempimg->nHig;

			nAvg = mvSumImgRio(&IntelImg, lt, br);

			nAvg = nAvg / ntemplateSum;

			RioSubAvgSum = 0;
			fMatchVal = 0.0f;

			for (j = 0; j < tempSubAvg.nHig; j++) {
				pdata = (int*)tempSubAvg.data + tempSubAvg.nWid * j;

				pMatchRio = pimg->data + pimg->nWid * (m + j);

				for (i = 0; i < tempSubAvg.nWid; i++) {
					RioSubAvgSum += (pMatchRio[i + n] - nAvg)
						* (pMatchRio[i + n] - nAvg);
					fMatchVal += pdata[i] * (pMatchRio[i + n] - nAvg);
				}
			}

			fMatchVal = fMatchVal / sqrtf((float)powT * RioSubAvgSum);

			pSimlarTr = (float*)MathchSimilar.data
				+ MathchSimilar.nWid * (int)(lt.y);
			pSimlarTr[(int)(lt.x)] = fMatchVal;

			if (fMatchVal > fMaxMatchSim) 
			{
				pMatchRec->x = n;
				pMatchRec->y = m;
				fMaxMatchSim = fMatchVal;
			}

		}
	}

	return 1;
}

unsigned char mvTemplatMatch(const WissenImage img, const WissenImage Tempimg,
	WissenObjectRectTracked SearcRec, WissenObjectRectTracked *pMatchRec, int nPubliBuffOff,
	unsigned char bComparSecd, int* tempSpace, float *fMatchScore)
{
	WissenImage IntelImg;
	int m, n, i, j;
	int addresoff = 0;
	unsigned char* pTemplateptr = 0;
	int* pdata = 0;
	float* pSimlarTr = 0;
	WissenImage tempSubAvg;
	WissenImage MathchSimilar;
	int nTemplateAvg = 0;
	unsigned char* ptr = (unsigned char*)tempSpace + nPubliBuffOff;
	int powT = 0;
	int ntemplateSum = Tempimg.nHig * Tempimg.nWid;
	Wissen16SPoint lt, br;
	int RioSubAvgSum = 0;
	float fMatchVal = 0.0f;
	int nAvg;
	unsigned char *pMatchRio = 0;
	float fMaxMatchSim = 0.0f;
	float fSecdMaxMatchSim = 0.0f;
	//float fMinMatchSim = 1.0f;
	WissenPoint point;
	int nMax_PublicSpace_byte = MAX_PUBLIC_SPACE_SIZE * sizeof(int);
	*fMatchScore = 0;

	IntelImg.data = ptr;

	if (SearcRec.object.width < Tempimg.nWid || SearcRec.object.height < Tempimg.nHig
		|| ntemplateSum < 10) {
		return 0;
	}

	IntelImg.nWid = SearcRec.object.width + 1;
	IntelImg.nHig = SearcRec.object.height + 1;
	memset(IntelImg.data, 0, IntelImg.nWid * IntelImg.nHig * sizeof(int));

	addresoff +=
		ADAS_ALIGN_16BYTE_SIZE(IntelImg.nWid * IntelImg.nHig * sizeof(int));
	ptr += addresoff;

	if (addresoff + nPubliBuffOff > nMax_PublicSpace_byte) {
		return 0;
	}

	point.x = SearcRec.object.x;
	point.y = SearcRec.object.y;
	mvInterlimg(&img, point, &IntelImg);

	tempSubAvg.data = ptr;
	tempSubAvg.nWid = Tempimg.nWid;
	tempSubAvg.nHig = Tempimg.nHig;
	addresoff +=
		ADAS_ALIGN_16BYTE_SIZE(Tempimg.nWid * Tempimg.nHig * sizeof(int));
	ptr += addresoff;

	MathchSimilar.data = ptr;
	MathchSimilar.nWid = SearcRec.object.width - Tempimg.nWid;
	MathchSimilar.nHig = SearcRec.object.height - Tempimg.nHig;
	addresoff +=
		ADAS_ALIGN_16BYTE_SIZE(MathchSimilar.nWid * MathchSimilar.nHig * sizeof(float));

	if (addresoff + nPubliBuffOff > nMax_PublicSpace_byte) {
		return 0;
	}

	for (m = 0; m < Tempimg.nHig; m++) {
		pTemplateptr = Tempimg.data + Tempimg.nWid * m;

		for (n = 0; n < Tempimg.nWid; n++) {
			nTemplateAvg += pTemplateptr[n];
		}
	}

	nTemplateAvg /= ntemplateSum;

	for (m = 0; m < tempSubAvg.nHig; m++) {
		pdata = (int*)tempSubAvg.data + tempSubAvg.nWid * m;

		pTemplateptr = Tempimg.data + Tempimg.nWid * m;

		for (n = 0; n < tempSubAvg.nWid; n++) {
			pdata[n] = pTemplateptr[n] - nTemplateAvg;
			powT += pdata[n] * pdata[n];
		}
	}

	pMatchRec->object.width = Tempimg.nWid;
	pMatchRec->object.height = Tempimg.nHig;

	for (m = SearcRec.object.y; m <= SearcRec.object.y + SearcRec.object.height - Tempimg.nHig;
		m++) {
		for (n = SearcRec.object.x; n <= SearcRec.object.x + SearcRec.object.width - Tempimg.nWid;
			n++) {
			lt.x = n - SearcRec.object.x;
			lt.y = m - SearcRec.object.y;
			br.x = n - SearcRec.object.x + Tempimg.nWid; //-1 + 1
			br.y = m - SearcRec.object.y + Tempimg.nHig;

			nAvg = mvSumImgRio(&IntelImg, lt, br);

			nAvg = nAvg / ntemplateSum;

			RioSubAvgSum = 0;
			fMatchVal = 0.0f;

			for (j = 0; j < tempSubAvg.nHig; j++) {
				pdata = (int*)tempSubAvg.data + tempSubAvg.nWid * j;

				pMatchRio = img.data + img.nWid * (m + j);

				for (i = 0; i < tempSubAvg.nWid; i++) {
					RioSubAvgSum += (pMatchRio[i + n] - nAvg)
						* (pMatchRio[i + n] - nAvg);
					fMatchVal += pdata[i] * (pMatchRio[i + n] - nAvg);
				}
			}

			fMatchVal = fMatchVal / sqrtf((float)powT * RioSubAvgSum);

			pSimlarTr = (float*)MathchSimilar.data
				+ MathchSimilar.nWid * (int)(lt.y);
			pSimlarTr[(int)(lt.x)] = fMatchVal;

			if (fMatchVal > fMaxMatchSim) {
				pMatchRec->object.x = n;
				pMatchRec->object.y = m;
				fMaxMatchSim = fMatchVal;

			}

		}
	}

	*fMatchScore = fMaxMatchSim;

	if (!bComparSecd) {
		if (fMaxMatchSim > 0.8f) {
			return 1;
		}

		return 0;
	}

	for (m = 0; m < MathchSimilar.nHig; m++) {
		pSimlarTr = (float*)MathchSimilar.data + MathchSimilar.nWid * m;

		for (n = 0; n < MathchSimilar.nWid; n++) {
			point.x = n + pMatchRec->object.x;
			point.y = m + pMatchRec->object.y;

			if (!isPointInRect(&point, &pMatchRec->object)) {
				if (pSimlarTr[n] > fSecdMaxMatchSim) {
					fSecdMaxMatchSim = pSimlarTr[n];
				}
			}
		}
	}


	if (fMaxMatchSim > 0.8f && (fMaxMatchSim - fSecdMaxMatchSim) > 0.1f) {
		return 1;
	}

	return 0;

}

void mvDerivemgRio(WissenImage * pSrcImg, WissenImage * RioImg, WissenObjectRect RioRec) 
{

	int j;
	unsigned char *pDstr = 0;
	unsigned char *pSrctr = 0;

	RioImg->nWid = RioRec.width;
	RioImg->nHig = RioRec.height;

	for (j = RioRec.y; j < RioRec.y + RioRec.height; j++) 
	{
		pDstr = RioImg->data + RioImg->nWid * (j - RioRec.y);

		pSrctr = pSrcImg->data + pSrcImg->nWid * j;

		memcpy(pDstr, pSrctr + RioRec.x, RioRec.width);
	}

}