#include "saveResult.h"

static void writeLDResult(FILE *writeFile, LDWS_Output *pTest)
{
	int index = 0;
	int alarm_result = 0;
	if (pTest == NULL)
	{
		fprintf(writeFile, "{\"LD\":{\"ldCount\":0,\"alarm\":0,\"ldResult\":[]}}\n");
		return;
	}
	alarm_result = pTest->alarm_result;
	if (pTest->Route == 1 && pTest->Route_half == 0 && pTest->Confidence_detection[0] > pTest->Confidence - KEEP_FRAME)
	{
		fprintf(writeFile, "{\"LD\":{\"ldCount\":%d,\"alarm\":%d,\"ldResult\":[", 2 * pTest->NB_INTERVALLES, alarm_result);
		for (index = 0; index < pTest->NB_INTERVALLES; index++)
		{
			if (0 != index)
			{
				fprintf(writeFile, ",");
			}
			fprintf(writeFile, "{\"x\":%d,\"y\":%d,\"position\":%d}",
				pTest->pCaPoint[index].x, pTest->pCaPoint[index].y, 0);
		}
		for (index = pTest->NB_INTERVALLES; index < 2 * pTest->NB_INTERVALLES; index++)
		{
			if (0 != index)
			{
				fprintf(writeFile, ",");
			}
			fprintf(writeFile, "{\"x\":%d,\"y\":%d,\"position\":%d}",
				pTest->pCaPoint[index].x, pTest->pCaPoint[index].y, 1);
		}
		fprintf(writeFile, "]}}\n");
	}
	else if (pTest->Route == 2 && (pTest->Confidence_detection[2] > pTest->Confidence - KEEP_FRAME))
	{
		fprintf(writeFile, "{\"LD\":{\"ldCount\":%d,\"alarm\":%d,\"ldResult\":[", pTest->NB_INTERVALLES, alarm_result);
		for (index = 0; index < pTest->NB_INTERVALLES; index++)
		{
			if (0 != index)
			{
				fprintf(writeFile, ",");
			}
			fprintf(writeFile, "{\"x\":%d,\"y\":%d,\"position\":%d}",
				pTest->pCaPoint[index].x, pTest->pCaPoint[index].y, 0);
		}
		fprintf(writeFile, "]}}\n");
	}
	else if (pTest->Route == 3 && (pTest->Confidence_detection[3] > pTest->Confidence - KEEP_FRAME))
	{
		fprintf(writeFile, "{\"LD\":{\"ldCount\":%d,\"alarm\":%d,\"ldResult\":[", pTest->NB_INTERVALLES, alarm_result);
		for (index = pTest->NB_INTERVALLES; index < 2 * pTest->NB_INTERVALLES; index++)
		{
			if (pTest->NB_INTERVALLES != index)
			{
				fprintf(writeFile, ",");
			}
			fprintf(writeFile, "{\"x\":%d,\"y\":%d,\"position\":%d}",
				pTest->pCaPoint[index].x, pTest->pCaPoint[index].y, 1);
		}
		fprintf(writeFile, "]}}\n");
	}
	else if (pTest->Route_half == 1 && pTest->Confidence_detection[1] > pTest->Confidence - KEEP_FRAME)
	{
		fprintf(writeFile, "{\"LD\":{\"ldCount\":%d,\"alarm\":%d,\"ldResult\":[", 2 * pTest->NB_INTERVALLES - 6, alarm_result);
		for (index = 3; index < pTest->NB_INTERVALLES; index++)
		{
			if (3 != index)
			{
				fprintf(writeFile, ",");
			}
			fprintf(writeFile, "{\"x\":%d,\"y\":%d,\"position\":%d}",
				pTest->pCaPoint[index].x, pTest->pCaPoint[index].y, 0);
		}
		for (index = pTest->NB_INTERVALLES + 3; index < 2 * pTest->NB_INTERVALLES; index++)
		{
			if (0 != index)
			{
				fprintf(writeFile, ",");
			}
			fprintf(writeFile, "{\"x\":%d,\"y\":%d,\"position\":%d}",
				pTest->pCaPoint[index].x, pTest->pCaPoint[index].y, 1);
		}
		fprintf(writeFile, "]}}\n");
	}
	else
	{
		fprintf(writeFile, "{\"LD\":{\"ldCount\":0,\"alarm\":0,\"ldResult\":[]}}\n");
	}
}

int writeTrackerReault(FILE *writeFile, int frameNo, float fzoom, MuliTracker *pTrackOutput, LDWS_Output *pTest)
{
	int index = 0;
	int flag = 0;
	int objectCount = 0;
	fprintf(writeFile, "freameNo:%d\n", frameNo);

	writeLDResult(writeFile, pTest);

	if (pTrackOutput == NULL)
	{
		fprintf(writeFile, "{\"FC\":{\"fcCount\":0,\"fcResult\":[]}}\n");
	}
	else
	{
		for (index = 0; index < pTrackOutput->nTrackeNum; index++)
		{
			trakobj pGroup = *(pTrackOutput->pTrackerset + index);

			if (!pGroup.bTrue)
			{
				continue;
			}
			objectCount++;
		}
		fprintf(writeFile, "{\"FC\":{\"fcCount\":%d,\"fcResult\":[", objectCount);
		for (index = 0; index < pTrackOutput->nTrackeNum; index++)
		{
			trakobj pGroup = *(pTrackOutput->pTrackerset + index);

			if (!pGroup.bTrue)
			{
				continue;
			}


			pGroup.objRec.object.x *= fzoom;
			pGroup.objRec.object.y *= fzoom;
			pGroup.objRec.object.height *= fzoom;
			pGroup.objRec.object.width *= fzoom;

			if (0 != flag)
			{
				fprintf(writeFile, ",");
			}

			fprintf(writeFile, "{\"x\":%d,\"y\":%d,\"wid\":%d,\"hig\":%d}", pGroup.objRec.object.x, pGroup.objRec.object.y, pGroup.objRec.object.width, pGroup.objRec.object.height);
			flag = 1;
		}
		fprintf(writeFile, "]}}\n");
	}

	return 0;
}

int writeDetecctorReault(FILE *writeFile, int frameNo, objectSetsCar *pFCWSOutput, LDWS_Output *pTest)
{
	int index = 0;
	int flag = 0;
	int objectCount = 0;
	fprintf(writeFile, "freameNo:%d\n", frameNo);

	writeLDResult(writeFile, pTest);

	if (pFCWSOutput == NULL)
	{
		fprintf(writeFile, "{\"FC\":{\"fcCount\":0,\"fcResult\":[]}}\n");
	}
	else
	{
		for (index = 0; index < pFCWSOutput->nObjectNum; index++)
		{
			if (pFCWSOutput->objects[index].confidence < 0)
				continue;
			objectCount++;
		}
		fprintf(writeFile, "{\"FC\":{\"fcCount\":%d,\"fcResult\":[", objectCount);
		for (index = 0; index < pFCWSOutput->nObjectNum; index++)
		{
			WissenObjectRect object = pFCWSOutput->objects[index];

			if (object.confidence < 0)
				continue;

			if (0 != flag)
			{
				fprintf(writeFile, ",");
			}

			fprintf(writeFile, "{\"x\":%d,\"y\":%d,\"wid\":%d,\"hig\":%d,\"confidence\":%d}", object.x, object.y,
				object.width, object.height, object.confidence);
			flag = 1;
		}
		fprintf(writeFile, "]}}\n");
	}

	return 0;
}

int write3DMapResult(FILE *writeFile, int frameNo, FENCE_LIST drawFrences)
{
	int index = 0;
	int flag = 0;
	int x = 0;
	int y = 0;
	int width = 0;
	int height = 0;
	int color = 0;
	fprintf(writeFile, "freameNo:%d\n", frameNo);
	if (drawFrences.nFenceNum > 0)
	{
		fprintf(writeFile, "{\"DISP\":{\"dispCount\":%d,\"dispResult\":[", drawFrences.nFenceNum);
		for (index = 0; index < drawFrences.nFenceNum; index++)
		{
			if (flag != 0)
			{
				fprintf(writeFile, ",");
			}
			x = drawFrences.frences[index].frame2D.left;
			y = drawFrences.frences[index].frame2D.up;
			width = drawFrences.frences[index].frame2D.right - drawFrences.frences[index].frame2D.left;
			height = drawFrences.frences[index].frame2D.down - drawFrences.frences[index].frame2D.up;
			color = drawFrences.frences[index].nColor;

			fprintf(writeFile, "{\"x\":%d,\"y\":%d,\"wid\":%d,\"hig\":%d, \"color\":%d}", x, y, width, height, color);
			flag = 1;
		}
		fprintf(writeFile, "]}}\n");
	}
	else
	{
		fprintf(writeFile, "{\"DISP\":{\"dispCount\":0,\"dispResult\":[]}}\n");
	}
	return 0;
}