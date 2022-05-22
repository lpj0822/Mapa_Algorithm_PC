#include "sort_algorithm.h"
#include <stdio.h>
#include <stdlib.h>

/*
Function process:
+ sort the int in decreasing order
Fan-in :
+ mvReDetcorByRec()
Fan-out:
+ N/A
ATTENTION: __________
*/
void binSort_INT(int *data, int n)
{
	int i, j, left, mid, right;
	int temp;

	for (i = 1; i < n; i++)
	{
		temp = data[i];
		left = 0; right = i - 1;

		while (left <= right)
		{
			mid = ((left + right) >> 1);
			if (temp < data[mid])
			{
				right = mid - 1;
			}
			else {
				left = mid + 1;
			}
		}

		for (j = i - 1; j >= left; j--)
		{
			*(data + j + 1) = *(data + j);
		}

		if (left != i)
		{
			*(data + left) = temp;
		}
	}
}

/*
Function process:
+ sort the float in decreasing order
Fan-in :
+ mvReDetcorByRec()
Fan-out:
+ N/A
ATTENTION: __________
*/
void binSort_FLOAT(float *data, int n)
{
	int i, j, left, mid, right;
	float temp;

	for (i = 1; i < n; i++)
	{
		temp = data[i];
		left = 0; right = i - 1;

		while (left <= right)
		{
			mid = ((left + right) >> 1);
			if (temp < data[mid])
			{
				right = mid - 1;
			}
			else {
				left = mid + 1;
			}
		}

		for (j = i - 1; j >= left; j--)
		{
			*(data + j + 1) = *(data + j);
		}

		if (left != i)
		{
			*(data + left) = temp;
		}
	}
}

/*
Function process:
+ sort the corner in decreasing order by data.val
Fan-in :
+ mvReDetcorByRec()
Fan-out:
+ N/A
ATTENTION: __________
*/
void binSort_Corner(AdasCorner *data, int n)
{
	int i, j, left, mid, right;
	AdasCorner temp;

	for (i = 1; i < n; i++)
	{
		temp = data[i];
		left = 0; right = i - 1;

		while (left <= right)
		{
			mid = ((left + right) >> 1);
			if (temp.val > data[mid].val)
			{
				right = mid - 1;
			}
			else {
				left = mid + 1;
			}
		}

		for (j = i - 1; j >= left; j--)
		{
			*(data + j + 1) = *(data + j);
		}

		if (left != i)
		{
			*(data + left) = temp;
		}
	}

}