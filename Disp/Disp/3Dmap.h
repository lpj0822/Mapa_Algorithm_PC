#ifndef FUNCTION_H
#define FUNCTION_H

#define Is_3DUI 0

//这边修改是为了兼容下位机
#define Is_PC 0
#define Is_DEBUG 0

#if defined(WIN32) && !defined(__cplusplus)

#define inline __inline

#endif

#if Is_PC

#include <opencv2\opencv.hpp>

using namespace cv;
using namespace std;



#endif
/***
*这边修改是为了兼容下位机
*/
#include <time.h>
#include "DISP_Interface.h"
#include "3Dmap_Interface.h" 
#include "CMulitTrack.h"
#if Is_PC
extern "C"
{

//#include "Fcw_Tracker.h"
	//#include "LDWS_Interface.h"
}
#else




#endif

//#define IMAGE_HEIGHT 340

#define LEVEL_1 30
#define LEVEL_2 60
#define LEVEL_3 120

//宏定义图像大小
#define IMG_W 320
#define IMG_H 128


#define COM_H 256 //100

#define OBJ_AMUNT 20
#define OBJ_SIZE_FILTER 5
#define OBJ_HEIGHT_FILTER 15

#define U_COM_INDEX 0
#define V_COM_INDEX 10

//forshow
#define SKYLINE_X 112.0
#define SKYLINE_Y 32.0
#define PROJ_DISP 124.0


#define UX_SCALE 15.0
#define UX_WIDTH 300

#define HISTOSIZE_X 750
#define HISTOSIZE_Y 200
#define HISTOSIZE_Z 256

#define MAX_X 2000.0
#define MIN_X -1000.0
#define STEP_X 4.0

#define MAX_Y 500.0
#define MIN_Y -500.0
#define STEP_Y 5.0

#define DISP2DIST  384 //768

#define HEIGHT_THD 60.0


inline static int maxof(int x, int y) {
	return ((x > y) ? x : y);
}

inline static int minof(int x, int y) {
	return ((x > y) ? y : x);
}

inline static float maxof_float(float x, float y) {
	return ((x > y) ? x : y);
}

inline static float minof_float(float x, float y) {
	return ((x > y) ? y : x);
}

inline static unsigned char maxof_uchar(unsigned char x, unsigned char y) {
	return ((x > y) ? x : y);
}

inline static unsigned char minof_uchar(unsigned char x, unsigned char y) {
	return ((x > y) ? y : x);
}

typedef struct Object
{
	int cnt;
	//int cnt_revised[IMAGE_HEIGHT];

	short size_right;
	short size_left;
	short size_up;
	short size_down;




	unsigned char biggest_disp;

	unsigned char smallest_disp;

	float proj_right;
	float proj_left;
	float proj_up;
	float proj_down;

	float dist;

	float proj_size;

	unsigned char isBackground;
	unsigned char isSelected;

	short tracked_cnt;

	short histo_X[HISTOSIZE_X];
	short histo_Y[HISTOSIZE_Y];
	short histo_Z[HISTOSIZE_Z];

#if Is_PC

	Object(){
		cnt = 0;
		size_right = 0;
		size_left = 0;
		size_up = 0;
		size_down = 0;

		biggest_disp = 0;
		dist = 0;


		smallest_disp = 0;

		proj_size = 0;

		isBackground = 0;
		tracked_cnt = 0;

	}

#endif

}Object;

typedef struct Mat_int
{
	int *data;
	int rows;
	int cols;
}Mat_int;
void doInitialMatInt(Mat_int* matint, int rows, int cols);
void doReleaseMatInt(Mat_int* matint);

typedef struct Mat_short
{
	short *data;
	int rows;
	int cols;
}Mat_short;
void doInitialMatShort(Mat_short* matshort, int rows, int cols);
void doReleaseMatShort(Mat_short* matshort);

typedef struct Mat_uchar
{
	unsigned char *data;
	int rows;
	int cols;
}Mat_uchar;
void doInitialMatUchar(Mat_uchar* matuchar, int rows, int cols);
void doReleaseMatUchar(Mat_uchar* matuchar);

typedef struct COLOR
{
	unsigned char blue;
	unsigned char green;
	unsigned char red;
}COLOR;

typedef struct Mat_color
{
	COLOR *data;
	int rows;
	int cols;
}Mat_color;



void doInitMatUX(const Mat_short DispImg, const Mat_short DXImg);
short getX(int value, int u);
//float getvalinv(int value);

void doInitialMatColor(Mat_color* matcolor, int rows, int cols);
void doReleaseMatColor(Mat_color* matcolor);


void doCalcDistImg(const Mat_short src, Mat_uchar *dst, const Mat_uchar vmask, const Mat_uchar umask);

void doCalcUProjImg(const Mat_short src, Mat_short *dst, const Mat_uchar vmask);
void doCalcDXimg(Mat_short *src, Mat_short *dst, const Mat_uchar vmask);

void Init_CalcObstacle();
void doCalcObstacle(const Mat_short src, Mat_short* lableImg);

void doFindConection(const Mat_uchar src, Mat_short *lableImg);

void doExtendConection(const Mat_uchar src, Mat_short *lableImg);

void getLUT(int x, int y, int *ref_x, int *ref_y);
void getBoundRect(int x, int y, int w, int h, int *up, int *down, int *left, int *right);


void Init_TrackObstacle();
void doTrackObstacle(Object cur_objs[]);
void doHistoResize(Object cur_objs[], int width);



void getDistImg(Mat_short *DispImg);
float getdist(int x, int y, int w, int h);


#if Is_PC

void doFilterObstacle(const Mat_short ObtImg, const Mat_short DispImg, Mat_color OriImg, Mat &OriImg_forshow);

void doDrawObjects(Object cur_objs[], const Mat_short ObtImg, const Mat_short DispImg, Mat &OriImg_forshow);

#else

void doFilterObstacle(const Mat_short ObtImg, const Mat_short DispImg, Mat_color OriImg);

#endif


#define  MAX_MOTION_NUM 8

const static COLOR colors2[18] = { { 200, 100, 100 },
{ 100, 200, 100 },
{ 100, 100, 200 },
{ 200, 200, 100 },
{ 200, 100, 200 },
{ 100, 200, 200 },

{ 200, 150, 250 },
{ 50, 250, 150 },
{ 100, 250, 250 },
{ 150, 250, 0 },
{ 250, 100, 200 },
{ 200, 250, 100 },

{ 250, 150, 0 },
{ 0, 200, 250 },
{ 100, 0, 250 },
{ 250, 50, 200 },
{ 0, 250, 50 },
{ 250, 200, 0 }
};
/*
const static uchar COM_ARRAY[300] = { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,
20,21,22,23,24,25,26,27,28,29,30,30,31,31,32,32,33,33,34,34,
35,35,36,36,37,37,38,38,39,39,40,40,41,41,42,42,43,43,44,44,
45,45,46,46,47,47,48,48,49,49,50,50,51,51,52,52,53,53,54,54,
55,55,56,56,57,57,58,58,59,59,60,60,61,61,62,62,63,63,64,64,
65,65,66,66,67,67,68,68,69,69,70,70,71,71,72,72,73,73,74,74,
75,75,76,76,77,77,78,78,79,79,80,80,81,81,82,82,83,83,84,84,
85,85,86,86,87,87,88,88,89,89,90,90,91,91,92,92,93,93,94,94,
95,95,96,96,97,97,98,98,99,99,99,99,99,99,99,99,99,99,99,99,
99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99 };
*/
const static unsigned char COM_ARRAY[300] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
	                                 41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,
									 81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,
									 121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,
									 161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,198,199,200,
									 201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,
									 241,242,243,244,245,246,247,248,249,250,251,252,253,254,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
									 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255};

const static int COLOR_ARRAY[256] = { 255,255,255,255,255,255,255,255,255,255,255,255,528886,1915617,3104463,4095424,5020082,5812390,6538907,7133586,
                                      7727753,8256129,8716154,9109364,9502574,9830249,10143844,10468959,10729051,10989143,11249235,11509583,11704652,11899465,12094790,12289859,12419649,12614974,12744764,12874810,
                                      13004856,13134902,13264948,13394994,13525296,13590063,13720109,13850411,13915434,13980201,14110503,14175526,14240293,14305316,14435618,14500641,14565664,14630687,14695710,14760733,
                                      14825756,14825500,14890523,14955546,15020825,15085848,15150871,15150615,15215894,15280917,15280661,15345684,15410963,15410707,15475986,15475730,15540753,15606032,15605776,15671055,
                                      15670799,15736078,15735822,15801101,15800845,15866124,15865868,15865868,15931147,15930891,15996170,15995914,16061193,16061193,16060937,16126216,16126216,16125960,16191239,16190983,
                                      16190983,16256262,16256262,16256006,16321285,16321285,16321029,16321029,16386308,16386052,16386052,16451331,16451331,16451075,16451075,16516354,16516354,16516098,16516098,16581377,
                                      16581377,16581121,16581121,16581121,16646400,16646400,16646144,16646144,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,
                                      16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,
                                      16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,
                                      16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,
                                      16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,
                                      16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,
                                      16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680,16711680 };

//const static int COLOR_ARRAY[256] = { 255, 66302, 132349, 198396, 264443, 330490, 396537, 462584, 528631, 594678, 660725, 726772, 792819, 858866, 924913, 990960, 1057007, 1123054, 1189101, 1255148, 1321195, 1387242, 1453289, 1519336, 1585383,
//1651430, 1717477, 1783524, 1849571, 1915618, 1981665, 2047712, 2113759, 2179806, 2245853, 2311900, 2377947, 2443994, 2510041, 2576088, 2642135, 2708182, 2774229, 2840276, 2906323, 2972370, 3038417, 3104464,
//3170511, 3236558, 3302605, 3368652, 3434699, 3500746, 3566793, 3632840, 3698887, 3764934, 3830981, 3897028, 3963075, 4029122, 4095169, 4161216, 4227263, 4293310, 4359357, 4425404, 4491451, 4557498, 4623545,
//4689592, 4755639, 4821686, 4887733, 4953780, 5019827, 5085874, 5151921, 5217968, 5284015, 5350062, 5416109, 5482156, 5548203, 5614250, 5680297, 5746344, 5812391, 5878438, 5944485, 6010532, 6076579, 6142626,
//6208673, 6274720, 6340767, 6406814, 6472861, 6538908, 6604955, 6671002, 6737049, 6803096, 6869143, 6935190, 7001237, 7067284, 7133331, 7199378, 7265425, 7331472, 7397519, 7463566, 7529613, 7595660, 7661707,
//7727754, 7793801, 7859848, 7925895, 7991942, 8057989, 8124036, 8190083, 8256130, 8322177, 8388224, 8388735, 8519038, 8584061, 8649084, 8714107, 8779130, 8844153, 8909176, 8974199, 9039222, 9104245, 9169268,
//9234291, 9299314, 9364337, 9429360, 9494383, 9559406, 9624429, 9689452, 9754475, 9819498, 9884521, 9949544, 10014567, 10079590, 10144613, 10209636, 10274659, 10339682, 10404705, 10469728, 10534751, 10599774,
//10664797, 10729820, 10794843, 10859866, 10924889, 10989912, 11054935, 11119958, 11184981, 11250004, 11315027, 11380050, 11445073, 11510096, 11575119, 11640142, 11705165, 11770188, 11835211, 11900234, 11965257,
//12030280, 12095303, 12160326, 12225349, 12290372, 12355395, 12420418, 12485441, 12550464, 12615487, 12680510, 12745533, 12810556, 12875579, 12940602, 13005625, 13070648, 13135671, 13200694, 13265717, 13330740,
//13395763, 13460786, 13525809, 13590832, 13655855, 13720878, 13785901, 13850924, 13915947, 13980970, 14045993, 14111016, 14176039, 14241062, 14306085, 14371108, 14436131, 14501154, 14566177, 14631200, 14696223,
//14761246, 14826269, 14891292, 14956315, 15021338, 15086361, 15151384, 15216407, 15281430, 15346453, 15411476, 15476499, 15541522, 15606545, 15671568, 15736591, 15801614, 15866637, 15931660, 15996683, 16061706,
//16126729, 16191752, 16256775, 16321798, 16386821, 16451844, 16516867, 16581890, 16646913, 16711936 };


#endif // !FUNCTION_H
