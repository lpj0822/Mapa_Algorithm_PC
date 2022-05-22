/*******************************************************************************
*
* Copyright (c) 2011-2013  Wissen Company             ��������������           
* Wuxi Wissen Intelligent Sensing Technology Co., Ltd.��Ȩ���� 2011-2013    
*                                                                           
* PROPRIETARY RIGHTS of Wissen Company are involved in the  ������������       
* subject matter of this material.  All manufacturing, reproduction, use,      
* and sales rights pertaining to this subject matter are governed by the       
* license agreement.  The recipient of this software implicitly accepts        
* the terms of the license.                                                    
* ������ĵ�������άɭ��˾���ʲ�,�κ���ʿ�Ķ���ʹ�ñ����ϱ�����              
* ��Ӧ��������Ȩ,�е��������κͽ�����Ӧ�ķ���Լ��.                             
*                                                                              
********************************************************************************   
* File Name          : sDas_ldws.c                                           
* Author             : Eason                                
* Revision           : 1.0                                           
* Date               : 14/4/2014 014:07:08                                     
* Description        : ����GB/T 26773-2011��ISO 17361-2007��׼����ƵĻ���άɭ��Ŀ
*                      �ĳ���ƫ�뱨��ϵͳ                        
*                                                                           
* HISTORY***********************************************************************
* Date        | Modification                                            | Author 
* 14/4/2014   | άɭ����ƫ�뱨��ϵͳ1.0                                 | Eason 
*
*******************************************************************************/

/* Includes files *************************************************************/
#include "LDWS_Interface.h"
#include "LDWS_AlarmDecision.h" 
#include <stdio.h>
#include <string.h>

#ifdef _LDWS_ENABLED_
#define LDWS_SUP_REQ_FLG_TLEFT          (0x01)           /* ��ת����Ч */
#define LDWS_SUP_REQ_FLG_TRIGHT         (0x02)           /* ��ת����Ч */
//#define LDWS_SUP_REQ_FLG_AP             (0x08)           /* ����̤���ź���Ч */
//#define LDWS_SUP_REQ_FLG_BRAKE          (0x10)           /* ɲ���ź���Ч */
//#define LDWS_SUP_REQ_FLG_SOT_TOO_MUCH   (0x40)           /* ������ת�ٹ��� */
#define LDWS_SUP_REQ_FLG_WS_VALID       (0x80)           /* �������ٽ�����Ч״̬ */
#define LDWS_SUP_REQ_FLG_TKEEP          (0x100)          /* ����ת��ά��״̬ */
#define LDWS_SUP_REQ_FLG_CHANGE_L       (0X200)          /* ���� */
#define LDWS_SUP_REQ_FLG_CHANGE_R       (0x400)          /* �ұ�� */
#define LDWS_SUP_REQ_VOICE_WARNING      (0x20)           /* �������� */
#define LDWS_SUP_LEFT_THETA             (0X04)           /*����߽Ƕȹ���*/
#define LDWS_SUP_RIGHT_THETA            (0X800)          /*�ұ��߽Ƕȹ���*/
#define CAR_ANGLE_VALUE                 (0x03)               /*ת��ǹ���*/

#define LFW_INTO_LV_WARNING_AREA        (-1)             /* ������ǰ�ֽ�����౨������ */
#define FW_INTO_NO_WARNING_AREA         (0)              /* ����ǰ�ֽ���Ǳ������� */
#define RFW_INTO_RV_WARNING_AREA        (1)              /* ������ǰ�ֽ����Ҳ౨������ */

#define LDWS_STATE_LOW_SPEED            (0x01)          /*������ʻ<60*/
#define LDWS_STATE_HIGH_SPEED           (0x02)          /*������ʻ>60*/
#define LDWS_STATE_CAR_LOAD             (0X04)          /*·���־*/

#define LS_STEERING_ANGLE_WARNING_MAX        (5.0)           /* ���Ʊ��������ǰ��ת�� */
#define LDWS_KEEP_TIME_CHANGE                (60)             /*�������ʱ��Ϊ60֡*/
#define LDWS_LOAD_SIGNAL_LAMP                (10)             /*���̵�·������*/
#define LDWS_LOAD_LOSE_LINE                   (15)            /*�����15֡û������������Ϊ����·��*/
#define NARROW_ROAD_WIDTH                     (2.7)           /*����2.7m���µ�խ��·*/

float leftAngle,leftOffset,rightAngle,rightOffset,roadWval;
int bmp_cnt=0,lineLevel,totalLevel,routeFlag;
int warningNumL = 0;
int warningVectorL[5] = {0};
int warningNumR = 0;
int warningVectorR[5] = {0};
/* Private typedef ************************************************************/
typedef struct                                           
{
	double earliestLine;                                 /* ���籨���� */
	double theresholdLine;                               /* �����ٽ��� */
	double latestLine;                                   /* �������� */

	double speed;                                        /* ��ǰ��Ч���� */
	double rateOfDeparture;                              /* ��ǰƫ���ٶ� */

	unsigned char deparSpeedWarningFlg;                             /* Ϊ1����ʾ��ƫ���ٶȱ�����0,���������ٽ��߱��� */
} WarningAreaInfoStruct;
/* �����������ò��� */
static WarningAreaInfoStruct	gsv_LDWSWarningAreaParam;  /* �����������ò��� */
static int                      giv_LDWSSREQFlg;           /* ���������� */
static char 					roadWidth;                  /* ·���־*/ 
static LDWS_OutputStruct		gsv_LDWSOutput;
static double Car_Width;
//static double Car_Length;


static float GetDWOffsetAndRate(WarningAreaInfoStruct *pWAParam, float theta, float offset)
{
	float temp;
#ifdef _CAN_REC_ENABLED_   
	/* ���ݳ��ټ�ƫ�ǣ���������籨����λ�� */
	if ((speed_60 & LDWS_STATE_HIGH_SPEED)==LDWS_STATE_HIGH_SPEED)
	{	
		pWAParam->rateOfDeparture =
				pWAParam->speed * (float)sin(theta);
		if (pWAParam->rateOfDeparture < 0)
		{
			pWAParam->rateOfDeparture = -pWAParam->rateOfDeparture;
		}
		if (pWAParam->rateOfDeparture > 0.0 && pWAParam->rateOfDeparture <= 0.5)
		{
			pWAParam->theresholdLine = 760.0 + pWAParam->earliestLine*2; //20cm
			
		}
		else if (pWAParam->rateOfDeparture > 0.5 && pWAParam->rateOfDeparture <= 1.0)
		{
			pWAParam->theresholdLine = 1500 * pWAParam->rateOfDeparture + pWAParam->earliestLine*2;//20cm~100cm
			
		}
		else if (pWAParam->rateOfDeparture > 1.0)//1m/s
		{
			pWAParam->theresholdLine = 1500.0 + pWAParam->earliestLine*2;//1m	
		}	
		else if(pWAParam->rateOfDeparture == 0)
		{
	          pWAParam->theresholdLine  = 160; //20cm
	    }
	}
	else if((speed_60 & LDWS_STATE_LOW_SPEED) == LDWS_STATE_LOW_SPEED)
	{
			pWAParam->theresholdLine =20;
	}
#else
	pWAParam->theresholdLine = Car_Width * 0.5;
	 pWAParam->latestLine = -1;
#endif	

	/*if ((float)sin(theta) == 0.0)
	{
		//temp = offset * scale/8;
		temp = offset * scale_tmp - giv_LDWSUserConfig.vpWidth*0.9;
	}
	else
	{
		k = (float)sin(theta) / (float)cos(theta);
		if (k < 0)
		{
			k = -k;
		}	
		temp = offset * scale_tmp - giv_LDWSUserConfig.vpLength * k - giv_LDWSUserConfig.vpWidth;
	}*/
	temp= offset;
	return temp;
}

static char LDWS_flag=0,l_count=0,r_count=0;
static char pltmp=0,r_tmp=0;
static int l_change_bmp=0,r_change_bmp=0;

static int DetWarningArea(WarningAreaInfoStruct *pWAParam)
{
	
	float l_offset,r_offset;
	int sumL = 0;
	int sumR = 0;
	int i = 0;
	float theretmp = 0;
	float camerleftdist = Car_Width / 2;
	float camerrightdist = Car_Width / 2;
	
	double LDWS_WARNING_THERE_L = LDWS_GetLeftDeviation();
	double LDWS_WARNING_THERE_R = LDWS_GetRightDeviation();
	//�ж����Ƿ�ȫ����ʧ�����ȫ����ʧ���п��ܵ��˺��̵�·�ڣ�����5s
	if(routeFlag == 0)
	{
		if(pltmp >LDWS_LOAD_LOSE_LINE && r_tmp == 1)
		{
			pltmp =0;
			r_tmp =0;
		}
	}

	if(routeFlag == 0 && pltmp < LDWS_LOAD_LOSE_LINE)
	{
		pltmp++;
	}
	else if(pltmp >= LDWS_LOAD_LOSE_LINE)
	{
		if(routeFlag)
		{			
			pltmp++;
			r_tmp=1;
			if (pltmp < LDWS_LOAD_SIGNAL_LAMP)
			{			
				return FW_INTO_NO_WARNING_AREA;
			}
			else
			{
				l_count=0;
				r_count=0;
				pltmp=0;
				r_tmp=0;
				giv_LDWSSREQFlg &=  ~LDWS_SUP_REQ_FLG_CHANGE_R;
				giv_LDWSSREQFlg &=  ~LDWS_SUP_REQ_FLG_CHANGE_L;
				
			}		
		}
	}
	else
	{
		pltmp=0;
	}


    //�����߶�����Ч��
	if ((routeFlag) && (lineLevel > (totalLevel - 3)))
    {
		 //����խ����
		if(roadWval < NARROW_ROAD_WIDTH)
		{
			roadWidth |= LDWS_STATE_CAR_LOAD;
		}
		else
		{
			roadWidth &= ~LDWS_STATE_CAR_LOAD;
		}

		// ���㳵����Ե��ǰ�ֱ�Ե�ľ��뼰ƫ���ٶ� 
		l_offset = GetDWOffsetAndRate(pWAParam, leftAngle,leftOffset);

		if(l_offset < 0)
		{
			giv_LDWSSREQFlg |= LDWS_SUP_REQ_FLG_CHANGE_L;
		}

		//�����볬���ٽ�ֵʱ���жϱ���������������
		if((l_offset > (camerleftdist + LDWS_WARNING_THERE_L))&& (r_count != 1))
		{
			 LDWS_flag=0;
			 giv_LDWSSREQFlg &= ~LDWS_SUP_REQ_FLG_CHANGE_R;
			 giv_LDWSSREQFlg &= ~LDWS_SUP_LEFT_THETA;
			 if((giv_LDWSSREQFlg & LDWS_SUP_REQ_VOICE_WARNING) == LDWS_SUP_REQ_VOICE_WARNING)
			 {
				giv_LDWSSREQFlg &= ~LDWS_SUP_REQ_VOICE_WARNING;
			 }
		}
		//������ƫ���ٽ�ֵ������Ѿ���ѹ�߱��������֣�������Ҫ����Զ��0.2�����ٽ������
		if(l_offset > camerleftdist)
		{
	   		if((gsv_LDWSOutput.warningSingal & LDWS_WARNING_LEFT_IMAGE)== LDWS_WARNING_LEFT_IMAGE)
	   		{
				theretmp=(float)LDWS_WARNING_THERE_L;
			}
		}
		// ������Ϊ��౨������ 
		//if (l_offset <= (pWAParam->theresholdLine + theretmp) && l_offset >= pWAParam->latestLine)
		if (l_offset <= (camerleftdist + LDWS_WARNING_THERE_L) && l_offset >= pWAParam->latestLine)
		{	
			l_count = 1;
			r_count = 0;
			//�ж����ߵĽǶȣ��������2��������
			if(leftAngle > 1.6)
			{
				giv_LDWSSREQFlg |= LDWS_SUP_RIGHT_THETA;
				//printf("leftAngle > 1.6 but turn left !");
			}
			else if(leftAngle < -1.6)
			{
				//giv_LDWSSREQFlg |= LDWS_SUP_LEFT_THETA;
				//printf("leftAngle < -1.6 ????????!");
			}
			else
			{
				giv_LDWSSREQFlg &= ~LDWS_SUP_LEFT_THETA;
			}
			//��������
			if((giv_LDWSSREQFlg & LDWS_SUP_REQ_FLG_CHANGE_R) == LDWS_SUP_REQ_FLG_CHANGE_R)
			{
				if(LDWS_flag == 0)
				{
					l_change_bmp = bmp_cnt;
					LDWS_flag=1;
				}
				bmp_cnt++;
				if (bmp_cnt - l_change_bmp > LDWS_KEEP_TIME_CHANGE) //���5s��û�лָ�������ʻ�������¿������
				{
				
					giv_LDWSSREQFlg &=  ~LDWS_SUP_REQ_FLG_CHANGE_R;	
					LDWS_flag=0;	
					l_change_bmp=0;
					bmp_cnt=0;
				}
				return FW_INTO_NO_WARNING_AREA;
			}
			else
			{
				bmp_cnt=0;
			}
		
			giv_LDWSSREQFlg |= LDWS_SUP_REQ_FLG_CHANGE_L;
			warningVectorL[warningNumL++] = 1;
			if(warningNumL >= 5)
				warningNumL = 0;

			giv_LDWSSREQFlg &= ~LDWS_SUP_REQ_FLG_CHANGE_R;

			for (i = 0; i < 5; i++)
			{
				sumL += warningVectorL[i];
			}
			if(sumL > 0.6 * 5)
				return LFW_INTO_LV_WARNING_AREA;
			else
				return FW_INTO_NO_WARNING_AREA;				
		}
		else
		{
			warningVectorL[warningNumL++] = 0;
			if(warningNumL >= 5)
				warningNumL = 0;
			theretmp = 0;
		}
	
		
		r_offset = GetDWOffsetAndRate(pWAParam, rightAngle,rightOffset);
		if(r_offset <0)
		{
			giv_LDWSSREQFlg |= LDWS_SUP_REQ_FLG_CHANGE_R;
		}
	
		if((r_offset >= (camerrightdist + LDWS_WARNING_THERE_R))&& (l_count != 1))
		{
			 LDWS_flag=0;
			 giv_LDWSSREQFlg &= ~LDWS_SUP_REQ_FLG_CHANGE_L;
			 giv_LDWSSREQFlg &= ~LDWS_SUP_RIGHT_THETA;
			 if((giv_LDWSSREQFlg & LDWS_SUP_REQ_VOICE_WARNING) == LDWS_SUP_REQ_VOICE_WARNING)
			 {
				giv_LDWSSREQFlg &= ~LDWS_SUP_REQ_VOICE_WARNING;
			 }
		}
		//������ƫ���ٽ�ֵ
		if(r_offset > camerrightdist)
		{
	   		if((gsv_LDWSOutput.warningSingal & LDWS_WARNING_RIGHT_IMAGE)== LDWS_WARNING_RIGHT_IMAGE)
	   		{
				theretmp=(float)LDWS_WARNING_THERE_R;
			}
		}
	
		// ������Ϊ�Ҳ౨������ 				
		//if (r_offset <= (pWAParam->theresholdLine + theretmp) && r_offset >= pWAParam->latestLine)
		if (r_offset <= (camerrightdist + LDWS_WARNING_THERE_R) && r_offset >= pWAParam->latestLine)
		{ 
			r_count = 1;
			l_count =0;

			//�ж����ߵĽǶȣ��������2��������
			if(rightAngle > 1.6)
			{
				//giv_LDWSSREQFlg |= LDWS_SUP_RIGHT_THETA;
				//printf("leftAngle > 1.6 ????????!");
			}
			else if(rightAngle < -1.6)
			{
				giv_LDWSSREQFlg |= LDWS_SUP_LEFT_THETA;
				//printf("leftAngle < -1.6 but turn right!");
			}
			else
			{
				giv_LDWSSREQFlg &= ~LDWS_SUP_RIGHT_THETA;
			}
			//�ұ������
			if((giv_LDWSSREQFlg & LDWS_SUP_REQ_FLG_CHANGE_L) == LDWS_SUP_REQ_FLG_CHANGE_L)
			{
				if(LDWS_flag == 0)
				{
					r_change_bmp = bmp_cnt;
					LDWS_flag=1;
				}
				bmp_cnt++;
				if (bmp_cnt - r_change_bmp > LDWS_KEEP_TIME_CHANGE) //���5s��û�лָ�������ʻ�������¿������
				{
					giv_LDWSSREQFlg &=  ~LDWS_SUP_REQ_FLG_CHANGE_L;
					LDWS_flag=0;	
					r_change_bmp=0;
					bmp_cnt=0;
				}
				return FW_INTO_NO_WARNING_AREA;
			}
			else
			{
				bmp_cnt = 0;
			}
			giv_LDWSSREQFlg |= LDWS_SUP_REQ_FLG_CHANGE_R;
			warningVectorR[warningNumR++] = 1;
			if(warningNumR >= 5)
				warningNumR = 0;

			giv_LDWSSREQFlg &= ~LDWS_SUP_REQ_FLG_CHANGE_L;

			for (i = 0; i < 5; i++)
			{
				sumR += warningVectorR[i];
			}
			if(sumR > 0.6 * 5)
			   return RFW_INTO_RV_WARNING_AREA;
			else
               return FW_INTO_NO_WARNING_AREA;
		}
		else
		{
			warningVectorR[warningNumR++] = 0;
			if(warningNumR >= 5)
				warningNumR = 0;
			theretmp = 0;
		}
	}	

	r_count=0;
	l_count=0;
	bmp_cnt=0;
    theretmp = 0;
	return FW_INTO_NO_WARNING_AREA;
}

int AlarmMain(LDWS_Output *lineAttri)
{
	 
#define PI 3.14159626
	int lcv_alarmAreaFlg;
	Car_Width = LDWS_GetCarWidth();    //????1.6m
   	routeFlag = lineAttri->Route;
	rightOffset = lineAttri->Param[0] - lineAttri->Param[1];                       //�ҳ����ߵľ���
	leftOffset = lineAttri->Param[1];  //�󳵵��ߵľ���
	leftAngle =lineAttri->Param[2] * 180 / PI;// <0 ������ƫ
	rightAngle =lineAttri->Param[2] * 180 / PI;// >0������ƫ
	roadWval = lineAttri->Param[0];

	if(lineAttri->Route_half == 1)
	   lineLevel =lineAttri->Confidence_detection[1];//���Ŷ�>3ʱ����������
	else if(lineAttri->Route == 1)
	   lineLevel =lineAttri->Confidence_detection[0];//���Ŷ�>3ʱ����������
	else if(lineAttri->Route == 2)
	   lineLevel =lineAttri->Confidence_detection[2];//���Ŷ�>3ʱ����������
	else if(lineAttri->Route == 3)
	   lineLevel =lineAttri->Confidence_detection[3];//���Ŷ�>3ʱ����������
	else
	   lineLevel = 0;

	totalLevel = lineAttri->Confidence;
//	 printf("roadw = %f,l_offset =%f, r_offset = %f, psi =%f ,alpha =%f, Ch=%f, cl=%f, level=%d\n",
 //                    roadWval, leftOffset,rightOffset,
//                     leftAngle,
//                     lineAttri->Param[3] * 180 / PI, lineAttri->Param[4] * 180 / PI, lineAttri->Param[5] * 180 / PI,lineLevel);
	lcv_alarmAreaFlg = DetWarningArea(&gsv_LDWSWarningAreaParam);

	if((roadWidth  & LDWS_STATE_CAR_LOAD) == LDWS_STATE_CAR_LOAD)
	{
		lcv_alarmAreaFlg = 0;
	}
	if ((lcv_alarmAreaFlg == LFW_INTO_LV_WARNING_AREA)
	//	&&((giv_LDWSSREQFlg & LDWS_SUP_REQ_FLG_TLEFT) != LDWS_SUP_REQ_FLG_TLEFT)
		&&((giv_LDWSSREQFlg & LDWS_SUP_LEFT_THETA) != LDWS_SUP_LEFT_THETA))     
	{		
		gsv_LDWSOutput.warningSingal |= LDWS_WARNING_LEFT_IMAGE;
		return 1;
	}
	else if ((lcv_alarmAreaFlg == RFW_INTO_RV_WARNING_AREA)
	//	&&((giv_LDWSSREQFlg & LDWS_SUP_REQ_FLG_TRIGHT) != LDWS_SUP_REQ_FLG_TRIGHT)
		&&((giv_LDWSSREQFlg & LDWS_SUP_RIGHT_THETA) != LDWS_SUP_RIGHT_THETA))
	{
		
		gsv_LDWSOutput.warningSingal |= LDWS_WARNING_RIGHT_IMAGE;
		return 2;
	}
	else
	{
		gsv_LDWSOutput.warningSingal = 0;
		return 0;	
	}
	
}



#endif

