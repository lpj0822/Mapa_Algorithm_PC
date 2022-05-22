/*******************************************************************************
*
* Copyright (c) 2011-2013  Wissen Company
* Wuxi Wissen Intelligent Sensing Technology Co., Ltd.��Ȩ���� 2011-2013
*                                                                           
* PROPRIETARY RIGHTS of Wissen Company are involved in the
* subject matter of this material.  All manufacturing, reproduction, use,      
* and sales rights pertaining to this subject matter are governed by the       
* license agreement.  The recipient of this software implicitly accepts        
* the terms of the license.                                                    
* ������ĵ�������άɭ��˾���ʲ�,�κ���ʿ�Ķ���ʹ�ñ����ϱ�����
* ��Ӧ��������Ȩ,�е��������κͽ�����Ӧ�ķ���Լ��.
*                                                                              
********************************************************************************   
* File Name          : sDas_ldws.h                                           
* Author             : Eason                                
* Revision           : 1.0                                           
* Date               : 14/4/2014 014:07:08                                     
* Description        : ����GB/T 26773-2011��ISO 17361-2007��׼����ƵĻ���άɭ����
*                      �ĳ���ƫ�뱨��ϵͳ
*                                                                           
* HISTORY***********************************************************************
* Date        | Modification                                            | Author 
* 14/4/2014   | άɭ����ƫ�뱨��ϵͳ1.0                                 | Eason
*
*******************************************************************************/
#ifndef _LDWS_H_
#define _LDWS_H_

#define _LDWS_ENABLED_
#ifdef _LDWS_ENABLED_


/* �������� */  
enum VehicleType                                                                   
{
	Passenger = 0,
	Commercial
};

/* ����LDWS�û����ò��� */
typedef struct
{

	double theresholdLine;                             /* ������ƫ�복�ٴ�������ʱ�����屨���ٽ���λ��
														  ����ƫ�복�ٱ���ʱ����ֵ�����ڱ�����Χ�ڣ��������籨���ߵľ��� */
	double vpLength;                                   /* ��������ƫ��ĳ����ض���λ��Գ����ĵ�����λ�� */
	double vpWidth;                                    /* ��������ƫ��ĳ����ض���λ��Գ����ĵĺ���λ�� */
	
	double vLength;                                    /* ���� */
	double vWidth;                                     /* ���� */
	double wheelbase;                                  /* ��� */
	float  factorIN;                                   /* ģ������������� */

	unsigned char vehicleType;                                    /* �������� */
	unsigned char deparSpeedWarningFlg;                           /* Ϊ1����ʾ��ƫ���ٶȱ�����0,���������ٽ��߱��� */
	
} LDWS_ConfigInfoStruct;

/* LDWS��� */
typedef struct
{
	signed long Reserve;

	unsigned char state;                                          /* ���LDWSϵͳ״̬ */
	unsigned char warningSingal;                                  /* ���LDWSϵͳ��� */
} LDWS_OutputStruct;

/* Extern define **************************************************************/

#define LDWS_STA_FLG_ACTIVE_OPEN               (0x01)  /* LDWS��/�� */
#define LDWS_STA_FLG_HARDWARE_FAULT            (0x02)  /* Ӳ������״̬ */
#define LDWS_STA_FLG_BUS_COM_FAULT             (0x04)  /* ����ͨ�Ź��� */

#define LDWS_WARNING_LEFT_IMAGE                (0x01)  /* ��ƫ��ͼ�񱨾� */
#define LDWS_WARNING_RIGHT_IMAGE               (0x02)  /* ��ƫ��ͼ�񱨾� */
#define LDWS_WARNING_VOICE                     (0x04)  /* ƫ���������� */
#define LDWS_WARNING_THERE                     0.21     /* ����ƫ����ʱֵ*/

extern int AlarmMain(LDWS_Output *lineAttri);

#endif
#endif