/**************************************************************************************************************
Copyright © Wissen Intelligent Sensing Technology Co., Ltd. 2011-2017. All rights reserved.
File name: FCWSD_Interface.h
Version: 1.0		Date: 2017-02-28		Author: Yanming Wang		ID: 1407930

Description:
	The functions in this file are defined to use the LBP feature for classification.
	The following function types are included:
	+ loadLbpDataCar(): Init the structures of vehicle LBP detector.
	+ lbpDetectCar(): Classify for a single scanning window.

Deviation:

History:
	+ Version: 1.0		Date: 2017-02-28		Author: Yanming Wang	ID: 1407930
	  Modification: Coding Standards are added.
**************************************************************************************************************/

#ifndef LBP_DETECT_H
#define LBP_DETECT_H

/*
I/O:	   Name		     Type	     Size			  	  Content
					     								  
[in/out]   l		     lbpCar*	 sizeof(lbpCar*)	  Structural for detector
[in]	   file		     char		 < 256	              The path of detector model.
					     								  
[out]	returned value   int         4-Byte	              If 0, detector read failed.

Realized function:
	+ Init the structures of vehicle detector.
*/
int loadLbpDataCar( lbpCar *l, const char *file);

/*
I/O:	Name		     Type	     Size			  	  Content
					     								  
[in]    l		         lbpCar*	 sizeof(lbpCar*)	  Structural for detector
[in]    img              unsigned int*        sizeof(unsigned int*)         integral image
[in]    x                unsigned int         4-Byte               point x position  for scanning window
[in]    y                unsigned int         4-Byte               point y position  for scanning window
					     								  
[out]	returned value   int         4-Byte	              detect result, if 0, not matched.

Realized function:
	+ Classify for a single scanning window.
*/
int lbpDetectCar( const lbpCar *l, const unsigned int *img, const int x, const int y, int arrNrow, const nrowCol *arrRowColDet);

#endif