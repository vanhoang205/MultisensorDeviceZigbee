/**************************************************************************************************
  Filename:       zcl_sampletemperaturesensor.h
  Revised:        $Date: 2013-04-22 14:49:05 -0700 (Mon, 22 Apr 2013) $
  Revision:       $Revision: 33994 $

  Description:    This file contains the Zigbee Cluster Library Home
                  Automation Sample Application.


  Copyright 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

#ifndef ZCL_MULTISENSOR_H
#define ZCL_MULTISENSOR_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"

/*********************************************************************
 * CONSTANTS
 */
  
// endpoint on end-device
#define MULTISENSOR_ENDPOINT            8


// endpoint on coordinator
#define COOR_ENDPOINT_LUX               1
#define COOR_ENDPOINT_TEMPERATURE       2        
#define COOR_ENDPOINT_HUMIDITY          3
#define COOR_ENDPOINT_PIR               4
#define COOR_ENDPOINT_TVOC              5
#define COOR_ENDPOINT_CO2               6
  
  
// Index for devices to identify device in list 
#define INDEX_LIGHT             0
#define INDEX_TVOC              1
#define INDEX_CO2               2
#define NUMBER_OF_SENSOR        3

#define LIGHT_OFF                       0x00
#define LIGHT_ON                        0x01

// Application Events
#define MULTISENSOR_END_DEVICE_REJOIN_EVT             0x0002    
#define MULTISENSOR_END_DEVICE_REJOIN_DELAY           1000
   
#define MULTISENSOR_CHECK_REPORT__EVT                 0x0004
#define MULTISENSOR_CHECK_HOLD_KEY_EVT                0x0008      


// Macro about information cluster

#define ZCL_MULTISENSOR_MAX_INCLUSTERS          8
#define ZCL_MULTISENSOR_MAX_OUTCLUSTERS         1



/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */
extern SimpleDescriptionFormat_t zclMultiSensor_SimpleDesc;

extern CONST zclAttrRec_t zclMultiSensor_Attrs[];

extern CONST uint8 zclMultiSensor_NumAttributes;

extern uint8  zclMultiSensor_OnOff;

extern uint16 zclMultiSensor_IdentifyTime;

// Temperature Sensor Cluster
extern int16 zclMultiSensor_Temperature_MeasuredValue;
extern int16 zclMultiSensor_Temperature_MinMeasuredValue; 
extern uint16 zclMultiSensor_Temperature_MaxMeasuredValue;

// Light Sensor Cluster
extern uint16 zclMultiSensor_Light_MeasuredValue;

// Humidity Sensor Cluster
extern uint16 zclMultiSensor_Humidity_MeasuredValue;
extern uint16 zclMultiSensor_Humidity_MinMeasuredValue;
extern uint16 zclMultiSensor_Humidity_MaxMeasuredValue;

// PIR Sensor Cluster
extern uint8 zclMultiSensor_Pir_Status;
extern uint8 zclMultiSensor_Pir_Type;

// TVOC Sensor Cluster
extern uint16 zclMultiSensor_TVOC_MeasuredValue;

// CO2 Sensor Cluster
extern uint16 zclMultiSensor_CO2_MeasuredValue;

// Declaring Cluster List
extern const cId_t zclMultiSensor_InClusterList[];
extern const cId_t zclMultiSensor_OutClusterList[];


extern zclConfigReportRec_t zclMultiSensor_ConfigReportRecs[];
extern uint8 CONST zclMultiSensor_NumConfigReportRecs;


/*********************************************************************
 * FUNCTIONS
 */

 /*
  * Initialization for the task
  */
extern void zclMultiSensor_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 zclMultiSensor_event_loop( byte task_id, UINT16 events );
   
/*
 *  Reset all writable attributes to their default values.
 */
extern void zclMultiSensor_ResetAttributesToDefaultValues(void);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZCL_MULTISENSOR_H */
