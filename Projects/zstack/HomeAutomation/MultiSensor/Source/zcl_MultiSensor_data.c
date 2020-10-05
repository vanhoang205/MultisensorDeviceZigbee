/**************************************************************************************************
Filename:       zcl_sampletemperaturesensor_data.c
Revised:        $Date: 2014-09-25 13:20:41 -0700 (Thu, 25 Sep 2014) $
Revision:       $Revision: 40295 $


Description:    Zigbee Cluster Library - sample device application.


Copyright 2013-2014 Texas Instruments Incorporated. All rights reserved.

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
PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
* INCLUDES
*/
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDConfig.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ms.h"

#include "zcl_MultiSensor.h"

/*********************************************************************
* CONSTANTS
*/

#define MULTISENSOR_DEVICE_VERSION     0
#define MULTISENSOR_FLAGS              0

#define MULTISENSOR_HWVERSION          1
#define MULTISENSOR_ZCLVERSION         6

#define MULTISENSOR_TEMPERATURE_MAX_MEASURED_VALUE  12500  // 27.00C
#define MULTISENSOR_TEMPERATURE_MIN_MEASURED_VALUE  -4000  // 17.00C

#define MULTISENSOR_HUMIDITY_MIN_MEASURED_VALUE     0
#define MULTISENSOR_HUMIDITY_MAX_MEASURED_VALUE     10000


/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* MACROS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/

// Global attributes
const uint16 zclMultiSensor_clusterRevision_all = 0x0001; 

// Basic Cluster
const uint8 zclMultiSensor_HWRevision = MULTISENSOR_HWVERSION;
const uint8 zclMultiSensor_ZCLVersion = MULTISENSOR_ZCLVERSION;
const uint8 zclMultiSensor_ManufacturerName[] = { 16, 'I','T','S','C','-','R','D','-','T','e','c','h',' ',' ',' ',' ' };
const uint8 zclMultiSensor_ModelId[] = { 16, 'I','T','S','C','0','0','0','1',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclMultiSensor_DateCode[] = { 16, '2','0','0','6','0','8','3','1',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclMultiSensor_PowerSource = POWER_SOURCE_MAINS_1_PHASE;
uint8 zclMultiSensor_LocationDescription[17];
uint8 zclMultiSensor_PhysicalEnvironment = 0x0B; //office
uint8 zclMultiSensor_DeviceEnable;

// Identify Cluster
uint16 zclMultiSensor_IdentifyTime;

// Temperature Sensor Cluster
int16 zclMultiSensor_Temperature_MeasuredValue;

int16 zclMultiSensor_Temperature_MinMeasuredValue = MULTISENSOR_TEMPERATURE_MIN_MEASURED_VALUE; 
uint16 zclMultiSensor_Temperature_MaxMeasuredValue = MULTISENSOR_TEMPERATURE_MAX_MEASURED_VALUE;

// Light Sensor Cluster
uint16 zclMultiSensor_Light_MeasuredValue;

// Humidity Sensor Cluster
uint16 zclMultiSensor_Humidity_MeasuredValue;
uint16 zclMultiSensor_Humidity_MinMeasuredValue = MULTISENSOR_HUMIDITY_MIN_MEASURED_VALUE;
uint16 zclMultiSensor_Humidity_MaxMeasuredValue = MULTISENSOR_HUMIDITY_MAX_MEASURED_VALUE;

// PIR Sensor Cluster
uint8 zclMultiSensor_Pir_Status;
uint8 zclMultiSensor_Pir_Type = MS_OCCUPANCY_SENSOR_TYPE_PIR;

// TVOC Sensor Cluster
uint16 zclMultiSensor_TVOC_MeasuredValue;

// CO2 Sensor Cluster
uint16 zclMultiSensor_CO2_MeasuredValue;

/*********************************************************************
* ATTRIBUTE DEFINITIONS - Uses REAL cluster IDs
*/

// NOTE: The attributes listed in the AttrRec must be in ascending order 
// per cluster to allow right function of the Foundation discovery commands

CONST zclAttrRec_t zclMultiSensor_Attrs[] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclMultiSensor_ZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,             // Cluster IDs - defined in the foundation (ie. zcl.h)
    {  // Attribute record
      ATTRID_BASIC_HW_VERSION,            // Attribute ID - Found in Cluster Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,                 // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,                // Variable access control - found in zcl.h
      (void *)&zclMultiSensor_HWRevision  // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclMultiSensor_ManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclMultiSensor_ModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclMultiSensor_DateCode
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_ENUM8,
      ACCESS_CONTROL_READ,
      (void *)&zclMultiSensor_PowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_LOCATION_DESC,
      ZCL_DATATYPE_CHAR_STR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)zclMultiSensor_LocationDescription
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_PHYSICAL_ENV,
      ZCL_DATATYPE_ENUM8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclMultiSensor_PhysicalEnvironment
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DEVICE_ENABLED,
      ZCL_DATATYPE_BOOLEAN,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclMultiSensor_DeviceEnable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclMultiSensor_clusterRevision_all
    }
  },
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclMultiSensor_IdentifyTime
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_GLOBAL,
      (void *)&zclMultiSensor_clusterRevision_all
    }
  },
  // *** Light Measurement Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_ILLUMINANCE_LEVEL_STATUS,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclMultiSensor_Light_MeasuredValue
    }
  },
  
  {
    ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclMultiSensor_clusterRevision_all
    }
  },
   // *** Temperature Measurement Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_TEMPERATURE_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclMultiSensor_Temperature_MeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_TEMPERATURE_MIN_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclMultiSensor_Temperature_MinMeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_TEMPERATURE_MAX_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclMultiSensor_Temperature_MaxMeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclMultiSensor_clusterRevision_all
    }
  },
  
  
  // *** Humidity Measurement Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
    { // Attribute record
      ATTRID_MS_RELATIVE_HUMIDITY_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclMultiSensor_Humidity_MeasuredValue
    }
  },
    {
    ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
    { // Attribute record
      ATTRID_MS_RELATIVE_HUMIDITY_MIN_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclMultiSensor_Humidity_MinMeasuredValue
    }
  },
    {
    ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
    { // Attribute record
      ATTRID_MS_RELATIVE_HUMIDITY_MAX_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclMultiSensor_Humidity_MaxMeasuredValue
    }
  }, 
  {
    ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclMultiSensor_clusterRevision_all
    }
  },
    // *** PIR Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING,
    { // Attribute record
      ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclMultiSensor_Pir_Status
    }
  },
  
  {
    ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING,
    { // Attribute record
      ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY_SENSOR_TYPE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclMultiSensor_Pir_Type
    }
  },
  
  {
    ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclMultiSensor_clusterRevision_all
    }
  },
  
      // *** TVOC Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_TVOC_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_TVOC_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclMultiSensor_TVOC_MeasuredValue
    }
  },
  
  {
    ZCL_CLUSTER_ID_MS_TVOC_MEASUREMENT,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclMultiSensor_clusterRevision_all
    }
  }, 
  
      // *** CO2 Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_CO2_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_CO2_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclMultiSensor_CO2_MeasuredValue
    }
  },
  
  {
    ZCL_CLUSTER_ID_MS_CO2_MEASUREMENT,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclMultiSensor_clusterRevision_all
    }
  }
};


uint8 CONST zclMultiSensor_NumAttributes = ( sizeof(zclMultiSensor_Attrs) / sizeof(zclMultiSensor_Attrs[0]) );

/*********************************************************************
* SIMPLE DESCRIPTOR
*/
// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.

/************************ Cluster List for Light Sensor Endpoint ********************************/
const cId_t zclMultiSensor_InClusterList[ZCL_MULTISENSOR_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,
  ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
  ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
  ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING,
  ZCL_CLUSTER_ID_MS_TVOC_MEASUREMENT,
  ZCL_CLUSTER_ID_MS_CO2_MEASUREMENT
};

const cId_t zclMultiSensor_OutClusterList[ZCL_MULTISENSOR_MAX_OUTCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_IDENTIFY
};


SimpleDescriptionFormat_t zclMultiSensor_SimpleDesc =
{
  
  /************************ Simple Description for mux Sensor Endpoint **********************************/
    MULTISENSOR_ENDPOINT,                        //  int Endpoint;
    ZCL_HA_PROFILE_ID,                                          //  uint16 AppProfId[2];
    ZCL_HA_DEVICEID_MULTI_SENSOR,                           //  uint16 AppDeviceId[2];
    MULTISENSOR_DEVICE_VERSION,                      //  int   AppDevVer:4;
    MULTISENSOR_FLAGS,                               //  int   AppFlags:4;
    ZCL_MULTISENSOR_MAX_INCLUSTERS,               //  byte  AppNumInClusters;
    (cId_t *)zclMultiSensor_InClusterList,       //  byte *pAppInClusterList;
    ZCL_MULTISENSOR_MAX_OUTCLUSTERS,              //  byte  AppNumInClusters;
    (cId_t *)zclMultiSensor_OutClusterList       //  byte *pAppInClusterList;
};

zclConfigReportRec_t zclMultiSensor_ConfigReportRecs[]= 
{
  {
    //******** Light mesurement Attribute *********
    ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,                    // Cluster ID
    0xFFFF,                                                       // timeup
    (void *)&zclMultiSensor_Light_MeasuredValue,   // &lastReportValue
    { //cdfReportRec
      ZCL_REPORT_SEND,                            // Direction
      ATTRID_MS_ILLUMINANCE_LEVEL_STATUS,         // Attribute ID
      ZCL_DATATYPE_UINT16,                         // Data Type
      0,                                          // min Report interval
      0xFFFF,                                     // max Report interval
      0,                                          // timeout period
      NULL                                        // reportable change
    }
  },
  
  {
    //******** TVOC measurement Attribute ***********
    ZCL_CLUSTER_ID_MS_TVOC_MEASUREMENT,
    0xFFFF,
    (void *)&zclMultiSensor_TVOC_MeasuredValue,
    { 
      ZCL_REPORT_SEND,
      ATTRID_MS_TVOC_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      0,
      0xFFFF,
      0,
      NULL
    }
  },
  
  {
    //******** CO2 measurement Attribute ***********
    ZCL_CLUSTER_ID_MS_CO2_MEASUREMENT,
    0xFFFF,
    (void *)&zclMultiSensor_CO2_MeasuredValue,
    { 
      ZCL_REPORT_SEND,
      ATTRID_MS_CO2_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      0,
      0xFFFF,
      0,
      NULL
    }
  }
};

uint8 CONST zclMultiSensor_NumConfigReportRecs = ( sizeof(zclMultiSensor_ConfigReportRecs) / sizeof(zclMultiSensor_ConfigReportRecs[0]) );

/*********************************************************************
* GLOBAL FUNCTIONS
*/

/*********************************************************************
* LOCAL FUNCTIONS
*/

/*********************************************************************
* @fn      zclSampleLight_ResetAttributesToDefaultValues
*
* @brief   Reset all writable attributes to their default values.
*
* @param   none
*
* @return  none
*/
void zclMultiSensor_ResetAttributesToDefaultValues(void)
{
  int i;
  
  zclMultiSensor_LocationDescription[0] = 16;
  for (i = 1; i <= 16; i++)
  {
    zclMultiSensor_LocationDescription[i] = ' ';
  }
  
  zclMultiSensor_PhysicalEnvironment = PHY_UNSPECIFIED_ENV;
  zclMultiSensor_DeviceEnable = DEVICE_ENABLED;
  
#ifdef ZCL_IDENTIFY
  zclMultiSensor_IdentifyTime = 0;
#endif
  
}

/****************************************************************************
****************************************************************************/


