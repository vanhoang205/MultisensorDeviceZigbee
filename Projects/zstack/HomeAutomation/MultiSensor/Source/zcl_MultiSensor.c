/**************************************************************************************************
  Filename:       zcl_sampletemperaturesensor.c
  Revised:        $Date: 2014-10-24 16:04:46 -0700 (Fri, 24 Oct 2014) $
  Revision:       $Revision: 40796 $

  Description:    Zigbee Cluster Library - sample device application.


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
  This application implements a ZigBee Multi Sensor, based on Z-Stack 3.0.

  - MultiSensor Object includes 6 sensors:
      + Illuminance Sensor - Report to linux gateway after period time
      + Temperature Sensor - Report to linux gateway after change value with step 1 celcius
      + Humidity Sensor - Report to linux gateway after change value with step 1%RH
      + Occupancy Sensor - Report to linux gateway after change value
      + TVOC Sensor - Report to linux gateway after period time
      + Co2 Sensor - Report to linux gateway after period time
  - Multi sensor device act as end device with always power on 
  - Multi sensor device will receive date of sensors from stm32 with protocol follwed:
     |XXXX,YYYY,ZZZZ,DDDD,KKKK,LLLL\r\n| : format packet

*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ms.h"

#include "zcl_MultiSensor.h"

#include "onboard.h"

/* HAL */
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

#include "bdb_interface.h"   
/*********************************************************************
 * MACROS
 */

#define zcl_AccessCtrlRead( a )       ( (a) & ACCESS_CONTROL_READ )

#define UART_NONE       0
#define UART_PERIOD     1
#define UART_TEMP       2
#define UART_HUM        3
#define UART_PIR        4
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclMultiSensor_TaskID;
afAddrType_t zclMultiSensor_dstAddr;
extern int16 zdpExternalStateTaskID;
static devStates_t NwkStateShadow = DEV_HOLD;

uint16 gTimeCounter;
uint8 holdKeyCounter;
uint8 uartFlag;
uint8 signTempFlag;
/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static endPointDesc_t multiSensor_Ep =
{
  MULTISENSOR_ENDPOINT,           
  0,
  &zclMultiSensor_TaskID,
  (SimpleDescriptionFormat_t *)NULL,        // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0                  // No Network Latency req
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
  
// Functions to process other events relate to hardware
static void zclMultiSensor_HandleKeys( byte shift, byte keys );
static void zclMultiSensor_BasicResetCB( void );
static void zclMultiSensor_BatteryWarningCB( uint8 voltLevel);

// Function to process Commisioning
static void zclMultiSensor_ProcessCommissioningStatus(bdbCommissioningModeMsg_t* bdbCommissioningModeMsg);

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclMultiSensor_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclMultiSensor_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclMultiSensor_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif

#ifdef ZCL_REPORT
// Functions to process ZCL REPORT message
static void zclMultiSensor_ProcessInReportCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclMultiSensor_ProcessInConfigReportCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclMultiSensor_ProcessInReadReportCfgCmd( zclIncomingMsg_t *pInMsg );
static void zclMultiSensor_CheckReportConfig(void);
static uint8 SendZclAttrReport(uint8 srcEp, uint16 clusterId, zclReportCmd_t *pReportCmd, uint8 datalen);
static void zclMultiSensor_CheckAndSendClusterAttrReport( uint8 endpoint, uint16 clusterId,
                                                          zclConfigReportRecsList *pConfigReportRecsList );
static void sendZclAttrChangeReport(uint16 clusterId, uint16 attrID, uint8 *currentValue);
//-- MOD END
#endif

static uint8 zclMultiSensor_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );

// Functions to process UART interface
static void zclMultiSensor_UART_Init(void);
void uartEventApplicationCB(uint8 port, uint8 event);




/*********************************************************************
 * STATUS STRINGS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclMultiSensor_CmdCallbacks =
{
// ************************************ Callback List of Multi Sensor Endpoint *********************************************/  
    zclMultiSensor_BasicResetCB,                    // Basic Cluster Reset command
    NULL,                                           // Identify Trigger Effect command
    NULL,             				    // On/Off cluster command
    NULL,                                           // On/Off cluster enhanced command Off with Effect
    NULL,                                           // On/Off cluster enhanced command On with Recall Global Scene
    NULL,                                           // On/Off cluster enhanced command On with Timed Off
  #ifdef ZCL_LEVEL_CTRL
    NULL,                                           // Level Control Move to Level command
    NULL,                                           // Level Control Move command
    NULL,                                           // Level Control Step command
    NULL,                                           // Level Control Stop command
  #endif
  #ifdef ZCL_GROUPS
    NULL,                                           // Group Response commands
  #endif
  #ifdef ZCL_SCENES
    NULL,                                           // Scene Store Request command
    NULL,                                           // Scene Recall Request command
    NULL,                                           // Scene Response command
  #endif
  #ifdef ZCL_ALARMS
    NULL,                                           // Alarm (Response) commands
  #endif
  #ifdef SE_UK_EXT
    NULL,                                           // Get Event Log command
    NULL,                                           // Publish Event Log command
  #endif
    NULL,                                           // RSSI Location command
    NULL                                            // RSSI Location Response command
};


/*********************************************************************
 * @fn          zclMultiSensor_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
*
* @return      none
*/
void zclMultiSensor_Init( byte task_id )
{
  zclMultiSensor_TaskID = task_id;
  
  // Register the Simple Descriptor for this application
  bdb_RegisterSimpleDescriptor( &zclMultiSensor_SimpleDesc );
  
  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( MULTISENSOR_ENDPOINT, &zclMultiSensor_CmdCallbacks );


  // Register the application's attribute list
  zclMultiSensor_ResetAttributesToDefaultValues();
  zcl_registerAttrList( MULTISENSOR_ENDPOINT, zclMultiSensor_NumAttributes, zclMultiSensor_Attrs );    
  
  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclMultiSensor_TaskID );
    
  // Register low voltage NV memory protection application callback
  RegisterVoltageWarningCB( zclMultiSensor_BatteryWarningCB );
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclMultiSensor_TaskID );
  
  zclMultiSensor_UART_Init();
   
#ifdef ZCL_REPORT
  // Register the application's config report record list
  zcl_registerConfigReportRecList( MULTISENSOR_ENDPOINT,
                                   zclMultiSensor_NumConfigReportRecs, zclMultiSensor_ConfigReportRecs );
#endif
  
  // Register commissioning status callback
  bdb_RegisterCommissioningStatusCB( zclMultiSensor_ProcessCommissioningStatus );
  
  //Register for Endpoints
  afRegister( &multiSensor_Ep );

  bdb_StartCommissioning( BDB_COMMISSIONING_MODE_NWK_STEERING);
  zdpExternalStateTaskID = zclMultiSensor_TaskID;
  
}


/*********************************************************************
 * @fn          zclSample_event_loop
 *
 * @brief       Event Loop Processor for zclGeneral.
 *
 * @param       none
 *
 * @return      none
 */
uint16 zclMultiSensor_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclMultiSensor_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclMultiSensor_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;
          
        case ZDO_STATE_CHANGE:
           NwkStateShadow = (devStates_t)(MSGpkt->hdr.status);
          break;
          
        case KEY_CHANGE:
          zclMultiSensor_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }
    
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
#if ZG_BUILD_ENDDEVICE_TYPE    
  if ( events & MULTISENSOR_END_DEVICE_REJOIN_EVT )
  {
    bdb_ZedAttemptRecoverNwk();
    return ( events ^ MULTISENSOR_END_DEVICE_REJOIN_EVT );
  }
#endif
  
  if ( events & MULTISENSOR_CHECK_REPORT__EVT )
  {
    zclMultiSensor_CheckReportConfig();
    return ( events ^ MULTISENSOR_CHECK_REPORT__EVT );
  }
  
  if ( events & MULTISENSOR_CHECK_HOLD_KEY_EVT)
  {
    if ( HalKeyRead() & HAL_KEY_SW_1 )
    {
      holdKeyCounter++;
      osal_start_timerEx( zclMultiSensor_TaskID, MULTISENSOR_CHECK_HOLD_KEY_EVT, 1000 );
    }
    else {
      if ( holdKeyCounter >= 5 )
      {
        if(NwkStateShadow == DEV_END_DEVICE)
        {
          NLME_LeaveReq_t leaveReq;        
          leaveReq.extAddr = NULL;
          leaveReq.removeChildren = FALSE;
          leaveReq.rejoin = FALSE;
          leaveReq.silent = FALSE;
          if ( NLME_LeaveReq( &leaveReq ) == ZSuccess )
          {
            HalLedBlink(HAL_LED_1, 12, 50, 100);
          }
        }
        else 
        {
        zgWriteStartupOptions( ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_NETWORK_STATE | ZCD_STARTOPT_DEFAULT_CONFIG_STATE );
        SystemResetSoft();
        }
      }
      holdKeyCounter = 0;
    }
   return ( events ^ MULTISENSOR_CHECK_HOLD_KEY_EVT );   
  }
  // Discard unknown events
  return 0;
}


/*********************************************************************
 * @fn      zclMultiSensor_LcdDisplayMainMode
 *
 * @brief   Called to display the main screen on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
static void zclMultiSensor_ProcessCommissioningStatus(bdbCommissioningModeMsg_t* bdbCommissioningModeMsg)
{
    switch(bdbCommissioningModeMsg->bdbCommissioningMode)
    {
      case BDB_COMMISSIONING_FORMATION:         // Not used
        if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
        {
          //After formation, perform nwk steering again plus the remaining commissioning modes that has not been process yet
        }
        else
        {
          //Want to try other channels?
          //try with bdb_setChannelAttribute
        }
      break;
    case BDB_COMMISSIONING_NWK_STEERING:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //YOUR JOB:
        //We are on the nwk, what now?
        //Flash 6 times to notify that this devigce is on network
        HalLedBlink(HAL_LED_1, 6, 50, 200);
      }
        else
        {
          //See the possible errors for nwk steering procedure
          //No suitable networks found
          //Want to try other channels?
          //try with bdb_setChannelAttribute
        }
      break;
    case BDB_COMMISSIONING_FINDING_BINDING:
        if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
        {
          //YOUR JOB:
        }
        else
        {
          //YOUR JOB:
          //retry?, wait for user interaction?
        }
      break;
    case BDB_COMMISSIONING_INITIALIZATION:
        //Initialization notification can only be successful. Failure on initialization 
        //only happens for ZED and is notified as BDB_COMMISSIONING_PARENT_LOST notification
        
        //YOUR JOB:
        //We are on a network, what now?
        
      break;
#if ZG_BUILD_ENDDEVICE_TYPE    
    case BDB_COMMISSIONING_PARENT_LOST:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_NETWORK_RESTORED)
      {
        //We did recover from losing parent
        //Flash 6 times to notify this device is on network
        HalLedBlink(HAL_LED_1, 6, 50, 200);
      }
      else
      {
        //Parent not found, attempt to rejoin again after a fixed delay
        osal_start_timerEx(zclMultiSensor_TaskID, MULTISENSOR_END_DEVICE_REJOIN_EVT, MULTISENSOR_END_DEVICE_REJOIN_DELAY);
      }
    break;
#endif 
    }
}

/*********************************************************************
 * @fn      zclMultiSensor_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclMultiSensor_BasicResetCB( void )
{
  zclMultiSensor_ResetAttributesToDefaultValues();
}

/*********************************************************************
 * @fn      zclMultiSensor_BatteryWarningCB
 *
 * @brief   Called to handle battery-low situation.
 *
 * @param   voltLevel - level of severity
 *
 * @return  none
 */
void zclMultiSensor_BatteryWarningCB( uint8 voltLevel )
{
  if ( voltLevel == VOLT_LEVEL_CAUTIOUS )
  {
    // Send warning message to the gateway and blink LED
  }
  else if ( voltLevel == VOLT_LEVEL_BAD )
  {
    // Shut down the system
  }
}

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclMultiSensor_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclMultiSensor_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg)
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclMultiSensor_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclMultiSensor_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
    case ZCL_CMD_CONFIG_REPORT:
      zclMultiSensor_ProcessInConfigReportCmd( pInMsg );
      break;
      case ZCL_CMD_READ_REPORT_CFG:
      zclMultiSensor_ProcessInReadReportCfgCmd( pInMsg );
      break;
    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclMultiSensor_ProcessInConfigReportRspCmd( pInMsg );
      break;
    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclMultiSensor_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      zclMultiSensor_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclMultiSensor_ProcessInDefaultRspCmd( pInMsg );
      break;
      
    default:
      break;
  }

  if ( pInMsg->attrCmd )
  {
    osal_mem_free( pInMsg->attrCmd );
  }
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclMultiSensor_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclMultiSensor_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < readRspCmd->numAttr; i++ )
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }

  return ( TRUE );
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclMultiSensor_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclMultiSensor_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < writeRspCmd->numAttr; i++ )
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return ( TRUE );
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclMultiSensor_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclMultiSensor_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSampleTemperatureSensor_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_5
*                 HAL_KEY_SW_4
*                 HAL_KEY_SW_3
*                 HAL_KEY_SW_2
*                 HAL_KEY_SW_1
*
* @return  none
*/
static void zclMultiSensor_HandleKeys( byte shift, byte keys )
{
  if ( keys & HAL_KEY_SW_1 )
  {
    osal_set_event( zclMultiSensor_TaskID, MULTISENSOR_CHECK_HOLD_KEY_EVT);
  }
}

static void zclMultiSensor_UART_Init(void)
{
  halUARTCfg_t uartConfig;
  
  // configure UART
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = HAL_UART_BR_38400;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 48;
  uartConfig.rx.maxBufSize        = 75;
  uartConfig.tx.maxBufSize        = 7;
  uartConfig.idleTimeout          = 6;
  uartConfig.intEnable            = TRUE;
  uartConfig.callBackFunc         = uartEventApplicationCB;
  
  HalUARTOpen( HAL_UART_PORT_0, &uartConfig);
}

void uartEventApplicationCB(uint8 port, uint8 event)
{
    uint16 valueOfSensors[NUMBER_OF_SENSOR] = {0};
    int16 valueOfTemp = 0;
    uint16 valueOfHum = 0;
    uint8 valueOfPir = 0;
    int fieldIndex = 0;         
    
    if ( event & HAL_UART_RX_TIMEOUT) 
    {
      while ( Hal_UART_RxBufLen(port) > 0 )
      {
        uint8 ch;
        HalUARTRead(port, &ch, 1);
        
        if ( ch >= '0' && ch <= '9' )     // is this an ascii digit between 0 and 9 ?
        {
          if ( uartFlag == UART_PERIOD )
          {
            valueOfSensors[fieldIndex] = (valueOfSensors[fieldIndex] * 10) + (ch - '0');
          }
          else if ( uartFlag == UART_TEMP)
          {
            valueOfTemp = ( valueOfTemp * 10 ) + (ch - '0');
          }
          else if ( uartFlag == UART_HUM )
          {
            valueOfHum = ( valueOfHum * 10 ) + (ch - '0');
          }
          else if ( uartFlag == UART_PIR )
          {
            valueOfPir = ( valueOfPir * 10 ) + (ch - '0');
          }
        }
        else if (ch == ',')     // comma is our separator, so move on to the next field
        {
          if ( uartFlag == UART_PERIOD )
          {
            if (fieldIndex < NUMBER_OF_SENSOR - 1)
              fieldIndex++;       // increment field index
          }

        }
        else if ( ch == 'O' )
        {
          
          uartFlag = UART_PIR;
        }
        else if ( ch == 'P' )
        {
          uartFlag = UART_PERIOD;
        }
        else if ( ch == 'T' )
        {
          uartFlag = UART_TEMP;
        }
        else if ( ch == 'H' )
        {
          uartFlag = UART_HUM;
        }
        else if ( ch == '-' )
        {
          signTempFlag = TRUE;
        }
        else
        {
//          // any character not a digit or comma ends the acquisition of fields
//          // In this project, it's the newline character sent by the STM32 MCU
          if(uartFlag == UART_PERIOD )
          {
            zclMultiSensor_Light_MeasuredValue = valueOfSensors[INDEX_LIGHT];
            zclMultiSensor_TVOC_MeasuredValue = valueOfSensors[INDEX_TVOC];
            zclMultiSensor_CO2_MeasuredValue = valueOfSensors[INDEX_CO2];
          }
          else if ( uartFlag == UART_HUM )
          {
            zclMultiSensor_Humidity_MeasuredValue = valueOfHum;
            sendZclAttrChangeReport( ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY, ATTRID_MS_RELATIVE_HUMIDITY_MEASURED_VALUE, (uint8 *)&zclMultiSensor_Humidity_MeasuredValue);
          }
          else if ( uartFlag == UART_TEMP )
          {
            if (signTempFlag)
            {
              zclMultiSensor_Temperature_MeasuredValue = (-1) * valueOfTemp;  
            }
            else
            {
              zclMultiSensor_Temperature_MeasuredValue = valueOfTemp;  
            }
            sendZclAttrChangeReport( ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT, ATTRID_MS_TEMPERATURE_MEASURED_VALUE, (uint8 *)&zclMultiSensor_Temperature_MeasuredValue);
          }

          else if ( uartFlag == UART_PIR )
          {
            zclMultiSensor_Pir_Status = (uint8)valueOfPir;
            sendZclAttrChangeReport( ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING, ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY, &zclMultiSensor_Pir_Status);
          }
        }
      }
      uartFlag = UART_NONE;
      signTempFlag = FALSE;
    }
}

#ifdef ZCL_REPORT
/*********************************************************************
* @fn      zclMultiSensor_CheckAndSendClusterAttrReport
*
* @brief   Check if there is a reportable attribute in all cluster is timeout to report
*
* @param   none
*
* @return  none
*/
static void zclMultiSensor_CheckAndSendClusterAttrReport( uint8 endpoint, uint16 clusterId,
                                                                     zclConfigReportRecsList *pConfigReportRecsList )
{
  uint8 numAttr = 0;
  uint8 x;
  uint16 len;
  zclReportCmd_t *pReportCmd;
  zclConfigReportRec_t *pConfigReportRec = NULL;
  zclAttrRec_t attrRec;
  
  for (x = 0; x < pConfigReportRecsList->numConfigReportRec; x++)
  {
    pConfigReportRec = &(pConfigReportRecsList->configReportRecs[x]);
    
    if (pConfigReportRec->clusterId == clusterId &&
        pConfigReportRec->cfgReportRec.maxReportInt != 0xFFFF)
    {
      if (pConfigReportRec->timeup == 0xFFFF || pConfigReportRec->timeup == gTimeCounter)
      {
        numAttr++;
      }
    }
  }
  
  if (numAttr != 0)
  {
    // we need to send a report - allocate space for it
    len = sizeof(zclReportCmd_t) + (numAttr * sizeof(zclReport_t));
    pReportCmd = (zclReportCmd_t *)zcl_mem_alloc( len );
    pReportCmd->numAttr = numAttr;
  }
  
  numAttr = 0;
  
  for (x = 0; x < pConfigReportRecsList->numConfigReportRec; x++ )
  {
    zclReport_t *reportRec;
    pConfigReportRec = &(pConfigReportRecsList->configReportRecs[x]);
    
    if (pConfigReportRec->clusterId == clusterId && pConfigReportRec->cfgReportRec.maxReportInt != 0xFFFF)      // need report
    {
      if (pConfigReportRec->timeup == 0xFFFF || pConfigReportRec->timeup == gTimeCounter)       //timeup to report
      {
        // fill the record in *pReportCmd
        reportRec = &(pReportCmd->attrList[numAttr]);
        zcl_memset( reportRec, 0, sizeof(zclReport_t));
        numAttr++;
        zclFindAttrRec(endpoint, pConfigReportRec->clusterId, pConfigReportRec->cfgReportRec.attrID, &attrRec);
        
        reportRec->attrID = attrRec.attr.attrId;
        reportRec->dataType = attrRec.attr.dataType;
        reportRec->attrData = attrRec.attr.dataPtr;
        
        if (pConfigReportRec->cfgReportRec.minReportInt == 0)
          pConfigReportRec->timeup = gTimeCounter + pConfigReportRec->cfgReportRec.maxReportInt;
        else
          pConfigReportRec->timeup = gTimeCounter + pConfigReportRec->cfgReportRec.minReportInt;
      }
    }
  }
  
  if (numAttr != 0)
  {
    SendZclAttrReport( endpoint, clusterId, pReportCmd, len);
  }
}

/*********************************************************************
* @fn      SendZclAttrReport
*
* @brief   Send the attr report. Let ZCL_CMD_REPORT event handler handle this
*
* @param   srcEp - source endpoint
*          clusterID - cluster id
*          pReportCmd - pointer to the report command packet
*          dataLen - data length of the report command
*
* @return  none
*/
static uint8 SendZclAttrReport(uint8 srcEp, uint16 clusterId, zclReportCmd_t *pReportCmd,
                               uint8 datalen)
{
  // this is for the inner-app osal msg, not OTA msg, thus some fields are not important
  zclIncomingMsg_t *pMsg;       
  
  // pMsg will be released by zclMultiSensor_event_loop()
  pMsg = (zclIncomingMsg_t *)osal_msg_allocate(sizeof(zclIncomingMsg_t) + (datalen));
  
  if (pMsg == NULL)
  {
    return FALSE;
  }
  
  if (pMsg)
  {
      pMsg->hdr.event = ZCL_INCOMING_MSG;
      pMsg->hdr.status = 0;
      //pMsg->zclHdr.fc = NULL;         // not important
      pMsg->zclHdr.manuCode = 0;        // not important
      pMsg->zclHdr.transSeqNum = 0;     // not important
      pMsg->zclHdr.commandID = ZCL_CMD_REPORT;
      pMsg->clusterId = clusterId;
      pMsg->srcAddr.addrMode = (afAddrMode_t)Addr16Bit;
      pMsg->srcAddr.addr.shortAddr = 0; // not important
      pMsg->srcAddr.panId = 0;          // inner-PAN, not important
      pMsg->srcAddr.endPoint = srcEp;   // src ep, SAMPLELIGHT_ENDPOINT send to himself
      pMsg->endPoint = srcEp;           // dest ep, send to SAMPLELIGHT_ENDPOINT himself
      pMsg->attrCmd = (zclReportCmd_t *)pReportCmd;
  }
  
  osal_msg_send( zclMultiSensor_TaskID, (uint8 *)pMsg);
  return TRUE;
}

static void zclMultiSensor_ProcessInReportCmd( zclIncomingMsg_t *pInMsg )
{
  zclReportCmd_t *pReportCmd;                           // numAttr, attrList[] : (zclReport_t) attrID, dataType, *attrData
  pReportCmd = (zclReportCmd_t *)pInMsg->attrCmd;       // *pReportCmd will be free by handle
  afAddrType_t dstAddr;
  
  dstAddr.addr.shortAddr = 0x0000;
  dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  switch ( pInMsg->clusterId )
  {
  case ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT:
    dstAddr.endPoint = 1;
    break;
  case ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT:
    dstAddr.endPoint = 2;
    break;
  case ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY:
    dstAddr.endPoint = 3;
    break;
  case ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING:
    dstAddr.endPoint = 4;
    break;
  case ZCL_CLUSTER_ID_MS_TVOC_MEASUREMENT:
    dstAddr.endPoint = 6;
    break;
  case ZCL_CLUSTER_ID_MS_CO2_MEASUREMENT:
    dstAddr.endPoint = 7;
    break;
  default:
    break;
  }
  
  if( zcl_SendReportCmd( MULTISENSOR_ENDPOINT, &dstAddr, pInMsg->clusterId, pReportCmd, ZCL_REPORT_RECEIVE, FALSE, NULL) != ZSuccess )
  {
    while(1);
  }  
}

static uint8 zclMultiSensor_ProcessInConfigReportCmd( zclIncomingMsg_t *pInMsg )
{
  zclCfgReportCmd_t *pCfgReportCmd;             // this is used to look up table
  zclCfgReportRspCmd_t *pCfgReportRspCmd;       // This is used to send responde message
  uint8 sendRsp = FALSE;                        // status of initialize dynamic memory for responde message
  uint16 len;
  uint8 j = 0;
  uint8 i;
  
  //1. Initialize message responde ( using structure zclCfgReportRspCmd_t )
  pCfgReportCmd = (zclCfgReportCmd_t *)pInMsg->attrCmd;
  
  if ( pInMsg->zclHdr.commandID == ZCL_CMD_CONFIG_REPORT )
  {
    // We need to send a response back - allocate space for it
    len = sizeof( zclCfgReportRspCmd_t ) + (pCfgReportCmd->numAttr * sizeof( zclCfgReportStatus_t ));
    pCfgReportRspCmd = (zclCfgReportRspCmd_t *)zcl_mem_alloc( len );
    
    if ( pCfgReportRspCmd == NULL )
    {
      return FALSE;     // embedded return
    }
    sendRsp = TRUE;     // sendRsp is active when we got correct commandID
  }
  
  for ( i = 0; i < pCfgReportCmd->numAttr; i++ )
  {
    
    //2. Look up config. report record of the incomming message in table "config. Report Record List"
    zclConfigReportRec_t *pConfigReportRec = NULL;      // find the rec and store here
    zclAttrRec_t attrRec;
    
    zclCfgReportStatus_t *statusRec = &(pCfgReportRspCmd->attrList[i]);
    zcl_memset( statusRec, 0, sizeof( zclCfgReportStatus_t ));
    
    if ( zclFindConfigReportRec( pInMsg->endPoint, pInMsg->clusterId,
                                pCfgReportCmd->attrList[i].attrID, &pConfigReportRec))
    {
      
      //3. Check "dataType" of variable to variable in incomming message
      uint8 status = ZCL_STATUS_SUCCESS;
      
      if (pCfgReportCmd->attrList[i].dataType != pConfigReportRec->cfgReportRec.dataType )
      {        
        status = ZCL_STATUS_INVALID_DATA_TYPE;
      }
      else 
      {
        
        //4. Look up Attribute record of incomming message in tabel "attribute list"
        if ( zclFindAttrRec( pInMsg->endPoint, pInMsg->clusterId,
                            pCfgReportCmd->attrList[i].attrID, &attrRec))
        {
          //5. Check "dataType" of variable to variable in incomming message
          if (pCfgReportCmd->attrList[i].dataType != attrRec.attr.dataType )
          {
            status = ZCL_STATUS_INVALID_DATA_TYPE;
          }
          else
          {
            //6. Check Access control type of each attribute in incomming message to attribute in table "attribute list"
            if ( !zcl_AccessCtrlRead(attrRec.attr.accessControl))
            {
              status = ZCL_STATUS_WRITE_ONLY;
            }
          }
        }
      }
      
      //7. Write config. value into variable in table "config. report record list"
      // If successful, store the record, and a CfgReportStatus record shall NOT be generated
      if ( sendRsp && status != ZCL_STATUS_SUCCESS )
      {
        //Attribute is write only or invalid data type - move on to the next record
        statusRec->status = status;
        statusRec->direction = pCfgReportCmd->attrList[i].direction;
        statusRec->attrID = pCfgReportCmd->attrList[i].attrID;
        j++;
      }
      else    // Success, set the config report rec
      {
        pConfigReportRec->cfgReportRec.direction = pCfgReportCmd->attrList[i].direction;
        pConfigReportRec->cfgReportRec.minReportInt = pCfgReportCmd->attrList[i].minReportInt;
        pConfigReportRec->cfgReportRec.maxReportInt = pCfgReportCmd->attrList[i].maxReportInt;
        pConfigReportRec->cfgReportRec.timeoutPeriod = pCfgReportCmd->attrList[i].timeoutPeriod;
        pConfigReportRec->timeup = 0xFFFF;
      }
    }
    else
    {
      //Attribute is not supported - move on to the next configReportRec record
      if (sendRsp)
      {
        statusRec->status = ZCL_STATUS_UNSUPPORTED_ATTRIBUTE;
        statusRec->status = pCfgReportCmd->attrList[i].direction;
        statusRec->attrID = pCfgReportCmd->attrList[i].attrID;
        j++;
      }
    }
  }//for loop
  //8. Send respond message with function "zcl_SendConfigReportRspCmd"
  if ( sendRsp )
  {
    pCfgReportRspCmd->numAttr = j;
    if (pCfgReportRspCmd->numAttr == 0)
    {
      //Since all records were written successful, include a single status record
      // in the response command with the status field set to SUCCESS and the
      // attribute ID and direction fields omitted.
      pCfgReportRspCmd->attrList[0].status = ZCL_STATUS_SUCCESS;
      pCfgReportRspCmd->numAttr = 1;
    }
    
    zcl_SendConfigReportRspCmd(pInMsg->endPoint, &(pInMsg->srcAddr),
                               pInMsg->clusterId, pCfgReportRspCmd,
                               !pInMsg->zclHdr.fc.direction, TRUE,
                               pInMsg->zclHdr.transSeqNum );
    zcl_mem_free ( pCfgReportRspCmd );
  }
  
  //9. When configured, check report config immediately with function "xxx_CheckReportConfig()"
  // when configured, check report config immediately
  zclMultiSensor_CheckReportConfig();
  // The MULTISENSOR_CHECK_REPORT_EVT will then be triggered again and again
  // if we never received the ConfiReportCmd, the MULTISENSOR_CHECK_REPORT_EVT
  // has no change to be triggered.
  
  // We think this makes sense, since there is no reason for your app to perform
  // constantly report unless the app is configured to report.
  // if your app just need to automatically report after bootup, you can trigger
  // MULTISENSOR_CHECK_REPORT_EVT in zclXXX_Init().
  return TRUE;
}

static uint8 zclMultiSensor_ProcessInReadReportCfgCmd( zclIncomingMsg_t *pInMsg )
{
//1. Get zclReadReportCfgCmd_t from incomming message
  zclReadReportCfgCmd_t *pReadReportCfgCmd; 
  zclReadReportCfgRspCmd_t *pReadReportCfgRspCmd;
  
  uint8 sendRsp = FALSE;
  uint16 len;
  uint8 i;
  
  pReadReportCfgCmd = (zclReadReportCfgCmd_t *)pInMsg->attrCmd;
  
//2. Check command in message == ZCL_CMD_READ_REPORT_CFG and allocate memory for respond message
  if (pInMsg->zclHdr.commandID == ZCL_CMD_READ_REPORT_CFG )
  {
    // We need to send a response back - allocate for it
    len = sizeof(zclReadReportCfgCmd_t) + (pReadReportCfgCmd->numAttr * sizeof(zclReportCfgRspRec_t));
    pReadReportCfgRspCmd = (zclReadReportCfgRspCmd_t *)zcl_mem_alloc(len);
    
    if ( pReadReportCfgRspCmd == NULL )
    {
      return FALSE;
    }
    sendRsp = TRUE;     // sendRsp is active when we got correct commandID
  }
  
//3. Find config. report record in table "config. report record list"
  for (i = 0; i < pReadReportCfgCmd->numAttr; i++)
  {
    zclConfigReportRec_t *pConfigReportRec = NULL;       // find the rec and store here
    zclReportCfgRspRec_t *pReportCfgRspRec = &(pReadReportCfgRspCmd->attrList[i]);
    zclAttrRec_t attrRec;
    
    zcl_memset( pReportCfgRspRec, 0, sizeof( zclReportCfgRspRec_t ));
    
    if ( zclFindConfigReportRec (pInMsg->endPoint, pInMsg->clusterId,
                                 pReadReportCfgCmd->attrList[i].attrID, &pConfigReportRec))
    {
//4. If found configReportRec in table, write value to ReportCfgRspRec
      if ( sendRsp )
      {
         pReportCfgRspRec->status = ZCL_STATUS_SUCCESS;
            pReportCfgRspRec->direction = pConfigReportRec->cfgReportRec.direction;
            pReportCfgRspRec->attrID = pConfigReportRec->cfgReportRec.attrID;
            pReportCfgRspRec->dataType = pConfigReportRec->cfgReportRec.dataType;
            pReportCfgRspRec->minReportInt = pConfigReportRec->cfgReportRec.minReportInt;
            pReportCfgRspRec->maxReportInt = pConfigReportRec->cfgReportRec.maxReportInt;
            pReportCfgRspRec->timeoutPeriod = pConfigReportRec->cfgReportRec.timeoutPeriod;
            pReportCfgRspRec->reportableChange = pConfigReportRec->cfgReportRec.reportableChange;
      }
    }
    else 
    {
//5. If not found configReportRec, check if the attribute is an un-reportable or an un support one
      uint8 status = ZCL_STATUS_UNSUPPORTED_ATTRIBUTE;
      
      if ( zclFindAttrRec( pInMsg->endPoint, pInMsg->clusterId, pReadReportCfgCmd->attrList[i].attrID, &attrRec))
      {
        // if found the attr rec, it is there bu un-reportale
        status = ZCL_STATUS_UNREPORTABLE_ATTRIBUTE;
      }
      // Attribute is not supported - move on to the next configReportRec record
      if ( sendRsp )
      {
        pReportCfgRspRec->status = status;
        pReportCfgRspRec->direction = pReadReportCfgCmd->attrList[i].direction;
        pReportCfgRspRec->attrID = pReadReportCfgCmd->attrList[i].attrID;
      }
    }
  } // for loop
 
//6. Send cfg. respond message 
  if (sendRsp)
  {
    pReadReportCfgCmd->numAttr = pReadReportCfgCmd->numAttr;
    
    zcl_SendReadReportCfgRspCmd( pInMsg->endPoint, &(pInMsg->srcAddr), pInMsg->clusterId,
                                 pReadReportCfgRspCmd, !pInMsg->zclHdr.fc.direction, true, pInMsg->zclHdr.transSeqNum);
    zcl_mem_free( pReadReportCfgRspCmd );
  }
  return TRUE;
}


/*********************************************************************
* @fn      zclMultiSensor_CheckReportConfig
*
* @brief   Check if there is a reportable attribute in all cluster is timeout to report
*
* @param   none
*
* @return  none
*/
static void zclMultiSensor_CheckReportConfig(void)
{
  uint8 x, y;
  uint8 stopChecking = TRUE;
  
  // Fill the "config. report rec list" for this endpoint
  zclConfigReportRecsList *pConfigReportRecsList = zclFindConfigReportRecsList( MULTISENSOR_ENDPOINT );
  
  if ( pConfigReportRecsList != NULL )
  {
    /* This piece of code will classify each luster in the list of endpoint, then send report
     * command coressponding cluster 
     */
    for ( x = 0; x < pConfigReportRecsList->numConfigReportRec; x++)
    {
      uint8 cIDuplicate = 0;
      
      for (y = 0; y < x; y++)
      {
        if ( pConfigReportRecsList->configReportRecs[x].clusterId == 
            pConfigReportRecsList->configReportRecs[y].clusterId)
        {
          cIDuplicate = 1;
        }
      }
      
      if (!cIDuplicate)
      {
        zclMultiSensor_CheckAndSendClusterAttrReport( MULTISENSOR_ENDPOINT, pConfigReportRecsList->configReportRecs[x].clusterId, pConfigReportRecsList);
      }
      
      if (pConfigReportRecsList->configReportRecs[x].cfgReportRec.maxReportInt != 0xFFFF)
      {
        stopChecking = FALSE;   // If there is any attribute setting to report, don't stop checking
      }
    }
  }
  
  gTimeCounter++;         // time ticks every second for checking attr report
  if (!stopChecking)
  {
      osal_start_timerEx( zclMultiSensor_TaskID, MULTISENSOR_CHECK_REPORT__EVT, 1000); 
  }
}

static void sendZclAttrChangeReport(uint16 clusterId, uint16 attrID, uint8 *currentValue) 
{

        zclReportCmd_t *pReportCmd;
        zclReport_t *reportRec;
        zclAttrRec_t attrRec;
        uint8 len;
    
        len = sizeof( zclReportCmd_t ) + (1 * sizeof( zclReport_t ));
        pReportCmd = (zclReportCmd_t *)zcl_mem_alloc( len );
        pReportCmd->numAttr = 1;
    
        reportRec = &(pReportCmd->attrList[0]);
        zcl_memset( reportRec, 0, sizeof( zclReport_t ) );
        zclFindAttrRec( MULTISENSOR_ENDPOINT, clusterId, attrID, &attrRec);
    
        reportRec->attrID = attrRec.attr.attrId;
        reportRec->dataType = attrRec.attr.dataType;
        reportRec->attrData = attrRec.attr.dataPtr; 
                
        SendZclAttrReport(MULTISENSOR_ENDPOINT, clusterId, pReportCmd, len);
    
}
#endif  //ZCL_REPORT


/****************************************************************************
****************************************************************************/


