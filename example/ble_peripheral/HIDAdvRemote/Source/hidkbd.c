/**************************************************************************************************
 
  Phyplus Microelectronics Limited confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Phyplus Microelectronics 
  Limited ("Phyplus"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of Phyplus. You may not use this 
  Software unless you agree to abide by the terms of these agreements. 
  You acknowledge that the Software may not be modified, copied, 
  distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy 
  (BLE) integrated circuit, either as a product or is integrated into your 
  products.  Other than for the aforementioned purposes, you may not use, 
  reproduce, copy, prepare derivative works of, modify, distribute, perform, 
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
  
**************************************************************************************************/


/******************************************************************************


 *****************************************************************************/


/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "linkdb.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "peripheral.h"
#include "hidkbdservice.h"
#include "devinfoservice.h"
#include "hiddev.h"
#include "AudioGATTprofile.h"

#include "global_config.h"

#include "voice_data.h"
#include "Voice_Queue.h"
#include "adpcm.h"
#include "voice_task.h"

#include "hidkbd.h"


#include "ll_def.h"
#include "log.h"
#include "voice_task.h"

#include "OSAL_Memory.h"
#include "hal_mcu.h"
#include "hci.h"
/*********************************************************************
 * MACROS
 */

// Selected HID keycodes
#define KEY_RIGHT_ARROW             0x4F
#define KEY_LEFT_ARROW              0x50
#define KEY_NONE                    0x00

// Selected HID LED bitmaps
#define LED_NUM_LOCK                0x01
#define LED_CAPS_LOCK               0x02

// Selected HID mouse button values
#define MOUSE_BUTTON_1              0x01
#define MOUSE_BUTTON_NONE           0x00

// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN     8

// HID LED output report length
#define HID_LED_OUT_RPT_LEN         1

// HID mouse input report length
#define HID_MOUSE_IN_RPT_LEN        5

#if EN_VOICE_MODE
#define HID_VOICE_IN_START_LEN        5

#endif

#if EN_CONSUMER_MODE
// HID consumer control input report length
#define HID_CC_IN_RPT_LEN                     1//2//1

#endif

/*********************************************************************
 * CONSTANTS
 */

// HID idle timeout in msec; set to zero to disable timeout
#define DEFAULT_HID_IDLE_TIMEOUT              0

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     10//10

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     10//10

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         50

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          500

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE//TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         15//6

// Default passcode
#define DEFAULT_PASSCODE                      0

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE//GAPBOND_PAIRING_MODE_INITIATE

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE//FALSE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL           6

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern gaprole_States_t hidDevGapState;

// Task ID
uint8 hidKbdTaskId;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

extern verInfo_t      verInfo;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

#if 0
// GAP Profile - Name attribute for SCAN RSP data
static uint8 scanData[] =
{
    0x0D,                             // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,   // AD Type = Complete local name
    'H',
    'I',
    'D',
    ' ',
    'K',
    'e',
    'y',
    'b',
    'o',
    'a',
    'r',
    'd'
};

// Advertising data
static uint8 advData[] =
{
    // flags
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // appearance
    0x03,   // length of this data
    GAP_ADTYPE_APPEARANCE,
    LO_UINT16(GAP_APPEARE_HID_KEYBOARD),
    HI_UINT16(GAP_APPEARE_HID_KEYBOARD),

    // service UUIDs
    0x05,   // length of this data
    GAP_ADTYPE_16BIT_MORE,
    LO_UINT16(HID_SERV_UUID),
    HI_UINT16(HID_SERV_UUID),
    LO_UINT16(BATT_SERV_UUID),
    HI_UINT16(BATT_SERV_UUID)
};

// Device name attribute value
static CONST uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "HID Keyboard";
#else
// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanData[] =
{
    0x0A,                             // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,   // AD Type = Complete local name
    'C',
    'C',
    '2',
    '6',
    '5',
    '0',
    ' ',
    'R',
    'C'
};

// Advertising data
static uint8_t advData[] =
{
    // flags
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // appearance
    0x03,   // length of this data
    GAP_ADTYPE_APPEARANCE,
    LO_UINT16(GAP_APPEARE_GENERIC_HID),
    HI_UINT16(GAP_APPEARE_GENERIC_HID),

    // service UUIDs
    0x05,   // length of this data
    GAP_ADTYPE_16BIT_MORE,
    LO_UINT16(HID_SERV_UUID),
    HI_UINT16(HID_SERV_UUID),
    LO_UINT16(BATT_SERV_UUID),
    HI_UINT16(BATT_SERV_UUID)
};

// Device name attribute value
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "CC2650 RC";

#endif

// HID Dev configuration
static hidDevCfg_t hidKbdCfg =
{
    DEFAULT_HID_IDLE_TIMEOUT,   // Idle timeout
    HID_KBD_FLAGS               // HID feature flags
};

// TRUE if boot mouse enabled
uint8 hidBootMouseEnabled = FALSE;


uint8 inqueue_buf[100];
uint8 outqueue_buf[100];

uint8 voice_tmp_buf[384];
short tmp_buf[192];

uint8 start_trans_flag=0;

uint8 trans_buf[20]={0,0,0,0,1,2,3,4,5,6,7,8,9,11,12,13,14,15,16};

uint32 trans_index=0;

uint8  trans_package_cnt;
uint8  trans_limit_cnt;





/*********************************************************************
 * LOCAL FUNCTIONS
 */



static void hidKbd_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void hidKbd_ProcessGattMsg( gattMsgEvent_t *pMsg );
void hidKbdSendReport( uint8 keycode );
void hidKbdSendMouseReport( uint8 buttons,uint8 x,uint8 y );
static uint8 hidKbdRcvReport( uint8 len, uint8 *pData );
static uint8 hidKbdRptCB( uint8 id, uint8 type, uint16 uuid,
                          uint8 oper, uint8 *pLen, uint8 *pData );
static void hidKbdEvtCB( uint8 evt );
#if 1//EN_VOICE_MODE
 uint8 hidKbdSendVoiceCMDtReport( uint8 keycode );


#endif

#if EN_CONSUMER_MODE
static void hidCCSendReport( uint8 cmd, bool keyPressed, uint8 keyRepeated );
void hidCCSendReportKey( uint8 cmd, bool keyPressed);

#endif

static void AudioProfileChangeCB( uint8 paramID );

static uint32  osal_memory_statics(void *ptr);



/*********************************************************************
 * PROFILE CALLBACKS
 */

static hidDevCB_t hidKbdHidCBs =
{
    hidKbdRptCB,
    hidKbdEvtCB,
    NULL//hidAdvRemotePasscodeCB
};


// Simple GATT Profile Callbacks
static AudioProfileCBs_t AudioBLEPeripheral_AudioProfileCBs =
{
    AudioProfileChangeCB    // Charactersitic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */





/*********************************************************************
 * @fn      HidEmuKbd_Init
 *
 * @brief   Initialization function for the HidEmuKbd App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void HidKbd_Init( uint8 task_id )
{
    hidKbdTaskId = task_id;
    LOG("%s\n",__FUNCTION__);
    // Setup the GAP
    VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );

    // set BLE version information
    verInfo.verNum= 0x0006;		   // BLE4.0
    verInfo.comId= 0x000D;		  // TI
    verInfo.subverNum=0x0101;		  // sub version number

    pGlobal_config[LL_TX_PKTS_PER_CONN_EVT] = 5;
    pGlobal_config[LL_RX_PKTS_PER_CONN_EVT] = 5;


		// calibration time for 2 connection event, will advance the next conn event receive window
		// SLAVE_CONN_DELAY for sync catch, SLAVE_CONN_DELAY_BEFORE_SYNC for sync not catch
		// pGlobal_config[SLAVE_CONN_DELAY] = 500;//0;//1500;//0;//3000;//0;          ---> update 11-20
		// pGlobal_config[SLAVE_CONN_DELAY_BEFORE_SYNC] = 1100;  //800-1100



    // Setup the GAP Peripheral Role Profile
    {
        uint8 initial_advertising_enable = TRUE;

        // By setting this to zero, the device will go into the waiting state after
        // being discoverable for 30.72 second, and will not being advertising again
        // until the enabler is set back to TRUE
        uint16 gapRole_AdvertOffTime = 0;

        uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
        uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
        uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

        // Set the GAP Role Parameters
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
        GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

        GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advData ), advData );
        GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanData ), scanData );

        GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
        GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
        GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
        GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
        GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
    }

    // Set the GAP Characteristics
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (void *) attDeviceName );

    // Setup the GAP Bond Manager
    {
        uint32 passkey = DEFAULT_PASSCODE;
        uint8 pairMode = DEFAULT_PAIRING_MODE;
        uint8 mitm = DEFAULT_MITM_MODE;
        uint8 ioCap = DEFAULT_IO_CAPABILITIES;
        uint8 bonding = DEFAULT_BONDING_MODE;
        GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
        GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
        GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
        GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
        GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
    }


    {
        // Use the same interval for general and limited advertising.
        // Note that only general advertising will occur based on the above configuration
        uint16_t advInt = 160;

        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
    }

    AudioProfile_AddService(GATT_ALL_SERVICES);

    // Set up HID keyboard service
    HidKbd_AddService( );

    // Register for HID Dev callback
    HidDev_Register( &hidKbdCfg, &hidKbdHidCBs );
    AudioProfile_RegisterAppCBs(&AudioBLEPeripheral_AudioProfileCBs);

    LOG("int:HEAP SIZE=%d\n\r",osal_memory_statics((int*)0x1fff1948));


// HCI_PPLUS_ExtendTRXCmd(1);
    // Setup a delayed profile startup
    osal_set_event( hidKbdTaskId, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      HidEmuKbd_ProcessEvent
 *
 * @brief   HidEmuKbd Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 HidKbd_ProcessEvent( uint8 task_id, uint16 events )
{

    VOID task_id; // OSAL required parameter that isn't used in this function
// LOG("%s\n",__FUNCTION__);

    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;

        if ( (pMsg = osal_msg_receive( hidKbdTaskId )) != NULL )
        {
            hidKbd_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & START_DEVICE_EVT )
    {

        //  osal_start_timerEx(hidKbdTaskId, HID_KEY_TEST_EVT, 5000);

        return ( events ^ START_DEVICE_EVT );
    }
    if(events &HID_KEY_TEST_EVT)
    {
        LOG("start adpcm event\n\r");
		//	static uint8 voice_start_index=0;

		//	hidKbdSendReport(HID_KEYBOARD_2);
		//	hidKbdSendReport(0);

		//voice_start_index++;
		//if(voice_start_index==2)
		//hidKbdSendVoiceStartReport(0);

		//hidKbdSendMouseReport(0,10,5);

		//	hidCCSendReport(0,1,0);
		//	hidCCSendReport(0,0,0);

		// hidKbdSendVoiceStartReport(0);





        InitQueue();
        voice_adpcmState.index=0;
        voice_adpcmState.valprev=0;
        VoiceQueue.VoiceSendFlg=1;

        voiceRaw_index=0;

        header_index=0;


		trans_package_cnt=0;
		trans_limit_cnt=0;

#if VOICE_MTU_SIZE_FIXED_20_BYTES
        VoiceSend_SubIndex=0;
#endif

		

		//if(hidKbdSendVoiceCMDtReport(0x04)==SUCCESS)
		{
          osal_set_event(hidKbdTaskId, ADPCM_CODEC_START_EVT);
		//  LOG("audio 04\n\r");
		}



        return ( events ^ HID_KEY_TEST_EVT );
    }

    if(events &ADPCM_CODEC_START_EVT)
    {


#if VOICE_FROM_FLASH
// const voice
        uint8 inqueue_first;
        if(VoiceQueue.VoiceSendFlg==1)
        {
  

                osal_memcpy(tmp_buf,&Voice_data[VOICE_RAW_FRAM_LEN*voiceRaw_index], VOICE_RAW_FRAM_LEN);

                adpcm_coder(tmp_buf, (char*)inqueue_buf, VOICE_RAW_FRAM_LEN, &voice_adpcmState);

                voiceRaw_index++;



                header_index++;
                inqueue_first=((header_index%32 << 3) | 0x01);
                inqueue_buf[0]=inqueue_first;

                InQueue(inqueue_buf);



         

            if(VoiceQueue.VoicePackageSN<VOICE_RAW_LEN/VOICE_RAW_FRAM_LEN)
            {
                osal_start_timerEx(hidKbdTaskId, ADPCM_CODEC_START_EVT, 10);
            }
            else
            {
                osal_start_timerEx(hidKbdTaskId, VOICE_TRANSF_EVT, 150);
                //VoiceSend_SubIndex=5;
                //hidKbdSendVoiceCMDtReport(0x04);
                LOG("开始语音传输\n\r");
                LOG("HEAP SIZE=%d\n\r",osal_memory_statics((int*)0x1fff1948));
				trans_index=0;

                osal_stop_timerEx(hidKbdTaskId, ADPCM_CODEC_START_EVT);
            }

        }

#else
        //adc voice
        uint8 inqueue_first;
        uint8 inqueue_i;



        if(VoiceQueue.VoiceSendFlg==1&&(hidDevGapState==GAPROLE_CONNECTED))
        {

            osal_start_timerEx(hidKbdTaskId, ADPCM_CODEC_START_EVT, 24);

            for(inqueue_i=0; inqueue_i<3; inqueue_i++)
            {
                if(voice_requeset_data(voice_tmp_buf, VOICE_RAW_FRAM_LEN))
                {


                    osal_memcpy(tmp_buf, voice_tmp_buf, VOICE_RAW_FRAM_LEN);
                    adpcm_coder(tmp_buf, (char*)inqueue_buf, VOICE_RAW_FRAM_LEN, &voice_adpcmState);



                    header_index++;
                    inqueue_first=((header_index%32 << 3) | 0x01);
                    inqueue_buf[0]=inqueue_first;

                    InQueue(inqueue_buf);



                    if(VoiceQueue.VoicePackageSN==12)  //12
                    {
                        osal_stop_timerEx(hidKbdTaskId, ADPCM_CODEC_START_EVT);
						if(hidKbdSendVoiceCMDtReport(0x04)==SUCCESS)
						{
						 osal_start_timerEx(hidKbdTaskId, VOICE_TRANSF_EVT, 50);
						 LOG("audio 04\n\r");
						}

						 

                        LOG("start voice send\n\r");
                        LOG("HEAP SIZE=%d\n\r",osal_memory_statics((int*)0x1fff1948));
                    }

                }
                else
                    break;

            }



        }

#endif

        return ( events ^ ADPCM_CODEC_START_EVT );
    }

    if(events &VOICE_TRANSF_EVT)
    {



#if VOICE_MTU_SIZE_FIXED_20_BYTES

#if VOICE_FROM_FLASH
//const voice

	   if(VoiceQueue.VoiceSendFlg==1&&start_trans_flag==1)
	   {

		  // osal_start_timerEx(hidKbdTaskId, VOICE_TRANSF_EVT, 20); // modify the period


		   for(VoiceSend_SubIndex=0; VoiceSend_SubIndex<8;VoiceSend_SubIndex++)
		   {

		       trans_buf[0]=(uint8)(trans_index>>24);
			   trans_buf[1]=(uint8)(trans_index>>16);
			   trans_buf[2]=(uint8)(trans_index>>8);
			   trans_buf[3]=(uint8)trans_index;
			   
			   if(AudioProfile_SetParameter(AUDIOPROFILE_CHAR2, 20, trans_buf)==SUCCESS&&VoiceQueue.VoiceSendFlg==1)
			   {
			      trans_index++;
			   }
			   else
			   {

				  // LOG("HEAP SIZE=%d\n\r",osal_memory_statics((int*)0x1fff1948));

				   break;
			   }

		   }


	   }


	   if(start_trans_flag==0)
	   {


		   if(hidKbdSendVoiceCMDtReport(0x04)==SUCCESS)
		   {
			   start_trans_flag=1;
			   LOG("start_tick:%d\n\r",osal_GetSystemClock());
			   VoiceSend_SubIndex=5;
			  // osal_start_timerEx(hidKbdTaskId, VOICE_TRANSF_EVT, 100);

			   HCI_PPLUS_ConnEventDoneNoticeCmd( hidKbdTaskId, VOICE_TRANSF_EVT);
		   }
		   else
		   {
			   osal_start_timerEx(hidKbdTaskId, VOICE_TRANSF_EVT, 20);
		   }


	   }



#else



#if 1
	   //adc voice

       uint8 inqueue_heard;
	  
       if(trans_package_cnt>6)
       {
        trans_package_cnt=0;
		trans_limit_cnt=0;
	   }

	   trans_package_cnt++;

	   if(VoiceQueue.VoiceSendFlg==1&&(hidDevGapState==GAPROLE_CONNECTED))
	   {

		 for(uint8 m=0;m<2;m++)
		 {
			if(voice_requeset_data(voice_tmp_buf, VOICE_RAW_FRAM_LEN))
			{

			osal_memcpy(tmp_buf, voice_tmp_buf, VOICE_RAW_FRAM_LEN);
			adpcm_coder(tmp_buf, (char*)inqueue_buf, VOICE_RAW_FRAM_LEN, &voice_adpcmState);


			header_index++;
			inqueue_heard=((header_index%32 << 3) | 0x01);
			inqueue_buf[0]=inqueue_heard;

			InQueue(inqueue_buf);

			}
			else
			break;

		 }


	   }


#endif

        //adc voice
        if(VoiceQueue.VoiceSendFlg==1&&start_trans_flag==1&&(hidDevGapState==GAPROLE_CONNECTED))
        {

          //  osal_start_timerEx(hidKbdTaskId, VOICE_TRANSF_EVT, 15);


            if(VoiceSend_SubIndex==5)
            {
                if(OutQueue(outqueue_buf)!=0)
                {

                    LOG("Voice Transf empty\n\r");

                    // osal_start_timerEx(hidKbdTaskId, VOICE_STOP_EVT, 200);
                }
                else
                {

                    VoiceSend_SubIndex=0;
                }

            }

            for(; VoiceSend_SubIndex<5;)
            {
               if(trans_limit_cnt>=29)
			   	break;
				
                if(AudioProfile_SetParameter(AUDIOPROFILE_CHAR2, VOICE_BLESEND_FRAME_SIZE, &outqueue_buf[VOICE_BLESEND_FRAME_SIZE*VoiceSend_SubIndex])==SUCCESS&&VoiceQueue.VoiceSendFlg==1)
                {

                    VoiceSend_SubIndex++;

					
                    trans_limit_cnt++;
				

                    if(VoiceSend_SubIndex==5)
                    {
                        #if 1
                        if(OutQueue(outqueue_buf)!=0)
                        {
                            LOG("Voice Transf empty\n\r");
                        }
                        else
                        {
                            VoiceSend_SubIndex=0;
                        }

						#else
                        break;
						#endif
						

                    }
					
                }
                else
                {

                    break;
                }

            }


        }


        if(start_trans_flag==0&&VoiceQueue.VoiceSendFlg==1)
        {

                start_trans_flag=1;
                LOG("start_tick:%d\n\r",osal_GetSystemClock());
                VoiceSend_SubIndex=5;
                //osal_start_timerEx(hidKbdTaskId, VOICE_TRANSF_EVT, 10);
               HCI_PPLUS_ConnEventDoneNoticeCmd( hidKbdTaskId, VOICE_TRANSF_EVT);
      
        }


#endif

#endif


        return ( events ^ VOICE_TRANSF_EVT );
    }

    if(events &VOICE_STOP_EVT)
    {
        if( hidKbdSendVoiceCMDtReport(0x00)==SUCCESS)
        {
            start_trans_flag=0;
            VoiceQueue.VoiceSendFlg=0;

            LOG("Voice Stop \n\r");
        }
        else
        {
          // hidKbdSendVoiceCMDtReport(0x00);
            osal_start_timerEx(hidKbdTaskId, VOICE_STOP_EVT, 30);
            start_trans_flag=0;
            VoiceQueue.VoiceSendFlg=0;
        }

        return ( events ^ VOICE_STOP_EVT );
    }

    return 0;
}

/*********************************************************************
 * @fn      hidKbd_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void hidKbd_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
    switch ( pMsg->event )
    {

    case GATT_MSG_EVENT:
        hidKbd_ProcessGattMsg( (gattMsgEvent_t *)pMsg );
        break;

    default:
        break;
    }
}


/*********************************************************************
 * @fn      hidKbd_ProcessGattMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void hidKbd_ProcessGattMsg( gattMsgEvent_t *pMsg )
{
// ble_ancs_handle_gatt_event(pMsg);
    //GATT_bm_free( &pMsg->msg, pMsg->method );
}

/*********************************************************************
 * @fn      hidKbdSendReport
 *
 * @brief   Build and send a HID keyboard report.
 *
 * @param   keycode - HID keycode.
 *
 * @return  none
 */
void hidKbdSendReport( uint8 keycode )
{
    uint8 buf[HID_KEYBOARD_IN_RPT_LEN];

    buf[0] = 0;         // Modifier keys
    buf[1] = 0;         // Reserved
    buf[2] = keycode;   // Keycode 1
    buf[3] = 0;         // Keycode 2
    buf[4] = 0;         // Keycode 3
    buf[5] = 0;         // Keycode 4
    buf[6] = 0;         // Keycode 5
    buf[7] = 0;         // Keycode 6

    HidDev_Report( HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT,
                   HID_KEYBOARD_IN_RPT_LEN, buf );
}

/*********************************************************************
 * @fn      hidKbdSendMouseReport
 *
 * @brief   Build and send a HID mouse report.
 *
 * @param   buttons - Mouse button code
 *
 * @return  none
 */
void hidKbdSendMouseReport( uint8 buttons,uint8 x,uint8 y )
{
    uint8 buf[HID_MOUSE_IN_RPT_LEN];

    buf[0] = buttons;   // Buttons
    buf[1] = x;         // X
    buf[2] = y;         // Y
    buf[3] = 0;         // Wheel
    buf[4] = 0;         // AC Pan

    HidDev_Report( HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT,
                   HID_MOUSE_IN_RPT_LEN, buf );
}


 uint8 hidKbdSendVoiceCMDtReport( uint8 keycode )
{
    uint8 status;
    uint8 buf[1];

    buf[0] = keycode;         // Modifier keys
    status=AudioProfile_SetParameter(AUDIOPROFILE_CHAR1, 1,buf);
// AudioProfile_SetParameter(AUDIOPROFILE_CHAR2, 1,buf);

    return status;
}

#if EN_CONSUMER_MODE
/*********************************************************************
 * @fn      hidCCSendReport
 *
 * @brief   Build and send a HID Consumer Control report.
 *
 * @param   cmd - bitmap of left/middle/right mouse button states
 * @param   keyPressed - amount of mouse movement along X-axis in units of Mickeys (1/200 of an inch)
 * @param   keyRepeated - amount of mouse movement along Y-axis
 *
 * @return  none
 */
static void hidCCSendReport( uint8 cmd, bool keyPressed, uint8 keyRepeated )
{
    // Only send the report if something meaningful to report
    if ( keyRepeated == 0 )
    {
        uint8 buf[HID_CC_IN_RPT_LEN] = {0};

        // No need to include Report Id
        if ( keyPressed )
        {
            buf[0]=0X40;
            buf[1]=0X00;
        }
        else
        {
            buf[0]=0X00;
            buf[1]=0X00;
        }



        HidDev_Report( HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT,
                       HID_CC_IN_RPT_LEN, buf );
    }
}
#endif
void hidCCSendReportKey( uint8 cmd, bool keyPressed)
{
  uint8 buf[1] = {0};

  if(keyPressed==1)
  {
   buf[0]=cmd;
  }
  else
  	buf[0]=0;

   HidDev_Report( HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT,
                       1, buf );

}


/*********************************************************************
 * @fn      hidKbdRcvReport
 *
 * @brief   Process an incoming HID keyboard report.
 *
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  status
 */
static uint8 hidKbdRcvReport( uint8 len, uint8 *pData )
{
    // verify data length
    if ( len == HID_LED_OUT_RPT_LEN )
    {
        // set keyfob LEDs
        //HalLedSet( HAL_LED_1, ((*pData & LED_CAPS_LOCK) == LED_CAPS_LOCK) );
        //HalLedSet( HAL_LED_2, ((*pData & LED_NUM_LOCK) == LED_NUM_LOCK) );

        return SUCCESS;
    }
    else
    {
        return ATT_ERR_INVALID_VALUE_SIZE;
    }
}

/*********************************************************************
 * @fn      hidKbdRptCB
 *
 * @brief   HID Dev report callback.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   uuid - attribute uuid.
 * @param   oper - operation:  read, write, etc.
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  GATT status code.
 */
static uint8 hidKbdRptCB( uint8 id, uint8 type, uint16 uuid,
                          uint8 oper, uint8 *pLen, uint8 *pData )
{
    uint8 status = SUCCESS;

    // write
    if ( oper == HID_DEV_OPER_WRITE )
    {
        if ( uuid == REPORT_UUID )
        {
            // process write to LED output report; ignore others
            if ( type == HID_REPORT_TYPE_OUTPUT )
            {
                status = hidKbdRcvReport( *pLen, pData );
            }
        }

        if ( status == SUCCESS )
        {
            status = HidKbd_SetParameter( id, type, uuid, *pLen, pData );
        }
    }
    // read
    else if ( oper == HID_DEV_OPER_READ )
    {
        status = HidKbd_GetParameter( id, type, uuid, pLen, pData );
    }
    // notifications enabled
    else if ( oper == HID_DEV_OPER_ENABLE )
    {
        if ( id == HID_RPT_ID_MOUSE_IN && type == HID_REPORT_TYPE_INPUT )
        {
            hidBootMouseEnabled = TRUE;
        }
    }
    // notifications disabled
    else if ( oper == HID_DEV_OPER_DISABLE )
    {
        if ( id == HID_RPT_ID_MOUSE_IN && type == HID_REPORT_TYPE_INPUT )
        {
            hidBootMouseEnabled = FALSE;
        }
    }

    return status;
}

/*********************************************************************
 * @fn      hidKbdEvtCB
 *
 * @brief   HID Dev event callback.
 *
 * @param   evt - event ID.
 *
 * @return  HID response code.
 */
static void hidKbdEvtCB( uint8 evt )
{
    // process enter/exit suspend or enter/exit boot mode

    return;
}


/*********************************************************************
* @fn      AudioProfileChangeCB
*
* @brief   Callback from SimpleBLEProfile indicating a value change
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  none
*/
static void AudioProfileChangeCB( uint8 paramID )
{

    switch( paramID )
    {


    default:
        // should not reach here!
        break;
    }
}

typedef uint8 halDataAlign_t;
/*
typedef struct
{
    // The 15 LSB's of 'val' indicate the total item size, including the header, in 8-bit bytes.
    unsigned short len : 15;   // unsigned short len : 15;
    // The 1 MSB of 'val' is used as a boolean to indicate in-use or freed.
    unsigned short inUse : 1;  // unsigned short inUse : 1;
} osalMemHdrHdr_t;

typedef union
{
    //Dummy variable so compiler forces structure to alignment of largest element while not wasting
    //space on targets when the halDataAlign_t is smaller than a UINT16.
    
    halDataAlign_t alignDummy;
    uint32 val;            // uint16    // TODO: maybe due to 4 byte alignment requirement in M0, this union should be 4 byte, change from uint16 to uint32, investigate more later -  04-25
    osalMemHdrHdr_t hdr;
} osalMemHdr_t;
*/




// ===========================================================
// ptr: the header of osal heap
static uint32  osal_memory_statics(void *ptr)
{
    osalMemHdr_t *header, *current;
    uint32  sum_alloc = 0;
    // halIntState_t intState;


    header = (osalMemHdr_t *)ptr;
    current = (osalMemHdr_t *)ptr;

    HAL_ENTER_CRITICAL_SECTION();  // Hold off interrupts.

    do
    {
        if ((uint32)ptr > (uint32)header + 4096)
        {
            LOG("==========error: memory audit failed===============\r\n");
            break;
        }

        // seek to the last block, return
        if ( current->val == 0 )       /// val = 0, so len = 0
        {
            break;
        }

        if (current->hdr.inUse)
            sum_alloc += current->hdr.len;

        current = (osalMemHdr_t *)((uint8 *)current + current->hdr.len);
    }
    while (1);

    HAL_EXIT_CRITICAL_SECTION();  // Re-enable interrupts.

    return sum_alloc;
}




/*********************************************************************
*********************************************************************/
