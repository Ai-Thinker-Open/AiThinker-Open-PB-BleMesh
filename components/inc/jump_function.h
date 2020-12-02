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

/**
 ****************************************************************************************
 *
 * @file jump_fucntion.h
 *
 * @brief This file contains the definitions of the macros and functions that are
 * architecture dependent.  The implementation of those is implemented in the
 * appropriate architecture directory.
 *
 *
 * $Rev:  $
 *
 ****************************************************************************************
 */


#ifndef _JUMP_FUNC_H_
#define _JUMP_FUNC_H_
#include <stdint.h>
#include "types.h"
#include "ll_def.h"
#include "ll_sleep.h"

#include "hci.h"
#include "l2cap.h"

// =====================  MACROS =======================
#define JUMP_BASE_ADDR 0x1fff0800
#define JUMP_FUNCTION(x)    (*(uint32 *)(JUMP_BASE_ADDR + (x << 2)))

// ROM function entries

        
// 0 - 10 for common         
#define     OSAL_INIT_TASKS                      1
#define     TASKS_ARRAY                          2
#define     TASK_COUNT                           3
#define     TASK_EVENTS                          4    
#define     OSAL_MEM_INIT                        5
    
#define     LL_INIT                              11
#define     LL_PROCESS_EVENT                     12
#define     LL_RESET                             13
#define     LL_TXDATA                            14
#define     LL_DISCONNECT                        15
#define     LL_SET_ADV_PARAM                     16
#define     LL_SET_ADV_DATA                      17
#define     LL_SET_ADV_CONTROL                   18
#define     LL_SET_DEFAULT_CONN_PARAM            19

#define     LL_EXT_SET_TX_POWER                  20

#define     LL_CLEAR_WHITE_LIST                  21
#define     LL_ADD_WHITE_LIST_DEV                22
#define     LL_REMOVE_WHITE_LIST_DEV             23
#define     LL_READ_WHITE_LIST_SIZE              24
#define     LL_NUM_EMPTY_WL_ENTRIES              25

#define     LL_SLAVE_EVT_ENDOK                   26
#define     LL_SETUP_NEXT_SLAVE_EVT              27
#define     LL_CHK_LSTO_DURING_SL                28
#define     LL_PROCESS_SLAVE_CTRL_PROC           29

#define     LL_PROCESS_SLAVE_CTRL_PKT            30
#define     LL_SLAVE_EVT_ABORT                   31
#define     LL_PROCESS_RX_DATA                   32
#define     LL_PROCESS_TX_DATA                   33
#define     LL_CONN_TERMINATE                    34
#define     LL_WRITE_TX_DATA                     35

#define     LL_EVT_SCHEDULE                      36
#define     LL_MOVE_TO_SLAVE_FUNCTION            37
#define     LL_SLAVE_CONN_EVENT                  38

#define     LL_SETUP_ADV                         39

#define     LL_SETUP_UNDIRECT_ADV                40
#define     LL_SETUP_NOCONN_ADV                  41

#define     LL_SETUP_SCAN_ADV                    42
#define     LL_SETUP_DIRECT_ADV                  43

#define     LL_CALC_TIMER_DRIFT                  44
#define     LL_GENERATE_TX_BUFFER                45
#define     LL_READ_RX_FIFO                      46
#define     LL_READ_TX_FIFO_RTLP                 47
#define     LL_READ_TX_FIFO_PKT                  48

#define     LL_HW_PROCESS_RTO                    49

#define     LL_HW_SET_TIMING                     50
#define     LL_RELEASE_CONN_ID                   51

#define     LL_READ_TX_PWR_LVL                   52   //  A1 ROM metal change add
#define     LL_READ_ADV_TX_PWR_LVL               53   //  A1 ROM metal change add
#define     LL_READ_RSSI                         54   //  A1 ROM metal change add
#define     LL_READ_REMOTE_USE_FEATURES          55   //  A1 ROM metal change add
#define     LL_ENCRYPT                           56   //  A1 ROM metal change add

#define     LL_DIRECT_TEST_END                   57   //  A1 ROM metal change add
#define     LL_DIRECT_TEST_TX_TEST               58   //  A1 ROM metal change add
#define     LL_DIRECT_TEST_RX_TEST               59   //  A1 ROM metal change add

#define     OSAL_POWER_CONSERVE                  60
#define     ENTER_SLEEP_PROCESS                  61
#define     WAKEUP_PROCESS                       62
#define     CONFIG_RTC                           63
#define     ENTER_SLEEP_OFF_MODE                 64   //  A1 ROM metal change add

#define     HAL_PROCESS_POLL                     65   //  A1 ROM metal change add
#define     LL_HW_GO                             66   //  A1 ROM metal change add
#define     LL_HW_TRIGGER                        67   //  A1 ROM metal change add
#define     LL_SET_TX_PWR_LVL                    68   //  A1 ROM metal change add

// LL AES
#define     LL_AES128_ENCRYPT                    70   //  A1 ROM metal change add
#define     LL_GEN_TRUE_RANDOM                   71   //  A1 ROM metal change add
#define     LL_GEN_DEVICE_SKD                    72   //  A1 ROM metal change add
#define     LL_GEN_DEVICE_IV                     73   //  A1 ROM metal change add
#define     LL_GENERATE_NOUNCE                   74   //  A1 ROM metal change add
#define     LL_ENC_ENCRYPT                       75   //  A1 ROM metal change add
#define     LL_ENC_DECRYPT                       76   //  A1 ROM metal change add

// host entries
#define     SMP_INIT                             80
#define     SMP_PROCESS_EVENT                    81

// l2cap entries
#define     L2CAP_PARSE_PACKET                   82
#define     L2CAP_ENCAP_PACKET                   83
#define     L2CAP_PKT_TO_SEGBUFF                 84
#define     L2CAP_SEGBUFF_TO_LINKLAYER           85
#define     L2CAP_PROCESS_FREGMENT_TX_DATA       86

//gap linkmgr entries
#define     GAP_LINK_MGR_PROCESS_CONNECT_EVT     87
#define     GAP_LINK_MGR_PROCESS_DISCONNECT_EVT  88

// hci tl
#define     HCI_INIT                             90   //  A1 ROM metal change add
#define     HCI_PROCESS_EVENT                    91   //  A1 ROM metal change add



// app entries
#define     APP_SLEEP_PROCESS                    100
#define     APP_WAKEUP_PROCESS                   101
#define     RF_INIT                              102
#define     WAKEUP_INIT                          103
#define     BOOT_INIT                            104
#define     DEBUG_PRINT                          105
#define     RF_CALIBRATTE                        106    //  A1 ROM metal change add

// LL master, A2 ROM metal change add
#define     LL_MASTER_EVT_ENDOK                  110    
#define     LL_SETUP_NEXT_MASTER_EVT             111
#define     LL_PROCESS_MASTER_CTRL_PROC          112
#define     LL_PROCESS_MASTER_CTRL_PKT           113
#define     LL_MOVE_TO_MASTER_FUNCTION           114
#define     LL_MASTER_CONN_EVENT                 115

#define     LL_SET_SCAN_CTRL                     116
#define     LL_SET_SCAN_PARAM                    117

#define     LL_CREATE_CONN                       118
#define     LL_CREATE_CONN_CANCEL                119

#define     LL_START_ENCRYPT                     120

#define     LL_SETUP_SCAN                        121

#define     LL_SETUP_SEC_ADV                     122
#define     LL_SETUP_SEC_SCAN                    123
#define     LL_SEC_ADV_ALLOW                     124
#define     LL_CALC_MAX_SCAN_TIME                125

//DLE   
#define     LL_SET_DATA_LENGTH                   130
#define     LL_PDU_LENGTH_UPDATE                 131
#define     LL_TRX_NUM_ADJUST                    132
//PHY UPDATE  
#define     LL_SET_PHY_MODE                      133
#define     LL_PHY_MODE_UPDATE                   134
#define     LL_SET_NEXT_PHY_MODE                 135



// interrupt request handler
#define     NMI_HANDLER                          219
#define     HARDFAULT_HANDLER                    220
#define     SVC_HANDLER                          221
#define     PENDSV_HANDLER                       222
#define     SYSTICK_HANDLER                      223

#define     V0_IRQ_HANDLER                       224
#define     V1_IRQ_HANDLER                       225
#define     V2_IRQ_HANDLER                       226
#define     V3_IRQ_HANDLER                       227
#define     V4_IRQ_HANDLER                       228
#define     V5_IRQ_HANDLER                       229
#define     V6_IRQ_HANDLER                       230
#define     V7_IRQ_HANDLER                       231
#define     V8_IRQ_HANDLER                       232
#define     V9_IRQ_HANDLER                       233
#define     V10_IRQ_HANDLER                      234
#define     V11_IRQ_HANDLER                      235
#define     V12_IRQ_HANDLER                      236
#define     V13_IRQ_HANDLER                      237
#define     V14_IRQ_HANDLER                      238
#define     V15_IRQ_HANDLER                      239
#define     V16_IRQ_HANDLER                      240
#define     V17_IRQ_HANDLER                      241
#define     V18_IRQ_HANDLER                      242
#define     V19_IRQ_HANDLER                      243
#define     V20_IRQ_HANDLER                      244
#define     V21_IRQ_HANDLER                      245
#define     V22_IRQ_HANDLER                      246
#define     V23_IRQ_HANDLER                      247
#define     V24_IRQ_HANDLER                      248
#define     V25_IRQ_HANDLER                      249
#define     V26_IRQ_HANDLER                      250
#define     V27_IRQ_HANDLER                      251
#define     V28_IRQ_HANDLER                      252
#define     V29_IRQ_HANDLER                      253
#define     V30_IRQ_HANDLER                      254
#define     V31_IRQ_HANDLER                      255

// ================== FUNCTIONS  ==================================
void move_to_slave_function0(void);
void LL_slave_conn_event0(void);
llStatus_t llSetupAdv0(void);
void llSetupUndirectedAdvEvt0(void);
void llSetupNonConnectableAdvEvt0( void );
void llSetupScannableAdvEvt0( void );
void llSetupDirectedAdvEvt0( void );
void LL_evt_schedule0(void);

void llCalcTimerDrift0( uint32    connInterval,
                        uint16   slaveLatency,
                        uint8    sleepClkAccuracy,
                        uint32   *timerDrift ) ;

uint16 ll_generateTxBuffer0(int txFifo_vacancy, uint16 *pSave_ptr);

void ll_hw_read_tfifo_rtlp0(void);

void ll_read_rxfifo0(void);

int ll_hw_read_tfifo_packet0(uint8 *pkt);

void ll_hw_process_RTO0(uint32 ack_num);

void LL_set_default_conn_params0(void);

// =====
void enterSleepProcess0(uint32 time);

void wakeupProcess0(void);

void config_RTC0(uint32 time);

void enter_sleep_off_mode0(Sleep_Mode mode);

void llSlaveEvt_TaskEndOk0( void );

uint8 llSetupNextSlaveEvent0( void );

uint8 llCheckForLstoDuringSL0( llConnState_t *connPtr );

uint8 llProcessSlaveControlProcedures0( llConnState_t *connPtr );

void llProcessSlaveControlPacket0( llConnState_t *connPtr,
                                  uint8         *pBuf );

void llSlaveEvt_TaskAbort0(void );

// ------
void llMasterEvt_TaskEndOk0( void );
void llProcessMasterControlPacket0( llConnState_t *connPtr,
                                   uint8         *pBuf );
uint8 llProcessMasterControlProcedures0( llConnState_t *connPtr );
uint8 llSetupNextMasterEvent0( void );

void move_to_master_function0(void);
void LL_master_conn_event0(void);

llStatus_t LL_SetScanControl0( uint8 scanMode,
                              uint8 filterReports );
llStatus_t LL_SetScanParam0( uint8  scanType,
                            uint16 scanInterval,
                            uint16 scanWindow,
                            uint8  ownAddrType,
                            uint8  scanWlPolicy );

llStatus_t LL_CreateConn0( uint16 scanInterval,
                          uint16 scanWindow,
                          uint8  initWlPolicy,
                          uint8  peerAddrType,
                          uint8  *peerAddr,
                          uint8  ownAddrType,
                          uint16 connIntervalMin,
                          uint16 connIntervalMax,
                          uint16 connLatency,
                          uint16 connTimeout,
                          uint16 minLength,
                          uint16 maxLength );
llStatus_t LL_CreateConnCancel0( void );    

llStatus_t LL_StartEncrypt0( uint16 connId,
                            uint8  *rand,
                            uint8  *eDiv,
                            uint8  *ltk );
                            
void llSetupScan0( uint8 chan );                            

//  ================== ll.c
void LL_Init0( uint8 taskId );
uint16 LL_ProcessEvent0( uint8 task_id, uint16 events );
llStatus_t LL_Reset0( void );
llStatus_t LL_TxData0( uint16 connId, uint8 *pBuf, uint8  pktLen, uint8  fragFlag );
llStatus_t LL_Disconnect0( uint16 connId, uint8  reason );
llStatus_t LL_SetAdvParam0( uint16 advIntervalMin,
                           uint16 advIntervalMax,
                           uint8  advEvtType,
                           uint8  ownAddrType,
                           uint8  directAddrType,
                           uint8  *directAddr,
                           uint8  advChanMap,
                           uint8  advWlPolicy );
llStatus_t LL_SetAdvData0( uint8  advDataLen, uint8 *advData );
llStatus_t LL_SetAdvControl0( uint8 advMode );

llStatus_t LL_EXT_SetTxPower0( uint8 txPower, uint8 *cmdComplete );

llStatus_t LL_ClearWhiteList0( void );
llStatus_t LL_AddWhiteListDevice0( uint8 *devAddr, uint8 addrType );
llStatus_t LL_RemoveWhiteListDevice0( uint8 *devAddr, uint8 addrType );
llStatus_t LL_ReadWlSize0( uint8 *numEntries );
llStatus_t LL_ReadTxPowerLevel0( uint8 connId, uint8 type, int8  *txPower );
llStatus_t LL_SetTxPowerLevel0( int8  txPower );
llStatus_t LL_ReadAdvChanTxPower0( int8 *txPower );
llStatus_t LL_ReadRssi0( uint16 connId, int8   *lastRssi );
llStatus_t LL_ReadRemoteUsedFeatures0( uint16 connId );
llStatus_t LL_Encrypt0( uint8 *key, uint8 *plaintextData, uint8 *encryptedData );

llStatus_t LL_DirectTestEnd0( void );
llStatus_t LL_DirectTestTxTest0( uint8 txFreq, uint8 payloadLen, uint8 payloadType );
llStatus_t LL_DirectTestRxTest0( uint8 rxFreq );

// ================ ll_common.c
void llProcessTxData0( llConnState_t *connPtr, uint8 context );
uint8 llProcessRxData0( void );
uint8 llWriteTxData0( llConnState_t *connPtr,
                      uint8          pktHdr,
                      uint8          pktLen,
                      uint8         *pBuf );
void llConnTerminate0( llConnState_t *connPtr, uint8 reason );
void llReleaseConnId0( llConnState_t *connPtr );

// ================ ll_enc.c
void LL_ENC_AES128_Encrypt0( uint8 *key,
                            uint8 *plaintext,
                            uint8 *ciphertext );
uint8 LL_ENC_GenerateTrueRandNum0( uint8 *buf,
                                  uint8 len );
void LL_ENC_GenDeviceSKD0( uint8 *SKD );
void LL_ENC_GenDeviceIV0( uint8 *IV );
void LL_ENC_GenerateNonce0( uint32 pktCnt,
                           uint8  direction,
                           uint8  *nonce );
void LL_ENC_Encrypt0( llConnState_t *connPtr,
                     uint8          pktHdr,
                     uint8          pktLen,
                     uint8         *pBuf );
uint8 LL_ENC_Decrypt0( llConnState_t *connPtr,
                      uint8          pktHdr,
                      uint8          pktLen,
                      uint8         *pBuf );

// =================== osal
void osal_pwrmgr_powerconserve0( void ) ;

// =================== ll_hw_drv.c
void ll_hw_set_timing0(uint8 pktFmt);
void ll_hw_go0(void);
void ll_hw_trigger0(void);

// ================== SMP functions
void SM_Init0( uint8 task_id );
uint16 SM_ProcessEvent0( uint8 task_id, uint16 events );

// ================== HCI_TL functions
void HCI_Init0( uint8 task_id );
uint16 HCI_ProcessEvent0( uint8 task_id, uint16 events );


// ======= OSAL memory 
void osal_mem_init0(void);

// =========== ROM -> APP function
void app_sleep_process(void);

void app_wakeup_process(void);

void rf_init(void);

void boot_init0(void);

void wakeup_init0(void);

void debug_print(uint32 state);

void rf_calibrate0(void);

// ========== A2, for conn-adv, conn-scan
uint8 llSetupSecNonConnectableAdvEvt0( void );
uint8 llSecAdvAllow0(void);
uint32 llCalcMaxScanTime0(void);
void llSetupSecScan0( uint8 chan );

//=============== gap_linkmgr.c
void gapProcessDisconnectCompleteEvt0( hciEvt_DisconnComplete_t *pPkt );
void gapProcessConnectionCompleteEvt0( hciEvt_BLEConnComplete_t *pPkt );


//=============== l2cap_util.c
uint8 l2capParsePacket0( l2capPacket_t *pPkt, hciDataEvent_t *pHciMsg );
uint8 l2capEncapSendData0( uint16 connHandle, l2capPacket_t *pPkt );
uint8 l2capPktToSegmentBuff0(uint16 connHandle, l2capSegmentBuff_t* pSegBuf, uint8 blen,uint8* pBuf);
void l2capPocessFragmentTxData0(uint16 connHandle);
uint8 l2capSegmentBuffToLinkLayer0(uint16 connHandle, l2capSegmentBuff_t* pSegBuf);
void l2capPocessFragmentTxData0(uint16 connHandle);

//=============== DLE
llStatus_t LL_SetDataLengh0( uint16 connId,uint16 TxOctets,uint16 TxTime );
void llPduLengthUpdate0(uint16 connHandle);
void llTrxNumAdaptiveConfig0(void);

//=============== PHY UPDATE
llStatus_t LL_SetPhyMode0( uint16 connId,uint8 allPhy,uint8 txPhy, uint8 rxPhy,uint16 phyOptions);
llStatus_t LL_PhyUpdate0( uint16 connId );
void llSetNextPhyMode0( llConnState_t *connPtr );


#endif // _JUMP_FUNC_H_
