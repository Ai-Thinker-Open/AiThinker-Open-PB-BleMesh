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


/**************************************************************
    COPYRIGHT(c)2017, iRAID Finance Info & Tech （Shanghai）Co. Ltd.
    All rights reserved.


    Module Name: error type definition
    File name:   iraid_err.h
    Brief description:
      iraid error type definition for globle use
    Author:  Eagle.Lao
    Email:   yufeng.lao@xwf-id.com
    Data:    2017-06-12
    Revision:V0.01
****************************************************************/


#ifndef __IRAID_ERR_DEFINE_HEAD
#define __IRAID_ERR_DEFINE_HEAD
enum
{
    APP_SUCCESS,
    APP_ERR_Q_FULL = 1,
    APP_ERR_Q_ERR,
    APP_ERR_BUSY,
    APP_ERR_INVALID_PAGE,
    APP_ERR_CRC,
    APP_ERR_UNKNOW_CMD,
    APP_ERR_FDS,
    APP_ERR_FDS_BUSY,
    APP_ERR_FDS_TIMEOUT,
    APP_ERR_PARAM,
    APP_ERR_OUTOF_RANGE,
    APP_ERR_DATA_ALIGN,
    APP_ERR_BLE_SEND_FAIL,
    APP_ERR_BLE_DISCONNECT,
    APP_ERR_NOT_IMPLEMENTED,
    APP_ERR_SPI_ERR,
    APP_ERR_TIME_OVER,
    APP_ERR_NOT_VALID_RECORD,
    APP_ERR_IO,
    APP_ERR_TIMER,
    APP_ERR_INVALID_STATE,
    APP_ERR_HEARTRATE,
    APP_ERR_ISO7816,
    APP_ERR_UNICODE,
    APP_ERR_FONT,
    APP_ERR_BINDING_FAILED,
    APP_ERR_BINDING_TIMEOUT,


};

#endif

