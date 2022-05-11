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

/**************************************************************************************************
    Filename:       bleuart_at_cmd.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "OSAL.h"
#include "bleuart_at_cmd.h"
#include "log.h"
#include "rf_phy_driver.h"

#include "gpio.h"
#include "clock.h"
#include "OSAL_Timers.h"

#include "pwrmgr.h"
#include "error.h"
#include "global_config.h"
#include "cliface.h"
#include "kscan.h"
#include "osal_snv.h"
#include "spi.h"
#include "flash.h"
#include "fs.h"
#include "spiflash.h"
#include "bleuart.h"
#include "version.h"
#include "peripheral.h"
#include "gap.h"
#include "bleuart_protocol.h"
#include "led.h"
#include "bleuart_at_dma.h"

// Flash r/w error for debug.
#define READ_FLASH_ERROR             3
#define READ_FLASH_DATA_ERROR        4
#define WRITE_FLASH_ERROR            5

#define AT_CMD_LENGTH_MAX            64

// snv ID index for bleuart_at module.
#define AT_SNV_ID_BASE                   384
#define AT_SNV_ID_CONNECTABLE_OFFSET     0
#define AT_SNV_ID_LED_MOD_OFFSET         1
#define AT_SNV_ID_BLE_ROLE_OFFSET        2
#define AT_SNV_ID_MOD_NAME_OFFSET        3
#define AT_SNV_ID_MAC_ADDR_OFFSET        4
#define AT_SNV_ID_MOD_PIN_OFFSET         5
#define AT_SNV_ID_RFPW_OFFSET            6
#define AT_SNV_ID_PW_MOD_OFFSET          7
#define AT_SNV_ID_ADV_INT_OFFSET         8
#define AT_SNV_ID_CONN_INT_OFFSET        9
#define AT_SNV_ID_CONN_TOUT_OFFSET       10
#define AT_SNV_ID_SEARCH_UUID_OFFSET     11
#define AT_SNV_ID_SRV_UUID_OFFSET        12
#define AT_SNV_ID_PT_UUID_OFFSET         13
#define AT_SNV_ID_BAUDRATE_OFFSET        14
#define AT_SNV_ID_AUTO_SLP_TIME_OFFSET   15
#define AT_SNV_ID_MAX_OFFSET             15
#define AT_SNV_ID_INDEX(idx)   ((AT_SNV_ID_BASE) + (idx))

/* TODO: Create a separe utility module or move to a common utility module */
/* Supporting Macros */
#define IS_SPACE(c) ((' ' == (c)) || ('\t' == (c)))
#define IS_DIGIT(c) (('0' <= (c)) && ('9' >= (c)))
#define IS_UPPER(c) (('A' <= (c)) && ('F' >= (c)))
#define IS_LOWER(c) (('a' <= (c)) && ('f' >= (c)))
#define IS_ALPHA(c) IS_LOWER(c) || IS_UPPER(c)
#define IS_HEX_NUMBER(c) ((IS_DIGIT(c)) || (IS_UPPER(c)) || (IS_LOWER(c)))

// used by at+avda cmd. It would not be saved after reset.
uint8_t g_adv_data[AT_ADV_DATA_SIZE] = "phy_01";
bool g_dma_flag    = false;  // flag whether DMA block transfer would be used.
// flag to indicate UART-TO-BLE(true, rx) or BLE-TO-UART(false, tx), only valid when g_dma_flag = true.
bool g_rxpath_flag = true;

// for at_cli_cmd.
uint8_t cmdstr[AT_CMD_LENGTH_MAX];
uint8_t cmdlen = 0;

AT_ctx_t mAT_Ctx;
AT_ctx_t mAT_Ctx_def =
{
    .connectable     = false, // connectable or not
    .led_mod         = true,  // on
    .ble_role        = false, // slave mode
    .mod_name        = "PHY-01",
    .mac_addr        = {0x01,0x02,0x03,0x04,0x05,0x06},
    .mod_pin         = "000001",
    .rfpw            = 0,  // 4dBm
    .pw_mod          = 0,  // power save mode.
    .adv_int         = AT_ADV_INTERVAL_DEF_625US, // 200ms
    .conn_int        = {6,12},
    .conn_timeout    = AT_CONN_TIMEOUT_DEF_10MS, //2S
    #ifdef BLEUART_DEDICATE
    .baudrate        = {9600,(uint32_t)UART_PARITY_N}, // 9600,no parity by default.
    #else
    .baudrate        = 115200,
    #endif
    .search_uuid     = 0xfff0,  // find UUID
    .srv_uuid        = 0xffe0,  // service UUID
    .pt_uuid         = 0xffe1,  // pass through UUID
    .auto_slp_time   = 20,      // auto sleep timeout
};

uint16_t at_at(uint32_t argc, uint8_t* argv[]);
uint16_t at_rx(uint32_t argc, uint8_t* argv[]);
uint16_t at_default(uint32_t argc, uint8_t* argv[]);
uint16_t at_reset(uint32_t argc, uint8_t* argv[]);
uint16_t at_version(uint32_t argc, uint8_t* argv[]);
uint16_t at_role(uint32_t argc, uint8_t* argv[]);
uint16_t at_name(uint32_t argc, uint8_t* argv[]);
uint16_t at_addr(uint32_t argc, uint8_t* argv[]);
uint16_t at_rfpw(uint32_t argc, uint8_t* argv[]);
uint16_t at_baud(uint32_t argc, uint8_t* argv[]);
uint16_t at_cont(uint32_t argc, uint8_t* argv[]);
uint16_t at_avda(uint32_t argc, uint8_t* argv[]);
uint16_t at_mode(uint32_t argc, uint8_t* argv[]);
uint16_t at_aint(uint32_t argc, uint8_t* argv[]);
uint16_t at_cint(uint32_t argc, uint8_t* argv[]);
uint16_t at_ctout(uint32_t argc, uint8_t* argv[]);
uint16_t at_clear(uint32_t argc, uint8_t* argv[]);
uint16_t at_led(uint32_t argc, uint8_t* argv[]);
uint16_t at_luuid(uint32_t argc, uint8_t* argv[]);
uint16_t at_suuid(uint32_t argc, uint8_t* argv[]);
uint16_t at_tuuid(uint32_t argc, uint8_t* argv[]);
uint16_t at_aust(uint32_t argc, uint8_t* argv[]);
// new added.
uint16_t at_div(uint32_t argc, uint8_t* argv[]);
uint16_t at_pcnt(uint32_t argc, uint8_t* argv[]);
uint16_t at_dma(uint32_t argc, uint8_t* argv[]);
uint16_t at_rxpath(uint32_t argc, uint8_t* argv[]);

/**
    Use FS API to fetch data from/to flash.
    osal_snv_read()/osal_snv_write()
**/
//uint8_t at_snv_read_flash(void);
uint8_t at_snv_write_flash(uint16_t idx);
void at_snv_write_flash_all(void);


/**
    Define AT command list.
    Format: "<at cmd>", "<description>", <fuction to be called>
**/
const CLI_COMMAND cli_cmd_list[] =
{
    #ifdef BLEUART_DEDICATE
    /* at cmd. */
    { "AT", "Check whether UART is in AT mode", at_at },

    /* at+rx cmd. */
    // name, role, baud, mac addr, and pin .
    { "AT+RX", "Check basic parameters of module", at_rx },

    /* at+default cmd. */
    { "AT+DEFAULT", "Restore module to factory default", at_default },

    /* at+reset cmd. */
    { "AT+RESET", "Reset module", at_reset },

    /* at+version cmd. */
    // module, software version, release data. eg. HC-08 V3.1,2017-07-07
    { "AT+VERSION", "Fetch module version", at_version },

    /* at+role cmd. */
    // slave or master
    { "AT+ROLE", "Switch Master/Slave role", at_role },

    /* at+name cmd. */
    { "AT+NAME", "Change bluetooth name", at_name },

    /* at+addr cmd. */
    { "AT+ADDR", "Change bluetooth address", at_addr },

    /* at+rfpw cmd. */
    { "AT+RFPW", "Change RF power", at_rfpw },

    /* at+baud cmd. */
    { "AT+BAUD", "Change UART baudrate", at_baud },

    /* at+cont cmd. */
    { "AT+CONT", "Set whether the module is connectable", at_cont },

    /* at+avda cmd. */
    { "AT+AVDA", "Change advertising data", at_avda },

    /* at+mode cmd. */
    { "AT+MODE", "Change power mode", at_mode },

    /* at+aint cmd. */
    { "AT+AINT", "Change advertising interval", at_aint },

    /* at+cint cmd. */
    { "AT+CINT", "Change connect interval", at_cint },

    /* at+ctout cmd. */
    { "AT+CTOUT", "Change connect timeout", at_ctout },

    /* at+clear cmd. */
    { "AT+CLEAR", "Master clean recorded slave address", at_clear },

    /* at+led cmd. */
    { "AT+LED", "Set LED work mode", at_led },

    /* at+luuid cmd. */
    { "AT+LUUID", "Set connect UUID", at_luuid },

    /* at+suuid cmd. */
    { "AT+SUUID", "Set service UUID", at_suuid },

    /* at+tuuid cmd. */
    { "AT+TUUID", "Set transfer UUID", at_tuuid },

    /* at+aust cmd. */
    { "AT+AUST", "Set time for auto sleep", at_aust },

    /* at+it cmd. */
    //{ "at+it", "Set timeout for Rx idle", at_it },
    #else
    /* at cmd. */
    { "at", "Check whether UART is in AT mode", at_at },

    /* at+rx cmd. */
    // name, role, baud, mac addr, and pin .
    { "at+rx", "Check basic parameters of module", at_rx },

    /* at+default cmd. */
    { "at+default", "Restore module to factory default", at_default },

    /* at+reset cmd. */
    { "at+reset", "Reset module", at_reset },

    /* at+version cmd. */
    // module, software version, release data. eg. HC-08 V3.1,2017-07-07
    { "at+version", "Fetch module version", at_version },

    /* at+role cmd. */
    // slave or master
    { "at+role", "Switch Master/Slave role", at_role },

    /* at+name cmd. */
    { "at+name", "Change bluetooth name", at_name },

    /* at+addr cmd. */
    { "at+addr", "Change bluetooth address", at_addr },

    /* at+rfpw cmd. */
    { "at+rfpw", "Change RF power", at_rfpw },

    /* at+baud cmd. */
    { "at+baud", "Change UART baudrate", at_baud },

    /* at+cont cmd. */
    { "at+cont", "Set whether the module is connectable", at_cont },

    /* at+avda cmd. */
    { "at+avda", "Change advertising data", at_avda },

    /* at+mode cmd. */
    { "at+mode", "Change power mode", at_mode },

    /* at+aint cmd. */
    { "at+aint", "Change advertising interval", at_aint },

    /* at+cint cmd. */
    { "at+cint", "Change connect interval", at_cint },

    /* at+ctout cmd. */
    { "at+ctout", "Change connect timeout", at_ctout },

    /* at+clear cmd. */
    { "at+clear", "Master clean recorded slave address", at_clear },

    /* at+led cmd. */
    { "at+led", "Set LED work mode", at_led },

    /* at+luuid cmd. */
    { "at+luuid", "Set connect UUID", at_luuid },

    /* at+suuid cmd. */
    { "at+suuid", "Set service UUID", at_suuid },

    /* at+tuuid cmd. */
    { "at+tuuid", "Set transfer UUID", at_tuuid },

    /* at+aust cmd. */
    { "at+aust", "Set time for auto sleep", at_aust },

    /* at+it cmd. */
    { "at+div", "Set buf size per packet", at_div },

    /* at+pcnt cmd. On debug purpose*/
    { "at+pcnt", "Get packet count number", at_pcnt },

    /* at+dma cmd. */
    { "at+dma", "Set DMA passthough", at_dma },

    /* at+rxpath cmd. */
    { "at+rxpath", "Set rx/tx DMA path", at_rxpath },
    #endif
};

/*  This is defined in clock.c file for now
    #ifdef PHY_6222
    void hal_system_soft_reset(void)
    {
     (volatile uint32_t *) 0x40000010 = 0x00;
    }
    #endif
*/

uint16_t at_at(uint32_t argc, uint8_t* argv[])
{
    if(get_uart_at_mod())
    {
        osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);
        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    }
    else  // skip auto sleep timer in PT mode.
    {
        AT_LOG("\nOK\n");
    }

    return 0;
}

uint16_t at_rx(uint32_t argc, uint8_t* argv[])
{
    uint8_t i;
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 0)
    {
        // Name, role, baudrate, ble addr, pin,
        #ifdef BLEUART_DEDICATE
        AT_LOG("\nName:%s",mAT_Ctx.mod_name);

        if(mAT_Ctx.ble_role)
            AT_LOG("\nRole:Master");
        else
            AT_LOG("\nRole:Slave");

        AT_LOG("\nBaud:%d",mAT_Ctx.baudrate[0]);

        switch(mAT_Ctx.baudrate[1])
        {
        case UART_PARITY_N:
            AT_LOG(",NONE");
            break;

        case UART_PARITY_O:
            AT_LOG(",ODD");
            break;

        case UART_PARITY_E:
            AT_LOG(",EVEN");
            break;

        default:
            AT_LOG(",NONE");
            break;
        }

        AT_LOG("\nAddr:%02x",mAT_Ctx.mac_addr[0]);

        for(i = 1; i < 6; i++ )
            AT_LOG(",%02x",mAT_Ctx.mac_addr[i]);

        AT_LOG("\nPIN :%s",mAT_Ctx.mod_pin);
        #else
        AT_LOG("\nName     £º%s",mAT_Ctx.mod_name);

        if(mAT_Ctx.ble_role)
            AT_LOG("\nRole     £ºMaster");
        else
            AT_LOG("\nRole     £ºSlave");

        AT_LOG("\nBaudrate : %d",mAT_Ctx.baudrate);
        AT_LOG("\nBLE addr £º%02x",mAT_Ctx.mac_addr[0]);

        for(i = 1; i < 6; i++ )
            AT_LOG(":%02x",mAT_Ctx.mac_addr[i]);

        AT_LOG("\nPIN      £º%s",mAT_Ctx.mod_pin);
        #endif
        AT_LOG("\nOK\n");
    }

    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_default(uint32_t argc, uint8_t* argv[])
{
    uint8_t i;
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 0)
    {
        mAT_Ctx.connectable   = mAT_Ctx_def.connectable;
        mAT_Ctx.led_mod       = mAT_Ctx_def.led_mod;
        mAT_Ctx.ble_role      = mAT_Ctx_def.ble_role;
        strcpy((char*)(mAT_Ctx.mod_name), (char*)(mAT_Ctx_def.mod_name));

        for(i = 0; i < 6; i++ )
            mAT_Ctx.mac_addr[i] = mAT_Ctx_def.mac_addr[i];

        strcpy((char*)(mAT_Ctx.mod_pin), (char*)(mAT_Ctx_def.mod_pin));
        mAT_Ctx.rfpw          = mAT_Ctx_def.rfpw;
        mAT_Ctx.pw_mod        = mAT_Ctx_def.pw_mod;
        mAT_Ctx.adv_int       = mAT_Ctx_def.adv_int;
        mAT_Ctx.conn_int[0]   = mAT_Ctx_def.conn_int[0];
        mAT_Ctx.conn_int[1]   = mAT_Ctx_def.conn_int[1];
        mAT_Ctx.conn_timeout  = mAT_Ctx_def.conn_timeout;
        #ifdef BLEUART_DEDICATE
        mAT_Ctx.baudrate[0]      = mAT_Ctx_def.baudrate[0];
        mAT_Ctx.baudrate[1]      = mAT_Ctx_def.baudrate[1];
        #else
        mAT_Ctx.baudrate      = mAT_Ctx_def.baudrate;
        #endif
        mAT_Ctx.search_uuid   = mAT_Ctx_def.search_uuid;
        mAT_Ctx.srv_uuid      = mAT_Ctx_def.srv_uuid;
        mAT_Ctx.pt_uuid       = mAT_Ctx_def.pt_uuid;
        mAT_Ctx.auto_slp_time = mAT_Ctx_def.auto_slp_time;
        // write data into flash.
        at_snv_write_flash_all();
        AT_LOG("\nOK\n");
    }

    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_reset(uint32_t argc, uint8_t* argv[])
{
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 0)
    {
        AT_LOG("\r\nOK\r\n");
        hal_system_soft_reset(); //in common.h
    }

    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_version(uint32_t argc, uint8_t* argv[])
{
    uint8_t i,ver[8] = {0};
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);
    ver[0] = '0' + (SDK_VER_MAJOR) / 10;
    ver[1] = '0' + (SDK_VER_MAJOR) % 10;
    ver[2] = '.';
    ver[3] = '0' + (SDK_VER_MINOR) / 10;
    ver[4] = '0' + (SDK_VER_MINOR) % 10;
    ver[5] = '.';
    ver[6] = '0' + (SDK_VER_REVISION) / 10;
    ver[7] = '0' + (SDK_VER_REVISION) % 10;
    AT_LOG("\nver: ");

    for(i=0; i<8; i++)
        AT_LOG("%c",ver[i]);

    AT_LOG("\nOK\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_role(uint32_t argc, uint8_t* argv[])
{
    uint8_t ret = 0;
    bool orig_role;
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 1)
    {
        if('?' == argv[0][0])
        {
            if(mAT_Ctx.ble_role) // true -- Master.
                AT_LOG("\nMaster");
            else // false -- slave
                AT_LOG("\nSlave");
        }
        else
        {
            orig_role = mAT_Ctx.ble_role;

            //check whether the data string is valid or not.
            switch(argv[0][0])
            {
            case 'M':
            case 'm':
                mAT_Ctx.ble_role = true;
                break;

            case 'S':
            case 's':
                mAT_Ctx.ble_role = false;
                break;

            default:
                goto ERR_exit;
            }

            ret = at_snv_write_flash(AT_SNV_ID_BLE_ROLE_OFFSET); // write data into flash.

            if(ret == SUCCESS)
            {
                AT_LOG("\nOK\n");
                hal_system_soft_reset(); //would reset after update the role.
            }
            else
            {
                mAT_Ctx.ble_role = orig_role;
                goto ERR_exit;
            }
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_name(uint32_t argc, uint8_t* argv[])
{
    uint8_t ret = 0;
    uint8_t orig_mod_name[AT_MODULE_NAME_SIZE] = {0};
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 1)
    {
        if('?' == argv[0][0])
        {
            AT_LOG("\n%s",mAT_Ctx.mod_name);
        }
        else
        {
            strcpy((char*)(orig_mod_name), (char*)(mAT_Ctx.mod_name));   // backup original one.

            if(strlen((char*)(argv[0])) >= AT_MODULE_NAME_SIZE)   // input string is too long, cut partial of it.
            {
                strncpy((char*)(mAT_Ctx.mod_name), (char*)(argv[0]), AT_MODULE_NAME_SIZE - 1);
                mAT_Ctx.mod_name[AT_MODULE_NAME_SIZE - 1] = '\0';
            }
            else
                strcpy((char*)(mAT_Ctx.mod_name), (char*)(argv[0]));

            ret = at_snv_write_flash(AT_SNV_ID_MOD_NAME_OFFSET); // write data into flash.

            if(ret != SUCCESS)
            {
                strcpy((char*)(mAT_Ctx.mod_name), (char*)(orig_mod_name));   // restore original one.
                goto ERR_exit;
            }

            bleuart_gen_scanRspData(mAT_Ctx.mod_name, mAT_Ctx.conn_int, mAT_Ctx.rfpw); //update scan response data.
            update_AdvDataFromAT(true); // update scan rsp data.
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_addr(uint32_t argc, uint8_t* argv[])
{
    uint8_t i = 0;
    uint8_t ret = 0;
    uint32_t data = 0;
    uint8_t mac_tmp[6] = {0};
    uint8_t orig_mac[6] = {0};
    uint8_t data_tmp[2] = {0};
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 1)
    {
        if('?' == argv[0][0])
        {
            AT_LOG("\n%02x",mAT_Ctx.mac_addr[0]);

            for(i = 1; i < 6; i++ )
                AT_LOG(":%02x",mAT_Ctx.mac_addr[i]);
        }
        else
        {
            // backup original one.
            for(i = 0; i < 6; i++ )
                orig_mac[i] = mAT_Ctx.mac_addr[i];

            if(strlen((char*)(argv[0])) != 17)  // the length should be 17 exactly. in 'xx:xx:xx:xx:xx:xx' format.
                goto ERR_exit;

            for(i = 0; i < 6; i++)
            {
                if(i < 5)
                {
                    if( IS_HEX_NUMBER(argv[0][3*i]) && IS_HEX_NUMBER(argv[0][3*i + 1]) && (':' == argv[0][3*i + 2]) )
                    {
                        data_tmp[0] = argv[0][3*i];
                        data_tmp[1] = argv[0][3*i + 1];
                        data = CLI_strtoi(data_tmp, 2,16);
                        mac_tmp[i] = (uint8_t)data;
                    }
                    else
                        goto ERR_exit;
                }
                else  // i == 5. the laster one skip ':'.
                {
                    if( IS_HEX_NUMBER(argv[0][3*i]) && IS_HEX_NUMBER(argv[0][3*i + 1]) )
                    {
                        data_tmp[0] = argv[0][3*i];
                        data_tmp[1] = argv[0][3*i + 1];
                        data = CLI_strtoi(data_tmp, 2,16);
                        mac_tmp[i] = (uint8_t)data;
                    }
                    else
                        goto ERR_exit;
                }
            }

            // copy the mac data into mAT_Ctx.
            for(i = 0; i < 6; i++ )
                mAT_Ctx.mac_addr[i] = mac_tmp[i];

            ret = at_snv_write_flash(AT_SNV_ID_MAC_ADDR_OFFSET); // write data into flash.

            if(ret != SUCCESS)
            {
                for(i = 0; i < 6; i++ )
                    mAT_Ctx.mac_addr[i] = orig_mac[i];

                goto ERR_exit;
            }

            // need write it into other flash addr? TBD.
            // skip it as no erase API is open for AT. alternatively, update the bd_addr at bleuart_init phase.
            //data = (mAT_Ctx.mac_addr[5] << 24) | (mAT_Ctx.mac_addr[4] << 16) | (mAT_Ctx.mac_addr[3] << 8) | (mAT_Ctx.mac_addr[2]);
            //WriteFlash(0x4000, data);
            //data16 = (mAT_Ctx.mac_addr[1] << 8) | (mAT_Ctx.mac_addr[0]);
            //WriteFlashShort(0x4004, data16);
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_rfpw(uint32_t argc, uint8_t* argv[])
{
    /**
          0 -- 5dBm (default)
          1 -- 0dBm
          2 -- -5dBm
          3 -- -20dBm
          4 -- 10dBm
     **/
    uint8_t ret = 0;
    uint8_t original_rfpw, t_rfpw = 0;
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 1)
    {
        if('?' == argv[0][0])
        {
            switch(mAT_Ctx.rfpw)  // true -- Master.
            {
            case 0:
                AT_LOG("\n5dBm");
                break;

            case 1:
                AT_LOG("\n0dBm");
                break;

            case 2:
                AT_LOG("\n-5dBm");
                break;

            case 3:
                AT_LOG("\n-20dBm");
                break;

            case 4:
                AT_LOG("\n10dBm");
                break;

            default:
                goto ERR_exit;
            }
        }
        else
        {
            original_rfpw = mAT_Ctx.rfpw;

            //check whether the data string is valid or not.
            switch(argv[0][0])
            {
            case '0':
                mAT_Ctx.rfpw = 0;  //RF_PHY_TX_POWER_5DBM. <rf_phy_driver.h>
                t_rfpw = RF_PHY_TX_POWER_5DBM;
                break;

            case '1':
                mAT_Ctx.rfpw = 1;  //RF_PHY_TX_POWER_0DBM
                t_rfpw = RF_PHY_TX_POWER_0DBM;
                break;

            case '2':
                mAT_Ctx.rfpw = 2;  //RF_PHY_TX_POWER_N5DBM
                t_rfpw = RF_PHY_TX_POWER_N5DBM;
                break;

            case '3':
                mAT_Ctx.rfpw = 3;  //RF_PHY_TX_POWER_N20DBM
                t_rfpw = RF_PHY_TX_POWER_N20DBM;
                break;

            case '4':
                mAT_Ctx.rfpw = 4;  //RF_PHY_TX_POWER_EXTRA_MAX
                t_rfpw = RF_PHY_TX_POWER_EXTRA_MAX;
                break;

            default:
                goto ERR_exit;
            }

            ret = at_snv_write_flash(AT_SNV_ID_RFPW_OFFSET); // write data into flash.

            if(ret != SUCCESS)   //restore original value in case failed.
            {
                mAT_Ctx.rfpw = original_rfpw;
                goto ERR_exit;
            }

            rf_phy_set_txPower(t_rfpw); // update tx power accordingly.
            bleuart_gen_scanRspData(mAT_Ctx.mod_name, mAT_Ctx.conn_int, mAT_Ctx.rfpw); //update scan response data.
            update_AdvDataFromAT(true); // update scan rsp data.
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_baud(uint32_t argc, uint8_t* argv[])
{
    uint8_t ret = 0;
    uint32_t orig_baudrate;
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);
    #ifdef BLEUART_DEDICATE
    uint32_t orig_parity = 0;

    if(argc == 1)
    {
        if('?' == argv[0][0])
        {
            AT_LOG("\n%d",mAT_Ctx.baudrate[0]);

            switch(mAT_Ctx.baudrate[1])
            {
            case UART_PARITY_N:
                AT_LOG(",NONE");
                break;

            case UART_PARITY_O:
                AT_LOG(",ODD");
                break;

            case UART_PARITY_E:
                AT_LOG(",EVEN");
                break;

            default:
                AT_LOG(",NONE");
                break;
            }

            AT_LOG("\nOK\n");
            osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
            return 0;
        }
    }
    else if(argc == 2)
    {
        // handle baudrate first
        orig_baudrate = mAT_Ctx.baudrate[0];

        //check whether the data string is valid or not.
        if(strcmp((char*)(argv[0]), "4800") == 0)
            mAT_Ctx.baudrate[0] = 4800;
        else if(strcmp((char*)(argv[0]), "9600") == 0)
            mAT_Ctx.baudrate[0] = 9600;
        else if(strcmp((char*)(argv[0]), "14400") == 0)
            mAT_Ctx.baudrate[0] = 14400;
        else if(strcmp((char*)(argv[0]), "19200") == 0)
            mAT_Ctx.baudrate[0] = 19200;
        else if(strcmp((char*)(argv[0]), "38400") == 0)
            mAT_Ctx.baudrate[0] = 38400;
        else if(strcmp((char*)(argv[0]), "57600") == 0)
            mAT_Ctx.baudrate[0] = 57600;
        else if(strcmp((char*)(argv[0]), "115200") == 0)
            mAT_Ctx.baudrate[0] = 115200;
        else
            goto ERR_exit;

        // handle parity.
        orig_parity = mAT_Ctx.baudrate[1];

        if((argv[1][0] == 'N') || (argv[1][0] == 'n'))
            mAT_Ctx.baudrate[1] = (uint32_t)(UART_PARITY_N);
        else if((argv[1][0] == 'O') || (argv[1][0] == 'o'))
            mAT_Ctx.baudrate[1] = (uint32_t)(UART_PARITY_O);
        else if((argv[1][0] == 'E') || (argv[1][0] == 'e'))
            mAT_Ctx.baudrate[1] = (uint32_t)(UART_PARITY_E);
        else
        {
            mAT_Ctx.baudrate[0] = orig_baudrate; // restore original baudrate.
            goto ERR_exit;
        }

        if((orig_baudrate != mAT_Ctx.baudrate[0]) || (orig_parity != mAT_Ctx.baudrate[1]))  // baudrate/parity is changed.
        {
            ret = at_snv_write_flash(AT_SNV_ID_BAUDRATE_OFFSET); // write data into flash.

            if(ret != SUCCESS)
            {
                mAT_Ctx.baudrate[0] = orig_baudrate;
                mAT_Ctx.baudrate[1] = orig_parity;
                goto ERR_exit;
            }

            AT_LOG("\nOK\n");
            // re-initialize the uart after updated the baudrate
            at_Init();
        }
        else
        {
            AT_LOG("\nOK\n");
        }

        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

    #else

    if(argc == 1)
    {
        if('?' == argv[0][0])
        {
            AT_LOG("\n%d",mAT_Ctx.baudrate);
            AT_LOG("\nOK\n");
        }
        else
        {
            orig_baudrate = mAT_Ctx.baudrate;

            //check whether the data string is valid or not.
            if(strcmp((char*)(argv[0]), "4800") == 0)
                mAT_Ctx.baudrate = 4800;
            else if(strcmp((char*)(argv[0]), "9600") == 0)
                mAT_Ctx.baudrate = 9600;
            else if(strcmp((char*)(argv[0]), "14400") == 0)
                mAT_Ctx.baudrate = 14400;
            else if(strcmp((char*)(argv[0]), "19200") == 0)
                mAT_Ctx.baudrate = 19200;
            else if(strcmp((char*)(argv[0]), "38400") == 0)
                mAT_Ctx.baudrate = 38400;
            else if(strcmp((char*)(argv[0]), "57600") == 0)
                mAT_Ctx.baudrate = 57600;
            else if(strcmp((char*)(argv[0]), "115200") == 0)
                mAT_Ctx.baudrate = 115200;
            else if(strcmp((char*)(argv[0]), "500000") == 0)
                mAT_Ctx.baudrate = 500000;
            else if(strcmp((char*)(argv[0]), "1000000") == 0)
                mAT_Ctx.baudrate = 1000000;
            else
                goto ERR_exit;

            if(orig_baudrate != mAT_Ctx.baudrate)  // baudrate is changed.
            {
                ret = at_snv_write_flash(AT_SNV_ID_BAUDRATE_OFFSET); // write data into flash.

                if(ret != SUCCESS)
                {
                    mAT_Ctx.baudrate = orig_baudrate;
                    goto ERR_exit;
                }

                AT_LOG("\nOK\n");
                // re-initialize the uart after updated the baudrate
                at_Init();
            }
            else
            {
                AT_LOG("\nOK\n");
            }
        }

        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

    #endif
ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_cont(uint32_t argc, uint8_t* argv[])
{
    uint8_t ret = 0;
    bool orig_cont;
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 1)
    {
        if('?' == argv[0][0])
        {
            if(mAT_Ctx.connectable) // 1 -- not connectable
                AT_LOG("\nNot-connectable");
            else
                AT_LOG("\nConnectable");
        }
        else
        {
            orig_cont = mAT_Ctx.connectable;

            if('1' == argv[0][0])
                mAT_Ctx.connectable = true;
            else if('0' == argv[0][0])
                mAT_Ctx.connectable = false;
            else
                goto ERR_exit;

            ret = at_snv_write_flash(AT_SNV_ID_CONNECTABLE_OFFSET); // write data into flash.

            if(ret != SUCCESS)
            {
                mAT_Ctx.connectable = orig_cont;
                goto ERR_exit;
            }
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_avda(uint32_t argc, uint8_t* argv[])
{
    // available in Slave mode only.
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(mAT_Ctx.ble_role)  // Master mode.
    {
        AT_LOG("\nNA in Master mode");
        goto ERR_exit;
    }

    if(argc == 1)
    {
        if('?' == argv[0][0])
        {
            AT_LOG("\n%s",g_adv_data);
        }
        else
        {
            if(strlen((char*)(argv[0])) >= AT_ADV_DATA_SIZE)   // input string is too long, cut partial of it.
            {
                strncpy((char*)g_adv_data, (char*)(argv[0]), AT_ADV_DATA_SIZE - 1);
                g_adv_data[AT_ADV_DATA_SIZE - 1] = '\0';
            }
            else
                strcpy((char*)g_adv_data, (char*)(argv[0]));

            // rebuild advertising data here.
            bleuart_gen_AdvData(mAT_Ctx.search_uuid, g_adv_data);
            update_AdvDataFromAT(false); // update adv data.
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_mode(uint32_t argc, uint8_t* argv[])
{
    uint8_t ret = 0;
    uint8_t orig_pw_mod = 0;
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    // available in Slave mode only.
    if(mAT_Ctx.ble_role)  // Master mode.
    {
        AT_LOG("\nNA in Master mode");
        goto ERR_exit;
    }

    if(argc == 1)
    {
        if('?' == argv[0][0])
        {
            switch(mAT_Ctx.pw_mod)
            {
            case 0:
                AT_LOG("\n0");
                break;

            case 1:
                AT_LOG("\n1");
                break;

            case 2:
                AT_LOG("\n2");
                break;

            default:
                goto ERR_exit;
            }
        }
        else
        {
            orig_pw_mod = mAT_Ctx.pw_mod;

            switch(argv[0][0])
            {
            case '0':
                mAT_Ctx.pw_mod = 0;
                break;

            case '1':
                mAT_Ctx.pw_mod = 1;
                break;

            case '2':
                mAT_Ctx.pw_mod = 2;
                break;

            default:
                goto ERR_exit;
            }

            ret = at_snv_write_flash(AT_SNV_ID_PW_MOD_OFFSET); // write data into flash.

            if(ret != SUCCESS)
            {
                mAT_Ctx.pw_mod = orig_pw_mod;
                goto ERR_exit;
            }

            if(mAT_Ctx.pw_mod != 0) // dma mode is not supported in lpm.
            {
                g_dma_flag = false;
            }
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_aint(uint32_t argc, uint8_t* argv[])
{
    uint8_t ret = 0;
    uint8_t i = 0;
    uint8_t len = 0;
    uint16_t aint_bak, aint_tmp = 0;
    uint8_t data_str[6] = {0}; // 32 - 6400 unit: 625 us.
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 1)
    {
        if('?' == argv[0][0])
            AT_LOG("\n%d",mAT_Ctx.adv_int);
        else
        {
            aint_bak = mAT_Ctx.adv_int;
            //check whether the data string is valid or not.
            len = strlen((char*)(argv[0]));

            if( len > 5 ) // too long data string.
                goto ERR_exit;

            for(i=0; i<len; i++)
            {
                if( IS_DIGIT(argv[0][i]) ) // is valid data number.
                    data_str[i] = argv[0][i];
                else
                    goto ERR_exit;
            }

            data_str[i+1] = '\0';
            aint_tmp = CLI_strtoi(data_str, strlen((char*)data_str),10);

            if((aint_tmp < AT_ADV_INTERVAL_MIN_625US) || (aint_tmp > AT_ADV_INTERVAL_MAX_625US)) // valid adv int scope(32 - 6400). mapping to : 20ms - 4s.
                goto ERR_exit;

            mAT_Ctx.adv_int = aint_tmp;
            ret = at_snv_write_flash(AT_SNV_ID_ADV_INT_OFFSET); // write data into flash.

            if(ret != SUCCESS)
            {
                mAT_Ctx.adv_int = aint_bak;
                goto ERR_exit;
            }

            // update adv interval. TBD
            GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, mAT_Ctx.adv_int );
            GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, mAT_Ctx.adv_int );
            GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, mAT_Ctx.adv_int );
            GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, mAT_Ctx.adv_int );
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_cint(uint32_t argc, uint8_t* argv[])
{
    uint8_t ret = 0;
    uint8_t i = 0;
    uint16_t cint_tmp = 0;
    uint16_t min_int, max_int, min_bak, max_bak;
    uint8_t data_str[5] = {0}; // 6 - 3199 (7.5ms - 4s) unit: 1.25 ms. conn_int
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 1)
    {
        if('?' == argv[0][0])
            AT_LOG("\n%d - %d", mAT_Ctx.conn_int[0], mAT_Ctx.conn_int[1]);
        else
        {
            goto ERR_exit;
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }
    else if(argc == 2)
    {
        min_bak = mAT_Ctx.conn_int[0];
        max_bak = mAT_Ctx.conn_int[1];

        //check whether the first data string is valid or not.
        for( i = 0; ((argv[0][i]) != '\0'); i++)
        {
            if( IS_DIGIT(argv[0][i]) ) // is valid data number.
                data_str[i] = argv[0][i];
            else
                goto ERR_exit;

            if( i >= 4 ) // too long data string.
                goto ERR_exit;
        }

        data_str[i+1] = '\0';
        cint_tmp = CLI_strtoi(data_str, strlen((char*)data_str),10);

        if((cint_tmp < AT_CONN_INTERVAL_MIN_1250US) || (cint_tmp > AT_CONN_INTERVAL_MAX_1250US)) // valid connect interval scope(6 - 3199). mapping to : 7.5ms - 4s.
            goto ERR_exit;

        min_int = cint_tmp;

        // re-initialize the data buffer.
        for(i=0; i<5 ; i++)
            data_str[i] = 0;

        cint_tmp = 0;

        //check whether the second data string is valid or not.
        for( i = 0; ((argv[1][i]) != '\0'); i++)
        {
            if( IS_DIGIT(argv[1][i]) ) // is valid data number.
                data_str[i] = argv[1][i];
            else
                goto ERR_exit;

            if( i >= 4 ) // too long data string.
                goto ERR_exit;
        }

        data_str[i+1] = '\0';
        cint_tmp = CLI_strtoi(data_str, strlen((char*)data_str),10);

        if((cint_tmp < AT_CONN_INTERVAL_MIN_1250US) || (cint_tmp > AT_CONN_INTERVAL_MAX_1250US)) // valid connect interval scope(6 - 3199). mapping to : 7.5ms - 4s.
            goto ERR_exit;

        if(cint_tmp < min_int) // make sure the second one is >= the first one.
            max_int = min_int;
        else
            max_int = cint_tmp;

        mAT_Ctx.conn_int[0] = min_int;
        mAT_Ctx.conn_int[1] = max_int;
        ret = at_snv_write_flash(AT_SNV_ID_CONN_INT_OFFSET); // write data into flash.

        if(ret != SUCCESS)
        {
            mAT_Ctx.conn_int[0] = min_bak;
            mAT_Ctx.conn_int[1] = max_bak;
            goto ERR_exit;
        }

        // rebuild advertising data here.
        bleuart_gen_scanRspData(mAT_Ctx.mod_name, mAT_Ctx.conn_int, mAT_Ctx.rfpw); //update scan response data.
        update_AdvDataFromAT(true); // update scan rsp parameters.
        // update parameters.
        GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &min_int );
        GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &max_int );
        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_ctout(uint32_t argc, uint8_t* argv[])
{
    uint8_t ret = 0;
    uint8_t i = 0;
    uint16_t ctout_bak, ctout_tmp = 0;
    uint8_t data_str[5] = {0}; // 10 - 3200 unit: 10 ms.
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 1)
    {
        if('?' == argv[0][0])
            AT_LOG("\n%d",mAT_Ctx.conn_timeout);
        else
        {
            ctout_bak = mAT_Ctx.conn_timeout;

            //check whether the data string is valid or not.
            for( i=0; ((argv[0][i]) != '\0'); i++)
            {
                if( IS_DIGIT(argv[0][i]) ) // is valid data number.
                    data_str[i] = argv[0][i];
                else
                    goto ERR_exit;

                if( i >= 4 ) // too long data string.
                    goto ERR_exit;
            }

            data_str[i+1] = '\0';
            ctout_tmp = CLI_strtoi(data_str, strlen((char*)data_str),10);

            if((ctout_tmp < AT_CONN_TIMEOUT_MIN_10MS) || (ctout_tmp > AT_CONN_TIMEOUT_MAX_10MS)) // valid adv int scope(10 - 3200). mapping to : 100ms - 30s.
                goto ERR_exit;

            mAT_Ctx.conn_timeout = ctout_tmp;
            ret = at_snv_write_flash(AT_SNV_ID_CONN_TOUT_OFFSET); // write data into flash.

            if(ret != SUCCESS)
            {
                mAT_Ctx.conn_timeout = ctout_bak;
                goto ERR_exit;
            }

            // update parameter.
            GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &mAT_Ctx.conn_timeout );
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_clear(uint32_t argc, uint8_t* argv[])
{
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);
    AT_LOG("\nOK\n");  // It's not supported at the moment.
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_led(uint32_t argc, uint8_t* argv[])
{
    uint8_t ret = 0;
    bool led_bak;
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 1)
    {
        if('?' == argv[0][0])
        {
            if(mAT_Ctx.led_mod) // true -- on.
                AT_LOG("\nOn");
            else // false -- slave
                AT_LOG("\nOff");
        }
        else
        {
            led_bak = mAT_Ctx.led_mod;

            //check whether the data string is valid or not.
            switch(argv[0][0])
            {
            case '1':
            case 't':
            case 'T':
                mAT_Ctx.led_mod = true;
                break;

            case '0':
            case 'f':
            case 'F':
                mAT_Ctx.led_mod = false;
                break;

            default:
                goto ERR_exit;
            }

            ret = at_snv_write_flash(AT_SNV_ID_LED_MOD_OFFSET); // write data into flash.

            if(ret != SUCCESS)
            {
                mAT_Ctx.led_mod = led_bak;
                goto ERR_exit;
            }

            if(mAT_Ctx.led_mod)
            {
                led_initial(LED_GPIO_PIN);
                led_set_status(LED_STATUS_ON);
                led_set_status(LED_STATUS_ON);
                led_set_status(LED_STATUS_OFF);
            }
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_luuid(uint32_t argc, uint8_t* argv[])
{
    uint8_t ret = 0;
    int16_t luuid_bak, luuid = 0;
    uint8_t i = 0, idx = 0;
    uint8_t data_str[5] = {0}; // 5 bytes should be enough for 16-bit data.
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 1)
    {
        if('?' == argv[0][0])
            AT_LOG("\n0x%x",mAT_Ctx.search_uuid);
        else
        {
            luuid_bak = mAT_Ctx.search_uuid;

            //check whether the data string is valid or not; skip '0x' if necessary.
            if( ('0' == argv[0][0]) && (('X' == argv[0][1]) || ('x' == argv[0][1])) )
                idx = 2;

            for(i=0; ((argv[0][i+idx]) != '\0'); i++)
            {
                if( (IS_DIGIT(argv[0][i+idx])) || (IS_UPPER(argv[0][i+idx])) || (IS_LOWER(argv[0][i+idx])) ) // is valid data number.
                    data_str[i] = argv[0][i+idx];
                else
                    goto ERR_exit;

                if( i >= 4 ) // too long data string.
                    goto ERR_exit;
            }

            data_str[i+1] = '\0';
            luuid = CLI_strtoi(data_str, strlen((char*)data_str),16);
            mAT_Ctx.search_uuid = luuid;
            ret = at_snv_write_flash(AT_SNV_ID_SEARCH_UUID_OFFSET); // write data into flash.

            if(ret != SUCCESS)
            {
                mAT_Ctx.search_uuid = luuid_bak;
                goto ERR_exit;
            }

            // rebuild advertising data here.
            bleuart_gen_AdvData(mAT_Ctx.search_uuid, g_adv_data);
            update_AdvDataFromAT(false); // update adv data.
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_suuid(uint32_t argc, uint8_t* argv[])
{
    uint8_t ret = 0;
    int16_t suuid_bak, suuid = 0;
    uint8_t i = 0, idx = 0;
    uint8_t data_str[5] = {0}; // 5 bytes should be enough for 16-bit data.
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 1)
    {
        if('?' == argv[0][0])
            AT_LOG("\n0x%x",mAT_Ctx.srv_uuid);
        else
        {
            suuid_bak = mAT_Ctx.srv_uuid;

            //check whether the data string is valid or not; skip '0x' if necessary.
            if( ('0' == argv[0][0]) && (('X' == argv[0][1]) || ('x' == argv[0][1])) )
                idx = 2;

            for(i=0; ((argv[0][i+idx]) != '\0'); i++)
            {
                if( (IS_DIGIT(argv[0][i+idx])) || (IS_UPPER(argv[0][i+idx])) || (IS_LOWER(argv[0][i+idx])) ) // is valid data number.
                    data_str[i] = argv[0][i+idx];
                else
                    goto ERR_exit;

                if( i >= 4 ) // too long data string.
                    goto ERR_exit;
            }

            data_str[i+1] = '\0';
            suuid = CLI_strtoi(data_str, strlen((char*)data_str),16);
            mAT_Ctx.srv_uuid = suuid;
            ret = at_snv_write_flash(AT_SNV_ID_SRV_UUID_OFFSET); // write data into flash.

            if(ret != SUCCESS)
            {
                mAT_Ctx.srv_uuid = suuid_bak;
                goto ERR_exit;
            }

            //update profile attribute table.
            update_Bleuart_ProfileAttrTbl( mAT_Ctx.srv_uuid, mAT_Ctx.pt_uuid);
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_tuuid(uint32_t argc, uint8_t* argv[])
{
    uint8_t ret = 0;
    int16_t tuuid_bak, tuuid = 0;
    uint8_t i = 0, idx = 0;
    uint8_t data_str[5] = {0}; // 5 bytes should be enough for 16-bit data.
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 1)
    {
        if('?' == argv[0][0])
            AT_LOG("\n0x%x",mAT_Ctx.pt_uuid);
        else
        {
            tuuid_bak = mAT_Ctx.pt_uuid;

            //check whether the data string is valid or not; skip '0x' if necessary.
            if( ('0' == argv[0][0]) && (('X' == argv[0][1]) || ('x' == argv[0][1])) )
                idx = 2;

            for(i=0; ((argv[0][i+idx]) != '\0'); i++)
            {
                if( (IS_DIGIT(argv[0][i+idx])) || (IS_UPPER(argv[0][i+idx])) || (IS_LOWER(argv[0][i+idx])) ) // is valid data number.
                    data_str[i] = argv[0][i+idx];
                else
                    goto ERR_exit;

                if( i >= 4 ) // too long data string.
                    goto ERR_exit;
            }

            data_str[i+1] = '\0';
            tuuid = CLI_strtoi(data_str, strlen((char*)data_str),16);
            mAT_Ctx.pt_uuid = tuuid;
            ret = at_snv_write_flash(AT_SNV_ID_PT_UUID_OFFSET); // write data into flash.

            if(ret != SUCCESS)
            {
                mAT_Ctx.pt_uuid = tuuid_bak;
                goto ERR_exit;
            }

            //update profile attribute table.
            update_Bleuart_ProfileAttrTbl( mAT_Ctx.srv_uuid, mAT_Ctx.pt_uuid);
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_aust(uint32_t argc, uint8_t* argv[])
{
    uint8_t ret = 0;
    int32_t aust_bak, aust = 0;  // auto sleep timeout value.
    uint8_t i = 0;
    uint8_t data_str[9] = {0}; // 9 bytes should be enough for 32-bit data.
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 1)
    {
        if('?' == argv[0][0])
            AT_LOG("\n%d",mAT_Ctx.auto_slp_time);
        else
        {
            aust_bak = mAT_Ctx.auto_slp_time;

            //check whether the data string is valid or not.
            for( i=0; ((argv[0][i]) != '\0'); i++)
            {
                if( IS_DIGIT(argv[0][i]) ) // is valid data number.
                    data_str[i] = argv[0][i];
                else
                    goto ERR_exit;

                if( i >= 8 ) // too long data string.
                    goto ERR_exit;
            }

            data_str[i+1] = '\0';
            aust = CLI_strtoi(data_str, strlen((char*)data_str),10);
            mAT_Ctx.auto_slp_time = aust;
            ret = at_snv_write_flash(AT_SNV_ID_AUTO_SLP_TIME_OFFSET); // write data into flash.

            if(ret != SUCCESS)
            {
                mAT_Ctx.auto_slp_time = aust_bak;
                goto ERR_exit;
            }
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_div(uint32_t argc, uint8_t* argv[])
{
    //uint8_t ret = 0;
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 1)
    {
        if('?' == argv[0][0])
        {
            switch(at_dma_get_div())
            {
            case AT_BUF_DIV_256:
                AT_LOG("\n256");
                break;

            case AT_BUF_DIV_128:
                AT_LOG("\n128");
                break;

            case AT_BUF_DIV_64:
                AT_LOG("\n64");
                break;

            case AT_BUF_DIV_32:
                AT_LOG("\n32");
                break;

            case AT_BUF_DIV_16:
                AT_LOG("\n16");
                break;

            default:
                goto ERR_exit;
            }
        }
        else
        {
            if(strcmp((char*)(argv[0]), "256") == 0)
                at_dma_set_div(AT_BUF_DIV_256);
            else if(strcmp((char*)(argv[0]), "128") == 0)
                at_dma_set_div(AT_BUF_DIV_128);
            else if(strcmp((char*)(argv[0]), "64") == 0)
                at_dma_set_div(AT_BUF_DIV_64);
            else if(strcmp((char*)(argv[0]), "32") == 0)
                at_dma_set_div(AT_BUF_DIV_32);
            else if(strcmp((char*)(argv[0]), "16") == 0)
                at_dma_set_div(AT_BUF_DIV_16);
            else
                goto ERR_exit;
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_pcnt(uint32_t argc, uint8_t* argv[])
{
    //uint8_t ret = 0;
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 1)
    {
        if('?' == argv[0][0])
        {
            AT_LOG("\n%d",at_get_count());
        }
        else
        {
            if(strcmp((char*)(argv[0]), "160") == 0)
                at_dma_set_buf_len(160);
            else if(strcmp((char*)(argv[0]), "128") == 0)
                at_dma_set_buf_len(128);
            else
                at_dma_set_buf_len(240);
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_dma(uint32_t argc, uint8_t* argv[])
{
    //uint8_t ret = 0;
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 1)
    {
        if('?' == argv[0][0])
        {
            if(g_dma_flag)
            {
                AT_LOG("\ntrue");
            }
            else
            {
                AT_LOG("\nfalse");
            }
        }
        else
        {
            switch(argv[0][0])
            {
            case 't':
            case 'T':
            case '1':
                if(mAT_Ctx.pw_mod == 0)
                {
                    g_dma_flag = true;
                    break;
                }
                else
                {
                    g_dma_flag = false;
                    goto ERR_exit;
                }

            case 'f':
            case 'F':
            case '0':
                g_dma_flag = false;
                break;

            default:
                g_dma_flag = false;
                break;
            }
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

ERR_exit:
    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

uint16_t at_rxpath(uint32_t argc, uint8_t* argv[])
{
    //uint8_t ret = 0;
    osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);

    if(argc == 1)
    {
        if('?' == argv[0][0])
        {
            if(g_rxpath_flag)
            {
                AT_LOG("\ntrue");
            }
            else
            {
                AT_LOG("\nfalse");
            }
        }
        else
        {
            switch(argv[0][0])
            {
            case 't':
            case 'T':
            case '1':
                g_rxpath_flag = true;
                break;

            case 'f':
            case 'F':
            case '0':
                g_rxpath_flag = false;
                break;

            default:
                g_rxpath_flag = false;
                break;
            }
        }

        AT_LOG("\nOK\n");
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
        return 0;
    }

    AT_LOG("\nERR\n");
    osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, mAT_Ctx.auto_slp_time * 1000 );
    return 0;
}

// following functions are not partial of AT cmd set.
void    at_update_bd_addr(void)
{
    // ll.c: ownPublicAddr   0x1fff1231  data  6
    // This address need to be updated in case it's changed.
    //volatile uint8_t * p_ownPublicAddr = (volatile uint8_t*)0x1fff1231;
    volatile uint8_t* p_ownPublicAddr = (volatile uint8_t*)0x1fff0965;  // It's 0x1fff0965 for 6222.
    *(p_ownPublicAddr++) = mAT_Ctx.mac_addr[5];
    *(p_ownPublicAddr++) = mAT_Ctx.mac_addr[4];
    *(p_ownPublicAddr++) = mAT_Ctx.mac_addr[3];
    *(p_ownPublicAddr++) = mAT_Ctx.mac_addr[2];
    *(p_ownPublicAddr++) = mAT_Ctx.mac_addr[1];
    *(p_ownPublicAddr++) = mAT_Ctx.mac_addr[0];
}

uint32_t at_get_baudrate(void)
{
    #ifdef BLEUART_DEDICATE
    return mAT_Ctx.baudrate[0];
    #else
    return mAT_Ctx.baudrate;
    #endif
}

#ifdef BLEUART_DEDICATE
//UART_PARITY_e at_get_uart_parity(void)
uint32_t at_get_uart_parity(void)
{
    switch(mAT_Ctx.baudrate[1])
    {
    case 0:
        return (uint32_t)UART_PARITY_N;

    case 1:
        return (uint32_t)UART_PARITY_O;

    case 2:
        return (uint32_t)UART_PARITY_E;

    default:
        return (uint32_t)UART_PARITY_N;
    }
}
#endif

uint8_t  at_get_pw_mode(void)
{
    return mAT_Ctx.pw_mod;
}

bool  at_get_led_mode(void)
{
    return mAT_Ctx.led_mod;
}

bool  at_get_dma_flag(void)
{
    return g_dma_flag;
}

bool  at_get_rxpath_flag(void)
{
    return g_rxpath_flag;
}

uint32_t at_get_auto_slp_time(void)
{
    return mAT_Ctx.auto_slp_time;
}

uint8_t  at_get_cmdlen(void)
{
    return cmdlen;
}

void at_initialize_fs()
{
    int ret;

    if(hal_fs_initialized() == FALSE)
    {
        ret = hal_fs_init(0x11034000,4);

        if(PPlus_SUCCESS != ret)
            LOG("error:%d\n",ret);
    }
}

void at_get_ctx(AT_ctx_t* m_at_ctx)
{
    uint8_t i;
    m_at_ctx->connectable   = mAT_Ctx.connectable;
    m_at_ctx->led_mod       = mAT_Ctx.led_mod;
    m_at_ctx->ble_role      = mAT_Ctx.ble_role;
    strcpy((char*)(m_at_ctx->mod_name), (char*)(mAT_Ctx.mod_name));

    for(i = 0; i < 6; i++ )
        m_at_ctx->mac_addr[i] = mAT_Ctx.mac_addr[i];

    strcpy((char*)(m_at_ctx->mod_pin), (char*)(mAT_Ctx.mod_pin));
    m_at_ctx->rfpw          = mAT_Ctx.rfpw;
    m_at_ctx->pw_mod        = mAT_Ctx.pw_mod;
    m_at_ctx->adv_int       = mAT_Ctx.adv_int;
    m_at_ctx->conn_int[0]   = mAT_Ctx.conn_int[0];
    m_at_ctx->conn_int[1]   = mAT_Ctx.conn_int[1];
    m_at_ctx->conn_timeout  = mAT_Ctx.conn_timeout;
    #ifdef BLEUART_DEDICATE
    m_at_ctx->baudrate[0]   = mAT_Ctx.baudrate[0];
    m_at_ctx->baudrate[1]   = mAT_Ctx.baudrate[1];
    #else
    m_at_ctx->baudrate      = mAT_Ctx.baudrate;
    #endif
    m_at_ctx->search_uuid   = mAT_Ctx.search_uuid;
    m_at_ctx->srv_uuid      = mAT_Ctx.srv_uuid;
    m_at_ctx->pt_uuid       = mAT_Ctx.pt_uuid;
    m_at_ctx->auto_slp_time = mAT_Ctx.auto_slp_time;
}

void at_get_ctx_def(AT_ctx_t* m_at_ctx)
{
    uint8_t i;
    m_at_ctx->connectable   = mAT_Ctx_def.connectable;
    m_at_ctx->led_mod       = mAT_Ctx_def.led_mod;
    m_at_ctx->ble_role      = mAT_Ctx_def.ble_role;
    strcpy((char*)(m_at_ctx->mod_name), (char*)(mAT_Ctx_def.mod_name));

    for(i = 0; i < 6; i++ )
        m_at_ctx->mac_addr[i] = mAT_Ctx_def.mac_addr[i];

    strcpy((char*)(m_at_ctx->mod_pin), (char*)(mAT_Ctx_def.mod_pin));
    m_at_ctx->rfpw          = mAT_Ctx_def.rfpw;
    m_at_ctx->pw_mod        = mAT_Ctx_def.pw_mod;
    m_at_ctx->adv_int       = mAT_Ctx_def.adv_int;
    m_at_ctx->conn_int[0]   = mAT_Ctx_def.conn_int[0];
    m_at_ctx->conn_int[1]   = mAT_Ctx_def.conn_int[1];
    m_at_ctx->conn_timeout  = mAT_Ctx_def.conn_timeout;
    #ifdef BLEUART_DEDICATE
    m_at_ctx->baudrate[0]   = mAT_Ctx_def.baudrate[0];
    m_at_ctx->baudrate[1]   = mAT_Ctx_def.baudrate[1];
    #else
    m_at_ctx->baudrate      = mAT_Ctx_def.baudrate;
    #endif
    m_at_ctx->search_uuid   = mAT_Ctx_def.search_uuid;
    m_at_ctx->srv_uuid      = mAT_Ctx_def.srv_uuid;
    m_at_ctx->pt_uuid       = mAT_Ctx_def.pt_uuid;
    m_at_ctx->auto_slp_time = mAT_Ctx_def.auto_slp_time;
}

uint8_t at_snv_read_flash(void)
{
    uint8_t  tmp8 = 0, i = 0, ret = 1;
    uint8_t  tmp8arr[AT_MODULE_NAME_SIZE] = {0};
    uint32_t tmp32 = 0;
    uint16_t tmp16 = 0;
    uint16_t tmp16arr[2] = {0};
    #ifdef BLEUART_DEDICATE
    uint32_t tmp32arr[2] = {0};
    #endif
    // read .connectable from flash.
    ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_CONNECTABLE_OFFSET), 1, &tmp8);

    if(ret == SUCCESS)
    {
        if(0 == tmp8)
            mAT_Ctx.connectable = false;
        else if(1 == tmp8)
            mAT_Ctx.connectable = true;
        else
        {
            AT_LOG("\nR_F_D_Err\n");
            return READ_FLASH_DATA_ERROR;
        }
    }
    else
    {
        AT_LOG("\nR_F_Err\n");
        return READ_FLASH_ERROR;
    }

    // read .led_mod from flash.
    tmp8 = 0;
    ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_LED_MOD_OFFSET), 1, &tmp8);

    if(ret == SUCCESS)
    {
        if(0 == tmp8)
            mAT_Ctx.led_mod = false;
        else if(1 == tmp8)
            mAT_Ctx.led_mod = true;
        else
        {
            AT_LOG("\nR_F_D_Err\n");
            return READ_FLASH_DATA_ERROR;
        }
    }
    else
    {
        AT_LOG("\nR_F_Err\n");
        return READ_FLASH_ERROR;
    }

    // read .ble_role from flash.
    tmp8 = 0;
    ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_BLE_ROLE_OFFSET), 1, &tmp8);

    if(ret == SUCCESS)
    {
        if(0 == tmp8)
            mAT_Ctx.ble_role = false;
        else if(1 == tmp8)
            mAT_Ctx.ble_role = true;
        else
        {
            AT_LOG("\nR_F_D_Err\n");
            return READ_FLASH_DATA_ERROR;
        }
    }
    else
    {
        AT_LOG("\nR_F_Err\n");
        return READ_FLASH_ERROR;
    }

    // read .mod_name from flash.
    ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_MOD_NAME_OFFSET), AT_MODULE_NAME_SIZE, tmp8arr);

    if(ret == SUCCESS)
    {
        strcpy((char*)(mAT_Ctx.mod_name), (char*)tmp8arr);

        for(i=0; i < AT_MODULE_NAME_SIZE; i++) // clear the data buffer.
            tmp8arr[i] = 0;
    }
    else
    {
        AT_LOG("\nR_F_Err\n");
        return READ_FLASH_ERROR;
    }

    // read .mac_addr from flash.
    ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_MAC_ADDR_OFFSET), 6, tmp8arr);

    if(ret == SUCCESS)
    {
        //strncpy((char *)(mAT_Ctx.mac_addr), (char *)tmp8arr, 6);
        for(i=0 ; i<6 ; i++)  // copy the data buffer.
            mAT_Ctx.mac_addr[i] = tmp8arr[i];

        for(i=0; i < 6; i++) // clear the data buffer.
            tmp8arr[i] = 0;
    }
    else
    {
        AT_LOG("\nR_F_Err\n");
        return READ_FLASH_ERROR;
    }

    // read .mod_pin from flash.
    ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_MOD_PIN_OFFSET), AT_MODULE_PIN_SIZE, tmp8arr);

    if(ret == SUCCESS)
    {
        strcpy((char*)(mAT_Ctx.mod_pin), (char*)tmp8arr);

        for(i=0; i < AT_MODULE_PIN_SIZE; i++) // clear the data buffer.
            tmp8arr[i] = 0;
    }
    else
    {
        AT_LOG("\nR_F_Err\n");
        return READ_FLASH_ERROR;
    }

    // read .rfpw from flash.
    tmp8 = 0;
    ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_RFPW_OFFSET), 1, &tmp8);

    if(ret == SUCCESS)
    {
        mAT_Ctx.rfpw = tmp8;
    }
    else
    {
        AT_LOG("\nR_F_Err\n");
        return READ_FLASH_ERROR;
    }

    // read .pw_mod from flash.
    tmp8 = 0;
    ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_PW_MOD_OFFSET), 1, &tmp8);

    if(ret == SUCCESS)
    {
        mAT_Ctx.pw_mod = tmp8;
    }
    else
    {
        AT_LOG("\nR_F_Err\n");
        return READ_FLASH_ERROR;
    }

    // read .adv_int from flash.
    tmp16 = 0;
    ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_ADV_INT_OFFSET), 2, &tmp16);

    if(ret == SUCCESS)
    {
        mAT_Ctx.adv_int = tmp16;
    }
    else
    {
        AT_LOG("\nR_F_Err\n");
        return READ_FLASH_ERROR;
    }

    // read .conn_int from flash.
    tmp16 = 0;
    ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_CONN_INT_OFFSET), 4, tmp16arr);

    if(ret == SUCCESS)
    {
        mAT_Ctx.conn_int[0] = tmp16arr[0];
        mAT_Ctx.conn_int[1] = tmp16arr[1];
    }
    else
    {
        AT_LOG("\nR_F_Err\n");
        return READ_FLASH_ERROR;
    }

    // read .conn_timeout from flash.
    tmp16 = 0;
    ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_CONN_TOUT_OFFSET), 2, &tmp16);

    if(ret == SUCCESS)
    {
        mAT_Ctx.conn_timeout = tmp16;
    }
    else
    {
        AT_LOG("\nR_F_Err\n");
        return READ_FLASH_ERROR;
    }

    // read .search_uuid from flash.
    tmp16 = 0;
    ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_SEARCH_UUID_OFFSET), 2, &tmp16);

    if(ret == SUCCESS)
    {
        mAT_Ctx.search_uuid = tmp16;
    }
    else
    {
        AT_LOG("\nR_F_Err\n");
        return READ_FLASH_ERROR;
    }

    // read .srv_uuid from flash.
    tmp16 = 0;
    ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_SRV_UUID_OFFSET), 2, &tmp16);

    if(ret == SUCCESS)
    {
        mAT_Ctx.srv_uuid = tmp16;
    }
    else
    {
        AT_LOG("\nR_F_Err\n");
        return READ_FLASH_ERROR;
    }

    // read .pt_uuid from flash.
    tmp16 = 0;
    ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_PT_UUID_OFFSET), 2, &tmp16);

    if(ret == SUCCESS)
    {
        mAT_Ctx.pt_uuid = tmp16;
    }
    else
    {
        AT_LOG("\nR_F_Err\n");
        return READ_FLASH_ERROR;
    }

    #ifdef BLEUART_DEDICATE
    // read .baudrate from flash.
    ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_BAUDRATE_OFFSET), 8, tmp32arr);

    if(ret == SUCCESS)
    {
        mAT_Ctx.baudrate[0] = tmp32arr[0];
        mAT_Ctx.baudrate[1] = tmp32arr[1];
    }
    else
    {
        AT_LOG("\nR_F_Err\n");
        return READ_FLASH_ERROR;
    }

    #else
    // read .baudrate from flash.
    tmp32 = 0;
    ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_BAUDRATE_OFFSET), 4, &tmp32);

    if(ret == SUCCESS)
    {
        mAT_Ctx.baudrate = tmp32;
    }
    else
    {
        AT_LOG("\nR_F_Err\n");
        return READ_FLASH_ERROR;
    }

    #endif
    // read .auto_slp_time from flash.
    tmp32 = 0;
    ret = osal_snv_read(AT_SNV_ID_INDEX(AT_SNV_ID_AUTO_SLP_TIME_OFFSET), 4, &tmp32);

    if(ret == SUCCESS)
    {
        mAT_Ctx.auto_slp_time = tmp32;
    }
    else
    {
        AT_LOG("\nR_F_Err\n");
        return READ_FLASH_ERROR;
    }

    return SUCCESS;
}


uint8_t at_snv_write_flash(uint16_t idx)
{
    uint8_t tmp8 = 0, i = 0, ret = 1;
    uint8_t tmp8arr[AT_MODULE_NAME_SIZE] = {0};
    uint32_t tmp32 = 0;
    uint16_t tmp16 = 0;
    uint16_t tmp16arr[2] = {0};
    #ifdef BLEUART_DEDICATE
    uint32_t tmp32arr[2] = {0};
    #endif

    switch(idx)
    {
    case AT_SNV_ID_CONNECTABLE_OFFSET: // write .connectable into flash.
        if(mAT_Ctx.connectable)
            tmp8 = 1;
        else
            tmp8 = 0;

        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), 1, &tmp8);
        break;

    case AT_SNV_ID_LED_MOD_OFFSET: // write .led_mod into flash.
        if(mAT_Ctx.led_mod)
            tmp8 = 1;
        else
            tmp8 = 0;

        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), 1, &tmp8);
        break;

    case AT_SNV_ID_BLE_ROLE_OFFSET: // write .ble_role into flash.
        if(mAT_Ctx.ble_role)
            tmp8 = 1;
        else
            tmp8 = 0;

        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), 1, &tmp8);
        break;

    case AT_SNV_ID_MOD_NAME_OFFSET: // write .mod_name into flash.
        strcpy((char*)tmp8arr, (char*)(mAT_Ctx.mod_name));
        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), AT_MODULE_NAME_SIZE, tmp8arr);

        for(i=0; i < AT_MODULE_NAME_SIZE; i++) // clear the data buffer.
            tmp8arr[i] = 0;

        break;

    case AT_SNV_ID_MAC_ADDR_OFFSET: // write .mac_addr into flash.

        //strncpy((char *)tmp8arr, (char *)(mAT_Ctx.mac_addr), 6);
        for(i=0 ; i<6 ; i++)  // clear the data buffer.
            tmp8arr[i] = mAT_Ctx.mac_addr[i];

        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), 6, tmp8arr);

        for(i=0 ; i<6 ; i++)  // clear the data buffer.
            tmp8arr[i] = 0;

        break;

    case AT_SNV_ID_MOD_PIN_OFFSET: // write .mod_pin into flash.
        strcpy((char*)tmp8arr, (char*)(mAT_Ctx.mod_pin));
        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), AT_MODULE_PIN_SIZE, tmp8arr);

        for(i=0 ; i<AT_MODULE_PIN_SIZE ; i++)  // clear the data buffer.
            tmp8arr[i] = 0;

        break;

    case AT_SNV_ID_RFPW_OFFSET: // write .rfpw into flash.
        tmp8 = mAT_Ctx.rfpw;
        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), 1, &tmp8);
        break;

    case AT_SNV_ID_PW_MOD_OFFSET: // write .pw_mod into flash.
        tmp8 = mAT_Ctx.pw_mod;
        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), 1, &tmp8);
        break;

    case AT_SNV_ID_ADV_INT_OFFSET: // write .adv_int into flash.
        tmp16 = mAT_Ctx.adv_int;
        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), 2, &tmp16);
        break;

    case AT_SNV_ID_CONN_INT_OFFSET: // write .conn_int into flash.
        tmp16arr[0] = mAT_Ctx.conn_int[0];
        tmp16arr[1] = mAT_Ctx.conn_int[1];
        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), 4, tmp16arr);
        break;

    case AT_SNV_ID_CONN_TOUT_OFFSET: // write .conn_timeout into flash.
        tmp16 = mAT_Ctx.conn_timeout;
        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), 2, &tmp16);
        break;

    case AT_SNV_ID_SEARCH_UUID_OFFSET: // write .search_uuid into flash.
        tmp16 = mAT_Ctx.search_uuid;
        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), 2, &tmp16);
        break;

    case AT_SNV_ID_SRV_UUID_OFFSET: // write .srv_uuid into flash.
        tmp16 = mAT_Ctx.srv_uuid;
        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), 2, &tmp16);
        break;

    case AT_SNV_ID_PT_UUID_OFFSET: // write .pt_uuid into flash.
        tmp16 = mAT_Ctx.pt_uuid;
        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), 2, &tmp16);
        break;
        #ifdef BLEUART_DEDICATE

    case AT_SNV_ID_BAUDRATE_OFFSET: // write .baudrate into flash.
        tmp32arr[0] = mAT_Ctx.baudrate[0];
        tmp32arr[1] = mAT_Ctx.baudrate[1];
        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), 8, tmp32arr);
        break;
        #else

    case AT_SNV_ID_BAUDRATE_OFFSET: // write .baudrate into flash.
        tmp32 = mAT_Ctx.baudrate;
        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), 4, &tmp32);
        break;
        #endif

    case AT_SNV_ID_AUTO_SLP_TIME_OFFSET: // write .auto_slp_time into flash.
        tmp32 = mAT_Ctx.auto_slp_time;
        ret = osal_snv_write(AT_SNV_ID_INDEX(idx), 4, &tmp32);
        break;

    default:
        break;
    }

    if(ret == SUCCESS)
        return ret;
    else
    {
        AT_LOG("\nW_F_Err\n");
        return WRITE_FLASH_ERROR;
    }
}

void at_snv_write_flash_all(void)
{
    uint8_t i = 0;

    for(i = 0; i <= AT_SNV_ID_MAX_OFFSET; i++)
        at_snv_write_flash(i);
}


__ATTR_SECTION_SRAM__ static void ProcessUartData(uart_Evt_t* evt)
{
    switch(evt->type)
    {
    case  UART_EVT_TYPE_RX_DATA:
    case  UART_EVT_TYPE_RX_DATA_TO:
    {
        if((cmdlen + evt->len) > (AT_CMD_LENGTH_MAX - 1))  // fix the mAT_Ctx data would be overrided issue.
        {
            //osal_memset((void *)(cmdstr[0]), 0, AT_CMD_LENGTH_MAX);
            cmdlen = 0;
            memset(cmdstr, 0, AT_CMD_LENGTH_MAX); // clean the cdmstr with fixed len = 64.
        }
        else
        {
            if((0x01 == evt->len) && (0x00 == evt->data[0]))
                break;

            osal_memcpy((cmdstr + cmdlen), evt->data, evt->len);
            cmdlen += evt->len;
            osal_set_event( bleuart_TaskID, BUP_OSAL_EVT_AT_UART_RX_CMD );
        }
    }
    break;

    case  UART_EVT_TYPE_TX_COMPLETED: // should not be here in AT mode.
        break;

    default:
        break;
    }
}

void at_uart_init(void)
{
    uart_Cfg_t cfg =
    {
        .tx_pin = P9,
        .rx_pin = P10,
        .rts_pin = GPIO_DUMMY,
        .cts_pin = GPIO_DUMMY,
        .baudrate = 115200,
        .use_fifo = TRUE,
        .hw_fwctrl = FALSE,
        .use_tx_buf = FALSE,
        .parity     = FALSE,
        .evt_handler = ProcessUartData,
    };
    #ifdef BLEUART_DEDICATE

    if(mAT_Ctx.baudrate[0] == 0x0)   //make sure we have default baudrate at the first boot.
    {
        mAT_Ctx.baudrate[0] = 9600;
        mAT_Ctx.baudrate[1] = UART_PARITY_N;
    }

    #else

    if(mAT_Ctx.baudrate == 0x0)   //make sure we have default baudrate at the first boot.
    {
        mAT_Ctx.baudrate = 115200;
    }

    #endif
    osal_memset((void*)(cmdstr[0]), 0, AT_CMD_LENGTH_MAX);
    cmdlen = 0;
    #ifdef BLEUART_DEDICATE
    cfg.baudrate = mAT_Ctx.baudrate[0];
    #else
    cfg.baudrate = mAT_Ctx.baudrate;
    #endif
    hal_uart_init(cfg, (UART_INDEX_e)get_uart_idx());
    #ifdef BLEUART_DEDICATE

    switch(mAT_Ctx.baudrate[1])
    {
    case 1:
        AP_UART0->LCR = 0x0b; //8bit, 1 stop odd parity
        break;

    case 2:
        AP_UART0->LCR = 0x1b; //8bit, 1 stop even parity
        break;

    default:
        break;
    }

    #endif
}


void at_Init()
{
    hal_uart_deinit((UART_INDEX_e)get_uart_idx());
    at_uart_init();
    set_uart_at_mod(true);
    AT_LOG("AT Mod\n");
}
