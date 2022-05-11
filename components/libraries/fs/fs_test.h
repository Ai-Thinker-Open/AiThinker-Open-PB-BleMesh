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


#ifndef __FS_TEST_H__
#define __FS_TEST_H__

//be care:when assign the fs size,please refer to Flash_Distribution
#define FS_OFFSET_ADDRESS         0x1103c000 //for 256KB Flash
//#define FS_OFFSET_ADDRESS         0x11034000 //for 512KB Flash
#define FS_SECTOR_NUM             2



#define FS_EXAMPLE       0x01
#define FS_XIP_TEST      0x02
//#define FS_MODULE_TEST   0x04
//#define FS_TIMING_TEST   0x08

#define FS_TEST_TYPE     FS_EXAMPLE

#if (FS_TEST_TYPE == FS_EXAMPLE)
    void fs_example(void);
#elif (FS_TEST_TYPE == FS_XIP_TEST)
    void fs_xip_test(void);
#elif (FS_TEST_TYPE == FS_MODULE_TEST)
    void ftcase_simple_write_test(void);
    void ftcase_write_del_test(void);
    void ftcase_write_del_and_ble_enable_test(void);
#elif (FS_TEST_TYPE == FS_TIMING_TEST)
    void fs_timing_test(void);
#else
    #error please check your config parameter
#endif


#endif
