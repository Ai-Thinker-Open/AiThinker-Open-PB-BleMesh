#include "OSAL.h"
#include "OSAL_Tasks.h"

/* LL */
#include "ll.h"

/* HCI */
#include "hci_tl.h"

/* L2CAP */
#include "l2cap.h"

/* gap */
#include "gap.h"
#include "gapgattserver.h"
#include "gapbondmgr.h"

/* GATT */
#include "gatt.h"

#include "gattservapp.h"

/* Profiles */
#include "central.h"
#include "peripheral.h"

/* Application */
#include "ota_mesh.h"

#include "bcomdef.h"
#include "version.h"
#include "ota_mesh_master.h"
#include "ota_mesh_slave.h"
#include "string.h"

enum{
  OTAMESH_MODE_OTA_APPLICATION= 0,
  //OTAMESH_MODE_OTA_FCT,//          1
  OTAMESH_MODE_OTA = 2,//              2
  OTAMESH_MODE_MASTER = 4//         3
};

#define OTA_MODE_SELECT_REG 0x4000f034

static bool s_is_ota_master = FALSE;



#define OSAL_TASK_MAX 12
const pTaskEventHandlerFn tasksArr[OSAL_TASK_MAX] = {0};
const uint8 tasksCnt = 0;
uint16* tasksEvents = NULL;

pTaskEventHandlerFn masterTasksArr[] =
{
  LL_ProcessEvent,
  HCI_ProcessEvent,
  L2CAP_ProcessEvent,
  GAP_ProcessEvent,
  GATT_ProcessEvent,
  SM_ProcessEvent,
  GAPCentralRole_ProcessEvent,
  GAPBondMgr_ProcessEvent,
  GATTServApp_ProcessEvent,
  otaMM_ProcessEvent
};


pTaskEventHandlerFn slaveTasksArr[] =
{
  LL_ProcessEvent,
  HCI_ProcessEvent,
  L2CAP_ProcessEvent,
  GAP_ProcessEvent,
  GATT_ProcessEvent,
  SM_ProcessEvent,
  GAPRole_ProcessEvent,
  GATTServApp_ProcessEvent,
  otaMS_ProcessEvent
};


static void masterInitTasks( void )
{
  uint8 taskID = 0;
  uint8_t* p = (uint8_t*)&(tasksArr[0]);
  uint8_t size = sizeof( masterTasksArr ) / sizeof( pTaskEventHandlerFn);
  memcpy(p , masterTasksArr, sizeof(masterTasksArr));
	p = (uint8_t*)&tasksCnt;
	*p = size;
  tasksEvents = (uint16 *)osal_mem_alloc( sizeof( uint16 ) * size);
  osal_memset( tasksEvents, 0, (sizeof( uint16 ) * size));

  LL_Init( taskID++ );
  HCI_Init( taskID++ );
  L2CAP_Init( taskID++ );
  GAP_Init( taskID++ );
  GATT_Init( taskID++ );
  SM_Init( taskID++ );
  GAPCentralRole_Init( taskID++ );
  GAPBondMgr_Init( taskID++ );
  GATTServApp_Init( taskID++ );
  otaMM_Init( taskID );
}


static void slaveInitTasks( void )
{
  uint8 taskID = 0;
  uint8_t* p = (uint8_t*)&(tasksArr[0]);
  uint8_t size = sizeof( slaveTasksArr ) / sizeof( pTaskEventHandlerFn);
  memcpy(p , slaveTasksArr, sizeof(slaveTasksArr));
	p = (uint8_t*)&tasksCnt;
	*p = size;
  tasksEvents = (uint16 *)osal_mem_alloc( sizeof( uint16 ) * size);
  osal_memset( tasksEvents, 0, (sizeof( uint16 ) * size));

  LL_Init( taskID++ );
  HCI_Init( taskID++ );
  L2CAP_Init( taskID++ );
  GAP_Init( taskID++ );
  GATT_Init( taskID++ );
  SM_Init( taskID++ );
  GAPRole_Init( taskID++ );
  GATTServApp_Init( taskID++ );
  otaMS_Init( taskID );
}





void otaMesh_BootMode(void)
{
  uint32_t ota_mode = read_reg(OTA_MODE_SELECT_REG) & 0xf;
  uint32_t reg = ((SDK_VER_MAJOR &0xf) << 4) | ((SDK_VER_MINOR &0xf)<< 8) | ((SDK_VER_REVISION &0xff)<<12);
  #ifdef SDK_VER_TEST_BUILD
    reg |= (((SDK_VER_TEST_BUILD - 'a' + 1)&0xf) << 20);
  #endif

  write_reg(OTA_MODE_SELECT_REG,reg);

  hal_gpio_pull_set(P14, STRONG_PULL_UP);

  
  if(hal_gpio_read(P14)){
    ota_mode = OTAMESH_MODE_MASTER;
  }
  
  
  switch(ota_mode){
  case OTAMESH_MODE_OTA_APPLICATION:
    run_application();
    break;
  case OTAMESH_MODE_MASTER:
    s_is_ota_master = TRUE;
    break;
  default:
    break;
  }
}




void osalInitTasks( void )
{
  if(s_is_ota_master)
    masterInitTasks();
  else
    slaveInitTasks();
}


