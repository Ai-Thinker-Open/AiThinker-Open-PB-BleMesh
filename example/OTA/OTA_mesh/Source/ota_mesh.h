
#ifndef _OTA_MESH_HEAD
#define _OTA_MESH_HEAD


void otaMesh_BootMode(void);

extern void otaMesh_Init( uint8 task_id );

extern uint16 otaMesh_ProcessEvent( uint8 task_id, uint16 events );

#endif

