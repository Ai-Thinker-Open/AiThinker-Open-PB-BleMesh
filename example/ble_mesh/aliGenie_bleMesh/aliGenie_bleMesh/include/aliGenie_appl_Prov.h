
#include  "MS_error.h"
#include  "EM_os.h"
#include  "MS_prov_api.h"
#ifndef _ALIGENIE_APPL_PROV_H_
#define _ALIGENIE_APPL_PROV_H_

/* Provisionee */
#define UI_PROV_OUTPUT_OOB_ACTIONS \
    (PROV_MASK_OOOB_ACTION_BLINK | PROV_MASK_OOOB_ACTION_BEEP | \
     PROV_MASK_OOOB_ACTION_VIBRATE | PROV_MASK_OOOB_ACTION_NUMERIC | \
     PROV_MASK_OOOB_ACTION_ALPHANUMERIC)

/** Output OOB Maximum size supported */
#define UI_PROV_OUTPUT_OOB_SIZE               0x08

/** Input OOB Actions supported */
#define UI_PROV_INPUT_OOB_ACTIONS \
    (PROV_MASK_IOOB_ACTION_PUSH | PROV_MASK_IOOB_ACTION_TWIST | \
     PROV_MASK_IOOB_ACTION_NUMERIC | PROV_MASK_IOOB_ACTION_ALPHANUMERIC)

/** Input OOB Maximum size supported */
#define UI_PROV_INPUT_OOB_SIZE                0x08

/** Beacon setup timeout in seconds */
#define UI_PROV_SETUP_TIMEOUT_SECS            30

/** Attention timeout for device in seconds */
#define UI_PROV_DEVICE_ATTENTION_TIMEOUT      30

#define PROV_AUTHVAL_SIZE_PL                  16

/** Authentication values for OOB Display - To be made random */
#define UI_DISPLAY_AUTH_DIGIT                 3
#define UI_DISPLAY_AUTH_NUMERIC               35007
#define UI_DISPLAY_AUTH_STRING                "f00l"



#define PROV_EVT_TBL_CNT    9

typedef enum _ALI_PROV_TYPE
{
    ALIG_UN_PROV,
    ALIG_IN_PROV,
    ALIG_CONFIG,
    ALIG_SILENCE_ADV

} ALI_PROV_TYPE;

typedef struct
{

    UCHAR event_type;
    UCHAR event_name[50];
    void (*func)(PROV_HANDLE* phandle,UCHAR event_type,API_RESULT event_result,void* event_data,UINT16 event_datalen);

} prov_event_tbl_t;


extern API_RESULT UI_prov_callback
(
    PROV_HANDLE* phandle,
    UCHAR         event_type,
    API_RESULT    event_result,
    void*         event_data,
    UINT16        event_datalen
);

extern uint8 UI_prov_state;

extern void UI_setup_prov(UCHAR role, UCHAR brr);
extern void UI_prov_bind(UCHAR brr, UCHAR index);

extern void UI_prov_cb_PROVISIONING_SETUP(PROV_HANDLE* phandle,UCHAR event_type,API_RESULT event_result,void* event_data,UINT16 event_datalen);
extern void UI_prov_cb_OOB_DISPLAY(PROV_HANDLE* phandle,UCHAR event_type,API_RESULT event_result,void* event_data,UINT16 event_datalen);
extern void UI_prov_cb_OOB_ENTRY(PROV_HANDLE* phandle,UCHAR event_type,API_RESULT event_result,void* event_data,UINT16 event_datalen);
extern void UI_prov_cb_DEVINPUT_COMPLETE(PROV_HANDLE* phandle,UCHAR event_type,API_RESULT event_result,void* event_data,UINT16 event_datalen);
extern void UI_prov_cb_PROVDATA_INFO(PROV_HANDLE* phandle,UCHAR event_type,API_RESULT event_result,void* event_data,UINT16 event_datalen);
extern void UI_prov_cb_PROVISIONING_COMPLETE(PROV_HANDLE* phandle,UCHAR event_type,API_RESULT event_result,void* event_data,UINT16 event_datalen);
extern void UI_prov_cb_default(PROV_HANDLE* phandle,UCHAR event_type,API_RESULT event_result,void* event_data,UINT16 event_datalen);

#endif //_ALIGENIE_APPL_PROV_H_
