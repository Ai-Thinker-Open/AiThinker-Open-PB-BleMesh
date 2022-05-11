
#ifndef _ALIGENIE_LIGHT_H_
#define _ALIGENIE_LIGHT_H_

#include "MS_light_lightness_api.h"
#include "MS_light_ctl_api.h"
#include "MS_light_hsl_api.h"

/** -- Light - CTL */
#define LIGHT_CTL_TEMPERATURE_T_MIN    0x0320
#define LIGHT_CTL_TEMPERATURE_T_MAX    0x4E20

typedef struct ui_data_light_ctl_model UI_DATA_LIGHT_CTL_MODEL_T;

struct ui_data_light_ctl_model
{
    MS_ACCESS_ELEMENT_HANDLE        element_handle;
    MS_ACCESS_MODEL_HANDLE          model_handle;
    MS_ACCESS_MODEL_HANDLE          setup_model_handle;

    MS_STATE_LIGHT_CTL_STRUCT               UI_light_ctl;
    MS_STATE_LIGHT_CTL_DEFAULT_STRUCT       UI_light_ctl_default;
    MS_STATE_LIGHT_CTL_TEMPERATURE_STRUCT   UI_light_ctl_temperature;


};

/** -- Light - HSL */

typedef struct ui_data_light_hsl_model UI_DATA_LIGHT_HSL_MODEL_T;

struct ui_data_light_hsl_model
{
    MS_ACCESS_ELEMENT_HANDLE        element_handle;
    MS_ACCESS_MODEL_HANDLE          model_handle;
    MS_ACCESS_MODEL_HANDLE          setup_model_handle;

    MS_STATE_LIGHT_HSL_STRUCT       UI_light_hsl;
    MS_STATE_LIGHT_HSL_RANGE_STRUCT UI_light_hsl_range;
    MS_STATE_LIGHT_HSL_DEFAULT_STRUCT UI_light_hsl_default;

};

/** -- Light - Lightness */
typedef struct ui_data_light_lightness_model
{
    MS_ACCESS_ELEMENT_HANDLE        element_handle;
    MS_ACCESS_MODEL_HANDLE          model_handle;
    MS_STATE_LIGHT_LIGHTNESS_STRUCT     UI_light_lightness;

} UI_DATA_LIGHT_LIGHTNESS_MODEL_T;


/**
    Light HSL state is a composite state that includes the Light HSL Lighness,
    the Light HSL Hue and the Light HSL Saturation states
*/

extern API_RESULT UI_light_ctl_model_server_create(MS_ACCESS_ELEMENT_HANDLE element_handle);
extern API_RESULT UI_light_hsl_model_server_create(MS_ACCESS_ELEMENT_HANDLE element_handle);
extern API_RESULT UI_light_lightness_model_server_create(MS_ACCESS_ELEMENT_HANDLE element_handle);

#endif //_ALIGENIE_LIGHT_H_


