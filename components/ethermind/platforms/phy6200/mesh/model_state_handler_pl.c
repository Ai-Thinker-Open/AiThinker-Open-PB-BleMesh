
/**
 *  \file model_state_handler_pl.c
 *
 *
 */

/*
 *  Copyright (C) 2013. Mindtree Limited.
 *  All rights reserved.
 */

/* --------------------------------------------- Header File Inclusion */
#include "model_state_handler_pl.h"
#include "led_light.h"

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Static Global Variables */

/* --------------------------------------------- Functions */
void mesh_model_platform_init_pl(void)
{
    /* Map Platform related initializations of GPIOs/LEDs etc here */
}

void mesh_model_device_bootup_ind_pl(void)
{
    /* LED ON/OFF for BOOT UP Indication to be mapped here */
}

void mesh_model_device_provisioned_ind_pl(void)
{
    /* LED ON/OFF for Provisioning Indication to be mapped here */
}
uint8_t onoff=0;
uint8_t firstuse=1;
extern uint16_t s_light[5];
uint16_t s_light_buff[5];
extern MS_STATE_LIGHT_HSL_STRUCT UI_light_hsl;//change by johhn
void generic_onoff_set_pl (UINT8 state)
{
    /* LED ON/OFF for GENERIC ONOFF to be mapped here */
    if (state)
    {
			
			if(s_light_buff[LIGHT_RED]==0&&s_light_buff[LIGHT_GREEN]==0&&s_light_buff[LIGHT_GREEN]==0\
				&&s_light_buff[LIGHT_WARM]==0&&s_light_buff[LIGHT_COLD]==0\
				&&firstuse)
			{
				firstuse=0;
				s_light_buff[LIGHT_RED]=0xff;
				s_light_buff[LIGHT_GREEN]=0;
				s_light_buff[LIGHT_BLUE]=0;
				
				light_ctrl(LIGHT_RED    ,  s_light_buff[LIGHT_RED]);
				light_ctrl(LIGHT_GREEN  ,  s_light_buff[LIGHT_GREEN]);
				light_ctrl(LIGHT_BLUE   , s_light_buff[LIGHT_BLUE]);
				
				s_light_buff[LIGHT_WARM]=0;
				s_light_buff[LIGHT_COLD]=0;
				light_ctrl(LIGHT_WARM  ,  s_light_buff[LIGHT_WARM]);
				light_ctrl(LIGHT_COLD   , s_light_buff[LIGHT_COLD]);
				/*
				//light_ctrl(LIGHT_RED    ,  LIGHT_TOP_VALUE-1);
				//light_ctrl(LIGHT_GREEN  ,  LIGHT_TOP_VALUE-1);
				//light_ctrl(LIGHT_BLUE   , LIGHT_TOP_VALUE-1);
				*/
			}
			else
			{
				firstuse=0;
					printf("\n*************************\
									\n*************************\
									\n*************************\
									\n[HSL_I] %02x %02x %02x\n\
									\n*************************\
									\n*************************\
									\n*************************\n",s_light_buff[LIGHT_RED],s_light_buff[LIGHT_GREEN],s_light_buff[LIGHT_BLUE]);//change by johhn	
				
				light_ctrl(LIGHT_RED    ,  s_light_buff[LIGHT_RED]);
				light_ctrl(LIGHT_GREEN  ,  s_light_buff[LIGHT_GREEN]);
				light_ctrl(LIGHT_BLUE   , s_light_buff[LIGHT_BLUE]);
				
				light_ctrl(LIGHT_WARM  ,  s_light_buff[LIGHT_WARM]);
				light_ctrl(LIGHT_COLD   , s_light_buff[LIGHT_COLD]);
			}
				
        
    }
    else
    {
				memcpy(&s_light_buff,&s_light,sizeof(uint16_t)*5);
        light_ctrl(LIGHT_RED    , 0);
        light_ctrl(LIGHT_GREEN  , 0);
        light_ctrl(LIGHT_BLUE   , 0);
			
				light_ctrl(LIGHT_WARM    , 0);
        light_ctrl(LIGHT_COLD  , 0);
				
    }
}

void vendor_mode_mainlight_onoff_set_pl (UINT8 state)
{
    /* LED ON/OFF for vendor model mainlight ONOFF to be mapped here */
    if (state)
    {
        light_ctrl(LIGHT_GREEN  , LIGHT_TOP_VALUE-1);
    }
    else
    {
        light_ctrl(LIGHT_GREEN  , 0);
    }
}

void vendor_mode_backlight_onoff_set_pl (UINT8 state)
{
    /* LED ON/OFF for vendor model backlight ONOFF to be mapped here */
    if (state)
    {
        light_ctrl(LIGHT_RED    , LIGHT_TOP_VALUE-1);
    }
    else
    {
        light_ctrl(LIGHT_RED    , 0);
    }
}




static float Hue_2_RGB( float v1, float v2, float vH ) //Function Hue_2_RGB
{
    if ( vH < 0 ) vH += 1;
    if ( vH > 1 ) vH -= 1;
    if (( 6 * vH ) < 1 ) return ( v1 + ( v2 - v1 ) * 6 * vH );
    if (( 2 * vH ) < 1 ) return ( v2 );
    if (( 3 * vH ) < 2 ) return ( v1 + ( v2 - v1 ) * ( ( 2/3.0 ) - vH ) * 6 );
    return ( v1 );
}
uint16_t set_ligtnessValue=LIGHT_TOP_VALUE;//色温默认亮度
void light_lightness_set_pl (uint16_t ligtnessValue)
{

    //light_ctrl(LIGHT_RED    , ligtnessValue>>10);
    //light_ctrl(LIGHT_GREEN  , ligtnessValue>>10);
    //light_ctrl(LIGHT_BLUE   , ligtnessValue>>10);change by johhn
	
	if(s_light[LIGHT_RED] + s_light[LIGHT_GREEN] + s_light[LIGHT_BLUE])
	{
		ligtnessValue=ligtnessValue/655;
		UI_light_hsl.hsl_lightness=65535*ligtnessValue/100/2;//亮到底就是白色，取中间L
		//light_hsl_set_pl();
		UI_light_hsl_set_actual(0,UI_light_hsl.hsl_lightness,UI_light_hsl.hsl_hue,UI_light_hsl.hsl_saturation,0);
		
		/*
			float R,G,B;
			R=(float)s_light[LIGHT_RED]/255.0;
			G=(float)s_light[LIGHT_GREEN]/255.0;
			B=(float)s_light[LIGHT_BLUE]/255.0;

			printf("\n*************************\
			\n*************************\
			\n*************************\
			\nlight_lightness_set_pl %f %f %f\n\n\
			\n*************************\
			\n*************************\
			\n*************************\n",R,G,B);//change by johhn
			uint16_t max;

			ligtnessValue=ligtnessValue/275;//255
			if(s_light[LIGHT_RED]>=s_light[LIGHT_GREEN])
			{
				max=s_light[LIGHT_RED];
			}
			else
			{
				max=s_light[LIGHT_GREEN];
			}
			if(s_light[LIGHT_BLUE]>max)
			{
				max=s_light[LIGHT_BLUE];
			}

			float ligtnessValue1=ligtnessValue;
			float max1=max;
			float num=ligtnessValue1/max1;
			uint16_t R_int = (uint16_t)(R*LIGHT_TURN_ON*num);
			uint16_t G_int = (uint16_t)(G*LIGHT_TURN_ON*num);
			uint16_t B_int = (uint16_t)(B*LIGHT_TURN_ON*num);

			printf("\n*************************\
			\n**ligtnessValue1:%f max1=%f**\
			\n*************************\
			\nnum %f max=%x \n\
			\n*************************\
			\n*************************\
			\n*************************\n",ligtnessValue1,max1,num,max);//change by johhn		

			light_ctrl(LIGHT_RED, R_int);
			light_ctrl(LIGHT_GREEN, G_int);
			light_ctrl(LIGHT_BLUE, B_int);
			printf("\n*************************\
			\n*************************\
			\n*************************\
			\n[light_lightness_set_pl] %d %x %d\n\
			\n*************************\
			\n*************************\
			\n*************************\n",R_int,G_int,B_int);//change by johhn	
			*/
			memcpy(&s_light_buff,&s_light,sizeof(uint16_t)*5);
	
	}
	else if(s_light[LIGHT_WARM] + s_light[LIGHT_COLD])
	{
		/*
			float W,C;
			W=(float)s_light[LIGHT_WARM]/255.0;
			C=(float)s_light[LIGHT_COLD]/255.0;
		
			printf("\n*************************\
			\n*************************\
			\n*************************\
		\nlight_lightness_set_pl W:%f C:%f \n\n\
			\n*************************\
			\n*************************\
			\n*************************\n",W,C);//change by johhn
			
			uint16_t max;

			ligtnessValue=ligtnessValue/275;//255
			set_ligtnessValue=ligtnessValue;
			
			if(abs(s_light[LIGHT_WARM]-ligtnessValue)<=abs(s_light[LIGHT_COLD]-ligtnessValue))
			{
				max=s_light[LIGHT_WARM];
			}
			else
			{
				max=s_light[LIGHT_COLD];
			}
			
			float ligtnessValue1=ligtnessValue;
			float max1=max;
			float num=ligtnessValue1/max1;
			uint16_t W_int = (uint16_t)(W*LIGHT_TURN_ON*num);
			uint16_t C_int = (uint16_t)(C*LIGHT_TURN_ON*num);
			
			printf("\n*************************\
			\n**ligtnessValue1:%f max1=%f**\
			\n*************************\
			\nnum %f max=%x \n\
			\n*************************\
			\n*************************\
			\n*************************\n",ligtnessValue1,max1,num,max);//change by johhn		
		
			{
				light_ctrl(LIGHT_WARM, W_int);
				light_ctrl(LIGHT_COLD, C_int);

				memcpy(&s_light_buff,&s_light,sizeof(uint16_t)*5);
			
			}
			printf("\n*************************\
			\n*************************\
			\n*************************\
			\n[light_lightness_set_pl] %d %d\n\
			\n*************************\
			\n*************************\
			\n*************************\n",W_int,C_int);//change by johhn	
			*/
			ligtnessValue=ligtnessValue/275;//255
			set_ligtnessValue=ligtnessValue;
			light_ctrl(LIGHT_WARM, ligtnessValue/2);
			light_ctrl(LIGHT_COLD, ligtnessValue/2);
			memcpy(&s_light_buff,&s_light,sizeof(uint16_t)*5);
	}
	
	

	
}

void light_ctl_set_pl (uint16_t ctlValue,uint16_t dltUV)
{
    if(ctlValue<6600)
    {
        light_ctrl(LIGHT_RED    , 255);
    }
    else
    {
        light_ctrl(LIGHT_RED    , 255-(ctlValue-6600)*(255-160)/(20000-6600));
    }

    if(ctlValue<6600)
    {
        light_ctrl(LIGHT_GREEN  , 50+200*(6600-ctlValue)/6600);
    }
    else
    {
        light_ctrl(LIGHT_GREEN  , 255-(ctlValue-6600)*(255-190)/(20000-6600));
    }
    if(ctlValue<2000)
    {
        light_ctrl(LIGHT_BLUE   , 0);
    }
    else if(ctlValue<6500)
    {
        light_ctrl(LIGHT_BLUE  , 255*(6600-ctlValue)/(6500-2000));
    }
    else
    {
        light_ctrl(LIGHT_BLUE   , 255);
    }
    

}



void light_hsl_set_pl (uint16_t H_int,uint16_t S_int,uint16_t L_int)
{




    float H = (float)H_int / 65535.0;
    float S = (float)S_int / 65535.0;
    float L = (float)L_int / 65535.0;
    float R,G,B,var_1,var_2;
    if ( S == 0 )
    {
        R = L;
        G = L;
        B = L;
    }
    else
    {
        if ( L < 0.5 )
            var_2 = L * ( 1 + S );
        else 
            var_2 = ( L + S ) - ( S * L );
        var_1 = 2 * L - var_2;
        R = Hue_2_RGB( var_1, var_2, H + ( 1/3.0 ));
        G = Hue_2_RGB( var_1, var_2, H );
        B = Hue_2_RGB( var_1, var_2, H - ( 1/3.0 ));
    }

    uint16_t R_int = (uint16_t)(R*LIGHT_TURN_ON);
    uint16_t G_int = (uint16_t)(G*LIGHT_TURN_ON);
    uint16_t B_int = (uint16_t)(B*LIGHT_TURN_ON);
//    printf("[HSL_f] %f %f %f\n",R,G,B);
//    printf("[HSL_I] %02x %02x %02x\n",R_int,G_int,B_int);
		printf("\n*************************\
									\n*************************\
									\n*************************\
									\n[light_hsl_set_pl] %d %d %d\n\n\
									\n*************************\
									\n*************************\
									\n*************************\n",H_int,S_int,L_int);//change by johhn
		
			printf("\n*************************\
									\n*************************\
									\n*************************\
									\n[HSL_f] %f %f %f\n\n\
									\n*************************\
									\n*************************\
									\n*************************\n",R,G,B);//change by johhn
		printf("\n*************************\
									\n*************************\
									\n*************************\
									\n[HSL_I] %02x %02x %02x\n\
									\n*************************\
									\n*************************\
									\n*************************\n",R_int,G_int,B_int);//change by johhn		
    light_ctrl(LIGHT_RED, R_int);
    light_ctrl(LIGHT_GREEN, G_int);
    light_ctrl(LIGHT_BLUE, B_int);
		light_ctrl(LIGHT_WARM    , 0);
     light_ctrl(LIGHT_COLD  , 0);
		memcpy(&s_light_buff,&s_light,sizeof(uint16_t)*5);

}

//    light_hsl_set_pl(0x5555,0xffff,0x8000); // 0x00 0xFF 0x00 Green
//    light_hsl_set_pl(0xaaaa,0xffff,0x8000); // 0x00 0x00 0xFF Blue
//    light_hsl_set_pl(0x0000,0xffff,0x8000);// 0xFF 0x00 0x00 Red
//

void light_temperature_set_pl (UINT16 temperature)
{
	UINT16 get=(temperature-0x320)/192;
	float rate=(float)get/100.0;
	
	UINT16 cold=(UINT16)set_ligtnessValue*rate;
	UINT16 warm=(UINT16)set_ligtnessValue*(1-rate);

	light_ctrl(LIGHT_WARM,warm);
	light_ctrl(LIGHT_COLD, cold);
	light_ctrl(LIGHT_RED    , 0);
  light_ctrl(LIGHT_GREEN  , 0);
  light_ctrl(LIGHT_BLUE   , 0);
	memcpy(&s_light_buff,&s_light,sizeof(uint16_t)*5);


}



