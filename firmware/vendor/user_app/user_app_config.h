/********************************************************************************************************
 * @file     user_app_config.h 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2010, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *           
 *			 The information contained herein is confidential and proprietary property of Telink 
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms 
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai) 
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in. 
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this 
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided. 
 *           
 *******************************************************************************************************/
#pragma once

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

/*
    Note: only can use "#define", "#if .. #elif..#endif", etc. here. Don't use "enum" "typedef".
*/
#define GY_DEV_100F		0x100F//白色BC，Indoor（兼容已经出货的BC）
#define GY_DEV_1047		0x1047//黑色BC，Outdoor
#define GY_DEV_1048		0x1048//白色BC，Indoor
#define GY_DEV_108F		0x108f

#define GY_DEV_TYPE		GY_DEV_108F

#define GY_CID	0x07A5

#define GY_VID_SET(a,b,c)	((a<<11)|(b<<6)|(c<<0))


	#if(GY_DEV_TYPE == GY_DEV_100F)
#define GY_PID		0x100F
	#elif(GY_DEV_TYPE == GY_DEV_1048)
#define GY_PID		0x1048
	#elif(GY_DEV_TYPE == GY_DEV_1047)
#define GY_PID		0x1047
	#elif(GY_DEV_TYPE == GY_DEV_108F)
#define GY_PID		0x108f
#endif

#define GY_VID		GY_VID_SET(1,8,8)

#define GY_STATIC_OOB	0xbb,0x7b,0x06,0x5e,0x88,0xaf,0x5c,0x52,0xa7,0x8d,0xe2,0x6a,0x0a,0xb3,0x89,0x79

#define GY_DIM_PIN		GPIO_PD3//DIM引脚

#define GY_LED_R_PIN	GPIO_PC0//红色指示灯
#define GY_LED_G_PIN	GPIO_PD4//绿色指示灯
#define GY_PWM_FUNC_R  AS_PWM  // AS_PWM_SECOND
#define GY_PWM_FUNC_G  AS_PWM  // AS_PWM_SECOND
#define GY_PWMID_R     (GET_PWMID(GY_LED_R_PIN, GY_PWM_FUNC_R))
#define GY_PWMID_G     (GET_PWMID(GY_LED_G_PIN, GY_PWM_FUNC_G))
#define GY_PWM_INV_R   (GET_PWM_INVERT_VAL(GY_LED_R_PIN, GY_PWM_FUNC_R))
#define GY_PWM_INV_G   (GET_PWM_INVERT_VAL(GY_LED_G_PIN, GY_PWM_FUNC_G))

#define GY_BUTTON_PIN	GPIO_PB6//按键

#define GY_RELAY_PIN	GPIO_PA1//继电器

#define GY_I2C_SDA_PIN	GPIO_PC2
#define GY_I2C_SCK_PIN	GPIO_PC3

#if(GY_DEV_TYPE == GY_DEV_108F)
//#define GY_DIMMER_CURVE_TRANSFORMATION_MIN_VALUE 12
#define GY_DIMMER_CURVE_TRANSFORMATION_1V_VALUE (12.0f)
#define GY_DIMMER_CURVE_TRANSFORMATION_MAX_VALUE (100.0f)//89
#else
#define GY_DIMMER_CURVE_TRANSFORMATION_1V_VALUE (12.0f)
#define GY_DIMMER_CURVE_TRANSFORMATION_MAX_VALUE (89.0f)//89
#endif


#define GY_ZCD_PIN      GPIO_PD2


// ------- function Macro ---------
// #define CB_USER_FACTORY_RESET_ADDITIONAL()   cb_user_factory_reset_additional()
// #define CB_USER_PROC_LED_ONOFF_DRIVER(on)    cb_user_proc_led_onoff_driver(on)


#include "user_app_default.h"   // must at the end of this file

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif
