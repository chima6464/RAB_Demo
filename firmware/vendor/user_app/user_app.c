/********************************************************************************************************
 * @file     user_app.c 
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
#include "proj/tl_common.h"
#if !WIN32
#include "proj/mcu/watchdog_i.h"
#include "proj_lib/mesh_crypto/mesh_md5.h"
#include "vendor/common/myprintf.h"
#endif 
#include "proj_lib/ble/ll/ll.h"
#include "proj_lib/ble/blt_config.h"
#include "vendor/common/user_config.h"
#include "proj_lib/ble/service/ble_ll_ota.h"
#include "vendor/common/app_health.h"
#include "proj_lib/sig_mesh/app_mesh.h"
#include "vendor/common/app_provison.h"
#include "vendor/common/lighting_model.h"
#include "vendor/common/sensors_model.h"
#include "vendor/common/remote_prov.h"
#include "proj_lib/mesh_crypto/sha256_telink.h"
#include "proj_lib/mesh_crypto/le_crypto.h"
#include "vendor/common/lighting_model_LC.h"
#include "vendor/common/mesh_ota.h"
#include "vendor/common/mesh_common.h"
#include "vendor/common/mesh_config.h"
#include "vendor/common/directed_forwarding.h"
#include "vendor/common/certify_base/certify_base_crypto.h"

#if(__TL_LIB_8258__ || (MCU_CORE_TYPE == MCU_CORE_8258))
#include "stack/ble/ble.h"
#elif(MCU_CORE_TYPE == MCU_CORE_8278)
#include "stack/ble_8278/ble.h"
#endif

#if FAST_PROVISION_ENABLE
#include "vendor/common/fast_provision_model.h"
#endif

#if (HCI_ACCESS==HCI_USE_UART)
#include "proj/drivers/uart.h"
#endif


void cb_user_factory_reset_additional()
{
    // TODO
}

void cb_user_proc_led_onoff_driver(int on)
{
    // TODO
}

u32 lc_energy_delay=0;
u8  lc_energy_report_delay_start_flag=0;
u16 lc_unicast_adress = 0;
u16 lc_threshold_value_lux=0;
u16 lc_current_lux=0;
GY_FLASH_WILL_NOT_ERASE_INFO gy_flash_will_not_erase_info;
s32 lc_pid_error=0;
s32 lc_pid_integral=0;
s32 lc_pid_lightness_s32=0;
u16 lc_pid_lightness_u16=0;
s32 lc_pid_error_last=0;
float lc_Kp=7.0;
float lc_Ki=5.8;


u8 lc_pid_is_stable=0;

u8 lc_tid=1;
u8 lc_power_on_flag=1;
u8 lc_pid_change_confirm=0;

u16 lc_photocell_threshold_high=390;
u16 lc_orig_lux=0;
u16 lc_measure_lux=0;

u8 lc_daylight_calibrate_flag=0;
u8 lc_photocell_freeze_flag=0;
u16 lc_photocell_control_count=0;
u8 lc_photocell_caused_by_motion_sensor_flag=0;
u16 lc_adjust_interval_count=0;

u8 lc_als_read_flag=0;
u8 lc_donot_refresh_occupied_countdown_flag=0;

//自定义对数调光曲线和平方调光曲线
u16 gy_curve_map[2][101] = {
		{
				0 ,
				655 ,
				694 ,
				731 ,
				769 ,
				809 ,
				850 ,
				894 ,
				939 ,
				986 ,
				1038 ,
				1086 ,
				1137 ,
				1190 ,
				1246 ,
				1304 ,
				1365 ,
				1429 ,
				1496 ,
				1565 ,
				1639 ,
				1717 ,
				1800 ,
				1886 ,
				1976 ,
				2071 ,
				2169 ,
				2273 ,
				2380 ,
				2493 ,
				2611 ,
				2734 ,
				2862 ,
				2995 ,
				3136 ,
				3282 ,
				3436 ,
				3597 ,
				3765 ,
				3941 ,
				4126 ,
				4321 ,
				4525 ,
				4739 ,
				4963 ,
				5197 ,
				5442 ,
				5698 ,
				5966 ,
				6247 ,
				6541 ,
				6851 ,
				7175 ,
				7514 ,
				7868 ,
				8240 ,
				8628 ,
				9035 ,
				9461 ,
				9906 ,
				10373 ,
				10864 ,
				11377 ,
				11915 ,
				12477 ,
				13066 ,
				13683 ,
				14328 ,
				15003 ,
				15710 ,
				16451 ,
				17228 ,
				18042 ,
				18894 ,
				19785 ,
				20719 ,
				21696 ,
				22719 ,
				23790 ,
				24912 ,
				26086 ,
				27317 ,
				28606 ,
				29956 ,
				31369 ,
				32849 ,
				34400 ,
				36024 ,
				37724 ,
				39504 ,
				41368 ,
				43346 ,
				45417 ,
				47583 ,
				49851 ,
				52224 ,
				54511 ,
				56915 ,
				59439 ,
				62091 ,
				65280
		},
		{
				0,
				655 ,
				674 ,
				707 ,
				752 ,
				810 ,
				881 ,
				965 ,
				1062 ,
				1172 ,
				1295 ,
				1426 ,
				1558 ,
				1690 ,
				1821 ,
				1953 ,
				2084 ,
				2216 ,
				2348 ,
				2479 ,
				2611 ,
				2879 ,
				3161 ,
				3455 ,
				3762 ,
				4082 ,
				4414 ,
				4760 ,
				5119 ,
				5491 ,
				5875 ,
				6274 ,
				6686 ,
				7110 ,
				7548 ,
				7998 ,
				8462 ,
				8938 ,
				9427 ,
				9930 ,
				10445 ,
				10974 ,
				11516 ,
				12072 ,
				12640 ,
				13221 ,
				13815 ,
				14422 ,
				15042 ,
				15674 ,
				16320 ,
				16980 ,
				17653 ,
				18339 ,
				19037 ,
				19749 ,
				20473 ,
				21211 ,
				21961 ,
				22725 ,
				23501 ,
				24291 ,
				25095 ,
				25911 ,
				26740 ,
				27582 ,
				28438 ,
				29306 ,
				30187 ,
				31080 ,
				31987 ,
				32908 ,
				33842 ,
				34789 ,
				35749 ,
				36722 ,
				37707 ,
				38706 ,
				39717 ,
				40742 ,
				41779 ,
				42831 ,
				43895 ,
				44973 ,
				46063 ,
				47166 ,
				48283 ,
				49412 ,
				50554 ,
				51709 ,
				52877 ,
				54059 ,
				55254 ,
				56462 ,
				57683 ,
				58917 ,
				60164 ,
				61423 ,
				62696 ,
				63982 ,
				65280
		}
};









//重定义light_dim_refresh函数
void light_dim_refresh(int idx)// idx: index of LIGHT_CNT.
{
	//st_transition_t *p_trans = P_ST_TRANS(idx, ST_TRANS_LIGHTNESS);
	st_transition_t *p_trans = (&light_res_sw[idx].trans[ST_TRANS_LIGHTNESS]);
	u8 lum_100 = level2lum(p_trans->present);
	CB_NL_PAR_NUM_3(p_nl_level_state_changed,idx * ELE_CNT_EVERY_LIGHT + ST_TRANS_LIGHTNESS, p_trans->present, p_trans->target);

	if((p_trans->target + 32768) != gy_dim_value_record)
	{
		if(gy_dim_value_record == 0 && (p_trans->target + 32768) != 0)//off -> on
		{
			gy_h_sun_off_on_off_count_down_refresh();
		}
		else if(gy_dim_value_record != 0 && (p_trans->target + 32768) == 0)
		{
			//if(gy_h_sun_off_on_off_10ms_count_down)
			//{
			//	gy_sensor_freeze_time_count_down = GY_H_SUN_OFF_ON_OFF_FREEZE_TIME*1000/(10*GY_READ_I2C_DATA_INTERVAL);//freeze time设置为2秒
			//}
		}
		gy_dim_value_record = p_trans->target + 32768;
	}

	/*
	if((p_trans->target + 32768) != gy_dim_value_record)
	{
		gy_dim_value_record = p_trans->target + 32768;
		if(gy_dim_value_record == 0)
		{
			gy_sensor_occupied_time_count_clear();
		}
		else
		{
			gy_sensor_occupied_time_count_refresh();
		}
	}

	if((p_trans->target + 32768) == 0)
	{
		gy_sensor_occupied_time_count_clear();
	}
	*/
	/*
	gy_dim_value_record = p_trans->target + 32768;

	if(gy_flash_info.sensor_property.unoccupied_onoff == 0 && (gy_dim_value_record/655 > 0))
	{
		gy_sensor_occupied_time_count_refresh();
	}
	else if(gy_flash_info.sensor_property.unoccupied_onoff == 1 && (gy_dim_value_record/655 == 0))
	{
		gy_sensor_occupied_time_count_refresh();
	}
	else if(gy_flash_info.sensor_property.unoccupied_level == 0 && (gy_dim_value_record/655 > 0))
	{
		gy_sensor_occupied_time_count_refresh();
	}
	else if(gy_flash_info.sensor_property.unoccupied_level != 0 && ((gy_flash_info.sensor_property.unoccupied_level/655) != (gy_dim_value_record/655)))
	{
		gy_sensor_occupied_time_count_refresh();
	}
	else
	{
		gy_sensor_occupied_time_count_clear();
	}
	*/

	if(lum_100/* && gy_public_info.current_light_onoff_state*/)
	{
#if(GY_DEV_TYPE == GY_DEV_108F)
		gy_public_info.light_ssr_state =  gy_relay_on ;
#else
		gy_relay_onoff_set(gy_relay_on);
#endif
		if(gy_light_info.state != gy_light_on)
		{
			gy_light_info.state = gy_light_on;
		}
	}
	else
	{
#if(GY_DEV_TYPE == GY_DEV_108F)
		gy_public_info.light_ssr_state =  gy_relay_off ;
#else
		gy_relay_onoff_set(gy_relay_off);
#endif
		if(gy_light_info.state != gy_light_off)
		{
			gy_light_info.state = gy_light_off;
		}
	}

	//if(gy_public_info.current_light_onoff_state || !lum_100)
	{
		extern void light_dim_set_hw(int idx, int idx2, u16 val);
		//light_dim_set_hw(idx, 0, get_pwm_cmp(0xff, lum_100));
		if(gy_flash_info.current_curve == gy_curve_linear)
		{
			//light_dim_set_hw(idx, 0, (u32)lum_100*(255*256)/100);
			light_dim_set_hw(idx, 0, (u32)gy_dimmer_curve_transformation_calculate((u32)lum_100*(255*256)/100));
		}
		else
		{
			//light_dim_set_hw(idx, 0, gy_curve_map[gy_flash_info.current_curve-1][lum_100]);
			light_dim_set_hw(idx, 0, (u32)gy_dimmer_curve_transformation_calculate(gy_curve_map[gy_flash_info.current_curve-1][lum_100]));
		}
	}
	return;
}














//****gy_user_define_beacon***//
void gy_dev_info_beacon_send(void)
{
	GY_DEV_INFO_BEACON gy_dev_info_beacon_data_t;
	gy_dev_info_beacon_data_t.trans_par_val = TRANSMIT_DEF_PAR_BEACON;
	gy_dev_info_beacon_data_t.len = 30;
	gy_dev_info_beacon_data_t.type = MESH_ADV_TYPE_BEACON;
	gy_dev_info_beacon_data_t.beacon_type = 0xFF;
	memcpy(gy_dev_info_beacon_data_t.uuid, prov_para.device_uuid, 16);
	gy_dev_info_beacon_data_t.version = GY_USER_DEFINE_BEACON_VERSION;
	gy_dev_info_beacon_data_t.pid = GY_PID;
	gy_dev_info_beacon_data_t.vid = GY_VID;
	mesh_tx_cmd_add_packet((u8 *)(&gy_dev_info_beacon_data_t));
	return;
}











//****gy_rsp_scan***//
void mesh_scan_rsp_init(void)//扫描回复函数重定义
{
	GY_SCAN_RSP_INFO gy_scan_rsp_info_t;
	gy_scan_rsp_info_t.type = GAP_ADTYPE_MANUFACTURER_SPECIFIC;
	gy_scan_rsp_info_t.cid = g_vendor_id;//GY_CID
	memcpy(gy_scan_rsp_info_t.mac, tbl_mac, sizeof(gy_scan_rsp_info_t.mac));
	gy_scan_rsp_info_t.version = GY_CURRENT_VERSION;
	gy_scan_rsp_info_t.pid = GY_PID;
	gy_scan_rsp_info_t.vid = GY_VID;

	gy_i2c_check_sensor_type();
	gy_scan_rsp_info_t.sensor_type = gy_i2c_info.sensor_type;
	//gy_scan_rsp_info_t.sensor_type = gy_i2c_sensor_none;

	GY_FLASH_FIX_INFO gy_flash_fix_info_t;
	gy_flash_fix_read(&gy_flash_fix_info_t);
	gy_scan_rsp_info_t.light_id = gy_flash_fix_info_t.light_id;

	memset(gy_scan_rsp_info_t.reserve, 0, sizeof(gy_scan_rsp_info_t.reserve));
	gy_scan_rsp_info_t.len = sizeof(gy_scan_rsp_info_t) - 1;
	bls_ll_setScanRspData((u8 *)&gy_scan_rsp_info_t, sizeof(gy_scan_rsp_info_t));
	return;
}









//***gy_app_init***//
u16 gy_dim_value_record = 0;
void gy_app_init(void)
{
	gy_FCT_init();
	gy_public_init();
	gy_led_init();
	gy_button_init();
	gy_light_init();
	gy_i2c_init();
	gy_vendor_model_init();
	gy_factory_reset_method_init();
//	LOG_USER_MSG_INFO(0,0,"GY_DEV_TYPE：0x%x",GY_DEV_TYPE);
#if(GY_DEV_TYPE == GY_DEV_108F)
	lc_zero_crossing_detecte_init();
#endif
	return;
}



void lc_zero_crossing_detecte_init()
{

	gpio_set_interrupt_init(GY_ZCD_PIN,PM_PIN_PULLDOWN_100K,0,FLD_IRQ_GPIO_RISC0_EN);
//	gpio_set_func(GY_ZCD_PIN, AS_GPIO);
//	gpio_set_input_en(GY_ZCD_PIN, 1);
//	gpio_setup_up_down_resistor(GY_ZCD_PIN, PM_PIN_PULLDOWN_100K);
}








//***gy_timer***//
void gy_10ms_timer(u32 count)
{
	//gy_three_defult_scene_set_10ms_timer(count);
	gy_flash_10ms_timer(count);
	gy_flash_new_10ms_timer(count);
	gy_led_10ms_timer(count);
	gy_button_10ms_timer(count);
	gy_i2c_10ms_timer(count);
	gy_online_status_10ms_timer(count);
	gy_factory_reset_method_10ms_timer(count);
	lc_energy_report_10ms_timer(count);
	gy_i2c_sensor_als_meter_10ms_timer(count);
	return;
}

void gy_i2c_sensor_als_meter_10ms_timer(u32 count)
{
	if(lc_als_read_flag)
	{
		u8 gy_buff_t[4] = {0x01, 0x1c, 0x00, 0x00};
		lc_als_read_flag=0;
		switch(gy_i2c_info.sensor_type)
		{
		case gy_i2c_sensor_1:
		{
			i2c_set_id(GY_LTR_329ALS_01_ADDR);
			i2c_read_series(GY_L_SENSOR_ALS_STATUS_REG,1,&gy_buff_t[0],1);
			if(gy_buff_t[0]&0x04)
			{
				i2c_read_series(GY_L_SENSOR_ALS_DATA_CH1_0REG,1,gy_buff_t,4);
				gy_buff_t[0] = 1;
				gy_buff_t[1] = 0x1c;

			}
			break;
		}
		case gy_i2c_sensor_2:
		{

			i2c_read_series(GY_MW_SENSOR_ALS_ADC1_REG,1,&gy_buff_t[2],2);
			break;
		}
		case gy_i2c_sensor_3:
		{

			gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x04, &gy_buff_t[3], 1);
			break;
		}

		}
		mesh_tx_cmd2normal(gy_vendor_opcode_sensor_status,gy_buff_t,4,ele_adr_primary,lc_unicast_adress,0);

	}

}

void lc_energy_report_10ms_timer(u32 count)
{
	static u32 lc_random_delay_energy_count=0;
	u16 gy_dev_addr_t = 0x0000;
    u32 lc_report_total_energy=0;

	if((lc_random_delay_energy_count>=lc_energy_delay)&&(lc_energy_delay!=0))
	{
		lc_report_total_energy = (u32)(gy_flash_energy_info.total_energy+0.5);
//		u8 lc_mac[6] = {0,0,0,0,0,0};
//		memcpy(lc_mac, tbl_mac, 6);
		gy_dev_addr_t = gy_get_sub(gy_sub_type_area);
		u8 gy_send_data_t[] = {gy_vendor_property_id_total_energy,(u8)gy_dev_addr_t, (u8)(gy_dev_addr_t>>8),(u8)(lc_report_total_energy>>0),(u8)(lc_report_total_energy>>8),(u8)(lc_report_total_energy>>16),(u8)(lc_report_total_energy>>24)};
		//mesh_tx_cmd2normal_primary(u16 op, u8 *par, u32 par_len, u16 adr_dst, int rsp_max)
		mesh_tx_cmd2normal_primary(0xE6, gy_send_data_t, sizeof(gy_send_data_t), lc_unicast_adress, 0);
		lc_random_delay_energy_count=0;
		lc_energy_report_delay_start_flag=0;
	}

	if(lc_energy_report_delay_start_flag)
		lc_random_delay_energy_count++;
}

//***gy_timer***//
void gy_1s_timer(u32 count)
{
	gy_flash_1s_timer(count);
	gy_flash_new_1s_timer(count);
	gy_button_1s_timer(count);
	gy_sensor_occupied_time_count_down_1s_tmer(count);
	gy_set_default_group_1s_timer(count);
	//LOG_USER_MSG_INFO(0,0,"light_res_sw_save[0].level[0].last： %d",s16_to_u16(light_res_sw_save[0].level[0].last));
	lc_energy_count_1s_timer(count);
	lc_daylight_close_loop_1s_tmer();
	lc_photocell_control_1s_timer();
	lc_photocell_freeze_1s_timer();
	lc_light_meter_1s_tmer();
	return;
}

void lc_light_meter_1s_tmer()
{
	static u8 lc_lux_count=0;
	if(lc_daylight_calibrate_flag)
	{
		lc_lux_count++;
	}

	if(lc_lux_count>=2)
	{
		lc_lux_count=0;
		lc_daylight_calibrate_flag=0;
		lc_read_lux();
		gy_flash_info.sensor_property.coefficient_of_als_calibration=(u16)(((float)lc_measure_lux/lc_orig_lux)*10+0.5);
		gy_flash_info_delay_write_start();
	}
}

void lc_photocell_freeze_1s_timer()
{
	static u16 lc_photocell_freeze_count=0;
	if(lc_photocell_freeze_flag)
		lc_photocell_freeze_count++;
	if(lc_photocell_freeze_count >= gy_flash_info.sensor_property.photocell_freeze_time*60)
	{
		lc_photocell_freeze_count=0;
		lc_photocell_freeze_flag=0;
	}
}

void lc_photocell_control_1s_timer()
{

	if(lc_photocell_control_count >= gy_flash_info.sensor_property.daylight_ctrl_freq)
	{
		lc_photocell_control_count =0;
		if(gy_flash_info.sensor_property.photocell_onoff_control)
		{
			lc_read_lux();
			if(lc_current_lux >= lc_photocell_threshold_high)
			{
				if((light_res_sw_save[0].level[0].onoff!=0)&&(gy_flash_info.sensor_property.sensor_role==1))
				{
					if(lc_photocell_freeze_flag==0)
					{
						if(gy_flash_info.sensor_property.photocell_onoff_control==2)
						gy_set_ctl_onoff(gy_get_sub(gy_sub_type_area), 0);
						else
							gy_set_ctl_onoff(gy_get_sub(gy_sub_type_sub), 0);
					}
				}
			}
			else if(lc_current_lux < gy_flash_info.sensor_property.photocell_threshold)
			{
				if(gy_flash_info.sensor_property.sensor_device_mode == gy_device_mode_disable)
				{
					if(gy_flash_info.sensor_property.sensor_role==1)
					//if((light_res_sw_save[0].level[0].onoff==0)&&(gy_flash_info.sensor_property.sensor_role==1))
						{
							if(gy_flash_info.sensor_property.photocell_onoff_control==2)
							{
							    gy_set_ctl_onoff(gy_get_sub(gy_sub_type_area), 1);
							}
							else
							{
								gy_set_ctl_onoff(gy_get_sub(gy_sub_type_sub), 1);
							}
							 lc_photocell_freeze_flag=1;
						}
				}
			}
		}

	}
	lc_photocell_control_count++;
}

void lc_daylight_close_loop_1s_tmer()
{
	if(lc_adjust_interval_count>=gy_flash_info.sensor_property.daylight_ctrl_freq)
	{
		lc_adjust_interval_count=0;

	}
	else
	{
		lc_adjust_interval_count++;
		LOG_USER_MSG_INFO(0,0,"lc_adjust_interval_count++",0);
		if(lc_adjust_interval_count==gy_flash_info.sensor_property.daylight_ctrl_freq&&gy_flash_info.sensor_property.daylight_ctrl_freq)
		{

			lc_adjust_interval_count=0;
			if(((gy_flash_info.sensor_property.sensor_role==1)&&(gy_flash_info.sensor_property.daylight_harvesting==2))||(gy_flash_info.sensor_property.daylight_harvesting==1))
			{
				if(((gy_public_info.occupied_time_count_down>0)&&(gy_flash_info.sensor_property.sensor_device_mode >= 1) && (gy_flash_info.sensor_property.sensor_device_mode <= 3))||(gy_flash_info.sensor_property.sensor_device_mode == gy_device_mode_disable))
				lc_pid_light_control();
			}
		}
	}
}

void lc_pid_light_control()
{

	lc_read_lux();

//	u16 gy_current_lightness_value_t = s16_to_u16(light_res_sw_save[0].level[0].last);
	lc_pid_error=(gy_flash_info.sensor_property.target_light_level-lc_current_lux);

//	if(lc_donot_change_integral==0)


	if(lc_pid_lightness_s32<=0)
	{
		if(lc_pid_error<0)
		{

		}
		else
			lc_pid_integral += lc_pid_error;
	}
	else if(lc_pid_lightness_s32>=65535)
	{
		if(lc_pid_error>0)
		{

		}
		else
			lc_pid_integral += lc_pid_error;
	}
	else
	{
		if(lc_pid_is_stable)
		{

		}
		else if(light_res_sw_save[0].level[0].onoff==0)
		{

		}
		else
		lc_pid_integral += lc_pid_error;
	}

	if(lc_power_on_flag)
	{
		lc_power_on_flag=0;
		lc_pid_integral=(s32)((s16_to_u16(light_res_sw_save[0].level[0].last)-lc_Kp*lc_pid_error)/lc_Ki+0.5);
	}

	lc_pid_lightness_s32=(s32)(lc_Kp*lc_pid_error+lc_Ki*lc_pid_integral+0.5);

//	u8 gy_buff_t[10]={1,89,(u8)(lc_pid_integral>>24),(u8)(lc_pid_integral>>16),(u8)(lc_pid_integral>>8),(u8)(lc_pid_integral>>0),(u8)(lc_pid_lightness_s32>>24),(u8)(lc_pid_lightness_s32>>16),(u8)(lc_pid_lightness_s32>>8),(u8)(lc_pid_lightness_s32>>0)};
//	mesh_tx_cmd2normal(gy_vendor_opcode_sensor_status,gy_buff_t,10,ele_adr_primary,lc_unicast_adress ,0);

	lc_pid_error_last = lc_pid_error;

	if(lc_pid_lightness_s32<=0)
	{
		if(light_res_sw_save[0].level[0].onoff!=0)
		{
			if(gy_flash_info.sensor_property.daylight_harvesting==1)
			lc_individual_daylight_control(655);
			else
			gy_light_dim_value_set_1000ms(gy_get_sub(gy_sub_type_area), 655);
		}
	}
	else if(lc_pid_lightness_s32>=65535)
	{

		if(light_res_sw_save[0].level[0].onoff!=0)
		{
				if(gy_flash_info.sensor_property.daylight_harvesting==1)
					lc_individual_daylight_control(0xffff);
				else
					gy_light_dim_value_set_1000ms(gy_get_sub(gy_sub_type_area), 0xffff);

		}
	}
	else
	{
		if((abs(lc_pid_error)*10)>(gy_flash_info.sensor_property.coefficient_of_als_calibration*2))
		{
			if(lc_pid_change_confirm==0)
			lc_pid_change_confirm=1;
			else
			{
				lc_pid_is_stable=0;
				if(light_res_sw_save[0].level[0].onoff!=0)
				//if((light_res_sw_save[0].level[0].onoff!=0)||(gy_flash_info.sensor_property.photocell_onoff_control==1)||(gy_flash_info.sensor_property.sensor_device_mode != gy_device_mode_disable))
				{
					if(gy_flash_info.sensor_property.daylight_harvesting==1)
						lc_individual_daylight_control(lc_pid_lightness_s32);
					else
					gy_light_dim_value_set_1000ms(gy_get_sub(gy_sub_type_area), lc_pid_lightness_s32);
				}
			}
		}
		else
		{
			lc_pid_change_confirm=0;
			lc_pid_is_stable=1;
		}
	}



}

void lc_energy_count_1s_timer(u32 count)
{
	static u16 lc_1min_energy_count=0;
	static u16 lc_20min_energy_count=0;



	if(lc_1min_energy_count>=60)
	{
		//gy_flash_will_not_erase_info.lc_total_energy += (lc_read_power_to_calculate_energy()/10.0);
		gy_flash_energy_info.total_energy += (lc_read_power_to_calculate_energy()/10.0);
		lc_1min_energy_count=0;
	}

	if(lc_20min_energy_count>=120)
	{
		//gy_flash_will_not_erase_info_write();
		gy_flash_energy_write();
		lc_20min_energy_count=0;
	}



	lc_1min_energy_count++;
	lc_20min_energy_count++;

}









//***gy_public***//
GY_PUBLIC_INFO gy_public_info = {
		.occupied_time_count_down = 0,
		.standby_time_count_down = 0,
		.standby_recorvery_flag = 0,
};
void gy_public_init(void)
{
	gy_public_info.light_blink_finish_state = gy_light_blink_finish_back;
	//注意：此处不能再赋值为0，因为灯具状态初始化时该变量已经开始倒计时了，如果此处再将变量值赋值为0，则达不到sensor倒计时的功能
	//gy_public_info.occupied_time_count_down = 0;
	//gy_sensor_occupied_time_count_refresh();
	return;
}

void gy_save_dim_value_before_into_standby(void)
{
	sw_level_save_t *p_save = P_SW_LEVEL_SAVE(0, ST_TRANS_LIGHTNESS);
	st_transition_t *p_trans = P_ST_TRANS(0, ST_TRANS_LIGHTNESS);
	if(p_trans->target == LEVEL_OFF)
	{
		p_save->onoff = 0;
	}
	else
	{
		p_save->onoff = 1;
		p_save->last = gy_public_info.dim_value_before_into_standby-32768;
	}
	light_par_save(0);
	return;
}

















//***gy_vendor_model***//
GY_VENDOR_MODEL_INFO gy_vendor_model_info;

//***gy_vendor_model***//
void gy_vendor_model_init(void)
{
	gy_vendor_model_info.emergency_event_flag = gy_emergency_event_close;
	return;
}

//***gy_vendor_model***//
u8 gy_rcv_cmd_is_emergency(u8 *params, int par_len, mesh_cb_fun_par_t *cb_par)
{
	if(cb_par->op == gy_vendor_opcode_generic_get || cb_par->op == gy_vendor_opcode_generic_set || cb_par->op == gy_vendor_opcode_generic_set_unacknowledged)
	{
		if(params[0] == gy_vendor_property_id_emergency_event)
		{
			return 1;
		}
	}
	return 0;
}

//***gy_vendor_model***//
int gy_cb_vendor_power_get_handle(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	GY_FLASH_FIX_INFO gy_flash_fix_info_t;
	gy_flash_fix_read(&gy_flash_fix_info_t);
	//if(gy_flash_fix_info_t.light_power == 0xFF)//如果没有写入固定的最大功率，则默认最大功率40W
	if(gy_flash_fix_info_t.light_power == 0xFFFF)//如果没有写入固定的最大功率，则默认最大功率40W
	{
		gy_flash_fix_info_t.light_power = 40;
	}

#if(GY_DEV_TYPE == GY_DEV_100F)
	if(gy_flash_fix_info_t.light_power > 1000)//如果已经写入固定的最大功率，如果判断最大功率大于1000W，则以低字节计算功率
	{
		gy_flash_fix_info_t.light_power = (u16)((u8)gy_flash_fix_info_t.light_power);
	}
#endif

	//u16 x1 = GY_DIMMER_CURVE_TRANSFORMATION_MIN_VALUE;
	u16 x1 = GY_DIMMER_CURVE_TRANSFORMATION_1V_VALUE;
	u16 x2 = GY_DIMMER_CURVE_TRANSFORMATION_MAX_VALUE;
	u16 y1 = gy_flash_fix_info_t.light_power;//单位：100mv
	u16 y2 = gy_flash_fix_info_t.light_power*10;//单位：100mv
	u16 gy_instat_power_100mW_value = 0;

	if(light_res_sw_save[0].level[0].onoff)//如果灯是开着的
	{
		extern u8 level2lum(s16 level);
		u8 gy_level_100 = level2lum(light_res_sw_save[0].level[0].last);
		extern u32 get_pwm_cmp(u8 val, u8 lum);
		u32 gy_current_curve_value = 0;
		if(gy_flash_info.current_curve == gy_curve_linear)
		{
			gy_current_curve_value = (u32)gy_dimmer_curve_transformation_calculate((u32)gy_level_100*(255*256)/100);
		}
		else //if(gy_flash_info.current_curve == gy_curve_logarithmic || gy_flash_info.current_curve == gy_curve_square)
		{
			gy_current_curve_value = (u32)gy_dimmer_curve_transformation_calculate(gy_curve_map[gy_flash_info.current_curve-1][gy_level_100]);
		}
		u32 gy_pwm_level = ((u32)gy_current_curve_value * PWM_MAX_TICK) / (255*256);

		/*//该计算方式精度不高
		u16 x =  gy_pwm_level*100/PWM_MAX_TICK;
		gy_instat_power_100mW_value = (y1*(x2 - x)+y2*(x - x1))/(x2 - x1);
		*/

		#define x	gy_pwm_level*100/PWM_MAX_TICK
		/*u16*/u32 gy_instat_power_1mW_value = (y1*(100*x2 - 100*x)+y2*(100*x - 100*x1))/(x2 - x1);
		gy_instat_power_100mW_value = gy_instat_power_1mW_value/100 + ((gy_instat_power_1mW_value%100 >= 50) ? 1 : 0);

		if(gy_instat_power_100mW_value > y2 || gy_level_100 == 100)
		{//如果计算的功率超过最大写入得功率 或者 当前处于满亮度运行，则直接返回满功率值
			gy_instat_power_100mW_value = y2;
		}
	}
	else
	{
		gy_instat_power_100mW_value = 5;
	}

	mesh_tx_cmd_rsp(cb_par->op_rsp, (u8*)&gy_instat_power_100mW_value, 2, ele_adr_primary, cb_par->adr_src, 0, 0);
	return 0;
}

u16 lc_read_power_to_calculate_energy(void)
{
	GY_FLASH_FIX_INFO gy_flash_fix_info_t;
	gy_flash_fix_read(&gy_flash_fix_info_t);
	//if(gy_flash_fix_info_t.light_power == 0xFF)//如果没有写入固定的最大功率，则默认最大功率40W
	if(gy_flash_fix_info_t.light_power == 0xFFFF)//如果没有写入固定的最大功率，则默认最大功率40W
	{
		gy_flash_fix_info_t.light_power = 40;
	}

#if(GY_DEV_TYPE == GY_DEV_100F)
	if(gy_flash_fix_info_t.light_power > 1000)//如果已经写入固定的最大功率，如果判断最大功率大于1000W，则以低字节计算功率
	{
		gy_flash_fix_info_t.light_power = (u16)((u8)gy_flash_fix_info_t.light_power);
	}
#endif

	//u16 x1 = GY_DIMMER_CURVE_TRANSFORMATION_MIN_VALUE;
	u16 x1 = GY_DIMMER_CURVE_TRANSFORMATION_1V_VALUE;
	u16 x2 = GY_DIMMER_CURVE_TRANSFORMATION_MAX_VALUE;
	u16 y1 = gy_flash_fix_info_t.light_power;//单位：100mv
	u16 y2 = gy_flash_fix_info_t.light_power*10;//单位：100mv
	u16 gy_instat_power_100mW_value = 0;

	if(light_res_sw_save[0].level[0].onoff)//如果灯是开着的
	{
		extern u8 level2lum(s16 level);
		u8 gy_level_100 = level2lum(light_res_sw_save[0].level[0].last);
		extern u32 get_pwm_cmp(u8 val, u8 lum);
		u32 gy_current_curve_value = 0;
		if(gy_flash_info.current_curve == gy_curve_linear)
		{
			gy_current_curve_value = (u32)gy_dimmer_curve_transformation_calculate((u32)gy_level_100*(255*256)/100);
		}
		else //if(gy_flash_info.current_curve == gy_curve_logarithmic || gy_flash_info.current_curve == gy_curve_square)
		{
			gy_current_curve_value = (u32)gy_dimmer_curve_transformation_calculate(gy_curve_map[gy_flash_info.current_curve-1][gy_level_100]);
		}
		u32 gy_pwm_level = ((u32)gy_current_curve_value * PWM_MAX_TICK) / (255*256);

		/*//该计算方式精度不高
		u16 x =  gy_pwm_level*100/PWM_MAX_TICK;
		gy_instat_power_100mW_value = (y1*(x2 - x)+y2*(x - x1))/(x2 - x1);
		*/

		#define x	gy_pwm_level*100/PWM_MAX_TICK
		/*u16*/u32 gy_instat_power_1mW_value = (y1*(100*x2 - 100*x)+y2*(100*x - 100*x1))/(x2 - x1);
		gy_instat_power_100mW_value = gy_instat_power_1mW_value/100 + ((gy_instat_power_1mW_value%100 >= 50) ? 1 : 0);

		if(gy_instat_power_100mW_value > y2 || gy_level_100 == 100)
		{//如果计算的功率超过最大写入得功率 或者 当前处于满亮度运行，则直接返回满功率值
			gy_instat_power_100mW_value = y2;
		}
	}
	else
	{
		gy_instat_power_100mW_value = 5;
	}


	return gy_instat_power_100mW_value;
}


//***gy_vendor_model***//
int gy_cb_vendor_generic_get_handle(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	if(par_len >= 1)
	{
		switch(par[0])
		{
		case gy_vendor_property_id_dimming_curve:
		{
			u8 gy_send_data_t[2] = {gy_vendor_property_id_dimming_curve, gy_flash_info.current_curve};
			mesh_tx_cmd_rsp(cb_par->op_rsp, gy_send_data_t, 2, ele_adr_primary, cb_par->adr_src, 0, 0);
			break;
		}
		case gy_vendor_property_id_emergency_event:
		{
			u8 gy_send_data_t[2] = {gy_vendor_property_id_emergency_event, !!gy_vendor_model_info.emergency_event_flag};
			mesh_tx_cmd_rsp(cb_par->op_rsp, gy_send_data_t, 2, ele_adr_primary, cb_par->adr_src, 0, 0);
			break;
		}
		case gy_vendor_property_id_fast_provision_supplementary_info:
		{
			u8 lc_standby_and_als_flag=0;
			if(gy_i2c_info.sensor_type)
			{
				lc_standby_and_als_flag=0x13;
			}
			else
			{
				lc_standby_and_als_flag=0x10;
			}
			GY_FLASH_FIX_INFO gy_flash_fix_info_t;
			gy_flash_fix_read(&gy_flash_fix_info_t);
			if(gy_flash_fix_info_t.light_power == 0xFFFF)//如果没有写入固定的最大功率，则默认最大功率40W
			{
				gy_flash_fix_info_t.light_power = 40;
			}

		#if(GY_DEV_TYPE == GY_DEV_100F)
			if(gy_flash_fix_info_t.light_power > 1000)//如果已经写入固定的最大功率，如果判断最大功率大于1000W，则以低字节计算功率
			{
				gy_flash_fix_info_t.light_power = (u16)((u8)gy_flash_fix_info_t.light_power);
			}
		#endif
			u8 gy_send_data_t[] = {gy_vendor_property_id_fast_provision_supplementary_info,
					(u8)(GY_PID>>0), (u8)(GY_PID>>8), (u8)(GY_VID>>0), (u8)(GY_VID>>8),
					(u8)(gy_flash_fix_info_t.light_id>>0), (u8)(gy_flash_fix_info_t.light_id>>8), gy_i2c_info.sensor_type,lc_standby_and_als_flag,
					(u8)(gy_flash_fix_info_t.light_power>>0),(u8)(gy_flash_fix_info_t.light_power>>8)};
			mesh_tx_cmd_rsp(cb_par->op_rsp, gy_send_data_t, sizeof(gy_send_data_t), ele_adr_primary, cb_par->adr_src, 0, 0);
			break;
		}
		case gy_vendor_property_id_Startup_Voltage:
		{
			u8 gy_send_data_t[] = {gy_vendor_property_id_Startup_Voltage, (u8)(gy_flash_new_info.Startup_Voltage_100mv>>0), (u8)(gy_flash_new_info.Startup_Voltage_100mv>>8)};
			mesh_tx_cmd_rsp(cb_par->op_rsp, gy_send_data_t, sizeof(gy_send_data_t), ele_adr_primary, cb_par->adr_src, 0, 0);
			break;
		}
		case gy_vendor_property_id_Runtime_State_Register:
		{
			/*
			mesh_cmd_lightness_st_t gy_rsp = {0};
			extern void mesh_level_u16_st_rsp_par_fill(mesh_cmd_lightness_st_t *rsp, u8 idx, int st_trans_type);
			mesh_level_u16_st_rsp_par_fill(&gy_rsp, 0, ST_TRANS_LIGHTNESS);
			*/
			u16 gy_current_lightness_value_t = s16_to_u16(light_res_sw_save[0].level[0].last);
			u8 gy_send_data_t[] = {gy_vendor_property_id_Runtime_State_Register, ((!!light_res_sw_save[0].level[0].onoff)<<7), (u8)(gy_current_lightness_value_t>>0), (u8)(gy_current_lightness_value_t>>8)};
			mesh_tx_cmd_rsp(cb_par->op_rsp, gy_send_data_t, sizeof(gy_send_data_t), ele_adr_primary, cb_par->adr_src, 0, 0);
			break;
		}
		case gy_vendor_property_id_Fix_Properties:
		{
			GY_FLASH_FIX_INFO gy_flash_fix_info_t;
			gy_flash_fix_read(&gy_flash_fix_info_t);
		#if(GY_DEV_TYPE == GY_DEV_100F)
			if(gy_flash_fix_info_t.light_power == 0xFFFF)//如果没有写入固定的最大功率，则默认最大功率40W
			{
				gy_flash_fix_info_t.light_power = 40;
			}
			else if(gy_flash_fix_info_t.light_power > 1000)//如果已经写入固定的最大功率，如果判断最大功率大于1000W，则以低字节计算功率
			{
				gy_flash_fix_info_t.light_power = (u16)((u8)gy_flash_fix_info_t.light_power);
			}
		#endif
			u8 gy_send_data_t[] = {gy_vendor_property_id_Fix_Properties, (u8)(gy_flash_fix_info_t.light_id>>0), (u8)(gy_flash_fix_info_t.light_id>>8), (u8)(gy_flash_fix_info_t.light_power>>0), (u8)(gy_flash_fix_info_t.light_power>>8)};
			mesh_tx_cmd_rsp(cb_par->op_rsp, gy_send_data_t, sizeof(gy_send_data_t), ele_adr_primary, cb_par->adr_src, 0, 0);
			break;
		}
		case gy_vendor_property_id_total_energy:
		{

			lc_unicast_adress = cb_par->adr_src;
			lc_energy_delay = ((rand() % ((par[1]+par[2]*256)*100)) + 1);
			lc_energy_report_delay_start_flag=1;

			break;
		}
		}
	}
	return 0;
}

//***gy_vendor_model***//
int gy_cb_vendor_generic_set_handle(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	if(par_len >= 1)
	{
		switch(par[0])
		{
		case gy_vendor_property_id_dimming_curve:
		{
			if(par_len >=2 && par[1] >= 0 && par[1] <= 2)
			{
				gy_flash_info.current_curve = par[1];
				gy_flash_info_write();
			}
			u8 gy_send_data_t[2] = {gy_vendor_property_id_dimming_curve, gy_flash_info.current_curve};
			mesh_tx_cmd_rsp(cb_par->op_rsp, gy_send_data_t, 2, ele_adr_primary, cb_par->adr_src, 0, 0);
			break;
		}
		case gy_vendor_property_id_Startup_Voltage:
		{
			if(par_len >= 3)
			{
				gy_flash_new_info.Startup_Voltage_100mv = (par[1]) + (par[2]<<8);
				gy_flash_new_info_delay_write_start();
				gy_dim_pwm_min_duty_cycle_refresh_by_startup_voltage(gy_flash_new_info.Startup_Voltage_100mv);//根据启动电压设置（刷新）PWM最低调光占空比
				light_dim_refresh(0);
			}
			u8 gy_send_data_t[] = {gy_vendor_property_id_Startup_Voltage, (u8)(gy_flash_new_info.Startup_Voltage_100mv>>0), (u8)(gy_flash_new_info.Startup_Voltage_100mv>>8)};
			mesh_tx_cmd_rsp(cb_par->op_rsp, gy_send_data_t, sizeof(gy_send_data_t), ele_adr_primary, cb_par->adr_src, 0, 0);
			break;
		}
		case gy_vendor_property_id_Fix_Properties:
		{//fixture id (2B) + fixture power (2B)
		#if(GY_DEV_TYPE == GY_DEV_100F)
			//0x100F固件为了兼容之前已经出货的BC，收到APP设定功率参数的指令时忽略，保留默认功率参数
			GY_FLASH_FIX_INFO gy_flash_fix_info_t;
			gy_flash_fix_read(&gy_flash_fix_info_t);
			if(gy_flash_fix_info_t.light_power == 0xFFFF)//如果没有写入固定的最大功率，则默认最大功率40W
			{
				gy_flash_fix_info_t.light_power = 40;
			}
			else if(gy_flash_fix_info_t.light_power > 1000)//如果已经写入固定的最大功率，但是判断最大功率大于1000W，则以低字节计算功率
			{
				gy_flash_fix_info_t.light_power = (u16)((u8)gy_flash_fix_info_t.light_power);
			}
			u8 gy_send_data_t[] = {gy_vendor_property_id_Fix_Properties, (u8)(gy_flash_fix_info_t.light_id>>0), (u8)(gy_flash_fix_info_t.light_id>>8), (u8)(gy_flash_fix_info_t.light_power>>0), (u8)(gy_flash_fix_info_t.light_power>>8)};
			mesh_tx_cmd_rsp(cb_par->op_rsp, gy_send_data_t, sizeof(gy_send_data_t), ele_adr_primary, cb_par->adr_src, 0, 0);
		#else
			GY_FLASH_FIX_INFO gy_flash_fix_info_t;
			gy_flash_fix_info_t.light_id = par[1] + (par[2]<<8);
			gy_flash_fix_info_t.light_power = par[3] + (par[4]<<8);
			gy_flash_fix_write(&gy_flash_fix_info_t);
			mesh_tx_cmd_rsp(cb_par->op_rsp, par, 5, ele_adr_primary, cb_par->adr_src, 0, 0);
		#endif
			break;
		}
		case gy_vendor_property_id_daylight_calibration:
		{
			lc_daylight_calibrate_flag=1;
			lc_measure_lux = par[1]+par[2]*256;
			u8 gy_send_data_t[3] = {gy_vendor_property_id_daylight_calibration,par[1],par[2]};
			mesh_tx_cmd_rsp(cb_par->op_rsp, gy_send_data_t, 3, ele_adr_primary, cb_par->adr_src, 0, 0);
			break;
		}
		}
	}
	return 0;
}

//***gy_vendor_model***//
int gy_cb_vendor_generic_set_unacknowledged_handle(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	if(par_len >= 1)
	{
		switch(par[0])
		{
		case gy_vendor_property_id_identity:
		{
			gy_identity_status();
			break;
		}
		case gy_vendor_property_id_cct_delta:
		{
			break;
		}
		case gy_vendor_property_id_dimming_curve:
		{
			if(par_len >=2 && par[1] >= 0 && par[1] <= 2)
			{
				gy_flash_info.current_curve = par[1];
				gy_flash_info_write();
			}
			break;
		}
		case gy_vendor_property_id_probability_switch://概率开关灯
		{
			gy_probability_switch_handle(par, par_len);//概率开关灯
			break;
		}
		case gy_vendor_property_id_multi_group_control:
		{
			gy_Multi_group_Control_handle(par, par_len);//多组控制处理
			break;
		}
		case gy_vendor_property_id_emergency_event:
		{
			if(par[1] == 1)//紧急事件触发
			{
				cfg_led_event(GY_LED_EVENT_EMERGENCY_EVENT);
				gy_vendor_model_info.emergency_event_flag = gy_emergency_event_set;
				//gy_sensor_occupied_time_count_clear();
				gy_sensor_freeze_time_count_down = 0;
			}
			else if(par[1] == 0)//紧急事件关闭
			{
				//cfg_led_event(LED_EVENT_FLASH_STOP);
				//gy_vendor_model_info.emergency_event_flag = gy_emergency_event_close;
				if(gy_vendor_model_info.emergency_event_flag == gy_emergency_event_execute)
				{
					gy_vendor_model_info.emergency_event_flag = gy_emergency_event_forced_stop;
				}
			}
			break;
		}
		case gy_vendor_property_id_ONOFF_Toggle:
		{
			u8 gy_onoff_flag_t = gy_light_on;
			if(light_res_sw_save[0].level[0].onoff)//如果灯是开着的
			{
				gy_onoff_flag_t = gy_light_off;
			}
			gy_light_onoff_set_new(ele_adr_primary, gy_onoff_flag_t, par[1]);
			break;
		}
		case gy_vendor_property_id_Startup_Voltage:
		{
			if(par_len >= 3)
			{
				gy_flash_new_info.Startup_Voltage_100mv = (par[1]) + (par[2]<<8);
				gy_flash_new_info_delay_write_start();
				gy_dim_pwm_min_duty_cycle_refresh_by_startup_voltage(gy_flash_new_info.Startup_Voltage_100mv);//根据启动电压设置（刷新）PWM最低调光占空比
				light_dim_refresh(0);
			}
			break;
		}
		case gy_vendor_property_id_Factory_Reset:
		{
			if(par_len >= 2)
			{
				gy_factory_reset_method_data_handle(&par[1]);
			}
			break;
		}
		case gy_vendor_property_id_rc_control:
		{
			break;
		}
		case gy_vendor_property_id_rssi_test:
		{
			break;
		}
		case gy_vendor_property_id_power_meter_self_check:
		{
			break;
		}
		case gy_vendor_property_id_Fix_Properties:
		{//fixture id (2B) + fixture power (2B)
		#if(GY_DEV_TYPE == GY_DEV_100F)
			//0x100F固件为了兼容之前已经出货的BC，收到APP设定功率参数的指令时忽略，保留默认功率参数
		#else
			GY_FLASH_FIX_INFO gy_flash_fix_info_t;
			gy_flash_fix_info_t.light_id = par[1] + (par[2]<<8);
			gy_flash_fix_info_t.light_power = par[3] + (par[4]<<8);
			gy_flash_fix_write(&gy_flash_fix_info_t);
		#endif
			break;
		}
		case gy_vendor_property_id_daylight_calibration:
		{
			lc_daylight_calibrate_flag=1;
			lc_measure_lux = par[1]+par[2]*256;
			break;
		}
		}
	}
	return 0;
}

//***gy_vendor_model***//
int gy_cb_vendor_sensor_get_handle(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	if(par[0] == 0 || (par[0]*3 + 1) != par_len)
	{
		return -1;
	}
	u8 gy_sensor_status_data[par_len];
	memcpy(gy_sensor_status_data, par, par_len);
	u8 i = 0;
	for( ; i < gy_sensor_status_data[0]; i++)
	{
		if(gy_sensor_status_data[1 + 3*i] == 0)//查询sensor ID（sensor type）
		{
			u16 gy_sensor_type_t = gy_i2c_info.sensor_type;
			memcpy(&gy_sensor_status_data[2 + 3*i], (u8*)&gy_sensor_type_t, 2);
		}
		else if(gy_sensor_status_data[1 + 3*i] == 28)//查询原始光感数据
		{
			lc_als_read_flag=1;
			lc_unicast_adress = cb_par->adr_src;
		}
		else if(gy_sensor_status_data[1 + 3*i] == 18)//查询原始光感数据
		{

			lc_read_lux();
			gy_sensor_status_data[2]= (u8)lc_current_lux;
			gy_sensor_status_data[3]=(lc_current_lux>>8);
		}
		else if(gy_sensor_status_data[1 + 3*i] <= gy_sensor_property_unoccupied_onoff)
		{
			memcpy(&gy_sensor_status_data[2 + 3*i], (u8*)&(((u16*)&gy_flash_info.sensor_property)[gy_sensor_status_data[1 + 3*i] - 1]), 2);
		}
		else if(gy_sensor_status_data[1 + 3*i] >= gy_sensor_property_standby_guard_level && gy_sensor_status_data[1 + 3*i] <= gy_sensor_property_photocell_freeze_time)
		{
			memcpy(&gy_sensor_status_data[2 + 3*i], (u8*)&(((u16*)&gy_flash_info.sensor_property)[gy_sensor_status_data[1 + 3*i] - 1 - 3]), 2);
		}

	}
	mesh_tx_cmd_rsp(cb_par->op_rsp, gy_sensor_status_data, par_len, ele_adr_primary, cb_par->adr_src, 0, 0);
	return 0;
}

//***gy_vendor_model***//
int gy_cb_vendor_sensor_set_handle(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	if(par[0] == 0 || (par[0]*3 + 1) != par_len)
	{
		return -1;
	}
	u8 gy_sensor_status_data[par_len];
	memcpy(gy_sensor_status_data, par, par_len);
	u8 i = 0;
	for( ; i < gy_sensor_status_data[0]; i++)
	{
		if(gy_sensor_status_data[1 + 3*i] == gy_sensor_property_MW_Sensitivity)//设置人感灵敏度
		{
			if(gy_i2c_info.sensor_type == gy_i2c_sensor_2)//如果是微波传感器
			{
				gy_i2c_info.sensor_set_state = 0;
			}
		}
		if(gy_sensor_status_data[1 + 3*i] == gy_sensor_property_light_device_mode)
		{
			if((gy_sensor_status_data[2 + 3*i] == gy_device_mode_occupancy)&&(gy_flash_info.sensor_property.light_device_mode ==gy_device_mode_vacancy))
			{
				gy_public_info.occupied_time_count_down = 0;
				gy_sensor_freeze_time_count_down = 0;
			}
			else if((gy_sensor_status_data[2 + 3*i] == gy_device_mode_vacancy)&&(gy_flash_info.sensor_property.light_device_mode ==gy_device_mode_occupancy))
			{
				gy_public_info.occupied_time_count_down = 0;
				gy_sensor_freeze_time_count_down = 0;
			}
		}

		if(gy_sensor_status_data[1 + 3*i] <= gy_sensor_property_unoccupied_onoff)
		{
			memcpy((u8*)&(((u16*)&gy_flash_info.sensor_property)[gy_sensor_status_data[1 + 3*i] - 1]), &gy_sensor_status_data[2 + 3*i], 2);
		}
		else if(gy_sensor_status_data[1 + 3*i] >= gy_sensor_property_standby_guard_level && gy_sensor_status_data[1 + 3*i] <= gy_sensor_property_photocell_freeze_time)
		{
			memcpy((u8*)&(((u16*)&gy_flash_info.sensor_property)[gy_sensor_status_data[1 + 3*i] - 1 - 3]), &gy_sensor_status_data[2 + 3*i], 2);
		}

		   if((gy_sensor_status_data[1 + 3*i]==25)||(gy_sensor_status_data[1 + 3*i]==29)||(gy_sensor_status_data[1 + 3*i]==21))
			{
				lc_adjust_interval_count=(gy_flash_info.sensor_property.daylight_ctrl_freq-1);
				lc_pid_change_confirm=1;
			}
		   if((gy_sensor_status_data[1 + 3*i]==25)&&(gy_flash_info.sensor_property.daylight_harvesting!=0))
				{
				   lc_power_on_flag=1;
				   lc_pid_change_confirm=1;
					lc_adjust_interval_count=(gy_flash_info.sensor_property.daylight_ctrl_freq-1);

				}

		if(gy_sensor_status_data[1 + 3*i]==30)
		{
			lc_Kp = (gy_flash_info.sensor_property.pid_Kp_one_tenth*0.1);
		}
		else if(gy_sensor_status_data[1 + 3*i]==31)
		{
			lc_Ki = (gy_flash_info.sensor_property.pid_Ki_one_tenth*0.1);
		}
		else if((gy_sensor_status_data[1 + 3*i]==35)||(gy_sensor_status_data[1 + 3*i]==36))
		{
			lc_photocell_threshold_high=(u16)(gy_flash_info.sensor_property.photocell_threshold*gy_flash_info.sensor_property.photocell_threshold_ratio*0.1+0.5);
		}

	}
	//gy_flash_info_write();
	gy_flash_info_delay_write_start();
	if(gy_flash_info.sensor_property.light_device_mode == gy_device_mode_disable)
	{
		if(gy_public_info.occupied_time_count_down)
		{
			gy_public_info.occupied_time_count_down = 0;
		}
		gy_sensor_freeze_time_count_down = 0;
	}
	mesh_tx_cmd_rsp(cb_par->op_rsp, gy_sensor_status_data, par_len, ele_adr_primary, cb_par->adr_src, 0, 0);
	return 0;
}











//***gy_sensor***//
void gy_sensor_occupied_time_count_refresh(void)
{
	if(gy_flash_info.sensor_property.light_device_mode != gy_device_mode_disable)
	{
		if((!is_provision_success()) && (fast_prov.not_need_prov == 0))//如果未配网
		{
			gy_public_info.occupied_time_count_down = GY_UNPRO_SENSOR_OCCUPY_1S_TIME_COUNT_DOWN_MAX;
		}
		else
		{
			gy_public_info.occupied_time_count_down = (gy_flash_info.sensor_property.unoccupied_time_delay*60);
		}
	}
	gy_public_info.standby_recorvery_flag = 0;
	return;
}

//***gy_sensor***//
void gy_sensor_occupied_time_count_clear(void)
{
	if(gy_public_info.occupied_time_count_down)
	{
		gy_public_info.occupied_time_count_down = 0;
	}

	if(gy_public_info.standby_time_count_down)
	{
		gy_public_info.standby_time_count_down = 0;
	}
	return;
}

//***gy_sensor***//
//void gy_sensor_occupied_time_handle_ctl_light(int st_trans_type)
//{
//	st_transition_t *p_trans = P_ST_TRANS(0, st_trans_type);
//	switch(st_trans_type)
//	{
//	case ST_TRANS_LIGHTNESS:
//	{
//		u8 gy_dim_level_present_t = level2lum(p_trans->present);
//		u8 gy_dim_level_target_t = level2lum(p_trans->target);
//
//		if(gy_dim_level_present_t != gy_dim_level_target_t && gy_dim_level_target_t != 0)
//		{
//			gy_sensor_occupied_time_count_refresh();
//		}
//		else if(gy_dim_level_target_t == 0)
//		{
//			gy_sensor_occupied_time_count_clear();
//		}
//
//		break;
//	}
//	}
//	return;
//}

//***gy_sensor***//
void gy_sensor_occupied_handle(mesh_cb_fun_par_t *cb_par)
{
	if(gy_flash_info.sensor_property.light_device_mode == gy_device_mode_disable)
	{
		gy_public_info.occupied_time_count_down = 0;
		return;
	}

	if((!is_provision_success()) && (fast_prov.not_need_prov == 0))//如果未配网
	{
		if(gy_flash_info.sensor_property.light_device_mode == gy_device_mode_occupancy/* && gy_public_info.occupied_time_count_down == 0*/)
		{
			gy_light_self_dim_value_set(65535);//100%亮度
		}
		gy_public_info.occupied_time_count_down = GY_UNPRO_SENSOR_OCCUPY_1S_TIME_COUNT_DOWN_MAX;
	}
	else
	{
		if(gy_flash_info.sensor_property.light_device_mode == gy_device_mode_occupancy && gy_public_info.occupied_time_count_down == 0)
		{
			if(gy_flash_info.sensor_property.occupied_onoff == 0x00)
			{
				gy_light_info.state = gy_light_on;
				gy_light_self_onoff_set();
			}
			else if(gy_flash_info.sensor_property.occupied_onoff == 0x01)
			{
				gy_light_info.state = gy_light_off;
				gy_light_self_onoff_set();
			}
			else
			{
				if(gy_flash_info.sensor_property.photocell_onoff_control)
				{
					if((gy_flash_info.sensor_property.sensor_role!=1)||(lc_current_lux > gy_flash_info.sensor_property.photocell_threshold))
					{

					}
					else
					{
						if(gy_flash_info.sensor_property.occupied_level == 0)//执行开灯指令
						{

								if(gy_flash_info.sensor_property.photocell_onoff_control==2)
								{
									 gy_set_ctl_onoff(gy_get_sub(gy_sub_type_area), 1);
								}
								else
								{
									 gy_set_ctl_onoff(gy_get_sub(gy_sub_type_sub), 1);
								}


						}
						else//灯具变更到指定的亮度
						{
							if(gy_flash_info.sensor_property.photocell_onoff_control==2)
							{
								gy_light_dim_value_set_1000ms(gy_get_sub(gy_sub_type_area), gy_flash_info.sensor_property.occupied_level);
							}
							else
							{
								gy_light_dim_value_set_1000ms(gy_get_sub(gy_sub_type_sub), gy_flash_info.sensor_property.occupied_level);
							}
						}
					}
				}
				else
				{
					if(gy_flash_info.sensor_property.occupied_level == 0)//执行开灯指令
					{
						{
							gy_light_info.state = gy_light_off;
							gy_light_self_onoff_set();
						}

					}
					else//灯具变更到指定的亮度
					{
						gy_light_self_dim_value_set(gy_flash_info.sensor_property.occupied_level);
					}


				}
			}

		}

		if(gy_public_info.occupied_time_count_down != 0)
		{
			gy_public_info.occupied_time_count_down = (gy_flash_info.sensor_property.unoccupied_time_delay*60);
			/***注意此处倒计时刷新的位置***/
		}
	}

	return;
}

//***gy_sensor***//
void gy_sensor_unoccupied_handle(mesh_cb_fun_par_t *cb_par)
{
	/*
	if((gy_flash_info.sensor_property.device_mode == gy_sensor_device_mode_independent_sensing && cb_par->adr_src != ele_adr_primary) ||//如果当前处于独立模式，则不处理其他传感器发送过来的数据
			(gy_flash_info.sensor_property.device_mode == gy_sensor_device_mode_disable))//如果当前处于禁用传感器模式，则不处理所有传感器发送过来的数据
	{
		return;
	}
	gy_public_info.occupied_time_count_down = (gy_flash_info.sensor_property.unoccupied_time_delay*60);
	*/
	return;
}

//***gy_sensor***//
void gy_sensor_occupied_time_count_down_1s_tmer(u32 count)
{
	if(gy_vendor_model_info.emergency_event_flag != gy_emergency_event_close)
	{
		return;
	}
	if(gy_public_info.occupied_time_count_down)
	{
		if((!is_provision_success()) && (fast_prov.not_need_prov == 0))//如果未配网
		{
			if(gy_public_info.occupied_time_count_down == GY_UNPRO_SENSOR_OCCUPY_1S_TIME_STEP_ONE_COUNT)
			{
				gy_light_self_dim_value_set(655*GY_UNPRO_SENSOR_OCCUPY_STEP_ONE_DIM_LEVEL_100);
				gy_public_info.occupied_time_count_down = GY_UNPRO_SENSOR_OCCUPY_1S_TIME_STEP_ONE_COUNT;
			}
			else if(gy_public_info.occupied_time_count_down == 1)
			{
				gy_light_info.state = gy_light_on;
				gy_light_self_onoff_set();
				gy_public_info.occupied_time_count_down = 0;
				return;
			}
		}
		else
		{
			/*
			if(gy_public_info.occupied_time_count_down == 1)
			{
				if(gy_flash_info.sensor_property.unoccupied_onoff == 0x00)
				{
					gy_light_info.state = gy_light_on;
					gy_light_self_onoff_set();
				}
				else if(gy_flash_info.sensor_property.unoccupied_onoff == 0x01)
				{
					gy_light_info.state = gy_light_off;
					gy_light_self_onoff_set();
				}
				else
				{
					if(gy_flash_info.sensor_property.unoccupied_level == 0)
					{
						gy_light_info.state = gy_light_on;
						gy_light_self_onoff_set();
					}
					else
					{
						gy_light_self_dim_value_set(gy_flash_info.sensor_property.unoccupied_level);
					}
				}
				gy_public_info.occupied_time_count_down = 0;
				return;//必须添加，如果不添加，继续执行下面的代码会导致gy_public_info.occupied_time_count_down不为零
			}
			*/
			if(gy_public_info.occupied_time_count_down == 1)
			{
				gy_public_info.standby_time_count_down = 0;

				if(gy_flash_info.sensor_property.light_device_mode == gy_device_mode_vacancy || gy_flash_info.sensor_property.standby_guard_time == 0)
				{//如果当前为空缺模式或者占用模式时standby时间为0
					if(gy_flash_info.sensor_property.unoccupied_onoff == 0x00)
					{
						gy_light_info.state = gy_light_on;
						gy_light_self_onoff_set();
					}
					else if(gy_flash_info.sensor_property.unoccupied_onoff == 0x01)
					{
						gy_light_info.state = gy_light_off;
						gy_light_self_onoff_set();
					}
					else
					{
						if(gy_flash_info.sensor_property.unoccupied_level == 0)
						{
							gy_light_info.state = gy_light_on;
							gy_light_self_onoff_set();
						}
						else
						{
							if(light_res_sw_save[0].level[0].onoff!=0)
							{
							st_transition_t *p_trans = (&light_res_sw[0].trans[ST_TRANS_LIGHTNESS]);
							gy_public_info.dim_value_before_into_standby = p_trans->present+32768;
							}
							gy_light_self_dim_value_set(gy_flash_info.sensor_property.unoccupied_level);
							gy_save_dim_value_before_into_standby();
							gy_public_info.standby_recorvery_flag = 1;
						}
					}
				}
				else//如果当前为占用模式并且standby时间不为0
				{
					if(light_res_sw_save[0].level[0].onoff!=0)
					{
					st_transition_t *p_trans = (&light_res_sw[0].trans[ST_TRANS_LIGHTNESS]);
					gy_public_info.dim_value_before_into_standby = p_trans->present+32768;
					}
					gy_light_self_dim_value_set(gy_flash_info.sensor_property.standby_guard_level);
					gy_save_dim_value_before_into_standby();
					gy_public_info.standby_recorvery_flag = 1;
					gy_public_info.standby_time_count_down = gy_flash_info.sensor_property.standby_guard_time;
				}
				gy_public_info.occupied_time_count_down = 0;
				return;//必须添加，如果不添加，继续执行下面的代码会导致gy_public_info.occupied_time_count_down不为零
			}
		}
		gy_public_info.occupied_time_count_down--;
	}

	if(gy_public_info.occupied_time_count_down == 0 && gy_public_info.standby_time_count_down)
	{
		if(gy_flash_info.sensor_property.light_device_mode == gy_device_mode_occupancy)
		{
			gy_public_info.standby_time_count_down--;
			if(gy_public_info.standby_time_count_down == 0)
			{
				if(gy_flash_info.sensor_property.unoccupied_onoff == 0x00)
				{
					gy_light_info.state = gy_light_on;
					gy_light_self_onoff_set();
				}
				else if(gy_flash_info.sensor_property.unoccupied_onoff == 0x01)
				{
					gy_light_info.state = gy_light_off;
					gy_light_self_onoff_set();
				}
				else
				{
					if(gy_flash_info.sensor_property.unoccupied_level == 0)
					{
						gy_light_info.state = gy_light_on;
						gy_light_self_onoff_set();
					}
					else
					{
						gy_light_self_dim_value_set(gy_flash_info.sensor_property.unoccupied_level);
					}
				}
				gy_save_dim_value_before_into_standby();
				gy_public_info.occupied_time_count_down = 0;
				gy_public_info.standby_recorvery_flag = 1;
			}
		}
		else
		{
			gy_public_info.standby_time_count_down = 0;
		}
	}

	return;
}

//***gy_sensor***//
void gy_sensor_sig_mesh_property_set(u16 property_id, u8* property_value)
{
	if(property_id == LC_PROP_ID_TimeOccupancyDelay)
	{
		gy_flash_info.sensor_property.unoccupied_time_delay = ((u32)*property_value & 0x00ffffff)/1000/60;//ms(毫秒) -> min(分钟)
		gy_flash_info_write();
	}
	return;
}

//***gy_sensor***//
void gy_sensor_status_send(u8 sensor_state)
{
	u16 gy_dev_addr_t = 0x0000;
	switch(gy_flash_info.sensor_property.sensor_device_mode)
	{
	case gy_device_mode_disable:
	{
		return;
		break;
	}
	case gy_device_mode_occupancy:
	case gy_device_mode_vacancy:
	{
		gy_dev_addr_t = gy_get_sub(gy_sub_type_area);
		break;
	}
	case gy_device_mode_independent_sensing:
	{
		gy_dev_addr_t = gy_get_sub(gy_sub_type_sub);
		if(gy_dev_addr_t == 0xffff)//如果没有sub，则只对自己生效
		{
			gy_dev_addr_t = ele_adr_primary;
		}
		break;
	}
	}
	if((!is_provision_success()) && (fast_prov.not_need_prov == 0))//如果未配网
	{
		gy_sensor_occupied_handle(NULL);
	}
	else
	{
		u8 gy_uart_send_data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, (u8)gy_dev_addr_t, (u8)(gy_dev_addr_t>>8), 0x52, 0x03, 0x42, 0x00, sensor_state};
		mesh_bulk_cmd((mesh_bulk_cmd_par_t *)(gy_uart_send_data), sizeof(gy_uart_send_data));
	}
	return;
}

//***gy_sensor***//
void gy_sensor_close(void)
{
	if(gy_flash_info.sensor_property.sensor_device_mode != gy_device_mode_disable
			|| gy_flash_info.sensor_property.light_device_mode != gy_device_mode_disable)
	{
		gy_flash_info.sensor_property.sensor_device_mode = gy_device_mode_disable;
		gy_flash_info.sensor_property.light_device_mode = gy_device_mode_disable;
		//gy_flash_info_delay_write_start();
		gy_flash_info_write();
	}

	if(gy_public_info.occupied_time_count_down)
	{
		gy_public_info.occupied_time_count_down = 0;
	}

	return;
}














//***gy_flash***//
GY_FLASH_INFO gy_flash_info;
u16 gy_flash_79000_write_10ms_time_count_down = 0;//倒计时结束之后再将flash数据存储到flash中
void gy_flash_init(void)
{
	flash_read_page(GY_FLASH_INFO_ADDR, sizeof(GY_FLASH_INFO), (u8*)&gy_flash_info);
	if(gy_flash_info.current_curve == 0xFF || (gy_flash_info.current_curve != gy_curve_linear &&
			gy_flash_info.current_curve != gy_curve_logarithmic && gy_flash_info.current_curve != gy_curve_square))
	{
		gy_flash_info.current_curve = gy_curve_square/*gy_curve_logarithmic*/;
		gy_flash_info.sensor_property.sensor_device_mode = gy_device_mode_independent_sensing;
		gy_flash_info.sensor_property.unoccupied_time_delay = 20;
		gy_flash_info.sensor_property.occupied_level = 0;
		gy_flash_info.sensor_property.unoccupied_level = 0;
		gy_flash_info.sensor_property.MW_Sensitivity = GY_MW_SENSITIVITY_VALUE_DEFAULT;
		gy_flash_info.sensor_property.ALS_Threshold = GY_ALS_THREASHOLD_VALUE_DEFAULT;
		gy_flash_info.sensor_property.light_device_mode = gy_device_mode_occupancy;
		gy_flash_info.sensor_property.override_timeout = 30;
		gy_flash_info.sensor_property.sensor_freeze_time = 20;
		gy_flash_info.sensor_property.occupied_onoff = 0xFFFF;
		gy_flash_info.sensor_property.unoccupied_onoff = 0xFFFF;
		gy_flash_info.sensor_property.standby_guard_level = 19660;
		gy_flash_info.sensor_property.standby_guard_time = 0;

		gy_flash_info.sensor_property.debounce_time=30;

		gy_flash_info.sensor_property.daylight_open_high=70;
		gy_flash_info.sensor_property.daylight_open_low=30;
		gy_flash_info.sensor_property.target_light_level=800;
		gy_flash_info.sensor_property.daylight_ctrl_freq=60;
		gy_flash_info.sensor_property.occupied_action=0;
		gy_flash_info.sensor_property.vacancy_action=0;
		gy_flash_info.sensor_property.daylight_harvesting=0;
		gy_flash_info.sensor_property.daylight_threshold_high=800;
		gy_flash_info.sensor_property.daylight_threshold_low=300;
		gy_flash_info.sensor_property.light_intensity_extra=0;
		gy_flash_info.sensor_property.sensor_role=0;

		gy_flash_info.sensor_property.pid_Kp_one_tenth=70;
		gy_flash_info.sensor_property.pid_Ki_one_tenth=58;

		gy_flash_info.sensor_property.photocell_onoff_control=0;
		gy_flash_info.sensor_property.subid_for_lclcsense_sh=1;
		gy_flash_info.sensor_property.coefficient_of_als_calibration=10;

		gy_flash_info.sensor_property.photocell_threshold=300;
		gy_flash_info.sensor_property.photocell_threshold_ratio=13;
		gy_flash_info.sensor_property.photocell_freeze_time=2;
		//gy_flash_info_write();
	}
	if(gy_flash_info.sensor_property.standby_guard_time == 0xFFFF)
	{
		gy_flash_info.sensor_property.standby_guard_level = 19660;
		gy_flash_info.sensor_property.standby_guard_time = 0;
	}

//	if(gy_flash_info.sensor_property.subid_for_lclcsense_sh==0xffff)
//		gy_flash_info.sensor_property.subid_for_lclcsense_sh=1;
	if(gy_flash_info.sensor_property.daylight_open_high==0xffff)
	{
		gy_flash_info.sensor_property.debounce_time=30;

		gy_flash_info.sensor_property.daylight_open_high=70;
		gy_flash_info.sensor_property.daylight_open_low=30;
		gy_flash_info.sensor_property.target_light_level=800;
		gy_flash_info.sensor_property.daylight_ctrl_freq=60;
		gy_flash_info.sensor_property.occupied_action=0;
		gy_flash_info.sensor_property.vacancy_action=0;
		gy_flash_info.sensor_property.daylight_harvesting=0;
		gy_flash_info.sensor_property.daylight_threshold_high=800;
		gy_flash_info.sensor_property.daylight_threshold_low=300;
		gy_flash_info.sensor_property.light_intensity_extra=0;
		gy_flash_info.sensor_property.sensor_role=0;

		gy_flash_info.sensor_property.pid_Kp_one_tenth=70;
		gy_flash_info.sensor_property.pid_Ki_one_tenth=58;

		gy_flash_info.sensor_property.photocell_onoff_control=0;
		gy_flash_info.sensor_property.subid_for_lclcsense_sh=1;
		gy_flash_info.sensor_property.coefficient_of_als_calibration=10;

		gy_flash_info.sensor_property.photocell_threshold=300;
		gy_flash_info.sensor_property.photocell_threshold_ratio=13;
		gy_flash_info.sensor_property.photocell_freeze_time=2;
	}

	gy_flash_info.fast_power_onoff_flag++;
	gy_flash_info_write();
	return;
}

//***gy_flash***//
void gy_flash_info_delay_write_start(void)
{
	gy_flash_79000_write_10ms_time_count_down = GY_FLASH_79000_WRITE_10MS_COUNT_MAX;
	return;
}

//***gy_flash***//
void gy_flash_info_write(void)
{
	flash_erase_sector(GY_FLASH_INFO_ADDR);
	flash_write_page(GY_FLASH_INFO_ADDR, sizeof(GY_FLASH_INFO), (u8*)&gy_flash_info);
	return;
}

//***gy_flash***//
//void gy_flash_reset(void)
//{
//	gy_flash_info.current_curve = gy_curve_logarithmic;
//	gy_flash_info_write();
//	return;
//}

//***gy_flash***//
void gy_flash_1s_timer(u32 count)
{
	if(count == 1)
	{
		if((!is_provision_success()) && (fast_prov.not_need_prov == 0))//恢复出厂设置（未配网）时一定使用对数调光，解决覆盖烧录可能遇到线性调光的问题
		{
			if(gy_flash_info.current_curve != gy_curve_square/*gy_curve_logarithmic*/)
			{
				gy_flash_info.current_curve = gy_curve_square/*gy_curve_logarithmic*/;
				gy_flash_info_write();
			}
		}
	}

	if(count == 3)
	{
		if(gy_flash_info.fast_power_onoff_flag)
		{
			gy_flash_info.fast_power_onoff_flag = 0;
			gy_flash_info_write();
		}
	}

	if((!is_provision_success()) && (fast_prov.not_need_prov == 0) && (fast_prov.not_need_prov == 0) && count >= 10 && count <= 60)//如果未配网
	{
		if(gy_i2c_info.sensor_type == gy_i2c_sensor_none && gy_flash_info.sensor_property.light_device_mode != gy_device_mode_disable)
		{
			/*
			gy_flash_info.sensor_property.sensor_device_mode = gy_device_mode_disable;
			gy_flash_info.sensor_property.light_device_mode = gy_device_mode_disable;
			gy_public_info.occupied_time_count_down = 0;
			gy_flash_info_write();
			*/
			gy_sensor_close();
		}
	}
	return;
}

//***gy_flash***//
void gy_flash_10ms_timer(u32 count)
{
	if(gy_flash_79000_write_10ms_time_count_down)
	{
		if(gy_flash_79000_write_10ms_time_count_down == 1)
		{
			gy_flash_info_write();
		}
		gy_flash_79000_write_10ms_time_count_down--;
	}
	return;
}

//***gy_flash***//
void gy_flash_factory_reset(void)
{
	gy_i2c_sensor_led_off();
	flash_erase_sector(GY_FLASH_INFO_ADDR);
	return;
}















//***gy_flash_fix***//
void gy_flash_fix_read(GY_FLASH_FIX_INFO *flash_fix_info)
{
	flash_read_page(GY_FLASH_FIX_INFO_ADDR, sizeof(GY_FLASH_FIX_INFO), (u8*)flash_fix_info);
	return;
}

//***gy_flash_fix***//
void gy_flash_fix_write(GY_FLASH_FIX_INFO *flash_fix_info)
{
	flash_erase_sector(GY_FLASH_FIX_INFO_ADDR);
	flash_write_page(GY_FLASH_FIX_INFO_ADDR, sizeof(GY_FLASH_FIX_INFO), (u8*)flash_fix_info);
	return;
}

//***gy_flash_fix***//
void gy_flash_fix_factory_reset(void)
{
	GY_FLASH_FIX_INFO gy_flash_fix_info_t;
	gy_flash_fix_read(&gy_flash_fix_info_t);
	if(gy_flash_fix_info_t.light_id == 0xFFFF && gy_flash_fix_info_t.light_power != 0xFFFF)
	{//如果flash_fix_info.light_power变量是用户使用APP自定义写入的值，则恢复出厂时将该flash存储数据清除。
		flash_erase_sector(GY_FLASH_FIX_INFO_ADDR);
	}
	return;
}










//***gy_flash_new***//
u16 gy_flash_new_info_write_10ms_count_down = 0;//倒计时结束之后再将flash数据存储到flash中
GY_FLASH_NEW_INFO gy_flash_new_info;

//***gy_flash_new***//
void gy_flash_new_init(void)
{
	gy_flash_new_info_read();
	if(gy_flash_new_info.Startup_Voltage_100mv == 0xFFFF)
	{
		gy_flash_new_info.Startup_Voltage_100mv = 10;
	}
	gy_dim_pwm_min_duty_cycle_refresh_by_startup_voltage(gy_flash_new_info.Startup_Voltage_100mv);

	return;
}

void gy_flash_will_not_erase_init(void)
{
	gy_flash_will_not_erase_info_read();
	if((u32)gy_flash_will_not_erase_info.lc_total_energy == 0xffffffff)
	{
		LOG_USER_MSG_INFO(0,0,"高包包",0);
		gy_flash_will_not_erase_info.lc_total_energy = 0;
	}
	LOG_USER_MSG_INFO(0,0,"gy_flash_will_not_erase_info.lc_total_energy：0x%x",gy_flash_will_not_erase_info.lc_total_energy);
}

//***gy_flash_new***//
void gy_flash_new_info_delay_write_start(void)
{
	gy_flash_new_info_write_10ms_count_down = GY_FLASH_NEW_INFO_WRITE_10MS_COUNT_MAX;
	return;
}

//***gy_flash_new***//
void gy_flash_new_info_read(void)
{
	flash_read_page(GY_FLASH_NEW_INFO_ADDR, sizeof(GY_FLASH_NEW_INFO), (u8*)&gy_flash_new_info);
	return;
}

//***gy_flash_new***//
void gy_flash_new_info_write(void)
{
	flash_erase_sector(GY_FLASH_NEW_INFO_ADDR);
	flash_write_page(GY_FLASH_NEW_INFO_ADDR, sizeof(GY_FLASH_NEW_INFO), (u8*)&gy_flash_new_info);
	return;
}

void gy_flash_will_not_erase_info_read(void)
{
	flash_read_page(GY_FLASH_WILL_NOT_ERASE_INFO_ADDR, sizeof(GY_FLASH_WILL_NOT_ERASE_INFO_ADDR), (u8*)&gy_flash_will_not_erase_info);
	return;
}

//***gy_flash_new***//
void gy_flash_will_not_erase_info_write(void)
{
	flash_erase_sector(GY_FLASH_WILL_NOT_ERASE_INFO_ADDR);
	flash_write_page(GY_FLASH_WILL_NOT_ERASE_INFO_ADDR, sizeof(GY_FLASH_WILL_NOT_ERASE_INFO_ADDR), (u8*)&gy_flash_will_not_erase_info);
	return;
}

//***gy_flash_new***//
void gy_flash_new_1s_timer(u32 count)
{
	return;
}

//***gy_flash_new***//
void gy_flash_new_10ms_timer(u32 count)
{
	if(gy_flash_new_info_write_10ms_count_down)
	{
		gy_flash_new_info_write_10ms_count_down--;
		if(gy_flash_new_info_write_10ms_count_down == 0)
		{
			gy_flash_new_info_write();
		}
	}
	return;
}

//***gy_flash_new***//
void gy_flash_new_factory_reset(void)
{
	flash_erase_sector(GY_FLASH_NEW_INFO_ADDR);
	return;
}















//***gy_FCT_opcode_handle***//
GY_FCT_OPCODE_HANDLE_INFO gy_fct_opcode_handle_info;

//***gy_FCT_opcode_handle***//
void gy_FCT_init(void)
{
	gy_fct_opcode_handle_info.current_test_step = gy_test_step_idle;
	gy_fct_opcode_handle_info.sensor_test_type = gy_sensor_test_idle;
	return;
}

//***gy_FCT_opcode_handle***//
void gy_FCT_opcode_handle(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par)
{
	switch(par[0])
	{
	case gy_FCT_opcode_RSSI_test:
	{
		if(gy_fct_opcode_handle_info.current_test_step == gy_test_step_1 && gy_fct_opcode_handle_info.test_result)
		{
			if(gy_fct_opcode_handle_info.rcv_rssi >= (s8)par[2])//rssi测试通过：控制器上绿灯常亮
			{
				gy_set_led_r_state(gy_led_off);
				gy_set_led_g_state(gy_led_on);
				gy_fct_opcode_handle_info.test_result = 1;
				//gy_fct_opcode_handle_info.current_test_step = gy_test_step_2;
			}
			else//rssi测试失败：控制器上红灯常亮；【0~10V】输出 1V；蓝牙控制器停止后续测试
			{
				gy_set_led_r_state(gy_led_on);
				gy_set_led_g_state(gy_led_off);
				gy_light_self_dim_value_set(655);
				gy_fct_opcode_handle_info.test_result = 0;
			}
		}
		break;
	}
	case gy_FCT_opcode_power_metering_test:
	{
		GY_FLASH_FIX_INFO gy_flash_fix_info_t;
		gy_flash_fix_read(&gy_flash_fix_info_t);
		//if(gy_flash_fix_info_t.light_power == 0xFF)//如果没有写入固定的最大功率
		if(gy_flash_fix_info_t.light_power == 0xFFFF)//如果没有写入固定的最大功率
		{
			gy_fct_opcode_handle_info.current_test_step = gy_test_step_1;
			gy_fct_opcode_handle_info.test_result = 1;
			gy_set_led_r_state(gy_led_on);
			gy_set_led_g_state(gy_led_off);
			gy_light_self_dim_value_set(0);
			gy_fct_opcode_handle_info.test_result = 0;
		}
		else
		{
			u8 level = par[2];
			u16 power_min = par[3] + ((u16)(par[4])<<8);
			u16 power_max = par[5] + ((u16)(par[6])<<8);
			if(level == 100 && (gy_flash_fix_info_t.light_power*10) >= power_min && (gy_flash_fix_info_t.light_power*10) <= power_max)
			{
				gy_fct_opcode_handle_info.current_test_step = gy_test_step_1;
				gy_fct_opcode_handle_info.test_result = 1;
				gy_set_led_r_state(gy_led_off);
				gy_set_led_g_state(gy_led_on);
				gy_light_self_dim_value_set(655*30);
				gy_fct_opcode_handle_info.test_result = 1;
			}
			else
			{
				gy_fct_opcode_handle_info.current_test_step = gy_test_step_1;
				gy_fct_opcode_handle_info.test_result = 1;
				gy_set_led_r_state(gy_led_on);
				gy_set_led_g_state(gy_led_off);
				gy_light_self_dim_value_set(0);
				gy_fct_opcode_handle_info.test_result = 0;
			}
		}
		break;
	}
	case gy_FCT_opcode_FCT_mode:
	{
		if(par[2] == 1)//进入产测模式:打开 relay 功能,【0~10V】输出 10V,关闭控制器上所有 LED 灯
		{
			gy_fct_opcode_handle_info.current_test_step = gy_test_step_1;
			gy_fct_opcode_handle_info.test_result = 1;
			gy_light_self_dim_value_set(0xFFFF);
			gy_set_led_r_state(gy_led_off);
			gy_set_led_g_state(gy_led_off);
		}
		else if(par[2] == 0)//退出产测模式
		{
			gy_FCT_init();
			gy_set_led_r_state(gy_led_off);
			gy_set_led_g_state(gy_led_off);
		}
		break;
	}
	case gy_FCT_opcode_sensor_test:
	{
		if(par[2] == 0)//RJ09通信测试
		{
			if(!is_provision_success() && fast_prov.not_need_prov == 0 && gy_fct_opcode_handle_info.current_test_step != gy_test_step_idle && gy_fct_opcode_handle_info.test_result == 1)
			{
				gy_i2c_check_sensor_type();
				if(gy_i2c_info.sensor_type == gy_i2c_sensor_none)//如果没有检测到传感器
				{
					if(gy_flash_info.current_curve == gy_curve_logarithmic)//对数调光
					{
						gy_light_self_dim_value_set((86*655));//输出5V
					}
					else if(gy_flash_info.current_curve == gy_curve_square)//平方调光
					{
						gy_light_self_dim_value_set((73*655));//输出5V
					}
					else//线性调光
					{
						gy_light_self_dim_value_set((50*655));//输出5V
					}
					gy_set_led_r_state(gy_led_on);
					gy_set_led_g_state(gy_led_off);
					gy_fct_opcode_handle_info.test_result = 0;

				}
				else//如果检测到传感器
				{
					gy_light_self_dim_value_set(65535);//输出10V
					gy_set_led_r_state(gy_led_off);
					gy_set_led_g_state(gy_led_on);
				}
			}
		}
		else if(par[2] == 1)//人感传感器测试
		{
			gy_fct_opcode_handle_info.sensor_test_type = gy_sensor_test_motion;
			gy_fct_opcode_handle_info.current_test_step = gy_test_step_1;
			gy_fct_opcode_handle_info.test_result = 1;
			gy_set_led_r_state(gy_led_off);
			gy_set_led_g_state(gy_led_on);
			gy_light_self_dim_value_set(0);
			gy_flash_info.sensor_property.unoccupied_time_delay = par[3];
			gy_flash_info.sensor_property.MW_Sensitivity = par[4] + ((u16)par[5]<<8);
		}
		else if(par[2] == 2)//光感传感器测试
		{
			gy_fct_opcode_handle_info.sensor_test_type = gy_sensor_test_ALS;
			gy_fct_opcode_handle_info.current_test_step = gy_test_step_1;
			gy_fct_opcode_handle_info.test_result = 1;
			gy_set_led_r_state(gy_led_off);
			gy_set_led_g_state(gy_led_on);
			gy_light_self_dim_value_set(0);
			gy_flash_info.sensor_property.ALS_Threshold = par[3] + ((u16)par[4]<<8);
		}
		break;
	}
	case gy_FCT_opcode_set_fixture_properties:
	{//fixture id (2B) + fixture power (2B)
		GY_FLASH_FIX_INFO gy_flash_fix_info_t;
		gy_flash_fix_info_t.light_id = par[2] + (par[3]<<8);
		gy_flash_fix_info_t.light_power = par[4] + (par[5]<<8);
		gy_flash_fix_write(&gy_flash_fix_info_t);
		break;
	}
	case gy_FCT_opcode_get_i2c_data:
	{
		u8 gy_buff_t[5];
		memset(gy_buff_t, 0xFF, sizeof(gy_buff_t));
		gpio_set_func(GY_I2C_SDA_PIN, AS_GPIO);
		gpio_set_output_en(GY_I2C_SDA_PIN, 1);
		gpio_set_input_en(GY_I2C_SDA_PIN, 0);
		GY_H_SUN_SDA(1);
		gpio_set_func(GY_I2C_SCK_PIN, AS_GPIO);
		gpio_set_output_en(GY_I2C_SCK_PIN, 1);
		gpio_set_input_en(GY_I2C_SCK_PIN, 0);
		GY_H_SUN_SCK_H;
		gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x01, &gy_buff_t[0], 1);
		gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x02, &gy_buff_t[1], 1);
		gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x03, &gy_buff_t[2], 1);
		gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x04, &gy_buff_t[3], 1);
		gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x05, &gy_buff_t[4], 1);

		u8 gy_uart_send_data[] = {gy_FCT_opcode_get_i2c_data, 0x00, gy_buff_t[0], gy_buff_t[1], gy_buff_t[2], gy_buff_t[3], gy_buff_t[4]};
		mesh_tx_cmd_rsp(0x8203, (u8 *)gy_uart_send_data, sizeof(gy_uart_send_data), ele_adr_primary, cb_par->adr_src, 0, 0);
		break;
	}
	case gy_FCT_opcode_i2c_chip_reset:
	{
		gpio_set_func(GY_I2C_SDA_PIN, AS_GPIO);
		gpio_set_output_en(GY_I2C_SDA_PIN, 1);
		gpio_set_input_en(GY_I2C_SDA_PIN, 0);
		GY_H_SUN_SDA(1);
		gpio_set_func(GY_I2C_SCK_PIN, AS_GPIO);
		gpio_set_output_en(GY_I2C_SCK_PIN, 1);
		gpio_set_input_en(GY_I2C_SCK_PIN, 0);
		GY_H_SUN_SCK_H;
		gy_h_sun_i2c_write(GY_H_SUN_DEV_ADDR, 0x06, NULL, 0);
		gy_h_sun_i2c_write(GY_H_SUN_DEV_ADDR, 0x06, NULL, 0);
		gy_h_sun_i2c_write(GY_H_SUN_DEV_ADDR, 0x06, NULL, 0);
		gy_h_sun_i2c_write(GY_H_SUN_DEV_ADDR, 0x06, NULL, 0);
		u8 gy_uart_data_t[] = {0x88, 0x99, 0x66};
		mesh_tx_cmd_rsp(0x8203, (u8 *)gy_uart_data_t, sizeof(gy_uart_data_t), ele_adr_primary, cb_par->adr_src, 0, 0);
		break;
	}
	case gy_FCT_opcode_read_sensor_count_down:
	{
		mesh_tx_cmd_rsp(0x8203, (u8 *)&gy_public_info.occupied_time_count_down, 2, ele_adr_primary, cb_par->adr_src, 0, 0);
		break;
	}
	}
	return;
}





























//***gy_sub(gy_group)***//
void gy_set_group(u16 sub_group)//设置分组
{
//	u8 gy_set_sub_data_t[] = {(u8)ele_adr_primary,(u8)(ele_adr_primary >> 8), (u8)sub_group,(u8)(sub_group >> 8), 0x00, 0x10};
//	mesh_cmd_sig_cfg_model_sub_set(gy_set_sub_data_t, sizeof(gy_set_sub_data_t), NULL);
	u8 gy_uart_send_data[] = { 0x00,0x00,0x00,0x00,0x00,0x01,(u8)ele_adr_primary,(u8)(ele_adr_primary >> 8),0x80,0x1B,
			(u8)ele_adr_primary,(u8)(ele_adr_primary >> 8), (u8)sub_group,(u8)(sub_group >> 8), 0x00, 0x10 };
	mesh_bulk_cmd((mesh_bulk_cmd_par_t *)gy_uart_send_data, sizeof(gy_uart_send_data));
	return;
}

//***gy_sub(gy_group)***//
void gy_set_default_group(void)//设置默认分组
{
	gy_set_group(GY_DEFAULT_GROUP);
	return;
}

//***gy_sub(gy_group)***//
void gy_set_default_group_1s_timer(u32 count)
{
	if(gy_public_info.light_blink_finish_state > gy_light_blink_finish_provision)
	{
		gy_public_info.light_blink_finish_state++;
		if(gy_public_info.light_blink_finish_state >= gy_light_blink_finish_add_defualt_group)
		{
			gy_set_default_group();
			gy_public_info.light_blink_finish_state = gy_light_blink_finish_back;
		}
	}
	return;
}

//***gy_sub(gy_group)***//
u8 gy_is_sub(u16 sub_addr)//判断是否分配到该组播地址
{
	u8 gy_i_t;
	for(gy_i_t = 0; gy_i_t < SUB_LIST_MAX; gy_i_t++)
	{
		if(model_sig_g_onoff_level.onoff_srv[0].com.sub_list[gy_i_t] == sub_addr)
		{
			return 1;
		}
	}
	return 0;
}














//***gy_scene***//
void gy_three_defult_scene_init(void)
{
	if(model_sig_scene.data[0][0].id == 0)
	{
		gy_scene_recovery(gy_scene_No1);
		gy_scene_recovery(gy_scene_No2);
		gy_scene_recovery(gy_scene_No3);
	}
	return;
}

//***gy_scene***//
void gy_three_defult_scene_set_10ms_timer(u32 count)
{
	if(count%100 == 0)
	{
		if(is_provision_success() || fast_prov.not_need_prov)
		{
			gy_three_defult_scene_init();
		}
	}
	return;
}

//***gy_scene***//
void gy_scene_recovery(u16 scene_No)
{
	switch(scene_No)
	{
	case gy_scene_No1:
	{
		model_sig_scene.data[0][0].id = gy_scene_No1;
		model_sig_scene.data[0][0].lightness_s16 = gy_scene_No1_lightness_value;//1%
		//model_sig_scene.data[0][0].temp_s16 = gy_scene_No1_temp_value;//2700K
		//model_sig_scene.data[0][0].delta_uv_s16 = 0;
		//model_sig_scene.data[0][0].ct_flag = ct_flag;
		extern void scene_active_set(int idx, u16 scene_id, int trans_flag);
		//CB_NL_PAR_NUM_2(p_nl_get_vendor_scene_data, gy_scene_No1, p_save->nl_data);
		scene_active_set(0, gy_scene_No1, 0);
		break;
	}
	case gy_scene_No2:
	{
		model_sig_scene.data[0][1].id = gy_scene_No2;
		model_sig_scene.data[0][1].lightness_s16 = gy_scene_No2_lightness_value;//50%
		//model_sig_scene.data[0][1].temp_s16 = gy_scene_No2_temp_value;//6500K
		//model_sig_scene.data[0][1].delta_uv_s16 = 0;
		//model_sig_scene.data[0][1].ct_flag = ct_flag;
		extern void scene_active_set(int idx, u16 scene_id, int trans_flag);
		//CB_NL_PAR_NUM_2(p_nl_get_vendor_scene_data, 2, p_save->nl_data);
		scene_active_set(0, gy_scene_No2, 0);
		break;
	}
	case gy_scene_No3:
	{
		model_sig_scene.data[0][2].id = gy_scene_No3;
		model_sig_scene.data[0][2].lightness_s16 = gy_scene_No3_lightness_value;//100%
		//model_sig_scene.data[0][2].temp_s16 = gy_scene_No3_temp_value;//4000K
		//model_sig_scene.data[0][2].delta_uv_s16 = 0;
		//model_sig_scene.data[0][2].ct_flag = ct_flag;
		extern void scene_active_set(int idx, u16 scene_id, int trans_flag);
		//CB_NL_PAR_NUM_2(p_nl_get_vendor_scene_data, 3, p_save->nl_data);
		scene_active_set(0, gy_scene_No3, 0);
		break;
	}
	}
	return;
}

//***gy_scene***//
u8 gy_scene_del_handle(u16 scene_No)
{
	switch(scene_No)
	{
	case gy_scene_No1:
	{
		gy_scene_recovery(scene_No);
		return 1;
		break;
	}
	case gy_scene_No2:
	{
		gy_scene_recovery(scene_No);
		return 1;
		break;
	}
	case gy_scene_No3:
	{
		gy_scene_recovery(scene_No);
		return 1;
		break;
	}
	}
	return 0;
}














//***gy_led***//
GY_LED gy_led;
unsigned short gy_calc_pwm_duty_by_100_level(u8 level)
{
	return (unsigned short)(level*PWM_MAX_TICK/100);
}

//***gy_led***//
void gy_led_init(void)
{
	//R
	pwm_set(GY_PWMID_R, PWM_MAX_TICK, GY_PWM_INV_R ? PWM_MAX_TICK - 0 : 0);
	pwm_start(GY_PWMID_R);
	gpio_set_func(GY_LED_R_PIN, GY_PWM_FUNC_R);
	gy_led.r_state = gy_led_off;

	//G
	pwm_set(GY_PWMID_G, PWM_MAX_TICK, GY_PWM_INV_G ? PWM_MAX_TICK - 0 : 0);
	pwm_start(GY_PWMID_G);
	gpio_set_func(GY_LED_G_PIN, GY_PWM_FUNC_G);
	gy_led.g_state = gy_led_off;

	return;
}

//***gy_led***//
void gy_set_led_r_state(u8 state)
{
	if(gy_led.r_state != state)
	{
		if(state == gy_led_off)
		{
			pwm_set_cmp (GY_PWMID_R, GY_PWM_INV_R ? PWM_MAX_TICK - GY_LED_OFF : GY_LED_OFF);
		}
		else if(state == gy_led_on)
		{
			pwm_set_cmp (GY_PWMID_R, GY_PWM_INV_R ? PWM_MAX_TICK - GY_LED_ON : GY_LED_ON);
		}
		gy_led.r_state = state;
	}
	return;
}

//***gy_led***//
void gy_set_led_g_state(u8 state)
{
	if(gy_led.g_state != state)
	{
		if(state == gy_led_off)
		{
			pwm_set_cmp (GY_PWMID_G, GY_PWM_INV_G ? PWM_MAX_TICK - GY_LED_OFF : GY_LED_OFF);
		}
		else if(state == gy_led_on)
		{
			pwm_set_cmp (GY_PWMID_G, GY_PWM_INV_G ? PWM_MAX_TICK - GY_LED_ON : GY_LED_ON);
		}
		gy_led.g_state = state;
	}
	return;
}

//***gy_led***//
void gy_led_10ms_timer(u32 count)
{
	if(is_provision_success() || fast_prov.not_need_prov)//如果配网成功
	{
		gy_set_led_r_state(gy_led_off);
		extern u8 ui_ota_is_working;
		if(ui_ota_is_working == 1)//如果正在OTA
		{
			if(count%50 == 0)//0.5s
			{
				gy_set_led_g_state(!gy_led.g_state);
			}
		}
		else
		{
			gy_set_led_g_state(gy_led_on);
		}
	}
	else//如果未配网或者正在配网
	{
		if(gy_fct_opcode_handle_info.current_test_step != gy_test_step_idle)
		{

		}
		else
		{
			if(count%100 == 0)//1s
			{
				gy_set_led_g_state(gy_led_off);
				gy_set_led_r_state(!gy_led.r_state);
			}
		}
	}
	return;
}






















//***gy_button***//
GY_BUTTON gy_button;
void gy_button_init(void)//按键初始化
{
	gpio_set_func(GY_BUTTON_PIN, AS_GPIO);
	gpio_set_input_en(GY_BUTTON_PIN, 1);
	gpio_setup_up_down_resistor(GY_BUTTON_PIN, PM_PIN_PULLUP_1M);
	gy_button.state = gy_button_release;
	gy_button.press_10ms_count = 0;
	gy_button.press_1s_count = 0;
	gy_button.double_click_interval_count = 0;
	gy_button.double_click_flag = 0;
	gy_button.double_click_and_hold_flag = 0;
	return;
}

//***gy_button***//
u8 gy_read_button(void)//获取按钮状态
{
	return !gpio_read(GY_BUTTON_PIN);
}

//***gy_button***//
void gy_button_10ms_timer(u32 count)
{
	gy_button_double_click_and_hold_10ms_timer(count);

	static u8 gy_button_state;//获取按钮状态
	gy_button_state = gy_read_button();//获取按钮状态
	if(gy_button.state != gy_button_state)
	{
		gy_button.state = gy_button_state;
		if(gy_button_state == gy_button_press)//按键刚刚按下
		{
			gy_button.press_10ms_count = 1;
		}
		else if(gy_button_state == gy_button_release)//按键刚刚抬起
		{//*****************执行按键抬起时的操作
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			if(gy_button.press_10ms_count >= GY_XIAODOU_10MS_TIME_MAX && gy_button.press_10ms_count <= GY_SHORT_PRESS_TIME)//短按功能：按下按钮1秒之内松开即为短按
			{
				if(gy_button.double_click_interval_count == 0 && gy_button.double_click_flag == 0)//第一次短按
				{
					gy_button.double_click_interval_count = GY_DOUBLE_CLICK_INTERVAL_10MS_TIME_MAX;//倒计时，如果在500毫秒之内没有再次短按，则执行开关灯命令，否则执行双击功能
				}
				else if(gy_button.double_click_flag == 1)//双击功能，第二次也是短按，执行开关灯
				{
					////////////////////////////////////////////////////////////////////////////////
					gy_light_self_onoff_set();
					////////////////////////////////////////////////////////////////////////////////
					gy_button.double_click_flag = 0;
				}
			}
			else
			{
				if(gy_button.double_click_flag)
				{
					gy_button.double_click_flag = 0;
				}
			}

			if(gy_button.double_click_and_hold_flag)
			{
				gy_button.double_click_and_hold_flag = 0;
			}
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			gy_button.press_10ms_count = 0;
			gy_button.press_1s_count = 0;
		}
	}
	else
	{
		if(gy_button.press_10ms_count)//如果按键按下
		{
			gy_button.press_10ms_count++;
			if(gy_button.press_10ms_count == GY_XIAODOU_10MS_TIME_MAX)//按键完成消抖
			{////////////////////////执行消抖完成时的操作
				////////////////////////////////////////////////////////
				if(gy_button.double_click_interval_count)//如果是双击
				{
					gy_button.double_click_flag = 1;
					gy_button.double_click_interval_count = 0;
				}
				////////////////////////////////////////////////////////
				gy_button.press_1s_count = 1;
			}
			if(gy_button.double_click_flag == 1)
			{
				if(gy_button.press_10ms_count > GY_SHORT_PRESS_TIME)//双击按键，第二次长按
				{////////////////////////Double click and hold - go through dim cycle.
					if(!is_provision_success() && fast_prov.not_need_prov == 0 && gy_fct_opcode_handle_info.current_test_step != gy_test_step_idle)
					{//产测模式

					}
					else
					{
						gy_button.double_click_and_hold_flag = 1;
					}
				}
			}
		}
		else//如果按键松开
		{
			if(gy_button.double_click_interval_count)
			{
				if(gy_button.double_click_interval_count == 1)//如果是单击按键
				{
					///////////////////////////执行单击按键功能/////////////////////////
					if(!is_provision_success() && fast_prov.not_need_prov == 0 && gy_fct_opcode_handle_info.current_test_step != gy_test_step_idle)
					{//产测模式

					}
					else
					{
						//gy_identity_status();
						u8 gy_send_data_t[] = {gy_vendor_property_id_identity, (u8)(GY_PID>>0), (u8)(GY_PID>>8), (u8)(GY_VID>>0), (u8)(GY_VID>>8)};
						//mesh_tx_cmd2normal_primary(u16 op, u8 *par, u32 par_len, u16 adr_dst, int rsp_max)
						mesh_tx_cmd2normal_primary(gy_vendor_opcode_generic_status, gy_send_data_t, sizeof(gy_send_data_t), 0xFFFF, 0);
					}
					///////////////////////////////////////////////////////////////////
				}
				gy_button.double_click_interval_count--;
			}
		}
	}
	return;
}

//***gy_button***//
void gy_button_1s_timer(u32 count)
{
	if(gy_button.press_1s_count)
	{
		if(gy_button.press_1s_count == 10 + 1)
		{
			if(gy_button.double_click_flag == 0)//单击长按按键10秒，恢复出厂设置
			{
				factory_reset();
				show_factory_reset();
				start_reboot();
			}
		}
		gy_button.press_1s_count++;
	}

	return;
}

//***gy_button***//
void gy_button_double_click_and_hold_10ms_timer(u32 count)
{
	if(gy_button.double_click_and_hold_flag)
	{
		u32 gy_10ms_count_t = (gy_button.press_10ms_count - GY_SHORT_PRESS_TIME);
		if(gy_10ms_count_t == 10)
		{
			gy_light_self_dim_value_set(65535);
		}
		else if(gy_10ms_count_t%20 == 0)
		{
			gy_light_self_dim_delta_set(-(655*5));
		}
	}
	return;
}
























//***gy_ctl_light***//
GY_LIGHT_INFO gy_light_info;
void gy_light_init(void)
{
	gy_light_info.state = gy_light_on;
	return;
}

//***gy_ctl_light***//
void gy_light_onoff_set(u16 device_addr, u8 light_onoff)
{
	u8 gy_uart_send_data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, (u8)device_addr, (u8)(device_addr>>8), 0x82, 0x03, light_onoff, 0x00};
	mesh_bulk_cmd((mesh_bulk_cmd_par_t *)(gy_uart_send_data), sizeof(gy_uart_send_data));
	return;
}

//***gy_ctl_light***//
void gy_light_onoff_set_new(u16 device_addr, u8 light_onoff, u8 delay_time)
{//如果需要渐变调光，协议指令的参数必须完整带入，否则不识别带入的渐变参数
	u8 gy_uart_send_data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, (u8)device_addr, (u8)(device_addr>>8), 0x82, 0x03, light_onoff, 0x00, delay_time, 0x00};
	mesh_bulk_cmd((mesh_bulk_cmd_par_t *)(gy_uart_send_data), sizeof(gy_uart_send_data));
	return;
}

//***gy_ctl_light***//
void gy_light_dim_delta_set(u16 device_addr, s32 dim_delta)
{
	//u8 gy_uart_send_data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, (u8)device_addr, (u8)(device_addr>>8), 0x82, 0x0a, (u8)dim_delta, (u8)(dim_delta >> 8), (u8)(dim_delta >> 16), (u8)(dim_delta >> 24), 0x00};
	u8 gy_uart_send_data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, (u8)device_addr, (u8)(device_addr>>8), 0x82, 0x0a, (u8)dim_delta, (u8)(dim_delta >> 8), (u8)(dim_delta >> 16), (u8)(dim_delta >> 24), 0x00, 0x41, 0x00, 0x00};
	mesh_bulk_cmd((mesh_bulk_cmd_par_t *)(gy_uart_send_data), sizeof(gy_uart_send_data));
	return;
}

//***gy_ctl_light***//
void gy_light_dim_value_set(u16 device_addr, u16 dim_value)
{
	u8 gy_uart_send_data[] = {0x00, 0x00, 0x00, 0x00, 0x02, 0x00, (u8)device_addr, (u8)(device_addr>>8), 0x82, 0x4D, (u8)dim_value, (u8)(dim_value >> 8), 0x00};
	mesh_bulk_cmd((mesh_bulk_cmd_par_t *)(gy_uart_send_data), sizeof(gy_uart_send_data));
	return;
}

//***gy_ctl_light***//
void gy_light_self_onoff_set(void)
{
	u8 gy_data_t[] = {0x00,0x00};
	if(gy_light_info.state == gy_light_on)
	{
		if(!is_provision_success() && fast_prov.not_need_prov == 0 && gy_fct_opcode_handle_info.current_test_step != gy_test_step_idle && gy_fct_opcode_handle_info.test_result == 1)
		{
			gy_set_led_r_state(gy_led_on);
			gy_set_led_g_state(gy_led_off);
		}
		//gy_light_info.state = gy_light_off;
	}
	else //if(gy_light_info.state == gy_light_off)
	{
		if(!is_provision_success() && fast_prov.not_need_prov == 0 && gy_fct_opcode_handle_info.current_test_step != gy_test_step_idle && gy_fct_opcode_handle_info.test_result == 1)
		{
			gy_set_led_r_state(gy_led_off);
			gy_set_led_g_state(gy_led_on);
		}
		gy_data_t[0] = 0x01;
		//gy_light_info.state = gy_light_on;
	}
	extern int mesh_cmd_sig_g_onoff_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
	mesh_cmd_sig_g_onoff_set(gy_data_t, sizeof(gy_data_t), NULL);
	return;
}

//***gy_ctl_light***//
void gy_light_self_dim_delta_set(s32 dim_delta)
{
	//u8 gy_data_t[] = {(u8)dim_delta, (u8)(dim_delta >> 8), (u8)(dim_delta >> 16), (u8)(dim_delta >> 24), 0x00};
	//extern int mesh_cmd_sig_g_level_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
	//mesh_cmd_sig_g_level_set(gy_data_t, sizeof(gy_data_t), NULL);
	gy_light_dim_delta_set(ele_adr_primary, dim_delta);
	return;
}

//***gy_ctl_light***//
void gy_light_self_dim_value_set(u16 dim_value)
{
	u8 gy_data_t[] = {(u8)dim_value, (u8)(dim_value >> 8), 0x00};
	extern int mesh_cmd_sig_lightness_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
	mesh_cmd_sig_lightness_set(gy_data_t, sizeof(gy_data_t), NULL);
	return;
}























//***gy_relay***//
GY_RELAY_INFO gy_relay_info;
void gy_relay_init(void)//vendor/mesh/app.c文件user_init函数中调用
{
	led_onoff_gpio(GY_RELAY_PIN, gy_relay_off);
#if(GY_DEV_TYPE == GY_DEV_108F)
	gy_public_info.light_ssr_state = gy_relay_on ;
#endif
	gy_relay_info.state = gy_relay_off;
	return;
}

//***gy_relay***//
void gy_relay_onoff_set(u8 relay_state)
{
	if(gy_relay_info.state != relay_state)
	{
#if(GY_DEV_TYPE == GY_DEV_108F)
		wd_clear();
		if(relay_state == gy_relay_off)
		sleep_us(505*10); //默认是6ms +4.4
		else
	    sleep_us(54*100);
#endif
		led_onoff_gpio(GY_RELAY_PIN, relay_state);
		gy_relay_info.state = relay_state;
	}
	return;
}


















//***gy_i2c***//
GY_I2C_INFO gy_i2c_info;

//***gy_i2c***//
void gy_i2c_read_MCP3021A5T(u8 *buff)
{
	i2c_set_id(GY_MCP3021A5T_ADDR);
	i2c_read_series(0x88,1,buff,2);
	return;
}

//***gy_i2c***//
u8 gy_h_sun_i2c_ack_handle(void)
{
	u8 gy_err_t = 0;

	GY_H_SUN_SCK_L;
	GY_H_SUN_QUARTER_CYCLE;
	gpio_set_output_en(GY_I2C_SDA_PIN, 0);
	gpio_set_input_en(GY_I2C_SDA_PIN, 1);
	GY_H_SUN_QUARTER_CYCLE;
	GY_H_SUN_SCK_H;
	GY_H_SUN_QUARTER_CYCLE;
	if(gpio_read(GY_I2C_SDA_PIN) == 0)//成功应答
	{
		gy_err_t = 1;
	}
	else//应答失败
	{
		GY_H_SUN_SDA(1);
		gpio_set_output_en(GY_I2C_SDA_PIN, 1);
		gpio_set_input_en(GY_I2C_SDA_PIN, 0);
		//GY_H_SUN_SDA(1);
	}
	GY_H_SUN_QUARTER_CYCLE;
	//gpio_set_output_en(GY_I2C_SDA_PIN, 1);
	//gpio_set_input_en(GY_I2C_SDA_PIN, 0);
	//GY_H_SUN_SDA(1);

	return gy_err_t;
}

//***gy_i2c***//
void gy_h_sun_i2c_write_one_byte(u8 w_data)
{
	u8 i;
	for(i = 7; i >= 0; i--)
	{
		GY_H_SUN_SCK_L;
		GY_H_SUN_QUARTER_CYCLE;

		if(i == 7)
		{
			gpio_set_output_en(GY_I2C_SDA_PIN, 1);
			gpio_set_input_en(GY_I2C_SDA_PIN, 0);
		}

		GY_H_SUN_SDA((w_data>>i) & 0x01);
		GY_H_SUN_QUARTER_CYCLE;
		GY_H_SUN_SCK_H;
		GY_H_SUN_QUARTER_CYCLE;
		GY_H_SUN_QUARTER_CYCLE;

		if(i == 0)
		{
			break;
		}
	}
}

//***gy_i2c***//
u8 gy_h_sun_i2c_write_one_byte_and_ack(u8 w_data)
{
	gy_h_sun_i2c_write_one_byte(w_data);
	u8 gy_err_t = gy_h_sun_i2c_ack_handle();//第9个时钟作为应答
	return gy_err_t;
}

//***gy_i2c***//
u8 gy_h_sun_i2c_write(u8 dev_addr, u8 reg_addr, u8* write_buff, u8 len)
{
	u8 gy_write_dev_addr = (dev_addr<<1)|0x00;

	GY_H_SUN_SCK_H;
	GY_H_SUN_SDA(1);
	GY_H_SUN_FRAME_INTERVAL;

	//起始位
	GY_H_SUN_SDA(0);
	GY_H_SUN_QUARTER_CYCLE;
	GY_H_SUN_QUARTER_CYCLE;

	//gy_h_sun_i2c_write_one_byte_and_ack(gy_write_dev_addr);//写入写地址字节
	if(gy_h_sun_i2c_write_one_byte_and_ack(gy_write_dev_addr) == 0)
	{
		//return 0;
	}
	//gy_h_sun_i2c_write_one_byte_and_ack(reg_addr);//写入寄存器地址字节
	if(gy_h_sun_i2c_write_one_byte_and_ack(reg_addr) == 0)
	{
		return 0;
	}

	u8 j;
	for(j = 0; j < len; j++)
	{
		//gy_h_sun_i2c_write_one_byte_and_ack(write_buff[j]);//写入数据字节
		if(gy_h_sun_i2c_write_one_byte_and_ack(write_buff[j]) == 0)
		{
			return 0;
		}
	}

	//停止位
	GY_H_SUN_SCK_L;
	GY_H_SUN_QUARTER_CYCLE;
	gpio_set_output_en(GY_I2C_SDA_PIN, 1);
	gpio_set_input_en(GY_I2C_SDA_PIN, 0);
	GY_H_SUN_SDA(0);
	GY_H_SUN_QUARTER_CYCLE;
	GY_H_SUN_SCK_H;
	GY_H_SUN_QUARTER_CYCLE;
	GY_H_SUN_SDA(1);
	GY_H_SUN_QUARTER_CYCLE;

	return 1;
}

//***gy_i2c***//
u8 gy_h_sun_i2c_read(u8 dev_addr, u8 reg_addr, u8* read_buff, u8 len)
{
	memset(read_buff, 0, len);

	//gy_h_sun_i2c_write(dev_addr, reg_addr, NULL, 0);
	if(gy_h_sun_i2c_write(dev_addr, reg_addr, NULL, 0) == 0)
	{
		return 0;
	}
	GY_H_SUN_QUARTER_CYCLE;
	GY_H_SUN_QUARTER_CYCLE;
	GY_H_SUN_QUARTER_CYCLE;
	GY_H_SUN_QUARTER_CYCLE;
	GY_H_SUN_QUARTER_CYCLE;//延时50us

	//起始位
	GY_H_SUN_SDA(0);
	GY_H_SUN_QUARTER_CYCLE;
	GY_H_SUN_QUARTER_CYCLE;
	//gy_h_sun_i2c_write_one_byte_and_ack(((dev_addr<<1)|0x01));
	if(gy_h_sun_i2c_write_one_byte_and_ack(((dev_addr<<1)|0x01)) == 0)
	{
		return 0;
	}
	//gy_h_sun_i2c_write_one_byte_and_ack(0x45);

	//gpio_set_output_en(GY_I2C_SDA_PIN, 0);
	//gpio_set_input_en(GY_I2C_SDA_PIN, 1);

	u8 i,j;
	for(j = 0; j < len; j++)
	{
		//read_buff[j] |= ((!!gpio_read(GY_I2C_SDA_PIN))<<7);
		for(i = 7; i >= 0; i--)
		{
			GY_H_SUN_SCK_L;
			GY_H_SUN_QUARTER_CYCLE;
			GY_H_SUN_QUARTER_CYCLE;
			GY_H_SUN_SCK_H;
			GY_H_SUN_QUARTER_CYCLE;
			read_buff[j] |= ((!!gpio_read(GY_I2C_SDA_PIN))<<i);
			GY_H_SUN_QUARTER_CYCLE;

			if(i == 0)
			{
				break;
			}
		}
		GY_H_SUN_SCK_L;
		GY_H_SUN_QUARTER_CYCLE;
		GY_H_SUN_QUARTER_CYCLE;
		GY_H_SUN_SCK_H;
		GY_H_SUN_QUARTER_CYCLE;
		GY_H_SUN_QUARTER_CYCLE;
	}

	//停止位
	GY_H_SUN_SCK_L;
	GY_H_SUN_QUARTER_CYCLE;
	gpio_set_output_en(GY_I2C_SDA_PIN, 1);
	gpio_set_input_en(GY_I2C_SDA_PIN, 0);
	GY_H_SUN_SDA(0);
	GY_H_SUN_QUARTER_CYCLE;
	GY_H_SUN_SCK_H;
	GY_H_SUN_QUARTER_CYCLE;
	GY_H_SUN_SDA(1);
	GY_H_SUN_QUARTER_CYCLE;

	return 1;
}

//***gy_i2c***//
void gy_h_sun_i2c_reset(void)
{
	gpio_set_func(GY_I2C_SDA_PIN, AS_GPIO);
	gpio_set_output_en(GY_I2C_SDA_PIN, 1);
	gpio_set_input_en(GY_I2C_SDA_PIN, 0);
	GY_H_SUN_SDA(1);
	gpio_set_func(GY_I2C_SCK_PIN, AS_GPIO);
	gpio_set_output_en(GY_I2C_SCK_PIN, 1);
	gpio_set_input_en(GY_I2C_SCK_PIN, 0);
	GY_H_SUN_SCK_H;
	gy_h_sun_i2c_write(GY_H_SUN_DEV_ADDR, 0x06, NULL, 0);
	gy_h_sun_i2c_write(GY_H_SUN_DEV_ADDR, 0x06, NULL, 0);
	gy_h_sun_i2c_write(GY_H_SUN_DEV_ADDR, 0x06, NULL, 0);
	gy_h_sun_i2c_write(GY_H_SUN_DEV_ADDR, 0x06, NULL, 0);
	return;
}

//***gy_i2c***//
u16 gy_h_sun_off_on_off_10ms_count_down = 0;//初始化为0
void gy_h_sun_off_on_off_10ms_timer(u32 count)
{
	if(gy_h_sun_off_on_off_10ms_count_down)
	{
		gy_h_sun_off_on_off_10ms_count_down--;
	}
	return;
}

//***gy_i2c***//
void gy_h_sun_off_on_off_count_down_refresh(void)
{
	gy_h_sun_off_on_off_10ms_count_down = GY_H_SUN_OFF_ON_OFF_10MS_COUNT_DOWN_MAX;
	return;
}

//***gy_i2c***//
void gy_h_sun_off_on_off_count_down_clear(void)
{
	if(gy_h_sun_off_on_off_10ms_count_down)
	{
		gy_h_sun_off_on_off_10ms_count_down = 0;
	}
	return;
}

//***gy_i2c***//
void gy_i2c_check_sensor_type(void)
{
	//if(!is_provision_success() && fast_prov.not_need_prov == 0 && gy_fct_opcode_handle_info.current_test_step != gy_test_step_idle)
	//{//产测模式

	//}
	//else
	{
		//wd_clear();

		//检测华芯微芯片做的传感器
		gpio_set_func(GY_I2C_SDA_PIN, AS_GPIO);
		gpio_set_output_en(GY_I2C_SDA_PIN, 1);
		gpio_set_input_en(GY_I2C_SDA_PIN, 0);
		GY_H_SUN_SDA(1);

		gpio_set_func(GY_I2C_SCK_PIN, AS_GPIO);
		gpio_set_output_en(GY_I2C_SCK_PIN, 1);
		gpio_set_input_en(GY_I2C_SCK_PIN, 0);
		GY_H_SUN_SCK_H;

		u8 gy_buff_t[2] = {0xFF, 0xFF};
		gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x05, &gy_buff_t[0], 1);

		//if(gy_buff_t[0] != 0xFF)
		if(gy_buff_t[0] == 0x14)
		{
			gy_i2c_info.sensor_type = gy_i2c_sensor_3;
			gy_i2c_info.sensor_set_state = GY_SENSOR_I2C_SET_FINISH;
			return;
		}

		//检测RAB自用的松下传感器
		i2c_gpio_set(I2C_GPIO_GROUP_C2C3);
		i2c_master_init(GY_MCP3021A5T_ADDR,GY_I2C_CLOCK);

		gy_i2c_read_MCP3021A5T(gy_buff_t);
		//if(gy_buff_t[0] != 0xFF || gy_buff_t[1] != 0xFF)
		if(((gy_buff_t[0]<<8)|gy_buff_t[1])>0 && ((gy_buff_t[0]<<8)|gy_buff_t[1])<0xFFFF)
		{
			gy_i2c_info.sensor_type = gy_i2c_sensor_1;
			if(gy_i2c_info.sensor_set_state != GY_SENSOR_I2C_SET_FINISH)
			{
				gy_i2c_info.sensor_set_state = 0;
			}
			return;
		}

		i2c_set_id(GY_MW_SENSOR_ADDR);
		gy_buff_t[0] = 0xFF;
		i2c_read_series(GY_MW_SENSOR_TEST_REG,1,&gy_buff_t[0],1);
		if(gy_buff_t[0] == 0x66 || gy_buff_t[0] == 0x88)
		{
			gy_i2c_info.sensor_type = gy_i2c_sensor_2;
			if(gy_i2c_info.sensor_set_state != GY_SENSOR_I2C_SET_FINISH)
			{
				gy_i2c_info.sensor_set_state = 0;
			}
			return;
		}

		//否则没有接任何传感器
		gy_i2c_info.sensor_type = gy_i2c_sensor_none;
	}
	return;
}

//***gy_i2c***//
void gy_i2c_init(void)
{
	gy_i2c_info.sensor_event_check_10ms_count_down = 0;
	gy_i2c_info.sensor_event_recheck_count_down = 0;
	gy_i2c_info.sensor_led_on_10ms_count_down = 0;
	gy_i2c_check_sensor_type();
	//gy_i2c_info.sensor_type = gy_i2c_sensor_none;
	//gy_i2c_info.sensor_flag = gy_i2c_sensor_disable;
	//gy_i2c_info.sensor_trigger_count_down = 0;
	return;
}

//***gy_i2c***//
u8 gy_i2c_read_motion_data_and_return_state(void)//读取sensor的人感数据，并且返回当前是否满足触发条件（0表示不满足条件，非零表示满足条件）
{
	u8 gy_err_t = 0;
	if(gy_i2c_info.sensor_type == gy_i2c_sensor_none)
	{
		return gy_err_t;
	}
	switch(gy_i2c_info.sensor_type)
	{
	case gy_i2c_sensor_1:
	{
		u8 gy_buff_t[2] = {0xFF, 0xFF};
		i2c_set_id(GY_MCP3021A5T_ADDR);
		i2c_read_series(0x88,1,gy_buff_t,2);
		if(((gy_buff_t[0]<<8)|gy_buff_t[1])>0 && ((gy_buff_t[0]<<8)|gy_buff_t[1])<0xFFFF)
		{
			u8 gy_motion_level_t = 0;
			if(gy_buff_t[0] < 0x05)
			{
				gy_motion_level_t = 0x05 - (0x05 - gy_buff_t[0]);
			}
			else
			{
				gy_motion_level_t = 0x05 - (gy_buff_t[0] - 0x05);
			}
			if(gy_flash_info.sensor_property.MW_Sensitivity == 0)//人感灵敏度为0，不做任何处理
			{

			}
			else
			{
				u8 gy_MW_Sensitivity_level_t = gy_flash_info.sensor_property.MW_Sensitivity/(65535/5);
				if(gy_MW_Sensitivity_level_t == 5)
				{
					gy_MW_Sensitivity_level_t--;
				}
				if(gy_MW_Sensitivity_level_t >= gy_motion_level_t)
				{
					gy_err_t = 1;
				}
			}
		}
		break;
	}
	case gy_i2c_sensor_2:
	{
		if(gy_flash_info.sensor_property.MW_Sensitivity == 0)//人感灵敏度为0，不做任何处理
		{

		}
		else
		{
			u8 gy_data_t = 0;
			i2c_read_series(GY_MW_SENSOR_MOTION_STATE_REG,1,&gy_data_t,1);
			if(gy_data_t == 1)//检测到人
			{
				gy_err_t = 1;
			}
		}
		break;
	}
	case gy_i2c_sensor_3:
	{
		u8 gy_buff_t[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
		if(0 == gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x05, &gy_buff_t[4], 1))
		{
			gy_h_sun_i2c_reset();
		}
		else
		{
			gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x01, &gy_buff_t[0], 1);
			gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x02, &gy_buff_t[1], 1);
			gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x03, &gy_buff_t[2], 1);
			//gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x04, &gy_buff_t[3], 1);

			if(gy_buff_t[4] == 0x14)
			{
				u16 gy_pir_value = 0;
				if(gy_buff_t[0] == 0xFF)
				{
					gy_pir_value = 0xffff - ((gy_buff_t[1]<<8)|(gy_buff_t[2]));
				}
				else
				{
					gy_pir_value = ((gy_buff_t[1]<<8)|(gy_buff_t[2]));
				}
				if((gy_pir_value >= GY_H_SUN_PIR_HIGH_SENSITIVITY_MAX + (100 - gy_flash_info.sensor_property.MW_Sensitivity/655)*2))
				{
					gy_err_t = 1;
				}
			}
		}
		break;
	}
	}
	return gy_err_t;
}

//***gy_i2c***//
u8 gy_i2c_read_als_data_and_return_state(void)//读取sensor的光感数据，并且返回当前是否满足触发条件（0表示不满足条件，非零表示满足条件）
{
	u8 gy_err_t = 0;
	if(gy_i2c_info.sensor_type == gy_i2c_sensor_none)
	{
		return gy_err_t;
	}
	if(gy_flash_info.sensor_property.daylight_harvesting!=0)
	{
		gy_err_t = 1;
		return gy_err_t;
	}
	if(gy_flash_info.sensor_property.photocell_onoff_control&&(gy_flash_info.sensor_property.sensor_role==0))
	{
		gy_err_t = 1;
		return gy_err_t;
	}
	if(gy_flash_info.sensor_property.photocell_onoff_control)
	{

		lc_read_lux();
		if(lc_current_lux < gy_flash_info.sensor_property.photocell_threshold)
		{
			gy_err_t = 1;

		}
	}
		else
		{
	switch(gy_i2c_info.sensor_type)
	{

	case gy_i2c_sensor_1:
	{
		u8 gy_buff_t[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
		i2c_set_id(GY_LTR_329ALS_01_ADDR);
		i2c_read_series(GY_L_SENSOR_ALS_STATUS_REG,1,&gy_buff_t[0],1);
		if(gy_buff_t[0]&0x04)
		{
			i2c_read_series(GY_L_SENSOR_ALS_DATA_CH1_0REG,1,&gy_buff_t[1],4);
			u16 gy_als_value_t = (gy_buff_t[4]<<8)|gy_buff_t[3];
			if(gy_flash_info.sensor_property.ALS_Threshold==65535)
				gy_err_t = 1;
				else
			{
					if(gy_flash_info.sensor_property.subid_for_lclcsense_sh==1)
					{
						if(gy_als_value_t <= gy_flash_info.sensor_property.ALS_Threshold)
						{
							gy_err_t = 1;
						}
					}
					else
					{
						lc_threshold_value_to_lux(gy_flash_info.sensor_property.ALS_Threshold);
						if(gy_als_value_t!=0xffff)
						{

								lc_current_lux=(u16)(0.136*gy_als_value_t-8.447+0.5);
								if(gy_als_value_t<500)
									lc_current_lux=60;
								else
								{
									if(lc_current_lux>1900)
										lc_current_lux=1900;
								}
						}
						if(lc_current_lux <= lc_threshold_value_lux)
						{
							gy_err_t = 1;
						}
					}
				}

		}
		break;
	}
	case gy_i2c_sensor_2:
	{
		u8 gy_buff_t[2] = {0xFF, 0xFF};
		i2c_read_series(GY_MW_SENSOR_ALS_ADC1_REG,1,&gy_buff_t[0],2);
		u16 gy_als_value_t = (gy_buff_t[0]<<8)|gy_buff_t[1];
		if(gy_als_value_t <= gy_flash_info.sensor_property.ALS_Threshold)
		{
			gy_err_t = 1;
		}
		break;
	}
	case gy_i2c_sensor_3:
	{
		u8 gy_buff_t[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
		if(0 == gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x05, &gy_buff_t[4], 1))
		{
			gy_h_sun_i2c_reset();
		}
		else
		{
			//gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x01, &gy_buff_t[0], 1);
			//gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x02, &gy_buff_t[1], 1);
			//gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x03, &gy_buff_t[2], 1);
			gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x04, &gy_buff_t[3], 1);

			if(gy_buff_t[4] == 0x14)
			{
				u8 gy_als_value = gy_buff_t[3];

				if(0x7F-gy_als_value <= gy_flash_info.sensor_property.ALS_Threshold*0x7F/(655*100))
				{
					gy_err_t = 1;
				}
			}
		}
		break;
	}

	}

}
	return gy_err_t;
	}

//***gy_i2c***//
u8 gy_i2c_read_data_and_return_state(void)//读取sensor数据，并且返回当前是否满足触发sensor的条件（0表示不满足条件，非零表示满足条件）
{
	u8 gy_err_t = 0;
	if(gy_i2c_info.sensor_type == gy_i2c_sensor_none)
	{
		return gy_err_t;
	}

	if(gy_i2c_read_motion_data_and_return_state())
	{
		if(gy_public_info.occupied_time_count_down || gy_i2c_read_als_data_and_return_state())
		{
			gy_err_t = 1;
		}
	}
	return gy_err_t;
}

//***gy_i2c***//
void gy_i2c_sensor_event_time_count_down_handle(u8 check_10ms_count_down, u8 recheck_count_down)
{
	gy_i2c_info.sensor_event_check_10ms_count_down = check_10ms_count_down;
	gy_i2c_info.sensor_event_recheck_count_down = recheck_count_down;
	return;
}

//***gy_i2c***//
void gy_i2c_10ms_timer(u32 count)
{
	gy_i2c_sensor_check_10ms_timer(count);
	gy_i2c_sensor_set_10ms_timer(count);
	gy_h_sun_off_on_off_10ms_timer(count);
	gy_i2c_sensor_execute_10ms_timer(count);
	gy_i2c_sensor_event_handle_10ms_timer(count);
	gy_i2c_sensor_led_10ms_timer(count);
	return;
}

//***gy_i2c***//
void gy_i2c_sensor_check_10ms_timer(u32 count)//检测传感器是否存在，以及确认传感器类型
{
	/*
	if(count == 100)//启动第一秒时检测是否有传感器
	{
		if(!is_provision_success() && fast_prov.not_need_prov == 0 && gy_fct_opcode_handle_info.current_test_step != gy_test_step_idle)
		{//产测模式

		}
		else
		{

		}
	}*/
	if(count <= 1000 && (count % 50 == 0) && gy_i2c_info.sensor_type == gy_i2c_sensor_none)
	{
		gy_i2c_check_sensor_type();
	}
	return;
}

//***gy_i2c***//
static u16 gy_mw_sensor_motion_sensitivity_value_transformation(u8 mw_sensor_motion_level)
{
	u16 gy_mw_sensor_motion_sensitivity_value_t = 0xFFFF;
	switch(mw_sensor_motion_level)
	{
	case 0:
	{
		gy_mw_sensor_motion_sensitivity_value_t = 700;
		break;
	}
	case 1:
	{
		gy_mw_sensor_motion_sensitivity_value_t = 600;
		break;
	}
	case 2:
	{
		gy_mw_sensor_motion_sensitivity_value_t = 500;
		break;
	}
	case 3:
	{
		gy_mw_sensor_motion_sensitivity_value_t = 400;
		break;
	}
	default:
	{
		gy_mw_sensor_motion_sensitivity_value_t = 400;
	}
	}
	return gy_mw_sensor_motion_sensitivity_value_t;
}

//***gy_i2c***//
void gy_i2c_sensor_set_10ms_timer(u32 count)//如果检测到传感器，需要根据传感器类型对传感器进行设定
{
	if(gy_i2c_info.sensor_type == gy_i2c_sensor_none)
	{
		return;
	}
	if(gy_i2c_info.sensor_set_state == GY_SENSOR_I2C_SET_FINISH)
	{
		return;
	}
	if(count > 0 && (count%10 == 0))
	{
		switch(gy_i2c_info.sensor_type)
		{
		case gy_i2c_sensor_1://RAB自己的松下PIR
		{
			if(gy_i2c_info.sensor_set_state == 0)
			{
				i2c_set_id(GY_LTR_329ALS_01_ADDR);
				i2c_write_byte(GY_L_SENSOR_ALS_CONTR_REG, 1, GY_L_SENSOR_ALS_CONTR_VALUE);
				gy_i2c_info.sensor_set_state = 1;
			}
			else if(gy_i2c_info.sensor_set_state == 1)
			{
				u8 gy_i2c_read_data_t = 0;
				i2c_set_id(GY_LTR_329ALS_01_ADDR);
				i2c_read_series(GY_L_SENSOR_ALS_CONTR_REG,1,&gy_i2c_read_data_t,1);
				if(gy_i2c_read_data_t == GY_L_SENSOR_ALS_CONTR_VALUE)
				{
					gy_i2c_info.sensor_set_state = 2;
				}
				else
				{
					gy_i2c_info.sensor_set_state = 0;
				}
			}
			else if(gy_i2c_info.sensor_set_state == 2)
			{
				i2c_set_id(GY_LTR_329ALS_01_ADDR);
				i2c_write_byte(GY_ALS_MEAS_RATE_REG, 1, GY_ALS_MEAS_RATE_VALUE);
				gy_i2c_info.sensor_set_state = 3;
			}
			else if(gy_i2c_info.sensor_set_state == 3)
			{
				u8 gy_i2c_read_data_t = 0;
				i2c_set_id(GY_LTR_329ALS_01_ADDR);
				i2c_read_series(GY_ALS_MEAS_RATE_REG,1,&gy_i2c_read_data_t,1);
				if(gy_i2c_read_data_t == GY_ALS_MEAS_RATE_VALUE)
				{
					gy_i2c_info.sensor_set_state = GY_SENSOR_I2C_SET_FINISH;
				}
				else
				{
					gy_i2c_info.sensor_set_state = 2;
				}
			}
			break;
		}
		case gy_i2c_sensor_2://微波人感
		{
			if(gy_i2c_info.sensor_set_state == 0)
			{
				u8 gy_i2c_read_data_t = 0;
				i2c_read_series(GY_MW_SENSOR_TEST_REG,1,&gy_i2c_read_data_t,1);
				if(gy_i2c_read_data_t == GY_MW_SENSOR_TEST_VALUE1)//初始化不做任何操作
				{

				}
				else if(gy_i2c_read_data_t == GY_MW_SENSOR_TEST_VALUE2)//初始化完成
				{
					gy_i2c_info.sensor_set_state = 1;
				}
			}
			else if(gy_i2c_info.sensor_set_state == 1)
			{
				if(gy_flash_info.sensor_property.MW_Sensitivity == 0)
				{
					gy_i2c_info.sensor_set_state = GY_SENSOR_I2C_SET_FINISH;
				}
				else
				{
					u8 gy_i2c_read_data_t[2] = {0xFF, 0xFF};
					i2c_read_series(GY_MW_SENSOR_MOTION_SENSITIVITY_REG,1,&gy_i2c_read_data_t[0],2);
					u16 gy_mw_sensor_motion_sensitivity_value = (gy_i2c_read_data_t[0]<<8) | gy_i2c_read_data_t[1];

					u8 gy_mw_sensor_motion_level_t = gy_flash_info.sensor_property.MW_Sensitivity/(65535/4+1);
					if(gy_mw_sensor_motion_sensitivity_value_transformation(gy_mw_sensor_motion_level_t) == gy_mw_sensor_motion_sensitivity_value)
					{
						gy_i2c_info.sensor_set_state = GY_SENSOR_I2C_SET_FINISH;
					}
					else
					{
						gy_i2c_info.sensor_set_state = 2;
					}
				}
			}
			else if(gy_i2c_info.sensor_set_state == 2)
			{
				u8 gy_mw_sensor_motion_level_t = gy_flash_info.sensor_property.MW_Sensitivity/(65535/4+1);
				u16 gy_mw_sensor_motion_sensitivity_value = gy_mw_sensor_motion_sensitivity_value_transformation(gy_mw_sensor_motion_level_t);
				u8 gy_i2c_write_data_t[2] = {(u8)(gy_mw_sensor_motion_sensitivity_value>>8), (u8)gy_mw_sensor_motion_sensitivity_value};
				i2c_write_series(GY_MW_SENSOR_MOTION_SENSITIVITY_REG, 1, &gy_i2c_write_data_t[0],2);
				gy_i2c_info.sensor_set_state = 1;
			}
			break;
		}
		case gy_i2c_sensor_3://华芯微芯片做的传感器
		{//暂没有什么需要设定的
			break;
		}
		}
	}
	return;
}

//***gy_i2c***//
//#define GY_READ_I2C_DATA_INTERVAL	10//每间隔10*10ms时间读取一次i2c数据
u16 gy_sensor_freeze_time_count_down = 0;
void gy_i2c_sensor_execute_10ms_timer(u32 count)//传感器执行程序
{
	if(gy_vendor_model_info.emergency_event_flag != gy_emergency_event_close)
	{
		return;
	}
	extern u8 ui_ota_is_working;
	if(ui_ota_is_working == 1)//如果正在OTA
	{
		return;
	}
	if(gy_h_sun_off_on_off_10ms_count_down)
	{
		return;
	}
	if(gy_i2c_info.sensor_type == gy_i2c_sensor_none)
	{
		return;
	}
	if(gy_i2c_info.sensor_set_state != GY_SENSOR_I2C_SET_FINISH)
	{
		return;
	}
	if(count%GY_READ_I2C_DATA_INTERVAL == 0 && count > 0)
	{
		//if(!is_provision_success() && fast_prov.not_need_prov == 0 && gy_fct_opcode_handle_info.current_test_step != gy_test_step_idle)
		if(gy_fct_opcode_handle_info.current_test_step != gy_test_step_idle)
		{//产测模式
			if(gy_fct_opcode_handle_info.sensor_test_type == gy_sensor_test_motion)
			{
				if(gy_i2c_read_motion_data_and_return_state())
				{
					gy_light_info.state = gy_light_off;
					gy_light_self_onoff_set();
					gy_public_info.occupied_time_count_down = (gy_flash_info.sensor_property.unoccupied_time_delay*1);
				}
			}
			else if(gy_fct_opcode_handle_info.sensor_test_type == gy_sensor_test_ALS)
			{
				if(gy_i2c_read_als_data_and_return_state())
				{
					gy_light_info.state = gy_light_off;
				}
				else
				{
					gy_light_info.state = gy_light_on;
				}
				gy_light_self_onoff_set();
			}
		}
		else
		{
			//static u16 gy_sensor_freeze_time_count_down = 0;
			if(gy_sensor_freeze_time_count_down)
			{
				gy_sensor_freeze_time_count_down--;
			}
			else
			{
				if(gy_i2c_read_data_and_return_state())
				{
					if(gy_i2c_info.sensor_type == gy_i2c_sensor_1)
					{
						gy_i2c_sensor_event_time_count_down_handle(5, 1);
					}
					else
					{
						gy_sensor_status_send(gy_sensor_state_people);
						//因为该位置每间隔10*GY_READ_I2C_DATA_INTERVAL毫秒执行一次，所以将秒单位转换为10*GY_READ_I2C_DATA_INTERVAL毫秒单位需要做如下转化
						gy_sensor_freeze_time_count_down = gy_flash_info.sensor_property.sensor_freeze_time*1000/(10*GY_READ_I2C_DATA_INTERVAL);
						gy_i2c_sensor_led_on_with_time(100);
					}
				}
			}
		}
	}
	return;
}

//***gy_i2c***//
void gy_i2c_sensor_event_handle_10ms_timer(u32 count)
{
	static u8 gy_sensor_event_check_10ms_count_down = 0;
	if(gy_i2c_info.sensor_event_recheck_count_down && gy_sensor_event_check_10ms_count_down == 0)
	{
		gy_sensor_event_check_10ms_count_down = gy_i2c_info.sensor_event_check_10ms_count_down;
	}
	if(gy_sensor_event_check_10ms_count_down)
	{
		if(gy_sensor_event_check_10ms_count_down == 1)
		{
			if(gy_i2c_info.sensor_event_recheck_count_down)
			{
				if(gy_i2c_info.sensor_event_recheck_count_down == 1)
				{
					//if(gy_i2c_read_data_and_return_state())
					if(gy_i2c_read_motion_data_and_return_state())
					{
						gy_sensor_status_send(gy_sensor_state_people);
						//因为该位置每间隔10*GY_READ_I2C_DATA_INTERVAL毫秒执行一次，所以将秒单位转换为10*GY_READ_I2C_DATA_INTERVAL毫秒单位需要做如下转化
						gy_sensor_freeze_time_count_down = gy_flash_info.sensor_property.sensor_freeze_time*1000/(10*GY_READ_I2C_DATA_INTERVAL);
						gy_i2c_sensor_led_on_with_time(100);
					}
				}
				gy_i2c_info.sensor_event_recheck_count_down--;
			}
		}
		gy_sensor_event_check_10ms_count_down--;
	}
	return;
}

//***gy_i2c***//
void gy_i2c_sensor_led_init(void)
{
	static u8 gy_i2c_sensor_led_set_finish_flag = 0;
	if(gy_i2c_sensor_led_set_finish_flag)
	{
		return;
	}

	if(gy_i2c_info.sensor_type == gy_i2c_sensor_none)
	{
		return;
	}

	switch(gy_i2c_info.sensor_type)
	{
	case gy_i2c_sensor_1://RAB自己的松下PIR
	{
		i2c_set_id(GY_PCA9536_ADDR);
		u8 gy_i2c_data_t = 0xff;
		i2c_read_series(GY_PCA9536_CONFIGURATION_REG, 1, &gy_i2c_data_t, 1);
		if((gy_i2c_data_t & 0x03))
		{
			gy_i2c_data_t = (gy_i2c_data_t & 0xfc);
			i2c_write_series(GY_PCA9536_CONFIGURATION_REG, 1, &gy_i2c_data_t, 1);
		}

		gy_i2c_data_t = 0xff;
		i2c_read_series(GY_PCA9536_OUTPUT_PORT_REG, 1, &gy_i2c_data_t, 1);
		if((gy_i2c_data_t & 0x03))
		{
			//gy_i2c_data_t = (gy_i2c_data_t & 0xfc);
			gy_i2c_data_t = (gy_i2c_data_t | 0x03);
			i2c_write_series(GY_PCA9536_OUTPUT_PORT_REG, 1, &gy_i2c_data_t, 1);
		}

		//gy_i2c_sensor_led_set_finish_flag = 1;
		break;
	}
	case gy_i2c_sensor_2://微波人感
	{
		gy_i2c_sensor_led_off();
		break;
	}
	case gy_i2c_sensor_3://华芯微芯片做的传感器
	{
		break;
	}
	}
	gy_i2c_sensor_led_set_finish_flag = 1;
	return;
}

//***gy_i2c***//
void gy_i2c_sensor_led_onoff_set(u8 led_onoff)
{
	switch(gy_i2c_info.sensor_type)
	{
	case gy_i2c_sensor_1://RAB自己的松下PIR
	{
		i2c_set_id(GY_PCA9536_ADDR);
		u8 gy_i2c_data_t = 0xff;
		i2c_read_series(GY_PCA9536_OUTPUT_PORT_REG, 1, &gy_i2c_data_t, 1);
		if(led_onoff)//开灯
		{
			if((gy_i2c_data_t && 0x03) != 0)//如果指示灯是关闭状态
			{
				gy_i2c_data_t = (gy_i2c_data_t & 0xfc);
			}
		}
		else//关灯
		{
			if((gy_i2c_data_t && 0x03) != 0x03)//如果指示灯是打开状态
			{
				gy_i2c_data_t = (gy_i2c_data_t | 0x03);
			}
		}
		i2c_write_series(GY_PCA9536_OUTPUT_PORT_REG, 1, &gy_i2c_data_t, 1);
		break;
	}
	case gy_i2c_sensor_2://微波人感
	{
		u8 gy_i2c_data_t = (!!led_onoff);
		i2c_write_series(GY_MW_SENSOR_LED_CTL_REG, 1, &gy_i2c_data_t, 1);
		break;
	}
	case gy_i2c_sensor_3://华芯微芯片做的传感器
	{
		break;
	}
	}
	return;
}

//***gy_i2c***//
void gy_i2c_sensor_led_on(void)
{
	gy_i2c_sensor_led_onoff_set(gy_sensor_led_on);
	return;
}

//***gy_i2c***//
void gy_i2c_sensor_led_off(void)
{
	gy_i2c_sensor_led_onoff_set(gy_sensor_led_off);
	return;
}

//***gy_i2c***//
void gy_i2c_sensor_led_on_with_time(u16 led_on_10ms_count)
{
	gy_i2c_info.sensor_led_on_10ms_count_down = led_on_10ms_count;
	gy_i2c_sensor_led_on();
	return;
}

//***gy_i2c***//
void gy_i2c_sensor_led_10ms_timer(u32 count)
{
	if((count%100) == 0)
	{
		gy_i2c_sensor_led_init();
	}

	if(gy_i2c_info.sensor_led_on_10ms_count_down)
	{
		if(gy_i2c_info.sensor_led_on_10ms_count_down == 1)
		{
			gy_i2c_sensor_led_off();
		}
		gy_i2c_info.sensor_led_on_10ms_count_down--;
	}
	return;
}















//***gy_sub(gy_group)***//
u16 gy_get_sub(u8 sub_type)//获取当前area地址或者sub地址
{//model_sig_g_onoff_level.onoff_srv[idx].com.sub_list[SUB_LIST_MAX]
	u16 gy_sub_addr = 0xffff;
	u8 gy_i_t;
	for(gy_i_t = 0; gy_i_t < SUB_LIST_MAX; gy_i_t++)
	{
		if(model_sig_g_onoff_level.onoff_srv[0].com.sub_list[gy_i_t] >= 0xC000 && model_sig_g_onoff_level.onoff_srv[0].com.sub_list[gy_i_t] <= 0xFEFF)
		{
			gy_sub_addr = model_sig_g_onoff_level.onoff_srv[0].com.sub_list[gy_i_t];
			if(sub_type == gy_sub_type_area)
			{
				if((gy_sub_addr & 0x000F) == 0)
				{
					break;
				}
				gy_sub_addr = 0xffff;
			}
			else if(sub_type == gy_sub_type_sub)
			{
				if((gy_sub_addr & 0x000F) != 0)
				{
					break;
				}
				gy_sub_addr = 0xffff;
			}
		}
	}
	return gy_sub_addr;
}










//***gy_dimmer_curve_transformation***//
u16 gy_dimmer_curve_transformation_calculate(u16 raw_value)//三种调光曲线二次线性变换
{
	if(raw_value != 0)
	{
		u16 gy_transformation_value_t = 0;
		//if(raw_value == 65280)
		//{
		//	gy_transformation_value_t = 65280;
		//}
		//else if(raw_value < 65280)
		{
			u32 y2 = GY_DIMMER_CURVE_TRANSFORMATION_MAX_VALUE*65280/100;
			//u32 y1 = ((u16)GY_DIMMER_CURVE_TRANSFORMATION_MIN_VALUE)*65280/100;
			u32 y1 = (u16)(GY_DIMMER_CURVE_TRANSFORMATION_MIN_VALUE*652.8f);
			u32 x2 = 0;
			u32 x1 = 0;
			if(gy_flash_info.current_curve == gy_curve_linear)//线性调光曲线
			{
				x2 = GY_LINEAR_DIMMER_CURVE_MAX_VALUE*65280/100;
				x1 = GY_LINEAR_DIMMER_CURVE_MIN_VALUE*65280/100;
			}
			else //if(gy_flash_info.current_curve == gy_curve_linear || gy_flash_info.current_curve == gy_curve_linear)//对数和平方调光曲线
			{
				x2 = gy_curve_map[gy_flash_info.current_curve-1][100];
				x1 = gy_curve_map[gy_flash_info.current_curve-1][1];
			}
			//gy_transformation_value_t = ((y2 - y1)*raw_value + x2*y1 - x1*y2)/((x2 - x1));
			gy_transformation_value_t = ((x2 - raw_value)*y1 + (raw_value - x1)*y2)/(x2 - x1);
		}
		return gy_transformation_value_t;
	}
	return 0;
}













//***gy_identity_status***//
void gy_identity_status(void)
{
	cfg_led_event(GY_LED_EVENT_BLINK_JION_MESH);
	return;
}




























//***gy_Multi_group_Control***//
//Property ID(1B:0x08) + No.of Group(1B) + Group 1(2B) + Group 2(2B) + ... + Group n(2B) + Opcode+Parameters(nB)
u8 gy_Multi_group_Control_handle(u8 *par, int par_len)//多组控制处理
{
	u8 err = 0;
	if(par[1] == 0)//如果是广播
	{
		err = 1;
	}

	u8 gy_i_t;
	for(gy_i_t = 0; gy_i_t < SUB_LIST_MAX; gy_i_t++)
	{
		u8 gy_j_t;
		for(gy_j_t = 0; gy_j_t < par[1]; gy_j_t++)
		{
			if(model_sig_g_onoff_level.onoff_srv[0].com.sub_list[gy_i_t] == par[2+2*gy_j_t]+(par[2+2*gy_j_t+1]<<8)/*(u16)*(&par[2] + 2*gy_j_t)*/)
			{
				err = 1;
			}
		}
	}

	if(err)
	{
		if(par[1 + 1 + 2*par[1]] == 0x82)
		{
			u8 gy_uart_send_data[30] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, (u8)ele_adr_primary, (u8)(ele_adr_primary>>8)};
			memcpy(&gy_uart_send_data[8], &par[1 + 1 + 2*par[1]], par_len - (1 + 1 + 2*par[1]));
			mesh_bulk_cmd((mesh_bulk_cmd_par_t *)(gy_uart_send_data), par_len - (1 + 1 + 2*par[1]) + 8);
		}
		else
		{
			mesh_tx_cmd2normal_primary(par[1 + 1 + 2*par[1]], &par[1 + 1 + 2*par[1] + 1], par_len - (1 + 1 + 2*par[1]) - 1, ele_adr_primary, 0);
		}
	}

	return err;
}



































//***gy_probability_switch***//
void gy_probability_switch_handle(u8 *par, int par_len)//概率开关灯
{
	u16 gy_probability_switch_value_t = (u16)rand();
	if(gy_probability_switch_value_t/655 <= par[2])
	{
		u8 gy_data_t[] = {par[1],0x00};
		extern int mesh_cmd_sig_g_onoff_set(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
		mesh_cmd_sig_g_onoff_set(gy_data_t, sizeof(gy_data_t), NULL);
	}
	return;
}

















//***gy_online_status***//
u16 gy_online_status_delay_send_10ms_time_count_down = 0;
void gy_online_status_delay_send_start(void)
{
	gy_online_status_delay_send_10ms_time_count_down = GY_ONLINE_STATUS_DELAY_SEND_10MS_TIME_MAX;
	return;
}

//***gy_online_status***//
void gy_online_status_send(void)
{
    // packet
    u8 st_val_par[MESH_NODE_ST_PAR_LEN] = {0};
    memset(st_val_par, 0xFF, sizeof(st_val_par));
    // led_lum should not be 0, because app will take it to be light off

	mesh_cmd_lightness_st_t gy_rsp = {0};
	extern void mesh_level_u16_st_rsp_par_fill(mesh_cmd_lightness_st_t *rsp, u8 idx, int st_trans_type);
	mesh_level_u16_st_rsp_par_fill(&gy_rsp, 0, ST_TRANS_LIGHTNESS);

    //st_val_par[0] = gy_rsp.remain_t;
    //st_val_par[0] = (0xC0 | light_res_sw_save[0].level[0].onoff);
    //st_val_par[1] = (u8)(s16_to_u16(light_res_sw_save[0].level[0].last)>>0);
    //st_val_par[2] = (u8)(s16_to_u16(light_res_sw_save[0].level[0].last)>>8);
    //st_val_par[3] = 0;
    //st_val_par[4] = 0;
	//st_val_par[0] = ((!!light_res_sw_save[0].level[0].onoff)<<7) + s16_to_u16(light_res_sw_save[0].level[0].last)/655;
	//st_val_par[0] = ((!!light_res_sw_save[0].level[0].onoff)<<7) + s16_to_u16(light_res_sw[0].trans[0].target)/655;
	if(s16_to_u16(light_res_sw[0].trans[0].target) == 0)//关灯，以light_res_sw_save[0].level[0].last作为关灯之前的亮度值
	{
		st_val_par[0] = s16_to_u16(light_res_sw_save[0].level[0].last)/655;
	}
	else//开灯，以light_res_sw[0].trans[0].target作为当前亮度值
	{
		st_val_par[0] = (1<<7) + s16_to_u16(light_res_sw[0].trans[0].target)/655;
	}
	st_val_par[1] = (u8)3000;
	st_val_par[2] = (u8)(3000>>8);

    ll_device_status_update(st_val_par, sizeof(st_val_par));
	return;
}

//***gy_online_status***//
void gy_online_status_10ms_timer(u32 count)
{
	if(gy_online_status_delay_send_10ms_time_count_down)
	{
		if(gy_online_status_delay_send_10ms_time_count_down == 1)
		{
			gy_online_status_send();
		}
		gy_online_status_delay_send_10ms_time_count_down--;
	}
	return;
}
















//***gy_fast_provision***//
void gy_fast_provision_reset_handle(void)
{
	u8 gy_not_need_prov_t = fast_prov.not_need_prov;
	memset(&fast_prov, 0x00, sizeof(fast_prov));
	if(is_provision_success() || gy_not_need_prov_t)
	{
		fast_prov.not_need_prov = 1;
	}
	else
	{
		mesh_device_key_set_default();
		fast_prov.get_mac_en = 1;
	}
	return;
}
















//***gy_pwm_min_duty_cycle***//
float GY_DIMMER_CURVE_TRANSFORMATION_MIN_VALUE = GY_DIMMER_CURVE_TRANSFORMATION_1V_VALUE;//最低调光占空比

//***gy_pwm_min_duty_cycle***//
void gy_dim_pwm_min_duty_cycle_refresh_by_startup_voltage(u16 Startup_Voltage_100mv)//根据启动电压设置（刷新）PWM最低调光占空比
{
	if(Startup_Voltage_100mv > 0 && Startup_Voltage_100mv < 10)
	{//为了以后的功能拓展，将最低调光电压的下限调整到0V以上（文档定义的有效范围为0.5V~2V）
		GY_DIMMER_CURVE_TRANSFORMATION_MIN_VALUE = (((float)Startup_Voltage_100mv)/10.0f)*GY_DIMMER_CURVE_TRANSFORMATION_1V_VALUE;
	}
	else if(Startup_Voltage_100mv > 10 && Startup_Voltage_100mv <= 50)
	{//为了以后的功能拓展，将最低调光电压的上限调整到5V（文档定义的有效范围为0.5V~2V）
		//GY_DIMMER_CURVE_TRANSFORMATION_MIN_VALUE = GY_DIMMER_CURVE_TRANSFORMATION_1V_VALUE + ((((float)Startup_Voltage_100mv)-10.f)/(100.0f-10.0f))*(GY_DIMMER_CURVE_TRANSFORMATION_MAX_VALUE-GY_DIMMER_CURVE_TRANSFORMATION_1V_VALUE);
		GY_DIMMER_CURVE_TRANSFORMATION_MIN_VALUE = GY_DIMMER_CURVE_TRANSFORMATION_1V_VALUE + ((float)Startup_Voltage_100mv - 10.0f)*1.1f;//经实际测试，该计算方法得到的启动电压更精确
	}
	else
	{
		GY_DIMMER_CURVE_TRANSFORMATION_MIN_VALUE = GY_DIMMER_CURVE_TRANSFORMATION_1V_VALUE;
	}
	return;
}













//***gy_factory_reset_method***//
GY_FACTORY_RESET_METHOD_INFO gy_factory_reset_method_info;

//***gy_factory_reset_method***//
void gy_factory_reset_method_init(void)
{
	memset((u8*)&gy_factory_reset_method_info, 0, sizeof(gy_factory_reset_method_info));
	return;
}

//***gy_factory_reset_method***//
void gy_factory_reset_method_10ms_timer(u32 count)
{
	if(count%10 == 0)
	{
		if(gy_factory_reset_method_info.Sub_Command == gy_Sub_Command_reset_confirm && gy_factory_reset_method_info.Delay_Time_100ms)
		{
			gy_factory_reset_method_info.Delay_Time_100ms--;
			if(gy_factory_reset_method_info.Delay_Time_100ms == 0)
			{
				switch(gy_factory_reset_method_info.Factory_Reset_Parameters)
				{
				case gy_Factory_Reset_Parameters_network_and_configurations:
				{
					factory_reset();
					show_factory_reset();
					start_reboot();
					break;
				}
				case gy_Factory_Reset_Parameters_network:
				{
					factory_reset();
					light_ev_with_sleep(3, 1000*1000);
					start_reboot();
					break;
				}
				case gy_Factory_Reset_Parameters_configurations:
				{
					show_factory_reset();
					start_reboot();
					break;
				}
				}
			}
		}
	}
	return;
}

//***gy_factory_reset_method***//
void gy_factory_reset_method_data_handle(u8* data_p)
{
	GY_FACTORY_RESET_METHOD_INFO * gy_factory_reset_method_info_t = (GY_FACTORY_RESET_METHOD_INFO *)data_p;
	switch(gy_factory_reset_method_info_t->Sub_Command)
	{
	case gy_Sub_Command_clear_flag:
	{
		gy_factory_reset_method_init();
		break;
	}
	case gy_Sub_Command_set_flag:
	{
		gy_factory_reset_method_info.Sub_Command = gy_Sub_Command_set_flag;
		break;
	}
	case gy_Sub_Command_reset_confirm:
	{
		if(gy_factory_reset_method_info.Sub_Command == gy_Sub_Command_set_flag)
		{
			gy_factory_reset_method_info.Sub_Command = gy_Sub_Command_reset_confirm;
			gy_factory_reset_method_info.Factory_Reset_Parameters = gy_factory_reset_method_info_t->Factory_Reset_Parameters;
			gy_factory_reset_method_info.Delay_Time_100ms = gy_factory_reset_method_info_t->Delay_Time_100ms;
		}
		break;
	}
	}
	return;
}



void lc_threshold_value_to_lux(u16 threshold_value)
{
	u32 y2 = 0;
	u32 y1 = 0;
	u32 x2 = 0;
	u32 x1 = 0;
	if(threshold_value<=54753)
	{
		x2=54753;
		y1=150;
		y2=351;
	}

	else if(threshold_value>54753 && threshold_value<=55173)
	{
		x1=54753;
		x2=55173;
		y1=351;
		y2=355;
	}

	else if(threshold_value>55173 && threshold_value<=55733)
	{
		x1=55173;
		x2=55733;
		y1=355;
		y2=365;
	}

	else if(threshold_value>55733 && threshold_value<=56223)
	{
		x1=55733;
		x2=56223;
		y1=365;
		y2=375;
	}

	else if(threshold_value>56223 && threshold_value<=56758)
	{
		x1=56223;
		x2=56758;
		y1=375;
		y2=400;
	}

	else if(threshold_value>56758 && threshold_value<=57273)
	{
		x1=56758;
		x2=57273;
		y1=400;
		y2=423;
	}

	else if(threshold_value>57273 && threshold_value<=57786)
	{
		x1=57273;
		x2=57786;
		y1=423;
		y2=445;
	}

	else if(threshold_value>57786 && threshold_value<=58300)
	{
		x1=57786;
		x2=58300;
		y1=445;
		y2=467;
	}

	else if(threshold_value>58300 && threshold_value<=58836)
	{
		x1=58300;
		x2=58836;
		y1=467;
		y2=520;
	}

	else if(threshold_value>58836 && threshold_value<=60371)
	{
		x1=58836;
		x2=60371;
		y1=520;
		y2=645;
	}

	else if(threshold_value>60371 && threshold_value<=61920)
	{
		x1=60371;
		x2=61920;
		y1=645;
		y2=846;
	}

	else if(threshold_value>61920 && threshold_value<=63984)
	{
		x1=61920;
		x2=63984;
		y1=846;
		y2=1200;
	}

	else if(threshold_value>63984 && threshold_value<=65534)
	{
		x1=63984;
		x2=65534;
		y1=1200;
		y2=1500;
	}



	lc_threshold_value_lux=((x2 - threshold_value)*y1 + (threshold_value - x1)*y2)/(x2 - x1);
	LOG_USER_MSG_INFO(0,0,"lc_threshold_value_lux：%d",lc_threshold_value_lux);
}


void lc_clear_pid_para(void)
{
	lc_pid_error=0;
	lc_pid_integral=0;
	lc_pid_error_last=0;
}


void lc_read_lux()
{

	if(gy_i2c_info.sensor_type == gy_i2c_sensor_none)
	{
		return ;
	}
	switch(gy_i2c_info.sensor_type)
	{
	case gy_i2c_sensor_1:
	{
		u8 gy_buff_t[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
		i2c_set_id(GY_LTR_329ALS_01_ADDR);
		i2c_read_series(GY_L_SENSOR_ALS_STATUS_REG,1,&gy_buff_t[0],1);
		if(gy_buff_t[0]&0x04)
		{
			i2c_read_series(GY_L_SENSOR_ALS_DATA_CH1_0REG,1,&gy_buff_t[1],4);
			u16 gy_als_value_t = (gy_buff_t[4]<<8)|gy_buff_t[3];
			if(gy_als_value_t!=0xffff)
			{
					if(gy_flash_info.sensor_property.subid_for_lclcsense_sh==1)
					{
						lc_orig_lux=(u16)(0.118*gy_als_value_t-15.8+0.5);
						if(gy_als_value_t<500)
							lc_orig_lux=43;

					}
					else
					{
						lc_orig_lux=(u16)(0.136*gy_als_value_t-8.447+0.5);
								if(gy_als_value_t<500)
									lc_orig_lux=60;
					}
			}
		}
		break;
	}
	case gy_i2c_sensor_2:
	{
		u8 gy_buff_t[2] = {0xFF, 0xFF};
		i2c_read_series(GY_MW_SENSOR_ALS_ADC1_REG,1,&gy_buff_t[0],2);
		u16 gy_als_raw_value = (gy_buff_t[0]<<8)|gy_buff_t[1];
		if(gy_als_raw_value!=0xffff)
		{
			if(gy_als_raw_value<=40)
				lc_orig_lux=0;
			else if(gy_als_raw_value>40 &&gy_als_raw_value<=3838)
				lc_orig_lux=(u16)(0.093*gy_als_raw_value-3+0.5);
			else if(gy_als_raw_value>3838 && gy_als_raw_value<3890)
				lc_orig_lux=(u16)(6.87*gy_als_raw_value-26022+0.5);
			else if	(gy_als_raw_value>=3890)
				lc_orig_lux=	(u16)(21.063*gy_als_raw_value-81355+0.5);

		}
		break;
	}
	case gy_i2c_sensor_3:
	{
		u8 gy_buff_t[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
		if(0 == gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x05, &gy_buff_t[4], 1))
		{
			gy_h_sun_i2c_reset();
		}
		else
		{

			gy_h_sun_i2c_read(GY_H_SUN_DEV_ADDR, 0x04, &gy_buff_t[3], 1);

			if(gy_buff_t[4] == 0x14)
			{
				u8 gy_als_value = gy_buff_t[3];
				if(gy_als_value>=21)
					lc_orig_lux=(u16)(386.82-1.922*gy_als_value+0.5);
				else if(gy_als_value>=13&&gy_als_value<=20)
					lc_orig_lux=(u16)(903-29.6*gy_als_value+0.5);
				else if(gy_als_value>=7&&gy_als_value<=12)
					lc_orig_lux=(u16)(3261-225.7*gy_als_value+0.5);
				else if(gy_als_value<7)
					lc_orig_lux=1700;

			}
		}
		break;
	}
	}
	lc_current_lux = (u16)(lc_orig_lux * (gy_flash_info.sensor_property.coefficient_of_als_calibration / 10.0) + 0.5);
	if(lc_current_lux>1900)
		lc_current_lux=1900;


}



void gy_light_dim_value_set_1000ms(u16 device_addr, u16 dim_value)
{
	lc_tid++;
	if(lc_tid==0)
		lc_tid=1;
	u8 lc_transtime=0x0a;
	if(gy_flash_info.sensor_property.daylight_ctrl_freq>60)
		lc_transtime=GY_TRANSITION_TIME;
	u8 gy_uart_send_data[] = {0x00, 0x00, 0x00, 0x00, 0x02, 0x00, (u8)device_addr, (u8)(device_addr>>8), 0x82, 0x4D, (u8)dim_value, (u8)(dim_value >> 8),lc_tid, lc_transtime, 0x00};
	mesh_bulk_cmd((mesh_bulk_cmd_par_t *)(gy_uart_send_data), sizeof(gy_uart_send_data));
	return;
}


void gy_set_ctl_onoff(u16 device_addr, u8 light_onoff)//G_ON  G_OFF
{//00 00 00 00 00 00 FF FF 82 03 01 00 00 00
	//access_cmd_onoff(0xffff, 0, light_onoff, CMD_NO_ACK, 0);
	lc_tid++;
	if(lc_tid==0)
		lc_tid=1;
	u8 gy_uart_send_data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, (u8)device_addr, (u8)(device_addr>>8), 0x82, 0x03, light_onoff, lc_tid, 0x00, 0x00};
	mesh_bulk_cmd((mesh_bulk_cmd_par_t *)(gy_uart_send_data), sizeof(gy_uart_send_data));
	return;
}

void lc_individual_daylight_control(u16 lightness)
{
	u16 gy_dev_addr_t = 0x0000;
	switch(gy_flash_info.sensor_property.sensor_device_mode)
	{
	case gy_device_mode_disable:
	case gy_device_mode_occupancy:
	case gy_device_mode_vacancy:
	{
		gy_dev_addr_t = ele_adr_primary;
		break;
	}
	case gy_device_mode_independent_sensing:
	{
		gy_dev_addr_t = gy_get_sub(gy_sub_type_sub);
		if(gy_dev_addr_t == 0xffff)//如果没有sub，则只对自己生效
		{
			gy_dev_addr_t = ele_adr_primary;
		}
		else if( gy_flash_info.sensor_property.sensor_role!=1)
			return;
		break;
	}
	}

	gy_light_dim_value_set_1000ms(gy_dev_addr_t, lightness);
	return;


}

//***gy_flash***//
u8 gy_flash_id_pos[gy_flash_end_id] = {0xFF,0xFF,0xFF};//当前存储信息的相对位置（需要根据实际的数组大小进行赋值）
u8 gy_flash_id_sectors_num[gy_flash_end_id] = {1,1,1};//存储占据多少个扇区（0x1000*n）（需要根据实际的数组大小进行赋值）
u8 gy_flash_id_valid_info_num[gy_flash_end_id] = {1,1,1};//每一组有效数据占据多少128个字节（128*n）（需要根据实际的数组大小进行赋值）
u16 gy_flash_id_info_write_delay_10ms_count_down[gy_flash_end_id] = {0,0,0};//将flash id对应的信息延时一定时间再写入到相应flash区域。单位：10ms（需要根据实际的数组大小进行赋值）

//***gy_flash***//
u16 gy_flash_sectors_size_clc(u8 flash_id)//计算flash id对应分配的flash区域大小
{
	return (gy_flash_id_sectors_num[flash_id]*GY_FLASH_SECTOR_UINT);
}

//***gy_flash***//
u16 gy_flash_valid_info_size_clc(u8 flash_id)//计算flash id对应每组数据分配的空间大小
{
	return (gy_flash_id_valid_info_num[flash_id]*GY_FLASH_VALID_INFO_UNIT);
}

//***gy_flash***//
u8 gy_flash_valid_info_num_max_clc(u8 flash_id)//计算flash id对应分配空间的总数量
{
	return (gy_flash_sectors_size_clc(flash_id)/gy_flash_valid_info_size_clc(flash_id));
}

//***gy_flash***//
u32 gy_flash_current_handle_addr_clc(u8 flash_id, u8 offset)//根据偏移量计算当前flash id操作的地址值
{
//	u8 gy_sector_offset_t = 0;
//	u8 i;
//	for(i = 0; i < flash_id; i++)
//	{
//		gy_sector_offset_t += gy_flash_id_sectors_num[i];
//	}
	return (0x78000 +  offset*gy_flash_valid_info_size_clc(flash_id));
}

//***gy_flash***//
void gy_flash_clear(u8 flash_id)//清除flash id对应的flash区域数据
{
//	u8 i;
//	for(i = 0; i < gy_flash_id_sectors_num[flash_id]; i++)
//	{
//		wd_clear();
//		flash_erase_sector(gy_flash_current_handle_addr_clc(flash_id, 0) + i*GY_FLASH_SECTOR_UINT);
//		sleep_us(500);
//		wd_clear();
//	}
	flash_erase_sector(0x78000);
	return;
}

u8 gy_flash_write_1(u8 flash_id, u8* write_data_p, u16 len)//未配网时也能向flash id对应的区域按序写入有效数据
{
	if(gy_flash_id_pos[flash_id] == 0xFF)//flash中未存储有效数据
	{
		gy_flash_id_pos[flash_id] = 0;
	}
	else if(gy_flash_id_pos[flash_id] < (gy_flash_valid_info_num_max_clc(flash_id) - 1))//flash中存储了有效数据，但是没有存满
	{
		gy_flash_id_pos[flash_id]++;
	}
	else if(gy_flash_id_pos[flash_id] == (gy_flash_valid_info_num_max_clc(flash_id) - 1))//flash中存满了有效数据
	{
		gy_flash_clear(flash_id);
		gy_flash_id_pos[flash_id] = 0;
	}
	else
	{
		return 0;
	}
	flash_write_page(gy_flash_current_handle_addr_clc(flash_id, gy_flash_id_pos[flash_id]), len, write_data_p);
	return 1;
}

u8 gy_flash_read(u8 flash_id, u8* read_data_p, u16 len)//读取flash id对应的最后有效数据和相对位置。如果len = 0，则表示只获取最后有效数据的相对位置，不读取任何数据
{
	if(gy_flash_id_pos[flash_id] == 0xFF)//程序刚启动或者还没有存储有效数据
	{
		u8 gy_flag_read_t = 0xFF;
		u8 gy_valid_info_num_max_t = gy_flash_valid_info_num_max_clc(flash_id);
		u8 i;
		for(i = 0; i < gy_valid_info_num_max_t; i++)
		{
			flash_read_page(gy_flash_current_handle_addr_clc(flash_id, i), 1, &gy_flag_read_t);
			if(gy_flag_read_t == 0xFF)
			{
				if(i == 0)
				{
					return 0;
				}
				else
				{
					break;
				}
			}
		}
		i--;
		gy_flash_id_pos[flash_id] = i;
	}
	if(len)
	{
		flash_read_page(gy_flash_current_handle_addr_clc(flash_id, gy_flash_id_pos[flash_id]), len, read_data_p);
	}
	return 1;
}

GY_FLASH_ENERGY_INFO gy_flash_energy_info;

//***gy_flash_energy***//
void gy_flash_energy_init(void)
{
	gy_flash_energy_read();
	return;
}

//***gy_flash_energy***//
void gy_flash_energy_read(void)
{
	if(gy_flash_read(0, (u8*)&gy_flash_energy_info, sizeof(gy_flash_energy_info)) == 0)
	{
		gy_flash_energy_info.flag = GY_FLASH_VALID_INFO_FLAG;
		gy_flash_energy_info.total_energy = 0.0f;
	}
	return;
}

//***gy_flash_energy***//
void gy_flash_energy_write(void)
{
	gy_flash_write_1(0, (u8*)&gy_flash_energy_info, sizeof(gy_flash_energy_info));//向flash id对应的区域按序写入有效数据
	return;
}
