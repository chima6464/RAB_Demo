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
#include "lc.h"
#include "gy.h"

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


static u32 lc_energy_delay = 0;
static u8  lc_energy_report_delay_start_flag=0;
static u16 lc_unicast_adress = 0;
static u16 lc_threshold_value_lux=0;
static u16 lc_current_lux=0;
static s32 lc_pid_error=0;
static s32 lc_pid_integral=0;
static s32 lc_pid_lightness_s32=0;
static u16 lc_pid_lightness_u16=0;
static s32 lc_pid_error_last=0;
static float lc_Kp=7.0;
static float lc_Ki=5.8;

static u8 lc_pid_is_stable=0;
static u8 lc_tid=1;
static u8 lc_power_on_flag=1;
static u8 lc_pid_change_confirm=0;
static u16 lc_photocell_threshold_high=390;
static u16 lc_orig_lux=0;
static u16 lc_measure_lux=0;
static u8 lc_daylight_calibrate_flag=0;
static u8 lc_photocell_freeze_flag=0;
static u16 lc_photocell_control_count=0;
static u8 lc_photocell_caused_by_motion_sensor_flag=0;
static u16 lc_adjust_interval_count=0;
static u8 lc_als_read_flag=0;
static u8 lc_donot_refresh_occupied_countdown_flag=0;

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

void lc_energy_count_1s_timer(u32 count)
{
#if defined(ENABLE_FEATURE_A)
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
#else
    u16 lc_1min_energy_count=0;
    u16 lc_20min_energy_count=0;
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
#endif
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

u16 lc_read_power_to_calculate_energy(void)
{
    GY_FLASH_FIX_INFO gy_flash_fix_info_t;
	gy_flash_fix_read(&gy_flash_fix_info_t);
	//if(gy_flash_fix_info_t.light_power == 0xFF)//���û��д��̶�������ʣ���Ĭ�������40W
	if(gy_flash_fix_info_t.light_power == 0xFFFF)//���û��д��̶�������ʣ���Ĭ�������40W
	{
		gy_flash_fix_info_t.light_power = 40;
	}

#if(GY_DEV_TYPE == GY_DEV_100F)
	if(gy_flash_fix_info_t.light_power > 1000)//����Ѿ�д��̶�������ʣ�����ж�����ʴ���1000W�����Ե��ֽڼ��㹦��
	{
		gy_flash_fix_info_t.light_power = (u16)((u8)gy_flash_fix_info_t.light_power);
	}
#endif

	//u16 x1 = GY_DIMMER_CURVE_TRANSFORMATION_MIN_VALUE;
	u16 x1 = GY_DIMMER_CURVE_TRANSFORMATION_1V_VALUE;
	u16 x2 = GY_DIMMER_CURVE_TRANSFORMATION_MAX_VALUE;
	u16 y1 = gy_flash_fix_info_t.light_power;//��λ��100mv
	u16 y2 = gy_flash_fix_info_t.light_power*10;//��λ��100mv
	u16 gy_instat_power_100mW_value = 0;

	if(light_res_sw_save[0].level[0].onoff)//������ǿ��ŵ�
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

		/*//�ü��㷽ʽ���Ȳ���
		u16 x =  gy_pwm_level*100/PWM_MAX_TICK;
		gy_instat_power_100mW_value = (y1*(x2 - x)+y2*(x - x1))/(x2 - x1);
		*/

		#define x	gy_pwm_level*100/PWM_MAX_TICK
		/*u16*/u32 gy_instat_power_1mW_value = (y1*(100*x2 - 100*x)+y2*(100*x - 100*x1))/(x2 - x1);
		gy_instat_power_100mW_value = gy_instat_power_1mW_value/100 + ((gy_instat_power_1mW_value%100 >= 50) ? 1 : 0);

		if(gy_instat_power_100mW_value > y2 || gy_level_100 == 100)
		{//�������Ĺ��ʳ������д��ù��� ���� ��ǰ�������������У���ֱ�ӷ���������ֵ
			gy_instat_power_100mW_value = y2;
		}
	}
	else
	{
		gy_instat_power_100mW_value = 5;
	}


	return gy_instat_power_100mW_value;
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


void lc_photocell_freeze_1s_timer(void)
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

void lc_clear_pid_para(void)
{
	lc_pid_error=0;
	lc_pid_integral=0;
	lc_pid_error_last=0;
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
		if(gy_dev_addr_t == 0xffff)//���û��sub����ֻ���Լ���Ч
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
	LOG_USER_MSG_INFO(0,0,"lc_threshold_value_lux��%d",lc_threshold_value_lux);
}

void lc_zero_crossing_detecte_init()
{

	gpio_set_interrupt_init(GY_ZCD_PIN,PM_PIN_PULLDOWN_100K,0,FLD_IRQ_GPIO_RISC0_EN);
//	gpio_set_func(GY_ZCD_PIN, AS_GPIO);
//	gpio_set_input_en(GY_ZCD_PIN, 1);
//	gpio_setup_up_down_resistor(GY_ZCD_PIN, PM_PIN_PULLDOWN_100K);
}