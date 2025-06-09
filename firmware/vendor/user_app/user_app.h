/********************************************************************************************************
 * @file     user_app.h
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

#include "proj/tl_common.h"
#include "vendor/mesh/app.h"
#if MD_PRIVACY_BEA
#include "vendor/mesh_lpn/app.h"
#include "vendor/mesh_provision/app.h"
#include "vendor/mesh_switch/app.h"
#endif

/** @addtogroup user_app
 * @{
 */

/** @defgroup user_app
 * @brief Common User Code.
 * @{
 */

// --------- function  ----------------
void cb_user_factory_reset_additional();
void cb_user_proc_led_onoff_driver(int on);

#define GY_LED_EVENT_BLINK_JION_MESH config_led_event(16, 16, 5, LED_MASK) // �����light.c�ļ���proc_led����
// Emergency Event���ܣ��㷨��light.c�ļ�proc_led������Ŀǰ�в�֧��10���ӵĳ���ʱ�䣨������Ϊ390��������һ���ֽڣ���ֻ�������������һ��ʹ�ã�
#define GY_LED_EVENT_EMERGENCY_EVENT config_led_event(12, 12, 0xFF, LED_MASK)

#define P_ST_TRANS(idx, type) (&light_res_sw[idx].trans[type])
#define P_SW_LEVEL_SAVE(idx, type) (&light_res_sw_save[idx].level[type])

extern u16 gy_curve_map[2][101]; // �Զ�������������ߺ�ƽ����������

// �ض���light_dim_refresh����
void light_dim_refresh(int idx); // idx: index of LIGHT_CNT.

void lc_energy_report_10ms_timer(u32 count);

//****gy_user_define_beacon***//
#define GY_USER_DEFINE_BEACON_VERSION 0x01
typedef struct
{
	u8 trans_par_val;
	u8 len;
	u8 type;
	u8 beacon_type;
	u8 uuid[16];
	u8 version;
	u16 pid;
	u16 vid;
	u8 reserve[7];
} GY_DEV_INFO_BEACON;
void gy_dev_info_beacon_send(void);

//***gy_rsp_scan***//
typedef struct
{
	u8 len;
	u8 type;
	u16 cid;
	u8 mac[6];
	u8 version;
	u16 pid;
	u16 vid;
	u8 sensor_type;
	u16 light_id;
	u8 reserve[13];
} GY_SCAN_RSP_INFO;
#define GY_CURRENT_VERSION 0x01
void mesh_scan_rsp_init(void); // ɨ��ظ������ض���

enum
{
	lc_sensor_id_high_daylight_threshold = 1,
	lc_sensor_id_high_daylight_harvesting = 2,
	lc_sensor_id_high_daylight_threshold_with_standby = 0x11

};

//***gy_app_init***//
extern u16 gy_dim_value_record;
void gy_app_init(void);

enum
{
	gy_flash_light_curve_and_sensor_property_id, // �洢�������ߺ͵ƾߵ�sensor����
	gy_flash_smartshift_id,						 // �洢smartshift��Ϣ
	gy_flash_energy_id,							 // �洢������Ϣ
	gy_flash_end_id,
};

#define GY_FLASH_VALID_INFO_FLAG 0x66 // flash�洢��Ч��Ϣ�ı�־λ

extern u8 gy_flash_id_pos[gy_flash_end_id];			   // ��ǰ�洢��Ϣ�����λ��
#define GY_FLASH_SECTOR_UINT 0x1000					   // ������С��Ԫ��BYTE��
extern u8 gy_flash_id_sectors_num[gy_flash_end_id];	   // �洢ռ�ݶ��ٸ�������0x1000*n��
#define GY_FLASH_VALID_INFO_UNIT 128				   // ��Ч���ݵ���С��Ԫ��BYTE��
extern u8 gy_flash_id_valid_info_num[gy_flash_end_id]; // ÿһ����Ч����ռ�ݶ��ٸ���С��Ԫ��128 BYTE����128*n��
#define GY_FLASH_WRITE_DELAY_10MS_COUNT_MAX 300
extern u16 gy_flash_id_info_write_delay_10ms_count_down[gy_flash_end_id]; // ��flash id��Ӧ����Ϣ��ʱһ��ʱ����д�뵽��Ӧflash���򡣵�λ��10ms

u16 gy_flash_sectors_size_clc(u8 flash_id);					  // ����flash id��Ӧ�����flash�����С
u16 gy_flash_valid_info_size_clc(u8 flash_id);				  // ����flash id��Ӧÿ�����ݷ���Ŀռ��С
u8 gy_flash_valid_info_num_max_clc(u8 flash_id);			  // ����flash id��Ӧ����ռ��������
u32 gy_flash_current_handle_addr_clc(u8 flash_id, u8 offset); // ����ƫ�������㵱ǰflash id�����ĵ�ֵַ
void gy_flash_clear(u8 flash_id);							  // ���flash id��Ӧ��flash��������
u8 gy_flash_read(u8 flash_id, u8 *read_data_p, u16 len);	  // ��ȡflash id��Ӧ�������Ч���ݺ����λ�á����len = 0�����ʾֻ��ȡ�����Ч���ݵ����λ�ã�����ȡ�κ�����
u8 gy_flash_write_1(u8 flash_id, u8 *write_data_p, u16 len);  // δ����ʱҲ����flash id��Ӧ��������д����Ч����

typedef struct
{
	u8 flag;
	float total_energy; // �ܵ�������λ���߷�
} GY_FLASH_ENERGY_INFO;
extern GY_FLASH_ENERGY_INFO gy_flash_energy_info;
void gy_flash_energy_init(void);
void gy_flash_energy_read(void);
void gy_flash_energy_write(void);

//***gy_timer***//
void gy_10ms_timer(u32 count);
void gy_1s_timer(u32 count);

void lc_energy_count_1s_timer(u32 count);

//***gy_public***//
#define GY_UNPRO_SENSOR_OCCUPY_1S_TIME_COUNT_DOWN_MAX 1260	   // δ����ʱ��sensor����ʱ2���ӣ�����ʱ��60��ʱ�����ȴ�100%�л���20%
#define GY_UNPRO_SENSOR_OCCUPY_1S_TIME_STEP_ONE_COUNT (60 + 1) // δ����ʱ��һ�׶ε���ʱ��60�룬�����л�Ϊ20%
#define GY_UNPRO_SENSOR_OCCUPY_STEP_ONE_DIM_LEVEL_100 20	   // 20%����
enum
{ // light_blink_finish_state
	gy_light_blink_finish_back,
	gy_light_blink_finish_provision,
	gy_light_blink_finish_add_defualt_group = 2 + 3, // �������3��֮�����Ĭ�Ϸ���
};
typedef struct
{
	u8 light_blink_finish_state;	   // �˱�������ָʾlight.c�ļ�proc_led������ִ���������֮��Ӧ�ûָ�����һ��״̬
	u16 occupied_time_count_down;	   // sensorռ��ʱ�䵹��ʱ����λ���룩
	u16 standby_time_count_down;	   // Occupancyģʽstandby����ʱ����λ���룩
	u8 standby_recorvery_flag;		   // standby֮ǰ�ĵƾ�����״̬�ı�־λ
	u16 dim_value_before_into_standby; // ��¼����standby֮ǰ�ĵƾ�����״̬
	// u8 current_light_onoff_state;//��¼��ǰ�ƾ߿���״̬����OTA��ɣ���������ϵ�Ĭ�Ͽ��ƣ�
	u8 light_ssr_state;
} GY_PUBLIC_INFO;
extern GY_PUBLIC_INFO gy_public_info;
void gy_public_init(void);
void gy_save_dim_value_before_into_standby(void);

u16 lc_read_power_to_calculate_energy(void);

//***gy_vendor_model***//
enum
{
	gy_vendor_opcode_power_get = 0xE1,
	gy_vendor_opcode_power_status = 0xE2,
	gy_vendor_opcode_generic_get = 0xE3,
	gy_vendor_opcode_generic_set = 0xE4,
	gy_vendor_opcode_generic_set_unacknowledged = 0xE5,
	gy_vendor_opcode_generic_status = 0xE6,
	gy_vendor_opcode_sensor_get = 0xE7,
	gy_vendor_opcode_sensor_set = 0xE8,
	gy_vendor_opcode_sensor_status = 0xE9,
};
enum
{
	gy_vendor_property_id_identity = 0x01,
	gy_vendor_property_id_cct_delta = 0x02,
	gy_vendor_property_id_dimming_curve = 0x03,
	gy_vendor_property_id_probability_switch = 0x07, // ���ʿ��ص�
	gy_vendor_property_id_multi_group_control = 0x08,
	gy_vendor_property_id_emergency_event = 0x09,
	gy_vendor_property_id_fast_provision_supplementary_info = 0x0b,
	gy_vendor_property_id_Startup_Voltage = 0x17, // ��͵����ѹ�趨
	gy_vendor_property_id_ONOFF_Toggle = 0x19,
	gy_vendor_property_id_Factory_Reset = 0x1B,			 // ����ɾ��
	gy_vendor_property_id_Runtime_State_Register = 0x1E, // ��ȡ��ǰ�ƾ�ģʽ��״̬��Ϣ��DIM CCT HSL��
	gy_vendor_property_id_rc_control = 0xA0,
	gy_vendor_property_id_rssi_test = 0xF0,
	gy_vendor_property_id_power_meter_self_check = 0xF1,
	gy_vendor_property_id_Fix_Properties = 0xFB,
	gy_vendor_property_id_total_energy = 0x36,
	gy_vendor_property_id_daylight_calibration = 0x42,
};
#define GY_EMERGENCY_EVENT_LED_BLINK_COUNT_MAX 390 // �����¼�ʱ��˸390�δ�Լ��10����
enum
{ // emergency_event_flag
	gy_emergency_event_close,
	gy_emergency_event_set,
	gy_emergency_event_execute,
	gy_emergency_event_forced_stop, // ǿ��ֹͣ
};
typedef struct
{
	u16 emergency_event_flag; // �����¼��ı�־λ
} GY_VENDOR_MODEL_INFO;
extern GY_VENDOR_MODEL_INFO gy_vendor_model_info;
void gy_vendor_model_init(void);
u8 gy_rcv_cmd_is_emergency(u8 *params, int par_len, mesh_cb_fun_par_t *cb_par);
int gy_cb_vendor_power_get_handle(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
int gy_cb_vendor_generic_get_handle(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
int gy_cb_vendor_generic_set_handle(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
int gy_cb_vendor_generic_set_unacknowledged_handle(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
int gy_cb_vendor_sensor_get_handle(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
int gy_cb_vendor_sensor_set_handle(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
#define GY_MESH_CMD_VENDOR_FUNCTION_POWER_GET {gy_vendor_opcode_power_get, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, gy_cb_vendor_power_get_handle, gy_vendor_opcode_power_status}
#define GY_MESH_CMD_VENDOR_FUNCTION_GENERIC_GET {gy_vendor_opcode_generic_get, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, gy_cb_vendor_generic_get_handle, gy_vendor_opcode_generic_status}
#define GY_MESH_CMD_VENDOR_FUNCTION_GENERIC_SET {gy_vendor_opcode_generic_set, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, gy_cb_vendor_generic_set_handle, gy_vendor_opcode_generic_status}
#define GY_MESH_CMD_VENDOR_FUNCTION_GENERIC_SET_UNACK {gy_vendor_opcode_generic_set_unacknowledged, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, gy_cb_vendor_generic_set_unacknowledged_handle, STATUS_NONE}
#define GY_MESH_CMD_VENDOR_FUNCTION_SENSOR_GET {gy_vendor_opcode_sensor_get, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, gy_cb_vendor_sensor_get_handle, gy_vendor_opcode_sensor_status}
#define GY_MESH_CMD_VENDOR_FUNCTION_SENSOR_SET {gy_vendor_opcode_sensor_set, 0, VENDOR_MD_LIGHT_C, VENDOR_MD_LIGHT_S, gy_cb_vendor_sensor_set_handle, gy_vendor_opcode_sensor_status}

void lc_light_meter_1s_tmer();
void lc_photocell_control_1s_timer();
void lc_photocell_freeze_1s_timer();

void gy_set_ctl_onoff(u16 device_addr, u8 light_onoff);

void lc_clear_pid_para(void);
void lc_individual_daylight_control(u16 lightness);
void lc_read_lux();
void lc_daylight_close_loop_1s_tmer();
void lc_pid_light_control();

void gy_light_dim_value_set_1000ms(u16 device_addr, u16 dim_value);
#define GY_TRANSITION_TIME 0x7c // 0b01111100

//***gy_sensor***//
#define GY_MW_SENSITIVITY_VALUE_DEFAULT (65535) //(70*655)//�˸�������Ĭ��ֵ
#define GY_ALS_THREASHOLD_VALUE_DEFAULT (65535) //(2152)//���������Ĭ��ֵ
enum
{ // sensor device mode
	gy_device_mode_disable = 0,
	gy_device_mode_occupancy,
	gy_device_mode_vacancy,
	gy_device_mode_independent_sensing,
	gy_device_mode_sensor_test,
};
enum
{ // Vendor Sensor Properties
	gy_sensor_property_sensor_device_mode = 1,
	gy_sensor_property_unoccupied_time_delay = 2,
	gy_sensor_property_occupied_level = 3,
	gy_sensor_property_unoccupied_level = 4,
	gy_sensor_property_MW_Sensitivity = 5,
	gy_sensor_property_ALS_Threshold = 6,
	gy_sensor_property_light_device_mode = 7,
	gy_sensor_property_override_timeout = 8,
	gy_sensor_property_sensor_freeze_time = 9,
	gy_sensor_property_occupied_onoff = 10,
	gy_sensor_property_unoccupied_onoff = 11,
	gy_sensor_property_standby_guard_level = 15,
	gy_sensor_property_standby_guard_time = 16,

	gy_sensor_property_debounce_time = 17,
	gy_sensor_property_ambient_light_intensity = 18,
	gy_sensor_property_daylight_open_high = 19,
	gy_sensor_property_daylight_open_low = 20,
	gy_sensor_property_target_light_level = 21,
	gy_sensor_property_daylight_ctrl_freq = 22,
	gy_sensor_property_occupied_action = 23,
	gy_sensor_property_vacancy_action = 24,
	gy_sensor_property_daylight_harvesting = 25,
	gy_sensor_property_daylight_threshold_high = 26,
	gy_sensor_property_daylight_threshold_low = 27,
	gy_sensor_property_light_intensity_extra = 28,
	gy_sensor_property_sensor_role = 29,
	gy_sensor_property_pid_Kp_one_tenth = 30,
	gy_sensor_property_pid_Ki_one_tenth = 31,
	gy_sensor_property_photocell_onoff_control = 32,

	gy_sensor_property_subid_for_lclcsense_sh = 33,

	gy_sensor_property_coefficient_of_als_calibration = 34,
	gy_sensor_property_photocell_threshold = 35,
	gy_sensor_property_photocell_threshold_ratio = 36,
	gy_sensor_property_photocell_freeze_time = 37,
};
typedef struct
{
	u16 sensor_device_mode;
	u16 unoccupied_time_delay;
	u16 occupied_level;
	u16 unoccupied_level;
	u16 MW_Sensitivity;
	u16 ALS_Threshold;
	u16 light_device_mode;
	u16 override_timeout;
	u16 sensor_freeze_time;
	u16 occupied_onoff;
	u16 unoccupied_onoff;
	u16 standby_guard_level;
	u16 standby_guard_time;

	u16 debounce_time; // unit:100ms
	u16 ambient_light_intensity;
	u16 daylight_open_high;
	u16 daylight_open_low;
	u16 target_light_level;
	u16 daylight_ctrl_freq;
	u16 occupied_action;
	u16 vacancy_action;
	u16 daylight_harvesting;
	u16 daylight_threshold_high;
	u16 daylight_threshold_low;
	u16 light_intensity_extra;
	u16 sensor_role;
	u16 pid_Kp_one_tenth;
	u16 pid_Ki_one_tenth;
	u16 photocell_onoff_control;

	u16 subid_for_lclcsense_sh;

	u16 coefficient_of_als_calibration;
	u16 photocell_threshold;
	u16 photocell_threshold_ratio;
	u16 photocell_freeze_time;
} GY_SENSOR_PROPERTY;
enum
{							// sensor_state
	gy_sensor_state_no_one, // ��⵽û��
	gy_sensor_state_people, // ��⵽����
};
void gy_sensor_occupied_time_count_refresh(void);
void gy_sensor_occupied_time_count_clear(void);
// void gy_sensor_occupied_time_handle_ctl_light(int st_trans_type);
void gy_sensor_occupied_handle(mesh_cb_fun_par_t *cb_par);
void gy_sensor_unoccupied_handle(mesh_cb_fun_par_t *cb_par);
void gy_sensor_occupied_time_count_down_1s_tmer(u32 count);
void gy_sensor_sig_mesh_property_set(u16 property_id, u8 *property_value);
void gy_sensor_status_send(u8 sensor_state);
void gy_sensor_close(void);

void lc_threshold_value_to_lux(u16 threshold_value);

void gy_i2c_sensor_als_meter_10ms_timer(u32 count);

//***gy_flash***//
#define GY_FLASH_INFO_ADDR 0x79000
enum
{
	gy_curve_linear,	  // ����
	gy_curve_logarithmic, // ����
	gy_curve_square,	  // ƽ��
};
typedef struct
{
	u8 current_curve;
	// u8 read_i2c_data[2];
	u8 fast_power_onoff_flag;			// ���ٿ��صƵı�־λ���������أ��������أ����εƾ߱�Ϊ100%���ȣ�
	GY_SENSOR_PROPERTY sensor_property; // �������������
} GY_FLASH_INFO;
extern GY_FLASH_INFO gy_flash_info;
#define GY_FLASH_79000_WRITE_10MS_COUNT_MAX 200		  // ����flash����һ��ʱ��֮���ٽ����ݴ洢��flash�У���λ��10ms
extern u16 gy_flash_79000_write_10ms_time_count_down; // ����ʱ����֮���ٽ�flash���ݴ洢��flash��
void gy_flash_init(void);
void gy_flash_info_delay_write_start(void);
void gy_flash_info_write(void);
// void gy_flash_reset(void);
void gy_flash_1s_timer(u32 count);
void gy_flash_10ms_timer(u32 count);
void gy_flash_factory_reset(void);

//***gy_flash_fix***//
#define GY_FLASH_FIX_INFO_ADDR 0x7A000
typedef struct
{
	u16 light_id;			// �Ƶ�ID
	/*u8*/ u16 light_power; // ������������ֵ��10Vʱ�Ĺ���ֵ��
} GY_FLASH_FIX_INFO;
void gy_flash_fix_read(GY_FLASH_FIX_INFO *flash_fix_info);
void gy_flash_fix_write(GY_FLASH_FIX_INFO *flash_fix_info);
void gy_flash_fix_factory_reset(void);

//***gy_flash_new***//
#define GY_FLASH_NEW_INFO_ADDR 0x7B000
#define GY_FLASH_NEW_INFO_WRITE_10MS_COUNT_MAX 200	// ����flash����һ��ʱ��֮���ٽ����ݴ洢��flash�У���λ��10ms
extern u16 gy_flash_new_info_write_10ms_count_down; // ����ʱ����֮���ٽ�flash���ݴ洢��flash��
typedef struct
{
	u16 Startup_Voltage_100mv; // ����������Ƶ�ѹ����λ��0.1V��Ĭ��ȡֵ10
} GY_FLASH_NEW_INFO;
extern GY_FLASH_NEW_INFO gy_flash_new_info;
void gy_flash_new_init(void);
void gy_flash_new_info_delay_write_start(void);
void gy_flash_new_info_read(void);
void gy_flash_new_info_write(void);
void gy_flash_new_1s_timer(u32 count);
void gy_flash_new_10ms_timer(u32 count);
void gy_flash_new_factory_reset(void);

void gy_flash_will_not_erase_init(void);
void gy_flash_will_not_erase_info_read(void);
void gy_flash_will_not_erase_info_write(void);

#define GY_FLASH_WILL_NOT_ERASE_INFO_ADDR 0x7C000
typedef struct
{

	float lc_total_energy; // watt*min
} GY_FLASH_WILL_NOT_ERASE_INFO;
extern GY_FLASH_WILL_NOT_ERASE_INFO gy_flash_will_not_erase_info;

//***gy_FCT_opcode_handle***//
// #define GY_I2C_SLAVE_ADDR		0x60
// #define GY_I2C_DATA				0x66//I2Cͨ�Ŵ�������Թ�װ�����ݣ�������յ����������ʾͨ������
enum
{
	gy_test_step_idle,
	gy_test_step_1, // Dongle ���͡����⿪ʼ��ָ����������������յ�ָ��󣬽������ģʽ
	gy_test_step_2, // RSSI ����
	gy_test_step_3, // Relay �ر�
	gy_test_step_4, // Relay ��
	gy_test_step_5, // 0~10V ��� 1V
	gy_test_step_6, // 0~10V ��� 5V
	gy_test_step_7, // 0~10V ��� 10V
	gy_test_step_8, // RJ09 ���ԣ�ʹ����ʵ��sensor���ԣ�
};
enum
{						   // sensor_test_type
	gy_sensor_test_idle,   // ��������������
	gy_sensor_test_motion, // �������˸в���
	gy_sensor_test_ALS,	   // ��������в���
};
typedef struct
{
	s8 rcv_rssi;
	u8 current_test_step; // ��ǰ���Բ���
	u8 test_result;		  // ���Խ��
	u8 sensor_test_type;
	// u16 motion_sensor_threshold;//�˸�������
	// u16 ALS_sensor_threshold;//���������
} GY_FCT_OPCODE_HANDLE_INFO;
extern GY_FCT_OPCODE_HANDLE_INFO gy_fct_opcode_handle_info;
enum
{
	gy_FCT_opcode_RSSI_test = 0xFF,
	gy_FCT_opcode_power_metering_test = 0xFE,
	gy_FCT_opcode_FCT_mode = 0xFD,
	gy_FCT_opcode_sensor_test = 0xFC,
	gy_FCT_opcode_set_fixture_properties = 0xFB,
	gy_FCT_opcode_get_i2c_data = 0xCC,
	gy_FCT_opcode_i2c_chip_reset = 0xCD,
	gy_FCT_opcode_read_sensor_count_down = 0xCE,
};
void gy_FCT_init(void);
void gy_FCT_opcode_handle(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);

//***gy_sub(gy_group)***//
u8 gy_is_sub(u16 sub_addr); // �ж��Ƿ���䵽���鲥��ַ

//***gy_scene***//
enum
{
	gy_scene_No1 = 1,
	gy_scene_No2 = 2,
	gy_scene_No3 = 3,
};
enum
{
	gy_scene_No1_lightness_value = -32111, // 1% 65535*0.01-32767
	gy_scene_No2_lightness_value = 0,	   // 50%
	gy_scene_No3_lightness_value = 32767,  // 100%
};
void gy_three_defult_scene_init(void);
void gy_three_defult_scene_set_10ms_timer(u32 count);
void gy_scene_recovery(u16 scene_No);
u8 gy_scene_del_handle(u16 scene_No);

//***gy_led***//
unsigned short gy_calc_pwm_duty_by_100_level(u8 level); // ����PWM�ٷֱ�
#define GY_PWM_DUTY(a) gy_calc_pwm_duty_by_100_level(a) // ����PWM�ٷֱ�
#define GY_LED_ON GY_PWM_DUTY(100)
#define GY_LED_OFF GY_PWM_DUTY(0)
enum
{
	gy_led_off,
	gy_led_on,
};
typedef struct
{
	u8 r_state;
	u8 g_state;
} GY_LED;
extern GY_LED gy_led;
void gy_led_init(void);
void gy_set_led_r_state(u8 state);
void gy_set_led_g_state(u8 state);
void gy_led_10ms_timer(u32 count);

//***gy_button***//
#define GY_DOUBLE_CLICK_INTERVAL_10MS_TIME_MAX 50 // ʵ��˫����ť����������һ�ζ̰������ɿ���ʼ��ʱ��50*10ms֮���ٴζ̰���������˫������
#define GY_XIAODOU_10MS_TIME_MAX 5				  // 5*10ms����ʱ��
#define GY_SHORT_PRESS_TIME 50					  // 50*10ms���ɿ�������Ϊ�̰�
enum
{
	gy_button_release,
	gy_button_press,
};
typedef struct
{
	u8 state;
	u32 press_10ms_count;
	u16 press_1s_count;
	u16 double_click_interval_count; // ˫����ť֮���ʱ����
	u8 double_click_flag;			 // 1��ʾ�����ɿ�֮��ִ��˫�����ܣ�0��ʾ�����ɿ�֮��ִ��˫������
	u8 double_click_and_hold_flag;	 // 1��ʾִ�а���˫�������ֳ������ܣ�0��ʾ˫�������ֳ������ܽ���
} GY_BUTTON;
extern GY_BUTTON gy_button;
void gy_button_init(void); // ������ʼ��
u8 gy_read_button(void);   // ��ȡ��ť״̬
void gy_button_10ms_timer(u32 count);
void gy_button_1s_timer(u32 count);
void gy_button_double_click_and_hold_10ms_timer(u32 count);

//***gy_ctl_light***//
enum
{
	gy_light_off,
	gy_light_on,
};
typedef struct
{
	u8 state;
} GY_LIGHT_INFO;
extern GY_LIGHT_INFO gy_light_info;
void gy_light_init(void);
void gy_light_onoff_set(u16 device_addr, u8 light_onoff);
void gy_light_onoff_set_new(u16 device_addr, u8 light_onoff, u8 delay_time);
void gy_light_dim_delta_set(u16 device_addr, s32 dim_delta);
void gy_light_dim_value_set(u16 device_addr, u16 dim_value);
void gy_light_self_onoff_set(void);
void gy_light_self_dim_delta_set(s32 dim_delta);
void gy_light_self_dim_value_set(u16 dim_value);

//***gy_relay***//
enum
{
	gy_relay_off,
	gy_relay_on,
};
typedef struct
{
	u8 state;
} GY_RELAY_INFO;
extern GY_RELAY_INFO gy_relay_info;
void gy_relay_init(void); // vendor/mesh/app.c�ļ�user_init�����е���
void gy_relay_onoff_set(u8 relay_state);

void lc_zero_crossing_detecte_init();

//***gy_i2c***//
//*********************************RAB���´�����********************************//
#define GY_I2C_CLOCK (unsigned char)(CLOCK_SYS_CLOCK_HZ / (2 * 200000))
#define GY_MCP3021A5T_ADDR (0x4D << 1)
#define GY_LTR_329ALS_01_ADDR (0x29 << 1)
#define GY_L_SENSOR_ALS_CONTR_REG (0x80)
#define GY_L_SENSOR_ALS_CONTR_VALUE (0x1D)
#define GY_ALS_MEAS_RATE_REG (0x85)
#define GY_ALS_MEAS_RATE_VALUE (0x02)
#define GY_L_SENSOR_ALS_DATA_CH1_0REG (0x88)
#define GY_L_SENSOR_ALS_DATA_CH1_1REG (0x89)
#define GY_L_SENSOR_ALS_DATA_CH0_0REG (0x8A)
#define GY_L_SENSOR_ALS_DATA_CH0_1REG (0x8B)
#define GY_L_SENSOR_ALS_STATUS_REG (0x8C)
// #define GY_SENSOR_TRIGGER_TIME_COUNT_MAX	120//10//����������֮���ά��ʱ�䣺120*10*50ms
#define GY_PCA9536_ADDR (0x41 << 1)
#define GY_PCA9536_INPUT_PORT_REG (0x00)
#define GY_PCA9536_OUTPUT_PORT_REG (0x01)
#define GY_PCA9536_POLARITY_INVERSION_REG (0x02)
#define GY_PCA9536_CONFIGURATION_REG (0x03)

//*********************************��о΢**********************************//
#define GY_H_SUN_DEV_ADDR 0x5A				  // 0x5A//0x12
#define GY_H_SUN_FRAME_INTERVAL sleep_us(270) // ֡���270us
#define GY_H_SUN_QUARTER_CYCLE sleep_us(10)	  // �ķ�֮һ��ʱ�����ڣ�10us��Ƶ��25KHz
#define GY_H_SUN_SCK_H gpio_write(GY_I2C_SCK_PIN, 1)
#define GY_H_SUN_SCK_L gpio_write(GY_I2C_SCK_PIN, 0)
#define GY_H_SUN_SDA(x) gpio_write(GY_I2C_SDA_PIN, x)

#define GY_H_SUN_PIR_HIGH_SENSITIVITY_MAX 25 // ��о΢�˸����������

//*********************************΢��������********************************//
#define GY_MW_SENSOR_ADDR (0x5B << 1)
#define GY_MW_SENSOR_TEST_REG 0xC0
#define GY_MW_SENSOR_TEST_VALUE1 0x66
#define GY_MW_SENSOR_TEST_VALUE2 0x88
#define GY_MW_SENSOR_MOTION_STATE_REG 0xC1
enum
{
	gy_mw_sensor_motion_state_people_no,
	gy_mw_sensor_motion_state_people_yes,
};
#define GY_MW_SENSOR_ALS_ADC0_REG 0xC3
#define GY_MW_SENSOR_ALS_ADC1_REG 0xC5
#define GY_MW_SENSOR_MOTION_SENSITIVITY_REG 0xC7
#define GY_MW_SENSOR_LED_CTL_REG 0xC9

enum
{ // sensor_type
	gy_i2c_sensor_none,
	gy_i2c_sensor_1, // RAB�Լ�������PIR
	gy_i2c_sensor_2, // ΢���˸�
	gy_i2c_sensor_3, // ��о΢оƬ���Ĵ�����
};
enum
{ // sensor_flag
	gy_i2c_sensor_disable,
	gy_i2c_sensor_enable,
};
#define GY_SENSOR_I2C_SET_FINISH 0x66 // sensor_set_state
typedef struct
{
	u8 sensor_type;
	// u8 sensor_flag;
	// u16 sensor_trigger_count_down;//����������֮���ά��ʱ�䵹��ʱ
	u8 sensor_set_state;				   // ���ʶ�𵽴�����������Ҫ��ʼ���ô��������������������Ҫ���û���������ɣ��ñ�����ֵGY_SENSOR_I2C_SET_FINISH
	u8 sensor_event_check_10ms_count_down; // sensor event�ٴμ�⵹��ʱ
	u8 sensor_event_recheck_count_down;	   // sensor event�¼��ظ�������
	u16 sensor_led_on_10ms_count_down;	   // sensorָʾ�ƿ���״̬����ʱ
} GY_I2C_INFO;
extern GY_I2C_INFO gy_i2c_info;
extern u16 gy_sensor_freeze_time_count_down;

//*********************************RAB���´�����********************************//
void gy_i2c_read_MCP3021A5T(u8 *buff);

//*********************************��о΢**********************************//
u8 gy_h_sun_i2c_ack_handle(void);
void gy_h_sun_i2c_write_one_byte(u8 w_data);
u8 gy_h_sun_i2c_write_one_byte_and_ack(u8 w_data);
u8 gy_h_sun_i2c_write(u8 dev_addr, u8 reg_addr, u8 *write_buff, u8 len);
u8 gy_h_sun_i2c_read(u8 dev_addr, u8 reg_addr, u8 *read_buff, u8 len);
void gy_h_sun_i2c_reset(void);

// ��������Ϊ�����о΢�������ڿ��ٶ��ON-OFF-ON-OFF����ʱ�����˸��󴥷�������
#define GY_H_SUN_OFF_ON_OFF_FREEZE_TIME 2 // ���ÿ���ON-OFF-ON-OFF����ʱ��freeze timeʱ��
#define GY_H_SUN_OFF_ON_OFF_10MS_COUNT_DOWN_MAX 200
extern u16 gy_h_sun_off_on_off_10ms_count_down; // ��ʼ��Ϊ0
void gy_h_sun_off_on_off_10ms_timer(u32 count);
void gy_h_sun_off_on_off_count_down_refresh(void);
void gy_h_sun_off_on_off_count_down_clear(void);

#define GY_READ_I2C_DATA_INTERVAL 10 // ÿ���10*10msʱ���ȡһ��i2c����
void gy_i2c_check_sensor_type(void);
void gy_i2c_init(void);
u8 gy_i2c_read_motion_data_and_return_state(void); // ��ȡsensor���˸����ݣ����ҷ��ص�ǰ�Ƿ����㴥��������0��ʾ�����������������ʾ����������
u8 gy_i2c_read_als_data_and_return_state(void);	   // ��ȡsensor�Ĺ�����ݣ����ҷ��ص�ǰ�Ƿ����㴥��������0��ʾ�����������������ʾ����������
u8 gy_i2c_read_data_and_return_state(void);		   // ��ȡsensor���ݣ����ҷ��ص�ǰ�Ƿ����㴥��sensor��������0��ʾ�����������������ʾ����������
void gy_i2c_sensor_event_time_count_down_handle(u8 check_10ms_count_down, u8 recheck_count_down);
void gy_i2c_10ms_timer(u32 count);
void gy_i2c_sensor_check_10ms_timer(u32 count);	  // ��⴫�����Ƿ���ڣ��Լ�ȷ�ϴ���������
void gy_i2c_sensor_set_10ms_timer(u32 count);	  // �����⵽����������Ҫ���ݴ��������ͶԴ����������趨
void gy_i2c_sensor_execute_10ms_timer(u32 count); // ������ִ�г���
void gy_i2c_sensor_event_handle_10ms_timer(u32 count);
enum
{
	gy_sensor_led_off,
	gy_sensor_led_on,
};
void gy_i2c_sensor_led_init(void);
void gy_i2c_sensor_led_onoff_set(u8 led_onoff);
void gy_i2c_sensor_led_on(void);
void gy_i2c_sensor_led_off(void);
void gy_i2c_sensor_led_on_with_time(u16 led_on_10ms_count);
void gy_i2c_sensor_led_10ms_timer(u32 count);

//***gy_dimmer_curve_transformation***//
#define GY_LINEAR_DIMMER_CURVE_MIN_VALUE 1					 // ���Ե�����С�ٷֱ�
#define GY_LINEAR_DIMMER_CURVE_MAX_VALUE 100				 // ���Ե������ٷֱ�
u16 gy_dimmer_curve_transformation_calculate(u16 raw_value); // ���ֵ������߶������Ա任

//***gy_sub(gy_group)***//
#define GY_DEFAULT_GROUP 0xC000
void gy_set_group(u16 sub_group); // ���÷���
void gy_set_default_group(void);  // ����Ĭ�Ϸ���
void gy_set_default_group_1s_timer(u32 count);
enum
{
	gy_sub_type_area,
	gy_sub_type_sub,
};
u16 gy_get_sub(u8 sub_type); // ��ȡ��ǰarea��ַ����sub��ַ

//***gy_identity_status***//
void gy_identity_status(void);

//***gy_Multi_group_Control***//
// Property ID(1B:0x08) + No.of Group(1B) + Group 1(2B) + Group 2(2B) + ... + Group n(2B) + Opcode+Parameters(nB)
u8 gy_Multi_group_Control_handle(u8 *par, int par_len); // ������ƴ���

//***gy_probability_switch***//
void gy_probability_switch_handle(u8 *par, int par_len); // ���ʿ��ص�

//***gy_online_status***//
#define GY_ONLINE_STATUS_DELAY_SEND_10MS_TIME_MAX 200
extern u16 gy_online_status_delay_send_10ms_time_count_down;
void gy_online_status_delay_send_start(void);
void gy_online_status_send(void);
void gy_online_status_10ms_timer(u32 count);

//***gy_fast_provision***//
void gy_fast_provision_reset_handle(void);

//***gy_pwm_min_duty_cycle***//
extern float GY_DIMMER_CURVE_TRANSFORMATION_MIN_VALUE;								  // ��͵���ռ�ձ�
void gy_dim_pwm_min_duty_cycle_refresh_by_startup_voltage(u16 Startup_Voltage_100mv); // ����������ѹ���ã�ˢ�£�PWM��͵���ռ�ձ�

//***gy_factory_reset_method***//
enum
{
	gy_Sub_Command_clear_flag,
	gy_Sub_Command_set_flag,
	gy_Sub_Command_reset_confirm,
};
enum
{
	gy_Factory_Reset_Parameters_network_and_configurations,
	gy_Factory_Reset_Parameters_network,
	gy_Factory_Reset_Parameters_configurations,
};
typedef struct
{
	u8 Sub_Command;
	u8 Factory_Reset_Parameters;
	u8 Delay_Time_100ms;
} GY_FACTORY_RESET_METHOD_INFO;
extern GY_FACTORY_RESET_METHOD_INFO gy_factory_reset_method_info;
void gy_factory_reset_method_init(void);
void gy_factory_reset_method_10ms_timer(u32 count);
void gy_factory_reset_method_data_handle(u8 *data_p);

/**
 * @}
 */

/**
 * @}
 */
