#include "proj/tl_common.h"
#include "vendor/mesh/app.h"
#if MD_PRIVACY_BEA
#include "vendor/mesh_lpn/app.h"
#include "vendor/mesh_provision/app.h"
#include "vendor/mesh_switch/app.h"
#endif

enum
{
	lc_sensor_id_high_daylight_threshold = 1,
	lc_sensor_id_high_daylight_harvesting = 2,
	lc_sensor_id_high_daylight_threshold_with_standby = 0x11

};
extern u16 gy_curve_map[2][101];
void lc_energy_count_1s_timer(u32 count);
void lc_energy_report_10ms_timer(u32 count);
u16 lc_read_power_to_calculate_energy(void);
void lc_light_meter_1s_tmer(void);
void lc_photocell_control_1s_timer(void);
void lc_photocell_freeze_1s_timer(void);
void lc_clear_pid_para(void);
void lc_individual_daylight_control(u16 lightness);
void lc_read_lux(void);
void lc_daylight_close_loop_1s_tmer(void);
void lc_pid_light_control(void);
void lc_threshold_value_to_lux(u16 threshold_value);
void lc_zero_crossing_detecte_init(void);
