/*
 * Fuelgauge battery driver
 *
 * Copyright (C) 2018 Siliconmitus
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#define pr_fmt(fmt)	"[sm5602g] %s(%d): " fmt, __func__, __LINE__
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <asm/unaligned.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/poll.h>

#include "sm5602g_fg.h"

#include "../oplus_charger.h"
#include "../oplus_gauge.h"
#include "../oplus_vooc.h"
#include "../oplus_debug_info.h"



#undef pr_debug
#define pr_debug pr_err
#undef pr_info
#define pr_info pr_err
#undef dev_dbg
#define dev_dbg dev_err

#define QC_PFM
//#define MTK_PFM

#define	INVALID_REG_ADDR	0xFF
#define RET_ERR 			-1

//#define ENABLE_TEM_RATE_CONTROL

#define EX_TEMP_MIN			(-40)
#define EX_TEMP_MAX			85
#define SM_INFO_LEN         255
#define SM_NAME_LEN         12

enum sm_fg_reg_idx {
	SM_FG_REG_DEVICE_ID = 0,
	SM_FG_REG_CNTL,
	SM_FG_REG_INT,
	SM_FG_REG_INT_MASK,
	SM_FG_REG_STATUS,
	SM_FG_REG_SOC,
	SM_FG_REG_OCV,
	SM_FG_REG_VOLTAGE,
	SM_FG_REG_CURRENT,
	SM_FG_REG_TEMPERATURE_IN,
	SM_FG_REG_TEMPERATURE_EX,
	SM_FG_REG_V_L_ALARM,
	SM_FG_REG_V_H_ALARM,	
	SM_FG_REG_A_H_ALARM,
	SM_FG_REG_T_IN_H_ALARM,
	SM_FG_REG_SOC_L_ALARM,
	SM_FG_REG_FG_OP_STATUS,
	SM_FG_REG_TOPOFFSOC,
	SM_FG_REG_PARAM_CTRL,
	SM_FG_REG_SHUTDOWN,	
	SM_FG_REG_VIT_PERIOD,
	SM_FG_REG_CURRENT_RATE,
	SM_FG_REG_RSNS_HR,	
	SM_FG_REG_BAT_CAP,
	SM_FG_REG_CURR_OFFSET,
	SM_FG_REG_CURR_SLOPE,	
	SM_FG_REG_MISC,
	SM_FG_REG_RESET,
	SM_FG_REG_RSNS_SEL,
	SM_FG_REG_VOL_COMP,
	NUM_REGS,
};

static u8 sm5602g_regs[NUM_REGS] = {
	0x00, /* DEVICE_ID */
	0x01, /* CNTL */
	0x02, /* INT */
	0x03, /* INT_MASK */
	0x04, /* STATUS */
	0x05, /* SOC */
	0x06, /* OCV */
	0x07, /* VOLTAGE */
	0x08, /* CURRENT */
	0x09, /* TEMPERATURE_IN */
	0x0A, /* TEMPERATURE_EX */
	0x0C, /* V_L_ALARM */
	0x0D, /* V_H_ALARM */	
	0x0E, /* A_H_ALARM */
	0x0F, /* T_IN_H_ALARM */
	0x10, /* SOC_L_ALARM */
	0x11, /* FG_OP_STATUS */
	0x12, /* TOPOFFSOC */
	0x13, /* PARAM_CTRL */
	0x14, /* SHUTDOWN */
	0x1A, /* VIT_PERIOD */
	0x1B, /* CURRENT_RATE */
	0x1F, /* RSNS_HR */
	0x62, /* BAT_CAP */	
	0x73, /* CURR_OFFSET */	
	0x74, /* CURR_SLOPE */
	0x90, /* MISC */
	0x91, /* RESET */
	0x95, /* RSNS_SEL */
	0x96, /* VOL_COMP */
};

enum sm_fg_device {
	SM5602G,
};

enum sm_fg_temperature_type {
	TEMPERATURE_IN = 0,
	TEMPERATURE_EX,
	TEMPERATURE_3RD,
};

const unsigned char *device2str[] = {
	"sm5602g",
};

enum battery_table_type {
	BATTERY_TABLE0 = 0,
	BATTERY_TABLE1,
	BATTERY_TABLE2,
	BATTERY_TABLE_MAX,
};

int tex_sim_uV[43] = {
	 	0,		 270,       480,        510, 	   910,      
	 1180,      1330,      2120,       2220, 	  2400,      
	 2660,      3290,      4000,       4790, 	  5910,      
	 6650,      7440,      8000,      11670, 	 11940,     
	13260,     13350,     14370,      16040, 	 20930,     
	23760,     26370,     28720,      42490, 	 47060,     
	64860,     75410,     87080,     114290, 	126320,    
   152320,    218180,    229150,     383190, 	508830,    
   783080,   1024200,    1374360
};

short tex_sim_adc_code[43] = {
   0x8001, 0x8D10,    0x8D1B,     
   0x8D1C, 0x8D30,    0x8D3E,    0x8D45,    0x8D6B,     
   0x8D70, 0x8D78,    0x8D86,    0x8DA5,    0x8DC8,     
   0x8DEE, 0x8E25,    0x8E4A,    0x8E70,    0x8E8B,     
   0x8F40, 0x8F4D,    0x8F8F,    0x8F93,    0x8FC4,     
   0x9018, 0x9106,    0x9192,    0x9213,    0x9288,     
   0x9530, 0x960B,    0x997E,    0x9B85,    0x9DD5,     
   0xA2FB, 0xA573,    0xAA5F,    0xB70C,    0xB9E2,     
   0xD720, 0xEFF9,    0x222B,    0x452A,    0x7FC0
};     

int tex_meas_uV[249] = {
		  0,    5000,   10000,   15000,   20000, 
	  25000,   30000,   35000,   40000,   45000, 
	  50000,   55000,   60000,   65000,   70000, 
	  75000,   80000,   85000,   90000,   95000, 
	 100000,  105000,  110000,  115000,  120000, 
	 125000,  130000,  135000,  140000,  145000, 
	 150000,  155000,  160000,  165000,  170000, 
	 175000,  180000,  185000,  190000,  195000, 
	 200000,  205000,  210000,  215000,  220000, 
	 225000,  230000,  235000,  240000,  245000, 
	 250000,  255000,  260000,  265000,  270000, 
	 275000,  280000,  285000,  290000,  295000, 
	 300000,  305000,  310000,  315000,  320000, 
	 325000,  330000,  335000,  340000,  345000, 
	 350000,  355000,  360000,  365000,  370000, 
	 375000,  380000,  385000,  390000,  395000, 
	 400000,  405000,  410000,  415000,  420000, 
	 425000,  430000,  435000,  440000,  445000, 
	 450000,  455000,  460000,  465000,  470000, 
	 475000,  480000,  485000,  490000,  495000, 
	 500000,  505000,  510000,  515000,  520000, 
	 525000,  530000,  535000,  540000,  545000, 
	 550000,  555000,  560000,  565000,  570000, 
	 575000,  580000,  585000,  590000,  595000, 
	 600000,  605000,  610000,  615000,  620000, 
	 625000,  630000,  635000,  640000,  645000, 
	 650000,  655000,  660000,  665000,  670000, 
	 675000,  680000,  685000,  690000,  695000, 
	 700000,  705000,  710000,  715000,  720000, 
	 725000,  730000,  735000,  740000,  745000, 
	 750000,  755000,  760000,  765000,  770000, 
	 775000,  780000,  785000,  790000,  795000, 
	 800000,  805000,  810000,  815000,  820000, 
	 825000,  830000,  835000,  840000,  845000, 
	 850000,  855000,  860000,  865000,  870000, 
	 875000,  880000,  885000,  890000,  895000, 
	 900000,  905000,  910000,  915000,  920000, 
	 925000,  930000,  935000,  940000,  945000, 
	 950000,  955000,  960000,  965000,  970000, 
	 975000,  980000,  985000,  990000,  995000, 
	1000000, 1005000, 1010000, 1015000, 1020000, 
	1025000, 1030000, 1035000, 1040000, 1045000, 
	1050000, 1055000, 1060000, 1065000, 1070000, 
	1075000, 1080000, 1085000, 1090000, 1095000, 
	1100000, 1105000, 1110000, 1115000, 1120000, 
	1125000, 1130000, 1135000, 1140000, 1145000, 
	1150000, 1155000, 1160000, 1165000, 1170000, 
	1175000, 1180000, 1185000, 1190000, 1195000, 
	1200000, 1205000, 1210000, 1215000, 1220000, 
	1225000, 1230000, 1235000, 1240000
}; 

short tex_meas_adc_code[249] = {
	0x8D18, 0x8DEF, 0x8ED4, 0x8FB5, 0x909C, 
	0x917F, 0x9267, 0x9348, 0x9430, 0x9516, 
	0x95FC, 0x96DF, 0x97C2, 0x98AA, 0x998B, 
	0x9A70, 0x9B54, 0x9C3C, 0x9D1F, 0x9E02, 
	0x9EEC, 0x9FD1, 0xA0B2, 0xA198, 0xA27D, 
	0xA35E, 0xA448, 0xA529, 0xA610, 0xA6F0, 
	0xA7DF, 0xA8C0, 0xA9A7, 0xAA8B, 0xAB6D, 
	0xAC53, 0xAD39, 0xAE20, 0xAF04, 0xAFEA, 
	0xB0CB, 0xB1B3, 0xB298, 0xB37E, 0xB461, 
	0xB547, 0xB62C, 0xB712, 0xB7F5, 0xB8D9, 
	0xB9C1, 0xBAA5, 0xBB88, 0xBC6F, 0xBD51, 
	0xBE3B, 0xBF1E, 0xC003, 0xC0E8, 0xC1CE, 
	0xC2B1, 0xC397, 0xC47C, 0xC562, 0xC644, 
	0xC72C, 0xC811, 0xC8F3, 0xC9D6, 0xCABE, 
	0xCB9F, 0xCC88, 0xCD6B, 0xCE52, 0xCF36, 
	0xD01A, 0xD101, 0xD1E6, 0xD2CB, 0xD3AE, 
	0xD492, 0xD578, 0xD65A, 0xD745, 0xD826, 
	0xD90D, 0xD9F2, 0xDAD9, 0xDBBE, 0xDCA1, 
	0xDD85, 0xDE6B, 0xDF4E, 0xE036, 0xE118, 
	0xE1FB, 0xE2E4, 0xE3C5, 0xE4AB, 0xE591, 
	0xE678, 0xE75C, 0xE840, 0xE925, 0xEA0B, 
	0xEAEC, 0xEBD1, 0xECBA, 0xED9E, 0xEE85, 
	0xEF85, 0xF061, 0xF148, 0xF22F, 0xF30A, 
	0xF3F6, 0xF4D9, 0xF5B0, 0xF6A1, 0xF78B, 
	0x9F, 0x17D, 0x26A, 0x34C, 0x43B, 
	0x520, 0x5F5, 0x6E0, 0x7C5, 0x8B5, 
	0x997, 0xA76, 0xB5B, 0xC3D, 0xD23, 
	0xE08, 0xEF4, 0xFDA, 0x10C0, 0x11A2, 
	0x128F, 0x1371, 0x142A, 0x150C, 0x15F7, 
	0x16D4, 0x17BD, 0x189E, 0x1950, 0x1A6A, 
	0x1B4F, 0x1C31, 0x1D1B, 0x1DFB, 0x1EDB, 
	0x1FBF, 0x20AE, 0x2191, 0x2273, 0x235B, 
	0x2445, 0x2530, 0x261B, 0x26FA, 0x27E5, 
	0x28BB, 0x29A4, 0x2A9F, 0x2B6F, 0x2C5C, 
	0x2D38, 0x2E11, 0x2F2A, 0x2FE9, 0x30D7, 
	0x31B9, 0x3296, 0x3381, 0x3467, 0x3553, 
	0x3637, 0x3717, 0x3802, 0x38E4, 0x39CA, 
	0x3AAD, 0x3B91, 0x3C79, 0x3D5B, 0x3E47, 
	0x3F20, 0x400B, 0x40F0, 0x41DB, 0x42C5, 
	0x43A0, 0x447F, 0x4566, 0x4653, 0x472D, 
	0x4815, 0x48F0, 0x49E1, 0x4ACA, 0x4BA8, 
	0x4C8D, 0x4D76, 0x4E5E, 0x4F36, 0x5021, 
	0x5110, 0x51F2, 0x52D3, 0x53BB, 0x54AA, 
	0x558D, 0x5671, 0x5759, 0x583B, 0x5926, 
	0x5A0E, 0x5AEF, 0x5BD5, 0x5CBD, 0x5D9C, 
	0x5E7E, 0x5F67, 0x604B, 0x612D, 0x6210, 
	0x62FB, 0x63E2, 0x64CE, 0x65A5, 0x668E, 
	0x6775, 0x6858, 0x6935, 0x6A22, 0x6B05, 
	0x6BEF, 0x6CCB, 0x6DD5, 0x6E9E, 0x6F6F, 
	0x7050, 0x7127, 0x7208, 0x72E7
};

#define BATT_TEMP_AVG_SAMPLES	8
struct batt_temp_params {
	bool		update_now;
	int			batt_raw_temp;
	int			batt_temp;
	int			samples_num;
	int			samples_index;
	int			batt_temp_avg_samples[BATT_TEMP_AVG_SAMPLES];
	int			batt_temp_avg;
	int			batt_temp_prev;
//	struct timespec		last_temp_change_time;
};

struct sm_fg_chip {
	struct device		*dev;
	struct i2c_client	*client;
	struct mutex i2c_rw_lock; /* I2C Read/Write Lock */
	struct mutex data_lock; /* Data Lock */
	u8 chip;
	u8 regs[NUM_REGS];
	int	batt_id;
	
	/* Status Tracking */
	bool batt_present;
	int	batt_soc;
	int batt_ocv;
	int batt_fcc;	/* Full charge capacity */
	int	batt_volt;
	int	batt_temp;
	int	batt_curr;
	int batt_rmc;
	int is_charging;	/* Charging informaion from charger IC */
    int batt_soc_cycle; /* Battery SOC cycle */
	int batt_soh; /* Battery SOH */
	int iocv_error_count;

	//ENABLE_IOCV_ADJ
	bool enable_iocv_adj;
	int iocv_max_adj_level;
	int iocv_min_adj_level;
	int ioci_max_adj_level;
	int ioci_min_adj_level;
	int iocv_i_slope;
	int iocv_i_offset;	

	//ENABLE_INSPECTION_TABLE
	bool enable_inspection_table; 

	//ENABLE_TEMBASE_ZDSCON
	bool enable_tembase_zdscon;
	int zdscon_act_temp_gap;
	int t_gap_denom;
	int hminman_t_value_fast;
	int i_gap_denom;
	int hminman_i_value_fast;

	//ENABLE_TEMP_AVG
	bool enable_temp_avg;
	int change_temp_time_limit;	

	//ENABLE_NTC_COMPENSATION
	bool enable_ntc_compensation;
	int rtrace;
	int overheat_th_deg;
	int cold_th_deg;	

	//ENABLE_LTIM_ACT
	bool enable_ltim_act;
	int ltim_act_temp_gap;
	int ltim_i_limit;
	int ltim_factor;
	int ltim_denom;
	int ltim_min;

	//ENABLE_HCIC_ACT
	bool enable_hcic_act;
	int hcic_min;
	int hcic_factor;
	int hcic_denom;

	//ENABLE_HCRSM_MODE
	bool enable_hcrsm_mode;
	int hcrsm_volt;
	int hcrsm_curr;
	int hcrsm_min_soc;

	//CHECK_ABNORMAL_TABLE
	bool enable_check_abnormal_table;
		
	/* previous battery voltage current*/
    int p_batt_voltage;
    int p_batt_current;
	int p_report_soc;
	
	/* DT */
	bool en_temp_ex;
	bool en_temp_in;
	bool en_temp_3rd;
	bool iocv_man_mode;
	int aging_ctrl;
	int batt_rsns;	/* Sensing resistor value */
	int cycle_cfg;
	int battery_table_num;
	int misc[2];
    int batt_v_max;
	int min_cap;
	u32 common_param_version;
	int rpara;
	int curr_voffset;
	int curr_vslope;

	/* Battery Data */
	int battery_table[BATTERY_TABLE_MAX][FG_TABLE_LEN];
	signed short battery_temp_table[FG_TEMP_TABLE_CNT_MAX]; /* Degree */
	int alpha;
	int beta;
	int rs;
	int rs_value[FG_RS_PARAM_MAX];
	int vit_period;
	int mix_value;
	const char		*battery_type;
	int volt_cal;
	int curr_offset;
	int curr_slope;
	int cap;
    int topoff_soc;
	int topoff_margin;
    int top_off;
    int n_tem_poff;
    int n_tem_poff_offset;
	int batt_max_voltage_uv;
	int temp_std;
	int en_high_fg_temp_offset;
    int high_fg_temp_offset_denom;
    int high_fg_temp_offset_fact;
    int en_low_fg_temp_offset;
    int low_fg_temp_offset_denom;
    int low_fg_temp_offset_fact;
	int en_high_fg_temp_cal;
    int high_fg_temp_p_cal_denom;
    int high_fg_temp_p_cal_fact;
    int high_fg_temp_n_cal_denom;
    int high_fg_temp_n_cal_fact;
    int en_low_fg_temp_cal;
    int low_fg_temp_p_cal_denom;
    int low_fg_temp_p_cal_fact;
    int low_fg_temp_n_cal_denom;
    int low_fg_temp_n_cal_fact;
	int	en_high_temp_cal;
    int high_temp_p_cal_denom;
    int high_temp_p_cal_fact;
    int high_temp_n_cal_denom;
    int high_temp_n_cal_fact;
    int en_low_temp_cal;
    int low_temp_p_cal_denom;
    int low_temp_p_cal_fact;
    int low_temp_n_cal_denom;
    int low_temp_n_cal_fact;
	u32 battery_param_version;

	struct delayed_work monitor_work;

	int en_static_scale_soc;
	int static_scale_soc_base_min;
	int static_scale_soc_base_max;
	int off_current_min;
	int off_current_max;
	int rsoc_delta_scale_limit;
	int	last_rsoc;
	int static_scale_soc_base_value;
	int static_scale_soc_rise_count;

	/* Debug */
	int	skip_reads;
	int	skip_writes;
	int fake_soc;
	int fake_temp;
	struct dentry *debug_root;
#ifdef QC_PFM	
	struct power_supply *batt_psy;
#endif
#ifdef MTK_PFM
	struct power_supply *usb_psy;
#endif

	struct batt_temp_params	temp_param;

	int batt_vol_pre;
	int max_vol_pre;
	int min_vol_pre;
	int batt_cell_max_vol;
	int batt_cell_min_vol;
	int batt_authentic;
	u8 device_name[SM_INFO_LEN];
	int device_type;
};

struct sm_fg_chip *smifg_ic = NULL;
static int show_registers(struct seq_file *m, void *data);
static bool fg_init(struct i2c_client *client);
static int calculate_delta_time(ktime_t time_stamp, int *delta_time_s);
static int fg_read_current(struct sm_fg_chip *sm);
static bool fg_reg_init(struct i2c_client *client);
static bool is_battery_charging(struct sm_fg_chip *sm);

static int __fg_read_word(struct i2c_client *client, u8 reg, u16 *val)
{
	s32 ret;
	int retry = 0;

	for(retry = 0; retry < 3; retry++){
		ret = i2c_smbus_read_word_data(client, reg);
		if (ret < 0) {
			pr_err("i2c read word fail: can't read from reg 0x%02X, retry = %d\n", reg, retry);
			mdelay(10);
		} else {
			break;
		}
	}

	if(retry >= 3){
		*val = 0;
		return ret;
	}

	*val = (u16)ret;

	return 0;
}

static int __fg_write_word(struct i2c_client *client, u8 reg, u16 val)
{
	s32 ret;
	int retry = 0;

	for(retry = 0; retry < 3; retry++){
		ret = i2c_smbus_write_word_data(client, reg, val);
		if (ret < 0) {
			pr_err("i2c write word fail: can't write 0x%02X to reg 0x%02X, retry %d\n",
					val, reg, retry);
			mdelay(10);
		} else {
			break;
		}
	}

	if(retry >= 3){
		return ret;
	}

	return 0;
}

static int fg_read_word(struct sm_fg_chip *sm, u8 reg, u16 *val)
{
	int ret;

	if (sm->skip_reads) {
		*val = 0;
		return 0;
	}
	/* TODO:check little endian */
	mutex_lock(&sm->i2c_rw_lock);
	ret = __fg_read_word(sm->client, reg, val);
	mutex_unlock(&sm->i2c_rw_lock);

	return ret;
}

static int fg_write_word(struct sm_fg_chip *sm, u8 reg, u16 val)
{
	int ret;

	if (sm->skip_writes)
		return 0;

	/* TODO:check little endian */
	mutex_lock(&sm->i2c_rw_lock);
	ret = __fg_write_word(sm->client, reg, val);
	mutex_unlock(&sm->i2c_rw_lock);

	return ret;
}

#define	FG_STATUS_SLEEP				BIT(10)
#define	FG_STATUS_BATT_PRESENT		BIT(9)
#define	FG_STATUS_SOC_UPDATE		BIT(8)
#define	FG_STATUS_TOPOFF			BIT(7)
#define	FG_STATUS_LOW_SOC2			BIT(6)
#define	FG_STATUS_LOW_SOC1			BIT(5)
#define	FG_STATUS_HIGH_CURRENT		BIT(4)
#define	FG_STATUS_HIGH_TEMPERATURE	BIT(3)
#define	FG_STATUS_LOW_TEMPERATURE	BIT(2)
#define	FG_STATUS_HIGH_VOLTAGE		BIT(1)
#define	FG_STATUS_LOW_VOLTAGE		BIT(0)
#define	FG_OP_STATUS_CHG_DISCHG		BIT(15)

static int fg_read_status(struct sm_fg_chip *sm)
{
	sm->batt_present	= true;

	return 0;
}

#define FG_SOC_REG_INT_MASK		0x7F00
#define FG_SOC_REG_INT_SHIFT	8
#define FG_SOC_REG_FRAC			10
#define FG_SOC_REG_FRAC_MASK	0x00FF
#define FG_SOC_REG_FRAC_DIV		256
static int fg_read_soc(struct sm_fg_chip *sm)
{
	int ret;
	unsigned int soc = 0;
	u16 data = 0;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_SOC], &data);
	if (ret < 0) {
		pr_err("could not read SOC, ret = %d\n", ret);
		return ret;
	} else {
		/*integer bit;*/
		soc = ((data&FG_SOC_REG_INT_MASK)>>FG_SOC_REG_INT_SHIFT) * FG_SOC_REG_FRAC;
		/* integer + fractional bit*/
		soc = soc + (((data&FG_SOC_REG_FRAC_MASK)*FG_SOC_REG_FRAC)/FG_SOC_REG_FRAC_DIV);
		
		if (data & 0x8000) {
			pr_err("fg_read_soc data=%d\n",data);
			soc *= -1;
		}
	}

	if (sm->en_static_scale_soc) {
		if ((sm->batt_soc == 1000) && (sm->batt_curr > sm->off_current_min)) {
			sm->static_scale_soc_base_value = sm->static_scale_soc_base_min;
		} else if ((is_battery_charging(sm)) && (sm->batt_curr > sm->off_current_max)) {
			if (sm->static_scale_soc_base_max > sm->static_scale_soc_base_value) {
				if (sm->last_rsoc == 0) {
					sm->last_rsoc = soc;
				}
				if (sm->last_rsoc < soc) {
					if ((soc - sm->last_rsoc) >= sm->rsoc_delta_scale_limit) {
						sm->static_scale_soc_rise_count++;
						if (sm->static_scale_soc_rise_count > 1) {
							sm->static_scale_soc_base_value = sm->static_scale_soc_base_value + 1;
							sm->static_scale_soc_rise_count = 0;
							if (sm->static_scale_soc_base_value > sm->static_scale_soc_base_max) {
								sm->static_scale_soc_base_value = sm->static_scale_soc_base_max;
							}
						}
					}
				}
			}
		}
		sm->last_rsoc = soc;
		soc = ( soc * 1000 ) / sm->static_scale_soc_base_value;
		if (soc > 1000) {
			soc = 1000;
		}
		pr_info("fg_read_soc soc=%d(0x%x), curr=%d, base=%d\n", soc, data, sm->batt_curr, sm->static_scale_soc_base_value);
	}else {
		pr_info("fg_read_soc soc=%d(0x%x), curr=%d\n", soc, data, sm->batt_curr);
	}

	return soc;
}

#define SOC_ONE_HUNDRED		100
#define SOC_NINETY_NINE		99
#define SCALE_TEN			10
#define SAMPLE_SIZE_TWO		2
#define WAIT_SOC_GAP 		1
static int sm5602_fg_read_remain_capacity(void)
{
	int ret;
	struct sm_fg_chip *sm = smifg_ic;

	if (!smifg_ic) {
		return 0;
	}

	if (sm->fake_soc >= 0) {
		return sm->fake_soc;
	}
	ret = fg_read_soc(sm);
	mutex_lock(&sm->data_lock);
	if (ret >= 0)
		sm->batt_soc = ret;

	ret = sm->batt_soc/SCALE_TEN;
	sm->is_charging = is_battery_charging(sm); /* From Charger Driver */
	if ((sm->p_report_soc < SOC_ONE_HUNDRED) && (ret > SOC_NINETY_NINE)) {
		if ((abs((sm->batt_curr + sm->p_batt_current)/SAMPLE_SIZE_TWO) > (sm->top_off+sm->topoff_margin)) && (sm->is_charging)) {
			ret = ret - WAIT_SOC_GAP;
			chg_err("soc wait for full chg - %d %d %d %d %d\n", sm->p_batt_current, sm->batt_curr, sm->top_off, sm->topoff_margin, sm->is_charging);
		}
	}
	chg_debug("sm->batt_soc: %d, val->intval: %d\n", sm->batt_soc, ret);
	mutex_unlock(&sm->data_lock);

	sm->p_report_soc = ret;
	return ret;
}

int  sm5602_get_battery_mvolts_2cell_max(void)
{
	if (!smifg_ic) {
		return 0;
	}
	return smifg_ic->max_vol_pre;
}

int  sm5602_get_battery_mvolts_2cell_min(void)
{
	if(!smifg_ic) {
		return 0;
	}
	return smifg_ic->batt_cell_min_vol;
}

#define FG_OCV_REG_INT_MASK		0xF000
#define FG_OCV_REG_INT_SHIFT	11
#define FG_OCV_REG_FRAC			10
#define FG_OCV_REG_FRAC_MASK	0x0FFF
#define FG_OCV_REG_FRAC_DIV		2038
#define FG_OCV_REG_SCALE_MV		1000
static unsigned int fg_read_ocv(struct sm_fg_chip *sm)
{
	int ret;	
	u16 data = 0;
	unsigned int ocv;// = 3500; /*3500 means 3500mV*/

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_OCV], &data);
	if (ret<0) {
		pr_err("could not read OCV, ret = %d\n", ret);
		ocv = 4000;
	} else {
        ocv = (((data & FG_OCV_REG_FRAC_MASK) * FG_OCV_REG_SCALE_MV) / FG_OCV_REG_FRAC_DIV) + (((data & FG_OCV_REG_INT_MASK) >> FG_OCV_REG_INT_SHIFT) * FG_OCV_REG_SCALE_MV);
	}

	pr_info("ocv=%d(0x%x)\n",ocv, data);
	
	return ocv; //mV
}

short interp_meas_to_adc(int len, int X, int *pX, short *pY)
{
	int i;
	int slope, tmp;
	short new_y; 

	pr_info("interp_meas_to_adc\n\n");
	
	if (X < pX[0])
		return pY[0];
	else if (X > pX[len-1])
		return pY[len-1];

	for (i=0; i<len-1; i++) {
		if (X >= pX[i] && X <= pX[i+1])	
			break;
	}

	slope = (pY[i+1]-pY[i]);
	slope *= 100000;
	slope /= (pX[i+1] - pX[i]);
	tmp = slope * (X - pX[i])/100000 + (pY[i]);
	new_y = tmp;

	pr_info("%d, (%d, %d), (%4X, %4X), %4X\n\n", X, tex_meas_uV[i], tex_meas_uV[i+1], tex_meas_adc_code[i], tex_meas_adc_code[i+1], new_y);

	return new_y;
}

int interp_adc_to_meas(int len, short X, short *pX, int *pY)
{
	int i;
	int slope, tmp;
	int new_y; 
	

	pr_info("interp_adc_to_meas\n\n");
	
	if (X < pX[0])
		return pY[0];
	else if (X > pX[len-1])
		return pY[len-1];

	for (i=0; i<len-1; i++) {
		if (X >= pX[i] && X <= pX[i+1])	
			break;
	}

	slope = (pY[i+1]-pY[i]);
	slope *= 100000;
	slope /= (pX[i+1] - pX[i]);
	tmp = (slope * (X - pX[i]))/100000 + pY[i];
	new_y = tmp;

	pr_info("%4X, (%4X, %4X), (%d, %d), %d\n\n", X, tex_meas_adc_code[i], tex_meas_adc_code[i+1], tex_meas_uV[i], tex_meas_uV[i+1], new_y);

	return new_y;
}

static void calculate_average_temperature(struct sm_fg_chip *sm)
{
	int i;
	int avg_temp = sm->temp_param.batt_temp;
	int time_since_last_change_sec;
	int delta_time = 0;
	static ktime_t last_change_time;
	//struct timespec last_change_time = sm->temp_param.last_temp_change_time;

	calculate_delta_time(last_change_time, &time_since_last_change_sec);
	delta_time = time_since_last_change_sec / sm->change_temp_time_limit;

	//pr_info("last_change_time = %d, time_since_last_change_sec = %d, delta_time = %d\n",
	//		last_change_time, time_since_last_change_sec, delta_time);

#if 0
	/* continue if temperature has changed, if exceed time */
	if ((sm->temp_param.batt_temp == sm->temp_param.batt_temp_prev)	
		&& (delta_time < 1)) {
#else
	/* continue if exceed time */
	if (delta_time < 1) {
#endif
		goto skip_avg;
	} else {
		sm->temp_param.batt_temp_prev = sm->temp_param.batt_temp;
		last_change_time = ktime_get();
	}

	pr_info("last_change_time = %d, time_since_last_change_sec = %d, delta_time = %d\n",
			last_change_time, time_since_last_change_sec, delta_time);


	sm->temp_param.batt_temp_avg_samples[sm->temp_param.samples_index] = avg_temp;
	sm->temp_param.samples_index = (sm->temp_param.samples_index + 1) % BATT_TEMP_AVG_SAMPLES;
	sm->temp_param.samples_num++;

	if (sm->temp_param.samples_num >= BATT_TEMP_AVG_SAMPLES)
		sm->temp_param.samples_num = BATT_TEMP_AVG_SAMPLES;

	if (sm->temp_param.samples_num) {
		avg_temp = 0;
		/* maintain a BATT_TEMP_AVG_SAMPLES sample average of battery temp */
		for (i = 0; i < sm->temp_param.samples_num; i++) {
			// pr_debug("temp_samples[%d] = %d\n", i, sm->temp_param.batt_temp_avg_samples[i]);
			avg_temp += sm->temp_param.batt_temp_avg_samples[i];
		}
		// BATT_TEMP_AVG_SAMPLES = 8
		pr_info("temp_samples[0:7] = %d %d %d %d %d %d %d %d, samples_index = %d\n",
					sm->temp_param.batt_temp_avg_samples[0], sm->temp_param.batt_temp_avg_samples[1],
					sm->temp_param.batt_temp_avg_samples[2], sm->temp_param.batt_temp_avg_samples[3],
					sm->temp_param.batt_temp_avg_samples[4], sm->temp_param.batt_temp_avg_samples[5],
					sm->temp_param.batt_temp_avg_samples[6], sm->temp_param.batt_temp_avg_samples[7],
					sm->temp_param.samples_index
					);
		
		sm->temp_param.batt_temp_avg = DIV_ROUND_CLOSEST(avg_temp, sm->temp_param.samples_num);
	}

skip_avg:
	pr_info("temp_now = %d, averaged_avg_temp = %d\n",
			sm->temp_param.batt_temp, sm->temp_param.batt_temp_avg);
}

static int _calculate_battery_temp_ex(struct sm_fg_chip *sm, u16 uval)
{
	int i = 0, temp = 0;
	signed short val = 0;
	int len_meas_data;
	signed short code_adc = 0;
	int code_meas, temp_mv = 0.0;
	int rtrace, curr = 0;

	if ((uval >= 0x8001) && (uval <= 0x823B)) {
		pr_info("sp_range uval = 0x%x\n",uval);
		uval = 0x0000;
	}

	val = uval;

	if (sm->enable_ntc_compensation){
		pr_info("ENABLE_NTC_COMPENSATION : val = 0x%x\n",val);

		len_meas_data = sizeof(tex_meas_uV)/sizeof(int);

		curr = fg_read_current(sm); 		//fg_read_current(sm) must return mA

		rtrace = sm->rtrace;//uohm : 7300uohm = 7.3mohm

		code_meas = interp_adc_to_meas(len_meas_data, val, tex_meas_adc_code, tex_meas_uV);

		//Charging : Vthem = Vntc-I*Rtrace, Discharging : Vthem = Vntc+I*Rtrace
		temp_mv = (code_meas) - (curr * rtrace)/1000;  

		code_adc = interp_meas_to_adc(len_meas_data, temp_mv, tex_meas_uV, tex_meas_adc_code);

		val = code_adc;

		pr_info("ENABLE_NTC_COMPENSATION : val(code_adc) = 0x%x, temp_mv = %d, code_meas = %d, curr = %d, rtrace = %d\n", 
			val, temp_mv, code_meas, curr, rtrace);	
	}
	if (val >= sm->battery_temp_table[0]) {
		temp = EX_TEMP_MIN; //Min : -20
	} else if (val <= sm->battery_temp_table[FG_TEMP_TABLE_CNT_MAX-1]) {
		temp = EX_TEMP_MAX; //Max : 80
	} else { 		
		for (i = 0; i < FG_TEMP_TABLE_CNT_MAX; i++) {
			if  (val >= sm->battery_temp_table[i]) {
				temp = EX_TEMP_MIN + i; 								//[ex] ~-20 : -20(skip), -19.9~-19.0 : 19, -18.9~-18 : 18, .., -0.9~0 : 0 				
				if ((temp >= 1) && (val != sm->battery_temp_table[i])) //+ range 0~79 degree. In same value case, no needed (temp-1)
					temp = temp -1; 								  //[ex] 0.1~0.9 : 0, 1.1~1.9 : 1, .., 79.1~79.9 : 79  
				break;
			}
		}
	}

	pr_info("uval = 0x%x, val = 0x%x, temp = %d\n",uval, val, temp);

	return temp;		
}

#define FG_TEMP_REG_IN_ERR		(-128)
#define FG_TEMP_REG_INT_MASK	0x00FF
#define FG_TEMP_REG_SIGN_MASK	0x8000
#define FG_TEMP_REG_SIGN_M		(-1)
static int fg_read_temperature(struct sm_fg_chip *sm, enum sm_fg_temperature_type temperature_type)
{
	int ret, temp = 0;
	u16 data = 0;
	union power_supply_propval psp_temp = {0,};

	switch (temperature_type) {
	case TEMPERATURE_IN:	
		ret = fg_read_word(sm, sm->regs[SM_FG_REG_TEMPERATURE_IN], &data);
		if (ret < 0) {
			pr_err("could not read temperature in , ret = %d\n", ret);
			return FG_TEMP_REG_IN_ERR;
		} else {
			/*integer bit*/
                temp = ((data & FG_TEMP_REG_INT_MASK));
                if (data & FG_TEMP_REG_SIGN_MASK)
                    temp *= FG_TEMP_REG_SIGN_M;
		}
		pr_info("fg_read_temperature_in temp_in=%d\n", temp);
		break;
	case TEMPERATURE_EX:
		ret = fg_read_word(sm, sm->regs[SM_FG_REG_TEMPERATURE_EX], &data);
		if (ret < 0) {
			pr_err("could not read temperature ex , ret = %d\n", ret);
			return ret;
		} else {
			temp = _calculate_battery_temp_ex(sm, data);
		}

		pr_info("fg_read_temperature_ex temp_ex=%d\n", temp);

	if (sm->enable_temp_avg) {
		sm->temp_param.batt_temp = temp;
		calculate_average_temperature(sm);
	}
	break;
	case TEMPERATURE_3RD:	
		// If temperature value is obtained from external IC, pls add code in below case.
		// The unit of temperaute shoud be matched with integer type. ex.Return value(temp) 10 is 10 deg.   
		if (sm->batt_psy == NULL)
				sm->batt_psy = power_supply_get_by_name("battery");

		if (sm->batt_psy) {
			power_supply_get_property(sm->batt_psy,
				POWER_SUPPLY_PROP_TEMP, &psp_temp);
		}
		temp = psp_temp.intval / 10;
		pr_info("fg_read_temperature_3rd temp_3rd=%d\n", temp);
		break;

	default:
		return -EINVAL;
	}
	
	return temp;
}

static int sm5602_fg_read_temperature(void)
{
	int ret;
	struct sm_fg_chip *sm = smifg_ic;
	chg_debug("%s:%d\n", __func__, __LINE__);

	if (!smifg_ic) {
		chg_err("%s:%d\n", __func__, __LINE__);
		return 0;
	}

	if (sm->fake_temp != -EINVAL) {
		return sm->fake_temp;
	}
	if (sm->en_temp_in)
		ret = fg_read_temperature(sm, TEMPERATURE_IN);
	else if (sm->en_temp_ex)
		ret = fg_read_temperature(sm, TEMPERATURE_EX);
	else if (sm->en_temp_3rd)
		ret = fg_read_temperature(sm, TEMPERATURE_3RD);
	else
		ret = -ENODATA;
	mutex_lock(&sm->data_lock);
	if (ret > 0)
		sm->batt_temp = ret;

	if (sm->temp_param.batt_temp >= -40 && sm->temp_param.batt_temp <= 85)
		ret = sm->temp_param.batt_temp_avg * 10; /* 1.0degree = 10 */
	else if (sm->temp_param.batt_temp == -EINVAL)
		ret = sm->batt_temp * 10;
	else
		ret = SM5602_DEFAULT_TEMP;
	mutex_unlock(&sm->data_lock);
	chg_err("%s:%d,temp = %d\n", __func__, __LINE__, ret);

	return ret;
}

static bool sm5602_fg_read_battery_authenticate(void)
{
	chg_debug("%s:%d\n", __func__, __LINE__);


	sm5602_fg_read_temperature();
	return TRUE;
}

/*
 *	Return : mV
 */
#define FG_VOLT_REG_MASK			0x7FFF
#define FG_VOLT_REG_DIV				19622
#define FG_VOLT_REG_SCALE			1800
#define FG_VOLT_REG_SIGN_MASK		0x8000
#define FG_VOLT_REG_OFFSET			2700
#define FG_VOLT_REG_SIGN_M			(-1)
static int fg_read_volt(struct sm_fg_chip *sm)
{
	int ret = 0;
	int volt = 0;
	u16 data = 0;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_VOLTAGE], &data);
	if (ret < 0) {
		pr_err("could not read voltage, ret = %d\n", ret);
		return ret;
	}  else {
        volt = FG_VOLT_REG_SCALE * (data & FG_VOLT_REG_MASK) / FG_VOLT_REG_DIV;
        if (data & FG_VOLT_REG_SIGN_MASK) {
            volt *= FG_VOLT_REG_SIGN_M;
        }
        volt += FG_VOLT_REG_OFFSET;
	}

	pr_err("volt = %d(0x%x)\n",volt, data);

	return volt;
}

static int sm5602_fg_read_volt(void)
{
	int ret;
	struct sm_fg_chip *sm = smifg_ic;

	if (!smifg_ic) {
		return 0;
	}

	ret = fg_read_volt(sm);
	mutex_lock(&sm->data_lock);
	if (ret >= 0) {
		sm->batt_volt = ret;
		sm->batt_cell_max_vol = ret;
		sm->batt_cell_min_vol = ret;
		sm->batt_vol_pre = ret;
		sm->max_vol_pre = sm->batt_cell_max_vol;
		sm->min_vol_pre = sm->batt_cell_min_vol;
	} else {
		sm->batt_cell_max_vol = sm->max_vol_pre;
		sm->batt_cell_min_vol = sm->min_vol_pre;
		sm->batt_volt = sm->batt_vol_pre;
	}
	mutex_unlock(&sm->data_lock);
	return ret;
}

static int sm5602_fg_read_anything_int(void)
{
	return SM5602_FG_READ_ANYTHING_VAL;
}

static bool sm5602_fg_read_anything_bool(void)
{
	return true;
}

static void sm5602_set_battery_full(bool full)
{
	/* Do nothing */
}

#define FG_CYCLE_REG_MASK			0x03FF
#define FG_CYCLE_REG_INIT			0
static int fg_get_cycle(struct sm_fg_chip *sm)
{
	int ret;
	int cycle;
	u16 data = 0;

	ret = fg_read_word(sm, FG_REG_SOC_CYCLE, &data);
	if (ret<0) {
		pr_err("read cycle reg fail ret = %d\n", ret);
		cycle = FG_CYCLE_REG_INIT;
	} else {
		cycle = data&FG_CYCLE_REG_MASK;
	}

	pr_err("cycle = %d(0x%x)\n",cycle, data);

	return cycle;
}

static int sm5602_fg_read_cc(void)
{
	struct sm_fg_chip *sm = smifg_ic;
	int ret;

	if (!sm) {
		return 0;
	}
	//sm->batt_soc_cycle = fg_get_cycle(sm);
	ret = fg_get_cycle(sm);

	return ret;
}

static int fg_get_soh(struct sm_fg_chip *sm)
{
	int ret;
	int soh, pre_soh, h_flag, c_flag, delta_t;
	u16 data, temp = 0;

	c_flag = fg_get_cycle(sm);

	ret = fg_write_word(sm, FG_REG_SRADDR, FG_SRAM_SOH);
	msleep(15); //15mS delay 		
	ret |= fg_read_word(sm, FG_REG_SRDATA, &data);
	if (ret<0) {
		pr_err("read soh sram fail ret = %d\n", ret);
		return 100;
	} else {
		if (data & 0x8000) {
			pr_err("read soh sram fail data = 0x%x\n", data);
			return 100;
		} else {
			soh = data;
		}
	}

	ret = fg_read_word(sm, FG_USER_RESERVED, &temp);
	if (ret<0) {
		pr_err("read pre_soh reg fail ret = %d\n", ret);
		return 100;
	} else {
		pre_soh = (temp & 0xff00)>>8;
		temp = temp & 0x00ff;
	}

	if (sm->en_temp_3rd)
    	delta_t = fg_read_temperature(sm, TEMPERATURE_3RD);
	else if (sm->en_temp_ex)
		delta_t = fg_read_temperature(sm, TEMPERATURE_EX);
	else if (sm->en_temp_in)
		delta_t = fg_read_temperature(sm, TEMPERATURE_IN);
	else
		delta_t = fg_read_temperature(sm, TEMPERATURE_IN);

    pr_info("soh = %d, ic_soh = 0x%x, pre = 0x%x, cycle = %d, t = %d \n", sm->batt_soh, soh, pre_soh,
            c_flag, delta_t);

    soh = soh * 100 / 1024;

    h_flag = (pre_soh & 0x80) >> 7;
    pre_soh = pre_soh & 0x7F;
    c_flag = (c_flag >> 4) % 2;

    delta_t = delta_t - sm->temp_std;
    if (delta_t >= 0)
    {
        if (soh < pre_soh)
        {
            if (c_flag != h_flag)
            {
                pre_soh = pre_soh - 1;
                temp = temp | (((c_flag << 7) | pre_soh)<<8);
				ret = fg_write_word(sm, FG_USER_RESERVED, temp);
				if (ret<0) {
					pr_err("write pre_soh fail ret = %d\n", ret);
					return 100;
				} else {
	                pr_info("pre_soh update to %d, write 0x%x\n", pre_soh, temp);
				}
            }
        }
    }
    soh = pre_soh;

    sm->batt_soh = soh;

    return sm->batt_soh;

}

static int sm5602_fg_read_soh(void)
{
	struct sm_fg_chip *sm = smifg_ic;

	if (!sm) {
		return 0;
	}

	return sm->batt_soh;
}

static void fg_soh_init(struct sm_fg_chip *sm)
{
    int ret, temp;
	u16 data = 0;

	ret = fg_read_word(sm, FG_USER_RESERVED, &data);
	if (ret<0) {
		pr_err("read reserved reg fail ret = %d\n", ret);
		sm->batt_soh = 100;
		return;
	}

    temp = data & 0x00FF;
    data = (data & 0x7F00)>>8;
    if (data == 0)
    {
        sm->batt_soh = 100;
        data = 0x80;
        temp = temp | ((data | sm->batt_soh)<<8);
		ret = fg_write_word(sm, FG_USER_RESERVED, temp);
        pr_info("soh reset %d, write 0x%x\n", sm->batt_soh, temp);
    } else {
        sm->batt_soh = data;
        pr_info("soh restore : %d\n", sm->batt_soh);
    }

	return;
}

#define FG_CURR_REG_ERR				0x7FFF
#define FG_CURR_REG_RSNS_ERR		(-1)
#define FG_CURR_REG_RSNS_10MO		10
#define FG_CURR_REG_RSNS_5MO		5
#define FG_CURR_REG_RSNS_MIN		0
#define FG_CURR_REG_MASK			0x7FFF
#define FG_CURR_REG_SCALE			1250
#define FG_CURR_REG_DIV				511
#define FG_CURR_REG_SIGN_MASK		0x8000
#define FG_CURR_REG_SIGN_M			(-1)
static int fg_read_current(struct sm_fg_chip *sm)
{
	int ret = 0, rsns = 0;
	u16 data = 0;
	int curr = 0;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_CURRENT], &data);
	if (ret < 0) {
		pr_err("could not read current, ret = %d\n", ret);
		return FG_CURR_REG_ERR;
	} else {
		if (sm->batt_rsns == -EINVAL || sm->batt_rsns < 1) {
			pr_err("could not read or rsns error sm->batt_rsns = %d, rsns = 10mohm\n", sm->batt_rsns);
			rsns = FG_CURR_REG_RSNS_10MO;
		} else {
			rsns = sm->batt_rsns;
		}

		curr = ((data & FG_CURR_REG_MASK) * FG_CURR_REG_SCALE / FG_CURR_REG_DIV / rsns );

		if (data & 0x8000)
			curr *= -1;
	}
	
	pr_err("curr = %d(0x%x), rsns = %d\n",curr,data, rsns);

	return curr;
}

static int sm5602_fg_read_current(void)
{
	struct sm_fg_chip *sm = smifg_ic;
	int batt_curr = 0;

	if (!smifg_ic) {
		return 0;
	}

	mutex_lock(&sm->data_lock);
	batt_curr = fg_read_current(sm);
	batt_curr *= -1;
	mutex_unlock(&sm->data_lock);
	return batt_curr;
}

static int fg_read_fcc(struct sm_fg_chip *sm)
{
	int ret = 0;
	int fcc = 0;	
	u16 data = 0;
	int64_t temp = 0;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_BAT_CAP], &data);
	if (ret < 0) {
		pr_err("could not read FCC, ret=%d\n", ret);
		return ret;
	} else {
		/* */
		temp = div_s64((data & 0x7FFF) * 1000, 2048);
		fcc = temp;
	}

	pr_err("fcc = %d(0x%x)\n",fcc, data);

	return fcc;
}

static int sm5602_fg_read_fcc(void)
{
	int ret;
	struct sm_fg_chip *sm = smifg_ic;

	if (!smifg_ic) {
		return 0;
	}

	ret = fg_read_fcc(sm);
	mutex_lock(&sm->data_lock);
	if (ret > 0)
		sm->batt_fcc = ret;
	mutex_unlock(&sm->data_lock);
	return ret;
}

static int fg_read_rmc(struct sm_fg_chip *sm)
{
	int ret = 0;
	int rmc = 0;	
	u16 data = 0;

	ret = fg_read_word(sm, FG_REG_RMC, &data);
	if (ret < 0) {
		pr_err("could not read RMC, ret=%d\n", ret);
		return ret;
	} else {
		/* */
		rmc = ((data & 0x7FFF) * 1000) >> 11;
	}

	pr_err("rmc = %d(0x%x)\n",rmc, data);

	return rmc;
}

#define FG_SOFT_RESET	0x1A6 
static int fg_reset(struct sm_fg_chip *sm)
{
    int ret;

    ret = fg_write_word(sm, sm->regs[SM_FG_REG_RESET], FG_SOFT_RESET);
	if (ret < 0) {
		pr_err("could not reset, ret=%d\n", ret);
		return ret;
	}

	pr_info("=== %s ===\n", __func__);

	msleep(800);

    return 0;
}

//no use
int fg_set_shutdown(struct sm_fg_chip *sm, int en_shutdown)
{
    int ret = 0;
    uint16_t data = 0;
	int retry = 0;

	for(retry = 0; retry < 3; retry++) {
		if (en_shutdown) {
			// Control Register for en_shutdown bit
			ret = fg_read_word(sm, sm->regs[SM_FG_REG_SHUTDOWN], &data);
			if (ret < 0) {
				pr_err("Failed to read CNTL, ret = %d\n", ret);
				continue;//return ret;
			}

			data |= ENABLE_EN_SHUTDOWN;
			ret = fg_write_word(sm, sm->regs[SM_FG_REG_SHUTDOWN], data);
			if (ret < 0) {
				pr_err("Failed to write SHUTDOWN, ret = %d\n", ret);
				continue;//return ret;
			} else {
				pr_info("SHUTDOWN = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_SHUTDOWN], data);
			}
			
			// Control Register for shutdown bit
			ret = fg_read_word(sm, sm->regs[SM_FG_REG_CNTL], &data);
			if (ret < 0) {
				pr_err("Failed to read CNTL, ret = %d\n", ret);
				continue;//return ret;
			}
			
			data |= ENABLE_SHUTDOWN;
			ret = fg_write_word(sm, sm->regs[SM_FG_REG_CNTL], data);
			if (ret < 0) {
				pr_err("Failed to write CNTL, ret = %d\n", ret);
				continue;//return ret;
			} else {
				pr_info("CNTL = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_CNTL], data);
			}
			break;
		} else {
			// Control Register for en_shutdown bit
			ret = fg_read_word(sm, sm->regs[SM_FG_REG_SHUTDOWN], &data);
			if (ret < 0) {
				pr_err("Failed to read CNTL, ret = %d\n", ret);
				continue;//return ret;
			}
			
			data &= ~ENABLE_EN_SHUTDOWN;
			ret = fg_write_word(sm, sm->regs[SM_FG_REG_SHUTDOWN], data);
			if (ret < 0) {
				pr_err("Failed to write SHUTDOWN, ret = %d\n", ret);
				continue;//return ret;
			} else {
				pr_info("SHUTDOWN = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_SHUTDOWN], data);
			}
			
			// Read Register for shutdown bit
			ret = fg_read_word(sm, sm->regs[SM_FG_REG_CNTL], &data);
			if (ret < 0) {
				pr_err("Failed to read CNTL, ret = %d\n", ret);
				continue;//return ret;
			} else {
				pr_info("CNTL = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_CNTL], data);
			}
			// Control Register for shutdown bit
			ret = fg_read_word(sm, sm->regs[SM_FG_REG_CNTL], &data);
			if (ret < 0) {
				pr_err("Failed to read CNTL, ret = %d\n", ret);
				continue;//return ret;
			}
			
			//data &= ~ENABLE_SHUTDOWN;
			//ret = fg_write_word(sm, sm->regs[SM_FG_REG_CNTL], data);
			//if (ret < 0) {
			//	pr_err("Failed to write CNTL, ret = %d\n", ret);
			//	continue;//return ret;
			//} else {
			//	pr_info("CNTL = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_CNTL], data);
			//}			
			break;
		}
	}
	
	msleep(1);

	// Printed log
	if (en_shutdown)
		pr_info("Enabled shutdown mode (%d)", retry);
	else
		pr_info("Disabled shutdown mode (%d)", retry);

    return ret;
}

#ifdef QC_PFM
static int get_battery_status(struct sm_fg_chip *sm)
{
	union power_supply_propval ret = {0,};
	int rc;

	if (sm->batt_psy == NULL)
		sm->batt_psy = power_supply_get_by_name("battery");
	if (sm->batt_psy) {
		/* if battery has been registered, use the status property */
		rc = power_supply_get_property(sm->batt_psy,
					POWER_SUPPLY_PROP_STATUS, &ret);
		if (rc) {
			pr_err("Battery does not export status: %d\n", rc);
			return POWER_SUPPLY_STATUS_UNKNOWN;
		}
		return ret.intval;
	}

	/* Default to false if the battery power supply is not registered. */
	pr_err("battery power supply is not registered\n");
	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static bool is_battery_charging(struct sm_fg_chip *sm)
{
	return get_battery_status(sm) == POWER_SUPPLY_STATUS_CHARGING; //From Charger
}
#endif

#ifdef MTK_PFM
static int get_ac_status(struct sm_fg_chip *sm)
{
	union power_supply_propval ret = {0,};
	int rc;

	if (sm->usb_psy == NULL)
		sm->usb_psy = power_supply_get_by_name("ac");
	if (sm->usb_psy) {
		/* if battery has been registered, use the status property */
		rc = power_supply_get_property(sm->usb_psy,
					POWER_SUPPLY_PROP_STATUS, &ret);
		if (rc) {
			pr_err("Battery does not export status: %d\n", rc);
			return POWER_SUPPLY_STATUS_UNKNOWN;
		}
		return ret.intval;
	}

	/* Default to false if the battery power supply is not registered. */
	pr_err("battery power supply is not registered\n");
	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static int get_usb_status(struct sm_fg_chip *sm)
{
	union power_supply_propval ret = {0,};
	int rc;

	if (sm->usb_psy == NULL)
		sm->usb_psy = power_supply_get_by_name("usb");
	if (sm->usb_psy) {
		/* if battery has been registered, use the status property */
		rc = power_supply_get_property(sm->usb_psy,
					POWER_SUPPLY_PROP_STATUS, &ret);
		if (rc) {
			pr_err("Battery does not export status: %d\n", rc);
			return POWER_SUPPLY_STATUS_UNKNOWN;
		}
		return ret.intval;
	}

	/* Default to false if the battery power supply is not registered. */
	pr_err("battery power supply is not registered\n");
	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static bool is_battery_charging(struct sm_fg_chip *sm)
{
	return (get_ac_status(sm) == POWER_SUPPLY_STATUS_CHARGING
			|| get_usb_status(sm) == POWER_SUPPLY_STATUS_CHARGING); //From Charger
}
#endif /* MTK_PFM */


static void fg_tembase_zdscon(struct sm_fg_chip *sm)
{
	int hminman_value = 0;
	u16 data = 0;
	u16	ltim_value = 0;
	int ret = 0;
	int fg_temp_gap = sm->batt_temp - sm->temp_std;

#ifdef ENABLE_TEM_RATE_CONTROL
	if (fg_temp_gap > -5)
	{
		ret = fg_read_word(sm, FG_REG_AGING_CTRL, &data);
		if (data != (sm->aging_ctrl | 0x0C))
		{
			pr_info("%s: last rate control data 0x%x change to fast rate\n", __func__, data);
			ret = fg_write_word(sm, FG_REG_AGING_CTRL, sm->aging_ctrl | 0x0C);
		}
	}
	else
	{
		ret = fg_read_word(sm, FG_REG_AGING_CTRL, &data);
		if (data != sm->aging_ctrl)
		{
			pr_info("%s: last rate control data 0x%x change to normal rate\n", __func__, data);
			ret = fg_write_word(sm, FG_REG_AGING_CTRL, sm->aging_ctrl);
		}
	}

	if (ret < 0) 
	{
		pr_err("could not control 0x%x, fg_temp_gap = %d, ret = %d, data = 0x%x \n", FG_REG_AGING_CTRL, fg_temp_gap, ret, data);
	}
#endif

	if ((fg_temp_gap < 0) && !(sm->is_charging))
	{
		fg_temp_gap = abs(fg_temp_gap);
		if (fg_temp_gap > sm->zdscon_act_temp_gap)
		{
			if (sm->enable_ltim_act) {
				if (fg_temp_gap > sm->ltim_act_temp_gap) {
					if (abs(sm->batt_curr) > sm->ltim_i_limit) {
						ltim_value = sm->rs_value[1] - (((sm->ltim_factor * abs(sm->batt_curr)) / 1000) + sm->ltim_denom);
						if (ltim_value < sm->ltim_min) {
							ltim_value = sm->ltim_min;
						}
						ret = fg_read_word(sm, FG_REG_RS_1, &data);
						if (ret < 0) {
							pr_err("could not read 0x%d, ret = %d\n", FG_REG_RS_1, ret);
						} else {
							if (data != ltim_value)
							{
								fg_write_word(sm, FG_REG_RS_1, ltim_value);
								pr_info("%s: ltim_value value set 0x%x tem(%d) i(%d)\n", __func__, ltim_value, sm->batt_temp, sm->batt_curr);
							}
						}
	 				} else {
						ret = fg_read_word(sm, FG_REG_RS_1, &data);
						if (ret < 0) {
							pr_err("could not read 0x%x, ret = %d\n", FG_REG_RS_1, ret);
						}
						else
						{
							if (data != sm->rs_value[1])
							{
								fg_write_word(sm, FG_REG_RS_1, sm->rs_value[1]);
								pr_info("%s: ltimfactor value restore 0x%x -> 0x%x tem(%d)\n", __func__, data, sm->rs_value[1], sm->batt_temp);
							}
						}
					}
				} else {
					ret = fg_read_word(sm, FG_REG_RS_1, &data);
					if (ret < 0) {
						pr_err("could not read 0x%x, ret = %d\n", FG_REG_RS_1, ret);
					} else {
						if (data != sm->rs_value[1])
						{
							fg_write_word(sm, FG_REG_RS_1, sm->rs_value[1]);
							pr_info("%s: ltimfactor value restore 0x%x -> 0x%x tem(%d)\n", __func__, data, sm->rs_value[1], sm->batt_temp);
						}
					}
				}
			}
			hminman_value = sm->rs_value[3] + (((fg_temp_gap - sm->zdscon_act_temp_gap) * sm->hminman_t_value_fast) / sm->t_gap_denom);
			hminman_value = hminman_value - ((abs(sm->batt_curr) * sm->hminman_i_value_fast) / sm->i_gap_denom);

			if (hminman_value < sm->rs_value[3]) {
				hminman_value = sm->rs_value[3];
			}
			ret = fg_read_word(sm, FG_REG_RS_3, &data);
			if (ret < 0) {
				pr_err("could not read , ret = %d\n", ret);
			}
			else
			{
				if (data != hminman_value)
				{
					fg_write_word(sm, FG_REG_RS_3, hminman_value);
					fg_write_word(sm, FG_REG_RS_0, hminman_value+2);
					pr_info("%s: hminman value set 0x%x tem(%d) i(%d)\n", __func__, hminman_value, sm->batt_temp, sm->batt_curr);
				}
			}
		} else {
			ret = fg_read_word(sm, FG_REG_RS_3, &data);
			if (ret < 0) {
				pr_err("could not read , ret = %d\n", ret);
			}
			else
			{
				if (data != sm->rs_value[3])
				{
					fg_write_word(sm, FG_REG_RS_3, sm->rs_value[3]);
					fg_write_word(sm, FG_REG_RS_0, sm->rs_value[0]);
					pr_info("%s: hminman value restore 0x%x -> 0x%x tem(%d)\n", __func__, data, sm->rs_value[3], sm->batt_temp);
				}
			}
			if (sm->enable_ltim_act) {
				ret = fg_read_word(sm, FG_REG_RS_1, &data);
				if (ret < 0) {
					pr_err("could not read 0x%x, ret = %d\n", FG_REG_RS_1, ret);
				}
				else
				{
					if (data != sm->rs_value[1])
					{
						fg_write_word(sm, FG_REG_RS_1, sm->rs_value[1]);
						pr_info("%s: ltimfactor value restore 0x%x -> 0x%x tem(%d)\n", __func__, data, sm->rs_value[1], sm->batt_temp);
					}
				}
			}
		}
	} else {
		ret = fg_read_word(sm, FG_REG_RS_3, &data);
		if (ret < 0) {
			pr_err("could not read 0x%x, ret = %d\n", FG_REG_RS_3, ret);
		} else {
			if (data != sm->rs_value[3])
			{
				fg_write_word(sm, FG_REG_RS_3, sm->rs_value[3]);
				fg_write_word(sm, FG_REG_RS_0, sm->rs_value[0]);
				pr_info("%s: hminman value restore 0x%x -> 0x%x tem(%d)\n", __func__, data, sm->rs_value[3], sm->batt_temp);
			}
		}

		if (sm->enable_ltim_act) {
			ret = fg_read_word(sm, FG_REG_RS_1, &data);
			if (ret < 0) {
				pr_err("could not read 0x%x, ret = %d\n", FG_REG_RS_1, ret);
			}
			else
			{
				if (data != sm->rs_value[1])
				{
					fg_write_word(sm, FG_REG_RS_1, sm->rs_value[1]);
					pr_info("%s: ltimfactor value restore 0x%x -> 0x%x tem(%d)\n", __func__, data, sm->rs_value[1], sm->batt_temp);
				}
			}
		}
	}

    return;
}


#define FG_VOC_SOC_THD				900
#define FG_VOC_VCGAP_THD			30
#define FG_VOC_IOCV_E_CNT_THD		5
#define FG_VOC_IOCV_E_CNT_MAX		6
#define FG_VOC_IOCV_E_CNT_INIT		0
#define FG_VOC_PVGAP_THD			15
#define FG_VOC_RS_SHIFT				1
static void fg_vbatocv_check(struct sm_fg_chip *sm)
{
	int ret = 0;
	u16 data = 0;	

	pr_info("sm->batt_curr (%d), sm->is_charging (%d), sm->top_off (%d), sm->batt_soc (%d), sm->topoff_margin (%d)\n", 
			sm->batt_curr, sm->is_charging, sm->top_off, sm->batt_soc, sm->topoff_margin);

	ret = fg_read_word(sm, FG_REG_RS_0, &data);
	if (ret < 0) {
		pr_err("could not read , ret = %d\n", ret);
	}

	if(((sm->is_charging) && (abs(sm->batt_curr)<(sm->top_off+sm->topoff_margin)) &&
		(abs(sm->batt_curr)>(sm->top_off-sm->topoff_margin)) && (sm->batt_soc>=FG_VOC_SOC_THD)))
	{
		if(abs(sm->batt_ocv-sm->batt_volt)>FG_VOC_VCGAP_THD)
		{
			sm->iocv_error_count ++;
		}

		pr_info("sm5602g FG iocv_error_count (%d)\n", sm->iocv_error_count);

		if(sm->iocv_error_count > FG_VOC_IOCV_E_CNT_THD)
			sm->iocv_error_count = FG_VOC_IOCV_E_CNT_MAX;
	}
	else if ((sm->enable_hcrsm_mode) && (sm->is_charging) && (sm->batt_volt>sm->hcrsm_volt) && (sm->p_batt_voltage>sm->hcrsm_volt)
	&& (abs(sm->batt_curr)<sm->hcrsm_curr) && (abs(sm->p_batt_current)<sm->hcrsm_curr))
	{
		sm->iocv_error_count = 10;
	}
	else
	{
		sm->iocv_error_count = FG_VOC_IOCV_E_CNT_INIT;
	}

	if(sm->iocv_error_count > FG_VOC_IOCV_E_CNT_THD)
	{
		pr_info("p_v - v = (%d)\n", sm->p_batt_voltage - sm->batt_volt);
		if(abs(sm->p_batt_voltage - sm->batt_volt)>15)
		{
			sm->iocv_error_count = 0;
		}
		else
		{
			fg_write_word(sm, FG_REG_RS_2, data);
			pr_info("mode change to RS m mode 0x%x\n", data);
		}
	}
	else
	{
		if((sm->p_batt_voltage < sm->n_tem_poff) &&
			(sm->batt_volt < sm->n_tem_poff) && (!sm->is_charging))
		{
			if((sm->p_batt_voltage <
				(sm->n_tem_poff - sm->n_tem_poff_offset)) &&
				(sm->batt_volt <
				(sm->n_tem_poff - sm->n_tem_poff_offset)))
			{
				fg_write_word(sm, FG_REG_RS_2, data>>FG_VOC_RS_SHIFT);
				pr_info("mode change to normal tem RS m mode 0x%x\n", data>>1);
			}
			else
			{
				fg_write_word(sm, FG_REG_RS_2, data);
				pr_info("mode change to normal tem RS m mode 0x%x\n", data);
			}
		}
		else
		{
			pr_info("mode change to RS a mode\n");

			fg_write_word(sm, FG_REG_RS_2, sm->rs_value[2]);
		}
	}
	sm->p_batt_voltage = sm->batt_volt;
	sm->p_batt_current = sm->batt_curr;
	// iocv error case cover end
}

#define TEMP_G_TH				0
#define TEMP_CO_TH 	   			0
#define TEMP_CO_PO_S_MASK		0x0080
#define TEMP_CO_PO_MASK			0x007F
#define TEMP_CO_NO_SHIFT		8
#define TEMP_CO_NO_MASK			0xFF00
#define TEMP_CO_NS_SHIFT		8
#define TEMP_CO_NS_MASK			0xFF00
#define TEMP_CO_PS_MASK			0x00FF
#define FCC_LOG_CNT_MAX			15
#define FCC_AG_MASK				0x0C
#define FCC_AG_OFF_MASK			0xFFFE
#define FCC_TEMP_GAP_TH_M5		(-5)
#define FCC_TEMP_GAP_TH_M31		(-31)
#define ZDSCON_ACT_TEMP_GAP 	15
#define T_GAP_DENOM 			5
#define HMINMAN_T_VALUE_FACT 	125
#define I_GAP_DENOM 			1000
#define HMINMAN_I_VALUE_FACT 	150
static int fg_cal_carc (struct sm_fg_chip *sm)
{
	int curr_cal = 0, p_curr_cal=0, n_curr_cal=0, p_delta_cal=0, n_delta_cal=0, p_fg_delta_cal=0, n_fg_delta_cal=0, temp_curr_offset=0;
	int temp_gap, fg_temp_gap = 0;
	int ret = 0;
	u16 data[17] = {0,};

	sm->is_charging = is_battery_charging(sm); //From Charger Driver

	if (sm->enable_tembase_zdscon)
 		fg_tembase_zdscon(sm);

	fg_vbatocv_check(sm);

	//fg_temp_gap = (sm->batt_temp/10) - sm->temp_std;
	fg_temp_gap = sm->batt_temp - sm->temp_std;
	
		temp_curr_offset = sm->curr_offset;
		if(sm->en_high_fg_temp_offset && (fg_temp_gap > TEMP_G_TH))
		{
			if(temp_curr_offset & TEMP_CO_PO_S_MASK)
			{
				temp_curr_offset = -(temp_curr_offset & TEMP_CO_PO_MASK);
			}
			temp_curr_offset = temp_curr_offset + (fg_temp_gap / sm->high_fg_temp_offset_denom)*sm->high_fg_temp_offset_fact;
			if(temp_curr_offset < TEMP_CO_TH)
			{
				temp_curr_offset = -temp_curr_offset;
				temp_curr_offset = temp_curr_offset|TEMP_CO_PO_S_MASK;
			}
		}
		else if (sm->en_low_fg_temp_offset && (fg_temp_gap < TEMP_G_TH))
		{
			if(temp_curr_offset & TEMP_CO_PO_S_MASK)
			{
				temp_curr_offset = -(temp_curr_offset & TEMP_CO_PO_MASK);
			}
			temp_curr_offset = temp_curr_offset + ((-fg_temp_gap) / sm->low_fg_temp_offset_denom)*sm->low_fg_temp_offset_fact;
			if(temp_curr_offset < TEMP_CO_TH)
			{
				temp_curr_offset = -temp_curr_offset;
				temp_curr_offset = temp_curr_offset|TEMP_CO_PO_S_MASK;
			}
		}

		temp_curr_offset = temp_curr_offset | (temp_curr_offset<<TEMP_CO_NO_SHIFT);
	
		ret = fg_write_word(sm, FG_REG_CURR_IN_OFFSET, temp_curr_offset);
		if (ret < 0) {
			pr_err("Failed to write CURR_IN_OFFSET, ret = %d\n", ret);
			return ret;
		} else {
			pr_err("CURR_IN_OFFSET [0x%x] = 0x%x\n", FG_REG_CURR_IN_OFFSET, temp_curr_offset);
		}
	
		n_curr_cal = (sm->curr_slope & TEMP_CO_NS_MASK)>>TEMP_CO_NS_SHIFT;
		p_curr_cal = (sm->curr_slope & TEMP_CO_PS_MASK);
		
		if (sm->en_high_fg_temp_cal && (fg_temp_gap > TEMP_G_TH))
		{
			p_fg_delta_cal = (fg_temp_gap / sm->high_fg_temp_p_cal_denom)*sm->high_fg_temp_p_cal_fact;
			n_fg_delta_cal = (fg_temp_gap / sm->high_fg_temp_n_cal_denom)*sm->high_fg_temp_n_cal_fact;
		}
		else if (sm->en_low_fg_temp_cal && (fg_temp_gap < TEMP_G_TH))
		{
			fg_temp_gap = -fg_temp_gap;
			p_fg_delta_cal = (fg_temp_gap / sm->low_fg_temp_p_cal_denom)*sm->low_fg_temp_p_cal_fact;
			n_fg_delta_cal = (fg_temp_gap / sm->low_fg_temp_n_cal_denom)*sm->low_fg_temp_n_cal_fact;
		}
		p_curr_cal = p_curr_cal + (p_fg_delta_cal);
		n_curr_cal = n_curr_cal + (n_fg_delta_cal);

	pr_info("<%d %d %d %d %d %d %d %d %d %d>, temp_fg = %d ,p_curr_cal = 0x%x, n_curr_cal = 0x%x, "
		"batt_temp = %d\n",	
		sm->en_high_fg_temp_cal,
		sm->high_fg_temp_p_cal_denom, sm->high_fg_temp_p_cal_fact, 
		sm->high_fg_temp_n_cal_denom, sm->high_fg_temp_n_cal_fact,
		sm->en_low_fg_temp_cal, 
		sm->low_fg_temp_p_cal_denom, sm->low_fg_temp_p_cal_fact, 
		sm->low_fg_temp_n_cal_denom, sm->low_fg_temp_n_cal_fact,
		sm->batt_temp, p_curr_cal, n_curr_cal, sm->batt_temp);

		temp_gap = sm->batt_temp - sm->temp_std;	
		if (sm->en_high_temp_cal && (temp_gap > TEMP_G_TH))
		{
			p_delta_cal = (temp_gap / sm->high_temp_p_cal_denom)*sm->high_temp_p_cal_fact;
			n_delta_cal = (temp_gap / sm->high_temp_n_cal_denom)*sm->high_temp_n_cal_fact;
		}
		else if (sm->en_low_temp_cal && (temp_gap < TEMP_G_TH))
		{
			temp_gap = -temp_gap;
			p_delta_cal = (temp_gap / sm->low_temp_p_cal_denom)*sm->low_temp_p_cal_fact;
			n_delta_cal = (temp_gap / sm->low_temp_n_cal_denom)*sm->low_temp_n_cal_fact;
		}
		p_curr_cal = p_curr_cal + (p_delta_cal);
		n_curr_cal = n_curr_cal + (n_delta_cal);

	if (sm->enable_hcic_act) {
		if (abs(sm->batt_curr) > sm->hcic_min) {
			if(sm->hcic_denom == 0) {
				p_delta_cal = sm->hcic_factor;
			} else {	
				p_delta_cal = ((abs(sm->batt_curr) - sm->hcic_min) / sm->hcic_denom) * sm->hcic_factor;
			}
			p_curr_cal = p_curr_cal + (p_delta_cal);
			n_curr_cal = n_curr_cal + (p_delta_cal);
			pr_info("[%s] %s: HCIC_ACT i_%d, p_curr_cal = 0x%x, n_curr_cal = 0x%x\n", device2str[sm->chip], __func__, sm->batt_curr, p_curr_cal, n_curr_cal);
		}
	}

	if (n_curr_cal < 0x01) {
		n_curr_cal = 0x01;
	} else if (n_curr_cal > 0xFF) {
		n_curr_cal = 0xFF;
	}
	if (p_curr_cal < 0x01) {
		p_curr_cal = 0x01;
	} else if (p_curr_cal > 0xFF) {
		p_curr_cal = 0xFF;
	}

    curr_cal = (n_curr_cal << TEMP_CO_NS_SHIFT) | p_curr_cal;

	ret = fg_write_word(sm, FG_REG_CURR_IN_SLOPE, curr_cal);
	if (ret < 0) {
		pr_err("Failed to write CURR_IN_SLOPE, ret = %d\n", ret);
		return ret;
	} else {
		pr_err("write CURR_IN_SLOPE [0x%x] = 0x%x\n", FG_REG_CURR_IN_SLOPE, curr_cal);
	}

	pr_info("<%d %d %d %d %d %d %d %d %d %d>, "
		"p_curr_cal = 0x%x, n_curr_cal = 0x%x, curr_cal = 0x%x\n",	
		sm->en_high_temp_cal,
		sm->high_temp_p_cal_denom, sm->high_temp_p_cal_fact, 
		sm->high_temp_n_cal_denom, sm->high_temp_n_cal_fact,
		sm->en_low_temp_cal, 
		sm->low_temp_p_cal_denom, sm->low_temp_p_cal_fact, 
		sm->low_temp_n_cal_denom, sm->low_temp_n_cal_fact,
		p_curr_cal, n_curr_cal, curr_cal);

	ret = fg_write_word(sm, sm->regs[SM_FG_REG_VOL_COMP], sm->rpara);
	if (ret < 0) {
		pr_err("Failed to write SM_FG_REG_VOL_COMP, ret = %d\n", ret);
		return ret;
	} else {
		pr_err("write SM_FG_REG_VOL_COMP [0x%x] = 0x%x\n", sm->regs[SM_FG_REG_VOL_COMP], sm->rpara);
	}

	ret = fg_write_word(sm, sm->regs[SM_FG_REG_CURR_OFFSET], sm->curr_voffset);
	if (ret < 0) {
		pr_err("Failed to write SM_FG_REG_CURR_OFFSET, ret = %d\n", ret);
		return ret;
	} else {
		pr_err("write SM_FG_REG_CURR_OFFSET [0x%x] = 0x%x\n", FG_REG_CURR_IN_SLOPE, sm->curr_voffset);
	}

	ret = fg_write_word(sm, sm->regs[SM_FG_REG_CURR_SLOPE], sm->curr_vslope);
	if (ret < 0) {
		pr_err("Failed to write SM_FG_REG_CURR_SLOPE, ret = %d\n", ret);
		return ret;
	} else {
		pr_err("write SM_FG_REG_CURR_SLOPE [0x%x] = 0x%x\n", SM_FG_REG_CURR_SLOPE, sm->curr_vslope);
	}

	ret |= fg_read_word(sm, 0x00, &data[0]);
	ret |= fg_read_word(sm, 0x06, &data[1]);
	ret |= fg_read_word(sm, 0x1E, &data[2]);
	ret |= fg_read_word(sm, 0x1F, &data[3]);
	ret |= fg_read_word(sm, 0x26, &data[4]);
	ret |= fg_read_word(sm, 0x27, &data[5]);
	ret |= fg_read_word(sm, 0x28, &data[6]);
	ret |= fg_read_word(sm, 0x73, &data[7]);
	ret |= fg_read_word(sm, 0x74, &data[8]);
	ret |= fg_read_word(sm, 0x80, &data[9]);
	ret |= fg_read_word(sm, 0x83, &data[10]);
	ret |= fg_read_word(sm, 0x84, &data[11]);
	ret |= fg_read_word(sm, 0x85, &data[12]);
	ret |= fg_read_word(sm, 0x86, &data[13]);
	ret |= fg_read_word(sm, 0x87, &data[14]);
	ret |= fg_read_word(sm, 0x93, &data[15]);
	ret |= fg_read_word(sm, 0x9C, &data[16]);
	if (ret < 0) {
		pr_err("could not read , ret = %d\n", ret);
		return ret;
	} else
		pr_info("0x00=0x%x, 0x06=0x%x, 0x1E=0x%x, 0x1F=0x%x, 0x26=0x%x, 0x27=0x%x, 0x28=0x%x, 0x73=0x%x, 0x74=0x%x, 0x80=0x%x, 0x83=0x%x, 0x84=0x%x, 0x85=0x%x, 0x86=0x%x, 0x87=0x%x, 0x93=0x%x, 0x9C=0x%x\n",
			data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15], data[16]);

	return 1;
}
struct gauge_track_info_reg {
	int addr;
	int len;
	int start_index;
	int end_index;
};

struct gauge_track_info_reg sm5602g_info[] = {
	{ 0x00, 2 },
	{ 0x06 , 2 },
	{ 0x08 , 2 },
	{ 0x0A , 2 },
	{ 0x1E , 2 },
	{ 0x26 , 2 },
	{ 0x28 , 2 },
	{ 0x73 , 2 },
	{ 0x80 , 2 },
	{ 0x83 , 2 },
	{ 0x85 , 2 },
	{ 0x87 , 2 },
	{ 0x93 , 2 },
	{ 0x9C , 2 },
};

static int sm5602g_get_info(u8 *info, int len)
{
	int i;
	int ret;
	unsigned char reg_val[2] = {0 , 0};
	int index = 0;

	if (!smifg_ic || !info || !len)
		return -1;

	for (i = 0; i < ARRAY_SIZE(sm5602g_info); i++) {
		ret = fg_read_word(smifg_ic, sm5602g_info[i].addr, (u16 *)reg_val);
		if (ret < 0)
			continue;
		index += snprintf(info + index, len - index,
			  "0x%02x=%02x,%02x|", sm5602g_info[i].addr, reg_val[0], reg_val[1]);
	}
	return 0;
}


static struct oplus_gauge_operations sm5602_gauge_ops = {
	.get_battery_mvolts = sm5602_fg_read_volt,
	//.get_battery_fc = bq27541_get_battery_fc,//log
	//.get_battery_qm = bq27541_get_battery_qm,//log
	//.get_battery_pd = bq27541_get_battery_pd,//log
	//.get_battery_rcu = bq27541_get_battery_rcu,//log
	//.get_battery_rcf = bq27541_get_battery_rcf,//log
	//.get_battery_fcu = bq27541_get_battery_fcu,//log
	//.get_battery_fcf = bq27541_get_battery_fcf,//log
	//.get_battery_sou = bq27541_get_battery_sou,//log
	//.get_battery_do0 = bq27541_get_battery_do0,//log
	//.get_battery_doe = bq27541_get_battery_doe,//log
	//.get_battery_trm = bq27541_get_battery_trm,//log
	//.get_battery_pc = bq27541_get_battery_pc,//log
	//.get_battery_qs = bq27541_get_battery_qs,//log
	.get_battery_temperature = sm5602_fg_read_temperature,
	//.is_battery_present = bq27541_is_battery_present,//for check
	.get_batt_remaining_capacity = sm5602_fg_read_remain_capacity,
	.get_battery_soc = sm5602_fg_read_remain_capacity,
	.get_average_current = sm5602_fg_read_current,
	.get_battery_fcc = sm5602_fg_read_fcc,
	//.get_prev_batt_fcc = bq27541_get_prev_batt_fcc,//need
	.get_battery_cc = sm5602_fg_read_cc,
	.get_battery_soh = sm5602_fg_read_soh,
	.get_battery_authenticate = sm5602_fg_read_battery_authenticate,
	.get_battery_hmac = sm5602_fg_read_anything_bool,
	.set_battery_full = sm5602_set_battery_full,
	.get_prev_battery_mvolts = sm5602_fg_read_volt,
	.get_prev_battery_temperature = sm5602_fg_read_temperature,
	.get_prev_battery_soc = sm5602_fg_read_remain_capacity,
	.get_prev_average_current = sm5602_fg_read_current,
	.get_prev_batt_remaining_capacity   = sm5602_fg_read_remain_capacity,
	.get_battery_mvolts_2cell_max = sm5602_get_battery_mvolts_2cell_max,
	.get_battery_mvolts_2cell_min = sm5602_get_battery_mvolts_2cell_min,
	.get_prev_battery_mvolts_2cell_max = sm5602_get_battery_mvolts_2cell_max,
	.get_prev_battery_mvolts_2cell_min = sm5602_get_battery_mvolts_2cell_min,
	//.update_battery_dod0 = bq28z610_modify_dod0,
	.update_soc_smooth_parameter = sm5602_fg_read_anything_int,

	//.get_battery_cb_status = bq28z610_get_battery_balancing_status,
	//.get_gauge_i2c_err = bq27541_get_gauge_i2c_err,
	//.clear_gauge_i2c_err = bq27541_clear_gauge_i2c_err,
	//.get_passdchg = bq27541_get_passedchg,
	//.dump_register = gauge_reg_dump,
	//.protect_check = zy0603_protect_check,
	//.afi_update_done = zy0603_afi_update_done,
	//.set_allow_reading = bq27541_set_allow_reading,
	//.wlchg_started_status = bq27541_set_wlchg_started_status,
	//.get_bcc_parameters = bq28z610_get_battery_bcc_parameters,
	//.get_update_bcc_parameters = bq28z610_get_fastchg_battery_bcc_parameters,
	//.set_bcc_parameters = bq28z610_set_bcc_debug_parameters,
	//.get_prev_bcc_parameters = bq28z610_get_prev_battery_bcc_parameters,
	//.check_rc_sfr = zy0603_check_rc_sfr,
	//.soft_reset_rc_sfr = zy0603_reset_rc_sfr,	
	.get_gauge_info = sm5602g_get_info,
	//.get_batt_qmax = gauge_get_qmax,
	//.get_batt_fcc = gauge_get_fcc,
	//.get_batt_cc = gauge_get_cc,
	//.get_batt_soh = gauge_get_soh,
	//.get_calib_time = gauge_get_calib_time,
	//.cal_model_check = zy0602_cal_model_check,
};

static const u8 fg_dump_regs[] = {
	0x00, 0x01, 0x03, 0x04,
	0x05, 0x06, 0x07, 0x08,
	0x09, 0x0A, 0x0C, 0x0D,
	0x0E, 0x0F, 0x10, 0x11,
	0x12, 0x13, 0x14, 0x1A, 
	0x1B, 0x1C, 0x1E, 0x1F,
	0x26, 0x27, 0x28, 0x62,
	0x73, 0x74, 0x80, 0x83, 
	0x84, 0x90, 0x91, 0x93, 
	0x95, 0x96, 0x9C
};

static int fg_dump_debug(struct sm_fg_chip *sm)
{
	int i;
	int ret;
	u16 val = 0;

	for (i = 0; i < ARRAY_SIZE(fg_dump_regs); i++) {
		ret = fg_read_word(sm, fg_dump_regs[i], &val);
		if (!ret)
			pr_info("Reg[0x%02X] = 0x%02X\n",
						fg_dump_regs[i], val);
	}
	return 0;
}


static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct sm_fg_chip *sm = inode->i_private;

	return single_open(file, show_registers, sm);
}

static const struct file_operations reg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= reg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debugfs_entry(struct sm_fg_chip *sm)
{
	sm->debug_root = debugfs_create_dir("sm_fg", NULL);
	if (!sm->debug_root)
		pr_err("Failed to create debug dir\n");

	if (sm->debug_root) {

		debugfs_create_file("registers", S_IFREG | S_IRUGO,
						sm->debug_root, sm, &reg_debugfs_ops);

		debugfs_create_x32("fake_soc",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  sm->debug_root,
					  &(sm->fake_soc));

		debugfs_create_x32("fake_temp",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  sm->debug_root,
					  &(sm->fake_temp));

		debugfs_create_x32("skip_reads",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  sm->debug_root,
					  &(sm->skip_reads));
		debugfs_create_x32("skip_writes",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  sm->debug_root,
					  &(sm->skip_writes));
	}
}

static int show_registers(struct seq_file *m, void *data)
{
	struct sm_fg_chip *sm = m->private;
	int i;
	int ret;
	u16 val = 0;

	for (i = 0; i < ARRAY_SIZE(fg_dump_regs); i++) {
		ret = fg_read_word(sm, fg_dump_regs[i], &val);
		if (!ret)
			seq_printf(m, "Reg[0x%02X] = 0x%02X\n",
						fg_dump_regs[i], val);
	}
	return 0;
}

static int calculate_delta_time(ktime_t time_stamp, int *delta_time_s)
{
	ktime_t now_time;

	now_time = ktime_get();
	*delta_time_s = ktime_ms_delta(now_time, time_stamp);
	if (*delta_time_s < 0)
		*delta_time_s = 0;

	//bq_dbg(PR_DEBUG,  "now:%ld, last:%ld, delta:%d\n", time_now, time_last, *delta_time);
	return 0;
}

#define BATTERY_TABLE0_MG		32
#define BATTERY_TABLE1_MG		48
#define BATTERY_CAP_DIV			100
#define SOC_MIN_TH				0
static void sm5602g_table_error_check(struct sm_fg_chip *sm)
{
	bool err_flag = false;
	u16 data = 0, ocv = 0, soc = 0, qest = 0, soc_min = 0;
	int ret = 0;

	{
		ret = fg_read_word(sm, 0x80, &data);
		ret |= fg_read_word(sm, 0x05, &soc);
		ret |= fg_read_word(sm, 0x06, &ocv);
		ret |= fg_read_word(sm, 0x84, &qest);
		if (ret < 0) {
			pr_err("Failed to read data, ret = 0x%x, data = 0x%x, soc = 0x%x, ocv = 0x%x, qest = 0x%x\n", ret, data, soc, ocv, qest);
			err_flag = false;
		}
		else {
			data = data & 0x001F;
			if (data != 0) {
				if ((ocv > (sm->battery_table[BATTERY_TABLE0][data-1] - BATTERY_TABLE0_MG)) && (ocv < sm->battery_table[BATTERY_TABLE0][data] + BATTERY_TABLE0_MG)) {
					if (data == 1) {
						soc_min = SOC_MIN_TH;
					}
					else {
						soc_min = sm->battery_table[BATTERY_TABLE1][data-1] - BATTERY_TABLE1_MG;
					}
					if ((soc >= soc_min) && (soc < sm->battery_table[BATTERY_TABLE1][data] + BATTERY_TABLE1_MG)) {
						err_flag = false;
					}
					else {
						err_flag = true;
						pr_info("err_true.. ref ocv = 0x%x, data = 0x%x, soc = 0x%x, ocv = 0x%x, qest = 0x%x\n", sm->battery_table[0][data], data, soc, ocv, qest);
					}
				}
				else {
					err_flag = true;
					pr_info("err_true.. ref soc = 0x%x, data = 0x%x, soc = 0x%x, ocv = 0x%x, qest = 0x%x\n", sm->battery_table[1][data], data, soc, ocv, qest);
				}

				if(!err_flag) {
					if (qest > (sm->cap + (sm->cap/100))) {
						pr_info("err_true.. ref soc = 0x%x, data = 0x%x, soc = 0x%x, ocv = 0x%x, qest = 0x%x\n", sm->battery_table[1][data], data, soc, ocv, qest);
						err_flag = true;
					}
				}
			}
			else {
				pr_info("state is zero, ref ocv = 0x%x, data = 0x%x, soc = 0x%x, ocv = 0x%x, qest = 0x%x\n", sm->battery_table[0][data], data, soc, ocv, qest);
			}
		}
	}

	if(err_flag) {
		pr_err("sm5602g_table_error_check err_flag true,do reset\n", err_flag);
		fg_reset(sm);
		mutex_lock(&sm->data_lock);
		fg_reg_init(sm->client);
		mutex_unlock(&sm->data_lock);
	}
}

static void fg_refresh_status(struct sm_fg_chip *sm)
{
	bool last_batt_inserted;

	last_batt_inserted	= sm->batt_present;

	fg_read_status(sm);

	pr_info("batt_present=%d", sm->batt_present);

	if (!last_batt_inserted && sm->batt_present) {/* battery inserted */
		pr_info("Battery inserted\n");
	} else if (last_batt_inserted && !sm->batt_present) {/* battery removed */
		pr_info("Battery removed\n");
		sm->batt_soc	= -ENODATA;
		sm->batt_fcc	= -ENODATA;
		sm->batt_volt	= -ENODATA;
		sm->batt_curr	= -ENODATA;
		sm->batt_temp	= -ENODATA;
		sm->batt_rmc	= -ENODATA;
	}

	if (sm->batt_present) {	
		sm->batt_soc = fg_read_soc(sm);
		sm->batt_ocv = fg_read_ocv(sm);
		sm->batt_volt = fg_read_volt(sm);
		sm->batt_curr = fg_read_current(sm);
		sm->batt_soc_cycle = fg_get_cycle(sm);
		if (sm->en_temp_in)
			sm->batt_temp = fg_read_temperature(sm, TEMPERATURE_IN);
		else if (sm->en_temp_ex)
			sm->batt_temp = fg_read_temperature(sm, TEMPERATURE_EX);
		else if (sm->en_temp_3rd)
			sm->batt_temp = fg_read_temperature(sm, TEMPERATURE_3RD);		
		else 
			sm->batt_temp = -ENODATA;
		sm->batt_rmc = fg_read_rmc(sm);
		fg_cal_carc(sm);
		
		pr_info("RSOC:%d, Volt:%d, Current:%d, Temperature:%d, OCV:%d, RMC:%d\n",
			sm->batt_soc, sm->batt_volt, sm->batt_curr, sm->batt_temp, sm->batt_ocv, sm->batt_rmc);	
	}

	if (sm->enable_check_abnormal_table)
		sm5602g_table_error_check(sm);

	fg_get_soh(sm);
}

static unsigned int poll_interval = 10; /*  10 sec */
static void fg_monitor_workfunc(struct work_struct *work)
{
	struct sm_fg_chip *sm = container_of(work, struct sm_fg_chip,
								monitor_work.work);

	mutex_lock(&sm->data_lock);
	fg_init(sm->client);
	mutex_unlock(&sm->data_lock);

	fg_refresh_status(sm);

	if (poll_interval > 0) {
		schedule_delayed_work(&sm->monitor_work, msecs_to_jiffies(poll_interval * 1000)); /* poll_interval(10) * 1000 = 10 sec */
	}

}

#define COMMON_PARAM_MASK		0x00C0
#define COMMON_PARAM_SHIFT		6
#define BATTERY_PARAM_MASK		0x003C
#define BATTERY_PARAM_SHIFT		2
#define RSNS_HR_MASK			0x0003

static bool fg_check_reg_init_need(struct i2c_client *client)
{
	struct sm_fg_chip *sm = i2c_get_clientdata(client); 
	int ret = 0;
	u16 data = 0;
	u16 param_ver = 0;
	
	ret = fg_read_word(sm, sm->regs[SM_FG_REG_FG_OP_STATUS], &data);
	if (ret < 0) {
			pr_err("Failed to read param_ctrl unlock, ret = %d\n", ret);
			return ret;
	} else {
		pr_info("FG_OP_STATUS = 0x%x\n", data);	

		ret = fg_read_word(sm, FG_PARAM_VERION, &param_ver);
		if (ret < 0) {
				pr_err("Failed to read FG_PARAM_VERION, ret = %d\n", ret);
				return ret;
		} 

		pr_info("param_ver = 0x%x, common_param_version = 0x%x, battery_param_version = 0x%x\n", param_ver, sm->common_param_version, sm->battery_param_version);

		if ((((param_ver & COMMON_PARAM_MASK) >> COMMON_PARAM_SHIFT) >= sm->common_param_version) 
			&& (((param_ver & BATTERY_PARAM_MASK) >> BATTERY_PARAM_SHIFT) >= sm->battery_param_version)) {

			if ((data & INIT_CHECK_MASK) == DISABLE_RE_INIT) {
				pr_info("SM_FG_REG_FG_OP_STATUS : 0x%x , return FALSE NO init need\n", data);
				return 0;
			}
			else
			{
				pr_info("SM_FG_REG_FG_OP_STATUS : 0x%x , return TRUE init need!!!!\n", data);
				return 1;
			}
		}
		else
		{
			if ((data & INIT_CHECK_MASK) == DISABLE_RE_INIT) {
				// Step1. Turn off charger
				// Step2. FG Reset
				ret = fg_reset(sm);
				if (ret < 0) {
					pr_err("%s: fail to do reset(%d) retry\n", __func__, ret);
					ret = fg_reset(sm);
					if (ret < 0) {
						pr_err("%s: fail to do reset(%d) retry_2\n", __func__, ret);
						ret = fg_reset(sm);
						if (ret < 0) {
							pr_err("%s: fail to do reset(%d) reset fail 3 times!!!! return 0!!!!\n", __func__, ret);
							return 0;
						}
					}
				}
				// Step3. If Step1 was charging-on, turn on charger.
				
				pr_info("SM_FG_REG_FG_OP_STATUS : 0x%x , return TRUE init need because diff_ver SW reset!!!!\n", data);
				return 1;

			}
			else
			{
				pr_info("SM_FG_REG_FG_OP_STATUS : 0x%x , return TRUE init need!!!!\n", data);
				return 1;
			}
		}
	}
}

#define MINVAL(a, b) ((a <= b) ? a : b)
#define MAXVAL(a, b) ((a > b) ? a : b)
static int fg_calculate_iocv(struct sm_fg_chip *sm)
{
	bool only_lb=false, sign_i_offset=0; //valid_cb=false, 
	int roop_start=0, roop_max=0, i=0, cb_last_index = 0, cb_pre_last_index =0;
	int lb_v_buffer[FG_INIT_B_LEN+1] = {0, 0, 0, 0, 0, 0, 0, 0};
	int lb_i_buffer[FG_INIT_B_LEN+1] = {0, 0, 0, 0, 0, 0, 0, 0};
	int cb_v_buffer[FG_INIT_B_LEN+1] = {0, 0, 0, 0, 0, 0, 0, 0};
	int cb_i_buffer[FG_INIT_B_LEN+1] = {0, 0, 0, 0, 0, 0, 0, 0};
	int i_offset_margin = 0x14, i_vset_margin = 0x67;
	int v_max=0, v_min=0, v_sum=0, lb_v_avg=0, cb_v_avg=0, lb_v_set=0, lb_i_set=0, i_offset=0;
	int i_max=0, i_min=0, i_sum=0, lb_i_avg=0, cb_i_avg=0, cb_v_set=0, cb_i_set=0;
	int lb_i_p_v_min=0, lb_i_n_v_max=0, cb_i_p_v_min=0, cb_i_n_v_max=0;

	u16 v_ret, i_ret = 0;
	int ret=0;

	u16 data = 0;

	ret = fg_read_word(sm, FG_REG_END_V_IDX, &data);
	if (ret < 0) {
		pr_err("Failed to read FG_REG_END_V_IDX, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("iocv_status_read = addr : 0x%x , data : 0x%x\n", FG_REG_END_V_IDX, data);
	}

	if((data & 0x0010) == 0x0000)
	{
		only_lb = true;
	}

    roop_max = (data & 0x000F);
    if(roop_max > FG_INIT_B_LEN)
    {
        roop_max = FG_INIT_B_LEN;
    }
	else if (roop_max < 3)
	{
		pr_info("iocv buffer count is too small, ret : 0x%x\n",ret);
		lb_v_set = ((fg_read_volt(sm)<<8)/125);
		lb_i_set = ((fg_read_current(sm)<<8)/125);
		ret = lb_v_set;
		if (sm->enable_iocv_adj) {
			if (((lb_v_set < sm->iocv_max_adj_level) && (lb_v_set > sm->iocv_min_adj_level)) && ((abs(lb_i_set) < sm->ioci_max_adj_level) && (abs(lb_i_set) > sm->ioci_min_adj_level)))
			{
				ret = ret - (((lb_i_set * sm->iocv_i_slope) + sm->iocv_i_offset) / 1000);
			pr_info("first boot vbat-soc adjust 1st_v=0x%x, 2nd_v=0x%x, all_i=0x%x\n", lb_v_set, ret, lb_i_set);
			}
		}
	    pr_info("iocv ret=%d, table=%d \n",ret, sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN-1]);
	    if(ret > sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN-1])
	    {
	        pr_info("iocv ret change hex 0x%x -> 0x%x ret is big than table data\n",ret, sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN-1]);
		    pr_info("iocv ret change dec %d -> %d ret is big than table data\n",ret, sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN-1]);
	        ret = sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN-1];
	    }
	    else if(ret < sm->battery_table[BATTERY_TABLE0][0])
	    {
	        pr_info("iocv ret change 0x%x -> 0x%x \n", ret, (sm->battery_table[BATTERY_TABLE0][0] + 0x10));
	        ret = sm->battery_table[BATTERY_TABLE0][0] + 0x10;
	    }

		return ret;
	}

	roop_start = FG_REG_START_LB_V;
	for (i = roop_start; i < roop_start + roop_max; i++)
	{
		//ret = fg_read_word(sm, i, &v_ret);
		ret = fg_write_word(sm, 0x8C, i);
		if (ret < 0) {
			pr_err("Failed to write 0x8C, %d ret = %d\n", i, ret);
			return ret;
		}
		msleep(15); //15mS delay 		
		ret = fg_read_word(sm, 0x8D, &v_ret);
		if (v_ret & 0x8000) {
			ret = fg_read_word(sm, 0x8D, &v_ret);
			msleep(15); //15mS delay
		}		
		if (ret < 0) {
			pr_err("Failed to read 0x%x, ret = %d\n",i, ret);
			return ret;
		}
		pr_info("iocv read = addr : 0x%x , data : 0x%x\n", i, v_ret);
		
		//ret = fg_read_word(sm, i+0x10, &i_ret);
		ret = fg_write_word(sm, 0x8C, i+0x10);
		if (ret < 0) {
			pr_err("Failed to write 0x8C, %d ret = %d\n", i+0x10, ret);
			return ret;
		}
		msleep(15); //15mS delay 		
		ret = fg_read_word(sm, 0x8D, &i_ret);
		if (i_ret & 0x8000) {
			ret = fg_read_word(sm, 0x8D, &i_ret);
			msleep(15); //15mS delay
		}		
		if (ret < 0) {
			pr_err("Failed to read 0x%x, ret = %d\n",i, ret);
			return ret;
		}
		pr_info("ioci read = addr : 0x%x , data : 0x%x\n", i+0x10, i_ret);

		if((i_ret&0x4000) == 0x4000)
		{
			i_ret = -(i_ret&0x3FFF);
		}

		lb_v_buffer[i-roop_start] = v_ret;
		lb_i_buffer[i-roop_start] = i_ret;

		if (i == roop_start)
		{
			v_max = v_ret;
			v_min = v_ret;
			v_sum = v_ret;
			i_max = i_ret;
			i_min = i_ret;
			i_sum = i_ret;
		}
		else
		{
			if(v_ret > v_max)
				v_max = v_ret;
			else if(v_ret < v_min)
				v_min = v_ret;
			v_sum = v_sum + v_ret;

			if(i_ret > i_max)
				i_max = i_ret;
			else if(i_ret < i_min)
				i_min = i_ret;
			i_sum = i_sum + i_ret;
		}

		if(abs(i_ret) > i_vset_margin)
		{
			if(i_ret > 0)
			{
				if(lb_i_p_v_min == 0)
				{
					lb_i_p_v_min = v_ret;
				}
				else
				{
					if(v_ret < lb_i_p_v_min)
						lb_i_p_v_min = v_ret;
				}
			}
			else
			{
				if(lb_i_n_v_max == 0)
				{
					lb_i_n_v_max = v_ret;
				}
				else
				{
					if(v_ret > lb_i_n_v_max)
						lb_i_n_v_max = v_ret;
				}
			}
		}
	}
	v_sum = v_sum - v_max - v_min;
	i_sum = i_sum - i_max - i_min;

	lb_v_avg = v_sum / (roop_max-2);
	lb_i_avg = i_sum / (roop_max-2);

	if(abs(lb_i_buffer[roop_max-1]) < i_vset_margin)
	{
		if(abs(lb_i_buffer[roop_max-2]) < i_vset_margin)
		{
			lb_v_set = MAXVAL(lb_v_buffer[roop_max-2], lb_v_buffer[roop_max-1]);
			if(abs(lb_i_buffer[roop_max-3]) < i_vset_margin)
			{
				lb_v_set = MAXVAL(lb_v_buffer[roop_max-3], lb_v_set);
			}
		}
		else
		{
			lb_v_set = lb_v_buffer[roop_max-1];
		}
	}
	else
	{
		lb_v_set = lb_v_avg;
	}

	if(lb_i_n_v_max > 0)
	{
		lb_v_set = MAXVAL(lb_i_n_v_max, lb_v_set);
	}

	if(roop_max > 3)
	{
		lb_i_set = (lb_i_buffer[2] + lb_i_buffer[3]) / 2;
	}

	if((abs(lb_i_buffer[roop_max-1]) < i_offset_margin) && (abs(lb_i_set) < i_offset_margin))
	{
		lb_i_set = MAXVAL(lb_i_buffer[roop_max-1], lb_i_set);
	}
	else if(abs(lb_i_buffer[roop_max-1]) < i_offset_margin)
	{
		lb_i_set = lb_i_buffer[roop_max-1];
	}
	else if(abs(lb_i_set) < i_offset_margin)
	{
		//lb_i_set = lb_i_set;
	}
	else
	{
		lb_i_set = 0;
	}

	i_offset = lb_i_set;

	i_offset = i_offset + 4;

	if(i_offset <= 0)
	{
		sign_i_offset = 1;
#ifdef IGNORE_N_I_OFFSET
		i_offset = 0;
#else
		i_offset = -i_offset;
#endif
	}

	i_offset = i_offset>>1;

	if(sign_i_offset == 0)
	{
		i_offset = i_offset|0x0080;
	}
    i_offset = i_offset | i_offset<<8;

	pr_info("iocv_l_max=0x%x, iocv_l_min=0x%x, iocv_l_avg=0x%x, lb_v_set=0x%x, roop_max=%d \n",
			v_max, v_min, lb_v_avg, lb_v_set, roop_max);
	pr_info("ioci_l_max=0x%x, ioci_l_min=0x%x, ioci_l_avg=0x%x, lb_i_set=0x%x, i_offset=0x%x, sign_i_offset=%d\n",
			i_max, i_min, lb_i_avg, lb_i_set, i_offset, sign_i_offset);

	if(!only_lb)
	{
		roop_start = FG_REG_START_CB_V;
		roop_max = 6;
		for (i = roop_start; i < roop_start + roop_max; i++)
		{
			//ret = fg_read_word(sm, i, &v_ret);
			ret = fg_write_word(sm, 0x8C, i);
			if (ret < 0) {
				pr_err("Failed to write 0x8C, %d ret = %d\n", i, ret);
				return ret;
			}
			msleep(15); //15mS delay 		
			ret = fg_read_word(sm, 0x8D, &v_ret);
			if (v_ret & 0x8000) {
				ret = fg_read_word(sm, 0x8D, &v_ret);
				msleep(15); //15mS delay
			}				
			if (ret < 0) {
				pr_err("Failed to read 0x%x, ret = %d\n",i, ret);
				return ret;
			}
			//ret = fg_read_word(sm, i+0x10, &i_ret);
			ret = fg_write_word(sm, 0x8C, i+0x10);
			if (ret < 0) {
				pr_err("Failed to write 0x8C, %d ret = %d\n", i+0x10, ret);
				return ret;
			}
			msleep(15); //15mS delay 		
			ret = fg_read_word(sm, 0x8D, &i_ret);
			if (i_ret & 0x8000) {
				ret = fg_read_word(sm, 0x8D, &i_ret);
				msleep(15); //15mS delay
			}				
			if (ret < 0) {
				pr_err("Failed to read 0x%x, ret = %d\n",i, ret);
				return ret;
			}

			if((i_ret&0x4000) == 0x4000)
			{
				i_ret = -(i_ret&0x3FFF);
			}

			cb_v_buffer[i-roop_start] = v_ret;
			cb_i_buffer[i-roop_start] = i_ret;

			if (i == roop_start)
			{
				v_max = v_ret;
				v_min = v_ret;
				v_sum = v_ret;
				i_max = i_ret;
				i_min = i_ret;
				i_sum = i_ret;
			}
			else
			{
				if(v_ret > v_max)
					v_max = v_ret;
				else if(v_ret < v_min)
					v_min = v_ret;
				v_sum = v_sum + v_ret;

				if(i_ret > i_max)
					i_max = i_ret;
				else if(i_ret < i_min)
					i_min = i_ret;
				i_sum = i_sum + i_ret;
			}

			if(abs(i_ret) > i_vset_margin)
			{
				if(i_ret > 0)
				{
					if(cb_i_p_v_min == 0)
					{
						cb_i_p_v_min = v_ret;
					}
					else
					{
						if(v_ret < cb_i_p_v_min)
							cb_i_p_v_min = v_ret;
					}
				}
				else
				{
					if(cb_i_n_v_max == 0)
					{
						cb_i_n_v_max = v_ret;
					}
					else
					{
						if(v_ret > cb_i_n_v_max)
							cb_i_n_v_max = v_ret;
					}
				}
			}
		}
		v_sum = v_sum - v_max - v_min;
		i_sum = i_sum - i_max - i_min;

		cb_v_avg = v_sum / (roop_max-2);
		cb_i_avg = i_sum / (roop_max-2);

		cb_last_index = (data & 0x000F)-7; //-6-1
		if(cb_last_index < 0)
		{
			cb_last_index = 5;
		}

    		if(cb_last_index > FG_INIT_B_LEN)
    		{
        		cb_last_index = FG_INIT_B_LEN;
		}

		for (i = roop_max; i > 0; i--)
		{
			if(abs(cb_i_buffer[cb_last_index]) < i_vset_margin)
			{
				cb_v_set = cb_v_buffer[cb_last_index];
				if(abs(cb_i_buffer[cb_last_index]) < i_offset_margin)
				{
					cb_i_set = cb_i_buffer[cb_last_index];
				}

				cb_pre_last_index = cb_last_index - 1;
				if(cb_pre_last_index < 0)
				{
					cb_pre_last_index = 5;
				}

				if(abs(cb_i_buffer[cb_pre_last_index]) < i_vset_margin)
				{
					cb_v_set = MAXVAL(cb_v_buffer[cb_pre_last_index], cb_v_set);
					if(abs(cb_i_buffer[cb_pre_last_index]) < i_offset_margin)
					{
						cb_i_set = MAXVAL(cb_i_buffer[cb_pre_last_index], cb_i_set);
					}
				}
			}
			else
			{
				cb_last_index--;
				if(cb_last_index < 0)
				{
					cb_last_index = 5;
				}
			}
		}

		if(cb_v_set == 0)
		{
			cb_v_set = cb_v_avg;
			if(cb_i_set == 0)
			{
				cb_i_set = cb_i_avg;
			}
		}

		if(cb_i_n_v_max > 0)
		{
			cb_v_set = MAXVAL(cb_i_n_v_max, cb_v_set);
		}

		if(abs(cb_i_set) < i_offset_margin)
		{
			if(cb_i_set > lb_i_set)
			{
				i_offset = cb_i_set;
				i_offset = i_offset + 4;

				if(i_offset <= 0)
				{
					sign_i_offset = 1;
#ifdef IGNORE_N_I_OFFSET
					i_offset = 0;
#else
					i_offset = -i_offset;
#endif
				}

				i_offset = i_offset>>1;

				if(sign_i_offset == 0)
				{
					i_offset = i_offset|0x0080;
				}
                i_offset = i_offset | i_offset<<8;

			}
		}

		pr_info("iocv_c_max=0x%x, iocv_c_min=0x%x, iocv_c_avg=0x%x, cb_v_set=0x%x, cb_last_index=%d\n",
				v_max, v_min, cb_v_avg, cb_v_set, cb_last_index);
		pr_info("ioci_c_max=0x%x, ioci_c_min=0x%x, ioci_c_avg=0x%x, cb_i_set=0x%x, i_offset=0x%x, sign_i_offset=%d\n",
				i_max, i_min, cb_i_avg, cb_i_set, i_offset, sign_i_offset);

	}

    /* final set */
	pr_info("cb_i_set=%d, i_vset_margin=%d, only_lb=%d\n", cb_i_set, i_vset_margin, only_lb);
	if (only_lb) 
	{
		ret = MAXVAL(lb_v_set, cb_i_n_v_max);
		cb_i_set = lb_i_avg;
	}
	else 
	{
		ret = cb_v_set;
		cb_i_set = cb_i_avg;
	}

	if (sm->enable_iocv_adj) {
		if (((ret < sm->iocv_max_adj_level) && (ret > sm->iocv_min_adj_level)) && ((abs(cb_i_set) < sm->ioci_max_adj_level) && (abs(cb_i_set) > sm->ioci_min_adj_level)))
		{
			cb_v_set = ret;
			ret = ret - (((cb_i_set * sm->iocv_i_slope) + sm->iocv_i_offset) / 1000);
		pr_info("first boot vbat-soc adjust 1st_v=0x%x, 2nd_v=0x%x, all_i=0x%x\n", cb_v_set, ret, cb_i_set);
		}
	}

    pr_info("iocv ret=%d, table=%d \n",ret, sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN-1]);
    if(ret > sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN-1])
    {
        pr_info("iocv ret change hex 0x%x -> 0x%x ret is big than table data\n",ret, sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN-1]);
	    pr_info("iocv ret change dec %d -> %d ret is big than table data\n",ret, sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN-1]);
        ret = sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN-1];
    }
    else if(ret < sm->battery_table[BATTERY_TABLE0][0])
    {
        pr_info("iocv ret change 0x%x -> 0x%x \n", ret, (sm->battery_table[BATTERY_TABLE0][0] + 0x10));
        ret = sm->battery_table[BATTERY_TABLE0][0] + 0x10;
    }

	return ret;
}

#define FG_TABLE_UNLOCK			0x03
#define FG_TABLE_UNLOCK_CNT		5
#define FG_DELAY_15MS			15
#define FG_DELAY_20MS			20
#define FG_DELAY_60MS			60
#define FG_DELAY_160MS			160
#define FG_DELAY_200MS			200
#define FG_DELAY_300MS			300
#define FG_SRDATA_WAIT_MASK		0x8000
#define FG_REG_STABLE2			0x60
#define FG_BATTERY_ID_SHIFT		8
static bool fg_reg_init(struct i2c_client *client)
{
	struct sm_fg_chip *sm = i2c_get_clientdata(client);
	int i, j, value, ret, cnt = 0;
	uint8_t table_reg;
	u16 tem_value, data;
	int k, m = 0;

	pr_info("sm5602g_fg_reg_init START!!\n");

	do {		
		ret = fg_write_word(sm, sm->regs[SM_FG_REG_PARAM_CTRL], (FG_PARAM_UNLOCK_CODE | ((sm->battery_table_num & 0x0003) << 6) | (FG_TABLE_LEN-1)));
		if (ret < 0) {
			pr_err("Failed to write param_ctrl unlock, ret = %d\n", ret);
			return ret;
		} else {
			pr_info("Param Unlock\n");
		}
		//msleep(3);
		msleep(60);
		ret = fg_read_word(sm, sm->regs[SM_FG_REG_FG_OP_STATUS], &data);
		if (ret < 0){
			pr_err("Failed to read FG_OP_STATUS, ret = %d\n", ret);	
		} else {
			pr_info(" FG_OP_STATUS = 0x%x\n", data);
		}
		cnt++;

		if((data & FG_TABLE_UNLOCK)!=FG_TABLE_UNLOCK)
			msleep(300);

	} while(((data & FG_TABLE_UNLOCK)!=FG_TABLE_UNLOCK) && cnt <= FG_TABLE_UNLOCK_CNT);

	/*Temperature/Batt Det -  control register set*/
	ret = fg_read_word(sm, sm->regs[SM_FG_REG_CNTL], &data);
	if (ret < 0) {
			pr_err("Failed to read CNTL, ret = %d\n", ret);
			return ret;
	}

	if (sm->en_temp_in)
		data |= ENABLE_EN_TEMP_IN;
	if (sm->en_temp_ex)
		data |= ENABLE_EN_TEMP_EX;;

	ret = fg_write_word(sm, sm->regs[SM_FG_REG_CNTL], data);
	if (ret < 0) {
		pr_err("Failed to write CNTL, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("CNTL = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_CNTL], data);
	}

	/* RPARA */
	ret = fg_write_word(sm, sm->regs[SM_FG_REG_VOL_COMP], sm->rpara);
	if (ret < 0) {
		pr_err("Failed to write SM_FG_REG_VOL_COMP, ret = %d\n", ret);
		return ret;
	} else {
			pr_info("Write SM_FG_REG_VOL_COMP = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_VOL_COMP], sm->rpara);
	}

	/* CURRENT OFFSET */
	ret = fg_write_word(sm, sm->regs[SM_FG_REG_CURR_OFFSET], sm->curr_voffset);
	if (ret < 0) {
		pr_err("Failed to write SM_FG_REG_CURR_OFFSET, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("Write SM_FG_REG_CURR_OFFSET = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_CURR_OFFSET], sm->curr_voffset);
	}

	/* CURRENT SLOPE */
	ret = fg_write_word(sm, sm->regs[SM_FG_REG_CURR_SLOPE], sm->curr_vslope);
	if (ret < 0) {
		pr_err("Failed to write SM_FG_REG_CURR_SLOPE, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("Write SM_FG_REG_CURR_SLOPE = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_CURR_SLOPE], sm->curr_vslope);
	}

	/* VIT_PERIOD write */
	ret = fg_write_word(sm, sm->regs[SM_FG_REG_VIT_PERIOD], sm->vit_period);
	if (ret < 0) {
		pr_err("Failed to write VIT PERIOD, ret = %d\n", ret);
		return ret;
	} else {
			pr_info("Write VIT_PERIOD = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_VIT_PERIOD], sm->vit_period);
	}

	/* Aging ctrl write */
	ret = fg_write_word(sm, FG_REG_AGING_CTRL, sm->aging_ctrl);
	if (ret < 0) {
		pr_err("Failed to write FG_REG_AGING_CTRL, ret = %d\n", ret);
		return ret;
	} else {
			pr_info("Write FG_REG_AGING_CTRL = 0x%x : 0x%x\n", FG_REG_AGING_CTRL, sm->aging_ctrl);
	}

	/* SOC Cycle ctrl write */
	ret = fg_write_word(sm, FG_REG_SOC_CYCLE_CFG, sm->cycle_cfg);
	if (ret < 0) {
		pr_err("Failed to write FG_REG_SOC_CYCLE_CFG, ret = %d\n", ret);
		return ret;
	} else {
			pr_info("Write FG_REG_SOC_CYCLE_CFG = 0x%x : 0x%x\n", FG_REG_SOC_CYCLE_CFG, sm->cycle_cfg);
	}

	/*RSNS write */
	if (sm->batt_rsns > 5) {
		ret = fg_write_word(sm, sm->regs[SM_FG_REG_RSNS_SEL], sm->batt_rsns/10);
		pr_info("Write SM_FG_REG_RSNS_SEL = 0x%x : 0x%x, batt_rsns = %d\n", sm->regs[SM_FG_REG_RSNS_SEL], sm->batt_rsns/10, sm->batt_rsns);
	} else {
		ret = fg_write_word(sm, sm->regs[SM_FG_REG_RSNS_SEL], 0x0000);
		pr_info("Write SM_FG_REG_RSNS_SEL = 0x%x : 0x%x, batt_rsns = %d\n", sm->regs[SM_FG_REG_RSNS_SEL], 0x0000, sm->batt_rsns);
		if (sm->batt_rsns < 3 && sm->batt_rsns > 0 ) {
			ret |= fg_write_word(sm, sm->regs[SM_FG_REG_RSNS_HR], sm->batt_rsns+1);
			pr_info("Write SM_FG_REG_RSNS_HR = 0x%x : 0x%x, batt_rsns = %d\n", sm->regs[SM_FG_REG_RSNS_HR], sm->batt_rsns+1, sm->batt_rsns);
		}
	}
	if (ret < 0) {
		pr_err("Failed to write SM_FG_REG_RSNS_SEL & SM_FG_REG_RSNS_HR, ret = %d\n", ret);
		return ret;
	}

	/* Battery_Table write */
	for (i = BATTERY_TABLE0; i < BATTERY_TABLE2; i++) {
		table_reg = FG_REG_TABLE0_A0 + (i*FG_TABLE_MAX_LEN);
		for (j = 0; j < FG_TABLE_LEN; j++) {
			ret = fg_write_word(sm, (table_reg + j), sm->battery_table[i][j]);
			if (ret < 0) {
				pr_err("Failed to write Battery Table, ret = %d\n", ret);
				msleep(10);
				ret = fg_write_word(sm, (table_reg + j), sm->battery_table[i][j]);
				if (ret < 0) {
					pr_err("twice Failed to write Battery Table, ret = %d\n", ret);
					return ret;
				}
			} else {
				pr_info("TABLE write OK [%d][%d] = 0x%x : 0x%x\n",
					i, j, (table_reg + j), sm->battery_table[i][j]);
			}
			msleep(10); //10mS delay
		}
	}	

	if (sm->enable_inspection_table) {
		/*--------------for verify table data write----------------------------*/	
			for (i = BATTERY_TABLE0; i < BATTERY_TABLE2; i++) {
				table_reg = FG_REG_TABLE0_A0 + (i*FG_TABLE_MAX_LEN);
				for (j = 0; j < FG_TABLE_LEN; j++) {
					/* 1. Read Table Value */
					ret = fg_write_word(sm, FG_REG_SRADDR, j+(i*FG_TABLE_MAX_LEN));
					if (ret < 0) {
						pr_err("Failed to write FG_REG_SRADDR, %d ret = %d\n", j+(i*FG_TABLE_MAX_LEN), ret);
					}
					msleep(15); //15mS delay 		
					ret = fg_read_word(sm, FG_REG_SRDATA, &data);
					ret = fg_read_word(sm, FG_REG_SRDATA, &data); /* Twice */
					for (k = 1; k <= I2C_ERROR_COUNT_MAX; k++) {
						if (data & 0x8000) {
							msleep(15); //15mS delay
							ret = fg_read_word(sm, FG_REG_SRDATA, &data);
							ret = fg_read_word(sm, FG_REG_SRDATA, &data); /* Twice */	
						} else {
							break;
						}
					}
					if (ret < 0) {
						pr_err("Failed to read FG_REG_SRDATA, %d ret = %d\n", data, ret);
					}
				
				chg_err("%s:%d--22------kongfanhong, [%d, %d][0x%x, 0x%x]\n", __func__, __LINE__, i, j, sm->battery_table[i][j], data);
				/* 2. Compare Table Value */
				if (sm->battery_table[i][j] == data) {
					// 2-1. Table value is same as dtsi value 
					pr_info("TABLE data verify OK [%d][%d] = 0x%x : 0x%x\n",
						i, j, (table_reg + j), data);
				} else {
					/* 3. Table value is not same as dtsi value */
					for (k = 1; k <= I2C_ERROR_COUNT_MAX; k++) {
						pr_info("TABLE write data ERROR!!!! rewrite [%d][%d] = 0x%x : 0x%x, count=%d\n",
						i, j, (table_reg + j), sm->battery_table[i][j], k);
						// 3-1. Rewrite Battery Table
						ret = fg_write_word(sm, (table_reg + j), sm->battery_table[i][j]);
						if (ret < 0) {
							pr_err("Failed to write Battery Table, ret = %d\n", ret);
						} else {
							pr_info("TABLE write OK [%d][%d] = 0x%x : 0x%x\n",
								i, j, (table_reg + j), sm->battery_table[i][j]);
						}
						msleep(15); //15mS delay

						// 3-2 Read Battery Table 
						ret = fg_write_word(sm, FG_REG_SRADDR, j+(i*FG_TABLE_MAX_LEN));					
						if (ret < 0) {
							pr_err("Failed to write Battery Table, ret = %d\n", ret);
						}
						msleep(15); //15mS delay					
							ret = fg_read_word(sm, FG_REG_SRDATA, &data);
							ret = fg_read_word(sm, FG_REG_SRDATA, &data); /* Twice */
						for (m = 1; m <= I2C_ERROR_COUNT_MAX; m++) {
							if (data & 0x8000) {
								msleep(15); //15mS delay
									ret = fg_read_word(sm, FG_REG_SRDATA, &data);
									ret = fg_read_word(sm, FG_REG_SRDATA, &data); /* Twice */	
							} else {
								break;
							}
						}
						// 3-3 Recompare Battery Table with dtsi value
						if (sm->battery_table[i][j] == data) {
							pr_info("TABLE rewrite OK [%d][%d] = 0x%x : 0x%x, count=%d\n",
							i, j, (table_reg + j), data, k);
							break;
						}
						// 3-4 Under I2C_ERROR_COUNT_MAX(5), re-try verify process.
						if (k >= I2C_ERROR_COUNT_MAX) {
							pr_err("rewrite %d times, Fail\n", I2C_ERROR_COUNT_MAX);
						}
					}
				}
			}
		}
	}

	for(j=0; j < FG_ADD_TABLE_LEN; j++)
	{
		table_reg = FG_REG_TABLE2_D0 + j;
		ret = fg_write_word(sm, table_reg, sm->battery_table[i][j]);
		if (ret < 0) {
			pr_err("Failed to write Battery Table, ret = %d\n", ret);
			return ret;
		} else {
			pr_info("TABLE write OK [%d][%d] = 0x%x : 0x%x\n",
				i, j, table_reg, sm->battery_table[i][j]);
		}
		msleep(10); //10mS delay
	}

	if (sm->enable_inspection_table) {
		/*--------------for verify table data write----------------------------*/
		for (j = 0; j < FG_ADD_TABLE_LEN; j++) {
			i = BATTERY_TABLE2;
			table_reg = FG_REG_TABLE2_D0 + j;
			chg_err("%s:%d--111------kongfanhong, [0x%x, 0x%x]\n", __func__, __LINE__, j, table_reg);
			/* 1. Read Table Value */
			ret = fg_write_word(sm, FG_REG_SRADDR, (j+FG_REG_STABLE2));
			if (ret < 0) {
				pr_err("Failed to write FG_REG_SRADDR, %d ret = %d\n", table_reg, ret);
			}
			msleep(15); //15mS delay		
			ret = fg_read_word(sm, FG_REG_SRDATA, &data);
			ret = fg_read_word(sm, FG_REG_SRDATA, &data); /* Twice */
			for (k = 1; k <= I2C_ERROR_COUNT_MAX; k++) {
				if (data & 0x8000) {
					msleep(15); //15mS delay
					ret = fg_read_word(sm, FG_REG_SRDATA, &data);
					ret = fg_read_word(sm, FG_REG_SRDATA, &data); /* Twice */	
				} else {
					break;
				}
			}
			if (ret < 0) {
			pr_err("Failed to read 0x8D, %d ret = %d\n", data, ret);
			}
				/* 2. Compare Table Value */
			if (sm->battery_table[i][j] == data) {
				// 2-1. Table value is same as dtsi value 
				pr_info("TABLE data verify OK [%d][%d] = 0x%x : 0x%x\n",
					i, j, table_reg, data);
			} else {
				/* 3. Table value is not same as dtsi value */
				for (k = 1; k <= I2C_ERROR_COUNT_MAX; k++) {
					pr_info("TABLE write data ERROR!!!! rewrite [%d][%d] = 0x%x : 0x%x, count=%d\n",
					i, j, table_reg, sm->battery_table[i][j], k);
					// 3-1. Rewrite Battery Table
					ret = fg_write_word(sm, table_reg, sm->battery_table[i][j]);
					if (ret < 0) {
						pr_err("Failed to write Battery Table, ret = %d\n", ret);
					} else {
						pr_info("TABLE write OK [%d][%d] = 0x%x : 0x%x\n",
							i, j, table_reg, sm->battery_table[i][j]);
					}
					msleep(15); //15mS delay

					// 3-2 Read Battery Table 
					ret = fg_write_word(sm, FG_REG_SRADDR, (j+FG_REG_STABLE2));					
					if (ret < 0) {
						pr_err("Failed to write Battery Table, ret = %d\n", ret);
					}
					msleep(15); //15mS delay					
					ret = fg_read_word(sm, FG_REG_SRDATA, &data);
					ret = fg_read_word(sm, FG_REG_SRDATA, &data); /* Twice */
					for (m = 1; m <= I2C_ERROR_COUNT_MAX; m++) {
						if (data & 0x8000) {
							msleep(15); //15mS delay
							ret = fg_read_word(sm, FG_REG_SRDATA, &data);
							ret = fg_read_word(sm, FG_REG_SRDATA, &data); /* Twice */	
						} else {
							break;
						}
					}
					// 3-3 Recompare Battery Table with dtsi value
					if (sm->battery_table[i][j] == data) {
						pr_info("TABLE rewrite OK [%d][%d] = 0x%x : 0x%x, count=%d\n",
						 i, j, table_reg, data, k);
						break;
					}
					// 3-4 Over I2C_ERROR_COUNT_MAX(5), stopped re-try verify process.
					if (k >= I2C_ERROR_COUNT_MAX){
						pr_err("rewrite %d times, Fail\n", I2C_ERROR_COUNT_MAX);
					}
				}
			}
		}
	}

	/*  RS write */
	ret = fg_write_word(sm, FG_REG_RS, sm->rs);
	if (ret < 0) {
		pr_err("Failed to write RS, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("RS = 0x%x : 0x%x\n",FG_REG_RS, sm->rs);
	}

	/*  alpha write */
	ret = fg_write_word(sm, FG_REG_ALPHA, sm->alpha);
	if (ret < 0) {
		pr_err("Failed to write FG_REG_ALPHA, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("ALPHA = 0x%x : 0x%x\n",FG_REG_ALPHA, sm->alpha);
	}

	/*  beta write */
	ret = fg_write_word(sm, FG_REG_BETA, sm->beta);
	if (ret < 0) {
		pr_err("Failed to write FG_REG_BETA, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("BETA = 0x%x : 0x%x\n",FG_REG_BETA, sm->beta);
	}

	/*  RS write */
	ret = fg_write_word(sm, FG_REG_RS_0, sm->rs_value[0]);
	if (ret < 0) {
		pr_err("Failed to write RS_0, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("RS = 0x%x : 0x%x\n",FG_REG_RS_0, sm->rs_value[0]);
	}

	ret = fg_write_word(sm, FG_REG_RS_1, sm->rs_value[1]);
	if (ret < 0) {
		pr_err("Failed to write RS_1, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("RS_1 = 0x%x : 0x%x\n", FG_REG_RS_1, sm->rs_value[1]);
	}
		
	ret = fg_write_word(sm, FG_REG_RS_2, sm->rs_value[2]);
	if (ret < 0) {
		pr_err("Failed to write RS_2, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("RS_2 = 0x%x : 0x%x\n", FG_REG_RS_2, sm->rs_value[2]);
	}

	ret = fg_write_word(sm, FG_REG_RS_3, sm->rs_value[3]);
	if (ret < 0) {
		pr_err("Failed to write RS_3, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("RS_3 = 0x%x : 0x%x\n", FG_REG_RS_3, sm->rs_value[3]);
	}

	ret = fg_write_word(sm, sm->regs[SM_FG_REG_CURRENT_RATE], sm->mix_value);
	if (ret < 0) {
		pr_err("Failed to write CURRENT_RATE, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("CURRENT_RATE = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_CURRENT_RATE], sm->mix_value);
	}

	pr_info("RS_0 = 0x%x, RS_1 = 0x%x, RS_2 = 0x%x, RS_3 = 0x%x, CURRENT_RATE = 0x%x\n",
		sm->rs_value[0], sm->rs_value[1], sm->rs_value[2], sm->rs_value[3], sm->mix_value);

	/* VOLT_CAL write*/
	ret = fg_write_word(sm, FG_REG_VOLT_CAL, sm->volt_cal);
	if (ret < 0) {
		pr_err("Failed to write FG_REG_VOLT_CAL, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("FG_REG_VOLT_CAL = 0x%x : 0x%x\n", FG_REG_VOLT_CAL, sm->volt_cal);
	}		

	/* CAL write*/	
	ret = fg_write_word(sm, FG_REG_CURR_IN_OFFSET, sm->curr_offset);
	if (ret < 0) {
		pr_err("Failed to write CURR_IN_OFFSET, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("CURR_IN_OFFSET = 0x%x : 0x%x\n", FG_REG_CURR_IN_OFFSET, sm->curr_offset);
	}		
	ret = fg_write_word(sm, FG_REG_CURR_IN_SLOPE, sm->curr_slope);
	if (ret < 0) {
		pr_err("Failed to write CURR_IN_SLOPE, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("CURR_IN_SLOPE = 0x%x : 0x%x\n", FG_REG_CURR_IN_SLOPE, sm->curr_slope);
	}
	/* BAT CAP write */
	ret = fg_write_word(sm, sm->regs[SM_FG_REG_BAT_CAP], sm->cap);
	if (ret < 0) {
		pr_err("Failed to write BAT_CAP, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("BAT_CAP = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_BAT_CAP], sm->cap);
	}

	/* MISC write */
	ret = fg_write_word(sm, sm->regs[SM_FG_REG_MISC], sm->misc[0]);
	if (ret < 0) {
		pr_err("Failed to write REG_MISC, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("REG_MISC 0x%x : 0x%x\n", sm->regs[SM_FG_REG_MISC], sm->misc[0]);
	}

	ret = fg_write_word(sm, FG_REG_MISC_2, sm->misc[1]);
	if (ret < 0) {
		pr_err("Failed to write REG_MISC_2, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("REG_MISC_2 0x%x : 0x%x\n", FG_REG_MISC_2, sm->misc[1]);
	}

	/* TOPOFF SOC */
	ret = fg_write_word(sm, sm->regs[SM_FG_REG_TOPOFFSOC], sm->topoff_soc);	
	if (ret < 0) {
		pr_err("Failed to write SM_FG_REG_TOPOFFSOC, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("SM_REG_TOPOFFSOC 0x%x : 0x%x\n", sm->regs[SM_FG_REG_TOPOFFSOC], sm->topoff_soc);
	}
	
	/*INIT_last -  control register set*/
	ret = fg_read_word(sm, sm->regs[SM_FG_REG_CNTL], &data);
	if (ret < 0) {
			pr_err("Failed to read CNTL, ret = %d\n", ret);
			return ret;
	}
	
	if (sm->iocv_man_mode)
		data |= ENABLE_IOCV_MAN_MODE;

	ret = fg_write_word(sm, sm->regs[SM_FG_REG_CNTL], data);
	if (ret < 0) {
		pr_err("Failed to write CNTL, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("CNTL = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_CNTL], data);
	}

	/* Parameter Version [COMMON(0~3) | BATTERY(0~15)] */
	ret = fg_read_word(sm, FG_PARAM_VERION, &data);
	data = data & RSNS_HR_MASK;
	data = data | ((sm->common_param_version << COMMON_PARAM_SHIFT) & COMMON_PARAM_MASK) | ((sm->battery_param_version << BATTERY_PARAM_SHIFT) & BATTERY_PARAM_MASK);
	ret = fg_write_word(sm, FG_PARAM_VERION, data);
	if (ret < 0) {
		pr_err("Failed to write FG_PARAM_VERION, ret = %d\n", ret);
		return ret;
	}

	if (sm->iocv_man_mode) {
		value = fg_calculate_iocv(sm);

		for (i = 1; i <= I2C_ERROR_COUNT_MAX; i++) {
		    msleep(10);
			ret = fg_write_word(sm, FG_REG_SWADDR, 0x75);
			if (ret < 0) {
				pr_err("Failed to write FG_REG_SWADDR, ret = %d\n", ret);
				return ret;
			}
			ret = fg_write_word(sm, FG_REG_SWDATA, value);
			if (ret < 0) {
				pr_err("Failed to write FG_REG_SWADDR, ret = %d\n", ret);
				return ret;
			}
			ret = fg_write_word(sm, 0x8C, 0x75);
			if (ret < 0) {
				pr_err("Failed to write 0x8C, %d ret = %d\n", 0x75, ret);
				return ret;
			}
			msleep(15); //15mS delay 		
			ret = fg_read_word(sm, 0x8D, &tem_value);
			ret |= fg_read_word(sm, 0x8D, &tem_value);
			if (tem_value & 0x8000) {
				msleep(15); //15mS delay
				ret = fg_read_word(sm, 0x8D, &tem_value);
				ret |= fg_read_word(sm, 0x8D, &tem_value);
			}		
			if (ret < 0) {
				pr_err("Failed to read 0x%x, ret = %d\n",0x8D, ret);
				return ret;
			} else {
				pr_info("IOCV_MAN(tem_value) :  0x%x\n", tem_value);
			}

			if (value == tem_value) {
				pr_info("value = 0x%x : 0x%x, count=%d\n",
				value, tem_value, i);
				break;
			}
			// Over I2C_ERROR_COUNT_MAX(5), stopped re-try verify process.
			if (i >= I2C_ERROR_COUNT_MAX) {
				pr_err("rewrite %d times, Fail\n", I2C_ERROR_COUNT_MAX);
			}
		}
		pr_info("IOCV_MAN : 0x%x\n", value);
	}
	
	msleep(20);

	ret = fg_write_word(sm, sm->regs[SM_FG_REG_PARAM_CTRL], ((FG_PARAM_LOCK_CODE | (sm->battery_table_num & 0x0003) << 6) | (FG_TABLE_LEN-1)));
	if (ret < 0) {
		pr_err("Failed to write param_ctrl lock, ret = %d\n", ret);
		return ret;
	} else {
		pr_info("Param Lock\n");
	}

	msleep(160);

	if (sm->en_temp_ex)
		msleep(150);

	return 1;
}

static unsigned int fg_get_device_id(struct i2c_client *client)
{
	struct sm_fg_chip *sm = i2c_get_clientdata(client);
	int ret;
	u16 data;
	int retry = 0;

	for(retry = 0; retry < 3; retry++) {
		ret = fg_read_word(sm, sm->regs[SM_FG_REG_DEVICE_ID], &data);
		if (ret < 0) {
			pr_err("Failed to read DEVICE_ID, ret = %d\n", ret);
			mdelay(10);
		} else {
			pr_info("revision_id = 0x%x\n",(data & 0x000f));
			pr_info("device_id = 0x%x\n",(data & 0x00f0)>>4);
			break;
		}
	}

	return ret;
}

static unsigned int fg_change_VIO(struct i2c_client *client)
{
	struct sm_fg_chip *sm = i2c_get_clientdata(client);
	int ret = 0;
	u16 data = 0;

	pr_info("%s Start\n",__func__);

	ret = fg_write_word(sm, 0xF1, 0x1234);
	mdelay(10);
	ret |= fg_write_word(sm, 0xF2,0x4321);
	mdelay(10);
	
	ret |= fg_write_word(sm, 0xE4,0x0006);
	ret |= fg_read_word(sm, 0xE5,&data);
	if((data & 0x0080) != 0)
		pr_info("%s mode confirmation fail\n",__func__);

	ret |= fg_write_word(sm, 0xE4,0x0004);
	ret |= fg_read_word(sm, 0xE5,&data);
	if((data & 0x0040) != 0 )
		pr_info("%s : 1.2 mode\n",__func__);
	ret |= fg_write_word(sm, 0xE4,0x0004);
	data = data & 0xFFBF;
	ret |= fg_write_word(sm, 0xE5,data);
	
	ret |= fg_write_word(sm, 0xE4,0x0004);	
	ret |= fg_read_word(sm, 0xE5,&data);
	if((data & 0x0040) == 0)
		pr_info("%s : 1.8 mode\n",__func__);

	ret |= fg_write_word(sm, 0xF0,0x0001);
	mdelay(10);

	pr_info("%s Exit\n",__func__);

	return ret;
}

static bool fg_init(struct i2c_client *client)
{
	int ret;
	struct sm_fg_chip *sm = i2c_get_clientdata(client);
	u16 data, cntl_ref = 0;	


	ret = fg_change_VIO(client);
	if (ret < 0) {
		pr_err("%s: change VIO mode 1.8V(%d)\n", __func__, ret);
		return false;
	}

	/*sm5602 i2c read check*/
	if (fg_check_reg_init_need(client)) {
		fg_reg_init(client);
	} else {
		/*control register check*/
		ret = fg_read_word(sm, sm->regs[SM_FG_REG_CNTL], &data);
		if (ret < 0) {
			pr_err("Failed to read CNTL, ret = %d\n", ret);
			return false;
		}

		if (sm->en_temp_in)
			cntl_ref |= ENABLE_EN_TEMP_IN;
		if (sm->en_temp_ex)
			cntl_ref |= ENABLE_EN_TEMP_EX;
		if (sm->iocv_man_mode)
			cntl_ref |= ENABLE_IOCV_MAN_MODE;

		if (data != cntl_ref) {
			ret = fg_write_word(sm, sm->regs[SM_FG_REG_CNTL], cntl_ref);
			if (ret < 0) {
				return false;
				pr_err("CNTL data is WRONG!!!! but Failed to write CNTL, ret = %d\n", ret);
			} else {
				pr_info("CNTL data is WRONG!!!! rewrite try CNTL = 0x%x : 0x%x\n", sm->regs[SM_FG_REG_CNTL], cntl_ref);
			}
		}		
	}	
	fg_soh_init(sm);

	return true;
}

#define PROPERTY_NAME_SIZE 128
static int fg_common_parse_dt(struct sm_fg_chip *sm)
{
	struct device *dev = &sm->client->dev;
	struct device_node *np = dev->of_node;
    int rc;
   	int misc[2]; 
	BUG_ON(dev == 0);
	BUG_ON(np == 0);

    /* EN TEMP EX/IN */
    if (of_property_read_bool(np, "sm,en_temp_ex"))
        sm->en_temp_ex = true;
	else
		sm->en_temp_ex = 0;
    pr_info("Temperature EX enabled = %d\n", sm->en_temp_ex);

    if (of_property_read_bool(np, "sm,en_temp_in"))
        sm->en_temp_in = true;
	else
		sm->en_temp_in = 0;
    pr_info("Temperature IN enabled = %d\n", sm->en_temp_in);

	if (of_property_read_bool(np, "sm,en_temp_3rd"))
        sm->en_temp_3rd = true;
	else
		sm->en_temp_3rd = 0;
    pr_info("Temperature from 3rd party enabled = %d\n", sm->en_temp_3rd);;

    /* MISC */
    rc = of_property_read_u32_array(np, "sm,misc", misc, 2);
    if (rc < 0)
    {
        sm->misc[0] = 0x0000;
		sm->misc[1] = 0x0000;
    } else {
	    sm->misc[0] = misc[0];
	    sm->misc[1] = misc[1];
	}

	/* IOCV MAN MODE */
	if (of_property_read_bool(np, "sm,iocv_man_mode"))
		sm->iocv_man_mode = true;
	else
		sm->iocv_man_mode = 0;
	pr_info("IOCV_MAN_MODE = %d\n", sm->iocv_man_mode);

    /* Aging */
    rc = of_property_read_u32(np, "sm,aging_ctrl",
                        &sm->aging_ctrl);
    if (rc < 0)
    {
        sm->aging_ctrl = -EINVAL;
    }
	pr_info("aging_ctrl = 0x%x\n", sm->aging_ctrl);

    /* SOC Cycle cfg */
    rc = of_property_read_u32(np, "sm,cycle_cfg",
                        &sm->cycle_cfg);
    if (rc < 0)
    {
        sm->cycle_cfg = -EINVAL;
    }

    /* RSNS */
    rc = of_property_read_u32(np, "sm,rsns",
                        &sm->batt_rsns);
    if (rc < 0)
    {
        sm->batt_rsns = -EINVAL;
    }

    /* Battery Table Number */
    rc = of_property_read_u32(np, "sm,battery_table_num",
                        &sm->battery_table_num);
    if (rc < 0)
    {
        sm->battery_table_num = -EINVAL;
    }

    /* Paramater Number */
    rc = of_property_read_u32(np, "sm,param_version",
                        &sm->common_param_version);
    if (rc < 0)
    {
        sm->common_param_version = -EINVAL;
    }
    pr_info("common_param_version = %d\n", sm->common_param_version);


	/* ENABLE_NTC_COMPENSATION */
	if (of_property_read_bool(np, "sm,enable_ntc_compensation"))
		sm->enable_ntc_compensation = true;
	else
		sm->enable_ntc_compensation = 0;
	pr_info("[%s] ENABLE_NTC_COMPENSATION enabled = %d\n", device2str[sm->chip], sm->enable_ntc_compensation);

	if (sm->enable_ntc_compensation) {
		rc = of_property_read_u32(np, "sm,rtrace",
							&sm->rtrace);
		if (rc < 0)
		{
			sm->rtrace = 0;
		}

		rc = of_property_read_u32(np, "sm,overheat_th_deg",
							&sm->overheat_th_deg);
		if (rc < 0)
		{
			sm->overheat_th_deg = 50;
		}

		rc = of_property_read_u32(np, "sm,cold_th_deg",
							&sm->cold_th_deg);
		if (rc < 0)
		{
			sm->cold_th_deg = 0;
		}		
	}

	/* RPARA */
    rc = of_property_read_u32(np, "sm,rpara",
                        &sm->rpara);
    if (rc < 0)
    {
        sm->rpara = 0;
    }
	pr_info("rpara = %d\n", sm->rpara);

	/* CURR VOFFSET */
    rc = of_property_read_u32(np, "sm,curr_voffset",
                        &sm->curr_voffset);
    if (rc < 0)
    {
        sm->curr_voffset = 0;
    }
	pr_info("curr_voffset = %d\n", sm->curr_voffset);


	/* CURR VSLOPE */
    rc = of_property_read_u32(np, "sm,curr_vslope",
                        &sm->curr_vslope);
    if (rc < 0)
    {
        sm->curr_vslope = 0x8080;
    }
	pr_info("curr_vslope = %d\n", sm->curr_vslope);

    /* ENABLE_IOCV_ADJ */
    if (of_property_read_bool(np, "sm,enable_iocv_adj"))
        sm->enable_iocv_adj = true;
	else
		sm->enable_iocv_adj = 0;
    pr_info("ENABLE_IOCV_ADJ enabled = %d\n", sm->enable_iocv_adj);

	if (sm->enable_iocv_adj) {
	    rc = of_property_read_u32(np, "sm,iocv_max_adj_level",
	                        &sm->iocv_max_adj_level);
	    if (rc < 0)
	    {
	        sm->iocv_max_adj_level = 0;
	    }
	    pr_info("iocv_max_adj_level = %d\n", sm->iocv_max_adj_level);

	    rc = of_property_read_u32(np, "sm,iocv_min_adj_level",
	                        &sm->iocv_min_adj_level);
	    if (rc < 0)
	    {
	        sm->iocv_min_adj_level = 0;
	    }
	    pr_info("iocv_min_adj_level = %d\n", sm->iocv_min_adj_level);

	    rc = of_property_read_u32(np, "sm,ioci_max_adj_level",
	                        &sm->ioci_max_adj_level);
	    if (rc < 0)
	    {
	        sm->ioci_max_adj_level = 0;
	    }
	    pr_info("ioci_max_adj_level = %d\n", sm->ioci_max_adj_level);

	    rc = of_property_read_u32(np, "sm,ioci_min_adj_level",
	                        &sm->ioci_min_adj_level);
	    if (rc < 0)
	    {
	        sm->ioci_min_adj_level = 0;
	    }
	    pr_info("ioci_min_adj_level = %d\n", sm->ioci_min_adj_level);

	    rc = of_property_read_u32(np, "sm,iocv_i_slope",
	                        &sm->iocv_i_slope);
	    if (rc < 0)
	    {
	        sm->iocv_i_slope = 0;
	    }
	    pr_info("iocv_i_slope = %d\n", sm->iocv_i_slope);

	    rc = of_property_read_u32(np, "sm,iocv_i_offset",
	                        &sm->iocv_i_offset);
	    if (rc < 0)
	    {
	        sm->iocv_i_offset = 0;
	    }
	    pr_info("iocv_i_offset = %d\n", sm->iocv_i_offset);
	}

    /* ENABLE_INSPECTION_TABLE */
    if (of_property_read_bool(np, "sm,enable_inspection_table"))
        sm->enable_inspection_table = true;
	else
		sm->enable_inspection_table = 0;
	//sm->enable_inspection_table = 0;
    pr_info("ENABLE_INSPECTION_TABLE enabled = %d\n", sm->enable_inspection_table);

    /* ENABLE_TEMBASE_ZDSCON */
    if (of_property_read_bool(np, "sm,enable_tembase_zdscon"))
        sm->enable_tembase_zdscon = true;
	else
		sm->enable_tembase_zdscon = 0;

    pr_info("ENABLE_TEMBASE_ZDSCON enabled = %d\n", sm->enable_tembase_zdscon);

	if (sm->enable_tembase_zdscon) {
		rc = of_property_read_u32(np, "sm,zdscon_act_temp_gap",
	                        &sm->zdscon_act_temp_gap);
	    if (rc < 0)
	    {
	        sm->zdscon_act_temp_gap = 0;
	    }
	    pr_info("zdscon_act_temp_gap = %d\n", sm->zdscon_act_temp_gap);

	    rc = of_property_read_u32(np, "sm,t_gap_denom",
	                        &sm->t_gap_denom);
	    if (rc < 0)
	    {
	        sm->t_gap_denom = 0;
	    }
	    pr_info("t_gap_denom = %d\n", sm->t_gap_denom);

	    rc = of_property_read_u32(np, "sm,hminman_t_value_fast",
                    &sm->hminman_t_value_fast);
	    if (rc < 0)
	    {
	        sm->hminman_t_value_fast = 0;
	    }
	    pr_info("hminman_t_value_fast = %d\n", sm->hminman_t_value_fast);

	    rc = of_property_read_u32(np, "sm,i_gap_denom",
                    &sm->i_gap_denom);
	    if (rc < 0)
	    {
	        sm->i_gap_denom = 0;
	    }
	    pr_info("i_gap_denom = %d\n", sm->i_gap_denom);

	    rc = of_property_read_u32(np, "sm,hminman_i_value_fast",
                    &sm->hminman_i_value_fast);
	    if (rc < 0)
	    {
	        sm->hminman_i_value_fast = 0;
	    }
	    pr_info("hminman_i_value_fast = %d\n", sm->hminman_i_value_fast);		
	}

    /* ENABLE_TEMP_AVG */
    if (of_property_read_bool(np, "sm,enable_temp_avg"))
        sm->enable_temp_avg = true;
	else
		sm->enable_temp_avg = 0;
    pr_info("ENABLE_TEMP_AVG enabled = %d\n", sm->enable_temp_avg);

	if (sm->enable_temp_avg) {
	    rc = of_property_read_u32(np, "sm,change_temp_time_limit",
	                        &sm->change_temp_time_limit);
	    if (rc < 0)
	    {
	        sm->change_temp_time_limit = 1;
	    }
	    pr_info("batt_temp_avg_samples = %d\n", sm->change_temp_time_limit);		
	}

	/* ENABLE_LTIM_ACT */
	if (of_property_read_bool(np, "sm,enable_ltim_act"))
		sm->enable_ltim_act = true;
	else
		sm->enable_ltim_act = 0;
	pr_info("ENABLE_LTIM_ACT enabled = %d\n", sm->enable_ltim_act);

	if (sm->enable_ltim_act) {
		rc = of_property_read_u32(np, "sm,ltim_act_temp_gap",
							&sm->ltim_act_temp_gap);
		if (rc < 0)
		{
			sm->ltim_act_temp_gap = 24;
		}
		pr_info("ltim_act_temp_gap = %d\n", sm->ltim_act_temp_gap);	

		rc = of_property_read_u32(np, "sm,ltim_i_limit",
							&sm->ltim_i_limit);
		if (rc < 0)
		{
			sm->ltim_i_limit = 1500;
		}
		pr_info("ltim_i_limit = %d\n", sm->ltim_i_limit);

		rc = of_property_read_u32(np, "sm,ltim_factor",
							&sm->ltim_factor);
		if (rc < 0)
		{
			sm->ltim_factor = 42;
		}
		pr_info("ltim_factor = %d\n", sm->ltim_factor);	

		rc = of_property_read_u32(np, "sm,ltim_denom",
							&sm->ltim_denom);
		if (rc < 0)
		{
			sm->ltim_denom = 130;
		}
		pr_info("ltim_denom = %d\n", sm->ltim_denom);

		rc = of_property_read_u32(np, "sm,ltim_min",
							&sm->ltim_min);
		if (rc < 0)
		{
			sm->ltim_min = 0xA;
		}
		pr_info("ltim_min = %d\n", sm->ltim_min);				
	}

	/* ENABLE_HCIC_ACT */
	if (of_property_read_bool(np, "sm,enable_hcic_act"))
		sm->enable_hcic_act = true;
	else
		sm->enable_hcic_act = 0;
	pr_info("ENABLE_HCIC_ACT enabled = %d\n", sm->enable_hcic_act);

	if (sm->enable_hcic_act) {
		rc = of_property_read_u32(np, "sm,hcic_min",
							&sm->hcic_min);
		if (rc < 0)
		{
			sm->hcic_min = 4000;
		}
		pr_info("hcic_min = %d\n", sm->hcic_min);	

		rc = of_property_read_u32(np, "sm,hcic_factor",
							&sm->hcic_factor);
		if (rc < 0)
		{
			sm->hcic_factor = 2;
		}
		pr_info("[hcic_factor = %d\n", sm->hcic_factor);

		rc = of_property_read_u32(np, "sm,hcic_denom",
							&sm->hcic_denom);
		if (rc < 0)
		{
			sm->hcic_denom = 200;
		}
		pr_info("hcic_denom = %d\n", sm->hcic_denom);	
	}

	/* ENABLE_HCRSM_MODE */
	if (of_property_read_bool(np, "sm,enable_hcrsm_mode"))
		sm->enable_hcrsm_mode = true;
	else
		sm->enable_hcrsm_mode = 0;
	pr_info("ENABLE_HCRSM_MODE enabled = %d\n", sm->enable_hcrsm_mode);

	if (sm->enable_hcrsm_mode) {
		rc = of_property_read_u32(np, "sm,hcrsm_volt",
							&sm->hcrsm_volt);
		if (rc < 0)
		{
			sm->hcrsm_volt = 4450;
		}
		pr_info("hcrsm_volt = %d\n", sm->hcrsm_volt);	

		rc = of_property_read_u32(np, "sm,hcrsm_curr",
							&sm->hcrsm_curr);
		if (rc < 0)
		{
			sm->hcrsm_curr = 1750;
		}
		pr_info("hcrsm_curr = %d\n", sm->hcrsm_curr);

		rc = of_property_read_u32(np, "sm,hcrsm_min_soc",
							&sm->hcrsm_min_soc);
		if (rc < 0)
		{
			sm->hcrsm_min_soc = 900;
		}
		pr_info("hcrsm_min_soc = %d\n", sm->hcrsm_min_soc);	
	}

	/* CHECK_ABNORMAL_TABLE */
	if (of_property_read_bool(np, "sm,enable_check_abnormal_table"))
		sm->enable_check_abnormal_table = true;
	else
		sm->enable_hcrsm_mode = 0;
	pr_info("CHECK_ABNORMAL_TABLE enabled = %d\n", sm->enable_check_abnormal_table);

	return 0;
}

static int get_battery_id(struct sm_fg_chip *sm)
{
	return 0;
}

static int fg_battery_parse_dt(struct sm_fg_chip *sm)
{
	struct device *dev = &sm->client->dev;
	struct device_node *np = dev->of_node;
	char prop_name[PROPERTY_NAME_SIZE];
	int battery_id = -1;
	int battery_temp_table[FG_TEMP_TABLE_CNT_MAX];
	int table[FG_TABLE_LEN];
	int rs_value[4];
	int topoff_soc[3];
	int temp_offset[6];
	int temp_cal[10];
	int ext_temp_cal[10];
	int battery_type[3];
	int set_temp_poff[4];
	int ret;
	int i, j;
	int static_scale_soc[6];

	BUG_ON(dev == 0);
	BUG_ON(np == 0);

	/* battery_params node*/
	np = of_find_node_by_name(of_node_get(np), "battery_params");
	if (np == NULL) {
		pr_info("Cannot find child node \"battery_params\"\n");
		return -EINVAL;
	}

	/* battery_id*/
	if (of_property_read_u32(np, "battery,id", &battery_id) < 0)
		pr_err("not battery,id property\n");
	if (battery_id == -1)
		battery_id = get_battery_id(sm);
	pr_info("battery id = %d\n", battery_id);

	/*  battery_table*/
	for (i = BATTERY_TABLE0; i < BATTERY_TABLE2; i++) {
		snprintf(prop_name, PROPERTY_NAME_SIZE,
			 "battery%d,%s%d", battery_id, "battery_table", i);

		ret = of_property_read_u32_array(np, prop_name, table, FG_TABLE_LEN);
		if (ret < 0)
			pr_info("Can get prop %s (%d)\n", prop_name, ret);
		for (j = 0; j < FG_TABLE_LEN; j++) {
			sm->battery_table[i][j] = table[j];
			pr_info("%s = <table[%d][%d] 0x%x>\n",
				prop_name, i, j, table[j]);
		}
	}

	i = BATTERY_TABLE2;
	snprintf(prop_name, PROPERTY_NAME_SIZE,
		 "battery%d,%s%d", battery_id, "battery_table", i);
	ret = of_property_read_u32_array(np, prop_name, table, FG_ADD_TABLE_LEN);
	if (ret < 0)
		pr_info("Can get prop %s (%d)\n", prop_name, ret);
	else {
		for(j=0; j < FG_ADD_TABLE_LEN; j++)
		{
			sm->battery_table[i][j] = table[j];
			pr_info("%s = <table[%d][%d] 0x%x>\n",
				prop_name, i, j, table[j]);
		
		}
	}
	
    /* rs */
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "rs");
	ret = of_property_read_u32_array(np, prop_name, &sm->rs, 1);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	pr_info("%s = <0x%x>\n", prop_name, sm->rs);

    /* alpha */
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "alpha");
	ret = of_property_read_u32_array(np, prop_name, &sm->alpha, 1);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	pr_info("%s = <0x%x>\n", prop_name, sm->alpha);

    /* beta */
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "beta");
	ret = of_property_read_u32_array(np, prop_name, &sm->beta, 1);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	pr_info("%s = <0x%x>\n", prop_name, sm->beta);

	/* rs_value*/
	for (i = 0; i < 4; i++) {
		snprintf(prop_name,
			PROPERTY_NAME_SIZE, "battery%d,%s",
			battery_id, "rs_value");
		ret = of_property_read_u32_array(np, prop_name, rs_value, 4);
		if (ret < 0)
			pr_err("Can get prop %s (%d)\n", prop_name, ret);
		sm->rs_value[i] = rs_value[i];
	}
	pr_info("%s = <0x%x 0x%x 0x%x 0x%x>\n",
		prop_name, rs_value[0], rs_value[1], rs_value[2], rs_value[3]);

	/* vit_period*/
	snprintf(prop_name,
		PROPERTY_NAME_SIZE, "battery%d,%s",
		battery_id, "vit_period");
	ret = of_property_read_u32_array(np,
		prop_name, &sm->vit_period, 1);
	if (ret < 0)
		pr_info("Can get prop %s (%d)\n", prop_name, ret);
	pr_info("%s = <0x%x>\n", prop_name, sm->vit_period);

    /* battery_type*/
    snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "battery_type");
    ret = of_property_read_u32_array(np, prop_name, battery_type, 3);
    if (ret < 0)
        pr_err("Can get prop %s (%d)\n", prop_name, ret);
    sm->batt_v_max = battery_type[0];
    sm->min_cap = battery_type[1];
    sm->cap = battery_type[2];

    pr_info("%s = <%d %d %d>\n", prop_name,
        sm->batt_v_max, sm->min_cap, sm->cap);

	/* tem poff level */
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "tem_poff");
	ret = of_property_read_u32_array(np, prop_name, set_temp_poff, 2);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	sm->n_tem_poff = set_temp_poff[0];
	sm->n_tem_poff_offset = set_temp_poff[1];

	pr_info("%s = <%d, %d>\n",
		prop_name,
		sm->n_tem_poff, sm->n_tem_poff_offset);

    /* max-voltage -mv*/
    snprintf(prop_name,
		PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "max_voltage_uv");    
	ret = of_property_read_u32(np, prop_name,
				        &sm->batt_max_voltage_uv);
	if (ret < 0)
	    pr_err("couldn't find battery max voltage\n");        

    // TOPOFF SOC
    snprintf(prop_name, 
    	PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "topoff_soc");
    ret = of_property_read_u32_array(np, prop_name, topoff_soc, 3);
    if (ret < 0)
        pr_err("Can get prop %s (%d)\n", prop_name, ret);
    sm->topoff_soc = topoff_soc[0];
    sm->top_off = topoff_soc[1];
	sm->topoff_margin = topoff_soc[2];

    pr_info("%s = <%d %d %d>\n", prop_name,
        sm->topoff_soc, sm->top_off, sm->topoff_margin);

    // Mix
    snprintf(prop_name, 
    	PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "mix_value");
    ret = of_property_read_u32_array(np, prop_name, &sm->mix_value, 1);
    if (ret < 0)
        pr_err("Can get prop %s (%d)\n", prop_name, ret);

    pr_info("%s = <%d>\n", prop_name,
        sm->mix_value);

    /* VOLT CAL */
    snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "volt_cal");
    ret = of_property_read_u32_array(np, prop_name, &sm->volt_cal, 1);
    if (ret < 0)
        pr_err("Can get prop %s (%d)\n", prop_name, ret);
    pr_info("%s = <0x%x>\n", prop_name, sm->volt_cal);

	/* CURR OFFSET */
	snprintf(prop_name,
		PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "curr_offset");
	ret = of_property_read_u32_array(np,
		prop_name, &sm->curr_offset, 1);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	pr_info("%s = <0x%x>\n", prop_name, sm->curr_offset);

	/* CURR SLOPE */
	snprintf(prop_name,
		PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "curr_slope");
	ret = of_property_read_u32_array(np,
		prop_name, &sm->curr_slope, 1);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	pr_info("%s = <0x%x>\n", prop_name, sm->curr_slope);

	/* temp_std */
	snprintf(prop_name, 
		PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "temp_std");
	ret = of_property_read_u32_array(np, prop_name, &sm->temp_std, 1);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	pr_info("%s = <%d>\n", prop_name, sm->temp_std);

	/* temp_offset */
	snprintf(prop_name, 
		PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "temp_offset");
	ret = of_property_read_u32_array(np, prop_name, temp_offset, 6);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	sm->en_high_fg_temp_offset = temp_offset[0];
	sm->high_fg_temp_offset_denom = temp_offset[1];
	sm->high_fg_temp_offset_fact = temp_offset[2];
	sm->en_low_fg_temp_offset = temp_offset[3];
	sm->low_fg_temp_offset_denom = temp_offset[4];
	sm->low_fg_temp_offset_fact = temp_offset[5];
	pr_info("%s = <%d, %d, %d, %d, %d, %d>\n", prop_name,
		sm->en_high_fg_temp_offset, 
		sm->high_fg_temp_offset_denom, sm->high_fg_temp_offset_fact,
		sm->en_low_fg_temp_offset,
		sm->low_fg_temp_offset_denom, sm->low_fg_temp_offset_fact);

	/* temp_calc */
	snprintf(prop_name, 
		PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "temp_cal");
	ret = of_property_read_u32_array(np, prop_name, temp_cal, 10);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	sm->en_high_fg_temp_cal = temp_cal[0];
	sm->high_fg_temp_p_cal_denom = temp_cal[1];
	sm->high_fg_temp_p_cal_fact = temp_cal[2];
	sm->high_fg_temp_n_cal_denom = temp_cal[3];
	sm->high_fg_temp_n_cal_fact = temp_cal[4];
	sm->en_low_fg_temp_cal = temp_cal[5];
	sm->low_fg_temp_p_cal_denom = temp_cal[6];
	sm->low_fg_temp_p_cal_fact = temp_cal[7];
	sm->low_fg_temp_n_cal_denom = temp_cal[8];
	sm->low_fg_temp_n_cal_fact = temp_cal[9];
	pr_info("%s = <%d, %d, %d, %d, %d, %d, %d, %d, %d, %d>\n", prop_name,
		sm->en_high_fg_temp_cal, 
		sm->high_fg_temp_p_cal_denom, sm->high_fg_temp_p_cal_fact,
		sm->high_fg_temp_n_cal_denom, sm->high_fg_temp_n_cal_fact,
		sm->en_low_fg_temp_cal,
		sm->low_fg_temp_p_cal_denom, sm->low_fg_temp_p_cal_fact,
		sm->low_fg_temp_n_cal_denom, sm->low_fg_temp_n_cal_fact);

	/* ext_temp_calc */
	snprintf(prop_name, 
		PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "ext_temp_cal");
	ret = of_property_read_u32_array(np, prop_name, ext_temp_cal, 10);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	sm->en_high_temp_cal = ext_temp_cal[0];
	sm->high_temp_p_cal_denom = ext_temp_cal[1];
	sm->high_temp_p_cal_fact = ext_temp_cal[2];
	sm->high_temp_n_cal_denom = ext_temp_cal[3];
	sm->high_temp_n_cal_fact = ext_temp_cal[4];
	sm->en_low_temp_cal = ext_temp_cal[5];
	sm->low_temp_p_cal_denom = ext_temp_cal[6];
	sm->low_temp_p_cal_fact = ext_temp_cal[7];
	sm->low_temp_n_cal_denom = ext_temp_cal[8];
	sm->low_temp_n_cal_fact = ext_temp_cal[9];
	pr_info("%s = <%d, %d, %d, %d, %d, %d, %d, %d, %d, %d>\n", prop_name,
		sm->en_high_temp_cal, 
		sm->high_temp_p_cal_denom, sm->high_temp_p_cal_fact,
		sm->high_temp_n_cal_denom, sm->high_temp_n_cal_fact,
		sm->en_low_temp_cal,
		sm->low_temp_p_cal_denom, sm->low_temp_p_cal_fact,
		sm->low_temp_n_cal_denom, sm->low_temp_n_cal_fact);

	snprintf(prop_name, 
		PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "static_scale_soc");
	ret = of_property_read_u32_array(np, prop_name, static_scale_soc, 6);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	sm->en_static_scale_soc = static_scale_soc[0];
	sm->static_scale_soc_base_min = static_scale_soc[1];
	sm->static_scale_soc_base_max = static_scale_soc[2];
	sm->off_current_min = static_scale_soc[3];
	sm->off_current_max = static_scale_soc[4];
	sm->rsoc_delta_scale_limit = static_scale_soc[5];
	pr_info("%s = <%d, %d, %d, %d, %d, %d>\n", prop_name,
		sm->en_static_scale_soc, 
		sm->static_scale_soc_base_min, sm->static_scale_soc_base_max,
		sm->off_current_min, sm->off_current_max,
		sm->rsoc_delta_scale_limit);

	/* get battery_temp_table*/
	 snprintf(prop_name, PROPERTY_NAME_SIZE,
		  "battery%d,%s", battery_id, "thermal_table");
	
	 ret = of_property_read_u32_array(np, prop_name, battery_temp_table, FG_TEMP_TABLE_CNT_MAX);
	 if (ret < 0)
		 pr_err("Can get prop %s (%d)\n", prop_name, ret);
	 for (i = 0; i < FG_TEMP_TABLE_CNT_MAX; i++) {
		 sm->battery_temp_table[i] = battery_temp_table[i];
		 pr_err("%s = <battery_temp_table[%d] 0x%x>\n",
			 prop_name, i,	battery_temp_table[i]);
	 }

    /* Battery Paramter */
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "param_version");
	ret = of_property_read_u32_array(np, prop_name, &sm->battery_param_version, 1);
	if (ret < 0)
		pr_err("Can get prop %s (%d)\n", prop_name, ret);
	pr_info("%s = <0x%x>\n", prop_name, sm->battery_param_version);

	return 0;
}

bool hal_fg_init(struct i2c_client *client)
{
	int ret;
	struct sm_fg_chip *sm = i2c_get_clientdata(client);

	pr_info("sm5602g hal_fg_init...\n");
	ret = fg_get_device_id(client);
	if (ret < 0) {
		pr_err("%s: fail to do i2c read(%d)\n", __func__, ret);
		return false;
	}
	strncpy(sm->device_name, "SM5602G", SM_NAME_LEN);
	sm->device_type = ret;
	mutex_lock(&sm->data_lock);
	if (client->dev.of_node) {
		/* Load common data from DTS*/
		fg_common_parse_dt(sm);
		/* Load battery data from DTS*/
		fg_battery_parse_dt(sm);
	}
	if(!fg_init(client))
        return false;

	mutex_unlock(&sm->data_lock);
	pr_info("hal fg init OK\n");
	return true;
}

static int sm_fg_probe(struct i2c_client *client,
							const struct i2c_device_id *id)
{

	int ret = -1;
	struct sm_fg_chip *sm;
	struct oplus_gauge_chip	*chip_gauge = NULL;
	u8 *regs;

	pr_info("enter\n");
	if (!oplus_gauge_check_chip_is_null()) {
		chg_err("gauge chip_is not null, skip %s\n", __func__);
		return -ENOMEM;
	}
	sm = devm_kzalloc(&client->dev, sizeof(*sm), GFP_KERNEL);

	if (!sm)
		return -ENOMEM;

	sm->dev = &client->dev;
	sm->client = client;
	sm->chip = id->driver_data;

	sm->batt_soc	= -ENODATA;
	sm->batt_fcc	= -ENODATA;
	//sm->batt_dc		= -ENODATA;
	sm->batt_volt	= -ENODATA;
	sm->batt_temp	= -ENODATA;
	sm->batt_curr	= -ENODATA;
	sm->fake_soc	= -EINVAL;
	sm->fake_temp	= -EINVAL;

	if (sm->chip == SM5602G) {
		regs = sm5602g_regs;
	} else {
		pr_err("unexpected fuel gauge: %d\n", sm->chip);
		regs = sm5602g_regs;
	}

	memcpy(sm->regs, regs, NUM_REGS);

	i2c_set_clientdata(client, sm);

	mutex_init(&sm->i2c_rw_lock);
	mutex_init(&sm->data_lock);

	if (hal_fg_init(client) == false) {
	    pr_err("Failed to Initialize Fuelgauge\n");
        goto err_0; 	
	}

	sm->temp_param.batt_temp = -EINVAL;
	sm->temp_param.update_now = true;

	sm->last_rsoc = 0;
	sm->static_scale_soc_base_value = sm->static_scale_soc_base_max;
	sm->static_scale_soc_rise_count = 0;

	INIT_DELAYED_WORK(&sm->monitor_work, fg_monitor_workfunc);	

	create_debugfs_entry(sm);

	fg_dump_debug(sm);

	schedule_delayed_work(&sm->monitor_work, msecs_to_jiffies(10000)); /* 10 sec */

	smifg_ic = sm;

	chip_gauge = devm_kzalloc(&client->dev,
		sizeof(struct oplus_gauge_chip), GFP_KERNEL);

	if (!chip_gauge) {
		chg_err("sm5602 chip_fuel_gauge null %d\n", __LINE__);
		return -ENOMEM;
	} else {
		chg_err("sm5602 chip_fuel_gauge success %d\n", __LINE__);
	}

	oplus_gauge_init(chip_gauge);

	chip_gauge->client = client;
	chip_gauge->dev = &client->dev;
	chip_gauge->gauge_ops = &sm5602_gauge_ops;
	chip_gauge->device_type = 2;/*temp*/
	chip_gauge->device_type = smifg_ic->device_type;
	chip_gauge->device_name = smifg_ic->device_name;

	pr_info("sm fuel gauge probe successfully, %s\n",device2str[sm->chip]);

	return 0;
err_0:	
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void sm_fg_remove(struct i2c_client *client)
#else
static int sm_fg_remove(struct i2c_client *client)
#endif
{
	struct sm_fg_chip *sm = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&sm->monitor_work);
	
	pr_info("mode change to shutdown mode in fg_remove\n");
	fg_write_word(sm, FG_REG_RS_2, sm->rs_value[2]);
	mutex_destroy(&sm->data_lock);
	mutex_destroy(&sm->i2c_rw_lock);

	debugfs_remove_recursive(sm->debug_root);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
		return 0;
#endif
}

static void sm_fg_shutdown(struct i2c_client *client)
{
	struct sm_fg_chip *sm = i2c_get_clientdata(client);

	if (sm) {
		pr_info("mode change to shutdown mode in fg_shutdown\n");
		fg_write_word(sm, FG_REG_RS_2, sm->rs_value[2]);
	}

	pr_info("sm fuel gauge driver shutdown!\n");
}

static const struct of_device_id sm_fg_match_table[] = {
	{.compatible = "sm,sm5602g",},
	{},
};
MODULE_DEVICE_TABLE(of, sm_fg_match_table);

static const struct i2c_device_id sm_fg_id[] = {
	{ "sm5602g", SM5602G },
	{},
};
MODULE_DEVICE_TABLE(i2c, sm_fg_id);

static struct i2c_driver sm_fg_driver = {
	.driver		= {
		.name		= "sm5602g",
		.owner		= THIS_MODULE,
		.of_match_table	= sm_fg_match_table,
	},
	.id_table   = sm_fg_id,
	.probe		= sm_fg_probe,
	.remove		= sm_fg_remove,
	.shutdown     = sm_fg_shutdown,
};


#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
static void __init sm5602_driver_init(void)
{
	chg_err("\n");
	i2c_add_driver(&sm_fg_driver);
}

static void __exit sm5602_driver_exit(void)
{
	i2c_del_driver(&sm_fg_driver);
}
module_init(sm5602_driver_init);
module_exit(sm5602_driver_exit);
#else
int sm5602_driver_init(void)
{
	chg_err("%s...", __func__);

	if (0 != i2c_add_driver(&sm_fg_driver)) {
		chg_debug("failed to register sm5602 i2c driver.\n");
	} else {
		chg_debug("succeed to register sm5602 i2c driver.\n");
	}
	return 0;
}

void sm5602_driver_exit(void)
{
	i2c_del_driver(&sm_fg_driver);
}
#endif


MODULE_DESCRIPTION("SM SM5602G Gauge Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Siliconmitus");

