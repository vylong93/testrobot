/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       inv_mpu.h
 *      @brief      An I2C-based driver for Invensense gyroscopes.
 *      @details    This driver currently works for the following devices:
 *                  MPU6050
 *                  MPU6500
 *                  MPU9150 (or MPU6050 w/ AK8975 on the auxiliary bus)
 *                  MPU9250 (or MPU6500 w/ AK8963 on the auxiliary bus)
 */

#ifndef _INV_MPU_H_
#define _INV_MPU_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "eMPL\inc\dmpKey.h"
#include "eMPL\inc\dmpmap.h"

/* The following functions must be defined for this platform:
 * i2c_write(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char const *data)
 * i2c_read(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char *data)
 * delay_ms(unsigned long num_ms)
 * get_ms(unsigned long *count)
 * reg_int_cb(void (*cb)(void), unsigned char port, unsigned char pin)
 * labs(long x)
 * fabsf(float x)
 * min(int a, int b)
 */
#include "libcustom\inc\custom_i2c.h"
#include "libcustom\inc\custom_uart_debug.h"

#define i2c_write   i2c_write
#define i2c_read	i2c_read
#define delay_ms	delay_ms
#define get_ms     	get_ms

#define fabs        fabsf
#define min(a,b) ((a<b)?a:b)


#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)

struct int_param_s {
    void (*cb)(void);
    unsigned short pin;
    unsigned char lp_exit;
    unsigned char active_low;
};

#define MPU_INT_STATUS_DATA_READY       (0x0001)
#define MPU_INT_STATUS_DMP              (0x0002)
#define MPU_INT_STATUS_PLL_READY        (0x0004)
#define MPU_INT_STATUS_I2C_MST          (0x0008)
#define MPU_INT_STATUS_FIFO_OVERFLOW    (0x0010)
#define MPU_INT_STATUS_ZMOT             (0x0020)
#define MPU_INT_STATUS_MOT              (0x0040)
#define MPU_INT_STATUS_FREE_FALL        (0x0080)
#define MPU_INT_STATUS_DMP_0            (0x0100)
#define MPU_INT_STATUS_DMP_1            (0x0200)
#define MPU_INT_STATUS_DMP_2            (0x0400)
#define MPU_INT_STATUS_DMP_3            (0x0800)
#define MPU_INT_STATUS_DMP_4            (0x1000)
#define MPU_INT_STATUS_DMP_5            (0x2000)

/* Hardware registers needed by driver. */
struct gyro_reg_s {
    unsigned char who_am_i;
    unsigned char rate_div;
    unsigned char lpf;
    unsigned char prod_id;
    unsigned char user_ctrl;
    unsigned char fifo_en;
    unsigned char gyro_cfg;
    unsigned char accel_cfg;
    unsigned char accel_cfg2;
    unsigned char lp_accel_odr;
    unsigned char motion_thr;
    unsigned char motion_dur;
    unsigned char fifo_count_h;
    unsigned char fifo_r_w;
    unsigned char raw_gyro;
    unsigned char raw_accel;
    unsigned char temp;
    unsigned char int_enable;
    unsigned char dmp_int_status;
    unsigned char int_status;
    unsigned char accel_intel;
    unsigned char pwr_mgmt_1;
    unsigned char pwr_mgmt_2;
    unsigned char int_pin_cfg;
    unsigned char mem_r_w;
    unsigned char accel_offs;
    unsigned char i2c_mst;
    unsigned char bank_sel;
    unsigned char mem_start_addr;
    unsigned char prgm_start_h;
};

/* Information specific to a particular device. */
struct hw_s {
    unsigned char addr;
    unsigned short max_fifo;
    unsigned char num_reg;
    unsigned short temp_sens;
    short temp_offset;
    unsigned short bank_size;
};

/* When entering motion interrupt mode, the driver keeps track of the
 * previous state so that it can be restored at a later time.
 * TODO: This is tacky. Fix it.
 */
struct motion_int_cache_s {
    unsigned short gyro_fsr;
    unsigned char accel_fsr;
    unsigned short lpf;
    unsigned short sample_rate;
    unsigned char sensors_on;
    unsigned char fifo_sensors;
    unsigned char dmp_on;
};

/* Cached chip configuration data.
 * TODO: A lot of these can be handled with a bitmask.
 */
struct chip_cfg_s {
    /* Matches gyro_cfg >> 3 & 0x03 */
    unsigned char gyro_fsr;
    /* Matches accel_cfg >> 3 & 0x03 */
    unsigned char accel_fsr;
    /* Enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2. */
    unsigned char sensors;
    /* Matches config register. */
    unsigned char lpf;
    unsigned char clk_src;
    /* Sample rate, NOT rate divider. */
    unsigned short sample_rate;
    /* Matches fifo_en register. */
    unsigned char fifo_enable;
    /* Matches int enable register. */
    unsigned char int_enable;
    /* 1 if devices on auxiliary I2C bus appear on the primary. */
    unsigned char bypass_mode;
    /* 1 if half-sensitivity.
     * NOTE: This doesn't belong here, but everything else in hw_s is const,
     * and this allows us to save some precious RAM.
     */
    unsigned char accel_half;
    /* 1 if device in low-power accel-only mode. */
    unsigned char lp_accel_mode;
    /* 1 if interrupts are only triggered on motion events. */
    unsigned char int_motion_only;
    struct motion_int_cache_s cache;
    /* 1 for active low interrupts. */
    unsigned char active_low_int;
    /* 1 for latched interrupts. */
    unsigned char latched_int;
    /* 1 if DMP is enabled. */
    unsigned char dmp_on;
    /* Ensures that DMP will only be loaded once. */
    unsigned char dmp_loaded;
    /* Sampling rate used when DMP is enabled. */
    unsigned short dmp_sample_rate;
};

/* Information for self-test. */
struct test_s {
    unsigned long gyro_sens;
    unsigned long accel_sens;
    unsigned char reg_rate_div;
    unsigned char reg_lpf;
    unsigned char reg_gyro_fsr;
    unsigned char reg_accel_fsr;
    unsigned short wait_ms;
    unsigned char packet_thresh;
    float min_dps;
    float max_dps;
    float max_gyro_var;
    float min_g;
    float max_g;
    float max_accel_var;
};

/* Gyro driver state variables. */
struct gyro_state_s {
    const struct gyro_reg_s *reg;
    const struct hw_s *hw;
    struct chip_cfg_s chip_cfg;
    const struct test_s *test;
};

/* Filter configurations. */
enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ,
    INV_FILTER_98HZ,
    INV_FILTER_42HZ,
    INV_FILTER_20HZ,
    INV_FILTER_10HZ,
    INV_FILTER_5HZ,
    INV_FILTER_2100HZ_NOLPF,
    NUM_FILTER
};

/* Full scale ranges. */
enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};

/* Full scale ranges. */
enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};

/* Clock sources. */
enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};

/* Low-power accel wakeup rates. */
enum lp_accel_rate_e {
    INV_LPA_1_25HZ,
    INV_LPA_5HZ,
    INV_LPA_20HZ,
    INV_LPA_40HZ
};

#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_S0_DELAY_EN     (0x01)
#define BIT_S2_DELAY_EN     (0x04)
#define BITS_SLAVE_LENGTH   (0x0F)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BITS_I2C_MASTER_DLY (0x1F)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BITS_WOM_EN         (0xC0)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

#define MAX_PACKET_LENGTH 	(12)

#ifdef USE_DMP
#define TAP_X               (0x01)
#define TAP_Y               (0x02)
#define TAP_Z               (0x04)
#define TAP_XYZ             (0x07)

#define TAP_X_UP            (0x01)
#define TAP_X_DOWN          (0x02)
#define TAP_Y_UP            (0x03)
#define TAP_Y_DOWN          (0x04)
#define TAP_Z_UP            (0x05)
#define TAP_Z_DOWN          (0x06)

#define ANDROID_ORIENT_PORTRAIT             (0x00)
#define ANDROID_ORIENT_LANDSCAPE            (0x01)
#define ANDROID_ORIENT_REVERSE_PORTRAIT     (0x02)
#define ANDROID_ORIENT_REVERSE_LANDSCAPE    (0x03)

#define DMP_INT_GESTURE     (0x01)
#define DMP_INT_CONTINUOUS  (0x02)

#define DMP_FEATURE_TAP             (0x001)
#define DMP_FEATURE_ANDROID_ORIENT  (0x002)
#define DMP_FEATURE_LP_QUAT         (0x004)
#define DMP_FEATURE_PEDOMETER       (0x008)
#define DMP_FEATURE_6X_LP_QUAT      (0x010)
#define DMP_FEATURE_GYRO_CAL        (0x020)
#define DMP_FEATURE_SEND_RAW_ACCEL  (0x040)
#define DMP_FEATURE_SEND_RAW_GYRO   (0x080)
#define DMP_FEATURE_SEND_CAL_GYRO   (0x100)

#define INV_WXYZ_QUAT       (0x100)

/* These defines are copied from dmpDefaultMPU6050.c in the general MPL
 * releases. These defines may change for each DMP image, so be sure to modify
 * these values when switching to a new image.
 */
#define CFG_LP_QUAT             (2712)
#define END_ORIENT_TEMP         (1866)
#define CFG_27                  (2742)
#define CFG_20                  (2224)
#define CFG_23                  (2745)
#define CFG_FIFO_ON_EVENT       (2690)
#define END_PREDICTION_UPDATE   (1761)
#define CGNOTICE_INTR           (2620)
#define X_GRT_Y_TMP             (1358)
#define CFG_DR_INT              (1029)
#define CFG_AUTH                (1035)
#define UPDATE_PROP_ROT         (1835)
#define END_COMPARE_Y_X_TMP2    (1455)
#define SKIP_X_GRT_Y_TMP        (1359)
#define SKIP_END_COMPARE        (1435)
#define FCFG_3                  (1088)
#define FCFG_2                  (1066)
#define FCFG_1                  (1062)
#define END_COMPARE_Y_X_TMP3    (1434)
#define FCFG_7                  (1073)
#define FCFG_6                  (1106)
#define FLAT_STATE_END          (1713)
#define SWING_END_4             (1616)
#define SWING_END_2             (1565)
#define SWING_END_3             (1587)
#define SWING_END_1             (1550)
#define CFG_8                   (2718)
#define CFG_15                  (2727)
#define CFG_16                  (2746)
#define CFG_EXT_GYRO_BIAS       (1189)
#define END_COMPARE_Y_X_TMP     (1407)
#define DO_NOT_UPDATE_PROP_ROT  (1839)
#define CFG_7                   (1205)
#define FLAT_STATE_END_TEMP     (1683)
#define END_COMPARE_Y_X         (1484)
#define SKIP_SWING_END_1        (1551)
#define SKIP_SWING_END_3        (1588)
#define SKIP_SWING_END_2        (1566)
#define TILTG75_START           (1672)
#define CFG_6                   (2753)
#define TILTL75_END             (1669)
#define END_ORIENT              (1884)
#define CFG_FLICK_IN            (2573)
#define TILTL75_START           (1643)
#define CFG_MOTION_BIAS         (1208)
#define X_GRT_Y                 (1408)
#define TEMPLABEL               (2324)
#define CFG_ANDROID_ORIENT_INT  (1853)
#define CFG_GYRO_RAW_DATA       (2722)
#define X_GRT_Y_TMP2            (1379)

#define D_0_22                  (22+512)
#define D_0_24                  (24+512)

#define D_0_36                  (36)
#define D_0_52                  (52)
#define D_0_96                  (96)
#define D_0_104                 (104)
#define D_0_108                 (108)
#define D_0_163                 (163)
#define D_0_188                 (188)
#define D_0_192                 (192)
#define D_0_224                 (224)
#define D_0_228                 (228)
#define D_0_232                 (232)
#define D_0_236                 (236)

#define D_1_2                   (256 + 2)
#define D_1_4                   (256 + 4)
#define D_1_8                   (256 + 8)
#define D_1_10                  (256 + 10)
#define D_1_24                  (256 + 24)
#define D_1_28                  (256 + 28)
#define D_1_36                  (256 + 36)
#define D_1_40                  (256 + 40)
#define D_1_44                  (256 + 44)
#define D_1_72                  (256 + 72)
#define D_1_74                  (256 + 74)
#define D_1_79                  (256 + 79)
#define D_1_88                  (256 + 88)
#define D_1_90                  (256 + 90)
#define D_1_92                  (256 + 92)
#define D_1_96                  (256 + 96)
#define D_1_98                  (256 + 98)
#define D_1_106                 (256 + 106)
#define D_1_108                 (256 + 108)
#define D_1_112                 (256 + 112)
#define D_1_128                 (256 + 144)
#define D_1_152                 (256 + 12)
#define D_1_160                 (256 + 160)
#define D_1_176                 (256 + 176)
#define D_1_178                 (256 + 178)
#define D_1_218                 (256 + 218)
#define D_1_232                 (256 + 232)
#define D_1_236                 (256 + 236)
#define D_1_240                 (256 + 240)
#define D_1_244                 (256 + 244)
#define D_1_250                 (256 + 250)
#define D_1_252                 (256 + 252)
#define D_2_12                  (512 + 12)
#define D_2_96                  (512 + 96)
#define D_2_108                 (512 + 108)
#define D_2_208                 (512 + 208)
#define D_2_224                 (512 + 224)
#define D_2_236                 (512 + 236)
#define D_2_244                 (512 + 244)
#define D_2_248                 (512 + 248)
#define D_2_252                 (512 + 252)

#define CPASS_BIAS_X            (35 * 16 + 4)
#define CPASS_BIAS_Y            (35 * 16 + 8)
#define CPASS_BIAS_Z            (35 * 16 + 12)
#define CPASS_MTX_00            (36 * 16)
#define CPASS_MTX_01            (36 * 16 + 4)
#define CPASS_MTX_02            (36 * 16 + 8)
#define CPASS_MTX_10            (36 * 16 + 12)
#define CPASS_MTX_11            (37 * 16)
#define CPASS_MTX_12            (37 * 16 + 4)
#define CPASS_MTX_20            (37 * 16 + 8)
#define CPASS_MTX_21            (37 * 16 + 12)
#define CPASS_MTX_22            (43 * 16 + 12)
#define D_EXT_GYRO_BIAS_X       (61 * 16)
#define D_EXT_GYRO_BIAS_Y       (61 * 16) + 4
#define D_EXT_GYRO_BIAS_Z       (61 * 16) + 8
#define D_ACT0                  (40 * 16)
#define D_ACSX                  (40 * 16 + 4)
#define D_ACSY                  (40 * 16 + 8)
#define D_ACSZ                  (40 * 16 + 12)

#define FLICK_MSG               (45 * 16 + 4)
#define FLICK_COUNTER           (45 * 16 + 8)
#define FLICK_LOWER             (45 * 16 + 12)
#define FLICK_UPPER             (46 * 16 + 12)

#define D_AUTH_OUT              (992)
#define D_AUTH_IN               (996)
#define D_AUTH_A                (1000)
#define D_AUTH_B                (1004)

#define D_PEDSTD_BP_B           (768 + 0x1C)
#define D_PEDSTD_HP_A           (768 + 0x78)
#define D_PEDSTD_HP_B           (768 + 0x7C)
#define D_PEDSTD_BP_A4          (768 + 0x40)
#define D_PEDSTD_BP_A3          (768 + 0x44)
#define D_PEDSTD_BP_A2          (768 + 0x48)
#define D_PEDSTD_BP_A1          (768 + 0x4C)
#define D_PEDSTD_INT_THRSH      (768 + 0x68)
#define D_PEDSTD_CLIP           (768 + 0x6C)
#define D_PEDSTD_SB             (768 + 0x28)
#define D_PEDSTD_SB_TIME        (768 + 0x2C)
#define D_PEDSTD_PEAKTHRSH      (768 + 0x98)
#define D_PEDSTD_TIML           (768 + 0x2A)
#define D_PEDSTD_TIMH           (768 + 0x2E)
#define D_PEDSTD_PEAK           (768 + 0X94)
#define D_PEDSTD_STEPCTR        (768 + 0x60)
#define D_PEDSTD_TIMECTR        (964)
#define D_PEDSTD_DECI           (768 + 0xA0)

#define D_HOST_NO_MOT           (976)
#define D_ACCEL_BIAS            (660)

#define D_ORIENT_GAP            (76)

#define D_TILT0_H               (48)
#define D_TILT0_L               (50)
#define D_TILT1_H               (52)
#define D_TILT1_L               (54)
#define D_TILT2_H               (56)
#define D_TILT2_L               (58)
#define D_TILT3_H               (60)
#define D_TILT3_L               (62)

#define DMP_CODE_SIZE           (3062)

#define INT_SRC_TAP             (0x01)
#define INT_SRC_ANDROID_ORIENT  (0x08)

#define DMP_FEATURE_SEND_ANY_GYRO   (DMP_FEATURE_SEND_RAW_GYRO | \
                                     DMP_FEATURE_SEND_CAL_GYRO)

#define MAX_PACKET_LENGTH_DMP   (32)

#define DMP_SAMPLE_RATE     (200)
#define GYRO_SF             (46850825LL * 200 / DMP_SAMPLE_RATE)

#define FIFO_CORRUPTION_CHECK
#ifdef FIFO_CORRUPTION_CHECK
#define QUAT_ERROR_THRESH       (1L<<24)
#define QUAT_MAG_SQ_NORMALIZED  (1L<<28)
#define QUAT_MAG_SQ_MIN         (QUAT_MAG_SQ_NORMALIZED - QUAT_ERROR_THRESH)
#define QUAT_MAG_SQ_MAX         (QUAT_MAG_SQ_NORMALIZED + QUAT_ERROR_THRESH)
#endif

struct dmp_s {
    void (*tap_cb)(unsigned char count, unsigned char direction);
    void (*android_orient_cb)(unsigned char orientation);
    unsigned short orient;
    unsigned short feature_mask;
    unsigned short fifo_rate;
    unsigned char packet_length;
};
#endif

class InvMPU {
public:
	InvMPU();

	/* Set up APIs */
	int mpu_init(void);
	int mpu_set_bypass(unsigned char bypass_on);

	/* Configuration APIs */
	int mpu_lp_accel_mode(unsigned char rate);
	int mpu_lp_motion_interrupt(unsigned short thresh, unsigned char time,
	    unsigned char lpa_freq);
	int mpu_set_int_level(unsigned char active_low);
	int mpu_set_int_latched(unsigned char enable);

	int mpu_set_dmp_state(unsigned char enable);
	int mpu_get_dmp_state(unsigned char *enabled);

	int mpu_get_lpf(unsigned short *lpf);
	int mpu_set_lpf(unsigned short lpf);

	int mpu_get_gyro_fsr(unsigned short *fsr);
	int mpu_set_gyro_fsr(unsigned short fsr);

	int mpu_get_accel_fsr(unsigned char *fsr);
	int mpu_set_accel_fsr(unsigned char fsr);

	int mpu_get_gyro_sens(float *sens);
	int mpu_get_accel_sens(unsigned short *sens);

	int mpu_get_sample_rate(unsigned short *rate);
	int mpu_set_sample_rate(unsigned short rate);

	int mpu_get_fifo_config(unsigned char *sensors);
	int mpu_configure_fifo(unsigned char sensors);

	int mpu_get_power_state(unsigned char *power_on);
	int mpu_set_sensors(unsigned char sensors);

	int mpu_set_accel_bias(const long *accel_bias);
	int mpu_read_6500_accel_bias(long *accel_bias);
	int mpu_read_gyro_bias(long *gyro_bias);
	int mpu_set_gyro_bias_reg(long * gyro_bias);
	int mpu_set_accel_bias_6500_reg(const long *accel_bias);
	int mpu_read_6050_accel_bias(long *accel_bias);
	int mpu_set_accel_bias_6050_reg(const long *accel_bias);

	/* Data getter/setter APIs */
	int mpu_get_gyro_reg(short *data, unsigned long *timestamp);
	int mpu_get_accel_reg(short *data, unsigned long *timestamp);
	int mpu_get_temperature(long *data, unsigned long *timestamp);

	int mpu_get_int_status(short *status);
	int mpu_read_fifo(short *gyro, short *accel, unsigned long *timestamp,
	    unsigned char *sensors, unsigned char *more);
	int mpu_read_fifo_stream(unsigned short length, unsigned char *data,
	    unsigned char *more);
	int mpu_reset_fifo(void);

	int mpu_write_mem(unsigned short mem_addr, unsigned short length,
	    unsigned char *data);
	int mpu_read_mem(unsigned short mem_addr, unsigned short length,
	    unsigned char *data);
	int mpu_load_firmware(unsigned short length, const unsigned char *firmware,
	    unsigned short start_addr, unsigned short sample_rate);

	int mpu_reg_dump(void);
	int mpu_read_reg(unsigned char reg, unsigned char *data);
	int mpu_run_self_test(long *gyro, long *accel);

	/**
	 *  @brief      Enable/disable data ready interrupt.
	 *  If the DMP is on, the DMP interrupt is enabled. Otherwise, the data ready
	 *  interrupt is used.
	 *  @param[in]  enable      1 to enable interrupt.
	 *  @return     0 if successful.
	 */
	static int set_int_enable(unsigned char enable)
	{
	    unsigned char tmp;

	    if (st.chip_cfg.dmp_on) {
	        if (enable)
	            tmp = BIT_DMP_INT_EN;
	        else
	            tmp = 0x00;
	        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &tmp))
	            return -1;
	        st.chip_cfg.int_enable = tmp;
	    } else {
	        if (!st.chip_cfg.sensors)
	            return -1;
	        if (enable && st.chip_cfg.int_enable)
	            return 0;
	        if (enable)
	            tmp = BIT_DATA_RDY_EN;
	        else
	            tmp = 0x00;
	        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &tmp))
	            return -1;
	        st.chip_cfg.int_enable = tmp;
	    }
	    return 0;
	}

	static int get_accel_prod_shift(float *st_shift)
	{
	    unsigned char tmp[4], shift_code[3], ii;

	    if (i2c_read(st.hw->addr, 0x0D, 4, tmp))
	        return 0x07;

	    shift_code[0] = ((tmp[0] & 0xE0) >> 3) | ((tmp[3] & 0x30) >> 4);
	    shift_code[1] = ((tmp[1] & 0xE0) >> 3) | ((tmp[3] & 0x0C) >> 2);
	    shift_code[2] = ((tmp[2] & 0xE0) >> 3) | (tmp[3] & 0x03);
	    for (ii = 0; ii < 3; ii++) {
	        if (!shift_code[ii]) {
	            st_shift[ii] = 0.f;
	            continue;
	        }
	        /* Equivalent to..
	         * st_shift[ii] = 0.34f * powf(0.92f/0.34f, (shift_code[ii]-1) / 30.f)
	         */
	        st_shift[ii] = 0.34f;
	        while (--shift_code[ii])
	            st_shift[ii] *= 1.034f;
	    }
	    return 0;
	}

	static int accel_self_test(long *bias_regular, long *bias_st)
	{
	    int jj, result = 0;
	    float st_shift[3], st_shift_cust, st_shift_var;

	    get_accel_prod_shift(st_shift);
	    for(jj = 0; jj < 3; jj++) {
	        st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
	        if (st_shift[jj]) {
	            st_shift_var = st_shift_cust / st_shift[jj] - 1.f;
	            if (fabs(st_shift_var) > test.max_accel_var)
	                result |= 1 << jj;
	        } else if ((st_shift_cust < test.min_g) ||
	            (st_shift_cust > test.max_g))
	            result |= 1 << jj;
	    }

	    return result;
	}

	static int gyro_self_test(long *bias_regular, long *bias_st)
	{
	    int jj, result = 0;
	    unsigned char tmp[3];
	    float st_shift, st_shift_cust, st_shift_var;

	    if (i2c_read(st.hw->addr, 0x0D, 3, tmp))
	        return 0x07;

	    tmp[0] &= 0x1F;
	    tmp[1] &= 0x1F;
	    tmp[2] &= 0x1F;

	    for (jj = 0; jj < 3; jj++) {
	        st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
	        if (tmp[jj]) {
	            st_shift = 3275.f / test.gyro_sens;
	            while (--tmp[jj])
	                st_shift *= 1.046f;
	            st_shift_var = st_shift_cust / st_shift - 1.f;
	            if (fabs(st_shift_var) > test.max_gyro_var)
	                result |= 1 << jj;
	        } else if ((st_shift_cust < test.min_dps) ||
	            (st_shift_cust > test.max_dps))
	            result |= 1 << jj;
	    }
	    return result;
	}

	static int get_st_biases(long *gyro, long *accel, unsigned char hw_test)
	{
	    unsigned char data[MAX_PACKET_LENGTH];
	    unsigned char packet_count, ii;
	    unsigned short fifo_count;

	    data[0] = 0x01;
	    data[1] = 0;
	    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, data))
	        return -1;
	    delay_ms(200);
	    data[0] = 0;
	    if (i2c_write(st.hw->addr, st.reg->int_enable, 1, data))
	        return -1;
	    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
	        return -1;
	    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
	        return -1;
	    if (i2c_write(st.hw->addr, st.reg->i2c_mst, 1, data))
	        return -1;
	    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
	        return -1;
	    data[0] = BIT_FIFO_RST | BIT_DMP_RST;
	    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
	        return -1;
	    delay_ms(15);
	    data[0] = st.test->reg_lpf;
	    if (i2c_write(st.hw->addr, st.reg->lpf, 1, data))
	        return -1;
	    data[0] = st.test->reg_rate_div;
	    if (i2c_write(st.hw->addr, st.reg->rate_div, 1, data))
	        return -1;
	    if (hw_test)
	        data[0] = st.test->reg_gyro_fsr | 0xE0;
	    else
	        data[0] = st.test->reg_gyro_fsr;
	    if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, data))
	        return -1;

	    if (hw_test)
	        data[0] = st.test->reg_accel_fsr | 0xE0;
	    else
	        data[0] = test.reg_accel_fsr;
	    if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, data))
	        return -1;
	    if (hw_test)
	        delay_ms(200);

	    /* Fill FIFO for test.wait_ms milliseconds. */
	    data[0] = BIT_FIFO_EN;
	    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
	        return -1;

	    data[0] = INV_XYZ_GYRO | INV_XYZ_ACCEL;
	    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
	        return -1;
	    delay_ms(test.wait_ms);
	    data[0] = 0;
	    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
	        return -1;

	    if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data))
	        return -1;

	    fifo_count = (data[0] << 8) | data[1];
	    packet_count = fifo_count / MAX_PACKET_LENGTH;
	    gyro[0] = gyro[1] = gyro[2] = 0;
	    accel[0] = accel[1] = accel[2] = 0;

	    for (ii = 0; ii < packet_count; ii++) {
	        short accel_cur[3], gyro_cur[3];
	        if (i2c_read(st.hw->addr, st.reg->fifo_r_w, MAX_PACKET_LENGTH, data))
	            return -1;
	        accel_cur[0] = ((short)data[0] << 8) | data[1];
	        accel_cur[1] = ((short)data[2] << 8) | data[3];
	        accel_cur[2] = ((short)data[4] << 8) | data[5];
	        accel[0] += (long)accel_cur[0];
	        accel[1] += (long)accel_cur[1];
	        accel[2] += (long)accel_cur[2];
	        gyro_cur[0] = (((short)data[6] << 8) | data[7]);
	        gyro_cur[1] = (((short)data[8] << 8) | data[9]);
	        gyro_cur[2] = (((short)data[10] << 8) | data[11]);
	        gyro[0] += (long)gyro_cur[0];
	        gyro[1] += (long)gyro_cur[1];
	        gyro[2] += (long)gyro_cur[2];
	    }

	    gyro[0] = (long)(((long long)gyro[0]<<16) / test.gyro_sens / packet_count);
	    gyro[1] = (long)(((long long)gyro[1]<<16) / test.gyro_sens / packet_count);
	    gyro[2] = (long)(((long long)gyro[2]<<16) / test.gyro_sens / packet_count);
	    accel[0] = (long)(((long long)accel[0]<<16) / test.accel_sens /
	        packet_count);
	    accel[1] = (long)(((long long)accel[1]<<16) / test.accel_sens /
	        packet_count);
	    accel[2] = (long)(((long long)accel[2]<<16) / test.accel_sens /
	        packet_count);
	    /* Don't remove gravity! */
	    if (accel[2] > 0L)
	        accel[2] -= 65536L;
	    else
	        accel[2] += 65536L;

	    return 0;
	}


#ifdef USE_DMP
	/* Set up functions. */
	int dmp_load_motion_driver_firmware(void);
	int dmp_set_fifo_rate(unsigned short rate);
	int dmp_get_fifo_rate(unsigned short *rate);
	int dmp_enable_feature(unsigned short mask);
	int dmp_get_enabled_features(unsigned short *mask);
	int dmp_set_interrupt_mode(unsigned char mode);
	int dmp_set_orientation(unsigned short orient);
	int dmp_set_gyro_bias(long *bias);
	int dmp_set_accel_bias(long *bias);

	/* Tap functions. */
	int dmp_register_tap_cb(void (*func)(unsigned char, unsigned char));
	int dmp_set_tap_thresh(unsigned char axis, unsigned short thresh);
	int dmp_set_tap_axes(unsigned char axis);
	int dmp_set_tap_count(unsigned char min_taps);
	int dmp_set_tap_time(unsigned short time);
	int dmp_set_tap_time_multi(unsigned short time);
	int dmp_set_shake_reject_thresh(long sf, unsigned short thresh);
	int dmp_set_shake_reject_time(unsigned short time);
	int dmp_set_shake_reject_timeout(unsigned short time);

	/* Android orientation functions. */
	int dmp_register_android_orient_cb(void (*func)(unsigned char));

	/* LP quaternion functions. */
	int dmp_enable_lp_quat(unsigned char enable);
	int dmp_enable_6x_lp_quat(unsigned char enable);

	/* Pedometer functions. */
	int dmp_get_pedometer_step_count(unsigned long *count);
	int dmp_set_pedometer_step_count(unsigned long count);
	int dmp_get_pedometer_walk_time(unsigned long *time);
	int dmp_set_pedometer_walk_time(unsigned long time);

	/* DMP gyro calibration functions. */
	int dmp_enable_gyro_cal(unsigned char enable);

	/* Read function. This function should be called whenever the MPU interrupt is
	 * detected.
	 */
	int dmp_read_fifo(short *gyro, short *accel, long *quat,
	    unsigned long *timestamp, short *sensors, unsigned char *more);

	/**
	 *  @brief      Decode the four-byte gesture data and execute any callbacks.
	 *  @param[in]  gesture Gesture data from DMP packet.
	 *  @return     0 if successful.
	 */
	static int decode_gesture(unsigned char *gesture)
	{
	    unsigned char tap, android_orient;

	    android_orient = gesture[3] & 0xC0;
	    tap = 0x3F & gesture[3];

	    if (gesture[1] & INT_SRC_TAP) {
	        unsigned char direction, count;
	        direction = tap >> 3;
	        count = (tap % 8) + 1;
	        if (dmp.tap_cb)
	            dmp.tap_cb(direction, count);
	    }

	    if (gesture[1] & INT_SRC_ANDROID_ORIENT) {
	        if (dmp.android_orient_cb)
	            dmp.android_orient_cb(android_orient >> 6);
	    }

	    return 0;
	}

#endif

private:
	static const struct gyro_reg_s reg;
	static const struct hw_s hw;
	static const struct test_s test;
	static struct gyro_state_s st;

#ifdef USE_DMP
	static struct dmp_s dmp;
	static const unsigned short sStartAddress;
	static const unsigned char dmp_memory[DMP_CODE_SIZE];

#endif
};

#endif  /* #ifndef _INV_MPU_H_ */

