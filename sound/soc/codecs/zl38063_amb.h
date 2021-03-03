#ifndef __ZL38063_AMB_H
#define __ZL38063_AMB_H

#include "zl38063_firmware_amb.h"
#include "zl38063_config_amb.h"



#undef MICROSEMI_DEMO_PLATFORM /*leave this macro undefined unless requested by Microsemi*/
/*-------------------------------------------------------------*
 *    HOST MACROS - Define/undefine as desired
 *    -----------------------------------------
 *    Supported combinations:
 *    ZL380XX_TW_ENABLE_CHAR_DEV_DRIVER + ZL380XX_TW_ENABLE_ALSA_CODEC_DRIVER  + ZL38063_I2C_IF or ZL38063_SPI_IF
 *    ZL380XX_TW_ENABLE_CHAR_DEV_DRIVER + ZL38063_I2C_IF or ZL38063_SPI_IF
 *    ZL380XX_TW_ENABLE_ALSA_CODEC_DRIVER  + ZL38063_I2C_IF or ZL38063_SPI_IF
 *
 *    all of the above can be used with the ZL380XX_TW_UPDATE_FIRMWARE
 *-------------------------------------------------------------*/
#define ZL380XX_TW_ENABLE_ALSA_CODEC_DRIVER /*Define this macro to create a /sound/soc ALSA codec device driver*/
#define ZL380XX_TW_ENABLE_CHAR_DEV_DRIVER   /*Define this macro to create a character device driver*/
#define ZL38063_SLAVE_DEVICE_SUPPORT
//#define ZL38063_DOA_UPLOAD

#define AMBA_ZATANNA_BOARD "Ambarella Zatanna Board"

/**
 * Default value for a register.  We use an array of structs rather
 * than a simple array as many modern devices have very sparse
 * register maps.
 *
 * @reg: Register address.
 * @def: Register default value.
 */
struct reg_format {
	u16 reg;
	u16 val;
};

static u16 last_doa = 0 ;

#define ZL38063_MAX_DATA_LENGTH	24
#define ZL38063_DEF_ASR_FWID	0x8001			//(FIXED me) the special FWID of ASR
#define ZL38063_DEF_AEC_FWID	0x800C


enum NL_MSG_TYPE {
	NL_MSG_SILENCE = 0,						//silent or under noise level
	NL_MSG_DIR_CHANGE_NOTIFY = 2,			//send direction change notify
	NL_MSG_DIR_SEND = 4,					//send direction to user
	NL_MSG_HOST_CMD = 0x80,
};
enum NL_MSG_PORT {
	NETLINK_TEST = 27,
};

enum NL_MIC_DIR_TYPE {
	MIC_CLOCKWISE = 0,
	MIC_ANTICLOCKWISE,
};

enum LED_STATUS {
	LED1_ON = 0x0002,
	LED2_ON = 0x0004,
	LED3_ON = 0x0008,
	LED4_ON = 0x0010,
	LED5_ON = 0x0020,
	LED6_ON = 0x0040,
	LED7_ON = 0x0080,
	LED8_ON = 0x0100,
	LED9_ON = 0x0200,
	LED10_ON = 0x0400,
	LED11_ON = 0x0800,
	LED12_ON = 0x1000,
	LED_ALL_ON  = 0x8000,
	LED_ALL_OFF = 0x0001,
};

enum MIC_DIR_REG {
	MIC_0 = 0x2040,
	MIC_30 = 0x2400,
	MIC_60 = 0x2040,
	MIC_90 = 0x2400,
	MIC_120 = 0x2080,
	MIC_150 = 0x2800,
	MIC_180 = 0x2080,
	MIC_210 = 0x2800,
	MIC_240 = 0x2100,
	MIC_270 = 0x3000,
	MIC_300 = 0x2100,
	MIC_330 = 0x3000,
	MIC_ALL_ON = 0x3dc0,
	MIC_ALL_OFF = 0x2000,
};

enum FW_LABEL {
	ASR = 1,
	AEC = 2,
	DOA = 3,
	UPDATE_PRAM =4,
	UPDATE_CRAM =8,
};

enum BEAM_WIDTH_CONF {
	WIDE=60,				//60 degree and set threshold
	GENERAL=40,				//40 degree :DEFAULT
	NARROW=20,				//20 degree
	VERY_NARROW=10,			//10 degree
};

enum NOISE_LEVEL {
	LIKE_DEEP_NIGHT=0,
	LIKE_OFFICE,		//DEFAULT
	LIKE_STREET,
	LIKE_COFFEE_BAR,
};

enum NL_MSG_CMD {
	NL_MSG_REV = 0,
	NL_MSG_GET_DIR ,			// data:  degree,clockwise from VIN0 at  0 degree (-90 ~~ +90)
	NL_MSG_GET_POLL_CONF,		//data[0]:  xxx ms pollling frequency, data[1] full beamforming width in degree, data[2] noise level(TBD)
	NL_MSG_SET_POLL_CONF,
	NL_MSG_GET_FW_VERSION,		// data:  e.g. FW:38063 rev:1.0.3
	NL_MSG_LOAD_FW,				// data:  ASR(1) / AEC(2) update data : file location
	NL_MSG_GET_MIC_DIR,			// data:  the degree offset between two MIC's centerline and VIN0 ,overlap 0 degree/ clockwise from (-90 ~~ +90)
	NL_MSG_SET_MIC_DIR,			// data[0] = e.g. 0 to adjust
	NL_MSG_READ_REG,			//data[0]=addr data[1]=length ,data[..]=data read back
	NL_MSG_WRITE_REG,			//data[0]=addr data[1]=length ,data[..]=data to be written
	NL_MSG_WAKEUP,				//data[0]=action (LED_STATUS)
};
enum NL_MSG_STATUS {
	NL_CMD_STATUS_SUCCESS = 0,
	NL_CMD_STATUS_FAIL,			//data: some info about error
};

struct nl_msg_audio {
	u16		data[ZL38063_MAX_DATA_LENGTH];
	u8		type;
	u8		port;
	u8		cmd;
	u8		status;
};

struct beamforming_conf {
	u16		polling_time;
	u16		offset_degree;
	u16		noise_level;
	u16		mic_number;
	u16		mic_Space;
	u16		steer_Angle;
	u16		halfbeam_Width;
	u16		oobMaxGain;
	u16		oobMinGain;
	u16		soundLocCtl;
	u16		soundLocTh;
};


/*Enable either one of this macro to create a SPI or an I2C device driver
*  to be used as the low-level transport for the ALSA and/or CHAR device read/write accesses
*/
#undef ZL38063_I2C_IF          /*Enable this macro if the HBI interface between the host CPU and the Twolf is I2C*/
#ifdef ZL38063_I2C_IF
    #undef ZL38063_SPI_IF
    #define MICROSEMI_I2C_ADDR 0x45  /*if DIN pin is tied to ground, else if DIN is tied to 3.3V address must be 0x52*/
    #define CONTROLLER_I2C_BUS_NUM 0
#else
    #define ZL38063_SPI_IF
    /*Define the SPI master signal config*/
    #define SPIM_CLK_SPEED  6000000
    #define SPIM_CHIP_SELECT 0
    #define SPIM_MODE SPI_MODE_0
    #define SPIM_BUS_NUM 1
#endif

//#undef ZL380XX_TW_UPDATE_FIRMWARE /*define if you want to update current firmware with a new one at power up*/
//#undef ZL380XX_TW_UPDATE_FIRMWARE
//#define ZL38063_TW_UPDATE_FIRMWARE

//#define ZL38063_INCLUDE_FIRMWARE_REQUEST_LIB
/*NOTE: Rename the *s3 firmware file as per below or simply change the file name below as per the firmware file name
*       If there is more than 1 device create new path name and pass them to the firmware loading function accordingly
*/
#define  ZLS380630_TWOLF "ZLS38062.0_E1.1.0_App.s3" /*firmware for your zl380xx device 0*/


#define  ZLS380630_TWOLF_CRK "ZLS38062_20151223_AMBA.CR2" /*configuration record for your zl380xx device 0*/



#undef ZL38063_SAVE_FWR_TO_FLASH  /*define if a slave flash is connected to the zl38063 and you want to save the loaded firmware/config to flash*/



/*HBI access to the T-wolf must not be interrupted by another process*/
#define PROTECT_CRITICAL_SECTION  /*define this macro to protect HBI critical section*/

/*The zl38063 device registration can be done using the Linux device OF matching scheme or
* or by using the old method via the board init file
* This driver includes the method to auto-detect the SPI or I2C master and register itself to that
* master as per the defined bus info above
*/
#define SUPPORT_LINUX_DEVICE_TREE_OF_MATCHING  /*define this if you are using the linux device tree (*dts, *dtb, etc.. of maching lib*/
#ifdef SUPPORT_LINUX_DEVICE_TREE_OF_MATCHING
#define CONTROLLER_DTS_STRING  "ambarella" /*Change the string name accordingly to your device tree controller/driver maching name*/
#else
/*define this macro if the board_info registration is not already done in your board init file
* This macro will cause a new slave device (SPI or I2C) to be created and attached to the master
* device controller
* If you already assigned the ressource for this zl38063 driver in your board init file, then undef this macro.
*/
//#define ZL38063_DEV_BIND_SLAVE_TO_MASTER
#endif


/*Define the ZL38063 interrupt pin drive mode 1:TTL, 0: Open Drain(default)*/
#define HBI_CONFIG_INT_PIN_DRIVE_MODE		0
/*-------------------------------------------------------------*
 *     HOST MACROS - end
 *-------------------------------------------------------------*/


/* local defines */
#define MAX_TWOLF_ACCESS_SIZE_IN_BYTES 264 /*127 16-bit words*/
#define MAX_TWOLF_FIRMWARE_SIZE_IN_BYTES 128 /*128 8-bit words*/

/*The timberwolf device reset modes*/
#define ZL38063_RST_HARDWARE_RAM 0
#define ZL38063_RST_HARDWARE_ROM 1
#define ZL38063_RST_SOFTWARE     2
#define ZL38063_RST_AEC          3
#define ZL38063_RST_TO_BOOT      4

#ifdef ZL380XX_TW_ENABLE_CHAR_DEV_DRIVER

#define NUMBER_OF_ZL380xx_DEVICES 1
#define FIRST_MINOR 0
/*structure for IOCL access*/
typedef struct {
	__u16	addr;
	__u16	data;
} ioctl_zl38063;


/* ioctl() calls that are permitted to the /dev/microsemi_spis_tw interface. */
#define TWOLF_MAGIC 'q'            /*Change this accordingly to your system*/
#define TWOLF_HBI_RD16		        _IOWR(TWOLF_MAGIC, 1,  ioctl_zl38063 *)
#define TWOLF_HBI_WR16		        _IOW(TWOLF_MAGIC, 2, ioctl_zl38063 *)
#define TWOLF_HBI_INIT		        _IOW(TWOLF_MAGIC, 3, __u16)
#define TWOLF_RESET        	        _IOW(TWOLF_MAGIC, 4,  __u16)
#define TWOLF_SAVE_FWR_TO_FLASH     _IO(TWOLF_MAGIC, 5)
#define TWOLF_LOAD_FWR_FROM_FLASH   _IOW(TWOLF_MAGIC, 6,  __u16)
#define TWOLF_SAVE_CFG_TO_FLASH     _IOW(TWOLF_MAGIC, 7,  __u16)
#define TWOLF_LOAD_CFG_FROM_FLASH   _IOW(TWOLF_MAGIC, 8,  __u16)
#define TWOLF_ERASE_IMGCFG_FLASH    _IOW(TWOLF_MAGIC, 9,  __u16)
#define TWOLF_ERASE_ALL_FLASH       _IO(TWOLF_MAGIC, 10)
#define TWOLF_STOP_FWR              _IO(TWOLF_MAGIC, 11)
#define TWOLF_START_FWR             _IO(TWOLF_MAGIC, 12)
#define TWOLF_LOAD_FWRCFG_FROM_FLASH    _IOW(TWOLF_MAGIC, 13,  __u16)
#define TWOLF_HBI_WR_ARB_SINGLE_WORD    _IOW(TWOLF_MAGIC, 14, __u16)
#define TWOLF_HBI_RD_ARB_SINGLE_WORD    _IOW(TWOLF_MAGIC, 15, __u16)
#define TWOLF_CMD_PARAM_REG_ACCESS	    _IOW(TWOLF_MAGIC, 16, __u16)
#define TWOLF_CMD_PARAM_RESULT_CHECK    _IO(TWOLF_MAGIC, 17)
#define TWOLF_BOOT_PREPARE              _IO(TWOLF_MAGIC, 18)
#define TWOLF_BOOT_SEND_MORE_DATA       _IOW(TWOLF_MAGIC, 19, int)
#define TWOLF_BOOT_CONCLUDE             _IO(TWOLF_MAGIC, 20)
#define TWOLF_LOAD_CFG		            _IOW(TWOLF_MAGIC, 21,  int)
#endif
/*------------------------------------------------------*/
/*TWOLF REGisters*/
#define ZL38063_CMD_REG             0x0032   /*Host Command register*/
#define ZL38063_CMD_IDLE            0x0000  /*idle/ operation complete*/
#define ZL38063_CMD_NO_OP           0x0001  /*no-op*/
#define ZL38063_CMD_IMG_CFG_LOAD    0x0002  /*load firmware and CR from flash*/
#define ZL38063_CMD_IMG_LOAD        0x0003  /*load firmware only from flash*/
#define ZL38063_CMD_IMG_CFG_SAVE    0x0004  /*save a firmware and CR to flash*/
#define ZL38063_CMD_IMG_CFG_ERASE   0x0005  /*erase a firmware and CR in flash*/
#define ZL38063_CMD_CFG_LOAD        0x0006  /*Load CR from flash*/
#define ZL38063_CMD_CFG_SAVE        0x0007  /*save CR to flash*/
#define ZL38063_CMD_FWR_GO          0x0008  /*start/restart firmware (GO)*/
#define ZL38063_CMD_HOST_LOAD_CMP   0x000D  /*Host Application Load Complete*/
#define ZL38063_CMD_HOST_FLASH_INIT 0x000B  /*Host Application flash discovery*/
#define ZL38063_CMD_FWR_STOP        0x8000  /*stop firmware */
#define ZL38063_CMD_CMD_IN_PROGRESS 0xFFFF  /*wait command is in progress */
#define ZL38063_CMD_APP_SLEEP		 0x8005  /*codec low power mode*/

#define PAGE_255_CHKSUM_LO_REG  0x000A
#define PAGE_255_CHKSUM_HI_REG  0x0008
#define CLK_STATUS_REG          0x0014   /*Clock status register*/
#define PAGE_255_BASE_LO_REG  0x000E
#define PAGE_255_BASE_HI_REG  0x000C
#define ZL38063_SW_FLAGS_REG     0x0006
#define ZL38063_SW_FLAGS_CMD     0x0001
#define ZL38063_SW_FLAGS_CMD_NORST     0x0004

#define ZL38063_DEVICE_ID_REG  0x0022

#define TWOLF_CLK_STATUS_HBI_BOOT       0x0001

#define HBI_CONFIG_REG			0xFD00
#define HBI_CONFIG_WAKE			1<<7
#define HBI_CONFIG_VAL (HBI_CONFIG_INT_PIN_DRIVE_MODE<<1)

#define ZL38063_CMD_PARAM_RESULT_REG   0x0034 /*Host Command Param/Result register*/
#define ZL38063_FWR_COUNT_REG   0x0026 /*Fwr on flash count register*/
#define ZL38063_FWR_EXEC_REG   0x012C  /*Fwr EXEC register*/

#define TOTAL_FWR_DATA_WORD_PER_LINE 24
#define TOTAL_FWR_DATA_BYTE_PER_LINE 128
#define TWOLF_STATUS_NEED_MORE_DATA 22
#define TWOLF_STATUS_BOOT_COMPLETE 23

#define TWOLF_MBCMDREG_SPINWAIT  10000
/*--------------------------------------------------------------------
 *    ALSA
 *--------------------------------------------------------------------*/
 /*Macros to enable one of the pre-defined audio cross-points*/
#define ZL38063_CR2_DEFAULT 0
#define ZL38063_CR2_STEREO_BYPASS      1
#define ZL38063_ADDA_LOOPBACK      2

/*Cached register range*/
#define ZL38063_CACHED_ADDR_LO  0x202
#define ZL38063_CACHED_ADDR_HI  0x23E
#define ZL38063_HBI_OFFSET_RANGE 128
#define ZL38063_CACHE_INDEX_TO_ADDR(index) (ZL38063_CACHED_ADDR_LO+(2*index))
#define ZL38063_ADDR_TO_CACHE_INDEX(addr) ((addr - ZL38063_CACHED_ADDR_LO)/2)

#define ZL38063_TDM_I2S_CFG_VAL 0x8000
#define ZL38063_TDM_PCM_CFG_VAL 0x0000
#define ZL38063_TDM_CLK_POL_VAL 0x0004
#define ZL38063_TDMA_FSALIGN 0x01     /*left justified*/
#define ZL38063_TDM_TDM_MASTER_VAL (1<<15)
#define ZL38063_TDMA_CH1_CFG_REG    	0x268
#define ZL38063_TDMA_CH2_CFG_REG    	0x26A
/*TDM -  Channel configuration*/
#define ZL38063_TDMA_16BIT_LIN (1<<8)
#define ZL38063_TDMA_8BIT_ALAW (2<<8)
#define ZL38063_TDMA_8BIT_ULAW (3<<8)
#define ZL38063_TDMA_8BIT_G722 (4<<8)
#define ZL38063_TDMA_16BIT_LINHFS (6<<8)

#define ZL38063_TDMA_FSRATE_8KHZ (1)
#define ZL38063_TDMA_FSRATE_16KHZ (2)
#define ZL38063_TDMA_FSRATE_24KHZ (3)
#define ZL38063_TDMA_FSRATE_44_1KHZ (5)
#define ZL38063_TDMA_FSRATE_48KHZ (6)

#define ZL38063_MIC_EN_REG 			0x2B0
#define ZL38063_MIC1_EN		0x01
#define ZL38063_MIC2_EN		0x02
#define ZL38063_MIC3_EN		0x04
#define ZL38063_MIC4_EN		0x08

#define ZL38063_LOW_POWER_REG  0x0206

#define ZL38063_DAC1_EN_REG  0x02A0
#define ZL38063_DAC2_EN_REG  0x02A2
#define ZL38063_DACx_P_EN  (1<<15)
#define ZL38063_DACx_M_EN  (1<<14)




/*Page 2 registers*/
#define ZL38063_USRGAIN		  0x30A
#define ZL38063_SYSGAIN		  0x30C
#define ZL38063_MICGAIN		  0x2B2 /*Range 0-7 step +/-1 = 6dB each*/

#define ZL38063_DAC_CTRL_REG  0x030A   /*ROUT GAIN control*/
#define ZL38063_DAC_VOL_MAX   0x78     /*Max volume control for Speaker +21dB*/
#define ZL38063_DAC_VOL_MAX_EXT   0x82     /*Max volume control for Speaker +29dB*/
#define ZL38063_DAC_VOL_MIN   0x00     /*Min volume control for Speaker -24dB*/
#define ZL38063_DAC_VOL_STEP  0x01     /*volume step control for Speaker -/+0.375dB*/

#define ZL38063_MIC_VOL_CTRL_REG  0x030C    /*SIN GAIN control*/
#define ZL38063_MIC_VOL_MAX   0x1F     /*Max volume control for Speaker +22.5dB*/
#define ZL38063_MIC_VOL_MIN   0x00     /*Min volume control for Speaker -24dB*/
#define ZL38063_MIC_VOL_STEP  0x01     /*volume step control for Speaker -/+1.5dB*/
#define ZL38063_SOUT_VOL_CTRL_REG  0x030C    /*SOUT DIGITAL GAIN control*/
#define ZL38063_SOUT_VOL_MAX   0x0F     /*Max volume control for Speaker +21dB*/
#define ZL38063_SOUT_VOL_MIN   0x00     /*Min volume control for Speaker -24dB*/
#define ZL38063_SOUT_VOL_STEP  0x01     /*volume step control for Speaker -/+3.0dB*/

#define ZL38063_DAC1_GAIN_REG  0x0238
#define ZL38063_DAC2_GAIN_REG  0x023A
#define ZL38063_I2S1L_GAIN_REG  0x023C
#define ZL38063_I2S1R_GAIN_REG  0x023E
#define ZL38063_I2S2L_GAIN_REG  0x0244
#define ZL38063_I2S2R_GAIN_REG  0x0246
#define ZL38063_TDMA3_GAIN_REG  0x0240
#define ZL38063_TDMA4_GAIN_REG  0x0242
#define ZL38063_TDMB3_GAIN_REG  0x0248
#define ZL38063_TDMB4_GAIN_REG  0x024A



#define ZL38063_AEC_CTRL_REG1  0x0302
#define ZL38063_AEC_CTRL_REG0  0x0300
#define ZL38063_EAC_RST_EN  (1 << 0)
#define ZL38063_MASTER_BYPASS_EN  (1 << 1)
#define ZL38063_EQ_RCV_DIS_EN  (1 << 2)
#define ZL38063_AEC_BYPASS_EN  (1 << 4)
#define ZL38063_AUD_ENH_BYPASS_EN  (1 << 5)
#define ZL38063_SPKR_LIN_EN  (1 << 6)
#define ZL38063_MUTE_ROUT_EN  (1 << 7)
#define ZL38063_MUTE_SOUT_EN  (1 << 8)
#define ZL38063_MUTE_ALL_EN   (ZL38063_MUTE_ROUT_EN | ZL38063_MUTE_SOUT_EN)
#define ZL38063_RIN_HPF_DIS_EN  (1 << 9)
#define ZL38063_SIN_HPF_DIS_EN  (1 << 10)
#define ZL38063_HOWLING_DIS_EN  (1 << 11)
#define ZL38063_AGC_DIS_EN  (1 << 12)
#define ZL38063_NB_DIS_EN  (1 << 13)
#define ZL38063_SATT_DIS_EN  (1 << 14)
#define ZL38063_HOWLING_MB_DIS_EN  (1 << 15)
#define ZL38063_HPF_DIS (ZL38063_RIN_HPF_DIS_EN | ZL38063_SIN_HPF_DIS_EN)

#define ZL38063_LEC_CTRL_REG  0x037A

#define ZL38063_AEC_HPF_NULL_REG  0x0310


/*****Boot ROM and Application Status Registers*****/


/*1 Boot Status and Control*/
#define ZL38063_INT_IND_REG			0x0000
#define ZL38063_INT_PARM_REG		0x0002
#define ZL38063_SIG_FLAG			0x0006
#define ZL38063_CLK_STA				0x0014
#define ZL38063_MAIN_CLK			0x0016
#define ZL38063_SYS_ERROR			0x0018
#define ZL38063_SYS_ERROR_MASK		0x001A
#define ZL38063_HW_VERSION			0x0020
#define ZL38063_PRODUCT_CODE		0x0022
#define ZL38063_FM_VERSION			0x0024
#define ZL38063_FM_NUM				0x0026
#define ZL38063_FM_CUR				0x0028
#define ZL38063_FM_SIZE				0x002A
#define ZL38063_FM_ADDR				0x002C
#define ZL38063_CONF_ADDR			0x002E
#define ZL38063_BOOTLOAD_STA		0x0030
#define ZL38063_HOST_CMD			0x0032
#define ZL38063_HOST_CMD_STA		0x0034
#define ZL38063_FREE_10MS			0x0036
#define ZL38063_BOOT_SENSE_H		0x0038
#define ZL38063_BOOT_SENSE_L		0x003A
#define ZL38063_RESET_TYPE			0x003C
#define ZL38063_HOST_CMD_CNT		0x003E

/*2 Application Status Registers*/
#define ZL38063_AEC_STA				0x0068
#define ZL38063_AEC_STA1			0x006A
#define ZL38063_AEC_MU_STA			0x006C
#define ZL38063_AEC_ERLE_STA		0x006E

/*3 Peak Signal Registers*/
#define ZL38063_SIN_PEAK			0x0072
#define ZL38063_AEC_PEAK_ERR		0x0074
#define ZL38063_ROUT_PEAK			0x0076
#define ZL38063_RIN_PEAK			0x0078
#define ZL38063_SOUT_PEAK			0x007C

/*4 Multi-AEC Status Registers*/
#define ZL38063_AEC_PRE_1			0x0080
#define ZL38063_AEC_PRE_2			0x0082
#define ZL38063_AEC_PRE_3			0x0084
#define ZL38063_SIN_PEAK_1			0x0088
#define ZL38063_SIN_PEAK_2			0x008A
#define ZL38063_SIN_PEAK_3			0x008C
#define ZL38063_AEC_ERLE_1			0x0090
#define ZL38063_AEC_ERLE_2			0x0092
#define ZL38063_AEC_ERLE_3			0x0094

#define ZL38063_SOUND_LOC_DIR		0x00A0


/*5 Compressor / Limiter / Expander (CLE) Status Registers*/
#define ZL38063_SOUT_CLE_LEVEL		0x00A4			//Q9.7
#define ZL38063_SOUT_CLE_STA		0x00A6
#define ZL38063_ROUT_CLE_LEVEL		0x00A8			//Q9.7
#define ZL38063_ROUT_CLE_STA		0x00AA
#define ZL38063_SLAVE_STA			0x00AE

/*6 Microphone Status and Energy Registers*/
#define ZL38063_MIC_ENERGY_1		0x00B0			//Q8.8
#define ZL38063_MIC_ENERGY_2		0x00B2
#define ZL38063_MIC_ENERGY_3		0x00B4
#define ZL38063_MIC_ENERGY_4		0x00B6
#define ZL38063_MIC_ENERGY_5		0x00BC
#define ZL38063_MIC_ENERGY_6		0x00BA
#define ZL38063_MIC_ENERGY_7		0x00BC
#define ZL38063_MIC_ENERGY_8		0x00BE
#define ZL38063_MIC_ENERGY_9		0x00C0
#define ZL38063_MIC_ENERGY_10		0x00C2
#define ZL38063_MIC_ENERGY_11		0x00C4
#define ZL38063_MIC_ENERGY_12		0x00C6
#define ZL38063_MIC_ENERGY_13		0x00C8
#define ZL38063_MIC_ENERGY_14		0x00CA
#define ZL38063_MIC_ENERGY_15		0x00CC			//Q8.8
#define ZL38063_MIC_SELECT			0x00CE

/*7 GPIO Read Data Registers*/
#define ZL38063_GPIO_RD_M			0x00D0
#define ZL38063_GPIO_RD_S1			0x00D2
#define ZL38063_GPIO_RD_S2			0x00D4
#define ZL38063_GPIO_RD_S3			0x00D6
#define ZL38063_GPIO_RD_S4			0x00D8


/*8 TDM Status Registers*/
#define ZL38063_TMDA_CLK_STA		0x00E0
#define ZL38063_TMDB_CLK_STA		0x00E2


/*****Boot ROM and Application Status Registers*****/


/*****Flash and Application Information Registers*****/

/*1 Flash Header Registers*/
/*2 Firmware Image Header*/
/*3 Current Configuration Record Image*/


/*****Flash and Application Information Registers*****/


/*****Configuration Record*****/


/*1 System Control Interface*/
#define ZL38063_GPIO_EVEN			0x0200
#define ZL38063_PATH_CPEN			0x0202
#define ZL38063_SAMPLE_RATE_RO		0x0204
#define ZL38063_SYS_CRL_GLAG		0x0206
#define ZL38063_SIN_SELECT			0x0208
/*	0x00 No Input Selected
	0x01 MIC1
	0x02 MIC2
	0x03 MIC3
	0x04 MIC4
	0x05 I2S-1L/TDMA-11
	0x06 I2S-1R/TDMA-21
	0x07 TDMA-31
	0x08 TDMA-41
	0x09 I2S-2L/TDMB-11
	0x0A I2S-2R/TDMB-21
	0x0B TDMB-31
	0x0C TDMB-41
	0x0D AEC Rout
	0x0E AEC Sout
	0x0F Tone Generator 1
	0x10 Tone Generator 2
	0x11-0xFF Reserved*/
#define ZL38063_CRCP_SRC_DAC1		0x0210
#define ZL38063_CRCP_SRC_DAC2		0x0212
#define ZL38063_CRCP_SRC_TDMA1		0x0214
#define ZL38063_CRCP_SRC_TDMA2		0x0216
#define ZL38063_CRCP_SRC_TDMA3		0x0218
#define ZL38063_CRCP_SRC_TDMA4		0x021A
#define ZL38063_CRCP_SRC_TDMB1		0x021C
#define ZL38063_CRCP_SRC_TDMB2		0x021E
#define ZL38063_CRCP_SRC_TDMB3		0x0220
#define ZL38063_CRCP_SRC_TDMB4		0x0222
#define ZL38063_CRCP_SRC_AECSIN		0x0224
#define ZL38063_CRCP_SRC_AECRSN		0x0226
#define ZL38063_CRCP_SRC_AECSIN2	0x0228
#define ZL38063_CRCP_SRC_AECSIN3	0x022A

#define ZL38063_CRCP_GAIN_DAC1		0x0238
#define ZL38063_CRCP_GAIN_DAC2		0x023A
#define ZL38063_CRCP_GAIN_TDMA1		0x023C
#define ZL38063_CRCP_GAIN_TDMA2		0x023E
#define ZL38063_CRCP_GAIN_TDMA3		0x0240
#define ZL38063_CRCP_GAIN_TDMA4		0x0242
#define ZL38063_CRCP_GAIN_TDMB1		0x0244
#define ZL38063_CRCP_GAIN_TDMB2		0x046
#define ZL38063_CRCP_GAIN_TDMB3		0x0248
#define ZL38063_CRCP_GAIN_TDMB4		0x024A
#define ZL38063_CRCP_GAIN_AECSIN	0x024C
#define ZL38063_CRCP_GAIN_AECRSN	0x024E
#define ZL38063_CRCP_GAIN_AECSIN2	0x0250
#define ZL38063_CRCP_GAIN_AECSIN3	0x0252

#define ZL38063_TDMA_FMT			0x0260
#define ZL38063_TDMA_CLK			0x0262
#define ZL38063_TDMA_PCM_CONF		0x0264
#define ZL38063_TDMA_CONF_CH1		0x0268
#define ZL38063_TDMA_CONF_CH2		0x026A
#define ZL38063_TDMA_CONF_CH3		0x026C
#define ZL38063_TDMA_CONF_CH4		0x026E
#define ZL38063_TDMB_FMT			0x0278
#define ZL38063_TDMB_CLK			0x027A
#define ZL38063_TDMB_PCM_CONF		0x027C
#define ZL38063_TDMB_CONF_CH1		0x0280
#define ZL38063_TDMB_CONF_CH2		0x0282
#define ZL38063_TDMB_CONF_CH3		0x0284
#define ZL38063_TDMB_CONF_CH4		0x0286

#define ZL38063_DAC1_CONF			0x02A0
#define ZL38063_DAC2_CONF			0x02A2
#define ZL38063_MIC_EN				0x02B0
#define ZL38063_MIC_GAIN			0x02B2
#define ZL38063_TONE_FREQ_1A		0x02B8
#define ZL38063_TONE_GAIN_1A		0x02BA
#define ZL38063_TONE_FREQ_1B		0x02BC
#define ZL38063_TONE_GAIN_1B		0x02BE
#define ZL38063_TONE_FREQ_2A		0x02C0
#define ZL38063_TONE_GAIN_2A		0x02C2
#define ZL38063_TONE_FREQ_2B		0x02C4
#define ZL38063_TONE_GAIN_2B		0x02C6

#define ZL38063_GPIO_FIXED_FUN_EN	0x02D8
#define ZL38063_GPIO_DATA			0x02DA
#define ZL38063_GPIO_DIR			0x02DC
#define ZL38063_GPIO_DS				0x02DE
#define ZL38063_GPIO_PULL_M			0x02E0		//0 Tristate 1 Pull Up  2 Pull Down  3 Keeper
#define ZL38063_GPIO_PULL_L			0x02E2
#define ZL38063_GPIO_EV_MASKP		0x02E4
#define ZL38063_GPIO_EV_MASKN		0x02E6

#define ZL38063_GPIO0_ADDR_P		0x02E8
#define ZL38063_GPIO0_ANDMASK_P		0x02EA
#define ZL38063_GPIO0_ORMASK_P		0x02EC
#define ZL38063_GPIO0_ADDR_N		0x02EE
#define ZL38063_GPIO0_ANDMASK_N		0x02F0
#define ZL38063_GPIO0_ORMASK_N		0x02F2
#define ZL38063_GPIO1_ADDR_P		0x02F4
#define ZL38063_GPIO1_ANDMASK_P		0x02F6
#define ZL38063_GPIO1_ORMASK_P		0x02F8
#define ZL38063_GPIO1_ADDR_N		0x02FA
#define ZL38063_GPIO1_ANDMASK_N		0x02FC
#define ZL38063_GPIO1_ORMASK_N		0x02FE

/*2 Audio Processing Configuration*/
#define ZL38063_AEC_CTL0			0x0300
#define ZL38063_AEC_CTL1			0x0302
#define ZL38063_ROUT_GAIN_CTL		0x030A
#define ZL38063_USR_GAIN_CTL		0x030C
#define ZL38063_RIN_HP_CUTOFF		0x0310
#define ZL38063_AEC_CNOISE			0x0324
#define ZL38063_NLP_SDT_TH			0x0326
#define ZL38063_LONG_SDT_TH			0x0328				//duration = silLen / 32
#define ZL38063_NOISE_TH			0x032A
#define ZL38063_ERLE_TH				0x032E
#define ZL38063_AEC_MAX_ADSTEP		0x0330
#define ZL38063_DTLOW_LEVEL_ADSTEP	0x0336
#define ZL38063_DT_HANDOVER_TIME	0x0338
#define ZL38063_DIV_DET_TH			0x033C
#define ZL38063_DT_PRESISTENT		0x033E
#define ZL38063_AEC_FILTC0_LEN		0x0342
#define ZL38063_AEC_FLAT_DELAY		0x0344
#define ZL38063_AEC_PROFILE_CTL		0x0346
#define ZL38063_AEC_PROFILE7		0x0348
#define ZL38063_AEC_PROFILE6		0x034A
#define ZL38063_AEC_PROFILE5		0x034C
#define ZL38063_AEC_PROFILE4		0x034E
#define ZL38063_AEC_PROFILE3		0x0350
#define ZL38063_AEC_PROFILE2		0x0352
#define ZL38063_AEC_PROFILE1		0x0354
#define ZL38063_AEC_PROFILE0		0x0356
#define ZL38063_AEC_DT_FILTER		0x035C



#define ZL38063_SWITCH_ATTEN_HANDOVER	0x0370
#define ZL38063_SWITCH_ATTEN_STARTUP	0x0372
#define ZL38063_AGNCL_CTL				0x0374



#define ZL38063_SWITCH_ATTEN		0x0376

#define ZL38063_LEC_FILTTER_LEN		0x0382

#define ZL38063_NOISE_WIN_LEN		0x03A2
#define ZL38063_NOISE_HINLP_TH		0x03A8
#define ZL38063_ECHO_RESIDUAL		0x03B2
#define ZL38063_NOISE_REDUCE_LEV	0x03C8
#define ZL38063_CNOISE_WIN_LEN		0x03CA
#define ZL38063_CNOISE_DET_TH		0x03CC
#define ZL38063_NLAEC_AGGRE_TH		0x03CE


#define ZL38063_NLAEC_FMIN			0x03D2
#define ZL38063_NLAEC_STA_TH		0x03D6
#define ZL38063_NLAEC_DT_TH			0x03DE
#define ZL38063_NLAEC_EPC_TH		0x03E0
#define ZL38063_NLAEC_DTD_TH		0x03E2
#define ZL38063_NLAEC_DTNLP_TH		0x03E4

#define ZL38063_NBSD_RIN_DETDEB		0x03E6
#define ZL38063_NBSD_RIN_RELDEB		0x03E8
#define ZL38063_NBSD_RIN_MIN_TH		0x03EA
#define ZL38063_NBSD_RIN_MINCAN_TH	0x03EC
#define ZL38063_NBSD_SIN_DETDEB		0x03EE
#define ZL38063_NBSD_SIN_RELDEB		0x03F0
#define ZL38063_NBSD_SIN_MIN_TH		0x03F2
#define ZL38063_NBSD_SIN_MINCAN_TH	0x03F4

#define ZL38063_HD_CTL				0x03F6
#define ZL38063_HD_RIN_ACT_TH		0x03F8
#define ZL38063_HD_HOLD_TIME		0x03FA
#define ZL38063_AEC_NLP_MOD			0x03FC
#define ZL38063_AEC_DT_TH			0x03FE


/*3 Tone Generators */
#define ZL38063_TONE_GEN_CTL		0x0400
#define ZL38063_TONE_BURST_LEN		0x0402
#define ZL38063_TONE_HL_GAIN		0x0404
#define ZL38063_TONE_FRE_A0			0x0408
#define ZL38063_TONE_FRE_A1			0x040A
#define ZL38063_TONE_FRE_A2			0x040C
#define ZL38063_TONE_FRE_A3			0x040E
#define ZL38063_TONE_FRE_A4			0x0410
#define ZL38063_TONE_FRE_A5			0x0412
#define ZL38063_TONE_FRE_A6			0x0414
#define ZL38063_TONE_FRE_A7			0x0416
#define ZL38063_TONE_FRE_B0			0x0418
#define ZL38063_TONE_FRE_B1			0x041A
#define ZL38063_TONE_FRE_B2			0x041C
#define ZL38063_TONE_FRE_B3			0x041E
#define ZL38063_TONE_FRE_B4			0x0420
#define ZL38063_TONE_FRE_B5			0x0422
#define ZL38063_TONE_FRE_B6			0x0424
#define ZL38063_TONE_FRE_B7			0x0426
#define ZL38063_TONE_RING_HTIME		0x0428
#define ZL38063_TONE_RING_LTIME		0x042A


/* Beamformer Registers*/

#define ZL38063_BF_MIC_NUM			0x04C0
#define ZL38063_BF_MIC_DISTANCE		0x04C2
#define ZL38063_BF_STEER_ANGLE		0x04C4
#define ZL38063_BF_HALF_WIDTH		0x04C6
#define ZL38063_BF_OOB_GAIN_MAX		0x04C8
#define ZL38063_BF_OOB_GAIN_MIX		0x04CA
#define ZL38063_SOUND_LOC_CTL		0x04D0
#define ZL38063_SOUND_LOC_THD		0x04D2

#define	MIC_CONF_TRIANG				0x0010
#define	MIC_CONF_LINEAR				0x0000
#define	MIC_NUM_3					0x0003
#define	MIC_NUM_2					0x0002


/*4 Rout Parametric Equalizers*/
#define ZL38063_ROUT_EQ_ENABLE		0x0500
#define ZL38063_ROUT_EQ_F1_TYPE		0x0504
#define ZL38063_ROUT_EQ_F1_FRE		0x0506
#define ZL38063_ROUT_EQ_F1_Q		0x0508
#define ZL38063_ROUT_EQ_F1_GAIN		0x050A
#define ZL38063_ROUT_EQ_F2_TYPE		0x050C
#define ZL38063_ROUT_EQ_F2_FRE		0x050E
#define ZL38063_ROUT_EQ_F2_Q		0x0510
#define ZL38063_ROUT_EQ_F2_GAIN		0x0512
#define ZL38063_ROUT_EQ_F3_TYPE		0x0514
#define ZL38063_ROUT_EQ_F3_FRE		0x0516
#define ZL38063_ROUT_EQ_F3_Q		0x0518
#define ZL38063_ROUT_EQ_F3_GAIN		0x051A
#define ZL38063_ROUT_EQ_F4_TYPE		0x051C
#define ZL38063_ROUT_EQ_F4_FRE		0x051E
#define ZL38063_ROUT_EQ_F4_Q		0x0520
#define ZL38063_ROUT_EQ_F4_GAIN		0x0522
#define ZL38063_ROUT_EQ_F5_TYPE		0x0524
#define ZL38063_ROUT_EQ_F5_FRE		0x0526
#define ZL38063_ROUT_EQ_F5_Q		0x0528
#define ZL38063_ROUT_EQ_F5_GAIN		0x052A
#define ZL38063_ROUT_EQ_F6_TYPE		0x052C
#define ZL38063_ROUT_EQ_F6_FRE		0x052E
#define ZL38063_ROUT_EQ_F6_Q		0x0530
#define ZL38063_ROUT_EQ_F6_GAIN		0x0532
#define ZL38063_ROUT_EQ_F7_TYPE		0x0534
#define ZL38063_ROUT_EQ_F7_FRE		0x0536
#define ZL38063_ROUT_EQ_F7_Q		0x0538
#define ZL38063_ROUT_EQ_F7_GAIN		0x053A
#define ZL38063_ROUT_EQ_F8_TYPE		0x053C
#define ZL38063_ROUT_EQ_F8_FRE		0x053E
#define ZL38063_ROUT_EQ_F8_Q		0x0540
#define ZL38063_ROUT_EQ_F8_GAIN		0x0542

/*5 Sin Parametric Equalizers*/
#define ZL38063_SIN_EQ_ENABLE		0x0544
#define ZL38063_SIN_EQ_F1_TYPE		0x0548
#define ZL38063_SIN_EQ_F1_FRE		0x054A
#define ZL38063_SIN_EQ_F1_Q			0x054C
#define ZL38063_SIN_EQ_F1_GAIN		0x054E
#define ZL38063_SIN_EQ_F2_TYPE		0x0550
#define ZL38063_SIN_EQ_F2_FRE		0x0552
#define ZL38063_SIN_EQ_F2_Q			0x0554
#define ZL38063_SIN_EQ_F2_GAIN		0x0556
#define ZL38063_SIN_EQ_F3_TYPE		0x0558
#define ZL38063_SIN_EQ_F3_FRE		0x055A
#define ZL38063_SIN_EQ_F3_Q			0x055C
#define ZL38063_SIN_EQ_F3_GAIN		0x055E
#define ZL38063_SIN_EQ_F4_TYPE		0x0560
#define ZL38063_SIN_EQ_F4_FRE		0x0562
#define ZL38063_SIN_EQ_F4_Q			0x0564
#define ZL38063_SIN_EQ_F4_GAIN		0x0566
#define ZL38063_SIN_EQ_F5_TYPE		0x0568
#define ZL38063_SIN_EQ_F5_FRE		0x056A
#define ZL38063_SIN_EQ_F5_Q			0x056C
#define ZL38063_SIN_EQ_F5_GAIN		0x056E
#define ZL38063_SIN_EQ_F6_TYPE		0x0570
#define ZL38063_SIN_EQ_F6_FRE		0x0572
#define ZL38063_SIN_EQ_F6_Q			0x0574
#define ZL38063_SIN_EQ_F6_GAIN		0x0576
#define ZL38063_SIN_EQ_F7_TYPE		0x0578
#define ZL38063_SIN_EQ_F7_FRE		0x057A
#define ZL38063_SIN_EQ_F7_Q			0x057C
#define ZL38063_SIN_EQ_F7_GAIN		0x057E
#define ZL38063_SIN_EQ_F8_TYPE		0x0580
#define ZL38063_SIN_EQ_F8_FRE		0x0582
#define ZL38063_SIN_EQ_F8_Q			0x0584
#define ZL38063_SIN_EQ_F8_GAIN		0x0586

/*6 Microphone Selection Control*/
#define ZL38063_MIC_EN_STREAM_MASK	0x05CE
#define ZL38063_MIC_ENERGY_FIL_TIME	0x05D0
#define ZL38063_MIC_ENERGY_TH		0x05D2
#define ZL38063_MIC_ENERGY_HYST		0x05D4
#define ZL38063_MIC_SWI_LOCK_TIME	0x05D6
#define ZL38063_MIC_MULT_GAIN		0x05DA

/*7 Compressor / Limiter / Expander*/
#define ZL38063_ROUT_CLE_CTL		0x0602
#define ZL38063_ROUT_CLE_MKUP		0x0604
#define ZL38063_ROUT_CLE_COTH		0x0608
#define ZL38063_ROUT_CLE_CORATIO	0x060A
#define ZL38063_ROUT_CLE_COATTACK	0x060C
#define ZL38063_ROUT_CLE_CODECAY	0x060E
#define ZL38063_ROUT_CLE_EXPTH		0x0610
#define ZL38063_ROUT_CLE_EXRATIO	0x0612
#define ZL38063_ROUT_CLE_EXATTACK	0x0614
#define ZL38063_ROUT_CLE_EXDECAY	0x0616
#define ZL38063_ROUT_CLE_LMTH		0x0618
#define ZL38063_ROUT_CLE_LMRATIO	0x061A
#define ZL38063_ROUT_CLE_LMATTACK	0x061C
#define ZL38063_ROUT_CLE_LMDECAY	0x061E

#define ZL38063_SOUT_CLE_CTL		0x0622
#define ZL38063_SOUT_CLE_MKUP		0x0624
#define ZL38063_SOUT_CLE_COMTH		0x0628
#define ZL38063_SOUT_CLE_CRATIO		0x062A
#define ZL38063_SOUT_CLE_CATTACK	0x062C
#define ZL38063_SOUT_CLE_CDECAY		0x062E
#define ZL38063_SOUT_CLE_EXPTH		0x0630
#define ZL38063_SOUT_CLE_EXRATIO	0x0632
#define ZL38063_SOUT_CLE_EXATTACK	0x0634
#define ZL38063_SOUT_CLE_EXDECAY	0x0636
#define ZL38063_SOUT_CLE_LMTH		0x0638
#define ZL38063_SOUT_CLE_LMRATIO	0x063A
#define ZL38063_SOUT_CLE_LMATTACK	0x063C
#define ZL38063_SOUT_CLE_LMDECAY	0x063E

#define ZL38063_SIN_CLE_CTL			0x0642
#define ZL38063_SIN_CLE_LMTH		0x0658
#define ZL38063_SIN_CLE_LMRATIO		0x065A
#define ZL38063_SIN_CLE_LMATTACK	0x065C
#define ZL38063_SNI_CLE_LMDECAY		0x065E
#define ZL38063_ROUTSIM_REF_LEV		0x0662
#define ZL38063_ROUTSIM_CTL			0x0664
#define ZL38063_ROUT_PEAK_EVTH		0x0666
#define ZL38063_ROUT_MAX_RMS		0x0668
#define ZL38063_ROUT_PEAK_LEV		0x066A
#define ZL38063_SOUT_CLE_DUCK_TH	0x0670
#define ZL38063_SOUTSIM_REF_TH		0x0672
#define ZL38063_SOUTSIM_CTL			0x0674
#define ZL38063_SOUT_PEAK_EVTH		0x0676
#define ZL38063_SOUT_MAX_RMS		0x0678
#define ZL38063_SOUT_PEAK_LEV		0x067A
#define ZL38063_SIN_PEAK_EVTH		0x0686
#define ZL38063_SIN_MAX_RMS			0x0688


/*****Configuration Record*****/
#define ZL38063_MAX_REG			0x07FF


/*****Slave Register Configuration*****/

/* Each register in this range mirrors the configuration records
from the respective registers in the range of 0xX00 ¨C0xXFF
(0x200 = 0x800, 0x201 = 0x801, etc.).
These configuration records are applied to Slave 1 in a multi-chip
configuration.

->1 0x800-0x8FF
->2 0x900-0x9FF
->3 0xA00-0xAFF
->4 0xB00-0xBFF

*/

/*****Slave Register Configuration*****/


/*
static struct reg_format zl38063_def_reg[] = {
		{ 0x00,0x00 },

};
*/

u16 need_reset[] ={
	ZL38063_PATH_CPEN,
	ZL38063_SAMPLE_RATE_RO,
	ZL38063_SYS_CRL_GLAG,
	ZL38063_SIN_SELECT,
	ZL38063_CRCP_SRC_DAC1,
	ZL38063_CRCP_SRC_DAC2,
	ZL38063_CRCP_SRC_TDMA1,
	ZL38063_CRCP_SRC_TDMA2,
	ZL38063_CRCP_SRC_TDMA3,
	ZL38063_CRCP_SRC_TDMA4,
	ZL38063_CRCP_SRC_TDMB1,
	ZL38063_CRCP_SRC_TDMB2,
	ZL38063_CRCP_SRC_TDMB3,
	ZL38063_CRCP_SRC_TDMB4,
	ZL38063_CRCP_SRC_AECSIN,
	ZL38063_CRCP_SRC_AECRSN,
	ZL38063_CRCP_SRC_AECSIN2,
	ZL38063_CRCP_SRC_AECSIN3,
	ZL38063_TDMA_FMT,
	ZL38063_TDMA_CLK,
	ZL38063_TDMA_PCM_CONF,
	ZL38063_TDMA_CONF_CH1,
	ZL38063_TDMA_CONF_CH2,
	ZL38063_TDMA_CONF_CH3,
	ZL38063_TDMA_CONF_CH4,
	ZL38063_TDMB_FMT,
	ZL38063_TDMB_CLK,
	ZL38063_TDMB_PCM_CONF,
	ZL38063_TDMB_CONF_CH1,
	ZL38063_TDMB_CONF_CH2,
	ZL38063_TDMB_CONF_CH3,
	ZL38063_TDMB_CONF_CH4,
	ZL38063_DAC1_CONF,
	ZL38063_DAC2_CONF,
	ZL38063_MIC_EN,
	ZL38063_MIC_GAIN,
	ZL38063_GPIO_FIXED_FUN_EN,
	ZL38063_GPIO_EV_MASKP,
	ZL38063_GPIO_EV_MASKN,
	ZL38063_GPIO0_ADDR_P,
	ZL38063_GPIO0_ANDMASK_P,
	ZL38063_GPIO0_ORMASK_P,
	ZL38063_GPIO0_ADDR_N,
	ZL38063_GPIO0_ANDMASK_N,
	ZL38063_GPIO0_ORMASK_N,
	ZL38063_GPIO1_ADDR_P,
	ZL38063_GPIO1_ANDMASK_P,
	ZL38063_GPIO1_ORMASK_P,
	ZL38063_GPIO1_ADDR_N,
	ZL38063_GPIO1_ANDMASK_N,
	ZL38063_GPIO1_ORMASK_N,
	ZL38063_AEC_CNOISE,
	ZL38063_DTLOW_LEVEL_ADSTEP,
	ZL38063_DT_HANDOVER_TIME,
	ZL38063_DIV_DET_TH,
	ZL38063_AEC_FILTC0_LEN,
	ZL38063_AEC_FLAT_DELAY,
	ZL38063_AEC_PROFILE_CTL,
	ZL38063_AEC_PROFILE7,
	ZL38063_AEC_PROFILE6,
	ZL38063_AEC_PROFILE5,
	ZL38063_AEC_PROFILE4,
	ZL38063_AEC_PROFILE3,
	ZL38063_AEC_PROFILE2,
	ZL38063_AEC_PROFILE1,
	ZL38063_AEC_PROFILE0,
	ZL38063_CNOISE_DET_TH,
	ZL38063_LEC_FILTTER_LEN,
	ZL38063_SWITCH_ATTEN_HANDOVER,
	ZL38063_SWITCH_ATTEN_STARTUP,
	ZL38063_AGNCL_CTL,
	ZL38063_AEC_DT_FILTER,
	ZL38063_AEC_NLP_MOD,
	ZL38063_AEC_DT_TH,
	ZL38063_SIN_EQ_ENABLE,
	ZL38063_SIN_EQ_F1_TYPE,
	ZL38063_SIN_EQ_F1_FRE,
	ZL38063_SIN_EQ_F1_Q,
	ZL38063_SIN_EQ_F1_GAIN,
	ZL38063_SIN_EQ_F2_TYPE,
	ZL38063_SIN_EQ_F2_FRE,
	ZL38063_SIN_EQ_F2_Q,
	ZL38063_SIN_EQ_F2_GAIN,
	ZL38063_SIN_EQ_F3_TYPE,
	ZL38063_SIN_EQ_F3_FRE,
	ZL38063_SIN_EQ_F3_Q,
	ZL38063_SIN_EQ_F3_GAIN,
	ZL38063_SIN_EQ_F4_TYPE,
	ZL38063_SIN_EQ_F4_FRE,
	ZL38063_SIN_EQ_F4_Q,
	ZL38063_SIN_EQ_F4_GAIN,
	ZL38063_SIN_EQ_F5_TYPE,
	ZL38063_SIN_EQ_F5_FRE,
	ZL38063_SIN_EQ_F5_Q,
	ZL38063_SIN_EQ_F5_GAIN,
	ZL38063_SIN_EQ_F6_TYPE,
	ZL38063_SIN_EQ_F6_FRE,
	ZL38063_SIN_EQ_F6_Q,
	ZL38063_SIN_EQ_F6_GAIN,
	ZL38063_SIN_EQ_F7_TYPE,
	ZL38063_SIN_EQ_F7_FRE,
	ZL38063_SIN_EQ_F7_Q,
	ZL38063_SIN_EQ_F7_GAIN,
	ZL38063_SIN_EQ_F8_TYPE,
	ZL38063_SIN_EQ_F8_FRE,
	ZL38063_SIN_EQ_F8_Q,
	ZL38063_SIN_EQ_F8_GAIN,
	ZL38063_SOUT_CLE_CTL,
	ZL38063_SOUT_CLE_MKUP,
	ZL38063_SOUT_CLE_COMTH,
	ZL38063_SOUT_CLE_CRATIO,
	ZL38063_SOUT_CLE_CATTACK,
	ZL38063_SOUT_CLE_CDECAY,
	ZL38063_SOUT_CLE_EXPTH,
	ZL38063_SOUT_CLE_EXRATIO,
	ZL38063_SOUT_CLE_EXATTACK,
	ZL38063_SOUT_CLE_EXDECAY,
	ZL38063_SOUT_CLE_LMTH,
	ZL38063_SOUT_CLE_LMRATIO,
	ZL38063_SOUT_CLE_LMATTACK,
	ZL38063_SOUT_CLE_LMDECAY,
	ZL38063_SOUT_CLE_DUCK_TH,
	ZL38063_SOUTSIM_REF_TH,
	ZL38063_SOUTSIM_CTL,
	ZL38063_SOUT_PEAK_EVTH,
	ZL38063_SOUT_MAX_RMS,
	ZL38063_SIN_PEAK_EVTH,
	ZL38063_SIN_MAX_RMS
};

#define NEED_RESET_NUM 127

#endif /* __MICROSEMI_SPIS_TW_H */
