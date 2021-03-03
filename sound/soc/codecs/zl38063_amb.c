/*
 * zl38063.c  --  zl38063 ALSA Soc Audio driver
 *
 * Copyright 2014 Microsemi Inc.
 *   History:
 *        2015/02/15  - created driver that combines a CHAR,
						   an ASLSA and a SPI/I2C driver in on single module
 *        2015/03/24  - Added compatibility to linux kernel 2.6.x to 3.x.x
 *                      - Verified with Linux kernels 2.6.38, and 3.10.50
 *        2015/09/30  - Added option to compile the firmware/config in c format with the kernel
 *        2015/10/7   - Updated the save config to flash to allow saving multiple config to flash
 * This program is free software you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option)any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/netlink.h>

#include <linux/delay.h>
#include <linux/pm.h>

#include <linux/sched.h>

#include <linux/timer.h>

#include <net/sock.h>



#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "zl38063_amb.h"

#include <linux/firmware.h>

#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>

#ifdef ZL38063_SPI_IF
#include <linux/spi/spi.h>
#include <plat/spi.h>

#endif
#ifdef ZL38063_I2C_IF
#include <linux/i2c.h>
#endif

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>

#include <linux/mutex.h>




//#define ZL38063_BASE_DEBUG
#undef ZL38063_NL_DEBUG

#ifdef ZL38063_BASE_DEBUG
#define TW_DEBUG1(s, args...) \
	printk("%s %d: "s, __func__, __LINE__, ##args);
#else
#define TW_DEBUG1(s, args...)
#endif

#ifdef ZL38063_NL_DEBUG
#define TW_DEBUG2(s, args...) \
	printk("%s %d: "s, __func__, __LINE__, ##args);
#else
#define TW_DEBUG2(s, args...)
#endif


/*TWOLF HBI ACCESS MACROS-------------------------------*/
#define HBI_PAGED_READ(offset,length) \
	((u16)(((u16)(offset) << 8) | (length)))
#define HBI_DIRECT_READ(offset,length) \
	((u16)(0x8000 | ((u16)(offset) << 8) | (length)))
#define HBI_PAGED_WRITE(offset,length) \
	((u16)(HBI_PAGED_READ(offset,length) | 0x0080))
#define HBI_DIRECT_WRITE(offset,length) \
	((u16)(HBI_DIRECT_READ(offset,length) | 0x0080))
#define HBI_GLOBAL_DIRECT_WRITE(offset,length) \
	((u16)(0xFC00 | ((offset) << 4) | (length)))
#define HBI_CONFIGURE(pinConfig) \
	((u16)(0xFD00 | (pinConfig)))
#define HBI_SELECT_PAGE(page) \
	((u16)(0xFE00 | (page)))
#define HBI_NO_OP \
	((u16)0xFFFF)
//#define REG_SLAVE_TRN(reg) ((u16)(0x0800 | (reg)))


/*HBI access type*/
#define TWOLF_HBI_READ 0
#define TWOLF_HBI_WRITE 1
/*HBI address type*/
#define TWOLF_HBI_DIRECT 2
#define TWOLF_HBI_PAGED  3
struct zlFirmware AecFirmware,AsrFirmware,DoaFirmware;		//ASR is ZLS38063.1_E0_10_0_firmware.s3   AEC is ZL38063_11App.s3


static char *fw_path ;
module_param(fw_path, charp, 0664);
MODULE_PARM_DESC(fw_path, "full name with path about the firmware");

static char *cr_path ;
module_param(cr_path, charp, 0664);
MODULE_PARM_DESC(cr_path, "full name with path about the configuration record");

static unsigned structural_offset =90;
module_param(structural_offset, uint, 0664);
MODULE_PARM_DESC(structural_offset, "the degree offset between two MIC's centerline and VIN0");

static unsigned twHBImaxTransferSize =(MAX_TWOLF_ACCESS_SIZE_IN_BYTES);
module_param(twHBImaxTransferSize, uint, S_IRUGO);
MODULE_PARM_DESC(twHBImaxTransferSize, "total number of data bytes >= 264");


static unsigned boot_to_BootMode = 0;
module_param(boot_to_BootMode, uint, 0644);
MODULE_PARM_DESC(boot_to_BootMode, "boot to Boot Mode, just for AEC tuning");

static unsigned doa = 0;
module_param(doa, uint, 0644);
MODULE_PARM_DESC(doa, "1 to enable master 38063 DOA function");


/* driver private data */
struct zl38063 {
#ifdef ZL38063_SPI_IF
	struct spi_device	*spi;
#endif
#ifdef ZL38063_I2C_IF
	struct i2c_client   *i2c;
#endif
	u8  *pData;
	int sysclk_rate;
	struct mutex		lock;

	u8 init_done;
	int pwr_pin;
	unsigned int pwr_active;
	int rst_pin;
	unsigned int rst_active;

	struct task_struct	*kthread;
	struct sock *nl_sock;
	u8 nl_init;
	u8 nl_port;
	u16 degree;
	enum FW_LABEL fw_label;
	u16 pid;
	struct beamforming_conf beamforming;
	u16 curr_FWID;
} *zl38063_priv,*zl38063_sla;

static unsigned bd_type = 0;


/* if mutual exclusion is required for your particular platform
 * then add mutex lock/unlock to this driver
 */

//static DEFINE_MUTEX(lock);
//static DEFINE_MUTEX(zl38063_list_lock);
//static LIST_HEAD(zl38063_list);


/* if mutual exclusion is required for your particular platform
 * then add mutex lock/unlock to this driver
 */

static void zl38063EnterCritical(void)
{
	mutex_lock(&zl38063_priv->lock);
}

static void zl38063ExitCritical(void)
{
	mutex_unlock(&zl38063_priv->lock);
}


/*  write up to 252 bytes data */
/*  zl38063_nbytes_wr()- rewrite one or up to 252 bytes to the device
 *  \param[in]     ptwdata     pointer to the data to write
 *  \param[in]     numbytes    the number of bytes to write
 *
  *  return ::status = the total number of bytes transferred successfully
 *                    or a negative error code
 */
static int
zl38063_nbytes_wr(struct zl38063 *zl38063,
				  int numbytes, u8 *pData)
{

	int status;
#ifdef ZL38063_SPI_IF
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len = numbytes,
		.tx_buf = pData,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	status = spi_sync(zl38063->spi, &msg);
#endif
#ifdef ZL38063_I2C_IF
	status = i2c_master_send(zl38063->i2c, pData, numbytes);
#endif
	if (status < 0)
		return -EFAULT;

	return 0;
}

/* ZL38063_hbi_rd16()- Decode the 16-bit T-WOLF Regs Host address into
 * page, offset and build the 16-bit command acordingly to the access type.
 * then read the 16-bit word data and store it in pdata
 *  \param[in]
 *                .addr     the 16-bit HBI address
 *                .pdata    pointer to the data buffer read or write
 *
 *  return ::status
 */
static int zl38063_hbi_rd16(struct zl38063 *zl38063,
							 u16 addr, u16 *pdata)
{

	u16 cmd;
	u8 page;
	u8 offset;
	u8 i = 0;
	u8 buf[4] = {0, 0, 0, 0};

	int status =0;
#ifdef ZL38063_I2C_IF
	struct  i2c_msg     msg[2];
#endif

	page = addr >> 8;
	offset = (addr & 0xFF)>>1;

	if (page == 0) /*Direct page access*/
	{
		cmd = HBI_DIRECT_READ(offset, 0);/*build the cmd*/
		buf[i++] = (cmd >> 8) & 0xFF;
		buf[i++] = (cmd & 0xFF);

	} else { /*indirect page access*/
		if (page != 0xFF) {
			page  -=  1;
		}
		cmd = HBI_SELECT_PAGE(page);
		i = 0;
		/*select the page*/
		buf[i++] = (cmd >> 8) & 0xFF;
		buf[i++] = (cmd & 0xFF);
		cmd = HBI_PAGED_READ(offset, 0); /*build the cmd*/
		buf[i++] = (cmd >> 8) & 0xFF;
		buf[i++] = (cmd & 0xFF);
	}
	/*perform the HBI access*/
	//printk("READ:: PAGE = %d, addr = 0x%x    buf %d ---  %d cmd=%x\n", page, addr,buf[0],buf[1],cmd);

#ifdef ZL38063_SPI_IF
	status = spi_write_then_read(zl38063->spi, buf, i, buf, 2);
#endif
#ifdef ZL38063_I2C_IF
	memset(msg,0,sizeof(msg));

	/* Make write msg 1st with write command */
	msg[0].addr    = zl38063->i2c->addr;
	msg[0].len     = i;
	msg[0].buf     = buf;
//    TW_DEBUG2("i2c addr = 0x%04x\n", msg[0].addr);
//    TW_DEBUG2("numbytes:%d, cmdword = 0x%02x, %02x\n", numbytes, msg[0].buf[0], msg[0].buf[1]);
	/* Make read msg */
	msg[1].addr = zl38063->i2c->addr;
	msg[1].len  = 2;
	msg[1].buf  = buf;
	msg[1].flags = I2C_M_RD;
	status = i2c_transfer(zl38063->i2c->adapter, msg, 2);
#endif
	if (status <0)
		return status;

	*pdata = (buf[0]<<8) | buf[1] ; /* Byte_HI, Byte_LO */


	//TW_DEBUG1(" read get 0x%x  0x%x %u \n", buf[0],buf[1],*pdata);
	return 0;
}

/* ZL38063_hbi_wr16()- this function is used for single word access.
 * It decodes the 16-bit T-WOLF Regs Host address into
 * page, offset and build the 16-bit command acordingly to the access type.
 * then write the command and data to the device
 *  \param[in]
 *                .addr      the 16-bit HBI address
 *                .data      the 16-bit word to write
 *
 *  return ::status
 */
static int zl38063_hbi_wr16(struct zl38063 *zl38063,
							 u16 addr, u16 data)
{

	u16 cmd;
	u8 page;
	u8 offset;
	u8 i =0;
	u8 buf[6] = {0, 0, 0, 0, 0, 0};
	int status =0;
	page = addr >> 8;
	offset = (addr & 0xFF)>>1;

	if (page == 0) /*Direct page access*/
	{
		cmd = HBI_DIRECT_WRITE(offset, 0);/*build the cmd*/
		buf[i++] = (cmd >> 8) & 0xFF;
		buf[i++] = (cmd & 0xFF);

	} else { /*indirect page access*/
		if (page != 0xFF) {
			page  -=  1;
		}
		cmd = HBI_SELECT_PAGE(page);
		i = 0;
		/*select the page*/
		buf[i++] = (cmd >> 8) & 0xFF;
		buf[i++] = (cmd & 0xFF);
		cmd = HBI_PAGED_WRITE(offset, 0); /*build the cmd*/
		buf[i++] = (cmd >> 8) & 0xFF;
		buf[i++] = (cmd & 0xFF);
	}
	buf[i++] = (data >> 8) & 0xFF ;
	buf[i++] = (data & 0xFF) ;

	status = zl38063_nbytes_wr(zl38063, i, buf);
	if (status <0)
		return status;
	return 0;
}

/*poll a specific bit of a register for clearance, used for state check
* [paran in] addr: the 16-bit register to poll
* [paran in] bit: the bit position (0-15) to poll
* [paran in] timeout: the if bit is not cleared within timeout exit loop
*/
static int zl38063_monitor_bit(struct zl38063 *zl38063, u16 addr, u8 bit, u16 timeout)
{
	u16 data = 0xBAD;
	do {
		zl38063_hbi_rd16(zl38063, addr, &data);
		msleep(10);
   } while ((((data & (1 << bit))>>bit) > 0) &&  (timeout-- >0));

	if (timeout <= 0) {
		printk("[%s](%d) Operation Mode, in timeout = %d ms\n",__func__, __LINE__, timeout*10);
		return -1;
	}
	return 0;
}

/* zl38063_reset(): use this function to reset the device.
 *
 *
 * Input Argument: mode  - the supported reset modes:
 *         VPROC_ZL38063_RST_HARDWARE_ROM,
 *         VPROC_ZL38063_RST_HARDWARE_ROM,
 *         VPROC_ZL38063_RST_SOFT,
 *         VPROC_ZL38063_RST_AEC
 *         VPROC_ZL38063_RST_TO_BOOT
 * Return:  type error code (0 = success, else= fail)
 */
static int zl38063_reset(struct zl38063 *zl38063,
						 u16 mode)
{

	u16 addr = CLK_STATUS_REG;
	u16 data = 0;
	int monitor_bit = -1; /*the bit (range 0 - 15) of that register to monitor*/

	TW_DEBUG1("Reset type %d\n",mode);
	//zl38063EnterCritical();
	/*PLATFORM SPECIFIC code*/
	if (mode  == ZL38063_RST_HARDWARE_RAM) {       /*hard reset*/
		/*hard reset*/
		data = 0x0005;
	} else if (mode == ZL38063_RST_HARDWARE_ROM) {  /*power on reset*/
		/*hard reset*/
		data = 0x0009;
	} else if (mode == ZL38063_RST_AEC) { /*AEC method*/
		addr = 0x0300;
		data = 0x0001;
		monitor_bit = 0;
	} else if (mode == ZL38063_RST_SOFTWARE) { /*soft reset*/
		addr = 0x0006;
		data = 0x0002;
		monitor_bit = 1;
	} else if (mode == ZL38063_RST_TO_BOOT) { /*reset to bootrom mode*/
		data = 0x0001;
	} else {
		TW_DEBUG1("Invalid reset type\n");
		return -EINVAL;
	}
	if (zl38063_hbi_wr16(zl38063, addr, data) < 0)
		return -EFAULT;
	msleep(50); /*wait for the HBI to settle*/

	if (monitor_bit >= 0) {
		if (zl38063_monitor_bit(zl38063, addr, monitor_bit, 1000) < 0)
		   return -EFAULT;
	}
	//zl38063ExitCritical();

	return 0;
}


/* tw_mbox_acquire(): use this function to
 *   check whether the host or the device owns the mailbox right
 *
 * Input Argument: None
 * Return: error code (0 = success, else= fail)
 */
static int zl38063_mbox_acquire(struct zl38063 *zl38063)
{

	int status =0;
   /*Check whether the host owns the command register*/
	u16 i=0;
	u16 temp = 0x0BAD;

	for (i = 0; i < TWOLF_MBCMDREG_SPINWAIT; i++) {
		status = zl38063_hbi_rd16(zl38063, ZL38063_SW_FLAGS_REG, &temp);
		if ((status < 0)) {
			TW_DEBUG1("ERROR %d: \n", status);
			return status;
		}
		if (!(temp & ZL38063_SW_FLAGS_CMD)) {break;}
		msleep(10); /*release*/
		TW_DEBUG2("cmdbox =0x%04x timeout count = %d: \n", temp, i);
	}
	TW_DEBUG2("timeout count = %d: \n", i);
	if ((i>= TWOLF_MBCMDREG_SPINWAIT) && (temp != ZL38063_SW_FLAGS_CMD)) {
		return -EBUSY;
	}
	/*read the Host Command register*/
	return 0;
}

/* zl38063_cmdreg_acquire(): use this function to
 *   check whether the last command is completed
 *
 * Input Argument: None
 * Return: error code (0 = success, else= fail)
 */
static int zl38063_cmdreg_acquire(struct zl38063 *zl38063)
{

	int status =0;
	u16 i=0;
	u16 temp = 0x0BAD;

	for (i = 0; i < TWOLF_MBCMDREG_SPINWAIT; i++) {
		status = zl38063_hbi_rd16(zl38063, ZL38063_CMD_REG, &temp);
		if ((status < 0)) {
			TW_DEBUG1("ERROR %d: \n", status);
			return status;
		}
		if (temp == ZL38063_CMD_IDLE) {break;}
		msleep(10); /*wait*/
		TW_DEBUG2("cmdReg =0x%04x timeout count = %d: \n", temp, i);
	}
	TW_DEBUG2("timeout count = %d: \n", i);
	if ((i>= TWOLF_MBCMDREG_SPINWAIT) && (temp != ZL38063_CMD_IDLE)) {
		return -EBUSY;
	}
	/*read the Host Command register*/
	return 0;
}

/* zl38063_write_cmdreg(): use this function to
 *   access the host command register (mailbox access type)
 *
 * Input Argument: cmd - the command to send
 * Return: error code (0 = success, else= fail)
 */

static int zl38063_write_cmdreg(struct zl38063 *zl38063, u16 cmd)
{

	int status = 0;
	u16 buf = cmd;
	/*Check whether the host owns the command register*/
	status = zl38063_mbox_acquire(zl38063);
	if ((status < 0)) {
		printk("ERROR %d: \n", status);
		return status;
	}
	/*write the command into the Host Command register*/
	status = zl38063_hbi_wr16(zl38063, ZL38063_CMD_REG, buf);
	if (status < 0) {
		printk("ERROR %d: \n", status);
		return status;
	}

	if ((cmd & 0x8000) >> 15)
		buf = ZL38063_SW_FLAGS_CMD_NORST;
	else
		buf = ZL38063_SW_FLAGS_CMD;

	/*Release the command reg*/
	buf = ZL38063_SW_FLAGS_CMD;
	status = zl38063_hbi_wr16(zl38063, ZL38063_SW_FLAGS_REG, buf);	//fix me
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}
	return zl38063_cmdreg_acquire(zl38063);
}

/* Write 16bit HBI Register */
/* zl38063_wr16()- write a 16bit word
 *  \param[in]     cmdword  the 16-bit word to write
 *
 *  return ::status
 */
static int zl38063_wr16(struct zl38063 *zl38063,
						u16 cmdword)
{

	u8 buf[2] = {(cmdword >> 8) & 0xFF, (cmdword & 0xFF)};
	int status = 0;
#ifdef ZL38063_SPI_IF
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len = 2,
		.tx_buf = buf,
	};

	spi_message_init(&msg);

	spi_message_add_tail(&xfer, &msg);
	status = spi_sync(zl38063->spi, &msg);
#endif
#ifdef ZL38063_I2C_IF
	status = i2c_master_send(zl38063->i2c, buf, 2);
#endif
	return status;
}

/*To initialize the Twolf HBI interface
 * Param[in] - cmd_config  : the 16-bit HBI init command ored with the
 *                           8-bit configuration.
 *              The command format is cmd_config = 0xFD00 | CONFIG_VAL
 *              CONFIG_VAL: Open Drain(default)
 */
static int zl38063_state_init(struct zl38063 *zl38063, u16 cmd_config)
{
	return zl38063_wr16(zl38063, HBI_CONFIGURE(cmd_config));
}

static int zl38063_cmdresult_check(struct zl38063 *zl38063)
{
	int status = 0;
	u16 buf;
	status = zl38063_hbi_rd16(zl38063, ZL38063_CMD_PARAM_RESULT_REG, &buf);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}

	if (buf !=0) {
		TW_DEBUG1("Command failed...Resultcode = 0x%04x\n", buf);
		return buf;
	}
	return 0;
}

/*stop_fwr_to_bootmode - use this function to stop the firmware currently
 *running
 * And set the device in boot mode
 * \param[in] none
 *
 * \retval ::0 success
 * \retval ::-EINVAL or device error code
 */
static int  zl38063_stop_fwr_to_bootmode(struct zl38063 *zl38063)
{
	return zl38063_write_cmdreg(zl38063, ZL38063_CMD_FWR_STOP);
}

/*start_fwr_from_ram - use this function to start/restart the firmware
 * previously stopped with VprocTwolfFirmwareStop()
 * \param[in] none
 *
 * \retval ::0 success
 * \retval ::-EINVAL or device error code
 */
static int  zl38063_start_fwr_from_ram(struct zl38063 *zl38063)
{
	if (zl38063_write_cmdreg(zl38063, ZL38063_CMD_FWR_GO) < 0) {
		return -EFAULT;
	}
	zl38063->init_done = 1; /*firmware is running in the device*/

	zl38063_hbi_rd16(zl38063,ZL38063_FM_CUR,&zl38063->curr_FWID);		//when download ok,get current FW ID
	TW_DEBUG1("loading FWID %d\n", zl38063->curr_FWID);
	return 0;
}

#ifdef ZL38063_SAVE_FWR_TO_FLASH
/*tw_init_check_flash - use this function to check if there is a flash on board
 * and initialize it
 * \param[in] none
 *
 * \retval ::0 success
 * \retval ::-EINVAL or device error code
 */
static int  zl38063_init_check_flash(struct zl38063 *zl38063)
{
	 /*if there is a flash on board initialize it*/
	return zl38063_write_cmdreg(zl38063, ZL38063_CMD_HOST_FLASH_INIT);
}

/*tw_erase_flash - use this function to erase all firmware image
* and related  config from flash
 * previously stopped with VprocTwolfFirmwareStop()
 * \param[in] none
 *
 * \retval ::0 success
 * \retval ::-EINVAL or device error code
 */
static int  zl38063_erase_flash(struct zl38063 *zl38063)
{

	int status =0;
	 /*if there is a flash on board initialize it*/
	status = zl38063_init_check_flash(zl38063);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}

	/*erase all config/fwr*/
	status = zl38063_hbi_wr16(zl38063, ZL38063_CMD_PARAM_RESULT_REG, 0xAA55);
	if (status < 0) {
		return status;
	}

	/*erase firmware*/
	return zl38063_write_cmdreg(zl38063, 0x0009);

}
/*erase_fwrcfg_from_flash - use this function to erase a psecific firmware image
* and related  config from flash
 * previously stopped with VprocTwolfFirmwareStop()
 * Input Argument: image_number   - (range 1-14)
 *
 * \retval ::0 success
 * \retval ::-EINVAL or device error code
 */
static int  zl38063_erase_fwrcfg_from_flash(struct zl38063 *zl38063,
														   u16 image_number)
{
	int status = 0;
	if (image_number <= 0) {
		return -EINVAL;
	}
	/*save the config/fwr to flash position image_number */
	status = zl38063_stop_fwr_to_bootmode(zl38063);
	if (status < 0) {
		return status;
	}
	msleep(50);
	 /*if there is a flash on board initialize it*/
	status = zl38063_init_check_flash(zl38063);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}

	status = zl38063_hbi_wr16(zl38063, ZL38063_CMD_PARAM_RESULT_REG, image_number);
	if (status < 0) {
		return status;
	}
	status  = zl38063_write_cmdreg(zl38063, ZL38063_CMD_IMG_CFG_ERASE);
	if (status < 0) {
		return status;
	}

	return zl38063_cmdresult_check(zl38063);
}


/* tw_save_image_to_flash(): use this function to
 *     save both the config record and the firmware to flash. It Sets the bit
 *     which initiates a firmware save to flash when the device
 *     moves from a STOPPED state to a RUNNING state (by using the GO bit)
 *
 * Input Argument: None
 * \retval ::0 success
 * \retval ::-EINVAL or device error code
 */
static int zl38063_save_image_to_flash(struct zl38063 *zl38063)
{
	int status = 0;
	/*Save firmware to flash*/

#if 0
	 /*delete all applications on flash*/
	status = zl38063_erase_flash(zl38063);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: tw_erase_flash\n", status);
		return status;
	}
#else
	 /*if there is a flash on board initialize it*/
	status = zl38063_init_check_flash(zl38063);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}
#endif
	/*save the image to flash*/
	status = zl38063_write_cmdreg(zl38063, ZL38063_CMD_IMG_CFG_SAVE);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: zl38063_write_cmdreg\n", status);
		return status;
	}
	return zl38063_cmdresult_check(zl38063);
	/*return status;*/
}

/*load_fwr_from_flash - use this function to load a specific firmware image
* from flash
 * previously stopped with VprocTwolfFirmwareStop()
 * Input Argument: image_number   - (range 1-14)
 *
 * \retval ::0 success
 * \retval ::-EINVAL or device error code
 */
static int  zl38063_load_fwr_from_flash(struct zl38063 *zl38063,
													   u16 image_number)
{
	int status = 0;

	if (image_number <= 0) {
		return -EINVAL;
	}

	status = zl38063_stop_fwr_to_bootmode(zl38063);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}

	 /*if there is a flash on board initialize it*/
	status = zl38063_init_check_flash(zl38063);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}
	/*load the fwr to flash position image_number */
	status = zl38063_hbi_wr16(zl38063, ZL38063_CMD_PARAM_RESULT_REG, image_number);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}
	status = zl38063_write_cmdreg(zl38063, ZL38063_CMD_IMG_LOAD);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}
	return zl38063_start_fwr_from_ram(zl38063);
}

/*zl38063_load_fwrcfg_from_flash - use this function to load a specific firmware image
* related and config from flash
 * previously stopped with VprocTwolfFirmwareStop()
 * Input Argument: image_number   - (range 1-14)
 *
 * \retval ::0 success
 * \retval ::-EINVAL or device error code
 */
static int  zl38063_load_fwrcfg_from_flash(struct zl38063 *zl38063,
												  u16 image_number)
{

	int status = 0;
	if (image_number <= 0) {
		return -EINVAL;
	}
	status = zl38063_stop_fwr_to_bootmode(zl38063);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}

	 /*if there is a flash on board initialize it*/
	status = zl38063_init_check_flash(zl38063);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}

	/*save the config to flash position image_number */
	status = zl38063_hbi_wr16(zl38063, ZL38063_CMD_PARAM_RESULT_REG, image_number);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}
	status = zl38063_write_cmdreg(zl38063, ZL38063_CMD_IMG_CFG_LOAD);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}

	return zl38063_start_fwr_from_ram(zl38063);
}



/*zl38063_load_cfg_from_flash - use this function to load a specific firmware image
* from flash
 *  * Input Argument: image_number   - (range 1-14)
 *
 * \retval ::0 success
 * \retval ::-EINVAL or device error code
 */
static int  zl38063_load_cfg_from_flash(struct zl38063 *zl38063,
													u16 image_number)
{
	int status = 0;

	if (image_number <= 0) {
		return -EINVAL;
	}
	 /*if there is a flash on board initialize it*/
	status = zl38063_init_check_flash(zl38063);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}

	/*load the config from flash position image_number */
	status = zl38063_hbi_wr16(zl38063, ZL38063_CMD_PARAM_RESULT_REG, image_number);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}
	return zl38063_write_cmdreg(zl38063, ZL38063_CMD_CFG_LOAD);
}

/* save_cfg_to_flash(): use this function to
 *     save the config record to flash. It Sets the bit
 *     which initiates a config save to flash when the device
 *     moves from a STOPPED state to a RUNNING state (by using the GO bit)
 *
 * \retval ::0 success
 * \retval ::-EINVAL or device error code
 */
static int  zl38063_save_cfg_to_flash(struct zl38063 *zl38063, u16 image_number)
{
	int status = 0;
	u16 buf = 0;
	/*Save config to flash*/

	 /*if there is a flash on board initialize it*/
	status = zl38063_init_check_flash(zl38063);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}
	/*check if a firmware exists on the flash*/
	status = zl38063_hbi_rd16(zl38063, ZL38063_FWR_COUNT_REG, &buf);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}
	if (buf ==0)
		return -EBADSLT; /*There is no firmware on flash position image_number to save config for*/

	/*save the config to flash position */
	status = zl38063_hbi_wr16(zl38063, ZL38063_CMD_PARAM_RESULT_REG, image_number);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}

	/*save the config to flash position */
	status = zl38063_hbi_wr16(zl38063, ZL38063_CMD_REG, 0x8002);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}

	/*save the config to flash position */
	status = zl38063_hbi_wr16(zl38063, ZL38063_SW_FLAGS_REG, 0x0004);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}
	status = zl38063_cmdreg_acquire(zl38063);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: tw_cmdreg_acquire\n", status);
		return status;
	}
	/*Verify wheter the operation completed sucessfully*/
	return zl38063_cmdresult_check(zl38063);
}
#endif

/*AsciiHexToHex() - to convert ascii char hex to integer hex
 * pram[in] - str - pointer to the char to convert.
 * pram[in] - len - the number of character to convert (2:u8, 4:u16, 8:u32).

 */
static unsigned int AsciiHexToHex(const char * str, unsigned char len)
{
	unsigned int val = 0;
	char c;
	unsigned char i = 0;
	for (i = 0; i< len; i++)
	{
		c = *str++;
		val <<= 4;

		if (c >= '0' && c <= '9')
		{
			val += c & 0x0F;
			continue;
		}

		c &= 0xDF;
		if (c >= 'A' && c <= 'F')
		{
			val += (c & 0x07) + 9;
			continue;
		}
		return 0;
	}
	return val;
}

/* These 3 functions provide an alternative method to loading an *.s3
 *  firmware image into the device
 * Procedure:
 * 1- Call zl38063_boot_prepare() to put the device in boot mode
 * 2- Call the zl38063_boot_Write() repeatedly by passing it a pointer
 *    to one line of the *.s3 image at a time until the full image (all lines)
 *    are transferred to the device successfully.
 *    When the transfer of a line is complete, this function will return the sta-
 *    tus VPROC_STATUS_BOOT_LOADING_MORE_DATA. Then when all lines of the image
 *    are transferred the function will return the status
 *         VPROC_STATUS_BOOT_LOADING_CMP
 * 3- zl38063_boot_conclude() to complete and verify the completion status of
 *    the image boot loading process
 *
 */
static int zl38063_boot_prepare(struct zl38063 *zl38063)
{

	u16 buf = 0;
	int status = 0;
	status = zl38063_hbi_wr16(zl38063, CLK_STATUS_REG, 1); /*go to boot rom mode*/
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}
	msleep(50); /*required for the reset to cmplete*/
	/*check whether the device has gone into boot mode as ordered*/
	status = zl38063_hbi_rd16(zl38063, (u16)ZL38063_CMD_PARAM_RESULT_REG, &buf);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}

	if ((buf != 0xD3D3)) {
		TW_DEBUG1("ERROR: HBI is not accessible, cmdResultCheck = 0x%04x\n",
		buf);
		return  -EFAULT;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/

static int zl38063_boot_conclude(struct zl38063 *zl38063)
{
	int status = 0;
	status = zl38063_write_cmdreg(zl38063, ZL38063_CMD_HOST_LOAD_CMP); /*loading complete*/
	if (status < 0) {
		TW_DEBUG1("ERROR %d: \n", status);
		return status;
	}

	/*check whether the device has gone into boot mode as ordered*/
	return zl38063_cmdresult_check(zl38063);
}

/*----------------------------------------------------------------------------*/
/*  Read up to 256 bytes data */
/*  slave_zl380xx_nbytesrd()- read one or up to 256 bytes from the device
 *  \param[in]     ptwdata     pointer to the data read
 *  \param[in]     numbytes    the number of bytes to read
 *
 *  return ::status = the total number of bytes received successfully
 *                   or a negative error code
 */
static int
zl38063_nbytes_rd(struct zl38063 *zl38063, u8 numbytes, u8 *pData, u8 hbiAddrType)
{
	int status = 0;
	int tx_len = (hbiAddrType == TWOLF_HBI_PAGED) ? 4 : 2;
	u8 tx_buf[4] = {pData[0], pData[1], pData[2], pData[3]};

#ifdef ZL38063_SPI_IF
	struct spi_message msg;

	struct spi_transfer txfer = {
		.len = tx_len,
		.tx_buf = tx_buf,
	};
	struct spi_transfer rxfer = {
		.len = numbytes,
		.rx_buf = zl38063->pData,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&txfer, &msg);
	spi_message_add_tail(&rxfer, &msg);
	status = spi_sync(zl38063->spi, &msg);

#endif
#ifdef ZL38063_I2C_IF
	struct  i2c_msg     msg[2];
	memset(msg,0,sizeof(msg));

	msg[0].addr    = zl38063->i2c->addr;
	msg[0].len     = tx_len;
	msg[0].buf     = tx_buf;

	msg[1].addr = zl38063->i2c->addr;
	msg[1].len  = numbytes;
	msg[1].buf  = zl38063->pData;
	msg[1].flags = I2C_M_RD;
	status = i2c_transfer(zl38063->i2c->adapter, msg, 2);
#endif
	if (status <0)
		return status;

#ifdef MSCCDEBUG2
	{
		int i = 0;
		printk("RD: Numbytes = %d, addrType = %d\n", numbytes, hbiAddrType);
		for(i=0;i<numbytes;i++)
		{
			printk("0x%02x, ", zl38063->pData[i]);
		}
		printk("\n");
	}
#endif
	return 0;
}


static int
zl38063_hbi_access(struct zl38063 *zl38063,
	   u16 addr, u16 numbytes, u8 *pData, u8 hbiAccessType)
{

	u16 cmd;
	u8 page = addr >> 8;
	u8 offset = (addr & 0xFF)>>1;
	int i = 0;
	u8 hbiAddrType = 0;
	u8 buf[MAX_TWOLF_ACCESS_SIZE_IN_BYTES];
	int status = 0;

	u8 numwords = (numbytes/2);

	//TW_DEBUG1("page =%d addr =0x%x\n",page,addr);
#ifdef ZL38063_SPI_IF
	if (zl38063->spi == NULL) {
		printk("spi device is not available \n");
		return -EFAULT;
	}
#endif
#ifdef ZL38063_I2C_IF
	if (zl38063->i2c == NULL) {
		printk("i2c device is not available \n");
		return -EFAULT;
	}
#endif
	if ((numbytes > MAX_TWOLF_ACCESS_SIZE_IN_BYTES) || (numbytes ==0))
		return -EINVAL;

	if (pData == NULL)
		return -EINVAL;

	if (!((hbiAccessType == TWOLF_HBI_WRITE) ||
		  (hbiAccessType == TWOLF_HBI_READ)))
		 return -EINVAL;

	if (page == 0) /*Direct page access*/
	{
		if (hbiAccessType == TWOLF_HBI_WRITE)
		   cmd = HBI_DIRECT_WRITE(offset, numwords-1);/*build the cmd*/
		else
		   cmd = HBI_DIRECT_READ(offset, numwords-1);/*build the cmd*/

		buf[i++] = (cmd >> 8) & 0xFF ;
		buf[i++] = (cmd & 0xFF) ;
		hbiAddrType = TWOLF_HBI_DIRECT;
	} else { /*indirect page access*/
		i = 0;
		if (page != 0xFF) {
			page  -=  1;
		}
		cmd = HBI_SELECT_PAGE(page);
		/*select the page*/
		buf[i++] = (cmd >> 8) & 0xFF ;
		buf[i++] = (cmd & 0xFF) ;

		/*address on the page to access*/
		if (hbiAccessType == TWOLF_HBI_WRITE)
		   cmd = HBI_PAGED_WRITE(offset, numwords-1);
		else
		   cmd = HBI_PAGED_READ(offset, numwords-1);/*build the cmd*/

		 buf[i++] = (cmd >> 8) & 0xFF ;
		 buf[i++] = (cmd & 0xFF) ;
		 hbiAddrType = TWOLF_HBI_PAGED;
	}
	memcpy(&buf[i], pData, numbytes);
#ifdef MSCCDEBUG2
	{
		int j = 0;
		int displaynum = numbytes;
		if (hbiAccessType == TWOLF_HBI_WRITE)
			displaynum = numbytes;
		else
			displaynum = i;

		printk("SENDING:: Numbytes = %d, accessType = %d\n", numbytes, hbiAccessType);
		for(j=0;j<numbytes;j++)
		{
			printk("0x%02x, ", pData[j]);
		}
		printk("\n");
	}
#endif


	if (hbiAccessType == TWOLF_HBI_WRITE)
	   status = zl38063_nbytes_wr(zl38063, numbytes+i, buf);
	else
	   status = zl38063_nbytes_rd(zl38063, numbytes, buf, hbiAddrType);
	if (status < 0)
		return -EFAULT;

	return status;
}
/*
 * __ Upload a romcode  by blocks
 * the user app will call this function by passing it one line from the *.s3
 * file at a time.
 * when the firmware boot loading process find the execution address
 * in that block of data it will return the number 23
 * indicating that the transfer on the image data is completed, otherwise,
 * it will return 22 indicating tha tit expect more data.
 * If error, then a negative error code will be reported
 */
static int zl38063_boot_Write(struct zl38063 *zl38063,
							  char *blockOfFwrData) /*0: HBI; 1:FLASH*/
{

/*Use this method to load the actual *.s3 file line by line*/
	int status = 0;
	int rec_type, i=0, j=0;
	u8 numbytesPerLine = 0;
	u8 buf[MAX_TWOLF_FIRMWARE_SIZE_IN_BYTES];
	unsigned long address = 0;
	u8 page255Offset = 0x00;
	u16 cmd = 0;

	TW_DEBUG2("firmware line# = %d :: blockOfFwrData = %s\n",++myCounter, blockOfFwrData);

	if (blockOfFwrData == NULL) {
	   TW_DEBUG1("blockOfFwrData[0] = %c\n", blockOfFwrData[0]);
	   return -EINVAL;
	}
	/* if this line is not an srecord skip it */
	if (blockOfFwrData[0] != 'S') {
		TW_DEBUG1("blockOfFwrData[0] = %c\n", blockOfFwrData[0]);
		return -EINVAL;
	}
	/* get the srecord type */
	rec_type = blockOfFwrData[1] - '0';

	numbytesPerLine = AsciiHexToHex(&blockOfFwrData[2], 2);
	//TW_DEBUG2("numbytesPerLine = %d\n", numbytesPerLine);
	if (numbytesPerLine == 0) {
		 TW_DEBUG1("blockOfFwrData[3] = %c\n", blockOfFwrData[3]);
		 return -EINVAL;
	}

	/* skip non-existent srecord types and block header */
	if ((rec_type == 4) || (rec_type == 5) || (rec_type == 6) ||
											  (rec_type == 0)) {
		return TWOLF_STATUS_NEED_MORE_DATA;
	}

	/* get the info based on srecord type (skip checksum) */
	address = AsciiHexToHex(&blockOfFwrData[4], 8);
	buf[0] = (u8)((address >> 24) & 0xFF);
	buf[1] = (u8)((address >> 16) & 0xFF);
	buf[2] = (u8)((address >> 8) & 0xFF);
	buf[3] = 0;
	page255Offset = (u8)(address & 0xFF);
	/* store the execution address */
	if ((rec_type == 7) || (rec_type == 8) || (rec_type == 9)) {
		/* the address is the execution address for the program */
		//TW_DEBUG2("execAddr = 0x%08lx\n", address);
		/* program the program's execution start register */
		buf[3] = (u8)(address & 0xFF);
		status = zl38063_hbi_access(zl38063,
					   ZL38063_FWR_EXEC_REG, 4, buf, TWOLF_HBI_WRITE);

		if(status < 0) {
			TW_DEBUG1("ERROR % d: unable to program page 1 execution address\n", status);
			return status;
		}
		TW_DEBUG2("Loading firmware data complete...\n");
		return TWOLF_STATUS_BOOT_COMPLETE;  /*BOOT_COMPLETE Sucessfully*/
	}

	/* put the address into our global target addr */


	//TW_DEBUG2("TW_DEBUG2:gTargetAddr = 0x%08lx: \n", address);
	status = zl38063_hbi_access(zl38063,
					   PAGE_255_BASE_HI_REG, 4, buf, TWOLF_HBI_WRITE);
	if (status < 0) {
		TW_DEBUG1("ERROR %d: gTargetAddr = 0x%08lx: \n", status, address);
		return -EFAULT;
	}

	/* get the data bytes */
	j = 12;
	//TW_DEBUG2("buf[]= 0x%02x, 0x%02x, \n", buf[0], buf[1]);
	for (i = 0; i < numbytesPerLine - 5; i++) {
		buf[i] = AsciiHexToHex(&blockOfFwrData[j], 2);
		j +=2;
		//TW_DEBUG2("0x%02x, ", buf[i+4]);
	}
	/* write the data to the device */
	cmd = (u16)(0xFF<<8) | (u16)page255Offset;
	status = zl38063_hbi_access(zl38063,
					   cmd, (numbytesPerLine - 5), buf, TWOLF_HBI_WRITE);
	if(status < 0) {
		return status;
	}

	//TW_DEBUG2("Provide next block of data...\n");
	return TWOLF_STATUS_NEED_MORE_DATA; /*REQUEST STATUS_MORE_DATA*/
}



/*----------------------------------------------------------------------*
 *   The kernel driver functions are defined below
 *-------------------------DRIVER FUNCTIONs-----------------------------*/
/* zl38063_load_file()
 * This function basically  will load the firmware into device
 * This is convenient for host pluging system that does not have
 * a slave EEPROM/FLASH to store the firmware, therefore, and that
 * requires the device to be fully operational at power up
*/

static int zl38063_load_file(struct zl38063 *zl38063)
{

	int status = 0;

	const struct firmware *twfw;
	u8 numbytesPerLine = 0;
	u32 j =0;
	u8 block_size = 0;

	zl38063EnterCritical();

	if (fw_path == NULL){
		  TW_DEBUG1("err %d, invalid firmware filename %s\n", status, fw_path);
		  zl38063ExitCritical();
		  return -EINVAL;
	}
	TW_DEBUG1("Using the %s Direct loading ...\n",fw_path);

	status = request_firmware(&twfw, fw_path,
#ifdef ZL38063_SPI_IF
	&zl38063->spi->dev
#endif
#ifdef ZL38063_I2C_IF
	&zl38063->i2c->dev
#endif
	);
	if (status) {
		  TW_DEBUG1("err %d, request_firmware failed to load %s\n",
													   status, fw_path);
		  zl38063ExitCritical();
		  return -EINVAL;
	}

	/*check validity of the S-record firmware file*/
	if (twfw->data[0] != 'S') {
		 TW_DEBUG1("Invalid S-record %s file for this device\n", fw_path);
		  release_firmware(twfw);
		  zl38063ExitCritical();
		  return -EINVAL;
	}

	/* Allocating memory for the data buffer if not already done*/
	if (!zl38063->pData) {
		zl38063->pData  = kmalloc(MAX_TWOLF_FIRMWARE_SIZE_IN_BYTES, GFP_KERNEL);
		if (!zl38063->pData) {
			TW_DEBUG1("can't allocate memory\n");
			release_firmware(twfw);
			zl38063ExitCritical();
			return -ENOMEM;
		}
		memset(zl38063->pData, 0, MAX_TWOLF_FIRMWARE_SIZE_IN_BYTES);
	}

	status = zl38063_boot_prepare(zl38063);
	if (status < 0) {
		  TW_DEBUG1("err %d, tw boot prepare failed\n", status);
		  goto fwr_cleanup;
	}

	TW_DEBUG1("loading firmware %s into the device ...\n", fw_path);

	do {
		numbytesPerLine = AsciiHexToHex(&twfw->data[j+2], 2);
		block_size = (4 + (2*numbytesPerLine));

		memcpy(zl38063->pData, &twfw->data[j], block_size);
		j += (block_size+2);

		status = zl38063_boot_Write(zl38063, zl38063->pData);
		if ((status != (int)TWOLF_STATUS_NEED_MORE_DATA) &&
			(status != (int)TWOLF_STATUS_BOOT_COMPLETE)) {
			  TW_DEBUG1("err %d, tw boot write failed\n", status);
			  goto fwr_cleanup;
		}

	} while ((j < twfw->size) && (status != TWOLF_STATUS_BOOT_COMPLETE));

	status = zl38063_boot_conclude(zl38063);
	if (status < 0) {
		  TW_DEBUG1("err %d, twfw->size = %d, tw boot conclude -firmware loading failed\n", status, (int)twfw->size);
		  goto fwr_cleanup;
	}
#ifdef ZL38063_SAVE_FWR_TO_FLASH
	status = zl38063_save_image_to_flash(zl38063);
	if (status < 0) {
		  TW_DEBUG1("err %d, twfw->size = %d, saving firmware failed\n", status, twfw->size);
		  goto fwr_cleanup;
	}
#endif  /*ZL38063_SAVE_FWR_TO_FLASH*/
	status = zl38063_start_fwr_from_ram(zl38063);
	if (status < 0) {
		  TW_DEBUG1("err %d, twfw->size = %d, starting firmware failed\n", status,(int)twfw->size);
		  goto fwr_cleanup;
	}
/*loading the configuration record*/
   {
		u16 reg = 0  ,val =0;
		release_firmware(twfw);
		j = 0; /*skip the CR2K key word*/

		if (cr_path == NULL){
		  TW_DEBUG1("err %d, invalid configuration record filename %s\n", status, cr_path);
		  zl38063ExitCritical();
		  return 0;
	}

		numbytesPerLine = 10;
		status = request_firmware(&twfw, cr_path,
	#ifdef ZL38063_SPI_IF
		&zl38063->spi->dev
	#endif
	#ifdef ZL38063_I2C_IF
		&zl38063->i2c->dev
	#endif
		);
		if (status) {
			  TW_DEBUG1("err %d, request_firmware failed to load %s\n",
														   status, cr_path);
			  goto fwr_cleanup;
		}
		TW_DEBUG1("loading config %s into the device  size %d ...\n", cr_path,(int)twfw->size);

		do {
			if ((twfw->data[j] == '0') && (twfw->data[j+1] == 'x') ){

				if (twfw->data[j+16] != 'R' && twfw->data[j+17] != 'e'){
					reg = AsciiHexToHex(&twfw->data[j+2], 4);
					val = AsciiHexToHex(&twfw->data[j+10], 4);
					status = zl38063_hbi_wr16(zl38063, reg, val);
					//printk("Write reg 0x[%x] with Val 0x[%x]\n",reg,val);
					if (status < 0) {
						  TW_DEBUG1("err %d, tw config write failed\n", status);
						  goto fwr_cleanup;
					}
				}
				j += 50;
			}
			j += 1;
		} while (j < twfw->size);
#ifdef ZL38063_SAVE_FWR_TO_FLASH
		if (zl38063_hbi_rd16(zl38063, ZL38063_FWR_COUNT_REG, &val) < 0) {
			return -EIO;
		}
		if (val == 0) {
			 TW_DEBUG1("err: there is no firmware on flash\n");
			 return -1;
		}

		status = zl38063_save_cfg_to_flash(zl38063, val);
		if (status < 0) {
			  TW_DEBUG1("err %d, tw save config to flash failed\n", status);
		}
		TW_DEBUG1("zl38063 image/cfg saved to flash position %d - success...\n", val);

#endif   /*ZL38063_SAVE_FWR_TO_FLASH*/
	}
	 /*status = zl38063_reset(zl38063, ZL38063_RST_SOFTWARE); */
	goto fwr_cleanup;

fwr_cleanup:
	zl38063ExitCritical();
	release_firmware(twfw);

	return status;
}

/*VprocTwolfLoadConfig() - use this function to load a custom or new config
 * record into the device RAM to override the default config
 * \retval ::VPROC_STATUS_SUCCESS
 * \retval ::VPROC_STATUS_ERR_HBI
 */
static int zl38063_load_config(struct zl38063 *zl38063,
	dataArr *pCr2Buf,
	unsigned short numElements)
{
	int status = 0;
	int i;
	u8 buf[MAX_TWOLF_FIRMWARE_SIZE_IN_BYTES];
	/*stop the current firmware but do not reset the device and do not go to boot mode*/
	TW_DEBUG1("Using the firmware-config c code loading ...\n");
	/*send the config to the device RAM*/
	for (i=0; i<numElements; i++) {
		u8 j=0, k=0;
		for (k = 0; k < ZL38063_CFG_BLOCK_SIZE; k++) {
			buf[j] = (u8)((pCr2Buf[i].value >> 8) & 0xFF);
			buf[j+1] = (u8)((pCr2Buf[i].value) & 0xFF);
			j +=2;
		}
		status = zl38063_hbi_access(zl38063, pCr2Buf[i].reg, ZL38063_CFG_BLOCK_SIZE*2, buf, TWOLF_HBI_WRITE);
	}

	return status;
}
/* zl38063_boot_alt Use this alternate method to load the asr_twFirmware.c
 *(converted *.s3 to c code) to the device
 */


static int zl38063_load_fwr(struct zl38063 *zl38063,struct zlFirmware load_fwr, const twFwr* data_twFwr)
{
	u8 page255Offset = 0x00;
	int status = 0;
	u8 buf[MAX_TWOLF_FIRMWARE_SIZE_IN_BYTES];

	u16 index = 0;

	while (index < load_fwr.FirmwareStreamLen) {
		int i=0, j=0;
		/* put the address into our global target addr */
		buf[0] = (u8)((data_twFwr[index].targetAddr >> 24) & 0xFF);
		buf[1] = (u8)((data_twFwr[index].targetAddr >> 16) & 0xFF);
		buf[2] = (u8)((data_twFwr[index].targetAddr >> 8) & 0xFF);
		buf[3] = 0;
		page255Offset = (u8)(data_twFwr[index].targetAddr & 0xFF);
		/* write the data to the device */
		if (data_twFwr[index].numWords != 0) {

			status = zl38063_hbi_access(zl38063,
					   PAGE_255_BASE_HI_REG, 4, buf, TWOLF_HBI_WRITE);
			if (status < 0) {
				TW_DEBUG1("ERROR %d: gTargetAddr = 0x%08lx: \n", status, data_twFwr[index].targetAddr);
				zl38063ExitCritical();
				return -EFAULT;
			}

			for (i = 0; i < data_twFwr[index].numWords; i++) {
				buf[j] = (u8)((data_twFwr[index].buf[i] >> 8) & 0xFF);
				buf[j+1] = (u8)((data_twFwr[index].buf[i]) & 0xFF);
				j+=2;
				//TW_DEBUG2("0x%02x, ", buf[i+4]);
			}
			/* write the data to the device */

			status = zl38063_hbi_access(zl38063,((u16)(0xFF<<8) | (u16)page255Offset),
							   (data_twFwr[index].numWords*2), buf, TWOLF_HBI_WRITE);
			if(status < 0) {
				zl38063ExitCritical();
				return status;
			}
		}
	index++;
	}

	/*
	 * convert the number of bytes to two 16 bit
	 * values and write them to the requested page register
	 */
	/* even number of bytes required */

	/* program the program's execution start register */
	buf[0] = (u8)((load_fwr.execAddr >> 24) & 0xFF);
	buf[1] = (u8)((load_fwr.execAddr >> 16) & 0xFF);
	buf[2] = (u8)((load_fwr.execAddr >> 8) & 0xFF);
	buf[3] = (u8)(load_fwr.execAddr & 0xFF);
	status = zl38063_hbi_access(zl38063,
				   ZL38063_FWR_EXEC_REG, 4, buf, TWOLF_HBI_WRITE);

	if(status < 0) {
		TW_DEBUG1("ERROR % d: unable to program page 1 execution address\n", status);
		zl38063ExitCritical();
		return status;
	}
	/* print out the srecord program info */
	TW_DEBUG2("prgmBase 0x%08lx\n", load_fwr.prgmBase);
	TW_DEBUG2("execAddr 0x%08lx\n", load_fwr.execAddr);

	status = zl38063_boot_conclude(zl38063);
	if (status < 0) {
		  TW_DEBUG1("err %d, tw boot conclude -firmware loading failed\n", status);
		  zl38063ExitCritical();
		  return -1;
	}
	if (zl38063_start_fwr_from_ram(zl38063) < 0) {
		  TW_DEBUG1("err: starting firmware failed\n", );
		  zl38063ExitCritical();
		  return -2;
	}
	return 0;
}
static int zl38063_load_firmware(struct zl38063 *zl38063,struct nl_msg_audio *msg)
{
	int status = 0;

	zl38063EnterCritical();
	status = zl38063_boot_prepare(zl38063);
	if (status < 0) {
		  TW_DEBUG1("err %d, boot prepare failed\n", status);
		  zl38063ExitCritical();
		  return -1;
	}
	if (boot_to_BootMode == 1){
		TW_DEBUG1("Enter DEBUG mode successed \nJump to boot mode and wait UART CMD ...\n");
		zl38063ExitCritical();
		return 0;
	}
	TW_DEBUG1("Using the firmware-config ID [%d]  c code loading ...\n",zl38063->fw_label);


	if (DOA == zl38063->fw_label){
		status = zl38063_load_fwr(zl38063,DoaFirmware,doa_fwr);
		if (status < 0)
			printk("%s  %d load F/W error: %d\n",__FUNCTION__,__LINE__,status);
		status = zl38063_load_config(zl38063, (dataArr *)doa_cfg, (u16)configStreamLen);
	}else if (AEC == zl38063->fw_label){
		status = zl38063_load_fwr(zl38063,AecFirmware,aec_fwr);
		if (status < 0)
			printk("%s  %d load F/W error: %d\n",__FUNCTION__,__LINE__,status);
		status = zl38063_load_config(zl38063, (dataArr *)aec_cfg, (u16)configStreamLen);
	}else{
		status = zl38063_load_fwr(zl38063,AsrFirmware,asr_fwr);
		if (status < 0)
			printk("%s %d load F/W error: %d\n",__FUNCTION__,__LINE__,status);
		status = zl38063_load_config(zl38063, (dataArr *)asr_cfg, (u16)configStreamLen);
	}
	if (status < 0) {
		 printk("%s load config error: %d\n",__FUNCTION__,status);
		  zl38063ExitCritical();
		  return status;
	}

#ifdef ZL38063_SAVE_FWR_TO_FLASH
	status = zl38063_save_image_to_flash(zl38063);
	if (status < 0) {
		  TW_DEBUG1("err %d, saving firmware to flash failed\n", status);
		  zl38063ExitCritical();
		  return status;
	}
		u16 val = 0;
		if (zl38063_hbi_rd16(zl38063, ZL38063_FWR_COUNT_REG, &val) < 0) {
			return -EIO;
		}
		if (val == 0) {
			 TW_DEBUG1("err: there is no firmware on flash\n");
			 return -1;
		}
		status = zl38063_save_cfg_to_flash(zl38063, val);
		if (status < 0) {
			  TW_DEBUG1("err %d, tw save config to flash failed\n", status);
			  zl38063ExitCritical();
			  return status;
		}
		TW_DEBUG1("zl38063 image/cfg saved to flash position %d - success...\n", val);

#endif   /*ZL38063_SAVE_FWR_TO_FLASH*/


	TW_DEBUG1("zl38063 boot init complete - success... \n");
	zl38063ExitCritical();
	return 0;
}

#ifdef ZL380XX_TW_ENABLE_ALSA_CODEC_DRIVER

/* ALSA soc codec default Timberwolf register settings
 * 3 Example audio cross-point configurations
 */
/* ALSA soc codec default Timberwolf register settings
 * 3 Example audio cross-point configurations
 */
#define CODEC_CONFIG_REG_NUM 42

u8 reg_stereo[] ={ 0x00, 0x00, 0x00, 0x06, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


/*4-port m
ode with AEC enabled  - ROUT to DAC1
 * MIC1 -> SIN (ADC)
 * I2S-L -> RIN (TDM Rx path)
 * SOUT -> I2S-L
 * ROUT -> DACx
 *reg 0x202 - 0x22A
 */
u8 reg_cache_default[CODEC_CONFIG_REG_NUM] ={0x0c, 0x05, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x0d, 0x00, 0x0d, 0x00, 0x0e, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x0d,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x01, 0x06, 0x05, 0x00, 0x02, 0x00, 0x00};



/*Formatting for the Audio*/
#define zl38063_DAI_RATES            (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000)
#define zl38063_DAI_FORMATS          (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

#define zl38063_DAI_CHANNEL_MIN      1
#define zl38063_DAI_CHANNEL_MAX      8

static unsigned int zl38063_reg_read(struct snd_soc_codec *codec,
			unsigned int reg)
{
	unsigned int value = 0;

	struct zl38063 *zl38063 = snd_soc_codec_get_drvdata(codec);
	u16 buf;

	if (zl38063_hbi_rd16(zl38063, reg, &buf) < 0) {
		return -EIO;
	}
	value = buf;
	TW_DEBUG1("reg = 0x%04x ,val = 0x%04x \n",reg,value);
	return value;

}

static int zl38063_reg_write(struct snd_soc_codec *codec,
			unsigned int reg, unsigned int value)
{
	struct zl38063 *zl38063 = snd_soc_codec_get_drvdata(codec);
	int i=0;
	if (zl38063_hbi_wr16(zl38063, reg, value) < 0) {
		return -EIO;
	}
	if (zl38063_priv->init_done == 1 && reg > ZL38063_GPIO_EVEN && reg <= ZL38063_SIN_MAX_RMS){
	for (i=0;i< NEED_RESET_NUM; i++)
		if (need_reset[i] == reg){
			zl38063_reset(zl38063, ZL38063_RST_SOFTWARE);
			i=0xfff;
			break;
		}
	}
	TW_DEBUG1("Write reg = 0x%04x ,val = 0x%04x need reset: %s\n",reg,value,i==0xfff?"Yes":"No");

	return 0;
}


/*
 * Cross Point Gains: (p.g. 103)
 * Values less than -90 and greater than +6 are clipped to each respective limit.
 *
 * max : 0x06 : +6 dB
 *       ( 1 dB step )
 * min : 0xa6 : -90 dB
 *
 * from -90 to 6 dB in 1 dB steps
 * Fixme current just -90 to -1
 */
static DECLARE_TLV_DB_RANGE(cp_gain_tlv,
	0xa6,0xff,TLV_DB_SCALE_ITEM(-9000, 100, 0),
	0x00,0x06,TLV_DB_SCALE_ITEM(0, 100, 0),
	0x07,0x7f,TLV_DB_SCALE_ITEM(600, 0, 0),
	0x80,0xa5,TLV_DB_SCALE_ITEM(-9000, 0, 0)
);


/*
 * User Rout Path Gain Pad: (p.g. 150)
 *
 * max : 0x78 : +21.0 dB
 *       ( 0.375 dB step )
 * min : 0x00 : -24.0 dB
 *
 * from -24.0 to 21 dB in 0.375 dB steps
 */
static DECLARE_TLV_DB_SCALE(ugain_tlv, -2400, 37, 0);

/*
 *Extended User Rout Gain Adjust: (p.g. 150)
 *
 * max : 0x05 : +30 dB
 *       ( 6 dB step )
 * min : 0x00 : 0 dB
 *
 * from 0 to 30 dB in 6 dB steps
 */
static DECLARE_TLV_DB_SCALE(ext_ugain_tlv, 0, 600, 0);

/*
 * Sout Gain Pad: (p.g. 150)
 *
 * max : 0x0f : +21 dB
 *       ( 3 dB step )
 * min : 0x00 : -24 dB
 *
 * from -24 to 21 dB in 3 dB steps
 */
static DECLARE_TLV_DB_SCALE(sout_gain_tlv, -2400, 300, 0);

/*
 * Sin Gain Pad: (p.g. 150)
 *
 * max : 0x1f : +22.5 dB
 *       ( 1.5 dB step )
 * min : 0x00 : -24.0 dB
 *
 * from -24.0 to +22.5 dB in 1.5 dB steps
 */
static DECLARE_TLV_DB_SCALE(sin_gain_tlv, -2400, 150, 0);

/*
 * Digital Mic Gain: (p.g. 134)
 *
 * max : 0x7 : +42 dB
 *       ( 6 dB step )
 * min : 0x00 : 0 dB
 *
 * from 0 to +42 dB in 6 dB steps
 */
static DECLARE_TLV_DB_SCALE(mic_gain_tlv, 0, 600, 0);


/*The DACx, I2Sx, TDMx Gains can be used in both AEC mode or Stereo bypass mode
 * however the AEC Gains can only be used when AEC is active.
 * Each input source of the cross-points has two input sources A and B.
 * The Gain for each source can be controlled independantly.
 */
static const struct snd_kcontrol_new zl38063_snd_controls[] = {
	SOC_SINGLE_TLV("DAC1 A Volume",
		ZL38063_DAC1_GAIN_REG, 0, 0xff, 0, cp_gain_tlv),
	SOC_SINGLE_TLV("DAC1 B Volume",
		ZL38063_DAC1_GAIN_REG, 8, 0xff, 0, cp_gain_tlv),
	SOC_SINGLE_TLV("DAC2 A Volume",
		ZL38063_DAC2_GAIN_REG, 0, 0xff, 0, cp_gain_tlv),
	SOC_SINGLE_TLV("DAC2 B Volume",
		ZL38063_DAC2_GAIN_REG, 8, 0xff, 0, cp_gain_tlv),
	SOC_SINGLE_TLV("I2S1L A Volume",
		ZL38063_I2S1L_GAIN_REG, 0, 0xff, 0, cp_gain_tlv),
	SOC_SINGLE_TLV("I2S1L B Volume",
		ZL38063_I2S1L_GAIN_REG, 8, 0xff, 0, cp_gain_tlv),
	SOC_SINGLE_TLV("I2S1R A Volume",
		ZL38063_I2S1R_GAIN_REG, 0, 0xff, 0, cp_gain_tlv),
	SOC_SINGLE_TLV("I2S1R B Volume",
		ZL38063_I2S1R_GAIN_REG, 8, 0xff, 0, cp_gain_tlv),
	SOC_SINGLE_TLV("I2S2L A Volume",
		ZL38063_I2S2L_GAIN_REG, 0, 0xff, 0, cp_gain_tlv),
	SOC_SINGLE_TLV("I2S2L B Volume",
		ZL38063_I2S2L_GAIN_REG, 8, 0xff, 0, cp_gain_tlv),
	SOC_SINGLE_TLV("I2S2R A Volume",
		ZL38063_I2S2R_GAIN_REG, 0, 0xff, 0, cp_gain_tlv),
	SOC_SINGLE_TLV("I2S2R B Volume",
		ZL38063_I2S2R_GAIN_REG, 8, 0xff, 0, cp_gain_tlv),
	SOC_SINGLE_TLV("AEC SIN1 A Volume",
		ZL38063_CRCP_GAIN_AECSIN, 0, 0xff, 0, cp_gain_tlv),
	SOC_SINGLE_TLV("AEC SIN1 B Volume",
		ZL38063_CRCP_GAIN_AECSIN, 8, 0xff, 0, cp_gain_tlv),
	SOC_SINGLE_TLV("AEC RIN A Volume",
		ZL38063_CRCP_GAIN_AECRSN, 0, 0xff, 0, cp_gain_tlv),
	SOC_SINGLE_TLV("AEC RIN B Volume",
		ZL38063_CRCP_GAIN_AECRSN, 8, 0xff, 0, cp_gain_tlv),
	SOC_SINGLE_TLV("AEC SIN2 A Volume",
		ZL38063_CRCP_GAIN_AECSIN2, 0, 0xff, 0, cp_gain_tlv),
	SOC_SINGLE_TLV("AEC SIN2 B Volume",
		ZL38063_CRCP_GAIN_AECSIN2, 8, 0xff, 0, cp_gain_tlv),

	SOC_SINGLE_TLV("AEC ROUT Volume",
		ZL38063_ROUT_GAIN_CTL, 0, 0x78, 0,ugain_tlv),
	SOC_SINGLE_TLV("AEC ROUT EXT Volume",
		ZL38063_ROUT_GAIN_CTL, 7, 0x5, 0,ext_ugain_tlv),
	SOC_SINGLE_TLV("AEC SOUT Volume",
		ZL38063_USR_GAIN_CTL, 8, 0xf, 0,sout_gain_tlv),
	SOC_SINGLE_TLV("AEC SIN Volume",
		ZL38063_USR_GAIN_CTL, 0, 0x1f, 0,sin_gain_tlv),


	SOC_SINGLE_TLV("AEC MIC Volume",
		ZL38063_MIC_GAIN, 0, 0x7, 0,mic_gain_tlv),


	SOC_SINGLE("MUTE SPEAKER ROUT Switch", ZL38063_AEC_CTL0, 7, 1, 0),
	SOC_SINGLE("MUTE MIC SOUT Switch", ZL38063_AEC_CTL0, 8, 1, 0),
	SOC_SINGLE("AEC Bypass Switch", ZL38063_AEC_CTL0, 4, 1, 1),
	SOC_SINGLE("AEC Audio enh bypass Switch", ZL38063_AEC_CTL0, 5, 1, 1),
	SOC_SINGLE("AEC Master Bypass Switch", ZL38063_AEC_CTL0, 1, 1, 1),
	SOC_SINGLE("ALC GAIN Switch",ZL38063_AEC_CTL1, 12, 1, 1),
	SOC_SINGLE("ALC Control Switch", ZL38063_AEC_CTL1, 10, 1, 1),
	SOC_SINGLE("AEC Tail disable Switch", ZL38063_AEC_CTL1, 12, 1, 1),
	SOC_SINGLE("AEC Comfort Noise Switch", ZL38063_AEC_CTL1, 6, 1, 1),
	SOC_SINGLE("NLAEC Control Switch", ZL38063_AEC_CTL1, 14, 1, 1),
	SOC_SINGLE("AEC Adaptation Switch", ZL38063_AEC_CTL1, 1, 1, 1),
	SOC_SINGLE("NLP Control Switch", ZL38063_AEC_CTL1, 5, 1, 1),
	SOC_SINGLE("Noise Inject Switch", ZL38063_AEC_CTL1, 6, 1, 1),

};


int zl38063_add_controls(struct snd_soc_codec *codec)
{
	return snd_soc_add_codec_controls(codec, zl38063_snd_controls,
			ARRAY_SIZE(zl38063_snd_controls));
 }


/* If the ZL38063 TDM is configured as a slace I2S/PCM,
 * The rate settings are not necessary. The Zl38063 has the ability to
 * auto-detect the master rate and configure itself accordingly
 * If configured as master, the rate must be set acccordingly
 */
static int zl38063_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	u16 rate = params_rate(params);
	u16 tmp, bits, slotnum, mode=0;

	TW_DEBUG1("Set zl38063_hw_params\n");

	if (zl38063_priv->fw_label == AEC && rate != 8000 && rate != 16000 )
	{
		printk("AEC mode can only support 8K and 16K sample rate, current %u\n",rate);
		return -EINVAL;
	}

	zl38063EnterCritical();

	switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			mode = (ZL38063_TDMA_16BIT_LIN);
			slotnum = 0;  /*If PCM use 0 timeslots*/
			bits = 31;
			break;
		case SNDRV_PCM_FORMAT_MU_LAW:
			mode |= (ZL38063_TDMA_8BIT_ULAW);
			slotnum |= 0;
			bits = 15;
			break;
		case SNDRV_PCM_FORMAT_A_LAW:
			mode = (ZL38063_TDMA_8BIT_ALAW );
			slotnum |= 0;
			bits = 15;
			break;

		case SNDRV_PCM_FORMAT_S20_3LE:
		case SNDRV_PCM_FORMAT_S24_LE:
		case SNDRV_PCM_FORMAT_S32_LE:
		default: {
			zl38063ExitCritical();
			return -EINVAL;
		}
	}
	snd_soc_update_bits(codec,ZL38063_TDMA_CONF_CH1,0x0700,mode);
	snd_soc_update_bits(codec,ZL38063_TDMA_CONF_CH1,0x00ff,slotnum);
	snd_soc_update_bits(codec,ZL38063_TDMA_CLK,0x7ff0,bits<<4);

	tmp = zl38063_reg_read(codec, ZL38063_TDMA_CLK);
	if (tmp & ZL38063_TDM_I2S_CFG_VAL){			//work in master mode
		switch (rate) {
			case 8000:
				tmp = ZL38063_TDMA_FSRATE_8KHZ;
				break;
			case 16000:
				tmp = ZL38063_TDMA_FSRATE_16KHZ;
				break;
			case 48000:
				tmp = ZL38063_TDMA_FSRATE_48KHZ;
				break;
			default: {
				zl38063ExitCritical();
				return -EINVAL;
			}
		}
		snd_soc_update_bits(codec,ZL38063_TDMA_CLK,0x0007,tmp);
	}else{
		TW_DEBUG1("ZL38063 work under slave mode, auto-detect the master rate\n");

	}
	zl38063ExitCritical();


	return 0;
}


/* When set, data on Rout audio path is muted. When cleared, the
Rout audio path is not muted. <Playback> */
static int zl38063_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	zl38063EnterCritical();

	TW_DEBUG1("Rout audio path mute: %d  \n",mute);
	snd_soc_update_bits(codec,ZL38063_AEC_CTL0,1<<7,mute<<7);

	zl38063ExitCritical();
	return 0;

}

static int zl38063_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *codec_dai)
{

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	default:
		break;
	}

	TW_DEBUG1("\t CMD =[%d]\n",cmd);
	return 0;
}

static int zl38063_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 format, clock;

	zl38063EnterCritical();
	format = zl38063_reg_read(codec, ZL38063_TDMA_FMT);
	clock = zl38063_reg_read(codec, ZL38063_TDMA_CLK);

	TW_DEBUG1("ZL38063_TDMA_FMT read is  0x%x,ZL38063_TDMA_CLK read 0x%x \n",format,clock);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_DSP_B:
			break;
		case SND_SOC_DAIFMT_DSP_A:
			break;
		case SND_SOC_DAIFMT_RIGHT_J:
			if ((format & ZL38063_TDMA_FSALIGN) )
				format &= (~ZL38063_TDMA_FSALIGN);
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			if (!((format & ZL38063_TDMA_FSALIGN) ))
				format |= (ZL38063_TDMA_FSALIGN);
			break;
		case SND_SOC_DAIFMT_I2S:
			if (!((format & ZL38063_TDM_I2S_CFG_VAL) )) {
				format |= (ZL38063_TDM_I2S_CFG_VAL );
			 }
			break;
		default: {
			zl38063ExitCritical();
			return -EINVAL;
		}
	}

	/* set master/slave TDM interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBM_CFM:
			clock |= ZL38063_TDM_TDM_MASTER_VAL;
			break;
		case SND_SOC_DAIFMT_CBS_CFS:
			if (clock & ZL38063_TDM_TDM_MASTER_VAL)
				clock &= (~ZL38063_TDM_TDM_MASTER_VAL);
			break;
		default: {
			zl38063ExitCritical();
			return -EINVAL;
		}
	}

	TW_DEBUG1("ZL38063_TDMA_FMT write is  0x%x,ZL38063_TDMA_PCM_CONF write 0x%x \n",format,clock);

	zl38063_reg_write(codec, ZL38063_TDMA_FMT, format);
	zl38063_reg_write(codec, ZL38063_TDMA_CLK, clock);
	zl38063ExitCritical();

	return 0;
}


/* ZL38063 use a 12M external clock */
static int zl38063_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
		unsigned int freq, int dir)
{
	zl38063_priv->sysclk_rate = freq;
	TW_DEBUG1("zl38063->sysclk_rate is %d ,dir is %d \n",freq,dir);
	return 0;
}

static const struct snd_soc_dai_ops zl38063_dai_ops = {
		.set_fmt        = zl38063_set_dai_fmt,
		.set_sysclk     = zl38063_set_dai_sysclk,
		.hw_params   	= zl38063_hw_params,
		.digital_mute   = zl38063_mute,
		.trigger	 	= zl38063_trigger,

};

static struct snd_soc_dai_driver zl38063_dai = {
		.name = "zl38063-hifi",
		.playback = {
				.stream_name = "Playback",
				.channels_min = zl38063_DAI_CHANNEL_MIN,
				.channels_max = zl38063_DAI_CHANNEL_MAX,
				.rates = zl38063_DAI_RATES,
				.formats = zl38063_DAI_FORMATS,
		},
		.capture = {
				.stream_name = "Capture",
				.channels_min = zl38063_DAI_CHANNEL_MIN,
				.channels_max = zl38063_DAI_CHANNEL_MAX,
				.rates = zl38063_DAI_RATES,
				.formats = zl38063_DAI_FORMATS,
		},
		.ops = &zl38063_dai_ops,
};
EXPORT_SYMBOL(zl38063_dai);

static int zl38063_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	//struct zl38063 *zl38063 = snd_soc_codec_get_drvdata(codec);

	zl38063EnterCritical();
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		/*wake up from sleep*/
		//zl38063_state_init(zl38063, (HBI_CONFIG_VAL | HBI_CONFIG_WAKE));
		//msleep(10);
		/*Clear the wake up bit*/
		//zl38063_state_init(zl38063, HBI_CONFIG_VAL);

		break;
	case SND_SOC_BIAS_OFF:
		 /*Low power sleep mode*/
		//zl38063_write_cmdreg(zl38063, ZL38063_CMD_APP_SLEEP);

		break;
	}


	TW_DEBUG1("level  is  %d  dir \n",level);
	zl38063ExitCritical();

	//codec->dapm.bias_level = level;
	return 0;
}

static int zl38063_suspend(struct snd_soc_codec *codec)
{
	zl38063_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int zl38063_resume(struct snd_soc_codec *codec)
{
	zl38063_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}

static void zl38063_sub_probe_clean(void);

static int locator_get(void *arg)
{
	//u16 buff;
	u16 ret,tmp=0,compens;

	zl38063EnterCritical();

	zl38063_hbi_wr16(zl38063_priv, ZL38063_BF_MIC_NUM, MIC_CONF_TRIANG |MIC_NUM_3);
	zl38063_hbi_wr16(zl38063_priv, ZL38063_BF_MIC_DISTANCE, 69);		//mic's construction distance
	zl38063_hbi_wr16(zl38063_priv, ZL38063_GPIO_DATA, 0x2000);
	zl38063_hbi_wr16(zl38063_sla, ZL38063_GPIO_DATA, 0x2000);

	zl38063ExitCritical();

	while(!kthread_should_stop()) {

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(zl38063_priv->beamforming.polling_time));

		zl38063EnterCritical();
		zl38063_hbi_rd16(zl38063_priv, ZL38063_SOUND_LOC_DIR, &ret);
		compens = (ret%30) > 15? 1:0;
		tmp = ret/30 + compens;

		//Step 1: Light up the LED based on DOA
		if ( (tmp&0x0f) != last_doa){

			zl38063_hbi_wr16(zl38063_priv, ZL38063_GPIO_DATA, 0x2000);
			zl38063_hbi_wr16(zl38063_sla, ZL38063_GPIO_DATA, 0x2000);

			switch(tmp&0x0f){
			case 0:
				zl38063_hbi_wr16(zl38063_priv, ZL38063_GPIO_DATA, MIC_0);break;
			case 1:
				zl38063_hbi_wr16(zl38063_priv, ZL38063_GPIO_DATA, MIC_30);break;
			case 2:
				zl38063_hbi_wr16(zl38063_sla, ZL38063_GPIO_DATA, MIC_60);break;
			case 3:
				zl38063_hbi_wr16(zl38063_sla, ZL38063_GPIO_DATA, MIC_90);break;
			case 4:
				zl38063_hbi_wr16(zl38063_priv, ZL38063_GPIO_DATA, MIC_120);break;
			case 5:
				zl38063_hbi_wr16(zl38063_priv, ZL38063_GPIO_DATA, MIC_150);break;
			case 6:
				zl38063_hbi_wr16(zl38063_sla, ZL38063_GPIO_DATA, MIC_180);break;
			case 7:
				zl38063_hbi_wr16(zl38063_sla, ZL38063_GPIO_DATA, MIC_210);break;
			case 8:
				zl38063_hbi_wr16(zl38063_priv, ZL38063_GPIO_DATA, MIC_240);break;
			case 9:
				zl38063_hbi_wr16(zl38063_priv, ZL38063_GPIO_DATA, MIC_270);break;
			case 10:
				zl38063_hbi_wr16(zl38063_sla, ZL38063_GPIO_DATA, MIC_300);break;
			case 11:
				zl38063_hbi_wr16(zl38063_sla, ZL38063_GPIO_DATA, MIC_330);break;
			}
		}
		last_doa = tmp&0x0f;

		//buff = buff -30 > 0? buff -30: 360 - buff;
		//printk("Offset to vin0 [%d]\n",buff);

#ifdef ZL38063_DOA_UPLOAD
		void *data;
		struct sk_buff *skb = NULL;
		struct nlmsghdr *nlhdr = NULL;
		struct nl_msg_audio *msg;

		msg =	kzalloc(sizeof(struct nl_msg_audio),GFP_KERNEL);
		skb = alloc_skb(NLMSG_SPACE(1024),GFP_KERNEL);

		zl38063_priv->degree = ret;
		msg->type = NL_MSG_DIR_CHANGE_NOTIFY;
		msg->cmd =NL_MSG_REV;
		msg->data[0] = zl38063_priv->degree;
		msg->port = zl38063_priv->pid ;
		msg->status = NL_CMD_STATUS_SUCCESS;

		//Step 2: send out the direction information
		skb = nlmsg_new(sizeof(struct nl_msg_audio), GFP_KERNEL);
		if (!skb) {
			printk("NETLINK ERR: Function nlmsgnl_msg_audio new failed!\n");
			return -1;
		}
		nlhdr = (struct nlmsghdr*)skb_put(skb, nlmsg_msg_size(sizeof(*msg)));
		nlhdr->nlmsg_type = NLMSG_NOOP;
		nlhdr->nlmsg_len = nlmsg_msg_size(sizeof(*msg));
		nlhdr->nlmsg_flags = 0;
		nlhdr->nlmsg_pid = zl38063_priv->pid;
		nlhdr->nlmsg_seq = 0;

		data = NLMSG_DATA(nlhdr);
		memcpy(data, (void *)msg, sizeof(*msg));
		netlink_unicast(zl38063_priv->nl_sock, skb, zl38063_priv->pid, 0);

		dev_kfree_skb(skb);
		kfree(msg);
#endif
		zl38063ExitCritical();

		TW_DEBUG1("my_net_link:send = %lu, message 0x%04x  msg->data %d degree\n",sizeof(*msg),zl38063_priv->degree,msg->data[0]);
	}

	return 0;
}


static int zl38063_nl_msg_unicast(struct nl_msg_audio *msg)
{
	void *data;
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlhdr = NULL;

	skb = nlmsg_new(sizeof(struct nl_msg_audio), GFP_KERNEL);
	if (!skb) {
		printk("NETLINK ERR: Function nlmsg_new failed!\n");
		return -1;
	}
	nlhdr = (struct nlmsghdr*)skb_put(skb, nlmsg_msg_size(sizeof(*msg)));
	nlhdr->nlmsg_type = NLMSG_NOOP;
	nlhdr->nlmsg_len = nlmsg_msg_size(sizeof(*msg));
	nlhdr->nlmsg_flags = 0;
	nlhdr->nlmsg_pid = zl38063_priv->pid;
	nlhdr->nlmsg_seq = 0;

	data = NLMSG_DATA(nlhdr);
	memcpy(data, (void *)msg, sizeof(*msg));

	netlink_unicast(zl38063_priv->nl_sock, skb, zl38063_priv->pid, 0);

	return 0;

}


static int zl38063_nl_send_string (struct nl_msg_audio *msg ,char *message)
{
	strcpy((char *)msg->data,message);
	return zl38063_nl_msg_unicast(msg);
}

static int zl38063_nl_get_dir (struct nl_msg_audio *msg)
{
	u16 reg=0;
	u8 soundLocEn =0;


	zl38063_hbi_rd16(zl38063_priv, ZL38063_SOUND_LOC_CTL, &reg);
	if (  reg ^ 0x0100 ){//check whether enable Sound Locator, if not ,enable it
		zl38063_hbi_wr16(zl38063_priv, ZL38063_SOUND_LOC_CTL, reg|0x0100);
		zl38063_reset(zl38063_priv, ZL38063_RST_SOFTWARE);
		soundLocEn = 1;
	}


	zl38063_hbi_rd16(zl38063_priv, ZL38063_SOUND_LOC_DIR, &(zl38063_priv->degree));

	if (zl38063_priv->degree > 180){
		msg->type = NL_MSG_SILENCE;
		zl38063_priv->degree = 0;
	}else{
		msg->type = NL_MSG_DIR_SEND;
		msg->data[0] = zl38063_priv->degree;
		msg->status = NL_CMD_STATUS_SUCCESS;
		msg->cmd = 0;
		msg->port = NETLINK_TEST;
	}
	if (1 == soundLocEn){
		zl38063_hbi_wr16(zl38063_priv, ZL38063_SOUND_LOC_CTL, reg&0xfeff);
		zl38063_reset(zl38063_priv, ZL38063_RST_SOFTWARE);
		soundLocEn = 0;
	}

	return zl38063_nl_msg_unicast(msg);
}

static int zl38063_get_beam_conf(u16 *data)
{
	int i=0;
	for (i=0;i<=7;){
		zl38063_hbi_rd16(zl38063_priv, i +ZL38063_BF_MIC_NUM, &data[i/2]);
		i+=2;
	}
	return 0;
}
/* USE this API to control polling frequence and enable default configuration for beamforming
* data[0]:    xxx ms pollling frequency, suggest > 100ms (TBD)
* data[1]:    full beamforming width in degree, 10,20,40,60
* data[2]:    noise level(TBD) LIKE_DEEP_NIGHT,	LIKE_OFFICE,LIKE_STREET,LIKE_COFFEE_BAR,
*/
static int zl38063_nl_handle_poll(struct nl_msg_audio *msg,enum NL_MSG_CMD cmd)
{
	u8 if_need_reset = 0;
	if (NL_MSG_GET_POLL_CONF == cmd){
		zl38063_get_beam_conf(msg->data +3);
		//memcpy(msg->data, &zl38063_priv->beamforming.polling_time, sizeof( *zl38063_priv->beamforming));
	}else if (NL_MSG_SET_POLL_CONF == cmd){
	//==============pollling frequency===============
		if (msg->data[0] >30)
			zl38063_priv->beamforming.polling_time = msg->data[0];
		else{
			zl38063_priv->beamforming.polling_time = 1000;
			zl38063_nl_send_string(msg,"Unsuggested polling frequence and set to default 1000ms");
		}

	//===============beamforming width==============
		if (msg->data[1] != msg->data[6] *2){
			if_need_reset = 1;
			zl38063_hbi_wr16(zl38063_priv, ZL38063_BF_HALF_WIDTH,msg->data[1] /2);
			}

	//=================noise level==================
		if (msg->data[2] != zl38063_priv->beamforming.noise_level){
			if_need_reset = 1;
			switch(msg->data[2]){
				case LIKE_DEEP_NIGHT:
					zl38063_hbi_wr16(zl38063_priv, ZL38063_BF_OOB_GAIN_MAX,0x002);		//2dB
					zl38063_hbi_wr16(zl38063_priv, ZL38063_BF_OOB_GAIN_MIX,0x000);		//0dB
					zl38063_hbi_wr16(zl38063_priv, ZL38063_SOUND_LOC_THD,0x0008);
					break;
				case LIKE_OFFICE:
					zl38063_hbi_wr16(zl38063_priv, ZL38063_BF_OOB_GAIN_MAX,0xFFFA);		//-6dB
					zl38063_hbi_wr16(zl38063_priv, ZL38063_BF_OOB_GAIN_MIX,0xFFF4);		//-12dB
					zl38063_hbi_wr16(zl38063_priv, ZL38063_SOUND_LOC_THD,0x00C8);
					break;

				case LIKE_STREET:
					zl38063_hbi_wr16(zl38063_priv, ZL38063_BF_OOB_GAIN_MAX,0xFFF4);		//-12dB
					zl38063_hbi_wr16(zl38063_priv, ZL38063_BF_OOB_GAIN_MIX,0xFFEC);		//-20dB
					zl38063_hbi_wr16(zl38063_priv, ZL38063_SOUND_LOC_THD,0x0258);
					break;

				case LIKE_COFFEE_BAR:
					zl38063_hbi_wr16(zl38063_priv, ZL38063_BF_OOB_GAIN_MAX,0xFFE8);		//-24dB
					zl38063_hbi_wr16(zl38063_priv, ZL38063_BF_OOB_GAIN_MIX,0xFFDC);		//-36dB
					zl38063_hbi_wr16(zl38063_priv, ZL38063_SOUND_LOC_THD,0x0388);
					break;
			}
		}
	}
	if (1 ==if_need_reset )
		zl38063_reset(zl38063_priv, ZL38063_RST_SOFTWARE);
	return 0;
}


static int zl38063_nl_get_version(struct nl_msg_audio *msg)
{
	u16 hwID = 0, prID = 0,fwVr = 0,fwID =0;

	zl38063_hbi_rd16(zl38063_priv, ZL38063_HW_VERSION, &hwID);
	zl38063_hbi_rd16(zl38063_priv, ZL38063_PRODUCT_CODE, &prID);
	zl38063_hbi_rd16(zl38063_priv, ZL38063_FM_VERSION, &fwVr);
	zl38063_hbi_rd16(zl38063_priv, ZL38063_FM_CUR, &fwID);

	snprintf((char *)msg->data, sizeof(msg->data),
		      "%5s%2d %3s%5d %3s%2d.%2d.%2d %3s %3d-%2d",
		      "hwID:", (u8)(hwID&0x001f),
		      "PR:", prID,
		      "FW:", (u8)(fwVr>>12), (u8)((fwVr>>8)&0x000f), (u8)(fwVr&0x00ff),
		      "ID:", (u8)((fwID>>8)&0x7f), (u8)(fwID&0x000f));
	msg->status = NL_CMD_STATUS_SUCCESS;
	return zl38063_nl_msg_unicast(msg);
}




/*-------------------------zl38063_nl_load_fw-----------------------------*/

/* ASR				check and load the ASR FW & CGF to device based on config.h & firmware.h
 * AEC				check and load the AEC FW & CGF to device based on config.h & firmware.h
 * UPDATE_PRAM		update a special FW to device, FW should be found in FS
 * UPDATE_CRAM		update a special CFG to device, FW should be found in FS
 * UPDATE_PRAM | UPDATE_CRAM
 *					update a special FW & CFG to device, FW should be found in FS
 * from ZL38063_FM_CUR can distinguish different FW(Current Loaded Firmware Image)
 */


static int zl38063_nl_load_fw(struct nl_msg_audio *msg)
{
	u16 ret = 0, fwID =0;

	msg->status = NL_CMD_STATUS_SUCCESS;
	zl38063_nl_send_string(msg, "start updating FW...");

	zl38063_hbi_rd16(zl38063_priv, ZL38063_FM_CUR, &fwID);
	printk("Currently working FWID is 0x%04x, CMD for FW operation is %d\n",fwID,msg->data[0]);

	switch(msg->data[0])
	{
		case ASR:
			if ( ZL38063_DEF_ASR_FWID == fwID){
				msg->status = NL_CMD_STATUS_SUCCESS;
				zl38063_nl_send_string(msg,"Target FW is same as Currently running FW");
			}else{
			zl38063_stop_fwr_to_bootmode(zl38063_priv);
				ret = zl38063_load_firmware(zl38063_priv,msg);
				if (ret<0){
					msg->status = NL_CMD_STATUS_FAIL;
					zl38063_nl_send_string(msg,"Chage to ASR FW error");
				}
			}
		break;
		case AEC:
			if ( ZL38063_DEF_AEC_FWID == fwID){
				msg->status = NL_CMD_STATUS_SUCCESS;
				zl38063_nl_send_string(msg,"Target FW is same as Currently running FW");
			}else{
				zl38063_stop_fwr_to_bootmode(zl38063_priv);
				ret = zl38063_load_firmware(zl38063_priv,msg);
				if (ret<0){
					msg->status = NL_CMD_STATUS_FAIL;
					zl38063_nl_send_string(msg,"Chage to AEC FW error");
				}
			}
		break;
		case UPDATE_PRAM:
			if ( ZL38063_DEF_ASR_FWID == fwID || ZL38063_DEF_AEC_FWID == fwID){
				msg->status = NL_CMD_STATUS_SUCCESS;
				zl38063_nl_send_string(msg,"Target FW is same as Currently running FW");
			}else{
				zl38063_stop_fwr_to_bootmode(zl38063_priv);
				ret = zl38063_load_firmware(zl38063_priv,msg);
				if (ret<0){
					msg->status = NL_CMD_STATUS_FAIL;
					zl38063_nl_send_string(msg,"UPDATE PRAM error");
				}
			}
		break;
		case UPDATE_CRAM:
			if ( ZL38063_DEF_ASR_FWID == fwID || ZL38063_DEF_AEC_FWID == fwID){
				msg->status = NL_CMD_STATUS_SUCCESS;
				zl38063_nl_send_string(msg,"Target FW is same as Currently running FW");
			}else{
				zl38063_stop_fwr_to_bootmode(zl38063_priv);
				ret = zl38063_load_firmware(zl38063_priv,msg);
				if (ret<0){
					msg->status = NL_CMD_STATUS_FAIL;
					zl38063_nl_send_string(msg,"UPDATE CRAM error");
				}
			}
		break;
		case UPDATE_PRAM | UPDATE_CRAM:
			if ( ZL38063_DEF_ASR_FWID == fwID || ZL38063_DEF_AEC_FWID == fwID){
				msg->status = NL_CMD_STATUS_SUCCESS;
				zl38063_nl_send_string(msg,"Target FW is same as Currently running FW");
			}else{
				zl38063_stop_fwr_to_bootmode(zl38063_priv);
				ret = zl38063_load_firmware(zl38063_priv,msg);
				if (ret<0){
					msg->status = NL_CMD_STATUS_FAIL;
					zl38063_nl_send_string(msg,"UPDATE RAM error");
				}
			}
		break;

		default:
			msg->status = NL_CMD_STATUS_FAIL;
			zl38063_nl_send_string(msg,"Unknow firmware or config option");
			return 0;

	}


	msg->status = NL_CMD_STATUS_SUCCESS;
	zl38063_nl_send_string(msg,"Successed updating FW");
	return 0;
}

/*----------------------------------------------------------------------*
 *   check the SCH for MIC direction
 *-------------------------zl38063_nl_handle_micpos-----------------------------*/
/*
* e.g. QFN  PIN 19 L channel for MIC1,
* PIN 19 R channel for MIC2,PIN 20 L channel for MIC3
* from MIC1 MIC2 MIC3 is 0 to 180 (from 0xA0) anticlockwise is forward direction
* NOTICE :
* 		For the first version to bringup, we just use U34 and U36 67mm's interval,master 2,3 MIC_ANTICLOCKWISE
* 		For future use, 6 MICs are all needed, 40mm's interval
*
*/
static int zl38063_nl_handle_micpos(struct nl_msg_audio *msg,enum NL_MSG_CMD cmd)
{
	if (NL_MSG_GET_MIC_DIR == cmd)
	{
		msg->data[0] = structural_offset;
		zl38063_hbi_rd16(zl38063_priv, ZL38063_SOUND_LOC_CTL, &msg->data[1]);
		if (msg->data[1] & 0x0010)
			msg->data[1] = MIC_CLOCKWISE;
		else
			msg->data[1] = MIC_ANTICLOCKWISE;
	}else if (NL_MSG_SET_MIC_DIR == cmd){
		structural_offset = msg->data[0];
		if (MIC_CLOCKWISE == msg->data[1]){
			zl38063_hbi_rd16(zl38063_priv, ZL38063_SOUND_LOC_CTL, &msg->data[2]);
			if ((msg->data[2] & 0x0010) == 0 )
				zl38063_hbi_wr16(zl38063_priv, ZL38063_SOUND_LOC_CTL, msg->data[2]| 0x0010);
		}else if (MIC_ANTICLOCKWISE == msg->data[1]){
			zl38063_hbi_rd16(zl38063_priv, ZL38063_SOUND_LOC_CTL, &msg->data[2]);
			if ((msg->data[2] & 0x0010) > 0)
				zl38063_hbi_wr16(zl38063_priv, ZL38063_SOUND_LOC_CTL, msg->data[2]| 0x0010);
		}else{
			msg->status = NL_CMD_STATUS_FAIL;
			zl38063_nl_send_string(msg,"Unsupport Mic position type");
		}
	}
	return 0;
}

static int zl38063_nl_handle_reg(struct nl_msg_audio *msg,enum NL_MSG_CMD cmd)
{
	s8 len =0;
	u8 i = 2;
	u16 addr = msg->data[0];
	if (NL_MSG_READ_REG == cmd)
	{
		len = (s8)msg->data[1];
		if ( len < 0 || addr > ZL38063_MAX_REG)
		{
			msg->status = NL_CMD_STATUS_FAIL;
			zl38063_nl_send_string(msg,"Invalid read data");
			return -1;
		}
		for (;len>0;len--,i++){
			zl38063_hbi_rd16(zl38063_priv, addr, &msg->data[i]);
			if (i>18){
				msg->data[1] = 18;
				msg->status = NL_CMD_STATUS_SUCCESS;
				zl38063_nl_msg_unicast(msg);
				memset(msg->data +1, 0, 19);
				i=2;
			}
		}
		msg->data[1] = i-3;
		msg->data[i] = 0;
		msg->status = NL_CMD_STATUS_SUCCESS;
		zl38063_nl_msg_unicast(msg);
	}
	else if ( NL_MSG_WRITE_REG == cmd)
	{
		len = (s8)msg->data[1];
		if ( len < 0 || addr > ZL38063_MAX_REG)
		{
			msg->status = NL_CMD_STATUS_FAIL;
			zl38063_nl_send_string(msg,"Invalid write data");
			return -1;
		}
		for (;len>0;len--,i++){
			zl38063_hbi_wr16(zl38063_priv, addr, msg->data[i]);
			if (i>19){
				msg->data[1] = i -2;
				break;		//just for test
			}
		}
	}
	return 0;
}


static int zl38063_nl_clt_LED(struct nl_msg_audio *msg)
{
	u16 led_mt = 0,led_st = 0,ret = 0;

	ret = msg->data[0] ;
	TW_DEBUG1("send date 0x%x\n",ret);

	if (ret & LED_ALL_OFF)
	{
		TW_DEBUG1("Controls All led off\n");
		zl38063_hbi_wr16(zl38063_priv,ZL38063_GPIO_DATA,0x2000 |MIC_ALL_OFF);
		if(bd_type == 1) {zl38063_hbi_wr16(zl38063_sla,ZL38063_GPIO_DATA,MIC_ALL_OFF);}
		return 0;
	}
	if (ret & LED_ALL_ON)
	{
		TW_DEBUG1("Controls All led on\n");
		zl38063_hbi_wr16(zl38063_priv,ZL38063_GPIO_DATA,0x2000 |MIC_ALL_ON);
		if(bd_type == 1) {zl38063_hbi_wr16(zl38063_sla,ZL38063_GPIO_DATA,MIC_ALL_ON);}
		return 0;
	}

	if(bd_type == 1)
	{
		if (ret & LED1_ON){led_mt |= MIC_0 ;}
		if (ret & LED2_ON){led_mt |= MIC_30;}
		if (ret & LED3_ON){led_st |= MIC_60 ;}
		if (ret & LED4_ON){led_st |= MIC_90;}
		if (ret & LED5_ON){led_mt |= MIC_120 ;}
		if (ret & LED6_ON){led_mt |= MIC_150;}
		if (ret & LED7_ON){led_st |= MIC_180 ;}
		if (ret & LED8_ON){led_st |= MIC_210;}
		if (ret & LED9_ON){led_mt |= MIC_240 ;}
		if (ret & LED10_ON){led_mt |= MIC_270;}
		if (ret & LED11_ON){led_st |= MIC_300 ;}
		if (ret & LED12_ON){led_st |= MIC_330;}

		zl38063_hbi_wr16(zl38063_priv,ZL38063_GPIO_DATA,0x2000 |led_mt);		//config master led
		zl38063_hbi_wr16(zl38063_sla,ZL38063_GPIO_DATA,led_st);					//config slave led
	}else{
		if (ret & LED1_ON){led_mt |= MIC_0 ;}
		if (ret & LED2_ON){led_mt |= MIC_30;}
		zl38063_hbi_wr16(zl38063_priv,ZL38063_GPIO_DATA,0x2000 |led_mt);		//config master led
	}

	return 0;
}


/*----------------------------------------------------------------------*
 *   handle the CMD get from host
 *-------------------------nl_recv_msg_handler-----------------------------*/

/* NL_MSG_REV				send no cmd by error or test socket
 * NL_MSG_GET_DIR			get currently direction
 * NL_MSG_GET_POLL_CONF		get the direction info of polling
 * NL_MSG_SET_POLL_CONF		setup the direction info of polling
 * NL_MSG_GET_FW_VERSION	get the running Firware veriosn
 * NL_MSG_LOAD_FW			update / change current running FW
 * NL_MSG_GET_MIC_DIR		get the degree offset between two MIC's centerline and VIN0
 * NL_MSG_SET_MIC_DIR		set the degree offset between two MIC's centerline and VIN0
 * NL_MSG_READ_REG			use for debug, read a single or serial 18 registers
 * NL_MSG_WRITE_REG			use for debug, write a single or serial 18 registers
 */
static void nl_recv_msg_handler(struct sk_buff * __skb)
{
	struct nlmsghdr * nlhdr = NULL;
	int msg_len =0;
	int len=0;
	struct sk_buff *skb;
	struct nl_msg_audio *msg;

	skb = skb_get (__skb);
	msg =	kzalloc(sizeof(struct nl_msg_audio),GFP_KERNEL);
	nlhdr = nlmsg_hdr(skb);
	len = skb->len;

	memcpy(msg, NLMSG_DATA(nlhdr), sizeof(*msg));

	zl38063_priv->pid = nlhdr->nlmsg_pid;

	TW_DEBUG1("\tmsg->type[%d]\tmsg->port[%d]\t\n",msg->type,msg->port);


	TW_DEBUG1("Message received:   app_pid = %d\n",nlhdr->nlmsg_pid) ;

	for(; NLMSG_OK(nlhdr, len); nlhdr = NLMSG_NEXT(nlhdr, len))
	{
		if (nlhdr->nlmsg_len < sizeof(struct nlmsghdr)) {
					printk("NETLINK ERR: Corruptted msg!\n");
					continue;
				}
				msg_len = nlhdr->nlmsg_len - NLMSG_LENGTH(0);
				if (msg_len < sizeof(*msg)) {
					printk("NETLINK ERR: Incorrect msg!\n");
					continue;
				}
				memcpy(msg, NLMSG_DATA(nlhdr), sizeof(*msg));
				if (NETLINK_CB(skb).portid != nlhdr->nlmsg_pid) {
					printk("NETLINK ERR: Unknown msg, port %u, skb portid %u!\n",
					msg->port, NETLINK_CB(skb).portid);
					continue;
				}

				if (  NL_MSG_HOST_CMD ==  msg->type){
				switch(msg->cmd){
					case NL_MSG_REV:
						zl38063EnterCritical();
						zl38063_nl_send_string(msg,"Connect net_link ok");
						zl38063ExitCritical();
					break;
					case NL_MSG_GET_DIR:
						zl38063EnterCritical();
							zl38063_nl_get_dir(msg);
						zl38063ExitCritical();
					break;
					case NL_MSG_GET_POLL_CONF:
						zl38063EnterCritical();
							zl38063_nl_handle_poll(msg,NL_MSG_GET_POLL_CONF);
						zl38063ExitCritical();
					break;
					case NL_MSG_SET_POLL_CONF:
						zl38063EnterCritical();
							zl38063_nl_handle_poll(msg,NL_MSG_SET_POLL_CONF);
						zl38063ExitCritical();
					break;
					case NL_MSG_GET_FW_VERSION:
						zl38063EnterCritical();
							zl38063_nl_get_version(msg);
						zl38063ExitCritical();
					break;
					case NL_MSG_LOAD_FW:
						zl38063EnterCritical();
							zl38063_nl_load_fw(msg);
						zl38063ExitCritical();
					break;
					case NL_MSG_GET_MIC_DIR:
						zl38063EnterCritical();
							zl38063_nl_handle_micpos(msg,NL_MSG_GET_MIC_DIR);
						zl38063ExitCritical();
					break;
					case NL_MSG_SET_MIC_DIR:
						zl38063EnterCritical();
							zl38063_nl_handle_micpos(msg,NL_MSG_SET_MIC_DIR);
						zl38063ExitCritical();
					break;
					case NL_MSG_READ_REG:
						zl38063EnterCritical();
							zl38063_nl_handle_reg(msg,NL_MSG_READ_REG);
						zl38063ExitCritical();
					break;
					case NL_MSG_WRITE_REG:
						zl38063EnterCritical();
							zl38063_nl_handle_reg(msg,NL_MSG_WRITE_REG);
						zl38063ExitCritical();
					break;
					case NL_MSG_WAKEUP:
						zl38063EnterCritical();
							zl38063_nl_clt_LED(msg);
						zl38063ExitCritical();
					break;
				   }
			}else if (NL_MSG_DIR_CHANGE_NOTIFY !=  msg->type){
				printk("NETLINK ERR: Unknown msg type[%d]\n",msg->type);
				continue;
			}
		}

	//kfree_skb(skb);
	//kfree(nlhdr);
	kfree(msg);

}


static int zl38063_probe(struct snd_soc_codec *codec)
{
	struct netlink_kernel_cfg *cfg;

	zl38063_priv->nl_init = 0;
	zl38063_priv->nl_port = NETLINK_TEST;
	cfg = kzalloc(sizeof(struct netlink_kernel_cfg),GFP_KERNEL);
	cfg->groups = 0;
	cfg->input = nl_recv_msg_handler;

	zl38063_priv->nl_sock = netlink_kernel_create(&init_net,
		zl38063_priv->nl_port, cfg);

	if(!zl38063_priv->nl_sock){
		printk(KERN_ERR "my_net_link: create netlink socket error.\n");
		return -1;
	}

	zl38063_priv->beamforming.polling_time  = 300;		//ms
	zl38063_priv->beamforming.offset_degree = GENERAL;
	zl38063_priv->beamforming.noise_level	 = LIKE_OFFICE;
	zl38063_priv->nl_init = 1;

	if(doa == 1 && zl38063_priv->fw_label == DOA)
		{zl38063_priv->kthread = kthread_run(locator_get,NULL,"locator_get");
	}

	dev_info(codec->dev, "Probing zl38063 SoC CODEC driver\n");
	return zl38063_add_controls(codec);
	//return 0;
}

static int zl38063_remove(struct snd_soc_codec *codec)
{
	struct zl38063 *zl38063 = snd_soc_codec_get_drvdata(codec);

	TW_DEBUG1("Release GPIOs");
	if(doa ==1 && zl38063_priv->fw_label == DOA)
		{kthread_stop(zl38063_priv->kthread);}

	if (zl38063->rst_pin > 0) {
		gpio_direction_output(zl38063->rst_pin, zl38063->pwr_active);
		gpio_free(zl38063->rst_pin);
	}
	if (zl38063->pwr_pin > 0) {
		gpio_direction_output(zl38063->pwr_pin, zl38063->pwr_active);
		gpio_free(zl38063->pwr_pin);
	}

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_zl38063 = {
	.probe =	zl38063_probe,
	.remove =	zl38063_remove,
	.suspend =	zl38063_suspend,
	.resume =	zl38063_resume,
	.read  = zl38063_reg_read,
	.write = zl38063_reg_write,
	.set_bias_level = zl38063_set_bias_level,
	.reg_cache_size = CODEC_CONFIG_REG_NUM,
	.reg_word_size = sizeof(u16),
	.reg_cache_default = reg_cache_default,
	.reg_cache_step = 2,
};
EXPORT_SYMBOL(soc_codec_dev_zl38063);
#endif /*ZL380XX_TW_ENABLE_ALSA_CODEC_DRIVER*/

/*--------------------------------------------------------------------
 *    ALSA  SOC CODEC driver  - END
 *--------------------------------------------------------------------*/

static struct of_device_id zl38063_of_match[] = {
	{ .compatible = "ambarella,zl38063",},
	{},
};
static struct of_device_id zl38063_of_match_slave[] = {
	{ .compatible = "zl_slave",},
	{},
};



MODULE_DEVICE_TABLE(of, zl38063_of_match);


static int zl38063_sub_probe(void)
{
	AecFirmware.havePrgmBase = 1;
	AecFirmware.prgmBase = 0x00080200;
	AecFirmware.execAddr = 0x00084040;
	AecFirmware.FirmwareStreamLen = 3947;

	AsrFirmware.havePrgmBase = 1;
	AsrFirmware.prgmBase = 0x00080200;
	AsrFirmware.execAddr = 0x00084040;
	AsrFirmware.FirmwareStreamLen = 2453;

	DoaFirmware.havePrgmBase = 1;
	DoaFirmware.prgmBase = 0x00080200;
	DoaFirmware.execAddr = 0x00084040;
	DoaFirmware.FirmwareStreamLen = 2453;


	/* Allocate driver data */
	zl38063_priv = kzalloc(sizeof(*zl38063_priv), GFP_KERNEL);
	if (zl38063_priv == NULL)
		return -ENOMEM;

	/* Allocating memory for the data buffer */
	zl38063_priv->pData  = kmalloc(twHBImaxTransferSize, GFP_KERNEL);
	if (!zl38063_priv->pData) {
		printk(KERN_ERR "Error allocating %d bytes pdata memory",
												 twHBImaxTransferSize);
		kfree(zl38063_priv);
		return -ENOMEM;
	}
	memset(zl38063_priv->pData, 0, twHBImaxTransferSize);
	return 0;
}
/*============== Load firmwre and config ==========
* try to find whether there was a supported flash
* 	if here had one , boot from flash as a faster speed
* 	else boot from built in
*/

static int zl38063_boot_mode(struct zl38063 *zl38063)
{
	int status = 0;
	TW_DEBUG1("Enter  Load firmwre and config ...\n");

	if ( (fw_path == NULL && cr_path == NULL) || boot_to_BootMode == 1 )
		zl38063_load_firmware(zl38063,NULL);
	else
		status = zl38063_load_file(zl38063);

	if (status <0)
	{
		TW_DEBUG1("zl38063_load_file error and change to Default firmwre and config ...\n");
		zl38063_load_firmware(zl38063,NULL);
	}
	zl38063_reset(zl38063, ZL38063_RST_SOFTWARE);
	msleep(50); /*required for the reset to cmplete*/

	return 0;


}

static void zl38063_sub_probe_clean(void)
{
		kfree(zl38063_priv);
		kfree((void *)zl38063_priv->pData);
		zl38063_priv->pData = NULL;
}


#ifdef ZL38063_DEV_BIND_SLAVE_TO_MASTER
/* Bind the client driver to a master (SPI or I2C) adapter */

static int zl380xx_slaveMaster_binding(void) {
#ifdef ZL38063_SPI_IF
	struct spi_board_info spi_device_info = {
		.modalias = "zl38063",
		.max_speed_hz = SPIM_CLK_SPEED,
		.bus_num = SPIM_BUS_NUM,
		.chip_select = SPIM_CHIP_SELECT,
		.mode = SPIM_MODE,
	};
	struct spi_device *client;
	struct spi_master *master;
	/* create a new slave device, given the master and device info
	 * if another device is alread assigned to that bus.cs, then this will fail
	 */
	/* get the master device, given SPI the bus number*/
	master = spi_busnum_to_master( spi_device_info.bus_num );
	if( !master )
		return -ENODEV;
	printk(KERN_INFO "SPI master device found at bus = %d\n", master->bus_num);

	client = spi_new_device( master, &spi_device_info );
	if( !client )
		return -ENODEV;
	printk(KERN_INFO "SPI slave device %s, module alias %s added at bus = %d, cs =%d\n",
		   client->dev.driver->name, client->modalias, client->master->bus_num, client->chip_select);
#endif
#ifdef ZL38063_I2C_IF /*#ifdef ZL38063_I2C_IF*/
	struct i2c_board_info i2c_device_info = {
		I2C_BOARD_INFO("zl38063", MICROSEMI_I2C_ADDR),
		.platform_data	= NULL,
	};
	struct i2c_client   *client;
	/*create a new client for that adapter*/
	/*get the i2c adapter*/
	struct i2c_adapter *master = i2c_get_adapter(CONTROLLER_I2C_BUS_NUM);
	if( !master )
		return -ENODEV;
	printk(KERN_INFO "found I2C master device %s \n", master->name);

	client = i2c_new_device( master, &i2c_device_info);
	if( !client)
		return -ENODEV;

	printk(KERN_INFO "I2C slave device %s, module alias %s attached to I2C master device %s\n",
		   client->dev.driver->name, client->name, master->name);

	i2c_put_adapter(master);
#endif /*ZL38063_SPI_IF/I2C interface selection macro*/

	return 0;

}

static int zl38063_slaveMaster_unbind(void)
{
#ifdef ZL38063_SPI_IF
		spi_unregister_device(zl38063_priv->spi);
#endif
#ifdef ZL38063_I2C_IF /*#ifdef ZL38063_I2C_IF*/
		i2c_unregister_device(zl38063_priv->i2c);
#endif

	return 0;
}

#endif //ZL38063_DEV_BIND_SLAVE_TO_MASTER
/*--------------------------------------------------------------------
* SPI driver registration
*--------------------------------------------------------------------*/

static int get_dts (struct spi_device *spi)
{
	int ret =0;
	enum of_gpio_flags flags;

	zl38063_priv->pwr_pin = of_get_named_gpio_flags(spi->dev.of_node, "zl38063,pwr-gpio", 0, &flags);
	zl38063_priv->pwr_active = !!(flags & OF_GPIO_ACTIVE_LOW);
	if(zl38063_priv->pwr_pin < 0 || !gpio_is_valid(zl38063_priv->pwr_pin)) {
		printk(KERN_ERR "zl380 power pin is invalid,If no pwr gpio is needed, just remove this\n");
		zl38063_priv->pwr_pin = -1;
		return -1;
	}
	ret = devm_gpio_request(&spi->dev, zl38063_priv->pwr_pin, "zl38063 pwr");
	if(ret < 0) {
		dev_err(&spi->dev, "Failed to request pwr pin GPIO[%d] for zl38063: %d\n",  zl38063_priv->pwr_pin,ret);
		return ret;
	}
	gpio_direction_output(zl38063_priv->pwr_pin, !zl38063_priv->pwr_active);


	zl38063_priv->rst_pin = of_get_named_gpio_flags(spi->dev.of_node, "zl38063,rst-gpio", 0, &flags);
	zl38063_priv->rst_active = !!(flags & OF_GPIO_ACTIVE_LOW);
	if(zl38063_priv->rst_pin < 0 || !gpio_is_valid(zl38063_priv->rst_pin)) {
		printk(KERN_ERR "zl38063 reset pin is invalid,If no reset gpio is needed, just remove this\n");
		zl38063_priv->rst_pin = -1;
		return -1;
	}
	ret = devm_gpio_request(&spi->dev, zl38063_priv->rst_pin, "zl38063 reset");
	devm_gpio_request(&spi->dev, zl38063_priv->rst_pin, "zl38063 slave reset");
	if(ret < 0) {
		dev_err(&spi->dev, "Failed to request reset pin GPIO[%d] for zl380xx: %d\n",zl38063_priv->rst_pin, ret);
		return ret;
	}
	gpio_direction_output(zl38063_priv->rst_pin, zl38063_priv->rst_active);

	return 0;

}

#ifdef ZL38063_SPI_IF

static int zl38063_slave_probe(struct spi_device *spi)
{
	int err;

	TW_DEBUG1("Probing zl38063_slave_probe spi[%d] device\n",spi->master->bus_num);
	TW_DEBUG1("Spi mode[%d]  max_speed_hz[%d] chip_select[%d] bits_per_word[%d]  cs_gpio[%d]\n",
				spi->mode,spi->max_speed_hz,spi->chip_select,spi->bits_per_word,spi->cs_gpio);

	zl38063_sla= kzalloc(sizeof(*zl38063_sla), GFP_KERNEL);
	if (zl38063_sla == NULL)
		return -ENOMEM;

	/* Allocating memory for the data buffer */
	zl38063_sla->pData  = kmalloc(twHBImaxTransferSize, GFP_KERNEL);
	if (!zl38063_sla->pData) {
		printk(KERN_ERR "Error allocating %d bytes pdata memory",
												 twHBImaxTransferSize);
		kfree(zl38063_sla);
		return -ENOMEM;
	}
	memset(zl38063_sla->pData, 0, twHBImaxTransferSize);

	spi_setup(spi);
	/* Initialize the driver data */
	spi_set_drvdata(spi, zl38063_sla);
	zl38063_sla->spi = spi;
	zl38063_sla->rst_pin = 130;

	err = gpio_request( zl38063_sla->rst_pin, "zl38063 slave reset");
	if(err < 0) {
		dev_err(&spi->dev, "Failed to request pwr pin GPIO[%d] for zl38063: %d\n",	zl38063_sla->rst_pin,err);
		return err;
	}
	gpio_direction_output(zl38063_sla->rst_pin, 1);
	msleep(10);

	zl38063_boot_mode(zl38063_sla);
	zl38063_hbi_wr16(zl38063_sla,ZL38063_GPIO_DIR,0x3fdc);		//config GPIO direction
	zl38063_hbi_wr16(zl38063_sla,ZL38063_GPIO_DATA,MIC_330 |MIC_180 |MIC_210);		//config GPIO Level

	return 0;

}
static int  zl38063_slave_remove(struct spi_device *spi)
{

	TW_DEBUG1("Free zl38063_slave_remove register");

	snd_soc_unregister_codec(&spi->dev);
	kfree(spi_get_drvdata(spi));
	kfree((void *)zl38063_sla->pData);
	zl38063_sla->pData = NULL;

	return 0;
}



static int zl38063_spi_probe(struct spi_device *spi)
{

	int err;
	u16 buff = 0;

	err = zl38063_sub_probe();
	if (err < 0)
		return err;
	spi->mode = SPI_MODE_0;        //req by datasheet

	TW_DEBUG1("Probing zl38063 spi[%d] device\n",spi->master->bus_num);
	TW_DEBUG1("Spi mode[%d]  max_speed_hz[%d] chip_select[%d] bits_per_word[%d] cs_gpio[%d]\n",
				spi->mode,spi->max_speed_hz,spi->chip_select,spi->bits_per_word,spi->cs_gpio);

	err = spi_setup(spi);
	if (err < 0) {
		printk(KERN_ERR "zl38063_spi_probe set up spi fail\n");
		zl38063_sub_probe_clean();
		return err;
	}
	/* Initialize the driver data */
	spi_set_drvdata(spi, zl38063_priv);
	zl38063_priv->spi = spi;

	mutex_init(&zl38063_priv->lock);

	if (get_dts(spi) < 0)
		return -1;
	TW_DEBUG1( "zl380 power up\n");
	gpio_direction_output(zl38063_priv->pwr_pin, !zl38063_priv->pwr_active);
	msleep(80);
	gpio_direction_output(zl38063_priv->rst_pin, !zl38063_priv->rst_active);


	zl38063_hbi_wr16(zl38063_priv,0x34,0xd3d3);
	zl38063_state_init(zl38063_priv, (HBI_CONFIG_VAL | HBI_CONFIG_WAKE));
	msleep(10);
	/*Clear the wake up bit*/
	zl38063_state_init(zl38063_priv, HBI_CONFIG_VAL);

	zl38063_priv->fw_label = DOA;			//use this flat to choose ASR or AEC cr

	err = snd_soc_register_codec(&spi->dev, &soc_codec_dev_zl38063, &zl38063_dai, 1);
	if(err < 0) {
		zl38063_sub_probe_clean();
		dev_dbg(&spi->dev, "zl38063 spi device not created!!!\n");
		return err;
	}else{

		zl38063_hbi_rd16(zl38063_priv, ZL38063_DEVICE_ID_REG, &buff);
		TW_DEBUG1("ZL38063_DEVICE_ID_REG get is  0x%04x\n",buff);
		if((buff > 38000)  && (buff < 38090)) {
			zl38063_priv->init_done =1;
			printk("SPI slave device ZL%d found at bus::cs = %d::%d\n", buff, spi->master->bus_num, spi->chip_select);
		} else {
			zl38063_priv->init_done =0;
			err =zl38063_boot_mode(zl38063_priv);
			if ( err< 0 )
				return err;
			zl38063_priv->init_done =1;
		}

		zl38063_hbi_wr16(zl38063_priv,ZL38063_GPIO_DIR,0x3fdc);		//config GPIO direction
		zl38063_hbi_wr16(zl38063_priv,ZL38063_GPIO_DATA,0x2000 |MIC_0 |MIC_30 |MIC_150);		//config GPIO Level

		zl38063_hbi_rd16(zl38063_priv, 0x20, &buff);
		TW_DEBUG1("Hardware Revision Code is  0x%04x\n",buff);
		return 0;
	}


}

static int  zl38063_spi_remove(struct spi_device *spi)
{

	TW_DEBUG1("Free SPI register");

#ifdef ZL380XX_TW_ENABLE_ALSA_CODEC_DRIVER
	snd_soc_unregister_codec(&spi->dev);
#endif
	kfree(spi_get_drvdata(spi));

	kfree((void *)zl38063_priv->pData);
	zl38063_priv->pData = NULL;
	return 0;
}

static const struct spi_device_id zl38063_spi_id[] = {
	{ "zl38063", 0 },
	{ }
};

static const struct spi_device_id zl38063_slave_id[] = {
	{ "zl_slave", 0 },
	{ }
};


static struct spi_driver zl38063_spi_driver[] = {
	{
		.driver = {
		.name = "zl38063",
		.owner = THIS_MODULE,
#ifdef SUPPORT_LINUX_DEVICE_TREE_OF_MATCHING
		.of_match_table = of_match_ptr(zl38063_of_match),
#endif
	},
		.probe = zl38063_spi_probe,
		.remove = zl38063_spi_remove,
		.id_table = zl38063_spi_id,
	},

	{
		.driver = {
			.name = "zl_slave",
			.owner = THIS_MODULE,
			.of_match_table = of_match_ptr(zl38063_of_match_slave),
		},
		.probe = zl38063_slave_probe,
		.remove = zl38063_slave_remove,
		.id_table = zl38063_slave_id,
	},

};

#endif


#ifdef ZL38063_I2C_IF

static int zl38063_i2c_probe(struct i2c_client *i2c,
				const struct i2c_device_id *id)
{
	int err = zl38063_sub_probe();
	if (err < 0) {
		return err;
	}

	TW_DEBUG1(&i2c->dev, "probing zl38063 I2C device\n");
	//i2c->addr = MICROSEMI_I2C_ADDR;

	i2c_set_clientdata(i2c, zl38063_priv);
	zl38063_priv->i2c = i2c;
	TW_DEBUG1("i2c slave device address = 0x%04x\n", i2c->addr);


		u16 buf = 0;

		zl38063_priv->fw_label = AEC;

		zl38063_hbi_rd16(zl38063_priv, ZL38063_DEVICE_ID_REG, &buf);
		if((buf > 38000)  && (buf < 38090)) {
			  zl38063_priv->init_done =1;
			  printk(KERN_ERR "I2C slave device %d found at address = 0x%04x\n", buf, i2c->addr);

		} else {
			  zl38063_priv->init_done =0;
		}


	err = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_zl38063, &zl38063_dai, 1);
	if(err < 0) {
		zl38063_sub_probe_clean();
		dev_dbg(&i2c->dev, "zl38063 I2c device not created!!!\n");
		return err;
	}

	if (zl38063_ldfwr(zl38063_priv) < 0) {
		dev_dbg(&i2c->dev, "error loading the firmware into the codec\n");
		snd_soc_unregister_codec(&i2c->dev);
		zl38063_sub_probe_clean();
		return -ENODEV;
	}
	dev_dbg(&i2c->dev, "zl38063 I2C codec device created...\n");
	return err;
}

static int zl38063_i2c_remove(struct i2c_client *i2c)
{

#ifdef ZL380XX_TW_ENABLE_ALSA_CODEC_DRIVER
	snd_soc_unregister_codec(&i2c->dev);
#endif
	kfree(i2c_get_clientdata(i2c));

	   kfree((void *)zl38063_priv->pData);
	   zl38063_priv->pData = NULL;
	return 0;
}

static struct i2c_device_id zl38063_id_table[] = {
	{"zl38063", 0 },
	{}
 };

static struct i2c_driver zl38063_i2c_driver = {
	.driver = {
		.name	= "zl38063",
		.owner = THIS_MODULE,
#ifdef SUPPORT_LINUX_DEVICE_TREE_OF_MATCHING
		.of_match_table = zl38063_of_match,
#endif
	},
	.probe = zl38063_i2c_probe,
	.remove = zl38063_i2c_remove,
	.id_table = zl38063_id_table,
};

#endif


static int __init zl38063_init(void)
{
	int status;
	struct device_node *root;

	struct property *prop ;

	root = of_find_node_by_path("/");
	if (root == NULL) {
		printk("%s %d: cannot find node [root]\n",__FUNCTION__,__LINE__);
	}
	prop= of_find_property(root, "model", NULL);
	status = strcmp(AMBA_ZATANNA_BOARD,(const char* )prop->value);
	TW_DEBUG1("There is Board : %s  status[%d]\n",(char* )prop->value,status);
	if (status ==0) {bd_type = 1;}
	of_node_put(root);

#ifdef ZL38063_SPI_IF
	status = spi_register_driver(&zl38063_spi_driver[0]);
	if (bd_type ==1)
		{status = spi_register_driver(&zl38063_spi_driver[1]);}


#endif
#ifdef ZL38063_I2C_IF
	status = i2c_add_driver(&zl38063_i2c_driver);
#endif
	if (status < 0) {
		printk(KERN_ERR "Failed to register zl38063   driver: %d\n", status);
	}

#ifdef ZL38063_DEV_BIND_SLAVE_TO_MASTER
	status =  zl380xx_slaveMaster_binding();
	if (status < 0) {
		printk(KERN_ERR "error =%d\n", status);

#ifdef ZL38063_SPI_IF
		spi_unregister_driver(&zl38063_spi_driver[0]);
#endif
#ifdef ZL38063_I2C_IF
		i2c_del_driver(&zl38063_i2c_driver);
#endif
		return -1;
	}
	status =0;
#endif
	return status;
}
module_init(zl38063_init);

static void __exit zl38063_exit(void)
{
#ifdef ZL38063_DEV_BIND_SLAVE_TO_MASTER
	zl38063_slaveMaster_unbind();
#endif
#ifdef ZL38063_SPI_IF
	spi_unregister_driver(&zl38063_spi_driver[0]);
#endif
#ifdef ZL38063_I2C_IF
	i2c_del_driver(&zl38063_i2c_driver);
#endif

}
module_exit(zl38063_exit);

MODULE_AUTHOR("Jean Bony <jean.bony@microsemi.com>");
MODULE_DESCRIPTION(" Microsemi Timberwolf i2c/spi/char/alsa codec driver");
MODULE_LICENSE("GPL");
