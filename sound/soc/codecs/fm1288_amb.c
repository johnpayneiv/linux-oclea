/*
 * fm1288_amb.c  --  audio driver for FM1288
 *
 * Copyright 2014 Ambarella Ltd.
 *
 * Author: Diao Chengdong <cddiao@ambarella.com>
 *
 * History:
 *	2014/03/27 - created
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "fm1288_amb.h"

#undef FM1288_DEBUG			//used at debug mode

#define FM1288_I2C_ADDR 		0x60 //0xC0
#define FM1288_I2C_SYNC_BYTE1	0xFC
#define FM1288_I2C_SYNC_BYTE2	0xF3
#define FM1288_CMD_ENTRY_MEMW	0x3B
#define FM1288_CMD_ENTRY_MEMR	0x37
#define FM1288_CMD_ENTRY_REGR	0x60

#define FM1288_READ_DATA_PORT_L	0x25
#define FM1288_READ_DATA_PORT_H	0x26

#define FM1288_CMD_ECHOC_RESET1	0x68
#define FM1288_CMD_ECHOC_RESET2	0x64
#define FM1288_CMD_ECHOC_RESET3	0x00

#define FM1288_CMD_ECHOC_MEMW1	0x0D
#define FM1288_CMD_ECHOC_MEMW2	0x3F

#ifdef FM1288_DEBUG
#define fmdbgprt printk
#else
#define fmdbgprt(format, arg...) do {} while (0)
#endif

/* FM1288 Codec Private Data */
struct fm1288_priv {
	unsigned int rst_pin;
	unsigned int rst_active;
	unsigned int sysclk;
	unsigned int clkid;
	//struct regmap *regmap;
	struct i2c_client* i2c_clt;
	//u8 reg_cache[FM1288_MAX_REGISTERS];
	int onStereo;
	int mic;
	u8 fmt;

    unsigned int amp_gpio;
    unsigned int amp_active;
};

static inline unsigned int fm1288_codec_read(struct snd_soc_component *component,
	unsigned int reg)
{
	struct fm1288_priv *fm1288 = snd_soc_component_get_drvdata(component);
	int ret=0;
	struct i2c_msg msgs[2];
	u8 pread_lowbuf[1];
	u8 pread_highbuf[1];

	//Step 1 buf
	u8 pbuf[] = {
		FM1288_I2C_SYNC_BYTE1,
		FM1288_I2C_SYNC_BYTE2,
		FM1288_CMD_ENTRY_MEMR,
		0x00, 0x00,
	};

	//Step 2 buf
    u8 pbuf1[] = {
		FM1288_I2C_SYNC_BYTE1,
		FM1288_I2C_SYNC_BYTE2,
		FM1288_CMD_ENTRY_REGR,
		FM1288_READ_DATA_PORT_L,
	};

	//Step 3 buf
	u8 pbuf2[] = {
		FM1288_I2C_SYNC_BYTE1,
		FM1288_I2C_SYNC_BYTE2,
		FM1288_CMD_ENTRY_REGR,
		FM1288_READ_DATA_PORT_H,
	};

	//Step 1
	pbuf[3] = (reg & 0xff00)>>8;
	pbuf[4] = (reg & 0x00ff);

	msgs[0].len = 5;
	msgs[0].addr = fm1288->i2c_clt->addr;
	msgs[0].flags = fm1288->i2c_clt->flags;
	msgs[0].buf = pbuf;

	ret = i2c_transfer(fm1288->i2c_clt->adapter, msgs, 1);
	if (ret < 0) {
		printk("FM1288 i2c_transfer failed(%d):\n", ret);
		return ret;
	}

	//Step 2
	msgs[0].len = 4;
	msgs[0].addr = fm1288->i2c_clt->addr;
	msgs[0].flags = fm1288->i2c_clt->flags;
	msgs[0].buf = pbuf1;

	msgs[1].addr = fm1288->i2c_clt->addr;
	msgs[1].flags = fm1288->i2c_clt->flags | I2C_M_RD;
	msgs[1].buf = pread_lowbuf;
	msgs[1].len = 1;

	ret = i2c_transfer(fm1288->i2c_clt->adapter, msgs, 2);
	if (ret < 0) {
		printk("FM1288 i2c_transfer failed(%d):\n", ret);
		return ret;
	}

	//Step 3
	msgs[0].len = 4;
	msgs[0].addr = fm1288->i2c_clt->addr;
	msgs[0].flags = fm1288->i2c_clt->flags;
	msgs[0].buf = pbuf2;

	msgs[1].addr = fm1288->i2c_clt->addr;
	msgs[1].flags = fm1288->i2c_clt->flags | I2C_M_RD;
	msgs[1].buf = pread_highbuf;
	msgs[1].len = 1;

	ret = i2c_transfer(fm1288->i2c_clt->adapter, msgs, 2);
	if (ret < 0) {
		printk("FM1822 i2c_transfer failed(%d):\n", ret);
		return ret;
	}

	return ret;
}

static inline int fm1288_codec_write(struct snd_soc_component *component, unsigned int reg,
	unsigned int value)
{
	struct fm1288_priv *fm1288 =  snd_soc_component_get_drvdata(component);
	int ret=0;
	struct i2c_msg msgs[1];
		
	u8 pbuf[] = {
		FM1288_I2C_SYNC_BYTE1, 
		FM1288_I2C_SYNC_BYTE2, 
		FM1288_CMD_ENTRY_MEMW, 
		0x00, 0x00, 
		0x00, 0x00
	};
	pbuf[3] = (reg & 0xff00)>>8;
	pbuf[4] = (reg & 0x00ff);
	pbuf[5] = (value & 0xff00)>>8;
	pbuf[6] = (value & 0x00ff);
	
	msgs[0].len = 7;
	msgs[0].addr = fm1288->i2c_clt->addr;
	msgs[0].flags = fm1288->i2c_clt->flags;
	msgs[0].buf = pbuf;	

	//printk("fm1288 codec write reg 0x%04x val 0x%04x\n",reg, value);
	
	ret = i2c_transfer(fm1288->i2c_clt->adapter, msgs, 1);
	if (ret < 0) {
		//printk("failed(%d): [0x%x:0x%x]\n", ret, subaddr, data);
		return ret;
	}
	return ret;
}

static inline int fm1288_codec_echo_reg_reset(struct snd_soc_component *component)
{
	struct fm1288_priv *fm1288 = snd_soc_component_get_drvdata(component);
	int ret=0;
	//struct i2c_client *client;
	struct i2c_msg msgs[1];
		
	u8 pbuf[] = {
		FM1288_I2C_SYNC_BYTE1, 
		FM1288_I2C_SYNC_BYTE2, 
		FM1288_CMD_ECHOC_RESET1, 
		FM1288_CMD_ECHOC_RESET2,
		FM1288_CMD_ECHOC_RESET3
	};
	
	msgs[0].len = 5;
	msgs[0].addr = fm1288->i2c_clt->addr;
	msgs[0].flags = fm1288->i2c_clt->flags;
	msgs[0].buf = pbuf;	

	//printk("fm1288_codec_echo_reg_reset 0xFCF3686400");
	
	ret = i2c_transfer(fm1288->i2c_clt->adapter, msgs, 1);
	if (ret < 0) {
		printk("fm1288_codec_echo_reg_reset failed(%d)", ret);
	}
	return ret;
}

static inline int fm1288_codec_echo_cancellation_reg_write(struct snd_soc_component *component, unsigned int reg,
	unsigned int value)
{
	struct fm1288_priv *fm1288 = snd_soc_component_get_drvdata(component);	
	int ret=0;
	struct i2c_msg msgs[1];
		
	u8 pbuf[] = {
		FM1288_I2C_SYNC_BYTE1, 
		FM1288_I2C_SYNC_BYTE2, 
		FM1288_CMD_ECHOC_MEMW1, 
		FM1288_CMD_ECHOC_MEMW2,
		0x00, 0x00, 
		0x00, 0x00
	};
	pbuf[4] = (reg & 0xff00)>>8;
	pbuf[5] = (reg & 0x00ff);
	pbuf[6] = (value & 0xff00)>>8;
	pbuf[7] = (value & 0x00ff);
	
	msgs[0].len = 8;
	msgs[0].addr = fm1288->i2c_clt->addr;
	msgs[0].flags = fm1288->i2c_clt->flags;
	msgs[0].buf = pbuf;

	ret = i2c_transfer(fm1288->i2c_clt->adapter, msgs, 1);
	if (ret < 0) {
		printk("fm1288_codec_echo_cancellation_reg_write failed(%d)\n", ret);
		return ret;
	}

	return ret;
}

/*
 *  MIC Gain control:
 * from 6 to 26 dB in 6.5 dB steps
 *
 * MIC Gain control:
 *
 * max : 0x7fff: 20dB
 * min : 0x00: +0 dB
 * from 0 to 20DB
 */
static DECLARE_TLV_DB_SCALE(mgain_tlv, 0, 32767, 1);

/*
 * Speaker output volume control:
 *
 * max : 0x7fff: 20dB
 * min : 0x00: +0 dB
 * from 0 to 20DB
 */
static DECLARE_TLV_DB_SCALE(spkout_tlv, 0, 32767, 1);

static const struct snd_kcontrol_new fm1288_snd_controls[] = {
	SOC_SINGLE_TLV("Mic Gain Control",
			FM1288_230C_MIC_VOL, 0, 0x7fff, 0, mgain_tlv),
	SOC_SINGLE_TLV("Speaker Output Volume",
			FM1288_230D_SPK_VOL, 0, 0x7fff, 0, spkout_tlv),
};

static int fm1288_spklo_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event) //CONFIG_LINF
{
	//printk("\t[FM1288] %s(%d)\n",__FUNCTION__,__LINE__);

	switch (event) {
		case SND_SOC_DAPM_PRE_PMU:	/* before widget power up */
			break;
		case SND_SOC_DAPM_POST_PMU:	/* after widget power up */
			printk("\t[FM1288] %s SND_SOC_DAPM_POST_PMU wait=300msec\n",__FUNCTION__);
			mdelay(300);
			break;
		case SND_SOC_DAPM_PRE_PMD:	/* before widget power down */
			printk("\t[FM1288] %s SND_SOC_DAPM_PRE_PMD wait=300msec\n",__FUNCTION__);
			mdelay(1);
			break;
		case SND_SOC_DAPM_POST_PMD:	/* after widget power down */
			printk("\t[FM1288] %s SND_SOC_DAPM_POST_PMD wait=300msec\n",__FUNCTION__);
			mdelay(300);
			break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget fm1288_dapm_widgets[] = {
      SND_SOC_DAPM_SPK("Speaker", fm1288_spklo_event),
};


static const struct snd_soc_dapm_route fm1288_intercon[] = {
};

static int fm1288_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct fm1288_priv *fm1288 = snd_soc_component_get_drvdata(component);

//	switch (params_format(params)) {
//		case SNDRV_PCM_FORMAT_S16_LE:
			fm1288->fmt = 2 << 4;
//			break;
//		case SNDRV_PCM_FORMAT_S24_LE:
//			fm1288->fmt = 3 << 4;
//			break;
//		case SNDRV_PCM_FORMAT_S32_LE:
//			fm1288->fmt = 3 << 4;
//			break;
//		default:
//			printk(codec->dev, "Can not support the format");
//			return -EINVAL;
//	}

	return 0;
}


static int fm1288_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
		unsigned int freq, int dir)
{
	struct snd_soc_component *component = dai->component;
	struct fm1288_priv *fm1288 = snd_soc_component_get_drvdata(component);

	fm1288->sysclk = 12288000; //18432000; //4096000; //27000000; //freq;
	fm1288->clkid = clk_id;

	printk("fm1288->sysclk 0x%x, clk_id 0x%xf", fm1288->sysclk, fm1288->clkid);
	return 0;
}

static int fm1288_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	return 0;
}

// * for FM1288
static int fm1288_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	int 	ret = 0;
 //   struct snd_soc_codec *codec = codec_dai->codec;
    struct snd_soc_component *component = dai->component;
    struct fm1288_priv *fm1288 = snd_soc_component_get_drvdata(component);

	printk("\t[FM1288] %s(%d)\n",__FUNCTION__,__LINE__);

    if(fm1288->amp_gpio > 0) {
        switch (cmd) {
        case SNDRV_PCM_TRIGGER_START:
        case SNDRV_PCM_TRIGGER_RESUME:
        case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
            if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
                printk("[FM1288] amp on\n");
                gpio_direction_output(fm1288->amp_gpio, fm1288->amp_active);
            }
            break;
        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
        case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
            if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
                printk("[FM1288] amp off\n");
                gpio_direction_output(fm1288->amp_gpio, !fm1288->amp_active);
            }
            break;
        default:
            break;
        }

    }

	return ret;
}


static int fm1288_set_bias_level(struct snd_soc_component *component,
		enum snd_soc_bias_level level)
{
	u8 reg;
    struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);

	printk("\t[FM1288] %s(%d)\n",__FUNCTION__,__LINE__);

	switch (level) {
	case SND_SOC_BIAS_ON:
        printk("bias on\n");
	case SND_SOC_BIAS_PREPARE:
        printk("bias prepare\n");
	case SND_SOC_BIAS_STANDBY:
        printk("bias standby\n");
		msleep(250);
		break;
	case SND_SOC_BIAS_OFF:
		break;
    default:
        break;
	}
    dapm->bias_level = level;
	return 0;
}

#define FM1288_RATES		(SNDRV_PCM_RATE_16000)
#define FM1288_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE)

static struct snd_soc_dai_ops fm1288_dai_ops = {
	.hw_params	= fm1288_hw_params,
	.set_sysclk	= fm1288_set_dai_sysclk,
	.set_fmt	= fm1288_set_dai_fmt,
	.trigger = fm1288_trigger,
};

struct snd_soc_dai_driver fm1288_dai[] = {
	{
		.name = "fm1288-hifi",
		.playback = {
		       .stream_name = "Playback",
		       .channels_min = 1,
		       .channels_max = 2,
		       .rates = FM1288_RATES,
		       .formats = FM1288_FORMATS,
		},
		.capture = {
		       .stream_name = "Capture",
		       .channels_min = 1,
		       .channels_max = 2,
		       .rates = FM1288_RATES,
		       .formats = FM1288_FORMATS,
		},
		.ops = &fm1288_dai_ops,
	},
};

static int fm1288_probe(struct snd_soc_component *component)
{
	struct fm1288_priv *fm1288 = snd_soc_component_get_drvdata(component);
	int ret = 0;

	printk("\t[FM1288] %s(%d)\n",__FUNCTION__,__LINE__);

	ret = devm_gpio_request(component->dev, fm1288->rst_pin, "fm1288 reset");
	if (ret < 0){
		dev_err(component->dev, "Failed to request rst_pin: %d\n", ret);
		return ret;
	}

	/* Reset FM1288 codec */
	gpio_direction_output(fm1288->rst_pin, fm1288->rst_active);
	msleep(1);
	gpio_direction_output(fm1288->rst_pin, !fm1288->rst_active);

    if (fm1288->amp_gpio >= 0) {
        ret = devm_gpio_request(component->dev, fm1288->amp_gpio, "fm1288 amp_gpio");
        if ( ret < 0 ) {
            dev_err(component->dev, "Failed to request amp_pin: %d\n", ret);
        }
    }

	fm1288_set_bias_level(component, SND_SOC_BIAS_STANDBY);
	printk("\t[FM1288 bias] %s(%d)\n",__FUNCTION__,__LINE__);

	fm1288->onStereo = 0;
	fm1288->mic = 1;

#if 1 //16K sound with echo cancellation

		// echo cancellation Part
    fm1288_codec_echo_reg_reset(component);	
    fm1288_codec_echo_cancellation_reg_write(component, 0x8090, 0x943E);
    fm1288_codec_echo_cancellation_reg_write(component, 0x8193, 0x83DE);
    fm1288_codec_echo_cancellation_reg_write(component, 0x8219, 0x2ABF);
    fm1288_codec_echo_cancellation_reg_write(component, 0x8340, 0x0508);
    fm1288_codec_echo_cancellation_reg_write(component, 0x8419, 0x3E7F);
    fm1288_codec_echo_cancellation_reg_write(component, 0x8580, 0x952A);
    fm1288_codec_echo_cancellation_reg_write(component, 0x862A, 0x7AAA);
    fm1288_codec_echo_cancellation_reg_write(component, 0x8719, 0x284F);
    fm1288_codec_echo_cancellation_reg_write(component, 0x8880, 0x943A);
    fm1288_codec_echo_cancellation_reg_write(component, 0x8982, 0x32D1);
    fm1288_codec_echo_cancellation_reg_write(component, 0x8A26, 0x790F);
    fm1288_codec_echo_cancellation_reg_write(component, 0x8B19, 0x2A80);
    fm1288_codec_echo_cancellation_reg_write(component, 0x8C80, 0x95BA);
    fm1288_codec_echo_cancellation_reg_write(component, 0x8D68, 0x00A1);
    fm1288_codec_echo_cancellation_reg_write(component, 0x8E94, 0x96D0);
    fm1288_codec_echo_cancellation_reg_write(component, 0x8F90, 0x95BE);
    fm1288_codec_echo_cancellation_reg_write(component, 0x9019, 0x2ABF);
    fm1288_codec_echo_cancellation_reg_write(component, 0x9140, 0xFA0A);
    fm1288_codec_echo_cancellation_reg_write(component, 0x9241, 0x770C);
    fm1288_codec_echo_cancellation_reg_write(component, 0x9341, 0xF40B);
    fm1288_codec_echo_cancellation_reg_write(component, 0x9442, 0xB11F);
    fm1288_codec_echo_cancellation_reg_write(component, 0x9542, 0xEE0E);
    fm1288_codec_echo_cancellation_reg_write(component, 0x9682, 0x3011);
    fm1288_codec_echo_cancellation_reg_write(component, 0x9727, 0x911F);
    fm1288_codec_echo_cancellation_reg_write(component, 0x9822, 0x7C01);
    fm1288_codec_echo_cancellation_reg_write(component, 0x9927, 0x915F);
    fm1288_codec_echo_cancellation_reg_write(component, 0x9A22, 0x7B01);
    fm1288_codec_echo_cancellation_reg_write(component, 0x9B27, 0x919F);
    fm1288_codec_echo_cancellation_reg_write(component, 0x9C22, 0x7F01);
    fm1288_codec_echo_cancellation_reg_write(component, 0x9D27, 0x91DF);
    fm1288_codec_echo_cancellation_reg_write(component, 0x9E22, 0x7E01);
    fm1288_codec_echo_cancellation_reg_write(component, 0x9F18, 0x34BF);
    fm1288_codec_echo_cancellation_reg_write(component, 0xA022, 0x7F9F);
    fm1288_codec_echo_cancellation_reg_write(component, 0xA119, 0xB05F);
    fm1288_codec_echo_cancellation_reg_write(component, 0xA241, 0x000A);
    fm1288_codec_echo_cancellation_reg_write(component, 0xA390, 0x96DA);
    fm1288_codec_echo_cancellation_reg_write(component, 0xA482, 0x2C4A);
    fm1288_codec_echo_cancellation_reg_write(component, 0xA523, 0xA25F);
    fm1288_codec_echo_cancellation_reg_write(component, 0xA692, 0x2C4A);
    fm1288_codec_echo_cancellation_reg_write(component, 0xA782, 0x325A);
    fm1288_codec_echo_cancellation_reg_write(component, 0xA819, 0x764F);
    fm1288_codec_echo_cancellation_reg_write(component, 0xA982, 0x2F2A);
    fm1288_codec_echo_cancellation_reg_write(component, 0xAA40, 0x0385);
    fm1288_codec_echo_cancellation_reg_write(component, 0xAB26, 0xEA0F);
    fm1288_codec_echo_cancellation_reg_write(component, 0xAC22, 0x0802);
    fm1288_codec_echo_cancellation_reg_write(component, 0xAD92, 0x2F2A);
    fm1288_codec_echo_cancellation_reg_write(component, 0xAE18, 0x283F);
    fm1288_codec_echo_reg_reset(component);		
		
	  fm1288_codec_write(component, 0x3FA0, 0x92AA);
	  fm1288_codec_write(component, 0x3FB0, 0x3F80);
	  fm1288_codec_write(component, 0x3FA1, 0x93E6);
	  fm1288_codec_write(component, 0x3FB1, 0x3F83);
	  fm1288_codec_write(component, 0x3FA2, 0x9282);
	  fm1288_codec_write(component, 0x3FB2, 0x3F85);
	  fm1288_codec_write(component, 0x3FA3, 0x92A7);
	  fm1288_codec_write(component, 0x3FB3, 0x3F88);
	  fm1288_codec_write(component, 0x3FA4, 0x8349);
	  fm1288_codec_write(component, 0x3FB4, 0x3F91);
	  fm1288_codec_write(component, 0x3FA5, 0x9B04);
	  fm1288_codec_write(component, 0x3FB5, 0x3FA0);
	  fm1288_codec_write(component, 0x3FA6, 0x9763);
	  fm1288_codec_write(component, 0x3FB6, 0x3FA2);
	  fm1288_codec_write(component, 0x3FA7, 0x8282);
	  fm1288_codec_write(component, 0x3FB7, 0x3FA9);
		// - echo cancellation Part

		fm1288_codec_write(component, 0x22C8, 0x0026); //26 is for 12.288MHz
		fm1288_codec_write(component, 0x22E5, 0x0226);
		fm1288_codec_write(component, 0x22F5, 0x8000);
		fm1288_codec_write(component, 0x22FA, 0x002d); //2d is line in disabled
		fm1288_codec_write(component, 0x22F8, 0x8002);
		fm1288_codec_write(component, 0x236E, 0x1800);
		fm1288_codec_write(component, 0x236F, 0x0800);
	
		//spk related
		fm1288_codec_write(component, 0x22e9, 0x0002); // -2DB 0x0002 1.91V // 0DB 0x0001 2.40V // +2 DB 0x0000 3V);
		fm1288_codec_write(component, 0x230D, 0x4fff); // maximum volume is 0x7fff//		

		fm1288_codec_write(component, 0x230C, 0x0200);
		fm1288_codec_write(component, 0x2303, 0x0DF9);
		fm1288_codec_write(component, 0x2304, 0x03CF);
		fm1288_codec_write(component, 0x2305, 0x0001);
		fm1288_codec_write(component, 0x23DC, 0x0600);
		fm1288_codec_write(component, 0x232F, 0x0040);
		fm1288_codec_write(component, 0x2339, 0x0003);
		fm1288_codec_write(component, 0x23B3, 0x0008);
		fm1288_codec_write(component, 0x23B4, 0x0003);
		fm1288_codec_write(component, 0x23E9, 0x3000);
		fm1288_codec_write(component, 0x23CF, 0x1000);
		fm1288_codec_write(component, 0x23D0, 0x0600);
		fm1288_codec_write(component, 0x23BE, 0x0200);
		fm1288_codec_write(component, 0x22F2, 0x0030);
		fm1288_codec_write(component, 0x23BF, 0x0140);
		fm1288_codec_write(component, 0x2333, 0x0010);
		fm1288_codec_write(component, 0x2332, 0x0040);
		fm1288_codec_write(component, 0x23E7, 0x0900);
		fm1288_codec_write(component, 0x23E8, 0x2500);
		fm1288_codec_write(component, 0x23EA, 0x6000);
		fm1288_codec_write(component, 0x23D5, 0x3800);
		fm1288_codec_write(component, 0x2349, 0x2000);
		fm1288_codec_write(component, 0x2348, 0x2000);
		fm1288_codec_write(component, 0x23B7, 0x0003);
		fm1288_codec_write(component, 0x23B5, 0x7800);
		fm1288_codec_write(component, 0x23BB, 0x2800);
		fm1288_codec_write(component, 0x2310, 0x120D);
		fm1288_codec_write(component, 0x22C4, 0x0622);
		fm1288_codec_write(component, 0x2301, 0x0002);		
		fm1288_codec_write(component, 0x22FB, 0x0000);
	
		//Read back for checking
		fm1288_codec_read(component, 0x22C8);
		fm1288_codec_read(component, 0x22E5);
		fm1288_codec_read(component, 0x22F5);
		fm1288_codec_read(component, 0x22FA);
		fm1288_codec_read(component, 0x22F8);
		fm1288_codec_read(component, 0x236E);
		fm1288_codec_read(component, 0x236F);
		fm1288_codec_read(component, 0x230C);
		fm1288_codec_read(component, 0x2303);
		fm1288_codec_read(component, 0x2304);
		fm1288_codec_read(component, 0x230D);
		fm1288_codec_read(component, 0x2305);
		fm1288_codec_read(component, 0x23DC);
		fm1288_codec_read(component, 0x232F);
		fm1288_codec_read(component, 0x2339);
		fm1288_codec_read(component, 0x23B3);
		fm1288_codec_read(component, 0x23B4);
		fm1288_codec_read(component, 0x23E9);
		fm1288_codec_read(component, 0x23CF);
		fm1288_codec_read(component, 0x23D0);
		fm1288_codec_read(component, 0x23BE);
		fm1288_codec_read(component, 0x22F2);
		fm1288_codec_read(component, 0x23BF);
		fm1288_codec_read(component, 0x2333);
		fm1288_codec_read(component, 0x2332);
		fm1288_codec_read(component, 0x23E7);
		fm1288_codec_read(component, 0x23E8);
		fm1288_codec_read(component, 0x23EA);
		fm1288_codec_read(component, 0x23D5);
		fm1288_codec_read(component, 0x2349);
		fm1288_codec_read(component, 0x2348);
		fm1288_codec_read(component, 0x23B7);
		fm1288_codec_read(component, 0x23B5);
		fm1288_codec_read(component, 0x23BB);
		fm1288_codec_read(component, 0x2310);
		fm1288_codec_read(component, 0x22C4);
		fm1288_codec_read(component, 0x2301);		
		fm1288_codec_read(component, 0x22FB);
#else //8k setting
		fm1288_codec_write(component, 0x22C8, 0x0026); //26 is for 12.288MHz
		fm1288_codec_write(component, 0x22E5, 0x0226);
		fm1288_codec_write(component, 0x22F5, 0x8000);
		fm1288_codec_write(component, 0x22FA, 0x002d); //2d is line in disabled
		fm1288_codec_write(component, 0x22F8, 0x8000);
		fm1288_codec_write(component, 0x236E, 0x1800);
		fm1288_codec_write(component, 0x236F, 0x0800);
	
		//spk
		fm1288_codec_write(component, 0x22e9, 0x0002); // -2DB 0x0002 1.91V // 0DB 0x0001 2.40V // +2 DB 0x0000 3V);
		fm1288_codec_write(component, 0x230D, 0x4fff); // maximum volume is 0x7fff//
				
		fm1288_codec_write(component, 0x230C, 0x0200);
		fm1288_codec_write(component, 0x2303, 0x0DF9);
		fm1288_codec_write(component, 0x2304, 0x03CF);
		fm1288_codec_write(component, 0x2305, 0x0001);
		fm1288_codec_write(component, 0x23DC, 0x0600);
		fm1288_codec_write(component, 0x232F, 0x0040);
		fm1288_codec_write(component, 0x2339, 0x0003);
		fm1288_codec_write(component, 0x23B3, 0x0008);
		fm1288_codec_write(component, 0x23B4, 0x0003);
		fm1288_codec_write(component, 0x23E9, 0x3000);
		fm1288_codec_write(component, 0x23CF, 0x1000);
		fm1288_codec_write(component, 0x23D0, 0x0600);
		fm1288_codec_write(component, 0x23BE, 0x0200);
		fm1288_codec_write(component, 0x22F2, 0x0030);
		fm1288_codec_write(component, 0x23BF, 0x0140);
		fm1288_codec_write(component, 0x2333, 0x0010);
		fm1288_codec_write(component, 0x2332, 0x0040);
		fm1288_codec_write(component, 0x23E7, 0x0900);
		fm1288_codec_write(component, 0x23E8, 0x2500);
		fm1288_codec_write(component, 0x23EA, 0x6000);
		fm1288_codec_write(component, 0x23D5, 0x3800);
		fm1288_codec_write(component, 0x2349, 0x2000);
		fm1288_codec_write(component, 0x2348, 0x2000);
		fm1288_codec_write(component, 0x23B7, 0x0003);
		fm1288_codec_write(component, 0x23B5, 0x7800);
		fm1288_codec_write(component, 0x23BB, 0x2800);
		fm1288_codec_write(component, 0x2310, 0x120D);
		fm1288_codec_write(component, 0x22C4, 0x0622);
		fm1288_codec_write(component, 0x22FB, 0x0000);
	
		//Read back
		fm1288_codec_read(component, 0x22C8);
		fm1288_codec_read(component, 0x22E5);
		fm1288_codec_read(component, 0x22F5);
		fm1288_codec_read(component, 0x22FA);
		fm1288_codec_read(component, 0x22F8);
		fm1288_codec_read(component, 0x236E);
		fm1288_codec_read(component, 0x236F);
		fm1288_codec_read(component, 0x230C);
		fm1288_codec_read(component, 0x2303);
		fm1288_codec_read(component, 0x2304);
		fm1288_codec_read(component, 0x230D);
		fm1288_codec_read(component, 0x2305);
		fm1288_codec_read(component, 0x23DC);
		fm1288_codec_read(component, 0x232F);
		fm1288_codec_read(component, 0x2339);
		fm1288_codec_read(component, 0x23B3);
		fm1288_codec_read(component, 0x23B4);
		fm1288_codec_read(component, 0x23E9);
		fm1288_codec_read(component, 0x23CF);
		fm1288_codec_read(component, 0x23D0);
		fm1288_codec_read(component, 0x23BE);
		fm1288_codec_read(component, 0x22F2);
		fm1288_codec_read(component, 0x23BF);
		fm1288_codec_read(component, 0x2333);
		fm1288_codec_read(component, 0x2332);
		fm1288_codec_read(component, 0x23E7);
		fm1288_codec_read(component, 0x23E8);
		fm1288_codec_read(component, 0x23EA);
		fm1288_codec_read(component, 0x23D5);
		fm1288_codec_read(component, 0x2349);
		fm1288_codec_read(component, 0x2348);
		fm1288_codec_read(component, 0x23B7);
		fm1288_codec_read(component, 0x23B5);
		fm1288_codec_read(component, 0x23BB);
		fm1288_codec_read(component, 0x2310);
		fm1288_codec_read(component, 0x22C4);
		fm1288_codec_read(component, 0x22FB);
#endif
    return ret;

}

static void fm1288_remove(struct snd_soc_component *component)
{
	printk("\t[FM1288] %s(%d)\n",__FUNCTION__,__LINE__);
	fm1288_set_bias_level(component, SND_SOC_BIAS_OFF);
}

static int fm1288_suspend(struct snd_soc_component *component)
{
	struct fm1288_priv *fm1288 = snd_soc_component_get_drvdata(component);
	int i;

	//for(i = 0; i < 7; i++) {
	//	fm1288->reg_cache[i] = snd_soc_read(codec, i);
	//}

	fm1288_set_bias_level(component, SND_SOC_BIAS_OFF);

	return 0;
}

static int fm1288_resume(struct snd_soc_component *component)
{
    struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);
	fm1288_set_bias_level(component, dapm->bias_level);

	return 0;
}


struct snd_soc_component_driver soc_component_dev_fm1288 = {
	.probe = fm1288_probe,
	.remove = fm1288_remove,
	.suspend =	fm1288_suspend,
	.resume =	fm1288_resume,

	.read =		fm1288_codec_read,
	.write =	fm1288_codec_write,

	.set_bias_level = fm1288_set_bias_level,

    .controls = fm1288_snd_controls,
    .num_controls = ARRAY_SIZE(fm1288_snd_controls),
    .dapm_widgets = fm1288_dapm_widgets,
    .num_dapm_widgets = ARRAY_SIZE(fm1288_dapm_widgets),
    .dapm_routes = fm1288_intercon,
    .num_dapm_routes = ARRAY_SIZE(fm1288_intercon),

    .idle_bias_on       = 1,
    .use_pmdown_time    = 1,
    .endianness     = 1,
    .non_legacy_dai_naming  = 1,
};

EXPORT_SYMBOL_GPL(soc_component_dev_fm1288);

static int fm1288_i2c_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
	struct device_node *np = i2c->dev.of_node;
	struct fm1288_priv *fm1288;
	enum of_gpio_flags flags;
	int rst_pin;
	int ret = 0;

	printk("\t[FM1288] %s(%d)\n",__FUNCTION__,__LINE__);

	fm1288 = devm_kzalloc(&i2c->dev, sizeof(struct fm1288_priv), GFP_KERNEL);
	if (fm1288 == NULL)
		return -ENOMEM;

    printk("[FM1288] get gpio flags\n");

	rst_pin = of_get_gpio_flags(np, 0, &flags);
	if (rst_pin < 0 || !gpio_is_valid(rst_pin))
		return -ENXIO;

    printk("[FM1288] register codec\n");

	fm1288->i2c_clt = i2c;
	fm1288->rst_pin = rst_pin;
	fm1288->rst_active = !!(flags & OF_GPIO_ACTIVE_LOW);

    fm1288->amp_gpio = of_get_named_gpio_flags(np, "amp-gpio", 0, &flags);
    fm1288->amp_active = !!(flags & OF_GPIO_ACTIVE_LOW);

    if (fm1288->amp_gpio < 0 || !gpio_is_valid(fm1288->amp_gpio)) {
        fm1288->amp_gpio = -1;
        printk("fm1288 amplifier pin(%u) is invalid\n", ret);
    }

	i2c_set_clientdata(i2c, fm1288);

	ret = snd_soc_register_component(&i2c->dev,
			&soc_component_dev_fm1288, &fm1288_dai[0], ARRAY_SIZE(fm1288_dai));
	if (ret < 0){
		kfree(fm1288);
		printk("\t[FM1288 Error!] %s(%d)\n",__FUNCTION__,__LINE__);
	}

    printk("[FM1288] probed\n");
	return ret;
}

static int fm1288_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_component(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static struct of_device_id fm1288_of_match[] = {
	{ .compatible = "ambarella,fm1288",},
	{},
};
MODULE_DEVICE_TABLE(of, fm1288_of_match);

static const struct i2c_device_id fm1288_i2c_id[] = {
	{ "fm1288", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fm1288_i2c_id);

static struct i2c_driver fm1288_i2c_driver = {
	.driver = {
		.name = "fm1288-codec",
		.owner = THIS_MODULE,
		.of_match_table = fm1288_of_match,
	},
	.probe		=	fm1288_i2c_probe,
	.remove		=	fm1288_i2c_remove,
	.id_table	=	fm1288_i2c_id,
};

static int __init fm1288_modinit(void)
{
	int ret;
	printk("\t[FM1288] %s(%d)\n", __FUNCTION__,__LINE__);

	ret = i2c_add_driver(&fm1288_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register fm1288 I2C driver: %d\n",ret);
	return ret;
}

module_init(fm1288_modinit);

static void __exit fm1288_exit(void)
{
	i2c_del_driver(&fm1288_i2c_driver);
}
module_exit(fm1288_exit);

MODULE_DESCRIPTION("Soc FM1288 driver");
MODULE_AUTHOR("Diao Chengdong<cddiao@ambarella.com>");
MODULE_LICENSE("GPL");

