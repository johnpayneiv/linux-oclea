/*
 * ak4957.c  --  audio driver for AK4957
 *
 * Copyright (C) 2014 Asahi Kasei Microdevices Corporation
 *  Author                Date        Revision
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                      14/05/08	    1.1
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/pm.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "ak4957.h"

//#define AK4957_DEBUG			//used at debug mode
//#define AK4957_CONTIF_DEBUG	//used at debug mode

#ifdef AK4957_DEBUG
#define akdbgprt printk
#else
#define akdbgprt(format, arg...) do {} while (0)
#endif

/* AK4957 Codec Private Data */
struct ak4957_priv {
	struct snd_soc_codec *codec;
	struct i2c_client* i2c_clt;
	u8 reg_cache[AK4957_MAX_REGISTERS];
	int rst_pin;
	int rst_active;
	int pwr_pin;
	int pwr_active;
	unsigned int sysclk;
	unsigned int clkid;
	int mic;
	u8 fmt;
	u8 rate;
	int onStereo;
	int mGainL;
	int mGainR;
	int fs2;
};

static struct ak4957_priv *ak4957_data;

/* ak4957 register cache & default register settings */
static struct reg_default ak4957_reg[AK4957_MAX_REGISTERS] = {
	{0x0 , 0x00 },	/*	0x00	AK4957_00_POWER_MANAGEMENT1 		*/
	{0x1 , 0x00 },	/*	0x01	AK4957_01_POWER_MANAGEMENT2 		*/
	{0x2 , 0x04 },	/*	0x02	AK4957_02_SIGNAL_SELECT1			*/
	{0x3 , 0x00 },	/*	0x03	AK4957_03_SIGNAL_SELECT2			*/
	{0x4 , 0x01 },	/*	0x04	AK4957_04_SIGNAL_SELECT3			*/
	{0x5 , 0xC2 },	/*	0x05	AK4957_05_MODE_CONTROL1 			*/
	{0x6 , 0x00 },	/*	0x06	AK4957_06_MODE_CONTROL2 			*/
	{0x7 , 0x0C },	/*	0x07	AK4957_07_MODE_CONTROL3 			*/
	{0x8 , 0x00 },	/*	0x08	AK4957_08_DIGITL_MIC				*/
	{0x9 , 0x00 },	/*	0x09	AK4957_09_TIMER_SELECT				*/
	{0xA , 0x71 },	/*	0x0A	AK4957_0A_ALC_TIMER_SELECT			*/
	{0xB , 0x81 },	/*	0x0B	AK4957_0B_ALC_MODE_CONTROL1 		*/
	{0xC , 0xE1 },	/*	0x0C	AK4957_0C_ALC_MODE_CONTROL2 		*/
	{0xD , 0x28 },	/*	0x0D	AK4957_0D_ALC_MODE_CONTROL3 		*/
	{0xE , 0x91 },	/*	0x0E	AK4957_0E_ALC_VOLUME				*/
	{0xF , 0x91 },	/*	0x0F	AK4957_0F_LCH_INPUT_VOLUME_CONTROL	*/
	{0x10, 0xE1 },	/*	0x10	AK4957_10_RCH_INPUT_VOLUME_CONTROL	*/
	{0x11, 0x91 },	/*	0x11	AK4957_11_LCH_OUTPUT_VOLUME_CONTROL */
	{0x12, 0x91 },	/*	0x12	AK4957_12_RCH_OUTPUT_VOLUME_CONTROL */
	{0x13, 0x18 },	/*	0x13	AK4957_13_DIGITAL_VOLUME2_CONTROL	*/
	{0x14, 0x00 },	/*	0x14	AK4957_14_RESERVED					*/
	{0x15, 0x00 },	/*	0x15	AK4957_15_ALC_INPUT_LEVEL			*/
	{0x16, 0x02 },	/*	0x16	AK4957_16_REF2_SETTING				*/
	{0x17, 0x00 },	/*	0x17	AK4957_17_LCH_MIC_GAIN_SETTING		*/
	{0x18, 0x00 },	/*	0x18	AK4957_18_RCH_MIC_GAIN_SETTING		*/
	{0x19, 0x00 },	/*	0x19	AK4957_19_BEEP_CONTROL				*/
	{0x1A, 0x00 },	/*	0x1A	AK4957_1A_VIDEO_CONTROL 			*/
	{0x1B, 0x01 },	/*	0x1B	AK4957_1B_HPF_FILTER_CONTROL		*/
	{0x1C, 0x00 },	/*	0x1C	AK4957_1C_DIGITAL_FILTER_SELECT1	*/
	{0x1D, 0x03 },	/*	0x1D	AK4957_1D_DIGITAL_FILTER_MODE		*/
	{0x1E, 0xB0 },	/*	0x1E	AK4957_1E_HPF2_COEFFICIENT0 		*/
	{0x1F, 0x1F },	/*	0x1F	AK4957_1F_HPF2_COEFFICIENT1 		*/
	{0x20, 0x9F },	/*	0x20	AK4957_20_HPF2_COEFFICIENT2 		*/
	{0x21, 0x20 },	/*	0x21	AK4957_21_HPF2_COEFFICIENT3 		*/
	{0x22, 0xB6 },	/*	0x22	AK4957_22_LPF_COEFFICIENT0			*/
	{0x23, 0x0B },	/*	0x23	AK4957_23_LPF_COEFFICIENT1			*/
	{0x24, 0x6D },	/*	0x24	AK4957_24_LPF_COEFFICIENT2			*/
	{0x25, 0x37 },	/*	0x25	AK4957_25_LPF_COEFFICIENT3			*/
	{0x26, 0x64 },	/*	0x26	AK4957_26_FIL3_COEFFICIENT0 		*/
	{0x27, 0x83 },	/*	0x27	AK4957_27_FIL3_COEFFICIENT1 		*/
	{0x28, 0x86 },	/*	0x28	AK4957_28_FIL3_COEFFICIENT2 		*/
	{0x29, 0x2D },	/*	0x29	AK4957_29_FIL3_COEFFICIENT3 		*/
	{0x2A, 0x26 },	/*	0x2A	AK4957_2A_EQ0_EFFICIENT0			*/
	{0x2B, 0x23 },	/*	0x2B	AK4957_2B_EQ0_EFFICIENT1			*/
	{0x2C, 0x72 },	/*	0x2C	AK4957_2C_EQ0_EFFICIENT2			*/
	{0x2D, 0x27 },	/*	0x2D	AK4957_2D_EQ0_EFFICIENT3			*/
	{0x2E, 0xB5 },	/*	0x2E	AK4957_2E_EQ0_EFFICIENT4			*/
	{0x2F, 0xEB },	/*	0x2F	AK4957_2F_EQ0_EFFICIENT5			*/
	{0x30, 0x00 },	/*	0x30	AK4957_30_DIGITAL_FILTER_SELECT2	*/
	{0x31, 0x00 },	/*	0x31	AK4957_31_EQ_COMMON_GAIN_SELECT */
	{0x32, 0x80 },	/*	0x32	AK4957_32_EQ_COMMON_GAIN_SETTING	*/
	{0x33, 0x28 },	/*	0x33	AK4957_33_E1_COEFFICIENT0			*/
	{0x34, 0x00 },	/*	0x34	AK4957_34_E1_COEFFICIENT1			*/
	{0x35, 0x5E },	/*	0x35	AK4957_35_E1_COEFFICIENT2			*/
	{0x36, 0x3F },	/*	0x36	AK4957_36_E1_COEFFICIENT3			*/
	{0x37, 0x9F },	/*	0x37	AK4957_37_E1_COEFFICIENT4			*/
	{0x38, 0xE0 },	/*	0x38	AK4957_38_E1_COEFFICIENT5			*/
	{0x39, 0x9E },	/*	0x39	AK4957_39_E2_COEFFICIENT0			*/
	{0x3A, 0x00 },	/*	0x3A	AK4957_3A_E2_COEFFICIENT1			*/
	{0x3B, 0xBF },	/*	0x3B	AK4957_3B_E2_COEFFICIENT2			*/
	{0x3C, 0x3E },	/*	0x3C	AK4957_3C_E2_COEFFICIENT3			*/
	{0x3D, 0x3C },	/*	0x3D	AK4957_3D_E2_COEFFICIENT4			*/
	{0x3E, 0xE1 },	/*	0x3E	AK4957_3E_E2_COEFFICIENT5			*/
	{0x3F, 0xB9 },	/*	0x3F	AK4957_3F_E3_COEFFICIENT0			*/
	{0x40, 0x03 },	/*	0x40	AK4957_40_E3_COEFFICIENT1			*/
	{0x41, 0x12 },	/*	0x41	AK4957_41_E3_COEFFICIENT2			*/
	{0x42, 0x38 },	/*	0x42	AK4957_42_E3_COEFFICIENT3			*/
	{0x43, 0x72 },	/*	0x43	AK4957_43_E3_COEFFICIENT4			*/
	{0x44, 0xE7 },	/*	0x44	AK4957_44_E3_COEFFICIENT5			*/
	{0x45, 0x83 },	/*	0x45	AK4957_45_E4_COEFFICIENT0			*/
	{0x46, 0x09 },	/*	0x46	AK4957_46_E4_COEFFICIENT1			*/
	{0x47, 0x01 },	/*	0x47	AK4957_47_E4_COEFFICIENT2			*/
	{0x48, 0x22 },	/*	0x48	AK4957_48_E4_COEFFICIENT3			*/
	{0x49, 0x23 },	/*	0x49	AK4957_49_E4_COEFFICIENT4			*/
	{0x4A, 0xF5 },	/*	0x4A	AK4957_4A_E4_COEFFICIENT5			*/
	{0x4B, 0x00 },	/*	0x4B	AK4957_4B_E5_COEFFICIENT0			*/
	{0x4C, 0x0C },	/*	0x4C	AK4957_4C_E5_COEFFICIENT1			*/
	{0x4D, 0xC1 },	/*	0x4D	AK4957_4D_E5_COEFFICIENT2			*/
	{0x4E, 0xF3 },	/*	0x4E	AK4957_4E_E5_COEFFICIENT3			*/
	{0x4F, 0x00 },	/*	0x4F	AK4957_4F_E5_COEFFICIENT4			*/
	{0x50, 0x00 },	/*	0x50	AK4957_50_E5_COEFFICIENT5			*/
};

#ifdef AK4957_CONTIF_DEBUG
static const struct {
	int readable;   /* Mask of readable bits */
	int writable;   /* Mask of writable bits */
} ak4957_access_masks[] = {
    { 0xFF, 0xFF },	//0x00
    { 0xFF, 0xDB },	//0x01
    { 0xFF, 0x97 },	//0x02
    { 0xFF, 0xF3 },	//0x03
    { 0xFF, 0xFB },	//0x04
    { 0xFF, 0xFB },	//0x05
    { 0xFF, 0xCF },	//0x06
    { 0xFF, 0xAC },	//0x07
    { 0xFF, 0x3B },	//0x08
    { 0xFF, 0xC1 },	//0x09
    { 0xFF, 0xFF },	//0x0A
    { 0xFF, 0xFF },	//0x0B
    { 0xFF, 0xFF },	//0x0C
    { 0xFF, 0xFF },	//0x0D
    { 0xFF, 0x00 },	//0x0E
    { 0xFF, 0xFF },	//0x0F
    { 0xFF, 0xFF },	//0x10
    { 0xFF, 0xFF },	//0x11
    { 0xFF, 0xFF },	//0x12
    { 0xFF, 0xFF },	//0x13
    { 0xFF, 0x00 },	//0x14
    { 0xFF, 0x00 },	//0x15
    { 0xFF, 0x77 },	//0x16
    { 0xFF, 0x1F },	//0x17
    { 0xFF, 0x1F },	//0x18
    { 0xFF, 0x5F },	//0x19
    { 0xFF, 0x0D },	//0x1A
    { 0xFF, 0x77 },	//0x1B
    { 0xFF, 0xBF },	//0x1C
    { 0xFF, 0x07 },	//0x1D
    { 0xFF, 0xFF },	//0x1E
    { 0xFF, 0xFF },	//0x1F
    { 0xFF, 0xFF },	//0x20
    { 0xFF, 0xFF },	//0x21
    { 0xFF, 0xFF },	//0x22
    { 0xFF, 0xFF },	//0x23
    { 0xFF, 0xFF },	//0x24
    { 0xFF, 0xFF },	//0x25
    { 0xFF, 0xFF },	//0x26
    { 0xFF, 0xFF },	//0x27
    { 0xFF, 0xFF },	//0x28
    { 0xFF, 0xFF },	//0x29
    { 0xFF, 0xFF },	//0x2A
    { 0xFF, 0xFF },	//0x2B
    { 0xFF, 0xFF },	//0x2C
    { 0xFF, 0xFF },	//0x2D
    { 0xFF, 0xFF },	//0x2E
    { 0xFF, 0xFF },	//0x2F
    { 0xFF, 0x1F },	//0x30
    { 0xFF, 0xFE },	//0x31
    { 0xFF, 0xFF },	//0x32
    { 0xFF, 0xFF },	//0x33
    { 0xFF, 0xFF },	//0x34
    { 0xFF, 0xFF },	//0x35
    { 0xFF, 0xFF },	//0x36
    { 0xFF, 0xFF },	//0x37
    { 0xFF, 0xFF },	//0x38
    { 0xFF, 0xFF },	//0x39
    { 0xFF, 0xFF },	//0x3A
    { 0xFF, 0xFF },	//0x3B
    { 0xFF, 0xFF },	//0x3C
    { 0xFF, 0xFF },	//0x3D
    { 0xFF, 0xFF },	//0x3E
    { 0xFF, 0xFF },	//0x3F
    { 0xFF, 0xFF },	//0x40
    { 0xFF, 0xFF },	//0x41
    { 0xFF, 0xFF },	//0x42
    { 0xFF, 0xFF },	//0x43
    { 0xFF, 0xFF },	//0x44
    { 0xFF, 0xFF },	//0x45
    { 0xFF, 0xFF },	//0x46
    { 0xFF, 0xFF },	//0x47
    { 0xFF, 0xFF },	//0x48
    { 0xFF, 0xFF },	//0x49
    { 0xFF, 0xFF },	//0x4A
    { 0xFF, 0xFF },	//0x4B
    { 0xFF, 0xFF },	//0x4C
    { 0xFF, 0xFF },	//0x4D
    { 0xFF, 0xFF },	//0x4E
    { 0xFF, 0xFF },	//0x4F
    { 0xFF, 0xFF },	//0x50
};
#endif

/*
 *  MIC Gain control:
 * from 0 to 21 dB (quantity of each step is various)
 */
static DECLARE_TLV_DB_MINMAX(mgain_tlv, 0, 2100);

/*
 * Input Digital volume control:
 * from -54.375 to 36 dB in 0.375 dB steps (mute instead of -54.375 dB)
 */
static DECLARE_TLV_DB_SCALE(ivol_tlv, -5437, 37, 0);

/*
 * Speaker output volume control:
 * from 8.1 to 14.1 dB (quantity of each step is various)
 */
static DECLARE_TLV_DB_MINMAX(spkout_tlv, 810, 1410);

/*
 * Output Digital volume control1:
 * from -54.375 to 36 dB in 0.375 dB steps (mute instead of -54.375 dB)
 */
static DECLARE_TLV_DB_SCALE(ovol_tlv, -5437, 37, 0);

/*
 * Output Digital volume control2:
 * from -90.0 to 6 dB in 0.5 dB steps (mute instead of -115.5 dB)
 */
static DECLARE_TLV_DB_SCALE(dvol_tlv, -11550, 50, 0);

/*
 * Beep volume control:
 * from -42 to 0 dB (quantity of each step is various)
 */
static DECLARE_TLV_DB_MINMAX(beep_tlv, -4200, 0);

/*
 * EQ Gain control: (EQ2, EQ3, EQ4, EQ5)
 * from xx(too small value) to 0 dB (quantity of each step is various)
 */
static DECLARE_TLV_DB_MINMAX(eq_tlv, -10000, 0);


static const char *stereo_on_select[]  =
{
	"Stereo Enphasis Filter OFF",
	"Stereo Enphasis Filter ON",
};

static const struct soc_enum ak4957_stereo_enum[] = {
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(stereo_on_select), stereo_on_select),
};

static const char *mic_sens_correct[]  =
{
	"-3","-2.625","-2.25","-1.875","-1.5","-1.125","-0.75","-0.375",
	"0","+0.375","+0.75","+1.125","+1.5","+1.875","+2.25","+2.625","+3",
};

static const struct soc_enum ak4957_mic_adjust_enum[] = {
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mic_sens_correct), mic_sens_correct),
};

static int ak4957_writeMask(struct snd_soc_codec *, u16, u16, u16);
static inline u32 ak4957_read_reg_cache(struct snd_soc_codec *, u16);

static int get_onstereo(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4957_priv *ak4957 = snd_soc_codec_get_drvdata(codec);

    ucontrol->value.enumerated.item[0] = ak4957->onStereo;

    return 0;
}

static int set_onstereo(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4957_priv *ak4957 = snd_soc_codec_get_drvdata(codec);

	ak4957->onStereo = ucontrol->value.enumerated.item[0];

	if ( ak4957->onStereo ) {
		ak4957_writeMask(codec, AK4957_1C_DIGITAL_FILTER_SELECT1, 0x09, 0x09);	//EQ0,FIL3
	}
	else {
		ak4957_writeMask(codec, AK4957_1C_DIGITAL_FILTER_SELECT1, 0x09, 0x00);
	}

    return 0;
}

static int get_micadjustl(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4957_priv *ak4957 = snd_soc_codec_get_drvdata(codec);

    ucontrol->value.enumerated.item[0] = ak4957->mGainL;

    return 0;
}

static int set_micadjustl(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4957_priv *ak4957 = snd_soc_codec_get_drvdata(codec);

	ak4957->mGainL = ucontrol->value.enumerated.item[0];

	if ( ak4957->mGainL < 8) {
//		ak4957->mGainL |= 0x18;		//set 11xxx
		ak4957_writeMask(codec, AK4957_17_LCH_MIC_GAIN_SETTING, 0x1F, (ak4957->mGainL | 0x18));
	}
	else {
//		ak4957->mGainL -= 8;		//set 0xxx
		ak4957_writeMask(codec, AK4957_17_LCH_MIC_GAIN_SETTING, 0x1F, (ak4957->mGainL - 8));
	}

    return 0;
}
static int get_micadjustr(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4957_priv *ak4957 = snd_soc_codec_get_drvdata(codec);

    ucontrol->value.enumerated.item[0] = ak4957->mGainR;

    return 0;
}

static int set_micadjustr(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4957_priv *ak4957 = snd_soc_codec_get_drvdata(codec);

	ak4957->mGainR = ucontrol->value.enumerated.item[0];

	if ( ak4957->mGainR < 8) {
//		ak4957->mGainR |= 0x18;		//set 11xxx
		ak4957_writeMask(codec, AK4957_18_RCH_MIC_GAIN_SETTING, 0x1F, (ak4957->mGainR | 0x18));
	}
	else {
//		ak4957->mGainR -= 8;		//set 0xxxx
		ak4957_writeMask(codec, AK4957_18_RCH_MIC_GAIN_SETTING, 0x1F, (ak4957->mGainR - 8));
	}

    return 0;
}

#ifdef AK4957_DEBUG

static const char *test_reg_select[]   =
{
    "read AK4957 Reg 00:2F",
    "read AK4957 Reg 30:50",
};

static const struct soc_enum ak4957_enum[] =
{
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(test_reg_select), test_reg_select),
};

static int nTestRegNo = 0;

static int get_test_reg(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    /* Get the current output routing */
    ucontrol->value.enumerated.item[0] = nTestRegNo;

    return 0;

}

static int set_test_reg(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
    u32    currMode = ucontrol->value.enumerated.item[0];
	int    i, value;
	int	   regs, rege;

	nTestRegNo = currMode;

	switch(nTestRegNo) {
		case 1:
			regs = 0x30;
			rege = 0x50;
			break;
		default:
			regs = 0x00;
			rege = 0x2F;
			break;
	}

	for ( i = regs ; i <= rege ; i++ ){
		value = snd_soc_read(codec, i);
		printk("***AK4957 Addr,Reg=(%x, %x)\n", i, value);
	}

	return(0);

}

#endif

static const struct snd_kcontrol_new ak4957_snd_controls[] = {
	SOC_SINGLE_TLV("Mic Gain Control",
			AK4957_02_SIGNAL_SELECT1, 0, 0x05, 0, mgain_tlv),
	SOC_SINGLE_TLV("Input Digital Volume",
			AK4957_0F_LCH_INPUT_VOLUME_CONTROL, 0, 0xF1, 0, ivol_tlv),
	SOC_SINGLE_TLV("Speaker Output Volume",
			AK4957_03_SIGNAL_SELECT2, 6, 0x03, 0, spkout_tlv),
	SOC_SINGLE_TLV("Digital Output Volume1",
			AK4957_11_LCH_OUTPUT_VOLUME_CONTROL, 0, 0xF1, 0, ovol_tlv),
	SOC_SINGLE_TLV("Digital Output Volume2",
			AK4957_13_DIGITAL_VOLUME2_CONTROL, 0, 0xFF, 1, dvol_tlv),
	SOC_ENUM_EXT("Mic Gain Adjust Control L", ak4957_mic_adjust_enum[0], get_micadjustl, set_micadjustl),
	SOC_ENUM_EXT("Mic Gain Adjust Control R", ak4957_mic_adjust_enum[0], get_micadjustr, set_micadjustr),
	SOC_SINGLE_TLV("Beep Volume Control",
			AK4957_19_BEEP_CONTROL, 0, 0x09, 1, beep_tlv),
	SOC_SINGLE("Video Amp Gain Control", AK4957_1A_VIDEO_CONTROL, 2, 0x03, 0),
	SOC_SINGLE_TLV("EQ Gain Control",
			AK4957_32_EQ_COMMON_GAIN_SETTING, 0, 0xFF, 0, eq_tlv),

    SOC_SINGLE("High Path Filter 1 FC", AK4957_1B_HPF_FILTER_CONTROL, 1, 3, 0),
    SOC_SINGLE("High Path Filter 3 FC", AK4957_1B_HPF_FILTER_CONTROL, 4, 5, 0),
    SOC_SINGLE("Stereo Emphasis Gain", AK4957_1C_DIGITAL_FILTER_SELECT1, 1, 2, 0),

	SOC_SINGLE("High Path Filter 2", AK4957_1C_DIGITAL_FILTER_SELECT1, 4, 1, 0),
    SOC_SINGLE("High Path Filter 3", AK4957_1C_DIGITAL_FILTER_SELECT1, 7, 1, 0),
    SOC_SINGLE("Low Path Filter", AK4957_1C_DIGITAL_FILTER_SELECT1, 5, 1, 0),
    SOC_SINGLE("5 Band Equalizer 1", AK4957_30_DIGITAL_FILTER_SELECT2, 0, 1, 0),
	SOC_SINGLE("5 Band Equalizer 2", AK4957_30_DIGITAL_FILTER_SELECT2, 1, 1, 0),
	SOC_SINGLE("5 Band Equalizer 3", AK4957_30_DIGITAL_FILTER_SELECT2, 2, 1, 0),
	SOC_SINGLE("5 Band Equalizer 4", AK4957_30_DIGITAL_FILTER_SELECT2, 3, 1, 0),
	SOC_SINGLE("5 Band Equalizer 5", AK4957_30_DIGITAL_FILTER_SELECT2, 4, 1, 0),
	SOC_SINGLE("5 Band Equalizer 2 Gain Enable", AK4957_31_EQ_COMMON_GAIN_SELECT, 1, 1, 0),
	SOC_SINGLE("5 Band Equalizer 3 Gain Enable", AK4957_31_EQ_COMMON_GAIN_SELECT, 2, 1, 0),
	SOC_SINGLE("5 Band Equalizer 4 Gain Enable", AK4957_31_EQ_COMMON_GAIN_SELECT, 3, 1, 0),
	SOC_SINGLE("5 Band Equalizer 5 Gain Enable", AK4957_31_EQ_COMMON_GAIN_SELECT, 4, 1, 0),
	SOC_SINGLE("ALC for Recording", AK4957_0B_ALC_MODE_CONTROL1, 5, 1, 0),
	SOC_SINGLE("ALC for Playback", AK4957_0B_ALC_MODE_CONTROL1, 6, 1, 0),
	SOC_ENUM_EXT("Stereo Enphasis Filter Control", ak4957_stereo_enum[0], get_onstereo, set_onstereo),
	SOC_SINGLE("Composite Video Control", AK4957_1A_VIDEO_CONTROL, 0, 1, 0),
	SOC_SINGLE("Mono/Stereo Control", AK4957_04_SIGNAL_SELECT3, 6, 3, 0),
	SOC_SINGLE("Soft Mute Control", AK4957_07_MODE_CONTROL3, 5, 1, 0),

#ifdef AK4957_DEBUG
	SOC_ENUM_EXT("Reg Read", ak4957_enum[0], get_test_reg, set_test_reg),
#endif

};

static const char *ak4957_lin_select_texts[] =
		{"LIN1", "LIN2"};

static const struct soc_enum ak4957_lin_mux_enum =
	SOC_ENUM_SINGLE(AK4957_03_SIGNAL_SELECT2, 0,
			ARRAY_SIZE(ak4957_lin_select_texts), ak4957_lin_select_texts);

static const struct snd_kcontrol_new ak4957_lin_mux_control =
	SOC_DAPM_ENUM("LIN Select", ak4957_lin_mux_enum);

static const char *ak4957_rin_select_texts[] =
		{"RIN1", "RIN2"};

static const struct soc_enum ak4957_rin_mux_enum =
	SOC_ENUM_SINGLE(AK4957_03_SIGNAL_SELECT2, 1,
			ARRAY_SIZE(ak4957_rin_select_texts), ak4957_rin_select_texts);

static const struct snd_kcontrol_new ak4957_rin_mux_control =
	SOC_DAPM_ENUM("RIN Select", ak4957_rin_mux_enum);

static const char *ak4957_micbias_select_texts[] =
		{"LineIN", "MicIN"};

static const struct soc_enum ak4957_micbias_mux_enum =
	SOC_ENUM_SINGLE(0, 0,
			ARRAY_SIZE(ak4957_micbias_select_texts), ak4957_micbias_select_texts);

static const struct snd_kcontrol_new ak4957_micbias_mux_control =
	SOC_DAPM_ENUM("Mic bias ON/OFF", ak4957_micbias_mux_enum);

static const char *ak4957_adcpf_select_texts[] =
		{"SDTI", "ADC"};

static const struct soc_enum ak4957_adcpf_mux_enum =
	SOC_ENUM_SINGLE(AK4957_1D_DIGITAL_FILTER_MODE, 1,
			ARRAY_SIZE(ak4957_adcpf_select_texts), ak4957_adcpf_select_texts);

static const struct snd_kcontrol_new ak4957_adcpf_mux_control =
	SOC_DAPM_ENUM("ADCPF Select", ak4957_adcpf_mux_enum);


static const char *ak4957_pfsdo_select_texts[] =
		{"ADC", "PFIL"};

static const struct soc_enum ak4957_pfsdo_mux_enum =
	SOC_ENUM_SINGLE(AK4957_1D_DIGITAL_FILTER_MODE, 0,
			ARRAY_SIZE(ak4957_pfsdo_select_texts), ak4957_pfsdo_select_texts);

static const struct snd_kcontrol_new ak4957_pfsdo_mux_control =
	SOC_DAPM_ENUM("PFSDO Select", ak4957_pfsdo_mux_enum);

static const char *ak4957_pfdac_select_texts[] =
		{"SDTI", "PFIL"};

static const struct soc_enum ak4957_pfdac_mux_enum =
	SOC_ENUM_SINGLE(AK4957_1D_DIGITAL_FILTER_MODE, 2,
			ARRAY_SIZE(ak4957_pfdac_select_texts), ak4957_pfdac_select_texts);

static const struct snd_kcontrol_new ak4957_pfdac_mux_control =
	SOC_DAPM_ENUM("PFDAC Select", ak4957_pfdac_mux_enum);

static const char *ak4957_mic_select_texts[] =
		{"AMIC", "DMIC"};

static const struct soc_enum ak4957_mic_mux_enum =
	SOC_ENUM_SINGLE(AK4957_08_DIGITL_MIC, 0,
			ARRAY_SIZE(ak4957_mic_select_texts), ak4957_mic_select_texts);

static const struct snd_kcontrol_new ak4957_mic_mux_control =
	SOC_DAPM_ENUM("MIC Select", ak4957_mic_mux_enum);

static const struct snd_kcontrol_new ak4957_dacs_mixer_controls[] = {
	SOC_DAPM_SINGLE("DACS", AK4957_03_SIGNAL_SELECT2, 4, 1, 0),
	SOC_DAPM_SINGLE("BEEPS", AK4957_03_SIGNAL_SELECT2, 5, 1, 0),
};

static int ak4957_spk_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event) //CONFIG_LINF
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	akdbgprt("\t[AK4957] %s(%d)\n",__FUNCTION__,__LINE__);

	switch (event) {
		case SND_SOC_DAPM_PRE_PMU:	/* before widget power up */
			ak4957_writeMask(codec, AK4957_02_SIGNAL_SELECT1, 0x80,0x00);	//Power Save Mode
			break;
		case SND_SOC_DAPM_POST_PMU:	/* after widget power up */
			akdbgprt("\t[AK4957] %s wait=1msec\n",__FUNCTION__);
			mdelay(1);
			ak4957_writeMask(codec, AK4957_02_SIGNAL_SELECT1, 0x80,0x80);	//Normal Operation
			break;
		case SND_SOC_DAPM_PRE_PMD:	/* before widget power down */
			ak4957_writeMask(codec, AK4957_02_SIGNAL_SELECT1, 0x80,0x00);	//Power Save Mode
			mdelay(1);
			break;
		case SND_SOC_DAPM_POST_PMD:	/* after widget power down */
			break;
	}

	return 0;
}

static const struct snd_kcontrol_new ak4957_dacl_mixer_controls[] = {
	SOC_DAPM_SINGLE("DACL", AK4957_04_SIGNAL_SELECT3, 4, 1, 0),
	SOC_DAPM_SINGLE("BEEPL", AK4957_04_SIGNAL_SELECT3, 3, 1, 0),
};

static int ak4957_line_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event) //CONFIG_LINF
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	akdbgprt("\t[AK4957] %s(%d)\n",__FUNCTION__,__LINE__);

	switch (event) {
		case SND_SOC_DAPM_PRE_PMU:	/* before widget power up */
			ak4957_writeMask(codec, AK4957_04_SIGNAL_SELECT3, 0x20,0x20);	//Power Save Mode
			break;
		case SND_SOC_DAPM_POST_PMU:	/* after widget power up */
			akdbgprt("\t[AK4957] %s wait=300msec\n",__FUNCTION__);
			mdelay(300);
			ak4957_writeMask(codec, AK4957_04_SIGNAL_SELECT3, 0x20,0x00);	//Normal Operation
			break;
		case SND_SOC_DAPM_PRE_PMD:	/* before widget power down */
			ak4957_writeMask(codec, AK4957_04_SIGNAL_SELECT3, 0x20,0x20);	//Power Save Mode
			mdelay(1);
			break;
		case SND_SOC_DAPM_POST_PMD:	/* after widget power down */
			akdbgprt("\t[AK4957] %s wait=300msec\n",__FUNCTION__);
			mdelay(300);
			ak4957_writeMask(codec, AK4957_04_SIGNAL_SELECT3, 0x20,0x00);	//Normal Operation
			break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget ak4957_dapm_widgets[] = {

// ADC, DAC
	SND_SOC_DAPM_ADC("ADC Left", "NULL", AK4957_00_POWER_MANAGEMENT1, 0, 0),
	SND_SOC_DAPM_ADC("ADC Right", "NULL", AK4957_00_POWER_MANAGEMENT1, 1, 0),
	SND_SOC_DAPM_DAC("DAC", "NULL", AK4957_00_POWER_MANAGEMENT1, 2, 0),

#ifdef PLL_32BICK_MODE
	SND_SOC_DAPM_SUPPLY("PMPLL", AK4957_01_POWER_MANAGEMENT2, 0, 0, NULL, 0),
#else
#ifdef PLL_64BICK_MODE
	SND_SOC_DAPM_SUPPLY("PMPLL", AK4957_01_POWER_MANAGEMENT2, 0, 0, NULL, 0),
#endif
#endif

	SND_SOC_DAPM_ADC("PFIL", "NULL", AK4957_00_POWER_MANAGEMENT1, 7, 0),

	SND_SOC_DAPM_ADC("DMICL", "NULL", AK4957_08_DIGITL_MIC, 4, 0),
	SND_SOC_DAPM_ADC("DMICR", "NULL", AK4957_08_DIGITL_MIC, 5, 0),

	SND_SOC_DAPM_AIF_OUT("SDTO", "Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("SDTI", "Playback", 0, SND_SOC_NOPM, 0, 0),

// Analog Output
	SND_SOC_DAPM_OUTPUT("SPK"),
	SND_SOC_DAPM_OUTPUT("Line"),

	SND_SOC_DAPM_PGA("SPK Amp", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Line Amp", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("BEEP Amp", AK4957_00_POWER_MANAGEMENT1, 5, 0, NULL, 0),
	SND_SOC_DAPM_MIXER_E("SPK Mixer", AK4957_00_POWER_MANAGEMENT1, 4, 0,
			&ak4957_dacs_mixer_controls[0], ARRAY_SIZE(ak4957_dacs_mixer_controls),
			ak4957_spk_event, (SND_SOC_DAPM_POST_PMU |SND_SOC_DAPM_PRE_PMD
                            |SND_SOC_DAPM_PRE_PMU |SND_SOC_DAPM_POST_PMD)),
	SND_SOC_DAPM_MIXER_E("Line Mixer", AK4957_00_POWER_MANAGEMENT1, 3, 0,
			&ak4957_dacl_mixer_controls[0], ARRAY_SIZE(ak4957_dacl_mixer_controls),
			ak4957_line_event, (SND_SOC_DAPM_POST_PMU |SND_SOC_DAPM_PRE_PMD
                            |SND_SOC_DAPM_PRE_PMU |SND_SOC_DAPM_POST_PMD)),

// Analog Input
	SND_SOC_DAPM_INPUT("LIN1"),
	SND_SOC_DAPM_INPUT("RIN1"),
	SND_SOC_DAPM_INPUT("LIN2"),
	SND_SOC_DAPM_INPUT("RIN2"),
	SND_SOC_DAPM_INPUT("BEEPIN"),

	SND_SOC_DAPM_MUX("RIN MUX", AK4957_01_POWER_MANAGEMENT2, 7, 0,	&ak4957_rin_mux_control),
	SND_SOC_DAPM_MUX("LIN MUX", AK4957_01_POWER_MANAGEMENT2, 6, 0,	&ak4957_lin_mux_control),

// MIC Bias
	SND_SOC_DAPM_MICBIAS("Mic Bias", AK4957_01_POWER_MANAGEMENT2, 4, 0),

	SND_SOC_DAPM_MUX("Mic Bias MUX", SND_SOC_NOPM, 0, 0, &ak4957_micbias_mux_control),

// PFIL
	SND_SOC_DAPM_MUX("PFIL MUX", SND_SOC_NOPM, 0, 0, &ak4957_adcpf_mux_control),
	SND_SOC_DAPM_MUX("PFSDO MUX", SND_SOC_NOPM, 0, 0, &ak4957_pfsdo_mux_control),
	SND_SOC_DAPM_MUX("PFDAC MUX", SND_SOC_NOPM, 0, 0, &ak4957_pfdac_mux_control),

// Digital Mic
	SND_SOC_DAPM_INPUT("DMICLIN"),
	SND_SOC_DAPM_INPUT("DMICRIN"),

	SND_SOC_DAPM_MUX("MIC MUX", SND_SOC_NOPM, 0, 0, &ak4957_mic_mux_control),

};

static const struct snd_soc_dapm_route ak4957_intercon[] = {

#ifdef PLL_32BICK_MODE
	{"ADC Left", NULL, "PMPLL"},
	{"ADC Right", NULL, "PMPLL"},
	{"DAC", NULL, "PMPLL"},
#else
#ifdef PLL_64BICK_MODE
	{"ADC Left", NULL, "PMPLL"},
	{"ADC Right", NULL, "PMPLL"},
	{"DAC", NULL, "PMPLL"},
#endif
#endif

	{"LIN MUX", "LIN1", "LIN1"},
	{"LIN MUX", "LIN2", "LIN2"},
	{"RIN MUX", "RIN1", "RIN1"},
	{"RIN MUX", "RIN2", "RIN2"},

	{"Mic Bias", NULL, "LIN MUX"},
	{"Mic Bias", NULL, "RIN MUX"},

	{"Mic Bias MUX", "LineIN", "LIN MUX"},
	{"Mic Bias MUX", "LineIN", "RIN MUX"},
	{"Mic Bias MUX", "MicIN", "Mic Bias"},

	{"ADC Left", NULL, "Mic Bias MUX"},
	{"ADC Right", NULL, "Mic Bias MUX"},

	{"DMICL", NULL, "DMICLIN"},
	{"DMICR", NULL, "DMICRIN"},

	{"MIC MUX", "AMIC", "ADC Left"},
	{"MIC MUX", "AMIC", "ADC Right"},
	{"MIC MUX", "DMIC", "DMICL"},
	{"MIC MUX", "DMIC", "DMICR"},

	{"PFIL MUX", "SDTI", "SDTI"},
	{"PFIL MUX", "ADC", "MIC MUX"},
	{"PFIL", NULL, "PFIL MUX"},

	{"PFSDO MUX", "ADC", "MIC MUX"},
	{"PFSDO MUX", "PFIL", "PFIL"},
	{"SDTO", NULL, "PFSDO MUX"},

	{"PFDAC MUX", "SDTI", "SDTI"},
	{"PFDAC MUX", "PFIL", "PFIL"},

	{"DAC", NULL, "PFDAC MUX"},

	{"BEEP Amp", NULL, "BEEPIN"},
	{"SPK Mixer", "BEEPS", "BEEP Amp"},
	{"SPK Mixer", "DACS", "DAC"},
	{"SPK Amp", NULL, "SPK Mixer"},
	{"SPK", NULL, "SPK Amp"},

	{"Line Mixer", "BEEPL", "BEEP Amp"},
	{"Line Mixer", "DACL", "DAC"},
	{"Line Amp", NULL, "Line Mixer"},
	{"Line", NULL, "Line Amp"},

};


static int ak4957_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 	fs;

	akdbgprt("\t[AK4957] %s(%d)\n",__FUNCTION__,__LINE__);

	fs = snd_soc_read(codec, AK4957_06_MODE_CONTROL2);
	fs &= ~AK4957_FS;

	ak4957_data->fs2 = params_rate(params);

	switch (ak4957_data->fs2) {
	case 8000:
		fs |= AK4957_FS_8KHZ;
		break;
	case 11025:
		fs |= AK4957_FS_11_025KHZ;
		break;
	case 12000:
		fs |= AK4957_FS_12KHZ;
		break;
	case 16000:
		fs |= AK4957_FS_16KHZ;
		break;
	case 22050:
		fs |= AK4957_FS_22_05KHZ;
		break;
	case 24000:
		fs |= AK4957_FS_24KHZ;
		break;
	case 32000:
		fs |= AK4957_FS_32KHZ;
		break;
	case 44100:
		fs |= AK4957_FS_44_1KHZ;
		break;
	case 48000:
		fs |= AK4957_FS_48KHZ;
		break;
	default:
		return -EINVAL;
	}
	snd_soc_write(codec, AK4957_06_MODE_CONTROL2, fs);

	return 0;
}

static int ak4957_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
		unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 pll;

	akdbgprt("\t[AK4957] %s(%d)\n",__FUNCTION__,__LINE__);

	pll = snd_soc_read(codec, AK4957_05_MODE_CONTROL1);
	pll &= ~AK4957_PLL;

#ifdef PLL_32BICK_MODE
	pll |= AK4957_PLL_BICK32;
#else
#ifdef PLL_64BICK_MODE
	pll |= AK4957_PLL_BICK64;
#else
	pll |= AK4957_EXT_SLAVE;
#endif
#endif

	snd_soc_write(codec, AK4957_05_MODE_CONTROL1, pll);

	return 0;
}

static int ak4957_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{

	struct snd_soc_codec *codec = dai->codec;
	u8 mode;
	u8 format;

	akdbgprt("\t[AK4957] %s(%d)\n",__FUNCTION__,__LINE__);

	/* set master/slave audio interface */
	mode = snd_soc_read(codec, AK4957_01_POWER_MANAGEMENT2);
	format = snd_soc_read(codec, AK4957_05_MODE_CONTROL1);
	format &= ~AK4957_DIF;

    switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
        case SND_SOC_DAIFMT_CBS_CFS:
			akdbgprt("\t[AK4957] %s(Slave)\n",__FUNCTION__);
            mode &= ~(AK4957_M_S);
//            format &= ~(AK4957_BCKO);
            break;
        case SND_SOC_DAIFMT_CBM_CFM:
			akdbgprt("\t[AK4957] %s(Master)\n",__FUNCTION__);
            mode |= (AK4957_M_S);
            format |= (AK4957_BCKO);
            break;
        case SND_SOC_DAIFMT_CBS_CFM:
        case SND_SOC_DAIFMT_CBM_CFS:
        default:
            dev_err(codec->dev, "Clock mode unsupported");
           return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:
			format |= AK4957_DIF_I2S_MODE;
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			format |= AK4957_DIF_24MSB_MODE;
			break;
		default:
			return -EINVAL;
	}

	/* set mode and format */

	snd_soc_write(codec, AK4957_01_POWER_MANAGEMENT2, mode);
	snd_soc_write(codec, AK4957_05_MODE_CONTROL1, format);

	return 0;
}

static bool ak4957_volatile(struct device *dev,
							unsigned int reg)
{
	int	ret;

	switch (reg) {
		//case :
		//	ret = 1;
		default:
			ret = 1;
			break;
	}
	return(ret);
}

#ifdef AK4957_CONTIF_DEBUG
static int ak4957_readable(struct device *dev,
							unsigned int reg)
{

	if (reg >= ARRAY_SIZE(ak4957_access_masks))
		return 0;
	return ak4957_access_masks[reg].readable != 0;
}
#endif

/*
* Read ak4957 register cache
 */
static inline u32 ak4957_read_reg_cache(struct snd_soc_codec *codec, u16 reg)
{
    u8 *cache = codec->reg_cache;
    BUG_ON(reg > ARRAY_SIZE(ak4957_reg));
    return (u32)cache[reg];
}

#ifdef AK4957_CONTIF_DEBUG
/*
 * Write ak4957 register cache
 */
static inline void ak4957_write_reg_cache(
struct snd_soc_codec *codec,
u16 reg,
u16 value)
{
    u8 *cache = codec->reg_cache;
    BUG_ON(reg > ARRAY_SIZE(ak4957_reg));
    cache[reg] = (u8)value;
}

unsigned int ak4957_i2c_read(struct snd_soc_codec *codec, unsigned int reg)
{

	int ret;

	if ( reg == AK4957_14_RESERVED ) { // Dummy Register.
		ret = ak4957_read_reg_cache(codec, reg);
		return ret;
	}

	ret = i2c_smbus_read_byte_data(codec->control_data, (u8)(reg & 0xFF));
//	ret = ak4957_read_reg_cache(codec, reg);

//	if ( reg < 3 )
//		akdbgprt("\t[ak4957] %s: (addr,data)=(%x, %x)\n",__FUNCTION__, reg, ret);

	if (ret < 0)
		akdbgprt("\t[AK4957] %s(%d)\n",__FUNCTION__,__LINE__);

	return ret;

}

static int ak4957_i2c_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	ak4957_write_reg_cache(codec, reg, value);

	akdbgprt("\t[ak4957] %s: (addr,data)=(%x, %x)\n",__FUNCTION__, reg, value);

	if ( reg == AK4957_14_RESERVED ) return 0;  // Dummy Register.

	if(i2c_smbus_write_byte_data(codec->control_data, (u8)(reg & 0xFF), (u8)(value & 0xFF))<0) {
		akdbgprt("\t[AK4957] %s(%d)\n",__FUNCTION__,__LINE__);
		return EIO;
	}

	return 0;
}
#endif

/*
 * Write with Mask to  AK4957 register space
 */
static int ak4957_writeMask(
struct snd_soc_codec *codec,
u16 reg,
u16 mask,
u16 value)
{
    u16 olddata;
    u16 newdata;

	if ( (mask == 0) || (mask == 0xFF) ) {
		newdata = value;
	}
	else {
		olddata = snd_soc_read(codec, reg);
	    newdata = (olddata & ~(mask)) | value;
	}

	snd_soc_write(codec, (unsigned int)reg, (unsigned int)newdata);

	akdbgprt("\t[ak4957_writeMask] %s(%d): (addr,data)=(%x, %x)\n",__FUNCTION__,__LINE__, reg, newdata);

    return(0);
}

// * for AK4957
static int ak4957_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *codec_dai)
{
	int 	ret = 0;
 //   struct snd_soc_codec *codec = codec_dai->codec;

	akdbgprt("\t[AK4957] %s(%d)\n",__FUNCTION__,__LINE__);

	return ret;
}


static int ak4957_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	u8 reg;

	akdbgprt("\t[AK4957] %s(%d)\n",__FUNCTION__,__LINE__);

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		reg = snd_soc_read(codec, AK4957_00_POWER_MANAGEMENT1);	// * for AK4957
		snd_soc_write(codec, AK4957_00_POWER_MANAGEMENT1,			// * for AK4957
				reg | AK4957_PMVCM);
		break;
	case SND_SOC_BIAS_OFF:
		snd_soc_write(codec, AK4957_00_POWER_MANAGEMENT1, 0x00);	// * for AK4957
		break;
	}

	return 0;
}

static int ak4957_set_dai_mute(struct snd_soc_dai *dai, int mute) {
    struct snd_soc_codec *codec = dai->codec;
	int ret = 0;
	int ndt;

	akdbgprt("\t[AK4957] %s mute[%s]\n",__FUNCTION__, mute ? "ON":"OFF");

	if (mute) {
		//SMUTE: 1 , MUTE
		ret = snd_soc_update_bits(codec, AK4957_07_MODE_CONTROL3, 0x20, 0x20);
		ndt = 1024000 / ak4957_data->fs2;
		mdelay(ndt);
	}
	else
		// SMUTE:  0  ,NORMAL operation
		ret =snd_soc_update_bits(codec, AK4957_07_MODE_CONTROL3, 0x20, 0x00);

	return ret;

}

#define AK4957_RATES		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
				SNDRV_PCM_RATE_48000)

#define AK4957_FORMATS		SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE


static struct snd_soc_dai_ops ak4957_dai_ops = {
	.hw_params	= ak4957_hw_params,
	.set_sysclk	= ak4957_set_dai_sysclk,
	.set_fmt	= ak4957_set_dai_fmt,
	.trigger = ak4957_trigger,
	.digital_mute = ak4957_set_dai_mute,
};

struct snd_soc_dai_driver ak4957_dai[] = {
	{
		.name = "ak4957-AIF1",
		.playback = {
		       .stream_name = "Playback",
		       .channels_min = 1,
		       .channels_max = 2,
		       .rates = AK4957_RATES,
		       .formats = AK4957_FORMATS,
		},
		.capture = {
		       .stream_name = "Capture",
		       .channels_min = 1,
		       .channels_max = 2,
		       .rates = AK4957_RATES,
		       .formats = AK4957_FORMATS,
		},
		.ops = &ak4957_dai_ops,
	},
};


static int ak4957_init_reg(struct snd_soc_codec *codec)
{
	struct ak4957_priv *ak4957 = ak4957_data;
	int ret = 0;

	if (ak4957->rst_pin > 0 ) {
		akdbgprt("\t[AK4957] %s(%d) reset gpio %d \n",__FUNCTION__,__LINE__,ak4957->rst_pin);
		ret = devm_gpio_request(codec->dev, ak4957->rst_pin, "ak4957 reset");
		if (ret < 0){
			dev_err(codec->dev, "Failed to request rst_pin: %d\n", ret);
			return ret;
		}
	}
	snd_soc_codec_set_drvdata(codec, ak4957);
	/* Reset AK4957 codec */
	if ( ak4957->rst_pin > 0) {
		gpio_direction_output(ak4957->rst_pin, ak4957->rst_active);
		msleep(10);
		gpio_direction_output(ak4957->rst_pin, !ak4957->rst_active);
	}

	/*The 0x00 register no Ack for the dummy command:write 0x00 to 0x00*/
	ak4957->i2c_clt->flags |= I2C_M_IGNORE_NAK;
	i2c_smbus_write_byte_data(ak4957->i2c_clt, (u8)(AK4957_00_POWER_MANAGEMENT1 & 0xFF), 0x00);
	ak4957->i2c_clt->flags &= ~I2C_M_IGNORE_NAK;

	ak4957_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	akdbgprt("\t[AK4957 bias] %s(%d)\n",__FUNCTION__,__LINE__);

	snd_soc_write(codec, AK4957_07_MODE_CONTROL3, 0x8C);

	snd_soc_write(codec, AK4957_0A_ALC_TIMER_SELECT, AK4957_0B_ALC_MODE_CONTROL1);
	snd_soc_write(codec, AK4957_0F_LCH_INPUT_VOLUME_CONTROL, AK4957_0F_LCH_INPUT_VOLUME_CONTROL);
	snd_soc_write(codec, AK4957_1E_HPF2_COEFFICIENT0, AK4957_2F_EQ0_EFFICIENT5);
	snd_soc_write(codec, AK4957_32_EQ_COMMON_GAIN_SETTING, AK4957_50_E5_COEFFICIENT5);

	akdbgprt("\t[AK4957 Effect] %s(%d)\n",__FUNCTION__,__LINE__);

	ak4957->onStereo = 0;
	ak4957->mGainL = 8;
	ak4957->mGainR = 8;
	ak4957->fs2 = 48000;

	return(0);

}

static int ak4957_probe(struct snd_soc_codec *codec)
{
	struct ak4957_priv *ak4957 = ak4957_data;
	int ret = 0;

	akdbgprt("\t[AK4957] %s(%d)\n",__FUNCTION__,__LINE__);

#ifdef AK4957_CONTIF_DEBUG
	codec->write = ak4957_i2c_write;
	codec->read = ak4957_i2c_read;
#endif
	ak4957->codec = codec;

	ak4957_init_reg(codec);

    return ret;

}

static int ak4957_remove(struct snd_soc_codec *codec)
{

	akdbgprt("\t[AK4957] %s(%d)\n",__FUNCTION__,__LINE__);

	ak4957_set_bias_level(codec, SND_SOC_BIAS_OFF);


	return 0;
}

static int ak4957_suspend(struct snd_soc_codec *codec)
{
	struct ak4957_priv *ak4957 = snd_soc_codec_get_drvdata(codec);
	int i;
	for(i = 0; i < 7; i++)
		ak4957->reg_cache[i] = snd_soc_read(codec, i);

	ak4957_set_bias_level(codec, SND_SOC_BIAS_OFF);

	if(ak4957->pwr_pin > 0 ) {
		gpio_direction_output(ak4957->pwr_pin, !ak4957->pwr_active);
	}

	return 0;
}

static int ak4957_resume(struct snd_soc_codec *codec)
{
	struct ak4957_priv *ak4957 = snd_soc_codec_get_drvdata(codec);
	int i;

	if(ak4957->pwr_pin > 0 ) {
		gpio_direction_output(ak4957->pwr_pin, ak4957->pwr_active);
		msleep(5);
	}

	/* Reset AK4957 codec */
	if (ak4957->rst_pin > 0) {
		gpio_direction_output(ak4957->rst_pin, ak4957->rst_active);
		msleep(1);
		gpio_direction_output(ak4957->rst_pin, !ak4957->rst_active);
	}
	/*The 0x00 register no Ack for the dummy command:write 0x00 to 0x00*/
	ak4957->i2c_clt->flags |= I2C_M_IGNORE_NAK;
	i2c_smbus_write_byte_data(ak4957->i2c_clt, (u8)(AK4957_00_POWER_MANAGEMENT1 & 0xFF), 0x00);
	ak4957->i2c_clt->flags &= ~I2C_M_IGNORE_NAK;

	ak4957_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	for(i = 0; i < 7; i++)
		snd_soc_write(codec, i, ak4957->reg_cache[i]);

	return 0;

}


struct snd_soc_codec_driver soc_codec_dev_ak4957 = {
	.probe = ak4957_probe,
	.remove = ak4957_remove,
	.suspend =	ak4957_suspend,
	.resume =	ak4957_resume,
	.set_bias_level = ak4957_set_bias_level,
	.component_driver = {
		.controls			= ak4957_snd_controls,
		.num_controls		= ARRAY_SIZE(ak4957_snd_controls),
		.dapm_widgets		= ak4957_dapm_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(ak4957_dapm_widgets),
		.dapm_routes		= ak4957_intercon,
		.num_dapm_routes	= ARRAY_SIZE(ak4957_intercon),
	},
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ak4957);

static struct regmap_config ak4957_regmap = {
	.reg_bits		= 8,
	.val_bits		= 8,
	.max_register		= AK4957_MAX_REGISTERS,
	.reg_defaults		= ak4957_reg,
	.num_reg_defaults	= ARRAY_SIZE(ak4957_reg),
#ifdef AK4957_CONTIF_DEBUG
	.readable_reg		= ak4957_readable,
#endif
	.volatile_reg		= ak4957_volatile,
	.cache_type		= REGCACHE_RBTREE,
};


static int ak4957_i2c_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
	struct device_node *np = i2c->dev.of_node;
	struct ak4957_priv *ak4957;
	struct regmap *regmap;
	enum of_gpio_flags flags;
	int rst_pin;
	int ret=0;

	akdbgprt("\t[AK4957] %s(%d)\n",__FUNCTION__,__LINE__);

	ak4957 = devm_kzalloc(&i2c->dev, sizeof(struct ak4957_priv), GFP_KERNEL);
	if (ak4957 == NULL)
		return -ENOMEM;

	rst_pin = of_get_gpio_flags(np, 0, &flags);
	if (rst_pin < 0 || !gpio_is_valid(rst_pin))
		return -ENXIO;

	ak4957->i2c_clt = i2c;
	ak4957->rst_pin = rst_pin;
	ak4957->rst_active = !!(flags & OF_GPIO_ACTIVE_LOW);
	i2c_set_clientdata(i2c, ak4957);
	regmap = devm_regmap_init_i2c(i2c, &ak4957_regmap);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(&i2c->dev, "regmap_init() for ak1951 failed: %d\n", ret);
		return ret;
	}

	ak4957_data = ak4957;
	ret = snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_ak4957, &ak4957_dai[0], ARRAY_SIZE(ak4957_dai));
	if (ret < 0){
		kfree(ak4957);
		akdbgprt("\t[AK4957 Error!] %s(%d)\n",__FUNCTION__,__LINE__);
	}
	return ret;
}

static int  ak4957_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static struct of_device_id ak4957_of_match[] = {
	{ .compatible = "ambarella,ak4957",},
	{},
};
MODULE_DEVICE_TABLE(of, ak4957_of_match);

static const struct i2c_device_id ak4957_i2c_id[] = {
	{ "ak4957", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak4957_i2c_id);

static struct i2c_driver ak4957_i2c_driver = {
	.driver = {
		.name = "ak4957",
		.owner = THIS_MODULE,
		.of_match_table = ak4957_of_match,
	},
	.probe = ak4957_i2c_probe,
	.remove = ak4957_i2c_remove,
	.id_table = ak4957_i2c_id,
};

static int __init ak4957_modinit(void)
{
	int ret;
	akdbgprt("\t[AK4957] %s(%d)\n", __FUNCTION__,__LINE__);
	ret = i2c_add_driver(&ak4957_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register ak4957 I2C driver: %d\n",ret);
	return ret;
}

module_init(ak4957_modinit);

static void __exit ak4957_exit(void)
{
	i2c_del_driver(&ak4957_i2c_driver);
}
module_exit(ak4957_exit);

MODULE_DESCRIPTION("ASoC ak4957 codec driver");
MODULE_LICENSE("GPL");
