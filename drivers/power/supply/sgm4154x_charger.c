// SPDX-License-Identifier: GPL-2.0
// SGM4154x driver version 2021-09-09-003
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.sg-micro.com

#define pr_fmt(fmt) "--PM-SGM_CHARGER: " fmt
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/usb/phy.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/power_supply.h>
#include <linux/acpi.h>
#include <linux/gpio.h>
#include <linux/extcon-provider.h>
#include <linux/of_gpio.h>
#include <linux/qti_power_supply.h>
#include <linux/pmic-voter.h>
#include "sgm4154x_charger.h"
#include <linux/his_debug_base.h>

static struct power_supply_desc sgm4154x_power_supply_desc;
static char *sgm4154x_charger_supplied_to[] = {
	"battery",	
};
static struct reg_default sgm4154x_reg_defs[] = {
	{SGM4154x_CHRG_CTRL_0, 0x0a},
	{SGM4154x_CHRG_CTRL_1, 0x1a},
	{SGM4154x_CHRG_CTRL_2, 0x88},
	{SGM4154x_CHRG_CTRL_3, 0x22},
	{SGM4154x_CHRG_CTRL_4, 0x58},
	{SGM4154x_CHRG_CTRL_5, 0x9f},
	{SGM4154x_CHRG_CTRL_6, 0xe6},
	{SGM4154x_CHRG_CTRL_7, 0x4e},
    {SGM4154x_CHRG_STAT,   0x00},
	{SGM4154x_CHRG_FAULT,  0x00},	
	{SGM4154x_CHRG_CTRL_a, 0x00},//
	{SGM4154x_CHRG_CTRL_b, 0x64},
	{SGM4154x_CHRG_CTRL_c, 0x75},
	{SGM4154x_CHRG_CTRL_d, 0x00},
	{SGM4154x_INPUT_DET,   0x00},
	{SGM4154x_CHRG_CTRL_f, 0x00},	
};

/* SGM4154x REG06 BOOST_LIM[5:4], uV */
static const unsigned int BOOST_VOLT_LIMIT[] = {
	4850000, 5000000, 5150000, 5300000		
};

static const unsigned int OVP_THD[] = {
	5500000, 6500000, 10500000, 14000000
};
 /* SGM4154x REG02 BOOST_LIM[7:7], uA */
#if defined(__SGM41542_CHIP_ID__)|| defined(__SGM41541_CHIP_ID__) || defined(__SGM41543D_CHIP_ID__)|| defined(__SGM41543_CHIP_ID__)
static const unsigned int BOOST_CURRENT_LIMIT[] = {
	1200000, 2000000
};
#else
static const unsigned int BOOST_CURRENT_LIMIT[] = {
	500000, 1200000
};
#endif

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))

static const unsigned int IPRECHG_CURRENT_STABLE[] = {
	5000, 10000, 15000, 20000, 30000, 40000, 50000, 60000,
	80000, 100000, 120000, 140000, 160000, 180000, 200000, 240000
};

static const unsigned int ITERM_CURRENT_STABLE[] = {
	5000, 10000, 15000, 20000, 30000, 40000, 50000, 60000,
	80000, 100000, 120000, 140000, 160000, 180000, 200000, 240000
};
#endif

enum SGM4154x_VREG_FT {
	VREG_FT_DISABLE,
	VREG_FT_UP_8mV,
	VREG_FT_DN_8mV,
	VREG_FT_DN_16mV,		
};

enum SGM4154x_VINDPM_OS {
	VINDPM_OS_3900mV,
	VINDPM_OS_5900mV,
	VINDPM_OS_7500mV,
	VINDPM_OS_10500mV,		
};

enum SGM4154x_QC_VOLT {
	QC_20_5000mV,
	QC_20_9000mV,
	QC_20_12000mV,		
};

static enum power_supply_usb_type sgm4154x_usb_type[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,	
};

static struct sgm4154x_device *g_sgm = NULL;
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
extern void sgm_usb_switch_gpio(int value);
extern bool sgm_typec_charger_ok(void);
extern  int sgm_typec_otg_ok(void);
extern void ucsi_usb_role_switch(void);
int register_sgm_usb_notifier(struct notifier_block *nb);
int unregister_sgm_usb_notifier(struct notifier_block *nb);
#endif /*CONFIG_DOUBLE_CHARGERS*/
static int sgm4154x_get_batt_health(struct sgm4154x_device *sgm, u8 *batt_health);
static int sgm4154x_usb_notifier(struct notifier_block *nb, unsigned long val,
				void *priv)
{
	struct sgm4154x_device *sgm =
			container_of(nb, struct sgm4154x_device, usb_nb);

	sgm->usb_event = val;

	queue_work(system_power_efficient_wq, &sgm->usb_work);

	return NOTIFY_OK;
}

#if 0
static int sgm4154x_get_term_curr(struct sgm4154x_device *sgm)
{
	int ret;
	int reg_val;
	int offset = SGM4154x_TERMCHRG_I_MIN_uA;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_3, &reg_val);
	if (ret)
		return ret;

	reg_val &= SGM4154x_TERMCHRG_CUR_MASK;
	reg_val = reg_val * SGM4154x_TERMCHRG_CURRENT_STEP_uA + offset;
	return reg_val;
}

static int sgm4154x_get_prechrg_curr(struct sgm4154x_device *sgm)
{
	int ret;
	int reg_val;
	int offset = SGM4154x_PRECHRG_I_MIN_uA;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_3, &reg_val);
	if (ret)
		return ret;

	reg_val = (reg_val&SGM4154x_PRECHRG_CUR_MASK)>>4;
	reg_val = reg_val * SGM4154x_PRECHRG_CURRENT_STEP_uA + offset;
	return reg_val;
}

static int sgm4154x_get_ichg_curr(struct sgm4154x_device *sgm)
{
	int ret;
	int ichg;
	unsigned int curr;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_2, &ichg);
	if (ret)
		return ret;	

	ichg &= SGM4154x_ICHRG_CUR_MASK;
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))	
	if (ichg <= 0x8)
		curr = ichg * 5000;
	else if (ichg <= 0xF)
		curr = 40000 + (ichg - 0x8) * 10000;
	else if (ichg <= 0x17)
		curr = 110000 + (ichg - 0xF) * 20000;
	else if (ichg <= 0x20)
		curr = 270000 + (ichg - 0x17) * 30000;
	else if (ichg <= 0x30)
		curr = 540000 + (ichg - 0x20) * 60000;
	else if (ichg <= 0x3C)
		curr = 1500000 + (ichg - 0x30) * 120000;
	else
		curr = 3000000;
#else
	curr = ichg * SGM4154x_ICHRG_I_STEP_uA;
#endif	
	return curr;
}
#endif

static int sgm4154x_set_term_curr(struct sgm4154x_device *sgm, int uA)
{
	int reg_val;	

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))	
	
	for(reg_val = 1; reg_val < 16 && uA >= ITERM_CURRENT_STABLE[reg_val]; reg_val++)
		;
	reg_val--;
#else
	if (uA < SGM4154x_TERMCHRG_I_MIN_uA)
		uA = SGM4154x_TERMCHRG_I_MIN_uA;
	else if (uA > SGM4154x_TERMCHRG_I_MAX_uA)
		uA = SGM4154x_TERMCHRG_I_MAX_uA;
	
	reg_val = (uA - SGM4154x_TERMCHRG_I_MIN_uA) / SGM4154x_TERMCHRG_CURRENT_STEP_uA;
#endif
	return regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_3,
				  SGM4154x_TERMCHRG_CUR_MASK, reg_val);
}

static int sgm4154x_set_prechrg_curr(struct sgm4154x_device *sgm, int uA)
{
	int reg_val;	
	
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	for(reg_val = 1; reg_val < 16 && uA >= IPRECHG_CURRENT_STABLE[reg_val]; reg_val++)
		;
	reg_val--;
#else
	if (uA < SGM4154x_PRECHRG_I_MIN_uA)
		uA = SGM4154x_PRECHRG_I_MIN_uA;
	else if (uA > SGM4154x_PRECHRG_I_MAX_uA)
		uA = SGM4154x_PRECHRG_I_MAX_uA;

	reg_val = (uA - SGM4154x_PRECHRG_I_MIN_uA) / SGM4154x_PRECHRG_CURRENT_STEP_uA;
#endif
	reg_val = reg_val << 4;
	return regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_3,
				  SGM4154x_PRECHRG_CUR_MASK, reg_val);
}

static int sgm_disable_chg_vote_callback(struct votable *votable, void *data,
			int chg_disable, const char *client)
{
    struct sgm4154x_device *sgm = data;
    int ret;

    pr_err("%s chg_disable %d\n",__func__,chg_disable);
    if(!sgm->chg_en){
	    ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_1, SGM4154x_CHRG_EN,
	                     chg_disable ? 0 : SGM4154x_CHRG_EN);
	    if(ret < 0){
		pr_err("%s write SGM4154x_CHRG_CTRL_1 fail\n",__func__);
		return ret;
	    }
    }else{
		gpio_direction_output(sgm->chg_en,chg_disable ? 1 : 0);
    }
	return 0;
}

static int sgm4154x_set_ichrg_curr(struct votable *votable, void *data,
			int uA, const char *client)
{
	struct sgm4154x_device *sgm = data;
	int ret;
	int reg_val;
	
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	if (uA <= 40000)
		reg_val = uA / 5000;
	else if (uA < 50000)
		reg_val = 0x08;
	else if (uA <= 110000)
		reg_val = 0x08 + (uA -40000) / 10000;
	else if (uA < 130000)
		reg_val = 0x0F;
	else if (uA <= 270000)
		reg_val = 0x0F + (uA -110000) / 20000;
	else if (uA < 300000)
		reg_val = 0x17;
	else if (uA <= 540000)
		reg_val = 0x17 + (uA -270000) / 30000;
	else if (uA < 600000)
		reg_val = 0x20;
	else if (uA <= 1500000)
		reg_val = 0x20 + (uA -540000) / 60000;
	else if (uA < 1620000)
		reg_val = 0x30;
	else if (uA <= 2940000)
		reg_val = 0x30 + (uA -1500000) / 120000;
	else 
		reg_val = 0x3d;
#else
	if (uA < SGM4154x_ICHRG_I_MIN_uA)
		uA = SGM4154x_ICHRG_I_MIN_uA;
	else if ( uA > sgm->init_data.max_ichg)
		uA = sgm->init_data.max_ichg;
	
	reg_val = uA / SGM4154x_ICHRG_I_STEP_uA;
#endif	

	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_2,
				  SGM4154x_ICHRG_CUR_MASK, reg_val);
	
	return ret;
}

static int sgm4154x_set_chrg_volt(struct sgm4154x_device *sgm, int chrg_volt)
{
	int ret;
	int reg_val;

	if (chrg_volt < SGM4154x_VREG_V_MIN_uV)
		chrg_volt = SGM4154x_VREG_V_MIN_uV;
	else if (chrg_volt > sgm->init_data.max_vreg)
		chrg_volt = sgm->init_data.max_vreg;
	
	
	reg_val = (chrg_volt-SGM4154x_VREG_V_MIN_uV) / SGM4154x_VREG_V_STEP_uV;
	reg_val = reg_val<<3;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_4,
				  SGM4154x_VREG_V_MASK, reg_val);

	return ret;
}

static int sgm4154x_set_otgf_itremr(struct sgm4154x_device *sgm, int val)
{
	int ret = 0;

	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d, SGM4154x_OTGF_ITREMR,
	                     val);
	if(ret < 0)
		pr_err("%s write SGM4154x_CHRG_CTRL_1 fail\n",__func__);

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_d, &val);
	dev_err(sgm->dev,"%s val %d\n",__func__,val);

	return ret;
}

#if 0
static int sgm4154x_get_chrg_volt(struct sgm4154x_device *sgm)
{
	int ret;
	int vreg_val;
	int chrg_volt = 0;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_4, &vreg_val);
	if (ret)
		return ret;	

	vreg_val = (vreg_val & SGM4154x_VREG_V_MASK)>>3;

	if (15 == vreg_val)
		chrg_volt = 4352000; //default
	else if (vreg_val < 25)	
		chrg_volt = vreg_val*SGM4154x_VREG_V_STEP_uV + SGM4154x_VREG_V_MIN_uV;	

	return chrg_volt;
}
#endif

#if 0//(defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__) || defined(__SGM41543D_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
static int sgm4154x_enable_qc20_hvdcp_9v(struct sgm4154x_device *sgm)
{
	int ret;
	int dp_val, dm_val;
	
	/*dp and dm connected,dp 0.6V dm Hiz*/
    dp_val = 0x2<<3;
    ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 0.6V
    if (ret)
        return ret;
	
	dm_val = 0;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm Hiz
    if (ret)
        return ret;
    //mdelay(1000);
	msleep(1400);	
	
	/* dp 3.3v and dm 0.6v out 9V */
	dp_val = SGM4154x_DP_VSEL_MASK;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 3.3v
	if (ret)
		return ret;	
	
	dm_val = 0x2<<1;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0.6v
	
	return ret;
}

static int sgm4154x_enable_qc20_hvdcp_12v(struct sgm4154x_device *sgm)
{
	int ret;
	int dp_val, dm_val;
	
	/*dp and dm connected,dp 0.6V dm Hiz*/
    dp_val = 0x2<<3;
    ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 0.6V
    if (ret)
        return ret;
	
	dm_val = 0<<1;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm Hiz
    if (ret)
        return ret;
    //mdelay(1000);
	msleep(1250);
	#if 0
    dm_val = 0x2;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0V
	mdelay(1);
	#endif
	/* dp 0.6v and dm 0.6v out 12V */
	dp_val = 0x2<<3;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 0.6v
	if (ret)
		return ret;
	//mdelay(1250);
	
	dm_val = 0x2<<1;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0.6v
	
	return ret;
}

/* step 1. entry QC3.0 mode
   step 2. up or down 200mv 
   step 3. retry step 2 */
static int sgm4154x_enable_qc30_hvdcp(struct sgm4154x_device *sgm)
{
	int ret;
	int dp_val, dm_val;
	
	dp_val = 0x2<<3;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 0.6v
	if (ret)
		return ret;
	
	dm_val = 0;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm Hiz
    if (ret)
        return ret;
    //mdelay(1000);
	msleep(1250);
	dm_val = 0x2;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0V
	mdelay(10);
	
	dm_val = SGM4154x_DM_VSEL_MASK;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 3.3v		

	return ret;
}

// Must enter 3.0 mode to call ,otherwise cannot step correctly.
static int sgm4154x_qc30_step_up_vbus(struct sgm4154x_device *sgm)
{
	int ret;
	int dp_val;
	
	/*  dm 3.3v to dm 0.6v  step up 200mV when IC is QC3.0 mode*/
	dp_val = SGM4154x_DP_VSEL_MASK;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 3.3v
	if (ret)
		return ret;
	
	dp_val = 0x2<<3;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DP_VSEL_MASK, dp_val); //dp 0.6v
	if (ret)
		return ret;	
	
	udelay(100);
	return ret;
}
// Must enter 3.0 mode to call ,otherwise cannot step correctly.
static int sgm4154x_qc30_step_down_vbus(struct sgm4154x_device *sgm)
{
	int ret;	
	int dm_val;
	
	/* dp 0.6v and dm 0.6v step down 200mV when IC is QC3.0 mode*/
	dm_val = 0x2<<1;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 0.6V
    if (ret)
        return ret;	
	
	dm_val = SGM4154x_DM_VSEL_MASK;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_d,
				  SGM4154x_DM_VSEL_MASK, dm_val); //dm 3.3v
	udelay(100);	

	return ret;
}


// fine tuning termination voltage,to Improve accuracy
static int sgm4154x_vreg_fine_tuning(struct sgm4154x_device *sgm,enum SGM4154x_VREG_FT ft)
{
	int ret;	
	int reg_val;
	
	switch(ft) {
		case VREG_FT_DISABLE:			
			reg_val = 0;
			break;

		case VREG_FT_UP_8mV:			
			reg_val = SGM4154x_VREG_FT_UP_8mV;
			break;

		case VREG_FT_DN_8mV:			
			reg_val = SGM4154x_VREG_FT_DN_8mV;
			break;

		case VREG_FT_DN_16mV:			
			reg_val = SGM4154x_VREG_FT_DN_16mV;
			break;

		default:
			reg_val = 0;
			break;
	}
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_f,
				  SGM4154x_VREG_FT_MASK, reg_val);
	pr_err("%s reg_val:%d\n",__func__,reg_val);	

	return ret;
}

#endif

static int sgm4154x_get_vindpm_offset_os(struct sgm4154x_device *sgm)
{
	int ret;
	int reg_val;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_f, &reg_val);
	if (ret)
		return ret;	

	reg_val = reg_val & SGM4154x_VINDPM_OS_MASK;	

	return reg_val;
}

static int sgm4154x_get_input_volt_lim(struct sgm4154x_device *sgm)
{
	int ret;
	int offset = 0;
	int vlim;
	int temp;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_6, &vlim);
	if (ret)
		return ret;
	
	temp = sgm4154x_get_vindpm_offset_os(sgm);
	if (VINDPM_OS_3900mV == temp)
		offset = 3900000; //uv
	else if (VINDPM_OS_5900mV == temp)
		offset = 5900000;
	else if (VINDPM_OS_7500mV == temp)
		offset = 7500000;
	else if (VINDPM_OS_10500mV == temp)
		offset = 10500000;
	
	vlim = offset + (vlim & 0x0F) * SGM4154x_VINDPM_STEP_uV;
	return vlim;
}

static int sgm4154x_set_vindpm_offset_os(struct sgm4154x_device *sgm,enum SGM4154x_VINDPM_OS offset_os)
{
	int ret;	
	
	
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_f,
				  SGM4154x_VINDPM_OS_MASK, offset_os);
	
	if (ret){
		pr_err("%s fail\n",__func__);
		return ret;
	}
	
	return ret;
}

static int sgm4154x_set_input_volt_lim(struct sgm4154x_device *sgm, unsigned int vindpm)
{
	int ret;
	unsigned int offset;
	u8 reg_val;
	enum SGM4154x_VINDPM_OS os_val;

	if (vindpm < SGM4154x_VINDPM_V_MIN_uV ||
	    vindpm > SGM4154x_VINDPM_V_MAX_uV)
 		return -EINVAL;	
	
	if (vindpm < 5900000){
		os_val = VINDPM_OS_3900mV;
		offset = 3900000;
	}		
	else if (vindpm >= 5900000 && vindpm < 7500000){
		os_val = VINDPM_OS_5900mV;
		offset = 5900000; //uv
	}		
	else if (vindpm >= 7500000 && vindpm < 10500000){
		os_val = VINDPM_OS_7500mV;
		offset = 7500000; //uv
	}		
	else{
		os_val = VINDPM_OS_10500mV;
		offset = 10500000; //uv
	}		
	
	sgm4154x_set_vindpm_offset_os(sgm,os_val);
	reg_val = (vindpm - offset) / SGM4154x_VINDPM_STEP_uV;	

	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_6,
				  SGM4154x_VINDPM_V_MASK, reg_val); 

	return ret;
}

bool sgm4154x_is_hvdcp(struct sgm4154x_device *sgm,enum SGM4154x_QC_VOLT val)
{	
    int i = 20;	
	int vlim;
	int temp;

	vlim = sgm4154x_get_input_volt_lim(sgm);
	
	if (QC_20_9000mV == val)
	{
		sgm4154x_set_input_volt_lim(sgm,8000000); //8v
	}
	else if (QC_20_12000mV == val)
	{
		sgm4154x_set_input_volt_lim(sgm,11000000); //11v
	}
	else
	{
		sgm4154x_set_input_volt_lim(sgm,4500000); //4.5v
		return 0;
	}
	mdelay(1);
	while(i--){
		regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_a, &temp);
	 		
		if(0 == (temp&0x40)){
			return 1;
		}
		else if(1 == !!(temp&0x40) && i == 1){
			sgm4154x_set_input_volt_lim(sgm,vlim);
			return 0;
		}
		mdelay(10);
	}	
	return 0;
}

static int sgm4154x_set_input_curr_lim(struct sgm4154x_device *sgm, int iindpm)
{
	
	int reg_val;	
	
	if (iindpm < SGM4154x_IINDPM_I_MIN_uA ||
			iindpm > SGM4154x_IINDPM_I_MAX_uA)
		return -EINVAL;
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	reg_val = (iindpm-SGM4154x_IINDPM_I_MIN_uA) / SGM4154x_IINDPM_STEP_uA;
#else	
	if (iindpm >= SGM4154x_IINDPM_I_MIN_uA && iindpm <= 3100000)//default
		reg_val = (iindpm-SGM4154x_IINDPM_I_MIN_uA) / SGM4154x_IINDPM_STEP_uA;
	else if (iindpm > 3100000 && iindpm < SGM4154x_IINDPM_I_MAX_uA)
		reg_val = 0x1E;
	else
		reg_val = 0x1F;
#endif
	return regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_0,
				  SGM4154x_IINDPM_I_MASK, reg_val);
}

static int sgm4154x_get_input_curr_lim(struct sgm4154x_device *sgm)
{
	int ret;	
	int ilim;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_0, &ilim);
	if (ret)
		return ret;
	if (SGM4154x_IINDPM_I_MASK == (ilim & SGM4154x_IINDPM_I_MASK))
		return SGM4154x_IINDPM_I_MAX_uA;

	ilim = (ilim & SGM4154x_IINDPM_I_MASK)*SGM4154x_IINDPM_STEP_uA + SGM4154x_IINDPM_I_MIN_uA;

	return ilim;
}

static int sgm4154x_set_watchdog_timer(struct sgm4154x_device *sgm, int time)
{
	int ret;
	u8 reg_val;

	if (time == 0)
		reg_val = SGM4154x_WDT_TIMER_DISABLE;
	else if (time == 40)
		reg_val = SGM4154x_WDT_TIMER_40S;
	else if (time == 80)
		reg_val = SGM4154x_WDT_TIMER_80S;
	else
		reg_val = SGM4154x_WDT_TIMER_160S;	

	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_5,
				SGM4154x_WDT_TIMER_MASK, reg_val);

	return ret;
}

#if 0
static int sgm4154x_set_wdt_rst(struct sgm4154x_device *sgm, bool is_rst)
{
	int val = 0;
	
	if (is_rst)
		val = SGM4154x_WDT_RST_MASK;
	else
		val = 0;
	return regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_1,
				  SGM4154x_WDT_RST_MASK, val);	
}
#endif

static int sgm4154x_get_state(struct sgm4154x_device *sgm,
			     struct sgm4154x_state *state)
{
	int chrg_stat;
	int fault;
	int chrg_param_0,chrg_param_1,chrg_param_2;
	int ret;
	u8 current_health = SGM4154x_TEMP_NORMAL;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_STAT, &chrg_stat);
	if (ret){
		ret = regmap_read(sgm->regmap, SGM4154x_CHRG_STAT, &chrg_stat);
		if (ret){
			pr_err("%s read SGM4154x_CHRG_STAT fail\n",__func__);
			return ret;
		}
	}

	state->chrg_type = chrg_stat & SGM4154x_VBUS_STAT_MASK;
	state->chrg_stat = chrg_stat & SGM4154x_CHG_STAT_MASK;
	state->online = !!(chrg_stat & SGM4154x_PG_STAT);
	state->therm_stat = !!(chrg_stat & SGM4154x_THERM_STAT);
	state->vsys_stat = !!(chrg_stat & SGM4154x_VSYS_STAT);
	
	pr_debug("%s chrg_type =%d,chrg_stat =%d online = %d\n",__func__, state->chrg_type,state->chrg_stat, state->online);
	

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_FAULT, &fault);
	if (ret){
		pr_err("%s read SGM4154x_CHRG_FAULT fail\n",__func__);
		return ret;
	}
	state->chrg_fault = fault;
	if((state->chrg_fault & SGM4154x_CHRG_FAULT_MASK) == SGM4154x_CHRG_FAULT_CHRG_SAFETY_TIMER_EXPIRED){
		dev_err(sgm->dev, "charge safety timer expired, current flag: %d\n", sgm->monitored_items.charge_safety_timer_expired);
		if(!sgm->monitored_items.charge_safety_timer_expired){
			sgm->monitored_items.charge_safety_timer_expired = true;
			cancel_delayed_work_sync(&sgm->recharge_delayed_work);
			schedule_delayed_work(&sgm->recharge_delayed_work, msecs_to_jiffies(500));
		}
	}else{
		if(sgm->monitored_items.charge_safety_timer_expired)
			sgm->monitored_items.charge_safety_timer_expired = false;
	}
	//state->ntc_fault = fault & SGM4154x_TEMP_MASK;
	ret = sgm4154x_get_batt_health(sgm, &current_health);
	if(ret < 0)
		return ret;

	state->health = current_health;
	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_0, &chrg_param_0);
	if (ret){
		pr_err("%s read SGM4154x_CHRG_CTRL_0 fail\n",__func__);
		return ret;
	}
	state->hiz_en = !!(chrg_param_0 & SGM4154x_HIZ_EN);
	
	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_5, &chrg_param_1);
	if (ret){
		pr_err("%s read SGM4154x_CHRG_CTRL_5 fail\n",__func__);
		return ret;
	}
	state->term_en = !!(chrg_param_1 & SGM4154x_TERM_EN);
	
	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_a, &chrg_param_2);
	if (ret){
		pr_err("%s read SGM4154x_CHRG_CTRL_a fail\n",__func__);
		return ret;
	}
	state->vbus_gd = !!(chrg_param_2 & SGM4154x_VBUS_GOOD);
	if(!state->vbus_gd){
		if(sgm->monitored_items.charge_safety_timer_expired)
			sgm->monitored_items.charge_safety_timer_expired = false;
	}

	return 0;
}

#if 0
static int sgm4154x_set_hiz_en(struct sgm4154x_device *sgm, bool hiz_en)
{
	int reg_val;

	dev_notice(sgm->dev, "%s:%d", __func__, hiz_en);
	reg_val = hiz_en ? SGM4154x_HIZ_EN : 0;

	return regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_0,
				  SGM4154x_HIZ_EN, reg_val);
}
#endif

static int sgm4154x_is_enabled_charger(struct sgm4154x_device *sgm)
{
	int temp = 0;
	int ret = 0;

	if(!sgm->chg_en){
		ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_1, &temp);
		return (temp&SGM4154x_CHRG_EN)? 1 : 0;
	}else{
		ret = gpio_get_value(sgm->chg_en);
		return ret ? 0 : 1;
	}
}

#if 0
float sgm4154x_get_charger_output_power(struct sgm4154x_device *sgm)
{
    int ret;
	int i = 0x1F;
	int j = 0;
	int vlim;
	int ilim;
	int temp;
	int offset;
	int output_volt;
	int output_curr;
	float o_i,o_v;

    ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_6, &vlim); //read default setting to save
	if (ret){
		pr_err("%s read SGM4154x_CHRG_CTRL_6 fail\n",__func__);
		return ret;
	}
	
	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_0, &ilim); //read default setting to save
	if (ret){
		pr_err("%s read SGM4154x_CHRG_CTRL_0 fail\n",__func__);
		return ret;
	}
	
	
	regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_6,
				  SGM4154x_VINDPM_V_MASK, 0);
	while(i--){
		
		regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_0,
				  SGM4154x_IINDPM_I_MASK, i);
		mdelay(50);
		ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_a, &temp);
		if (ret){
			pr_err("%s read SGM4154x_CHRG_CTRL_a fail\n",__func__);
			return ret;
		}
		if (1 == !!(temp&0x20)){
			output_curr = 100 + i*100; //mA
			if (0x1F == i)
				output_curr = 3800;	      //mA
		}			
	}
	
	regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_0,
				  SGM4154x_IINDPM_I_MASK, SGM4154x_IINDPM_I_MASK);
	for(j = 0;j <= 0xF;j ++)
	{
		regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_6,
				  SGM4154x_VINDPM_V_MASK, j);
		
		mdelay(10);
		ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_a, &temp);
		if (ret){
			pr_err("%s read SGM4154x_CHRG_CTRL_a fail\n",__func__);
			return ret;
		}
		
		if (1 == !!(temp&0x40)){
			
			temp = sgm4154x_get_vindpm_offset_os(sgm);
			if (0 == temp)
				offset = 3900;  //mv
			else if (1 == temp)
				offset = 5900;  //mv
			else if (2 == temp)
				offset = 7500;  //mv
			else if (3 == temp)
				offset = 10500; //mv
			output_volt = offset + j*100; //mv			
		}
		
	}	
	o_i = (float)output_curr/1000;
	o_v = (float)output_volt/1000;
    return o_i * o_v;
}
#endif

static int sgm4154x_set_vac_ovp(struct sgm4154x_device *sgm, int ovp_thd)
{
	int reg_val = -1;
	int i = 0;
	
	dev_notice(sgm->dev, "%s:%d", __func__, ovp_thd);

	while(i<4){
		if (ovp_thd == OVP_THD[i]){
			reg_val = i;
			break;
		}
		i++;
	}
	if (reg_val < 0)
		return reg_val;
	reg_val = reg_val << 6;
	
	return regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_6,
				  SGM4154x_VAC_OVP_MASK, reg_val);
}

static int sgm4154x_set_recharge_volt(struct sgm4154x_device *sgm, int recharge_volt)
{
	int reg_val;
	dev_notice(sgm->dev, "%s:%d", __func__, recharge_volt);
	reg_val = (recharge_volt - SGM4154x_VRECHRG_OFFSET_mV) / SGM4154x_VRECHRG_STEP_mV;

	return regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_4,
				  SGM4154x_VRECHARGE, reg_val);
}

static int sgm4154x_set_vdpm_track(struct sgm4154x_device *sgm, int vdpm_thd)
{
	int reg_val = -1;

	dev_notice(sgm->dev, "%s:vdpm_thd %dmV", __func__, vdpm_thd);

	switch(vdpm_thd) {
		case 0:
			reg_val = 0x00;
			break;
		case 200:
			reg_val = 0x01;
			break;
		case 250:
			reg_val = 0x02;
			break;
		case 300:
			reg_val = 0x03;
			break;
		default:
			reg_val = 0x00;
			break;
	}

	return regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_7,
				  SGM4154x_VDPM_TRACK_MASK, reg_val);
}


#if (defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__) || defined(__SGM41543D_CHIP_ID__)|| defined(__SGM41513D_CHIP_ID__))
extern struct power_supply_desc usb_psy_desc;
static int get_charger_type(struct sgm4154x_device * sgm)
{
	enum power_supply_usb_type usb_type;
	switch(sgm->state.chrg_type) {
		case SGM4154x_USB_SDP:
			usb_type = POWER_SUPPLY_USB_TYPE_SDP;
			break;

		case SGM4154x_USB_CDP:
			usb_type = POWER_SUPPLY_USB_TYPE_CDP;
			break;

		case SGM4154x_USB_DCP:
			usb_type = POWER_SUPPLY_USB_TYPE_DCP;
			break;

		case SGM4154x_NON_STANDARD:
			usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			break;

		default:
			usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			break;
	}
	pr_err("%s usb_type:%d\n",__func__, usb_type);
	
	return usb_type;
}
#endif

static int sgm4154x_get_batt_health(struct sgm4154x_device *sgm, u8 *batt_health){
	int ret = 0;
	static u8 last_batt_health = SGM4154x_TEMP_NORMAL;
	union power_supply_propval pval = {0, };

	if(!sgm->battery){
		sgm->battery = power_supply_get_by_name(sgm4154x_charger_supplied_to[0]);
		if(!sgm->battery){
			dev_err(sgm->dev, "Could not get power_supply %s\n", sgm4154x_charger_supplied_to[0]);
			return PTR_ERR(sgm->battery);
		}
	}
	ret = power_supply_get_property(sgm->battery, POWER_SUPPLY_PROP_TEMP, &pval);
	if(ret < 0){
		pr_err("%s get battery temp fail\n",__func__);
		return ret;
	}

	if(pval.intval <= (sgm->data.temp_t1_thres * 10))
		*batt_health = SGM4154x_TEMP_COLD;
	else if(pval.intval > (sgm->data.temp_t1_thres * 10) && pval.intval <= (sgm->data.temp_t2_thres * 10))
		*batt_health = SGM4154x_TEMP_COOL;
	else if(pval.intval > (sgm->data.temp_t2_thres * 10) && pval.intval < (sgm->data.temp_t3_thres * 10))
		*batt_health = SGM4154x_TEMP_NORMAL;
	else if(pval.intval >= (sgm->data.temp_t3_thres) && pval.intval < (sgm->data.temp_t4_thres * 10))
		*batt_health = SGM4154x_TEMP_WARM;
	else if(pval.intval >= (sgm->data.temp_t4_thres * 10))
		*batt_health = SGM4154x_TEMP_HOT;

	if(last_batt_health != *batt_health) {
		pr_err("%s health changed from %d to %d\n",
					__func__, last_batt_health, *batt_health);
		last_batt_health = *batt_health;
	}

	return 0;
}

static int sgm4154x_get_batt_capacity(struct sgm4154x_device *sgm){
	int ret = 0;
	union power_supply_propval pval = {0, };
	if(!sgm->battery){
		sgm->battery = power_supply_get_by_name(sgm4154x_charger_supplied_to[0]);
		if(!sgm->battery){
			dev_err(sgm->dev, "Could not get power_supply %s\n", sgm4154x_charger_supplied_to[0]);
			return PTR_ERR(sgm->battery);
		}
	}
	ret = power_supply_get_property(sgm->battery, POWER_SUPPLY_PROP_CAPACITY, &pval);
	if(ret < 0){
		pr_err("%s get battery capacity fail\n",__func__);
		return ret;
	}
	pr_err("%s get battery capacity: %d\n",__func__, pval.intval);
	return pval.intval;
}

static int sgm4154x_enable_vbus(struct sgm4154x_device *sgm)
{
	int ret = 0;
	
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_1, SGM4154x_OTG_EN,
                     SGM4154x_OTG_EN);
	return ret;
}

static int sgm4154x_disable_vbus(struct sgm4154x_device *sgm)
{
	int ret = 0;

	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_1, SGM4154x_OTG_EN,
                     0);

	return ret;
}

static int sgm4154x_is_enabled_vbus(struct sgm4154x_device *sgm)
{
	int temp = 0;
	int ret = 0;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_1, &temp);
	return (temp&SGM4154x_OTG_EN)? 1 : 0;
}

static sgm4154x_init_typec_class(struct sgm4154x_device *sgm){
	int rc = 0;

	mutex_init(&sgm->typec_lock);

	sgm->typec_caps.type = TYPEC_PORT_DRP;
	sgm->typec_caps.data = TYPEC_PORT_DRD;
	sgm->typec_partner_desc.usb_pd = false;
	sgm->typec_partner_desc.accessory = TYPEC_ACCESSORY_NONE;
	sgm->typec_caps.revision = 0x0130;

	sgm->typec_port = typec_register_port(sgm->dev, &sgm->typec_caps);
	if (IS_ERR(sgm->typec_port)) {
		rc = PTR_ERR(sgm->typec_port);
		pr_err("failed to register typec_port rc=%d\n", rc);
		return rc;
	}

	return rc;
}
static sgm4154x_typec_register_partner(struct sgm4154x_device *sgm){
	int rc = 0;

	mutex_lock(&sgm->typec_lock);

	if (!sgm->typec_port)
		goto unlock;

	if (!sgm->typec_partner) {
			sgm->typec_partner_desc.accessory =
					TYPEC_ACCESSORY_NONE;

		sgm->typec_partner = typec_register_partner(sgm->typec_port,
				&sgm->typec_partner_desc);
		if (IS_ERR(sgm->typec_partner)) {
			rc = PTR_ERR(sgm->typec_partner);
			pr_err("failed to register typec_partner rc=%d\n", rc);
			goto unlock;
		}
		pr_info("Registering typeC partner\n");
	}

	typec_set_data_role(sgm->typec_port, TYPEC_DEVICE);
	typec_set_pwr_role(sgm->typec_port, TYPEC_SINK);

unlock:
	mutex_unlock(&sgm->typec_lock);
	return rc;
}

static void sgm4154x_typec_unregister_partner(struct sgm4154x_device *sgm)
{
	mutex_lock(&sgm->typec_lock);

	if (!sgm->typec_port)
		goto unlock;

	if (sgm->typec_partner) {
		pr_info("Un-registering typeC partner\n");
		typec_unregister_partner(sgm->typec_partner);
		sgm->typec_partner = NULL;
	}

unlock:
	mutex_unlock(&sgm->typec_lock);
}

static int sgm4154x_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		return true;
	default:
		return false;
	}
}
static int sgm4154x_charger_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	struct sgm4154x_device *sgm = power_supply_get_drvdata(psy);
	int ret = -EINVAL;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sgm4154x_set_input_curr_lim(sgm, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = vote(sgm->fcc_votable, BATT_PROFILE_VOTER, true, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		ret = vote(sgm->chg_disable_votable, USER_VOTER, val->intval ? false : true, 0);
		power_supply_changed(sgm->battery);
		break;
#if 0
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = sgm4154x_set_input_volt_lim(sgm, val->intval);
		break;
		
#endif
	default:
		return -EINVAL;
	}

	return ret;
}

static int sgm4154x_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct sgm4154x_device *sgm = power_supply_get_drvdata(psy);
	struct sgm4154x_state state;
	int ret = 0;

	mutex_lock(&sgm->lock);
	ret = sgm4154x_get_state(sgm, &state);
	sgm->state = state;
	mutex_unlock(&sgm->lock);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!state.chrg_type || (state.chrg_type == SGM4154x_OTG_MODE))
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (!state.chrg_stat)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else if (state.chrg_stat == SGM4154x_TERM_CHRG){
			val->intval = POWER_SUPPLY_STATUS_FULL;
			ret = sgm4154x_get_batt_capacity(sgm);
			if(ret < 0)
				return ret;
			if(ret != 100)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
		}
		else{
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			ret = sgm4154x_get_batt_capacity(sgm);
			if(ret < 0)
				return ret;
			if(ret == 100)
				val->intval = POWER_SUPPLY_STATUS_FULL;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		switch (state.chrg_stat) {		
		case SGM4154x_PRECHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case SGM4154x_FAST_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;		
		case SGM4154x_TERM_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case SGM4154x_NOT_CHRGING:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = SGM4154x_MANUFACTURER;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = SGM4154x_NAME;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = state.online;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = state.vbus_gd;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = sgm4154x_power_supply_desc.type;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
#if (defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__) || defined(__SGM41543D_CHIP_ID__)|| defined(__SGM41513D_CHIP_ID__))
		val->intval = get_charger_type(sgm);
#endif
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (state.chrg_fault & 0xF8)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;

		switch (state.health) {
		case SGM4154x_TEMP_HOT:
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
			break;
		case SGM4154x_TEMP_WARM:
			val->intval = POWER_SUPPLY_HEALTH_WARM;
			break;
		case SGM4154x_TEMP_COOL:
			val->intval = POWER_SUPPLY_HEALTH_COOL;
			break;
		case SGM4154x_TEMP_COLD:
			val->intval = POWER_SUPPLY_HEALTH_COLD;
			break;
		default:
			break;
		}
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		//val->intval = state.vbus_adc;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		//val->intval = state.ibus_adc;
		break;

	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = sgm4154x_get_input_volt_lim(sgm);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sgm4154x_get_input_curr_lim(sgm);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		ret = sgm4154x_is_enabled_charger(sgm);
		if(ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_SCOPE:
		ret = sgm4154x_is_enabled_vbus(sgm);
		if(ret < 0)
			return ret;
		val->intval = state.vbus_gd ? POWER_SUPPLY_SCOPE_DEVICE : 
					ret ? POWER_SUPPLY_SCOPE_SYSTEM : 
						POWER_SUPPLY_SCOPE_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = get_effective_result(sgm->fcc_votable);
		if(ret < 0)
			return ret;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = get_client_vote(sgm->fcc_votable, BATT_PROFILE_VOTER);
		if(ret < 0)
			return ret;
		val->intval = ret;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}


#if 0
static bool sgm4154x_state_changed(struct sgm4154x_device *sgm,
				  struct sgm4154x_state *new_state)
{
	struct sgm4154x_state old_state;

	mutex_lock(&sgm->lock);
	old_state = sgm->state;
	mutex_unlock(&sgm->lock);

	return (old_state.chrg_type != new_state->chrg_type ||
		old_state.chrg_stat != new_state->chrg_stat	||
		old_state.online != new_state->online		||
		old_state.therm_stat != new_state->therm_stat	||	
		old_state.vsys_stat != new_state->vsys_stat	||
		old_state.chrg_fault != new_state->chrg_fault
		);
}
#endif

static void sgm4154x_dump_register(struct sgm4154x_device * sgm)
{
	int i = 0;
	int reg = 0;

	for(i=0; i<=SGM4154x_CHRG_CTRL_f; i++) {
		regmap_read(sgm->regmap, i, &reg);
		pr_err("%s REG%x    %X\n", __func__, i, reg);
	}
}

#if 0
static bool sgm4154x_dpdm_detect_is_done(struct sgm4154x_device * sgm)
{
	int chrg_stat;
	int ret;

	ret = regmap_read(sgm->regmap, SGM4154x_INPUT_DET, &chrg_stat);
	if(ret) {
		dev_err(sgm->dev, "Check DPDM detecte error\n");
	}

	return (chrg_stat&SGM4154x_DPDM_ONGOING)?true:false;
}
#endif

static void sgm4154x_notify_device_mode(struct sgm4154x_device * sgm, bool enable){
	union extcon_property_value val;
	if(enable){
		val.intval = 0;
		extcon_set_property(sgm->extcon, EXTCON_USB,
				EXTCON_PROP_USB_TYPEC_POLARITY, val);
		val.intval = false;
		extcon_set_property(sgm->extcon, EXTCON_USB,
				EXTCON_PROP_USB_SS, val);
	}
	extcon_set_state_sync(sgm->extcon, EXTCON_USB, enable);
	return;
}
static void sgm4154x_notify_host_mode(struct sgm4154x_device * sgm, bool enable){
	union extcon_property_value val;
	val.intval = 0;
	if(enable){
		extcon_set_property(sgm->extcon, EXTCON_USB_HOST,
				EXTCON_PROP_USB_TYPEC_POLARITY, val);
		val.intval = false;
		extcon_set_property(sgm->extcon, EXTCON_USB_HOST,
				EXTCON_PROP_USB_SS, val);
	}
	extcon_set_state_sync(sgm->extcon, EXTCON_USB_HOST, enable);
	return;
}

static void charger_monitor_work_func(struct work_struct *work)
{
	int ret = 0;
	struct sgm4154x_device * sgm = NULL;
	struct delayed_work *charge_monitor_work = NULL;
	//static u8 last_chg_method = 0;
	struct sgm4154x_state state;
	union power_supply_propval pval = {0, };

	charge_monitor_work = container_of(work, struct delayed_work, work);
	if(charge_monitor_work == NULL) {
		pr_err("%s Cann't get charge_monitor_work\n", __func__);
		return ;
	}
	sgm = container_of(charge_monitor_work, struct sgm4154x_device, charge_monitor_work);
	if(sgm == NULL) {
		pr_err("%s Cann't get sgm \n", __func__);
		return ;
	}

	ret = sgm4154x_get_state(sgm, &state);
	mutex_lock(&sgm->lock);
	sgm->state = state;
	mutex_unlock(&sgm->lock);

	if (!sgm->state.chrg_type) {
		pr_err("%s not present vbus_status \n",__func__);
		goto OUT;
	}

	if(sgm->state.health == sgm->monitored_items.health){
		pr_err("%s health status not changed\n",__func__);
		goto OUT;
	}
	//health status changed
	switch(sgm->state.health){
		case SGM4154x_TEMP_COLD:
			sgm4154x_set_chrg_volt(sgm, sgm->data.jeita_temp_t0_to_t1_cv);
			vote(sgm->fcc_votable, JEITA_ARB_VOTER, true, sgm->data.jeita_temp_t0_to_t1_cc_current);
			pr_err("%s disable charger since switch to cold status\n",__func__);
			vote(sgm->chg_disable_votable, JEITA_ARB_VOTER, true, 0);
			break;
		case SGM4154x_TEMP_COOL:
			if(sgm->monitored_items.health == SGM4154x_TEMP_COLD){
				if(!sgm->battery){
					sgm->battery = power_supply_get_by_name(sgm4154x_charger_supplied_to[0]);
					if(!sgm->battery){
						dev_err(sgm->dev, "Could not get power_supply %s\n", sgm4154x_charger_supplied_to[0]);
						goto OUT;
					}
				}
				ret = power_supply_get_property(sgm->battery, POWER_SUPPLY_PROP_TEMP, &pval);
				if(ret < 0){
					pr_err("%s get battery temp fail\n",__func__);
					goto OUT;
				}
				pr_err("%s get battery temp: %d\n",__func__, pval.intval);
				if(pval.intval < sgm->data.temp_t1_thres_plus_x_degree * 10){
					pr_err("%s batt temp is below t1_thres_plus, maintain cold status\n",__func__);
					goto OUT;
				}else{
					pr_err("%s batt temp is above t1_thres_plus, recover cool status\n",__func__);
					sgm4154x_set_chrg_volt(sgm, sgm->data.jeita_temp_t1_to_t2_cv);
					vote(sgm->fcc_votable, JEITA_ARB_VOTER, true, sgm->data.jeita_temp_t1_to_t2_cc_current);
					pr_err("%s enable charger since switch from cold to cool status\n",__func__);
					vote(sgm->chg_disable_votable, JEITA_ARB_VOTER, false, 0);
				}
			}else{
				sgm4154x_set_chrg_volt(sgm, sgm->data.jeita_temp_t1_to_t2_cv);
				vote(sgm->fcc_votable, JEITA_ARB_VOTER, true, sgm->data.jeita_temp_t1_to_t2_cc_current);
				pr_err("%s enable charger since switch from other(charger disabled) to cool status\n",__func__);
				vote(sgm->chg_disable_votable, JEITA_ARB_VOTER, false, 0);
			}
			break;
		case SGM4154x_TEMP_NORMAL:
			if(sgm->monitored_items.health == SGM4154x_TEMP_COOL){
				if(!sgm->battery){
					sgm->battery = power_supply_get_by_name(sgm4154x_charger_supplied_to[0]);
					if(!sgm->battery){
						dev_err(sgm->dev, "Could not get power_supply %s\n", sgm4154x_charger_supplied_to[0]);
						goto OUT;
					}
				}
				ret = power_supply_get_property(sgm->battery, POWER_SUPPLY_PROP_TEMP, &pval);
				if(ret < 0){
					pr_err("%s get battery temp fail\n",__func__);
					goto OUT;
				}
				pr_err("%s get battery temp: %d\n",__func__, pval.intval);
				if(pval.intval < sgm->data.temp_t2_thres_plus_x_degree * 10){
					pr_err("%s batt temp is below t2_thres_plus, maintain cool status\n",__func__);
					goto OUT;
				}else{
					pr_err("%s batt temp is above t2_thres_plus, recover good status\n",__func__);
					sgm4154x_set_chrg_volt(sgm, sgm->data.jeita_temp_t2_to_t3_cv);
					vote(sgm->fcc_votable, JEITA_ARB_VOTER, true, sgm->data.jeita_temp_t2_to_t3_cc_current);
				}
			}else if(sgm->monitored_items.health == SGM4154x_TEMP_WARM){
				if(!sgm->battery){
					sgm->battery = power_supply_get_by_name(sgm4154x_charger_supplied_to[0]);
					if(!sgm->battery){
						dev_err(sgm->dev, "Could not get power_supply %s\n", sgm4154x_charger_supplied_to[0]);
						goto OUT;
					}
				}
				ret = power_supply_get_property(sgm->battery, POWER_SUPPLY_PROP_TEMP, &pval);
				if(ret < 0){
					pr_err("%s get battery temp fail\n",__func__);
					goto OUT;
				}
				pr_err("%s get battery temp: %d\n",__func__, pval.intval);
				if(pval.intval > sgm->data.temp_t3_thres_minus_x_degree * 10){
					pr_err("%s batt temp is above t3_thres_minus, maintain warm status\n",__func__);
					goto OUT;
				}else{
					pr_err("%s batt temp is below t3_thres_minus, recover good status\n",__func__);
					sgm4154x_set_chrg_volt(sgm, sgm->data.jeita_temp_t2_to_t3_cv);
					vote(sgm->fcc_votable, JEITA_ARB_VOTER, true, sgm->data.jeita_temp_t2_to_t3_cc_current);
				}
			}else{
				sgm4154x_set_chrg_volt(sgm, sgm->data.jeita_temp_t2_to_t3_cv);
				vote(sgm->fcc_votable, JEITA_ARB_VOTER, true, sgm->data.jeita_temp_t2_to_t3_cc_current);
				pr_err("%s enable charger since switch from other(charger disabled) to good status\n",__func__);
				vote(sgm->chg_disable_votable, JEITA_ARB_VOTER, false, 0);

			}
			break;
		case SGM4154x_TEMP_WARM:
			if(sgm->monitored_items.health == SGM4154x_TEMP_HOT){
				if(!sgm->battery){
					sgm->battery = power_supply_get_by_name(sgm4154x_charger_supplied_to[0]);
					if(!sgm->battery){
						dev_err(sgm->dev, "Could not get power_supply %s\n", sgm4154x_charger_supplied_to[0]);
						goto OUT;
					}
				}
				ret = power_supply_get_property(sgm->battery, POWER_SUPPLY_PROP_TEMP, &pval);
				if(ret < 0){
					pr_err("%s get battery temp fail\n",__func__);
					goto OUT;
				}
				pr_err("%s get battery temp: %d\n",__func__, pval.intval);
				if(pval.intval > sgm->data.temp_t4_thres_minus_x_degree * 10){
					pr_err("%s batt temp is above t4_thres_minus, maintain overheat status\n",__func__);
					goto OUT;
				}else{
					pr_err("%s batt temp is below t4_thres_minus, recover warm status\n",__func__);
					sgm4154x_set_chrg_volt(sgm, sgm->data.jeita_temp_t3_to_t4_cv);
					vote(sgm->fcc_votable, JEITA_ARB_VOTER, true, sgm->data.jeita_temp_t3_to_t4_cc_current);
					pr_err("%s enable charger since switch from overheat to warm status\n",__func__);
					vote(sgm->chg_disable_votable, JEITA_ARB_VOTER, false, 0);
				}
			}else{
				sgm4154x_set_chrg_volt(sgm, sgm->data.jeita_temp_t3_to_t4_cv);
				vote(sgm->fcc_votable, JEITA_ARB_VOTER, true, sgm->data.jeita_temp_t3_to_t4_cc_current);
				pr_err("%s enable charger since switch from other(charger disabled) to warm status\n",__func__);
				vote(sgm->chg_disable_votable, JEITA_ARB_VOTER, false, 0);

			}
			break;
		case SGM4154x_TEMP_HOT:
			sgm4154x_set_chrg_volt(sgm, sgm->data.jeita_temp_above_t4_cv);
			vote(sgm->fcc_votable, JEITA_ARB_VOTER, true, sgm->data.jeita_temp_above_t4_cc_current);
			pr_err("%s disable charger since switch to overheat status\n",__func__);
			vote(sgm->chg_disable_votable, JEITA_ARB_VOTER, true, 0);
			break;
		default:
			break;
	}
	sgm->monitored_items.health = sgm->state.health;		//store the previous health status
	sgm4154x_dump_register(sgm);
	pr_err("%s\n",__func__);
OUT:	
	schedule_delayed_work(&sgm->charge_monitor_work, msecs_to_jiffies(10000));
}

static char *maginput[2] = {"MAG_EVENT=MAG_IN", NULL };
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
static char *magoutput[2] = {"MAG_EVENT=MAG_OUT", NULL };
#endif/*CONFIG_DOUBLE_CHARGERS*/
static void charger_detect_work_func(struct work_struct *work)
{
	struct delayed_work *charge_detect_delayed_work = NULL;
	struct sgm4154x_device * sgm = NULL;
	//static int charge_type_old = 0;
	int curr_in_limit = 0;	
	struct sgm4154x_state state;	
	int ret;
	bool device_mode = false;
	union power_supply_propval pval = {0, };
	
	charge_detect_delayed_work = container_of(work, struct delayed_work, work);
	if(charge_detect_delayed_work == NULL) {
		pr_err("%s Cann't get charge_detect_delayed_work\n", __func__);
		return;
	}
	sgm = container_of(charge_detect_delayed_work, struct sgm4154x_device, charge_detect_delayed_work);
	if(sgm == NULL) {
		pr_err("%s Cann't get sgm4154x_device\n", __func__);
		return;
	}

	dev_err(sgm->dev, "work triggered by %s\n", (sgm->usb_event == USB_EVENT_VBUS) ? "vbus" : "irq");
	if(!sgm->battery){
		sgm->battery = power_supply_get_by_name("battery");
		if(!sgm->battery){
			dev_err(sgm->dev, "Could not find battery psy\n");
			return;
		}
	}

	ret = sgm4154x_get_state(sgm, &state);
	mutex_lock(&sgm->lock);
	sgm->state = state;	
	mutex_unlock(&sgm->lock);	
	
	/*if(!sgm4154x_dpdm_detect_is_done(sgm)) {
		dev_err(sgm->dev, "DPDM detecte not done, disable charge\n");
		goto err;
	}*/
#if (defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)|| defined(__SGM41543D_CHIP_ID__)|| defined(__SGM41513D_CHIP_ID__))
	switch(sgm->state.chrg_type) {
		case SGM4154x_USB_SDP:
			pr_err("%s SGM4154x charger type: SDP\n", __func__);
			sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_USB;
			curr_in_limit = 500000;
			device_mode = true;
			break;

		case SGM4154x_USB_CDP:
			pr_err("%s SGM4154x charger type: CDP\n", __func__);
			sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
			curr_in_limit = 1500000;
			device_mode = true;
			break;

		case SGM4154x_USB_DCP:
			pr_err("%s SGM4154x charger type: DCP can select up volt\n", __func__);
			sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
			curr_in_limit = 2000000;
			//sgm4154x_enable_qc20_hvdcp_9v(sgm);
			//sgm4154x_is_hvdcp(sgm,0);
			break;

		case SGM4154x_UNKNOWN:
			pr_err("%s SGM4154x charger type: UNKNOWN\n", __func__);
			sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_APPLE_BRICK_ID;
			curr_in_limit = 500000;
			break;
		case SGM4154x_NON_STANDARD:
			pr_err("%s SGM4154x charger type: NON_STANDARD\n", __func__);
			sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_APPLE_BRICK_ID;
			curr_in_limit = 1000000;
			break;

		case SGM4154x_OTG_MODE:
			pr_err("%s SGM4154x OTG mode do nothing\n", __func__);
			break;

		default:
			pr_err("%s SGM4154x charger type: default\n", __func__);
			sgm4154x_power_supply_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
			//curr_in_limit = 500000;
			break;
	}
	
	pr_err("%s psy.type:%d, Update: curr_in_limit = %d\n",__func__, sgm4154x_power_supply_desc.type, curr_in_limit);
	
	if(!sgm->usb){
		sgm->usb = power_supply_get_by_name("usb");
		if(!sgm->usb)
			dev_err(sgm->dev, "Could not find usb psy\n");
	}
	if(sgm->usb){
		ret = power_supply_get_property(sgm->usb, POWER_SUPPLY_PROP_PRESENT, &pval);
		if(ret < 0)
			pr_err("%s get usb present fail\n",__func__);
	}
	if(!pval.intval)
		usb_psy_desc.type = sgm4154x_power_supply_desc.type;

	if(!sgm->state.vbus_gd) {
		dev_err(sgm->dev, "Vbus not present, disable charge\n");
		sgm4154x_set_input_curr_lim(sgm, SGM4154x_IINDPM_DEF_uA);
		sgm4154x_typec_unregister_partner(sgm);
		goto err;
	}
	if(!state.online)
	{
		dev_err(sgm->dev, "Vbus not online\n");		
		goto err;
	}

	//set charge parameters
	sgm4154x_set_input_curr_lim(sgm, curr_in_limit);
#endif
	if(!sgm->use_second_usb){
		if(sgm_typec_charger_ok()){
			if(device_mode &&
				!(usb_psy_desc.type == POWER_SUPPLY_TYPE_USB || 
				usb_psy_desc.type == POWER_SUPPLY_TYPE_USB_CDP)){
				pval.intval = 0;
				dev_err(sgm->dev, "sgm device mode, typec not device mode, disable typec charger\n");
				power_supply_set_property(sgm->battery, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &pval);
			}else{
				power_supply_get_property(sgm->battery, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &pval);
				if(pval.intval == 1){
					dev_err(sgm->dev, "typec charger ok, disable sgm charger\n");
					vote(sgm->chg_disable_votable, BATT_PROFILE_VOTER, true, 0);
					if(device_mode)
						device_mode = false;
					goto err;
				}
			}
		}else if(sgm_typec_otg_ok()){
			if(device_mode)
				device_mode = false;
		}
	}
	//enable charge
	if(device_mode && sgm->usb_data_only_mode)
		vote(sgm->chg_disable_votable, OEM_VOTER, true, 0);
	else
		vote(sgm->chg_disable_votable, BATT_PROFILE_VOTER, false, 0);
	sgm4154x_typec_register_partner(sgm);
	if(device_mode){
		dev_err(sgm->dev, "Update: device_mode = %d\n", device_mode);
		if(sgm->use_second_usb)
			kobject_uevent_env(&sgm->dev->kobj, KOBJ_CHANGE, maginput);
		if(!sgm->device_mode){
			if(!sgm->use_second_usb)
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
				sgm_usb_switch_gpio(true);
#endif/*CONFIG_DOUBLE_CHARGERS*/
			sgm4154x_notify_device_mode(sgm, true);
			sgm->device_mode = device_mode;
		}else{
			if(sgm->usb_event == USB_EVENT_VBUS){
				sgm4154x_notify_device_mode(sgm, false);
				sgm4154x_notify_device_mode(sgm, true);
			}
		}
	}else{
		if(sgm->device_mode){
			sgm->device_mode = device_mode;
			sgm4154x_notify_device_mode(sgm, false);
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
			if(!sgm->use_second_usb){
				sgm_usb_switch_gpio(false);
				if(sgm_typec_otg_ok())
					ucsi_usb_role_switch();
			}
#endif/*CONFIG_DOUBLE_CHARGERS*/
		}
	}

	dev_err(sgm->dev, "Start Jeita temp monitoring\n");
	cancel_delayed_work_sync(&sgm->charge_monitor_work);
	sgm->monitored_items.health = SGM4154x_TEMP_NORMAL;
	schedule_delayed_work(&sgm->charge_monitor_work, 100);
	if(sgm->usb_event == USB_EVENT_VBUS)
		sgm->usb_event = USB_EVENT_NONE;
	sgm4154x_dump_register(sgm);
	power_supply_changed(sgm->charger);
	return;
err:
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
	//release wakelock
	if(sgm->device_mode){
		dev_err(sgm->dev, "Update: device_mode = %d\n", device_mode);
		sgm->device_mode = device_mode;
		sgm4154x_notify_device_mode(sgm, false);
		if(!sgm->use_second_usb){
			sgm_usb_switch_gpio(false);
			if(sgm_typec_charger_ok() && 
				(usb_psy_desc.type == POWER_SUPPLY_TYPE_USB || usb_psy_desc.type == POWER_SUPPLY_TYPE_USB_CDP))
				ucsi_usb_role_switch();
		}
		else
			kobject_uevent_env(&sgm->dev->kobj, KOBJ_CHANGE, magoutput);
	}
#endif /*CONFIG_DOUBLE_CHARGERS*/
	dev_err(sgm->dev, "Stop Jeita temp monitoring\n");
	cancel_delayed_work_sync(&sgm->charge_monitor_work);
	vote(sgm->fcc_votable, JEITA_ARB_VOTER, false, 0);
	vote(sgm->fcc_votable, BATT_PROFILE_VOTER, true, sgm->bat_info.constant_charge_current_max_ua);
	if(!sgm_typec_charger_ok()){
		vote(sgm->chg_disable_votable, BATT_PROFILE_VOTER, false, 0);
		vote(sgm->chg_disable_votable, USER_VOTER, false, 0);
		vote(sgm->chg_disable_votable, JEITA_ARB_VOTER, false, 0);
		vote(sgm->chg_disable_votable, FORCE_RECHARGE_VOTER, false, 0);
		vote(sgm->chg_disable_votable, HW_LIMIT_VOTER, false, 0);
	}
	if(sgm->usb_event == USB_EVENT_VBUS)
		sgm->usb_event = USB_EVENT_NONE;
	power_supply_changed(sgm->charger);
	dev_err(sgm->dev, "Relax wakelock\n");
	__pm_relax(sgm->charger_wakelock);
	return;
}

static void otg_detect_work_func(struct work_struct *work){
	struct delayed_work *otg_detect_delayed_work = NULL;
	struct sgm4154x_device * sgm = NULL;
	bool host_mode = false;
	
	otg_detect_delayed_work = container_of(work, struct delayed_work, work);
	if(otg_detect_delayed_work == NULL) {
		pr_err("%s Cann't get otg_detect_delayed_work\n", __func__);
		return;
	}
	sgm = container_of(otg_detect_delayed_work, struct sgm4154x_device, otg_detect_delayed_work);
	if(sgm == NULL) {
		pr_err("%s Cann't get sgm4154x_device\n", __func__);
		return;
	}
	
	if(sgm->usb_event == USB_EVENT_ID){
		pr_err("%s SGM4154x OTG mode\n", __func__);
		host_mode = true;
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
		if(!sgm->use_second_usb){
			if(sgm_typec_otg_ok())
				host_mode = false;
		}
#endif/*CONFIG_DOUBLE_CHARGERS*/
		if(host_mode){
			dev_err(sgm->dev, "Update: host_mode = %d\n", host_mode);
			if(!sgm4154x_is_enabled_vbus(sgm)){
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
				if(!sgm->use_second_usb) {
					sgm_usb_switch_gpio(true);
					dev_err(sgm->dev, "Update: sgm_usb_switch_gpio\n");
				}
#endif/*CONFIG_DOUBLE_CHARGERS*/
				/* otg cable inserted */
				if (sgm->typec_port) {
					sgm4154x_typec_register_partner(sgm);
					typec_set_data_role(sgm->typec_port,
								TYPEC_HOST);
					typec_set_pwr_role(sgm->typec_port,
								TYPEC_SOURCE);
				}
				sgm4154x_notify_host_mode(sgm, false);
				sgm4154x_notify_host_mode(sgm, true);
				sgm4154x_enable_vbus(sgm);
				dev_err(sgm->dev, "Update: enable host\n");
			}else{
				sgm4154x_notify_host_mode(sgm, false);
				sgm4154x_notify_host_mode(sgm, true);
			}
		}else{
			if(sgm4154x_is_enabled_vbus(sgm)){
				sgm4154x_disable_vbus(sgm);
				sgm4154x_notify_host_mode(sgm, false);
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
				if(!sgm->use_second_usb){					
					sgm_usb_switch_gpio(false);
					ucsi_usb_role_switch();
				}
#endif/*CONFIG_DOUBLE_CHARGERS*/
				if(sgm_typec_otg_ok()){
					dev_err(sgm->dev, "Update:typec enable host\n");
					sgm4154x_notify_host_mode(sgm, true);
				}
				if (sgm->typec_port && !sgm_typec_otg_ok()) {
					/* otg cable removed */
					typec_set_data_role(sgm->typec_port, TYPEC_DEVICE);
					typec_set_pwr_role(sgm->typec_port, TYPEC_SINK);
					sgm4154x_typec_unregister_partner(sgm);
				}
			}
		}
	}else if(sgm->usb_event == USB_EVENT_NONE){
		pr_err("%s SGM4154x non OTG mode\n", __func__);
		if(sgm4154x_is_enabled_vbus(sgm)){
			dev_err(sgm->dev, "Update: host_mode = %d\n", host_mode);
			sgm4154x_disable_vbus(sgm);
			sgm4154x_notify_host_mode(sgm, false);
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
			if(!sgm->use_second_usb){
				sgm_usb_switch_gpio(false);
				if(sgm_typec_otg_ok())
					ucsi_usb_role_switch();
			}
#endif/*CONFIG_DOUBLE_CHARGERS*/
			if (sgm->typec_port) {
				/* otg cable removed */
				typec_set_data_role(sgm->typec_port, TYPEC_DEVICE);
				typec_set_pwr_role(sgm->typec_port, TYPEC_SINK);
				sgm4154x_typec_unregister_partner(sgm);
			}
		}
	}
	return;
}

static void recharge_work_func(struct work_struct *work){
	struct delayed_work *recharge_delayed_work = NULL;
	struct sgm4154x_device * sgm = NULL;
	struct sgm4154x_state state;
	int capacity = 0;

	recharge_delayed_work = container_of(work, struct delayed_work, work);
	if(recharge_delayed_work == NULL) {
		pr_err("%s Cann't get recharge_delayed_work\n", __func__);
		return;
	}
	sgm = container_of(recharge_delayed_work, struct sgm4154x_device, recharge_delayed_work);
	if(sgm == NULL) {
		pr_err("%s Cann't get sgm4154x_device\n", __func__);
		return;
	}

	if(!sgm->monitored_items.charge_safety_timer_expired)
		return;
	capacity = sgm4154x_get_batt_capacity(sgm);
	if(capacity < 0)
		return;
	if(capacity < 80){
		dev_err(sgm->dev, "enable recharging caused by charge safety timer expired\n");
		vote(g_sgm->chg_disable_votable, FORCE_RECHARGE_VOTER, true, 0);
		vote(g_sgm->chg_disable_votable, FORCE_RECHARGE_VOTER, false, 0);
		sgm4154x_get_state(sgm, &state);
		mutex_lock(&sgm->lock);
		sgm->state = state;	
		mutex_unlock(&sgm->lock);
		sgm->monitored_items.charge_safety_timer_expired = false;
		power_supply_changed(sgm->charger);
	}else
		schedule_delayed_work(&sgm->recharge_delayed_work, msecs_to_jiffies(10000));
	dev_err(sgm->dev, "charge_safety_timer_expired: %d\n", sgm->monitored_items.charge_safety_timer_expired);
	return;
}

static void init_typec_class_work_func(struct work_struct *work){
	struct delayed_work *init_typec_class_delay_work = NULL;
	struct sgm4154x_device * sgm = NULL;
	int ret = 0;

	init_typec_class_delay_work = container_of(work, struct delayed_work, work);
	if(init_typec_class_delay_work == NULL) {
		pr_err("%s Cann't get init_typec_class_delay_work\n", __func__);
		return;
	}
	sgm = container_of(init_typec_class_delay_work, struct sgm4154x_device, init_typec_class_delay_work);
	if(sgm == NULL) {
		pr_err("%s Cann't get sgm4154x_device\n", __func__);
		return;
	}

	ret = sgm4154x_init_typec_class(sgm);
	if(ret)
		return;

	dev_err(sgm->dev, "%s \n", __func__);
}
static irqreturn_t sgm4154x_irq_handler_thread(int irq, void *private)
{
	struct sgm4154x_device *sgm = private;

	//lock wakelock
	if (!sgm->charger_wakelock->active)
		__pm_stay_awake(sgm->charger_wakelock);
	cancel_delayed_work_sync(&sgm->charge_detect_delayed_work);
	schedule_delayed_work(&sgm->charge_detect_delayed_work, msecs_to_jiffies(300));
	//power_supply_changed(sgm->charger);
	
	return IRQ_HANDLED;
}

static enum power_supply_property sgm4154x_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX
};

static struct power_supply_desc sgm4154x_power_supply_desc = {
	.name = "sgm_charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = sgm4154x_usb_type,
	.num_usb_types = ARRAY_SIZE(sgm4154x_usb_type),
	.properties = sgm4154x_power_supply_props,
	.num_properties = ARRAY_SIZE(sgm4154x_power_supply_props),
	.get_property = sgm4154x_charger_get_property,
	.set_property = sgm4154x_charger_set_property,
	.property_is_writeable = sgm4154x_property_is_writeable,
};


static bool sgm4154x_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {	
	case SGM4154x_CHRG_CTRL_0...SGM4154x_CHRG_CTRL_f:	
		return true;
	default:
		return false;
	}
}

static const struct regmap_config sgm4154x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = SGM4154x_CHRG_CTRL_f,
	.reg_defaults	= sgm4154x_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(sgm4154x_reg_defs),
	.cache_type = REGCACHE_RBTREE,
	
	.volatile_reg = sgm4154x_is_volatile_reg,
};

static int sgm4154x_power_supply_init(struct sgm4154x_device *sgm,
							struct device *dev)
{
	struct power_supply_config psy_cfg = { .drv_data = sgm,
						.of_node = dev->of_node, };

	psy_cfg.supplied_to = sgm4154x_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(sgm4154x_charger_supplied_to);

	sgm->charger = devm_power_supply_register(sgm->dev,
						 &sgm4154x_power_supply_desc,
						 &psy_cfg);
	if (IS_ERR(sgm->charger))
		return -EINVAL;

	return 0;
}

static int sgm4154x_hw_init(struct sgm4154x_device *sgm)
{
	int ret = 0;	

	ret = sgm4154x_set_watchdog_timer(sgm, 0);
	if(ret)
		goto err_out;
	sgm->init_data.max_ichg =
			SGM4154x_ICHRG_I_MAX_uA;

	sgm->init_data.max_vreg =
			SGM4154x_VREG_V_MAX_uV;

	ret = vote(sgm->fcc_votable, HW_LIMIT_VOTER, sgm->bat_info.constant_charge_current_max_ua > 0, sgm->bat_info.constant_charge_current_max_ua);
	ret |= vote(sgm->fcc_votable, BATT_PROFILE_VOTER, sgm->bat_info.constant_charge_current_max_ua > 0, sgm->bat_info.constant_charge_current_max_ua);
	ret |= vote(sgm->chg_disable_votable, HW_LIMIT_VOTER, false, 0);
	if (ret)
		goto err_out;

	if(sgm->dev_rev) {
		sgm4154x_set_otgf_itremr(sgm, 0);
	}

	ret = sgm4154x_set_prechrg_curr(sgm, sgm->bat_info.precharge_current_ua);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_chrg_volt(sgm,
				sgm->bat_info.constant_charge_voltage_max_uv);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_term_curr(sgm, sgm->bat_info.charge_term_current_ua);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_input_volt_lim(sgm, sgm->init_data.vlim);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_input_curr_lim(sgm, SGM4154x_IINDPM_DEF_uA);
	if (ret)
		goto err_out;

	ret = sgm4154x_set_vac_ovp(sgm, sgm->init_data.ovp_thd);//14V
	if (ret)
		goto err_out;	

	ret = sgm4154x_set_recharge_volt(sgm, sgm->init_data.rechg_thd);//100~200mv
	if (ret)
		goto err_out;

	ret = sgm4154x_set_vdpm_track(sgm, 250); //set vdpm track thd 250mV
	if (ret)
		goto err_out;

	dev_notice(sgm->dev, "ichrg_curr:%d prechrg_curr:%d chrg_vol:%d"
		" term_curr:%d input_curr_lim:%d input_volt_lim:%d",
		sgm->bat_info.constant_charge_current_max_ua,
		sgm->bat_info.precharge_current_ua,
		sgm->bat_info.constant_charge_voltage_max_uv,
		sgm->bat_info.charge_term_current_ua,
		sgm->init_data.ilim, sgm->init_data.vlim);

	return 0;

err_out:
	return ret;
}

static int sgm4154x_parse_dt(struct sgm4154x_device *sgm)
{
	int ret;
	u32 val = 0;
	int irq_gpio = 0, irqn = 0;
	int chg_en_gpio = 0;
	#if 0
	ret = device_property_read_u32(sgm->dev, "watchdog-timer",
				       &sgm->watchdog_timer);
	if (ret)
		sgm->watchdog_timer = SGM4154x_WATCHDOG_DIS;

	if (sgm->watchdog_timer > SGM4154x_WATCHDOG_MAX ||
	    sgm->watchdog_timer < SGM4154x_WATCHDOG_DIS)
		return -EINVAL;
	#endif
	ret = device_property_read_u32(sgm->dev,
				       "input-voltage-limit-microvolt",
				       &sgm->init_data.vlim);
	if (ret)
		sgm->init_data.vlim = SGM4154x_VINDPM_DEF_uV;

	if (sgm->init_data.vlim > SGM4154x_VINDPM_V_MAX_uV ||
	    sgm->init_data.vlim < SGM4154x_VINDPM_V_MIN_uV)
		return -EINVAL;

	ret = device_property_read_u32(sgm->dev,
				       "input-current-limit-microamp",
				       &sgm->init_data.ilim);
	if (ret)
		sgm->init_data.ilim = SGM4154x_IINDPM_DEF_uA;

	if (sgm->init_data.ilim > SGM4154x_IINDPM_I_MAX_uA ||
	    sgm->init_data.ilim < SGM4154x_IINDPM_I_MIN_uA)
		return -EINVAL;

	ret = device_property_read_u32(sgm->dev,
				       "constant-charge-current-max-ua",
				       &sgm->bat_info.constant_charge_current_max_ua);

	if(ret)
		sgm->bat_info.constant_charge_current_max_ua =
			SGM4154x_ICHRG_I_DEF_uA;

	if(sgm->bat_info.constant_charge_current_max_ua > SGM4154x_ICHRG_I_MAX_uA || 
		sgm->bat_info.constant_charge_current_max_ua < SGM4154x_ICHRG_I_MIN_uA)
		return -EINVAL;

	ret = device_property_read_u32(sgm->dev,
				       "constant-charge-voltage-max-uv",
				       &sgm->bat_info.constant_charge_voltage_max_uv);

	if(ret)
		sgm->bat_info.constant_charge_voltage_max_uv =
				SGM4154x_VREG_V_DEF_uV;

	if(sgm->bat_info.constant_charge_voltage_max_uv > SGM4154x_VREG_V_MAX_uV || 
		sgm->bat_info.constant_charge_voltage_max_uv < SGM4154x_VREG_V_MIN_uV)
		return -EINVAL;

	ret = device_property_read_u32(sgm->dev,
				       "precharge-current-ua",
				       &sgm->bat_info.precharge_current_ua);

	if(ret) {
		if(sgm->dev_rev)
			sgm->bat_info.precharge_current_ua =
					SGM4155x_PRECHRG_I_DEF_uA;
		else
			sgm->bat_info.precharge_current_ua =
					SGM4154x_PRECHRG_I_DEF_uA;
	}

	ret = device_property_read_u32(sgm->dev,
				       "charge-term-current-ua",
				       &sgm->bat_info.charge_term_current_ua);

	if(ret) {
		if(sgm->dev_rev)
			sgm->bat_info.charge_term_current_ua =
					SGM4155x_TERMCHRG_I_DEF_uA;
		else
			sgm->bat_info.charge_term_current_ua =
					SGM4154x_TERMCHRG_I_DEF_uA;
	}

	ret = device_property_read_u32(sgm->dev,
				       "ovp-threshold-uv",
				       &sgm->init_data.ovp_thd);
	if(ret)
		sgm->init_data.ovp_thd = OVP_THD[3];

	ret = device_property_read_u32(sgm->dev,
				       "recharge-threshold-mv",
				       &sgm->init_data.rechg_thd);

	if(ret)
		sgm->init_data.rechg_thd = SGM4154x_VRECHRG_DEFAULT_mV;

	ret = device_property_read_u32(sgm->dev,
				       "boost-voltage-limit-uv",
				       &sgm->init_data.vboost);
	if(ret)
		sgm->init_data.vboost = BOOST_VOLT_LIMIT[1];
	
	ret = device_property_read_u32(sgm->dev,
				       "boost-current-limit-ua",
				       &sgm->init_data.iboost);
	if(ret)
		sgm->init_data.iboost = BOOST_CURRENT_LIMIT[0];

	ret = device_property_read_u32(sgm->dev,
				       "use-second-usb",
				       &sgm->use_second_usb);
	if(ret)
		sgm->use_second_usb = 0;
	
	irq_gpio = of_get_named_gpio(sgm->dev->of_node, "sgm,int-gpio", 0);
	if (!gpio_is_valid(irq_gpio))
	{
		dev_err(sgm->dev, "%s: %d gpio get failed\n", __func__, irq_gpio);
		return -EINVAL;
	}
	sgm->subchg_int = irq_gpio;
	ret = gpio_request(irq_gpio, "sgm4154x irq pin");
	if (ret) {
		dev_err(sgm->dev, "%s: %d gpio request failed\n", __func__, irq_gpio);
		return ret;
	}
	//gpio_direction_input(irq_gpio);
	irqn = gpio_to_irq(irq_gpio);
	if (irqn < 0) {
		dev_err(sgm->dev, "%s:%d gpio_to_irq failed\n", __func__, irqn);
		return irqn;
	}
	sgm->client->irq = irqn;
	
	chg_en_gpio = of_get_named_gpio(sgm->dev->of_node, "sgm,chg-en-gpio", 0);
	if (!gpio_is_valid(chg_en_gpio))
	{
		dev_err(sgm->dev, "%s: %d gpio get failed\n", __func__, chg_en_gpio);
		return -EINVAL;
	}
	sgm->chg_en = chg_en_gpio;
	ret = gpio_request(chg_en_gpio, "sgm chg en pin");
	if (ret) {
		dev_err(sgm->dev, "%s: %d gpio request failed\n", __func__, chg_en_gpio);
		return ret;
	}
	//gpio_direction_output(chg_en_gpio,0);//default enable charge
	/* sw jeita */
	sgm->enable_sw_jeita = of_property_read_bool(sgm->dev->of_node, "enable_sw_jeita");

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_above_t4_cv", &val) >= 0)
		sgm->data.jeita_temp_above_t4_cv = val;
	else {
		dev_err(sgm->dev, "use default JEITA_TEMP_ABOVE_T4_CV:%d\n",JEITA_TEMP_ABOVE_T4_CV);
		sgm->data.jeita_temp_above_t4_cv = JEITA_TEMP_ABOVE_T4_CV;
	}

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_t3_to_t4_cv", &val) >= 0)
		sgm->data.jeita_temp_t3_to_t4_cv = val;
	else {
		dev_err(sgm->dev, "use default JEITA_TEMP_T3_TO_T4_CV:%d\n",JEITA_TEMP_T3_TO_T4_CV);
		sgm->data.jeita_temp_t3_to_t4_cv = JEITA_TEMP_T3_TO_T4_CV;
	}

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_t2_to_t3_cv", &val) >= 0)
		sgm->data.jeita_temp_t2_to_t3_cv = val;
	else {
		dev_err(sgm->dev, "use default JEITA_TEMP_T2_TO_T3_CV:%d\n",JEITA_TEMP_T2_TO_T3_CV);
		sgm->data.jeita_temp_t2_to_t3_cv = JEITA_TEMP_T2_TO_T3_CV;
	}

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_t1_to_t2_cv", &val) >= 0)
		sgm->data.jeita_temp_t1_to_t2_cv = val;
	else {
		dev_err(sgm->dev, "use default JEITA_TEMP_T1_TO_T2_CV:%d\n",JEITA_TEMP_T1_TO_T2_CV);
		sgm->data.jeita_temp_t1_to_t2_cv = JEITA_TEMP_T1_TO_T2_CV;
	}

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_t0_to_t1_cv", &val) >= 0)
		sgm->data.jeita_temp_t0_to_t1_cv = val;
	else {
		dev_err(sgm->dev, "use default JEITA_TEMP_T0_TO_T1_CV:%d\n",JEITA_TEMP_T0_TO_T1_CV);
		sgm->data.jeita_temp_t0_to_t1_cv = JEITA_TEMP_T0_TO_T1_CV;
	}

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_below_t0_cv", &val) >= 0)
		sgm->data.jeita_temp_below_t0_cv = val;
	else {
		dev_err(sgm->dev, "use default JEITA_TEMP_BELOW_T0_CV:%d\n",JEITA_TEMP_BELOW_T0_CV);
		sgm->data.jeita_temp_below_t0_cv = JEITA_TEMP_BELOW_T0_CV;
	}
	pr_err("%s,enable_sw_jeita = %d,CV1 = %d,CV2 = %d,CV3 = %d,CV4 = %d,CV5 = %d,CV6 = %d\n",__func__,
			sgm->enable_sw_jeita,sgm->data.jeita_temp_above_t4_cv,sgm->data.jeita_temp_t3_to_t4_cv,
			sgm->data.jeita_temp_t2_to_t3_cv,sgm->data.jeita_temp_t1_to_t2_cv,
			sgm->data.jeita_temp_t0_to_t1_cv,sgm->data.jeita_temp_below_t0_cv);

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_above_t4_cc_current", &val) >= 0)
		sgm->data.jeita_temp_above_t4_cc_current = val;

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_t3_to_t4_cc_current", &val) >= 0)
		sgm->data.jeita_temp_t3_to_t4_cc_current = val;
	else
		sgm->data.jeita_temp_t3_to_t4_cc_current = JEITA_TEMP_T3_TO_T4_CC_CURRENT;

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_t2_to_t3_cc_current", &val) >= 0)
		sgm->data.jeita_temp_t2_to_t3_cc_current = val;
	else
		sgm->data.jeita_temp_t2_to_t3_cc_current = JEITA_TEMP_T2_TO_T3_CC_CURRENT;

	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_t1_to_t2_cc_current", &val) >= 0)
		sgm->data.jeita_temp_t1_to_t2_cc_current = val;
	else
		sgm->data.jeita_temp_t1_to_t2_cc_current = JEITA_TEMP_T1_TO_T2_CC_CURRENT;

	if(of_property_read_u32(sgm->dev->of_node, "jeita_temp_t0_to_t1_cc_current", &val) >= 0)
		sgm->data.jeita_temp_t0_to_t1_cc_current = val;
	else
		sgm->data.jeita_temp_t0_to_t1_cc_current = JEITA_TEMP_T0_TO_T1_CC_CURRENT;
	
	if (of_property_read_u32(sgm->dev->of_node, "jeita_temp_below_t0_cc_current", &val) >= 0)
		sgm->data.jeita_temp_below_t0_cc_current = val;
	else
		sgm->data.jeita_temp_below_t0_cc_current = JEITA_TEMP_BELOW_T0_CC_CURRENT;

	pr_err("%s,CC1 = %d,CC2 = %d,CC3 = %d,CC4 = %d,CC5 = %d,CC6 = %d\n",__func__,
			sgm->data.jeita_temp_above_t4_cc_current,sgm->data.jeita_temp_t3_to_t4_cc_current,
			sgm->data.jeita_temp_t2_to_t3_cc_current,sgm->data.jeita_temp_t1_to_t2_cc_current,
			sgm->data.jeita_temp_t0_to_t1_cc_current,sgm->data.jeita_temp_below_t0_cc_current);

	if (of_property_read_u32(sgm->dev->of_node, "temp_t4_thres", &val) >= 0)
		sgm->data.temp_t4_thres = val;
	else {
		dev_err(sgm->dev, "use default TEMP_T4_THRES:%d\n",TEMP_T4_THRES);
		sgm->data.temp_t4_thres = TEMP_T4_THRES;
	}
	if (of_property_read_u32(sgm->dev->of_node, "temp_t4_thres_minus_x_degree", &val) >= 0)
		sgm->data.temp_t4_thres_minus_x_degree = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T4_THRES_MINUS_X_DEGREE:%d\n",TEMP_T4_THRES_MINUS_X_DEGREE);
		sgm->data.temp_t4_thres_minus_x_degree = TEMP_T4_THRES_MINUS_X_DEGREE;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_t3_thres", &val) >= 0)
		sgm->data.temp_t3_thres = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T3_THRES:%d\n",TEMP_T3_THRES);
		sgm->data.temp_t3_thres = TEMP_T3_THRES;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_t3_thres_minus_x_degree", &val) >= 0)
		sgm->data.temp_t3_thres_minus_x_degree = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T3_THRES_MINUS_X_DEGREE:%d\n",TEMP_T3_THRES_MINUS_X_DEGREE);
		sgm->data.temp_t3_thres_minus_x_degree = TEMP_T3_THRES_MINUS_X_DEGREE;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_t2_thres", &val) >= 0)
		sgm->data.temp_t2_thres = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T2_THRES:%d\n",TEMP_T2_THRES);
		sgm->data.temp_t2_thres = TEMP_T2_THRES;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_t2_thres_plus_x_degree", &val) >= 0)
		sgm->data.temp_t2_thres_plus_x_degree = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T2_THRES_PLUS_X_DEGREE:%d\n",TEMP_T2_THRES_PLUS_X_DEGREE);
		sgm->data.temp_t2_thres_plus_x_degree = TEMP_T2_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_t1_thres", &val) >= 0)
		sgm->data.temp_t1_thres = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T1_THRES:%d\n",TEMP_T1_THRES);
		sgm->data.temp_t1_thres = TEMP_T1_THRES;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_t1_thres_plus_x_degree", &val) >= 0)
		sgm->data.temp_t1_thres_plus_x_degree = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T1_THRES_PLUS_X_DEGREE:%d\n",TEMP_T1_THRES_PLUS_X_DEGREE);
		sgm->data.temp_t1_thres_plus_x_degree = TEMP_T1_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_t0_thres", &val) >= 0)
		sgm->data.temp_t0_thres = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T0_THRES:%d\n",TEMP_T0_THRES);
		sgm->data.temp_t0_thres = TEMP_T0_THRES;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_t0_thres_plus_x_degree", &val) >= 0)
		sgm->data.temp_t0_thres_plus_x_degree = val;
	else {
		dev_err(sgm->dev,"use default TEMP_T0_THRES_PLUS_X_DEGREE:%d\n",TEMP_T0_THRES_PLUS_X_DEGREE);
		sgm->data.temp_t0_thres_plus_x_degree = TEMP_T0_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(sgm->dev->of_node, "temp_neg_10_thres", &val) >= 0)
		sgm->data.temp_neg_10_thres = val;
	else {
		dev_err(sgm->dev,"use default TEMP_NEG_10_THRES:%d\n",TEMP_NEG_10_THRES);
		sgm->data.temp_neg_10_thres = TEMP_NEG_10_THRES;
	}
	pr_err("%s,thres4 = %d,thres3 = %d,thres2 = %d,thres1 = %d,thres0 = %d\n",__func__,
			sgm->data.temp_t4_thres,sgm->data.temp_t3_thres,
			sgm->data.temp_t2_thres,sgm->data.temp_t1_thres,
			sgm->data.temp_t0_thres);
	return 0;
}

static int sgm4154x_set_otg_voltage(struct sgm4154x_device *sgm, int uv)
{
	int ret = 0;
	int reg_val = -1;
	int i = 0;
	while(i<4){
		if (uv == BOOST_VOLT_LIMIT[i]){
			reg_val = i;
			break;
		}
		i++;
	}
	if (reg_val < 0)
		return reg_val;
	reg_val = reg_val << 4;
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_6,
				  SGM4154x_BOOSTV, reg_val);

	return ret;
}

static int sgm4154x_set_otg_current(struct sgm4154x_device *sgm, int ua)
{
	int ret = 0;	
	
	if (ua == BOOST_CURRENT_LIMIT[0]){
		ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_2, SGM4154x_BOOST_LIM,
                     0); 
	}
		
	else if (ua == BOOST_CURRENT_LIMIT[1]){
		ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_2, SGM4154x_BOOST_LIM,
                     BIT(7)); 
	}
	return ret;
}

static int sgm4154x_get_dev_rev(struct sgm4154x_device *sgm)
{
	int ret = 0;
	int val;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_b, &val);
	if (ret)
		return ret;

	sgm->dev_rev= val & SGM4154x_DEV_REV_MASK;
	dev_err(sgm->dev,"sgm->dev_rev %d\n",sgm->dev_rev);
	return 0;
}

#if 0
static int sgm4154x_enable_vbus(struct regulator_dev *rdev)
{
	struct sgm4154x_device *sgm = rdev_get_drvdata(rdev);	
	int ret = 0;
	
	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_1, SGM4154x_OTG_EN,
                     SGM4154x_OTG_EN);
	return ret;
}

static int sgm4154x_disable_vbus(struct regulator_dev *rdev)
{
	struct sgm4154x_device *sgm = rdev_get_drvdata(rdev);	
	int ret = 0;

	ret = regmap_update_bits(sgm->regmap, SGM4154x_CHRG_CTRL_1, SGM4154x_OTG_EN,
                     0);

	return ret;
}

static int sgm4154x_is_enabled_vbus(struct regulator_dev *rdev)
{
	struct sgm4154x_device *sgm = rdev_get_drvdata(rdev);
	int temp = 0;
	int ret = 0;

	ret = regmap_read(sgm->regmap, SGM4154x_CHRG_CTRL_1, &temp);
	return (temp&SGM4154x_OTG_EN)? 1 : 0;
}

static struct regulator_ops sgm4154x_vbus_ops = {
	.enable = sgm4154x_enable_vbus,
	.disable = sgm4154x_disable_vbus,
	.is_enabled = sgm4154x_is_enabled_vbus,
};

static struct regulator_desc sgm4154x_otg_rdesc = {
	.of_match = "usb-otg-vbus",
	.name = "usb-otg-vbus",
	.ops = &sgm4154x_vbus_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

static int sgm4154x_vbus_regulator_register(struct sgm4154x_device *sgm)
{
	struct regulator_config config = {};
	int ret = 0;
	/* otg regulator */
	config.dev = sgm->dev;
	config.driver_data = sgm;
	sgm->otg_rdev = devm_regulator_register(sgm->dev,
						&sgm4154x_otg_rdesc, &config);
	sgm->otg_rdev->constraints->valid_ops_mask |= REGULATOR_CHANGE_STATUS;
	if (IS_ERR(sgm->otg_rdev)) {
		ret = PTR_ERR(sgm->otg_rdev);
		pr_info("%s: register otg regulator failed (%d)\n", __func__, ret);
	}
	return ret;
}
#endif

static int sgm4154x_suspend_notifier(struct notifier_block *nb,
                unsigned long event,
                void *dummy)
{
    struct sgm4154x_device *sgm = container_of(nb, struct sgm4154x_device, pm_nb);

    switch (event) {

    case PM_SUSPEND_PREPARE:
        pr_err("%s sgm4154x PM_SUSPEND \n", __func__);

        //cancel_delayed_work_sync(&sgm->charge_monitor_work);

        sgm->sgm4154x_suspend_flag = 1;

        return NOTIFY_OK;

    case PM_POST_SUSPEND:
        pr_err("%s sgm4154x PM_RESUME \n", __func__);

        //schedule_delayed_work(&sgm->charge_monitor_work, 0);

        sgm->sgm4154x_suspend_flag = 0;

        return NOTIFY_OK;

    default:
        return NOTIFY_DONE;
    }
}

static int sgm4154x_hw_chipid_detect(struct sgm4154x_device *sgm)
{
	int ret = 0;
	int val = 0;
	
	ret = regmap_read(sgm->regmap,SGM4154x_CHRG_CTRL_b,&val);
	if (ret < 0)
	{
		pr_info("[%s] read SGM4154x_CHRG_CTRL_b fail\n", __func__);
		return ret;
	}		

	pr_info("[%s] Reg[0x0B]=0x%x\n", __func__,val);
	
	return val;
}

static void sgm4154x_usb_work(struct work_struct *data)
{
	struct sgm4154x_device *sgm =
			container_of(data, struct sgm4154x_device, usb_work);
	
	pr_err("%s sgm usb event: %d\n",__func__, sgm->usb_event);
	switch (sgm->usb_event) {
	case USB_EVENT_NONE:
		schedule_delayed_work(&sgm->otg_detect_delayed_work, 0);
		break;
	case USB_EVENT_ID:
		schedule_delayed_work(&sgm->otg_detect_delayed_work, msecs_to_jiffies(100));
		break;
	case USB_EVENT_VBUS:
		schedule_delayed_work(&sgm->charge_detect_delayed_work, 0);
		break;
	default:
		break;
	}

	return;

	dev_err(sgm->dev, "Error switching to charger mode.\n");
}

static const unsigned int usb_phy_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};
static int sgm4154x_usb_extcon_register(struct sgm4154x_device *sgm)
{
	int ret;

	/* Register extcon to notify USB driver */
	sgm->extcon = devm_extcon_dev_allocate(sgm->dev, usb_phy_extcon_cable);
	if (IS_ERR(sgm->extcon)) {
		dev_err(sgm->dev, "failed to allocate extcon device\n");
		return PTR_ERR(sgm->extcon);
	}

	ret = devm_extcon_dev_register(sgm->dev, sgm->extcon);
	if (ret) {
		dev_err(sgm->dev, "failed to register extcon device\n");
		return ret;
	}

	if (ret < 0)
		pr_err("%s failed to configure extcon capabilities ret=%d\n", __func__, ret);
	return 0;
}

int sgm_charger_set_usb_data_only_mode(bool enable){

	if(!g_sgm){
		pr_err("no sgm_charger device\n");
		return -ENODEV;
	}
	if(g_sgm->device_mode){
		vote(g_sgm->chg_disable_votable, OEM_VOTER, enable ? true : false, 0);
		if(g_sgm->charger)
			power_supply_changed(g_sgm->charger);
	}
	pr_err("%s enable %d\n",__func__,enable);
	g_sgm->usb_data_only_mode = enable;
	return 0;
}
EXPORT_SYMBOL_GPL(sgm_charger_set_usb_data_only_mode);

int sgm_charger_get_usb_data_only_mode(bool *enable){
	if(!g_sgm){
		pr_err("no sgm_charger device\n");
		return -ENODEV;
	}
	*enable = g_sgm->usb_data_only_mode;
	return 0;
}
EXPORT_SYMBOL_GPL(sgm_charger_get_usb_data_only_mode);

#if IS_ENABLED(CONFIG_QTI_BATTERY_CHARGER)
extern ssize_t get_typec_usb_data_only_mode(bool *enable);
extern ssize_t set_typec_usb_data_only_mode(bool enable);
#endif/*CONFIG_QTI_BATTERY_CHARGER*/

static ssize_t usb_data_only_mode_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf){
	bool usb_data_only_mode = false;
#if IS_ENABLED(CONFIG_QTI_BATTERY_CHARGER)
	int rc;
	bool typec_charger_usb_data_only_mode = false;
#endif /*CONFIG_QTI_BATTERY_CHARGER*/
	if(!g_sgm){
		pr_err("no sgm_charger device\n");
		return -ENODEV;
	}
#if IS_ENABLED(CONFIG_QTI_BATTERY_CHARGER)
	rc = get_typec_usb_data_only_mode(&typec_charger_usb_data_only_mode);
	if(rc < 0)
		return rc;
	usb_data_only_mode =  usb_data_only_mode || g_sgm->usb_data_only_mode;
#endif /*CONFIG_QTI_BATTERY_CHARGER*/
	pr_info("%s: %d\n",__func__, usb_data_only_mode);
	return snprintf(buf, PAGE_SIZE, "%d\n", usb_data_only_mode);
}
static ssize_t usb_data_only_mode_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int rc;
	bool enable = false;
	unsigned long mode = 0;

	rc = kstrtoul(buf, 10, &mode);
	if(rc){
		pr_buf_err("kstrtoul operation failed  rc=%d\n", rc);
		return rc;
	}

	enable = !!mode;
	pr_info("%s: %d\n", __func__,enable);
#if IS_ENABLED(CONFIG_QTI_BATTERY_CHARGER)
	rc = set_typec_usb_data_only_mode(enable);
	if(rc < 0)
		return rc;
#endif/*CONFIG_QTI_BATTERY_CHARGER*/
	sgm_charger_set_usb_data_only_mode(enable);
	rc = count;
	return count;
}
static struct kobj_attribute usb_data_only_mode_attr = __ATTR_RW(usb_data_only_mode);
static struct attribute *sgm_sys_node_attrs[] = {
	&usb_data_only_mode_attr.attr,
	NULL,
};
static struct attribute_group sgm_sys_node_attr_group = {
	.attrs = sgm_sys_node_attrs,
};
static int sgm4154x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct sgm4154x_device *sgm;
	int ret;
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
	int otg_notify;
#endif/*CONFIG_DOUBLE_CHARGERS*/
	char *name = NULL;

	sgm = devm_kzalloc(dev, sizeof(*sgm), GFP_KERNEL);
	if (!sgm)
		return -ENOMEM;

	sgm->client = client;
	sgm->dev = dev;

	mutex_init(&sgm->lock);

	strncpy(sgm->model_name, id->name, I2C_NAME_SIZE);

	sgm->regmap = devm_regmap_init_i2c(client, &sgm4154x_regmap_config);
	if (IS_ERR(sgm->regmap)) {
		dev_err(dev, "Failed to allocate register map\n");
		return PTR_ERR(sgm->regmap);
	}

	i2c_set_clientdata(client, sgm);

	ret = sgm4154x_hw_chipid_detect(sgm);
	if ((ret & SGM4154x_PN_MASK) != SGM4154x_PN_ID){
		pr_info("[%s] device not found !!!\n", __func__);
		return ret;
	}

	ret = sgm4154x_get_dev_rev(sgm);
	if(ret) {
		dev_err(dev, "Failed to read device dev rev %d\n", ret);
	}

	// Customer customization
	ret = sgm4154x_parse_dt(sgm);
	if (ret) {
		dev_err(dev, "Failed to read device tree properties%d\n", ret);
		return ret;
	}

	sgm->monitored_items.health = SGM4154x_TEMP_NORMAL;
	sgm->monitored_items.charge_safety_timer_expired = false;
	name = devm_kasprintf(sgm->dev, GFP_KERNEL, "%s",
		"sgm4154x suspend wakelock");
	sgm->charger_wakelock =
		wakeup_source_register(NULL, name);

	/* OTG reporting */
	/*sgm->usb2_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
	if (!IS_ERR_OR_NULL(sgm->usb2_phy)) {
		INIT_WORK(&sgm->usb_work, sgm4154x_usb_work);
		sgm->usb_nb.notifier_call = sgm4154x_usb_notifier;
		otg_notify = usb_register_notifier(sgm->usb2_phy, &sgm->usb_nb);
	}

	sgm->usb3_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB3);
	if (!IS_ERR_OR_NULL(sgm->usb3_phy)) {
		INIT_WORK(&sgm->usb_work, sgm4154x_usb_work);
		sgm->usb_nb.notifier_call = sgm4154x_usb_notifier;
		otg_notify = usb_register_notifier(sgm->usb3_phy, &sgm->usb_nb);
	}*/
	sgm->fcc_votable = create_votable("FCC", VOTE_MIN,
					sgm4154x_set_ichrg_curr,
					sgm);
	if (IS_ERR(sgm->fcc_votable)) {
		ret = PTR_ERR(sgm->fcc_votable);
		sgm->fcc_votable = NULL;
		goto destroy_votable;
	}
	sgm->chg_disable_votable = create_votable("CHG_DISABLE", VOTE_SET_ANY,
					sgm_disable_chg_vote_callback,
					sgm);
	if (IS_ERR(sgm->chg_disable_votable)) {
		ret = PTR_ERR(sgm->chg_disable_votable);
		sgm->chg_disable_votable = NULL;
		goto destroy_fcc_votable;
	}
	INIT_WORK(&sgm->usb_work, sgm4154x_usb_work);
	sgm->usb_nb.notifier_call = sgm4154x_usb_notifier;
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
	otg_notify = register_sgm_usb_notifier(&sgm->usb_nb);
	if (otg_notify) {
		dev_err(sgm->dev,
		  "%s: sgm usb notifier registration failed: %d\n",
		  __func__, otg_notify);
		return otg_notify;
	}
#endif/*CONFIG_DOUBLE_CHARGERS*/
	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						sgm4154x_irq_handler_thread,
						IRQF_TRIGGER_FALLING | 
						IRQF_ONESHOT,
						dev_name(&client->dev), sgm);
		if (ret)
			goto error_out;
		enable_irq_wake(client->irq);
	}

	INIT_DELAYED_WORK(&sgm->init_typec_class_delay_work, init_typec_class_work_func);
	INIT_DELAYED_WORK(&sgm->charge_detect_delayed_work, charger_detect_work_func);
	INIT_DELAYED_WORK(&sgm->otg_detect_delayed_work, otg_detect_work_func);
	INIT_DELAYED_WORK(&sgm->charge_monitor_work, charger_monitor_work_func);
	INIT_DELAYED_WORK(&sgm->recharge_delayed_work, recharge_work_func);

	schedule_delayed_work(&sgm->init_typec_class_delay_work, msecs_to_jiffies(30000));
	sgm->pm_nb.notifier_call = sgm4154x_suspend_notifier;
    	register_pm_notifier(&sgm->pm_nb);

	ret = sgm4154x_power_supply_init(sgm, dev);
	if (ret) {
		dev_err(dev, "Failed to register power supply\n");
		goto error_out;
	}

	ret = sgm4154x_usb_extcon_register(sgm);
	if (ret) {
		dev_err(dev, "request sgm4154x_usb_extcon_register failed\n");
		goto error_out;
	}

	ret = sgm4154x_hw_init(sgm);
	if (ret) {
		dev_err(dev, "Cannot initialize the chip.\n");
		goto error_out;
	}
	

	//OTG setting
	sgm4154x_set_otg_voltage(sgm, sgm->init_data.vboost); //5V
	sgm4154x_set_otg_current(sgm, sgm->init_data.iboost); //1.2A

	//ret = sgm4154x_vbus_regulator_register(sgm);
	g_sgm = sgm;
	ret = his_register_sysfs_attr_group(&sgm_sys_node_attr_group);
	if(ret < 0){
		dev_err(dev,"Couldn't create sgm_sys_node_attr_group rc=%d\n", ret);
		goto error_out;
	}

	pr_err("%s sgm4154x_probe success\n", __func__);
	return ret;
error_out:
	if (!IS_ERR_OR_NULL(sgm->usb2_phy))
		usb_unregister_notifier(sgm->usb2_phy, &sgm->usb_nb);

	if (!IS_ERR_OR_NULL(sgm->usb3_phy))
		usb_unregister_notifier(sgm->usb3_phy, &sgm->usb_nb);

	destroy_votable(sgm->chg_disable_votable);
destroy_fcc_votable:
	destroy_votable(sgm->fcc_votable);
	pr_info("%s sgm4154x_probe fcc_votable error\n", __func__);
destroy_votable:
	pr_info("%s sgm4154x_probe error\n", __func__);
	return ret;
}

static int sgm4154x_charger_remove(struct i2c_client *client)
{
    struct sgm4154x_device *sgm = i2c_get_clientdata(client);

    cancel_delayed_work_sync(&sgm->charge_monitor_work);
    cancel_delayed_work_sync(&sgm->recharge_delayed_work);
	
    regulator_unregister(sgm->otg_rdev);

    power_supply_unregister(sgm->charger); 
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
	unregister_sgm_usb_notifier(&sgm->usb_nb);
#endif/*CONFIG_DOUBLE_CHARGERS*/
	if(sgm->fcc_votable)
		destroy_votable(sgm->fcc_votable);
	if(sgm->chg_disable_votable)
		destroy_votable(sgm->chg_disable_votable);
	mutex_destroy(&sgm->lock);       
	g_sgm = NULL;
	mutex_destroy(&sgm->lock);
	pr_info("%s sgm4154x_charger_remove\n", __func__);
    return 0;
}

static void sgm4154x_charger_shutdown(struct i2c_client *client)
{
    int ret = 0;
	
	struct sgm4154x_device *sgm = i2c_get_clientdata(client);

    vote(sgm->chg_disable_votable, HW_LIMIT_VOTER, true, 0);
    ret = sgm4154x_disable_vbus(sgm);
    if (ret) {
        pr_err("%s Failed to disable otg vbus, ret = %d\n", __func__, ret);
    }
    g_sgm = NULL;
    pr_info("%s sgm4154x_charger_shutdown\n", __func__);
}

static const struct i2c_device_id sgm4154x_i2c_ids[] = {
	{ "sgm41541", 0 },
	{ "sgm41542", 0 },
	{ "sgm41543", 0 },
	{ "sgm41543D", 0 },
	{ "sgm41513", 0 },
	{ "sgm41513A", 0 },
	{ "sgm41513d_chg", 0 },
	{ "sgm41516", 0 },
	{ "sgm41516D", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sgm4154x_i2c_ids);

static const struct of_device_id sgm4154x_of_match[] = {
	{ .compatible = "sgm,sgm41541", },
	{ .compatible = "sgm,sgm41542", },
	{ .compatible = "sgm,sgm41543", },
	{ .compatible = "sgm,sgm41543D", },
	{ .compatible = "sgm,sgm41513", },
	{ .compatible = "sgm,sgm41513A", },
	{ .compatible = "sgm,sgm41513d_chg", },
	{ .compatible = "sgm,sgm41516", },
	{ .compatible = "sgm,sgm41516D", },
	{ },
};
MODULE_DEVICE_TABLE(of, sgm4154x_of_match);

static struct i2c_driver sgm4154x_driver = {
	.driver = {
		.name = "sgm4154x_charger",
		.of_match_table = sgm4154x_of_match,		
	},
	.probe = sgm4154x_probe,
	.remove = sgm4154x_charger_remove,
	.shutdown = sgm4154x_charger_shutdown,
	.id_table = sgm4154x_i2c_ids,
};
module_i2c_driver(sgm4154x_driver);

MODULE_AUTHOR(" qhq <Allen_qin@sg-micro.com>");
MODULE_DESCRIPTION("sgm4154x charger driver");
MODULE_LICENSE("GPL v2");
