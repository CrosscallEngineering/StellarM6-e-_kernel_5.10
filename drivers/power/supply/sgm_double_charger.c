/* Copyright (c) 2015 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "--PM-SGM: %s: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/usb.h>
#include <linux/usb/phy.h>
#include <linux/usb/ucsi_glink.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/his_debug_base.h>

#define	SGMCHARGER_ATTR_RO(_name)	\
	static struct kobj_attribute _name##_attr ={\
		.attr =	{			\
			.name = __stringify(_name),	\
			.mode = 0444,			\
		},					\
		.show = _name##_show,			\
		.store = NULL,				\
	}

#define	SGMCHARGER_ATTR_RW(_name)	\
	static struct kobj_attribute _name##_attr = __ATTR_RW(_name)

struct sgm_chip {
	struct platform_device *pdev;
	struct delayed_work	chg_redet_work;
	struct delayed_work	ext_otg_detect_work;
	struct power_supply	*batt_psy;
	struct power_supply	*sgm_chg_psy;
	struct power_supply	*usb_psy;
	struct notifier_block 	nb;
	struct notifier_block	ucsi_glink_nb;
	u32		usb_connector_num;
	u32		mag_chg_gpio;
	int		mag_chg_irq;
	u32		ext_otg_gpio;
	int		ext_otg_irq;
	u32		typec_chg_gpio;
	int		typec_chg_irq;
	u32		usb_switch_gpio;
	bool		ext_otg_disabled;
	bool		typec_otg_present;
	unsigned long inputs;
	bool		bootup_detect;
	struct completion typec_charger_apsd_done;
};

#define MAG_BUS_VLD		0
#define TYPEC_BUS_VLD	1
#define EXT_OTG_VLD		2
#define TYPEC_OTG_VLD	3
#define MAG_USB_VLD		4
#define SGM_INIT			7

enum sgm_mode
{
    sgm_default_mode = 0,
    ext_otg_mode,
    usb_otg_mode,
    magcon_chg_mode,
    usb_chg_mode,
    magcon_usb_chg_mode,
    usb_chg_ext_otg_mode,
    magcon_chg_usb_otg_mode,
    usb_ext_otg_mode,
};

static struct sgm_chip *global_chip = NULL;
static char *doubleinput[2] = { "AF_BF_EVENT=AF_BF_EVENT", NULL };

static RAW_NOTIFIER_HEAD(sgm_usb_notifier);
int register_sgm_usb_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&sgm_usb_notifier, nb);
}
EXPORT_SYMBOL(register_sgm_usb_notifier);

int unregister_sgm_usb_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_unregister(&sgm_usb_notifier, nb);
}
EXPORT_SYMBOL(unregister_sgm_usb_notifier);
void sgm_usb_switch_gpio(int value){
	gpio_direction_output(global_chip->usb_switch_gpio, value);
	return;
}
EXPORT_SYMBOL_GPL(sgm_usb_switch_gpio);

void sgm_complete_apsd_done(void){
	if(!global_chip){
		pr_err("no sgm device\n");
		return;
	}
	complete(&global_chip->typec_charger_apsd_done);
	return;
}
EXPORT_SYMBOL_GPL(sgm_complete_apsd_done);

bool sgm_mag_charger_ok(void)
{
	if (!global_chip) {
		pr_err("sgm: err,no chip\n");
		return 0;
	}

	return !!test_bit(MAG_BUS_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm_mag_charger_ok);

bool sgm_typec_charger_ok(void)
{
	if (!global_chip) {
		pr_err("sgm: err,no chip\n");
		return 0;
	}

	return !!test_bit(TYPEC_BUS_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm_typec_charger_ok);

bool sgm_ext_otg_ok(void)
{
	if (!global_chip) {
		pr_err("sgm: err,no chip\n");
		return 0;
	}

	return !!test_bit(EXT_OTG_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm_ext_otg_ok);

 int sgm_typec_otg_ok(void)
{
	if (!global_chip) {
		pr_err("sgm: err,no chip\n");
		return 0;
	}

	return global_chip->typec_otg_present;
}
EXPORT_SYMBOL_GPL(sgm_typec_otg_ok);

static bool sgm_double_charger_ok(void)
{
	if (!global_chip) {
		pr_err("no chip\n");
		return 0;
	}

	return !!(test_bit(MAG_BUS_VLD, &global_chip->inputs)
		&& test_bit(TYPEC_BUS_VLD, &global_chip->inputs));
}

static irqreturn_t sgm_mag_chg_irq(int irq, void *data)
{
	struct sgm_chip *chip = data;

	pr_err("sgm: trigger.\n");

	cancel_delayed_work_sync(&chip->chg_redet_work);
	schedule_delayed_work(&chip->chg_redet_work, msecs_to_jiffies(300));
	return IRQ_HANDLED;
}

static irqreturn_t sgm_typec_chg_irq(int irq, void *data)
{
	struct sgm_chip *chip = data;

	pr_err("sgm: trigger.\n");

	cancel_delayed_work_sync(&chip->chg_redet_work);
	schedule_delayed_work(&chip->chg_redet_work, msecs_to_jiffies(100));	//confirm dpdm detected

	return IRQ_HANDLED;
}

static irqreturn_t sgm_ext_otg_irq(int irq, void *data)
{
	struct sgm_chip *chip = data;

	pr_err("sgm: triggered\n");

	cancel_delayed_work_sync(&chip->ext_otg_detect_work);
	schedule_delayed_work(&chip->ext_otg_detect_work, msecs_to_jiffies(50));

	return IRQ_HANDLED;
}

static int sgm_check_power_supply_psy(struct sgm_chip *chip)
{
	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
		if (!chip->batt_psy) {
			dev_err(&chip->pdev->dev, "Could not get battery power_supply\n");
			return -ENODEV;
		}
	}

	if (!chip->sgm_chg_psy) {
		chip->sgm_chg_psy= power_supply_get_by_name("sgm_charger");
		if (!chip->sgm_chg_psy) {
			dev_err(&chip->pdev->dev, "Could not get sgm_charger power_supply\n");
			return -ENODEV;
		}
	}

	if(!chip->usb_psy){
		chip->usb_psy = power_supply_get_by_name("usb");
		if(!chip->usb_psy){
			dev_err(&chip->pdev->dev, "Could not get usb power_supply\n");
			return -ENODEV;
		}
	}
	return 0;
}

static int sgm_power_supply_notifier_call(struct notifier_block *nb,
		unsigned long ev, void *v)
{
	struct power_supply *psy = v;
	struct sgm_chip *chip = container_of(nb, struct sgm_chip, nb);

	if (ev != PSY_EVENT_PROP_CHANGED)
			return NOTIFY_OK;
	
	if(!strcmp(psy->desc->name, "battery") || !strcmp(psy->desc->name, "usb") || !strcmp(psy->desc->name, "sgm_charger")){
		if(chip->bootup_detect){
			pr_err("sgm: bootup detect, schedule otg and chg det works\n");
			schedule_delayed_work(&chip->ext_otg_detect_work,msecs_to_jiffies(1500));
			schedule_delayed_work(&chip->chg_redet_work,0);
		}
	}
	
	return NOTIFY_OK;
}

static int sgm_ucsi_glink_notifier_call(struct notifier_block *nb_ptr,
				      unsigned long evt, void *ptr){
	struct sgm_chip *chip = container_of(nb_ptr, struct sgm_chip, ucsi_glink_nb);
	bool partner_otg = ((struct ucsi_glink_constat_info *)ptr)->partner_otg;
	char *usb3x_device_attached[2] = {"USB3X_DEVICE=1", NULL};
	char *usb3x_device_detached[2] = {"USB3X_DEVICE=0", NULL};
	if(partner_otg != chip->typec_otg_present){
		pr_err("sgm: typec otg status change to %s, schedule otg det work\n", 
			partner_otg ? "present" : "absent");
		if(partner_otg)
			kobject_uevent_env(&chip->pdev->dev.kobj, KOBJ_CHANGE, usb3x_device_attached);
		else
			kobject_uevent_env(&chip->pdev->dev.kobj, KOBJ_CHANGE, usb3x_device_detached);
		chip->typec_otg_present = partner_otg;
		cancel_delayed_work_sync(&chip->ext_otg_detect_work);
		schedule_delayed_work(&chip->ext_otg_detect_work, msecs_to_jiffies(200));
	}
	return NOTIFY_OK;
}

static void sgm_ext_otg_detect_work(struct work_struct *work)
{
	struct sgm_chip *chip = container_of(work, struct sgm_chip,
		ext_otg_detect_work.work);
	bool otg_id_status = !gpio_get_value(chip->ext_otg_gpio);
	bool ext_vbus_status = !gpio_get_value(chip->mag_chg_gpio);
	bool enable_ext_otg = false, disable_ext_otg = false, need_redet = false;
	union power_supply_propval pval = {0, };
	pr_err("sgm: input 0x%lx, otg_id_status=%d typec_otg_present=%d,ext_vbus_status=%d\n", chip->inputs, otg_id_status,
		chip->typec_otg_present,ext_vbus_status);

	if(((otg_id_status << EXT_OTG_VLD) | (chip->typec_otg_present << TYPEC_OTG_VLD))
		== (chip->inputs & (BIT(EXT_OTG_VLD) | BIT(TYPEC_OTG_VLD)))) {
		pr_err("sgm: double otg status are same as last, do nothing\n");
		return;
	}

	if (sgm_check_power_supply_psy(chip))
		return;

	/*If ext chg is working, ignore ext vbus status.*/
	if (sgm_mag_charger_ok() && otg_id_status) {
		if(ext_vbus_status)
			otg_id_status = false;
		else
			clear_bit(MAG_BUS_VLD, &chip->inputs);
	}
	if(!(test_bit(EXT_OTG_VLD, &chip->inputs) || test_bit(TYPEC_OTG_VLD, &chip->inputs)) && 
		(!chip->typec_otg_present && otg_id_status)){
		//none to ext otg 
		pr_err("sgm: none to ext otg\n");
		enable_ext_otg = true;
		goto set_otg_flag;
	}else if((test_bit(EXT_OTG_VLD, &chip->inputs) && !test_bit(TYPEC_OTG_VLD, &chip->inputs)) && 
			!(otg_id_status || chip->typec_otg_present)){
		//ext otg to none
		pr_err("sgm: ext otg to none\n");
		disable_ext_otg = true;
		goto set_otg_flag;
	}else if(!(test_bit(EXT_OTG_VLD, &chip->inputs) || test_bit(TYPEC_OTG_VLD, &chip->inputs)) && 
		(chip->typec_otg_present && !otg_id_status)){
		pr_err("sgm: none to typec otg\n");
		if(chip->usb_connector_num == 1){
			if(sgm_mag_charger_ok()){
				power_supply_get_property(chip->sgm_chg_psy, POWER_SUPPLY_PROP_USB_TYPE, &pval);
				if(pval.intval == POWER_SUPPLY_USB_TYPE_SDP || pval.intval == POWER_SUPPLY_USB_TYPE_CDP){
					pr_err("sgm: notify sgm charger to disable device mode\n");
					need_redet = true;
				}
			}
		}
		goto set_otg_flag;
	}else if((!test_bit(EXT_OTG_VLD, &chip->inputs) && test_bit(TYPEC_OTG_VLD, &chip->inputs)) && 
		!(otg_id_status || chip->typec_otg_present)){
		pr_err("sgm: typec otg to none\n");
		if(chip->usb_connector_num == 1){
			if(sgm_mag_charger_ok()){
				power_supply_get_property(chip->sgm_chg_psy, POWER_SUPPLY_PROP_USB_TYPE, &pval);
				if(pval.intval == POWER_SUPPLY_USB_TYPE_SDP || pval.intval == POWER_SUPPLY_USB_TYPE_CDP){
					pr_err("sgm: notify sgm charger to enable device mode\n");
					need_redet = true;
				}
			}
		}
		goto set_otg_flag;
	}else if(otg_id_status && chip->typec_otg_present){
		//single otg to double otgs
		if(test_bit(EXT_OTG_VLD, &chip->inputs) && !test_bit(TYPEC_OTG_VLD, &chip->inputs)){
			pr_err("sgm: ext otg to double otg\n");
			if(chip->usb_connector_num == 1)
				need_redet = true;

		}else if(!test_bit(EXT_OTG_VLD, &chip->inputs) && test_bit(TYPEC_OTG_VLD, &chip->inputs)){
			pr_err("sgm: typec otg to double otg\n");
			enable_ext_otg = true;
		}
		goto set_otg_flag;
	}else if(test_bit(EXT_OTG_VLD, &chip->inputs) && test_bit(TYPEC_OTG_VLD, &chip->inputs)
		&& ((!otg_id_status && chip->typec_otg_present) || (otg_id_status && !chip->typec_otg_present))){
		//double otgs to single otg
		pr_err("sgm: double otg to %s otg", chip->typec_otg_present ? "typec" : "ext");
		if(chip->usb_connector_num == 1){
			if(chip->typec_otg_present)
				disable_ext_otg = true;
			else 
				need_redet = true;
		}
		goto set_otg_flag;
	}else{
set_otg_flag:		
		if(enable_ext_otg){
			disable_irq(chip->mag_chg_irq);
			clear_bit(MAG_BUS_VLD, &chip->inputs);
			if(chip->ext_otg_disabled)
				chip->ext_otg_disabled = false;
			raw_notifier_call_chain(&sgm_usb_notifier, USB_EVENT_ID, NULL);
		}
		
		if(disable_ext_otg){
			if(chip->ext_otg_disabled)
				chip->ext_otg_disabled = false;
			raw_notifier_call_chain(&sgm_usb_notifier, USB_EVENT_NONE, NULL);
			enable_irq(chip->mag_chg_irq);
		}
		
		otg_id_status ? set_bit(EXT_OTG_VLD, &chip->inputs) : clear_bit(EXT_OTG_VLD, &chip->inputs);
		chip->typec_otg_present ? set_bit(TYPEC_OTG_VLD, &chip->inputs) : clear_bit(TYPEC_OTG_VLD, &chip->inputs);

		if(need_redet){
			if(sgm_mag_charger_ok()){
				pr_err("sgm: redet sgm device mode");
				raw_notifier_call_chain(&sgm_usb_notifier, USB_EVENT_VBUS, NULL);
			}else{
				pr_err("sgm: redet sgm host mode");
				raw_notifier_call_chain(&sgm_usb_notifier, USB_EVENT_ID, NULL);
			}
		}		
	}
	return;
}

static void sgm_chg_redet_work(struct work_struct *w)
{
	struct sgm_chip *chip = container_of(w, struct sgm_chip,
		chg_redet_work.work);
	bool ext_vbus_status = !gpio_get_value(chip->mag_chg_gpio);
	bool typec_vbus_status = !gpio_get_value(chip->typec_chg_gpio);
	bool need_redet = false, need_report = true, typec_vbus_rising = false;
	union power_supply_propval pval = {3000000, };	//default usb_in_icl is 2A

	if (sgm_check_power_supply_psy(chip))
		return;

	pr_err("sgm: input 0x%lx, ext_status=%d typec_status=%d\n", chip->inputs, ext_vbus_status, typec_vbus_status);
	if(((ext_vbus_status << MAG_BUS_VLD) | (typec_vbus_status << TYPEC_BUS_VLD))
		== (chip->inputs & (BIT(MAG_BUS_VLD) | BIT(TYPEC_BUS_VLD)))) {
		pr_err("sgm: double charge status are same as last, do nothing\n");
		goto work_finished;
	}

	/*If ext otg is working, ignore ext vbus status.*/
	if (sgm_ext_otg_ok() && ext_vbus_status)
		ext_vbus_status = 0;

	if (typec_vbus_status){
		pval.intval = 0;
		power_supply_get_property(chip->usb_psy, POWER_SUPPLY_PROP_PRESENT, &pval);
		if(!pval.intval){
			pr_info("typec vbus not present, may be otg\n");
			typec_vbus_status = 0;
		}
	}

	if(!(test_bit(MAG_BUS_VLD, &chip->inputs) || test_bit(TYPEC_BUS_VLD, &chip->inputs)) && 
		(!typec_vbus_status && ext_vbus_status)){
		//none to sgm charger 
		pr_err("sgm: none to sgm charger\n");
		if(chip->bootup_detect)
			need_redet = true;
		goto set_chg_flag;
	}else if((test_bit(MAG_BUS_VLD, &chip->inputs) && !test_bit(TYPEC_BUS_VLD, &chip->inputs)) && 
			!(ext_vbus_status || typec_vbus_status)){
		//sgm charger to none
		pr_err("sgm: sgm charger to none\n");
		goto set_chg_flag;
	}else if((!(test_bit(MAG_BUS_VLD, &chip->inputs) || test_bit(TYPEC_BUS_VLD, &chip->inputs)) && 
		(typec_vbus_status && !ext_vbus_status)) || 
		((!test_bit(MAG_BUS_VLD, &chip->inputs) && test_bit(TYPEC_BUS_VLD, &chip->inputs)) && 
			!(ext_vbus_status || typec_vbus_status))){
		//none to typec charger or typec charger to none
		pr_err("sgm: %s\n", typec_vbus_status ? "none to typec charger" : "typec charger to none");
		if(chip->usb_connector_num == 1){
			if(sgm_ext_otg_ok()){
				pr_err("sgm: notify sgm charger to redet otg\n");
				need_redet = true;
			}
		}
		if(typec_vbus_status)
			typec_vbus_rising = true;
		else
			reinit_completion(&chip->typec_charger_apsd_done);
		goto set_chg_flag;
	}else if (test_bit(MAG_BUS_VLD, &chip->inputs) && test_bit(TYPEC_BUS_VLD, &chip->inputs)
		&& ((!ext_vbus_status && typec_vbus_status) || (ext_vbus_status && !typec_vbus_status))) {
		/*double charger to single charger*/
		pr_err("sgm: double charger to %s charger, unsuspend typec input if suspended\n", ext_vbus_status ? "sgm" : "typec");
		if(ext_vbus_status){
			reinit_completion(&chip->typec_charger_apsd_done);
			if(chip->usb_connector_num == 1)
				need_redet = true;
		}

		if(chip->usb_connector_num == 1){
				power_supply_get_property(chip->batt_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &pval);
				if(pval.intval == 0){
					pval.intval = 1;
					power_supply_set_property(chip->batt_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &pval);
			}
		}else if(chip->usb_connector_num == 2){
			pval.intval = 3000 * 1000;
			power_supply_set_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &pval);
			power_supply_set_property(chip->sgm_chg_psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &pval);
			pr_err("sgm: double inputs to single input, set fcc %d mA\n", pval.intval / 1000);
		}
		goto set_chg_flag;
	} else if(ext_vbus_status && typec_vbus_status &&
		((test_bit(MAG_BUS_VLD, &chip->inputs) && !test_bit(TYPEC_BUS_VLD, &chip->inputs)) ||
		(!test_bit(MAG_BUS_VLD, &chip->inputs) && test_bit(TYPEC_BUS_VLD, &chip->inputs)))) {
		/*single charger to double charger, notify sgm charger to start/stop charging/redet device mode, or notify typec redet device mode if needed.*/
		if(test_bit(MAG_BUS_VLD, &chip->inputs)){
			pr_err("sgm: sgm charger to double chargers\n");
			typec_vbus_rising = true;
			if(chip->usb_connector_num == 1)
				need_redet = true;
		}
		else{
			pr_err("sgm: typec charger to double chargers\n");
			if(chip->bootup_detect)
				need_redet = true;
		}

		if(chip->usb_connector_num == 2){
			pr_err("sgm: double inputs, set charge current to 500mA\n");
			pval.intval = 500 * 1000;
			power_supply_set_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &pval);

			power_supply_set_property(chip->sgm_chg_psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &pval);
		}
		goto set_chg_flag;
	} else {
set_chg_flag:
		ext_vbus_status ? set_bit(MAG_BUS_VLD, &chip->inputs) : clear_bit(MAG_BUS_VLD, &chip->inputs);
		typec_vbus_status ? set_bit(TYPEC_BUS_VLD, &chip->inputs) : clear_bit(TYPEC_BUS_VLD, &chip->inputs);

		if(typec_vbus_rising)
			wait_for_completion_timeout(&chip->typec_charger_apsd_done, msecs_to_jiffies(3500));
		
		if (need_redet) {
			if(sgm_ext_otg_ok()){
				pr_err("sgm: redet sgm host mode\n");
				raw_notifier_call_chain(&sgm_usb_notifier, USB_EVENT_ID, NULL);
			}
			else if(sgm_mag_charger_ok()){
				pr_err("sgm: redet sgm charging enabled or device mode\n");
				raw_notifier_call_chain(&sgm_usb_notifier, USB_EVENT_VBUS, NULL);
			}
			else{
				//redet typec charger if needed
			}
		}
		if(need_report)
			kobject_uevent_env(&chip->pdev->dev.kobj, KOBJ_CHANGE, doubleinput);
	}
work_finished:
	if(chip->bootup_detect)
		chip->bootup_detect = false;
	return;
}

static int sgm_parse_dt(struct device *dev,
		struct sgm_chip *chip)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	ret = device_property_read_u32(dev, "sgm,usb-connector-num", &chip->usb_connector_num);
	if(ret)
		chip->usb_connector_num = 1;
	/* mag-chg-flag-gpio */
	ret = of_get_named_gpio(np, "sgm,mag-chg-flag-gpio", 0);
	chip->mag_chg_gpio = ret;
	if(!gpio_is_valid(chip->mag_chg_gpio)) {
		dev_err(dev, "invalid mag-chg-flag-gpio. ret = %d\n", ret);
		return ret;
	}

	ret = gpio_request(chip->mag_chg_gpio, "sgm,mag-chg-flag-gpio");
	if (ret < 0) {
		dev_err(dev,
			"%s: gpio %d request failed. ret = %d\n",
			__func__, chip->mag_chg_gpio, ret);
		return ret;
	}


	/*ext-otg-id-gpio*/
	ret = of_get_named_gpio(np, "sgm,ext-otg-id-gpio", 0);
	chip->ext_otg_gpio = ret;
	if(!gpio_is_valid(chip->ext_otg_gpio)) {
		dev_err(dev, "invalid ext_otg_gpio gpio: %d\n", ret);
		return ret;
	}

	ret = gpio_request(chip->ext_otg_gpio, "sgm-ext-otg-id-gpio");
	if (ret < 0) {
		dev_err(dev,
			"%s: gpio %d request failed. ret = %d\n",
			__func__, chip->ext_otg_gpio, ret);
		return ret;
	}

	/*typec-chg-flag-gpio*/
	ret = of_get_named_gpio(np, "sgm,typec-chg-flag-gpio", 0);
	chip->typec_chg_gpio = ret;
	if(!gpio_is_valid(chip->typec_chg_gpio))
		dev_err(dev, "invalid typec_chg_gpio rc = %d\n", ret);

	ret = gpio_request(chip->typec_chg_gpio, "sgm-typec-chg-flag-gpio");
	if (ret < 0)
		dev_err(dev,
			"gpio %d request failed. rc = %d\n", chip->typec_chg_gpio, ret);

	/*usb-switch-gpio*/
	ret = of_get_named_gpio(np, "sgm,usb-switch-gpio", 0);
	if(!gpio_is_valid(ret)){
		dev_err(dev, "invalid usb_switch_gpio rc = %d\n", ret);
		return ret;
	}
	chip->usb_switch_gpio = ret;
	ret = gpio_request(chip->usb_switch_gpio, "sgm-usb-switch-gpio");
	if(ret < 0){
		dev_err(dev,
			"%s: gpio %d request failed. ret = %d\n",
			__func__, chip->usb_switch_gpio, ret);
		return ret;
	}
		
	return 0;
}

static ssize_t sgm2540_mode_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	 int mode = 0;
	if (global_chip->inputs == 0x84)
		mode = ext_otg_mode;
	else if (global_chip->inputs == 0x88)
		mode = usb_otg_mode;
	else if (global_chip->inputs == 0x83)
		mode = magcon_usb_chg_mode;
	else if (global_chip->inputs == 0x81)
		mode = magcon_chg_mode;
	else if (global_chip->inputs == 0x82)
		mode = usb_chg_mode;
	else if(global_chip->inputs == 0x86)
		mode = usb_chg_ext_otg_mode;
	else if(global_chip->inputs == 0x89)
		mode = magcon_chg_usb_otg_mode;
	else if(global_chip->inputs == 0x8C)
		mode = usb_ext_otg_mode;
	else
		mode = sgm_default_mode;

	pr_err("sgm: global_chip->inputs=0x%x\n", global_chip->inputs);

	return sprintf(buf, "%d\n", mode);
}

static ssize_t typec_attached_state_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int attached_state = 0;
	bool typec_vbus_status = !gpio_get_value(global_chip->typec_chg_gpio);

	if (sgm_typec_otg_ok() || typec_vbus_status)
		attached_state = 1;
	else
		attached_state = 0;

	pr_err("attached_state=0x%x\n", attached_state);

	return sprintf(buf, "%d\n", attached_state);
}

static ssize_t double_charger_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int double_charger_state = sgm_double_charger_ok();

	pr_err("attached_state=0x%x\n", double_charger_state);

	return sprintf(buf, "%d\n", double_charger_state);
}

static ssize_t mag_otg_ctrl_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count){
	int rc = 0;
	int value;

	if (sscanf(buf, "%d", &value) != 1){
		pr_buf_err("sscanf is wrong!\n");
		return -EINVAL;
	}

	global_chip->ext_otg_disabled = !value;
	
	if(sgm_ext_otg_ok()){
		pr_buf_info("set mag otg status to %s\n", value ? "enabled" : "disabled");
		raw_notifier_call_chain(&sgm_usb_notifier, value ? USB_EVENT_ID : USB_EVENT_NONE, NULL);
	}

	rc = count;
	return count;
}

static ssize_t mag_otg_ctrl_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf){
		
	return sprintf(buf, "%d\n", !global_chip->ext_otg_disabled);
}

SGMCHARGER_ATTR_RO(sgm2540_mode);
SGMCHARGER_ATTR_RO(typec_attached_state);
SGMCHARGER_ATTR_RO(double_charger);
SGMCHARGER_ATTR_RW(mag_otg_ctrl);

static struct attribute *sgm_sys_node_attrs[] = {
	&sgm2540_mode_attr.attr,
	&typec_attached_state_attr.attr,
	&double_charger_attr.attr,
	&mag_otg_ctrl_attr.attr,
	NULL,
};

static struct attribute_group sgm_sys_node_attr_group = {
	.attrs = sgm_sys_node_attrs,
};

static int sgm_debugfs_init(struct sgm_chip *chip)
{
	int ret;

	ret = his_register_sysfs_attr_group(&sgm_sys_node_attr_group);
	if (ret < 0)
		pr_err("Error create sgm_sys_node_attr_group %d\n", ret);

	return 0;
}

static int sgm_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct sgm_chip *chip;

	//pr_err("sgm:sgm_probe start\n");

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->pdev = pdev;
	dev_set_drvdata(&pdev->dev, chip);
	if (pdev->dev.of_node) {
		ret = sgm_parse_dt(&pdev->dev, chip);
		if (ret) {
			dev_err(&pdev->dev,"%s: sgm_parse_dt() err\n", __func__);
			goto err_parse_dt;
		}
	} else {
		dev_err(&pdev->dev, "No dts data\n");
		goto err_parse_dt;
	}

	global_chip = chip;
	init_completion(&chip->typec_charger_apsd_done);
	INIT_DELAYED_WORK(&chip->chg_redet_work, sgm_chg_redet_work);
	INIT_DELAYED_WORK(&chip->ext_otg_detect_work, sgm_ext_otg_detect_work);

	chip->nb.notifier_call = sgm_power_supply_notifier_call;
	ret = power_supply_reg_notifier(&chip->nb);
	if (ret < 0) {
		dev_err(&pdev->dev,
		  "%s: power supply notifier registration failed: %d\n", __func__, ret);
		goto err_parse_dt;
	}

	chip->ucsi_glink_nb.notifier_call = sgm_ucsi_glink_notifier_call;
	ret = register_ucsi_glink_notifier(&chip->ucsi_glink_nb);
	if (ret < 0) {
		dev_err(&pdev->dev,
		  "%s: usci glink notifier registration failed: %d\n", __func__, ret);
		goto err_parse_dt;
	}

	chip->typec_chg_irq = gpio_to_irq(chip->typec_chg_gpio);
	chip->mag_chg_irq = gpio_to_irq(chip->mag_chg_gpio);
	chip->ext_otg_irq = gpio_to_irq(chip->ext_otg_gpio);

	ret = request_threaded_irq(chip->typec_chg_irq,
			NULL,
			sgm_typec_chg_irq,
			IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"sgm_typec_chg_irq", chip);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed for typec chg det\n");
		goto err_parse_dt;
	}

	ret = request_threaded_irq(chip->mag_chg_irq,
			NULL,
			sgm_mag_chg_irq,
			IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"sgm_mag_chg_irq", chip);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed for sgm_mag_chg\n");
		goto err_parse_dt;
	}

	ret = request_threaded_irq(chip->ext_otg_irq,
			NULL,
			sgm_ext_otg_irq,
			IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"sgm_ext_otg_irq", chip);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed for ext otg fault\n");
		goto err_parse_dt;
	}

	enable_irq_wake(chip->typec_chg_irq);
	enable_irq_wake(chip->mag_chg_irq);
	enable_irq_wake(chip->ext_otg_irq);

	set_bit(SGM_INIT, &chip->inputs);
	chip->bootup_detect = true;
	sgm_debugfs_init(chip);

	pr_err("sgm:sgm_probe success\n");
	return 0;

err_parse_dt:
	devm_kfree(&pdev->dev,chip);
	return ret;
}

static void sgm_shutdown(struct platform_device *pdev)
{
	struct sgm_chip *chip = platform_get_drvdata(pdev);

	if (chip) {
		cancel_delayed_work_sync(&chip->ext_otg_detect_work);
		cancel_delayed_work_sync(&chip->chg_redet_work);
	}

	global_chip = NULL;
}

static int sgm_remove(struct platform_device *pdev)
{
	struct sgm_chip *chip = platform_get_drvdata(pdev);

	if (chip) {
		cancel_delayed_work_sync(&chip->ext_otg_detect_work);
		cancel_delayed_work_sync(&chip->chg_redet_work);
	}
	power_supply_unreg_notifier(&chip->nb);
	global_chip = NULL;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sgm_suspend(struct device *dev)
{
	//struct sgm_chip *chip  = dev_get_drvdata(dev);

	return 0;
}

static int sgm_resume(struct device *dev)
{
	//struct sgm_chip *chip  = dev_get_drvdata(dev);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(sgm_pm_ops, sgm_suspend,
			  sgm_resume);

static struct of_device_id sgm_match_table[] = {
	{	.compatible = "sgmicro,double-chg",
	},
	{}
};

static struct platform_driver sgm_driver = {
	.driver		= {
		.name		= "sgm",
		.owner		= THIS_MODULE,
		.of_match_table	= sgm_match_table,
		.pm	= &sgm_pm_ops,
	},
	.probe		= sgm_probe,
	.remove		= sgm_remove,
	.shutdown 	= sgm_shutdown,
};

module_platform_driver(sgm_driver);

MODULE_DESCRIPTION("SGM Double Charger Driver");
MODULE_AUTHOR("Shaoxiang.Li");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sgm");
