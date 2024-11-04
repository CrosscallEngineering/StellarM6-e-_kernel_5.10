// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 */
#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/usb/typec.h>
#include <linux/usb/ucsi_glink.h>
#include <linux/soc/qcom/fsa4480-i2c.h>
#include <linux/qti-regmap-debugfs.h>
#if IS_ENABLED(CONFIG_QCOM_FSA4480_HEADSET)
#include <linux/iio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/irqreturn.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/pm_wakeup.h>
#endif 

#define FSA4480_I2C_NAME	"fsa4480-driver"

#define FSA4480_SWITCH_SETTINGS 0x04
#define FSA4480_SWITCH_CONTROL  0x05
#define FSA4480_SWITCH_STATUS1  0x07
#define FSA4480_SLOW_L          0x08
#define FSA4480_SLOW_R          0x09
#define FSA4480_SLOW_MIC        0x0A
#define FSA4480_SLOW_SENSE      0x0B
#define FSA4480_SLOW_GND        0x0C
#define FSA4480_DELAY_L_R       0x0D
#define FSA4480_DELAY_L_MIC     0x0E
#define FSA4480_DELAY_L_SENSE   0x0F
#define FSA4480_DELAY_L_AGND    0x10
#define FSA4480_RESET           0x1E

#if IS_ENABLED(CONFIG_QCOM_FSA4480_HEADSET)
/*was4780 reg*/
#define WAS4780_SWITCH_SETTINGS 0x04
#define WAS4780_SWITCH_CONTROL  0x05
#define WAS4780_SWITCH_STATUS1  0x07
#define WAS4780_SLOW_L          0x08
#define WAS4780_SLOW_R          0x09
#define WAS4780_SLOW_MIC        0x0A
#define WAS4780_SLOW_SENSE      0x0B
#define WAS4780_SLOW_GND        0x0C
#define WAS4780_DELAY_L_R       0x0D
#define WAS4780_DELAY_L_MIC     0x0E
#define WAS4780_DELAY_L_SENSE   0x0F
#define WAS4780_DELAY_L_AGND    0x10
#define WAS4780_FUN_EN		    0x12
#define WAS4780_DET_INT		    0x18
#define WAS4780_RESET           0x1E

#define FSA4480_CHIP_ID         0x00

#define FSA4480_OVP_INT_MASK 0x01
#define FSA4480_OVP_INT_FLAG  0x02
#define FSA4480_OVP_STATUS   0x03
#define FSA4480_SWITCH_STATUS0  0x06
#define FSA4480_AUDIO_ACCESSORY_STATUS  0x11
#define FSA4480_FUNCTION_ENABLE  0x12
#define FSA4480_RES_PIN_DETEC     0x13
#define FSA4480_RES_VALUE      0x14
#define FSA4480_RES_THRESHOLD   0x15
#define FSA4480_RES_DETECT_INTERVAL  0x16
#define FSA4480_AUDIO_JACK_STATUS  0x17
#define FSA4480_RES_INTERRUPT_FLAG    0x18
#define FSA4480_RES_INTERRUPT_MASK   0x19
#define FSA4480_AUDIO_REG1_VALUE  0x1A
#define FSA4480_AUDIO_REG2_VALUE  0x1B
#define FSA4480_MIC_DETECT_THRESOLD_DATA0  0x1C
#define FSA4480_MIC_DETECT_THRESOLD_DATA1  0x1D
#define FSA4480_VENDOR_NUM      0x09
#define ET7480_VENDOR_NUM       0x88
#define WAS4780_VENDOR_NUM      0x11
#define FSA4480_RES_THRESHOLD_VALUE  0xFE   // threshold 2560k

#define AS6480_MODE_CTRL              1
#define AS6480_SWITCH_USB             0
#define AS6480_SWITCH_HEADSET         2
#define AS6480_SWITCH_GND_MIC_SWAP    3
#define AS6480_SWITCH_OFF             7


int usb_other_status = 1; /*first probe, disable res check*/
int headset_status = HEADSET_UNPLUGED;
EXPORT_SYMBOL(headset_status);

static void fsa4480_update_reg_defaults(struct regmap *regmap);
#endif

#if IS_ENABLED(CONFIG_QCOM_FSA4480_WATER_DETECT)
extern void water_dry_open_usbhost(void);
extern void water_present_close_usbhost(void);
static void fsa4480_update_reg_irq(struct regmap *regmap, int det_pin);
#endif 

struct fsa4480_priv {
	struct regmap *regmap;
	struct device *dev;
	struct notifier_block ucsi_nb;
	atomic_t usbc_mode;
	struct work_struct usbc_analog_work;
	struct blocking_notifier_head fsa4480_notifier;
	struct mutex notification_lock;
#if IS_ENABLED(CONFIG_QCOM_FSA4480_HEADSET)
	int rst_gpio;
	int chip_id;
	struct regulator *reg_vdd;
	int power_enabled;
	int irq_gpio;
#endif
#if IS_ENABLED(CONFIG_QCOM_FSA4480_WATER_DETECT)
	struct delayed_work timer_work;
	struct workqueue_struct *work_queue;
	int det_pin;
	int register_value1;
	int register_value2;
	u32 water_status;
	int usb_status;
	int count_times;
	int no_water_count_times;
	struct wakeup_source *wakelock;
	int is_support_water_det;
#endif
};

struct fsa4480_reg_val {
	u16 reg;
	u8 val;
};

static const struct regmap_config fsa4480_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FSA4480_RESET,
};

static const struct fsa4480_reg_val fsa_reg_i2c_defaults[] = {
	{FSA4480_SLOW_L, 0x00},
	{FSA4480_SLOW_R, 0x00},
	{FSA4480_SLOW_MIC, 0x00},
	{FSA4480_SLOW_SENSE, 0x00},
	{FSA4480_SLOW_GND, 0x00},
	{FSA4480_DELAY_L_R, 0x00},
	{FSA4480_DELAY_L_MIC, 0x00},
	{FSA4480_DELAY_L_SENSE, 0x00},
	{FSA4480_DELAY_L_AGND, 0x09},
	{FSA4480_SWITCH_SETTINGS, 0x98},
#if IS_ENABLED(CONFIG_QCOM_FSA4480_WATER_DETECT)
	{FSA4480_SWITCH_CONTROL, 0x18},
#endif 
};

#if IS_ENABLED(CONFIG_QCOM_FSA4480_WATER_DETECT)
static const struct fsa4480_reg_val fsa_reg_read[] = {
	{FSA4480_OVP_STATUS,0x0},
	{FSA4480_SWITCH_SETTINGS, 0},
	{FSA4480_SWITCH_CONTROL, 0},
	{FSA4480_SWITCH_STATUS0,0},
	{FSA4480_SWITCH_STATUS1,0},
	{FSA4480_SLOW_L, 0x00},
	{FSA4480_SLOW_R, 0x00},
	{FSA4480_SLOW_MIC, 0x00},
	{FSA4480_SLOW_SENSE, 0x00},
	{FSA4480_SLOW_GND, 0x00},
	{FSA4480_DELAY_L_R, 0x00},
	{FSA4480_DELAY_L_MIC, 0x00},
	{FSA4480_DELAY_L_SENSE, 0x00},
	{FSA4480_DELAY_L_AGND, 0},
	{FSA4480_AUDIO_ACCESSORY_STATUS,0},
	{FSA4480_FUNCTION_ENABLE,0},
	{FSA4480_RES_PIN_DETEC,0},
	{FSA4480_RES_VALUE,0},
	{FSA4480_RES_THRESHOLD,0},
	{FSA4480_RES_DETECT_INTERVAL,0},
	{FSA4480_AUDIO_JACK_STATUS,0},
	{FSA4480_AUDIO_REG1_VALUE,0},
	{FSA4480_AUDIO_REG2_VALUE,0},
	{FSA4480_MIC_DETECT_THRESOLD_DATA0,0},
	{FSA4480_MIC_DETECT_THRESOLD_DATA1,0},	
};

static const struct fsa4480_reg_val fsa_reg_i2c_irq_enable[] = {
	{FSA4480_RES_DETECT_INTERVAL, 0x2},  //res 1s
	{FSA4480_RES_THRESHOLD, FSA4480_RES_THRESHOLD_VALUE},   // threshold 16k
	{FSA4480_RES_PIN_DETEC, 0x04},   //SUB2
	{FSA4480_FUNCTION_ENABLE, 0x62},  // res detection 10k~2560k
};

static const struct fsa4480_reg_val fsa_reg_i2c_irq_enable_sub1[] = {
	{FSA4480_RES_DETECT_INTERVAL, 0x2},  //res 1s
	{FSA4480_RES_THRESHOLD, FSA4480_RES_THRESHOLD_VALUE},   // threshold 16k
	{FSA4480_RES_PIN_DETEC, 0x03},   //SUB1
	{FSA4480_FUNCTION_ENABLE, 0x62},  // res detection 10k~2560k
};
#endif 

static void fsa4480_usbc_update_settings(struct fsa4480_priv *fsa_priv,
		u32 switch_control, u32 switch_enable)
{
	u32 prev_control, prev_enable;

	if (!fsa_priv->regmap) {
		dev_err(fsa_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}

	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, &prev_control);
	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, &prev_enable);
	if (prev_control == switch_control && prev_enable == switch_enable) {
		dev_dbg(fsa_priv->dev, "%s: settings unchanged\n", __func__);
		return;
	}

	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x80);
	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, switch_control);
	/* FSA4480 chip hardware requirement */
	usleep_range(50, 55);
	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, switch_enable);
}

static int fsa4480_usbc_event_changed(struct notifier_block *nb,
				      unsigned long evt, void *ptr)
{
	struct fsa4480_priv *fsa_priv =
			container_of(nb, struct fsa4480_priv, ucsi_nb);
	struct device *dev;
	enum typec_accessory acc = ((struct ucsi_glink_constat_info *)ptr)->acc;
#if IS_ENABLED(CONFIG_QCOM_FSA4480_HEADSET)
	bool partner_usb = ((struct ucsi_glink_constat_info *)ptr)->partner_usb;
	if(partner_usb && acc != TYPEC_ACCESSORY_AUDIO)
		acc = TYPEC_MAX_ACCESSORY;
#endif /*CONFIG_QCOM_FSA4480_WATER_DETECT*/

	if (!fsa_priv)
		return -EINVAL;

	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

	dev_dbg(dev, "%s: USB change event received, supply mode %d, usbc mode %ld, expected %d\n",
			__func__, acc, fsa_priv->usbc_mode.counter,
			TYPEC_ACCESSORY_AUDIO);

	switch (acc) {
	case TYPEC_ACCESSORY_AUDIO:
	case TYPEC_ACCESSORY_NONE:
		if (atomic_read(&(fsa_priv->usbc_mode)) == acc)
			break; /* filter notifications received before */
		atomic_set(&(fsa_priv->usbc_mode), acc);

		dev_dbg(dev, "%s: queueing usbc_analog_work\n",
			__func__);
		pm_stay_awake(fsa_priv->dev);
		queue_work(system_freezable_wq, &fsa_priv->usbc_analog_work);
		break;
	default:
#if IS_ENABLED(CONFIG_QCOM_FSA4480_HEADSET)
		if (atomic_read(&(fsa_priv->usbc_mode)) == acc)
			break; /* filter notifications received before */
		atomic_set(&(fsa_priv->usbc_mode), acc);

		dev_dbg(dev, "%s: queueing usbc_analog_work\n", __func__);
		pm_stay_awake(fsa_priv->dev);
		queue_work(system_freezable_wq, &fsa_priv->usbc_analog_work);
#endif
		break;
	}

	return 0;
}

static int fsa4480_usbc_analog_setup_switches(struct fsa4480_priv *fsa_priv)
{
	int rc = 0;
	int mode;
	struct device *dev;
#if IS_ENABLED(CONFIG_QCOM_FSA4480_WATER_DETECT)
	char event_string[128];
	char *envp[]={event_string,NULL};
#endif

	pr_err("%s: event changed.\n", __func__);

	if (!fsa_priv)
		return -EINVAL;
	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

	mutex_lock(&fsa_priv->notification_lock);
	/* get latest mode again within locked context */
	mode = atomic_read(&(fsa_priv->usbc_mode));

	dev_dbg(dev, "%s: mode=%d, headset_status=%d\n",__func__, mode, headset_status);

	switch (mode) {
	/* add all modes FSA should notify for in here */
	case TYPEC_ACCESSORY_AUDIO:
#if IS_ENABLED(CONFIG_QCOM_FSA4480_HEADSET)
		/*vendor modified*/
		dev_err(dev, "%s: audio device plug in \n", __func__);
		/* activate switches */
		if(headset_status == HEADSET_UNPLUGED)
		{
			headset_status = HEADSET_PLUGING;
			pr_info("%s:audio connecting\n", __func__);
		} else {
			pr_info("%s: audio had connected\n",__func__);
			mutex_unlock(&fsa_priv->notification_lock);
			return rc;
		}

		if((fsa_priv->chip_id == FSA4480_VENDOR_NUM)
				||(fsa_priv->chip_id ==ET7480_VENDOR_NUM)
				||(fsa_priv->chip_id ==WAS4780_VENDOR_NUM))
		{
			regmap_write(fsa_priv->regmap,FSA4480_RESET,0x01);
			pr_info("%s: audio	connected to reset chip\n",__func__);
			fsa4480_update_reg_defaults(fsa_priv->regmap);
#if IS_ENABLED(CONFIG_QCOM_FSA4480_WATER_DETECT)
			if(fsa_priv->is_support_water_det == 1) {
				regmap_write(fsa_priv->regmap,FSA4480_FUNCTION_ENABLE,0x08); //disable res dect
			}
			fsa_priv->count_times=0;
			fsa_priv->no_water_count_times = 0;
#endif
			fsa4480_usbc_update_settings(fsa_priv, 0x00, 0x9F);
		} else {
			rc =  regmap_write(fsa_priv->regmap, AS6480_MODE_CTRL, AS6480_SWITCH_OFF);
			rc |= regmap_write(fsa_priv->regmap,AS6480_MODE_CTRL, AS6480_SWITCH_HEADSET);			
		}

		/* notify call chain on event */
		dev_err(dev, "%s: mbhc typec audio notify", __func__);
		blocking_notifier_call_chain(&fsa_priv->fsa4480_notifier,
				mode, NULL);
#else /* CONFIG_QCOM_FSA4480_HEADSET */
		fsa4480_usbc_update_settings(fsa_priv, 0x00, 0x9F);/* activate switches */
		//WAS4780_autoset_switch(fsa_priv);
		dev_err(dev, "%s: mbhc typec audio notify", __func__);
		/* notify call chain on event */
		blocking_notifier_call_chain(&fsa_priv->fsa4480_notifier,
				mode, NULL);
#endif /* CONFIG_QCOM_FSA4480_HEADSET */
		break;

	case TYPEC_ACCESSORY_NONE:
#if IS_ENABLED(CONFIG_QCOM_FSA4480_HEADSET)
		headset_status = HEADSET_UNPLUGED;
		if(usb_other_status == 1){
			usb_other_status = 0;
			dev_err(dev, "%s: usb_other_status %d \n", __func__, usb_other_status);
		}
		/* notify call chain on event */
		blocking_notifier_call_chain(&fsa_priv->fsa4480_notifier,
				TYPEC_ACCESSORY_NONE, NULL);
		/*vendor modified*/
		dev_err(dev, "%s: plug out\n", __func__);

		/* deactivate switches */
		if((fsa_priv->chip_id == FSA4480_VENDOR_NUM)
				||(fsa_priv->chip_id ==ET7480_VENDOR_NUM)
				||(fsa_priv->chip_id ==WAS4780_VENDOR_NUM))
		{
			fsa4480_update_reg_defaults(fsa_priv->regmap);
			
#if IS_ENABLED(CONFIG_QCOM_FSA4480_WATER_DETECT)
			if(fsa_priv->is_support_water_det ==1) {
				fsa_priv->det_pin =2;
				fsa4480_update_reg_irq(fsa_priv->regmap,fsa_priv->det_pin);
			}
			fsa_priv->count_times=0;
			fsa_priv->no_water_count_times = 0;
#endif
			fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
		}else{
			rc =  regmap_write(fsa_priv->regmap, AS6480_MODE_CTRL, AS6480_SWITCH_OFF);
			rc |= regmap_write(fsa_priv->regmap,AS6480_MODE_CTRL, AS6480_SWITCH_USB);
		}
#else
		dev_err(dev, "%s: mbhc audio none notify", __func__);
		/* notify call chain on event */
		blocking_notifier_call_chain(&fsa_priv->fsa4480_notifier,
				TYPEC_ACCESSORY_NONE, NULL);

		/* deactivate switches */
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
#endif
		break;
	default:
#if IS_ENABLED(CONFIG_QCOM_FSA4480_HEADSET)
		headset_status = HEADSET_UNPLUGED;
#endif
		/* ignore other usb connection modes */
#if IS_ENABLED(CONFIG_QCOM_FSA4480_WATER_DETECT)
		fsa_priv->count_times = 0;
		fsa_priv->no_water_count_times = 0;
		fsa_priv->water_status = 0;
		if(fsa_priv->usb_status == 0) {
			pr_info("%s:send no water uevent2\n",__func__);
			sprintf(event_string,"waterproof EVENT=%ld",fsa_priv->water_status);
			kobject_uevent_env(&fsa_priv->dev->kobj,KOBJ_CHANGE,envp);
			water_dry_open_usbhost();
			fsa_priv->usb_status = 1;  //usb close-----open
		}
		if(usb_other_status == 0){
			usb_other_status = 1;
		  regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x1);	   
		  fsa4480_update_reg_defaults(fsa_priv->regmap);
		  regmap_write(fsa_priv->regmap,FSA4480_FUNCTION_ENABLE,0x08); //disable res dect
		  fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
		  dev_err(dev, "%s: usb_other_status %d \n", __func__,usb_other_status);
		}
#endif
		break;
	}

	mutex_unlock(&fsa_priv->notification_lock);
	return rc;
}

/*
 * fsa4480_reg_notifier - register notifier block with fsa driver
 *
 * @nb - notifier block of fsa4480
 * @node - phandle node to fsa4480 device
 *
 * Returns 0 on success, or error code
 */
int fsa4480_reg_notifier(struct notifier_block *nb,
			 struct device_node *node)
{
	int rc = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;

	rc = blocking_notifier_chain_register
				(&fsa_priv->fsa4480_notifier, nb);

	dev_dbg(fsa_priv->dev, "%s: registered notifier for %s\n",
		__func__, node->name);
	if (rc)
		return rc;

	/*
	 * as part of the init sequence check if there is a connected
	 * USB C analog adapter
	 */
	if (atomic_read(&(fsa_priv->usbc_mode)) == TYPEC_ACCESSORY_AUDIO) {
		dev_dbg(fsa_priv->dev, "%s: analog adapter already inserted\n",
			__func__);
		rc = fsa4480_usbc_analog_setup_switches(fsa_priv);
	}

	return rc;
}
EXPORT_SYMBOL(fsa4480_reg_notifier);

/*
 * fsa4480_unreg_notifier - unregister notifier block with fsa driver
 *
 * @nb - notifier block of fsa4480
 * @node - phandle node to fsa4480 device
 *
 * Returns 0 on pass, or error code
 */
int fsa4480_unreg_notifier(struct notifier_block *nb,
			     struct device_node *node)
{
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;

#if IS_ENABLED(CONFIG_QCOM_FSA4480_HEADSET)
	if((fsa_priv->chip_id == FSA4480_VENDOR_NUM)
			||(fsa_priv->chip_id ==ET7480_VENDOR_NUM)
			||(fsa_priv->chip_id ==WAS4780_VENDOR_NUM))
	{
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
	}else
	{
		regmap_write(fsa_priv->regmap, AS6480_MODE_CTRL, AS6480_SWITCH_OFF);		
	}
#else
	fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
#endif

	return blocking_notifier_chain_unregister
					(&fsa_priv->fsa4480_notifier, nb);
}
EXPORT_SYMBOL(fsa4480_unreg_notifier);

static int fsa4480_validate_display_port_settings(struct fsa4480_priv *fsa_priv)
{
	u32 switch_status = 0;

#if IS_ENABLED(CONFIG_QCOM_FSA4480_WATER_DETECT)
	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, &switch_status);
	pr_err("FSA4480_SWITCH_SETTINGS = %u\n",switch_status);
	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, &switch_status);
	pr_err("FSA4480_SWITCH_CONTROL = %u\n",switch_status);
	switch_status = 0;
#endif

	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS1, &switch_status);

	if ((switch_status != 0x23) && (switch_status != 0x1C)) {
		pr_err("AUX SBU1/2 switch status is invalid = %u\n",
				switch_status);
		return -EIO;
	}

	return 0;
}
/*
 * fsa4480_switch_event - configure FSA switch position based on event
 *
 * @node - phandle node to fsa4480 device
 * @event - fsa_function enum
 *
 * Returns int on whether the switch happened or not
 */
int fsa4480_switch_event(struct device_node *node,
			 enum fsa_function event)
{
	int switch_control = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;
#if IS_ENABLED(CONFIG_QCOM_FSA4480_HEADSET)
	int old_mode, new_mode = 0;
#endif

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;
	if (!fsa_priv->regmap)
		return -EINVAL;
	pr_info("%s: event=%d\n",__func__,event);
	switch (event) {
	case FSA_MIC_GND_SWAP:
#if IS_ENABLED(CONFIG_QCOM_FSA4480_HEADSET)
		if((fsa_priv->chip_id == FSA4480_VENDOR_NUM)
				||(fsa_priv->chip_id ==ET7480_VENDOR_NUM)
				||(fsa_priv->chip_id ==WAS4780_VENDOR_NUM))
		{
			regmap_read(fsa_priv->regmap, FSA4480_SWITCH_CONTROL,
					&switch_control);
			if ((switch_control & 0x07) == 0x07)
				switch_control = 0x0;
			else
				switch_control = 0x7;
			fsa4480_usbc_update_settings(fsa_priv, switch_control, 0x9F);

		}else {
			regmap_read(fsa_priv->regmap,AS6480_MODE_CTRL,&old_mode);
			new_mode = (old_mode == AS6480_SWITCH_HEADSET) ? AS6480_SWITCH_GND_MIC_SWAP : AS6480_SWITCH_HEADSET;
			regmap_write(fsa_priv->regmap,AS6480_MODE_CTRL, AS6480_SWITCH_OFF);
			regmap_write(fsa_priv->regmap,AS6480_MODE_CTRL, new_mode);			
		}
#else
		regmap_read(fsa_priv->regmap, FSA4480_SWITCH_CONTROL,
				&switch_control);
		if ((switch_control & 0x07) == 0x07)
			switch_control = 0x0;
		else
			switch_control = 0x7;
		fsa4480_usbc_update_settings(fsa_priv, switch_control, 0x9F);
#endif 
		pr_info("%s: swap mic and gnd\n",__func__);
		pr_info("%s: swap switch_control=%d\n",__func__, switch_control);
		break;
	case FSA_USBC_ORIENTATION_CC1:
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0xF8);
		return fsa4480_validate_display_port_settings(fsa_priv);
	case FSA_USBC_ORIENTATION_CC2:
		fsa4480_usbc_update_settings(fsa_priv, 0x78, 0xF8);
		return fsa4480_validate_display_port_settings(fsa_priv);
	case FSA_USBC_DISPLAYPORT_DISCONNECTED:
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL(fsa4480_switch_event);

static void fsa4480_usbc_analog_work_fn(struct work_struct *work)
{
	struct fsa4480_priv *fsa_priv =
		container_of(work, struct fsa4480_priv, usbc_analog_work);

	if (!fsa_priv) {
		pr_err("%s: fsa container invalid\n", __func__);
		return;
	}
	pr_info("%s: get into the function\n", __func__);
	fsa4480_usbc_analog_setup_switches(fsa_priv);
	pm_relax(fsa_priv->dev);
}

static void fsa4480_update_reg_defaults(struct regmap *regmap)
{
	u8 i;

	for (i = 0; i < ARRAY_SIZE(fsa_reg_i2c_defaults); i++)
		regmap_write(regmap, fsa_reg_i2c_defaults[i].reg,
				   fsa_reg_i2c_defaults[i].val);
}

#if IS_ENABLED(CONFIG_QCOM_FSA4480_WATER_DETECT)
static void fsa4480_update_reg_irq(struct regmap *regmap, int det_pin)
{
	u8 i;

	if(det_pin ==1) {   //detect sub1 res
		for (i = 0; i < ARRAY_SIZE(fsa_reg_i2c_irq_enable_sub1); i++)
			regmap_write(regmap, fsa_reg_i2c_irq_enable_sub1[i].reg,
					fsa_reg_i2c_irq_enable_sub1[i].val);		
	}else if(det_pin ==2) { // detect sub2 res
		for (i = 0; i < ARRAY_SIZE(fsa_reg_i2c_irq_enable); i++)
			regmap_write(regmap, fsa_reg_i2c_irq_enable[i].reg,
					fsa_reg_i2c_irq_enable[i].val);
	}else {
		pr_err("%s: det_pin set error\n",__func__);
	}
}

static ssize_t fsa4480_res_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fsa4480_priv *fsa_priv = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len,
			"resister value: %d K\n", fsa_priv->register_value2);

	return len;
}

static ssize_t fsa4480_water_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fsa4480_priv *fsa_priv = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len,
			"water status: %d\n", fsa_priv->water_status);

	return len;
}

static ssize_t fsa4480_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fsa4480_priv *fsa_priv = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint8_t i = 0;
	int reg_val = 0;

	for (i = 0; i < ARRAY_SIZE(fsa_reg_read); i++) {
		regmap_read(fsa_priv->regmap,fsa_reg_read[i].reg,&reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%04x\n", fsa_reg_read[i].reg, reg_val);
	}
	return len;
}

static ssize_t fsa4480_reg_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct fsa4480_priv *fsa_priv = dev_get_drvdata(dev);
	unsigned int databuf[2] = { 0 };

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1]))
		regmap_write(fsa_priv->regmap,databuf[0], databuf[1]);	

	return count;
}

static DEVICE_ATTR(res_value, S_IWUSR | S_IRUGO,
		fsa4480_res_show, NULL);
static DEVICE_ATTR(water_status, S_IWUSR | S_IRUGO,
		fsa4480_water_show, NULL);
static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO,
		fsa4480_reg_show,fsa4480_reg_store);

static struct attribute *fsa4480_attributes[] = {
	&dev_attr_res_value.attr,
	&dev_attr_water_status.attr,
	&dev_attr_reg.attr,
	NULL
};

static struct attribute_group fsa4480_attribute_group = {
	.attrs = fsa4480_attributes
};

static void fsa4480_enable_irq_mask(struct fsa4480_priv *fsa_priv)
{
	int reg_value;
	regmap_read(fsa_priv->regmap, FSA4480_RES_INTERRUPT_MASK ,&reg_value);		
	regmap_read(fsa_priv->regmap, FSA4480_RES_INTERRUPT_FLAG,&reg_value);	  
	regmap_write(fsa_priv->regmap, FSA4480_RES_INTERRUPT_MASK, 0x0);	
	regmap_write(fsa_priv->regmap, FSA4480_RES_INTERRUPT_FLAG, 0x0);	
	regmap_read(fsa_priv->regmap, FSA4480_RES_INTERRUPT_MASK ,&reg_value);		
	regmap_read(fsa_priv->regmap, FSA4480_RES_INTERRUPT_FLAG,&reg_value);
}

static void fsa4480_delay_work_func(struct work_struct *work)
{
	struct fsa4480_priv *fsa_priv = container_of(work,
			struct fsa4480_priv, timer_work.work);
	int value = 0;
	char event_string[128];
	char *envp[]={event_string,NULL};
	pr_info("%s: get in\n",__func__);
	if(fsa_priv == NULL)
	{
		pr_err("%s: can not get struct fsa_priv\n",__func__);
		return;
	}
	if (usb_other_status == 1) {
		fsa4480_update_reg_defaults(fsa_priv->regmap);
		regmap_write(fsa_priv->regmap,FSA4480_FUNCTION_ENABLE,0x08); //disable res dect
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
		pr_err("%s: had usb other exit\n",__func__);
		return;
	}
	if(fsa_priv->det_pin == 1) {
		regmap_read(fsa_priv->regmap, FSA4480_RES_VALUE, &value);
		fsa_priv->register_value1 = value;
		pr_info("%s: res value1 is %d\n",__func__,value);
		fsa_priv->det_pin =2;
	}else if(fsa_priv->det_pin == 2) {
		regmap_read(fsa_priv->regmap, FSA4480_RES_VALUE, &value);
		fsa_priv->register_value2 = value;
		pr_info("%s: res value2 is %d\n",__func__,value);
		fsa_priv->det_pin =1;
	}
	if((fsa_priv->register_value2 <= FSA4480_RES_THRESHOLD_VALUE)
			&& (fsa_priv->register_value2 > 1)
			&& (fsa_priv->register_value1 <= FSA4480_RES_THRESHOLD_VALUE)
			&& (fsa_priv->register_value1 > 1)) {
		if(fsa_priv->no_water_count_times)
			fsa_priv->no_water_count_times = 0;
		fsa_priv->count_times++;
		if(fsa_priv->count_times > 4) {
			fsa_priv->water_status = 1;
			if(fsa_priv->usb_status ==1) {
				pr_info("%s:send have water uevent\n",__func__);
				sprintf(event_string,"waterproof EVENT=%ld",fsa_priv->water_status);
				kobject_uevent_env(&fsa_priv->dev->kobj,KOBJ_CHANGE,envp);
				water_present_close_usbhost();
				pr_info("%s close usb host\n",__func__);
				fsa_priv->usb_status = 0;    //usb open-----close
			}
		}else {
			fsa_priv->water_status = 0;
		}
	} else {
		if(fsa_priv->count_times)
			fsa_priv->count_times = 0;
		if(fsa_priv->no_water_count_times < 10)
			fsa_priv->no_water_count_times++;
		if(fsa_priv->no_water_count_times > 4){
			fsa_priv->water_status = 0;
			if(fsa_priv->usb_status ==0)
			{
				pr_info("%s:send no water uevent2\n",__func__);
				sprintf(event_string,"waterproof EVENT=%ld",fsa_priv->water_status);
				kobject_uevent_env(&fsa_priv->dev->kobj,KOBJ_CHANGE,envp);
				water_dry_open_usbhost();
				pr_info("%s open usb host\n",__func__);
				fsa_priv->usb_status = 1;  //usb close-----open
			}
		}else
			fsa_priv->water_status = 1;
	}

	if((fsa_priv->register_value2 <= FSA4480_RES_THRESHOLD_VALUE) || (fsa_priv->register_value1 <= FSA4480_RES_THRESHOLD_VALUE)) {
		if(headset_status == HEADSET_UNPLUGED) {
			pr_info("%s: sub1<T || sub2<T, no headset schedule count=%d, no_water_count = %d\n",__func__,fsa_priv->count_times, fsa_priv->no_water_count_times);
			queue_delayed_work(fsa_priv->work_queue, &fsa_priv->timer_work,msecs_to_jiffies(3000));
		} else {
			fsa_priv->count_times = 0;
			fsa_priv->no_water_count_times = 0;
			pr_info("%s: sub1<T || sub2<T, have headset\n",__func__);
			return;
		}
	}
	value = gpio_get_value(fsa_priv->irq_gpio);
	pr_debug("%s: gpio value is %d\n",__func__,value);
	regmap_read(fsa_priv->regmap, FSA4480_FUNCTION_ENABLE, &value);
	pr_debug("%s: res function is %d\n",__func__,value);
	fsa4480_update_reg_irq(fsa_priv->regmap,fsa_priv->det_pin);
}

static irqreturn_t fsa4480_irq_handle(int irq, void *data)
{
	struct fsa4480_priv *fsa_priv = data;
	if(fsa_priv == NULL) {
		pr_err("%s: can not get struct fsa_priv\n",__func__);
		return IRQ_HANDLED;
	}
	fsa4480_enable_irq_mask(fsa_priv);      
	__pm_wakeup_event(fsa_priv->wakelock, msecs_to_jiffies(5000));
	queue_delayed_work(fsa_priv->work_queue, &fsa_priv->timer_work,0);
	pr_debug("%s: enter\n",__func__);
	return IRQ_HANDLED;
}
#endif

static int fsa4480_probe(struct i2c_client *i2c,
			 const struct i2c_device_id *id)
{
	struct fsa4480_priv *fsa_priv;
	int rc = 0;
#if IS_ENABLED(CONFIG_QCOM_FSA4480_WATER_DETECT)
	int water_dect=0;
	struct device_node *np = i2c->dev.of_node;
#endif 
#if IS_ENABLED(CONFIG_QCOM_FSA4480_HEADSET)
	int err;
	int chip_id=0;
	pr_info("%s: >>\n", __func__);
#endif

	fsa_priv = devm_kzalloc(&i2c->dev, sizeof(*fsa_priv),
				GFP_KERNEL);
	if (!fsa_priv)
		return -ENOMEM;

	fsa_priv->dev = &i2c->dev;

#if IS_ENABLED(CONFIG_QCOM_FSA4480_HEADSET)
	fsa_priv->reg_vdd = regulator_get(fsa_priv->dev, "vdd");
	if (IS_ERR(fsa_priv->reg_vdd)) {
		pr_err("%s:%d: Error getting vdd regulator\n", __func__, __LINE__);
	} else {
		pr_info("%s:%d: getting vdd regulator succeed\n", __func__, __LINE__);
		err = regulator_enable(fsa_priv->reg_vdd);
		if (err) {
			pr_err("%s:%d: vdd enable failed, error=%d\n", __func__, __LINE__, err);
		} else {
			pr_info("%s:%d: vdd enable succeed\n", __func__, __LINE__);
		}
	}

	fsa_priv->rst_gpio = of_get_named_gpio(fsa_priv->dev->of_node,"reset-gpio", 0);
	if (gpio_is_valid(fsa_priv->rst_gpio)) {
		rc = devm_gpio_request_one(fsa_priv->dev, fsa_priv->rst_gpio,GPIOF_OUT_INIT_LOW, "fsa4480_rst");
		if (rc){
			pr_err("%s: rst request failed\n", __func__);
		}else{
			pr_info("%s: rst request ok\n", __func__);
		}
	}else{
		pr_err("%s: rst gpio is not valid\n", __func__);
	}
#endif

#if IS_ENABLED(CONFIG_QCOM_FSA4480_WATER_DETECT)
	rc = of_property_read_u32(fsa_priv->dev->of_node,
			"qcom,support-water-decect", &water_dect);
	if (rc || water_dect == 0) {
		fsa_priv->is_support_water_det = 0;
		pr_info("%s, is not support water detect\n",__func__);
	} else {
		fsa_priv->is_support_water_det = 1;
		pr_info("%s, support water detect\n",__func__);	
	}
#endif

	fsa_priv->regmap = devm_regmap_init_i2c(i2c, &fsa4480_regmap_config);
	if (IS_ERR_OR_NULL(fsa_priv->regmap)) {
		dev_err(fsa_priv->dev, "%s: Failed to initialize regmap: %d\n",
			__func__, rc);
		if (!fsa_priv->regmap) {
			rc = -EINVAL;
			goto err_data;
		}
		rc = PTR_ERR(fsa_priv->regmap);
		goto err_data;
	}

#if IS_ENABLED(CONFIG_QCOM_FSA4480_HEADSET)
	regmap_read(fsa_priv->regmap, FSA4480_CHIP_ID ,&chip_id);
	pr_info("%s: chip_id is %d\n",__func__, chip_id);
	fsa_priv->chip_id = chip_id;
	i2c_set_clientdata(i2c, fsa_priv);
	if((chip_id == FSA4480_VENDOR_NUM) 
			||(chip_id == ET7480_VENDOR_NUM)
			||(chip_id == WAS4780_VENDOR_NUM)) {
		regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x1);		
		pr_info("%s,%d: reset the chip\n",__func__, __LINE__);
		fsa4480_update_reg_defaults(fsa_priv->regmap);
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);

#if IS_ENABLED(CONFIG_QCOM_FSA4480_WATER_DETECT)
		if(fsa_priv->is_support_water_det == 1) {
			fsa_priv->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
			if (fsa_priv->irq_gpio < 0)
				pr_info("%s: no irq gpio provided.\n", __func__);
			else
				pr_info("%s: irq gpio provided ok.\n", __func__);

			if (gpio_is_valid(fsa_priv->irq_gpio)) {
				rc = devm_gpio_request_one(&i2c->dev, fsa_priv->irq_gpio,GPIOF_DIR_IN, "fsa4480-irq");
				if (rc) {
					pr_err("%s: int gpio request failed\n",__func__);
				}

				fsa_priv->work_queue = create_singlethread_workqueue("fsa4480");
				if (fsa_priv->work_queue == NULL) {
					pr_err("%s:create workqueue failed!\n",__func__);
					return -EINVAL;
				}
				INIT_DELAYED_WORK(&fsa_priv->timer_work, fsa4480_delay_work_func);
				fsa_priv->wakelock = wakeup_source_register(NULL, "fsa4480");
				fsa_priv->water_status = 0;
				fsa_priv->usb_status =1;
				fsa_priv->det_pin = 2;
				fsa_priv->register_value1=0xff;
				fsa_priv->register_value2=0xff;

				rc = devm_request_threaded_irq(&i2c->dev,
						gpio_to_irq(fsa_priv->irq_gpio),
						NULL, fsa4480_irq_handle, IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
						"fsa4480-irq-name", fsa_priv);

				if(rc != 0) {
					pr_err("%s: int request failed\n",__func__);
				}else {
					enable_irq_wake(gpio_to_irq(fsa_priv->irq_gpio));
				}
				fsa4480_enable_irq_mask(fsa_priv);
				fsa4480_update_reg_irq(fsa_priv->regmap,fsa_priv->det_pin);
			}
		}
#endif /* CONFIG_QCOM_FSA4480_WATER_DETECT */
	}
#else /* CONFIG_QCOM_FSA4480_HEADSET */
	fsa4480_update_reg_defaults(fsa_priv->regmap);
#endif /* CONFIG_QCOM_FSA4480_HEADSET */

	devm_regmap_qti_debugfs_register(fsa_priv->dev, fsa_priv->regmap);

	fsa_priv->ucsi_nb.notifier_call = fsa4480_usbc_event_changed;
	fsa_priv->ucsi_nb.priority = 0;
	rc = register_ucsi_glink_notifier(&fsa_priv->ucsi_nb);
	if (rc) {
		dev_err(fsa_priv->dev, "%s: ucsi glink notifier registration failed: %d\n",
			__func__, rc);
		goto err_data;
	}

#if IS_ENABLED(CONFIG_QCOM_FSA4480_WATER_DETECT)
	rc = sysfs_create_group(&i2c->dev.kobj, &fsa4480_attribute_group);
	if (rc < 0) {
		pr_err("%s error creating sysfs attr files, err %d\n",__func__, rc);
	}
#endif

	mutex_init(&fsa_priv->notification_lock);
	i2c_set_clientdata(i2c, fsa_priv);

	INIT_WORK(&fsa_priv->usbc_analog_work,
		  fsa4480_usbc_analog_work_fn);

	BLOCKING_INIT_NOTIFIER_HEAD(&fsa_priv->fsa4480_notifier);

	return 0;

err_data:
	devm_kfree(&i2c->dev, fsa_priv);
	return rc;
}

static int fsa4480_remove(struct i2c_client *i2c)
{
	struct fsa4480_priv *fsa_priv =
			(struct fsa4480_priv *)i2c_get_clientdata(i2c);

	if (!fsa_priv)
		return -EINVAL;

	unregister_ucsi_glink_notifier(&fsa_priv->ucsi_nb);
	fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
	cancel_work_sync(&fsa_priv->usbc_analog_work);
	pm_relax(fsa_priv->dev);
	mutex_destroy(&fsa_priv->notification_lock);
#if IS_ENABLED(CONFIG_QCOM_FSA4480_WATER_DETECT)
	wakeup_source_unregister(fsa_priv->wakelock);
#endif
	dev_set_drvdata(&i2c->dev, NULL);

	return 0;
}

static const struct of_device_id fsa4480_i2c_dt_match[] = {
	{
		.compatible = "qcom,fsa4480-i2c",
	},
	{}
};

#if IS_ENABLED(CONFIG_QCOM_FSA4480_WATER_DETECT)
//static int suspend_times = 0;
static int fsa4480_suspend(struct device *device)
{
	//	int err = 0;
	//	int is_already_enabled = 0;
	struct i2c_client *client = to_i2c_client(device);
	struct fsa4480_priv *fsa_priv =
		(struct fsa4480_priv *)i2c_get_clientdata(client);

	pr_info("%s:enter\n",__func__);

	if (!fsa_priv)
		return -EINVAL;
	if(headset_status == HEADSET_UNPLUGED)
	{
		if((fsa_priv->is_support_water_det ==1) && (usb_other_status == 0))
		{
			pr_info("%s:enable usb water detect\n",__func__);
			fsa4480_enable_irq_mask(fsa_priv);
			fsa4480_update_reg_irq(fsa_priv->regmap,2);
		}
	}
#if 0  // use vph_power
	if (IS_ERR(fsa_priv->reg_vdd)) {
		pr_info("%s:%d:vdd not set\n",__func__, __LINE__);
	}else {
		is_already_enabled = regulator_is_enabled(fsa_priv->reg_vdd);
		pr_err("%s:%d: suspend regulator_enabled: %d, suspend_times:%d\n",
				__func__, __LINE__, is_already_enabled, suspend_times);

		err = regulator_disable(fsa_priv->reg_vdd);
		if (err) {
			pr_err("%s:%d:vdd disable failed, error=%d\n", __func__, __LINE__,err);
		} else {
			pr_info("%s:%d:vdd disable succeed\n",__func__, __LINE__);
		}		
	}
#endif
	return 0;
}

static int fsa4480_resume(struct device *device)
{
	//	int err = 0;
	//	int is_already_enabled = 0;
	//	int value;
	//	char event_string[128];
	//	char *envp[]={event_string,NULL};
	struct i2c_client *client = to_i2c_client(device);
	struct fsa4480_priv *fsa_priv =
		(struct fsa4480_priv *)i2c_get_clientdata(client);

	pr_info("%s:enter\n",__func__);

	if (!fsa_priv)
		return -EINVAL;
#if 0 // usb vph_power
	if (IS_ERR(fsa_priv->reg_vdd)) {
		pr_info("%s:%d:vdd not set\n",__func__, __LINE__);
	}else {
		is_already_enabled = regulator_is_enabled(fsa_priv->reg_vdd);
		pr_err("%s:%d: resume:regulator_enabled: %d, suspend_times:%d\n",
				__func__, __LINE__, is_already_enabled, suspend_times++);

		err = regulator_enable(fsa_priv->reg_vdd);
		if (err) {
			pr_err("%s:%d: vdd enable failed, error=%d\n", __func__, __LINE__, err);
		} else {
			pr_info("%s:%d: vdd enable succeed\n",__func__, __LINE__);
			if(is_already_enabled == 0) {
				mdelay(100);
			}
		}		
	}
#endif
	if(fsa_priv->is_support_water_det == 1) {
		if(fsa_priv->water_status == 1) {
			pr_info("%s:enable usb no water detect\n",__func__);
			queue_delayed_work(fsa_priv->work_queue, &fsa_priv->timer_work,msecs_to_jiffies(0));
#if 0
			fsa4480_update_reg_irq(fsa_priv->regmap, 1);
			mdelay(50);
			regmap_read(fsa_priv->regmap, FSA4480_RES_VALUE, &value);
			fsa_priv->register_value1 = value;
			fsa4480_update_reg_irq(fsa_priv->regmap, 2);
			mdelay(50);
			regmap_read(fsa_priv->regmap, FSA4480_RES_VALUE, &value);
			fsa_priv->register_value2 = value;
			pr_info("%s: sub1=%d,sub2=%d \n",__func__,fsa_priv->register_value1,fsa_priv->register_value2);
			if((fsa_priv->register_value2 > FSA4480_RES_THRESHOLD_VALUE) || (fsa_priv->register_value1 > FSA4480_RES_THRESHOLD_VALUE))
			{
				if(fsa_priv->usb_status ==0)
				{
					water_dry_open_usbhost();
					fsa_priv->usb_status=1;
					fsa_priv->water_status=0;
					pr_info("%s open usb\n",__func__);
					pr_info("%s:send no water uevent2\n",__func__);
					sprintf(event_string,"waterproof EVENT=%ld",fsa_priv->water_status);
					kobject_uevent_env(&fsa_priv->dev->kobj,KOBJ_CHANGE,envp);
				}
			}
#endif
		}
	}
	return 0;
}

static const struct dev_pm_ops fsa4480_pm_ops = {
	.suspend = fsa4480_suspend,
	.resume = fsa4480_resume,
};
#endif

static struct i2c_driver fsa4480_i2c_driver = {
	.driver = {
		.name = FSA4480_I2C_NAME,
		.of_match_table = fsa4480_i2c_dt_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
#if IS_ENABLED(CONFIG_QCOM_FSA4480_WATER_DETECT)
		.pm = &fsa4480_pm_ops,
#endif 
	},
	.probe = fsa4480_probe,
	.remove = fsa4480_remove,
};

static int __init fsa4480_init(void)
{
	int rc;

	rc = i2c_add_driver(&fsa4480_i2c_driver);
	if (rc)
		pr_err("fsa4480: Failed to register I2C driver: %d\n", rc);

	return rc;
}

module_init(fsa4480_init);


static void __exit fsa4480_exit(void)
{
	i2c_del_driver(&fsa4480_i2c_driver);
}
module_exit(fsa4480_exit);

MODULE_DESCRIPTION("FSA4480 I2C driver");
MODULE_LICENSE("GPL v2");
