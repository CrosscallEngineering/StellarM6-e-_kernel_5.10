// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define pr_fmt(fmt)	"BATTERY_CHG: %s: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/extcon-provider.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/rpmsg.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/soc/qcom/battery_charger.h>
#include <linux/soc/qcom/panel_event_notifier.h>
#include "qti_typec_class.h"
#ifdef CONFIG_CHARGE_FG_FUNCTION
#include <linux/his_debug_base.h>
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
#include <linux/usb/phy.h>
#include <linux/usb/ch9.h>

#define MSG_OWNER_BC			32778
#define MSG_TYPE_REQ_RESP		1
#define MSG_TYPE_NOTIFY			2

/* opcode for battery charger */
#define BC_SET_NOTIFY_REQ		0x04
#define BC_DISABLE_NOTIFY_REQ		0x05
#define BC_NOTIFY_IND			0x07
#define BC_BATTERY_STATUS_GET		0x30
#define BC_BATTERY_STATUS_SET		0x31
#define BC_USB_STATUS_GET		0x32
#define BC_USB_STATUS_SET		0x33
#define BC_WLS_STATUS_GET		0x34
#define BC_WLS_STATUS_SET		0x35
#define BC_SHIP_MODE_REQ_SET		0x36
#define BC_WLS_FW_CHECK_UPDATE		0x40
#define BC_WLS_FW_PUSH_BUF_REQ		0x41
#define BC_WLS_FW_UPDATE_STATUS_RESP	0x42
#define BC_WLS_FW_PUSH_BUF_RESP		0x43
#define BC_WLS_FW_GET_VERSION		0x44
#define BC_SHUTDOWN_NOTIFY		0x47
#define BC_HBOOST_VMAX_CLAMP_NOTIFY	0x79
#define BC_GENERIC_NOTIFY		0x80
#define BC_SET_USB_SSPHY_REQ        0x99

/* Generic definitions */
#define MAX_STR_LEN			128
#define BC_WAIT_TIME_MS			1000
#define WLS_FW_PREPARE_TIME_MS		1000
#define WLS_FW_WAIT_TIME_MS		500
#define WLS_FW_UPDATE_TIME_MS		1000
#define WLS_FW_BUF_SIZE			128
#define DEFAULT_RESTRICT_FCC_UA		1000000
#ifdef CONFIG_CHARGE_FG_FUNCTION
#define FAST_CHARGE_DISABLED_CURRENT_UA		1900000
#endif /*CONFIG_CHARGE_FG_FUNCTION*/

enum usb_connector_type {
	USB_CONNECTOR_TYPE_TYPEC,
	USB_CONNECTOR_TYPE_MICRO_USB,
};

enum psy_type {
	PSY_TYPE_BATTERY,
	PSY_TYPE_USB,
	PSY_TYPE_WLS,
	PSY_TYPE_MAX,
};

enum ship_mode_type {
	SHIP_MODE_PMIC,
	SHIP_MODE_PACK_SIDE,
};

/* property ids */
enum battery_property_id {
	BATT_STATUS,
	BATT_HEALTH,
	BATT_PRESENT,
	BATT_CHG_TYPE,
	BATT_CAPACITY,
	BATT_SOH,
	BATT_VOLT_OCV,
	BATT_VOLT_NOW,
	BATT_VOLT_MAX,
	BATT_CURR_NOW,
	BATT_CHG_CTRL_LIM,
	BATT_CHG_CTRL_LIM_MAX,
	BATT_TEMP,
	BATT_TECHNOLOGY,
	BATT_CHG_COUNTER,
	BATT_CYCLE_COUNT,
	BATT_CHG_FULL_DESIGN,
	BATT_CHG_FULL,
	BATT_MODEL_NAME,
	BATT_TTF_AVG,
	BATT_TTE_AVG,
	BATT_RESISTANCE,
	BATT_POWER_NOW,
	BATT_POWER_AVG,
#ifdef CONFIG_CHARGE_FG_FUNCTION
	BATT_INPUT_CURRENT_LIMIT,
	BATT_CONSTANT_CHARGE_CURRENT,
	BATT_CHARGING_ENABLED,
	BATT_SAFETY_TIMER_ENABLE,
	BATT_DEAD,
	BATT_CHARGING_STOPPED,
#endif/*CONFIG_CHARGE_FG_FUNCTION*/
	BATT_PROP_MAX,
};

enum usb_property_id {
	USB_ONLINE,
	USB_VOLT_NOW,
	USB_VOLT_MAX,
	USB_CURR_NOW,
	USB_CURR_MAX,
	USB_INPUT_CURR_LIMIT,
	USB_TYPE,
	USB_ADAP_TYPE,
	USB_MOISTURE_DET_EN,
	USB_MOISTURE_DET_STS,
	USB_TEMP,
	USB_REAL_TYPE,
	USB_TYPEC_COMPLIANT,
	USB_SCOPE,
	USB_CONNECTOR_TYPE,
	F_ACTIVE,
#ifdef CONFIG_CHARGE_FG_FUNCTION
	USB_CC_ORIENTATION,
	USB_PRESENT,
	USB_HOST_ENABLED,
	USB_DATA_ONLY_MODE,
#endif/*CONFIG_CHARGE_FG_FUNCTION*/
	USB_PROP_MAX,
};

enum wireless_property_id {
	WLS_ONLINE,
	WLS_VOLT_NOW,
	WLS_VOLT_MAX,
	WLS_CURR_NOW,
	WLS_CURR_MAX,
	WLS_TYPE,
	WLS_BOOST_EN,
	WLS_PROP_MAX,
};

enum {
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP = 0x80,
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3,
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5,
};

struct battery_charger_set_notify_msg {
	struct pmic_glink_hdr	hdr;
	u32			battery_id;
	u32			power_state;
	u32			low_capacity;
	u32			high_capacity;
};

struct battery_charger_notify_msg {
	struct pmic_glink_hdr	hdr;
	u32			notification;
};

struct battery_charger_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			battery_id;
	u32			property_id;
	u32			value;
};

struct battery_charger_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	u32			value;
	u32			ret_code;
};

struct battery_model_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	char			model[MAX_STR_LEN];
};

struct wireless_fw_check_req {
	struct pmic_glink_hdr	hdr;
	u32			fw_version;
	u32			fw_size;
	u32			fw_crc;
};

struct wireless_fw_check_resp {
	struct pmic_glink_hdr	hdr;
	u32			ret_code;
};

struct wireless_fw_push_buf_req {
	struct pmic_glink_hdr	hdr;
	u8			buf[WLS_FW_BUF_SIZE];
	u32			fw_chunk_id;
};

struct wireless_fw_push_buf_resp {
	struct pmic_glink_hdr	hdr;
	u32			fw_update_status;
};

struct wireless_fw_update_status {
	struct pmic_glink_hdr	hdr;
	u32			fw_update_done;
};

struct wireless_fw_get_version_req {
	struct pmic_glink_hdr	hdr;
};

struct wireless_fw_get_version_resp {
	struct pmic_glink_hdr	hdr;
	u32			fw_version;
};

struct battery_charger_ship_mode_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			ship_mode_type;
};

struct psy_state {
	struct power_supply	*psy;
	char			*model;
	const int		*map;
	u32			*prop;
	u32			prop_count;
	u32			opcode_get;
	u32			opcode_set;
};

struct battery_chg_dev {
	struct device			*dev;
	struct class			battery_class;
	struct pmic_glink_client	*client;
	struct typec_role_class		*typec_class;
	struct mutex			rw_lock;
	struct rw_semaphore		state_sem;
	struct completion		ack;
	struct completion		fw_buf_ack;
	struct completion		fw_update_ack;
	struct psy_state		psy_list[PSY_TYPE_MAX];
	struct dentry			*debugfs_dir;
	void				*notifier_cookie;
	/* extcon for VBUS/ID notification for USB for micro USB */
	struct extcon_dev		*extcon;
	u32				*thermal_levels;
	const char			*wls_fw_name;
	int				curr_thermal_level;
	int				num_thermal_levels;
	int				shutdown_volt_mv;
	atomic_t			state;
	struct work_struct		subsys_up_work;
	struct work_struct		usb_type_work;
	struct work_struct		battery_check_work;
#ifdef CONFIG_CHARGE_FG_FUNCTION
	struct delayed_work		qti_chg_status_work;
	struct wakeup_source	*qti_batt_chg_ws;
	bool				vbat_is_dead;
	bool				fast_charge_disabled;
	bool				moisture_present;
	bool				fast_charge_switch;
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
	int				fake_soc;
	bool				block_tx;
	bool				ship_mode_en;
	bool				debug_battery_detected;
	bool				wls_not_supported;
	bool				wls_fw_update_reqd;
	u32				wls_fw_version;
	u16				wls_fw_crc;
	u32				wls_fw_update_time_ms;
	struct notifier_block		reboot_notifier;
	u32				thermal_fcc_ua;
	u32				restrict_fcc_ua;
	u32				last_fcc_ua;
	u32				usb_icl_ua;
	u32				thermal_fcc_step;
	u32				connector_type;
	u32				usb_prev_mode;
	bool				restrict_chg_en;
	/* To track the driver initialization status */
	bool				initialized;
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
    struct power_supply             *sgm_chg_psy;
#endif /*CONFIG_DOUBLE_CHARGERS*/
	bool				notify_en;
	bool				error_prop;
	struct usb_phy *ssphy;
};
#ifdef CONFIG_CHARGE_FG_FUNCTION
struct charge_status{
	int				ibatt_now;
	int				vbatt_now;
	int				msoc;
	int				batt_temp;
	int				input_current_now;
	int				input_current_limit;
	int				usb_online;
	int				input_voltage_now;
	int				vbat_dead;
};
#endif /* CONFIG_CHARGE_FG_FUNCTION*/

static const int battery_prop_map[BATT_PROP_MAX] = {
	[BATT_STATUS]		= POWER_SUPPLY_PROP_STATUS,
	[BATT_HEALTH]		= POWER_SUPPLY_PROP_HEALTH,
	[BATT_PRESENT]		= POWER_SUPPLY_PROP_PRESENT,
	[BATT_CHG_TYPE]		= POWER_SUPPLY_PROP_CHARGE_TYPE,
	[BATT_CAPACITY]		= POWER_SUPPLY_PROP_CAPACITY,
	[BATT_VOLT_OCV]		= POWER_SUPPLY_PROP_VOLTAGE_OCV,
	[BATT_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[BATT_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[BATT_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[BATT_CHG_CTRL_LIM]	= POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	[BATT_CHG_CTRL_LIM_MAX]	= POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	[BATT_TEMP]		= POWER_SUPPLY_PROP_TEMP,
	[BATT_TECHNOLOGY]	= POWER_SUPPLY_PROP_TECHNOLOGY,
	[BATT_CHG_COUNTER]	= POWER_SUPPLY_PROP_CHARGE_COUNTER,
	[BATT_CYCLE_COUNT]	= POWER_SUPPLY_PROP_CYCLE_COUNT,
	[BATT_CHG_FULL_DESIGN]	= POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	[BATT_CHG_FULL]		= POWER_SUPPLY_PROP_CHARGE_FULL,
	[BATT_MODEL_NAME]	= POWER_SUPPLY_PROP_MODEL_NAME,
	[BATT_TTF_AVG]		= POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	[BATT_TTE_AVG]		= POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	[BATT_POWER_NOW]	= POWER_SUPPLY_PROP_POWER_NOW,
	[BATT_POWER_AVG]	= POWER_SUPPLY_PROP_POWER_AVG,
#ifdef CONFIG_CHARGE_FG_FUNCTION
	[BATT_INPUT_CURRENT_LIMIT]	= POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	[BATT_CONSTANT_CHARGE_CURRENT]	= POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
};

static const int usb_prop_map[USB_PROP_MAX] = {
	[USB_ONLINE]		= POWER_SUPPLY_PROP_ONLINE,
	[USB_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[USB_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[USB_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[USB_CURR_MAX]		= POWER_SUPPLY_PROP_CURRENT_MAX,
	[USB_INPUT_CURR_LIMIT]	= POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	[USB_ADAP_TYPE]		= POWER_SUPPLY_PROP_USB_TYPE,
	[USB_TEMP]		= POWER_SUPPLY_PROP_TEMP,
	[USB_SCOPE]		= POWER_SUPPLY_PROP_SCOPE,
#ifdef CONFIG_CHARGE_FG_FUNCTION
	[USB_PRESENT]		= POWER_SUPPLY_PROP_PRESENT,
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
};

static const int wls_prop_map[WLS_PROP_MAX] = {
	[WLS_ONLINE]		= POWER_SUPPLY_PROP_ONLINE,
	[WLS_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[WLS_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[WLS_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[WLS_CURR_MAX]		= POWER_SUPPLY_PROP_CURRENT_MAX,
};

static const unsigned int bcdev_usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

/* Standard usb_type definitions similar to power_supply_sysfs.c */
static const char * const power_supply_usb_type_text[] = {
	"Unknown", "SDP", "DCP", "CDP", "ACA", "C",
	"PD", "PD_DRP", "PD_PPS", "BrickID"
};

/* Custom usb_type definitions */
static const char * const qc_power_supply_usb_type_text[] = {
	"HVDCP", "HVDCP_3", "HVDCP_3P5"
};

#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
extern bool sgm_mag_charger_ok(void);
extern bool sgm_typec_charger_ok(void);
int battery_chg_check_typec_priority(struct battery_chg_dev * bcdev, bool * typec_priority);
extern void sgm_complete_apsd_done(void);
#endif /*CONFIG_DOUBLE_CHARGERS*/

static RAW_NOTIFIER_HEAD(hboost_notifier);

int register_hboost_event_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&hboost_notifier, nb);
}
EXPORT_SYMBOL(register_hboost_event_notifier);

int unregister_hboost_event_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_unregister(&hboost_notifier, nb);
}
EXPORT_SYMBOL(unregister_hboost_event_notifier);

static int battery_chg_fw_write(struct battery_chg_dev *bcdev, void *data,
				int len)
{
	int rc;

	down_read(&bcdev->state_sem);
	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		pr_debug("glink state is down\n");
		up_read(&bcdev->state_sem);
		return -ENOTCONN;
	}

	reinit_completion(&bcdev->fw_buf_ack);
	rc = pmic_glink_write(bcdev->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&bcdev->fw_buf_ack,
					msecs_to_jiffies(WLS_FW_WAIT_TIME_MS));
		if (!rc) {
			pr_err("Error, timed out sending message\n");
			up_read(&bcdev->state_sem);
			return -ETIMEDOUT;
		}

		rc = 0;
	}

	up_read(&bcdev->state_sem);
	return rc;
}

static int battery_chg_write(struct battery_chg_dev *bcdev, void *data,
				int len)
{
	int rc;

	/*
	 * When the subsystem goes down, it's better to return the last
	 * known values until it comes back up. Hence, return 0 so that
	 * pmic_glink_write() is not attempted until pmic glink is up.
	 */
	down_read(&bcdev->state_sem);
	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		pr_debug("glink state is down\n");
		up_read(&bcdev->state_sem);
		return 0;
	}

	if (bcdev->debug_battery_detected && bcdev->block_tx) {
		up_read(&bcdev->state_sem);
		return 0;
	}

	mutex_lock(&bcdev->rw_lock);
	reinit_completion(&bcdev->ack);
	bcdev->error_prop = false;
	rc = pmic_glink_write(bcdev->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&bcdev->ack,
					msecs_to_jiffies(BC_WAIT_TIME_MS));
		if (!rc) {
			pr_err("Error, timed out sending message\n");
			up_read(&bcdev->state_sem);
			mutex_unlock(&bcdev->rw_lock);
			return -ETIMEDOUT;
		}
		rc = 0;

		/*
		 * In case the opcode used is not supported, the remote
		 * processor might ack it immediately with a return code indicating
		 * an error. This additional check is to check if such an error has
		 * happened and return immediately with error in that case. This
		 * avoids wasting time waiting in the above timeout condition for this
		 * type of error.
		 */
		if (bcdev->error_prop) {
			bcdev->error_prop = false;
			rc = -ENODATA;
		}
	}
	mutex_unlock(&bcdev->rw_lock);
	up_read(&bcdev->state_sem);

	return rc;
}

static int write_property_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id, u32 val)
{
	struct battery_charger_req_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.battery_id = 0;
	req_msg.value = val;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_set;

	if (pst->psy)
		pr_debug("psy: %s prop_id: %u val: %u\n", pst->psy->desc->name,
			req_msg.property_id, val);

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static int read_property_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id)
{
	struct battery_charger_req_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.battery_id = 0;
	req_msg.value = 0;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;

	if (pst->psy)
		pr_debug("psy: %s prop_id: %u\n", pst->psy->desc->name,
			req_msg.property_id);

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static int get_property_id(struct psy_state *pst,
			enum power_supply_property prop)
{
	u32 i;

	for (i = 0; i < pst->prop_count; i++)
		if (pst->map[i] == prop)
			return i;

	if (pst->psy)
		pr_err("No property id for property %d in psy %s\n", prop,
			pst->psy->desc->name);

	return -ENOENT;
}

static void battery_chg_notify_disable(struct battery_chg_dev *bcdev)
{
	struct battery_charger_set_notify_msg req_msg = { { 0 } };
	int rc;

	if (bcdev->notify_en) {
		/* Send request to disable notification */
		req_msg.hdr.owner = MSG_OWNER_BC;
		req_msg.hdr.type = MSG_TYPE_NOTIFY;
		req_msg.hdr.opcode = BC_DISABLE_NOTIFY_REQ;

		rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
		if (rc < 0)
			pr_err("Failed to disable notification rc=%d\n", rc);
		else
			bcdev->notify_en = false;
	}
}

static void battery_chg_notify_enable(struct battery_chg_dev *bcdev)
{
	struct battery_charger_set_notify_msg req_msg = { { 0 } };
	int rc;

	if (!bcdev->notify_en) {
		/* Send request to enable notification */
		req_msg.hdr.owner = MSG_OWNER_BC;
		req_msg.hdr.type = MSG_TYPE_NOTIFY;
		req_msg.hdr.opcode = BC_SET_NOTIFY_REQ;

		rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
		if (rc < 0)
			pr_err("Failed to enable notification rc=%d\n", rc);
		else
			bcdev->notify_en = true;
	}
}

static void battery_chg_state_cb(void *priv, enum pmic_glink_state state)
{
	struct battery_chg_dev *bcdev = priv;

	pr_debug("state: %d\n", state);

	down_write(&bcdev->state_sem);
	if (!bcdev->initialized) {
		pr_warn("Driver not initialized, pmic_glink state %d\n", state);
		up_write(&bcdev->state_sem);
		return;
	}
	atomic_set(&bcdev->state, state);
	up_write(&bcdev->state_sem);

	if (state == PMIC_GLINK_STATE_UP)
		schedule_work(&bcdev->subsys_up_work);
	else if (state == PMIC_GLINK_STATE_DOWN)
		bcdev->notify_en = false;
}

/**
 * qti_battery_charger_get_prop() - Gets the property being requested
 *
 * @name: Power supply name
 * @prop_id: Property id to be read
 * @val: Pointer to value that needs to be updated
 *
 * Return: 0 if success, negative on error.
 */
int qti_battery_charger_get_prop(const char *name,
				enum battery_charger_prop prop_id, int *val)
{
	struct power_supply *psy;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst;
	int rc = 0;

	if (prop_id >= BATTERY_CHARGER_PROP_MAX)
		return -EINVAL;

	if (strcmp(name, "battery") && strcmp(name, "usb") &&
	    strcmp(name, "wireless"))
		return -EINVAL;

	psy = power_supply_get_by_name(name);
	if (!psy)
		return -ENODEV;

	bcdev = power_supply_get_drvdata(psy);
	power_supply_put(psy);
	if (!bcdev)
		return -ENODEV;

	switch (prop_id) {
	case BATTERY_RESISTANCE:
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		rc = read_property_id(bcdev, pst, BATT_RESISTANCE);
		if (!rc)
			*val = pst->prop[BATT_RESISTANCE];
		break;
	default:
		break;
	}

	return rc;
}
EXPORT_SYMBOL(qti_battery_charger_get_prop);

int qti_battery_charger_set_prop(const char *name,
				enum battery_charger_prop prop_id, int val)
{
	struct power_supply *psy;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst;
	int rc = 0;

	if (prop_id >= BATTERY_CHARGER_PROP_MAX)
		return -EINVAL;

	if (strcmp(name, "battery") && strcmp(name, "usb") &&
	    strcmp(name, "wireless"))
		return -EINVAL;

	psy = power_supply_get_by_name(name);
	if (!psy)
		return -ENODEV;

	bcdev = power_supply_get_drvdata(psy);
	power_supply_put(psy);
	if (!bcdev)
		return -ENODEV;

	switch (prop_id) {
	case FLASH_ACTIVE:
		pst = &bcdev->psy_list[PSY_TYPE_USB];
		rc = write_property_id(bcdev, pst, F_ACTIVE, val);
		break;
	default:
		break;
	}

	return rc;
}
EXPORT_SYMBOL(qti_battery_charger_set_prop);

static bool validate_message(struct battery_chg_dev *bcdev,
			struct battery_charger_resp_msg *resp_msg, size_t len)
{
	if (len != sizeof(*resp_msg)) {
		pr_err("Incorrect response length %zu for opcode %#x\n", len,
			resp_msg->hdr.opcode);
		return false;
	}

	if (resp_msg->ret_code) {
		pr_err_ratelimited("Error in response for opcode %#x prop_id %u, rc=%d\n",
			resp_msg->hdr.opcode, resp_msg->property_id,
			(int)resp_msg->ret_code);
		bcdev->error_prop = true;
		return false;
	}

	return true;
}

#define MODEL_DEBUG_BOARD	"Debug_Board"
static void handle_message(struct battery_chg_dev *bcdev, void *data,
				size_t len)
{
	struct battery_charger_resp_msg *resp_msg = data;
	struct battery_model_resp_msg *model_resp_msg = data;
	struct wireless_fw_check_resp *fw_check_msg;
	struct wireless_fw_push_buf_resp *fw_resp_msg;
	struct wireless_fw_update_status *fw_update_msg;
	struct wireless_fw_get_version_resp *fw_ver_msg;
	struct psy_state *pst;
	bool ack_set = false;

	switch (resp_msg->hdr.opcode) {
	case BC_BATTERY_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

		/* Handle model response uniquely as it's a string */
		if (pst->model && len == sizeof(*model_resp_msg)) {
			memcpy(pst->model, model_resp_msg->model, MAX_STR_LEN);
			ack_set = true;
			bcdev->debug_battery_detected = !strcmp(pst->model,
					MODEL_DEBUG_BOARD);
			break;
		}

		/* Other response should be of same type as they've u32 value */
		if (validate_message(bcdev, resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
	case BC_USB_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_USB];
		if (validate_message(bcdev, resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
	case BC_WLS_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_WLS];
		if (validate_message(bcdev, resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
	case BC_BATTERY_STATUS_SET:
	case BC_USB_STATUS_SET:
	case BC_WLS_STATUS_SET:
		if (validate_message(bcdev, data, len))
			ack_set = true;

		break;
	case BC_SET_NOTIFY_REQ:
	case BC_DISABLE_NOTIFY_REQ:
	case BC_SHUTDOWN_NOTIFY:
	case BC_SHIP_MODE_REQ_SET:
		/* Always ACK response for notify or ship_mode request */
		ack_set = true;
		break;
	case BC_WLS_FW_CHECK_UPDATE:
		if (len == sizeof(*fw_check_msg)) {
			fw_check_msg = data;
			if (fw_check_msg->ret_code == 1)
				bcdev->wls_fw_update_reqd = true;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for wls_fw_check_update\n",
				len);
		}
		break;
	case BC_WLS_FW_PUSH_BUF_RESP:
		if (len == sizeof(*fw_resp_msg)) {
			fw_resp_msg = data;
			if (fw_resp_msg->fw_update_status == 1)
				complete(&bcdev->fw_buf_ack);
		} else {
			pr_err("Incorrect response length %zu for wls_fw_push_buf_resp\n",
				len);
		}
		break;
	case BC_WLS_FW_UPDATE_STATUS_RESP:
		if (len == sizeof(*fw_update_msg)) {
			fw_update_msg = data;
			if (fw_update_msg->fw_update_done == 1)
				complete(&bcdev->fw_update_ack);
			else
				pr_err("Wireless FW update not done %d\n",
					(int)fw_update_msg->fw_update_done);
		} else {
			pr_err("Incorrect response length %zu for wls_fw_update_status_resp\n",
				len);
		}
		break;
	case BC_WLS_FW_GET_VERSION:
		if (len == sizeof(*fw_ver_msg)) {
			fw_ver_msg = data;
			bcdev->wls_fw_version = fw_ver_msg->fw_version;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for wls_fw_get_version\n",
				len);
		}
		break;
	default:
		pr_err("Unknown opcode: %u\n", resp_msg->hdr.opcode);
		break;
	}

	if (ack_set || bcdev->error_prop)
		complete(&bcdev->ack);
}

static void battery_chg_update_uusb_type(struct battery_chg_dev *bcdev,
					 u32 adap_type)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	/* Handle the extcon notification for uUSB case only */
	if (bcdev->connector_type != USB_CONNECTOR_TYPE_MICRO_USB)
		return;

	rc = read_property_id(bcdev, pst, USB_SCOPE);
	if (rc < 0) {
		pr_err("Failed to read USB_SCOPE rc=%d\n", rc);
		return;
	}

	switch (pst->prop[USB_SCOPE]) {
	case POWER_SUPPLY_SCOPE_DEVICE:
		if (adap_type == POWER_SUPPLY_USB_TYPE_SDP ||
		    adap_type == POWER_SUPPLY_USB_TYPE_CDP) {
			/* Device mode connect notification */
			extcon_set_state_sync(bcdev->extcon, EXTCON_USB, 1);
			bcdev->usb_prev_mode = EXTCON_USB;
			rc = qti_typec_partner_register(bcdev->typec_class,
							TYPEC_DEVICE);
			if (rc < 0)
				pr_err("Failed to register typec partner rc=%d\n",
					rc);
		}
		break;
	case POWER_SUPPLY_SCOPE_SYSTEM:
		/* Host mode connect notification */
		extcon_set_state_sync(bcdev->extcon, EXTCON_USB_HOST, 1);
		bcdev->usb_prev_mode = EXTCON_USB_HOST;
		rc = qti_typec_partner_register(bcdev->typec_class, TYPEC_HOST);
		if (rc < 0)
			pr_err("Failed to register typec partner rc=%d\n",
				rc);
		break;
	default:
		if (bcdev->usb_prev_mode == EXTCON_USB ||
		    bcdev->usb_prev_mode == EXTCON_USB_HOST) {
			/* Disconnect notification */
			extcon_set_state_sync(bcdev->extcon,
					      bcdev->usb_prev_mode, 0);
			bcdev->usb_prev_mode = EXTCON_NONE;
			qti_typec_partner_unregister(bcdev->typec_class);
		}
		break;
	}
}
#ifdef CONFIG_CHARGE_FG_FUNCTION
struct power_supply_desc usb_psy_desc;
EXPORT_SYMBOL_GPL(usb_psy_desc);
#else  /*CONFIG_CHARGE_FG_FUNCTION*/
static struct power_supply_desc usb_psy_desc;
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
static void battery_chg_update_usb_type_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev, usb_type_work);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;
#ifdef CONFIG_CHARGE_FG_FUNCTION
	/*Send uevent to notify BatteryServices that quick charger has beed detected.*/
	char *quick_charger[2] = {"QUICK_CHARGER=YES", NULL};
	bool fastcharge_flag = false;
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
	union power_supply_propval val = {0,};
#endif /*CONFIG_DOUBLE_CHARGERS*/
	rc = read_property_id(bcdev, pst, USB_ADAP_TYPE);
	if (rc < 0) {
		pr_err("Failed to read USB_ADAP_TYPE rc=%d\n", rc);
		return;
	}

	/* Reset usb_icl_ua whenever USB adapter type changes */
	if (pst->prop[USB_ADAP_TYPE] != POWER_SUPPLY_USB_TYPE_SDP &&
	    pst->prop[USB_ADAP_TYPE] != POWER_SUPPLY_USB_TYPE_PD)
		bcdev->usb_icl_ua = 0;

#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
	if(pst->prop[USB_ADAP_TYPE] != POWER_SUPPLY_USB_TYPE_UNKNOWN)
		sgm_complete_apsd_done();

#endif /*CONFIG_DOUBLE_CHARGERS*/
	pr_err("usb_adap_type: %u\n", pst->prop[USB_ADAP_TYPE]);

	switch (pst->prop[USB_ADAP_TYPE]) {
	case POWER_SUPPLY_USB_TYPE_SDP:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB;
		break;
	case POWER_SUPPLY_USB_TYPE_DCP:
#ifdef CONFIG_CHARGE_FG_FUNCTION
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
		rc = read_property_id(bcdev, pst, USB_REAL_TYPE);
		if(rc < 0)
			pr_err("Failed to read USB_REAL_TYPE rc=%d\n", rc);
		else{
			pr_err("usb_real_type: %u\n", pst->prop[USB_REAL_TYPE]);
			switch(pst->prop[USB_REAL_TYPE]){
				case QTI_POWER_SUPPLY_USB_TYPE_HVDCP:
				case QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3:
				case QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5:
					fastcharge_flag = true;
					break;
				default:
					break;
			}
		}
		break;
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
	case POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID:
#ifdef CONFIG_CHARGE_FG_FUNCTION
		usb_psy_desc.type = POWER_SUPPLY_TYPE_APPLE_BRICK_ID;
		break;
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
	case QTI_POWER_SUPPLY_USB_TYPE_HVDCP:
	case QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3:
	case QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
#ifdef CONFIG_CHARGE_FG_FUNCTION
		fastcharge_flag = true;
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
		break;
	case POWER_SUPPLY_USB_TYPE_CDP:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case POWER_SUPPLY_USB_TYPE_ACA:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	case POWER_SUPPLY_USB_TYPE_C:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_TYPE_C;
		break;
	case POWER_SUPPLY_USB_TYPE_PD_DRP:
#ifdef CONFIG_CHARGE_FG_FUNCTION
		fastcharge_flag = true;
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_PD_DRP;
		break;
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
	case POWER_SUPPLY_USB_TYPE_PD:
	case POWER_SUPPLY_USB_TYPE_PD_PPS:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_PD;
#ifdef CONFIG_CHARGE_FG_FUNCTION
		fastcharge_flag = true;
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
		break;
	default:
#ifdef CONFIG_CHARGE_FG_FUNCTION
		usb_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
#else /*CONFIG_CHARGE_FG_FUNCTION*/
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB;
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
		if(!bcdev->sgm_chg_psy){
			bcdev->sgm_chg_psy= power_supply_get_by_name("sgm_charger");
			if (!bcdev->sgm_chg_psy) {
				pr_err("Couldn't get sgm_chg_psy power_supply\n");
				return;
			}
		}
		rc = power_supply_get_property(bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_ONLINE, &val);
		if(rc < 0){
			pr_err("Couldn't get sgm_chg_psy online\n");
			return;
		}
		if(val.intval){
			rc = power_supply_get_property(bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_TYPE, &val);
			if(rc < 0){
				pr_err("Couldn't get sgm_chg_psy type\n");
				return;
			}
			usb_psy_desc.type = val.intval;
		}
#endif /*CONFIG_DOUBLE_CHARGERS*/
		break;
	}
#ifdef CONFIG_CHARGE_FG_FUNCTION
	if(fastcharge_flag && bcdev->fast_charge_switch){
		pr_err("%s quick_charger\n",__func__);
		kobject_uevent_env(&bcdev->dev->kobj, KOBJ_CHANGE, quick_charger);
	}
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
	battery_chg_update_uusb_type(bcdev, pst->prop[USB_ADAP_TYPE]);
}

static void battery_chg_check_status_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev,
					battery_check_work);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int rc;

	rc = read_property_id(bcdev, pst, BATT_STATUS);
	if (rc < 0) {
		pr_err("Failed to read BATT_STATUS, rc=%d\n", rc);
		return;
	}

	if (pst->prop[BATT_STATUS] == POWER_SUPPLY_STATUS_CHARGING) {
		pr_debug("Battery is charging\n");
		return;
	}

	rc = read_property_id(bcdev, pst, BATT_CAPACITY);
	if (rc < 0) {
		pr_err("Failed to read BATT_CAPACITY, rc=%d\n", rc);
		return;
	}

	if (DIV_ROUND_CLOSEST(pst->prop[BATT_CAPACITY], 100) > 0) {
		pr_debug("Battery SOC is > 0\n");
		return;
	}

	/*
	 * If we are here, then battery is not charging and SOC is 0.
	 * Check the battery voltage and if it's lower than shutdown voltage,
	 * then initiate an emergency shutdown.
	 */

	rc = read_property_id(bcdev, pst, BATT_VOLT_NOW);
	if (rc < 0) {
		pr_err("Failed to read BATT_VOLT_NOW, rc=%d\n", rc);
		return;
	}

	if (pst->prop[BATT_VOLT_NOW] / 1000 > bcdev->shutdown_volt_mv) {
		pr_debug("Battery voltage is > %d mV\n",
			bcdev->shutdown_volt_mv);
		return;
	}

	pr_emerg("Initiating a shutdown in 100 ms\n");
	msleep(100);
	pr_emerg("Attempting kernel_power_off: Battery voltage low\n");
	kernel_power_off();
}

static void handle_notification(struct battery_chg_dev *bcdev, void *data,
				size_t len)
{
	struct battery_charger_notify_msg *notify_msg = data;
	struct psy_state *pst = NULL;
	u32 hboost_vmax_mv, notification;

	if (len != sizeof(*notify_msg)) {
		pr_err("Incorrect response length %zu\n", len);
		return;
	}

	notification = notify_msg->notification;
	pr_debug("notification: %#x\n", notification);
	if ((notification & 0xffff) == BC_HBOOST_VMAX_CLAMP_NOTIFY) {
		hboost_vmax_mv = (notification >> 16) & 0xffff;
		raw_notifier_call_chain(&hboost_notifier, VMAX_CLAMP, &hboost_vmax_mv);
		pr_debug("hBoost is clamped at %u mV\n", hboost_vmax_mv);
		return;
	}

	switch (notification) {
	case BC_BATTERY_STATUS_GET:
#ifdef CONFIG_CHARGE_FG_FUNCTION
		cancel_delayed_work_sync(&bcdev->qti_chg_status_work);
		schedule_delayed_work(&bcdev->qti_chg_status_work, msecs_to_jiffies(2 * BC_WAIT_TIME_MS));
		break;
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
	case BC_GENERIC_NOTIFY:
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		if (bcdev->shutdown_volt_mv > 0)
			schedule_work(&bcdev->battery_check_work);
		break;
	case BC_USB_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_USB];
		schedule_work(&bcdev->usb_type_work);
		break;
	case BC_WLS_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_WLS];
		break;
	case BC_SET_USB_SSPHY_REQ:
		pr_info("Disable SSPHY\n");
		usb_phy_notify_disconnect(bcdev->ssphy, USB_SPEED_SUPER);
		return;
	default:
		break;
	}

	if (pst && pst->psy) {
		/*
		 * For charger mode, keep the device awake at least for 50 ms
		 * so that device won't enter suspend when a non-SDP charger
		 * is removed. This would allow the userspace process like
		 * "charger" to be able to read power supply uevents to take
		 * appropriate actions (e.g. shutting down when the charger is
		 * unplugged).
		 */
		power_supply_changed(pst->psy);
		pm_wakeup_dev_event(bcdev->dev, 50, true);
	}
}

static int battery_chg_callback(void *priv, void *data, size_t len)
{
	struct pmic_glink_hdr *hdr = data;
	struct battery_chg_dev *bcdev = priv;

	pr_debug("owner: %u type: %u opcode: %#x len: %zu\n", hdr->owner,
		hdr->type, hdr->opcode, len);

	down_read(&bcdev->state_sem);

	if (!bcdev->initialized) {
		pr_debug("Driver initialization failed: Dropping glink callback message: state %d\n",
			 bcdev->state);
		up_read(&bcdev->state_sem);
		return 0;
	}

	if (hdr->opcode == BC_NOTIFY_IND)
		handle_notification(bcdev, data, len);
	else
		handle_message(bcdev, data, len);

	up_read(&bcdev->state_sem);

	return 0;
}

static int wls_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_WLS];
	int prop_id, rc;

	pval->intval = -ENODATA;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	pval->intval = pst->prop[prop_id];

	return 0;
}

static int wls_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	return 0;
}

static int wls_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	return 0;
}

static enum power_supply_property wls_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static const struct power_supply_desc wls_psy_desc = {
	.name			= "wireless",
	.type			= POWER_SUPPLY_TYPE_WIRELESS,
	.properties		= wls_props,
	.num_properties		= ARRAY_SIZE(wls_props),
	.get_property		= wls_psy_get_prop,
	.set_property		= wls_psy_set_prop,
	.property_is_writeable	= wls_psy_prop_is_writeable,
};

static const char *get_usb_type_name(u32 usb_type)
{
	u32 i;

	if (usb_type >= QTI_POWER_SUPPLY_USB_TYPE_HVDCP &&
	    usb_type <= QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5) {
		for (i = 0; i < ARRAY_SIZE(qc_power_supply_usb_type_text);
		     i++) {
			if (i == (usb_type - QTI_POWER_SUPPLY_USB_TYPE_HVDCP))
				return qc_power_supply_usb_type_text[i];
		}
		return "Unknown";
	}

	for (i = 0; i < ARRAY_SIZE(power_supply_usb_type_text); i++) {
		if (i == usb_type)
			return power_supply_usb_type_text[i];
	}

	return "Unknown";
}

static int usb_psy_set_icl(struct battery_chg_dev *bcdev, u32 prop_id, int val)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	u32 temp;
	int rc;

	rc = read_property_id(bcdev, pst, USB_ADAP_TYPE);
	if (rc < 0) {
		pr_err("Failed to read prop USB_ADAP_TYPE, rc=%d\n", rc);
		return rc;
	}

	/* Allow this only for SDP, CDP or USB_PD and not for other charger types */
	switch (pst->prop[USB_ADAP_TYPE]) {
	case POWER_SUPPLY_USB_TYPE_SDP:
	case POWER_SUPPLY_USB_TYPE_PD:
	case POWER_SUPPLY_USB_TYPE_CDP:
		break;
	default:
		return -EINVAL;
	}

	/*
	 * Input current limit (ICL) can be set by different clients. E.g. USB
	 * driver can request for a current of 500/900 mA depending on the
	 * port type. Also, clients like EUD driver can pass 0 or -22 to
	 * suspend or unsuspend the input for its use case.
	 */

	temp = val;
	if (val < 0)
		temp = UINT_MAX;

	rc = write_property_id(bcdev, pst, prop_id, temp);
	if (rc < 0) {
		pr_err("Failed to set ICL (%u uA) rc=%d\n", temp, rc);
	} else {
		pr_debug("Set ICL to %u\n", temp);
		bcdev->usb_icl_ua = temp;
	}

	return rc;
}

static int usb_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int prop_id, rc;

	pval->intval = -ENODATA;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	pval->intval = pst->prop[prop_id];
	if (prop == POWER_SUPPLY_PROP_TEMP)
		pval->intval = DIV_ROUND_CLOSEST((int)pval->intval, 10);

#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
	if (prop == POWER_SUPPLY_PROP_ONLINE && !pval->intval){
		if(!bcdev->sgm_chg_psy){
			bcdev->sgm_chg_psy = power_supply_get_by_name("sgm_charger");
			if(!bcdev->sgm_chg_psy){
				dev_err(bcdev->dev, "Could not get sgm_charger power_supply\n");
				return -ENODEV;
			}
		}
		rc = power_supply_get_property(bcdev->sgm_chg_psy, prop, pval);
		if(rc < 0){
			pr_err("Failed to get  usb online from sgm_chg_psy rc=%d.\n", rc);
			return rc;
		}
	}
#endif /*CONFIG_DOUBLE_CHARGERS*/
	return 0;
}

static int usb_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int prop_id, rc = 0;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		rc = usb_psy_set_icl(bcdev, prop_id, pval->intval);
		break;
	default:
		break;
	}

	return rc;
}

static int usb_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_SCOPE,
#ifdef CONFIG_CHARGE_FG_FUNCTION
	POWER_SUPPLY_PROP_PRESENT,
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
};

static enum power_supply_usb_type usb_psy_supported_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_ACA,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_PD_PPS,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID,
};
#ifdef CONFIG_CHARGE_FG_FUNCTION
struct power_supply_desc usb_psy_desc = {
#else /*CONFIG_CHARGE_FG_FUNCTION*/
static struct power_supply_desc usb_psy_desc = {
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
	.name			= "usb",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= usb_props,
	.num_properties		= ARRAY_SIZE(usb_props),
	.get_property		= usb_psy_get_prop,
	.set_property		= usb_psy_set_prop,
	.usb_types		= usb_psy_supported_types,
	.num_usb_types		= ARRAY_SIZE(usb_psy_supported_types),
	.property_is_writeable	= usb_psy_prop_is_writeable,
};

static int __battery_psy_set_charge_current(struct battery_chg_dev *bcdev,
					u32 fcc_ua)
{
	int rc;

	if (bcdev->restrict_chg_en) {
		fcc_ua = min_t(u32, fcc_ua, bcdev->restrict_fcc_ua);
		fcc_ua = min_t(u32, fcc_ua, bcdev->thermal_fcc_ua);
	}

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_BATTERY],
				BATT_CHG_CTRL_LIM, fcc_ua);
	if (rc < 0) {
		pr_err("Failed to set FCC %u, rc=%d\n", fcc_ua, rc);
	} else {
		pr_debug("Set FCC to %u uA\n", fcc_ua);
		bcdev->last_fcc_ua = fcc_ua;
	}

	return rc;
}

static int battery_psy_set_charge_current(struct battery_chg_dev *bcdev,
					int val)
{
	int rc;
	u32 fcc_ua, prev_fcc_ua;

	if (!bcdev->num_thermal_levels)
		return 0;

	if (bcdev->num_thermal_levels < 0) {
		pr_err("Incorrect num_thermal_levels\n");
		return -EINVAL;
	}

	if (val < 0 || val > bcdev->num_thermal_levels)
		return -EINVAL;

	if (bcdev->thermal_fcc_step == 0)
		fcc_ua = bcdev->thermal_levels[val];
	else
		fcc_ua = bcdev->psy_list[PSY_TYPE_BATTERY].prop[BATT_CHG_CTRL_LIM_MAX]
				- (bcdev->thermal_fcc_step * val);

	prev_fcc_ua = bcdev->thermal_fcc_ua;
	bcdev->thermal_fcc_ua = fcc_ua;

	rc = __battery_psy_set_charge_current(bcdev, fcc_ua);
	if (!rc)
		bcdev->curr_thermal_level = val;
	else
		bcdev->thermal_fcc_ua = prev_fcc_ua;

	return rc;
}

#ifdef CONFIG_CHARGE_FG_FUNCTION
static int battery_psy_set_input_current_limit(struct battery_chg_dev *bcdev,
					int val)
{
	int rc = 0;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_BATTERY],
				BATT_INPUT_CURRENT_LIMIT, val);
	if (rc < 0) {
		pr_err("Failed to set val %d, rc=%d\n", val, rc);
	} else {
		pr_err("Set val to %d \n", val);
	}
	return rc;
}

static int battery_psy_set_constant_current(struct battery_chg_dev *bcdev,
					int val)
{
	int rc = 0;
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
	union power_supply_propval pval = {0, };
	bool typec_priority = true;
	battery_chg_check_typec_priority(bcdev, &typec_priority);
	if(!typec_priority){
		pval.intval = val;
		rc = power_supply_set_property(bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
		if(rc < 0)
			pr_err("Failed to set constant_charge_current_max of sgm_chg_psy to %d, rc=%d\n", val, rc);
		else
			pr_err("Set constant_charge_current_max of sgm_chg_psy to %d \n", val);
	}
#endif /*CONFIG_DOUBLE_CHARGERS*/
	val = val/1000;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_BATTERY],
				BATT_CONSTANT_CHARGE_CURRENT, val);
	if (rc < 0) {
		pr_err("Failed to set val %d, rc=%d\n", val, rc);
	} else {
		pr_err("Set val to %d \n", val);
	}
	return rc;
}
#endif/*CONFIG_CHARGE_FG_FUNCTION*/

static int battery_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int prop_id, rc;
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
	union power_supply_propval val = {0,};
	bool typec_priority = true;
#endif /*CONFIG_DOUBLE_CHARGERS*/
	pval->intval = -ENODATA;

	/*
	 * The prop id of TIME_TO_FULL_NOW and TIME_TO_FULL_AVG is same.
	 * So, map the prop id of TIME_TO_FULL_AVG for TIME_TO_FULL_NOW.
	 */
	if (prop == POWER_SUPPLY_PROP_TIME_TO_FULL_NOW)
		prop = POWER_SUPPLY_PROP_TIME_TO_FULL_AVG;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	switch (prop) {
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
	case POWER_SUPPLY_PROP_STATUS:
		battery_chg_check_typec_priority(bcdev, &typec_priority);
		if(typec_priority){
			pval->intval = pst->prop[prop_id];
			if(pval->intval == POWER_SUPPLY_STATUS_CHARGING) {
				power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
				if(val.intval == 100)
					pval->intval = POWER_SUPPLY_STATUS_FULL;
			}
		}else{
			rc = power_supply_get_property(bcdev->sgm_chg_psy, prop, pval);
			if(rc < 0)
				return rc;
		}
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		battery_chg_check_typec_priority(bcdev, &typec_priority);
		if(typec_priority){
			pval->intval = pst->prop[prop_id]*1000;
		}else{
			rc = power_supply_get_property(bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, pval);
			if(rc < 0)
				return rc;
		}
		break;
#endif /*CONFIG_DOUBLE_CHARGERS*/
#ifdef CONFIG_CHARGE_FG_FUNCTION
	case POWER_SUPPLY_PROP_HEALTH:
		if(bcdev->vbat_is_dead)
			pval->intval = POWER_SUPPLY_HEALTH_DEAD;
		else{
#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
			battery_chg_check_typec_priority(bcdev, &typec_priority);
			if(typec_priority){
				pval->intval = pst->prop[prop_id];
			}else{
				rc = power_supply_get_property(bcdev->sgm_chg_psy, prop, pval);
				if(rc < 0)
					return rc;
			}
#else /*CONFIG_DOUBLE_CHARGERS*/
			pval->intval = pst->prop[prop_id];
#endif /*CONFIG_DOUBLE_CHARGERS*/
		}
		break;
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
	case POWER_SUPPLY_PROP_MODEL_NAME:
		pval->strval = pst->model;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		pval->intval = DIV_ROUND_CLOSEST(pst->prop[prop_id], 100);
		if (IS_ENABLED(CONFIG_QTI_PMIC_GLINK_CLIENT_DEBUG) &&
		   (bcdev->fake_soc >= 0 && bcdev->fake_soc <= 100))
			pval->intval = bcdev->fake_soc;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		pval->intval = DIV_ROUND_CLOSEST((int)pst->prop[prop_id], 10);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		pval->intval = bcdev->curr_thermal_level;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		pval->intval = bcdev->num_thermal_levels;
		break;
	default:
		pval->intval = pst->prop[prop_id];
		break;
	}

	return rc;
}

static int battery_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		return battery_psy_set_charge_current(bcdev, pval->intval);
#ifdef CONFIG_CHARGE_FG_FUNCTION
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return battery_psy_set_input_current_limit(bcdev, pval->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return battery_psy_set_constant_current(bcdev, pval->intval);
#endif/*CONFIG_CHARGE_FG_FUNCTION*/
	default:
		return -EINVAL;
	}

	return 0;
}

static int battery_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		return 1;
#ifdef CONFIG_CHARGE_FG_FUNCTION
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return 1;
#endif/*CONFIG_CHARGE_FG_FUNCTION*/
	default:
		break;
	}

	return 0;
}

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
#ifdef CONFIG_CHARGE_FG_FUNCTION
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
#endif/*CONFIG_CHARGE_FG_FUNCTION*/
};

static const struct power_supply_desc batt_psy_desc = {
	.name			= "battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= battery_props,
	.num_properties		= ARRAY_SIZE(battery_props),
	.get_property		= battery_psy_get_prop,
	.set_property		= battery_psy_set_prop,
	.property_is_writeable	= battery_psy_prop_is_writeable,
};

#if IS_ENABLED(CONFIG_DOUBLE_CHARGERS)
int battery_chg_check_typec_priority(struct battery_chg_dev * bcdev, bool *typec_priority){
	int rc = 0;
	bool temp_typec_priority = true;
	union power_supply_propval val = {0,};
	struct psy_state *usb_pst = &bcdev->psy_list[PSY_TYPE_USB];
	if (!bcdev->sgm_chg_psy) {
		bcdev->sgm_chg_psy= power_supply_get_by_name("sgm_charger");
		if (!bcdev->sgm_chg_psy)
			pr_err("Couldn't get sgm_chg_psy power_supply\n");
			return -ENODEV;
	}
	rc = power_supply_get_property(bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_PRESENT, &val);
	if(rc < 0){
		pr_err("Couldn't get sgm_chg_psy present\n");
		return rc;
	}

	if (val.intval) {
		rc = read_property_id(bcdev, usb_pst, USB_PRESENT);
		if(rc < 0)
			return rc;
		if(usb_pst->prop[USB_PRESENT]){
			rc = power_supply_get_property(bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_TYPE, &val);
			if(rc < 0){
				pr_err("Couldn't get sgm_chg_psy type\n");
				return rc;
			}
			if(val.intval == POWER_SUPPLY_TYPE_USB || val.intval == POWER_SUPPLY_TYPE_USB_CDP){
				rc = read_property_id(bcdev, usb_pst, USB_ADAP_TYPE);
				if(rc < 0)
					return rc;
				if(!(usb_pst->prop[USB_ADAP_TYPE] == POWER_SUPPLY_USB_TYPE_SDP || usb_pst->prop[USB_ADAP_TYPE] == POWER_SUPPLY_USB_TYPE_CDP)){
					pr_buf_info("sgm priority since it is SDP/CDP type\n");
					temp_typec_priority = false;
				}
			}
		}else{
			pr_buf_info("sgm priority since it is present only\n");
			temp_typec_priority = false;
		}
	}

	*typec_priority = temp_typec_priority;
	return 0;
}
#endif /*CONFIG_DOUBLE_CHARGERS*/

static int battery_chg_init_psy(struct battery_chg_dev *bcdev)
{
	struct power_supply_config psy_cfg = {};
	int rc;

	psy_cfg.drv_data = bcdev;
	psy_cfg.of_node = bcdev->dev->of_node;
	bcdev->psy_list[PSY_TYPE_USB].psy =
		devm_power_supply_register(bcdev->dev, &usb_psy_desc, &psy_cfg);
	if (IS_ERR(bcdev->psy_list[PSY_TYPE_USB].psy)) {
		rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_USB].psy);
		bcdev->psy_list[PSY_TYPE_USB].psy = NULL;
		pr_err("Failed to register USB power supply, rc=%d\n", rc);
		return rc;
	}

	if (bcdev->wls_not_supported) {
		pr_debug("Wireless charging is not supported\n");
	} else {
		bcdev->psy_list[PSY_TYPE_WLS].psy =
			devm_power_supply_register(bcdev->dev, &wls_psy_desc, &psy_cfg);

		if (IS_ERR(bcdev->psy_list[PSY_TYPE_WLS].psy)) {
			rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_WLS].psy);
			bcdev->psy_list[PSY_TYPE_WLS].psy = NULL;
			pr_err("Failed to register wireless power supply, rc=%d\n", rc);
			return rc;
		}
	}

	bcdev->psy_list[PSY_TYPE_BATTERY].psy =
		devm_power_supply_register(bcdev->dev, &batt_psy_desc,
						&psy_cfg);
	if (IS_ERR(bcdev->psy_list[PSY_TYPE_BATTERY].psy)) {
		rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_BATTERY].psy);
		bcdev->psy_list[PSY_TYPE_BATTERY].psy = NULL;
		pr_err("Failed to register battery power supply, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

#ifdef CONFIG_CHARGE_FG_FUNCTION
#define BATT_DEAD_COUNT		10

static void qti_chg_status_update_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev, qti_chg_status_work.work);
	int ret = 0;
	static int vbat_dead_count;
	static int dead_wake_flag;
	union power_supply_propval pval = {0, };
	struct power_supply *batt_psy = bcdev->psy_list[PSY_TYPE_BATTERY].psy;
	struct power_supply *usb_psy = bcdev->psy_list[PSY_TYPE_USB].psy;
	struct charge_status qti_charge_status = {0};
	struct psy_state *usb_pst = &bcdev->psy_list[PSY_TYPE_USB];
	struct psy_state *batt_pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	if(!batt_psy){
		batt_psy = power_supply_get_by_name("battery");
		if (!batt_psy){
			dev_err(bcdev->dev, "Could not get battery power_supply\n");
			return;
		}
	}

	if(!usb_psy){
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy){
		dev_err(bcdev->dev, "Could not get usb power_supply\n");
		return;
		}
	}

	ret = power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_CAPACITY, &pval);
	if (ret) {
		dev_err(bcdev->dev, "get failed to get battery capacity now %d\n", ret);
		return;
	}
	qti_charge_status.msoc = pval.intval;
	ret = power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);
	if (ret) {
		dev_err(bcdev->dev, "get failed to get battery voltage now %d\n", ret);
		return;
	}
	qti_charge_status.vbatt_now = pval.intval;
	ret = power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &pval);
	if (ret) {
		dev_err(bcdev->dev, "get failed to get battery current now %d\n", ret);
		return;
	}
	qti_charge_status.ibatt_now = pval.intval;

	ret = power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_TEMP, &pval);
	if (ret) {
		dev_err(bcdev->dev, "get failed to get battery temp now %d\n", ret);
		return;
	}
	qti_charge_status.batt_temp = pval.intval;
	ret = read_property_id(bcdev, batt_pst, BATT_DEAD);
	if (ret) {
		dev_err(bcdev->dev, "get failed to get battery dead now %d\n", ret);
		return;
	}
	qti_charge_status.vbat_dead = batt_pst->prop[BATT_DEAD];

	if (qti_charge_status.vbat_dead && qti_charge_status.msoc == 0) {
		if (!dead_wake_flag) {
			dead_wake_flag = true;
			__pm_stay_awake(bcdev->qti_batt_chg_ws);
		}
		vbat_dead_count++;
		pr_err("bat is becoming dead %d count %d\n",qti_charge_status.vbatt_now, vbat_dead_count);
		if (vbat_dead_count >= BATT_DEAD_COUNT) {
			bcdev->vbat_is_dead = true;
			if(batt_psy)
				power_supply_changed(batt_psy);
		}
	} else {
		vbat_dead_count = 0;
		bcdev->vbat_is_dead = false;
		if (dead_wake_flag) {
			dead_wake_flag = false;
			__pm_relax(bcdev->qti_batt_chg_ws);
		}
	}

	ret = power_supply_get_property(usb_psy, POWER_SUPPLY_PROP_ONLINE, &pval);
	if (ret) {
		dev_err(bcdev->dev, "get failed to get usb online now %d\n", ret);
		return;
	}
	qti_charge_status.usb_online = pval.intval;

	ret = power_supply_get_property(usb_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &pval);
	if (ret) {
		dev_err(bcdev->dev, "get failed to get input current now %d\n", ret);
		return;
	}
	qti_charge_status.input_current_now = pval.intval;

	ret = power_supply_get_property(usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);
	if (ret) {
		dev_err(bcdev->dev, "get failed to get input voltage now %d\n", ret);
		return;
	}
	qti_charge_status.input_voltage_now = pval.intval;

	ret = power_supply_get_property(usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &pval);
	if(ret < 0){
		dev_err(bcdev->dev, "get failed to get POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT %d\n", ret);
		return;
	}
	qti_charge_status.input_current_limit = pval.intval;

	ret = read_property_id(bcdev, usb_pst, USB_REAL_TYPE);
	if (ret) {
		dev_err(bcdev->dev, "get failed to get usb real type %d\n", ret);
		return;
	}

	pr_err("qti_charge_status.input_current_limit   %d  usb_online %d  USB_REAL_TYPE %d\n", qti_charge_status.input_current_limit,
		qti_charge_status.usb_online,usb_pst->prop[USB_REAL_TYPE]);
	if (qti_charge_status.input_current_limit < 200000 && qti_charge_status.usb_online) {
		if(usb_pst->prop[USB_REAL_TYPE] == POWER_SUPPLY_USB_TYPE_SDP)
			pval.intval = 500000;
		else if(usb_pst->prop[USB_REAL_TYPE] == POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID)
			pval.intval = 900000;
		else if(usb_pst->prop[USB_REAL_TYPE] == POWER_SUPPLY_USB_TYPE_PD)
			pval.intval = 1000000;
		else if(usb_pst->prop[USB_REAL_TYPE] == POWER_SUPPLY_USB_TYPE_CDP)
			pval.intval = 1500000;
		else
			pval.intval = 2000000;
		ret = power_supply_set_property(usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &pval);
		if(ret < 0)
			pr_err("Failed to set INPUT_CURRENT_LIMIT to %d, rc=%d\n", pval.intval, ret);
		else
			pr_err("Set INPUT_CURRENT_LIMIT  to %d \n", pval.intval);
	}
	pr_err("hs_info :: vbus=%d, usb_online=%d, ibatt_now=%d, vbatt_now=%d, capacity=%d, temp=%d, input_current_now=%d,real_charger_type=%s,input_current_limit=%d\n",
			qti_charge_status.input_voltage_now, qti_charge_status.usb_online, qti_charge_status.ibatt_now, qti_charge_status.vbatt_now,
			qti_charge_status.msoc, qti_charge_status.batt_temp, qti_charge_status.input_current_now,get_usb_type_name(usb_pst->prop[USB_REAL_TYPE]),
			qti_charge_status.input_current_limit);
	schedule_delayed_work(&bcdev->qti_chg_status_work, msecs_to_jiffies(20000));
}
#endif /*CONFIG_CHARGE_FG_FUNCTION*/

static void battery_chg_subsys_up_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev, subsys_up_work);
	int rc;

	battery_chg_notify_enable(bcdev);

	/*
	 * Give some time after enabling notification so that USB adapter type
	 * information can be obtained properly which is essential for setting
	 * USB ICL.
	 */
	msleep(200);

	if (bcdev->last_fcc_ua) {
		rc = __battery_psy_set_charge_current(bcdev,
				bcdev->last_fcc_ua);
		if (rc < 0)
			pr_err("Failed to set FCC (%u uA), rc=%d\n",
				bcdev->last_fcc_ua, rc);
	}

	if (bcdev->usb_icl_ua) {
		rc = usb_psy_set_icl(bcdev, USB_INPUT_CURR_LIMIT,
				bcdev->usb_icl_ua);
		if (rc < 0)
			pr_err("Failed to set ICL(%u uA), rc=%d\n",
				bcdev->usb_icl_ua, rc);
	}
}

static int wireless_fw_send_firmware(struct battery_chg_dev *bcdev,
					const struct firmware *fw)
{
	struct wireless_fw_push_buf_req msg = {};
	const u8 *ptr;
	u32 i, num_chunks, partial_chunk_size;
	int rc;

	num_chunks = fw->size / WLS_FW_BUF_SIZE;
	partial_chunk_size = fw->size % WLS_FW_BUF_SIZE;

	if (!num_chunks)
		return -EINVAL;

	pr_debug("Updating FW...\n");

	ptr = fw->data;
	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_WLS_FW_PUSH_BUF_REQ;

	for (i = 0; i < num_chunks; i++, ptr += WLS_FW_BUF_SIZE) {
		msg.fw_chunk_id = i + 1;
		memcpy(msg.buf, ptr, WLS_FW_BUF_SIZE);

		pr_debug("sending FW chunk %u\n", i + 1);
		rc = battery_chg_fw_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			return rc;
	}

	if (partial_chunk_size) {
		msg.fw_chunk_id = i + 1;
		memset(msg.buf, 0, WLS_FW_BUF_SIZE);
		memcpy(msg.buf, ptr, partial_chunk_size);

		pr_debug("sending partial FW chunk %u\n", i + 1);
		rc = battery_chg_fw_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			return rc;
	}

	return 0;
}

static int wireless_fw_check_for_update(struct battery_chg_dev *bcdev,
					u32 version, size_t size)
{
	struct wireless_fw_check_req req_msg = {};

	bcdev->wls_fw_update_reqd = false;

	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = BC_WLS_FW_CHECK_UPDATE;
	req_msg.fw_version = version;
	req_msg.fw_size = size;
	req_msg.fw_crc = bcdev->wls_fw_crc;

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

#define IDT9415_FW_MAJOR_VER_OFFSET		0x84
#define IDT9415_FW_MINOR_VER_OFFSET		0x86
#define IDT_FW_MAJOR_VER_OFFSET		0x94
#define IDT_FW_MINOR_VER_OFFSET		0x96
static int wireless_fw_update(struct battery_chg_dev *bcdev, bool force)
{
	const struct firmware *fw;
	struct psy_state *pst;
	u32 version;
	u16 maj_ver, min_ver;
	int rc;

	if (!bcdev->wls_fw_name) {
		pr_err("wireless FW name is not specified\n");
		return -EINVAL;
	}

	pm_stay_awake(bcdev->dev);

	/*
	 * Check for USB presence. If nothing is connected, check whether
	 * battery SOC is at least 50% before allowing FW update.
	 */
	pst = &bcdev->psy_list[PSY_TYPE_USB];
	rc = read_property_id(bcdev, pst, USB_ONLINE);
	if (rc < 0)
		goto out;

	if (!pst->prop[USB_ONLINE]) {
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		rc = read_property_id(bcdev, pst, BATT_CAPACITY);
		if (rc < 0)
			goto out;

		if ((pst->prop[BATT_CAPACITY] / 100) < 50) {
			pr_err("Battery SOC should be at least 50%% or connect charger\n");
			rc = -EINVAL;
			goto out;
		}
	}

	rc = firmware_request_nowarn(&fw, bcdev->wls_fw_name, bcdev->dev);
	if (rc) {
		pr_err("Couldn't get firmware rc=%d\n", rc);
		goto out;
	}

	if (!fw || !fw->data || !fw->size) {
		pr_err("Invalid firmware\n");
		rc = -EINVAL;
		goto release_fw;
	}

	if (fw->size < SZ_16K) {
		pr_err("Invalid firmware size %zu\n", fw->size);
		rc = -EINVAL;
		goto release_fw;
	}

	if (strstr(bcdev->wls_fw_name, "9412")) {
		maj_ver = le16_to_cpu(*(__le16 *)(fw->data + IDT_FW_MAJOR_VER_OFFSET));
		min_ver = le16_to_cpu(*(__le16 *)(fw->data + IDT_FW_MINOR_VER_OFFSET));
	} else {
		maj_ver = le16_to_cpu(*(__le16 *)(fw->data + IDT9415_FW_MAJOR_VER_OFFSET));
		min_ver = le16_to_cpu(*(__le16 *)(fw->data + IDT9415_FW_MINOR_VER_OFFSET));
	}
	version = maj_ver << 16 | min_ver;

	if (force)
		version = UINT_MAX;

	pr_debug("FW size: %zu version: %#x\n", fw->size, version);

	rc = wireless_fw_check_for_update(bcdev, version, fw->size);
	if (rc < 0) {
		pr_err("Wireless FW update not needed, rc=%d\n", rc);
		goto release_fw;
	}

	if (!bcdev->wls_fw_update_reqd) {
		pr_warn("Wireless FW update not required\n");
		goto release_fw;
	}

	/* Wait for IDT to be setup by charger firmware */
	msleep(WLS_FW_PREPARE_TIME_MS);

	reinit_completion(&bcdev->fw_update_ack);
	rc = wireless_fw_send_firmware(bcdev, fw);
	if (rc < 0) {
		pr_err("Failed to send FW chunk, rc=%d\n", rc);
		goto release_fw;
	}

	pr_debug("Waiting for fw_update_ack\n");
	rc = wait_for_completion_timeout(&bcdev->fw_update_ack,
				msecs_to_jiffies(bcdev->wls_fw_update_time_ms));
	if (!rc) {
		pr_err("Error, timed out updating firmware\n");
		rc = -ETIMEDOUT;
		goto release_fw;
	} else {
		pr_debug("Waited for %d ms\n",
			bcdev->wls_fw_update_time_ms - jiffies_to_msecs(rc));
		rc = 0;
	}

	pr_info("Wireless FW update done\n");

release_fw:
	bcdev->wls_fw_crc = 0;
	release_firmware(fw);
out:
	pm_relax(bcdev->dev);

	return rc;
}

static ssize_t wireless_fw_update_time_ms_store(struct class *c,
				struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtou32(buf, 0, &bcdev->wls_fw_update_time_ms))
		return -EINVAL;

	return count;
}

static ssize_t wireless_fw_update_time_ms_show(struct class *c,
				struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%u\n", bcdev->wls_fw_update_time_ms);
}
static CLASS_ATTR_RW(wireless_fw_update_time_ms);

static ssize_t wireless_fw_crc_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	u16 val;

	if (kstrtou16(buf, 0, &val) || !val)
		return -EINVAL;

	bcdev->wls_fw_crc = val;

	return count;
}
static CLASS_ATTR_WO(wireless_fw_crc);

static ssize_t wireless_fw_version_show(struct class *c,
					struct class_attribute *attr,
					char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct wireless_fw_get_version_req req_msg = {};
	int rc;

	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = BC_WLS_FW_GET_VERSION;

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get FW version rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%#x\n", bcdev->wls_fw_version);
}
static CLASS_ATTR_RO(wireless_fw_version);

static ssize_t wireless_fw_force_update_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	bool val;
	int rc;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	rc = wireless_fw_update(bcdev, true);
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_WO(wireless_fw_force_update);

static ssize_t wireless_fw_update_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	bool val;
	int rc;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	rc = wireless_fw_update(bcdev, false);
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_WO(wireless_fw_update);

static ssize_t usb_typec_compliant_show(struct class *c,
				struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_TYPEC_COMPLIANT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			(int)pst->prop[USB_TYPEC_COMPLIANT]);
}
static CLASS_ATTR_RO(usb_typec_compliant);

static ssize_t usb_real_type_show(struct class *c,
				struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_REAL_TYPE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			get_usb_type_name(pst->prop[USB_REAL_TYPE]));
}
static CLASS_ATTR_RO(usb_real_type);

static ssize_t restrict_cur_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	u32 fcc_ua, prev_fcc_ua;

	if (kstrtou32(buf, 0, &fcc_ua) || fcc_ua > bcdev->thermal_fcc_ua)
		return -EINVAL;

	prev_fcc_ua = bcdev->restrict_fcc_ua;
	bcdev->restrict_fcc_ua = fcc_ua;
	if (bcdev->restrict_chg_en) {
		rc = __battery_psy_set_charge_current(bcdev, fcc_ua);
		if (rc < 0) {
			bcdev->restrict_fcc_ua = prev_fcc_ua;
			return rc;
		}
	}

	return count;
}

static ssize_t restrict_cur_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%u\n", bcdev->restrict_fcc_ua);
}
static CLASS_ATTR_RW(restrict_cur);

static ssize_t restrict_chg_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	bcdev->restrict_chg_en = val;
	rc = __battery_psy_set_charge_current(bcdev, bcdev->restrict_chg_en ?
			bcdev->restrict_fcc_ua : bcdev->thermal_fcc_ua);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t restrict_chg_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->restrict_chg_en);
}
static CLASS_ATTR_RW(restrict_chg);

static ssize_t fake_soc_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	bcdev->fake_soc = val;
	pr_debug("Set fake soc to %d\n", val);

	if (IS_ENABLED(CONFIG_QTI_PMIC_GLINK_CLIENT_DEBUG) && pst->psy)
		power_supply_changed(pst->psy);

	return count;
}

static ssize_t fake_soc_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->fake_soc);
}
static CLASS_ATTR_RW(fake_soc);

static ssize_t wireless_boost_en_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_WLS],
				WLS_BOOST_EN, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t wireless_boost_en_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_WLS];
	int rc;

	rc = read_property_id(bcdev, pst, WLS_BOOST_EN);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[WLS_BOOST_EN]);
}
static CLASS_ATTR_RW(wireless_boost_en);

static ssize_t moisture_detection_en_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_USB],
				USB_MOISTURE_DET_EN, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t moisture_detection_en_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_MOISTURE_DET_EN);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			pst->prop[USB_MOISTURE_DET_EN]);
}
static CLASS_ATTR_RW(moisture_detection_en);

static ssize_t moisture_detection_status_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_MOISTURE_DET_STS);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			pst->prop[USB_MOISTURE_DET_STS]);
}
static CLASS_ATTR_RO(moisture_detection_status);

static ssize_t resistance_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int rc;

	rc = read_property_id(bcdev, pst, BATT_RESISTANCE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[BATT_RESISTANCE]);
}
static CLASS_ATTR_RO(resistance);

static ssize_t flash_active_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, F_ACTIVE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[F_ACTIVE]);
}
static CLASS_ATTR_RO(flash_active);

static ssize_t soh_show(struct class *c, struct class_attribute *attr,
			char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int rc;

	rc = read_property_id(bcdev, pst, BATT_SOH);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[BATT_SOH]);
}
static CLASS_ATTR_RO(soh);

static ssize_t ship_mode_en_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtobool(buf, &bcdev->ship_mode_en))
		return -EINVAL;

	return count;
}

static ssize_t ship_mode_en_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->ship_mode_en);
}
static CLASS_ATTR_RW(ship_mode_en);

static struct attribute *battery_class_attrs[] = {
	&class_attr_soh.attr,
	&class_attr_resistance.attr,
	&class_attr_flash_active.attr,
	&class_attr_moisture_detection_status.attr,
	&class_attr_moisture_detection_en.attr,
	&class_attr_wireless_boost_en.attr,
	&class_attr_fake_soc.attr,
	&class_attr_wireless_fw_update.attr,
	&class_attr_wireless_fw_force_update.attr,
	&class_attr_wireless_fw_version.attr,
	&class_attr_wireless_fw_crc.attr,
	&class_attr_wireless_fw_update_time_ms.attr,
	&class_attr_ship_mode_en.attr,
	&class_attr_restrict_chg.attr,
	&class_attr_restrict_cur.attr,
	&class_attr_usb_real_type.attr,
	&class_attr_usb_typec_compliant.attr,
	NULL,
};
ATTRIBUTE_GROUPS(battery_class);

static struct attribute *battery_class_no_wls_attrs[] = {
	&class_attr_soh.attr,
	&class_attr_resistance.attr,
	&class_attr_flash_active.attr,
	&class_attr_moisture_detection_status.attr,
	&class_attr_moisture_detection_en.attr,
	&class_attr_fake_soc.attr,
	&class_attr_ship_mode_en.attr,
	&class_attr_restrict_chg.attr,
	&class_attr_restrict_cur.attr,
	&class_attr_usb_real_type.attr,
	&class_attr_usb_typec_compliant.attr,
	NULL,
};
ATTRIBUTE_GROUPS(battery_class_no_wls);

#ifdef CONFIG_DEBUG_FS
static void battery_chg_add_debugfs(struct battery_chg_dev *bcdev)
{
	int rc;
	struct dentry *dir;

	dir = debugfs_create_dir("battery_charger", NULL);
	if (IS_ERR(dir)) {
		rc = PTR_ERR(dir);
		pr_err("Failed to create charger debugfs directory, rc=%d\n",
			rc);
		return;
	}

	bcdev->debugfs_dir = dir;
	debugfs_create_bool("block_tx", 0600, dir, &bcdev->block_tx);
}
#else
static void battery_chg_add_debugfs(struct battery_chg_dev *bcdev) { }
#endif

static int battery_chg_parse_dt(struct battery_chg_dev *bcdev)
{
	struct device_node *node = bcdev->dev->of_node;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int i, rc, len;
	u32 prev, val;

	bcdev->wls_not_supported = of_property_read_bool(node,
			"qcom,wireless-charging-not-supported");

	of_property_read_string(node, "qcom,wireless-fw-name",
				&bcdev->wls_fw_name);

	of_property_read_u32(node, "qcom,shutdown-voltage",
				&bcdev->shutdown_volt_mv);


	rc = read_property_id(bcdev, pst, BATT_CHG_CTRL_LIM_MAX);
	if (rc < 0) {
		pr_err("Failed to read prop BATT_CHG_CTRL_LIM_MAX, rc=%d\n",
			rc);
		return rc;
	}

	rc = of_property_count_elems_of_size(node, "qcom,thermal-mitigation",
						sizeof(u32));
	if (rc <= 0) {

		rc = of_property_read_u32(node, "qcom,thermal-mitigation-step",
						&val);

		if (rc < 0)
			return 0;

		if (val < 500000 || val >= pst->prop[BATT_CHG_CTRL_LIM_MAX]) {
			pr_err("thermal_fcc_step %d is invalid\n", val);
			return -EINVAL;
		}

		bcdev->thermal_fcc_step = val;
		len = pst->prop[BATT_CHG_CTRL_LIM_MAX] / bcdev->thermal_fcc_step;

		/*
		 * FCC values must be above 500mA.
		 * Since len is truncated when calculated, check and adjust len so
		 * that the above requirement is met.
		 */
		if (pst->prop[BATT_CHG_CTRL_LIM_MAX] - (bcdev->thermal_fcc_step * len) < 500000)
			len = len - 1;
	} else {
		bcdev->thermal_fcc_step = 0;
		len = rc;
		prev = pst->prop[BATT_CHG_CTRL_LIM_MAX];

		for (i = 0; i < len; i++) {
			rc = of_property_read_u32_index(node, "qcom,thermal-mitigation",
				i, &val);
			if (rc < 0)
				return rc;

			if (val > prev) {
				pr_err("Thermal levels should be in descending order\n");
				bcdev->num_thermal_levels = -EINVAL;
				return 0;
			}

			prev = val;
		}

		bcdev->thermal_levels = devm_kcalloc(bcdev->dev, len + 1,
						sizeof(*bcdev->thermal_levels),
						GFP_KERNEL);
		if (!bcdev->thermal_levels)
			return -ENOMEM;

		/*
		 * Element 0 is for normal charging current. Elements from index 1
		 * onwards is for thermal mitigation charging currents.
		 */

		bcdev->thermal_levels[0] = pst->prop[BATT_CHG_CTRL_LIM_MAX];

		rc = of_property_read_u32_array(node, "qcom,thermal-mitigation",
					&bcdev->thermal_levels[1], len);
		if (rc < 0) {
			pr_err("Error in reading qcom,thermal-mitigation, rc=%d\n", rc);
			return rc;
		}
	}

	bcdev->num_thermal_levels = len;
	bcdev->thermal_fcc_ua = pst->prop[BATT_CHG_CTRL_LIM_MAX];

	bcdev->ssphy = devm_usb_get_phy_by_phandle(bcdev->dev, "usb-ssphy", 0);
	if(IS_ERR(bcdev->ssphy)){
		pr_err("Failed to get usb super speed phy\n");
		rc = PTR_ERR(bcdev->ssphy);
		return rc;
	}
	return 0;
}

static int battery_chg_ship_mode(struct notifier_block *nb, unsigned long code,
		void *unused)
{
	struct battery_charger_notify_msg msg_notify = { { 0 } };
	struct battery_charger_ship_mode_req_msg msg = { { 0 } };
	struct battery_chg_dev *bcdev = container_of(nb, struct battery_chg_dev,
						     reboot_notifier);
	int rc;

	msg_notify.hdr.owner = MSG_OWNER_BC;
	msg_notify.hdr.type = MSG_TYPE_NOTIFY;
	msg_notify.hdr.opcode = BC_SHUTDOWN_NOTIFY;

	rc = battery_chg_write(bcdev, &msg_notify, sizeof(msg_notify));
	if (rc < 0)
		pr_err("Failed to send shutdown notification rc=%d\n", rc);

	if (!bcdev->ship_mode_en)
		return NOTIFY_DONE;

	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_SHIP_MODE_REQ_SET;
	msg.ship_mode_type = SHIP_MODE_PMIC;

	if (code == SYS_POWER_OFF) {
		rc = battery_chg_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			pr_emerg("Failed to write ship mode: %d\n", rc);
	}

	return NOTIFY_DONE;
}

static void panel_event_notifier_callback(enum panel_event_notifier_tag tag,
			struct panel_event_notification *notification, void *data)
{
	struct battery_chg_dev *bcdev = data;

	if (!notification) {
		pr_debug("Invalid panel notification\n");
		return;
	}

	pr_debug("panel event received, type: %d\n", notification->notif_type);
	switch (notification->notif_type) {
	case DRM_PANEL_EVENT_BLANK:
		battery_chg_notify_disable(bcdev);
		break;
	case DRM_PANEL_EVENT_UNBLANK:
		battery_chg_notify_enable(bcdev);
		break;
	default:
		pr_debug("Ignore panel event: %d\n", notification->notif_type);
		break;
	}
}

static int battery_chg_register_panel_notifier(struct battery_chg_dev *bcdev)
{
	struct device_node *np = bcdev->dev->of_node;
	struct device_node *pnode;
	struct drm_panel *panel, *active_panel = NULL;
	void *cookie = NULL;
	int i, count, rc;

	count = of_count_phandle_with_args(np, "qcom,display-panels", NULL);
	if (count <= 0)
		return 0;

	for (i = 0; i < count; i++) {
		pnode = of_parse_phandle(np, "qcom,display-panels", i);
		if (!pnode)
			return -ENODEV;

		panel = of_drm_find_panel(pnode);
		of_node_put(pnode);
		if (!IS_ERR(panel)) {
			active_panel = panel;
			break;
		}
	}

	if (!active_panel) {
		rc = PTR_ERR(panel);
		if (rc != -EPROBE_DEFER)
			dev_err(bcdev->dev, "Failed to find active panel, rc=%d\n");
		return rc;
	}

	cookie = panel_event_notifier_register(
			PANEL_EVENT_NOTIFICATION_PRIMARY,
			PANEL_EVENT_NOTIFIER_CLIENT_BATTERY_CHARGER,
			active_panel,
			panel_event_notifier_callback,
			(void *)bcdev);
	if (IS_ERR(cookie)) {
		rc = PTR_ERR(cookie);
		dev_err(bcdev->dev, "Failed to register panel event notifier, rc=%d\n", rc);
		return rc;
	}

	pr_debug("register panel notifier successful\n");
	bcdev->notifier_cookie = cookie;
	return 0;
}

static int register_extcon_conn_type(struct battery_chg_dev *bcdev)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_CONNECTOR_TYPE);
	if (rc < 0) {
		pr_err("Failed to read prop USB_CONNECTOR_TYPE, rc=%d\n",
			rc);
		return rc;
	}

	bcdev->connector_type = pst->prop[USB_CONNECTOR_TYPE];
	bcdev->usb_prev_mode = EXTCON_NONE;

	bcdev->extcon = devm_extcon_dev_allocate(bcdev->dev,
						bcdev_usb_extcon_cable);
	if (IS_ERR(bcdev->extcon)) {
		rc = PTR_ERR(bcdev->extcon);
		pr_err("Failed to allocate extcon device rc=%d\n", rc);
		return rc;
	}

	rc = devm_extcon_dev_register(bcdev->dev, bcdev->extcon);
	if (rc < 0) {
		pr_err("Failed to register extcon device rc=%d\n", rc);
		return rc;
	}
	rc = extcon_set_property_capability(bcdev->extcon, EXTCON_USB,
					    EXTCON_PROP_USB_SS);
	rc |= extcon_set_property_capability(bcdev->extcon,
					     EXTCON_USB_HOST, EXTCON_PROP_USB_SS);
	if (rc < 0)
		pr_err("failed to configure extcon capabilities rc=%d\n", rc);
	else
		pr_debug("Registered extcon, connector_type %s\n",
			 bcdev->connector_type ? "uusb" : "Typec");

	return rc;
}

#ifdef CONFIG_CHARGE_FG_FUNCTION
static struct battery_chg_dev *g_bcdev = NULL;

void water_present_close_usbhost(void){
	int rc = 0;
	pr_buf_err("moisture detected present, current moisture_present: %d\n", g_bcdev->moisture_present);
	if(!g_bcdev->moisture_present){
		rc = write_property_id(g_bcdev, &g_bcdev->psy_list[PSY_TYPE_USB],
					USB_HOST_ENABLED, 0);
		if (rc < 0)
			return;
		g_bcdev->moisture_present = true;
	}
	return;
}
EXPORT_SYMBOL_GPL(water_present_close_usbhost);

void water_dry_open_usbhost(void){
	int rc = 0;
	pr_buf_err("moisture detected dry, current moisture_present: %d\n", g_bcdev->moisture_present);
	if(g_bcdev->moisture_present){
		rc = write_property_id(g_bcdev, &g_bcdev->psy_list[PSY_TYPE_USB],
					USB_HOST_ENABLED, 1);
		if (rc < 0)
			return;
		g_bcdev->moisture_present = false;
	}
	return;
}
EXPORT_SYMBOL_GPL(water_dry_open_usbhost);

static ssize_t water_lpd_test_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf){
	return sprintf(buf, "%d\n", g_bcdev->moisture_present);
}
static ssize_t water_lpd_test_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count){
	int rc = 0;
	bool enable = false;
	unsigned long state = 0;
	rc = kstrtoul(buf, 10, &state);
	if(rc){
		pr_buf_err("kstrtoul operation failed  rc=%d\n", rc);
		return rc;
	}
	enable = !!state;
	pr_buf_err("%sable water lpd %d\n", enable ? "en" : "dis", g_bcdev->moisture_present);
	if(enable != g_bcdev->moisture_present){
		rc = write_property_id(g_bcdev, &g_bcdev->psy_list[PSY_TYPE_USB],
					USB_HOST_ENABLED, !enable);
		if(rc < 0)
			return rc;
		g_bcdev->moisture_present = enable;
	}
	rc = count;
	return rc;
}
static struct kobj_attribute water_lpd_test_attr = __ATTR_RW(water_lpd_test);

static ssize_t fast_charge_status_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf){
	struct psy_state *pst;
	int rc;
	u32 fcc_ua;
	bool fastcharge_flag= false;
	pst  = &g_bcdev->psy_list[PSY_TYPE_USB];
	rc = read_property_id(g_bcdev, pst, USB_REAL_TYPE);
	if(rc < 0)
		return rc;
	if(pst->prop[USB_REAL_TYPE] == QTI_POWER_SUPPLY_USB_TYPE_HVDCP ||
		pst->prop[USB_REAL_TYPE] == QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3 ||
		pst->prop[USB_REAL_TYPE] == QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5 ||
		pst->prop[USB_REAL_TYPE] == POWER_SUPPLY_USB_TYPE_PD ||
		pst->prop[USB_REAL_TYPE] == POWER_SUPPLY_USB_TYPE_PD_DRP ||
		pst->prop[USB_REAL_TYPE] == POWER_SUPPLY_USB_TYPE_PD_PPS){

		pst  = &g_bcdev->psy_list[PSY_TYPE_BATTERY];
		rc = read_property_id(g_bcdev, pst, BATT_CONSTANT_CHARGE_CURRENT);
		if(rc < 0)
			return rc;
		pr_buf_info("@@@@%s: BATT_CONSTANT_CHARGE_CURRENT is %dmA\n", __func__, pst->prop[BATT_CONSTANT_CHARGE_CURRENT]);
		fcc_ua = pst->prop[BATT_CONSTANT_CHARGE_CURRENT] * 1000;
		if(fcc_ua > FAST_CHARGE_DISABLED_CURRENT_UA)
			fastcharge_flag = true;
	}

	pr_buf_info("@@@@%s: fastcharge_flags is %d\n", __func__, fastcharge_flag);
	return snprintf(buf, PAGE_SIZE, "%d\n",  fastcharge_flag);
}
static struct kobj_attribute fast_charge_status_attr = __ATTR_RO(fast_charge_status);

static ssize_t typec_cc_orientation_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct psy_state *pst = &g_bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(g_bcdev, pst, USB_CC_ORIENTATION);
	if (rc < 0)
		return rc;

	return sprintf(buf, "%d\n", pst->prop[USB_CC_ORIENTATION]);
}
static struct kobj_attribute typec_cc_orientation_attr = __ATTR_RO(typec_cc_orientation);

static ssize_t charging_enabled_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct psy_state *pst = &g_bcdev->psy_list[PSY_TYPE_BATTERY];
	int rc;

	rc = read_property_id(g_bcdev, pst, BATT_CHARGING_ENABLED);
	if (rc < 0)
		return rc;

	return sprintf(buf, "%d\n", pst->prop[BATT_CHARGING_ENABLED]);
}

static ssize_t charging_enabled_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count){
	int rc = 0;
	bool enable = false;
	unsigned long state = 0;
	rc = kstrtoul(buf, 10, &state);
	if(rc){
		pr_buf_err("kstrtoul operation failed  rc=%d\n", rc);
		return rc;
	}
	enable = !!state;
	rc = write_property_id(g_bcdev, &g_bcdev->psy_list[PSY_TYPE_BATTERY],
				BATT_CHARGING_ENABLED, enable);
	if(rc < 0){
		pr_buf_err("set charging_enable err rc =%d\n", rc);
		return rc;
	}
	pr_buf_err("set charging_enable %d\n", enable);
	rc = count;
	return rc;
}
static struct kobj_attribute charging_enabled_attr = __ATTR_RW(charging_enabled);

static ssize_t safety_timer_enable_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct psy_state *pst = &g_bcdev->psy_list[PSY_TYPE_BATTERY];
	int rc;

	rc = read_property_id(g_bcdev, pst, BATT_SAFETY_TIMER_ENABLE);
	if (rc < 0)
		return rc;

	return sprintf(buf, "%d\n", pst->prop[BATT_SAFETY_TIMER_ENABLE]);
}

static ssize_t safety_timer_enable_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count){
	int rc = 0;
	bool enable = false;
	unsigned long state = 0;
	rc = kstrtoul(buf, 10, &state);
	if(rc){
		pr_buf_err("kstrtoul operation failed  rc=%d\n", rc);
		return rc;
	}
	enable = !!state;
	rc = write_property_id(g_bcdev, &g_bcdev->psy_list[PSY_TYPE_BATTERY],
				BATT_SAFETY_TIMER_ENABLE, enable);
	if(rc < 0){
		pr_buf_err("set charging_enable err rc =%d\n", rc);
		return rc;
	}
	pr_buf_err("set safety_timer_enable %d\n", enable);
	rc = count;
	return rc;
}
static struct kobj_attribute safety_timer_enable_attr = __ATTR_RW(safety_timer_enable);

ssize_t set_typec_usb_data_only_mode(bool enable)
{
	int rc;
	struct psy_state *pst = &g_bcdev->psy_list[PSY_TYPE_USB];

	rc = write_property_id(g_bcdev, pst, USB_DATA_ONLY_MODE, enable);
	if(rc < 0)
		return rc;
	return 0;
}
EXPORT_SYMBOL_GPL(set_typec_usb_data_only_mode);
ssize_t get_typec_usb_data_only_mode(bool *enable)
{
	int rc;
	struct psy_state *pst = &g_bcdev->psy_list[PSY_TYPE_USB];
	bool usb_data_only_mode = false;

	rc = read_property_id(g_bcdev, pst, USB_DATA_ONLY_MODE);
	if(rc < 0)
		return rc;
	usb_data_only_mode = !!pst->prop[USB_DATA_ONLY_MODE];

	pr_info("usb_data_only_mode: %d\n", usb_data_only_mode);
	return 0;
}
EXPORT_SYMBOL_GPL(get_typec_usb_data_only_mode);
static ssize_t charging_stopped_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int rc;
	bool stopped = false;
	bool typec_significant = true;
	unsigned long state = 0;
	union power_supply_propval val = {0,};

	rc = kstrtoul(buf, 10, &state);
	if(rc){
		pr_buf_err("kstrtoul operation failed  rc=%d\n", rc);
		return rc;
	}

	stopped = !!state;
	pr_buf_err("set charging_stopped to %d\n", stopped);
	if(!g_bcdev->sgm_chg_psy){
		g_bcdev->sgm_chg_psy = power_supply_get_by_name("sgm_charger");
		if(!g_bcdev->sgm_chg_psy){
			rc = PTR_ERR(g_bcdev->sgm_chg_psy);
			dev_err(g_bcdev->dev, "Could not get sgm_charger power_supply\n");
			return rc;
		}
	}
	rc = power_supply_get_property(g_bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_PRESENT, &val);
	if(rc < 0){
		pr_err("Couldn't get sgm_chg_psy present\n");
		return rc;
	}
	if(val.intval){
		if(sgm_typec_charger_ok()){
			rc = power_supply_get_property(g_bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_TYPE, &val);
			if(rc < 0){
				pr_err("Couldn't get sgm_chg_psy type\n");
				return rc;
			}
			if(val.intval == POWER_SUPPLY_TYPE_USB || val.intval == POWER_SUPPLY_TYPE_USB_CDP){
				rc = read_property_id(g_bcdev, &g_bcdev->psy_list[PSY_TYPE_USB], USB_ADAP_TYPE);
				if(rc < 0)
					return rc;
				if(!(g_bcdev->psy_list[PSY_TYPE_USB].prop[USB_ADAP_TYPE] == POWER_SUPPLY_USB_TYPE_SDP ||
					g_bcdev->psy_list[PSY_TYPE_USB].prop[USB_ADAP_TYPE] == POWER_SUPPLY_USB_TYPE_CDP)){
					pr_buf_err("%s: sgm significant since it is sdp/cdp\n", __func__);
					typec_significant = false;
				}
			}
		}else{
			pr_buf_err("%s: sgm significant since it is present\n", __func__);
			typec_significant = false;
		}
	}
	if(typec_significant){
		rc = write_property_id(g_bcdev, &g_bcdev->psy_list[PSY_TYPE_BATTERY], BATT_CHARGING_STOPPED, stopped);
		if(rc < 0)
			return rc;
	}else{
		val.intval = !stopped;
		rc = power_supply_set_property(g_bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &val);
		if(rc < 0){
			pr_err("Couldn't set sgm_chg_psy charging_enabled\n");
			return rc;
		}
	}
	rc = count;
	return count;

}
static ssize_t charging_stopped_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int rc;
	bool stopped = false;
	bool typec_significant = true;
	union power_supply_propval val = {0,};
	if(!g_bcdev->sgm_chg_psy){
		g_bcdev->sgm_chg_psy = power_supply_get_by_name("sgm_charger");
		if(!g_bcdev->sgm_chg_psy){
			rc = PTR_ERR(g_bcdev->sgm_chg_psy);
			dev_err(g_bcdev->dev, "Could not get sgm_charger power_supply\n");
			return rc;
		}
	}
	rc = power_supply_get_property(g_bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_PRESENT, &val);
	if(rc < 0){
		pr_err("Couldn't get sgm_chg_psy present\n");
		return rc;
	}
	if(val.intval){
		if(sgm_typec_charger_ok()){
			rc = power_supply_get_property(g_bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_TYPE, &val);
			if(rc < 0){
				pr_err("Couldn't get sgm_chg_psy type\n");
				return rc;
			}
			if(val.intval == POWER_SUPPLY_TYPE_USB || val.intval == POWER_SUPPLY_TYPE_USB_CDP){
				rc = read_property_id(g_bcdev, &g_bcdev->psy_list[PSY_TYPE_USB], USB_ADAP_TYPE);
				if(rc < 0)
					return rc;
				if(!(g_bcdev->psy_list[PSY_TYPE_USB].prop[USB_ADAP_TYPE] == POWER_SUPPLY_USB_TYPE_SDP ||
					g_bcdev->psy_list[PSY_TYPE_USB].prop[USB_ADAP_TYPE] == POWER_SUPPLY_USB_TYPE_CDP)){
					pr_buf_err("%s: sgm significant since it is sdp/cdp\n", __func__);
					typec_significant = false;
				}
			}
		}else{
			pr_buf_err("%s: sgm significant since it is present\n", __func__);
			typec_significant = false;
		}
	}
	if(typec_significant){
		rc = read_property_id(g_bcdev, &g_bcdev->psy_list[PSY_TYPE_BATTERY], BATT_CHARGING_STOPPED);
		if(rc < 0)
			return rc;
		stopped = g_bcdev->psy_list[PSY_TYPE_BATTERY].prop[BATT_CHARGING_STOPPED];
	}else{
		rc = power_supply_get_property(g_bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &val);
		if(rc < 0){
			pr_err("Couldn't get sgm_chg_psy charging_enabled\n");
			return rc;
		}
		stopped = !val.intval;
	}
	pr_buf_info("@@@@%s: charging_stopped is %d\n", __func__, stopped);
	return snprintf(buf, PAGE_SIZE, "%d\n",  stopped);
}
static struct kobj_attribute charging_stopped_attr = __ATTR_RW(charging_stopped);

static ssize_t charging_disabled_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int rc;
	bool disabled = false;
	bool typec_significant = true;
	unsigned long state = 0;
	union power_supply_propval val = {0,};

	rc = kstrtoul(buf, 10, &state);
	if(rc){
		pr_buf_err("kstrtoul operation failed  rc=%d\n", rc);
		return rc;
	}

	disabled = !!state;
	pr_buf_err("set charging_disabled to %d\n", disabled);
	if(!g_bcdev->sgm_chg_psy){
		g_bcdev->sgm_chg_psy = power_supply_get_by_name("sgm_charger");
		if(!g_bcdev->sgm_chg_psy){
			rc = PTR_ERR(g_bcdev->sgm_chg_psy);
			dev_err(g_bcdev->dev, "Could not get sgm_charger power_supply\n");
			return rc;
		}
	}
	rc = power_supply_get_property(g_bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_PRESENT, &val);
	if(rc < 0){
		pr_err("Couldn't get sgm_chg_psy present\n");
		return rc;
	}
	if(val.intval){
		if(sgm_typec_charger_ok()){
			rc = power_supply_get_property(g_bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_TYPE, &val);
			if(rc < 0){
				pr_err("Couldn't get sgm_chg_psy type\n");
				return rc;
			}
			if(val.intval == POWER_SUPPLY_TYPE_USB || val.intval == POWER_SUPPLY_TYPE_USB_CDP){
				rc = read_property_id(g_bcdev, &g_bcdev->psy_list[PSY_TYPE_USB], USB_ADAP_TYPE);
				if(rc < 0)
					return rc;
				if(!(g_bcdev->psy_list[PSY_TYPE_USB].prop[USB_ADAP_TYPE] == POWER_SUPPLY_USB_TYPE_SDP ||
					g_bcdev->psy_list[PSY_TYPE_USB].prop[USB_ADAP_TYPE] == POWER_SUPPLY_USB_TYPE_CDP)){
					pr_buf_err("%s: sgm significant since it is sdp/cdp\n", __func__);
					typec_significant = false;
				}
			}
		}else{
			pr_buf_err("%s: sgm significant since it is present\n", __func__);
			typec_significant = false;
		}
	}
	if(typec_significant){
		rc = write_property_id(g_bcdev, &g_bcdev->psy_list[PSY_TYPE_BATTERY], BATT_CHARGING_ENABLED, !disabled);
		if(rc < 0)
			return rc;
	}else{
		val.intval = !disabled;
		rc = power_supply_set_property(g_bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &val);
		if(rc < 0){
			pr_err("Couldn't set sgm_chg_psy charging_enabled\n");
			return rc;
		}
	}

	rc = count;
	return count;
}
static ssize_t charging_disabled_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int rc;
	bool disabled = false;
	bool typec_significant = true;
	union power_supply_propval val = {0,};
	if(!g_bcdev->sgm_chg_psy){
		g_bcdev->sgm_chg_psy = power_supply_get_by_name("sgm_charger");
		if(!g_bcdev->sgm_chg_psy){
			rc = PTR_ERR(g_bcdev->sgm_chg_psy);
			dev_err(g_bcdev->dev, "Could not get sgm_charger power_supply\n");
			return rc;
		}
	}
	rc = power_supply_get_property(g_bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_PRESENT, &val);
	if(rc < 0){
		pr_err("Couldn't get sgm_chg_psy present\n");
		return rc;
	}
	if(val.intval){
		if(sgm_typec_charger_ok()){
			rc = power_supply_get_property(g_bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_TYPE, &val);
			if(rc < 0){
				pr_err("Couldn't get sgm_chg_psy type\n");
				return rc;
			}
			if(val.intval == POWER_SUPPLY_TYPE_USB || val.intval == POWER_SUPPLY_TYPE_USB_CDP){
				rc = read_property_id(g_bcdev, &g_bcdev->psy_list[PSY_TYPE_USB], USB_ADAP_TYPE);
				if(rc < 0)
					return rc;
				if(!(g_bcdev->psy_list[PSY_TYPE_USB].prop[USB_ADAP_TYPE] == POWER_SUPPLY_USB_TYPE_SDP ||
					g_bcdev->psy_list[PSY_TYPE_USB].prop[USB_ADAP_TYPE] == POWER_SUPPLY_USB_TYPE_CDP)){
					pr_buf_err("%s: sgm significant since it is sdp/cdp\n", __func__);
					typec_significant = false;
				}
			}
		}else{
			pr_buf_err("%s: sgm significant since it is present\n", __func__);
			typec_significant = false;
		}
	}
	if(typec_significant){
		rc = read_property_id(g_bcdev, &g_bcdev->psy_list[PSY_TYPE_BATTERY], BATT_CHARGING_ENABLED);
		if(rc < 0)
			return rc;
		disabled = !g_bcdev->psy_list[PSY_TYPE_BATTERY].prop[BATT_CHARGING_ENABLED];
	}else{
		rc = power_supply_get_property(g_bcdev->sgm_chg_psy, POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &val);
		if(rc < 0){
			pr_err("Couldn't get sgm_chg_psy charging_enabled\n");
			return rc;
		}
		disabled = !val.intval;
	}
	pr_buf_info("@@@@%s: charging_disabled is %d\n", __func__, disabled);
	return snprintf(buf, PAGE_SIZE, "%d\n",  disabled);
}
static struct kobj_attribute charging_disabled_attr = __ATTR_RW(charging_disabled);

static ssize_t fast_charge_switch_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_bcdev->fast_charge_switch);
}

static ssize_t fast_charge_switch_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count){
	int rc = 0;
	char *quick_charger_yes[2] = {"QUICK_CHARGER=YES", NULL};
	char *quick_charger_no[2] = {"QUICK_CHARGER=NO", NULL};
	struct psy_state *usb_pst = &g_bcdev->psy_list[PSY_TYPE_USB];
	unsigned long state = 0;
	rc = kstrtoul(buf, 10, &state);
	if(rc){
		pr_buf_err("kstrtoul operation failed  rc=%d\n", rc);
		return rc;
	}
	g_bcdev->fast_charge_switch = !!state;
	pr_buf_err("set fast_charge_switch %d\n", g_bcdev->fast_charge_switch);
	rc = read_property_id(g_bcdev, usb_pst, USB_REAL_TYPE);
	if(rc < 0)
		return rc;
	if(!g_bcdev->fast_charge_switch){
		pr_buf_err("set fast_charge_switch QUICK_CHARGER=NO\n");
		kobject_uevent_env(&g_bcdev->dev->kobj, KOBJ_CHANGE, quick_charger_no);
	} else {
		if(usb_pst->prop[USB_REAL_TYPE] == QTI_POWER_SUPPLY_USB_TYPE_HVDCP ||
			usb_pst->prop[USB_REAL_TYPE] == QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3 ||
			usb_pst->prop[USB_REAL_TYPE] == QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5 ||
			usb_pst->prop[USB_REAL_TYPE] == POWER_SUPPLY_USB_TYPE_PD ||
			usb_pst->prop[USB_REAL_TYPE] == POWER_SUPPLY_USB_TYPE_PD_DRP ||
			usb_pst->prop[USB_REAL_TYPE] == POWER_SUPPLY_USB_TYPE_PD_PPS){
			pr_buf_err("set fast_charge_switch QUICK_CHARGER=YES\n");
			kobject_uevent_env(&g_bcdev->dev->kobj, KOBJ_CHANGE, quick_charger_yes);
		}
	}

	rc = count;
	return rc;
}
static struct kobj_attribute fast_charge_switch_attr = __ATTR_RW(fast_charge_switch);

static struct attribute *bc_sys_node_attrs[] = {
	&water_lpd_test_attr.attr,
	&fast_charge_status_attr.attr,
	&typec_cc_orientation_attr.attr,
	&charging_enabled_attr.attr,
	&safety_timer_enable_attr.attr,
	&charging_stopped_attr.attr,
	&charging_disabled_attr.attr,
	&fast_charge_switch_attr.attr,
	NULL,
};
static struct attribute_group bc_sys_node_attr_group = {
	.attrs = bc_sys_node_attrs,
};
#endif/*CONFIG_CHARGE_FG_FUNCTION*/
static int battery_chg_probe(struct platform_device *pdev)
{
	struct battery_chg_dev *bcdev;
	struct device *dev = &pdev->dev;
	struct pmic_glink_client_data client_data = { };
	int rc, i;

	bcdev = devm_kzalloc(&pdev->dev, sizeof(*bcdev), GFP_KERNEL);
	if (!bcdev)
		return -ENOMEM;

	bcdev->psy_list[PSY_TYPE_BATTERY].map = battery_prop_map;
	bcdev->psy_list[PSY_TYPE_BATTERY].prop_count = BATT_PROP_MAX;
	bcdev->psy_list[PSY_TYPE_BATTERY].opcode_get = BC_BATTERY_STATUS_GET;
	bcdev->psy_list[PSY_TYPE_BATTERY].opcode_set = BC_BATTERY_STATUS_SET;
	bcdev->psy_list[PSY_TYPE_USB].map = usb_prop_map;
	bcdev->psy_list[PSY_TYPE_USB].prop_count = USB_PROP_MAX;
	bcdev->psy_list[PSY_TYPE_USB].opcode_get = BC_USB_STATUS_GET;
	bcdev->psy_list[PSY_TYPE_USB].opcode_set = BC_USB_STATUS_SET;
	bcdev->psy_list[PSY_TYPE_WLS].map = wls_prop_map;
	bcdev->psy_list[PSY_TYPE_WLS].prop_count = WLS_PROP_MAX;
	bcdev->psy_list[PSY_TYPE_WLS].opcode_get = BC_WLS_STATUS_GET;
	bcdev->psy_list[PSY_TYPE_WLS].opcode_set = BC_WLS_STATUS_SET;
#ifdef CONFIG_CHARGE_FG_FUNCTION
	bcdev->fast_charge_switch = true;
#endif/*CONFIG_CHARGE_FG_FUNCTION*/

	for (i = 0; i < PSY_TYPE_MAX; i++) {
		bcdev->psy_list[i].prop =
			devm_kcalloc(&pdev->dev, bcdev->psy_list[i].prop_count,
					sizeof(u32), GFP_KERNEL);
		if (!bcdev->psy_list[i].prop)
			return -ENOMEM;
	}

	bcdev->psy_list[PSY_TYPE_BATTERY].model =
		devm_kzalloc(&pdev->dev, MAX_STR_LEN, GFP_KERNEL);
	if (!bcdev->psy_list[PSY_TYPE_BATTERY].model)
		return -ENOMEM;

	mutex_init(&bcdev->rw_lock);
	init_rwsem(&bcdev->state_sem);
	init_completion(&bcdev->ack);
	init_completion(&bcdev->fw_buf_ack);
	init_completion(&bcdev->fw_update_ack);
	INIT_WORK(&bcdev->subsys_up_work, battery_chg_subsys_up_work);
	INIT_WORK(&bcdev->usb_type_work, battery_chg_update_usb_type_work);
	INIT_WORK(&bcdev->battery_check_work, battery_chg_check_status_work);
#ifdef CONFIG_CHARGE_FG_FUNCTION
	INIT_DELAYED_WORK(&bcdev->qti_chg_status_work, qti_chg_status_update_work);
#endif /*CONFIG_CHARGE_FG_FUNCTION*/
	bcdev->dev = dev;

	rc = battery_chg_register_panel_notifier(bcdev);
	if (rc < 0)
		return rc;

	client_data.id = MSG_OWNER_BC;
	client_data.name = "battery_charger";
	client_data.msg_cb = battery_chg_callback;
	client_data.priv = bcdev;
	client_data.state_cb = battery_chg_state_cb;

	bcdev->client = pmic_glink_register_client(dev, &client_data);
	if (IS_ERR(bcdev->client)) {
		rc = PTR_ERR(bcdev->client);
		if (rc != -EPROBE_DEFER)
			dev_err(dev, "Error in registering with pmic_glink %d\n",
				rc);
		goto reg_error;
	}

	down_write(&bcdev->state_sem);
	atomic_set(&bcdev->state, PMIC_GLINK_STATE_UP);
	/*
	 * This should be initialized here so that battery_chg_callback
	 * can run successfully when battery_chg_parse_dt() starts
	 * reading BATT_CHG_CTRL_LIM_MAX parameter and waits for a response.
	 */
	bcdev->initialized = true;
	up_write(&bcdev->state_sem);

	bcdev->reboot_notifier.notifier_call = battery_chg_ship_mode;
	bcdev->reboot_notifier.priority = 255;
	register_reboot_notifier(&bcdev->reboot_notifier);

	rc = battery_chg_parse_dt(bcdev);
	if (rc < 0) {
		dev_err(dev, "Failed to parse dt rc=%d\n", rc);
		goto error;
	}

	bcdev->restrict_fcc_ua = DEFAULT_RESTRICT_FCC_UA;
	platform_set_drvdata(pdev, bcdev);
	bcdev->fake_soc = -EINVAL;
	rc = battery_chg_init_psy(bcdev);
	if (rc < 0)
		goto error;

	bcdev->battery_class.name = "qcom-battery";

	if (bcdev->wls_not_supported)
		bcdev->battery_class.class_groups = battery_class_no_wls_groups;
	else
		bcdev->battery_class.class_groups = battery_class_groups;

	rc = class_register(&bcdev->battery_class);
	if (rc < 0) {
		dev_err(dev, "Failed to create battery_class rc=%d\n", rc);
		goto error;
	}

	bcdev->wls_fw_update_time_ms = WLS_FW_UPDATE_TIME_MS;
	battery_chg_add_debugfs(bcdev);
	bcdev->notify_en = false;
	battery_chg_notify_enable(bcdev);
	device_init_wakeup(bcdev->dev, true);
	rc = register_extcon_conn_type(bcdev);
	if (rc < 0)
		dev_warn(dev, "Failed to register extcon rc=%d\n", rc);

	if (bcdev->connector_type == USB_CONNECTOR_TYPE_MICRO_USB) {
		bcdev->typec_class = qti_typec_class_init(bcdev->dev);
		if (IS_ERR_OR_NULL(bcdev->typec_class)) {
			dev_err(dev, "Failed to init typec class err=%d\n",
				PTR_ERR(bcdev->typec_class));
			return PTR_ERR(bcdev->typec_class);
		}
	}
#ifdef CONFIG_CHARGE_FG_FUNCTION
	bcdev->qti_batt_chg_ws = wakeup_source_register(bcdev->dev, "qti-battery-charger");
	schedule_delayed_work(&bcdev->qti_chg_status_work, msecs_to_jiffies(10000));
	rc = his_register_sysfs_attr_group(&bc_sys_node_attr_group);
	if(rc < 0){
		pr_buf_err("Couldn't create bc_sys_node_attr_group rc=%d\n", rc);
		goto error;
	}
	g_bcdev = bcdev;
#endif/*CONFIG_CHARGE_FG_FUNCTION*/
	schedule_work(&bcdev->usb_type_work);

	return 0;
error:
	down_write(&bcdev->state_sem);
	atomic_set(&bcdev->state, PMIC_GLINK_STATE_DOWN);
	bcdev->initialized = false;
	up_write(&bcdev->state_sem);

	pmic_glink_unregister_client(bcdev->client);
	cancel_work_sync(&bcdev->usb_type_work);
	cancel_work_sync(&bcdev->subsys_up_work);
	cancel_work_sync(&bcdev->battery_check_work);
	complete(&bcdev->ack);
	unregister_reboot_notifier(&bcdev->reboot_notifier);
reg_error:
	if (bcdev->notifier_cookie)
		panel_event_notifier_unregister(bcdev->notifier_cookie);
	return rc;
}

static int battery_chg_remove(struct platform_device *pdev)
{
	struct battery_chg_dev *bcdev = platform_get_drvdata(pdev);

	down_write(&bcdev->state_sem);
	atomic_set(&bcdev->state, PMIC_GLINK_STATE_DOWN);
	bcdev->initialized = false;
	up_write(&bcdev->state_sem);

	qti_typec_class_deinit(bcdev->typec_class);
	if (bcdev->notifier_cookie)
		panel_event_notifier_unregister(bcdev->notifier_cookie);

	device_init_wakeup(bcdev->dev, false);
	debugfs_remove_recursive(bcdev->debugfs_dir);
	class_unregister(&bcdev->battery_class);
	pmic_glink_unregister_client(bcdev->client);
	cancel_work_sync(&bcdev->subsys_up_work);
	cancel_work_sync(&bcdev->usb_type_work);
	cancel_work_sync(&bcdev->battery_check_work);
	unregister_reboot_notifier(&bcdev->reboot_notifier);

	return 0;
}

static const struct of_device_id battery_chg_match_table[] = {
	{ .compatible = "qcom,battery-charger" },
	{},
};

static struct platform_driver battery_chg_driver = {
	.driver = {
		.name = "qti_battery_charger",
		.of_match_table = battery_chg_match_table,
	},
	.probe = battery_chg_probe,
	.remove = battery_chg_remove,
};
module_platform_driver(battery_chg_driver);

MODULE_DESCRIPTION("QTI Glink battery charger driver");
MODULE_LICENSE("GPL v2");
