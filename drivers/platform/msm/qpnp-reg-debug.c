/*
 * Copyright (c) 2017, All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MEretHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/his_debug_base.h>
#include <linux/spmi.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>
#include <linux/iio/iio.h>

#define _DEBUG_QPNP_REG_      0

/* PMIC vreg register list */
#define QPNP_REG_TYPE         0x04
#define QPNP_REG_SUB_TYPE     0x05
#define QPNP_REG_STATUS       0x08
#define QPNP_REG_STATUS2      0x09
#define QPNP_REG_VSET_LB      0x40
#define QPNP_REG_VSET_UB      0x41
#define QPNP_REG_MODE         0x45
#define QPNP_REG_EN           0x46
#define QPNP_REG_CT_LTD       0x4A

#define BIT_VREG_READY        7
#define BIT_VREG_NPM          7
#define BIT_QPNP_VREG_ON      7

#define QPNP_REG_SMPS_OFFS    0x300
#define QPNP_REG_LDO_OFFS     0x100

#define QPNP_GET_BIT(VAL, B)  ((VAL & BIT(B)) >> B)

enum qpnp_regulator_type {
	HFS_BULK        = 0x03,
	ELSE_BULK       = 0x1c,
	LOW_noise_LDO   = 0x04,
	LDO             = 0x21,
};

struct qpnp_vreg_status {
	u8 npm;
	u8 on_off;
	u8 vreg_ready;
	u32 volt;
	u8 mode;
	u8 ctrl;
};

struct qpnp_reg_dbg {
	struct device  *dev;
	struct regmap  *regmap;
	u16    ldo_base_addr;
	u16    smps_base_addr;
	u16    ldo_count;
	u16    smps_count;
	u8      pmic_id;
};

#define QPNP_PMIC_NUM      5

extern struct iio_dev *pmi_iio_dev;
extern struct iio_dev *pmic_iio_dev;

static int qpnp_register_num = 0;
static struct qpnp_reg_dbg *vreg_dev[QPNP_PMIC_NUM];

static int proc_created = 0;

static int qpnp_reg_read_u8(struct qpnp_reg_dbg *chip, u16 reg, u8 *data)
{
	int ret;

	ret = regmap_raw_read(chip->regmap, reg, data, 1);
	if (ret < 0)
		dev_err(chip->dev, "Error read reg: %X ret: %X\n", reg, ret);

	return ret;
}

static int reg_dbg_dump_reg_status(struct qpnp_reg_dbg *chip,
		struct qpnp_vreg_status *sts, u16 base_addr)
{
	int ret;
	u8 reg_val = 0;
	u8 reg_val1 = 0;
#if _DEBUG_QPNP_REG_
	u8 sub_type = 0;
	enum qpnp_regulator_type type;

	pr_err("Base Addr=%x\n", base_addr);
	ret = qpnp_reg_read_u8(chip, (base_addr + QPNP_REG_TYPE), (u8 *)&type);
	if (ret)
		return ret;
	pr_err("\ttype is %x\n", type);

	ret = qpnp_reg_read_u8(chip, (base_addr + QPNP_REG_SUB_TYPE), &sub_type);
	if (ret)
		return ret;
	pr_err("\tsub_type is %x\n", sub_type);
#endif

	ret = qpnp_reg_read_u8(chip, (base_addr + QPNP_REG_STATUS), &reg_val);
	if (ret)
		return ret;
	sts->vreg_ready = QPNP_GET_BIT(reg_val, BIT_VREG_READY);

	ret = qpnp_reg_read_u8(chip, (base_addr + QPNP_REG_MODE), &reg_val);
	if (ret)
		return ret;
	sts->mode = reg_val;

	ret = qpnp_reg_read_u8(chip, (base_addr + QPNP_REG_EN), &reg_val);
	if (ret)
		return ret;
	sts->ctrl = reg_val;
	sts->on_off = QPNP_GET_BIT(reg_val, BIT_QPNP_VREG_ON);
	sts->npm = (sts->mode  & BIT_VREG_NPM) == BIT_VREG_NPM;

	ret = qpnp_reg_read_u8(chip, (base_addr + QPNP_REG_VSET_LB), &reg_val);
	if (ret)
		return ret;

	ret = qpnp_reg_read_u8(chip, (base_addr + QPNP_REG_VSET_UB), &reg_val1);
	if (ret)
		return ret;

	sts->volt = ((reg_val1 & 0xF) << 8) | reg_val;
#if _DEBUG_QPNP_REG_
	pr_err("\t\nQPNP_REG_VSET_LB: %d QPNP_REG_VSET_UB: %d volt: %d\n",
			reg_val, reg_val1, sts->volt);
#endif

	return 0;
}

static void  vreg_common_show(struct seq_file *m, void *v)
{
	struct qpnp_reg_dbg *chip = NULL;
	struct qpnp_vreg_status reg_sts;
	int ret = 0, i = 0, j = 0;

	if (qpnp_register_num <= 0)
		return;

	for (j = 0; j < qpnp_register_num; j++) {
		chip = vreg_dev[j];
		PRINT_OUT(m, "\nPMIC ID %d\n", chip->pmic_id);
		PRINT_OUT(m, "SMPS Status:\n");
		PRINT_OUT(m, "NAME    ON  OK  VOL_PWM  MODE  CTL\n");
		for (i = 0; i < chip->smps_count; i++) {
			ret = reg_dbg_dump_reg_status(chip, &reg_sts,
					(chip->smps_base_addr + QPNP_REG_SMPS_OFFS * i));
			if (ret >= 0)
				PRINT_OUT(m, "VREG_S%d%3u%3u%10u  0x%2x  0x%2x\n", (i+1),
						reg_sts.on_off,
						reg_sts.vreg_ready,
						reg_sts.volt,
						reg_sts.mode,
						reg_sts.ctrl);
			else
				PRINT_OUT(m, "VREG_S%d status get failed\n", (i+1));

			memset(&reg_sts, 0, sizeof(reg_sts));
		}

		PRINT_OUT(m, "LDO Status:\n");
		PRINT_OUT(m, "NAME    \t  ON OK     VOL     MODE  CTL\n");
		for (i = 0; i < chip->ldo_count; i++) {
			ret = reg_dbg_dump_reg_status(chip, &reg_sts,
					(chip->ldo_base_addr + QPNP_REG_LDO_OFFS * i));
			if (ret >= 0)
				PRINT_OUT(m, "VREG_L%d  \t %3u%3u%8u    0x%2x  0x%2x\n",
						(i+1),
						reg_sts.on_off,
						reg_sts.vreg_ready,
						reg_sts.volt,
						reg_sts.mode,
						reg_sts.ctrl);
			else
				PRINT_OUT(m, "VREG_L%1d status get failed\n", (i+1));

			memset(&reg_sts, 0, sizeof(reg_sts));
		}
	}
}

static int vreg_proc_show(struct seq_file *m, void *v)
{

	vreg_common_show(m, v);
	return 0;
}

void vreg_status_sleep_show(void)
{
	vreg_common_show(NULL, NULL);
}

EXPORT_SYMBOL(vreg_status_sleep_show);

static int vreg_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, vreg_proc_show, NULL);
}

static const struct proc_ops vreg_proc_fops = {
	.proc_open		= vreg_proc_open,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
};

static int adc_proc_show(struct seq_file *m, void *v)
{
    struct iio_dev *iio_dev = NULL;
    int val = 0, fake = 0;
    int i = 0, ret = 0;

    seq_puts(m, "CHIP      CH                NAME          V(uv) or T\n");

    if(pmi_iio_dev != NULL &&
        pmi_iio_dev->info->read_raw != NULL) {
        iio_dev = pmi_iio_dev;
	    for (i = 0; i < iio_dev->num_channels; i++) {
            ret = iio_dev->info->read_raw(iio_dev, &iio_dev->channels[i],
                                    &val, &fake, IIO_CHAN_INFO_PROCESSED);
            if(ret < 0) {
                pr_err("%s  adc read failed\n", iio_dev->channels[i].extend_name);
                continue;
            }
            seq_printf(m, "PMI:    0x%x%23s%11lld\n",
					iio_dev->channels[i].channel, iio_dev->channels[i].extend_name, val);
        }
    }

    seq_puts(m, "CHIP      CH                NAME          RAW      V(uv) or T\n");

    if(pmic_iio_dev != NULL &&
        pmic_iio_dev->info->read_raw != NULL) {
        iio_dev = pmic_iio_dev;
	    for (i = 0; i < iio_dev->num_channels; i++) {
            ret = iio_dev->info->read_raw(iio_dev, &iio_dev->channels[i],
                                    &val, &fake, IIO_CHAN_INFO_PROCESSED);
            if(ret < 0) {
                pr_err("%s  adc read failed\n", iio_dev->channels[i].extend_name);
                continue;
            }
            seq_printf(m, "PMIC:    0x%x%23s%11lld\n",
					iio_dev->channels[i].channel, iio_dev->channels[i].extend_name, val);
        }
    }

    return 0;
}

static int adc_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, adc_proc_show, NULL);
}

static const struct proc_ops adc_proc_fops = {
	.proc_open		= adc_proc_open,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
};

static ssize_t vreg_status_dbg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct qpnp_reg_dbg *chip = dev_get_drvdata(dev);
	struct qpnp_vreg_status reg_sts;
	int ret, i;
	size_t size = 0;

	if (!chip)
		return 0;

	size += snprintf(buf + size, PAGE_SIZE, "SMPS Status:\n");
	size += snprintf(buf + size, PAGE_SIZE,
			"NAME    ON  OK  VOL_PWM  MODE  CTL\n");
	for (i = 0; i < chip->smps_count; i++) {
		ret = reg_dbg_dump_reg_status(chip, &reg_sts,
				(chip->smps_base_addr + QPNP_REG_SMPS_OFFS * i));
		if (ret >= 0)
			size += snprintf(buf + size, PAGE_SIZE,
					"VREG_S%d%3u%3u%10u  0x%2x  0x%2x\n",
					(i+1),
					reg_sts.on_off,
					reg_sts.vreg_ready,
					reg_sts.volt,
					reg_sts.mode,
					reg_sts.ctrl);
		else
			size += snprintf(buf + size, PAGE_SIZE,
					"VREG_S%d status get failed\n", (i+1));

		memset(&reg_sts, 0, sizeof(reg_sts));
	}

	size += snprintf(buf + size, PAGE_SIZE, "LDO Status:\n");
	size += snprintf(buf + size, PAGE_SIZE,
			"NAME   \t          ON OK NPM   VOL     MODE  CTL\n");
	for (i = 0; i < chip->ldo_count; i++) {
		ret = reg_dbg_dump_reg_status(chip, &reg_sts,
				(chip->ldo_base_addr + QPNP_REG_LDO_OFFS * i));
		if (ret >= 0)
			size += snprintf(buf + size, PAGE_SIZE,
					"VREG_L%d \t %3u%3u%3u%8u    0x%2x  0x%2x\n",
					(i+1),
					reg_sts.on_off,
					reg_sts.vreg_ready,
					reg_sts.npm,
					reg_sts.volt,
					reg_sts.mode,
					reg_sts.ctrl);
		else
			size += snprintf(buf + size, PAGE_SIZE,
					"VREG_L%1d status get failed\n", (i+1));

		memset(&reg_sts, 0, sizeof(reg_sts));
	}

	return size;
}
static DEVICE_ATTR(vreg_status, S_IRUGO, vreg_status_dbg_show, NULL);

static struct attribute *dbg_attrs[] = {
	&dev_attr_vreg_status.attr,
	NULL,
};

static struct attribute_group qpnp_dbg_attr_group = {
	.attrs = dbg_attrs,
};

static int qpnp_reg_dbg_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct qpnp_reg_dbg *chip;
	int ret;
	u32 temp_val;

	dev_err(&pdev->dev, "enter %s\n", __func__);
	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chip->regmap) {
		dev_err(&pdev->dev, "Couldn't get parent's regmap\n");
		return -EINVAL;
	}
	chip->dev = dev;

	chip->pmic_id =  to_spmi_device(pdev->dev.parent)->usid;

	ret = of_property_read_u32(node, "qcom,smps-base-addr", &temp_val);
	if (ret) {
		dev_err(dev, "Unable to read smps-base-addr\n");
		return ret;
	}
	chip->smps_base_addr = temp_val;

	ret = of_property_read_u32(node, "qcom,ldo-base-addr", &temp_val);
	if (ret) {
		dev_err(dev, "Unable to read ldo-base-addr\n");
		return ret;
	}
	chip->ldo_base_addr = temp_val;

	ret = of_property_read_u32(node, "qcom,smps-count", &temp_val);
	if (ret) {
		dev_err(dev, "Unable to read smps-count\n");
		return ret;
	}
	chip->smps_count = temp_val;

	ret = of_property_read_u32(node, "qcom,ldo-count", &temp_val);
	if (ret) {
		dev_err(dev, "Unable to read ldo-count\n");
		return ret;
	}
	chip->ldo_count = temp_val;

	pr_err("smps addr:0x%x ldo addr:0x%x smps count:0x%x ldo count:0x%x\n",
			chip->smps_base_addr, chip->ldo_base_addr,
			chip->smps_count, chip->ldo_count);

	ret = sysfs_create_group(&dev->kobj, &qpnp_dbg_attr_group);
	if (ret)
		pr_err("%s: failed to create sysfs group %d\n", __func__, ret);

	if (proc_created == 0) {
		proc_created = 1;
		ret = his_create_procfs_file("vreg_sts", 0, &vreg_proc_fops);
		if (ret)
		    dev_err(dev, "failed to create vreg_sts proc\n");

		ret = his_create_procfs_file("adc_sts", 0, &adc_proc_fops);
		if (ret)
			dev_err(dev, "failed to create adc_sts proc\n");
	}

	dev_set_drvdata(dev, chip);
	vreg_dev[qpnp_register_num++] = chip;

	return ret;
}

static int qpnp_reg_dbg_remove(struct platform_device *pdev)
{
	remove_proc_entry("vreg_sts", NULL);
	remove_proc_entry("adc_sts", NULL);

	dev_set_drvdata(&pdev->dev, NULL);
	return 0;
}

static const struct of_device_id spmi_match_table[] = {
	{.compatible = "qcom,qpnp-reg-dbg",	},
	{}
};

static struct platform_driver qpnp_reg_dbg_driver = {
	.driver		= {
		.name	= "qcom,qpnp-reg-dbg",
		.of_match_table = spmi_match_table,
	},
	.probe		= qpnp_reg_dbg_probe,
	.remove		= qpnp_reg_dbg_remove,
};
module_platform_driver(qpnp_reg_dbg_driver);

MODULE_DESCRIPTION("qpnp reg debug driver");
MODULE_AUTHOR("Andrew.C.Lee");
MODULE_LICENSE("GPL v2");
