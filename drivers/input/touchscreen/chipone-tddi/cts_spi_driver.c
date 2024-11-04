#define LOG_TAG         "SPIDrv"

#include <linux/spi/spi.h>
#include <linux/of.h>
#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"
#include "cts_driver.h"
#include "cts_strerror.h"

#ifdef CONFIG_CTS_SPI_HOST

static struct spi_device *chipone_spi_dev = NULL;

struct workqueue_struct *cts_probe_workqueue;
struct delayed_work cts_probe_delay_work;

static void chipone_probe_delay_work(struct work_struct *work)
{
	if (chipone_spi_dev == NULL)
		return;
	cts_driver_probe(&chipone_spi_dev->dev, CTS_SPI_BUS);
}

static int cts_spi_driver_probe(struct spi_device *spi_dev)
{
    if (spi_dev == NULL) {
        cts_info("Spi driver probe with spi_dev = NULL");
        return -EINVAL;
    }

    cts_info("Probe spi device '%s': "
             "mode='%u' speed=%u bits_per_word=%u "
             "chip_select=%u, cs_gpio=%d irq=%d",
        spi_dev->modalias,
        spi_dev->mode, spi_dev->max_speed_hz, spi_dev->bits_per_word,
        spi_dev->chip_select, spi_dev->cs_gpio, spi_dev->irq);

    //return cts_driver_probe(&spi_dev->dev, CTS_SPI_BUS);
    chipone_spi_dev = spi_dev;
	cts_probe_workqueue = create_singlethread_workqueue("chipone_probe_wq");
	INIT_DELAYED_WORK(&cts_probe_delay_work, chipone_probe_delay_work);
	queue_delayed_work(cts_probe_workqueue, &cts_probe_delay_work,
					usecs_to_jiffies(8000000));

	return 0;
}

static int cts_spi_driver_remove(struct spi_device *spi_dev)
{
    if (spi_dev == NULL) {
        cts_info("Spi driver remove with spi_dev = NULL");
        return -EINVAL;
    }

    return cts_driver_remove(&spi_dev->dev);
}

static int cts_spi_pm_suspend(struct device *dev)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	cts_info("cts_spi_pm_suspend\n");

	cts_data->pdata->cts_sys_suspend = true;
	return 0;
}

static int cts_spi_pm_resume(struct device *dev)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	cts_info("cts_spi_pm_resume\n");
	cts_data->pdata->cts_sys_suspend = false;
	wake_up_interruptible(&cts_data->pdata->irq_waitq_head);
	return 0;
}

static const struct dev_pm_ops cts_spi_dev_pm_ops = {
	.suspend = cts_spi_pm_suspend,
	.resume  = cts_spi_pm_resume,
};
static const struct spi_device_id cts_spi_device_id_table[] = {
    {CFG_CTS_DEVICE_NAME, 0},
    {}
};

struct spi_driver cts_spi_driver = {
    .probe = cts_spi_driver_probe,
    .remove = cts_spi_driver_remove,
    .driver = {
        .name = CFG_CTS_DRIVER_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_CTS_OF
        .of_match_table = of_match_ptr(cts_driver_of_match_table),
#endif /* CONFIG_CTS_OF */
#ifdef CONFIG_CTS_SYSFS
        .groups = cts_driver_config_groups,
        .pm = &cts_spi_dev_pm_ops,
#endif /* CONFIG_CTS_SYSFS */
    },
    .id_table = cts_spi_device_id_table,
};
#endif /* CONFIG_CTS_SPI_HOST */

