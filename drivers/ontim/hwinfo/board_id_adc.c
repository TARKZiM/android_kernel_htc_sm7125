/*************************************************************************
    > File Name: board_id_qcom_adc5.c
    > Author: jiangfuxiong
    > Mail: fuxiong.jiang@ontim.cn
    > Created Time: 2019年12月10日 星期二 20时26分00秒
 ************************************************************************/

#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include "board_id_adc.h"

struct adc_board_id *chip_adc;

#ifndef QCOM_ADC5_TM
struct qpnp_vadc_result adc_result;
int get_board_id_voltage(struct adc_board_id *chip,int *voltage)
{
	int rc;
	rc = qpnp_vadc_read(chip->vadc_dev, chip->vadc_mux, &adc_result);
	if (rc) {
		pr_err("ontim: %s: qpnp_vadc_read failed(%d)\n",
				__func__, rc);
		return rc;
	} else {
		*voltage = (int)adc_result.physical / 1000;
		//pr_info("ontim:adc read voltage=%d,adc_result=%lld\n", *voltage, adc_result.physical);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(get_board_id_voltage);

static int board_id_parse_dt(struct device *dev, struct adc_board_id *chip)
{
	struct device_node *of_node = dev->of_node;
	u32 data;
	int ret = 0;
	ret = of_property_read_u32(of_node,"qcom,board-id-adc-channel-num", &data);
	if (ret) {
		pr_info("ontim:%s: board id channel not found\n", __func__);
	} else {
		chip->vadc_mux = data;
		pr_info("ontim:%s: board id channel chip->vadc_mux=%d\n",__func__, chip->vadc_mux);
	}
	return 0;
}
#else
int get_board_id_ohm(struct adc_board_id *chip, u32 *board_id_ohm)
{
	int rc;

	rc = iio_read_channel_processed(chip->board_id_channel, &chip->voltage);
	if (rc < 0) {
		pr_err("Error in reading board id channel, rc:%d\n", rc);
		return rc;
	}

	chip->voltage = div_s64(chip->voltage, 1000);
	if (chip->voltage == 0) {
		pr_err("chip->voltage = 0 from ADC\n");
		return 0;
	}
#if 0
	chip->denom = div64_s64(BID_VREF_MV * 1000, chip->voltage) - 1000;
	if (chip->denom <= 0) {
		/*  board id connector might be open, return 0 kohms */
		pr_err("chip->donom = 0 from board id ADC\n");
		return 0;
	}
	*board_id_ohm = div64_u64(BID_RPULL_OHM * 1000 + (chip->denom) / 2, (chip->denom));
#else
	*board_id_ohm = div64_u64(BID_RPULL_OHM * chip->voltage, (BID_VREF_MV - chip->voltage));
#endif
	pr_info("ontim chip->voltage=%d, chip->board_id_ohm=%d\n",chip->voltage, *board_id_ohm);
	return 0;
}
EXPORT_SYMBOL_GPL(get_board_id_ohm);
#endif

static ssize_t board_id_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct adc_board_id *chip = dev_get_drvdata(dev);
	int value = 0;

#ifndef QCOM_ADC5_TM
	value = get_board_id_voltage(chip, &chip->voltage);
	if (value < 0) {
		pr_err("Failed to detect board_id rc=%d\n", value);
	}
	value = chip->voltage;
#else
	value = get_board_id_ohm(chip, &chip->board_id_ohm);
	if (value < 0) {
		pr_err("Failed to detect board_id rc=%d\n", value);
	}
	pr_info("ontim:%s board_id_ohm=%d Ohm\n",__func__,chip->board_id_ohm);
	value = (int)chip->board_id_ohm;
#endif
	pr_err("ontim: adc read value=%d\n",value);
	return scnprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t board_id_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	//struct board_id *data = dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
#if 0
	if (0 == data->user_enabled) {
		dev_dbg(dev, "user space can NOT set the state at this moment.\n");
		return -1;
	}
#endif
	ret = kstrtoul(buf, 16, &value);
	if (ret < 0) {
		dev_err(dev, "%s:kstrtoul failed, ret=0x%x\n",
				__func__, ret);
		return ret;
	}

	dev_dbg(dev, "set value %ld\n", value);
	if (1 == value) {
	} else if (0 == value) {
	} else {
		dev_err(dev, "set a invalid value %ld\n", value);
	}
	return size;
}

static DEVICE_ATTR(enable, 0664, board_id_enable_show, board_id_enable_store);

static int board_id_probe(struct platform_device *pdev)
{
	struct adc_board_id *chip;
	int rc = 0;
	int err = 0;

	//pr_err(" %s start\n",__func__);
	chip = devm_kzalloc(&pdev->dev,
		sizeof(struct adc_board_id), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, chip);
#ifndef QCOM_ADC5_TM
	if (pdev->dev.of_node) {
		err = board_id_parse_dt(&pdev->dev, chip);
		if (err < 0) {
			dev_err(&pdev->dev, "Failed to parse device tree\n");
			return -ENOMEM;
		}
	} else if (pdev->dev.platform_data != NULL) {
		memcpy(chip, pdev->dev.platform_data, sizeof(*chip));
	} else {
		dev_err(&pdev->dev, "No valid platform data.\n");
		return -ENOMEM;
	}

	chip->vadc_dev = qpnp_get_vadc(&pdev->dev, "board_id");
	if (IS_ERR(chip->vadc_dev)) {
		rc = PTR_ERR(chip->vadc_dev);
		dev_err(&pdev->dev,"ontim:  vadc property missing\n");
		if (rc != -EPROBE_DEFER)
			dev_err(&pdev->dev,"ontim: vadc property missing\n");
		goto err_board_id;
	}

	rc = get_board_id_voltage(chip, &chip->voltage);
	if (rc < 0) {
		dev_err(&pdev->dev,"Failed to detect board_id rc=%d\n", rc);
		goto err_board_id;
	}

#else
	chip->board_id_channel = iio_channel_get(&pdev->dev, "board_id");
	if (IS_ERR(chip->board_id_channel)) {
		rc = PTR_ERR(chip->board_id_channel);
		if (rc != -EPROBE_DEFER)
			dev_err(&pdev->dev,"board-id channel unavailable, rc=%d\n", rc);
		chip->board_id_channel = NULL;
		goto err_board_id;
	}
	if (!chip->board_id_channel)
		return -EINVAL;

	rc = get_board_id_ohm(chip, &chip->board_id_ohm);
	if (rc < 0) {
		dev_err(&pdev->dev,"Failed to detect board_id rc=%d\n", rc);
		goto err_board_id;
	}

#endif

	chip_adc = chip;

	err =  device_create_file(&pdev->dev, &dev_attr_enable);
	if (err) {
		dev_err(&pdev->dev, "create enable sys-file failed,\n");
		goto err_board_id;
	}

	//pr_err("%s end\n",__func__);
	return 0;

err_board_id:
	devm_kfree(&pdev->dev, chip);
	return rc;
}

static int board_id_remove(struct platform_device *pdev)
{
//	struct board_id *data = dev_get_drvdata(pdev);
//	devm_kfree(&pdev->dev, data);
//	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct of_device_id platform_match_table[] = {
	     { .compatible = "ontim,board-id-adc",},
	     { },
	 };

static struct platform_driver board_id_driver = {
	.driver = {
		.name = "board_id",
		.of_match_table = platform_match_table,
		.owner = THIS_MODULE,
	},
	.probe = board_id_probe,
	.remove = board_id_remove,
	//.id_table = board_id_id,
};

static int __init board_id_init(void)
{
	return platform_driver_register(&board_id_driver);
}

static void __exit board_id_exit(void)
{
	return platform_driver_unregister(&board_id_driver);
}

module_init(board_id_init);
module_exit(board_id_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("board_id driver");
