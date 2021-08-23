/*************************************************************************
    > File Name: board_id_adc.h
    > Author: jiangfuxiong
    > Mail: fuxiong.jiang@ontim.cn
    > Created Time: 2018年09月18日 星期二 15时33分33秒
 ************************************************************************/

#ifndef _LINUX_BOARD_ID_ADC_H_
#define _LINUX_BOARD_ID_ADC_H_

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>

#define QCOM_ADC5_TM

#define BID_VREF_MV		1875
#define BID_RPULL_OHM		100000

//#define ADC_ERROR(fmt, args...) printk(KERN_ERR "[ADC_BOARD_ID][Error]"fmt"\n", ##args)

struct adc_board_id {
	int voltage;
#ifndef QCOM_ADC5_TM
	struct qpnp_vadc_chip *vadc_dev;
	int vadc_mux;
#else
	struct iio_channel *board_id_channel;
	int64_t denom;
	u32 board_id_ohm;
#endif
};

#ifndef QCOM_ADC5_TM
#include <linux/qpnp/qpnp-adc.h>
//extern struct qpnp_vadc_result adc_result;
int get_board_id_voltage(struct adc_board_id *chip, int *voltage);
#else
#include <dt-bindings/iio/qcom,spmi-vadc.h>
#include <linux/iio/consumer.h>
int get_board_id_ohm(struct adc_board_id *chip, u32 *board_id_ohm);
#endif

extern struct adc_board_id *chip_adc;

#endif
