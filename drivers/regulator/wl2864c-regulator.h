
#ifndef __WL2864C_REGISTERS_H__
#define __WL2864C_REGISTERS_H__

/* Regulator Registers */
#define	WL2864C_REG_CHIP_REV			0x00
#define	WL2864C_REG_CHIP_ID				0x01
#define	WL2864C_REG_CURR_LIMT			0x01
#define	WL2864C_REG_DISCHARG_EN			0x02
#define	WL2864C_REG_LDO1_VOUT			0x03
#define	WL2864C_REG_LDO2_VOUT			0x04
#define	WL2864C_REG_LDO3_VOUT			0x05
#define	WL2864C_REG_LDO4_VOUT			0x06
#define	WL2864C_REG_LDO5_VOUT			0x07
#define	WL2864C_REG_LDO6_VOUT			0x08
#define	WL2864C_REG_LDO7_VOUT			0x09

#define	WL2864C_REG_LDO1_SEQ				0x0A
#define	WL2864C_REG_LDO2_SEQ				0x0A
#define	WL2864C_REG_LDO3_SEQ				0x0B
#define	WL2864C_REG_LDO4_SEQ				0x0B
#define	WL2864C_REG_LDO5_SEQ				0x0C
#define	WL2864C_REG_LDO6_SEQ				0x0C
#define	WL2864C_REG_LDO7_SEQ				0x0D

#define	WL2864C_REG_LDO_EN				0x0E
#define	WL2864C_REG_SEQ_CONF				0x0F

#define	WL2864C_MAX_REGISTER				0x10

#define WL2864C_MAX_REGULATORS	7

#define WL2864C_CURR_ILIM_MASK		0x03

#define WL2864C_LDO_VOUT_MASK		0xFF

#define WL2864C_LDO1_VOUT_MASK		0x24
#define WL2864C_LDO2_VOUT_MASK		0x30
#define WL2864C_LDO3_VOUT_MASK		0x80
#define WL2864C_LDO4_VOUT_MASK		0x80
#define WL2864C_LDO5_VOUT_MASK		0x80
#define WL2864C_LDO6_VOUT_MASK		0x80
#define WL2864C_LDO7_VOUT_MASK		0xA8

#define MAX_REG_NAME			20

/* WL2864C REGULATOR IDs */
enum {
	/* LDOs */
	WL2864C_ID_LDO1,
	WL2864C_ID_LDO2,
	WL2864C_ID_LDO3,
	WL2864C_ID_LDO4,
	WL2864C_ID_LDO5,
	WL2864C_ID_LDO6,
	WL2864C_ID_LDO7
};

enum {
	RWREG_OP_READ = 0,
	RWREG_OP_WRITE = 1,
};

static struct rwreg_operation_t {
	int type;			/*  0: read, 1: write */
	u8 reg;				/*  register */
	int len;			/*  read/write length */
	u8 val;				/*  length = 1; read: return value, write: op return */
	int ret;			/*  0: success, otherwise: fail */
	char *opbuf;		/*  length >= 1, read return value, write: op return */
} rw_buf;

typedef struct wl2864c_config_ldo_voltage {
	u8 reg;
	u8 reg_value;
} wl2864c_config_match;

static wl2864c_config_match wl2864c_ldo_table[] = {
	{.reg = WL2864C_REG_LDO1_VOUT , .reg_value = WL2864C_LDO1_VOUT_MASK},
	{.reg = WL2864C_REG_LDO2_VOUT , .reg_value = WL2864C_LDO2_VOUT_MASK},
	{.reg = WL2864C_REG_LDO3_VOUT , .reg_value = WL2864C_LDO3_VOUT_MASK},
	{.reg = WL2864C_REG_LDO4_VOUT , .reg_value = WL2864C_LDO4_VOUT_MASK},
	{.reg = WL2864C_REG_LDO5_VOUT , .reg_value = WL2864C_LDO5_VOUT_MASK},
	{.reg = WL2864C_REG_LDO6_VOUT , .reg_value = WL2864C_LDO6_VOUT_MASK},
	{.reg = WL2864C_REG_LDO7_VOUT , .reg_value = WL2864C_LDO7_VOUT_MASK},
};

struct wl2864c_regulator {
	struct regulator_desc desc;
	/* Current limiting */
	unsigned	n_current_limits;
	const int	*current_limits;
	unsigned char limit_mask;
	unsigned char conf;		/* curr limits register */

	struct device		*dev;
	struct regmap		*regmap;
	struct regulator_dev	*rdev;
	struct device_node	*of_node;


};

struct wl2864c {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_dev *rdev[WL2864C_MAX_REGULATORS];
	int enable_gpio;
};

static const int wl2864c_ldo1_2_curr_limits[] = {
	900000, 750000, 960000, 950000
};

static const int wl2864c_ldo3_4_curr_limits[] = {
	550000, 450000, 700000, 650000
};

static const int wl2864c_ldo5_6_curr_limits[] = {
	900000, 800000, 950000, 920000
};

static const int wl2864c_ldo7_curr_limits[] = {
	220000, 300000
};


#endif	/* __WL2864C_REGISTERS_H__ */

