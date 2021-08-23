
#ifndef _CAM_SENSOR_EFUSE_ID_H_
#define _CAM_SENSOR_EFUSE_ID_H_

/*                                       otp_sid    sid
#define SENSER_ID_ov16b10  0x1642        0xA0       0x20
#define SENSER_ID_s5k3p9   0x3109        0xA0       0x20
#define SENSER_ID_s5k5e9   0x559B        0x5A       0x5A
#define SENSER_ID_ov8856   0x885A        0xa2       0x6c
 */

#define EEPROM_SID_s5k3p9   0xA0
#define EEPROM_SID_s5kgw1sp   0xB0
 
#define SENSER_ID_s5k3p9            0x3109  // front
#define SENSER_ID_ofilm_s5k3p9      0x3209 // front  ofilm_s5k3p9 
#define SENSER_ID_truly_s5k3p9      0x3309 // front  truly_s5k3p9 
#define SENSER_ID_ov16b10  			0x1642  // back
#define SENSER_ID_ov8856   			0x885A  // backaux



/*********      imx        ********/
#define SENSER_ID_imx350   0x02
#define SENSER_ID_imx486   0x486


/*********      ov        ********/
#define SENSER_ID_ov2281   0x56     
#define SENSER_ID_ov12a10  0x1241   

/*********      s5k        ********/
#define SENSER_ID_s5k2x7   0x2187


#define SENSER_ID_s5kgw1sp  0x971  // lgit_s5kgw1sp     cam0
#define SENSER_ID_s5kgm2sp  0x08D2 // lgit_s5kgm2sp     cam0 
#define SENSER_ID_hi1634    0x1634 // cowell_hi1634     cam1
#define SENSER_ID_s5kgd1sp  0x0841 // cowell_s5kgd1sp   cam1
#define SENSER_ID_gc2375h   0x2375 // sunrise_gc2375h   cam2
#define SENSER_ID_gc02m1b   0x02e0 // sunrise_gc02m1b   cam2
#define SENSER_ID_hi556     0x0556 // synology_hi556    cam3
#define SENSER_ID_s5k5e9   	0x559b // synology_s5k5e9   cam4

#define BOARD_ID_PATH  "/sys/hwinfo/board_id"


#define  WRITE_OTP_ADDR(reg_addr, reg_data)\
    cam_sensor_UpdateSettings(reg_addr, CAMERA_SENSOR_I2C_TYPE_WORD, \
    reg_data, CAMERA_SENSOR_I2C_TYPE_BYTE, &i2c_reg_settings);\
    camera_io_dev_write(&s_ctrl->io_master_info, &i2c_reg_settings);

#define  WRITE_OTP_ADDR_BYTE(reg_addr, reg_data)\
    cam_sensor_UpdateSettings(reg_addr, CAMERA_SENSOR_I2C_TYPE_BYTE, \
    reg_data, CAMERA_SENSOR_I2C_TYPE_BYTE, &i2c_reg_settings);\
    camera_io_dev_write(&s_ctrl->io_master_info, &i2c_reg_settings);

#define  READ_OTP_ADDR(reg_addr, pu32_buf)\
    camera_io_dev_read(&s_ctrl->io_master_info, reg_addr, pu32_buf, \
    CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);

#define  READ_OTP_ADDR_BYTE(reg_addr, pu32_buf)\
    camera_io_dev_read(&s_ctrl->io_master_info, reg_addr, pu32_buf, \
    CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);

#define  READ_EFUSE_ID(head_addr, cnt)\
for(i=0; i<cnt; i++)\
{\
    READ_OTP_ADDR(head_addr + i, (uint32_t *)&hex_efuseid)\
    sprintf(str_efuseid+2*i,"%02x", hex_efuseid);\
}

int32_t get_CameName_and_EfuseId(struct cam_sensor_ctrl_t *s_ctrl);

int8_t get_s5k3p9_supplier(struct cam_sensor_ctrl_t *s_ctrl);

int8_t cam_read_board_id(void);
#endif


