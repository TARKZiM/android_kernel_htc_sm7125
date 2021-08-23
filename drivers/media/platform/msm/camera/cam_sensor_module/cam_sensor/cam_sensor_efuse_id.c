
#include <linux/module.h>
#include <cam_sensor_cmn_header.h>
#include "cam_sensor_core.h"
#include "cam_sensor_util.h"
#include "cam_soc_util.h"
#include "cam_trace.h"

#include "cam_sensor_efuse_id.h"

/* #define USE_MY_DBG
#define USE_MY_ERR
#include <linux/yzm_dbg.h>
#undef FILE_NAME
#define  FILE_NAME  "cam_sensor_efuse_id.c"
 */
extern char front_cam_name[64];// 
//extern char frontaux_cam_name[64];//  s5k5e9
extern char back_cam_name[64];//  
extern char backaux_cam_name[64];// 
extern char backaux2_cam_name[64];// 
extern char backaux3_cam_name[64];// 


extern char front_cam_efuse_id[64];// s5k3p9
extern char frontaux_cam_efuse_id[64];// s5k5e9
extern char back_cam_efuse_id[64];//  ov16e10
extern char backaux_cam_efuse_id[64];// ov8856
extern char backaux2_cam_efuse_id[64];// ov8856

char cam_board_id[64] = {1};

static void cam_sensor_UpdateSettings(
uint32_t reg_addr,
enum camera_sensor_i2c_type addr_type,
uint32_t reg_data,
enum camera_sensor_i2c_type data_type,
struct cam_sensor_i2c_reg_setting  *i2c_reg_settings)
{
	struct cam_sensor_i2c_reg_array    *i2c_reg_array = i2c_reg_settings->reg_setting;
	
	i2c_reg_settings->size = 1;
	i2c_reg_settings->delay = 0;
	i2c_reg_settings->addr_type = addr_type;
	i2c_reg_settings->data_type = data_type;

	i2c_reg_array->reg_addr = reg_addr;
	i2c_reg_array->reg_data = reg_data;

}

int8_t cam_read_board_id(void)
{
    int8_t ret = 0;
    struct file *filp = NULL;
    mm_segment_t old_fs;
    loff_t pos;

 	filp = filp_open(BOARD_ID_PATH, O_RDONLY, 0);
  	if (IS_ERR(filp)) {
  		pr_err("%s: open %s failed!", __func__, BOARD_ID_PATH);
  		return -EINVAL;
  	}

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    ret = vfs_read(filp, cam_board_id, sizeof(cam_board_id) , &pos);
    if (ret < 0)
        pr_err("read file fail");
    CAM_DBG(CAM_SENSOR,"cam_%s file len:%d read len:%d pos:%d", cam_board_id, sizeof(cam_board_id), ret, (u32)pos);
    filp_close(filp, NULL);
    set_fs(old_fs);

    return ret;
}

static int8_t get_efuse_id_gc02m1b
	(struct cam_sensor_ctrl_t *s_ctrl, uint32_t SensorId, char * pstr_efuse_id)
{
    uint8_t i = 0;
    uint32_t reg_info;
    uint8_t otp_info[32] = {0};
    struct cam_sensor_i2c_reg_setting  i2c_reg_settings;
    struct cam_sensor_i2c_reg_array    i2c_reg_array;
    
    i2c_reg_settings.reg_setting = &i2c_reg_array;

    WRITE_OTP_ADDR_BYTE(0xfe, 0x00);
    WRITE_OTP_ADDR_BYTE(0xfc, 0x01);
    WRITE_OTP_ADDR_BYTE(0xf4, 0x41);
    WRITE_OTP_ADDR_BYTE(0xf5, 0xc0);
    WRITE_OTP_ADDR_BYTE(0xf6, 0x44);
    WRITE_OTP_ADDR_BYTE(0xf8, 0x38);
    WRITE_OTP_ADDR_BYTE(0xf9, 0x82);
    WRITE_OTP_ADDR_BYTE(0xfa, 0x00);
    WRITE_OTP_ADDR_BYTE(0xfd, 0x80);
    WRITE_OTP_ADDR_BYTE(0xfc, 0x81);
    WRITE_OTP_ADDR_BYTE(0xf7, 0x01);
    WRITE_OTP_ADDR_BYTE(0xfc, 0x80);
    WRITE_OTP_ADDR_BYTE(0xfc, 0x80);
    WRITE_OTP_ADDR_BYTE(0xfc, 0x80);
    WRITE_OTP_ADDR_BYTE(0xfc, 0x8e);
    WRITE_OTP_ADDR_BYTE(0xf3, 0x30);
    WRITE_OTP_ADDR_BYTE(0xfe, 0x02);

    for(i = 0; i < 32; i++){
        WRITE_OTP_ADDR_BYTE(0x17, i*8);
        WRITE_OTP_ADDR_BYTE(0xf3, 0x34);
        READ_OTP_ADDR_BYTE(0x19, &reg_info);

        otp_info[i] = reg_info&0xFF;
        //CAM_ERR(CAM_SENSOR,"otp_info[%d] = 0x%x\n", i, otp_info[i]);
    }

    WRITE_OTP_ADDR_BYTE(0xf7, 0x00);
    WRITE_OTP_ADDR_BYTE(0xfe, 0x00);

    if((otp_info[18] == 0x14)&&(otp_info[17] == 0x00))
    {
        CAM_ERR(CAM_SENSOR,"otp_info[18] == 0x14");
        for(i = 0; i < 5; i++)
        {
            sprintf(pstr_efuse_id+2*i,"%02x",0x00);
        }

    }else
    {
        for(i = 0; i < 5; i++)
        {
            sprintf(pstr_efuse_id+2*i,"%02x",otp_info[16+i]);
        }
    }
    CAM_ERR(CAM_SENSOR,"pstr_efuse_id=%s strlen=%lu", pstr_efuse_id, strlen(pstr_efuse_id));

    return 0;
}

static int8_t get_efuse_id_s5k3p9_ByEeprom
	(struct cam_sensor_ctrl_t *s_ctrl, uint32_t SensorId, char * pstr_efuse_id)
{
    uint16_t dummy_sid = 0;
    uint8_t i = 0;
    char hex_efuseid;
    char str_efuseid[64]={0};
    
    uint32_t head_addr = 0x0015;
    uint8_t efuse_id_size = 6;
    
    dummy_sid = s_ctrl->io_master_info.cci_client->sid;
    if(SENSER_ID_s5kgm2sp == SensorId)
    {
        s_ctrl->io_master_info.cci_client->sid = EEPROM_SID_s5k3p9 >> 1;
        head_addr = 0x000A;
        efuse_id_size = 20;
    }
    else if(SENSER_ID_s5kgw1sp == SensorId)
    {
        s_ctrl->io_master_info.cci_client->sid = EEPROM_SID_s5kgw1sp >> 1;
        head_addr = 0x000A;
    }
    else if ((SENSER_ID_s5kgd1sp == SensorId) || (SENSER_ID_hi1634 == SensorId))
    {
        s_ctrl->io_master_info.cci_client->sid = EEPROM_SID_s5k3p9 >> 1;
        head_addr = 0x000A;
    }
    else
    {
        s_ctrl->io_master_info.cci_client->sid = EEPROM_SID_s5k3p9 >> 1;
    }
    READ_EFUSE_ID(head_addr, efuse_id_size)
    strcpy(pstr_efuse_id, str_efuseid);
    CAM_ERR(CAM_SENSOR,"pstr_efuse_id=%s strlen=%lu", pstr_efuse_id, strlen(pstr_efuse_id));
    
    s_ctrl->io_master_info.cci_client->sid = dummy_sid;
    
    return 0;
}

static int8_t get_efuse_id_s5k(struct cam_sensor_ctrl_t *s_ctrl, uint32_t SensorId, char * pstr_efuse_id) 
{

    char hex_efuseid;
    char str_efuseid[64]={0};
    uint8_t i;
    
    uint32_t page_number = 0;
    uint32_t head_addr = 0;
    uint8_t efuse_id_size = 0;
    
    struct cam_sensor_i2c_reg_setting  i2c_reg_settings;
    struct cam_sensor_i2c_reg_array    i2c_reg_array;
    
    i2c_reg_settings.reg_setting = &i2c_reg_array;
    
    // 8B use for 5E9
    if(SensorId == SENSER_ID_s5k5e9)
    {
        head_addr = 0x0A04;
        efuse_id_size = 8;
    }
    // 6B use for 3p9 2x7 
    else if(SensorId == SENSER_ID_s5k2x7)
    {
        head_addr = 0x0A24;
        efuse_id_size = 6;
    }
    else if(SensorId == SENSER_ID_gc02m1b)
    {
        get_efuse_id_gc02m1b(s_ctrl, SensorId, pstr_efuse_id);
        return 0;
    }
    else if(SensorId == SENSER_ID_ofilm_s5k3p9)
    {
        get_efuse_id_s5k3p9_ByEeprom(s_ctrl, SensorId, pstr_efuse_id);

        return 0;
    }
    else if(SensorId == SENSER_ID_truly_s5k3p9)
    {
        //get_efuse_id_truly_s5k3p9_ByEeprom(s_ctrl, pstr_efuse_id);

        return 0;
    }
    else if((SENSER_ID_s5kgw1sp == SensorId) ||
                (SENSER_ID_s5kgm2sp == SensorId) ||
                (SENSER_ID_s5kgd1sp == SensorId) ||
                (SENSER_ID_hi1634 == SensorId))
    {
        CAM_INFO(CAM_SENSOR, "s5kgw1sp: 0x%x", SensorId);
        get_efuse_id_s5k3p9_ByEeprom(s_ctrl, SensorId, pstr_efuse_id);
        return 0;
        page_number = 0x0000;
        head_addr = 0x0A24;
        efuse_id_size = 6;
    }
    else
    {
        CAM_ERR(CAM_SENSOR, "sensor id error   0x%x", SensorId);
        return -1;
    }

    WRITE_OTP_ADDR(0x0100, 0x01)// Streaming ON
    msleep(50);
    WRITE_OTP_ADDR(0x0A02, page_number)  // Write OTP Page
    

    WRITE_OTP_ADDR(0x0A00, 0x01)//  Read CMD
    msleep(1);

    READ_EFUSE_ID(head_addr, efuse_id_size)
    
    strcpy(pstr_efuse_id, str_efuseid);
    CAM_ERR(CAM_SENSOR,"pstr_efuse_id=%s strlen=%lu", pstr_efuse_id, strlen(pstr_efuse_id));
    
    WRITE_OTP_ADDR(0x0A00, 0x04)// Clear error bits
    WRITE_OTP_ADDR(0x0A00, 0x00)// Initial command

    return 0;
}



static int32_t get_efuse_id_ov(struct cam_sensor_ctrl_t *s_ctrl, char * pstr_efuse_id) 
{

    char hex_efuseid;
    char str_efuseid[33]={0};

    uint8_t i;
    
    struct cam_sensor_i2c_reg_setting  i2c_reg_settings;
    struct cam_sensor_i2c_reg_array    i2c_reg_array;
    
    
    i2c_reg_settings.reg_setting = &i2c_reg_array;
    
    WRITE_OTP_ADDR(0x0100, 0x01)
    WRITE_OTP_ADDR(0x3D84, 0x40)  
/*     WRITE_OTP_ADDR(0x3D88, 0x7000)
    WRITE_OTP_ADDR(0x3D8A, 0x700F) */
    WRITE_OTP_ADDR(0x3D88, 0x00)
    WRITE_OTP_ADDR(0x3D89, 0x70)
    WRITE_OTP_ADDR(0x3D8A, 0x0F)
    WRITE_OTP_ADDR(0x3D8B, 0x70)
    
    WRITE_OTP_ADDR(0x0100, 0x01)

    // Read Chip ID
    for(i=0; i<16; i++)
    {
        READ_OTP_ADDR(0x7000 + i, (uint32_t *)&hex_efuseid)
        //CAM_DBG(CAM_SENSOR, "i=%d   hex_efuseid=0x%x ", i, hex_efuseid);
        sprintf(str_efuseid+2*i,"%02x", hex_efuseid);
		msleep(1);
    }
    //CAM_DBG(CAM_SENSOR, "str_efuseid=%s", str_efuseid);

    strcpy(pstr_efuse_id, str_efuseid); 
    CAM_ERR(CAM_SENSOR,"pstr_efuse_id=%s strlen=%lu", pstr_efuse_id, strlen(pstr_efuse_id));
    
    return 0;
}

#if 0 // use it  if you need
static int8_t get_efuse_id_imx(struct cam_sensor_ctrl_t *s_ctrl, uint32_t SensorId, char * pstr_efuse_id) 
{

    char hex_efuseid;
    char str_efuseid[23]={0};

    uint8_t i;
    
    uint32_t page_number=0;
    uint32_t head_addr=0;
    
    struct cam_sensor_i2c_reg_setting  i2c_reg_settings;
    struct cam_sensor_i2c_reg_array    i2c_reg_array;
    
    if(SensorId == SENSER_ID_imx350)
    {
        page_number=39;
        head_addr=0x0A20;
    }
    else if(SensorId == SENSER_ID_imx486)
    {
        page_number=11;
        head_addr=0x0A27;
    }
    else
    {
        CAM_ERR(CAM_SENSOR, "sensor id error   0x%x", SensorId);
        return -1;
    }
    
    i2c_reg_settings.reg_setting = &i2c_reg_array;
    
    WRITE_OTP_ADDR(0x0136, 0x18)
    WRITE_OTP_ADDR(0x0137, 0x00)// Set up real INCK frequency EXCK_FREQ
    
    WRITE_OTP_ADDR(0x0A02, page_number)// Set the OTP page number
    WRITE_OTP_ADDR(0x0A00, 0x01)// Set up for OTP read transfer mode

/* Read OTP status register.
0d: = OTP access is in prog
1d: = OTP read has complet
5d: = OTP read has failed */
    READ_OTP_ADDR(0x0A01, (uint32_t *)&hex_efuseid)
    CAM_DBG(CAM_SENSOR, " Read OTP status=%d ", hex_efuseid);
    
    // Read Chip ID
    for(i=0; i<11; i++)
    {
        READ_OTP_ADDR(head_addr + i, (uint32_t *)&hex_efuseid)
        //CAM_DBG(CAM_SENSOR, "i=%d   hex_efuseid=0x%x ", i, hex_efuseid);
        sprintf(str_efuseid+2*i,"%02x", hex_efuseid);
		msleep(1);
    }
    //CAM_DBG(CAM_SENSOR, "str_efuseid=%s", str_efuseid);

    strcpy(pstr_efuse_id, str_efuseid); 
    CAM_ERR(CAM_SENSOR,"pstr_efuse_id=%s strlen=%lu", pstr_efuse_id, strlen(pstr_efuse_id));
    
    return 0;
}

#endif

int32_t get_CameName_and_EfuseId(struct cam_sensor_ctrl_t *s_ctrl)
{
    int32_t rc = 0;
    uint32_t sensor_id = 0;
    
    sensor_id = s_ctrl->sensordata->slave_info.sensor_id;

    switch(sensor_id){
        
        case SENSER_ID_ofilm_s5k3p9: {
            CAM_DBG(CAM_SENSOR," case SENSER_ID_ofilm_s5k3p9 ");
            strcpy(front_cam_name,"1_oof_s5k3p9");
            rc = get_efuse_id_s5k(s_ctrl, sensor_id, front_cam_efuse_id);
        }
        break;
        
        case SENSER_ID_truly_s5k3p9: {
            CAM_DBG(CAM_SENSOR," case SENSER_ID_truly_s5k3p9 ");
            strcpy(front_cam_name,"1_truly_s5k3p9");
            //rc = get_efuse_id_s5k(s_ctrl, sensor_id, front_cam_efuse_id);
        }
        break;

        case SENSER_ID_ov16b10: {
            CAM_DBG(CAM_SENSOR," case SENSER_ID_ov16b10 ");
            strcpy(back_cam_name,"0_qtech_ov16b10");
            get_efuse_id_ov(s_ctrl, back_cam_efuse_id);
        }
        break;
    
        case SENSER_ID_ov8856: {
            CAM_DBG(CAM_SENSOR," case SENSER_ID_ov8856 ");
            strcpy(backaux2_cam_name,"3_qtech_ov8856");
            get_efuse_id_ov(s_ctrl, backaux2_cam_efuse_id);
        }
        break;
    
//-------------------------------   for LG -------------------------------

        case SENSER_ID_s5kgw1sp: {
            CAM_DBG(CAM_SENSOR," case SENSER_ID_s5kgw1sp ");
            strcpy(back_cam_name, "0_lgit_s5kgw1sp");
            //get_efuse_id_s5k(s_ctrl, sensor_id, back_cam_efuse_id);
        }
        break;

        case SENSER_ID_s5kgm2sp: {
            CAM_DBG(CAM_SENSOR," case SENSER_ID_s5kgm2sp ");
            strcpy(back_cam_name, "0_lgit_s5kgm2sp");
            get_efuse_id_s5k(s_ctrl, sensor_id, back_cam_efuse_id);
        }
        break;

        case SENSER_ID_hi1634: {
            CAM_DBG(CAM_SENSOR," case SENSER_ID_hi1634 ");
            strcpy(front_cam_name, "1_cowell_hi1634");
            //get_efuse_id_s5k(s_ctrl, sensor_id, front_cam_efuse_id);
        }
        break;
        
        case SENSER_ID_s5kgd1sp: {
            CAM_DBG(CAM_SENSOR," case SENSER_ID_s5kgd1sp ");
            strcpy(front_cam_name, "1_cowell_s5kgd1sp");
            //get_efuse_id_s5k(s_ctrl, sensor_id, front_cam_efuse_id);
        }
        break;
        
        case SENSER_ID_gc2375h: {
            CAM_DBG(CAM_SENSOR," case SENSER_ID_gc2375h ");
            strcpy(backaux_cam_name, "2_sunrise_gc2375h");
            //get_efuse_id_s5k(s_ctrl, sensor_id, backaux_cam_efuse_id);
        }
        break;
        
        case SENSER_ID_gc02m1b: {
            CAM_DBG(CAM_SENSOR," case SENSER_ID_gc02m1b ");
            strcpy(backaux_cam_name, "2_sunrise_gc02m1b");
            get_efuse_id_s5k(s_ctrl, sensor_id, backaux_cam_efuse_id);
        }
        break;

        case SENSER_ID_hi556: {
            CAM_DBG(CAM_SENSOR," case SENSER_ID_hi556 ");
            strcpy(backaux2_cam_name, "3_jsl_hi556");
            //get_efuse_id_s5k(s_ctrl, sensor_id, backaux2_cam_efuse_id);
        }
        break;
        
        case SENSER_ID_s5k5e9: {
            CAM_DBG(CAM_SENSOR," case SENSER_ID_s5k5e9 ");
            strcpy(backaux3_cam_name, "4_jsl_s5k5e9yx04");
            //get_efuse_id_s5k(s_ctrl, sensor_id, backaux3_cam_efuse_id);
        }
        break;
    
    
    
        default:
        {
            CAM_ERR(CAM_SENSOR, "sensor id error   %d", sensor_id);
            rc = -1;
        }
    }

    return rc;
    
}


int8_t get_s5k3p9_supplier(struct cam_sensor_ctrl_t *s_ctrl) 
{
    uint16_t dummy_sid = 0;
    uint8_t  rc = 1;
    uint32_t eep_data = 0;
    
    dummy_sid = s_ctrl->io_master_info.cci_client->sid;
    s_ctrl->io_master_info.cci_client->sid = EEPROM_SID_s5k3p9 >> 1;

	camera_io_dev_read(&s_ctrl->io_master_info, 
	6, 
	&eep_data,
	CAMERA_SENSOR_I2C_TYPE_WORD, 
	CAMERA_SENSOR_I2C_TYPE_BYTE);
	CAM_ERR(CAM_SENSOR," yzm_did  eep_data=%d ",  eep_data);
	if(eep_data == 2)
	{
		camera_io_dev_read(&s_ctrl->io_master_info, 
		7, 
		&eep_data,
		CAMERA_SENSOR_I2C_TYPE_WORD, 
		CAMERA_SENSOR_I2C_TYPE_BYTE);
		CAM_ERR(CAM_SENSOR,"  yzm_did eep_data=%d ",  eep_data);
		if(eep_data == 0)
		{
			rc = 2;
		}
	}

    s_ctrl->io_master_info.cci_client->sid = dummy_sid;
    return rc;
}




