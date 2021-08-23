/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "ilitek.h"

/*gesture info mode*/
struct demo_debug_info_id0 {
	u8 id;
	u8 app_sys_powr_state_e : 3;
	u8 app_sys_state_e : 3;
	u8 tp_state_e : 2;

	u8 touch_palm_state_e : 2;
	u8 app_an_statu_e : 3;
	u8 app_sys_check_bg_abnormal : 1;
	u8 g_b_wrong_bg: 1;
	u8 reserved0 : 1;

	u8 status_of_dynamic_th_e : 4;
	u8 reserved1 : 4;

	u32 algo_pt_status0 : 3;
	u32 algo_pt_status1 : 3;
	u32 algo_pt_status2 : 3;
	u32 algo_pt_status3 : 3;
	u32 algo_pt_status4 : 3;
	u32 algo_pt_status5 : 3;
	u32 algo_pt_status6 : 3;
	u32 algo_pt_status7 : 3;
	u32 algo_pt_status8 : 3;
	u32 algo_pt_status9 : 3;
	u32 reserved2 : 2;

	u16  hopping_flag : 1;
	u16  hopping_index : 5;
	u16  frequency_h : 2;
	u16  frequency_l : 8;

	u16  reserved3 : 16;
};

void ilitek_dump_data(void *data, int type, int len, int row_len, const char *name)
{
	int i, row = 31;
	u8 *p8 = NULL;
	s32 *p32 = NULL;
	s16 *p16 = NULL;

	if (!ipio_debug_level)
		return;

	if (row_len > 0)
		row = row_len;

	if (data == NULL) {
		ipio_err("The data going to dump is NULL\n");
		return;
	}

	pr_cont("\n\n");
	pr_cont("ILITEK: Dump %s data\n", name);
	pr_cont("ILITEK: ");

	if (type == 8)
		p8 = (u8 *) data;
	if (type == 32 || type == 10)
		p32 = (s32 *) data;
	if (type == 16)
		p16 = (s16 *) data;

	for (i = 0; i < len; i++) {
		if (type == 8)
			pr_cont(" %4x ", p8[i]);
		else if (type == 32)
			pr_cont(" %4x ", p32[i]);
		else if (type == 10)
			pr_cont(" %4d ", p32[i]);
		else if (type == 16)
			pr_cont(" %4d ", p16[i]);

		if ((i % row) == row - 1) {
			pr_cont("\n");
			pr_cont("ILITEK: ");
		}
	}
	pr_cont("\n\n");
}

static void dma_clear_reg_setting(void)
{
	/* 1. interrupt t0/t1 enable flag */
	if (ilitek_ice_mode_bit_mask_write(INTR32_ADDR, INTR32_reg_t0_int_en | INTR32_reg_t1_int_en, (0 << 24)) < 0)
		ipio_err("Write %lu at %x failed\n", INTR32_reg_t0_int_en | INTR32_reg_t1_int_en, INTR32_ADDR);

	/* 2. clear tdi_err_int_flag */
	if (ilitek_ice_mode_bit_mask_write(INTR2_ADDR, INTR2_tdi_err_int_flag_clear, (1 << 18)) < 0)
		ipio_err("Write %lu at %x failed\n", INTR2_tdi_err_int_flag_clear, INTR2_ADDR);

	/* 3. clear dma channel 0 src1 info */
	if (ilitek_ice_mode_write(DMA49_reg_dma_ch0_src1_addr, 0x00000000, 4) < 0)
		ipio_err("Write 0x00000000 at %x failed\n", DMA49_reg_dma_ch0_src1_addr);
	if (ilitek_ice_mode_write(DMA50_reg_dma_ch0_src1_step_inc, 0x00, 1) < 0)
		ipio_err("Write 0x0 at %x failed\n", DMA50_reg_dma_ch0_src1_step_inc);
	if (ilitek_ice_mode_bit_mask_write(DMA50_ADDR, DMA50_reg_dma_ch0_src1_format | DMA50_reg_dma_ch0_src1_en, BIT(31)) < 0)
		ipio_err("Write %lu at %x failed\n", DMA50_reg_dma_ch0_src1_format | DMA50_reg_dma_ch0_src1_en, DMA50_ADDR);

	/* 4. clear dma channel 0 trigger select */
	if (ilitek_ice_mode_bit_mask_write(DMA48_ADDR, DMA48_reg_dma_ch0_trigger_sel, (0 << 16)) < 0)
		ipio_err("Write %lu at %x failed\n", DMA48_reg_dma_ch0_trigger_sel, DMA48_ADDR);
	if (ilitek_ice_mode_bit_mask_write(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25)) < 0)
		ipio_err("Write %lu at %x failed\n", INTR1_reg_flash_int_flag, INTR1_ADDR);

	/* 5. clear dma flash setting */
	ilitek_tddi_flash_clear_dma();
}

static void dma_trigger_reg_setting(u32 reg_dest_addr, u32 flash_start_addr, u32 copy_size)
{
	int retry = 30;
	u32 stat = 0;

	/* 1. set dma channel 0 clear */
	if (ilitek_ice_mode_bit_mask_write(DMA48_ADDR, DMA48_reg_dma_ch0_start_clear, BIT(25)) < 0)
		ipio_err("Write %lu at %x failed\n", DMA48_reg_dma_ch0_start_clear, DMA48_ADDR);

	/* 2. set dma channel 0 src1 info */
	if (ilitek_ice_mode_write(DMA49_reg_dma_ch0_src1_addr, 0x00041010, 4) < 0)
		ipio_err("Write 0x00041010 at %x failed\n", DMA49_reg_dma_ch0_src1_addr);
	if (ilitek_ice_mode_write(DMA50_reg_dma_ch0_src1_step_inc, 0x00, 1) < 0)
		ipio_err("Write 0x00 at %x failed\n", DMA50_reg_dma_ch0_src1_step_inc);
	if (ilitek_ice_mode_bit_mask_write(DMA50_ADDR, DMA50_reg_dma_ch0_src1_format | DMA50_reg_dma_ch0_src1_en, BIT(31)) < 0)
		ipio_err("Write %lu at %x failed\n", DMA50_reg_dma_ch0_src1_format | DMA50_reg_dma_ch0_src1_en, DMA50_ADDR);

	/* 3. set dma channel 0 src2 info */
	if (ilitek_ice_mode_bit_mask_write(DMA52_ADDR, DMA52_reg_dma_ch0_src2_en, (0 << 31)) < 0)
		ipio_err("Write %lu at %x failed\n", DMA52_reg_dma_ch0_src2_en, DMA52_ADDR);

	/* 4. set dma channel 0 dest info */
	if (ilitek_ice_mode_write(DMA53_reg_dma_ch0_dest_addr, reg_dest_addr, 3) < 0)
		ipio_err("Write %x at %x failed\n", reg_dest_addr, DMA53_reg_dma_ch0_dest_addr);
	if (ilitek_ice_mode_write(DMA54_reg_dma_ch0_dest_step_inc, 0x01, 1) < 0)
		ipio_err("Write 0x01 at %x failed\n", DMA54_reg_dma_ch0_dest_step_inc);
	if (ilitek_ice_mode_write(DMA54_ADDR, DMA54_reg_dma_ch0_dest_format | DMA54_reg_dma_ch0_dest_en, BIT(31)) < 0)
		ipio_err("Write %lu at %x failed\n", DMA54_reg_dma_ch0_dest_format | DMA54_reg_dma_ch0_dest_en, DMA54_ADDR);

	/* 5. set dma channel 0 trafer info */
	if (ilitek_ice_mode_write(DMA55_reg_dma_ch0_trafer_counts, copy_size, 4) < 0)
		ipio_err("Write %x at %x failed\n", copy_size, DMA55_reg_dma_ch0_trafer_counts);
	if (ilitek_ice_mode_bit_mask_write(DMA55_ADDR, DMA55_reg_dma_ch0_trafer_mode, (0 << 24)) < 0)
		ipio_err("Write %lu at %x failed\n", DMA55_reg_dma_ch0_trafer_mode, DMA55_ADDR);

	/* 6. set dma channel 0 int info */
	if (ilitek_ice_mode_bit_mask_write(INTR33_ADDR, INTR33_reg_dma_ch0_int_en, (1 << 17)) < 0)
		ipio_err("Write %lu at %x failed\n", INTR33_reg_dma_ch0_int_en, INTR33_ADDR);

	/* 7. set dma channel 0 trigger select */
	if (ilitek_ice_mode_bit_mask_write(DMA48_ADDR, DMA48_reg_dma_ch0_trigger_sel, (1 << 16)) < 0)
		ipio_err("Write %lu at %x failed\n", DMA48_reg_dma_ch0_trigger_sel, DMA48_ADDR);

	/* 8. set dma flash setting */
	ilitek_tddi_flash_dma_write(flash_start_addr, (flash_start_addr+copy_size), copy_size);

	/* 9. clear flash and dma ch0 int flag */
	if (ilitek_ice_mode_bit_mask_write(INTR1_ADDR, INTR1_reg_dma_ch0_int_flag | INTR1_reg_flash_int_flag, BIT(17) | BIT(25)) < 0)
		ipio_err("Write %lu at %x failed\n", INTR1_reg_dma_ch0_int_flag | INTR1_reg_flash_int_flag, INTR1_ADDR);
	if (ilitek_ice_mode_bit_mask_write(0x041013, BIT(0), 1) < 0) //patch
		ipio_err("Write %lu at %x failed\n", BIT(0), 0x041013);

	/* DMA Trigger */
	if (ilitek_ice_mode_write(FLASH4_reg_rcv_data, 0xFF, 1) < 0)
		ipio_err("Trigger DMA failed\n");

	/* waiting for fw reload code completed. */
	while (retry > 0) {
		if (ilitek_ice_mode_read(INTR1_ADDR, &stat, sizeof(u32)) < 0) {
			ipio_err("Read 0x%x error\n", INTR1_ADDR);
			retry--;
			continue;
		}

		ipio_debug("fw dma stat = %x\n", stat);

		if ((stat & BIT(17)) == BIT(17))
			break;

		retry--;
		usleep_range(1000, 1000);
	}

	if (retry <= 0)
		ipio_err("DMA fail: Regsiter = 0x%x Flash = 0x%x, Size = %d\n",
			reg_dest_addr, flash_start_addr, copy_size);

	/* CS High */
	if (ilitek_ice_mode_write(FLASH0_reg_flash_csb, 0x1, 1) < 0)
		ipio_err("Pull CS High failed\n");
	/* waiting for CS status done */
	mdelay(10);
}

int ilitek_tddi_move_mp_code_flash(void)
{
	int ret = 0;
	u32 mp_text_size = 0, mp_andes_init_size = 0;
	u32 mp_flash_addr, mp_size, overlay_start_addr, overlay_end_addr;
	bool dma_trigger_enable = 0;
	u8 cmd[16] = {0};

	cmd[0] = P5_X_MP_TEST_MODE_INFO;
	ret = idev->write(cmd, 1);
	if (ret < 0)
		goto out;

	memset(cmd, 0, sizeof(cmd));

	ipio_info("read mp info length = %d\n", idev->protocol->mp_info_len);
	ret = idev->read(cmd, idev->protocol->mp_info_len);
	if (ret < 0)
		goto out;

	ilitek_dump_data(cmd, 8, idev->protocol->mp_info_len, 0, "MP overlay info");

	dma_trigger_enable = 0;

	mp_flash_addr = cmd[3] + (cmd[2] << 8) + (cmd[1] << 16);
	mp_size = cmd[6] + (cmd[5] << 8) + (cmd[4] << 16);
	overlay_start_addr = cmd[9] + (cmd[8] << 8) + (cmd[7] << 16);
	overlay_end_addr = cmd[12] + (cmd[11] << 8) + (cmd[10] << 16);

	if (overlay_start_addr != 0x0 && overlay_end_addr != 0x0
		&& cmd[0] == P5_X_MP_TEST_MODE_INFO)
		dma_trigger_enable = 1;

	ipio_info("MP info Overlay: Enable = %d, addr = 0x%x ~ 0x%x, flash addr = 0x%x, mp size = 0x%x\n",
		dma_trigger_enable, overlay_start_addr,
		overlay_end_addr, mp_flash_addr, mp_size);

	cmd[0] = P5_X_MODE_CONTROL;
	cmd[1] = P5_X_FW_TEST_MODE;
	ret = idev->write(cmd, 2);
	if (ret < 0)
		goto out;

	/* Check if ic is ready switching test mode from demo mode */
	idev->actual_tp_mode = P5_X_FW_AP_MODE;
	ret = ilitek_tddi_ic_check_busy(50, 50); /* Set busy as 0x41 */
	if (ret < 0)
		goto out;

	ret = ilitek_ice_mode_ctrl(ENABLE, OFF);
	if (ret < 0)
		goto out;

	if (dma_trigger_enable) {
		mp_andes_init_size = overlay_start_addr;
		mp_text_size = (mp_size - overlay_end_addr) + 1;
		ipio_info("MP andes init size = %d , MP text size = %d\n", mp_andes_init_size, mp_text_size);

		dma_clear_reg_setting();

		ipio_info("[Move ANDES.INIT to DRAM]\n");
		dma_trigger_reg_setting(0, mp_flash_addr, mp_andes_init_size);	 /* DMA ANDES.INIT */

		dma_clear_reg_setting();

		ipio_info("[Move MP.TEXT to DRAM]\n");
		dma_trigger_reg_setting(overlay_end_addr, (mp_flash_addr + overlay_start_addr), mp_text_size);

		dma_clear_reg_setting();
	} else {
		/* DMA Trigger */
		if (ilitek_ice_mode_write(FLASH4_reg_rcv_data, 0xFF, 1) < 0)
			ipio_err("Trigger DMA failed\n");
		/* waiting for fw reload code completed. */
		mdelay(30);

		/* CS High */
		if (ilitek_ice_mode_write(FLASH0_reg_flash_csb, 0x1, 1) < 0)
			ipio_err("Pull CS High failed\n");
		/* waiting for CS status done */
		mdelay(10);
	}

	if (ilitek_tddi_reset_ctrl(TP_IC_CODE_RST) < 0)
		ipio_err("IC Code reset failed during moving mp code\n");

	ret = ilitek_ice_mode_ctrl(DISABLE, OFF);
	if (ret < 0)
		goto out;

	/* Check if ic is already in test mode */
	idev->actual_tp_mode = P5_X_FW_TEST_MODE; /* set busy as 0x51 */
	ret = ilitek_tddi_ic_check_busy(300, 50);
	if (ret < 0)
		ipio_err("Check cdc timeout failed after moved mp code\n");

out:
	return ret;
}

int ilitek_tddi_move_mp_code_iram(void)
{
	ipio_info("Download MP code to iram\n");
	return ilitek_tddi_fw_upgrade_handler(NULL);
}

int ilitek_tddi_proximity_near(int mode)
{
	int ret = 0;

	idev->prox_near = true;

	switch (mode) {
	case DDI_POWER_ON:
		/*
		 * If the power of VSP and VSN keeps alive when proximity near event
		 * occures, TP can just go to sleep in.
		 */
		ret = ilitek_tddi_ic_func_ctrl("sleep", SLEEP_IN);
		if (ret < 0)
			ipio_err("Write sleep in cmd failed\n");
		break;
	case DDI_POWER_OFF:
		ipio_info("DDI POWER OFF, do nothing\n");
		break;
	default:
		ipio_err("Unknown mode (%d)\n", mode);
		ret = -EINVAL;
		break;
	}
	return ret;
}

int ilitek_tddi_proximity_far(int mode)
{
	int ret = 0;
	u8 cmd[2] = {0};

	if (!idev->prox_near) {
		ipio_info("No proximity near event, break\n");
		return 0;
	}

	switch (mode) {
	case WAKE_UP_GESTURE_RECOVERY:
		/*
		 * If the power of VSP and VSN has been shut down previsouly,
		 * TP should go through gesture recovery to get back.
		 */
		ilitek_tddi_gesture_recovery();
		break;
	case WAKE_UP_SWITCH_GESTURE_MODE:
		/*
		 * If the power of VSP and VSN keeps alive in the event of proximity near,
		 * TP can be just recovered by switching gesture mode to get back.
		 */
		cmd[0] = 0xF6;
		cmd[1] = 0x0A;

		ipio_info("write prepare gesture command 0xF6 0x0A\n");
		ret = idev->write(cmd, 2);
		if (ret < 0) {
			ipio_info("write prepare gesture command error\n");
			break;
		}

		ret = ilitek_tddi_switch_tp_mode(P5_X_FW_GESTURE_MODE);
		if (ret < 0)
			ipio_err("Switch to gesture mode failed during proximity far\n");
		break;
	default:
		ipio_err("Unknown mode (%d)\n", mode);
		ret = -EINVAL;
		break;
	}

	idev->prox_near = false;

	return ret;
}

int ilitek_tddi_move_gesture_code_flash(int mode)
{
	int ret = 0;

	/*
	 * NOTE: If functions need to be added during suspend,
	 * they must be called before gesture cmd reaches to FW.
	 */

	ipio_info("Gesture mode = %d\n", mode);
	ret = ilitek_set_tp_data_len(mode, true);

	return ret;
}

int ilitek_tddi_move_gesture_code_iram(int mode)
{
	int i, ret = 0;
	int timeout = 10;
	u8 cmd[3] = {0};

	/*
	 * NOTE: If functions need to be added during suspend,
	 * they must be called before gesture cmd reaches to FW.
	 */

	if (!idev->gesture_load_code) {
		ipio_info("Gesture code loaded by firmware, mode = %d\n",  mode);
		ret = ilitek_set_tp_data_len(mode, true);
		goto out;
	}

	ipio_info("Gesture code loaded by driver, mode = %d\n", mode);

	ret = ilitek_tddi_ic_func_ctrl("lpwg", 0x3);
	if (ret < 0) {
		ipio_err("write gesture flag failed\n");
		goto out;
	}

	ret = ilitek_set_tp_data_len(mode, true);
	if (ret < 0) {
		ipio_err("Failed to set tp data length\n");
		goto out;
	}

	for (i = 0; i < timeout; i++) {
		/* Prepare Check Ready */
		cmd[0] = P5_X_READ_DATA_CTRL;
		cmd[1] = 0xA;
		ret = idev->write(cmd, 2);
		if (ret < 0) {
			ipio_err("Write 0xF6,0xA failed\n");
			goto out;
		}

		/* Check ready for load code */
		ret = ilitek_tddi_ic_func_ctrl("lpwg", 0x5);
		if (ret < 0) {
			ipio_err("write check load code error");
			goto out;
		}

		ret = idev->read(cmd, 1);
		if (ret < 0) {
			ipio_err("read gesture ready byte error\n");
			goto out;
		}

		ipio_debug("gesture ready byte = 0x%x\n", cmd[0]);

		if (cmd[0] == 0x91) {
			ipio_info("Ready to load gesture code\n");
			break;
		}
	}

	if (i >= timeout) {
		ipio_err("Gesture is not ready (0x%x), try to run its recovery\n", cmd[0]);
		return ilitek_tddi_gesture_recovery();
	}

	ret = ilitek_tddi_fw_upgrade_handler(NULL);
	if (ret < 0) {
		ipio_err("FW upgrade failed during moving code\n");
		goto out;
	}

	/* Resume gesture loader */
	ret = ilitek_tddi_ic_func_ctrl("lpwg", 0x6);
	if (ret < 0) {
		ipio_err("write resume loader error");
		goto out;
	}

out:
	return ret;
}

u8 ilitek_calc_packet_checksum(u8 *packet, int len)
{
	int i;
	s32 sum = 0;

	for (i = 0; i < len; i++)
		sum += packet[i];

	return (u8) ((-sum) & 0xFF);
}

int ilitek_tddi_touch_esd_gesture_flash(void)
{
	int ret = 0, retry = 100;
	u32 answer = 0;

	ret = ilitek_ice_mode_ctrl(ENABLE, OFF);
	if (ret < 0) {
		ipio_err("Enable ice mode failed during gesture recovery\n");
		return ret;
	}

	ipio_info("ESD Gesture PWD Addr = 0x%X, PWD = 0x%X\n",
		I2C_ESD_GESTURE_PWD_ADDR, ESD_GESTURE_PWD);

	/* write a special password to inform FW go back into gesture mode */
	ret = ilitek_ice_mode_write(I2C_ESD_GESTURE_PWD_ADDR, ESD_GESTURE_PWD, 4);
	if (ret < 0) {
		ipio_err("write password failed\n");
		goto fail;
	}

	/* HW reset gives effect to FW receives password successed */
	idev->actual_tp_mode = P5_X_FW_AP_MODE;
	ret = ilitek_tddi_reset_ctrl(idev->reset);
	if (ret < 0) {
		ipio_err("TP Reset failed during gesture recovery\n");
		goto fail;
	}

	ret = ilitek_ice_mode_ctrl(ENABLE, ON);
	if (ret < 0) {
		ipio_err("Enable ice mode failed during gesture recovery\n");
		goto fail;
	}

	/* polling another specific register to see if gesutre is enabled properly */
	do {
		ret = ilitek_ice_mode_read(I2C_ESD_GESTURE_PWD_ADDR, &answer, sizeof(u32));
		if (ret < 0) {
			ipio_err("Read gesture answer error\n");
			goto fail;
		}

		if (answer != I2C_ESD_GESTURE_RUN)
			ipio_info("ret = 0x%X, answer = 0x%X\n", answer, I2C_ESD_GESTURE_RUN);

		mdelay(1);
	} while (answer != I2C_ESD_GESTURE_RUN && --retry > 0);

	if (retry <= 0) {
		ipio_err("Enter gesture failed\n");
		ret = -1;
		goto fail;
	}

	ipio_info("Enter gesture successfully\n");

	ret = ilitek_ice_mode_ctrl(DISABLE, ON);
	if (ret < 0) {
		ipio_err("Disable ice mode failed during gesture recovery\n");
		goto fail;
	}

	idev->actual_tp_mode = P5_X_FW_GESTURE_MODE;
	ilitek_set_tp_data_len(idev->gesture_mode, false);
	return ret;

fail:
	ilitek_ice_mode_ctrl(DISABLE, ON);
	return ret;
}

int ilitek_tddi_touch_esd_gesture_iram(void)
{
	int ret = 0, retry = 100;
	u32 answer = 0;
	u32 esd_ges_pwd_addr = 0x0;

	ret = ilitek_ice_mode_ctrl(ENABLE, OFF);
	if (ret < 0) {
		ipio_err("Enable ice mode failed during gesture recovery\n");
		return ret;
	}

	if (idev->chip->core_ver >= CORE_VER_1420)
		esd_ges_pwd_addr = I2C_ESD_GESTURE_PWD_ADDR;
	else
		esd_ges_pwd_addr = SPI_ESD_GESTURE_PWD_ADDR;

	ipio_info("ESD Gesture PWD Addr = 0x%X, PWD = 0x%X\n",
		esd_ges_pwd_addr, ESD_GESTURE_PWD);

	/* write a special password to inform FW go back into gesture mode */
	ret = ilitek_ice_mode_write(esd_ges_pwd_addr, ESD_GESTURE_PWD, 4);
	if (ret < 0) {
		ipio_err("write password failed\n");
		goto fail;
	}

	/* Host download gives effect to FW receives password successed */
	idev->actual_tp_mode = P5_X_FW_AP_MODE;
	ret = ilitek_tddi_fw_upgrade_handler(NULL);
	if (ret < 0) {
		ipio_err("FW upgrade failed during gesture recovery\n");
		goto fail;
	}

	/* Wait for fw running code finished. */
	if (idev->info_from_hex || (idev->chip->core_ver >= CORE_VER_1410))
		msleep(50);

	ret = ilitek_ice_mode_ctrl(ENABLE, ON);
	if (ret < 0) {
		ipio_err("Enable ice mode failed during gesture recovery\n");
		goto fail;
	}

	/* polling another specific register to see if gesutre is enabled properly */
	do {
		ret = ilitek_ice_mode_read(esd_ges_pwd_addr, &answer, sizeof(u32));
		if (ret < 0) {
			ipio_err("Read gesture answer error\n");
			break;
		}

		if (answer != SPI_ESD_GESTURE_RUN)
			ipio_info("ret = 0x%X, answer = 0x%X\n", answer, SPI_ESD_GESTURE_RUN);

		mdelay(1);
	} while (answer != SPI_ESD_GESTURE_RUN && --retry > 0);

	if (retry <= 0) {
		ipio_err("Enter gesture failed\n");
		ret = -1;
		goto fail;
	}

	ipio_info("Enter gesture successfully\n");

	ret = ilitek_ice_mode_ctrl(DISABLE, ON);
	if (ret < 0) {
		ipio_err("Disable ice mode failed during gesture recovery\n");
		goto fail;
	}

	/* Load gesture code */
	idev->actual_tp_mode = P5_X_FW_GESTURE_MODE;
	ilitek_set_tp_data_len(idev->gesture_mode, false);
	ret = ilitek_tddi_fw_upgrade_handler(NULL);
	if (ret < 0) {
		ipio_err("Failed to load code during gesture recovery\n");
		goto fail;
	}

	/* Resume gesture loader */
	ret = ilitek_tddi_ic_func_ctrl("lpwg", 0x6);
	if (ret < 0) {
		ipio_err("write resume loader error");
		goto fail;
	}

	return ret;

fail:
	ilitek_ice_mode_ctrl(DISABLE, ON);
	return ret;
}

void demo_debug_info_id0(u8 *buf, size_t len)
{
	struct demo_debug_info_id0 id0;
	int size = sizeof(id0);

	ipio_memcpy(&id0, buf, size, len);
	ipio_info("id0 len = %d, struct size = %d\n", (int)len, size);

	ipio_info("id = %d\n", id0.id);
	ipio_info("app_sys_powr_state_e = %d\n", id0.app_sys_powr_state_e);
	ipio_info("app_sys_state_e = %d\n", id0.app_sys_state_e);
	ipio_info("tp_state_e = %d\n", id0.tp_state_e);
	ipio_info("touch_palm_state_e = %d\n", id0.touch_palm_state_e);
	ipio_info("app_an_statu_e = %d\n", id0.app_an_statu_e);
	ipio_info("app_sys_check_bg_abnormal = %d\n", id0.app_sys_check_bg_abnormal);
	ipio_info("g_b_wrong_bg = %d\n", id0.g_b_wrong_bg);
	ipio_info("status_of_dynamic_th_e = %d\n", id0.status_of_dynamic_th_e);
	ipio_info("algo_pt_status0 = %d\n", id0.algo_pt_status0);
	ipio_info("algo_pt_status1 = %d\n", id0.algo_pt_status1);
	ipio_info("algo_pt_status2 = %d\n", id0.algo_pt_status2);
	ipio_info("algo_pt_status3 = %d\n", id0.algo_pt_status3);
	ipio_info("algo_pt_status4 = %d\n", id0.algo_pt_status4);
	ipio_info("algo_pt_status5 = %d\n", id0.algo_pt_status5);
	ipio_info("algo_pt_status6 = %d\n", id0.algo_pt_status6);
	ipio_info("algo_pt_status7 = %d\n", id0.algo_pt_status7);
	ipio_info("algo_pt_status8 = %d\n", id0.algo_pt_status8);
	ipio_info("algo_pt_status9 = %d\n", id0.algo_pt_status9);
	ipio_info("hopping_flag = %d\n", id0.hopping_flag);
	ipio_info("hopping_index = %d\n", id0.hopping_index);
	ipio_info("frequency = %d\n", (id0.frequency_h << 8 | id0.frequency_l));
}

void demo_debug_info_mode(u8 *buf, size_t len)
{
	u8 *info_ptr;
	u8 info_id, info_len;

	ilitek_tddi_report_ap_mode(buf, P5_X_DEMO_MODE_PACKET_LEN);
	info_ptr = buf + P5_X_DEMO_MODE_PACKET_LEN;
	info_len = info_ptr[0];
	info_id = info_ptr[1];

	ipio_info("info len = %d ,id = %d\n", info_len, info_id);

	idev->demo_debug_info[info_id](&info_ptr[1], info_len);
}

static void ilitek_tddi_touch_send_debug_data(u8 *buf, int len)
{
	int index;
	mutex_lock(&idev->debug_mutex);

	if (!idev->netlink && !idev->dnp)
		goto out;

	/* Send data to netlink */
	if (idev->netlink) {
		netlink_reply_msg(buf, len);
		goto out;
	}

	/* Sending data to apk via the node of debug_message node */
	if (idev->dnp) {
		index = idev->dbf;
		if (!idev->dbl[idev->dbf].mark) {
			idev->dbf = ((idev->dbf + 1) % TR_BUF_LIST_SIZE);
		} else {
			if (idev->dbf == 0)
				index = TR_BUF_LIST_SIZE - 1;
			else
				index = idev->dbf - 1;
		}
		if (idev->dbl[index].data == NULL) {
			ipio_info("BUFFER %d error\n", index);
			goto out;
		}
		ipio_memcpy(idev->dbl[index].data, buf, len, 2048);
		idev->dbl[index].mark = true;
		wake_up(&(idev->inq));
		goto out;
	}

out:
	mutex_unlock(&idev->debug_mutex);
}

void ilitek_tddi_touch_press(u16 x, u16 y, u16 pressure, u16 id)
{
	ipio_debug("Touch Press: id = %d, x = %d, y = %d, p = %d\n", id, x, y, pressure);

	if (MT_B_TYPE) {
		input_mt_slot(idev->input, id);
		input_mt_report_slot_state(idev->input, MT_TOOL_FINGER, true);
		input_report_abs(idev->input, ABS_MT_POSITION_X, x);
		input_report_abs(idev->input, ABS_MT_POSITION_Y, y);
		if (MT_PRESSURE)
			input_report_abs(idev->input, ABS_MT_PRESSURE, pressure);
	} else {
		input_report_key(idev->input, BTN_TOUCH, 1);
		input_report_abs(idev->input, ABS_MT_TRACKING_ID, id);
		input_report_abs(idev->input, ABS_MT_TOUCH_MAJOR, 1);
		input_report_abs(idev->input, ABS_MT_WIDTH_MAJOR, 1);
		input_report_abs(idev->input, ABS_MT_POSITION_X, x);
		input_report_abs(idev->input, ABS_MT_POSITION_Y, y);
		if (MT_PRESSURE)
			input_report_abs(idev->input, ABS_MT_PRESSURE, pressure);

		input_mt_sync(idev->input);
	}
}

void ilitek_tddi_touch_release(u16 x, u16 y, u16 id)
{
	ipio_debug("Touch Release: id = %d, x = %d, y = %d\n", id, x, y);

	if (MT_B_TYPE) {
		input_mt_slot(idev->input, id);
		input_mt_report_slot_state(idev->input, MT_TOOL_FINGER, false);
	} else {
		input_report_key(idev->input, BTN_TOUCH, 0);
		input_mt_sync(idev->input);
	}
}

void ilitek_tddi_touch_release_all_point(void)
{
	int i;

	if (MT_B_TYPE) {
		for (i = 0 ; i < MAX_TOUCH_NUM; i++)
			ilitek_tddi_touch_release(0, 0, i);

		input_report_key(idev->input, BTN_TOUCH, 0);
		input_report_key(idev->input, BTN_TOOL_FINGER, 0);
	} else {
		ilitek_tddi_touch_release(0, 0, 0);
	}
	input_sync(idev->input);
}

static struct ilitek_touch_info touch_info[MAX_TOUCH_NUM];

void ilitek_tddi_report_ap_mode(u8 *buf, int len)
{
	int i = 0;
	u32 xop = 0, yop = 0;

	memset(touch_info, 0x0, sizeof(touch_info));

	idev->finger = 0;

	for (i = 0; i < MAX_TOUCH_NUM; i++) {
		if ((buf[(4 * i) + 1] == 0xFF) && (buf[(4 * i) + 2] == 0xFF)
			&& (buf[(4 * i) + 3] == 0xFF)) {
			if (MT_B_TYPE)
				idev->curt_touch[i] = 0;
			continue;
		}

		xop = (((buf[(4 * i) + 1] & 0xF0) << 4) | (buf[(4 * i) + 2]));
		yop = (((buf[(4 * i) + 1] & 0x0F) << 8) | (buf[(4 * i) + 3]));

		if (idev->trans_xy) {
			touch_info[idev->finger].x = xop;
			touch_info[idev->finger].y = yop;
		} else {
			touch_info[idev->finger].x = xop * idev->panel_wid / TPD_WIDTH;
			touch_info[idev->finger].y = yop * idev->panel_hei / TPD_HEIGHT;
		}

		touch_info[idev->finger].id = i;

		if (MT_PRESSURE)
			touch_info[idev->finger].pressure = buf[(4 * i) + 4];
		else
			touch_info[idev->finger].pressure = 1;

		ipio_debug("original x = %d, y = %d\n", xop, yop);
		idev->finger++;
		if (MT_B_TYPE)
			idev->curt_touch[i] = 1;
	}

	ipio_debug("figner number = %d, LastTouch = %d\n", idev->finger, idev->last_touch);

	if (idev->finger) {
		if (MT_B_TYPE) {
			for (i = 0; i < idev->finger; i++) {
				input_report_key(idev->input, BTN_TOUCH, 1);
				ilitek_tddi_touch_press(touch_info[i].x, touch_info[i].y, touch_info[i].pressure, touch_info[i].id);
				input_report_key(idev->input, BTN_TOOL_FINGER, 1);
			}
			for (i = 0; i < MAX_TOUCH_NUM; i++) {
				if (idev->curt_touch[i] == 0 && idev->prev_touch[i] == 1)
					ilitek_tddi_touch_release(0, 0, i);
				idev->prev_touch[i] = idev->curt_touch[i];
			}
		} else {
			for (i = 0; i < idev->finger; i++)
				ilitek_tddi_touch_press(touch_info[i].x, touch_info[i].y, touch_info[i].pressure, touch_info[i].id);
		}
		input_sync(idev->input);
		idev->last_touch = idev->finger;
	} else {
		if (idev->last_touch) {
			if (MT_B_TYPE) {
				for (i = 0; i < MAX_TOUCH_NUM; i++) {
					if (idev->curt_touch[i] == 0 && idev->prev_touch[i] == 1)
						ilitek_tddi_touch_release(0, 0, i);
					idev->prev_touch[i] = idev->curt_touch[i];
				}
				input_report_key(idev->input, BTN_TOUCH, 0);
				input_report_key(idev->input, BTN_TOOL_FINGER, 0);
			} else {
				ilitek_tddi_touch_release(0, 0, 0);
			}
			input_sync(idev->input);
			idev->last_touch = 0;
		}
	}
	ilitek_tddi_touch_send_debug_data(buf, len);
}

void ilitek_tddi_report_debug_mode(u8 *buf, int len)
{
	int i = 0;
	u32 xop = 0, yop = 0;
	static u8 p[MAX_TOUCH_NUM];

	memset(touch_info, 0x0, sizeof(touch_info));

	idev->finger = 0;

	for (i = 0; i < MAX_TOUCH_NUM; i++) {
		if ((buf[(3 * i) + 5] == 0xFF) && (buf[(3 * i) + 6] == 0xFF)
			&& (buf[(3 * i) + 7] == 0xFF)) {
			if (MT_B_TYPE)
				idev->curt_touch[i] = 0;
			continue;
		}

		xop = (((buf[(3 * i) + 5] & 0xF0) << 4) | (buf[(3 * i) + 6]));
		yop = (((buf[(3 * i) + 5] & 0x0F) << 8) | (buf[(3 * i) + 7]));

		if (idev->trans_xy) {
			touch_info[idev->finger].x = xop;
			touch_info[idev->finger].y = yop;
		} else {
			touch_info[idev->finger].x = xop * idev->panel_wid / TPD_WIDTH;
			touch_info[idev->finger].y = yop * idev->panel_hei / TPD_HEIGHT;
		}

		touch_info[idev->finger].id = i;

		if (MT_PRESSURE) {
			/*
			 * Since there's no pressure data in debug mode, we make fake values
			 * for android system if pressure needs to be reported.
			 */
			if (p[idev->finger] == 1)
				touch_info[idev->finger].pressure = p[idev->finger] = 2;
			else
				touch_info[idev->finger].pressure = p[idev->finger] = 1;
		} else {
			touch_info[idev->finger].pressure = 1;
		}

		ipio_debug("original x = %d, y = %d\n", xop, yop);
		idev->finger++;
		if (MT_B_TYPE)
			idev->curt_touch[i] = 1;
	}

	ipio_debug("figner number = %d, LastTouch = %d\n", idev->finger, idev->last_touch);

	if (idev->finger) {
		if (MT_B_TYPE) {
			for (i = 0; i < idev->finger; i++) {
				input_report_key(idev->input, BTN_TOUCH, 1);
				ilitek_tddi_touch_press(touch_info[i].x, touch_info[i].y, touch_info[i].pressure, touch_info[i].id);
				input_report_key(idev->input, BTN_TOOL_FINGER, 1);
			}
			for (i = 0; i < MAX_TOUCH_NUM; i++) {
				if (idev->curt_touch[i] == 0 && idev->prev_touch[i] == 1)
					ilitek_tddi_touch_release(0, 0, i);
				idev->prev_touch[i] = idev->curt_touch[i];
			}
		} else {
			for (i = 0; i < idev->finger; i++)
				ilitek_tddi_touch_press(touch_info[i].x, touch_info[i].y, touch_info[i].pressure, touch_info[i].id);
		}
		input_sync(idev->input);
		idev->last_touch = idev->finger;
	} else {
		if (idev->last_touch) {
			if (MT_B_TYPE) {
				for (i = 0; i < MAX_TOUCH_NUM; i++) {
					if (idev->curt_touch[i] == 0 && idev->prev_touch[i] == 1)
						ilitek_tddi_touch_release(0, 0, i);
					idev->prev_touch[i] = idev->curt_touch[i];
				}
				input_report_key(idev->input, BTN_TOUCH, 0);
				input_report_key(idev->input, BTN_TOOL_FINGER, 0);
			} else {
				ilitek_tddi_touch_release(0, 0, 0);
			}
			input_sync(idev->input);
			idev->last_touch = 0;
		}
	}
	ilitek_tddi_touch_send_debug_data(buf, len);
}

void ilitek_tddi_report_gesture_mode(u8 *buf, int len)
{
	int i, lu_x = 0, lu_y = 0, rd_x = 0, rd_y = 0, score = 0;
	u8 ges[P5_X_GESTURE_INFO_LENGTH] = {0};
	struct gesture_coordinate *gc = idev->gcoord;
	struct input_dev *input = idev->input;
	bool transfer = idev->trans_xy;

	for (i = 0; i < len; i++)
		ges[i] = buf[i];

	memset(gc, 0x0, sizeof(struct gesture_coordinate));

	gc->code = ges[1];
	score = ges[36];
	ipio_debug("gesture code = 0x%x, score = %d\n", gc->code, score);

	/* Parsing gesture coordinate */
	gc->pos_start.x = ((ges[4] & 0xF0) << 4) | ges[5];
	gc->pos_start.y = ((ges[4] & 0x0F) << 8) | ges[6];
	gc->pos_end.x   = ((ges[7] & 0xF0) << 4) | ges[8];
	gc->pos_end.y   = ((ges[7] & 0x0F) << 8) | ges[9];
	gc->pos_1st.x   = ((ges[16] & 0xF0) << 4) | ges[17];
	gc->pos_1st.y   = ((ges[16] & 0x0F) << 8) | ges[18];
	gc->pos_2nd.x   = ((ges[19] & 0xF0) << 4) | ges[20];
	gc->pos_2nd.y   = ((ges[19] & 0x0F) << 8) | ges[21];
	gc->pos_3rd.x   = ((ges[22] & 0xF0) << 4) | ges[23];
	gc->pos_3rd.y   = ((ges[22] & 0x0F) << 8) | ges[24];
	gc->pos_4th.x   = ((ges[25] & 0xF0) << 4) | ges[26];
	gc->pos_4th.y   = ((ges[25] & 0x0F) << 8) | ges[27];

	switch (gc->code) {
	case GESTURE_DOUBLECLICK:
		ipio_info("Double Click key event\n");
		input_report_key(input, KEY_GESTURE_POWER, 1);
		input_sync(input);
		input_report_key(input, KEY_GESTURE_POWER, 0);
		input_sync(input);
		gc->type  = GESTURE_DOUBLECLICK;
		gc->clockwise = 1;
		gc->pos_end.x = gc->pos_start.x;
		gc->pos_end.y = gc->pos_start.y;
		break;
	case GESTURE_LEFT:
		gc->type  = GESTURE_LEFT;
		gc->clockwise = 1;
		break;
	case GESTURE_RIGHT:
		gc->type  = GESTURE_RIGHT;
		gc->clockwise = 1;
		break;
	case GESTURE_UP:
		gc->type  = GESTURE_UP;
		gc->clockwise = 1;
		break;
	case GESTURE_DOWN:
		gc->type  = GESTURE_DOWN;
		gc->clockwise = 1;
		break;
	case GESTURE_O:
		gc->type  = GESTURE_O;
		gc->clockwise = (ges[34] > 1) ? 0 : ges[34];

		lu_x = (((ges[28] & 0xF0) << 4) | (ges[29]));
		lu_y = (((ges[28] & 0x0F) << 8) | (ges[30]));
		rd_x = (((ges[31] & 0xF0) << 4) | (ges[32]));
		rd_y = (((ges[31] & 0x0F) << 8) | (ges[33]));

		gc->pos_1st.x = ((rd_x + lu_x) / 2);
		gc->pos_1st.y = lu_y;
		gc->pos_2nd.x = lu_x;
		gc->pos_2nd.y = ((rd_y + lu_y) / 2);
		gc->pos_3rd.x = ((rd_x + lu_x) / 2);
		gc->pos_3rd.y = rd_y;
		gc->pos_4th.x = rd_x;
		gc->pos_4th.y = ((rd_y + lu_y) / 2);
		break;
	case GESTURE_W:
		gc->type  = GESTURE_W;
		gc->clockwise = 1;
		break;
	case GESTURE_M:
		gc->type  = GESTURE_M;
		gc->clockwise = 1;
		break;
	case GESTURE_V:
		gc->type  = GESTURE_V;
		gc->clockwise = 1;
		break;
	case GESTURE_C:
		gc->type  = GESTURE_C;
		gc->clockwise = 1;
		break;
	case GESTURE_E:
		gc->type  = GESTURE_E;
		gc->clockwise = 1;
		break;
	case GESTURE_S:
		gc->type  = GESTURE_S;
		gc->clockwise = 1;
		break;
	case GESTURE_Z:
		gc->type  = GESTURE_Z;
		gc->clockwise = 1;
		break;
	case GESTURE_TWOLINE_DOWN:
		gc->type  = GESTURE_TWOLINE_DOWN;
		gc->clockwise = 1;
		gc->pos_1st.x  = (((ges[10] & 0xF0) << 4) | (ges[11]));
		gc->pos_1st.y  = (((ges[10] & 0x0F) << 8) | (ges[12]));
		gc->pos_2nd.x  = (((ges[13] & 0xF0) << 4) | (ges[14]));
		gc->pos_2nd.y  = (((ges[13] & 0x0F) << 8) | (ges[15]));
		break;
	default:
		ipio_err("Unknown gesture code\n");
		break;
	}

	if (!transfer) {
		gc->pos_start.x	= gc->pos_start.x * idev->panel_wid / TPD_WIDTH;
		gc->pos_start.y = gc->pos_start.y * idev->panel_hei / TPD_HEIGHT;
		gc->pos_end.x   = gc->pos_end.x * idev->panel_wid / TPD_WIDTH;
		gc->pos_end.y   = gc->pos_end.y * idev->panel_hei / TPD_HEIGHT;
		gc->pos_1st.x   = gc->pos_1st.x * idev->panel_wid / TPD_WIDTH;
		gc->pos_1st.y   = gc->pos_1st.y * idev->panel_hei / TPD_HEIGHT;
		gc->pos_2nd.x   = gc->pos_2nd.x * idev->panel_wid / TPD_WIDTH;
		gc->pos_2nd.y   = gc->pos_2nd.y * idev->panel_hei / TPD_HEIGHT;
		gc->pos_3rd.x   = gc->pos_3rd.x * idev->panel_wid / TPD_WIDTH;
		gc->pos_3rd.y   = gc->pos_3rd.y * idev->panel_hei / TPD_HEIGHT;
		gc->pos_4th.x   = gc->pos_4th.x * idev->panel_wid / TPD_WIDTH;
		gc->pos_4th.y   = gc->pos_4th.y * idev->panel_hei / TPD_HEIGHT;
	}

	ipio_debug("Transfer = %d, Type = %d, clockwise = %d\n", transfer, gc->type, gc->clockwise);
	ipio_debug("Gesture Points: (%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)\n",
			gc->pos_start.x, gc->pos_start.y,
			gc->pos_end.x, gc->pos_end.y,
			gc->pos_1st.x, gc->pos_1st.y,
			gc->pos_2nd.x, gc->pos_2nd.y,
			gc->pos_3rd.x, gc->pos_3rd.y,
			gc->pos_4th.x, gc->pos_4th.y);

	ilitek_tddi_touch_send_debug_data(buf, len);
}

void ilitek_tddi_report_i2cuart_mode(u8 *buf, int len)
{
	int type = buf[3] & 0x0F;
	int need_read_len = 0, one_data_bytes = 0;
	int actual_len = len - 5;
	int uart_len;
	u8 *uart_buf = NULL, *total_buf = NULL;

	ipio_debug("data[3] = %x, type = %x, actual_len = %d\n",
					buf[3], type, actual_len);

	need_read_len = buf[1] * buf[2];

	if (type == 0 || type == 1 || type == 6) {
		one_data_bytes = 1;
	} else if (type == 2 || type == 3) {
		one_data_bytes = 2;
	} else if (type == 4 || type == 5) {
		one_data_bytes = 4;
	}

	need_read_len =  need_read_len * one_data_bytes + 1;
	ipio_debug("need_read_len = %d  one_data_bytes = %d\n", need_read_len, one_data_bytes);

	if (need_read_len < actual_len) {
		ilitek_tddi_touch_send_debug_data(buf, len);
		goto out;
	}

	uart_len = need_read_len - actual_len;
	ipio_debug("uart len = %d\n", uart_len);

	uart_buf = kcalloc(uart_len, sizeof(u8), GFP_KERNEL);
	if (ERR_ALLOC_MEM(uart_buf)) {
		ipio_err("Failed to allocate uart_buf memory %ld\n", PTR_ERR(uart_buf));
		goto out;
	}

	if (idev->read(uart_buf, uart_len) < 0) {
		ipio_err("i2cuart read data failed\n");
		goto out;
	}

	total_buf = kcalloc(len + uart_len, sizeof(u8), GFP_KERNEL);
	if (ERR_ALLOC_MEM(total_buf)) {
		ipio_err("Failed to allocate total_buf memory %ld\n", PTR_ERR(total_buf));
		goto out;
	}

	memcpy(total_buf, buf, len);
	memcpy(total_buf + len, uart_buf, uart_len);
	ilitek_tddi_touch_send_debug_data(total_buf, len + uart_len);

out:
	ipio_kfree((void **)&uart_buf);
	ipio_kfree((void **)&total_buf);
	return;
}
