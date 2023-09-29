/*
 * ar1335.c - AR1335 sensor driver
 * Copyright (c) 2018-2019, e-con Systems.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#include "ar1335_af.h"
#include "mcu_firmware_1335_af.h"

/*
#define AR1335_DEBUG 1
*/
/*!
 * Maintains the information on the current state of the sensor.
 */
static struct ar1335_af ar1335_data;

static int pwdn_gpio, reset_gpio;

/**********************************************************************
 *
 * START of AR1335 related code
 *
 **********************************************************************
 */

static int retries_for_i2c_commands = 5;

static int ar1335_write(struct i2c_client *client, u8 * val, u32 count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = count,
		.buf = val,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register.\n");
		dev_err(&client->dev, "addr: %x; val = %hhu, ret = %d!\n",
			client->addr, *val, ret);
		return ret;
	}

	return 0;
}

static int ar1335_read(struct i2c_client *client, u8 * val, u32 count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.buf = val,
	};

	msg.flags = I2C_M_RD;
	msg.len = count;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;

	return 0;

 err:
	dev_err(&client->dev, "Failed reading register ret = %d!\n", ret);
	return ret;
}

/*
 * The MCU specific functions below depend on the sensor-specific
 * functions above.
 */

/*
 * ---------------------------------------------------------
 *  START of MCU realed functions
 * ---------------------------------------------------------
 */

/*
 * NOTE about modularizing this MCU related function.
 *
 * - The functions:
 *
 *  	x mcu_get_fw_version
 *      x mcu_bload_get_version
 *      x mcu_bload_erase_flash
 *      x mcu_bload_parse_send_cmd
 *      x mcu_bload_go
 *      x mcu_bload_read
 *      x mcu_count_or_list_ctrls
 *      x mcu_count_or_list_fmts
 *      x mcu_get_sensor_id
 *      x mcu_get_ctrl_ui
 *      x mcu_stream_config
 *      x mcu_isp_power_down
 *      x mcu_isp_power_wakeup
 *      x mcu_set_ctrl
 *      x mcu_get_ctrl
 *
 *   seem to directly use platform specific functions:
 *
 *   	x ar1335_write
 *   	x ar1335_read
 *
 *   This could be passed in as a function pointer.
 */

static unsigned short int mcu_bload_calc_crc16(unsigned char *buf, int len)
{
	unsigned short int crc = 0;
	int i = 0;

	if (!buf || !(buf + len))
		return 0;

	for (i = 0; i < len; i++) {
		crc ^= buf[i];
	}

	return crc;
}

static int mcu_bload_ascii2hex(unsigned char ascii)
{
	if (ascii <= '9') {
		return (ascii - '0');
	} else if ((ascii >= 'a') && (ascii <= 'f')) {
		return (0xA + (ascii - 'a'));
	} else if ((ascii >= 'A') && (ascii <= 'F')) {
		return (0xA + (ascii - 'A'));
	}
	return -1;
}

static unsigned char errorcheck(char *data, unsigned int len)
{
	unsigned int i = 0;
	unsigned char crc = 0x00;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
	}

	return crc;
}

/*
 * mcu_get_fw_version:
 *
 * Read the firmware version from the MCU.
 *
 * A success value (0) is returned when the MCU version could be successfully read.
 * else a negative value indicating error is returned.
 */
static int mcu_get_fw_version(struct i2c_client *client, unsigned char *fw_version_1335_af)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	int ret = 0, err = 0, loop;
	/* Query firmware version from MCU */

	/* lock semaphore */
	mutex_lock(&mcu_i2c_mutex_1335_af);

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_VERSION;
	mc_data_1335_af[2] = payload_len >> 8;
	mc_data_1335_af[3] = payload_len & 0xFF;
	mc_data_1335_af[4] = errorcheck(&mc_data_1335_af[2], 2);

	err = ar1335_write(client, mc_data_1335_af, TX_LEN_PKT);
	if (err != 0)
	{
		dev_err(&client->dev, "MCU CMD ID version Error-  %d\n", err);
		ret = -EIO;
		goto exit;
	}

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_VERSION;
	err = ar1335_write(client, mc_data_1335_af, 2);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) MCU CMD ID Write PKT fw Version Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = ar1335_read(client, mc_ret_data_1335_af, RX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) MCU CMD ID Read PKT fw Version Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data_1335_af[4];
	calc_crc = errorcheck(&mc_ret_data_1335_af[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) MCU CMD ID fw Version Error CRC 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	errcode = mc_ret_data_1335_af[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) MCU CMD ID fw Errcode - 0x%02x \n", __func__,
				__LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	/* Read the actual version from MCU*/
	payload_len =
	    ((mc_ret_data_1335_af[2] << 8) | mc_ret_data_1335_af[3]) + HEADER_FOOTER_SIZE;
	memset(mc_ret_data_1335_af, 0x00, payload_len);
	err = ar1335_read(client, mc_ret_data_1335_af, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) MCU fw CMD ID Read Version Error - %d \n", __func__,
				__LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data_1335_af[payload_len - 2];
	calc_crc = errorcheck(&mc_ret_data_1335_af[2], 32);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) MCU fw  CMD ID Version CRC ERROR 0x%02x != 0x%02x \n",
				__func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data_1335_af[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) MCU fw CMD ID Read Payload Error - 0x%02x \n", __func__,
				__LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	for (loop = 0 ; loop < VERSION_SIZE ; loop++ )
		*(fw_version_1335_af+loop) = mc_ret_data_1335_af[2+loop];

	ret = ERRCODE_SUCCESS;
exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex_1335_af);

	return ret;
}

/**
 * mcu_verify_fw_version:
 *
 * Verify the firmware version obtained from the MCU and that found in the
 * firmware file present in our driver.
 *
 * The return value after verification is as follows:
 *
 *   - If the version number matches a success value (0) is returned.
 *
 *   - In case the version number mismatches, a negative value indicating error
 *     is returned.
 *
 *   - In case the  force update bit is set in firmware version in the firmware
 *     file, a positive value is returned.
 */
static int mcu_verify_fw_version(const unsigned char *const fw_version_1335_af)
{
	int loop, i = 0, ret;
	char fw_version_in_file[32] = {0};
	unsigned long file_fw_pos = ARRAY_SIZE(g_mcu_fw_buf_1335_af)-VERSION_FILE_OFFSET;

	/* Get Firmware version from the firmware file */
	for(loop = file_fw_pos; loop < (file_fw_pos+64); loop=loop+2) {
		*(fw_version_in_file+i) = (mcu_bload_ascii2hex(g_mcu_fw_buf_1335_af[loop]) << 4 |
				mcu_bload_ascii2hex(g_mcu_fw_buf_1335_af[loop+1]));
		i++;
	}

	/* Check for forced/always update field in the firmware version present in the firmware file */
	if(fw_version_in_file[17] == '1') {

#ifdef AR1335_DEBUG
		pr_info("Forced Update Enabled - Firmware Version - (%.32s) \n",
			fw_version_1335_af);
#endif

		ret = 2;
	}
	else {
		for(i = 0; i < VERSION_SIZE; i++) {
			if(fw_version_in_file[i] != fw_version_1335_af[i]) {

				pr_info("Previous Firmware Version - (%.32s)\n", fw_version_1335_af);

				ret = -1;
				break;
			}
		}

		if (i == VERSION_SIZE)
			ret = ERRCODE_SUCCESS;
	}

	return ret;
}

static int mcu_bload_get_version(struct i2c_client *client)
{
	int ret = 0;

	/*----------------------------- GET VERSION -------------------- */

	/*   Write Get Version CMD */
	g_bload_buf_1335_af[0] = BL_GET_VERSION;
	g_bload_buf_1335_af[1] = ~(BL_GET_VERSION);

	ret = ar1335_write(client, g_bload_buf_1335_af, 2);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = ar1335_read(client, g_bload_buf_1335_af, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	if (g_bload_buf_1335_af[0] != 'y') {
		/*   NACK Received */
		dev_err(&client->dev," NACK Received... exiting.. \n");
		return -1;
	}

	ret = ar1335_read(client, g_bload_buf_1335_af, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	ret = ar1335_read(client, g_bload_buf_1335_af, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed\n");
		return -1;
	}

	/* ---------------- GET VERSION END ------------------- */

	return 0;
}

static int mcu_bload_erase_flash(struct i2c_client *client)
{
	unsigned short int pagenum = 0x0000;
	int ret = 0, i = 0, checksum = 0;

	/* --------------- ERASE FLASH --------------------- */

	for (i = 0; i < NUM_ERASE_CYCLES; i++) {

		checksum = 0x00;
		/*   Write Erase Pages CMD */
		g_bload_buf_1335_af[0] = BL_ERASE_MEM_NS;
		g_bload_buf_1335_af[1] = ~(BL_ERASE_MEM_NS);

		ret = ar1335_write(client, g_bload_buf_1335_af, 2);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = ar1335_read(client, g_bload_buf_1335_af, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf_1335_af[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

		g_bload_buf_1335_af[0] = (MAX_PAGES - 1) >> 8;
		g_bload_buf_1335_af[1] = (MAX_PAGES - 1) & 0xFF;
		g_bload_buf_1335_af[2] = g_bload_buf_1335_af[0] ^ g_bload_buf_1335_af[1];

		ret = ar1335_write(client, g_bload_buf_1335_af, 3);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = ar1335_read(client, g_bload_buf_1335_af, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf_1335_af[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

		for (pagenum = 0; pagenum < MAX_PAGES; pagenum++) {
			g_bload_buf_1335_af[(2 * pagenum)] =
			    (pagenum + (i * MAX_PAGES)) >> 8;
			g_bload_buf_1335_af[(2 * pagenum) + 1] =
			    (pagenum + (i * MAX_PAGES)) & 0xFF;
			checksum =
			    checksum ^ g_bload_buf_1335_af[(2 * pagenum)] ^
			    g_bload_buf_1335_af[(2 * pagenum) + 1];
		}
		g_bload_buf_1335_af[2 * MAX_PAGES] = checksum;

		ret = ar1335_write(client, g_bload_buf_1335_af, (2 * MAX_PAGES) + 1);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

 poll_busy:
		/*   Wait for ACK or NACK */
		ret = ar1335_read(client, g_bload_buf_1335_af, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf_1335_af[0] == RESP_BUSY)
			goto poll_busy;

		if (g_bload_buf_1335_af[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

#ifdef AR1335_DEBUG
		pr_info(" ERASE Sector %d success !! \n", i + 1);
#endif
	}

	/* ------------ ERASE FLASH END ----------------------- */

	return 0;
}

static unsigned char mcu_bload_inv_checksum(unsigned char *buf, int len)
{
	unsigned int checksum = 0x00;
	int i = 0;

	if (!buf || !(buf + len))
		return 0;

	for (i = 0; i < len; i++) {
		checksum = (checksum + buf[i]);
	}

	checksum &= (0xFF);
	return (~(checksum) + 1);
}

static int mcu_bload_parse_send_cmd(struct i2c_client *client,
			     unsigned char *bytearray, int rec_len)
{
	IHEX_RECORD *ihex_rec = NULL;
	unsigned char checksum = 0, calc_checksum = 0;
	int i = 0, ret = 0;

	if (!bytearray)
		return -1;

	ihex_rec = (IHEX_RECORD *) bytearray;
	ihex_rec->addr = htons(ihex_rec->addr);

	checksum = bytearray[rec_len - 1];

	calc_checksum = mcu_bload_inv_checksum(bytearray, rec_len - 1);
	if (checksum != calc_checksum) {
		dev_err(&client->dev," Invalid Checksum 0x%02x != 0x%02x !! \n",
		       checksum, calc_checksum);
		return -1;
	}

	if ((ihex_rec->rectype == REC_TYPE_ELA)
	    && (ihex_rec->addr == 0x0000)
	    && (ihex_rec->datasize = 0x02)) {
		/*   Upper 32-bit configuration */
		g_bload_flashaddr_1335_af = (ihex_rec->recdata[0] <<
				     24) | (ihex_rec->recdata[1]
					    << 16);
#ifdef AR1335_DEBUG
		pr_info("Updated Flash Addr = 0x%08x \n",
			     g_bload_flashaddr_1335_af);
#endif

	} else if (ihex_rec->rectype == REC_TYPE_DATA) {
		/*   Flash Data into Flashaddr */

		g_bload_flashaddr_1335_af =
		    (g_bload_flashaddr_1335_af & 0xFFFF0000) | (ihex_rec->addr);
		g_bload_crc16_1335_af ^=
		    mcu_bload_calc_crc16(ihex_rec->recdata, ihex_rec->datasize);

		/*   Write Erase Pages CMD */
		g_bload_buf_1335_af[0] = BL_WRITE_MEM_NS;
		g_bload_buf_1335_af[1] = ~(BL_WRITE_MEM_NS);

		ret = ar1335_write(client, g_bload_buf_1335_af, 2);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = ar1335_read(client, g_bload_buf_1335_af, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf_1335_af[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

		g_bload_buf_1335_af[0] = (g_bload_flashaddr_1335_af & 0xFF000000) >> 24;
		g_bload_buf_1335_af[1] = (g_bload_flashaddr_1335_af & 0x00FF0000) >> 16;
		g_bload_buf_1335_af[2] = (g_bload_flashaddr_1335_af & 0x0000FF00) >> 8;
		g_bload_buf_1335_af[3] = (g_bload_flashaddr_1335_af & 0x000000FF);
		g_bload_buf_1335_af[4] =
		    g_bload_buf_1335_af[0] ^ g_bload_buf_1335_af[1] ^ g_bload_buf_1335_af[2] ^
		    g_bload_buf_1335_af[3];

		ret = ar1335_write(client, g_bload_buf_1335_af, 5);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

		/*   Wait for ACK or NACK */
		ret = ar1335_read(client, g_bload_buf_1335_af, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf_1335_af[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

		g_bload_buf_1335_af[0] = ihex_rec->datasize - 1;
		checksum = g_bload_buf_1335_af[0];
		for (i = 0; i < ihex_rec->datasize; i++) {
			g_bload_buf_1335_af[i + 1] = ihex_rec->recdata[i];
			checksum ^= g_bload_buf_1335_af[i + 1];
		}

		g_bload_buf_1335_af[i + 1] = checksum;

		ret = ar1335_write(client, g_bload_buf_1335_af, i + 2);
		if (ret < 0) {
			dev_err(&client->dev,"Write Failed \n");
			return -1;
		}

 poll_busy:
		/*   Wait for ACK or NACK */
		ret = ar1335_read(client, g_bload_buf_1335_af, 1);
		if (ret < 0) {
			dev_err(&client->dev,"Read Failed \n");
			return -1;
		}

		if (g_bload_buf_1335_af[0] == RESP_BUSY)
			goto poll_busy;

		if (g_bload_buf_1335_af[0] != RESP_ACK) {
			/*   NACK Received */
			dev_err(&client->dev," NACK Received... exiting.. \n");
			return -1;
		}

	} else if (ihex_rec->rectype == REC_TYPE_SLA) {
		/*   Update Instruction pointer to this address */

	} else if (ihex_rec->rectype == REC_TYPE_EOF) {
		/*   End of File - Issue I2C Go Command */
		return 0;
	} else {

		/*   Unhandled Type */
		dev_err(&client->dev,"Unhandled Command Type \n");
		return -1;
	}

	return 0;
}

static int mcu_bload_update_fw(struct i2c_client *client)
{
	/* exclude NULL character at end of string */
	unsigned long hex_file_size = ARRAY_SIZE(g_mcu_fw_buf_1335_af) - 1;
	unsigned char *wbuf = devm_kzalloc(&client->dev, MAX_BUF_LEN, GFP_KERNEL);
	int i = 0, recindex = 0, ret = 0;

	for (i = 0; i < hex_file_size; i++) {
		if ((recindex == 0) && (g_mcu_fw_buf_1335_af[i] == ':')) {
			/*  pr_info("Start of a Record \n"); */
		} else if (g_mcu_fw_buf_1335_af[i] == CR) {
			/*   No Implementation */
		} else if (g_mcu_fw_buf_1335_af[i] == LF) {
			if (recindex == 0) {
				/*   Parsing Complete */
				break;
			}

			/*   Analyze Packet and Send Commands */
			ret = mcu_bload_parse_send_cmd(client, wbuf, recindex);
			if (ret < 0) {
				dev_err(&client->dev,"Error in Processing Commands \n");
				break;
			}

			recindex = 0;

		} else {
			/*   Parse Rec Data */
			if ((ret = mcu_bload_ascii2hex(g_mcu_fw_buf_1335_af[i])) < 0) {
				dev_err(&client->dev,
					"Invalid Character - 0x%02x !! \n",
				     g_mcu_fw_buf_1335_af[i]);
				break;
			}

			wbuf[recindex] = (0xF0 & (ret << 4));
			i++;

			if ((ret = mcu_bload_ascii2hex(g_mcu_fw_buf_1335_af[i])) < 0) {
				dev_err(&client->dev,
				    "Invalid Character - 0x%02x !!!! \n",
				     g_mcu_fw_buf_1335_af[i]);
				break;
			}

			wbuf[recindex] |= (0x0F & ret);
			recindex++;
		}
	}

#ifdef AR1335_DEBUG
	pr_info("Program FLASH Success !! - CRC = 0x%04x \n",
		     g_bload_crc16_1335_af);
#endif

	/* ------------ PROGRAM FLASH END ----------------------- */
	devm_kfree(&client->dev,wbuf);
	return ret;
}

static int mcu_bload_read(struct i2c_client *client,
		   unsigned int g_bload_flashaddr_1335_af, char *bytearray,
		   unsigned int len)
{
	int ret = 0;

	g_bload_buf_1335_af[0] = BL_READ_MEM;
	g_bload_buf_1335_af[1] = ~(BL_READ_MEM);

	ret = ar1335_write(client, g_bload_buf_1335_af, 2);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = ar1335_read(client, g_bload_buf_1335_af, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	if (g_bload_buf_1335_af[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev," NACK Received... exiting.. \n");
		return -1;
	}

	g_bload_buf_1335_af[0] = (g_bload_flashaddr_1335_af & 0xFF000000) >> 24;
	g_bload_buf_1335_af[1] = (g_bload_flashaddr_1335_af & 0x00FF0000) >> 16;
	g_bload_buf_1335_af[2] = (g_bload_flashaddr_1335_af & 0x0000FF00) >> 8;
	g_bload_buf_1335_af[3] = (g_bload_flashaddr_1335_af & 0x000000FF);
	g_bload_buf_1335_af[4] =
	    g_bload_buf_1335_af[0] ^ g_bload_buf_1335_af[1] ^ g_bload_buf_1335_af[2] ^ g_bload_buf_1335_af[3];

	ret = ar1335_write(client, g_bload_buf_1335_af, 5);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = ar1335_read(client, g_bload_buf_1335_af, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	if (g_bload_buf_1335_af[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev," NACK Received... exiting.. \n");
		return -1;
	}

	g_bload_buf_1335_af[0] = len - 1;
	g_bload_buf_1335_af[1] = ~(len - 1);

	ret = ar1335_write(client, g_bload_buf_1335_af, 2);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	/*   Wait for ACK or NACK */
	ret = ar1335_read(client, g_bload_buf_1335_af, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	if (g_bload_buf_1335_af[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev," NACK Received... exiting.. \n");
		return -1;
	}

	ret = ar1335_read(client, bytearray, len);
	if (ret < 0) {
		dev_err(&client->dev,"Read Failed \n");
		return -1;
	}

	return 0;
}

static int mcu_bload_verify_flash(struct i2c_client *client,
			   unsigned short int orig_crc)
{
	char bytearray[FLASH_READ_LEN];
	unsigned short int calc_crc = 0;
	unsigned int flash_addr = FLASH_START_ADDRESS, i = 0;

	while ((i + FLASH_READ_LEN) <= FLASH_SIZE) {
		memset(bytearray, 0x0, FLASH_READ_LEN);

		if (mcu_bload_read(
			client, flash_addr + i, bytearray, FLASH_READ_LEN) < 0) {
			dev_err(&client->dev," i2c_bload_read FAIL !! \n");
			return -1;
		}

		calc_crc ^= mcu_bload_calc_crc16(bytearray, FLASH_READ_LEN);
		i += FLASH_READ_LEN;
	}

	if ((FLASH_SIZE - i) > 0) {
		memset(bytearray, 0x0, FLASH_READ_LEN);

		if (mcu_bload_read(
			client, flash_addr + i, bytearray, (FLASH_SIZE - i)
			) < 0) {
			dev_err(&client->dev," i2c_bload_read FAIL !! \n");
			return -1;
		}

		calc_crc ^= mcu_bload_calc_crc16(bytearray, FLASH_READ_LEN);
	}

	if (orig_crc != calc_crc) {
		dev_err(&client->dev," CRC verification fail !! 0x%04x != 0x%04x \n",
		       orig_crc, calc_crc);
//		return -1;
	}

#ifdef AR1335_DEBUG
	pr_info(" CRC Verification Success 0x%04x == 0x%04x \n",
		     orig_crc, calc_crc);
#endif

	return 0;
}

static int mcu_bload_go(struct i2c_client *client)
{
	int ret = 0;

	g_bload_buf_1335_af[0] = BL_GO;
	g_bload_buf_1335_af[1] = ~(BL_GO);

	ret = ar1335_write(client, g_bload_buf_1335_af, 2);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	ret = ar1335_read(client, g_bload_buf_1335_af, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Failed Read 1 \n");
		return -1;
	}

	/*   Start Address */
	g_bload_buf_1335_af[0] = (FLASH_START_ADDRESS & 0xFF000000) >> 24;
	g_bload_buf_1335_af[1] = (FLASH_START_ADDRESS & 0x00FF0000) >> 16;
	g_bload_buf_1335_af[2] = (FLASH_START_ADDRESS & 0x0000FF00) >> 8;
	g_bload_buf_1335_af[3] = (FLASH_START_ADDRESS & 0x000000FF);
	g_bload_buf_1335_af[4] =
	    g_bload_buf_1335_af[0] ^ g_bload_buf_1335_af[1] ^ g_bload_buf_1335_af[2] ^ g_bload_buf_1335_af[3];

	ret = ar1335_write(client, g_bload_buf_1335_af, 5);
	if (ret < 0) {
		dev_err(&client->dev,"Write Failed \n");
		return -1;
	}

	ret = ar1335_read(client, g_bload_buf_1335_af, 1);
	if (ret < 0) {
		dev_err(&client->dev,"Failed Read 1 \n");
		return -1;
	}

	if (g_bload_buf_1335_af[0] != RESP_ACK) {
		/*   NACK Received */
		dev_err(&client->dev," NACK Received... exiting.. \n");
		return -1;
	}

	return 0;
}

static int mcu_fw_update(struct i2c_client *client, unsigned char *mcu_fw_version)
{
	int ret = 0;
	g_bload_crc16_1335_af = 0;

	/* Read Firmware version from bootloader MCU */
	ret = mcu_bload_get_version(client);
	if (ret < 0) {
		dev_err(&client->dev," Error in Get Version \n");
		goto exit;
	}

#ifdef AR1335_DEBUG
	pr_info(" Get Version SUCCESS !! \n");
#endif

	/* Erase firmware present in the MCU and flash new firmware*/
	ret = mcu_bload_erase_flash(client);
	if (ret < 0) {
		dev_err(&client->dev," Error in Erase Flash \n");
		goto exit;
	}

#ifdef AR1335_DEBUG
	pr_info("Erase Flash Success !! \n");
#endif

	/* Read the firmware present in the firmware file */
	if ((ret = mcu_bload_update_fw(client)) < 0) {
		dev_err(&client->dev," Write Flash FAIL !! \n");
		goto exit;
	}

	/* Verify the checksum for the update firmware */
	if ((ret = mcu_bload_verify_flash(client, g_bload_crc16_1335_af)) < 0) {
		dev_err(&client->dev," verify_flash FAIL !! \n");
		goto exit;
	}

	/* Reverting from bootloader mode */
	/* I2C GO Command */
	if ((ret = mcu_bload_go(client)) < 0) {
		dev_err(&client->dev," i2c_bload_go FAIL !! \n");
		goto exit;
	}

	if(mcu_fw_version) {

#ifdef AR1335_DEBUG
	pr_info("(%s) - Firmware Updated - (%.32s)\n",
				__func__, mcu_fw_version);
#endif

	}
 exit:
	return ret;
}

static int mcu_count_or_list_ctrls(struct i2c_client *client,
			  ISP_CTRL_INFO * mcu_cam_ctrl, int *numctrls)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	uint16_t index = 0;
	int ret = 0, err = 0;

	/* lock semaphore */
	mutex_lock(&mcu_i2c_mutex_1335_af);

	/* Array of Ctrl Info */
	while (1) {
		/* First Txn Payload length = 0 */
		payload_len = 2;

		mc_data_1335_af[0] = CMD_SIGNATURE;
		mc_data_1335_af[1] = CMD_ID_GET_CTRL_INFO;
		mc_data_1335_af[2] = payload_len >> 8;
		mc_data_1335_af[3] = payload_len & 0xFF;
		mc_data_1335_af[4] = errorcheck(&mc_data_1335_af[2], 2);

		ar1335_write(client, mc_data_1335_af, TX_LEN_PKT);

		mc_data_1335_af[0] = CMD_SIGNATURE;
		mc_data_1335_af[1] = CMD_ID_GET_CTRL_INFO;
		mc_data_1335_af[2] = index >> 8;
		mc_data_1335_af[3] = index & 0xFF;
		mc_data_1335_af[4] = errorcheck(&mc_data_1335_af[2], 2);
		err = ar1335_write(client, mc_data_1335_af, 5);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			ret = -EIO;
			goto exit;
		}

		err = ar1335_read(client, mc_ret_data_1335_af, RX_LEN_PKT);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			ret = -EIO;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data_1335_af[4];
		calc_crc = errorcheck(&mc_ret_data_1335_af[2], 2);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
			    " %s(%d) CRC 0x%02x != 0x%02x \n",
			     __func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			goto exit;
		}

		if (((mc_ret_data_1335_af[2] << 8) | mc_ret_data_1335_af[3]) == 0) {
			*numctrls = index;
			break;
		}

		payload_len =
		    ((mc_ret_data_1335_af[2] << 8) | mc_ret_data_1335_af[3]) +
		    HEADER_FOOTER_SIZE;
		errcode = mc_ret_data_1335_af[5];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
			    " %s(%d) Errcode - 0x%02x \n",
			     __func__, __LINE__, errcode);
			ret = -EIO;
			goto exit;
		}

		memset(mc_ret_data_1335_af, 0x00, payload_len);
		err = ar1335_read(client, mc_ret_data_1335_af, payload_len);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			ret = -1;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data_1335_af[payload_len - 2];
		calc_crc =
		    errorcheck(&mc_ret_data_1335_af[2],
				 payload_len - HEADER_FOOTER_SIZE);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
			    " %s(%d) CRC 0x%02x != 0x%02x \n",
			     __func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			goto exit;
		}

		/* Verify Errcode */
		errcode = mc_ret_data_1335_af[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
			    " %s(%d) Errcode - 0x%02x \n",
			     __func__, __LINE__, errcode);
			ret = -EINVAL;
			goto exit;
		}

		if(mcu_cam_ctrl != NULL) {
			int sorted_elem = index - 1, elem = index;

			/* append ctrl info in array */
			mcu_cam_ctrl[index].ctrl_id =
				mc_ret_data_1335_af[2] << 24 | mc_ret_data_1335_af[3] << 16 | mc_ret_data_1335_af[4]
				<< 8 | mc_ret_data_1335_af[5];
			mcu_cam_ctrl[index].ctrl_type = mc_ret_data_1335_af[6];

			switch (mcu_cam_ctrl[index].ctrl_type) {
				case CTRL_STANDARD:
					mcu_cam_ctrl[index].ctrl_data.std.ctrl_min =
						mc_ret_data_1335_af[7] << 24 | mc_ret_data_1335_af[8] << 16
						| mc_ret_data_1335_af[9] << 8 | mc_ret_data_1335_af[10];

					mcu_cam_ctrl[index].ctrl_data.std.ctrl_max =
						mc_ret_data_1335_af[11] << 24 | mc_ret_data_1335_af[12] <<
						16 | mc_ret_data_1335_af[13]
						<< 8 | mc_ret_data_1335_af[14];

					mcu_cam_ctrl[index].ctrl_data.std.ctrl_def =
						mc_ret_data_1335_af[15] << 24 | mc_ret_data_1335_af[16] <<
						16 | mc_ret_data_1335_af[17]
						<< 8 | mc_ret_data_1335_af[18];

					mcu_cam_ctrl[index].ctrl_data.std.ctrl_step =
						mc_ret_data_1335_af[19] << 24 | mc_ret_data_1335_af[20] <<
						16 | mc_ret_data_1335_af[21]
						<< 8 | mc_ret_data_1335_af[22];

					mcu_cam_ctrl[index].mcu_ctrl_index = index;
					break;

				case CTRL_EXTENDED:
					/* Not Implemented */
					break;
			}

#ifdef AR1335_DEBUG
			pr_info("Control: ID: 0x%x; Type: %u; min: %d; Max: %d; Def: %d; Step: %u\n",
					mcu_cam_ctrl[index].ctrl_id,
					mcu_cam_ctrl[index].ctrl_type,
					mcu_cam_ctrl[index].ctrl_data.std.ctrl_min,
					mcu_cam_ctrl[index].ctrl_data.std.ctrl_max,
					mcu_cam_ctrl[index].ctrl_data.std.ctrl_def,
					mcu_cam_ctrl[index].ctrl_data.std.ctrl_step
			);
#endif

			ctrldb[index] = mcu_cam_ctrl[index].ctrl_id;

			/*
			 * Keep the control list and control db sorted.
			 */
			while(
				sorted_elem >= 0 &&
				(
					mcu_cam_ctrl[sorted_elem].ctrl_id >
					mcu_cam_ctrl[elem].ctrl_id
				)
			)
			{
				ISP_CTRL_INFO swap_ctrl_elem;
				uint32_t swap_ctrldb_elem;

				/*
				 * Swap the elements in the mcu_cam_ctrl list
				 */
				memcpy(&swap_ctrl_elem, (mcu_cam_ctrl + sorted_elem), sizeof(ISP_CTRL_INFO));
				memcpy((mcu_cam_ctrl + sorted_elem), (mcu_cam_ctrl + elem), sizeof(ISP_CTRL_INFO));
				memcpy((mcu_cam_ctrl + elem), &swap_ctrl_elem, sizeof(ISP_CTRL_INFO));

				/*
				 * Swap the elements in ctrldb
				 */
				swap_ctrldb_elem = ctrldb[sorted_elem];
				ctrldb[sorted_elem] = ctrldb[elem];
				ctrldb[elem] = swap_ctrldb_elem;

				elem = sorted_elem;
				sorted_elem = elem - 1;
			}
		}
		index++;
	}

 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex_1335_af);

	return ret;
}

static int mcu_count_or_list_fmts(struct i2c_client *client, ISP_STREAM_INFO *stream_info, int *frm_fmt_size)
{
	uint32_t payload_len = 0, err = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0, skip = 0;
	uint16_t index = 0, mode = 0;

	int loop = 0, num_frates = 0, ret = 0;

	/* Stream Info Variables */

	/* lock semaphore */
	mutex_lock(&mcu_i2c_mutex_1335_af);

	/* List all formats from MCU and append to mcu_ar1335_frmfmt array */

	for (index = 0;; index++) {
		/* First Txn Payload length = 0 */
		payload_len = 2;

		mc_data_1335_af[0] = CMD_SIGNATURE;
		mc_data_1335_af[1] = CMD_ID_GET_STREAM_INFO;
		mc_data_1335_af[2] = payload_len >> 8;
		mc_data_1335_af[3] = payload_len & 0xFF;
		mc_data_1335_af[4] = errorcheck(&mc_data_1335_af[2], 2);

		ar1335_write(client, mc_data_1335_af, TX_LEN_PKT);

		mc_data_1335_af[0] = CMD_SIGNATURE;
		mc_data_1335_af[1] = CMD_ID_GET_STREAM_INFO;
		mc_data_1335_af[2] = index >> 8;
		mc_data_1335_af[3] = index & 0xFF;
		mc_data_1335_af[4] = errorcheck(&mc_data_1335_af[2], 2);
		err = ar1335_write(client, mc_data_1335_af, 5);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			ret = -EIO;
			goto exit;
		}

		err = ar1335_read(client, mc_ret_data_1335_af, RX_LEN_PKT);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			ret = -EIO;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data_1335_af[4];
		calc_crc = errorcheck(&mc_ret_data_1335_af[2], 2);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
				" %s(%d) CRC 0x%02x != 0x%02x \n",
			     __func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			goto exit;
		}

		if (((mc_ret_data_1335_af[2] << 8) | mc_ret_data_1335_af[3]) == 0) {
			if(stream_info == NULL) {
				*frm_fmt_size = index;
			} else {
				*frm_fmt_size = mode;
			}
			break;
		}

		payload_len =
		    ((mc_ret_data_1335_af[2] << 8) | mc_ret_data_1335_af[3]) +
		    HEADER_FOOTER_SIZE;
		errcode = mc_ret_data_1335_af[5];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
				" %s(%d) Errcode - 0x%02x \n",
			     __func__, __LINE__, errcode);
			ret = -EIO;
			goto exit;
		}

		memset(mc_ret_data_1335_af, 0x00, payload_len);
		err = ar1335_read(client, mc_ret_data_1335_af, payload_len);
		if (err != 0) {
			dev_err(&client->dev," %s(%d) Error - %d \n",
			       __func__, __LINE__, err);
			ret = -1;
			goto exit;
		}

		/* Verify CRC */
		orig_crc = mc_ret_data_1335_af[payload_len - 2];
		calc_crc =
		    errorcheck(&mc_ret_data_1335_af[2],
				 payload_len - HEADER_FOOTER_SIZE);
		if (orig_crc != calc_crc) {
			dev_err(&client->dev,
				" %s(%d) CRC 0x%02x != 0x%02x \n",
			     __func__, __LINE__, orig_crc, calc_crc);
			ret = -EINVAL;
			goto exit;
		}

		/* Verify Errcode */
		errcode = mc_ret_data_1335_af[payload_len - 1];
		if (errcode != ERRCODE_SUCCESS) {
			dev_err(&client->dev,
				" %s(%d) Errcode - 0x%02x \n",
			     __func__, __LINE__, errcode);
			ret = -EIO;
			goto exit;
		}
		if(stream_info != NULL) {
			stream_info->fmt_fourcc =
			    mc_ret_data_1335_af[2] << 24 | mc_ret_data_1335_af[3] << 16 | mc_ret_data_1335_af[4]
			    << 8 | mc_ret_data_1335_af[5];
			stream_info->width = mc_ret_data_1335_af[6] << 8 | mc_ret_data_1335_af[7];
			stream_info->height = mc_ret_data_1335_af[8] << 8 | mc_ret_data_1335_af[9];
			stream_info->frame_rate_type = mc_ret_data_1335_af[10];

			switch (stream_info->frame_rate_type) {
			case FRAME_RATE_DISCRETE:
				stream_info->frame_rate.disc.frame_rate_num =
				    mc_ret_data_1335_af[11] << 8 | mc_ret_data_1335_af[12];

				stream_info->frame_rate.disc.frame_rate_denom =
				    mc_ret_data_1335_af[13] << 8 | mc_ret_data_1335_af[14];

				break;

			case FRAME_RATE_CONTINOUS:
				dev_err(&client->dev,
					" The Stream format at index 0x%04x has FRAME_RATE_CONTINOUS,"
				     "which is unsupported !! \n", index);

#if 0
				stream_info.frame_rate.cont.frame_rate_min_num =
				    mc_ret_data_1335_af[11] << 8 | mc_ret_data_1335_af[12];
				stream_info.frame_rate.cont.frame_rate_min_denom =
				    mc_ret_data_1335_af[13] << 8 | mc_ret_data_1335_af[14];

				stream_info.frame_rate.cont.frame_rate_max_num =
				    mc_ret_data_1335_af[15] << 8 | mc_ret_data_1335_af[16];
				stream_info.frame_rate.cont.frame_rate_max_denom =
				    mc_ret_data_1335_af[17] << 8 | mc_ret_data_1335_af[18];

				stream_info.frame_rate.cont.frame_rate_step_num =
				    mc_ret_data_1335_af[19] << 8 | mc_ret_data_1335_af[20];
				stream_info.frame_rate.cont.frame_rate_step_denom =
				    mc_ret_data_1335_af[21] << 8 | mc_ret_data_1335_af[22];
				break;
#endif
				continue;
			}

			switch (stream_info->fmt_fourcc) {
			/*
			 * We check for UYVY here instead of YUYV as the output from the sensor
			 * is UYVY. We swap it to YUYV only making changes in the platform driver.
			 */
			case V4L2_PIX_FMT_UYVY:
				/* ar1335_codes is already populated with V4L2_PIX_FMT_YUYV */
				/* check if width and height are already in array - update frame rate only */
				for (loop = 0; loop < (mode); loop++) {
					if ((ar1335_data.mcu_cam_frmfmt[loop].size.width ==
					     stream_info->width)
					    && (ar1335_data.mcu_cam_frmfmt[loop].size.height ==
						stream_info->height)) {

						num_frates =
						    ar1335_data.mcu_cam_frmfmt[loop].num_framerates;
						*((int *)(ar1335_data.mcu_cam_frmfmt[loop].framerates) + num_frates)
						    = (int)(stream_info->frame_rate.
							    disc.frame_rate_num /
							    stream_info->frame_rate.
							    disc.frame_rate_denom);

						ar1335_data.mcu_cam_frmfmt[loop].num_framerates++;

						streamdb[index] = loop;
						skip = 1;
						break;
					}
				}

				if (skip) {
					skip = 0;
					continue;
				}

				/* Add Width, Height, Frame Rate array, Mode into mcu_ar1335_frmfmt array */
				ar1335_data.mcu_cam_frmfmt[mode].size.width = stream_info->width;
				ar1335_data.mcu_cam_frmfmt[mode].size.height = stream_info->height;

				num_frates = ar1335_data.mcu_cam_frmfmt[mode].num_framerates;

				*(ar1335_data.mcu_cam_frmfmt[mode].framerates + num_frates) =
				    (int)(stream_info->frame_rate.disc.frame_rate_num /
					  stream_info->frame_rate.disc.frame_rate_denom);

				ar1335_data.mcu_cam_frmfmt[mode].num_framerates++;

				ar1335_data.mcu_cam_frmfmt[mode].mode = mode;
				ar1335_data.mcu_cam_frmfmt[mode].mode = mode;
				streamdb[index] = mode;
				mode++;
				break;

			default:
				dev_err(&client->dev,
					" The Stream format at index 0x%04x has format 0x%08x ,"
				     "which is unsupported !! \n", index,
				     stream_info->fmt_fourcc);
			}
		}
	}

 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex_1335_af);

	return ret;
}

/*
 * Function to initialise the data related to MCU. Needs to be called
 * before trying to use them.
 */
static int mcu_data_init(struct device *dev, int frm_fmt_size)
{
	int loop = 0;

	if (dev == NULL)
	{
		dev_err(dev, "%s: Invalid device parameter\n", __func__);
		return -EINVAL;
	}

	mcu_ctrl_info = devm_kzalloc(dev, sizeof(ISP_CTRL_INFO) * num_ctrls, GFP_KERNEL);
	if(!mcu_ctrl_info) {
		dev_err(dev, "Unable to allocate memory \n");
		return -ENOMEM;
	}

	ctrldb = devm_kzalloc(dev, sizeof(uint32_t) * num_ctrls, GFP_KERNEL);
	if(!ctrldb) {
		dev_err(dev, "Unable to allocate memory \n");
		return -ENOMEM;
	}

	stream_info = devm_kzalloc(dev, sizeof(ISP_STREAM_INFO) * (frm_fmt_size + 1), GFP_KERNEL);

	streamdb = devm_kzalloc(dev, sizeof(int) * (frm_fmt_size + 1), GFP_KERNEL);
	if(!streamdb) {
		dev_err(dev,"Unable to allocate memory \n");
		return -ENOMEM;
	}

	ar1335_data.mcu_cam_frmfmt = devm_kzalloc(dev, sizeof(struct mcu_frmfmt) * (frm_fmt_size), GFP_KERNEL);
	if(!ar1335_data.mcu_cam_frmfmt) {
		dev_err(dev, "Unable to allocate memory \n");
		return -ENOMEM;
	}

	for(; loop < frm_fmt_size; loop++) {
		ar1335_data.mcu_cam_frmfmt[loop].framerates = devm_kzalloc(dev, sizeof(int) * MAX_NUM_FRATES, GFP_KERNEL);
		if(!ar1335_data.mcu_cam_frmfmt[loop].framerates) {
			dev_err(dev, "Unable to allocate memory \n");
			return -ENOMEM;
		}
	}

	return 0;
}

static int mcu_get_sensor_id(struct i2c_client *client, uint16_t * sensor_id)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;

	int ret = 0, err = 0;

	/* lock semaphore */
	mutex_lock(&mcu_i2c_mutex_1335_af);

	/* Read the version info. from Micro controller */

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_GET_SENSOR_ID;
	mc_data_1335_af[2] = payload_len >> 8;
	mc_data_1335_af[3] = payload_len & 0xFF;
	mc_data_1335_af[4] = errorcheck(&mc_data_1335_af[2], 2);

	ar1335_write(client, mc_data_1335_af, TX_LEN_PKT);

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_GET_SENSOR_ID;
	err = ar1335_write(client, mc_data_1335_af, 2);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = ar1335_read(client, mc_ret_data_1335_af, RX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data_1335_af[4];
	calc_crc = errorcheck(&mc_ret_data_1335_af[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	errcode = mc_ret_data_1335_af[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
		       __func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	payload_len =
	    ((mc_ret_data_1335_af[2] << 8) | mc_ret_data_1335_af[3]) + HEADER_FOOTER_SIZE;

	memset(mc_ret_data_1335_af, 0x00, payload_len);
	err = ar1335_read(client, mc_ret_data_1335_af, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data_1335_af[payload_len - 2];
	calc_crc = errorcheck(&mc_ret_data_1335_af[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data_1335_af[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
		       __func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	*sensor_id = mc_ret_data_1335_af[2] << 8 | mc_ret_data_1335_af[3];

 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex_1335_af);

	return ret;
}

static int mcu_get_cmd_status(struct i2c_client *client,
			      uint8_t * cmd_id, uint16_t * cmd_status,
			      uint8_t * ret_code)
{
	uint32_t payload_len = 0;
	uint8_t orig_crc = 0, calc_crc = 0;
	int err = 0;

	/* No Semaphore in Get command Status */

	/* First Txn Payload length = 0 */
	payload_len = 1;

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_GET_STATUS;
	mc_data_1335_af[2] = payload_len >> 8;
	mc_data_1335_af[3] = payload_len & 0xFF;
	mc_data_1335_af[4] = errorcheck(&mc_data_1335_af[2], 2);

	ar1335_write(client, mc_data_1335_af, TX_LEN_PKT);

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_GET_STATUS;
	mc_data_1335_af[2] = *cmd_id;
	err = ar1335_write(client, mc_data_1335_af, 3);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		return -EIO;
	}

	payload_len = CMD_STATUS_MSG_LEN;
	memset(mc_ret_data_1335_af, 0x00, payload_len);
	err = ar1335_read(client, mc_ret_data_1335_af, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		return -EIO;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data_1335_af[payload_len - 2];
	calc_crc = errorcheck(&mc_ret_data_1335_af[2], 3);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		return -EINVAL;
	}

	*cmd_id = mc_ret_data_1335_af[2];
	*cmd_status = mc_ret_data_1335_af[3] << 8 | mc_ret_data_1335_af[4];
	*ret_code = mc_ret_data_1335_af[payload_len - 1];

	return 0;
}

static int mcu_isp_init(struct i2c_client *client)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 1000, err = 0;

	/* check current status - if initialized, no need for Init */
	cmd_id = CMD_ID_INIT_CAM;
	if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) < 0) {
		dev_err(&client->dev," %s(%d) Error \n", __func__, __LINE__);
		return -EIO;
	}

	if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
	    (retcode == ERRCODE_SUCCESS)) {

#ifdef AR1335_DEBUG
		pr_info(" Already Initialized !! \n");
#endif

		return 0;
	}

	/* call ISP init command */

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_INIT_CAM;
	mc_data_1335_af[2] = payload_len >> 8;
	mc_data_1335_af[3] = payload_len & 0xFF;
	mc_data_1335_af[4] = errorcheck(&mc_data_1335_af[2], 2);

	ar1335_write(client, mc_data_1335_af, TX_LEN_PKT);

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_INIT_CAM;
	err = ar1335_write(client, mc_data_1335_af, 2);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		return -EIO;
	}

	while (--retry > 0) {
		/* Some Sleep for init to process */
		mdelay(5);

		cmd_id = CMD_ID_INIT_CAM;
		if (mcu_get_cmd_status(
			client, &cmd_id, &cmd_status, &retcode) < 0) {
			dev_err(&client->dev," %s(%d) Error \n",
			       __func__, __LINE__);
			return -EIO;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
		    ((retcode == ERRCODE_SUCCESS) || (retcode == ERRCODE_ALREADY))) {

#ifdef AR1335_DEBUG
			pr_info(" ISP Already Initialized !! \n");
#endif

			return 0;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
			    "(%s) %d Init Error STATUS = 0x%04x RET = 0x%02x\n",
			     __func__, __LINE__, cmd_status, retcode);
			return -EIO;
		}
	}

	return -ETIMEDOUT;
}

static int mcu_get_ctrl_ui(struct i2c_client *client,
			   ISP_CTRL_INFO * mcu_ui_info, int index)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	int ret = 0, i = 0, err = 0;

	/* lock semaphore */
	mutex_lock(&mcu_i2c_mutex_1335_af);

	/* First Txn Payload length = 0 */
	payload_len = 2;

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_GET_CTRL_UI_INFO;
	mc_data_1335_af[2] = payload_len >> 8;
	mc_data_1335_af[3] = payload_len & 0xFF;
	mc_data_1335_af[4] = errorcheck(&mc_data_1335_af[2], 2);

	ar1335_write(client, mc_data_1335_af, TX_LEN_PKT);

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_GET_CTRL_UI_INFO;
	mc_data_1335_af[2] = index >> 8;
	mc_data_1335_af[3] = index & 0xFF;
	mc_data_1335_af[4] = errorcheck(&mc_data_1335_af[2], 2);
	err = ar1335_write(client, mc_data_1335_af, 5);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = ar1335_read(client, mc_ret_data_1335_af, RX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data_1335_af[4];
	calc_crc = errorcheck(&mc_ret_data_1335_af[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	payload_len =
	    ((mc_ret_data_1335_af[2] << 8) | mc_ret_data_1335_af[3]) + HEADER_FOOTER_SIZE;
	errcode = mc_ret_data_1335_af[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
		       __func__, __LINE__, errcode);
		ret = -EINVAL;
		goto exit;
	}

	memset(mc_ret_data_1335_af, 0x00, payload_len);
	err = ar1335_read(client, mc_ret_data_1335_af, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data_1335_af[payload_len - 2];
	calc_crc =
	    errorcheck(&mc_ret_data_1335_af[2], payload_len - HEADER_FOOTER_SIZE);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data_1335_af[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
		       __func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	strncpy((char *)mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_name, &mc_ret_data_1335_af[2],MAX_CTRL_UI_STRING_LEN);

	mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_ui_type = mc_ret_data_1335_af[34];
	mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_ui_flags = mc_ret_data_1335_af[35] << 8 |
	    mc_ret_data_1335_af[36];

	if (mcu_ui_info->ctrl_ui_data.ctrl_ui_info.ctrl_ui_type == V4L2_CTRL_TYPE_MENU) {
		mcu_ui_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem = mc_ret_data_1335_af[37];

		mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu =
		    devm_kzalloc(&client->dev,((mcu_ui_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem +1) * sizeof(char *)), GFP_KERNEL);
		for (i = 0; i < mcu_ui_info->ctrl_ui_data.ctrl_menu_info.num_menu_elem; i++) {
			mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i] =
			    devm_kzalloc(&client->dev,MAX_CTRL_UI_STRING_LEN, GFP_KERNEL);
			strncpy((char *)mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i],
				&mc_ret_data_1335_af[38 +(i *MAX_CTRL_UI_STRING_LEN)], MAX_CTRL_UI_STRING_LEN);

#ifdef AR1335_DEBUG
			pr_info(" Menu Element %d : %s \n",
				     i, mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i]);
#endif

		}

		mcu_ui_info->ctrl_ui_data.ctrl_menu_info.menu[i] = NULL;
	}

 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex_1335_af);

	return ret;

}

static int mcu_stream_config(struct i2c_client *client, uint32_t format,
			     int mode, int frate_index)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0, index = 0xFFFF;
	uint8_t retcode = 0, cmd_id = 0;
	int loop = 0, ret = 0, err = 0, retry = 1000;
	static uint16_t prev_index = 0xFFFE;

	/* lock semaphore */
	mutex_lock(&mcu_i2c_mutex_1335_af);

	cmd_id = CMD_ID_STREAM_CONFIG;
	if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) < 0) {
		dev_err(&client->dev," %s(%d) Error \n", __func__, __LINE__);
		ret = -EIO;
		goto exit;
	}

	if ((cmd_status != MCU_CMD_STATUS_SUCCESS) ||
	    (retcode != ERRCODE_SUCCESS)) {
		dev_err(&client->dev,
			" ISP is Unintialized or Busy STATUS = 0x%04x Errcode = 0x%02x !! \n",
		     cmd_status, retcode);
		ret = -EBUSY;
		goto exit;
	}

	for (loop = 0;(&streamdb[loop]) != NULL; loop++) {
		if (streamdb[loop] == mode) {
			index = loop + frate_index;
			break;
		}
	}

#ifdef AR1335_DEBUG
	pr_info(" Index = 0x%04x , format = 0x%08x, width = %hu,"
		     " height = %hu, frate num = %hu \n", index, format,
		     ar1335_data.mcu_cam_frmfmt[mode].size.width,
		     ar1335_data.mcu_cam_frmfmt[mode].size.height,
		     ar1335_data.mcu_cam_frmfmt[mode].framerates[frate_index]);
#endif

	if (index == 0xFFFF) {
		ret = -EINVAL;
		goto exit;
	}

	if(prev_index == index) {
#ifdef AR1335_DEBUG
		pr_info("Skipping Previous mode set ... \n");
#endif
		ret = 0;
		goto exit;
	}

issue_cmd:
	/* First Txn Payload length = 0 */
	payload_len = 14;

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_STREAM_CONFIG;
	mc_data_1335_af[2] = payload_len >> 8;
	mc_data_1335_af[3] = payload_len & 0xFF;
	mc_data_1335_af[4] = errorcheck(&mc_data_1335_af[2], 2);

	ar1335_write(client, mc_data_1335_af, TX_LEN_PKT);

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_STREAM_CONFIG;
	mc_data_1335_af[2] = index >> 8;
	mc_data_1335_af[3] = index & 0xFF;

	/* Format Fourcc - currently only YUYV */
	mc_data_1335_af[4] = format >> 24;
	mc_data_1335_af[5] = format >> 16;
	mc_data_1335_af[6] = format >> 8;
	mc_data_1335_af[7] = format & 0xFF;

	/* width */
	mc_data_1335_af[8] = ar1335_data.mcu_cam_frmfmt[mode].size.width >> 8;
	mc_data_1335_af[9] = ar1335_data.mcu_cam_frmfmt[mode].size.width & 0xFF;

	/* height */
	mc_data_1335_af[10] = ar1335_data.mcu_cam_frmfmt[mode].size.height >> 8;
	mc_data_1335_af[11] = ar1335_data.mcu_cam_frmfmt[mode].size.height & 0xFF;

	/* frame rate num */
	mc_data_1335_af[12] = ar1335_data.mcu_cam_frmfmt[mode].framerates[frate_index] >> 8;
	mc_data_1335_af[13] = ar1335_data.mcu_cam_frmfmt[mode].framerates[frate_index] & 0xFF;

	/* frame rate denom */
	mc_data_1335_af[14] = 0x00;
	mc_data_1335_af[15] = 0x01;

	mc_data_1335_af[16] = errorcheck(&mc_data_1335_af[2], 14);
	err = ar1335_write(client, mc_data_1335_af, 17);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	while (--retry > 0) {
		cmd_id = CMD_ID_STREAM_CONFIG;
		if (mcu_get_cmd_status(
			client, &cmd_id, &cmd_status, &retcode) < 0) {
			dev_err(&client->dev,
				" %s(%d) MCU GET CMD Status Error : loop : %d \n",
				__func__, __LINE__, loop);
			ret = -EIO;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
		    (retcode == ERRCODE_SUCCESS)) {
			ret = 0;
			goto exit;
		}

		if(retcode == ERRCODE_AGAIN) {
			/* Issue Command Again if Set */
			retry = 1000;
			goto issue_cmd;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
				"(%s) %d Error STATUS = 0x%04x RET = 0x%02x\n",
				__func__, __LINE__, cmd_status, retcode);
			ret = -EIO;
			goto exit;
		}

		/* Delay after retry */
		mdelay(10);
	}

	dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
			__LINE__, err);
	ret = -ETIMEDOUT;

exit:
	if(!ret)
		prev_index = index;

	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex_1335_af);

	return ret;
}

static int mcu_isp_power_down(struct i2c_client *client)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 1000, err = 0;

	/*lock semaphore */
	mutex_lock(&mcu_i2c_mutex_1335_af);

	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_ISP_PDOWN;
	mc_data_1335_af[2] = payload_len >> 8;
	mc_data_1335_af[3] = payload_len & 0xFF;
	mc_data_1335_af[4] = errorcheck(&mc_data_1335_af[2], 2);

	ar1335_write(client, mc_data_1335_af, TX_LEN_PKT);

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_ISP_PDOWN;
	err = ar1335_write(client, mc_data_1335_af, 2);
	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d \n",
			__func__, __LINE__, err);
		goto exit;
	}

	while (--retry > 0) {
		msleep(20);
		cmd_id = CMD_ID_ISP_PDOWN;
		if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
		    0) {
			dev_err(&client->dev, " %s(%d) Get Status Error \n",
				__func__, __LINE__);
			err = -EINVAL;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_ISP_PWDN) &&
		    ((retcode == ERRCODE_SUCCESS) || retcode == ERRCODE_ALREADY)) {
			err = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
				"(%s) %d Error STATUS = 0x%04x RET = 0x%02x\n",
				__func__, __LINE__, cmd_status,
				retcode);
			err = -EIO;
			goto exit;
		}

	}
	err = -ETIMEDOUT;
 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex_1335_af);
	return err;
}

static int mcu_isp_power_wakeup(struct i2c_client *client)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0;
	uint8_t retcode = 0, cmd_id = 0;
	int retry = 1000, err = 0;

	/*lock semaphore */
	mutex_lock(&mcu_i2c_mutex_1335_af);
	/* First Txn Payload length = 0 */
	payload_len = 0;

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_ISP_PUP;
	mc_data_1335_af[2] = payload_len >> 8;
	mc_data_1335_af[3] = payload_len & 0xFF;
	mc_data_1335_af[4] = errorcheck(&mc_data_1335_af[2], 2);

	ar1335_write(client, mc_data_1335_af, TX_LEN_PKT);

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_ISP_PUP;
	err = ar1335_write(client, mc_data_1335_af, 2);
	if (err != 0) {
		dev_err(&client->dev, " %s(%d) Error - %d \n",
			__func__, __LINE__, err);
		goto exit;
	}

	while (--retry > 0) {
		msleep(20);
		cmd_id = CMD_ID_ISP_PUP;
		if (mcu_get_cmd_status(client, &cmd_id, &cmd_status, &retcode) <
		    0) {
			dev_err(&client->dev, " %s(%d) Error \n",
				__func__, __LINE__);
			err = -EIO;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
		    ((retcode == ERRCODE_SUCCESS) || retcode == ERRCODE_ALREADY)) {
			err = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
				"(%s) %d Error STATUS = 0x%04x RET = 0x%02x\n",
				__func__, __LINE__, cmd_status, retcode);
			err = -EIO;
			goto exit;
		}

	}

	err = -ETIMEDOUT;
 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex_1335_af);
	return err;
}

static int mcu_set_ctrl(struct i2c_client *client, uint32_t arg_ctrl_id,
			uint8_t ctrl_type, int32_t curr_val)
{
	uint32_t payload_len = 0;

	uint16_t cmd_status = 0, index = 0xFFFF;
	uint8_t retcode = 0, cmd_id = 0;
	int loop = 0, ret = 0, err = 0;
	uint32_t ctrl_id = 0;

	/* lock semaphore */
	mutex_lock(&mcu_i2c_mutex_1335_af);

	ctrl_id = arg_ctrl_id;

	/* call ISP Ctrl config command */

	for (loop = 0; loop < num_ctrls; loop++) {
		if (ctrldb[loop] == ctrl_id) {
			index = mcu_ctrl_info[loop].mcu_ctrl_index;
			break;
		}
	}

	if (index == 0xFFFF) {
		ret = -EINVAL;
		goto exit;
	}

	/* First Txn Payload length = 0 */
	payload_len = 11;

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_SET_CTRL;
	mc_data_1335_af[2] = payload_len >> 8;
	mc_data_1335_af[3] = payload_len & 0xFF;
	mc_data_1335_af[4] = errorcheck(&mc_data_1335_af[2], 2);

	ar1335_write(client, mc_data_1335_af, TX_LEN_PKT);

	/* Second Txn */
	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_SET_CTRL;

	/* Index */
	mc_data_1335_af[2] = index >> 8;
	mc_data_1335_af[3] = index & 0xFF;

	/* Control ID */
	mc_data_1335_af[4] = ctrl_id >> 24;
	mc_data_1335_af[5] = ctrl_id >> 16;
	mc_data_1335_af[6] = ctrl_id >> 8;
	mc_data_1335_af[7] = ctrl_id & 0xFF;

	/* Ctrl Type */
	mc_data_1335_af[8] = ctrl_type;

	/* Ctrl Value */
	mc_data_1335_af[9] = curr_val >> 24;
	mc_data_1335_af[10] = curr_val >> 16;
	mc_data_1335_af[11] = curr_val >> 8;
	mc_data_1335_af[12] = curr_val & 0xFF;

	/* CRC */
	mc_data_1335_af[13] = errorcheck(&mc_data_1335_af[2], 11);

	err = ar1335_write(client, mc_data_1335_af, 14);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	while (1) {
		cmd_id = CMD_ID_SET_CTRL;
		if (mcu_get_cmd_status(
			client, &cmd_id, &cmd_status, &retcode) < 0) {
			dev_err(&client->dev," %s(%d) Error \n",
			       __func__, __LINE__);
			ret = -EINVAL;
			goto exit;
		}

		if ((cmd_status == MCU_CMD_STATUS_SUCCESS) &&
		    (retcode == ERRCODE_SUCCESS)) {
			ret = 0;
			goto exit;
		}

		if ((retcode != ERRCODE_BUSY) &&
		    ((cmd_status != MCU_CMD_STATUS_PENDING))) {
			dev_err(&client->dev,
				"(%s) %d ISP Error STATUS = 0x%04x RET = 0x%02x\n",
			     __func__, __LINE__, cmd_status, retcode);
			ret = -EIO;
			goto exit;
		}
	}

 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex_1335_af);

	return ret;
}

static int mcu_get_ctrl(struct i2c_client *client, uint32_t arg_ctrl_id,
			uint8_t * ctrl_type, int32_t * curr_val)
{
	uint32_t payload_len = 0;
	uint8_t errcode = ERRCODE_SUCCESS, orig_crc = 0, calc_crc = 0;
	uint16_t index = 0xFFFF;
	int loop = 0, ret = 0, err = 0;

	uint32_t ctrl_id = 0;

	/* lock semaphore */
	mutex_lock(&mcu_i2c_mutex_1335_af);

	ctrl_id = arg_ctrl_id;

	/* Read the Ctrl Value from Micro controller */

	for (loop = 0; loop < num_ctrls; loop++) {
		if (ctrldb[loop] == ctrl_id) {
			index = mcu_ctrl_info[loop].mcu_ctrl_index;
			break;
		}
	}

	if (index == 0xFFFF) {
		ret = -EINVAL;
		goto exit;
	}

	if (
		mcu_ctrl_info[loop].ctrl_ui_data.ctrl_ui_info.ctrl_ui_flags &
		V4L2_CTRL_FLAG_WRITE_ONLY
	) {
		ret = -EACCES;
		goto exit;
	}

	/* First Txn Payload length = 2 */
	payload_len = 2;

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_GET_CTRL;
	mc_data_1335_af[2] = payload_len >> 8;
	mc_data_1335_af[3] = payload_len & 0xFF;
	mc_data_1335_af[4] = errorcheck(&mc_data_1335_af[2], 2);

	ar1335_write(client, mc_data_1335_af, TX_LEN_PKT);

	mc_data_1335_af[0] = CMD_SIGNATURE;
	mc_data_1335_af[1] = CMD_ID_GET_CTRL;
	mc_data_1335_af[2] = index >> 8;
	mc_data_1335_af[3] = index & 0xFF;
	mc_data_1335_af[4] = errorcheck(&mc_data_1335_af[2], 2);
	err = ar1335_write(client, mc_data_1335_af, 5);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	err = ar1335_read(client, mc_ret_data_1335_af, RX_LEN_PKT);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data_1335_af[4];
	calc_crc = errorcheck(&mc_ret_data_1335_af[2], 2);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		ret = -1;
		goto exit;
	}

	if (((mc_ret_data_1335_af[2] << 8) | mc_ret_data_1335_af[3]) == 0) {
		ret = -EIO;
		goto exit;
	}

	errcode = mc_ret_data_1335_af[5];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
		       __func__, __LINE__, errcode);
		ret = -EIO;
		goto exit;
	}

	payload_len =
	    ((mc_ret_data_1335_af[2] << 8) | mc_ret_data_1335_af[3]) + HEADER_FOOTER_SIZE;
	memset(mc_ret_data_1335_af, 0x00, payload_len);
	err = ar1335_read(client, mc_ret_data_1335_af, payload_len);
	if (err != 0) {
		dev_err(&client->dev," %s(%d) Error - %d \n", __func__,
		       __LINE__, err);
		ret = -EIO;
		goto exit;
	}

	/* Verify CRC */
	orig_crc = mc_ret_data_1335_af[payload_len - 2];
	calc_crc =
	    errorcheck(&mc_ret_data_1335_af[2], payload_len - HEADER_FOOTER_SIZE);
	if (orig_crc != calc_crc) {
		dev_err(&client->dev," %s(%d) CRC 0x%02x != 0x%02x \n",
		       __func__, __LINE__, orig_crc, calc_crc);
		ret = -EINVAL;
		goto exit;
	}

	/* Verify Errcode */
	errcode = mc_ret_data_1335_af[payload_len - 1];
	if (errcode != ERRCODE_SUCCESS) {
		dev_err(&client->dev," %s(%d) Errcode - 0x%02x \n",
		       __func__, __LINE__, errcode);
		ret = -EINVAL;
		goto exit;
	}

	/* Ctrl type starts from index 6 */

	*ctrl_type = mc_ret_data_1335_af[6];

	switch (*ctrl_type) {
	case CTRL_STANDARD:
		*curr_val =
		    mc_ret_data_1335_af[7] << 24 | mc_ret_data_1335_af[8] << 16 | mc_ret_data_1335_af[9]
		    << 8 | mc_ret_data_1335_af[10];
		break;

	case CTRL_EXTENDED:
		/* Not Implemented */
		break;
	}

 exit:
	/* unlock semaphore */
	mutex_unlock(&mcu_i2c_mutex_1335_af);

	return ret;
}

/*
 * ---------------------------------------------------------
 *  END of MCU realed functions
 * ---------------------------------------------------------
 */

/*
 * Macro to retry a specific sequence (a function call) for a specific
 * number of times to check if the sequence succeeds within the specified
 * number of tries.
 *
 * Arguments:
 * retries - number of retries for the sequence
 * sequence - the sequence to retry (typically a function call).
 *            It is assumed that the sequence returns a negative value in
 *            case of an error.
 *
 * Evaluation value:
 * - The macro evaluates to zero if the sequence succeeds within the specified
 *   number of tries.
 * - The macro evaluates to a negative value when the sequence does not succeed
 *   within the specified number of tries.
 *
 */
#define RETRY_SEQUENCE(retries, sequence)		\
({							\
	int status = 0, err, retry;			\
	for (retry = 0; retry < retries; retry++) {	\
		err = sequence;				\
							\
		if (err < 0)				\
			msleep(5);			\
		else					\
			break;				\
	}						\
	if (retry == retries) {				\
		status = err;				\
	}						\
	status;						\
})

static void toggle_gpio(unsigned int gpio, int val)
{
	if (gpio_cansleep(gpio)){
		gpio_direction_output(gpio,val);
		gpio_set_value_cansleep(gpio, val);
	} else{
		gpio_direction_output(gpio,val);
		gpio_set_value(gpio, val);
	}
}

static int ar1335_querymenu(struct v4l2_subdev *sd, struct v4l2_querymenu *qm)
{
	uint32_t index = 0;
	int loop;

	if (sd == NULL || qm == NULL)
		return -EINVAL;

	for (loop = 0; loop < num_ctrls; loop++) {
		if (ctrldb[loop] == qm->id) {
			index = loop;
			break;
		}
	}

	if (loop == num_ctrls) {
		return -EINVAL;
	}

	if (
		!(
			0 <= qm->index &&
			qm->index < mcu_ctrl_info[index].ctrl_ui_data.ctrl_menu_info.num_menu_elem
		 )
	) {
		return -EINVAL;
	}

	/*
	 * Copy the name of the menu.
	 *
	 * We deal only with V4L2_CTRL_TYPE_MENU and not
	 * V4L2_CTRL_TYPE_INTEGER_MENU. So, this should be
	 * enough.
	 */
	strcpy(qm->name, mcu_ctrl_info[index].ctrl_ui_data.ctrl_menu_info.menu[qm->index]);

	/*
	 * Set the reserved to zero as mentioned in spec
	 */
	qm->reserved = 0;

	return 0;
}

static int ar1335_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int index, ctrl_index = -1, ctrl_id;
	bool next_ctrl = (qc->id & V4L2_CTRL_FLAG_NEXT_CTRL);

	if (sd == NULL || qc == NULL)
		return -EINVAL;

	if (next_ctrl) {
		ctrl_id = qc->id & (~V4L2_CTRL_FLAG_NEXT_CTRL);

		/*
		 * Ignore the V4L2_CTRL_FLAG_NEXT_COMPOUND for now
		 */
		ctrl_id = ctrl_id & (~V4L2_CTRL_FLAG_NEXT_COMPOUND);
	}
	else {
		/*
		 * Assume we've just got the control ID itself
		 * directly.
		 */
		ctrl_id = qc->id;
	}

	if (ctrl_id) {
		for (index = 0; index < num_ctrls; index++) {
			if (ctrldb[index] == ctrl_id) {
				ctrl_index = (next_ctrl) ? index + 1 : index;
				break;
			}
		}

		if (index == num_ctrls) {
			/*
			 * We do not know about this control
			 */
			return -EINVAL;
		}
		else if (
			next_ctrl &&
			index == num_ctrls - 1
		)
		{
			/*
			 * We've got a request for the control
			 * after the last one.
			 */
			return -EINVAL;
		}
	}
	else if (next_ctrl) {
		ctrl_index = 0;
	}
	else {
		return -EINVAL;
	}

	if (
		mcu_ctrl_info[ctrl_index].ctrl_type == CTRL_STANDARD
	) {
		/*
		 * We cannot use `v4l2_ctrl_query_fill` instead of manually filling
		 * the details even for standard controls as we sometimes implement our
		 * own version for some controls.
		 *
		 * e.g., V4L2_CID_FOCUS_AUTO has a max value of 1 according to standard
		 * but our version of it has a max value of 5.
		 */
		qc->id = mcu_ctrl_info[ctrl_index].ctrl_id;

		strcpy(qc->name, mcu_ctrl_info[ctrl_index].ctrl_ui_data.ctrl_ui_info.ctrl_name);

		qc->type = mcu_ctrl_info[ctrl_index].ctrl_ui_data.ctrl_ui_info.ctrl_ui_type;
		qc->flags = mcu_ctrl_info[ctrl_index].ctrl_ui_data.ctrl_ui_info.ctrl_ui_flags;

		qc->minimum = mcu_ctrl_info[ctrl_index].ctrl_data.std.ctrl_min;
		qc->maximum = mcu_ctrl_info[ctrl_index].ctrl_data.std.ctrl_max;
		qc->step = mcu_ctrl_info[ctrl_index].ctrl_data.std.ctrl_step;
		qc->default_value = mcu_ctrl_info[ctrl_index].ctrl_data.std.ctrl_def;
	}
	else {
		return -EINVAL;
	}

	return 0;
}

static int ar1335_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = ar1335_data.i2c_client;
	int err = 0;
	uint8_t ctrl_type = 0;
	int ctrl_val = 0;

	if (sd == NULL || ctrl == NULL)
		return -EINVAL;

	if ((err = mcu_get_ctrl(client, ctrl->id, &ctrl_type, &ctrl_val)) < 0) {
		return err;
	}

	if (ctrl_type == CTRL_STANDARD) {
		ctrl->value = ctrl_val;
	} else {
		/* Not Implemented */
		return -EINVAL;
	}

	return err;
}

static int ar1335_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = ar1335_data.i2c_client;
	int err = 0, index, ctrl_index = 0;

	if (sd == NULL || ctrl == NULL)
		return -EINVAL;

	for (index = 0; index < num_ctrls; index++) {
		if (ctrldb[index] == ctrl->id) {
			ctrl_index = index;
			break;
		}
	}

	if (index == num_ctrls) {
		return -EINVAL;
	}

	if (
		ctrl->value < mcu_ctrl_info[ctrl_index].ctrl_data.std.ctrl_min ||
		ctrl->value > mcu_ctrl_info[ctrl_index].ctrl_data.std.ctrl_max
	)
		return -ERANGE;

	if ((err =
	     mcu_set_ctrl(client, ctrl->id, CTRL_STANDARD, ctrl->value)) < 0) {
		dev_err(&client->dev," %s (%d ) \n", __func__, __LINE__);
		return -EINVAL;
	}

	return err;
}

static int ar1335_g_ext_ctrls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls)
{
	int i, err = 0;

	if (sd == NULL || ctrls == NULL)
		return -EINVAL;

	for (i = 0; i < ctrls->count; i++) {
		struct v4l2_ext_control *ext_ctrl = ctrls->controls + i;
		struct v4l2_control ctrl = {
			.id = ext_ctrl->id,
		};

		err = ar1335_g_ctrl(sd, &ctrl);
		if (err) {
			ctrls->error_idx = ctrls->count;
			break;
		}
		else {
			ext_ctrl->value = ctrl.value;
		}
	}

	return err;
}

static int ar1335_try_ext_ctrls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls)
{
	int i;

	if (sd == NULL || ctrls == NULL)
		return -EINVAL;

	for (i = 0; i < ctrls->count; i++) {
		struct v4l2_ext_control *ext_ctrl = ctrls->controls + i;
		int ctrl_index = 0, index;

		for (index = 0; index < num_ctrls; index++) {
			if (ctrldb[index] == ext_ctrl->id) {
				ctrl_index = index;
				break;
			}
		}

		if (index == num_ctrls) {
			ctrls->error_idx = ext_ctrl->id;
			return -EINVAL;
		}

		if (
			ext_ctrl->value < mcu_ctrl_info[ctrl_index].ctrl_data.std.ctrl_min ||
			ext_ctrl->value > mcu_ctrl_info[ctrl_index].ctrl_data.std.ctrl_max
		) {
			ctrls->error_idx = ext_ctrl->id;
			return -ERANGE;
		}
	}

	return 0;

}

static int ar1335_s_ext_ctrls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls)
{
	int i, err = 0;

	if (sd == NULL || ctrls == NULL)
		return -EINVAL;

	for (i = 0; i < ctrls->count; i++) {
		struct v4l2_ext_control *ext_ctrl = ctrls->controls + i;
		struct v4l2_control ctrl = {
			.id = ext_ctrl->id,
			.value = ext_ctrl->value
		};

		err = ar1335_s_ctrl(sd, &ctrl);
		if (err) {
			/*
			 * TODO: We would have to indicate whether there
			 * is an issue in validation or in the
			 * hardware by correctly setting the error_idx
			 * to count only when the validation failed
			 * and setting it to index when there is an
			 * issue in communication with the hardware.
			 *
			 * For now, just return the count for all cases.
			 */
			ctrls->error_idx = ctrls->count;
			break;
		}
	}

	return err;
}

static int ar1335_enum_frameintervals(struct v4l2_subdev *sd,struct v4l2_subdev_state *state,struct v4l2_subdev_frame_interval_enum *fival)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int j;

	if (fival->width == 0 || fival->height == 0)
	{
		dev_err(&client->dev, "Please assign width and height.\n");
		return -EINVAL;
	}

	if (fival->code != ar1335_data.fmt.code)
	{
		return -EINVAL;
	}

	for (j = 0; j < ar1335_data.num_frm_fmts; j++) {
		if (
			fival->width == ar1335_data.mcu_cam_frmfmt[j].size.width &&
			fival->height == ar1335_data.mcu_cam_frmfmt[j].size.height
		) {
			if (fival->index >= ar1335_data.mcu_cam_frmfmt[j].num_framerates)
			{
				return -EINVAL;
			}

			fival->interval.numerator = 1;
			fival->interval.denominator = ar1335_data.mcu_cam_frmfmt[j].framerates[fival->index];

			return 0;
		}
	}
	return -EINVAL;

}

static int ar1335_enum_framesizes(struct v4l2_subdev *sd,struct v4l2_subdev_state *state,struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ar1335_data.num_frm_fmts)
	{
		return -EINVAL;
	}

	if (fse->code != ar1335_data.fmt.code)
	{
		return -EINVAL;
	}

	fse->max_width = ar1335_data.mcu_cam_frmfmt[fse->index].size.width;
	fse->min_width = fse->max_width;

	fse->max_height = ar1335_data.mcu_cam_frmfmt[fse->index].size.height;
	fse->min_height = fse->max_height;

	return 0;
}

static int ar1335_enum_mbus_code(struct v4l2_subdev *sd,struct v4l2_subdev_state *state,struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= AR1335_MAX_FORMAT_SUPPORTED)
		return -EINVAL;

	code->code = ar1335_data.fmt.code;

	return 0;
}

static int ar1335_s_stream(struct v4l2_subdev *sd, int enable)
{
	if (!enable) {
		/* Perform Stream Off Sequence - if any */
	}

	/* Perform Stream On Sequence - if any  */
	mdelay(10);

	return 0;
}

static int ar1335_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	int mode = ar1335_data.streamcap.capturemode;

	param->parm.capture.capability |= V4L2_CAP_TIMEPERFRAME;

	param->parm.capture.timeperframe.denominator =
	    ar1335_data.mcu_cam_frmfmt[mode].framerates[ar1335_data.frate_index];
	param->parm.capture.timeperframe.numerator = 1;

	return 0;
}

static int ar1335_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0, err = 0;
	int mode = ar1335_data.streamcap.capturemode;
	int fourcc = ar1335_data.pix.pixelformat;

	param->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	memset(param->parm.capture.reserved, 0, 4*sizeof(u32));

	if (
		param->parm.capture.timeperframe.denominator == 0 &&
		param->parm.capture.timeperframe.numerator == 0 &&
		ar1335_data.mcu_cam_frmfmt[mode].num_framerates == 1
	) {
		param->parm.capture.timeperframe.denominator =
			ar1335_data.mcu_cam_frmfmt[mode].framerates[ar1335_data.frate_index];
		param->parm.capture.timeperframe.numerator = 1;
		/*
		 * We would have to reset the frame interval to a
		 * nominal value in this case but as we just have one
		 * frame interval we just return success.
		 */
		return 0;
	}

	if (param->parm.capture.timeperframe.numerator != 1) {
		dev_err(&client->dev, "Invalid numerator for timeperframe\n");
		return -EINVAL;
	}

	for (ret = 0; ret < ar1335_data.mcu_cam_frmfmt[mode].num_framerates;
	     ret++) {
		if ((ar1335_data.mcu_cam_frmfmt[mode].framerates[ret] ==
		     param->parm.capture.timeperframe.denominator)) {
			int retries = retries_for_i2c_commands * 2;
			ar1335_data.frate_index = ret;

			/* call stream config with width, height, frame rate */
			err = RETRY_SEQUENCE(
				retries,
				mcu_stream_config(client, fourcc, mode,	ar1335_data.frate_index)
			);
			if (err < 0) {
				dev_err(&client->dev, "%s: Failed stream_config \n", __func__);
				return err;
			}

			mdelay(10);

			return 0;
		}
	}

	return -EINVAL;
}

static int ar1335_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0,
	    retries = retries_for_i2c_commands * 2;

	if (on) {
		if (ar1335_data.power_on == 0) {
			/*
			 * We power wakeup the ISP only for the first time.
			 */
			/* Perform power wakeup sequence */
			err = RETRY_SEQUENCE(
				retries,
				mcu_isp_power_wakeup(client)
			);
			if (err < 0)
			{
				dev_err(&client->dev, "%s: Failed Power_wakeup\n", __func__);
				return err;
			}

			mdelay(80);

#ifdef AR1335_DEBUG
			pr_info("Sensor Hardware Power Up Sequence\n");
#endif
		}

		ar1335_data.power_on++;
	}
	else {
		if (ar1335_data.power_on == 1) {
			/* Perform power down Sequence */
			err = RETRY_SEQUENCE(
				retries,
				mcu_isp_power_down(client)
			);
			if (err < 0)
			{
				dev_err(&client->dev, "%s: Failed power_down\n", __func__);
				return err;
			}

#ifdef AR1335_DEBUG
			pr_info("Sensor Hardware Power Down Sequence\n");
#endif
		}

		ar1335_data.power_on--;
	}

	return 0;
}

static int ar1335_get_fmt(struct v4l2_subdev *sd,struct v4l2_subdev_state *state,struct v4l2_subdev_format *format)
{
	int ret = 0;

	if (format->pad)
		return -EINVAL;

	format->format.code = ar1335_data.fmt.code;
	format->format.colorspace = ar1335_data.fmt.colorspace;
	format->format.field = V4L2_FIELD_NONE;

	format->format.width	= ar1335_data.pix.width;
	format->format.height	= ar1335_data.pix.height;

	return ret;
}

static int ar1335_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *format)
{
	int i;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int flag = 0, err = 0;
	int retries = retries_for_i2c_commands * 2;

	format->format.code = ar1335_data.fmt.code;
	format->format.colorspace = ar1335_data.fmt.colorspace;
	format->format.field = V4L2_FIELD_NONE;

	for (i = 0; i < ar1335_data.num_frm_fmts ; i++) {
		if (
			ar1335_data.mcu_cam_frmfmt[i].size.width == format->format.width &&
			ar1335_data.mcu_cam_frmfmt[i].size.height == format->format.height
		) {
			flag = 1;
			break;
		}
	}

	if(flag == 0) {
		format->format.width	= ar1335_data.pix.width;
		format->format.height	= ar1335_data.pix.height;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
	{
		return 0;
	}

	/* call stream config with width, height, frame rate */
	err = RETRY_SEQUENCE(
		retries,
		mcu_stream_config(
			client,
			ar1335_data.pix.pixelformat,
			ar1335_data.mcu_cam_frmfmt[i].mode,
			ar1335_data.frate_index
		)
	);
	if (err < 0)
	{
		dev_err(&client->dev, "%s: Failed stream_config \n", __func__);
		return err;
	}

	ar1335_data.pix.width = format->format.width;
	ar1335_data.pix.height = format->format.height;
	ar1335_data.streamcap.capturemode = ar1335_data.mcu_cam_frmfmt[i].mode;

	mdelay(10);

	return 0;
}

static int ar1335_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	return 0;
}

static struct v4l2_subdev_video_ops ar1335_subdev_video_ops = {
	.g_parm = ar1335_g_parm,
	.s_parm = ar1335_s_parm,
	.s_stream = ar1335_s_stream,
};

static const struct v4l2_subdev_pad_ops ar1335_subdev_pad_ops = {
	.enum_frame_size       = ar1335_enum_framesizes,
	.enum_frame_interval   = ar1335_enum_frameintervals,
	.enum_mbus_code        = ar1335_enum_mbus_code,
	.set_fmt               = ar1335_set_fmt,
	.get_fmt               = ar1335_get_fmt,
};

static struct v4l2_subdev_core_ops ar1335_subdev_core_ops = {
	.s_power	= ar1335_s_power,
	.queryctrl = ar1335_queryctrl,
	.g_ctrl = ar1335_g_ctrl,
	.s_ctrl = ar1335_s_ctrl,
	.g_ext_ctrls = ar1335_g_ext_ctrls,
	.s_ext_ctrls = ar1335_s_ext_ctrls,
	.try_ext_ctrls = ar1335_try_ext_ctrls,
	.querymenu = ar1335_querymenu,
};

static struct v4l2_subdev_ops ar1335_subdev_ops = {
	.core	= &ar1335_subdev_core_ops,
	.video	= &ar1335_subdev_video_ops,
	.pad	= &ar1335_subdev_pad_ops,
};

static const struct media_entity_operations ar1335_sd_media_ops = {
	.link_setup = ar1335_link_setup,
};

static int ar1335_init(struct i2c_client *client)
{
	u32 tgt_xclk;	/* target xclk */
	int ret = 0;

	ar1335_data.on = true;

	/* mclk */
	tgt_xclk = ar1335_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)AR1335_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)AR1335_XCLK_MIN);
	ar1335_data.mclk = tgt_xclk;

#ifdef AR1335_DEBUG
	pr_info("mclk: %d MHz\n", tgt_xclk / 1000000);
#endif

	ret = mcu_stream_config(client, ar1335_data.pix.pixelformat,
			ar1335_data.streamcap.capturemode,
			ar1335_data.frate_index);

	return ret;
}

static int ar1335_ctrls_init(ISP_CTRL_INFO *mcu_cam_ctrls)
{
	struct i2c_client *client = NULL;
	int numctrls = 0;
	int err = 0, i = 0;

	client = ar1335_data.i2c_client;

	if (mcu_cam_ctrls == NULL)
	{
		dev_err(
			&client->dev,
			"%s: MCU control data hasn't been allocated\n",
			__func__
		);
		return -EINVAL;
	}
	/*
	 * Enumerate the controls from the MCU
	 */
	err = mcu_count_or_list_ctrls(client, mcu_cam_ctrls, &numctrls);
	if (err < 0) {
		dev_err(&client->dev, "Unable to enumerate the controls in the sensor\n");
		return err;
	}

	for (i = 0; i < numctrls; i++) {
		if (mcu_cam_ctrls[i].ctrl_type == CTRL_STANDARD) {
			err = mcu_get_ctrl_ui(client, &mcu_ctrl_info[i], mcu_ctrl_info[i].mcu_ctrl_index);
			if (err != ERRCODE_SUCCESS) {
				dev_err(&client->dev, "Error Enumerating Control 0x%08x !! \n",
					mcu_ctrl_info[i].ctrl_id);
				return err;
			}
			else if (
				mcu_ctrl_info[i].ctrl_ui_data.ctrl_ui_info.ctrl_ui_type ==
				V4L2_CTRL_TYPE_MENU
			) {
				mcu_ctrl_info[i].ctrl_data.std.ctrl_step = 1;
			}
		}
	}

	return 0;
}

static int ar1335_verify_mcu(struct i2c_client *client)
{
	int ret = 0;
	unsigned char fw_version_1335_af[32] = {0};

	if (client == NULL)
	{
		dev_err(&client->dev, "%s: Invalid I2C client parameter\n", __func__);
		return -EINVAL;
	}

	toggle_gpio(pwdn_gpio, 0);
	msleep(1);
	toggle_gpio(reset_gpio, 0);
	msleep(1);
	toggle_gpio(reset_gpio, 1);
	msleep(10);

	ret = mcu_get_fw_version(client, fw_version_1335_af);

	if (ret == 0)
	{
		ret = mcu_verify_fw_version(fw_version_1335_af);
	}
	else
	{
		dev_dbg(
			&client->dev,
			"Could not read the firmware version from the MCU\n"
		);
	}

	/*
	 * Try booting and flashing in bootloader mode when an error is detected
	 * or the force update bit is set in the firmware version
	 */
	if (ret != 0) {
		int loop = 0;

		/*
		 * Verification of the MCU in firmware mode failed so
		 * try to boot the MCU in bootloader mode.
		 */

#ifdef AR1335_DEBUG
		pr_info(" Trying to Detect Bootloader mode\n");
#endif

		toggle_gpio(reset_gpio, 0);
		msleep(1);
		toggle_gpio(pwdn_gpio, 1);
		msleep(1);
		toggle_gpio(reset_gpio, 1);
		msleep(1);

		for(loop = 0; loop < 10; loop++) {
			ret = mcu_bload_get_version(client);
			if (ret < 0) {
				msleep(1);
				continue;
			} else {
#ifdef AR1335_DEBUG
				pr_info(" Get Bload Version Success\n");
#endif
				break;
			}
		}

		if(loop == 10) {
			dev_err(&client->dev, "Error getting firmware version in bootloader mode\n");
			return -EFAULT;
		}

		pr_info("Updating firmware. Please wait ...\n");

		if (mcu_fw_update(client, NULL) < 0) {
			dev_err(&client->dev, "Error when trying to update the firmware\n");
			return -EFAULT;
		}

		toggle_gpio(pwdn_gpio, 0);

		/* Allow FW Updated MCU to reboot */
		msleep(10);

		/*
		 * Ensure the firmware has been flashed correctly by getting the version
		 * of the firmware (in firmware mode).
		 */
		for(loop = 0; loop < 100; loop++) {
			ret = mcu_get_fw_version(client, fw_version_1335_af);

			if (ret == 0)
			{
				ret = mcu_verify_fw_version(fw_version_1335_af);
			}

			if (ret < 0) {
				msleep(1);
				continue;
			} else {
#ifdef AR1335_DEBUG
				pr_info(" Get FW Version Success\n");
#endif
				break;
			}
		}

		if(loop == 100) {
			dev_err(
				&client->dev,
				"Couldn't get firmware version correctly after update (did the update fail?)\n"
			);
			return ret;
		}
		else {
			pr_info("Firmware has been updated successfully.\n");
		}
	}

	dev_info(&client->dev, "Current Firmware Version - (%.32s)\n", fw_version_1335_af);

	return ret;
}

static int ar1335_parse_and_get_clocks(struct device *dev)
{
	int retval = 0;

	if (dev == NULL)
	{
		dev_err(dev, "%s: Invalid device parameter\n", __func__);
		return -EINVAL;
	}

	if (ar1335_data.soc_format_check){
		ar1335_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
	}
	else{
		ar1335_data.sensor_clk = devm_clk_get(dev, "xclk");
	}
	if (IS_ERR(ar1335_data.sensor_clk)) {
		/* assuming clock enabled by default */
		ar1335_data.sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(ar1335_data.sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk",
					&(ar1335_data.mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
					(u32 *) &(ar1335_data.mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(ar1335_data.csi));
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return retval;
	}

	return 0;
}

static int ar1335_parse_and_get_gpios(struct device *dev)
{
	int err;
	struct device_node *node = NULL;

	if (dev == NULL)
	{
		dev_err(dev, "%s: Invalid device parameter\n", __func__);
		return -EINVAL;
	}

	node = dev->of_node;

	pwdn_gpio = of_get_named_gpio(node, "pwn-gpios", 0);
	if (!gpio_is_valid(pwdn_gpio)) {
		dev_err(dev, "no sensor pwdn pin available");
		return -EINVAL;
	}
	else {
#ifdef AR1335_DEBUG
		printk("BOOT = %x \n", pwdn_gpio);
#endif
	}

	reset_gpio = of_get_named_gpio(node, "rst-gpios", 0);
	if (!gpio_is_valid(reset_gpio)) {
		dev_err(dev, "no sensor reset pin available");
		return -EINVAL;
	}
	else {
#ifdef AR1335_DEBUG
		printk("RESET = %x \n", reset_gpio);
#endif
	}

	err = devm_gpio_request_one(dev, pwdn_gpio, GPIOF_OUT_INIT_HIGH,
					"ar1335_mipi_pwdn");
	if (err < 0) {
		dev_warn(dev, "Failed to set power pin\n");
		dev_warn(dev, "err = %d\n", err);
		return err;
	}

	err = devm_gpio_request_one(dev, reset_gpio, GPIOF_OUT_INIT_HIGH,
					"ar1335_mipi_reset");
	if (err < 0) {
		dev_warn(dev, "Failed to set reset pin\n");
		dev_warn(dev, "err = %d\n", err);
		return err;
	}

	return 0;
}

/*!
 * ar1335 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ar1335_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct pinctrl *pinctrl;
	struct device_node *node = client->dev.of_node;
	struct device *dev = &client->dev;

	int ret, frm_fmt_size = 0, i;
	uint16_t sensor_id = 0;

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl))
		dev_warn(dev, "no pin available\n");

	/* Set initial values for the sensor struct. */
	memset(&ar1335_data, 0, sizeof(ar1335_data));

	ret = ar1335_parse_and_get_gpios(dev);
	if (ret)
	{
		pr_info("Warning: couldn't get GPIOs\n");
	}

	ar1335_data.soc_format_check = of_property_read_bool(dev->of_node, "cam-format-yuyv");
	ret = ar1335_parse_and_get_clocks(dev);
	if (ret)
	{
		dev_err(dev, "Error occurred when getting clock\n");
		return ret;
	}

	clk_prepare_enable(ar1335_data.sensor_clk);

	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		ar1335_verify_mcu(client)
	);
	if (ret < 0)
	{
		dev_err(dev, "Error occurred when verifying MCU\n");
		return ret;
	}

	/*
	 * Query the number of controls from MCU
	 */
	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		mcu_count_or_list_ctrls(client, NULL, &num_ctrls)
	);
	if (ret < 0)
	{
		dev_err(dev, "%s, Failed to get number of controls for sensor\n", __func__);
		return ret;
	}

	/*
	 * Query the number for Formats available from MCU
	 */
	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		mcu_count_or_list_fmts(client, NULL, &frm_fmt_size)
	);
	if (ret < 0)
	{
		dev_err(dev, "%s, Failed to get number of formats for sensor\n", __func__);
		return ret;
	}

	/*
	 * Initialise the MCU related data as we're about to use them.
	 */
	ret = mcu_data_init(dev, frm_fmt_size);
	if (ret < 0)
	{
		dev_err(dev, "%s: failed to initialize MCU related data\n", __func__);
		return ret;
	}

	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		mcu_get_sensor_id(client, &sensor_id)
	);
	if (ret < 0)
	{
		dev_err(dev, "Unable to get MCU Sensor ID \n");
		return ret;
	}

	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		mcu_isp_init(client)
	);
	if (ret < 0)
	{
		dev_err(dev, "Unable to INIT ISP \n");
		return ret;
	}

	/*
	 * Enumerate the Formats in the sensor
	 */
	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		mcu_count_or_list_fmts(client, stream_info, &frm_fmt_size)
	);
	if (ret < 0)
	{
		dev_err(dev, "Unable to enumerate the formats in the sensor\n");
		return ret;
	}

	/*
	 * Fill some state information as required.
	 */
	ar1335_data.i2c_client = client;

	ar1335_data.pix.pixelformat = V4L2_PIX_FMT_UYVY;

	if(ar1335_data.soc_format_check){
		ar1335_data.fmt.code = AR1335_DEFAULT_DATAFMT_YUYV;
	}
	else{
		ar1335_data.fmt.code = AR1335_DEFAULT_DATAFMT;
	}
	ar1335_data.fmt.colorspace = AR1335_DEFAULT_COLORSPACE;
	ar1335_data.pix.width = AR1335_DEFAULT_WIDTH;
	ar1335_data.pix.height = AR1335_DEFAULT_HEIGHT;
	ar1335_data.streamcap.capability =  V4L2_MODE_HIGHQUALITY | V4L2_CAP_TIMEPERFRAME;
	ar1335_data.streamcap.capturemode = AR1335_DEFAULT_MODE;
	ar1335_data.streamcap.timeperframe.denominator = AR1335_DEFAULT_FPS;
	ar1335_data.streamcap.timeperframe.numerator = 1;
	ar1335_data.num_frm_fmts = frm_fmt_size;
	ar1335_data.power_on = 0;

	/*
	 * Configure the stream with default configuration
	 */
	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		ar1335_init(client)
	);
	if (ret < 0)
	{
		dev_err(dev, "Failed to initialise the device with default configuration\n");
		return ret;
	}

	v4l2_i2c_subdev_init(&ar1335_data.subdev, client, &ar1335_subdev_ops);

	/*
	 * Initialize Controls by getting details about the controls from the MCU
	 */
	ret = RETRY_SEQUENCE(
		retries_for_i2c_commands,
		ar1335_ctrls_init(mcu_ctrl_info)
	);
	if (ret < 0)
	{
		dev_warn(dev, "Failed to initialise the controls. Controls might not work\n");
	}

	/*
	 * Write default values for all controls
	 */
	for (i = 0; i < num_ctrls; i++) {
		if (mcu_ctrl_info[i].ctrl_type == CTRL_STANDARD) {
			int ret;
			struct v4l2_control ctrl = {
				.id = mcu_ctrl_info[i].ctrl_id,
				.value = mcu_ctrl_info[i].ctrl_data.std.ctrl_def
			};

			if (
				mcu_ctrl_info[i].ctrl_id == 0x9a0926
			)
			{
				continue;
			}

			ret = RETRY_SEQUENCE(
				retries_for_i2c_commands,
				ar1335_s_ctrl(&ar1335_data.subdev, &ctrl)
			);
			if (ret < 0)
			{
				dev_warn(dev, "Failed to write default value for a control: %d; Control ID: %x\n", i, mcu_ctrl_info[i].ctrl_id);
			}
		}
	}

	ar1335_data.subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ar1335_data.subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ar1335_data.pads[0].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&ar1335_data.subdev.entity, 1, ar1335_data.pads);
	ar1335_data.subdev.entity.ops = &ar1335_sd_media_ops;
	if (ret < 0) {
		dev_err(dev, "Failed to init media entity pads\n");
		return ret;
	}

	ret = v4l2_async_register_subdev(&ar1335_data.subdev);
	if (ret)
	{
		dev_err(dev, "Failed to register the I2C subdev for the sensor\n");
		return ret;
	}

	pr_info("AR1335 detected.\n");

	return 0;
}

/*!
 * ar1335 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static void ar1335_remove(struct i2c_client *client)
{
//	int err;
	v4l2_async_unregister_subdev(&ar1335_data.subdev);

	clk_disable_unprepare(ar1335_data.sensor_clk);

	/*
	 * Power down the MCU
	 */
	if (reset_gpio >= 0)
	{
		toggle_gpio(reset_gpio, 0);
	}

#if 0
	/*
	 * Free up the GPIOs
	 */
	if (pwdn_gpio >= 0)
		devm_gpio_free(&client->dev, pwdn_gpio);

	if (reset_gpio >= 0)
		devm_gpio_free(&client->dev, reset_gpio);
#endif
	return;
}

static const struct i2c_device_id ar1335_af_id[] = {
	{"ar1335_af", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ar1335_af_id);

static const struct of_device_id ar1335_camera_dt_ids[] = {
	{ .compatible = "econ,ar1335_af" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ar1335_camera_dt_ids);

static struct i2c_driver ar1335_i2c_driver = {
	.driver = {
		   .name = "ar1335_af",
		   .of_match_table = ar1335_camera_dt_ids,
		   .owner = THIS_MODULE
	},
	.probe = ar1335_probe,
	.remove = ar1335_remove,
	.id_table = ar1335_af_id,
};


module_i2c_driver(ar1335_i2c_driver);

MODULE_DESCRIPTION("AR1335 Auto-Focus V4L2 driver");
MODULE_AUTHOR("e-con Systems");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
