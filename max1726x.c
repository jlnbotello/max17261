/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 * $Date: 2019-10-18 09:19:34 -0500 (18 Oct 2019) $
 * $Revision: 1.0 $
 *
 ******************************************************************************/
 
 /**
 * @file    max1726x.c
 * @brief   max1726x driver.
 *          
 */


/**** Includes ****/
#include "max1726x.h"
#include "low_iface.h"
#include "esp_log.h"

/**** Globals ****/
uint16_t max1726x_regs[256];
uint16_t max1726x_serialnum[8];
max1726x_ez_config_t max1726x_ez_config;
max1726x_short_ini_t max1726x_short_ini;
max1726x_full_ini_t max1726x_full_ini;
max1726x_learned_parameters_t max1726x_learned_parameters;

/**** Static ****/
static void exit_hibernate_mode(){
	// Exit Hibernate Mode step storing orignal HibCFG value. See user guide Soft-Wakeup

	/// Store original HibCFG value
	max1726x_read_reg(MAX1726X_HIBCFG_REG, &max1726x_regs[MAX1726X_HIBCFG_REG]);
	/// Exit hibernate
	uint16_t tempdata = 0x0090;
	max1726x_write_reg(MAX1726X_SOFTWAKEUP_REG, &tempdata);
	tempdata = 0x0000;
	max1726x_write_reg(MAX1726X_HIBCFG_REG, &tempdata);
	max1726x_write_reg(MAX1726X_SOFTWAKEUP_REG, &tempdata);
}
static void rest_hibernate_mode(){
	/// Restore Original HibCFG value
	max1726x_write_reg(MAX1726X_HIBCFG_REG, &max1726x_regs[MAX1726X_HIBCFG_REG]);
}


/**** Functions ****/

/* ************************************************************************* */
ret_t max1726x_write_reg(uint8_t reg_addr, uint16_t *reg_data)
{
	uint8_t i2c_data[3];
	
	i2c_data[0] = reg_addr;
	i2c_data[1] = (*reg_data) & 0xFF;
	i2c_data[2] = (*reg_data) >> 8;
	return i2c_iface_write(MAX1726X_I2C_ADDR, i2c_data, 3, 0);

}

/* ************************************************************************* */
ret_t max1726x_read_reg(uint8_t reg_addr, uint16_t *reg_data)
{
	ret_t ret;
	uint8_t i2c_data[2];
	
	i2c_data[0] = reg_addr;
	i2c_iface_write(MAX1726X_I2C_ADDR, i2c_data, 1, 1);
	
	ret = i2c_iface_read(MAX1726X_I2C_ADDR, i2c_data, 2, 0);
	
	*reg_data = i2c_data[1];
	*reg_data = ((*reg_data)<<8) | i2c_data[0];

	return ret;
}

/* ************************************************************************* */
uint8_t max1726x_write_and_verify_reg(uint8_t reg_addr, uint16_t *reg_data)
{
	uint8_t i2c_data[3];
	uint16_t readback_data;
	int8_t retry;
	
	retry = 3;
	
	while(retry>0)
	{
		i2c_data[0] = reg_addr;
		i2c_data[1] = (*reg_data) & 0xFF;
		i2c_data[2] = (*reg_data) >> 8;
		i2c_iface_write(MAX1726X_I2C_ADDR, i2c_data, 3, 0);
		
		delay_iface_ms(10);	// about 10ms
		
		i2c_data[0] = reg_addr;
		i2c_iface_write(MAX1726X_I2C_ADDR, i2c_data, 1, 1);
		
		i2c_data[0] = 0x00;
		i2c_data[1] = 0x00;
		i2c_iface_read(MAX1726X_I2C_ADDR, i2c_data, 2, 0);
		readback_data = i2c_data[1];
		readback_data = (readback_data<<8) | i2c_data[0];
		
		if(readback_data == (*reg_data))
		{
			return 0; 	// no error
		}
		else
		{
			retry--;
		}
	}
	
	return 1;	// error
}

/* ************************************************************************* */
uint8_t max1726x_check_por(void)
{
	max1726x_read_reg(MAX1726X_STATUS_REG, &max1726x_regs[MAX1726X_STATUS_REG]);
	
	if((max1726x_regs[MAX1726X_STATUS_REG] & 0x0002) == 0x0000)
	{
		return 0;	// No power on reset
	}
	else
	{
		return 1;	// Power on reset
	}
}

/* ************************************************************************* */
uint8_t max1726x_clear_por(void)
{
	max1726x_read_reg(MAX1726X_STATUS_REG, &max1726x_regs[MAX1726X_STATUS_REG]);
	
	max1726x_regs[MAX1726X_STATUS_REG] = max1726x_regs[MAX1726X_STATUS_REG] & 0xFFFD;
	
	return max1726x_write_and_verify_reg(MAX1726X_STATUS_REG, &max1726x_regs[MAX1726X_STATUS_REG]);
}

/* ************************************************************************* */
void max1726x_wait_dnr(void)
{
	max1726x_read_reg(MAX1726X_FSTAT_REG, &max1726x_regs[MAX1726X_FSTAT_REG]);
	
	while((max1726x_regs[MAX1726X_FSTAT_REG] & 0x0001) == 0x0001)
	{
		delay_iface_ms(10);	// about 10ms
		max1726x_read_reg(MAX1726X_FSTAT_REG, &max1726x_regs[MAX1726X_FSTAT_REG]);
	}
		
}

/* ************************************************************************* */
void max1726x_initialize_ez_config(max1726x_ez_config_t *ez_cfg)
{
	max1726x_ez_config.designcap = ez_cfg->designcap;
	max1726x_ez_config.ichgterm = ez_cfg->ichgterm;
	max1726x_ez_config.modelcfg = ez_cfg->modelcfg;
	max1726x_ez_config.vempty = ez_cfg->vempty;

	/// Exit Hibernate Mode storing original HibCFG value
	exit_hibernate_mode();
	
	/// OPTION 1 EZ Config (No INI file is needed)
	max1726x_regs[MAX1726X_DESIGNCAP_REG] = max1726x_ez_config.designcap;
	max1726x_regs[MAX1726X_ICHGTERM_REG] = max1726x_ez_config.ichgterm;
	max1726x_regs[MAX1726X_VEMPTY_REG] = max1726x_ez_config.vempty;
	max1726x_regs[MAX1726X_MODELCFG_REG] = max1726x_ez_config.modelcfg;
	
	max1726x_write_reg(MAX1726X_DESIGNCAP_REG, &max1726x_regs[MAX1726X_DESIGNCAP_REG]);
	max1726x_write_reg(MAX1726X_ICHGTERM_REG, &max1726x_regs[MAX1726X_ICHGTERM_REG]);
	max1726x_write_reg(MAX1726X_VEMPTY_REG, &max1726x_regs[MAX1726X_VEMPTY_REG]);
	max1726x_write_reg(MAX1726X_MODELCFG_REG, &max1726x_regs[MAX1726X_MODELCFG_REG]);
	
	
	//Poll ModelCFG.Refresh bit, do not continue until ModelCFG.Refresh==0
	max1726x_read_reg(MAX1726X_MODELCFG_REG, &max1726x_regs[MAX1726X_MODELCFG_REG]);
	
	while((max1726x_regs[MAX1726X_MODELCFG_REG] & 0x8000) == 0x8000)
	{
		delay_iface_ms(10);	// about 10ms	
		max1726x_read_reg(MAX1726X_MODELCFG_REG, &max1726x_regs[MAX1726X_MODELCFG_REG]);
	}
	
	/// Restore Original HibCFG value
	rest_hibernate_mode();

}

/* ************************************************************************* */
void max1726x_initialize_short_ini(void)
{
	/// customer must provide the battery parameters, get the parameters from MAXIM INI file
	max1726x_short_ini.designcap  = 0x06aE;
	max1726x_short_ini.ichgterm   = 0x0100;
	max1726x_short_ini.modelcfg   = 0x8410;
	max1726x_short_ini.learncfg   = 0x0000;		// Optional
	max1726x_short_ini.fullsocthr = 0x0000;		// Optional
	max1726x_short_ini.qrtable00  = 0x1050;
	max1726x_short_ini.qrtable10  = 0x0014;
	max1726x_short_ini.qrtable20  = 0x0000;		// Optional
	max1726x_short_ini.qrtable30  = 0x0000;		// Optional
	max1726x_short_ini.vempty     = 0x965A;
	max1726x_short_ini.rcomp0     = 0x0070;
	max1726x_short_ini.tempco     = 0x223E;
	/// customer must provide the battery parameters, get the parameters from MAXIM INI file
	
	
	/// Exit Hibernate Mode storing original HibCFG value
	exit_hibernate_mode();
	
	/// OPTION 2 Custom Short INI without OCV Table
	max1726x_regs[MAX1726X_DESIGNCAP_REG] = max1726x_short_ini.designcap;
	max1726x_regs[MAX1726X_ICHGTERM_REG] = max1726x_short_ini.ichgterm;
	max1726x_regs[MAX1726X_VEMPTY_REG] = max1726x_short_ini.vempty;
	max1726x_regs[MAX1726X_MODELCFG_REG] = max1726x_short_ini.modelcfg;
	max1726x_regs[MAX1726X_LEARNCFG_REG] = max1726x_short_ini.learncfg;
	max1726x_regs[MAX1726X_FULLSOCTHR_REG] = max1726x_short_ini.fullsocthr;
	max1726x_regs[MAX1726X_QRTABLE00_REG] = max1726x_short_ini.qrtable00;
	max1726x_regs[MAX1726X_QRTABLE10_REG] = max1726x_short_ini.qrtable10;
	max1726x_regs[MAX1726X_QRTABLE20_REG] = max1726x_short_ini.qrtable20;
	max1726x_regs[MAX1726X_QRTABLE30_REG] = max1726x_short_ini.qrtable30;
	max1726x_regs[MAX1726X_RCOMP0_REG] = max1726x_short_ini.rcomp0;
	max1726x_regs[MAX1726X_TEMPCO_REG] = max1726x_short_ini.tempco;
	
	max1726x_write_reg(MAX1726X_DESIGNCAP_REG, &max1726x_regs[MAX1726X_DESIGNCAP_REG]);
	max1726x_write_reg(MAX1726X_ICHGTERM_REG, &max1726x_regs[MAX1726X_ICHGTERM_REG]);
	max1726x_write_reg(MAX1726X_VEMPTY_REG, &max1726x_regs[MAX1726X_VEMPTY_REG]);
	// max1726x_write_and_verify_reg(MAX1726X_LEARNCFG_REG, &max1726x_regs[MAX1726X_LEARNCFG_REG]);	// optional
	// max1726x_write_and_verify_reg(MAX1726X_FULLSOCTHR_REG, &max1726x_regs[MAX1726X_FULLSOCTHR_REG]);	// optional
	max1726x_write_reg(MAX1726X_MODELCFG_REG, &max1726x_regs[MAX1726X_MODELCFG_REG]);
	
	
	//Poll ModelCFG.Refresh bit, do not continue until ModelCFG.Refresh==0
	max1726x_read_reg(MAX1726X_MODELCFG_REG, &max1726x_regs[MAX1726X_MODELCFG_REG]);
	
	while((max1726x_regs[MAX1726X_MODELCFG_REG] & 0x8000) == 0x8000)
	{
		delay_iface_ms(10);	// about 10ms	
		max1726x_read_reg(MAX1726X_MODELCFG_REG, &max1726x_regs[MAX1726X_MODELCFG_REG]);
	}
	
	max1726x_write_reg(MAX1726X_RCOMP0_REG, &max1726x_regs[MAX1726X_RCOMP0_REG]);
	max1726x_write_reg(MAX1726X_TEMPCO_REG, &max1726x_regs[MAX1726X_TEMPCO_REG]);
	max1726x_write_reg(MAX1726X_QRTABLE00_REG, &max1726x_regs[MAX1726X_QRTABLE00_REG]);
	max1726x_write_reg(MAX1726X_QRTABLE10_REG, &max1726x_regs[MAX1726X_QRTABLE10_REG]);
	// max1726x_write_reg(MAX1726X_QRTABLE20_REG, &max1726x_regs[MAX1726X_QRTABLE20_REG]);	// optional
	// max1726x_write_reg(MAX1726X_QRTABLE30_REG, &max1726x_regs[MAX1726X_QRTABLE30_REG]);	// optional
	
	/// Restore Original HibCFG value
	rest_hibernate_mode();

}

/* ************************************************************************* */
void max1726x_initialize_full_ini(void)
{
	uint8_t ret;
	
	
	/// customer must provide the battery parameters, get the parameters from MAXIM INI file
	max1726x_full_ini.designcap  = 0x06aE;
	max1726x_full_ini.fullcaprep = 0x06aE;
	max1726x_full_ini.ichgterm   = 0x0100;
	max1726x_full_ini.modelcfg   = 0x8410;
	max1726x_full_ini.qrtable00  = 0x1050;
	max1726x_full_ini.qrtable10  = 0x0014;
	max1726x_full_ini.qrtable20  = 0x1300;	// optional
	max1726x_full_ini.qrtable30  = 0x0C00;	// optional
	max1726x_full_ini.vempty     = 0x965A;	
	max1726x_full_ini.rcomp0     = 0x0070;
	max1726x_full_ini.tempco     = 0x223E;
	max1726x_full_ini.learncfg   = 0x0000;	// optional
	max1726x_full_ini.relaxcfg   = 0x0000;	// optional
	max1726x_full_ini.config     = 0x0000;	// optional
	max1726x_full_ini.config2    = 0x0000;	// optional
	max1726x_full_ini.fullsocthr = 0x5F05;	// optional
	max1726x_full_ini.tgain      = 0x0000;	// optional
	max1726x_full_ini.toff       = 0x0000;	// optional
	max1726x_full_ini.curve      = 0x0000;	// optional
	
	
	
	max1726x_full_ini.modeldata0[0]  = 0x0000;
	max1726x_full_ini.modeldata0[1]  = 0x0000;
	max1726x_full_ini.modeldata0[2]  = 0x0000;
	max1726x_full_ini.modeldata0[3]  = 0x0000;
	max1726x_full_ini.modeldata0[4]  = 0x0000;
	max1726x_full_ini.modeldata0[5]  = 0x0000;
	max1726x_full_ini.modeldata0[6]  = 0x0000;
	max1726x_full_ini.modeldata0[7]  = 0x0000;
	max1726x_full_ini.modeldata0[8]  = 0x0000;
	max1726x_full_ini.modeldata0[9]  = 0x0000;
	max1726x_full_ini.modeldata0[10] = 0x0000;
	max1726x_full_ini.modeldata0[11] = 0x0000;
	max1726x_full_ini.modeldata0[12] = 0x0000;
	max1726x_full_ini.modeldata0[13] = 0x0000;
	max1726x_full_ini.modeldata0[14] = 0x0000;
	max1726x_full_ini.modeldata0[15] = 0x0000;
	
	max1726x_full_ini.modeldata1[0]  = 0x0000;
	max1726x_full_ini.modeldata1[1]  = 0x0000;
	max1726x_full_ini.modeldata1[2]  = 0x0000;
	max1726x_full_ini.modeldata1[3]  = 0x0000;
	max1726x_full_ini.modeldata1[4]  = 0x0000;
	max1726x_full_ini.modeldata1[5]  = 0x0000;
	max1726x_full_ini.modeldata1[6]  = 0x0000;
	max1726x_full_ini.modeldata1[7]  = 0x0000;
	max1726x_full_ini.modeldata1[8]  = 0x0000;
	max1726x_full_ini.modeldata1[9]  = 0x0000;
	max1726x_full_ini.modeldata1[10] = 0x0000;
	max1726x_full_ini.modeldata1[11] = 0x0000;
	max1726x_full_ini.modeldata1[12] = 0x0000;
	max1726x_full_ini.modeldata1[13] = 0x0000;
	max1726x_full_ini.modeldata1[14] = 0x0000;
	max1726x_full_ini.modeldata1[15] = 0x0000;
	/// customer must provide the battery parameters, get the parameters from MAXIM INI file

	/// Exit Hibernate Mode storing original HibCFG value
	exit_hibernate_mode();
	
	/// OPTION 3 Custom Full INI with OCV Table
	max1726x_regs[MAX1726X_DESIGNCAP_REG] = max1726x_full_ini.designcap;
	max1726x_regs[MAX1726X_FULLCAPREP_REG] = max1726x_full_ini.fullcaprep;
	max1726x_regs[MAX1726X_ICHGTERM_REG] = max1726x_full_ini.ichgterm;
	max1726x_regs[MAX1726X_VEMPTY_REG] = max1726x_full_ini.vempty;
	max1726x_regs[MAX1726X_MODELCFG_REG] = max1726x_full_ini.modelcfg;
	max1726x_regs[MAX1726X_QRTABLE00_REG] = max1726x_full_ini.qrtable00;
	max1726x_regs[MAX1726X_QRTABLE10_REG] = max1726x_full_ini.qrtable10;
	max1726x_regs[MAX1726X_QRTABLE20_REG] = max1726x_full_ini.qrtable20;	// optional
	max1726x_regs[MAX1726X_QRTABLE30_REG] = max1726x_full_ini.qrtable30;	// optional
	max1726x_regs[MAX1726X_RCOMP0_REG] = max1726x_full_ini.rcomp0;
	max1726x_regs[MAX1726X_TEMPCO_REG] = max1726x_full_ini.tempco;
	max1726x_regs[MAX1726X_LEARNCFG_REG] = max1726x_full_ini.learncfg;		// optional
	max1726x_regs[MAX1726X_RELAXCFG_REG] = max1726x_full_ini.relaxcfg;		// optional
	max1726x_regs[MAX1726X_CONFIG_REG] = max1726x_full_ini.config;			// optional
	max1726x_regs[MAX1726X_CONFIG2_REG] = max1726x_full_ini.config2;		// optional
	max1726x_regs[MAX1726X_FULLSOCTHR_REG] = max1726x_full_ini.fullsocthr;	// optional
	max1726x_regs[MAX1726X_TGAIN_REG] = max1726x_full_ini.tgain;			// optional
	max1726x_regs[MAX1726X_TOFF_REG] = max1726x_full_ini.toff;				// optional
	max1726x_regs[MAX1726X_CURVE_REG] = max1726x_full_ini.curve;			// optional
	
	
	
	/// Write/Read/Verify the Custom Model Data
	ret = 1;
	while(ret!=0)
	{
		/// Unlock Model Access
		max1726x_unlock_model_data();
		
		delay_iface_ms(10);	// about 10ms
		
		/// Write/Read/Verify the Custom Model Data
		ret = max1726x_write_model_data(max1726x_full_ini.modeldata0, max1726x_full_ini.modeldata1);
	}
	
	/// Lock Model Access
	ret = 1;
	while(ret!=0)
	{
		/// Lock Model Access
		max1726x_lock_model_data();
		
		delay_iface_ms(10);	// about 10ms
		
		ret = max1726x_verify_model_data_locked();
	}
	
	/// Write Custom Parameters
	max1726x_regs[MAX1726X_REPCAP_REG] = 0x0000;
	max1726x_write_reg(MAX1726X_REPCAP_REG, &max1726x_regs[MAX1726X_REPCAP_REG]);	
	
	max1726x_write_reg(MAX1726X_DESIGNCAP_REG, &max1726x_regs[MAX1726X_DESIGNCAP_REG]);
	max1726x_write_reg(MAX1726X_FULLCAPREP_REG, &max1726x_regs[MAX1726X_FULLCAPREP_REG]);
	
	
	max1726x_regs[MAX1726X_DQACC_REG] = max1726x_regs[MAX1726X_DESIGNCAP_REG] / 2;
	max1726x_regs[MAX1726X_DPACC_REG] = 0x0C80;
	max1726x_write_reg(MAX1726X_DQACC_REG, &max1726x_regs[MAX1726X_DQACC_REG]);
	max1726x_write_reg(MAX1726X_DPACC_REG, &max1726x_regs[MAX1726X_DPACC_REG]);
	
	max1726x_write_reg(MAX1726X_ICHGTERM_REG, &max1726x_regs[MAX1726X_ICHGTERM_REG]);
	max1726x_write_reg(MAX1726X_VEMPTY_REG, &max1726x_regs[MAX1726X_VEMPTY_REG]);
	max1726x_write_reg(MAX1726X_RCOMP0_REG, &max1726x_regs[MAX1726X_RCOMP0_REG]);
	max1726x_write_reg(MAX1726X_TEMPCO_REG, &max1726x_regs[MAX1726X_TEMPCO_REG]);
	max1726x_write_reg(MAX1726X_QRTABLE00_REG, &max1726x_regs[MAX1726X_QRTABLE00_REG]);
	max1726x_write_reg(MAX1726X_QRTABLE10_REG, &max1726x_regs[MAX1726X_QRTABLE10_REG]);
	
	// max1726x_write_reg(MAX1726X_QRTABLE20_REG, &max1726x_regs[MAX1726X_QRTABLE20_REG]);			// optional
	// max1726x_write_reg(MAX1726X_QRTABLE30_REG, &max1726x_regs[MAX1726X_QRTABLE30_REG]);			// optional
	// max1726x_write_and_verify_reg(MAX1726X_LEARNCFG_REG, &max1726x_regs[MAX1726X_LEARNCFG_REG]);	// optional
	// max1726x_write_reg(MAX1726X_RELAXCFG_REG, &max1726x_regs[MAX1726X_RELAXCFG_REG]);				// optional
	// max1726x_write_reg(MAX1726X_CONFIG_REG, &max1726x_regs[MAX1726X_CONFIG_REG]);					// optional
	// max1726x_write_reg(MAX1726X_CONFIG2_REG, &max1726x_regs[MAX1726X_CONFIG2_REG]);				// optional
	// max1726x_write_reg(MAX1726X_FULLSOCTHR_REG, &max1726x_regs[MAX1726X_FULLSOCTHR_REG]);			// optional
	// max1726x_write_reg(MAX1726X_TGAIN_REG, &max1726x_regs[MAX1726X_TGAIN_REG]);					// optional
	// max1726x_write_reg(MAX1726X_TOFF_REG, &max1726x_regs[MAX1726X_TOFF_REG]);						// optional
	// max1726x_write_reg(MAX1726X_CURVE_REG, &max1726x_regs[MAX1726X_CURVE_REG]);					// optional
	
	
	
	/// Initiate Model Loading, by setting the LdMdl bit in the Config2 register
	max1726x_read_reg(MAX1726X_CONFIG2_REG, &max1726x_regs[MAX1726X_CONFIG2_REG]);
	max1726x_regs[MAX1726X_CONFIG2_REG] = max1726x_regs[MAX1726X_CONFIG2_REG] | 0x0020;
	max1726x_write_reg(MAX1726X_CONFIG2_REG, &max1726x_regs[MAX1726X_CONFIG2_REG]);
	
	
	/// Poll Config2.LdMdl bit, do not continue until Config2.LdMdl==0
	max1726x_read_reg(MAX1726X_CONFIG2_REG, &max1726x_regs[MAX1726X_CONFIG2_REG]);
	
	while((max1726x_regs[MAX1726X_CONFIG2_REG] & 0x0020) == 0x0020)
	{
		delay_iface_ms(10);	// about 10ms	
		max1726x_read_reg(MAX1726X_CONFIG2_REG, &max1726x_regs[MAX1726X_CONFIG2_REG]);
	}
	
	/// Update QRTable20 and QRTable30
	max1726x_write_and_verify_reg(MAX1726X_QRTABLE20_REG, &max1726x_regs[MAX1726X_QRTABLE20_REG]);	
	max1726x_write_and_verify_reg(MAX1726X_QRTABLE30_REG, &max1726x_regs[MAX1726X_QRTABLE30_REG]);	

	
	/// Restore Original HibCFG value
	rest_hibernate_mode();

}

/* ************************************************************************* */
float max1726x_get_repcap(float Rsense)
{
	float repcap;
	max1726x_read_reg(MAX1726X_REPCAP_REG, &max1726x_regs[MAX1726X_REPCAP_REG]);
	
	repcap = (float)max1726x_regs[MAX1726X_REPCAP_REG] * 5.0f / (float)Rsense;
	return repcap;
}

/* ************************************************************************* */
float max1726x_get_repsoc(void)
{
	float repsoc;
	max1726x_read_reg(MAX1726X_REPSOC_REG, &max1726x_regs[MAX1726X_REPSOC_REG]);
	
	repsoc = (float)max1726x_regs[MAX1726X_REPSOC_REG] / 256.0f;
	return repsoc;
}

/* ************************************************************************* */
float max1726x_get_tte(void)
{
	float tte;
	max1726x_read_reg(MAX1726X_TTE_REG, &max1726x_regs[MAX1726X_TTE_REG]);
	
	tte = (float)max1726x_regs[MAX1726X_TTE_REG] * 5.625f;
	return tte;
}

/* ************************************************************************* */
ret_t max1726x_get_learned_parameters(max1726x_learned_parameters_t *lp)
{
	ret_t ret = RET_FAIL;

	ret = max1726x_read_reg(MAX1726X_RCOMP0_REG, &max1726x_regs[MAX1726X_RCOMP0_REG]);
	ret = max1726x_read_reg(MAX1726X_TEMPCO_REG, &max1726x_regs[MAX1726X_TEMPCO_REG]);
	ret = max1726x_read_reg(MAX1726X_FULLCAPREP_REG, &max1726x_regs[MAX1726X_FULLCAPREP_REG]);
	ret = max1726x_read_reg(MAX1726X_CYCLES_REG, &max1726x_regs[MAX1726X_CYCLES_REG]);
	ret = max1726x_read_reg(MAX1726X_FULLCAPNOM_REG, &max1726x_regs[MAX1726X_FULLCAPNOM_REG]);
	
	
	lp->saved_rcomp0 = max1726x_regs[MAX1726X_RCOMP0_REG];
	lp->saved_tempco = max1726x_regs[MAX1726X_TEMPCO_REG];
	lp->saved_fullcaprep = max1726x_regs[MAX1726X_FULLCAPREP_REG];
	lp->saved_cycles = max1726x_regs[MAX1726X_CYCLES_REG];
	lp->saved_fullcapnom = max1726x_regs[MAX1726X_FULLCAPNOM_REG];

	return ret;
}

/* ************************************************************************* */
ret_t max1726x_restore_learned_parameters(max1726x_learned_parameters_t *lp)
{
	max1726x_regs[MAX1726X_RCOMP0_REG] = lp->saved_rcomp0;
	max1726x_regs[MAX1726X_TEMPCO_REG] = lp->saved_tempco;
	max1726x_regs[MAX1726X_FULLCAPREP_REG] = lp->saved_fullcaprep;
	max1726x_regs[MAX1726X_CYCLES_REG] = lp->saved_cycles;
	max1726x_regs[MAX1726X_FULLCAPNOM_REG] = lp->saved_fullcapnom;
	
	uint16_t temp = 0x000F; // value meaning not documented. GUI does it and now restoring works
	max1726x_write_reg(MAX1726X_SOFTWAKEUP_REG, &temp); 
	temp = 0x0000; 			// clears command register
	max1726x_write_reg(MAX1726X_SOFTWAKEUP_REG, &temp);

	max1726x_write_and_verify_reg(MAX1726X_RCOMP0_REG, &max1726x_regs[MAX1726X_RCOMP0_REG]);
	ESP_LOGI("RESTORING....","write rcomp0: 0x%x",max1726x_regs[MAX1726X_RCOMP0_REG]);

	max1726x_write_and_verify_reg(MAX1726X_TEMPCO_REG, &max1726x_regs[MAX1726X_TEMPCO_REG]);
	ESP_LOGI("RESTORING....","write tempco: 0x%x",max1726x_regs[MAX1726X_TEMPCO_REG]);

	max1726x_write_and_verify_reg(MAX1726X_FULLCAPREP_REG, &max1726x_regs[MAX1726X_FULLCAPREP_REG]);
	ESP_LOGI("RESTORING....","write fullcaprep: 0x%x",max1726x_regs[MAX1726X_FULLCAPREP_REG]);

	max1726x_write_and_verify_reg(MAX1726X_FULLCAPNOM_REG, &max1726x_regs[MAX1726X_FULLCAPNOM_REG]);
	ESP_LOGI("RESTORING....","write fullcapnom: 0x%x",max1726x_regs[MAX1726X_FULLCAPNOM_REG]);

	// Write dPacc to 200%
	max1726x_regs[MAX1726X_DPACC_REG] = 0x0C80;
	max1726x_write_and_verify_reg(MAX1726X_DPACC_REG, &max1726x_regs[MAX1726X_DPACC_REG]);
	ESP_LOGI("RESTORING....","write dpacc: 0x%x",max1726x_regs[MAX1726X_DPACC_REG]);

	// Write dQacc to 200% of Capacity
	max1726x_regs[MAX1726X_DQACC_REG] = (uint16_t) (lp->saved_fullcapnom / 2); // factor explanation: 2 * [CAP_LSB]/[DQACC_LSB] = 2 * (1/4) = (1/2) 
	max1726x_write_and_verify_reg(MAX1726X_DQACC_REG, &max1726x_regs[MAX1726X_DQACC_REG]);
	ESP_LOGI("RESTORING....","write dqacc: 0x%x",max1726x_regs[MAX1726X_DQACC_REG]);
	
	max1726x_write_and_verify_reg(MAX1726X_CYCLES_REG, &max1726x_regs[MAX1726X_CYCLES_REG]);
	ESP_LOGI("RESTORING....","write cycles: 0x%x",max1726x_regs[MAX1726X_CYCLES_REG]);

	return RET_OK;
}

/* ************************************************************************* */
void max1726x_get_serial_number(uint16_t *sn)
{
	
	max1726x_read_reg(MAX1726X_CONFIG2_REG, &max1726x_regs[MAX1726X_CONFIG2_REG]);
	
	// clear AtRateEn bit and DPEn bit in Config2 register
	max1726x_regs[MAX1726X_CONFIG2_REG] = max1726x_regs[MAX1726X_CONFIG2_REG] & 0xCFFF;
	max1726x_write_and_verify_reg(MAX1726X_CONFIG2_REG, &max1726x_regs[MAX1726X_CONFIG2_REG]);
	delay_iface_ms(40);	// about 40ms
	
	max1726x_read_reg(MAX1726X_MAXPEAKPOWER_REG, &max1726x_regs[MAX1726X_MAXPEAKPOWER_REG]);
	max1726x_read_reg(MAX1726X_SUSPEAKPOWER_REG, &max1726x_regs[MAX1726X_SUSPEAKPOWER_REG]);
	max1726x_read_reg(MAX1726X_MPPCURRENT_REG, &max1726x_regs[MAX1726X_MPPCURRENT_REG]);
	max1726x_read_reg(MAX1726X_SPPCURRENT_REG, &max1726x_regs[MAX1726X_SPPCURRENT_REG]);
	max1726x_read_reg(MAX1726X_ATQRESIDUAL_REG, &max1726x_regs[MAX1726X_ATQRESIDUAL_REG]);
	max1726x_read_reg(MAX1726X_ATTTE_REG, &max1726x_regs[MAX1726X_ATTTE_REG]);
	max1726x_read_reg(MAX1726X_ATAVSOC_REG, &max1726x_regs[MAX1726X_ATAVSOC_REG]);
	max1726x_read_reg(MAX1726X_ATAVCAP_REG, &max1726x_regs[MAX1726X_ATAVCAP_REG]);
	
	
	sn[0] = max1726x_regs[MAX1726X_MAXPEAKPOWER_REG];
	sn[1] = max1726x_regs[MAX1726X_SUSPEAKPOWER_REG];
	sn[2] = max1726x_regs[MAX1726X_MPPCURRENT_REG];
	sn[3] = max1726x_regs[MAX1726X_SPPCURRENT_REG];
	sn[4] = max1726x_regs[MAX1726X_ATQRESIDUAL_REG];
	sn[5] = max1726x_regs[MAX1726X_ATTTE_REG];
	sn[6] = max1726x_regs[MAX1726X_ATAVSOC_REG];
	sn[7] = max1726x_regs[MAX1726X_ATAVCAP_REG];
	
	
	// set AtRateEn bit and DPEn bit in Config2 register
	max1726x_regs[MAX1726X_CONFIG2_REG] = max1726x_regs[MAX1726X_CONFIG2_REG] | 0x3000;
	max1726x_write_and_verify_reg(MAX1726X_CONFIG2_REG, &max1726x_regs[MAX1726X_CONFIG2_REG]);
	delay_iface_ms(40);	// about 40ms
}

/* ************************************************************************* */
void max1726x_unlock_model_data(void)
{
	uint16_t tempdata;
	
	tempdata = 0x0059;
	max1726x_write_reg(0x62, &tempdata);
	tempdata = 0x00C4;
	max1726x_write_reg(0x63, &tempdata);
}

/* ************************************************************************* */
uint8_t max1726x_write_model_data(uint16_t *data0, uint16_t *data1)
{
	uint8_t err_num;
	
	uint8_t i;
	uint16_t readback0[16];
	uint16_t readback1[16];
	
	err_num = 32;
	
	for(i=0; i<16; i++)
	{
		max1726x_write_reg(MAX1726X_MODELDATA0_START_REG+i, &data0[i]);
	}
	for(i=0; i<16; i++)
	{
		max1726x_write_reg(MAX1726X_MODELDATA1_START_REG+i, &data1[i]);
	}
	
	delay_iface_ms(10);	// about 10ms
	
	for(i=0; i<16; i++)
	{
		max1726x_read_reg(MAX1726X_MODELDATA0_START_REG+i, &readback0[i]);
	}
	for(i=0; i<16; i++)
	{
		max1726x_read_reg(MAX1726X_MODELDATA1_START_REG+i, &readback1[i]);
	}
	
	for(i=0; i<16; i++)
	{
		if(readback0[i] == data0[i])
		{
			err_num--;
		}
		if(readback1[i] == data1[i])
		{
			err_num--;
		}
	}
	
	
	
	return err_num;
}

/* ************************************************************************* */
void max1726x_lock_model_data(void)
{
	uint16_t tempdata;

	tempdata = 0x0000;
	max1726x_write_reg(0x62, &tempdata);
	max1726x_write_reg(0x63, &tempdata);
}

/* ************************************************************************* */
uint8_t max1726x_verify_model_data_locked(void)
{
	uint8_t err_num;
	uint8_t i;
	uint16_t readback0[16];
	uint16_t readback1[16];
	
	
	err_num = 32;
	
	for(i=0; i<16; i++)
	{
		max1726x_read_reg(MAX1726X_MODELDATA0_START_REG+i, &readback0[i]);
	}
	for(i=0; i<16; i++)
	{
		max1726x_read_reg(MAX1726X_MODELDATA1_START_REG+i, &readback1[i]);
	}
	
	for(i=0; i<16; i++)
	{
		if(readback0[i] == 0x0000)
		{
			err_num--;
		}
		if(readback1[i] == 0x0000)
		{
			err_num--;
		}
	}
		
	
	return err_num;
}

ret_t max1726x_read_ez_config(max1726x_ez_config_t *ez_cfg){
	
	ret_t ret = RET_FAIL;

	ret = max1726x_read_reg(MAX1726X_DESIGNCAP_REG, &ez_cfg->designcap);
	ret = max1726x_read_reg(MAX1726X_ICHGTERM_REG, &ez_cfg->ichgterm);
	ret = max1726x_read_reg(MAX1726X_VEMPTY_REG, &ez_cfg->vempty);
	ret = max1726x_read_reg(MAX1726X_MODELCFG_REG, &ez_cfg->modelcfg);

	return ret;
}
