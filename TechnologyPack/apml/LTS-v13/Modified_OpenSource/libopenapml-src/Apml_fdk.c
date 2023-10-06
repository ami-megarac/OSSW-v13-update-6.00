/*
 * APML FDK Routines - V1.0
 *
 ******************************************************************************


============================================================
License Agreement

Copyright (c) 2008, 2009 Advanced Micro Devices, Inc.

All rights reserved.

Redistribution and use in any form of this material and any product thereof
including software in source or binary forms, along with any related
documentation, with or without modification ("this material"), is permitted
provided that the following conditions are met:

+ Redistributions of source code of any software must retain the above
copyright notice and all terms of this license as part of the code.

+ Redistributions in binary form of any software must reproduce the above
copyright notice and all terms of this license in any related documentation
and/or other materials.

+ Neither the names nor trademarks of Advanced Micro Devices, Inc. or any
copyright holders or contributors may be used to endorse or promote products
derived from this material without specific prior written permission.

+ Notice about U.S. Government restricted rights: This material is provided
with RESTRICTED RIGHTS. Use, duplication or disclosure by the U.S. Government
is subject to the full extent of restrictions set forth in FAR52.227 and
DFARS252.227 et seq., or any successor or applicable regulations. Use of this
material by the U.S. Government constitutes acknowledgment of the proprietary
rights of Advanced Micro Devices, Inc. and any copyright holders and
contributors.

+ In no event shall anyone redistributing or accessing or using this material
commence or participate in any arbitration or legal action relating to this
material against Advanced Micro Devices, Inc. or any copyright holders or
contributors. The foregoing shall survive any expiration or termination of this
license or any agreement or access or use related to this material.

+ ANY BREACH OF ANY TERM OF THIS LICENSE SHALL RESULT IN THE IMMEDIATE
REVOCATION OF ALL RIGHTS TO REDISTRIBUTE, ACCESS OR USE THIS MATERIAL.

THIS MATERIAL IS PROVIDED BY ADVANCED MICRO DEVICES, INC. AND ANY COPYRIGHT
HOLDERS AND CONTRIBUTORS "AS IS" IN ITS CURRENT CONDITION AND WITHOUT ANY
REPRESENTATIONS, GUARANTEE, OR WARRANTY OF ANY KIND OR IN ANY WAY RELATED TO
SUPPORT, INDEMNITY, ERROR FREE OR UNINTERRUPTED OPERATION, OR THAT IT IS FREE
FROM DEFECTS OR VIRUSES.  ALL OBLIGATIONS ARE HEREBY DISCLAIMED - WHETHER
EXPRESS, IMPLIED, OR STATUTORY - INCLUDING, BUT NOT LIMITED TO, ANY IMPLIED
WARRANTIES OF TITLE, MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
ACCURACY, COMPLETENESS, OPERABILITY, QUALITY OF SERVICE, OR NON-INFRINGEMENT.
IN NO EVENT SHALL ADVANCED MICRO DEVICES, INC. OR ANY COPYRIGHT HOLDERS OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, PUNITIVE,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, REVENUE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED OR BASED ON ANY THEORY OF LIABILITY
ARISING IN ANY WAY RELATED TO THIS MATERIAL, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE. THE ENTIRE AND AGGREGATE LIABILITY OF ADVANCED MICRO DEVICES,
INC. AND ANY COPYRIGHT HOLDERS AND CONTRIBUTORS SHALL NOT EXCEED TEN DOLLARS
(US $10.00). ANYONE REDISTRIBUTING OR ACCESSING OR USING THIS MATERIAL ACCEPTS
THIS ALLOCATION OF RISK AND AGREES TO RELEASE ADVANCED MICRO DEVICES, INC. AND
ANY COPYRIGHT HOLDERS AND CONTRIBUTORS FROM ANY AND ALL LIABILITIES,
OBLIGATIONS, CLAIMS, OR DEMANDS IN EXCESS OF TEN DOLLARS (US $10.00). THE
FOREGOING ARE ESSENTIAL TERMS OF THIS LICENSE AND, IF ANY OF THESE TERMS ARE
CONSTRUED AS UNENFORCEABLE, FAIL IN ESSENTIAL PURPOSE, OR BECOME VOID OR
DETRIMENTAL TO ADVANCED MICRO DEVICES, INC. OR ANY COPYRIGHT HOLDERS OR
CONTRIBUTORS FOR ANY REASON, THEN ALL RIGHTS TO REDISTRIBUTE, ACCESS OR USE
THIS MATERIAL SHALL TERMINATE IMMEDIATELY. MOREOVER, THE FOREGOING SHALL
SURVIVE ANY EXPIRATION OR TERMINATION OF THIS LICENSE OR ANY AGREEMENT OR
ACCESS OR USE RELATED TO THIS MATERIAL.

NOTICE IS HEREBY PROVIDED, AND BY REDISTRIBUTING OR ACCESSING OR USING THIS
MATERIAL SUCH NOTICE IS ACKNOWLEDGED, THAT THIS MATERIAL MAY BE SUBJECT TO
RESTRICTIONS UNDER THE LAWS AND REGULATIONS OF THE UNITED STATES OR OTHER
COUNTRIES, WHICH INCLUDE BUT ARE NOT LIMITED TO, U.S. EXPORT CONTROL LAWS SUCH
AS THE EXPORT ADMINISTRATION REGULATIONS AND NATIONAL SECURITY CONTROLS AS
DEFINED THEREUNDER, AS WELL AS STATE DEPARTMENT CONTROLS UNDER THE U.S.
MUNITIONS LIST. THIS MATERIAL MAY NOT BE USED, RELEASED, TRANSFERRED, IMPORTED,
EXPORTED AND/OR RE-EXPORTED IN ANY MANNER PROHIBITED UNDER ANY APPLICABLE LAWS,
INCLUDING U.S. EXPORT CONTROL LAWS REGARDING SPECIFICALLY DESIGNATED PERSONS,
COUNTRIES AND NATIONALS OF COUNTRIES SUBJECT TO NATIONAL SECURITY CONTROLS.
MOREOVER, THE FOREGOING SHALL SURVIVE ANY EXPIRATION OR TERMINATION OF ANY
LICENSE OR AGREEMENT OR ACCESS OR USE RELATED TO THIS MATERIAL.

This license forms the entire agreement regarding the subject matter hereof and
supersedes all proposals and prior discussions and writings between the parties
with respect thereto. This license does not affect any ownership, rights,
title, or interest in, or relating to, this material. No terms of this license
can be modified or waived, and no breach of this license can be excused, unless
done so in a writing signed by all affected parties. Each term of this license
is separately enforceable. If any term of this license is determined to be or
becomes unenforceable or illegal, such term shall be reformed to the minimum
extent necessary in order for this license to remain in effect in accordance
with its terms as modified by such reformation. This license shall be governed
by and construed in accordance with the laws of the State of Texas without
regard to rules on conflicts of law of any state or jurisdiction or the United
Nations Convention on the International Sale of Goods. All disputes arising out
of this license shall be subject to the jurisdiction of the federal and state
courts in Austin, Texas, and all defenses are hereby waived concerning personal
jurisdiction and venue of these courts.
============================================================


 ****************************************************************************** 
 */



#include <stdio.h>

#include "Apml_fdk.h"
#include "mmap.h"
#include "EINTR_wrappers.h"
#include "dbgout.h"
#include "safesystem.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

/* **************************************************************************/
/*                                                                          */
/*                         Private Static Defines                           */
/*                                                                          */
/* **************************************************************************/

//PEC enable will append PEC byte to write data and check read data PEC byte
//set to 1 is preferable
//set to 0 for debugging block write-block read process 
//only user_smbus_bwr_brd_process be affected, according to APML spec 1.04
//note this will NOT effect SBRMI_x01 bit7:PECEn
#define IS_PEC_ENABLE 1

#define MAX_PEC_RETRIES 3

#define MAX_ESPIRST_RETRIES 10

/* **************************************************************************/
/*                                                                          */
/*                            Public Functions                              */
/*                                                                          */
/* **************************************************************************/


/* Initializes the FDK for use */
uint32 apml_init_fdk(APML_DEV_CTL *ctl,int BMCInst)
{
	uint32	i, res;
	unsigned long reg;
    struct stat apmldriver;
    int mount_retry=3, platform_retry=3;

	for (i=0 ; i < APML_MAX_PROCS ; i++)
	{
		ctl->threads[i] = 0;
		ctl->rmi_rev[i] = 0;
	    ctl->rmi_thread_target[i] = DEFAULT_RMI_CORE;
	}

    if (0 != stat(FILE_APMLDRIVER, &apmldriver))
    {
        while (mount_retry-- > 0 && mount_apml_driver(ctl, BMCInst) != 0)
        {
            sleep(1);
            printf("Mount APML driver retry %d...\n", 3-mount_retry);
        }

        if (mount_retry < 0)
        {
            printf("APML driver retry time out\n");
            return -1;
        }
    }

    do {
	    res = user_platform_init(ctl, BMCInst);
    } while(platform_retry-- > 0 && res != APML_SUCCESS);

    if (APML_SUCCESS!=res)
    {
        safe_system("rm -f /var/HOST_READY");
        TINFO("APML initial fail");
    }

	/* Set GPIO ESPI_RST direction */
	if( !mmap_read32(GPIO_ESPI_RST_DIR, &reg) )
	{
		reg &= ~0x800000;
		mmap_write32( GPIO_ESPI_RST_DIR, reg );
	}

	return(res);
}



/* Gracefully close the FDK */
uint32 apml_close_fdk(APML_DEV_CTL *ctl,int BMCInst)
{
    uint32	res;

    if(0)
    {
        BMCInst=BMCInst; /* -Wextra, fix for unused parameters */
    }

    res = user_platform_close(ctl);
	
    return(res);
}

uint32 apml_check_ESPI_RST(void)
{
	int i = 0;
	unsigned long reg;
    struct timeval tv;
	
	do
	{
		if( mmap_read32(GPIO_ESPI_RST_DV, &reg) )
		{
			TCRIT( "APML: Read ESPI_RST_L failed!\n" );
			return APML_FAILURE;
		}
		if( reg & 0x800000 )
			return APML_SUCCESS;
		/* sleep 3 ms */
		tv.tv_sec = 0;
		tv.tv_usec = 3000;
		while(sigwrap_select(0, NULL, NULL, NULL, &tv) < 0);
		if( i == 0 )
			TCRIT( "APML: ESPI_RST_L In Busy!\n" );
		i++;
	}while( i < MAX_ESPIRST_RETRIES);
	
	TCRIT( "APML: Wait ESPI_RST timeout error!\n" );
	
	return APML_FAILURE;
}


/* Core-specific read CPUID */
uint32 apml_read_cpuid(
	APML_DEV_CTL	*ctl,
	//uint8		proc,
	uint8       socketid,
	uint8       dieid,
	uint8		core,
	uint32		cpuid_func,
	uint32		*eax_val,
	uint32		*ebx_val,
	uint32		*ecx_val,
	uint32		*edx_val,
	int			BMCInst)
{
    uint8       thread_ndx;
    uint32      res;

    if(0)
    {
        dieid=dieid;
        BMCInst = BMCInst;
    }

    *eax_val = cpuid_func;
    *ebx_val = 0;
    *ecx_val = 0;
    *edx_val = 0;
    if ( (core*2) >= ctl->threads[socketid])
        return(APML_UNK_THREAD);
    // "core" range 00h - 3Fh, P0 use "thread_ndx" range 00h - 7Fh
    thread_ndx = core * 2;
    res = apml_driver_cpuid(socketid, thread_ndx, eax_val, ebx_val, ecx_val, edx_val);

    return(res);
}



/* Processor-wide read APML RMI version  */
uint32 apml_interface_version(
	APML_DEV_CTL	*ctl,
	//uint8		proc,
	uint8       socketid,
	uint8       dieid,
	uint8		*version,
    int BMCInst
    )
{
    uint8   proc_ndx;

    proc_ndx = socketid;
    if(0)
    {
        dieid=dieid;
        BMCInst=BMCInst; /* -Wextra, fix for unused parameters */
    }
    *version = ctl->rmi_rev[proc_ndx];

    return(APML_SUCCESS);
}



/* Core-specific read Machine Check Register (MCR) */
uint32 apml_read_mcr(
	APML_DEV_CTL	*ctl,
	//uint8		proc,
	uint8       socketid,
	uint8       dieid,
	uint8		core,
	uint32		mcr_addr,
	uint32		*reg_val_h,
	uint32		*reg_val_l,
	int			BMCInst)
{
    uint32		res, thread;
    uint64      buffer;

    if(0)
    {
        ctl=ctl;
        dieid=dieid;
        BMCInst = BMCInst;
    }

    thread = core * 2;
    res = apml_driver_read_msr(socketid, thread, mcr_addr, &buffer);

    *reg_val_l = buffer & 0xFFFFFFFF;
    *reg_val_h = (buffer >> 32) & 0xFFFFFFFF;

    return(res);
}



/* Processor-wide read 1 byte RMI Register */
uint32 apml_read_rmi_reg(
	APML_DEV_CTL	*dev,
	//uint8		proc,
	uint8       socketid,
	uint8       dieid,
	uint8		option,
	uint8		reg,
	uint8		*data,
	int			BMCInst)
{
    uint32	res;

    if(0)
    {
        dev=dev;
        dieid=dieid;
        BMCInst = BMCInst;
        option=option;
    }

    *data = 0;

    if( (res = apml_check_ESPI_RST()) != APML_SUCCESS )
        return(res);

    res = apml_driver_read_byte(socketid, reg, SBRMI, data);
    if (res != APML_SUCCESS)
    return(res);

    return(res);
}



/* Processor-wide write 1 byte RMI register */
uint32 apml_write_rmi_reg(
	APML_DEV_CTL	*dev,
	//uint8		proc,
	uint8       socketid,
	uint8       dieid,
	uint8		option,
	uint8		reg,
	uint8		data,
	int			BMCInst)
{
    uint32	res;

    if(0)
    {
        dev=dev;
        dieid=dieid;
        BMCInst = BMCInst;
        option=option;
    }

    if( (res = apml_check_ESPI_RST()) != APML_SUCCESS )
        return(res);

    res = apml_driver_write_byte(socketid, reg, SBRMI, data);

    return(res);
}


/* Processor-wide read 1 byte TSI Register */
uint32 apml_read_tsi_reg(
	APML_DEV_CTL	*dev,
	//uint8		proc,
	uint8       socketid,
	uint8       dieid,
	uint8		reg,
	uint8		*data,
	int			BMCInst)
{
    uint32	res;
    if(0)
    {
        dev=dev;
        dieid=dieid;
        BMCInst = BMCInst;
    }

    *data = 0;

    if( (res = apml_check_ESPI_RST()) != APML_SUCCESS )
        return(res);

    res = apml_driver_read_byte(socketid, reg, SBTSI, data);
    if (res != APML_SUCCESS)
        return(res);

    return(res);
}


/* Processor-wide write 1 byte tsi register */
uint32 apml_write_tsi_reg(
	APML_DEV_CTL	*dev,
	//uint8		proc,
	uint8       socketid,
	uint8       dieid,
	uint8		reg,
	uint8		data,
	int			BMCInst)
{
    uint32	res;

    if(0)
    {
        dev=dev;
        dieid=dieid;
        BMCInst = BMCInst;
    }

    if( (res = apml_check_ESPI_RST()) != APML_SUCCESS )
        return(res);

    res = apml_driver_write_byte(socketid, reg, SBTSI, data);

    return(res);
}



/* Processor-wide read DRAM Controller Command Throttle Register */
uint32 apml_read_dram_throttle(
	APML_DEV_CTL	*ctl,
	//uint8		proc,
	uint8       socketid,
	uint8       dieid,
	uint8		*bw_cap_cmd_throttle_mode,
	uint8		*bw_cap_en,
	int			BMCInst)
{
    uint32		res, output;

    if(0)
    {
        ctl=ctl;
        dieid=dieid;
        BMCInst = BMCInst;
    }

    // uniformly applied to whole socket, ignore dieid parameter
    res = apml_driver_read_mailbox(socketid, 0x0c, 0, &output);
    *bw_cap_cmd_throttle_mode = output;
    *bw_cap_en = 0;

    return(res);
}



/* Processor-wide write DRAM Controller Command Throttle Register */
uint32 apml_write_dram_throttle(
	APML_DEV_CTL	*ctl,
	//uint8		proc,
	uint8       socketid,
	uint8       dieid,
	uint8		bw_cap_cmd_throttle_mode,
	uint8		bw_cap_en,
	int			BMCInst)
{
    uint32		res;
    if(0)
    {
        ctl=ctl;
        dieid=dieid;
        BMCInst = BMCInst;
        bw_cap_en = bw_cap_en;
    }

    // uniformly applied to whole socket, ignore dieid parameter
    res = apml_driver_write_mailbox(socketid, 0x0d, bw_cap_cmd_throttle_mode);

    return(res);
}

uint32 apml_read_package_power_consumption(
    APML_DEV_CTL    *ctl,
        //uint8     proc,
        uint8       socketid,
        uint8       dieid,
        uint32       *pkg_pwr,
        int         BMCInst)
{
    uint32      res; 

    if(0)
    {
        ctl=ctl;
        dieid=dieid;
        BMCInst = BMCInst;
    }
    	
    //mailbox command 01 for read package power consumption
    res = apml_driver_read_mailbox(socketid, 0x01, 0, pkg_pwr);

    return(res);
}
    
uint32 apml_write_package_power_limit(
        APML_DEV_CTL    *ctl,
            //uint8     proc,
            uint8       socketid,
            uint8       dieid,
            uint32      pkg_pwr_limit,
            int         BMCInst)
{
    uint32      res; 

    if(0)
    {
        ctl=ctl;
        dieid=dieid;
        BMCInst = BMCInst;
    }

    res = apml_driver_write_mailbox(socketid, 0x02, pkg_pwr_limit); 

    return(res);
}


uint32 apml_read_boot_code_status(
    APML_DEV_CTL    *ctl,
        uint8       socketid,
        uint8       dieid,
        uint8       *boot_code_status,
        int         BMCInst)
{
    uint32      res; 
    uint8      reg_id = 0x80;
    uint8       reg_data = 0x0;
    int         i;
	
    if(0)
    {
        ctl=ctl;
        dieid=dieid;
        BMCInst = BMCInst;
    }

    //start from 0x80, Boot Code Status use 8 bytes to communicate.

    if( (res = apml_check_ESPI_RST()) != APML_SUCCESS )
        return(res);

    for (i=0; i<8; i++)
    {
        res = apml_driver_read_byte(socketid, reg_id, SBRMI, &reg_data);
        if (res != 0)
            return (res);
        else 
            *(boot_code_status + i) = (uint8) reg_data;

        reg_id++;
    }
    
    return(res);
}


uint32 apml_read_rmi_mailbox_service(
        APML_DEV_CTL    *dev,
        uint8           socketid,
        uint8           dieid,
        uint8           mboxcmd,
        uint32          *mboxdata,
        int             BMCInst)
{
    uint32      res; 

    if(0)
    {
        dev=dev;
        dieid=dieid;
        BMCInst = BMCInst;
    }

    res = apml_driver_read_mailbox(socketid, mboxcmd, 0, mboxdata);

    return(res);
}
    
uint32 apml_write_rmi_mailbox_service(
        APML_DEV_CTL    *dev,
        uint8           socketid,
        uint8           dieid,
        uint8           mboxcmd,
        uint32          mboxdata,
        int             BMCInst)
{
    uint32      res; 

    if(0)
    {
        dev=dev;
        dieid=dieid;
        BMCInst = BMCInst;
    }

    res = apml_driver_write_mailbox(socketid, mboxcmd, mboxdata); 
    
    return(res);
}

uint32 apml_recover_rmi(
        uint8           socketid,
        int             BMCInst)
{
    uint8 data;
    uint32 res;
    int retry=20;

    if(0)
    {
        BMCInst=BMCInst;
    }

    //Read SBTSI::Revision
	res = apml_driver_read_byte(socketid, SBTSI_REVISION, SBTSI, &data);
	if (res != APML_SUCCESS)
		return(res);

    if(APML_SB_TSI_REVISION_0_4 != data)
        return APML_FAILURE;

    //Read current value of SBTSI::Config before write
	res = apml_driver_read_byte(socketid, SBTSI_CONFIGURATION, SBTSI, &data);
	if (res != APML_SUCCESS)
		return(res);

    //SBI controller writes 1 to SBTSI::ConfigWr[RMISoftReset]
    data |= 1;
    res = apml_driver_write_byte(socketid, SBTSI_CONFIGURATION_WRITE, SBTSI, data);

    //BMC polls for SBTSI::Config[RMISoftReset] to clear. Once satisfied, BMC can start sending I2C/I3C commands to SB-RMI again.
    do {
	    res = apml_driver_read_byte(socketid, SBTSI_CONFIGURATION, SBTSI, &data);
	    if (res != APML_SUCCESS)
		    return(res);

        if (!(data & 1))
            break;

        usleep(1000);

    } while(retry--);

    //Read SBTSI::Revision
	res = apml_driver_read_byte(socketid, SBRMI_REVISION, SBRMI, &data);
	if (res != APML_SUCCESS)
		return(res);
    
    if(APML_SB_RMI_REVISION_2_0 != data)
    {
        TCRIT("APML SBRMI revision is not 2.0 after soft reset recovery");
        return APML_FAILURE;
    }

    return(res);    
}

uint32 apml_recover_tsi(
        uint8           socketid,
        int             BMCInst)
{
    uint8 data;
    uint32 res;
    int retry=20;

    if(0)
    {
        BMCInst=BMCInst;
    }

    //Read SBRMI::Revision
	res = apml_driver_read_byte(socketid, SBRMI_REVISION, SBRMI, &data);
	if (res != APML_SUCCESS)
		return(res);

    if(APML_SB_RMI_REVISION_2_0 != data)
        return APML_FAILURE;

    //Read current value of SBRMI::Control before write
	res = apml_driver_read_byte(socketid, SBRMI_CONTROL, SBRMI, &data);
	if (res != APML_SUCCESS)
		return(res);

    //SBI controller writes 1 to SBRMI::Control[TSISoftReset]
    data |= 2;
    res = apml_driver_write_byte(socketid, SBRMI_CONTROL, SBRMI, data);

    //BMC polls for TSISoftReset to clear. Once satisfied, BMC can start sending I2C/I3C commands to TSI again.
    do {
	    res = apml_driver_read_byte(socketid, SBRMI_CONTROL, SBRMI, &data);
	    if (res != APML_SUCCESS)
		    return(res);

        if (!(data & 1))
            break;

        usleep(1000);

    } while(retry--);

    //Read SBTSI::Revision
	res = apml_driver_read_byte(socketid, SBTSI_REVISION, SBTSI, &data);
	if (res != APML_SUCCESS)
		return(res);
    
    if(APML_SB_TSI_REVISION_0_4 != data)
    {
        TCRIT("APML SBTSI revision is not 0.4 after soft reset recovery");
        return APML_FAILURE;
    }

    return(res);    
}

/* **************************************************************************/
/*                                                                          */
/*                           Private Functions                              */
/*                                                                          */
/* **************************************************************************/

uint32 apml_writeread_rmi_mailbox_service(
        APML_DEV_CTL    *ctl,
            uint8       socketid,
            uint8       dieid,
        	uint8		mboxcmd,
            uint32      WRData,
            uint32      *Ptr_RDData,
            int         BMCInst)
{
    uint32      res; 

    if(0)
    {
        ctl=ctl;
        dieid=dieid;
        BMCInst = BMCInst;
    }

    res = apml_driver_read_mailbox(socketid, mboxcmd, WRData, Ptr_RDData);

    return(res);
}
