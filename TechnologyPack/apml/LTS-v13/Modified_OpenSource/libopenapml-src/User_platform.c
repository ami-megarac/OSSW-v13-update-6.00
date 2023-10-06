/*
 * User Platform Layer and OS Customization - V1.0
 *
 ******************************************************************************


===========================================================
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


#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/amd-apml.h>
#include <ctype.h>
#include <errno.h>

#include "libi2c.h"
#include "hal_hw.h"
#include "IPMIConf.h"
#include "Apml_fdk.h"
#include "dbgout.h"
#include "safesystem.h"

#define MAX_I2C_BUS_NAME_LEN 64

#define I3C4                     "1e7a6000.i3c4"
#define I3C5                     "1e7a7000.i3c5"
#define SBTSI_UNBIND             "/sys/bus/i3c/drivers/sbtsi_i3c/unbind"
#define SBRMI_UNBIND             "/sys/bus/i3c/drivers/sbrmi_i3c/unbind"
#define I3C_DRIVER_UNBIND        "/sys/bus/platform/drivers/ast2600-i3c-master/unbind"
#define I3C_DRIVER_BIND          "/sys/bus/platform/drivers/ast2600-i3c-master/bind"
#define I3C_OFFSET_FROM_SOCKETID 4
#define I2C_OFFSET_FROM_SOCKETID 2

/* IOCTL commands */
#define I3C_DEV_IOC_MAGIC	0x07

#define I3C_PRIV_XFER_SIZE(N)	\
	((((sizeof(struct i3c_ioc_priv_xfer)) * (N)) < (1 << _IOC_SIZEBITS)) \
	? ((sizeof(struct i3c_ioc_priv_xfer)) * (N)) : 0)

#define I3C_IOC_PRIV_XFER(N)	\
	_IOC(_IOC_READ|_IOC_WRITE, I3C_DEV_IOC_MAGIC, 30, I3C_PRIV_XFER_SIZE(N))

/**
 * struct i3c_ioc_priv_xfer - I3C SDR ioctl private transfer
 * @data: Holds pointer to userspace buffer with transmit data.
 * @len: Length of data buffer buffers, in bytes.
 * @rnw: encodes the transfer direction. true for a read, false for a write
 */
struct i3c_ioc_priv_xfer {
	__u64 data;
	__u16 len;
	__u8 rnw;
	__u8 pad[5];
};

/**
 * @fn get_i2c_bus_name
 * @brief This function is used by all below i2c apis to get bus name
 *
**/
uint32
get_i2c_bus_name (int bus_num, char *i2c_bus_name)
{
   /* Convert Bus number into corresponding i2c device */
   if ( snprintf(i2c_bus_name,64,"/dev/i2c-%d", bus_num) >= MAX_I2C_BUS_NAME_LEN )
   {
       TCRIT("Buffer Overflow \n");
       return -1;
   }

   return 0;
}

/* **************************************************************************/
/*                                                                          */
/*                          Private Static Defines                          */
/*                                                                          */
/* **************************************************************************/

const uint8 rmi_addr_lst[] = {
	0x3C,   //0x3c *2 ==0x78, SBI Address Encoding, socketID_packageID[2:0] : 000b 
	0x3D,   //0x7A, 001b
	0x3E,   //0x7C, 010b
	0x3F,   //0x7E, 011b
	0x38,   //0x70, 100b
	0x39,   //0x72, 101b
	0x3A,   //0x74, 110b
	0x3B    //0x76, 111b
};

/* Maps user_platform status messages to descriptions */
const USER_ERR_DESC err_desc[] = {
	{ APML_FAILURE,	"Error in Communicating with APML" },
};

const uint8 thread_enable_status_reg[] = {
    0x04,       //SBRMI_x04 Thread Enable Status Register 0, thread 0-7
    0x05,       //SBRMI_x05 Thread Enable Status Register 1, thread 8-15
    0x08,       //SBRMI_x08 Thread Enable Status Register 2, thread 16-23
    0x09,       //SBRMI_x09 Thread Enable Status Register 3, thread 24-31
    0x0A,       //SBRMI_x0A Thread Enable Status Register 4, thread 32-39
    0x0B,       //SBRMI_x0B Thread Enable Status Register 5, thread 40-47
    0x0C,       //SBRMI_x0C Thread Enable Status Register 6, thread 48-55
    0x0D,       //SBRMI_x0D Thread Enable Status Register 7, thread 56-63
    0x43,       //SBRMI_x43 Thread Enable Status Register 8, thread 64-71
    0x44,       //SBRMI_x44 Thread Enable Status Register 9, thread 72-79
    0x45,       //SBRMI_x45 Thread Enable Status Register 10, thread 80-87
    0x46,       //SBRMI_x46 Thread Enable Status Register 11, thread 88-95
    0x47,       //SBRMI_x47 Thread Enable Status Register 12, thread 96-103
    0x48,       //SBRMI_x48 Thread Enable Status Register 13, thread 104-111
    0x49,       //SBRMI_x49 Thread Enable Status Register 14, thread 112-119
    0x4A,       //sbrmi_x4a thread enable status register 15, thread 120-127
    0x91,       //sbrmi_x91 thread enable status register 16, thread 128-135
    0x92,       //SBRMI_x92 Thread Enable Status Register 17, thread 136-143
    0x93,       //SBRMI_x93 Thread Enable Status Register 18, thread 144-151
    0x94,       //SBRMI_x94 Thread Enable Status Register 19, thread 152-159
    0x95,       //SBRMI_x95 Thread Enable Status Register 20, thread 160-167
    0x96,       //SBRMI_x96 Thread Enable Status Register 21, thread 168-175
    0x97,       //SBRMI_x97 Thread Enable Status Register 22, thread 176-183
    0x98        //SBRMI_x98 Thread Enable Status Register 23, thread 184-191
};

/* **************************************************************************/
/*                                                                          */
/*          Mandatory Private User Platform Function Implementations        */
/*                                                                          */
/* **************************************************************************/


int apml_reload_i3c_driver(
	APML_DEV_CTL	*dev,
	uint8       socketid,
	int			BMCInst)
{
	int fd_reload;
    if(0)
    {
        BMCInst=BMCInst; /* -Wextra, fix for unused parameters */
        dev=dev;
    }

    if (socketid == 0)
    {
        fd_reload = sigwrap_open(I3C_DRIVER_UNBIND, O_WRONLY);
        if (fd_reload < 0)
        {
            TCRIT("Can't open %s", I3C_DRIVER_UNBIND);
            return -1;
        }
        write(fd_reload, I3C4, 14);
        sigwrap_close(fd_reload);

        fd_reload = sigwrap_open(I3C_DRIVER_BIND, O_WRONLY);
        if (fd_reload < 0)
        {
            TCRIT("Can't open %s", I3C_DRIVER_BIND);
            return -1;
        }
        write(fd_reload, I3C4, 14);
        sigwrap_close(fd_reload);
    }
    else if(socketid == 1)
    {
        fd_reload = sigwrap_open(I3C_DRIVER_UNBIND, O_WRONLY);
        if (fd_reload < 0)
        {
            TCRIT("Can't open %s", I3C_DRIVER_UNBIND);
            return -1;
        }

        write(fd_reload, I3C5, 14);
        sigwrap_close(fd_reload);

        fd_reload = sigwrap_open(I3C_DRIVER_BIND, O_WRONLY);
        if (fd_reload < 0)
        {
            TCRIT("Can't open %s", I3C_DRIVER_BIND);
            return -1;
        }
        write(fd_reload, I3C5, 14);
        sigwrap_close(fd_reload);
    }

    return 0;
}

uint32 MUX_IMX3112_init(
	APML_DEV_CTL	*dev,
	uint8       socketid,
	uint8		proc_addr,
	int			BMCInst)
{
    uint32	res=0;
            
    if(0)
    {
    	BMCInst=BMCInst; /* -Wextra, fix for unused parameters */
    }

    res = user_smbus_write_word(dev, socketid, proc_addr, 0x46, 0x01, BMCInst);
    if (res != APML_SUCCESS)
        TCRIT(" Quartz socketID:%x MUX_IMX3112 register_0x46 set 0x01 fail\n", socketid);

    res = user_smbus_write_word(dev, socketid, proc_addr, 0x40, 0x40, BMCInst);
    if (res != APML_SUCCESS)
        TCRIT(" Quartz socketID:%x MUX_IMX3112 register_0x40 set 0x40 fail\n", socketid);

    res = user_smbus_write_word(dev, socketid, proc_addr, 0x41, 0x40, BMCInst);
    if (res != APML_SUCCESS)
        TCRIT(" Quartz socketID:%x MUX_IMX3112 register_0x41 set 0x40 fail\n", socketid);

    return res;
}

/*-------------------------------------------------------------------
**@fn: i3c_write_IMX3112
**@brief: write IMX3112 register in I3C mode
**@parameter:
**  device:         the IMX3112 i3c device node blow the /dev
**  addr1 & addr2:  according IMX3112-IML3112 datasheet, the address need two byes
**  value:          the value want to write
**@return 0 for success 
**    -1 for fail
--------------------------------------------------------------------*/
int i3c_write_IMX3112(char *device, uint8_t addr1, uint8_t addr2, uint8_t value)
{
    struct i3c_ioc_priv_xfer *xfers;
    int i3cdev, nxfers, i, ret = 0;
    uint8_t *tmp;

    i3cdev = sigwrap_open(device, O_RDWR);
    if (i3cdev < 0)
    {
        TCRIT("APML open IMX3112 MUX fail");
        return -1;
    }

    nxfers = 1; //write one regitser at one time
    xfers = (struct i3c_ioc_priv_xfer *)calloc(nxfers, sizeof(*xfers));
    if (!xfers)
    {
        sigwrap_close(i3cdev);
        TCRIT("APML i3c writing alloc memory fail");
        return -1;
    }

    tmp = (uint8_t *)calloc(3, sizeof(uint8_t));
    if (!tmp)
    {
        sigwrap_close(i3cdev);
        free(xfers);
        TCRIT("APML i3c writing alloc memory fail");
        return -1;
    }

    tmp[0] = addr1;
    tmp[1] = addr2;
    tmp[2] = value;

    xfers->len = 3;
    xfers->data = (uintptr_t)tmp;

    if (ioctl(i3cdev, I3C_IOC_PRIV_XFER(nxfers), xfers) < 0)
    {
        TCRIT("APML IMX3112 MUX write data fail");
        ret = -1;
    }

    for (i = 0; i < nxfers; i++)
        free((void *)(uintptr_t)xfers[i].data);
    free(xfers);

    sigwrap_close(i3cdev);

    return ret;
}

uint32 MUX_IMX3112_init_over_I3C(
	APML_DEV_CTL	*dev,
	uint8       socketid,
	int			BMCInst)
{
    char imx3112[32];

    if(0)
    {
    	BMCInst=BMCInst; /* -Wextra, fix for unused parameters */
        dev=dev;
    }
    
    safe_snprintf(imx3112, 32, "/dev/i3c-%d-4cc00000000", socketid+I3C_OFFSET_FROM_SOCKETID);

    if(0 != i3c_write_IMX3112(imx3112, 0x46, 0x00, 0x01))
    {
        printf("APML write IMX3112 MUX register 0x46 fail\n");
        return -1;
    }
    if(0 != i3c_write_IMX3112(imx3112, 0x40, 0x00, 0x40))
    {
        printf("APML write IMX3112 MUX register 0x46 fail\n");
        return -1;
    }
    if(0 != i3c_write_IMX3112(imx3112, 0x41, 0x00, 0x40))
    {
        printf("APML write IMX3112 MUX register 0x46 fail\n");
        return -1;
    }
	return 0;
}

uint32 mount_apml_driver(APML_DEV_CTL *dev,int BMCInst)
{
    int res=0, sys_ret, accessiable_cpu = 0;
    uint8 socket;

    //Unmount SBTSI and SBRMI driver
    sys_ret = safe_system("modprobe -r apml_sbrmi");
    if (sys_ret != 0)
    {
        TCRIT("apml_sbrmi driver remove fail");
    }
    sys_ret = safe_system("modprobe -r apml_sbtsi");
    if (sys_ret != 0)
    {
        TCRIT("apml_sbtsi driver remove fail");
    }

    //Set APML MUX and reload I3C driver
    for (socket=0; socket < APML_MAX_PROCS; socket++)
    {
#ifdef CONFIG_SPX_FEATURE_APML_OVER_I2C
        res = MUX_IMX3112_init(dev, socket, 0x70, BMCInst);
        if (res != APML_SUCCESS)
        {
            TCRIT("MUX_IMX3112 initialize fail, BMC can't access CPU%x\n", socket);
            continue;
        }
#else
        //Reload I3C driver to let IMX3112 MUX be available
        res = apml_reload_i3c_driver(dev, socket, BMCInst);
        if (res != APML_SUCCESS)
        {
            TCRIT("I3C driver reload fail, BMC can't access CPU%x over I3C\n", socket);
            continue;
        }
        res = MUX_IMX3112_init_over_I3C(dev, socket, BMCInst);
        if (res != APML_SUCCESS)
        {
            TCRIT("MUX_IMX3112 initialize fail, BMC can't access CPU%x\n", socket);
            continue;
        }
        //Reload I3C driver to let APML device be available 
        res = apml_reload_i3c_driver(dev, socket, BMCInst);
        if (res != APML_SUCCESS)
        {
            TCRIT("I3C driver reload fail, BMC can't access CPU%x over I3C\n", socket);
            continue;
        }
#endif
        accessiable_cpu++;
    }

    if (0 == accessiable_cpu)
    {
        TINFO("IMX3112 MUX setting fail");
        return -1;
    }

    TINFO("IMX3112 MUX setting done");

    //Mount SBTSI and SBRMI driver
    sys_ret = safe_system("modprobe apml_sbtsi");
    if (sys_ret != 0)
    {
        TCRIT("apml_sbtsi driver probe fail");
        return -1;
    }
    sys_ret = safe_system("modprobe apml_sbrmi");
    if (sys_ret != 0)
    {
        TCRIT("apml_sbrmi driver probe fail");
        return -1;
    }

    if(0 != touch(FILE_APMLDRIVER))
    {
            TINFO("create %s fail", FILE_APMLDRIVER);
    }

    return APML_SUCCESS;
}

void unmount_apml_driver()
{
    int sys_ret;

    sys_ret = safe_system("modprobe -r apml_sbrmi");
    if (sys_ret != 0)
    {
        TCRIT("apml_sbrmi driver remove fail");
    }
    sys_ret = safe_system("modprobe -r apml_sbtsi");
    if (sys_ret != 0)
    {
        TCRIT("apml_sbtsi driver remove fail");
    }
    TINFO("Unmount APML driver");
}

/* Called at FDK initialization to configure the FDK and read the current
 * processor status
 *
 * A robust implementation should read RMI 0x41 register and set the 
 * dev->rmi_thread_target.  The default works for now however. */
uint32 user_platform_init(APML_DEV_CTL *dev, int BMCInst)
{
	uint8			tmp8, sectmp8;
	uint32			proc, i, j, k, socket, res, package;
	
    USER_PLATFORM_DATA	*platform;

	platform = &dev->platform;
    struct stat CPUnumber, APMLstat;
    int usable_processor_exist = 0;
    bool available_socket[APML_MAX_PROCS];
    
    for (i=0; i < APML_MAX_PROCS; i++)
        available_socket[i] = 0;

    TINFO("Start to initialize APML user platform");

	/* Initialize platform structure: slave address and bus number */
    for (socket=0; socket < APML_MAX_PROCS; socket++)
    { 
        if (socket == 0)
        {
            for (package=0; package < APML_MAX_PACKAGES; package++){
		        platform->addr[socket][package] = rmi_addr_lst[package];
                //printf("platform->addr[%d][%d] = 0x%x\n",socket, package, platform->addr[socket][package]);
            }
        }
        else if (socket == 1)
        {
            for (package=0; package < APML_MAX_PACKAGES; package++){
                platform->addr[socket][package] = rmi_addr_lst[package + 4]; 
                //printf("platform->addr[%d][%d] = 0x%x\n",socket,package, platform->addr[socket][package]);
            }
        }
    }

    platform->last_addr_val = 0;	

    /* Iterate each possible processor in the platform.*/
    for (proc = 0; proc < APML_MAX_PROCS; proc++)
    {
        /* Configure each proc to have 1 thread to send commands ,then decrese back after getting thread numbers*/
        dev->threads[proc] = 1;

        /* Read the APML version from SBRMI_x00 */
        res = apml_read_rmi_reg(dev, proc, 0, PROC_USE_RMI, 0x00, &tmp8, BMCInst);//using first Die should be ok
        if (res == APML_SUCCESS)
        {
            dev->rmi_rev[proc] = tmp8;

            //Check the available proccessor its APML version
            if (APML_SB_RMI_REVISION_2_0 != tmp8)
            {
                TCRIT("APML socket %d, APML version from SBRMI_x00 == 0x%x\n", proc,tmp8);
                available_socket[proc] = 0;
                return(APML_BAD_RMI_VERSION);
            }

            /* Check the maximum number of threads present from SBRMI_x4E for low byte and SBRMI_x4F for high byte */
            res = apml_read_rmi_reg(dev, proc, 0, PROC_USE_RMI, 0x4F, &tmp8, BMCInst);
            if(res == APML_SUCCESS)
            {
                dev->rmi_thread_target[proc] = tmp8 << 8;
                //INFO("APML socket %d, Max Threads number reg SBRMI_x4F == 0x%x\n", proc,tmp8);
            }
            else
            {
                TCRIT("APML: socket %d, Read Max Threads number or SBRMI_x4F fail, res==0x%x\n", proc,res);
                return res;
            }

            res = res + apml_read_rmi_reg(dev, proc, 0, PROC_USE_RMI, 0x4E, &sectmp8, BMCInst);
            if(res == APML_SUCCESS)
            {
                dev->rmi_thread_target[proc] = dev->rmi_thread_target[proc] + sectmp8;
                //INFO("APML socket %d, Max Threads number reg SBRMI_x4E == 0x%x\n", proc,tmp8);
            }
            else
            {
                TCRIT("APML: socket %d, Read Max Threads number SBRMI_x4E fail, res==0x%x\n", proc,res);
                return res;
            }

           // TINFO("APML socket %d, Max Threads number == 0x%x\n", proc, dev->rmi_thread_target[proc]);
            if( 0 == tmp8 && 0 == sectmp8)
            {   //CPU is not ready
                TCRIT("APML: socket %d Max Threads is 0x00, CPU is not ready", proc);
                return (APML_CORE_NA);
            }

            /* by each packages, check threads enable status*/
            for (package=0; package < APML_MAX_PACKAGES; package++)
            {
                /* check thread enabled status */
                for(k=0;k<sizeof(thread_enable_status_reg);k++)
                {
                    /* Read thread enabled status */
                    res = apml_read_rmi_reg(dev, proc, package, PROC_USE_RMI, thread_enable_status_reg[k], &tmp8,BMCInst); 
                    if (res == APML_SUCCESS)
                    {
                        //printf("socket %d, thread enable status reg SBRMI_x%hhx == 0x%x\n", proc,thread_enable_status_reg[k],tmp8);
                        /* Store # of threads, 8 threads per reg */
                        for (j=1; j < 256; j = j * 2)
                        {
                            if ((tmp8 & j) == j)
                            {
                                dev->threads[proc] = dev->threads[proc] + 1;
                            }
                        }
                        //printf("read thread enable status reg, dev->threads[%d] == %d\n", proc,dev->threads[proc]);
                    }
                }
                if (res != APML_SUCCESS)
                {
                    TCRIT(" APML initial Mailbox: write 0x3F with 0x80 fail, socket:%d, package:%d\n", proc, package);
                }
            }
        }
        else
        {
            TCRIT("APML initial CPU%d read version fail", proc);
        }

        dev->threads[proc]--; /* Remove the temp thread count we added at the start for sending commands */

        //TINFO("APML initialization: APML_DEV_CTL ctl->threads[%d] == %d", proc,dev->threads[proc]);

        if (res != APML_SUCCESS)
        {
            dev->threads[proc] = 0;
            for (package=0; package<APML_MAX_PACKAGES;package++)
                platform->addr[proc][package] = 0;
        }
        
        if(0 < dev->threads[proc])
        {
            usable_processor_exist++;
            available_socket[proc] = 1;
        }
    }

    //Check the threads situation about the available socket
    for( proc=0; proc<APML_MAX_PROCS; proc++)
    {
        if (0 == available_socket[proc])
        {
            continue;
        }

        /* Should check if both sockets contain corect thread number*/
    	if(0 == dev->threads[proc])
    	{
            TINFO("APML socket ready threads[%d]==0", proc);
    		//return (APML_CORE_NA);
    	}
        else
        {
            TINFO("APML: CPU%d ready", proc);
        }

    	if( dev->rmi_thread_target[proc] != dev->threads[proc] )
    	{
            TINFO("APML socket threads[%d] (%d) != (%d) RMI Core Number", proc, dev->threads[proc], dev->rmi_thread_target[proc]);
    		//return (APML_CORE_NA);
    	}
    }
	
    if(0 == usable_processor_exist)
    {
        TINFO("APML: CPU not ready");
        return (APML_CORE_NA);
    }
    else if(1 == usable_processor_exist)
    {
        if (0 != stat("/var/1CPU", &CPUnumber))
        {
            if(0 == touch("/var/1CPU"))
                TINFO("APML: create /var/1CPU");
        }
    }
    else if(2 == usable_processor_exist)
    {
        if (0 != stat("/var/2CPU", &CPUnumber))
        {
            if(0 == touch("/var/2CPU"))
                TINFO("APML: create /var/2CPU");
        }
    }
    else
    {
        TCRIT("Error about Usable processor number\n");
        return (APML_CORE_NA);
    }
    
    if (0 != stat(FILE_APMLREADY, &APMLstat))
    {
        if(0 == touch(FILE_APMLREADY))
        {
            TINFO("APML: create /var/APML_READ");
            TINFO("APML initial done!");
        }
    }

    return (APML_SUCCESS);
}



/* Gracefully closes the FDK */
uint32 user_platform_close(APML_DEV_CTL *ctl)
{
	uint32 counter;
    uint32 j;
    int res_remove;

	/* Invalidate all of the processors */
    for (counter = 0; counter < APML_MAX_PROCS; counter++)
    {
        for (j = 0; j < APML_MAX_PACKAGES; j++)
        {
            ctl->platform.addr[counter][j] = 0;
        }
        ctl->threads[counter] = 0;
    }

    unmount_apml_driver();

    res_remove = remove(FILE_APMLREADY);
    if (0 == res_remove)
    {
        printf("Remove %s success\n", FILE_APMLREADY);
    }
    else
    {
        printf("Remove %s fail\n", FILE_APMLREADY);
    }

    res_remove = remove(FILE_APMLDRIVER);
    if (0 == res_remove)
    {
        printf("Remove %s success\n", FILE_APMLDRIVER);
    }
    else
    {
        printf("Remove %s fail\n", FILE_APMLDRIVER);
    }

    return (APML_SUCCESS);
}



/* SMBus write word - transmits data[7:0] first, data[15:8] second */
uint32 user_smbus_write_word(
	APML_DEV_CTL	*dev,
	uint8       socketid,
	uint8		proc_addr,
	uint8		reg,
	uint8		data,
	int			BMCInst)
{

    uint32	res= APML_SUCCESS;
    uint8  writeBuf[2];
    writeBuf[0] = reg;
    writeBuf[1] = data;
    ssize_t   uerr;
    size_t wr_len=sizeof(writeBuf);
    char i2c_bus_name[MAX_I2C_BUS_NAME_LEN];

    if(0)
    {
    	BMCInst=BMCInst; /* -Wextra, fix for unused parameters */
        dev=dev;
    }

    if(g_HALI2CHandle[HAL_I2C_MW] != NULL)
    {
        res = get_i2c_bus_name(socketid+I2C_OFFSET_FROM_SOCKETID, i2c_bus_name);
        uerr = ((ssize_t(*)(char *,u8,u8 *,size_t))g_HALI2CHandle[HAL_I2C_MW]) (i2c_bus_name, proc_addr, (uint8*) writeBuf, wr_len);
        if(0 > uerr)
        {
            printf("APML I2C write fail\n");
            res = APML_FAILURE;
        }
    }
    else
    {
        res=APML_FAILURE;
    }

    return (res);
}

/* READ MODE */
#define READ_MODE       1
/*WRITE MODE */
#define WRITE_MODE      0
#define THREAD_MASK 0xFFFF

int sbrmi_xfer_msg(uint8_t socket_num, char *filename, struct apml_message *msg)
{
    int fd = 0, ret = 0;
    char dev_file[12];

    safe_snprintf(dev_file, 12, "/dev/%s%d", filename, socket_num);

    fd = sigwrap_open(dev_file, O_RDWR);
    if (fd < 0)
    {
        printf("Fail to open /dev/%s%d\n",  filename, socket_num);
        return -1;
    }

    if (ioctl(fd, SBRMI_IOCTL_CMD, msg) < 0)
        ret = errno;

    sigwrap_close(fd);

    if (ret == EPROTOTYPE)
    {
        ret = msg->fw_ret_code;
    }

    return ret;
}

int apml_driver_read_byte(uint8_t soc_num,
				uint8_t reg_offset,
				char *file_name,
				uint8_t *buffer)
{
	struct apml_message msg = {0};
	int ret;

	/* NULL Pointer check */
	if (!buffer)
		return -1;
	/* Readi/write register command is 0x1002 */
	msg.cmd = 0x1002;
	/* Assign register_offset to msg.data_in[0] */
	msg.data_in.reg_in[0] = reg_offset;
	/* Assign 1  to the msg.data_in[7] for the read operation */
	msg.data_in.reg_in[7] = 1;

	ret = sbrmi_xfer_msg(soc_num, file_name, &msg);
	if (ret)
		return ret;

	*buffer = msg.data_out.reg_out[0];

	return 0;
}

int apml_driver_write_byte(uint8_t soc_num,
				 uint8_t reg_offset,
				 char *file_name,
				 uint8_t value)
{
	struct apml_message msg = {0};

	/* Read/Write register command is 0x1002 */
	msg.cmd = 0x1002;
	/* Assign register_offset to msg.data_in[0] */
	msg.data_in.reg_in[0] = reg_offset;

	/* Assign value to write to the data_in[4] */
	msg.data_in.reg_in[4] = value;

	/* Assign 0 to the msg.data_in[7] */
	msg.data_in.reg_in[7] = 0;

	return sbrmi_xfer_msg(soc_num, file_name, &msg);
}

/*
 * For a Mailbox write:
 *
 * Write 0x3f to an 0x80, Send the command to 0x38 and write the desired
 * value to 0x39 through 3C. The arguments of logical core numbers,
 * the % of DRAM throttle, NBIO Quadrant, CCD and CCX instances are all
 * written to 0x39.
 * The answer for our mailbox request is placed on registers 0x31-0x34.
 */
int apml_driver_write_mailbox(uint8_t soc_num,
                                    uint32_t cmd, uint32_t data)
{
	struct apml_message msg = {0};

	msg.cmd = cmd;
	msg.data_in.mb_in[0] = data;

	msg.data_in.mb_in[1] = (uint32_t)WRITE_MODE << 24;

	return sbrmi_xfer_msg(soc_num, SBRMI, &msg);
}

/*
 * The answer for our mailbox request is placed on registers 0x31-0x34
 */
int apml_driver_read_mailbox(uint8_t soc_num,
                                   uint32_t cmd, uint32_t input, uint32_t *buffer)
{
	struct apml_message msg = {0};
	int ret = 0;

	/* NULL pointer check */
	if (!buffer)
		return -1;

	msg.cmd = cmd;
	msg.data_in.mb_in[0] = input;

	msg.data_in.mb_in[1] = (uint32_t)READ_MODE << 24;
	ret = sbrmi_xfer_msg(soc_num, SBRMI, &msg);
	if (ret)
		return ret;

	*buffer = msg.data_out.mb_out[0];
	return 0;
}

int apml_driver_read_msr(uint8_t soc_num,
			       uint32_t thread, uint32_t msraddr,
			       uint64_t *buffer)
{
	struct apml_message msg = {0};
	int ret;

	/* NULL pointer check */
	if (!buffer)
		return -1;

	/* cmd for MCAMSR is 0x1001 */
	msg.cmd = 0x1001;
	msg.data_in.cpu_msr_in = msraddr;

	thread &= THREAD_MASK;
	/* Assign thread number to data_in[4:5] */
	msg.data_in.cpu_msr_in = msg.data_in.cpu_msr_in
				 | ((uint64_t)thread << 32);

	/* Assign 7 byte to READ Mode */
	msg.data_in.reg_in[7] = 1;
	ret = sbrmi_xfer_msg(soc_num, SBRMI, &msg);
	if (ret)
		return ret;

	*buffer = msg.data_out.cpu_msr_out;

	return 0;
}

int apml_driver_cpuid(uint8_t soc_num, uint32_t thread,
			    uint32_t *eax, uint32_t *ebx,
			    uint32_t *ecx, uint32_t *edx)
{
	uint32_t fn_eax, fn_ecx;
	int ret;

	fn_eax = *eax;
	fn_ecx = *ecx;

	ret = apml_driver_cpuid_eax(soc_num, thread, fn_eax, fn_ecx, eax);
	if (ret)
		return ret;

	ret = apml_driver_cpuid_ebx(soc_num, thread, fn_eax, fn_ecx, ebx);
	if (ret)
		return ret;

	ret = apml_driver_cpuid_ecx(soc_num, thread, fn_eax, fn_ecx, ecx);
	if (ret)
		return ret;

	return apml_driver_cpuid_edx(soc_num, thread, fn_eax, fn_ecx, edx);
}

static int apml_driver_cpuid_fn(uint8_t soc_num, uint32_t thread,
				      uint32_t fn_eax, uint32_t fn_ecx,
				      uint8_t mode, uint32_t *value)
{
	struct apml_message msg = {0};
	uint8_t ext = 0, read_reg = 0;
	int ret;

	if (!value)
	{
		printf("APML: CPUID fn parameter value is null\n");
		return -1;
	}
	/* cmd for CPUID is 0x1000 */
	msg.cmd = 0x1000;
	msg.data_in.cpu_msr_in = fn_eax;

	/* Assign thread number to data_in[4:5] */
	msg.data_in.cpu_msr_in = msg.data_in.cpu_msr_in
				 | ((uint64_t)thread << 32);

	/* Assign extended function to data_in[6][4:7] */
	if (mode == EAX || mode == EBX)
		/* read eax/ebx */
		read_reg = 0;
	else
		/* read ecx/edx */
		read_reg = 1;

	ext = (uint8_t)fn_ecx;
        ext = ext << 4 | read_reg;
        msg.data_in.cpu_msr_in = msg.data_in.cpu_msr_in | ((uint64_t) ext << 48);
	/* Assign 7 byte to READ Mode */
	msg.data_in.reg_in[7] = 1;
        ret = sbrmi_xfer_msg(soc_num, SBRMI, &msg);
        if (ret)
                return ret;

	if (mode == EAX || mode == ECX)
		/* Read low word/mbout[0] */
		*value = msg.data_out.mb_out[0];
	else
		/* Read high word/mbout[1] */
		*value = msg.data_out.mb_out[1];

	return 0;
}


/*
 * CPUID functions returning a single datum
 */
int apml_driver_cpuid_eax(uint8_t soc_num,
				uint32_t thread, uint32_t fn_eax,
				uint32_t fn_ecx, uint32_t *eax)
{
	return apml_driver_cpuid_fn(soc_num, thread, fn_eax, fn_ecx,
				 EAX, eax);
}

int apml_driver_cpuid_ebx(uint8_t soc_num,
				uint32_t thread, uint32_t fn_eax,
				uint32_t fn_ecx, uint32_t *ebx)
{
	return apml_driver_cpuid_fn(soc_num, thread, fn_eax, fn_ecx,
				 EBX, ebx);
}

int apml_driver_cpuid_ecx(uint8_t soc_num,
				uint32_t thread, uint32_t fn_eax,
				uint32_t fn_ecx, uint32_t *ecx)
{
    return apml_driver_cpuid_fn(soc_num, thread, fn_eax, fn_ecx,
				 ECX, ecx);
}

int apml_driver_cpuid_edx(uint8_t soc_num,
				uint32_t thread, uint32_t fn_eax,
				uint32_t fn_ecx, uint32_t *edx)
{
    return apml_driver_cpuid_fn(soc_num, thread, fn_eax, fn_ecx,
				 EDX, edx);
}
