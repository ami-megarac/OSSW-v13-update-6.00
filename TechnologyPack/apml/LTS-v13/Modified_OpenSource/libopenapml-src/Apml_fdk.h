/*
 * Master include file for the APML FDK - V1.0
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


#ifndef APML_FDK_H
#define APML_FDK_H 1

#include <linux/types.h>
#include <stdint.h>
#include <linux/amd-apml.h>
#include "coreTypes.h"
#include "EINTR_wrappers.h"

#define FILE_APMLREADY		"/var/APML_READY"
#define FILE_APMLDRIVER		"/var/APML_DRIVER"

//SBRMI Register Number Define
#define SBRMI_REVISION              0x00
#define SBRMI_CONTROL               0x01
#define SBRMI_STATUS                0x02
#define SBRMI_READSIZE              0x03
#define SBRMI_SOFTWAREINTERRUPT     0x40
#define SBRMI_THREADNUMBER          0x41
#define SBRMI_THREAD128CS           0x4B
#define SBRMI_RASSTATUS             0x4C
#define SBRMI_THREADNUMBERLOW       0x4E
#define SBRMI_THREADNUMBERHIGH      0x4F

//SBTSI Register Number Define
#define SBTSI_CPUTEMPINT            0x01
#define SBTSI_STATUS                0x02
#define SBTSI_CONFIGURATION         0x03
#define SBTSI_UPDATERATE            0x04
#define SBTSI_CONFIGURATION_WRITE   0x09
#define SBTSI_MANUFID               0xFE
#define SBTSI_REVISION              0xFF

/* **************************************************************************/
/*                                                                          */
/*             Static Defines Used By User Platform and APML FDK            */
/*                                                                          */
/* **************************************************************************/

/* Static maximum processors APML supports natively */
//#define APML_MAX_PROCS	2 // Natively it can support upto 8 Proc
#ifdef CONFIG_SPX_FEATURE_NUMBER_OF_APML_DEVICES
#define APML_MAX_PROCS	(CONFIG_SPX_FEATURE_NUMBER_OF_APML_DEVICES) // Set in PRJ
#else
#define APML_MAX_PROCS	2 // Natively it can support upto 2 Sockets  for Diesel platform
#endif

#define APML_MAX_PACKAGES  1 //ROME it can support upto 1 Packages  every Socket
#define APML_MAX_DIES   (APML_MAX_PACKAGES * 4) //ROME it can support upto 4 DIEs every Socket
#define APML_MAX_CORES  (APML_MAX_DIES * 16) //ROME it can support up to 64 cores every Packages
#define APML_MAX_THREADS (APML_MAX_CORES * 2) //ROME it can support up to 128 threads every Packages


/* Option value to user_get_proc_addr() in user_platform */
#define PROC_USE_RMI	0x00	/* Return RMI adddress */
#define PROC_USE_TSI	0x01	/* Return TSI adddress rather than RMI */

/* **************************************************************************/
/*                                                                          */
/*                   Mandatory Platform Specific Definitions                */
/*                                                                          */
/* **************************************************************************/


/* user_platform_data is mandatory for the FDK but the contents are 
 * completely user defined. */
typedef struct user_platform_data {
	uint8	addr[APML_MAX_PROCS][APML_MAX_PACKAGES];
	uint8	last_addr_val;
	int32	apml_smbus_number[APML_MAX_PROCS];
	;
} USER_PLATFORM_DATA;

//Revision
#define APML_SB_RMI_REVISION_1_0        0x02 
#define APML_SB_RMI_REVISION_1_1        0x03 
#define APML_SB_RMI_REVISION_2_0        0x20

#define APML_SB_TSI_REVISION_0_4        0x04


/* Platform specific error codes 
 * These errors should be from 300-399 for maximal compatiblity with other
 * projects. */
#define USER_IO_ERROR		300
#define USER_REMOTE_IO_ERROR	301
#define USER_GEN_ERROR		302
#define USER_UNK_ERROR		303
#define USER_BUS_OPEN_FAIL	304


/* Maps platform specific error codes to a description */
typedef struct user_err_desc {
	uint32		code;
	const CHAR	*desc;
} USER_ERR_DESC;



/* **************************************************************************/
/*                                                                          */
/*                             Static Defines                               */
/*                                                                          */
/* **************************************************************************/

/* Boolean values */
#define APML_TRUE	0x01
#define APML_FALSE	0x00


/* Initialization and configuration */
#define APML_TRACE_OFF		APML_FALSE
#define APML_TRACE_ON		APML_TRUE
#define APML_DEBUG_OFF		APML_FALSE
#define APML_DEBUG_ON		APML_TRUE
#define APML_CMD_RETRIES	20
#define LOAD_ADDRESS_COMMAND	0x81
#define DEFAULT_RMI_CORE	0x00
#define CORE_BRCST		0xFF


/* Status codes */
#define APML_SUCCESS		0
#define APML_FAILURE		-1
#define APML_BAD_PROC_NDX	101
#define APML_UNK_PROC_NUM	102
#define APML_PEC_FAIL		103
#define APML_RCV_LEN_MATCH	104
#define APML_CMD_TMO		106
#define APML_RSP_TMO		107
#define APML_SBI_BUSY		108
#define APML_HDT_BUSY		109
#define APML_CORE_NA		110
#define APML_UNK_CMD		111
#define APML_INV_RD_LEN		112
#define APML_XS_DATA_LEN	113
#define APML_UNK_THREAD		114
#define APML_CMD_NA		115
#define APML_UNK_RESPONSE	116
#define APML_NO_RESPONSE	117
#define APML_BAD_RMI_VERSION	118
#define APML_ILL_BCAST		119
#define APML_ABORT_TMO		120
#define APML_NOT_SUPPORTED	199
#define APML_BAD_DIE_NDX	121

/* ESPI_RST_L in Genoa is GPIOW[7] */
#define GPIO_BASE           0x1E780000
#define GPIO_ESPI_RST_DV     (GPIO_BASE + 0x88)    //GPIO_U/V/W/X Data Value Register
#define GPIO_ESPI_RST_DIR    (GPIO_BASE + 0x8C)    //GPIO_U/V/W/X Direction Register


/* **************************************************************************/
/*                                                                          */
/*                              Static Structures                           */
/*                                                                          */
/* **************************************************************************/


/* Main APML state and configuration structure */
typedef struct apml_ctl_struct {
	uint8			rmi_rev[APML_MAX_PROCS];
	uint16			rmi_thread_target[APML_MAX_PROCS];
	uint16			threads[APML_MAX_PROCS];
	USER_PLATFORM_DATA	platform;
} APML_DEV_CTL;



/* **************************************************************************/
/*                                                                          */
/*                      Public APML Function Definitions                    */
/*                                                                          */
/* **************************************************************************/


extern uint32 apml_read_cpuid(
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
	int			BMCInst);

extern uint32 apml_interface_version(
	APML_DEV_CTL	*ctl,
	//uint8		proc,
	uint8       socketid,
	uint8       dieid,
	uint8		*version,
    int         BMCInst);

extern uint32 apml_read_mcr(
	APML_DEV_CTL	*ctl,
	//uint8		proc,
	uint8       socketid,
	uint8       dieid,
	uint8		core,
	uint32		mcr_addr,
	uint32		*reg_val_h,
	uint32		*reg_val_l,
	int			BMCInst);

extern uint32 apml_read_rmi_reg(
	APML_DEV_CTL	*dev,
	//uint8		proc,
	uint8       socketid,
	uint8       dieid,
	uint8		option,
	uint8		reg,
	uint8		*data,
	int 			BMCInst);

extern uint32 apml_write_rmi_reg(
	APML_DEV_CTL	*dev,
	//uint8		proc,
	uint8       socketid,
	uint8       dieid,
	uint8		option,
	uint8		reg,
	uint8		data,
	int			BMCInst);

extern uint32 apml_read_tsi_reg(
	APML_DEV_CTL	*dev,
	//uint8		proc,
	uint8       socketid,
	uint8       dieid,
	uint8		reg,
	uint8		*data,
	int			BMCInst);

extern uint32 apml_write_tsi_reg(
	APML_DEV_CTL	*dev,
	//uint8		proc,
	uint8       socketid,
	uint8       dieid,
	uint8		reg,
	uint8		data,
	int			BMCInst);

extern uint32 apml_read_dram_throttle(
        APML_DEV_CTL    *ctl,
        //uint8		proc,
	    uint8           socketid,
	    uint8           dieid,
        uint8           *bw_cap_cmd_throttle_mode,
        uint8           *bw_cap_en,
        int			    BMCInst);

extern uint32 apml_write_dram_throttle(
        APML_DEV_CTL    *ctl,
        //uint8		proc,
	    uint8           socketid,
	    uint8           dieid,
        uint8           bw_cap_cmd_throttle_mode,
        uint8           bw_cap_en,
        int			    BMCInst);

extern uint32 apml_read_package_power_consumption(
    APML_DEV_CTL    *ctl,
        //uint8     proc,
        uint8       socketid,
        uint8       dieid,
        uint32       *pkg_pwr,
        int         BMCInst);

extern uint32 apml_write_package_power_limit(
        APML_DEV_CTL    *ctl,
            //uint8     proc,
            uint8       socketid,
            uint8       dieid,
            uint32      pkg_pwr_limit,
            int         BMCInst);

extern uint32 apml_writeread_rmi_mailbox_service(
        APML_DEV_CTL    *ctl,
            uint8       socketid,
            uint8       dieid,
        	uint8		mboxcmd,
            uint32      WRData,
            uint32      *Ptr_RDData,
            int         BMCInst);

extern uint32 apml_read_boot_code_status(
        APML_DEV_CTL    *ctl,
        uint8       socketid,
        uint8       dieid,
        uint8       *boot_code_status,
        int         BMCInst);

extern uint32 apml_init_fdk(APML_DEV_CTL *ctl,int BMCInst);

extern uint32 apml_close_fdk(APML_DEV_CTL *ctl,int BMCInst);

extern uint32 apml_read_rmi_mailbox_service(
        APML_DEV_CTL    *dev,
        uint8           socketid,
        uint8           dieid,
        uint8           mboxcmd,
        uint32          *mboxdata,
        int             BMCInst);

extern uint32 apml_write_rmi_mailbox_service(
        APML_DEV_CTL    *dev,
        uint8           socketid,
        uint8           dieid,
        uint8           mboxcmd,
        uint32          mboxdata,
        int             BMCInst);

extern uint32 apml_recover_rmi(
        uint8           socketid,
        int             BMCInst);

extern uint32 apml_recover_tsi(
        uint8           socketid,
        int             BMCInst);


/* **************************************************************************/
/*                                                                          */
/*            Mandatory Private User Platform Function Definitions          */
/*                                                                          */
/* **************************************************************************/

uint32 mount_apml_driver(APML_DEV_CTL *dev,int BMCInst);

void unmount_apml_driver();

uint32 user_platform_init(APML_DEV_CTL *dev,int BMCInst);

uint32 user_platform_close(APML_DEV_CTL *ctl);

uint32 user_smbus_write_word(
	APML_DEV_CTL	*dev,
	uint8       socketid,
	uint8		proc_addr,
	uint8		reg,
	uint8		data,
	int			BMCInst);

uint32 user_smbus_read_byte(
	APML_DEV_CTL	*dev,
	uint8       socketid,
	uint8		proc_addr,
	uint32		reg,
	uint32		*data,
	int			BMCInst);

typedef enum {
	EAX = 0,
	EBX,
	ECX,
	EDX
} cpuid_reg;

#define SBRMI		"sbrmi"
#define SBTSI		"sbtsi"
/**
 *  @brief Reads data for the given register.
 *
 *  @details This function will read the data for the given register.
 *
 *  @param[in] soc_num Socket index.
 *
 *  @param[in] reg_offset Register offset.
 *
 *  @param[in] reg_offset Register offset for RMI/TSI I/F.
 *
 *  @param[in] file_name Character device file name for RMI/TSI I/F.
 *
 *  @param[out] buffer output value for the register.
 *
 *  @retval ::0 is returned upon successful call.
 *  @retval Non-zero is returned upon failure.
 *
 */
int apml_driver_read_byte(uint8_t soc_num, uint8_t reg_offset,
				char *file_name, uint8_t *buffer);

/**
 *  @brief Writes data to the specified register.
 *
 *  @details This function will write the data to the specified register.
 *
 *  @param[in] soc_num Socket index.
 *
 *  @param[in] file_name Character device file name for RMI/TSI I/F.
 *
 *  @param[in] reg_offset Register offset for RMI/TSI I/F.
 *
 *  @param[in] value data to write to the register.
 *
 *  @retval ::0 is returned upon successful call.
 *  @retval Non-zero is returned upon failure.
 *
 */
 int apml_driver_write_byte(uint8_t soc_num, uint8_t reg_offset,
				 char *file_name, uint8_t value);

/**
 *  @brief Reads mailbox command data.
 *
 *  @details This function will read mailbox command data.
 *
 *  @param[in] soc_num Socket index.
 *
 *  @param[in] cmd mailbox command.
 *
 *  @param[in] input data.
 *
 *  @param[out] buffer output data for the given mailbox command.
 *
 *  @retval ::0 is returned upon successful call.
 *  @retval Non-zero is returned upon failure.
 *
 */
int apml_driver_read_mailbox(uint8_t soc_num, uint32_t cmd,
				   uint32_t input, uint32_t *buffer);

/**
 *  @brief Writes data to the given  mailbox command.
 *
 *  @details This function will writes data to mailbox command.
 *
 *  @param[in] soc_num Socket index.
 *
 *  @param[in] cmd mailbox command.
 *
 *  @param[in] data input data.
 *
 *  @retval ::0 is returned upon successful call.
 *  @retval Non-zero is returned upon failure.
 *
 */
int apml_driver_write_mailbox(uint8_t soc_num,
				    uint32_t cmd, uint32_t data);
/**
 *  @brief Writes data to device file
 *
 *  @details This function will write data to character device file,
 *  through ioctl.
 *
 *  @param[in] soc_num  Socket index.
 *
 *  @param[in] file_name Character device file name for RMI/TSI I/F
 *
 *  @param[in] msg struct apml_message which contains information about the protocol,
 *  input/output data etc.
 *
 *  @retval ::0 is returned upon successful call.
 *  @retval Non-zero is returned upon failure.
 *
 */
int sbrmi_xfer_msg(uint8_t soc_num, char *file_name,
			    struct apml_message *msg);
/*****************************************************************************/
/** @defgroup ProcessorAccess SB_RMI Read Processor Register Access
 *  Below function provide interface to read the SB-RMI MCA MSR register.
 *  output from MCA MSR commmand will be written into the buffer.
 *  @{
 */

/**
 *  @brief Read the MCA MSR register for a given thread.
 *
 *  @details Given a @p thread and SB-RMI register command, this function reads
 *  msr value.
 *
 *  @param[in] soc_num Socket index.
 *
 *  @param[in] thread is a particular thread in the system.
 *
 *  @param[in] msraddr MCA MSR register to read
 *
 *  @param[out] buffer is to hold the return output of msr value.
 *
 *  @retval ::0 is returned upon successful call.
 *  @retval None-zero is returned upon failure.
 */
int apml_driver_read_msr(uint8_t soc_num,
			       uint32_t thread, uint32_t msraddr,
			       uint64_t *buffer);

/** @} */  // end of ProcessorAccess

/*****************************************************************************/
/** @defgroup cpuidAccess SB-RMI CPUID Register Access
 *  Below function provide interface to get the CPUID access via the SBRMI.
 *
 *  Output from CPUID commmand will be written into registers eax, ebx, ecx
 *  and edx.
 *  @{
 */

/**
 *  @brief Read CPUID functionality for a particular thread in a system.
 *
 *  @details Given a @p thread , @p eax as function input and @p ecx as
 *  extended function input. this function will get the cpuid details
 *  for a particular thread in a pointer to @p eax, @p ebx, @p ecx, @p edx
 *
 *  @param[in] soc_num Socket index.
 *
 *  @param[in] thread is a particular thread in the system.
 *
 *  @param[inout] eax a pointer uint32_t to get eax value
 *
 *  @param[out] ebx a pointer uint32_t to get ebx value
 *
 *  @param[inout] ecx a pointer uint32_t to get ecx value
 *
 *  @param[out] edx a pointer uint32_t to get edx value
 *
 *  @retval ::0 is returned upon successful call.
 *  @retval None-zero is returned upon failure.
 */
int apml_driver_cpuid(uint8_t soc_num,
			    uint32_t thread, uint32_t *eax, uint32_t *ebx,
			    uint32_t *ecx, uint32_t *edx);

/**
 *  @brief Read eax register on CPUID functionality.
 *
 *  @details Given a @p thread, @p fn_eax as function and @p fn_ecx as
 *  extended function input, this function will get the cpuid
 *  details for a particular thread at @p eax.
 *
 *  @param[in] soc_num Socket index.
 *
 *  @param[in] thread is a particular thread in the system.
 *
 *  @param[in] fn_eax cpuid function
 *
 *  @param[in] fn_ecx cpuid extended function
 *
 *  @param[out] eax is to read eax from cpuid functionality.
 *
 *  @retval ::0 is returned upon successful call.
 *  @retval None-zero is returned upon failure.
 */
int apml_driver_cpuid_eax(uint8_t soc_num,
				uint32_t thread, uint32_t fn_eax,
				uint32_t fn_ecx, uint32_t *eax);

/**
 *  @brief Read ebx register on CPUID functionality.
 *
 *  @details Given a @p thread, @p fn_eax as function and @p fn_ecx as
 *  extended function input, this function will get the cpuid
 *  details for a particular thread at @p ebx.
 *
 *  @param[in] soc_num Socket index.
 *
 *  @param[in] thread is a particular thread in the system.
 *
 *  @param[in] fn_eax cpuid function
 *
 *  @param[in] fn_ecx cpuid extended function
 *
 *  @param[out] ebx is to read ebx from cpuid functionality.
 *
 *  @retval ::0 is returned upon successful call.
 *  @retval None-zero is returned upon failure.
 */
int apml_driver_cpuid_ebx(uint8_t soc_num,
				uint32_t thread, uint32_t fn_eax,
				uint32_t fn_ecx, uint32_t *ebx);

/**
 *  @brief Read ecx register on CPUID functionality.
 *
 *  @details Given a @p thread, @p fn_eax as function and @p fn_ecx as
 *  extended function input, this function will get the cpuid
 *  details for a particular thread at @p ecx.
 *
 *  @param[in] soc_num Socket index.
 *
 *  @param[in] thread is a particular thread in the system.
 *
 *  @param[in] fn_eax cpuid function
 *
 *  @param[in] fn_ecx cpuid extended function
 *
 *  @param[out] ecx is to read ecx from cpuid functionality.
 *
 *  @retval ::0 is returned upon successful call.
 *  @retval None-zero is returned upon failure.
 */
int apml_driver_cpuid_ecx(uint8_t soc_num,
				uint32_t thread, uint32_t fn_eax,
				uint32_t fn_ecx, uint32_t *ecx);

/**
 *  @brief Read edx register on CPUID functionality.
 *
 *  @details Given a @p thread, @p fn_eax as function and @p fn_ecx as
 *  extended function input, this function will get the cpuid
 *  details for a particular thread at @p edx.
 *
 *  @param[in] soc_num Socket index.
 *
 *  @param[in] thread is a particular thread in the system.
 *
 *  @param[in] fn_eax cpuid function
 *
 *  @param[in] fn_ecx cpuid extended function
 *
 *  @param[out] edx is to read edx from cpuid functionality.
 *
 *  @retval ::0 is returned upon successful call.
 *  @retval None-zero is returned upon failure.
 */
int apml_driver_cpuid_edx(uint8_t soc_num,
				uint32_t thread, uint32_t fn_eax,
				uint32_t fn_ecx, uint32_t *edx);

/** @} */  // end of cpuidAccess

/*****************************************************************************/
#endif /* !APML_FDK_H */
