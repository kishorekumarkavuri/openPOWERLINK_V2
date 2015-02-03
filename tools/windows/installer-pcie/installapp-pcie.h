/**
********************************************************************************
\file   installapp-pcie.h

\brief  Installation application for Windows driver - Header

This file contains the definitions and prototypes required for installation
application to install a PCIe driver on Windows system.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Kalycito Infotech Private Limited
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

#ifndef _INC_installapp_pcie_H_ 
#define _INC_installapp_pcie_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <cfgmgr32.h>
#include <stdio.h>
#include <tchar.h>
#include <string.h>
#include <newdev.h>
//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define OPLK_INF_NAME              "drv_ndis_pcie.inf"
#define OPLK_DEVICE_ID             "PCI\\VEN_1677&DEV_E53F"

#define UPDATEDRIVERFORPLUGANDPLAYDEVICES "UpdateDriverForPlugAndPlayDevicesA"

#ifndef NDEBUG
#define TRACE(...)    printf(__VA_ARGS__)
#else
#define TRACE(...)
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
//
// UpdateDriverForPlugAndPlayDevices
//
typedef BOOL (WINAPI *UpdateDriverForPlugAndPlayDevicesProto)(__in HWND hwndParent,
                                                              __in LPCTSTR HardwareId,
                                                              __in LPCTSTR FullInfPath,
                                                              __in DWORD InstallFlags,
                                                              __out_opt PBOOL bRebootRequired
                                                         );

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INC_installapp_pcie_H_ */
