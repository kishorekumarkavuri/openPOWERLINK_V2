/**
********************************************************************************
\file   main.c

\brief  main file for Windows kernel module

This file contains the main part of the Windows kernel module implementation for
PCIe device.

\ingroup module_driver_ndispcie
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

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <ndis.h>
#include <ntddk.h>

#include <oplk/oplk.h>
#include <ndis-intf.h>

#include <drvintf.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PLK_MEM_TAG       'klpO'

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief  Instance for POWERLINK driver

The structure specifies the instance variable for the application interface device
*/
typedef struct
{
    NDIS_SPIN_LOCK    syncQueueLock;                ///< synchronization queue lock
    LIST_ENTRY        syncQueueHead;                ///< Pointer to sync queue head entry
    BOOL              fSyncClean;                   ///< Clean pending synchronization IOCTLs
}tPlkDeviceInstance;

/**
\brief  Instance for POWERLINK driver

The structure specifies the instance variable of the Windows kernel driver
*/
typedef struct
{
    PDEVICE_OBJECT        pAppDeviceObject;         ///< IOCTL interface device object
    NDIS_HANDLE           pAppDeviceHandle;         ///< IOCTL interface device handle
    NDIS_HANDLE           driverHandle;             ///< Miniport driver handle
    BOOL                  fInitialized;             ///< Initialization status
    tPlkDeviceInstance*   pDeviceInst;              ///< Pointer to IOCTL device instance
}tPlkDriverInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
tPlkDriverInstance    plkDriverInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
DRIVER_DISPATCH       powerlinkCreate;
DRIVER_DISPATCH       powerlinkCleanup;
DRIVER_DISPATCH       powerlinkClose;
DRIVER_DISPATCH       powerlinkIoctl;

static void registerDrvIntf(NDIS_HANDLE driverHandle_p);
static void deregisterDrvIntf(void);
static void increaseHeartbeatCb(void* unusedParameter1_p, void* functionContext_p,
                                void* unusedParameter2_p, void* unusedParameter3_p);
static void syncCleanUp(void);
static void syncInterruptHandler(void);
//------------------------------------------------------------------------------
//  Kernel module specific data structures
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//---------------------------------------------------------------------------
//  Initialize driver
//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
/**
\brief  Driver initialization routine

The function implements openPOWERLINK Windows kernel driver initialization callback.
OS calls this routine on driver registration.

\param  driverObject_p       Pointer to the system's driver object structure
                             for this driver.
\param  registryPath_p       System's registry path for this driver.

\return This routine returns a NTSTATUS error code.
\retval STATUS_SUCCESS If no error occurs

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS DriverEntry(PDRIVER_OBJECT driverObject_p, PUNICODE_STRING registryPath_p)
{
    NDIS_STATUS    ndisStatus;

    DEBUG_LVL_ALWAYS_TRACE("PLK: + Driver Entry\n");
    ndisStatus = ndis_initDriver(driverObject_p, registryPath_p);

    if (ndisStatus != NDIS_STATUS_SUCCESS)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Failed to initialize driver 0x%X\n", __FUNCTION__,
                              ndisStatus);
        return ndisStatus;
    }

    // register application interface handlers
    ndis_registerDrvIntf(registerDrvIntf, deregisterDrvIntf);
    plkDriverInstance_l.fInitialized = FALSE;

    DEBUG_LVL_ALWAYS_TRACE("PLK: + Driver Entry - OK\n");
    return ndisStatus;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver create function

The function implements openPOWERLINK kernel module create callback. OS calls
this routine when a application tries to open an FILE interface to this driver
using CreateFile().

\param  pDeviceObject_p     Pointer to device object allocated for the IOCTL device.
\param  pIrp_p              Pointer to I/O request packet for this call.

\return This routine returns a NTSTATUS error code.
\retval STATUS_SUCCESS If no error occurs

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkCreate(PDEVICE_OBJECT pDeviceObject_p, PIRP pIrp_p)
{
    NDIS_TIMER_CHARACTERISTICS    timerChars;
    tFileContext*                 pFileContext;
    PIO_STACK_LOCATION            irpStack;
    NDIS_STATUS                   status;

    UNREFERENCED_PARAMETER(pDeviceObject_p);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkCreate ...\n");

    irpStack = IoGetCurrentIrpStackLocation(pIrp_p);

    pFileContext = ExAllocatePoolWithQuotaTag(NonPagedPool, sizeof(tFileContext),
                                              PLK_MEM_TAG);

    if (pFileContext == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("PLK: Failed to create file context\n");
    }

    IoInitializeRemoveLock(&pFileContext->driverAccessLock, PLK_MEM_TAG, 0, 0);

    irpStack->FileObject->FsContext = (void*) pFileContext;

    if (!plkDriverInstance_l.fInitialized)
    {
        drv_initDualProcDrv();
        plkDriverInstance_l.fInitialized = TRUE;
    }

    plkDriverInstance_l.pDeviceInst->fSyncClean = TRUE;

    pIrp_p->IoStatus.Information = 0;
    pIrp_p->IoStatus.Status = STATUS_SUCCESS;
    IoCompleteRequest(pIrp_p, IO_NO_INCREMENT);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkCreate - OK\n");

    return STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver clean-up function

The function implements the clean-up callback. OS calls this when an application
closes the FILE interface to the IOCTL device.

\param  pDeviceObject_p     Pointer to device object allocated for the IOCTL device.
\param  pIrp_p              Pointer to I/O request packet for this call.

\return This routine returns a NTSTATUS error code.
\retval STATUS_SUCCESS If no error occurs

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkCleanup(PDEVICE_OBJECT pDeviceObject_p, PIRP pIrp_p)
{
    syncCleanUp();
    return STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver close function

The function implements openPOWERLINK kernel driver close callback. OS calls
this function when the user application calls CloseHandle() for the device.

\param  pDeviceObject_p     Pointer to device object allocated for the IOCTL device.
\param  pIrp_p              Pointer to I/O request packet for this call.

\return This routine returns a NTSTATUS error code.
\retval Always return STATUS_SUCCESS.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkClose(PDEVICE_OBJECT pDeviceObject_p, PIRP pIrp_p)
{
    tFileContext*         pFileContext;
    PIO_STACK_LOCATION    irpStack;
    UINT16                status;
    tCtrlCmd              ctrlCmd;
    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkClose...\n");

    irpStack = IoGetCurrentIrpStackLocation(pIrp_p);

    pFileContext = irpStack->FileObject->FsContext;
    ExFreePoolWithTag(pFileContext, PLK_MEM_TAG);

    if (plkDriverInstance_l.fInitialized)
    {
        plkDriverInstance_l.fInitialized = FALSE;
        drv_exitDualProcDrv();
    }

    pIrp_p->IoStatus.Information = 0;
    pIrp_p->IoStatus.Status = STATUS_SUCCESS;
    IoCompleteRequest(pIrp_p, IO_NO_INCREMENT);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkClose - OK\n");

    return STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK driver IOCTL handler

The function implements IOCTL callback. OS calls this routine when the user
application calls DeviceIoControl() for the device.

\param  pDeviceObject_p     Pointer to device object allocated for the IOCTL device.
\param  pIrp_p              Pointer to I/O request packet for this call.

\return This routine returns a NTSTATUS error code.
\retval Always return STATUS_SUCCESS.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
NTSTATUS powerlinkIoctl(PDEVICE_OBJECT pDeviceObject_p, PIRP pIrp_p)
{
    PIO_STACK_LOCATION    irpStack;
    NTSTATUS              status = STATUS_SUCCESS;
    ULONG                 inlen, outlen;
    void*                 pInBuffer;
    void*                 pOutBuffer;
    tFileContext*         pFileContext;
    tOplkError            oplRet;

    UNREFERENCED_PARAMETER(pDeviceObject_p);

    irpStack = IoGetCurrentIrpStackLocation(pIrp_p);

    pFileContext = irpStack->FileObject->FsContext;

    // Acquire the IRP remove lock
    status = IoAcquireRemoveLock(&pFileContext->driverAccessLock, pIrp_p);

    if (!NT_SUCCESS(status))
    {
        // Lock is in a removed state. That means we have already received
        // cleaned up request for this handle.
        pIrp_p->IoStatus.Status = status;
        IoCompleteRequest(pIrp_p, IO_NO_INCREMENT);
        return status;
    }

    inlen = irpStack->Parameters.DeviceIoControl.InputBufferLength;
    outlen = irpStack->Parameters.DeviceIoControl.OutputBufferLength;

    switch (irpStack->Parameters.DeviceIoControl.IoControlCode)
    {
        case PLK_CMD_CTRL_EXECUTE_CMD:
        {
            tCtrlCmd*   pCtrlCmd = (tCtrlCmd*) pIrp_p->AssociatedIrp.SystemBuffer;
            drv_executeCmd(pCtrlCmd);
            pIrp_p->IoStatus.Information = sizeof(tCtrlCmd);
            break;
        }
        case PLK_CMD_CTRL_STORE_INITPARAM:
        {
            tCtrlInitParam*   pCtrlInitCmd = (tCtrlInitParam*) pIrp_p->AssociatedIrp.SystemBuffer;
            drv_storeInitParam(pCtrlInitCmd);
            pIrp_p->IoStatus.Information = sizeof(tCtrlInitParam);
            break;
        }
        case PLK_CMD_CTRL_READ_INITPARAM:
        {
            tCtrlInitParam*   pCtrlInitCmd = (tCtrlInitParam*) pIrp_p->AssociatedIrp.SystemBuffer;
            drv_readInitParam(pCtrlInitCmd);
            pIrp_p->IoStatus.Information = sizeof(tCtrlInitParam);
            break;
        }
        case PLK_CMD_CTRL_GET_STATUS:
        {
            UINT16*   pStatus = (UINT16*) pIrp_p->AssociatedIrp.SystemBuffer;
            drv_getStatus(pStatus);
            pIrp_p->IoStatus.Information = sizeof(UINT16);
            break;
        }
        case PLK_CMD_CTRL_GET_HEARTBEAT:
        {
            UINT16*   pHeartBeat = (UINT16*) pIrp_p->AssociatedIrp.SystemBuffer;
            drv_getHeartbeat(pHeartBeat);
            pIrp_p->IoStatus.Information = sizeof(UINT16);
            break;
        }
        case PLK_CMD_POST_EVENT:
        {
            pInBuffer = pIrp_p->AssociatedIrp.SystemBuffer;
            drv_postEvent(pInBuffer);
            break;
        }
        case PLK_CMD_GET_EVENT:
        {
            size_t    eventSize = 0;
            pOutBuffer = pIrp_p->AssociatedIrp.SystemBuffer;
            drv_getEvent(pOutBuffer, &eventSize);

            if (!pIrp_p->Cancel)
                pIrp_p->IoStatus.Information = eventSize;
            else
                pIrp_p->IoStatus.Information = 0;

            break;
        }
        case PLK_CMD_DLLCAL_ASYNCSEND:
        {
            pInBuffer = pIrp_p->AssociatedIrp.SystemBuffer;
            drv_sendAsyncFrame(pInBuffer);
            break;
        }
        case PLK_CMD_ERRHND_WRITE:
        {
            tErrHndIoctl*   pWriteObject = (tErrHndIoctl*) pIrp_p->AssociatedIrp.SystemBuffer;
            drv_writeErrorObject(pWriteObject);
            pIrp_p->IoStatus.Information = 0;
            break;
        }

        case PLK_CMD_ERRHND_READ:
        {
            tErrHndIoctl*   pReadObject = (tErrHndIoctl*) pIrp_p->AssociatedIrp.SystemBuffer;
            drv_readErrorObject(pReadObject);
            pIrp_p->IoStatus.Information = sizeof(tErrHndIoctl);
            break;
        }
        case PLK_CMD_PDO_SYNC:
        {
            if (!pIrp_p->Cancel)
            {
                IoMarkIrpPending(pIrp_p);
                NdisInterlockedInsertTailList(&plkDriverInstance_l.pDeviceInst->syncQueueHead,
                                              &pIrp_p->Tail.Overlay.ListEntry,
                                              &plkDriverInstance_l.pDeviceInst->syncQueueLock);
            }
            else
            {
                pIrp_p->IoStatus.Information = 0;
                status = STATUS_CANCELLED;
                break;
            }
            status = STATUS_PENDING;
            break;
        }
        case PLK_CMD_PDO_GET_MEM:
        {
            tPdoMem*   pPdoMem = (tPdoMem*) pIrp_p->AssociatedIrp.SystemBuffer;
            oplRet = drv_getPdoMem((UINT8**)&pPdoMem->pPdoAddr, pPdoMem->memSize);

            if (oplRet != kErrorOk)
            {
                pIrp_p->IoStatus.Information = 0;
            }
            else
            {
                pIrp_p->IoStatus.Information = sizeof(tPdoMem);
            }
            status = STATUS_SUCCESS;
            break;
        }
        case PLK_CMD_PDO_FREE_MEM:
        {
            tPdoMem*   pPdoMem = (tPdoMem*) pIrp_p->AssociatedIrp.SystemBuffer;
            drv_freePdoMem(pPdoMem->pPdoAddr, pPdoMem->memSize);
            status = STATUS_SUCCESS;
            pIrp_p->IoStatus.Information = 0;
            break;
        }
        case PLK_CMD_CLEAN:
        {
            syncCleanUp();
            break;
        }
        case PLK_GET_BENCHMARK_BASE:
        {
            tBenchmarkMem*   pBenchmarkMem = (tBenchmarkMem*) pIrp_p->AssociatedIrp.SystemBuffer;
            oplRet = drv_getBenchmarkMem((UINT8**)&pBenchmarkMem->pBaseAddr);
            if (oplRet != kErrorOk)
            {
                pIrp_p->IoStatus.Information = 0;
            }
            else
            {
                pIrp_p->IoStatus.Information = sizeof(tBenchmarkMem);
            }
            status = STATUS_SUCCESS;
            break;
        }
        case PLK_FREE_BENCHMARK_BASE:
        {
            tBenchmarkMem*   pBenchmarkMem = (tBenchmarkMem*)pIrp_p->AssociatedIrp.SystemBuffer;
            drv_freeBenchmarkMem(pBenchmarkMem->pBaseAddr);
            status = STATUS_SUCCESS;
            pIrp_p->IoStatus.Information = 0;
            break;
        }
        default:
            DEBUG_LVL_ERROR_TRACE("PLK: - Invalid cmd (cmd=%d)\n",
                                  irpStack->Parameters.DeviceIoControl.IoControlCode);
            break;
    }

    if (status != STATUS_PENDING)
    {
        // complete the Irp if its not pended
        pIrp_p->IoStatus.Status = status;
        IoCompleteRequest(pIrp_p, IO_NO_INCREMENT);
    }

    // Release lock
    IoReleaseRemoveLock(&pFileContext->driverAccessLock, pIrp_p);

    return status;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief Register application interface device

This routine is called from the miniport initialize to register an IOCTL
interface for the driver. A user application can use this interfaces to
communicate with this driver.

A device object to be used for this purpose is created by NDIS when we call
NdisMRegisterDevice.

This routine is called whenever a new miniport instance is initialized.
However, we only create one global device object, when the first miniport
instance is initialized.

/param  driverHandle_p      Miniport driver handle returned by OS on registration

*/
//------------------------------------------------------------------------------
static void registerDrvIntf(NDIS_HANDLE driverHandle_p)
{
    NDIS_STATUS                      status = NDIS_STATUS_SUCCESS;
    UNICODE_STRING                   deviceName;
    UNICODE_STRING                   deviceLinkUnicodeString;
    NDIS_DEVICE_OBJECT_ATTRIBUTES    deviceObjectAttributes;
    PDRIVER_DISPATCH                 dispatchTable[IRP_MJ_MAXIMUM_FUNCTION + 1];

    DEBUG_LVL_ALWAYS_TRACE("PLK %s()...\n", __func__);
    plkDriverInstance_l.driverHandle = driverHandle_p;
    NdisZeroMemory(dispatchTable, (IRP_MJ_MAXIMUM_FUNCTION + 1) * sizeof(PDRIVER_DISPATCH));

    dispatchTable[IRP_MJ_CREATE] = powerlinkCreate;
    dispatchTable[IRP_MJ_CLEANUP] = powerlinkCleanup;
    dispatchTable[IRP_MJ_CLOSE] = powerlinkClose;
    dispatchTable[IRP_MJ_DEVICE_CONTROL] = powerlinkIoctl;

    NdisInitUnicodeString(&deviceName, PLK_DEV_STRING);
    NdisInitUnicodeString(&deviceLinkUnicodeString, PLK_LINK_NAME);

    NdisZeroMemory(&deviceObjectAttributes, sizeof(NDIS_DEVICE_OBJECT_ATTRIBUTES));

    // type implicit from the context
    deviceObjectAttributes.Header.Type = NDIS_OBJECT_TYPE_DEVICE_OBJECT_ATTRIBUTES;
    deviceObjectAttributes.Header.Revision = NDIS_DEVICE_OBJECT_ATTRIBUTES_REVISION_1;
    deviceObjectAttributes.Header.Size = sizeof(NDIS_DEVICE_OBJECT_ATTRIBUTES);
    deviceObjectAttributes.DeviceName = &deviceName;
    deviceObjectAttributes.SymbolicName = &deviceLinkUnicodeString;
    deviceObjectAttributes.MajorFunctions = &dispatchTable[0];
    deviceObjectAttributes.ExtensionSize = sizeof(tPlkDeviceInstance);

    status = NdisRegisterDeviceEx(driverHandle_p,
                                  &deviceObjectAttributes,
                                  &plkDriverInstance_l.pAppDeviceObject,
                                  &plkDriverInstance_l.pAppDeviceHandle);

    plkDriverInstance_l.pAppDeviceObject->Flags |= DO_BUFFERED_IO;
    plkDriverInstance_l.pDeviceInst = (tPlkDeviceInstance*) NdisGetDeviceReservedExtension(plkDriverInstance_l.pAppDeviceObject);
    if (plkDriverInstance_l.pDeviceInst == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Unable to retrieve Device Extension\n", __func__);
        return;
    }

    NdisInitializeListHead(&plkDriverInstance_l.pDeviceInst->syncQueueHead);
    NdisAllocateSpinLock(&plkDriverInstance_l.pDeviceInst->syncQueueLock);
    plkDriverInstance_l.pDeviceInst->fSyncClean = TRUE;

    ndis_registerSyncHandler(syncInterruptHandler);

    DEBUG_LVL_ALWAYS_TRACE("PLK %s() - OK\n", __func__);
}

//------------------------------------------------------------------------------
/**
\brief De-register application interface device

De-register the IOCTL interface registered during initialization,

*/
//------------------------------------------------------------------------------
static void deregisterDrvIntf(void)
{
    ndis_registerSyncHandler(NULL);
    
    // Completed all pending IOCTLs
    syncCleanUp();

    NdisFreeSpinLock(&plkDriverInstance_l.pDeviceInst->syncQueueLock);

    if (plkDriverInstance_l.pAppDeviceHandle != NULL)
    {
        NdisDeregisterDeviceEx(plkDriverInstance_l.pAppDeviceHandle);
    }
}

//------------------------------------------------------------------------------
/**
\brief Synchronization interrupt handler

This is the interrupt routine for synchronization interrupt from PCIe.
On a interrupt it searches for any pending IOCTLs from the user layer and
completes it to indicate presence of new data from PCP.

*/
//------------------------------------------------------------------------------
static void syncInterruptHandler(void)
{
    PLIST_ENTRY           pSyncListEntry = NULL;
    PIRP                  pIrp = NULL;
    NTSTATUS              status;
    PIO_STACK_LOCATION    pIrpStack;

    if (IsListEmpty(&plkDriverInstance_l.pDeviceInst->syncQueueHead) ||
        &plkDriverInstance_l.pDeviceInst->syncQueueHead == NULL)
        return;

    pSyncListEntry = NdisInterlockedRemoveHeadList(&plkDriverInstance_l.pDeviceInst->syncQueueHead,
                                                   &plkDriverInstance_l.pDeviceInst->syncQueueLock);
    pIrp = CONTAINING_RECORD(pSyncListEntry, IRP, Tail.Overlay.ListEntry);

    if (pIrp->Cancel)
    {
        status = STATUS_CANCELLED;
    }
    status = STATUS_SUCCESS;

    pIrp->IoStatus.Status = status;
    pIrp->IoStatus.Information = 0;
    IoCompleteRequest(pIrp, IO_NO_INCREMENT);
    return;
}

//------------------------------------------------------------------------------
/**
\brief Clean up pending IOCTL

This function cleans all the pending IOCTL from user application and completes
them with CANCELLED status.

*/
//------------------------------------------------------------------------------
static void syncCleanUp(void)
{
    PIRP           pIrp;
    PLIST_ENTRY    pListEntry;

    if (&plkDriverInstance_l.pDeviceInst->syncQueueHead != NULL &&
        plkDriverInstance_l.pDeviceInst->fSyncClean)
    {
        while (!IsListEmpty(&plkDriverInstance_l.pDeviceInst->syncQueueHead))
        {
            pListEntry = NdisInterlockedRemoveHeadList(&plkDriverInstance_l.pDeviceInst->syncQueueHead,
                                                       &plkDriverInstance_l.pDeviceInst->syncQueueLock);
            pIrp = CONTAINING_RECORD(pListEntry, IRP, Tail.Overlay.ListEntry);

            if (pIrp != NULL)
            {
                pIrp->IoStatus.Status = STATUS_CANCELLED;
                pIrp->IoStatus.Information = 0;
                IoCompleteRequest(pIrp, IO_NO_INCREMENT);
            }
        }
    }
    plkDriverInstance_l.pDeviceInst->fSyncClean = FALSE;
}

///\}

